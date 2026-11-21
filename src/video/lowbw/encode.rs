//! The encode side's additions: per-frame caps for slow and fast links, keyframes
//! sized for viewers not yet measured fast, forced-QP sweep passes, and the reports
//! that pace the sweep.

use super::super::{Job, StripeJob, HDR};
use super::sweep::{Outcome, Report};
use super::StripeJobState;
use crate::encoder::Encoder;
use crate::hub::{Client, Hub, Inflight};
use crate::lowbw::SLOW_LINK_KBPS;
use std::collections::VecDeque;
use std::sync::atomic::Ordering;
use std::time::Instant;

pub(in crate::video) use super::sweep::quant_offsets;

/// The rate a keyframe for a viewer not yet measured fast is sized from, whatever
/// `--start-kbps` is, so its first picture crosses a slow link well within the stall limit.
const UNMEASURED_KEY_KBPS: u32 = 1500;

/// A stripe worker's state (`lb` in `stripe_worker`).
#[derive(Default)]
pub(in crate::video) struct WorkerState {
    /// The encoder was just reopened: its next frame is an IDR, which can't be a sweep
    /// pass (the blocks outside the pass would be coded at QP 51).
    pub fresh: bool,
}

impl WorkerState {
    /// Puts the encoder in or out of forced-QP mode for `job`; returns the sweep pass's
    /// QP, if it is one. `crf`: the CRF the job is coded at otherwise.
    pub(in crate::video) fn set_qp(
        &mut self,
        enc: &mut Encoder,
        job: &StripeJob,
        crf: f32,
    ) -> Option<i32> {
        let was_fresh = std::mem::take(&mut self.fresh);
        let sweep_qp = job.lb.sweep_qp.filter(|_| !was_fresh);
        if sweep_qp.is_some() {
            enc.begin_forced_qp();
        } else {
            enc.end_forced_qp(crf);
        }
        sweep_qp
    }
}

/// A keyframe's VBV cap and buffer (kbit/s, kbit): `upstream`'s, except on a slow link,
/// where the single IDR stays within ~1.25 s of the cap so it does not trip the stall
/// resync loop.
pub(in crate::video) fn key_rate(job: &StripeJob, upstream: (u32, u32)) -> (u32, u32) {
    if !job.lb.slow_link {
        return upstream;
    }
    let boost = job.cap.saturating_mul(2).max(400);
    let buf_kbit = (job.cap * 5 / 4).max(120);
    (boost, buf_kbit)
}

/// A sweep pass codes the stripes it has blocks in, instead of `selected`.
pub(in crate::video) fn sweep_stripes(job: &Job, selected: Vec<bool>) -> Vec<bool> {
    match &job.lb.sweep {
        Some(p) => p.per_stripe.iter().map(|&b| b > 0).collect(),
        None => selected,
    }
}

/// A sweep pass with no stripe to code: reports it as coded at once.
pub(in crate::video) fn report_skipped(job: &Job, hub: &Hub, n: usize) {
    if let Some(p) = &job.lb.sweep {
        let _ = p.report.set(Outcome {
            bytes: vec![None; n],
            keyframe: false,
            sent: Instant::now(),
        });
        hub.wake();
    }
}

/// A sweep pass's report, filed once the pass is queued for sending.
pub(in crate::video) struct PassReport(Option<(Report, Outcome)>);

impl PassReport {
    pub(in crate::video) fn new(job: &Job, results: &[Option<(bool, Vec<u8>)>]) -> Self {
        PassReport(job.lb.sweep.as_ref().map(|p| {
            let outcome = Outcome {
                bytes: results
                    .iter()
                    .map(|r| r.as_ref().map(|r| r.1.len() - HDR))
                    .collect(),
                keyframe: results.iter().flatten().any(|r| r.0),
                sent: Instant::now(),
            };
            (p.report.clone(), outcome)
        }))
    }

    /// Files the report, if this was a pass; `sent`: when it went out, if it did.
    pub(in crate::video) fn file(&mut self, hub: &Hub, sent: Option<Instant>) {
        if let Some((r, mut o)) = self.0.take() {
            if let Some(t) = sent {
                o.sent = t;
            }
            let _ = r.set(o);
            hub.wake();
        }
    }
}

/// The encode loop's per-frame decisions (`lb` in `encode_loop`).
pub(in crate::video) struct EncodePlan {
    /// The bitrate cap to share between stripes (kbit/s).
    pub target: u32,
    pub slow_link: bool,
    congested: bool,
    capacity_kbps: u32,
    measured: bool,
    /// Pinned near `--min-kbps` on a congested slow link: motion gets a higher CRF.
    near_floor: bool,
    /// Every paint-over and refine pass fits ~50 ms of measured delivery (kbit/s per
    /// stripe; their VBV buffers hold 1.25 frames of cap each).
    budget: u32,
    /// With measured links, a keyframe's viewers: those not started (joining or
    /// restarting) as of now. One that joins during the encode waits for the next.
    key_for: Option<Vec<u64>>,
}

impl EncodePlan {
    /// `live`: the target as read at encode, and the stripe count.
    pub(in crate::video) fn new(job: &Job, hub: &Hub, live: (u32, usize)) -> Self {
        let (live_target, n) = live;
        let r = job.lb.rate;
        // Measured links use the capture-time snapshot (it matches `rate_scale`); without
        // them the rate is read live, at encode.
        let (target, congested, slow_link) = if r.measured {
            (r.target_kbps, r.congested, r.slow_link)
        } else {
            let t = live_target;
            (
                t,
                hub.lb.congested.load(Ordering::Relaxed),
                t < SLOW_LINK_KBPS,
            )
        };
        let key_for: Option<Vec<u64>> = (job.force_idr && r.measured).then(|| {
            hub.video_clients()
                .iter()
                .filter(|c| {
                    !c.lb.draining.load(Ordering::SeqCst) && !c.started.load(Ordering::SeqCst)
                })
                .map(|c| c.id)
                .collect()
        });
        // If any of them is not measured fast, the keyframe takes the slow path, sized
        // from at most `UNMEASURED_KEY_KBPS`.
        let key_slow = key_for.as_ref().is_some_and(|ids| {
            hub.video_clients()
                .iter()
                .any(|c| ids.contains(&c.id) && c.link_kbps() < SLOW_LINK_KBPS)
        });
        let near_floor = slow_link && congested && target <= job.lb.min_kbps.saturating_mul(3) / 2;
        let (slow_link, target) = if key_slow {
            (true, target.min(UNMEASURED_KEY_KBPS))
        } else {
            (slow_link, target)
        };
        EncodePlan {
            target,
            slow_link,
            congested,
            capacity_kbps: r.capacity_kbps,
            measured: r.measured,
            near_floor,
            budget: (r.capacity_kbps as u64 * job.lb.fps as u64 / (25 * n as u64)) as u32,
            key_for,
        }
    }

    /// The CRF of a frame that is not a paint-over.
    pub(in crate::video) fn crf(&self, crf: f32) -> f32 {
        if self.near_floor {
            // Smoothly raise QP when pinned near the floor instead of letting VBV slam QP 51.
            (crf + 4.0).min(38.0)
        } else {
            crf
        }
    }

    /// A single stripe on a slow link gets the exact target instead of `share`, since
    /// one burst at 2x already queues.
    pub(in crate::video) fn share(&self, share: u32, n: usize) -> u32 {
        if self.slow_link && n == 1 {
            self.target.max(1)
        } else {
            share
        }
    }

    /// A stripe's VBV cap, from upstream's `cap`.
    pub(in crate::video) fn cap(&self, job: &Job, cap: u32) -> u32 {
        // Keyframes keep the unscaled cap: their size is bounded on its own.
        let cap = if job.force_idr {
            cap
        } else {
            (cap as f64 * job.lb.rate_scale).round() as u32
        };
        // On a link measured fast a pass that can't reach the paint-over CRF in its
        // `budget` leaves the rest to the next. The first fast pass may take up to 4x share
        // where acks show twice the target, so it can reach the paint-over CRF in one frame.
        let fast = self.measured && !self.slow_link;
        if fast && job.paintover && !job.force_idr {
            let first =
                job.lb.paintover_first && !self.congested && self.capacity_kbps / 2 >= self.target;
            if first { cap.saturating_mul(2) } else { cap }.min(self.budget)
        } else {
            cap
        }
    }

    pub(in crate::video) fn stripe_job(&self, job: &Job) -> StripeJobState {
        StripeJobState {
            slow_link: self.slow_link,
            sweep_qp: job.lb.sweep.as_ref().map(|p| p.qp),
            motion_quality: job.lb.motion_quality,
        }
    }

    /// Whether frame `frame_id` goes to `c`, under its inflight lock (`q`): not while it
    /// drains (checked under the lock `mark_stalled` takes, so nothing is queued after
    /// it), and one not started only on a keyframe meant for it, which starts it.
    pub(in crate::video) fn admit(
        &self,
        c: &Client,
        q: &mut VecDeque<Inflight>,
        frame: (bool, u32),
    ) -> bool {
        let (all_key, frame_id) = frame;
        if c.lb.draining.load(Ordering::SeqCst) {
            return false;
        }
        if !c.started.load(Ordering::SeqCst) {
            // With measured links only a forced keyframe starts a viewer, the one sized
            // for it: not an IDR from an encoder that reopened with new threads.
            let for_it = match &self.key_for {
                Some(ids) => ids.contains(&c.id),
                None => !self.measured,
            };
            if !all_key || !for_it {
                return false;
            }
            if c.lb.stalled.load(Ordering::SeqCst) {
                // The restart keyframe: acking it ends the stall.
                c.lb.resync_id.store(frame_id, Ordering::SeqCst);
            }
            c.started.store(true, Ordering::SeqCst);
            q.clear();
        }
        true
    }
}
