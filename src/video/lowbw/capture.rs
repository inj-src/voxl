//! The capture loop's additions: adaptive FPS pacing, paint-over timing for links
//! measured fast (`--fast-paintover-delay-ms`), and driving the paint-over sweep.

use super::super::{RateCtl, Refiner, VideoConfig, REFINE_EVERY, REFINE_STILL_MS};
use super::sweep::{self, Next, PassJob, Sweep};
use super::JobState;
use crate::capture::Band;
use crate::hub::{Client, Hub};
use std::sync::atomic::Ordering;
use std::sync::Arc;
use std::time::{Duration, Instant};

/// How long a block stays still before it is sharpened, and how often a refinement
/// pass may run, on a link measured fast: half of `REFINE_STILL_MS` and `REFINE_EVERY`.
/// Slow links wait twice those.
const FAST_REFINE_STILL_MS: u32 = 150;
const FAST_REFINE_EVERY: Duration = Duration::from_millis(120);

/// The fork's capture-loop state (`lb` in `capture_loop`).
pub(in crate::video) struct CaptureState {
    fps: u32,
    adaptive_fps: bool,
    fast_paintover_delay_ms: Option<u64>,
    base_paintover_delay_ms: u64,
    paintover_burst: u32,
    min_kbps: u32,
    motion_quality: bool,
    /// See `super::note_layout`; None: no sweep.
    sweep_layout: Option<Vec<(u32, u32)>>,
    sweep_qps: (i32, i32),
    /// The capture rate adaptive FPS settled on (`pace`).
    eff_fps: u32,
    /// The last frame sent was a paint-over pass.
    last_paintover: bool,
    /// The sweep in progress.
    pub sweep: Option<Sweep>,
    // Set by `begin` for the current iteration:
    /// `--fast-paintover-delay-ms` while the link is measured fast.
    fast_delay: Option<u64>,
    /// The paint-over delay that applies now.
    pub paintover_delay_ms: u64,
    /// A paint-over may be a sweep.
    sweeping: bool,
    /// What the sweep waits for.
    next: Option<Next>,
    /// The sweep holds back a due paint-over this long (ms).
    sweep_wait: Option<i32>,
    /// This capture's sweep pass and whether it is the last.
    sweep_pass: Option<(PassJob, bool)>,
    /// This capture is the first pass of a fast paint-over burst.
    paintover_first: bool,
}

impl CaptureState {
    pub(in crate::video) fn new(cfg: &VideoConfig, hub: &Hub) -> Self {
        let measured = cfg.lb.fast_paintover_delay_ms.is_some();
        hub.lb.slow_link_measured.store(measured, Ordering::Relaxed);
        CaptureState {
            fps: cfg.fps,
            adaptive_fps: cfg.lb.adaptive_fps,
            fast_paintover_delay_ms: cfg.lb.fast_paintover_delay_ms,
            base_paintover_delay_ms: cfg.paintover_delay_ms,
            paintover_burst: cfg.paintover_burst,
            min_kbps: cfg.min_kbps,
            motion_quality: cfg.lb.motion_quality,
            sweep_layout: hub.lb.sweep_layout.lock().clone(),
            sweep_qps: sweep::qp_range(cfg.paintover_crf),
            eff_fps: cfg.fps,
            last_paintover: false,
            sweep: None,
            fast_delay: None,
            paintover_delay_ms: cfg.paintover_delay_ms,
            sweeping: false,
            next: None,
            sweep_wait: None,
            sweep_pass: None,
            paintover_first: false,
        }
    }

    /// After `rc.update`: publishes the slow-link verdict and, with `--adaptive-fps`,
    /// returns the capture interval and its ms for the FPS the rate allows instead of
    /// `upstream` (`min_interval`, `interval_ms`).
    pub(in crate::video) fn pace(
        &mut self,
        hub: &Hub,
        rc: &RateCtl,
        upstream: (Duration, f64),
    ) -> (Duration, f64) {
        hub.lb.slow_link.store(rc.lb.slow_link, Ordering::Relaxed);
        if !self.adaptive_fps {
            self.eff_fps = self.fps;
            return upstream;
        }
        self.eff_fps = rc.effective_fps(self.fps);
        let interval = Duration::from_secs_f64(1.0 / self.eff_fps.max(1) as f64);
        (interval.mul_f64(0.85), interval.as_secs_f64() * 1000.0)
    }

    /// Starts an iteration, once `want_key` is known.
    pub(in crate::video) fn begin(
        &mut self,
        rc: &RateCtl,
        clients: &[Arc<Client>],
        want_key: bool,
    ) {
        // With --fast-paintover-delay-ms a link measured fast sharpens sooner; paint-overs
        // never run while congested.
        self.fast_delay = self.fast_paintover_delay_ms.filter(|_| !rc.lb.slow_link);
        self.paintover_delay_ms = self.fast_delay.unwrap_or(self.base_paintover_delay_ms);
        // On a link measured fast, with AQ, a paint-over is a sweep. A keyframe, a viewer
        // that isn't measured fast and congestion end it (motion does too). So does a
        // stalled viewer, until its restart keyframe is acked: the link floor leaves it out
        // until that keyframe is sent, and passes queued meanwhile would follow it.
        let stalled = clients.iter().any(|c| c.lb.stalled.load(Ordering::SeqCst));
        self.sweeping = self.fast_delay.is_some()
            && !rc.lb.congested
            && !want_key
            && !stalled
            && self.sweep_layout.is_some();
        self.sweep_wait = None;
        self.sweep_pass = None;
        self.paintover_first = false;
    }

    /// A pass's blocks stop being pending once it is coded, before this capture's
    /// changes are marked. True when a keyframe replaced a pass: every block is pending
    /// again, and the burst starts over.
    pub(in crate::video) fn sweep_restarts(&mut self, refiner: &mut Refiner) -> bool {
        self.next = self.sweep.as_mut().map(|s| s.wait(&mut refiner.pending));
        let restart = matches!(self.next, Some(Next::Restart));
        if restart {
            refiner.mark_all();
            self.sweep = None;
        }
        if !self.sweeping || self.sweep.is_none() {
            self.sweep = None;
            self.next = None;
        }
        restart
    }

    /// A sweep's next pass waits for the last one's size (the encoder wakes us), then
    /// for the link to carry it: a due paint-over is held back until then.
    pub(in crate::video) fn sweep_due(&mut self, paintover_due: bool) -> bool {
        if !paintover_due {
            return false;
        }
        match &self.next {
            Some(Next::Report) => self.sweep_wait = Some(100),
            Some(Next::After(d)) if !d.is_zero() => {
                self.sweep_wait = Some(d.as_millis() as i32 + 1)
            }
            _ => {}
        }
        self.sweep_wait.is_none()
    }

    /// How long the idle capture loop waits (ms).
    pub(in crate::video) fn idle_timeout(&self, paintover_left: u32, last_motion: Instant) -> i32 {
        let wait = self
            .paintover_delay_ms
            .saturating_sub(last_motion.elapsed().as_millis() as u64);
        // A paint-over that is due but held back by congestion re-checks at the rate
        // controller's pace instead of spinning.
        if let Some(t) = self.sweep_wait {
            t
        } else if paintover_left == 0 || wait == 0 {
            100
        } else {
            wait as i32 + 1
        }
    }

    /// Motion right after a paint-over pass goes out at once: the pass was extra.
    pub(in crate::video) fn skip_interval(&self, damaged: bool) -> bool {
        self.fast_delay.is_some() && self.last_paintover && damaged
    }

    /// The blocks a paint-over sharpens and their rows: a sweep takes the next run of
    /// them, sized for the link; else all that are pending. None: nothing is pending.
    pub(in crate::video) fn paint_map(
        &mut self,
        refiner: &Refiner,
        rc: &RateCtl,
        min_interval: Duration,
    ) -> Option<(Vec<u8>, Band)> {
        let next = match (self.sweeping, &self.sweep_layout) {
            (true, Some(layout)) => self
                .sweep
                .get_or_insert_with(|| Sweep::new(layout, refiner.cols, self.sweep_qps))
                .plan(&refiner.pending, rc.lb.capacity_kbps, min_interval)
                .map(|p| {
                    self.sweep_pass = Some((p.job, p.last));
                    (p.map, p.band)
                }),
            _ => refiner.pending_map(),
        };
        if next.is_none() {
            self.sweep = None;
        }
        next
    }

    /// Sharpening to piggyback on a motion frame, unless the link is congested.
    pub(in crate::video) fn refine(
        &self,
        refiner: &mut Refiner,
        rc: &RateCtl,
    ) -> Option<(Vec<u8>, Band)> {
        if rc.lb.congested || self.paintover_burst == 0 {
            return None;
        }
        // Slow links wait twice as long so refinement doesn't compete with motion.
        refiner.due(if rc.lb.slow_link {
            (REFINE_STILL_MS * 2, REFINE_EVERY * 2)
        } else if self.fast_delay.is_some() {
            (FAST_REFINE_STILL_MS, FAST_REFINE_EVERY)
        } else {
            (REFINE_STILL_MS, REFINE_EVERY)
        })
    }

    /// A paint-over pass goes out. A sweep runs until nothing is pending, which its next
    /// pass finds out once this one is coded, so its passes leave the burst count alone.
    pub(in crate::video) fn paintover_pass(&mut self, paintover_left: &mut u32, want_key: bool) {
        if self.sweep_pass.is_none() {
            self.paintover_first = self.fast_delay.is_some()
                && *paintover_left == self.paintover_burst.max(1)
                && !want_key;
            *paintover_left -= 1;
        }
    }

    /// This capture is a sweep's last pass.
    pub(in crate::video) fn last_pass(&self) -> bool {
        self.sweep_pass.as_ref().is_some_and(|p| p.1)
    }

    /// The fork's part of this capture's job.
    pub(in crate::video) fn job(&mut self, frame_paintover: bool, rc: &RateCtl) -> JobState {
        self.last_paintover = frame_paintover;
        JobState {
            paintover_first: frame_paintover && self.paintover_first,
            sweep: self.sweep_pass.take().map(|(j, _)| j),
            rate_scale: self.fps.max(1) as f64 / self.eff_fps.max(1) as f64,
            rate: rc.snapshot(),
            min_kbps: self.min_kbps,
            motion_quality: self.motion_quality,
            fps: self.fps.max(1),
        }
    }
}
