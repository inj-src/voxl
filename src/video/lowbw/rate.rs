//! The rate controller's additions: the slow-link verdict (from delivery samples with
//! `--fast-paintover-delay-ms`, else from the target), adaptive FPS, and flow control's
//! handling of viewers that stopped acking.

use super::super::{RateCtl, VideoConfig};
use crate::hub::{Client, Hub};
use crate::lowbw::SLOW_LINK_KBPS;
use std::sync::atomic::Ordering;
use std::sync::Arc;

/// The fork's rate controller state (`RateCtl::lb`).
pub(in crate::video) struct RateState {
    pub congested: bool,
    /// The target is under `SLOW_LINK_KBPS`. When `measured`: instead, some active viewer
    /// is unmeasured or measured under it, or `--max-kbps` is under it; it leaves only
    /// on a fast delivery sample from every viewer.
    pub slow_link: bool,
    /// The slowest active viewer's lowest fast delivery sample, else its last sample
    /// (kbit/s, 0: unmeasured). Under `SLOW_LINK_KBPS` exactly when that viewer has no
    /// fast evidence, so it decides `slow_link` as its best sample would, while a burst
    /// sized from it can't be stretched by one high outlier.
    pub capacity_kbps: u32,
    /// `--fast-paintover-delay-ms` is set.
    measured: bool,
}

impl Default for RateState {
    fn default() -> Self {
        RateState {
            congested: false,
            slow_link: true,
            capacity_kbps: 0,
            measured: false,
        }
    }
}

/// What the encoder needs from the rate controller, taken at one instant.
#[derive(Clone, Copy)]
pub(in crate::video) struct RateSnapshot {
    pub target_kbps: u32,
    pub congested: bool,
    pub slow_link: bool,
    /// The slowest active viewer's lowest fast delivery sample, else its last sample
    /// (0: unmeasured).
    pub capacity_kbps: u32,
    /// `--fast-paintover-delay-ms` is set: `slow_link` comes from delivery samples.
    pub measured: bool,
}

/// Some viewer is not stalled: stalled ones then neither gate nor slow the others.
pub(in crate::video) fn healthy(clients: &[Arc<Client>]) -> bool {
    clients.iter().any(|c| !c.lb.stalled.load(Ordering::SeqCst))
}

/// A viewer's queueing delay, for `RateCtl::update`; `n`: its frames in flight.
pub(in crate::video) fn queue_ms(c: &Client, n: usize) -> f64 {
    let mut e = c.delay.lock();
    // Queueing delay is only measured on acks. Once everything sent has been acked
    // nothing of ours is queued, so the last burst's delay is stale: keeping it would
    // hold an idle session congested forever.
    if n == 0 {
        e.queue_ms = 0.0;
    }
    e.queue_ms
}

impl RateCtl {
    /// After `RateCtl::new`: `--start-kbps` as the first target (and `--min-kbps` no
    /// higher than `--max-kbps`).
    pub(in crate::video) fn lowbw(mut self, cfg: &VideoConfig) -> Self {
        self.min = self.min.min(self.max);
        self.target = (cfg.lb.start_kbps as f64).clamp(self.min, self.max);
        self.lb.measured = cfg.lb.fast_paintover_delay_ms.is_some();
        self
    }

    pub(in crate::video) fn snapshot(&self) -> RateSnapshot {
        RateSnapshot {
            target_kbps: self.target as u32,
            congested: self.lb.congested,
            slow_link: self.lb.slow_link,
            capacity_kbps: self.lb.capacity_kbps,
            measured: self.lb.measured,
        }
    }

    /// Recomputes `slow_link` from the viewers now watching, so a join, leave or restart
    /// counts at once rather than on the next rate tick.
    pub(in crate::video) fn refresh_link(&mut self, clients: &[Arc<Client>]) {
        let healthy = healthy(clients);
        // A viewer that joined is unmeasured (0) until its keyframe is acked. A stalled
        // one counts once it is sent frames again: its restart keyframe goes out before
        // it acks it, and what follows queues behind that.
        self.lb.capacity_kbps = clients
            .iter()
            .filter(|c| {
                !(healthy
                    && c.lb.stalled.load(Ordering::SeqCst)
                    && !c.started.load(Ordering::SeqCst))
            })
            .map(|c| c.link_floor_kbps())
            .min()
            .unwrap_or(0);
        let slow = SLOW_LINK_KBPS as f64;
        self.lb.slow_link = if self.lb.measured {
            self.max < slow || self.lb.capacity_kbps < SLOW_LINK_KBPS
        } else {
            self.target < slow
        };
    }

    /// End of `update`, before the target is published.
    pub(in crate::video) fn lb_update(
        &mut self,
        hub: &Hub,
        clients: &[Arc<Client>],
        congested: bool,
    ) {
        self.lb.congested = congested;
        self.refresh_link(clients);
        hub.lb.congested.store(congested, Ordering::Relaxed);
    }

    /// Capture FPS ceiling adjusted for current bandwidth target and congestion.
    pub(in crate::video) fn effective_fps(&self, base_fps: u32) -> u32 {
        let mut fps = base_fps.max(1);
        if self.target < 350.0 {
            fps = fps.min(8);
        } else if self.target < 600.0 {
            fps = fps.min(12);
        } else if self.target < 1000.0 {
            fps = fps.min(15);
        }
        if self.lb.congested {
            fps = (fps / 2).max(6).min(base_fps.max(1));
        }
        fps
    }
}

/// Flow control's view of stalled viewers (`flow_ok`).
pub(in crate::video) struct Stall {
    /// A viewer whose oldest frame in flight is older than this has stalled. On slow
    /// uplinks (<1 Mbps) a legitimate keyframe or dense delta can take >4 s to cross the
    /// link; stalling it at 4 s would cause an endless keyframe loop.
    ms: u128,
    /// See `healthy`.
    healthy: bool,
}

impl Stall {
    /// Also returns the viewers it looked at, for `flow_ok` to go through.
    pub(in crate::video) fn new(hub: &Hub) -> (Self, Vec<Arc<Client>>) {
        let target_kbps = hub.target_kbps.load(Ordering::Relaxed);
        let ms = if target_kbps > 0 && target_kbps < 1000 {
            12_000
        } else {
            4_000
        };
        let clients = hub.video_clients();
        // Stalled viewers are ignored while a healthy one is watching; a lone stalled
        // viewer still gets flow control, or nothing would pace its stream.
        let healthy = healthy(&clients);
        (Stall { ms, healthy }, clients)
    }

    /// Leaves `c` out of flow control (`n` frames in flight, the oldest `age` ms old):
    /// it is draining, it has just stalled, or it restarted and has not acked its
    /// keyframe yet while a healthy viewer is watching.
    pub(in crate::video) fn skip(&self, c: &Client, n: usize, age: u128) -> bool {
        if c.lb.draining.load(Ordering::SeqCst) {
            if age > self.ms * 2 {
                // It never acked what it was sent (a lost ack or a decoder reset): give up
                // waiting and restart it with a keyframe.
                log::warn!(
                    target: "voxl::video",
                    "client {} did not catch up in {} ms; restarting",
                    c.id,
                    age
                );
                c.inflight.lock().clear();
                c.lb.draining.store(false, Ordering::SeqCst);
                c.needs_keyframe.store(true, Ordering::Relaxed);
            }
            return true;
        }
        if age > self.ms {
            // Stalled client (hidden tab, dead socket): stop letting it gate everyone,
            // and stop sending to it until it has read what it was already sent.
            log::warn!(
                target: "voxl::video",
                "client {} stalled ({} frames, {} ms); pausing its video",
                c.id,
                n,
                age
            );
            c.mark_stalled();
            return true;
        }
        self.healthy && c.lb.stalled.load(Ordering::SeqCst)
    }
}
