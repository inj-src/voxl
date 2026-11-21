//! Per-viewer link state: delivery samples from acks (`Client::link_kbps`), and the
//! stall that pauses video to a viewer that stopped acking until it catches up.

use crate::hub::{Client, Hub, Inflight};
use parking_lot::Mutex;
use std::collections::VecDeque;
use std::sync::atomic::{AtomicBool, AtomicU32, Ordering};
use std::time::{Duration, Instant};

/// A link counts as slow until its acks show it delivers at least this (kbit/s).
pub const SLOW_LINK_KBPS: u32 = 2000;

/// A frame this big is timed on its own: 50 ms at `SLOW_LINK_KBPS`, well above RTT
/// jitter. Smaller frames are also added up until they reach it.
const LINK_SAMPLE_MIN_BYTES: usize = 12_500;
/// Shortest delivery time a sample is credited with, and the closest two frames may
/// arrive and still count: a retransmit releases several at once.
const LINK_SAMPLE_MIN_MS: f64 = 10.0;
/// An added-up sample of small frames carries each frame's RTT jitter, so it reads low:
/// it revokes fast only under this, and at or above it keeps existing fast evidence fresh.
const LINK_POOL_SLOW_KBPS: u32 = 1000;
/// Fast evidence counts this long; without fresh evidence the viewer is unmeasured.
const LINK_WINDOW: Duration = Duration::from_secs(2);
/// Most fast samples kept: a fast link acks many frames within `LINK_WINDOW`.
const LINK_FAST_MAX: usize = 64;

/// The fork's per-viewer state (`Client::lb`).
pub struct ClientState {
    /// Cursor image IDs already sent to this client (capped FIFO for cache hits).
    pub known_cursors: Mutex<VecDeque<u32>>,
    /// Delivery samples from acks; see `link_kbps`.
    pub(crate) link: Mutex<LinkEst>,
    /// Went too long without acking (hidden tab, dead or saturated socket): left out of
    /// flow control and rate control until it acks the keyframe that restarts it.
    pub stalled: AtomicBool,
    /// Stalled and still reading what was queued before the stall: it gets no video
    /// until it acks `resync_id`, the last frame it was sent.
    pub draining: AtomicBool,
    /// While draining, the last frame sent before the stall; after it, the restart keyframe.
    pub resync_id: AtomicU32,
    /// Keeps delivery samples (`--fast-paintover-delay-ms`); without them `link` stays empty.
    measure_link: bool,
}

impl ClientState {
    pub fn new(hub: &Hub) -> Self {
        ClientState {
            known_cursors: Mutex::new(VecDeque::new()),
            link: Mutex::new(LinkEst::default()),
            stalled: AtomicBool::new(false),
            draining: AtomicBool::new(false),
            resync_id: AtomicU32::new(0),
            measure_link: hub.lb.slow_link_measured.load(Ordering::Relaxed),
        }
    }
}

/// Delivery samples for `Client::link_kbps`.
#[derive(Default)]
pub(crate) struct LinkEst {
    /// When the last acked frame reached the viewer (ack time minus its hold).
    last_arrival: Option<Instant>,
    /// The same arrival on the viewer's clock (ms), if the viewer reports it.
    last_arrival_v: Option<f64>,
    /// The viewer reports arrival times. Without them a frame that queued is timed from
    /// a server stamp that return-path delay can make late, so it never counts as fast.
    viewer_clock: bool,
    /// The run of frames that queued one behind the other, from the last that didn't:
    /// bytes and delivery ms. A frame that queued is timed with the whole run.
    burst: Option<(usize, f64)>,
    /// Frames too small to time alone, added up: bytes, delivery ms, first added.
    pool_bytes: usize,
    pool_ms: f64,
    pool_at: Option<Instant>,
    /// Samples at or above `SLOW_LINK_KBPS` (kbit/s, when), of the last `LINK_WINDOW`.
    fast: VecDeque<(u32, Instant)>,
    /// The sample that last revoked fast (kbit/s); 0 if none since a reset.
    slow_kbps: u32,
}

impl LinkEst {
    /// `whole`: one frame of `LINK_SAMPLE_MIN_BYTES` or more; else small frames added up.
    fn record(&mut self, kbps: u32, whole: bool, now: Instant) {
        self.fast
            .retain(|&(_, t)| now.saturating_duration_since(t) <= LINK_WINDOW);
        if kbps >= SLOW_LINK_KBPS {
            if self.viewer_clock {
                self.fast.push_back((kbps, now));
                if self.fast.len() > LINK_FAST_MAX {
                    self.fast.pop_front();
                }
            }
        } else if !whole && kbps >= LINK_POOL_SLOW_KBPS {
            // Typing over a jittery relay: not evidence on its own, but no sign of a
            // slow link either. Only the newest sample stays fresh, so old ones can't
            // hold the floor down or up for long.
            if let Some(e) = self.fast.back_mut() {
                e.1 = now;
            }
        } else {
            self.fast.clear();
            self.slow_kbps = kbps;
        }
    }

    fn live_fast(&self) -> impl Iterator<Item = u32> + '_ {
        let now = Instant::now();
        self.fast
            .iter()
            .filter(move |&&(_, t)| now.saturating_duration_since(t) <= LINK_WINDOW)
            .map(|&(k, _)| k)
    }

    fn clear_pool(&mut self) {
        self.pool_bytes = 0;
        self.pool_ms = 0.0;
        self.pool_at = None;
    }
}

/// `a <= b` in wrapping u32 frame-id order (as in hub.rs).
fn wrapping_le(a: u32, b: u32) -> bool {
    b.wrapping_sub(a) < 0x8000_0000
}

impl Client {
    /// `on_ack`, under the inflight lock before the acked frames are popped: takes a
    /// delivery sample of them. `viewer`: how long the viewer held the frame after its
    /// last part arrived (decode and present, 0 from a viewer that does not say), and
    /// when that last part arrived on the viewer's clock (ms).
    pub(crate) fn sample_ack(&self, q: &VecDeque<Inflight>, id: u32, viewer: (f64, Option<f64>)) {
        // A frame read while draining may have waited many seconds: no delivery sample.
        // Read under the lock `mark_stalled` takes, as the sample is taken, so no sample
        // lands after its reset.
        if self.lb.draining.load(Ordering::SeqCst) || !self.lb.measure_link {
            return;
        }
        let mut bytes = 0;
        let mut first_sent = None;
        let mut last_sent = None;
        for f in q.iter().take_while(|f| wrapping_le(f.id, id)) {
            bytes += f.bytes;
            first_sent.get_or_insert(f.sent);
            if f.id == id {
                last_sent = Some(f.sent);
            }
        }
        if let (Some(first), Some(last)) = (first_sent, last_sent) {
            self.sample_link([first, last], bytes, viewer);
        }
    }

    /// `on_ack`, after the inflight lock: ends a stall once the viewer has read what was
    /// queued before it (then it gets a keyframe), and once it acks that keyframe.
    pub(crate) fn resync_ack(&self, id: u32) {
        if self.lb.stalled.load(Ordering::SeqCst)
            && wrapping_le(self.lb.resync_id.load(Ordering::SeqCst), id)
        {
            if self.lb.draining.load(Ordering::SeqCst) {
                // It has read everything queued before the stall: restart it.
                log::info!(target: "voxl::hub", "client {} caught up; sending a keyframe", self.id);
                self.needs_keyframe.store(true, Ordering::Relaxed);
                self.lb.draining.store(false, Ordering::SeqCst);
            } else if self.started.load(Ordering::SeqCst) {
                log::info!(target: "voxl::hub", "client {} acked its restart keyframe", self.id);
                self.lb.stalled.store(false, Ordering::SeqCst);
            }
        }
    }

    /// Delivery rate of the frames just acked (`sent`: the first and the acked one):
    /// their bytes over the time they took to arrive, arrival - send - min RTT. Arrival
    /// is the ack time minus the viewer's hold (`viewer`: hold, arrival on its clock).
    /// Frames that queued behind the previous ones are timed from the previous arrival,
    /// on the viewer's clock, which a delayed or bunched ack can't shorten, together with
    /// the whole run they queued in: one that finished late (a retransmit) releases the
    /// rest soon after, and each of those alone would read fast.
    fn sample_link(&self, sent: [Instant; 2], bytes: usize, viewer: (f64, Option<f64>)) {
        let (hold_ms, arrived_v) = viewer;
        let arrived_v = arrived_v.filter(|v| v.is_finite());
        let now = Instant::now();
        // A hold as long as the whole round trip is wrong; it says nothing.
        let interval_ms = now.saturating_duration_since(sent[1]).as_secs_f64() * 1000.0;
        if !(hold_ms >= 0.0 && hold_ms < interval_ms) {
            return;
        }
        let arrived = now - Duration::from_secs_f64(hold_ms / 1000.0);
        let mut l = self.lb.link.lock();
        let prev = l.last_arrival.replace(arrived);
        let prev_v = std::mem::replace(&mut l.last_arrival_v, arrived_v);
        l.viewer_clock = arrived_v.is_some();
        let gap_v = arrived_v.zip(prev_v).map(|(a, p)| a - p);
        let gap =
            gap_v.or(prev.map(|p| arrived.saturating_duration_since(p).as_secs_f64() * 1000.0));
        // Released together (a retransmit): too close to time, so it joins the run.
        if let Some(g) = gap.filter(|&g| g < LINK_SAMPLE_MIN_MS) {
            if let Some(b) = l.burst.as_mut() {
                b.0 += bytes;
                b.1 += g.max(0.0);
            }
            return;
        }
        // The smaller of the ping RTT and the fastest ack, so a high ping can't inflate it.
        let ping = match self.min_rtt_ms.load(Ordering::Relaxed) {
            0 => f64::MAX,
            r => r as f64,
        };
        let rtt_ms = ping.min(self.delay.lock().base_ms);
        let rtt_ms = if rtt_ms.is_finite() && rtt_ms < 10_000.0 {
            rtt_ms
        } else {
            0.0
        };
        let start = sent[0] + Duration::from_secs_f64(rtt_ms / 1000.0);
        let since_start = arrived.saturating_duration_since(start).as_secs_f64() * 1000.0;
        let queued = prev.filter(|&p| p > start);
        let ms = match (queued, gap_v) {
            // The viewer's gap: not shortened by return-path delay, nor by an RTT
            // baseline that grew (which moves `start` later).
            (Some(_), Some(g)) => g,
            (Some(p), None) => (arrived - p).as_secs_f64() * 1000.0,
            // Arrived within the RTT baseline: too small to time, and no bytes are credited.
            (None, _) if arrived <= start => return,
            (None, _) => since_start,
        };
        let kbps = |b: usize, ms: f64| (b as f64 * 8.0 / ms.max(LINK_SAMPLE_MIN_MS)) as u32;
        // Frames released one after another each read fast alone: a frame that queued
        // adds to the run it queued behind, and one that didn't starts a new run.
        let run = match (queued, l.burst) {
            (Some(_), Some((b, t))) => (b + bytes, t + ms),
            _ => (bytes, ms),
        };
        l.burst = Some(run);
        if bytes >= LINK_SAMPLE_MIN_BYTES {
            l.record(kbps(run.0, run.1), true, now);
            return;
        }
        if l.pool_at
            .is_some_and(|t| now.saturating_duration_since(t) > LINK_WINDOW)
        {
            l.clear_pool();
        }
        l.pool_at.get_or_insert(now);
        l.pool_bytes += bytes;
        l.pool_ms += ms;
        if l.pool_bytes < LINK_SAMPLE_MIN_BYTES || l.pool_ms < LINK_SAMPLE_MIN_MS {
            return;
        }
        let pooled = kbps(l.pool_bytes, l.pool_ms);
        l.clear_pool();
        l.record(pooled, false, now);
    }

    /// Delivery rate (kbit/s): the best fast sample of the last `LINK_WINDOW`, else the
    /// sample that revoked fast, else 0 (unmeasured). Under `SLOW_LINK_KBPS` the viewer
    /// counts as a slow link.
    pub fn link_kbps(&self) -> u32 {
        let l = self.lb.link.lock();
        l.live_fast().max().unwrap_or(l.slow_kbps)
    }

    /// Like `link_kbps` but the lowest fast sample: what a burst may be sized from, so
    /// one high outlier can't stretch it.
    pub fn link_floor_kbps(&self) -> u32 {
        let l = self.lb.link.lock();
        l.live_fast().min().unwrap_or(l.slow_kbps)
    }

    /// Forgets the delivery rate: the viewer counts as a slow link until measured again.
    pub fn reset_link(&self) {
        let mut l = self.lb.link.lock();
        l.clear_pool();
        l.fast.clear();
        l.slow_kbps = 0;
        l.last_arrival = None;
        l.last_arrival_v = None;
        l.burst = None;
    }

    /// Stops video to a client that stopped acking, so it neither gates the healthy
    /// viewers nor piles more frames behind the ones it has not read yet.
    /// Only the last frame stays in flight, so `backlog` times how long draining takes.
    pub fn mark_stalled(&self) {
        let mut q = self.inflight.lock();
        let last = q.pop_back();
        q.clear();
        if let Some(f) = last {
            self.lb.resync_id.store(f.id, Ordering::SeqCst);
            q.push_back(f);
        }
        self.lb.stalled.store(true, Ordering::SeqCst);
        self.lb.draining.store(true, Ordering::SeqCst);
        self.started.store(false, Ordering::SeqCst);
        self.reset_link();
    }

    /// Starts video to this client over (a new stream, or video turned back on): it waits
    /// for a keyframe, and what was measured of its link no longer counts. Under the
    /// inflight lock, like `mark_stalled`, so no sample from an older ack lands after.
    pub fn restart_video(&self) {
        let mut q = self.inflight.lock();
        q.clear();
        self.lb.stalled.store(false, Ordering::SeqCst);
        self.lb.draining.store(false, Ordering::SeqCst);
        self.started.store(false, Ordering::Relaxed);
        self.needs_keyframe.store(true, Ordering::Relaxed);
        self.reset_link();
    }
}
