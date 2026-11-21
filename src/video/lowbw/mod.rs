//! This fork's additions to the video pipeline, kept out of video.rs so merges from
//! upstream stay small (FORK.md lists the hooks):
//! - `rate`: the rate controller's slow-link verdict from delivery samples, adaptive
//!   FPS, and pausing viewers that stopped acking;
//! - `capture`: paint-over timing for fast links and driving the sweep;
//! - `encode`: per-frame encoder settings (caps, keyframes for slow links, forced QP);
//! - `sweep`: the paint-over sweep itself.

mod capture;
mod encode;
mod rate;
mod sweep;

pub(super) use capture::CaptureState;
pub(super) use encode::{
    key_rate, quant_offsets, report_skipped, sweep_stripes, EncodePlan, PassReport, WorkerState,
};
pub(super) use rate::{healthy, queue_ms, RateSnapshot, RateState, Stall};

use super::{Stripe, VideoConfig};
use crate::hub::Hub;
use std::sync::atomic::Ordering;
use sweep::PassJob;

/// The fork's part of a capture job (`Job::lb`).
pub(super) struct JobState {
    /// First pass of a fast paint-over burst (`--fast-paintover-delay-ms`).
    paintover_first: bool,
    /// A sweep pass (`mb_dirty` holds its blocks).
    sweep: Option<PassJob>,
    /// `--framerate` over the capture rate adaptive FPS settled on. x264 budgets each
    /// frame as 1/`--framerate` of the cap, so the cap is scaled up by this to keep the
    /// bitrate at the target when fewer frames are captured.
    rate_scale: f64,
    /// The rate controller's state at capture, read as one with `rate_scale`.
    rate: RateSnapshot,
    /// `--min-kbps`, `--motion-quality` and `--framerate` (at least 1).
    min_kbps: u32,
    motion_quality: bool,
    fps: u32,
}

/// The fork's part of a stripe's job (`StripeJob::lb`).
pub(super) struct StripeJobState {
    /// The rate controller sees a slow link (`RateSnapshot::slow_link`).
    slow_link: bool,
    /// A sweep pass at this QP: `mb_dirty` marks its blocks, the rest are skipped.
    pub sweep_qp: Option<i32>,
    /// `--motion-quality`: motion frames get MotionQuality's CRF.
    pub motion_quality: bool,
}

/// `spawn`, before the first pipeline is built.
pub(super) fn spawn(cfg: &VideoConfig, hub: &Hub) {
    // Before any viewer joins: it decides whether viewers keep delivery samples.
    hub.lb
        .slow_link_measured
        .store(cfg.lb.fast_paintover_delay_ms.is_some(), Ordering::Relaxed);
    crate::encoder::set_tuning(cfg.lb.ref_frames, cfg.lb.me_range);
}

/// `build`, once the stripes' encoders are open: the sweep needs AQ on every one.
pub(super) fn note_layout(hub: &Hub, stripes: &[Stripe], layout: &[(u32, u32)]) {
    *hub.lb.sweep_layout.lock() = stripes
        .iter()
        .all(|s| s.enc.aq_enabled())
        .then(|| layout.to_vec());
}
