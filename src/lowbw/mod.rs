//! This fork's low-bandwidth additions, kept out of upstream files so merges from
//! upstream stay small: ack-based link measurement and stall handling (`link`), the PNG
//! cursor cache (`cursor`), and the flags and shared state the video pipeline's own
//! additions (`video/lowbw`) read. FORK.md lists the hooks upstream files keep.

mod cursor;
mod link;

pub use link::{ClientState, SLOW_LINK_KBPS};

use crate::hub::Hub;
use crate::{range_max, truthy};
use std::collections::HashMap;
use std::sync::atomic::{AtomicBool, AtomicU32, AtomicU64, Ordering};

/// The fork's video flags (`VideoConfig::lb`).
#[derive(Clone)]
pub struct Config {
    /// `--start-kbps`: a new session's first bitrate cap.
    pub start_kbps: u32,
    /// Paint-over delay once acks show the link has headroom (None: off).
    pub fast_paintover_delay_ms: Option<u64>,
    pub ref_frames: Option<u32>,
    pub me_range: u32,
    pub motion_quality: bool,
    pub adaptive_fps: bool,
}

impl Config {
    pub fn parse(a: &HashMap<String, String>) -> Self {
        Config {
            start_kbps: range_max(a.get("start-kbps"), 8000.0) as u32,
            fast_paintover_delay_ms: a
                .get("fast-paintover-delay-ms")
                .map(|v| range_max(Some(v), 0.0) as u64)
                .filter(|&ms| ms > 0),
            ref_frames: a.get("ref-frames").map(|v| range_max(Some(v), 1.0) as u32),
            me_range: range_max(a.get("me-range"), 64.0) as u32,
            motion_quality: truthy(a.get("motion-quality"), true),
            adaptive_fps: truthy(a.get("adaptive-fps"), false),
        }
    }
}

/// The fork's shared state (`Hub::lb`).
pub struct HubState {
    /// True while the delay-based rate controller sees queueing on any client.
    pub congested: AtomicBool,
    /// The rate controller's slow-link verdict, and whether it comes from delivery
    /// samples (`--fast-paintover-delay-ms`), for `/stats`.
    pub slow_link: AtomicBool,
    pub slow_link_measured: AtomicBool,
    /// The last video frame id sent, kept across display rebuilds.
    pub frame_id: AtomicU32,
    /// Maximum outbound clipboard image size in bytes (0 disables outbound image push).
    pub max_clipboard_image_bytes: AtomicU64,
    /// The stripe layout when every stripe's encoder has AQ on, which the sweep needs.
    pub sweep_layout: parking_lot::Mutex<Option<Vec<(u32, u32)>>>,
}

impl Default for HubState {
    fn default() -> Self {
        HubState {
            congested: AtomicBool::new(false),
            slow_link: AtomicBool::new(false),
            slow_link_measured: AtomicBool::new(false),
            frame_id: AtomicU32::new(0),
            max_clipboard_image_bytes: AtomicU64::new(15 << 20),
            sweep_layout: parking_lot::Mutex::new(None),
        }
    }
}

/// Applies the fork's server flags to a new hub (`--max-clipboard-image-kb`).
pub fn init_hub(hub: &Hub, a: &HashMap<String, String>) {
    let max_clip_img_kb = range_max(a.get("max-clipboard-image-kb"), 15360.0) as u64;
    hub.lb
        .max_clipboard_image_bytes
        .store(max_clip_img_kb.saturating_mul(1024), Ordering::Relaxed);
}
