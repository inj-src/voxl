//! This fork's low-bandwidth additions, kept out of upstream files so merges from
//! upstream stay small: ack-based link measurement and stall handling (`link`), the PNG
//! cursor cache (`cursor`), and the shared state the video pipeline reads.

mod cursor;
mod link;

pub use link::{ClientState, SLOW_LINK_KBPS};

use crate::hub::Hub;
use crate::range_max;
use std::collections::HashMap;
use std::sync::atomic::{AtomicBool, AtomicU32, AtomicU64, Ordering};

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
}

impl Default for HubState {
    fn default() -> Self {
        HubState {
            congested: AtomicBool::new(false),
            slow_link: AtomicBool::new(false),
            slow_link_measured: AtomicBool::new(false),
            frame_id: AtomicU32::new(0),
            max_clipboard_image_bytes: AtomicU64::new(15 << 20),
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
