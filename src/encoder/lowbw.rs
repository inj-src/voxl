//! Fork additions to the x264 wrapper: tuning flags, per-macroblock QP offsets and
//! forced-QP frames for the paint-over sweep. See FORK.md for the hooks in encoder.rs.

use super::{ffi, Encoded, Encoder, Planes};
use std::ffi::{c_char, c_int, c_void, CStr};
use std::ptr;
use std::sync::atomic::{AtomicU32, Ordering};

/// `--ref-frames`, clamped to 1-16 (0: keep the preset's).
static REF_FRAMES: AtomicU32 = AtomicU32::new(0);
/// `--me-range`, before clamping.
static ME_RANGE: AtomicU32 = AtomicU32::new(64);

/// Sets `--ref-frames` (3-4 cut modal/tab re-encoding; None keeps the preset's) and
/// `--me-range` (motion search range in pixels; 96-128 helps low-fps scrolls) for every
/// encoder opened after this.
pub fn set_tuning(ref_frames: Option<u32>, me_range: u32) {
    REF_FRAMES.store(ref_frames.map_or(0, |r| r.clamp(1, 16)), Ordering::Relaxed);
    ME_RANGE.store(me_range, Ordering::Relaxed);
}

/// Applies `set_tuning` and the log filter to the parameters `Encoder::new` opens with.
pub(super) fn tune(param: &mut ffi::x264_param_t) {
    match REF_FRAMES.load(Ordering::Relaxed) {
        0 => {}
        r => param.i_frame_reference = r as i32,
    }
    param.analyse.i_me_range = ME_RANGE.load(Ordering::Relaxed).clamp(16, 256) as i32;
    if let Some(log) = param.pf_log {
        param.p_log_private = log as *mut c_void;
        param.pf_log = Some(filter_log);
    }
}

/// Per-picture hints for `Encoder::encode_hinted`.
#[derive(Default)]
pub struct Hints<'a> {
    /// Forces the frame QP.
    pub qp: Option<i32>,
    /// One byte per macroblock, X264_MBINFO_CONSTANT for unchanged ones: lets x264 skip
    /// analysis of static areas.
    pub mb_info: Option<&'a mut [u8]>,
    /// QP offsets per macroblock, added to adaptive quantization's (none with AQ off).
    pub quant_offsets: Option<&'a mut [f32]>,
}

/// Forced-QP bookkeeping (`Encoder::lb`).
#[derive(Default)]
pub(super) struct QpState {
    /// Forced-QP frames are being encoded (`begin_forced_qp`).
    forced_qp: bool,
    /// `end_forced_qp` ran and no frame has been encoded since. x264 applies only the
    /// last reconfig before a frame, so until then every reconfig turns the CRF max off.
    clear_rf_max: bool,
}

/// The message x264 logs when a reconfig sets the CRF max to the CRF, which
/// `end_forced_qp` does on purpose: that is what turns the max off again.
const CRF_MAX_WARNING: &CStr = c"CRF max must be greater than CRF\n";

/// x264's logger, minus `CRF_MAX_WARNING`. `p` is x264's own logger, passed on with the
/// message untouched (`A` is the platform's va_list).
unsafe extern "C" fn filter_log<A>(p: *mut c_void, level: c_int, fmt: *const c_char, args: A) {
    if !fmt.is_null() && CStr::from_ptr(fmt) == CRF_MAX_WARNING {
        return;
    }
    let log: unsafe extern "C" fn(*mut c_void, c_int, *const c_char, A) =
        std::mem::transmute_copy(&p);
    log(ptr::null_mut(), level, fmt, args);
}

impl Encoder {
    /// Adaptive quantization is on, so `Hints::quant_offsets` take effect.
    pub fn aq_enabled(&self) -> bool {
        unsafe {
            let mut p: ffi::x264_param_t = std::mem::zeroed();
            ffi::x264_encoder_parameters(self.h, &mut p);
            p.rc.i_aq_mode != 0 && p.rc.f_aq_strength > 0.0
        }
    }

    /// Before frames with a forced QP (`Hints::qp`): a CRF of 0.01 with a max of 0.02
    /// caps the QP VBV may raise rows to under the forced QP, so the frame keeps it.
    /// The VBV buffer is left as it is and accounts for the frame.
    pub fn begin_forced_qp(&mut self) {
        if self.lb.forced_qp {
            return;
        }
        self.lb.forced_qp = true;
        self.lb.clear_rf_max = false;
        unsafe {
            self.param.rc.f_rf_constant = 0.01;
            self.param.rc.f_rf_constant_max = 0.02;
            ffi::x264_encoder_reconfig(self.h, &mut self.param);
        }
    }

    /// Back to CRF after forced-QP frames, at `crf`. A CRF max equal to the CRF turns
    /// the max off (0 would keep the last one). x264 builds each reconfig from the
    /// parameters it last applied, and applies only the last one before a frame, so
    /// `set_rate_buf` repeats this until the next frame is encoded.
    pub fn end_forced_qp(&mut self, crf: f32) {
        if !self.lb.forced_qp {
            return;
        }
        self.lb.forced_qp = false;
        self.lb.clear_rf_max = true;
        self.param.rc.f_rf_constant = crf;
        self.clear_rf_max(crf);
        unsafe {
            ffi::x264_encoder_reconfig(self.h, &mut self.param);
        }
    }

    /// After `set_rate_buf` set the CRF: while `clear_rf_max`, the max equal to it. x264
    /// takes a max of 0 as "keep the last one", so a CRF of 0 is raised to 0.01 for it.
    pub(super) fn clear_rf_max(&mut self, crf: f32) {
        if self.lb.clear_rf_max {
            let c = crf.max(0.01);
            self.param.rc.f_rf_constant = c;
            self.param.rc.f_rf_constant_max = c;
        }
    }

    /// `encode` with `hints`, which are consumed before this returns.
    pub fn encode_hinted(
        &mut self,
        p: &Planes,
        ts_ms: i64,
        force_idr: bool,
        hints: Hints,
        out: &mut Vec<u8>,
    ) -> Result<Encoded, String> {
        self.pic.prop.quant_offsets = hints
            .quant_offsets
            .map_or(ptr::null_mut(), |q| q.as_mut_ptr());
        self.pic.prop.quant_offsets_free = None;
        let r = self.encode(p, ts_ms, force_idr, hints.qp, hints.mb_info, out);
        self.pic.prop.quant_offsets = ptr::null_mut();
        if self.lb.clear_rf_max {
            // The max is off now; later reconfigs leave it off.
            self.lb.clear_rf_max = false;
            self.param.rc.f_rf_constant_max = 0.0;
        }
        r
    }
}
