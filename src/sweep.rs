//! The paint-over sweep, for links measured fast (`--fast-paintover-delay-ms`).
//!
//! Once the screen stops moving, the blocks still at motion quality are coded again at
//! one forced QP (`pick_qp`), a run of them per pass: each pass is sized to `PASS_MS` of measured
//! delivery, so it neither queues on the link nor needs repeating. Repeated whole-frame
//! passes under a cap that small never got a dense 1080p frame sharp: each one started
//! from the same capped quality. Blocks not in a pass are marked constant with a QP far
//! above 51, so x264 skips them and they keep what they have.

use crate::capture::Band;
use std::sync::{Arc, OnceLock};
use std::time::{Duration, Instant};

/// A pass's share of the link: this long at the slowest viewer's measured floor.
const PASS_MS: f64 = 50.0;
/// Bytes a block of dense text is guessed to cost at `SEED_QP` before a pass has shown
/// it (measured on 1080p), and the least a block is taken to cost after.
const SEED_BLOCK_BYTES: f64 = 55.0;
const SEED_QP: i32 = 25;
const MIN_BLOCK_BYTES: f64 = 15.0;
/// The seed is from a page at 8 Mbps; a slower link's motion leaves the screen blurrier,
/// and its blocks cost up to 1.4x that. The first pass is sized for this much more, and
/// the first measurement then replaces the guess.
const FIRST_PASS_MARGIN: f64 = 1.5;
/// Of the measured rate, what the passes are paced to, so motion still gets through.
const PACE_SHARE: f64 = 0.85;
/// QP offset given to blocks outside a pass: past 51 whatever adaptive quantization
/// adds, so a block x264 can't skip costs next to nothing.
const SKIP_QP_OFFSET: f32 = 60.0;

/// QP steps around `--paintover-crf` a sweep's QP is picked from. A CRF raises the QP of
/// complex content and a forced QP doesn't: at the CRF itself a dense 1080p page cost
/// 1.7-1.8x an uncapped CRF 20 pass, 3 steps up is the least that stays under 1.5x
/// (1.35-1.45x) and still lands 3 dB above CRF 20. An area that fits one pass costs one
/// pass whatever its QP, so it gets as low as 3 steps down: a typed line then lands within
/// 0.5 dB of an uncapped CRF 20 encode of it, where 3 steps up left it 4-5 dB short.
const QP_SPREAD: f32 = 3.0;

/// The QPs a sweep picks from (`pick_qp`), for `--paintover-crf`.
pub fn qp_range(paintover_crf: f32) -> (i32, i32) {
    let q = |o: f32| (paintover_crf + o).round().clamp(0.0, 51.0) as i32;
    (q(-QP_SPREAD), q(QP_SPREAD))
}

/// Bytes a block is guessed to cost at `qp` before a pass has shown it. Halving the
/// quantizer step (6 QP) about doubles the bits.
fn seed_bytes(qp: i32) -> f64 {
    (SEED_BLOCK_BYTES * 2f64.powf((SEED_QP - qp) as f64 / 6.0)).max(MIN_BLOCK_BYTES)
}

/// The lowest QP of `range` at which `blocks` are guessed to fit `budget` bytes (one
/// pass), else the highest.
fn pick_qp(blocks: usize, budget: f64, (lo, hi): (i32, i32)) -> i32 {
    (lo..hi)
        .find(|&q| blocks as f64 * seed_bytes(q) <= budget)
        .unwrap_or(hi)
}

/// What the encoder made of a pass: per stripe, blocks coded and bytes.
pub type Report = Arc<OnceLock<Vec<(usize, usize)>>>;

/// One pass: the blocks to code (1) and the rest (0), whole screen.
pub struct Pass {
    pub map: Vec<u8>,
    /// Rows the pass's blocks span.
    pub band: Band,
    /// No blocks are left after this one.
    pub last: bool,
    pub job: PassJob,
}

/// What the encoder needs to know of a pass besides its map.
pub struct PassJob {
    pub qp: i32,
    /// Blocks in the pass per stripe.
    pub per_stripe: Vec<usize>,
    pub report: Report,
}

/// x264 setup for one stripe's part of a pass: `mb_info` and `quant_offsets` for its
/// macroblocks. `map` is the stripe's slice of `Pass::map`.
pub fn stripe_hints(map: &[u8]) -> (Vec<u8>, Vec<f32>) {
    let info = map.iter().map(|&a| (a == 0) as u8).collect();
    let offsets = map
        .iter()
        .map(|&a| if a == 0 { SKIP_QP_OFFSET } else { 0.0 })
        .collect();
    (info, offsets)
}

struct StripeState {
    /// First macroblock row and rows.
    r0: usize,
    rows: usize,
    /// Bytes a block costs: a guess until a pass has measured it.
    block_bytes: f64,
    measured: bool,
    /// Blocks in its last pass (None: no pass yet).
    last: Option<usize>,
}

/// A sweep in progress.
pub struct Sweep {
    cols: usize,
    stripes: Vec<StripeState>,
    /// The pass in flight: its report, when it went out, and the rate it was sized for.
    inflight: Option<(Report, Instant, u32)>,
    next_at: Instant,
    min_interval: Duration,
    range: (i32, i32),
    /// Picked on the first pass.
    qp: Option<i32>,
}

impl Sweep {
    /// `layout`: stripes as (y, height) in pixels, `cols`: macroblock columns, `range`:
    /// the QPs to pick from (`qp_range`).
    pub fn new(layout: &[(u32, u32)], cols: usize, range: (i32, i32)) -> Self {
        let stripes = layout
            .iter()
            .map(|&(y, h)| StripeState {
                r0: y as usize / 16,
                rows: (h as usize).div_ceil(16),
                block_bytes: 0.0,
                measured: false,
                last: None,
            })
            .collect();
        Sweep {
            cols,
            stripes,
            inflight: None,
            next_at: Instant::now(),
            min_interval: Duration::ZERO,
            range,
            qp: None,
        }
    }

    /// How long until the next pass may go out (zero: now). None while the last pass
    /// hasn't been encoded yet; the encoder wakes the capture loop when it has.
    pub fn wait(&mut self) -> Option<Duration> {
        if let Some((report, sent, kbps)) = &self.inflight {
            let r = report.get()?;
            let mut bytes = 0;
            for (s, &(blocks, b)) in self.stripes.iter_mut().zip(r) {
                if blocks > 0 {
                    let per = b as f64 / blocks as f64;
                    let mixed = if s.measured {
                        s.block_bytes * 0.5 + per * 0.5
                    } else {
                        per
                    };
                    s.block_bytes = mixed.max(MIN_BLOCK_BYTES);
                    s.measured = true;
                }
                bytes += b;
            }
            let send_ms = bytes as f64 * 8.0 / (*kbps as f64 * PACE_SHARE).max(1.0);
            self.next_at = *sent
                + self
                    .min_interval
                    .max(Duration::from_secs_f64(send_ms / 1000.0));
            self.inflight = None;
        }
        Some(self.next_at.saturating_duration_since(Instant::now()))
    }

    /// The next pass over `pending` (blocks still at motion quality, whole screen),
    /// sized for `kbps` and taken off `pending`; None if nothing is pending. Passes
    /// go out at least `min_interval` apart.
    pub fn plan(
        &mut self,
        pending: &mut [bool],
        kbps: u32,
        min_interval: Duration,
    ) -> Option<Pass> {
        let counts: Vec<usize> = self
            .stripes
            .iter()
            .map(|s| {
                let a = s.r0 * self.cols;
                let b = ((s.r0 + s.rows) * self.cols).min(pending.len());
                pending[a.min(b)..b].iter().filter(|&&p| p).count()
            })
            .collect();
        let total: usize = counts.iter().sum();
        if total == 0 {
            return None;
        }
        let budget = kbps as f64 * 1000.0 / 8.0 * PASS_MS / 1000.0;
        let qp = match self.qp {
            Some(q) => q,
            None => {
                let q = pick_qp(total, budget, self.range);
                for s in self.stripes.iter_mut() {
                    s.block_bytes = seed_bytes(q) * FIRST_PASS_MARGIN;
                }
                *self.qp.insert(q)
            }
        };
        let mut map = vec![0u8; pending.len()];
        let (mut r0, mut r1) = (usize::MAX, 0);
        let mut per_stripe = vec![0; self.stripes.len()];
        for (i, s) in self.stripes.iter_mut().enumerate() {
            if counts[i] == 0 {
                continue;
            }
            let share = budget * counts[i] as f64 / total as f64;
            let mut want = ((share / s.block_bytes) as usize).max(1);
            if let Some(l) = s.last {
                want = want.min(l * 2);
            }
            let mut took = 0;
            'units: for r in interior_rows(s.rows) {
                for c in 0..self.cols {
                    for rr in unit_rows(r, s.rows) {
                        let k = (s.r0 + rr) * self.cols + c;
                        if k < pending.len() && pending[k] {
                            pending[k] = false;
                            map[k] = 1;
                            took += 1;
                            r0 = r0.min(s.r0 + rr);
                            r1 = r1.max(s.r0 + rr + 1);
                        }
                    }
                    if took >= want {
                        break 'units;
                    }
                }
            }
            s.last = Some(took);
            per_stripe[i] = took;
        }
        let report: Report = Arc::new(OnceLock::new());
        self.inflight = Some((report.clone(), Instant::now(), kbps));
        self.min_interval = min_interval;
        Some(Pass {
            map,
            band: Band {
                y0: (r0 * 16) as u32,
                y1: (r1 * 16) as u32,
            },
            last: total == per_stripe.iter().sum::<usize>(),
            job: PassJob {
                qp,
                per_stripe,
                report,
            },
        })
    }
}

/// The rows a stripe's passes walk, top to bottom: all but its first and last, which
/// go with their neighbours (`unit_rows`), so no pass holds an edge row alone.
fn interior_rows(rows: usize) -> std::ops::Range<usize> {
    if rows <= 2 {
        0..1
    } else {
        1..rows - 1
    }
}

/// The rows of one block column taken with row `r`: `r`, plus the edge row next to it.
fn unit_rows(r: usize, rows: usize) -> impl Iterator<Item = usize> {
    let (lo, hi) = if rows <= 2 {
        (0, rows)
    } else {
        (
            if r == 1 { 0 } else { r },
            if r == rows - 2 { rows } else { r + 1 },
        )
    };
    lo..hi
}
