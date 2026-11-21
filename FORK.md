# This fork

Low-bandwidth changes on top of upstream `tembo/voxl` (remote `origin`). Most fork
code lives in files upstream doesn't have:

- `src/lowbw/`: flags (`Config`), shared state (`HubState`), per-viewer link
  measurement and stalls (`link.rs`), the PNG cursor cache (`cursor.rs`).
- `src/video/lowbw/`: rate controller additions (`rate.rs`), paint-over timing and
  sweep driving (`capture.rs`), per-frame encoder settings (`encode.rs`), the sweep
  (`sweep.rs`).
- `src/encoder/lowbw.rs`: `--ref-frames`/`--me-range`, forced-QP frames, QP offsets.

- `vcpkg/ports/x264/`: a vcpkg overlay port for the Windows build, vcpkg's own x264
  port plus upstream's `patches/x264` (read from there by relative path, not copied).
  Install with `vcpkg install x264 ... --overlay-ports=<repo>/vcpkg/ports`. The port
  pins x264 `b35605ac`, which the patches were checked against. If you move the ref,
  check them again. vcpkg's binary cache hashes only the port directory, so after
  `patches/x264` changes, bump `port-version` in `vcpkg.json` or a cached build is reused.
  `vcpkg/.gitattributes` and the `patches/x264` line in `.gitattributes` keep the
  patches byte-exact on Windows checkouts (`core.autocrlf`).

Fork state hangs off one field per upstream struct, always named `lb`.

## Hooks left in upstream files

- `lib.rs`: `mod lowbw`; `serve` sets `VideoConfig::lb` (`lowbw::Config::parse`) and
  calls `lowbw::init_hub` after `Hub::new`.
- `hub.rs`: `Client::lb` and `Hub::lb`, set in `add_client` and `Hub::new`. `Hub::cursor`
  holds `(id, message)`. `on_ack` takes the viewer's hold and arrival time, calls
  `sample_ack` under the inflight lock and `resync_ack` after it. `restart_clients`
  calls `restart_video`.
- `encoder.rs`: `Encoder::lb`; `new` calls `lowbw::tune` after `x264_param_apply_profile`;
  `set_rate_buf` calls `clear_rf_max`.
- `video.rs`:
  - `lb` fields on `VideoConfig`, `Job`, `RateCtl`, `StripeJob`.
  - `spawn`: `lowbw::spawn`. `build`: `lowbw::note_layout`.
  - `flow_ok`: `Stall::new`, `stall.skip`.
  - `RateCtl::update`: `refresh_link`, `healthy`, `queue_ms`, `reset_link`, `lb_update`.
  - `Refiner::due` takes its thresholds from `CaptureState::refine`.
  - `capture_loop`: `RateCtl::new(..).lowbw()`, then `CaptureState` (`pace`, `begin`,
    `sweep_restarts`, `sweep_due`, `idle_timeout`, `skip_interval`, `paint_map`,
    `refine`, `paintover_pass`, `last_pass`, `job`), and draining viewers left out of
    `want_key`.
  - `stripe_worker`: `WorkerState::set_qp`, `key_rate`, `quant_offsets`,
    `Encoder::encode_hinted`, `--motion-quality`, no governor reopen during a sweep.
  - `encode_loop`: `hub.lb.frame_id`, `sweep_stripes`, `report_skipped`, `EncodePlan`
    (`crf`, `share`, `cap`, `stripe_job`, `admit`), `PassReport`.
- `.gitattributes`: `patches/x264/*.patch -text` (see `vcpkg/ports/x264/` above).
- `server.rs`: `/stats` link fields, `send_current_cursor`, ack hold/arrival parsing,
  `restart_video` on "v". `record.rs`: `on_ack(id, 0.0, None)`.
- `desktop.rs`, `mac/desktop.rs`, `win/desktop.rs`: `hub.lb.max_clipboard_image_bytes`,
  `publish_cursor`.

Fork features still written inline: capture downscaling on Windows (`convert.rs`,
`win/capture.rs`, `win/input.rs`, `win/mod.rs`), Opus DTX and noise gate (`audio.rs`,
`win/audio.rs`), flag docs (`help.rs`, `README.md`), and the web client's cursor
cache and ack timing (`web/src/`).

## Syncing with upstream

    git fetch origin && git merge origin/main

Merge, don't rebase: published SHAs on this branch are pinned elsewhere. Conflicts
should only touch the hooks above; keep upstream's new code and put the hook back.
Then run `cargo fmt --check`, `cargo clippy --all-targets` and `cargo test`.
