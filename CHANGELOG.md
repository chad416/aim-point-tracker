# Changelog

The format follows [Keep a Changelog](https://keepachangelog.com/en/1.1.0/). Versions follow the milestones of [SPEC.md](SPEC.md) §11 (0.1.0 = M1 ... 0.4.0 = M4). The CAN frames have their own version in `protocol/tracker.dbc` (`tracker 0.1`); no frame layout has changed since it was introduced.

## [0.5.0] - 2026-11-08

### Added
- SocketCAN frames carry the kernel's reception time (`SO_TIMESTAMPNS`, moved to `CLOCK_MONOTONIC`) instead of the time the loop read them (SPEC §6.7).
- Outlier gating in `timesync`: a SYNC more than 200 µs from the fit is held out; four in a row that agree with each other restart the fit (a real step). Unit tests `test_late_sync_rejected`, `test_step_accepted_after_n`.
- IT11: `aim_host` live on a vcan interface (skipped without one); CI creates `vcan0` where the runner's kernel has the module.
- TRK_STATUS fault bit 6, CAN_LOSS: since the last status a frame could not be sent, or a SHOT_EVENT found the pending-shot table full.
- `aim_host` summary: `frames_bad_size`, `sync` (received, rejected, resets), `match.shots_lost`, `match.untracked_shots`, `can_tx_dropped`, `can_user_stamps`, `can_rx_delay_max_us`, `hits.invalid`; `wall_ms` per frame in the frame log.
- `aim_host --rt`: `mlockall` and `SCHED_FIFO`, best effort.
- `requirements.txt` (used by CI), `LICENSE` (MIT), this changelog.
- IT10 recorded again with weapon firmware v0.6.2, which stamps SHOT_EVENT with the trigger sample at the break: 8 of 8 OK within 0.18 mm, 9.5–10.4 ms; pulses 480–525 µs, starts −49 to +140 µs. `tests/data/s1_v0.6.2_lane.txt` replaces the v0.6.1 and v0.6.0 recordings.

### Changed
- A shot of a weapon with neither a track nor a candidate takes only a spot that is new and unexplained; two or more such spots give AMBIGUOUS, none NO_SPOT. Before, it took the strongest spot of at least `min_init_snr`, so a stray could become an OK hit (SPEC §6.6). Unit test `test_untracked_shot`.
- A track ignores (and counts) an update that is not later than its last one, instead of restarting and losing its velocity.
- Calibration: a fit is reported converged when no step lowers the cost only if the gradient vanishes there (MINPACK's orthogonality test); the residual check refuses NaN; `AT_CAL_FIND_ALL` names the default of `min_found` (all grid points).
- Detection: each blob records its last run, so its centroid window and its peak search read only its own runs (they scanned every run of the frame per blob).
- IT09 counts a hit as correct within 5 mm (P01's 99th-percentile limit) instead of 20 mm, and reports CPU and wall time per frame. IT04 does not fail on the timing-only SLOW fault bit.
- The integration tests write their calibration to a temporary directory, so a run no longer rewrites `results/calib.json`. Import paths are set in `pytest.ini`.
- Scripts and tools take their paths from the environment (`AIMTRK_WS`, `AIMTRK_VENV`, `AIMTRK_SRC`, `S1_ELF`, `S1_SIM`, `S1_SRC`, `COSIM_OUT`) instead of one machine's paths.
- Documentation: SPEC trimmed (results by milestone, revision log as a table); stale figures corrected (weapon pulse starts −58 to +170 µs with v0.6.1, not within the ±100 µs it should hold; centroid bias up to 0.034 px; saturation up to 24 × full scale).

### Fixed
- `canio_open_socket` closed nothing on its error paths (the socket leaked); `canio_open_files` leaked its input file if the output could not be opened.
- A frame whose size differs from the first one is skipped and counted; it used to be read with the first frame's stride.
- A SHOT_EVENT refused because the pending-shot table was full was silently lost; it is now counted, logged and flagged. A refused SocketCAN send is counted and flagged.
- A HIT result outside the known codes was counted as OK in the summary.
- The SocketCAN reader stopped at the first extended, RTR or error frame instead of skipping it.

### Removed
- `tests/data/s1_v0.6.0_lane.txt`, the IT10 recording of firmware v0.6.0 (no longer replayed).

## [0.4.1] - 2026-11-08

### Changed
- IT10 recorded again with weapon firmware v0.6.1: pulses 480–526 µs, none past its exposure; IT10 also checks pulse width and overrun.

## [0.4.0] - 2026-11-08 (M4)

### Added
- IT09, 100 000 frames with the interference set; IT10, end to end with the weapon firmware in Renode (recorded and replayed).
- Lane view and demo session (`tools/lane/`), GIF and video; traceability; CI.

### Changed
- Detection: spots inside a scene change are recovered, a spot must stand out from its surroundings, blobs too small to be a spot take no table place, blob table 1024.
- Tracks start only on a confirmed strong spot and let go of a wrong one after three slot frames.

## [0.3.0] - 2026-11-08 (M3)

### Added
- `timesync`, `track`, `match`; `aim_host` with the file transport and SocketCAN; HIT, AIM and TRK_STATUS frames; scenarios and the region renderer; IT03–IT07.

### Changed
- Break-to-shot-pulse margin 2 ms (was 0.5 ms); the background copes with scene changes.

## [0.2.0] - 2026-11-08 (M2)

### Added
- `geom`, `calib` and `zero`; OpenCV cross-check; IT01, IT02, IT08.

## [0.1.0] - 2026-11-07 (M1)

### Added
- Camera simulator; `detect` and `centroid` with the NumPy reference; centroid accuracy curves.
