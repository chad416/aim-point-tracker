# Traceability: requirements to tests

Every requirement of [SPEC.md](../SPEC.md) §3, with the tests that check it and the measured result. Unit tests are ztest suites on `native_sim` (`tests/unit`): 58 of 58 pass, 97.9 % line coverage. IT01–IT10 are pytest integration tests (`tests/integration`). IT01, IT02 and IT08 drive the C library through ctypes; IT03–IT07, IT09 and IT10 run `aim_host` itself, on simulated lane runs or, in IT10, on S1's firmware recorded in Renode. Numbers come from the files in `results/` (week 4 runs, 8 Nov 2026).

## Functional

| ID | Requirement (short) | Tests | Status | Measured / note |
|---|---|---|---|---|
| F01 | Frames from a frame source (simulator through a pipe, or a file) | IT03–IT07, IT09 (frames piped into `aim_host`); `aimtrk_c` tests on files | Met | The header carries the camera's frame counter and the exposure start on the tracker's clock. `aim_host` turns that into lane time (F06), so it is not written in lane time by the camera (SPEC §8). |
| F02 | Spot detection: background, threshold, runs and blobs, rejection rules | unit `detect` (13); C vs NumPy cross-check (40 frames, 0 mismatches); IT06, IT09 | Met | Week 3: the background snaps to scene changes, the threshold uses the 3 × 3 cell maximum. Week 4: spots inside scene changes are recovered, a spot must stand out from its surroundings, small blobs take no table place (SPEC §6.1). |
| F03 | Sub-pixel centroid; area, peak, SNR, saturation flag | unit `centroid` (9); `centroid_curve.py`; cross-check | Met | 0.018 px RMS at the nominal spot; rim fit for saturated spots within 0.009 px up to 24 × full scale |
| F04 | Undistort, homography to screen mm; off-screen reported, not dropped | unit `geom` (6); OpenCV cross-check; IT07 (OFF_SCREEN with position) | Met | Distortion = OpenCV to 1e-16, undistortion to 1e-12; exact homography to 5e-13 px |
| F05 | Calibration from a 9 × 6 grid, `calib.json` with version and CRC, refuse a bad fit | unit `calib` (6); IT01 (4 tests); OpenCV `calibrateCamera` cross-check | Met | Fit 0.023 px, held-out 0.023 px. A displaced grid and a partly hidden grid are refused. `aim_host` checks `geometry_crc32`. |
| F06 | Lane time from SYNC, offset and drift | unit `timesync` (3); IT03 (frame times); IT09; IT10 | Met | Frame exposure starts within 36 µs of the truth with the tracker clock drifting 40 ppm (IT03), 44 µs over 1000 s (IT09). In IT10 the same SYNC frames reach S1's firmware and the tracker. |
| F07 | Aim strobing in slots; one track per weapon | unit `match`, `track` (2); IT04, IT05, IT10 | Met | 486 of 488 aim pulses gave an AIM, 0.11 mm RMS from the truth (IT04; a new track's first spot is a candidate only, SPEC §6.6). With S1's firmware: a pulse in every slot frame of its weapon, none in the other's (IT10). |
| F08 | SHOT_EVENT → shot frame on lane time → HIT | unit `match` (`test_shot_frame_rule`, `test_aim_and_hit`, `test_late_shot`); IT03; IT10 | Met | 100 of 100 shots from the right frame (IT03). Same rule as S1 v0.6 (`strobe_shot_frame`), with a 2 ms margin. IT10: S1's own 8 SHOT_EVENTs gave 8 OK HITs within 0.19 mm. |
| F09 | Several spots per frame: slot, gate, prediction; AMBIGUOUS instead of a guess | unit `match` (`test_two_weapons_same_frame`, `test_ambiguous`, `test_stray_and_gate`, `test_min_init_snr`, `test_reacquire`, `test_unconfirmed_stray`); IT05, IT09 | Met | IT05: 160 of 160 attributed; 60 of 60 at nearly the same point reported AMBIGUOUS, never a wrong one |
| F10 | HIT for every SHOT_EVENT, with a reason when there is no position | unit `match` (`test_no_hit_reasons`), `proto` (4); IT07 | Met | OK, OFF_SCREEN, NO_SPOT (out of view, covered), DROPPED; SATURATED in the unit test (a failed rim fit) |
| F11 | AIM per slot frame; TRK_STATUS at 1 Hz | unit `proto`; IT04, IT07 | Met | TRK_STATUS at 1 Hz: 100 fps, worst frame, ambient, calibration age, FRAMES_DROPPED bit (IT07) |
| F12 | After-action record: trace −1.0 to +0.5 s, hit, S1's SHOT_DETAIL merged | IT04 | Met | 12 of 12 records with SHOT_DETAIL merged, 73–74 trace points, the flinch visible |
| F13 | Zeroing per weapon | unit `zero` (2); IT08; `aim_host --zero` (demo) | Met | 11.5 mm RMS before, 0.64 mm after five shots (IT08); the demo zeroes weapon 2 before its session |
| F17 (S1) | S1's camera mode: pulses inside the exposures of its slot, shot pulse in the shot frame | IT10 (S1 v0.6.0 in Renode) | Met, with a finding | Pulse starts −30 to +236 µs from the plan, inside the ±250 µs margin. Pulses are 563–610 µs instead of 500 µs: 8 of 306 ended up to 80 µs after the exposure, with at least 86.5 % of each inside. A follow-up for S1 (SPEC §12). |

## Performance

| ID | Requirement (short) | Tests | Status | Measured / note |
|---|---|---|---|---|
| P01 | ≤ 2 mm RMS, ≤ 5 mm at the 99th percentile over the screen | IT02 | Met | 0.042 mm RMS, 0.082 mm at the 99th percentile. Against the simulator's own truth: the lens model is the one the calibration fits, and the screen is flat (SPEC §3.3). |
| P02 | HIT on CAN ≤ 25 ms after the break | IT03; IT10 | Met | 7.8–17.7 ms in true lane time (IT03); 9.5–10.5 ms with S1's firmware (IT10) |
| P03 | ≥ 99.9 % of shots with a position; ≤ 1 false HIT per 100 000 frames, interference set | IT06 (4 leak levels), IT09 (100 000 frames) | Met | IT06: 57 of 57 visible shots at 4, 8, 12 and 30 DN, no false HIT. IT09: 1953 of 1954 visible shots (99.95 %), 0 false HITs in 100 000 frames. |
| P04 | 100 % attributed one frame apart; ≥ 99 % in the same frame | IT05 | Met | 60 of 60 and 100 of 100 |
| P05 | ≤ 3 ms per frame, worst case; no heap after start-up | IT09; `test_library_has_no_heap`; cross-check timing | Met at the 99.9th percentile; worst case not shown | The library has no `malloc`; the heap is 10 800 bytes after the first frame and at the end of IT09. Thread CPU time 2.4 ms at the 99.9th percentile, 3.9 ms at the 99.99th, 13.4 ms worst: the slowest frames are ordinary ones, slowed by the virtual machine (SPEC §9.4). A worst-case claim needs a target board. |
| P06 | Calibration fit ≤ 0.3 px, held-out ≤ 0.5 px | IT01; unit `calib` | Met | 0.023 px and 0.023 px |
| P07 | Frame lane-time stamp within ±100 µs | IT03; IT09 | Met | Within 36 µs in IT03 and 44 µs in IT09. IT10 measures the weapon side instead: S1's pulses start −30 to +236 µs from the plan, timer latency included (F17 row). |

## Long run and end to end

| Test | Covers | Status | Result |
|---|---|---|---|
| IT09 | P03, P05: 100 000 frames, two weapons, interference | Pass (`pytest -m long`, about 5 min; weekly in CI) | 1953 of 1954 visible shots, 0 false HITs, 40 of 40 hidden shots NO_SPOT, 0.16 mm RMS; 2.4 ms at the 99.9th percentile; no heap growth |
| IT10 | F06, F08, F17 (S1), P02 with S1's firmware (v0.6.0) in Renode | Pass (replay in CI; `pytest -m cosim` re-records with Renode) | 8 of 8 S1 shots OK within 0.19 mm, 9.5–10.5 ms; 148 AIM frames per weapon; S1's pulse timing and length measured (F17 row) |
