# S3 — Camera Aim-Point Tracker

Specification, revision C (8 Nov 2026): week 2 done. Revision B (7 Nov) added the week 1 results (§2a, §3.3, §6.1, §6.2, §12) and the minimum of two time slots (§6.5). Revision C adds the week 2 results (§3.3, §6.3, §6.4, §9.3) and a separate `zero` module (§5).

Camera-based hit and aim tracking for a small-arms training lane. Each trainer weapon has an infrared emitter at the muzzle. An IR camera watches the projection screen, finds the emitter's spot in every frame and maps it to screen coordinates. It then tells the lane which weapon hit where:
- a **hit** for every shot, matched to the weapon's SHOT_EVENT by lane time;
- an **aim trace** for every weapon between shots, for after-action review (holding, flinch, follow-through).

It is the "camera hit detection" item from the parked hardware spec (`HW_PROJECTS/01_instrumented-trainer-module/SPEC.md`, §7.4 and v2 item 2). It closes the loop with S1: S1's firmware fires the emitter and sends the shot over CAN, and S3 turns that into a hit on the screen. No physical camera is needed: a camera simulator renders the frames with known ground truth, so every accuracy figure is measured, not estimated.

> Keep company names out of this repository, the README and the commit messages. It is a public portfolio project.

**Emitter: IR LED, not a laser.** The plan called S3 the "laser aim-point tracker". The emitter is an 850 nm IR LED with a collimating lens, as in the hardware spec (§2, "Safety and legal limits": assessed against IEC 62471, low duty cycle). Nothing in the tracker depends on the source type.

---

## 1. Why this project

| Junior electronics & firmware profile | Where S3 shows it |
|---|---|
| C/C++ for embedded systems | Real-time image pipeline in C: streaming, fixed memory, no heap after init, measured time per frame |
| Sensors and optics (the "big plus") | IR camera model: exposure, global shutter, noise, saturation, lens distortion, 850 nm filter, emitter strobing |
| CAN communication | Extends S1's DBC: SYNC lane time in; HIT, AIM and tracker status out; shot matching on lane time |
| Integration with the software team | Calibration file, after-action records merged with S1's SHOT_DETAIL, lane view for the instructor |
| Reliable electronics for real use | Health monitoring (frame drops, saturation, ambient IR, stale calibration), no-hit reasons instead of silent misses |

It reuses the calibration habits of industrial robot work: a camera-to-world transform fitted from known points, residuals checked before use, and a zeroing step per tool (here: per weapon).

Together with S1 and S2 it makes a complete lane story: the trainee's weapon (S1), the motion base under a vehicle cabin (S2), and the lane's sensing of where the trainee aims and hits (S3).

---

## 2. Scope

### In scope
- **Portable C library `aimtrk`**, unit-tested on `native_sim` (as `mcue` in S2):
  - spot detection: background estimate, threshold, connected components on run lengths,
  - sub-pixel centroid with quality figures (area, peak, SNR, saturation),
  - lens undistortion and screen homography,
  - calibration fit (homography plus radial and tangential distortion, Levenberg–Marquardt),
  - shot matching on lane time, aim tracks per weapon, spot-to-weapon assignment.
- **Linux service `aim_host`** (C): takes frames from a frame source, runs `aimtrk`, talks CAN (SocketCAN, or a file/pipe transport for CI), writes the after-action log.
- **Camera simulator** (Python): renders the screen as the IR camera sees it, frame by frame, with ground truth for every spot.
- **S1 firmware change (v0.6):** the emitter strobes inside the camera's exposure windows, scheduled on lane time (§6.5). This is the only change outside this repository.
- **Python tools:** calibration runner, lane view (screen with traces and hits), after-action plots, reference implementation for cross-checks.
- **Tests:** unit tests, a reference cross-check against NumPy and OpenCV, integration tests on rendered sequences, an end-to-end run with S1 in Renode, CI.

### Out of scope (v2)
- A real camera (V4L2 driver path, exposure control, a real lens calibration)
- Running the pipeline on a target board (embedded Linux SBC, or an MCU with a camera interface); the library is written so it can move there (§5)
- More than four weapons per camera, several cameras per screen
- Photodiode hit sensors for live-fire style targets (hardware spec v2 item 3)
- Shot-ID codes inside a single pulse (hardware spec v2 item 5); S3 uses time slots instead (§6.5)

---

## 2a. Reference lane (assumptions, checked in week 1)

| Item | Value | Note |
|---|---|---|
| Screen | 3.0 × 1.69 m (16:9), flat, matte | Projected scenario |
| Shooter distance | 5 m from the screen | Typical indoor lane; a parameter |
| Camera | 1 MP global-shutter monochrome sensor, 1280 × 800, 8-bit | Sensors of this class reach 120 fps at full resolution; S3 runs at **100 fps** so that frames line up with S1's 100 Hz SYNC |
| Camera position | Above the shooters, centred, about 4 m from the screen, tilted down | Gives the keystone the homography must remove |
| Field of view | About 3.2 m across at the screen | **About 2.5 mm per pixel** |
| Filter | 850 nm band-pass | The projector image is mostly removed |
| Exposure | 1.0 ms per frame, triggered by the lane on lane time | Exposure k starts at lane time k × 10 ms + 4.5 ms (mid-way between SYNC frames) |
| Emitter spot | About 2 mrad divergence: about 10 mm on the screen at 5 m, so about 4 px across | Gaussian PSF with σ ≈ 1.2 px, set in the simulator |
| Weapons per camera | 2 in the demo, up to 4 by design | Time slots, §6.5 |

At 5 m, 1 mm on the screen is 0.2 mrad of aim.

**Week 1 check** (`sim/camsim.py lane`, `results/lane_check.json`): with the camera 0.75 m above the screen centre and 4.0 m out, f = 1600 px and mild barrel distortion (k1 = −0.08, assumed), the whole screen is inside the frame with 35 px to spare. The scale is 2.54–2.59 mm/px at the centre, 2.43–2.57 mm/px at the top corners and 2.65–2.92 mm/px at the bottom corners (the keystone). With an emitter spot of σ = 2.8 mm on the screen plus 0.5 px of lens blur, the spot is σ = 1.08–1.26 px across the screen. The sensor values in the simulator (full well 10 000 e⁻, read noise 12 e⁻, 1 % PRNU, ambient 12 DN in 1 ms) are assumptions for a sensor of this class, not datasheet values; together they give a background noise of 0.71 DN.

---

## 3. Requirements

### 3.1 Functional
| ID | Requirement |
|---|---|
| F01 | Take frames from a frame source with a frame index and the exposure window in lane time. Sources: the camera simulator through a pipe, or a recorded file. |
| F02 | Detect IR spots: running background estimate, threshold at background + k·σ, connected components from run lengths, rejection by area, peak and shape. |
| F03 | Locate each spot to sub-pixel accuracy (intensity-weighted centroid on background-subtracted pixels). Report area, peak, SNR and a saturation flag. |
| F04 | Map image points to screen millimetres: undistort (radial k1, k2; tangential p1, p2), then apply the screen homography. Points outside the screen are reported as off-screen, not dropped. |
| F05 | Calibration mode: capture a projected dot grid (9 × 6), find and order the dots, fit the homography and distortion, report the residuals, and store the result in `calib.json` with a version and CRC. Refuse a fit whose residual is over the limit (P06). |
| F06 | Lane time: estimate offset and drift from SYNC (same scheme as S1 F12). Every frame carries its exposure window in lane time. |
| F07 | Aim strobing: weapon slot s flashes in frames where `frame_index mod N_slots = s` (§6.5). Keep one aim track per weapon (constant-velocity filter), fed only by spots in that weapon's slots. |
| F08 | Shot matching: a SHOT_EVENT with lane time t maps to the frame whose exposure contains the shot pulse (§6.5). The spot in that frame that belongs to the shooter becomes the HIT. |
| F09 | Several spots in one frame (other weapons' aim strobes, two shots): assign spots to weapons by slot and by distance to each weapon's predicted track position, with a gate. Report AMBIGUOUS rather than guess when the gate cannot separate them. |
| F10 | Send HIT for every SHOT_EVENT: weapon, shot number, x/y in mm, quality, latency. If there is no usable spot, send HIT with a reason (no spot, off-screen, ambiguous, saturated, frame dropped). |
| F11 | Send AIM at each weapon's slot rate while it has a track, and TRK_STATUS at 1 Hz: frame rate, dropped frames, worst processing time, ambient level, calibration age, fault bits. |
| F12 | After-action record per shot: aim trace from 1.0 s before the break to 0.5 s after, the hit point, and S1's SHOT_DETAIL (take-up, squeeze, jerk, cant, flinch) merged by shot number. |
| F13 | Zeroing per weapon: fire N shots (default 5) at a zeroing target and store the mean offset between hit and target. The offset covers the emitter-to-bore misalignment and is applied to that weapon's hits. |

### 3.2 Performance (measured against simulator ground truth)
| ID | Requirement |
|---|---|
| P01 | Hit accuracy over the whole screen at nominal signal: ≤ 2 mm RMS and ≤ 5 mm at the 99th percentile, after calibration and zeroing. |
| P02 | HIT on CAN ≤ 25 ms after the trigger break, in lane time. |
| P03 | Detection: ≥ 99.9 % of shots give a HIT with a position. False HITs (a position where there was no shot spot): ≤ 1 per 100 000 frames with the interference set (projector leak, reflections, a sunlit window, hot pixels). |
| P04 | Attribution: 100 % correct when shots from different weapons are at least one frame apart; ≥ 99 % correct when two weapons fire into the same frame. |
| P05 | Processing ≤ 3 ms per 1280 × 800 frame, worst case, one core of the CI runner; no heap allocation after start-up. |
| P06 | Calibration: fit residual ≤ 0.3 px RMS; held-out grid points ≤ 0.5 px RMS. |
| P07 | Lane-time stamp of each frame within ±100 µs of the simulator's true exposure time. |

### 3.3 Error budget for P01 (measured in weeks 1 and 2)
| Source | Size | On the screen |
|---|---|---|
| Centroid noise at nominal SNR | 0.1 px | 0.25 mm |
| Calibration residual | 0.3 px | 0.75 mm |
| Spot motion during the 0.5 ms strobe (muzzle at 50 mrad/s) | 0.025 mrad | 0.13 mm |
| **Total (root sum of squares)** | | **about 0.8 mm**, against the 2 mm limit |

Week 1 measured the first line (§6.2): 0.018 px RMS at the nominal spot (0.05 mm), well inside the 0.1 px allowed.

Week 2 measured the rest:

| Source | Budget | Measured |
|---|---|---|
| Calibration residual | 0.3 px (0.75 mm) | 0.023 px fit, 0.023 px held-out (IT01); the mapping is within 0.04 mm of the true camera over the whole screen |
| Hit accuracy, everything together (P01) | ≤ 2 mm RMS, ≤ 5 mm at the 99th percentile | 0.039 mm RMS and 0.082 mm at the 99th percentile over 500 shots (IT02) |

The simulated system meets P01 by a factor of 50. That margin shows what the simulation leaves out, not a reason to tighten P01:
- The simulator's lens follows the same Brown–Conrady model the calibration fits. Only the focal length and principal point differ from the nominal values, so the fit can match it almost exactly. A real lens has terms the model does not hold.
- The screen is perfectly flat, and the projected dots sit exactly where the grid says. A fabric screen that bows, or a projector's own distortion, moves real points by millimetres.

The 2 mm limit keeps room for those.

Not in the budget: screen flatness and the projector's own geometry (the screen is assumed flat), and the shooter's eye-to-bore parallax (a sight setting, not the tracker's).

---

## 4. Architecture

```
 camera simulator (Python)            S1 firmware in Renode (one or two weapons)
   frames + ground truth                emitter strobes on lane time; SHOT_EVENT, SHOT_DETAIL
        │ pipe (raw 8-bit + header)              │ CAN (SocketCAN vcan0, or file transport in CI)
        ▼                                        ▼
 ┌────────────────────────── aim_host (C, Linux) ──────────────────────────┐
 │ frame_src ─► aimtrk_detect ─► aimtrk_geom ─► aimtrk_track ─► aimtrk_match│
 │  (index,       (background,     (undistort,     (per-weapon     (SHOT_EVENT│
 │   exposure)     threshold,       homography,     tracks,         → frame, │
 │                 runs, blobs,     zeroing)        gating)         HIT)     │
 │                 centroid)                                                 │
 │ timesync (SYNC) · health · after-action log · CLI                         │
 └───────────────────────────────────┬───────────────────────────────────────┘
                                     │ HIT, AIM, TRK_STATUS on CAN; JSON lines log
                                     ▼
                           lane view (Python): screen, aim traces, hits, after-action
```

Design choices:
- **The library has no I/O.** Frames come in as a buffer with a stride; results go out as structs. This makes it testable on `native_sim` and portable to a target board.
- **Detection streams row by row.** Run lengths are found per row and merged into blobs with a union-find on a fixed-size table, so memory does not grow with the image and the same code works on a line-buffered camera interface.
- **Integer arithmetic per pixel, floating point per spot.** The background, threshold and run finding are 8/16-bit integer work; only the few spots per frame use floating point (centroid, undistortion, homography).
- **Static memory.** Blob tables, tracks and the background image are sized at init from the configuration; nothing is allocated per frame (P05).
- **Lane time everywhere.** Frames, shots, hits and aim samples all carry lane time, so the after-action review lines up S1's trigger data with S3's optical trace.

---

## 5. Library modules (`aimtrk`)

| Module | Responsibility |
|---|---|
| `detect` | Background (per-pixel running mean on a 1/4-scale grid, updated only where there is no spot), noise estimate, threshold, row runs, union-find blobs, rejection rules |
| `centroid` | Intensity-weighted centroid, second moments (ellipticity), SNR; saturated spots flagged and located from the unsaturated rim |
| `geom` | Distortion model, undistortion by Newton iteration, homography (normalised DLT), image ↔ screen mapping |
| `calib` | Dot-grid detection and ordering, homography and distortion fit by Levenberg–Marquardt, residuals, hold-out check |
| `track` | Constant-velocity track per weapon (α-β filter), prediction to any lane time, gating |
| `match` | SHOT_EVENT → frame, spot assignment (slot, gate, nearest predicted position; optimal assignment for ≤ 4 weapons), HIT and no-hit reasons |
| `zero` | Zeroing offset per weapon: mean hit − target over N shots, refused if too few shots or the group is too scattered; the old offset stays until a new one succeeds (week 2: separate from `match`, so IT08 could run before `match` exists) |
| `linalg` | Linear solve and symmetric eigenproblem (Jacobi) for the rim fit, the DLT and the calibration |
| `timesync` | SYNC → offset and drift (shared design with S1) |

Python reference (`tools/reference/`): the same steps in NumPy, plus OpenCV's `findHomography` and `calibrateCamera` as independent checks for `geom` and `calib`.

---

## 6. Algorithms

### 6.1 Spot detection
1. Background b(x, y) on a 1/4-scale grid, updated with a slow running mean, frozen where a spot was found.
2. Noise σ from the median absolute deviation of (frame − background), per frame.
3. Threshold T = b + max(k·σ, T_min), k = 6 by default.
4. Runs of pixels above T per row; runs touching runs in the previous row join a blob (union-find).
5. Reject blobs that are too small (< 3 px), too large (reflections, a window), too elongated, or too weak (peak SNR below the limit).

**Week 1 implementation** (`aimtrk/src/detect.c`):
- The background uses 4 × 4 pixel cells in 16-bit fixed point (DN × 256, at most 65 280). It is updated by 1/16 per frame (time constant 160 ms) and frozen in the cells under a spot plus one cell around it.
- The noise comes from the median absolute deviation of every 4th pixel of every 4th row, read from a histogram.
- One pass over the frame does the threshold, the cell sums and the runs.
- Defaults: k = 6, T_min = 8 DN, area 3–300 px, extent ≤ 24 px, elongation ≤ 3, peak SNR ≥ 8.
- Memory for 1280 × 800 is 475 kB, fixed at start-up.
- On 40 simulated frames, the C code agrees with an independent NumPy/SciPy version (`scipy.ndimage.label` for the blobs) to 2e-4 px.
- It takes 1.1 ms per frame on average and 1.9 ms at worst on the development PC (WSL2), against the 3 ms of P05.

### 6.2 Centroid
- Weighted centroid over the blob's pixels plus a one-pixel border, weights = frame − background, clipped at 0.
- Saturated spots: the plateau's centroid is biased by clipping. Flag the spot, and locate it from the unsaturated rim; a week 1 test measures the bias.
- Expected noise: for a Gaussian spot the centroid error falls roughly as σ_PSF / SNR; week 1 measures the curve on rendered spots (P01 budget).

**Week 1 results** (`tools/reference/centroid_curve.py`, `results/centroid_curve.json`; 400 spots per point at random sub-pixel positions, full sensor model):

| Spot σ_PSF | RMS error at the nominal spot (peak 150 DN) | Weakest spot found every time | Error below 0.1 px from |
|---|---|---|---|
| 0.8 px | 0.020 px | 25 DN | SNR 35 |
| **1.2 px** | **0.018 px** | **16 DN** | **SNR 23 (16 DN, 0.093 px)** |
| 1.6 px | 0.016 px | 12 DN | SNR 24 |
| 2.0 px | 0.015 px | 12 DN | SNR 25 |

- **Spot size:** σ_PSF ≈ 1.2 px is enough, so no defocus is needed; larger spots gain little.
- **Signal margin:** the nominal spot has about 9 times the signal of the weakest usable one (150 against 16 DN). That leaves room for emitter ageing, off-axis loss and longer ranges.
- **Bias without noise:** an error of up to 0.034 px (σ 1.2 px) remains, depending on where the spot sits within its pixel. It comes mostly from the mask (the blob plus a one-pixel border), which cuts the spot's tails; on a whole window without a mask the bias is 0.010 px. That is small enough to keep the one-pixel border.
- **Saturation:** not a problem. Up to 24 times full scale, the plain centroid stays at 0.03 px or better and the rim fit at 0.009 px or better. The rim fit is kept because it improves as the spot gets brighter.

### 6.3 Geometry
- Distortion: Brown–Conrady with k1, k2, p1, p2 about the principal point.
- Screen mapping: undistorted image point → homography H → screen (x, y) in mm, origin at the screen centre, y up.
- Undistortion is the inverse of the distortion model, solved by Newton iteration (3 iterations, checked against the forward model).

**Week 2 implementation** (`aimtrk/src/geom.c`):
- **Undistortion:** Newton iteration with the analytic Jacobian, to 1e-12 (usually 3–4 iterations). A root where the Jacobian is not positive definite is refused, because a polynomial distortion folds over far out, and a root there is not the physical inverse.
- **Homography:** normalised DLT (Hartley), with the smallest singular vector taken from a Jacobi eigen-decomposition of AᵀA. A point set without a unique solution (collinear points, fewer than four) is refused.
- **Against OpenCV** (`tools/reference/geom_check.py`, in the CI cross-check):
  - Distortion matches `projectPoints` to 1e-16.
  - Undistortion matches `undistortPoints` to 1e-12.
  - On exact data, the C homography reproduces the true one to 5e-13 px. OpenCV's `findHomography` is 3e-5 px off: its refinement stops earlier.

### 6.4 Calibration
- The projector shows a 9 × 6 dot grid; the camera captures it with a long exposure. Projectors leak some light at 850 nm; in the simulator the leak is a parameter, and the real fallback is IR LED markers at known screen points.
- Fit: normalised DLT for H, then Levenberg–Marquardt on H and the distortion together, minimising the reprojection error in pixels.
- Accept only if P06 holds; otherwise keep the previous calibration and raise a fault bit.
- Every fifth grid point is held out of the fit and used to check it.

**Week 2 implementation** (`aimtrk/src/calib.c`, `tools/calib/calibrate.py`):
- **Single view.** The camera sees one plane, so its focal length and principal point cannot be fitted apart from the pose. They stay at the lens's nominal values. The homography (8 degrees of freedom) absorbs their error, and the distortion k1, k2, p1, p2 is fitted with it. Levenberg–Marquardt on 12 parameters, with screen coordinates normalised.
  - In the tests, the nominal values are deliberately off: 1590 px against the true 1600 px, and a principal point 4.3 px from the true one. The fit absorbs that: 0.0003 px residual on noise-free dots (unit test).
- **Ordering.** The four extreme dots are the grid's corners (the camera must be roughly upright). A homography through them predicts every grid point, each grid point takes the nearest dot inside a 30 px gate, and a second pass repeats it with all matches. Shuffled dots and spurious detections between grid points give the same result (unit test).
- **Refusals.** A missing dot, too few dots, or a residual over P06 refuse the fit, and the previous calibration stays in use. IT01 refuses:
  - a grid with a block of dots hidden by a person;
  - a grid whose dots are displaced by 8 mm: fit residual 3 px against the 0.3 px limit.
- **calib.json.** The Python runner writes the geometry, the fit report and the grid with a version and a CRC-32. A changed byte or a wrong version is refused on loading. `aim_host` (C) reads the same file in week 3.
- **Against OpenCV.** On the same grid points, with the true intrinsics for both, the C fit and `calibrateCamera` agree:
  - to 1e-5 px on noise-free dots;
  - to 0.003 px and 0.008 mm on the screen with 0.02 px dot noise.
- **Findings:**
  - **k2 on its own is not trustworthy.** A single plane determines it poorly. Over 20 draws of 0.02 px dot noise, the fitted k2 ranged from 0.0003 to 0.0195 around the true 0.01, while the mapping stayed within 0.073 mm of the truth inside the grid (`results/crosscheck.json`, `k2_over_20_noise_draws`). Only the mapping is used, never k2 alone.
  - **Error grows outside the grid.** With 0.05 px dot noise, the error at the screen corners, 150 mm and 95 mm outside the grid, reaches 0.75 mm against 0.13 mm inside (unit test `calib.test_fit_with_noise`). The grid should therefore reach as close to the screen edges as the projector allows.

### 6.5 Strobing on lane time (S1 v0.6)
A 0.5 ms emitter pulse fired at a random moment overlaps a 1 ms exposure only 15 % of the time at 100 fps, and lies fully inside it 5 % of the time. So the weapon schedules its pulses into the exposure windows, which it knows from lane time (S1 F12):
- **Aim pulses:** weapon slot s pulses in the middle of exposure k when `k mod N_slots = s`. With 2 weapons each has a 50 Hz aim trace; with 4, 25 Hz.
- **Shot pulse:** at the trigger break the weapon pulses in the middle of the **next** exposure, whatever its slot, and skips its next aim pulse. That adds at most one frame (10 ms) to the optical latency; the recoil valve still fires at once (S1 T02 unchanged).
- **Who fired:** the SHOT_EVENT carries the break time t. The shot frame is the first exposure starting after t + the firmware's scheduling margin. In that frame the shooter's spot is the one nearest the shooter's predicted aim position; another weapon's aim strobe in the same frame belongs to that weapon's slot and is near that weapon's track.
- **Two shots in one frame:** both shooters are known from their SHOT_EVENTs; the two spots are assigned by distance to the two predicted positions. If the predictions are closer than the gate allows, the HITs are sent as AMBIGUOUS (P04 counts these).
- Timing margin: the pulse sits in the middle of the exposure, so the weapon's lane-time estimate may be off by ±250 µs before the pulse leaves the exposure. S1's time sync must stay inside ±100 µs (P07, and a test on the S1 side).
- **At least two slots** (week 1): the background is not updated under a spot (§6.1). If a weapon flashed in every frame, the background at a steady aim point would never be updated, and it would go wrong there when the projected scene changes. So N_slots ≥ 2 even with one weapon: a single weapon flashes in every other frame, and the frames in between update the background under its aim point (unit test `detect.test_background_tracks_and_freezes`).

### 6.6 Tracks
- α-β filter per weapon on screen coordinates, updated in that weapon's slots.
- A track is lost after 0.5 s without a spot (weapon lowered or off-screen) and restarts on the next spot in its slot.
- Prediction gives the expected position at any lane time, for gating and assignment.

---

## 7. CAN protocol (extends S1's `trainer.dbc`)

Same bus, ID layout `function (4 bits) | node ID (7 bits)`. S1 uses functions 0x1–0x9. The tracker adds:

| Frame | Function | Dir | Rate | Signals |
|---|---|---|---|---|
| HIT | 0xA | tracker → lane | per SHOT_EVENT | node ID of the shooter (in the ID), shot_no (u16), x_mm (i16), y_mm (i16), result (OK, NO_SPOT, OFF_SCREEN, AMBIGUOUS, SATURATED, DROPPED), quality (u8), latency 0.1 ms (u8, saturating) |
| AIM | 0xB | tracker → lane | per slot frame | node ID (in the ID), x_mm (i16), y_mm (i16), frame_seq (u16) |
| TRK_STATUS | 0xC | tracker → lane | 1 Hz | fps ×10 (u16), dropped (u8), worst processing 0.1 ms (u8), ambient (u8), calibration age h (u8), faults (u8) |

The tracker uses node ID 0x7E. Bus load with 2 weapons (S1's frames at 900 rpm plus 100 AIM/s, HIT per shot, TRK_STATUS) is about 310 frames/s, under 10 % of 500 kbit/s; week 3 measures it. As in S1, a unit test encodes every frame in C and decodes it with `cantools`.

---

## 8. Camera simulator (`sim/camsim.py`)

Renders 1280 × 800 8-bit frames, streamed into a pipe with a small header (frame index, exposure start and end in lane time):
- **Geometry:** a pinhole camera with pose and focal length from §2a, plus the distortion model, looking at the screen plane.
- **Scene in IR:** projector leak (the scenario image at a low level, flickering at the projector's refresh rate), room ambient, an optional sunlit window, specular reflections from a target frame.
- **Spots:** Gaussian PSF per emitter pulse, energy from the pulse's overlap with the exposure, smeared along the aim motion during the pulse.
- **Sensor:** shot noise, read noise, fixed-pattern noise, hot pixels, 8-bit quantisation and saturation.
- **Aim motion:** from S1's IMU recordings (steady aim, shaky aim, flinch before break, cant), integrated to muzzle angles and projected to the screen, plus scripted target engagements.
- **Ground truth:** for every pulse, the true screen position and which weapon fired it, in a JSON lines file.
- **Interference set** (P03): projector leak at three levels, a reflective target frame, a sunlit window, hot pixels, a passer-by blocking part of the view.

Rendering cost is measured in week 1 (open point §11). If it is too slow for long runs, the simulator renders only a region around each spot over a cached background.

---

## 9. Verification

### 9.1 Unit tests (ztest on `native_sim`)
| Suite | Covers |
|---|---|
| detect | Thresholds, runs, union-find across rows, rejection rules, background freeze under spots, noise estimate |
| centroid | Sub-pixel accuracy on rendered spots against SNR (the P01 curve), saturation bias and the rim method, border pixels |
| geom | Linear solve and eigen-decomposition; distortion round trip and the fold-over refusal; homography (DLT) on known point sets and degenerate ones; image ↔ screen round trip and off-screen handling |
| calib | Fit on synthetic grids, noise-free and with noise, with deliberately wrong nominal intrinsics; ordering with shuffled and spurious dots; missing dots; rejection of a bad fit (P06), the previous calibration kept; iteration limit |
| zero | Offset and its use; refusal of too few shots and of a scattered group, the old offset kept |
| track | Constant-velocity tracking, gating, loss and restart |
| match | Shot frame selection on lane time, slot assignment, two-shots-in-one-frame, AMBIGUOUS cases |
| proto | Every frame against the DBC |

### 9.2 Reference cross-check (`tools/reference/`)
- C vs NumPy: centroids, distortion, homography, calibration fit on the same inputs.
- C vs OpenCV: homography against `findHomography`; calibration residuals against `calibrateCamera` on the same grid images.

### 9.3 Integration tests (rendered sequences, `aim_host` with the file transport)
| Test | Covers | Sequence |
|---|---|---|
| IT01 calibration | F05, P06 | Grid capture, fit, residual and hold-out check; a bad grid is refused |
| IT02 single weapon accuracy | F02–F04, F10, P01 | 500 shots spread over the screen, steady aim |
| IT03 latency | F08, P02 | Shots at random break times; HIT latency in lane time |
| IT04 aim trace | F07, F11, F12 | Shaky aim and flinch; trace vs ground truth; after-action record |
| IT05 two weapons | F09, P04 | Two weapons, staggered and same-frame shots |
| IT06 interference | F02, P03 | The interference set; detections and false HITs |
| IT07 no-hit reasons | F10 | Weapon pointed off-screen, emitter blocked, dropped frame, saturated spot |
| IT08 zeroing | F13 | Emitter misaligned by a known offset; zeroing removes it |
| IT09 long run | P03, P05 | 100 000 frames, two weapons: false HITs, worst processing time, no heap growth |
| IT10 end to end with S1 | F06, F08, P07 | S1 firmware (v0.6) in Renode over vcan0, two weapons firing; local and in the demo, not in CI (no vcan on hosted runners) |

**Week 2:** IT01, IT02 and IT08 pass (`tests/integration/`, pytest). Until `aim_host` exists (week 3), they drive the C library through ctypes on frames from the camera simulator. The pipeline is the same: render, detect, map to the screen, zero. The HIT frame part of IT02 (F10) comes with `match` in week 3.

| Test | Result |
|---|---|
| IT01 | 54 of 54 dots found; fit 0.023 px RMS, held-out 0.023 px RMS (P06: 0.3 / 0.5 px); the calibrated mapping is within 0.041 mm of the true camera everywhere on the screen. A displaced grid and a partly hidden grid are both refused. |
| IT02 | 500 of 500 shots found; 0.039 mm RMS, 0.082 mm at the 99th percentile, worst 0.093 mm (P01: 2 / 5 mm); no bias above 0.005 mm. |
| IT08 | Emitter 2 mrad right and 1 mrad down from the bore: 11.5 mm RMS error before zeroing. Five zeroing shots at the centre, with 2 mm of aim scatter, measure the offset to 0.29 mm. After zeroing: 0.63 mm RMS and 1.2 mm worst over the screen. The rest is the zeroing shots' mean aim error, plus the offset growing towards the screen edges, because the misalignment is an angle. |

Plots: `results/plots/it01_calibration.png`, `results/plots/it02_accuracy.png`.

### 9.4 Honest limits
- **The camera is simulated.** Accuracy is measured against the simulator's own truth. A real lens, sensor and screen bring effects the model leaves out or simplifies (screen texture, non-flat screens, lens flare, sensor non-linearity).
- **Processing time is measured on an x86 CI runner**, not on a target board. P05 shows the algorithm's cost and its memory behaviour; a target board needs its own measurement.
- **Renode time is virtual** (as in S1 and S2): the end-to-end test proves the timing logic and the matching, not silicon latency.
- **One camera, one flat screen, up to four weapons.**

---

## 10. Host tools and deliverables
- `aim_host` with a CLI: `run`, `calibrate`, `zero`, `status`.
- `tools/lane/lane_view.py`: the screen with targets, live aim traces per weapon, hits with shot numbers, and the after-action panel (aim trace before the break, hit, S1 trigger metrics).
- `scripts/demo.sh`: S1 in Renode with two weapons, the camera simulator, `aim_host` and the lane view, recorded to a session log, plots, a GIF for the README and a video.
- `docs/traceability.md`: every requirement to its tests.
- README with screenshots, the GIF and the results table; CI badge.

---

## 11. Milestones (about 8–10 h/week)

| Week | Work | Done when |
|---|---|---|
| 1 ✓ | Repository from the S2 template; camera simulator (geometry, spots, sensor, ground truth); `detect` and `centroid` in C with the NumPy reference; centroid accuracy vs SNR | detect and centroid suites green; the centroid curve meets the P01 budget line. **Done 7 Nov:** 21 of 21 unit tests pass, 99.7 % line coverage; C matches NumPy on 40 frames; 0.018 px at the nominal spot |
| 2 ✓ | `geom` and `calib`; OpenCV cross-check; accuracy over the screen; zeroing | IT01, IT02 and IT08 pass; P01 shown on plots. **Done 8 Nov:** 35 of 35 unit tests, 97.8 % line coverage; IT01, IT02 and IT08 pass; P01 0.039 mm RMS |
| 3 | `timesync`, `track`, `match`; `aim_host` with the file transport and SocketCAN; DBC frames; S1 v0.6 emitter strobing with its own tests | IT03–IT07 pass; S1's tests still pass with v0.6 |
| 4 | IT09, IT10; lane view; demo, GIF and video; traceability; CI; README | Portfolio-ready repository with green CI |

---

## 12. Open points to verify early (week 1)

- [x] **Simulator speed** (`camsim.py bench`, `results/camsim_bench.json`): a full frame takes 48 ms with Poisson shot noise, or 26 ms with the Gaussian approximation.
  - At those rates, the 100 000 frames of IT09 would take 43–80 minutes. Live 100 fps needs under 10 ms per frame.
  - So IT09 and the live demo need the region-only renderer (§8), in week 3.
- [x] **Centroid accuracy against SNR and spot size:** σ_PSF ≈ 1.2 px is enough, giving 0.018 px RMS at the nominal spot (§6.2).
- [x] **Saturation:** no limit up to 24 times full scale; the rim fit stays within 0.009 px (§6.2).
- [ ] **S1 time sync accuracy:**
  - S1's unit tests give ≤ 50 µs at ±200 ppm drift (S1 README, `timesync` row).
  - In Renode the absolute accuracy cannot be measured, because frames arrive after a host-dependent delay (S1 README, "Honest limits").
  - P07 will be checked in virtual time in IT10 (week 4).
- [x] **Frame transport:** a pipe carries 1270 MB/s (a Python writer sending 1 MB frames into `cat`), 12 times the 100 MB/s needed, so shared memory is not needed.
- [ ] **Projector leak at 850 nm:** moved to week 3, with the interference set (IT06).

---

## 13. Interview story (60 seconds)

> "I wrote the camera side of a training lane. Each trainer weapon flashes an IR LED, and the camera has to say which weapon hit where. A flash at a random time almost never lands in a 1 ms exposure, so the weapon firmware strobes on lane time inside the exposure windows, and between shots each weapon flashes in its own frame slot, which gives an aim trace for the after-action review. The C pipeline finds the spot to a tenth of a pixel, removes the lens distortion and maps it to the screen with a calibrated homography, and matches it to the weapon's CAN shot event by lane time. Because the camera is simulated with ground truth, every accuracy figure is measured: [hit accuracy] RMS over the screen, HIT on the bus [latency] after the break, and [false hits] in 100 000 frames of interference."
