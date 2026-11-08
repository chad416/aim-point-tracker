# Camera Aim-Point Tracker: specification

Revision G. Requirements, design, verification and what testing found; the revision history is in §13.

Camera-based hit and aim tracking for a small-arms training lane. Each trainer weapon has an infrared emitter at the muzzle. An IR camera watches the projection screen, finds the emitter's spot in every frame and maps it to screen coordinates. It then tells the lane which weapon hit where:
- a **hit** for every shot, matched to the weapon's SHOT_EVENT by lane time;
- an **aim trace** for every weapon between shots, for after-action review (holding, flinch, follow-through).

No physical camera is needed: a camera simulator renders the frames with known ground truth, so every accuracy figure is measured against that truth, not estimated.

**Emitter: an 850 nm IR LED with a collimating lens.** Nothing in the tracker depends on the source type; a laser would be handled the same way.

---

## 1. Context

The tracker is one of three parts of a training lane in software. The weapon firmware is [trainer-weapon-firmware-twin](https://github.com/chad416/trainer-weapon-firmware-twin) (called S1 in some file and variable names here, such as `tests/data/s1_*` and `S1_ELF`); it fires the emitter and sends the shot over CAN. The motion base under a vehicle cabin is [motion-platform-controller](https://github.com/chad416/motion-platform-controller). This repository turns the weapon's flashes into hits and aim traces.

---

## 2. Scope

### In scope
- **Portable C library `aimtrk`**, unit-tested on Zephyr's `native_sim`:
  - spot detection: background estimate, threshold, connected components on run lengths,
  - sub-pixel centroid with quality figures (area, peak, SNR, saturation),
  - lens undistortion and screen homography,
  - calibration fit (homography plus radial and tangential distortion, Levenberg–Marquardt),
  - shot matching on lane time, aim tracks per weapon, spot-to-weapon assignment.
- **Linux service `aim_host`** (C): takes frames from a frame source, runs `aimtrk`, talks CAN (SocketCAN, or a file transport for repeatable runs), writes the after-action log.
- **Camera simulator** (Python): renders the screen as the IR camera sees it, frame by frame, with ground truth for every spot.
- **Weapon firmware change (v0.6, in trainer-weapon-firmware-twin):** the emitter strobes inside the camera's exposure windows, scheduled on lane time (§6.5). This is the only change outside this repository.
- **Python tools:** calibration runner, lane view (screen with traces and hits), after-action plots, reference implementation for cross-checks.
- **Tests:** unit tests, a reference cross-check against NumPy and OpenCV, integration tests on rendered sequences, an end-to-end run with the weapon firmware in Renode, a live SocketCAN run on vcan, CI.

### Out of scope
- A real camera (V4L2 driver path, exposure control, a real lens calibration)
- Running the pipeline on a target board (embedded Linux SBC, or an MCU with a camera interface); the library is written so it can move there (§5)
- More than four weapons per camera, several cameras per screen
- Photodiode hit sensors on targets
- Shot-ID codes inside a single pulse; time slots are used instead (§6.5)

---

## 2a. Reference lane (assumptions, checked in M1)

| Item | Value | Note |
|---|---|---|
| Screen | 3.0 × 1.69 m (16:9), flat, matte | Projected scenario |
| Shooter distance | 5 m from the screen | Typical indoor lane; a parameter |
| Camera | 1 MP global-shutter monochrome sensor, 1280 × 800, 8-bit | Sensors of this class reach 120 fps at full resolution; the tracker runs at **100 fps** so that frames line up with the 100 Hz SYNC |
| Camera position | Above the shooters, centred, about 4 m from the screen, tilted down | Gives the keystone the homography must remove |
| Field of view | About 3.2 m across at the screen | **About 2.5 mm per pixel** |
| Filter | 850 nm band-pass | The projector image is mostly removed |
| Exposure | 1.0 ms per frame, triggered by the lane on lane time | Exposure k starts at lane time k × 10 ms + 4.5 ms (mid-way between SYNC frames) |
| Emitter spot | About 2 mrad divergence: about 10 mm on the screen at 5 m, so about 4 px across | Gaussian PSF with σ ≈ 1.2 px, set in the simulator |
| Weapons per camera | 2 in the demo, up to 4 by design | Time slots, §6.5 |

At 5 m, 1 mm on the screen is 0.2 mrad of aim.

**Check** (`sim/camsim.py lane`, `results/lane_check.json`): with the camera 0.75 m above the screen centre and 4.0 m out, f = 1600 px and mild barrel distortion (k1 = −0.08, assumed), the whole screen is inside the frame with 35 px to spare. The scale is 2.54–2.59 mm/px at the centre, 2.43–2.57 mm/px at the top corners and 2.65–2.92 mm/px at the bottom corners (the keystone). With an emitter spot of σ = 2.8 mm on the screen plus 0.5 px of lens blur, the spot is σ = 1.08–1.26 px across the screen. The sensor values in the simulator (full well 10 000 e⁻, read noise 12 e⁻, 1 % PRNU, ambient 12 DN in 1 ms) are assumptions for a sensor of this class, not datasheet values; together they give a background noise of 0.71 DN.

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
| F06 | Lane time: estimate offset and drift from SYNC (the weapon firmware's scheme, its F12). Every frame carries its exposure window in lane time. |
| F07 | Aim strobing: weapon slot s flashes in frames where `frame_index mod N_slots = s` (§6.5). Keep one aim track per weapon (constant-velocity filter), fed only by spots in that weapon's slots. |
| F08 | Shot matching: a SHOT_EVENT with lane time t maps to the frame whose exposure contains the shot pulse (§6.5). The spot in that frame that belongs to the shooter becomes the HIT. |
| F09 | Several spots in one frame (other weapons' aim strobes, two shots): assign spots to weapons by slot and by distance to each weapon's predicted track position, with a gate. Report AMBIGUOUS rather than guess when the gate cannot separate them. |
| F10 | Send HIT for every SHOT_EVENT: weapon, shot number, x/y in mm, quality, latency. If there is no usable spot, send HIT with a reason (no spot, off-screen, ambiguous, saturated, frame dropped). |
| F11 | Send AIM at each weapon's slot rate while it has a track, and TRK_STATUS at 1 Hz: frame rate, dropped frames, worst processing time, ambient level, calibration age, fault bits. |
| F12 | After-action record per shot: aim trace from 1.0 s before the break to 0.5 s after, the hit point, and the weapon's SHOT_DETAIL (take-up, squeeze, jerk, cant, flinch) merged by shot number. |
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

### 3.3 Error budget for P01
| Source | Budget | On the screen | Measured |
|---|---|---|---|
| Centroid noise at nominal SNR | 0.1 px | 0.25 mm | 0.018 px RMS at the nominal spot (§6.2) |
| Calibration residual | 0.3 px | 0.75 mm | 0.023 px fit, 0.023 px held-out (IT01); the mapping is within 0.055 mm of the true camera over the whole screen |
| Spot motion during the 0.5 ms strobe (muzzle at 50 mrad/s) | 0.025 mrad | 0.13 mm | |
| **Total (root sum of squares)** | | **about 0.8 mm**, against the 2 mm limit | **0.042 mm RMS, 0.082 mm at the 99th percentile** over 500 shots (IT02) |

The simulated system meets P01 by a factor of 50. That margin shows what the simulation leaves out, not a reason to tighten P01:
- The simulator's lens follows the same Brown–Conrady model the calibration fits. Only the focal length and principal point differ from the nominal values, so the fit can match it almost exactly. A real lens has terms the model does not hold.
- The screen is perfectly flat, and the projected dots sit exactly where the grid says. A fabric screen that bows, or a projector's own distortion, moves real points by millimetres.

The 2 mm limit keeps room for those. Not in the budget: screen flatness and the projector's own geometry, and the shooter's eye-to-bore parallax (a sight setting, not the tracker's).

---

## 4. Architecture

```
 camera simulator (Python)            weapon firmware in Renode (one or two weapons)
   frames + ground truth                emitter strobes on lane time; SHOT_EVENT, SHOT_DETAIL
        │ pipe (raw 8-bit + header)              │ CAN (SocketCAN, or a log file for repeatable runs)
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
- **Lane time everywhere.** Frames, shots, hits and aim samples all carry lane time, so the after-action review lines up the weapon's trigger data with the optical trace.

---

## 5. Library modules (`aimtrk`)

| Module | Responsibility |
|---|---|
| `detect` | Background (per-pixel running mean on a 1/4-scale grid, updated only where there is no spot), noise estimate, threshold, row runs, union-find blobs, rejection rules |
| `centroid` | Intensity-weighted centroid, second moments (ellipticity), SNR; saturated spots flagged and located from the unsaturated rim |
| `geom` | Distortion model, undistortion by Newton iteration, homography (normalised DLT), image ↔ screen mapping |
| `calib` | Dot-grid detection and ordering, homography and distortion fit by Levenberg–Marquardt, residuals, hold-out check |
| `track` | Constant-velocity track per weapon (α-β filter), prediction to any lane time, gating |
| `match` | SHOT_EVENT → frame, spot assignment (slot, gate, nearest predicted position; exhaustive for ≤ 4 weapons), HIT and no-hit reasons |
| `zero` | Zeroing offset per weapon: mean hit − target over N shots, refused if too few shots or the group is too scattered; the old offset stays until a new one succeeds |
| `linalg` | Linear solve and symmetric eigenproblem (Jacobi) for the rim fit, the DLT and the calibration |
| `timesync` | SYNC → offset and drift (the weapon firmware's design), outlier gating, its inverse, and unwrapping of 32-bit lane stamps |
| `proto` | HIT, AIM, TRK_STATUS encoders; SYNC, SHOT_EVENT, SHOT_DETAIL decoders (§7) |

Python reference (`tools/reference/`): the same steps in NumPy, plus OpenCV's `findHomography` and `calibrateCamera` as independent checks for `geom` and `calib`.

---

## 6. Algorithms

### 6.1 Spot detection
1. Background b(x, y) on a 1/4-scale grid, updated with a slow running mean, frozen where a spot was found.
2. Noise σ from the median absolute deviation of (frame − background), per frame.
3. Threshold T = b + max(k·σ, T_min), k = 6 by default.
4. Runs of pixels above T per row; runs touching runs in the previous row join a blob (union-find).
5. Reject blobs that are too small (< 3 px), too large (reflections, a window), too elongated, or too weak (peak SNR below the limit).
6. A blob rejected as too large is searched for compact peaks (below).

**Implementation** (`aimtrk/src/detect.c`):
- The background uses 4 × 4 pixel cells in 16-bit fixed point (DN × 256, at most 65 280). It is updated by 1/16 per frame (time constant 160 ms) and frozen in the cells under a spot plus one cell around it.
- The noise comes from the median absolute deviation of every 4th pixel of every 4th row, read from a histogram.
- One pass over the frame does the threshold, the cell sums and the runs. Each blob records its first and last run, so its centroid window and the peak search of step 6 read only its own runs.
- Defaults: k = 6, T_min = 8 DN, area 3–300 px, extent ≤ 24 px, elongation ≤ 3, peak SNR ≥ 8; tables of 8192 runs and 1024 blobs.
- Memory for 1280 × 800 is 488 kB, fixed at start-up.
- On 40 simulated frames, the C code agrees with an independent NumPy/SciPy version (`scipy.ndimage.label` for the blobs) to 2e-4 px.

**Found by testing** (C and NumPy changed together each time; the cross-check still passes):

| Found by | Problem | Fix |
|---|---|---|
| IT06 (M3) | A person walking through left 4083 false spots in 299 frames: the background held their dark shape, and the returning light showed as spots | A cell whose mean moves by more than half the detection threshold takes the new value at once (cells under a spot stay frozen) |
| IT06 (M3) | A 4 × 4 cell across a sharp edge averages its bright and dark halves; the bright half's noise crossed the threshold as 3-px spots | Each cell's threshold uses the highest background of the cell and its 8 neighbours (+0.3 DN on a flat background), as a separable 3 + 3 maximum to stay inside 3 ms. Result: 33 false spots in the same frames |
| IT09 (M4) | A spot on the edge of a moving bright shape merged with it into a blob too large to keep | Step 6: the large blob's brightest pixels (up to 4) are measured against the brightest pixel on a ring 6 px out; a spot-sized peak (σ_major ≤ 2.5 px) at least 2 thresholds above it is kept, flagged `AT_SPOT_ON_CHANGE`. (Measuring against the blob's median made 18 false spots at 30 DN.) |
| Demo, 8 DN (M4) | A silhouette step about one threshold high broke up in the noise into spot-sized fragments: 6499 strays in 1060 frames, 4 of 12 shots lost | A spot must stand out from its own surroundings: the brightest ring pixel 6 px out is at most half its peak, or min_snr sigmas below it |
| IT06, 8 DN (M4) | Fragments filled the 256-place blob table in raster order and pushed out the real spots (12 of 57 shots lost) | Areas are summed at each root first; blobs under the minimum area take no place; table 1024 |

Result (IT06, 800 frames per level): 57 of 57 visible shots at 4, 8, 12 and 30 DN, no false HIT; stray spots per 1000 frames at 12 DN from 634 to 3.8. At 30 DN the same fixes recovered 7 shots the silhouette had swallowed.

### 6.2 Centroid
- Weighted centroid over the blob's pixels plus a one-pixel border, weights = frame − background, clipped at 0.
- Saturated spots: the plateau's centroid is biased by clipping. Flag the spot, and locate it from the unsaturated rim.
- Expected noise: for a Gaussian spot the centroid error falls roughly as σ_PSF / SNR.

**Measured** (`tools/reference/centroid_curve.py`, `results/centroid_curve.json`; 400 spots per point at random sub-pixel positions, full sensor model):

| Spot σ_PSF | RMS error at the nominal spot (peak 150 DN) | Weakest spot found every time | Error below 0.1 px from | Worst bias without noise |
|---|---|---|---|---|
| 0.8 px | 0.020 px | 25 DN | SNR 35 | 0.015 px |
| **1.2 px** | **0.018 px** | **16 DN** | **SNR 23 (16 DN, 0.093 px)** | **0.034 px** |
| 1.6 px | 0.016 px | 12 DN | SNR 24 | 0.023 px |
| 2.0 px | 0.015 px | 12 DN | SNR 25 | 0.038 px |

- **Spot size:** σ_PSF ≈ 1.2 px is enough, so no defocus is needed; larger spots gain little.
- **Signal margin:** the nominal spot has about 9 times the signal of the weakest usable one (150 against 16 DN). That leaves room for emitter ageing, off-axis loss and longer ranges.
- **Bias without noise:** up to 0.034 px at σ 1.2 px, depending on where the spot sits within its pixel. That is larger than the 0.018 px RMS noise at the nominal spot; it comes mostly from the mask (the blob plus a one-pixel border), which cuts the spot's tails. On a whole window without a mask the bias is 0.010 px. At about 2.5 mm/px, 0.034 px is 0.09 mm, small against P01, so the one-pixel border is kept.
- **Saturation:** not a problem. Up to 24 times full scale (peak 6000 DN), the plain centroid stays at 0.03 px or better and the rim fit at 0.009 px or better. The rim fit is kept because it improves as the spot gets brighter.

### 6.3 Geometry
- Distortion: Brown–Conrady with k1, k2, p1, p2 about the principal point.
- Screen mapping: undistorted image point → homography H → screen (x, y) in mm, origin at the screen centre, y up.
- **Undistortion:** Newton iteration with the analytic Jacobian, to 1e-12 (usually 3–4 iterations). A root where the Jacobian is not positive definite is refused, because a polynomial distortion folds over far out, and a root there is not the physical inverse.
- **Homography:** normalised DLT (Hartley), with the smallest singular vector taken from a Jacobi eigen-decomposition of AᵀA. A point set without a unique solution (collinear points, fewer than four) is refused.
- **Against OpenCV** (`tools/reference/geom_check.py`, in the CI cross-check): distortion matches `projectPoints` to 1e-16, undistortion matches `undistortPoints` to 1e-12. On exact data, the C homography reproduces the true one to 5e-13 px; OpenCV's `findHomography` is 3e-5 px off (its refinement stops earlier).

### 6.4 Calibration
- The projector shows a 9 × 6 dot grid; the camera captures it with a long exposure. Projectors leak some light at 850 nm; in the simulator the leak is a parameter, and the real fallback is IR LED markers at known screen points.
- Fit: normalised DLT for H, then Levenberg–Marquardt on H and the distortion together, minimising the reprojection error in pixels. Every fifth grid point is held out of the fit and used to check it.
- Accept only if P06 holds; otherwise keep the previous calibration and raise a fault bit.

**Implementation** (`aimtrk/src/calib.c`, `tools/calib/calibrate.py`):
- **Single view.** One plane cannot separate focal length and principal point from the pose, so they stay at nominal values; the homography (8 DOF) absorbs their error and k1, k2, p1, p2 are fitted with it (Levenberg–Marquardt, 12 parameters, normalised screen coordinates). With nominal values deliberately off (1590 against 1600 px, principal point 4.3 px off) the noise-free residual is 0.0003 px (unit test).
- **Convergence.** Accepted when a step no longer lowers the cost by a relative 1e-12, or when no step lowers it and the gradient vanishes (every Jacobian column orthogonal to the residuals to 1e-6, MINPACK's test); otherwise refused. A NaN in the data is refused, never stored.
- **Ordering.** The four extreme dots are the grid's corners (camera roughly upright); a homography through them predicts every grid point, each takes the nearest dot within 30 px, and a second pass repeats it with all matches. Shuffled and spurious dots give the same result (unit test).
- **Refusals.** A missing dot (by default all 54 must be found, `AT_CAL_FIND_ALL`), too few dots, or a residual over P06 refuse the fit and keep the previous calibration. IT01 refuses a grid partly hidden by a person and one displaced by 8 mm (3 px residual).
- **calib.json** holds the geometry, the fit report and the grid with a version and CRC-32s; a changed byte or wrong version is refused; `aim_host` checks `geometry_crc32`.
- **Against OpenCV** (`calibrateCamera`, true intrinsics for both): 1e-5 px on noise-free dots; 0.003 px and 0.008 mm on the screen with 0.02 px dot noise.
- **k2 on its own is not trustworthy:** over 20 draws of 0.02 px dot noise it ranged from 0.0003 to 0.0195 around the true 0.01, while the mapping stayed within 0.073 mm inside the grid (`results/crosscheck.json`). Only the mapping is used.
- **Error grows outside the grid:** with 0.05 px dot noise, 0.75 mm at the screen corners (95–150 mm outside the grid) against 0.13 mm inside (unit test `calib.test_fit_with_noise`). The grid should reach as close to the screen edges as the projector allows.

### 6.5 Strobing on lane time (weapon firmware v0.6)
A 0.5 ms emitter pulse fired at a random moment overlaps a 1 ms exposure only 15 % of the time at 100 fps, and lies fully inside it 5 % of the time. So the weapon schedules its pulses into the exposure windows, which it knows from lane time (the firmware's F12):
- **Aim pulses:** weapon slot s pulses in the middle of exposure k when `k mod N_slots = s`. With 2 weapons each has a 50 Hz aim trace; with 4, 25 Hz.
- **Shot pulse:** at the trigger break the weapon pulses in the middle of the first exposure whose pulse starts at least **2 ms** after the break, whatever its slot.
  - The shot pulse replaces the weapon's aim pulse in that frame only; its aim pulses carry on, so the follow-through after the shot is traced (F12).
  - That adds at most 12 ms to the optical latency. The recoil valve still fires at once.
  - The margin was 0.5 ms at first (M3 change). The weapon firmware only sees a break up to 1 ms after it happens and fires within 1.5 ms (its T01), so it would have missed its own frame.
  - Both sides use the same rule: `at_match_shot_frame` (aimtrk) and `strobe_shot_frame` (the firmware).
- **Who fired:** the SHOT_EVENT carries the break time t on the weapon's lane time. The shot frame is the first exposure whose pulse starts at least 2 ms after t. In that frame the shooter's spot is the one nearest the shooter's predicted aim position; another weapon's aim strobe in the same frame belongs to that weapon's slot and is near that weapon's track.
- **Two shots in one frame:** both shooters are known from their SHOT_EVENTs; the two spots are assigned by distance to the two predicted positions. If the predictions are closer than the gate allows, the HITs are sent as AMBIGUOUS (P04 counts these).
- **At least two slots:** the background is not updated under a spot (§6.1). If a weapon flashed in every frame, the background at a steady aim point would never be updated, and it would go wrong there when the projected scene changes. So N_slots ≥ 2 even with one weapon: a single weapon flashes in every other frame, and the frames in between update the background under its aim point (unit test `detect.test_background_tracks_and_freezes`).

**Implementation** (`aimtrk/src/match.c`):
- **Settling.** A frame is settled 5 ms after its exposure ends, once its SHOT_EVENTs have arrived (the weapon sends them within 2 ms of the break).
  - A frame not yet processed is waited for, until a newer frame arrives (frames come in order, so it was dropped) or 30 ms have passed.
  - A SHOT_EVENT that arrives after its frame was settled is matched against the stored frame (the last 32 frames are kept).
  - At most 16 shots wait for their frame. A SHOT_EVENT beyond that is lost: it is counted (`shots_lost`), logged, and flagged in TRK_STATUS (fault bit CAN_LOSS).
- **Assignment** is exhaustive over at most 8 emitters and 8 spots.
  - A spot further than 150 mm from a weapon's prediction is not that weapon's.
  - An assignment is ambiguous when swapping two emitters' spots costs less than 20 mm more than the best assignment.
- **Results:** every SHOT_EVENT gives a HIT, with a reason when there is no usable position. Every aim pulse with a spot gives an AIM. Positions are corrected by the weapon's zeroing; the tracks run on the raw positions.
- **Timing margin.** The pulse sits in the middle of the exposure, so the weapon's pulse may start up to ±250 µs off its plan before the pulse leaves the exposure. The weapon firmware is meant to keep its pulses within ±100 µs (the same figure as P07). **Measured in IT10 with firmware v0.6.2: starts −49 to +140 µs from the plan (v0.6.1: −58 to +170 µs), so that is not met;** every pulse was still fully inside its exposure, so no light was lost (§12).

### 6.6 Tracks
- α-β filter per weapon on screen coordinates, updated in that weapon's slots.
- A track is lost after 0.5 s without a spot (weapon lowered or off-screen) and restarts on the next spot in its slot. An update that is not later than the last one is ignored and counted, not taken as a restart.
- Prediction gives the expected position at any lane time, for gating and assignment.
- Values (`aimtrk/src/track.c`): α = 0.7, β = 0.3; a spot more than the gate (150 mm) from the prediction is not taken. So a weapon that swings more than about 150 mm between its slot frames (more than 7.5 m/s on the screen with two slots) loses its track for 0.5 s.

**Found by IT09 (M4)** (`aimtrk/src/match.c`):
- **A stray could take over a track.** In the first IT09 run, 24 % of the shots were missed and 13 were false: when a weapon's real spot was lost (on the saturated window), its track started on a stray that came back every 48 frames, and held it.
- **Starting a track:**
  - the first spot must have a peak SNR of at least 40 (`min_init_snr`, a quarter of the nominal spot; the edges of scene changes reach about 20);
  - that spot is only a candidate. The track starts when the weapon's next slot frame has a spot within the gate around it, as a real emitter does in every slot frame and a stray does not. AIM frames start with the confirmed track.
  - A candidate not seen again within two slot cycles is forgotten; a stale one is replaced by the strongest free spot.
  - A shot with a candidate but no track takes only a spot consistent with the candidate, else NO_SPOT.
- **Letting go:** a track whose weapon's spot is missing for 3 slot frames in a row, while another free spot of at least `min_init_snr` is there, restarts (`reacquired` in the summary).
- **A shot with neither a track nor a candidate** (the weapon's aim pulses were not seen) has no position to compare with. It used to take any spot of at least `min_init_snr` in its frame, the strongest first, so with its own flash hidden it could report a stray as an OK hit (found in review). It now takes a spot only if exactly one strong spot in the frame is unexplained: not within the gate of another emitter's prediction, and not already there in either of the two frames before (a glint, a hot pixel, a recurring stray). Two or more: AMBIGUOUS; none: NO_SPOT. A one-off stray in exactly that frame, with the weapon's own flash hidden, would still be taken. The results of IT01–IT11 did not change with it; the `aim_host` summary counts such shots (`untracked_shots`: 39 of the 1994 in IT09).
- Unit tests: `test_min_init_snr`, `test_reacquire`, `test_unconfirmed_stray`, `test_untracked_shot`.

### 6.7 Lane time on the tracker
- SYNC every 10 ms carries the lane time; the tracker stamps its reception on its own clock and fits offset and drift by least squares over the last 32 SYNCs (`aimtrk/src/timesync.c`). A frame's exposure start (camera timestamp, tracker clock) is turned into lane time with that fit, which gives its exposure number.
- **Reception stamps.** On SocketCAN the stamp is the kernel's reception time (`SO_TIMESTAMPNS`), moved from `CLOCK_REALTIME` to `CLOCK_MONOTONIC`, not the time the loop gets round to reading the frame. The loop may be busy reading or processing a 1 MB frame for milliseconds: in IT11 (vcan) frames waited up to 1.1–4.6 ms between reception and reading, which a stamp taken on reading would have added to SYNC (found in review). In the file transport the stamp is in the log.
- **Outliers.** A SYNC more than 200 µs from the current fit is held out. Only when 4 in a row disagree with the fit and agree with each other to 200 µs has the lane time stepped (a lane restart, a long gap); then the fit restarts on them. Before, a single late SYNC was taken into the fit: one 3 ms late moves the 32-point line by about 0.1 ms at its centre and 0.36 ms at its newest end, where the next frames are stamped; one more than 10 ms late restarted the fit on itself. Unit tests: `test_late_sync_rejected`, `test_step_accepted_after_n`.

---

## 7. CAN protocol (extends the weapon firmware's `trainer.dbc`)

Same bus, ID layout `function (4 bits) | node ID (7 bits)`. The weapon firmware uses functions 0x1–0x9. The tracker adds:

| Frame | Function | Dir | Rate | Signals |
|---|---|---|---|---|
| HIT | 0xA | tracker → lane | per SHOT_EVENT | node ID of the shooter (in the ID), shot_no (u16), x_mm and y_mm (14 bits each, 0.25 mm steps, ±2048 mm), result (4 bits: OK, NO_SPOT, OFF_SCREEN, AMBIGUOUS, SATURATED, DROPPED), quality (u8, the peak SNR), latency (u8, 0.1 ms, saturating) |
| AIM | 0xB | tracker → lane | per slot frame | node ID (in the ID), frame_seq (u16, exposure number mod 65536), x_mm and y_mm (14 bits, 0.25 mm); 6 bytes |
| TRK_STATUS | 0xC | tracker → lane | 1 Hz | fps_x10 (u16, factor 0.1: decoded in fps), dropped (u8), worst processing 0.1 ms (u8), ambient (u8), calibration age h (u8), faults (u8: NO_CALIBRATION, CALIBRATION_OLD, FRAMES_DROPPED, DETECT_OVERFLOW, SLOW, NO_SYNC, CAN_LOSS) |

The tracker uses node ID 0x7E. A unit test checks every frame against `protocol/tracker.dbc`: bytes generated with `cantools` from `protocol/vectors.json`, and `gen_vectors.py --check` in CI.
- **Positions** are 14-bit with 0.25 mm steps, not 16-bit whole millimetres. A whole millimetre would throw away most of the measured accuracy (0.04 mm), and 14 bits leave room for the result field in 8 bytes.
- **Bus load** (IT05): 311 frames/s and 7.6 % of 500 kbit/s in the worst case, counting standard frames with worst-case bit stuffing. The worst case is two weapons at 900 rpm (SHOT_EVENT and SHOT_DETAIL per shot, STATUS at 10 Hz each), SYNC at 100 Hz, a HIT per shot, AIM at 100 Hz and TRK_STATUS at 1 Hz. The tracker's own frames in IT05 were 2.5 %.
- **Lost frames.** SocketCAN sends are non-blocking: a full transmit queue drops the frame. Dropped frames are counted (`can_tx_dropped`) and flagged with CAN_LOSS in the next TRK_STATUS.

---

## 8. Camera simulator (`sim/camsim.py`, `sim/scenario.py`)

Renders 1280 × 800 8-bit frames, streamed into a pipe with a small header (frame index, exposure start on the tracker's clock):
- **Geometry:** a pinhole camera with pose and focal length from §2a, plus the distortion model, looking at the screen plane.
- **Spots:** Gaussian PSF per emitter pulse, energy from the pulse's overlap with the exposure, smeared along the aim motion during the pulse.
- **Sensor:** shot noise, read noise, fixed-pattern noise, hot pixels, 8-bit quantisation and saturation.
- **Ground truth:** for every pulse, the true screen position and which weapon fired it, in a JSON file.

**Scenarios** (`sim/scenario.py`). A scenario describes the weapons: slots, aim paths, breaks, misalignment, raised, covered, brightness. It is turned into what `aim_host` sees:
- camera frames stamped on the tracker's own clock (offset, +40 ppm drift, 5 µs timestamp jitter);
- the CAN traffic as the tracker receives it: SYNC every 10 ms with up to 20 µs reception delay, and the weapon's SHOT_EVENT and SHOT_DETAIL per shot;
- ground truth for every pulse.
- The weapons follow the firmware's v0.6 rule. They place pulses on their own estimate of lane time, which is off by a Gaussian error of σ = 20 µs, and their shot frame comes from the same estimate they report in SHOT_EVENT.

**Region renderer.** Full frames take 26–48 ms in NumPy (§12), too slow for 100 000 frames. So 48 noisy background frames (with the interference) are rendered once; each frame is one of them, with the spots rendered over it in 25 × 25 pixel patches with their own noise. A frame takes about 1 ms. The background noise repeats every 48 frames (0.48 s); the spots' noise does not. Frames go to `aim_host` through a pipe, so no frame file is written.

**Interference set** (P03), in image pixels:
- projector leak: a smooth scene with a target silhouette crossing it, ±10 % flicker;
- a sunlit window, saturating;
- three glints from a target frame;
- extra hot pixels (1e-4);
- a person crossing the lower part of the view in 2 s.

**What counts as visible.** A spot on the saturated window cannot be seen, and neither can one that straddles its edge. The truth marks a spot hidden when a saturated background pixel lies within 3 px of its centre (2.5 spot sigmas). Before that rule (M4), the 6 IT09 misses were all spots 0–3 px below the window's bottom edge, with half their light on saturated pixels; the tracker answered NO_SPOT, which is right.

**Co-simulation with the weapon firmware** (`tools/cosim/`), used by IT10:
- `s1_lane_driver.py` runs inside Renode on the firmware's `sim/lane.resc` (two weapons on one CAN hub). It sends SYNC every 10 ms and the camera-mode commands, loads both weapons and pulls the triggers. It records every emitter gate edge (the LED's state hook) and every CAN frame each weapon sends, on virtual time, with a 10 µs quantum (the default quantum made the edge times several hundred µs coarse).
- `cosim.py` turns the recording into the CAN log and the camera frames: each pulse the firmware actually gave is rendered where that weapon aims at that moment, with the part of it that falls inside the exposure. Then it runs `aim_host`.

**Not done:** aim motion from recorded IMU data; the tests use scripted aim paths (sway, flinch, sweeps).

---

## 9. Verification

### 9.1 Unit tests (ztest on `native_sim`, 63 tests)
| Suite | Covers |
|---|---|
| detect | Thresholds, runs, union-find across rows, rejection rules, background freeze under spots, noise estimate |
| centroid | Sub-pixel accuracy on rendered spots against SNR (the P01 curve), saturation bias and the rim method, border pixels |
| geom | Linear solve and eigen-decomposition; distortion round trip and the fold-over refusal; homography (DLT) on known point sets and degenerate ones; image ↔ screen round trip and off-screen handling |
| calib | Fit on synthetic grids, noise-free and with noise, with deliberately wrong nominal intrinsics; ordering with shuffled and spurious dots; missing dots; rejection of a bad fit (P06), the previous calibration kept; iteration limit; NaN dots refused |
| zero | Offset and its use; refusal of too few shots and of a scattered group, the old offset kept |
| timesync | Offset and drift, wrap, restart, drift bound; a late SYNC held out; a real step taken after 4 |
| track | Constant-velocity tracking, gating, loss and restart; updates out of time order ignored |
| match | Shot frame selection on lane time, slot assignment, two-shots-in-one-frame, AMBIGUOUS cases, untracked shots, lost shots counted |
| proto | Every frame against the DBC |

### 9.2 Reference cross-check (`tools/reference/`)
- C vs NumPy: centroids, distortion, homography, calibration fit on the same inputs.
- C vs OpenCV: homography against `findHomography`; calibration residuals against `calibrateCamera` on the same grid images.

### 9.3 Integration tests (`tests/integration`, pytest)
IT01, IT02 and IT08 drive the C library through ctypes on frames from the camera simulator. IT03–IT07 and IT09–IT11 run `aim_host`: `sim/scenario.py` streams each scenario into it through a pipe; its CAN input is a log (IT11: vcan), its CAN output is decoded with the DBC. Results are in `results/` and, requirement by requirement, in `docs/traceability.md`.

| Test | Covers | Sequence | Result |
|---|---|---|---|
| IT01 calibration | F05, P06 | Grid capture, fit, hold-out; bad grids | 54 of 54 dots; fit 0.023 px, held-out 0.023 px; mapping within 0.055 mm of the true camera; a displaced and a partly hidden grid refused |
| IT02 accuracy | F02–F04, P01 | 500 shots over the screen, steady aim | 500 of 500; 0.042 mm RMS, 0.082 mm at the 99th percentile, worst 0.099 mm |
| IT03 latency | F06, F08, P02, P07 | 100 shots at random break times | 100 OK from the right exposure; latency 7.8–17.7 ms (mean 12.9); the HIT's latency field within 0.1 ms of true lane time; frame times within 36 µs with the tracker clock drifting 40 ppm |
| IT04 aim trace | F07, F11, F12 | Shaky hold, flinch, 12 shots | 486 of 488 aim pulses gave an AIM (the first spot of a track is a candidate), 0.11 mm RMS; 12 records with SHOT_DETAIL merged, 73–74 trace points each; the 25 mm flinch shows as a 14 mm mean dip in the last 60 ms; TRK_STATUS at 1 Hz |
| IT05 two weapons | F09, P04 | Staggered, same frame 0.5 m apart, same frame 8 mm apart | 60 of 60, 100 of 100, 60 of 60 AMBIGUOUS; no wrong attribution |
| IT06 interference | F02, P03 | 800 frames per leak level (4, 8, 12, 30 DN) | 57 of 57 visible shots at every level, no false HIT; the 3 shots behind the person NO_SPOT |
| IT07 no-hit reasons | F10, F01 | Off-screen, out of view, covered, dropped frame, 400 × too bright; a frame of another size | OK, OFF_SCREEN with position, NO_SPOT, NO_SPOT, DROPPED (counted in TRK_STATUS), OK via the rim fit within 0.14 mm; the odd-sized frame skipped and counted |
| IT08 zeroing | F13 | Emitter 2 mrad right, 1 mrad down; 5 zeroing shots | 11.5 mm RMS before, 0.64 mm after (offset measured to 0.30 mm; the rest grows towards the edges, as the misalignment is an angle) |
| IT09 long run | P03, P05 | 100 000 frames, two weapons sweeping, 1994 shots, the interference set at 12 DN | 1953 of 1954 visible shots within 5 mm (0.16 mm RMS, worst 2.8 mm), 0 false HITs, 40 of 40 hidden shots NO_SPOT; the one miss is a spot 3–4 px below the saturated window (NO_SPOT); heap unchanged at 10 800 bytes; frame times within 44 µs; timing in §9.4 (`pytest -m long`) |
| IT10 weapon firmware | F06, F08, F17 (firmware), P02 | Firmware v0.6.2, two weapons in Renode, recorded in `tests/data/s1_v0.6.2_lane.txt` and replayed (`pytest -m cosim` records it again) | A pulse in every slot frame of its weapon and none in the other's, 153 per weapon; starts −49 to +140 µs from plan, 480–525 µs long, all inside the exposure; 8 of 8 SHOT_EVENTs gave OK HITs within 0.18 mm, 9.5–10.4 ms; 148 AIM frames per weapon |
| IT11 SocketCAN | F06, F10, P02, P07 | `aim_host` live on vcan: SYNC and a SHOT_EVENT in real time, frames as their exposures happen | OK HIT on the spot, 9.5 ms; every frame kernel-stamped, no SYNC held out; frame times within 19–23 µs; skipped without vcan |

Plots: `results/plots/it01_calibration.png`, `it02_accuracy.png`, `it03_latency.png`, `it04_aim_trace.png`.

### 9.4 Honest limits
- **The camera is simulated.** Accuracy is measured against the simulator's own truth. A real lens, sensor and screen bring effects the model leaves out or simplifies (screen texture, non-flat screens, lens flare, sensor non-linearity).
- **P05 (worst case ≤ 3 ms) is not demonstrated.** Processing time is measured on an x86 PC (WSL2) and CI runners, both virtual machines, not on a target board.
  - `aim_host` measures each frame's thread CPU time (`CLOCK_THREAD_CPUTIME_ID`) and its wall time. The CPU time leaves out other threads of the guest, but not the time the hypervisor takes the virtual CPU away while this thread runs (steal time). On WSL2 the CPU-time clock also reads about 8 % more than `CLOCK_MONOTONIC` on the same frames (IT09: 1.07 against 0.98 ms at the median, 2.90 against 2.66 ms at the 99.9th percentile), so it is not even a lower bound of the wall time there. The slowest frames are the same by either clock: ordinary frames (one spot, about 120 small blobs, as a typical frame) slowed by the host, up to 12–14 ms.
  - IT09 therefore asserts the 99.9th percentile of the CPU time and reports both clocks with their percentiles and slowest frames (`results/it09_long_run.json`). It was 2.4 ms in a run on a quiet PC, 2.9 ms in the recorded run, and 3.4–3.6 ms with other work on the same PC (for example a Renode run), over the limit.
  - Nothing locks memory or raises the scheduling priority by default. `aim_host --rt` does `mlockall` and `SCHED_FIFO` (best effort); the results in `results/` were measured without it.
- **Renode time is virtual**: the end-to-end test proves the timing logic and the matching, not silicon latency. Edge times in Renode come from virtual time with a 10 µs quantum, so the firmware's pulse timing is known to about 10 µs.
- **The weapon firmware's pulse timing is not within ±100 µs** (IT10 with v0.6.2: −49 to +140 µs, §6.5). It stays inside the ±250 µs margin, so no light was lost.
- **The SocketCAN path is tested on vcan only** (IT11), not on a real CAN controller with arbitration delays and bus errors; IT11 is skipped where the runner has no vcan module.
- **IT10's camera is still simulated.** The firmware decides when each pulse happens and what it sends on CAN; where the weapon aims is scripted.
- **One camera, one flat screen, up to four weapons.** The region renderer's background noise repeats every 48 frames (§8). The projector-leak levels are not measured data (§12).

---

## 10. Host tools and deliverables
- `aim_host` (`host/`, `make -C host`): one program with options. Transport: `--can-in/--can-out` log files, or `--can IFNAME` (SocketCAN, `--can-log` keeps a copy of what was sent). Zeroing is `--zero NODE:X:Y:N --zero-out FILE`; saved offsets are loaded with `--zero-in`. The status is the 1 Hz TRK_STATUS frame plus a JSON summary at exit (counters for dropped, lost and held-out frames included). `--rt` for `mlockall` and `SCHED_FIFO`. Calibration is the Python runner `tools/calib/calibrate.py`; `aim_host` reads its `calib.json` and checks `geometry_crc32`.
- `tools/lane/demo_session.py`: calibrates the lane, zeroes weapon 2 (emitter 2 mrad right, 1 mrad up), then runs a 10.6 s session: two shooters, two targets each, three shots per target with sway and flinch. Output in `docs/session/` (CAN logs, after-action records, summary, targets). It uses the camera simulator, not the firmware in Renode; the firmware's part end to end is IT10.
- `tools/lane/lane_view.py`: draws the session from the tracker's own CAN output and records only: the screen with targets, the aim of the last second, every hit with weapon.shot, the after-action panel of the latest shot, and the tracker status. `--png` for one moment, `--frames` for the GIF and video (`docs/img/`, made with ffmpeg).
- `docs/traceability.md`: every requirement to its tests and the measured result.

---

## 11. Milestones

| Milestone | Work | Done when | Result |
|---|---|---|---|
| M1 | Camera simulator (geometry, spots, sensor, ground truth); `detect` and `centroid` in C with the NumPy reference; centroid accuracy vs SNR | detect and centroid suites green; the centroid curve meets the P01 budget line | 21 unit tests; C matches NumPy on 40 frames; 0.018 px at the nominal spot |
| M2 | `geom` and `calib`; OpenCV cross-check; accuracy over the screen; zeroing | IT01, IT02 and IT08 pass; P01 shown on plots | 35 unit tests; P01 0.039 mm RMS |
| M3 | `timesync`, `track`, `match`; `aim_host` with the file transport and SocketCAN; DBC frames; firmware v0.6 emitter strobing with its own tests | IT03–IT07 pass; the firmware's tests still pass with v0.6 | 54 unit tests; 13 integration tests |
| M4 | IT09, IT10; lane view; demo, GIF and video; traceability; CI | All of the above green in CI | 58 unit tests; 16 integration tests plus IT09 and the IT10 recording; demo 12 of 12 hits |

After M4, a review led to the changes in `CHANGELOG.md` (Unreleased): kernel timestamps and outlier gating for SYNC (§6.7), IT11 on vcan, the untracked-shot rule (§6.6), counted CAN and shot losses, and others.

---

## 12. Open points and findings

- [x] **Simulator speed** (`camsim.py bench`, `results/camsim_bench.json`): a full frame takes 48 ms with Poisson shot noise, or 26 ms with the Gaussian approximation. The 100 000 frames of IT09 would take 43–80 minutes, and live 100 fps needs under 10 ms per frame, so IT09 and the demo use the region renderer (§8).
- [x] **Centroid accuracy against SNR and spot size:** σ_PSF ≈ 1.2 px is enough, giving 0.018 px RMS at the nominal spot (§6.2).
- [x] **Saturation:** no limit up to 24 times full scale; the rim fit stays within 0.009 px (§6.2).
- [x] **The weapon firmware's pulse timing** (IT10, `results/it10_s1_cosim.json`). With v0.6.0 the pulses lasted 563–610 µs, not 500 µs, and 8 of 306 ran up to 80 µs past the exposure (every shot still OK). The gate's off-timer was a relative kernel timeout, which Zephyr rounds up by one 100 µs tick; v0.6.1 takes both edges from the timer interrupt on absolute tick edges and stamps SYNC with the cycle counter. IT10 now sees 480–526 µs and no overrun. v0.6.2 stamps each SHOT_EVENT with the trigger sample that crossed the break (it was the time the control thread handled it, 38–57 µs later); IT10 recorded again with it: 480–525 µs, no overrun. **Still open:** starts of −49 to +140 µs, not within ±100 µs (§6.5); in some runs one weapon has a stretch of pulses 100–200 µs late, most likely SYNC delivery jitter between the two Renode machines (the firmware's single-machine test sees ≤ 29 µs).
- [x] **Frame transport:** a pipe carries 1270 MB/s (a Python writer sending 1 MB frames into `cat`), 12 times the 100 MB/s needed, so shared memory is not needed.
- [x] **Projector leak at 850 nm:** no public data was found for the 850 nm leak of a lane projector in a 1 ms exposure, so IT06 brackets it at 4, 8, 12 and 30 DN, against an 8 DN detection floor and a 150 DN nominal spot.
  - M3: P03 was met up to 12 DN; at 30 DN a moving bright silhouette swallowed 12 % of the spots, without any false HIT.
  - M4: after the detector changes (§6.1), P03 is met at all four levels. The 30 DN losses were a full blob table, not a limit of the method. 8 DN was added because the demo found it: a step about one detection threshold high is the hard case, not the brightest one.
  - The leak still has to be measured on site; the calibration capture shows it, as the ambient level in TRK_STATUS.
- [x] **SYNC stamped in user space** (review, §6.7): the live loop read SYNC up to 4.6 ms after it arrived (IT11). Kernel timestamps and outlier gating fixed it; frame times from SYNC are within about 20 µs on vcan.

---

## 13. Revision history

| Rev | Milestone | Change |
|---|---|---|
| A | — | First issue: requirements, design, test plan |
| B | M1 | Reference lane checked, centroid curve, at least two time slots |
| C | M2 | Calibration and geometry results; separate `zero` module |
| D | M3 | Matching, `aim_host`, CAN frames; break-to-pulse margin 2 ms; background copes with scene changes |
| E | M4 | IT09, IT10, lane view, demo; detector and track changes found by IT09 and the demo |
| F | M4 | Weapon firmware v0.6.1 fixes its pulse length; IT10 recorded again and checks pulse width and overrun |
| G | after M4 | Review: kernel timestamps and outlier gating for SYNC, IT11 on vcan, untracked shots, counted losses, frame-size check; weapon pulse timing stated as not within ±100 µs; P05 stated as not demonstrated |
