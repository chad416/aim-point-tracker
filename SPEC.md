# S3 — Camera Aim-Point Tracker

Specification, revision E (8 Nov 2026): week 4 done.

- **Revision B** (7 Nov) added the week 1 results (§2a, §3.3, §6.1, §6.2, §12) and the minimum of two time slots (§6.5).
- **Revision C** (8 Nov) added the week 2 results (§3.3, §6.3, §6.4, §9.3) and a separate `zero` module (§5).
- **Revision D** (8 Nov) adds the week 3 results (§6.1, §6.5, §7, §8, §9.3, §12) and three changes found while testing:
  - the break-to-shot-pulse margin is 2 ms, not 0.5 ms (§6.5);
  - the shot pulse no longer skips the next aim pulse (§6.5);
  - the background now copes with sharp scene changes (§6.1).
- **Revision E** (8 Nov) adds the week 4 results (§6.1, §6.6, §8, §9.3, §9.4, §10, §12): IT09, IT10 with S1's firmware, the lane view and the demo. Changes found while testing:
  - spots inside a scene change are found again by a search for compact peaks (§6.1);
  - a spot must stand out from its own surroundings, and blobs too small to be a spot no longer take a place in the blob table (§6.1);
  - a new track starts only when its first spot is strong and is seen again in the next slot frame, and a track held by a wrong spot lets go after three slot frames (§6.6);
  - S1's pulses are 0.57–0.61 ms long, not 0.5 ms (§6.5, §12).

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
| Calibration residual | 0.3 px (0.75 mm) | 0.023 px fit, 0.023 px held-out (IT01); the mapping is within 0.055 mm of the true camera over the whole screen |
| Hit accuracy, everything together (P01) | ≤ 2 mm RMS, ≤ 5 mm at the 99th percentile | 0.042 mm RMS and 0.082 mm at the 99th percentile over 500 shots (IT02, re-run in week 3) |

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
| `timesync` | SYNC → offset and drift (shared design with S1), its inverse, and unwrapping of 32-bit lane stamps |
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

**Week 1 implementation** (`aimtrk/src/detect.c`):
- The background uses 4 × 4 pixel cells in 16-bit fixed point (DN × 256, at most 65 280). It is updated by 1/16 per frame (time constant 160 ms) and frozen in the cells under a spot plus one cell around it.
- The noise comes from the median absolute deviation of every 4th pixel of every 4th row, read from a histogram.
- One pass over the frame does the threshold, the cell sums and the runs.
- Defaults: k = 6, T_min = 8 DN, area 3–300 px, extent ≤ 24 px, elongation ≤ 3, peak SNR ≥ 8.
- Memory for 1280 × 800 is 475 kB, fixed at start-up.
- On 40 simulated frames, the C code agrees with an independent NumPy/SciPy version (`scipy.ndimage.label` for the blobs) to 2e-4 px.
- It takes 1.1 ms per frame on average and 1.9 ms at worst on the development PC (WSL2), against the 3 ms of P05.

**Week 3 changes** (found by IT06, the interference set; C and NumPy changed together, cross-check still 0 mismatches):
- **A person walking through the view left thousands of false spots.** Diagnostic: 4083 spots in 299 frames with only the passer-by.
  - Behind the person, the background still held their dark shape for many frames, so the ambient light coming back showed up as spots.
  - Fix: a cell whose mean moves by more than half the detection threshold takes the new value at once, instead of 1/16 per frame. Cells under a spot stay frozen.
- **Sharp edges made noise spots.** A 4 × 4 cell across a sharp edge (the person's outline, a window frame) averages its bright and dark halves. Against that mean, the noise of the bright half crossed the threshold as 3-pixel spots.
  - Fix: each cell's threshold uses the highest background of the cell and its 8 neighbours. On a flat background that adds about 0.3 DN.
  - It is computed as two passes of 3 values (row, then column): the first version, 9 comparisons per cell, took processing over 3 ms.
- **Result:**
  - 33 false spots in the same 299 frames, all where the person leaves the image.
  - The week 1 figures are unchanged at the precision quoted (0.018 px at the nominal spot, every spot found from 16 DN up).

**Week 4 changes** (found by IT09 and the demo session; C and NumPy changed together, the cross-check still passes):
- **Step 6: spots inside a scene change.** A spot that lands on the edge of a moving bright shape merges with it into a blob that is too large, and was lost. Such a blob is now searched for compact peaks:
  - its brightest pixels are taken in turn (up to 4), each measured against the brightest pixel on a ring 6 px out;
  - a peak at least 2 detection thresholds above its ring, spot-sized (σ_major ≤ 2.5 px), is located against the ring level and flagged `AT_SPOT_ON_CHANGE`.
  - A first version measured peaks against the blob's median level. It made false spots on the bright side of the change (18 at 30 DN in IT06), so it was replaced.
- **A spot must stand out from its own surroundings.** The demo session, at 8 DN of projector leak, found 6499 stray spots in 1060 frames and lost 4 of 12 shots:
  - The moving silhouette's step was about one detection threshold high (T_min = 8 DN). Noise broke it into hundreds of spot-sized fragments, and those passed every rule. Each one also froze the background under it, which kept them coming.
  - Fix: the brightest pixel on the ring 6 px out must be at most half the spot's peak, or at least min_snr noise sigmas below it (a side cut by the image border has no ring). Fragments of a step fail; a real spot on a flat background, or on top of the step, passes.
- **Blobs too small to be a spot took places in the blob table.** With the fix above, IT06 at 8 DN still lost 12 of 57 shots: the 256-place table filled with up to 340 fragments per frame, in raster order, and the spots after them were dropped.
  - Fix: areas are summed at each blob's root first (a blob's root is its first run, so the order is still raster order). Blobs under the minimum area are counted and never take a place.
  - The table is now 1024 places: the frame where the simulated silhouette jumps back across the screen leaves about 250 fragments of 3 px or more. The detector's memory grows by 6 kB.
- **Result** (IT06, 800 frames per level): 57 of 57 visible shots at 4, 8, 12 and 30 DN, no false HIT. Stray spots per 1000 frames at 12 DN: from 634 to 3.8. At 30 DN the same fixes recovered the 7 shots the silhouette had swallowed: the blob table was full there too.

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
- **Shot pulse:** at the trigger break the weapon pulses in the middle of the first exposure whose pulse starts at least **2 ms** after the break, whatever its slot.
  - The shot pulse replaces the weapon's aim pulse in that frame only; its aim pulses carry on, so the follow-through after the shot is traced (F12).
  - That adds at most 12 ms to the optical latency. The recoil valve still fires at once (S1 T02 unchanged).
  - **Week 3 changes from revision C:**
    - The margin was 0.5 ms. The weapon firmware only sees a break up to 1 ms after it happens and fires within 1.5 ms (S1 T01), so it would have missed its own frame.
    - The pulse also no longer skips the next aim pulse, which had no purpose once a weapon flashes at most once per frame.
  - Both sides use the same rule: `at_match_shot_frame` (aimtrk) and `strobe_shot_frame` (S1 v0.6).
- **Who fired:** the SHOT_EVENT carries the break time t on the weapon's lane time. The shot frame is the first exposure whose pulse starts at least 2 ms after t. In that frame the shooter's spot is the one nearest the shooter's predicted aim position; another weapon's aim strobe in the same frame belongs to that weapon's slot and is near that weapon's track.
- **Two shots in one frame:** both shooters are known from their SHOT_EVENTs; the two spots are assigned by distance to the two predicted positions. If the predictions are closer than the gate allows, the HITs are sent as AMBIGUOUS (P04 counts these).

**Week 3 implementation** (`aimtrk/src/match.c`):
- **Settling.** A frame is settled 5 ms after its exposure ends, once its SHOT_EVENTs have arrived (S1 sends them within 2 ms of the break).
  - A frame not yet processed is waited for, until a newer frame arrives (frames come in order, so it was dropped) or 30 ms have passed.
  - A SHOT_EVENT that arrives after its frame was settled is matched against the stored frame (the last 32 frames are kept).
- **Assignment** is exhaustive over at most 8 emitters and 8 spots.
  - A spot further than 150 mm from a weapon's prediction is not that weapon's.
  - An assignment is ambiguous when swapping two emitters' spots costs less than 20 mm more than the best assignment.
- **Results:** every SHOT_EVENT gives a HIT, with a reason when there is no usable position. Every aim pulse with a spot gives an AIM. Positions are corrected by the weapon's zeroing; the tracks run on the raw positions.
- Timing margin: the pulse sits in the middle of the exposure, so the weapon's lane-time estimate may be off by ±250 µs before the pulse leaves the exposure. S1's time sync must stay inside ±100 µs (P07, and a test on the S1 side).
  - **Measured with S1's firmware (IT10, week 4):** pulse starts −30 to +236 µs from the plan, and pulses 563–610 µs long instead of 500 µs. Together those put the end of 8 of 306 pulses up to 80 µs past the exposure (§12).
- **At least two slots** (week 1): the background is not updated under a spot (§6.1). If a weapon flashed in every frame, the background at a steady aim point would never be updated, and it would go wrong there when the projected scene changes. So N_slots ≥ 2 even with one weapon: a single weapon flashes in every other frame, and the frames in between update the background under its aim point (unit test `detect.test_background_tracks_and_freezes`).

### 6.6 Tracks
- α-β filter per weapon on screen coordinates, updated in that weapon's slots.
- A track is lost after 0.5 s without a spot (weapon lowered or off-screen) and restarts on the next spot in its slot.
- Prediction gives the expected position at any lane time, for gating and assignment.
- Week 3 values (`aimtrk/src/track.c`): α = 0.7, β = 0.3; a spot more than the gate (150 mm) from the prediction is not taken. So a weapon that swings more than about 150 mm between its slot frames (more than 7.5 m/s on the screen with two slots) loses its track for 0.5 s.

**Week 4 changes** (`aimtrk/src/match.c`, found by IT09):
- **A stray could take over a track.** In the first IT09 run, 24 % of the shots were missed and 13 were false: when a weapon's real spot was lost (on the saturated window), its track started on a stray that came back every 48 frames, and held it.
- **Starting a track:**
  - the first spot must have a peak SNR of at least 40 (`min_init_snr`, a quarter of the nominal spot; the edges of scene changes reach about 20);
  - that spot is only a candidate. The track starts when the weapon's next slot frame has a spot within the gate around it, as a real emitter does in every slot frame and a stray does not. AIM frames start with the confirmed track.
  - A candidate not seen again within two slot cycles is forgotten; a stale one is replaced by the strongest free spot.
  - A shot with no track takes only a spot consistent with the candidate, else NO_SPOT.
- **Letting go:** a track whose weapon's spot is missing for 3 slot frames in a row, while another free spot of at least `min_init_snr` is there, restarts (`reacquired` in the summary).
- Unit tests: `test_min_init_snr`, `test_reacquire`, `test_unconfirmed_stray`.

---

## 7. CAN protocol (extends S1's `trainer.dbc`)

Same bus, ID layout `function (4 bits) | node ID (7 bits)`. S1 uses functions 0x1–0x9. The tracker adds:

| Frame | Function | Dir | Rate | Signals |
|---|---|---|---|---|
| HIT | 0xA | tracker → lane | per SHOT_EVENT | node ID of the shooter (in the ID), shot_no (u16), x_mm and y_mm (14 bits each, 0.25 mm steps, ±2048 mm), result (4 bits: OK, NO_SPOT, OFF_SCREEN, AMBIGUOUS, SATURATED, DROPPED), quality (u8, the peak SNR), latency (u8, 0.1 ms, saturating) |
| AIM | 0xB | tracker → lane | per slot frame | node ID (in the ID), frame_seq (u16, exposure number mod 65536), x_mm and y_mm (14 bits, 0.25 mm); 6 bytes |
| TRK_STATUS | 0xC | tracker → lane | 1 Hz | fps ×10 (u16), dropped (u8), worst processing 0.1 ms (u8), ambient (u8), calibration age h (u8), faults (u8) |

The tracker uses node ID 0x7E. As in S1, a unit test checks every frame against `protocol/tracker.dbc`: bytes generated with `cantools` from `protocol/vectors.json`, and `gen_vectors.py --check` in CI.

**Week 3:**
- **Positions** are 14-bit with 0.25 mm steps, not 16-bit whole millimetres. A whole millimetre would throw away most of the measured accuracy (0.04 mm), and 14 bits leave room for the result field in 8 bytes.
- **Bus load** (IT05): 311 frames/s and 7.6 % of 500 kbit/s in the worst case, counting standard frames with worst-case bit stuffing. The worst case is two weapons at 900 rpm (SHOT_EVENT and SHOT_DETAIL per shot, STATUS at 10 Hz each), SYNC at 100 Hz, a HIT per shot, AIM at 100 Hz and TRK_STATUS at 1 Hz. The tracker's own frames in IT05 were 2.5 %.

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

**Week 3** (`sim/scenario.py`):
- **Scenarios.** A scenario describes the weapons: slots, aim paths, breaks, misalignment, raised, covered, brightness. It is turned into what `aim_host` sees:
  - camera frames stamped on the tracker's own clock (offset, +40 ppm drift, 5 µs timestamp jitter);
  - the CAN traffic as the tracker receives it: SYNC every 10 ms with up to 20 µs reception delay, and S1's SHOT_EVENT and SHOT_DETAIL per shot;
  - ground truth for every pulse.
- **The weapons follow S1 v0.6.** They place pulses on their own lane-time estimate (20 µs error), and their shot frame comes from the same estimate they report in SHOT_EVENT.
- **Region renderer.**
  - 48 noisy background frames (with the interference) are rendered once. Each frame is one of them, with the spots rendered over it in 25 × 25 pixel patches with their own noise.
  - A frame takes about 1 ms instead of 26–48 ms. The background noise repeats every 48 frames (0.48 s); the spots' noise does not.
  - Frames go to `aim_host` through a pipe, so no frame file is written.
- **Interference set**, in image pixels:
  - projector leak: a smooth scene with a target silhouette crossing it, ±10 % flicker;
  - a sunlit window, saturating;
  - three glints from a target frame;
  - extra hot pixels (1e-4);
  - a person crossing the lower part of the view in 2 s.
- **Not done:** aim motion from S1's IMU recordings. The ITs use scripted aim paths (sway, flinch, sweeps).

**Week 4:**
- **What counts as visible.** A spot on the saturated window cannot be seen, and neither can one that straddles its edge. The truth now marks a spot hidden when a saturated background pixel lies within 3 px of its centre (2.5 spot sigmas). Before that, the 6 IT09 misses were all spots 0–3 px below the window's bottom edge, with half their light on saturated pixels; the tracker answered NO_SPOT, which is right.
- **Co-simulation with S1** (`tools/cosim/`), used by IT10:
  - `s1_lane_driver.py` runs inside Renode on S1's `sim/lane.resc` (two weapons on one CAN hub). It sends SYNC every 10 ms and the camera-mode commands, loads both weapons and pulls the triggers. It records every emitter gate edge (the LED's state hook) and every CAN frame each weapon sends, on virtual time, with a 10 µs quantum (the default quantum made the edge times several hundred µs coarse).
  - `cosim.py` turns the recording into the CAN log and the camera frames: each pulse S1 actually gave is rendered where that weapon aims at that moment, with the part of it that falls inside the exposure. Then it runs `aim_host`.

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
| IT10 end to end with S1 | F06, F08, F17 (S1), P02 | S1 firmware (v0.6) in Renode, two weapons firing; its emitter edges and CAN frames recorded and replayed into `aim_host` (week 4: a recording instead of vcan0, so it runs in CI) |

**Week 2:** IT01, IT02 and IT08 pass (`tests/integration/`, pytest). Until `aim_host` exists (week 3), they drive the C library through ctypes on frames from the camera simulator. The pipeline is the same: render, detect, map to the screen, zero. The HIT frame part of IT02 (F10) comes with `match` in week 3.

| Test | Result |
|---|---|
| IT01 | 54 of 54 dots found; fit 0.023 px RMS, held-out 0.023 px RMS (P06: 0.3 / 0.5 px); the calibrated mapping is within 0.055 mm of the true camera everywhere on the screen (0.041 mm before the week 3 detection changes). A displaced grid and a partly hidden grid are both refused. |
| IT02 | 500 of 500 shots found; 0.042 mm RMS, 0.082 mm at the 99th percentile, worst 0.099 mm (P01: 2 / 5 mm); bias 0.01 mm (week 3 re-run; 0.039 mm RMS in week 2). |
| IT08 | Emitter 2 mrad right and 1 mrad down from the bore: 11.5 mm RMS error before zeroing. Five zeroing shots at the centre, with 2 mm of aim scatter, measure the offset to 0.30 mm. After zeroing: 0.64 mm RMS and 1.2 mm worst over the screen. The rest is the zeroing shots' mean aim error, plus the offset growing towards the screen edges, because the misalignment is an angle. |

**Week 3:** IT03–IT07 pass through `aim_host` itself:
- `sim/scenario.py` streams each scenario into it through a pipe.
- It runs on the file transport: its CAN input is a log, its CAN output is decoded with the DBC.
- It writes the after-action records and a line per frame.
IT01, IT02 and IT08 still use the ctypes path. All 13 integration tests run in about 65 s.

| Test | Result |
|---|---|
| IT03 | 100 random breaks: 100 OK HITs, each from the right exposure.<br>• HIT latency 7.8–17.7 ms, mean 12.9 ms (P02: 25 ms), with the 2 ms margin. The HIT's own latency field agrees with true lane time to 0.1 ms.<br>• Positions within 0.2 mm.<br>• **P07:** every frame's exposure start, computed from SYNC and the camera timestamp, within 36 µs of the truth (limit ±100 µs); the tracker clock drifts 40 ppm. |
| IT04 | 487 of 488 aim pulses gave an AIM, 0.11 mm RMS from the true aim point, about 49 Hz; no AIM without a pulse.<br>• 12 records, each with S1's SHOT_DETAIL merged by shot number and 73–74 trace points from −1.0 to +0.5 s (0.04 mm RMS from the truth).<br>• The 25 mm flinch shows in every trace as a 14 mm mean dip in the last 60 ms.<br>• TRK_STATUS at 1 Hz: 100 fps, worst frame 1.9 ms, no faults. |
| IT05 | Two weapons:<br>• staggered shots: 60 of 60 attributed;<br>• both weapons in the same frame, 0.5 m apart: 100 of 100;<br>• both in the same frame, 8 mm apart: 60 of 60 AMBIGUOUS.<br>No wrong attribution anywhere. |
| IT06 | Interference set, 60 shots, 800 frames per leak level:<br>• 4 DN and 12 DN projector leak: 57 of 57 visible shots OK, no false HIT; the 3 shots behind the person are NO_SPOT.<br>• 30 DN: 50 of 57 (88 %), still no false HIT. The moving silhouette swallows spots on its edges (§12). |
| IT07 | Normal: OK. Beside the screen, inside the view: OFF_SCREEN with the position. Far off: NO_SPOT. Emitter covered: NO_SPOT. Frame dropped by the camera: DROPPED, and TRK_STATUS counts the 3 drops with its FRAMES_DROPPED bit. 400 times too bright: OK through the rim fit, within 0.14 mm. |

Plots: `results/plots/it01_calibration.png`, `results/plots/it02_accuracy.png`, `results/plots/it03_latency.png`, `results/plots/it04_aim_trace.png`.

**Week 4:** IT09 and IT10 pass, and IT01–IT08 still pass with the week 4 changes (16 tests in about 70 s; IT09 alone takes about 5 minutes and runs with `pytest -m long`). IT06 has a fourth leak level, 8 DN. Changed results: IT04 486 of 488 aim pulses gave an AIM (a new track's first spot is only a candidate, §6.6); IT06 below.

| Test | Result |
|---|---|
| IT06 | 57 of 57 visible shots OK at 4, 8, 12 and 30 DN of projector leak, no false HIT; the 3 shots behind the person are NO_SPOT. Stray spots per 1000 frames: 1.2, 1.2, 3.8 and 24 (634 at 12 DN in week 3). |
| IT09 | 100 000 frames (1000 s), two weapons sweeping the screen, 1994 shots, the interference set at 12 DN with the window, glints, hot pixels and a person:<br>• **P03:** 1953 of 1954 visible shots OK (99.95 %), 0.16 mm RMS, worst 2.8 mm; **0 false HITs**; the 40 hidden shots (behind the person, on or at the edge of the window) all NO_SPOT. The one miss is a spot 3–4 px below the saturated window's edge: NO_SPOT, not a wrong position.<br>• **P05:** thread CPU time per frame 1.0 ms median, 1.8 ms at the 99th, **2.4 ms at the 99.9th percentile**, 3.9 ms at the 99.99th, 13.4 ms worst. The slowest frames are ordinary ones (about 120 small blobs, as a typical frame), with the same wall time: the virtual machine took the CPU away (§9.4). Heap 10 800 bytes after the first frame and at the end.<br>• Frame times from SYNC within 44 µs; 96 798 AIM frames; 20.6 stray spots per 1000 frames; no track had to let go. |
| IT10 | S1's firmware (v0.6.0) on two weapons in Renode, 3.4 s of virtual time; replayed from `tests/data/s1_v0.6.0_lane.txt`, and re-recorded with `pytest -m cosim` (about 30 s):<br>• S1 strobed in every slot frame of its weapon and none of the other: 153 pulses per weapon, starting −30 to +236 µs from the plan, 563–610 µs long; at least 86.5 % of every pulse inside its exposure (§12).<br>• Its 8 SHOT_EVENTs gave 8 OK HITs, 0.03–0.19 mm from the true spot, 9.5–10.5 ms after the break (P02: 25 ms).<br>• 148 AIM frames per weapon, no stray spot; the tracker clock's 40 ppm drift estimated from S1's lane SYNC. |

### 9.4 Honest limits
- **The camera is simulated.** Accuracy is measured against the simulator's own truth. A real lens, sensor and screen bring effects the model leaves out or simplifies (screen texture, non-flat screens, lens flare, sensor non-linearity).
- **Processing time is measured on an x86 PC (WSL2) and a CI runner**, not on a target board. P05 shows the algorithm's cost and its memory behaviour; a target board needs its own measurement.
  - Both are virtual machines: time the host takes away still counts as the thread's CPU time. IT09 therefore checks P05 on the 99.9th percentile and reports the slowest frames with what was in them. With other work running on the PC, one IT09 run gave 3.4 ms at the 99.9th percentile, with the same ordinary frames as the slowest; on a quiet PC it gave 2.4 ms.
- **Renode time is virtual** (as in S1 and S2): the end-to-end test proves the timing logic and the matching, not silicon latency.
- **One camera, one flat screen, up to four weapons.**
- **Region renderer:** the background noise repeats every 48 frames (§8).
- **The SocketCAN transport of `aim_host` is written and builds, but no test runs it.** IT10 replays S1's recorded CAN traffic through the file transport instead, so it runs in CI, where hosted runners have no vcan.
- **IT10's camera is still simulated.** S1's firmware decides when each pulse happens and what it sends on CAN; where the weapon aims is scripted, because S1 has no notion of it.
- **Edge times in Renode come from virtual time with a 10 µs quantum**, so S1's pulse timing is known to about 10 µs.
- **The projector-leak levels are not measured data** (§12).

---

## 10. Host tools and deliverables
- `aim_host` with a CLI: `run`, `calibrate`, `zero`, `status`. *As built (week 3):* one program with options rather than sub-commands. The run mode has a choice of transport (`--can-in/--can-out` log files, or `--can IFNAME`). Zeroing is `--zero NODE:X:Y:N --zero-out FILE`, and saved offsets are loaded with `--zero-in`. The status is the 1 Hz TRK_STATUS frame plus a JSON summary at exit. Calibration is the Python runner `tools/calib/calibrate.py`; `aim_host` reads its `calib.json` and checks `geometry_crc32`.
- `tools/lane/lane_view.py`: the screen with targets, live aim traces per weapon, hits with shot numbers, and the after-action panel (aim trace before the break, hit, S1 trigger metrics).
- `scripts/demo.sh`: S1 in Renode with two weapons, the camera simulator, `aim_host` and the lane view, recorded to a session log, plots, a GIF for the README and a video.
- `docs/traceability.md`: every requirement to its tests.
- README with screenshots, the GIF and the results table; CI badge.

*As built (week 4):*
- `tools/lane/demo_session.py`: calibrates the lane, zeroes weapon 2 (emitter 2 mrad right, 1 mrad up), then runs a 10.6 s session: two shooters, two targets each, three shots per target with sway and flinch. Output in `docs/session/` (CAN logs, after-action records, summary, targets).
- `tools/lane/lane_view.py`: draws the session from the tracker's own CAN output and records only: the screen with targets, the aim of the last second, every hit with weapon.shot, the after-action panel of the latest shot, and the tracker status. `--png` for one moment, `--frames` for the GIF and video (`docs/img/`, made with ffmpeg).
- The demo runs the camera simulator, not S1 in Renode: S1's part end to end is IT10 (`tools/cosim/`). `scripts/demo.sh` was not written; the two Python tools replace it.

---

## 11. Milestones (about 8–10 h/week)

| Week | Work | Done when |
|---|---|---|
| 1 ✓ | Repository from the S2 template; camera simulator (geometry, spots, sensor, ground truth); `detect` and `centroid` in C with the NumPy reference; centroid accuracy vs SNR | detect and centroid suites green; the centroid curve meets the P01 budget line. **Done 7 Nov:** 21 of 21 unit tests pass, 99.7 % line coverage; C matches NumPy on 40 frames; 0.018 px at the nominal spot |
| 2 ✓ | `geom` and `calib`; OpenCV cross-check; accuracy over the screen; zeroing | IT01, IT02 and IT08 pass; P01 shown on plots. **Done 8 Nov:** 35 of 35 unit tests, 97.8 % line coverage; IT01, IT02 and IT08 pass; P01 0.039 mm RMS |
| 3 ✓ | `timesync`, `track`, `match`; `aim_host` with the file transport and SocketCAN; DBC frames; S1 v0.6 emitter strobing with its own tests | IT03–IT07 pass; S1's tests still pass with v0.6. **Done 8 Nov:** 54 of 54 unit tests, 98.4 % line coverage; 13 of 13 integration tests; S1 v0.6: 81 of 81 unit tests, 32 of 32 Renode tests (3 new) |
| 4 ✓ | IT09, IT10; lane view; demo, GIF and video; traceability; CI; README | Portfolio-ready repository with green CI. **Done 8 Nov:** 58 of 58 unit tests, 97.9 % line coverage; 16 of 16 integration tests plus IT09 (long) and the IT10 re-recording; demo 12 of 12 hits |

---

## 12. Open points to verify early (week 1)

- [x] **Simulator speed** (`camsim.py bench`, `results/camsim_bench.json`): a full frame takes 48 ms with Poisson shot noise, or 26 ms with the Gaussian approximation.
  - At those rates, the 100 000 frames of IT09 would take 43–80 minutes. Live 100 fps needs under 10 ms per frame.
  - So IT09 and the live demo need the region-only renderer (§8), in week 3.
- [x] **Centroid accuracy against SNR and spot size:** σ_PSF ≈ 1.2 px is enough, giving 0.018 px RMS at the nominal spot (§6.2).
- [x] **Saturation:** no limit up to 24 times full scale; the rim fit stays within 0.009 px (§6.2).
- [x] **S1 time sync and pulse timing** (IT10, virtual time, `results/it10_s1_cosim.json`):
  - S1's unit tests give ≤ 50 µs at ±200 ppm drift (S1 README, `timesync` row).
  - In IT10, 306 pulses of S1's firmware in Renode started −30 to +236 µs from the planned 4.75 ms into their frame: inside the ±250 µs that §6.5 allows, not inside ±100 µs. The figure includes the firmware's timer latency, not only its lane-time error.
  - **The pulses are 563–610 µs long, not 500 µs.** With a late start the end of 8 of the 306 pulses ran up to 80 µs past the exposure, so up to 13.5 % of their light was lost. Every shot still gave an OK HIT. For S1: shorten the gate by the measured overrun, or start it earlier (a follow-up in S1, not done here).
- [x] **Frame transport:** a pipe carries 1270 MB/s (a Python writer sending 1 MB frames into `cat`), 12 times the 100 MB/s needed, so shared memory is not needed.
- [x] **Projector leak at 850 nm:** I have not found public data for the 850 nm leak of a lane projector in a 1 ms exposure, so IT06 brackets it at 4, 12 and 30 DN, against an 8 DN detection floor and a 150 DN nominal spot.
  - **Week 3:** P03 was met up to 12 DN; at 30 DN a moving bright silhouette swallowed 12 % of the spots, without any false HIT.
  - **Week 4:** after the detector changes (§6.1), P03 is met at 4, 8, 12 and 30 DN. The 30 DN losses were a full blob table, not a limit of the method. The leak still has to be measured on site; the calibration capture shows it, as the ambient level in TRK_STATUS.
  - 8 DN was added because the demo found it: a step about one detection threshold high is the hard case, not the brightest one.

---

## 13. Interview story (60 seconds)

> "I wrote the camera side of a training lane. Each trainer weapon flashes an IR LED, and the camera has to say which weapon hit where. A flash at a random time almost never lands in a 1 ms exposure, so the weapon firmware strobes on lane time inside the exposure windows, and between shots each weapon flashes in its own frame slot, which gives an aim trace for the after-action review. The C pipeline finds the spot to a tenth of a pixel, removes the lens distortion and maps it to the screen with a calibrated homography, and matches it to the weapon's CAN shot event by lane time. Because the camera is simulated with ground truth, every accuracy figure is measured: 0.04 mm RMS over the screen, HIT on the bus 8 to 18 ms after the break, and no false hit in 100 000 frames of interference. The end-to-end test runs my weapon firmware in Renode, and it found that the firmware's pulses run up to a fifth longer than specified."
