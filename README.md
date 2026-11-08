# Camera aim-point tracker

[![ci](https://github.com/chad416/aim-point-tracker/actions/workflows/ci.yml/badge.svg)](https://github.com/chad416/aim-point-tracker/actions/workflows/ci.yml)

Camera-based hit and aim tracking for a small-arms training lane, in C with a simulated IR camera. Each trainer weapon flashes an infrared LED at the muzzle. A camera watching the projection screen finds the spot in every frame and maps it to the screen. It then reports which weapon hit where, plus an aim trace for each weapon between shots.

It is the third part of a lane in software:
- the trainee's weapon firmware: [trainer-weapon-firmware-twin](https://github.com/chad416/trainer-weapon-firmware-twin) (its v0.6 times the emitter's flashes to this camera)
- a motion platform: [motion-platform-controller](https://github.com/chad416/motion-platform-controller)
- this tracker

**Contents:**
- [SPEC.md](SPEC.md): specification, requirements and week-by-week results
- [protocol/tracker.dbc](protocol/tracker.dbc): the CAN frames (HIT, AIM, TRK_STATUS), extending the weapon's DBC

## Status: week 3 of 4

**How it works:**
1. **Strobed on lane time.** The camera takes 100 frames a second, 1 ms each, on the lane's clock. Each weapon flashes its LED in the middle of the frames of its own time slot. When the trigger breaks, the weapon puts its shot flash into the first frame at least 2 ms later. A random flash would hit a 1 ms exposure only 15 % of the time.
2. **Each frame:** find the spots, locate them to about 0.02 pixel, and map them to millimetres on the screen through the lens calibration.
3. **Each shot:** the weapon's SHOT_EVENT on CAN says when the trigger broke. That fixes the frame, and the spot nearest the shooter's tracked aim is the hit. Two weapons firing into the same frame are told apart by their aim tracks; when they can't be, the HIT says AMBIGUOUS rather than guessing.
4. **Out on CAN:** a HIT for every shot (with a reason if there is no position), the aim of every weapon in its slot frames, and a status every second. Each shot also gets an after-action record: its aim trace from 1 s before to 0.5 s after the break, merged with the weapon's trigger data.

**Done:**
- **C library** (`aimtrk/`, no I/O, no heap):
  - spot detection and sub-pixel centroid;
  - lens and screen geometry, and the calibration from a projected dot grid;
  - zeroing;
  - lane time from SYNC;
  - aim tracks and shot matching;
  - the CAN frames.
- **Tracker service** (`host/aim_host`, Linux):
  - frames from a pipe;
  - CAN from SocketCAN or, for repeatable runs, a log;
  - HIT, AIM and TRK_STATUS out, after-action records, zeroing.
- **Camera simulator** (`sim/`):
  - lens, sensor, spots and the calibration grid;
  - lane scenarios with weapons and their CAN traffic;
  - a fast renderer that only draws around each spot;
  - an interference set: projector leak, a sunlit window, glints, hot pixels, a person walking through.
- **Tests:**
  - **Unit tests:** 54 of 54 pass (ztest on `native_sim`), 98.4 % line coverage.
  - **Cross-checks:** against NumPy/SciPy (detection) and OpenCV (geometry, calibration).
  - **Integration tests:** 13 of 13 pass (IT01–IT08). IT03–IT07 run `aim_host` itself on simulated lane runs.
- **Weapon firmware v0.6** (other repository): camera mode, 3 new Renode tests; all 32 of its Renode tests pass.

**Not yet** (week 4): the 100,000-frame run (IT09), the end-to-end run with the weapon firmware in Renode (IT10), the lane view, the demo and the traceability table.

## Results

![HIT latency](results/plots/it03_latency.png)

*IT03: 100 shots at random break times; HIT on CAN 7.8–17.7 ms after the trigger break, in true lane time.*

| Check | Result | Limit |
|---|---|---|
| Hit accuracy over the screen (IT02, P01) | **0.042 mm RMS**, 0.082 mm at the 99th percentile, 500 of 500 shots | 2 mm RMS, 5 mm at the 99th pct |
| HIT latency after the break (IT03, P02) | 7.8–17.7 ms | 25 ms |
| Frame time from SYNC (IT03, P07) | within 36 µs of the true exposure | ±100 µs |
| Two weapons, which one hit (IT05, P04) | 160 of 160; 60 of 60 at nearly the same point reported AMBIGUOUS, never a wrong one | 100 %, ≥ 99 % in one frame |
| Interference (IT06, P03) | 57 of 57 visible shots, no false HIT, up to 12 DN projector leak | ≥ 99.9 %, ≤ 1 false HIT per 100,000 frames |
| No-hit reasons (IT07, F10) | off-screen, out of view, covered, dropped frame: each reported as such | — |
| Calibration residual (IT01, P06) | 0.023 px fitted, 0.023 px held-out | 0.3 px, 0.5 px |
| Zeroing (IT08) | 11.5 mm RMS before, 0.64 mm after five shots | P01 after zeroing |
| Processing per 1280 × 800 frame, C | about 1 ms mean, under 2 ms worst in these runs | 3 ms |
| CAN bus load, two weapons at 900 rpm (IT05) | 311 frames/s, 7.6 % of 500 kbit/s | under 10 % |

The simulation meets P01 by a factor of 50. That is not a claim about a real lane. The simulated lens follows the same distortion model the calibration fits, and the simulated screen is flat, with the projected dots exactly where they should be. A real lens, a fabric screen and a projector move points by millimetres, and the 2 mm limit is there for them (SPEC §3.3).

![after-action aim trace](results/plots/it04_aim_trace.png)

*IT04: one after-action record. The aim trace around the break (50 Hz, two slots), with the shooter's flinch just before it, and the weapon's own trigger data merged in.*

![hit accuracy over the screen](results/plots/it02_accuracy.png)

*IT02: 500 shots spread over the screen, after calibration, measured against the simulator's ground truth.*

**Found so far:**
- **The shot flash needs 2 ms, not 0.5 ms.** The first spec put the shot flash 0.5 ms after the break. The weapon firmware only sees a break up to 1 ms late and fires within 1.5 ms, so it would have missed its own frame. Both repositories now use 2 ms, which costs at most 1.5 ms of latency (SPEC §6.5).
- **A person walking past broke the background.** The background held their dark shape and produced thousands of false spots behind them, and cells across sharp edges made more. The background now snaps to big changes, and the threshold uses the brightest neighbouring cell: from 4083 to 33 false spots in the same run (SPEC §6.1).
- **The projector's IR leak sets a limit for the lane.** With 12 DN of leak in a 1 ms frame, every visible shot is found. At 30 DN, a moving bright silhouette swallows 12 % of the spots, though still without a false HIT. So the lane's IR filter must keep the leak at or below about 12 DN. These leak levels are bracketing assumptions, not measured projector data (SPEC §12).
- **Every weapon needs dark frames.** The background is frozen under a spot, so a weapon flashing in every frame would leave its aim point's background stale. Even a single weapon gets a dark frame between flashes (SPEC §6.5).
- **A single plane does not pin down k2.** The fitted k2 ranged from 0.0003 to 0.0195 around the true 0.01 while the mapping stayed within 0.073 mm. The tracker uses the mapping, never k2 alone (SPEC §6.4).
- **OpenCV's `findHomography` is the less exact one on exact data.** The C DLT reproduces the true homography to 5e-13 px; OpenCV stops at 3e-5 px (SPEC §6.3).

## Quick start (Windows 11 + WSL2 Ubuntu 24.04)

The Zephyr workspace and Python environment are shared with the other two projects. The CI workflow (`.github/workflows/ci.yml`) lists the exact setup.

```bash
bash scripts/test-unit.sh        # ztest unit tests on native_sim (twister)
bash scripts/crosscheck.sh       # C vs NumPy and OpenCV, then the centroid curves
bash scripts/test-it.sh          # integration tests IT01-IT08 and their plots
make -C host                     # build/aim_host
python3 tools/calib/calibrate.py # calibrate the simulated lane -> results/calib.json
python3 sim/camsim.py lane       # reference lane check
```

## Honest limits

- **The camera is simulated.** Accuracy is measured against the simulator's own ground truth. The sensor and lens values are assumptions for a sensor of this class, not datasheet values. The lens model is the one the calibration fits.
- **The screen is flat**, and the projected calibration dots are where the grid says.
- **The fast renderer repeats its background noise every 48 frames.** Only the spots get fresh noise in every frame.
- **The projector leak levels are assumptions** (4, 12, 30 DN), not measured data.
- **Timing is from the development PC (x86, WSL2), not a target board.**
- **The SocketCAN path of `aim_host` builds but is not exercised yet.** All tests so far use the CAN log transport; the live run with the weapon firmware (IT10) is week 4.

## Layout

```
aimtrk/            C library: detect, centroid, geom, calib, zero, timesync, track, match, proto, linalg
host/              aim_host, the tracker service (Linux)
protocol/          tracker.dbc, test vectors and their generator
sim/               camera simulator and lane scenarios
tests/unit/        ztest suites
tests/integration/ IT01-IT08 (pytest)
tools/reference/   NumPy reference, ctypes bindings, cross-checks, accuracy curves, plots
tools/calib/       calibration runner (calib.json with version and CRCs)
results/           measured results and plots
scripts/           test helpers
```
