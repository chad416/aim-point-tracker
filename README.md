# Camera aim-point tracker

[![ci](https://github.com/chad416/aim-point-tracker/actions/workflows/ci.yml/badge.svg)](https://github.com/chad416/aim-point-tracker/actions/workflows/ci.yml)

Hit and aim tracking for a small-arms training lane, in C, with a simulated IR camera. Each trainer weapon flashes an infrared LED at the muzzle; a camera watching the projection screen finds the spot in every frame and maps it to the screen. The tracker reports on CAN which weapon hit where, and an aim trace for each weapon between shots. Because the camera is simulated with ground truth, every accuracy figure below is measured against that truth.

![lane view of a two-weapon session](docs/img/lane_view.gif)

*A simulated session, drawn only from the tracker's own CAN output: two shooters, two targets each, three shots per target. Weapon 2's emitter is 2 mrad off its bore and was zeroed first. On the right, the after-action record of the latest shot: its aim trace around the break, merged with the weapon's own trigger data. ([video](docs/img/lane_view.mp4))*

**[Live demo](https://chad416.github.io/aim-point-tracker/):** the spot detector (`aimtrk/`, the same C) compiled to WebAssembly, running on a simulated 1280 × 800 IR camera in the browser. Aim with the mouse, click to fire, and switch on a sunlit window, glints or a person walking through the view to see what the detector rejects. Built and smoke-tested in CI (`.github/workflows/pages.yml`, `scripts/build-web.sh`).

It is one part of a training lane in software, with [trainer-weapon-firmware-twin](https://github.com/chad416/trainer-weapon-firmware-twin) (the weapon firmware: it times the emitter's flashes to this camera from v0.6, and IT10 runs it here end to end; `s1` in file names and `S1_*` variables refer to it) and [motion-platform-controller](https://github.com/chad416/motion-platform-controller).

[SPEC.md](SPEC.md) has the requirements, design and findings; [docs/traceability.md](docs/traceability.md) maps every requirement to its tests and result; [protocol/tracker.dbc](protocol/tracker.dbc) defines the CAN frames; [CHANGELOG.md](CHANGELOG.md) lists the changes by milestone.

## Key results

| Check | Result | Requirement |
|---|---|---|
| Hit accuracy over the screen (IT02, P01) | 0.042 mm RMS, 0.082 mm at the 99th percentile, 500 of 500 shots | 2 mm RMS, 5 mm at the 99th pct |
| HIT latency after the trigger break (IT03, P02) | 7.8–17.7 ms in lane time | 25 ms |
| Long run: 100 000 frames with interference (IT09, P03) | 1953 of 1954 visible shots within 5 mm (99.95 %), 0 false HITs | ≥ 99.9 %, ≤ 1 false HIT |
| Two weapons, which one hit (IT05, P04) | 160 of 160; 60 of 60 at nearly the same point reported AMBIGUOUS, none wrong | 100 %, ≥ 99 % in one frame |
| Processing per 1280 × 800 frame (IT09, P05) | CPU time 1.1 ms median, 2.9 ms at the 99.9th percentile (2.4–3.6 ms over runs, with the load on the PC); worst case not shown; no heap growth | 3 ms worst case |
| Frame time from SYNC (IT03, IT09, IT11, P07) | within 36 µs, 44 µs over 1000 s, about 20 µs live on vcan | ±100 µs |
| With the weapon firmware in Renode (IT10) | 8 of 8 shots OK within 0.18 mm, 9.5–10.4 ms (firmware v0.6.2); its pulses start −49 to +140 µs from plan | P02; firmware ±100 µs: not met |
| Tests | 63 ztest unit tests (≥ 90 % line coverage gate in CI), NumPy/OpenCV cross-checks, IT01–IT11 | |

The simulation meets P01 by a factor of 50. That is not a claim about a real lane: the simulated lens follows the same distortion model the calibration fits, and the simulated screen is flat with the projected dots exactly where they should be. A real lens, a fabric screen and a projector move points by millimetres; the 2 mm limit is there for them (SPEC §3.3).

## Honest limits

- **The camera is simulated.** The sensor and lens values are assumptions for a sensor of this class, not datasheet values; the screen is flat; the projector-leak levels (4–30 DN) are assumptions. The fast renderer repeats its background noise every 48 frames.
- **P05 is shown at the 99.9th percentile, not as a worst case, and not under load.** Timing comes from WSL2 and CI runners, both virtual machines. `aim_host` measures each frame's thread CPU time and wall time; on WSL2 the CPU-time clock reads about 8 % more than the wall clock on the same frames, and the slowest frames by either clock are ordinary frames slowed by the host (12–14 ms). The 99.9th percentile was 2.4 ms on a quiet PC, 2.9 ms in the recorded run, and 3.4–3.6 ms with other work running on the PC, over the limit. A worst case needs a target board; nothing locks memory or sets real-time scheduling unless `--rt` is given.
- **The weapon firmware does not hold its pulses within ±100 µs.** IT10 measured −49 to +140 µs with v0.6.2 (inside the ±250 µs the exposure allows, so no light was lost). In IT10 only the weapon is real firmware; where it aims is scripted and the camera is the simulator.
- **SocketCAN is tested on vcan only** (IT11), not on a CAN controller with arbitration and bus errors. IT11 is skipped where no vcan interface is up; whether GitHub's hosted runners provide the vcan module is not confirmed.

## Quick start

**Without Zephyr** (Linux or WSL2: gcc, make, Python 3.12):

```bash
make -C host                              # build/aim_host (-Werror)
python3 -m venv .venv && . .venv/bin/activate
pip install -r requirements.txt
python3 -m pytest tests/integration       # IT01-IT08, IT10 (replay), IT11 if vcan0 is up
python3 tools/lane/demo_session.py --out build/session
python3 tools/lane/lane_view.py build/session --png build/lane_view.png
```

IT11 needs `sudo modprobe vcan && sudo ip link add dev vcan0 type vcan && sudo ip link set up vcan0`. IT09, the 100 000-frame run, is `python3 -m pytest tests/integration -m long` (about 6 minutes).

**With Zephyr** (the unit tests on `native_sim`, and the C vs NumPy/OpenCV cross-check): set up a west workspace from `west.yml` as `.github/workflows/ci.yml` does (`west init -l`, `west update`, `west packages pip --install`, `west sdk install`), then

```bash
export AIMTRK_WS=/path/to/west-workspace  AIMTRK_VENV=/path/to/venv   # defaults: ~/trainer-ws, ~/zvenv
bash scripts/test-unit.sh                 # 63 ztest unit tests (twister)
bash scripts/crosscheck.sh                # C vs NumPy and OpenCV, centroid accuracy curves
bash scripts/test-it.sh                   # the integration tests and their plots
```

Zephyr's build breaks on paths with spaces; the scripts then bind-mount the checkout at `$AIMTRK_SRC` (default `~/aimtrk-src`, needs root). Recording IT10 again (`pytest -m cosim`) needs Renode 1.17 and the weapon firmware's ELF and `sim/` (`S1_ELF`, `S1_SIM`, see `tools/cosim/cosim.py`).

## How it works

1. **Strobed on lane time.** The camera takes 100 frames a second, 1 ms each, on the lane's clock. Each weapon flashes in the middle of the frames of its own time slot; at a trigger break, it puts its shot flash into the first frame at least 2 ms later. A flash at a random time would hit a 1 ms exposure only 15 % of the time.
2. **Each frame:** find the spots against a running background, locate them to about 0.02 pixel RMS (0.034 px worst-case bias without noise), and map them to millimetres on the screen through the lens calibration.
3. **Each shot:** the weapon's SHOT_EVENT on CAN gives the break time, which fixes the frame; the spot nearest the shooter's tracked aim is the hit. When two weapons fire into the same frame and their tracks cannot tell the spots apart, the HIT says AMBIGUOUS rather than guessing.
4. **Out on CAN:** a HIT for every shot (with a reason if there is no position), the aim of every weapon in its slot frames, and a status every second. Each shot also gets an after-action record: its aim trace from 1 s before to 0.5 s after the break, merged with the weapon's trigger data.

The C library (`aimtrk/`) has no I/O and no heap; `aim_host` adds frames from a pipe, CAN from SocketCAN (kernel-timestamped) or a log file, and the records.

## What testing found

- **SYNC was stamped when the loop read it, not when it arrived.** Live, the loop can be busy with a 1 MB frame: on vcan, frames waited up to 4.6 ms before being read, and one late SYNC moves the clock fit by a tenth of a millisecond or more. SocketCAN frames now carry kernel timestamps, and a SYNC that disagrees with the fit is held out unless four in a row agree on a step (SPEC §6.7, IT11).
- **The weapon firmware's flashes were up to a fifth too long.** IT10 runs the real firmware in Renode and records its emitter edges. In v0.6.0 the pulses lasted 563–610 µs instead of 500 µs, so 8 of 306 ran up to 80 µs past the end of the exposure. The firmware's timer added a tick to each pulse; v0.6.1 fixed it and IT10 now sees 480–526 µs, all inside the exposure (SPEC §12).
- **A stray could take over a weapon's track.** The first 100 000-frame run missed 24 % of the shots and made 13 false hits: when a weapon's spot disappeared, its track jumped to a stray that came back every 48 frames. A track now starts only on a strong spot seen again in the weapon's next slot frame, and lets go of a wrong spot after three slot frames (SPEC §6.6). A shot of a weapon with no track at all now takes only a new, unexplained spot; before, it could take any strong stray.
- **A scene change one threshold high was the hard case, not the brightest one.** At 8 DN of projector leak the demo lost 4 of 12 shots among 6499 stray spots: noise broke a moving silhouette's edge into spot-sized pieces, which also filled the blob table and pushed the real spots out. A spot must now stand out from its own surroundings, and blobs too small to be a spot take no table place; the same fix recovered 7 of 57 shots lost at 30 DN (SPEC §6.1).
- **A person walking past broke the background**: it held their dark shape and produced thousands of false spots. It now snaps to big changes, and the threshold uses the brightest neighbouring cell: from 4083 to 33 false spots in the same run (SPEC §6.1).
- **The shot flash needs 2 ms, not 0.5 ms**, because the firmware only sees a break up to 1 ms late and fires within 1.5 ms. **Every weapon needs dark frames**, because the background is frozen under a spot. **A single plane does not pin down k2** (0.0003–0.0195 around the true 0.01), so only the fitted mapping is used (SPEC §6.4, §6.5).

## Layout

```
aimtrk/            C library: detect, centroid, geom, calib, zero, timesync, track, match, proto, linalg
host/              aim_host, the tracker service (Linux), and its Makefile
protocol/          tracker.dbc, test vectors and their generator
sim/               camera simulator and lane scenarios
tests/unit/        ztest suites (native_sim)
tests/integration/ IT01-IT11 (pytest)
tests/data/        the weapon firmware's Renode run, replayed by IT10
tools/cosim/       Renode driver and replay for IT10
tools/lane/        demo session, lane view, scenario runner
tools/reference/   NumPy reference, ctypes bindings, cross-checks, accuracy curves, plots
tools/calib/       calibration runner (calib.json with version and CRCs)
docs/              traceability, the demo session, lane view images
web/               browser demo: WebAssembly entry points, page, renderer, smoke test
results/           measured results and plots
scripts/           test helpers (environment: AIMTRK_WS, AIMTRK_VENV, AIMTRK_SRC)
```

MIT licence ([LICENSE](LICENSE)).
