"""IT07 no-hit reasons (SPEC F10), through aim_host.

Every SHOT_EVENT gets a HIT; when there is no usable position it says why.
One weapon, three shots per situation:
  normal                                   OK
  aimed beside the screen, inside the view OFF_SCREEN, with the position
  aimed far off, outside the camera view   NO_SPOT
  emitter covered                          NO_SPOT
  the camera drops the shot frame          DROPPED (and TRK_STATUS counts it)
  a spot 400 times brighter than nominal   still located: OK via the rim fit
                                           (SATURATED is for a failed rim fit,
                                           unit test match.test_no_hit_reasons)
"""
from __future__ import annotations

import json
import math
import subprocess

import numpy as np

import lane as LN
import scenario as SC

T0 = 1000.0
# situation, aim point (mm)
PLAN = [("normal", (300.0, 200.0)), ("beside", (1560.0, 0.0)), ("far_off", (2600.0, 0.0)),
        ("covered", (-400.0, -300.0)), ("dropped", (-100.0, 100.0)), ("bright", (500.0, -500.0))]
SEG_S = 1.5   # each situation lasts 1.5 s; shots at +0.8, +1.0, +1.2 s


def seg_of(t: float) -> int:
    return min(max(int((t - T0) / SEG_S), 0), len(PLAN) - 1)


def test_no_hit_reasons(lib, cam, calibration, tmp_path):
    breaks, expect = [], []
    for i, (name, _) in enumerate(PLAN):
        for j in range(3):
            # 1.5 ms into the frame: the shot frame is that frame, whatever the 20 us sync error.
            k = int(round((T0 + i * SEG_S + 0.8 + 0.2 * j) * 100))
            breaks.append((k * SC.FRAME_US + 1500) * 1e-6)
            expect.append((name, k))

    def aim(t):
        x, y = PLAN[seg_of(t)][1]
        return x + 2 * math.sin(6 * t), y + 2 * math.cos(5 * t)

    def covered(t):
        return PLAN[seg_of(t)][0] == "covered"

    def brightness(t):
        # 400 times the nominal energy: the weapon close to the screen, or a wrong emitter setting.
        return 400.0 if PLAN[seg_of(t)][0] == "bright" else 1.0

    w = SC.Weapon(node=1, slot=0, aim=aim, breaks_s=breaks, blocked=covered, brightness=brightness)
    scn = SC.Scenario(weapons=[w], t0_s=T0, duration_s=len(PLAN) * SEG_S + 0.2,
                      drop_k={k for name, k in expect if name == "dropped"}, seed=71)
    truth, out = LN.run_scenario(scn, tmp_path / "it07", calibration["path"], cam)

    names = {0: "OK", 1: "NO_SPOT", 2: "OFF_SCREEN", 3: "AMBIGUOUS", 4: "SATURATED", 5: "DROPPED"}
    hits = {h["shot_no"]: h for h in out["hits"]}
    got = {}
    for s, (name, _) in zip(sorted(truth["shots"], key=lambda s: s["shot_no"]), expect):
        h = hits[s["shot_no"]]
        rec = {"result": names[h["result"]]}
        if h["result"] in (0, 2, 4):
            rec["error_mm"] = round(math.hypot(h["x_mm"] - s["x_mm"], h["y_mm"] - s["y_mm"]), 3)
        got.setdefault(name, []).append(rec)
    dropped_status = sum(s["dropped"] for s in out["status"])
    faults = sorted({s["faults"] for s in out["status"]})
    LN.write_result("it07_no_hit_reasons.json", {"results": got, "status_dropped_total": dropped_status,
                                                  "status_faults_seen": faults})
    want = {"normal": "OK", "beside": "OFF_SCREEN", "far_off": "NO_SPOT", "covered": "NO_SPOT",
            "dropped": "DROPPED", "bright": "OK"}
    for name, result in want.items():
        assert [r["result"] for r in got[name]] == [result] * 3, (name, got[name])
    for name in ("normal", "beside", "bright"):
        assert all(r["error_mm"] <= 1.0 for r in got[name]), (name, got[name])
    assert dropped_status == 3
    assert any(f & 0x04 for f in faults), "FRAMES_DROPPED fault bit"


def test_frame_of_another_size_skipped(calibration, tmp_path):
    """A frame whose size differs from the first one is skipped and counted (its exposure counts as
    dropped), not read with the first frame's stride."""
    k0 = 100_000  # exposure numbers; the tracker's clock is lane time here
    frames, can_in = tmp_path / "frames.bin", tmp_path / "can_in.log"
    with open(frames, "wb") as f:
        for i in range(6):
            img = np.full((400 if i == 3 else 800, 1280), 20, np.uint8)
            LN.camsim.write_frame(f, img, i, t_local_us=(k0 + i) * SC.FRAME_US + SC.EXPOSURE_START_US)
    sync = [(k0 - 40 + j) * SC.SYNC_PERIOD_US for j in range(50)]  # SYNC (ID 0x080): 04 + lane time, LE
    can_in.write_text("".join(f"({t // 10**6}.{t % 10**6:06d}) can0 080#04{t.to_bytes(4, 'little').hex()}\n"
                              for t in sync))
    r = subprocess.run([str(LN.build_host()), "--frames", str(frames), "--calib", str(calibration["path"]),
                        "--weapon", "1:0", "--can-in", str(can_in), "--can-out", str(tmp_path / "can_out.log")],
                       capture_output=True, text=True, check=True)
    summ = json.loads(r.stdout.strip().splitlines()[-1])
    assert summ["frames_bad_size"] == 1 and summ["frames"] == 5, summ
    assert summ["camera_dropped"] == 0 and summ["match"]["dropped"] == 1, summ
    assert "1280x400, not 1280x800: skipped" in r.stderr
