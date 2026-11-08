"""IT06 interference (SPEC F02, P03), through aim_host.

The interference set (SPEC §8) at three projector-leak levels: the projected
scenario leaking through the 850 nm filter (a smooth scene with a target
silhouette moving across it, flickering), a sunlit window, glints from a
target frame, extra hot pixels, and a person crossing in front of the screen.
One weapon sweeps over the whole screen and fires 60 shots on the way.

  P03  >= 99.9 % of the visible shots give an OK HIT on the true spot;
       false HITs (a position where there was no shot spot) <= 1 per 100 000
       frames, so none in these 800 frames. Shots hidden by the person must
       come back as NO_SPOT, not as a position.

The leak levels are not measured projector data (SPEC §12): they bracket a
dark lane (4 DN), a typical one (12 DN) and a bright one (30 DN) against the
8 DN detection floor. P03 is required up to 12 DN. At 30 DN the moving
silhouette swallows spots that land on its edges; the test records how many
and still requires no false HIT: it sets the leak limit for the lane (the
850 nm filter and the projector), SPEC §12.
"""
from __future__ import annotations

import math

import numpy as np
import pytest

import lane as LN
import scenario as SC

T0 = 1000.0
N_SHOTS = 60


@pytest.mark.parametrize("leak_dn", [4.0, 12.0, 30.0])  # P03 required at 4 and 12 DN
def test_interference(lib, calibration, tmp_path, leak_dn):
    cam = LN.camsim.Camera()  # its own camera: the extra hot pixels must not leak into other tests
    breaks = [T0 + 0.5 + 0.12 * i for i in range(N_SHOTS)]

    def aim(t: float) -> tuple[float, float]:
        # A sweep over the whole screen (up to about 0.6 m/s), with a little sway.
        u = t - T0
        return (1300 * math.sin(2 * math.pi * 0.07 * u) + 3 * math.sin(9 * t),
                700 * math.sin(2 * math.pi * 0.11 * u + 0.5) + 3 * math.cos(7 * t))

    interf = SC.Interference(leak_dn=leak_dn, window=True, glints=True, extra_hot=1e-4,
                             passer=(int(T0 * 100) + 300, int(T0 * 100) + 500))
    w = SC.Weapon(node=1, slot=0, aim=aim, breaks_s=breaks)
    scn = SC.Scenario(weapons=[w], t0_s=T0, duration_s=8.0, interference=interf, seed=61 + int(leak_dn))
    truth, out = LN.run_scenario(scn, tmp_path / f"it06_{int(leak_dn)}", calibration["path"], cam)

    hits = {(h["node"], h["shot_no"]): h for h in out["hits"]}
    visible = ok = hidden = hidden_ok = false_hits = 0
    for s in truth["shots"]:
        h = hits[(s["node"], s["shot_no"])]
        if s.get("hidden"):
            hidden += 1
            hidden_ok += h["result"] == 1
            false_hits += h["result"] in (0, 2, 4)
            continue
        visible += 1
        d = math.hypot(h["x_mm"] - s["x_mm"], h["y_mm"] - s["y_mm"])
        if h["result"] == 0 and d <= 2.0:
            ok += 1
        elif h["result"] in (0, 2, 4):
            false_hits += 1
    frames = out["summary"]["frames"]
    rej = np.sum([f["rejected"] for f in out["frames"]], axis=0)
    res = {
        "leak_dn": leak_dn, "frames": frames, "shots_visible": visible, "shots_ok": ok,
        "detection_pct": 100.0 * ok / visible, "shots_hidden": hidden, "hidden_no_spot": hidden_ok,
        "false_hits": false_hits, "stray_spots": out["summary"]["match"]["stray_spots"],
        "stray_spots_per_1000_frames": 1000.0 * out["summary"]["match"]["stray_spots"] / frames,
        "rejected": dict(zip(("small", "large", "weak", "elongated"), (int(v) for v in rej))),
        "proc_ms": out["summary"]["proc_ms"],
    }
    LN.write_result(f"it06_interference_{int(leak_dn)}dn.json", res)
    assert hidden > 0, "the person must hide some shots"
    assert hidden_ok == hidden
    assert false_hits == 0
    if leak_dn <= 12.0:
        assert ok >= 0.999 * visible
