"""IT10 end to end with S1 (SPEC §9.3): S1's real firmware (v0.6.1, two weapons
in Renode) strobes its emitter on its own lane-time estimate and sends its own
SHOT_EVENT frames; the camera frames are built from the emitter edges it gave,
and aim_host runs on them with S1's CAN traffic.

The Renode run is recorded in tests/data/s1_v0.6.1_lane.txt (made with
tools/cosim/cosim.py --renode), so this test replays it without Renode;
test_rerecord (marker cosim) runs S1 again and checks the same.

  F17 / S1   every exposure of a weapon's slot has its pulse, none in the other
             slot's; each pulse starts within 250 us of 4.75 ms into its frame,
             is 500 us +- 50 us long (the 10 us quantum coarsens interrupt
             timing; S1's own test, IT17d, sees 500 us), and lies entirely
             inside the 1 ms exposure. (v0.6.0 gave 0.56-0.61 ms pulses, some
             ending past the exposure; S1 fixed that in v0.6.1, SPEC 12.)
  F10, P01   every SHOT_EVENT gives a HIT OK within 1 mm of the true spot
  P02        HIT latency (break -> HIT, lane time) <= 25 ms
  F09        AIM frames for both weapons; no stray spots
"""
from __future__ import annotations

import math
import os
import pathlib
import shutil
import sys

import pytest

import lane as LN

sys.path.insert(0, str(LN.ROOT / "tools" / "cosim"))
import cosim  # noqa: E402

RECORDING = LN.ROOT / "tests" / "data" / "s1_v0.6.1_lane.txt"


def check(rec: cosim.Recording, run_dir: pathlib.Path, calib: pathlib.Path) -> dict:
    r = cosim.build_and_run(rec, run_dir, calib)
    truth, out = r["truth"], r["out"]
    pulses = truth["pulses"]

    # S1's strobes: slot frames from the first pulse on, timing inside the exposure.
    res = {"pulses": len(pulses), "shots": len(truth["shots"])}
    for node, slot in cosim.WEAPONS.items():
        ks = sorted(p["k"] for p in pulses if p["node"] == node)
        assert ks, f"node {node}: no pulses"
        wrong_slot = [k for k in ks if k % cosim.N_SLOTS != slot]
        assert not wrong_slot, f"node {node}: pulses in the other slot's frames {wrong_slot[:5]}"
        expect = set(range(ks[0], min(truth["k_last"], ks[-1] + 1), cosim.N_SLOTS))
        missing = sorted(expect - set(ks))
        assert not missing, f"node {node}: slot frames without a pulse {missing[:5]}"
        res[f"node{node}_pulses"] = len(ks)
    err = [p["err_us"] for p in pulses]
    width = [p["end_us"] - p["start_us"] for p in pulses]
    inside, overrun = [], []
    for p in pulses:
        e0 = p["k"] * cosim.SC.FRAME_US + cosim.SC.EXPOSURE_START_US
        e1 = e0 + cosim.camsim.EXPOSURE_US
        inside.append((min(p["end_us"], e1) - max(p["start_us"], e0)) / (p["end_us"] - p["start_us"]))
        overrun.append(max(0, p["end_us"] - e1))
    res["pulse_start_error_us"] = {"min": min(err), "max": max(err)}
    res["pulse_width_us"] = {"min": min(width), "max": max(width)}
    res["pulse_inside_exposure_min"] = round(min(inside), 3)
    res["pulses_past_exposure_end"] = sum(1 for o in overrun if o > 0)
    res["exposure_overrun_us_max"] = max(overrun)
    assert max(abs(e) for e in err) <= 250, res["pulse_start_error_us"]
    assert min(inside) >= 0.8, res
    assert all(450 <= w <= 550 for w in width), res["pulse_width_us"]
    assert res["exposure_overrun_us_max"] == 0, res

    # Shots: S1's SHOT_EVENTs, the tracker's HITs.
    hits = {(h["node"], h["shot_no"]): h for h in out["hits"]}
    rows = []
    for s in truth["shots"]:
        h, p = hits.get((s["node"], s["shot_no"])), s["pulse"]
        assert p is not None, f"no shot pulse from node {s['node']} in frame {s['k']}"
        assert h is not None, f"no HIT for node {s['node']} shot {s['shot_no']}"
        d = math.hypot(h["x_mm"] - p["x_mm"], h["y_mm"] - p["y_mm"])
        rows.append({"node": s["node"], "shot_no": s["shot_no"], "k": s["k"], "result": h["result"],
                     "error_mm": round(d, 3), "latency_ms": round(float(h["latency"]), 1)})
    res["hits"] = rows
    assert len(rows) == 8 and {r["node"] for r in rows} == {1, 2}
    assert all(r["result"] == 0 for r in rows), rows
    assert all(r["error_mm"] <= 1.0 for r in rows), rows
    assert all(r["latency_ms"] <= 25.0 for r in rows), rows

    summ = out["summary"]
    aims = {n: sum(1 for a in out["aims"] if a["node"] == n) for n in cosim.WEAPONS}
    res.update(aims=aims, stray_spots=summ["match"]["stray_spots"], frames=summ["frames"],
               can_in=summ["can_in"], drift_ppm=summ["drift_ppm"])
    for n in cosim.WEAPONS:
        assert aims[n] >= 0.9 * res[f"node{n}_pulses"] - 8, (n, aims[n])
    assert summ["match"]["stray_spots"] == 0
    return res


def test_s1_end_to_end(lib, calibration, tmp_path):
    res = check(cosim.parse(RECORDING), tmp_path / "it10", calibration["path"])
    LN.write_result("it10_s1_cosim.json", res)


@pytest.mark.cosim
def test_rerecord(lib, calibration, tmp_path):
    """Run S1 in Renode again (needs renode, S1's ELF and sim/, see cosim.run_renode) and check the same."""
    if shutil.which("renode") is None or not os.path.exists(os.environ.get(
            "S1_ELF", "/root/trainer-ws/build/app/zephyr/zephyr.elf")):
        pytest.skip("Renode or S1's firmware not available")
    events = tmp_path / "s1_events.txt"
    cosim.run_renode(events)
    check(cosim.parse(events), tmp_path / "it10", calibration["path"])
