#!/usr/bin/env python3
"""Centroid accuracy against signal level and spot size (SPEC §6.2, §12; P01 budget line).

Renders single spots on 64 x 64 patches of the simulated sensor (Poisson shot
noise, read noise, fixed-pattern noise, 8-bit quantisation) at random
sub-pixel positions, runs the C detect + centroid on each, and measures the
error against the true position:

  unsaturated  radial RMS error vs peak SNR for spot sizes sigma_PSF 0.8-2.0 px;
               where it crosses the 0.1 px budget line (SPEC §3.3)
  noise-free   worst pixel-phase bias (8-bit quantisation only)
  saturated    plain centroid vs rim fit, peak 1-20x full scale (sigma 1.2 px)

Usage: centroid_curve.py [--trials 400]
  -> results/centroid_curve.json, results/plots/centroid_snr.png, results/plots/centroid_saturation.png
"""
from __future__ import annotations

import argparse
import json
import math
import pathlib
import sys

import numpy as np

HERE = pathlib.Path(__file__).resolve().parent
ROOT = HERE.parents[1]
sys.path.insert(0, str(ROOT / "sim"))

import aimtrk_c as AT  # noqa: E402
import camsim  # noqa: E402

N = 64
SIGMAS = (0.8, 1.2, 1.6, 2.0)
NOMINAL_SIGMA = 1.2
NOMINAL_PEAK = 150.0  # DN, about 60 % of full scale: the emitter design point
BUDGET_PX = 0.1       # SPEC §3.3: centroid noise in the P01 error budget
PEAKS = (5, 6, 8, 10, 12, 16, 20, 25, 32, 40, 50, 64, 80, 100, 128, 150, 180, 220)
SAT_PEAKS = (150, 255, 400, 600, 1000, 1500, 2500, 4000, 6000)

SERIES = ["#2a78d6", "#eb6834", "#1baf7a", "#eda100"]  # categorical slots 1-4 (as S2's plots)
INK, INK2, MUTED, GRID, SURFACE = "#0b0b0b", "#52514e", "#8a8984", "#e6e5e0", "#fcfcfb"


def trial_set(lib, cam: camsim.Camera, rng, sigma: float, peak: float, trials: int,
              noise: bool = True, phases: np.ndarray | None = None) -> dict:
    det = AT.Detector(lib, AT.default_cfg(lib, N, N))
    cov = sigma * sigma * np.eye(2)
    total = peak * 2 * math.pi * sigma * sigma
    sens = cam.sensor
    err, err_c, found, extra, snr = [], [], 0, 0, []
    n = len(phases) if phases is not None else trials
    for i in range(n):
        r0 = int(rng.integers(0, (cam.lane.height - N) // 4)) * 4
        c0 = int(rng.integers(0, (cam.lane.width - N) // 4)) * 4
        rows, cols = slice(r0, r0 + N), slice(c0, c0 + N)
        amb = np.array(cam.ambient[rows, cols], dtype=np.float32)
        d = phases[i] if phases is not None else rng.uniform(-0.5, 0.5, 2)
        u, v = 31.5 + d[0], 31.5 + d[1]
        sig = amb.copy()
        camsim.render_gaussian(sig, [(u, v)], cov, total)
        if noise:
            bg_img, img = cam.expose(amb, rng, rows, cols), cam.expose(sig, rng, rows, cols)
        else:
            bg_img = np.clip(np.floor(amb + sens.black_dn + 0.5), 0, 255).astype(np.uint8)
            img = np.clip(np.floor(sig + sens.black_dn + 0.5), 0, 255).astype(np.uint8)
        det.learn(bg_img)
        spots = det.frame(img)
        near = [s for s in spots if math.hypot(s["x"] - u, s["y"] - v) < 2.0]
        extra += len(spots) - len(near)
        if near:
            s = near[0]
            found += 1
            err.append((s["x"] - u, s["y"] - v))
            err_c.append((s["cx"] - u, s["cy"] - v))
            snr.append(s["snr"])
    e = np.array(err) if err else np.zeros((0, 2))
    ec = np.array(err_c) if err_c else np.zeros((0, 2))
    out = {"sigma_px": sigma, "peak_dn": peak, "trials": n, "found": found / n, "false_spots": extra,
           "snr": round(float(np.mean(snr)), 1) if snr else None}
    if found:
        out.update(rms_px=float(np.sqrt(np.mean(np.sum(e**2, axis=1)))),
                   bias_px=[float(b) for b in e.mean(axis=0)],
                   max_px=float(np.max(np.hypot(e[:, 0], e[:, 1]))),
                   rms_plain_px=float(np.sqrt(np.mean(np.sum(ec**2, axis=1)))))
    return out


def crossing(points: list[dict], limit: float) -> float | None:
    """Lowest peak SNR at which the RMS error is at or below `limit` and stays there (log interpolation)."""
    pts = [p for p in points if "rms_px" in p and p["found"] > 0.99]
    for i in range(len(pts) - 1, -1, -1):
        if pts[i]["rms_px"] > limit:
            if i == len(pts) - 1:
                return None
            a, b = pts[i], pts[i + 1]
            t = (math.log(limit) - math.log(a["rms_px"])) / (math.log(b["rms_px"]) - math.log(a["rms_px"]))
            return round(math.exp(math.log(a["snr_nom"]) + t * (math.log(b["snr_nom"]) - math.log(a["snr_nom"]))), 1)
    return pts[0]["snr_nom"] if pts else None


def style(plt) -> None:
    plt.rcParams.update({
        "font.size": 9, "axes.edgecolor": MUTED, "axes.labelcolor": INK2, "xtick.color": INK2,
        "ytick.color": INK2, "axes.grid": True, "grid.color": GRID, "grid.linewidth": 0.6,
        "axes.spines.top": False, "axes.spines.right": False, "axes.titlecolor": INK,
        "axes.titleweight": "bold", "axes.titlelocation": "left", "legend.frameon": False,
        "figure.facecolor": SURFACE, "axes.facecolor": SURFACE, "savefig.facecolor": SURFACE,
    })


def plain_ticks(ax, xt, yt) -> None:
    """Plain numbers on log axes, inside the data range."""
    from matplotlib.ticker import NullFormatter
    for axis, ticks, lim in ((ax.xaxis, xt, ax.get_xlim()), (ax.yaxis, yt, ax.get_ylim())):
        t = [v for v in ticks if lim[0] <= v <= lim[1]]
        axis.set_ticks(t, labels=[f"{v:g}" for v in t])
        axis.set_minor_formatter(NullFormatter())


def plot_snr(res: dict, dst: pathlib.Path) -> None:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    style(plt)
    fig, ax = plt.subplots(figsize=(7.2, 4.2), dpi=150)
    for i, s in enumerate(SIGMAS):
        pts = [p for p in res["unsaturated"] if p["sigma_px"] == s and "rms_px" in p and p["found"] > 0.99]
        x = [p["snr_nom"] for p in pts]
        y = [p["rms_px"] for p in pts]
        ax.plot(x, y, color=SERIES[i], lw=2, marker="o", ms=4, label=f"σ_PSF {s} px")
    ax.set_xscale("log")
    ax.set_yscale("log")
    _, y1 = ax.get_ylim()
    ax.axhline(BUDGET_PX, color=MUTED, lw=1, ls=(0, (4, 2)))
    ax.text(45, BUDGET_PX * 1.08, "0.1 px budget (SPEC §3.3) = 0.25 mm on the screen",
            color=INK2, fontsize=8, va="bottom")
    nom = NOMINAL_PEAK / res["background_noise_dn"]
    ax.axvline(nom, color=MUTED, lw=1, ls=(0, (2, 2)))
    ax.text(nom, y1, " nominal spot", color=INK2, fontsize=8, va="top")
    plain_ticks(ax, [10, 20, 50, 100, 200, 300], [0.01, 0.02, 0.05, 0.1, 0.2])
    ax.set_xlabel(f"peak SNR (peak signal / background noise {res['background_noise_dn']} DN)")
    ax.set_ylabel("radial RMS error, px")
    ax.set_title("Centroid error against signal, by spot size")
    ax.legend(loc="lower left")
    fig.tight_layout()
    fig.savefig(dst)
    plt.close(fig)


def plot_sat(res: dict, dst: pathlib.Path) -> None:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    style(plt)
    fig, ax = plt.subplots(figsize=(7.2, 4.0), dpi=150)
    pts = [p for p in res["saturation"] if "rms_px" in p]
    x = [p["peak_dn"] / 255.0 for p in pts]
    ax.plot(x, [p["rms_plain_px"] for p in pts], color=SERIES[0], lw=2, marker="o", ms=4,
            label="plain centroid")
    ax.plot(x, [p["rms_px"] for p in pts], color=SERIES[1], lw=2, marker="o", ms=4,
            label="reported position (rim fit when saturated)")
    ax.axhline(BUDGET_PX, color=MUTED, lw=1, ls=(0, (4, 2)))
    ax.text(x[0], BUDGET_PX * 1.08, "0.1 px budget", color=INK2, fontsize=8, va="bottom")
    ax.axvline(1.0, color=MUTED, lw=1, ls=(0, (2, 2)))
    ax.text(1.0, BUDGET_PX * 0.3, " full scale", color=INK2, fontsize=8)
    ax.set_xscale("log")
    ax.set_yscale("log")
    plain_ticks(ax, [0.5, 1, 2, 5, 10, 20], [0.005, 0.01, 0.02, 0.05, 0.1])
    ax.set_xlabel("spot peak / full scale (255 DN), σ_PSF 1.2 px")
    ax.set_ylabel("radial RMS error, px")
    ax.set_title("Saturated spots: plain centroid against the rim fit")
    ax.legend(loc="upper right", bbox_to_anchor=(1.0, 0.86))
    fig.tight_layout()
    fig.savefig(dst)
    plt.close(fig)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--trials", type=int, default=400)
    ap.add_argument("--seed", type=int, default=11)
    args = ap.parse_args(argv)

    lib = AT.load()
    cam = camsim.Camera()
    rng = np.random.default_rng(args.seed)
    bn = round(cam.sensor.background_noise_dn(), 2)
    res = {"trials": args.trials, "background_noise_dn": bn, "nominal_peak_dn": NOMINAL_PEAK,
           "budget_px": BUDGET_PX, "unsaturated": [], "noise_free": [], "saturation": []}
    phases = np.stack(np.meshgrid(np.linspace(-0.5, 0.5, 11), np.linspace(-0.5, 0.5, 11)), -1).reshape(-1, 2)
    for s in SIGMAS:
        for p in PEAKS:
            r = trial_set(lib, cam, rng, s, p, args.trials)
            r["snr_nom"] = round(p / bn, 2)
            res["unsaturated"].append(r)
        nf = trial_set(lib, cam, rng, s, NOMINAL_PEAK, 0, noise=False, phases=phases)
        res["noise_free"].append({"sigma_px": s, "max_bias_px": round(nf["max_px"], 4)})
    for p in SAT_PEAKS:
        res["saturation"].append(trial_set(lib, cam, rng, NOMINAL_SIGMA, p, args.trials))

    summary = {}
    for s in SIGMAS:
        pts = [p for p in res["unsaturated"] if p["sigma_px"] == s]
        nom = next(p for p in pts if p["peak_dn"] == NOMINAL_PEAK)
        det = [p for p in pts if p["found"] > 0.999]
        summary[str(s)] = {
            "rms_at_nominal_px": round(nom["rms_px"], 4),
            "snr_for_budget": crossing(pts, BUDGET_PX),
            "min_peak_dn_detected_999": det[0]["peak_dn"] if det else None,
        }
    res["summary"] = summary
    for p in res["unsaturated"] + res["saturation"]:
        for k in ("rms_px", "max_px", "rms_plain_px"):
            if k in p:
                p[k] = round(p[k], 4)
        if "bias_px" in p:
            p["bias_px"] = [round(b, 4) for b in p["bias_px"]]

    (ROOT / "results" / "plots").mkdir(parents=True, exist_ok=True)
    dst = ROOT / "results" / "centroid_curve.json"
    dst.write_text(json.dumps(res, indent=1) + "\n")
    plot_snr(res, ROOT / "results" / "plots" / "centroid_snr.png")
    plot_sat(res, ROOT / "results" / "plots" / "centroid_saturation.png")
    print(json.dumps({"summary": summary, "noise_free": res["noise_free"]}, indent=1))
    print("saturation (peak DN: plain / reported RMS px, found):")
    for p in res["saturation"]:
        print(f"  {p['peak_dn']:6.0f}: {p.get('rms_plain_px')} / {p.get('rms_px')}  {p['found']:.3f}")
    print("->", dst)
    ok = summary[str(NOMINAL_SIGMA)]["rms_at_nominal_px"] <= BUDGET_PX
    print("nominal spot", "meets" if ok else "MISSES", "the 0.1 px budget")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
