#!/usr/bin/env python3
"""Plots of the integration results (run after tests/integration).

  results/plots/it01_calibration.png  reprojection residual of every grid dot
  results/plots/it02_accuracy.png     hit error over the screen and its distribution (P01)
  results/plots/it03_latency.png      HIT latency after the trigger break (P02)
  results/plots/it04_aim_trace.png    one after-action aim trace with the flinch (F12)
"""
from __future__ import annotations

import json
import pathlib

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
from matplotlib.colors import LinearSegmentedColormap  # noqa: E402

ROOT = pathlib.Path(__file__).resolve().parents[2]
RES = ROOT / "results"
SERIES = ["#2a78d6", "#eb6834"]
INK, INK2, MUTED, GRID, SURFACE = "#0b0b0b", "#52514e", "#8a8984", "#e6e5e0", "#fcfcfb"
# Sequential, one hue (blue), light to dark.
BLUES = LinearSegmentedColormap.from_list("seq", ["#cfe0f6", "#2a78d6", "#0d3a73"])
SCREEN_W, SCREEN_H = 3000.0, 1690.0


def style() -> None:
    plt.rcParams.update({
        "font.size": 9, "axes.edgecolor": MUTED, "axes.labelcolor": INK2, "xtick.color": INK2,
        "ytick.color": INK2, "axes.grid": True, "grid.color": GRID, "grid.linewidth": 0.6,
        "axes.spines.top": False, "axes.spines.right": False, "axes.titlecolor": INK,
        "axes.titleweight": "bold", "axes.titlelocation": "left", "legend.frameon": False,
        "figure.facecolor": SURFACE, "axes.facecolor": SURFACE, "savefig.facecolor": SURFACE,
    })


def screen_outline(ax) -> None:
    ax.plot([-SCREEN_W / 2, SCREEN_W / 2, SCREEN_W / 2, -SCREEN_W / 2, -SCREEN_W / 2],
            [-SCREEN_H / 2, -SCREEN_H / 2, SCREEN_H / 2, SCREEN_H / 2, -SCREEN_H / 2], color=MUTED, lw=1)
    ax.set_aspect("equal")
    ax.set_xlabel("screen x, mm")
    ax.set_ylabel("screen y, mm")


def plot_calibration() -> None:
    d = json.loads((RES / "it01_calibration.json").read_text())
    r = np.array(d["residuals_px"])
    rep = d["report"]
    fig, ax = plt.subplots(figsize=(7.6, 4.6), dpi=150)
    screen_outline(ax)
    scale = 3000.0  # mm of arrow per px of residual
    for held, color, label in ((0, SERIES[0], "fitted dot"), (1, SERIES[1], "held-out dot")):
        m = r[:, 4] == held
        ax.quiver(r[m, 0], r[m, 1], r[m, 2] * scale, -r[m, 3] * scale, angles="xy", scale_units="xy",
                  scale=1, color=color, width=0.004)
        ax.plot(r[m, 0], r[m, 1], "o", ms=4, color=color, label=label)
    ax.quiver([-1350], [-960], [0.05 * scale], [0], angles="xy", scale_units="xy", scale=1, color=INK2, width=0.004)
    ax.text(-1350 + 0.05 * scale + 40, -960, "0.05 px", color=INK2, va="center", fontsize=8)
    ax.set_ylim(-1050, 920)
    ax.set_title(f"IT01: calibration residual per grid dot (arrows x {scale:.0f})\n"
                 f"fit {rep['fit_rms_px']:.3f} px RMS, held-out {rep['holdout_rms_px']:.3f} px; "
                 f"P06 limits 0.3 / 0.5 px", fontsize=10)
    ax.legend(loc="lower right", ncol=2)
    fig.tight_layout()
    fig.savefig(RES / "plots" / "it01_calibration.png")
    plt.close(fig)


def plot_accuracy() -> None:
    d = json.loads((RES / "it02_accuracy.json").read_text())
    a = np.loadtxt(RES / "it02_hits.csv", delimiter=",", skiprows=1)
    err = np.hypot(a[:, 2], a[:, 3])
    fig, (ax, ah) = plt.subplots(1, 2, figsize=(10.5, 4.0), dpi=150, gridspec_kw={"width_ratios": [1.7, 1]})
    screen_outline(ax)
    sc = ax.scatter(a[:, 0], a[:, 1], c=err, cmap=BLUES, s=14, vmin=0, vmax=float(np.percentile(err, 99.5)),
                    edgecolors="none")
    cb = fig.colorbar(sc, ax=ax, fraction=0.03, pad=0.02)
    cb.set_label("hit error, mm", color=INK2)
    cb.outline.set_edgecolor(MUTED)
    ax.set_title(f"{d['hits']} shots over the screen", fontsize=10)
    ah.hist(err, bins=30, color=SERIES[0], edgecolor=SURFACE, linewidth=1)
    for v, name in ((d["rms_mm"], "RMS"), (d["p99_mm"], "99th pct")):
        ah.axvline(v, color=INK2, lw=1, ls=(0, (4, 2)))
        ah.text(v, ah.get_ylim()[1] * 0.97, f" {name} {v:.3f} mm", color=INK2, fontsize=8, va="top")
    ah.set_xlabel("hit error, mm")
    ah.set_ylabel("shots")
    ah.set_title("P01 limits: 2 mm RMS, 5 mm at the 99th pct", fontsize=10)
    fig.suptitle("IT02: hit accuracy against the simulator's ground truth", x=0.01, ha="left",
                 fontsize=11.5, fontweight="bold", color=INK)
    fig.tight_layout()
    fig.savefig(RES / "plots" / "it02_accuracy.png")
    plt.close(fig)


def plot_latency() -> None:
    d = json.loads((RES / "it03_latency.json").read_text())
    lat = np.array(d["latencies_ms"])
    fig, ax = plt.subplots(figsize=(7.2, 3.6), dpi=150)
    ax.hist(lat, bins=np.arange(0, 26, 1.0), color=SERIES[0], edgecolor=SURFACE, linewidth=1)
    ax.axvline(25.0, color=MUTED, lw=1, ls=(0, (4, 2)))
    ax.text(24.8, ax.get_ylim()[1] * 0.95, "P02 limit 25 ms ", color=INK2, fontsize=8, ha="right", va="top")
    ax.set_xlim(0, 26)
    ax.set_xlabel("HIT on CAN after the trigger break, ms (true lane time)")
    ax.set_ylabel("shots")
    ax.set_title(f"IT03: HIT latency, {len(lat)} shots at random break times: "
                 f"{lat.min():.1f}-{lat.max():.1f} ms", fontsize=10)
    fig.tight_layout()
    fig.savefig(RES / "plots" / "it03_latency.png")
    plt.close(fig)


def plot_trace() -> None:
    d = json.loads((RES / "it04_aim_trace.json").read_text())
    r = d["example_record"]
    tr = np.array(r["trace"])
    hx, hy = r["hit"]["x_mm"], r["hit"]["y_mm"]
    fig, ax = plt.subplots(figsize=(7.2, 3.8), dpi=150)
    ax.plot(tr[:, 0], tr[:, 1] - hx, color=SERIES[0], lw=2, marker="o", ms=3, label="horizontal")
    ax.plot(tr[:, 0], tr[:, 2] - hy, color=SERIES[1], lw=2, marker="o", ms=3, label="vertical")
    ax.axvline(0, color=MUTED, lw=1, ls=(0, (4, 2)))
    ax.text(5, ax.get_ylim()[1] * 0.9, "trigger break", color=INK2, fontsize=8)
    ax.set_xlabel("time from the trigger break, ms")
    ax.set_ylabel("aim point relative to the hit, mm")
    det = r["detail"]
    ax.set_title(f"IT04: after-action record, shot {r['shot_no']}: aim trace (50 Hz), the flinch before the "
                 f"break;\nS1 detail merged: take-up {det['takeup_ms']} ms, squeeze {det['squeeze_ms']} ms",
                 fontsize=9.5)
    ax.legend(loc="lower left")
    fig.tight_layout()
    fig.savefig(RES / "plots" / "it04_aim_trace.png")
    plt.close(fig)


def main() -> int:
    style()
    (RES / "plots").mkdir(parents=True, exist_ok=True)
    plot_calibration()
    plot_accuracy()
    plot_latency()
    plot_trace()
    print("->", RES / "plots")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
