#!/usr/bin/env python3
"""Synthetic trade-off sweep: accuracy vs false activations/min vs latency over theta and N.

Decoding is independent of the intent parameters, so each session is decoded once and the
IntentDetector is replayed for every (theta, N). Output: outputs/metrics/bci_tradeoff.png + .json
"""
from __future__ import annotations

import json
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from eyesteer_core import config  # noqa: E402
from eyesteer_core.bci.eeg_sim import SyntheticEEG  # noqa: E402
from eyesteer_core.bci.intent import IntentDetector  # noqa: E402
from eyesteer_core.bci.ssvep_cca import SSVEPDecoder  # noqa: E402

THETAS = [0.25, 0.30, 0.35, 0.40, 0.45, 0.50]
NS = [2, 3, 4, 5, 6]
# validated reference categorical palette (dataviz skill references/palette.md), fixed order
SERIES = ["#2a78d6", "#eb6834", "#1baf7a", "#eda100", "#e87ba4"]
INK, INK2, SURFACE, GRID = "#0b0b0b", "#52514e", "#fcfcfb", "#e4e3df"


def session(seed: int, snr: float, freqs: list, fs=256):
    """Attend trials (5 s each, each marker twice) separated by 6 s gaps that contain 0.75 s glances,
    followed by 90 s of looking around (glances + blinks). Returns decode results + ground truth."""
    rng = np.random.default_rng(seed)
    sim = SyntheticEEG(snr=snr, seed=seed)
    d = config.bci()["decoder"]
    dec = SSVEPDecoder(freqs, list(range(len(freqs))), fs=fs, window_s=d["window_s"], step_s=d["step_s"])
    timeline = []
    t = 4.0
    order = list(range(len(freqs) - 1)) * 2
    rng.shuffle(order)
    for m in order:
        timeline.append((t, t + 5.0, m, True))
        g = t + 5.0 + rng.uniform(1.5, 3.5)
        timeline.append((g, g + 0.75, int(rng.integers(len(freqs))), False))
        t += 11.0
    end_trials = t
    while t < end_trials + 90:
        g = t + rng.uniform(2, 6)
        timeline.append((g, g + rng.uniform(0.3, 0.9), int(rng.integers(len(freqs))), False))
        t = g + 1
    dur = t + 2
    results = []
    while sim.t < dur:
        tgt = None
        for a, b, m, _ in timeline:
            if a <= sim.t < b:
                tgt = freqs[m]
        results += dec.push(sim.generate(32, tgt, 0.0)[:, [4]], sim.t + 32 / fs)
    attend_time = sum(b - a for a, b, _, att in timeline if att)
    return results, timeline, dur - attend_time


def evaluate(results, timeline, rest_time, theta, N, delta=0.08):
    det = IntentDetector(theta, delta, N, 2.0)
    trials = [(a, b, m) for a, b, m, att in timeline if att]
    hits, lat, false = 0, [], 0
    first = {}
    for r in results:
        s = det.update(r)
        if s is None:
            continue
        # a repeat of the trial marker while the 2 s window still holds trial data is not a false activation
        tr = next(((a, b, m) for a, b, m in trials if a <= s.t <= b + 2.0), None)
        if tr and s.marker_id == tr[2]:
            if tr not in first:
                first[tr] = s.t - tr[0]
        else:
            false += 1
    hits = len(first)
    lat = list(first.values())
    return {"accuracy": hits / len(trials), "false_per_min": false / (rest_time / 60.0),
            "latency_s": float(np.median(lat)) if lat else float("nan")}


def main() -> int:
    table = config.frequency_table()
    freqs = [s["frequency_exact"] for s in table["slots"]]
    snr = float(sys.argv[1]) if len(sys.argv) > 1 else 0.8
    snr_hi = 1.5   # stronger responders: short glances produce more correlation -> false-activation check
    grids = {}
    for sn in (snr, snr_hi):
        sessions = [session(seed, sn, freqs) for seed in range(4)]
        g = {}
        for th in THETAS:
            for N in NS:
                ev = [evaluate(r, tl, rt, th, N) for r, tl, rt in sessions]
                g[(th, N)] = {k: float(np.nanmean([e[k] for e in ev])) for k in ev[0]}
        grids[sn] = g
    grid = grids[snr]
    # choose: N >= 3 (>= 0.75 s of consistent evidence: anti-Midas-touch), at BOTH SNRs
    # false <= 0.2/min and latency <= 3.0 s; then max accuracy at the low SNR, fewer false, lower latency
    # latency <= 2.75 s: the median leaves >= 0.25 s for the tail under the 3.0 s budget (Section 24.7)
    ok = [(k, v) for k, v in grid.items() if k[1] >= 3 and v["latency_s"] <= 2.75
          and max(grids[sn][k]["false_per_min"] for sn in grids) <= 0.2]
    best = max(ok, key=lambda kv: (round(kv[1]["accuracy"], 2), -kv[1]["false_per_min"], -kv[1]["latency_s"])) if ok else None
    out = ROOT / "outputs/metrics"
    out.mkdir(parents=True, exist_ok=True)
    (out / "bci_tradeoff.json").write_text(json.dumps(
        {"snr": snr, "grid": [{"theta": k[0], "N": k[1], **v} for k, v in grid.items()],
         "grid_snr_high": [{"theta": k[0], "N": k[1], **v} for k, v in grids[snr_hi].items()],
         "chosen": {"theta": best[0][0], "N": best[0][1], **best[1]} if best else None}, indent=2))
    plot(grid, best, snr, out / "bci_tradeoff.png")
    if best:
        print(f"[tradeoff] SNR {snr}: chosen theta={best[0][0]} N={best[0][1]} -> "
              f"acc {best[1]['accuracy']:.2f}, false {best[1]['false_per_min']:.2f}/min, latency {best[1]['latency_s']:.2f} s")
    return 0


def plot(grid, best, snr, path):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    plt.rcParams.update({"font.family": "DejaVu Sans", "font.size": 11, "axes.edgecolor": GRID,
                         "axes.labelcolor": INK2, "xtick.color": INK2, "ytick.color": INK2})
    fig, axes = plt.subplots(1, 3, figsize=(15, 4.8), facecolor=SURFACE)
    panels = [("accuracy", "Accuracy (attended trials selected)", (0, 1.05)),
              ("false_per_min", "False activations per minute", None),
              ("latency_s", "Median latency, gaze onset to selection (s)", None)]
    for ax, (key, title, ylim) in zip(axes, panels):
        ax.set_facecolor(SURFACE)
        for spine in ("top", "right"):
            ax.spines[spine].set_visible(False)
        ax.grid(axis="y", color=GRID, linewidth=0.8)
        ax.set_axisbelow(True)
        for i, N in enumerate(NS):
            ys = [grid[(th, N)][key] for th in THETAS]
            ax.plot(THETAS, ys, color=SERIES[i], linewidth=2, marker="o", markersize=5, label=f"N = {N}")
            if key != "false_per_min":
                ax.annotate(f"N={N}", (THETAS[-1], ys[-1]), xytext=(6, 0), textcoords="offset points",
                            color=INK2, fontsize=9, va="center")
        if best:
            (th, N), v = best
            ax.plot([th], [v[key]], marker="o", markersize=13, markerfacecolor="none",
                    markeredgecolor=INK, markeredgewidth=1.5)
        ax.set_title(title, color=INK, fontsize=12, loc="left")
        ax.set_xlabel("CCA threshold θ")
        ax.set_xlim(THETAS[0] - 0.02, THETAS[-1] + 0.06)
        if ylim:
            ax.set_ylim(*ylim)
    axes[0].legend(frameon=False, loc="lower left", fontsize=9, labelcolor=INK2)
    sub = (f"Synthetic EEG, SNR {snr}, decoder step {config.bci()['decoder']['step_s']} s, margin δ = 0.08. Circled: chosen default"
           + (f" θ = {best[0][0]}, N = {best[0][1]}" if best else "") + " (N ≥ 3; ≤ 0.2 false/min at SNR 0.8 and 1.5; median latency ≤ 2.75 s; then max accuracy).")
    fig.suptitle("SSVEP intent trade-off: SIMULATED EEG", x=0.01, ha="left", color=INK, fontsize=14, weight="bold")
    fig.text(0.01, 0.89, sub, color=INK2, fontsize=10)
    fig.tight_layout(rect=(0, 0, 1, 0.87))
    fig.savefig(path, dpi=110, facecolor=SURFACE)


if __name__ == "__main__":
    sys.exit(main())
