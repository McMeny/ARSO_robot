"""Per-user calibration: fit theta / delta / N from attend-each-marker trials + a rest period.

Protocol (headset and sim): each marker 10 s (others dimmed), then 20 s of "look at nothing".
"""
from __future__ import annotations

import os
from pathlib import Path

import numpy as np
import yaml

from .ssvep_cca import SSVEPDecoder


def fit_profile(trials: dict, rest: np.ndarray, freqs: dict, fs: int = 256, window_s: float = 2.0,
                step_s: float = 0.25, notch: float | None = 60.0, oz_index: int = 4) -> dict:
    """trials: {marker_id: (n, 5) raw}, rest: (n, 5) raw. Returns a profile dict."""
    ids = list(freqs.keys())
    dec = SSVEPDecoder([freqs[i] for i in ids], ids, fs=fs, window_s=window_s, step_s=step_s, notch=notch)

    def windows(x):
        d2 = SSVEPDecoder([freqs[i] for i in ids], ids, fs=fs, window_s=window_s, step_s=step_s, notch=notch)
        return d2.push(x[:, [oz_index]])

    target_rho, target_margin, correct = [], [], 0
    total = 0
    for mid, x in trials.items():
        res = windows(x)[4:]  # skip the onset ramp
        for r in res:
            total += 1
            if r.best_id == mid:
                correct += 1
                target_rho.append(r.best_rho)
                target_margin.append(r.margin)
    rest_res = windows(rest)
    rest_rho = np.array([r.best_rho for r in rest_res]) if rest_res else np.array([0.3])
    rest_margin = np.array([r.margin for r in rest_res]) if rest_res else np.array([0.05])
    acc = correct / max(total, 1)
    t_lo = float(np.percentile(rest_rho, 95))
    t_hi = float(np.percentile(target_rho, 25)) if target_rho else t_lo + 0.1
    theta = float(np.clip(max(t_lo + 0.02, (t_lo + t_hi) / 2), 0.2, 0.7))
    m_lo = float(np.percentile(rest_margin, 90))
    m_hi = float(np.percentile(target_margin, 25)) if target_margin else m_lo + 0.05
    delta = float(np.clip((m_lo + m_hi) / 2, 0.03, 0.25))
    # dwell: more windows when the per-window false-positive rate at rest is higher
    fp = float(np.mean((rest_rho >= theta) & (rest_margin >= delta)))
    N = int(np.clip(3 + round(fp * 20), 3, 8))
    del dec
    return {"rho_threshold": round(theta, 3), "margin_threshold": round(delta, 3), "dwell_windows": N,
            "window_accuracy": round(acc, 3), "rest_false_window_rate": round(fp, 4),
            "rest_rho_p95": round(t_lo, 3), "target_rho_p25": round(t_hi, 3)}


def profile_path(user: str, profile_dir: str = "~/.eyesteer/profiles") -> Path:
    return Path(os.path.expanduser(profile_dir)) / f"{user}.yaml"


def save_profile(user: str, profile: dict, profile_dir: str = "~/.eyesteer/profiles") -> Path:
    p = profile_path(user, profile_dir)
    p.parent.mkdir(parents=True, exist_ok=True)
    p.write_text(yaml.safe_dump({"user": user, **profile}, sort_keys=False))
    return p


def load_profile(user: str, profile_dir: str = "~/.eyesteer/profiles") -> dict | None:
    p = profile_path(user, profile_dir)
    return yaml.safe_load(p.read_text()) if p.exists() else None


def synthetic_calibration(freqs: dict, fs: int = 256, per_marker_s: float = 10.0, rest_s: float = 20.0,
                          snr: float = 1.0, seed: int = 11) -> dict:
    """The same flow driven by synthetic EEG (sim)."""
    from .eeg_sim import SyntheticEEG
    sim = SyntheticEEG(fs=fs, snr=snr, seed=seed)
    trials = {}
    for mid, f in freqs.items():
        sim.set_attention(None)
        sim.generate(fs // 2)
        trials[mid] = sim.generate(int(per_marker_s * fs), f, 0.5)
    rest = sim.generate(int(rest_s * fs), None, 0.0)
    return fit_profile(trials, rest, freqs, fs)
