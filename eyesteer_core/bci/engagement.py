"""Engagement proxy from the Muse's frontal/temporal channels: beta / (alpha + theta), z-scored."""
from __future__ import annotations

from collections import deque

import numpy as np
from scipy.signal import welch

_trapz = getattr(np, "trapezoid", None) or np.trapz


def band_power(x: np.ndarray, fs: float, lo: float, hi: float) -> float:
    f, p = welch(x, fs=fs, nperseg=min(len(x), int(fs)))
    m = (f >= lo) & (f < hi)
    return float(_trapz(p[m], f[m])) if m.any() else 0.0


def engagement_index(x: np.ndarray, fs: float) -> float:
    """x: (n, channels) frontal window. Returns log(beta / (alpha + theta)) averaged over channels."""
    vals = []
    for c in range(x.shape[1]):
        beta = band_power(x[:, c], fs, 13, 30)
        alpha = band_power(x[:, c], fs, 8, 12)
        theta = band_power(x[:, c], fs, 4, 7.5)
        vals.append(np.log((beta + 1e-9) / (alpha + theta + 1e-9)))
    return float(np.mean(vals))


class EngagementTracker:
    """Running baseline (last `baseline_s`), z-score of the current index."""

    def __init__(self, fs: float, baseline_s: float = 30.0, update_s: float = 0.25, window_s: float = 2.0):
        self.fs = fs
        self.history = deque(maxlen=max(4, int(baseline_s / update_s)))
        self.window = int(window_s * fs)
        self.z = 0.0
        self.index = 0.0

    def update(self, frontal_window: np.ndarray) -> float:
        if len(frontal_window) < self.window // 2:
            return self.z
        self.index = engagement_index(frontal_window[-self.window:], self.fs)
        self.history.append(self.index)
        if len(self.history) >= 8:
            h = np.array(self.history)
            sd = h.std()
            self.z = float((self.index - h.mean()) / (sd if sd > 1e-6 else 1.0))
        return self.z


def gate_allows(mode: str, z: float, z_required: float = 0.0) -> bool:
    if mode == "required":
        return z >= z_required
    return True


def gate_threshold(mode: str, base_threshold: float, z: float, bonus: float = 0.05) -> float:
    """'soft' gate: a clearly DISengaged user needs more evidence. One-sided on purpose: lowering the
    threshold for engaged users produced wrong selections at gaze onset (engagement rises exactly while
    the 2 s window still holds a half-formed SSVEP): 2/150 wrong onsets vs 0/150 one-sided."""
    if mode != "soft":
        return base_threshold
    if z < -1.0:
        return base_threshold + bonus
    return base_threshold
