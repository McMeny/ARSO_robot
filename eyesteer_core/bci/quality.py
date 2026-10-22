"""Signal quality checks: flat-line, saturation, huge variance, excessive line noise -> BAD SIGNAL."""
from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np
from scipy.signal import welch


@dataclass
class Quality:
    ok: bool
    flags: list = field(default_factory=list)
    oz_std: float = 0.0
    line_ratio: float = 0.0

    @property
    def label(self) -> str:
        return "GOOD" if self.ok else "BAD SIGNAL (" + ", ".join(self.flags) + ")"


def assess(x: np.ndarray, fs: float, oz_index: int = 4, flat_uv: float = 0.5, saturation_uv: float = 1500.0,
           max_std_uv: float = 300.0, line_freq: float = 60.0, line_noise_max_ratio: float = 50.0) -> Quality:
    """x: (n, channels) raw window (~1 s). Only the Oz channel gates selections."""
    flags = []
    if len(x) < fs // 4:
        return Quality(False, ["no data"])
    oz = x[:, oz_index]
    sd = float(np.std(oz))
    if sd < flat_uv:
        flags.append("flat-line")
    if np.max(np.abs(oz)) > saturation_uv:
        flags.append("saturation")
    if sd > max_std_uv:
        flags.append("huge variance")
    ratio = 0.0
    if len(oz) >= fs // 2 and line_freq < fs / 2:
        f, p = welch(oz - oz.mean(), fs=fs, nperseg=min(len(oz), int(fs)))
        line = p[(f > line_freq - 1.5) & (f < line_freq + 1.5)].mean()
        ref = p[(f > 5) & (f < 40)].mean() + 1e-12
        ratio = float(line / ref)
        if ratio > line_noise_max_ratio:
            flags.append("line noise")
    return Quality(not flags, flags, sd, ratio)
