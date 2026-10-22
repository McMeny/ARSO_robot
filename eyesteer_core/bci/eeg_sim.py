"""Synthetic Muse + Oz EEG (always labeled SIMULATED EEG downstream, D18).

Channels (Muse order): TP9, AF7, AF8, TP10, AUX(=Oz). Units: microvolts, fs = 256 Hz.

Oz:      pink background + alpha bump (stronger when not attending) + 60 Hz line noise
         + SSVEP at f, 2f, 3f (latency ~150 ms, ~0.5 s amplitude ramp) + blink/motion artifacts.
Frontal: pink background + band oscillations whose beta/(alpha+theta) ratio follows engagement.
"""
from __future__ import annotations

import math

import numpy as np
from scipy.signal import lfilter, lfilter_zi

CHANNELS = ["TP9", "AF7", "AF8", "TP10", "Oz"]

# Paul Kellet's economy pink-noise filter (1/f within ~1 dB above 10 Hz at 44 kHz; good enough at 256 Hz)
_PINK_B = np.array([0.049922035, -0.095993537, 0.050612699, -0.004408786])
_PINK_A = np.array([1.0, -2.494956002, 2.017265875, -0.522189400])


class SyntheticEEG:
    def __init__(self, fs: int = 256, snr: float = 1.0, ssvep_amplitude_uv: float = 2.5,
                 latency_s: float = 0.15, ramp_s: float = 0.5, line_freq: float = 60.0,
                 seed: int = 7, background_uv: float = 6.0, artifacts: bool = True):
        self.fs = fs
        self.snr = snr
        self.amp = ssvep_amplitude_uv
        self.latency = latency_s
        self.ramp = ramp_s
        self.line_freq = line_freq
        self.bg = background_uv
        self.artifacts = artifacts
        self.rng = np.random.default_rng(seed)
        self.n = 0                     # samples generated so far
        zi = lfilter_zi(_PINK_B, _PINK_A)
        self._zi = [zi * 0.0 for _ in CHANNELS]
        self._pink_gain = None
        self._target = None            # frequency currently attended (Hz) or None
        self._target_since = 0.0       # time attention started
        self._phase = {}
        self._alpha_phase = self.rng.uniform(0, 2 * math.pi, 5)
        self._band_phase = self.rng.uniform(0, 2 * math.pi, (5, 3))
        self._artifact_until = -1.0
        self._blink_times: list[float] = []
        self.blink_rate_hz = 0.25      # ~15 blinks/min
        self.motion_rate_hz = 0.02
        self._calibrate_pink()

    @property
    def t(self) -> float:
        return self.n / self.fs

    def _calibrate_pink(self):
        x = lfilter(_PINK_B, _PINK_A, np.random.default_rng(0).standard_normal(20000))
        self._pink_gain = 1.0 / np.std(x[2000:])

    def set_attention(self, freq: float | None) -> None:
        if freq != self._target:
            self._target = freq
            self._target_since = self.t

    def generate(self, n: int, attended_freq: float | None = None, engagement: float = 0.0,
                 blink: bool | None = None) -> np.ndarray:
        """Next n samples, shape (n, 5). engagement in [-2, 2] (z-like units)."""
        self.set_attention(attended_freq)
        fs = self.fs
        t = (self.n + np.arange(n)) / fs
        out = np.zeros((n, len(CHANNELS)))
        for c in range(len(CHANNELS)):
            w = self.rng.standard_normal(n)
            y, self._zi[c] = lfilter(_PINK_B, _PINK_A, w, zi=self._zi[c])
            out[:, c] = y * self._pink_gain * self.bg
        # line noise (all channels, small)
        out += 1.5 * np.sin(2 * math.pi * self.line_freq * t)[:, None]

        attending = self._target is not None
        # alpha: strong when idle/eyes relaxed, suppressed when fixating a flicker
        alpha_amp = 4.0 if not attending else 1.5
        for c in range(len(CHANNELS)):
            scale = 1.0 if c == 4 else 0.4
            out[:, c] += scale * alpha_amp * np.sin(2 * math.pi * 10.0 * t + self._alpha_phase[c])

        # SSVEP on Oz
        if attending:
            f = self._target
            onset = self._target_since + self.latency
            env = np.clip((t - onset) / self.ramp, 0.0, 1.0)
            a = self.amp * self.snr
            ph = self._phase.setdefault(f, self.rng.uniform(0, 2 * math.pi))
            for h, rel in ((1, 1.0), (2, 0.5), (3, 0.25)):
                out[:, 4] += env * a * rel * np.sin(2 * math.pi * h * f * t + h * ph)

        # frontal engagement bands: theta 6 Hz, alpha 10 Hz, beta 20 Hz
        e = float(np.clip(engagement, -2.5, 2.5))
        beta = 2.0 * math.exp(0.35 * e)
        slow = 2.0 * math.exp(-0.35 * e)
        for c in range(4):
            out[:, c] += beta * np.sin(2 * math.pi * 20.0 * t + self._band_phase[c, 0])
            out[:, c] += slow * np.sin(2 * math.pi * 6.0 * t + self._band_phase[c, 1])
            out[:, c] += slow * np.sin(2 * math.pi * 10.5 * t + self._band_phase[c, 2])

        if self.artifacts:
            self._add_artifacts(out, t, blink)
        self.n += n
        return out

    def _add_artifacts(self, out, t, blink):
        dur = len(t) / self.fs
        if blink or (blink is None and self.rng.random() < self.blink_rate_hz * dur):
            t0 = t[0] + self.rng.uniform(0, dur)
            self._blink_times.append(t0)
        keep = []
        for tb in self._blink_times:
            g = np.exp(-0.5 * ((t - tb) / 0.08) ** 2)
            out[:, 1] += 120 * g  # AF7
            out[:, 2] += 120 * g  # AF8
            out[:, 4] += 8 * g    # little at Oz
            if tb + 0.5 > t[-1]:
                keep.append(tb)
        self._blink_times = keep
        if self.rng.random() < self.motion_rate_hz * dur:
            self._artifact_until = t[0] + self.rng.uniform(0.2, 0.6)
        if self._artifact_until > t[0]:
            m = t < self._artifact_until
            out[m] += self.rng.standard_normal((m.sum(), out.shape[1])) * 25.0


def gaze_timeline_to_eeg(timeline, duration_s: float, freqs: dict, fs: int = 256, **kw):
    """Offline helper: timeline = [(t_start, t_end, marker_id_or_None, engagement)]."""
    sim = SyntheticEEG(fs=fs, **kw)
    n_total = int(duration_s * fs)
    block = fs // 8
    chunks = []
    while sim.n < n_total:
        tt = sim.t
        target, eng = None, 0.0
        for (a, b, mid, e) in timeline:
            if a <= tt < b:
                target, eng = (freqs.get(mid) if mid is not None else None), e
        chunks.append(sim.generate(min(block, n_total - sim.n), target, eng))
    return np.concatenate(chunks)
