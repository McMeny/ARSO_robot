"""SSVEP decoding with canonical correlation analysis (numpy QR/SVD implementation).

Sliding window (default 2.0 s, step 0.25 s), 5-40 Hz band-pass + line notch, references
sin/cos at f, 2f, 3f. Output per window: rho per marker, argmax, margin (best - second best).
"""
from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np
from scipy.signal import butter, iirnotch, sosfiltfilt, tf2sos


def cca_max_corr(X: np.ndarray, Y: np.ndarray) -> float:
    """Largest canonical correlation between column spaces of X (n, p) and Y (n, q)."""
    X = X - X.mean(axis=0)
    Y = Y - Y.mean(axis=0)
    qx, _ = np.linalg.qr(X)
    qy, _ = np.linalg.qr(Y)
    s = np.linalg.svd(qx.T @ qy, compute_uv=False)
    return float(np.clip(s[0], 0.0, 1.0))


def reference_signals(freq: float, n: int, fs: float, harmonics: int = 3, t0: float = 0.0) -> np.ndarray:
    t = t0 + np.arange(n) / fs
    cols = []
    for h in range(1, harmonics + 1):
        cols.append(np.sin(2 * np.pi * h * freq * t))
        cols.append(np.cos(2 * np.pi * h * freq * t))
    return np.stack(cols, axis=1)


@dataclass
class DecodeResult:
    t: float
    rhos: np.ndarray
    argmax: int
    margin: float
    ids: list = field(default_factory=list)

    @property
    def best_id(self):
        return self.ids[self.argmax]

    @property
    def best_rho(self) -> float:
        return float(self.rhos[self.argmax])


class Preprocessor:
    def __init__(self, fs: float, band=(5.0, 40.0), notch: float | None = 60.0):
        self.sos = butter(4, [band[0], band[1]], btype="bandpass", fs=fs, output="sos")
        if notch and notch < fs / 2:
            b, a = iirnotch(notch, 30.0, fs=fs)
            self.sos = np.vstack([self.sos, tf2sos(b, a)])

    def __call__(self, x: np.ndarray) -> np.ndarray:
        return sosfiltfilt(self.sos, x, axis=0, padlen=min(len(x) - 1, 3 * 3 * len(self.sos)))


class SSVEPDecoder:
    """Streaming CCA decoder. push() raw samples (n, channels); returns new DecodeResults."""

    def __init__(self, freqs: list[float], ids: list, fs: float = 256, window_s: float = 2.0,
                 step_s: float = 0.25, harmonics: int = 3, band=(5.0, 40.0), notch: float | None = 60.0):
        self.freqs = list(freqs)
        self.ids = list(ids)
        self.fs = fs
        self.win = int(round(window_s * fs))
        self.step = int(round(step_s * fs))
        self.pre = Preprocessor(fs, band, notch)
        self.refs = [reference_signals(f, self.win, fs, harmonics) for f in self.freqs]
        self.buf = np.zeros((0, 1))
        self.n_seen = 0
        self._next_at = self.win

    def decode_window(self, x: np.ndarray, t: float = 0.0) -> DecodeResult:
        x = np.atleast_2d(x.T).T if x.ndim == 1 else x
        xf = self.pre(x)
        rhos = np.array([cca_max_corr(xf, Y) for Y in self.refs])
        order = np.argsort(rhos)[::-1]
        margin = float(rhos[order[0]] - rhos[order[1]]) if len(rhos) > 1 else float(rhos[0])
        return DecodeResult(t=t, rhos=rhos, argmax=int(order[0]), margin=margin, ids=self.ids)

    def push(self, samples: np.ndarray, t_end: float | None = None) -> list[DecodeResult]:
        samples = np.atleast_2d(samples.T).T if samples.ndim == 1 else samples
        if self.buf.shape[1] != samples.shape[1]:
            self.buf = np.zeros((0, samples.shape[1]))
        self.buf = np.vstack([self.buf, samples])[-self.win * 2:]
        self.n_seen += len(samples)
        out = []
        while self.n_seen >= self._next_at:
            end = len(self.buf) - (self.n_seen - self._next_at)
            seg = self.buf[end - self.win:end]
            if len(seg) == self.win:
                t = (self._next_at / self.fs) if t_end is None else t_end - (self.n_seen - self._next_at) / self.fs
                out.append(self.decode_window(seg, t))
            self._next_at += self.step
        return out
