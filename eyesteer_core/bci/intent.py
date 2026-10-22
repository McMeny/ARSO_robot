"""Intent detection: selection != intent (D3, anti-Midas-touch).

A selection fires only when argmax == m for N consecutive windows, rho >= theta, margin >= delta,
the engagement gate allows it, and marker m is outside its refractory period. Otherwise NO-CONTROL.
"""
from __future__ import annotations

from dataclasses import dataclass

from .engagement import gate_allows, gate_threshold
from .ssvep_cca import DecodeResult


@dataclass
class Selection:
    marker_id: object
    confidence: float
    t: float
    rho: float
    margin: float


class IntentDetector:
    def __init__(self, rho_threshold: float = 0.35, margin_threshold: float = 0.08, dwell_windows: int = 4,
                 refractory_s: float = 2.0, gate: str = "off", z_required: float = 0.0, soft_bonus: float = 0.05):
        self.theta = rho_threshold
        self.delta = margin_threshold
        self.N = dwell_windows
        self.refractory = refractory_s
        self.gate = gate
        self.z_required = z_required
        self.soft_bonus = soft_bonus
        self.candidate = None
        self.count = 0
        self.last_fired: dict = {}
        self.no_control = True

    def reset(self):
        self.candidate, self.count, self.no_control = None, 0, True

    @property
    def dwell_progress(self) -> float:
        return min(1.0, self.count / self.N) if self.candidate is not None else 0.0

    def update(self, r: DecodeResult, engagement_z: float = 0.0, signal_ok: bool = True) -> Selection | None:
        theta = gate_threshold(self.gate, self.theta, engagement_z, self.soft_bonus)
        ok = (signal_ok and r.best_rho >= theta and r.margin >= self.delta
              and gate_allows(self.gate, engagement_z, self.z_required))
        mid = r.best_id
        if not ok:
            self.reset()
            return None
        if mid == self.candidate:
            self.count += 1
        else:
            self.candidate, self.count = mid, 1
        self.no_control = False
        if self.count >= self.N:
            last = self.last_fired.get(mid, -1e9)
            if r.t - last < self.refractory:
                return None
            self.last_fired[mid] = r.t
            conf = min(1.0, 0.5 * (r.best_rho / max(theta, 1e-6)) + 0.5 * min(1.0, r.margin / max(self.delta, 1e-6)) / 2)
            self.count = 0
            return Selection(marker_id=mid, confidence=round(conf, 3), t=r.t, rho=r.best_rho, margin=r.margin)
        return None
