"""BCI pipeline: raw (n, 5) Muse+Oz samples in -> status + selections out.

Used unchanged by the standalone sim and by the ROS ssvep_node.
"""
from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

from .engagement import EngagementTracker
from .intent import IntentDetector, Selection
from .quality import Quality, assess
from .ssvep_cca import DecodeResult, SSVEPDecoder


@dataclass
class BciStatus:
    t: float = 0.0
    ids: list = field(default_factory=list)
    labels: list = field(default_factory=list)
    rhos: list = field(default_factory=list)
    candidate: object = None
    dwell: float = 0.0
    engagement_z: float = 0.0
    quality: str = "NO DATA"
    signal_ok: bool = False
    source: str = "synthetic"
    threshold: float = 0.35
    no_control: bool = True

    def to_dict(self) -> dict:
        return {k: (v.tolist() if isinstance(v, np.ndarray) else v) for k, v in self.__dict__.items()}


class BciPipeline:
    def __init__(self, markers: list, cfg: dict, source: str = "synthetic", channel_map: dict | None = None,
                 profile: dict | None = None):
        """markers: objects with .id, .label, .frequency. cfg: config/bci.yaml contents."""
        self.markers = [m for m in markers if m.frequency > 0]
        self.ids = [m.id for m in self.markers]
        self.labels = [m.label for m in self.markers]
        self.fs = cfg["fs"]
        d = cfg["decoder"]
        self.decoder = SSVEPDecoder([m.frequency for m in self.markers], self.ids, fs=self.fs,
                                    window_s=d["window_s"], step_s=d["step_s"], harmonics=d["harmonics"],
                                    band=tuple(d["bandpass"]), notch=d.get("notch_hz"))
        it = dict(cfg["intent"])
        if profile:
            it.update({k: profile[k] for k in ("rho_threshold", "margin_threshold", "dwell_windows") if k in profile})
        eng = cfg["engagement"]
        self.intent = IntentDetector(it["rho_threshold"], it["margin_threshold"], int(it["dwell_windows"]),
                                     it["refractory_s"], eng["gate"], eng.get("z_required", 0.0),
                                     eng.get("soft_rho_bonus", 0.05))
        self.engagement = EngagementTracker(self.fs, eng["baseline_s"], d["step_s"])
        self.sq = cfg["signal_quality"]
        self.line_freq = d.get("notch_hz") or 60.0
        cm = channel_map or {"TP9": 0, "AF7": 1, "AF8": 2, "TP10": 3, "Oz": 4}
        self.oz = cm["Oz"]
        self.frontal = [cm[k] for k in ("TP9", "AF7", "AF8", "TP10")]
        self.raw = np.zeros((0, 5))
        self.status = BciStatus(ids=self.ids, labels=self.labels, rhos=[0.0] * len(self.ids), source=source,
                                threshold=self.intent.theta)
        self.last_result: DecodeResult | None = None
        self.quality = Quality(False, ["no data"])
        self.force_bad = False

    def push(self, samples: np.ndarray, t_end: float) -> list[Selection]:
        """samples: (n, >=5) in the Muse channel order; t_end: timestamp of the last sample."""
        self.raw = np.vstack([self.raw, samples[:, :5]])[-self.fs * 4:]
        oz = samples[:, [self.oz]]
        results = self.decoder.push(oz, t_end)
        sels = []
        for r in results:
            win = self.raw[-self.fs:]
            self.quality = assess(win, self.fs, self.oz, self.sq["flat_uv"], self.sq["saturation_uv"],
                                  self.sq["max_std_uv"], self.line_freq, self.sq["line_noise_max_ratio"])
            if self.force_bad:
                self.quality = Quality(False, ["stream lost"])
            z = self.engagement.update(self.raw[-2 * self.fs:, self.frontal])
            s = self.intent.update(r, z, self.quality.ok)
            self.last_result = r
            st = self.status
            st.t, st.rhos, st.engagement_z = r.t, [round(float(v), 4) for v in r.rhos], round(z, 3)
            st.candidate, st.dwell = self.intent.candidate, self.intent.dwell_progress
            st.quality, st.signal_ok, st.no_control = self.quality.label, self.quality.ok, self.intent.no_control
            if s is not None:
                sels.append(s)
        return sels
