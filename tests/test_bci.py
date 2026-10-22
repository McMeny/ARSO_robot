"""M2: BCI core - CCA accuracy, chance level, dwell/Midas-touch, quality, engagement, calibration."""
import numpy as np
import pytest

from eyesteer_core.bci.calibration import fit_profile, synthetic_calibration
from eyesteer_core.bci.eeg_sim import SyntheticEEG
from eyesteer_core.bci.engagement import EngagementTracker, gate_allows, gate_threshold
from eyesteer_core.bci.intent import IntentDetector
from eyesteer_core.bci.pipeline import BciPipeline
from eyesteer_core.bci.quality import assess
from eyesteer_core.bci.ssvep_cca import SSVEPDecoder, cca_max_corr, reference_signals
from eyesteer_core import config
from eyesteer_core.world import Marker

FREQS = [6.923, 7.5, 8.182, 9.0, 11.25, 12.857]
IDS = [1, 2, 3, 4, 5, 6]


def _accuracy(snr, seed=3, trials=36):
    sim = SyntheticEEG(snr=snr, seed=seed)
    dec = SSVEPDecoder(FREQS, IDS)
    correct = total = 0
    for k in range(trials):
        tgt = k % 6
        res = dec.push(sim.generate(256 * 3, FREQS[tgt])[:, [4]])
        for r in res[-3:]:            # windows fully inside the attended period
            total += 1
            correct += r.argmax == tgt
    return correct / total


def test_cca_math_identity():
    Y = reference_signals(8.0, 512, 256)
    x = Y[:, :1] * 3 + 0.5
    assert cca_max_corr(x, Y) > 0.999
    rng = np.random.default_rng(0)
    assert cca_max_corr(rng.standard_normal((512, 1)), Y) < 0.3


def test_cca_high_snr_99_percent():
    assert _accuracy(snr=3.0) >= 0.99


def test_cca_zero_snr_chance():
    acc = _accuracy(snr=0.0, trials=60)
    assert acc < 0.4  # chance = 1/6


def _markers():
    return [Marker(id=i, label=str(i), slot=i, x=0, y=0, frequency=f) for i, f in zip(IDS, FREQS)]


def _run_pipeline(timeline, duration, snr=1.5, seed=5, cfg=None):
    cfg = cfg or config.bci()
    p = BciPipeline(_markers(), cfg)
    sim = SyntheticEEG(snr=snr, seed=seed, artifacts=False)
    fmap = dict(zip(IDS, FREQS))
    sels = []
    block = 32
    while sim.t < duration:
        tgt = None
        for a, b, mid in timeline:
            if a <= sim.t < b:
                tgt = fmap[mid]
        x = sim.generate(block, tgt, 0.5)
        sels += p.push(x, sim.t)
    return sels, p


@pytest.mark.parametrize("snr", [1.0, 1.5])
def test_short_glance_never_fires(snr):
    # 0.75 s glances: far shorter than the minimum selection time (window 2 s + dwell 4 x 0.25 s)
    tl = [(5 + 4 * k, 5.75 + 4 * k, IDS[k % 5]) for k in range(8)]
    for seed in range(4):
        sels, _ = _run_pipeline(tl, 40, snr=snr, seed=seed)
        assert sels == []


def test_sustained_attention_fires_once_then_refractory():
    sels, p = _run_pipeline([(2.0, 4.6, 3)], 9)
    assert [s.marker_id for s in sels] == [3]
    assert 2.0 + 1.25 <= sels[0].t <= 2.0 + 3.0   # latency budget: gaze onset -> selection
    # continuing to look after the selection: refractory + dwell restart -> at most one more per ~2 s
    sels2, _ = _run_pipeline([(2.0, 12.0, 3)], 12)
    ts = [s.t for s in sels2]
    assert all(b - a >= 2.0 for a, b in zip(ts, ts[1:]))


def test_no_control_at_rest():
    sels, p = _run_pipeline([], 60, snr=1.5)
    assert sels == []
    assert p.status.no_control or p.status.dwell < 1.0


def test_intent_requires_margin_and_threshold():
    from eyesteer_core.bci.ssvep_cca import DecodeResult
    det = IntentDetector(0.35, 0.08, 3, 2.0)
    for k in range(5):
        assert det.update(DecodeResult(k * 0.25, np.array([0.5, 0.45]), 0, 0.05, [1, 2])) is None
    for k in range(5):
        assert det.update(DecodeResult(k * 0.25, np.array([0.3, 0.1]), 0, 0.2, [1, 2])) is None
    out = [det.update(DecodeResult(k * 0.25, np.array([0.6, 0.2]), 0, 0.4, [1, 2])) for k in range(3)]
    assert out[-1] is not None and out[-1].marker_id == 1


def test_quality_flags():
    rng = np.random.default_rng(0)
    good = rng.standard_normal((256, 5)) * 10
    assert assess(good, 256).ok
    flat = np.zeros((256, 5))
    assert "flat-line" in assess(flat, 256).flags
    sat = good.copy()
    sat[10, 4] = 5000
    assert "saturation" in assess(sat, 256).flags
    t = np.arange(256) / 256
    line = good + 400 * np.sin(2 * np.pi * 60 * t)[:, None]
    assert "line noise" in assess(line, 256).flags


def test_bad_signal_blocks_selection():
    cfg = config.bci()
    p = BciPipeline(_markers(), cfg)
    sim = SyntheticEEG(snr=3.0, seed=1, artifacts=False)
    sels = []
    while sim.t < 8:
        x = sim.generate(32, FREQS[2])
        x[:, 4] = 0.0  # flat Oz (electrode off)
        sels += p.push(x, sim.t)
    assert sels == [] and not p.status.signal_ok


def test_engagement_tracks_and_gates():
    sim = SyntheticEEG(seed=2, artifacts=False)
    tr = EngagementTracker(256, baseline_s=30)
    zs = []
    for k in range(160):
        eng = 2.0 if k >= 120 else 0.0
        sim.generate(64, None, eng)
        x = sim.generate(512, None, eng)
        zs.append(tr.update(x[:, :4]))
    assert np.mean(zs[-20:]) > 0.8
    assert gate_allows("required", 1.0, 0.0) and not gate_allows("required", -0.5, 0.0)
    assert gate_allows("off", -5) and gate_threshold("soft", 0.35, 1.5) < 0.35 < gate_threshold("soft", 0.35, -1.5)


def test_calibration_profile_reasonable():
    prof = synthetic_calibration(dict(zip(IDS, FREQS)), snr=1.0, per_marker_s=8, rest_s=20)
    assert 0.2 <= prof["rho_threshold"] <= 0.7
    assert 3 <= prof["dwell_windows"] <= 8
    assert prof["window_accuracy"] > 0.9
