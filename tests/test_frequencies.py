import itertools

import pytest

from compute_frequencies import DEFAULT_RULES, compute


def test_90hz_expected_set():
    t = compute(90.0)
    assert [s["frequency"] for s in t["slots"]] == [6.923, 7.5, 8.182, 9.0, 11.25, 12.857]
    assert t["slots"][-1]["slot"] == "CANCEL" and t["slots"][-1]["frequency"] == 12.857
    assert not t["reduced"]
    for s in t["slots"]:
        assert abs(90.0 / s["frames_per_cycle"] - s["frequency_exact"]) < 1e-12


@pytest.mark.parametrize("R", [72.0, 90.0, 120.0])
def test_rules_hold(R):
    t = compute(R)
    fs = [s["frequency_exact"] for s in t["slots"]]
    assert len(fs) >= 5
    for f in fs:
        assert 6.0 <= f <= 16.0
        assert not (9.5 <= f <= 10.5)
        n = R / f
        assert abs(n - round(n)) < 1e-9
    for a, b in itertools.permutations(fs, 2):
        assert abs(2 * a - b) > DEFAULT_RULES["harmonic_tolerance"]
        assert abs(a - b) >= DEFAULT_RULES["min_spacing"]


def test_hash_changes_with_refresh():
    assert compute(90.0)["hash"] != compute(120.0)["hash"]
    assert compute(90.0)["hash"] == compute(90.0)["hash"]
