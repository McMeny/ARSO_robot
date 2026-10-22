"""S1-S6 in Runtime C (headless, no video), in parallel processes: every expectation must hold, zero
collisions. Also run by deploy/preflight.sh to prove the deployed config passes in sim first."""
import json
import subprocess
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
SCENARIOS = ["field_basic", "field_queue", "field_no_control", "field_overrides", "supermarket_walker", "supermarket_queue"]


@pytest.fixture(scope="module")
def results(tmp_path_factory):
    out = tmp_path_factory.mktemp("scn")
    procs = {s: subprocess.Popen([sys.executable, str(ROOT / "standalone/app/run_scenario.py"), s, "--no-video", "--out", str(out)],
                                 stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True) for s in SCENARIOS}
    logs = {s: p.communicate(timeout=900)[0] for s, p in procs.items()}
    res = {}
    for s in SCENARIOS:
        f = out / "metrics" / f"{s}.json"
        res[s] = (json.loads(f.read_text()) if f.exists() else None, logs[s])
    return res


@pytest.mark.parametrize("scenario", SCENARIOS)
def test_scenario(results, scenario):
    res, log = results[scenario]
    assert res is not None, log[-2000:]
    assert res["collisions"] == 0
    assert res["passed"], res["expectation_failures"]


def test_determinism():
    """Same seed -> identical metrics (selections, arrivals)."""
    outs = []
    for _ in range(2):
        r = subprocess.run([sys.executable, "-c", (
            "import sys; sys.path.insert(0, r'%s');" % ROOT +
            "from eyesteer_core import config; from standalone.app.runner import ScenarioRunner, DT\n"
            "r = ScenarioRunner(config.scenario('field_basic'))\n"
            "[r.step() for _ in range(int(20 / DT))]\n"
            "res = r.finish(); print(res['selections'], res['path_length_m'])")], capture_output=True, text=True, timeout=300)
        outs.append(r.stdout.strip())
    assert outs[0] == outs[1] and outs[0]
