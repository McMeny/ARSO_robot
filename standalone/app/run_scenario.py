#!/usr/bin/env python3
"""Run a scenario in Runtime C: metrics JSON (+ mp4 video and PNG stills unless --no-video)."""
from __future__ import annotations

import argparse
import json
import os
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))

from eyesteer_core import config  # noqa: E402
from standalone.app.runner import DT, ScenarioRunner  # noqa: E402


def check(scn: dict, res: dict) -> list[str]:
    """Scenario expectations -> list of failures (empty = pass)."""
    ex = scn.get("expect", {})
    fails = []
    sel_ids = [s["id"] for s in res["selections"] if s["correct"]]
    if "selections" in ex and sel_ids[:len(ex["selections"])] != ex["selections"]:
        # repeats of the same marker (continued looking) are fine; compare de-duplicated order
        dedup = [i for k, i in enumerate(sel_ids) if k == 0 or sel_ids[k - 1] != i]
        if dedup != ex["selections"]:
            fails.append(f"selections {dedup} != {ex['selections']}")
    visited = [a["id"] for a in res["arrivals"]]
    if "visited" in ex and visited != ex["visited"]:
        fails.append(f"visited {visited} != {ex['visited']}")
    for s in ex.get("states_include", []):
        if s not in res["states_seen"]:
            fails.append(f"state {s} never reached ({res['states_seen']})")
    for s in ex.get("states_exclude", []):
        if s in res["states_seen"]:
            fails.append(f"state {s} must not occur")
    if "final_state" in ex and res["final_state"] != ex["final_state"]:
        fails.append(f"final state {res['final_state']} != {ex['final_state']}")
    if "max_false_selections" in ex and res["false_selections"] > ex["max_false_selections"]:
        fails.append(f"false selections {res['false_selections']} > {ex['max_false_selections']}")
    if res["collisions"] > ex.get("collisions", 0):
        fails.append(f"collisions {res['collisions']}")
    for src in ex.get("sources_seen", []):
        if res["mux_share"].get(src, 0) <= 0:
            fails.append(f"mux source {src} never active")
    if ex.get("collision_monitor_min", 0) > res["collision_monitor_activations"]:
        fails.append("collision monitor never activated")
    return fails


def main(argv=None) -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("scenario")
    ap.add_argument("--no-video", action="store_true")
    ap.add_argument("--true-flicker", action="store_true", help="PHOTOSENSITIVITY WARNING: render real flicker rates")
    ap.add_argument("--out", default=str(ROOT / "outputs"))
    ap.add_argument("--seed", type=int, default=None)
    a = ap.parse_args(argv)
    scn = config.scenario(a.scenario)
    out = Path(a.out)
    hud = None
    if not a.no_video:
        if a.true_flicker:
            print("WARNING: --true-flicker renders flicker; photosensitive-epilepsy risk. 30 fps cannot represent the "
                  "real SSVEP frequencies anyway.", file=sys.stderr)
        from standalone.viz.hud import Hud
        hud = Hud(scn, out, true_flicker=a.true_flicker)
    r = ScenarioRunner(scn, seed=a.seed)
    if hud:
        hud.attach(r)
    n = int(scn["duration"] / DT)
    for _ in range(n):
        r.step()
        if hud:
            hud.frame()
    res = r.finish()
    if hud:
        res["video"], res["stills"] = hud.close()
    res["expectation_failures"] = check(scn, res)
    res["passed"] = not res["expectation_failures"]
    (out / "metrics").mkdir(parents=True, exist_ok=True)
    (out / "metrics" / f"{scn['name']}.json").write_text(json.dumps(res, indent=2))
    print(f"[{scn.get('id', '')} {scn['name']}] {'PASS' if res['passed'] else 'FAIL'} selections="
          f"{[s['label'] for s in res['selections']]} visited={[x['label'] for x in res['arrivals']]} "
          f"states={res['states_seen']} collisions={res['collisions']} minclear={res['min_clearance_m']} "
          + ("" if res["passed"] else f"FAILURES: {res['expectation_failures']}"))
    return 0 if res["passed"] else 1


if __name__ == "__main__":
    sys.exit(main())
