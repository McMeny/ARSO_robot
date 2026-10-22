#!/usr/bin/env python3
"""outputs/metrics/*.json (Runtime C) + ros_*.json (ROS sim) -> outputs/metrics/summary.md"""
from __future__ import annotations

import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
M = ROOT / "outputs/metrics"
SCN = [("S1", "field_basic"), ("S2", "field_queue"), ("S3", "field_no_control"), ("S4", "field_overrides"),
       ("S5", "supermarket_walker"), ("S6", "supermarket_queue")]


def load(name):
    p = M / f"{name}.json"
    return json.loads(p.read_text(encoding="utf-8")) if p.exists() else None


def fmt_lat(r):
    lat = r.get("detection_latency_s") or []
    return f"{sum(lat) / len(lat):.2f}" if lat else "–"


def main() -> int:
    L = ["# Scenario metrics", "",
         "All EEG in these runs is **SIMULATED** (synthetic SSVEP through the same decoder). Runtime C = standalone "
         "MuJoCo + eyesteer_core + built-in planner; ROS sim = the deployable ROS 2 stack (Nav2, twist_mux, collision "
         "monitor, Muse LSL emulator, ...) against MuJoCo.", ""]
    for runtime, prefix in (("Runtime C (standalone)", ""), ("ROS sim (mujoco backend)", "ros_"), ("ROS HIL (mujoco_hil backend)", "hil_")):
        rows = [(sid, n, load(prefix + n)) for sid, n in SCN]
        if not any(r for _, _, r in rows):
            continue
        L += [f"## {runtime}", "",
              "| ID | scenario | pass | selections (correct/false) | false/min (look-around) | mean latency s | visited (time-to-goal s) "
              "| path m | min clearance m | collisions | CM activations | mux share | odom drift m/m |",
              "|---|---|---|---|---|---|---|---|---|---|---|---|---|"]
        for sid, n, r in rows:
            if not r:
                L.append(f"| {sid} | {n} | not run | | | | | | | | | | |")
                continue
            vis = ", ".join(f"{a['label']} ({a.get('time_to_goal_s', '–')})" for a in r["arrivals"]) or "–"
            mux = ", ".join(f"{k} {v * 100:.0f}%" for k, v in r["mux_share"].items())
            fpm = r.get("false_per_min_lookaround")
            L.append(f"| {sid} | {n} | {'PASS' if r['passed'] else 'FAIL'} | {r['correct_selections']}/{r['false_selections']} | "
                     f"{fpm if fpm is not None else '–'} | {fmt_lat(r)} | {vis} | {r['path_length_m']} | {r['min_clearance_m']} | "
                     f"**{r['collisions']}** | {r['collision_monitor_activations']} | {mux} | {r.get('odom_drift_per_m', '–')} |")
        L.append("")
    t = M / "bci_tradeoff.json"
    if t.exists():
        c = json.loads(t.read_text(encoding="utf-8"))["chosen"]
        L += ["## BCI defaults (bci_tradeoff.png)", "",
              f"θ = {c['theta']}, N = {c['N']} → accuracy {c['accuracy']:.2f}, false activations {c['false_per_min']:.2f}/min, "
              f"median latency {c['latency_s']:.2f} s (synthetic, SNR 0.8; checked at SNR 1.5).", ""]
    for f in sorted(M.glob("golden_*.json")):
        g = json.loads(f.read_text(encoding="utf-8"))
        L += [f"## Full-system golden path: {g['robot']}", "", f"{g['passes']}/{g['runs']} consecutive passes. Latency budget (median):", "",
              "| segment | measured | target |", "|---|---|---|"]
        for k, v in g["latency"].items():
            L.append(f"| {k} | {v['measured']} | {v['target']} |")
        L.append("")
    (M / "summary.md").write_text("\n".join(L) + "\n", encoding="utf-8")
    # stills index rebuilt from the scenario files (parallel renders race on a shared index)
    import yaml
    idx = {}
    for sid, n in SCN:
        scn = yaml.safe_load((ROOT / "config/scenarios" / f"{n}.yaml").read_text(encoding="utf-8"))
        idx[n] = [{"file": f"outputs/stills/{sid}_{n}_{s['name']}.png", "t": s["t"], "caption": s["caption"]}
                  for s in scn.get("stills", []) if (ROOT / f"outputs/stills/{sid}_{n}_{s['name']}.png").exists()]
    (ROOT / "outputs/stills/index.json").write_text(json.dumps(idx, indent=2), encoding="utf-8")
    print(f"[metrics] wrote {M / 'summary.md'}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
