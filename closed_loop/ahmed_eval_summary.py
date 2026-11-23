"""Summary table for Ahmed's closed-loop request: per controller and per episode, the metrics he asked for.

  python closed_loop/ahmed_eval_summary.py <out_dir of run_ahmed_eval.sh>

Writes <dir>/summary.json and <dir>/episodes.csv. success = goal reached without machine contact; timeout = time budget
or no-progress stop; stops = times the sent command dropped to ~zero for >= 0.5 s; risk_best per planner step comes from
planner_decisions.json (ahmed only).
"""
import csv
import json
import sys
from pathlib import Path

import numpy as np

D = Path(sys.argv[1])
rows_out, summary = [], {}
for mode_dir in sorted(p for p in D.iterdir() if p.is_dir()):
    eps = []
    for ep in sorted(mode_dir.glob("ep_*")):
        if not (ep / "metadata.json").is_file():
            continue
        meta = json.loads((ep / "metadata.json").read_text())
        rows = [json.loads(l) for l in (ep / "trajectory.jsonl").read_text().splitlines()]
        term = meta["termination"]
        cmd = np.array([r["command"] for r in rows])
        still = np.abs(cmd).max(1) < 0.05
        stops, run = 0, 0
        for s in still:
            run = run + 1 if s else 0
            stops += run == 5                                  # 0.5 s at 10 Hz
        dec = []
        if (ep / "planner_decisions.json").is_file():
            dec = json.loads((ep / "planner_decisions.json").read_text())["decisions"]
        e = {"mode": mode_dir.name, "seed": meta["seed"], "scenario": meta["scenario"], "termination": term,
             "success": term == "goal_reached", "machine_collision": term == "machine_collision", "fall": term == "fall",
             "timeout": term in ("budget", "stuck", "env_truncated"), "duration_s": rows[-1]["time_s"],
             "time_to_goal_s": rows[-1]["time_s"] if term == "goal_reached" else None,
             "min_clearance_m": round(min(r["contact"]["clearance_m"] for r in rows), 3), "stops": int(stops),
             "planner_steps": len(dec),
             "risk_best_mean": round(float(np.mean([d["risk_best"] for d in dec])), 3) if dec and "risk_best" in dec[0] else None,
             "planner_ms_mean": round(float(np.mean([d["ms"] for d in dec])), 1) if dec and "ms" in dec[0] else None}
        eps.append(e)
    if not eps:
        continue
    rows_out += eps
    n = len(eps)
    goal_t = [e["time_to_goal_s"] for e in eps if e["time_to_goal_s"] is not None]
    summary[mode_dir.name] = {
        "episodes": n, **{k: sum(e[k] for e in eps) for k in ("success", "machine_collision", "fall", "timeout")},
        "other": sum(not (e["success"] or e["machine_collision"] or e["fall"] or e["timeout"]) for e in eps),
        "median_time_to_goal_s": float(np.median(goal_t)) if goal_t else None,
        "median_min_clearance_m": float(np.median([e["min_clearance_m"] for e in eps])),
        "mean_stops": round(float(np.mean([e["stops"] for e in eps])), 2)}
(D / "summary.json").write_text(json.dumps(summary, indent=1) + "\n")
with open(D / "episodes.csv", "w", newline="") as f:
    w = csv.DictWriter(f, fieldnames=list(rows_out[0]))
    w.writeheader()
    w.writerows(rows_out)
print(json.dumps(summary, indent=1))
