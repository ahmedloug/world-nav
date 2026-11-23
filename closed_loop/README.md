# Closed-loop evaluation in Isaac Lab

Runs the world-model planner (and baselines) on the Go2 in the simulated construction sites, with the same seeds for
every controller, and turns each run into a review video. Results of the two tests so far are in [`../results/`](../results/).

The runner plugs into the simulation project (the data collector `excavator_proto/collect_excavator_episode.py`,
Isaac Lab 3.0 / Isaac Sim 6.0.1), which is not in this repository yet, so these scripts do not run on their own.

## Files

| File | What it does |
|---|---|
| `ahmed_adapter.py` | Controllers the collector calls every 0.2 s of sim time (the sim is paused while the planner runs): `AhmedWMController` (planner.py v1, or planner_v2.py with mode shield / mppi / reactive_vision), `CVMPCController` (baselines.CVMPC on ground-truth machine parts), `ReactiveStopController` (stops while the ground-truth clearance is < 2 m). Every planner step is logged. |
| `run_ahmed_eval.sh` | Runs a list of seeds for several controllers (`MODES`), one Isaac instance at a time by default, resumable. `HELDOUT=1` uses the held-out environment, `PROPS=0` removes props. |
| `ahmed_eval_summary.py` | Builds `summary.json` and `episodes.csv` (outcome, time, min clearance, stops, planner time) from a run folder. |
| `review_video.py` | Review video per episode: chase camera \| front camera (model input) \| top-view map (start, goal, naive route, trail, machines and props, executed plan). |
| `clean_stale_carb_shm.sh` | Removes Carbonite shared memory left by killed Isaac processes; without it a new Isaac process can hang at start-up. |
| `part_circles_excerpt.py` | Excerpt of the simulation code: machine parts as circles on the ground, given to CVMPC. |
| `seeds_ahmed20.json` | The 20 test seeds (90000–90019, 4 per scenario). |
| `configs/test_heldout_v1.yaml` | Held-out environment: skies and ground materials never used in the dataset, 14–24 props, excavator scale 1.0–1.35, faster operators. |

## Collector hooks used

| Flag | Effect |
|---|---|
| `--ahmed-planner BUNDLE [--ahmed-version v1\|v2] [--ahmed-mode shield\|mppi\|reactive_vision]` | wraps the naive go-to-goal controller with `AhmedWMController` |
| `--cvmpc` | `CVMPCController`; the collector sets `controller.scene` (kinematic scene with `part_circles(t)`) |
| `--reactive-stop-m 2.0` | `ReactiveStopController`; the collector calls `set_clearance()` with the ground-truth clearance every step |
| `--chase-video` | also records a 480×480 chase camera (3.2 m behind, 1.8 m up) for `review_video.py` |

Every test uses `--behaviour naive --split test`, branching off and command noise off. The naive controller only gives
the goal and the "goal reached" event.

## Running

```bash
export SIM_ROOT=/path/to/simulation          # contains vendor/IsaacLab and excavator_proto/
export VJEPA_REPO=/path/to/vjepa2            # clone of facebookresearch/vjepa2 (for the planner bundle)
# planner.py, planner_v2.py and baselines.py must be importable (repository root or this folder)
MODES="straight reactive shield cvmpc" bash closed_loop/run_ahmed_eval.sh closed_loop/seeds_ahmed20.json runs/train_v2
HELDOUT=1 MODES="shield" bash closed_loop/run_ahmed_eval.sh closed_loop/seeds_ahmed20.json runs/heldout_v2
python closed_loop/ahmed_eval_summary.py runs/train_v2
python closed_loop/review_video.py runs/train_v2/shield/ep_90012
```

Each run folder holds the episode (`metadata.json`, `trajectory.jsonl`, `planner_decisions.json`, `chase.mp4`, frames)
and its log. On a GB10 (DGX Spark) one 20-seed controller takes about 1–3 h.

## Data

The training data is the simulated dataset described in [`../docs/dataset.md`](../docs/dataset.md), generated in
Isaac Lab and stored on the Hugging Face Hub (private for now, may be made public later).
