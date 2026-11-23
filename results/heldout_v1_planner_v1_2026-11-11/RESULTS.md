# Closed-loop test of Ahmed's WM + MPPI planner (2026-11-11)

Planner: `planner.py` + `planner_bundle_vjepa2_1_vitl_g2.pt` as received 2026-11-10 (unchanged, incl. vy ±0.25),
called every 0.2 s of sim time through `closed_loop/ahmed_adapter.py`; the sim is paused while it plans.

Conditions: held-out environment `closed_loop/configs/test_heldout_v1.yaml` (skies and ground materials never used in
collection, 14–24 props, excavator scale 1.0–1.35, faster operators), props ON, command noise OFF, 20 seeds
90000–90019 (4 per scenario: excavator_loading, wheel_loader, dozer, haul_road, pit_roller), naive start/goal
(goal on the far side of the work zone).

Controllers: `ahmed` (WM + MPPI), `straight` (naive go-to-goal), `reactive` (zero command while the GROUND-TRUTH machine
clearance < 2 m, else go to goal).

| | success | machine collision | prop collision | fall | pit fall | timeout (no progress) | median time to goal | median min clearance* |
|---|---|---|---|---|---|---|---|---|
| ahmed | 3 | 8 | 6 | 2 | 1 | 0 | 53.6 s | 0.19 m |
| straight | 4 | 9 | 7 | 0 | 0 | 0 | 39.8 s | −0.02 m |
| reactive | 2 | 2 | 7 | 1 | 0 | 8 | 53.9 s | 1.42 m |

\* 2-D footprint clearance to the nearest machine part or the stockpile, with the boom / arm projected to the ground,
so walking under the boom gives a negative value without a contact (straight 90000: −0.35 m under the boom, goal reached).

Per episode: `episodes.csv` (+ `summary.json`); per planner step: `planner_logs/ep_*_planner_decisions.json`
(t, cmd, risk_best, risk_mean, ms). Review videos (not in the repository, shared separately) in `videos/` (left: chase camera | middle: Go2 front camera, the
model input | right: top-view map with start S, goal G, naive route, robot trail, machines and props at that moment,
and the executed command held for 3 s), real time at 10 fps:

| file | what it shows |
|---|---|
| 90005_ahmed_success.mp4 | success in 53.6 s, min clearance 0.41 m (straight-to-goal hits a prop on this seed) |
| 90014_ahmed_success.mp4 | success in 38.0 s, min clearance 0.51 m (straight-to-goal hits a machine on this seed) |
| 90000_ahmed_machine_collision.mp4 | collision on a seed where straight-to-goal succeeds |
| 90000_straight_success.mp4 | the straight-to-goal run of the same seed |
| 90002_ahmed_fall.mp4 | fall, with lateral commands just before |
| 90003_ahmed_prop_collision.mp4 | prop collision |
| 90005_reactive_stuck.mp4 | reactive baseline stopping next to a machine until the no-progress timeout |

Integration: `ahmed_adapter.py` (how planner.step is called: every 0.2 s of sim time, frame = front RGB now,
prev_frame = front RGB 0.1 s earlier, pose and goal in the env frame, sim paused while planning). Environment config:
`test_heldout_v1.yaml`. Note: when a zero command lasts > 2 s, the collector adds a ±0.2 rad/s yaw wiggle every 1 s
(keep-alive that stops the Isaac walking policy from tipping over while standing); this affects stops of all controllers.

Observations
- Props: the probe scores machine danger only (danger_H), so prop collisions are not avoided (6/20).
- Falls (2): both had |vy| 0.15–0.17 within the last 1.5 s; 26 % of all planner steps used |vy| > 0.1. In our branch
  data 3-s strafes fall 14–21 % of the time vs ≤ 2 % for other commands.
- Risk saturation: near machines the imagined risk of all samples goes to ~1.0, so the cost difference comes only from
  progress and MPPI keeps going forward (seed 90000 in the default env: risk 0.997–1.0 for the last second before contact).
- Planning time on the Spark (2 runs in parallel): mean 3.2 s per step (p95 4.4 s), vs ~1 s expected.
