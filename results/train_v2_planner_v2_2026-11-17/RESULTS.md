# Closed-loop test of planner_v2.py and baselines.py, training environment (2026-11-17)

Conditions: default v4.9 config (training skies / ground, 8-16 props, normal machine sizes), props ON, command noise
OFF, 20 seeds 90000-90019 (4 per scenario), start / goal on opposite sides of the work zone. Every controller is called
every 0.2 s of sim time; the sim is paused while it plans. planner_v2.py and baselines.py are used unchanged with
planner_bundle_vjepa2_1_vitl_g2.pt.

Controllers
- straight: go to the goal, no avoidance.
- reactive: stop while the ground-truth machine clearance is < 2 m, else go to the goal.
- reactive_vision: planner_v2 mode="reactive_vision" (danger probe on the current frame, no imagination).
- cvmpc: baselines.CVMPC with ground-truth machine parts as circles (see closed_loop/part_circles_excerpt.py).
- shield: planner_v2 mode="shield".
- mppi: planner_v2 mode="mppi" (v2 defaults: no strafe, logit-mean risk, progress gated by p_now, 128 x 2).

## Summary (20 seeds each)

| | success | machine coll. | prop coll. | fall | stuck (no progress 30 s) | timeout (300 s) | median time to goal |
|---|---|---|---|---|---|---|---|
| straight | 2 | 12 | 6 | 0 | 0 | 0 | 30.4 s |
| reactive (GT < 2 m) | 1 | 4 | 5 | 2 | 8 | 0 | 42.2 s |
| reactive_vision | 3 | 2 | 5 | 4 | 6 | 0 | 49.7 s |
| CVMPC (GT parts) | 6 | 1 | 4 | 3 | 6 | 0 | 43.4 s |
| shield | 8 | 3 | 4 | 2 | 3 | 0 | 48.5 s |
| mppi (v2) | 0 | 4 | 1 | 4 | 5 | 6 | - |

Mean planning time per step on the Spark: shield 62 ms, reactive_vision 41 ms, mppi 306 ms.

## Per seed

| seed | scenario | straight | reactive | reactive_vision | cvmpc | shield | mppi |
|---|---|---|---|---|---|---|---|
| 90000 | excavator_loading | prop | prop | stuck | stuck | machine | stuck |
| 90001 | wheel_loader | prop | fall | fall | stuck | stuck | timeout |
| 90002 | dozer | machine | fall | fall | fall | stuck | machine |
| 90003 | haul_road | prop | prop | prop | prop | prop | stuck |
| 90004 | pit_roller | GOAL | stuck | stuck | stuck | GOAL | machine |
| 90005 | excavator_loading | prop | stuck | stuck | stuck | prop | stuck |
| 90006 | wheel_loader | machine | prop | prop | prop | prop | timeout |
| 90007 | dozer | machine | stuck | prop | machine | GOAL | fall |
| 90008 | haul_road | machine | machine | GOAL | GOAL | GOAL | prop |
| 90009 | pit_roller | machine | stuck | stuck | stuck | stuck | stuck |
| 90010 | excavator_loading | machine | stuck | stuck | fall | GOAL | stuck |
| 90011 | wheel_loader | machine | stuck | stuck | stuck | fall | fall |
| 90012 | dozer | machine | machine | GOAL | GOAL | GOAL | timeout |
| 90013 | haul_road | machine | machine | GOAL | GOAL | GOAL | machine |
| 90014 | pit_roller | machine | GOAL | fall | GOAL | GOAL | fall |
| 90015 | excavator_loading | GOAL | stuck | fall | fall | fall | timeout |
| 90016 | wheel_loader | machine | stuck | machine | GOAL | GOAL | timeout |
| 90017 | dozer | prop | prop | prop | prop | prop | machine |
| 90018 | haul_road | machine | machine | machine | GOAL | machine | timeout |
| 90019 | pit_roller | prop | prop | prop | prop | machine | fall |

## Files
- `episodes.csv`: one row per controller and seed (termination, success, collisions, fall, timeout, time to goal,
  min clearance, stops, planner steps, planning time); `summary.json`: the table above.
- `planner_logs/<controller>/ep_<seed>.json`: every planner step (t, cmd, risk_best, risk_mean, score_best, p_now,
  avoid, ms; CVMPC: clear_goal, avoid).
- `videos/` (not in the repository, shared separately): chase camera | front camera (model input) | top-view map (start S, goal G, naive route, robot trail,
  machines and props at that moment, executed command held for 3 s), real time at 10 fps.

| video | what it shows |
|---|---|
| 90012_shield_success.mp4 | shield reaches the goal (straight-to-goal collides on this seed) |
| 90000_shield_machine_collision.mp4 | shield failure |
| 90018_cvmpc_success.mp4 | CVMPC success where shield collides |
| 90016_mppi_timeout.mp4 | mppi keeps moving without progress until the 300 s limit |
| 90014_reactive_vision_fall_while_stopped.mp4 | fall while standing (zero command + keep-alive wiggle) |
| 90013_straight_machine_collision.mp4 | straight-to-goal collision, for reference |

- `ahmed_adapter.py`: how planner.step / CVMPC.step are called from the collector.
- `closed_loop/part_circles_excerpt.py`: how the machine parts are turned into circles for CVMPC.

Note: when a zero command lasts > 2 s the collector adds a +-0.2 rad/s yaw wiggle every 1 s (keep-alive that stops
the walking policy from tipping over while standing). It applies to all controllers.
