"""Controllers for Ahmed's closed-loop request (2026-11-10): his WM + MPPI planner, and a reactive-stop baseline.

Both wrap the naive controller, which only supplies the goal and the "goal reached" event (the planner) or the
go-to-goal command (reactive). Used by the simulation collector excavator_proto/collect_excavator_episode.py
(--ahmed-planner / --ahmed-version / --ahmed-mode / --cvmpc / --reactive-stop-m, see closed_loop/README.md).
The planner modules are imported as planner / planner_v2 / baselines (this repository) or, in the simulation project,
as ahmed_planner / ahmed_planner_v2 / ahmed_baselines. VJEPA_REPO (env) points to a clone of facebookresearch/vjepa2.

AhmedWMController: every 0.2 s of sim time calls ahmed_planner.WMPlanner.step(frame, prev_frame, pose, goal) with the
front RGB, the RGB 0.1 s earlier, the env-frame pose and goal; the command is held until the next call. The sim does
not advance while the planner runs (the collector loop is synchronous). Each call is logged in .decisions.
ReactiveStopController: zero command while the ground-truth machine clearance is below stop_m, else the naive command.
v2 (2026-11-17): version="v2" loads ahmed_planner_v2.py (his planner_v2.py) with mode shield / mppi / reactive_vision;
CVMPCController wraps his baselines.CVMPC, fed every 0.2 s with the ground-truth machine parts as circles
(scene_kinematics.part_circles) through the collector's kinematic scene.
"""

from __future__ import annotations

import importlib
import os
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
VJEPA_REPO = Path(os.environ.get("VJEPA_REPO", HERE.parent / "vendor" / "vjepa2"))


def _module(*names):
    for n in names:
        try:
            return importlib.import_module(n)
        except ModuleNotFoundError:
            continue
    raise ModuleNotFoundError(f"none of {names} found on sys.path")


class AhmedWMController:
    behaviour = "naive+ahmed_wm"

    def __init__(self, inner, bundle_path, device: str = "cuda", period_s: float = 0.2, version: str = "v1",
                 mode: str | None = None):
        sys.path.insert(0, str(HERE))
        sys.path.insert(0, str(HERE.parent))
        if version == "v2":
            WMPlanner = _module("planner_v2", "ahmed_planner_v2").WMPlanner
            kwargs = {"mode": mode} if mode else {}
        else:
            WMPlanner = _module("planner", "ahmed_planner").WMPlanner
            kwargs = {}

        self.inner, self.period = inner, period_s
        self.behaviour = f"naive+ahmed_{version}" + (f"_{mode}" if mode else "")
        self.planner = WMPlanner(str(bundle_path), vjepa_repo=str(VJEPA_REPO), device=device, **kwargs)
        self.planner.reset()
        self.prev, self.cur = None, None
        self.cmd = np.zeros(3, np.float32)
        self.next_t = 0.0
        self.event = "warmup"
        self.decisions: list[dict] = []

    def observe(self, rgb: np.ndarray, proprio, sent_cmd) -> None:     # 10 Hz, after each recorded sample
        self.prev, self.cur = self.cur, np.ascontiguousarray(rgb)

    def command(self, t: float, xy, yaw: float, dt: float):
        follow = self.inner.command(t, xy, yaw, dt)
        if self.inner.event == "goal_reached":
            self.event = "goal_reached"
            return tuple(follow)
        if self.cur is None or self.prev is None:
            self.event = "warmup"
            return (0.0, 0.0, 0.0)
        if t >= self.next_t:
            goal = self.inner.follower.waypoints[-1]
            self.cmd = self.planner.step(self.cur, self.prev, pose=(float(xy[0]), float(xy[1]), float(yaw)),
                                         goal=(float(goal[0]), float(goal[1])))
            last = self.planner.last      # v1: risk_best, risk_mean, ms; v2 also p_now, score_best, avoid
            rec = {"t": round(t, 2), "cmd": [round(float(v), 3) for v in self.cmd]}
            for k, v in last.items():
                if k != "cmd":
                    rec[k] = bool(v) if isinstance(v, (bool, np.bool_)) else round(float(v), 4)
            self.decisions.append(rec)
            self.next_t = t + self.period
        self.event = "ahmed_wm"
        return tuple(float(v) for v in self.cmd)


class CVMPCController:
    """Ahmed's privileged constant-velocity MPC (baselines.py), called every 0.2 s with the true machine parts."""
    behaviour = "naive+cvmpc"

    def __init__(self, inner, period_s: float = 0.2):
        sys.path.insert(0, str(HERE))
        sys.path.insert(0, str(HERE.parent))
        CVMPC = _module("baselines", "ahmed_baselines").CVMPC

        self.inner, self.period = inner, period_s
        self.mpc = CVMPC(dt=period_s)
        self.mpc.reset()
        self.scene = None             # set by the collector (kinematic scene with part_circles(t))
        self.cmd = np.zeros(3, np.float32)
        self.next_t = 0.0
        self.event = "go"
        self.decisions: list[dict] = []

    def command(self, t: float, xy, yaw: float, dt: float):
        follow = self.inner.command(t, xy, yaw, dt)
        if self.inner.event == "goal_reached":
            self.event = "goal_reached"
            return tuple(follow)
        if t >= self.next_t:
            goal = self.inner.follower.waypoints[-1]
            self.cmd = self.mpc.step(pose=(float(xy[0]), float(xy[1]), float(yaw)), goal=(float(goal[0]), float(goal[1])),
                                     parts=self.scene.part_circles(t))
            last = self.mpc.last
            self.decisions.append({"t": round(t, 2), "cmd": [round(float(v), 3) for v in self.cmd],
                                   "clear_goal": round(float(last["clear_goal"]), 3), "avoid": bool(last["avoid"])})
            self.next_t = t + self.period
        self.event = "cvmpc_avoid" if self.decisions and self.decisions[-1]["avoid"] else "cvmpc_go"
        return tuple(float(v) for v in self.cmd)


class ReactiveStopController:
    behaviour = "naive+reactive"

    def __init__(self, inner, stop_m: float = 2.0):
        self.inner, self.stop_m = inner, stop_m
        self.clearance = float("inf")
        self.event = "go"

    def set_clearance(self, clearance_m: float) -> None:              # ground truth, set by the collector every step
        self.clearance = float(clearance_m)

    def command(self, t: float, xy, yaw: float, dt: float):
        follow = self.inner.command(t, xy, yaw, dt)
        if self.inner.event == "goal_reached":
            self.event = "goal_reached"
            return tuple(follow)
        if self.clearance < self.stop_m:
            self.event = "reactive_stop"
            return (0.0, 0.0, 0.0)
        self.event = "go"
        return tuple(follow)
