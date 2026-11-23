"""Review video of one episode, real time (10 fps): chase camera | Go2 front camera | top-view map.

  SIM_ROOT=/path/to/simulation python closed_loop/review_video.py <episode_dir> [--out review.mp4]

Needs chase.mp4 (collector --chase-video) and rgb.mp4. The top bar shows time, the command source (naive follow,
a shield override or the MPC sequence, from trajectory driver_event), the sent command and, with
planner_decisions.json, the predicted danger. The map (env frame, north up) draws the start (S), the goal (G), the
naive route, the robot trail, machines / props at this moment and the plan: the MPC's chosen 3 s sequence (blue) and
its runners-up (light blue), or the shield's held command (orange). Paths are integrated from the commands with
unicycle kinematics, so they show intent, not the walked path.
"""

from __future__ import annotations

import argparse
import json
import math
import subprocess
import sys
from pathlib import Path

import cv2
import numpy as np

HERE = Path(__file__).resolve().parent
_SIM = Path(__import__("os").environ.get("SIM_ROOT", HERE.parent))
sys.path.insert(0, str(_SIM / "excavator_proto"))         # machine footprints come from the simulation code
sys.path.insert(0, str(HERE))
from compute_offline_labels import machine_footprints  # noqa: E402

S = 640           # each panel is S x S
BAR = 64
FUT = 6
COL = {"undercarriage": (70, 70, 70), "house": (0, 200, 240), "boom": (0, 200, 240), "arm": (0, 200, 240),
       "bucket": (0, 140, 255), "truck": (45, 82, 160), "stockpile": (63, 133, 205), "wheel_loader": (32, 165, 218),
       "dozer": (0, 165, 255), "roller": (140, 230, 240), "haul_truck": (180, 130, 70), "excavator_travel": (11, 134, 184)}


def frames(path: Path) -> list[np.ndarray]:
    cap, out = cv2.VideoCapture(str(path)), []
    while True:
        ok, f = cap.read()
        if not ok:
            break
        out.append(f)
    return out


def yaw_of(q) -> float:
    x, y, z, w = q
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def integrate(seq, xy, yaw) -> np.ndarray:
    """Unicycle integration of 0.5 s command steps (same as the planner's progress estimate)."""
    x, y, th, pts = float(xy[0]), float(xy[1]), yaw, [(float(xy[0]), float(xy[1]))]
    for c in seq:
        for _ in range(5):
            x += 0.1 * (c[0] * math.cos(th) - c[1] * math.sin(th))
            y += 0.1 * (c[0] * math.sin(th) + c[1] * math.cos(th))
            th += 0.1 * c[2]
        pts.append((x, y))
    return np.array(pts)


def rect(c, yaw, length, width) -> np.ndarray:
    ca, sa = math.cos(yaw), math.sin(yaw)
    pts = [(-length / 2, -width / 2), (length / 2, -width / 2), (length / 2, width / 2), (-length / 2, width / 2)]
    return np.array([(c[0] + ca * u - sa * v, c[1] + sa * u + ca * v) for u, v in pts])


class Map:
    def __init__(self, meta, rows):
        e = meta["episode"]
        pts = [e["start_xy"], e["goal_xy"], *[r["robot"]["pos"][:2] for r in rows]]
        if e.get("hazard_center"):
            pts.append(e["hazard_center"])
        pts = np.array(pts, float)
        lo, hi = pts.min(0) - 6.0, pts.max(0) + 6.0
        self.c = (lo + hi) / 2
        self.half = max(hi - lo) / 2
        self.e, self.meta = e, meta

    def px(self, xy) -> np.ndarray:
        xy = np.asarray(xy, float).reshape(-1, 2)
        u = (xy[:, 0] - self.c[0]) / self.half * (S / 2 - 10) + S / 2
        v = S / 2 - (xy[:, 1] - self.c[1]) / self.half * (S / 2 - 10)
        return np.stack([u, v], 1).round().astype(np.int32)

    def m2px(self, m: float) -> int:
        return max(2, int(m / self.half * (S / 2 - 10)))

    def poly(self, img, pts, color):
        p = self.px(pts).reshape(-1, 1, 2)
        cv2.fillPoly(img, [p], color, cv2.LINE_AA)
        cv2.polylines(img, [p], True, (40, 40, 40), 1, cv2.LINE_AA)

    def draw(self, row, trail, plan, alts, plan_col) -> np.ndarray:
        img = np.full((S, S, 3), 236, np.uint8)
        e = self.e
        if e.get("road"):
            r = e["road"]
            self.poly(img, rect(r["centre"], r["direction"], r["length"], r["width"]), (205, 205, 205))
        if e.get("pit"):
            pt = e["pit"]
            self.poly(img, rect(pt["centre"], pt["yaw"], pt["length"], pt["width"]), (40, 40, 40))
        for pr in e.get("props") or []:
            if pr.get("shape") == "circle":
                cv2.circle(img, tuple(int(v) for v in self.px(pr["xy"])[0]), self.m2px(pr["radius"]), (90, 120, 150),
                           -1, cv2.LINE_AA)
            else:
                self.poly(img, rect(pr["xy"], pr["yaw"], pr["length"], pr["width"]), (90, 120, 150))
        for name, centre, yaw, length, width in machine_footprints(row, self.meta):
            if length < 0:      # stockpile circle (length = width = -radius)
                cv2.circle(img, tuple(int(v) for v in self.px(centre)[0]), self.m2px(-length), COL["stockpile"], -1,
                           cv2.LINE_AA)
            else:
                self.poly(img, rect(centre, yaw, length, width), COL.get(name, (100, 100, 100)))
        route = np.array([e["start_xy"], *(e.get("waypoints") or [e["goal_xy"]])], float)
        rp = self.px(route)
        for a, b in zip(rp[:-1], rp[1:]):          # naive route, dashed
            n = max(1, int(np.linalg.norm(b - a) / 12))
            for k in range(0, n, 2):
                p0 = a + (b - a) * k / n
                p1 = a + (b - a) * min(k + 1, n) / n
                cv2.line(img, tuple(int(v) for v in p0), tuple(int(v) for v in p1), (150, 150, 150), 2, cv2.LINE_AA)
        if len(trail) > 1:
            cv2.polylines(img, [self.px(trail).reshape(-1, 1, 2)], False, (200, 60, 200), 2, cv2.LINE_AA)
        for alt in alts:
            cv2.polylines(img, [self.px(alt).reshape(-1, 1, 2)], False, (240, 200, 150), 2, cv2.LINE_AA)
        if plan is not None:
            cv2.polylines(img, [self.px(plan).reshape(-1, 1, 2)], False, plan_col, 3, cv2.LINE_AA)
        s, g = (tuple(int(v) for v in self.px(e[k])[0]) for k in ("start_xy", "goal_xy"))
        cv2.circle(img, s, 10, (60, 170, 60), -1, cv2.LINE_AA)
        cv2.putText(img, "S", (s[0] - 6, s[1] + 6), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
        cv2.drawMarker(img, g, (40, 40, 220), cv2.MARKER_STAR, 24, 3, cv2.LINE_AA)
        cv2.putText(img, "G", (g[0] + 12, g[1] - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (40, 40, 220), 2)
        pos, yaw = row["robot"]["pos"][:2], yaw_of(row["robot"]["quat_xyzw"])
        tip = (pos[0] + 1.0 * math.cos(yaw), pos[1] + 1.0 * math.sin(yaw))
        p0 = tuple(int(v) for v in self.px(pos)[0])
        cv2.arrowedLine(img, p0, tuple(int(v) for v in self.px(tip)[0]), (200, 0, 200), 3, cv2.LINE_AA, tipLength=0.45)
        cv2.circle(img, p0, 5, (200, 0, 200), -1, cv2.LINE_AA)
        L = self.m2px(5.0)                         # 5 m scale bar
        cv2.line(img, (S - 20 - L, S - 18), (S - 20, S - 18), (40, 40, 40), 3)
        cv2.putText(img, "5 m", (S - 20 - L, S - 26), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (40, 40, 40), 1)
        return img


def main() -> None:
    p = argparse.ArgumentParser()
    p.add_argument("episode", type=Path)
    p.add_argument("--out", type=Path, default=None)
    args = p.parse_args()
    ep = args.episode
    meta = json.loads((ep / "metadata.json").read_text())
    rows = [json.loads(l) for l in (ep / "trajectory.jsonl").read_text().splitlines()]
    chase, ego = frames(ep / "chase.mp4"), frames(ep / "rgb.mp4")
    dec, pcfg = [], {}
    if (ep / "planner_decisions.json").is_file():
        pcfg = json.loads((ep / "planner_decisions.json").read_text())
        dec = pcfg["decisions"]
    is_mpc = bool(dec) and "n_seq" in dec[0]
    is_ahmed = bool(dec) and ("risk_best" in dec[0] or "p_now" in dec[0])
    ahmed_names = {"shield": "Ahmed WM shield", "mppi": "Ahmed WM + MPPI v2", "reactive_vision": "reactive vision (no imagination)"}
    if pcfg.get("cvmpc"):
        mode = "CVMPC (ground-truth machines)"
    elif dec:
        mode = (ahmed_names.get(pcfg.get("ahmed_mode"), "Ahmed WM + MPPI") if is_ahmed else
                "world-model MPC" if is_mpc else "naive + world model")
    else:
        mode = "reactive stop (GT < 2 m)" if any("reactive" in (r.get("driver_event") or "") for r in rows) else "naive"
    PRIMITIVES = {"forward": (0.7, 0.0, 0.0), "turn_left": (0.3, 0.0, 0.5), "turn_right": (0.3, 0.0, -0.5),
                  "wait": (0.0, 0.0, 0.0), "backward": (-0.3, 0.0, 0.0)}
    goal = meta["episode"]["goal_xy"]
    times = np.array([r["time_s"] for r in rows])
    plans = []                                     # (chosen path, runner-up paths, colour) per decision
    for d in dec:
        row = rows[int(np.abs(times - d["t"]).argmin())]
        pos, yaw = row["robot"]["pos"][:2], yaw_of(row["robot"]["quat_xyzw"])
        if "n_seq" in d:                           # sequence planners that log their top sequences
            seqs = d.get("seqs") or {}
            chosen = seqs.get(d["pick"])
            plans.append((integrate(chosen, pos, yaw) if chosen is not None else None,
                          [integrate(seqs[k], pos, yaw) for k in d.get("top3", [])[1:] if k in seqs], (200, 90, 20)))
        elif "cmd" in d:                           # Ahmed's planners / CVMPC: the executed command held for 3 s
            plans.append((integrate([d["cmd"]] * FUT, pos, yaw), [], (200, 90, 20)))
        elif d.get("pick") in PRIMITIVES:          # shield override: the held primitive for 3 s
            plans.append((integrate([PRIMITIVES[d["pick"]]] * FUT, pos, yaw), [], (0, 140, 255)))
        else:                                      # shield following naive
            plans.append((None, [], (0, 140, 255)))
    world = Map(meta, rows)
    out = args.out or ep / "review.mp4"
    n = min(len(chase), len(ego), len(rows))
    ff = subprocess.Popen(["ffmpeg", "-y", "-loglevel", "error", "-f", "rawvideo", "-pix_fmt", "bgr24", "-s",
                           f"{3 * S}x{S + BAR}", "-r", "10", "-i", "-", "-vcodec", "libx264", "-crf", "20",
                           "-pix_fmt", "yuv420p", str(out)], stdin=subprocess.PIPE)
    di, trail = 0, []
    for i in range(n + 15):                        # hold the last frame 1.5 s with the termination
        j = min(i, n - 1)
        r = rows[j]
        if i < n:
            trail.append(r["robot"]["pos"][:2])
        canvas = np.full((S + BAR, 3 * S, 3), 32, np.uint8)
        canvas[BAR:, :S] = cv2.resize(chase[j], (S, S))
        canvas[BAR:, S:2 * S] = cv2.resize(ego[j], (S, S), interpolation=cv2.INTER_CUBIC)
        while di + 1 < len(dec) and dec[di + 1]["t"] <= r["time_s"]:
            di += 1
        active = bool(dec) and dec[di]["t"] <= r["time_s"]
        plan, alts, pcol = plans[di] if active else (None, [], (0, 0, 0))
        canvas[BAR:, 2 * S:] = world.draw(r, np.array(trail), plan, alts, pcol)
        ev = (r.get("driver_event") or "").split(":")[-1]          # e.g. "naive+wm:override_wait" -> "override_wait"
        if ev == "ahmed_wm" or ev.startswith("cvmpc_"):
            avoid = ev == "cvmpc_avoid" or (ev == "ahmed_wm" and active and bool(dec[di].get("avoid")))
            src, col = ("avoid / stop" if avoid else "go to goal"), ((80, 200, 255) if avoid else (120, 255, 160))
        elif ev == "reactive_stop":
            src, col = "STOP (machine < 2 m)", (80, 200, 255)
        elif ev.startswith("mpc_"):
            src, col = "MPC: " + ev[4:], (120, 255, 160)
        else:
            src = ev.replace("override_", "WM: ") if ev.startswith("override_") else "follow (naive)"
            col = (80, 200, 255) if ev.startswith("override_") else (220, 220, 220)
        cmd = r["command"]
        txt = f"t {r['time_s']:5.1f}s  {mode}  |  {src}  |  cmd vx {cmd[0]:+.2f} vy {cmd[1]:+.2f} wz {cmd[2]:+.2f}"
        cv2.putText(canvas, txt, (12, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.62, col, 2, cv2.LINE_AA)
        line2 = f"seed {meta['seed']}  {meta['scenario']}  |  clearance now {r['contact']['clearance_m']:.2f} m"
        if active and "clear_goal" in dec[di]:
            line2 += f"  |  predicted 3 s clearance of go-to-goal {dec[di]['clear_goal']:.2f} m"
        elif active and "risk_best" in dec[di]:
            line2 += f"  |  imagined risk (go-to-goal) {dec[di]['risk_best']:.2f}  danger now {dec[di].get('p_now', float('nan')):.2f}"
        elif active and "p_now" in dec[di]:
            line2 += f"  |  danger now (probe on current frame) {dec[di]['p_now']:.2f}"
        elif active:
            if dec[di]["danger"].get("follow") is not None:
                line2 += f"  |  predicted danger of naive cmd {dec[di]['danger']['follow']:.2f}"
            if "top3" in dec[di]:
                line2 += f"  |  chosen {dec[di]['danger']['best']:.2f}  top: {', '.join(dec[di]['top3'])}"
        if i >= n - 1:
            line2 += f"  |  END: {meta['termination']}"
        cv2.putText(canvas, line2, (12, 54), cv2.FONT_HERSHEY_SIMPLEX, 0.56, (200, 200, 200), 1, cv2.LINE_AA)
        for k, title in enumerate(("chase camera", "Go2 front camera (model input)",
                                   "top view: S start, G goal, dashed naive route, purple trail, plan")):
            cv2.putText(canvas, title, (k * S + 12, BAR + 24), cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                        (255, 255, 255) if k < 2 else (30, 30, 30), 2 if k < 2 else 1, cv2.LINE_AA)
        ff.stdin.write(canvas.tobytes())
    ff.stdin.close()
    ff.wait()
    print(out)


if __name__ == "__main__":
    main()
