"""Excerpt from the simulation code (excavator_proto/scene_kinematics.py, KinematicScene.part_circles): how the
machine parts are given to CVMPC every 0.2 s. Reference only; it runs inside the simulation project."""

class KinematicScene:  # excerpt
    def part_circles(self, t: float) -> dict[str, tuple[float, float, float]]:
        """Machine parts at time t as circles {name: (x, y, radius)} in the env frame, for privileged baselines
        (Ahmed's CVMPC). Same footprints as clearance(); each oriented box becomes a fixed number of circles along its
        long axis (so names stay the same every step), radius covering the box. Absent machines are left out."""
        def circles(name, c, yaw, length, width, n):
            co, si = math.cos(yaw), math.sin(yaw)
            r = math.hypot(width / 2, length / (2 * n))
            return {f"{name}_{i}": (float(c[0] + co * (-length / 2 + (i + 0.5) * length / n)),
                                    float(c[1] + si * (-length / 2 + (i + 0.5) * length / n)), float(r))
                    for i in range(n)}

        sizes = link_sizes(self.geometry)
        uc = self.geometry["undercarriage_size"]
        out = circles("undercarriage", self.base_xy, self.layout["base_yaw"], uc[0], uc[1], 2)
        for name, pose in self.link_poses(t).items():
            yaw = yaw_from_quat(pose["quat"])
            if name == "house":
                length, width, n = sizes["house"][0], sizes["house"][1], 1
            else:
                length = max(math.hypot(*(pose["end"][:2] - pose["start"][:2])), sizes[name][1])
                width, n = sizes[name][1], (3 if name in ("boom", "arm") else 1)
            out.update(circles(name, pose["pos"], yaw, length, width, n))
        ts = self.truck_state_env(t)
        if ts["mode"] != "absent":
            out.update(circles("truck", ts["xy"], ts["yaw"], *self.truck_size[:2], 3))
        out["stockpile"] = (float(self.stockpile_xy[0]), float(self.stockpile_xy[1]), float(self.stockpile_radius))
        if self.vehicles:
            from scenarios import part_obbs
            for vi, st in enumerate(self.vehicle_states(t)):
                if st["mode"] == "absent":
                    continue
                for pi, (name, c, yaw, length, width) in enumerate(part_obbs(st)):
                    out.update(circles(f"v{vi}_{name}{pi}", c, yaw, length, width, 2))
        return out

