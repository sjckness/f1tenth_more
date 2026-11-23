"""Standstill drift log (``main.py --drift-log PATH``): what scripts/sim/win/hil_drift_probe.py analyses.

Every ``period`` s of sim time one CSV row: the ground truth as published (pose, and the chassis
velocity PhysX reports, in base_link), /odom as published, and the raw joint state (the 4 wheel
drives' angles and velocities, the 2 steering angles) with the targets the controller sent. Next to
the CSV, ``PATH.json``: the physics rate, the joint gains and efforts, the options in use.
"""
from __future__ import annotations

import json
import math

import numpy as np

COLUMNS = ("t gt_x gt_y gt_z gt_yaw gt_vx gt_vy odom_x odom_y odom_yaw odom_v "
           "q_fl q_fr q_rl q_rr w_fl w_fr w_rl w_rr steer_l steer_r tgt_w_rl tgt_w_rr speed timed_out").split()


def _yaw(q) -> float:
    w, x, y, z = (float(v) for v in q)
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


class DriftLog:
    def __init__(self, path: str, period: float, meta: dict) -> None:
        self.path, self.period, self.next = path, float(period), 0.0
        self.fh = open(path, "w", encoding="utf-8")
        self.fh.write(",".join(COLUMNS) + "\n")
        with open(path + ".json", "w", encoding="utf-8") as fh:
            json.dump(meta, fh, indent=1, default=lambda o: np.asarray(o).tolist())

    def update(self, t: float, state, wheel_odom, controller) -> None:
        if t + 1e-9 < self.next:
            return
        self.next = t + self.period
        p, q = state.base_pose()
        v, _w = state.base_twist()
        art = controller.art
        qd = np.asarray(art.get_joint_positions(joint_indices=controller.drive_idx), dtype=float)
        wd = np.asarray(art.get_joint_velocities(joint_indices=controller.drive_idx), dtype=float)
        qs = np.asarray(art.get_joint_positions(joint_indices=controller.steer_idx), dtype=float)
        tgt = controller.last_targets[1] if controller.last_targets is not None else np.zeros(4)
        row = [t, p[0], p[1], p[2], _yaw(q), v[0], v[1], wheel_odom.x, wheel_odom.y, wheel_odom.yaw,
               wheel_odom.v, *qd, *wd, *qs, tgt[2], tgt[3], controller.speed, int(controller.timed_out)]
        self.fh.write(",".join(f"{float(x):.9g}" for x in row) + "\n")

    def close(self) -> None:
        self.fh.close()
