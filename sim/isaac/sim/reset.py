"""Reset: car (pose, velocities, joints, controller, odometry) and person back to their start poses."""
from __future__ import annotations

import logging
import math

import numpy as np
from pxr import Gf, UsdGeom

log = logging.getLogger(__name__)


class Resetter:
    def __init__(self, stage, articulation, controller, wheel_odom, person_path: str | None,
                 person_pose: tuple[np.ndarray, float] | None, person_asset_yaw: float = 0.0) -> None:
        self.stage, self.art, self.ctrl, self.odom = stage, articulation, controller, wheel_odom
        self.person_path, self.person_pose, self.person_asset_yaw = person_path, person_pose, person_asset_yaw
        self.root_pose = articulation.get_world_pose()
        self.joint_pos0 = np.asarray(articulation.get_joint_positions(), dtype=float)
        self.count = 0

    def __call__(self) -> tuple[bool, str]:
        try:
            p, q = self.root_pose
            self.art.set_world_pose(position=p, orientation=q)
            self.art.set_linear_velocity(np.zeros(3))
            self.art.set_angular_velocity(np.zeros(3))
            self.art.set_joint_positions(self.joint_pos0)
            self.art.set_joint_velocities(np.zeros_like(self.joint_pos0))
            self.ctrl.reset_state()
            self.odom.reset()
            if self.person_path and self.person_pose:
                pos, _facing = self.person_pose
                prim = self.stage.GetPrimAtPath(self.person_path)
                xf = UsdGeom.Xformable(prim)
                for op in xf.GetOrderedXformOps():
                    if op.GetOpType() == UsdGeom.XformOp.TypeTranslate:
                        op.Set(Gf.Vec3d(*map(float, pos)))
                    elif op.GetOpType() == UsdGeom.XformOp.TypeRotateZ:
                        op.Set(math.degrees(self.person_asset_yaw))
            self.count += 1
            log.info("reset #%d done", self.count)
            return True, f"reset #{self.count}: car and person at start poses"
        except Exception as exc:  # noqa: BLE001
            log.exception("reset failed")
            return False, f"reset failed: {exc}"
