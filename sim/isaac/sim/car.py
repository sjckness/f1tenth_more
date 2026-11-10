"""The F1Tenth-like car: NVIDIA Leatherback referenced (not copied), uniformly scaled.

* ``spawn_car``: reference + uniform scale + start pose + mass + tyre friction; measures the
  scaled geometry back from the stage and checks it against ``car_spec.yaml`` (within 1 %).
* ``CarController``: command watchdog, steering/speed limits, first-order lag + rate limits
  (actuator model, all from config) and NVIDIA's ``AckermannController`` for the joint targets.
* ``CarState``: base_link ground-truth pose/twist from the chassis rigid body.
"""
from __future__ import annotations

import logging
import math
import os
from dataclasses import dataclass

import numpy as np
from pxr import Gf, Usd, UsdGeom, UsdPhysics

from sim.config import SimConfig, get
from sim.scene_builder import bind_physics_material, physics_material
from sim.transforms import make_tf, matrix_to_quat, quat_from_rpy, quat_to_matrix, tf_inv

log = logging.getLogger(__name__)

CAR_XF = "/World/car"
CAR_ROOT = "/World/car/Leatherback"


class GeometryError(RuntimeError):
    """Scaled model geometry does not match car_spec.yaml."""


@dataclass
class CarGeometry:
    """Geometry of the scaled model, measured on the stage (metres)."""

    wheelbase: float
    track: float
    wheel_radius: float
    scale: float
    base_in_world0: np.ndarray        # base_link world pose at spawn (4x4)
    chassis_T_base: np.ndarray        # base_link in chassis body frame (4x4, unscaled)
    chassis_path: str
    total_mass: float


def _world_matrix(prim: Usd.Prim) -> np.ndarray:
    m = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
    return np.array(m).T   # Gf is row-vector convention


def _rigid_tf(m: np.ndarray) -> np.ndarray:
    """Strip scale from a 4x4 matrix (orthonormalise rotation)."""
    out = np.eye(4)
    r = m[:3, :3]
    u, _, vt = np.linalg.svd(r)
    out[:3, :3] = u @ vt
    out[:3, 3] = m[:3, 3]
    return out


def spawn_car(stage: Usd.Stage, cfg: SimConfig, assets_root: str, app) -> CarGeometry:
    """Reference and scale the Leatherback, place base_link at the scene start pose."""
    from isaacsim.core.utils.semantics import add_labels
    from isaacsim.core.utils.stage import add_reference_to_stage

    car, lb = cfg.car, cfg.car["leatherback"]
    s = float(get(lb, "scale"))
    xf = UsdGeom.Xform.Define(stage, CAR_XF)
    xops = UsdGeom.Xformable(xf)
    t_op = xops.AddTranslateOp()
    r_op = xops.AddRotateZOp()
    xops.AddScaleOp().Set(Gf.Vec3f(s, s, s))
    t_op.Set(Gf.Vec3d(0, 0, 0))
    r_op.Set(0.0)
    add_reference_to_stage(f"{assets_root}{get(lb, 'usd')}", CAR_ROOT)
    for _ in range(10):
        app.update()
    bodies = f"{CAR_ROOT}/Rigid_Bodies"

    def wpos(name: str) -> np.ndarray:
        prim = stage.GetPrimAtPath(f"{bodies}/{name}")
        if not prim.IsValid():
            raise GeometryError(f"Leatherback body not found: {bodies}/{name}")
        return _world_matrix(prim)[:3, 3]

    rl, rr = wpos("Wheel_Rear_Left"), wpos("Wheel_Rear_Right")
    fl, fr = wpos("Wheel_Front_Left"), wpos("Wheel_Front_Right")
    bb = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_, UsdGeom.Tokens.render])
    wr = bb.ComputeWorldBound(stage.GetPrimAtPath(f"{bodies}/Wheel_Front_Left")).ComputeAlignedRange()
    radius = float((wr.GetMax()[2] - wr.GetMin()[2]) / 2)
    rear_mid, front_mid = (rl + rr) / 2, (fl + fr) / 2
    wheelbase = float(front_mid[0] - rear_mid[0])
    track = float(((fl[1] - fr[1]) + (rl[1] - rr[1])) / 2)
    base0 = np.array([rear_mid[0], 0.0, rear_mid[2] - radius])   # rear axle centre on the floor, car frame
    log.info("scaled geometry: wheelbase %.4f track %.4f radius %.4f base_link(car frame) %s",
             wheelbase, track, radius, np.round(base0, 4))
    _check_geometry(car, wheelbase, track, radius, s)

    # Place: base_link at start pose. A small clearance avoids initial ground penetration.
    st = cfg.scene["car"]["start"]
    yaw = math.radians(float(st["yaw_deg"]))
    rz = quat_to_matrix(quat_from_rpy(0, 0, yaw))
    clearance = 0.003
    target = np.array([float(st["x"]), float(st["y"]), float(st.get("z", 0.0))])   # z = floor top at the start
    t = target - rz @ base0 + np.array([0, 0, clearance])
    t_op.Set(Gf.Vec3d(*t))
    r_op.Set(math.degrees(yaw))
    base_world = make_tf(target + np.array([0, 0, clearance]), rot=rz)

    chassis_path = f"{CAR_ROOT}/{get(lb, 'chassis_body')}"
    ch_world = _rigid_tf(_world_matrix(stage.GetPrimAtPath(chassis_path)))
    chassis_T_base = tf_inv(ch_world) @ base_world

    total = _set_mass(stage, cfg, chassis_path)
    _tyre_friction(stage, cfg)
    add_labels(stage.GetPrimAtPath(CAR_XF), ["car"], instance_name="class")
    if get(lb, "hide_body_shell"):
        _hide_shell(stage, chassis_path)
    return CarGeometry(wheelbase=wheelbase, track=track, wheel_radius=radius, scale=s, base_in_world0=base_world,
                       chassis_T_base=chassis_T_base, chassis_path=chassis_path, total_mass=total)


def _check_geometry(car: dict, wheelbase: float, track: float, radius: float, s: float) -> None:
    """Assert measured scaled geometry matches car_spec (scaled Leatherback within 1 %, wheelbase vs real)."""
    lb = car["leatherback"]
    checks = {
        "wheelbase vs real": (wheelbase, get(car, "geometry.wheelbase")),
        "wheelbase vs s*leatherback": (wheelbase, s * get(lb, "wheelbase")),
        "track vs s*leatherback": (track, s * get(lb, "track")),
        "wheel_radius vs s*leatherback": (radius, s * get(lb, "wheel_radius")),
    }
    bad = []
    for name, (meas, exp) in checks.items():
        err = abs(meas - exp) / exp
        log.info("geometry check %-30s measured %.4f expected %.4f err %.2f%%", name, meas, exp, 100 * err)
        if err > 0.01:
            bad.append(f"{name}: measured {meas:.4f}, expected {exp:.4f} ({100 * err:.1f} %)")
    for name, meas, real in (("track", track, get(car, "geometry.track_front")),
                             ("wheel_radius", radius, get(car, "geometry.wheel_radius"))):
        log.info("deviation from real car: %s sim %.4f real %.4f (%+.1f%%)", name, meas, real,
                 100 * (meas - real) / real)
    if bad:
        raise GeometryError("; ".join(bad))


def _set_mass(stage: Usd.Stage, cfg: SimConfig, chassis_path: str) -> float:
    """Chassis mass = total (car_spec) - explicit masses of all other bodies."""
    total = float(get(cfg.car, "mass.total"))
    others = 0.0
    for prim in Usd.PrimRange(stage.GetPrimAtPath(CAR_ROOT)):
        if prim.HasAPI(UsdPhysics.RigidBodyAPI) and str(prim.GetPath()) != chassis_path:
            m = UsdPhysics.MassAPI(prim).GetMassAttr().Get() if prim.HasAPI(UsdPhysics.MassAPI) else None
            others += float(m or 0.0)
    chassis_mass = total - others
    if chassis_mass <= 0:
        raise GeometryError(f"total mass {total} kg is below the other bodies' mass {others:.3f} kg")
    UsdPhysics.MassAPI.Apply(stage.GetPrimAtPath(chassis_path)).CreateMassAttr(chassis_mass)
    log.info("mass: chassis %.3f kg + other bodies %.3f kg = %.3f kg", chassis_mass, others, total)
    return total


def _tyre_friction(stage: Usd.Stage, cfg: SimConfig) -> None:
    fr = cfg.car["friction"]
    mat = physics_material(stage, f"{CAR_XF}/tyre_physics", get(fr, "static"), get(fr, "dynamic"),
                           get(fr, "restitution"))
    n = 0
    for prim in Usd.PrimRange(stage.GetPrimAtPath(f"{CAR_ROOT}/Rigid_Bodies")):
        if "Wheel_" in str(prim.GetPath()) and prim.HasAPI(UsdPhysics.CollisionAPI):
            bind_physics_material(prim, mat)
            n += 1
    log.info("tyre friction material bound to %d wheel colliders", n)


def _hide_shell(stage: Usd.Stage, chassis_path: str) -> None:
    for prim in Usd.PrimRange(stage.GetPrimAtPath(chassis_path)):
        if prim.IsA(UsdGeom.Mesh):
            UsdGeom.Imageable(prim).MakeInvisible()
    log.warning("chassis body shell hidden (leatherback.hide_body_shell=true)")


class CarState:
    """base_link ground truth from the chassis rigid body (physics view)."""

    def __init__(self, geom: CarGeometry) -> None:
        from isaacsim.core.prims import SingleRigidPrim

        self.geom = geom
        self.chassis = SingleRigidPrim(geom.chassis_path, name="chassis")

    def initialize(self) -> None:
        self.chassis.initialize()

    def base_pose(self) -> tuple[np.ndarray, np.ndarray]:
        """(position, quaternion wxyz) of base_link in the world (map_sim)."""
        p, q = self.chassis.get_world_pose()
        t = make_tf(np.asarray(p, dtype=float), np.asarray(q, dtype=float)) @ self.geom.chassis_T_base
        return t[:3, 3], matrix_to_quat(t[:3, :3])

    def base_twist(self) -> tuple[np.ndarray, np.ndarray]:
        """(linear, angular) velocity of base_link expressed in base_link."""
        p_ch, q_ch = self.chassis.get_world_pose()
        v = np.asarray(self.chassis.get_linear_velocity(), dtype=float)
        w = np.asarray(self.chassis.get_angular_velocity(), dtype=float)
        pos, q = self.base_pose()
        v_base = v + np.cross(w, pos - np.asarray(p_ch, dtype=float))
        r = quat_to_matrix(q)
        return r.T @ v_base, r.T @ w


class RigidBodyImu:
    """IMU computed from the chassis rigid body (specific force + body rates at the IMU frame).

    Fallback for ``isaacsim.sensors.physics`` IMUSensor, which returns NaN readings in some environments
    (observed in office.usd). Call ``update`` every physics step.
    """

    GRAVITY = np.array([0.0, 0.0, -9.81])

    def __init__(self, state: "CarState", imu_in_base: np.ndarray) -> None:
        self.state, self.T = state, imu_in_base
        self.prev_v: np.ndarray | None = None
        self.acc = np.array([0.0, 0.0, 9.81])
        self.gyro = np.zeros(3)
        self.q = np.array([1.0, 0.0, 0.0, 0.0])

    def reset(self) -> None:
        self.prev_v = None

    def update(self, dt: float) -> None:
        p_ch, _ = self.state.chassis.get_world_pose()
        v = np.asarray(self.state.chassis.get_linear_velocity(), dtype=float)
        w = np.asarray(self.state.chassis.get_angular_velocity(), dtype=float)
        pos, q = self.state.base_pose()
        r_base = quat_to_matrix(q)
        r_imu = r_base @ self.T[:3, :3]
        p_imu = pos + r_base @ self.T[:3, 3]
        v_imu = v + np.cross(w, p_imu - np.asarray(p_ch, dtype=float))
        a = np.zeros(3) if self.prev_v is None or dt <= 0 else (v_imu - self.prev_v) / dt
        self.prev_v = v_imu
        self.acc = r_imu.T @ (a - self.GRAVITY)
        self.gyro = r_imu.T @ w
        self.q = matrix_to_quat(r_imu)


class CarController:
    """Ackermann command -> joint targets, with the real car's limits and a simple actuator model."""

    def __init__(self, cfg: SimConfig, geom: CarGeometry) -> None:
        from isaacsim.robot.wheeled_robots.controllers.ackermann_controller import AckermannController

        lim = cfg.car["limits"]
        self.max_left = float(get(lim, "max_steering_left"))
        self.max_right = float(get(lim, "max_steering_right"))
        self.max_speed = float(get(lim, "max_speed"))
        self.max_reverse = float(get(lim, "max_reverse_speed"))
        self.steer_rate = float(get(lim, "steering_rate"))
        self.accel = float(get(lim, "acceleration"))
        self.speed_tau = float(get(lim, "speed_tau"))
        self.steer_tau = float(get(lim, "steering_tau"))
        self.timeout = float(get(lim, "command_timeout"))
        lb = cfg.car["leatherback"]
        self.steer_sign = -1.0 if get(lb, "invert_steering") else 1.0
        self.steering_joints = list(get(lb, "steering_joints"))
        self.drive_joints = list(get(lb, "drive_joints"))
        self.rear_joints = [j for j in self.drive_joints if "Rear" in j]
        self.drive_max_torque = float(lb.get("drive_max_torque", {}).get("value", 0.0) or 0.0)
        self.geom = geom
        self.ctrl = AckermannController(
            "ackermann", wheel_base=geom.wheelbase, track_width=geom.track,
            front_wheel_radius=geom.wheel_radius, back_wheel_radius=geom.wheel_radius,
            max_wheel_velocity=0.0, invert_steering=False,
            # 0 = no clip inside the controller: it would clip the INNER wheel angle, while the real limit
            # (servo clip) applies to the commanded (bicycle) angle, which is clamped in step().
            max_wheel_rotation_angle=0.0,
            max_acceleration=0.0, max_steering_angle_velocity=0.0)
        self.art = None
        self.debug = bool(os.environ.get("SIM_DEBUG_CAR"))
        self._n = 0
        self.reset_state()

    def reset_state(self) -> None:
        self.cmd_speed = 0.0
        self.cmd_steer = 0.0
        self.last_cmd_time = -1e9
        self.speed = 0.0       # actuator-model output
        self.steer = 0.0
        self.timed_out = True
        self.ctrl.reset()

    def initialize(self, articulation) -> None:
        self.art = articulation
        names = list(articulation.dof_names)
        missing = [j for j in self.steering_joints + self.drive_joints if j not in names]
        if missing:
            raise GeometryError(f"joints not found in articulation: {missing}; available: {names}")
        self.steer_idx = np.array([names.index(j) for j in self.steering_joints])
        self.drive_idx = np.array([names.index(j) for j in self.drive_joints])
        self.rear_idx = np.array([names.index(j) for j in self.rear_joints])
        log.info("articulation dofs: %d; steering %s drive %s", len(names), self.steer_idx, self.drive_idx)
        ac = articulation.get_articulation_controller()
        if self.drive_max_torque > 0:
            ac.set_max_efforts(np.full(len(self.drive_idx), self.drive_max_torque), joint_indices=self.drive_idx)
            log.info("wheel drive max torque %.2f N m (car_spec leatherback.drive_max_torque); max efforts now %s",
                     self.drive_max_torque, np.round(np.asarray(ac.get_max_efforts())[self.drive_idx], 3))
        kps, kds = ac.get_gains()
        log.info("joint gains (USD drives): drive kp %s kd %s | steering kp %s kd %s",
                 np.round(np.asarray(kps)[self.drive_idx], 3), np.round(np.asarray(kds)[self.drive_idx], 3),
                 np.round(np.asarray(kps)[self.steer_idx], 3), np.round(np.asarray(kds)[self.steer_idx], 3))

    def command(self, speed: float, steering: float, t: float) -> None:
        self.cmd_speed = float(speed)
        self.cmd_steer = float(steering)
        self.last_cmd_time = t

    def step(self, t: float, dt: float) -> None:
        """Run the watchdog + actuator model and send joint targets (call every physics step)."""
        from isaacsim.core.utils.types import ArticulationAction

        self.timed_out = (t - self.last_cmd_time) > self.timeout
        tgt_v = 0.0 if self.timed_out else float(np.clip(self.cmd_speed, -self.max_reverse, self.max_speed))
        tgt_d = float(np.clip(self.cmd_steer, -self.max_right, self.max_left))
        self.speed = _lag_rate(self.speed, tgt_v, self.speed_tau, self.accel, dt)
        self.steer = _lag_rate(self.steer, tgt_d, self.steer_tau, self.steer_rate, dt)
        act = self.ctrl.forward(np.array([self.steer, 0.0, self.speed, 0.0, dt]))
        if act.joint_positions is None or act.joint_velocities is None:
            return
        self.art.apply_action(ArticulationAction(
            joint_positions=self.steer_sign * np.asarray(act.joint_positions, dtype=float),
            joint_indices=self.steer_idx))
        self.art.apply_action(ArticulationAction(
            joint_velocities=np.asarray(act.joint_velocities, dtype=float), joint_indices=self.drive_idx))
        self._n += 1
        if self.debug and self._n % 30 == 0 and (abs(self.speed) > 1e-3 or abs(self.steer) > 1e-3):
            q = self.art.get_joint_positions(joint_indices=self.steer_idx)
            w = self.art.get_joint_velocities(joint_indices=self.drive_idx)
            log.info("car dbg t=%.2f v=%.3f d=%.4f | wheel angle cmd %s actual %s | wheel vel cmd %s actual %s", t,
                     self.speed, self.steer, np.round(self.steer_sign * np.asarray(act.joint_positions), 4),
                     np.round(np.asarray(q), 4), np.round(np.asarray(act.joint_velocities), 2), np.round(np.asarray(w), 2))

    def rear_wheel_speed(self) -> float:
        """Mean rear wheel surface speed [m/s] from joint velocities (wheel encoder / ERPM analogue)."""
        v = self.art.get_joint_velocities(joint_indices=self.rear_idx)
        return float(np.mean(np.asarray(v, dtype=float))) * self.geom.wheel_radius


def _lag_rate(y: float, target: float, tau: float, rate: float, dt: float) -> float:
    """First-order lag towards ``target`` (time constant ``tau``), then rate limit ``rate``/s."""
    want = target if tau <= 0 else y + (target - y) * (1.0 - math.exp(-dt / tau))
    step = float(np.clip(want - y, -rate * dt, rate * dt)) if rate > 0 else want - y
    return y + step
