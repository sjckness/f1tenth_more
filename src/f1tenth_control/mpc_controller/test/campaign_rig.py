"""Closed-loop rig for the M02 campaign fixes (A1-A3): a goal_distance move
past a static obstacle that may only APPEAR once the car is close (R011).

Same nominal-model loop as test_person_pass_closed_loop.py -- the real
MPCController.build_straight_corridor (goal_distance branch),
compute_local_target and solve_mpc_step around f1tenth_state_fcn_dt_beta --
with the campaign's knobs exposed: corridor half-widths, prediction step vs
control period, and (A3) the speed gate and the BLOCKED detector, both
called exactly as MPC_corr.control_loop calls them.

Not a test module (no test_ prefix): imported by test_wide_corridor.py and
test_blocked_closed_loop.py.
"""

import math
from dataclasses import dataclass, field
from typing import List, Optional, Tuple

import numpy as np

from mpc_controller.MPC_corr import MPCController, _project_onto_line
from mpc_controller.blocked import (
    BlockedDetector, BodyBox, first_floor_stage, gate_speed, plan_body_gap,
    plan_keepouts)
from mpc_controller.drive_limits import clamp_drive_speed
from mpc_controller.mpc_solver import shift_warm_start, solve_mpc_step
from mpc_controller.vehicle_model import f1tenth_state_fcn_dt_beta

import test_object_approach_closed_loop as rig

# Footprint for the body-gap metric: the URDF chassis rectangle around the
# rear axle (the MPC state point) -- stack_params' swept_clearance_body_*.
BODY = BodyBox(front=0.443, rear=-0.082, half_width=0.136)
REAR_OVERHANG_M = -BODY.rear
NOSE_M = BODY.front
HALF_WIDTH_M = BODY.half_width


@dataclass
class Obstacle:
    x: float
    y: float
    r: float
    # The obstacle is published only once the car's rear axle is within this
    # straight-line distance of it (inf = visible from the start).
    appear_within_m: float = math.inf


@dataclass
class Config:
    half_width_start: float = 0.60
    half_width_end: float = 0.60
    corr_L_base: float = 3.0
    lookahead_frac: float = 0.85
    period: float = 0.1
    dt: float = 0.2
    horizon: int = 20
    speed: float = 0.4
    goal_distance: float = 5.0
    duration: float = 30.0
    max_forward: float = 0.5
    # A3 on: the operating floor on the command and in the QP, keep-outs,
    # a zero command on an infeasible tick, and the BLOCKED detector -- the
    # same sequence MPC_corr.control_loop runs. Off: the pre-A3 speed path.
    a3: bool = False
    min_moving_speed: float = 0.4
    progress_window_s: float = 2.5
    min_progress_m: float = 0.15
    lock_window_s: float = 2.0
    lock_fraction: float = 0.95
    infeasible_s: float = 1.0


@dataclass
class Result:
    reached_s: Optional[float] = None
    blocked_s: Optional[float] = None
    blocked_reason: Optional[str] = None
    min_body_gap: float = math.inf
    peak_cross_track: float = 0.0
    commanded: List[float] = field(default_factory=list)
    xs: List[Tuple[float, float, float, float]] = field(default_factory=list)
    stalled_s: float = 0.0


def body_gap(state, obstacle) -> float:
    """Distance from the obstacle disk to the car rectangle (negative = contact)."""
    c, s = math.cos(state[2]), math.sin(state[2])
    dx, dy = obstacle.x - state[0], obstacle.y - state[1]
    lon = dx * c + dy * s
    lat = -dx * s + dy * c
    qx = min(max(lon, -REAR_OVERHANG_M), NOSE_M)
    qy = min(max(lat, -HALF_WIDTH_M), HALF_WIDTH_M)
    return math.hypot(lon - qx, lat - qy) - obstacle.r


def make_fake(cfg: Config):
    fake = rig._ObjectMPC(standoff=1.0, speed=cfg.speed)
    fake.corr_wmin = cfg.half_width_start
    fake.corr_wmax = cfg.half_width_end
    fake.corr_L_base = cfg.corr_L_base
    fake.corr_lookahead_frac = cfg.lookahead_frac
    fake.N = cfg.horizon
    fake.ts = cfg.period
    fake.ts_pred = cfg.dt
    fake.goal_distance = cfg.goal_distance
    fake.goal_start_xy = (0.0, 0.0)
    fake.goal_anchor_odom = (0.0, 0.0, 0.0)
    fake.psi_init_corridor = 0.0
    return fake


def run(cfg: Config, obstacles: List[Obstacle]) -> Result:
    fake = make_fake(cfg)
    x = np.zeros(4)
    last_u = np.zeros(2)
    warm = None
    corridor = None
    last_build = None
    seen = [False] * len(obstacles)
    out = Result()
    detector = BlockedDetector(cfg.min_moving_speed, cfg.progress_window_s,
                               cfg.min_progress_m, cfg.lock_window_s,
                               cfg.lock_fraction, cfg.infeasible_s)
    prev_sides = []
    vdes = max(cfg.speed, cfg.min_moving_speed) if cfg.a3 else cfg.speed
    for tick in range(int(round(cfg.duration / cfg.period))):
        t = tick * cfg.period
        for i, ob in enumerate(obstacles):
            if not seen[i] and math.hypot(ob.x - x[0], ob.y - x[1]) <= ob.appear_within_m:
                seen[i] = True
        live = [(ob.x, ob.y, ob.r) for ob, s in zip(obstacles, seen) if s]
        if _project_onto_line((x[0], x[1]), (0.0, 0.0), 0.0) >= cfg.goal_distance:
            out.reached_s = t
            break
        if corridor is None or t - last_build >= 1.0:
            corridor = MPCController.build_straight_corridor(fake, x)
            last_build = t
        corridor['obstacles_world'] = live
        corridor['d_safe'] = rig.DMIN
        corridor['car_radius'] = rig.CAR_RADIUS
        corridor['avoidance_margin'] = rig.AVOIDANCE_MARGIN
        pref_nom = MPCController.compute_local_target(fake, x, corridor)
        kwargs = {}
        ko_plan = None
        if cfg.a3:
            kwargs['v_floor'] = cfg.min_moving_speed
            kwargs['v_floor_from_stage'] = first_floor_stage(
                x[3], last_u[1], cfg.min_moving_speed, rig.LIMITS['a_max'],
                rig.LIMITS['dAMax'], cfg.dt, cfg.period, cfg.horizon)
            ko_plan = plan_keepouts(
                live, corridor, (x[0], x[1]), rig.CAR_RADIUS, rig.AVOIDANCE_MARGIN,
                BODY, cfg.horizon * cfg.dt * vdes, prev_sides)
            prev_sides = [(k.ox, k.oy, k.side) for k in ko_plan.keepouts]
            kwargs['keepouts'] = [k.as_dict() for k in ko_plan.keepouts]
        u0, info = solve_mpc_step(
            x0=x, last_u=last_u, pref_nom=pref_nom, corridor=corridor,
            horizon=cfg.horizon, ts=cfg.dt, params=rig.PARAMS, limits=rig.LIMITS,
            weights=dict(rig.WEIGHTS), obstacles=live, dmin=rig.DMIN,
            vdes=vdes, solver='rti', warm_start_z=warm,
            control_period=cfg.period, **kwargs)
        if info.get('success'):
            warm = shift_warm_start(info.get('zopt'), cfg.horizon,
                                    shift=cfg.period / cfg.dt)
        else:
            warm = None
        v_cmd = x[3] + u0[1] * cfg.period
        steer = float(u0[0])
        if cfg.a3:
            gap = plan_body_gap(info.get('x_pred'), live, BODY)
            feasible = bool(info.get('success')) and not ko_plan.blocked and gap >= 0.0
            v_cmd = gate_speed(v_cmd, cfg.min_moving_speed) if feasible else 0.0
            verdict = detector.update(
                t, x[0], x[1], v_cmd, steer, rig.LIMITS['delta_min'],
                rig.LIMITS['delta_max'], False, 0.0, feasible,
                {'no_side_fits': len(ko_plan.blocked), 'plan_gap': gap,
                 'success': bool(info.get('success'))})
            if verdict is not None:
                out.blocked_s = t
                out.blocked_reason = verdict.reason
                break
        v_cmd, _ = clamp_drive_speed(v_cmd, cfg.max_forward, 0.0)
        out.commanded.append(float(v_cmd))
        # A command under the car's breakaway speed does not move it (R011:
        # 0.09 m/s against the ~0.12-0.14 needed).
        if 0.0 < v_cmd < 0.13 and x[3] < 0.13:
            v_cmd = 0.0
            out.stalled_s += cfg.period
        u_plant = np.array([steer, (v_cmd - x[3]) / cfg.period])
        x = np.array(f1tenth_state_fcn_dt_beta(x, u_plant, cfg.period, rig.WHEELBASE, rig.LR))
        last_u = np.asarray(u0, dtype=float)
        out.xs.append(tuple(float(v) for v in x))
        out.peak_cross_track = max(out.peak_cross_track, abs(x[1]))
        for ob in obstacles:
            out.min_body_gap = min(out.min_body_gap, body_gap(x, ob))
    return out
