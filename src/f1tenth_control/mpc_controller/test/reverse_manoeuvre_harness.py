"""
Closed loop for the reverse manoeuvre (reverse_manoeuvre.py), nominal model.

Not a test module. test_reverse_manoeuvre.py runs these scenarios; the node
(MPC_corr) makes the same two supervisor calls around its forward solve.

THE FORWARD SIDE is today's MPC unchanged: the real build_straight_corridor
(goal_distance branch) and compute_local_target on the shipped geometry, and
solve_mpc_step. THE PLANT is the solver's model with a first-order speed
response (tau 0.25 s), so a gear change really takes time to come to rest,
and the /drive clamp (forward 0.5, reverse = max_reverse_speed_mps). THE
WORLD is obstacle disks (what /perception/obstacles_2d and the semantic
memory deliver), a synthetic costmap in a map frame equal to odom, the BT's
permission and the mission state as scripted inputs.
"""

from dataclasses import dataclass, field
import math
import os
import sys
from typing import Callable, Dict, List, Optional, Tuple

from f1tenth_params.object_geometry import SOLVER_FRONT_POINT_OFFSETS_M
from mpc_controller.drive_limits import clamp_drive_speed
from mpc_controller.MPC_corr import MPCController
from mpc_controller.mpc_solver import shift_warm_start, solve_mpc_step
from mpc_controller.reverse_manoeuvre import (
    Grid, ManoeuvreConfig, MODE_FORWARD, RearEnv, RearSolver, ReverseSupervisor)
from mpc_controller.reverse_mpc import rear_probe_offsets
from mpc_controller.vehicle_model import f1tenth_state_fcn_dt_beta
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import reverse_harness as rh  # noqa: E402

TS = rh.TS
TAU = 0.25


def free_grid(x0=-6.0, y0=-4.0, w_m=14.0, h_m=8.0, res=0.05) -> Grid:
    """Return a known, free costmap (value 0) over the test area, map frame."""
    h, w = int(round(h_m / res)), int(round(w_m / res))
    return Grid(data=np.zeros((h, w), dtype=int), resolution=res, origin_x=x0, origin_y=y0)


def mark(grid: Grid, x_range, y_range, value):
    """Set the cells of an axis-aligned box (map frame) to value."""
    i0 = int(math.floor((x_range[0] - grid.origin_x) / grid.resolution))
    i1 = int(math.ceil((x_range[1] - grid.origin_x) / grid.resolution))
    j0 = int(math.floor((y_range[0] - grid.origin_y) / grid.resolution))
    j1 = int(math.ceil((y_range[1] - grid.origin_y) / grid.resolution))
    grid.data[max(j0, 0):j1, max(i0, 0):i1] = value


def shipped_manoeuvre_config(**over) -> ManoeuvreConfig:
    """Build the ManoeuvreConfig MPC_corr builds from stack_params, plus overrides."""
    from f1tenth_params.param_defaults import get_value
    hw = float(get_value('rear_corridor_half_width_m'))
    cfg = {
        'auto_enabled': bool(get_value('reverse_auto_enabled')),
        'solver_mode': str(get_value('rear_solver_mode')),
        'corridor_length_m': float(get_value('rear_corridor_length_m')),
        'corridor_half_width_m': hw if hw > 0.0 else float(get_value('corr_half_width_start_m')),
        'v_rev_mps': float(get_value('v_rev_mps')),
        'rear_dt_s': float(get_value('mpc_rear.prediction_dt_s')),
        'stuck_time_s': float(get_value('stuck_time_s')),
        'stuck_front_dist_m': float(get_value('stuck_front_dist_m')),
        'stuck_progress_m': float(get_value('stuck_progress_m')),
        'gear_change_dwell_s': float(get_value('gear_change_dwell_s')),
        'forward_min_time_s': float(get_value('forward_min_time_s')),
        'max_attempts': int(get_value('reverse_max_attempts')),
        'progress_reset_m': float(get_value('reverse_progress_reset_m')),
        'rear_stop_margin_m': float(get_value('rear_stop_margin_m')),
        'brake_delay_s': float(get_value('rear_brake_delay_s')),
        'brake_decel_mps2': float(get_value('rear_brake_decel_mps2')),
        'rear_check_margin_m': float(get_value('rear_check_margin_m')),
        'car_rear_offset_m': float(get_value('car_rear_offset_m')),
        'car_half_width_m': float(get_value('swept_clearance_body_half_width_m')),
        'stuck_speed_mps': float(get_value('stuck_speed_mps')),
        'bt_allow_max_age_s': float(get_value('bt_allow_reverse_max_age_s')),
        'solver_budget_s': float(get_value('rear_solver_budget_s')),
        'timeout_factor': float(get_value('reverse_timeout_factor')),
        'costmap_free_max': int(get_value('rear_costmap_free_max')),
        'max_reverse_speed_mps': 0.4,
    }
    cfg.update(over)
    return ManoeuvreConfig(**cfg)


def rear_weights() -> Dict:
    from f1tenth_params.param_defaults import get_value
    return {k: float(get_value(f'mpc_rear.{k}')) for k in rh.WEIGHTS}


@dataclass
class Script:
    """Time-varying inputs; each a function of t (seconds)."""

    bt_allow: Callable[[float], Optional[bool]] = lambda t: True
    mission_active: Callable[[float], bool] = lambda t: True
    intentional: Callable[[float], bool] = lambda t: False
    abort: Callable[[float], bool] = lambda t: False
    objects: Callable[[float], List[Tuple[float, float, float]]] = lambda t: []
    # Objects that exist only while a reverse is under way (something that
    # appears behind the car mid-manoeuvre), as a function of the reverse's
    # start pose (x, y, psi).
    objects_in_reverse: Optional[Callable[[Tuple[float, float, float]], List]] = None
    grid: Callable[[float], Optional[Grid]] = None
    # Hand-made forward failures (for the stuck-timer tests), None = real.
    forward_ok: Optional[Callable[[float], bool]] = None


@dataclass
class Run:
    t: List[float] = field(default_factory=list)
    x: List[np.ndarray] = field(default_factory=list)
    v_cmd: List[float] = field(default_factory=list)
    steer: List[float] = field(default_factory=list)
    source: List[str] = field(default_factory=list)
    mode: List[str] = field(default_factory=list)
    status: List[Dict] = field(default_factory=list)
    forward_calls: List[bool] = field(default_factory=list)
    rear_calls: List[int] = field(default_factory=list)
    terminations: List[Tuple[float, str]] = field(default_factory=list)
    tick_ms: List[float] = field(default_factory=list)


def run(script: Script, *, duration=20.0, start=(0.0, 0.0, 0.0, 0.0), goal_distance=6.0,
        walls_half=0.5, front_wall_x: Optional[float] = None, cfg: ManoeuvreConfig = None,
        perturb: Optional[Callable] = None, supervisor_enabled=True) -> Run:
    """
    Drive a goal_distance move under the supervisor; return the record.

    front_wall_x adds a wall across the corridor as the boundary row the
    costmap delivers for it (x <= front_wall_x): the forward MPC cannot pass
    it, which is what being stuck against a wall looks like to this stack.
    Arrival (travelled >= goal_distance) stops the car the way the node does:
    zero speed and an early return that the supervisor does not see.
    """
    import time as _time
    cfg = cfg or shipped_manoeuvre_config()
    fake = rh._Stand()
    fake.goal_distance = goal_distance
    fake.goal_start_xy = (start[0], start[1])
    fake.goal_anchor_odom = (start[0], start[1], start[2])
    fake.psi_init_corridor = start[2]
    rear_off = rear_probe_offsets(rh.WHEELBASE, rh.FRONT_X, rh.REAR_X,
                                  SOLVER_FRONT_POINT_OFFSETS_M)
    rear = RearSolver(cfg, rear_weights(), rh.LIMITS, rh.PARAMS, rear_off,
                      boundary_opts={'boundary_max_sources': 3, 'boundary_hard': False,
                                     'boundary_slack_weight': rh.SLACK_W})
    sup = ReverseSupervisor(cfg, rear, TS)
    walls = [(0.0, 1.0, walls_half), (0.0, -1.0, walls_half)]
    if front_wall_x is not None:
        walls.append((1.0, 0.0, float(front_wall_x)))
    x = np.array(start, dtype=float)
    last_u = np.zeros(2)
    warm = None
    corridor = None
    last_build = -math.inf
    grid_fn = script.grid or (lambda t: free_grid())
    out = Run()
    prev_term = ''
    rev_start = None
    for tick in range(int(round(duration / TS))):
        t = tick * TS
        t0 = _time.perf_counter()
        objs = list(script.objects(t))
        if script.objects_in_reverse is not None and sup.mode == 'reverse':
            if rev_start is None:
                rev_start = (float(x[0]), float(x[1]), float(x[2]))
            objs += list(script.objects_in_reverse(rev_start))
        env = RearEnv(grid=grid_fn(t), map_to_odom=(0.0, 0.0, 0.0), objects=objs,
                      boundaries=walls, bt_allow=script.bt_allow(t),
                      bt_allow_age_s=0.05 if script.bt_allow(t) is not None else math.inf,
                      abort=script.abort(t))
        rear_before = rear.calls
        if (x[0] - start[0]) * math.cos(start[2]) + (x[1] - start[1]) * math.sin(start[2]) \
                >= goal_distance and sup.mode == MODE_FORWARD:
            # Goal reached: the node publishes zero and returns early.
            out.t.append(t)
            out.x.append(x.copy())
            out.v_cmd.append(0.0)
            out.steer.append(float(last_u[0]))
            out.source.append('goal_reached')
            out.mode.append(sup.mode)
            out.status.append(sup.status())
            out.forward_calls.append(False)
            out.rear_calls.append(0)
            out.tick_ms.append(0.0)
            a = (0.0 - x[3]) * min(1.0, TS / TAU) / TS
            x = np.array(f1tenth_state_fcn_dt_beta(x, [last_u[0], a], TS, rh.WHEELBASE, rh.LR))
            continue
        cmd = sup.before_forward(t, x, env) if supervisor_enabled else None
        forward_called = False
        info = {}
        if cmd is None:
            if corridor is None or t - last_build >= 1.0 - 1e-9:
                corridor = MPCController.build_straight_corridor(fake, x)
                last_build = t
            corridor['obstacles_world'] = objs
            corridor['d_safe'] = rh.CAR_RADIUS + rh.AVOIDANCE_MARGIN
            corridor['car_radius'] = rh.CAR_RADIUS
            corridor['avoidance_margin'] = rh.AVOIDANCE_MARGIN
            pref = MPCController.compute_local_target(fake, x, corridor)
            u0, info = solve_mpc_step(
                x0=x, last_u=last_u, pref_nom=pref, corridor=corridor, horizon=rh.N,
                ts=rh.TS_PRED, params=rh.PARAMS, limits=rh.LIMITS, weights=dict(rh.WEIGHTS),
                obstacles=objs, dmin=rh.CAR_RADIUS + rh.AVOIDANCE_MARGIN, vdes=rh.VDES,
                solver='rti', boundaries=walls, boundary_max_sources=3,
                boundary_slack_weight=rh.SLACK_W, warm_start_z=warm, control_period=TS)
            forward_called = True
            ok = bool(info.get('success'))
            if script.forward_ok is not None:
                ok = ok and script.forward_ok(t)
            warm = shift_warm_start(info.get('zopt'), rh.N, TS / rh.TS_PRED) if ok else None
            v_cmd, steer = x[3] + u0[1] * TS, float(u0[0])
            last_u = np.asarray(u0, dtype=float)
            sup.set_steer(steer)
            front = _front_distance(x, objs, front_wall_x)
            cmd2 = sup.after_forward(
                t, x, ok, info.get('x_pred'), script.mission_active(t), script.intentional(t),
                front, env) if supervisor_enabled else None
            source = 'forward'
            if cmd2 is not None:
                v_cmd, steer, source = cmd2.speed, cmd2.steer, cmd2.source
        else:
            v_cmd, steer, source = cmd.speed, cmd.steer, cmd.source
            last_u = np.array([steer, 0.0])
        v_cmd, _ = clamp_drive_speed(v_cmd, rh.MAX_FORWARD, cfg.max_reverse_speed_mps)
        out.tick_ms.append((_time.perf_counter() - t0) * 1e3)
        st = sup.status()
        if st['last_termination_reason'] and st['last_termination_reason'] != prev_term:
            out.terminations.append((t, st['last_termination_reason']))
        prev_term = st['last_termination_reason']
        out.t.append(t)
        out.x.append(x.copy())
        out.v_cmd.append(float(v_cmd))
        out.steer.append(float(steer))
        out.source.append(source)
        out.mode.append(sup.mode)
        out.status.append(st)
        out.forward_calls.append(forward_called)
        out.rear_calls.append(rear.calls - rear_before)
        # The plant: first-order speed, the model for the rest.
        a = (v_cmd - x[3]) * min(1.0, TS / TAU) / TS
        x = np.array(f1tenth_state_fcn_dt_beta(x, [steer, a], TS, rh.WHEELBASE, rh.LR))
        if perturb is not None:
            x = perturb(t, x, sup)
    out.x.append(x.copy())
    return out


def _front_distance(x, objs, front_wall_x=None):
    """Distance from the front bumper to the nearest obstacle or wall ahead (>= 0)."""
    fx = x[0] + rh.FRONT_X * math.cos(x[2])
    fy = x[1] + rh.FRONT_X * math.sin(x[2])
    best = None if front_wall_x is None else max(front_wall_x - fx, 0.0)
    for ox, oy, r in objs:
        dx, dy = ox - fx, oy - fy
        if dx * math.cos(x[2]) + dy * math.sin(x[2]) <= 0.0:
            continue
        d = math.hypot(dx, dy) - r
        best = d if best is None else min(best, d)
    return best


def reverses(r: Run) -> List[Tuple[float, float]]:
    """(start, end) times of each reverse phase (mode 'reverse')."""
    spans, start = [], None
    for t, m in zip(r.t, r.mode):
        if m == 'reverse' and start is None:
            start = t
        elif m != 'reverse' and start is not None:
            spans.append((start, t))
            start = None
    if start is not None:
        spans.append((start, r.t[-1]))
    return spans


__all__ = ['MODE_FORWARD', 'Script', 'free_grid', 'mark', 'reverses', 'run',
           'shipped_manoeuvre_config']
