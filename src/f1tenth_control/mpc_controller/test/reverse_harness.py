"""
Closed-loop harness for reverse_mpc: the nominal model, real corridor, real planner.

Not a test module (no test_ prefix). test_reverse_scenarios.py runs the
scenarios and asserts the gates; running this file writes the plots, GIFs and
metrics to output/reverse/:

    python3 test/reverse_harness.py --out ../../../output/reverse

WHAT IS REAL. MPCController.build_straight_corridor (goal_pose and
goal_distance branches) and compute_local_target, bound onto a stand-in that
carries the SHIPPED corridor geometry and weights (stack_params.yaml);
ReversePlanner with its candidates, selector and CostToGo; solve_mpc_step; the
/drive clamp. WHAT IS NOT: the plant is the solver's own model
(f1tenth_state_fcn_dt_beta) tracking the published speed in one tick, as in
test_object_approach_closed_loop.py -- no actuator lag, no slip, no VESC
direction-change behaviour, perfect odometry unless a scenario adds noise.

WALLS. A scenario's walls are segments. The lattice gets them as segments;
the QP gets them as boundary rows the way a wall-row source would deliver
them: one half-plane per segment the car is alongside (its projection falls
inside the segment), nearest three, so an open door is not a row. Door jambs
are also obstacle disks (w_obs), as a detector would report the frame.
"""

import argparse
from dataclasses import dataclass, field
import json
import math
import os
import sys
import time
from typing import Dict, List, Optional, Sequence, Tuple

from f1tenth_description.vehicle_frames import base_front_x
from f1tenth_params.corridor_geometry import CORRIDOR_WIDTH_RATIO
from f1tenth_params.object_geometry import SOLVER_FRONT_POINT_OFFSETS_M
from f1tenth_params.param_defaults import get_value
from mpc_controller.cost_to_go import CostToGo, LatticeConfig
from mpc_controller.drive_limits import clamp_drive_speed
from mpc_controller.MPC_corr import MPCController
from mpc_controller.mpc_solver import shift_warm_start, solve_mpc_step
from mpc_controller.reverse_mpc import rear_probe_offsets, ReverseConfig, ReversePlanner
from mpc_controller.vehicle_model import f1tenth_state_fcn_dt_beta
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import test_object_approach_closed_loop as rig  # noqa: E402

TS = 0.1
N = 20
TS_PRED = float(get_value('mpc_prediction_dt_s'))
WHEELBASE = float(get_value('mpc_wheelbase_m'))
LR = 0.17
PARAMS = {'L': WHEELBASE, 'lr': LR}
LIMITS = {
    'delta_min': float(get_value('mpc_steering_angle_min_rad')),
    'delta_max': float(get_value('mpc_steering_angle_max_rad')),
    'a_min': -2.0, 'a_max': 3.0, 'dDeltaMin': -0.5, 'dDeltaMax': 0.5,
    'dAMin': -2.0, 'dAMax': 2.0, 'vMin': -1.0, 'vMax': 3.0,
}
WEIGHTS = {k: float(get_value(f'mpc_{k}')) for k in (
    'w_term', 'w_v', 'w_psi', 'w_u_a', 'w_du_delta', 'w_du_a', 'w_delta0', 'w_obs',
    'w_corr', 'w_line', 'w_psi_stage')}
CAR_RADIUS = 0.20
AVOIDANCE_MARGIN = 0.12
FRONT_X = base_front_x()
REAR_X = -float(get_value('car_rear_offset_m'))
HALF_W = float(get_value('swept_clearance_body_half_width_m'))
POSE_TOL = 0.15
SLACK_W = float(get_value('boundary_slack_weight'))
VDES = 0.5
MAX_FORWARD = float(get_value('max_forward_speed_mps'))


def shipped_reverse_config(**over) -> ReverseConfig:
    """Build the ReverseConfig MPC_corr._reverse_config builds, from stack_params."""
    cfg = {
        'allow_reverse': True,
        'switch_times_s': tuple(float(t) for t in get_value('reverse_switch_times_s')),
        'v_rev_max': float(get_value('reverse_v_rev_max_mps')),
        'w_rev': float(get_value('reverse_w_rev')),
        'switch_threshold': float(get_value('reverse_switch_threshold')),
        'switch_threshold_rel': float(get_value('reverse_switch_threshold_rel')),
        'commit_break_margin': float(get_value('reverse_commit_break_margin')),
        'min_dwell_s': float(get_value('reverse_min_dwell_s')),
        'stop_speed_mps': float(get_value('reverse_stop_speed_mps')),
        'corr_back_length_m': float(get_value('reverse_corr_back_length_m')),
        'rear_offsets_m': rear_probe_offsets(WHEELBASE, FRONT_X, REAR_X,
                                             SOLVER_FRONT_POINT_OFFSETS_M),
        'w_ctg': float(get_value('reverse_w_ctg')),
        'ctg_follow_m': float(get_value('reverse_ctg_follow_m')),
        'ctg_h_xy': float(get_value('reverse_ctg_h_xy')),
        'ctg_h_psi': float(get_value('reverse_ctg_h_psi')),
    }
    cfg.update(over)
    return ReverseConfig(**cfg)


def shipped_lattice(**over) -> LatticeConfig:
    """Build the LatticeConfig MPC_corr._reverse_cost_to_go builds."""
    cfg = {
        'window_m': float(get_value('reverse_ctg_window_m')),
        'resolution_m': float(get_value('reverse_ctg_resolution_m')),
        'n_theta': int(get_value('reverse_ctg_n_theta')),
        'primitive_length_m': float(get_value('reverse_ctg_primitive_length_m')),
        'k_rev': float(get_value('reverse_ctg_k_rev')),
        'gear_switch_cost': float(get_value('reverse_ctg_gear_switch_cost')),
        'steer_cost_per_m': float(get_value('reverse_ctg_steer_cost_per_m')),
        'unreachable_cost': float(get_value('reverse_ctg_unreachable_cost')),
        'goal_tolerance_m': float(get_value('reverse_ctg_goal_tolerance_m')),
        'footprint_margin_m': float(get_value('reverse_ctg_footprint_margin_m')),
        'front_x': FRONT_X, 'rear_x': REAR_X, 'half_width': HALF_W,
        'wheelbase': WHEELBASE, 'lr': LR,
        'steer_min': LIMITS['delta_min'], 'steer_max': LIMITS['delta_max'],
    }
    cfg.update(over)
    return LatticeConfig(**cfg)


class _Stand(rig._ObjectMPC):
    """The rig's stand-in with the SHIPPED corridor size and solver numbers."""

    def __init__(self):
        super().__init__(standoff=1.0, speed=VDES)
        self.corr_wmin = float(get_value('corr_half_width_start_m'))
        self.corr_wmax = self.corr_wmin * CORRIDOR_WIDTH_RATIO
        self.ts_pred = TS_PRED
        self.params = dict(PARAMS)
        self.limits = dict(LIMITS)


# ------------------------------------------------------------------ geometry

def footprint(x, y, psi, front=None, rear=None, half_w=None):
    """Corners of the car body (base_link at (x, y)), counter-clockwise."""
    front = FRONT_X if front is None else front
    rear = REAR_X if rear is None else rear
    half_w = HALF_W if half_w is None else half_w
    c, s = math.cos(psi), math.sin(psi)
    pts = [(front, half_w), (rear, half_w), (rear, -half_w), (front, -half_w)]
    return np.array([(x + c * a - s * b, y + s * a + c * b) for a, b in pts])


def _seg_dist(p, a, b):
    a, b, p = np.asarray(a, float), np.asarray(b, float), np.asarray(p, float)
    d = b - a
    t = np.clip(((p - a) @ d) / max(d @ d, 1e-12), 0.0, 1.0)
    return float(np.hypot(*(p - (a + t * d))))


def _point_in_poly(p, poly):
    inside = False
    n = len(poly)
    for i in range(n):
        x1, y1 = poly[i]
        x2, y2 = poly[(i + 1) % n]
        if (y1 > p[1]) != (y2 > p[1]):
            if p[0] < x1 + (p[1] - y1) * (x2 - x1) / (y2 - y1):
                inside = not inside
    return inside


def body_clearance(state, walls) -> float:
    """Smallest distance [m] from the car body to any wall; < 0 means overlap."""
    poly = footprint(state[0], state[1], state[2])
    best = math.inf
    edges = [(poly[i], poly[(i + 1) % 4]) for i in range(4)]
    for a, b in walls:
        for e0, e1 in edges:
            for t in np.linspace(0.0, 1.0, 9):
                best = min(best, _seg_dist(e0 + t * (e1 - e0), a, b))
            best = min(best, _seg_dist(a, e0, e1), _seg_dist(b, e0, e1))
        if _point_in_poly(a, poly) or _point_in_poly(b, poly):
            best = min(best, -0.01)
    return best


def wall_rows(walls, x, max_rows=3, rng=3.0):
    """Half-planes (nx, ny, offset) for the segments the car is alongside."""
    p = np.array([x[0], x[1]], dtype=float)
    rows = []
    for a, b in walls:
        a, b = np.asarray(a, float), np.asarray(b, float)
        L = float(np.hypot(*(b - a)))
        if L < 1e-9:
            continue
        t = (b - a) / L
        s = float((p - a) @ t)
        if s < 0.0 or s > L:
            continue
        n = np.array([-t[1], t[0]])
        d = float((a - p) @ n)
        if d < 0.0:
            n, d = -n, -d
        if d > rng:
            continue
        rows.append((d, (float(n[0]), float(n[1]), float(n @ a))))
    rows.sort(key=lambda r: r[0])
    return [r for _, r in rows[:max_rows]]


# ------------------------------------------------------------------ scenarios

@dataclass
class Scenario:
    name: str
    start: Tuple[float, float, float, float]
    goal: Tuple[float, float]                 # pose goal (position only)
    walls: List[Tuple[Tuple[float, float], Tuple[float, float]]] = field(default_factory=list)
    obstacles: List[Tuple[float, float, float]] = field(default_factory=list)
    # Late detection: the car first drives toward pre_goal (or a goal_distance
    # move if pre_goal_distance is set) and the real goal appears once the
    # car's x passes detect_x.
    pre_goal: Optional[Tuple[float, float]] = None
    pre_goal_distance: Optional[float] = None
    detect_x: Optional[float] = None
    goal_distance: Optional[float] = None     # a goal_distance move instead of a pose
    duration: float = 30.0
    odom_noise: Tuple[float, float] = (0.0, 0.0)   # (position m, yaw rad) std
    seed: int = 20261019
    note: str = ''


HALL_HALF = 0.65
DOOR = (2.0, 2.9)          # door opening in the left hall wall, x range
ROOM = (1.0, 4.0, 3.0)     # room x range and far wall y


def door_walls():
    y = HALL_HALF
    return [
        ((-3.0, y), (DOOR[0], y)), ((DOOR[1], y), (9.0, y)),        # left wall, door gap
        ((-3.0, -y), (9.0, -y)),                                      # right wall
        ((ROOM[0], y), (ROOM[0], ROOM[2])), ((ROOM[1], y), (ROOM[1], ROOM[2])),
        ((ROOM[0], ROOM[2]), (ROOM[1], ROOM[2])),                     # the room
    ]


def door_jambs():
    return [(DOOR[0], HALL_HALF, 0.05), (DOOR[1], HALL_HALF, 0.05)]


DOOR_GOAL = (0.5 * (DOOR[0] + DOOR[1]), HALL_HALF + 1.0)


def scenarios() -> Dict[str, Scenario]:
    hall = [((-3.0, 1.0), (9.0, 1.0)), ((-3.0, -1.0), (9.0, -1.0))]
    return {
        'a_free_corridor': Scenario(
            name='a_free_corridor', start=(0.0, 0.0, 0.0, 0.0), goal=(5.0, 0.0),
            walls=hall, duration=20.0,
            note='pose goal 5 m ahead in a 2 m corridor; the cost-to-go is active'),
        'a2_free_corridor_distance': Scenario(
            name='a2_free_corridor_distance', start=(0.0, 0.0, 0.0, 0.0), goal=(5.0, 0.0),
            goal_distance=5.0, walls=hall, duration=20.0,
            note='goal_distance 5 m: no goal pose, so no cost-to-go'),
        'b_door_late': Scenario(
            name='b_door_late', start=(-1.0, 0.0, 0.0, 0.0), goal=DOOR_GOAL,
            pre_goal=(8.0, 0.0), detect_x=2.2, walls=door_walls(), obstacles=door_jambs(),
            duration=40.0,
            note='door seen when base_link is at x 2.2 (body alongside the 0.9 m opening)'),
        'c_door_early': Scenario(
            name='c_door_early', start=(-1.0, 0.0, 0.0, 0.0), goal=DOOR_GOAL,
            pre_goal=(8.0, 0.0), detect_x=0.4, walls=door_walls(), obstacles=door_jambs(),
            duration=40.0, note='same door, seen 1.6 m before the opening'),
        'd_goal_behind_1p5': Scenario(
            name='d_goal_behind_1p5', start=(0.0, 0.0, 0.0, 0.0), goal=(-1.5, 0.0),
            duration=30.0, note='open floor, goal 1.5 m straight behind, from rest'),
        'e_goal_far_behind': Scenario(
            name='e_goal_far_behind', start=(0.0, 0.0, 0.0, 0.0), goal=(-3.5, 0.0),
            duration=45.0, note='open floor, goal 3.5 m straight behind, from rest'),
    }


def near_tie_scenario(distance: float) -> Scenario:
    return Scenario(
        name='f_near_tie', start=(0.0, 0.0, 0.0, 0.0), goal=(-distance, 0.0),
        duration=45.0, odom_noise=(0.01, math.radians(0.5)),
        note=f'goal {distance:.2f} m behind: the distance where forward and reverse '
             'scores are closest at the start; 1 cm / 0.5 deg odometry noise')


# ------------------------------------------------------------------ the loop

@dataclass
class Run:
    scenario: Scenario
    allow_reverse: bool
    t: List[float] = field(default_factory=list)
    x: List[np.ndarray] = field(default_factory=list)
    v_cmd: List[float] = field(default_factory=list)
    steer: List[float] = field(default_factory=list)
    winner: List[Optional[str]] = field(default_factory=list)
    direction: List[str] = field(default_factory=list)
    reason: List[str] = field(default_factory=list)
    scores: List[Dict] = field(default_factory=list)
    tick_ms: List[float] = field(default_factory=list)
    cand_ms: List[float] = field(default_factory=list)
    ctg_build_ms: List[float] = field(default_factory=list)
    clearance: List[float] = field(default_factory=list)
    pred: List[np.ndarray] = field(default_factory=list)
    reached_t: Optional[float] = None
    goal_t: Optional[float] = None


def run_scenario(sc: Scenario, allow_reverse: bool = True, cfg: ReverseConfig = None,
                 lattice: LatticeConfig = None) -> Run:
    """Drive one scenario; return the per-tick record."""
    rng = np.random.default_rng(sc.seed)
    fake = _Stand()
    cfg = cfg or shipped_reverse_config()
    ctg = CostToGo(lattice or shipped_lattice())
    planner = ReversePlanner(cfg, ctg=ctg) if allow_reverse else None
    max_rev = cfg.v_rev_max if allow_reverse else 0.0
    x = np.array(sc.start, dtype=float)
    last_u = np.zeros(2)
    warm = None
    corridor = None
    last_build = -math.inf
    run = Run(scenario=sc, allow_reverse=allow_reverse)
    active = None

    def set_goal(kind, value, t):
        nonlocal corridor, warm
        fake.smoothed_target = None
        fake.deflection_decay_remaining = 0
        corridor = None
        if kind == 'distance':
            fake.goal_pose_xy = None
            fake.goal_distance = float(value)
            fake.goal_start_xy = (float(x[0]), float(x[1]))
            fake.goal_anchor_odom = (float(x[0]), float(x[1]), float(x[2]))
            fake.psi_init_corridor = float(x[2])
        else:
            fake.goal_distance = None
            fake.goal_start_xy = None
            fake.goal_anchor_odom = None
            fake.goal_pose_xy = (float(value[0]), float(value[1]))
        return (kind, value)

    if sc.detect_x is not None:
        active = set_goal('pose', sc.pre_goal, 0.0) if sc.pre_goal_distance is None \
            else set_goal('distance', sc.pre_goal_distance, 0.0)
    elif sc.goal_distance is not None:
        active = set_goal('distance', sc.goal_distance, 0.0)
        run.goal_t = 0.0
    else:
        active = set_goal('pose', sc.goal, 0.0)
        run.goal_t = 0.0

    for tick in range(int(round(sc.duration / TS))):
        t = tick * TS
        if sc.detect_x is not None and run.goal_t is None and x[0] >= sc.detect_x:
            active = set_goal('pose', sc.goal, t)
            run.goal_t = t
        # Arrival (pose: position within POSE_TOL; distance: along the line).
        if active[0] == 'pose' and run.goal_t is not None:
            if math.hypot(x[0] - sc.goal[0], x[1] - sc.goal[1]) <= POSE_TOL:
                run.reached_t = t
                break
        if active[0] == 'distance' and run.goal_t is not None:
            traveled = (x[0] - fake.goal_start_xy[0]) * math.cos(fake.psi_init_corridor) + \
                (x[1] - fake.goal_start_xy[1]) * math.sin(fake.psi_init_corridor)
            if traveled >= fake.goal_distance:
                run.reached_t = t
                break

        meas = x.copy()
        if sc.odom_noise[0] > 0.0:
            meas[0:2] += rng.normal(0.0, sc.odom_noise[0], 2)
            meas[2] += rng.normal(0.0, sc.odom_noise[1])

        if corridor is None or t - last_build >= 1.0 - 1e-9:
            corridor = MPCController.build_straight_corridor(fake, meas)
            last_build = t
        corridor['obstacles_world'] = list(sc.obstacles)
        corridor['d_safe'] = CAR_RADIUS + AVOIDANCE_MARGIN
        corridor['car_radius'] = CAR_RADIUS
        corridor['avoidance_margin'] = AVOIDANCE_MARGIN
        pref_nom = MPCController.compute_local_target(fake, meas, corridor)
        rows = wall_rows(sc.walls, meas)
        kw = {
            'x0': meas, 'last_u': last_u, 'pref_nom': pref_nom, 'corridor': corridor,
            'horizon': N, 'ts': TS_PRED, 'params': PARAMS, 'limits': LIMITS,
            'weights': dict(WEIGHTS), 'obstacles': list(sc.obstacles),
            'dmin': CAR_RADIUS + AVOIDANCE_MARGIN, 'vdes': VDES, 'solver': 'rti',
            'boundaries': rows, 'boundary_max_sources': 3, 'boundary_hard': False,
            'boundary_slack_weight': SLACK_W, 'warm_start_z': warm, 'control_period': TS,
        }
        t0 = time.perf_counter()
        if planner is not None:
            if active[0] == 'pose':
                if ctg.update(t, (meas[0], meas[1]), (active[1][0], active[1][1], None),
                              segments=sc.walls, circles=sc.obstacles):
                    run.ctg_build_ms.append(ctg.field.build_ms)
            else:
                ctg.clear()
            u0, info = planner.step(now_sec=t, **kw)
        else:
            u0, info = solve_mpc_step(**kw)
        tick_ms = (time.perf_counter() - t0) * 1e3
        warm = (shift_warm_start(info.get('zopt'), N, step_fraction=TS / TS_PRED)
                if info.get('success') else None)

        v_cmd, _ = clamp_drive_speed(x[3] + u0[1] * TS, MAX_FORWARD, max_rev)
        dbg = info.get('reverse', {})
        run.t.append(t)
        run.x.append(x.copy())
        run.v_cmd.append(float(v_cmd))
        run.steer.append(float(u0[0]))
        run.winner.append(dbg.get('winner', 'A' if planner is None else None))
        run.direction.append(dbg.get('direction', 'forward'))
        run.reason.append(dbg.get('reason', ''))
        run.scores.append({c['name']: c['score'] for c in dbg.get('candidates', [])})
        run.tick_ms.append(tick_ms)
        run.cand_ms.extend(c['solve_ms'] for c in dbg.get('candidates', [])
                           if c['solve_ms'] is not None)
        run.clearance.append(body_clearance(x, sc.walls) if sc.walls else math.inf)
        run.pred.append(np.asarray(info.get('x_pred', np.zeros((0, 4)))))

        u_plant = np.array([u0[0], (v_cmd - x[3]) / TS])
        x = np.array(f1tenth_state_fcn_dt_beta(x, u_plant, TS, WHEELBASE, LR))
        last_u = np.asarray(u0, dtype=float)
    run.x.append(x.copy())
    run.clearance.append(body_clearance(x, sc.walls) if sc.walls else math.inf)
    return run


# ------------------------------------------------------------------ metrics

def direction_changes(v_cmd: Sequence[float], dead: float = 0.02) -> int:
    """Sign changes of the commanded speed, ignoring |v| <= dead."""
    signs = [1 if v > dead else -1 for v in v_cmd if abs(v) > dead]
    return sum(1 for a, b in zip(signs, signs[1:]) if a != b)


def metrics(run: Run) -> Dict:
    xs = np.array(run.x)
    v = np.array(run.v_cmd)
    steps = np.hypot(np.diff(xs[:, 0]), np.diff(xs[:, 1]))
    rev = xs[1:, 3] < -1e-3
    sc = run.scenario
    final = xs[-1]
    out = {
        'scenario': sc.name, 'allow_reverse': run.allow_reverse,
        'reached': run.reached_t is not None,
        'time_to_goal_s': (None if run.reached_t is None or run.goal_t is None
                           else round(run.reached_t - run.goal_t, 2)),
        'final_dist_to_goal_m': round(
            float(math.hypot(final[0] - sc.goal[0], final[1] - sc.goal[1])), 3),
        'path_m': round(float(steps.sum()), 3),
        'reverse_m': round(float(steps[rev[:len(steps)]].sum()), 3),
        'direction_changes': direction_changes(v),
        'selector_switches': sum(1 for a, b in zip(run.direction, run.direction[1:]) if a != b),
        'min_body_clearance_m': (None if not sc.walls else round(float(min(run.clearance)), 3)),
        'wall_contact': bool(sc.walls) and float(min(run.clearance)) < 0.0,
        'min_cmd_speed_after_cruise': None,
        'tick_ms_p50': round(float(np.percentile(run.tick_ms, 50)), 1),
        'tick_ms_p95': round(float(np.percentile(run.tick_ms, 95)), 1),
        'tick_ms_max': round(float(np.max(run.tick_ms)), 1),
        'cand_ms_p50': (round(float(np.percentile(run.cand_ms, 50)), 1) if run.cand_ms else None),
        'cand_ms_p95': (round(float(np.percentile(run.cand_ms, 95)), 1) if run.cand_ms else None),
        'ctg_builds': len(run.ctg_build_ms),
        'ctg_build_ms_max': (round(float(max(run.ctg_build_ms)), 1) if run.ctg_build_ms else None),
        'ticks': len(run.t),
    }
    cruise = np.flatnonzero(v >= 0.9 * min(VDES, MAX_FORWARD))
    if cruise.size:
        # From first reaching cruise until 1.5 m before the goal (the approach).
        d_goal = np.hypot(xs[:len(v), 0] - sc.goal[0], xs[:len(v), 1] - sc.goal[1])
        window = np.arange(cruise[0], len(v))
        window = window[d_goal[window] > 1.5]
        if window.size:
            out['min_cmd_speed_after_cruise'] = round(float(v[window].min()), 3)
    return out


def find_near_tie(lo=1.6, hi=4.0, step=0.1, cfg=None, lattice=None) -> Tuple[float, List]:
    """Goal distance behind where the best forward and reverse scores are closest at rest."""
    table = []
    for d in np.arange(lo, hi + 1e-9, step):
        sc = near_tie_scenario(float(d))
        sc.duration = 3 * TS
        run = run_scenario(sc, True, cfg=cfg, lattice=lattice)
        s = run.scores[-1]
        fwd = s.get('A')
        rev = min(v for k, v in s.items() if k != 'A' and v is not None)
        table.append((round(float(d), 2), fwd, rev))
    best = min(table, key=lambda r: abs(r[1] - r[2]))
    return best[0], table


# ------------------------------------------------------------------ plots

def _plot(run: Run, path_png: str, path_gif: Optional[str] = None, title: str = ''):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation, PillowWriter
    from matplotlib.patches import Polygon

    sc = run.scenario
    xs = np.array(run.x)
    fig, (ax, ax2) = plt.subplots(1, 2, figsize=(12, 5),
                                  gridspec_kw={'width_ratios': [1.4, 1.0]})
    for a, b in sc.walls:
        ax.plot([a[0], b[0]], [a[1], b[1]], color='#222222', lw=2.5)
    for ox, oy, r in sc.obstacles:
        ax.add_patch(plt.Circle((ox, oy), r, color='#888888'))
    ax.plot(*sc.goal, marker='*', ms=16, color='#2a9d3a', zorder=5, label='goal')
    v = np.array(run.v_cmd)
    for i in range(len(run.v_cmd)):
        col = '#d9480f' if v[i] < -0.02 else '#1f5fbf'
        ax.plot(xs[i:i + 2, 0], xs[i:i + 2, 1], color=col, lw=2)
    every = max(len(run.t) // 14, 1)
    for i in range(0, len(xs), every):
        poly = footprint(*xs[i, :3])
        col = '#d9480f' if i < len(v) and v[i] < -0.02 else '#1f5fbf'
        ax.add_patch(Polygon(poly, closed=True, fill=False, ec=col, lw=0.8, alpha=0.6))
    ax.add_patch(Polygon(footprint(*xs[-1, :3]), closed=True, fill=True, fc='#1f5fbf',
                         alpha=0.35))
    ax.set_aspect('equal')
    pad = 0.8
    allx = np.concatenate([xs[:, 0], [sc.goal[0]]])
    ally = np.concatenate([xs[:, 1], [sc.goal[1]]])
    ax.set_xlim(allx.min() - pad, allx.max() + pad)
    ax.set_ylim(ally.min() - pad, ally.max() + pad)
    ax.set_title(title or sc.name)
    ax.plot([], [], color='#1f5fbf', label='forward')
    ax.plot([], [], color='#d9480f', label='reverse')
    ax.legend(loc='best', fontsize=8)
    ax.grid(alpha=0.3)
    t = np.array(run.t)
    ax2.plot(t, v, color='#333333', label='v_cmd [m/s]')
    ax2.axhline(0, color='#999999', lw=0.8)
    if run.allow_reverse:
        names = sorted({k for s in run.scores for k in s})
        for k in names:
            ax2.plot(t, [s.get(k) if s.get(k) is not None else np.nan for s in run.scores],
                     lw=0.8, label=f'score {k}', alpha=0.8)
        ax2.set_yscale('symlog', linthresh=1.0)
    if run.goal_t is not None:
        ax2.axvline(run.goal_t, color='#2a9d3a', ls='--', lw=0.8, label='goal known')
    ax2.set_xlabel('t [s]')
    ax2.legend(fontsize=7, loc='best')
    ax2.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(path_png, dpi=110)
    plt.close(fig)

    if path_gif is None:
        return
    fig, ax = plt.subplots(figsize=(6, 5))
    for a, b in sc.walls:
        ax.plot([a[0], b[0]], [a[1], b[1]], color='#222222', lw=2.5)
    for ox, oy, r in sc.obstacles:
        ax.add_patch(plt.Circle((ox, oy), r, color='#888888'))
    ax.plot(*sc.goal, marker='*', ms=14, color='#2a9d3a')
    ax.set_aspect('equal')
    ax.set_xlim(allx.min() - pad, allx.max() + pad)
    ax.set_ylim(ally.min() - pad, ally.max() + pad)
    ax.grid(alpha=0.3)
    trail, = ax.plot([], [], color='#1f5fbf', lw=1.5)
    pred, = ax.plot([], [], color='#2a9d3a', lw=1, ls=':')
    body = Polygon(footprint(*xs[0, :3]), closed=True, fc='#1f5fbf', alpha=0.5)
    ax.add_patch(body)
    label = ax.text(0.02, 0.97, '', transform=ax.transAxes, va='top', fontsize=8,
                    family='monospace')
    frames = list(range(0, len(run.t), 2))

    def draw(i):
        trail.set_data(xs[:i + 1, 0], xs[:i + 1, 1])
        p = run.pred[i]
        pred.set_data(p[:, 0] if len(p) else [], p[:, 1] if len(p) else [])
        body.set_xy(footprint(*xs[i, :3]))
        body.set_facecolor('#d9480f' if run.v_cmd[i] < -0.02 else '#1f5fbf')
        label.set_text(f't={run.t[i]:5.1f}s v={run.v_cmd[i]:+.2f}\n'
                       f'{run.direction[i]:8s} {run.winner[i] or "-":5s} {run.reason[i]}')
        return trail, pred, body, label

    anim = FuncAnimation(fig, draw, frames=frames, blit=False)
    anim.save(path_gif, writer=PillowWriter(fps=10), dpi=70)
    plt.close(fig)


def main(argv=None):
    parser = argparse.ArgumentParser(description='Run the reverse scenarios, write plots')
    parser.add_argument('--out', required=True)
    parser.add_argument('--no-gif', action='store_true')
    args = parser.parse_args(argv)
    os.makedirs(args.out, exist_ok=True)
    results = {}
    runs = []
    for name, sc in scenarios().items():
        runs.append((name, run_scenario(sc, True)))
        if name in ('a_free_corridor', 'a2_free_corridor_distance', 'b_door_late'):
            runs.append((name + '__flag_off', run_scenario(sc, False)))
    d_tie, tie_table = find_near_tie()
    runs.append(('f_near_tie', run_scenario(near_tie_scenario(d_tie), True)))
    results['near_tie_scan'] = tie_table
    for name, run in runs:
        m = metrics(run)
        results[name] = m
        print(json.dumps(m))
        _plot(run, os.path.join(args.out, f'{name}.png'),
              None if (args.no_gif or name.endswith('__flag_off'))
              else os.path.join(args.out, f'{name}.gif'),
              title=f'{name}: {run.scenario.note}')
    with open(os.path.join(args.out, 'metrics.json'), 'w', encoding='utf-8') as fh:
        json.dump(results, fh, indent=1, default=lambda o: None)
        fh.write('\n')


if __name__ == '__main__':
    main()
