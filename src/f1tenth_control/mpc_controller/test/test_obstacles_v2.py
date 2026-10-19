"""
Obstacles v2 on the MPC side: one obstacle list, aged and capped, a speed cap, a real solve.

The closed loop is test_person_pass_closed_loop.py's rig (real
build_straight_corridor, compute_local_target and OSQP solve_mpc_step around
f1tenth_state_fcn_dt_beta), with an UNKNOWN obstacle as the semantic layer
publishes it: always present (the LiDAR has no field-of-view dropout), r =
half its measured extent, no class margin.

Print the timing table:
    python3 -m pytest test/test_obstacles_v2.py -s -k timing
"""

import math
import statistics
import time

from mpc_controller.drive_limits import clamp_drive_speed
from mpc_controller.MPC_corr import _project_onto_line, MPCController
from mpc_controller.mpc_solver import shift_warm_start, solve_mpc_step
from mpc_controller.obstacle_select import apply_speed_cap, select_obstacles
from mpc_controller.vehicle_model import f1tenth_state_fcn_dt_beta

import numpy as np

import pytest

import test_object_approach_closed_loop as rig

GOAL_DISTANCE_M = 4.0
SPEED_LIMITS = (1.0, 0.0)


def run(obstacles, duration=12.0, timing=None):
    """Drive a goal_distance move with a fixed obstacle list; return min body gaps."""
    fake = rig._ObjectMPC(standoff=1.0, speed=0.5)
    fake.goal_distance = GOAL_DISTANCE_M
    fake.goal_start_xy = (0.0, 0.0)
    fake.goal_anchor_odom = (0.0, 0.0, 0.0)
    fake.psi_init_corridor = 0.0
    x = np.zeros(4)
    last_u = np.zeros(2)
    warm = None
    corridor = None
    last_build = None
    gaps = [math.inf] * len(obstacles)
    reached = None
    for tick in range(int(round(duration / rig.TS))):
        t = tick * rig.TS
        if _project_onto_line((x[0], x[1]), (0.0, 0.0), 0.0) >= GOAL_DISTANCE_M:
            reached = t
            break
        if corridor is None or t - last_build >= 1.0:
            corridor = MPCController.build_straight_corridor(fake, x)
            last_build = t
        corridor['obstacles_world'] = obstacles
        corridor['d_safe'] = rig.DMIN
        corridor['car_radius'] = rig.CAR_RADIUS
        corridor['avoidance_margin'] = rig.AVOIDANCE_MARGIN
        pref_nom = MPCController.compute_local_target(fake, x, corridor)
        t0 = time.perf_counter()
        u0, info = solve_mpc_step(
            x0=x, last_u=last_u, pref_nom=pref_nom, corridor=corridor,
            horizon=rig.HORIZON, ts=rig.TS, params=rig.PARAMS, limits=rig.LIMITS,
            weights=dict(rig.WEIGHTS), obstacles=obstacles, dmin=rig.DMIN, vdes=fake.vdes,
            solver='rti', warm_start_z=warm)
        if timing is not None:
            timing.append(time.perf_counter() - t0)
        warm = shift_warm_start(info.get('zopt'), rig.HORIZON) if info else None
        v_cmd, _ = clamp_drive_speed(x[3] + u0[1] * rig.TS, *SPEED_LIMITS)
        u_plant = np.array([u0[0], (v_cmd - x[3]) / rig.TS])
        x = np.array(f1tenth_state_fcn_dt_beta(x, u_plant, rig.TS, rig.WHEELBASE, rig.LR))
        last_u = np.asarray(u0, dtype=float)
        for i, (ox, oy, r) in enumerate(obstacles):
            gaps[i] = min(gaps[i], math.hypot(ox - x[0], oy - x[1]) - r - rig.CAR_RADIUS)
    return gaps, reached


class TestSelect:

    OBS = [(3.0, 0.0, 0.2), (1.0, 0.5, 0.1), (6.0, -1.0, 0.3), (2.0, 2.0, 0.1)]

    def test_a_stale_list_is_no_list(self):
        assert select_obstacles(self.OBS, 0.0, 0.0, 1.5, 1.0, 8) == []
        assert select_obstacles(self.OBS, 0.0, 0.0, None, 1.0, 8) == []
        assert len(select_obstacles(self.OBS, 0.0, 0.0, 0.2, 1.0, 8)) == 4

    def test_only_the_nearest_n_by_edge_distance(self):
        out = select_obstacles(self.OBS, 0.0, 0.0, 0.1, 1.0, 2)
        assert out == [(1.0, 0.5, 0.1), (2.0, 2.0, 0.1)]
        assert select_obstacles(self.OBS, 0.0, 0.0, 0.1, 1.0, 0) == self.OBS

    def test_the_speed_cap_only_while_fresh_and_positive(self):
        assert apply_speed_cap(0.8, 0.4, 0.1, 0.5) == 0.4
        assert apply_speed_cap(0.3, 0.4, 0.1, 0.5) == 0.3
        assert apply_speed_cap(0.8, -1.0, 0.1, 0.5) == 0.8
        assert apply_speed_cap(0.8, 0.4, 0.9, 0.5) == 0.8
        assert apply_speed_cap(0.8, float('nan'), 0.1, 0.5) == 0.8
        assert apply_speed_cap(0.8, None, None, 0.5) == 0.8


class TestTheSolve:

    def test_a_real_osqp_solve_plans_around_an_unknown_obstacle(self):
        """
        Plan around a 0.36 m LiDAR cluster 2 m ahead, 0.3 m off the path.

        Ignored (driving straight) the disk would overlap the car's (body gap
        0.3 - 0.18 - 0.20 = -0.08 m); in the solve it is passed clear. The
        unknown object reaches the solve exactly like a recognised one: an
        (x, y, r) disk on the same w_obs term.
        """
        unknown = (2.0, 0.3, 0.18)
        gaps, reached = run([unknown])
        assert reached is not None
        straight_gap = 0.3 - 0.18 - rig.CAR_RADIUS
        assert gaps[0] > 0.0
        assert gaps[0] > straight_gap + 0.05


def test_print_the_clearance_grid(capsys):
    """
    Print body gap vs lateral offset and radius (the report quotes it).

    Not an assertion. The soft w_obs cost does not hold clearance for an
    obstacle within ~0.3 m of the straight path, recognised or unknown alike:
    obstacles v2 changes WHICH obstacles reach this term, not the term.
    """
    rows = [(off, r, run([(2.0, off, r)])[0][0])
            for off in (0.0, 0.1, 0.2, 0.3, 0.5, 0.7) for r in (0.1, 0.18, 0.3)]
    with capsys.disabled():
        print('\noffset  r     body gap (disk model, rear-axle centre, car_radius 0.20)')
        for off, r, gap in rows:
            print(f'{off:5.1f}  {r:4.2f}  {gap:+.3f}')

    @pytest.mark.parametrize('n', [8])
    def test_eight_obstacles_solve_within_1_3x_of_one(self, n):
        """The timing budget: p95 with 8 obstacles <= 1.3 x p95 with 1."""
        one, eight = [], []
        base = [(2.0, 0.1, 0.18)]
        clutter = [(2.5 + 0.6 * k, (-1.0) ** k * (1.0 + 0.1 * k), 0.15) for k in range(n - 1)]
        for _ in range(2):
            run(base, timing=one)
            run(base + clutter, timing=eight)
        p95_one = float(np.percentile(one, 95))
        p95_eight = float(np.percentile(eight, 95))
        assert p95_eight <= 1.3 * p95_one, (p95_one, p95_eight)


def test_print_the_timing(capsys):
    """Not an assertion: the numbers the report quotes (linus)."""
    rows = []
    for n in (0, 1, 4, 8, 12):
        obs = [(2.0, 0.1, 0.18)] + [(2.5 + 0.6 * k, (-1.0) ** k * (1.0 + 0.1 * k), 0.15)
                                    for k in range(max(0, n - 1))]
        obs = obs[:n]
        t = []
        for _ in range(3):
            run(obs, timing=t)
        rows.append((n, statistics.median(t) * 1e3, float(np.percentile(t, 95)) * 1e3))
    with capsys.disabled():
        print('\nobstacles  p50 ms  p95 ms   (solve_mpc_step, rti, linus)')
        for n, p50, p95 in rows:
            print(f'{n:9d}  {p50:6.2f}  {p95:6.2f}')
