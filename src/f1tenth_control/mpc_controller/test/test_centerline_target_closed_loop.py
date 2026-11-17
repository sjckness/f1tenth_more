"""
Branch mpc-centerline-target: the target stays on the centreline, the QP avoids.

compute_local_target no longer moves the lookahead target off an obstacle.
These are the three closed-loop checks the branch's work order asks for, run
with the SHIPPED weights (read from stack_params.yaml, never copied):

  1. an obstacle disc of radius 0.5 m on the centreline 1.5 m ahead, car
     aligned, 0.5 m/s (run for 10 s, see DURATION): the target is the centreline point on every
     tick, the car stays at least R_safe from the obstacle, inside the
     corridor, and its lateral error is back under 0.10 m after it;
  2. an obstacle beside the corridor (outside it): the trajectory moves by no
     more than 0.02 m against the run with no obstacle;
  3. anti-oscillation smoke on case 1: the steering changes sign at most
     twice on the way round.

THE RIG is test_person_pass_closed_loop.py's: the real
build_straight_corridor (goal_distance branch, rebuilt every
corridor_update_period like the node), the real compute_local_target and the
real RTI solve_mpc_step, around f1tenth_state_fcn_dt_beta. Nominal model: no
actuator lag, no estimator error, no detection noise. The obstacle is static
and always in the list.

WHAT "THE CAR" IS for the distance check: the centre of its car_radius disc,
the axle midpoint (base_link + wheelbase / 2 along the heading), against
R_safe = r + car_radius + avoidance_margin -- the same disc the solver's
obstacle cost uses. base_link is the rear axle, 0.15 m behind it.

DURATION. The work order says 6 s; the runs last 10 s. From a standstill
with the obstacle ahead the solver waits ~2 s for the wheels to reach full
lock before it accelerates, so at 6 s the car is still beside the obstacle
and "after the obstacle" has not happened yet.

SYMMETRY. Case 1 exactly as written (obstacle ON the line, car aligned) is a
mirror-symmetric QP: the obstacle cost has no lateral gradient and the solver
has no side to prefer. Any real detection breaks that symmetry, so case 1 is
also run with the obstacle 5 mm off the line ('nudged'); the exact case is
kept to show what it does.

Assertions that do NOT hold today are kept, marked xfail(strict=True) with
the reason, so they flip loudly the day they start to hold; the work order
says to report these, not to add a mechanism (a side choice, a homing term)
to make them pass.

Run standalone: python3 -m pytest test/test_centerline_target_closed_loop.py -v
Print the runs:  python3 -m pytest test/test_centerline_target_closed_loop.py -s -k table
"""

import math

import numpy as np
import pytest

from f1tenth_params.corridor_geometry import CORRIDOR_WIDTH_RATIO
from f1tenth_params.param_defaults import get_value
from mpc_controller.MPC_corr import MPCController
from mpc_controller.drive_limits import clamp_drive_speed
from mpc_controller.mpc_solver import shift_warm_start, solve_mpc_step
from mpc_controller.vehicle_model import f1tenth_state_fcn_dt_beta

import test_object_approach_closed_loop as rig

SPEED_MPS = 0.5
DURATION_S = 10.0
OBSTACLE_AHEAD_M = 1.5
OBSTACLE_R_M = 0.5
NUDGE_M = 0.005
# Test 2: the obstacle's near surface 1.25 m off the line, outside the
# corridor's widest half-width (corr_half_width_start_m * CORRIDOR_WIDTH_RATIO).
BESIDE_LATERAL_M = 1.5
BESIDE_R_M = 0.25

CENTRELINE_TOL_M = 1e-6
RETURN_TOL_M = 0.10
UNCHANGED_TOL_M = 0.02
MAX_SIGN_CHANGES = 2
# Steering below this magnitude counts as straight, so solver noise around
# zero is not counted as a sign change (0.01 rad = 0.57 deg).
STEER_DEADBAND_RAD = 0.01

WEIGHT_KEYS = ('w_term', 'w_v', 'w_psi', 'w_u_a', 'w_du_delta', 'w_du_a',
               'w_delta0', 'w_obs', 'w_corr', 'w_line', 'w_psi_stage')


def shipped_weights():
    """Build the weights dict MPCController.__init__ builds, from the same keys."""
    return {k: float(get_value('mpc_' + k)) for k in WEIGHT_KEYS}


def r_safe(r):
    return r + rig.CAR_RADIUS + rig.AVOIDANCE_MARGIN


def _fake_controller():
    fake = rig._ObjectMPC(standoff=1.0, speed=SPEED_MPS)
    fake.corr_wmin = float(get_value('corr_half_width_start_m'))
    fake.corr_wmax = fake.corr_wmin * CORRIDOR_WIDTH_RATIO
    fake.corridor_heading_return = bool(get_value('corridor_heading_return'))
    fake.goal_distance = 4.0
    fake.goal_start_xy = (0.0, 0.0)
    fake.goal_anchor_odom = (0.0, 0.0, 0.0)
    fake.psi_init_corridor = 0.0
    return fake


class Run:
    """Per-tick record of one closed-loop drive."""

    def __init__(self):
        self.xs = []              # state after each tick
        self.deltas = []          # steering command of each tick
        self.target_off_line = []  # |target - nearest centreline sample|
        self.out_of_corridor = []  # |lateral| - halfWidth, corridor in force

    @property
    def states(self):
        return np.array(self.xs)

    def disc_centres(self):
        s = self.states
        half = rig.WHEELBASE / 2.0
        return s[:, 0] + half * np.cos(s[:, 2]), s[:, 1] + half * np.sin(s[:, 2])

    def min_centre_distance(self, obstacle):
        cx, cy = self.disc_centres()
        return float(np.min(np.hypot(cx - obstacle[0], cy - obstacle[1])))

    def passed(self, obstacle):
        """Tell whether the car's disc centre got beyond the obstacle's far edge."""
        cx, _ = self.disc_centres()
        return bool(np.any(cx > obstacle[0] + obstacle[2]))

    def lateral_error_after(self, obstacle):
        """|y| from the original line at the end, once past the obstacle."""
        if not self.passed(obstacle):
            return math.inf
        return abs(float(self.states[-1, 1]))

    def sign_changes(self):
        d = np.asarray(self.deltas)
        signs = np.sign(np.where(np.abs(d) < STEER_DEADBAND_RAD, 0.0, d))
        signs = signs[signs != 0.0]
        return int(np.sum(signs[1:] != signs[:-1]))


def drive(obstacle, duration=DURATION_S):
    """Drive the goal_distance move for `duration` s with `obstacle` (x, y, r) or None."""
    fake = _fake_controller()
    weights = shipped_weights()
    period = float(get_value('corridor_update_period'))
    obstacles = [obstacle] if obstacle is not None else []
    x = np.zeros(4)
    last_u = np.zeros(2)
    warm = None
    corridor = None
    last_build = None
    run = Run()
    for tick in range(int(round(duration / rig.TS))):
        t = tick * rig.TS
        if corridor is None or t - last_build >= period - 1e-9:
            corridor = MPCController.build_straight_corridor(fake, x)
            last_build = t
        corridor['obstacles_world'] = obstacles
        corridor['d_safe'] = rig.DMIN
        corridor['car_radius'] = rig.CAR_RADIUS
        corridor['avoidance_margin'] = rig.AVOIDANCE_MARGIN
        pref_nom = MPCController.compute_local_target(fake, x, corridor)
        xc = np.asarray(corridor['xc'])
        yc = np.asarray(corridor['yc'])
        run.target_off_line.append(float(np.min(np.hypot(xc - pref_nom[0], yc - pref_nom[1]))))
        u0, info = solve_mpc_step(
            x0=x, last_u=last_u, pref_nom=pref_nom, corridor=corridor,
            horizon=rig.HORIZON, ts=rig.TS, params=rig.PARAMS, limits=rig.LIMITS,
            weights=dict(weights), obstacles=obstacles, dmin=rig.DMIN, vdes=fake.vdes,
            solver='rti', warm_start_z=warm)
        warm = shift_warm_start(info.get('zopt'), rig.HORIZON) if info else None
        v_cmd, _ = clamp_drive_speed(x[3] + u0[1] * rig.TS, rig.MAX_FORWARD_SPEED, 0.0)
        u_plant = np.array([u0[0], (v_cmd - x[3]) / rig.TS])
        x = np.array(f1tenth_state_fcn_dt_beta(x, u_plant, rig.TS, rig.WHEELBASE, rig.LR))
        last_u = np.asarray(u0, dtype=float)
        idx = int(np.argmin((xc - x[0]) ** 2 + (yc - x[1]) ** 2))
        normal = np.array([corridor['nx'][idx], corridor['ny'][idx]])
        lateral = float(normal @ (x[:2] - np.array([xc[idx], yc[idx]])))
        run.out_of_corridor.append(abs(lateral) - float(corridor['halfWidth'][idx]))
        run.xs.append(x.copy())
        run.deltas.append(float(u0[0]))
    return run


ON_LINE = (OBSTACLE_AHEAD_M, 0.0, OBSTACLE_R_M)
NUDGED = (OBSTACLE_AHEAD_M, NUDGE_M, OBSTACLE_R_M)
BESIDE = (OBSTACLE_AHEAD_M, BESIDE_LATERAL_M, BESIDE_R_M)

SYMMETRIC_DEADLOCK = (
    'exact symmetry: obstacle on the line, car aligned -- the obstacle cost has '
    'no lateral gradient, the QP has no side to prefer and the car stops in '
    'front of the obstacle. Needs a decision (a side rule is a bias, and the '
    'work order forbids adding one); any real detection breaks the symmetry, '
    'see the nudged case')
NO_LATERAL_HOMING = (
    'no lateral homing by design: build_straight_corridor re-anchors the '
    'straight corridor on the car at every rebuild ("THE PRICE" in '
    'MPC_corr), so after the swerve the centreline -- and the target on it -- '
    'runs parallel to the original line and nothing pulls the car back')


@pytest.fixture(scope='module')
def runs():
    return {
        'free': drive(None),
        'on_line': drive(ON_LINE),
        'nudged': drive(NUDGED),
        'beside': drive(BESIDE),
    }


def _obstacle(name):
    return {'on_line': ON_LINE, 'nudged': NUDGED, 'beside': BESIDE}[name]


def test_print_the_table(runs, capsys):
    """Not an assertion: the numbers. Run with -s -k table."""
    with capsys.disabled():
        print(f'\n{"run":<8} {"min ctr":>8} {"R_safe":>7} {"passed":>6} '
              f'{"end y":>7} {"peak|y|":>7} {"x end":>6} {"sign ch":>7}')
        for name, run in runs.items():
            s = run.states
            if name == 'free':
                print(f'{name:<8} {"-":>8} {"-":>7} {"-":>6} {s[-1, 1]:>+7.3f} '
                      f'{np.max(np.abs(s[:, 1])):>7.3f} {s[-1, 0]:>6.2f} '
                      f'{run.sign_changes():>7}')
                continue
            obs = _obstacle(name)
            print(f'{name:<8} {run.min_centre_distance(obs):>8.3f} {r_safe(obs[2]):>7.2f} '
                  f'{str(run.passed(obs)):>6} {s[-1, 1]:>+7.3f} '
                  f'{np.max(np.abs(s[:, 1])):>7.3f} {s[-1, 0]:>6.2f} {run.sign_changes():>7}')


# ---- 1. obstacle on the centreline ------------------------------------------

@pytest.mark.parametrize('name', ['on_line', 'nudged'])
def test_1_the_target_is_the_centreline_point_on_every_tick(runs, name):
    assert max(runs[name].target_off_line) <= CENTRELINE_TOL_M


@pytest.mark.parametrize('name', ['on_line', 'nudged'])
def test_1_the_car_stays_inside_the_corridor(runs, name):
    assert max(runs[name].out_of_corridor) <= 1e-6


@pytest.mark.parametrize('name', ['on_line', 'nudged'])
def test_1_the_car_keeps_r_safe_from_the_obstacle(runs, name):
    obs = _obstacle(name)
    assert runs[name].min_centre_distance(obs) >= r_safe(obs[2])


@pytest.mark.parametrize('name', ['on_line', 'nudged'])
def test_1_the_car_never_touches_the_obstacle(runs, name):
    """The weaker statement behind R_safe: the car's disc clears the obstacle's."""
    obs = _obstacle(name)
    assert runs[name].min_centre_distance(obs) > obs[2] + rig.CAR_RADIUS


@pytest.mark.parametrize('name', [
    pytest.param('on_line', marks=pytest.mark.xfail(strict=True, reason=SYMMETRIC_DEADLOCK)),
    'nudged',
])
def test_1_the_car_gets_past_the_obstacle(runs, name):
    assert runs[name].passed(_obstacle(name))


@pytest.mark.parametrize('name', [
    pytest.param('on_line', marks=pytest.mark.xfail(strict=True, reason=SYMMETRIC_DEADLOCK)),
    pytest.param('nudged', marks=pytest.mark.xfail(strict=True, reason=NO_LATERAL_HOMING)),
])
def test_1_the_lateral_error_returns_under_10_cm_after_the_obstacle(runs, name):
    assert runs[name].lateral_error_after(_obstacle(name)) < RETURN_TOL_M


# ---- 2. obstacle beside the corridor -----------------------------------------

def test_2_the_beside_obstacle_really_is_outside_the_corridor():
    widest = float(get_value('corr_half_width_start_m')) * CORRIDOR_WIDTH_RATIO
    assert BESIDE_LATERAL_M - BESIDE_R_M > widest


def test_2_an_obstacle_outside_the_corridor_changes_nothing(runs):
    free = runs['free'].states
    beside = runs['beside'].states
    deviation = np.hypot(beside[:, 0] - free[:, 0], beside[:, 1] - free[:, 1])
    assert float(np.max(deviation)) <= UNCHANGED_TOL_M


# ---- 3. anti-oscillation smoke -----------------------------------------------

def test_3_the_steering_changes_sign_at_most_twice_going_round(runs):
    """On the nudged case, the one that actually goes round."""
    assert runs['nudged'].passed(NUDGED)
    assert runs['nudged'].sign_changes() <= MAX_SIGN_CHANGES
