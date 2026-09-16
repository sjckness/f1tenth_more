"""Closed-loop object approaches against the nominal model.

Same rig as the 09-08 A/B work and test_warm_start.py: the REAL
build_straight_corridor, the REAL compute_local_target and the REAL
solve_mpc_step, driven around f1tenth_state_fcn_dt_beta as the plant. The
only thing faked is the rclpy Node -- a duck-typed stand-in carrying the
instance state those three methods read, exactly as every other corridor test
in this package does.

WHAT THIS DOES AND DOES NOT PROVE. The plant is the model the solver
linearizes, so this is a nominal-model study, not a substitute for a live
run: it cannot see actuator lag, tyre slip, estimator error or the real
detector's latency. What it does catch is everything geometric -- a corridor
that points past the target, a schedule that lets jitter through, a cap that
binds when it should not, an approach that converges to the wrong place --
and those are the failure modes this mode was designed around.

The reported metrics, per scenario:
  final range error   final |car - target| minus the standoff. Signed:
                      negative means the car stopped short.
  arrival bearing     |wrap(bearing to target - final yaw)| at the end. How
                      squarely the car ended up facing the thing.
  peak cross-track    worst perpendicular distance from the car to the line
                      through the target along the FINAL psi_c. The approach
                      is supposed to converge onto that line, not just to the
                      point.
  steer saturation    fraction of ticks where |delta| was within 1e-3 of a
                      bound. High means the geometry was asking for more
                      rotation than the car has.

Run standalone: python3 -m pytest test/test_object_approach_closed_loop.py -v
Print the table:  python3 -m pytest test/test_object_approach_closed_loop.py -s -k table
"""

import math

import numpy as np
import pytest

from mpc_controller.MPC_corr import MPCController
from mpc_controller.mpc_solver import shift_warm_start, solve_mpc_step
from mpc_controller.object_approach import goal_point, object_speed_ref
from mpc_controller.vehicle_model import f1tenth_state_fcn_dt_beta

WHEELBASE = 0.305
LR = 0.17
TS = 0.1
HORIZON = 20
PARAMS = {'L': WHEELBASE, 'lr': LR}

LIMITS = {
    'delta_min': -0.283, 'delta_max': 0.278,
    'a_min': -2.0, 'a_max': 3.0,
    'dDeltaMin': -0.5, 'dDeltaMax': 0.5,
    'dAMin': -2.0, 'dAMax': 2.0,
    'vMin': -1.0, 'vMax': 3.0,
}

WEIGHTS = {
    'w_term': 9.0, 'w_v': 4.2857, 'w_psi': 4.5, 'w_psi_stage': 4.7736,
    'w_corr': 3.5714, 'w_u_a': 0.0, 'w_du_delta': 8.5714, 'w_du_a': 0.0,
    'w_delta0': 0.0, 'w_obs': 8.0,
}

SATURATION_EPS = 1e-3


class _FakeLogger:

    def info(self, *_a, **_k):
        pass

    debug = warn = warning = error = info


class _ObjectMPC:
    """Instance state build_straight_corridor / compute_local_target read."""

    def __init__(self, standoff=1.0, speed=0.5):
        # Corridor geometry, mirroring MPCController.__init__.
        self.front_distance = 10.0
        self.corr_L_base = 3.0
        self.corr_N = 120
        self.corr_wmin = 0.4333
        self.corr_wmax = 0.7667
        self.corr_turn_u_start = 0.00
        self.corr_turn_u_end = 0.40
        self.corr_lookahead_frac = 0.5
        self.corr_lookahead_reach_margin = 1.25
        self.obstacle_target_shift = 0.30
        self.car_radius = 0.20
        self.avoidance_margin = 0.12
        self.N = HORIZON
        self.ts = TS
        self.vdes = speed
        self.params = dict(PARAMS)
        self.limits = dict(LIMITS)

        # Modes this run is not in.
        self.drive_cmd = None
        self.goal_pose_xy = None
        self.goal_distance = None
        self.goal_start_xy = None
        self.goal_anchor_odom = None
        self.psi_init_corridor = None
        self.corridor_heading_return = False

        # Object mode.
        self.goal_object_odom_xy = None
        self.goal_object_standoff = standoff
        self.goal_object_speed = speed
        self.object_psi_c = None
        self.object_target_at_build = None
        self.object_r_full = 1.5
        self.object_r_freeze = 0.4
        self.object_c_safety = 1.5
        self.object_retarget_distance = 0.2
        self.object_a_dec = 0.3
        self.object_lead_in_m = 0.5
        self.object_last_step = None
        self.object_target_held = False

        # compute_local_target's smoothing/deflection state. The two constants
        # mirror MPCController.__init__'s declare_parameter defaults (neither
        # is in stack_params.yaml), so the target this rig feeds the solver is
        # the one the car would get.
        self.target_smoothing_alpha = 0.5
        self.deflection_decay_ticks = 5
        self.smoothed_target = None
        self.last_deflection_vec = np.zeros(2)
        self.deflection_decay_remaining = 0

    def get_logger(self):
        return _FakeLogger()

    def _corridor_lookahead(self, corridor):
        """The real one -- including its object-mode warning suppression.

        Bound rather than reimplemented: this is the method that decides where
        pref_nom sits on the final approach, and a test copy of it would be
        free to disagree with the shipped one about exactly the case these
        tests exist to check.
        """
        return MPCController._corridor_lookahead(self, corridor)


class Result:

    def __init__(self, xs, deltas, targets, psi_c_final, standoff, steps):
        self.xs = xs
        self.deltas = deltas
        self.targets = targets
        self.psi_c_final = psi_c_final
        self.standoff = standoff
        self.steps = steps

    @property
    def final_range_error(self):
        x, y = self.xs[-1][0], self.xs[-1][1]
        tx, ty = self.targets[-1]
        return math.hypot(tx - x, ty - y) - self.standoff

    @property
    def arrival_bearing_error(self):
        x, y, psi = self.xs[-1][0], self.xs[-1][1], self.xs[-1][2]
        tx, ty = self.targets[-1]
        bearing = math.atan2(ty - y, tx - x)
        return abs(math.atan2(math.sin(bearing - psi), math.cos(bearing - psi)))

    @property
    def peak_cross_track(self):
        """Worst perpendicular distance from the final approach line."""
        tx, ty = self.targets[-1]
        psi = self.psi_c_final
        worst = 0.0
        for state in self.xs:
            dx, dy = state[0] - tx, state[1] - ty
            worst = max(worst, abs(-dx * math.sin(psi) + dy * math.cos(psi)))
        return worst

    @property
    def steer_saturation_fraction(self):
        if not self.deltas:
            return 0.0
        hits = sum(1 for d in self.deltas
                   if d <= LIMITS['delta_min'] + SATURATION_EPS
                   or d >= LIMITS['delta_max'] - SATURATION_EPS)
        return hits / len(self.deltas)

    @property
    def any_infeasible(self):
        return any(s is not None and not s.feasible for s in self.steps)

    @property
    def reasons(self):
        return sorted({s.reason for s in self.steps if s is not None and s.reason})


def run_approach(target_fn, *, standoff=1.0, speed=0.5, start=(0.0, 0.0, 0.0),
                 duration=30.0, corridor_update_period=1.0, seed=None,
                 jitter=0.0, jitter_hz=12.5):
    """Drive one approach and return a Result.

    `target_fn(t)` gives the TRUE target position at time t. `jitter` adds
    white noise of that standard deviation to what the controller is told,
    refreshed at `jitter_hz` -- the detector's rate, not the control rate,
    because that is how a real estimate arrives.
    """
    rng = np.random.default_rng(seed)
    fake = _ObjectMPC(standoff=standoff, speed=speed)

    x = np.array([start[0], start[1], start[2], 0.0], dtype=float)
    last_u = np.array([0.0, 0.0], dtype=float)
    warm = None

    xs, deltas, targets, steps = [x.copy()], [], [], []
    corridor = None
    last_build_t = None
    observed = target_fn(0.0)
    last_jitter_t = -1e9

    n_ticks = int(round(duration / TS))
    for tick in range(n_ticks):
        t = tick * TS
        true_target = target_fn(t)

        if t - last_jitter_t >= 1.0 / jitter_hz:
            noise = rng.normal(0.0, jitter, size=2) if jitter > 0.0 else (0.0, 0.0)
            observed = (true_target[0] + noise[0], true_target[1] + noise[1])
            last_jitter_t = t
        fake.goal_object_odom_xy = observed
        if fake.object_psi_c is None:
            fake.object_psi_c = math.atan2(observed[1] - x[1], observed[0] - x[0])

        need = (corridor is None
                or (t - last_build_t) >= corridor_update_period
                or MPCController._object_target_moved_since_build(fake))
        if need:
            corridor = MPCController.build_straight_corridor(fake, x)
            corridor['obstacles_world'] = []
            corridor['d_safe'] = 0.0
            last_build_t = t
            steps.append(fake.object_last_step)

        r = math.hypot(observed[0] - x[0], observed[1] - x[1]) - standoff
        vdes = object_speed_ref(r, speed, fake.object_a_dec)
        fake.vdes = speed          # the corridor's own reach cap stays nominal

        pref_nom = MPCController.compute_local_target(fake, x, corridor)
        u0, info = solve_mpc_step(
            x0=x, last_u=last_u, pref_nom=pref_nom, corridor=corridor,
            horizon=HORIZON, ts=TS, params=PARAMS, limits=LIMITS,
            weights=dict(WEIGHTS), obstacles=[], dmin=0.32, vdes=vdes,
            solver='rti', warm_start_z=warm)
        warm = shift_warm_start(info.get('zopt'), HORIZON) if info else None

        x = np.array(f1tenth_state_fcn_dt_beta(x, u0, TS, WHEELBASE, LR),
                     dtype=float)
        last_u = np.asarray(u0, dtype=float)

        xs.append(x.copy())
        deltas.append(float(u0[0]))
        targets.append(true_target)

        # Stopped at the standoff: the speed reference is zero and the car has
        # actually come to rest. Not an arrival LATCH -- the run simply has
        # nothing left to show.
        if r <= 0.02 and abs(x[3]) < 0.02 and t > 2.0:
            break

    if not targets:
        targets = [target_fn(0.0)]
    return Result(xs, deltas, targets, fake.object_psi_c, standoff, steps)


# --------------------------------------------------------------- scenarios

def _static(px, py):
    return lambda _t: (px, py)


SCENARIOS = {
    'ahead 4 m': dict(target_fn=_static(4.0, 0.0)),
    '45 deg off, 4 m': dict(
        target_fn=_static(4.0 * math.cos(math.pi / 4), 4.0 * math.sin(math.pi / 4))),
    '70 deg off, 2.5 m': dict(
        target_fn=_static(2.5 * math.cos(math.radians(70)),
                          2.5 * math.sin(math.radians(70)))),
    # TWO lateral cases, because the one originally asked for turns out to be
    # two different questions and only one of them is about this controller.
    #
    # 'lateral 0.3 m/s' runs the target sideways FOREVER. A target starting
    # 4 m ahead and walking at 0.3 m/s is 12 m away across in 40 s, and the
    # car's shipped speed is 0.5 m/s: the target passes abeam and then astern,
    # and no forward-only vehicle at 0.5 m/s wins that chase. It is kept
    # because the failure is worth having in the table and because the
    # controller's behaviour in it is the honest one -- it reports
    # target_behind rather than pretending -- but it is a statement about the
    # vehicle's speed, not about the corridor.
    #
    # 'lateral 0.3 m/s, 8 s' is the tracking question: the target really moves
    # (2.4 m of it, well past object_retarget_distance_m) and then holds, so
    # the approach has to follow a moving target AND converge.
    'lateral 0.3 m/s': dict(
        target_fn=lambda t: (4.0, 0.3 * t), duration=40.0),
    'lateral 0.3 m/s, 8 s': dict(
        target_fn=lambda t: (4.0, 0.3 * min(t, 8.0)), duration=40.0),
    'jitter 5 cm @ 12.5 Hz': dict(
        target_fn=_static(4.0, 0.0), jitter=0.05, seed=20260916),
}


@pytest.fixture(scope='module')
def table():
    """Every scenario at both standoffs, run once and shared."""
    out = {}
    for name, kwargs in SCENARIOS.items():
        for standoff in (1.0, 1.2):
            out[(name, standoff)] = run_approach(standoff=standoff, **kwargs)
    return out


class TestClosedLoopTable:

    def test_print_the_table(self, table, capsys):
        """Not an assertion -- the reported numbers. Run with -s to see it."""
        with capsys.disabled():
            print(f'\n{"scenario":<24} {"s/off":>5} {"range err":>10} '
                  f'{"bearing":>9} {"x-track":>8} {"sat":>6}  {"flags"}')
            print('-' * 78)
            for (name, standoff), res in table.items():
                flags = ','.join(res.reasons) or '-'
                print(f'{name:<24} {standoff:>5.1f} '
                      f'{res.final_range_error:>+10.3f} '
                      f'{math.degrees(res.arrival_bearing_error):>8.2f}d '
                      f'{res.peak_cross_track:>8.3f} '
                      f'{res.steer_saturation_fraction:>6.2f}  {flags}')
            print()

    @pytest.mark.parametrize('standoff', [1.0, 1.2])
    def test_straight_ahead_arrives_at_the_standoff(self, table, standoff):
        res = table[('ahead 4 m', standoff)]
        assert abs(res.final_range_error) < 0.10
        assert math.degrees(res.arrival_bearing_error) < 5.0
        assert res.peak_cross_track < 0.10

    @pytest.mark.parametrize('standoff', [1.0, 1.2])
    def test_forty_five_degrees_off_converges(self, table, standoff):
        res = table[('45 deg off, 4 m', standoff)]
        assert abs(res.final_range_error) < 0.20
        assert math.degrees(res.arrival_bearing_error) < 15.0

    @pytest.mark.parametrize('standoff', [1.0, 1.2])
    def test_seventy_degrees_off_at_two_and_a_half_metres(self, table, standoff):
        """Expected to be hard: report which of the two outcomes happened.

        Either the geometry is declared infeasible (the goal starts inside a
        full-lock circle) or the approach recovers. Both are acceptable; what
        is NOT acceptable is converging somewhere wrong while reporting
        feasible.
        """
        res = table[('70 deg off, 2.5 m', standoff)]
        if res.any_infeasible:
            assert res.reasons, 'infeasible with no reason given'
        else:
            assert abs(res.final_range_error) < 0.40, (
                'reported feasible throughout but did not arrive')

    @pytest.mark.parametrize('standoff', [1.0, 1.2])
    def test_a_laterally_moving_target_is_tracked(self, table, standoff):
        """Real target motion, then a hold. This is the tracking question.

        RANGE converges: -0.175 m at standoff 1.0, -0.231 m at 1.2.

        BEARING DOES NOT, and that is a real limitation of this design rather
        than a loose threshold. It comes out at about 37 degrees at both
        standoffs, with roughly a metre of final cross-track. The cause is
        structural: the car chases a target that has moved 2.4 m sideways, so
        it closes from the flank, and psi_c FREEZES at r_freeze -- by design,
        to keep jitter out of the last 0.4 m. Whatever approach direction the
        pursuit left is therefore the one it arrives on. The vehicle ends up
        the right distance from the target while pointing about 37 degrees
        away from it.

        Pinned at 45 rather than asserted away: the number is a property worth
        noticing if it moves, and worth fixing at the MISSION level (a final
        alignment step, or bearing error carried into move_scoring) rather
        than by making psi_c chase through the freeze band, which is what
        r_freeze exists to prevent.
        """
        res = table[('lateral 0.3 m/s, 8 s', standoff)]
        assert abs(res.final_range_error) < 0.40
        assert math.degrees(res.arrival_bearing_error) < 45.0

    @pytest.mark.parametrize('standoff', [1.0, 1.2])
    def test_an_unwinnable_chase_is_reported_not_faked(self, table, standoff):
        """A target that outruns the car must not read as a clean arrival.

        0.3 m/s sideways forever against a 0.5 m/s car is a chase the vehicle
        loses once the target passes abeam. The controller's job there is to
        say so -- target_behind -- rather than to converge on something.
        """
        res = table[('lateral 0.3 m/s', standoff)]
        assert 'target_behind' in res.reasons, (
            'the target went astern and the approach never said so')

    @pytest.mark.parametrize('standoff', [1.0, 1.2])
    def test_jitter_does_not_destabilise_the_approach(self, table, standoff):
        """The freeze radius exists for exactly this."""
        clean = table[('ahead 4 m', standoff)]
        noisy = table[('jitter 5 cm @ 12.5 Hz', standoff)]
        assert abs(noisy.final_range_error) < 0.15
        assert noisy.steer_saturation_fraction <= clean.steer_saturation_fraction + 0.10
        assert noisy.peak_cross_track < 0.25


class TestTheCorridorEndsAtTheGoal:

    def test_pref_nom_never_goes_past_the_goal(self):
        """The defect the goal_pose branch has, pinned as absent here.

        goal_pose floors its corridor length at 1.0 m, so inside a metre its
        far end -- and therefore pref_nom, which clamps to that end -- sits
        BEYOND the target. Here the corridor ends at the goal, so the terminal
        cost can pull the car to the standoff and no further.
        """
        fake = _ObjectMPC(standoff=1.0)
        target = (4.0, 0.0)
        fake.goal_object_odom_xy = target
        fake.object_psi_c = 0.0

        for car_x in (0.0, 1.0, 2.0, 2.5, 2.9, 2.99):
            x = np.array([car_x, 0.0, 0.0, 0.3])
            corridor = MPCController.build_straight_corridor(fake, x)
            corridor['obstacles_world'] = []
            corridor['d_safe'] = 0.0
            fake.smoothed_target = None
            pref = MPCController.compute_local_target(fake, x, corridor)
            goal = goal_point(target, fake.object_psi_c, 1.0)
            assert pref[0] <= goal[0] + 1e-6, (
                f'pref_nom at car_x={car_x} is {pref[0]:.4f}, past the goal '
                f'{goal[0]:.4f}')

    def test_the_corridor_end_is_the_goal(self):
        fake = _ObjectMPC(standoff=1.0)
        fake.goal_object_odom_xy = (4.0, 1.0)
        fake.object_psi_c = 0.2
        x = np.array([0.0, 0.0, 0.0, 0.3])
        corridor = MPCController.build_straight_corridor(fake, x)
        goal = goal_point((4.0, 1.0), fake.object_psi_c, 1.0)
        assert corridor['Pend'][0] == pytest.approx(goal[0], abs=1e-6)
        assert corridor['Pend'][1] == pytest.approx(goal[1], abs=1e-6)

    def test_there_is_no_one_metre_length_floor(self):
        fake = _ObjectMPC(standoff=1.0)
        fake.goal_object_odom_xy = (4.0, 0.0)
        fake.object_psi_c = 0.0
        x = np.array([2.9, 0.0, 0.0, 0.2])     # goal is 0.1 m ahead
        corridor = MPCController.build_straight_corridor(fake, x)
        assert corridor['L'] < 1.0

    def test_the_object_corridor_is_flagged_for_the_lookahead(self):
        fake = _ObjectMPC(standoff=1.0)
        fake.goal_object_odom_xy = (4.0, 0.0)
        fake.object_psi_c = 0.0
        corridor = MPCController.build_straight_corridor(
            fake, np.array([0.0, 0.0, 0.0, 0.0]))
        assert corridor['objectMode'] is True

    def test_other_modes_are_not_flagged(self):
        fake = _ObjectMPC()
        fake.goal_object_odom_xy = None
        fake.goal_pose_xy = (4.0, 0.0)
        corridor = MPCController.build_straight_corridor(
            fake, np.array([0.0, 0.0, 0.0, 0.0]))
        assert corridor['objectMode'] is False
