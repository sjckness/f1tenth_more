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
  min body gap        smallest (car-to-target-centre distance) minus the
                      person's true half-width and car_radius over the run:
                      the disk-model clearance between the car and the
                      person, whatever radius the obstacle list carried.
  deflected           fraction of ticks on which compute_local_target pushed
                      the lookahead target off the centreline.

THE TARGET IS ALSO AN OBSTACLE, as on the car: the person the approach drives
at is in the same detection stream that feeds /perception/obstacles_2d, and it
is not excluded. Each obstacle row puts one disk at the observed target with
the radius obstacle_projector_node would publish for a 0.50 m wide, 1.75 m tall
person: 0.875 in legacy mode (half the HEIGHT), 0.25 in footprint mode, plus
any obstacle_class_margin_m. 'none' is the obstacle-free reference.

Run standalone: python3 -m pytest test/test_object_approach_closed_loop.py -v
Print the table:  python3 -m pytest test/test_object_approach_closed_loop.py -s -k table
"""

import math
import os

import numpy as np
import pytest

from mpc_controller.MPC_corr import MPCController
from mpc_controller.drive_limits import clamp_drive_speed
from mpc_controller.mpc_solver import shift_warm_start, solve_mpc_step
from mpc_controller.object_approach import (
    TargetBehindPersistence, goal_point, heading_margin_for, object_speed_ref)
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

R_FREEZE = 0.4
BEHIND_PERSIST_SEC = 0.5
BEHIND_PERSIST_TICKS = int(round(BEHIND_PERSIST_SEC / TS))

CAR_RADIUS = 0.20
AVOIDANCE_MARGIN = 0.12
DMIN = CAR_RADIUS + AVOIDANCE_MARGIN

# A standing adult, as detection_3d_node back-projects one: bbox.size.x is the
# width, bbox.size.y the height. obstacle_projector_node's two radius rules.
PERSON_WIDTH = 0.50
PERSON_HEIGHT = 1.75
PERSON_RADIUS = {
    'legacy': max(PERSON_WIDTH, PERSON_HEIGHT) / 2.0,
    'footprint': PERSON_WIDTH / 2.0,
}

# (label, obstacle radius the MPC sees or None). The class-margin rows are the
# obstacle_class_margin_m options under consideration for 'person'; none of
# them is a chosen value.
OBSTACLE_ROWS = (
    ('none', None),
    ('legacy', PERSON_RADIUS['legacy']),
    ('footprint', PERSON_RADIUS['footprint']),
    ('footprint+0.2', PERSON_RADIUS['footprint'] + 0.2),
    ('footprint+0.3', PERSON_RADIUS['footprint'] + 0.3),
    ('footprint+0.4', PERSON_RADIUS['footprint'] + 0.4),
)


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
        self.car_radius = CAR_RADIUS
        self.avoidance_margin = AVOIDANCE_MARGIN
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
        self.object_r_freeze = R_FREEZE
        self.object_heading_margin = heading_margin_for(R_FREEZE)
        self.object_behind = TargetBehindPersistence(BEHIND_PERSIST_SEC)
        self.object_last_flags = None
        self.object_behind_terminal = False
        self.object_behind_for_s = 0.0
        self.goal_object_move_id = 'rig'
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

    def __init__(self, xs, deltas, targets, psi_c_final, standoff, steps,
                 step_ticks=(), deflected=(), observed=(), psi_cs=(),
                 flags=(), terminal=()):
        self.xs = xs
        self.deltas = deltas
        self.targets = targets
        self.psi_c_final = psi_c_final
        self.standoff = standoff
        self.steps = steps
        # Tick index of each entry in `steps` (the rebuild it was planned at).
        self.step_ticks = list(step_ticks)
        self.deflected = list(deflected)
        # Per tick: the target position the controller was told, and the held
        # corridor heading in force for that tick's solve.
        self.observed = list(observed)
        self.psi_cs = list(psi_cs)
        # Per tick: MPCController._assess_object_tick's flags, and the
        # terminal target_behind state it left behind.
        self.flags = list(flags)
        self.terminal = list(terminal)

    @property
    def min_centre_distance(self):
        """Smallest car-to-TRUE-target distance over the run [m]."""
        return min(math.hypot(tx - s[0], ty - s[1])
                   for s, (tx, ty) in zip(self.xs[1:], self.targets))

    @property
    def min_body_gap(self):
        """Disk-model clearance to the real person: see the module docstring."""
        return self.min_centre_distance - PERSON_WIDTH / 2.0 - CAR_RADIUS

    @property
    def deflected_fraction(self):
        return sum(self.deflected) / len(self.deflected) if self.deflected else 0.0

    def first_terminal_tick(self):
        ticks = self.terminal_ticks
        return ticks[0] if ticks else None

    def first_behind_tick(self, sustain):
        """First tick from which the target stays in the rear half-plane, or None.

        Geometry only, from the state ENTERING each tick and the target at
        that tick -- what the planner sees. It must hold for `sustain`
        consecutive ticks: a car circling a target that walks past it can
        graze 90 degrees and turn back (the footprint 1.0 m unbounded chase
        sits at 89.3 degrees for a stretch), and that is not "behind".
        """
        run = 0
        for tick, (state, (tx, ty)) in enumerate(zip(self.xs, self.targets)):
            bearing = math.atan2(ty - state[1], tx - state[0])
            behind = abs(math.atan2(math.sin(bearing - state[2]),
                                    math.cos(bearing - state[2]))) > math.pi / 2.0
            run = run + 1 if behind else 0
            if run >= sustain:
                return tick - sustain + 1
        return None

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
    def advisory_ticks(self):
        """Ticks inside_turn_radius (advisory) was set."""
        return [t for t, f in enumerate(self.flags) if f is not None and f.inside_turn_radius]

    @property
    def terminal_ticks(self):
        """Ticks target_behind_terminal was set."""
        return [t for t, term in enumerate(self.terminal) if term]

    def advisory_ticks_within(self, r_max):
        return [t for t in self.advisory_ticks if self.flags[t].r <= r_max]


def run_approach(target_fn, *, standoff=1.0, speed=0.5, start=(0.0, 0.0, 0.0),
                 duration=30.0, corridor_update_period=1.0, seed=None,
                 jitter=0.0, jitter_hz=12.5, obstacle_r=None, stop_at_rest=True,
                 speed_limits=None):
    """Drive one approach and return a Result.

    `target_fn(t)` gives the TRUE target position at time t. `jitter` adds
    white noise of that standard deviation to what the controller is told,
    refreshed at `jitter_hz` -- the detector's rate, not the control rate,
    because that is how a real estimate arrives.

    `obstacle_r`, when given, puts the target in the obstacle list as a disk
    of that radius at the OBSERVED position, attached every tick the way
    MPC_corr's control loop attaches obstacles_global_live: into
    corridor['obstacles_world'] for compute_local_target and the solver's
    corridor cost, and as solve_mpc_step's `obstacles`.

    `stop_at_rest` ends the run once the car is at rest at the standoff.
    Turn it off for a target that walks PAST the car: the speed ramp brings
    the car to rest as the target goes abeam, and the run would end right
    where the interesting part starts.

    `speed_limits`, (max_forward, max_reverse), applies MPC_corr's /drive
    clamp (drive_limits.clamp_drive_speed) to the plant: the published speed
    is v + a * ts, and the VESC tracks the CLAMPED value, so the plant is
    stepped with the acceleration that reaches it. None reproduces the
    unclamped controller the earlier tables were built with.
    """
    rng = np.random.default_rng(seed)
    fake = _ObjectMPC(standoff=standoff, speed=speed)

    x = np.array([start[0], start[1], start[2], 0.0], dtype=float)
    last_u = np.array([0.0, 0.0], dtype=float)
    warm = None

    xs, deltas, targets, steps = [x.copy()], [], [], []
    step_ticks, deflected, observed_log, psi_c_log = [], [], [], []
    flags_log, terminal_log = [], []
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
            last_build_t = t
            steps.append(fake.object_last_step)
            step_ticks.append(tick)
        # Every tick, after the rebuild check -- exactly where control_loop
        # calls it -- and through the real method, so the rig's flags cannot
        # drift from the node's.
        flags_log.append(MPCController._assess_object_tick(fake, x, t))
        terminal_log.append(fake.object_behind_terminal)
        obstacles = [] if obstacle_r is None else [
            (observed[0], observed[1], float(obstacle_r))]
        corridor['obstacles_world'] = obstacles
        corridor['d_safe'] = DMIN if obstacles else 0.0
        corridor['car_radius'] = fake.car_radius
        corridor['avoidance_margin'] = fake.avoidance_margin

        r = math.hypot(observed[0] - x[0], observed[1] - x[1]) - standoff
        vdes = object_speed_ref(r, speed, fake.object_a_dec)
        fake.vdes = speed          # the corridor's own reach cap stays nominal

        pref_nom = MPCController.compute_local_target(fake, x, corridor)
        # compute_local_target re-arms the coast counter only on a tick that
        # really deflected, so a full counter means "this tick".
        deflected.append(bool(obstacles) and fake.deflection_decay_remaining
                         == fake.deflection_decay_ticks)
        u0, info = solve_mpc_step(
            x0=x, last_u=last_u, pref_nom=pref_nom, corridor=corridor,
            horizon=HORIZON, ts=TS, params=PARAMS, limits=LIMITS,
            weights=dict(WEIGHTS), obstacles=obstacles, dmin=DMIN, vdes=vdes,
            solver='rti', warm_start_z=warm)
        warm = shift_warm_start(info.get('zopt'), HORIZON) if info else None

        u_plant = np.asarray(u0, dtype=float)
        if speed_limits is not None:
            v_cmd, _ = clamp_drive_speed(x[3] + u_plant[1] * TS, *speed_limits)
            u_plant = np.array([u_plant[0], (v_cmd - x[3]) / TS])
        x = np.array(f1tenth_state_fcn_dt_beta(x, u_plant, TS, WHEELBASE, LR),
                     dtype=float)
        last_u = np.asarray(u0, dtype=float)

        xs.append(x.copy())
        deltas.append(float(u0[0]))
        targets.append(true_target)
        observed_log.append(tuple(observed))
        psi_c_log.append(fake.object_psi_c)

        # Stopped at the standoff: the speed reference is zero and the car has
        # actually come to rest. Not an arrival LATCH -- the run simply has
        # nothing left to show.
        if stop_at_rest and r <= 0.02 and abs(x[3]) < 0.02 and t > 2.0:
            break

    if not targets:
        targets = [target_fn(0.0)]
    return Result(xs, deltas, targets, fake.object_psi_c, standoff, steps,
                  step_ticks, deflected, observed_log, psi_c_log,
                  flags_log, terminal_log)


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
    # The target really moves (2.4 m, well past object_retarget_distance_m)
    # and then holds, so the approach has to follow a moving target AND
    # converge. The unbounded version (sideways forever) is not a row: it is a
    # chase a 0.5 m/s forward-only car loses once the target passes abeam, and
    # the only question worth asking of it is how quickly the controller says
    # so -- see TestTargetBehindIsRaised.
    'lateral 0.3 m/s, 8 s': dict(
        target_fn=lambda t: (4.0, 0.3 * min(t, 8.0)), duration=40.0),
    'jitter 5 cm @ 12.5 Hz': dict(
        target_fn=_static(4.0, 0.0), jitter=0.05, seed=20260916),
}

STANDOFFS = (1.0, 1.2)

# The class-margin rows are a sweep of options, not shipped configuration, and
# they triple the run count. Off by default; the report table sets it.
MARGIN_SWEEP = os.environ.get('OBJECT_APPROACH_MARGIN_SWEEP') == '1'


def _rows():
    return [row for row in OBSTACLE_ROWS
            if MARGIN_SWEEP or not row[0].startswith('footprint+')]


@pytest.fixture(scope='module')
def table():
    """Every scenario x obstacle row x standoff, run once and shared."""
    out = {}
    for name, kwargs in SCENARIOS.items():
        for label, obstacle_r in _rows():
            for standoff in STANDOFFS:
                out[(name, label, standoff)] = run_approach(
                    standoff=standoff, obstacle_r=obstacle_r, **kwargs)
    return out


class TestClosedLoopTable:

    def test_print_the_table(self, table, capsys):
        """Not an assertion -- the reported numbers. Run with -s to see it."""
        with capsys.disabled():
            print(f'\n{"scenario":<22} {"obstacle":<14} {"s/off":>5} '
                  f'{"range err":>9} {"bearing":>8} {"x-track":>7} {"sat":>5} '
                  f'{"body gap":>8} {"defl":>5}  flags')
            print('-' * 104)
            for (name, label, standoff), res in table.items():
                adv = res.advisory_ticks
                flags = (f'advisory {len(adv)}t r {res.flags[adv[0]].r:+.2f}..'
                         f'{res.flags[adv[-1]].r:+.2f}' if adv else '-')
                if res.terminal_ticks:
                    flags += f' TERMINAL@{res.terminal_ticks[0]}'
                print(f'{name:<22} {label:<14} {standoff:>5.1f} '
                      f'{res.final_range_error:>+9.3f} '
                      f'{math.degrees(res.arrival_bearing_error):>7.2f}d '
                      f'{res.peak_cross_track:>7.3f} '
                      f'{res.steer_saturation_fraction:>5.2f} '
                      f'{res.min_body_gap:>8.3f} '
                      f'{res.deflected_fraction:>5.2f}  {flags}')
            print()

    @pytest.mark.parametrize('standoff', STANDOFFS)
    def test_straight_ahead_arrives_at_the_standoff(self, table, standoff):
        res = table[('ahead 4 m', 'none', standoff)]
        assert abs(res.final_range_error) < 0.10
        assert math.degrees(res.arrival_bearing_error) < 5.0
        assert res.peak_cross_track < 0.10

    @pytest.mark.parametrize('standoff', STANDOFFS)
    def test_forty_five_degrees_off_converges(self, table, standoff):
        res = table[('45 deg off, 4 m', 'none', standoff)]
        assert abs(res.final_range_error) < 0.20
        assert math.degrees(res.arrival_bearing_error) < 15.0

    @pytest.mark.parametrize('standoff', STANDOFFS)
    def test_seventy_degrees_off_is_flagged_advisory_and_still_arrives(
            self, table, standoff):
        """The goal starts inside the full-lock circle, and the approach recovers.

        inside_turn_radius is raised from the first tick, as advisory: nothing
        ends the move, and the car arrives within 0.1 m. No terminal flag --
        the target is never astern.
        """
        res = table[('70 deg off, 2.5 m', 'none', standoff)]
        assert res.advisory_ticks and res.advisory_ticks[0] == 0
        assert res.terminal_ticks == []
        assert abs(res.final_range_error) < 0.10

    @pytest.mark.parametrize('standoff', STANDOFFS)
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
        res = table[('lateral 0.3 m/s, 8 s', 'none', standoff)]
        assert abs(res.final_range_error) < 0.40
        assert math.degrees(res.arrival_bearing_error) < 45.0

    @pytest.mark.parametrize('standoff', STANDOFFS)
    def test_jitter_does_not_destabilise_the_approach(self, table, standoff):
        """The freeze radius exists for exactly this."""
        clean = table[('ahead 4 m', 'none', standoff)]
        noisy = table[('jitter 5 cm @ 12.5 Hz', 'none', standoff)]
        assert abs(noisy.final_range_error) < 0.15
        assert noisy.steer_saturation_fraction <= clean.steer_saturation_fraction + 0.10
        assert noisy.peak_cross_track < 0.25


class TestTargetObstacleRadius:
    """The target is in the obstacle list: what its radius does to the approach.

    Two mechanisms act on it and they are not the same one. compute_local_target
    DEFLECTS the lookahead target sideways when it falls inside
    R_safe = r + car_radius + avoidance_margin. Separately, the solver's w_obs
    term penalises softplus((car_radius + avoidance_margin) - d_front), with
    d_front measured from nose points up to L/2 + 0.40 m ahead of the car, so
    the car is held back once the target centre is closer than about
    0.55 + r + 0.32 m -- whether or not the goal is inside R_safe.
    """

    @pytest.mark.parametrize('scenario', ['ahead 4 m', '45 deg off, 4 m'])
    @pytest.mark.parametrize('standoff', STANDOFFS)
    def test_legacy_height_radius_holds_the_car_well_short(self, table, scenario,
                                                           standoff):
        res = table[(scenario, 'legacy', standoff)]
        assert res.final_range_error > 0.40

    @pytest.mark.parametrize('scenario', ['ahead 4 m', '45 deg off, 4 m'])
    def test_legacy_radius_puts_the_one_metre_goal_inside_r_safe(self, table, scenario):
        """R_safe 0.875 + 0.32 = 1.195 > 1.0: the lookahead target is deflected."""
        assert table[(scenario, 'legacy', 1.0)].deflected_fraction > 0.5

    @pytest.mark.parametrize('scenario', ['ahead 4 m', '45 deg off, 4 m'])
    def test_footprint_radius_never_deflects_the_goal(self, table, scenario):
        for standoff in STANDOFFS:
            assert table[(scenario, 'footprint', standoff)].deflected_fraction == 0.0

    @pytest.mark.parametrize('scenario', ['ahead 4 m', '45 deg off, 4 m'])
    def test_footprint_at_one_metre_is_held_about_a_decimetre_short(self, table, scenario):
        """Not by deflection (none happens): by w_obs. 0.55 + 0.25 + 0.32 = 1.12."""
        assert 0.05 < table[(scenario, 'footprint', 1.0)].final_range_error < 0.20

    @pytest.mark.parametrize('scenario', ['ahead 4 m', '45 deg off, 4 m'])
    def test_footprint_at_one_point_two_metres_arrives(self, table, scenario):
        assert abs(table[(scenario, 'footprint', 1.2)].final_range_error) < 0.05


class TestTargetBehindIsRaised:
    """A target that walks past the car forever: the terminal flag, and when.

    0.3 m/s sideways forever against a 0.5 m/s forward-only car is a chase the
    car loses once the target passes abeam. There is no arrival to measure;
    the question is only whether the controller says so, and exactly when.

    target_behind is evaluated every tick and made terminal after
    object_behind_persist_sec of continuous hold, so the terminal flag must
    rise exactly BEHIND_PERSIST_TICKS after the target goes astern for good,
    and never on a shorter excursion.
    """

    @pytest.mark.parametrize('label', ['none', 'footprint'])
    @pytest.mark.parametrize('standoff', STANDOFFS)
    def test_terminal_exactly_one_persistence_after_the_target_goes_astern(
            self, label, standoff):
        res = run_approach(target_fn=lambda t: (4.0, 0.3 * t), duration=40.0,
                           standoff=standoff, obstacle_r=dict(OBSTACLE_ROWS)[label],
                           stop_at_rest=False)
        astern = res.first_behind_tick(BEHIND_PERSIST_TICKS + 1)
        assert astern is not None, 'scenario never put the target astern'
        terminal = res.first_terminal_tick()
        assert terminal == astern + BEHIND_PERSIST_TICKS, (
            f'astern for good from tick {astern}, terminal at {terminal}')

    @pytest.mark.parametrize('label', ['none', 'footprint'])
    @pytest.mark.parametrize('standoff', STANDOFFS)
    def test_raw_flag_follows_the_geometry_every_tick(self, label, standoff):
        res = run_approach(target_fn=lambda t: (4.0, 0.3 * t), duration=40.0,
                           standoff=standoff, obstacle_r=dict(OBSTACLE_ROWS)[label],
                           stop_at_rest=False)
        for tick, flags in enumerate(res.flags):
            state = res.xs[tick]
            tx, ty = res.observed[tick]
            bearing = math.atan2(ty - state[1], tx - state[0])
            alpha = math.atan2(math.sin(bearing - state[2]), math.cos(bearing - state[2]))
            assert flags.target_behind is (abs(alpha) > math.pi / 2.0), tick


class TestTheDriveClampHoldsInTheChase:
    """A person crossing close in front drove the unclamped solver to +2.07 and
    -1.04 m/s against a zero reference (docs/analysis/2026-09-16_chase_overspeed.md).
    With MPC_corr's /drive clamp at its defaults the plant never leaves
    [0, 1.0] m/s."""

    def test_peak_speeds_stay_inside_the_default_limits(self):
        res = run_approach(target_fn=lambda t: (4.0, 0.3 * t), duration=40.0,
                           standoff=1.0, obstacle_r=PERSON_RADIUS['footprint'],
                           stop_at_rest=False, speed_limits=(1.0, 0.0))
        speeds = [state[3] for state in res.xs]
        assert max(speeds) <= 1.0 + 1e-9
        assert min(speeds) >= -1e-9


class TestFlagAcceptance:
    """The approaches that work raise nothing that matters.

    ahead, 45 degrees off and 5 cm jitter end with ZERO terminal flags and
    ZERO advisory flags inside r_freeze, at both standoffs, with and without
    the target in the obstacle list.
    """

    SCENARIOS_OK = ('ahead 4 m', '45 deg off, 4 m', 'jitter 5 cm @ 12.5 Hz')

    @pytest.mark.parametrize('scenario', SCENARIOS_OK)
    @pytest.mark.parametrize('label', ['none', 'footprint'])
    @pytest.mark.parametrize('standoff', STANDOFFS)
    def test_no_terminal_flag(self, table, scenario, label, standoff):
        assert table[(scenario, label, standoff)].terminal_ticks == []

    @pytest.mark.parametrize('scenario', SCENARIOS_OK)
    @pytest.mark.parametrize('label', ['none', 'footprint'])
    @pytest.mark.parametrize('standoff', STANDOFFS)
    def test_no_advisory_flag_inside_r_freeze(self, table, scenario, label, standoff):
        assert table[(scenario, label, standoff)].advisory_ticks_within(R_FREEZE) == []


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
