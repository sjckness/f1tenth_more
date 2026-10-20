"""Closed-loop wall approaches against the nominal model, ramp off vs ramp on.

Same rig as test_object_approach_closed_loop.py: the REAL solve_mpc_step
driven around f1tenth_state_fcn_dt_beta as the plant, with a duck-typed node
stand-in. What changes here is the scenario -- a straight approach to a wall
with a mission that says "stop TARGET metres from it, measured at the front
bumper" -- and the comparison, which is the whole point:

  RAMP OFF   drive at v_cruise until the wall distance crosses the target
             (the mission's front_clearance guard, unchanged), then command
             zero. This is the behaviour the stack shipped.
  RAMP ON    cap vdes by stop_profile's braking parabola, hand over to a zero
             command one d_final short of the target.

Both then stop under THE SAME braking plant, the one fitted to the six
archived stops (tau 0.144 s, a 1.50 m/s^2 -- test_object_approach_closed_loop's
BRAKING_MEASURED, and object_stop_distance_m's own note). The command is held
at speed for tau and the car decelerates at a; that is what the car does after
a zero command and it is not something the solver gets a say in.

THE SENSING DELAY IS MODELLED, because the ramp exists partly to compensate
it: the wall distance a tick reads is the one the car's position produced
SENSE_DELAY_TICKS ago (2 ticks, 0.2 s, matching the ~0.19 s measured on the
10-16 bag -- /perception/virtual_wall at 17 Hz with a 28.6 ms median publish
latency into a 10 Hz control loop). Turn it off and both columns improve, so
leaving it in is the honest setting.

WHAT THIS DOES AND DOES NOT PROVE. The plant is the model the solver
linearizes, so this is a nominal-model study and not a substitute for a live
run: no tyre slip, no estimator error, no VESC tracking curve, and the braking
plant is a two-parameter fit to six stops rather than a model of the brakes.
What it does catch is everything the profile is responsible for -- whether the
solver actually tracks a falling reference, whether the hand-over happens at
the floor, whether the stop error stops growing with the approach speed, and
what the ramp costs in time.

Run standalone:   python3 -m pytest test/test_stop_ramp_closed_loop.py -v
Print the table:  python3 -m pytest test/test_stop_ramp_closed_loop.py -s -k table
Write the report: python3 test/test_stop_ramp_closed_loop.py [outdir]
"""

import math
import os
import sys

import numpy as np
import pytest

from f1tenth_description.vehicle_frames import base_front_x
from f1tenth_params.param_defaults import get_value
from mpc_controller.MPC_corr import lookahead_clamp_length
from mpc_controller.drive_limits import clamp_drive_speed
from mpc_controller.mpc_solver import shift_warm_start, solve_mpc_step
from mpc_controller.stop_profile import (
    RAMP_FLOOR, StopRampLatch, StopRampProfile)
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

# The scenario: a wall across the corridor, and a mission that stops short of
# it. 2.0 m is the order's example and the one missions/drive_stop_2m_from_
# wall.json is named for; the wall is far enough out that even the 1.5 m/s
# approach reaches cruise before the ramp starts binding.
WALL_X = 12.0
TARGET_M = 2.0
BUMPER_X = base_front_x()
SENSE_DELAY_TICKS = 2

A_DEC = float(get_value('stop_ramp_a_dec_mps2'))
A_STOP = float(get_value('stop_ramp_a_stop_mps2'))
TAU = float(get_value('stop_ramp_tau_latency_s'))
V_MIN = float(get_value('min_moving_speed_mps'))
# The /drive clamp (max_forward_speed_mps) ships at 0.5 m/s, the operator's
# floor value, and its own description says what it is: "equal to the highest
# speed any mission or code path requests". A 1.5 m/s approach is therefore
# studied with the clamp AT 1.5, which is what an operator running one would
# set -- not at 0.5, which would cut all three runs to the same speed, and not
# at the solver's 3.0 box, which lets the terminal cost sprint the car to
# three times the speed the mission asked for (the first version of this rig
# did exactly that, and measured the sprint instead of the ramp).
MAX_REVERSE = float(get_value('max_reverse_speed_mps'))
BRAKING_MEASURED = (0.144, 1.50)

# The corridor is rebuilt every tick from the car, corr_L_base long, as
# MPC_corr.build_straight_corridor's straight/drive branch does (L =
# max(corr_L_base, 1.0), with no front-distance term -- that one belongs to
# the wall_turn branch). A single fixed corridor reaching all the way to the
# wall would instead put the terminal cost 12 m ahead, which is not a geometry
# the node ever builds.
CORR_L = 3.0
# pref_nom is NOT the corridor end: the node passes compute_local_target's
# LOOKAHEAD point, at max(corr_lookahead_frac * L, lookahead_clamp_length(...))
# along the centreline, pinned to the end when that exceeds L. With no
# obstacles to deflect around -- this scenario has none -- compute_local_target
# reduces to exactly that point, so the rig computes it directly rather than
# standing up the duck-typed node the deflection logic would need. The
# distinction matters: at the corridor end the terminal cost is an ARRIVAL
# target and the solver sprints at it (see _stop_ramp_limits), at the lookahead
# it is a direction pull.
LOOKAHEAD_FRAC = 0.5
LOOKAHEAD_REACH_MARGIN = 1.25

APPROACH_SPEEDS = (0.6, 1.0, 1.5)


def braking_distance(v0, braking=BRAKING_MEASURED):
    """Distance [m] covered after the command goes to zero from v0."""
    tau, a = braking
    return v0 * tau + v0 * v0 / (2.0 * a)


def _straight_corridor(length, origin=(0.0, 0.0), psi=0.0, n=120):
    """The tx/ty/nx/ny frames build_straight_corridor produces, along +x.

    Hand-built rather than built by the node: the geometry of a straight
    approach is not what is under test here, and a fixed corridor keeps the
    comparison between the two columns down to the speed reference alone.
    """
    s = np.linspace(0.0, length, n)
    xc = origin[0] + s * math.cos(psi)
    yc = origin[1] + s * math.sin(psi)
    tx = np.full(n, math.cos(psi))
    ty = np.full(n, math.sin(psi))
    return {
        'xc': xc, 'yc': yc, 'tx': tx, 'ty': ty, 'nx': -ty, 'ny': tx,
        'halfWidth': np.full(n, 0.4333),
        'L': float(length),
        'obstacles_world': [],
        'd_safe': 0.32,
        'car_radius': 0.20,
        'avoidance_margin': 0.12,
        'Pend': np.array([xc[-1], yc[-1]], dtype=float),
        'psiRef': psi,
    }


def _pref_nom(corridor, vdes):
    """compute_local_target's answer with an empty obstacle list.

    The lookahead rule is MPC_corr._corridor_lookahead's, verbatim: the
    fractional term, floored by the horizon's reach, pinned to the corridor
    end when it runs past it.
    """
    length = float(corridor['L'])
    lookahead = max(LOOKAHEAD_FRAC * length,
                    lookahead_clamp_length(HORIZON, TS, vdes,
                                           LOOKAHEAD_REACH_MARGIN))
    s_along = min(lookahead, length)
    frac = s_along / length if length > 0.0 else 0.0
    idx = min(int(round(frac * (len(corridor['xc']) - 1))),
              len(corridor['xc']) - 1)
    return np.array([corridor['xc'][idx], corridor['yc'][idx]], dtype=float)


def _ramp_limits(v_ref, v_now, a_dec):
    """MPC_corr._stop_ramp_limits, mirrored: a feasible hard box on the speed."""
    v_max = float(LIMITS['vMax'])
    if not (v_ref < v_max):
        return LIMITS
    box = max(float(v_ref), float(v_now) - a_dec * TS)
    if box >= v_max:
        return LIMITS
    return dict(LIMITS, vMax=box)


class Run:
    """One approach, and the numbers the report quotes."""

    def __init__(self, label, v_cruise, v_log, d_log, stop_v, stop_d_est, ticks):
        self.label = label
        self.v_cruise = v_cruise
        # Commanded speed and the measured bumper-to-wall distance, per tick.
        self.v_log = list(v_log)
        self.d_log = list(d_log)
        # Speed at the moment the command went to zero, and the distance the
        # mission's own sensor read then.
        self.stop_v = stop_v
        self.stop_d_est = stop_d_est
        self.ticks = ticks

    @property
    def rest_distance(self):
        """Bumper-to-wall distance [m] once the car has stopped."""
        return self.stop_d_est - braking_distance(self.stop_v)

    @property
    def stop_error(self):
        """Signed error [m] against the target. NEGATIVE = too close to the wall."""
        return self.rest_distance - TARGET_M

    @property
    def peak_decel(self):
        """Largest commanded deceleration [m/s^2] before the zero command."""
        steps = [(self.v_log[i] - self.v_log[i + 1]) / TS
                 for i in range(len(self.v_log) - 1)]
        return max(steps, default=0.0)

    @property
    def approach_seconds(self):
        return self.ticks * TS


def run_approach(v_cruise, ramp=True, start_x=0.0, target=TARGET_M,
                 sense_delay=SENSE_DELAY_TICKS):
    """Drive at the wall from start_x; return the Run.

    The loop is mpc_corr's, reduced to what this scenario exercises: read the
    (delayed) wall distance, decide the speed reference, solve, publish, step
    the plant. The mission's front_clearance guard is the `ramp=False` stop and
    stays in force with the ramp on as well -- which is the point of the
    arrival arm in f1tenth_behavior: with the ramp on, the car comes to rest ON
    the target and the strict guard never fires, so the stop here is the
    ramp's own latch instead.
    """
    profile = StopRampProfile(a_dec=A_DEC, a_stop=A_STOP, tau_latency_s=TAU,
                              v_min=V_MIN)
    latch = StopRampLatch(profile)
    speed_limits = (v_cruise, MAX_REVERSE)

    x = np.array([start_x, 0.0, 0.0, v_cruise], dtype=float)
    last_u = np.array([0.0, 0.0], dtype=float)
    warm = None
    history = [x[0]] * (sense_delay + 1)
    v_log, d_log = [], []

    for tick in range(600):
        # What the wall topic says this tick: the bumper-to-wall distance the
        # car's position produced `sense_delay` ticks ago.
        d_est = WALL_X - (history[-1 - sense_delay] + BUMPER_X)
        d_log.append(WALL_X - (x[0] + BUMPER_X))

        if ramp:
            d_rem = profile.predict(d_est, x[3]) - target
            decision = profile.decide(d_rem, v_cruise)
            stop_now = latch.update(decision)
            vdes = 0.0 if stop_now else decision.v_ref
        else:
            # The threshold it replaces: the mission's own guard, fired on the
            # same reading, with no ramp anywhere above it.
            stop_now = d_est < target
            vdes = v_cruise

        if stop_now:
            return Run(('ramp' if ramp else 'threshold'), v_cruise, v_log,
                       d_log, float(x[3]), float(d_log[-1]), tick)

        corridor = _straight_corridor(length=CORR_L, origin=(float(x[0]), 0.0))
        limits = (_ramp_limits(vdes, x[3], profile.a_dec) if ramp else LIMITS)
        u0, info = solve_mpc_step(
            x0=x, last_u=last_u, pref_nom=_pref_nom(corridor, vdes),
            corridor=corridor, horizon=HORIZON, ts=TS, params=PARAMS,
            limits=limits, weights=WEIGHTS, obstacles=[], dmin=0.32,
            vdes=vdes, solver='rti', warm_start_z=warm, control_period=TS)
        warm = shift_warm_start(info.get('zopt'), HORIZON)

        v_pub, _clamped = clamp_drive_speed(x[3] + float(u0[1]) * TS, *speed_limits)
        v_log.append(v_pub)
        # The plant is driven by the acceleration the published speed implies,
        # so the command the VESC would see is the command the model gets.
        u_plant = np.array([float(u0[0]), (v_pub - x[3]) / TS], dtype=float)
        x = np.array(f1tenth_state_fcn_dt_beta(x, u_plant, TS, WHEELBASE, LR),
                     dtype=float)
        last_u = np.array([float(u0[0]), u_plant[1]], dtype=float)
        history.append(x[0])

    raise AssertionError(f'{v_cruise} m/s approach never stopped')


@pytest.fixture(scope='module')
def runs():
    """{(label, v_cruise): Run} for both columns at all three speeds."""
    out = {}
    for v in APPROACH_SPEEDS:
        for ramp in (False, True):
            r = run_approach(v, ramp=ramp)
            out[(r.label, v)] = r
    return out


class TestTheStopError:

    # The measured stop error per approach speed, pinned. +0.015 / -0.025 /
    # -0.068 m: the sign flips with speed because the hand-over speed does
    # (see test_it_hands_over_near_the_floor), and the magnitude is bounded by
    # the discrete tracking lag rather than by anything in the profile.
    MEASURED_ERROR = {0.6: +0.015, 1.0: -0.025, 1.5: -0.068}

    @pytest.mark.parametrize('v_cruise', APPROACH_SPEEDS)
    def test_the_ramp_lands_the_car_within_7_cm_of_the_target(self, runs, v_cruise):
        err = runs[('ramp', v_cruise)].stop_error
        assert abs(err) <= 0.07, f'stop error {err:+.3f} m at {v_cruise} m/s'

    @pytest.mark.parametrize('v_cruise', APPROACH_SPEEDS)
    def test_the_measured_error_is_pinned(self, runs, v_cruise):
        assert runs[('ramp', v_cruise)].stop_error == pytest.approx(
            self.MEASURED_ERROR[v_cruise], abs=0.01)

    @pytest.mark.parametrize('v_cruise', APPROACH_SPEEDS)
    def test_it_beats_the_threshold_at_every_speed(self, runs, v_cruise):
        ramped = runs[('ramp', v_cruise)].stop_error
        threshold = runs[('threshold', v_cruise)].stop_error
        assert abs(ramped) < abs(threshold)

    def test_the_threshold_error_grows_with_speed_and_the_ramps_does_not(self, runs):
        """The actual defect, and the actual fix, in one assertion.

        The threshold's error is a braking distance, which is quadratic in the
        approach speed; the ramp always hands over at the operating floor, so
        its error is the same at every speed.
        """
        thr = [abs(runs[('threshold', v)].stop_error) for v in APPROACH_SPEEDS]
        ramp = [abs(runs[('ramp', v)].stop_error) for v in APPROACH_SPEEDS]
        assert thr[0] < thr[1] < thr[2]
        # The threshold's error spans 0.94 m over this speed range; the ramp's
        # spans 0.05 m, and what is left of that is the discrete tracking lag,
        # not the profile.
        assert max(thr) - min(thr) > 0.9
        assert max(ramp) - min(ramp) < 0.08

    def test_the_threshold_stops_a_metre_too_close_at_1_5(self, runs):
        """Pinned so the baseline cannot quietly improve and flatter the ramp."""
        assert runs[('threshold', 1.5)].stop_error < -0.8

    @pytest.mark.parametrize('v_cruise', APPROACH_SPEEDS)
    def test_what_overshoot_is_left_is_centimetres_not_metres(self, runs, v_cruise):
        """Short of the target is safe, past it is toward the wall.

        The ramp does not eliminate the overshoot, it bounds it: 6.8 cm at
        1.5 m/s, where the threshold it replaces was 1.27 m. What remains is
        the hand-over happening at 0.54 m/s instead of the floor's 0.40, which
        is the discrete tracking lag and is bounded by a_dec * ts.
        """
        assert runs[('ramp', v_cruise)].stop_error > -0.10


class TestTheProfileItself:

    @pytest.mark.parametrize('v_cruise', APPROACH_SPEEDS)
    def test_the_solver_actually_tracks_the_falling_reference(self, runs, v_cruise):
        """The cap is only worth having if the command follows it down.

        It did NOT before the hard speed box went in beside the reference: the
        solver held the cruise command all the way down a reference falling
        from 1.0 to 0.4 (MPC_corr._stop_ramp_limits has the trace). This is
        the regression test for that.
        """
        run = runs[('ramp', v_cruise)]
        assert run.v_log[0] > run.v_log[-1]
        assert run.v_log[-1] <= V_MIN + 0.15

    @pytest.mark.parametrize('v_cruise', APPROACH_SPEEDS)
    def test_it_hands_over_near_the_floor_not_at_cruise_speed(self, runs, v_cruise):
        """0.37 / 0.46 / 0.54 m/s at the three approach speeds, against a floor
        of 0.40 -- and the deviation is the whole of the residual stop error.

        d_final is the stopping distance from v_min, so handing over at v
        overshoots by braking_distance(v) - d_final. The spread comes from the
        box tracking the reference one tick late while the reference is falling
        fastest (a_dec / v grows as v shrinks), so a faster approach is still
        slightly fast at the hand-over and a slower one is slightly slow.
        Bounded by a_dec * ts = 0.06 m/s per tick of lag, and NOT a tuning
        failure -- lowering a_dec shrinks it and lengthens the approach.
        """
        run = runs[('ramp', v_cruise)]
        assert run.stop_v == pytest.approx(V_MIN, abs=0.15), (
            f'the command was {run.stop_v:.3f} m/s when the stop latched')
        assert abs(braking_distance(run.stop_v) - 0.1333) < 0.06

    @pytest.mark.parametrize('v_cruise', APPROACH_SPEEDS)
    def test_the_commanded_deceleration_stays_within_the_solver_box(
            self, runs, v_cruise):
        assert runs[('ramp', v_cruise)].peak_decel <= abs(LIMITS['a_min'])

    @pytest.mark.parametrize('v_cruise', APPROACH_SPEEDS)
    def test_the_threshold_asks_for_no_deceleration_at_all(self, runs, v_cruise):
        """Which is exactly why it overshoots: all the braking is after the stop."""
        assert runs[('threshold', v_cruise)].peak_decel < 0.2

    def test_the_ramp_costs_time_and_the_report_says_how_much(self, runs):
        """A slower arrival is the price; it has to be bounded, not zero."""
        for v in APPROACH_SPEEDS:
            extra = (runs[('ramp', v)].approach_seconds
                     - runs[('threshold', v)].approach_seconds)
            # -0.3 s to +0.2 s over this speed range: the ramp spends time
            # slowing down and gets it back by stopping d_final earlier, and
            # the two very nearly cancel. The ramp is not a cost in time.
            assert -1.0 <= extra <= 1.0, f'{extra:+.1f} s at {v} m/s'


class TestTheSensingDelay:

    def test_tau_buys_about_7_cm_per_tick_of_delay(self):
        """THE RAMP'S ACCURACY IS AS GOOD AS tau IS, and this is the exchange rate.

        Sweeping the modelled sensing delay with tau fixed at the shipped
        0.2 s: 0 ticks +0.092 m, 1 tick +0.053, 2 ticks -0.025, 3 ticks
        -0.129. tau is right for a 2-tick (0.2 s) delay, which is what the
        10-16 bag measured, and each 0.1 s of mismatch is worth about 7 cm of
        stop error.

        THE SIGN IS THE SAFE WAY ROUND when tau is too large for the true
        delay: the car stops SHORT of the target (positive error, further from
        the wall), never past it. That is why the shipped tau is 0.2 s against
        a 0.144 s measured actuation dead time rather than the other way
        about, and this test pins that sign.
        """
        errs = {d: run_approach(1.0, ramp=True, sense_delay=d).stop_error
                for d in (0, 1, 2, 3)}
        assert errs[0] > 0.0, 'over-compensating must stop the car short'
        assert errs[0] > errs[1] > errs[2] > errs[3]
        assert abs(errs[SENSE_DELAY_TICKS]) <= 0.05
        per_tick = (errs[0] - errs[3]) / 3.0
        assert 0.05 <= per_tick <= 0.09

    def test_the_threshold_is_hurt_by_it(self):
        """The baseline's error includes the delay, uncompensated."""
        with_delay = run_approach(1.0, ramp=False, sense_delay=SENSE_DELAY_TICKS)
        without = run_approach(1.0, ramp=False, sense_delay=0)
        assert abs(with_delay.stop_error) > abs(without.stop_error)


class TestADifferentTarget:

    @pytest.mark.parametrize('target', (0.5, 1.0, 4.0))
    def test_the_target_is_where_the_car_stops(self, target):
        """-0.026 / -0.027 / -0.030 m: the error is the profile's, not the target's."""
        run = run_approach(1.0, ramp=True, target=target)
        assert abs(run.rest_distance - target) <= 0.05


def _table(runs):
    lines = [
        '| v_cruise | stop error, threshold | stop error, ramp | peak decel, '
        'ramp | approach s, threshold | approach s, ramp |',
        '|---|---|---|---|---|---|',
    ]
    for v in APPROACH_SPEEDS:
        t, r = runs[('threshold', v)], runs[('ramp', v)]
        lines.append(
            f'| {v:.1f} m/s | {t.stop_error:+.3f} m | {r.stop_error:+.3f} m | '
            f'{r.peak_decel:.2f} m/s^2 | {t.approach_seconds:.1f} s | '
            f'{r.approach_seconds:.1f} s |')
    return '\n'.join(lines)


class TestPrintTheTable:

    def test_table(self, runs, capsys):
        with capsys.disabled():
            print()
            print(_table(runs))
            print(f'\nd_final = {StopRampProfile(A_DEC, A_STOP, TAU, V_MIN).d_final:.4f} m'
                  f'   (hand-over distance, computed from v_min/tau/a_stop)')


def _write_report(outdir):
    """The order's deceleration-profile plot and a short report. Not a test."""
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    runs = {}
    for v in APPROACH_SPEEDS:
        for ramp in (False, True):
            r = run_approach(v, ramp=ramp)
            runs[(r.label, v)] = r

    os.makedirs(outdir, exist_ok=True)
    fig, axes = plt.subplots(1, 2, figsize=(12, 4.5))
    colours = {0.6: 'tab:blue', 1.0: 'tab:orange', 1.5: 'tab:red'}
    for v in APPROACH_SPEEDS:
        for label, style in (('threshold', '--'), ('ramp', '-')):
            run = runs[(label, v)]
            # Distance left to the target on the x axis, counting down.
            d_rem = [d - TARGET_M for d in run.d_log[:len(run.v_log)]]
            axes[0].plot(d_rem, run.v_log, style, color=colours[v],
                         label=f'{v:.1f} m/s {label}')
        for ax, label in ((axes[1], 'ramp'),):
            run = runs[(label, v)]
            ax.plot([t * TS for t in range(len(run.v_log))], run.v_log,
                    color=colours[v], label=f'{v:.1f} m/s')
    prof = StopRampProfile(A_DEC, A_STOP, TAU, V_MIN)
    grid = np.linspace(0.0, 3.0, 400)
    axes[0].plot(grid, [max(min(prof.ramp_speed(d), 1.5), V_MIN) for d in grid],
                 ':', color='black', lw=1, label='profile (v_cruise 1.5)')
    axes[0].axvline(prof.d_final, color='grey', lw=1)
    axes[0].annotate(f'd_final {prof.d_final:.3f} m', (prof.d_final, 1.35),
                     fontsize=8, rotation=90, va='top')
    axes[0].set_xlabel('distance left to the target [m] (bumper, counting down)')
    axes[0].set_ylabel('commanded speed [m/s]')
    axes[0].set_title('Deceleration profile: threshold (dashed) vs ramp (solid)')
    axes[0].invert_xaxis()
    axes[0].grid(alpha=0.3)
    axes[0].legend(fontsize=7)
    axes[1].set_xlabel('time [s]')
    axes[1].set_ylabel('commanded speed [m/s]')
    axes[1].set_title('Ramp, commanded speed against time')
    axes[1].grid(alpha=0.3)
    axes[1].legend(fontsize=7)
    fig.tight_layout()
    png = os.path.join(outdir, 'stop_ramp_profile.png')
    fig.savefig(png, dpi=130)

    with open(os.path.join(outdir, 'report.md'), 'w') as f:
        f.write(
            '# Deceleration ramp -- nominal-model closed loop\n\n'
            'mpc_controller/test/test_stop_ramp_closed_loop.py, the real '
            'solve_mpc_step around f1tenth_state_fcn_dt_beta, with the '
            'measured braking plant (tau 0.144 s, a 1.50 m/s^2) and a 0.2 s '
            'sensing delay. Target: 2.0 m from the wall at the front bumper. '
            'Negative stop error = too close to the wall.\n\n'
            + _table(runs)
            + f'\n\nd_final (hand-over) = {prof.d_final:.4f} m, computed from '
              f'v_min {V_MIN}, tau {TAU} s, a_stop {A_STOP} m/s^2.\n\n'
              '![profile](stop_ramp_profile.png)\n')
    return png


if __name__ == '__main__':
    out = sys.argv[1] if len(sys.argv) > 1 else 'output/stop_distance'
    print('wrote', _write_report(out))
    assert RAMP_FLOOR  # imported for the rigs' vocabulary; keeps flake8 honest
