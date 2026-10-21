"""
The offset-free speed law through the situations a mission puts it in.

Same rig as test_speed_channel_matrix.py (speed_channel_harness: the real QP,
the real SpeedChannel, a plant that is not the model), on the plant the data
describes -- the fitted speed-dependent gain, the identified lag, no delay,
sigma 0.02 m/s speed noise -- unless a scenario says otherwise:

  * a goal_distance move with the stop ramp: the car must still stop AT the
    target, never past it, under the measured braking plant;
  * a hold mid-move (an abort, then a release): zero while held, d reset, no
    lurch on release, back to vdes;
  * blocked wheels while commanding: d frozen, the command bounded, recovery;
  * the reverse manoeuvre taking over and handing back: d reset on both
    switches, no negative command from the forward side, back to vdes;
  * a car stopped against an obstacle: the reverse manoeuvre's stuck detector
    fires exactly as it does under the legacy law;
  * LEGACY reproducing the 2026-10-21 fixed point (~0.35 commanded, ~0.27
    real at vdes 0.5 on a car returning 76 %) -- the rig's own sanity check;
  * solve time, legacy vs offset-free, against the 0.1 s budget.

Run standalone: python3 -m pytest test/test_speed_channel_scenarios.py -v -s
"""

import os
import sys

from mpc_controller.reverse_manoeuvre import ManoeuvreConfig, StuckDetector
import numpy as np
import pytest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import speed_channel_harness as harness  # noqa: E402

BRAKING_MEASURED = (0.144, 1.50)
V_BREAKAWAY = harness.nominal_config().breakaway_mps


def _fitted(seed=11, **kw):
    return harness.Plant(gain=harness.FITTED_GAIN, tau=harness.TAU_IDENTIFIED,
                         seed=seed, **kw)


def _window(trace, t0, t1, field):
    a = trace.arrays()
    sel = (a['t'] >= t0) & (a['t'] < t1)
    return a[field][sel]


class TestTheStopRampStillStopsTheCar:
    """
    M02_approach_wall's shape: drive straight, stop 2.0 m from a wall.

    The mission's front_clearance stop through the ramp, exactly as
    MPC_corr's wall branch does it (latency-predicted distance, two ticks of
    sensing delay as test_stop_ramp_closed_loop measures them), under the
    measured braking plant. Offset-free now arrives at the speed the ramp
    asks for -- v_min, 0.4 m/s -- where legacy arrived slower than it asked;
    the ramp was designed for the former, and must still land the car short
    of the target, not past it.
    """

    WALL_X = 8.0
    TARGET = 2.0

    @pytest.fixture(scope='class')
    def runs(self):
        out = {}
        for mode in ('legacy', 'offset_free'):
            for vdes in (0.5, 0.4):
                out[(mode, vdes)] = harness.run(
                    mode=mode, vdes=vdes, wall=(self.WALL_X, self.TARGET, 2), duration=25.0,
                    plant=_fitted(braking=BRAKING_MEASURED))
        return out

    # One tick of travel at the ramp's hand-over speed v_min: the hand-over is
    # decided once per tick, so a car that really arrives at v_min can rest up
    # to v_min * ts inside the target. Legacy never showed it because it
    # arrived at ~0.29 m/s, not the 0.4 the ramp asked for.
    ONE_TICK = harness.V_MIN * harness.TS

    @pytest.mark.parametrize('vdes', [0.5, 0.4])
    @pytest.mark.parametrize('seed', [11, 12, 13, 14, 15])
    def test_it_comes_to_rest_at_the_target(self, vdes, seed):
        trace = harness.run(
            mode='offset_free', vdes=vdes, wall=(self.WALL_X, self.TARGET, 2), duration=25.0,
            plant=_fitted(seed=seed, braking=BRAKING_MEASURED))
        gap = trace.rest_gap
        assert gap >= self.TARGET - self.ONE_TICK, (
            f'rested {gap:.3f} m from the wall, more than one tick inside {self.TARGET}')
        assert gap <= self.TARGET + 0.10, f'rested {gap:.3f} m from the wall, too far out'

    @pytest.mark.parametrize('vdes', [0.5, 0.4])
    def test_it_drives_the_cruise_faster_than_legacy(self, runs, vdes):
        fast = np.mean(_window(runs[('offset_free', vdes)], 4.0, 6.0, 'v_true'))
        slow = np.mean(_window(runs[('legacy', vdes)], 4.0, 6.0, 'v_true'))
        assert fast > slow + 0.05

    def test_the_command_ends_at_zero(self, runs):
        assert runs[('offset_free', 0.5)].arrays()['v_cmd'][-1] == 0.0


class TestAGoalDistanceArrival:
    """
    mpc_corr's own goal_distance stop: d_rem is NOT latency-predicted there.

    The ramp hands over on the first tick with d_rem <= d_final, up to one
    tick (v * ts = 4 cm at 0.4 m/s) after the crossing, so a car that really
    arrives at v_min rests within a few cm either side of the goal. Pinned
    loosely: it is the ramp's own quantisation, unchanged by this law, and
    legacy only hid it by arriving slower.
    """

    @pytest.mark.parametrize('vdes', [0.5, 0.4])
    def test_it_rests_within_5_cm_of_the_goal(self, vdes):
        trace = harness.run(mode='offset_free', vdes=vdes, goal_distance=3.0, duration=16.0,
                            plant=_fitted(braking=BRAKING_MEASURED))
        assert trace.rest_x == pytest.approx(3.0, abs=0.05)


class TestAHoldMidMove:

    @pytest.fixture(scope='class')
    def trace(self):
        return harness.run(mode='offset_free', vdes=0.4, hold=(4.0, 6.0), duration=11.0,
                           plant=_fitted())

    def test_zero_while_held_and_d_reset_and_frozen(self, trace):
        assert np.all(_window(trace, 4.0, 6.0, 'v_cmd') == 0.0)
        assert np.all(_window(trace, 4.0, 6.0, 'd_hat') == 0.0)
        assert ('hold' in [e[1] for e in trace.events]
                and 'hold_release' in [e[1] for e in trace.events])

    def test_the_release_restarts_from_rest_without_a_lurch(self, trace):
        first = _window(trace, 6.0, 6.3, 'v_cmd')
        # the tick after the release solves from c = 0, the next ones jump to
        # the breakaway bound and climb -- never straight back to the
        # pre-hold command.
        assert first[0] < 0.05
        assert np.max(first) <= V_BREAKAWAY + 0.1

    def test_it_returns_to_vdes(self, trace):
        assert np.mean(_window(trace, 9.0, 11.0, 'v_true')) == pytest.approx(0.4, abs=0.02)


class TestBlockedWheels:

    @pytest.fixture(scope='class')
    def trace(self):
        return harness.run(mode='offset_free', vdes=0.4, block=(4.0, 7.0), duration=12.0,
                           plant=_fitted())

    def test_d_freezes_the_tick_the_wheels_stop(self, trace):
        d = _window(trace, 4.2, 7.0, 'd_hat')
        assert np.ptp(d) < 1e-9, 'd moved while the wheels were blocked'
        before = np.mean(_window(trace, 3.0, 4.0, 'd_hat'))
        assert abs(d[0] - before) < 0.03

    def test_it_stalls_and_never_pushes_harder(self, trace):
        stalled = _window(trace, 4.0, 7.0, 'stalled')
        assert stalled[-1] == 1.0
        cmd = _window(trace, 4.0, 7.0, 'v_cmd')
        onset = np.mean(_window(trace, 3.0, 4.0, 'v_cmd'))
        assert np.max(cmd) <= max(onset, V_BREAKAWAY) + 0.02
        assert np.max(cmd) <= harness.MAX_FORWARD

    def test_it_recovers_once_free(self, trace):
        assert np.mean(_window(trace, 10.0, 12.0, 'v_true')) == pytest.approx(0.4, abs=0.02)


class TestTheReverseManoeuvreSwitch:

    MANOEUVRE = (4.0, [(0.0, 0.5), (-0.4, 1.2), (0.0, 0.5)])

    @pytest.fixture(scope='class')
    def trace(self):
        return harness.run(mode='offset_free', vdes=0.4, manoeuvre=self.MANOEUVRE,
                           duration=13.0, plant=_fitted())

    def test_d_resets_on_both_switches(self, trace):
        switches = [t for t, e in trace.events if e == 'reverse_switch']
        assert len(switches) == 2
        a = trace.arrays()
        k_back = int(np.argmin(np.abs(a['t'] - switches[1])))
        assert a['d_hat'][k_back] == 0.0

    def test_only_the_manoeuvre_commands_a_negative_speed(self, trace):
        for cmd, src in zip(trace.v_cmd, trace.source):
            if src != 'manoeuvre':
                assert cmd >= 0.0

    def test_it_returns_to_vdes(self, trace):
        assert np.mean(_window(trace, 11.0, 13.0, 'v_true')) == pytest.approx(0.4, abs=0.02)


class TestTheStuckDetectorStillFires:
    """blocked_front: stopped, mission active, something within 0.6 m of the bumper."""

    def _first_stuck(self, mode):
        cfg = ManoeuvreConfig()
        detector = StuckDetector(cfg, harness.TS)
        trace = harness.run(
            mode=mode, vdes=0.4, block=(3.0, 8.0), duration=8.0, plant=_fitted(),
            stuck=detector, front_dist=lambda t: 0.4 if t >= 3.0 else 2.0)
        return trace.stuck[0] if trace.stuck else None, cfg

    def test_it_fires_within_the_same_window_as_under_legacy(self):
        t_new, cfg = self._first_stuck('offset_free')
        t_old, _ = self._first_stuck('legacy')
        assert t_new is not None and t_old is not None
        assert t_new <= 3.0 + cfg.stuck_time_s + 0.5
        assert abs(t_new - t_old) <= 0.25


class TestLegacyReproducesTheFixedPoint:
    """report.md section 1: v_cmd* = a * ts / (1 - r), 0.353 at r = 0.758."""

    R = 0.758

    @pytest.fixture(scope='class')
    def trace(self):
        return harness.run(mode='legacy', vdes=0.5, duration=12.0,
                           plant=harness.Plant(gain=self.R, tau=harness.TAU_IDENTIFIED,
                                               seed=11))

    def test_it_settles_near_the_cars_numbers(self, trace):
        cmd = np.mean(_window(trace, 6.0, 12.0, 'v_cmd'))
        real = np.mean(_window(trace, 6.0, 12.0, 'v_true'))
        assert 0.30 <= cmd <= 0.40, cmd      # the car: 0.33 median /drive
        assert 0.22 <= real <= 0.30, real    # the car: 0.25 median /odom
        assert real == pytest.approx(self.R * cmd, abs=0.01)

    def test_the_fixed_point_formula_holds(self, trace):
        cmd = _window(trace, 6.0, 12.0, 'v_cmd')
        meas = _window(trace, 6.0, 12.0, 'v_meas')
        a_ts = float(np.mean(cmd - meas))
        assert np.mean(cmd) == pytest.approx(a_ts / (1.0 - self.R), rel=0.15)

    def test_offset_free_leaves_it_on_the_same_car(self):
        fixed = harness.run(mode='offset_free', vdes=0.4, duration=12.0,
                            plant=harness.Plant(gain=self.R, tau=harness.TAU_IDENTIFIED,
                                                seed=11))
        # 0.4 / 0.758 = 0.528 is past the 0.5 ceiling: the car gets what the
        # ceiling delivers, 0.379, not legacy's 0.23.
        assert np.mean(_window(fixed, 8.0, 12.0, 'v_true')) == pytest.approx(
            0.5 * self.R, abs=0.02)


class TestSolveTime:
    """p50/p95 of the solve, legacy vs offset-free, on the same drive."""

    @pytest.fixture(scope='class')
    def times(self):
        out = {}
        for mode in ('legacy', 'offset_free'):
            trace = harness.run(mode=mode, vdes=0.5, duration=12.0, plant=_fitted())
            ms = trace.arrays()['solve_ms']
            out[mode] = ms[np.isfinite(ms)]
        return out

    def test_it_stays_far_inside_the_budget(self, times):
        p50 = {m: float(np.percentile(v, 50)) for m, v in times.items()}
        p95 = {m: float(np.percentile(v, 95)) for m, v in times.items()}
        print(f'\nsolve ms  legacy p50 {p50["legacy"]:.1f} p95 {p95["legacy"]:.1f} | '
              f'offset_free p50 {p50["offset_free"]:.1f} p95 {p95["offset_free"]:.1f}')
        assert p95['offset_free'] < 0.3 * 100.0
        assert p95['offset_free'] < 2.0 * p95['legacy']
