"""The deceleration ramp's own arithmetic: stop_profile.py, no solver, no node.

Four properties the ramp has to have, and the invalid-distance case:

  MONOTONE       v_ref never falls as the stopping point gets further away. A
                 distance estimate that walks inward can then only ever ask
                 for less speed, never more.
  FLOORED        v_ref is never between zero and v_min while the car is
                 moving. That band is the one min_moving_speed_mps forbids and
                 the one the VESC tracks worst -- it is the reason d_final
                 exists at all (stop_profile's module docstring).
  CAPPED         v_ref never exceeds the speed the mission asked for. The ramp
                 is a cap; speeding a move up is a different feature.
  EXACT          driving the profile brings the car to rest AT the target, for
                 every approach speed, because d_final is the stopping
                 distance from v_min under the same braking plant the rig and
                 object_stop_distance_m use.
  UNKNOWN        no reading, a stale one or "clear ahead" is not a distance:
                 no cap, no latch, and never a zero treated as arrival.

The plant here is PERFECT TRACKING (the car is at v_ref the tick after it is
asked for) -- deliberately, so these tests measure the PROFILE and nothing
else. test_stop_ramp_closed_loop.py runs the same profile through the real
solver and the real bicycle model, where the tracking lag is the whole point.

Run standalone: python3 -m pytest test/test_stop_profile.py -v
"""

import math

import pytest

from f1tenth_params.param_defaults import get_value
from mpc_controller.stop_profile import (
    RAMP_CRUISE, RAMP_FLOOR, RAMP_RAMP, RAMP_STOP, RAMP_UNKNOWN,
    StopRampLatch, StopRampProfile, final_stop_distance, predicted_distance)

# The shipped tuning, read from stack_params.yaml so this file cannot drift
# from the node (every other rig in this package does the same).
A_DEC = float(get_value('stop_ramp_a_dec_mps2'))
A_STOP = float(get_value('stop_ramp_a_stop_mps2'))
TAU = float(get_value('stop_ramp_tau_latency_s'))
V_MIN = float(get_value('min_moving_speed_mps'))

TS = 0.1
APPROACH_SPEEDS = (0.6, 1.0, 1.5)


def profile(**over):
    """The shipped profile, with any field overridden by keyword."""
    kw = dict(a_dec=A_DEC, a_stop=A_STOP, tau_latency_s=TAU, v_min=V_MIN)
    kw.update(over)
    return StopRampProfile(**kw)


def braking_distance(v0, tau, a):
    """Distance [m] covered after the command goes to zero from v0.

    The same (tau, a) plant as test_object_approach_closed_loop's
    BRAKING_MEASURED and as object_stop_distance_m's own fit: the command
    holds the car at v0 for tau, then it decelerates at a.
    """
    return v0 * tau + v0 * v0 / (2.0 * a)


def drive_to_rest(prof, d_rem0, v_cruise, ts=TS, braking=None):
    """Drive the profile from d_rem0 under perfect tracking; return the log.

    Returns (final_d_rem, v_log, mode_log). The car starts at v_cruise and is
    at v_ref the tick after it is asked for. Once the ramp latches, the car
    covers braking_distance(v_at_latch) and stops -- `braking` defaults to the
    profile's own (tau_latency_s, a_stop), i.e. the plant d_final was computed
    against.
    """
    if braking is None:
        braking = (prof.tau_latency_s, prof.a_stop)
    latch = StopRampLatch(prof)
    d_rem = float(d_rem0)
    v = float(v_cruise)
    v_log = []
    mode_log = []
    for _ in range(2000):
        decision = prof.decide(d_rem, v_cruise)
        mode_log.append(decision.mode)
        v_log.append(v)
        if latch.update(decision):
            d_rem -= braking_distance(v, *braking)
            return d_rem, v_log, mode_log
        d_rem -= v * ts
        v = decision.v_ref
    raise AssertionError('the ramp never latched')


class TestDFinalIsComputedNotTuned:

    def test_it_is_the_stopping_distance_from_the_floor(self):
        prof = profile()
        assert prof.d_final == pytest.approx(
            braking_distance(V_MIN, TAU, A_STOP))

    def test_formula_matches_the_free_function(self):
        assert profile().d_final == pytest.approx(
            final_stop_distance(V_MIN, TAU, A_STOP))

    def test_it_agrees_with_the_independently_measured_stop(self):
        """The one cross-check this module gets without a car.

        object_stop_distance_m is the SAME quantity -- the true distance
        covered after the command steps to zero from 0.4 m/s -- measured from
        six archived stops (tools/measure_stop_distance.py), median 0.111 and
        shipped at 0.14 after a 2-sd margin. d_final is derived instead, from
        tau_latency and a_stop. If these two ever disagree by more than a
        couple of centimetres, one of them has been retuned without the other
        and the ramp is braking on a number the car does not obey.
        """
        measured = float(get_value('object_stop_distance_m'))
        assert profile().d_final == pytest.approx(measured, abs=0.03)

    def test_a_gentler_measured_stop_means_a_longer_hand_over(self):
        assert profile(a_stop=0.75).d_final > profile(a_stop=1.5).d_final


class TestMonotone:

    def test_v_ref_never_falls_as_the_target_gets_further_away(self):
        prof = profile()
        last = -1.0
        d = 0.0
        while d <= 6.0:
            v_ref = prof.decide(d, 1.5).v_ref
            assert v_ref >= last - 1e-12, (
                f'v_ref dropped at d_rem={d:.3f}: {v_ref:.4f} after {last:.4f}')
            last = v_ref
            d += 0.005

    def test_the_parabola_is_the_speed_it_can_still_shed(self):
        """sqrt(2 a d) is defined by "comes to rest in exactly d"."""
        prof = profile()
        for s in (0.05, 0.4, 1.0, 3.0):
            v = prof.ramp_speed(prof.d_final + s)
            assert v * v / (2.0 * prof.a_dec) == pytest.approx(s)


class TestTheOperatingFloor:

    @pytest.mark.parametrize('v_cruise', APPROACH_SPEEDS)
    def test_v_ref_is_never_in_the_forbidden_band(self, v_cruise):
        prof = profile()
        d = 0.0
        while d <= 6.0:
            v_ref = prof.decide(d, v_cruise).v_ref
            assert v_ref == 0.0 or v_ref >= V_MIN - 1e-12, (
                f'v_ref={v_ref:.4f} at d_rem={d:.3f} is between zero and the '
                f'{V_MIN} m/s floor')
            d += 0.005

    def test_the_floor_region_is_where_the_parabola_passes_under_it(self):
        prof = profile()
        # Under the floor exactly when d_rem - d_final < v_min^2 / (2 a_dec).
        edge = prof.d_final + V_MIN ** 2 / (2.0 * prof.a_dec)
        assert prof.decide(edge - 0.01, 1.0).mode == RAMP_FLOOR
        assert prof.decide(edge + 0.01, 1.0).mode == RAMP_RAMP

    def test_a_mission_slower_than_the_floor_is_not_sped_up(self):
        """0.3 m/s moves exist (drive_turn_180.json and three others)."""
        prof = profile()
        for d in (0.2, 0.5, 2.0, 50.0):
            assert prof.decide(d, 0.3).v_ref == pytest.approx(0.3)

    def test_a_zero_cruise_speed_is_passed_through(self):
        assert profile().decide(3.0, 0.0).v_ref == 0.0


class TestItIsOnlyEverACap:

    @pytest.mark.parametrize('v_cruise', (0.3,) + APPROACH_SPEEDS)
    def test_v_ref_never_exceeds_the_commanded_speed(self, v_cruise):
        prof = profile()
        d = 0.0
        while d <= 30.0:
            assert prof.decide(d, v_cruise).v_ref <= v_cruise + 1e-12
            d += 0.01

    def test_far_out_it_does_not_bind_at_all(self):
        prof = profile()
        decision = prof.decide(50.0, 1.5)
        assert decision.mode == RAMP_CRUISE
        assert decision.v_ref == pytest.approx(1.5)

    def test_where_it_starts_binding_is_set_by_a_dec(self):
        """v_cruise^2 / (2 a_dec) + d_final out, and not before."""
        prof = profile()
        for v_cruise in APPROACH_SPEEDS:
            onset = v_cruise ** 2 / (2.0 * prof.a_dec) + prof.d_final
            assert prof.decide(onset + 0.02, v_cruise).mode == RAMP_CRUISE
            assert prof.decide(onset - 0.02, v_cruise).mode in (
                RAMP_RAMP, RAMP_FLOOR)


class TestItStopsAtTheTarget:

    @pytest.mark.parametrize('v_cruise', APPROACH_SPEEDS)
    def test_the_car_comes_to_rest_on_the_target(self, v_cruise):
        """Final |d_rem| within one tick of travel at the floor, every speed.

        This is the property the whole feature exists for: the stop error does
        not grow with the approach speed. Compare the threshold it replaces,
        which is braking_distance(v_cruise) past the target -- 0.97 m at
        1.5 m/s against the 0.11 m the floor costs.
        """
        prof = profile()
        d_final, _v_log, _modes = drive_to_rest(prof, 8.0, v_cruise)
        assert abs(d_final) <= V_MIN * TS + 0.01, (
            f'came to rest {d_final:+.4f} m from the target at '
            f'v_cruise={v_cruise}')

    @pytest.mark.parametrize('v_cruise', APPROACH_SPEEDS)
    def test_it_is_strictly_better_than_the_threshold_it_replaces(self, v_cruise):
        prof = profile()
        ramped, _v, _m = drive_to_rest(prof, 8.0, v_cruise)
        threshold = -braking_distance(v_cruise, TAU, A_STOP)
        assert abs(ramped) < abs(threshold)

    @pytest.mark.parametrize('v_cruise', APPROACH_SPEEDS)
    def test_it_hands_over_to_the_zero_command_at_the_floor(self, v_cruise):
        """The latch must trip from v_min, not from cruise speed.

        d_final is the stopping distance from v_min; tripping the latch while
        still travelling faster than that would overshoot by the difference,
        which is exactly the bug the ramp is here to remove.
        """
        prof = profile()
        _d, v_log, modes = drive_to_rest(prof, 8.0, v_cruise)
        assert modes[-1] == RAMP_STOP
        assert v_log[-1] == pytest.approx(V_MIN)
        assert RAMP_FLOOR in modes

    def test_the_continuous_profile_decelerates_at_exactly_a_dec(self):
        """v * dv/dd == a_dec everywhere on the parabola. That is its definition."""
        prof = profile()
        h = 1e-6
        for d in (0.3, 0.8, 2.0, 5.0):
            v = prof.ramp_speed(d)
            dv_dd = (prof.ramp_speed(d + h) - prof.ramp_speed(d - h)) / (2.0 * h)
            assert v * dv_dd == pytest.approx(prof.a_dec, rel=1e-4)

    def test_sampled_at_10_hz_the_last_ramp_tick_asks_for_more_than_a_dec(self):
        """A FINDING, pinned: the discrete request overshoots a_dec by ~22 %.

        a_dec is a continuous-time specification, and the control loop samples
        it at 10 Hz. Over one tick the car covers v_old * ts, but the speed it
        is then asked for is the parabola's value at that NEW position, whose
        slope dv/dd = a_dec / v is steepest exactly where the ramp is slowest.
        So the implied deceleration per tick is a_dec * v_old / v_new rather
        than a_dec, and it peaks on the last tick before the floor: 0.75 m/s^2
        at the shipped a_dec 0.6.

        Left alone rather than compensated, because both things that have to
        deliver it can: the car's own measured stop is a_stop 1.50 m/s^2 and
        the solver's box allows a_min -2.0. Lowering ts or a_dec shrinks it;
        pre-stepping the lookup to the predicted next position would make the
        ramp MORE aggressive, not less, which is the wrong direction.
        """
        prof = profile()
        worst = 0.0
        for v_cruise in APPROACH_SPEEDS:
            _d, v_log, _m = drive_to_rest(prof, 8.0, v_cruise)
            peaks = [(v_log[i] - v_log[i + 1]) / TS for i in range(len(v_log) - 1)]
            worst = max(worst, max(peaks, default=0.0))
        assert worst == pytest.approx(0.749, abs=0.01)
        assert prof.a_dec < worst < prof.a_stop

    def test_a_longer_than_measured_stop_lands_short_not_past(self):
        """The error direction when the plant is worse than the fit.

        BRAKING_LONG in test_object_approach_closed_loop is the longest of the
        six archived stops (0.133 m at 0.4 m/s against the 0.111 median). A
        car that brakes that softly overshoots the target by the difference --
        2 cm, and INTO the wall, which is why the shipped tau_latency is the
        conservative 0.2 s rather than the measured 0.144 s.
        """
        prof = profile()
        long_a = 0.08 / (0.133 - 0.4 * 0.144)
        d_rest, _v, _m = drive_to_rest(prof, 8.0, 1.0, braking=(0.144, long_a))
        assert -0.05 < d_rest < 0.0


class TestAnAbsentDistanceIsNotADistance:

    @pytest.mark.parametrize('d_rem', (None, float('nan'), float('inf'),
                                       float('-inf')))
    def test_it_does_not_cap_and_does_not_latch(self, d_rem):
        prof = profile()
        latch = StopRampLatch(prof)
        decision = prof.decide(d_rem, 1.5)
        assert decision.mode == RAMP_UNKNOWN
        assert decision.v_ref == pytest.approx(1.5)
        assert decision.d_rem is None
        assert latch.update(decision) is False

    def test_an_unknown_tick_does_not_clear_a_tripped_latch(self):
        """The sensor going quiet during the stop it commanded must not undo it."""
        prof = profile()
        latch = StopRampLatch(prof)
        assert latch.update(prof.decide(0.0, 1.0)) is True
        assert latch.update(prof.decide(None, 1.0)) is True

    def test_zero_is_never_what_an_absent_reading_means(self):
        """A zero distance stops; an absent one does not. Different answers."""
        prof = profile()
        assert prof.decide(0.0, 1.0).mode == RAMP_STOP
        assert prof.decide(None, 1.0).mode == RAMP_UNKNOWN


class TestInsideTheTarget:

    def test_a_target_already_behind_the_bumper_stops(self):
        assert profile().decide(-0.4, 1.0).mode == RAMP_STOP

    def test_the_hand_over_point_itself_stops(self):
        prof = profile()
        assert prof.decide(prof.d_final, 1.0).mode == RAMP_STOP
        assert prof.decide(prof.d_final + 1e-6, 1.0).mode != RAMP_STOP


class TestLatencyCompensation:

    def test_the_prediction_is_where_the_car_will_be(self):
        assert predicted_distance(2.0, 1.5, 0.2) == pytest.approx(1.7)

    def test_it_is_not_clamped_at_zero(self):
        """A negative prediction has to reach decide() as a stop."""
        assert predicted_distance(0.1, 1.5, 0.2) < 0.0

    def test_at_rest_it_changes_nothing(self):
        assert predicted_distance(2.0, 0.0, 0.2) == pytest.approx(2.0)

    def test_the_profile_uses_its_own_tau(self):
        assert profile(tau_latency_s=0.5).predict(2.0, 1.0) == pytest.approx(1.5)

    def test_a_faster_car_hands_over_further_out(self):
        """Latency compensation is a speed-dependent shift, not a constant."""
        prof = profile()
        assert prof.predict(1.0, 1.5) < prof.predict(1.0, 0.4)


class TestTheLatch:

    def test_it_holds_once_tripped(self):
        prof = profile()
        latch = StopRampLatch(prof)
        latch.update(prof.decide(0.05, 1.0))
        assert latch.latched
        assert latch.latched_d_rem == pytest.approx(0.05)
        # Jitter pushing d_rem back out must not restart the car.
        assert latch.update(prof.decide(3.0, 1.0)) is True

    def test_a_new_move_clears_it(self):
        prof = profile()
        latch = StopRampLatch(prof)
        latch.update(prof.decide(0.0, 1.0))
        latch.reset()
        assert latch.latched is False
        assert latch.latched_d_rem is None

    def test_it_does_not_trip_before_the_hand_over(self):
        prof = profile()
        latch = StopRampLatch(prof)
        assert latch.update(prof.decide(prof.d_final + 0.05, 1.0)) is False


class TestTheTuningIsValidated:

    @pytest.mark.parametrize('bad', (0.0, -0.6, float('nan'), float('inf')))
    def test_a_dec_must_be_a_deceleration(self, bad):
        with pytest.raises(ValueError, match='a_dec'):
            profile(a_dec=bad)

    @pytest.mark.parametrize('bad', (0.0, -1.5, float('nan')))
    def test_a_stop_must_be_a_deceleration(self, bad):
        with pytest.raises(ValueError, match='a_stop'):
            profile(a_stop=bad)

    @pytest.mark.parametrize('bad', (-0.1, float('nan'), float('inf')))
    def test_tau_must_be_a_time(self, bad):
        with pytest.raises(ValueError, match='tau_latency_s'):
            profile(tau_latency_s=bad)

    @pytest.mark.parametrize('bad', (0.0, -0.4, float('nan')))
    def test_v_min_must_be_a_speed(self, bad):
        with pytest.raises(ValueError, match='v_min'):
            profile(v_min=bad)

    def test_zero_latency_is_allowed(self):
        """A tau of 0 is "no compensation", which is a choice, not an error."""
        prof = profile(tau_latency_s=0.0)
        assert prof.predict(2.0, 1.5) == pytest.approx(2.0)
        assert prof.d_final == pytest.approx(V_MIN ** 2 / (2.0 * A_STOP))


class TestTheShippedTuning:

    def test_a_dec_is_gentler_than_the_car_can_stop(self):
        """a_dec is a request, a_stop a measurement; see the module docstring."""
        assert A_DEC < A_STOP

    def test_the_ramp_is_long_enough_to_be_felt(self):
        """At 1.5 m/s the cap starts binding more than a metre out."""
        onset = 1.5 ** 2 / (2.0 * A_DEC) + profile().d_final
        assert onset > 1.0

    def test_a_stop_is_the_value_fitted_to_the_archived_stops(self):
        assert A_STOP == pytest.approx(1.50)

    def test_tau_covers_the_measured_pipeline_delay(self):
        """0.144 s actuation dead time, ~0.19 s sensing; one conservative tau."""
        assert TAU >= 0.144

    def test_the_floor_is_the_stacks_own_floor(self):
        assert V_MIN == pytest.approx(0.4)
        assert math.isfinite(profile().d_final)
