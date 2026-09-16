"""object_guard.py -- the steering ramp-out and the refresh watchdog.

Ported from the 2026-09-15 go_to_object prototype's tests
(tests/test_pursuit_geometry.py's CurvatureLimiter block and
tests/test_mission_state.py's LOST-ramp and watchdog blocks), re-expressed on
steering angle instead of curvature. The prototype tracker and its tests were
not ported. Pure: no rclpy, no node.

Run standalone: python3 -m pytest test/test_object_guard.py -v
"""

import pytest

from mpc_controller.object_guard import RefreshWatchdog, SteeringRamp


# -- the ramp ----------------------------------------------------------------

def test_it_starts_synchronised_to_centred_wheels_and_limits_immediately():
    ramp = SteeringRamp(0.6)
    assert ramp.previous == 0.0
    assert ramp.apply(0.25, 0.1) == pytest.approx(0.06)


def test_it_honours_an_explicit_cold_start_seed():
    ramp = SteeringRamp(0.6, initial=-0.2)
    assert ramp.apply(0.0, 0.1) == pytest.approx(-0.14)


@pytest.mark.parametrize('rate, dt, target', [
    (0.6, 0.1, 0.28), (0.6, 0.05, -0.28), (2.0, 0.1, 0.28), (0.1, 0.01, -0.28)])
def test_every_step_is_bounded_by_rate_times_dt(rate, dt, target):
    ramp = SteeringRamp(rate)
    out = [0.0]
    for _ in range(100):
        out.append(ramp.apply(target, dt))
    steps = [abs(b - a) for a, b in zip(out, out[1:])]
    assert max(steps) <= rate * dt + 1e-12
    assert out[-1] == pytest.approx(target) or len(out) * rate * dt < abs(target)


def test_steps_smaller_than_the_limit_pass_unchanged():
    ramp = SteeringRamp(0.6)
    assert ramp.apply(0.03, 0.1) == pytest.approx(0.03)
    assert ramp.saturated is False


def test_it_reaches_the_target_in_the_expected_number_of_ticks():
    ramp = SteeringRamp(0.6, initial=0.28)
    ticks = 0
    while ramp.apply(0.0, 0.1) != 0.0:
        ticks += 1
    assert ticks + 1 == 5          # ceil(0.28 / 0.06)


def test_dt_zero_or_negative_holds_the_previous_value():
    """The first tick after a mode change has no elapsed time; it must not step."""
    for dt in (0.0, -0.1):
        ramp = SteeringRamp(0.6, initial=0.2)
        assert ramp.apply(0.0, dt) == 0.2
        assert ramp.saturated is True


def test_resync_without_a_measurement_keeps_the_last_output():
    ramp = SteeringRamp(0.6)
    ramp.apply(0.25, 1.0)
    assert ramp.resync(None) == pytest.approx(0.25)


def test_resync_with_a_measurement_overrides_the_last_output():
    ramp = SteeringRamp(0.6)
    ramp.apply(0.25, 1.0)
    assert ramp.resync(-0.31) == pytest.approx(-0.31)
    assert ramp.apply(0.0, 0.1) == pytest.approx(-0.25)


def test_a_sign_reversal_is_a_ramp_not_a_step():
    """The prototype's Part A failure: hard left, then hard right."""
    ramp = SteeringRamp(0.6, initial=0.28)
    out = [0.28] + [ramp.apply(-0.28, 0.1) for _ in range(12)]
    steps = [abs(b - a) for a, b in zip(out, out[1:])]
    assert max(steps) <= 0.06 + 1e-12
    assert out[-1] == pytest.approx(-0.28)


def test_it_counts_its_own_saturations():
    ramp = SteeringRamp(0.6)
    ramp.apply(0.28, 0.1)
    ramp.apply(0.28, 0.1)
    ramp.apply(0.13, 0.1)
    assert (ramp.applications, ramp.saturations) == (3, 2)


def test_a_non_positive_rate_is_rejected():
    with pytest.raises(ValueError):
        SteeringRamp(0.0)


# -- the watchdog ------------------------------------------------------------

def test_the_watchdog_fires_when_refreshes_stop_arriving():
    dog = RefreshWatchdog(0.5)
    for i in range(10):
        dog.note(i * 0.1)
    assert dog.tripped(0.9 + 0.5) is None, 'still inside the timeout'
    reason = dog.tripped(0.9 + 0.51)
    assert reason is not None and 'no ObjectGoal refresh' in reason


def test_a_healthy_stream_never_trips_it():
    dog = RefreshWatchdog(0.5)
    for i in range(500):
        dog.note(i * 0.1)
        assert dog.tripped(i * 0.1 + 0.09) is None


def test_the_watchdog_recovers_when_refreshes_return():
    dog = RefreshWatchdog(0.5)
    dog.note(0.0)
    assert dog.tripped(1.0) is not None
    dog.note(1.01)
    assert dog.tripped(1.02) is None


def test_never_refreshed_is_not_tripped_and_reset_forgets():
    dog = RefreshWatchdog(0.5)
    assert dog.tripped(100.0) is None
    dog.note(0.0)
    dog.reset()
    assert dog.tripped(100.0) is None


def test_a_non_positive_timeout_is_rejected():
    with pytest.raises(ValueError):
        RefreshWatchdog(0.0)
