"""drive_bridge's (speed, steering_angle) -> (v, omega) conversion."""
import math

import pytest

from f1tenth_sim.kinematics import (
    ackermann_to_twist,
    clamp_steering,
    STEERING_MAX_RAD,
    STEERING_MIN_RAD,
)

L = 0.325


@pytest.mark.parametrize('delta', [-0.28, -0.1, 0.0, 0.05, 0.27])
@pytest.mark.parametrize('v', [-1.0, 0.3, 2.5])
def test_controller_inverse_recovers_steering_inside_envelope(v, delta):
    # ackermann_steering_controller turns the twist back into
    # delta = atan(omega * L / v); inside the envelope that must be exact.
    out_v, omega = ackermann_to_twist(v, delta, L)
    assert out_v == v
    assert math.atan(omega * L / out_v) == pytest.approx(delta, abs=1e-12)


def test_left_steering_beyond_servo_limit_clamps_to_plus_0_2780():
    assert clamp_steering(0.4) == STEERING_MAX_RAD == 0.2780


def test_right_steering_beyond_servo_limit_clamps_to_minus_0_2838():
    assert clamp_steering(-0.4) == STEERING_MIN_RAD == -0.2838


def test_clamped_steering_is_what_reaches_the_twist():
    _, omega = ackermann_to_twist(1.0, 0.4, L)
    assert omega == pytest.approx(math.tan(STEERING_MAX_RAD) / L)


def test_positive_steering_forward_gives_positive_yaw_rate():
    # REP-103: + steering_angle = left, so forward + left is a CCW (+z) turn.
    assert ackermann_to_twist(1.0, 0.2, L)[1] > 0.0


def test_reversing_with_left_steering_gives_negative_yaw_rate():
    assert ackermann_to_twist(-1.0, 0.2, L)[1] < 0.0


def test_standstill_steering_is_lost():
    # Documented limitation: a twist cannot carry steering at v == 0.
    assert ackermann_to_twist(0.0, 0.2, L) == (0.0, 0.0)
