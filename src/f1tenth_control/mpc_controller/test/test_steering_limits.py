"""The MPC's steering envelope must agree with the servo calibration.

WHAT THIS PINS, and why it is a test rather than a comment.

delta_min/delta_max in MPC_corr's self.limits are not free parameters. They
are the inverse of the command path the car actually has:

    ackermann_to_vesc.cpp   servo = gain_side * steering_angle + offset
                            (gain_left for angle >= 0, gain_right below)
    vesc_driver.cpp         servo = clip(servo, servo_min, servo_max)

so the largest steering angle the QP may usefully command is the one that
lands exactly on a clip bound. Anything beyond it is a command the servo
cannot produce, and the clip is SILENT: the solver's own model integrates
the angle it chose, not the angle the car applied.

All five inputs to that inversion live in f1tenth_hardware's
steering_calibration.yaml, and four of them are expected to move -- the
offset was retuned 2026-09-08 (0.4494 -> 0.4874), and
steering_angle_to_servo_gain_left/_right moved off their shared -1.2135
placeholder on 2026-09-17 (-1.0552 / -1.1133, fitted from logged turns).
A recalibration that leaves stack_params.yaml behind puts the QP back to
optimising over commands the servo will clip, which is exactly the defect the limits were changed to fix -- and it would do so
silently, because nothing else in the stack compares the two files.

So this test recomputes the envelope from the calibration and fails if
stack_params.yaml has drifted from it. Per CLAUDE.md: when a test encodes a
config default it is a coupling, and this one is deliberate and named.

NOT A MECHANICAL MEASUREMENT. This checks that two config files agree about
the arithmetic. Neither file is grounded in an instrument reading of the
linkage; if the calibration itself is wrong, both will be wrong together and
this test will happily pass.

Run standalone: python3 -m pytest test/test_steering_limits.py -v
"""

import math

import pytest
import yaml
from ament_index_python.packages import get_package_share_directory

from f1tenth_params.param_defaults import get_value

# Inward rounding budget. The yaml values are the exact inversion rounded
# INWARD (toward zero) to three decimals, so a command at the limit is one
# the servo can actually produce rather than one that merely reaches the
# clip. Half a milliradian covers that rounding and nothing else -- it is
# not a tolerance for the values disagreeing.
_ROUNDING_M_RAD = 1.0e-3


def _calibration():
    path = (
        get_package_share_directory('f1tenth_hardware')
        + '/config/steering_calibration.yaml'
    )
    with open(path) as handle:
        params = yaml.safe_load(handle)['/**']['ros__parameters']
    return params


def _envelope(cal):
    """(min_angle, max_angle) reachable through the clip, in radians.

    Positive angles use gain_left, negative use gain_right (see
    ackermann_to_vesc.cpp). Both gains are negative on this car, so the
    servo_min end is the POSITIVE steering limit and servo_max the negative
    one -- the inversion flips the order, which is easy to get backwards and
    is the reason this is computed rather than transcribed.
    """
    offset = float(cal['steering_angle_to_servo_offset'])
    gain_left = float(cal['steering_angle_to_servo_gain_left'])
    gain_right = float(cal['steering_angle_to_servo_gain_right'])
    servo_min = float(cal['servo_min'])
    servo_max = float(cal['servo_max'])

    at_servo_min = (servo_min - offset) / gain_left
    at_servo_max = (servo_max - offset) / gain_right
    return min(at_servo_min, at_servo_max), max(at_servo_min, at_servo_max)


class TestTheEnvelopeMatchesTheCalibration:

    def test_stack_params_agrees_with_steering_calibration(self):
        lo, hi = _envelope(_calibration())
        yaml_min = float(get_value('mpc_steering_angle_min_rad'))
        yaml_max = float(get_value('mpc_steering_angle_max_rad'))

        assert yaml_min == pytest.approx(lo, abs=_ROUNDING_M_RAD), (
            'mpc_steering_angle_min_rad has drifted from the servo '
            'calibration -- recompute it, or the QP is optimising over '
            'commands the servo will clip')
        assert yaml_max == pytest.approx(hi, abs=_ROUNDING_M_RAD), (
            'mpc_steering_angle_max_rad has drifted from the servo '
            'calibration -- see above')

    def test_the_limits_are_rounded_inward_never_outward(self):
        """A limit past the clip is the bug; a limit inside it is safe."""
        lo, hi = _envelope(_calibration())
        assert float(get_value('mpc_steering_angle_min_rad')) >= lo
        assert float(get_value('mpc_steering_angle_max_rad')) <= hi

    def test_the_old_literal_was_far_outside_the_servos_reach(self):
        """The defect, stated as a number rather than a claim.

        1.05 rad was the previous hardcoded bound. Documenting the ratio
        here so the size of the mismatch survives in the test suite and not
        only in a commit message. It was 3.7x against the placeholder gains;
        the 2026-09-17 gains widened the envelope, and it is 3.3x now.
        """
        lo, hi = _envelope(_calibration())
        assert 1.05 / hi > 3.2
        assert 1.05 / abs(lo) > 3.2


class TestTheWorkOrdersPairIsStillNotUsed:
    """-0.264 / +0.314 rad were cited as the real limits. They are not used.

    They appear in this repo only as declared parameter defaults in
    f1tenth_diagnostics' steering_offset_calibration_node.py, whose docstring
    calls them "the real ones for this car" with no derivation recorded.

    Against the old -1.2135 placeholder gains they were rejected outright:
    +0.314 mapped below servo_min, and the asymmetry pointed the other way.
    The gains fitted from logged turns on 2026-09-17 overturned both of those
    reasons, and this class pins what is true now, so nobody re-derives the
    old argument from a stale comment. The envelope stays computed from the
    calibration rather than transcribed from that pair.
    """

    WORK_ORDER_MIN = -0.264
    WORK_ORDER_MAX = 0.314

    def test_the_claimed_positive_limit_is_commandable_and_inside_the_envelope(self):
        """+0.314 rad now maps to servo 0.156, above servo_min, just inside
        the +0.3197 rad reach."""
        cal = _calibration()
        servo = (
            float(cal['steering_angle_to_servo_gain_left']) * self.WORK_ORDER_MAX
            + float(cal['steering_angle_to_servo_offset'])
        )
        assert servo >= float(cal['servo_min'])
        _lo, hi = _envelope(cal)
        assert self.WORK_ORDER_MAX < hi

    def test_the_claimed_asymmetry_now_points_the_same_way(self):
        """Both the work order and the fitted calibration give the left more
        travel (+18.32 vs -17.72 deg)."""
        lo, hi = _envelope(_calibration())
        assert abs(self.WORK_ORDER_MAX) > abs(self.WORK_ORDER_MIN)
        assert abs(hi) > abs(lo)

    def test_the_asymmetry_is_small_and_comes_from_the_fitted_gains(self):
        """The two gains now differ (left 1.15x, right 1.09x of the model's
        curvature before the fit), so the asymmetry is the gains' as well as
        the offset's. Still about a hundredth of a radian."""
        cal = _calibration()
        assert (cal['steering_angle_to_servo_gain_left']
                != cal['steering_angle_to_servo_gain_right'])
        lo, hi = _envelope(cal)
        assert abs(abs(lo) - abs(hi)) < 0.02

    def test_the_work_order_pair_is_a_narrower_span(self):
        """-0.264 is 45 mrad inside the right-hand reach, so applying the
        pair would now give up steering the servo can deliver."""
        lo, hi = _envelope(_calibration())
        claimed_span = self.WORK_ORDER_MAX - self.WORK_ORDER_MIN
        real_span = hi - lo
        assert claimed_span < real_span


class TestDegreesForTheRecord:
    """The envelope in degrees, so a human reading a failure sees a number
    they can compare against a protractor on the actual car."""

    def test_the_envelope_in_degrees(self):
        lo, hi = _envelope(_calibration())
        # -16.26 / +15.93 deg before the 2026-09-17 steering gains.
        assert math.degrees(lo) == pytest.approx(-17.72, abs=0.05)
        assert math.degrees(hi) == pytest.approx(18.32, abs=0.05)
