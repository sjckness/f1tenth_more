"""/mpc/steering_model_diag -- the Foxglove live check added for the
2026-10-31 jamb-hit/wobble investigation.

Two independent things under test:

  _along_lateral_error: the along/lateral decomposition of the car's position
  against a pose-mode goal's own axis -- the same quantity f1tenth_behavior's
  enter_door.py computes as door_frame_xy, duplicated here only because the
  two packages do not share behaviour code. Suspect A's frame bug (a goal
  captured once in map, never reprojected into odom) shows up on this as a
  lateral error that jumps between ticks instead of staying quiet; this is
  what to watch in Foxglove tomorrow.

  _publish_steering_model_diag: predicted_yaw_rate must be the EXACT formula
  vehicle_model.f1tenth_state_fcn_dt_beta solves with (verified below against
  that real function, not re-derived by hand), so the published ratio against
  measured_yaw_rate is a direct instrument for suspect B (is the model the
  solver trusts the car it is actually driving).

Same "testable without constructing a real node" shape as the sibling test
files: a duck-typed stand-in, the REAL unbound method called against it. No
rclpy, no node, no topics, no hardware.

Run standalone: python3 -m pytest test/test_steering_model_diag.py -v
"""

import math
from types import SimpleNamespace

import pytest

from mpc_controller.MPC_corr import MPCController, _along_lateral_error
from mpc_controller.vehicle_model import f1tenth_state_fcn_dt_beta


class TestAlongLateralError:

    def test_a_point_dead_ahead_on_the_axis_is_pure_along(self):
        along, lateral = _along_lateral_error((3.0, 0.0), (0.0, 0.0), 0.0)
        assert along == pytest.approx(3.0)
        assert lateral == pytest.approx(0.0)

    def test_a_point_beside_the_origin_is_pure_lateral(self):
        """Left of the axis (REP-103: yaw 0 -> +y is left) is positive."""
        along, lateral = _along_lateral_error((0.0, 1.0), (0.0, 0.0), 0.0)
        assert along == pytest.approx(0.0)
        assert lateral == pytest.approx(1.0)

    def test_the_origin_itself_is_zero_both_ways(self):
        along, lateral = _along_lateral_error((2.0, -1.0), (2.0, -1.0), 0.7)
        assert along == pytest.approx(0.0, abs=1e-12)
        assert lateral == pytest.approx(0.0, abs=1e-12)

    def test_a_rotated_axis_rotates_the_decomposition_with_it(self):
        """Axis pointing straight up (+y): along becomes what used to be y,
        lateral becomes -x (left of +y is -x)."""
        along, lateral = _along_lateral_error(
            (1.0, 2.0), (0.0, 0.0), math.pi / 2.0)
        assert along == pytest.approx(2.0)
        assert lateral == pytest.approx(-1.0)

    def test_a_car_behind_the_opening_has_negative_along(self):
        """The car has not reached the goal yet -- same sign enter_door.py's
        own door_frame_xy uses (negative is short of the opening)."""
        along, _ = _along_lateral_error((-0.8, 0.0), (0.0, 0.0), 0.0)
        assert along == pytest.approx(-0.8)


class _FakePublisher:

    def __init__(self):
        self.published = []

    def publish(self, msg):
        self.published.append(msg)


class _FakeClock:
    def now(self):
        return SimpleNamespace(to_msg=lambda: SimpleNamespace(sec=0, nanosec=0))


class _FakeMPC:
    """Just enough of MPCController for _publish_steering_model_diag."""

    def __init__(self, x=0.0, y=0.0, v=1.5, yaw_rate=0.7,
                 goal_pose_xy=None, goal_pose_yaw=None, smooth_alpha=0.1):
        self.x, self.y, self.v = x, y, v
        self.yaw_rate = yaw_rate
        self.params = {'L': 0.305, 'lr': 0.17}
        self.goal_pose_xy = goal_pose_xy
        self.goal_pose_yaw = goal_pose_yaw
        self.steering_model_diag_smooth_alpha = smooth_alpha
        self._steering_model_ratio_ema = None
        self.steering_model_diag_pub = _FakePublisher()

    def get_clock(self):
        return _FakeClock()


def _kv(msg, key):
    (match,) = [kv.value for kv in msg.status[0].values if kv.key == key]
    return match


def _send(fake, speed, steering):
    MPCController._publish_steering_model_diag(fake, speed, steering)


class TestPredictedYawRateMatchesTheRealModel:

    def test_matches_vehicle_model_to_a_tight_tolerance(self):
        """Not re-derived by hand: integrated one tiny step of the REAL
        state function the solver uses and compared the resulting psidot."""
        v, delta, L, lr = 1.5, 0.18, 0.305, 0.17
        ts = 1e-7
        x1 = f1tenth_state_fcn_dt_beta([0.0, 0.0, 0.0, v], [delta, 0.0], ts, L, lr)
        psidot_numeric = x1[2] / ts

        fake = _FakeMPC(v=v, yaw_rate=psidot_numeric)
        _send(fake, v, delta)
        (msg,) = fake.steering_model_diag_pub.published
        assert float(_kv(msg, 'predicted_yaw_rate_rad_s')) == pytest.approx(
            psidot_numeric, rel=1e-5)

    def test_zero_steering_predicts_zero_yaw_rate(self):
        fake = _FakeMPC(v=1.0, yaw_rate=0.0)
        _send(fake, 1.0, 0.0)
        (msg,) = fake.steering_model_diag_pub.published
        assert float(_kv(msg, 'predicted_yaw_rate_rad_s')) == pytest.approx(0.0, abs=1e-12)


class TestRatioAndSmoothing:

    def test_agreement_gives_a_ratio_of_one(self):
        v, delta, L, lr = 1.5, 0.18, 0.305, 0.17
        beta = math.atan((lr / L) * math.tan(delta))
        yaw_rate_pred = (v / L) * math.cos(beta) * math.tan(delta)
        fake = _FakeMPC(v=v, yaw_rate=yaw_rate_pred)
        _send(fake, v, delta)
        (msg,) = fake.steering_model_diag_pub.published
        assert float(_kv(msg, 'yaw_rate_ratio')) == pytest.approx(1.0, rel=1e-6)

    def test_a_measured_rate_short_of_the_model_gives_the_offline_ratio(self):
        """Pins the ~0.84 figure another approach move's own analysis found:
        the measured rate running 0.84x the model's prediction."""
        v, delta, L, lr = 1.5, 0.18, 0.305, 0.17
        beta = math.atan((lr / L) * math.tan(delta))
        yaw_rate_pred = (v / L) * math.cos(beta) * math.tan(delta)
        fake = _FakeMPC(v=v, yaw_rate=0.84 * yaw_rate_pred)
        _send(fake, v, delta)
        (msg,) = fake.steering_model_diag_pub.published
        assert float(_kv(msg, 'yaw_rate_ratio')) == pytest.approx(0.84, rel=1e-6)

    def test_the_smoothed_ratio_is_an_ema_not_the_raw_value(self):
        fake = _FakeMPC(v=1.0, yaw_rate=0.0, smooth_alpha=0.5)
        # Ratio 2.0 every tick, from identical inputs each time.
        v, delta, L, lr = 1.0, 0.1, 0.305, 0.17
        beta = math.atan((lr / L) * math.tan(delta))
        pred = (v / L) * math.cos(beta) * math.tan(delta)
        fake.yaw_rate = 2.0 * pred
        _send(fake, v, delta)
        first = float(_kv(fake.steering_model_diag_pub.published[-1], 'yaw_rate_ratio_smoothed'))
        _send(fake, v, delta)
        second = float(_kv(fake.steering_model_diag_pub.published[-1], 'yaw_rate_ratio_smoothed'))
        assert first == pytest.approx(2.0)   # first sample seeds the EMA exactly
        assert second == pytest.approx(2.0)  # steady input -> EMA converges, does not overshoot

    def test_a_zero_predicted_rate_reports_nan_not_a_zero_division_crash(self):
        fake = _FakeMPC(v=1.0, yaw_rate=0.5)
        _send(fake, 1.0, 0.0)   # delta=0 -> yaw_rate_pred == 0.0
        (msg,) = fake.steering_model_diag_pub.published
        assert math.isnan(float(_kv(msg, 'yaw_rate_ratio')))

    def test_no_measured_yaw_rate_yet_reports_nan(self):
        fake = _FakeMPC(v=1.0, yaw_rate=None)
        _send(fake, 1.0, 0.1)
        (msg,) = fake.steering_model_diag_pub.published
        assert math.isnan(float(_kv(msg, 'measured_yaw_rate_rad_s')))
        assert math.isnan(float(_kv(msg, 'yaw_rate_ratio')))


class TestGoalFields:

    def test_no_pose_mode_goal_reports_nan_for_all_four_fields(self):
        fake = _FakeMPC(goal_pose_xy=None, goal_pose_yaw=None)
        _send(fake, 1.0, 0.1)
        (msg,) = fake.steering_model_diag_pub.published
        for key in ('goal_pose_x_odom', 'goal_pose_y_odom', 'goal_pose_yaw_odom',
                    'goal_along_error_m', 'goal_lateral_error_m'):
            assert math.isnan(float(_kv(msg, key))), key

    def test_an_active_pose_mode_goal_reports_the_real_decomposition(self):
        fake = _FakeMPC(x=0.0, y=0.5, goal_pose_xy=(3.0, 0.0), goal_pose_yaw=0.0)
        _send(fake, 1.0, 0.0)
        (msg,) = fake.steering_model_diag_pub.published
        assert float(_kv(msg, 'goal_pose_x_odom')) == pytest.approx(3.0)
        assert float(_kv(msg, 'goal_pose_y_odom')) == pytest.approx(0.0)
        # Car is BEHIND the goal (along axis 0.0) by 3 m, and 0.5 m to its left.
        assert float(_kv(msg, 'goal_along_error_m')) == pytest.approx(-3.0)
        assert float(_kv(msg, 'goal_lateral_error_m')) == pytest.approx(0.5)


class TestNotReadyGuards:

    def test_no_pose_yet_publishes_nothing(self):
        fake = _FakeMPC(x=None, y=None)
        _send(fake, 1.0, 0.1)
        assert fake.steering_model_diag_pub.published == []

    def test_no_speed_yet_publishes_nothing(self):
        fake = _FakeMPC(v=None)
        _send(fake, 1.0, 0.1)
        assert fake.steering_model_diag_pub.published == []


class TestDisabledByDefaultCostsNothing:

    def test_publish_drive_skips_the_diagnostic_when_disabled(self):
        import inspect
        src = inspect.getsource(MPCController._publish_drive)
        assert 'if self.steering_model_diag_enabled:' in src
        assert '_publish_steering_model_diag' in src.split(
            'if self.steering_model_diag_enabled:', 1)[1]

    def test_the_param_defaults_to_false(self):
        import inspect
        import re
        src = inspect.getsource(MPCController.__init__)
        # Whitespace-tolerant: only the param name and its False default matter.
        pattern = r"declare_parameter\(\s*'steering_model_diag_enabled',\s*False\)"
        assert re.search(pattern, src) is not None
