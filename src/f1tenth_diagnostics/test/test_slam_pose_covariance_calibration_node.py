"""slam_pose_covariance_calibration_node.py tests -- constructs a real
(but never spun) rclpy Node and calls its callbacks/_finish() directly,
same "no live topics/hardware needed, pure in-process method calls"
convention this workspace already uses for testing node-level logic without
a live ROS graph. Covers the two things this pass's own task explicitly
calls out: variance computation from synthetic samples, and the honest-
failure "no data -> clean non-zero exit, no write" path -- named explicitly
below (test_no_data_exits_cleanly_without_writing) since it's the single
most important case given /slam/pose's own currently-known-broken state.

Run standalone: python3 -m pytest test/test_slam_pose_covariance_calibration_node.py -v
"""

import math
import statistics
from types import SimpleNamespace

import pytest
import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped

from f1tenth_diagnostics.calibration_common import (
    EXIT_INSUFFICIENT_SAMPLES,
    EXIT_SUCCESS,
    Welford,
)
from f1tenth_diagnostics.slam_pose_covariance_calibration_node import (
    SlamPoseCovarianceCalibrationNode,
    _yaw_from_quaternion,
)


@pytest.fixture(scope='module', autouse=True)
def _rclpy_context():
    rclpy.init()
    yield
    if rclpy.ok():
        rclpy.shutdown()


def _construct_with_params(param_dict):
    """rclpy.node.Node doesn't take a plain dict of parameter overrides at
    construction time the way launch's parameters=[{...}] does -- the
    equivalent for a directly-constructed test Node is `parameter_overrides`
    (a list of rclpy.parameter.Parameter). Built here once so each test
    doesn't repeat the conversion."""
    from rclpy.parameter import Parameter
    overrides = [Parameter(k, value=v) for k, v in param_dict.items()]
    return SlamPoseCovarianceCalibrationNode(parameter_overrides=overrides)


def _fake_pose(x, y, yaw):
    msg = PoseWithCovarianceStamped()
    msg.pose.pose.position.x = x
    msg.pose.pose.position.y = y
    msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
    msg.pose.pose.orientation.w = math.cos(yaw / 2.0)
    return msg


_YAML_TEMPLATE = """\
slam_pose_relay_node:
  ros__parameters:
    input_topic: /slam/pose
    output_topic: /slam/pose_calibrated
    pose_variance_x: 0.1
    pose_variance_y: 0.1
    pose_variance_yaw: 0.05
ekf_global_filter_node:
  ros__parameters:
    frequency: 50.0
"""


# ==============================================================================
# _yaw_from_quaternion -- pure function
# ==============================================================================

class TestYawFromQuaternion:

    def test_identity_is_zero_yaw(self):
        q = SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0)
        assert _yaw_from_quaternion(q) == pytest.approx(0.0, abs=1e-9)

    def test_recovers_a_known_yaw(self):
        yaw = math.radians(37.0)
        q = SimpleNamespace(x=0.0, y=0.0, z=math.sin(yaw / 2.0), w=math.cos(yaw / 2.0))
        assert _yaw_from_quaternion(q) == pytest.approx(yaw)


# ==============================================================================
# Welford -- variance computation from synthetic samples (shared accumulator,
# see calibration_common.py's own module docstring for why this moved there)
# ==============================================================================

class TestWelfordVariance:

    def test_matches_statistics_module_on_synthetic_samples(self):
        samples = [1.0, 2.0, 3.0, 4.0, 5.0, 2.5, 3.5]
        acc = Welford()
        for s in samples:
            acc.update(s)
        assert acc.mean == pytest.approx(statistics.mean(samples))
        assert acc.variance() == pytest.approx(statistics.variance(samples))

    def test_zero_variance_for_identical_samples(self):
        acc = Welford()
        for _ in range(10):
            acc.update(5.0)
        assert acc.variance() == pytest.approx(0.0, abs=1e-12)


# ==============================================================================
# SlamPoseCovarianceCalibrationNode -- _finish() decision logic
# ==============================================================================

class TestFinish:

    def test_honest_failure_no_data_exits_cleanly_without_writing(self, tmp_path):
        """THE MOST IMPORTANT CASE given /slam/pose's own currently-known-
        broken (never publishes) state -- see module docstring. Zero samples
        received (as if the sampling window elapsed with nothing on
        pose_topic at all) must exit EXIT_INSUFFICIENT_SAMPLES, set done,
        and -- critically -- must NOT create/write the target yaml file at
        all (write_yaml_config is never even called at all in this path).

        Matches the REAL control flow for "pose_topic never publishes a
        single message" -- NOT _finish() (which requires a first message to
        have already armed self._timer/self._pose_sub -- see _arm_sampling())
        but _on_first_message_timeout(), the watchdog that fires when
        first_message_timeout_sec elapses with zero messages received. This
        is the actual code path a live run hits today, given /slam/pose's
        own currently-known-broken (never publishes) state."""
        yaml_path = tmp_path / 'ekf_global.yaml'
        yaml_path.write_text(_YAML_TEMPLATE)
        before_mtime = yaml_path.stat().st_mtime_ns
        before_content = yaml_path.read_text()

        node = _construct_with_params({'ekf_global_yaml_path': str(yaml_path)})
        try:
            node._arm_sampling()  # stationary confirmed -- subscribes, arms the watchdog
            node._on_first_message_timeout()  # simulate: watchdog fires, zero messages ever

            assert node.done is True
            assert node.exit_code == EXIT_INSUFFICIENT_SAMPLES
            assert yaml_path.stat().st_mtime_ns == before_mtime
            assert yaml_path.read_text() == before_content
            # No backup file created either -- confirms write_yaml_config's
            # own backup-then-patch path was never entered at all.
            assert list(tmp_path.glob('*.bak.*')) == []
        finally:
            node.destroy_node()

    def test_single_sample_is_still_insufficient(self, tmp_path):
        """n=1 -- Welford.variance() is undefined below n=2 (see its own
        docstring); the insufficient-samples guard must catch this BEFORE
        ever calling .variance(), not divide by zero. Reachable via _finish()
        for real: one message arrives (arming self._timer via _pose_callback's
        own first-message bookkeeping), then the sample_duration_sec window
        elapses with no further messages."""
        yaml_path = tmp_path / 'ekf_global.yaml'
        yaml_path.write_text(_YAML_TEMPLATE)

        node = _construct_with_params({'ekf_global_yaml_path': str(yaml_path)})
        try:
            node._arm_sampling()
            node._pose_callback(_fake_pose(1.0, 1.0, 0.0))  # arms self._timer for real
            node._finish()

            assert node.exit_code == EXIT_INSUFFICIENT_SAMPLES
            assert node.done is True
        finally:
            node.destroy_node()

    def test_sufficient_samples_computes_and_writes_real_variance(self, tmp_path):
        """Enough synthetic samples to clear the n>=2 floor -- confirms the
        full success path: correct variance computed AND actually written
        into the target yaml file's slam_pose_relay_node section (the real,
        functional destination -- see slam_pose_relay_node.py's own module
        docstring for why it's not a robot_localization param directly)."""
        yaml_path = tmp_path / 'ekf_global.yaml'
        yaml_path.write_text(_YAML_TEMPLATE)

        node = _construct_with_params({'ekf_global_yaml_path': str(yaml_path)})
        try:
            node._arm_sampling()
            xs = [1.00, 1.02, 0.99, 1.01, 1.03, 0.98]
            ys = [2.00, 2.01, 1.99, 2.02, 1.98, 2.00]
            yaws = [0.10, 0.11, 0.09, 0.10, 0.12, 0.08]
            for x, y, yaw in zip(xs, ys, yaws):
                node._pose_callback(_fake_pose(x, y, yaw))
            # _pose_callback's own first-message bookkeeping starts a real
            # rclpy Timer (self._timer) on the first call -- calling _finish()
            # directly here (instead of waiting for it to actually fire)
            # mirrors what that timer's own callback would do.
            node._finish()

            assert node.exit_code == EXIT_SUCCESS
            assert node.done is True

            expected_var_x = statistics.variance(xs)
            expected_var_y = statistics.variance(ys)
            expected_var_yaw = statistics.variance(yaws)

            written = yaml_path.read_text()
            assert 'pose_variance_x: ' in written

            import ruamel.yaml
            loaded = ruamel.yaml.YAML().load(yaml_path.read_text())
            section = loaded['slam_pose_relay_node']['ros__parameters']
            assert float(section['pose_variance_x']) == pytest.approx(expected_var_x)
            assert float(section['pose_variance_y']) == pytest.approx(expected_var_y)
            assert float(section['pose_variance_yaw']) == pytest.approx(expected_var_yaw)
            # ekf_global_filter_node's own, untouched section/keys must
            # survive the round-trip patch unchanged (ruamel.yaml's whole
            # point -- see write_yaml_config's own docstring).
            assert loaded['ekf_global_filter_node']['ros__parameters']['frequency'] == 50.0

            # Exactly one backup created for this one write.
            backups = list(tmp_path.glob('ekf_global.yaml.bak.*'))
            assert len(backups) == 1
        finally:
            # _finish() already cancelled self._timer -- just clean up the node.
            node.destroy_node()
