"""sensor_covariance_calibration_node.py unit-basis tests.

Regression cover for the 2026-09-10 gyro_variance_z correction. The bug: this
node samples the PUBLISHED /sensors/imu/raw topic, so the variance it wrote
silently inherited whatever unit vesc_driver.cpp happened to be emitting. The
2026-07-20 run measured a deg/s topic; gyro_scale_z (pi/180) went live in the
same commit that stored the result, so vesc.yaml ended up holding a deg^2/s^2
variance against a rad/s signal -- 3282.8x too large, nothing logged.

The node now divides the driver's gyro_scale_<axis> out of each sample and
re-applies scale^2 to the variance, so the value written is in the unit of the
message field it will be assigned to by construction, per axis.

Same convention as test_gyro_bias_calibration_node.py /
test_slam_pose_covariance_calibration_node.py: construct a real (never spun)
rclpy Node via parameter_overrides and call its callbacks/_finish() directly --
no live topics, no executor, no hardware.

Run standalone:
  python3 -m pytest test/test_sensor_covariance_calibration_node.py -v
"""
import math
import statistics

import pytest
import rclpy
from rclpy.parameter import Parameter
from sensor_msgs.msg import Imu

from f1tenth_diagnostics.sensor_covariance_calibration_node import (
    SensorCovarianceCalibrationNode,
)

DEG2RAD = math.pi / 180.0
# The exact rounded value vesc.yaml stores, not the full-precision constant --
# the node has to recognise this as "this axis is published in rad/s".
VESC_YAML_GYRO_SCALE_Z = 0.0174533

# A deg/s gyro noise sequence with a non-trivial spread, reused across tests so
# the expected variance is computed from one source.
RAW_DEG_PER_SEC = [0.20, -0.15, 0.05, -0.30, 0.11, 0.00, -0.07, 0.22]


@pytest.fixture(scope='module', autouse=True)
def _rclpy_context():
    rclpy.init()
    yield
    if rclpy.ok():
        rclpy.shutdown()


def _vesc_yaml(tmp_path, scale_x=1.0, scale_y=1.0, scale_z=VESC_YAML_GYRO_SCALE_Z):
    """Write a minimal vesc.yaml carrying just the keys the node reads back."""
    path = tmp_path / 'vesc.yaml'
    path.write_text(
        '/**:\n'
        '  ros__parameters:\n'
        f'    gyro_scale_x: {scale_x}\n'
        f'    gyro_scale_y: {scale_y}\n'
        f'    gyro_scale_z: {scale_z}\n'
        '    gyro_variance_x: 0.0\n'
        '    gyro_variance_y: 0.0\n'
        '    gyro_variance_z: 0.0\n'
        '    accel_variance_x: 0.0\n'
        '    accel_variance_y: 0.0\n'
        '    accel_variance_z: 0.0\n'
        'vesc_to_odom_node:\n'
        '  ros__parameters:\n'
        '    vx_variance: 0.0\n')
    return str(path)


def _construct(vesc_yaml_path, **extra):
    params = {'vesc_yaml_path': vesc_yaml_path, 'calibration_mode': 'stationary'}
    params.update(extra)
    overrides = [Parameter(k, value=v) for k, v in params.items()]
    return SensorCovarianceCalibrationNode(parameter_overrides=overrides)


def _imu(gyro_x=0.0, gyro_y=0.0, gyro_z=0.0, accel_z=1.0):
    msg = Imu()
    msg.angular_velocity.x = float(gyro_x)
    msg.angular_velocity.y = float(gyro_y)
    msg.angular_velocity.z = float(gyro_z)
    msg.linear_acceleration.z = float(accel_z)
    return msg


def _feed_axis(node, axis, values):
    """Push `values` at the node as the published value of one gyro axis."""
    for v in values:
        node._imu_callback(_imu(**{f'gyro_{axis}': v}))


def test_rad_scaled_z_axis_variance_is_written_in_rad2_not_deg2(tmp_path):
    """The actual 2026-09-10 bug: gyro_scale_z = pi/180 means the topic is
    rad/s, so the written variance must be rad^2/s^2 -- 3282.8x smaller than
    the deg^2/s^2 number the old code stored from the same samples."""
    node = _construct(_vesc_yaml(tmp_path))
    try:
        published_rad = [v * VESC_YAML_GYRO_SCALE_Z for v in RAW_DEG_PER_SEC]
        _feed_axis(node, 'z', published_rad)

        written = node._accumulators['gyro_variance_z'].variance()
        written *= node._gyro_scale['gyro_variance_z'] ** 2

        expected_rad2 = statistics.variance(published_rad)
        assert written == pytest.approx(expected_rad2, rel=1e-9)

        # And it is emphatically NOT the deg^2/s^2 figure for the same motion.
        expected_deg2 = statistics.variance(RAW_DEG_PER_SEC)
        assert expected_deg2 / written == pytest.approx((180.0 / math.pi) ** 2, rel=1e-3)
    finally:
        node.destroy_node()


def test_unscaled_x_and_y_axes_keep_deg2_variance_matched_to_their_deg_topic(tmp_path):
    """gyro_scale_x/y are 1.0, so those axes publish deg/s unconverted and
    their variances must stay deg^2/s^2 -- dividing them by (180/pi)^2 would
    introduce a mismatch that is not there today."""
    node = _construct(_vesc_yaml(tmp_path, scale_x=1.0, scale_y=1.0))
    try:
        _feed_axis(node, 'x', RAW_DEG_PER_SEC)
        expected_deg2 = statistics.variance(RAW_DEG_PER_SEC)

        written = node._accumulators['gyro_variance_x'].variance()
        written *= node._gyro_scale['gyro_variance_x'] ** 2

        assert written == pytest.approx(expected_deg2, rel=1e-12)
    finally:
        node.destroy_node()


def test_describe_unit_names_the_published_unit_from_the_scale(tmp_path):
    node = _construct(_vesc_yaml(tmp_path))
    try:
        assert node._describe_unit(VESC_YAML_GYRO_SCALE_Z) == 'rad/s'
        assert node._describe_unit(DEG2RAD) == 'rad/s'
        assert node._describe_unit(1.0) == 'deg/s'
        # Anything else must not be silently claimed as either unit.
        assert node._describe_unit(0.5) not in ('rad/s', 'deg/s')
    finally:
        node.destroy_node()


def test_finish_writes_each_gyro_axis_in_its_own_axis_unit(tmp_path):
    """End to end through _finish(): a mixed-scale driver (x/y deg/s, z rad/s)
    must produce a results dict whose three gyro entries are each matched to
    their own message field, not to one shared unit."""
    written = {}
    node = _construct(_vesc_yaml(tmp_path))
    try:
        # Installed before feeding: a non-None _timer makes _maybe_start_timer()
        # return early, which is what keeps the callbacks off the real arming
        # path (_first_message_watchdog only exists once _arm_sampling has run).
        node._timer = _FakeTimer()
        node._imu_sub = _FakeSub()
        node._odom_sub = _FakeSub()

        published_rad = [v * VESC_YAML_GYRO_SCALE_Z for v in RAW_DEG_PER_SEC]
        for deg, rad in zip(RAW_DEG_PER_SEC, published_rad):
            node._imu_callback(_imu(gyro_x=deg, gyro_y=deg, gyro_z=rad))
        for v in RAW_DEG_PER_SEC:
            node._odom_callback(_odom(v))
        import f1tenth_diagnostics.sensor_covariance_calibration_node as mod
        original_write = mod.write_vesc_yaml
        mod.write_vesc_yaml = lambda path, results, logger: written.update(results)
        try:
            node._finish()
        finally:
            mod.write_vesc_yaml = original_write

        assert written['gyro_variance_x'] == pytest.approx(
            statistics.variance(RAW_DEG_PER_SEC), rel=1e-12)
        assert written['gyro_variance_y'] == pytest.approx(
            statistics.variance(RAW_DEG_PER_SEC), rel=1e-12)
        assert written['gyro_variance_z'] == pytest.approx(
            statistics.variance(published_rad), rel=1e-9)
        # z must be far smaller than x/y for the same physical rotation.
        assert written['gyro_variance_z'] < written['gyro_variance_x'] / 1000.0
    finally:
        node.destroy_node()


def test_zero_gyro_scale_does_not_divide_by_zero(tmp_path):
    """A 0.0 scale means the axis is published as a constant 0 (variance 0 by
    construction) -- it must not raise ZeroDivisionError in the hot callback."""
    node = _construct(_vesc_yaml(tmp_path, scale_z=0.0))
    try:
        _feed_axis(node, 'z', [0.0] * len(RAW_DEG_PER_SEC))
        variance = node._accumulators['gyro_variance_z'].variance()
        assert variance * node._gyro_scale['gyro_variance_z'] ** 2 == pytest.approx(0.0)
    finally:
        node.destroy_node()


def test_unreadable_vesc_yaml_degrades_to_no_op_scales_not_a_crash(tmp_path):
    """A missing/garbled vesc.yaml must not abort an otherwise-good run: the
    scales fall back to 1.0, which reproduces this node's original behaviour
    of writing the topic's own unit."""
    node = _construct(str(tmp_path / 'does_not_exist.yaml'))
    try:
        assert node._gyro_scale == {
            'gyro_variance_x': 1.0,
            'gyro_variance_y': 1.0,
            'gyro_variance_z': 1.0,
        }
        _feed_axis(node, 'z', RAW_DEG_PER_SEC)
        assert node._accumulators['gyro_variance_z'].variance() == pytest.approx(
            statistics.variance(RAW_DEG_PER_SEC), rel=1e-12)
    finally:
        node.destroy_node()


# --- small stand-ins so _finish() can run without a live node --------------

class _FakeTimer:
    def cancel(self):
        pass


class _FakeSub:
    def destroy(self):
        pass


def _odom(vx):
    from nav_msgs.msg import Odometry
    msg = Odometry()
    msg.twist.twist.linear.x = float(vx)
    return msg
