"""swept_clearance_node.py tests -- a real (never spun) rclpy Node whose
callbacks are called directly with publishers mocked, the same convention as
test_lidar_front_wall_node.py.

These defend the wiring; test_swept_corridor.py and test_swept_clearance.py
cover the maths.
  * Both sensors reach clearance() through their static transforms, and a
    sensor without one stays stale rather than guessing.
  * The corridor follows the LAGGED command off /ackermann_drive by default.
  * Fusion: the smaller fresh value, a stale sensor left out, 0.0 when none.
  * Sensor subscriptions cannot back-pressure urg_node.
  * Parameter defaults equal stack_params.yaml's.

Run standalone: python3 -m pytest test/test_swept_clearance_node.py -v
"""

import math
from unittest.mock import MagicMock

import numpy as np
import pytest
import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import TransformStamped
from rclpy.parameter import Parameter
from rclpy.qos import ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image, LaserScan
from tf2_msgs.msg import TFMessage

from f1tenth_params.param_defaults import get_value
from f1tenth_perception.swept_clearance import quaternion_to_rotation
from f1tenth_perception.swept_clearance_node import SweptClearanceNode, depth_image_to_metres
from f1tenth_perception.swept_corridor import BODY_FRONT_X_M

ANGLE_MIN = -math.pi
ANGLE_INCREMENT = math.radians(0.25)
N_BEAMS = 1440
LASER_X = 0.12
CAMERA_T = (0.12, 0.0, 0.15)
OPTICAL_QUATERNION = (-0.5, 0.5, -0.5, 0.5)   # optical z forward, x right, y down
FULL_LOCK = get_value('mpc_steering_angle_max_rad')
MAX_RANGE = get_value('swept_clearance_max_range_m')

STACK_PARAMS = [
    'wheelbase_m', 'body_front_x_m', 'body_rear_x_m', 'body_half_width_m', 'margin_m',
    'max_range_m', 'absolute_min_clearance_m', 'rear_axle_x_m', 'steering_estimate',
    'steering_lag_sec', 'steering_topic', 'lidar_timeout_sec', 'camera_timeout_sec',
    'publish_rate_hz', 'camera_stride_px', 'camera_max_points', 'camera_min_depth_m',
    'camera_max_depth_m', 'camera_z_min_m', 'camera_z_max_m',
]


@pytest.fixture(scope='module', autouse=True)
def _rclpy_context():
    rclpy.init()
    yield
    if rclpy.ok():
        rclpy.shutdown()


class _Clock:
    def __init__(self):
        self.now = 100.0


@pytest.fixture
def make_node():
    created = []

    def make(**params):
        overrides = [Parameter(name, value=value) for name, value in params.items()]
        node = SweptClearanceNode(parameter_overrides=overrides)
        for publisher in (node.clearance_pub, node.lidar_pub, node.camera_pub, node.steering_pub):
            publisher.publish = MagicMock()
        clock = _Clock()
        node._now = lambda: clock.now
        node.test_clock = clock
        created.append(node)
        return node

    yield make
    for node in created:
        node.destroy_node()


def _last(publisher):
    return publisher.publish.call_args[0][0].data


def _transform(child, translation, quaternion=(0.0, 0.0, 0.0, 1.0)):
    t = TransformStamped()
    t.header.frame_id = 'base_link'
    t.child_frame_id = child
    t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = translation
    (t.transform.rotation.x, t.transform.rotation.y,
     t.transform.rotation.z, t.transform.rotation.w) = quaternion
    return t


def _tf_static():
    return TFMessage(transforms=[
        _transform('laser', (LASER_X, 0.0, 0.20)),
        _transform('zed2_left_camera_optical_frame', CAMERA_T, OPTICAL_QUATERNION),
    ])


def _scan(segments):
    """A full 360-degree scan in frame `laser` of line segments given in
    base_link ((x0, y0), (x1, y1)); beams that hit nothing return inf."""
    angles = ANGLE_MIN + np.arange(N_BEAMS) * ANGLE_INCREMENT
    d = np.stack([np.cos(angles), np.sin(angles)], 1)
    ranges = np.full(N_BEAMS, np.inf)
    origin = np.array([LASER_X, 0.0])
    for p0, p1 in segments:
        p0 = np.asarray(p0, dtype=float)
        e = np.asarray(p1, dtype=float) - p0
        w = p0 - origin
        denom = d[:, 0] * (-e[1]) - d[:, 1] * (-e[0])
        with np.errstate(divide='ignore', invalid='ignore'):
            t = (w[0] * (-e[1]) - w[1] * (-e[0])) / denom
            u = (d[:, 0] * w[1] - d[:, 1] * w[0]) / denom
        hit = (np.abs(denom) > 1e-12) & (t > 0.0) & (u >= 0.0) & (u <= 1.0)
        ranges = np.where(hit, np.minimum(ranges, t), ranges)
    msg = LaserScan()
    msg.header.frame_id = 'laser'
    msg.angle_min = ANGLE_MIN
    msg.angle_max = ANGLE_MIN + (N_BEAMS - 1) * ANGLE_INCREMENT
    msg.angle_increment = ANGLE_INCREMENT
    msg.range_min = 0.02
    msg.range_max = 30.0
    msg.ranges = [float(r) for r in ranges]
    return msg


def _drive(steering):
    msg = AckermannDriveStamped()
    msg.drive.steering_angle = float(steering)
    return msg


def _camera_info(width=64, height=48, f=60.0):
    msg = CameraInfo()
    msg.header.frame_id = 'zed2_left_camera_optical_frame'
    msg.width, msg.height = width, height
    msg.k = [f, 0.0, width / 2.0, 0.0, f, height / 2.0, 0.0, 0.0, 1.0]
    return msg


def _depth_image(depth, encoding='32FC1'):
    msg = Image()
    msg.header.frame_id = 'zed2_left_camera_optical_frame'
    msg.height, msg.width = depth.shape
    msg.encoding = encoding
    msg.is_bigendian = 0
    if encoding == '32FC1':
        msg.step = msg.width * 4
        msg.data = depth.astype('<f4').tobytes()
    else:
        msg.step = msg.width * 2
        msg.data = depth.astype('<u2').tobytes()
    return msg


FRONT_WALL_2M = [((BODY_FRONT_X_M + 2.0, -3.0), (BODY_FRONT_X_M + 2.0, 3.0))]
RIGHT_WALL_AHEAD = [((0.6, -0.6), (2.0, 0.3))]


class TestSensorPaths:

    def test_a_wall_ahead_through_the_laser_transform_reads_its_gap_to_the_bumper(self, make_node):
        node = make_node(use_camera=False)
        node._tf_static_cb(_tf_static())
        node._scan_cb(_scan(FRONT_WALL_2M))
        assert _last(node.lidar_pub) == pytest.approx(2.0, abs=1e-4)

    def test_without_the_laser_transform_the_scan_is_skipped_and_goes_stale(self, make_node):
        node = make_node(use_camera=False)
        node._scan_cb(_scan(FRONT_WALL_2M))
        node.lidar_pub.publish.assert_not_called()
        node._publish_fused()
        assert _last(node.clearance_pub) == 0.0

    def test_a_depth_frame_through_the_optical_transform_reads_its_gap(self, make_node):
        node = make_node(use_lidar=False, camera_stride_px=1)
        node._tf_static_cb(_tf_static())
        node._depth_info_cb(_camera_info())
        node._depth_cb(_depth_image(np.full((48, 64), 2.0, dtype=np.float32)))
        expected = CAMERA_T[0] + 2.0 - BODY_FRONT_X_M
        assert _last(node.camera_pub) == pytest.approx(expected, abs=1e-5)

    def test_depth_before_camera_info_is_skipped(self, make_node):
        node = make_node(use_lidar=False)
        node._tf_static_cb(_tf_static())
        node._depth_cb(_depth_image(np.full((48, 64), 2.0, dtype=np.float32)))
        node.camera_pub.publish.assert_not_called()

    def test_depth_below_the_height_band_like_the_floor_is_not_an_obstacle(self, make_node):
        node = make_node(use_lidar=False, camera_stride_px=1)
        node._tf_static_cb(_tf_static())
        node._depth_info_cb(_camera_info())
        # Each row at the depth where its ray meets the floor (z = 0 in base_link).
        rows = np.arange(48) - 24.0
        with np.errstate(divide='ignore'):
            floor_depth = np.where(rows > 0, CAMERA_T[2] * 60.0 / rows, np.nan)
        depth = np.repeat(floor_depth[:, None], 64, axis=1).astype(np.float32)
        node._depth_cb(_depth_image(depth))
        assert _last(node.camera_pub) == MAX_RANGE

    def test_16uc1_millimetres_become_metres_and_zero_becomes_no_data(self):
        raw = np.array([[2000, 0], [150, 65535]], dtype=np.uint16)
        metres = depth_image_to_metres(_depth_image(raw, encoding='16UC1'))
        np.testing.assert_allclose(metres, [[2.0, np.nan], [0.15, 65.535]])

    def test_the_optical_quaternion_is_the_zed_optical_rotation(self):
        np.testing.assert_allclose(quaternion_to_rotation(*OPTICAL_QUATERNION),
                                   [[0, 0, 1], [-1, 0, 0], [0, -1, 0]], atol=1e-12)


class TestSteering:

    def test_the_corridor_follows_the_lagged_command_not_the_newest(self, make_node):
        node = make_node(use_camera=False)
        node._tf_static_cb(_tf_static())
        clock = node.test_clock

        clock.now = 100.0
        node._steering_cb(_drive(FULL_LOCK))
        clock.now = 100.5
        node._scan_cb(_scan(RIGHT_WALL_AHEAD))
        assert _last(node.lidar_pub) == MAX_RANGE          # left lock curves away

        clock.now = 100.55
        node._steering_cb(_drive(0.0))
        clock.now = 100.6
        node._scan_cb(_scan(RIGHT_WALL_AHEAD))
        assert _last(node.lidar_pub) == MAX_RANGE          # wheels still at lock

        clock.now = 100.9
        node._scan_cb(_scan(RIGHT_WALL_AHEAD))
        assert _last(node.lidar_pub) < 1.0                 # now straight, into the wall

    @pytest.mark.parametrize('mode', ['latest_command', 'envelope'])
    def test_latest_and_envelope_see_the_wall_once_the_command_changes(self, make_node, mode):
        node = make_node(use_camera=False, steering_estimate=mode)
        node._tf_static_cb(_tf_static())
        clock = node.test_clock
        clock.now = 100.0
        node._steering_cb(_drive(FULL_LOCK))
        clock.now = 100.55
        node._steering_cb(_drive(0.0))
        clock.now = 100.6
        node._scan_cb(_scan(RIGHT_WALL_AHEAD))
        assert _last(node.lidar_pub) < 1.0

    def test_no_command_yet_uses_a_straight_corridor(self, make_node):
        node = make_node(use_camera=False)
        node._tf_static_cb(_tf_static())
        node._scan_cb(_scan(RIGHT_WALL_AHEAD))
        assert _last(node.lidar_pub) < 1.0
        node._publish_fused()
        assert math.isnan(_last(node.steering_pub))

    def test_the_published_steering_is_the_lagged_angle(self, make_node):
        node = make_node(use_camera=False)
        node.test_clock.now = 100.0
        node._steering_cb(_drive(0.2))
        node.test_clock.now = 100.1
        node._steering_cb(_drive(-0.1))
        node.test_clock.now = 100.35
        node._publish_fused()
        assert _last(node.steering_pub) == pytest.approx(0.2)

    def test_an_unknown_steering_estimate_is_rejected_at_startup(self, make_node):
        with pytest.raises(ValueError):
            make_node(steering_estimate='newest')


class TestFusion:

    def _both(self, make_node):
        node = make_node(camera_stride_px=1)
        node._tf_static_cb(_tf_static())
        node._depth_info_cb(_camera_info())
        node._steering_cb(_drive(0.0))
        return node

    def test_the_smaller_fresh_value_then_the_survivor_then_zero(self, make_node):
        node = self._both(make_node)
        clock = node.test_clock
        clock.now = 100.0
        node._scan_cb(_scan(FRONT_WALL_2M))                                    # 2.0
        node._depth_cb(_depth_image(np.full((48, 64), 1.0, dtype=np.float32)))  # 0.677
        node._publish_fused()
        assert _last(node.clearance_pub) == pytest.approx(CAMERA_T[0] + 1.0 - BODY_FRONT_X_M,
                                                          abs=1e-5)

        clock.now = 100.4          # lidar (0.25 s) stale, camera (0.5 s) fresh
        node._publish_fused()
        assert _last(node.clearance_pub) == pytest.approx(CAMERA_T[0] + 1.0 - BODY_FRONT_X_M,
                                                          abs=1e-5)

        node._scan_cb(_scan(FRONT_WALL_2M))
        clock.now = 100.6          # camera stale, lidar fresh again
        node._publish_fused()
        assert _last(node.clearance_pub) == pytest.approx(2.0, abs=1e-4)

        clock.now = 101.0          # both stale
        node._publish_fused()
        assert _last(node.clearance_pub) == 0.0

    def test_a_disabled_camera_has_no_subscription_and_is_not_fused(self, make_node):
        node = make_node(use_camera=False)
        assert node.depth_sub is None and node.depth_info_sub is None
        node._tf_static_cb(_tf_static())
        node._scan_cb(_scan(FRONT_WALL_2M))
        node._publish_fused()
        assert _last(node.clearance_pub) == pytest.approx(2.0, abs=1e-4)

    def test_no_sensor_enabled_publishes_zero(self, make_node):
        node = make_node(use_lidar=False, use_camera=False)
        node._publish_fused()
        assert _last(node.clearance_pub) == 0.0


class TestDecouplingAndDefaults:

    def test_sensor_subscriptions_are_best_effort(self, make_node):
        node = make_node()
        for sub in (node.scan_sub, node.depth_sub, node.depth_info_sub):
            assert sub.qos_profile.reliability == ReliabilityPolicy.BEST_EFFORT

    @pytest.mark.parametrize('name', STACK_PARAMS)
    def test_parameter_default_matches_stack_params(self, make_node, name):
        assert make_node().get_parameter(name).value == get_value('swept_clearance_' + name)
