"""lidar_front_wall_node.py tests -- a real (never spun) rclpy Node whose
callbacks are called directly with publishers mocked, the same convention as
test_front_clearance_node.py.

These defend the wire contract; test_lidar_front_wall.py covers the maths.
  * Both topics publish on EVERY scan: before the laser transform is known,
    when the fit is rejected, and when there is no value at all. A topic that
    goes quiet cannot be told apart from a dead node.
  * The scan subscription is best-effort, so this node cannot back-pressure
    urg_node, the e-stop's only /scan publisher.
  * The message constants and the pure module's codes agree.
  * The node's parameter defaults equal stack_params.yaml's, and its odometry
    is the local EKF by decision rather than get_odom_topic().

Run standalone: python3 -m pytest test/test_lidar_front_wall_node.py -v
"""

import math
from unittest.mock import MagicMock

import numpy as np
import pytest
import rclpy
from f1tenth_messages.msg import WallEstimate, WallLineFit
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.qos import ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32
from tf2_msgs.msg import TFMessage

from f1tenth_params.param_defaults import get_value
from f1tenth_perception import lidar_front_wall as logic
from f1tenth_perception.lidar_front_wall_node import LidarFrontWallNode

ANGLE_MIN = float(np.float32(-2.356194496154785))
ANGLE_INCREMENT = float(np.float32(0.004363323096185923))
N_BEAMS = 1081
LASER_X = 0.12

TUNING_PARAMS = [
    'sector_half_angle_deg', 'inlier_distance_m', 'min_inlier_fraction',
    'min_inlier_count_ratio', 'oblique_max_deg', 'max_hypothesis_pairs',
    'wrong_surface_gate_m', 'stale_age_sec', 'odom_topic', 'odom_max_age_sec',
    'seed_max_age_sec', 'seed_max_valid_m',
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
def node():
    n = LidarFrontWallNode()
    n.fit_pub.publish = MagicMock()
    n.estimate_pub.publish = MagicMock()
    clock = _Clock()
    n._now = lambda: clock.now
    n.test_clock = clock
    yield n
    n.destroy_node()


def _last(publisher):
    return publisher.publish.call_args[0][0]


def _tf_static():
    t = TransformStamped()
    t.header.frame_id = 'base_link'
    t.child_frame_id = 'laser'
    t.transform.translation.x = LASER_X
    t.transform.rotation.w = 1.0
    return TFMessage(transforms=[t])


def _scan(distance=None, stamp_sec=7):
    """A full scan in frame `laser`: a wall square to base_link at `distance`
    [m], or no returns at all when distance is None."""
    msg = LaserScan()
    msg.header.frame_id = 'laser'
    msg.header.stamp.sec = stamp_sec
    msg.angle_min = ANGLE_MIN
    msg.angle_max = -ANGLE_MIN
    msg.angle_increment = ANGLE_INCREMENT
    msg.range_min = 0.02
    msg.range_max = 30.0
    angles = ANGLE_MIN + np.arange(N_BEAMS) * ANGLE_INCREMENT
    ranges = np.full(N_BEAMS, np.inf)
    if distance is not None:
        ahead = np.cos(angles) > 0.05
        ranges[ahead] = (distance - LASER_X) / np.cos(angles[ahead])
    msg.ranges = [float(r) for r in ranges.astype(np.float32)]
    return msg


def _odom(x, y=0.0):
    msg = Odometry()
    msg.pose.pose.position.x = x
    msg.pose.pose.position.y = y
    msg.pose.pose.orientation.w = 1.0
    return msg


class TestEveryScanPublishesBothTopics:

    def test_before_the_laser_transform_is_known(self, node):
        node._scan_cb(_scan(2.0))
        assert node.fit_pub.publish.call_count == 1
        assert node.estimate_pub.publish.call_count == 1
        fit, estimate = _last(node.fit_pub), _last(node.estimate_pub)
        assert fit.reason == WallLineFit.REASON_NO_TRANSFORM and not fit.valid
        assert estimate.provenance == WallEstimate.PROVENANCE_NONE and not estimate.valid
        assert math.isnan(estimate.distance)

    def test_with_no_returns_in_the_sector(self, node):
        node._tf_static_cb(_tf_static())
        node._scan_cb(_scan(None))
        assert node.estimate_pub.publish.call_count == 1
        fit = _last(node.fit_pub)
        assert fit.reason == WallLineFit.REASON_TOO_FEW_RETURNS
        assert fit.sector_beams == 81 and fit.valid_returns == 0
        assert math.isnan(fit.distance)

    def test_on_every_one_of_consecutive_scans(self, node):
        node._tf_static_cb(_tf_static())
        for i in range(5):
            node.test_clock.now += 0.025
            node._scan_cb(_scan(2.0 if i % 2 else None))
        assert node.fit_pub.publish.call_count == 5
        assert node.estimate_pub.publish.call_count == 5


class TestWireContents:

    def test_a_wall_measured_through_the_static_transform(self, node):
        node._tf_static_cb(_tf_static())
        node._scan_cb(_scan(2.0, stamp_sec=7))
        fit, estimate = _last(node.fit_pub), _last(node.estimate_pub)
        assert fit.valid and fit.reason == WallLineFit.REASON_OK
        assert fit.distance == pytest.approx(2.0, abs=1e-3)
        assert fit.normal_angle == pytest.approx(0.0, abs=1e-3)
        assert fit.sector_beams == 81 and fit.inlier_count == 81
        assert fit.header.frame_id == 'base_link' and fit.header.stamp.sec == 7
        assert math.isnan(fit.innovation)
        assert estimate.provenance == WallEstimate.PROVENANCE_MEASURED and estimate.valid
        assert estimate.distance == pytest.approx(2.0, abs=1e-3)
        assert estimate.header.frame_id == 'base_link' and estimate.header.stamp.sec == 7

    def test_losing_the_wall_dead_reckons_on_odometry(self, node):
        node._tf_static_cb(_tf_static())
        node._odom_cb(_odom(0.0))
        node._scan_cb(_scan(2.0))
        node.test_clock.now += 0.1
        node._odom_cb(_odom(0.3))
        node._scan_cb(_scan(None))
        estimate = _last(node.estimate_pub)
        assert estimate.provenance == WallEstimate.PROVENANCE_DEAD_RECKONED and estimate.valid
        assert estimate.distance == pytest.approx(1.7, abs=1e-3)
        assert estimate.source_age == pytest.approx(0.1)

    def test_a_seed_when_there_is_nothing_to_dead_reckon(self, node):
        node._tf_static_cb(_tf_static())
        node._seed_cb(Float32(data=1.25))
        node._scan_cb(_scan(None))
        estimate = _last(node.estimate_pub)
        assert estimate.provenance == WallEstimate.PROVENANCE_SEEDED and estimate.valid
        assert estimate.distance == pytest.approx(1.25)
        assert math.isnan(estimate.normal_angle)

    def test_a_contradicted_fit_goes_out_as_wrong_surface_showing_what_it_hit(self, node):
        node._tf_static_cb(_tf_static())
        node._odom_cb(_odom(0.0))
        node._scan_cb(_scan(3.0))
        node.test_clock.now += 0.05
        node._odom_cb(_odom(0.0))
        node._scan_cb(_scan(1.0))
        fit = _last(node.fit_pub)
        assert fit.reason == WallLineFit.REASON_WRONG_SURFACE and not fit.valid
        assert fit.distance == pytest.approx(1.0, abs=1e-3)
        assert fit.predicted_distance == pytest.approx(3.0, abs=1e-3)
        assert fit.innovation == pytest.approx(-2.0, abs=1e-3)
        assert _last(node.estimate_pub).provenance == WallEstimate.PROVENANCE_DEAD_RECKONED


class TestDecoupling:

    def test_the_scan_subscription_is_best_effort(self, node):
        assert node.scan_sub.qos_profile.reliability == ReliabilityPolicy.BEST_EFFORT

    def test_odometry_is_the_local_ekf_by_decision(self, node):
        assert node.odom_topic == '/odometry/filtered'


class TestInSync:

    @pytest.mark.parametrize('prefix,message', [('REASON_', WallLineFit),
                                                ('PROVENANCE_', WallEstimate)])
    def test_message_constants_match_the_pure_module(self, prefix, message):
        module_codes = {name: value for name, value in vars(logic).items()
                        if name.startswith(prefix) and isinstance(value, int)}
        message_codes = {name for name in dir(type(message)) if name.startswith(prefix)}
        assert message_codes == set(module_codes)
        for name, value in module_codes.items():
            assert getattr(message, name) == value, name

    def test_every_code_has_a_log_name(self):
        reasons = {v for n, v in vars(logic).items() if n.startswith('REASON_') and isinstance(v, int)}
        provenances = {v for n, v in vars(logic).items()
                       if n.startswith('PROVENANCE_') and isinstance(v, int)}
        assert set(logic.REASON_NAMES) == reasons
        assert set(logic.PROVENANCE_NAMES) == provenances

    @pytest.mark.parametrize('name', TUNING_PARAMS)
    def test_parameter_default_matches_stack_params(self, node, name):
        assert node.get_parameter(name).value == get_value('lidar_front_wall_' + name)
