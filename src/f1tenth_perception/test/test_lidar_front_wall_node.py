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

import json
import math
from unittest.mock import MagicMock

import numpy as np
import pytest
import rclpy
from f1tenth_messages.msg import WallEstimate, WallLineFit
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.parameter import Parameter
from rclpy.qos import ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32
from tf2_msgs.msg import TFMessage

from f1tenth_description.vehicle_frames import base_front_x
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
    'seed_max_age_sec', 'seed_max_valid_m', 'wall_tracker_front_max_coast_sec',
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
    # The plain sector fit's wire contract: scene labels (and so the tracker)
    # off explicitly, since both default on in stack_params.yaml.
    n = LidarFrontWallNode(parameter_overrides=[
        Parameter('enable_scene_labels', value=False),
        Parameter('use_wall_tracker', value=False)])
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
        # /costmap/front_clearance is from the front bumper; this node's
        # distances are from base_link's origin, so the bumper's x goes back on.
        node._seed_cb(Float32(data=1.25))
        node._scan_cb(_scan(None))
        estimate = _last(node.estimate_pub)
        assert estimate.provenance == WallEstimate.PROVENANCE_SEEDED and estimate.valid
        assert estimate.distance == pytest.approx(1.25 + base_front_x())
        assert math.isnan(estimate.normal_angle)

    def test_a_no_reading_seed_stays_no_reading(self, node):
        node._tf_static_cb(_tf_static())
        node._seed_cb(Float32(data=-1.0))
        node._scan_cb(_scan(None))
        assert _last(node.estimate_pub).provenance != WallEstimate.PROVENANCE_SEEDED

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


class TestMultiWallTracker:
    """use_wall_tracker: gated on enable_scene_labels, feeds the heading wall."""

    def test_both_switches_default_to_stack_params(self):
        n = LidarFrontWallNode()
        try:
            assert n.scene_enabled is bool(get_value('enable_scene_labels'))
            assert n.use_wall_tracker is bool(get_value('use_wall_tracker'))
            assert (n.wall_tracker is not None) is bool(get_value('use_wall_tracker'))
        finally:
            n.destroy_node()

    @staticmethod
    def _make(scene, tracker):
        n = LidarFrontWallNode(parameter_overrides=[
            Parameter('enable_scene_labels', value=scene),
            Parameter('use_wall_tracker', value=tracker)])
        for name in ('fit_pub', 'estimate_pub', 'planes_pub', 'virtual_pub',
                     'virtual_debug_pub', 'walls_pub', 'walls_markers_pub'):
            if hasattr(n, name):
                getattr(n, name).publish = MagicMock()
        return n

    def test_without_scene_labels_the_tracker_stays_off_with_an_error(self):
        n = self._make(False, True)
        try:
            assert n.use_wall_tracker is False and n.wall_tracker is None
            assert not hasattr(n, 'walls_pub')
        finally:
            n.destroy_node()

    def test_a_front_track_coasting_past_the_limit_makes_the_virtual_wall_minus_one(self):
        n = self._make(True, True)
        limit = n.wall_tracker.p.front_max_coast_s

        def feed(t, distance):
            odom = _odom(0.0)
            odom.header.stamp.sec, odom.header.stamp.nanosec = int(t), int(t % 1 * 1e9)
            n._odom_cb(odom)
            scan = _scan(distance, stamp_sec=int(t))
            scan.header.stamp.nanosec = int(t % 1 * 1e9)
            n._scan_cb(scan)
            return _last(n.virtual_pub).data, json.loads(_last(n.virtual_debug_pub).data)

        try:
            n._tf_static_cb(_tf_static())
            for k in range(8):
                value, debug = feed(7.0 + 0.05 * k, 3.0)
            assert debug['wall_source'] == 'tracker' and value > 0.0
            last_hit = 7.0 + 0.05 * 7
            # Unseen, but within the limit: still the heading wall, still a wall.
            value, debug = feed(last_hit + 0.5 * limit, None)
            assert value > 0.0 and debug['wall_source'] == 'tracker'
            # Past the limit: no wall, on every scan, whatever else is left.
            for dt in (1.5 * limit, 2.0 * limit, 3.0 * limit):
                value, debug = feed(last_hit + dt, None)
                assert value == -1.0
                assert debug['valid'] is False
                assert debug['wall_source'] == 'front_coast_expired'
            # The wall is seen again: a front track again, a wall again.
            for k in range(3):
                value, debug = feed(last_hit + 3.0 * limit + 0.05 * (k + 1), 3.0)
            assert debug['wall_source'] == 'tracker'
            assert value == pytest.approx(3.0 - 0.405, abs=0.02)
        finally:
            n.destroy_node()

    def test_the_front_track_becomes_the_heading_wall(self):
        n = self._make(True, True)
        try:
            n._tf_static_cb(_tf_static())
            for k in range(8):
                stamp = 7 + k
                odom = _odom(0.0)
                odom.header.stamp.sec = stamp
                n._odom_cb(odom)
                n._scan_cb(_scan(3.0, stamp_sec=stamp))
            walls = _last(n.walls_pub)
            assert [t.label for t in walls.tracks] == ['front']
            assert walls.tracks[0].d_ray_front == pytest.approx(3.0 - 0.405, abs=0.02)
            debug = json.loads(_last(n.virtual_debug_pub).data)
            assert debug['wall_source'] == 'tracker'
            assert debug['distance_m'] == pytest.approx(3.0 - 0.405, abs=0.02)
        finally:
            n.destroy_node()


# ==============================================================================
# Step 3: the stateless per-cycle fusion -> /perception/wall_observations
# ==============================================================================

class TestWallObservations:
    """The wire contract of the per-cycle fusion. The decision logic itself is
    tested in test_wall_fusion.py; what matters here is that the node emits
    something on every cycle, ages the stereo rather than waiting for it, and
    never turns a missing camera into a missing wall."""

    @staticmethod
    def _node(**overrides):
        from f1tenth_perception.lidar_front_wall_node import LidarFrontWallNode as N
        params = {'enable_scene_labels': True, 'use_wall_tracker': False}
        params.update(overrides)
        n = N(parameter_overrides=[Parameter(k, value=v) for k, v in params.items()])
        n.fit_pub.publish = MagicMock()
        n.estimate_pub.publish = MagicMock()
        n.planes_pub.publish = MagicMock()
        n.virtual_pub.publish = MagicMock()
        n.virtual_debug_pub.publish = MagicMock()
        n.observations_pub.publish = MagicMock()
        clock = _Clock()
        n._now = lambda: clock.now
        n._tf_static_cb(_tf_static())
        return n

    @staticmethod
    def _columns(distance_m, width=64, stamp_sec=7, camera_frame='zed_optical'):
        from f1tenth_messages.msg import StereoColumns
        msg = StereoColumns()
        msg.header.stamp.sec = stamp_sec
        msg.header.frame_id = 'base_link'
        msg.camera_frame = camera_frame
        msg.width, msg.height = width, 48
        msg.fx = msg.fy = 100.0
        msg.cx, msg.cy = width / 2.0, 24.0
        msg.distance_m = [float(d) for d in distance_m]
        msg.z_min = [0.0] * width
        msg.z_max = [1.5] * width
        msg.n_px = [20] * width
        msg.percentile = 20.0
        msg.min_px = 5
        return msg

    @staticmethod
    def _camera_tf(frame='zed_optical'):
        """base_link -> a forward-looking optical frame at the ZED's mount."""
        t = TransformStamped()
        t.header.frame_id = 'base_link'
        t.child_frame_id = frame
        t.transform.translation.x = 0.2299
        t.transform.translation.z = 0.22
        # optical (x right, y down, z fwd) -> base: the ZED macro's own
        # -90/0/-90 roll-pitch-yaw, as a quaternion.
        t.transform.rotation.x = -0.5
        t.transform.rotation.y = 0.5
        t.transform.rotation.z = -0.5
        t.transform.rotation.w = 0.5
        return TFMessage(transforms=[t])

    def test_every_scan_publishes_an_observation_array_even_with_no_camera(self):
        """A missing camera must never read as a missing wall."""
        n = self._node()
        try:
            n._scan_cb(_scan(distance=2.0))
            assert n.observations_pub.publish.call_count == 1
            msg = _last(n.observations_pub)
            assert msg.header.frame_id == 'base_link'
            assert all(o.source == o.SOURCE_LIDAR for o in msg.observations)
            assert all(o.height_measured is False for o in msg.observations)
        finally:
            n.destroy_node()

    def test_an_empty_scene_still_publishes_an_empty_array(self):
        """'Both sensors looked and there is no wall' is the open corridor,
        and it has to be distinguishable from a dead node."""
        n = self._node()
        try:
            n._scan_cb(_scan(distance=None))
            assert n.observations_pub.publish.call_count == 1
            assert list(_last(n.observations_pub).observations) == []
        finally:
            n.destroy_node()

    def test_a_stale_stereo_profile_is_ignored_rather_than_waited_for(self):
        n = self._node()
        try:
            n._tf_static_cb(self._camera_tf())
            n._stereo_columns_cb(self._columns([2.0] * 64, stamp_sec=1))
            n._scan_cb(_scan(distance=2.0, stamp_sec=7))     # 6 s later
            msg = _last(n.observations_pub)
            assert all(o.source == o.SOURCE_LIDAR for o in msg.observations)
            assert msg.n_disagree_columns == 0
        finally:
            n.destroy_node()

    def test_a_fresh_agreeing_profile_makes_the_wall_sourced_both(self):
        n = self._node()
        try:
            n._tf_static_cb(self._camera_tf())
            # The wall is 2.0 m from base_link, so about 1.77 m of optical
            # depth from a lens 0.23 m ahead of it.
            n._stereo_columns_cb(self._columns([1.77] * 64, stamp_sec=7))
            n._scan_cb(_scan(distance=2.0, stamp_sec=7))
            msg = _last(n.observations_pub)
            assert msg.observations, 'the LiDAR saw a wall'
            assert any(o.source == o.SOURCE_BOTH for o in msg.observations)
            both = [o for o in msg.observations if o.source == o.SOURCE_BOTH][0]
            assert both.n_agree > 0
            assert both.height_measured is True
            assert both.z_max == pytest.approx(1.5)
            assert both.confidence > 0.0
        finally:
            n.destroy_node()

    def test_a_contradicting_profile_rejects_the_wall_and_says_so(self):
        n = self._node()
        try:
            n._tf_static_cb(self._camera_tf())
            n._stereo_columns_cb(self._columns([6.0] * 64, stamp_sec=7))
            n._scan_cb(_scan(distance=2.0, stamp_sec=7))
            msg = _last(n.observations_pub)
            assert msg.n_rejected >= 1
            assert msg.n_disagree_columns > 0
            assert all(o.source != o.SOURCE_BOTH for o in msg.observations)
        finally:
            n.destroy_node()

    def test_the_observation_carries_the_line_and_its_endpoints(self):
        n = self._node()
        try:
            n._scan_cb(_scan(distance=2.0))
            obs = _last(n.observations_pub).observations
            assert obs
            o = obs[0]
            assert o.d > 0.0
            assert math.hypot(o.nx, o.ny) == pytest.approx(1.0, abs=1e-3)
            assert o.p0.z == 0.0 and o.p1.z == 0.0
            assert (o.p0.x, o.p0.y) != (o.p1.x, o.p1.y)
        finally:
            n.destroy_node()
