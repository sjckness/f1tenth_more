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
  * Doors (TestDoors, TestDoorStereo, TestDoorParamsCoupling): /perception/doors
    carries CONFIRMED doors only unless door_publish_candidates, /viz/doors every
    live door on every cycle; a failing door cycle never stops the walls; the
    stereo pair is carried from its own stamp to the scan's; and every door
    default is one number in DoorParams, the node and stack_params.yaml.

Run standalone: python3 -m pytest test/test_lidar_front_wall_node.py -v
"""

import dataclasses
import json
import math
import os
from unittest.mock import MagicMock, patch

import numpy as np
import pytest
import rclpy
from builtin_interfaces.msg import Time as StampMsg
from f1tenth_messages.msg import Door, WallEstimate, WallLineFit
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.parameter import Parameter
from rclpy.qos import ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32
from tf2_msgs.msg import TFMessage
import yaml

from corridor_perception.geometry import Pose2D
from f1tenth_description.vehicle_frames import base_front_x
from f1tenth_params.param_defaults import get_value
from f1tenth_perception import door_detection, door_markers, scene_mask
from f1tenth_perception import lidar_front_wall as logic
from f1tenth_perception.lidar_front_wall_node import (
    DOOR_DEBUG_KEYS, DOOR_MAX_CONSECUTIVE_FAILURES, extrinsic_at_scan, LidarFrontWallNode)

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
    def _make(scene, tracker, debug_tracks=False):
        n = LidarFrontWallNode(parameter_overrides=[
            Parameter('enable_scene_labels', value=scene),
            # Every scan: the decimation has its own tests below.
            Parameter('wall_tracker_scan_decimation', value=1),
            # No publish-rate cap: these tests call _scan_cb back to back on
            # the wall clock, and the cpu-cleanup R2 cap (75 ms) would leave
            # _last(virtual_pub) holding the FIRST scan's verdict.
            Parameter('virtual_wall_rate_hz', value=0.0),
            Parameter('wall_debug_tracks', value=debug_tracks),
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

    @staticmethod
    def _feeder(n):
        def feed(t, distance, scan=None):
            odom = _odom(0.0)
            odom.header.stamp.sec, odom.header.stamp.nanosec = int(t), int(t % 1 * 1e9)
            n._odom_cb(odom)
            if scan is None:
                scan = _scan(distance, stamp_sec=int(t))
            scan.header.stamp.sec = int(t)
            scan.header.stamp.nanosec = int(t % 1 * 1e9)
            n._scan_cb(scan)
            return _last(n.virtual_pub).data, json.loads(_last(n.virtual_debug_pub).data)
        return feed

    def test_a_front_track_coasting_past_the_limit_makes_the_virtual_wall_clear(self):
        """UPDATED for block 3: this used to assert -1.0 (NO_DATA) until a new
        front track appeared -- a latch that HOLDS the car, indefinitely in an
        open hall. The sensors are working and nothing is in the way: CLEAR."""
        n = self._make(True, True)
        limit = n.wall_tracker.p.front_max_coast_s
        feed = self._feeder(n)
        clear = float(get_value('wall_clear_distance_m'))
        try:
            n._tf_static_cb(_tf_static())
            for k in range(8):
                value, debug = feed(7.0 + 0.05 * k, 3.0)
            assert debug['wall_source'] == 'tracker' and value > 0.0
            assert debug['front_reason'] == 'front'
            last_hit = 7.0 + 0.05 * 7
            # Unseen over an EMPTY strip (no returns at all), within the limit:
            # still the front wall.
            value, debug = feed(last_hit + 0.5 * limit, None)
            assert 0.0 < value < clear and debug['state'] == 'wall'
            assert debug['strip_range_m'] is None
            # Past the limit: CLEAR on every scan, never -1.
            for dt in (1.5 * limit, 2.0 * limit, 3.0 * limit):
                value, debug = feed(last_hit + dt, None)
                assert value == pytest.approx(clear)
                assert debug['valid'] is True and debug['state'] == 'clear'
                assert debug['front_reason'] == 'coast_expired'
            # The wall is seen again: a front track again, a wall again.
            for k in range(3):
                value, debug = feed(last_hit + 3.0 * limit + 0.05 * (k + 1), 3.0)
            assert debug['wall_source'] == 'tracker'
            assert value == pytest.approx(3.0 - 0.405, abs=0.02)
        finally:
            n.destroy_node()

    def test_a_front_wall_the_strip_sees_through_is_dropped_to_clear_at_once(self):
        """Block 3, the incident's shape: the FRONT wall stops being fitted and
        the strip reads 3 m further. The first such scan already drops it
        (coasting over a strip that sees through it), and the contract is
        CLEAR with the reason on the debug topic -- not NO_DATA."""
        n = self._make(True, True, debug_tracks=True)
        feed = self._feeder(n)
        try:
            n._tf_static_cb(_tf_static())
            for k in range(8):
                value, debug = feed(7.0 + 0.05 * k, 3.0)
            front_id = debug['front_id']
            assert debug['state'] == 'wall' and front_id is not None
            value, debug = feed(7.45, 6.0)
            assert debug['state'] == 'clear' and debug['valid'] is True
            assert debug['front_reason'] == 'coast_contradicted'
            assert debug['strip_range_m'] == pytest.approx(6.0 - base_front_x(), abs=0.05)
            value, debug = feed(7.50, 6.0)
            mine = [t for t in debug['tracks'] if t['id'] == front_id]
            assert mine and mine[0]['contradicted'] is True
            assert mine[0]['front_reason'] == 'contradicted'
        finally:
            n.destroy_node()

    def test_the_debug_payload_carries_the_block_3_fields(self):
        n = self._make(True, True)
        feed = self._feeder(n)
        try:
            n._tf_static_cb(_tf_static())
            for k in range(4):
                _value, debug = feed(7.0 + 0.05 * k, 3.0)
            for key in ('strip_range_m', 'front_reason', 'front_id', 'gap_rejected',
                        'merge_gap_refused', 'stereo_near_rejected', 'rejections'):
                assert key in debug, key
            # The mission wall stop reads these three from this payload
            # (f1tenth_behavior check_stop_condition.front_from_debug).
            assert debug['state'] in ('wall', 'clear', 'no_data')
            assert 'distance_m' in debug
            if debug['state'] == 'wall':
                assert isinstance(debug['front_id'], int)
                assert debug['distance_m'] == pytest.approx(_value, abs=1e-3)
            assert isinstance(debug['gap_rejected'], int)
            assert isinstance(debug['stereo_near_rejected'], int)
            assert isinstance(debug['rejections'], list)
            assert 'front_coast_expired' != debug['wall_source']
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


def _scan_at(distance, stamp, yaw_deg=0.0):
    """A scan at a float stamp; yaw_deg rotates the wall in the laser frame."""
    sec = int(stamp)
    msg = _scan(distance=distance, stamp_sec=sec)
    msg.header.stamp.nanosec = int(round((stamp - sec) * 1e9))
    if yaw_deg:
        yaw = math.radians(yaw_deg)
        angles = ANGLE_MIN + np.arange(N_BEAMS) * ANGLE_INCREMENT
        ranges = np.full(N_BEAMS, np.inf)
        ahead = np.cos(angles - yaw) > 0.05
        ranges[ahead] = (distance - LASER_X) / np.cos(angles[ahead] - yaw)
        msg.ranges = [float(r) for r in ranges.astype(np.float32)]
    return msg


def _corridor_scan(stamp, half_width=1.0, stamp_sec=None):
    """An OPEN corridor: walls either side, nothing ahead for 30 m."""
    sec = int(stamp)
    msg = _scan(distance=None, stamp_sec=sec)
    msg.header.stamp.nanosec = int(round((stamp - sec) * 1e9))
    angles = ANGLE_MIN + np.arange(N_BEAMS) * ANGLE_INCREMENT
    ranges = np.full(N_BEAMS, np.inf)
    sin = np.sin(angles)
    side = np.abs(sin) > 0.08
    ranges[side] = half_width / np.abs(sin[side])
    ranges[ranges > 25.0] = np.inf
    msg.ranges = [float(r) for r in ranges.astype(np.float32)]
    return msg


def _odom_at(stamp, yaw):
    msg = Odometry()
    msg.header.frame_id = 'odom'
    msg.header.stamp.sec = int(stamp)
    msg.header.stamp.nanosec = int(round((stamp - int(stamp)) * 1e9))
    msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
    msg.pose.pose.orientation.w = math.cos(yaw / 2.0)
    msg.twist.twist.linear.x = 0.5
    return msg


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

    @classmethod
    def _columns_at(cls, distance_m, stamp, width=64):
        msg = cls._columns(distance_m, width=width)
        msg.header.stamp.sec = int(stamp)
        msg.header.stamp.nanosec = int(round((stamp - int(stamp)) * 1e9))
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

    def test_a_profile_is_weighted_by_its_age_and_dropped_only_past_the_limit(self):
        """Aged on the camera IMAGE stamp, not on arrival, and the age is a
        WEIGHT rather than a gate.

        The 0.15 s hard gate this replaces passed 44 of 238 live cycles on
        corridor10m_20261015T171753: four fifths of the run was LiDAR-only not
        because the camera had nothing to say but because it had said it
        slightly too early. Past stereo_profile_max_age_s the cycle is still
        LiDAR only -- a stale STEREO_ONLY span is the one path that can create
        a wall out of nothing.
        """
        n = self._node()
        try:
            n._tf_static_cb(self._camera_tf())
            limit = n.profile_max_age
            assert limit == pytest.approx(0.6)
            assert n.profile_min_weight == pytest.approx(0.3)
            # Well past the OLD 0.15 s gate, and now used -- at a weight.
            n._stereo_columns_cb(self._columns_at([1.77] * 64, 7.0 - 0.5 * limit))
            n._scan_cb(_scan_at(2.0, 7.0))
            assert any(o.source == o.SOURCE_BOTH
                       for o in _last(n.observations_pub).observations)
            assert n._stereo_weight == pytest.approx(0.65, abs=0.01)
            assert n._stereo_age == pytest.approx(0.3, abs=0.01)
            # Past the limit: refused, and the cycle says why.
            n._stereo_columns_cb(self._columns_at([1.77] * 64, 7.0 - 1.1 * limit))
            n._scan_cb(_scan_at(2.0, 7.0))
            msg = _last(n.observations_pub)
            assert msg.observations
            assert all(o.source == o.SOURCE_LIDAR for o in msg.observations)
            assert n._stereo_weight == 0.0
        finally:
            n.destroy_node()

    def test_the_cycle_publishes_the_profiles_age_and_weight(self):
        """What the debug video shows instead of "no fresh stereo profile":
        0.16 s worth 0.65 and 2.5 s worth nothing are different problems."""
        n = self._node()
        try:
            n._tf_static_cb(self._camera_tf())
            n._stereo_columns_cb(self._columns_at([1.77] * 64, 6.8))
            n._scan_cb(_scan_at(2.0, 7.0))
            d = json.loads(_last(n.virtual_debug_pub).data)
            assert d['stereo_age_s'] == pytest.approx(0.2, abs=0.01)
            assert d['stereo_weight'] == pytest.approx(0.767, abs=0.01)
            assert d['stereo_max_age_s'] == pytest.approx(0.6)
        finally:
            n.destroy_node()

    def test_the_fitted_lines_are_carried_to_the_image_moment_too(self):
        """Not just the scan returns. The lines are fitted in base_link at the
        SCAN stamp, and their column span is what decides which columns judge
        them, so an uncompensated line is judged by the wrong columns -- 128 of
        them at 0.6 s and this car's minimum radius.
        """
        n = self._node()
        try:
            n._tf_static_cb(self._camera_tf())
            n._odom_cb(_odom_at(7.00, 0.0))
            n._odom_cb(_odom_at(7.40, math.radians(20.0)))
            n._stereo_columns_cb(self._columns_at([2.0] * 64, 7.00))
            n._scan_cb(_scan_at(2.0, 7.40))
            msg = _last(n.observations_pub)
            assert msg.observations
            turned = [(o.col_begin, o.col_end) for o in msg.observations]
            # The same cycle with no odometry to carry anything: the columns
            # the wall is judged over differ, which is the whole point.
            m = self._node()
            try:
                m._tf_static_cb(self._camera_tf())
                m._stereo_columns_cb(self._columns_at([2.0] * 64, 7.00))
                m._scan_cb(_scan_at(2.0, 7.40))
                bare = [(o.col_begin, o.col_end) for o in
                        _last(m.observations_pub).observations]
            finally:
                m.destroy_node()
            assert turned != bare
        finally:
            n.destroy_node()

    def test_compensation_carries_the_scan_by_the_odometry_yaw(self):
        """The mechanism: a point fixed in the world, seen from a car that has
        turned 7 deg, moves by 7 deg in base_link."""
        n = self._node()
        try:
            n._odom_cb(_odom_at(7.00, 0.0))
            n._odom_cb(_odom_at(7.12, math.radians(7.0)))
            pts = np.array([[2.0, 0.0, 0.19]])
            out = n._compensate(pts, scan_stamp=7.12, image_stamp=7.00)
            # Carried BACK to the image moment: the car had not yet turned, so
            # the point sits 7 deg to the other side.
            assert math.degrees(math.atan2(out[0, 1], out[0, 0])) == pytest.approx(7.0, abs=0.1)
            assert math.hypot(out[0, 0], out[0, 1]) == pytest.approx(2.0, abs=1e-6)
            assert out[0, 2] == pytest.approx(0.19)
        finally:
            n.destroy_node()

    def test_a_turning_car_does_not_shift_the_projected_columns(self):
        """7 deg of yaw in 0.12 s is a ~32-column shift at fx 262. Compensated,
        the wall lands on the columns the image actually saw; uncompensated it
        does not, and every AGREE would become DISAGREE."""
        from f1tenth_perception import scene_mask
        from f1tenth_perception.ground_plane import quat_to_matrix
        n = self._node()
        try:
            n._tf_static_cb(self._camera_tf())
            n._odom_cb(_odom_at(7.00, 0.0))
            n._odom_cb(_odom_at(7.12, math.radians(7.0)))
            tf = n._camera_transform('zed_optical')
            assert tf is not None
            rot, trans = tf
            fx = 262.4

            def columns(points):
                uv = scene_mask.project_to_image(points, fx, fx, 320.0, 180.0, rot, trans)
                return uv[np.isfinite(uv[:, 0]), 0]

            # The same world point, seen by a car that has since turned.
            pts = np.array([[2.0, 0.0, 0.19]])
            bare = columns(pts)
            carried = columns(n._compensate(pts, scan_stamp=7.12, image_stamp=7.00))
            assert abs(carried[0] - bare[0]) > 25.0
            # And the carried column is where the image, taken before the
            # left turn, really saw that point. The bearing is from the LENS,
            # 0.2299 m ahead of base_link, not from base_link -- that offset
            # is 4 px here and is part of the geometry, not slack. base_link
            # +y is a SMALLER column.
            carried_pt = n._compensate(pts, scan_stamp=7.12, image_stamp=7.00)[0]
            expected = 320.0 - fx * (carried_pt[1] / (carried_pt[0] - 0.2299))
            assert carried[0] == pytest.approx(expected, abs=0.5)
            assert carried[0] < 320.0
        finally:
            n.destroy_node()

    def test_compensation_is_skipped_without_odometry_rather_than_failing(self):
        n = self._node()
        try:
            pts = np.array([[2.0, 0.0, 0.19]])
            out = n._compensate(pts, scan_stamp=7.12, image_stamp=7.00)
            assert out is pts or np.allclose(out, pts)
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


class TestTrackFieldsOnTheWire:
    """The additive MultiWallTrack fields of step 4."""

    def test_a_published_track_carries_its_source_role_height_and_front_flag(self):
        from f1tenth_perception.lidar_front_wall_node import LidarFrontWallNode as N
        n = N(parameter_overrides=[Parameter('enable_scene_labels', value=True),
                                   Parameter('wall_tracker_scan_decimation', value=1),
                                   Parameter('use_wall_tracker', value=True)])
        try:
            for pub in ('fit_pub', 'estimate_pub', 'planes_pub', 'virtual_pub',
                        'virtual_debug_pub', 'observations_pub', 'walls_pub',
                        'walls_markers_pub'):
                getattr(n, pub).publish = MagicMock()
            n._tf_static_cb(_tf_static())
            for i in range(6):
                n._odom_cb(_odom_at(7.0 + i * 0.05, 0.0))
                n._scan_cb(_scan_at(2.0, 7.0 + i * 0.05))
            msg = _last(n.walls_pub)
            assert msg.tracks
            t = msg.tracks[0]
            assert t.role == t.label
            assert t.source == t.SOURCE_LIDAR
            assert t.z_max == pytest.approx(2.5) and t.height_measured is False
            assert 0.0 <= t.confidence <= 1.0
            # A wall square across the path at 2 m IS the wall in the way.
            assert t.is_front is True
            assert math.isnan(t.d_side)
        finally:
            n.destroy_node()


class TestTentativeTracksAreReportedAsUnknown:
    """A TENTATIVE track is 'unknown' on the wire, drawn and in the debug payload."""

    @staticmethod
    def _node():
        # ONE scan of a wall square across the path 2 m ahead: the tracker
        # holds it as a tentative track whose own (geometric) role is front.
        from f1tenth_perception.lidar_front_wall_node import LidarFrontWallNode as N
        n = N(parameter_overrides=[Parameter('enable_scene_labels', value=True),
                                   Parameter('wall_tracker_scan_decimation', value=1),
                                   Parameter('virtual_wall_rate_hz', value=0.0),
                                   Parameter('wall_debug_tracks', value=True),
                                   Parameter('use_wall_tracker', value=True)])
        for pub in ('fit_pub', 'estimate_pub', 'planes_pub', 'virtual_pub',
                    'virtual_debug_pub', 'observations_pub', 'walls_pub',
                    'walls_markers_pub'):
            getattr(n, pub).publish = MagicMock()
        n.viz_walls_pub = MagicMock()
        n.viz_walls_pub.get_subscription_count.return_value = 1
        n._tf_static_cb(_tf_static())
        # /perception/walls carries published()'s ever-confirmed tracks, so a
        # tentative one reaches it only on the cycle after a merge; publishing
        # every held track takes the same _publish_walls path.
        n.wall_tracker.published = lambda car: list(n.wall_tracker.tracks)
        n._odom_cb(_odom_at(7.0, 0.0))
        n._scan_cb(_scan_at(2.0, 7.0))
        return n

    def test_the_tracker_keeps_the_tentative_track_s_geometric_role_front(self):
        from f1tenth_perception import multi_wall_tracker as mwt
        n = self._node()
        try:
            held = list(n.wall_tracker.tracks)
            assert held and all(t.state == mwt.STATE_TENTATIVE for t in held)
            assert 'front' in {t.role for t in held}
        finally:
            n.destroy_node()

    def test_a_tentative_track_is_published_as_unknown_whatever_its_geometric_role(self):
        n = self._node()
        try:
            wire = _last(n.walls_pub).tracks
            assert {t.id for t in wire} == {t.id for t in n.wall_tracker.tracks}
            for t in wire:
                assert t.state == t.STATE_TENTATIVE
                assert (t.role, t.label) == ('unknown', 'unknown')
                assert math.isnan(t.d_side)
        finally:
            n.destroy_node()

    def test_its_perception_walls_marker_is_the_unknown_grey_and_says_unknown(self):
        from f1tenth_perception.lidar_front_wall_node import _ROLE_COLOUR
        from visualization_msgs.msg import Marker
        n = self._node()
        try:
            markers = _last(n.walls_markers_pub).markers
            slabs = [m for m in markers if m.type == Marker.CUBE]
            texts = [m for m in markers if m.type == Marker.TEXT_VIEW_FACING]
            assert slabs and texts
            for m in slabs:
                assert (m.color.r, m.color.g, m.color.b) == pytest.approx(
                    _ROLE_COLOUR['unknown'])
            for m in texts:
                words = m.text.split()
                assert words[1] == 'unknown' and words[-1] == 'tentative'
        finally:
            n.destroy_node()

    def test_it_is_drawn_as_unknown_on_viz_walls(self):
        from f1tenth_perception import wall_markers as wm
        n = self._node()
        try:
            markers = _last(n.viz_walls_pub).markers
            lines = [m for m in markers if m.id % 4 == wm.SLOT_LINE]
            texts = [m for m in markers if m.id % 4 == wm.SLOT_TEXT]
            assert lines and texts
            for m in lines:
                assert (m.color.r, m.color.g, m.color.b) == pytest.approx(
                    wm.ROLE_COLOUR['unknown'])
            for m in texts:
                assert m.text.split()[1] == 'unknown'
        finally:
            n.destroy_node()

    def test_the_debug_payload_reports_it_as_unknown(self):
        n = self._node()
        try:
            tracks = json.loads(_last(n.virtual_debug_pub).data)['tracks']
            assert tracks
            for t in tracks:
                assert (t['state'], t['role']) == ('tentative', 'unknown')
        finally:
            n.destroy_node()


class TestTheOutputContract:
    """Step 5: WALL / CLEAR / NO_DATA on the wire.

    The state a consumer branches on, and the one the old pipeline could not
    express. Each state is reached through the real node, not by calling the
    pure function again."""

    @staticmethod
    def _node():
        from f1tenth_perception.lidar_front_wall_node import LidarFrontWallNode as N
        n = N(parameter_overrides=[Parameter('enable_scene_labels', value=True),
                                   Parameter('wall_tracker_scan_decimation', value=1),
                                   # See TestMultiWallTracker._make.
                                   Parameter('virtual_wall_rate_hz', value=0.0),
                                   Parameter('use_wall_tracker', value=True)])
        for pub in ('fit_pub', 'estimate_pub', 'planes_pub', 'virtual_pub',
                    'virtual_debug_pub', 'observations_pub', 'walls_pub',
                    'walls_markers_pub'):
            getattr(n, pub).publish = MagicMock()
        n._tf_static_cb(_tf_static())
        return n

    @staticmethod
    def _run(n, distance, scans=6, t0=7.0, dt=0.05):
        for i in range(scans):
            n._odom_cb(_odom_at(t0 + i * dt, 0.0))
            n._scan_cb(_scan_at(distance, t0 + i * dt) if distance is not None
                       else _scan(distance=None, stamp_sec=int(t0 + i * dt)))

    @staticmethod
    def _state(n):
        import json
        return json.loads(_last(n.virtual_debug_pub).data)['state']

    def test_a_wall_across_the_path_is_WALL_with_its_distance(self):
        n = self._node()
        try:
            self._run(n, 2.0)
            assert self._state(n) == 'wall'
            assert _last(n.virtual_pub).data == pytest.approx(2.0 - base_front_x(), abs=0.05)
            assert _last(n.estimate_pub).state == WallEstimate.STATE_WALL
        finally:
            n.destroy_node()

    def test_an_open_corridor_is_CLEAR_not_a_phantom_wall(self):
        """The corridor_phantom failure. Nothing in the strip used to become
        the stereo plane -- the SIDE wall -- at 1.1 m. It is now CLEAR."""
        n = self._node()
        try:
            self._run(n, None)
            assert self._state(n) == 'clear'
            clear = float(get_value('wall_clear_distance_m'))
            assert _last(n.virtual_pub).data == pytest.approx(clear)
            assert _last(n.estimate_pub).state == WallEstimate.STATE_CLEAR
        finally:
            n.destroy_node()

    def test_a_side_wall_only_corridor_is_CLEAR(self):
        """Walls on both sides and nothing ahead: still nothing in the way."""
        n = self._node()
        try:
            for i in range(8):
                t = 7.0 + i * 0.05
                n._odom_cb(_odom_at(t, 0.0))
                n._scan_cb(_corridor_scan(t))
            assert self._state(n) == 'clear'
            assert _last(n.virtual_pub).data > 10.0
        finally:
            n.destroy_node()

    def test_a_scan_with_no_laser_transform_is_NO_DATA(self):
        """Block 3: NO_DATA is for missing data only, and a scan that cannot
        be placed is missing data -- not the last placed scan's verdict."""
        from f1tenth_perception.lidar_front_wall_node import LidarFrontWallNode as N
        n = N(parameter_overrides=[Parameter('enable_scene_labels', value=True),
                                   Parameter('wall_tracker_scan_decimation', value=1),
                                   Parameter('virtual_wall_rate_hz', value=0.0),
                                   Parameter('use_wall_tracker', value=True)])
        try:
            for pub in ('fit_pub', 'estimate_pub', 'planes_pub', 'virtual_pub',
                        'virtual_debug_pub', 'observations_pub', 'walls_pub',
                        'walls_markers_pub'):
                getattr(n, pub).publish = MagicMock()
            n._odom_cb(_odom_at(7.0, 0.0))
            n._scan_cb(_scan_at(2.0, 7.0))          # no /tf_static yet
            assert self._state(n) == 'no_data'
            assert _last(n.virtual_pub).data == -1.0
        finally:
            n.destroy_node()

    def test_no_odometry_is_NO_DATA_and_minus_one_never_clear(self):
        n = self._node()
        try:
            n._scan_cb(_scan_at(2.0, 7.0))      # no odom at all
            assert self._state(n) == 'no_data'
            assert _last(n.virtual_pub).data == -1.0
            assert _last(n.estimate_pub).state == WallEstimate.STATE_NO_DATA
        finally:
            n.destroy_node()

    def test_the_three_states_are_three_different_published_values(self):
        clear = float(get_value('wall_clear_distance_m'))
        wall_n, clear_n, nodata_n = self._node(), self._node(), self._node()
        try:
            self._run(wall_n, 2.0)
            self._run(clear_n, None)
            nodata_n._scan_cb(_scan_at(2.0, 7.0))
            values = [_last(x.virtual_pub).data for x in (wall_n, clear_n, nodata_n)]
            assert values[2] == -1.0
            assert values[1] == pytest.approx(clear)
            assert 0.0 < values[0] < clear
            assert len(set(values)) == 3
        finally:
            for x in (wall_n, clear_n, nodata_n):
                x.destroy_node()


class TestWallMarkers:
    """Step 7: a wall is drawn as the vertical plane it is."""

    def _publish(self, **track_kw):
        from f1tenth_perception.lidar_front_wall_node import LidarFrontWallNode as N
        n = N(parameter_overrides=[Parameter('enable_scene_labels', value=True),
                                   Parameter('wall_tracker_scan_decimation', value=1),
                                   Parameter('use_wall_tracker', value=True)])
        for pub in ('fit_pub', 'estimate_pub', 'planes_pub', 'virtual_pub',
                    'virtual_debug_pub', 'observations_pub', 'walls_pub',
                    'walls_markers_pub'):
            getattr(n, pub).publish = MagicMock()
        n._tf_static_cb(_tf_static())
        for i in range(6):
            n._odom_cb(_odom_at(7.0 + i * 0.05, 0.0))
            n._scan_cb(_scan_at(2.0, 7.0 + i * 0.05))
        return n

    def test_each_confirmed_wall_is_a_thin_vertical_slab_coloured_by_role(self):
        from visualization_msgs.msg import Marker
        n = self._publish()
        try:
            markers = _last(n.walls_markers_pub).markers
            cubes = [m for m in markers if m.type == Marker.CUBE]
            assert cubes
            slab = cubes[0]
            assert slab.scale.y == pytest.approx(n.wall_marker_thickness_m)
            assert slab.scale.y <= 0.02
            assert slab.scale.z > 1.0           # it stands up
            assert slab.scale.x > 0.1           # and runs along the wall
            # Red-ish: this one is the front wall.
            assert slab.color.r > slab.color.b
        finally:
            n.destroy_node()

    def test_the_label_says_id_role_distance_source_and_state(self):
        from visualization_msgs.msg import Marker
        n = self._publish()
        try:
            texts = [m for m in _last(n.walls_markers_pub).markers
                     if m.type == Marker.TEXT_VIEW_FACING]
            assert texts
            words = texts[0].text.split()
            assert len(words) == 5
            assert words[1] == 'front'
            assert words[2].endswith('m')
            assert words[3] == 'lidar'
            assert words[4] in ('tentative', 'confirmed', 'coasting')
        finally:
            n.destroy_node()

    def test_an_assumed_height_is_drawn_fainter_than_a_measured_one(self):
        n = self._publish()
        try:
            from visualization_msgs.msg import Marker
            slab = [m for m in _last(n.walls_markers_pub).markers
                    if m.type == Marker.CUBE][0]
            assert slab.color.a == pytest.approx(0.35)   # nothing measured it
        finally:
            n.destroy_node()

    def test_no_marker_outlives_the_tracker_by_more_than_its_lifetime(self):
        """Replaces test_every_array_starts_by_clearing_so_gone_tracks_disappear.

        That test asserted a DELETEALL at the head of every array. The pairing
        it was half of -- DELETEALL every cycle, and no lifetime on any marker
        -- is what made the walls flicker and vanish in Foxglove: the
        torn-down state is the one a client sees whenever the rebuild meant to
        follow it is dropped or late. The guarantee it was reaching for, that
        a gone track does not linger, is now carried by a per-marker lifetime
        plus a per-id DELETE, which is what these two tests check instead.
        """
        n = self._publish()
        try:
            markers = _last(n.walls_markers_pub).markers
            assert markers
            for m in markers:
                lifetime = m.lifetime.sec + m.lifetime.nanosec * 1e-9
                assert lifetime == pytest.approx(n.wall_marker_lifetime_s)
                # 3x the tracker period at 40 Hz scans and decimation 4.
                assert lifetime >= 3 * (4 / 40.0)
        finally:
            n.destroy_node()

    def test_no_array_clears_every_namespace(self):
        from visualization_msgs.msg import Marker
        n = self._publish()
        try:
            actions = [m.action for m in _last(n.walls_markers_pub).markers]
            assert actions
            assert Marker.DELETEALL not in actions
        finally:
            n.destroy_node()

    def test_a_track_that_stops_being_published_is_deleted_by_id(self):
        from visualization_msgs.msg import Marker
        n = self._publish()
        try:
            drawn = set(n._drawn_track_ids)
            assert drawn, 'nothing was drawn, so there is nothing to delete'
            # The tracker now reports no tracks at all: the next array must
            # carry a DELETE for both marker slots of every id drawn before,
            # and must not fall back on a DELETEALL.
            n.wall_tracker.published = lambda car: []
            n._odom_cb(_odom_at(7.4, 0.0))
            n._scan_cb(_scan_at(2.0, 7.4))
            markers = _last(n.walls_markers_pub).markers
            assert all(m.action == Marker.DELETE for m in markers)
            deleted = {m.id for m in markers}
            assert deleted == {slot for i in drawn for slot in (2 * i, 2 * i + 1)}
            assert {m.ns for m in markers} == {'walls'}
            assert n._drawn_track_ids == set()
        finally:
            n.destroy_node()


class TestVizWalls:
    """/viz/walls: the whole tracker state, and only while someone is looking.

    The marker CONTENT is tested in test_wall_markers.py against the pure
    builder. What matters here is the wiring: the three gates, and that the
    topic is fed every track the tracker holds rather than published()'s four.
    """

    def _node(self, **overrides):
        from f1tenth_perception.lidar_front_wall_node import LidarFrontWallNode as N
        params = {'enable_scene_labels': True, 'wall_tracker_scan_decimation': 1,
                  'use_wall_tracker': True}
        params.update(overrides)
        n = N(parameter_overrides=[Parameter(k, value=v) for k, v in params.items()])
        for pub in ('fit_pub', 'estimate_pub', 'planes_pub', 'virtual_pub',
                    'virtual_debug_pub', 'observations_pub', 'walls_pub',
                    'walls_markers_pub'):
            getattr(n, pub).publish = MagicMock()
        n.viz_walls_pub = MagicMock()
        n.viz_walls_pub.get_subscription_count.return_value = 1
        n._tf_static_cb(_tf_static())
        return n

    @staticmethod
    def _drive(n, cycles=6, t0=7.0):
        for i in range(cycles):
            n._odom_cb(_odom_at(t0 + i * 0.05, 0.0))
            n._scan_cb(_scan_at(2.0, t0 + i * 0.05))

    def test_it_publishes_while_something_is_subscribed(self):
        n = self._node()
        try:
            self._drive(n)
            assert n.viz_walls_pub.publish.called
            assert _last(n.viz_walls_pub).markers
        finally:
            n.destroy_node()

    def test_nothing_is_published_when_no_one_is_subscribed(self):
        n = self._node()
        try:
            n.viz_walls_pub.get_subscription_count.return_value = 0
            self._drive(n)
            assert not n.viz_walls_pub.publish.called
            # And it did not even build an array to throw away.
            assert n._viz_walls_builder.drawn_ids == set()
        finally:
            n.destroy_node()

    def test_the_flag_off_means_the_subscription_is_never_even_counted(self):
        n = self._node(viz_walls_enabled=False)
        try:
            self._drive(n)
            assert not n.viz_walls_pub.publish.called
            assert not n.viz_walls_pub.get_subscription_count.called
        finally:
            n.destroy_node()

    def test_the_topic_exists_even_with_the_flag_off(self):
        """So the flag is a publish-time decision, not a restart-time one."""
        n = self._node(viz_walls_enabled=False)
        try:
            assert n.viz_walls_pub is not None
        finally:
            n.destroy_node()

    def test_the_rate_is_capped_below_the_tracker_s_own(self):
        n = self._node(viz_walls_max_hz=5.0)
        try:
            # 20 cycles of 0.05 s is 1.0 s of scan stamps at 20 Hz; a 5 Hz cap
            # must let through far fewer than one per cycle.
            self._drive(n, cycles=20)
            tracker_cycles = n.walls_markers_pub.publish.call_count
            viz_cycles = n.viz_walls_pub.publish.call_count
            assert viz_cycles < tracker_cycles
            assert viz_cycles <= 6          # 1.0 s at 5 Hz, plus the first one
        finally:
            n.destroy_node()

    def test_it_is_fed_every_track_not_only_the_four_published(self):
        n = self._node()
        try:
            self._drive(n)
            held = {t.id for t in n.wall_tracker.tracks}
            assert held
            drawn = {m.id // 4 for m in _last(n.viz_walls_pub).markers}
            assert drawn == held
            # /perception/walls/markers is the narrower view by construction
            # (it is fed published()), and this topic must never be the one
            # that hides a track the other one shows.
            asserted = {m.id // 2 for m in _last(n.walls_markers_pub).markers}
            assert asserted <= drawn
        finally:
            n.destroy_node()

    def test_the_markers_are_in_the_tracker_s_own_frame(self):
        n = self._node()
        try:
            self._drive(n)
            frames = {m.header.frame_id for m in _last(n.viz_walls_pub).markers}
            assert frames == {n._odom_frame}
        finally:
            n.destroy_node()

    def test_going_unsubscribed_forgets_what_was_drawn(self):
        n = self._node()
        try:
            self._drive(n)
            assert n._viz_walls_builder.drawn_ids
            n.viz_walls_pub.get_subscription_count.return_value = 0
            self._drive(n, cycles=2, t0=8.0)
            assert n._viz_walls_builder.drawn_ids == set()
        finally:
            n.destroy_node()


class TestTheRectanglePair:
    """The wire side of the rectangle verification (E2). The verdicts
    themselves are tested in test_wall_rectangle.py; what matters here is that
    the node pairs the two images by stamp, ages them on the IMAGE stamp, and
    never verifies against a half-arrived tick."""

    @staticmethod
    def _node(**overrides):
        from f1tenth_perception.lidar_front_wall_node import LidarFrontWallNode as N
        params = {'enable_scene_labels': True, 'use_wall_tracker': False}
        params.update(overrides)
        n = N(parameter_overrides=[Parameter(k, value=v) for k, v in params.items()])
        for pub in ('fit_pub', 'estimate_pub', 'planes_pub', 'virtual_pub',
                    'virtual_debug_pub', 'observations_pub'):
            getattr(n, pub).publish = MagicMock()
        n._tf_static_cb(_tf_static())
        return n

    @staticmethod
    def _images(stamp_cls=5.0, stamp_dep=5.0, width=8, height=4):
        from sensor_msgs.msg import Image
        from f1tenth_perception import scene_mask
        out = []
        for stamp, encoding, data in (
                (stamp_cls, 'mono8',
                 np.full((height, width), scene_mask.CLASS_WALL, dtype=np.uint8)),
                (stamp_dep, 'mono16',
                 np.full((height, width), 3000, dtype=np.uint16))):
            msg = Image()
            msg.header.stamp.sec = int(stamp)
            msg.header.stamp.nanosec = int(round((stamp % 1.0) * 1e9))
            msg.header.frame_id = 'zed_optical'
            msg.height, msg.width = height, width
            msg.encoding = encoding
            msg.step = width * data.itemsize
            msg.data = data.tobytes()
            out.append(msg)
        return out

    def test_a_matched_pair_is_read_without_copying(self):
        from f1tenth_perception import scene_mask
        n = self._node()
        cls_msg, dep_msg = self._images()
        n._mask_classes_cb(cls_msg)
        n._mask_depth_cb(dep_msg)
        pair = n._mask_pair(5.0)
        assert pair is not None
        labels, depth, age, weight = pair
        assert labels.shape == (4, 8) and depth.shape == (4, 8)
        assert labels[0, 0] == scene_mask.CLASS_WALL and depth[0, 0] == 3000
        assert not labels.flags.owndata and not depth.flags.owndata
        assert age == pytest.approx(0.0) and weight == pytest.approx(1.0)
        n.destroy_node()

    def test_mismatched_stamps_are_a_half_arrived_tick(self):
        n = self._node()
        cls_msg, dep_msg = self._images(stamp_cls=5.0, stamp_dep=5.2)
        n._mask_classes_cb(cls_msg)
        n._mask_depth_cb(dep_msg)
        assert n._mask_pair(5.0) is None
        n.destroy_node()

    def test_the_pair_ages_on_the_image_stamp(self):
        n = self._node()
        cls_msg, dep_msg = self._images(stamp_cls=5.0, stamp_dep=5.0)
        n._mask_classes_cb(cls_msg)
        n._mask_depth_cb(dep_msg)
        assert n._mask_pair(5.0 + n.profile_max_age + 0.01) is None
        assert n._rect_weight == 0.0
        pair = n._mask_pair(5.0 + 0.5 * n.profile_max_age)
        assert pair is not None and 0.0 < pair[3] < 1.0
        n.destroy_node()

    def test_no_mask_at_all_leaves_every_track_unverified(self):
        n = self._node(use_wall_tracker=True)
        from corridor_perception.geometry import Pose2D
        assert n._verify_tracks(Pose2D(0.0, 0.0, 0.0), 1.0, None) == {}
        assert math.isnan(n._rect_ms)
        n.destroy_node()

    def test_the_verification_can_be_turned_off(self):
        n = self._node(wall_rect_verify=False)
        topics = {t for t, _ in n.get_topic_names_and_types()}
        assert '/perception/scene_labels/classes' not in topics
        assert '/perception/scene_labels/depth_mm' not in topics
        n.destroy_node()


class TestTrackerDecimation:
    """wall_tracker_scan_decimation: the tracker on one scan in N (H2).

    The contract the rest of the node keeps is the thing to protect: the
    WallLineFit/WallEstimate pair still publishes on EVERY scan, and the scan
    subscription is still best-effort so this node cannot back-pressure
    urg_node. What changes is only how often the tracker steps, and on a
    skipped scan its last verdict stands rather than flapping to NO_DATA.
    Since cpu-cleanup R2 /perception/virtual_wall (+ /debug) publishes only on
    the tracker steps themselves (lidar_front_wall.virtual_wall_due); the
    repeats are gone, the WallEstimate state still carries the held verdict.
    """

    @staticmethod
    def _node(decimation=4):
        from f1tenth_perception.lidar_front_wall_node import LidarFrontWallNode as N
        n = N(parameter_overrides=[
            Parameter('enable_scene_labels', value=True),
            Parameter('use_wall_tracker', value=True),
            Parameter('wall_tracker_scan_decimation', value=decimation),
            # No wall-clock rate cap: these scans are fed back to back.
            Parameter('virtual_wall_rate_hz', value=0.0)])
        for name in ('fit_pub', 'estimate_pub', 'planes_pub', 'virtual_pub',
                     'virtual_debug_pub', 'walls_pub', 'walls_markers_pub',
                     'observations_pub'):
            if getattr(n, name, None) is not None:
                getattr(n, name).publish = MagicMock()
        n._tf_static_cb(_tf_static())
        return n

    def test_the_default_comes_from_stack_params(self):
        n = self._node(decimation=int(get_value(
            'lidar_front_wall_wall_tracker_scan_decimation')))
        try:
            assert n.tracker_decimation == 4
        finally:
            n.destroy_node()

    def test_one_scan_in_four_reaches_the_tracker(self):
        n = self._node(4)
        try:
            due = [n._tracker_due() for _ in range(12)]
            assert due == [True, False, False, False] * 3
            assert sum(due) == 3
        finally:
            n.destroy_node()

    def test_decimation_one_is_every_scan(self):
        n = self._node(1)
        try:
            assert all(n._tracker_due() for _ in range(8))
        finally:
            n.destroy_node()

    def test_zero_and_negative_are_clamped_to_every_scan(self):
        for value in (0, -3):
            n = self._node(value)
            try:
                assert n.tracker_decimation == 1
                assert all(n._tracker_due() for _ in range(4))
            finally:
                n.destroy_node()

    def test_the_phase_is_a_counter_so_a_replay_decimates_identically(self):
        first = self._node(4)
        second = self._node(4)
        try:
            a = [first._tracker_due() for _ in range(20)]
            b = [second._tracker_due() for _ in range(20)]
            assert a == b
        finally:
            first.destroy_node()
            second.destroy_node()

    def test_the_scan_subscription_is_depth_one_and_still_best_effort(self):
        n = self._node()
        try:
            assert n.scan_sub.qos_profile.depth == 1
            assert n.scan_sub.qos_profile.reliability == ReliabilityPolicy.BEST_EFFORT
        finally:
            n.destroy_node()

    def test_the_pair_publishes_every_scan_the_virtual_wall_every_step(self):
        """The WallLineFit/WallEstimate contract the decimation must not
        touch (a topic that goes quiet cannot be told from a dead node), and
        since cpu-cleanup R2 the virtual wall only on tracker steps: 2 of 8."""
        n = self._node(4)
        try:
            for i in range(8):
                n._odom_cb(_odom_at(7.0 + 0.025 * i, 0.0))
                n._scan_cb(_scan_at(3.0, 7.0 + 0.025 * i))
            assert n.fit_pub.publish.call_count == 8
            assert n.estimate_pub.publish.call_count == 8
            assert n.virtual_pub.publish.call_count == 2
            assert n.virtual_debug_pub.publish.call_count == 2
        finally:
            n.destroy_node()

    def test_a_skipped_scan_holds_the_last_verdict_rather_than_no_data(self):
        n = self._node(4)
        try:
            for i in range(5):
                n._odom_cb(_odom_at(7.0 + 0.025 * i, 0.0))
                n._scan_cb(_scan_at(3.0, 7.0 + 0.025 * i))
            # The held verdict now rides on the per-scan WallEstimate: the
            # three skipped scans after the first step repeat its state, and
            # none of them is NO_DATA.
            states = [c[0][0].state for c in n.estimate_pub.publish.call_args_list]
            assert states[1:4] == [states[0]] * 3
            # The debug payload goes out on the steps only (scans 1 and 5).
            payloads = [json.loads(c[0][0].data)
                        for c in n.virtual_debug_pub.publish.call_args_list]
            assert len(payloads) == 2
            assert all(p['tracker_decimation'] == 4 for p in payloads)
            assert all(p['tracker_skipped'] == 0 for p in payloads)
        finally:
            n.destroy_node()


# ==============================================================================
# Door detection (perception only) and the fixed wall marker height
# ==============================================================================

def _door_scan(stamp, gap=True, wall_x=3.0, half_gap=0.425, back_x=6.0):
    """
    Build a scan of a wall ahead at wall_x with a doorway at y in +-half_gap.

    With gap False the wall is closed. The next room's wall is at back_x, and
    the laser sits at x = LASER_X.
    """
    msg = _scan(distance=None, stamp_sec=int(stamp))
    msg.header.stamp.nanosec = int(round((stamp - int(stamp)) * 1e9))
    angles = ANGLE_MIN + np.arange(N_BEAMS) * ANGLE_INCREMENT
    c, s = np.cos(angles), np.sin(angles)
    ranges = np.full(N_BEAMS, np.inf)
    with np.errstate(divide='ignore', invalid='ignore'):
        for x in (back_x, wall_x):
            r = (x - LASER_X) / c
            y = r * s
            hit = (c > 0.05) & (np.abs(y) <= 6.0)
            if x == wall_x and gap:
                hit &= np.abs(y) > half_gap
            ranges = np.where(hit, r, ranges)
    msg.ranges = [float(r) for r in ranges.astype(np.float32)]
    return msg


#: The scene the door tests drive past, as test_door_detection.py's
#: TestThroughTheRealTracker: a corridor along +x, its LEFT wall at y = SIDE_Y
#: with a doorway at x in +-DOOR_HALF, the office behind it 3 m deep and the
#: right wall at -SIDE_Y. The two pieces of the left wall are confirmed LEFT
#: tracks: HOSTS. (A doorway straight AHEAD would not do: it splits the end
#: wall into two pieces that are neither FRONT nor a side wall -- unknown,
#: and an unknown segment never hosts a door.)
SIDE_Y = 1.2
DOOR_HALF = 0.425
#: 2 m before the doorway to 0.5 m before it, 0.1 m a cycle: the gap is
#: oblique_view first, then a door seen from enough places to be CONFIRMED.
DRIVE_PAST = tuple(round(-2.0 + 0.1 * i, 3) for i in range(16))
#: Standing still 1 m before it: a door seen from ONE place, a candidate.
STAND = (-1.0,) * 6
#: Nearly abeam the doorway, where the left-looking camera below sees it.
#: Nine cycles, not six: THE CONFIRMATION LEVEL needs five in a row before
#: the two wall pieces either side of the doorway are confirmed HOSTS, and
#: the piece beyond it shrinks below the birth span as the car comes abeam,
#: so starting at -0.6 m it never confirms and there is no door to measure.
ABEAM = tuple(round(-0.9 + 0.1 * i, 3) for i in range(9))

#: Every key of one door_candidates record on the debug topic (door_record).
RECORD_KEYS = {'code', 'cls', 'visible', 'detail', 'width', 'n_beams', 'n_required',
               'view_deg', 'behind_frac', 'boundary_hits', 'leaf_side', 'glass_suspect',
               'jamb_height_unknown', 'jamb_tops', 'host_offset_deg', 'parents',
               'centre_bl'}


def _corridor_segments(doorway=True):
    """Return the corridor's walls (odom); doorway False closes the left wall."""
    left = ([((-6.0, SIDE_Y), (-DOOR_HALF, SIDE_Y)), ((DOOR_HALF, SIDE_Y), (6.0, SIDE_Y))]
            if doorway else [((-6.0, SIDE_Y), (6.0, SIDE_Y))])
    return left + [((-6.0, -SIDE_Y), (6.0, -SIDE_Y)),
                   ((-9.0, SIDE_Y + 3.0), (9.0, SIDE_Y + 3.0))]


def _raycast_scan(stamp, car_x, segments):
    """Return the scan of a car at (car_x, 0) heading +x, against segments (odom)."""
    msg = _scan(distance=None, stamp_sec=int(stamp))
    msg.header.stamp.nanosec = int(round((stamp - int(stamp)) * 1e9))
    angles = ANGLE_MIN + np.arange(N_BEAMS) * ANGLE_INCREMENT
    d = np.stack([np.cos(angles), np.sin(angles)], axis=1)
    origin = np.array([car_x + LASER_X, 0.0])
    best = np.full(N_BEAMS, np.inf)
    for (ax, ay), (bx, by) in segments:
        e = np.array([bx - ax, by - ay], dtype=float)
        w = np.array([ax, ay], dtype=float) - origin
        den = d[:, 0] * (-e[1]) - d[:, 1] * (-e[0])
        with np.errstate(divide='ignore', invalid='ignore'):
            t = (w[0] * (-e[1]) - w[1] * (-e[0])) / den
            u = (d[:, 0] * w[1] - d[:, 1] * w[0]) / den
        hit = (np.abs(den) > 1e-12) & (t > 0) & (u >= 0) & (u <= 1)
        best = np.where(hit & (t < best), t, best)
    msg.ranges = [float(r) for r in best.astype(np.float32)]
    return msg


def _strict_json(text):
    """Parse a debug payload, failing on NaN or Infinity (not JSON)."""
    def refuse(constant):
        raise AssertionError(f'{constant} in the debug payload')
    return json.loads(text, parse_constant=refuse)


def _payloads(n):
    """Return every /perception/virtual_wall/debug payload the node published, in order."""
    return [_strict_json(c.args[0].data) for c in n.virtual_debug_pub.publish.call_args_list]


def _door_track(door_id, state, x=0.0):
    """Return a DoorTrack in the corridor's left wall, its numbers numpy scalars."""
    door = door_detection.DoorTrack(
        door_id, np.array([x - DOOR_HALF, SIDE_Y]), np.array([x + DOOR_HALF, SIDE_Y]),
        np.array([0.0, 1.0]), born=6.0, last_positive=6.5, state=state,
        hits=np.int64(4), parents=(np.int64(3), np.int64(4)), n_beams=np.int64(70000),
        leaf_side=np.int64(door_detection.LEAF_LEFT))
    door.confidence = np.float64(1.3)
    return door


class TestDoors:
    """
    /perception/doors, /viz/doors and the door debug keys: the node's wiring.

    The detection itself is tested in test_door_detection.py. Here the car
    drives past a doorway in a corridor's left wall through the node: the
    real extraction, wall tracker, Manhattan prior and door tracker.
    """

    PUBS = ('fit_pub', 'estimate_pub', 'planes_pub', 'virtual_pub', 'virtual_debug_pub',
            'observations_pub', 'walls_pub', 'walls_markers_pub', 'doors_pub')

    def _node(self, door_on=True, **overrides):
        params = {'enable_scene_labels': True, 'wall_tracker_scan_decimation': 1,
                  'use_wall_tracker': True, 'wall_extract_backend': 'python',
                  # No publish-rate cap: every cycle's debug payload is read.
                  'virtual_wall_rate_hz': 0.0}
        if door_on:
            # Off by default since 2026-10-20 (stack_params and the node's
            # fallback); these tests are about the doors.
            params['door_detection_enabled'] = True
        params.update(overrides)
        n = LidarFrontWallNode(
            parameter_overrides=[Parameter(k, value=v) for k, v in params.items()])
        for pub in self.PUBS:
            if hasattr(n, pub):
                getattr(n, pub).publish = MagicMock()
        if hasattr(n, 'viz_doors_pub'):
            # A bare mock, so a call to get_subscription_count() shows.
            n.viz_doors_pub = MagicMock()
        n._tf_static_cb(_tf_static())
        return n

    @staticmethod
    def _drive(n, cycles=8, t0=7.0, gap=True):
        """Stand still before a wall ahead at x = 3 (TestFixedWallHeight's scene)."""
        for i in range(cycles):
            n._odom_cb(_odom_at(t0 + i * 0.1, 0.0))
            n._scan_cb(_door_scan(t0 + i * 0.1, gap=gap))

    @staticmethod
    def _drive_past(n, xs=DRIVE_PAST, t0=7.0, doorway=True):
        """Drive the corridor, a tracker cycle every 0.1 s at x = xs[i]: odometry, then scan."""
        for i, x in enumerate(xs):
            t = round(t0 + 0.1 * i, 6)
            odom = _odom_at(t, 0.0)
            odom.pose.pose.position.x = float(x)
            n._odom_cb(odom)
            n._scan_cb(_raycast_scan(t, x, _corridor_segments(doorway)))

    def test_a_doorway_driven_past_is_published_as_a_confirmed_door(self):
        n = self._node()
        try:
            self._drive_past(n)
            msg = _last(n.doors_pub)
            assert msg.header.frame_id == n._odom_frame
            (door,) = msg.doors
            assert door.state == Door.STATE_CONFIRMED
            assert door.width == pytest.approx(2 * DOOR_HALF, abs=0.04)
            assert (door.center.x, door.center.y) == pytest.approx((0.0, SIDE_Y), abs=0.04)
            assert (door.normal.x, door.normal.y) == pytest.approx((0.0, 1.0), abs=0.01)
            car_x = DRIVE_PAST[-1]
            assert (door.center_bl.x, door.center_bl.y) == pytest.approx(
                (door.center.x - car_x, door.center.y), abs=1e-6)
            live = {t.id for t in n.wall_tracker.tracks}
            assert {door.parent_wall_id0, door.parent_wall_id1} <= live
            assert door.parent_wall_id0 != door.parent_wall_id1
            assert door.hits >= 2 and door.since_seen == 0.0 and door.age > 0.0
            assert door.occupied is False and 0.0 < door.confidence <= 1.0
            assert door.n_beams > 0 and door.leaf_side == Door.LEAF_NONE
            # No camera on this node: the jambs' height was never measured.
            assert door.jamb_height_unknown is True and door.glass_suspect is False
            # /viz/doors went out on every cycle, and nobody counted subscribers.
            assert n.viz_doors_pub.publish.call_count == len(DRIVE_PAST)
            n.viz_doors_pub.get_subscription_count.assert_not_called()
        finally:
            n.destroy_node()

    def test_a_door_seen_from_one_place_is_a_candidate_kept_off_perception_doors(self):
        n = self._node()
        try:
            self._drive_past(n, xs=STAND)
            (door,) = n.door_tracker.doors
            assert door.state == door_detection.STATE_CANDIDATE and door.hits >= 2
            assert _last(n.doors_pub).doors == []
            # ...while /viz/doors draws it, faded.
            markers = _last(n.viz_doors_pub).markers
            assert {m.id // 4 for m in markers} == {door.id}
            assert [m.color.a for m in markers] == pytest.approx(
                [door_markers.CANDIDATE_ALPHA] * 4)
        finally:
            n.destroy_node()

    def test_door_publish_candidates_puts_the_candidate_on_perception_doors_too(self):
        n = self._node(door_publish_candidates=True)
        try:
            self._drive_past(n, xs=STAND)
            (door,) = _last(n.doors_pub).doors
            assert door.state == Door.STATE_CANDIDATE
        finally:
            n.destroy_node()

    def test_perception_doors_carries_only_the_confirmed_door_and_viz_doors_both(self):
        """One door of each state on one cycle; the flag adds the candidate."""
        for publish_candidates, ids in ((False, [1]), (True, [1, 2])):
            n = self._node(door_publish_candidates=publish_candidates)
            try:
                doors = [_door_track(1, door_detection.STATE_CONFIRMED),
                         _door_track(2, door_detection.STATE_CANDIDATE, x=2.0)]
                n._publish_doors(StampMsg(sec=7), doors, Pose2D(-1.0, 0.0, 0.0), 7.0)
                out = _last(n.doors_pub).doors
                assert [d.id for d in out] == ids
                assert [d.state for d in out] == [
                    Door.STATE_CONFIRMED, Door.STATE_CANDIDATE][:len(ids)]
                alpha = {m.id // 4: m.color.a for m in _last(n.viz_doors_pub).markers}
                assert alpha == pytest.approx({1: 1.0, 2: door_markers.CANDIDATE_ALPHA})
            finally:
                n.destroy_node()

    def test_every_door_field_goes_out_clamped_and_as_its_exact_wire_type(self):
        """Humble type-checks every field setter: numpy scalars are cast, ranges clamped."""
        n = self._node()
        try:
            door = _door_track(1, door_detection.STATE_CONFIRMED)
            door.jamb_height_unknown = False
            n._publish_doors(StampMsg(sec=7), [door], Pose2D(-1.0, 0.0, 0.0), 7.0)
            (m,) = _last(n.doors_pub).doors
            assert type(m.id) is int and type(m.hits) is int and m.hits == 4
            assert (m.parent_wall_id0, m.parent_wall_id1) == (3, 4)
            assert type(m.parent_wall_id0) is int
            assert m.n_beams == 65535 and type(m.n_beams) is int
            assert m.confidence == 1.0 and type(m.confidence) is float
            assert m.leaf_side == Door.LEAF_LEFT and type(m.leaf_side) is int
            assert m.since_seen == pytest.approx(0.5) and m.age == pytest.approx(1.0)
            assert m.jamb_height_unknown is False and m.glass_suspect is False
            assert (m.center_bl.x, m.center_bl.y) == pytest.approx((1.0, SIDE_Y))
        finally:
            n.destroy_node()

    def test_viz_doors_goes_out_every_cycle_even_with_no_door_and_no_subscriber(self):
        """So a mission bag records it: doors show in Foxglove on recorded runs."""
        n = self._node()
        try:
            self._drive_past(n, xs=STAND[:4], doorway=False)
            assert n.viz_doors_pub.publish.call_count == n.doors_pub.publish.call_count == 4
            assert _last(n.viz_doors_pub).markers == [] and _last(n.doors_pub).doors == []
            n.viz_doors_pub.get_subscription_count.assert_not_called()
        finally:
            n.destroy_node()

    def test_the_door_topics_are_not_created_with_door_detection_off(self):
        for enabled in (True, False):
            n = self._node(door_detection_enabled=enabled)
            try:
                topics = {p.topic_name for p in n.publishers}
                assert ('/perception/doors' in topics) is enabled
                assert ('/viz/doors' in topics) is enabled
                assert (n.door_tracker is not None) is enabled
            finally:
                n.destroy_node()

    def test_off_means_no_door_work_and_every_door_debug_key_null(self):
        n = self._node(door_detection_enabled=False)
        try:
            self._drive_past(n, xs=STAND[:3])
            d = _payloads(n)[-1]
            assert {k: d[k] for k in DOOR_DEBUG_KEYS} == dict.fromkeys(DOOR_DEBUG_KEYS)
        finally:
            n.destroy_node()

    def test_the_debug_payload_lists_every_gap_with_its_code_and_the_manhattan_prior(self):
        n = self._node()
        try:
            self._drive_past(n)
            payloads = _payloads(n)
            records = [r for d in payloads for r in d['door_candidates']]
            first, last = records[0], payloads[-1]['door_candidates'][-1]
            assert set(first) == set(last) == RECORD_KEYS
            # Two metres before the doorway it is seen too obliquely to decide...
            assert (first['code'], first['cls'], first['visible']) == (
                'oblique_view', 'weak', False)
            assert first['view_deg'] > 45.0
            # ...and at the end it is a door between the two left wall tracks.
            assert (last['code'], last['cls'], last['visible']) == ('', 'ok', True)
            assert last['width'] == pytest.approx(2 * DOOR_HALF, abs=0.04)
            assert last['n_beams'] >= last['n_required'] > 0
            assert last['behind_frac'] == 1.0 and last['boundary_hits'] == 0
            assert last['leaf_side'] == 'none' and last['glass_suspect'] is None
            assert last['jamb_height_unknown'] is True and last['jamb_tops'] == [None, None]
            assert last['host_offset_deg'] == pytest.approx(0.0, abs=0.5)
            assert len(set(last['parents'])) == 2
            assert last['centre_bl'] == pytest.approx([-DRIVE_PAST[-1], SIDE_Y], abs=0.04)
            d = payloads[-1]
            assert d['door_candidates_total'] == len(d['door_candidates']) == 1
            assert d['doors'] == d['doors_confirmed'] == d['door_n_confirmed'] == 1
            assert d['door_n_strikes'] == d['door_n_deleted_by_strikes'] == 0
            assert d['door_errors'] == 0 and d['door_disabled'] is False
            assert d['door_ms'] >= 0.0
            prior = d['door_manhattan']
            assert min(prior['dominant_deg'], 90.0 - prior['dominant_deg']) < 0.5
            assert prior['n_walls'] >= 2 and prior['strength'] > 0.9
            # No camera pair on this node: a gap reached the stereo stage and
            # found nothing to use.
            assert d['door_stereo'] == 'no_pair' and d['door_mask_age_s'] is None
            for gone in ('door_rejects', 'door_pieces', 'door_unsquared', 'door_panes',
                         'door_pane_kills', 'door_pane_suppressed'):
                assert gone not in d
        finally:
            n.destroy_node()

    def test_an_exception_in_the_door_cycle_is_logged_counted_and_the_walls_still_publish(self):
        n = self._node()
        logger = MagicMock()
        n.get_logger = lambda: logger
        try:
            with patch.object(door_detection, 'door_candidates',
                              side_effect=RuntimeError('boom')):
                self._drive_past(n, xs=DRIVE_PAST[:5])
            assert n.walls_pub.publish.call_count == 5
            assert n.virtual_pub.publish.call_count == 5
            assert n.doors_pub.publish.call_count == 0
            d = _payloads(n)[-1]
            assert d['door_errors'] == 5 and d['door_disabled'] is False
            assert d['door_candidates'] == [] and d['door_ms'] >= 0.0
            failed = [c for c in logger.error.call_args_list
                      if c.args[0].startswith("door detection failed: RuntimeError('boom')")]
            assert len(failed) == 5
            assert all(c.kwargs == {'throttle_duration_sec': 5.0} for c in failed)
        finally:
            n.destroy_node()

    def test_fifty_failures_in_a_row_turn_door_detection_off_for_good_with_one_error(self):
        n = self._node()
        logger = MagicMock()
        n.get_logger = lambda: logger
        try:
            with patch.object(door_detection, 'door_candidates',
                              side_effect=RuntimeError('boom')) as candidates:
                for i in range(DOOR_MAX_CONSECUTIVE_FAILURES + 10):
                    n._detect_doors(Pose2D(), None, 7.0 + 0.1 * i, StampMsg(sec=7))
            assert DOOR_MAX_CONSECUTIVE_FAILURES == 50
            assert candidates.call_count == 50
            d = n._door_payload()
            assert d['door_errors'] == 50 and d['door_disabled'] is True
            assert d['door_ms'] is None and d['door_candidates'] == []
            disabled = [c for c in logger.error.call_args_list if 'DISABLED' in c.args[0]]
            assert len(disabled) == 1
        finally:
            n.destroy_node()

    def test_a_good_cycle_resets_the_failure_streak(self):
        n = self._node()
        n.get_logger = lambda: MagicMock()
        try:
            boom = RuntimeError('boom')
            effects = [boom] * 49 + [[]] + [boom] * 49
            with patch.object(door_detection, 'door_candidates', side_effect=effects):
                for i in range(len(effects)):
                    n._detect_doors(Pose2D(), None, 7.0 + 0.1 * i, StampMsg(sec=7))
            assert n._door_errors == 98 and n._door_disabled is False
        finally:
            n.destroy_node()

    def test_a_candidate_that_ages_out_is_deleted_from_viz_doors(self):
        from visualization_msgs.msg import Marker
        n = self._node(door_candidate_max_age_sec=0.5)
        try:
            self._drive_past(n, xs=STAND)
            (door,) = n.door_tracker.doors
            assert {m.id // 4 for m in _last(n.viz_doors_pub).markers} == {door.id}
            # The doorway closes; past door_candidate_max_age_sec the door is gone.
            self._drive_past(n, xs=STAND, t0=7.6, doorway=False)
            assert n.door_tracker.doors == []
            deletes = [m for c in n.viz_doors_pub.publish.call_args_list
                       for m in c.args[0].markers if m.action == Marker.DELETE]
            assert sorted(m.id for m in deletes) == [4 * door.id + k for k in range(4)]
        finally:
            n.destroy_node()

    def test_one_parameter_is_the_door_width_and_the_collinear_gap(self):
        n = self._node(door_width_min_m=0.8)
        try:
            assert n.door_params.width_min_m == pytest.approx(0.8)
            assert n.wall_tracker.p.collinear_gap_m == pytest.approx(0.8)
            assert n.extraction_params.merge_gap == pytest.approx(0.8)
        finally:
            n.destroy_node()

    def test_a_person_detection_carries_its_class_to_the_door_check(self):
        from vision_msgs.msg import Detection3D, Detection3DArray, ObjectHypothesisWithPose
        n = self._node()
        try:
            msg = Detection3DArray()
            msg.header.frame_id = 'base_link'
            msg.header.stamp.sec = 7
            for cls, y in (('person', 0.0), ('chair', 1.0)):
                det = Detection3D()
                det.bbox.center.position.x, det.bbox.center.position.y = 3.0, y
                det.bbox.size.x = 0.4
                hyp = ObjectHypothesisWithPose()
                hyp.hypothesis.class_id = cls
                det.results.append(hyp)
                msg.detections.append(det)
            n._tf_static_cb(_tf_static())
            n._odom_cb(_odom_at(7.0, 0.0))
            n._detections_cb(msg)
            assert n._detection_classes == ['person', 'chair']
            car = n._poses.at(7.0)
            people = n._persons_at(car, 7.0)
            assert people == [pytest.approx((3.0, 0.0, 0.2))]
        finally:
            n.destroy_node()


# ---- the door stereo -----------------------------------------------------------

FX, CX, CY, W, H = 262.0, 320.0, 180.0, 640, 360
#: A camera looking LEFT, at the corridor's left wall: optical x = base x,
#: optical y = -base z, optical z = base y. Mounted 1.2 m up, so from 1.2 m
#: away it sees the wall from 0.4 to 2.0 m (the jamb height needs 1.8 m).
ROT_LEFT = np.array([[1.0, 0.0, 0.0], [0.0, 0.0, 1.0], [0.0, -1.0, 0.0]])
CAM_T_LEFT = np.array([0.12, 0.0, 1.2])


def _render_left(car_x, pane=False):
    """
    Return (labels, depth_mm) of the corridor's left side from the car at (car_x, 0).

    The floor, the left wall either side of the doorway, 2.5 m tall, and the
    office's far wall through it; pane True closes the doorway with a WALL
    surface ON the line that the stereo ranged (glass the LiDAR saw through).
    """
    u, v = np.meshgrid(np.arange(W), np.arange(H))
    d = np.stack([(u - CX) / FX, (v - CY) / FX, np.ones((H, W))], -1) @ ROT_LEFT.T
    cam = CAM_T_LEFT + np.array([car_x, 0.0, 0.0])
    best = np.full((H, W), np.inf)
    labels = np.full((H, W), scene_mask.CLASS_INVALID, dtype=np.uint8)
    planes = [(SIDE_Y + 3.0, -50.0, 50.0), (SIDE_Y, -50.0, -DOOR_HALF),
              (SIDE_Y, DOOR_HALF, 50.0)] + ([(SIDE_Y, -DOOR_HALF, DOOR_HALF)] if pane else [])
    with np.errstate(divide='ignore', invalid='ignore'):
        s = -cam[2] / d[..., 2]
        floor = (d[..., 2] < 0) & (s > 0)
        best[floor] = s[floor]
        labels[floor] = scene_mask.CLASS_FLOOR
        for y0, x_lo, x_hi in planes:
            s = (y0 - cam[1]) / d[..., 1]
            x = cam[0] + s * d[..., 0]
            z = cam[2] + s * d[..., 2]
            hit = (s > 0) & (x >= x_lo) & (x <= x_hi) & (z >= 0.0) & (z <= 2.5) & (s < best)
            best[hit] = s[hit]
            labels[hit] = scene_mask.CLASS_WALL
    depth = np.where(np.isfinite(best), np.round(best * 1000.0), 0).astype(np.uint16)
    return labels, depth


def _in_base(pose, world):
    """World (odom) points (N, 3) in base_link at `pose` (planar: z unchanged)."""
    xy = pose.inverse().transform_points(world[:, :2])
    return np.column_stack([xy, world[:, 2]])


class TestDoorStereo:
    """The door stereo: the mask/depth pair carried from its own stamp to the scan's."""

    def test_a_point_seen_at_the_scan_stamp_projects_where_the_mask_saw_it(self):
        """extrinsic_at_scan, on a car that moved AND turned between the two stamps."""
        pose_scan = Pose2D(1.3, -0.4, math.radians(25.0))
        pose_mask = Pose2D(0.9, -0.55, math.radians(12.0))
        world = np.array([[2.0, 1.5, 0.3], [0.5, 2.5, 1.7], [3.0, 0.8, 0.0]])
        rot, trans = extrinsic_at_scan(ROT_LEFT, CAM_T_LEFT, pose_scan, pose_mask)
        seen = scene_mask.project_to_image(_in_base(pose_mask, world), FX, FX, CX, CY,
                                           ROT_LEFT, CAM_T_LEFT)
        carried = scene_mask.project_to_image(_in_base(pose_scan, world), FX, FX, CX, CY,
                                              rot, trans)
        assert np.all(np.isfinite(seen))
        assert carried == pytest.approx(seen, abs=1e-9)
        # The optical depth too: the stereo tests compare it with the pixel's.
        assert ((_in_base(pose_scan, world) - trans) @ rot)[:, 2] == pytest.approx(
            ((_in_base(pose_mask, world) - CAM_T_LEFT) @ ROT_LEFT)[:, 2])
        # Uncompensated, the same points land elsewhere: it is not a no-op.
        naive = scene_mask.project_to_image(_in_base(pose_scan, world), FX, FX, CX, CY,
                                            ROT_LEFT, CAM_T_LEFT)
        assert np.max(np.abs(naive - seen)) > 20.0

    def test_no_motion_leaves_the_extrinsic_as_it_is(self):
        pose = Pose2D(0.7, 0.2, 0.3)
        rot, trans = extrinsic_at_scan(ROT_LEFT, CAM_T_LEFT, pose, pose)
        assert rot == pytest.approx(ROT_LEFT) and trans == pytest.approx(CAM_T_LEFT)

    @staticmethod
    def _node_with_odometry():
        n = TestDoors()._node()
        for t, x, yaw in ((7.0, 0.0, 0.0), (7.5, 0.5, math.radians(10.0))):
            odom = _odom_at(t, yaw)
            odom.pose.pose.position.x = x
            n._odom_cb(odom)
        return n

    @staticmethod
    def _blank_pair(mask_stamp):
        labels = np.zeros((H, W), dtype=np.uint8)
        depth = np.zeros((H, W), dtype=np.uint16)
        return lambda scan_stamp: (labels, depth, FX, FX, CX, CY, ROT_LEFT, CAM_T_LEFT,
                                   mask_stamp)

    def test_the_frame_is_carried_on_odometry_from_the_mask_stamp_to_the_scan_stamp(self):
        n = self._node_with_odometry()
        try:
            n._door_mask_geometry = self._blank_pair(7.2)
            car = n._poses.at(7.5)
            frame = n._door_stereo_frame(car, 7.5)
            rot, trans = extrinsic_at_scan(ROT_LEFT, CAM_T_LEFT, car, n._poses.at(7.2))
            assert frame.rotation == pytest.approx(rot)
            assert frame.translation == pytest.approx(trans)
            assert frame.depth_scale == 0.001
            assert n._door_stereo == 'compensated'
            assert n._door_mask_age == pytest.approx(0.3)
        finally:
            n.destroy_node()

    def test_an_old_mask_with_no_pose_at_its_stamp_gives_no_stereo_opinion(self):
        n = self._node_with_odometry()
        try:
            n._door_mask_geometry = self._blank_pair(6.0)     # before the odometry
            assert n._door_stereo_frame(n._poses.at(7.5), 7.5) is None
            assert n._door_stereo == 'no_pose' and n._door_mask_age == pytest.approx(1.5)
        finally:
            n.destroy_node()

    def test_a_mask_within_0_1_s_and_no_pose_is_used_as_it_is(self):
        n = self._node_with_odometry()
        try:
            # The odometry ends at 7.5; the scan at 7.64 still gets its end pose,
            # the mask at 7.70 none -- 0.06 s from the scan.
            n._door_mask_geometry = self._blank_pair(7.70)
            frame = n._door_stereo_frame(n._poses.at(7.64, 0.15), 7.64)
            assert frame.rotation == pytest.approx(ROT_LEFT)
            assert frame.translation == pytest.approx(CAM_T_LEFT)
            assert n._door_stereo == 'uncompensated'
        finally:
            n.destroy_node()

    @staticmethod
    def _camera(n, lag, pane=False):
        """Hand the node the left camera's pair, taken `lag` s before each scan from there."""
        def geometry(scan_stamp):
            mask_stamp = scan_stamp - lag
            then = n._poses.at(mask_stamp, 0.0)
            if then is None:
                return None
            labels, depth = _render_left(then.x, pane)
            return (labels, depth, FX, FX, CX, CY, ROT_LEFT, CAM_T_LEFT, mask_stamp)
        n._door_mask_geometry = geometry

    def test_the_stereo_measures_the_jambs_of_a_doorway_through_the_node(self):
        """An image 0.2 s older than the scan, from 0.2 m back: carried, it fits."""
        n = TestDoors()._node(door_publish_candidates=True)
        try:
            self._camera(n, lag=0.2)
            TestDoors._drive_past(n, xs=ABEAM)
            (door,) = _last(n.doors_pub).doors
            assert door.jamb_height_unknown is False
            d = _payloads(n)[-1]
            (rec,) = d['door_candidates']
            assert rec['code'] == '' and rec['jamb_height_unknown'] is False
            assert min(rec['jamb_tops']) >= 1.8
            assert d['door_stereo'] == 'compensated'
            assert d['door_mask_age_s'] == pytest.approx(0.2)
        finally:
            n.destroy_node()

    def test_the_glass_veto_rejects_a_pane_the_stereo_ranged_only_when_it_is_on(self):
        for veto, code in ((False, ''), (True, 'stereo_wall')):
            n = TestDoors()._node(door_publish_candidates=True, door_stereo_glass_veto=veto)
            try:
                self._camera(n, lag=0.0, pane=True)
                TestDoors._drive_past(n, xs=ABEAM)
                assert [r['code'] for r in _payloads(n)[-1]['door_candidates']] == [code]
                assert bool(_last(n.doors_pub).doors) is (not veto)
            finally:
                n.destroy_node()


# ---- the door parameters: three places, one default -------------------------

def _stack_params():
    path = os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(
        os.path.realpath(__file__)))), 'f1tenth_params', 'config', 'stack_params.yaml')
    with open(path) as f:
        return yaml.safe_load(f)


class TestDoorParamsCoupling:
    """
    Every door default lives in three places and they must agree (CLAUDE.md).

    DoorParams' default, the node's declare_parameter fallback and
    stack_params.yaml's lidar_front_wall_door_* (which the launch file
    passes, test_lidar_front_wall_launch.py). A change to one is a change to
    all three, in the same commit.
    """

    @staticmethod
    def _node():
        # door_on=False: no override at all, so every value read is a fallback.
        return TestDoors()._node(door_on=False)

    def test_the_node_declares_exactly_the_stack_params_door_keys_with_their_defaults(self):
        doc = _stack_params()
        keys = {k[len('lidar_front_wall_'):] for k in doc
                if k.startswith('lidar_front_wall_door_')}
        n = self._node()
        try:
            declared = {name for name in n.list_parameters([], 0).names
                        if name.startswith('door_')}
            assert declared == keys
            for name in sorted(declared):
                fallback = n.get_parameter(name).value
                default = doc['lidar_front_wall_' + name]['default']
                # The type too: the launch passes the yaml's with an explicit
                # value_type, and a declared int refuses a float at startup.
                assert type(fallback) is type(default), name
                assert fallback == default, name
        finally:
            n.destroy_node()

    def test_the_node_s_fallbacks_build_exactly_the_core_s_door_params(self):
        """Degrees in the parameter, radians in DoorParams: converted, still equal."""
        core = door_detection.DoorParams()
        n = self._node()
        try:
            for f in dataclasses.fields(door_detection.DoorParams):
                assert getattr(n.door_params, f.name) == getattr(core, f.name), f.name
        finally:
            n.destroy_node()

    def test_the_door_numbers_the_node_takes_from_elsewhere_have_the_core_s_defaults(self):
        """wall_tol_m is the tracker's negative_tol_m; width_min_m the collinear gap."""
        core = door_detection.DoorParams()
        assert get_value('lidar_front_wall_wall_negative_tol_m') == core.wall_tol_m
        assert get_value('lidar_front_wall_door_width_min_m') == core.width_min_m
        assert get_value('lidar_front_wall_door_collinear_angle_deg') == pytest.approx(
            math.degrees(core.collinear_angle_rad))
        assert get_value('lidar_front_wall_door_square_incidence_deg') == pytest.approx(
            math.degrees(core.square_incidence_rad))
        assert get_value('lidar_front_wall_door_manhattan_tau_sec') == core.manhattan_tau_s
        assert get_value('lidar_front_wall_door_candidate_max_age_sec') == \
            core.candidate_max_age_s
        assert get_value('lidar_front_wall_door_confirmed_max_age_sec') == \
            core.confirmed_max_age_s

    def test_candidates_stay_off_perception_doors_by_default(self):
        assert get_value('lidar_front_wall_door_publish_candidates') is False


class TestFixedWallHeight:
    """Every wall marker is wall_viz_height_m tall from the floor, whatever was measured."""

    def test_the_slab_is_2_5_m_on_the_floor_whatever_the_track_says(self):
        from visualization_msgs.msg import Marker
        n = TestDoors()._node()
        try:
            TestDoors._drive(n, cycles=6, gap=False)
            car = n._car_pose
            tracks = n.wall_tracker.published(car)
            assert tracks
            heights = [(0.1, 1.3, True), (0.0, 0.4, True), (-0.2, 4.0, True),
                       (0.0, 2.5, False)]
            for trk, (z0, z1, measured) in zip(tracks, heights):
                trk.z_min, trk.z_max, trk.height_measured = z0, z1, measured
            n._publish_walls(_door_scan(7.6, gap=False).header.stamp, tracks, car)
            slabs = [m for m in _last(n.walls_markers_pub).markers if m.type == Marker.CUBE]
            assert len(slabs) == len(tracks)
            for slab in slabs:
                assert slab.scale.z == pytest.approx(2.5)
                assert slab.pose.position.z == pytest.approx(1.25)
            # The data is untouched: the message still carries the track's height.
            assert _last(n.walls_pub).tracks[0].z_max == pytest.approx(1.3)
        finally:
            n.destroy_node()

    def test_the_height_is_a_parameter_and_feeds_viz_walls_too(self):
        n = TestDoors()._node(wall_viz_height_m=3.0)
        try:
            assert n.wall_viz_height_m == pytest.approx(3.0)
            assert n._viz_walls_builder.wall_height_m == pytest.approx(3.0)
            assert n._viz_doors_builder.height_m == pytest.approx(3.0)
        finally:
            n.destroy_node()
