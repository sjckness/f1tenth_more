"""
semantic_layer_node's object memory wiring: topics, sensors, freshness, Foxglove.

Same convention as test_semantic_layer_node.py: a real, never-spun rclpy Node
built with parameter_overrides, callbacks called directly, publishers mocked.
TF is an identity stub (camera body frame = laser frame = base_link), and the
node's clock is replaced by a fake one so 15 s of memory take no wall time.
The lifecycle rules themselves are test_object_memory.py's; this file checks
that the node feeds them the right sensors and publishes what they decide.

Run standalone: python3 -m pytest test/test_object_memory_node.py -v
"""

import math
from types import SimpleNamespace
from unittest.mock import MagicMock

from f1tenth_costmap import object_memory as om
from f1tenth_costmap.semantic_layer_node import build_object_markers, SemanticLayerNode
from f1tenth_messages.msg import SemanticObject
from f1tenth_params import semantic_objects
from f1tenth_params.param_defaults import get_value
from geometry_msgs.msg import Transform, TransformStamped
from nav_msgs.msg import Odometry
import pytest
import rclpy
from rclpy.exceptions import InvalidParameterValueException
from rclpy.parameter import Parameter
from sensor_msgs.msg import LaserScan
from vision_msgs.msg import (
    Detection3D,
    Detection3DArray,
    ObjectHypothesis,
    ObjectHypothesisWithPose,
)

DT = 0.1


@pytest.fixture(scope='module', autouse=True)
def _rclpy_context():
    rclpy.init()
    yield
    if rclpy.ok():
        rclpy.shutdown()


def _identity():
    ts = TransformStamped()
    ts.transform = Transform()
    ts.transform.rotation.w = 1.0
    return ts


class Car:
    """A node plus a fake clock and the three sensor streams it reads."""

    def __init__(self, params=None):
        overrides = {'confirm_hit_count': 1, 'score_threshold': 0.0}
        overrides.update(params or {})
        self.node = SemanticLayerNode(
            parameter_overrides=[Parameter(k, value=v) for k, v in overrides.items()])
        self.now = 1000.0
        self.node._now_sec = lambda: self.now
        for pub in ('marker_pub', 'tracks_pub', 'objects_pub', 'viz_objects_pub'):
            getattr(self.node, pub).publish = MagicMock()
        self.node.viz_objects_pub.get_subscription_count = MagicMock(return_value=0)
        self.node.tf_buffer.lookup_transform = MagicMock(return_value=_identity())
        self.pose = (0.0, 0.0, 0.0)
        self.scan_ranges = None           # None: no LiDAR at all
        self.camera = True

    def tick(self, detections=(), seconds=DT):
        """Advance the clock in DT steps: pose, scan, camera batch, memory cycle."""
        for _ in range(int(round(seconds / DT))):
            self.now += DT
            self.node._pose_cb(_pose(*self.pose))
            if self.scan_ranges is not None:
                self.node._scan_cb(_scan(self.scan_ranges))
            if self.camera:
                self.node._detections_cb(_detections(self.now, detections))
            self.node._memory_step(self.now)

    def objects(self):
        return list(self.node.objects_pub.publish.call_args[0][0].objects)

    def one(self):
        objs = self.objects()
        assert len(objs) == 1, objs
        return objs[0]

    def legacy_track_ids(self):
        return [d.id for d in self.node.tracks_pub.publish.call_args[0][0].detections]

    def close(self):
        self.node.destroy_node()


@pytest.fixture
def car():
    c = Car()
    yield c
    c.close()


def _pose(x, y, yaw):
    msg = Odometry()
    msg.pose.pose.position.x = x
    msg.pose.pose.position.y = y
    msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
    msg.pose.pose.orientation.w = math.cos(yaw / 2.0)
    return msg


def _scan(ranges):
    msg = LaserScan()
    msg.header.frame_id = 'laser'
    msg.angle_min = -math.radians(135.0)
    msg.angle_increment = math.radians(0.25)
    msg.angle_max = msg.angle_min + (len(ranges) - 1) * msg.angle_increment
    msg.range_min = 0.02
    msg.range_max = 10.0
    msg.ranges = list(ranges)
    return msg


def _free_scan(occluder=None):
    """1081 beams of open space; occluder = (bearing, range) over +-3 deg."""
    ranges = [10.0] * 1081
    if occluder is not None:
        bearing, rng = occluder
        for i in range(1081):
            if abs(math.radians(-135.0 + 0.25 * i) - bearing) <= math.radians(3.0):
                ranges[i] = rng
    return ranges


def _detections(stamp_sec, entries):
    """entries: (class_id, x, y, width) in the camera frame (identity to base_link)."""
    msg = Detection3DArray()
    msg.header.frame_id = 'zed2_left_camera_frame'
    msg.header.stamp.sec = int(stamp_sec)
    msg.header.stamp.nanosec = int(round((stamp_sec - int(stamp_sec)) * 1e9)) % 1000000000
    for class_id, x, y, width in entries:
        det = Detection3D()
        hyp = ObjectHypothesisWithPose()
        hyp.hypothesis = ObjectHypothesis(class_id=class_id, score=0.9)
        det.results.append(hyp)
        det.bbox.center.position.x = x
        det.bbox.center.position.y = y
        det.bbox.center.orientation.w = 1.0
        det.bbox.size.x = width
        msg.detections.append(det)
    return msg


CHAIR = [('chair', 3.0, 0.0, 0.45)]


def _pitched(pitch, t=(0.12, 0.0, 0.15)):
    """Return camera -> base_link: a pitch about y and the ZED mount's translation."""
    ts = TransformStamped()
    ts.transform = Transform()
    ts.transform.translation.x, ts.transform.translation.y, ts.transform.translation.z = t
    ts.transform.rotation.y = math.sin(pitch / 2.0)
    ts.transform.rotation.w = math.cos(pitch / 2.0)
    return ts


class TestTheCentreHeight:
    """z_m: the centre's base_link height, what mpc_corr's obstacle gate tests."""

    def _with_height(self, car, z_cam):
        msg = _detections(car.now + DT, CHAIR)
        msg.detections[0].bbox.center.position.z = z_cam
        car.now += DT
        car.node._pose_cb(_pose(*car.pose))
        car.node._detections_cb(msg)
        car.node._memory_step(car.now)

    def test_identity_camera_publishes_the_detection_height(self, car):
        self._with_height(car, 0.7)
        assert car.one().z_m == pytest.approx(0.7)

    def test_the_camera_pitch_and_mount_height_are_applied(self, car):
        pitch = 0.1
        car.node.tf_buffer.lookup_transform = MagicMock(return_value=_pitched(pitch))
        self._with_height(car, 0.3)
        # (x, z) = (3, 0.3) rotated by the pitch about y: z' = -x sin + z cos, + 0.15.
        expected = -3.0 * math.sin(pitch) + 0.3 * math.cos(pitch) + 0.15
        assert car.one().z_m == pytest.approx(expected, abs=1e-6)


class TestPublishedEveryCycle:

    def test_a_confirmed_object_is_published_tracked_in_the_map_frame(self, car):
        car.tick(CHAIR, seconds=0.3)
        msg = car.node.objects_pub.publish.call_args[0][0]
        assert msg.header.frame_id == 'map'
        assert msg.memory_decay_s == pytest.approx(15.0)
        assert msg.fov_miss_remove_s == pytest.approx(1.5)
        obj = car.one()
        assert obj.state == SemanticObject.TRACKED and obj.class_id == 'chair'
        assert (obj.x, obj.y) == pytest.approx((3.0, 0.0))
        assert obj.radius_m == pytest.approx(0.225)
        assert obj.effective_radius_m == pytest.approx(0.225)
        assert obj.confidence == 1.0 and obj.in_fov and obj.in_fov_stereo
        assert str(obj.id) in car.legacy_track_ids()

    def test_publishes_even_with_no_objects_and_no_sensors(self, car):
        car.camera = False
        car.tick(seconds=0.2)
        assert car.node.objects_pub.publish.call_count == 2
        assert car.objects() == []


class TestLifecycleThroughTheNode:

    def test_turning_away_makes_it_memory_then_it_expires(self, car):
        car.tick(CHAIR, seconds=0.3)
        object_id = car.one().id
        car.pose = (0.0, 0.0, math.pi)    # facing away; the camera keeps running
        car.tick(seconds=1.0)
        obj = car.one()
        assert obj.id == object_id and obj.state == SemanticObject.MEMORY
        assert not obj.in_fov
        assert obj.time_in_memory_s == pytest.approx(1.0, abs=1e-3)
        assert obj.memory_remaining_s == pytest.approx(14.0, abs=1e-3)
        assert car.legacy_track_ids() == []      # the old outputs dropped it as before
        car.tick(seconds=13.8)
        assert car.one().confidence == pytest.approx(0.2 / 15.0, abs=1e-3)
        car.tick(seconds=0.3)
        assert car.objects() == []

    def test_in_view_and_not_detected_is_removed_after_1_5_s(self, car):
        car.tick(CHAIR, seconds=0.3)
        car.tick(seconds=1.4)
        assert car.one().state == SemanticObject.TRACKED
        assert car.one().fov_unseen_s == pytest.approx(1.4, abs=1e-3)
        car.tick(seconds=0.2)
        assert car.objects() == []

    def test_a_silent_camera_is_not_looking(self, car):
        car.tick(CHAIR, seconds=0.3)
        car.camera = False                # detector stalls: no batches at all
        car.tick(seconds=3.0)
        obj = car.one()
        assert obj.state == SemanticObject.MEMORY and not obj.in_fov_stereo

    def test_lidar_occlusion_keeps_it_as_memory(self, car):
        car.scan_ranges = _free_scan()
        car.tick(CHAIR, seconds=0.3)
        assert car.one().in_fov_lidar
        car.scan_ranges = _free_scan(occluder=(0.0, 1.5))   # someone steps in between
        car.tick(seconds=3.0)
        obj = car.one()
        assert obj.state == SemanticObject.MEMORY
        assert not obj.in_fov_lidar and not obj.in_fov_stereo and not obj.in_fov

    def test_reappearing_object_keeps_its_id(self, car):
        car.tick(CHAIR, seconds=0.3)
        object_id = car.one().id
        car.pose = (0.0, 0.0, math.pi)
        car.tick(seconds=4.0)
        assert car.one().state == SemanticObject.MEMORY
        car.pose = (0.0, 0.0, 0.0)
        car.tick([('chair', 3.1, 0.1, 0.45)])
        obj = car.one()
        assert obj.id == object_id and obj.state == SemanticObject.TRACKED
        assert car.legacy_track_ids() == [str(object_id)]

    def test_a_remembered_person_grows(self, car):
        car.tick([('person', 3.0, 0.0, 0.5)], seconds=0.3)
        car.pose = (0.0, 0.0, math.pi)
        car.tick(seconds=2.0)
        obj = car.one()
        assert obj.radius_m == pytest.approx(0.25)
        assert obj.effective_radius_m == pytest.approx(0.25 + 0.4, abs=1e-3)

    def test_a_pose_jump_does_not_move_a_remembered_object(self, car):
        car.tick(CHAIR, seconds=0.3)
        car.pose = (0.0, 0.0, math.pi)
        car.tick(seconds=1.0)
        before = (car.one().x, car.one().y)
        car.pose = (0.8, -0.5, math.pi + 0.4)   # map -> odom correction lands
        car.tick(seconds=1.0)
        assert (car.one().x, car.one().y) == before
        assert car.one().state == SemanticObject.MEMORY


class TestMemoryDisabled:

    def test_the_old_pruning_is_restored(self):
        car = Car({'memory_enabled': False, 'lost_miss_count': 2})
        try:
            car.tick(CHAIR, seconds=0.3)
            assert car.one().state == SemanticObject.TRACKED
            car.pose = (0.0, 0.0, math.pi)
            car.tick(seconds=0.3)
            assert car.node._tracks == [] and car.objects() == []
        finally:
            car.close()


class TestFoxglove:

    def test_nothing_is_built_without_a_subscriber(self, car):
        car.tick(CHAIR, seconds=0.3)
        car.node.viz_objects_pub.publish.assert_not_called()

    def test_memory_is_fainter_and_labelled_with_its_remaining_time(self):
        car = Car({'viz_objects_max_hz': 100.0})    # no rate cap inside a 0.1 s tick
        try:
            car.node.viz_objects_pub.get_subscription_count.return_value = 1
            car.tick(CHAIR + [('person', 2.0, 1.0, 0.5)], seconds=0.3)
            car.node._tracks[1].x_map = -4.0   # the person is behind the car now
            car.tick(CHAIR, seconds=1.0)
            markers = car.node.viz_objects_pub.publish.call_args[0][0].markers
            disks = {m.id: m for m in markers if m.ns == 'semantic_objects'}
            labels = {m.id: m.text for m in markers if m.ns == 'semantic_objects_label'}
            chair_id, person_id = (t.track_id for t in car.node._tracks)
            assert disks[person_id].color.a < disks[chair_id].color.a
            assert 'MEM 14.0s' in labels[person_id]
            assert 'MEM' not in labels[chair_id]
            assert disks[person_id].scale.x == pytest.approx(2.0 * (0.25 + 0.2), abs=1e-3)
            assert all(m.lifetime.sec > 0 or m.lifetime.nanosec > 0 for m in markers)
        finally:
            car.close()

    def test_a_removed_object_gets_deletes(self):
        mem = om.ObjectMemory()
        trk = SimpleNamespace(track_id=7, class_id='chair', x_map=1.0, y_map=0.0, width=0.45)
        params = om.MemoryParams()
        _markers, drawn = build_object_markers(
            [(trk, mem)], params, 'map', rclpy.time.Time().to_msg(), 0.6, set())
        markers, drawn = build_object_markers(
            [], params, 'map', rclpy.time.Time().to_msg(), 0.6, drawn)
        assert drawn == set()
        assert {(m.ns, m.id, m.action) for m in markers.markers} == {
            ('semantic_objects', 7, 2), ('semantic_objects_label', 7, 2)}


class TestParameters:

    def test_memory_decay_outside_10_to_20_s_is_rejected(self):
        with pytest.raises(InvalidParameterValueException):
            SemanticLayerNode(parameter_overrides=[Parameter('memory_decay_s', value=25.0)])

    def test_defaults_come_from_stack_params(self, car):
        """COUPLING: these four numbers are the task's; change yaml and tests together."""
        p = car.node.memory_params
        assert p.fov_miss_remove_s == get_value('semantic_layer_fov_miss_remove_s') == 1.5
        assert p.memory_decay_s == get_value('semantic_layer_memory_decay_s') == 15.0
        assert p.person_growth_mps == get_value('semantic_layer_person_memory_growth_mps') == 0.2
        assert p.person_growth_max_m == get_value(
            'semantic_layer_person_memory_growth_max_m') == 0.6
        assert car.node.memory_enabled is get_value('semantic_layer_memory_enabled') is True

    def test_state_constants_agree_everywhere(self):
        assert SemanticObject.TRACKED == om.TRACKED == semantic_objects.TRACKED == 0
        assert SemanticObject.MEMORY == om.MEMORY == semantic_objects.MEMORY == 1


class TestMpcHelperOnTheRealMessage:

    def test_published_objects_become_odom_circles(self, car):
        car.tick(CHAIR + [('person', 2.0, 1.0, 0.5)], seconds=0.3)
        car.node._tracks[1].x_map = -4.0
        car.tick(CHAIR, seconds=1.0)
        objs = car.objects()
        map_to_odom = (1.0, 0.0, 0.0)     # odom origin 1 m ahead of map's
        circles = semantic_objects.objects_to_odom_circles(objs, map_to_odom)
        assert circles[0] == pytest.approx((2.0, 0.0, 0.225), abs=1e-6)
        assert circles[1][2] == pytest.approx(0.25 + 0.2, abs=0.02)   # grown person
        tracked_only = semantic_objects.objects_to_odom_circles(
            objs, map_to_odom, include_memory=False)
        assert len(tracked_only) == 1
