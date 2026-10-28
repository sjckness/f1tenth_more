"""freespace_door_node's WIRING -- the glue, not the geometry.

freespace_doors.py's own tests cover what a door is. These cover what the node
does with one: the gates that decide whether the detector exists at all, the
grid-to-detector handover, the beam policy, the four shared outputs, the
staleness guard, and the guarantee that a cycle which throws neither kills the
node nor loses the doors already confirmed.

Same convention as test_semantic_doors_node.py: a real, never-spun rclpy node
built through parameter_overrides, publishers mocked, callbacks called
directly.

Run standalone: python3 -m pytest test/test_freespace_door_node.py -v
"""

import json
import math
from unittest.mock import MagicMock

import numpy as np
import pytest
import rclpy
from geometry_msgs.msg import Transform, TransformStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.parameter import Parameter
from sensor_msgs.msg import LaserScan

from f1tenth_costmap import freespace_doors as fd
from f1tenth_costmap.freespace_door_node import (
    FreespaceDoorNode, beams_from_scan, compose, yaw_from_quaternion,
)

RES = 0.05
ORIGIN = (-6.0, -6.0)
CELLS = 240
GAP = 0.90


@pytest.fixture(scope='module', autouse=True)
def _rclpy_context():
    rclpy.init()
    yield
    if rclpy.ok():
        rclpy.shutdown()


# ---------------------------------------------------------------------------
# the scene: the same room - door - room the geometry tests use
# ---------------------------------------------------------------------------
def _grid_array():
    a = np.full((CELLS, CELLS), -1, dtype=np.int16)

    def rect(x0, y0, x1, y1, v):
        r0 = int(round((y0 - ORIGIN[1]) / RES))
        r1 = int(round((y1 - ORIGIN[1]) / RES))
        c0 = int(round((x0 - ORIGIN[0]) / RES))
        c1 = int(round((x1 - ORIGIN[0]) / RES))
        a[r0:r1, c0:c1] = v

    rect(-4, -4, 4, 4, 0)
    rect(-4.05, -4.05, 4.05, -4.0, 100)
    rect(-4.05, 4.0, 4.05, 4.05, 100)
    rect(-4.05, -4.05, -4.0, 4.05, 100)
    rect(4.0, -4.05, 4.05, 4.05, 100)
    rect(0.0, -4.0, 0.05, -GAP / 2.0, 100)
    rect(0.0, GAP / 2.0, 0.05, 4.0, 100)
    return a


def _grid_msg():
    msg = OccupancyGrid()
    msg.header.frame_id = 'map'
    msg.info.resolution = RES
    msg.info.width = CELLS
    msg.info.height = CELLS
    msg.info.origin.position.x = ORIGIN[0]
    msg.info.origin.position.y = ORIGIN[1]
    msg.info.origin.orientation.w = 1.0
    msg.data = [int(v) for v in _grid_array().ravel()]
    return msg


def _odom(x, y, yaw=0.0):
    msg = Odometry()
    msg.header.frame_id = 'map'
    msg.pose.pose.position.x = float(x)
    msg.pose.pose.position.y = float(y)
    msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
    msg.pose.pose.orientation.w = math.cos(yaw / 2.0)
    return msg


def _scan_through_the_door(car_x=-2.0, n=61, range_max=25.0):
    """A scan from the car, in the LASER frame, aimed across the opening.

    The laser frame is the identity on base_link in these tests (the stubbed
    /tf_static transform below), so a bearing here is a map bearing for a car
    at yaw 0, and every beam is given NO RETURN -- range_max -- which is what
    a beam through an open doorway into an 8 m room actually does.
    """
    msg = LaserScan()
    msg.header.frame_id = 'laser'
    half = math.atan2(GAP / 2.0 - 0.10, abs(car_x))
    msg.angle_min = -half
    msg.angle_max = half
    msg.angle_increment = (2.0 * half) / (n - 1)
    msg.range_min = 0.05
    msg.range_max = range_max
    msg.ranges = [float('inf')] * n
    return msg


def _identity_transform_stamped():
    ts = TransformStamped()
    ts.transform = Transform()
    ts.transform.rotation.w = 1.0
    return ts


def _node(**params):
    base = {'freespace_doors_enabled': True, 'door_source': 'freespace'}
    base.update(params)
    node = FreespaceDoorNode(
        parameter_overrides=[Parameter(k, value=v) for k, v in base.items()])
    if not node.enabled:
        # A disabled node has no buffer and no publishers to stub -- which is
        # itself what the gate tests assert.
        return node
    node.tf_buffer.lookup_transform = MagicMock(
        return_value=_identity_transform_stamped())
    for name in ('doors_pub', 'viz_pub', 'objects_pub', 'debug_pub',
                 'debug_viz_pub'):
        if getattr(node, name, None) is not None:
            setattr(node, name, MagicMock())
    if getattr(node, '_tf', None) is not None:
        node._tf = MagicMock()
    return node


def _feed(node, car_x=-2.0, scan=True):
    node._map_cb(_grid_msg())
    node._pose_cb(_odom(car_x, 0.0))
    if scan:
        node._scan_cb(_scan_through_the_door(car_x))


def _debug(node):
    """The JSON of the last /costmap/freespace_doors/debug publish."""
    return json.loads(node.debug_pub.publish.call_args[0][0].data)


# ---------------------------------------------------------------------------
# the two gates
# ---------------------------------------------------------------------------
def test_door_source_semantic_leaves_this_detector_off():
    """Nothing subscribed, nothing published, nothing computed."""
    node = _node(door_source='semantic')
    assert node.enabled is False
    assert node._tracker is None
    assert not hasattr(node, 'doors_pub')
    assert not hasattr(node, 'timer')
    node.destroy_node()


def test_freespace_doors_disabled_leaves_this_detector_off():
    node = _node(freespace_doors_enabled=False)
    assert node.enabled is False
    assert node._tracker is None
    node.destroy_node()


def test_an_unknown_door_source_is_refused_at_startup():
    """A typo must stop the node, not silently turn every detector off."""
    with pytest.raises(ValueError, match='door_source'):
        _node(door_source='freespaec')


def test_every_known_door_source_is_accepted_and_only_one_runs_this_node():
    from f1tenth_costmap.semantic_doors import DOOR_SOURCES
    assert 'freespace' in DOOR_SOURCES
    for source in DOOR_SOURCES:
        node = _node(door_source=source)
        assert node.enabled == (source == 'freespace'), source
        node.destroy_node()


def test_a_non_positive_rate_is_refused():
    with pytest.raises(ValueError, match='rate_hz'):
        _node(freespace_doors_rate_hz=0.0)


# ---------------------------------------------------------------------------
# one cycle, end to end
# ---------------------------------------------------------------------------
def test_a_cycle_confirms_the_door_and_publishes_the_four_outputs():
    node = _node()
    _feed(node)
    for _ in range(node.params.confirm_hits):
        node._cycle()

    doors = node.doors_pub.publish.call_args[0][0]
    assert len(doors.doors) == 1
    assert doors.header.frame_id == 'map'
    door = doors.doors[0]
    assert door.width == pytest.approx(GAP, abs=RES)
    assert door.center.x == pytest.approx(0.025, abs=RES)
    # The crossing direction points away from the car, which sat at x = -2.
    assert door.normal.x > 0.9

    objects = node.objects_pub.publish.call_args[0][0]
    assert [o.class_id for o in objects.objects] == ['door']
    assert objects.objects[0].radius_m == pytest.approx(GAP / 2.0, abs=RES)

    namespaces = {m.ns for m in node.viz_pub.publish.call_args[0][0].markers}
    assert namespaces == {'viz_doors_semantic'}

    sent = node._tf.sendTransform.call_args[0][0]
    assert [t.child_frame_id for t in sent] == [f'door_{door.id}']
    assert sent[0].header.frame_id == 'map'
    node.destroy_node()


def test_the_outputs_are_the_SAME_topics_the_semantic_detector_uses():
    """What makes enter_door work whichever detector found the door.

    f1tenth_behavior's EnterDoor reads /perception/doors_semantic and nothing
    else. Renaming the topic per door source would mean every consumer
    growing a door_source branch, so the three sources -- which are mutually
    exclusive -- deliberately share these names.
    """
    from f1tenth_costmap import semantic_door_markers as sdm
    node = _node()
    assert node.get_parameter('doors_topic').value == sdm.TOPICS[0]
    assert node.get_parameter('viz_doors_topic').value == sdm.TOPICS[1]
    assert node.get_parameter('door_objects_topic').value == sdm.TOPICS[2]
    node.destroy_node()


def test_the_debug_json_carries_the_same_keys_as_the_semantic_detector():
    """A health plot does not change when door_source does, only its topic."""
    node = _node()
    _feed(node)
    node._cycle()
    d = _debug(node)
    for key in ('door_ms', 'door_errors', 'door_codes', 'doors_confirmed'):
        assert key in d
    assert d['source'] == 'freespace'
    assert d['door_codes'].get('ok') == 1
    assert d['skeleton_cells'] > 0
    assert d['door_ms'] >= 0.0
    node.destroy_node()


def test_rejection_codes_reach_the_debug_dict():
    node = _node()
    _feed(node, scan=False)          # the grid says doorway, no scan at all
    node._cycle()
    d = _debug(node)
    assert d['door_codes'] == {'not_beyond': 1}
    assert d['doors_confirmed'] == 0
    node.destroy_node()


def test_the_debug_markers_draw_the_skeleton_and_every_candidate():
    node = _node()
    _feed(node)
    node._cycle()
    markers = node.debug_viz_pub.publish.call_args[0][0].markers
    assert {m.ns for m in markers} == {'viz_doors_freespace'}
    by_id = {m.id: m for m in markers}
    assert len(by_id[0].points) > 10             # the skeleton
    assert len(by_id[1].points) == 1             # one accepted candidate
    assert by_id[2].points == [] and by_id[3].points == []
    node.destroy_node()


def test_debug_markers_can_be_turned_off():
    node = _node(freespace_doors_debug_markers=False)
    assert node.debug_viz_pub is None
    _feed(node)
    node._cycle()                                # must not raise
    node.destroy_node()


# ---------------------------------------------------------------------------
# what the node will not do
# ---------------------------------------------------------------------------
def test_no_grid_or_no_pose_means_no_cycle():
    node = _node()
    node._cycle()
    assert node.doors_pub.publish.call_count == 0
    node._map_cb(_grid_msg())
    node._cycle()
    assert node.doors_pub.publish.call_count == 0, 'ran without a pose'
    node._pose_cb(_odom(-2.0, 0.0))
    node._cycle()
    assert node.doors_pub.publish.call_count == 1
    node.destroy_node()


def test_a_stale_grid_skips_the_cycle_and_keeps_the_doors():
    """The grid is LATCHED: an old one keeps arriving forever.

    Without the age check a dead slam_toolbox would leave this node
    confidently republishing doors off a grid minutes out of date. A skipped
    cycle must not lose what is already confirmed either -- the tracker holds
    it, and only the cycle is skipped.
    """
    node = _node(freespace_doors_grid_max_age_s=5.0)
    _feed(node)
    for _ in range(node.params.confirm_hits):
        node._cycle()
    assert len(node._tracker.confirmed()) == 1
    published = node.doors_pub.publish.call_count

    node._grid_stamp = node._now_sec() - 60.0
    node._cycle()
    assert node.doors_pub.publish.call_count == published, 'published off a stale grid'
    assert len(node._tracker.confirmed()) == 1, 'a skipped cycle lost the door'
    node.destroy_node()


def test_a_cycle_that_throws_is_counted_and_does_not_kill_the_node():
    node = _node()
    _feed(node)
    node._detect = MagicMock(side_effect=RuntimeError('boom'))
    for _ in range(3):
        node._cycle()
    assert node._errors == 3
    assert _debug(node)['door_errors'] == 3
    assert node._tracker is not None
    node.destroy_node()


def test_enough_consecutive_failures_turn_the_detector_off_for_good():
    node = _node()
    _feed(node)
    node._detect = MagicMock(side_effect=RuntimeError('boom'))
    for _ in range(FreespaceDoorNode.FREESPACE_MAX_ERRORS):
        node._cycle()
    assert node._tracker is None
    node._detect.reset_mock()
    node._cycle()
    assert node._detect.call_count == 0
    node.destroy_node()


def test_a_missing_laser_transform_stops_confirmation_not_the_cycle():
    """No base_link -> laser edge means no beams, so nothing can confirm.

    The cycle still runs -- the geometry is found and drawn -- which is what
    makes the debug markers useful while a TF problem is being diagnosed.
    """
    from tf2_ros import LookupException
    node = _node()
    _feed(node)
    node.tf_buffer.lookup_transform = MagicMock(side_effect=LookupException('no'))
    node._laser_bl = None
    for _ in range(node.params.confirm_hits):
        node._cycle()
    assert _debug(node)['door_codes'] == {'not_beyond': 1}
    assert node._tracker.confirmed() == []
    node.destroy_node()


# ---------------------------------------------------------------------------
# the pure helpers
# ---------------------------------------------------------------------------
def test_compose_puts_the_laser_where_the_car_is_looking():
    # Car at (1, 2) facing +y, laser 0.3 m ahead of base_link.
    x, y, yaw = compose((1.0, 2.0, math.pi / 2.0), (0.3, 0.0, 0.0))
    assert (x, y) == pytest.approx((1.0, 2.3))
    assert yaw == pytest.approx(math.pi / 2.0)


def test_yaw_from_quaternion_round_trips():
    for yaw in (-2.0, -0.3, 0.0, 1.1, 3.0):
        q = type('Q', (), {'x': 0.0, 'y': 0.0,
                           'z': math.sin(yaw / 2.0), 'w': math.cos(yaw / 2.0)})
        assert yaw_from_quaternion(q) == pytest.approx(yaw)


def test_a_beam_with_no_return_is_placed_at_range_max_not_dropped():
    """The policy that separates this detector's scan use from the others'.

    A beam that goes through a doorway and hits nothing is the STRONGEST
    evidence the doorway is open. unknown_objects.scan_points drops those
    beams -- correctly, for its question, which is whether a return LIES ON
    something. Here dropping them would make an open door look shut.
    """
    scan = LaserScan()
    scan.angle_min = 0.0
    scan.angle_increment = math.pi / 2.0
    scan.range_min = 0.10
    scan.range_max = 10.0
    scan.ranges = [float('inf'), 2.0, float('nan'), 0.01]
    beams = beams_from_scan(scan, (0.0, 0.0, 0.0))
    # inf -> 10 m along +x, 2.0 -> 2 m along +y, nan -> 10 m along -x,
    # and 0.01 (below range_min) dropped entirely.
    assert beams.points.shape == (3, 2)
    assert beams.points[0] == pytest.approx((10.0, 0.0), abs=1e-6)
    assert beams.points[1] == pytest.approx((0.0, 2.0), abs=1e-6)
    assert beams.points[2] == pytest.approx((-10.0, 0.0), abs=1e-6)


def test_beams_are_placed_in_the_map_frame():
    scan = LaserScan()
    scan.angle_min = 0.0
    scan.angle_increment = 1.0
    scan.range_min = 0.10
    scan.range_max = 10.0
    scan.ranges = [3.0]
    # Laser at (1, 2) in map, rotated 90 degrees: a bearing-0 beam of 3 m
    # must land at (1, 5), not (4, 2).
    beams = beams_from_scan(scan, (1.0, 2.0, math.pi / 2.0))
    assert beams.points[0] == pytest.approx((1.0, 5.0), abs=1e-6)
    assert beams.laser == (1.0, 2.0)


def test_an_empty_scan_is_no_beams_rather_than_a_crash():
    scan = LaserScan()
    scan.angle_min = 0.0
    scan.angle_increment = 1.0
    scan.range_min = 0.1
    scan.range_max = 10.0
    scan.ranges = []
    assert beams_from_scan(scan, (0.0, 0.0, 0.0)).points.shape == (0, 2)
