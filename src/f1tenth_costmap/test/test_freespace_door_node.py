"""freespace_door_node's WIRING -- the glue, not the geometry.

freespace_doors.py's own tests cover what a door is. These cover what the node
does with one: the gates that decide whether the detector exists at all, the
scan-driven cycle and its throttle, the grid being OPTIONAL, the four shared
outputs, and the guarantee that a cycle which throws neither kills the node
nor loses the doors already confirmed.

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
import yaml
from geometry_msgs.msg import Transform, TransformStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.parameter import Parameter
from sensor_msgs.msg import LaserScan

from f1tenth_costmap import freespace_doors as fd
from f1tenth_costmap.freespace_door_node import (
    FreespaceDoorNode, compose, yaw_from_quaternion,
)

RES = 0.05
ORIGIN = (-8.0, -8.0)
CELLS = 400
DOOR_X, DOOR_W, WALL_Y = 3.0, 0.90, 1.0


@pytest.fixture(scope='module', autouse=True)
def _rclpy_context():
    rclpy.init()
    yield
    if rclpy.ok():
        rclpy.shutdown()


# ---------------------------------------------------------------------------
# the scene: a corridor with one door in its left wall, as test_scan_doors
# builds it -- a scan by ray casting, a grid by rasterising those same rays.
# ---------------------------------------------------------------------------
def _segments():
    from test_scan_doors import corridor
    return corridor(doors=(DOOR_X,), door_w=DOOR_W, half_w=WALL_Y)


def _scan_msg(car_x=0.0):
    from test_scan_doors import cast
    bearings, ranges = cast(_segments(), (car_x, 0.0, 0.0))
    msg = LaserScan()
    msg.header.frame_id = 'laser'
    msg.angle_min = float(bearings[0])
    msg.angle_increment = float(bearings[1] - bearings[0])
    msg.angle_max = float(bearings[-1])
    msg.range_min = 0.05
    msg.range_max = 20.0
    msg.ranges = [float(r) for r in ranges]
    return msg


def _grid_msg():
    from test_scan_doors import Grid
    g = Grid()
    for x in (0.0, 0.5, 1.0):
        g.ray_cast(_segments(), (x, 0.0, 0.0))
    msg = OccupancyGrid()
    msg.header.frame_id = 'map'
    msg.info.resolution = RES
    msg.info.width = CELLS
    msg.info.height = CELLS
    msg.info.origin.position.x = ORIGIN[0]
    msg.info.origin.position.y = ORIGIN[1]
    msg.info.origin.orientation.w = 1.0
    msg.data = [int(v) for v in g.a.ravel()]
    return msg


def _odom(x, y, yaw=0.0):
    msg = Odometry()
    msg.header.frame_id = 'map'
    msg.pose.pose.position.x = float(x)
    msg.pose.pose.position.y = float(y)
    msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
    msg.pose.pose.orientation.w = math.cos(yaw / 2.0)
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


def _drive(node, xs, grid=True):
    """Feed one scan and pose per position in `xs`, firing a cycle each time."""
    if grid:
        node._map_cb(_grid_msg())
    for x in xs:
        node._pose_cb(_odom(x, 0.0))
        node._last_cycle_sec = None          # bypass the throttle, not test it
        node._scan_cb(_scan_msg(x))
    return node


def _debug(node):
    """The JSON of the last /costmap/freespace_doors/debug publish."""
    return json.loads(node.debug_pub.publish.call_args[0][0].data)


# ---------------------------------------------------------------------------
# the two gates
# ---------------------------------------------------------------------------
def test_door_source_semantic_leaves_this_detector_off():
    node = _node(door_source='semantic')
    assert node.enabled is False
    assert node._tracker is None
    assert not hasattr(node, 'doors_pub')
    node.destroy_node()


def test_freespace_doors_disabled_leaves_this_detector_off():
    node = _node(freespace_doors_enabled=False)
    assert node.enabled is False
    assert node._tracker is None
    node.destroy_node()


def test_an_unknown_door_source_is_refused_at_startup():
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
# the parameter mapping the node relies on
# ---------------------------------------------------------------------------
def test_every_freespace_param_is_in_stack_params_with_the_shipped_default():
    """The node builds FreespaceParams by iterating __slots__.

    That loop is the reason there is no hand-written parameter list to drift
    out of step -- but it only works if stack_params.yaml has a
    `freespace_doors_<slot>` key for every field, with the same TYPE as the
    shipped default (the YAML value is cast to it). A field added to the
    dataclass without a key here would raise at startup, on the car.
    """
    import os
    path = os.path.join(os.path.dirname(os.path.dirname(
        os.path.dirname(os.path.abspath(__file__)))),
        'f1tenth_params', 'config', 'stack_params.yaml')
    with open(path) as f:
        raw = yaml.safe_load(f)
    proto = fd.FreespaceParams()
    for slot in fd.FreespaceParams.__slots__:
        key = 'freespace_doors_' + slot
        assert key in raw, f'{key} missing from stack_params.yaml'
        assert 'description' in raw[key], key
        shipped, default = raw[key]['default'], getattr(proto, slot)
        assert type(shipped) is type(default), (key, type(shipped), type(default))
        assert shipped == default, (key, shipped, default)


def test_the_node_builds_its_params_from_stack_params():
    node = _node(freespace_doors_jump_min_m=0.42, freespace_doors_use_skeleton=False)
    assert node.params.jump_min_m == pytest.approx(0.42)
    assert node.params.use_skeleton is False
    assert node.params.width_min_m == pytest.approx(fd.FreespaceParams().width_min_m)
    node.destroy_node()


# ---------------------------------------------------------------------------
# one cycle, end to end
# ---------------------------------------------------------------------------
def test_driving_past_the_door_confirms_it_and_publishes_the_four_outputs():
    node = _drive(_node(), (0.0, 0.3, 0.6))

    doors = node.doors_pub.publish.call_args[0][0]
    assert len(doors.doors) == 1
    assert doors.header.frame_id == 'map'
    door = doors.doors[0]
    assert door.center.x == pytest.approx(DOOR_X, abs=0.45)
    assert door.center.y == pytest.approx(WALL_Y, abs=0.30)
    # The crossing direction points through the wall, away from the corridor.
    assert door.normal.y > 0.8

    objects = node.objects_pub.publish.call_args[0][0]
    assert [o.class_id for o in objects.objects] == ['door']

    namespaces = {m.ns for m in node.viz_pub.publish.call_args[0][0].markers}
    assert namespaces == {'viz_doors_semantic'}

    sent = node._tf.sendTransform.call_args[0][0]
    assert [t.child_frame_id for t in sent] == [f'door_{door.id}']
    node.destroy_node()


def test_the_outputs_are_the_SAME_topics_the_other_detectors_use():
    """What makes enter_door work whichever detector found the door."""
    from f1tenth_costmap import semantic_door_markers as sdm
    node = _node()
    assert node.get_parameter('doors_topic').value == sdm.TOPICS[0]
    assert node.get_parameter('viz_doors_topic').value == sdm.TOPICS[1]
    assert node.get_parameter('door_objects_topic').value == sdm.TOPICS[2]
    node.destroy_node()


def test_the_debug_json_carries_the_semantic_detectors_key_names():
    node = _drive(_node(), (0.0,))
    d = _debug(node)
    for key in ('door_ms', 'door_errors', 'door_codes', 'doors_confirmed'):
        assert key in d
    assert d['source'] == 'freespace'
    assert d['method'] == 'scan_jumps'
    assert d['scan_jumps'] > 0
    assert d['door_codes'].get('ok') == 1
    assert d['door_ms'] >= 0.0
    node.destroy_node()


def test_flags_are_counted_in_the_debug_dict_separately_from_codes():
    """A flag is not a rejection, so it must not be filed as one."""
    node = _drive(_node(), (0.0,), grid=False)
    d = _debug(node)
    assert d['door_flags'].get('no_grid') == 1
    assert 'no_grid' not in d['door_codes']
    node.destroy_node()


def test_the_debug_markers_draw_jumps_and_candidates():
    node = _drive(_node(), (0.0,))
    markers = node.debug_viz_pub.publish.call_args[0][0].markers
    assert {m.ns for m in markers} == {'viz_doors_freespace'}
    by_id = {m.id: m for m in markers}
    assert len(by_id[0].points) >= 2          # the raw jumps, yellow
    assert len(by_id[1].points) == 1          # the accepted candidate, orange
    assert by_id[3].points == []              # no skeleton: it is off
    node.destroy_node()


def test_debug_markers_can_be_turned_off():
    node = _node(freespace_doors_debug_markers=False)
    assert node.debug_viz_pub is None
    _drive(node, (0.0,))                      # must not raise
    node.destroy_node()


# ---------------------------------------------------------------------------
# the grid is OPTIONAL and the scan is not
# ---------------------------------------------------------------------------
def test_doors_are_found_before_any_grid_has_arrived():
    """Refusing to see doors until a map exists would make the detector
    useless exactly when a mission is most likely to be starting."""
    node = _drive(_node(), (0.0, 0.3), grid=False)
    doors = node.doors_pub.publish.call_args[0][0]
    assert len(doors.doors) == 1
    assert _debug(node)['door_flags'].get('no_grid')
    node.destroy_node()


def test_no_scan_or_no_pose_means_no_cycle():
    node = _node()
    node._cycle()
    assert node.doors_pub.publish.call_count == 0
    node._scan = _scan_msg(0.0)
    node._cycle()
    assert node.doors_pub.publish.call_count == 0, 'ran without a pose'
    node._pose_cb(_odom(0.0, 0.0))
    node._cycle()
    assert node.doors_pub.publish.call_count == 1
    node.destroy_node()


def test_a_stale_grid_is_skipped_but_the_scan_cycle_still_runs():
    """The grid is LATCHED: an old one keeps arriving forever.

    Without the age check a dead slam_toolbox would go on silently
    cross-checking doors against a map minutes out of date. The cycle is not
    skipped, though -- the scan is the evidence, so the candidate is simply
    produced uncross-checked, flagged `no_grid`.
    """
    node = _node(freespace_doors_grid_max_age_s=5.0)
    _drive(node, (0.0,))
    assert not _debug(node)['door_flags'].get('no_grid')
    node._grid_stamp = node._now_sec() - 60.0
    node._last_cycle_sec = None
    node._scan_cb(_scan_msg(0.0))
    assert _debug(node)['door_flags'].get('no_grid') == 1
    node.destroy_node()


def test_a_missing_laser_transform_stops_the_cycle():
    """Without base_link -> laser the scan cannot be placed in the map at
    all, so there is nothing honest to publish."""
    from tf2_ros import LookupException
    node = _node()
    node.tf_buffer.lookup_transform = MagicMock(side_effect=LookupException('no'))
    node._laser_bl = None
    node._pose_cb(_odom(0.0, 0.0))
    node._scan_cb(_scan_msg(0.0))
    assert node.doors_pub.publish.call_count == 0
    node.destroy_node()


# ---------------------------------------------------------------------------
# the throttle
# ---------------------------------------------------------------------------
def test_the_cycle_is_throttled_to_the_rate():
    """The lidar runs at ~40 Hz and nothing downstream needs doors that often."""
    node = _node(freespace_doors_rate_hz=1.0)
    node._pose_cb(_odom(0.0, 0.0))
    node._map_cb(_grid_msg())
    for _ in range(5):
        node._scan_cb(_scan_msg(0.0))
    assert node.doors_pub.publish.call_count == 1
    node._last_cycle_sec -= 10.0
    node._scan_cb(_scan_msg(0.0))
    assert node.doors_pub.publish.call_count == 2
    node.destroy_node()


# ---------------------------------------------------------------------------
# failure
# ---------------------------------------------------------------------------
def test_a_cycle_that_throws_is_counted_and_does_not_kill_the_node():
    node = _node()
    node._pose_cb(_odom(0.0, 0.0))
    node._scan = _scan_msg(0.0)
    node._detect = MagicMock(side_effect=RuntimeError('boom'))
    for _ in range(3):
        node._cycle()
    assert node._errors == 3
    assert _debug(node)['door_errors'] == 3
    assert node._tracker is not None
    node.destroy_node()


def test_enough_consecutive_failures_turn_the_detector_off_for_good():
    node = _node()
    node._pose_cb(_odom(0.0, 0.0))
    node._scan = _scan_msg(0.0)
    node._detect = MagicMock(side_effect=RuntimeError('boom'))
    for _ in range(FreespaceDoorNode.FREESPACE_MAX_ERRORS):
        node._cycle()
    assert node._tracker is None
    node._detect.reset_mock()
    node._cycle()
    assert node._detect.call_count == 0
    node.destroy_node()


# ---------------------------------------------------------------------------
# the optional medial-axis detector
# ---------------------------------------------------------------------------
def test_use_skeleton_runs_the_other_detector_and_says_so():
    node = _node(freespace_doors_use_skeleton=True)
    assert node.params.use_skeleton is True
    _drive(node, (0.0,))
    d = _debug(node)
    assert d['method'] == 'skeleton'
    assert d['skeleton_cells'] > 0
    node.destroy_node()


# ---------------------------------------------------------------------------
# the pure helpers
# ---------------------------------------------------------------------------
def test_compose_puts_the_laser_where_the_car_is_looking():
    x, y, yaw = compose((1.0, 2.0, math.pi / 2.0), (0.3, 0.0, 0.0))
    assert (x, y) == pytest.approx((1.0, 2.3))
    assert yaw == pytest.approx(math.pi / 2.0)


def test_yaw_from_quaternion_round_trips():
    for yaw in (-2.0, -0.3, 0.0, 1.1, 3.0):
        q = type('Q', (), {'x': 0.0, 'y': 0.0,
                           'z': math.sin(yaw / 2.0), 'w': math.cos(yaw / 2.0)})
        assert yaw_from_quaternion(q) == pytest.approx(yaw)
