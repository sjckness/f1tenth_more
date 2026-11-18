"""semantic_layer_node's DOOR wiring -- the glue, not the geometry.

semantic_doors.py's own tests cover what a door is. These cover what the node
does with one: the two gates that decide whether the detector exists at all,
the walls it hands it, the four outputs, and the guarantee that a door cycle
which throws leaves the semantic layer untouched.

Same convention as test_semantic_layer_node.py: a real, never-spun rclpy node
built through parameter_overrides, publishers mocked, callbacks called
directly.
"""

import math
from unittest.mock import MagicMock

import pytest
import rclpy
from geometry_msgs.msg import Transform, TransformStamped
from nav_msgs.msg import Odometry
from rclpy.parameter import Parameter
from sensor_msgs.msg import LaserScan

from f1tenth_messages.msg import MultiWallTrack, MultiWallTrackArray
from f1tenth_costmap.semantic_layer_node import SemanticLayerNode


@pytest.fixture(scope='module', autouse=True)
def _rclpy_context():
    rclpy.init()
    yield
    if rclpy.ok():
        rclpy.shutdown()


# The same room test_semantic_doors.py uses, in base_link: a wall along
# y = 3 m with a 0.90 m opening, the car at the origin looking down +x... with
# one difference that matters for the node -- the LiDAR here is at base_link's
# origin, because the node reads the real laser offset from /tf_static and the
# stub transform below is the identity.
GAP = 0.90
LEFT = ((-3.0, 3.0), (-GAP / 2.0, 3.0))
RIGHT = ((GAP / 2.0, 3.0), (3.0, 3.0))


def _identity_transform_stamped():
    ts = TransformStamped()
    ts.transform = Transform()
    ts.transform.rotation.w = 1.0
    return ts


def _node(**params):
    base = {'semantic_doors_enabled': True, 'door_source': 'semantic',
            'unknown_objects_mode': 'off'}
    base.update(params)
    node = SemanticLayerNode(
        parameter_overrides=[Parameter(k, value=v) for k, v in base.items()])
    node.marker_pub.publish = MagicMock()
    node.tf_buffer.lookup_transform = MagicMock(
        return_value=_identity_transform_stamped())
    for name in ('doors_pub', 'viz_doors_pub', 'door_objects_pub'):
        if hasattr(node, name):
            setattr(node, name, MagicMock())
    if getattr(node, '_door_tf', None) is not None:
        node._door_tf = MagicMock()
    return node


def _walls(*segments, state=MultiWallTrack.STATE_CONFIRMED,
           source=MultiWallTrack.SOURCE_LIDAR):
    msg = MultiWallTrackArray()
    msg.header.frame_id = 'odom'
    for i, (p0, p1) in enumerate(segments):
        t = MultiWallTrack()
        t.id = i + 1
        t.state = state
        t.source = source
        t.role = 'left'
        t.p0_bl.x, t.p0_bl.y = float(p0[0]), float(p0[1])
        t.p1_bl.x, t.p1_bl.y = float(p1[0]), float(p1[1])
        msg.tracks.append(t)
    return msg


def _ray(origin, direction, a, b):
    ox, oy = origin
    dx, dy = direction
    ex, ey = b[0] - a[0], b[1] - a[1]
    den = dx * ey - dy * ex
    if abs(den) < 1e-12:
        return math.inf
    qx, qy = a[0] - ox, a[1] - oy
    t = (qx * ey - qy * ex) / den
    u = (qx * dy - qy * dx) / den
    return t if t > 0.0 and 0.0 <= u <= 1.0 else math.inf


def _scan(segments, n=721, half_fov=math.radians(120.0), range_max=25.0):
    msg = LaserScan()
    msg.header.frame_id = 'laser'
    msg.angle_min = -half_fov
    msg.angle_max = half_fov
    msg.angle_increment = (2.0 * half_fov) / (n - 1)
    msg.range_min = 0.05
    msg.range_max = range_max
    ranges = []
    for i in range(n):
        th = msg.angle_min + i * msg.angle_increment
        d = (math.cos(th), math.sin(th))
        r = min((_ray((0.0, 0.0), d, a, b) for a, b in segments), default=math.inf)
        ranges.append(float(r) if r <= range_max else float('inf'))
    msg.ranges = ranges
    return msg


def _drive_cycles(node, n, scan=None, walls=None):
    """Run n of the node's OWN cycles over the room, the car at the origin.

    Through _cycle(), not _door_step() directly, because the thing most
    likely to be wrong about this wiring is whether the door cycle is reached
    at all -- it is not reached through the unknown-object path, which ships
    'off' (unknown_objects_mode), and these tests run with it off for exactly
    that reason.
    """
    scan = scan if scan is not None else _scan([LEFT, RIGHT])
    walls = walls if walls is not None else _walls(LEFT, RIGHT)
    node._laser_pose = ('laser', (0.0, 0.0, 0.0))
    node._walls_msg_cb(walls)
    for i in range(n):
        stamp = 100.0 + i * 0.1
        pose = Odometry()
        pose.header.stamp.sec = int(stamp)
        pose.header.stamp.nanosec = int(round((stamp - int(stamp)) * 1e9))
        pose.pose.pose.orientation.w = 1.0
        node._pose_cb(pose)
        scan.header.stamp.sec = int(stamp)
        scan.header.stamp.nanosec = int(round((stamp - int(stamp)) * 1e9))
        node._scan_cb(scan)
        node._cycle()
    return node


# ---------------------------------------------------------------------------
# The two gates
# ---------------------------------------------------------------------------
def test_nothing_is_created_when_the_detector_is_off():
    node = _node(semantic_doors_enabled=False)
    try:
        assert node.doors_enabled is False
        assert node._door_tracker is None
        for name in ('doors_pub', 'viz_doors_pub', 'door_objects_pub'):
            assert not hasattr(node, name)
    finally:
        node.destroy_node()


def test_door_source_profile_leaves_this_detector_off():
    """The other detector owns the doors; this node must not also publish."""
    node = _node(door_source='profile')
    try:
        assert node.doors_enabled is False
        assert node._door_tracker is None
        assert not hasattr(node, 'doors_pub')
    finally:
        node.destroy_node()


def test_an_unknown_door_source_is_refused_at_startup():
    with pytest.raises(ValueError, match='door_source'):
        _node(door_source='profiel')


def test_an_unknown_wall_level_is_refused_at_startup():
    with pytest.raises(ValueError, match='min_wall_level'):
        _node(semantic_doors_min_wall_level='very_confirmed')


# ---------------------------------------------------------------------------
# The walls the node hands the detector
# ---------------------------------------------------------------------------
def test_tentative_tracks_reach_the_detector_with_their_level():
    node = _node()
    try:
        node._walls_msg_cb(_walls(LEFT, RIGHT, state=MultiWallTrack.STATE_TENTATIVE))
        walls = node._door_walls_map((0.0, 0.0, 0.0))
        assert [w.level for w in walls] == [0, 0]       # LEVEL_TENTATIVE
        node._walls_msg_cb(_walls(LEFT, RIGHT))
        assert [w.level for w in node._door_walls_map((0.0, 0.0, 0.0))] == [2, 2]
    finally:
        node.destroy_node()


def test_the_walls_are_moved_into_the_map_frame():
    """base_link p0/p1 through the car's pose -- the semantic layer's own hop."""
    node = _node()
    try:
        node._walls_msg_cb(_walls(LEFT, RIGHT))
        walls = node._door_walls_map((10.0, 0.0, math.pi / 2.0))
        # (-3, 3) in base_link, car at (10, 0) facing +y -> (10 - 3, -3)
        assert walls[0].p0 == pytest.approx((7.0, -3.0), abs=1e-6)
    finally:
        node.destroy_node()


# ---------------------------------------------------------------------------
# The four outputs
# ---------------------------------------------------------------------------
def test_a_confirmed_door_reaches_all_four_outputs():
    node = _node()
    try:
        _drive_cycles(node, node.door_params.confirm_hits)
        doors = node._door_tracker.confirmed()
        assert len(doors) == 1
        door = doors[0]

        msg = node.doors_pub.publish.call_args[0][0]
        assert msg.header.frame_id == 'map'
        assert [d.id for d in msg.doors] == [door.id]
        assert msg.doors[0].width == pytest.approx(GAP, abs=0.05)
        assert msg.doors[0].state == 1                     # STATE_CONFIRMED
        # The crossing direction points away from the car that confirmed it.
        assert msg.doors[0].normal.y > 0.9

        objects = node.door_objects_pub.publish.call_args[0][0]
        assert [o.class_id for o in objects.objects] == ['door']
        assert objects.objects[0].radius_m == pytest.approx(GAP / 2.0, abs=0.03)

        markers = node.viz_doors_pub.publish.call_args[0][0]
        assert len(markers.markers) == 5                   # jamb, jamb, line, arrow, text
        assert {m.ns for m in markers.markers} == {'viz_doors_semantic'}

        tfs = node._door_tf.sendTransform.call_args[0][0]
        assert [t.child_frame_id for t in tfs] == [f'door_{door.id}']
        assert tfs[0].header.frame_id == 'map'
    finally:
        node.destroy_node()


def test_a_candidate_is_drawn_but_published_nowhere():
    node = _node()
    try:
        _drive_cycles(node, node.door_params.confirm_hits - 1)
        assert node._door_tracker.confirmed() == []
        assert node.doors_pub.publish.call_args[0][0].doors == []
        assert node.door_objects_pub.publish.call_args[0][0].objects == []
        # ...but /viz draws it, faded.
        markers = node.viz_doors_pub.publish.call_args[0][0]
        assert len(markers.markers) == 5
        assert markers.markers[0].color.a < 0.5
        node._door_tf.sendTransform.assert_not_called()
    finally:
        node.destroy_node()


def test_the_debug_payload_carries_the_door_counters():
    import json
    node = _node()
    try:
        node.unknown_debug_pub.publish = MagicMock()
        _drive_cycles(node, node.door_params.confirm_hits)
        payload = json.loads(node.unknown_debug_pub.publish.call_args[0][0].data)
        assert payload['doors_confirmed'] == 1
        assert payload['door_errors'] == 0
        assert payload['door_codes']['ok'] >= 1
        assert payload['door_ms'] >= 0.0
    finally:
        node.destroy_node()


# ---------------------------------------------------------------------------
# The overlay rule: a door cycle must never take the semantic layer down
# ---------------------------------------------------------------------------
def test_a_failing_door_cycle_is_counted_and_does_not_raise():
    node = _node()
    try:
        node._door_walls_map = MagicMock(side_effect=RuntimeError('boom'))
        _drive_cycles(node, 3)
        assert node._door_errors == 3
        assert node.doors_enabled is True
    finally:
        node.destroy_node()


def test_fifty_failures_in_a_row_turn_the_detector_off_for_good():
    node = _node()
    try:
        node._door_walls_map = MagicMock(side_effect=RuntimeError('boom'))
        _drive_cycles(node, node.DOOR_MAX_ERRORS)
        assert node._door_errors == node.DOOR_MAX_ERRORS
        assert node.doors_enabled is False
        calls = node.doors_pub.publish.call_count
        _drive_cycles(node, 5)
        assert node.doors_pub.publish.call_count == calls   # nothing more
    finally:
        node.destroy_node()


def test_a_good_cycle_resets_the_failure_streak():
    node = _node()
    try:
        real = node._door_walls_map
        node._door_walls_map = MagicMock(side_effect=RuntimeError('boom'))
        _drive_cycles(node, 3)
        assert node._door_errors == 3
        node._door_walls_map = real
        _drive_cycles(node, 1)
        assert node._door_errors == 0
    finally:
        node.destroy_node()


@pytest.mark.parametrize('mode', ['off', 'shadow'])
def test_doors_run_whatever_unknown_objects_mode_says(mode):
    """The regression this file exists for, named.

    The door cycle hung off _lidar_step, which runs only while
    unknown_objects_mode is 'shadow' or 'active'. That parameter ships 'off',
    so a car with semantic_doors_enabled true would have published nothing
    and said nothing about why.
    """
    node = _node(unknown_objects_mode=mode)
    try:
        _drive_cycles(node, node.door_params.confirm_hits)
        assert len(node._door_tracker.confirmed()) == 1
        assert node.doors_pub.publish.call_count >= node.door_params.confirm_hits
    finally:
        node.destroy_node()
