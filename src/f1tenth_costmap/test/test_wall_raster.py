"""wall_raster.py: confirmed walls as cells that keep their id and role.

The lifecycle cases (create, update, merge, delete) are tested as what they
actually are here: the raster is rebuilt from the current tracks every cycle,
so each case is "render this track set and look at the cells".
"""

import numpy as np
import pytest

from f1tenth_costmap.wall_raster import (
    CELL_FREE,
    CELL_NO_ID,
    ROLE_VALUE,
    VALUE_ROLE,
    rasterize_walls,
    transform_segment,
    wall_cells,
)

RES = 0.1
W = H = 60
ORIGIN = (-3.0, -3.0)


def raster(walls, thickness_m=0.06):
    return rasterize_walls(walls, ORIGIN[0], ORIGIN[1], RES, W, H, thickness_m)


def wall(wall_id, role, p0, p1):
    return (wall_id, role, p0, p1)


def cell_of(x, y):
    """(row, col) whose CENTRE is nearest (x, y)."""
    return (int(round((y - ORIGIN[1]) / RES - 0.5)), int(round((x - ORIGIN[0]) / RES - 0.5)))


# ---------------------------------------------------------------------------
# What a cell carries
# ---------------------------------------------------------------------------

def test_a_wall_paints_cells_with_its_role_and_its_id():
    roles, ids = raster([wall(7, 'front', (1.0, -1.0), (1.0, 1.0))])
    row, col = cell_of(1.0, 0.0)
    assert roles[row, col] == ROLE_VALUE['front']
    assert ids[row, col] == 7
    assert VALUE_ROLE[roles[row, col]] == 'front'


def test_cells_away_from_any_wall_are_free_and_have_no_id():
    roles, ids = raster([wall(7, 'front', (1.0, -1.0), (1.0, 1.0))])
    row, col = cell_of(-2.0, -2.0)
    assert roles[row, col] == CELL_FREE
    assert ids[row, col] == CELL_NO_ID


def test_every_role_gets_its_own_distinct_value_and_other_only_aliases_unknown():
    current = {role: v for role, v in ROLE_VALUE.items() if role != 'other'}
    assert len(set(current.values())) == len(current)
    assert ROLE_VALUE['other'] == ROLE_VALUE['unknown']
    assert CELL_FREE not in ROLE_VALUE.values()
    # Never 100: these are not occupancy probabilities.
    assert 100 not in ROLE_VALUE.values()


def test_a_side_wall_carries_its_own_role():
    roles, _ = raster([wall(3, 'right', (-1.0, -1.2), (2.0, -1.2))])
    row, col = cell_of(0.0, -1.2)
    assert VALUE_ROLE[roles[row, col]] == 'right'


# ---------------------------------------------------------------------------
# Geometry
# ---------------------------------------------------------------------------

def test_a_wall_does_not_paint_past_its_own_extent():
    """The segment, not the infinite line: a tracked wall's endpoints are what
    was actually observed."""
    _roles, ids = raster([wall(1, 'front', (1.0, -0.5), (1.0, 0.5))])
    assert ids[cell_of(1.0, 0.0)] == 1
    assert ids[cell_of(1.0, 2.0)] == CELL_NO_ID


def test_a_thin_wall_leaves_no_holes_along_its_length():
    """A zero-width line through a cell grid skips cells at the corners, and a
    consumer would read those as a gap in the wall."""
    _roles, ids = raster([wall(1, 'front', (0.37, -1.0), (0.37, 1.0))])
    column = [r for r, _c in wall_cells(ids, 1)]
    assert len(column) >= 18
    assert sorted(column) == list(range(min(column), max(column) + 1))


def test_a_diagonal_wall_is_continuous_too():
    _roles, ids = raster([wall(1, 'front', (-1.0, -1.0), (1.0, 1.0))])
    cells = wall_cells(ids, 1)
    assert len(cells) >= 25
    rows = sorted({r for r, _ in cells})
    assert rows == list(range(rows[0], rows[-1] + 1))


def test_thickness_is_at_least_one_cell_however_thin_it_is_asked_to_be():
    _roles, ids = raster([wall(1, 'front', (0.0, -1.0), (0.0, 1.0))], thickness_m=0.0)
    assert len(wall_cells(ids, 1)) > 0


def test_a_wall_outside_the_grid_paints_nothing():
    roles, ids = raster([wall(1, 'front', (50.0, 0.0), (50.0, 2.0))])
    assert not np.any(roles != CELL_FREE)
    assert not np.any(ids != CELL_NO_ID)


def test_an_empty_grid_is_handled():
    roles, ids = rasterize_walls([wall(1, 'front', (0, 0), (1, 1))], 0.0, 0.0, 0.1, 0, 0)
    assert roles.size == 0 and ids.size == 0


# ---------------------------------------------------------------------------
# The lifecycle, as the raster sees it
# ---------------------------------------------------------------------------

def test_create_a_second_wall_and_both_are_present():
    walls = [wall(1, 'front', (1.5, -1.0), (1.5, 1.0)),
             wall(2, 'right', (-1.0, -1.2), (2.0, -1.2))]
    _roles, ids = raster(walls)
    assert wall_cells(ids, 1) and wall_cells(ids, 2)


def test_update_moves_a_walls_cells_and_leaves_none_behind():
    """The raster is rebuilt every cycle, so a wall that moved has no cells
    where it used to be."""
    _r0, ids0 = raster([wall(1, 'front', (1.0, -1.0), (1.0, 1.0))])
    _r1, ids1 = raster([wall(1, 'front', (2.0, -1.0), (2.0, 1.0))])
    assert ids0[cell_of(1.0, 0.0)] == 1
    assert ids1[cell_of(1.0, 0.0)] == CELL_NO_ID
    assert ids1[cell_of(2.0, 0.0)] == 1


def test_delete_removes_every_cell_of_that_track():
    walls = [wall(1, 'front', (1.5, -1.0), (1.5, 1.0)),
             wall(2, 'right', (-1.0, -1.2), (2.0, -1.2))]
    _r0, ids0 = raster(walls)
    assert wall_cells(ids0, 2)
    _r1, ids1 = raster(walls[:1])          # track 2 deleted by the lifecycle
    assert wall_cells(ids1, 1)
    assert wall_cells(ids1, 2) == []


def test_delete_everything_clears_the_whole_layer():
    roles, ids = raster([])
    assert np.all(roles == CELL_FREE)
    assert np.all(ids == CELL_NO_ID)


def test_merge_leaves_one_id_over_the_shared_surface():
    """Two tracks on the same surface before the merge: the lower id owns the
    cells, so the layer does not flicker while the tracker decides."""
    both = [wall(4, 'front', (1.0, -1.0), (1.0, 0.2)),
            wall(9, 'front', (1.0, -0.2), (1.0, 1.0))]
    _roles, ids = raster(both)
    overlap = ids[cell_of(1.0, 0.0)]
    assert overlap == 4
    # After the merge the survivor covers the whole span, alone.
    _roles2, ids2 = raster([wall(4, 'front', (1.0, -1.0), (1.0, 1.0))])
    assert wall_cells(ids2, 9) == []
    assert len(wall_cells(ids2, 4)) >= len(wall_cells(ids, 4))


def test_a_role_change_repaints_the_same_cells_with_the_new_role():
    seg = ((1.0, -1.0), (1.0, 1.0))
    r0, _ = raster([wall(1, 'front', *seg)])
    r1, _ = raster([wall(1, 'left', *seg)])
    row, col = cell_of(1.0, 0.0)
    assert VALUE_ROLE[r0[row, col]] == 'front'
    assert VALUE_ROLE[r1[row, col]] == 'left'


# ---------------------------------------------------------------------------
# Frames
# ---------------------------------------------------------------------------

def test_transform_segment_moves_base_link_into_the_map():
    p0, p1 = transform_segment((1.0, 0.0), (1.0, 1.0), (10.0, 5.0, np.pi / 2))
    assert p0[0] == pytest.approx(10.0, abs=1e-6)
    assert p0[1] == pytest.approx(6.0, abs=1e-6)
    assert p1[0] == pytest.approx(9.0, abs=1e-6)
    assert p1[1] == pytest.approx(6.0, abs=1e-6)


def test_transform_segment_at_the_identity_changes_nothing():
    p0, p1 = transform_segment((1.0, 2.0), (3.0, 4.0), (0.0, 0.0, 0.0))
    assert p0 == pytest.approx((1.0, 2.0))
    assert p1 == pytest.approx((3.0, 4.0))


# ---------------------------------------------------------------------------
# The node glue: only CONFIRMED walls become cells
# ---------------------------------------------------------------------------

class TestTheWallLayerNode:

    @staticmethod
    def _track(track_id, role, p0, p1, state=None):
        from f1tenth_messages.msg import MultiWallTrack
        from geometry_msgs.msg import Point
        t = MultiWallTrack()
        t.id = track_id
        t.role = t.label = role
        t.state = MultiWallTrack.STATE_CONFIRMED if state is None else state
        t.p0_bl = Point(x=float(p0[0]), y=float(p0[1]))
        t.p1_bl = Point(x=float(p1[0]), y=float(p1[1]))
        return t

    def _node(self):
        import rclpy
        from f1tenth_costmap.semantic_layer_node import SemanticLayerNode
        if not rclpy.ok():
            rclpy.init()
        node = SemanticLayerNode()
        node.wall_cells_pub.publish = _Capture()
        node.wall_ids_pub.publish = _Capture()
        node._pose_history = [(10.0, (0.0, 0.0, 0.0))]
        return node

    @staticmethod
    def _msg(tracks, stamp_sec=10):
        from f1tenth_messages.msg import MultiWallTrackArray
        msg = MultiWallTrackArray()
        msg.header.stamp.sec = stamp_sec
        msg.tracks = tracks
        return msg

    def test_a_confirmed_wall_becomes_cells_with_its_role(self):
        from f1tenth_costmap.wall_raster import ROLE_VALUE
        node = self._node()
        try:
            node._walls_cb(self._msg([self._track(1, 'front', (1.0, -1.0), (1.0, 1.0))]))
            grid = node.wall_cells_pub.publish.last
            assert grid is not None
            assert grid.header.frame_id == 'map'
            assert ROLE_VALUE['front'] in grid.data
            ids = node.wall_ids_pub.publish.last
            assert 1 in ids.data and len(ids.data) == len(grid.data)
        finally:
            node.destroy_node()

    def test_a_tentative_wall_is_not_a_fact_and_gets_no_cells(self):
        from f1tenth_messages.msg import MultiWallTrack
        node = self._node()
        try:
            node._walls_cb(self._msg([self._track(
                1, 'front', (1.0, -1.0), (1.0, 1.0), MultiWallTrack.STATE_TENTATIVE)]))
            assert set(node.wall_cells_pub.publish.last.data) == {0}
        finally:
            node.destroy_node()

    def test_a_deleted_track_leaves_no_cells_on_the_next_message(self):
        node = self._node()
        try:
            node._walls_cb(self._msg([self._track(1, 'front', (1.0, -1.0), (1.0, 1.0)),
                                      self._track(2, 'right', (-1.0, -1.2), (2.0, -1.2))]))
            assert 2 in node.wall_ids_pub.publish.last.data
            node._walls_cb(self._msg([self._track(1, 'front', (1.0, -1.0), (1.0, 1.0))]))
            assert 2 not in node.wall_ids_pub.publish.last.data
            assert 1 in node.wall_ids_pub.publish.last.data
        finally:
            node.destroy_node()

    def test_only_the_newest_message_is_rasterised_at_the_layer_rate(self):
        # cpu-cleanup: messages are only kept; the timer rasterises the newest.
        node = self._node()
        try:
            assert node.wall_timer.timer_period_ns == round(1e9 / 5.0)
            node._walls_msg_cb(self._msg([self._track(1, 'front', (1.0, -1.0), (1.0, 1.0))]))
            node._walls_msg_cb(self._msg([self._track(2, 'right', (-1.0, -1.2), (2.0, -1.2))]))
            assert node.wall_ids_pub.publish.last is None      # no work per message
            node._wall_layer_tick()
            ids = node.wall_ids_pub.publish.last
            assert 2 in ids.data and 1 not in ids.data
            node.wall_ids_pub.publish.last = None
            node._wall_layer_tick()                              # nothing new: nothing out
            assert node.wall_ids_pub.publish.last is None
        finally:
            node.destroy_node()

    def test_the_layer_carries_the_same_values_as_the_raster(self):
        import numpy as np
        from f1tenth_costmap.wall_raster import rasterize_walls
        node = self._node()
        try:
            node._walls_cb(self._msg([self._track(1, 'front', (1.0, -1.0), (1.0, 1.0))]))
            grid = node.wall_cells_pub.publish.last
            ids = node.wall_ids_pub.publish.last
            cells = grid.info.width
            roles, want_ids = rasterize_walls(
                [(1, 'front', (1.0, -1.0), (1.0, 1.0))], grid.info.origin.position.x,
                grid.info.origin.position.y, grid.info.resolution, cells, cells,
                node.wall_thickness_m)
            assert list(grid.data) == roles.ravel().astype(np.int8).tolist()
            assert list(ids.data) == want_ids.ravel().tolist()
        finally:
            node.destroy_node()

    def test_tf_comes_from_tf_static_only_by_default(self):
        from geometry_msgs.msg import TransformStamped
        from tf2_msgs.msg import TFMessage
        node = self._node()
        try:
            assert node.tf_listener is None and node.tf_static_sub is not None
            t = TransformStamped()
            t.header.frame_id = 'base_link'
            t.child_frame_id = 'zed2_left_camera_frame'
            t.transform.translation.x = 0.3
            t.transform.rotation.w = 1.0
            node._tf_static_cb(TFMessage(transforms=[t]))
            import rclpy.time
            got = node.tf_buffer.lookup_transform(
                'base_link', 'zed2_left_camera_frame', rclpy.time.Time())
            assert got.transform.translation.x == 0.3
        finally:
            node.destroy_node()

    def test_nothing_is_published_without_a_pose_to_place_it_with(self):
        node = self._node()
        try:
            node._pose_history = []
            node._walls_cb(self._msg([self._track(1, 'front', (1.0, -1.0), (1.0, 1.0))]))
            assert node.wall_cells_pub.publish.last is None
        finally:
            node.destroy_node()


class _Capture:
    def __init__(self):
        self.last = None

    def __call__(self, msg):
        self.last = msg


def test_unknown_is_its_own_cell_value_and_not_back():
    """A wall the tracker uses as a reference for nothing is still a surface,
    and must not be filed as the wall BEHIND the car."""
    from f1tenth_costmap.wall_raster import ROLE_VALUE, VALUE_ROLE, rasterize_walls
    assert ROLE_VALUE['unknown'] == 50
    assert VALUE_ROLE[50] == 'unknown'
    roles, ids = rasterize_walls([(1, 'unknown', (0.0, 0.0), (1.0, 0.0))],
                                 -0.5, -0.5, 0.05, 40, 40)
    assert (roles == ROLE_VALUE['unknown']).any()
    assert not (roles == ROLE_VALUE['back']).any()
    assert (ids == 1).any()


def test_unknown_and_the_legacy_other_paint_the_same_cells_with_the_same_value():
    # 'other' is the role's name before 2026-10-19: a bag or a node that still
    # says it must paint exactly what 'unknown' paints, and read back as it.
    from f1tenth_costmap.wall_raster import VALUE_ROLE, rasterize_walls
    seg = ((0.0, 0.0), (1.0, 0.0))
    r_unknown, ids_unknown = rasterize_walls([(1, 'unknown', *seg)], -0.5, -0.5, 0.05, 40, 40)
    r_other, ids_other = rasterize_walls([(1, 'other', *seg)], -0.5, -0.5, 0.05, 40, 40)
    assert (r_unknown == 50).any()
    assert np.array_equal(r_other, r_unknown) and np.array_equal(ids_other, ids_unknown)
    assert VALUE_ROLE[50] == 'unknown'


def test_a_role_this_module_has_never_heard_of_is_unknown():
    from f1tenth_costmap.wall_raster import ROLE_VALUE, rasterize_walls
    roles, _ids = rasterize_walls([(7, 'spaceship', (0.0, 0.0), (1.0, 0.0))],
                                  -0.5, -0.5, 0.05, 40, 40)
    assert (roles == ROLE_VALUE['unknown']).any()
    assert not (roles == ROLE_VALUE['back']).any()
