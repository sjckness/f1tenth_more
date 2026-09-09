"""costmap_boundary_node.py's convex-safe-corridor path.

Same "construct a real but never-spun rclpy Node and call its callbacks/
_extraction_tick() directly, with the publishers mocked" convention
test_costmap_boundary_node.py's own module docstring establishes -- see
that file for the reasoning. The pure geometry underneath is covered by
test_safe_corridor.py; this file covers the WIRING, which is where the
frames and the flag live.

WHAT THIS PINS:

  1. THE FLAG IS OFF BY DEFAULT and off means off -- with
     use_convex_polytope unset, the node publishes exactly the three
     nearest-cell constraints it published before, no corridor
     subscription is consulted, and no TF is required. That is the
     "build only, do NOT enable" requirement, checked rather than
     asserted in a comment.

  2. front_clearance IS INDEPENDENT OF THE FLAG. It feeds a live
     mission-stopping behaviour-tree condition, so it is published from
     the nearest-cell extraction in both modes.

  3. THE FRAME CHAIN. The corridor arrives in odom, the grid is map, the
     output is base_link. Each leg is checked on its own, and the
     round-trip is checked end to end, because a silent frame error here
     produces constraints that look entirely plausible and are wrong by
     the whole map -> odom offset.

  4. THE CORRIDOR STAMP IS CARRIED ONTO THE CONSTRAINTS, so a consumer can
     detect a generation mismatch.

  5. DEGENERACY PUBLISHES NOTHING AND SAYS WHY, rather than shipping a
     shrunken polytope.

Run standalone: python3 -m pytest test/test_safe_corridor_node.py -v
"""

import json
import math
from types import SimpleNamespace
from unittest.mock import MagicMock

import numpy as np
import pytest
import rclpy
from builtin_interfaces.msg import Time as TimeMsg
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.parameter import Parameter

from f1tenth_costmap.costmap_boundary_node import (
    CostmapBoundaryNode,
    centreline_from_marker_array,
    transform_points_odom_to_map,
    trim_centreline_ahead,
)

_OCC = 100
_FREE = 0
_THRESH = 65

# Grid: 12 m x 12 m at 5 cm, origin (-4, -4), so map coordinates in
# [-4, 8] are addressable and the action sits well clear of every edge --
# out-of-bounds cells block in the polytope path (deliberately, see
# safe_corridor.py), so a fixture that crowds the edge measures the map
# border rather than the geometry under test.
_RES = 0.05
_ORIGIN = -4.0
_EXTENT = 12.0
_W = _H = int(_EXTENT / _RES)


@pytest.fixture(scope='module', autouse=True)
def _rclpy_context():
    rclpy.init()
    yield
    if rclpy.ok():
        rclpy.shutdown()


def _corridor_between_walls(wall_y=(-1.0, 1.0)):
    """Straight map-frame corridor along y = 0, walls 2 m apart."""
    grid = np.full((_H, _W), _FREE, dtype=np.int16)
    for y in wall_y:
        grid[int((y - _ORIGIN) / _RES), :] = _OCC
    return grid.flatten().tolist()


def _grid_msg(data):
    msg = OccupancyGrid()
    msg.info.width = _W
    msg.info.height = _H
    msg.info.resolution = _RES
    msg.info.origin.position.x = _ORIGIN
    msg.info.origin.position.y = _ORIGIN
    msg.data = data
    return msg


def _odom_msg(x, y, yaw):
    msg = Odometry()
    msg.pose.pose.position.x = x
    msg.pose.pose.position.y = y
    msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
    msg.pose.pose.orientation.w = math.cos(yaw / 2.0)
    return msg


def _marker_array(points, ns='corridor_centerline', stamp=None):
    """Duck-typed MarkerArray -- centreline_from_marker_array only touches
    .markers / .ns / .points / .header.stamp."""
    stamp = stamp if stamp is not None else TimeMsg(sec=17, nanosec=250)
    marker = SimpleNamespace(
        ns=ns,
        header=SimpleNamespace(stamp=stamp, frame_id='odom'),
        points=[SimpleNamespace(x=float(px), y=float(py), z=0.0) for px, py in points],
    )
    # A real /mpc/corridor_markers carries the two wall polylines too; they
    # must be ignored, so the fixture includes one.
    wall = SimpleNamespace(
        ns='corridor_left',
        header=SimpleNamespace(stamp=stamp, frame_id='odom'),
        points=[SimpleNamespace(x=0.0, y=9.0, z=0.0)],
    )
    return SimpleNamespace(markers=[wall, marker])


def _identity_tf(node):
    """map -> odom = identity, so odom coordinates ARE map coordinates.

    Used by every test that is not specifically about the transform, so a
    fixture's map-frame numbers can be read straight off the corridor it
    hands in.
    """
    node._map_from_odom = lambda: (0.0, 0.0, 0.0)


def _node(**params):
    overrides = [Parameter(k, value=v) for k, v in params.items()]
    node = CostmapBoundaryNode(parameter_overrides=overrides)
    node.boundary_pub.publish = MagicMock()
    node.clearance_pub.publish = MagicMock()
    node.report_pub.publish = MagicMock()
    return node


def _armed(node, corridor_points=None, robot=(0.0, 0.0, 0.0)):
    """Feed the node one grid, one pose and (optionally) one corridor."""
    node._map_cb(_grid_msg(_corridor_between_walls()))
    node._pose_cb(_odom_msg(*robot))
    if corridor_points is not None:
        node._corridor_cb(_marker_array(corridor_points))
    return node


def _straight_corridor_points(x0=0.0, x1=2.5, n=40):
    return [(x, 0.0) for x in np.linspace(x0, x1, n)]


# =====================================================================
# 1. The flag is off by default, and off means off.
# =====================================================================
class TestBuiltButNotEnabled:

    def test_the_parameter_defaults_to_false(self):
        node = _node()
        try:
            assert node.get_parameter('use_convex_polytope').value is False
            assert node.use_convex_polytope is False
        finally:
            node.destroy_node()

    def test_with_the_flag_off_the_output_is_the_old_nearest_cell_set(self):
        """The default path must be untouched by this whole change."""
        node = _armed(_node(), _straight_corridor_points())
        try:
            # A corridor HAS arrived and TF would work; neither matters.
            _identity_tf(node)
            node._extraction_tick()
            arr = node.boundary_pub.publish.call_args[0][0]
            # THREE, the nearest-cell method's own maximum: one per
            # front/left/right window. The front cone finds a wall too --
            # its half-angle is 35 deg and the walls are 1 m off the
            # centreline, so the +y wall enters the cone at x = 1.43 m,
            # inside the 5 m search radius. That is the old method's
            # ceiling and is exactly what the polytope path lifts.
            assert len(arr.constraints) == 3
            # Stamped with the node's own clock, not a corridor stamp.
            assert (arr.header.stamp.sec, arr.header.stamp.nanosec) != (17, 250)
            node.report_pub.publish.assert_not_called()
        finally:
            node.destroy_node()

    def test_with_the_flag_off_no_corridor_and_no_tf_are_needed(self):
        node = _node()
        node._map_cb(_grid_msg(_corridor_between_walls()))
        node._pose_cb(_odom_msg(0.0, 0.0, 0.0))
        try:
            # _map_from_odom would raise if consulted -- it must not be.
            node._map_from_odom = MagicMock(side_effect=AssertionError('TF consulted'))
            node._extraction_tick()
            node.boundary_pub.publish.assert_called_once()
            node._map_from_odom.assert_not_called()
        finally:
            node.destroy_node()


# =====================================================================
# 2. front_clearance never moves behind the flag.
# =====================================================================
class TestFrontClearanceIsIndependentOfTheFlag:
    """It feeds condition_eval.py's front_clearance stop condition -- a
    live mission-stopping path that must not depend on an experimental
    flag being on or off, or on the polytope path succeeding."""

    def _clearance(self, **params):
        node = _armed(_node(**params), _straight_corridor_points())
        try:
            _identity_tf(node)
            node._extraction_tick()
            return float(node.clearance_pub.publish.call_args[0][0].data)
        finally:
            node.destroy_node()

    def test_the_same_value_in_both_modes(self):
        off = self._clearance()
        on = self._clearance(use_convex_polytope=True)
        assert off == on

    def test_it_is_still_published_when_the_polytope_path_bails_out(self):
        """No corridor received: the polytope path cannot run, but the
        clearance must arrive anyway."""
        node = _node(use_convex_polytope=True)
        node._map_cb(_grid_msg(_corridor_between_walls()))
        node._pose_cb(_odom_msg(0.0, 0.0, 0.0))
        try:
            node._extraction_tick()
            node.clearance_pub.publish.assert_called_once()
            # ...and it fell back to the nearest-cell constraints (three,
            # see TestBuiltButNotEnabled) rather than publishing nothing.
            assert len(node.boundary_pub.publish.call_args[0][0].constraints) == 3
        finally:
            node.destroy_node()


# =====================================================================
# 3. The polytope path itself.
# =====================================================================
class TestPolytopePath:

    def test_it_publishes_more_than_three_faces_which_is_the_whole_point(self):
        node = _armed(_node(use_convex_polytope=True), _straight_corridor_points())
        try:
            _identity_tf(node)
            node._extraction_tick()
            arr = node.boundary_pub.publish.call_args[0][0]
            assert len(arr.constraints) > 3
            assert arr.header.frame_id == 'base_link'
        finally:
            node.destroy_node()

    def test_the_published_faces_contain_the_reference(self):
        """The property the change exists to provide, checked on the wire.

        The robot sits at the origin facing +x with the corridor running
        straight ahead, so base_link coordinates and map coordinates
        coincide and the centreline points can be tested against the
        published constraints directly.
        """
        points = _straight_corridor_points()
        node = _armed(_node(use_convex_polytope=True), points)
        try:
            _identity_tf(node)
            node._extraction_tick()
            arr = node.boundary_pub.publish.call_args[0][0]
            assert arr.constraints
            for px, py in points:
                for c in arr.constraints:
                    lhs = c.normal[0] * px + c.normal[1] * py
                    assert lhs <= c.offset + 1e-9
        finally:
            node.destroy_node()

    def test_both_walls_are_represented(self):
        node = _armed(_node(use_convex_polytope=True), _straight_corridor_points())
        try:
            _identity_tf(node)
            node._extraction_tick()
            arr = node.boundary_pub.publish.call_args[0][0]
            normals = np.array([[c.normal[0], c.normal[1]] for c in arr.constraints])
            assert np.any(normals[:, 1] > 0.9), 'no face on the +y wall'
            assert np.any(normals[:, 1] < -0.9), 'no face on the -y wall'
        finally:
            node.destroy_node()

    def test_a_report_is_published_every_tick(self):
        node = _armed(_node(use_convex_polytope=True), _straight_corridor_points())
        try:
            _identity_tf(node)
            node._extraction_tick()
            report = json.loads(node.report_pub.publish.call_args[0][0].data)
            for key in ('n_faces', 'cap_hit', 'covered_fraction', 'seed_clearance',
                        'area', 'degenerate', 'unknown_fraction', 'oob_fraction'):
                assert key in report
            assert report['degenerate'] is False
        finally:
            node.destroy_node()

    def test_missing_tf_falls_back_instead_of_going_quiet(self):
        node = _armed(_node(use_convex_polytope=True), _straight_corridor_points())
        try:
            node._map_from_odom = lambda: None
            node._extraction_tick()
            # Fell back to nearest-cell rather than publishing nothing.
            assert len(node.boundary_pub.publish.call_args[0][0].constraints) == 3
            node.report_pub.publish.assert_not_called()
        finally:
            node.destroy_node()


# =====================================================================
# 4. The corridor stamp is carried, so a mismatch is detectable.
# =====================================================================
class TestGenerationStamp:

    def test_the_constraints_carry_the_corridor_stamp_not_the_clock(self):
        """Consistency with the corridor beats freshness -- constraints
        derived from a corridor the MPC has already replaced are unsafe
        however recent they are, so the stamp has to identify the
        GENERATION, not the publish instant."""
        node = _armed(_node(use_convex_polytope=True), _straight_corridor_points())
        try:
            _identity_tf(node)
            node._extraction_tick()
            arr = node.boundary_pub.publish.call_args[0][0]
            assert (arr.header.stamp.sec, arr.header.stamp.nanosec) == (17, 250)
        finally:
            node.destroy_node()

    def test_a_new_corridor_generation_changes_the_stamp(self):
        node = _armed(_node(use_convex_polytope=True), _straight_corridor_points())
        try:
            _identity_tf(node)
            node._corridor_cb(_marker_array(
                _straight_corridor_points(), stamp=TimeMsg(sec=99, nanosec=1)))
            node._extraction_tick()
            arr = node.boundary_pub.publish.call_args[0][0]
            assert (arr.header.stamp.sec, arr.header.stamp.nanosec) == (99, 1)
        finally:
            node.destroy_node()


# =====================================================================
# 5. Degeneracy publishes nothing and says why.
# =====================================================================
class TestDegeneracyIsRefusable:

    def test_a_boxed_in_polytope_publishes_no_constraints_but_does_report(self):
        """Not "shrink quietly and ship it": an empty array plus a report
        naming the failing tests, so the consumer refuses rather than
        squeezing into a box carved out of unexplored cells."""
        node = _armed(
            _node(use_convex_polytope=True, polytope_min_seed_clearance_m=5.0),
            _straight_corridor_points())
        try:
            _identity_tf(node)
            node._extraction_tick()
            arr = node.boundary_pub.publish.call_args[0][0]
            assert len(arr.constraints) == 0
            report = json.loads(node.report_pub.publish.call_args[0][0].data)
            assert report['degenerate'] is True
            assert any('seed_clearance' in r for r in report['degenerate_reasons'])
            # Still stamped with the corridor generation it came from.
            assert (arr.header.stamp.sec, arr.header.stamp.nanosec) == (17, 250)
        finally:
            node.destroy_node()


# =====================================================================
# 6. The frame chain: odom corridor -> map grid -> base_link output.
# =====================================================================
class TestFrameChain:

    def test_the_corridor_is_transformed_out_of_odom_into_map(self):
        """A non-identity map -> odom edge must actually move the seed.

        With the edge applied, an odom-frame corridor along y = 0 lands on
        the map-frame corridor at y = +2; the walls in this fixture are at
        map y = -1 and +1, so a corridor that was NOT transformed would be
        seeded between them and come back clean, while a transformed one
        is seeded on top of the +1 wall. The two are therefore trivially
        distinguishable, which is the point of the fixture.
        """
        pts = transform_points_odom_to_map(
            [(0.0, 0.0), (1.0, 0.0)], tf_x=0.0, tf_y=2.0, tf_yaw=0.0)
        assert pts[0] == pytest.approx([0.0, 2.0])
        assert pts[1] == pytest.approx([1.0, 2.0])

    def test_a_rotated_edge_rotates_the_corridor(self):
        pts = transform_points_odom_to_map(
            [(1.0, 0.0)], tf_x=0.0, tf_y=0.0, tf_yaw=math.radians(90.0))
        assert pts[0] == pytest.approx([0.0, 1.0], abs=1e-9)

    def test_the_output_is_car_frame_so_a_yawed_robot_sees_rotated_normals(self):
        """The robot yawed 90 deg reads the same walls on its own left and
        right, i.e. the face normals must come back rotated into base_link
        rather than left in map."""
        # Corridor along map +y (the robot's own +x once yawed 90 deg).
        corridor = [(0.0, y) for y in np.linspace(0.0, 2.0, 40)]
        grid = np.full((_H, _W), _FREE, dtype=np.int16)
        for x in (-1.0, 1.0):
            grid[:, int((x - _ORIGIN) / _RES)] = _OCC

        node = _node(use_convex_polytope=True)
        node._map_cb(_grid_msg(grid.flatten().tolist()))
        node._pose_cb(_odom_msg(0.0, 0.0, math.radians(90.0)))
        node._corridor_cb(_marker_array(corridor))
        try:
            _identity_tf(node)
            node._extraction_tick()
            arr = node.boundary_pub.publish.call_args[0][0]
            normals = np.array([[c.normal[0], c.normal[1]] for c in arr.constraints])
            # Map-frame walls face +/-x; in the car frame they must face
            # +/-y, i.e. the car's own left and right.
            assert np.any(normals[:, 1] > 0.9)
            assert np.any(normals[:, 1] < -0.9)
            assert np.all(np.abs(normals[:, 0]) < 0.5)
        finally:
            node.destroy_node()


# =====================================================================
# 7. The pure helpers.
# =====================================================================
class TestCentrelineExtraction:

    def test_the_centreline_namespace_is_picked_out_of_the_array(self):
        msg = _marker_array([(0.0, 0.0), (1.0, 0.0)])
        pts, stamp = centreline_from_marker_array(msg, 'corridor_centerline')
        assert pts.shape == (2, 2)
        assert (stamp.sec, stamp.nanosec) == (17, 250)

    def test_the_wall_polylines_are_ignored(self):
        """corridor_left/corridor_right are the corridor's own soft bound,
        not an observation -- seeding on the reference means the
        centreline."""
        msg = _marker_array([(0.0, 0.0), (1.0, 0.0)])
        pts, _stamp = centreline_from_marker_array(msg, 'corridor_centerline')
        # The fixture's wall marker sits at y = 9.0; nothing that far out.
        assert float(np.max(np.abs(pts[:, 1]))) < 1.0

    def test_a_missing_namespace_returns_none(self):
        msg = _marker_array([(0.0, 0.0), (1.0, 0.0)], ns='something_else')
        assert centreline_from_marker_array(msg, 'corridor_centerline') is None

    def test_a_degenerate_single_point_centreline_returns_none(self):
        """None, not an empty array: the caller has to tell "no corridor
        yet" from "a corridor with no room in it"."""
        msg = _marker_array([(0.0, 0.0)])
        assert centreline_from_marker_array(msg, 'corridor_centerline') is None

    def test_a_stale_corridor_is_kept_when_a_new_array_lacks_the_namespace(self):
        node = _node(use_convex_polytope=True)
        try:
            node._corridor_cb(_marker_array([(0.0, 0.0), (1.0, 0.0)]))
            first = node._latest_corridor
            node._corridor_cb(_marker_array([(5.0, 5.0), (6.0, 5.0)], ns='other'))
            assert node._latest_corridor is first
        finally:
            node.destroy_node()


class TestTrimming:

    def test_it_starts_at_the_nearest_sample_and_walks_forward(self):
        pts = np.array([[float(x), 0.0] for x in range(10)])
        out = trim_centreline_ahead(pts, robot_x=3.0, robot_y=0.0, r_local=2.0)
        assert out[0] == pytest.approx([3.0, 0.0])
        assert float(out[-1][0] - out[0][0]) <= 2.0 + 1e-9

    def test_corridor_behind_the_car_is_dropped(self):
        """Spending the face budget separating obstacles the car has
        already driven past would be waste, and a covered prefix that
        starts behind the car describes nothing it is about to do."""
        pts = np.array([[float(x), 0.0] for x in range(10)])
        out = trim_centreline_ahead(pts, robot_x=7.0, robot_y=0.0, r_local=5.0)
        assert float(np.min(out[:, 0])) == 7.0

    def test_a_short_centreline_is_returned_whole(self):
        pts = np.array([[0.0, 0.0], [0.5, 0.0]])
        out = trim_centreline_ahead(pts, 0.0, 0.0, r_local=10.0)
        assert out.shape[0] == 2

    def test_an_empty_centreline_survives(self):
        out = trim_centreline_ahead(np.zeros((0, 2)), 0.0, 0.0, r_local=3.0)
        assert out.shape[0] == 0

    def test_at_least_one_point_always_comes_back(self):
        """r_local smaller than the sample spacing must not produce an
        empty arc -- inflate_polytope needs a seed."""
        pts = np.array([[0.0, 0.0], [5.0, 0.0]])
        out = trim_centreline_ahead(pts, 0.0, 0.0, r_local=0.01)
        assert out.shape[0] >= 1
