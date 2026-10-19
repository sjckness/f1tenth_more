"""Costmap-mode cleanup: the occupied-cell filters behind the boundary rows.

Synthetic grids built by hand, no node and no SLAM. The failure these exist for
is in costmap_boundary.py's own "COSTMAP-MODE CLEANUP" section: the nearest-
occupied-cell extraction takes ONE cell and makes a half-plane tangent to it, so
a single stale/spurious cell becomes a wall row across the MPC's whole horizon,
and the extraction cannot tell it from a wall. The three tests of substance are
therefore all the same shape -- put an island and a real wall on one grid, check
the island stops producing a boundary and the wall does not.

Run standalone: python3 -m pytest test/test_costmap_component_filter.py -v
"""

import math

import numpy as np
import pytest
import rclpy

from f1tenth_costmap.costmap_boundary import (
    ComponentPersistence,
    FREE_CELL,
    extract_boundary_constraints,
    filter_occupied_cells,
    object_footprint_mask,
    occupied_components,
)


@pytest.fixture(scope='module', autouse=True)
def _rclpy_context():
    """One rclpy context for the node-wiring tests at the bottom of this file."""
    rclpy.init()
    yield
    if rclpy.ok():
        rclpy.shutdown()


RES = 0.05
WIDTH = 160
HEIGHT = 160
ORIGIN_X = -4.0
ORIGIN_Y = -4.0
OCCUPIED = 100
THRESHOLD = 65.0
MAX_RANGE = 5.0

# The extraction's own windows, as costmap_boundary_node declares them.
FRONT_MAX = math.radians(35.0)
SIDE_MIN = math.radians(45.0)
SIDE_MAX = math.radians(135.0)


def blank():
    """An all-FREE grid: 8 m square at 5 cm, origin at (-4, -4) so (0, 0) is central."""
    return np.zeros((HEIGHT, WIDTH), dtype=np.int16)


def cell_of(x, y):
    """(row, col) of the cell containing map point (x, y).

    floor with an epsilon, not int(): (3.5 + 3 * 0.05) / 0.05 evaluates to
    72.99999999999999 in binary floating point, and plain truncation puts that
    cell one row short of where it belongs -- which silently breaks the
    DIAGONAL fixture below into a row of disconnected cells and would have this
    file assert the opposite of what the code does.
    """
    return (int(math.floor((y - ORIGIN_Y) / RES + 1e-9)),
            int(math.floor((x - ORIGIN_X) / RES + 1e-9)))


def put_cell(grid, x, y, value=OCCUPIED):
    """One occupied cell at map (x, y) -- the island this filter is for."""
    row, col = cell_of(x, y)
    grid[row, col] = value
    return grid


def put_wall(grid, x, y0, y1, value=OCCUPIED):
    """A vertical (constant-x) occupied run from y0 to y1 -- a real wall."""
    _r0, col = cell_of(x, y0)
    row0, _c = cell_of(x, y0)
    row1, _c = cell_of(x, y1)
    grid[min(row0, row1):max(row0, row1) + 1, col] = value
    return grid


def extract(cells):
    """The node's own boundary extraction, robot at the origin facing +x."""
    return extract_boundary_constraints(
        cells, WIDTH, HEIGHT, RES, ORIGIN_X, ORIGIN_Y, 0.0, 0.0, 0.0,
        FRONT_MAX, SIDE_MIN, SIDE_MAX, THRESHOLD, MAX_RANGE)


def apply_filter(grid, **kwargs):
    """filter_occupied_cells at this grid's geometry."""
    return filter_occupied_cells(
        grid, WIDTH, HEIGHT, RES, ORIGIN_X, ORIGIN_Y, THRESHOLD, **kwargs)


# ==============================================================================
# Components: what the grid says is one surface.
# ==============================================================================

class TestOccupiedComponents:

    def test_an_empty_grid_has_no_components(self):
        _labels, comps = occupied_components(
            blank(), WIDTH, HEIGHT, RES, ORIGIN_X, ORIGIN_Y, THRESHOLD)
        assert comps == []

    def test_a_single_cell_is_a_component_far_shorter_than_the_threshold(self):
        grid = put_cell(blank(), 1.0, 0.0)
        _labels, comps = occupied_components(
            grid, WIDTH, HEIGHT, RES, ORIGIN_X, ORIGIN_Y, THRESHOLD)
        assert len(comps) == 1
        assert comps[0].n_cells == 1
        assert comps[0].length_m == pytest.approx(math.hypot(1, 1) * RES)
        assert comps[0].length_m < 0.5

    def test_a_one_metre_wall_measures_about_a_metre(self):
        grid = put_wall(blank(), 1.0, -0.5, 0.5)
        _labels, comps = occupied_components(
            grid, WIDTH, HEIGHT, RES, ORIGIN_X, ORIGIN_Y, THRESHOLD)
        assert len(comps) == 1
        assert comps[0].length_m == pytest.approx(1.05, abs=0.06), (
            'cell COUNTS, not count-1: the length a tape measure would give')

    def test_a_diagonal_wall_stays_one_component(self):
        # 8-connectivity, deliberately: a wall rendered into a grid at an angle
        # is a staircase of diagonally-touching cells, and 4-connectivity would
        # chop it into a row of "short" components -- the exact mistake this
        # filter exists to avoid making.
        grid = blank()
        for i in range(20):
            put_cell(grid, 1.0 + i * RES, -0.5 + i * RES)
        _labels, comps = occupied_components(
            grid, WIDTH, HEIGHT, RES, ORIGIN_X, ORIGIN_Y, THRESHOLD)
        assert len(comps) == 1, f'a diagonal run must be ONE surface -- got {comps}'
        assert comps[0].length_m > 0.5

    def test_the_centroid_is_in_map_metres(self):
        grid = put_wall(blank(), 1.5, -0.5, 0.5)
        _labels, comps = occupied_components(
            grid, WIDTH, HEIGHT, RES, ORIGIN_X, ORIGIN_Y, THRESHOLD)
        assert comps[0].centroid_x == pytest.approx(1.5, abs=RES)
        assert comps[0].centroid_y == pytest.approx(0.0, abs=RES)

    def test_unknown_cells_are_never_components(self):
        grid = blank()
        grid[:, :] = -1
        _labels, comps = occupied_components(
            grid, WIDTH, HEIGHT, RES, ORIGIN_X, ORIGIN_Y, THRESHOLD)
        assert comps == [], (
            'slam_toolbox writes -1 for unexplored; the >= threshold test excludes '
            'it structurally and this filter must not reintroduce it')


# ==============================================================================
# 10. Costmap mode with the filter on: the island goes, the real wall stays.
# ==============================================================================

class TestIslandVersusWall:

    def _grid(self):
        """A real 1 m wall 2 m ahead, and one spurious cell 1 m ahead of it."""
        grid = put_wall(blank(), 2.0, -0.5, 0.5)
        put_cell(grid, 1.0, 0.0)
        return grid

    def test_unfiltered_the_single_cell_IS_the_front_boundary(self):
        # The premise. Without the filter the nearer of the two wins, and the
        # nearer one is the island: a half-plane at 1 m from one cell.
        front = extract(self._grid())['front']
        assert front is not None
        assert front[2] == pytest.approx(1.0, abs=2 * RES), (
            f'the island at 1 m must be what the extraction picks -- got {front}')

    def test_filtered_the_island_is_gone_and_the_wall_is_the_boundary(self):
        cells, report = apply_filter(self._grid(), min_component_length_m=0.5)
        assert report['dropped_short'] == 1
        front = extract(cells)['front']
        assert front is not None, 'the real wall must survive the filter'
        assert front[2] == pytest.approx(2.0, abs=2 * RES), (
            f'the boundary must now be the 1 m wall at 2 m -- got {front}')

    def test_the_wall_s_own_cells_are_untouched(self):
        grid = self._grid()
        cells, _report = apply_filter(grid, min_component_length_m=0.5)
        row, col = cell_of(2.0, 0.0)
        assert cells[row, col] == OCCUPIED
        row, col = cell_of(1.0, 0.0)
        assert cells[row, col] == FREE_CELL

    def test_a_dropped_cell_reads_free_not_unknown(self):
        # 0, not -1: the cell WAS observed and the grid's record of that should
        # survive this filter's opinion about what it means.
        cells, _report = apply_filter(put_cell(blank(), 1.0, 0.0),
                                      min_component_length_m=0.5)
        row, col = cell_of(1.0, 0.0)
        assert cells[row, col] == 0

    def test_the_filter_never_writes_through_to_the_caller_s_grid(self):
        # front_clearance is recomputed from the SAME latched message on the
        # same tick, deliberately unfiltered. An in-place write would silently
        # filter a live mission-stopping signal too.
        grid = self._grid()
        before = grid.copy()
        apply_filter(grid, min_component_length_m=0.5)
        assert np.array_equal(grid, before)

    def test_both_switches_off_is_the_grid_exactly_as_it_arrived(self):
        grid = self._grid()
        cells, report = apply_filter(grid)
        assert np.array_equal(np.asarray(cells), grid)
        assert report['n_components'] == 0
        assert extract(cells)['front'][2] == pytest.approx(
            extract(grid)['front'][2]), 'the default path must be unchanged'

    def test_a_wall_just_over_the_threshold_is_kept(self):
        # 0.5 m is the threshold, and the test is >=: a wall measuring exactly
        # the threshold is a wall. Erring the other way would throw away the
        # shortest real surfaces, which is this filter's whole risk.
        grid = put_wall(blank(), 2.0, 0.0, 0.45)
        _labels, comps = occupied_components(
            grid, WIDTH, HEIGHT, RES, ORIGIN_X, ORIGIN_Y, THRESHOLD)
        assert comps[0].length_m >= 0.5
        cells, report = apply_filter(grid, min_component_length_m=0.5)
        assert report['dropped_short'] == 0
        assert extract(cells)['front'] is not None


# ==============================================================================
# Persistence: consecutive extraction TICKS, not scans (this node sees no scans).
# ==============================================================================

class TestComponentPersistence:

    def test_a_component_seen_once_counts_one(self):
        grid = put_wall(blank(), 2.0, -0.5, 0.5)
        _labels, comps = occupied_components(
            grid, WIDTH, HEIGHT, RES, ORIGIN_X, ORIGIN_Y, THRESHOLD)
        counts = ComponentPersistence().step(comps, RES)
        assert counts == {comps[0].label: 1}

    def test_the_same_component_on_consecutive_ticks_accumulates(self):
        grid = put_wall(blank(), 2.0, -0.5, 0.5)
        _labels, comps = occupied_components(
            grid, WIDTH, HEIGHT, RES, ORIGIN_X, ORIGIN_Y, THRESHOLD)
        persistence = ComponentPersistence()
        seen = [persistence.step(comps, RES)[comps[0].label] for _ in range(4)]
        assert seen == [1, 2, 3, 4]

    def test_a_gap_of_one_tick_resets_the_count(self):
        grid = put_wall(blank(), 2.0, -0.5, 0.5)
        _labels, comps = occupied_components(
            grid, WIDTH, HEIGHT, RES, ORIGIN_X, ORIGIN_Y, THRESHOLD)
        persistence = ComponentPersistence()
        persistence.step(comps, RES)
        persistence.step(comps, RES)
        persistence.step([], RES)
        assert persistence.step(comps, RES)[comps[0].label] == 1, (
            'CONSECUTIVE is the point: a component that flickers is exactly the '
            'one-tick artefact this is for')

    def test_a_transient_component_produces_no_boundary_on_its_first_tick(self):
        # A long enough "wall" that has only ever been seen once. The length
        # test passes it; persistence does not.
        grid = put_wall(blank(), 1.0, -0.5, 0.5)
        labels, comps = occupied_components(
            grid, WIDTH, HEIGHT, RES, ORIGIN_X, ORIGIN_Y, THRESHOLD)
        counts = ComponentPersistence().step(comps, RES)
        cells, report = apply_filter(
            grid, min_component_length_m=0.5, min_component_ticks=2,
            tick_counts=counts, components=comps, labels=labels)
        assert report['dropped_transient'] == 1
        assert extract(cells)['front'] is None

    def test_and_does_produce_one_on_the_second(self):
        grid = put_wall(blank(), 1.0, -0.5, 0.5)
        labels, comps = occupied_components(
            grid, WIDTH, HEIGHT, RES, ORIGIN_X, ORIGIN_Y, THRESHOLD)
        persistence = ComponentPersistence()
        persistence.step(comps, RES)
        counts = persistence.step(comps, RES)
        cells, report = apply_filter(
            grid, min_component_length_m=0.5, min_component_ticks=2,
            tick_counts=counts, components=comps, labels=labels)
        assert report['dropped_transient'] == 0
        assert extract(cells)['front'] is not None

    def test_min_component_scans_of_one_disables_the_test(self):
        grid = put_wall(blank(), 1.0, -0.5, 0.5)
        _cells, report = apply_filter(
            grid, min_component_length_m=0.5, min_component_ticks=1)
        assert report['dropped_transient'] == 0


# ==============================================================================
# Tracked objects: already handled by w_obs, so never also a wall row.
# ==============================================================================

class TestObjectFootprintExclusion:

    def test_an_empty_disk_list_masks_nothing(self):
        mask = object_footprint_mask(WIDTH, HEIGHT, RES, ORIGIN_X, ORIGIN_Y, [], 0.12)
        assert mask.shape == (HEIGHT, WIDTH)
        assert not mask.any()

    def test_the_mask_covers_the_footprint_plus_the_margin(self):
        mask = object_footprint_mask(
            WIDTH, HEIGHT, RES, ORIGIN_X, ORIGIN_Y, [(1.0, 0.0, 0.2)], 0.12)
        inside = cell_of(1.0 + 0.25, 0.0)
        outside = cell_of(1.0 + 0.40, 0.0)
        assert mask[inside], 'r 0.20 + margin 0.12 = 0.32 m must cover 0.25 m out'
        assert not mask[outside], 'and must not cover 0.40 m out'

    def test_a_person_s_cells_stop_producing_a_boundary(self):
        # A person 1 m ahead rendered into the map as a small blob, plus the
        # real wall at 2 m. The blob is long enough to pass the length test, so
        # only the object exclusion removes it -- which is the point: the MPC's
        # soft w_obs cost already has this object, and the hard-ish copy is the
        # one that can stop the car dead.
        grid = put_wall(blank(), 2.0, -0.5, 0.5)
        for dy in np.arange(-0.25, 0.26, RES):
            put_cell(grid, 1.0, float(dy))
        assert extract(grid)['front'][2] == pytest.approx(1.0, abs=2 * RES)

        cells, report = apply_filter(
            grid, object_disks_map=[(1.0, 0.0, 0.3)], object_margin_m=0.12)
        assert report['dropped_object_cells'] > 0
        front = extract(cells)['front']
        assert front is not None and front[2] == pytest.approx(2.0, abs=2 * RES), (
            f'the wall behind the person must become the boundary -- got {front}')

    def test_the_wall_is_not_removed_by_a_nearby_object(self):
        grid = put_wall(blank(), 2.0, -0.5, 0.5)
        cells, report = apply_filter(
            grid, object_disks_map=[(1.0, 0.0, 0.3)], object_margin_m=0.12)
        assert report['dropped_object_cells'] == 0
        assert extract(cells)['front'][2] == pytest.approx(2.0, abs=2 * RES)

    def test_a_zero_radius_disk_with_no_margin_masks_nothing(self):
        mask = object_footprint_mask(
            WIDTH, HEIGHT, RES, ORIGIN_X, ORIGIN_Y, [(1.0, 0.0, 0.0)], 0.0)
        assert not mask.any()


# ==============================================================================
# The node wiring: the filter reaches /costmap/boundaries and NOT front_clearance.
#
# Same harness as test_costmap_boundary_node.py -- a real but never-spun rclpy
# Node with its two publishers mocked, so each test asserts exactly what would
# have gone on the wire. The pure-function tests above prove the rule; these
# prove the node applies it where it should and nowhere else.
# ==============================================================================

class TestNodeWiring:

    def _node(self, params):
        from unittest.mock import MagicMock
        from rclpy.parameter import Parameter
        from f1tenth_costmap.costmap_boundary_node import CostmapBoundaryNode
        node = CostmapBoundaryNode(
            parameter_overrides=[Parameter(k, value=v) for k, v in params.items()])
        node.boundary_pub.publish = MagicMock()
        node.clearance_pub.publish = MagicMock()
        return node

    def _feed(self, node, grid_array):
        from nav_msgs.msg import OccupancyGrid, Odometry
        grid = OccupancyGrid()
        grid.info.width = WIDTH
        grid.info.height = HEIGHT
        grid.info.resolution = RES
        grid.info.origin.position.x = ORIGIN_X
        grid.info.origin.position.y = ORIGIN_Y
        grid.data = [int(v) for v in np.asarray(grid_array).ravel()]
        pose = Odometry()
        pose.pose.pose.orientation.w = 1.0
        node._map_cb(grid)
        node._pose_cb(pose)

    def _island_and_wall(self):
        grid = put_wall(blank(), 2.0, -0.5, 0.5)
        put_cell(grid, 1.0, 0.0)
        return grid

    def test_the_filter_is_off_by_default(self):
        node = self._node({})
        try:
            assert node.component_filter is False
            assert node.exclude_tracked_object_cells is False
            assert node.obstacles_sub is None, (
                'an unread subscription would deserialise every detection frame '
                'for nothing in the default configuration')
            assert node._persistence is None
        finally:
            node.destroy_node()

    def test_off_the_island_is_still_the_published_front_boundary(self):
        node = self._node({'min_component_scans': 1})
        try:
            self._feed(node, self._island_and_wall())
            node._extraction_tick()
            offsets = [c.offset for c
                       in node.boundary_pub.publish.call_args[0][0].constraints]
            assert any(abs(o - 1.0) < 2 * RES for o in offsets), (
                f'the premise: unfiltered, the island at 1 m wins -- got {offsets}')
        finally:
            node.destroy_node()

    def test_on_the_island_is_gone_from_the_published_rows(self):
        node = self._node({'component_filter': True,
                           'min_component_length_m': 0.5,
                           'min_component_scans': 1})
        try:
            self._feed(node, self._island_and_wall())
            node._extraction_tick()
            offsets = [c.offset for c
                       in node.boundary_pub.publish.call_args[0][0].constraints]
            assert not any(abs(o - 1.0) < 2 * RES for o in offsets), (
                f'the island must produce no row -- got {offsets}')
            assert any(abs(o - 2.0) < 2 * RES for o in offsets), (
                f'and the real wall behind it must produce one -- got {offsets}')
        finally:
            node.destroy_node()

    def test_front_clearance_is_computed_from_the_UNFILTERED_grid(self):
        # The one thing the task says not to touch. front_clearance feeds the
        # behaviour tree's own mission-stopping condition, and "would the car
        # hit something going straight" is a question a single cell is a
        # perfectly good answer to. The island sits in the car-width strip at
        # 1 m, so the clearance must still report it even with the filter on.
        node = self._node({'component_filter': True,
                           'min_component_length_m': 0.5,
                           'min_component_scans': 1})
        try:
            self._feed(node, self._island_and_wall())
            node._extraction_tick()
            clearance = node.clearance_pub.publish.call_args[0][0].data
            from f1tenth_description.vehicle_frames import base_front_x
            assert clearance == pytest.approx(1.0 - base_front_x(), abs=3 * RES), (
                'the filter must not reach front_clearance: a filtered clearance '
                f'would read ~{2.0 - base_front_x():.2f} m -- got {clearance:.3f}')
        finally:
            node.destroy_node()

    def test_the_persistence_test_withholds_a_brand_new_component(self):
        node = self._node({'component_filter': True,
                           'min_component_length_m': 0.5,
                           'min_component_scans': 2})
        try:
            assert node._persistence is not None
            self._feed(node, put_wall(blank(), 1.0, -0.5, 0.5))
            node._extraction_tick()
            first = list(node.boundary_pub.publish.call_args[0][0].constraints)
            node._extraction_tick()
            second = list(node.boundary_pub.publish.call_args[0][0].constraints)
            assert first == [], f'tick 1: one sighting is not enough -- got {first}'
            assert second, 'tick 2: the same component on consecutive ticks counts'
        finally:
            node.destroy_node()

    def test_the_object_exclusion_subscribes_only_when_enabled(self):
        node = self._node({'exclude_tracked_object_cells': True})
        try:
            assert node.obstacles_sub is not None
        finally:
            node.destroy_node()

    def test_a_tracked_object_s_cells_stop_producing_a_row(self):
        from f1tenth_messages.msg import Obstacle2D, Obstacle2DArray
        node = self._node({'exclude_tracked_object_cells': True,
                           'object_footprint_margin_m': 0.12})
        try:
            grid = put_wall(blank(), 2.0, -0.5, 0.5)
            for dy in np.arange(-0.25, 0.26, RES):
                put_cell(grid, 1.0, float(dy))
            self._feed(node, grid)
            msg = Obstacle2DArray()
            msg.obstacles = [Obstacle2D(x=1.0, y=0.0, r=0.3)]
            node._obstacles_cb(msg)
            node._extraction_tick()
            offsets = [c.offset for c
                       in node.boundary_pub.publish.call_args[0][0].constraints]
            assert not any(abs(o - 1.0) < 2 * RES for o in offsets), (
                f'the object is already handled by w_obs -- got {offsets}')
            assert any(abs(o - 2.0) < 2 * RES for o in offsets)
        finally:
            node.destroy_node()

    def test_the_base_link_disks_are_placed_in_map_with_the_robot_s_pose(self):
        # The transform, isolated: a disk 1 m ahead in base_link with the robot
        # at (2, 1) yawed 90 deg must land at (2, 2) in map.
        node = self._node({'exclude_tracked_object_cells': True})
        try:
            node._latest_obstacles_bl = [(1.0, 0.0, 0.3)]
            (x, y, r), = node._object_disks_map(2.0, 1.0, math.pi / 2.0)
            assert (x, y, r) == pytest.approx((2.0, 2.0, 0.3), abs=1e-9)
        finally:
            node.destroy_node()
