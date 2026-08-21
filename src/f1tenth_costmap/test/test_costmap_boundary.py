"""costmap_boundary.py tests -- static/synthetic grids only, no live
hardware, no rclpy Node instantiation required. Same convention test_
costmap_renderer.py's own module docstring already establishes for this
package's pure-logic modules.

Run standalone: python3 -m pytest test/test_costmap_boundary.py -v
"""

import math
from types import SimpleNamespace

import numpy as np
import pytest

from f1tenth_costmap.costmap_boundary import (
    boundary_from_nearest_point,
    extract_boundary_constraints,
    front_clearance_from_extraction,
    nearest_occupied_in_window,
    yaw_from_quaternion,
)

_WIDTH = 10
_HEIGHT = 10
_RES = 1.0
_ORIGIN_X = 0.0
_ORIGIN_Y = 0.0
# Robot sits at cell-center (2,5) -- (2.5, 5.5) -- facing map +x (yaw=0),
# same fixture every test in this module reuses unless noted otherwise.
_ROBOT_X, _ROBOT_Y, _ROBOT_YAW = 2.5, 5.5, 0.0
_FRONT_MAX = math.radians(35.0)
_SIDE_MIN = math.radians(45.0)
_SIDE_MAX = math.radians(135.0)
_OCC_THRESH = 65
_MAX_RANGE = 5.0


def _idx(col: int, row: int) -> int:
    return row * _WIDTH + col


def _blank_grid(fill=0) -> list:
    return [fill] * (_WIDTH * _HEIGHT)


def _extract(grid, robot_x=_ROBOT_X, robot_y=_ROBOT_Y, robot_yaw=_ROBOT_YAW,
             max_range_m=_MAX_RANGE):
    return extract_boundary_constraints(
        grid, _WIDTH, _HEIGHT, _RES, _ORIGIN_X, _ORIGIN_Y, robot_x, robot_y, robot_yaw,
        _FRONT_MAX, _SIDE_MIN, _SIDE_MAX, _OCC_THRESH, max_range_m)


# ==============================================================================
# yaw_from_quaternion
# ==============================================================================

class TestYawFromQuaternion:

    def test_identity_is_zero_yaw(self):
        q = SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0)
        assert yaw_from_quaternion(q) == pytest.approx(0.0, abs=1e-9)

    def test_recovers_a_known_yaw(self):
        yaw = math.radians(42.0)
        q = SimpleNamespace(x=0.0, y=0.0, z=math.sin(yaw / 2.0), w=math.cos(yaw / 2.0))
        assert yaw_from_quaternion(q) == pytest.approx(yaw)


# ==============================================================================
# nearest_occupied_in_window
# ==============================================================================

class TestNearestOccupiedInWindow:

    def test_finds_the_single_occupied_cell_straight_ahead(self):
        grid = _blank_grid()
        grid[_idx(6, 5)] = 100  # 4m straight ahead
        hit = nearest_occupied_in_window(
            grid, _WIDTH, _HEIGHT, _RES, _ORIGIN_X, _ORIGIN_Y,
            _ROBOT_X, _ROBOT_Y, _ROBOT_YAW, -_FRONT_MAX, _FRONT_MAX,
            _OCC_THRESH, _MAX_RANGE)
        assert hit is not None
        car_dx, car_dy, dist = hit
        assert car_dx == pytest.approx(4.0)
        assert car_dy == pytest.approx(0.0, abs=1e-9)
        assert dist == pytest.approx(4.0)

    def test_picks_the_nearer_of_two_occupied_cells_in_window(self):
        grid = _blank_grid()
        grid[_idx(6, 5)] = 100  # 4m
        grid[_idx(4, 5)] = 100  # 2m -- nearer, must win
        hit = nearest_occupied_in_window(
            grid, _WIDTH, _HEIGHT, _RES, _ORIGIN_X, _ORIGIN_Y,
            _ROBOT_X, _ROBOT_Y, _ROBOT_YAW, -_FRONT_MAX, _FRONT_MAX,
            _OCC_THRESH, _MAX_RANGE)
        assert hit[2] == pytest.approx(2.0)

    def test_below_threshold_cell_is_not_selected(self):
        grid = _blank_grid()
        grid[_idx(6, 5)] = _OCC_THRESH - 1  # just below the occupied cutoff
        hit = nearest_occupied_in_window(
            grid, _WIDTH, _HEIGHT, _RES, _ORIGIN_X, _ORIGIN_Y,
            _ROBOT_X, _ROBOT_Y, _ROBOT_YAW, -_FRONT_MAX, _FRONT_MAX,
            _OCC_THRESH, _MAX_RANGE)
        assert hit is None

    def test_unknown_cells_are_never_selected_as_occupied(self):
        # -1 (unknown) is below any positive occupied_threshold -- a grid
        # that is ENTIRELY unknown must behave identically to nothing being
        # in range at all (None), not a false "clear" or false "tight"
        # result -- see module docstring's own "UNKNOWN CELLS" section.
        grid = _blank_grid(fill=-1)
        hit = nearest_occupied_in_window(
            grid, _WIDTH, _HEIGHT, _RES, _ORIGIN_X, _ORIGIN_Y,
            _ROBOT_X, _ROBOT_Y, _ROBOT_YAW, -_FRONT_MAX, _FRONT_MAX,
            _OCC_THRESH, _MAX_RANGE)
        assert hit is None

    def test_out_of_window_cell_is_ignored(self):
        grid = _blank_grid()
        grid[_idx(2, 9)] = 100  # directly to the left (car frame), not front
        hit = nearest_occupied_in_window(
            grid, _WIDTH, _HEIGHT, _RES, _ORIGIN_X, _ORIGIN_Y,
            _ROBOT_X, _ROBOT_Y, _ROBOT_YAW, -_FRONT_MAX, _FRONT_MAX,
            _OCC_THRESH, _MAX_RANGE)
        assert hit is None

    def test_out_of_range_cell_is_ignored(self):
        grid = _blank_grid()
        grid[_idx(9, 5)] = 100  # straight ahead but 6.5m away
        hit = nearest_occupied_in_window(
            grid, _WIDTH, _HEIGHT, _RES, _ORIGIN_X, _ORIGIN_Y,
            _ROBOT_X, _ROBOT_Y, _ROBOT_YAW, -_FRONT_MAX, _FRONT_MAX,
            _OCC_THRESH, 5.0)
        assert hit is None

    def test_robot_box_entirely_outside_grid_returns_none_not_a_crash(self):
        grid = _blank_grid()
        hit = nearest_occupied_in_window(
            grid, _WIDTH, _HEIGHT, _RES, _ORIGIN_X, _ORIGIN_Y,
            robot_x=1000.0, robot_y=1000.0, robot_yaw=0.0,
            window_min_rad=-_FRONT_MAX, window_max_rad=_FRONT_MAX,
            occupied_threshold=_OCC_THRESH, max_range_m=_MAX_RANGE)
        assert hit is None

    def test_yaw_rotates_which_cells_count_as_front(self):
        grid = _blank_grid()
        grid[_idx(2, 8)] = 100  # map +y direction from the robot
        # yaw=0: this cell is 'left', not 'front'.
        hit_yaw0 = nearest_occupied_in_window(
            grid, _WIDTH, _HEIGHT, _RES, _ORIGIN_X, _ORIGIN_Y,
            _ROBOT_X, _ROBOT_Y, 0.0, -_FRONT_MAX, _FRONT_MAX, _OCC_THRESH, _MAX_RANGE)
        assert hit_yaw0 is None
        # yaw=+90deg: car's front axis now points toward map +y -- same
        # physical cell is now 'front'.
        hit_yaw90 = nearest_occupied_in_window(
            grid, _WIDTH, _HEIGHT, _RES, _ORIGIN_X, _ORIGIN_Y,
            _ROBOT_X, _ROBOT_Y, math.radians(90.0),
            -_FRONT_MAX, _FRONT_MAX, _OCC_THRESH, _MAX_RANGE)
        assert hit_yaw90 is not None


# ==============================================================================
# boundary_from_nearest_point
# ==============================================================================

class TestBoundaryFromNearestPoint:

    def test_straight_ahead_point(self):
        nx, ny, offset = boundary_from_nearest_point(4.0, 0.0)
        assert (nx, ny) == pytest.approx((1.0, 0.0))
        assert offset == pytest.approx(4.0)

    def test_left_point(self):
        nx, ny, offset = boundary_from_nearest_point(0.0, 3.0)
        assert (nx, ny) == pytest.approx((0.0, 1.0))
        assert offset == pytest.approx(3.0)

    def test_diagonal_point_normal_is_unit_length(self):
        nx, ny, offset = boundary_from_nearest_point(3.0, 4.0)
        assert math.hypot(nx, ny) == pytest.approx(1.0)
        assert offset == pytest.approx(5.0)

    def test_zero_distance_returns_inert_sentinel_not_a_crash(self):
        nx, ny, offset = boundary_from_nearest_point(0.0, 0.0)
        assert (nx, ny) == (0.0, 0.0)
        assert math.isinf(offset)

    def test_sign_convention_matches_BoundaryConstraint_msg(self):
        # The robot's own origin (0, 0) must satisfy normal . (0,0) <= offset
        # -- trivially 0 <= offset, true for any real (non-inert) result,
        # confirming the halfspace is oriented the right way (free side
        # small, obstacle side large) per BoundaryConstraint.msg's own
        # documented convention.
        nx, ny, offset = boundary_from_nearest_point(2.0, -1.0)
        assert 0.0 * nx + 0.0 * ny <= offset


# ==============================================================================
# extract_boundary_constraints -- full per-direction orchestration
# ==============================================================================

class TestExtractBoundaryConstraints:

    def test_all_three_directions_independently_populated(self):
        grid = _blank_grid()
        grid[_idx(6, 5)] = 100   # front, 4m
        grid[_idx(2, 8)] = 100   # left, 2.5m
        grid[_idx(2, 2)] = 100   # right, 3.5m
        result = _extract(grid)
        assert result['front'] is not None
        assert result['left'] is not None
        assert result['right'] is not None

    def test_empty_grid_returns_all_none(self):
        result = _extract(_blank_grid())
        assert result == {'front': None, 'left': None, 'right': None}

    def test_only_populated_directions_are_non_none(self):
        grid = _blank_grid()
        grid[_idx(6, 5)] = 100  # front only
        result = _extract(grid)
        assert result['front'] is not None
        assert result['left'] is None
        assert result['right'] is None

    def test_left_and_right_windows_are_mirrored(self):
        grid_left = _blank_grid()
        grid_left[_idx(2, 8)] = 100
        grid_right = _blank_grid()
        grid_right[_idx(2, 2)] = 100
        left_result = _extract(grid_left)
        right_result = _extract(grid_right)
        assert left_result['left'] is not None and left_result['right'] is None
        assert right_result['right'] is not None and right_result['left'] is None


# ==============================================================================
# front_clearance_from_extraction
# ==============================================================================

class TestFrontClearanceFromExtraction:

    def test_reports_the_front_direction_distance(self):
        grid = _blank_grid()
        grid[_idx(6, 5)] = 100
        result = _extract(grid)
        assert front_clearance_from_extraction(result, _MAX_RANGE) == pytest.approx(4.0)

    def test_no_front_occupied_cell_returns_finite_value_past_max_range(self):
        # NOT math.inf -- see module docstring for why an honest "searched
        # this far and found nothing" value is used instead of overstating
        # confidence with an unbounded number.
        result = _extract(_blank_grid())
        clearance = front_clearance_from_extraction(result, _MAX_RANGE)
        assert math.isfinite(clearance)
        assert clearance > _MAX_RANGE

    def test_left_or_right_hits_do_not_affect_front_clearance(self):
        grid = _blank_grid()
        grid[_idx(2, 8)] = 100  # left only
        result = _extract(grid)
        clearance = front_clearance_from_extraction(result, _MAX_RANGE)
        assert clearance > _MAX_RANGE


# ==============================================================================
# dtype/shape robustness -- OccupancyGrid.data is a real int8 array/array.array
# in production, not a plain Python list -- confirm np.asarray handles both,
# same precedent test_costmap_renderer.py's own module docstring establishes.
# ==============================================================================

class TestInputFlexibility:

    def test_accepts_a_numpy_int8_array_without_overflow(self):
        grid = np.zeros(_WIDTH * _HEIGHT, dtype=np.int8)
        grid[_idx(6, 5)] = 100
        result = _extract(list(grid))
        assert result['front'] is not None


if __name__ == '__main__':
    import sys
    sys.exit(pytest.main([__file__, '-v']))
