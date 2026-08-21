"""is_proximity_too_close.py tests -- mirrors f1tenth_perception/test/
test_lidar_boundary.py's own convention: static/synthetic only, no live
hardware, no rclpy Node instantiation required (the symbol under test,
_in_lidar_window, is a plain function with zero rclpy dependency; this
imports the whole behaviour module, which does still import py_trees/
rclpy-adjacent packages at module scope, but never constructs a Node or
a live py_trees tree).

Added alongside the lidar remount (rear-facing -> front-facing) pass --
this behaviour previously had no dedicated test file at all (its window
math lived inline in _scan_callback); the remount's own angle-window flip
was the occasion to both extract a pure, testable function
(_in_lidar_window) and give it real coverage, same discipline
test_lidar_boundary.py's own front-facing-remount test updates followed.

Run standalone: python3 -m pytest test/test_is_proximity_too_close.py -v
"""

import math

import pytest

from f1tenth_behavior.behaviours.is_proximity_too_close import _in_lidar_window

BLIND_CONE_HALF_ANGLE_RAD = math.radians(45.0)


# ==============================================================================
# Windowing -- the front-facing-mount fact (lidar remount, rear-facing ->
# front-facing) this module now relies on, see module docstring.
# ==============================================================================

class TestInLidarWindow:

    def test_raw_zero_front_is_excluded(self):
        # raw angle 0 = lidar's own forward axis = car's FRONT (yaw=0.0
        # mount, post-remount) -- already covered by the ZED-based front
        # check, so must be excluded (False) here.
        assert _in_lidar_window(0.0, BLIND_CONE_HALF_ANGLE_RAD) is False

    def test_rear_is_included(self):
        # raw angle +-pi = car's REAR -- not covered by the front check,
        # must be included (True).
        assert _in_lidar_window(math.pi, BLIND_CONE_HALF_ANGLE_RAD) is True
        assert _in_lidar_window(-math.pi, BLIND_CONE_HALF_ANGLE_RAD) is True

    def test_sides_are_included(self):
        assert _in_lidar_window(math.radians(90.0), BLIND_CONE_HALF_ANGLE_RAD) is True
        assert _in_lidar_window(math.radians(-90.0), BLIND_CONE_HALF_ANGLE_RAD) is True

    def test_blind_cone_edges_are_inclusive_of_the_boundary_itself(self):
        # >= at the boundary -- exactly blind_cone_half_angle_rad counts as
        # OUTSIDE the blind cone (included), matching _in_lidar_window's own
        # >= comparison.
        assert _in_lidar_window(BLIND_CONE_HALF_ANGLE_RAD, BLIND_CONE_HALF_ANGLE_RAD) is True
        assert _in_lidar_window(-BLIND_CONE_HALF_ANGLE_RAD, BLIND_CONE_HALF_ANGLE_RAD) is True

    def test_just_inside_the_blind_cone_is_excluded(self):
        just_inside = BLIND_CONE_HALF_ANGLE_RAD - math.radians(1.0)
        assert _in_lidar_window(just_inside, BLIND_CONE_HALF_ANGLE_RAD) is False
        assert _in_lidar_window(-just_inside, BLIND_CONE_HALF_ANGLE_RAD) is False

    def test_just_outside_the_blind_cone_is_included(self):
        just_outside = BLIND_CONE_HALF_ANGLE_RAD + math.radians(1.0)
        assert _in_lidar_window(just_outside, BLIND_CONE_HALF_ANGLE_RAD) is True
        assert _in_lidar_window(-just_outside, BLIND_CONE_HALF_ANGLE_RAD) is True

    def test_covers_a_270_degree_arc_total(self):
        # 90deg blind cone (2 * 45deg half-angle) excluded, 270deg included
        # -- sample densely and count both ways as a coarse cross-check of
        # the arc SIZE, not just individual boundary points.
        n = 3600  # 0.1deg resolution
        included = sum(
            1 for i in range(n)
            if _in_lidar_window(
                -math.pi + 2 * math.pi * i / n, BLIND_CONE_HALF_ANGLE_RAD)
        )
        included_deg = included * 360.0 / n
        assert included_deg == pytest.approx(270.0, abs=1.0)


if __name__ == '__main__':
    import sys
    sys.exit(pytest.main([__file__, '-v']))
