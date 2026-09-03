"""is_proximity_too_close.py tests -- mirrors f1tenth_perception/test/
test_lidar_boundary.py's own convention: static/synthetic only, no live
hardware, no rclpy Node instantiation required. `_in_lidar_window`/
`_in_front_cone` are plain functions with zero rclpy dependency, tested
directly; the `IsProximityTooClose` behaviour itself is constructed directly
too (its `__init__` has no rclpy dependency either -- only `setup()` does,
which none of these tests call), with `.node` set to a `MagicMock()` where a
scan callback needs to log through it, so `_scan_callback`/`update()` can be
exercised end to end without a live subscription, publisher, or rclpy
context.

Added alongside the lidar remount (rear-facing -> front-facing) pass --
this behaviour previously had no dedicated test file at all (its window
math lived inline in _scan_callback); the remount's own angle-window flip
was the occasion to both extract a pure, testable function
(_in_lidar_window) and give it real coverage, same discipline
test_lidar_boundary.py's own front-facing-remount test updates followed.

Extended for the camera -> lidar front-cone swap (front e-stop: ZED depth ->
LiDAR, see is_proximity_too_close.py's own module docstring for the full
rationale/trade-off): `_in_front_cone` is `_in_lidar_window`'s counterpart
(same angle convention, opposite membership), and TestScanCallbackFrontCone
below exercises `_scan_callback`/`update()` together against synthetic
LaserScan messages -- the actual wiring, not just the pure per-angle
functions in isolation -- confirming the front check is genuinely driven by
`/scan` now and that removing the old `/perception/front_distance`
subscription didn't leave a stray reference to it anywhere on the class.

Run standalone: python3 -m pytest test/test_is_proximity_too_close.py -v
"""

import math
from unittest.mock import MagicMock

import py_trees
import pytest
from sensor_msgs.msg import LaserScan

from f1tenth_behavior.behaviours.is_proximity_too_close import (
    IsProximityTooClose,
    _in_front_cone,
    _in_lidar_window,
)

BLIND_CONE_HALF_ANGLE_RAD = math.radians(45.0)
FRONT_CONE_HALF_ANGLE_RAD = math.radians(45.0)


# ==============================================================================
# Windowing -- the front-facing-mount fact (lidar remount, rear-facing ->
# front-facing) this module now relies on, see module docstring.
# ==============================================================================

class TestInLidarWindow:
    """Side/rear window -- UNCHANGED by the camera -> lidar front-cone swap;
    kept here verbatim as the existing regression coverage for that pass."""

    def test_raw_zero_front_is_excluded(self):
        # raw angle 0 = lidar's own forward axis = car's FRONT (yaw=0.0
        # mount, post-remount) -- covered by the dedicated front-cone check
        # (_in_front_cone below) instead, so must be excluded (False) here.
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
        # 90deg front cone excluded, 270deg included -- sample densely and
        # count both ways as a coarse cross-check of the arc SIZE, not just
        # individual boundary points.
        n = 3600  # 0.1deg resolution
        included = sum(
            1 for i in range(n)
            if _in_lidar_window(
                -math.pi + 2 * math.pi * i / n, BLIND_CONE_HALF_ANGLE_RAD)
        )
        included_deg = included * 360.0 / n
        assert included_deg == pytest.approx(270.0, abs=1.0)


class TestInFrontCone:
    """Front cone -- _in_lidar_window's counterpart, added for the camera ->
    lidar front-cone swap. Same raw-scan-frame angle convention; opposite
    membership (INSIDE the cone, not outside it)."""

    def test_raw_zero_front_is_included(self):
        assert _in_front_cone(0.0, FRONT_CONE_HALF_ANGLE_RAD) is True

    def test_rear_is_excluded(self):
        assert _in_front_cone(math.pi, FRONT_CONE_HALF_ANGLE_RAD) is False
        assert _in_front_cone(-math.pi, FRONT_CONE_HALF_ANGLE_RAD) is False

    def test_sides_are_excluded(self):
        assert _in_front_cone(math.radians(90.0), FRONT_CONE_HALF_ANGLE_RAD) is False
        assert _in_front_cone(math.radians(-90.0), FRONT_CONE_HALF_ANGLE_RAD) is False

    def test_cone_edge_is_exclusive_of_the_boundary_itself(self):
        # Strict < at the boundary -- exactly front_cone_half_angle_rad counts
        # as OUTSIDE the front cone (excluded), the exact complement of
        # _in_lidar_window's own >= (which counts the boundary as OUTSIDE the
        # blind cone, i.e. included there) -- see _in_front_cone's own
        # docstring for why this is deliberate, not an off-by-one.
        assert _in_front_cone(FRONT_CONE_HALF_ANGLE_RAD, FRONT_CONE_HALF_ANGLE_RAD) is False
        assert _in_front_cone(-FRONT_CONE_HALF_ANGLE_RAD, FRONT_CONE_HALF_ANGLE_RAD) is False

    def test_just_inside_the_cone_is_included(self):
        just_inside = FRONT_CONE_HALF_ANGLE_RAD - math.radians(1.0)
        assert _in_front_cone(just_inside, FRONT_CONE_HALF_ANGLE_RAD) is True
        assert _in_front_cone(-just_inside, FRONT_CONE_HALF_ANGLE_RAD) is True

    def test_just_outside_the_cone_is_excluded(self):
        just_outside = FRONT_CONE_HALF_ANGLE_RAD + math.radians(1.0)
        assert _in_front_cone(just_outside, FRONT_CONE_HALF_ANGLE_RAD) is False
        assert _in_front_cone(-just_outside, FRONT_CONE_HALF_ANGLE_RAD) is False

    def test_covers_a_90_degree_arc_total(self):
        n = 3600  # 0.1deg resolution
        included = sum(
            1 for i in range(n)
            if _in_front_cone(
                -math.pi + 2 * math.pi * i / n, FRONT_CONE_HALF_ANGLE_RAD)
        )
        included_deg = included * 360.0 / n
        assert included_deg == pytest.approx(90.0, abs=1.0)

    def test_partitions_the_full_circle_with_in_lidar_window_at_equal_angles(self):
        # With equal half-angles (the default for both, see is_proximity_
        # too_close.py's own module docstring), every sample belongs to
        # EXACTLY ONE of the two windows -- never both, never neither. This
        # is the property _scan_callback's independent-per-sample evaluation
        # relies on for the default config to behave like a clean partition.
        n = 3600
        for i in range(n):
            angle = -math.pi + 2 * math.pi * i / n
            front = _in_front_cone(angle, FRONT_CONE_HALF_ANGLE_RAD)
            side_rear = _in_lidar_window(angle, BLIND_CONE_HALF_ANGLE_RAD)
            assert front != side_rear, f'angle={angle} counted as both or neither'


# ==============================================================================
# _scan_callback / update() -- the actual wiring, exercised end to end
# against synthetic LaserScan messages. Confirms the front check is
# genuinely LiDAR-driven now, not just that the pure window functions are
# individually correct.
# ==============================================================================

def _construct(**kwargs):
    """IsProximityTooClose with .node stubbed to a MagicMock -- __init__ has
    no rclpy dependency, and _scan_callback only ever touches self.node to
    log through it, so this is enough to call _scan_callback/update()
    directly without a live rclpy context, subscription, or Node."""
    behaviour = IsProximityTooClose(**kwargs)
    behaviour.node = MagicMock()
    return behaviour


def _fake_scan(samples, range_min=0.05):
    """`samples`: list of (angle_rad, range_m) pairs, evenly spaced and
    ordered by angle (every test below constructs them that way) -- built as
    a LaserScan whose angle_min is samples[0]'s own angle and whose
    angle_increment reproduces every subsequent sample's angle exactly, so
    msg.angle_min + i * msg.angle_increment recovers each pair's intended
    angle_rad precisely (not just within these tests' >=1deg-wide windows)."""
    msg = LaserScan()
    angles = [a for a, _ in samples]
    ranges = [r for _, r in samples]
    msg.angle_min = angles[0]
    if len(angles) > 1:
        increments = [angles[i + 1] - angles[i] for i in range(len(angles) - 1)]
        angle_increment = increments[0]
        assert all(abs(inc - angle_increment) < 1e-9 for inc in increments), (
            '_fake_scan requires evenly-spaced angles')
    else:
        angle_increment = 1.0  # irrelevant for a single sample
    msg.angle_increment = angle_increment
    msg.angle_max = angles[-1]
    msg.range_min = range_min
    msg.range_max = 30.0
    msg.ranges = ranges
    return msg


class TestScanCallbackFrontCone:

    def test_front_min_reads_only_front_window_side_min_reads_only_side_window(self):
        behaviour = _construct()
        # Front-cone sample (angle 0) reads 1.5m; side sample (angle 90deg)
        # reads a much closer 0.3m -- if front_min picked up the side
        # reading (or vice versa) this test would catch it.
        scan = _fake_scan([
            (0.0, 1.5),
            (math.radians(90.0), 0.3),
        ])
        behaviour._scan_callback(scan)
        assert behaviour.latest_lidar_front_min == pytest.approx(1.5)
        assert behaviour.latest_lidar_min_in_window == pytest.approx(0.3)

    def test_update_trips_on_front_cone_alone(self):
        behaviour = _construct(front_distance_threshold=0.40, lidar_distance_threshold=0.20)
        scan = _fake_scan([
            (0.0, 0.30),                    # front: below 0.40 -- trips
            (math.radians(90.0), 5.0),      # side/rear: far, does not trip
        ])
        behaviour._scan_callback(scan)
        assert behaviour.update() == py_trees.common.Status.SUCCESS

    def test_update_does_not_trip_when_front_reading_is_above_threshold(self):
        behaviour = _construct(front_distance_threshold=0.40, lidar_distance_threshold=0.20)
        scan = _fake_scan([
            (0.0, 0.50),                    # front: above 0.40 -- clear
            (math.radians(90.0), 5.0),      # side/rear: far, clear
        ])
        behaviour._scan_callback(scan)
        assert behaviour.update() == py_trees.common.Status.FAILURE

    def test_update_fails_before_any_scan_received(self):
        # Same "no data yet must not mean tripped" guard IsBatteryLow's own
        # has_data reasoning uses -- see module docstring. No _scan_callback
        # call at all here.
        behaviour = _construct()
        assert behaviour.latest_lidar_front_min is None
        assert behaviour.update() == py_trees.common.Status.FAILURE

    def test_invalid_front_samples_are_filtered_out(self):
        behaviour = _construct()
        scan = _fake_scan([
            (math.radians(-2.0), float('nan')),
            (math.radians(-1.0), float('inf')),
            (0.0, 0.02),                    # below range_min (0.05) -- filtered
            (math.radians(1.0), 2.0),       # the only valid front sample
            (math.radians(2.0), -1.0),      # negative, well below range_min
        ], range_min=0.05)
        behaviour._scan_callback(scan)
        assert behaviour.latest_lidar_front_min == pytest.approx(2.0)

    def test_front_and_side_rear_windows_evaluated_independently_when_unequal(self):
        # front_cone_half_angle_deg (60) > lidar_blind_cone_half_angle_deg
        # (30) here -- deliberately unequal, to confirm _scan_callback
        # degrades gracefully (a sample in the resulting OVERLAP counts
        # toward both) rather than assuming the default equal-angle
        # partition, per _in_front_cone's own docstring.
        behaviour = _construct(
            front_cone_half_angle_deg=60.0, lidar_blind_cone_half_angle_deg=30.0)
        scan = _fake_scan([
            (math.radians(45.0), 0.7),  # inside BOTH windows (30 < 45 < 60)
        ])
        behaviour._scan_callback(scan)
        assert behaviour.latest_lidar_front_min == pytest.approx(0.7)
        assert behaviour.latest_lidar_min_in_window == pytest.approx(0.7)

    def test_no_camera_front_distance_subscription_remains(self):
        # Regression coverage for the camera -> lidar front-cone swap itself:
        # confirms the ZED-based front path was actually REMOVED from the
        # class, not just left unused. See module docstring / is_proximity_
        # too_close.py's own module docstring "Front check: camera -> lidar".
        behaviour = _construct()
        assert not hasattr(behaviour, 'front_distance_sub')
        assert not hasattr(behaviour, 'front_distance_topic')
        assert not hasattr(behaviour, 'latest_front_distance')
        assert not hasattr(behaviour, '_front_distance_callback')
        with pytest.raises(TypeError):
            IsProximityTooClose(front_distance_topic='/perception/front_distance')


if __name__ == '__main__':
    import sys
    sys.exit(pytest.main([__file__, '-v']))
