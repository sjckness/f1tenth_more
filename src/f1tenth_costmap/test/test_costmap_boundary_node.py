"""costmap_boundary_node.py tests -- constructs a real (but never spun)
rclpy Node and calls its callbacks/_extraction_tick() directly, same "no
live topics/hardware needed, pure in-process method calls" convention
established this session by f1tenth_diagnostics/test/test_slam_pose_
covariance_calibration_node.py (see that file's own module docstring).
Publishers are never actually sent over DDS here (no spin() anywhere in
this file) -- self.boundary_pub.publish/self.clearance_pub.publish are
directly mocked so each test can assert exactly what would have been
published without needing a live subscriber or `ros2 topic echo`.

periodic-publish pass: the two previous staleness tests (test_stale_map_
publishes_empty_even_with_fresh_pose / test_stale_pose_publishes_empty_
even_with_fresh_map) tested a mechanism that no longer exists --
map_stale_timeout_sec/pose_stale_timeout_sec are gone, staleness no longer
gates the publish at all (see costmap_boundary_node.py's own module
docstring's "PERIODIC-PUBLISH pass" paragraph for why: front_clearance
published exactly once, coincident with /slam/pose's own single publish,
was the actual live bug this closes). Replaced by
TestPeriodicPublishRegardlessOfAge below, which directly demonstrates the
fix: manipulating _map_last_time/_pose_last_time (the exact bookkeeping
that used to trigger the stale path) now has NO effect on whether a tick
publishes real data. TestFailSafeNeverReceived below covers the fail-safe
condition that DOES still apply -- a message of that kind never having
arrived at all, as opposed to merely being old.

Run standalone: python3 -m pytest test/test_costmap_boundary_node.py -v
"""

import math
from unittest.mock import MagicMock

import pytest
import rclpy
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.parameter import Parameter

from f1tenth_costmap.costmap_boundary_node import CostmapBoundaryNode


@pytest.fixture(scope='module', autouse=True)
def _rclpy_context():
    rclpy.init()
    yield
    if rclpy.ok():
        rclpy.shutdown()


def _construct_with_params(param_dict=None):
    overrides = [Parameter(k, value=v) for k, v in (param_dict or {}).items()]
    node = CostmapBoundaryNode(parameter_overrides=overrides)
    # Never actually sent over DDS in this test file (no spin()) -- mocked
    # so tests can assert on exactly what WOULD have been published.
    node.boundary_pub.publish = MagicMock()
    node.clearance_pub.publish = MagicMock()
    return node


def _fake_grid(width, height, resolution, origin_x, origin_y, data):
    msg = OccupancyGrid()
    msg.info.width = width
    msg.info.height = height
    msg.info.resolution = resolution
    msg.info.origin.position.x = origin_x
    msg.info.origin.position.y = origin_y
    msg.data = list(data)
    return msg


def _fake_pose(x, y, yaw):
    msg = Odometry()
    msg.pose.pose.position.x = x
    msg.pose.pose.position.y = y
    msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
    msg.pose.pose.orientation.w = math.cos(yaw / 2.0)
    return msg


class TestFailSafeNeverReceived:
    """periodic-publish pass: the ONLY remaining fail-safe condition -- a
    message of that kind has never arrived at all yet. Not staleness (see
    module docstring's "PERIODIC-PUBLISH pass" paragraph and
    TestPeriodicPublishRegardlessOfAge below for the mechanism this
    replaced)."""

    def test_no_data_received_publishes_empty_and_withholds_clearance(self):
        node = _construct_with_params()
        try:
            node._extraction_tick()

            node.boundary_pub.publish.assert_called_once()
            published_arr = node.boundary_pub.publish.call_args[0][0]
            assert list(published_arr.constraints) == []
            # front_clearance must NOT be published at all this tick.
            node.clearance_pub.publish.assert_not_called()
        finally:
            node.destroy_node()

    def test_map_never_received_publishes_empty_even_with_pose_present(self):
        node = _construct_with_params()
        try:
            node._pose_cb(_fake_pose(2.5, 5.5, 0.0))
            # _map_cb deliberately never called.

            node._extraction_tick()

            published_arr = node.boundary_pub.publish.call_args[0][0]
            assert list(published_arr.constraints) == []
            node.clearance_pub.publish.assert_not_called()
        finally:
            node.destroy_node()

    def test_pose_never_received_publishes_empty_even_with_map_present(self):
        node = _construct_with_params()
        try:
            node._map_cb(_fake_grid(10, 10, 1.0, 0.0, 0.0, [0] * 100))
            # _pose_cb deliberately never called.

            node._extraction_tick()

            published_arr = node.boundary_pub.publish.call_args[0][0]
            assert list(published_arr.constraints) == []
            node.clearance_pub.publish.assert_not_called()
        finally:
            node.destroy_node()


class TestPeriodicPublishRegardlessOfAge:
    """Would have caught the actual live bug directly: front_clearance
    published exactly once, coincident with /slam/pose's own single
    publish, because the old code re-evaluated staleness against
    _map_last_time/_pose_last_time on every tick. Demonstrates the fix by
    using that EXACT same age-injection technique the old staleness tests
    used (pushing the bookkeeping timestamps far into the past) and
    confirming it now has zero effect -- every tick still publishes real
    data, over and over, from the same never-refreshed cached inputs.
    /slam/pose going silent at rest is normal (see the separate hypothesis-3
    investigation this session), so this is the realistic case, not a
    contrived one."""

    def test_repeated_ticks_with_aged_static_inputs_all_publish_real_data(self):
        node = _construct_with_params()
        try:
            grid_data = [0] * 100
            grid_data[5 * 10 + 6] = 100  # occupied cell, straight ahead of (2.5, 5.5)
            node._map_cb(_fake_grid(10, 10, 1.0, 0.0, 0.0, grid_data))
            node._pose_cb(_fake_pose(2.5, 5.5, 0.0))
            # Simulate a long time having passed since the last real update
            # on EITHER input -- the exact condition that used to trigger
            # the stale/withhold path. Neither input is ever re-cached
            # after this point.
            node._map_last_time -= 3600.0
            node._pose_last_time -= 3600.0

            n_ticks = 10
            for _ in range(n_ticks):
                node._extraction_tick()

            assert node.boundary_pub.publish.call_count == n_ticks
            assert node.clearance_pub.publish.call_count == n_ticks
            for call in node.boundary_pub.publish.call_args_list:
                arr = call[0][0]
                assert len(arr.constraints) == 1
                assert arr.constraints[0].normal[0] == pytest.approx(1.0)
                assert arr.constraints[0].offset == pytest.approx(4.0)
            for call in node.clearance_pub.publish.call_args_list:
                assert call[0][0].data == pytest.approx(4.0)
        finally:
            node.destroy_node()


class TestExtractionTimerConfig:
    """Confirms extraction_rate_hz actually drives the timer's own period
    (the param existed before this pass too -- this specifically guards the
    default bump 5.0 -> 20.0 and that overriding the param still reaches
    the timer, not just the extraction-vs-publish decision logic above)."""

    def test_default_rate_is_20hz(self):
        node = _construct_with_params()
        try:
            expected_period_ns = round(1.0 / 20.0 * 1e9)
            assert node.timer.timer_period_ns == expected_period_ns
        finally:
            node.destroy_node()

    def test_rate_param_override_reaches_timer(self):
        node = _construct_with_params({'extraction_rate_hz': 8.0})
        try:
            expected_period_ns = round(1.0 / 8.0 * 1e9)
            assert node.timer.timer_period_ns == expected_period_ns
        finally:
            node.destroy_node()


class TestExtractionTickFreshData:

    def test_fresh_map_and_pose_extracts_and_publishes_real_constraints(self):
        node = _construct_with_params()
        try:
            grid_data = [0] * 100
            grid_data[5 * 10 + 6] = 100  # occupied cell, straight ahead of (2.5, 5.5)
            node._map_cb(_fake_grid(10, 10, 1.0, 0.0, 0.0, grid_data))
            node._pose_cb(_fake_pose(2.5, 5.5, 0.0))

            node._extraction_tick()

            node.boundary_pub.publish.assert_called_once()
            published_arr = node.boundary_pub.publish.call_args[0][0]
            assert len(published_arr.constraints) == 1
            c = published_arr.constraints[0]
            assert c.normal[0] == pytest.approx(1.0)
            assert c.offset == pytest.approx(4.0)

            node.clearance_pub.publish.assert_called_once()
            published_clearance = node.clearance_pub.publish.call_args[0][0]
            assert published_clearance.data == pytest.approx(4.0)
        finally:
            node.destroy_node()

    def test_fresh_but_nothing_occupied_still_publishes_finite_clearance(self):
        """Distinct from the staleness tests above: fresh (non-stale) data
        that simply has no occupied cell in range IS a real, positive
        measurement ("clear as far as this map knows") -- front_clearance
        IS published here, unlike the stale-data case where it's withheld
        entirely. See costmap_boundary.py's own front_clearance_from_
        extraction docstring for why this is a finite value, not math.inf."""
        node = _construct_with_params()
        try:
            node._map_cb(_fake_grid(10, 10, 1.0, 0.0, 0.0, [0] * 100))
            node._pose_cb(_fake_pose(2.5, 5.5, 0.0))

            node._extraction_tick()

            published_arr = node.boundary_pub.publish.call_args[0][0]
            assert list(published_arr.constraints) == []
            node.clearance_pub.publish.assert_called_once()
            published_clearance = node.clearance_pub.publish.call_args[0][0]
            assert math.isfinite(published_clearance.data)
            assert published_clearance.data > node.max_range_m
        finally:
            node.destroy_node()


class TestCallbacksUpdateBookkeeping:

    def test_map_cb_stores_grid_and_receipt_time(self):
        node = _construct_with_params()
        try:
            assert node._latest_grid is None
            assert node._map_last_time is None
            node._map_cb(_fake_grid(4, 4, 1.0, 0.0, 0.0, [0] * 16))
            assert node._latest_grid is not None
            assert node._map_last_time is not None
        finally:
            node.destroy_node()

    def test_pose_cb_stores_xy_yaw_and_receipt_time(self):
        node = _construct_with_params()
        try:
            assert node._latest_pose_xy_yaw is None
            node._pose_cb(_fake_pose(1.0, 2.0, math.radians(30.0)))
            assert node._latest_pose_xy_yaw is not None
            x, y, yaw = node._latest_pose_xy_yaw
            assert x == pytest.approx(1.0)
            assert y == pytest.approx(2.0)
            assert yaw == pytest.approx(math.radians(30.0))
            assert node._pose_last_time is not None
        finally:
            node.destroy_node()


if __name__ == '__main__':
    import sys
    sys.exit(pytest.main([__file__, '-v']))
