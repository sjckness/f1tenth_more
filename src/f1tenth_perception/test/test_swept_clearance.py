"""swept_clearance.py tests -- sensor adapters, steering estimate and fusion, no rclpy.

Run standalone: python3 -m pytest test/test_swept_clearance.py -v
"""

import math

import numpy as np
import pytest

from f1tenth_perception.swept_clearance import (
    STEERING_ENVELOPE,
    STEERING_LAGGED,
    STEERING_LATEST,
    SensorReading,
    SteeringHistory,
    depth_to_points,
    fuse,
    quaternion_to_rotation,
    scan_to_points,
)

IDENTITY = np.eye(3)
LASER_T = np.array([0.12, 0.0, 0.20])
# base_link <- ZED optical frame: optical z forward, x right, y down. The same
# matrix /tf_static carries for zed2_left_camera_frame -> ..._optical_frame.
OPTICAL = np.array([[0.0, 0.0, 1.0], [-1.0, 0.0, 0.0], [0.0, -1.0, 0.0]])
CAMERA_T = np.array([0.12, 0.0, 0.15])


# ---------------------------------------------------------------------------
# LiDAR
# ---------------------------------------------------------------------------

class TestScanToPoints:

    def test_drops_inf_nan_and_returns_outside_the_valid_range(self):
        ranges = [1.0, math.inf, math.nan, 0.01, 40.0, 2.0]
        points = scan_to_points(ranges, 0.0, 0.0, 0.02, 30.0, IDENTITY, np.zeros(3))
        np.testing.assert_allclose(points, [[1.0, 0.0], [2.0, 0.0]])

    def test_keeps_the_full_circle_including_straight_behind(self):
        angles = np.array([-math.pi, -math.pi / 2, 0.0, math.pi / 2])
        points = scan_to_points(np.ones(4), angles[0], math.pi / 2, 0.02, 30.0,
                                IDENTITY, np.zeros(3))
        np.testing.assert_allclose(points, [[-1.0, 0.0], [0.0, -1.0], [1.0, 0.0], [0.0, 1.0]],
                                   atol=1e-12)

    def test_applies_the_static_translation(self):
        points = scan_to_points([1.0], 0.0, 0.0, 0.02, 30.0, IDENTITY, LASER_T)
        np.testing.assert_allclose(points, [[1.12, 0.0]])

    def test_a_rear_facing_mount_puts_its_forward_beam_behind_the_car(self):
        yaw_pi = quaternion_to_rotation(0.0, 0.0, 1.0, 0.0)
        points = scan_to_points([1.0], 0.0, 0.0, 0.02, 30.0, yaw_pi, np.array([-0.12, 0.0, 0.15]))
        np.testing.assert_allclose(points, [[-1.12, 0.0]], atol=1e-12)


# ---------------------------------------------------------------------------
# Camera depth
# ---------------------------------------------------------------------------

FX = FY = 300.0
WIDTH, HEIGHT = 64, 48
CX, CY = WIDTH / 2.0, HEIGHT / 2.0


def _depth_points(depth, **overrides):
    options = dict(stride=1, min_depth=0.2, max_depth=6.0, z_min=-10.0, z_max=10.0,
                   max_points=0)
    options.update(overrides)
    return depth_to_points(depth, FX, FY, CX, CY, OPTICAL, CAMERA_T, **options)


class TestDepthToPoints:

    def test_a_fronto_parallel_wall_lands_at_its_depth_ahead_of_the_camera(self):
        points = _depth_points(np.full((HEIGHT, WIDTH), 2.0))
        assert points.shape == (HEIGHT * WIDTH, 2)
        np.testing.assert_allclose(points[:, 0], 2.12)
        # Columns right of the principal point are to the car's right (-y).
        right_column = points.reshape(HEIGHT, WIDTH, 2)[:, -1, 1]
        np.testing.assert_allclose(right_column, -(WIDTH - 1 - CX) * 2.0 / FX)

    def test_the_height_band_drops_floor_and_overhead_pixels(self):
        depth = np.full((HEIGHT, WIDTH), 2.0)
        heights = CAMERA_T[2] - (np.arange(HEIGHT) - CY) * 2.0 / FY
        kept = _depth_points(depth, z_min=0.08, z_max=0.35)
        expected_rows = int(np.count_nonzero((heights >= 0.08) & (heights <= 0.35)))
        assert 0 < expected_rows < HEIGHT
        assert kept.shape[0] == expected_rows * WIDTH

    def test_minus_inf_is_placed_at_min_depth_and_nan_or_plus_inf_is_dropped(self):
        depth = np.full((HEIGHT, WIDTH), np.nan)
        depth[int(CY), int(CX)] = -np.inf
        depth[int(CY), int(CX) + 1] = np.inf
        points = _depth_points(depth)
        np.testing.assert_allclose(points, [[0.12 + 0.2, 0.0]])

    def test_depths_beyond_max_depth_or_not_positive_are_dropped(self):
        depth = np.full((HEIGHT, WIDTH), 7.0)
        depth[0, 0] = 0.0
        depth[0, 1] = -1.0
        assert _depth_points(depth).shape == (0, 2)

    def test_stride_samples_every_nth_pixel_at_its_true_coordinates(self):
        depth = np.full((HEIGHT, WIDTH), 1.0)
        points = _depth_points(depth, stride=4)
        assert points.shape[0] == (HEIGHT // 4) * (WIDTH // 4)
        np.testing.assert_allclose(points[:, 1].max(), (CX - 0) * 1.0 / FX)

    def test_max_points_caps_the_count(self):
        points = _depth_points(np.full((HEIGHT, WIDTH), 1.0), max_points=100)
        assert 0 < points.shape[0] <= 100


class TestQuaternion:

    def test_identity_and_a_quarter_turn_about_z(self):
        np.testing.assert_allclose(quaternion_to_rotation(0, 0, 0, 1), IDENTITY)
        quarter = quaternion_to_rotation(0.0, 0.0, math.sin(math.pi / 4), math.cos(math.pi / 4))
        np.testing.assert_allclose(quarter @ [1.0, 0.0, 0.0], [0.0, 1.0, 0.0], atol=1e-12)

    def test_zero_quaternion_is_rejected(self):
        with pytest.raises(ValueError):
            quaternion_to_rotation(0, 0, 0, 0)


# ---------------------------------------------------------------------------
# Steering
# ---------------------------------------------------------------------------

class TestSteeringHistory:

    def _history(self, commands, lag=0.3):
        history = SteeringHistory(lag)
        for t, value in commands:
            history.add(t, value)
        return history

    def test_lagged_is_the_command_in_effect_lag_seconds_ago(self):
        history = self._history([(0.0, 0.0), (0.1, 0.1), (0.2, 0.2), (0.3, 0.278)])
        assert history.lagged(0.45) == 0.1
        assert history.latest() == 0.278

    def test_until_a_command_is_lag_old_the_oldest_held_one_is_used(self):
        history = self._history([(10.0, -0.2), (10.1, 0.278)])
        assert history.lagged(10.15) == -0.2

    def test_no_command_means_no_estimate(self):
        history = SteeringHistory(0.3)
        assert history.lagged(1.0) is None
        assert history.latest() is None
        assert history.angles(1.0, STEERING_LAGGED) == []

    def test_modes_select_lagged_latest_or_lagged_plus_commands_in_transit(self):
        history = self._history([(0.0, 0.0), (0.2, 0.1), (0.3, 0.278)])
        assert history.angles(0.35, STEERING_LAGGED) == [0.0]
        assert history.angles(0.35, STEERING_LATEST) == [0.278]
        assert history.angles(0.35, STEERING_ENVELOPE) == [0.0, 0.1, 0.278]
        with pytest.raises(ValueError):
            history.angles(0.35, 'newest')

    def test_old_commands_are_pruned_but_the_one_in_effect_is_kept(self):
        history = self._history([(t / 10.0, t / 100.0) for t in range(50)])
        assert len(history) <= 5
        assert history.lagged(4.9) == pytest.approx(0.46)

    def test_non_finite_commands_are_ignored(self):
        history = self._history([(0.0, 0.1)])
        assert history.add(0.1, math.nan) is False
        assert history.latest() == 0.1

    def test_a_clock_jumping_backwards_restarts_the_history(self):
        history = self._history([(100.0, 0.2), (0.5, -0.1)])
        assert len(history) == 1
        assert history.lagged(1.0) == -0.1

    def test_negative_lag_is_rejected(self):
        with pytest.raises(ValueError):
            SteeringHistory(-0.1)


# ---------------------------------------------------------------------------
# Fusion
# ---------------------------------------------------------------------------

class TestFuse:

    def test_both_fresh_gives_the_smaller(self):
        value, stale = fuse(10.0, [SensorReading('lidar', 9.95, 1.4, 0.25),
                                   SensorReading('camera', 9.8, 0.9, 0.5)])
        assert value == 0.9
        assert stale == []

    def test_a_stale_sensor_is_left_out_not_taken_as_clear_or_blocked(self):
        value, stale = fuse(10.0, [SensorReading('lidar', 9.95, 1.4, 0.25),
                                   SensorReading('camera', 9.0, 0.3, 0.5)])
        assert value == 1.4
        assert stale == ['camera']

    def test_a_sensor_that_never_reported_is_stale(self):
        value, stale = fuse(10.0, [SensorReading('lidar', None, None, 0.25),
                                   SensorReading('camera', 9.9, 2.0, 0.5)])
        assert value == 2.0
        assert stale == ['lidar']

    def test_both_stale_is_zero(self):
        value, stale = fuse(10.0, [SensorReading('lidar', 1.0, 4.0, 0.25),
                                   SensorReading('camera', None, None, 0.5)])
        assert value == 0.0
        assert stale == ['lidar', 'camera']

    def test_no_sensor_is_zero(self):
        assert fuse(10.0, []) == (0.0, [])

    def test_exactly_at_the_timeout_is_still_fresh(self):
        assert fuse(10.0, [SensorReading('lidar', 9.75, 1.0, 0.25)]) == (1.0, [])
