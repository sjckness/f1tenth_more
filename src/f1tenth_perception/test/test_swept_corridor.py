"""swept_corridor.py tests -- the geometry alone, no rclpy.

The seven cases the swept-clearance work order requires come first, then the
properties they rest on: the closed form agrees with rotating the footprint
step by step, it reduces to the work order's point-body sketch, the band comes
from the footprint, the horizon never wraps, and the car never sees itself.

Run standalone: python3 -m pytest test/test_swept_corridor.py -v
"""

import math

import numpy as np
import pytest

from f1tenth_params.param_defaults import get_value
from f1tenth_perception.swept_corridor import (
    BODY_FRONT_X_M,
    BODY_HALF_WIDTH_M,
    BODY_REAR_X_M,
    STRAIGHT_CURVATURE,
    clearance,
    corridor_bounds,
    first_contact_arc_length,
)

WHEELBASE = 0.305
MARGIN = 0.10
MAX_RANGE = 5.0
FLOOR = 0.15
FULL_LOCK = get_value('mpc_steering_angle_max_rad')
CORRIDOR_HALF_WIDTH = BODY_HALF_WIDTH_M + MARGIN

GEOMETRY = dict(wheelbase=WHEELBASE, half_width=BODY_HALF_WIDTH_M, margin=MARGIN,
                max_range=MAX_RANGE, absolute_min_clearance=FLOOR)


def _clearance(points, delta, **overrides):
    return clearance(np.asarray(points, dtype=float), delta, **{**GEOMETRY, **overrides})


def _wall(start, end, spacing=0.005):
    start, end = np.asarray(start, dtype=float), np.asarray(end, dtype=float)
    n = int(math.ceil(np.linalg.norm(end - start) / spacing)) + 1
    return start + np.linspace(0.0, 1.0, n)[:, None] * (end - start)


def _radius(delta):
    return WHEELBASE / math.tan(delta)


# ---------------------------------------------------------------------------
# The seven required cases
# ---------------------------------------------------------------------------

class TestRequiredCases:

    def test_1_straight_wall_two_metres_ahead_of_the_bumper_reads_two_metres(self):
        wall = _wall((BODY_FRONT_X_M + 2.0, -2.0), (BODY_FRONT_X_M + 2.0, 2.0))
        assert _clearance(wall, 0.0) == pytest.approx(2.0, abs=1e-9)

    def test_2_straight_wall_two_metres_ahead_but_beside_the_corridor_reads_max_range(self):
        wall = _wall((BODY_FRONT_X_M + 2.0, CORRIDOR_HALF_WIDTH + 0.01),
                     (BODY_FRONT_X_M + 2.0, 2.0))
        assert _clearance(wall, 0.0) == MAX_RANGE
        assert _clearance(wall * [1.0, -1.0], 0.0) == MAX_RANGE

    def test_3_full_left_lock_ignores_a_right_wall_the_straight_corridor_brakes_for(self):
        # Runs ahead-right into the straight corridor about 0.7 m past the
        # bumper -- the fixed forward check's false brake -- while the left-
        # lock arc stays inside radius 1.38 m of its turn centre.
        wall = _wall((0.6, -0.6), (2.0, 0.3))
        assert _clearance(wall, 0.0) < 1.0
        assert _clearance(wall, FULL_LOCK) == MAX_RANGE

    def test_4_full_left_lock_sees_an_inside_wall_the_straight_corridor_misses(self):
        wall = _wall((0.0, 0.6), (3.0, 0.6))
        assert _clearance(wall, 0.0) == MAX_RANGE
        turning = _clearance(wall, FULL_LOCK)
        assert 0.0 < turning < 1.0
        # The body's inner front corner gets there well before the rear axle
        # would: the axle's own arc reaches y = 0.6 only after R * acos(1 - 0.6 / R).
        rho = _radius(FULL_LOCK)
        assert turning < rho * math.acos(1.0 - 0.6 / rho) - 0.3

    def test_5_clearance_is_continuous_through_straight_ahead(self):
        # A front wall plus two long side walls, fixed, 5 mm spacing. Walls
        # long enough that no contact can vanish by running off an end.
        cloud = np.vstack([_wall((2.5, -6.0), (2.5, 6.0)),
                           _wall((-1.0, 1.2), (6.0, 1.2)),
                           _wall((-1.0, -1.2), (6.0, -1.2))])
        guard = math.atan(STRAIGHT_CURVATURE * WHEELBASE)
        special = [0.0, guard, -guard, 1e-2, -1e-2]
        deltas = np.unique(np.concatenate([
            np.linspace(-0.28, 0.28, 1121),
            [d * (1.0 + e) for d in special for e in (-1e-6, 0.0, 1e-6)],
        ]))
        values = np.array([_clearance(cloud, d) for d in deltas])

        # Across the whole sweep: bounded by the point spacing, not a jump.
        assert np.abs(np.diff(values)).max() < 0.02

        # At the kappa = 0 switch, and at the 1e-2 rad the work order named,
        # both sides agree to well under a millimetre.
        for delta in (guard, -guard, 1e-2, -1e-2):
            below = _clearance(cloud, delta * (1.0 - 1e-6))
            above = _clearance(cloud, delta * (1.0 + 1e-6))
            assert abs(above - below) < 1e-6, delta
        assert _clearance(cloud, guard * 1.001) == pytest.approx(_clearance(cloud, 0.0), abs=1e-6)

    @pytest.mark.parametrize('delta', np.linspace(-0.28, 0.28, 9))
    def test_6_nothing_behind_the_car_is_forward_clearance(self, delta):
        just_behind_the_bumper = [BODY_REAR_X_M - 0.05, 0.0]   # inside the floor distance
        assert _clearance([just_behind_the_bumper, [-1.0, 0.0], [-3.0, 0.0]], delta) == MAX_RANGE
        # A whole wall across the back, including where it crosses the far
        # side of the turn circle.
        assert _clearance(_wall((-0.3, -3.0), (-0.3, 3.0)), delta) == MAX_RANGE

    @pytest.mark.parametrize('delta', [0.0, FULL_LOCK])
    def test_7_a_point_inside_the_floor_outside_the_corridor_returns_its_gap(self, delta):
        # Beside the right front wheel, just outside the corridor, with the
        # car going straight or turning away from it.
        beside_the_front_wheel = [0.30, -(CORRIDOR_HALF_WIDTH + 0.01)]
        gap = CORRIDOR_HALF_WIDTH + 0.01 - BODY_HALF_WIDTH_M
        assert gap < FLOOR
        assert _clearance([beside_the_front_wheel], delta, absolute_min_clearance=0.0) == MAX_RANGE
        assert _clearance([beside_the_front_wheel], delta) == pytest.approx(gap)

    def test_7b_turning_toward_that_point_the_flank_reaches_it_sooner_than_the_floor(self):
        beside_the_front_wheel = [0.30, -(CORRIDOR_HALF_WIDTH + 0.01)]
        gap = CORRIDOR_HALF_WIDTH + 0.01 - BODY_HALF_WIDTH_M
        assert _clearance([beside_the_front_wheel], -FULL_LOCK) < gap


# ---------------------------------------------------------------------------
# The closed form against brute force
# ---------------------------------------------------------------------------

def _covered(x, y, delta, theta):
    """Whether the margin-widened leading half covers (x, y) after the car
    has turned by theta (array) about its turn centre."""
    rho = _radius(delta)
    c, s = np.cos(-math.copysign(1.0, delta) * theta), np.sin(-math.copysign(1.0, delta) * theta)
    # The point as seen from the car after the turn.
    rx = c * x - s * (y - rho)
    ry = s * x + c * (y - rho) + rho
    return (rx >= 0.0) & (rx <= BODY_FRONT_X_M) & (np.abs(ry) <= CORRIDOR_HALF_WIDTH)


class TestAgainstBruteForce:

    @pytest.mark.parametrize('delta', [FULL_LOCK, -FULL_LOCK, 0.42, -0.1, 0.03])
    def test_first_contact_matches_rotating_the_footprint(self, delta):
        rng = np.random.default_rng(7)
        points = rng.uniform([0.0, -3.0], [4.0, 3.0], size=(300, 2))
        closed = first_contact_arc_length(points[:, 0], points[:, 1], delta, wheelbase=WHEELBASE,
                                          half_width=BODY_HALF_WIDTH_M, margin=MARGIN)
        rho = abs(_radius(delta))
        theta = np.linspace(0.0, math.pi, 8000)
        tolerance = 2.0 * (theta[1] - theta[0]) * rho
        for (x, y), c in zip(points, closed):
            covered = _covered(x, y, delta, theta)
            brute = theta[np.argmax(covered)] * rho if covered.any() else -np.inf
            if brute > tolerance:
                assert c == pytest.approx(brute, abs=tolerance), (x, y)
            elif c > tolerance:
                # A graze of the outer front corner can be shorter than one
                # brute-force step; then check just after the claimed contact.
                assert brute == -np.inf, (x, y, c, brute)
                assert _covered(x, y, delta, np.array([c / rho + 1e-9])).all(), (x, y, c)


class TestReducesToTheWorkOrderSketch:
    """With a zero-length body alpha is 0, and the arc length is exactly
    |R| * atan2(p.x, sign(R) * (R - p.y)) inside |R| -+ (half_width + margin)."""

    POINT_BODY = dict(front_x=0.0, rear_x=0.0, absolute_min_clearance=0.0)

    def test_straight_is_min_x_inside_the_half_width(self):
        points = [[3.0, 0.2], [2.0, -CORRIDOR_HALF_WIDTH], [1.0, CORRIDOR_HALF_WIDTH + 1e-6]]
        assert _clearance(points, 0.0, **self.POINT_BODY) == pytest.approx(2.0)

    @pytest.mark.parametrize('delta', [FULL_LOCK, -FULL_LOCK, 0.1])
    def test_curved_is_radius_times_the_swept_angle(self, delta):
        big_r = _radius(delta)
        rng = np.random.default_rng(3)
        points = rng.uniform([0.0, -2.5], [2.5, 2.5], size=(400, 2))
        d = np.hypot(points[:, 0], points[:, 1] - big_r)
        in_band = (d >= abs(big_r) - CORRIDOR_HALF_WIDTH) & (d <= abs(big_r) + CORRIDOR_HALF_WIDTH)
        sign = math.copysign(1.0, big_r)
        sketch = abs(big_r) * np.arctan2(points[:, 0], sign * (big_r - points[:, 1]))
        s = first_contact_arc_length(points[:, 0], points[:, 1], delta, wheelbase=WHEELBASE,
                                     half_width=BODY_HALF_WIDTH_M, margin=MARGIN, front_x=0.0)
        np.testing.assert_allclose(s[in_band], sketch[in_band], atol=1e-9)
        assert np.all(np.isneginf(s[~in_band]))


# ---------------------------------------------------------------------------
# The band comes from the footprint
# ---------------------------------------------------------------------------

def _leading_half_outline(n=2000):
    """Dense samples of the margin-widened leading half's boundary, in (x, y)."""
    xs = np.linspace(0.0, BODY_FRONT_X_M, n)
    ys = np.linspace(-CORRIDOR_HALF_WIDTH, CORRIDOR_HALF_WIDTH, n)
    return np.vstack([np.stack([xs, np.full(n, CORRIDOR_HALF_WIDTH)], 1),
                      np.stack([xs, np.full(n, -CORRIDOR_HALF_WIDTH)], 1),
                      np.stack([np.full(n, BODY_FRONT_X_M), ys], 1),
                      np.stack([np.zeros(n), ys], 1)])


class TestFootprintBand:

    @pytest.mark.parametrize('delta', [FULL_LOCK, -FULL_LOCK, 0.05])
    def test_inner_and_outer_are_the_nearest_and_farthest_footprint_points(self, delta):
        bounds = corridor_bounds(delta, wheelbase=WHEELBASE, half_width=BODY_HALF_WIDTH_M,
                                 margin=MARGIN)
        outline = _leading_half_outline()
        radii = np.hypot(outline[:, 0], outline[:, 1] - bounds.radius)
        assert bounds.inner == pytest.approx(radii.min(), abs=1e-4)
        assert bounds.outer == pytest.approx(radii.max(), abs=1e-4)

    def test_the_outer_front_corner_swings_past_radius_plus_half_width(self):
        bounds = corridor_bounds(FULL_LOCK, wheelbase=WHEELBASE, half_width=BODY_HALF_WIDTH_M,
                                 margin=MARGIN)
        assert bounds.outer - (abs(bounds.radius) + CORRIDOR_HALF_WIDTH) > 0.05

    def test_the_inside_rear_corner_is_farther_from_the_centre_than_the_inner_bound(self):
        # The turn centre is on the rear-axle line, so the corner behind it
        # sweeps a LARGER radius than the flank level with the axle.
        bounds = corridor_bounds(FULL_LOCK, wheelbase=WHEELBASE, half_width=BODY_HALF_WIDTH_M,
                                 margin=MARGIN)
        inside_rear_corner = math.hypot(BODY_REAR_X_M, abs(bounds.radius) - CORRIDOR_HALF_WIDTH)
        assert inside_rear_corner > bounds.inner

    def test_straight_bounds_are_unbounded_radii(self):
        bounds = corridor_bounds(0.0, wheelbase=WHEELBASE, half_width=BODY_HALF_WIDTH_M,
                                 margin=MARGIN)
        assert bounds.curvature == 0.0
        assert math.isinf(bounds.radius) and math.isinf(bounds.inner) and math.isinf(bounds.outer)

    def test_radius_sign_follows_the_steering_sign(self):
        left = corridor_bounds(FULL_LOCK, wheelbase=WHEELBASE, half_width=BODY_HALF_WIDTH_M,
                               margin=MARGIN)
        right = corridor_bounds(-FULL_LOCK, wheelbase=WHEELBASE, half_width=BODY_HALF_WIDTH_M,
                                margin=MARGIN)
        assert left.radius > 0.0 > right.radius
        assert left.radius == pytest.approx(-right.radius)
        assert left.radius == pytest.approx(_radius(FULL_LOCK))


# ---------------------------------------------------------------------------
# Horizon, self, symmetry, range and inputs
# ---------------------------------------------------------------------------

class TestHorizon:

    def test_a_point_reached_only_past_the_rear_axle_line_is_not_returned(self):
        rho = _radius(FULL_LOCK)
        far_side_just_behind = [-0.05, 2.0 * rho]
        assert _clearance([far_side_just_behind], FULL_LOCK) == MAX_RANGE

    def test_a_quarter_turn_point_is_returned_before_a_quarter_turn_of_travel(self):
        rho = _radius(FULL_LOCK)
        value = _clearance([[rho, rho]], FULL_LOCK)
        assert 0.0 < value < rho * math.pi / 2.0

    def test_no_point_on_the_path_circle_is_reached_after_half_a_turn(self):
        rho = _radius(FULL_LOCK)
        angles = np.linspace(-math.pi, math.pi, 3601)
        ring = np.stack([rho * np.sin(angles), rho - rho * np.cos(angles)], 1)
        s = first_contact_arc_length(ring[:, 0], ring[:, 1], FULL_LOCK, wheelbase=WHEELBASE,
                                     half_width=BODY_HALF_WIDTH_M, margin=MARGIN)
        assert s[np.isfinite(s)].max() < math.pi * rho


class TestTheCarItself:

    def test_points_inside_the_body_are_ignored_by_corridor_and_floor(self):
        lidar_housing = [0.12, 0.0]
        chassis_corners = [[BODY_FRONT_X_M - 0.01, BODY_HALF_WIDTH_M - 0.01],
                           [BODY_REAR_X_M + 0.01, -BODY_HALF_WIDTH_M + 0.01]]
        for delta in (-FULL_LOCK, 0.0, FULL_LOCK):
            assert _clearance([lidar_housing, *chassis_corners], delta) == MAX_RANGE

    def test_a_point_touching_the_bumper_reads_zero(self):
        assert _clearance([[BODY_FRONT_X_M, 0.0]], 0.0) == pytest.approx(0.0, abs=1e-12)


class TestSymmetryAndRange:

    @pytest.mark.parametrize('delta', [0.05, FULL_LOCK])
    def test_mirroring_the_scene_and_the_steering_gives_the_same_clearance(self, delta):
        rng = np.random.default_rng(11)
        cloud = rng.uniform([-1.0, -3.0], [4.0, 3.0], size=(2000, 2))
        assert _clearance(cloud, delta) == pytest.approx(_clearance(cloud * [1.0, -1.0], -delta))

    def test_an_obstacle_beyond_max_range_along_the_path_reads_max_range(self):
        assert _clearance([[BODY_FRONT_X_M + MAX_RANGE + 0.5, 0.0]], 0.0) == MAX_RANGE

    def test_no_points_reads_max_range(self):
        assert _clearance(np.empty((0, 2)), FULL_LOCK) == MAX_RANGE
        assert _clearance([], 0.0) == MAX_RANGE

    def test_non_finite_points_are_skipped(self):
        points = [[np.nan, 0.0], [np.inf, 0.1], [BODY_FRONT_X_M + 1.0, 0.0]]
        assert _clearance(points, 0.0) == pytest.approx(1.0)

    def test_malformed_input_is_rejected(self):
        with pytest.raises(ValueError):
            _clearance(np.zeros((4, 3)), 0.0)
        with pytest.raises(ValueError):
            _clearance([[1.0, 0.0]], math.nan)
