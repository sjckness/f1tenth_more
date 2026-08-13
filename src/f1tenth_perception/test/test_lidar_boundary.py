"""lidar_boundary_node.py tests -- mirrors test_wall_detector.py's own
convention: static/synthetic only, no live hardware, no rclpy Node
instantiation required (every symbol under test is a plain function with
zero rclpy dependency; this imports the whole node module, which does
still import rclpy at module scope, but never constructs a Node).

Run standalone: python3 -m pytest test/test_lidar_boundary.py -v
"""

import math

import numpy as np
import pytest

from f1tenth_perception.lidar_boundary_node import (
    TrackedBoundaryLine,
    _classify_side,
    _scan_angle_to_car_frame,
    _scan_point_car_xy,
    fit_side_boundary,
    side_track_eligible,
    update_side_track,
)

WINDOW_MIN_RAD = math.radians(45.0)
WINDOW_MAX_RAD = math.radians(135.0)
MIN_INLIERS = 20
MAX_RESIDUAL_M = 0.05
OUTLIER_THRESHOLD_M = 0.05
EMA_ALPHA = 0.3
HOLD_FRAMES = 5
MIN_FRAMES_TO_PUBLISH = 5


# ==============================================================================
# Angle conversion -- the rear-facing-mount fact reused (not re-derived)
# from IsProximityTooClose, see module docstring.
# ==============================================================================

class TestScanAngleToCarFrame:

    def test_raw_zero_is_car_rear(self):
        # raw angle 0 = lidar's own forward axis = car's REAR (yaw=pi mount).
        assert _scan_angle_to_car_frame(0.0) == pytest.approx(math.pi)

    def test_raw_pi_is_car_front(self):
        assert abs(_scan_angle_to_car_frame(math.pi)) == pytest.approx(0.0, abs=1e-9)

    def test_raw_negative_pi_is_also_car_front(self):
        assert abs(_scan_angle_to_car_frame(-math.pi)) == pytest.approx(0.0, abs=1e-9)

    def test_stays_wrapped_into_valid_range(self):
        for raw in np.linspace(-math.pi, math.pi, 37):
            car = _scan_angle_to_car_frame(raw)
            assert -math.pi < car <= math.pi + 1e-9


class TestClassifySide:

    def test_front_cone_excluded(self):
        assert _classify_side(0.0, WINDOW_MIN_RAD, WINDOW_MAX_RAD) is None

    def test_rear_excluded(self):
        assert _classify_side(math.pi, WINDOW_MIN_RAD, WINDOW_MAX_RAD) is None
        assert _classify_side(-math.pi, WINDOW_MIN_RAD, WINDOW_MAX_RAD) is None

    def test_left_window(self):
        assert _classify_side(math.radians(90.0), WINDOW_MIN_RAD, WINDOW_MAX_RAD) == 'left'

    def test_right_window(self):
        assert _classify_side(math.radians(-90.0), WINDOW_MIN_RAD, WINDOW_MAX_RAD) == 'right'

    def test_window_edges_inclusive(self):
        assert _classify_side(WINDOW_MIN_RAD, WINDOW_MIN_RAD, WINDOW_MAX_RAD) == 'left'
        assert _classify_side(WINDOW_MAX_RAD, WINDOW_MIN_RAD, WINDOW_MAX_RAD) == 'left'
        assert _classify_side(-WINDOW_MIN_RAD, WINDOW_MIN_RAD, WINDOW_MAX_RAD) == 'right'
        assert _classify_side(-WINDOW_MAX_RAD, WINDOW_MIN_RAD, WINDOW_MAX_RAD) == 'right'

    def test_just_outside_windows_excluded(self):
        assert _classify_side(WINDOW_MIN_RAD - 0.01, WINDOW_MIN_RAD, WINDOW_MAX_RAD) is None
        assert _classify_side(WINDOW_MAX_RAD + 0.01, WINDOW_MIN_RAD, WINDOW_MAX_RAD) is None


def _run_windowing(raw_angle_range_pairs):
    """Mirrors lidar_boundary_node.py's own _scan_callback loop, using
    ONLY the pure functions under test -- no LaserScan message / Node
    needed. Returns (left_pts, right_pts), each a list of (x, y) car-frame
    points."""
    left_pts, right_pts = [], []
    for raw_angle, r in raw_angle_range_pairs:
        car_angle = _scan_angle_to_car_frame(raw_angle)
        side = _classify_side(car_angle, WINDOW_MIN_RAD, WINDOW_MAX_RAD)
        if side == 'left':
            left_pts.append(_scan_point_car_xy(car_angle, r))
        elif side == 'right':
            right_pts.append(_scan_point_car_xy(car_angle, r))
    return left_pts, right_pts


class TestWindowingPipeline:

    def test_front_blind_cone_points_excluded_from_both_sides(self):
        # raw angle +-pi is car-frame front (see TestScanAngleToCarFrame) --
        # feed a spread of raw angles all mapping into the front +-45deg
        # car-frame cone.
        front_raw_angles = [
            math.pi, math.pi - math.radians(30), math.pi + math.radians(30),
            -math.pi, -math.pi + math.radians(20),
        ]
        pairs = [(a, 1.0) for a in front_raw_angles]
        left_pts, right_pts = _run_windowing(pairs)
        assert left_pts == []
        assert right_pts == []

    def test_rear_points_also_excluded(self):
        # raw angle 0 is car-frame rear.
        rear_raw_angles = [0.0, math.radians(20), math.radians(-20)]
        pairs = [(a, 1.0) for a in rear_raw_angles]
        left_pts, right_pts = _run_windowing(pairs)
        assert left_pts == []
        assert right_pts == []

    def test_left_and_right_points_land_in_the_correct_bucket(self):
        # car-frame +90deg (left) -> raw = 90-180 = -90deg.
        left_raw = math.radians(90.0) - math.pi
        # car-frame -90deg (right) -> raw = -90-180 wrapped = 90deg.
        right_raw = math.radians(-90.0) - math.pi
        pairs = [(left_raw, 2.0), (right_raw, 3.0)]
        left_pts, right_pts = _run_windowing(pairs)
        assert len(left_pts) == 1
        assert len(right_pts) == 1
        # Left point should be roughly (x=0, y=+2) -- car-frame 90deg, +=left.
        assert left_pts[0] == pytest.approx((0.0, 2.0), abs=1e-6)
        assert right_pts[0] == pytest.approx((0.0, -3.0), abs=1e-6)


# ==============================================================================
# Robust line fit -- see module docstring's "Robust line fit" section.
# ==============================================================================

def _clean_line_points(x_offset=1.5, y_min=-1.0, y_max=1.0, n=60, seed=0):
    """Points along the line x = x_offset (car frame), y spanning
    [y_min, y_max], with tiny jitter so the fit is well-defined (not
    perfectly degenerate) -- same "real capture always has some noise"
    reasoning test_wall_detector.py's own _make_wall helper uses."""
    rng = np.random.default_rng(seed)
    y = np.linspace(y_min, y_max, n)
    x = x_offset + rng.uniform(-0.002, 0.002, size=n)
    return np.stack([x, y], axis=1)


class TestFitSideBoundary:

    def test_clean_line_extracts_correct_normal_and_offset(self):
        points = _clean_line_points(x_offset=1.5)
        result = fit_side_boundary(points, MIN_INLIERS, MAX_RESIDUAL_M, OUTLIER_THRESHOLD_M)
        assert result is not None
        nx, ny, offset = result
        # Line is x=1.5 -- normal must point AWAY from the origin, i.e.
        # toward +x, and offset must be the perpendicular distance (1.5).
        assert (nx, ny) == pytest.approx((1.0, 0.0), abs=0.02)
        assert offset == pytest.approx(1.5, abs=0.02)
        # Robot's own origin must land on the free-space side.
        assert nx * 0.0 + ny * 0.0 <= offset

    def test_sparse_points_rejected(self):
        points = _clean_line_points(x_offset=1.5, n=5)  # < MIN_INLIERS=20
        assert fit_side_boundary(points, MIN_INLIERS, MAX_RESIDUAL_M, OUTLIER_THRESHOLD_M) is None

    def test_scattered_garbage_points_rejected(self):
        rng = np.random.default_rng(1)
        # No coherent line at all -- points scattered over a wide 2D patch,
        # RMS residual from ANY single-line fit will be far above the
        # 0.05m gate.
        points = rng.uniform(-1.0, 1.0, size=(60, 2)) + np.array([1.5, 0.0])
        assert fit_side_boundary(points, MIN_INLIERS, MAX_RESIDUAL_M, OUTLIER_THRESHOLD_M) is None

    def test_outlier_rejection_recovers_a_clean_fit(self):
        # A clean line plus a handful of stray returns off it -- the
        # outlier-rejection pass should drop the strays and still recover
        # the correct fit, not get pulled off by them. Strays are
        # deliberately moderate (~0.2-0.3m off the line, matching a
        # realistic stray-return magnitude and clearing
        # OUTLIER_THRESHOLD_M=0.05 comfortably) rather than extreme --
        # this is a single-pass TLS + residual-threshold rejection (see
        # module docstring), not full RANSAC, so a handful of severe
        # leverage points far enough off to skew the INITIAL fit itself is
        # a known, accepted limitation of "basic" outlier rejection, not
        # what this test is checking.
        clean = _clean_line_points(x_offset=1.5, n=50)
        strays = np.array([[1.7, 0.9], [1.35, -0.9], [1.8, 0.0]])
        points = np.concatenate([clean, strays], axis=0)
        result = fit_side_boundary(points, MIN_INLIERS, MAX_RESIDUAL_M, OUTLIER_THRESHOLD_M)
        assert result is not None
        nx, ny, offset = result
        assert (nx, ny) == pytest.approx((1.0, 0.0), abs=0.02)
        assert offset == pytest.approx(1.5, abs=0.02)

    def test_normal_sign_flips_for_a_line_on_the_negative_side(self):
        # Line at x = -1.2 -- normal must point toward -x (away from the
        # origin, toward the line), not +x.
        points = _clean_line_points(x_offset=-1.2, n=40)
        result = fit_side_boundary(points, MIN_INLIERS, MAX_RESIDUAL_M, OUTLIER_THRESHOLD_M)
        assert result is not None
        nx, ny, offset = result
        assert (nx, ny) == pytest.approx((-1.0, 0.0), abs=0.02)
        assert offset == pytest.approx(1.2, abs=0.02)


# ==============================================================================
# Temporal tracking -- see module docstring's "Temporal tracking" section.
# Added by the "Boundary detection hardening" pass, after a live bag capture
# (boundary_constraint_diag_20260813_124824) confirmed unsmoothed per-frame
# publishing produced 0.017-0.5m flicker and 25 present/absent transitions
# in under 9 seconds, which mpc_solver's own OSQP QP correctly flagged as
# infeasible on some ticks.
# ==============================================================================

class TestTrackedBoundaryLine:

    def test_seeding_takes_the_raw_fit_unsmoothed(self):
        track = TrackedBoundaryLine((1.0, 0.0), 1.5)
        assert track.normal == pytest.approx((1.0, 0.0))
        assert track.offset == pytest.approx(1.5)
        assert track.frames_matched == 1
        assert track.frames_since_seen == 0

    def test_update_blends_by_alpha_not_jumping_to_the_new_value(self):
        track = TrackedBoundaryLine((1.0, 0.0), 1.5)
        track.update((1.0, 0.0), 1.6, EMA_ALPHA)
        # smoothed = alpha*new + (1-alpha)*prev = 0.3*1.6 + 0.7*1.5 = 1.53.
        assert track.offset == pytest.approx(1.53)
        assert track.frames_matched == 2
        assert track.frames_since_seen == 0

    def test_update_renormalizes_the_blended_normal(self):
        track = TrackedBoundaryLine((1.0, 0.0), 1.5)
        # A rotated new normal -- the raw convex blend of two unit vectors
        # isn't itself unit length, so update() must renormalize.
        track.update((0.0, 1.0), 1.5, 0.5)
        mag = math.hypot(*track.normal)
        assert mag == pytest.approx(1.0, abs=1e-9)

    def test_repeated_updates_converge_toward_a_stable_new_value(self):
        track = TrackedBoundaryLine((1.0, 0.0), 1.0)
        for _ in range(50):
            track.update((1.0, 0.0), 2.0, EMA_ALPHA)
        assert track.offset == pytest.approx(2.0, abs=1e-3)


class TestUpdateSideTrack:

    def test_seeds_a_fresh_track_on_first_fit(self):
        track = update_side_track(None, (1.0, 0.0, 1.5), EMA_ALPHA, HOLD_FRAMES)
        assert track is not None
        assert track.normal == pytest.approx((1.0, 0.0))
        assert track.offset == pytest.approx(1.5)
        assert track.frames_matched == 1

    def test_miss_with_no_existing_track_stays_none(self):
        assert update_side_track(None, None, EMA_ALPHA, HOLD_FRAMES) is None

    def test_single_bad_frame_does_not_immediately_drop_an_eligible_track(self):
        # Build up an eligible track first (mirrors "a single noisy frame
        # can no longer alone produce/withdraw a hard constraint").
        track = None
        for _ in range(MIN_FRAMES_TO_PUBLISH):
            track = update_side_track(track, (1.0, 0.0, 1.5), EMA_ALPHA, HOLD_FRAMES)
        assert side_track_eligible(track, MIN_FRAMES_TO_PUBLISH)

        # One missed frame (quality gate rejected this tick).
        track = update_side_track(track, None, EMA_ALPHA, HOLD_FRAMES)
        assert track is not None
        assert track.frames_since_seen == 1
        # frames_matched is untouched by a miss -- still eligible, still
        # publishing the held (last-known-good) value, not withdrawn.
        assert track.frames_matched == MIN_FRAMES_TO_PUBLISH
        assert side_track_eligible(track, MIN_FRAMES_TO_PUBLISH)
        assert track.offset == pytest.approx(1.5)

    def test_a_matched_frame_after_a_miss_resets_frames_since_seen_and_blends(self):
        track = TrackedBoundaryLine((1.0, 0.0), 1.5)
        track.frames_since_seen = 2  # simulate two prior misses
        track = update_side_track(track, (1.0, 0.0, 1.6), EMA_ALPHA, HOLD_FRAMES)
        assert track.frames_since_seen == 0
        assert track.offset == pytest.approx(0.3 * 1.6 + 0.7 * 1.5)

    def test_sustained_miss_survives_up_to_hold_frames_not_before(self):
        track = TrackedBoundaryLine((1.0, 0.0), 1.5)
        for i in range(HOLD_FRAMES):
            track = update_side_track(track, None, EMA_ALPHA, HOLD_FRAMES)
            assert track is not None, f'dropped too early, after only {i + 1} miss(es)'
            assert track.frames_since_seen == i + 1

    def test_sustained_miss_drops_once_hold_frames_is_exceeded(self):
        track = TrackedBoundaryLine((1.0, 0.0), 1.5)
        for _ in range(HOLD_FRAMES):
            track = update_side_track(track, None, EMA_ALPHA, HOLD_FRAMES)
        assert track is not None  # still alive right at the boundary
        track = update_side_track(track, None, EMA_ALPHA, HOLD_FRAMES)
        assert track is None  # exceeded hold_frames -- dropped

    def test_held_track_that_never_recovers_stays_dropped(self):
        track = TrackedBoundaryLine((1.0, 0.0), 1.5)
        for _ in range(HOLD_FRAMES + 1):
            track = update_side_track(track, None, EMA_ALPHA, HOLD_FRAMES)
        assert track is None
        # One more miss on an already-None track: still None, not an error.
        track = update_side_track(track, None, EMA_ALPHA, HOLD_FRAMES)
        assert track is None

    def test_noisy_flicker_sequence_smooths_to_a_stable_output(self):
        # Synthesizes a comparably large frame-to-frame jump pattern to the
        # live bag capture's own observed 0.017-0.5m flicker (not a literal
        # replay of that capture's numbers, which weren't saved standalone)
        # -- alternates around a stable true line (offset=1.5) with swings
        # of roughly +-0.3m frame to frame, all individually valid fits
        # (quality-gate acceptance is fit_side_boundary's own job, already
        # covered by TestFitSideBoundary above; this tests what the tracker
        # does with a valid-but-noisy INPUT sequence).
        raw_offsets = [1.5, 1.8, 1.2, 1.7, 1.3, 1.6, 1.2, 1.8, 1.3, 1.7] * 3
        track = None
        smoothed = []
        for raw_offset in raw_offsets:
            track = update_side_track(track, (1.0, 0.0, raw_offset), EMA_ALPHA, HOLD_FRAMES)
            smoothed.append(track.offset)

        raw_deltas = [abs(b - a) for a, b in zip(raw_offsets, raw_offsets[1:])]
        smoothed_deltas = [abs(b - a) for a, b in zip(smoothed, smoothed[1:])]
        # The smoothed sequence's own frame-to-frame jumps must be
        # meaningfully damped relative to the raw input's -- the whole
        # point of EMA smoothing here (this is a real regression target,
        # not an arbitrary tolerance: with an unsmoothed pass-through,
        # these two averages would be equal).
        assert (sum(smoothed_deltas) / len(smoothed_deltas)) < 0.5 * (
            sum(raw_deltas) / len(raw_deltas))
        # And it should still be tracking the true stable line, not
        # drifting away from it.
        assert smoothed[-1] == pytest.approx(1.5, abs=0.15)


class TestSideTrackEligible:

    def test_no_track_is_never_eligible(self):
        assert side_track_eligible(None, MIN_FRAMES_TO_PUBLISH) is False

    def test_fresh_track_below_the_frame_bar_is_not_eligible(self):
        track = TrackedBoundaryLine((1.0, 0.0), 1.5)
        assert track.frames_matched == 1 < MIN_FRAMES_TO_PUBLISH
        assert side_track_eligible(track, MIN_FRAMES_TO_PUBLISH) is False

    def test_track_at_the_frame_bar_is_eligible(self):
        track = None
        for _ in range(MIN_FRAMES_TO_PUBLISH):
            track = update_side_track(track, (1.0, 0.0, 1.5), EMA_ALPHA, HOLD_FRAMES)
        assert side_track_eligible(track, MIN_FRAMES_TO_PUBLISH) is True

    def test_track_held_through_a_miss_stays_eligible(self):
        # frames_matched (what eligibility checks) is untouched by a miss --
        # holding is exactly about NOT re-litigating eligibility on a miss.
        track = None
        for _ in range(MIN_FRAMES_TO_PUBLISH):
            track = update_side_track(track, (1.0, 0.0, 1.5), EMA_ALPHA, HOLD_FRAMES)
        track = update_side_track(track, None, EMA_ALPHA, HOLD_FRAMES)
        assert side_track_eligible(track, MIN_FRAMES_TO_PUBLISH) is True


if __name__ == '__main__':
    import sys
    sys.exit(pytest.main([__file__, '-v']))
