"""lidar_front_wall.py tests -- the fit, its checks and the estimator, no rclpy.

Synthetic scans use this car's real Hokuyo geometry (270 deg at 0.25 deg,
1081 beams, float32 angles) and its base_link -> laser offset, so the sector
counts and distances are the ones the node actually sees.

What these defend, beyond "the maths is right":
  * dead reckoning projects motion on the wall normal: a car sliding along a
    wall does not close on it, which path-length arithmetic would claim;
  * the wrong-surface gate rejects an object face that dead reckoning
    contradicts, and is never applied against a seed;
  * the estimate snaps to a measurement (no blending) and reports the jump;
  * an age-out re-seeds when a seed is fresh, and is NONE only without one;
  * a flat object face filling the sector PASSES the single-scan checks --
    the documented blind spot the gate exists for.

Run standalone: python3 -m pytest test/test_lidar_front_wall.py -v
"""

import math

import numpy as np
import pytest

from f1tenth_perception.lidar_front_wall import (
    PROVENANCE_DEAD_RECKONED,
    PROVENANCE_MEASURED,
    PROVENANCE_NONE,
    PROVENANCE_SEEDED,
    REASON_LOW_INLIER_COUNT,
    REASON_LOW_INLIER_FRACTION,
    REASON_OBLIQUE,
    REASON_OK,
    REASON_TOO_FEW_RETURNS,
    REASON_WRONG_SURFACE,
    SectorFit,
    WallEstimator,
    classify_fit,
    fit_forward_line,
    min_inlier_count,
)

ANGLE_MIN = float(np.float32(-2.356194496154785))
ANGLE_INCREMENT = float(np.float32(0.004363323096185923))
N_BEAMS = 1081
LASER = (0.12, 0.0, 0.0)  # description.launch.py's base_link -> laser


def _bearings(laser=LASER):
    angles = ANGLE_MIN + np.arange(N_BEAMS) * ANGLE_INCREMENT + laser[2]
    return np.arctan2(np.sin(angles), np.cos(angles))


def _wall_ranges(distance, normal_deg, laser=LASER):
    """Ranges from the laser to the base_link line n . p = distance, inf where a
    beam never meets it."""
    nx, ny = math.cos(math.radians(normal_deg)), math.sin(math.radians(normal_deg))
    bearings = _bearings(laser)
    along = nx * np.cos(bearings) + ny * np.sin(bearings)
    ranges = np.full(N_BEAMS, np.inf)
    hit = along > 1e-9
    ranges[hit] = (distance - (nx * laser[0] + ny * laser[1])) / along[hit]
    return ranges


def _sector_indices(half_deg=10.0, laser=LASER):
    return np.flatnonzero(
        np.abs(_bearings(laser)) <= math.radians(half_deg) + 0.5 * ANGLE_INCREMENT)


def _fit(ranges, half_deg=10.0, laser=LASER, max_pairs=800):
    return fit_forward_line(ranges, ANGLE_MIN, ANGLE_INCREMENT, 0.02, 30.0, laser,
                            math.radians(half_deg), 0.03, max_pairs)


def _verdict(fit):
    return classify_fit(fit, 0.6, 0.5, math.radians(45.0))


# ============================================================================
# The sector
# ============================================================================

class TestSector:

    @pytest.mark.parametrize('half_deg,beams,floor', [(10.0, 81, 41), (5.0, 41, 21)])
    def test_sector_beam_count_and_inlier_floor(self, half_deg, beams, floor):
        fit = _fit(_wall_ranges(2.0, 0.0), half_deg=half_deg)
        assert fit.sector_beams == beams
        assert min_inlier_count(fit.sector_beams, 0.5) == floor

    def test_sector_is_centred_on_the_car_not_the_laser_mount(self):
        # A laser yawed 20 deg on its mount still looks along the car's heading:
        # a wall square to base_link reads square, with the full sector.
        laser = (0.12, 0.0, math.radians(20.0))
        fit = _fit(_wall_ranges(2.0, 0.0, laser=laser), laser=laser)
        assert fit.sector_beams == 81
        assert fit.normal_angle == pytest.approx(0.0, abs=1e-6)
        assert fit.distance == pytest.approx(2.0, abs=1e-6)


# ============================================================================
# The fit
# ============================================================================

class TestFit:

    def test_distance_is_from_base_link_not_from_the_laser(self):
        fit = _fit(_wall_ranges(2.0, 0.0))
        assert fit.distance == pytest.approx(2.0, abs=1e-6)
        assert fit.inlier_count == fit.valid_returns == 81

    @pytest.mark.parametrize('normal_deg', [30.0, -40.0])
    def test_positive_normal_angle_is_to_the_left(self, normal_deg):
        fit = _fit(_wall_ranges(2.0, normal_deg))
        assert math.degrees(fit.normal_angle) == pytest.approx(normal_deg, abs=1e-4)
        assert fit.distance == pytest.approx(2.0, abs=1e-6)

    def test_non_returns_do_not_count_as_valid_or_against_the_fraction(self):
        ranges = _wall_ranges(2.0, 0.0)
        sector = _sector_indices()
        ranges[sector[0]] = np.inf
        ranges[sector[1]] = np.nan
        ranges[sector[2]] = 0.0     # below range_min
        ranges[sector[3]] = 45.0    # above range_max
        fit = _fit(ranges)
        assert fit.sector_beams == 81
        assert fit.valid_returns == 77
        assert fit.inlier_fraction == pytest.approx(1.0)

    def test_an_object_in_a_third_of_the_sector_leaves_the_fit_on_the_wall(self):
        rng = np.random.default_rng(3)
        ranges = _wall_ranges(3.0, 0.0) + rng.normal(0.0, 0.004, N_BEAMS)
        in_front = np.abs(_bearings()) < math.radians(3.5)
        ranges[in_front] = 1.0 + rng.normal(0.0, 0.004, np.count_nonzero(in_front))
        fit = _fit(ranges)
        assert fit.distance == pytest.approx(3.0, abs=0.01)
        assert _verdict(fit) == REASON_OK

    def test_same_scan_same_answer(self):
        rng = np.random.default_rng(7)
        ranges = _wall_ranges(2.5, 10.0) + rng.normal(0.0, 0.005, N_BEAMS)
        assert _fit(ranges) == _fit(ranges)

    def test_capped_hypotheses_agree_with_the_exhaustive_search(self):
        rng = np.random.default_rng(11)
        ranges = _wall_ranges(3.0, 5.0) + rng.normal(0.0, 0.004, N_BEAMS)
        clutter = _sector_indices()[::3]
        ranges[clutter] = rng.uniform(0.6, 2.5, clutter.size)
        capped, exhaustive = _fit(ranges), _fit(ranges, max_pairs=10 ** 6)
        assert capped.distance == pytest.approx(exhaustive.distance, abs=0.005)
        assert capped.normal_angle == pytest.approx(exhaustive.normal_angle, abs=math.radians(0.3))


# ============================================================================
# Single-scan verdicts
# ============================================================================

class TestVerdict:

    def test_too_few_returns(self):
        ranges = _wall_ranges(2.0, 0.0)
        ranges[_sector_indices()[::2]] = np.inf   # 41 of 81 gone -> 40 returns < floor 41
        fit = _fit(ranges)
        assert fit.valid_returns == 40
        assert _verdict(fit) == REASON_TOO_FEW_RETURNS

    def test_low_inlier_count_even_when_the_fraction_would_pass(self):
        rng = np.random.default_rng(5)
        ranges = _wall_ranges(2.0, 0.0)
        sector = _sector_indices()
        ranges[sector[:21]] = np.inf                                # 60 returns
        ranges[sector[21:43]] = rng.uniform(0.4, 1.6, 22)           # 22 scattered
        fit = _fit(ranges)                                          # 38 on the wall
        assert fit.valid_returns == 60
        assert fit.inlier_count == 38
        assert fit.inlier_fraction >= 0.6
        assert _verdict(fit) == REASON_LOW_INLIER_COUNT

    def test_low_inlier_fraction_when_the_wall_is_not_the_dominant_surface(self):
        ranges = _wall_ranges(3.0, 0.0)
        ranges[_sector_indices()[:36]] = 1.0     # 36 on an object, 45 on the wall
        fit = _fit(ranges)
        assert fit.distance == pytest.approx(3.0, abs=1e-3)
        assert fit.inlier_count == 45
        assert _verdict(fit) == REASON_LOW_INLIER_FRACTION

    @pytest.mark.parametrize('normal_deg,expected', [(40.0, REASON_OK), (50.0, REASON_OBLIQUE),
                                                     (-50.0, REASON_OBLIQUE)])
    def test_oblique_walls_are_rejected_past_45_degrees(self, normal_deg, expected):
        assert _verdict(_fit(_wall_ranges(2.0, normal_deg))) == expected

    def test_a_flat_object_face_filling_the_sector_passes_every_single_scan_check(self):
        # The documented blind spot: nothing in one scan distinguishes this from
        # a wall. WallEstimator's wrong-surface gate is what catches it, and
        # only when there is a prediction to contradict it.
        ranges = _wall_ranges(3.0, 0.0)
        ranges[np.abs(_bearings()) < math.radians(7.1)] = 1.0
        fit = _fit(ranges)
        assert fit.distance == pytest.approx(1.12, abs=0.01)
        assert _verdict(fit) == REASON_OK


# ============================================================================
# The estimator
# ============================================================================

def _estimator():
    return WallEstimator(stale_age_sec=3.0, wrong_surface_gate_m=0.3, odom_max_age_sec=0.2,
                         seed_max_age_sec=0.5, seed_max_valid_m=5.0)


def _wall(distance, normal_angle=0.0):
    return SectorFit(81, 81, distance, normal_angle, 81, 0.003)


_LOST = SectorFit(81, 5)


def _measure(est, now, distance, normal_angle=0.0):
    return est.step(now, _wall(distance, normal_angle), REASON_OK)


def _lose(est, now, reason=REASON_TOO_FEW_RETURNS):
    return est.step(now, _LOST, reason)


class TestProvenance:

    def test_first_fit_is_measured_and_an_acquisition_without_a_jump(self):
        est = _estimator()
        est.update_odometry(0.0, 0.0, 0.0, 0.0)
        step = _measure(est, 0.0, 2.0)
        assert step.estimate.provenance == PROVENANCE_MEASURED
        assert step.estimate.valid and step.estimate.distance == 2.0
        assert step.estimate.source_age == 0.0
        assert step.reacquisition.previous_provenance == PROVENANCE_NONE
        assert math.isnan(step.reacquisition.jump)

    def test_measured_to_measured_is_not_an_acquisition(self):
        est = _estimator()
        est.update_odometry(0.0, 0.0, 0.0, 0.0)
        _measure(est, 0.0, 2.0)
        est.update_odometry(0.05, 0.02, 0.0, 0.0)
        assert _measure(est, 0.05, 1.98).reacquisition is None

    def test_nothing_at_all_is_none_and_invalid(self):
        step = _lose(_estimator(), 1.0)
        assert step.estimate.provenance == PROVENANCE_NONE
        assert not step.estimate.valid
        assert math.isnan(step.estimate.distance) and math.isnan(step.estimate.source_age)

    def test_never_measured_with_a_fresh_seed_is_seeded_without_a_normal(self):
        est = _estimator()
        est.update_seed(0.9, 1.25)
        step = _lose(est, 1.0)
        assert step.estimate.provenance == PROVENANCE_SEEDED
        assert step.estimate.valid and step.estimate.distance == 1.25
        assert math.isnan(step.estimate.normal_angle)
        assert step.estimate.source_age == pytest.approx(0.1)


class TestDeadReckoning:

    def _anchored(self):
        est = _estimator()
        est.update_odometry(0.0, 0.0, 0.0, 0.0)
        _measure(est, 0.0, 2.0)
        return est

    def test_driving_straight_at_the_wall_closes_by_the_distance_driven(self):
        est = self._anchored()
        est.update_odometry(1.0, 0.5, 0.0, 0.0)
        step = _lose(est, 1.0)
        assert step.estimate.provenance == PROVENANCE_DEAD_RECKONED
        assert step.estimate.distance == pytest.approx(1.5)
        assert step.estimate.source_age == pytest.approx(1.0)

    def test_sliding_along_the_wall_does_not_close_on_it(self):
        # One metre of path, zero metres of closing. d0 - distance_travelled
        # would say 1.0 m; the wall has not moved closer at all.
        est = self._anchored()
        est.update_odometry(1.0, 0.0, 1.0, 0.0)
        assert _lose(est, 1.0).estimate.distance == pytest.approx(2.0)

    def test_driving_60_degrees_off_the_normal_closes_by_the_cosine(self):
        est = self._anchored()
        heading = math.radians(60.0)
        est.update_odometry(1.0, math.cos(heading), math.sin(heading), heading)
        assert _lose(est, 1.0).estimate.distance == pytest.approx(1.5)

    def test_the_reported_normal_turns_with_the_car(self):
        est = self._anchored()
        est.update_odometry(0.5, 0.0, 0.0, math.radians(60.0))
        assert math.degrees(_lose(est, 0.5).estimate.normal_angle) == pytest.approx(-60.0)

    def test_the_normal_captured_while_turned_is_kept_in_the_odometry_frame(self):
        est = _estimator()
        est.update_odometry(0.0, 0.0, 0.0, math.radians(30.0))
        _measure(est, 0.0, 2.0, normal_angle=math.radians(-30.0))   # wall normal is world +x
        est.update_odometry(1.0, 0.4, 0.0, 0.0)
        step = _lose(est, 1.0)
        assert step.estimate.distance == pytest.approx(1.6)
        assert step.estimate.normal_angle == pytest.approx(0.0, abs=1e-9)

    def test_still_dead_reckoned_at_the_stale_age_and_not_just_past_it(self):
        est = self._anchored()
        est.update_odometry(3.0, 0.0, 0.0, 0.0)
        assert _lose(est, 3.0).estimate.provenance == PROVENANCE_DEAD_RECKONED
        est.update_odometry(3.01, 0.0, 0.0, 0.0)
        assert _lose(est, 3.01).estimate.provenance == PROVENANCE_NONE

    def test_stale_odometry_turns_off_dead_reckoning(self):
        est = self._anchored()
        step = _lose(est, 0.5)          # last odometry at t=0, max age 0.2
        assert step.estimate.provenance == PROVENANCE_NONE
        assert not step.odometry_fresh

    def test_a_fit_accepted_without_odometry_can_never_be_dead_reckoned(self):
        est = _estimator()
        _measure(est, 0.0, 2.0)          # no odometry yet
        est.update_odometry(0.5, 0.0, 0.0, 0.0)
        assert _lose(est, 0.5).estimate.provenance == PROVENANCE_NONE


class TestWrongSurfaceGate:

    def test_a_fit_far_from_the_prediction_is_rejected_and_dead_reckoning_continues(self):
        est = _estimator()
        est.update_odometry(0.0, 0.0, 0.0, 0.0)
        _measure(est, 0.0, 3.0)
        est.update_odometry(0.1, 0.05, 0.0, 0.0)
        step = _measure(est, 0.1, 1.0)   # something stepped in front of the wall
        assert step.reason == REASON_WRONG_SURFACE
        assert step.predicted_distance == pytest.approx(2.95)
        assert step.innovation == pytest.approx(-1.95)
        assert step.estimate.provenance == PROVENANCE_DEAD_RECKONED
        assert step.estimate.distance == pytest.approx(2.95)
        assert step.reacquisition is None

    def test_rejection_keeps_the_original_anchor(self):
        est = _estimator()
        est.update_odometry(0.0, 0.0, 0.0, 0.0)
        _measure(est, 0.0, 3.0)
        est.update_odometry(0.5, 0.0, 0.0, 0.0)
        _measure(est, 0.5, 1.0)
        est.update_odometry(1.0, 0.0, 0.0, 0.0)
        assert _lose(est, 1.0).estimate.source_age == pytest.approx(1.0)

    def test_a_fit_within_the_gate_snaps_without_blending_and_reports_the_jump(self):
        est = _estimator()
        est.update_odometry(0.0, 0.0, 0.0, 0.0)
        _measure(est, 0.0, 2.0)
        est.update_odometry(1.0, 0.5, 0.0, 0.0)
        _lose(est, 1.0)                                 # dead-reckoned 1.5
        est.update_odometry(1.1, 0.5, 0.0, 0.0)
        step = _measure(est, 1.1, 1.42)
        assert step.reason == REASON_OK
        assert step.estimate.distance == 1.42           # exactly: no blend
        assert step.reacquisition.previous_provenance == PROVENANCE_DEAD_RECKONED
        assert step.reacquisition.jump == pytest.approx(-0.08)
        assert step.reacquisition.since_last_measurement == pytest.approx(1.1)

    def test_never_applied_against_a_seed(self):
        est = _estimator()
        est.update_odometry(0.0, 0.0, 0.0, 0.0)
        est.update_seed(0.0, 1.2)
        _lose(est, 0.1)                                 # seeded 1.2
        est.update_odometry(0.2, 0.0, 0.0, 0.0)
        step = _measure(est, 0.2, 3.0)
        assert step.reason == REASON_OK
        assert math.isnan(step.innovation)
        assert step.reacquisition.previous_provenance == PROVENANCE_SEEDED
        assert step.reacquisition.jump == pytest.approx(1.8)

    def test_lapses_once_the_prediction_has_aged_out(self):
        est = _estimator()
        est.update_odometry(0.0, 0.0, 0.0, 0.0)
        _measure(est, 0.0, 3.0)
        est.update_odometry(3.5, 0.0, 0.0, 0.0)
        step = _measure(est, 3.5, 1.0)
        assert step.reason == REASON_OK
        assert step.estimate.distance == 1.0

    def test_off_while_odometry_is_stale(self):
        est = _estimator()
        est.update_odometry(0.0, 0.0, 0.0, 0.0)
        _measure(est, 0.0, 3.0)
        assert _measure(est, 0.5, 1.0).reason == REASON_OK

    def test_a_single_scan_rejection_is_not_overridden_by_the_gate(self):
        est = _estimator()
        est.update_odometry(0.0, 0.0, 0.0, 0.0)
        _measure(est, 0.0, 3.0)
        est.update_odometry(0.05, 0.0, 0.0, 0.0)
        step = est.step(0.05, _wall(3.0, math.radians(50.0)), REASON_OBLIQUE)
        assert step.reason == REASON_OBLIQUE
        assert math.isnan(step.innovation)


class TestAgeOutAndSeed:

    def _aged_out(self):
        est = _estimator()
        est.update_odometry(0.0, 0.0, 0.0, 0.0)
        _measure(est, 0.0, 2.0)
        est.update_odometry(4.0, 0.0, 0.0, 0.0)
        return est

    def test_age_out_re_seeds_from_a_fresh_seed(self):
        est = self._aged_out()
        est.update_seed(3.9, 1.7)
        step = _lose(est, 4.0)
        assert step.estimate.provenance == PROVENANCE_SEEDED
        assert step.estimate.distance == 1.7

    def test_age_out_without_a_seed_is_none_with_the_age_of_the_last_fit(self):
        step = _lose(self._aged_out(), 4.0)
        assert step.estimate.provenance == PROVENANCE_NONE
        assert not step.estimate.valid
        assert step.estimate.source_age == pytest.approx(4.0)

    def test_a_stale_seed_is_not_used(self):
        est = self._aged_out()
        est.update_seed(3.4, 1.7)
        assert _lose(est, 4.0).estimate.provenance == PROVENANCE_NONE

    @pytest.mark.parametrize('value', [6.0, 0.0, -1.0, float('inf'), float('nan')])
    def test_out_of_range_seed_values_are_not_distances(self, value):
        est = self._aged_out()
        est.update_seed(4.0, value)
        assert _lose(est, 4.0).estimate.provenance == PROVENANCE_NONE

    def test_the_seed_limit_is_inclusive(self):
        est = self._aged_out()
        est.update_seed(4.0, 5.0)
        assert _lose(est, 4.0).estimate.provenance == PROVENANCE_SEEDED
