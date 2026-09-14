"""Forward wall-line fit and the measured / dead-reckoned wall estimate.

Pure logic, no rclpy: lidar_front_wall_node.py is the ROS glue, and every
number that node publishes is decided here, so it can be unit-tested and
replayed against archived scans without a running graph (the same split as
f1tenth_costmap's costmap_boundary.py / costmap_boundary_node.py).

The contract, every threshold's measured justification, the known odometry
bias and the blind spots are documented ONCE, in lidar_front_wall_node.py's
module docstring. Read that before changing anything here.

Frames and signs used throughout:
  * base_link, x forward, y left.
  * distance      perpendicular distance from the base_link ORIGIN to the
                  fitted line [m], always >= 0.
  * normal_angle  bearing of the line's normal pointing FROM the car TOWARD
                  the wall [rad], 0 = dead ahead, + = left. |normal_angle| is
                  the angle between the heading and the wall normal.
  * world normal  normal_angle + odometry yaw: the wall normal's direction in
                  the odometry frame, constant for a fixed wall however the
                  car turns. Dead reckoning projects motion onto it.
"""

import math
from dataclasses import dataclass
from typing import Optional

import numpy as np

# Verdict codes for one scan's fit. Mirror f1tenth_messages/WallLineFit.msg's
# REASON_* constants; test_lidar_front_wall_node.py asserts they stay equal.
# A rejected fit reports the FIRST failing check, in this order.
REASON_OK = 0
REASON_NO_TRANSFORM = 1
REASON_TOO_FEW_RETURNS = 2
REASON_LOW_INLIER_COUNT = 3
REASON_LOW_INLIER_FRACTION = 4
REASON_OBLIQUE = 5
REASON_WRONG_SURFACE = 6
REASON_NAMES = {
    REASON_OK: 'ok',
    REASON_NO_TRANSFORM: 'no_transform',
    REASON_TOO_FEW_RETURNS: 'too_few_returns',
    REASON_LOW_INLIER_COUNT: 'low_inlier_count',
    REASON_LOW_INLIER_FRACTION: 'low_inlier_fraction',
    REASON_OBLIQUE: 'oblique',
    REASON_WRONG_SURFACE: 'wrong_surface',
}

# Mirror f1tenth_messages/WallEstimate.msg's PROVENANCE_* constants.
PROVENANCE_NONE = 0
PROVENANCE_MEASURED = 1
PROVENANCE_DEAD_RECKONED = 2
PROVENANCE_SEEDED = 3
PROVENANCE_NAMES = {
    PROVENANCE_NONE: 'none',
    PROVENANCE_MEASURED: 'measured',
    PROVENANCE_DEAD_RECKONED: 'dead_reckoned',
    PROVENANCE_SEEDED: 'seeded',
}

# A hypothesis line through two returns closer than this is dominated by
# range noise in its direction (2.8 mm median residual on this Hokuyo) --
# skip it rather than score it.
_MIN_HYPOTHESIS_BASELINE_M = 0.02


def _wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


# ============================================================================
# Part 1 -- one scan's robust line fit
# ============================================================================

@dataclass(frozen=True)
class SectorFit:
    """One scan's forward-sector line fit. The line fields are NaN/0 when no
    line could be fitted at all; sector_beams/valid_returns are always set."""

    sector_beams: int
    valid_returns: int
    distance: float = math.nan
    normal_angle: float = math.nan
    inlier_count: int = 0
    rms_residual: float = math.nan

    @property
    def has_line(self):
        return not math.isnan(self.distance)

    @property
    def inlier_fraction(self):
        """Inliers / VALID returns -- not / sector beams. A beam with no return
        is not evidence against the line; a return somewhere else is."""
        if self.valid_returns == 0:
            return 0.0
        return self.inlier_count / self.valid_returns


_pair_cache = {}


def _hypothesis_pairs(n_points, max_pairs):
    """Index pairs (i < j) that seed the candidate lines. Exhaustive when that
    is at most max_pairs, otherwise every k-th pair in lexicographic order, so
    every point still anchors a spread of partners. Deterministic on purpose:
    the same scan must give the same verdict, in replay and in tests."""
    key = (n_points, max_pairs)
    pairs = _pair_cache.get(key)
    if pairs is None:
        i, j = np.triu_indices(n_points, 1)
        if i.size > max_pairs:
            step = -(-i.size // max_pairs)
            i, j = i[::step], j[::step]
        pairs = (i, j)
        _pair_cache[key] = pairs
    return pairs


def _robust_line(points, inlier_distance_m, max_pairs):
    """MSAC over two-point hypotheses, then two total-least-squares refits on
    the inliers. Returns (normal_x, normal_y, offset, inlier_count, rms) with
    the normal oriented so offset >= 0 (pointing from the origin to the line),
    or None when fewer than three returns agree on any line.

    Robust, not least squares, because the sector routinely holds a doorway,
    a table leg or a person in front of the wall: least squares would average
    them into a line that is neither surface. MSAC (truncated residual cost)
    rather than plain inlier counting, so a tie between two surfaces goes to
    the one the returns sit tighter on."""
    i, j = _hypothesis_pairs(len(points), max_pairs)
    delta = points[j] - points[i]
    length = np.hypot(delta[:, 0], delta[:, 1])
    keep = length > _MIN_HYPOTHESIS_BASELINE_M
    if not np.any(keep):
        return None
    i, delta, length = i[keep], delta[keep], length[keep]
    normals = np.column_stack([-delta[:, 1], delta[:, 0]]) / length[:, None]
    offsets = np.einsum('ij,ij->i', normals, points[i])
    residuals = np.abs(points @ normals.T - offsets)
    best = int(np.argmin(np.minimum(residuals, inlier_distance_m).sum(axis=0)))
    inliers = residuals[:, best] <= inlier_distance_m

    for _ in range(2):
        if np.count_nonzero(inliers) < 3:
            return None
        inlier_points = points[inliers]
        centroid = inlier_points.mean(axis=0)
        _, _, vt = np.linalg.svd(inlier_points - centroid, full_matrices=False)
        normal = vt[-1]
        offset = float(normal @ centroid)
        if offset < 0.0:
            normal, offset = -normal, -offset
        distances = np.abs(points @ normal - offset)
        inliers = distances <= inlier_distance_m

    count = int(np.count_nonzero(inliers))
    if count < 3:
        return None
    rms = float(np.sqrt(np.mean(distances[inliers] ** 2)))
    return float(normal[0]), float(normal[1]), offset, count, rms


def fit_forward_line(ranges, angle_min, angle_increment, range_min, range_max,
                     laser_pose, half_angle_rad, inlier_distance_m, max_pairs):
    """Fit the dominant line to the returns within +-half_angle_rad of the
    CAR's heading, in base_link.

    laser_pose is (x, y, yaw) of the scan frame in base_link. The sector is
    selected on bearing = beam angle + laser yaw, so a mount-yaw correction in
    the TF moves the sector with it. A beam is in the sector when its centre
    is within half an increment of the edge, which makes the count symmetric
    (81 beams at +-10 deg and 41 at +-5 deg on this Hokuyo's 0.25 deg grid)
    instead of losing an edge beam to float32 rounding.

    NaN, +-Inf, and anything outside [range_min, range_max] is not a return.
    """
    ranges = np.asarray(ranges, dtype=np.float64)
    laser_x, laser_y, laser_yaw = laser_pose
    angles = angle_min + np.arange(ranges.size) * angle_increment + laser_yaw
    bearings = np.arctan2(np.sin(angles), np.cos(angles))
    in_sector = np.abs(bearings) <= half_angle_rad + 0.5 * abs(angle_increment)
    sector_beams = int(np.count_nonzero(in_sector))
    with np.errstate(invalid='ignore'):
        valid = (in_sector & np.isfinite(ranges)
                 & (ranges >= range_min) & (ranges <= range_max))
    valid_returns = int(np.count_nonzero(valid))
    if valid_returns < 3:
        return SectorFit(sector_beams, valid_returns)

    r = ranges[valid]
    b = bearings[valid]
    points = np.column_stack([laser_x + r * np.cos(b), laser_y + r * np.sin(b)])
    line = _robust_line(points, inlier_distance_m, max_pairs)
    if line is None:
        return SectorFit(sector_beams, valid_returns)
    normal_x, normal_y, offset, count, rms = line
    return SectorFit(sector_beams, valid_returns, offset,
                     math.atan2(normal_y, normal_x), count, rms)


def min_inlier_count(sector_beams, min_inlier_count_ratio):
    """The absolute inlier floor, scaled with the sector: ceil(ratio x beams).
    41 at +-10 deg and 21 at +-5 deg with the default 0.5."""
    return max(3, math.ceil(min_inlier_count_ratio * sector_beams - 1e-9))


def classify_fit(fit, min_inlier_fraction, min_inlier_count_ratio, oblique_max_rad):
    """Single-scan verdict. The wrong-surface test needs a prediction and lives
    in WallEstimator.step() instead."""
    floor = min_inlier_count(fit.sector_beams, min_inlier_count_ratio)
    if not fit.has_line or fit.valid_returns < floor:
        return REASON_TOO_FEW_RETURNS
    if fit.inlier_count < floor:
        return REASON_LOW_INLIER_COUNT
    if fit.inlier_fraction < min_inlier_fraction:
        return REASON_LOW_INLIER_FRACTION
    if abs(fit.normal_angle) > oblique_max_rad:
        return REASON_OBLIQUE
    return REASON_OK


# ============================================================================
# Part 2 -- measured / dead-reckoned / seeded estimate
# ============================================================================

@dataclass(frozen=True)
class Estimate:
    provenance: int
    valid: bool
    distance: float
    normal_angle: float
    source_age: float


@dataclass(frozen=True)
class Reacquisition:
    """Emitted on the step a fit is accepted after a step that was not
    measured. jump = measured - previous estimate: NEGATIVE means the wall was
    nearer than the estimate said, which is the direction the known odometry
    scale bias drives dead reckoning (see the node docstring)."""

    previous_provenance: int
    previous_distance: float
    jump: float
    since_last_measurement: float


@dataclass(frozen=True)
class Step:
    reason: int
    predicted_distance: float
    innovation: float
    estimate: Estimate
    reacquisition: Optional[Reacquisition]
    odometry_fresh: bool


@dataclass(frozen=True)
class _Pose:
    received: float
    x: float
    y: float
    yaw: float


@dataclass(frozen=True)
class _Anchor:
    """The last ACCEPTED measurement: where dead reckoning starts from."""

    time: float
    distance: float
    world_normal: Optional[float]
    pose: Optional[_Pose]


class WallEstimator:
    """Owns the measured -> dead_reckoned -> seeded -> none state. Callers feed
    odometry and seed messages as they arrive and call step() once per scan;
    `now` is always passed in, so replay and tests control the clock."""

    def __init__(self, stale_age_sec, wrong_surface_gate_m, odom_max_age_sec,
                 seed_max_age_sec, seed_max_valid_m):
        self.stale_age_sec = float(stale_age_sec)
        self.wrong_surface_gate_m = float(wrong_surface_gate_m)
        self.odom_max_age_sec = float(odom_max_age_sec)
        self.seed_max_age_sec = float(seed_max_age_sec)
        self.seed_max_valid_m = float(seed_max_valid_m)
        self._pose = None
        self._seed = None
        self._anchor = None
        self._previous = None

    def update_odometry(self, now, x, y, yaw):
        self._pose = _Pose(float(now), float(x), float(y), float(yaw))

    def update_seed(self, now, value):
        # Receipt time, not a stamp: /costmap/front_clearance is a bare
        # Float32 with no header.
        self._seed = (float(now), float(value))

    def _fresh_pose(self, now):
        if self._pose is None or now - self._pose.received > self.odom_max_age_sec:
            return None
        return self._pose

    def _fresh_seed(self, now):
        if self._seed is None:
            return None
        received, value = self._seed
        if now - received > self.seed_max_age_sec:
            return None
        # Above seed_max_valid_m is costmap_boundary_node's "no occupied cell
        # within max_range_m" sentinel (max_range_m + 1.0), not a distance. A
        # non-positive value cannot be a wall ahead of the car.
        if not math.isfinite(value) or value <= 0.0 or value > self.seed_max_valid_m:
            return None
        return received, value

    def _predict(self, now, pose):
        """Dead-reckoned (distance, normal_angle) from the anchor, or None.

        d = d0 - (p - p0) . n_world. That is the closing-rate integral of
        ds * cos(theta) in closed form: it uses the odometry POSITION change
        projected on the wall normal, never the path length. Path length is
        only right when driving square at the wall; during a wall_turn it
        over-counts closing and fires the turn early."""
        anchor = self._anchor
        if anchor is None or anchor.pose is None or pose is None:
            return None
        if now - anchor.time > self.stale_age_sec:
            return None
        c = math.cos(anchor.world_normal)
        s = math.sin(anchor.world_normal)
        closing = (pose.x - anchor.pose.x) * c + (pose.y - anchor.pose.y) * s
        return anchor.distance - closing, _wrap(anchor.world_normal - pose.yaw)

    def step(self, now, fit, reason):
        now = float(now)
        pose = self._fresh_pose(now)
        prediction = self._predict(now, pose)

        predicted_distance = prediction[0] if prediction is not None else math.nan
        innovation = math.nan
        if reason == REASON_OK and prediction is not None:
            innovation = fit.distance - predicted_distance
            # THE WRONG-SURFACE GATE. Not over-engineering -- do not remove it.
            # On the archived runs, 28% of fit losses (0.3-3 s) came back on a
            # surface more than 2 m from where dead reckoning put the wall: the
            # fit had landed on an object face, and snapping to it would report
            # the object as the wall. Without this gate, snap-on-reacquisition
            # is actively unsafe. 0.30 m sits in the valley of the recorded
            # innovation histogram (same-wall reacquisitions -16/+6 cm p10/p90).
            # It is applied only against a prediction from a real measurement,
            # never against a seed, and lapses when that prediction ages out.
            if abs(innovation) > self.wrong_surface_gate_m:
                reason = REASON_WRONG_SURFACE

        previous = self._previous
        reacquisition = None
        if reason == REASON_OK:
            if previous is None or previous.provenance != PROVENANCE_MEASURED:
                had_value = previous is not None and previous.valid
                reacquisition = Reacquisition(
                    previous_provenance=(
                        previous.provenance if previous is not None else PROVENANCE_NONE),
                    previous_distance=previous.distance if had_value else math.nan,
                    jump=fit.distance - previous.distance if had_value else math.nan,
                    since_last_measurement=(
                        now - self._anchor.time if self._anchor is not None else math.nan),
                )
            # Snap, never blend: the jump is the drift diagnostic.
            self._anchor = _Anchor(
                time=now,
                distance=fit.distance,
                world_normal=pose.yaw + fit.normal_angle if pose is not None else None,
                pose=pose,
            )
            estimate = Estimate(PROVENANCE_MEASURED, True, fit.distance, fit.normal_angle, 0.0)
        elif prediction is not None:
            estimate = Estimate(PROVENANCE_DEAD_RECKONED, True, prediction[0], prediction[1],
                                now - self._anchor.time)
        else:
            seed = self._fresh_seed(now)
            if seed is not None:
                estimate = Estimate(PROVENANCE_SEEDED, True, seed[1], math.nan, now - seed[0])
            else:
                since = now - self._anchor.time if self._anchor is not None else math.nan
                estimate = Estimate(PROVENANCE_NONE, False, math.nan, math.nan, since)

        self._previous = estimate
        return Step(reason, predicted_distance, innovation, estimate, reacquisition,
                    pose is not None)
