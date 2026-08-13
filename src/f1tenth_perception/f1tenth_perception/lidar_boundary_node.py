#!/usr/bin/env python3
"""
lidar_boundary_node.py

Fits a single nearest boundary line per side (left/right) from raw /scan
returns and publishes them as hard linear constraints for mpc_controller's
OSQP/RTI solver -- complements wall_detector_node's own ZED-based front
wall boundary (/perception/front_wall_boundary): together they cover
front (ZED plane fit) + left/right (this node, lidar line fit), all three
consumed uniformly by mpc_controller as f1tenth_messages/BoundaryConstraint
(see that message's own field comments for the shared halfspace
convention: normal . (x, y) <= offset, normal pointing AWAY from the
robot toward the obstacle side).

Publishes /perception/lidar_boundaries (f1tenth_messages/BoundaryConstraintArray,
0-2 entries: left, right, either/both omitted if that side's fit doesn't
clear the quality gate this tick).

Lidar-frame -> car-frame angle conversion: the lidar is mounted rear-
facing (yaw=pi relative to base_link) -- confirmed against f1tenth_bringup/
config/sensors.yaml (angle_min/max = +-3.14, a full-circle scan) and the
base_link->laser static transform, the SAME fact f1tenth_behavior's
IsProximityTooClose._scan_callback already relies on (see that file's own
module docstring for the original verification) -- reused here as the
same +pi offset, not re-derived independently. Also matching
IsProximityTooClose's own established simplification: works directly in
raw angle/range terms, no tf2 lookup for the lidar's own small physical
translation offset from base_link (this rig's lidar mount sits close
enough to base_link's own origin that this offset is treated as
negligible, same judgment call IsProximityTooClose already made for its
own raw-range proximity check -- not a fresh, independent approximation).

Angle windows (car frame, 0 = front, + = left, matching WallDetection's
own bearing convention): left ~45-135 deg, right ~-135 to -45 deg.
Deliberately excludes the ~90 deg front cone (already covered by ZED's
own front_wall_boundary -- including lidar-derived front points here
would double up on, and could disagree with, that source) and the rear
(behind the robot, not relevant to a forward-driving MPC's own hard
boundary constraints).

Robust line fit: total-least-squares (SVD, same technique
wall_detector_node's own _refit_plane_least_squares uses for the analogous
3D-plane case) plus ONE outlier-rejection pass (refit after dropping
points whose perpendicular residual from the initial fit exceeds
outlier_threshold_m) -- not full RANSAC/split-and-merge multi-line
detection, since only the single nearest boundary line per side is
needed, not full wall mapping. Quality-gated (minimum inlier count + RMS
residual floor) before publishing -- mirrors wall_detector_node's own
Part A raw-detection-stage gating philosophy (see that file's module
docstring): a side that doesn't clear the gate this tick is OMITTED
entirely from the published array, never a garbage/best-effort fit.

KNOWN LIMITATION (single-pass, not iterative): outlier rejection uses the
FIRST fit's own residuals to decide what to drop, so a handful of severe
leverage points -- far enough off the true line to meaningfully skew that
first fit's own direction -- can defeat rejection (a well-known weakness
of any single-pass least-squares-based robustification, not specific to
this implementation). Acceptable here because this targets ordinary
stray-return noise (a few points a few centimeters off), not adversarial
or wildly-far outliers -- full RANSAC would handle the latter but is
explicitly out of scope (see above: single nearest line per side, not
full wall mapping).

All quality-gate parameter defaults (min_inliers=20, max_residual_m=0.05,
outlier_threshold_m=0.05) are REASONED STARTING POINTS, not tuned against
real lidar noise -- same discipline wall_detector_node's own Part A gates
followed, flagged deliberately rather than silently. Live validation is
pending (see wall_detector_node.py's own note on the ZED being blocked --
this node has never been validated against a live Hokuyo feed either).

Temporal tracking (added by the "Boundary detection hardening" pass, after
a live bag capture -- boundary_constraint_diag_20260813_124824 --
confirmed a real bug here: with no smoothing at all, each side's line fit
was published directly from a single frame's raw geometry, and the bag
showed genuine 0.017-0.5m frame-to-frame offset flicker with 25 present/
absent transitions in under 9 seconds. mpc_solver's own OSQP QP correctly
detected this as producing infeasible constraint sets on some ticks -- a
real, not hypothetical, consequence of publishing single-frame noise
directly as a HARD constraint). Mirrors wall_detector_node's own
TrackedWall/WallTracker pattern (EMA smoothing + hold-before-drop), scoped
down for two reasons specific to this node:

  - A LINE, not a plane: TrackedBoundaryLine holds (normal, offset) --
    the same halfspace representation fit_side_boundary already returns --
    not TrackedWall's fuller (distance, bearing, normal, centroid,
    is_corner, extent) shape, since nothing downstream of this node needs
    the extra fields.
  - AT MOST ONE live track per side, not a general N-track list:
    fit_side_boundary already scopes to "the single nearest line for this
    side", so there is never more than one raw per-frame candidate to
    reconcile against the existing track. "Association" (see module
    docstring for WallTracker's own general, multi-candidate version)
    therefore reduces to a simple existence check here -- a successful fit
    always blends into that side's existing track (if any) or seeds a
    fresh one (if not); there is no candidate-vs-candidate disambiguation
    to do, and no separate reject-threshold parameter is needed for it
    (deliberately not added, not an oversight).

EMA smoothing reuses wall_ema_alpha's own formula (smoothed = alpha*new +
(1-alpha)*smoothed_prev, unit-normal blended-then-renormalized exactly as
TrackedWall.update() already does) and, by default, its own tuned VALUE
(lidar_boundary_ema_alpha defaults to wall_ema_alpha's 0.3) -- reusing a
value already reasoned about for this exact class of problem rather than
guessing a new one for a structurally similar smoothing job.

Publish gate: mirrors front_clearance selection eligibility's own
stability bar -- a side's track must reach lidar_boundary_min_frames_to_
publish (defaults to 5, the same frame-count front_clearance_min_track_
frames/ambiguous_confirm_frames already established as this codebase's
precedent for "distinguish real persistence from transient noise") before
its BoundaryConstraint is published at all. A single good frame can no
longer alone produce a hard constraint.

Hold-before-drop: a side that fails to fit THIS frame keeps its track
alive (last smoothed value held, not zeroed) for up to lidar_boundary_
hold_frames (defaults to 5, reusing wall_track_hold_frames's own value)
consecutive misses before actually dropping to "no constraint for this
side" -- this directly targets the bag's own 25-transitions-in-9s
flicker pattern, where a side's raw fit was intermittently failing the
quality gate for isolated frames while the physical wall never actually
moved or disappeared.
"""

import math

import numpy as np
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan

from f1tenth_messages.msg import BoundaryConstraint, BoundaryConstraintArray


# ==============================================================================
# Angle/point conversion -- pure functions, no rclpy dependency, independently
# unit-testable. See module docstring for the rear-facing-mount fact this
# reuses (not re-derives) from IsProximityTooClose.
# ==============================================================================

def _wrap_angle(angle: float) -> float:
    """Wrap `angle` (radians) into (-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


def _scan_angle_to_car_frame(raw_angle: float) -> float:
    """Convert a raw /scan angle (msg.angle_min + i*angle_increment, where
    angle 0 is the lidar's OWN forward axis) into car/base_link-frame
    angle (0 = car's front, + = left) -- the lidar is physically mounted
    rear-facing (yaw=pi), see module docstring."""
    return _wrap_angle(raw_angle + math.pi)


def _scan_point_car_xy(car_angle: float, range_val: float):
    """(x, y) of one lidar return in car frame (x forward, y left), given
    its ALREADY-car-frame angle (see _scan_angle_to_car_frame) and range."""
    return range_val * math.cos(car_angle), range_val * math.sin(car_angle)


def _classify_side(car_angle: float, window_min_rad: float, window_max_rad: float):
    """Which side (if either) `car_angle` (already car-frame, see
    _scan_angle_to_car_frame) falls into: 'left' for
    [window_min_rad, window_max_rad], 'right' for the mirrored negative
    range, None otherwise (the front cone, |car_angle| < window_min_rad,
    or the rear, |car_angle| > window_max_rad -- see module docstring for
    why both are deliberately excluded)."""
    if window_min_rad <= car_angle <= window_max_rad:
        return 'left'
    if -window_max_rad <= car_angle <= -window_min_rad:
        return 'right'
    return None


# ==============================================================================
# Robust line fit -- pure functions, no rclpy dependency, independently
# unit-testable. See module docstring for the outlier-rejection/quality-gate
# design.
# ==============================================================================

def _refit_line_least_squares(points: np.ndarray):
    """2D total-least-squares line fit via SVD -- centroid + the smallest-
    singular-value right-singular-vector as the unit NORMAL (perpendicular
    to the line's own direction). Same technique wall_detector_node's own
    _refit_plane_least_squares uses for the analogous 3D-plane case (most
    variance IS along the line; least variance is perpendicular to it)."""
    centroid = points.mean(axis=0)
    centered = points - centroid
    _, _, vt = np.linalg.svd(centered, full_matrices=False)
    normal = vt[-1]
    normal = normal / np.linalg.norm(normal)
    return normal, centroid


def fit_side_boundary(
        points: np.ndarray, min_inliers: int, max_residual_m: float,
        outlier_threshold_m: float = 0.05):
    """Robust line fit through `points` (Nx2, car-frame XY, already
    windowed to one side) -> (normal_x, normal_y, offset) BoundaryConstraint
    tuple, or None if the fit doesn't clear the quality gate. See module
    docstring's "Robust line fit" section for the full design.

    Sign convention matches BoundaryConstraint.msg / wall_detector_node's
    own derivation directly: normal oriented AWAY from the robot's own
    origin (0, 0) -- offset = dot(normal, centroid) after that
    reorientation, guaranteed >= 0 (the perpendicular distance from the
    robot's own origin to the fitted line)."""
    if len(points) < min_inliers:
        return None

    normal, centroid = _refit_line_least_squares(points)
    residuals = np.abs((points - centroid) @ normal)
    inliers = points[residuals <= outlier_threshold_m]
    if len(inliers) < min_inliers:
        return None

    normal, centroid = _refit_line_least_squares(inliers)
    residuals = np.abs((inliers - centroid) @ normal)
    rms_residual = float(np.sqrt(np.mean(residuals ** 2)))
    if rms_residual > max_residual_m:
        return None

    offset = float(np.dot(normal, centroid))
    if offset < 0:
        normal = -normal
        offset = -offset
    return float(normal[0]), float(normal[1]), offset


# ==============================================================================
# Temporal tracking -- pure functions/classes, no rclpy dependency,
# independently unit-testable. See module docstring's "Temporal tracking"
# section for the full design (mirrors, scoped down, wall_detector_node's
# own TrackedWall/WallTracker EMA-smoothing + hold-before-drop pattern).
# ==============================================================================

class TrackedBoundaryLine:
    """Smoothed, held state for ONE side's boundary line. Seeded directly
    (no blending) from the first accepted fit_side_boundary result for that
    side; every later matched frame blends in via update()."""

    def __init__(self, normal, offset: float):
        self.normal = (float(normal[0]), float(normal[1]))
        self.offset = float(offset)
        # 1 on the seeding frame itself -- matches TrackedWall's own
        # convention (this track has been matched exactly once so far).
        self.frames_matched = 1
        # 0 immediately after being seen (spawned or matched this frame);
        # counts consecutive missed frames since -- see update_side_track's
        # own hold/drop bookkeeping.
        self.frames_since_seen = 0

    def update(self, normal, offset: float, alpha: float) -> None:
        """Blend this frame's matched raw fit into the smoothed state --
        same EMA formula and unit-normal blend-then-renormalize technique
        TrackedWall.update() already uses for the analogous 3D-plane case."""
        blended_nx = alpha * float(normal[0]) + (1.0 - alpha) * self.normal[0]
        blended_ny = alpha * float(normal[1]) + (1.0 - alpha) * self.normal[1]
        norm_mag = math.hypot(blended_nx, blended_ny)
        if norm_mag > 1e-9:
            self.normal = (blended_nx / norm_mag, blended_ny / norm_mag)
        # else: pathological (new and old normals exactly antipodal, alpha
        # exactly 0.5) -- keep the previous normal rather than divide by
        # ~zero, same guard TrackedWall.update() uses; practically
        # unreachable for real consecutive-frame data.
        self.offset = alpha * float(offset) + (1.0 - alpha) * self.offset
        self.frames_matched += 1
        self.frames_since_seen = 0


def update_side_track(track, fit_result, alpha: float, hold_frames: int):
    """Advance ONE side's track state by one tick. `track`: the existing
    TrackedBoundaryLine for this side, or None (no live track yet/track
    already dropped). `fit_result`: this frame's (nx, ny, offset) tuple
    from fit_side_boundary, or None if this side's raw fit didn't clear the
    quality gate this tick. Returns the new track state (TrackedBoundaryLine
    or None) to carry into the next tick.

    Association is deliberately simple here (see module docstring): at most
    one raw candidate ever exists per side per frame, so a successful fit
    always blends into (or seeds) that side's own track -- there is no
    multi-candidate matching to do. The only real decision is what happens
    on a MISS (fit_result is None): hold the existing track (if any) for up
    to hold_frames consecutive misses before dropping it to None."""
    if fit_result is not None:
        nx, ny, offset = fit_result
        if track is None:
            return TrackedBoundaryLine((nx, ny), offset)
        track.update((nx, ny), offset, alpha)
        return track

    if track is None:
        return None
    track.frames_since_seen += 1
    if track.frames_since_seen <= hold_frames:
        return track
    return None  # exceeded the hold window -- dropped, not carried forward.


def side_track_eligible(track, min_frames_to_publish: int) -> bool:
    """True if `track` (a TrackedBoundaryLine or None) has accumulated
    enough matched frames to be trusted for a published hard constraint --
    see module docstring's "Publish gate" section. A track currently being
    held through a miss (frames_since_seen > 0) is still eligible as long
    as its frames_matched count (which misses don't reset) still clears the
    bar -- holding is about NOT instantly withdrawing a good constraint,
    not about re-litigating whether it was ever trustworthy."""
    return track is not None and track.frames_matched >= min_frames_to_publish


class LidarBoundaryNode(Node):
    def __init__(self):
        super().__init__('lidar_boundary_node')

        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('robot_frame', 'base_link')
        # deg, car frame, 0 = front, + = left -- see module docstring.
        self.declare_parameter('side_window_min_deg', 45.0)
        self.declare_parameter('side_window_max_deg', 135.0)
        # Quality gate -- REASONED STARTING POINTS, see module docstring.
        self.declare_parameter('min_inliers', 20)
        self.declare_parameter('max_residual_m', 0.05)
        self.declare_parameter('outlier_threshold_m', 0.05)
        # Temporal tracking -- see module docstring's own section + stack_
        # params.yaml's matching keys for the reused-value reasoning.
        self.declare_parameter('ema_alpha', 0.3)
        self.declare_parameter('min_frames_to_publish', 5)
        self.declare_parameter('hold_frames', 5)

        p = self.get_parameter
        self.robot_frame = p('robot_frame').value
        self.side_window_min_rad = math.radians(p('side_window_min_deg').value)
        self.side_window_max_rad = math.radians(p('side_window_max_deg').value)
        self.min_inliers = p('min_inliers').value
        self.max_residual_m = p('max_residual_m').value
        self.outlier_threshold_m = p('outlier_threshold_m').value
        self.ema_alpha = p('ema_alpha').value
        self.min_frames_to_publish = p('min_frames_to_publish').value
        self.hold_frames = p('hold_frames').value

        # This side's live TrackedBoundaryLine, or None -- see module
        # docstring's "Temporal tracking" section. Carried across
        # _scan_callback ticks (one Node instance, one lidar -- no need for
        # per-message state beyond this).
        self._left_track = None
        self._right_track = None

        self.sub = self.create_subscription(
            LaserScan, p('scan_topic').value, self._scan_callback, 10)
        self.pub = self.create_publisher(
            BoundaryConstraintArray, '/perception/lidar_boundaries', 10)

        self.get_logger().info('lidar_boundary_node started')

    # ------------------------------------------------------------------
    def _scan_callback(self, msg: LaserScan):
        left_pts = []
        right_pts = []
        for i, r in enumerate(msg.ranges):
            # Same validity filter IsProximityTooClose._scan_callback
            # already uses: inf/nan and anything below range_min are
            # skipped before any windowing, never treated as a valid
            # near-zero hit.
            if not math.isfinite(r) or r < msg.range_min:
                continue
            raw_angle = msg.angle_min + i * msg.angle_increment
            car_angle = _scan_angle_to_car_frame(raw_angle)
            side = _classify_side(car_angle, self.side_window_min_rad, self.side_window_max_rad)
            if side == 'left':
                left_pts.append(_scan_point_car_xy(car_angle, r))
            elif side == 'right':
                right_pts.append(_scan_point_car_xy(car_angle, r))

        constraints = []
        for side_name, pts in (('left', left_pts), ('right', right_pts)):
            result = None
            if len(pts) >= self.min_inliers:
                result = fit_side_boundary(
                    np.asarray(pts, dtype=float), self.min_inliers,
                    self.max_residual_m, self.outlier_threshold_m)

            # Route this frame's raw fit (or miss) through the temporal
            # tracker BEFORE it's allowed anywhere near the published
            # array -- see module docstring's "Temporal tracking" section.
            # Every consumer below reads the TRACKED/SMOOTHED state, not
            # the raw per-frame fit, same discipline wall_detector_node's
            # own _publish() already follows for TrackedWall.
            if side_name == 'left':
                self._left_track = update_side_track(
                    self._left_track, result, self.ema_alpha, self.hold_frames)
                track = self._left_track
            else:
                self._right_track = update_side_track(
                    self._right_track, result, self.ema_alpha, self.hold_frames)
                track = self._right_track

            eligible = side_track_eligible(track, self.min_frames_to_publish)
            self.get_logger().info(
                f'[LIDAR_BOUNDARY] {side_name}: n_points={len(pts)} '
                f'(need >={self.min_inliers}) -> raw_fit='
                + ('accepted' if result is not None else 'REJECTED') + ', '
                + (
                    f'track frames_matched={track.frames_matched} '
                    f'(need >={self.min_frames_to_publish}) '
                    f'frames_since_seen={track.frames_since_seen} '
                    f'(hold<={self.hold_frames})' if track is not None else 'no live track'
                )
                + f' -> published={eligible}',
                throttle_duration_sec=1.0)
            if eligible:
                nx, ny = track.normal
                constraints.append(
                    BoundaryConstraint(normal=[nx, ny], offset=track.offset))

        arr = BoundaryConstraintArray()
        arr.header = msg.header
        arr.header.frame_id = self.robot_frame
        arr.constraints = constraints
        self.pub.publish(arr)


def main(args=None):
    rclpy.init(args=args)
    node = LidarBoundaryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
