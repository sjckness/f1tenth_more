"""Semantic layer -- pure functions/classes, no rclpy dependency, independently
unit-testable (same "pure logic separate from ROS glue" convention this
codebase's other accumulation/tracking modules already use -- see
f1tenth_perception/wall_detector_node.py's WallTracker, lidar_boundary_node.py's
TrackedBoundaryLine). Accumulates detected-object positions into MAP-FRAME
persistent state.

Source choice (/camera/detections_3d, NOT /perception/obstacles_2d) -- see
semantic_layer_node.py's own module docstring for the full reasoning:
obstacles_2d (f1tenth_messages/Obstacle2DArray) carries NO class label at all
(Obstacle2D.msg is x/y/r only, stripped by obstacle_projector_node's own
projection stage) -- structurally unable to feed a genuinely SEMANTIC,
class-aware layer. detections_3d (vision_msgs/Detection3DArray) carries
class_id + score per detection, at the cost of being raw/per-frame (not
already deduplicated the way obstacles_2d is).

Two-hop transform per detection (see semantic_layer_node.py's own docstring
for why both hops are necessary, and why hop 2 is NOT a tf2 lookup):
  1. camera_frame -> base_link: a real tf2 lookup, done by the NODE (not pure
     -- needs a live TF buffer), before any function here is called.
  2. base_link -> map: _compose_base_link_to_map, pure, below -- applied from
     slam_toolbox's own /slam/pose message (temporally-nearest to the
     detection's own stamp), NOT a tf2 lookup (no map->odom edge exists in
     the TF tree at all -- see slam.launch.py's own module docstring for why
     that's deliberate, not a missing feature).

Real per-frame tracking (batch-association pass): the original implementation
merged each detection one at a time, in arrival order, against whatever
existed so far (merge_or_add_object(), removed by this pass -- see git
history if that simpler version is ever needed again). Live testing kept
showing duplicate/multiplying objects despite that working correctly on its
own 22-test suite -- diagnosed as an architecture problem, not a threshold
one: matching detections one at a time in arrival order can split what
should be one assignment (e.g. detection A steals the nearest track before
detection B gets a look, even though B was the better match), and matching
against a track's last RAW position (not where it's predicted to be now)
means genuine motion gets treated identically to "this might be a new
object" -- there's no way to tell those apart from a single distance check
alone. Replaced by update_tracks_batch() below:
  - Whole-frame assignment (scipy.optimize.linear_sum_assignment -- already
    a real dependency elsewhere in this workspace, e.g. mpc_solver.py; a
    hand-rolled greedy-nearest-first was the documented fallback option but
    since scipy's already paid for, the real (optimal, not approximate)
    Hungarian solve is strictly better for the same cost) instead of
    one-at-a-time greedy -- avoids the split-assignment failure mode above.
  - Per-object motion model (EMA on position delta -- SemanticObject.predict()
    below): matching compares each detection against where a track is
    PREDICTED to be at the detection's own timestamp, not its last known
    position, so real motion no longer competes with "new object" under the
    same distance check.
  - confirm/lost lifecycle (SemanticObject.hit_streak/miss_streak/confirmed):
    a track only gets published once confirm_hit_count CONSECUTIVE hits land
    (one-off false detections never survive long enough to become a
    lingering phantom marker) and gets pruned entirely after
    lost_miss_count CONSECUTIVE misses (the "is it still there" half of the
    original ask -- nothing pruned tracks before this pass at all).
Class-gating (same class_id required to match at all) is UNCHANGED from the
original merge_or_add_object -- a cone and a person in roughly the same spot
are still two distinct objects, not one. Whether frame-to-frame class-label
flicker on the SAME physical object (the model calling it two different
classes across frames) is also contributing to the duplicate-object symptom
is a separate, still-open hypothesis this pass does not address -- gating
stays strict class-match here regardless.
"""

import math

from scipy.optimize import linear_sum_assignment

# Real detection/track pairs never cost anywhere near this (it's derived from
# max_distance_m, itself normally well under a meter) -- large enough that
# linear_sum_assignment always prefers any real pairing over one of these,
# small enough to never overflow/behave oddly as a plain float. Only matters
# when the assignment is forced to fully pair up min(n_det, n_trk) items even
# though some of those forced pairs are class-mismatched or out of gate --
# every such pair is rejected again explicitly by cost after the solve
# (belt-and-braces, not load-bearing on its own -- see update_tracks_batch).
_INFEASIBLE_SENTINEL_MULTIPLIER = 1000.0

# predict()'s own dt is clamped to this -- a track missed for a long time
# (well past when it would already have been pruned by a sane
# lost_miss_count) must not extrapolate its position arbitrarily far from a
# stale velocity estimate; capping dt bounds how far a prediction can drift
# even in a pathological case.
_MAX_PREDICTION_DT_SEC = 2.0


def pose_to_xytheta(position, orientation):
    """Extract (x, y, yaw) from a geometry_msgs Pose's position/orientation
    (each just needs .x/.y/.z/.w attributes -- works for a real Pose message
    or a plain duck-typed stand-in, e.g. in tests). Same planar-yaw-from-
    quaternion formula used throughout this codebase (MPC_corr.py's own
    quaternion_to_yaw(), check_stop_condition.py's own _quaternion_to_yaw())
    -- valid for the roll=pitch=0 planar case this stack always operates in."""
    yaw = math.atan2(
        2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
        1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z),
    )
    return float(position.x), float(position.y), yaw


def compose_base_link_to_map(x_bl: float, y_bl: float, map_pose_xytheta) -> tuple:
    """Transform a point (x_bl, y_bl) expressed in base_link frame into map
    frame, given the robot's own (x, y, yaw) pose IN map frame (map_pose_
    xytheta, from slam_toolbox's /slam/pose -- see module docstring). Standard
    rigid-transform composition: rotate the base_link-frame offset by the
    robot's own map-frame heading, then translate by the robot's own map-frame
    position -- the same "point in a moving frame -> fixed frame" pattern
    wall_detector_node.py's own _transform_to_robot_frame uses for the
    analogous camera-frame -> base_link step (there via tf2's own rotation
    matrix; here via plain 2D rotation, since map_pose_xytheta is already a
    flat (x, y, yaw), not a full quaternion+translation tf2 Transform)."""
    x_r, y_r, yaw_r = map_pose_xytheta
    cos_yaw, sin_yaw = math.cos(yaw_r), math.sin(yaw_r)
    x_map = x_r + cos_yaw * x_bl - sin_yaw * y_bl
    y_map = y_r + sin_yaw * x_bl + cos_yaw * y_bl
    return x_map, y_map


class SemanticObject:
    """One tracked, map-frame semantic detection. EMA-blended position AND
    velocity on every real (matched) update -- same alpha-blend formula
    TrackedWall.update()/TrackedBoundaryLine.update() already use elsewhere
    in this codebase for position, extended here to also blend a velocity
    estimate (see predict()). No expiry by staleness/wall-clock time -- track
    lifecycle is purely hit/miss-streak driven (confirmed/lost), matched
    frame-by-frame in update_tracks_batch(), not a separate timeout.

    track_id is caller-assigned (see update_tracks_batch's next_track_id
    param) and is the ONLY thing marker identity should ever be keyed off of
    now that tracks can be pruned (semantic_layer_node.py's old id scheme
    was list-position-derived, which silently breaks once removal makes list
    position no longer stable across ticks)."""

    def __init__(
            self, track_id, class_id: str, x_map: float, y_map: float, score: float,
            stamp_sec: float, confirm_hit_count: int):
        self.track_id = track_id
        self.class_id = class_id
        self.x_map = float(x_map)
        self.y_map = float(y_map)
        self.score = float(score)
        self.vx_map = 0.0
        self.vy_map = 0.0
        self.last_update_stamp_sec = float(stamp_sec)
        # hit_count: total hits ever (diagnostic/label use only, e.g. the
        # marker label's own "(N)" suffix -- unaffected by streak resets).
        self.hit_count = 1
        # hit_streak/miss_streak: CONSECUTIVE, each resets the other to 0 --
        # this is what confirm/lost actually gate on, not hit_count.
        self.hit_streak = 1
        self.miss_streak = 0
        self._confirm_hit_count = confirm_hit_count
        # confirm_hit_count <= 1 means "confirmed on first sighting" (the
        # pre-lifecycle behavior) -- supported explicitly rather than as an
        # off-by-one accident, since a caller might reasonably want that.
        self.confirmed = confirm_hit_count <= 1

    def predict(self, stamp_sec: float) -> tuple:
        """Where this track is expected to be AT stamp_sec, extrapolated from
        its last known position + blended velocity -- this, not the last raw
        position, is what update_tracks_batch() matches new detections
        against, so genuine motion doesn't compete with "is this a new
        object" under the same distance check (see module docstring)."""
        dt = stamp_sec - self.last_update_stamp_sec
        dt = max(0.0, min(dt, _MAX_PREDICTION_DT_SEC))
        return self.x_map + self.vx_map * dt, self.y_map + self.vy_map * dt

    def update(self, x_map: float, y_map: float, score: float, stamp_sec: float, alpha: float):
        """Apply one real (matched) detection: EMA-blend both position and
        velocity from the RAW last stored state (not the predicted one --
        blending against a prediction that already incorporates the motion
        model would double-apply it), advance hit/miss streaks, and confirm
        once confirm_hit_count consecutive hits have landed."""
        dt = stamp_sec - self.last_update_stamp_sec
        if dt > 0.0:
            raw_vx = (float(x_map) - self.x_map) / dt
            raw_vy = (float(y_map) - self.y_map) / dt
            self.vx_map = alpha * raw_vx + (1.0 - alpha) * self.vx_map
            self.vy_map = alpha * raw_vy + (1.0 - alpha) * self.vy_map
        # dt <= 0 (out-of-order/duplicate-timestamp input): position is still
        # blended below, velocity just isn't re-estimated off a non-positive
        # dt -- keeps the existing velocity estimate rather than divide-by-
        # zero or a garbage sign flip.
        self.x_map = alpha * float(x_map) + (1.0 - alpha) * self.x_map
        self.y_map = alpha * float(y_map) + (1.0 - alpha) * self.y_map
        self.score = float(score)
        self.last_update_stamp_sec = float(stamp_sec)
        self.hit_count += 1
        self.hit_streak += 1
        self.miss_streak = 0
        if self.hit_streak >= self._confirm_hit_count:
            self.confirmed = True

    def mark_missed(self):
        """No detection matched this track this frame."""
        self.hit_streak = 0
        self.miss_streak += 1


def update_tracks_batch(
        tracks: list, detections: list, stamp_sec: float,
        max_distance_m: float, alpha: float, confirm_hit_count: int,
        lost_miss_count: int, next_track_id) -> list:
    """One frame's worth of association + lifecycle, replacing the old
    per-detection merge_or_add_object() (see module docstring). Mutates
    `tracks` in place where possible and returns the (possibly shorter --
    pruned) list; matches this module's existing mutate-and-return
    convention.

    tracks: list of SemanticObject (the accumulated state, across calls).
    detections: list of (class_id, x_map, y_map, score) tuples, ALL of this
        frame's detections, already transformed into map frame by the caller
        -- batching is the whole point (see module docstring's "split-
        assignment failure mode"), so this must be the full frame, not one
        detection at a time.
    stamp_sec: this frame's own timestamp (detection batch's header.stamp,
        NOT wall-clock "now") -- used for both predict() (where each track
        is expected to be AT this instant) and as update()'s new
        last_update_stamp_sec on a match.
    next_track_id: zero-arg callable returning a fresh, never-reused id for
        a newly spawned SemanticObject -- caller-owned counter (see
        semantic_layer_node.py's own _next_track_id) so this function stays
        a pure function of its arguments, no hidden global state here.

    Association: same-class-only (class mismatch is never a valid pair,
    unchanged from the original merge_or_add_object -- see module docstring)
    and within max_distance_m of the track's PREDICTED (not last raw)
    position, solved as a whole-frame optimal assignment (scipy Hungarian)
    rather than one detection at a time in arrival order.

    Lifecycle: a matched track calls update() (confirms once
    confirm_hit_count consecutive hits land); an unmatched track calls
    mark_missed() and is dropped entirely once its miss_streak reaches
    lost_miss_count. An unmatched detection spawns a fresh (not yet
    confirmed) track.
    """
    if not tracks:
        for class_id, x_map, y_map, score in detections:
            tracks.append(SemanticObject(
                next_track_id(), class_id, x_map, y_map, score, stamp_sec, confirm_hit_count))
        return tracks

    if not detections:
        for trk in tracks:
            trk.mark_missed()
        return [t for t in tracks if t.miss_streak < lost_miss_count]

    n_det, n_trk = len(detections), len(tracks)
    # cost[i][j]: real distance if (detection i, track j) is a legal pairing
    # (same class, within max_distance_m of track j's predicted position at
    # stamp_sec), else math.inf -- checked again explicitly after the solve
    # (see _INFEASIBLE_SENTINEL_MULTIPLIER's own comment for why a sentinel
    # substitute is needed for the solver call itself, separate from this).
    cost = [[math.inf] * n_trk for _ in range(n_det)]
    for i, (class_id, x_map, y_map, score) in enumerate(detections):
        for j, trk in enumerate(tracks):
            if trk.class_id != class_id:
                continue
            px, py = trk.predict(stamp_sec)
            dist = math.hypot(px - x_map, py - y_map)
            if dist <= max_distance_m:
                cost[i][j] = dist

    sentinel = max_distance_m * _INFEASIBLE_SENTINEL_MULTIPLIER + 1e6
    solver_matrix = [[c if math.isfinite(c) else sentinel for c in row] for row in cost]
    row_ind, col_ind = linear_sum_assignment(solver_matrix)

    matched_det_idx = set()
    matched_trk_idx = set()
    for i, j in zip(row_ind, col_ind):
        if not math.isfinite(cost[i][j]):
            # Forced pairing (the solver must fully assign min(n_det, n_trk)
            # pairs even when every option is bad) but never actually a
            # legal one -- treat both sides as unmatched, same as if this
            # pair had never been proposed at all.
            continue
        class_id, x_map, y_map, score = detections[i]
        tracks[j].update(x_map, y_map, score, stamp_sec, alpha)
        matched_det_idx.add(i)
        matched_trk_idx.add(j)

    for j, trk in enumerate(tracks):
        if j not in matched_trk_idx:
            trk.mark_missed()

    for i, (class_id, x_map, y_map, score) in enumerate(detections):
        if i not in matched_det_idx:
            tracks.append(SemanticObject(
                next_track_id(), class_id, x_map, y_map, score, stamp_sec, confirm_hit_count))

    return [t for t in tracks if t.miss_streak < lost_miss_count]


def find_nearest_pose_by_stamp(pose_history: list, target_stamp_sec: float):
    """pose_history: list of (stamp_sec, (x, y, yaw)) tuples, most-recent-last
    (see semantic_layer_node.py's own bounded-deque usage). Returns the
    (x, y, yaw) entry whose stamp is temporally nearest to target_stamp_sec --
    "at the time it was seen" (matching each detection against the pose that
    was actually current when the CAMERA captured it, not whatever pose
    happens to be latest by the time this node's callback runs -- a real,
    if usually small, distinction at any nonzero processing latency). None if
    pose_history is empty."""
    if not pose_history:
        return None
    return min(pose_history, key=lambda entry: abs(entry[0] - target_stamp_sec))[1]
