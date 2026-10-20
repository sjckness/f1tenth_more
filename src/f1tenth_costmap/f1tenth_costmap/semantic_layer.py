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
are still two distinct objects, not one -- with ONE exception since obstacles
v2: a track of class "unknown" (a LiDAR obstacle nothing recognised,
unknown_objects.py) may be matched by a detection of ANY class, and is then
UPGRADED in place: same track_id, the detection's class from then on.
Whether frame-to-frame class-label flicker on the SAME physical object (the
model calling it two different classes across frames) is also contributing to
the duplicate-object symptom is a separate, still-open hypothesis this pass
does not address -- gating stays strict class-match here regardless.

OBJECT MEMORY (Part B, object_memory.py). With prune_confirmed=False a
CONFIRMED track is no longer dropped at lost_miss_count: it stays in the list
and object_memory.memory_cycle() decides when it goes, from what the sensors
could see (in view and undetected for fov_miss_remove_s, or out of view for
memory_decay_s). Staying in the list is what lets a remembered object that is
detected again keep its id: it is an ordinary association candidate, matched
at its stored position (memory zeroes its velocity) with a gate widened by its
extra_gate_m. live_tracks() is the old pruning predicate, so the outputs that
predate the memory still see exactly the tracks they always saw.

ONE LIST, EACH TRACK PRUNED BY ITS OWN SENSOR (merge of the two, 2026-10-20).
Every track -- recognised or unknown, tentative, confirmed or remembered --
is in this one list and is associated by the same two functions: camera
batches by update_tracks_batch (class-gated, an unknown track upgradable),
LiDAR clusters by update_unknown_batch (any class: an unknown track is
updated, a recognised one absorbs its own cluster). Each track is removed by
exactly one rule, chosen by what it is:
  * tentative recognised: lost_miss_count camera misses (update_tracks_batch);
  * confirmed recognised: the object memory (memory_cycle), or
    lost_miss_count when the memory is off;
  * unknown: the LiDAR (update_unknown_batch: a broken run of sightings, or
    decay_s / unobserved_decay_s); camera frames and the memory never touch it.
A recognised track the memory lets go while the LiDAR still hits it is not
dropped: hand_over_to_lidar makes it a confirmed unknown, same id, so the
obstacle goes on without a gap. dedupe_tracks folds an unknown into whatever
track it turns out to be. SemanticObject.radius() is the one footprint radius
every consumer uses; unknown_objects.plausible the one plausibility gate
(height and radius) -- a recognised track that fails it is no obstacle,
owns no cells and absorbs no LiDAR cluster, so what is under it stays seen.
"""

import math
from typing import Optional

from f1tenth_costmap.object_memory import track_base_radius
from f1tenth_costmap.unknown_objects import UNKNOWN_CLASS

import numpy as np

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
    estimate (see predict()). No expiry by staleness/wall-clock time HERE --
    the tracker's own lifecycle is purely hit/miss-streak driven
    (confirmed/lost), matched frame-by-frame in update_tracks_batch(). The
    time- and visibility-based expiry of CONFIRMED tracks is the object
    memory's (object_memory.py), applied when update_tracks_batch is called
    with prune_confirmed=False.

    track_id is caller-assigned (see update_tracks_batch's next_track_id
    param) and is the ONLY thing marker identity should ever be keyed off of
    now that tracks can be pruned (semantic_layer_node.py's old id scheme
    was list-position-derived, which silently breaks once removal makes list
    position no longer stable across ticks)."""

    def __init__(
            self, track_id, class_id: str, x_map: float, y_map: float, score: float,
            stamp_sec: float, confirm_hit_count: int, width: Optional[float] = None,
            z_base: Optional[float] = None):
        self.track_id = track_id
        # Fused footprint WIDTH [m] (obstacle_projector_node's footprint rule,
        # never the height), EMA-blended over associated detections with the
        # same alpha as position. None until a detection carrying a width lands.
        self.width = _valid_width(width)
        # Fused height [m] of the object's CENTRE in base_link (z = 0 is the
        # floor), the quantity obstacle_projector_node's height band tests.
        # Blended like width; None until a detection carrying one lands.
        self.z_base = _valid_z(z_base)
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
        # Added to the association gate for THIS track only. 0 for every
        # tracked object; the object memory sets it to the radius growth of a
        # person in MEMORY (object_memory.apply_memory_to_track), so the gate
        # that re-acquires a remembered person grows with how far they may
        # have walked.
        self.extra_gate_m = 0.0
        # --- obstacles v2: LiDAR evidence (update_unknown_batch) ------------
        #: (length, width) [m] of the LiDAR cluster, unknown tracks only, and
        #: the direction [rad, map] of its length (the latest cluster's).
        self.extent = None
        self.angle = 0.0
        #: Start of the current run of CONSECUTIVE LiDAR sightings: an
        #: unknown track is confirmed once that run spans persist_s.
        self.streak_start_sec = float(stamp_sec)
        #: Last LiDAR cycle a cluster was associated with this track.
        self.last_lidar_hit_sec = None
        #: The LiDAR has seen THROUGH this track's area since its last hit.
        self.seen_through = False
        #: When a detection upgraded this track from 'unknown' (else None).
        self.upgraded_sec = None

    @property
    def is_unknown(self):
        """Whether this track is an unrecognised LiDAR obstacle."""
        return self.class_id == UNKNOWN_CLASS

    def radius(self):
        """
        Footprint radius [m]: the one every consumer uses.

        An unknown track: half its LiDAR cluster's larger extent. A recognised
        one: object_memory.track_base_radius (fused width / 2, else the class's
        nominal footprint), the radius the memory grows and the MPC is given.
        """
        if self.is_unknown and self.extent is not None:
            return 0.5 * max(self.extent)
        return track_base_radius(self)

    def predict(self, stamp_sec: float) -> tuple:
        """Where this track is expected to be AT stamp_sec, extrapolated from
        its last known position + blended velocity -- this, not the last raw
        position, is what update_tracks_batch() matches new detections
        against, so genuine motion doesn't compete with "is this a new
        object" under the same distance check (see module docstring)."""
        dt = stamp_sec - self.last_update_stamp_sec
        dt = max(0.0, min(dt, _MAX_PREDICTION_DT_SEC))
        return self.x_map + self.vx_map * dt, self.y_map + self.vy_map * dt

    def update(self, x_map: float, y_map: float, score: float, stamp_sec: float,
               alpha: float, width: Optional[float] = None, z_base: Optional[float] = None):
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
        width = _valid_width(width)
        if width is not None:
            self.width = width if self.width is None else (
                alpha * width + (1.0 - alpha) * self.width)
        z_base = _valid_z(z_base)
        if z_base is not None:
            self.z_base = z_base if self.z_base is None else (
                alpha * z_base + (1.0 - alpha) * self.z_base)
        self.score = float(score)
        self.last_update_stamp_sec = float(stamp_sec)
        self.hit_count += 1
        self.hit_streak += 1
        self.miss_streak = 0
        if self.hit_streak >= self._confirm_hit_count:
            self.confirmed = True

    def mark_missed(self, confirm_grace_misses: int = 0):
        """No detection matched this track this frame.

        confirm_grace_misses: how many consecutive misses a not-yet-confirmed
        track may absorb WITHOUT having its hit_streak reset. 0 reproduces the
        original behaviour exactly (any miss resets the streak).

        WHY THIS EXISTS (2026-09-01 mission-analysis follow-up): confirmation
        requires confirm_hit_count CONSECUTIVE hits, and this method used to
        zero hit_streak on the very first miss. A real, physically-present
        object frequently could not assemble 3 consecutive hits: it would
        accumulate 1-2 hits, miss, reset, and eventually be pruned at
        lost_miss_count, whereupon the next sighting spawned a brand-new track
        id. That is the mechanism behind the observed churn -- 5 `person` + 4
        `tv` track ids in a 21 s run for what was almost certainly one person
        and one TV.

        CORRECTED 2026-09-02: this was originally justified by the "35-45% of
        detection frames produce no 3D output" figure from the same analysis.
        Measuring that properly showed the figure to be real but almost
        entirely benign for THIS purpose -- 99.2% of frames containing at
        least one detection did produce output; the missing frames were
        overwhelmingly EMPTY ones (only 3.0% of zero-detection frames produced
        output, because yolo_detector_node publishes no mask image when a
        frame has no instances, so detection_3d_node's 3-way sync never
        fires). Wholesale loss of detection-bearing frames runs at ~0.8%, not
        35-45%.

        The grace window is still warranted, but by a different and smaller
        mechanism: per-CLASS presence flicker WITHIN non-empty frames (in run
        15-04-45, class presence toggled on/off 24 times for `person`, 20 for
        `bottle`, 31 for `tv` across 275 frames) plus association failures on
        noisy detections. Sized accordingly -- do not re-derive it from the
        35-45% figure.

        A grace window lets an intermittent-but-real object keep the progress
        it has made toward confirmation, without weakening what confirmation
        means for a genuine one-off false positive: a spurious detection still
        has to reappear repeatedly, it just no longer has to do so on perfectly
        consecutive frames. miss_streak itself is ALWAYS advanced, so the
        lost_miss_count pruning path is completely unaffected by this.
        """
        self.miss_streak += 1
        if self.miss_streak > confirm_grace_misses:
            self.hit_streak = 0


def _valid_width(width):
    """Return a usable footprint width, or None (missing, non-finite or non-positive)."""
    if width is None:
        return None
    width = float(width)
    return width if math.isfinite(width) and width > 0.0 else None


def _unpack_width(det):
    """Return the optional 6th element, footprint width [m], or None."""
    return det[5] if len(det) >= 6 else None


def _valid_z(z_base):
    """Return a usable centre height, or None (missing or non-finite)."""
    if z_base is None:
        return None
    z_base = float(z_base)
    return z_base if math.isfinite(z_base) else None


def _unpack_z(det):
    """Return the optional 7th element, the centre's base_link height [m], or None."""
    return det[6] if len(det) >= 7 else None


def _unpack_detection(det):
    """Accept both the original 4-tuple (class_id, x_map, y_map, score) and
    the 5-tuple form carrying a per-detection 1-sigma position uncertainty in
    metres, (class_id, x_map, y_map, score, sigma). A 6th element, when
    present, is the footprint width (see _unpack_width), and a 7th the centre's
    base_link height (see _unpack_z).

    Backward compatibility is deliberate rather than incidental: sigma only
    exists once detection_3d_node populates the pose covariance, and callers
    that have no uncertainty estimate (including this module's own existing
    tests) must keep working unchanged. sigma=None means "unknown", which the
    callers below treat as "apply no confidence weighting", NOT as "zero
    uncertainty" -- the latter would silently make unknown-quality detections
    the most trusted ones.
    """
    if len(det) >= 5:
        return det[0], det[1], det[2], det[3], det[4]
    class_id, x_map, y_map, score = det
    return class_id, x_map, y_map, score, None


def _effective_alpha(alpha: float, sigma, sigma_ref: float, min_alpha_scale: float):
    """EMA weight for one detection, reduced when that detection is noisy.

    A single global alpha gave a 25 cm-noisy low-confidence detection exactly
    the same influence over a track's position as a 3 cm-noisy high-confidence
    one -- the 2026-09-01 analysis measured that 2-7x confidence-dependent
    spread directly (see detection_3d_node's position_sigma_* params, which
    produce the sigma passed in here). Scaling alpha by sigma_ref / sigma makes
    a noisy detection move the track proportionally less, which is the standard
    inverse-variance intuition applied to the EMA this tracker already uses,
    rather than a second smoothing stage bolted on afterwards.

    Clamped below by min_alpha_scale so a very poor detection still contributes
    something: driving alpha to ~0 would freeze a track in place and make it
    unable to follow a genuinely moving object it can only see poorly.
    """
    if sigma is None or not (sigma > 0.0) or not (sigma_ref > 0.0):
        return alpha
    scale = min(1.0, sigma_ref / float(sigma))
    return alpha * max(min_alpha_scale, scale)


def update_tracks_batch(
        tracks: list, detections: list, stamp_sec: float,
        max_distance_m: float, alpha: float, confirm_hit_count: int,
        lost_miss_count: int, next_track_id,
        confirm_grace_misses: int = 0,
        alpha_sigma_ref_m: float = 0.0,
        min_alpha_scale: float = 0.25,
        gate_sigma_scale: float = 0.0,
        prune_confirmed: bool = True) -> list:
    """One frame's worth of association + lifecycle, replacing the old
    per-detection merge_or_add_object() (see module docstring). Mutates
    `tracks` in place where possible and returns the (possibly shorter --
    pruned) list; matches this module's existing mutate-and-return
    convention.

    tracks: list of SemanticObject (the accumulated state, across calls).
    detections: list of (class_id, x_map, y_map, score) or
        (class_id, x_map, y_map, score, sigma) tuples, ALL of this
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

    Uncertainty-aware parameters (all default to the pre-existing behaviour, so
    omitting every one of them reproduces this function exactly as it was):

    confirm_grace_misses: passed through to mark_missed() -- see there.
    alpha_sigma_ref_m: the sigma at which a detection gets the full `alpha`.
        0.0 (default) disables confidence weighting entirely.
    min_alpha_scale: floor on that scaling, so a poor detection still moves
        its track somewhat.
    gate_sigma_scale: widens the association gate for uncertain detections,
        max_distance_m + gate_sigma_scale * sigma. A fixed gate is the wrong
        shape against confidence-dependent noise: it is simultaneously too
        tight for a noisy detection (which then fails to match its own track
        and spawns a duplicate -- the churn mechanism) and looser than it
        needs to be for a precise one. 0.0 (default) keeps the fixed gate.
        NOTE the widened gate is used for matching only; the cost matrix still
        holds true distances, so the assignment still prefers the nearest
        legal pairing.
    prune_confirmed: True (default, the original behaviour) drops ANY track at
        lost_miss_count consecutive misses. False drops only TENTATIVE ones
        there: a confirmed track stays, for the object memory to expire
        (object_memory.memory_cycle). Each track's own extra_gate_m is added
        to its gate either way (0 unless the memory set it).
    """
    def _survives(trk):
        return (trk.is_unknown or trk.miss_streak < lost_miss_count
                or (trk.confirmed and not prune_confirmed))

    if not tracks:
        for det in detections:
            class_id, x_map, y_map, score, _sigma = _unpack_detection(det)
            tracks.append(SemanticObject(
                next_track_id(), class_id, x_map, y_map, score, stamp_sec, confirm_hit_count,
                width=_unpack_width(det), z_base=_unpack_z(det)))
        return tracks

    if not detections:
        # An UNKNOWN track's lifecycle is the LiDAR's (update_unknown_batch):
        # a camera frame that does not mention it is no evidence either way.
        for trk in tracks:
            # An UNKNOWN track's lifecycle is the LiDAR's (update_unknown_batch).
            if not trk.is_unknown:
                trk.mark_missed(confirm_grace_misses)
        return [t for t in tracks if _survives(t)]

    n_det, n_trk = len(detections), len(tracks)
    # cost[i][j]: real distance if (detection i, track j) is a legal pairing
    # (same class, within max_distance_m of track j's predicted position at
    # stamp_sec), else math.inf -- checked again explicitly after the solve
    # (see _INFEASIBLE_SENTINEL_MULTIPLIER's own comment for why a sentinel
    # substitute is needed for the solver call itself, separate from this).
    cost = [[math.inf] * n_trk for _ in range(n_det)]
    unpacked = [_unpack_detection(d) for d in detections]
    for i, (class_id, x_map, y_map, score, sigma) in enumerate(unpacked):
        # Gate widened by this detection's own uncertainty (see the
        # gate_sigma_scale docstring paragraph). The stored cost stays the TRUE
        # distance, so a widened gate only makes a pairing legal -- it never
        # makes a far pairing look closer than a near one to the solver.
        gate = max_distance_m
        if gate_sigma_scale > 0.0 and sigma is not None and sigma > 0.0:
            gate += gate_sigma_scale * float(sigma)
        for j, trk in enumerate(tracks):
            # Same class, or an UNKNOWN track (upgraded below if chosen).
            if trk.class_id != class_id and not trk.is_unknown:
                continue
            px, py = trk.predict(stamp_sec)
            dist = math.hypot(px - x_map, py - y_map)
            if dist <= gate + trk.extra_gate_m:
                cost[i][j] = dist

    # Sentinel must exceed every legal cost, and legal costs are now bounded by
    # the WIDEST gate any detection used, not by max_distance_m alone.
    max_gate = max_distance_m + max(trk.extra_gate_m for trk in tracks)
    if gate_sigma_scale > 0.0:
        sigmas = [sg for (_c, _x, _y, _s, sg) in unpacked if sg is not None and sg > 0.0]
        if sigmas:
            max_gate += gate_sigma_scale * max(sigmas)
    sentinel = max_gate * _INFEASIBLE_SENTINEL_MULTIPLIER + 1e6
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
        class_id, x_map, y_map, score, sigma = unpacked[i]
        if tracks[j].is_unknown:
            # THE UPGRADE: the same track, recognised from now on. Its lifecycle
            # becomes the detections' own (hit/miss streaks), restarted here;
            # an unknown track that was already published stays published.
            tracks[j].class_id = class_id
            tracks[j].upgraded_sec = float(stamp_sec)
            tracks[j].miss_streak = 0
        tracks[j].update(x_map, y_map, score, stamp_sec,
                         _effective_alpha(alpha, sigma, alpha_sigma_ref_m, min_alpha_scale),
                         width=_unpack_width(detections[i]), z_base=_unpack_z(detections[i]))
        matched_det_idx.add(i)
        matched_trk_idx.add(j)

    for j, trk in enumerate(tracks):
        if j not in matched_trk_idx and not trk.is_unknown:
            trk.mark_missed(confirm_grace_misses)

    for i, (class_id, x_map, y_map, score, _sigma) in enumerate(unpacked):
        if i not in matched_det_idx:
            tracks.append(SemanticObject(
                next_track_id(), class_id, x_map, y_map, score, stamp_sec, confirm_hit_count,
                width=_unpack_width(detections[i]), z_base=_unpack_z(detections[i])))

    return [t for t in tracks if _survives(t)]


def hand_over_to_lidar(trk, now_sec: float, window_s: float) -> bool:
    """
    Turn a recognised track the object memory just let go into a CONFIRMED unknown one.

    Only when the LiDAR still hit it (absorbed its cluster) within window_s:
    the object is physically there, so the obstacle goes on without a gap --
    same id, same place, confirmed at once -- instead of being dropped and
    coming back 0.5 s later as a new tentative unknown. From here on its
    lifecycle is the LiDAR's (update_unknown_batch). Returns whether it did.
    """
    if trk.last_lidar_hit_sec is None or now_sec - trk.last_lidar_hit_sec > window_s:
        return False
    r = trk.radius()
    trk.class_id = UNKNOWN_CLASS
    trk.confirmed = True
    trk.extent = (2.0 * r, 2.0 * r)
    trk.width = 2.0 * r
    trk.angle = 0.0
    trk.vx_map = trk.vy_map = 0.0
    trk.extra_gate_m = 0.0
    trk.seen_through = False
    trk.upgraded_sec = None
    trk.handed_over_sec = float(now_sec)
    return True


def live_tracks(tracks: list, lost_miss_count: int) -> list:
    """
    Return the tracks the original lifecycle would still hold.

    That is every track below lost_miss_count consecutive misses. With
    prune_confirmed=True this is all of them; with the object memory on it
    leaves out the confirmed tracks that only the memory is keeping, so
    /costmap/semantic_markers and /costmap/semantic_tracks are unchanged by it.
    """
    return [t for t in tracks if t.miss_streak < lost_miss_count]


def update_unknown_batch(tracks: list, candidates: list, stamp_sec: float, p,
                         max_distance_m: float, alpha: float, next_track_id,
                         observe=None, can_absorb=None):
    """
    One LiDAR cycle of unknown-object evidence, into the SAME track list.

    Returns (tracks, stats). `candidates` are this cycle's
    unknown_objects.Candidate list (map frame), `p` its UnknownParams.

    Association: whole-cycle Hungarian against EVERY track, of any class, at
    the track's predicted position, gated at max(max_distance_m, track
    radius + candidate radius).
      * matched UNKNOWN track: EMA position and extent (the same alpha as a
        detection), last LiDAR hit; CONFIRMED once its run of consecutive
        sightings spans persist_s.
      * matched RECOGNISED track: ABSORBED -- the cluster is that object's own
        LiDAR footprint, never a second, parallel unknown. Nothing else
        changes: a recognised track keeps the detections' lifecycle.
      * unmatched candidate: a new TENTATIVE unknown track -- unless it lies
        within the gate of a track already held, when it is a second cluster
        of that object ('split') and only refreshes its last hit.
      * unmatched unknown track: tentative -> dropped (the sightings must be
        consecutive); confirmed -> kept until decay_s after its last hit once
        the LiDAR has seen THROUGH its area (observe(track) -> (observed,
        passed[, hit]); hit: an occupied cell still lies inside its disk,
        which keeps it alive), or unobserved_decay_s if it never could.

    can_absorb(track) -> bool, for RECOGNISED tracks: False keeps that track
    out of the association and of the 'split' test, so its clusters become
    unknown objects. The node passes the obstacle plausibility gate: an
    object the MPC will never avoid (say, a bottle on a table) must not hide
    what the LiDAR sees under it (the table's legs).
    """
    stats = {'matched': 0, 'absorbed': 0, 'split': 0, 'spawned': 0, 'confirmed': 0,
             'deleted': 0}
    stamp_sec = float(stamp_sec)
    matched_trk, matched_cand = set(), set()
    def absorbs(trk):
        return trk.is_unknown or can_absorb is None or can_absorb(trk)
    if tracks and candidates:
        pred = np.array([trk.predict(stamp_sec) for trk in tracks])
        t_rad = np.array([trk.radius() for trk in tracks])
        cxy = np.array([(cand.x, cand.y) for cand in candidates])
        c_rad = np.array([cand.radius for cand in candidates])
        dist = np.hypot(cxy[:, None, 0] - pred[None, :, 0], cxy[:, None, 1] - pred[None, :, 1])
        gate = np.maximum(max_distance_m, c_rad[:, None] + t_rad[None, :])
        eligible = np.array([absorbs(trk) for trk in tracks])
        cost = np.where((dist <= gate) & eligible[None, :], dist, np.inf)
        finite = np.isfinite(cost)
        if finite.any():
            sentinel = cost[finite].max() * _INFEASIBLE_SENTINEL_MULTIPLIER + 1e6
            rows, cols = linear_sum_assignment(np.where(finite, cost, sentinel))
            for i, j in zip(rows, cols):
                if not finite[i, j]:
                    continue
                matched_cand.add(i)
                matched_trk.add(j)
                trk, cand = tracks[j], candidates[i]
                trk.last_lidar_hit_sec = stamp_sec
                trk.seen_through = False
                if not trk.is_unknown:
                    stats['absorbed'] += 1
                    continue
                stats['matched'] += 1
                trk.x_map = alpha * cand.x + (1.0 - alpha) * trk.x_map
                trk.y_map = alpha * cand.y + (1.0 - alpha) * trk.y_map
                ext = (cand.length, cand.width)
                trk.extent = ext if trk.extent is None else tuple(
                    alpha * a + (1.0 - alpha) * b for a, b in zip(ext, trk.extent))
                trk.width = max(trk.extent)
                trk.angle = cand.angle
                trk.last_update_stamp_sec = stamp_sec
                trk.hit_count += 1
                if not trk.confirmed and stamp_sec - trk.streak_start_sec >= p.persist_s:
                    trk.confirmed = True
                    stats['confirmed'] += 1
    kept = []
    for j, trk in enumerate(tracks):
        if j in matched_trk or not trk.is_unknown:
            kept.append(trk)
            continue
        if not trk.confirmed:
            stats['deleted'] += 1          # a broken run of sightings
            continue
        if observe is not None:
            seen = observe(trk)
            if len(seen) > 2 and seen[2]:
                # Its cells are still hit -- the cluster merely went to another
                # track or was bridged differently this cycle: still there.
                trk.last_lidar_hit_sec = stamp_sec
                trk.seen_through = False
            else:
                trk.seen_through = trk.seen_through or seen[1]
        age = stamp_sec - (trk.last_lidar_hit_sec if trk.last_lidar_hit_sec is not None
                           else trk.streak_start_sec)
        if (trk.seen_through and age > p.decay_s) or age > p.unobserved_decay_s:
            stats['deleted'] += 1
            continue
        kept.append(trk)
    for i, cand in enumerate(candidates):
        if i in matched_cand:
            continue
        # A second cluster of an object a track already has (one physical
        # object split into two components this cycle): evidence for that
        # track, never a new one beside it.
        near = [t for t in kept if absorbs(t) and math.hypot(t.x_map - cand.x, t.y_map - cand.y)
                <= max(max_distance_m, t.radius() + cand.radius)]
        if near:
            for t in near:
                if t.is_unknown:
                    t.last_lidar_hit_sec = stamp_sec
            stats['split'] += 1
            continue
        trk = SemanticObject(next_track_id(), UNKNOWN_CLASS, cand.x, cand.y, 0.0,
                             stamp_sec, confirm_hit_count=1, width=2.0 * cand.radius)
        trk.confirmed = p.persist_s <= 0.0
        trk.extent = (cand.length, cand.width)
        trk.angle = cand.angle
        trk.last_lidar_hit_sec = stamp_sec
        kept.append(trk)
        stats['spawned'] += 1
    return kept, stats


def dedupe_tracks(tracks: list, max_distance_m: float, on_fold=None, can_absorb=None):
    """
    Fold every pair that is ONE physical object into the older track.

    Only pairs involving an UNKNOWN track (two recognised tracks keep today's
    class-gated rule): centres within max(max_distance_m, the two radii).
    The older track_id survives; a recognised class, position and width win
    over an unknown's. Returns (tracks, number of folds) -- the folds are
    the duplicates association let through, and should be 0.

    hit_count stays the count of the survivor's own sensor: detections for a
    recognised survivor (LiDAR hits never add to it -- the object memory reads
    a change of hit_count as "the camera saw it"), LiDAR hits for two unknowns.
    on_fold(survivor, folded) is called for every fold where the survivor took
    a recognised track's identity, so the caller can move that track's memory
    to the surviving id. can_absorb as in update_unknown_batch: a recognised
    track it rejects is never folded with an unknown.
    """
    merged = 0
    tracks = sorted(tracks, key=lambda t: t.track_id)
    i = 0
    while i < len(tracks):
        a = tracks[i]
        j = i + 1
        while j < len(tracks):
            b = tracks[j]
            rec = b if a.is_unknown else a
            if (a.is_unknown or b.is_unknown) and (
                    rec.is_unknown or can_absorb is None or can_absorb(rec)) and math.hypot(
                    a.x_map - b.x_map, a.y_map - b.y_map) <= max(
                    max_distance_m, a.radius() + b.radius()):
                took_identity = a.is_unknown and not b.is_unknown
                if took_identity:
                    a.class_id, a.x_map, a.y_map = b.class_id, b.x_map, b.y_map
                    a.width, a.score = b.width, b.score
                    a.hit_streak, a.miss_streak = b.hit_streak, b.miss_streak
                    a.upgraded_sec = b.last_update_stamp_sec
                    a.extent = None
                    a.hit_count = b.hit_count
                    a.z_base, a.extra_gate_m = b.z_base, b.extra_gate_m
                    a.vx_map, a.vy_map = b.vx_map, b.vy_map
                    a.last_update_stamp_sec = b.last_update_stamp_sec
                elif a.is_unknown and b.is_unknown:
                    a.hit_count += b.hit_count
                a.confirmed = a.confirmed or b.confirmed
                hits = [t.last_lidar_hit_sec for t in (a, b) if t.last_lidar_hit_sec is not None]
                a.last_lidar_hit_sec = max(hits) if hits else None
                if took_identity and on_fold is not None:
                    on_fold(a, b)
                del tracks[j]
                merged += 1
                continue
            j += 1
        i += 1
    return tracks, merged


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
