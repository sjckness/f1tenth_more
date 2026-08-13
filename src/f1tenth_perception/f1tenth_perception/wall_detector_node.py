#!/usr/bin/env python3
"""
wall_detector_node.py

Runs in parallel to YOLO. Consumes the stereo camera's point cloud (ZED),
and uses iterative RANSAC plane segmentation to find near-vertical planar
surfaces ("walls") ahead of the robot. Publishes:

  - /perception/wall_detections   (f1tenth_messages/WallArray)  structured data
  - /perception/front_clearance   (std_msgs/Float32)             for move_straight stop_condition
  - /perception/wall_markers      (visualization_msgs/MarkerArray) for Foxglove

Handles corners: after removing the dominant wall plane's inliers, it looks
for a second plane in the remaining cloud. If found and roughly perpendicular
to the first, both are reported as separate WallDetection entries.

NOTE on point cloud library: true python-pcl bindings are effectively
unmaintained on ROS 2 / Python 3. This uses Open3D's RANSAC plane
segmentation (open3d.geometry.PointCloud.segment_plane), which is the
same algorithm as PCL's SACSegmentation (RANSAC + plane model) and is
actively maintained. If you specifically want the C++ PCL library
(pcl::SACSegmentation) instead, this logic maps 1:1 onto it -- port
if/when that decision is made.

Frame handling (verified, not assumed -- see the integration notes this
node was wired in with): the ZED wrapper publishes /zed2/zed_node/point_cloud/
cloud_registered in frame `zed2_left_camera_frame` -- NOT the `_optical_`
frame RGB/depth images use. Confirmed by reading the vendored zed_ros2_wrapper
source directly: `mPointCloudFrameId = mDepthFrameId = mLeftCamFrameId`
(zed_camera_component.cpp), and the SDK itself is initialized with
`coordinate_system = sl::COORDINATE_SYSTEM::RIGHT_HANDED_Z_UP_X_FWD`
(sl_types.hpp's ROS_COORDINATE_SYSTEM constant) -- so the cloud's XYZ data is
already Z-up/X-forward, matching this node's own default
`input_frame_convention='z_up_x_forward'` (`_maybe_reorient()`'s no-op path
is correct here; the `'zed_optical'` axis-swap path would NOT be). What that
source read does NOT give you for free is the ORIGIN: `zed2_left_camera_frame`
is offset from base_link by a real, non-trivial translation (the camera's
physical mount position, ~0.12m forward / 0.15m up on this robot) plus
whatever small additional offset the ZED's own URDF gives the left lens
relative to its mounting link -- axis convention alone doesn't correct for
that. So the primary frame-handling path here is a real tf2 lookup + a
vectorized rotation/translation applied straight to the extracted Nx3 array
(see `_transform_to_robot_frame()`), transforming into `robot_frame`
(base_link) properly -- mirroring the tf2 pattern
f1tenth_perception/obstacle_projector_node.py already uses for its own
camera-frame -> base_link step, rather than trusting a hardcoded axis-only
swap with an implicit zero-origin assumption that doesn't hold on this rig.
`_maybe_reorient()`/`input_frame_convention` are kept below (unused in the
default flow) for anyone bench-testing against a cloud from a different,
non-TF'd source where 'zed_optical' really would apply.

Plane merge + tracking (two independent fixes, run in this order, never
conflated -- see each stage's own module-level docstring for the full
reasoning):

  1. merge_duplicate_walls() -- runs on each frame's RAW candidate plane
     list from _find_walls's segmentation loop, BEFORE is_corner tagging.
     Fixes a single real flat wall occasionally being reported as two (or
     more) overlapping plane detections, which iterative re-segmentation
     can produce when one noisy surface splits at the inlier-distance-
     threshold boundary. Pure geometry (normal similarity + inter-plane
     offset gap), no frame history involved at all.
  2. WallTracker -- runs AFTER merge, frame to frame, entirely separate
     concern: EMA-smooths each already-deduplicated wall's distance/
     bearing/normal/centroid to fix jitter, associating this frame's walls
     to existing tracks by (distance, bearing) proximity rather than trusting
     array order. A duplicate that only sometimes splits apart would break a
     smoothing-only fix (two "walls" jittering into and out of existence);
     smoothing two independently-jittering true duplicates would just
     produce two stable-but-wrong walls -- hence doing dedup strictly first,
     on each frame's raw geometry, before any temporal state ever sees it.

Every consumer of this node's output reflects the tracked/smoothed values,
not the raw per-frame ones -- WallDetection (distance/bearing/normal),
/perception/front_clearance (derived from the same), and the Foxglove
markers (position/orientation now driven by TrackedWall's smoothed centroid/
normal; the drawn quad's SIZE/extent is deliberately NOT smoothed, see
TrackedWall's own docstring for why).

Merge threshold live-tuning history: initial thresholds (0.97 / 0.08m) were a
reasoned starting point, not tuned against real sensor noise -- flagged as such
when added. A dedicated live debug pass (566-frame capture against a real
flat wall, TEMP DEBUG instrumentation on every merge-stage decision) found
them too tight: real single-wall RANSAC splits clustered at
normal_similarity 0.951-0.966 and offset_gap 0.08-0.11m, both routinely
failing the original thresholds despite clearly being the same physical
wall (worst clean case: cos=0.99959 with offset_gap=0.08296m -- 3mm over the
old 0.08m cutoff). Genuine corner pairs measured 84.4-89.4 deg apart in the
same capture, nowhere near either failure cluster. Loosened to 0.94 / 0.15m
on that evidence -- both stack_params.yaml keys' own comments carry the full
numbers; see them for the corner-safety margin math (acos(0.94) ~= 20 deg of
headroom below the real corner range).

Ambiguous zone (second pass, on top of the loosened thresholds above): even
a well-tuned merge threshold is a single hard cutoff -- a pair sitting just
past it instantly becomes a second permanent track, which is what actually
produced the "dozens of near-duplicate wall labels" symptom the loosened
thresholds alone don't fully solve (they narrow the failure band, not
eliminate the hard-cutoff problem itself). Three zones instead of two, per
pairwise wall comparison (a candidate wall vs. an existing TrackedWall's own
current normal/centroid/plane, reusing the exact normal_similarity/
offset_gap math merge_duplicate_walls already uses -- see
_classify_wall_relationship):
  - SAME: normal_similarity >= wall_merge_normal_cos_thresh AND
    offset_gap < wall_merge_distance_thresh_m -- merges into that track
    immediately (same as always).
  - DISTINCT: normal_similarity < wall_merge_normal_cos_thresh (this
    already covers genuine corners, whose normal_similarity is near 0 --
    see the module's own unit tests) OR offset_gap >=
    wall_min_distinct_separation_m -- becomes/stays its own track
    immediately, same as today's behavior for anything that fails the
    merge check outright.
  - AMBIGUOUS: parallel-ish (not a corner) with offset_gap sitting between
    the two thresholds above -- neither confidently the same wall nor
    confidently a different one. Does NOT spawn a permanent track on first
    sight. Held in a separate pending queue (PendingWallGate, owned by
    WallTracker) and only promoted to a real TrackedWall once the SAME
    recurring candidate (matched frame-to-frame by WallTracker's own
    distance/bearing association, not the plane-geometry check) has shown
    up for wall_ambiguous_confirm_frames CONSECUTIVE frames -- a gap in
    that streak drops the pending candidate outright, no partial credit.
    Never published to Foxglove/WallArray while pending, since it's not
    part of WallTracker's own _tracks list until promoted. If, on any later
    frame, the same recurring pending candidate turns out to satisfy SAME
    against an existing track instead, it merges into that track directly
    and its pending state is simply never re-extended (see
    WallTracker.update()).
This only changes the SPAWN path for walls that don't already position-match
an existing track (WallTracker's existing distance/bearing association is
untouched) -- and only checks candidate-vs-existing-track, not candidate-vs-
candidate within one frame's own raw list (that's still merge_duplicate_
walls' job, upstream, unaffected by any of this).

Raw candidate quality gating (second-plane-only) -- added after a live-
mission-log investigation (real driving run, not a stationary bench test)
found the merge/ambiguous-zone layers above were fighting a losing battle:
normal_similarity for genuinely-close, should-be-the-same-wall pairs came in
at median 0.812 (range 0.544-0.994), far below even the already-loosened
0.94 threshold -- no further threshold loosening could plausibly absorb
that without also risking real corners. The concrete example that pinned
this down: frame 9 of that capture had TWO RANSAC planes fit from the SAME
single point cloud, 44 deg apart in normal (normal_similarity=0.718), both
independently clearing the old min_inliers=150-only floor -- the iterative
second segment_plane() call had fit a different, noisier patch of the SAME
physical wall face, not a real second surface. EKF/odometry drift was ruled
out entirely (wall_detector's own geometry is base_link-relative via a
static TF lookup -- see "Frame handling" above -- structurally independent
of localization_source).

The fix targets the root instead of the symptom: the FIRST/dominant plane
in a frame is, by construction, the strongest RANSAC fit available and
doesn't show this failure mode -- only the SECOND (and any further) plane,
fit from whatever residual is left after removing the first plane's
inliers, does. Two independent stages, both gated to idx > 0 in
_find_walls's segmentation loop (never applied to the dominant plane):

  1. Statistical outlier removal (Open3D remove_statistical_outlier) on the
     whole ROI-filtered, voxel-downsampled cloud, before ANY segment_plane
     call -- reduces the sparse/noisy points a second iteration could latch
     onto in the first place.
  2. Per-candidate quality gates on the second-plane search specifically
     (_residual_supports_second_plane / _second_plane_inlier_ratio_ok /
     _second_plane_compact_ok, pure functions, independently unit-tested):
     a residual-size floor (don't even attempt a second fit if what's left
     isn't plausibly big enough to hide a real wall under the first one's
     worth of support), an inlier-support-ratio floor (a fit that only
     explains a small fraction of the residual it was searched in is the
     empirical signature of the frame-9 failure mode), and an in-plane
     bounding-extent aspect-ratio ceiling (a coherent wall face has
     comparable spread along both in-plane axes; a fit through a sparse/
     linear residual cluster tends to be long and thin in one axis).

All five new parameters (outlier_nb_neighbors/outlier_std_ratio,
second_plane_min_residual_ratio/second_plane_min_inlier_ratio/
second_plane_max_aspect_ratio) are REASONED STARTING POINTS, not tuned
against real sensor noise -- flagged as such deliberately, same discipline
merge_normal_cos_thresh/merge_distance_thresh_m's own history followed
before their live-tuning pass. Live validation is blocked as of this
writing on a hardware issue (ZED camera intermittently failing "CAMERA NOT
DETECTED" despite lsusb showing it enumerated -- a cable/USB-level problem,
not a code issue) -- kept node-local (not exposed as launch args), same
precedent as their nearest neighbors ransac_distance_threshold/min_inliers/
ransac_n/ransac_iterations, which are also node-local internals of this
same detection stage. GATE_DEBUG logging (opt-in-by-being-unconditional,
same pattern as MERGE_DEBUG) reports pass/fail counts so a future live
capture can validate/retune these from real numbers rather than guesses.

front_clearance selection eligibility -- a SEPARATE, independent fix for a
bug the same investigation found in _publish()'s front_clearance
computation: it took min(distance) over every TRACKED wall within the
front-facing bearing cone, with no notion of how well-supported or stable
that track actually was. In the failed mission this investigated, a
short-lived (spawned ~4.6s earlier), internally noisy track sitting at
bearing -17 deg (well inside the 35 deg cone) dipped to ~0.96m purely from
its own per-frame RANSAC noise while the REAL front wall the mission was
approaching was still at 1.6-1.8m -- front_clearance's naive min() picked
the phantom, falsely satisfying the mission's stop_condition roughly 2.3
seconds before the car had actually gotten close to the wall it was aiming
at, before MPC's own independent obstacle-avoidance deflection (a separate
mechanism entirely, fed by /perception/obstacles_2d, not front_clearance)
ever needed to engage.

Fix: a track must now clear a stability bar (_front_clearance_eligible,
pure function) to be eligible for front_clearance selection at all, on top
of the existing bearing-cone check -- a minimum matched-frame age
(front_clearance_min_track_frames, reusing the same kind of frame-count
gate ambiguous_confirm_frames already established as this codebase's
precedent for "distinguish real persistence from transient noise") AND a
minimum RANSAC inlier-support floor (front_clearance_min_inliers, tied
directly to this fix's own root-cause finding: a marginal/fragment second-
plane fit has much weaker point support than a genuine dominant wall).
Both are REPORTED, NOT GUARANTEED, to reject the specific investigated
case: replaying the real log data through this logic, the phantom track
had already accumulated ~38 matched frames (about 4.4s) by the moment the
mission's stop_condition falsely fired -- an age gate alone tight enough to
reject that specific instance would need to sit north of ~40 frames
(~4.5-5s), which would cost EVERY freshly-(re)spawned real wall the same
latency before front_clearance trusts it at all, an unacceptable
responsiveness cost for a safety-relevant stop condition. So
front_clearance_min_track_frames is kept modest (reusing
ambiguous_confirm_frames's own default, 5 frames -- insurance against a
single-frame outlier, not the primary defense), and the inlier-support
floor -- directly targeting the SAME low-support signature the raw-
detection-stage gates above target at the source -- is the fix's real
teeth. This is intentionally DEFENSIVE/independent of the raw-detection-
stage gates above: those should reduce how often a phantom track like this
ever forms; this is a second, independent check at the publish layer
regardless of whether it does.

Motion compensation -- a separate gap in WallTracker's own per-frame
update: association (track_assoc_distance_thresh_m/_bearing_thresh_deg)
and EMA blending both compared a fresh detection directly against a
track's last-known base_link-relative position, with no accounting for
real ego motion between frames. At real driving speed this delta is not
negligible -- approaching a wall legitimately shrinks its relative
distance every single frame, which can exceed track_assoc_distance_thresh_m
(0.4m) on its own at highway-adjacent closing speeds, or smaller but still
non-zero deltas that quietly feed EMA a target it perpetually lags behind
even when association still succeeds.

Fix: before running association each frame, every live track's stored
position is advanced ("predicted") by the robot's own short-term motion
since the last update tick, via _predict_track_position (pure function) --
association and the subsequent EMA blend then compare against this
predicted position, not the stale pre-motion one. The ego-motion delta
itself comes from consecutive ODOMETRY poses (get_odom_topic() -- follows
localization_source, same precedent MPC_corr.py already established) via
_odom_delta -- deliberately the SHORT frame-to-frame delta between the
immediately-preceding tick and now, never an accumulated/absolute pose:
the earlier front_clearance investigation already established that a
localization source's long-horizon absolute pose can drift while its
short-term relative delta stays accurate (this is exactly the same
distinction, applied to a different consumer). Odometry unavailable (no
message received yet) or stale (older than _ODOM_STALE_SEC) disables
compensation for that frame only -- WallTracker.update() already treats
ego_delta=None as a no-op predict step, identical to this feature not
existing at all, so a lost odometry source degrades gracefully to the
pre-existing behavior rather than crashing or corrupting track state.

Confidence-based pruning -- the other gap: a track was only ever dropped
via track_hold_frames (stops matching entirely for that many consecutive
frames). Nothing removed a track that keeps matching (so never ages out)
but is a weaker, less-supported duplicate of a competing track occupying
roughly the same physical space -- exactly the shape of the
front_clearance investigation's track_id=14 (persistent, low-support,
never cleaned up by the hold-frame path since it kept re-matching itself
every frame).

Fix: prune_inconsistent_tracks (pure function), called once per update()
tick after association/EMA/spawn/promotion, right before publish. For
every pair of live tracks, reuses _classify_wall_relationship (the SAME
normal_similarity/offset_gap test merge_duplicate_walls and the ambiguous
zone already use, with wall_prune_conflict_radius_m standing in for the
distinct-separation threshold) to find pairs that are geometrically
consistent enough to be plausibly the same physical surface (zone != a
genuine corner or genuinely-far-apart 'distinct') -- these are the
"conflicting" pairs. For each conflicting pair, support is compared via
the n_inliers/frames_matched fields front_clearance selection eligibility
already added to TrackedWall: a track only loses (dropped outright, same
as exceeding track_hold_frames -- next _publish() call's existing
Marker.DELETE diffing picks it up automatically, no separate delete-path
needed) if it's MEANINGFULLY weaker on at least one signal
(wall_prune_inlier_ratio_floor / wall_prune_match_streak_ratio_floor,
both default 0.5 -- the weaker track's value is under half the stronger
track's) -- comparable support on both signals leaves both tracks alone,
since two independently-jittering real measurements of one physical wall
will rarely have identical support and pruning is meant to catch clear
duplicates, not adjudicate every close call. wall_prune_min_frames_before_
eligible (default 3) exempts brand-new tracks from being the LOSING side
of a comparison (they can still out-compete a genuinely weaker rival) so
an early low-inlier frame alone can't get a fresh, legitimate track pruned
before it's had a chance to establish itself. In the rare case both sides
of a pair look weaker than the other by different signals simultaneously,
neither is dropped (ambiguous, not a confident prune).

Hard boundary constraints (/perception/front_wall_boundary) -- complements
front_clearance with a HARD linear constraint for mpc_controller's OSQP/RTI
solver (f1tenth_messages/BoundaryConstraintArray, 0 or 1 entries: the
front wall, when one is currently eligible; empty otherwise). Deliberately
reuses _front_clearance_eligible() itself (same call, not reimplemented
logic) to pick which track counts -- a track that isn't trustworthy enough
to report a scalar distance isn't trustworthy enough to constrain the
solver's trajectory either; among eligible front-facing tracks, the same
minimum-distance one front_clearance itself would report is the one
converted.

SIGN CONVENTION (this is the part actually worth getting right, verified
against WallDetection.msg's own long-documented convention, not assumed):
WallDetection.normal is "oriented back toward the robot" (see that
message's own field comment, and _find_walls'/_wall_from_points'
"orient normal to point back toward the robot" reorientation step, which
TrackedWall.normal inherits unmodified through EMA blending). This means,
for the robot's own origin p=(0,0): dot(track.normal, p) + track.distance
>= 0 always (the reorientation invariant, restated) -- i.e. the robot's
own position already satisfies `(-track.normal) . p <= track.distance`.
BoundaryConstraint.msg's own convention is `normal . (x, y) <= offset`
meaning the free-space side is where the dot product is SMALL -- so
BoundaryConstraint.normal = -track.normal (flip WallDetection's own
"toward the robot" convention to "toward the wall") and
BoundaryConstraint.offset = track.distance, BEFORE any margin: a
consumer keeps at least car_radius + obstacle_safety_margin_m clearance
by additionally requiring `BoundaryConstraint.normal . p <= offset -
car_radius - margin` at the point of use (mpc_controller's own job, not
this node's -- see BoundaryConstraint.msg's own note on why offset isn't
pre-shrunk here).

2D projection: track.normal's z-component is small but not exactly zero
(verticality_max_deg allows up to 20 deg off pure-horizontal), so a naive
[nx, ny] slice isn't unit length in 2D. _wall_boundary_from_track
renormalizes AND rescales the offset by the same factor (1/k, where
k = |[nx, ny]|) so the halfspace test stays an exact perpendicular-
distance test for the robot's ground-plane (X, Y) position (evaluated at
the robot's own Z=0, where the 3D plane equation nx*X+ny*Y+nz*Z+d=0
reduces exactly to nx*X+ny*Y+d=0 regardless of nz) -- not just
approximately so. Both corrections are small in practice for a
well-fit, near-vertical wall, but cost nothing to do properly.

Track-identity stickiness -- added by the "Boundary detection hardening"
pass, after a live bag capture (boundary_constraint_diag_20260813_124824)
found front_wall_boundary exhibiting track-identity churn: an empty gap
followed by a discontinuous jump to a different apparent distance. Never
caused a false trigger in that run, but is a real gap worth closing.

FIRST CHECKED (before writing anything new, per this pass's own
instructions): whether the ALREADY-IMPLEMENTED WallTracker mechanisms
(motion compensation + track_hold_frames + confidence-based pruning, all
described above -- and, contrary to this pass's own initial assumption,
already fully wired into live update()/_consume_ego_delta(), not merely
designed) would plausibly explain and fix the observed pattern. They do
NOT, and the same bag capture supplies the evidence why: front_wall_
boundary published at ~8.8Hz in this run, and every one of the 21
present -> absent -> present cycles measured directly from the bag lasted
0.3-7.8s (median well over 1s; the longest held 68 CONSECUTIVE empty
messages) -- one to two orders of magnitude longer than track_hold_frames'
own ~5-frame (~0.5-0.6s at this rate) hold window. Motion compensation
improves ASSOCIATION ACCURACY within that window (keeping a track's
predicted position aligned with real ego motion so a re-detection re-
matches the SAME track rather than spawning a new one); it does not, and
structurally cannot, extend how long a track survives with zero
detections. hold_frames itself is a hard, deliberately-short frame-count
cutoff -- widening it to bridge multi-second gaps would mean publishing a
HARD boundary constraint built from track state that is, by the time it's
used, multiple seconds stale (the robot may have moved substantially in
7.8s) -- a materially worse hazard than briefly publishing no constraint
while genuinely lacking current information. So this pattern is NOT a
tracking-continuity bug: it reflects genuine, sustained loss of ZED wall
detection/eligibility (most plausibly FOV/ROI geometry during turns, or a
sustained raw-detection-stage quality-gate failure) -- a separate,
upstream concern, out of this pass's scope, and not something any
track-survival-duration knob should be tuned to paper over.

Given that, the fix actually implemented here (_select_boundary_track) is
the ONE part of this pass's originally-proposed fallback design that
targets a real, independently-confirmed, currently-unguarded gap: even
setting the large-gap pattern above aside, _publish()'s old
`min(eligible_front_tracks, key=distance)` re-selected fresh every single
tick, with zero memory of what was published last tick -- so two
tracks that are BOTH simultaneously eligible (e.g. a corner's two walls,
or a momentarily-duplicated near-track before confidence-pruning catches
up) sitting close enough in distance for ordinary per-frame RANSAC/EMA
noise to occasionally swap their relative order would flip the published
constraint's geometry back and forth on pure noise, even though neither
track's own eligibility ever lapsed. _select_boundary_track fixes exactly
this: prefer continuing to publish the SAME track_id as last tick if it's
still in this tick's eligible set, falling back to min(distance) only when
there's no previous selection to preserve (first publish, or the
previous winner genuinely dropped out of eligibility/existence). This is
a real, worthwhile, low-risk fix on its own merits -- it is NOT, based on
the evidence above, expected to eliminate the specific large-gap
empty-then-jump pattern the bag's headline finding describes, since that
pattern's root cause sits upstream of any selection-layer logic. Flagged
explicitly rather than overclaiming a fix.
"""

import math

import numpy as np
import open3d as o3d
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from scipy.spatial.transform import Rotation
from tf2_ros import ConnectivityException, ExtrapolationException, LookupException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Float32, Header
from geometry_msgs.msg import Point, Vector3
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker, MarkerArray

# get_odom_topic(): the active odometry topic given localization_source (see
# f1tenth_params' own module docstring) -- single source of truth so this
# node's own motion-compensation odometry source stays in lockstep with
# whichever map -> odom source is actually live, same precedent MPC_corr.py
# and CheckStopCondition already established for their own odometry reads.
from f1tenth_params.param_defaults import get_odom_topic

# Custom message -- built from f1tenth_messages per Step 1 of this prompt.
from f1tenth_messages.msg import (
    BoundaryConstraint, BoundaryConstraintArray, WallArray, WallDetection)


# ==============================================================================
# Duplicate/near-coplanar plane merge -- pure functions, no rclpy dependency,
# independently unit-testable (same "pure logic separate from ROS glue"
# convention f1tenth_behavior's mission package already uses). Runs on each
# frame's RAW candidate plane list from _find_walls's segmentation loop,
# BEFORE is_corner tagging and before WallTracker -- see module docstring's
# "Plane merge + tracking" section for why these stay two independent passes.
#
# Each wall dict this operates on/returns has (at minimum): distance, bearing,
# normal (unit np.ndarray), centroid (np.ndarray), extent (np.ndarray),
# points (Nx3 np.ndarray), plane (a, b, c, d) with (a,b,c) == normal (already
# unit-length, see _find_walls) and d the SIGNED, matching-scale offset --
# after _find_walls's own "orient normal toward the robot origin" step, d is
# always exactly +distance (derived once, in comments, not re-derived here):
# the plane equation is normal.x + d = 0; evaluated at a point roughly ON the
# plane (the centroid), d ~= -normal.centroid; the reorientation step
# guarantees normal.centroid <= 0, so d = -normal.centroid >= 0 always, i.e.
# d == distance exactly (both already non-negative, same unit-normal scale).
# ==============================================================================

def _plane_point_distance(plane, point) -> float:
    """Perpendicular distance from `point` (3,) to `plane` (a, b, c, d),
    assuming (a, b, c) is already unit-length (true for every plane tuple
    this module constructs -- see the module-level note above)."""
    a, b, c, d = plane
    return abs(a * point[0] + b * point[1] + c * point[2] + d)


def _offset_gap(wall_a: dict, wall_b: dict) -> float:
    """Symmetric perpendicular-distance gap between two candidate planes:
    the average of each plane's distance to the OTHER's centroid. Only a
    meaningful "how far apart are these" measure for near-parallel planes
    (which is exactly the only case this is ever called for -- see
    _are_duplicate_planes' short-circuit on normal_similarity below; two
    genuinely different, e.g. perpendicular, planes have a distance that
    varies wildly depending on where along each you evaluate it, so this
    number would be meaningless for them). Averaged rather than the plain
    min: with normal_similarity already gating this to near-parallel pairs,
    both directions are nearly equally valid/stable estimates of the same
    gap, and averaging is symmetric (doesn't privilege whichever plane's
    centroid estimate happens to be noisier that frame)."""
    d_b_to_plane_a = _plane_point_distance(wall_a['plane'], wall_b['centroid'])
    d_a_to_plane_b = _plane_point_distance(wall_b['plane'], wall_a['centroid'])
    return 0.5 * (d_b_to_plane_a + d_a_to_plane_b)


def _are_duplicate_planes(
        wall_a: dict, wall_b: dict, normal_cos_thresh: float, distance_thresh_m: float) -> bool:
    """True if wall_a/wall_b look like the same physical surface, noisily
    split into two RANSAC segments. Both conditions required: near-parallel
    normals (dot product of two unit vectors, so 1.0 = identical direction)
    AND a small perpendicular gap between the two planes. The AND, not OR,//
    is what keeps genuinely perpendicular corner walls from ever merging --
    their normal_similarity is close to 0 by construction (see the module's
    own unit tests), which alone already fails the first check regardless of
    how the offset_gap happens to come out.
    """
    normal_similarity = float(np.dot(wall_a['normal'], wall_b['normal']))
    if normal_similarity < normal_cos_thresh:
        return False
    return _offset_gap(wall_a, wall_b) < distance_thresh_m


def _refit_plane_least_squares(points: np.ndarray):
    """Deterministic least-squares plane fit through `points` (Nx3): the
    centroid, plus the smallest-singular-value right-singular-vector of the
    centered point matrix as the unit normal (standard total-least-squares
    plane fit via SVD). Deliberately NOT another RANSAC segment_plane call --
    every point in the pooled set already survived RANSAC once each as part
    of its own original candidate plane, so a straight least-squares fit
    through their union is both cheaper and, unlike RANSAC, fully
    deterministic -- important for these merge results to be exactly
    reproducible in a unit test, not just "close enough across runs".

    Returns (normal (unit, 3,), d (float, matching-scale plane offset,
    UNORIENTED -- caller reorients toward the robot origin the same way
    _find_walls does), centroid (3,)).
    """
    centroid = points.mean(axis=0)
    centered = points - centroid
    _, _, vt = np.linalg.svd(centered, full_matrices=False)
    normal = vt[-1]
    normal = normal / np.linalg.norm(normal)
    d = -float(np.dot(normal, centroid))
    return normal, d, centroid


def _wall_from_points(points: np.ndarray) -> dict:
    """Build a full wall dict (same shape _find_walls produces) from a raw
    Nx3 point set via the least-squares refit above -- shared by
    merge_duplicate_walls (refitting a pooled cluster) below."""
    normal, d, centroid = _refit_plane_least_squares(points)
    # Same "orient normal back toward the robot origin" convention
    # _find_walls's own segmentation loop already uses -- see this module's
    # own docstring for why this also pins d == distance afterward.
    if np.dot(normal, -centroid) < 0:
        normal = -normal
        d = -d
    distance = abs(d)
    bearing = math.atan2(centroid[1], centroid[0])
    extent = points.max(axis=0) - points.min(axis=0)
    return {
        'distance': float(distance),
        'bearing': float(bearing),
        'normal': normal,
        'centroid': centroid,
        'extent': extent,
        'points': points,
        'plane': (float(normal[0]), float(normal[1]), float(normal[2]), float(distance)),
    }


def merge_duplicate_walls(
        walls: list, normal_cos_thresh: float, distance_thresh_m: float,
        debug_log=None) -> list:
    """Cluster and merge near-coplanar duplicate wall detections within one
    frame's candidate list. Union-find over the pairwise _are_duplicate_planes
    relation -- same technique f1tenth_perception's own obstacle_projector_
    node.py already uses for its structurally identical "collapse duplicate
    detections into one" problem (_merge_close_obstacles), reused here for
    consistency rather than re-invented -- so THREE OR MORE mutually-
    duplicate candidates (a bad frame splitting one wall three ways) all
    collapse into a single merged wall, not just adjacent pairs.

    A cluster of size 1 (no duplicate found) passes through unchanged. A
    cluster of size >= 2 is merged by pooling every member's inlier points
    and refitting ONE plane through the union (_wall_from_points) --
    preferred over "keep the larger candidate" because a symmetric noise
    split means neither raw candidate alone is the best estimate of the true
    surface; the pooled refit uses strictly more real data than either.

    `debug_log`: TEMP DEBUG, merge-stage instrumentation pass, remove after --
    optional callable(str), None (default) in every normal caller including
    every existing unit test, so this is a pure no-op / zero-behavior-change
    everywhere it isn't explicitly passed. When given, logs every pairwise
    normal_similarity/offset_gap this function computes (recomputed here
    independently of _are_duplicate_planes' own internal values -- cheap,
    and keeps _are_duplicate_planes' signature/tests completely untouched)
    plus the final group count/membership.
    """
    n = len(walls)
    if n < 2:
        if debug_log is not None:
            debug_log(f'merge_duplicate_walls: n={n} candidate(s) -- nothing to compare')
        return list(walls)

    parent = list(range(n))

    def find(i):
        while parent[i] != i:
            parent[i] = parent[parent[i]]
            i = parent[i]
        return i

    def union(i, j):
        ri, rj = find(i), find(j)
        if ri != rj:
            parent[ri] = rj

    for i in range(n):
        for j in range(i + 1, n):
            is_dup = _are_duplicate_planes(
                walls[i], walls[j], normal_cos_thresh, distance_thresh_m)
            if debug_log is not None:
                normal_similarity = float(np.dot(walls[i]['normal'], walls[j]['normal']))
                gap = _offset_gap(walls[i], walls[j])
                debug_log(
                    f'pair ({i},{j}): normal_similarity={normal_similarity:.5f} '
                    f'vs normal_cos_thresh>={normal_cos_thresh:.5f} | '
                    f'offset_gap={gap:.5f}m vs distance_thresh_m<{distance_thresh_m:.5f} '
                    f'-> {"DUPLICATE (merge)" if is_dup else "separate"}'
                )
            if is_dup:
                union(i, j)

    groups = {}
    for i in range(n):
        groups.setdefault(find(i), []).append(i)

    if debug_log is not None:
        membership = {k: v for k, v in groups.items()}
        debug_log(f'merge result: {len(groups)} group(s) from {n} candidate(s), '
                  f'membership={membership}')

    merged = []
    for member_idxs in groups.values():
        if len(member_idxs) == 1:
            merged.append(walls[member_idxs[0]])
            continue
        pooled_points = np.concatenate([walls[i]['points'] for i in member_idxs], axis=0)
        merged.append(_wall_from_points(pooled_points))
    return merged


# ==============================================================================
# Ambiguous zone -- three-way classification (SAME/DISTINCT/AMBIGUOUS) of a
# candidate wall against an existing track's own plane, reusing the exact
# normal_similarity/offset_gap math above. Pure functions/classes, no rclpy
# dependency -- see module docstring's "Ambiguous zone" section for the full
# reasoning. Sits between merge_duplicate_walls (intra-frame, candidate vs.
# candidate) and WallTracker's existing distance/bearing association
# (candidate vs. track, by position) -- this is candidate vs. track, by
# plane geometry, only reached for whatever WallTracker's own association
# didn't already match by position.
# ==============================================================================

def _wall_like_from_track(track: 'TrackedWall') -> dict:
    """Build the same {'normal', 'centroid', 'plane'} shape merge_duplicate_
    walls' own wall dicts have, from a TrackedWall's current (possibly EMA-
    smoothed) state -- so _classify_wall_relationship/_offset_gap can compare
    a fresh candidate against a track exactly the same way they already
    compare two raw candidates against each other.

    Computes `plane`'s offset directly as d = -dot(normal, centroid) --
    the general plane-through-a-point-with-a-given-normal formula, valid
    regardless of orientation convention -- rather than assuming the "d ==
    distance after reorientation" shortcut _find_walls/_wall_from_points use
    for freshly-fit raw candidates. That shortcut relies on their specific
    "orient normal back toward the robot origin" step; nothing guarantees
    repeated EMA blending of several already-reoriented (normal, centroid)
    pairs stays exactly on that same convention, so this recomputes it
    properly instead of assuming it.
    """
    normal = np.asarray(track.normal, dtype=float)
    centroid = np.asarray(track.centroid, dtype=float)
    d = -float(np.dot(normal, centroid))
    return {
        'normal': normal,
        'centroid': centroid,
        'plane': (float(normal[0]), float(normal[1]), float(normal[2]), d),
    }


def _classify_wall_relationship(
        wall_a: dict, wall_b: dict, normal_cos_thresh: float, distance_thresh_m: float,
        min_distinct_separation_m: float) -> str:
    """Classify the pairwise relationship between two walls (candidate vs.
    candidate, or -- via _wall_like_from_track -- candidate vs. an existing
    track) into one of three zones. Boundaries are exhaustive/non-
    overlapping: normal_similarity below threshold is unconditionally
    'distinct' (matches _are_duplicate_planes' own short-circuit, so a
    genuine corner pair -- normal_similarity near 0 -- is always 'distinct',
    never 'ambiguous'); otherwise 'same' below distance_thresh_m (identical
    cutoff/inequality direction _are_duplicate_planes already uses), then
    'distinct' at/above min_distinct_separation_m, and whatever's left in
    between is 'ambiguous'.
    """
    normal_similarity = float(np.dot(wall_a['normal'], wall_b['normal']))
    if normal_similarity < normal_cos_thresh:
        return 'distinct'
    gap = _offset_gap(wall_a, wall_b)
    if gap < distance_thresh_m:
        return 'same'
    if gap >= min_distinct_separation_m:
        return 'distinct'
    return 'ambiguous'


class PendingWallGate:
    """Ambiguous-zone hold: candidates classified 'ambiguous' against every
    existing track (see WallTracker.update()) don't spawn a permanent track
    on first sight. They sit here instead until the SAME recurring candidate
    has been seen for `confirm_frames` CONSECUTIVE frames, matched frame to
    frame by the identical nearest-cost distance/bearing association
    WallTracker itself uses for confirmed tracks (assoc_distance_thresh_m/
    assoc_bearing_thresh_rad, passed in from the same WallTracker instance)
    -- kept a separate class, not folded into WallTracker's own body, so its
    spawn/match/promote/drop lifecycle stays independently testable, same
    "pure logic separate from ROS glue" split as merge_duplicate_walls/
    TrackedWall/WallTracker.

    A pending candidate not re-matched in a given frame is dropped outright,
    not held/decayed like a confirmed track -- "consecutive" is meant
    literally, no partial credit toward a later attempt.
    """

    def __init__(self, assoc_distance_thresh_m: float, assoc_bearing_thresh_rad: float,
                 confirm_frames: int):
        self.assoc_distance_thresh_m = assoc_distance_thresh_m
        self.assoc_bearing_thresh_rad = assoc_bearing_thresh_rad
        self.confirm_frames = confirm_frames
        self._pending: list = []  # [{'wall': dict, 'streak': int}, ...]

    @property
    def pending(self) -> list:
        return list(self._pending)

    def update(self, ambiguous_walls: list, debug_log=None) -> list:
        """ambiguous_walls: this frame's walls classified 'ambiguous' against
        every currently-alive track (see WallTracker.update()). Matches each
        to an existing pending candidate by distance/bearing (same shape as
        WallTracker's own association loop), increments its streak, or
        spawns a fresh pending entry (streak=1) for anything unmatched.
        Any previously-pending candidate NOT matched this frame is dropped.
        Returns the raw wall dict for every candidate that just reached
        confirm_frames -- the caller (WallTracker) is responsible for
        actually promoting these into real TrackedWall instances and must
        not see them here again (already removed from self._pending).

        `debug_log`: TEMP DEBUG (fragmentation-diagnosis pass, remove after)
        -- same no-op-when-None contract as everywhere else in this module.
        """
        candidates = []
        for pi, pending in enumerate(self._pending):
            for wi, w in enumerate(ambiguous_walls):
                d_gap = abs(w['distance'] - pending['wall']['distance'])
                raw_b_gap = w['bearing'] - pending['wall']['bearing']
                b_gap = abs(math.atan2(math.sin(raw_b_gap), math.cos(raw_b_gap)))
                if (d_gap <= self.assoc_distance_thresh_m
                        and b_gap <= self.assoc_bearing_thresh_rad):
                    candidates.append((d_gap + b_gap, pi, wi))
        candidates.sort(key=lambda c: c[0])

        assigned_pending = {}
        used_walls = set()
        for _cost, pi, wi in candidates:
            if pi in assigned_pending or wi in used_walls:
                continue
            assigned_pending[pi] = wi
            used_walls.add(wi)

        promoted = []
        new_pending = []
        for pi, pending in enumerate(self._pending):
            if pi not in assigned_pending:
                # ==== TEMP DEBUG (fragmentation-diagnosis pass, remove after) ====
                if debug_log is not None:
                    debug_log(
                        f'pending candidate (was streak={pending["streak"]}, '
                        f'distance={pending["wall"]["distance"]:.4f}) NOT re-matched '
                        'this frame -- dropped, no partial credit')
                # ==== END TEMP DEBUG ====
                continue  # not re-matched this frame -- dropped, no partial credit
            w = ambiguous_walls[assigned_pending[pi]]
            pending['wall'] = w
            pending['streak'] += 1
            if pending['streak'] >= self.confirm_frames:
                promoted.append(w)
                # ==== TEMP DEBUG (fragmentation-diagnosis pass, remove after) ====
                if debug_log is not None:
                    debug_log(
                        f'pending candidate (distance={w["distance"]:.4f}) reached '
                        f'streak={pending["streak"]}/{self.confirm_frames} -- PROMOTED')
                # ==== END TEMP DEBUG ====
            else:
                new_pending.append(pending)
                # ==== TEMP DEBUG (fragmentation-diagnosis pass, remove after) ====
                if debug_log is not None:
                    debug_log(
                        f'pending candidate (distance={w["distance"]:.4f}) streak now '
                        f'{pending["streak"]}/{self.confirm_frames}')
                # ==== END TEMP DEBUG ====

        for wi, w in enumerate(ambiguous_walls):
            if wi in used_walls:
                continue
            new_pending.append({'wall': w, 'streak': 1})
            # ==== TEMP DEBUG (fragmentation-diagnosis pass, remove after) ====
            if debug_log is not None:
                debug_log(
                    f'new pending candidate spawned (distance={w["distance"]:.4f} '
                    f'bearing_deg={math.degrees(w["bearing"]):.2f}), streak=1')
            # ==== END TEMP DEBUG ====

        self._pending = new_pending
        return promoted


# ==============================================================================
# Per-wall tracking + EMA smoothing -- runs AFTER merge, frame to frame. Pure
# Python/numpy, no rclpy dependency, independently unit-testable -- state
# lives entirely in a WallTracker instance owned by WallDetectorNode (not
# persisted anywhere else, per the task's own scope note).
# ==============================================================================

class TrackedWall:
    """One physical wall's smoothed state across frames. distance/bearing/
    normal are EMA-smoothed on every matched update (see update() below) --
    centroid is ALSO smoothed, for a reason not in the original enumerated
    smoothing list: distance (plane-to-origin) and bearing (direction to
    centroid) alone don't determine a 3D position, so there is no way to
    satisfy "wall_markers reflects the merged+smoothed values" (a separate,
    explicit requirement) without also smoothing SOME notion of where the
    wall actually is. Flagged here plainly since it's an addition beyond the
    task's own explicit distance/bearing/normal list, not a silent one.

    points/extent are the LAST-MATCHED frame's raw values, held (not
    smoothed) while unmatched -- blending point CLOUDS frame to frame isn't
    meaningful, so Foxglove's quad marker uses whichever real points were
    most recently observed rather than an average of several different
    point sets. is_corner is likewise just the latest observed value, never
    smoothed -- see the module's own note on why corner status specifically
    must not be treated as a smoothed/hysteresis'd quantity.

    frames_matched/n_inliers (added for front_clearance selection
    eligibility -- see module docstring's own section) are NOT smoothed
    either: frames_matched counts every successful match this track has
    ever had (monotonically increasing, never reset by a held-but-unmatched
    frame -- "how long has this track been real", not "how long since it
    was last seen", which frames_since_seen above already covers).
    n_inliers is just the latest matched frame's RANSAC inlier count
    (len(points)), same "last observed, not blended" treatment as points/
    extent above -- averaging inlier counts across frames wouldn't mean
    anything a plain latest-value doesn't already capture better.
    """

    def __init__(self, track_id: int, distance: float, bearing: float,
                 normal: np.ndarray, centroid: np.ndarray, is_corner: bool,
                 points: np.ndarray, extent: np.ndarray):
        self.track_id = track_id
        self.distance = float(distance)
        self.bearing = float(bearing)
        self.normal = np.asarray(normal, dtype=float) / np.linalg.norm(normal)
        self.centroid = np.asarray(centroid, dtype=float)
        self.is_corner = bool(is_corner)
        self.points = points
        self.extent = extent
        self.n_inliers = len(points) if points is not None else 0
        # 1 on the seeding frame itself -- this track has been matched
        # (this very sighting) exactly once so far. See front_clearance
        # selection eligibility note above for why this never resets.
        self.frames_matched = 1
        # 0 immediately after being seen (spawned or matched this frame);
        # counts consecutive frames since -- see WallTracker.update()'s own
        # hold/drop bookkeeping.
        self.frames_since_seen = 0

    def update(self, distance: float, bearing: float, normal: np.ndarray,
               centroid: np.ndarray, is_corner: bool, points: np.ndarray,
               extent: np.ndarray, alpha: float) -> None:
        """Blend this frame's matched raw detection into the smoothed state.
        Called only for tracks that found a match this frame -- see
        WallTracker.update(); a brand-new track is seeded directly (no
        blending on its first frame at all) via __init__ instead of this."""
        self.distance = alpha * float(distance) + (1.0 - alpha) * self.distance

        # Bearing: wrap the raw angular difference into (-pi, pi] via atan2
        # BEFORE blending, then re-wrap the result the same way -- a naive
        # `alpha*new + (1-alpha)*old` on raw bearing values would be wrong
        # any time old/new straddle the +-pi seam (e.g. old=+179deg,
        # new=-179deg is actually only a 2deg change, not a ~358deg one).
        raw_delta = float(bearing) - self.bearing
        wrapped_delta = math.atan2(math.sin(raw_delta), math.cos(raw_delta))
        blended_bearing = self.bearing + alpha * wrapped_delta
        self.bearing = math.atan2(math.sin(blended_bearing), math.cos(blended_bearing))

        # Normal: linear-blend then re-normalize -- a convex combination of
        # two unit vectors is NOT unit length in general (it's exactly unit
        # only when they're identical or antipodal), so every update must
        # renormalize, not just the first one.
        blended_normal = alpha * np.asarray(normal, dtype=float) + (1.0 - alpha) * self.normal
        norm_mag = np.linalg.norm(blended_normal)
        if norm_mag > 1e-9:
            self.normal = blended_normal / norm_mag
        # else: pathological (new and old normals exactly antipodal, alpha
        # exactly 0.5) -- keep the previous normal rather than divide by
        # ~zero; practically unreachable for real consecutive-frame data.

        self.centroid = alpha * np.asarray(centroid, dtype=float) + (1.0 - alpha) * self.centroid
        self.is_corner = bool(is_corner)
        self.points = points
        self.extent = extent
        self.n_inliers = len(points) if points is not None else 0
        self.frames_matched += 1
        self.frames_since_seen = 0


# ==============================================================================
# Motion compensation -- pure functions, no rclpy dependency, independently
# unit-testable. Runs at the START of WallTracker.update(), advancing every
# live track's stored (base_link-relative) position by the robot's own
# short-term motion since the last update tick, BEFORE association/EMA ever
# compares it against this frame's fresh detections. See module docstring's
# "Motion compensation" section for the full reasoning.
# ==============================================================================

# How old the most recent odometry message may be (seconds) before
# WallDetectorNode._consume_ego_delta() treats it as unavailable rather than
# using it -- a plain module constant, not a declared ROS param: this isn't
# a real tuning knob (any reasonable perception-stack odometry rate is a
# tiny fraction of this), just a sanity guard against a stalled/dead
# odometry publisher silently feeding stale motion data forever.
_ODOM_STALE_SEC = 0.5


def _wrap_angle(angle: float) -> float:
    """Wrap `angle` (radians) into (-pi, pi] -- same wraparound convention
    TrackedWall.update()'s own bearing blending already uses inline (see
    that method), pulled out here since motion-delta math needs it too."""
    return math.atan2(math.sin(angle), math.cos(angle))


def _odom_delta(prev_pose, curr_pose):
    """prev_pose/curr_pose: (x, y, yaw) tuples in a common fixed (odom)
    frame, from two consecutive odometry readings. Returns (dx, dy, dtheta):
    curr_pose's motion since prev_pose, expressed in prev_pose's OWN
    (robot-frame-at-prev-pose) coordinates -- exactly the "ego_delta" shape
    _predict_track_position expects.

    Deliberately a SHORT frame-to-frame delta (this function only ever sees
    two adjacent ticks, never an accumulated pose) -- see module docstring:
    this stays accurate even though the SAME odometry source's long-horizon
    absolute pose can drift, because it only asks "how far did the robot
    move since the immediately preceding frame", never "where is the robot
    in some fixed world frame"."""
    px, py, pyaw = prev_pose
    cx, cy, cyaw = curr_pose
    dtheta = _wrap_angle(cyaw - pyaw)
    ddx, ddy = cx - px, cy - py
    # Rotate the world-frame translation into prev_pose's own robot frame.
    cos_p, sin_p = math.cos(-pyaw), math.sin(-pyaw)
    dx = cos_p * ddx - sin_p * ddy
    dy = sin_p * ddx + cos_p * ddy
    return dx, dy, dtheta


def _predict_track_position(track: 'TrackedWall', ego_delta) -> dict:
    """Predict where `track` (a physically-stationary wall) should now
    appear in the CURRENT base_link frame, given the robot's own
    (dx, dy, dtheta) motion since track's position was last computed (see
    _odom_delta). A world-fixed point's coordinates in the new robot frame
    are obtained by undoing the robot's own motion: translate by -[dx, dy]
    then rotate by -dtheta (both expressed in the OLD frame, exactly what
    _odom_delta returns) -- standard relative-pose composition, applied
    here to a single tracked point+direction instead of a whole cloud.

    centroid's z and normal's z are left untouched (planar-motion
    assumption: the robot moves/rotates about the vertical axis only, never
    rolls/pitches -- the same assumption _transform_to_robot_frame's own
    tf2 lookup relies on being small/negligible for this rig). Returns a
    dict with the same {'distance', 'bearing', 'normal', 'centroid'} shape
    TrackedWall's own fields use, recomputed from the transformed geometry
    via the exact same reorientation convention _wall_from_points uses (so
    the invariants _offset_gap/_classify_wall_relationship rely on --
    d == distance after reorientation -- keep holding)."""
    dx, dy, dtheta = ego_delta
    cos_d, sin_d = math.cos(-dtheta), math.sin(-dtheta)

    cx, cy, cz = track.centroid
    tx, ty = cx - dx, cy - dy
    new_cx = cos_d * tx - sin_d * ty
    new_cy = sin_d * tx + cos_d * ty
    centroid = np.array([new_cx, new_cy, cz])

    nx, ny, nz = track.normal
    new_nx = cos_d * nx - sin_d * ny
    new_ny = sin_d * nx + cos_d * ny
    normal = np.array([new_nx, new_ny, nz])
    norm_mag = np.linalg.norm(normal)
    if norm_mag > 1e-9:
        normal = normal / norm_mag
    # else: pathological zero-length result -- practically unreachable for
    # a real unit normal rotated by a finite angle; leave unnormalized
    # rather than divide by ~zero (mirrors TrackedWall.update()'s own
    # guard for the same class of edge case).

    d = -float(np.dot(normal, centroid))
    if d < 0:
        normal = -normal
        d = -d
    distance = d
    bearing = math.atan2(centroid[1], centroid[0])

    return {'distance': distance, 'bearing': bearing, 'normal': normal, 'centroid': centroid}


class WallTracker:
    """Associates each frame's (post-merge) wall detections to existing
    tracks by nearest distance+bearing, EMA-smooths matched tracks, holds
    unmatched tracks for a bounded number of frames before dropping them,
    and spawns new tracks for unmatched detections. Deliberately does NOT
    assume stable ordering out of _find_walls/merge_duplicate_walls (their
    output order can and does change frame to frame, e.g. whichever plane
    RANSAC happens to fit first) -- association is by (distance, bearing)
    proximity, never by list index.

    Unmatched walls (didn't position-match any existing track above) no
    longer spawn a permanent track unconditionally -- see module docstring's
    "Ambiguous zone" section. Each is classified (_classify_wall_relationship,
    plane geometry, not position) against every still-alive track: SAME
    merges it in directly; DISTINCT (or no tracks at all yet) spawns
    immediately, same as the old behavior; AMBIGUOUS is held in
    self._pending_gate (a PendingWallGate) until it's reappeared for
    ambiguous_confirm_frames consecutive frames.
    """

    def __init__(self, assoc_distance_thresh_m: float, assoc_bearing_thresh_rad: float,
                 hold_frames: int, alpha: float, merge_normal_cos_thresh: float,
                 merge_distance_thresh_m: float, min_distinct_separation_m: float,
                 ambiguous_confirm_frames: int,
                 prune_conflict_radius_m: float = 2.0,
                 prune_min_frames_before_eligible: int = 3,
                 prune_inlier_ratio_floor: float = 0.5,
                 prune_match_streak_ratio_floor: float = 0.5):
        self.assoc_distance_thresh_m = assoc_distance_thresh_m
        self.assoc_bearing_thresh_rad = assoc_bearing_thresh_rad
        self.hold_frames = hold_frames
        self.alpha = alpha
        self.merge_normal_cos_thresh = merge_normal_cos_thresh
        self.merge_distance_thresh_m = merge_distance_thresh_m
        self.min_distinct_separation_m = min_distinct_separation_m
        # ---- Confidence-based pruning (see module docstring + prune_
        # inconsistent_tracks, called once per update() tick below).
        # Defaulted (unlike the params above) so every existing caller --
        # tests included -- that doesn't care about pruning keeps working
        # unchanged; WallDetectorNode's own construction always passes
        # these explicitly.
        self.prune_conflict_radius_m = prune_conflict_radius_m
        self.prune_min_frames_before_eligible = prune_min_frames_before_eligible
        self.prune_inlier_ratio_floor = prune_inlier_ratio_floor
        self.prune_match_streak_ratio_floor = prune_match_streak_ratio_floor
        self._tracks: list = []
        self._next_id = 0
        self._pending_gate = PendingWallGate(
            assoc_distance_thresh_m, assoc_bearing_thresh_rad, ambiguous_confirm_frames)

    @property
    def tracks(self) -> list:
        return list(self._tracks)

    @property
    def pending(self) -> list:
        """This frame's still-unconfirmed ambiguous-zone candidates -- never
        published (see module docstring), exposed only for tests/debugging."""
        return self._pending_gate.pending

    def update(self, walls: list, ego_delta=None, debug_log=None) -> list:
        """walls: this frame's post-merge wall dicts (distance/bearing/
        normal/centroid/is_corner/points/extent). Returns the current list
        of live TrackedWall instances -- every track that was matched this
        frame, every track still within its hold window (matched or not),
        every freshly-spawned track for an unmatched DISTINCT detection, and
        every ambiguous-zone candidate that just reached
        ambiguous_confirm_frames. Tracks that just exceeded hold_frames, and
        ambiguous candidates not confirmed yet, are NOT included. Tracks
        pruned this tick (see prune_inconsistent_tracks below) are also NOT
        included, same as a hold-frame drop.

        `ego_delta`: (dx, dy, dtheta) -- the robot's own motion since the
        last update() tick (see _odom_delta), or None (default) to disable
        motion compensation entirely for this call -- identical to this
        feature not existing, used both when odometry is genuinely
        unavailable/stale and by every existing caller (tests included)
        that doesn't pass it. See module docstring's "Motion compensation"
        section.

        `debug_log`: TEMP DEBUG (fragmentation-diagnosis pass, remove after)
        -- optional callable(str), None (default) everywhere except the
        node's own live-capture wiring, same no-op-when-absent contract
        merge_duplicate_walls' own debug_log already uses. Logs every
        candidate-vs-track classification this method computes (the
        ambiguous-zone loop below) -- the one decision point the ORIGINAL
        merge-stage TEMP DEBUG never covered, since that only logs
        candidate-vs-candidate comparisons within one frame
        (merge_duplicate_walls), not candidate-vs-existing-track across
        frames (here).
        """
        # ---- Motion compensation: advance every live track's stored
        # position to where it should now appear, BEFORE association ever
        # compares it against this frame's fresh detections -- see module
        # docstring. A no-op when ego_delta is None (odometry unavailable/
        # stale this frame, or the caller doesn't use this feature at all).
        if ego_delta is not None:
            for track in self._tracks:
                predicted = _predict_track_position(track, ego_delta)
                track.distance = predicted['distance']
                track.bearing = predicted['bearing']
                track.normal = predicted['normal']
                track.centroid = predicted['centroid']

        # Greedy nearest-first assignment: build every (track, wall) pair
        # that passes BOTH association thresholds, sort by combined cost,
        # then assign lowest-cost-first, skipping anything whose track or
        # wall side is already claimed. Simple and sufficient at the scale
        # this ever runs at (at most a handful of walls per frame) --
        # not the Hungarian algorithm, deliberately, for that reason.
        candidates = []
        for ti, track in enumerate(self._tracks):
            for wi, w in enumerate(walls):
                d_gap = abs(w['distance'] - track.distance)
                raw_b_gap = w['bearing'] - track.bearing
                b_gap = abs(math.atan2(math.sin(raw_b_gap), math.cos(raw_b_gap)))
                if (d_gap <= self.assoc_distance_thresh_m
                        and b_gap <= self.assoc_bearing_thresh_rad):
                    candidates.append((d_gap + b_gap, ti, wi))
        candidates.sort(key=lambda c: c[0])

        assigned_tracks = {}
        used_walls = set()
        for _cost, ti, wi in candidates:
            if ti in assigned_tracks or wi in used_walls:
                continue
            assigned_tracks[ti] = wi
            used_walls.add(wi)

        new_track_list = []
        for ti, track in enumerate(self._tracks):
            if ti in assigned_tracks:
                w = walls[assigned_tracks[ti]]
                track.update(
                    w['distance'], w['bearing'], w['normal'], w['centroid'],
                    w['is_corner'], w['points'], w['extent'], self.alpha,
                )
                new_track_list.append(track)
            else:
                track.frames_since_seen += 1
                if track.frames_since_seen <= self.hold_frames:
                    new_track_list.append(track)
                # else: exceeded the hold window -- dropped, not carried
                # forward, so it stops rendering in Foxglove/publishing.

        # ---- Ambiguous zone: classify every position-unmatched wall against
        # every still-alive track (new_track_list -- matched or held, NOT the
        # ones just dropped above) by plane geometry, not position. See
        # module docstring's "Ambiguous zone" section.
        distinct_walls = []
        ambiguous_walls = []
        for wi, w in enumerate(walls):
            if wi in used_walls:
                continue

            merged_into = None
            any_ambiguous = False
            for track in new_track_list:
                track_like = _wall_like_from_track(track)
                zone = _classify_wall_relationship(
                    w, track_like, self.merge_normal_cos_thresh,
                    self.merge_distance_thresh_m, self.min_distinct_separation_m)
                # ==== TEMP DEBUG (fragmentation-diagnosis pass, remove after) ====
                if debug_log is not None:
                    normal_similarity = float(np.dot(w['normal'], track_like['normal']))
                    gap = _offset_gap(w, track_like)
                    debug_log(
                        f'candidate(wall_distance={w["distance"]:.4f} '
                        f'bearing_deg={math.degrees(w["bearing"]):.2f}) vs '
                        f'track_id={track.track_id}(distance={track.distance:.4f} '
                        f'bearing_deg={math.degrees(track.bearing):.2f}): '
                        f'normal_similarity={normal_similarity:.5f} '
                        f'vs merge_normal_cos_thresh>={self.merge_normal_cos_thresh:.5f} | '
                        f'offset_gap={gap:.5f}m vs merge_distance_thresh_m<'
                        f'{self.merge_distance_thresh_m:.5f} vs '
                        f'min_distinct_separation_m>={self.min_distinct_separation_m:.5f} '
                        f'-> zone={zone}')
                # ==== END TEMP DEBUG ====
                if zone == 'same':
                    merged_into = track
                    break
                if zone == 'ambiguous':
                    any_ambiguous = True

            if merged_into is not None:
                # Reappeared as a confident geometric match for an existing
                # track (possibly after previously sitting in the pending
                # gate under a different position bucket) -- merge directly,
                # same EMA update the position-matched path above uses. Its
                # own pending state (if any) is simply never re-extended: it
                # isn't added to ambiguous_walls below, so PendingWallGate.
                # update() won't re-match it and will drop it as unmatched.
                merged_into.update(
                    w['distance'], w['bearing'], w['normal'], w['centroid'],
                    w['is_corner'], w['points'], w['extent'], self.alpha,
                )
                # ==== TEMP DEBUG (fragmentation-diagnosis pass, remove after) ====
                if debug_log is not None:
                    debug_log(f'outcome: merged directly into track_id={merged_into.track_id}')
                # ==== END TEMP DEBUG ====
            elif any_ambiguous:
                ambiguous_walls.append(w)
                # ==== TEMP DEBUG (fragmentation-diagnosis pass, remove after) ====
                if debug_log is not None:
                    debug_log('outcome: AMBIGUOUS -- routed to pending gate')
                # ==== END TEMP DEBUG ====
            else:
                # DISTINCT from every still-alive track (or there are no
                # tracks at all yet) -- spawn immediately, same as the old
                # unconditional-spawn behavior.
                distinct_walls.append(w)
                # ==== TEMP DEBUG (fragmentation-diagnosis pass, remove after) ====
                if debug_log is not None:
                    debug_log('outcome: DISTINCT from every live track -- spawning immediately')
                # ==== END TEMP DEBUG ====

        for w in distinct_walls:
            # No smoothing at all on a track's first frame -- EMA state is
            # seeded directly with the raw detection, not blended against
            # some assumed prior.
            new_track_list.append(TrackedWall(
                self._next_id, w['distance'], w['bearing'], w['normal'],
                w['centroid'], w['is_corner'], w['points'], w['extent'],
            ))
            self._next_id += 1

        for w in self._pending_gate.update(ambiguous_walls, debug_log=debug_log):
            # Just reached ambiguous_confirm_frames -- promote to a real
            # track exactly like a DISTINCT first sighting (unsmoothed seed).
            new_track_list.append(TrackedWall(
                self._next_id, w['distance'], w['bearing'], w['normal'],
                w['centroid'], w['is_corner'], w['points'], w['extent'],
            ))
            self._next_id += 1

        # ---- Confidence-based pruning: after association/EMA/spawn/
        # promotion, before publish -- see module docstring. Drops the
        # weaker of any pair of live tracks that are conflicting (close +
        # non-perpendicular) AND meaningfully unequal in support; a
        # near-duplicate that keeps re-matching itself every frame (so
        # never ages out via hold_frames) is exactly what this catches.
        drop_ids = prune_inconsistent_tracks(
            new_track_list, self.merge_normal_cos_thresh, self.merge_distance_thresh_m,
            self.prune_conflict_radius_m, self.prune_min_frames_before_eligible,
            self.prune_inlier_ratio_floor, self.prune_match_streak_ratio_floor)
        if drop_ids:
            # ==== TEMP DEBUG (fragmentation-diagnosis pass, remove after) ====
            if debug_log is not None:
                for tid in drop_ids:
                    debug_log(
                        f'PRUNED track_id={tid} -- meaningfully weaker duplicate of '
                        'a conflicting live track')
            # ==== END TEMP DEBUG ====
            drop_id_set = set(drop_ids)
            new_track_list = [t for t in new_track_list if t.track_id not in drop_id_set]

        self._tracks = new_track_list
        return list(self._tracks)


# ==============================================================================
# Confidence-based pruning -- pure functions, no rclpy dependency,
# independently unit-testable. Called once per WallTracker.update() tick,
# after association/EMA/spawn/promotion, right before publish. See module
# docstring's "Confidence-based pruning" section for the full reasoning.
# ==============================================================================

def _weaker_track(
        track_a: 'TrackedWall', track_b: 'TrackedWall', min_frames_before_prune: int,
        inlier_ratio_floor: float, match_streak_ratio_floor: float):
    """Given a conflicting pair (track_a, track_b) already known to occupy
    roughly the same physical space (see prune_inconsistent_tracks), decide
    which one -- if either -- is a MEANINGFULLY weaker duplicate of the
    other. Returns the loser's track_id, or None if neither is meaningfully
    weaker (comparable support -- keep both; or, rarely, each looks weaker
    than the other by a DIFFERENT signal simultaneously -- ambiguous, so
    also keep both rather than guess).

    A track is only eligible to LOSE (not to win -- a fresh track can still
    correctly out-compete a genuinely weaker, older rival) once it has
    accumulated min_frames_before_prune matched frames -- see module
    docstring's grace-period note."""
    a_eligible = track_a.frames_matched >= min_frames_before_prune
    b_eligible = track_b.frames_matched >= min_frames_before_prune

    def _meaningfully_weaker(weaker, stronger, eligible) -> bool:
        if not eligible:
            return False
        inlier_weak = (
            stronger.n_inliers > 0
            and weaker.n_inliers < inlier_ratio_floor * stronger.n_inliers)
        streak_weak = (
            stronger.frames_matched > 0
            and weaker.frames_matched < match_streak_ratio_floor * stronger.frames_matched)
        return inlier_weak or streak_weak

    a_weaker = _meaningfully_weaker(track_a, track_b, a_eligible)
    b_weaker = _meaningfully_weaker(track_b, track_a, b_eligible)

    if a_weaker and not b_weaker:
        return track_a.track_id
    if b_weaker and not a_weaker:
        return track_b.track_id
    return None


def prune_inconsistent_tracks(
        tracks: list, normal_cos_thresh: float, distance_thresh_m: float,
        conflict_radius_m: float, min_frames_before_prune: int,
        inlier_ratio_floor: float, match_streak_ratio_floor: float) -> list:
    """Returns the track_ids to drop this tick: for every pair of `tracks`,
    reuses _classify_wall_relationship (the SAME normal_similarity/
    offset_gap "should be one wall" test merge_duplicate_walls and the
    ambiguous zone already use) with conflict_radius_m standing in for the
    distinct-separation threshold -- zone != 'distinct' (i.e. 'same' or
    'ambiguous') means the pair is geometrically consistent enough to be
    plausibly the same physical surface ("conflicting"); 'distinct' means
    either a genuine corner (normal_similarity too low -- see
    _classify_wall_relationship's own corner-safety note) or genuinely far
    apart, and is left alone regardless of support.

    normal_cos_thresh/distance_thresh_m: the SAME merge_normal_cos_thresh/
    merge_distance_thresh_m values _classify_wall_relationship is already
    called with elsewhere in this module (passed in by WallTracker.update(),
    which already holds them) -- not new tunables, just the existing
    "is this a corner vs. a duplicate" thresholds reused for a third
    purpose, per module docstring.

    For each conflicting pair, _weaker_track decides whether one side is
    meaningfully weaker (n_inliers/frames_matched, both gated by
    min_frames_before_prune's grace period) and should be dropped. A
    track_id already queued for drop is skipped as either side of any
    further pair this same tick (already resolved)."""
    to_drop = []
    dropped_ids = set()
    n = len(tracks)
    for i in range(n):
        track_a = tracks[i]
        if track_a.track_id in dropped_ids:
            continue
        a_like = _wall_like_from_track(track_a)
        for j in range(i + 1, n):
            track_b = tracks[j]
            if track_b.track_id in dropped_ids:
                continue
            b_like = _wall_like_from_track(track_b)
            zone = _classify_wall_relationship(
                a_like, b_like, normal_cos_thresh, distance_thresh_m, conflict_radius_m)
            if zone == 'distinct':
                continue
            loser_id = _weaker_track(
                track_a, track_b, min_frames_before_prune,
                inlier_ratio_floor, match_streak_ratio_floor)
            if loser_id is not None:
                to_drop.append(loser_id)
                dropped_ids.add(loser_id)
                if loser_id == track_a.track_id:
                    break  # track_a itself is gone -- no point comparing it further
    return to_drop


# ==============================================================================
# Raw candidate quality gating -- pure functions, no rclpy dependency,
# independently unit-testable (same convention as merge_duplicate_walls/
# _classify_wall_relationship above). Applied ONLY to the SECOND (and any
# further) RANSAC plane candidate within _find_walls's iterative search --
# the FIRST/dominant plane doesn't show the failure mode these target. See
# module docstring's "Raw candidate quality gating (second-plane-only)"
# section for the full investigation writeup these are built from.
# ==============================================================================

def _in_plane_extent(points: np.ndarray, normal: np.ndarray, centroid: np.ndarray):
    """Project `points` (Nx3) onto an orthonormal in-plane basis (the same
    along/up construction _wall_markers uses for its quad marker) and
    return (extent_along, extent_up) -- the point spread along each in-
    plane axis. A coherent planar surface has comparable spread in both
    directions; a spurious fit through a sparse/linear residual cluster
    tends to be long and thin in one axis, near-zero in the other -- see
    _second_plane_compact_ok, the only caller."""
    normal = np.asarray(normal, dtype=float)
    world_up = np.array([0.0, 0.0, 1.0])
    if abs(np.dot(normal, world_up)) > 0.99:
        # Only reachable if normal is near-vertical, which verticality_max_deg
        # should already have filtered out upstream -- guarded anyway so this
        # never divides by a near-zero cross product (mirrors _wall_markers).
        world_up = np.array([1.0, 0.0, 0.0])
    along = np.cross(world_up, normal)
    along = along / np.linalg.norm(along)
    up = np.cross(normal, along)
    up = up / np.linalg.norm(up)
    rel = np.asarray(points, dtype=float) - np.asarray(centroid, dtype=float)
    proj_along = rel @ along
    proj_up = rel @ up
    return float(proj_along.max() - proj_along.min()), float(proj_up.max() - proj_up.min())


def _residual_supports_second_plane(
        residual_size: int, min_inliers: int, min_ratio: float) -> bool:
    """True if the residual cloud (what's left after removing the dominant
    plane's inliers) is large enough to plausibly contain a REAL second
    surface, checked BEFORE spending a segment_plane() call on it at all. A
    residual barely bigger than min_inliers itself is implausible to hide a
    genuine second wall's worth of RANSAC support underneath a real one."""
    return residual_size >= min_ratio * min_inliers


def _second_plane_inlier_ratio_ok(
        inlier_count: int, residual_size_before: int, min_ratio: float):
    """(passed, inlier_ratio) -- inlier support as a fraction of the residual
    cloud the second segment_plane() call actually searched. A low-density
    fit (this function's failure case) is the empirical signature of the
    investigation's frame-9 example: a plane that only explains a small
    slice of what was left, rather than a genuine, well-supported second
    surface (e.g. a real corner's other wall) explaining most of it."""
    ratio = (inlier_count / residual_size_before) if residual_size_before else 0.0
    return ratio >= min_ratio, ratio


def _second_plane_compact_ok(
        pts_arr: np.ndarray, normal: np.ndarray, centroid: np.ndarray,
        max_aspect_ratio: float):
    """(passed, aspect_ratio) -- in-plane bounding-extent aspect ratio
    (longer axis / shorter axis) of the fitted inlier set, via
    _in_plane_extent. A coherent planar wall patch has comparable spread in
    both in-plane axes; a fragment/sliver fit through sparse residual noise
    tends to be long and thin in one axis, near-zero in the other -- this
    function's failure case."""
    ext_along, ext_up = _in_plane_extent(pts_arr, normal, centroid)
    lo, hi = sorted((ext_along, ext_up))
    ratio = (hi / lo) if lo > 1e-6 else math.inf
    return ratio <= max_aspect_ratio, ratio


# ==============================================================================
# front_clearance selection eligibility -- pure function, no rclpy
# dependency. Runs in _publish(), on top of the existing front-facing
# bearing-cone check, before a TrackedWall's distance counts toward
# min(front_candidates). See module docstring's "front_clearance selection
# eligibility" section for the full investigation writeup this is built
# from (independent of, and defensive on top of, the raw-detection-stage
# gates above).
# ==============================================================================

def _front_clearance_eligible(
        track: 'TrackedWall', front_facing_max_rad: float,
        min_track_frames: int, min_inliers: int) -> bool:
    """True if `track` is both within the front-facing bearing cone AND
    meets the stability bar (minimum matched-frame age + minimum RANSAC
    inlier support) required to count toward /perception/front_clearance.
    Kept as a pure function (mirrors _classify_wall_relationship's own
    split from WallTracker) so the selection rule is unit-testable without
    a live Node/publisher."""
    if abs(track.bearing) >= front_facing_max_rad:
        return False
    return track.frames_matched >= min_track_frames and track.n_inliers >= min_inliers


# ==============================================================================
# Hard boundary constraint conversion -- pure function, no rclpy dependency.
# See module docstring's "Hard boundary constraints" section for the full
# sign-convention derivation this is built from.
# ==============================================================================

def _wall_boundary_from_track(track: 'TrackedWall'):
    """Convert `track`'s plane fit into a (normal_x, normal_y, offset)
    BoundaryConstraint tuple, base_link-relative (the SAME frame track's
    own normal/centroid/distance already live in -- no transform here,
    that's the consumer's job, see module docstring). Returns (0.0, 0.0,
    math.inf) -- a naturally inert/disabled constraint under mpc_solver's
    own pad_boundary_constraints sentinel convention -- for the
    practically-unreachable pathological case of a near-perfectly-vertical
    normal (should already be excluded upstream by verticality_max_deg),
    rather than dividing by ~zero."""
    nx, ny = float(track.normal[0]), float(track.normal[1])
    k = math.hypot(nx, ny)
    if k < 1e-9:
        return 0.0, 0.0, math.inf
    return -nx / k, -ny / k, track.distance / k


def _select_boundary_track(eligible_tracks: list, last_track_id):
    """Pick which of this tick's `eligible_tracks` (a non-empty list of
    TrackedWall, already filtered by _front_clearance_eligible) becomes the
    published front_wall_boundary constraint -- STICKY, not a fresh
    min(distance) re-selection every tick. See module docstring's "Track-
    identity stickiness" section for why: a fresh re-selection has no
    memory of what was published last tick, so two simultaneously-eligible
    tracks sitting close enough in distance for per-frame RANSAC/EMA noise
    to occasionally reorder them would flip the published constraint back
    and forth on pure noise, even though NEITHER track's own eligibility
    ever lapsed.

    If `last_track_id` (the track_id published last tick, or None) is
    still present in `eligible_tracks`, keep publishing that SAME track --
    even if a different eligible track now has a strictly smaller
    `distance`; both are independently trustworthy enough to publish, and
    switching between them on noise alone is exactly the churn this
    exists to prevent. Otherwise (no previous selection, or it dropped out
    of eligibility/existence entirely this tick) there is no continuity to
    preserve, so this falls back to the same min(distance) selection as
    before."""
    for t in eligible_tracks:
        if t.track_id == last_track_id:
            return t
    return min(eligible_tracks, key=lambda t: t.distance)


class WallDetectorNode(Node):
    def __init__(self):
        super().__init__('wall_detector_node')

        # ---- Parameters ----
        self.declare_parameter('input_topic', '/zed2/zed_node/point_cloud/cloud_registered')
        self.declare_parameter('input_frame_convention', 'z_up_x_forward')  # or 'zed_optical'
        self.declare_parameter('voxel_size', 0.05)          # m, downsample resolution
        self.declare_parameter('roi_x_min', 0.2)             # m, min forward distance to consider
        self.declare_parameter('roi_x_max', 6.0)             # m, max forward distance
        self.declare_parameter('roi_y_half_width', 3.0)      # m, +/- lateral window
        self.declare_parameter('roi_z_min', 0.05)            # m, exclude ground near z=0
        # m, exclude anything above robot height of interest
        self.declare_parameter('roi_z_max', 1.5)
        self.declare_parameter('ransac_distance_threshold', 0.03)  # m, inlier distance
        self.declare_parameter('ransac_n', 3)
        self.declare_parameter('ransac_iterations', 1000)
        self.declare_parameter('min_inliers', 150)            # reject spurious/small planes
        # look for up to 2 planes (handles corners)
        self.declare_parameter('max_walls', 2)
        # normal must be within this of horizontal
        self.declare_parameter('verticality_max_deg', 20.0)
        # |bearing| under this counts toward front_clearance
        self.declare_parameter('front_facing_max_deg', 35.0)
        # how close to 90 deg counts as a corner
        self.declare_parameter('corner_perp_tolerance_deg', 25.0)
        self.declare_parameter('publish_rate_hz', 10.0)
        self.declare_parameter('robot_frame', 'base_link')

        # ---- Duplicate/near-coplanar plane merge (see merge_duplicate_walls) ----
        # dot(normal_a, normal_b) above this = "same direction" -- see
        # stack_params.yaml's own comment for the live-tuning evidence
        # behind 0.94 (was 0.97).
        self.declare_parameter('merge_normal_cos_thresh', 0.94)
        # m, inter-plane offset gap below this = "same surface" -- see
        # stack_params.yaml's own comment for the live-tuning evidence
        # behind 0.15 (was 0.08, then 0.12).
        self.declare_parameter('merge_distance_thresh_m', 0.15)

        # ---- Ambiguous zone (see PendingWallGate / module docstring) ----
        # m, offset gap at/above this = confidently a different wall, no
        # ambiguous hold.
        self.declare_parameter('min_distinct_separation_m', 2.0)
        # consecutive frames an ambiguous-zone candidate must recur before
        # promotion.
        self.declare_parameter('ambiguous_confirm_frames', 5)

        # ---- Per-wall tracking + EMA smoothing (see WallTracker) ----
        # m, max distance jump to still count as the same wall.
        self.declare_parameter('track_assoc_distance_thresh_m', 0.4)
        # deg, max bearing jump to still count as the same wall.
        self.declare_parameter('track_assoc_bearing_thresh_deg', 15.0)
        # frames an unmatched track survives before being dropped.
        self.declare_parameter('track_hold_frames', 5)
        # smoothing factor -- see stack_params.yaml's own note on tuning.
        self.declare_parameter('ema_alpha', 0.3)

        # ---- Raw candidate quality gating (see module docstring's "Raw
        # candidate quality gating (second-plane-only)" section). Node-
        # local, not launch-arg-exposed -- same precedent as their nearest
        # neighbors ransac_distance_threshold/min_inliers/ransac_n/
        # ransac_iterations above, also internals of this detection stage.
        # All defaults below are REASONED STARTING POINTS pending live-
        # capture validation (blocked as of this writing on a ZED hardware/
        # cable issue, unrelated to this code) -- not tuned against real
        # sensor noise yet, flagged deliberately, not silently.
        #
        # nb_neighbors/std_ratio for Open3D's remove_statistical_outlier,
        # applied to the whole cloud once, before any segment_plane call.
        self.declare_parameter('outlier_nb_neighbors', 20)
        self.declare_parameter('outlier_std_ratio', 2.0)
        # Second-plane-only gates (idx > 0 in _find_walls's loop -- never
        # applied to the dominant/first plane, which doesn't show this
        # failure mode). residual size must be at least this multiple of
        # min_inliers before even attempting a second segment_plane() call.
        self.declare_parameter('second_plane_min_residual_ratio', 2.0)
        # inliers / residual_size_before must clear this floor.
        self.declare_parameter('second_plane_min_inlier_ratio', 0.6)
        # in-plane bounding-extent aspect ratio (longer/shorter axis) must
        # stay at/under this.
        self.declare_parameter('second_plane_max_aspect_ratio', 6.0)

        # ---- front_clearance selection eligibility (see module docstring's
        # own section) -- min_track_frames reuses ambiguous_confirm_frames's
        # own default (5) as this codebase's established "distinguish real
        # persistence from transient noise" frame count; min_inliers is a
        # REASONED STARTING POINT (2x the base min_inliers floor above),
        # same live-validation-pending caveat as the gates above. Exposed
        # as launch args (unlike the detection-stage gates above) since
        # this directly affects a mission-critical stop condition -- same
        # precedent as front_facing_max_deg/track_hold_frames.
        self.declare_parameter('front_clearance_min_track_frames', 5)
        self.declare_parameter('front_clearance_min_inliers', 300)

        # ---- Confidence-based pruning (see module docstring's own
        # section). Exposed as launch args (like front_clearance_min_*
        # above, not node-local like the raw-detection-stage gates) since
        # this affects which tracks survive to be published/considered for
        # front_clearance at all.
        self.declare_parameter('prune_conflict_radius_m', 2.0)
        self.declare_parameter('prune_min_frames_before_eligible', 3)
        self.declare_parameter('prune_inlier_ratio_floor', 0.5)
        self.declare_parameter('prune_match_streak_ratio_floor', 0.5)

        # ---- Motion compensation (see module docstring's own section).
        # odom_topic follows get_odom_topic() (localization_source) by
        # default -- node-local, not launch-arg-exposed, since there's no
        # real reason to override it independently of localization_source
        # itself (same reasoning the raw-detection-stage gates above
        # already used for staying node-local).
        self.declare_parameter('odom_topic', get_odom_topic())

        p = self.get_parameter
        self.voxel_size = p('voxel_size').value
        self.roi_x_min = p('roi_x_min').value
        self.roi_x_max = p('roi_x_max').value
        self.roi_y_half = p('roi_y_half_width').value
        self.roi_z_min = p('roi_z_min').value
        self.roi_z_max = p('roi_z_max').value
        self.dist_thresh = p('ransac_distance_threshold').value
        self.ransac_n = p('ransac_n').value
        self.ransac_iters = p('ransac_iterations').value
        self.min_inliers = p('min_inliers').value
        self.max_walls = p('max_walls').value
        self.verticality_max = math.radians(p('verticality_max_deg').value)
        self.front_facing_max = math.radians(p('front_facing_max_deg').value)
        self.corner_perp_tol = math.radians(p('corner_perp_tolerance_deg').value)
        self.frame_convention = p('input_frame_convention').value
        self.robot_frame = p('robot_frame').value
        self.merge_normal_cos_thresh = p('merge_normal_cos_thresh').value
        self.merge_distance_thresh_m = p('merge_distance_thresh_m').value
        self.outlier_nb_neighbors = p('outlier_nb_neighbors').value
        self.outlier_std_ratio = p('outlier_std_ratio').value
        self.second_plane_min_residual_ratio = p('second_plane_min_residual_ratio').value
        self.second_plane_min_inlier_ratio = p('second_plane_min_inlier_ratio').value
        self.second_plane_max_aspect_ratio = p('second_plane_max_aspect_ratio').value
        self.front_clearance_min_track_frames = p('front_clearance_min_track_frames').value
        self.front_clearance_min_inliers = p('front_clearance_min_inliers').value

        self.tracker = WallTracker(
            assoc_distance_thresh_m=p('track_assoc_distance_thresh_m').value,
            assoc_bearing_thresh_rad=math.radians(p('track_assoc_bearing_thresh_deg').value),
            hold_frames=p('track_hold_frames').value,
            alpha=p('ema_alpha').value,
            merge_normal_cos_thresh=self.merge_normal_cos_thresh,
            merge_distance_thresh_m=self.merge_distance_thresh_m,
            min_distinct_separation_m=p('min_distinct_separation_m').value,
            ambiguous_confirm_frames=p('ambiguous_confirm_frames').value,
            prune_conflict_radius_m=p('prune_conflict_radius_m').value,
            prune_min_frames_before_eligible=p('prune_min_frames_before_eligible').value,
            prune_inlier_ratio_floor=p('prune_inlier_ratio_floor').value,
            prune_match_streak_ratio_floor=p('prune_match_streak_ratio_floor').value,
        )

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.sub = self.create_subscription(
            PointCloud2, p('input_topic').value, self.cloud_callback, qos)

        # ---- Motion compensation odometry (see module docstring). BEST_
        # EFFORT/VOLATILE, matching MPC_corr.py's own odometry subscription
        # QoS exactly -- both /odometry/filtered and /odom are published
        # with this QoS on this stack, and a mismatched DDS QoS silently
        # drops the connection rather than erroring, so this must match.
        odom_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.odom_topic = p('odom_topic').value
        self.odom_sub = self.create_subscription(
            Odometry, self.odom_topic, self._odom_cb, odom_qos)
        # (x, y, yaw, stamp_sec) from the most recent odometry message --
        # updated every odom callback (cheap), independent of cloud_callback's
        # own (much slower) rate.
        self._latest_odom_pose = None
        # (x, y, yaw) snapshot as of the LAST cloud_callback tick -- this IS
        # the "since the tracker's last update tick" reference _odom_delta
        # needs; advanced once per cloud_callback call by
        # _consume_ego_delta(), never touched by _odom_cb directly.
        self._last_tracker_odom_pose = None

        self.wall_pub = self.create_publisher(WallArray, '/perception/wall_detections', 10)
        self.clearance_pub = self.create_publisher(Float32, '/perception/front_clearance', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/perception/wall_markers', 10)
        self.front_wall_boundary_pub = self.create_publisher(
            BoundaryConstraintArray, '/perception/front_wall_boundary', 10)

        self._last_clearance = math.inf
        # Which track_ids had markers published last cycle -- diffed against
        # the current cycle's ids every _publish() call so a dropped/no-
        # longer-detected track's markers get an explicit DELETE instead of
        # lingering in Foxglove until something else happens to reuse that
        # id. See _publish()'s own comment for the full reasoning.
        self._last_published_track_ids = set()
        # track_id last selected as front_wall_boundary's own winner, or
        # None -- see _select_boundary_track's own docstring ("Track-
        # identity stickiness" in the module docstring).
        self._last_boundary_track_id = None

        # ==== TEMP DEBUG (merge-stage instrumentation pass, remove after) ====
        self._merge_debug_frame_count = 0
        # ==== END TEMP DEBUG ====

        # GATE_DEBUG: cumulative pass/fail counts for the three Part A
        # second-plane gates + outlier removal -- see module docstring's
        # "Raw candidate quality gating" section. Opt-in-by-being-
        # unconditional, same as MERGE_DEBUG above; not gated behind a
        # separate parameter. Intended to be read from a future live
        # capture to validate/retune the reasoned-starting-point defaults
        # above from real numbers.
        self._gate_stats = {
            'outlier_points_removed_total': 0,
            'residual_too_small_for_second_plane': 0,
            'second_plane_rejected_low_inlier_ratio': 0,
            'second_plane_rejected_fragment_shaped': 0,
            'second_plane_accepted': 0,
        }

        self.get_logger().info('wall_detector_node started')

    # ------------------------------------------------------------------
    def _maybe_reorient(self, pts: np.ndarray) -> np.ndarray:
        """Reorient raw points into z-up/x-forward if they arrive in the
        ZED optical convention (z-forward, x-right, y-down). NOT called from
        the default cloud_callback path -- see module docstring's "Frame
        handling" section: _transform_to_robot_frame() (tf2, below) handles
        both axis convention AND the real base_link origin offset, which this
        method alone cannot (it has no notion of translation). Left in place
        for anyone bench-testing against a cloud from a source that isn't
        TF-connected to base_link at all, where a manual axis-only swap is
        the only option and 'zed_optical' input_frame_convention would apply."""
        if self.frame_convention == 'zed_optical':
            x, y, z = pts[:, 0], pts[:, 1], pts[:, 2]
            forward = z
            right = x
            up = -y
            return np.stack([forward, -right, up], axis=1)
        return pts

    def _transform_to_robot_frame(self, pts: np.ndarray, header: Header):
        """Transform raw points from the cloud's own frame (header.frame_id,
        normally zed2_left_camera_frame) into self.robot_frame (base_link)
        via a real tf2 lookup -- see module docstring's "Frame handling"
        section for why this replaces a hardcoded axis-only assumption.
        Applied as a vectorized rotation+translation directly on the Nx3
        array (rather than routing through tf2_sensor_msgs.do_transform_cloud's
        full PointCloud2 round-trip) since cloud_callback already extracts
        this array anyway, and this is a per-frame cost on a real-time
        perception node. Returns None (skip this frame, already logged) if
        the transform isn't available yet -- mirrors obstacle_projector_node's
        own tf2-failure handling.
        """
        if header.frame_id == self.robot_frame:
            return pts

        try:
            transform = self.tf_buffer.lookup_transform(
                self.robot_frame, header.frame_id, rclpy.time.Time())
        except (LookupException, ConnectivityException, ExtrapolationException) as exc:
            self.get_logger().warn(
                f'tf2 lookup "{header.frame_id}" -> "{self.robot_frame}" failed: '
                f'{exc} -- skipping frame', throttle_duration_sec=5.0)
            return None

        t = transform.transform.translation
        q = transform.transform.rotation
        translation = np.array([t.x, t.y, t.z])
        rotation_matrix = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
        return pts @ rotation_matrix.T + translation

    def _roi_filter(self, pts: np.ndarray) -> np.ndarray:
        mask = (
            (pts[:, 0] > self.roi_x_min) & (pts[:, 0] < self.roi_x_max) &
            (np.abs(pts[:, 1]) < self.roi_y_half) &
            (pts[:, 2] > self.roi_z_min) & (pts[:, 2] < self.roi_z_max) &
            np.isfinite(pts).all(axis=1)
        )
        return pts[mask]

    # ------------------------------------------------------------------
    def _odom_cb(self, msg: Odometry):
        """Just records the latest odometry pose -- cheap, fires at
        odometry's own (much higher than point-cloud) rate. See module
        docstring's "Motion compensation" section; _consume_ego_delta()
        below is where this actually gets turned into a per-tick delta."""
        q = msg.pose.pose.orientation
        # Direct yaw-from-quaternion (not Rotation.as_euler, whose axis
        # convention would be ambiguous if roll/pitch aren't exactly zero
        # due to sensor noise) -- standard planar-yaw extraction formula.
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self._latest_odom_pose = (
            msg.pose.pose.position.x, msg.pose.pose.position.y, yaw, stamp)

    def _consume_ego_delta(self):
        """Called once per cloud_callback tick (i.e. once per WallTracker.
        update() call) -- returns the ego-motion delta since the LAST time
        this was called (see _odom_delta), or None if odometry is
        unavailable/stale/this is the first tick with odometry available.
        Advances self._last_tracker_odom_pose as a side effect (this IS the
        "since the tracker's last update tick" bookkeeping) -- must be
        called at most once per frame. See module docstring."""
        if self._latest_odom_pose is None:
            self.get_logger().warn(
                f'no odometry received yet on {self.odom_topic} -- motion '
                'compensation disabled this frame', throttle_duration_sec=5.0)
            return None

        x, y, yaw, stamp = self._latest_odom_pose
        now_sec = self.get_clock().now().nanoseconds * 1e-9
        if (now_sec - stamp) > _ODOM_STALE_SEC:
            self.get_logger().warn(
                f'odometry stale ({now_sec - stamp:.2f}s old, topic='
                f'{self.odom_topic}) -- motion compensation disabled this frame',
                throttle_duration_sec=5.0)
            # Force a fresh baseline once odometry recovers, rather than
            # computing a delta across the stale gap (which would try to
            # compensate for however long odometry was actually missing,
            # not "since the last real tick").
            self._last_tracker_odom_pose = None
            return None

        curr_pose = (x, y, yaw)
        ego_delta = None
        if self._last_tracker_odom_pose is not None:
            ego_delta = _odom_delta(self._last_tracker_odom_pose, curr_pose)
        self._last_tracker_odom_pose = curr_pose
        return ego_delta

    # ------------------------------------------------------------------
    def cloud_callback(self, msg: PointCloud2):
        pts = point_cloud2.read_points_numpy(msg, field_names=('x', 'y', 'z'), skip_nans=True)
        if pts.size == 0:
            return

        pts = self._transform_to_robot_frame(pts.astype(np.float64), msg.header)
        if pts is None:
            return  # tf2 lookup failed this frame -- already logged, throttled

        pts = self._roi_filter(pts)
        if pts.shape[0] < self.min_inliers:
            # Zero candidate walls this frame -- routed through the SAME
            # tracker.update()/publish() path as a normal frame (not a
            # separate "publish empty" special case) so existing tracks
            # correctly hold-then-drop through a transient no-data frame
            # instead of vanishing (or lingering forever) the instant one
            # bad frame happens. See WallTracker's own docstring.
            walls = []
        else:
            pc = o3d.geometry.PointCloud()
            pc.points = o3d.utility.Vector3dVector(pts)
            if self.voxel_size > 0:
                pc = pc.voxel_down_sample(self.voxel_size)

            # ==== Part A: statistical outlier removal, before ANY
            # segment_plane call -- see module docstring's "Raw candidate
            # quality gating" section. Guarded on having more points than
            # nb_neighbors (Open3D needs at least that many to evaluate each
            # point's neighborhood) -- a cloud this sparse is already headed
            # for the min_inliers break in _find_walls's very first
            # iteration regardless, so skipping outlier removal here changes
            # nothing about the eventual outcome.
            if self.outlier_nb_neighbors > 0 and len(pc.points) > self.outlier_nb_neighbors:
                pre_count = len(pc.points)
                pc, _ = pc.remove_statistical_outlier(
                    nb_neighbors=self.outlier_nb_neighbors,
                    std_ratio=self.outlier_std_ratio)
                removed = pre_count - len(pc.points)
                self._gate_stats['outlier_points_removed_total'] += removed
                self.get_logger().info(
                    f'[GATE_DEBUG] statistical outlier removal: {pre_count} -> '
                    f'{len(pc.points)} points (removed={removed}, '
                    f'nb_neighbors={self.outlier_nb_neighbors} '
                    f'std_ratio={self.outlier_std_ratio}) | cumulative removed='
                    f'{self._gate_stats["outlier_points_removed_total"]}',
                    throttle_duration_sec=1.0)

            walls = self._find_walls(pc)

        # ==== TEMP DEBUG (fragmentation-diagnosis pass, remove after) ====
        def _track_debug_log(msg):
            self.get_logger().info(f'[TRACK_DEBUG] {msg}')
        # ==== END TEMP DEBUG ====
        ego_delta = self._consume_ego_delta()
        tracked_walls = self.tracker.update(
            walls, ego_delta=ego_delta, debug_log=_track_debug_log)
        self._publish(msg.header, tracked_walls)

    # ------------------------------------------------------------------
    def _find_walls(self, pc: o3d.geometry.PointCloud):
        """Iterative RANSAC: find up to max_walls near-vertical planes.
        Removing each plane's inliers before the next search is what lets
        a second, distinct wall (e.g. the other side of a corner) surface
        instead of being re-found.

        Deduplicates the RAW candidate list (merge_duplicate_walls) BEFORE
        is_corner tagging -- see module docstring's "Plane merge + tracking"
        section for why this order matters. Tracking/EMA is a separate,
        later stage (cloud_callback calls self.tracker.update() on this
        method's return value) -- deliberately not done here."""
        walls = []
        remaining = pc
        log = self.get_logger().info

        for idx in range(self.max_walls):
            residual_size_before = len(remaining.points)
            if residual_size_before < self.min_inliers:
                break

            # ==== Part A gate 1: residual-size floor, SECOND plane only --
            # see module docstring. The dominant/first plane (idx == 0)
            # doesn't need this; the min_inliers check above already gates
            # it.
            if idx > 0 and not _residual_supports_second_plane(
                    residual_size_before, self.min_inliers,
                    self.second_plane_min_residual_ratio):
                self._gate_stats['residual_too_small_for_second_plane'] += 1
                log(f'[GATE_DEBUG] second-plane search skipped: residual='
                    f'{residual_size_before} pts < '
                    f'{self.second_plane_min_residual_ratio}x min_inliers='
                    f'{self.min_inliers} | cumulative='
                    f'{self._gate_stats["residual_too_small_for_second_plane"]}')
                break

            plane_model, inlier_idx = remaining.segment_plane(
                distance_threshold=self.dist_thresh,
                ransac_n=self.ransac_n,
                num_iterations=self.ransac_iters,
            )
            if len(inlier_idx) < self.min_inliers:
                break

            # ==== Part A gate 2: inlier-support ratio, SECOND plane only.
            if idx > 0:
                ratio_ok, inlier_ratio = _second_plane_inlier_ratio_ok(
                    len(inlier_idx), residual_size_before,
                    self.second_plane_min_inlier_ratio)
                if not ratio_ok:
                    self._gate_stats['second_plane_rejected_low_inlier_ratio'] += 1
                    log(f'[GATE_DEBUG] second-plane candidate REJECTED: '
                        f'inlier_ratio={inlier_ratio:.4f} < '
                        f'{self.second_plane_min_inlier_ratio:.4f} '
                        f'(inliers={len(inlier_idx)}/{residual_size_before}) | '
                        f'cumulative='
                        f'{self._gate_stats["second_plane_rejected_low_inlier_ratio"]}')
                    remaining = remaining.select_by_index(inlier_idx, invert=True)
                    continue

            a, b, c, d = plane_model
            normal = np.array([a, b, c])
            normal = normal / np.linalg.norm(normal)

            # Verticality check: a wall's normal should be roughly horizontal
            # (small z component). Angle from horizontal plane:
            angle_from_horizontal = abs(math.asin(np.clip(normal[2], -1.0, 1.0)))
            inlier_cloud = remaining.select_by_index(inlier_idx)
            remaining = remaining.select_by_index(inlier_idx, invert=True)

            if angle_from_horizontal > self.verticality_max:
                # Likely floor/ceiling, not a wall -- discard and keep looking
                continue

            # Orient normal to point back toward the robot (origin)
            centroid = np.asarray(inlier_cloud.get_center())
            if np.dot(normal, -centroid) < 0:
                normal = -normal
                d = -d

            pts_arr = np.asarray(inlier_cloud.points)

            # ==== Part A gate 3: in-plane compactness, SECOND plane only.
            if idx > 0:
                compact_ok, aspect_ratio = _second_plane_compact_ok(
                    pts_arr, normal, centroid, self.second_plane_max_aspect_ratio)
                if not compact_ok:
                    self._gate_stats['second_plane_rejected_fragment_shaped'] += 1
                    log(f'[GATE_DEBUG] second-plane candidate REJECTED: '
                        f'in-plane aspect_ratio={aspect_ratio:.2f} > '
                        f'{self.second_plane_max_aspect_ratio:.2f} '
                        f'(fragment/sliver-shaped, not a coherent patch) | '
                        f'cumulative='
                        f'{self._gate_stats["second_plane_rejected_fragment_shaped"]}')
                    continue
                self._gate_stats['second_plane_accepted'] += 1
                log(f'[GATE_DEBUG] second-plane candidate ACCEPTED: '
                    f'inlier_ratio={inlier_ratio:.4f} aspect_ratio={aspect_ratio:.2f} | '
                    f'cumulative accepted={self._gate_stats["second_plane_accepted"]}')

            distance = abs(d) / np.linalg.norm([a, b, c])
            bearing = math.atan2(centroid[1], centroid[0])
            # bearing = direction TO the wall centroid relative to robot forward (+x), + = left

            extent = pts_arr.max(axis=0) - pts_arr.min(axis=0)

            walls.append({
                'distance': float(distance),
                'bearing': float(bearing),
                'normal': normal,
                'centroid': centroid,
                'extent': extent,
                'points': pts_arr,
                # (a, b, c) == normal (already unit, post-reorientation) and
                # d == distance exactly -- see the module-level note above
                # merge_duplicate_walls for the derivation. Needed by
                # _are_duplicate_planes/_offset_gap, not consumed elsewhere.
                'plane': (float(normal[0]), float(normal[1]), float(normal[2]), float(distance)),
            })

        # ==== TEMP DEBUG (merge-stage instrumentation pass, remove after) ====
        # Point 1: raw candidate list, before any merge/corner logic runs.
        # (log already bound to self.get_logger().info above.)
        self._merge_debug_frame_count += 1
        log(f'[MERGE_DEBUG] frame={self._merge_debug_frame_count}: '
            f'{len(walls)} raw candidate plane(s) before merge')
        for i, w in enumerate(walls):
            n = w['normal']
            c = w['centroid']
            log(f'[MERGE_DEBUG]   raw[{i}]: normal=({n[0]:.5f}, {n[1]:.5f}, {n[2]:.5f}) '
                f'centroid=({c[0]:.4f}, {c[1]:.4f}, {c[2]:.4f}) '
                f'distance={w["distance"]:.4f} bearing_deg={math.degrees(w["bearing"]):.2f}')

        # Point 2: confirm merge_duplicate_walls is actually about to be
        # invoked this frame, and log the THRESHOLD VALUES AS THE NODE SEES
        # THEM RIGHT NOW via a fresh get_parameter() call -- not the yaml
        # file's value, not even self.merge_normal_cos_thresh/
        # self.merge_distance_thresh_m (those are read once at __init__ and
        # cached; this re-reads live to catch any discrepancy between the
        # two, however unlikely).
        live_normal_thresh = self.get_parameter('merge_normal_cos_thresh').value
        live_distance_thresh = self.get_parameter('merge_distance_thresh_m').value
        log(f'[MERGE_DEBUG] about to call merge_duplicate_walls() -- '
            f'live get_parameter(merge_normal_cos_thresh)={live_normal_thresh!r} '
            f'live get_parameter(merge_distance_thresh_m)={live_distance_thresh!r} | '
            f'cached self.merge_normal_cos_thresh={self.merge_normal_cos_thresh!r} '
            f'cached self.merge_distance_thresh_m={self.merge_distance_thresh_m!r}')

        def _merge_debug_log(msg):
            log(f'[MERGE_DEBUG]   {msg}')
        # ==== END TEMP DEBUG (point 1/2) ====

        walls = merge_duplicate_walls(
            walls, self.merge_normal_cos_thresh, self.merge_distance_thresh_m,
            debug_log=_merge_debug_log)

        log(f'[MERGE_DEBUG] frame={self._merge_debug_frame_count}: '
            f'{len(walls)} wall(s) after merge')

        # Corner tagging: if two (post-merge) walls remain and their normals
        # are close to perpendicular, mark both as part of a corner pair.
        # Runs on the DEDUPLICATED list -- a real wall that got split into
        # two near-parallel candidates would otherwise occasionally look
        # like a spurious near-90-degree pair depending on the split's own
        # noise, which is exactly the kind of bug this merge step exists to
        # prevent from ever reaching corner detection at all.
        is_corner = False
        if len(walls) == 2:
            cos_angle = abs(np.dot(walls[0]['normal'], walls[1]['normal']))
            angle = math.acos(np.clip(cos_angle, 0.0, 1.0))
            if abs(angle - math.pi / 2) < self.corner_perp_tol:
                is_corner = True
            # ==== TEMP DEBUG (point 5, remove after) ====
            angle_gap_deg = math.degrees(abs(angle - math.pi / 2))
            log(f'[MERGE_DEBUG] is_corner check: angle_between_normals_deg='
                f'{math.degrees(angle):.2f} |angle-90|_deg={angle_gap_deg:.2f} '
                f'vs corner_perp_tol_deg={math.degrees(self.corner_perp_tol):.2f} '
                f'live get_parameter(corner_perp_tolerance_deg)='
                f'{self.get_parameter("corner_perp_tolerance_deg").value!r} '
                f'-> is_corner={is_corner}')
            # ==== END TEMP DEBUG ====

        for w in walls:
            w['is_corner'] = is_corner

        # ==== TEMP DEBUG (point 5, remove after) ====
        if is_corner:
            log(f'[MERGE_DEBUG] TAGGED [corner]: wall[0] normal='
                f'{tuple(round(x, 4) for x in walls[0]["normal"])} vs wall[1] normal='
                f'{tuple(round(x, 4) for x in walls[1]["normal"])}')
        # ==== END TEMP DEBUG ====

        return walls

    # ------------------------------------------------------------------
    def _publish(self, header: Header, tracked_walls):
        """tracked_walls: this frame's live TrackedWall list from
        self.tracker.update() -- every consumer here (WallDetection,
        front_clearance, the Foxglove markers) reads the TRACKED/SMOOTHED
        fields, not raw per-frame geometry -- see module docstring."""
        wall_array = WallArray()
        wall_array.header = header
        wall_array.header.frame_id = self.robot_frame

        marker_array = MarkerArray()
        front_candidates = []
        eligible_front_tracks = []

        for w in tracked_walls:
            wd = WallDetection()
            wd.distance = w.distance
            wd.bearing = w.bearing
            wd.normal = Vector3(x=w.normal[0], y=w.normal[1], z=w.normal[2])
            wd.centroid = Point(x=w.centroid[0], y=w.centroid[1], z=w.centroid[2])
            wd.width = float(max(w.extent[0], w.extent[1]))
            wd.is_corner = w.is_corner
            wall_array.walls.append(wd)

            # Part B: front-facing cone AND stability-bar eligibility -- see
            # module docstring's "front_clearance selection eligibility"
            # section. Note this deliberately does NOT gate wall_array/
            # marker publication above -- a short-lived/low-support track
            # still gets reported/rendered normally, it just doesn't count
            # toward the safety-relevant front_clearance number yet.
            eligible = _front_clearance_eligible(
                w, self.front_facing_max, self.front_clearance_min_track_frames,
                self.front_clearance_min_inliers)
            # ==== TEMP DEBUG (front_clearance eligibility diagnosis, remove
            # after -- see module docstring) ====
            self.get_logger().info(
                f'[GATE_DEBUG] track_id={w.track_id} distance={w.distance:.3f} '
                f'bearing_deg={math.degrees(w.bearing):.2f} '
                f'frames_matched={w.frames_matched} (need >='
                f'{self.front_clearance_min_track_frames}) '
                f'n_inliers={w.n_inliers} (need >={self.front_clearance_min_inliers}) '
                f'-> front_clearance_eligible={eligible}',
                throttle_duration_sec=1.0)
            # ==== END TEMP DEBUG ====
            if eligible:
                front_candidates.append(w.distance)
                eligible_front_tracks.append(w)

            marker_array.markers.extend(self._wall_markers(header, w))

        # Explicit per-track DELETE for any track_id that was rendered last
        # cycle but isn't part of this one (dropped by the tracker, or the
        # count just shrank) -- an empty/smaller MarkerArray does NOT clear
        # markers a client already has for an id it isn't seeing again this
        # message; only an explicit DELETE (or DELETEALL) does. Using each
        # TrackedWall's own stable track_id (not list position) as the
        # marker id is what makes this correct across a track's whole
        # lifetime, unlike the previous per-frame `idx`, which would relabel
        # a still-alive wall's markers to a different id if the ordering out
        # of segment_plane happened to change between frames (real markers
        # get correctly replaced-in-place across frames only if id is
        # stable) -- and which also had no way to know one specific id
        # needed clearing when the list shrank rather than went to zero.
        current_ids = {w.track_id for w in tracked_walls}
        for stale_id in self._last_published_track_ids - current_ids:
            for ns in ('wall_plane', 'wall_normal', 'wall_label'):
                marker_array.markers.append(
                    Marker(header=header, ns=ns, id=stale_id, action=Marker.DELETE))
        self._last_published_track_ids = current_ids

        self.wall_pub.publish(wall_array)

        # PRE-EXISTING BUG, fixed here (found live while diagnosing the
        # front_clearance eligibility gate above): `clearance` already
        # falls back to self._last_clearance when front_candidates is
        # empty this frame, but the OLD line below threw that fallback
        # away and published math.inf regardless -- so _last_clearance's
        # own bookkeeping was write-only, never actually reaching the
        # topic. Harmless while front_candidates was rarely empty (no
        # eligibility gate existed yet); with the front_clearance
        # selection eligibility gate above, a single momentarily-
        # ineligible frame would otherwise reset the published value to
        # infinity even though a perfectly good last-known distance was
        # sitting right there in `clearance`.
        clearance = min(front_candidates) if front_candidates else self._last_clearance
        self._last_clearance = clearance
        self.clearance_pub.publish(Float32(data=float(clearance)))

        # Hard boundary constraint (see module docstring's "Hard boundary
        # constraints" + "Track-identity stickiness" sections) -- selected
        # from the SAME eligible-track list front_clearance's own min()
        # selection draws from (eligible_front_tracks is exactly
        # front_candidates' own source list), but via _select_boundary_track
        # rather than a fresh min() here: front_clearance is a scalar
        # distance (a momentary flip between two close eligible tracks only
        # changes a number MPC's own stop-condition already treats
        # continuously); front_wall_boundary is a HARD constraint fed
        # straight into the solver, where the same flip is a discontinuous
        # jump in the constraint geometry itself -- worth the extra
        # continuity logic here specifically. Empty array (not a stale/
        # garbage one) when no front-facing track is currently eligible.
        boundary_array = BoundaryConstraintArray()
        boundary_array.header = header
        boundary_array.header.frame_id = self.robot_frame
        if eligible_front_tracks:
            winner = _select_boundary_track(eligible_front_tracks, self._last_boundary_track_id)
            self._last_boundary_track_id = winner.track_id
            nx, ny, offset = _wall_boundary_from_track(winner)
            boundary_array.constraints.append(
                BoundaryConstraint(normal=[nx, ny], offset=offset))
        else:
            self._last_boundary_track_id = None
        self.front_wall_boundary_pub.publish(boundary_array)

        self.marker_pub.publish(marker_array)

    # ------------------------------------------------------------------
    def _wall_markers(self, header: Header, w: 'TrackedWall'):
        markers = []
        idx = w.track_id
        color = (1.0, 0.2, 0.2) if not w.is_corner else (1.0, 0.6, 0.0)

        # Plane patch: draw as a flat quad using the inlier bounding extent,
        # centered on the centroid, oriented along the plane's dominant axes.
        #
        # FIX (found via live Foxglove testing, not in the originally given
        # code): the previous version took min/max of the inlier points'
        # raw X/Y and held Z fixed at the centroid -- i.e. it always drew a
        # HORIZONTAL quad, regardless of the plane RANSAC actually found.
        # For anything that passed verticality_max_deg (a wall, by
        # definition near-vertical -- its NORMAL is the near-horizontal
        # vector, not the plane itself), that rendered as a flat patch lying
        # parallel to the ground and coplanar with the normal arrow, instead
        # of a vertical rectangle roughly perpendicular to it. Fixed by
        # building an orthonormal in-plane basis (an "along the wall"
        # direction and an "up the wall" direction, both perpendicular to
        # the real normal) and projecting the inlier points onto THAT basis
        # for the quad's extent, so the quad actually lies in the detected
        # plane at whatever orientation that plane really has.
        #
        # centroid/normal here are the TRACKED (EMA-smoothed) values -- the
        # quad's POSITION/ORIENTATION are therefore stable frame to frame;
        # only pts_arr (the last-MATCHED frame's raw inlier points, held not
        # smoothed -- see TrackedWall's own docstring) still varies, so the
        # quad's exact SIZE can still show minor frame-to-frame variation
        # even though where/how it's oriented no longer jitters.
        pts_arr = w.points
        centroid = w.centroid
        normal = w.normal

        world_up = np.array([0.0, 0.0, 1.0])
        if abs(np.dot(normal, world_up)) > 0.99:
            # Only reachable if normal is near-vertical, which verticality_max_deg
            # should already have filtered out upstream -- guarded anyway so this
            # never divides by a near-zero cross product.
            world_up = np.array([1.0, 0.0, 0.0])
        along = np.cross(world_up, normal)
        along = along / np.linalg.norm(along)
        up = np.cross(normal, along)
        up = up / np.linalg.norm(up)

        rel = pts_arr - centroid
        proj_along = rel @ along
        proj_up = rel @ up
        a_min, a_max = proj_along.min(), proj_along.max()
        u_min, u_max = proj_up.min(), proj_up.max()

        def _corner(a, u):
            v = centroid + a * along + u * up
            return Point(x=float(v[0]), y=float(v[1]), z=float(v[2]))

        p1 = _corner(a_min, u_min)
        p2 = _corner(a_max, u_min)
        p3 = _corner(a_max, u_max)
        p4 = _corner(a_min, u_max)

        quad = Marker()
        quad.header = header
        quad.header.frame_id = self.robot_frame
        quad.ns = 'wall_plane'
        quad.id = idx
        quad.type = Marker.TRIANGLE_LIST
        quad.action = Marker.ADD
        quad.scale = Vector3(x=1.0, y=1.0, z=1.0)
        quad.color.r, quad.color.g, quad.color.b, quad.color.a = (*color, 0.4)
        quad.points = [p1, p2, p3, p1, p3, p4]
        markers.append(quad)

        # Normal direction arrow, length scaled by distance for quick reading
        arrow = Marker()
        arrow.header = header
        arrow.header.frame_id = self.robot_frame
        arrow.ns = 'wall_normal'
        arrow.id = idx
        arrow.type = Marker.ARROW
        arrow.action = Marker.ADD
        arrow.scale = Vector3(x=0.05, y=0.1, z=0.1)
        arrow.color.r, arrow.color.g, arrow.color.b, arrow.color.a = (*color, 0.9)
        start = Point(x=centroid[0], y=centroid[1], z=centroid[2])
        tip = Point(
            x=centroid[0] + normal[0] * 0.5,
            y=centroid[1] + normal[1] * 0.5,
            z=centroid[2] + normal[2] * 0.5,
        )
        arrow.points = [start, tip]
        markers.append(arrow)

        # Distance label
        text = Marker()
        text.header = header
        text.header.frame_id = self.robot_frame
        text.ns = 'wall_label'
        text.id = idx
        text.type = Marker.TEXT_VIEW_FACING
        text.action = Marker.ADD
        text.scale.z = 0.2
        text.color.r, text.color.g, text.color.b, text.color.a = (1.0, 1.0, 1.0, 1.0)
        text.pose.position = Point(x=centroid[0], y=centroid[1], z=centroid[2] + 0.3)
        label = f"wall {idx}: {w.distance:.2f} m, {math.degrees(w.bearing):.0f} deg"
        if w.is_corner:
            label += " [corner]"
        text.text = label
        markers.append(text)

        return markers


def main(args=None):
    rclpy.init(args=args)
    node = WallDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
