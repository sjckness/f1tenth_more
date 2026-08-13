"""First real logic tests in f1tenth_perception (previously only the stock
ament linter boilerplate -- and even that was never actually wired up for
this package, see setup.cfg). Added by the wall_detector_node duplicate-
merge + tracking/EMA-smoothing fix, mirroring mpc_controller/test/
test_solver_rti.py's own "no prior synthetic-scenario test infrastructure
existed, so these are constructed fresh here" precedent.

Two independent problems, two independent test groups, deliberately not
conflated (same rule the implementation itself follows -- see
wall_detector_node.py's module docstring):

  1. TestMergeDuplicateWalls -- merge_duplicate_walls() pure geometry: a
     coplanar pair merges into one, a ~90 deg pair stays separate, a 3-way
     split all collapses into one.
  2. TestTrackedWallEMA / TestWallTracker -- TrackedWall.update()'s EMA math
     (bearing wraparound, unit-normal renormalization) and WallTracker's
     spawn/match/hold/drop lifecycle.
  3. TestCornerIndependence -- the two fixes' interaction at the one place
     they actually touch: a near-perpendicular corner pair must neither
     merge (group 1) nor cross-contaminate each other's independent EMA
     state (group 2) once tracked.

Static/synthetic only, no live hardware, no rclpy node instantiation
required -- every symbol under test here is a plain function/class with zero
rclpy dependency (see wall_detector_node.py's own "pure logic separate from
ROS glue" module comments), so this imports the whole node module (which
does still import rclpy/open3d/tf2_ros at module scope) but never
constructs a Node.

Run standalone: python3 -m pytest test/test_wall_detector.py -v
"""

import math

import numpy as np
import pytest

from f1tenth_perception.wall_detector_node import (
    TrackedWall,
    WallTracker,
    _front_clearance_eligible,
    _odom_delta,
    _predict_track_position,
    _residual_supports_second_plane,
    _second_plane_compact_ok,
    _second_plane_inlier_ratio_ok,
    _select_boundary_track,
    _wall_boundary_from_track,
    _weaker_track,
    merge_duplicate_walls,
    prune_inconsistent_tracks,
)


# ==============================================================================
# Shared helpers -- build synthetic wall dicts with the exact shape
# merge_duplicate_walls/_wall_from_points/_find_walls all produce (distance,
# bearing, normal, centroid, extent, points, plane), from a plane definition
# and a small synthetic point patch, so tests exercise the real geometry
# (offset_gap/plane-distance math) rather than hand-waving pre-built dicts.
# ==============================================================================

def _make_wall(normal, centroid, n_points=200, jitter=0.002, seed=0):
    """A synthetic near-planar point patch: `n_points` samples scattered in
    the plane through `centroid` with the given unit `normal` (direction
    HINT only -- see reorientation below), offset by +-`jitter` along the
    normal (so refits are well-defined, not perfectly degenerate), then
    packaged into the same dict shape _find_walls/_wall_from_points produce.

    `distance` is deliberately NOT a caller-supplied param (an earlier
    version of this helper took one and built `plane` from it directly,
    which silently desynced from the actual normal/centroid geometry and
    broke _offset_gap in a way that looked like an implementation bug but
    wasn't). It's always DERIVED here, mirroring _wall_from_points' own
    "orient normal toward the robot origin" step exactly: after
    reorientation, normal.centroid < 0 always, so d = -normal.centroid > 0,
    and d == distance (both already the same, non-negative scale) -- the
    exact invariant _offset_gap relies on.
    """
    rng = np.random.default_rng(seed)
    normal = np.asarray(normal, dtype=float)
    normal = normal / np.linalg.norm(normal)
    centroid = np.asarray(centroid, dtype=float)
    # Same reorientation _wall_from_points uses: normal must end up pointing
    # back toward the robot origin (normal.centroid < 0).
    if np.dot(normal, centroid) > 0:
        normal = -normal
    plane_d = -float(np.dot(normal, centroid))
    # Two orthonormal in-plane basis vectors.
    arbitrary = np.array([1.0, 0.0, 0.0]) if abs(normal[0]) < 0.9 else np.array([0.0, 1.0, 0.0])
    u = np.cross(normal, arbitrary)
    u = u / np.linalg.norm(u)
    v = np.cross(normal, u)
    in_plane = rng.uniform(-0.5, 0.5, size=(n_points, 2))
    normal_noise = rng.uniform(-jitter, jitter, size=(n_points, 1))
    points = (
        centroid
        + in_plane[:, 0:1] * u
        + in_plane[:, 1:2] * v
        + normal_noise * normal
    )
    bearing = math.atan2(centroid[1], centroid[0])
    extent = points.max(axis=0) - points.min(axis=0)
    return {
        'distance': float(plane_d),
        'bearing': float(bearing),
        'normal': normal,
        'centroid': centroid,
        'extent': extent,
        'points': points,
        'plane': (float(normal[0]), float(normal[1]), float(normal[2]), float(plane_d)),
        'is_corner': False,
    }


# Same thresholds stack_params.yaml ships as defaults -- tests are written
# against these specific numbers deliberately (per the task's own request),
# not arbitrary looser ones. Loosened 0.97->0.94 / 0.08->0.15 (then ->2.0/5
# for the two new ambiguous-zone params below) after a live 566-frame debug
# capture -- see wall_detector_node.py's own module docstring for the full
# evidence writeup.
NORMAL_COS_THRESH = 0.94
DISTANCE_THRESH_M = 0.15


class TestMergeDuplicateWalls:

    def test_coplanar_pair_merges_into_one(self):
        # Two near-identical detections of the same physical wall: same
        # normal, centroids ~3cm apart (well under the 0.08m offset-gap
        # threshold) -- the exact "iterative re-segmentation splits one
        # noisy surface" scenario from the task's own context.
        wall_a = _make_wall(normal=[1.0, 0.0, 0.0], centroid=[2.0, 0.5, 0.5], seed=1)
        wall_b = _make_wall(normal=[1.0, 0.0, 0.0], centroid=[2.03, 0.52, 0.48], seed=2)
        merged = merge_duplicate_walls([wall_a, wall_b], NORMAL_COS_THRESH, DISTANCE_THRESH_M)
        assert len(merged) == 1
        # The refit result should land close to the true shared plane.
        # normal=[1,0,0] is a direction HINT into _make_wall -- since the
        # centroid sits at positive x, the reorientation step (mirrored by
        # both _make_wall and the real _wall_from_points) flips it to point
        # back toward the origin, i.e. [-1,0,0].
        assert merged[0]['normal'] == pytest.approx([-1.0, 0.0, 0.0], abs=0.02)
        assert merged[0]['distance'] == pytest.approx(2.0, abs=0.05)
        # Pooled points: both inputs' point counts, not either one alone.
        assert len(merged[0]['points']) == len(wall_a['points']) + len(wall_b['points'])

    def test_perpendicular_pair_stays_separate(self):
        # A genuine corner: two walls ~90 deg apart. Must NOT merge even
        # though nothing constrains their offset_gap -- the normal_similarity
        # check alone must already fail this pair (dot ~ 0, nowhere near
        # 0.97), confirming corner detection is never at risk from the merge
        # step (see wall_detector_node.py's own claim about this).
        wall_a = _make_wall(normal=[1.0, 0.0, 0.0], centroid=[2.0, 0.0, 0.5], seed=3)
        wall_b = _make_wall(normal=[0.0, 1.0, 0.0], centroid=[1.0, 1.5, 0.5], seed=4)
        merged = merge_duplicate_walls([wall_a, wall_b], NORMAL_COS_THRESH, DISTANCE_THRESH_M)
        assert len(merged) == 2

    def test_three_way_split_all_merges_into_one(self):
        # Three overlapping candidates for one physical wall (a worse split
        # than the 2-way case) -- union-find must collapse ALL THREE into a
        # single merged wall, not just the closest adjacent pair, confirming
        # this isn't accidentally an adjacent-pairs-only check.
        wall_a = _make_wall(normal=[0.0, 1.0, 0.0], centroid=[1.0, 1.0, 0.5], seed=5)
        wall_b = _make_wall(normal=[0.02, 0.9998, 0.0], centroid=[1.0, 1.02, 0.45], seed=6)
        wall_c = _make_wall(normal=[-0.01, 0.9999, 0.0], centroid=[1.0, 0.98, 0.55], seed=7)
        merged = merge_duplicate_walls(
            [wall_a, wall_b, wall_c], NORMAL_COS_THRESH, DISTANCE_THRESH_M)
        assert len(merged) == 1
        total_points = len(wall_a['points']) + len(wall_b['points']) + len(wall_c['points'])
        assert len(merged[0]['points']) == total_points

    def test_loosened_thresholds_merge_a_pair_the_old_thresholds_would_reject(self):
        # Live-debug-capture-shaped case: normal_similarity=0.96 (median of
        # the real failure cluster was 0.960), offset_gap~0.098m (median of
        # that cluster was 0.103m) -- both would have FAILED the original
        # 0.97/0.08m thresholds (this exact pair is constructed to fail
        # them) but must merge under the new 0.94/0.15m ones, confirming the
        # loosening actually does something, not just a number bump.
        # wall_b's hint normal is given already-final (not a direction hint
        # that gets flipped) -- picked so dot(hint, centroid) <= 0, the same
        # "no flip needed" condition _make_wall's own reorientation checks,
        # so normal_similarity against wall_a's normal comes out to exactly
        # 0.96 (both are already unit vectors: 0.96^2+0.28^2 == 1.0).
        wall_a = _make_wall(normal=[1.0, 0.0, 0.0], centroid=[2.0, 0.0, 0.5], seed=8)
        wall_b = _make_wall(normal=[-0.96, 0.28, 0.0], centroid=[2.10, 0.0, 0.5], seed=9)
        normal_similarity = float(np.dot(wall_a['normal'], wall_b['normal']))
        assert normal_similarity == pytest.approx(0.96), (
            f'test fixture bug: normal_similarity={normal_similarity}, expected 0.96')
        old_thresholds_merged = merge_duplicate_walls([wall_a, wall_b], 0.97, 0.08)
        assert len(old_thresholds_merged) == 2, 'fixture should fail the OLD thresholds'
        new_thresholds_merged = merge_duplicate_walls(
            [wall_a, wall_b], NORMAL_COS_THRESH, DISTANCE_THRESH_M)
        assert len(new_thresholds_merged) == 1, 'fixture should merge under the NEW thresholds'

    def test_single_wall_passes_through_unchanged(self):
        wall_a = _make_wall(normal=[1.0, 0.0, 0.0], centroid=[2.0, 0.0, 0.5])
        merged = merge_duplicate_walls([wall_a], NORMAL_COS_THRESH, DISTANCE_THRESH_M)
        assert len(merged) == 1
        assert merged[0] is wall_a

    def test_empty_list_passes_through(self):
        assert merge_duplicate_walls([], NORMAL_COS_THRESH, DISTANCE_THRESH_M) == []


# ==============================================================================
# TrackedWall.update() -- EMA math in isolation, no WallTracker involved.
# ==============================================================================

class TestTrackedWallEMA:

    def test_seed_has_no_smoothing(self):
        normal = np.array([1.0, 0.0, 0.0])
        centroid = np.array([2.0, 0.0, 0.5])
        tw = TrackedWall(
            track_id=0, distance=2.0, bearing=0.1, normal=normal, centroid=centroid,
            is_corner=False, points=np.zeros((1, 3)), extent=np.zeros(3))
        assert tw.distance == 2.0
        assert tw.bearing == pytest.approx(0.1)
        assert tw.normal == pytest.approx([1.0, 0.0, 0.0])

    def test_matched_wall_smooths_toward_new_value(self):
        tw = TrackedWall(
            track_id=0, distance=2.0, bearing=0.0, normal=[1.0, 0.0, 0.0],
            centroid=[2.0, 0.0, 0.0], is_corner=False, points=np.zeros((1, 3)),
            extent=np.zeros(3))
        alpha = 0.3
        tw.update(
            distance=3.0, bearing=0.0, normal=[1.0, 0.0, 0.0], centroid=[3.0, 0.0, 0.0],
            is_corner=False, points=np.zeros((1, 3)), extent=np.zeros(3), alpha=alpha)
        # smoothed = alpha*new + (1-alpha)*old = 0.3*3.0 + 0.7*2.0 = 2.3
        assert tw.distance == pytest.approx(2.3)
        # A second update should move it further toward (but not to) 3.0.
        tw.update(
            distance=3.0, bearing=0.0, normal=[1.0, 0.0, 0.0], centroid=[3.0, 0.0, 0.0],
            is_corner=False, points=np.zeros((1, 3)), extent=np.zeros(3), alpha=alpha)
        assert 2.3 < tw.distance < 3.0

    def test_bearing_wraparound_does_not_spike(self):
        # old bearing = +179deg, new bearing = -179deg -- a real angular
        # change of only 2deg (the short way around the +-pi seam), NOT the
        # ~358deg a naive raw lerp would compute.
        old_bearing = math.radians(179.0)
        new_bearing = math.radians(-179.0)
        tw = TrackedWall(
            track_id=0, distance=2.0, bearing=old_bearing, normal=[1.0, 0.0, 0.0],
            centroid=[2.0, 0.0, 0.0], is_corner=False, points=np.zeros((1, 3)),
            extent=np.zeros(3))
        alpha = 0.5
        tw.update(
            distance=2.0, bearing=new_bearing, normal=[1.0, 0.0, 0.0],
            centroid=[2.0, 0.0, 0.0], is_corner=False, points=np.zeros((1, 3)),
            extent=np.zeros(3), alpha=alpha)
        # Expected: halfway along the short 2deg arc from 179deg, i.e.
        # 180deg exactly (179 + 0.5*2) -- NOT anywhere near 0deg, which is
        # what a naive raw-value lerp (0.5*(-179)+0.5*179 = 0) would give.
        expected = math.radians(180.0)
        diff = abs(math.atan2(math.sin(tw.bearing - expected), math.cos(tw.bearing - expected)))
        assert diff < 1e-6

    def test_normal_stays_unit_length_after_every_update(self):
        rng = np.random.default_rng(42)
        tw = TrackedWall(
            track_id=0, distance=2.0, bearing=0.0, normal=[1.0, 0.0, 0.0],
            centroid=[2.0, 0.0, 0.0], is_corner=False, points=np.zeros((1, 3)),
            extent=np.zeros(3))
        for _ in range(20):
            random_normal = rng.uniform(-1, 1, size=3)
            random_normal /= np.linalg.norm(random_normal)
            tw.update(
                distance=2.0, bearing=0.0, normal=random_normal, centroid=[2.0, 0.0, 0.0],
                is_corner=False, points=np.zeros((1, 3)), extent=np.zeros(3), alpha=0.3)
            assert np.linalg.norm(tw.normal) == pytest.approx(1.0, abs=1e-9)

    def test_centroid_also_smooths(self):
        # Documented, deliberate addition beyond the task's explicit
        # distance/bearing/normal list -- see TrackedWall's own docstring.
        tw = TrackedWall(
            track_id=0, distance=2.0, bearing=0.0, normal=[1.0, 0.0, 0.0],
            centroid=[2.0, 0.0, 0.0], is_corner=False, points=np.zeros((1, 3)),
            extent=np.zeros(3))
        tw.update(
            distance=2.0, bearing=0.0, normal=[1.0, 0.0, 0.0], centroid=[4.0, 0.0, 0.0],
            is_corner=False, points=np.zeros((1, 3)), extent=np.zeros(3), alpha=0.5)
        assert tw.centroid == pytest.approx([3.0, 0.0, 0.0])


# ==============================================================================
# WallTracker -- association + spawn/hold/drop lifecycle.
# ==============================================================================

ASSOC_DISTANCE_THRESH_M = 0.4
ASSOC_BEARING_THRESH_RAD = math.radians(15.0)
HOLD_FRAMES = 5
ALPHA = 0.3
MIN_DISTINCT_SEPARATION_M = 2.0
AMBIGUOUS_CONFIRM_FRAMES = 5


def _make_tracker():
    return WallTracker(
        assoc_distance_thresh_m=ASSOC_DISTANCE_THRESH_M,
        assoc_bearing_thresh_rad=ASSOC_BEARING_THRESH_RAD,
        hold_frames=HOLD_FRAMES, alpha=ALPHA,
        merge_normal_cos_thresh=NORMAL_COS_THRESH,
        merge_distance_thresh_m=DISTANCE_THRESH_M,
        min_distinct_separation_m=MIN_DISTINCT_SEPARATION_M,
        ambiguous_confirm_frames=AMBIGUOUS_CONFIRM_FRAMES)


class TestWallTracker:

    def test_new_wall_spawns_unsmoothed_track(self):
        tracker = _make_tracker()
        wall = _make_wall(normal=[1.0, 0.0, 0.0], centroid=[2.0, 0.0, 0.5])
        wall['is_corner'] = False
        tracks = tracker.update([wall])
        assert len(tracks) == 1
        assert tracks[0].distance == wall['distance']
        assert tracks[0].bearing == pytest.approx(wall['bearing'])
        assert tracks[0].frames_since_seen == 0

    def test_matched_wall_smooths_across_frames(self):
        tracker = _make_tracker()
        wall1 = _make_wall(normal=[1.0, 0.0, 0.0], centroid=[2.0, 0.0, 0.5])
        wall1['is_corner'] = False
        tracker.update([wall1])
        wall2 = _make_wall(normal=[1.0, 0.0, 0.0], centroid=[2.2, 0.0, 0.5])
        wall2['is_corner'] = False
        tracks = tracker.update([wall2])
        assert len(tracks) == 1
        # Same track_id carried across frames (associated, not re-spawned).
        assert tracks[0].track_id == 0
        # Smoothed distance strictly between the two raw observations.
        assert 2.0 < tracks[0].distance < 2.2

    def test_unmatched_track_holds_then_drops(self):
        tracker = _make_tracker()
        wall = _make_wall(normal=[1.0, 0.0, 0.0], centroid=[2.0, 0.0, 0.5])
        wall['is_corner'] = False
        tracker.update([wall])

        # No detections for HOLD_FRAMES consecutive frames -- track must
        # still be alive (held) through all of them.
        for i in range(HOLD_FRAMES):
            tracks = tracker.update([])
            assert len(tracks) == 1, f'expected held track alive on empty frame {i + 1}'

        # One frame past the hold budget -- now dropped.
        tracks = tracker.update([])
        assert len(tracks) == 0

    def test_held_track_value_frozen_while_unmatched(self):
        tracker = _make_tracker()
        wall = _make_wall(normal=[1.0, 0.0, 0.0], centroid=[2.0, 0.0, 0.5])
        wall['is_corner'] = False
        tracker.update([wall])
        tracks = tracker.update([])
        assert tracks[0].distance == pytest.approx(2.0)

    def test_association_ignores_input_order(self):
        # Two walls, association must match by (distance, bearing), not by
        # list index -- feed them in one order, then the other, and confirm
        # each track's identity follows its wall, not its position.
        tracker = _make_tracker()
        wall_near = _make_wall(normal=[1.0, 0.0, 0.0], centroid=[1.0, 0.0, 0.5])
        wall_near['is_corner'] = False
        wall_far = _make_wall(normal=[0.0, 1.0, 0.0], centroid=[0.5, 3.0, 0.5])
        wall_far['is_corner'] = False
        tracker.update([wall_near, wall_far])
        near_id = next(t.track_id for t in tracker.tracks if t.distance == pytest.approx(1.0))
        far_id = next(t.track_id for t in tracker.tracks if t.distance == pytest.approx(3.0))

        # Same two walls, reversed order, slightly perturbed.
        wall_near2 = _make_wall(normal=[1.0, 0.0, 0.0], centroid=[1.05, 0.0, 0.5])
        wall_near2['is_corner'] = False
        wall_far2 = _make_wall(normal=[0.0, 1.0, 0.0], centroid=[0.5, 3.05, 0.5])
        wall_far2['is_corner'] = False
        tracker.update([wall_far2, wall_near2])

        near_track = next(t for t in tracker.tracks if t.track_id == near_id)
        far_track = next(t for t in tracker.tracks if t.track_id == far_id)
        assert near_track.distance == pytest.approx(1.05, abs=0.05)
        assert far_track.distance == pytest.approx(3.05, abs=0.05)


# ==============================================================================
# Corner case: the one place merge and tracking actually interact.
# ==============================================================================

class TestCornerIndependence:

    def test_perpendicular_corner_walls_never_merge(self):
        wall_a = _make_wall(normal=[1.0, 0.0, 0.0], centroid=[2.0, 0.0, 0.5])
        wall_b = _make_wall(normal=[0.0, 1.0, 0.0], centroid=[1.0, 1.0, 0.5])
        merged = merge_duplicate_walls([wall_a, wall_b], NORMAL_COS_THRESH, DISTANCE_THRESH_M)
        assert len(merged) == 2

    def test_corner_walls_get_independent_ema_state_no_cross_contamination(self):
        tracker = _make_tracker()
        wall_a = _make_wall(normal=[1.0, 0.0, 0.0], centroid=[2.0, 0.0, 0.5])
        wall_a['is_corner'] = True
        wall_b = _make_wall(normal=[0.0, 1.0, 0.0], centroid=[1.0, 1.0, 0.5])
        wall_b['is_corner'] = True
        tracker.update([wall_a, wall_b])

        # Drive wall_a's distance up over several frames; wall_b must stay
        # completely unaffected -- each wall's EMA state is independent, and
        # in particular there must be no shared "corner angle" quantity that
        # would couple the two (see wall_detector_node.py's own explicit
        # warning against double-lagging a derived corner-angle value).
        for step in range(5):
            wall_a_new = _make_wall(normal=[1.0, 0.0, 0.0], centroid=[2.0 + step * 0.1, 0.0, 0.5])
            wall_a_new['is_corner'] = True
            wall_b_same = _make_wall(normal=[0.0, 1.0, 0.0], centroid=[1.0, 1.0, 0.5])
            wall_b_same['is_corner'] = True
            tracks = tracker.update([wall_a_new, wall_b_same])

        assert len(tracks) == 2
        # Reorientation (see _make_wall's own docstring) flips both hinted
        # normals to point back toward the origin -- [-1,0,0] / [0,-1,0] --
        # so distinguish tracks by which axis carries the magnitude, not by
        # sign.
        track_a = next(t for t in tracks if abs(t.normal[0]) > 0.5)
        track_b = next(t for t in tracks if abs(t.normal[1]) > 0.5)
        assert track_a.distance > 2.0  # moved, as driven
        assert track_b.distance == pytest.approx(1.0)  # untouched by wall_a's drift
        # Normals stay ~orthogonal throughout -- no coupling leaked in.
        assert abs(float(np.dot(track_a.normal, track_b.normal))) < 0.05


# ==============================================================================
# Ambiguous zone -- WallTracker's spawn path, three-way classified against
# EXISTING TRACKS (not candidate-vs-candidate within one frame -- see module
# docstring's own scope note). Every scenario here seeds a real track first
# (a separate tracker.update() call) so the classification actually runs
# against something -- a wall's own very first sighting, with zero tracks
# alive yet, always classifies "distinct" and spawns immediately regardless
# of geometry (nothing to compare against), which is deliberately NOT what
# these tests are checking.
# ==============================================================================

class TestAmbiguousZone:

    def _seed_track_a(self, tracker):
        """x=2.0 plane, centroid=[2,0,0.5], normal->[-1,0,0], bearing=0,
        distance=2.0 -- the reference track every scenario below classifies
        a second wall against."""
        wall_a = _make_wall(normal=[1.0, 0.0, 0.0], centroid=[2.0, 0.0, 0.5], seed=100)
        tracks = tracker.update([wall_a])
        assert len(tracks) == 1
        return tracks[0].track_id

    def _wall_a_repeat(self, seed):
        """Same plane/position as _seed_track_a's wall every frame, so
        track_a keeps position-matching (via the existing, unmodified
        distance/bearing path) and never itself becomes a distinct/pending
        candidate while a scenario feeds a second wall alongside it."""
        return _make_wall(normal=[1.0, 0.0, 0.0], centroid=[2.0, 0.0, 0.5], seed=seed)

    def test_corner_pair_spawns_distinct_immediately_not_pending(self):
        tracker = _make_tracker()
        self._seed_track_a(tracker)
        wall_perp = _make_wall(normal=[0.0, 1.0, 0.0], centroid=[1.0, 1.0, 0.5], seed=101)
        tracks = tracker.update([self._wall_a_repeat(102), wall_perp])
        assert len(tracks) == 2
        assert tracker.pending == []

    def test_far_parallel_pair_spawns_distinct_immediately_not_pending(self):
        # Same normal direction as track_a, but offset_gap=2.5m (>=
        # MIN_DISTINCT_SEPARATION_M=2.0) -- confidently a different wall,
        # not held for confirmation. Also fails the existing distance/
        # bearing position-match outright (track_a.distance=2.0 vs this
        # wall's 4.5 -- see the module-level geometry note in this class'
        # own docstring), so it genuinely reaches the new classification
        # path rather than the pre-existing position-match one.
        tracker = _make_tracker()
        self._seed_track_a(tracker)
        wall_far = _make_wall(normal=[1.0, 0.0, 0.0], centroid=[4.5, 0.0, 0.5], seed=103)
        tracks = tracker.update([self._wall_a_repeat(104), wall_far])
        assert len(tracks) == 2
        assert tracker.pending == []

    def test_ambiguous_candidate_held_then_promoted_after_confirm_frames(self):
        # offset_gap=0.5m relative to track_a's plane -- strictly between
        # DISTANCE_THRESH_M (0.15) and MIN_DISTINCT_SEPARATION_M (2.0), and
        # this wall's own (distance=2.5, bearing=0) is far enough from
        # track_a's (2.0, 0) to fail the existing position-match too (d_gap
        # =0.5 > ASSOC_DISTANCE_THRESH_M=0.4), so it reaches the ambiguous
        # classification, not a trivial position-based re-match.
        tracker = _make_tracker()
        self._seed_track_a(tracker)

        seed = 200
        for i in range(AMBIGUOUS_CONFIRM_FRAMES - 1):
            wall_ambiguous = _make_wall(
                normal=[1.0, 0.0, 0.0], centroid=[2.5, 0.0, 0.5], seed=seed + i)
            tracks = tracker.update([self._wall_a_repeat(seed + 100 + i), wall_ambiguous])
            assert len(tracks) == 1, f'must not promote before confirm_frames (frame {i + 1})'
            assert len(tracker.pending) == 1
            assert tracker.pending[0]['streak'] == i + 1

        # One more consecutive appearance reaches AMBIGUOUS_CONFIRM_FRAMES.
        wall_ambiguous = _make_wall(
            normal=[1.0, 0.0, 0.0], centroid=[2.5, 0.0, 0.5],
            seed=seed + AMBIGUOUS_CONFIRM_FRAMES)
        tracks = tracker.update(
            [self._wall_a_repeat(seed + 999), wall_ambiguous])
        assert len(tracks) == 2
        assert tracker.pending == []

    def test_ambiguous_candidate_not_rematched_drops_no_partial_credit(self):
        tracker = _make_tracker()
        self._seed_track_a(tracker)

        # Two consecutive ambiguous appearances -- streak reaches 2, well
        # short of AMBIGUOUS_CONFIRM_FRAMES (5).
        for i in range(2):
            wall_ambiguous = _make_wall(
                normal=[1.0, 0.0, 0.0], centroid=[2.5, 0.0, 0.5], seed=300 + i)
            tracker.update([self._wall_a_repeat(400 + i), wall_ambiguous])
        assert len(tracker.pending) == 1
        assert tracker.pending[0]['streak'] == 2

        # A frame with nothing ambiguous in it at all -- the pending
        # candidate is not re-matched and must be dropped outright, not
        # held/decayed like a confirmed track.
        tracker.update([self._wall_a_repeat(500)])
        assert tracker.pending == []

        # Re-appearing now must take the FULL confirm_frames count again --
        # not resume from the old streak=2 (that would be partial credit).
        # AMBIGUOUS_CONFIRM_FRAMES - 1 more appearances must still NOT
        # promote.
        seed = 600
        for i in range(AMBIGUOUS_CONFIRM_FRAMES - 1):
            wall_ambiguous = _make_wall(
                normal=[1.0, 0.0, 0.0], centroid=[2.5, 0.0, 0.5], seed=seed + i)
            tracks = tracker.update([self._wall_a_repeat(seed + 100 + i), wall_ambiguous])
            assert len(tracks) == 1, (
                f'partial credit bug: promoted after only {i + 1} fresh appearances')
            assert tracker.pending[0]['streak'] == i + 1

        wall_ambiguous = _make_wall(
            normal=[1.0, 0.0, 0.0], centroid=[2.5, 0.0, 0.5],
            seed=seed + AMBIGUOUS_CONFIRM_FRAMES)
        tracks = tracker.update(
            [self._wall_a_repeat(seed + 999), wall_ambiguous])
        assert len(tracks) == 2  # confirmed on the full, fresh 5th appearance

    def test_ambiguous_candidate_later_satisfying_same_criteria_merges_into_track(self):
        # Frames 1-2: an ambiguous candidate (offset_gap=0.5m) accumulates a
        # streak, not yet confirmed.
        tracker = _make_tracker()
        self._seed_track_a(tracker)
        for i in range(2):
            wall_ambiguous = _make_wall(
                normal=[1.0, 0.0, 0.0], centroid=[2.5, 0.0, 0.5], seed=700 + i)
            tracker.update([self._wall_a_repeat(800 + i), wall_ambiguous])
        assert len(tracker.pending) == 1
        assert tracker.pending[0]['streak'] == 2

        # Frame 3: a DIFFERENT wall -- same exact plane as track_a (x=2.0,
        # same normal, offset_gap=0 -- unambiguously SAME) but shifted far
        # in y (centroid=[2.0, 3.0, 0.5]) so its OWN bearing (~56 deg) fails
        # the existing distance/bearing position-match against track_a
        # (bearing gate is +-15 deg) even though its 'distance' field
        # matches exactly -- so this can only reach track_a via the NEW
        # plane-geometry classification, not the pre-existing position
        # check. Also does NOT feed the earlier ambiguous wall again this
        # frame at all, so PendingWallGate.update() gets an empty ambiguous_
        # walls list -- exactly the "candidate merges into a track instead
        # of continuing its own pending streak" case: its pending entry
        # simply never gets re-matched and is dropped.
        wall_same_plane_far_bearing = _make_wall(
            normal=[1.0, 0.0, 0.0], centroid=[2.0, 3.0, 0.5], seed=701)
        tracks = tracker.update(
            [self._wall_a_repeat(801), wall_same_plane_far_bearing])

        assert len(tracks) == 1, 'must merge into track_a, not spawn a second track'
        # distance is provably UNCHANGED by this update (both the old and
        # new observation sit on the exact same x=2.0 plane, so EMA-
        # blending 2.0 with 2.0 stays 2.0) -- bearing/centroid are what
        # actually move here (the new wall's centroid is offset far in y),
        # so those are what confirm a real EMA update happened rather than
        # this being a no-op.
        assert tracks[0].bearing != pytest.approx(0.0, abs=1e-6), (
            'track_a should have EMA-updated its bearing toward the new '
            'observation (which sits far off in y), not stayed frozen at 0'
        )
        assert tracker.pending == [], (
            "the old ambiguous streak must be dropped, not promoted separately "
            "once its candidate turned out to be the same wall as track_a"
        )


# ==============================================================================
# Part A: raw candidate quality gating -- second-plane-only, see
# wall_detector_node.py's own module docstring ("Raw candidate quality
# gating (second-plane-only)") for the full investigation writeup these are
# built from. Motivated directly by that investigation's frame-9 example: a
# real live-mission log had two RANSAC planes fit from the SAME single
# point cloud, 44 deg apart in normal (normal_similarity=0.718), both
# independently clearing the old min_inliers=150-only floor -- these are
# the hypothesis for what a low-support/fragment-shaped second fit like
# that would fail, pending live-capture validation once the ZED camera
# hardware issue found during that same investigation is resolved (a
# cable/USB problem, not a code issue -- see the module docstring).
#
# _find_walls itself isn't exercised directly here (it's a bound
# WallDetectorNode method requiring a live Node + a real Open3D point cloud
# through segment_plane, whose own RANSAC has no fixed-seed determinism
# guarantee) -- same "pure logic separate from ROS glue" split this file's
# existing tests already follow, testing the gates as the standalone pure
# functions they are.
# ==============================================================================

MIN_INLIERS = 150
SECOND_PLANE_MIN_RESIDUAL_RATIO = 2.0
SECOND_PLANE_MIN_INLIER_RATIO = 0.6
SECOND_PLANE_MAX_ASPECT_RATIO = 6.0


class TestResidualSupportsSecondPlane:

    def test_residual_too_small_rejected(self):
        # 250 < 2.0 * 150 -- not plausibly big enough to hide a real second
        # wall's worth of RANSAC support underneath the dominant plane's.
        assert not _residual_supports_second_plane(
            residual_size=250, min_inliers=MIN_INLIERS,
            min_ratio=SECOND_PLANE_MIN_RESIDUAL_RATIO)

    def test_residual_large_enough_accepted(self):
        assert _residual_supports_second_plane(
            residual_size=400, min_inliers=MIN_INLIERS,
            min_ratio=SECOND_PLANE_MIN_RESIDUAL_RATIO)

    def test_boundary_is_inclusive(self):
        # Exactly the ratio floor -- >= , not > (matches the function's own
        # "residual_size >= min_ratio * min_inliers" implementation).
        assert _residual_supports_second_plane(
            residual_size=300, min_inliers=MIN_INLIERS,
            min_ratio=SECOND_PLANE_MIN_RESIDUAL_RATIO)


class TestSecondPlaneInlierRatio:

    def test_low_inlier_ratio_rejected(self):
        # 140/300 ~= 0.467, below the 0.6 floor -- a marginal fit that
        # barely cleared min_inliers relative to how much residual data it
        # had to work with (the frame-9 investigation's hypothesized
        # signature: a plane through only a small slice of the residual,
        # not a genuine, well-supported second surface).
        passed, ratio = _second_plane_inlier_ratio_ok(
            inlier_count=140, residual_size_before=300,
            min_ratio=SECOND_PLANE_MIN_INLIER_RATIO)
        assert not passed
        assert ratio == pytest.approx(140 / 300)

    def test_high_inlier_ratio_accepted(self):
        # A genuine second wall (e.g. a real corner) explaining most of the
        # residual it was fit from.
        passed, ratio = _second_plane_inlier_ratio_ok(
            inlier_count=250, residual_size_before=300,
            min_ratio=SECOND_PLANE_MIN_INLIER_RATIO)
        assert passed
        assert ratio == pytest.approx(250 / 300)

    def test_zero_residual_does_not_divide_by_zero(self):
        passed, ratio = _second_plane_inlier_ratio_ok(
            inlier_count=0, residual_size_before=0,
            min_ratio=SECOND_PLANE_MIN_INLIER_RATIO)
        assert not passed
        assert ratio == 0.0


class TestSecondPlaneCompactness:

    def test_fragment_shaped_inlier_set_rejected(self):
        # A thin sliver: wide along one in-plane axis, near-zero spread
        # along the other -- the geometric signature of a fit through a
        # sparse/linear residual cluster rather than a coherent wall face.
        rng = np.random.default_rng(11)
        normal = np.array([1.0, 0.0, 0.0])
        centroid = np.array([2.0, 0.0, 0.5])
        along = np.array([0.0, 1.0, 0.0])
        up = np.array([0.0, 0.0, 1.0])
        along_coord = rng.uniform(-1.0, 1.0, size=200)
        up_coord = rng.uniform(-0.02, 0.02, size=200)
        pts = centroid + along_coord[:, None] * along + up_coord[:, None] * up
        passed, ratio = _second_plane_compact_ok(
            pts, normal, centroid, max_aspect_ratio=SECOND_PLANE_MAX_ASPECT_RATIO)
        assert not passed
        assert ratio > SECOND_PLANE_MAX_ASPECT_RATIO

    def test_coherent_patch_accepted(self):
        # Comparable spread along both in-plane axes -- a real wall-face
        # patch, not a sliver.
        rng = np.random.default_rng(12)
        normal = np.array([1.0, 0.0, 0.0])
        centroid = np.array([2.0, 0.0, 0.5])
        along = np.array([0.0, 1.0, 0.0])
        up = np.array([0.0, 0.0, 1.0])
        along_coord = rng.uniform(-0.5, 0.5, size=200)
        up_coord = rng.uniform(-0.4, 0.4, size=200)
        pts = centroid + along_coord[:, None] * along + up_coord[:, None] * up
        passed, ratio = _second_plane_compact_ok(
            pts, normal, centroid, max_aspect_ratio=SECOND_PLANE_MAX_ASPECT_RATIO)
        assert passed
        assert ratio <= SECOND_PLANE_MAX_ASPECT_RATIO


# ==============================================================================
# Part B: front_clearance selection eligibility -- see wall_detector_node.py's
# own module docstring ("front_clearance selection eligibility") for the
# full investigation writeup. Directly mirrors the investigated failure: a
# short-lived, low-RANSAC-support track (track_id=14 in the real log --
# bearing -17 deg, well inside a 35 deg front-facing cone, distance dipping
# under 1m purely from its own fit noise) sitting CLOSER than the real,
# persistent, well-supported wall the mission was actually approaching.
# ==============================================================================

FRONT_FACING_MAX_RAD = math.radians(35.0)
FRONT_CLEARANCE_MIN_TRACK_FRAMES = 5
FRONT_CLEARANCE_MIN_INLIERS = 300


class TestFrontClearanceEligibility:

    def test_short_lived_low_support_track_not_eligible(self):
        tw = TrackedWall(
            track_id=14, distance=0.97, bearing=math.radians(-17.0),
            normal=[1.0, 0.0, 0.0], centroid=[0.97, 0.0, 0.5], is_corner=False,
            points=np.zeros((140, 3)), extent=np.zeros(3))
        assert not _front_clearance_eligible(
            tw, FRONT_FACING_MAX_RAD, FRONT_CLEARANCE_MIN_TRACK_FRAMES,
            FRONT_CLEARANCE_MIN_INLIERS)

        # Still short-lived AND still low-support after a few matched
        # updates -- must remain ineligible on both counts.
        for _ in range(3):
            tw.update(
                distance=0.96, bearing=math.radians(-17.0), normal=[1.0, 0.0, 0.0],
                centroid=[0.96, 0.0, 0.5], is_corner=False, points=np.zeros((145, 3)),
                extent=np.zeros(3), alpha=0.3)
        assert tw.frames_matched == 4
        assert not _front_clearance_eligible(
            tw, FRONT_FACING_MAX_RAD, FRONT_CLEARANCE_MIN_TRACK_FRAMES,
            FRONT_CLEARANCE_MIN_INLIERS)

    def test_track_frames_alone_is_not_enough_without_inlier_support(self):
        # Investigation's own finding: the real phantom track had already
        # accumulated ~38 matched frames by the time it mattered -- a
        # frame-count bar alone, at a value that doesn't cost real walls
        # unacceptable latency, does NOT reject it. Confirms
        # front_clearance_min_inliers is doing real work here, not
        # min_track_frames alone.
        tw = TrackedWall(
            track_id=14, distance=1.2, bearing=math.radians(-17.0),
            normal=[1.0, 0.0, 0.0], centroid=[1.2, 0.0, 0.5], is_corner=False,
            points=np.zeros((140, 3)), extent=np.zeros(3))
        for _ in range(40):
            tw.update(
                distance=0.96, bearing=math.radians(-17.0), normal=[1.0, 0.0, 0.0],
                centroid=[0.96, 0.0, 0.5], is_corner=False, points=np.zeros((140, 3)),
                extent=np.zeros(3), alpha=0.3)
        assert tw.frames_matched >= FRONT_CLEARANCE_MIN_TRACK_FRAMES
        # A bearing-cone + age-only rule (no inlier floor) would have
        # called this eligible by now -- confirming min_inliers is the
        # check actually doing the rejecting here.
        assert not _front_clearance_eligible(
            tw, FRONT_FACING_MAX_RAD, FRONT_CLEARANCE_MIN_TRACK_FRAMES,
            FRONT_CLEARANCE_MIN_INLIERS)

    def test_persistent_well_supported_track_is_eligible(self):
        tw = TrackedWall(
            track_id=3, distance=1.8, bearing=math.radians(7.0),
            normal=[-1.0, 0.0, 0.0], centroid=[1.8, 0.0, 0.5], is_corner=False,
            points=np.zeros((600, 3)), extent=np.zeros(3))
        for _ in range(5):
            tw.update(
                distance=1.7, bearing=math.radians(7.0), normal=[-1.0, 0.0, 0.0],
                centroid=[1.7, 0.0, 0.5], is_corner=False, points=np.zeros((620, 3)),
                extent=np.zeros(3), alpha=0.3)
        assert _front_clearance_eligible(
            tw, FRONT_FACING_MAX_RAD, FRONT_CLEARANCE_MIN_TRACK_FRAMES,
            FRONT_CLEARANCE_MIN_INLIERS)

    def test_off_axis_track_never_eligible_regardless_of_support(self):
        # bearing well outside front_facing_max -- must fail regardless of
        # age/support, exactly like today's existing bearing-cone check.
        tw = TrackedWall(
            track_id=5, distance=1.0, bearing=math.radians(50.0),
            normal=[-1.0, 0.0, 0.0], centroid=[1.0, 0.0, 0.5], is_corner=False,
            points=np.zeros((900, 3)), extent=np.zeros(3))
        for _ in range(10):
            tw.update(
                distance=1.0, bearing=math.radians(50.0), normal=[-1.0, 0.0, 0.0],
                centroid=[1.0, 0.0, 0.5], is_corner=False, points=np.zeros((900, 3)),
                extent=np.zeros(3), alpha=0.3)
        assert not _front_clearance_eligible(
            tw, FRONT_FACING_MAX_RAD, FRONT_CLEARANCE_MIN_TRACK_FRAMES,
            FRONT_CLEARANCE_MIN_INLIERS)

    def test_selection_ignores_ineligible_phantom_picks_real_wall(self):
        """End-to-end mirror of the investigation's actual failure: a
        short-lived, low-support phantom track sits CLOSER and within the
        front-facing cone than the real, persistent, well-supported wall --
        min(distance) selection must pick the real wall's distance once
        Part B's eligibility gate is applied first, not the phantom's."""
        phantom = TrackedWall(
            track_id=14, distance=0.97, bearing=math.radians(-17.0),
            normal=[1.0, 0.0, 0.0], centroid=[0.97, 0.0, 0.5], is_corner=False,
            points=np.zeros((140, 3)), extent=np.zeros(3))
        real_wall = TrackedWall(
            track_id=3, distance=1.8, bearing=math.radians(7.0),
            normal=[-1.0, 0.0, 0.0], centroid=[1.8, 0.0, 0.5], is_corner=False,
            points=np.zeros((600, 3)), extent=np.zeros(3))
        for _ in range(6):
            real_wall.update(
                distance=1.75, bearing=math.radians(7.0), normal=[-1.0, 0.0, 0.0],
                centroid=[1.75, 0.0, 0.5], is_corner=False, points=np.zeros((620, 3)),
                extent=np.zeros(3), alpha=0.3)

        tracked_walls = [phantom, real_wall]
        eligible_distances = [
            w.distance for w in tracked_walls
            if _front_clearance_eligible(
                w, FRONT_FACING_MAX_RAD, FRONT_CLEARANCE_MIN_TRACK_FRAMES,
                FRONT_CLEARANCE_MIN_INLIERS)
        ]
        assert eligible_distances == pytest.approx([real_wall.distance])

        # The OLD, naive rule (bearing-cone only, no eligibility gate) would
        # have picked the phantom's smaller distance instead -- confirms
        # this scenario actually exercises the fix, not a vacuous case.
        naive_candidates = [
            w.distance for w in tracked_walls if abs(w.bearing) < FRONT_FACING_MAX_RAD]
        assert min(naive_candidates) == pytest.approx(phantom.distance)


# ==============================================================================
# Motion compensation -- see wall_detector_node.py's own module docstring
# ("Motion compensation" section). Three layers: _odom_delta (two odometry
# poses -> a robot-frame ego delta), _predict_track_position (one track +
# a delta -> its predicted current position), and WallTracker.update()'s
# own ego_delta parameter (the actual integration point -- the association-
# level test below is the one that proves this matters at all, not just
# that the transform math is correct in isolation).
# ==============================================================================

class TestOdomDelta:

    def test_pure_forward_translation(self):
        delta = _odom_delta((0.0, 0.0, 0.0), (1.0, 0.0, 0.0))
        assert delta == pytest.approx((1.0, 0.0, 0.0))

    def test_pure_rotation(self):
        delta = _odom_delta((0.0, 0.0, 0.0), (0.0, 0.0, math.pi / 2))
        assert delta == pytest.approx((0.0, 0.0, math.pi / 2))

    def test_translation_expressed_in_prev_poses_own_frame(self):
        # Robot at world heading 90deg (facing +y) moves 1m further in +y
        # (world frame), no further rotation -- in the robot's OWN frame at
        # prev_pose, "forward" IS +y, so this must read as pure forward
        # motion (dx=1, dy=0), not a naive world-frame [0, 1] translation.
        delta = _odom_delta((0.0, 0.0, math.pi / 2), (0.0, 1.0, math.pi / 2))
        assert delta == pytest.approx((1.0, 0.0, 0.0), abs=1e-9)

    def test_bearing_wraparound_seam(self):
        # Same +-pi seam case TrackedWall.update()'s own bearing blending
        # already handles -- old=+179deg, new=-179deg is really only a
        # 2deg change, not ~358deg.
        delta = _odom_delta(
            (0.0, 0.0, math.radians(179.0)), (0.0, 0.0, math.radians(-179.0)))
        assert delta[2] == pytest.approx(math.radians(2.0))


class TestPredictTrackPosition:

    def test_forward_translation_shrinks_distance(self):
        tw = TrackedWall(
            track_id=0, distance=2.0, bearing=0.0, normal=[-1.0, 0.0, 0.0],
            centroid=[2.0, 0.0, 0.5], is_corner=False, points=np.zeros((1, 3)),
            extent=np.zeros(3))
        predicted = _predict_track_position(tw, (0.5, 0.0, 0.0))
        assert predicted['distance'] == pytest.approx(1.5)
        assert predicted['bearing'] == pytest.approx(0.0)
        assert predicted['centroid'] == pytest.approx([1.5, 0.0, 0.5])
        assert predicted['normal'] == pytest.approx([-1.0, 0.0, 0.0])

    def test_zero_delta_is_identity(self):
        tw = TrackedWall(
            track_id=0, distance=2.0, bearing=0.0, normal=[-1.0, 0.0, 0.0],
            centroid=[2.0, 0.0, 0.5], is_corner=False, points=np.zeros((1, 3)),
            extent=np.zeros(3))
        predicted = _predict_track_position(tw, (0.0, 0.0, 0.0))
        assert predicted['distance'] == pytest.approx(2.0)
        assert predicted['bearing'] == pytest.approx(0.0)
        assert predicted['centroid'] == pytest.approx([2.0, 0.0, 0.5])


class TestWallTrackerMotionCompensation:

    def test_association_succeeds_with_compensation_fails_without(self):
        """The scenario this whole feature exists for: the robot closes
        0.5m on a wall in a single tick (exceeds ASSOC_DISTANCE_THRESH_M=
        0.4 on its own) -- WITH compensation, the track correctly
        associates and its distance actually updates to reflect the new,
        closer reading. WITHOUT compensation, the exact same real-world
        approach looks like too large a jump: the track is simply held at
        its STALE distance (never actually updated this frame) while the
        fresh detection sits in ambiguous-zone limbo instead of being
        trusted -- proving the gap costs real, wrong front_clearance-
        relevant behavior, not just failing an assertion in the abstract.
        """
        wall1 = _make_wall(normal=[1.0, 0.0, 0.0], centroid=[2.0, 0.0, 0.5], seed=900)
        wall1['is_corner'] = False
        ego_delta = (0.5, 0.0, 0.0)
        wall2 = _make_wall(normal=[1.0, 0.0, 0.0], centroid=[1.5, 0.0, 0.5], seed=901)
        wall2['is_corner'] = False

        tracker_compensated = _make_tracker()
        tracker_compensated.update([wall1])
        tracks = tracker_compensated.update([wall2], ego_delta=ego_delta)
        assert len(tracks) == 1
        assert tracks[0].track_id == 0
        assert tracks[0].distance == pytest.approx(1.5, abs=0.05), (
            'compensated track should have EMA-updated toward wall2s real '
            '(post-motion) distance of 1.5, not stayed near 2.0'
        )
        assert tracker_compensated.pending == []

        tracker_uncompensated = _make_tracker()
        tracker_uncompensated.update([wall1])
        tracks_unc = tracker_uncompensated.update([wall2])  # no ego_delta
        assert len(tracks_unc) == 1
        assert tracks_unc[0].distance == pytest.approx(2.0), (
            'without compensation, track1 should stay frozen at its stale '
            'distance -- it was never actually matched/updated this frame'
        )
        assert len(tracker_uncompensated.pending) == 1, (
            'the fresh detection should be stuck in ambiguous-zone limbo, '
            'not confidently matched -- this IS the cost of missing '
            'compensation, not a vacuous side effect'
        )

    def test_ego_delta_none_is_a_graceful_noop(self):
        """Odometry unavailable/stale -- WallDetectorNode passes ego_delta=
        None in that case (see _consume_ego_delta), identical to every
        existing caller (all 36 pre-existing tests) that never passes it at
        all. Must not crash, and normal association must keep working
        exactly as it did before this feature existed."""
        tracker = _make_tracker()
        wall = _make_wall(normal=[1.0, 0.0, 0.0], centroid=[2.0, 0.0, 0.5])
        wall['is_corner'] = False
        tracks = tracker.update([wall], ego_delta=None)
        assert len(tracks) == 1
        assert tracks[0].distance == pytest.approx(2.0)

        wall2 = _make_wall(normal=[1.0, 0.0, 0.0], centroid=[2.02, 0.0, 0.5], seed=1)
        wall2['is_corner'] = False
        tracks = tracker.update([wall2], ego_delta=None)
        assert len(tracks) == 1
        assert tracks[0].track_id == 0


# ==============================================================================
# Confidence-based pruning -- see wall_detector_node.py's own module
# docstring ("Confidence-based pruning" section).
# ==============================================================================

PRUNE_CONFLICT_RADIUS_M = 2.0
PRUNE_MIN_FRAMES_BEFORE_ELIGIBLE = 3
PRUNE_INLIER_RATIO_FLOOR = 0.5
PRUNE_MATCH_STREAK_RATIO_FLOOR = 0.5


def _make_track(track_id, distance, bearing, normal, centroid, n_inliers, frames_matched):
    tw = TrackedWall(
        track_id=track_id, distance=distance, bearing=bearing, normal=normal,
        centroid=centroid, is_corner=False, points=np.zeros((n_inliers, 3)),
        extent=np.zeros(3))
    tw.frames_matched = frames_matched
    return tw


def _prune(tracks):
    return prune_inconsistent_tracks(
        tracks, NORMAL_COS_THRESH, DISTANCE_THRESH_M, PRUNE_CONFLICT_RADIUS_M,
        PRUNE_MIN_FRAMES_BEFORE_ELIGIBLE, PRUNE_INLIER_RATIO_FLOOR,
        PRUNE_MATCH_STREAK_RATIO_FLOOR)


class TestPruneInconsistentTracks:

    def test_close_nonperpendicular_pair_with_support_gap_prunes_weaker(self):
        strong = _make_track(
            0, 1.8, math.radians(7.0), [-1.0, 0.0, 0.0], [1.8, 0.0, 0.5],
            n_inliers=600, frames_matched=10)
        weak = _make_track(
            1, 1.75, math.radians(6.0), [-1.0, 0.0, 0.0], [1.75, 0.0, 0.5],
            n_inliers=200, frames_matched=10)
        assert _prune([strong, weak]) == [1]

    def test_comparable_support_neither_pruned(self):
        strong = _make_track(
            0, 1.8, math.radians(7.0), [-1.0, 0.0, 0.0], [1.8, 0.0, 0.5],
            n_inliers=550, frames_matched=10)
        similar = _make_track(
            1, 1.75, math.radians(6.0), [-1.0, 0.0, 0.0], [1.75, 0.0, 0.5],
            n_inliers=500, frames_matched=10)
        assert _prune([strong, similar]) == []

    def test_grace_period_protects_new_track_despite_support_gap(self):
        strong = _make_track(
            0, 1.8, math.radians(7.0), [-1.0, 0.0, 0.0], [1.8, 0.0, 0.5],
            n_inliers=600, frames_matched=10)
        fresh_weak = _make_track(
            1, 1.75, math.radians(6.0), [-1.0, 0.0, 0.0], [1.75, 0.0, 0.5],
            n_inliers=50, frames_matched=1)  # < PRUNE_MIN_FRAMES_BEFORE_ELIGIBLE
        assert _prune([strong, fresh_weak]) == []

    def test_corner_pair_never_pruned_regardless_of_support_gap(self):
        strong = _make_track(
            0, 1.8, math.radians(7.0), [-1.0, 0.0, 0.0], [1.8, 0.0, 0.5],
            n_inliers=5000, frames_matched=10)
        weak_but_perpendicular = _make_track(
            1, 1.0, math.radians(-83.0), [0.0, -1.0, 0.0], [0.1, -1.0, 0.5],
            n_inliers=50, frames_matched=10)
        assert _prune([strong, weak_but_perpendicular]) == []

    def test_weaker_track_helper_ambiguous_double_conflict_keeps_both(self):
        # Pathological: a is weaker on inliers, b is weaker on frames_matched
        # -- each looks weaker than the other by a DIFFERENT signal. Neither
        # should be dropped (see _weaker_track's own docstring).
        a = _make_track(
            0, 1.8, 0.0, [-1.0, 0.0, 0.0], [1.8, 0.0, 0.5],
            n_inliers=100, frames_matched=20)
        b = _make_track(
            1, 1.8, 0.0, [-1.0, 0.0, 0.0], [1.8, 0.0, 0.5],
            n_inliers=1000, frames_matched=5)
        assert _weaker_track(
            a, b, PRUNE_MIN_FRAMES_BEFORE_ELIGIBLE, PRUNE_INLIER_RATIO_FLOOR,
            PRUNE_MATCH_STREAK_RATIO_FLOOR) is None

    def test_track_id_14_scenario_gets_pruned_within_reasonable_frames(self):
        """Mirrors the front_clearance investigation's track_id=14: a
        short-lived, persistently low-support track sitting close to
        (offset_gap well within PRUNE_CONFLICT_RADIUS_M) and non-
        perpendicular to a real, well-supported wall, matching ITSELF
        every frame via ordinary position-based association -- so it never
        ages out via track_hold_frames -- exactly the shape the
        investigation found nothing in the original code ever cleaned up.
        Confidence-based pruning is the only mechanism that removes it."""
        tracker = _make_tracker()
        # Seed two already-separate, already-confirmed tracks directly
        # (spawn mechanics aren't what's under test here) -- different
        # enough in BEARING (7deg vs -17deg, a 24deg gap > ASSOC_BEARING_
        # THRESH_RAD=15deg) that ordinary position-based association below
        # keeps matching each candidate to its OWN track, never cross-
        # matching, while still being close+non-perpendicular in PLANE
        # geometry (offset_gap ~0.3m, well under PRUNE_CONFLICT_RADIUS_M).
        real = TrackedWall(
            track_id=0, distance=1.8, bearing=math.radians(7.0),
            normal=np.array([-1.0, 0.0, 0.0]), centroid=np.array([1.8, 0.22, 0.5]),
            is_corner=False, points=np.zeros((600, 3)), extent=np.zeros(3))
        phantom = TrackedWall(
            track_id=1, distance=1.5, bearing=math.radians(-17.0),
            normal=np.array([-1.0, 0.0, 0.0]), centroid=np.array([1.5, -0.46, 0.5]),
            is_corner=False, points=np.zeros((140, 3)), extent=np.zeros(3))
        tracker._tracks = [real, phantom]
        tracker._next_id = 2

        tracks = []
        for i in range(4):
            real_i = _make_wall(
                normal=[1.0, 0.0, 0.0], centroid=[1.8, 0.22, 0.5], n_points=600,
                seed=2000 + i)
            real_i['is_corner'] = False
            phantom_i = _make_wall(
                normal=[1.0, 0.0, 0.0], centroid=[1.5, -0.46, 0.5], n_points=140,
                seed=2100 + i)
            phantom_i['is_corner'] = False
            tracks = tracker.update([real_i, phantom_i])

        remaining_ids = {t.track_id for t in tracks}
        assert 1 not in remaining_ids, 'phantom track should have been pruned by now'
        assert 0 in remaining_ids, 'the real, well-supported track must survive'


# ==============================================================================
# Hard boundary constraints (/perception/front_wall_boundary) -- see
# wall_detector_node.py's own module docstring ("Hard boundary constraints"
# section) for the full sign-convention derivation.
# ==============================================================================

class TestWallBoundaryFromTrack:

    def test_straight_ahead_wall_sign_and_offset(self):
        # Wall directly ahead at x=2.0 -- WallDetection convention: normal
        # oriented BACK toward the robot, i.e. [-1, 0, 0].
        tw = TrackedWall(
            track_id=0, distance=2.0, bearing=0.0, normal=[-1.0, 0.0, 0.0],
            centroid=[2.0, 0.0, 0.5], is_corner=False, points=np.zeros((1, 3)),
            extent=np.zeros(3))
        nx, ny, offset = _wall_boundary_from_track(tw)
        # Flipped -- points AWAY from the robot, toward the wall.
        assert (nx, ny) == pytest.approx((1.0, 0.0))
        assert offset == pytest.approx(2.0)
        # Sanity: the robot's own origin must satisfy the free-space side of
        # the resulting halfspace (normal . p <= offset), with margin to spare.
        assert nx * 0.0 + ny * 0.0 <= offset

    def test_angled_wall_projection_rescales_offset(self):
        # Normal tilted 20deg off pure-horizontal (verticality_max_deg's own
        # extreme) -- 2D projection is NOT unit length ([-cos20, 0] has
        # magnitude cos20 < 1), so offset must be rescaled by 1/cos20, not
        # reused as-is (see module docstring's derivation).
        cos20, sin20 = math.cos(math.radians(20.0)), math.sin(math.radians(20.0))
        tw = TrackedWall(
            track_id=0, distance=2.0, bearing=0.0, normal=[-cos20, 0.0, sin20],
            centroid=[2.0, 0.0, 0.5], is_corner=False, points=np.zeros((1, 3)),
            extent=np.zeros(3))
        nx, ny, offset = _wall_boundary_from_track(tw)
        assert (nx, ny) == pytest.approx((1.0, 0.0), abs=1e-9), (
            'the 2D projection must be renormalized to unit length'
        )
        assert offset == pytest.approx(2.0 / cos20), (
            'offset must be rescaled by the SAME 1/cos20 factor, not left at '
            'the raw 3D distance'
        )

    def test_angled_bearing_wall(self):
        # Wall off to the side (bearing != 0) -- normal need not be
        # axis-aligned; just confirm the flip + renormalization holds
        # generally, not only for the straight-ahead case.
        normal = np.array([-0.8, 0.6, 0.0])  # already unit in 2D
        tw = TrackedWall(
            track_id=0, distance=1.5, bearing=math.radians(30.0), normal=normal,
            centroid=[1.2, 0.9, 0.5], is_corner=False, points=np.zeros((1, 3)),
            extent=np.zeros(3))
        nx, ny, offset = _wall_boundary_from_track(tw)
        assert (nx, ny) == pytest.approx((0.8, -0.6))
        assert offset == pytest.approx(1.5)


class TestFrontWallBoundaryGating:
    """Mirrors TestFrontClearanceEligibility's own selection-logic tests --
    _publish()'s boundary-array construction reuses the EXACT SAME
    eligible_front_tracks list front_clearance's own min() draws from, so
    these replicate that selection expression directly (same pattern the
    front_clearance tests already use), not a live Node/publisher."""

    def test_ineligible_phantom_produces_no_constraint(self):
        phantom = TrackedWall(
            track_id=14, distance=0.97, bearing=math.radians(-17.0),
            normal=[1.0, 0.0, 0.0], centroid=[0.97, 0.0, 0.5], is_corner=False,
            points=np.zeros((140, 3)), extent=np.zeros(3))
        tracked_walls = [phantom]
        eligible_front_tracks = [
            w for w in tracked_walls
            if _front_clearance_eligible(
                w, FRONT_FACING_MAX_RAD, FRONT_CLEARANCE_MIN_TRACK_FRAMES,
                FRONT_CLEARANCE_MIN_INLIERS)
        ]
        assert eligible_front_tracks == [], (
            'an ineligible track must produce an EMPTY boundary array, not '
            'a constraint built from it'
        )

    def test_eligible_winner_matches_front_clearance_selection(self):
        phantom = TrackedWall(
            track_id=14, distance=0.97, bearing=math.radians(-17.0),
            normal=[1.0, 0.0, 0.0], centroid=[0.97, 0.0, 0.5], is_corner=False,
            points=np.zeros((140, 3)), extent=np.zeros(3))
        real_wall = TrackedWall(
            track_id=3, distance=1.8, bearing=math.radians(7.0),
            normal=[-1.0, 0.0, 0.0], centroid=[1.8, 0.0, 0.5], is_corner=False,
            points=np.zeros((600, 3)), extent=np.zeros(3))
        for _ in range(6):
            real_wall.update(
                distance=1.75, bearing=math.radians(7.0), normal=[-1.0, 0.0, 0.0],
                centroid=[1.75, 0.0, 0.5], is_corner=False, points=np.zeros((620, 3)),
                extent=np.zeros(3), alpha=0.3)

        tracked_walls = [phantom, real_wall]
        eligible_front_tracks = [
            w for w in tracked_walls
            if _front_clearance_eligible(
                w, FRONT_FACING_MAX_RAD, FRONT_CLEARANCE_MIN_TRACK_FRAMES,
                FRONT_CLEARANCE_MIN_INLIERS)
        ]
        assert len(eligible_front_tracks) == 1
        winner = min(eligible_front_tracks, key=lambda t: t.distance)
        assert winner.track_id == real_wall.track_id

        nx, ny, offset = _wall_boundary_from_track(winner)
        assert offset == pytest.approx(real_wall.distance)
        # NOT built from the phantom's distance.
        assert offset != pytest.approx(phantom.distance, abs=0.5)


# ==============================================================================
# Track-identity stickiness (/perception/front_wall_boundary) -- see module
# docstring's "Track-identity stickiness" section, added by the "Boundary
# detection hardening" pass.
# ==============================================================================

class TestSelectBoundaryTrack:

    def test_no_previous_selection_falls_back_to_min_distance(self):
        # First-ever publish (or the very first tick after a full restart):
        # nothing to be sticky about, must behave exactly like the old
        # plain min(distance) selection.
        near = _make_track(
            0, 1.0, 0.0, [-1.0, 0.0, 0.0], [1.0, 0.0, 0.5], n_inliers=400, frames_matched=6)
        far = _make_track(
            1, 1.5, 0.0, [-1.0, 0.0, 0.0], [1.5, 0.0, 0.5], n_inliers=400, frames_matched=6)
        winner = _select_boundary_track([far, near], last_track_id=None)
        assert winner.track_id == 0

    def test_sticks_with_the_previous_winner_even_when_a_closer_track_is_eligible(self):
        # This is the actual flip-flop this fix exists to prevent: TWO
        # tracks simultaneously eligible, and ordinary per-frame noise
        # swaps which one is momentarily closer -- the old fresh-min()
        # selection would flip the published constraint's geometry every
        # time; this must not.
        previous_winner = _make_track(
            0, 1.05, 0.0, [-1.0, 0.0, 0.0], [1.05, 0.0, 0.5],
            n_inliers=400, frames_matched=6)
        now_closer = _make_track(
            1, 0.98, 0.0, [-1.0, 0.0, 0.0], [0.98, 0.0, 0.5],
            n_inliers=400, frames_matched=6)
        winner = _select_boundary_track(
            [now_closer, previous_winner], last_track_id=previous_winner.track_id)
        assert winner.track_id == previous_winner.track_id, (
            'must keep publishing the SAME track even though a different '
            'eligible track is now (marginally) closer'
        )

    def test_falls_back_to_min_distance_once_the_previous_winner_is_no_longer_eligible(self):
        # The previous winner genuinely dropped out (pruned, exceeded
        # track_hold_frames, or just aged past eligibility) -- there is no
        # continuity left to preserve, so this must fall back to min().
        still_here = _make_track(
            0, 1.2, 0.0, [-1.0, 0.0, 0.0], [1.2, 0.0, 0.5], n_inliers=400, frames_matched=6)
        winner = _select_boundary_track([still_here], last_track_id=999)
        assert winner.track_id == 0

    def test_reproduces_the_bag_pattern_track_present_briefly_gone_different_track_appears(self):
        """Reproduces the exact shape boundary_constraint_diag_20260813_124824
        showed: track A present and selected, a gap (A no longer in
        eligible_front_tracks at all -- see _publish()'s own reset of
        self._last_boundary_track_id to None whenever eligible_front_tracks
        is empty for a tick), then a DIFFERENT track B becomes eligible.

        Simulates _publish()'s own tick-by-tick state machine directly
        (last_track_id carried by hand across calls, mirroring self._last_
        boundary_track_id) rather than constructing a live Node.

        IMPORTANT, evidence-based (see module docstring): this fix does
        NOT and structurally CANNOT prevent the jump in THIS scenario --
        once A's track_id is genuinely gone (there is nothing left to be
        sticky about), B is the only eligible candidate and must be
        selected. Direct measurement of the bag itself confirmed every one
        of its 21 observed gaps (0.3-7.8s, one measured at 68 consecutive
        misses) vastly exceeds track_hold_frames' own ~5-frame hold
        window, so A is never still-alive-and-held by the time B appears --
        this is a genuine information gap, not a selection-layer bug. This
        test exists to PIN DOWN that expected, correct behavior (so a
        future change doesn't accidentally start fabricating continuity
        across a real gap), not to assert the jump is eliminated."""
        track_a = _make_track(
            0, 1.5, 0.0, [-1.0, 0.0, 0.0], [1.5, 0.0, 0.5], n_inliers=400, frames_matched=6)

        # Tick 1: A is present and eligible -- selected (first-ever pick).
        last_track_id = None
        winner = _select_boundary_track([track_a], last_track_id)
        last_track_id = winner.track_id
        assert winner.track_id == track_a.track_id

        # Ticks 2..N: A drops out of eligible_front_tracks entirely (a real
        # gap, longer than track_hold_frames) -- _publish() never calls
        # _select_boundary_track on an empty list; it resets the
        # remembered id to None directly instead (see _publish()'s own
        # `else: self._last_boundary_track_id = None` branch).
        for _ in range(10):
            eligible_front_tracks = []
            if eligible_front_tracks:
                winner = _select_boundary_track(eligible_front_tracks, last_track_id)
                last_track_id = winner.track_id
            else:
                last_track_id = None
        assert last_track_id is None

        # Tick N+1: a DIFFERENT track (B) becomes eligible -- genuinely new
        # information, no prior identity survives to prefer.
        track_b = _make_track(
            1, 1.1, 0.0, [-1.0, 0.0, 0.0], [1.1, 0.0, 0.5], n_inliers=400, frames_matched=6)
        winner = _select_boundary_track([track_b], last_track_id)
        assert winner.track_id == track_b.track_id, (
            'B is correctly selected -- there is no A identity left to '
            'stick to after a genuine, multi-frame gap'
        )
        # The jump itself (1.5m -> 1.1m) is real and expected here, not a
        # bug this fix is meant to hide -- see the docstring above.
        assert winner.distance != pytest.approx(track_a.distance)


if __name__ == '__main__':
    import sys
    sys.exit(pytest.main([__file__, '-v']))
