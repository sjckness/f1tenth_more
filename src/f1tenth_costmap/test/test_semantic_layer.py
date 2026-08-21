"""semantic_layer.py tests -- mirrors f1tenth_perception/test/test_lidar_
boundary.py's own convention: static/synthetic only, no live hardware, no
rclpy Node instantiation required (every symbol under test is a plain
function/class with zero rclpy dependency).

Real per-frame tracking pass: TestMergeOrAddObject (the old per-detection
greedy merge) is replaced by TestUpdateTracksBatch below -- merge_or_add_
object() itself is gone (see semantic_layer.py's own module docstring).
SemanticObject gained predict()/mark_missed() and update()'s own signature
grew (stamp_sec, for the velocity estimate) -- TestSemanticObject covers
both the unchanged parts (position EMA, latest-not-blended score) and the
new ones (velocity EMA, hit/miss streaks, confirm lifecycle).

Run standalone: python3 -m pytest test/test_semantic_layer.py -v
"""

import itertools
import math
from types import SimpleNamespace

import pytest

from f1tenth_costmap.semantic_layer import (
    SemanticObject,
    compose_base_link_to_map,
    find_nearest_pose_by_stamp,
    pose_to_xytheta,
    update_tracks_batch,
)


def _quat(yaw: float):
    """Identity roll/pitch, given yaw only -- same planar-quaternion shape
    every test in this file needs."""
    return SimpleNamespace(x=0.0, y=0.0, z=math.sin(yaw / 2.0), w=math.cos(yaw / 2.0))


def _counter():
    """Fresh next_track_id callable for one test -- a plain itertools.count(),
    matching what semantic_layer_node.py's own _next_track_id wraps live."""
    c = itertools.count()
    return lambda: next(c)


# ==============================================================================
# pose_to_xytheta
# ==============================================================================

class TestPoseToXytheta:

    def test_identity_pose(self):
        position = SimpleNamespace(x=1.5, y=-2.5, z=0.0)
        orientation = _quat(0.0)
        x, y, yaw = pose_to_xytheta(position, orientation)
        assert (x, y) == pytest.approx((1.5, -2.5))
        assert yaw == pytest.approx(0.0, abs=1e-9)

    def test_extracts_yaw_from_quaternion(self):
        position = SimpleNamespace(x=0.0, y=0.0, z=0.0)
        orientation = _quat(math.radians(90.0))
        _, _, yaw = pose_to_xytheta(position, orientation)
        assert yaw == pytest.approx(math.radians(90.0))

    def test_negative_yaw(self):
        orientation = _quat(math.radians(-45.0))
        _, _, yaw = pose_to_xytheta(SimpleNamespace(x=0.0, y=0.0, z=0.0), orientation)
        assert yaw == pytest.approx(math.radians(-45.0))


# ==============================================================================
# compose_base_link_to_map
# ==============================================================================

class TestComposeBaseLinkToMap:

    def test_robot_at_map_origin_no_rotation_is_identity(self):
        x_map, y_map = compose_base_link_to_map(2.0, 3.0, (0.0, 0.0, 0.0))
        assert (x_map, y_map) == pytest.approx((2.0, 3.0))

    def test_translation_only(self):
        # Robot sits at (5, 5) in map frame, facing +X (yaw=0) -- a point
        # 1m directly ahead of it (base_link x=1, y=0) should land at (6, 5).
        x_map, y_map = compose_base_link_to_map(1.0, 0.0, (5.0, 5.0, 0.0))
        assert (x_map, y_map) == pytest.approx((6.0, 5.0))

    def test_rotation_only_90deg(self):
        # Robot at map origin, facing +Y (yaw=90deg) -- a point 1m ahead of
        # it in base_link frame (x=1, y=0) should land at map (0, 1), not
        # (1, 0) -- the robot's own "ahead" now points along map +Y.
        x_map, y_map = compose_base_link_to_map(1.0, 0.0, (0.0, 0.0, math.radians(90.0)))
        assert (x_map, y_map) == pytest.approx((0.0, 1.0), abs=1e-9)

    def test_rotation_and_translation_combined(self):
        # Robot at (10, 0), facing +Y -- a point 2m ahead (base_link x=2,
        # y=0) should land at map (10, 2).
        x_map, y_map = compose_base_link_to_map(2.0, 0.0, (10.0, 0.0, math.radians(90.0)))
        assert (x_map, y_map) == pytest.approx((10.0, 2.0), abs=1e-9)

    def test_lateral_offset_respects_left_convention(self):
        # Robot at origin facing +X -- a point 1m to its LEFT (base_link
        # y=+1) should land at map (0, 1), matching the "+y = left" body-
        # frame convention this whole codebase already uses (WallDetection.
        # bearing, etc.).
        x_map, y_map = compose_base_link_to_map(0.0, 1.0, (0.0, 0.0, 0.0))
        assert (x_map, y_map) == pytest.approx((0.0, 1.0), abs=1e-9)


# ==============================================================================
# SemanticObject
# ==============================================================================

class TestSemanticObject:

    def test_seeding_takes_the_raw_value_unsmoothed(self):
        obj = SemanticObject(0, 'cone', 1.0, 2.0, 0.9, stamp_sec=100.0, confirm_hit_count=3)
        assert (obj.x_map, obj.y_map) == pytest.approx((1.0, 2.0))
        assert obj.score == pytest.approx(0.9)
        assert obj.hit_count == 1
        assert obj.hit_streak == 1
        assert obj.miss_streak == 0

    def test_confirm_hit_count_one_is_confirmed_immediately(self):
        obj = SemanticObject(0, 'cone', 0.0, 0.0, 0.9, stamp_sec=0.0, confirm_hit_count=1)
        assert obj.confirmed is True

    def test_confirm_hit_count_above_one_starts_unconfirmed(self):
        obj = SemanticObject(0, 'cone', 0.0, 0.0, 0.9, stamp_sec=0.0, confirm_hit_count=3)
        assert obj.confirmed is False

    def test_becomes_confirmed_after_n_consecutive_hits(self):
        obj = SemanticObject(0, 'cone', 0.0, 0.0, 0.5, stamp_sec=0.0, confirm_hit_count=3)
        obj.update(0.0, 0.0, 0.5, stamp_sec=1.0, alpha=0.3)
        assert obj.confirmed is False  # hit_streak == 2
        obj.update(0.0, 0.0, 0.5, stamp_sec=2.0, alpha=0.3)
        assert obj.confirmed is True  # hit_streak == 3

    def test_update_blends_position_by_alpha(self):
        obj = SemanticObject(0, 'cone', 0.0, 0.0, 0.5, stamp_sec=0.0, confirm_hit_count=3)
        obj.update(10.0, 0.0, 0.5, stamp_sec=1.0, alpha=0.3)
        assert obj.x_map == pytest.approx(3.0)  # 0.3*10 + 0.7*0
        assert obj.hit_count == 2

    def test_update_takes_latest_score_not_blended(self):
        obj = SemanticObject(0, 'cone', 0.0, 0.0, 0.5, stamp_sec=0.0, confirm_hit_count=3)
        obj.update(0.0, 0.0, 0.95, stamp_sec=1.0, alpha=0.3)
        assert obj.score == pytest.approx(0.95)

    def test_update_blends_velocity_from_position_delta_over_dt(self):
        # Moves 2m in x over 1s -> raw velocity (2, 0) m/s; alpha=1.0 takes
        # it unblended, isolating just the raw-velocity-from-delta math.
        obj = SemanticObject(0, 'cone', 0.0, 0.0, 0.5, stamp_sec=0.0, confirm_hit_count=3)
        obj.update(2.0, 0.0, 0.5, stamp_sec=1.0, alpha=1.0)
        assert (obj.vx_map, obj.vy_map) == pytest.approx((2.0, 0.0))

    def test_update_with_nonpositive_dt_does_not_touch_velocity(self):
        obj = SemanticObject(0, 'cone', 0.0, 0.0, 0.5, stamp_sec=5.0, confirm_hit_count=3)
        obj.vx_map, obj.vy_map = 1.0, 1.0
        obj.update(100.0, 100.0, 0.5, stamp_sec=5.0, alpha=1.0)  # dt == 0
        assert (obj.vx_map, obj.vy_map) == pytest.approx((1.0, 1.0))

    def test_hit_resets_miss_streak_and_miss_resets_hit_streak(self):
        obj = SemanticObject(0, 'cone', 0.0, 0.0, 0.5, stamp_sec=0.0, confirm_hit_count=5)
        obj.mark_missed()
        obj.mark_missed()
        assert obj.miss_streak == 2
        obj.update(0.0, 0.0, 0.5, stamp_sec=1.0, alpha=0.3)
        assert obj.miss_streak == 0
        assert obj.hit_streak == 1  # reset to 0 by the 2 misses, then this one hit
        obj.mark_missed()
        assert obj.hit_streak == 0
        assert obj.miss_streak == 1

    def test_predict_extrapolates_by_blended_velocity(self):
        obj = SemanticObject(0, 'cone', 0.0, 0.0, 0.5, stamp_sec=0.0, confirm_hit_count=3)
        obj.vx_map, obj.vy_map = 1.0, 2.0
        x, y = obj.predict(stamp_sec=1.5)
        assert (x, y) == pytest.approx((1.5, 3.0))

    def test_predict_at_same_stamp_is_last_known_position(self):
        obj = SemanticObject(0, 'cone', 3.0, 4.0, 0.5, stamp_sec=10.0, confirm_hit_count=3)
        obj.vx_map, obj.vy_map = 5.0, 5.0
        assert obj.predict(stamp_sec=10.0) == pytest.approx((3.0, 4.0))

    def test_predict_clamps_dt_for_a_long_missed_track(self):
        # Even a stale velocity estimate shouldn't extrapolate arbitrarily
        # far into the future for a track that's been missed a long time --
        # see _MAX_PREDICTION_DT_SEC's own comment.
        obj = SemanticObject(0, 'cone', 0.0, 0.0, 0.5, stamp_sec=0.0, confirm_hit_count=3)
        obj.vx_map = 1.0
        x_at_cap, _ = obj.predict(stamp_sec=2.0)   # exactly at the 2.0s cap
        x_way_later, _ = obj.predict(stamp_sec=500.0)  # way past it
        assert x_way_later == pytest.approx(x_at_cap)


# ==============================================================================
# update_tracks_batch
# ==============================================================================

class TestUpdateTracksBatch:

    def test_empty_tracks_spawns_one_tentative_track_per_detection(self):
        tracks = update_tracks_batch(
            [], [('cone', 1.0, 1.0, 0.8)], stamp_sec=0.0, max_distance_m=0.5,
            alpha=0.3, confirm_hit_count=3, lost_miss_count=5, next_track_id=_counter())
        assert len(tracks) == 1
        assert tracks[0].class_id == 'cone'
        assert tracks[0].confirmed is False  # confirm_hit_count=3, only 1 hit so far

    def test_empty_detections_marks_every_track_missed(self):
        tracks = update_tracks_batch(
            [], [('cone', 1.0, 1.0, 0.8)], stamp_sec=0.0, max_distance_m=0.5,
            alpha=0.3, confirm_hit_count=1, lost_miss_count=5, next_track_id=_counter())
        tracks = update_tracks_batch(
            tracks, [], stamp_sec=1.0, max_distance_m=0.5, alpha=0.3,
            confirm_hit_count=1, lost_miss_count=5, next_track_id=_counter())
        assert len(tracks) == 1
        assert tracks[0].miss_streak == 1

    def test_nearby_same_class_merges_instead_of_spawning(self):
        ids = _counter()
        tracks = update_tracks_batch(
            [], [('cone', 1.0, 1.0, 0.8)], stamp_sec=0.0, max_distance_m=0.5,
            alpha=0.3, confirm_hit_count=1, lost_miss_count=5, next_track_id=ids)
        tracks = update_tracks_batch(
            tracks, [('cone', 1.1, 1.0, 0.8)], stamp_sec=1.0, max_distance_m=0.5,
            alpha=0.3, confirm_hit_count=1, lost_miss_count=5, next_track_id=ids)
        assert len(tracks) == 1
        assert tracks[0].hit_count == 2

    def test_far_same_class_spawns_a_new_track(self):
        ids = _counter()
        tracks = update_tracks_batch(
            [], [('cone', 1.0, 1.0, 0.8)], stamp_sec=0.0, max_distance_m=0.5,
            alpha=0.3, confirm_hit_count=1, lost_miss_count=5, next_track_id=ids)
        tracks = update_tracks_batch(
            tracks, [('cone', 10.0, 10.0, 0.8)], stamp_sec=1.0, max_distance_m=0.5,
            alpha=0.3, confirm_hit_count=1, lost_miss_count=5, next_track_id=ids)
        assert len(tracks) == 2

    def test_nearby_different_class_spawns_a_new_track(self):
        # Same position, different class_id -- must NOT merge across classes
        # (a cone and a person standing in roughly the same spot are still
        # two distinct semantic objects, not one) -- unchanged from the old
        # merge_or_add_object's own guard.
        ids = _counter()
        tracks = update_tracks_batch(
            [], [('cone', 1.0, 1.0, 0.8)], stamp_sec=0.0, max_distance_m=0.5,
            alpha=0.3, confirm_hit_count=1, lost_miss_count=5, next_track_id=ids)
        tracks = update_tracks_batch(
            tracks, [('person', 1.0, 1.0, 0.8)], stamp_sec=1.0, max_distance_m=0.5,
            alpha=0.3, confirm_hit_count=1, lost_miss_count=5, next_track_id=ids)
        assert len(tracks) == 2
        assert {t.class_id for t in tracks} == {'cone', 'person'}

    def test_merge_distance_boundary_is_inclusive(self):
        ids = _counter()
        tracks = update_tracks_batch(
            [], [('cone', 0.0, 0.0, 0.8)], stamp_sec=0.0, max_distance_m=1.0,
            alpha=0.3, confirm_hit_count=1, lost_miss_count=5, next_track_id=ids)
        tracks = update_tracks_batch(
            tracks, [('cone', 1.0, 0.0, 0.8)], stamp_sec=1.0, max_distance_m=1.0,
            alpha=0.3, confirm_hit_count=1, lost_miss_count=5, next_track_id=ids)
        assert len(tracks) == 1

    def test_per_frame_assignment_is_exclusive_one_to_one(self):
        """The concrete structural gap in the old per-detection greedy merge
        (merge_or_add_object): it searched/updated one detection at a time
        with NO notion of "already claimed this frame" -- two detections in
        the SAME batch, both within merge_distance_m of the SAME single
        track, would BOTH merge into it in sequence (silently conflating
        two simultaneous detections into one track, not even the "too many
        objects" symptom, an under-counting bug in the opposite direction).
        Batch (Hungarian) assignment is exclusive by construction: at most
        one detection can be assigned to a given track per frame. (Two
        detections this close together in one message are far more likely
        to be two distinct nearby real objects than one physical object
        double-detected -- Ultralytics already runs NMS per-frame upstream,
        which is specifically what suppresses genuine same-object duplicate
        boxes before they ever reach this layer.)"""
        ids = _counter()
        tracks = update_tracks_batch(
            [], [('cone', 0.0, 0.0, 0.8)], stamp_sec=0.0, max_distance_m=5.0,
            alpha=0.3, confirm_hit_count=1, lost_miss_count=5, next_track_id=ids)

        # Two detections this frame, both within max_distance_m=5.0 of the
        # one existing track -- the nearer one (1.0) should claim it; the
        # farther one (2.0) must spawn a NEW track, not also merge in.
        tracks = update_tracks_batch(
            tracks, [('cone', 1.0, 0.0, 0.8), ('cone', 2.0, 0.0, 0.8)], stamp_sec=1.0,
            max_distance_m=5.0, alpha=0.3, confirm_hit_count=1, lost_miss_count=5,
            next_track_id=ids)

        assert len(tracks) == 2
        matched = next(t for t in tracks if t.hit_count == 2)
        spawned = next(t for t in tracks if t.hit_count == 1)
        assert matched.x_map == pytest.approx(0.3)  # 0.3*1.0 + 0.7*0.0 -- claimed the nearer one
        assert spawned.x_map == pytest.approx(2.0)   # the farther one spawned fresh, unblended

    def test_matches_against_predicted_not_last_raw_position(self):
        """A track moving at a steady velocity must keep matching as ONE
        track, not spawn a duplicate every frame -- the "moving-object"
        scenario from this pass's own verify section, as a pure-function
        unit test. Without predict()-based matching (i.e. gating on
        distance from the last RAW position instead), a fast-enough-moving
        object (or, as here, a longer gap between two matched frames) would
        outrun max_distance_m and spawn a new track even though it never
        actually left continuous view."""
        ids = _counter()
        tracks = update_tracks_batch(
            [], [('cone', 0.0, 0.0, 0.8)], stamp_sec=0.0, max_distance_m=0.5,
            alpha=1.0, confirm_hit_count=1, lost_miss_count=5, next_track_id=ids)
        # First real (gate-passing) update establishes a velocity estimate:
        # moved 0.4m in 1.0s -> 0.4 m/s. (0.4 <= max_distance_m=0.5, so this
        # step matches on raw position too -- it doesn't yet distinguish
        # predicted from raw, it's just how the track LEARNS its velocity.)
        tracks = update_tracks_batch(
            tracks, [('cone', 0.4, 0.0, 0.8)], stamp_sec=1.0, max_distance_m=0.5,
            alpha=1.0, confirm_hit_count=1, lost_miss_count=5, next_track_id=ids)
        assert len(tracks) == 1
        assert tracks[0].vx_map == pytest.approx(0.4)

        # Next detection is 2.0s later (a longer gap), continuing at the
        # same 0.4 m/s: 0.4 + 0.4*2.0 = 1.2. Distance from the track's last
        # RAW position (0.4) is 0.8 -- OVER max_distance_m=0.5, would spawn
        # a spurious new track if matching were raw-position-based. Distance
        # from the track's PREDICTED position (0.4 + 0.4*2.0 = 1.2) is 0.0
        # -- an easy match.
        tracks = update_tracks_batch(
            tracks, [('cone', 1.2, 0.0, 0.8)], stamp_sec=3.0, max_distance_m=0.5,
            alpha=1.0, confirm_hit_count=1, lost_miss_count=5, next_track_id=ids)
        assert len(tracks) == 1
        assert tracks[0].hit_count == 3

    def test_lost_after_m_consecutive_misses(self):
        ids = _counter()
        tracks = update_tracks_batch(
            [], [('cone', 0.0, 0.0, 0.8)], stamp_sec=0.0, max_distance_m=0.5,
            alpha=0.3, confirm_hit_count=1, lost_miss_count=2, next_track_id=ids)
        tracks = update_tracks_batch(
            tracks, [], stamp_sec=1.0, max_distance_m=0.5, alpha=0.3,
            confirm_hit_count=1, lost_miss_count=2, next_track_id=ids)
        assert len(tracks) == 1  # 1 miss, lost_miss_count=2 not reached yet
        tracks = update_tracks_batch(
            tracks, [], stamp_sec=2.0, max_distance_m=0.5, alpha=0.3,
            confirm_hit_count=1, lost_miss_count=2, next_track_id=ids)
        assert len(tracks) == 0  # 2 consecutive misses -- pruned

    def test_one_off_false_detection_never_confirms_and_gets_pruned(self):
        """confirm_hit_count > 1: a single spurious detection spawns
        tentative (unconfirmed -- see SemanticLayerNode._publish(), which
        only emits markers for confirmed tracks) and, since nothing ever
        matches it again, is pruned by lost_miss_count without ever
        becoming a visible marker -- this is the actual fix for "one-off
        false detections becoming a lingering phantom object"."""
        ids = _counter()
        tracks = update_tracks_batch(
            [], [('cone', 0.0, 0.0, 0.8)], stamp_sec=0.0, max_distance_m=0.5,
            alpha=0.3, confirm_hit_count=3, lost_miss_count=2, next_track_id=ids)
        assert tracks[0].confirmed is False
        tracks = update_tracks_batch(
            tracks, [], stamp_sec=1.0, max_distance_m=0.5, alpha=0.3,
            confirm_hit_count=3, lost_miss_count=2, next_track_id=ids)
        tracks = update_tracks_batch(
            tracks, [], stamp_sec=2.0, max_distance_m=0.5, alpha=0.3,
            confirm_hit_count=3, lost_miss_count=2, next_track_id=ids)
        assert len(tracks) == 0

    def test_hit_streak_must_be_consecutive_to_confirm(self):
        """A track that alternates hit/miss forever must never confirm on
        total hit_count alone -- only CONSECUTIVE hits count (hit_streak,
        reset by every miss)."""
        ids = _counter()
        tracks = update_tracks_batch(
            [], [('cone', 0.0, 0.0, 0.8)], stamp_sec=0.0, max_distance_m=0.5,
            alpha=0.3, confirm_hit_count=3, lost_miss_count=100, next_track_id=ids)
        for t in range(1, 9):
            dets = [('cone', 0.0, 0.0, 0.8)] if t % 2 == 1 else []
            tracks = update_tracks_batch(
                tracks, dets, stamp_sec=float(t), max_distance_m=0.5, alpha=0.3,
                confirm_hit_count=3, lost_miss_count=100, next_track_id=ids)
        assert len(tracks) == 1
        assert tracks[0].hit_count >= 3  # plenty of total hits by now...
        assert tracks[0].confirmed is False  # ...but never 3 IN A ROW

    def test_track_id_stable_across_pruning_of_an_earlier_track(self):
        """Regression guard for the old list-index-derived marker id scheme
        this pass replaced: pruning track A must not change track B's own
        identity (track_id), even though B's position in the underlying
        list shifts left by one once A is removed."""
        ids = _counter()
        tracks = update_tracks_batch(
            [], [('cone', 0.0, 0.0, 0.8), ('cone', 20.0, 0.0, 0.8)], stamp_sec=0.0,
            max_distance_m=0.5, alpha=0.3, confirm_hit_count=1, lost_miss_count=1,
            next_track_id=ids)
        track_a_id = next(t.track_id for t in tracks if t.x_map == pytest.approx(0.0))
        track_b_id = next(t.track_id for t in tracks if t.x_map == pytest.approx(20.0))

        # Only re-detect B this frame -- A misses once and (lost_miss_count=1) is pruned.
        tracks = update_tracks_batch(
            tracks, [('cone', 20.0, 0.0, 0.8)], stamp_sec=1.0, max_distance_m=0.5,
            alpha=0.3, confirm_hit_count=1, lost_miss_count=1, next_track_id=ids)

        assert len(tracks) == 1
        assert tracks[0].track_id == track_b_id
        assert tracks[0].track_id != track_a_id


# ==============================================================================
# find_nearest_pose_by_stamp
# ==============================================================================

class TestFindNearestPoseByStamp:

    def test_empty_history_returns_none(self):
        assert find_nearest_pose_by_stamp([], 100.0) is None

    def test_picks_the_closest_stamp(self):
        history = [(10.0, (1.0, 0.0, 0.0)), (20.0, (2.0, 0.0, 0.0)), (30.0, (3.0, 0.0, 0.0))]
        assert find_nearest_pose_by_stamp(history, 21.0) == (2.0, 0.0, 0.0)
        assert find_nearest_pose_by_stamp(history, 14.0) == (1.0, 0.0, 0.0)

    def test_exact_match(self):
        history = [(10.0, (1.0, 0.0, 0.0)), (20.0, (2.0, 0.0, 0.0))]
        assert find_nearest_pose_by_stamp(history, 20.0) == (2.0, 0.0, 0.0)

    def test_target_before_all_history_picks_earliest(self):
        history = [(10.0, (1.0, 0.0, 0.0)), (20.0, (2.0, 0.0, 0.0))]
        assert find_nearest_pose_by_stamp(history, 0.0) == (1.0, 0.0, 0.0)

    def test_target_after_all_history_picks_latest(self):
        history = [(10.0, (1.0, 0.0, 0.0)), (20.0, (2.0, 0.0, 0.0))]
        assert find_nearest_pose_by_stamp(history, 100.0) == (2.0, 0.0, 0.0)


if __name__ == '__main__':
    import sys
    sys.exit(pytest.main([__file__, '-v']))
