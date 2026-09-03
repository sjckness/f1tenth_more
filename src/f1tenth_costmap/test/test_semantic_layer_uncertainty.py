"""Regression tests for the uncertainty-aware tracking added by the 2026-09-01
mission-analysis follow-up (see mission_analysis_2026-09-01.md).

The analysis measured, in detection_3d_node's own output frame (so with both
ego-motion and localisation removed), that frame-to-frame detection
displacement is strongly confidence-dependent:

    confidence >= 0.75 : median 0.033-0.121 m
    confidence <  0.50 : median 0.143-0.249 m   (2-7x)

and that the tracker spawned 5 'person' + 4 'tv' ids in a 21 s run for what
was almost certainly one of each. These tests pin the three mechanisms added
in response, and -- just as importantly -- pin that all three stay OFF by
default so the pure functions remain backward compatible.
"""

import pytest

from f1tenth_costmap.semantic_layer import (
    SemanticObject,
    _effective_alpha,
    _unpack_detection,
    update_tracks_batch,
)


def _ids():
    counter = iter(range(1000))
    return lambda: next(counter)


class TestUnpackDetection:

    def test_four_tuple_is_still_accepted(self):
        assert _unpack_detection(('car', 1.0, 2.0, 0.9)) == ('car', 1.0, 2.0, 0.9, None)

    def test_five_tuple_carries_sigma(self):
        assert _unpack_detection(('car', 1.0, 2.0, 0.9, 0.3)) == ('car', 1.0, 2.0, 0.9, 0.3)


class TestEffectiveAlpha:

    def test_unknown_sigma_means_no_weighting_not_zero_uncertainty(self):
        """sigma=None must leave alpha untouched. Treating 'unknown' as
        'perfectly precise' would make the least-characterised detections the
        most trusted ones -- exactly backwards."""
        assert _effective_alpha(0.3, None, 0.05, 0.25) == 0.3

    def test_disabled_when_sigma_ref_is_zero(self):
        assert _effective_alpha(0.3, 0.5, 0.0, 0.25) == 0.3

    def test_precise_detection_keeps_full_alpha(self):
        assert _effective_alpha(0.3, 0.05, 0.05, 0.25) == pytest.approx(0.3)

    def test_better_than_reference_is_not_boosted_above_alpha(self):
        """scale is capped at 1.0 -- a very precise detection should not get
        MORE than the configured alpha, or the EMA stops smoothing at all."""
        assert _effective_alpha(0.3, 0.001, 0.05, 0.25) == pytest.approx(0.3)

    def test_noisy_detection_is_down_weighted(self):
        # sigma 4x the reference -> quarter weight (still above the floor).
        assert _effective_alpha(0.4, 0.20, 0.05, 0.1) == pytest.approx(0.1)

    def test_floor_prevents_a_track_freezing(self):
        """Driving alpha to ~0 would freeze a track and stop it following a
        genuinely moving object it can only see poorly."""
        assert _effective_alpha(0.4, 100.0, 0.05, 0.25) == pytest.approx(0.1)


class TestConfidenceWeightedBlending:

    def test_noisy_detection_moves_track_less_than_precise_one(self):
        """The headline behaviour: same displacement, different confidence,
        different influence."""
        def final_x(sigma):
            tracks = [SemanticObject(0, 'person', 0.0, 0.0, 0.9, 0.0, 1)]
            update_tracks_batch(
                tracks, [('person', 1.0, 0.0, 0.9, sigma)], 1.0,
                max_distance_m=2.0, alpha=0.5, confirm_hit_count=1,
                lost_miss_count=5, next_track_id=_ids(),
                alpha_sigma_ref_m=0.05, min_alpha_scale=0.05)
            return tracks[0].x_map

        assert final_x(0.05) > final_x(0.5)

    def test_defaults_reproduce_unweighted_behaviour(self):
        tracks = [SemanticObject(0, 'person', 0.0, 0.0, 0.9, 0.0, 1)]
        update_tracks_batch(
            tracks, [('person', 1.0, 0.0, 0.9, 0.5)], 1.0,
            max_distance_m=2.0, alpha=0.5, confirm_hit_count=1,
            lost_miss_count=5, next_track_id=_ids())
        # alpha_sigma_ref_m defaults to 0.0 -> weighting disabled -> plain EMA.
        assert tracks[0].x_map == pytest.approx(0.5)


class TestAssociationGateWidening:

    def _spawned_new_track(self, gate_sigma_scale, sigma, jump):
        tracks = [SemanticObject(7, 'person', 0.0, 0.0, 0.9, 0.0, 1)]
        out = update_tracks_batch(
            tracks, [('person', jump, 0.0, 0.9, sigma)], 1.0,
            max_distance_m=0.3, alpha=0.3, confirm_hit_count=1,
            lost_miss_count=5, next_track_id=_ids(),
            gate_sigma_scale=gate_sigma_scale)
        return len(out) > 1

    def test_fixed_gate_spawns_a_duplicate_on_a_noisy_jump(self):
        """The churn mechanism, reproduced: a 0.45 m jump from a detection
        that is genuinely +-0.25 m noisy exceeds a fixed 0.3 m gate, so the
        tracker treats its own object as a new one."""
        assert self._spawned_new_track(0.0, 0.25, 0.45) is True

    def test_widened_gate_keeps_the_same_track(self):
        assert self._spawned_new_track(2.0, 0.25, 0.45) is False

    def test_widening_does_not_absorb_a_genuinely_distant_object(self):
        """The gate must not become unbounded -- a far detection is still a
        new object, however uncertain it is."""
        assert self._spawned_new_track(2.0, 0.25, 5.0) is True

    def test_precise_detection_keeps_the_tight_gate(self):
        assert self._spawned_new_track(2.0, 0.001, 0.45) is True


class TestConfirmationGrace:

    def test_single_miss_resets_streak_without_grace(self):
        trk = SemanticObject(0, 'person', 0.0, 0.0, 0.9, 0.0, confirm_hit_count=3)
        trk.hit_streak = 2
        trk.mark_missed(confirm_grace_misses=0)
        assert trk.hit_streak == 0
        assert trk.miss_streak == 1

    def test_grace_preserves_progress_toward_confirmation(self):
        """At the measured 35-45% dropout, 3 CONSECUTIVE hits is unlikely even
        for a real object -- which is what produced the observed id churn."""
        trk = SemanticObject(0, 'person', 0.0, 0.0, 0.9, 0.0, confirm_hit_count=3)
        trk.hit_streak = 2
        trk.mark_missed(confirm_grace_misses=2)
        assert trk.hit_streak == 2
        trk.mark_missed(confirm_grace_misses=2)
        assert trk.hit_streak == 2

    def test_grace_is_exceeded_eventually(self):
        trk = SemanticObject(0, 'person', 0.0, 0.0, 0.9, 0.0, confirm_hit_count=3)
        trk.hit_streak = 2
        for _ in range(3):
            trk.mark_missed(confirm_grace_misses=2)
        assert trk.hit_streak == 0

    def test_miss_streak_always_advances_so_pruning_is_unaffected(self):
        """lost_miss_count pruning must not be weakened by the grace window."""
        trk = SemanticObject(0, 'person', 0.0, 0.0, 0.9, 0.0, confirm_hit_count=3)
        for i in range(1, 6):
            trk.mark_missed(confirm_grace_misses=99)
            assert trk.miss_streak == i

    def test_intermittent_object_confirms_with_grace(self):
        """End-to-end: hit, miss, hit, hit against confirm_hit_count=3 --
        confirms with grace, never confirms without it."""
        def confirms(grace):
            tracks = []
            ids = _ids()
            frames = [
                [('person', 0.0, 0.0, 0.9, 0.05)],
                [],
                [('person', 0.0, 0.0, 0.9, 0.05)],
                [('person', 0.0, 0.0, 0.9, 0.05)],
            ]
            for t, dets in enumerate(frames):
                tracks = update_tracks_batch(
                    tracks, dets, float(t), max_distance_m=0.5, alpha=0.3,
                    confirm_hit_count=3, lost_miss_count=5, next_track_id=ids,
                    confirm_grace_misses=grace)
            return any(t.confirmed for t in tracks)

        assert confirms(0) is False
        assert confirms(2) is True


class TestEmptyFrameAging:
    """The 2026-09-02 stale-track fix, from the tracker's side.

    semantic_layer_node ages a track out via consecutive misses, and a frame
    with zero detections is a legitimate miss for every live track. That signal
    only arrives if an (empty) Detection3DArray is actually published for that
    frame -- which, before the fix, it was not: yolo_detector_node emitted no
    mask on a zero-detection frame, so detection_3d_node's 3-way synchronizer
    never fired and the message never existed.

    Replaying run 15-04-45/15-08-12/15-11-29's real frame occupancy through
    update_tracks_batch measured the consequence: a track survived up to 98
    frames (~7 s at 14 Hz) past its object's last sighting, versus exactly
    lost_miss_count once empty frames are delivered. Run 15-11-29 left tracks
    that were never aged out at all.
    """

    @staticmethod
    def _run(include_empty, present_frames=6, empty_frames=12, lost_miss_count=5):
        tracks = []
        ids = _ids()
        last_seen = None
        pruned_at = None
        for i in range(present_frames + empty_frames):
            dets = [('person', 1.0, 0.0, 0.9, 0.05)] if i < present_frames else []
            if not dets and not include_empty:
                continue  # pre-fix: no mask -> no message at all for this frame
            before = {t.track_id for t in tracks}
            tracks = update_tracks_batch(
                tracks, dets, float(i), max_distance_m=0.5, alpha=0.3,
                confirm_hit_count=3, lost_miss_count=lost_miss_count,
                next_track_id=ids, confirm_grace_misses=2)
            now = {t.track_id for t in tracks}
            if dets:
                last_seen = i
            if before - now and pruned_at is None:
                pruned_at = i
        return tracks, last_seen, pruned_at

    def test_track_is_never_aged_out_when_empty_frames_are_dropped(self):
        """Reproduces the bug: the object leaves, but with no empty-frame
        messages the tracker is never told, so the marker persists."""
        tracks, _, pruned_at = self._run(include_empty=False)
        assert pruned_at is None
        assert len(tracks) == 1
        assert tracks[0].miss_streak == 0  # never even registered a miss

    def test_track_ages_out_at_lost_miss_count_when_empty_frames_arrive(self):
        tracks, last_seen, pruned_at = self._run(include_empty=True)
        assert tracks == []
        # Pruned exactly lost_miss_count frames after the last real sighting --
        # the designed behaviour, no earlier and no later.
        assert pruned_at == last_seen + 5

    def test_confirmation_grace_does_not_delay_aging(self):
        """confirm_grace_misses protects hit_streak only; miss_streak must
        still advance on every miss or the two fixes would interact to keep
        stale markers alive."""
        tracks, last_seen, pruned_at = self._run(include_empty=True)
        assert pruned_at == last_seen + 5
