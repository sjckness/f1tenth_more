"""
Unknown objects (obstacles v2, B1-B3): cell ownership, islands, and the one tracker.

Synthetic LiDAR points in base_link, the car at the map origin facing +x
unless a test moves it. No rclpy.
"""

import itertools
import math

from f1tenth_costmap import semantic_layer as sl
from f1tenth_costmap import unknown_objects as uo

import numpy as np

import pytest

P = uo.UnknownParams()
POSE = (0.0, 0.0, 0.0)
DT = 0.1


def disk_points(cx, cy, radius=0.15, n=24):
    """Return the visible half of a disk's outline, as a LiDAR at the origin sees it."""
    ang = np.linspace(-math.pi / 2, math.pi / 2, n) + math.atan2(cy, cx) + math.pi
    return np.column_stack([cx + radius * np.cos(ang), cy + radius * np.sin(ang)])


def wall_points(x0, y0, x1, y1, step=0.03):
    n = max(2, int(math.hypot(x1 - x0, y1 - y0) / step))
    t = np.linspace(0.0, 1.0, n)
    return np.column_stack([x0 + t * (x1 - x0), y0 + t * (y1 - y0)])


def observe(points, walls=(), objects=(), pose=POSE, p=P):
    view = uo.cell_view(points, pose, p)
    owner = uo.cell_ownership(view, list(walls), list(objects), p)
    return owner, uo.island_candidates(view, owner, pose, p)


class Tracker:
    """The node's per-cycle loop, minus ROS: cells -> candidates -> one track list."""

    def __init__(self):
        self.tracks = []
        self.ids = itertools.count(1)
        self.t = 0.0

    def lidar(self, points, walls=(), pose=POSE):
        objects = [(t.x_map, t.y_map, t.radius()) for t in self.tracks
                   if not t.is_unknown and t.confirmed]
        _owner, cands = observe(points, walls, objects, pose)

        def seen(trk):
            bl = uo.to_base_link([(trk.x_map, trk.y_map)], pose)[0]
            # A scan of `points` only: every beam not at a point is a no-return.
            ang = np.arctan2(points[:, 1], points[:, 0]) if len(points) else np.empty(0)
            rr = np.hypot(points[:, 0], points[:, 1]) if len(points) else np.empty(0)
            bearings = np.concatenate([ang, np.linspace(-math.pi, math.pi, 1441)])
            ranges = np.concatenate([rr, np.full(1441, np.inf)])
            return uo.rays_pass_through(bl, trk.radius(), bearings, ranges, (0.0, 0.0), P)
        self.tracks, stats = sl.update_unknown_batch(
            self.tracks, cands, self.t, P, 0.5, 0.3, lambda: next(self.ids), seen)
        self.tracks, merged = sl.dedupe_tracks(self.tracks, 0.5)
        self.t += DT
        return stats, merged

    def camera(self, detections):
        self.tracks = sl.update_tracks_batch(
            self.tracks, detections, self.t, 0.5, 0.3, 3, 5, lambda: next(self.ids))
        self.tracks, merged = sl.dedupe_tracks(self.tracks, 0.5)
        return merged

    def confirmed_unknown(self):
        return [t for t in self.tracks if t.is_unknown and t.confirmed]


# ==============================================================================
# B1: who owns a cell
# ==============================================================================

class TestOwnership:

    def test_cells_on_a_confirmed_wall_belong_to_the_wall(self):
        wall = ((2.0, -2.0), (2.0, 2.0))
        owner, cands = observe(wall_points(2.0, -2.0, 2.0, 2.0), walls=[wall])
        assert np.count_nonzero(owner == uo.OWNER_WALL) > 50
        assert np.count_nonzero(owner == uo.OWNER_ISLAND) == 0
        assert cands == []

    def test_the_same_wall_without_its_track_is_an_island(self):
        owner, cands = observe(wall_points(2.0, -2.0, 2.0, 2.0))
        assert np.count_nonzero(owner == uo.OWNER_WALL) == 0
        assert len(cands) == 1

    def test_a_tracked_objects_own_cells_are_freed(self):
        owner, cands = observe(disk_points(1.5, 0.3), objects=[(1.5, 0.3, 0.15)])
        assert np.count_nonzero(owner == uo.OWNER_OBJECT) > 0
        assert np.count_nonzero(owner == uo.OWNER_ISLAND) == 0
        assert cands == []

    def test_the_cars_own_returns_are_never_cells(self):
        own = np.array([[0.3, 0.1], [-0.05, -0.12], [0.41, 0.0]])
        owner, _cands = observe(own)
        assert not owner.any()

    def test_nothing_past_the_range(self):
        owner, cands = observe(disk_points(4.6, 0.0))
        assert not owner.any() and cands == []


# ==============================================================================
# B2: island components -> candidates
# ==============================================================================

class TestCandidates:

    def test_one_cluster_is_one_candidate_with_its_extent(self):
        _owner, cands = observe(disk_points(1.5, 0.0, radius=0.2))
        assert len(cands) == 1
        c = cands[0]
        assert (c.x, c.y) == pytest.approx((1.5 - 0.08, 0.0), abs=0.1)
        assert 0.3 <= max(c.length, c.width) <= 0.5 + 1e-9    # 0.4 m plus one cell
        assert c.n_cells >= 2

    def test_a_persons_two_legs_are_one_candidate(self):
        legs = np.vstack([disk_points(1.5, -0.1, radius=0.05, n=8),
                          disk_points(1.5, 0.1, radius=0.05, n=8)])
        _owner, cands = observe(legs)
        assert len(cands) == 1

    def test_a_single_cell_is_below_min_cells(self):
        _owner, cands = observe(np.array([[1.5, 0.0]]))
        assert cands == []


# ==============================================================================
# B3: the one tracker
# ==============================================================================

class TestTheTracker:

    def test_a_single_noisy_cell_never_becomes_an_object(self):
        trk = Tracker()
        rng = np.random.default_rng(0)
        for _ in range(30):
            # one stray return per scan, somewhere new each time
            trk.lidar(rng.uniform([0.6, -2.0], [3.5, 2.0], size=(1, 2)))
        assert trk.confirmed_unknown() == []

    def test_a_flickering_cluster_is_not_promoted(self):
        trk = Tracker()
        for i in range(30):
            trk.lidar(disk_points(1.5, 0.0) if i % 3 else np.empty((0, 2)))
        assert trk.confirmed_unknown() == []

    def test_a_persistent_cluster_becomes_an_unknown_object_after_persist_s(self):
        trk = Tracker()
        stamps = []
        for i in range(10):
            trk.lidar(disk_points(1.5, 0.0))
            if trk.confirmed_unknown():
                stamps.append(i * DT)
        assert stamps and stamps[0] == pytest.approx(P.persist_s, abs=DT + 1e-9)
        (obj,) = trk.confirmed_unknown()
        assert obj.class_id == uo.UNKNOWN_CLASS
        assert 0.1 <= obj.radius() <= 0.3

    def test_a_yolo_detection_upgrades_the_unknown_in_place(self):
        trk = Tracker()
        for _ in range(8):
            trk.lidar(disk_points(1.5, 0.0))
        (obj,) = trk.confirmed_unknown()
        merged = trk.camera([('person', 1.55, 0.02, 0.9, None, 0.4)])
        assert merged == 0
        assert len(trk.tracks) == 1
        upgraded = trk.tracks[0]
        assert upgraded.track_id == obj.track_id
        assert upgraded.class_id == 'person'
        assert upgraded.upgraded_sec is not None
        assert upgraded.confirmed
        # ...and its LiDAR cluster is absorbed, not reborn as an unknown.
        for _ in range(10):
            stats, merged = trk.lidar(disk_points(1.5, 0.0))
            assert merged == 0
        assert [t.track_id for t in trk.tracks] == [obj.track_id]

    def test_a_recognised_objects_own_cells_never_make_an_unknown(self):
        trk = Tracker()
        for _ in range(3):
            trk.camera([('chair', 1.5, 0.0, 0.9, None, 0.4)])
        assert [t.class_id for t in trk.tracks if t.confirmed] == ['chair']
        for _ in range(10):
            trk.lidar(disk_points(1.5, 0.0))
        assert [t.class_id for t in trk.tracks] == ['chair']

    def test_a_person_walking_away_leaves_no_unknown_behind_within_1_s(self):
        trk = Tracker()
        for _ in range(8):
            trk.lidar(disk_points(1.5, 0.0))
        (obj,) = trk.confirmed_unknown()
        # They step out of the scan entirely: the rays now pass through.
        gone = None
        for i in range(30):
            trk.lidar(np.empty((0, 2)))
            if not any(t.track_id == obj.track_id for t in trk.tracks):
                gone = (i + 1) * DT
                break
        assert gone is not None and gone <= P.decay_s + DT + 1e-9

    def test_a_person_walking_slowly_is_followed_not_duplicated(self):
        trk = Tracker()
        for i in range(25):
            trk.lidar(disk_points(1.5, -0.3 + 0.02 * i))
        assert len(trk.confirmed_unknown()) == 1
        assert len(trk.tracks) == 1

    def test_wall_cells_never_become_unknown_objects(self):
        trk = Tracker()
        wall = ((2.0, -2.0), (2.0, 2.0))
        for _ in range(20):
            trk.lidar(wall_points(2.0, -2.0, 2.0, 2.0), walls=[wall])
        assert trk.tracks == []

    def test_an_unknown_found_beside_a_recognised_track_is_folded_into_it(self):
        """The dedupe: a camera track spawned apart from the unknown it is."""
        trk = Tracker()
        for _ in range(8):
            trk.lidar(disk_points(1.5, 0.0))
        (obj,) = trk.confirmed_unknown()
        recognised = sl.SemanticObject(999, 'cone', 1.6, 0.05, 0.9, trk.t, 1, width=0.3)
        trk.tracks.append(recognised)
        tracks, merged = sl.dedupe_tracks(trk.tracks, 0.5)
        assert merged == 1
        assert [(t.track_id, t.class_id) for t in tracks] == [(obj.track_id, 'cone')]

    def test_two_separate_unknowns_stay_two(self):
        trk = Tracker()
        for _ in range(8):
            trk.lidar(np.vstack([disk_points(1.5, -0.8), disk_points(1.5, 0.8)]))
        assert len(trk.confirmed_unknown()) == 2

    def test_a_camera_frame_never_ticks_an_unknowns_lifecycle(self):
        trk = Tracker()
        for _ in range(8):
            trk.lidar(disk_points(1.5, 0.0))
        for _ in range(20):
            trk.camera([])
        assert len(trk.confirmed_unknown()) == 1


def test_rays_pass_through_tells_seen_through_from_unobserved():
    bearings = np.linspace(-math.pi, math.pi, 1441)
    far = np.full(bearings.size, 5.0)
    observed, passed = uo.rays_pass_through((1.5, 0.0), 0.2, bearings, far, (0.0, 0.0), P)
    assert observed and passed
    hit = np.where(np.abs(bearings) < 0.1, 1.4, 5.0)
    assert uo.rays_pass_through((1.5, 0.0), 0.2, bearings, hit, (0.0, 0.0), P) == (True, False)
    narrow = np.linspace(2.0, 3.0, 100)        # nothing points at it
    assert uo.rays_pass_through((1.5, 0.0), 0.2, narrow, np.full(100, 5.0), (0.0, 0.0),
                                P) == (False, False)


# ==============================================================================
# What the bags taught (D5-D7)
# ==============================================================================

class TestFromTheBags:

    def test_a_cluster_seen_through_a_gap_in_a_wall_is_dropped(self):
        wall = ((2.0, -2.0), (2.0, 2.0))
        beyond = uo.Candidate(3.0, 0.0, 0.3, 0.3, 0.0, 10)
        before = uo.Candidate(1.5, 0.0, 0.3, 0.3, 0.0, 10)
        kept = uo.drop_behind_walls([beyond, before], [wall], (0.0, 0.0))
        assert kept == [before]
        assert uo.drop_behind_walls([beyond], [], (0.0, 0.0)) == [beyond]

    def test_a_stale_recognised_track_owns_nothing_and_never_reaches_the_mpc(self):
        chair = sl.SemanticObject(1, 'chair', 1.5, 0.0, 0.9, 0.0, 1, width=0.4)
        assert uo.fresh_recognised([chair], 1.0, 2.0) == [chair]
        assert uo.fresh_recognised([chair], 5.0, 2.0) == []
        assert uo.obstacles_for_mpc([chair], POSE, 'active', None, 1.0, 2.0) != []
        assert uo.obstacles_for_mpc([chair], POSE, 'active', None, 5.0, 2.0) == []

    def test_a_still_hit_unknown_survives_losing_its_cluster_to_a_neighbour(self):
        trk = Tracker()
        for _ in range(8):
            trk.lidar(disk_points(1.5, 0.0))
        (obj,) = trk.confirmed_unknown()
        view = uo.cell_view(disk_points(1.5, 0.0), POSE, P)
        owner = uo.cell_ownership(view, [], [], P)
        island_xy = view.centres(owner == uo.OWNER_ISLAND)[2]
        assert uo.cells_hit((obj.x_map, obj.y_map), obj.radius(), island_xy, view.resolution)
        # No candidate at all this cycle, but its cells are still hit: alive.
        tracks, stats = sl.update_unknown_batch(
            trk.tracks, [], trk.t + 5.0, P, 0.5, 0.3, lambda: 99,
            lambda t: (True, False, True))
        assert [t.track_id for t in tracks] == [obj.track_id]

    def test_a_second_cluster_of_a_held_object_is_not_a_new_track(self):
        trk = Tracker()
        for _ in range(8):
            trk.lidar(disk_points(1.5, 0.0, radius=0.3))
        n = len(trk.tracks)
        stats, _merged = trk.lidar(np.vstack([disk_points(1.5, 0.0, radius=0.3),
                                              disk_points(1.62, 0.32, radius=0.04, n=6)]))
        assert len(trk.tracks) == n
