"""Doors from JAMB BLOBS (jamb_doors.py): glass walls, long walls, small posts.

Scenes are ray-cast like the scan-jump tests (test_scan_doors.py): the door is
in the y = 1 wall, the car looks +y from y = -2 and slides along x, a back
wall at y = 3.5 is what the rays going through the gap end on.

  long wall (left) + small blob (right) + clear gap + free beyond + rays
      -> STRONG, centre stable
  free-space gap with no occupied cells at the edges   -> not a door
  blobs with an obstacle behind the gap                -> not strong
  blobs too close / too far apart                      -> rejected
  enter_door resolves over strong doors only           -> see test_enter_door
"""

import math

import numpy as np
import pytest

from f1tenth_costmap import freespace_doors as fd
from f1tenth_costmap import jamb_doors as jd
from f1tenth_costmap import semantic_doors as sd

from test_scan_doors import Grid, profile, erase, RES


def scene(gap=0.90, wall_to=-3.0, blob_w=0.15, obstacle_y=None, xc=0.0):
    """Long wall on the left of the gap, a small post on the right, glass beyond
    it (the back wall is seen straight through the blob's side)."""
    left_end = xc - gap / 2.0
    right_start = xc + gap / 2.0
    segs = [((wall_to, 1.0), (left_end, 1.0)),                      # long opaque wall
            ((right_start, 1.0), (right_start + blob_w, 1.0)),      # small blob
            ((-4.0, 3.5), (4.0, 3.5))]                              # back wall
    if obstacle_y is not None:
        segs.append(((xc - 1.2, obstacle_y), (xc + 1.2, obstacle_y)))
    return segs


def run(segs, xs, p=None, y=-2.0, tracker=None):
    """Drive the car along x, one detector cycle per position; return the tracker."""
    p = p or fd.FreespaceParams()
    tracker = tracker or sd.DoorTracker(p.door_params())
    grid = Grid()
    for i, x in enumerate(xs):
        laser = (float(x), y, math.pi / 2.0)
        grid.ray_cast(segs, laser)
    for i, x in enumerate(xs):
        laser = (float(x), y, math.pi / 2.0)
        scan = profile(segs, laser, p)
        window = grid.window(laser[:2], p)
        cands = fd.scan_candidates(scan, window, laser[:2], p)
        tracker.update(cands, laser[:2], float(i))
    return tracker, grid


XS = [-0.9 + 0.2 * i for i in range(10)]          # 1.8 m of travel, 10 cycles


def test_long_wall_on_one_side_and_a_small_blob_on_the_other_is_strong():
    tracker, _ = run(scene(), XS)
    assert tracker.tracks, 'the gap between a wall and a blob must be found'
    strong = [t for t in tracker.tracks if t.strong]
    assert len(strong) == 1, [(t.id, t.flags, tracker.first_failing_gate(t))
                              for t in tracker.tracks]
    trk = strong[0]
    assert trk.width == pytest.approx(0.90, abs=0.15)
    # blob positions are cells nearest the gap centre: the gap edges
    assert abs(trk.centre[0] - 0.0) < 0.15 and abs(trk.centre[1] - 1.0) < 0.15
    f, r = sd.track_evidence(trk)
    assert f >= jd.STRIP_FREE_MIN and r >= jd.RAYS_MIN


def test_the_centre_stays_within_5cm_over_50_noisy_updates():
    p = fd.FreespaceParams()
    tracker = sd.DoorTracker(p.door_params())
    rng = np.random.default_rng(7)
    true_c = (2.0, 1.0)
    for i in range(50):
        ja = (true_c[0] - 0.45 + rng.normal(0, 0.03), true_c[1] + rng.normal(0, 0.03))
        jb = (true_c[0] + 0.45 + rng.normal(0, 0.03), true_c[1] + rng.normal(0, 0.03))
        cand = sd.DoorCandidate(ja, jb, fd.CASE_FREESPACE, fd.NO_WALL_ID, fd.NO_WALL_ID)
        cand.blobs = True
        cand.ev = {'f': 1.0, 'r': 12, 'far': (0.0, 1.0)}
        tracker.update([cand], (2.0 + 0.05 * i, -2.0), float(i))
        trk = tracker.tracks[0]
        if i >= jd.JAMB_PERSIST_SCANS:
            assert math.hypot(trk.centre[0] - true_c[0], trk.centre[1] - true_c[1]) < 0.05
    assert trk.strong and len(tracker.tracks) == 1


def test_a_free_space_gap_with_no_occupied_cells_at_its_edges_is_not_a_door():
    p = fd.FreespaceParams()
    segs = scene()
    grid = Grid().ray_cast(segs, (0.0, -2.0, math.pi / 2.0))
    scan = profile(segs, (0.0, -2.0, math.pi / 2.0), p)
    cands = fd.scan_candidates(scan, grid.window((0.0, -2.0), p), (0.0, -2.0), p)
    edges = [(c.jamb0, c.jamb1) for c in cands if c.ok]
    assert edges, 'precondition: the unedited scene has a door'
    for a, b in edges:
        erase(grid, a, half=0.45)
        erase(grid, b, half=0.45)
    cands = fd.scan_candidates(scan, grid.window((0.0, -2.0), p), (0.0, -2.0), p)
    assert not [c for c in cands if c.ok]
    assert any(c.code == 'no_jamb' for c in cands)


def test_blobs_with_an_obstacle_behind_the_gap_are_not_strong():
    tracker, _ = run(scene(obstacle_y=1.6), XS)
    assert tracker.tracks, 'the door is still found, it is just not strong'
    assert not any(t.strong for t in tracker.tracks)
    assert tracker.first_failing_gate(tracker.tracks[0]) == 'strip'


def test_blobs_too_close_or_too_far_are_rejected():
    p = fd.FreespaceParams()
    for gap in (0.35, 2.0):
        segs = scene(gap=gap)
        laser = (0.0, -2.0, math.pi / 2.0)
        grid = Grid().ray_cast(segs, laser)
        cands = fd.scan_candidates(profile(segs, laser, p), grid.window(laser[:2], p),
                                   laser[:2], p)
        assert not [c for c in cands if c.ok], gap


def test_the_strip_counts_blocked_columns_not_cells():
    """A thin wall across the strip is one row of cells but every column."""
    segs = scene(obstacle_y=1.6)
    laser = (0.0, -2.0, math.pi / 2.0)
    grid = Grid().ray_cast(segs, laser)
    window = grid.window(laser[:2], fd.FreespaceParams())
    f, far = jd.strip_free_fraction(window, (-0.45, 1.0), (0.45, 1.0), laser[:2])
    assert f < 0.2 and far[1] > 0.9
    clear = Grid().ray_cast(scene(), laser).window(laser[:2], fd.FreespaceParams())
    f2, _ = jd.strip_free_fraction(clear, (-0.45, 1.0), (0.45, 1.0), laser[:2])
    assert f2 >= 0.95


def test_the_crossing_side_flips_only_on_30_points_better_evidence():
    p = fd.FreespaceParams()
    tracker = sd.DoorTracker(p.door_params())

    def cand(far_y, f, r=12):
        c = sd.DoorCandidate((1.55, 1.0), (2.45, 1.0), fd.CASE_FREESPACE,
                             fd.NO_WALL_ID, fd.NO_WALL_ID)
        c.blobs = True
        c.ev = {'f': f, 'r': r, 'far': (0.0, far_y)}
        return c
    # seen from y < 1: the crossing direction is +y
    for i in range(8):
        tracker.update([cand(+1.0, 0.95)], (2.0 + 0.2 * i, -2.0), float(i))
    trk = tracker.tracks[0]
    assert trk.confirmed and trk.normal[1] > 0.9
    # the other side (-y) looks 0.2 better... not enough
    ev = trk.side_ev
    for i in range(30):
        tracker.update([cand(-1.0, 1.0)], (2.0, 4.0), 10.0 + i)
    assert trk.normal[1] > 0.9 or ev[-1]['f'] - ev[1]['f'] < jd.SIDE_FLIP_MARGIN
    # make the original side poor: the margin is now exceeded and it flips
    trk.side_ev[1]['f'] = 0.5
    tracker.update([cand(-1.0, 1.0)], (2.0, 4.0), 50.0)
    assert trk.normal[1] < -0.9


def test_neither_side_passing_is_not_strong():
    p = fd.FreespaceParams()
    tracker = sd.DoorTracker(p.door_params())
    for i in range(8):
        c = sd.DoorCandidate((1.55, 1.0), (2.45, 1.0), fd.CASE_FREESPACE,
                             fd.NO_WALL_ID, fd.NO_WALL_ID)
        c.blobs = True
        c.ev = {'f': 0.6, 'r': 12, 'far': (0.0, 1.0)}        # strip not free
        tracker.update([c], (2.0 + 0.2 * i, -2.0), float(i))
    trk = tracker.tracks[0]
    assert not trk.strong and tracker.first_failing_gate(trk) == 'strip'


def test_the_label_shows_width_free_fraction_and_rays_and_a_question_mark():
    from f1tenth_costmap import semantic_door_markers as sdm
    p = fd.FreespaceParams()
    tracker = sd.DoorTracker(p.door_params())
    c = sd.DoorCandidate((1.55, 1.0), (2.45, 1.0), fd.CASE_FREESPACE,
                         fd.NO_WALL_ID, fd.NO_WALL_ID)
    c.blobs = True
    c.ev = {'f': 0.95, 'r': 14, 'far': (0.0, 1.0)}
    tracker.update([c], (2.0, -2.0), 0.0)
    trk = tracker.tracks[0]
    label = sdm.label_of(trk)
    assert label.startswith(f'door_{trk.id}? w=0.90 f=0.95 r=14')
