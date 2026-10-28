"""The SCAN-JUMP detector: synthetic range profiles, and grids built from them.

This is the detector that runs (`use_skeleton` false). A doorway here is a
near-far-near step in the range profile, cross-checked against an occupancy
grid in which UNKNOWN means unknown.

Scenes are built as line segments and turned into both inputs the same way
the car produces them -- a scan by ray-casting, a grid by rasterising those
same rays (free along the ray, occupied at the hit). That matters more than
it sounds: it is what makes a room behind a door come out as a RAY FAN with
unknown between the rays, which is the real-world shape that defeated the
medial-axis detector (see output/doors/freespace.md) and the thing these
tests have to keep working.

Run standalone: python3 -m pytest test/test_scan_doors.py -v
"""

import math

import numpy as np
import pytest

from f1tenth_costmap import freespace_doors as fd
from f1tenth_costmap import semantic_doors as sd

RES = 0.05
ORIGIN = (-8.0, -8.0)
CELLS = 400                      # 20 m square, bigger than any window here
RANGE_MAX = 20.0
HALF_FOV = math.radians(135.0)
BEAMS = 1081


# ---------------------------------------------------------------------------
# scenes
# ---------------------------------------------------------------------------
def corridor(doors=(), door_w=0.90, length=10.0, half_w=1.0, back=-2.0):
    """A corridor along +x with openings of `door_w` in the LEFT wall.

    `doors` are the x centres of the openings. The right wall is solid and
    both ends are capped, so the only way out is through a doorway -- which
    is what makes "0 doors" a meaningful answer for the no-door case.
    """
    segs = [((back, -half_w), (length, -half_w)),        # right wall
            ((length, -half_w), (length, half_w)),       # far cap
            ((back, -half_w), (back, half_w))]           # near cap
    cursor = back
    for xc in sorted(doors):
        segs.append(((cursor, half_w), (xc - door_w / 2.0, half_w)))
        cursor = xc + door_w / 2.0
    segs.append(((cursor, half_w), (length, half_w)))
    return segs


def room_behind(xc, depth=3.0, width=3.0, wall_y=1.0):
    """Three walls making a room on the far side of an opening at `xc`."""
    y0, y1 = wall_y, wall_y + depth
    return [((xc - width / 2.0, y1), (xc + width / 2.0, y1)),
            ((xc - width / 2.0, y0), (xc - width / 2.0, y1)),
            ((xc + width / 2.0, y0), (xc + width / 2.0, y1))]


def _ray(origin, direction, segments, range_max):
    best = range_max
    ox, oy = origin
    dx, dy = direction
    for (ax, ay), (bx, by) in segments:
        ex, ey = bx - ax, by - ay
        den = dx * ey - dy * ex
        if abs(den) < 1e-12:
            continue
        qx, qy = ax - ox, ay - oy
        t = (qx * ey - qy * ex) / den
        u = (qx * dy - qy * dx) / den
        if t > 0.0 and 0.0 <= u <= 1.0:
            best = min(best, t)
    return best


def cast(segments, laser=(0.0, 0.0, 0.0), n=BEAMS, half=HALF_FOV,
         range_max=RANGE_MAX):
    """(bearings, ranges) of one scan from `laser` (x, y, yaw)."""
    bearings = np.linspace(-half, half, n)
    ranges = np.array([
        _ray(laser[:2], (math.cos(laser[2] + b), math.sin(laser[2] + b)),
             segments, range_max) for b in bearings])
    return bearings, ranges


def profile(segments, laser=(0.0, 0.0, 0.0), p=None, **cast_kw):
    p = p or fd.FreespaceParams()
    bearings, ranges = cast(segments, laser, **cast_kw)
    return fd.scan_profile(bearings, ranges, 0.05, RANGE_MAX, laser, p)


# ---------------------------------------------------------------------------
# grids, rasterised from the same rays -- so a far room is a RAY FAN
# ---------------------------------------------------------------------------
class Grid:
    """An OccupancyGrid that starts entirely UNKNOWN and is filled by rays."""

    def __init__(self):
        self.a = np.full((CELLS, CELLS), -1, dtype=np.int16)

    def cell(self, x, y):
        return (int(math.floor((y - ORIGIN[1]) / RES)),
                int(math.floor((x - ORIGIN[0]) / RES)))

    def set(self, x, y, value):
        r, c = self.cell(x, y)
        if 0 <= r < CELLS and 0 <= c < CELLS:
            self.a[r, c] = value
        return self

    def rect(self, x0, y0, x1, y1, value):
        r0, c0 = self.cell(x0, y0)
        r1, c1 = self.cell(x1, y1)
        self.a[r0:r1, c0:c1] = value
        return self

    def ray_cast(self, segments, laser=(0.0, 0.0, 0.0), **cast_kw):
        """Mark free along every ray and occupied where it ends. As SLAM does.

        A beam that returns nothing leaves free cells to range_max and marks
        no obstacle; a beam that hits marks the hit cell occupied. Nothing
        else is touched, so everything the beams never crossed stays UNKNOWN
        -- including most of a room behind a doorway.
        """
        bearings, ranges = cast(segments, laser, **cast_kw)
        for b, r in zip(bearings, ranges):
            ux, uy = math.cos(laser[2] + b), math.sin(laser[2] + b)
            for d in np.arange(0.0, max(0.0, r - RES), RES / 2.0):
                self.set(laser[0] + ux * d, laser[1] + uy * d, 0)
            if r < RANGE_MAX:
                for k in (0.0, RES / 2.0):
                    self.set(laser[0] + ux * (r + k), laser[1] + uy * (r + k), 100)
        return self

    def window(self, robot_xy, p=None):
        p = p or fd.FreespaceParams()
        return fd.occupancy_window(self.a.ravel(), CELLS, CELLS, RES,
                                   ORIGIN[0], ORIGIN[1], robot_xy, p)


def detect(segments, laser=(0.0, 0.0, 0.0), p=None, grid=None, **cast_kw):
    """(candidates, accepted) from one scan, with an optional grid check."""
    p = p or fd.FreespaceParams()
    scan = profile(segments, laser, p, **cast_kw)
    window = None if grid is None else grid.window(laser[:2], p)
    cands = fd.scan_candidates(scan, window, laser[:2], p)
    return cands, [c for c in cands if c.ok]


def codes(cands):
    out = {}
    for c in cands:
        if c.code:
            out[c.code] = out.get(c.code, 0) + 1
    return out


def jamb_points(segments, laser=(0.0, 0.0, 0.0), p=None):
    """Where the detector thinks this scene's one opening's jambs are.

    Taken from the scan alone (no grid), so a test can then edit the grid
    AROUND those exact points. Guessing where a jamb "should" be does not
    work: the jamb beam grazes the wall, so A and B sit a decimetre or more
    outside the geometric opening.
    """
    p = p or fd.FreespaceParams()
    cands = fd.scan_candidates(profile(segments, laser, p), None, laser[:2], p)
    ok = [c for c in cands if c.ok]
    assert len(ok) == 1, f'this helper wants exactly one opening, got {len(ok)}'
    return ok[0].jamb0, ok[0].jamb1


def erase(grid, xy, half=0.40):
    """Make every cell within `half` of a map point UNKNOWN again."""
    return grid.rect(xy[0] - half, xy[1] - half, xy[0] + half, xy[1] + half, -1)


# ---------------------------------------------------------------------------
# the scenes the brief asks for
# ---------------------------------------------------------------------------
def test_a_corridor_with_two_side_doors_finds_both():
    """The case the whole rewrite is for: doors in the SIDE wall.

    Neither is ahead of the car, neither has a mapped room behind it, and a
    bearing-limited detector would see neither. The near one is found with
    many beams through it, the far one with few -- which is why
    beyond_beams_min is 2 and not a larger number.
    """
    segs = corridor(doors=(3.0, 7.0))
    grid = Grid().ray_cast(segs)
    _cands, doors = detect(segs, grid=grid)
    assert len(doors) == 2
    xs = sorted(d.centre[0] for d in doors)
    assert xs[0] == pytest.approx(3.0, abs=0.35)
    assert xs[1] == pytest.approx(7.0, abs=0.60)
    for d in doors:
        # The opening is in the wall at y = 1, whatever the detector thinks
        # its width is.
        assert d.centre[1] == pytest.approx(1.0, abs=0.25)
        assert d.case == fd.CASE_FREESPACE
        assert d.n_beams >= fd.FreespaceParams().beyond_beams_min


def test_a_corridor_without_doors_finds_none():
    segs = corridor()
    grid = Grid().ray_cast(segs)
    _cands, doors = detect(segs, grid=grid)
    assert doors == []


def test_a_room_behind_the_door_is_a_ray_fan_and_the_door_is_still_found():
    """The scene the medial-axis detector could not do, stated as a test.

    The room behind the opening is reached only by the beams that went
    through it, so in the grid it is a fan of thin free lines with UNKNOWN
    between them -- asserted here, not assumed. The door is found anyway,
    because the scan is the evidence and the grid is only asked whether
    ANYTHING got through.
    """
    segs = corridor(doors=(3.0,)) + room_behind(3.0)
    grid = Grid().ray_cast(segs)
    window = grid.window((0.0, 0.0))
    # The room's own interior: mostly unknown, with a few thin free lines
    # raked across it. Sampled over the WHOLE room rather than a box in
    # front of the opening, because the beams fan sideways as soon as they
    # are through -- the same geometry check_behind had to learn.
    r0, c0 = window.to_cell(1.6, 1.1)
    r1, c1 = window.to_cell(4.4, 3.9)
    patch_unknown = window.unknown[r0:r1, c0:c1]
    patch_free = window.free[r0:r1, c0:c1]
    assert patch_unknown.mean() > 0.5, 'the scene is not a ray fan at all'
    assert 0 < patch_free.mean() < 0.25, 'a fan of lines, not an open room'

    _cands, doors = detect(segs, grid=grid)
    assert len(doors) == 1
    assert doors[0].centre[0] == pytest.approx(3.0, abs=0.35)


def test_a_half_metre_gap_is_not_a_door():
    segs = corridor(doors=(3.0,), door_w=0.50)
    grid = Grid().ray_cast(segs)
    cands, doors = detect(segs, grid=grid)
    assert doors == []
    assert 'width' in codes(cands)


def test_a_two_metre_gap_is_not_a_door():
    segs = corridor(doors=(3.0,), door_w=2.00)
    grid = Grid().ray_cast(segs)
    _cands, doors = detect(segs, grid=grid)
    assert doors == []


def test_the_width_band_is_what_rejects_them():
    """Widen the band and both gaps become doors, so the two tests above are
    testing the band and not some unrelated gate."""
    wide = fd.FreespaceParams(width_min_detect_m=0.30, width_max_detect_m=3.00,
                              width_min_m=0.30, width_max_m=3.00)
    for gap in (0.50, 2.00):
        segs = corridor(doors=(3.0,), door_w=gap)
        grid = Grid().ray_cast(segs)
        _cands, doors = detect(segs, grid=grid, p=wide)
        assert len(doors) == 1, gap


def test_one_occupied_cell_in_the_gap_still_leaves_a_door():
    """A chair leg in the doorway is a chair leg, not a closed door.

    Both halves have to tolerate it: the scan sees two or three short beams
    inside the opening, and the grid sees an occupied cell on the line
    between the jambs. Neither may veto on its own -- only a CONTINUOUS
    occupied run of occupied_run_veto_m does.
    """
    segs = corridor(doors=(3.0,))
    leg = [((2.97, 1.00), (3.03, 1.04))]
    grid = Grid().ray_cast(segs + leg)
    grid.set(3.0, 1.0, 100)
    _cands, doors = detect(segs + leg, grid=grid)
    assert len(doors) == 1


def test_a_wall_across_the_opening_blocks_it_on_the_grid():
    """The other side of the same coin: a continuous run DOES veto.

    The scan is given the open corridor -- so the scan half accepts -- and
    only the grid is told the opening is bricked up. Without this the
    tolerance above would mean the grid could never say no at all.
    """
    segs = corridor(doors=(3.0,))
    grid = Grid().ray_cast(segs)
    grid.rect(2.75, 0.95, 3.25, 1.05, 100)       # 0.50 m of solid wall
    cands, doors = detect(segs, grid=grid)
    assert doors == []
    assert codes(cands).get('blocked', 0) >= 1


# ---------------------------------------------------------------------------
# the permissive gates: a flag is not a rejection
# ---------------------------------------------------------------------------
def test_an_opening_outside_the_nominal_band_is_flagged_not_dropped():
    segs = corridor(doors=(3.0,), door_w=0.64)
    grid = Grid().ray_cast(segs)
    _cands, doors = detect(segs, grid=grid)
    assert len(doors) == 1
    assert 'marginal' in doors[0].flags


def test_a_missing_jamb_keeps_the_door_with_a_flag():
    """Glass, a corner reflection, a jamb seen edge-on -- all of them.

    The scan is unchanged (so the jamb BEAM is still there); only the grid
    has nothing where one jamb should be, which is exactly what a surface
    the lidar sees but the mapper never accumulated looks like.
    """
    segs = corridor(doors=(3.0,))
    j0, _j1 = jamb_points(segs)
    grid = erase(Grid().ray_cast(segs), j0)      # one jamb, gone from the map
    _cands, doors = detect(segs, grid=grid)
    assert len(doors) == 1
    assert 'one_jamb' in doors[0].flags


def test_both_jambs_missing_is_a_rejection():
    segs = corridor(doors=(3.0,))
    j0, j1 = jamb_points(segs)
    grid = Grid().ray_cast(segs)
    erase(grid, j0)
    erase(grid, j1)
    cands, doors = detect(segs, grid=grid)
    assert doors == []
    assert codes(cands).get('no_jamb', 0) >= 1


def test_no_grid_at_all_still_yields_doors_flagged_no_grid():
    """Before slam_toolbox has published anything, the scan is enough.

    Refusing to see doors until a map exists would make the detector useless
    exactly when a mission is most likely to be starting.
    """
    segs = corridor(doors=(3.0,))
    _cands, doors = detect(segs, grid=None)
    assert len(doors) == 1
    assert 'no_grid' in doors[0].flags


def test_a_few_near_beams_inside_the_opening_do_not_split_it():
    p = fd.FreespaceParams()
    segs = corridor(doors=(3.0,))
    scan = profile(segs + [((2.97, 1.00), (3.03, 1.04))], p=p)
    assert [c for c in fd.scan_candidates(scan, None, (0.0, 0.0), p) if c.ok]


def test_something_filling_the_opening_does_split_it():
    """A panel across most of the doorway: many near beams, in a row."""
    p = fd.FreespaceParams()
    segs = corridor(doors=(3.0,))
    panel = [((2.60, 0.98), (3.40, 0.98))]
    cands = fd.scan_candidates(profile(segs + panel, p=p), None, (0.0, 0.0), p)
    assert [c for c in cands if c.ok] == []


# ---------------------------------------------------------------------------
# assembly
# ---------------------------------------------------------------------------
def test_one_doorway_yields_one_candidate_not_a_cluster():
    """The tracker associates each candidate to at most ONE track, so five
    copies of one door would spawn four spurious tracks."""
    segs = corridor(doors=(3.0,))
    grid = Grid().ray_cast(segs)
    _cands, doors = detect(segs, grid=grid)
    assert len(doors) == 1


def test_merge_keeps_the_best_seen_copy_and_the_union_of_the_flags():
    def cand(x, n_beams, flags):
        return sd.DoorCandidate((x, -0.45), (x, 0.45), fd.CASE_FREESPACE,
                                fd.NO_WALL_ID, fd.NO_WALL_ID,
                                n_beams=n_beams, flags=flags)
    merged = fd.merge_candidates(
        [cand(0.0, 3, ('marginal',)), cand(0.10, 9, ()), cand(5.0, 1, ())], 0.30)
    assert len(merged) == 2
    best = [c for c in merged if abs(c.centre[0] - 0.05) < 0.2][0]
    assert best.n_beams == 9
    assert 'marginal' in best.flags, 'a merge dropped a warning'


def test_a_jump_point_is_published_for_every_discontinuity():
    segs = corridor(doors=(3.0, 7.0))
    p = fd.FreespaceParams()
    pts = fd.jump_points(profile(segs, p=p), p)
    assert pts.shape[1] == 2
    assert pts.shape[0] >= 4, 'two doorways have four jamb edges between them'


# ---------------------------------------------------------------------------
# confirmation: where the car was standing, not how often it looked
# ---------------------------------------------------------------------------
def _hits(tracker, p, segs, xs, grid=None, t0=0.0):
    for i, x in enumerate(xs):
        laser = (float(x), 0.0, 0.0)
        scan = profile(segs, laser, p)
        window = None if grid is None else grid.window(laser[:2], p)
        cands = fd.scan_candidates(scan, window, laser[:2], p)
        tracker.update(cands, laser[:2], t0 + i)
    return tracker


def test_two_hits_from_poses_far_enough_apart_confirm_a_door():
    """Two sightings, a metre of travel between them -- that is the whole bar.

    A metre and not 0.15 m since the measured sweep: see
    test_the_shipped_defaults_require_a_metre_of_travel_and_no_fallback.
    """
    p = fd.FreespaceParams()
    segs = corridor(doors=(3.0,))
    tracker = _hits(sd.DoorTracker(p.door_params()), p, segs, (0.0, 1.0))
    assert len(tracker.confirmed()) == 1


def test_two_hits_from_the_same_place_do_not():
    """A reflection is perfectly repeatable from one standing position.

    Two hits and no movement must NOT confirm -- that is the whole point of
    the pose-spread policy. Three from the same place does, by
    confirm_same_pose_hits, so a stationary car is not stuck forever.
    """
    p = fd.FreespaceParams(confirm_same_pose_hits=3)
    segs = corridor(doors=(3.0,))
    tracker = _hits(sd.DoorTracker(p.door_params()), p, segs, (0.0, 0.0))
    assert tracker.confirmed() == []
    assert len(tracker.tracks) == 1
    _hits(tracker, p, segs, (0.0,), t0=5.0)
    assert len(tracker.confirmed()) == 1


def test_a_tiny_shuffle_is_not_two_poses():
    p = fd.FreespaceParams(confirm_same_pose_hits=0)
    segs = corridor(doors=(3.0,))
    tracker = _hits(sd.DoorTracker(p.door_params()), p, segs, (0.0, 0.02, 0.04))
    assert tracker.confirmed() == [], 'a 4 cm shuffle confirmed a door'


def test_the_crossing_direction_is_fixed_at_confirmation():
    """Still the tracker's job, and still the thing enter_door cannot
    tolerate changing. Confirmed from y < 1, the arrow must point to +y and
    must not turn round when the car is driven past."""
    p = fd.FreespaceParams()
    segs = corridor(doors=(3.0,))
    tracker = _hits(sd.DoorTracker(p.door_params()), p, segs, (0.0, 0.6, 1.2))
    door = tracker.confirmed()[0]
    first = door.normal
    assert first[1] > 0.8, 'the crossing direction does not point through the wall'
    _hits(tracker, p, segs, (2.0, 2.6, 3.2), t0=20.0)
    assert tracker.by_id(door.id).normal == first
    assert len(tracker.tracks) == 1


def test_flags_reach_the_track_so_the_markers_can_show_them():
    p = fd.FreespaceParams()
    segs = corridor(doors=(3.0,))
    tracker = _hits(sd.DoorTracker(p.door_params()), p, segs, (0.0, 1.0))
    assert 'no_grid' in tracker.confirmed()[0].flags


# ---------------------------------------------------------------------------
# the pieces
# ---------------------------------------------------------------------------
def test_smoothing_is_a_median_so_a_jamb_step_does_not_move():
    r = np.array([1.0, 1.0, 1.0, 5.0, 5.0, 5.0])
    out = fd.smooth_ranges(r, 3)
    assert list(out) == [1.0, 1.0, 1.0, 5.0, 5.0, 5.0]
    spike = np.array([1.0, 1.0, 9.0, 1.0, 1.0])
    assert list(fd.smooth_ranges(spike, 3)) == [1.0, 1.0, 1.0, 1.0, 1.0]


def test_a_beam_with_no_return_is_placed_at_range_max_not_dropped():
    p = fd.FreespaceParams(smooth_beams=0)
    bearings = np.array([0.0, 0.5, 1.0, 1.5])
    ranges = np.array([math.inf, 2.0, math.nan, 0.01])
    scan = fd.scan_profile(bearings, ranges, 0.10, 10.0, (0.0, 0.0, 0.0), p)
    assert list(scan.ranges) == [10.0, 2.0, 10.0, 10.0]
    assert list(scan.no_return) == [True, False, True, True]
    assert len(scan) == 4, 'a beam was dropped, which breaks beam order'


def test_points_are_placed_in_the_map_frame():
    p = fd.FreespaceParams(smooth_beams=0)
    scan = fd.scan_profile(np.array([0.0]), np.array([3.0]), 0.1, 10.0,
                           (1.0, 2.0, math.pi / 2.0), p)
    assert scan.points[0] == pytest.approx((1.0, 5.0), abs=1e-6)
    assert scan.laser == (1.0, 2.0)


def test_unknown_is_not_occupied():
    """The single most important thing about this detector's grid reading."""
    g = Grid()
    g.rect(0.0, 0.0, 1.0, 1.0, 0)
    g.rect(1.0, 0.0, 1.05, 1.0, 100)
    w = g.window((0.5, 0.5))
    assert w.is_free(0.5, 0.5)
    assert w.is_occupied(1.02, 0.5)
    assert not w.is_occupied(3.0, 3.0), 'unknown read as an obstacle'
    assert w.is_unknown(3.0, 3.0)


def test_outside_the_window_is_unknown_not_occupied():
    """A door at the edge of the crop must not be vetoed by the crop."""
    g = Grid().rect(-5.0, -5.0, 5.0, 5.0, 0)
    w = g.window((0.0, 0.0), fd.FreespaceParams(window_m=1.0))
    assert not w.is_occupied(6.0, 0.0)
    assert w.is_unknown(6.0, 0.0)


def test_params_refuse_an_unknown_key():
    with pytest.raises(TypeError, match='nonsense'):
        fd.FreespaceParams(nonsense=1.0)


def test_door_params_carries_the_confirmation_policy_through():
    p = fd.FreespaceParams(confirm_pose_spread_m=0.42, confirm_same_pose_hits=7)
    dp = p.door_params()
    assert dp.confirm_pose_spread_m == pytest.approx(0.42)
    assert dp.confirm_same_pose_hits == 7


def test_the_wall_detectors_keep_plain_hit_counting():
    """DoorParams' defaults must leave semantic_doors exactly as it was."""
    dp = sd.DoorParams()
    assert dp.confirm_pose_spread_m == 0.0
    assert dp.confirm_same_pose_hits == 0
    trk = sd.DoorTracker(dp)
    cand = sd.DoorCandidate((0.0, -0.45), (0.0, 0.45), 'A', 1, 2)
    for i in range(dp.confirm_hits):
        trk.update([cand], (0.0, -2.0), float(i))
    assert len(trk.confirmed()) == 1


def test_max_tracks_drops_the_longest_unseen_not_the_youngest():
    """The budget must not fight the detector.

    The obvious ranking -- confirmed first, then most hits -- was measured to
    be the wrong one: a long-lived door with hundreds of hits always
    outranked a young but genuine candidate, so the cap evicted candidates
    BEFORE they could earn strength, and on corridor10m_20261019T131815 a cap
    of 32 cost 10 of the 29 strong doors the uncapped run found. Recency
    first: what the car can currently see survives long enough to earn its
    strength, and the eviction falls on doors long out of view.
    """
    p = fd.FreespaceParams(max_tracks=2, assoc_max_m=0.10)
    tracker = sd.DoorTracker(p.door_params())

    def cand(x):
        return sd.DoorCandidate((x, -0.45), (x, 0.45), fd.CASE_FREESPACE,
                                fd.NO_WALL_ID, fd.NO_WALL_ID)
    # An old, heavily-evidenced door seen many times and then left behind.
    for i in range(20):
        tracker.update([cand(0.0)], (float(i) * 0.2, 0.0), float(i))
    old = tracker.confirmed()[0]
    assert old.hits == 20

    # Two fresh candidates, one hit each, long after.
    tracker.update([cand(5.0), cand(6.0)], (5.5, 0.0), 100.0)
    kept = sorted(round(t.centre[0], 1) for t in tracker.tracks)
    assert len(tracker.tracks) == 2
    assert kept == [5.0, 6.0], 'the budget evicted what the car is looking at'
    assert tracker.by_id(old.id) is None


def test_evidence_still_breaks_ties_between_equally_stale_doors():
    p = fd.FreespaceParams(max_tracks=1, assoc_max_m=0.10)
    tracker = sd.DoorTracker(p.door_params())

    def cand(x):
        return sd.DoorCandidate((x, -0.45), (x, 0.45), fd.CASE_FREESPACE,
                                fd.NO_WALL_ID, fd.NO_WALL_ID)
    # Both last seen at the same instant; one has far more behind it.
    for i in range(6):
        tracker.update([cand(0.0)], (float(i) * 0.3, 0.0), float(i))
    tracker.update([cand(0.0), cand(5.0)], (1.5, 0.0), 6.0)
    assert len(tracker.tracks) == 1
    assert round(tracker.tracks[0].centre[0], 1) == 0.0


def test_the_wall_detectors_are_not_capped():
    """max_tracks 0 is unbounded, which is what semantic_doors keeps: it is
    capped upstream by /perception/walls carrying at most four tracks."""
    assert sd.DoorParams().max_tracks == 0


# ---------------------------------------------------------------------------
# strength: confirmed means publishable, strong means drivable
# ---------------------------------------------------------------------------
def test_the_shipped_defaults_require_a_metre_of_travel_and_no_fallback():
    """The measured sweep's choice, pinned so it cannot drift back.

    confirm_same_pose_hits 0 means A STATIONARY CAR NEVER CONFIRMS A DOOR.
    That is the point, not an oversight: with the fallback at 3, almost
    everything seen three times in 0.3 s confirmed and the pose-spread policy
    barely bit.
    """
    p = fd.FreespaceParams()
    assert p.confirm_pose_spread_m == pytest.approx(1.0)
    assert p.confirm_same_pose_hits == 0
    assert p.strong_min_hits == 5
    assert p.strong_min_spread_m == pytest.approx(1.0)


def test_a_door_becomes_strong_only_after_enough_travel_and_sightings():
    p = fd.FreespaceParams()
    segs = corridor(doors=(3.0,))
    grid = Grid().ray_cast(segs)
    tracker = sd.DoorTracker(p.door_params())

    # Two sightings 1.0 m apart: confirmed, but only two hits.
    _hits(tracker, p, segs, (0.0, 1.0), grid=grid)
    door = tracker.confirmed()[0]
    assert door.hits == 2
    assert door.strong is False, 'confirmed on arrival is not the same as earned'

    # Keep driving: the hit count catches up and it becomes strong.
    _hits(tracker, p, segs, (1.2, 1.4, 1.6, 1.8), grid=grid, t0=10.0)
    door = tracker.by_id(door.id)
    assert door.hits >= p.strong_min_hits
    assert door.pose_spread() >= p.strong_min_spread_m
    assert door.strong is True


def test_a_stationary_car_confirms_nothing_with_the_shipped_defaults():
    p = fd.FreespaceParams()
    segs = corridor(doors=(3.0,))
    tracker = _hits(sd.DoorTracker(p.door_params()), p, segs, (0.0,) * 10)
    assert tracker.confirmed() == []
    assert len(tracker.tracks) == 1, 'the candidate should still be there, faded'


def test_a_flagged_door_is_never_strong_however_often_it_is_seen():
    """'marginal', 'one_jamb' and 'no_grid' all mean look twice.

    Such a door is published and drawn -- it is real evidence -- but a
    mission may not resolve to it, because the thing that is uncertain about
    it is exactly the geometry the car would be driving at.
    """
    p = fd.FreespaceParams()
    segs = corridor(doors=(3.0,))
    # No grid at all, so every candidate carries 'no_grid'.
    tracker = _hits(sd.DoorTracker(p.door_params()), p, segs,
                    (0.0, 0.5, 1.0, 1.5, 2.0, 2.5))
    door = tracker.confirmed()[0]
    assert door.hits >= p.strong_min_hits
    assert door.pose_spread() >= p.strong_min_spread_m
    assert 'no_grid' in door.flags
    assert door.strong is False


def test_the_wall_detectors_publish_every_confirmed_door_strong():
    """Both thresholds zero is their setting, and it must mean "unchanged"."""
    dp = sd.DoorParams()
    assert dp.strong_min_hits == 0
    assert dp.strong_min_spread_m == 0.0
    trk = sd.DoorTracker(dp)
    cand = sd.DoorCandidate((0.0, -0.45), (0.0, 0.45), 'A', 1, 2)
    for i in range(dp.confirm_hits):
        trk.update([cand], (0.0, -2.0), float(i))
    door = trk.confirmed()[0]
    assert door.strong is True, 'a wall door stopped being selectable'


def test_the_marker_alpha_separates_strong_from_merely_confirmed():
    from f1tenth_costmap import semantic_door_markers as sdm

    class Fake:
        case, width, id = 'F', 0.9, 1

        def __init__(self, confirmed, strong, flags=()):
            self.confirmed, self.strong, self.flags = confirmed, strong, flags

    assert sdm.colour_of(Fake(True, True))[3] == pytest.approx(sdm.STRONG_ALPHA)
    assert sdm.colour_of(Fake(True, False))[3] == pytest.approx(sdm.CONFIRMED_ALPHA)
    assert sdm.colour_of(Fake(False, False))[3] == pytest.approx(sdm.CANDIDATE_ALPHA)
    assert sdm.STRONG_ALPHA > sdm.CONFIRMED_ALPHA > sdm.CANDIDATE_ALPHA
    assert 'STRONG' in sdm.label_of(Fake(True, True))
    assert 'STRONG' not in sdm.label_of(Fake(True, False))


def test_the_door_message_carries_strong():
    from f1tenth_messages.msg import Door
    from f1tenth_costmap import semantic_door_markers as sdm
    p = fd.FreespaceParams()
    segs = corridor(doors=(3.0,))
    grid = Grid().ray_cast(segs)
    tracker = _hits(sd.DoorTracker(p.door_params()), p, segs,
                    (0.0, 0.4, 0.8, 1.2, 1.6, 2.0), grid=grid)
    door = tracker.confirmed()[0]
    stamp = Door().header.stamp if hasattr(Door(), 'header') else None
    from builtin_interfaces.msg import Time
    msg = sdm.door_message(door, Door, Time())
    assert msg.strong is door.strong
    assert msg.strong is True
