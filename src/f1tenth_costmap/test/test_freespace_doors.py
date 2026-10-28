"""THE MEDIAL-AXIS detector (skeleton_candidates) on hand-drawn grids.

Still here and still green, but OFF by default: `use_skeleton` ships false
because on the real map this detector finds nothing -- a room behind a door
is a ray fan rather than a region, so "clearance rises on both sides" cannot
hold there. See the module docstring and output/doors/freespace.md. The
geometry is sound wherever the map IS filled in, which is why these tests
stay rather than being deleted with the switch.

test_scan_doors.py covers the detector that actually runs.


Every scene below is an int16 array of OccupancyGrid.data values (-1 unknown,
0 free, 100 occupied) painted by exact cell ranges, so a width assertion is a
real measurement and not a brush artefact: a 0.90 m gap here is eighteen
0.05 m cells, and the detector is expected to report 0.90 to within a cell.

The seven scenes the design has to get right, each with its own test:
  A  room - door - room            -> one door, the right width and axis
  B  an opening beside a corner     -> one door (a wall end against a
                                       perpendicular wall, no wall fitted)
  C  a plain 1.0 m corridor         -> NO door: it never gets wider
  D  a 0.50 m gap                   -> rejected, too narrow
  E  a 2.00 m gap                   -> rejected, too wide
  F  a door with no beams through    -> found, never confirmed
  G  the crossing direction          -> fixed at first confirmation, and still
                                       the same after the car drives through

Run standalone: python3 -m pytest test/test_freespace_doors.py -v
"""

import math
import time

import numpy as np
import pytest

from f1tenth_costmap import freespace_doors as fd
from f1tenth_costmap import semantic_doors as sd

RES = 0.05
ORIGIN = (-6.0, -6.0)
CELLS = 240                      # 12 m x 12 m, comfortably outside any window


class Grid:
    """A 12 m square OccupancyGrid, painted by map-frame rectangles.

    Starts entirely UNKNOWN, which is the honest starting point: a cell is
    free only once something has said so.
    """

    def __init__(self):
        self.a = np.full((CELLS, CELLS), -1, dtype=np.int16)

    def _col(self, x):
        return int(round((x - ORIGIN[0]) / RES))

    def _row(self, y):
        return int(round((y - ORIGIN[1]) / RES))

    def rect(self, x0, y0, x1, y1, value):
        self.a[self._row(y0):self._row(y1), self._col(x0):self._col(x1)] = value
        return self

    def free(self, x0, y0, x1, y1):
        return self.rect(x0, y0, x1, y1, 0)

    def occ(self, x0, y0, x1, y1):
        return self.rect(x0, y0, x1, y1, 100)

    def detect(self, robot_xy, beams=None, p=None):
        """(window, skeleton, candidates) for this grid as seen from robot_xy."""
        p = p or fd.FreespaceParams(use_skeleton=True)
        window = fd.occupancy_window(self.a.ravel(), CELLS, CELLS, RES,
                                     ORIGIN[0], ORIGIN[1], robot_xy, p,
                                     with_clearance=True)
        skel = fd.skeleton(window, p)
        return window, skel, fd.skeleton_candidates(window, skel, beams, robot_xy, p)

    def doors(self, robot_xy, beams=None, p=None):
        return [c for c in self.detect(robot_xy, beams, p)[2] if c.ok]

    def codes(self, robot_xy, beams=None, p=None):
        return sorted(c.code for c in self.detect(robot_xy, beams, p)[2] if not c.ok)


def walled_room():
    """A 8 m x 8 m room, free inside, a 0.05 m wall all the way round."""
    g = Grid().free(-4, -4, 4, 4)
    g.occ(-4.05, -4.05, 4.05, -4.0).occ(-4.05, 4.0, 4.05, 4.05)
    g.occ(-4.05, -4.05, -4.0, 4.05).occ(4.0, -4.05, 4.05, 4.05)
    return g


def room_door_room(gap_m):
    """The room above, split at x = 0 by a wall with a `gap_m` opening at y = 0."""
    g = walled_room()
    g.occ(0.0, -4.0, 0.05, -gap_m / 2.0)
    g.occ(0.0, gap_m / 2.0, 0.05, 4.0)
    return g


def beams_at(laser, targets, reach_m=6.0):
    """Beams from `laser` aimed at each target and extended to `reach_m`.

    Extended rather than stopped at the target because that is what a real
    beam through an open doorway does: it carries on until it hits the far
    wall. A beam that stops ON the threshold is exactly what `beyond_m` is
    there to reject, and `test_beams_stopping_at_the_threshold_do_not_count`
    uses un-extended ones to check that.
    """
    pts = []
    for tx, ty in targets:
        dx, dy = tx - laser[0], ty - laser[1]
        n = math.hypot(dx, dy)
        pts.append((laser[0] + dx / n * reach_m, laser[1] + dy / n * reach_m))
    return fd.ScanBeams(np.array(pts, dtype=float), laser)


def through_the_gap(laser, gap_m=0.90, x=0.0, n=15):
    """Beams fanned across an opening of width `gap_m` centred on (x, 0)."""
    span = gap_m / 2.0 - 0.10
    return beams_at(laser, [(x, y) for y in np.linspace(-span, span, n)])


# ---------------------------------------------------------------------------
# A. room - door - room
# ---------------------------------------------------------------------------
def test_room_door_room_is_one_door_of_the_right_width_and_axis():
    robot = (-2.0, 0.0)
    doors = room_door_room(0.90).doors(robot, through_the_gap(robot))
    assert len(doors) == 1
    door = doors[0]
    # 18 free cells of 0.05 m, measured to within a cell.
    assert door.width == pytest.approx(0.90, abs=RES)
    assert door.centre[0] == pytest.approx(0.025, abs=RES)
    assert door.centre[1] == pytest.approx(0.0, abs=RES)
    # The jamb line runs along y, so the crossing direction runs along x.
    assert abs(door.axis[0]) < 0.08
    assert abs(abs(door.axis[1]) - 1.0) < 0.08
    assert door.case == fd.CASE_FREESPACE
    assert door.n_beams >= fd.FreespaceParams().min_beams


def test_a_door_needs_no_wall_track_of_any_kind():
    """The whole reason this detector exists, stated as a test.

    Nothing in the call chain touches a wall, a MultiWallTrack or
    semantic_doors' wall gates -- the only inputs are the grid and the beams.
    If a future refactor routes this through the wall extractor, this fails.
    """
    robot = (-2.0, 0.0)
    window, skel, cands = room_door_room(0.90).detect(robot, through_the_gap(robot))
    door = [c for c in cands if c.ok][0]
    assert door.wall_id0 == fd.NO_WALL_ID
    assert door.wall_id1 == fd.NO_WALL_ID


# ---------------------------------------------------------------------------
# B. an opening beside a corner
# ---------------------------------------------------------------------------
def corner_opening(gap_m=0.80):
    """A wall along y = 0 ending at x = 1.20, and a perpendicular wall `gap_m` on.

    Case B of the semantic detector, drawn as occupancy: the opening is
    between one wall's END and another wall's FACE, so there is no pair of
    collinear walls anywhere and the gap is still a doorway.
    """
    g = walled_room()
    g.occ(-4.0, 0.0, 1.20, 0.05)
    g.occ(1.20 + gap_m, 0.0, 1.25 + gap_m, 4.0)
    return g


def test_an_opening_beside_a_corner_is_a_door():
    robot = (1.60, -2.0)
    beams = beams_at(robot, [(x, 0.0) for x in np.linspace(1.35, 1.90, 12)])
    doors = corner_opening(0.80).doors(robot, beams)
    assert len(doors) == 1
    door = doors[0]
    assert door.width == pytest.approx(0.80, abs=RES)
    assert door.centre[0] == pytest.approx(1.60, abs=2 * RES)
    assert door.centre[1] == pytest.approx(0.025, abs=2 * RES)
    # Mirror image of scene A: the jamb line runs along x here, so the
    # crossing direction runs along y.
    assert abs(abs(door.axis[0]) - 1.0) < 0.08
    assert abs(door.axis[1]) < 0.08


# ---------------------------------------------------------------------------
# C. a plain corridor
# ---------------------------------------------------------------------------
def test_a_plain_corridor_has_no_door():
    """1.0 m wide end to end: door-width everywhere, and never a bottleneck.

    This is the scene the rise test exists for. Every skeleton cell down the
    middle has 2d = 1.00 m, inside the width band, so a detector that only
    looked at width would report a door every 0.05 m along a corridor the car
    is simply driving down.
    """
    g = Grid().free(-4, -0.5, 4, 0.5)
    g.occ(-4.05, -0.55, 4.05, -0.5).occ(-4.05, 0.5, 4.05, 0.55)
    g.occ(-4.05, -0.55, -4.0, 0.55).occ(4.0, -0.55, 4.05, 0.55)
    robot = (-2.0, 0.0)
    assert g.doors(robot, beams_at(robot, [(3.9, 0.0)])) == []


# ---------------------------------------------------------------------------
# D / E. the width band
# ---------------------------------------------------------------------------
def test_a_half_metre_gap_is_not_a_door():
    robot = (-2.0, 0.0)
    g = room_door_room(0.50)
    assert g.doors(robot, through_the_gap(robot, 0.50)) == []


def test_a_two_metre_gap_is_not_a_door():
    robot = (-2.0, 0.0)
    g = room_door_room(2.00)
    assert g.doors(robot, through_the_gap(robot, 2.00)) == []


def test_the_width_band_is_what_rejects_them():
    """Widen the band and the same two gaps both become doors.

    Without this the two tests above would pass just as well if the gaps were
    being thrown out for some unrelated reason -- a tracking bug, a scan bug,
    a skeleton that never reached them.
    """
    wide = fd.FreespaceParams(use_skeleton=True, width_min_m=0.30, width_max_m=2.40)
    for gap in (0.50, 2.00):
        robot = (-2.0, 0.0)
        doors = room_door_room(gap).doors(robot, through_the_gap(robot, gap), wide)
        assert len(doors) == 1, gap
        assert doors[0].width == pytest.approx(gap, abs=2 * RES), gap


# ---------------------------------------------------------------------------
# F. the live scan confirmation
# ---------------------------------------------------------------------------
def test_a_door_with_no_beams_through_it_is_found_but_never_confirmed():
    """The grid says doorway, the scan says nothing: no door reaches a mission.

    `candidate_doors` still returns it -- with its real geometry and the code
    that says why -- because that is what the debug markers draw. What must
    not happen is a CONFIRMED door, which is the only kind published,
    given a TF, or visible to enter_door.
    """
    robot = (-2.0, 0.0)
    g = room_door_room(0.90)
    elsewhere = beams_at(robot, [(-0.5, 2.0), (-0.5, -2.0)])
    window, skel, cands = g.detect(robot, elsewhere)
    assert [c.code for c in cands] == ['not_beyond']
    assert cands[0].width == pytest.approx(0.90, abs=RES)

    p = fd.FreespaceParams(use_skeleton=True)
    tracker = sd.DoorTracker(p.door_params())
    for i in range(3 * p.confirm_hits):
        tracker.update(cands, robot, float(i))
    assert tracker.confirmed() == []


def test_beams_stopping_at_the_threshold_do_not_count():
    """A dent in a wall is not a hole through it.

    Beams that end ON the opening (not extended past it) are what a shallow
    alcove returns. `beyond_m` is the whole difference, so this is checked
    directly rather than through a second scene.
    """
    robot = (-2.0, 0.0)
    g = room_door_room(0.90)
    stop_short = fd.ScanBeams(
        np.array([(0.0, y) for y in np.linspace(-0.3, 0.3, 15)]), robot)
    assert g.doors(robot, stop_short) == []
    assert g.codes(robot, stop_short) == ['not_beyond']


def test_beams_are_credited_to_the_opening_they_pass_through():
    """A beam crossing the jamb LINE outside the opening is not evidence.

    The jamb line is infinite; the opening is 0.90 m of it. A beam sailing
    past the wall 2 m away crosses that line too, and counting it would
    confirm a door from beams that never went near it.
    """
    p = fd.FreespaceParams(use_skeleton=True)
    j0, j1 = (0.0, -0.45), (0.0, 0.45)
    laser = (-2.0, 0.0)
    through = fd.ScanBeams(np.array([(2.0, 0.0)]), laser)
    beside = fd.ScanBeams(np.array([(2.0, 3.0)]), laser)
    assert fd.beam_hits(j0, j1, through, p) == 1
    assert fd.beam_hits(j0, j1, beside, p) == 0


def test_min_beams_is_the_threshold_not_one_beam():
    p = fd.FreespaceParams(use_skeleton=True)
    robot = (-2.0, 0.0)
    g = room_door_room(0.90)
    few = through_the_gap(robot, 0.90, n=p.min_beams - 1)
    enough = through_the_gap(robot, 0.90, n=p.min_beams)
    assert g.codes(robot, few) == ['few_beams']
    assert len(g.doors(robot, enough)) == 1


def test_no_scan_at_all_confirms_nothing():
    """beams=None is the startup state, not a licence to trust the grid."""
    robot = (-2.0, 0.0)
    g = room_door_room(0.90)
    assert g.doors(robot, None) == []
    assert g.codes(robot, None) == ['not_beyond']


# ---------------------------------------------------------------------------
# unknown space
# ---------------------------------------------------------------------------
def test_unknown_space_is_occupied_so_an_unexplored_doorway_is_not_free():
    """Nothing beyond the opening has been observed: there is no far region.

    The far side being UNKNOWN makes it occupied, so the clearance cannot
    rise there and the bottleneck is a dead end rather than a passage. This
    is the gate that stops the detector inventing a doorway at the frontier
    of a half-built map, which at 0.2 Hz is most of the map most of the time.
    """
    robot = (-2.0, 0.0)
    beams = through_the_gap(robot)

    unexplored = walled_room()
    unexplored.occ(0.0, -4.0, 0.05, -0.45)
    unexplored.occ(0.0, 0.45, 0.05, 4.0)
    unexplored.rect(0.05, -4.0, 4.0, 4.0, -1)   # the far room, never observed
    # Not even a candidate: with the far side occupied the opening is a
    # one-cell-deep alcove, there is no bottleneck BETWEEN two regions to
    # find, and the beams saying otherwise are given no chance to matter.
    _w, _s, cands = unexplored.detect(robot, beams)
    assert cands == []

    # And the SAME grid with the far room observed free is a door -- which is
    # what makes this a test of unknown-is-occupied and not of some unrelated
    # gate rejecting this scene.
    explored = walled_room()
    explored.occ(0.0, -4.0, 0.05, -0.45)
    explored.occ(0.0, 0.45, 0.05, 4.0)
    assert len(explored.doors(robot, beams)) == 1


def test_a_pinhole_in_a_wall_is_closed_by_the_cleanup():
    """One missing cell is a missed ray, not a doorway.

    A 0.05 m hole is far too narrow to pass the width band on its own; what
    the cleanup prevents is the hole JOINING the two rooms' free space, which
    would put a skeleton through it and a bottleneck on that skeleton.
    """
    g = walled_room()
    g.occ(0.0, -4.0, 0.05, 4.0)
    g.free(0.0, -0.025, 0.05, 0.025)       # one cell knocked out of the wall
    robot = (-2.0, 0.0)
    window, _skel, _c = g.detect(robot)
    assert not window.is_free(0.025, 0.0)


# ---------------------------------------------------------------------------
# G. the crossing direction
# ---------------------------------------------------------------------------
def test_the_crossing_direction_is_fixed_at_confirmation_and_survives_crossing():
    """The one thing an enter_door mission cannot tolerate changing.

    The direction is the door frame's x axis. It is set once, when the door
    is confirmed, from the car's side of the opening towards the far side --
    and the car then drives through, which puts it on the far side. A
    direction recomputed from the current pose would flip there, and a
    mission aiming at the door frame's origin with that yaw would be told to
    turn round in the doorway.
    """
    p = fd.FreespaceParams(use_skeleton=True)
    g = room_door_room(0.90)
    tracker = sd.DoorTracker(p.door_params())

    def cycle(x, t):
        robot = (x, 0.0)
        far = 3.0 if x < 0 else -3.0
        beams = beams_at(robot, [(far, y) for y in np.linspace(-0.3, 0.3, 9)])
        _w, _s, cands = g.detect(robot, beams, p)
        tracker.update(cands, robot, float(t))

    for i, x in enumerate((-2.0, -1.5, -1.0)):
        cycle(x, i)
    assert len(tracker.confirmed()) == 1
    door = tracker.confirmed()[0]
    first_normal, first_yaw, door_id = door.normal, door.yaw(), door.id
    # Confirmed from x < 0, so the crossing direction points towards +x.
    assert first_normal[0] > 0.9
    assert abs(math.degrees(first_yaw)) < 10.0

    for i, x in enumerate((1.0, 2.0, 2.5)):
        cycle(x, 10 + i)
    after = tracker.by_id(door_id)
    assert after is not None, 'the door lost its identity after the crossing'
    assert after.normal == first_normal
    assert after.yaw() == first_yaw
    assert len(tracker.tracks) == 1, 'a second track was spawned for the same door'


def test_a_door_is_kept_in_the_map_after_it_leaves_view():
    """forget_unseen_s 0: confirmed is forever, which is what lets a mission
    drive back to a door it saw earlier."""
    p = fd.FreespaceParams(use_skeleton=True)
    tracker = sd.DoorTracker(p.door_params())
    robot = (-2.0, 0.0)
    g = room_door_room(0.90)
    _w, _s, cands = g.detect(robot, through_the_gap(robot), p)
    # Two sightings a metre apart: the shipped confirm policy wants travel,
    # not repetition (confirm_pose_spread_m 1.0, no same-pose fallback).
    for i in range(2):
        tracker.update(cands, (robot[0] + i * 1.0, robot[1]), float(i))
    assert len(tracker.confirmed()) == 1
    for i in range(20):                    # nothing seen for 200 s
        tracker.update([], (20.0, 20.0), 100.0 + 10.0 * i)
    assert len(tracker.confirmed()) == 1


# ---------------------------------------------------------------------------
# the pieces, checked on their own
# ---------------------------------------------------------------------------
def test_code_group_covers_every_code_and_refuses_a_foreign_one():
    assert fd.code_group('') == 'ok'
    for code in fd.SKELETON_CODES:
        assert fd.code_group(code) == 'skeleton'
    for code in fd.SCAN_CODES:
        assert fd.code_group(code) == 'scan'
    for code in fd.GRID_CODES:
        assert fd.code_group(code) == 'grid'
    # semantic_doors' own codes are a DIFFERENT set; silently grouping one of
    # them would send someone to read the wrong half of this module.
    with pytest.raises(ValueError):
        fd.code_group('end_fov')


def test_thinning_reduces_a_block_to_one_cell_wide_and_keeps_it_connected():
    block = np.zeros((21, 41), dtype=bool)
    block[5:16, 5:36] = True
    skel = fd.thin(block)
    assert skel.any()
    assert skel.sum() < block.sum() / 5
    from scipy import ndimage
    _labels, n = ndimage.label(skel, structure=np.ones((3, 3), dtype=bool))
    assert n == 1


def test_thinning_keeps_two_rooms_connected_through_the_doorway():
    """The reason for a topological skeleton rather than a distance ridge."""
    from scipy import ndimage
    robot = (-2.0, 0.0)
    _w, skel, _c = room_door_room(0.90).detect(robot)
    _labels, n = ndimage.label(skel, structure=np.ones((3, 3), dtype=bool))
    assert n == 1


def test_a_window_reaching_past_the_grid_is_filled_occupied():
    """No door at the edge of the map, where there is no map."""
    p = fd.FreespaceParams(use_skeleton=True, window_m=2.0)
    g = Grid().free(-6.0, -6.0, 6.0, 6.0)
    window = fd.occupancy_window(g.a.ravel(), CELLS, CELLS, RES,
                                 ORIGIN[0], ORIGIN[1], (5.5, 0.0), p)
    assert not window.is_free(6.5, 0.0)
    assert window.is_free(5.5, 0.0)


def test_the_jamb_march_lands_on_the_face_not_inside_the_wall():
    """Four bisections, so the two jambs do not both overshoot outward.

    Without the refinement each march stops up to half a cell inside the
    obstacle and the measured width is a whole cell too wide -- which both
    inflates it against the band and spoils the asymmetric check against 2d.
    """
    p = fd.FreespaceParams(use_skeleton=True)
    robot = (-2.0, 0.0)
    window, _s, _c = room_door_room(0.90).detect(robot, None, p)
    up = fd.march_to_jamb(window, (0.025, 0.0), (0.0, 1.0), 2.0)
    down = fd.march_to_jamb(window, (0.025, 0.0), (0.0, -1.0), 2.0)
    assert up[1] == pytest.approx(0.45, abs=0.01)
    assert down[1] == pytest.approx(-0.45, abs=0.01)


def test_one_doorway_yields_one_candidate_not_a_row_of_them():
    """The plateau of equally narrow skeleton cells is reduced to its pinch.

    Without that reduction every cell of the plateau is a candidate, and the
    tracker -- which associates each candidate to at most one track -- spawns
    a track for each, so a single doorway comes out as a row of doors.
    """
    robot = (-2.0, 0.0)
    window, skel, _c = room_door_room(0.90).detect(robot)
    p = fd.FreespaceParams(use_skeleton=True)
    in_band = (skel & (2.0 * window.clearance >= p.width_min_m)
               & (2.0 * window.clearance <= p.width_max_m))
    assert in_band.sum() > 1, 'the scene does not even have a plateau to reduce'
    assert len(fd._bottleneck_cells(window, skel, p)) == 1


def test_the_cycle_fits_the_two_hertz_budget():
    """A SOFT budget check on the default 8 m window, not a benchmark.

    The limit is deliberately loose (250 ms against the 500 ms a 2 Hz tick
    allows) because this runs on whatever CI or laptop happens to execute it,
    and because the real number worth knowing is the Orin's. It is here to
    catch a change that makes the cycle an ORDER slower -- which is what
    happened once already: the first draft of `thin` built an (8, rows, cols)
    stack and an np.roll per sub-iteration and cost 284 ms on its own.
    """
    p = fd.FreespaceParams(use_skeleton=True)
    g = room_door_room(0.90)
    robot = (-2.0, 0.0)
    beams = through_the_gap(robot)
    g.detect(robot, beams, p)                      # warm the import/JIT paths
    t0 = time.perf_counter()
    g.detect(robot, beams, p)
    elapsed_ms = (time.perf_counter() - t0) * 1e3
    assert elapsed_ms < 250.0, f'{elapsed_ms:.0f} ms per cycle'


def test_freespace_params_refuses_an_unknown_key():
    with pytest.raises(TypeError, match='nonsense'):
        fd.FreespaceParams(nonsense=1.0)


def test_door_params_carries_only_the_tracking_knobs():
    """One tracker for all three door sources: the handover is explicit."""
    p = fd.FreespaceParams(confirm_hits=7, assoc_max_m=0.42)
    dp = p.door_params()
    assert dp.confirm_hits == 7
    assert dp.assoc_max_m == pytest.approx(0.42)
    assert dp.smooth_alpha == pytest.approx(p.smooth_alpha)
    assert dp.forget_unseen_s == pytest.approx(p.forget_unseen_s)


# ---------------------------------------------------------------------------
# the two gates the real-grid replay added (output/doors/freespace.md)
# ---------------------------------------------------------------------------
def test_a_jamb_must_be_part_of_something_not_a_two_cell_blob():
    """A chair leg the right distance from a wall is not a doorway.

    The jamb march stops at the FIRST non-free cell and one cell is enough to
    stop it, so every gate before this one passes happily on a pair made of a
    wall and a speck. Found on a real grid, not reasoned: the replay of
    corridor10m_20261019T131815 confirmed a 1.11 m "door" whose far jamb was
    a 2-cell blob -- after the bottleneck, rise, symmetry AND a 12-beam scan
    confirmation had all passed.
    """
    g = walled_room()
    g.occ(0.0, -4.0, 0.05, -0.45)        # a real wall, ending at y = -0.45
    g.occ(0.0, 0.45, 0.05, 0.55)         # and a two-cell blob 0.90 m away
    robot = (-2.0, 0.0)
    beams = through_the_gap(robot)
    assert g.doors(robot, beams) == []
    assert 'thin_jamb' in g.codes(robot, beams)

    # The SAME geometry with a real wall in place of the blob is a door,
    # which is what makes this a test of the gate and not of the scene.
    solid = room_door_room(0.90)
    assert len(solid.doors(robot, beams)) == 1


def test_the_jamb_gate_can_be_turned_off():
    """jamb_min_cells 0 is the pre-gate behaviour, kept reachable on purpose.

    If a real doorway on the car ever comes back as 'thin_jamb' -- a jamb seen
    nearly edge-on, a wall the grid holds as two cells -- this is the knob,
    and it has to be provably the only thing standing in the way.
    """
    g = walled_room()
    g.occ(0.0, -4.0, 0.05, -0.45)
    g.occ(0.0, 0.45, 0.05, 0.55)
    robot = (-2.0, 0.0)
    off = fd.FreespaceParams(use_skeleton=True, jamb_min_cells=0)
    assert len(g.doors(robot, through_the_gap(robot), off)) == 1


def test_a_candidate_is_a_local_minimum_before_the_width_band_is_applied():
    """The band must not be what defines the pinch. See _bottleneck_cells.

    A skeleton that descends through width_min_m into a tight pocket has an
    in-band cluster cut off at exactly width_min_m, and that cut is not a
    local minimum of anything. Selecting it wastes a max_candidates slot --
    which on a real grid meant 22 of 38 clusters were never evaluated at all
    and every one of the 16 that were came back at 2d = 0.700 exactly.
    """
    p = fd.FreespaceParams(use_skeleton=True)
    g = walled_room()
    # A funnel: free space narrowing steadily from 1.6 m down to 0.3 m, so
    # the skeleton passes through the whole width band without ever pinching.
    g.rect(-4.0, -4.0, 4.0, 4.0, 100)
    for i in range(40):
        x0 = 0.0 + i * 0.05
        half = (1.60 - i * 0.0325) / 2.0
        g.free(x0, -half, x0 + 0.05, half)
    g.free(-2.0, -0.80, 0.0, 0.80)
    robot = (-1.0, 0.0)
    window, skel, _c = g.detect(robot, None, p)
    in_band = (skel & (2.0 * window.clearance >= p.width_min_m)
               & (2.0 * window.clearance <= p.width_max_m))
    assert in_band.sum() > 5, 'the funnel does not cross the width band'
    # Nothing in a monotonically narrowing funnel is a local minimum, so
    # nothing is a candidate -- the old band-first selection returned the
    # band's own edge cell here.
    cells = fd._bottleneck_cells(window, skel, p)
    for cell in cells:
        assert window.clearance[cell] == pytest.approx(
            float(window.clearance[skel].min()), abs=1e-6), (
            'a band-edge cell was selected as a pinch')


def test_only_curve_cells_are_offered_as_candidates():
    """A tip has no two sides and a branch point has three.

    Both are rejected as 'junction' by _sides anyway; filtering them out
    first is what stops a bushy real-grid skeleton (814 of 2303 cells were
    branch points on corridor10m_20261019T131815) from spending the whole
    max_candidates budget on cells that can never be doors.
    """
    p = fd.FreespaceParams(use_skeleton=True)
    robot = (-2.0, 0.0)
    window, skel, _c = room_door_room(0.90).detect(robot, None, p)
    deg = fd.neighbour_count(skel)
    for cell in fd._bottleneck_cells(window, skel, p):
        assert deg[cell] == 2, cell
