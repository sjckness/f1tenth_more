"""semantic_doors.py: the geometry, the two liveness tests, and the tracker.

Everything here is BASE_LINK with the laser at the origin, so `to_base_link`
is the identity and the synthetic scan is a plain ray cast against the walls
(cast_scan below). What the node adds on top -- the map frame, the pose
history, the real laser offset -- is semantic_layer_node's, and is covered by
the replay (tools/semantic_door_replay.py), not here.

The scene every case starts from is ROOM: a 6 m wall along y = 3 m with a
0.90 m opening in it, the car at the origin looking at it. Each test breaks
exactly one thing about that scene and asserts the code it earns.
"""

import math
import os
import sys

import numpy as np
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from f1tenth_costmap import semantic_doors as sd  # noqa: E402


# ---------------------------------------------------------------------------
# A synthetic scan: ray casting against segments
# ---------------------------------------------------------------------------
HALF_FOV = math.radians(120.0)
N_BEAMS = 1081
RANGE_MAX = 25.0


def _ray_segment(origin, direction, a, b):
    """Range along `direction` at which the ray hits segment a->b, or inf."""
    ox, oy = origin
    dx, dy = direction
    ex, ey = b[0] - a[0], b[1] - a[1]
    den = dx * ey - dy * ex
    if abs(den) < 1e-12:
        return math.inf
    qx, qy = a[0] - ox, a[1] - oy
    t = (qx * ey - qy * ex) / den     # along the ray
    u = (qx * dy - qy * dx) / den     # along the segment, 0..1
    if t > 0.0 and 0.0 <= u <= 1.0:
        return t
    return math.inf


def cast_scan(segments, laser=(0.0, 0.0), n=N_BEAMS, half_fov=HALF_FOV,
              range_max=RANGE_MAX):
    """A ScanView of `segments` (base_link) seen from `laser`."""
    bearings = np.linspace(-half_fov, half_fov, n)
    ranges = np.full(n, np.inf)
    for i, th in enumerate(bearings):
        d = (math.cos(th), math.sin(th))
        r = min((_ray_segment(laser, d, a, b) for a, b in segments),
                default=math.inf)
        ranges[i] = r if r <= range_max else math.inf
    return sd.ScanView(bearings, ranges, laser, range_max)


def wall(wid, p0, p1, level=sd.LEVEL_LIDAR_CONFIRMED):
    return sd.Wall(wid, p0, p1, level=level)


# The room: one 6 m wall at y = 3, a 0.90 m opening centred on x = 0.
GAP = 0.90
LEFT = ((-3.0, 3.0), (-GAP / 2.0, 3.0))
RIGHT = ((GAP / 2.0, 3.0), (3.0, 3.0))


def room_walls():
    return [wall(1, *LEFT), wall(2, *RIGHT)]


def room_scan(extra=()):
    """The scan of the room; `extra` adds segments (something in the gap)."""
    return cast_scan([LEFT, RIGHT] + list(extra))


def params(**kw):
    return sd.DoorParams(**kw)


def doors_of(cands):
    return [c for c in cands if c.ok]


def only(cands, case=None):
    """The one candidate of `case` with the smallest width -- the gap itself."""
    pool = [c for c in cands if case is None or c.case == case]
    assert pool, 'no candidate at all'
    return min(pool, key=lambda c: c.width)


# ---------------------------------------------------------------------------
# Case A -- two collinear walls
# ---------------------------------------------------------------------------
def test_case_a_is_a_door():
    cands = sd.candidate_doors(room_walls(), room_scan(), [], params())
    doors = doors_of(cands)
    assert len(doors) == 1
    door = doors[0]
    assert door.case == 'A'
    assert door.width == pytest.approx(GAP, abs=1e-6)
    assert door.centre == pytest.approx((0.0, 3.0), abs=1e-6)
    assert door.n_beams >= params().min_beams
    assert door.through_frac == pytest.approx(1.0)


def test_case_a_needs_the_walls_collinear():
    """A corner is not a doorway: same gap, one wall turned 30 degrees."""
    turned = ((GAP / 2.0, 3.0), (GAP / 2.0 + 3.0 * math.cos(math.radians(30.0)),
                                 3.0 + 3.0 * math.sin(math.radians(30.0))))
    walls = [wall(1, *LEFT), wall(2, *turned)]
    cands = sd.candidate_doors(walls, cast_scan([LEFT, turned]), [], params())
    assert not doors_of(cands)
    assert only(cands, 'A').code == 'angle'


def test_case_a_needs_the_walls_on_one_line():
    """Collinear in direction, but one wall is 0.4 m further out: 'offset'."""
    offset = ((GAP / 2.0, 3.4), (3.0, 3.4))
    walls = [wall(1, *LEFT), wall(2, *offset)]
    cands = sd.candidate_doors(walls, cast_scan([LEFT, offset]), [], params())
    assert not doors_of(cands)
    assert only(cands, 'A').code == 'offset'


# ---------------------------------------------------------------------------
# Case B -- a wall end against a perpendicular wall
# ---------------------------------------------------------------------------
#: A corridor mouth: a wall along y = 3 ending at x = -0.45, and a return
#: wall running away from the car at x = +0.45. The opening is the 0.90 m
#: from the first wall's end, perpendicular, to the second wall's plane.
B_WALL = ((-3.0, 3.0), (-GAP / 2.0, 3.0))
B_RETURN = ((GAP / 2.0, 1.0), (GAP / 2.0, 5.0))


def test_case_b_is_a_door():
    walls = [wall(1, *B_WALL), wall(2, *B_RETURN)]
    cands = sd.candidate_doors(walls, cast_scan([B_WALL, B_RETURN]), [], params())
    doors = doors_of(cands)
    assert [d.case for d in doors] == ['B']
    door = doors[0]
    assert door.width == pytest.approx(GAP, abs=1e-6)
    assert door.jamb0 == pytest.approx((-GAP / 2.0, 3.0), abs=1e-6)
    assert door.jamb1 == pytest.approx((GAP / 2.0, 3.0), abs=1e-6)


def test_case_b_rejects_a_corner():
    """The foot at the other wall's own END is a corner, not an opening."""
    short_return = ((GAP / 2.0, 3.0), (GAP / 2.0, 5.0))
    walls = [wall(1, *B_WALL), wall(2, *short_return)]
    cands = sd.candidate_doors(walls, cast_scan([B_WALL, short_return]), [], params())
    assert not doors_of(cands)
    assert 'foot_outside' in {c.code for c in cands if c.case == 'B'}


# ---------------------------------------------------------------------------
# The rejections the task names
# ---------------------------------------------------------------------------
def test_occluded_gap_is_rejected():
    """The wall is UNBROKEN; a nearer box hides the stretch the gap sits in.

    This is the shadow the tracker cannot see past: it has no returns between
    x = -0.45 and x = +0.45, so it holds two tracks ending exactly where a
    doorway's jambs would be -- the same wall pair every other case here
    starts from. What tells the two scenes apart is the scan, and only the
    scan: a beam just inside the gap comes back at 1.5 m, far nearer than the
    3.0 m jamb it is supposed to be beside.
    """
    whole_wall = ((-3.0, 3.0), (3.0, 3.0))
    box = ((-0.225, 1.5), (0.225, 1.5))
    scan = cast_scan([whole_wall, box])
    cands = sd.candidate_doors(room_walls(), scan, [], params())
    assert not doors_of(cands)
    assert only(cands, 'A').code == 'end_occluded'


def test_a_wall_end_at_the_edge_of_the_scan_is_not_a_jamb():
    """Same geometry, a scan that only just reaches the right jamb."""
    narrow = cast_scan([LEFT, RIGHT], half_fov=math.atan2(GAP / 2.0, 3.0) + 1e-3)
    cands = sd.candidate_doors(room_walls(), narrow, [], params())
    assert not doors_of(cands)
    assert only(cands, 'A').code == 'end_fov'


def test_object_in_the_gap_is_rejected():
    """Nothing blocks the beams; a semantic-layer object stands in the gap."""
    cands = sd.candidate_doors(room_walls(), room_scan(), [(0.0, 3.0, 0.25)],
                               params())
    assert not doors_of(cands)
    assert only(cands, 'A').code == 'object'


def test_an_object_beside_the_gap_does_not_reject_it():
    cands = sd.candidate_doors(room_walls(), room_scan(), [(2.0, 1.0, 0.25)],
                               params())
    assert len(doors_of(cands)) == 1


def test_a_lidar_return_on_the_gap_segment_is_rejected():
    """A closed leaf flush in the opening: beams stop ON the line."""
    leaf = ((-GAP / 2.0, 3.0), (GAP / 2.0, 3.0))
    cands = sd.candidate_doors(room_walls(), room_scan([leaf]), [], params())
    assert not doors_of(cands)
    assert only(cands, 'A').code in ('not_through', 'on_segment')


def test_something_just_behind_the_gap_is_not_a_room():
    """A cabinet 0.2 m behind the opening: the beams cross, but not far."""
    back = ((-1.0, 3.2), (1.0, 3.2))
    cands = sd.candidate_doors(room_walls(), room_scan([back]), [], params())
    assert not doors_of(cands)
    assert only(cands, 'A').code == 'not_through'


@pytest.mark.parametrize('gap, code', [(0.40, 'width'), (2.00, 'width'),
                                       (0.90, '')])
def test_width_out_of_range_is_rejected(gap, code):
    left = ((-3.0, 3.0), (-gap / 2.0, 3.0))
    right = ((gap / 2.0, 3.0), (3.0, 3.0))
    walls = [wall(1, *left), wall(2, *right)]
    cands = sd.candidate_doors(walls, cast_scan([left, right]), [], params())
    assert only(cands, 'A').code == code


def test_tentative_walls_are_ignored():
    """The same room, both walls tentative: no pair, so no candidate at all."""
    walls = [wall(1, *LEFT, level=sd.LEVEL_TENTATIVE),
             wall(2, *RIGHT, level=sd.LEVEL_TENTATIVE)]
    assert sd.candidate_doors(walls, room_scan(), [], params()) == []
    # One tentative wall is enough to remove the pair.
    walls = [wall(1, *LEFT), wall(2, *RIGHT, level=sd.LEVEL_TENTATIVE)]
    assert sd.candidate_doors(walls, room_scan(), [], params()) == []
    # Lowering the level brings the same scene back as a door.
    assert len(doors_of(sd.candidate_doors(
        walls, room_scan(), [], params(min_wall_level=sd.LEVEL_TENTATIVE)))) == 1


def test_a_stereo_only_wall_is_below_the_default_level():
    assert sd.wall_level(sd.STATE_CONFIRMED, sd.SOURCE_STEREO) == sd.LEVEL_CONFIRMED
    assert sd.wall_level(sd.STATE_CONFIRMED, sd.SOURCE_LIDAR) == sd.LEVEL_LIDAR_CONFIRMED
    assert sd.wall_level(sd.STATE_CONFIRMED, sd.SOURCE_BOTH) == sd.LEVEL_STEREO_CHECKED
    assert sd.wall_level(sd.STATE_TENTATIVE, sd.SOURCE_BOTH) == sd.LEVEL_TENTATIVE
    assert sd.wall_level(sd.STATE_COASTING, sd.SOURCE_LIDAR) == sd.LEVEL_LIDAR_CONFIRMED
    stereo_only = [wall(1, *LEFT, level=sd.LEVEL_CONFIRMED),
                   wall(2, *RIGHT, level=sd.LEVEL_CONFIRMED)]
    assert sd.candidate_doors(stereo_only, room_scan(), [], params()) == []


# ---------------------------------------------------------------------------
# The tracker
# ---------------------------------------------------------------------------
def confirm_a_door(tracker, p, robot=(0.0, 0.0), start=0.0):
    """Feed the room until the door confirms; return it."""
    cands = sd.candidate_doors(room_walls(), room_scan(), [], p)
    for i in range(p.confirm_hits):
        tracker.update(cands, robot, start + i * 0.1)
    doors = tracker.confirmed()
    assert len(doors) == 1
    return doors[0]


def test_a_door_confirms_after_five_hits_and_keeps_its_id():
    p = params()
    tracker = sd.DoorTracker(p)
    cands = sd.candidate_doors(room_walls(), room_scan(), [], p)
    for i in range(p.confirm_hits - 1):
        assert tracker.update(cands, (0.0, 0.0), i * 0.1) == []
    doors = tracker.update(cands, (0.0, 0.0), 0.5)
    assert len(doors) == 1
    door = doors[0]
    assert door.hits == p.confirm_hits
    for i in range(10):
        tracker.update(cands, (0.0, 0.0), 0.6 + i * 0.1)
    assert [d.id for d in tracker.confirmed()] == [door.id]


def test_a_confirmed_door_is_kept_after_it_leaves_view():
    p = params()
    tracker = sd.DoorTracker(p)
    door = confirm_a_door(tracker, p)
    for i in range(100):
        tracker.update([], (0.0, 0.0), 10.0 + i)
    assert [d.id for d in tracker.confirmed()] == [door.id]


def test_a_candidate_that_never_confirms_is_dropped():
    p = params(candidate_forget_s=1.0)
    tracker = sd.DoorTracker(p)
    tracker.update(sd.candidate_doors(room_walls(), room_scan(), [], p),
                   (0.0, 0.0), 0.0)
    assert len(tracker.tracks) == 1
    tracker.update([], (0.0, 0.0), 2.0)
    assert tracker.tracks == []


def test_the_crossing_direction_is_fixed_at_first_confirmation():
    """The car drives through; the door frame's x axis must not flip."""
    p = params()
    tracker = sd.DoorTracker(p)
    door = confirm_a_door(tracker, p, robot=(0.0, 0.0))
    # Seen from the origin, the far side is +y.
    assert door.normal == pytest.approx((0.0, 1.0), abs=1e-6)
    yaw_before = door.yaw()
    cands = sd.candidate_doors(room_walls(), room_scan(), [], p)
    for i, robot in enumerate([(0.0, 2.0), (0.0, 3.5), (0.0, 6.0)]):
        tracker.update(cands, robot, 1.0 + i)
        assert tracker.confirmed()[0].normal == pytest.approx((0.0, 1.0), abs=1e-6)
        assert tracker.confirmed()[0].yaw() == pytest.approx(yaw_before, abs=1e-9)


def test_the_direction_points_away_from_the_car_that_confirmed_it():
    """The same door approached from the far side gets the opposite normal."""
    p = params()
    tracker = sd.DoorTracker(p)
    door = confirm_a_door(tracker, p, robot=(0.0, 10.0))
    assert door.normal == pytest.approx((0.0, -1.0), abs=1e-6)


def test_the_pose_is_smoothed_and_the_axis_does_not_collapse():
    """A jittering fit, with p0/p1 swapped on alternate cycles."""
    p = params(smooth_alpha=0.3)
    tracker = sd.DoorTracker(p)
    rng = np.random.default_rng(7)
    for i in range(40):
        jitter = rng.normal(0.0, 0.01, 2)
        j0 = (-GAP / 2.0 + jitter[0], 3.0)
        j1 = (GAP / 2.0 + jitter[1], 3.0)
        cand = sd.DoorCandidate(*((j0, j1) if i % 2 else (j1, j0)), 'A', 1, 2)
        tracker.update([cand], (0.0, 0.0), i * 0.1)
    door = tracker.confirmed()[0]
    assert math.hypot(*door.axis) == pytest.approx(1.0, abs=1e-9)
    assert door.centre == pytest.approx((0.0, 3.0), abs=0.02)
    assert door.width == pytest.approx(GAP, abs=0.02)


def test_association_is_by_centre_distance():
    p = params(assoc_max_m=0.3)
    tracker = sd.DoorTracker(p)
    near = sd.DoorCandidate((-0.45, 3.0), (0.45, 3.0), 'A', 1, 2)
    tracker.update([near], (0.0, 0.0), 0.0)
    # 0.2 m away: the same door.
    tracker.update([sd.DoorCandidate((-0.25, 3.0), (0.65, 3.0), 'A', 1, 2)],
                   (0.0, 0.0), 0.1)
    assert len(tracker.tracks) == 1
    # 2 m away: a second door.
    tracker.update([sd.DoorCandidate((1.55, 3.0), (2.45, 3.0), 'A', 1, 2)],
                   (0.0, 0.0), 0.2)
    assert len(tracker.tracks) == 2
    assert [t.id for t in tracker.tracks] == [1, 2]


def test_nearest_confirmed_picks_the_closer_door():
    p = params()
    tracker = sd.DoorTracker(p)
    far = sd.DoorCandidate((5.55, 3.0), (6.45, 3.0), 'A', 3, 4)
    near = sd.DoorCandidate((-0.45, 3.0), (0.45, 3.0), 'A', 1, 2)
    for i in range(p.confirm_hits):
        tracker.update([near, far], (0.0, 0.0), i * 0.1)
    assert len(tracker.confirmed()) == 2
    assert sd.nearest_confirmed(tracker, (0.0, 0.0)).centre[0] == pytest.approx(0.0)
    assert sd.nearest_confirmed(tracker, (6.0, 0.0)).centre[0] == pytest.approx(6.0)


def test_code_groups():
    assert sd.code_group('') == 'ok'
    assert sd.code_group('width') == 'geometry'
    assert sd.code_group('end_occluded') == 'ends'
    assert sd.code_group('object') == 'passage'
