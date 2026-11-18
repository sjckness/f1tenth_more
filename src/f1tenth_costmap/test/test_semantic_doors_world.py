"""The sim world's geometry, judged by the detector that has to find it.

f1tenth_sim/worlds/test_room_doors.sdf exists to give semantic_doors.py one
doorway of each case in Gazebo. That claim is only worth something if it is
checked: nudge a wall by 10 cm and a door silently stops being a door, and
nobody would find out until a sim run came back empty.

So this reads the world file, turns its boxes into the wall lines a LiDAR
would actually fit (the FACE turned toward the car, not the box centre line),
casts a scan from the spawn, and asserts the detector finds exactly the two
doors the world's own header documents.

It SKIPS if the world file is not there: f1tenth_costmap does not depend on
f1tenth_sim and must not start to, so this is a check on the repo when both
are present, not a dependency.
"""

import collections
import math
import os
import sys
import xml.etree.ElementTree as ET

import numpy as np
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from f1tenth_costmap import semantic_doors as sd  # noqa: E402

#: src/f1tenth_costmap/test/<this> -> the repo root.
REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', '..'))
WORLD = os.path.join(REPO, 'src', 'f1tenth_sim', 'worlds', 'test_room_doors.sdf')

#: Where sim_bringup.launch.py spawns the car, and the heading it spawns with.
SPAWN = (0.0, 0.0)

#: The scan the world is judged with: the planar LiDAR's own span, wide
#: enough to see both doorways from the spawn.
HALF_FOV = math.radians(135.0)
N_BEAMS = 1081
RANGE_MAX = 25.0


def _boxes(path):
    """[(name, cx, cy, sx, sy)] of every box collision in the world."""
    out = []
    for model in ET.parse(path).getroot().find('world').findall('model'):
        for link in model.findall('link'):
            for col in link.findall('collision'):
                box = col.find('geometry/box')
                if box is None:
                    continue
                px, py = [float(v) for v in col.find('pose').text.split()[:2]]
                sx, sy = [float(v) for v in box.find('size').text.split()[:2]]
                out.append((f'{model.get("name")}/{col.get("name")}', px, py, sx, sy))
    return out


def _edges(boxes):
    """Each box as its four edge segments, for the ray cast."""
    segs = []
    for _n, cx, cy, sx, sy in boxes:
        hx, hy = sx / 2.0, sy / 2.0
        c = [(cx - hx, cy - hy), (cx + hx, cy - hy), (cx + hx, cy + hy), (cx - hx, cy + hy)]
        segs += [(c[i], c[(i + 1) % 4]) for i in range(4)]
    return segs


def _near_face(index, name, cx, cy, sx, sy, car):
    """The wall LINE the LiDAR fits: the box face turned toward the car.

    This is the whole reason the world's nominal numbers and the detector's
    measured ones differ by the wall thickness, and why the world header
    spells both out.
    """
    if sx >= sy:                                        # the box runs along x
        y = cy - sy / 2.0 if car[1] < cy else cy + sy / 2.0
        return sd.Wall(index, (cx - sx / 2.0, y), (cx + sx / 2.0, y), role='unknown')
    x = cx - sx / 2.0 if car[0] < cx else cx + sx / 2.0
    return sd.Wall(index, (x, cy - sy / 2.0), (x, cy + sy / 2.0), role='unknown')


def _ray(origin, direction, a, b):
    ex, ey = b[0] - a[0], b[1] - a[1]
    den = direction[0] * ey - direction[1] * ex
    if abs(den) < 1e-12:
        return math.inf
    qx, qy = a[0] - origin[0], a[1] - origin[1]
    t = (qx * ey - qy * ex) / den
    u = (qx * direction[1] - qy * direction[0]) / den
    return t if t > 1e-6 and 0.0 <= u <= 1.0 else math.inf


def _scan(segments, car):
    bearings = np.linspace(-HALF_FOV, HALF_FOV, N_BEAMS)
    ranges = np.full(N_BEAMS, np.inf)
    for i, th in enumerate(bearings):
        d = (math.cos(th), math.sin(th))
        r = min((_ray(car, d, a, b) for a, b in segments), default=math.inf)
        if r <= RANGE_MAX:
            ranges[i] = r
    return sd.ScanView(bearings, ranges, (0.0, 0.0), RANGE_MAX)


@pytest.fixture(scope='module')
def doors():
    """The confirmed doors of the world, seen from the spawn."""
    if not os.path.exists(WORLD):
        pytest.skip(f'{WORLD} is not in this checkout')
    boxes = _boxes(WORLD)
    walls = [_near_face(i, *b, SPAWN) for i, b in enumerate(boxes)]
    view = _scan(_edges(boxes), SPAWN)
    params = sd.DoorParams()
    cands = sd.candidate_doors(walls, view, [], params, to_base_link=lambda xy: xy)
    tracker = sd.DoorTracker(params)
    for i in range(params.confirm_hits):
        tracker.update(cands, SPAWN, i * 0.1)
    return tracker, cands


def test_the_world_holds_exactly_two_doors(doors):
    """Not one, not three: the distractors must all be rejected."""
    tracker, cands = doors
    confirmed = tracker.confirmed()
    assert len(confirmed) == 2, (
        'codes: ' + repr(collections.Counter(c.code or 'ok' for c in cands)))
    assert {d.case for d in confirmed} == {'A', 'B'}


def test_door_a_is_the_partition_gap_dead_ahead(doors):
    tracker, _ = doors
    door = next(d for d in tracker.confirmed() if d.case == 'A')
    # The LINE is the partition's near face at x = 2.45, not its centre 2.50.
    assert door.centre == pytest.approx((2.45, 0.0), abs=0.01)
    assert door.width == pytest.approx(0.90, abs=0.01)
    # The crossing direction points away from the spawn, through the opening.
    assert math.degrees(door.yaw()) == pytest.approx(0.0, abs=1.0)


def test_door_b_is_the_wall_end_against_the_perpendicular_wall(doors):
    tracker, _ = doors
    door = next(d for d in tracker.confirmed() if d.case == 'B')
    assert door.centre == pytest.approx((1.40, -3.95), abs=0.01)
    # 0.80 m FACE to face, which is what the world's B2 offset is chosen for.
    assert door.width == pytest.approx(0.80, abs=0.01)
    assert math.degrees(door.yaw()) == pytest.approx(-90.0, abs=1.0)


def test_the_nearest_door_is_the_one_the_example_mission_will_pick(doors):
    """missions/enter_nearest_door.json asks for 'nearest' from the spawn."""
    tracker, _ = doors
    nearest = sd.nearest_confirmed(tracker, SPAWN)
    assert nearest.case == 'A'
    assert math.hypot(*nearest.centre) == pytest.approx(2.45, abs=0.05)


def test_every_other_gap_in_the_world_is_rejected_on_geometry(doors):
    """The distractors the world documents, and which gate catches each."""
    _tracker, cands = doors
    codes = collections.Counter(c.code or 'ok' for c in cands)
    assert codes['ok'] == 2
    # The two wide partition mouths and the corners.
    assert codes['width'] > 0
    # A1's end and B2's end against B1's plane: in the width band, but the
    # foot falls outside B1's extent, so they are corners, not openings.
    assert codes['foot_outside'] > 0
    # Nothing reached the scan-dependent gates: in an empty room every gap is
    # either a door or wrong on geometry alone.
    for scan_code in ('end_fov', 'end_occluded', 'not_through', 'on_segment'):
        assert codes[scan_code] == 0, f'{scan_code} fired: {codes!r}'
