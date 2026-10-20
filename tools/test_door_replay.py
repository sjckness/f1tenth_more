"""
Unit tests of tools/door_replay.py on synthetic scans and messages: no bag, no ROS.

The tool is pure (no rclpy), so it is imported from its file and drives this
checkout's own extraction, wall tracker and door detector. The scene is
test_door_detection's: the laser at (0.305, 0, 0) on a car at the odom origin
unless a test moves it, 270 deg / 0.25 deg, range noise sigma 0.01 m; a wall
square to the car at x = 3 m with a doorway centred on y = 0, and the next
room's wall at x = 6 m. Run:

    python3 -m pytest tools/test_door_replay.py

This checkout's detector is the addendum's core. The tests of the shims for
the older cores (old_api_only: capture() around _judge_gap / _groups, the
16-entry reject cap, the old candidate rows) skip on it; one test here runs
them in a child pytest against a git archive of OLD_API_COMMIT, the last
commit before the addendum's core, through DOOR_REPLAY_SRC (which any run
may set to replay another checkout's detector).
"""

import csv
import importlib.util
import math
import os
from pathlib import Path
import struct
import subprocess
import sys
from types import SimpleNamespace as NS

import numpy as np
import pytest

TOOL = Path(__file__).resolve().parent / 'door_replay.py'
#: The checkout whose detector the tests replay (default: this one).
SRC_ENV = 'DOOR_REPLAY_SRC'
#: The step-3 core (a335113's door_detection) with the tracker and the yaml
#: it last ran with; the old_api_only tests were written against it.
OLD_API_COMMIT = '1148636'
OLD_API_PATHS = ('src/f1tenth_perception/f1tenth_perception',
                 'src/corridor_perception/corridor_perception', 'src/f1tenth_params/config')
#: The names of the old_api_only tests (filled by the decorator).
OLD_API_TESTS = []
LASER = (0.305, 0.0, 0.0)
N_BEAMS = 1081
ANGLE_MIN = -math.radians(135.0)
INC = math.radians(0.25)
NO_RETURN = 65.533
WALL_X = 3.0
BACK_X = 6.0


@pytest.fixture(scope='module')
def dr():
    spec = importlib.util.spec_from_file_location('door_replay', TOOL)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    mod.import_detector(Path(os.environ.get(SRC_ENV) or mod.REPO))
    return mod


@pytest.fixture
def old_api(dr):
    if dr.new_api():
        pytest.skip(f'an older door core only: runs through {SRC_ENV} (OLD_API_COMMIT)')


@pytest.fixture
def new_api(dr):
    if not dr.new_api():
        pytest.skip("the addendum's door core only")


def old_api_only(test):
    """Mark a test of the older cores' shims: skipped on the addendum's core."""
    OLD_API_TESTS.append(test.__name__)
    return pytest.mark.usefixtures('old_api')(test)


def header(t):
    sec = int(math.floor(t))
    return NS(stamp=NS(sec=sec, nanosec=int(round((t - sec) * 1e9))), frame_id='')


def raycast(segments, seed=0, noise=0.01, car=(0.0, 0.0, 0.0), circles=()):
    """Return one scan's ranges from the laser of a car at `car` (x, y, yaw) against segments."""
    rng = np.random.default_rng(seed)
    c, s = math.cos(car[2]), math.sin(car[2])
    origin = np.array([car[0] + c * LASER[0] - s * LASER[1], car[1] + s * LASER[0] + c * LASER[1]])
    ang = car[2] + LASER[2] + ANGLE_MIN + np.arange(N_BEAMS) * INC
    d = np.stack([np.cos(ang), np.sin(ang)], axis=1)
    best = np.full(N_BEAMS, np.inf)
    for (ax, ay), (bx, by) in segments:
        e = np.array([bx - ax, by - ay], dtype=float)
        w = np.array([ax, ay], dtype=float) - origin
        den = d[:, 0] * (-e[1]) - d[:, 1] * (-e[0])
        with np.errstate(divide='ignore', invalid='ignore'):
            t = (w[0] * (-e[1]) - w[1] * (-e[0])) / den
            u = (d[:, 0] * w[1] - d[:, 1] * w[0]) / den
        hit = (np.abs(den) > 1e-12) & (t > 0) & (u >= 0) & (u <= 1)
        best = np.where(hit & (t < best), t, best)
    for (cx, cy), r in circles:
        oc = origin - np.array([cx, cy], dtype=float)
        b = d @ oc
        disc = b * b - (oc @ oc - r * r)
        t = -b - np.sqrt(np.maximum(disc, 0.0))
        best = np.where((disc >= 0) & (t > 0) & (t < best), t, best)
    ranges = best + rng.normal(0.0, noise, N_BEAMS)
    return np.where(np.isfinite(best), ranges, NO_RETURN)


def doorway(half_gap):
    """Return the x = 3 wall either side of an opening 2 * half_gap wide, and the room behind."""
    return [((WALL_X, -3.0), (WALL_X, -half_gap)), ((WALL_X, half_gap), (WALL_X, 3.0)),
            ((BACK_X, -6.0), (BACK_X, 6.0))]


def scan_msg(ranges, t):
    """Return a LaserScan stand-in, as rosbags decodes one."""
    return NS(header=header(t), ranges=np.asarray(ranges, dtype=np.float32),
              angle_min=ANGLE_MIN, angle_increment=INC, range_min=0.06, range_max=30.0)


def run_retrack(dr, rule, half_gap=0.425, cycles=6):
    """
    Replay `cycles` scans of the doorway scene through the tool: retrack, detector, CSV rows.

    Returns (wall tracks, live doors, candidate rows, the last cycle's rejects).
    """
    p = dr.door_params(dr.REPO, 'code', True, [])
    tp, ep, node, _record = dr.wall_setup(dr.REPO, 'code', rule, p, [])
    retrack = dr.Retracker(tp, ep, node, LASER)
    detector = dr.Detector(p)
    car = dr.Pose2D(0.0, 0.0, 0.0)
    rows, sink, doors, rejects = [], [], [], []
    with dr.capture(sink):
        for i in range(cycles):
            t = 100.0 + 0.25 * i
            msg = scan_msg(raycast(doorway(half_gap), seed=i), t)
            r, b, _angles, cut = dr.mwt.scan_in_base_link(
                msg.ranges, msg.angle_min, msg.angle_increment, msg.range_min, msg.range_max,
                LASER, (), with_cut=True)
            scan = (r, b, LASER[:2], cut)
            tracks = retrack.step(msg, t, car, scan)
            del sink[:]
            _pieces, _cands, doors, rejects = detector.cycle(tracks, car, scan, t)
            rows = dr.candidate_rows(sink, i, t - 100.0, t, car, LASER[:2], detector.tracker,
                                     doors)
    return retrack.trk.tracks, doors, rows, rejects


def walls_on_the_line(tracks):
    """Return the confirmed tracks on the x = 3 wall line, ordered along it."""
    on = [t for t in tracks if t.ever_confirmed
          and abs(t.rho - WALL_X) < 0.1 and abs(math.sin(t.theta)) < 0.1]
    return sorted(on, key=lambda t: t.t0)


# ==============================================================================
# --walls retrack-new / retrack-old: THE COLLINEAR GAP RULE offline
# ==============================================================================

@old_api_only
def test_retrack_new_keeps_two_walls_at_a_0_85_m_doorway_and_a_door_candidate_between(dr):
    tracks, doors, rows, _rejects = run_retrack(dr, 'new')
    walls = walls_on_the_line(tracks)
    assert len(walls) == 2
    assert walls[0].t1 < -0.4 and walls[1].t0 > 0.4          # neither spans the doorway
    accepted = [r for r in rows if r['code'] == 'accepted']
    assert len(accepted) == 1
    row = accepted[0]
    assert {row['parent0'], row['parent1']} == {walls[0].id, walls[1].id}
    assert row['width'] == pytest.approx(0.85, abs=0.05)
    assert row['n_open'] >= row['n_req'] > 0
    assert row['door'] == str(doors[0].id) and len(doors) == 1


@old_api_only
def test_retrack_old_joins_the_wall_across_a_0_85_m_doorway_into_one_track(dr):
    tracks, _doors, rows, _rejects = run_retrack(dr, 'old')
    walls = walls_on_the_line(tracks)
    assert len(walls) == 1
    assert walls[0].t0 < -0.4 and walls[0].t1 > 0.4          # it spans the doorway
    # Whatever the detector finds there, it finds INSIDE that one track.
    assert all(r['parent0'] == r['parent1'] == walls[0].id
               for r in rows if r['code'] == 'accepted')


def test_the_old_rule_is_a_1_5_m_merge_and_a_0_5_m_inclusive_gap_on_either_tracker(dr):
    new_tracker = dr.mwt.TrackerParams()
    assert dr.wall_rule(new_tracker, 'old', 0.7) == (1.5, 'emulated')
    assert new_tracker.collinear_gap_m == pytest.approx(0.5) and new_tracker.collinear_gap_m > 0.5
    assert dr.wall_rule(dr.mwt.TrackerParams(), 'new', 0.8) == (0.8, 'native')
    pre_step_2 = NS(assoc_max_gap_m=0.5, merge_max_gap_m=1.5)      # <= assoc_max_gap_m
    assert dr.wall_rule(pre_step_2, 'old', 0.7) == (1.5, 'native')
    assert pre_step_2.assoc_max_gap_m == 0.5
    assert dr.wall_rule(pre_step_2, 'new', 0.7) == (0.7, 'emulated')
    assert pre_step_2.assoc_max_gap_m == pytest.approx(0.7) and pre_step_2.assoc_max_gap_m < 0.7


# ==============================================================================
# <tag>.candidates.csv
# ==============================================================================

@old_api_only
def test_the_candidates_csv_has_one_row_per_judged_gap_with_code_width_beams_and_door(
        dr, tmp_path):
    _tracks, _doors, rows, rejects = run_retrack(dr, 'new')
    path = tmp_path / 'x.doors.candidates.csv'
    dr.write_csv(path, dr.CSV_FIELDS, rows)
    with open(path, newline='') as fh:
        reader = csv.DictReader(fh)
        back = list(reader)
    assert tuple(reader.fieldnames) == dr.CSV_FIELDS
    assert {'code', 'width', 'n_open', 'n_req', 'parent0', 'parent1', 'j0x', 'j0y', 'j1x',
            'j1y', 'door'} <= set(reader.fieldnames)
    # Every refused gap is a row, the live trail's 16-entry cap lifted.
    assert sorted(r['code'] for r in back if r['code'] != 'accepted') \
        == sorted(code for code, _ids, _w in rejects)
    row = next(r for r in back if r['code'] == 'accepted')
    assert row['stage'] == 'gap' and row['door'] != ''
    assert float(row['j0x']) == pytest.approx(WALL_X, abs=0.05)
    assert float(row['cy']) == pytest.approx(0.0, abs=0.05)
    assert int(row['n_open']) + int(row['on']) + int(row['short']) + int(row['cut']) \
        <= int(row['n_gap'])


@old_api_only
def test_a_pair_of_tracks_too_far_apart_is_a_width_row_at_their_facing_ends(dr):
    def track(track_id, t0, t1):
        return dr.mwt.Track(id=track_id, x=np.array([WALL_X, 0.0]), P=np.eye(2) * 1e-3,
                            t0=t0, t1=t1, born=0.0, last_seen=0.0, ever_confirmed=True,
                            state=dr.mwt.STATE_CONFIRMED)
    car = dr.Pose2D(0.0, 0.0, 0.0)
    r, b, _angles, cut = dr.mwt.scan_in_base_link(
        raycast(doorway(0.7)), ANGLE_MIN, INC, 0.06, 30.0, LASER, (), with_cut=True)
    sink, rejects = [], []
    with dr.capture(sink):
        dr.dd.door_candidates([track(1, -3.0, -0.7), track(2, 0.7, 3.0)], car,
                              (r, b, LASER[:2], cut), dr.dd.DoorParams(frame_jambs=False),
                              rejects)
    rows = dr.candidate_rows(sink, 0, 0.0, 0.0, car, LASER[:2], dr.dd.DoorTracker(), [])
    assert [(code, ids) for code, ids, _w in rejects] == [('width', (1, 2))]
    (row,) = rows
    assert (row['stage'], row['code'], row['width']) == ('pair', 'width', pytest.approx(1.4))
    assert sorted([row['j0y'], row['j1y']]) == pytest.approx([-0.7, 0.7])
    assert row['j0x'] == pytest.approx(WALL_X) and row['n_open'] == ''


@old_api_only
def test_the_capture_restores_the_detector_when_the_replay_ends(dr):
    judge, groups = dr.dd._judge_gap, dr.dd._groups
    with dr.capture([]):
        assert dr.dd._judge_gap is not judge and dr.dd._MAX_REJECTS > 10 ** 6
    assert dr.dd._judge_gap is judge and dr.dd._groups is groups
    assert dr.dd._MAX_REJECTS == 16


# ==============================================================================
# The cadence, the pose and the bag
# ==============================================================================

def test_the_steps_cadence_runs_one_cycle_per_walls_message_on_the_scan_of_its_stamp(dr):
    scans = [NS(header=header(100.0 + 0.025 * k)) for k in range(40)]
    walls = [NS(header=header(100.0 + 0.025 * k)) for k in range(0, 40, 4)]
    walls.insert(3, NS(header=header(100.31)))           # its scan is not in the bag
    plan, cadence, missing = dr.plan_cycles(scans, walls, 'steps', 4)
    assert (cadence, missing, len(plan)) == ('steps', 1, 10)
    assert all(s.header.stamp == w.header.stamp for s, w in plan)
    plan, cadence, missing = dr.plan_cycles(scans, [], 'steps', 4)
    assert (cadence, missing, len(plan)) == ('every', 0, 10)


def test_the_car_pose_is_interpolated_at_the_scan_stamp_through_the_yaw_wrap(dr):
    def odom(t, x, yaw):
        q = NS(x=0.0, y=0.0, z=math.sin(yaw / 2.0), w=math.cos(yaw / 2.0))
        return NS(header=header(t), pose=NS(pose=NS(position=NS(x=x, y=0.0), orientation=q)))
    poses = dr.Poses([odom(10.0, 0.0, math.radians(179.0)), odom(10.1, 1.0, math.radians(-179.0))])
    car = poses.at(10.05)
    assert car.x == pytest.approx(0.5)
    assert abs(math.sin(car.theta)) < 1e-6 and math.cos(car.theta) < 0.0   # 180 deg, not 0
    assert poses.at(10.1 + 0.1) is not None and poses.at(10.1 + 0.2) is None


def test_an_undecodable_message_keeps_its_header_stamp(dr):
    cdr = bytes([0, 1, 0, 0]) + struct.pack('<iI', 1792154779, 925453000) + b'\x05\0\0\0odom\0'
    msg = dr.header_only(cdr)
    assert dr.stamp(msg) == pytest.approx(1792154779.925453)
    with pytest.raises(SystemExit):
        dr.tracks_from(msg)


# ==============================================================================
# door_<id>.png: the frame and the jambs in it
# ==============================================================================

def test_the_debug_frame_is_the_nearest_by_header_stamp_not_by_receipt(dr):
    # Captured every 0.2 s, received 0.26 s later -- and one pair out of order.
    frames = [NS(header=header(100.0 + 0.2 * k), receipt=100.26 + 0.2 * k) for k in range(10)]
    frames[3].receipt, frames[4].receipt = frames[4].receipt, frames[3].receipt
    received = sorted(frames, key=lambda f: f.receipt)
    dr.sort_by_stamp(received)
    t = 100.45                     # by receipt the nearest would be the 100.2 frame
    ((first, _label),) = dr.select_frames(received, t, (99.0, 102.0))
    assert dr.stamp(first) == pytest.approx(100.4)
    # Not in view there: the nearest frame inside the window that shows it.
    shown = dr.select_frames(received, t, (100.1, 102.0),
                             in_view=lambda f: dr.stamp(f) < 100.3 or dr.stamp(f) > 101.0)
    assert [round(dr.stamp(f), 1) for f, _label in shown] == [100.4, 100.2]
    shown = dr.select_frames(received, t, (100.3, 102.0),
                             in_view=lambda f: dr.stamp(f) < 100.3 or dr.stamp(f) > 101.0)
    assert [round(dr.stamp(f), 1) for f, _label in shown] == [100.4, 101.2]


def test_a_door_ahead_projects_about_the_principal_point_and_one_abeam_is_out_of_view(dr):
    # Optical x right, y down, z ahead, 0.2 m above base_link: p_bl = R p + T.
    rot = np.array([[0.0, 0.0, 1.0], [-1.0, 0.0, 0.0], [0.0, -1.0, 0.0]])
    cam = (rot, np.array([0.0, 0.0, 0.2]), (262.5, 262.5, 318.7, 179.3), 640)
    car = dr.Pose2D(0.0, 0.0, 0.0)
    (u0, v0), (u1, v1) = dr.project_posts([[3.0, 0.4], [3.0, -0.4]], car, cam, 480)
    assert 0.5 * (u0 + u1) == pytest.approx([0.75 * 318.7] * 2)
    assert u0[0] < u1[0]                                  # +y is left in the image
    assert v0[0] > 0.75 * 179.3 > v0[1]                   # the floor below, 2 m above
    assert dr.project_posts([[0.0, 3.0], [0.4, 3.0]], car, cam, 480) is None
    turned = dr.Pose2D(0.0, 0.0, math.pi / 2.0)           # the car faces the door
    assert dr.project_posts([[0.4, 3.0], [-0.4, 3.0]], turned, cam, 480) is not None


# ==============================================================================
# The addendum's core: every DoorCandidate a row, and what it did to the doors
# ==============================================================================

#: A box face 0.3 m behind the doorway across its core: a STRONG reject.
BOX_IN_THE_DOORWAY = [((WALL_X + 0.3, -0.3), (WALL_X + 0.3, 0.3))]
#: A pole in front of the middle of the doorway: something unknown in the gap.
POLE_IN_THE_DOORWAY = (((WALL_X - 0.4, 0.0), 0.05),)
#: A corridor along x, its left wall at y = 1.2 with a 0.85 m doorway at x = 0
#: and the office behind it, its right wall at y = -1.2.
SIDE_Y = 1.2
SIDE_DOORWAY = [((-6.0, SIDE_Y), (-0.425, SIDE_Y)), ((0.425, SIDE_Y), (6.0, SIDE_Y)),
                ((-6.0, -SIDE_Y), (6.0, -SIDE_Y)), ((-9.0, SIDE_Y + 3.0), (9.0, SIDE_Y + 3.0))]


def front_hosts(dr, half_gap=0.425):
    """Return the x = 3 wall as two CONFIRMED 'front' tracks either side of the doorway."""
    def track(track_id, t0, t1):
        trk = dr.mwt.Track(id=track_id, x=np.array([WALL_X, 0.0]), P=np.eye(2) * 1e-3,
                           t0=t0, t1=t1, born=0.0, last_seen=0.0, ever_confirmed=True,
                           state=dr.mwt.STATE_CONFIRMED)
        trk.role = 'front'
        return trk
    return [track(1, -3.0, -half_gap), track(2, half_gap, 3.0)]


class NewReplay:
    """What replay() does per cycle on the addendum's core, on fixed walls."""

    def __init__(self, dr, walls):
        self.dr, self.walls = dr, walls
        self.detector = dr.Detector(dr.door_params(dr.REPO, 'code', True, []))
        self.life, self.rows, self.doors, self.k = {}, [], [], 0

    def cycle(self, t, car=(0.0, 0.0, 0.0), segments=(), circles=()):
        """Run one cycle with the car at `car` (x, y, yaw); return its rows."""
        dr, pose = self.dr, self.dr.Pose2D(*car)
        r, b, _angles, cut = dr.mwt.scan_in_base_link(
            raycast(doorway(0.425) + list(segments), seed=self.k, car=car, circles=circles),
            ANGLE_MIN, INC, 0.06, 30.0, LASER, (), with_cut=True)
        scan = (r, b, LASER[:2], cut)
        _pieces, cands, self.doors, _rejects = self.detector.cycle(self.walls, pose, scan, t)
        rows = dr.candidate_rows_new(cands, self.k, t, t, pose, LASER[:2], self.detector,
                                     self.doors)
        dr.fold_life_new(self.life, self.detector, self.doors, t, t, pose)
        self.rows += rows
        self.k += 1
        return rows

    def confirm(self):
        """Confirm the doorway: seen from y = 0 twice, then from y = 0.5 (two viewpoints)."""
        for y in (0.0, 0.0, 0.5):
            self.cycle(0.1 * self.k, (0.0, y, 0.0))
        assert [d.state for d in self.doors] == [self.dr.dd.STATE_CONFIRMED]


@pytest.mark.usefixtures('new_api')
def test_a_doorway_between_two_confirmed_front_hosts_is_a_row_with_code_empty_feeding_a_door(
        dr, tmp_path):
    run = NewReplay(dr, front_hosts(dr))
    (row,) = run.cycle(0.0)
    assert (row['code'], row['cls'], row['visible'], row['detail']) == ('', 'ok', 1, '')
    assert row['width'] == pytest.approx(0.85, abs=0.03)
    assert row['n_beams'] >= row['n_required'] > 0 and row['behind_frac'] == pytest.approx(1.0)
    assert (row['parent0'], row['parent1'], row['leaf']) == (1, 2, 'none')
    assert row['j0x'] == pytest.approx(WALL_X, abs=0.03)
    assert row['cy'] == pytest.approx(0.0, abs=0.03)
    assert row['jamb_height_unknown'] == 1                  # no camera in the replay
    (door,) = run.doors
    assert (row['effect'], row['door'], row['door_state']) == ('spawn', str(door.id), 'candidate')
    (row,) = run.cycle(0.1)
    assert (row['effect'], row['door'], row['door_state']) == ('hit', str(door.id), 'candidate')
    # The replay's one Manhattan prior was fed the walls: the room's axis is theirs.
    assert math.degrees(run.detector.manhattan.offset(0.0)) == pytest.approx(0.0, abs=0.5)
    path = tmp_path / 'x.doors.candidates.csv'
    dr.write_csv(path, dr.CSV_FIELDS_NEW, run.rows)
    with open(path, newline='') as fh:
        reader = csv.DictReader(fh)
        back = list(reader)
    assert tuple(reader.fieldnames) == dr.CSV_FIELDS_NEW
    assert [(r['code'], r['cls'], r['effect']) for r in back] == [
        ('', 'ok', 'spawn'), ('', 'ok', 'hit')]


@pytest.mark.usefixtures('new_api')
def test_a_pole_in_the_doorway_is_a_weak_row_that_leaves_the_confirmed_door_alone(dr):
    run = NewReplay(dr, front_hosts(dr))
    run.confirm()
    (row,) = run.cycle(0.3, circles=POLE_IN_THE_DOORWAY)
    assert (row['code'], row['cls'], row['detail'], row['visible']) == (
        'unknown_in_gap', 'weak', 'blocked', 0)
    assert (row['effect'], row['door'], row['door_state']) == ('', '', '')
    assert [(d.state, d.strikes) for d in run.doors] == [(dr.dd.STATE_CONFIRMED, 0)]


@pytest.mark.usefixtures('new_api')
def test_a_strong_visible_row_strikes_the_confirmed_door_to_candidate_and_a_second_deletes_it(dr):
    run = NewReplay(dr, front_hosts(dr))
    run.confirm()
    door_id = run.doors[0].id
    (row,) = run.cycle(0.3, segments=BOX_IN_THE_DOORWAY)
    assert (row['cls'], row['visible']) == ('strong', 1)
    assert (row['effect'], row['door'], row['door_state']) == ('strike', str(door_id), 'candidate')
    (row,) = run.cycle(0.4, segments=BOX_IN_THE_DOORWAY)
    assert (row['effect'], row['door'], row['door_state']) == ('delete', str(door_id), 'deleted')
    assert run.doors == [] and run.detector.tracker.n_deleted_by_strikes == 1
    life = dr._track_summary(run.life[door_id], 0.0, None)
    assert life['states'] == ['candidate', 'confirmed']
    assert life['first_confirmed'] == pytest.approx(0.2)
    assert (life['hits'], life['strikes'], life['deleted_by_strikes']) == (3, 2, True)
    assert life['n_viewpoints'] == 2 and life['width_std'] < 0.05
    assert life['best']['state'] in ('candidate', 'confirmed')


@pytest.mark.usefixtures('new_api')
def test_retrack_new_driving_past_a_side_doorway_confirms_one_door_between_two_left_hosts(dr):
    p = dr.door_params(dr.REPO, 'code', True, [])
    tp, ep, node, _record = dr.wall_setup(dr.REPO, 'code', 'new', p, [])
    retrack = dr.Retracker(tp, ep, node, LASER)
    detector = dr.Detector(p)
    life, rows = {}, []
    for i, x in enumerate(np.arange(-2.5, -0.45, 0.1)):
        t = 100.0 + 0.1 * i
        car = dr.Pose2D(float(x), 0.0, 0.0)
        msg = scan_msg(raycast(SIDE_DOORWAY, seed=i, car=(float(x), 0.0, 0.0)), t)
        r, b, _angles, cut = dr.mwt.scan_in_base_link(
            msg.ranges, msg.angle_min, msg.angle_increment, msg.range_min, msg.range_max,
            LASER, (), with_cut=True)
        scan = (r, b, LASER[:2], cut)
        tracks = retrack.step(msg, t, car, scan)
        _pieces, cands, doors, _rejects = detector.cycle(tracks, car, scan, t)
        rows += dr.candidate_rows_new(cands, i, t - 100.0, t, car, LASER[:2], detector, doors)
        dr.fold_life_new(life, detector, doors, t - 100.0, t, car)
    (door,) = doors
    assert door.state == dr.dd.STATE_CONFIRMED
    left = {trk.id for trk in tracks if dr.dd.is_host(trk) and trk.role == 'left'}
    assert set(door.parents) <= left
    positive = [row for row in rows if row['code'] == '']
    assert {row['door'] for row in positive} == {str(door.id)}
    assert [row['effect'] for row in positive] == ['spawn'] + ['hit'] * (len(positive) - 1)
    assert dr._track_summary(life[door.id], 0.0, None)['n_viewpoints'] >= 2


@pytest.mark.usefixtures('new_api')
def test_yaml_keys_the_checkout_lacks_leave_their_fields_on_the_code_defaults_and_are_listed(
        dr, tmp_path):
    config = tmp_path / 'src/f1tenth_params/config'
    config.mkdir(parents=True)
    (config / 'stack_params.yaml').write_text(
        'lidar_front_wall_door_width_max_m:\n  default: 0.9\n'
        'lidar_front_wall_door_manhattan_tol_deg:\n  default: 7.5\n'
        'lidar_front_wall_door_confirm_hits:\n  default: 3\n')
    record = {}
    p = dr.door_params(tmp_path, 'stack', True, ['behind_frac=0.6'], record)
    default = dr.dd.DoorParams()
    assert (p.width_max_m, p.manhattan_tol_deg, p.behind_frac) == (0.9, 7.5, 0.6)
    assert (p.min_viewpoints, p.manhattan_prior) == (default.min_viewpoints,
                                                     default.manhattan_prior)
    assert record['yaml'] == ['manhattan_tol_deg', 'width_max_m']
    assert record['override'] == ['behind_frac']
    assert {'min_viewpoints', 'manhattan_prior', 'confirmed_max_age_s'} \
        <= set(record['code_default'])
    assert {'lidar_front_wall_door_min_viewpoints', 'lidar_front_wall_door_behind_frac'} \
        <= set(record['yaml_keys_absent'])
    assert record['yaml_keys_ignored'] == ['lidar_front_wall_door_confirm_hits']


def test_a_door_image_draws_a_confirmed_door_solid_and_a_candidate_faded_titled_by_its_life(dr):
    life = {'states': ['candidate', 'confirmed'], 'first_confirmed': 12.34, 'hits': 7,
            'n_viewpoints': 3, 'width_std': 0.0123, 'strikes': 1, 'deleted_by_strikes': False,
            'square_looks': 0}
    _colour, _top, alpha, style, title = dr.door_style(life)
    assert (alpha, style) == (1.0, '-')
    assert title == ('confirmed at 12.3 s: hits 7, viewpoints 3, width std 0.012 m, '
                     'strikes 1')
    _colour, _top, alpha, style, title = dr.door_style(
        {**life, 'states': ['candidate'], 'strikes': 2, 'deleted_by_strikes': True})
    assert alpha < 1.0 and style == '--'
    assert title.startswith('candidate, deleted by strikes: hits 7')
    # An older core's track (no strikes): drawn as it always was.
    assert dr.door_style({'states': ['tentative'], 'square_looks': 2})[:4] == (
        'lime', 'g', 1.0, '-')


# ==============================================================================
# --summary-md
# ==============================================================================

def test_the_summary_md_counts_rows_by_class_and_code_with_median_width_and_beams(dr):
    def row(code, cls, width, beams, required, visible=1, detail=''):
        return {'code': code, 'cls': cls, 'width': width, 'n_beams': beams,
                'n_required': required, 'visible': visible, 'detail': detail}
    rows = [row('', 'ok', 0.85, 70, 58), row('', 'ok', 0.87, 80, 58), row('', 'ok', 0.95, 60, 50),
            row('unknown_in_gap', 'weak', 0.9, 40, 50, 0, 'blocked'),
            row('width', 'geometry', 1.4, 0, 0, 0)]
    summary = {'api': 'new', 'bag': 'bag', 'src': '/wt/x', 'src_commit': 'abc1234',
               'walls': 'recorded', 'cadence': 'steps', 'params': 'stack', 'overrides': [],
               'cycles': 10, 'door_tracks': 1, 'confirmed_doors': 1,
               'lifecycle': {'strikes': 0, 'deleted_by_strikes': 0}, 'seconds': {'total': 1.0}}
    lines = dr.summary_md(rows, summary).splitlines()
    assert '| ok | 3 | 60.0 | 3 |' in lines and '| geometry | 1 | 20.0 | 0 |' in lines
    assert "| ok | `''` (door) | 3 | 60.0 | 3 | 0.870 | 70 |  |" in lines
    assert '| weak | `unknown_in_gap` | 1 | 20.0 | 0 | 0.900 | 40 | blocked 1 |' in lines
    assert '| geometry | `width` | 1 | 20.0 | 0 | 1.400 | - |  |' in lines
    # An older core: accepted is 'ok', its beams the judged gap's through + shallow ones.
    old = [{'stage': 'gap', 'code': 'accepted', 'width': 0.9, 'beyond': 50, 'none': 10,
            'shallow': 4},
           {'stage': 'pair', 'code': 'not_collinear', 'width': 0.8}]
    lines = dr.summary_md(old, {**summary, 'api': 'old'}).splitlines()
    assert '| ok | `accepted` | 1 | 50.0 | - | 0.900 | 64 |  |' in lines
    assert '| - | `not_collinear` | 1 | 50.0 | - | 0.800 | - |  |' in lines


# ==============================================================================
# The older cores' shims, against the real step-3 detector
# ==============================================================================

def test_the_old_api_tests_pass_on_the_step_3_detector_through_src(dr, tmp_path):
    """Run the old_api_only tests in a child pytest on a git archive of OLD_API_COMMIT."""
    if os.environ.get(SRC_ENV) or not dr.new_api():
        pytest.skip(f'run from the addendum core only, without {SRC_ENV}')
    try:
        archive = subprocess.run(['git', '-C', str(dr.REPO), 'archive', OLD_API_COMMIT,
                                  *OLD_API_PATHS], capture_output=True, check=True,
                                 timeout=60).stdout
        subprocess.run(['tar', '-x', '-C', str(tmp_path)], input=archive, check=True,
                       timeout=60)
    except (OSError, subprocess.SubprocessError) as exc:
        pytest.skip(f'no git archive of {OLD_API_COMMIT} here: {exc}')
    ids = [f'{Path(__file__).name}::{name}' for name in OLD_API_TESTS]
    res = subprocess.run([sys.executable, '-m', 'pytest', '-q', '-p', 'no:cacheprovider', *ids],
                         cwd=Path(__file__).resolve().parent, capture_output=True, text=True,
                         timeout=600, env={**os.environ, SRC_ENV: str(tmp_path),
                                           'PYTHONDONTWRITEBYTECODE': '1'})
    assert res.returncode == 0, res.stdout[-4000:] + res.stderr[-2000:]
    assert f'{len(OLD_API_TESTS)} passed' in res.stdout, res.stdout[-2000:]
