"""multi_wall_tracker on synthetic scans: 270 deg / 0.25 deg, range noise sigma 0.01 m.

The world frame is odom. The car pose is odom from base_link, and the laser
sits at LASER = (0.305, 0, 0) in base_link, as on the car.
"""

import collections
import math
import time

import numpy as np
import pytest

from corridor_perception.geometry import Pose2D
from f1tenth_perception import multi_wall_tracker as mwt
from f1tenth_perception.scene_labels import detection_sectors

LASER = (0.305, 0.0, 0.0)
FRONT_X = 0.405
N_BEAMS = 1081
ANGLE_MIN = -math.radians(135.0)
INC = math.radians(0.25)
NO_RETURN = 65.533
ROOM = [((0, 0), (6, 0)), ((6, 0), (6, 6)), ((6, 6), (0, 6)), ((0, 6), (0, 0))]


def raycast(car, segments, circles=(), noise=0.01, rng=None):
    """Ranges of one scan from the laser on car (Pose2D) against segments and circles."""
    rng = np.random.default_rng(0) if rng is None else rng
    laser = car.compose(Pose2D(*LASER))
    origin = laser.t
    ang = laser.theta + ANGLE_MIN + np.arange(N_BEAMS) * INC
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
        oc = origin - np.array([cx, cy])
        b = d @ oc
        disc = b * b - (oc @ oc - r * r)
        t = -b - np.sqrt(np.maximum(disc, 0.0))
        hit = (disc >= 0) & (t > 0)
        best = np.where(hit & (t < best), t, best)
    ranges = best + rng.normal(0.0, noise, N_BEAMS)
    return np.where(np.isfinite(best), ranges, NO_RETURN)


def step(tracker, car, t, segments, circles=(), objects=(), rng=None):
    """One scan through extraction and the tracker; objects are (x, y, r) in base_link."""
    sectors = detection_sectors(objects, LASER[:2], math.radians(3.0)) if objects else ()
    lines = mwt.extract_lines(raycast(car, segments, circles, rng=rng), ANGLE_MIN, INC,
                              0.06, 30.0, LASER, sectors)
    return tracker.step([ln.transform(car, 'odom') for ln in lines], t, car)


def run(tracker, car, segments, n=10, t0=0.0, dt=0.05, **kw):
    out = None
    for i in range(n):
        out = step(tracker, car, t0 + i * dt, segments, **kw)
    return out


def by_label(tracker, tracks, car):
    return {tracker.label(t, car): t for t in tracks}


def test_room_from_two_poses_four_walls_correct_and_ids_kept():
    trk = mwt.MultiWallTracker()
    a = Pose2D(2.0, 3.0, 0.0)
    tracks = run(trk, a, ROOM)
    assert len(tracks) == 4
    labels = by_label(trk, tracks, a)
    assert set(labels) == {'front', 'back', 'left', 'right'}
    assert labels['front'].rho == pytest.approx(6.0, abs=0.03)
    assert labels['left'].rho == pytest.approx(6.0, abs=0.03)
    for t in tracks:
        assert min(abs(mwt.wrap_pi(t.theta - k * math.pi / 2)) for k in range(4)) < math.radians(1)
    d_perp, d_ray = mwt.front_distances(labels['front'], a, FRONT_X)
    assert d_ray == pytest.approx(6.0 - 2.0 - FRONT_X, abs=0.03)
    ids = {t.id for t in tracks}
    b = Pose2D(4.0, 2.0, 1.0)
    tracks_b = run(trk, b, ROOM, t0=1.0)
    assert {t.id for t in tracks_b} == ids
    assert by_label(trk, tracks_b, b)['front'].rho == pytest.approx(6.0, abs=0.03)


def test_corner_gives_two_tracks_at_ninety_degrees():
    trk = mwt.MultiWallTracker()
    corner = [((3.0, -2.0), (3.0, 1.0)), ((3.0, 1.0), (-1.0, 1.0))]
    tracks = run(trk, Pose2D(0.0, 0.0, 0.0), corner)
    assert len(tracks) == 2
    diff = abs(mwt.wrap_pi(tracks[0].theta - tracks[1].theta))
    assert diff == pytest.approx(math.pi / 2, abs=math.radians(1.0))


def test_person_in_front_of_the_wall_is_not_a_track_and_the_wall_does_not_bend():
    trk = mwt.MultiWallTracker()
    car = Pose2D(0.0, 0.0, 0.0)
    wall = [((4.0, -3.0), (4.0, 3.0))]
    person = [((2.0, 0.0), 0.25)]
    tracks = run(trk, car, wall, circles=person, objects=[(2.0, 0.0, 0.25)])
    assert len(tracks) == 1
    front = trk.front_track(car, FRONT_X)
    assert front is tracks[0]
    assert front.rho == pytest.approx(4.0, abs=0.02)
    assert abs(front.theta) < math.radians(0.5)
    assert mwt.front_distances(front, car, FRONT_X)[1] == pytest.approx(4.0 - FRONT_X, abs=0.02)
    assert all(t.rho > 3.5 for t in trk.tracks)


def test_slow_full_rotation_keeps_ids_and_the_cap():
    trk = mwt.MultiWallTracker()
    rng = np.random.default_rng(1)
    wall_id = {}
    issued = set()
    rate, dt = 0.3, 0.05
    # The room away from the odom origin, so no wall passes through it.
    room = [((ax + 10, ay + 10), (bx + 10, by + 10)) for (ax, ay), (bx, by) in ROOM]
    mids = np.array([[(ax + bx) / 2, (ay + by) / 2] for (ax, ay), (bx, by) in room])
    for i in range(int(2 * math.pi / rate / dt) + 40):
        car = Pose2D(13.0, 13.0, mwt.wrap_pi(rate * i * dt))
        tracks = step(trk, car, i * dt, room, rng=rng)
        assert len(tracks) <= 4
        for t in tracks:
            issued.add(t.id)
            mid = 0.5 * (t.endpoints()[0] + t.endpoints()[1])
            k = int(np.argmin(np.linalg.norm(mids - mid, axis=1)))
            assert wall_id.setdefault(k, t.id) == t.id
    assert len(wall_id) == 4
    assert issued == set(wall_id.values())


def test_cap_holds_with_more_than_four_walls():
    trk = mwt.MultiWallTracker()
    car = Pose2D(3.0, 3.0, 0.0)
    extra = [((4.5, 2.0), (4.5, 2.8)), ((4.0, 4.5), (5.0, 4.5))]
    tracks = run(trk, car, ROOM + extra)
    assert len(tracks) == 4
    assert len([t for t in trk.tracks if t.ever_confirmed]) > 4


def test_unseen_track_coasts_then_is_deleted():
    trk = mwt.MultiWallTracker()
    car = Pose2D(0.0, 0.0, 0.0)
    wall = [((4.0, -3.0), (4.0, 3.0))]
    run(trk, car, wall)
    trk.step([], 1.0, car)
    assert trk.tracks[0].state == mwt.STATE_COASTING
    trk.step([], 4.0, car)
    assert trk.tracks == []


def test_a_coasting_front_track_is_the_front_wall_only_up_to_the_coast_limit():
    trk = mwt.MultiWallTracker()
    car = Pose2D(0.0, 0.0, 0.0)
    wall = [((4.0, -3.0), (4.0, 3.0))]
    run(trk, car, wall)                      # last hit at t = 0.45
    limit = trk.p.front_max_coast_s
    trk.step([], 0.45 + 0.5 * limit, car)
    front = trk.front_track(car, FRONT_X)
    assert front is not None and front.state == mwt.STATE_COASTING
    assert not trk.coast_expired(front)
    trk.step([], 0.45 + 1.5 * limit, car)
    assert trk.front_track(car, FRONT_X) is None
    (coasting,) = trk.tracks                 # still alive, just not the front wall
    assert trk.coast_expired(coasting) and trk.track(coasting.id) is coasting
    run(trk, car, wall, n=1, t0=0.45 + 2.0 * limit)
    assert trk.front_track(car, FRONT_X) is coasting


def test_object_is_carried_to_the_scan_stamp_on_odometry():
    hist = mwt.PoseHistory()
    hist.add(0.0, 0.0, 0.0, 0.0)
    hist.add(1.0, 1.0, 0.0, 0.0)
    then, now = hist.at(0.0), hist.at(0.5)
    assert now.x == pytest.approx(0.5)
    assert mwt.propagate_objects([(2.0, 0.0, 0.25)], then, now) == [
        pytest.approx((1.5, 0.0, 0.25))]


def test_per_scan_cost_on_this_machine():
    """Extraction + tracking on a 6x6 room scan (report figure, generous bound)."""
    trk = mwt.MultiWallTracker()
    car = Pose2D(2.0, 3.0, 0.3)
    run(trk, car, ROOM)
    ranges = raycast(car, ROOM)
    times = []
    for i in range(50):
        t0 = time.perf_counter()
        lines = mwt.extract_lines(ranges, ANGLE_MIN, INC, 0.06, 30.0, LASER)
        trk.step([ln.transform(car, 'odom') for ln in lines], 1.0 + i * 0.025, car)
        times.append((time.perf_counter() - t0) * 1e3)
    print(f'multi_wall_tracker per-scan ms: p50 {np.percentile(times, 50):.2f} '
          f'p95 {np.percentile(times, 95):.2f}')
    assert np.percentile(times, 50) < 25.0


def _turn_with_yaw_scale(params, scale, deg=90.0):
    """Rotate 90 deg in place in a room; odometry reports yaw * scale. Wall -> ids."""
    trk = mwt.MultiWallTracker(params)
    rng = np.random.default_rng(2)
    room = [((ax + 10, ay + 10), (bx + 10, by + 10)) for (ax, ay), (bx, by) in ROOM]
    mids = np.array([[(ax + bx) / 2, (ay + by) / 2] for (ax, ay), (bx, by) in room])
    ids = collections.defaultdict(set)
    n, dt = 60, 0.05
    for i in range(n + 20):
        yaw = math.radians(deg) * min(i, n) / n
        true = Pose2D(13.0, 12.0, yaw)
        odom = Pose2D(13.0, 12.0, yaw * scale)
        sectors = ()
        lines = mwt.extract_lines(raycast(true, room, rng=rng), ANGLE_MIN, INC, 0.06, 30.0,
                                  LASER, sectors)
        for t in trk.step([ln.transform(odom, 'odom') for ln in lines], i * dt, odom):
            mid = 0.5 * (t.endpoints()[0] + t.endpoints()[1])
            ids[int(np.argmin(np.linalg.norm(mids - mid, axis=1)))].add(t.id)
    return ids


def test_a_90_degree_turn_with_7_percent_yaw_scale_error_keeps_ids():
    ids = _turn_with_yaw_scale(mwt.TrackerParams(), 1.07)
    assert len(ids) >= 3
    assert all(len(v) == 1 for v in ids.values()), dict(ids)
