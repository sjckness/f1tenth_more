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
from f1tenth_perception import wall_fusion
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


def observe(lines, source=wall_fusion.SOURCE_LIDAR, confidence=0.8,
            z_min=0.0, z_max=2.5, height_measured=False):
    """Wrap base_link extraction lines as the observations the tracker eats.

    Since step 4 the tracker consumes ONLY wall observations -- it no longer
    fits anything to a scan itself -- so these tests put the extraction's own
    lines through that same door.
    """
    return [wall_fusion.Observation(
        mwt.line_to_wall_line(ln), z_min, z_max, height_measured, source, confidence)
        for ln in lines]


def step(tracker, car, t, segments, circles=(), objects=(), rng=None, **obs_kw):
    """One scan through extraction and the tracker; objects are (x, y, r) in base_link."""
    sectors = detection_sectors(objects, LASER[:2], math.radians(3.0)) if objects else ()
    lines = mwt.extract_lines(raycast(car, segments, circles, rng=rng), ANGLE_MIN, INC,
                              0.06, 30.0, LASER, sectors)
    return tracker.step(observe(lines, **obs_kw), t, car)


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
        trk.step(observe(lines), 1.0 + i * 0.025, car)
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
        for t in trk.step(observe(lines), i * dt, odom):
            mid = 0.5 * (t.endpoints()[0] + t.endpoints()[1])
            ids[int(np.argmin(np.linalg.norm(mids - mid, axis=1)))].add(t.id)
    return ids


def test_a_90_degree_turn_with_7_percent_yaw_scale_error_keeps_ids():
    ids = _turn_with_yaw_scale(mwt.TrackerParams(), 1.07)
    assert len(ids) >= 3
    assert all(len(v) == 1 for v in ids.values()), dict(ids)


# ===========================================================================
# Step 4: the tracker consumes only observations
# ===========================================================================

from corridor_perception.geometry import Pose2D          # noqa: E402
from f1tenth_perception.scene_labels import WallLine     # noqa: E402

ORIGIN = Pose2D(0.0, 0.0, 0.0)
FRONT_X = 0.405


def wall(d, nx=1.0, ny=0.0, t0=-1.0, t1=1.0, inliers=120):
    return WallLine(nx, ny, d, t0, t1, inliers, 0.01, 'lidar')


def obs(line, source=wall_fusion.SOURCE_LIDAR, confidence=0.8, z=None):
    z_min, z_max, measured = (0.0, 2.5, False) if z is None else (z[0], z[1], True)
    return wall_fusion.Observation(line, z_min, z_max, measured, source, confidence)


def feed(tracker, observations, n, car=ORIGIN, t0=1.0, dt=0.05):
    out = None
    for i in range(n):
        out = tracker.step(list(observations), t0 + i * dt, car)
    return out


# --- the measurement covariance model --------------------------------------

def test_stereo_noise_grows_with_range_and_lidar_barely_does():
    p = mwt.TrackerParams()
    near_l = mwt.observation_covariance(wall_fusion.SOURCE_LIDAR, 1.0, 2.0, p)[0, 0]
    far_l = mwt.observation_covariance(wall_fusion.SOURCE_LIDAR, 6.0, 2.0, p)[0, 0]
    near_s = mwt.observation_covariance(wall_fusion.SOURCE_STEREO, 1.0, 2.0, p)[0, 0]
    far_s = mwt.observation_covariance(wall_fusion.SOURCE_STEREO, 6.0, 2.0, p)[0, 0]
    assert far_l > near_l and far_s > near_s
    # Stereo degrades far faster: that ratio is the point of the model.
    assert (far_s / near_s) > 10.0 * (far_l / near_l)
    assert far_s > 20.0 * far_l


def test_a_wall_both_sensors_saw_is_tighter_than_either_alone():
    p = mwt.TrackerParams()
    lidar = mwt.observation_covariance(wall_fusion.SOURCE_LIDAR, 3.0, 2.0, p)[0, 0]
    stereo = mwt.observation_covariance(wall_fusion.SOURCE_STEREO, 3.0, 2.0, p)[0, 0]
    both = mwt.observation_covariance(wall_fusion.SOURCE_BOTH, 3.0, 2.0, p)[0, 0]
    assert both < lidar < stereo


def test_a_short_segments_angle_is_trusted_less_than_a_long_ones():
    p = mwt.TrackerParams()
    stub = mwt.observation_covariance(wall_fusion.SOURCE_LIDAR, 2.0, 0.4, p)[1, 1]
    long = mwt.observation_covariance(wall_fusion.SOURCE_LIDAR, 2.0, 4.0, p)[1, 1]
    assert stub > long


# --- lifecycle --------------------------------------------------------------

def test_a_stereo_only_wall_needs_more_scans_than_a_lidar_one():
    p = mwt.TrackerParams()
    lidar_t = mwt.MultiWallTracker(p)
    feed(lidar_t, [obs(wall(2.0))], p.confirm_hits)
    assert lidar_t.tracks[0].ever_confirmed

    stereo_t = mwt.MultiWallTracker(p)
    feed(stereo_t, [obs(wall(2.0), wall_fusion.SOURCE_STEREO)], p.confirm_hits)
    assert not stereo_t.tracks[0].ever_confirmed, 'three scans is not enough on stereo alone'
    feed(stereo_t, [obs(wall(2.0), wall_fusion.SOURCE_STEREO)],
         p.stereo_confirm_hits - p.confirm_hits, t0=2.0)
    assert stereo_t.tracks[0].ever_confirmed


def test_a_one_frame_reflection_never_confirms():
    """A glossy-floor reflection appears for a frame and is gone. It must not
    leave a confirmed wall behind."""
    p = mwt.TrackerParams()
    trk = mwt.MultiWallTracker(p)
    trk.step([obs(wall(1.1), wall_fusion.SOURCE_STEREO)], 1.0, ORIGIN)
    for i in range(p.stereo_confirm_window + 1):
        trk.step([], 1.05 + i * 0.05, ORIGIN)
    assert all(not t.ever_confirmed for t in trk.tracks)
    assert trk.published(ORIGIN) == []


def test_a_lidar_hit_promotes_a_stereo_track_to_the_ordinary_rule():
    """Not an instant confirm -- one line must still not make a wall -- but the
    stricter stereo-only bar stops applying the moment the LiDAR sees it."""
    p = mwt.TrackerParams()
    trk = mwt.MultiWallTracker(p)
    feed(trk, [obs(wall(2.0), wall_fusion.SOURCE_STEREO)], 2)
    track = trk.tracks[0]
    assert not track.ever_confirmed
    assert trk._confirm_rule(track) == (p.stereo_confirm_hits, p.stereo_confirm_window)
    trk.step([obs(wall(2.0), wall_fusion.SOURCE_LIDAR)], 1.1, ORIGIN)
    assert trk._confirm_rule(track) == (p.confirm_hits, p.confirm_window)
    assert track.ever_confirmed           # 3 hits in 5 now satisfied
    assert track.source == wall_fusion.SOURCE_LIDAR


def test_a_track_the_lidar_has_seen_never_decays_to_stereo_only():
    trk = mwt.MultiWallTracker()
    feed(trk, [obs(wall(2.0), wall_fusion.SOURCE_BOTH)], 3)
    feed(trk, [obs(wall(2.0), wall_fusion.SOURCE_STEREO)], 3, t0=2.0)
    assert trk.tracks[0].source == wall_fusion.SOURCE_BOTH


def test_a_measured_height_is_carried_onto_the_track():
    trk = mwt.MultiWallTracker()
    feed(trk, [obs(wall(2.0), z=(0.05, 1.9))], 3)
    t = trk.tracks[0]
    assert t.height_measured and t.z_min == pytest.approx(0.05) and t.z_max == pytest.approx(1.9)


# --- what may be FRONT ------------------------------------------------------

def test_a_wall_across_the_path_is_front_with_the_right_ray_distance():
    trk = mwt.MultiWallTracker()
    feed(trk, [obs(wall(2.0))], 4)
    front = trk.front_track(ORIGIN, FRONT_X)
    assert front is not None
    _perp, d_ray = mwt.front_distances(front, ORIGIN, FRONT_X)
    assert d_ray == pytest.approx(2.0 - FRONT_X, abs=0.02)


def test_the_side_wall_of_an_open_corridor_is_never_front():
    """The corridor_phantom failure, as a test: a wall running ALONGSIDE the
    car has a normal within 45 deg of the heading for most of a turn, but the
    car cannot drive into it going straight."""
    trk = mwt.MultiWallTracker()
    side = wall(1.2, nx=0.0, ny=-1.0, t0=0.0, t1=4.0)
    feed(trk, [obs(side)], 5)
    assert trk.tracks, 'the side wall is still tracked'
    assert not mwt.crosses_strip(trk.tracks[0], ORIGIN, FRONT_X, 0.186)
    assert trk.front_track(ORIGIN, FRONT_X) is None


def test_a_tentative_track_is_never_front():
    """A maybe must not stop the car."""
    trk = mwt.MultiWallTracker()
    trk.step([obs(wall(2.0))], 1.0, ORIGIN)
    assert trk.tracks[0].state == mwt.STATE_TENTATIVE
    assert trk.front_track(ORIGIN, FRONT_X) is None


def test_a_glancing_stereo_only_wall_is_not_front_but_a_square_one_is():
    p = mwt.TrackerParams()
    square = mwt.MultiWallTracker(p)
    feed(square, [obs(wall(2.0), wall_fusion.SOURCE_STEREO)], p.stereo_confirm_hits)
    assert square.front_track(ORIGIN, FRONT_X) is not None

    oblique = mwt.MultiWallTracker(p)
    # 40 deg off the heading: still crosses the strip, but nothing cross-checks it.
    a = math.radians(40.0)
    slanted = wall(2.0, nx=math.cos(a), ny=math.sin(a), t0=-2.0, t1=2.0)
    feed(oblique, [obs(slanted, wall_fusion.SOURCE_STEREO)], p.stereo_confirm_hits)
    assert oblique.tracks[0].ever_confirmed
    assert mwt.crosses_strip(oblique.tracks[0], ORIGIN, FRONT_X, 0.186)
    assert oblique.front_track(ORIGIN, FRONT_X) is None
    # The same geometry seen by the LiDAR is allowed to be the front wall.
    lidar = mwt.MultiWallTracker(p)
    feed(lidar, [obs(slanted, wall_fusion.SOURCE_LIDAR)], p.confirm_hits)
    assert lidar.front_track(ORIGIN, FRONT_X) is not None


def test_crosses_strip_needs_the_wall_to_be_ahead_of_the_bumper():
    trk = mwt.MultiWallTracker()
    feed(trk, [obs(wall(0.2))], 4)          # closer than the bumper
    assert not mwt.crosses_strip(trk.tracks[0], ORIGIN, FRONT_X, 0.186)


def test_a_wall_beside_the_strip_does_not_cross_it():
    trk = mwt.MultiWallTracker()
    # A short front-facing wall 1 m to the left: normal forward, off the path.
    feed(trk, [obs(wall(2.0, t0=0.8, t1=2.0))], 4)
    assert not mwt.crosses_strip(trk.tracks[0], ORIGIN, FRONT_X, 0.186)
    assert trk.front_track(ORIGIN, FRONT_X) is None


# --- roles ------------------------------------------------------------------

def test_a_role_sticks_across_its_sector_edge_instead_of_flipping():
    p = mwt.TrackerParams(role_hysteresis_rad=math.radians(5.0))
    trk = mwt.MultiWallTracker(p)
    feed(trk, [obs(wall(2.0))], 3)
    track = trk.tracks[0]
    assert track.role == 'front'
    # Rotate the car so the wall sits just past the 45 deg boundary.
    for deg, expected in ((43.0, 'front'), (47.0, 'front'), (52.0, 'left')):
        trk.update_roles(Pose2D(0.0, 0.0, math.radians(-deg)))
        assert track.role == expected, deg


def test_a_role_with_no_history_is_just_the_sector():
    trk = mwt.MultiWallTracker()
    feed(trk, [obs(wall(2.0))], 3)
    trk.tracks[0].role = ''
    assert trk.label(trk.tracks[0], ORIGIN) == 'front'


# --- d_side -----------------------------------------------------------------

def test_the_side_distance_is_measured_from_the_bodywork_not_the_centreline():
    trk = mwt.MultiWallTracker()
    feed(trk, [obs(wall(1.2, nx=0.0, ny=-1.0, t0=0.0, t1=4.0))], 4)
    d = mwt.side_distance(trk.tracks[0], ORIGIN, 0.136)
    assert d == pytest.approx(1.2 - 0.136, abs=0.02)


def test_the_side_distance_never_goes_negative():
    trk = mwt.MultiWallTracker()
    feed(trk, [obs(wall(0.05, nx=0.0, ny=-1.0, t0=0.0, t1=4.0))], 4)
    assert mwt.side_distance(trk.tracks[0], ORIGIN, 0.136) == 0.0
