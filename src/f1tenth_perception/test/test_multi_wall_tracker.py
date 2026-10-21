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


def scan_of(ranges, sectors=()):
    """The scan tuple the node hands the tracker: (ranges, bearings, laser xy, cut)."""
    r, b, _angles, cut = mwt.scan_in_base_link(ranges, ANGLE_MIN, INC, 0.06, 30.0, LASER,
                                               sectors, with_cut=True)
    return r, b, LASER[:2], cut


def step(tracker, car, t, segments, circles=(), objects=(), rng=None, observations=None,
         extra=(), **obs_kw):
    """One scan through extraction and the tracker; objects are (x, y, r) in base_link.

    The scan goes in too, cut mask and all, exactly as lidar_front_wall_node
    passes it: the gap, strip and negative-evidence rules all read the beams.
    `observations` replaces the extraction's own; `extra` is added to them
    (a hand-built bridge, a stereo line).
    """
    sectors = detection_sectors(objects, LASER[:2], math.radians(3.0)) if objects else ()
    ranges = raycast(car, segments, circles, rng=rng)
    if observations is None:
        lines = mwt.extract_lines(ranges, ANGLE_MIN, INC, 0.06, 30.0, LASER, sectors)
        observations = observe(lines, **obs_kw)
    return tracker.step(list(observations) + list(extra), t, car,
                        scan=scan_of(ranges, sectors))


def run(tracker, car, segments, n=10, t0=0.0, dt=0.05, **kw):
    out = None
    for i in range(n):
        out = step(tracker, car, t0 + i * dt, segments, **kw)
    return out


def by_label(tracker, tracks, car):
    return {tracker.label(t, car): t for t in tracks}


def test_room_from_two_poses_walls_correct_and_ids_kept():
    """The three walls the laser can actually SEE as walls, and their ids.

    Not four. The wall directly behind the car is outside the +-135 deg FOV
    except for two 0.70 m arcs near its corners, and since the span gate
    (min_span_birth_m) a 0.70 m arc does not start a wall track. It is born as
    soon as the car turns enough to see it properly, which the second pose
    below is: the x = 0 wall comes in at 4.65 m as track 4, and the three
    already-tracked walls keep their ids. That is the gate working as intended
    -- a wall is born when it is seen as a wall -- and it is the one behaviour
    it changed on a scan with nothing wrong with it, so it is asserted here
    rather than left as a surprise.
    """
    trk = mwt.MultiWallTracker()
    a = Pose2D(2.0, 3.0, 0.0)
    tracks = run(trk, a, ROOM)
    assert len(tracks) == 3
    labels = by_label(trk, tracks, a)
    assert set(labels) == {'front', 'left', 'right'}
    assert labels['front'].rho == pytest.approx(6.0, abs=0.03)
    assert labels['left'].rho == pytest.approx(6.0, abs=0.03)
    for t in tracks:
        assert min(abs(mwt.wrap_pi(t.theta - k * math.pi / 2)) for k in range(4)) < math.radians(1)
    d_perp, d_ray = mwt.front_distances(labels['front'], a, FRONT_X)
    assert d_ray == pytest.approx(6.0 - 2.0 - FRONT_X, abs=0.03)
    ids = {t.id for t in tracks}
    b = Pose2D(4.0, 2.0, 1.0)
    tracks_b = run(trk, b, ROOM, t0=1.0)
    # THE IDS ARE KEPT, which is a statement about the TRACKER, not about the
    # published four: since the cap became role-priority (H3) a wall whose
    # role is 'unknown' takes no slot, and at this pose the y = 6 wall is one
    # -- the car has turned a radian, so it is neither in the strip nor beside
    # the car with a lateral normal. It is still tracked, still confirmed and
    # still an occluder.
    live = {t.id for t in trk.tracks}
    assert ids < live and len(live) == 4
    unknown = [t for t in trk.tracks if t.role == 'unknown']
    assert len(unknown) == 1 and unknown[0].ever_confirmed
    assert unknown[0] not in tracks_b
    assert by_label(trk, tracks_b, b)['front'].rho == pytest.approx(6.0, abs=0.03)
    # The late arrival is the wall that was behind the car at pose a.
    late = next(t for t in trk.tracks if t.id not in ids)
    assert late.rho == pytest.approx(0.0, abs=0.05)
    assert late.length > 4.0


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
    """The extras are 2 m partitions, not 0.8 m stubs.

    They used to be 0.8 and 1.0 m, which the span gate (min_span_birth_m) now
    refuses to born at all -- so the test was no longer about the publish cap.
    A partition the cap has to rank has to be a wall in the first place.
    """
    trk = mwt.MultiWallTracker()
    car = Pose2D(3.0, 3.0, 0.0)
    extra = [((4.5, 1.0), (4.5, 3.0)), ((3.5, 4.5), (5.5, 4.5))]
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


#: THE CONFIRMATION LEVEL's run (wall_confirm_min_consecutive_hits): a steady
#: LiDAR wall fed this many cycles in a row is confirmed on the last of them.
N_CONFIRM = mwt.TrackerParams().confirm_min_consecutive_hits


def confirm(tracker, observations, car=ORIGIN, t_end=1.1, dt=0.05):
    """Feed exactly the N_CONFIRM cycles that confirm a steady wall, ending at t_end.

    Ends where the 3-in-5 rule's three-cycle feed did (1.1), so what a test
    does AFTER the wall is confirmed keeps its timing.
    """
    return feed(tracker, observations, N_CONFIRM, car=car,
                t0=t_end - (N_CONFIRM - 1) * dt, dt=dt)


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

# THE CONFIRMATION LEVEL itself -- promotion after exactly N hits, never on
# stereo alone, never above the jitter bounds, never demoted -- is
# test_wall_confirm_level.py's. These are the lifecycle around it.

def test_a_one_frame_reflection_never_confirms():
    """A glossy-floor reflection appears for a frame and is gone. It must not
    leave a confirmed wall behind -- nor a tentative one: the first cycle
    without it ends its run, and the track with it."""
    p = mwt.TrackerParams()
    trk = mwt.MultiWallTracker(p)
    trk.step([obs(wall(1.1), wall_fusion.SOURCE_STEREO)], 1.0, ORIGIN)
    for i in range(N_CONFIRM + 1):
        trk.step([], 1.05 + i * 0.05, ORIGIN)
    assert trk.tracks == []
    assert trk.published(ORIGIN) == []


def test_a_track_the_lidar_has_seen_never_decays_to_stereo_only():
    trk = mwt.MultiWallTracker()
    feed(trk, [obs(wall(2.0), wall_fusion.SOURCE_BOTH)], 3)
    feed(trk, [obs(wall(2.0), wall_fusion.SOURCE_STEREO)], 3, t0=2.0)
    assert trk.tracks[0].source == wall_fusion.SOURCE_LIDAR


def test_both_needs_the_rectangle_as_well_as_the_columns():
    """E2/E3: a column agreement alone is one row of pixels through the scan
    plane. "BOTH" means both sensors looked at the WALL, so the rectangle must
    have had something to say about this track too."""
    trk = mwt.MultiWallTracker()
    feed(trk, [obs(wall(2.0), wall_fusion.SOURCE_BOTH)], 3)
    track = trk.tracks[0]
    assert track.source == wall_fusion.SOURCE_LIDAR and not track.has_both
    trk.apply_rect_verdicts({track.id: {'coverage': 0.9, 'support': 0.9,
                                        'beyond_share': 0.0}})
    feed(trk, [obs(wall(2.0), wall_fusion.SOURCE_BOTH)], 1, t0=2.0)
    assert track.source == wall_fusion.SOURCE_BOTH


def test_a_measured_height_is_carried_onto_the_track():
    trk = mwt.MultiWallTracker()
    feed(trk, [obs(wall(2.0), z=(0.05, 1.9))], 3)
    t = trk.tracks[0]
    assert t.height_measured and t.z_min == pytest.approx(0.05) and t.z_max == pytest.approx(1.9)


# --- what may be FRONT ------------------------------------------------------

def test_a_wall_across_the_path_is_front_with_the_right_ray_distance():
    trk = mwt.MultiWallTracker()
    confirm(trk, [obs(wall(2.0))])
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


VERIFIED = {'coverage': 0.9, 'support': 0.9, 'beyond_share': 0.0}


def test_a_glancing_stereo_only_wall_is_not_front_but_a_square_verified_one_is():
    """UPDATED for block 3 (WHAT FRONT REQUIRES): a square stereo-only wall
    used to be FRONT on the stereo's word alone. It now needs the rectangle to
    have VERIFIED it -- nothing else has measured it (no LiDAR fit in the last
    front_lidar_recent_cycles cycles) -- and a glancing one stays refused.

    Run with wall_confirm_require_lidar OFF: with it on (the default) a
    stereo-only track never confirms at all (test_wall_confirm_level.py), and
    these FRONT rules are what still guard the stereo-only walls that setting
    lets through."""
    p = mwt.TrackerParams(confirm_require_lidar=False)
    square = mwt.MultiWallTracker(p)
    feed(square, [obs(wall(2.0), wall_fusion.SOURCE_STEREO)], N_CONFIRM)
    assert square.front_track(ORIGIN, FRONT_X) is None
    assert square.front_status['reason'] == 'unverified'
    square.apply_rect_verdicts({square.tracks[0].id: VERIFIED})
    assert square.front_track(ORIGIN, FRONT_X) is square.tracks[0]

    oblique = mwt.MultiWallTracker(p)
    # 40 deg off the heading: still crosses the strip, but nothing cross-checks it.
    a = math.radians(40.0)
    slanted = wall(2.0, nx=math.cos(a), ny=math.sin(a), t0=-2.0, t1=2.0)
    feed(oblique, [obs(slanted, wall_fusion.SOURCE_STEREO)], N_CONFIRM)
    assert oblique.tracks[0].ever_confirmed
    oblique.apply_rect_verdicts({oblique.tracks[0].id: VERIFIED})
    assert oblique.front_reason(oblique.tracks[0], ORIGIN) == 'stereo_oblique'
    assert mwt.crosses_strip(oblique.tracks[0], ORIGIN, FRONT_X, 0.186)
    assert oblique.front_track(ORIGIN, FRONT_X) is None
    # The same geometry seen by the LiDAR is allowed to be the front wall.
    lidar = mwt.MultiWallTracker(p)
    feed(lidar, [obs(slanted, wall_fusion.SOURCE_LIDAR)], N_CONFIRM)
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

def test_a_wall_across_the_path_is_front():
    trk = mwt.MultiWallTracker()
    feed(trk, [obs(wall(2.0))], 3)
    assert trk.tracks[0].role == 'front'


def test_a_wall_the_car_drives_along_is_a_side_wall():
    """Normal along y, 1.2 m to the left, running past the car."""
    trk = mwt.MultiWallTracker()
    feed(trk, [obs(wall(1.2, nx=0.0, ny=1.0, t0=-2.0, t1=2.0))], 3)
    assert trk.tracks[0].role == 'left'
    other = mwt.MultiWallTracker()
    feed(other, [obs(wall(1.2, nx=0.0, ny=-1.0, t0=-2.0, t1=2.0))], 3)
    assert other.tracks[0].role == 'right'


def test_a_perpendicular_segment_three_metres_to_the_left_is_unknown():
    """In the x window, but its normal points along the car: not a side wall,
    not in the strip, so it is tracked and is a reference for nothing."""
    trk = mwt.MultiWallTracker()
    confirm(trk, [obs(wall(2.0, nx=1.0, ny=0.0, t0=2.0, t1=4.0))])
    track = trk.tracks[0]
    assert track.role == 'unknown'
    assert track in trk.tracks and track.ever_confirmed      # still tracked
    assert not trk.is_front(track, ORIGIN, FRONT_X)


def test_a_side_wall_outside_the_x_window_is_unknown():
    """The same wall 8 m behind the car: real, tracked, not a reference."""
    trk = mwt.MultiWallTracker()
    feed(trk, [obs(wall(1.2, nx=0.0, ny=1.0, t0=-9.0, t1=-8.0))], 3)
    assert trk.tracks[0].role == 'unknown'


def test_a_role_sticks_across_its_edge_instead_of_flipping():
    p = mwt.TrackerParams(role_hysteresis_m=0.3)
    trk = mwt.MultiWallTracker(p)
    feed(trk, [obs(wall(1.2, nx=0.0, ny=1.0, t0=-2.0, t1=2.0))], 3)
    track = trk.tracks[0]
    assert track.role == 'left'
    # Drive forward until the wall's x window is only just lost, then past it.
    trk.update_roles(Pose2D(2.9, 0.0, 0.0))
    assert track.role == 'left'
    trk.update_roles(Pose2D(3.5, 0.0, 0.0))
    assert track.role == 'unknown'


def test_a_rejected_track_is_never_a_reference_wall():
    trk = mwt.MultiWallTracker()
    feed(trk, [obs(wall(2.0))], 3)
    track = trk.tracks[0]
    assert track.role == 'front'
    verdict = {'coverage': 0.9, 'support': 0.0, 'beyond_share': 0.0}
    # Rejection needs reject_cycles verifiable cycles (F1).
    for _ in range(trk.rect.reject_cycles):
        trk.apply_rect_verdicts({track.id: verdict})
    trk.update_roles(ORIGIN)
    assert track.verify == wall_fusion.VERIFY_REJECTED and track.role == 'unknown'
    assert not trk.is_front(track, ORIGIN, FRONT_X)
    assert trk.front_track(ORIGIN, FRONT_X) is None


def test_a_role_with_no_history_is_just_the_geometry():
    trk = mwt.MultiWallTracker()
    feed(trk, [obs(wall(2.0))], 3)
    trk.tracks[0].role = ''
    assert trk.label(trk.tracks[0], ORIGIN) == 'front'


# --- 'unknown', its legacy name 'other', and the REPORTED role ---------------

def _bare_track(role, ever_confirmed=True, state=mwt.STATE_CONFIRMED):
    return mwt.Track(1, np.array([2.0, 0.0]), np.eye(2) * 0.01, -1.0, 1.0, 0.0, 0.0,
                     ever_confirmed=ever_confirmed, state=state, role=role)


def test_the_role_for_nothing_is_spelled_unknown_and_its_legacy_name_other():
    assert mwt.ROLE_UNKNOWN == 'unknown'
    assert mwt.LEGACY_ROLE_OTHER == 'other'


def test_is_unknown_role_accepts_unknown_the_legacy_other_and_no_role_yet():
    for role in ('unknown', 'other', ''):
        assert mwt.is_unknown_role(role), role
    for role in ('front', 'back', 'left', 'right'):
        assert not mwt.is_unknown_role(role), role


def test_a_tentative_track_is_reported_unknown_but_keeps_its_geometric_role():
    # Only what is REPORTED changes: the tracker's own role stays the
    # geometry's, and the same track is reported with it once confirmed.
    trk = mwt.MultiWallTracker()
    feed(trk, [obs(wall(2.0))], 1)
    track = trk.tracks[0]
    assert track.state == mwt.STATE_TENTATIVE and not track.ever_confirmed
    assert track.role == 'front'
    assert mwt.reported_role(track) == 'unknown'
    assert mwt.reported_role(track, trk.label(track, ORIGIN)) == 'unknown'
    feed(trk, [obs(wall(2.0))], N_CONFIRM - 1, t0=1.05)
    assert track.ever_confirmed and track.state == mwt.STATE_CONFIRMED
    assert mwt.reported_role(track) == 'front'


def test_a_track_still_in_the_tentative_state_is_reported_unknown_even_if_ever_confirmed():
    """A hand-built inconsistent track: the state test in is_tentative still
    guards it, although _merge no longer leaves a confirmed track TENTATIVE
    (test_wall_confirm_level.py)."""
    track = _bare_track('front', ever_confirmed=True, state=mwt.STATE_TENTATIVE)
    assert mwt.is_tentative(track)
    assert mwt.reported_role(track) == 'unknown'


@pytest.mark.parametrize('role, reported', [
    ('front', 'front'), ('back', 'back'), ('left', 'left'), ('right', 'right'),
    ('unknown', 'unknown'), ('other', 'unknown'), ('', 'unknown')])
def test_a_confirmed_track_is_reported_with_its_role_and_any_unknown_spelling_as_unknown(
        role, reported):
    for state in (mwt.STATE_CONFIRMED, mwt.STATE_COASTING):
        assert mwt.reported_role(_bare_track(role, state=state)) == reported


def test_the_front_gate_refuses_unknown_and_legacy_other_but_not_an_unclassified_role():
    # front_reason's role check is the old `role == 'other'` under the new
    # name: the legacy spelling is still refused, and '' still is not.
    trk = mwt.MultiWallTracker()
    confirm(trk, [obs(wall(2.0))])
    track = trk.tracks[0]
    assert trk.front_reason(track, ORIGIN, FRONT_X) == ''
    for role, code in (('unknown', 'role_other'), ('other', 'role_other'), ('', '')):
        track.role = role
        assert trk.front_reason(track, ORIGIN, FRONT_X) == code, role


# --- first obstacle: a wall behind a wall -----------------------------------

def test_a_segment_behind_a_confirmed_wall_is_unknown():
    trk = mwt.MultiWallTracker()
    trk.laser_xy = LASER[:2]
    confirm(trk, [obs(wall(2.0, t0=-1.5, t1=1.5)),
                  obs(wall(5.0, t0=-1.5, t1=1.5))])
    near = min(trk.tracks, key=lambda t: t.rho)
    far = max(trk.tracks, key=lambda t: t.rho)
    assert near.role == 'front'
    assert far.occluded and far.role == 'unknown'
    assert trk.front_track(ORIGIN, FRONT_X) is near


def test_glass_still_blocks_what_is_behind_it():
    trk = mwt.MultiWallTracker()
    trk.laser_xy = LASER[:2]
    confirm(trk, [obs(wall(2.0, t0=-1.5, t1=1.5)),
                  obs(wall(5.0, t0=-1.5, t1=1.5))])
    near = min(trk.tracks, key=lambda t: t.rho)
    near.glass = True
    trk.update_roles(ORIGIN)
    assert max(trk.tracks, key=lambda t: t.rho).role == 'unknown'


def test_a_wall_beside_the_near_one_is_not_occluded_by_it():
    trk = mwt.MultiWallTracker()
    trk.laser_xy = LASER[:2]
    feed(trk, [obs(wall(2.0, t0=-1.5, t1=-0.4)),
               obs(wall(5.0, t0=0.6, t1=2.0))], 4)
    far = max(trk.tracks, key=lambda t: t.rho)
    assert not far.occluded


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


# ===========================================================================
# Minimum width for a wall: the hysteretic span gate
# ===========================================================================

def line_of_span(span_m, d=2.0, y_centre=0.0, n_points=40, rms=0.01):
    """A base_link WallLine straight across the heading, `span_m` long."""
    from f1tenth_perception.scene_labels import WallLine
    return WallLine(1.0, 0.0, float(d), y_centre - span_m / 2.0, y_centre + span_m / 2.0,
                    n_points, rms, 'lidar')


def obs_of_span(span_m, **kw):
    return [wall_fusion.Observation(line_of_span(span_m, **kw), 0.0, 2.5, False,
                                    wall_fusion.SOURCE_LIDAR, 0.8)]


def test_a_short_segment_never_starts_a_wall_track():
    """The corridor phantom: a 1.39 m face 2 m ahead became a 'front' wall and
    sat there for 30 s of an empty corridor (corridor10m, track 8)."""
    trk = mwt.MultiWallTracker(mwt.TrackerParams(min_span_birth_m=1.5))
    car = Pose2D(0.0, 0.0, 0.0)
    for i in range(10):
        trk.step(obs_of_span(1.39), i * 0.05, car)
    assert trk.tracks == []
    assert trk.published(car) == []
    assert trk.n_short_dropped == 0        # 1.39 >= min_span_update_m: not dropped


def test_a_segment_shorter_than_the_update_floor_never_reaches_the_tracker():
    trk = mwt.MultiWallTracker()
    car = Pose2D(0.0, 0.0, 0.0)
    out = trk.step(obs_of_span(0.3), 0.0, car)
    assert out == []
    assert trk.tracks == []
    assert trk.n_short_dropped == 1


def test_a_long_segment_starts_a_track_as_before():
    trk = mwt.MultiWallTracker()
    car = Pose2D(0.0, 0.0, 0.0)
    for i in range(5):
        tracks = trk.step(obs_of_span(2.0), i * 0.05, car)
    assert len(tracks) == 1
    assert tracks[0].length == pytest.approx(2.0, abs=0.01)


def test_a_confirmed_wall_seen_short_keeps_being_updated():
    """The whole point of the hysteresis: a wall half-hidden behind someone is
    still that wall, and must not be dropped and re-born (a new id, a new
    tentative lifecycle, and no FRONT wall meanwhile)."""
    trk = mwt.MultiWallTracker()
    car = Pose2D(0.0, 0.0, 0.0)
    for i in range(5):
        tracks = trk.step(obs_of_span(2.0), i * 0.05, car)
    first_id, hits = tracks[0].id, tracks[0].hits
    for i in range(5, 15):
        tracks = trk.step(obs_of_span(0.6), i * 0.05, car)
    assert len(tracks) == 1
    assert tracks[0].id == first_id
    assert tracks[0].hits > hits
    assert tracks[0].state == mwt.STATE_CONFIRMED


def test_a_short_segment_cannot_feed_a_tentative_track():
    """A maybe fed only by short segments must stay a maybe and die.

    Without this a 0.6 m face could be confirmed by three 0.6 m faces: the
    birth gate would be the only thing standing in the way, and it would be
    bypassed by whichever of them happened to arrive first.
    """
    trk = mwt.MultiWallTracker()
    car = Pose2D(0.0, 0.0, 0.0)
    # One long observation starts a TENTATIVE track...
    trk.step(obs_of_span(1.2), 0.0, car)
    assert len(trk.tracks) == 1 and not trk.tracks[0].ever_confirmed
    # ...and short ones may not confirm it.
    for i in range(1, 6):
        trk.step(obs_of_span(0.6), i * 0.05, car)
    assert all(not t.ever_confirmed for t in trk.tracks)
    assert trk.published(car) == []


def test_a_short_segment_does_not_consume_the_assignment_a_long_one_needs():
    """Gated in the cost matrix, not after the assignment: the Hungarian
    solver would otherwise be free to hand the track to the short segment and
    leave the long one to start a duplicate."""
    trk = mwt.MultiWallTracker()
    car = Pose2D(0.0, 0.0, 0.0)
    for i in range(5):
        trk.step(obs_of_span(2.0), i * 0.05, car)
    assert len(trk.tracks) == 1
    confirmed = trk.tracks[0].id
    # The same wall, reported twice in one cycle: once short, once long.
    both = obs_of_span(0.6) + obs_of_span(2.0)
    trk.step(both, 0.3, car)
    assert [t.id for t in trk.tracks] == [confirmed]


def test_the_span_is_the_observations_own_extent_not_the_tracks():
    """A track's extent only ever grows, so gating on it would be a one-way
    door: the first long observation would license every later short one."""
    trk = mwt.MultiWallTracker(mwt.TrackerParams(min_span_birth_m=1.0,
                                                 min_span_update_m=0.5))
    car = Pose2D(0.0, 0.0, 0.0)
    # Confirmed, so the short observation's one way in (an UPDATE of a
    # confirmed track) is open, and a miss cannot end the track either.
    for i in range(N_CONFIRM):
        trk.step(obs_of_span(3.0), i * 0.05, car)
    long_track = trk.tracks[0]
    assert long_track.ever_confirmed
    assert long_track.length == pytest.approx(3.0, abs=0.01)
    hits = long_track.hits
    # A 0.3 m observation of the SAME line is still dropped, although the
    # track it would match is 3 m long.
    trk.step(obs_of_span(0.3), N_CONFIRM * 0.05, car)
    assert trk.n_short_dropped == 1
    assert long_track.hits == hits


def test_the_real_walls_of_a_room_still_survive_the_gate():
    """The gate must not cost the pipeline what it exists to track.

    Every wall the laser sees along its whole length is still tracked; what it
    costs is the wall BEHIND the car, which the +-135 deg FOV only ever shows
    as two 0.70 m arcs (test_room_from_two_poses_... says the rest).
    """
    trk = mwt.MultiWallTracker()
    car = Pose2D(2.0, 3.0, 0.0)
    tracks = run(trk, car, ROOM)
    assert len(tracks) == 3
    assert set(by_label(trk, tracks, car)) == {'front', 'left', 'right'}
    assert all(t.length > 5.0 for t in tracks)


def test_a_chair_leg_in_an_otherwise_empty_room_starts_no_wall():
    """Raycast, not a hand-built line: the extraction's own output through the
    gate. A 0.12 m radius circle 1.5 m ahead is about 0.3 m of visible arc."""
    trk = mwt.MultiWallTracker()
    car = Pose2D(0.0, 0.0, 0.0)
    for i in range(12):
        tracks = step(trk, car, i * 0.05, [], circles=[((1.5, 0.0), 0.12)])
    assert tracks == []


def test_the_collinear_gap_is_one_number_the_narrowest_door():
    """
    Default the extraction's merge, the tracker and the doors to one constant.

    THE COLLINEAR GAP RULE: never corridor_perception's own 1.5 m.
    """
    from f1tenth_perception.door_detection import DoorParams
    assert mwt.DOOR_WIDTH_MIN_M == pytest.approx(0.70)
    assert mwt.extraction_params(None).merge_gap == pytest.approx(mwt.DOOR_WIDTH_MIN_M)
    assert mwt.extraction_params(0.5).merge_gap == pytest.approx(0.5)
    assert mwt.TrackerParams().collinear_gap_m == pytest.approx(mwt.DOOR_WIDTH_MIN_M)
    assert DoorParams().width_min_m == pytest.approx(mwt.DOOR_WIDTH_MIN_M)


def test_the_code_default_door_width_is_the_stack_params_value():
    """
    DOOR_WIDTH_MIN_M is lidar_front_wall_door_width_min_m, the deployed number.

    The node overrides the code default from stack_params.yaml, but the
    offline tools (tools/door_replay.py, tools/extract_backends_equiv.py,
    tools/wall_foot_eval.py) and these tests run on the code default: a
    retune of one without the other would have them check a different rule
    from the car's. Change both in the same commit.
    """
    from f1tenth_params.param_defaults import get_value
    assert mwt.DOOR_WIDTH_MIN_M == pytest.approx(get_value('lidar_front_wall_door_width_min_m'))


def test_a_wall_with_a_person_in_front_of_it_is_merged_before_it_is_measured():
    """The merge gap decides whether a hole splits the extraction's wall.

    A person 1 m in front of a 6 m wall hides part of it and splits it in
    two. The merge is what makes the span the WALL's: with the gap wider than
    the hole the extraction reports one wall, tightened below it two shorter
    ones. (Since THE COLLINEAR GAP RULE the stack's gap is door_width_min_m,
    0.70 m: a hole that wide or wider stays split in the extraction, and the
    tracker reunites it -- its span gate measures the pieces together (THE
    WALL'S SPAN) and its association counts only the open gap.)
    """
    car = Pose2D(0.0, 0.0, 0.0)
    wall = [((4.0, -3.0), (4.0, 3.0))]
    person = [((3.0, -0.25), 0.25)]
    wide = mwt.extract_lines(raycast(car, wall, person), ANGLE_MIN, INC, 0.06, 30.0,
                             LASER, params=mwt.extraction_params(1.5))
    tight = mwt.extract_lines(raycast(car, wall, person), ANGLE_MIN, INC, 0.06, 30.0,
                              LASER, params=mwt.extraction_params(0.05))
    wide_wall = [ln for ln in wide if abs(ln.rho - 4.0) < 0.2]
    tight_wall = [ln for ln in tight if abs(ln.rho - 4.0) < 0.2]
    assert len(wide_wall) == 1
    assert max(ln.length for ln in wide_wall) > max(ln.length for ln in tight_wall)


def test_a_stale_profile_moves_the_fused_distance_less_than_a_fresh_one():
    """What the weight buys in the filter: the stereo sigma is divided by it,
    so the inverse-variance combination leans on the LiDAR in exactly the
    proportion the age says."""
    p = mwt.TrackerParams()
    fresh = mwt.observation_covariance(wall_fusion.SOURCE_BOTH, 6.0, 3.0, p, 1.0)
    stale = mwt.observation_covariance(wall_fusion.SOURCE_BOTH, 6.0, 3.0, p, 0.3)
    lidar_only = mwt.observation_covariance(wall_fusion.SOURCE_LIDAR, 6.0, 3.0, p)
    # Tighter than the LiDAR alone when fresh, and closer to it when stale.
    assert fresh[0, 0] < stale[0, 0] <= lidar_only[0, 0]
    assert stale[0, 0] / lidar_only[0, 0] > 0.9


def test_a_zero_weight_is_the_lidars_own_covariance():
    p = mwt.TrackerParams()
    none = mwt.observation_covariance(wall_fusion.SOURCE_BOTH, 6.0, 3.0, p, 0.0)
    lidar_only = mwt.observation_covariance(wall_fusion.SOURCE_LIDAR, 6.0, 3.0, p)
    assert none[0, 0] == pytest.approx(lidar_only[0, 0])


def test_the_weight_reaches_the_covariance_through_the_observation():
    """It is carried on the Observation, so nothing in the tracker has to be
    told the age separately."""
    car = Pose2D(0.0, 0.0, 0.0)
    obs = wall_fusion.Observation(
        line_of_span(2.0, d=6.0), 0.0, 2.5, False, wall_fusion.SOURCE_BOTH, 0.8,
        stereo_weight=0.3)
    fresh = wall_fusion.Observation(
        line_of_span(2.0, d=6.0), 0.0, 2.5, False, wall_fusion.SOURCE_BOTH, 0.8,
        stereo_weight=1.0)
    p = mwt.TrackerParams()
    assert mwt.observation_line(obs, car, p).cov[0, 0] > \
        mwt.observation_line(fresh, car, p).cov[0, 0]


def test_an_observation_without_a_weight_is_treated_as_fresh():
    """Nothing that builds an Observation without one changes behaviour."""
    car = Pose2D(0.0, 0.0, 0.0)
    bare = wall_fusion.Observation(line_of_span(2.0, d=6.0), 0.0, 2.5, False,
                                   wall_fusion.SOURCE_BOTH, 0.8)
    assert bare.stereo_weight == pytest.approx(1.0)
    p = mwt.TrackerParams()
    explicit = wall_fusion.Observation(line_of_span(2.0, d=6.0), 0.0, 2.5, False,
                                       wall_fusion.SOURCE_BOTH, 0.8, stereo_weight=1.0)
    assert mwt.observation_line(bare, car, p).cov[0, 0] == pytest.approx(
        mwt.observation_line(explicit, car, p).cov[0, 0])


# --- negative evidence ------------------------------------------------------

def neg_scan(ranges_of_bearing, laser_xy=LASER[:2]):
    """(ranges, base_link bearings, laser xy) -- mwt.scan_in_base_link's output.

    ranges_of_bearing(bearings) returns the range each beam reports, so a test
    says what the LiDAR saw rather than building a world for it to see.
    """
    bearings = ANGLE_MIN + np.arange(N_BEAMS) * INC
    return np.asarray(ranges_of_bearing(bearings), dtype=float), bearings, laser_xy


def on_the_line(d, laser_x=LASER[0]):
    """The range at which each beam meets the line x = d in base_link."""
    def ranges(bearings):
        with np.errstate(divide='ignore', invalid='ignore'):
            s = (d - laser_x) / np.cos(bearings)
        return np.where(s > 0.0, s, NO_RETURN)
    return ranges


def test_beams_passing_through_a_wall_kill_it_in_five_scans():
    trk = mwt.MultiWallTracker()
    confirm(trk, [obs(wall(2.0))])
    track = trk.tracks[0]
    assert track.ever_confirmed
    # Everything in the wall's span now reads 6 m: the beams are going
    # straight through where the wall is supposed to be.
    through = neg_scan(lambda b: np.full(b.size, 6.0))
    for i in range(4):
        trk.step([], 2.0 + 0.05 * i, ORIGIN, scan=through)
        assert track in trk.tracks, i
        assert track.miss == i + 1
    trk.step([], 2.2, ORIGIN, scan=through)
    assert track not in trk.tracks
    assert trk.n_negative_killed == 1


def test_beams_landing_on_the_wall_are_not_negative_evidence():
    trk = mwt.MultiWallTracker()
    feed(trk, [obs(wall(2.0))], 3)
    track = trk.tracks[0]
    hits = neg_scan(on_the_line(2.0))
    for i in range(6):
        trk.step([obs(wall(2.0))], 2.0 + 0.05 * i, ORIGIN, scan=hits)
    assert track in trk.tracks and track.miss == 0


def test_one_good_scan_resets_the_count():
    trk = mwt.MultiWallTracker()
    confirm(trk, [obs(wall(2.0))])
    track = trk.tracks[0]
    through = neg_scan(lambda b: np.full(b.size, 6.0))
    for i in range(3):
        trk.step([], 2.0 + 0.05 * i, ORIGIN, scan=through)
    assert track.miss == 3
    trk.step([obs(wall(2.0))], 2.2, ORIGIN, scan=neg_scan(on_the_line(2.0)))
    assert track.miss == 0 and track in trk.tracks


def test_glass_is_never_killed_by_beams_going_through_it():
    """Glass is the surface the beams are SUPPOSED to pass through."""
    trk = mwt.MultiWallTracker()
    confirm(trk, [obs(wall(2.0))])
    track = trk.tracks[0]
    track.verify, track.glass = wall_fusion.VERIFY_VERIFIED, True
    through = neg_scan(lambda b: np.full(b.size, 6.0))
    for i in range(8):
        trk.step([], 2.0 + 0.02 * i, ORIGIN, scan=through)
        if track not in trk.tracks:
            pytest.fail(f'the pane was killed on scan {i}')
    assert track.miss == 0


def test_a_beam_blocked_by_something_nearer_says_nothing():
    """Short returns are an occluder, not evidence against the wall."""
    trk = mwt.MultiWallTracker()
    confirm(trk, [obs(wall(2.0))])
    track = trk.tracks[0]
    short = neg_scan(lambda b: np.full(b.size, 1.0))
    for i in range(8):
        trk.step([], 2.0 + 0.02 * i, ORIGIN, scan=short)
    assert track in trk.tracks and track.miss == 0


def test_a_tentative_track_is_not_judged_by_negative_evidence():
    """It has its own lifecycle; the beams only overrule a CONFIRMED wall."""
    trk = mwt.MultiWallTracker()
    trk.step([obs(wall(2.0))], 1.0, ORIGIN)
    track = trk.tracks[0]
    assert not track.ever_confirmed
    trk.step([], 1.05, ORIGIN, scan=neg_scan(lambda b: np.full(b.size, 6.0)))
    assert track.miss == 0


# --- coasting ---------------------------------------------------------------

def test_a_coasting_track_dies_after_two_seconds():
    trk = mwt.MultiWallTracker()
    confirm(trk, [obs(wall(2.0))])
    track = trk.tracks[0]
    trk.step([], 1.1 + 1.9, ORIGIN)
    assert track in trk.tracks
    trk.step([], 1.1 + 2.1, ORIGIN)
    assert track not in trk.tracks


def test_a_coasting_track_dies_after_a_metre_of_odometry():
    """Standing still costs a wall nothing; driving past it does."""
    trk = mwt.MultiWallTracker()
    confirm(trk, [obs(wall(2.0))])
    track = trk.tracks[0]
    trk.step([], 1.2, Pose2D(0.9, 0.0, 0.0))
    assert track in trk.tracks
    trk.step([], 1.25, Pose2D(1.1, 0.0, 0.0))
    assert track not in trk.tracks


def test_the_odometer_is_path_length_not_displacement():
    trk = mwt.MultiWallTracker()
    feed(trk, [obs(wall(2.0))], 3)
    track = trk.tracks[0]
    for i, x in enumerate((0.4, 0.0, 0.4, 0.0)):
        trk.step([], 1.2 + 0.05 * i, Pose2D(x, 0.0, 0.0))
    assert track not in trk.tracks      # 1.6 m walked, 0 m displaced


def test_only_the_pairs_whose_sectors_overlap_are_tested_for_occlusion():
    """F3: a wall to the left cannot hide a wall to the right.

    The sector test is a pure cost optimisation, so what this pins is that it
    changes no verdict: the occluded set is the same as the all-pairs answer
    on a scene built to have both kinds of pair.
    """
    trk = mwt.MultiWallTracker()
    trk.laser_xy = LASER[:2]
    confirm(trk, [obs(wall(2.0, t0=-1.5, t1=1.5)),      # near, dead ahead
                  obs(wall(5.0, t0=-1.5, t1=1.5)),      # behind it: occluded
                  obs(wall(3.0, nx=0.0, ny=1.0, t0=-2.0, t1=2.0)),   # left wall
                  obs(wall(3.0, nx=0.0, ny=-1.0, t0=-2.0, t1=2.0))])  # right wall
    by_rho = {round(t.rho, 1): t for t in trk.tracks}
    assert by_rho[5.0].occluded          # behind the near wall
    assert not by_rho[2.0].occluded      # nothing in front of it
    # The two side walls share no bearing with each other.
    assert not by_rho[3.0].occluded


def test_the_sector_helpers_agree_about_a_shared_bearing():
    left = mwt._bearing_sector(((2.0, 0.5), (2.0, 2.0)), (0.0, 0.0))
    right = mwt._bearing_sector(((2.0, -2.0), (2.0, -0.5)), (0.0, 0.0))
    ahead = mwt._bearing_sector(((4.0, -1.0), (4.0, 1.0)), (0.0, 0.0))
    assert not mwt._sectors_overlap(left, right)
    assert mwt._sectors_overlap(left, ahead) and mwt._sectors_overlap(right, ahead)


# --- H2: does association survive a 10 Hz cycle? ----------------------------

def wall_in_base_link(car, rho_odom=3.0, theta_odom=0.0, t0=-1.5, t1=1.5):
    """The observation a FIXED odom wall produces from `car`.

    The tracker is given observations in base_link and places them in odom
    with the pose at the scan stamp, so this is the only honest way to ask
    whether association survives motion: move the car, re-derive what it would
    see, and hand that in.
    """
    from corridor_perception.geometry import Line
    n = np.array([math.cos(theta_odom), math.sin(theta_odom)])
    tan = np.array([-n[1], n[0]])
    p0 = n * rho_odom + tan * t0
    p1 = n * rho_odom + tan * t1
    odom = Line(rho_odom, theta_odom, np.zeros((2, 2)), p0, p1, 120, 'odom')
    bl = odom.transform(car.inverse(), 'base_link')
    return obs(mwt.line_to_wall_line(bl))


def test_association_survives_four_centimetres_a_cycle_at_ten_hz():
    """0.4 m/s at a decimated 10 Hz is 4 cm of travel between tracker steps."""
    trk = mwt.MultiWallTracker()
    car = Pose2D(0.0, 0.0, 0.0)
    confirm(trk, [wall_in_base_link(car)], car=car)
    track = trk.tracks[0]
    assert track.ever_confirmed
    hits = track.hits
    for i in range(1, 21):
        car = Pose2D(0.04 * i, 0.0, 0.0)
        trk.step([wall_in_base_link(car)], 1.1 + 0.1 * i, car)
        assert len(trk.tracks) == 1, f'a second track appeared at cycle {i}'
        assert trk.tracks[0].id == track.id, f'the track was replaced at cycle {i}'
    assert track.hits == hits + 20            # every cycle associated
    # 80 cm of travel later the wall is still where the world put it.
    assert track.rho == pytest.approx(3.0, abs=0.02)


def test_association_survives_ten_degrees_a_cycle_at_ten_hz():
    """The worst case the decimation creates: at the steering limit the car
    turns ~10 deg between tracker steps, and k_yaw adds (k_yaw |dyaw| rho)^2
    to the track's rho variance for it."""
    trk = mwt.MultiWallTracker()
    car = Pose2D(0.0, 0.0, 0.0)
    feed(trk, [wall_in_base_link(car)], 3, car=car)
    track = trk.tracks[0]
    for i in range(1, 9):
        car = Pose2D(0.0, 0.0, math.radians(10.0 * i))
        trk.step([wall_in_base_link(car)], 1.1 + 0.1 * i, car)
        assert len(trk.tracks) == 1, f'a second track appeared at cycle {i}'
        assert trk.tracks[0].id == track.id, f'the track was replaced at cycle {i}'
    assert track.rho == pytest.approx(3.0, abs=0.05)


def test_a_far_wall_coasting_through_a_turn_is_the_decimation_s_real_limit():
    """What the yaw term costs a FAR track when a cycle is missed.

    k_yaw adds (k_yaw |dyaw| rho)^2 per predict, and max_sigma_rho_m deletes
    the track above 0.3 m. At 15.5 m and 10 deg of yaw in one step that is
    0.27 m from a single predict -- inside the limit, but ONE more unmatched
    cycle at that yaw rate deletes a wall that a 40 Hz tracker would have
    kept. This test states the number rather than asserting it is fine.
    """
    p = mwt.TrackerParams()
    rho = 15.5
    one = p.k_yaw * math.radians(10.0) * rho
    assert one == pytest.approx(0.271, abs=0.002)
    assert one < p.max_sigma_rho_m                      # survives one
    assert math.sqrt(2.0) * one > p.max_sigma_rho_m     # not two
    # At the undecimated 40 Hz the same turn is four steps of 2.5 deg, which
    # is HALF the sigma, because the model adds squares per step.
    four = math.sqrt(4.0 * (p.k_yaw * math.radians(2.5) * rho) ** 2)
    assert four == pytest.approx(0.5 * one, rel=1e-6)


# --- H3: the publish cap, by role -------------------------------------------

def test_the_front_wall_is_never_the_one_the_cap_drops():
    """The defect H3 fixes: a short far wall dead ahead ranked below the long
    near side walls, because the old ranking was length * hits / distance."""
    p = mwt.TrackerParams(max_published=4)
    trk = mwt.MultiWallTracker(p)
    trk.laser_xy = LASER[:2]
    # Three long near side walls and one short wall 15.5 m dead ahead.
    obss = [obs(wall(1.2, nx=0.0, ny=1.0, t0=-4.0, t1=4.0, inliers=400)),
            obs(wall(1.2, nx=0.0, ny=-1.0, t0=-4.0, t1=4.0, inliers=400)),
            obs(wall(1.3, nx=0.0, ny=1.0, t0=-4.0, t1=4.0, inliers=400)),
            obs(wall(15.5, t0=-1.5, t1=1.5, inliers=60))]
    confirm(trk, obss)
    published = trk.published(ORIGIN)
    front = [t for t in trk.tracks if t.role == 'front']
    assert front, 'the test needs a front wall to be hidden'
    assert front[0] in published, 'the FRONT wall must never lose its slot'
    assert trk.front_track(ORIGIN, FRONT_X) is front[0]


def test_the_budget_is_front_then_nearest_left_then_nearest_right():
    p = mwt.TrackerParams(max_published=4)
    trk = mwt.MultiWallTracker(p)
    trk.laser_xy = LASER[:2]
    confirm(trk, [obs(wall(5.0, t0=-1.5, t1=1.5)),
                  obs(wall(1.2, nx=0.0, ny=1.0, t0=-3.0, t1=3.0)),
                  obs(wall(2.4, nx=0.0, ny=1.0, t0=-3.0, t1=3.0)),
                  obs(wall(1.1, nx=0.0, ny=-1.0, t0=-3.0, t1=3.0)),
                  obs(wall(2.6, nx=0.0, ny=-1.0, t0=-3.0, t1=3.0))])
    published = trk.published(ORIGIN)
    roles = [t.role for t in published]
    assert roles[0] == 'front'
    assert roles[1] == 'left' and roles[2] == 'right'
    # the NEAREST of each side, measured from the bodywork
    lefts = sorted((t for t in trk.tracks if t.role == 'left'), key=lambda t: t.rho)
    rights = sorted((t for t in trk.tracks if t.role == 'right'), key=lambda t: t.rho)
    assert published[1] is lefts[0] and published[2] is rights[0]


def test_unknown_never_takes_a_slot_even_when_the_budget_is_free():
    trk = mwt.MultiWallTracker(mwt.TrackerParams(max_published=4))
    confirm(trk, [obs(wall(2.0, nx=1.0, ny=0.0, t0=2.0, t1=4.0))])
    track = trk.tracks[0]
    assert track.role == 'unknown' and track.ever_confirmed
    assert trk.published(ORIGIN) == []


def test_a_rejected_front_wall_gives_its_slot_up():
    trk = mwt.MultiWallTracker(mwt.TrackerParams(max_published=4))
    confirm(trk, [obs(wall(2.0))])
    track = trk.tracks[0]
    assert trk.published(ORIGIN) == [track]
    verdict = {'coverage': 0.9, 'support': 0.0, 'beyond_share': 0.0}
    for _ in range(trk.rect.reject_cycles):
        trk.apply_rect_verdicts({track.id: verdict})
    trk.update_roles(ORIGIN)
    assert track.role == 'unknown'
    assert trk.published(ORIGIN) == []


# ---------------------------------------------------------------------------
# cpu-cleanup R2: the EKF at 20 Hz -- PoseHistory's extrapolation window is a
# parameter (lidar_front_wall_pose_max_extrapolation_sec, 0.15 s)
# ---------------------------------------------------------------------------

def test_pose_history_extrapolates_up_to_the_given_window_only():
    hist = mwt.PoseHistory()
    hist.add(10.00, 0.0, 0.0, 0.0)
    hist.add(10.05, 0.05, 0.0, 0.0)          # 20 Hz
    # A scan 120 ms past the newest pose: inside 3 periods at 20 Hz.
    assert hist.at(10.17) is None                         # old default 0.1 s
    assert hist.at(10.17, 0.15).x == pytest.approx(0.05)  # new 0.15 s
    assert hist.at(10.21, 0.15) is None
    assert hist.at(10.025, 0.15).x == pytest.approx(0.025)  # interpolation unchanged


# ===========================================================================
# Block 3: FRONT is the MEASURED extent, judged by the strip's own beams
# (corridor10m_20261016T144537, t = 145-150.3 s: track 955, 13.67 m, built
# from the two faces either side of a corridor mouth, held FRONT over a strip
# that read 6.75-6.96 m and stopped the car ~5 m early)
# ===========================================================================

# The fixture's geometry: base_link at the car pose, the laser at x 0.305,
# the bumper at FRONT_X 0.405. A wall at x = 3.5 is therefore 3.5 - 0.405 =
# 3.095 m from the bumper (the "~3.4 m" of the incident notes is the same wall
# measured from a bumper ~0.1 m from base_link; this fixture's is 0.405 m).
WALL_X = 3.5
PIECES = [((WALL_X, 1.2), (WALL_X, 6.0)), ((WALL_X, -6.0), (WALL_X, -1.2))]
HALL = [((7.3, -1.2), (7.3, 1.2))]
CLEAR_M = 50.0


def contract(trk):
    """(state, value) the node would publish for this tracker's last front_track()."""
    return wall_fusion.front_wall_output(True, trk.front_status['distance_m'], CLEAR_M)


def test_two_collinear_pieces_either_side_of_an_opening_are_not_one_front_wall():
    """The incident's geometry. The old _merge joined the two pieces on the
    (rho, theta) gate alone and the union of their extents crossed the strip:
    FRONT at 3.1 m over an empty strip."""
    trk = mwt.MultiWallTracker()
    trk.laser_xy = LASER[:2]
    car = Pose2D(0.0, 0.0, 0.0)
    for i in range(6):
        step(trk, car, i * 0.1, PIECES)
        if i == 0:
            # Both pieces spawn on the first cycle; the merge is refused.
            assert trk.n_merge_gap_refused == 1
            assert trk.rejections[0][0] == 'gap_merge'
    confirmed = [t for t in trk.tracks if t.ever_confirmed]
    assert len(confirmed) == 2
    assert all(abs(t.t1 - t.t0) < 5.0 for t in confirmed)   # never 12 m
    assert trk.front_track(car, FRONT_X) is None
    assert trk.front_status['reason'] == 'no_candidate'
    assert trk.strip.range_m is None                         # nothing in the strip
    assert contract(trk) == (wall_fusion.STATE_CLEAR, CLEAR_M)


def test_through_the_opening_the_front_wall_is_the_hall_wall_never_the_phantom():
    """Skeptic #5: a strip that reads 6.9 m is a SURFACE, and the right answer
    is WALL at that surface, not CLEAR and never the 3.1 m line."""
    trk = mwt.MultiWallTracker()
    trk.laser_xy = LASER[:2]
    car = Pose2D(0.0, 0.0, 0.0)
    for i in range(6):
        step(trk, car, i * 0.1, PIECES + HALL)
    front = trk.front_track(car, FRONT_X)
    assert front is not None and front.rho == pytest.approx(7.3, abs=0.05)
    assert trk.front_status['distance_m'] == pytest.approx(7.3 - FRONT_X, abs=0.05)
    assert trk.strip.range_m == pytest.approx(7.3 - FRONT_X, abs=0.05)


def test_a_real_front_wall_across_the_strip_is_front_from_the_bumper():
    trk = mwt.MultiWallTracker()
    car = Pose2D(0.0, 0.0, 0.0)
    for i in range(5):
        step(trk, car, i * 0.1, [((WALL_X, -3.0), (WALL_X, 3.0))])
    front = trk.front_track(car, FRONT_X)
    assert front is not None and trk.front_status['reason'] == 'front'
    assert trk.front_status['distance_m'] == pytest.approx(WALL_X - FRONT_X, abs=0.03)
    assert front.strip_verdict == mwt.STRIP_SUPPORTED
    state, value = contract(trk)
    assert state == wall_fusion.STATE_WALL and value == pytest.approx(3.095, abs=0.03)


def _confirmed_front_at_3_5(trk, car, n=N_CONFIRM):
    for i in range(n):
        step(trk, car, i * 0.1, [((WALL_X, -3.0), (WALL_X, 3.0))])
    front = trk.front_track(car, FRONT_X)
    assert front is not None
    return front, (n - 1) * 0.1


def test_a_front_track_losing_its_lidar_fit_over_a_strip_3_m_longer_is_dropped():
    """The wall is gone and the strip reads +3 m (a 0.6 m stub at 6.5 m, too
    short to be a wall itself). Coasting, the first contradicting scan already
    ends its FRONT; the second marks it CONTRADICTED."""
    trk = mwt.MultiWallTracker()
    car = Pose2D(0.0, 0.0, 0.0)
    front, t = _confirmed_front_at_3_5(trk, car)
    beyond = [((6.5, -0.3), (6.5, 0.3))]
    step(trk, car, t + 0.1, beyond)
    assert trk.strip.range_m == pytest.approx(6.5 - FRONT_X, abs=0.03)
    assert trk.front_track(car, FRONT_X) is None
    assert trk.front_status['rejects'][front.id] == 'coast_contradicted'
    step(trk, car, t + 0.2, beyond)
    assert front.contradicted and front.role == 'unknown'
    assert trk.front_track(car, FRONT_X) is None
    assert contract(trk) == (wall_fusion.STATE_CLEAR, CLEAR_M)


def test_a_front_track_fed_only_by_stereo_over_a_strip_3_m_longer_drops_in_two_scans():
    """Not coasting -- a stereo line keeps associating with it -- but no LiDAR
    fit: the owner's rule, two consecutive contradicting scans."""
    trk = mwt.MultiWallTracker()
    car = Pose2D(0.0, 0.0, 0.0)
    front, t = _confirmed_front_at_3_5(trk, car)
    stereo = [obs(wall(WALL_X, t0=-3.0, t1=3.0), wall_fusion.SOURCE_STEREO)]
    beyond = [((6.5, -0.3), (6.5, 0.3))]
    step(trk, car, t + 0.1, beyond, observations=stereo)
    assert front.state == mwt.STATE_CONFIRMED and front.strip_miss == 1
    step(trk, car, t + 0.2, beyond, observations=stereo)
    assert front.contradicted
    assert trk.front_track(car, FRONT_X) is None
    assert trk.front_status['rejects'][front.id] == 'contradicted'


def test_a_bridge_the_extraction_built_is_contradicted_and_the_hall_wall_takes_over():
    """The single-scan bridge (the extraction's 1.5 m merge can join faces a
    shorter opening apart): an observation spanning the opening, LiDAR-fitted
    every cycle. The strip's beams pass through it to the hall wall; once it
    is CONTRADICTED it stops occluding, and the hall wall is FRONT."""
    trk = mwt.MultiWallTracker()
    trk.laser_xy = LASER[:2]
    car = Pose2D(0.0, 0.0, 0.0)
    bridge = [obs(wall(WALL_X, t0=-6.0, t1=6.0))]
    for i in range(6):
        step(trk, car, i * 0.1, PIECES + HALL, extra=bridge)
    phantom = min((t for t in trk.tracks if t.ever_confirmed), key=lambda t: t.rho)
    assert phantom.rho == pytest.approx(WALL_X, abs=0.05) and phantom.contradicted
    front = trk.front_track(car, FRONT_X)
    assert front is not None and front.rho == pytest.approx(7.3, abs=0.05)


def test_a_stereo_observation_at_rho_0_2_is_the_chassis_and_is_dropped():
    """23 throwaway stereo 'right' tracks at rho_bl 0.20 m in the incident run."""
    trk = mwt.MultiWallTracker()
    chassis = obs(wall(0.2, nx=0.0, ny=-1.0, t0=0.0, t1=1.2), wall_fusion.SOURCE_STEREO)
    for i in range(8):
        trk.step([chassis], 1.0 + 0.1 * i, ORIGIN)
        assert trk.n_stereo_near_rejected == 1
    assert trk.tracks == []
    assert trk.rejections == [('stereo_near', None, 0.2)]
    # The LiDAR at the same place is a real surface touching the car.
    lidar = mwt.MultiWallTracker()
    lidar.step([obs(wall(0.2, nx=0.0, ny=-1.0, t0=0.0, t1=1.2))], 1.0, ORIGIN)
    assert len(lidar.tracks) == 1 and lidar.n_stereo_near_rejected == 0


def test_an_observation_past_the_collinear_gap_starts_its_own_track():
    p = mwt.TrackerParams()
    trk = mwt.MultiWallTracker(p)
    confirm(trk, [obs(wall(2.0, t0=-2.5, t1=-1.0))])
    first = trk.tracks[0]
    # Same line, 1.0 m beyond the extent: refused, counted, its own track.
    trk.step([obs(wall(2.0, t0=0.0, t1=1.5))], 1.2, ORIGIN)
    assert trk.n_gap_rejected == 1
    assert ('gap_assoc', first.id, 1.0) in trk.rejections
    assert len(trk.tracks) == 2 and first.t1 == pytest.approx(-1.0, abs=0.01)
    # 0.4 m beyond: inside the gap, the same wall, and the extent follows it.
    near = mwt.MultiWallTracker(p)
    confirm(near, [obs(wall(2.0, t0=-2.5, t1=-1.0))])
    near.step([obs(wall(2.0, t0=-0.6, t1=0.9))], 1.2, ORIGIN)
    assert len(near.tracks) == 1 and near.n_gap_rejected == 0
    assert near.tracks[0].t1 == pytest.approx(0.9, abs=0.01)


def test_a_gap_hidden_by_a_detected_person_is_not_an_opening():
    """The gap is counted only where the beams went THROUGH. Behind a person
    (a cut sector) the wall is unobserved, not open: the M02 case, and why
    test_person_in_front_of_the_wall_... still gets one wall."""
    trk = mwt.MultiWallTracker()
    car = Pose2D(0.0, 0.0, 0.0)
    wall_seg = [((4.0, -3.0), (4.0, 3.0))]
    for i in range(5):
        step(trk, car, i * 0.1, wall_seg, circles=[((2.0, 0.0), 0.25)],
             objects=[(2.0, 0.0, 0.25)])
    assert len(trk.tracks) == 1 and trk.n_merge_gap_refused == 0
    assert trk.tracks[0].length > 5.5
    assert trk.front_track(car, FRONT_X) is trk.tracks[0]


def test_an_end_nobody_re_observes_decays_after_the_parameter():
    p = mwt.TrackerParams()
    trk = mwt.MultiWallTracker(p)
    feed(trk, [obs(wall(2.0, t0=-3.0, t1=3.0))], 3, t0=1.0, dt=0.1)
    track = trk.tracks[0]
    # From now on only the left half is seen.
    for i in range(1, 16):
        trk.step([obs(wall(2.0, t0=-3.0, t1=0.0))], 1.2 + 0.1 * i, ORIGIN)
        if 1.2 + 0.1 * i - 1.2 <= p.extent_decay_s - 0.05:
            assert track.t1 == pytest.approx(3.0, abs=0.02)
    assert track.t1 == pytest.approx(0.0, abs=0.02)
    assert track.t0 == pytest.approx(-3.0, abs=0.02)


def test_a_coast_holds_front_only_over_an_empty_strip_and_only_for_the_caps():
    p = mwt.TrackerParams()
    car = Pose2D(0.0, 0.0, 0.0)
    nothing = scan_of(np.full(N_BEAMS, NO_RETURN))
    # Empty strip (glass, out of range): FRONT holds while coasting...
    trk = mwt.MultiWallTracker(p)
    front, t = _confirmed_front_at_3_5(trk, car)
    trk.step([], t + 0.1, car, scan=nothing)
    assert trk.front_track(car, FRONT_X) is front and front.state == mwt.STATE_COASTING
    assert front.strip_verdict == mwt.STRIP_NO_RETURNS
    # ...but no longer than front_lidar_recent_cycles cycles without a LiDAR
    # fit (unverified), nor past front_max_coast_s.
    trk.step([], t + 0.2, car, scan=nothing)
    trk.step([], t + 0.3, car, scan=nothing)
    assert trk.front_track(car, FRONT_X) is None
    assert trk.front_status['rejects'][front.id] == 'unverified'
    trk.step([], t + 0.1 + p.front_max_coast_s + 0.05, car, scan=nothing)
    assert trk.front_track(car, FRONT_X) is None
    assert trk.front_status['rejects'][front.id] == 'coast_expired'
    # On odometry: 0.35 m driven on a prediction is past front_max_coast_m.
    moved = mwt.MultiWallTracker(p)
    front, t = _confirmed_front_at_3_5(moved, car)
    moved.step([], t + 0.1, Pose2D(0.35, 0.0, 0.0), scan=nothing)
    assert moved.front_track(Pose2D(0.35, 0.0, 0.0), FRONT_X) is None
    assert moved.front_status['rejects'][front.id] == 'coast_expired'


def test_cut_beams_are_not_free_space_in_the_strip():
    bearings = ANGLE_MIN + np.arange(N_BEAMS) * INC
    cut = np.abs(bearings) < math.radians(30.0)
    # Every strip beam cut (a detected object right ahead), the rest return
    # nothing: the strip has NO return, never "free to infinity".
    scan = (np.full(N_BEAMS, np.inf), bearings, LASER[:2], cut)
    reading = mwt.strip_returns(scan, FRONT_X, 0.286)
    assert reading.range_m is None and reading.n_cut > 0
    assert mwt.strip_verdict(3.0, reading.range_m, 0.5) == mwt.STRIP_NO_RETURNS
    # A cut beam's range is never used, even if one slipped through finite.
    leaked = (np.where(cut, 9.0, np.inf), bearings, LASER[:2], cut)
    assert mwt.strip_returns(leaked, FRONT_X, 0.286).range_m is None


def test_cut_beams_are_not_negative_evidence():
    """The pre-block-3 defect: a cut beam is inf, and inf counted as a PASS."""
    trk = mwt.MultiWallTracker()
    confirm(trk, [obs(wall(2.0))])
    track = trk.tracks[0]
    r, b, lxy = neg_scan(lambda x: np.full(x.size, np.inf))
    every = np.ones(N_BEAMS, dtype=bool)
    for i in range(6):
        trk.step([], 2.0 + 0.05 * i, ORIGIN, scan=(r, b, lxy, every))
    assert track in trk.tracks and track.miss == 0
    # The same beams as genuine no-returns are still a pass (a wall returns).
    trk.step([], 2.5, ORIGIN, scan=(r, b, lxy))
    assert track.miss == 1


def test_the_contract_is_clear_without_a_front_and_no_data_only_without_data():
    trk = mwt.MultiWallTracker()
    car = Pose2D(0.0, 0.0, 0.0)
    front, t = _confirmed_front_at_3_5(trk, car)
    step(trk, car, t + 0.1, [((6.5, -0.3), (6.5, 0.3))])
    step(trk, car, t + 0.2, [((6.5, -0.3), (6.5, 0.3))])
    assert trk.front_track(car, FRONT_X) is None and front.contradicted
    # A dropped (contradicted) front is CLEAR, not NO_DATA: the data is there.
    assert contract(trk) == (wall_fusion.STATE_CLEAR, CLEAR_M)
    assert wall_fusion.front_wall_output(False, math.nan, CLEAR_M) == (
        wall_fusion.STATE_NO_DATA, -1.0)
    # The debug reason: no candidate but returns inside r_clear is flagged.
    assert mwt.contract_reason('no_candidate', 1.0, 2.0) == 'unexplained_returns'
    assert mwt.contract_reason('no_candidate', None, 2.0) == 'no_candidate'
    assert mwt.contract_reason('no_candidate', 6.1, 2.0) == 'no_candidate'
    assert mwt.contract_reason('contradicted', 1.0, 2.0) == 'contradicted'


def test_merge_keeps_the_larger_miss_and_strip_count():
    """Skeptic #1: min() reset a merged phantom's count every cycle a second
    piece spawned, so the five-in-a-row kill could never fire."""
    trk = mwt.MultiWallTracker()
    feed(trk, [obs(wall(2.0, t0=-1.0, t1=1.0))], 3)
    old = trk.tracks[0]
    old.miss, old.strip_miss = 3, 1
    trk._spawn(mwt.observation_line(obs(wall(2.0, t0=0.5, t1=1.5)), ORIGIN, trk.p), 1.2)
    trk._merge()
    assert trk.tracks == [old] and old.miss == 3 and old.strip_miss == 1
    assert old.t1 == pytest.approx(1.5, abs=0.02)


def test_strip_distance_is_where_the_strip_meets_the_measured_segment():
    half = 0.286
    assert mwt.strip_distance(((3.0, -1.0), (3.0, 1.0)), FRONT_X, half) == pytest.approx(
        3.0 - FRONT_X)
    # Ends 0.1 m short of the strip edge: no intersection, no FRONT.
    assert math.isnan(mwt.strip_distance(((3.0, 0.386), (3.0, 3.0)), FRONT_X, half))
    # Oblique: the NEAR edge of the strip, not the centreline.
    d = mwt.strip_distance(((2.0, -1.0), (4.0, 1.0)), FRONT_X, half)
    assert d == pytest.approx(3.0 - half - FRONT_X, abs=1e-9)
    # Behind the bumper only: not ahead, no FRONT.
    assert math.isnan(mwt.strip_distance(((0.2, -1.0), (0.2, 1.0)), FRONT_X, half))


# ===========================================================================
# THE WALL'S SPAN (door-detection step 2 audit): the span gate reads the
# span of the WALL a LiDAR piece belongs to. Since the extraction merges only
# across gaps shorter than collinear_gap_m, a detected person in front of a
# narrow wall left two pieces too short to be born, and the wall was never
# tracked: no FRONT on any cycle of the approach, the contract CLEAR.
# ===========================================================================

#: A 2.5 m wall square across the path at x = 5, and the same 2.5 m of wall
#: split by an OPEN 0.85 m doorway into two 1.25 m pieces.
NARROW_X = 5.0
NARROW = [((NARROW_X, -1.25), (NARROW_X, 1.25))]
NARROW_DOORWAY = [((NARROW_X, -1.675), (NARROW_X, -0.425)),
                  ((NARROW_X, 0.425), (NARROW_X, 1.675))]
#: A person 0.5 m in front of it, on the path (odom), radius 0.25 m.
PERSON_IN_FRONT = (NARROW_X - 0.5, 0.0)


def approach(segments, cycles, person=None):
    """
    Drive from x = 0 towards x = 5 at 0.1 m a cycle; yield (cycle, car, tracker).

    `person` is an odom (x, y): a 0.25 m radius person in the scan AND
    detected, so their bearing sector is cut out of it as the node cuts a
    /camera/detections_3d position (step's `objects`, base_link).
    """
    trk = mwt.MultiWallTracker()
    trk.laser_xy = LASER[:2]
    for i in range(cycles):
        car = Pose2D(0.1 * i, 0.0, 0.0)
        circles = [(person, 0.25)] if person else []
        objects = [(person[0] - car.x, person[1] - car.y, 0.25)] if person else []
        step(trk, car, 0.1 * i, segments, circles=circles, objects=objects,
             rng=np.random.default_rng(i))
        yield i, car, trk


def on_the_narrow_line(trk, confirmed=True):
    """Return the tracks on the x = 5 line, ordered along it."""
    on = [t for t in trk.tracks if (t.ever_confirmed or not confirmed)
          and abs(t.rho - NARROW_X) < 0.1 and abs(math.sin(t.theta)) < 0.1]
    return sorted(on, key=lambda t: t.t0)


def test_a_narrow_wall_first_seen_behind_a_detected_person_is_born_and_front_within_12_cycles():
    """
    The person's cut sector is ~1 m wide on the wall, wider than collinear_gap_m.

    So the extraction reports two pieces, each shorter than min_span_birth_m
    (asserted below: that is what makes this the audit's case). Measured on
    the WALL they are 2.5 m: born on the first scan, and FRONT as soon as the
    track is confirmed -- one wall across the hole behind the person.
    """
    p = mwt.TrackerParams()
    sectors = detection_sectors([(PERSON_IN_FRONT[0], 0.0, 0.25)], LASER[:2],
                                math.radians(3.0))
    pieces = [ln for ln in mwt.extract_lines(
        raycast(Pose2D(0.0, 0.0, 0.0), NARROW, [(PERSON_IN_FRONT, 0.25)]),
        ANGLE_MIN, INC, 0.06, 30.0, LASER, sectors) if abs(ln.rho - NARROW_X) < 0.1]
    assert len(pieces) == 2 and all(ln.length < p.min_span_birth_m for ln in pieces), \
        'premise: the person splits the wall into two pieces too short to be born'
    front = None
    for i, car, trk in approach(NARROW, 12, person=PERSON_IN_FRONT):
        front = trk.front_track(car, FRONT_X)
        if front is not None:
            break
    assert front is not None, 'no FRONT in 12 cycles: the wall was never born'
    assert front.rho == pytest.approx(NARROW_X, abs=0.03)
    assert abs(front.theta) < math.radians(1.0)
    assert front.t0 < -1.0 and front.t1 > 1.0            # the wall, not a piece of it
    assert on_the_narrow_line(trk) == [front]
    assert trk.front_status['distance_m'] == pytest.approx(
        NARROW_X - car.x - FRONT_X, abs=0.03)


def test_two_wall_pieces_either_side_of_an_open_0_85_m_doorway_stay_two_tracks():
    """
    The same 2.5 m of wall split by an OPEN 0.85 m doorway, and no person.

    The beams see through the doorway, so nothing joins the pieces -- not
    the birth span, the association, the merge or the extent: two tracks,
    never one across the doorway (a door lies BETWEEN two tracks).
    """
    for i, _car, trk in approach(NARROW_DOORWAY, 12):
        walls = on_the_narrow_line(trk, confirmed=False)
        assert not any(t.t0 < -0.4 and t.t1 > 0.4 for t in walls), f'one wall at cycle {i}'
    walls = on_the_narrow_line(trk)
    assert len(walls) == 2
    assert walls[0].t1 < -0.4 and walls[1].t0 > 0.4


def test_two_stubs_either_side_of_an_open_doorway_are_measured_alone_and_start_no_track():
    """
    The 2.5 m wall with an OPEN 0.85 m doorway cut out of it: two 0.825 m stubs.

    Unlike the hole behind a person, the doorway is seen through, so neither
    stub lends the other its span: each is measured alone, is shorter than
    min_span_birth_m, and no track starts on the line -- in particular none
    across the doorway.
    """
    stubs = [((NARROW_X, -1.25), (NARROW_X, -0.425)), ((NARROW_X, 0.425), (NARROW_X, 1.25))]
    assert 0.825 < mwt.TrackerParams().min_span_birth_m       # premise
    for i, _car, trk in approach(stubs, 12):
        assert on_the_narrow_line(trk, confirmed=False) == [], f'a track at cycle {i}'


def test_a_narrow_wall_first_seen_behind_an_undetected_box_is_born_as_one_wall():
    """
    Returns IN FRONT of the line hide it as surely as a cut sector does.

    A 0.8 m box 0.5 m in front of the 2.5 m wall, which nothing detects,
    shadows ~0.9 m of it: the beams never reached that stretch, so it is not
    an opening and the two pieces either side are one wall.
    """
    box = [((NARROW_X - 0.5, -0.4), (NARROW_X - 0.5, 0.4))]
    for _i, _car, trk in approach(NARROW + box, 12):
        pass
    walls = on_the_narrow_line(trk)
    assert len(walls) == 1
    assert walls[0].t0 < -1.0 and walls[0].t1 > 1.0
