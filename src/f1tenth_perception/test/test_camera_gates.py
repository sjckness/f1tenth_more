"""The camera observation gates and the camera half of the roles (Q2 / P3).

What the camera may say, and how much of it counts:

  * a segment lying along the viewing ray is not observable, however well it
    was fitted -- the radial gate;
  * a stereo-only line nothing cross-checks must look like a wall on its own
    -- tight, tall, long;
  * the same stale profile re-presented over three tracker cycles is ONE
    observation, so only a fresh cycle counts towards confirmation;
  * a camera-only track is a side wall only inside a tighter zone than the
    LiDAR's, and at a corridor-like offset;
  * a wall both sensors saw is one track, labelled 'both'.
"""

import math

import pytest

from corridor_perception.geometry import Pose2D
from f1tenth_perception import multi_wall_tracker as mwt
from f1tenth_perception import wall_fusion
from f1tenth_perception.scene_labels import WallLine

ORIGIN = Pose2D(0.0, 0.0, 0.0)
FRONT_X = 0.405
CAM = (0.2294, 0.06)


def line(d, nx=1.0, ny=0.0, t0=-1.0, t1=1.0, inliers=120, rms=0.01):
    return WallLine(nx, ny, d, t0, t1, inliers, rms, 'lidar')


def line_through(p0, p1, inliers=120, rms=0.01):
    """A WallLine with exactly these two base_link endpoints."""
    (x0, y0), (x1, y1) = p0, p1
    dx, dy = x1 - x0, y1 - y0
    n = math.hypot(dx, dy)
    tx, ty = dx / n, dy / n
    nx, ny = -ty, tx                      # tangent is (-ny, nx), so this inverts
    d = nx * x0 + ny * y0
    px, py = nx * d, ny * d
    t0 = (x0 - px) * tx + (y0 - py) * ty
    return WallLine(nx, ny, d, t0, t0 + n, inliers, rms, 'lidar')


def obs(ln, source=wall_fusion.SOURCE_STEREO, confidence=0.35,
        z=(0.0, 2.5), measured=True):
    return wall_fusion.Observation(ln, z[0], z[1], measured, source, confidence)


def tracker(**kw):
    t = mwt.MultiWallTracker(mwt.TrackerParams(**kw))
    t.camera_xy = CAM
    return t


def feed(t, observations, n, car=ORIGIN, t0=1.0, dt=0.05):
    for i in range(n):
        t.step(list(observations), t0 + i * dt, car)
    return t


# --- the radial gate -------------------------------------------------------

def test_a_segment_along_the_viewing_ray_is_rejected():
    """A 3 m line running straight away from the camera: every point of it is
    on the same bearing, so its range is a guess."""
    # 15 deg off the ray from the camera to its own midpoint, and far enough
    # from base_link that the near-field gate is not what catches it. (An
    # EXACTLY radial line passes through the camera, 0.23 m from base_link,
    # so stereo_min_rho_m already refuses that case from the other side.)
    radial = line_through((1.894, 0.486), (4.106, 2.514))
    assert abs(radial.d) > mwt.TrackerParams().stereo_min_rho_m, 'premise'
    t = tracker()
    feed(t, [obs(radial)], 6)
    assert not t.tracks, 'a near-radial segment must never become a track'
    assert any(r[0] == 'camera_radial' for r in t.rejections)


def test_an_exactly_radial_line_is_refused_by_the_near_field_gate():
    """The complementary case, stated so the pair is not mistaken for a hole:
    a line truly along the ray passes through the camera, which is 0.23 m from
    base_link, so stereo_min_rho_m catches it first."""
    t = tracker()
    feed(t, [obs(line_through((1.0, 0.06), (4.0, 0.06)))], 6)
    assert not t.tracks
    assert any(r[0] == 'stereo_near' for r in t.rejections)


def test_a_real_oblique_wall_is_kept():
    """45 deg to the ray is not along it: an oblique wall the camera sees
    side-on stays."""
    wall = line(2.0, nx=math.cos(0.0), ny=math.sin(0.0), t0=-1.5, t1=1.5)
    t = tracker()
    feed(t, [obs(wall)], 6)
    assert t.tracks, 'a wall across the view is not radial'


def test_a_sliver_subtending_almost_no_bearing_is_rejected():
    far = line(20.0, t0=-0.35, t1=0.35)      # 0.7 m wide at 20 m: ~2.0 deg
    t = tracker(stereo_min_length_m=0.0, min_span_update_m=0.3, min_span_birth_m=0.3)
    feed(t, [obs(far)], 6)
    assert not t.tracks
    assert any(r[0] == 'camera_radial' for r in t.rejections)


def test_the_gate_never_touches_a_lidar_observation():
    radial = line_through((1.894, 0.486), (4.106, 2.514))
    t = tracker()
    feed(t, [obs(radial, wall_fusion.SOURCE_LIDAR, confidence=0.8)], 6)
    assert t.tracks, 'the LiDAR measures range per beam; the gate is not its problem'


# --- the stereo quality gates ---------------------------------------------

def test_a_badly_fitted_stereo_line_is_dropped():
    t = tracker()
    feed(t, [obs(line(2.0, rms=0.2))], 4)
    assert not t.tracks
    assert any(r[0] == 'stereo_rms' for r in t.rejections)


def test_a_flat_stereo_line_is_dropped_only_when_the_height_was_measured():
    flat = line(2.0)
    dropped = tracker()
    feed(dropped, [obs(flat, z=(0.0, 0.2), measured=True)], 4)
    assert not dropped.tracks
    assert any(r[0] == 'stereo_flat' for r in dropped.rejections)

    unmeasured = tracker()
    feed(unmeasured, [obs(flat, z=(0.0, 0.2), measured=False)], 4)
    assert unmeasured.tracks, 'a wall the profile does not cover has no measured height'


def test_a_short_stereo_line_is_dropped():
    t = tracker()
    feed(t, [obs(line(2.0, t0=-0.3, t1=0.3))], 4)
    assert not t.tracks
    assert any(r[0] in ('stereo_short', 'camera_radial') for r in t.rejections)


# --- confirmation on fresh cycles only ------------------------------------

def wide(d=2.0):
    return line(d, t0=-1.0, t1=1.0)


def test_a_camera_only_track_confirms_on_fresh_cycles():
    p = mwt.TrackerParams()
    t = tracker()
    feed(t, [obs(wide())], p.stereo_confirm_hits + 1)
    assert t.tracks[0].ever_confirmed


def test_a_stale_camera_cycle_does_not_count_towards_confirmation():
    """The same profile, re-presented while the camera is late, is one
    observation. The track still exists and still tracks; it just does not
    graduate on repetition."""
    p = mwt.TrackerParams()
    t = tracker()
    t.camera_fresh = False
    feed(t, [obs(wide())], p.stereo_confirm_hits + 4)
    assert t.tracks, 'the track is still tracked'
    assert not t.tracks[0].ever_confirmed


def test_turning_the_rule_off_restores_the_old_behaviour():
    p = mwt.TrackerParams()
    t = tracker(camera_confirm_fresh_only=False)
    t.camera_fresh = False
    feed(t, [obs(wide())], p.stereo_confirm_hits + 1)
    assert t.tracks[0].ever_confirmed


def test_a_lidar_backed_track_is_unaffected_by_a_stale_camera():
    p = mwt.TrackerParams()
    t = tracker()
    t.camera_fresh = False
    feed(t, [obs(wide(), wall_fusion.SOURCE_LIDAR, confidence=0.8)], p.confirm_hits + 1)
    assert t.tracks[0].ever_confirmed


# --- the camera side zone --------------------------------------------------

def side(offset, source=wall_fusion.SOURCE_STEREO):
    """A wall running along x at y = offset."""
    return obs(WallLine(0.0, 1.0 if offset > 0 else -1.0, abs(offset),
                        0.0, 3.0, 120, 0.01, 'lidar'), source,
               confidence=0.35 if source == wall_fusion.SOURCE_STEREO else 0.8)


def test_a_camera_wall_at_a_corridor_offset_is_a_side_wall():
    t = tracker()
    feed(t, [side(1.6)], 8)
    assert t.tracks
    t.update_roles(ORIGIN)
    assert t.tracks[0].role in ('left', 'right')


def test_a_camera_wall_too_far_out_is_not_a_side_wall():
    t = tracker(camera_side_offset_max_m=3.0)
    feed(t, [side(4.5)], 8)
    assert t.tracks
    t.update_roles(ORIGIN)
    assert t.tracks[0].role == 'unknown', 'the far side of a hall is not a reference wall'


def test_the_lidar_keeps_its_own_wider_zone():
    t = tracker()
    feed(t, [side(4.5, wall_fusion.SOURCE_LIDAR)], 8)
    assert t.tracks
    t.update_roles(ORIGIN)
    # The LiDAR zone is judged on x reach and normal only, not on offset.
    assert t.tracks[0].source == wall_fusion.SOURCE_LIDAR


# --- LiDAR + camera -> one track, labelled 'both' -------------------------

def test_a_wall_both_sensors_saw_is_one_track_called_both():
    t = tracker()
    feed(t, [obs(wide(), wall_fusion.SOURCE_LIDAR, confidence=0.8)], 3)
    assert len(t.tracks) == 1
    # "both" needs the rectangle to have had something to say about this
    # track on its latest run; a column agreement alone is LiDAR evidence.
    t.tracks[0].rect_informative = True
    feed(t, [obs(wide(), wall_fusion.SOURCE_BOTH, confidence=0.8)], 3, t0=2.0)
    assert len(t.tracks) == 1, 'the same wall seen by both is not two walls'
    trk = t.tracks[0]
    assert trk.has_lidar and trk.has_both
    assert trk.source == wall_fusion.SOURCE_BOTH
    assert wall_fusion.SOURCE_NAMES[trk.source] == 'both'


def test_both_evidence_without_an_informative_rectangle_stays_lidar():
    t = tracker()
    feed(t, [obs(wide(), wall_fusion.SOURCE_LIDAR, confidence=0.8)], 3)
    t.tracks[0].rect_informative = False
    feed(t, [obs(wide(), wall_fusion.SOURCE_BOTH, confidence=0.8)], 3, t0=2.0)
    assert t.tracks[0].source == wall_fusion.SOURCE_LIDAR


def test_a_lidar_hit_outranks_a_later_stereo_only_one():
    t = tracker()
    feed(t, [obs(wide(), wall_fusion.SOURCE_LIDAR, confidence=0.8)], 3)
    feed(t, [obs(wide())], 3, t0=2.0)
    assert t.tracks[0].source != wall_fusion.SOURCE_STEREO
