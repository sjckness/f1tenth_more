"""scene_mask.py: one class per pixel, in priority order, on the real mount TF.

The synthetic frames here are built through the SAME transform chain the live
stack publishes (base_link -> zed2_camera_link at z 0.205 with the measured
-2.083 deg nose-up pitch, then the ZED 2i macro's optical frame), so "a flat
floor comes out FLOOR" is a statement about the deployed extrinsic, not about
an idealised camera. Change the mount pitch and these tests move with it --
that coupling is the point (CLAUDE.md, "when a test encodes a config default").
"""

import math
import time

import numpy as np
import pytest

from f1tenth_perception import scene_mask as sm

#: base_link -> zed2_camera_link, camera.launch.py (2026-10-15).
MOUNT_XYZ = (0.2399, 0.0, 0.205)
MOUNT_PITCH_RAD = -0.03635
#: zed2_camera_link -> zed2_left_camera_frame, via zed2_camera_center.
LEFT_OFFSET = (-0.01, 0.06, 0.015)
#: 640x360 ZED depth, fy 262.4 (perception timing notes, 2026-10-14).
FX = FY = 262.4
CX, CY = 320.0, 179.3
WIDTH, HEIGHT = 640, 360

#: optical (x right, y down, z forward) -> camera body (x fwd, y left, z up).
R_OPTICAL_TO_BODY = np.array([[0.0, 0.0, 1.0], [-1.0, 0.0, 0.0], [0.0, -1.0, 0.0]])


def mount_tf(pitch_rad=MOUNT_PITCH_RAD):
    """(rotation, translation) taking left-optical-frame points into base_link."""
    c, s = math.cos(pitch_rad), math.sin(pitch_rad)
    r_pitch = np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])
    rotation = r_pitch @ R_OPTICAL_TO_BODY
    translation = np.asarray(MOUNT_XYZ) + r_pitch @ np.asarray(LEFT_OFFSET)
    return rotation, translation


def rays(width=WIDTH, height=HEIGHT):
    """(H, W, 3) unit-z optical rays of every pixel."""
    u = (np.arange(width) - CX) / FX
    v = (np.arange(height) - CY) / FY
    out = np.empty((height, width, 3), dtype=float)
    out[..., 0] = u[None, :]
    out[..., 1] = v[:, None]
    out[..., 2] = 1.0
    return out


def plane_depth(normal_base, offset, rotation, translation, width=WIDTH, height=HEIGHT):
    """Z-depth of the plane n . p = offset (base_link), NaN where a ray misses it.

    Exact inverse of the back-projection: p_base = R (ray * D) + t, so
    n . p_base = offset gives D directly.
    """
    d_base = rays(width, height) @ rotation.T
    n = np.asarray(normal_base, dtype=float)
    denom = d_base @ n
    num = offset - float(np.asarray(translation) @ n)
    with np.errstate(divide='ignore', invalid='ignore'):
        depth = np.where(np.abs(denom) > 1e-9, num / denom, np.nan)
    return np.where(depth > 0, depth, np.nan).astype(np.float32)


def floor_depth(z=0.0, **kw):
    """A flat horizontal surface at height z in base_link."""
    rotation, translation = kw.pop('tf', mount_tf())
    return plane_depth((0.0, 0.0, 1.0), z, rotation, translation, **kw)


def wall_depth(x, **kw):
    """A flat vertical wall at base_link x."""
    rotation, translation = kw.pop('tf', mount_tf())
    return plane_depth((1.0, 0.0, 0.0), x, rotation, translation, **kw)


def classify(depth, **kw):
    rotation, translation = kw.pop('tf', mount_tf())
    return sm.classify(depth, FX, FY, CX, CY, rotation, translation, **kw)


# ---------------------------------------------------------------------------
# The depth-derived classes
# ---------------------------------------------------------------------------

def test_a_flat_floor_is_floor_at_the_measured_mount_pitch():
    """The whole lower half of a floor-only frame is FLOOR, none of it WALL."""
    depth = floor_depth()
    labels, z = classify(depth)
    # Only where the floor is actually within the sensor's usable range: the
    # rows approaching the horizon run past 20 m and are INVALID, correctly.
    seen = labels[sm.valid_depth(depth)]
    assert seen.size > 10000
    assert np.all(seen == sm.CLASS_FLOOR)
    assert np.nanmax(np.abs(z[labels == sm.CLASS_FLOOR])) < 0.05


def test_a_floor_misread_by_a_wrong_pitch_stops_being_floor():
    """Why the pitch matters: the same frame read with pitch 0 is not all FLOOR.

    The floor is 1.4-3 m ahead, where 2.08 deg of pitch is 5-11 cm of height --
    more than floor_tol, so the far rows leave the floor band entirely.
    """
    depth = floor_depth()
    labels, _ = classify(depth, tf=mount_tf(pitch_rad=0.0))
    seen = labels[np.isfinite(depth)]
    assert np.count_nonzero(seen == sm.CLASS_FLOOR) < 0.9 * seen.size


def test_points_below_the_floor_are_reflection_not_floor_or_wall():
    """The glossy tiles mirror the wall: those points sit well below z = 0."""
    depth = floor_depth(z=-0.30)
    labels, _ = classify(depth)
    seen = labels[sm.valid_depth(depth)]
    assert np.all(seen == sm.CLASS_REFLECTION)


def test_a_wall_ahead_is_wall():
    depth = wall_depth(2.5)
    labels, _ = classify(depth)
    seen = labels[sm.valid_depth(depth)]
    # The wall spans the floor line to well above the camera; the band around
    # z = 0 is FLOOR by definition and the top of a 2.5 m wall is CEILING.
    assert np.count_nonzero(seen == sm.CLASS_WALL) > 0.5 * seen.size
    assert np.count_nonzero(seen == sm.CLASS_FLOOR) > 0


def test_high_points_are_ceiling():
    depth = floor_depth(z=2.6)
    labels, _ = classify(depth)
    seen = labels[sm.valid_depth(depth)]
    assert np.all(seen == sm.CLASS_CEILING)


def test_no_depth_is_invalid():
    depth = np.full((20, 30), np.nan, dtype=np.float32)
    labels, _ = classify(depth)
    assert np.all(labels == sm.CLASS_INVALID)


def test_minus_inf_is_invalid_but_counted_as_too_close():
    """-Inf means "nearer than the camera can measure", not "no data"."""
    depth = np.full((10, 10), -np.inf, dtype=np.float32)
    labels, _ = classify(depth)
    assert np.all(labels == sm.CLASS_INVALID)
    assert sm.class_stats(labels, depth=depth)['too_close_frac'] == 1.0


# ---------------------------------------------------------------------------
# Priority
# ---------------------------------------------------------------------------

def test_an_object_standing_on_the_floor_stays_object():
    """The priority rule that matters: OBJECT beats FLOOR on the same pixels."""
    depth = floor_depth()
    obj = np.zeros(depth.shape, dtype=bool)
    obj[200:260, 300:340] = True
    plain, _ = classify(depth)
    assert np.all(plain[obj] == sm.CLASS_FLOOR)   # in range: ~1.5 m ahead
    labels, _ = classify(depth, object_mask=obj)
    assert np.all(labels[obj] == sm.CLASS_OBJECT)
    assert np.all(labels[~obj] == plain[~obj])


def test_the_self_mask_is_applied_first_and_beats_an_object():
    depth = wall_depth(2.0)
    region = (slice(300, 360), slice(560, 640))
    selfm = np.zeros(depth.shape, dtype=bool)
    selfm[region] = True
    obj = np.ones(depth.shape, dtype=bool)
    labels, _ = classify(depth, self_mask=selfm, object_mask=obj)
    assert np.all(labels[region] == sm.CLASS_SELF)
    assert np.all(labels[:200] == sm.CLASS_OBJECT)


def test_the_self_mask_wins_even_where_there_is_no_depth():
    depth = np.full((20, 30), np.nan, dtype=np.float32)
    selfm = np.zeros(depth.shape, dtype=bool)
    selfm[10:, 20:] = True
    labels, _ = classify(depth, self_mask=selfm)
    assert np.all(labels[10:, 20:] == sm.CLASS_SELF)
    assert np.all(labels[:10] == sm.CLASS_INVALID)


def test_every_pixel_gets_exactly_one_known_class():
    depth = wall_depth(3.0)
    depth[:40] = np.nan
    obj = np.zeros(depth.shape, dtype=bool)
    obj[100:150, 100:150] = True
    selfm = np.zeros(depth.shape, dtype=bool)
    selfm[340:, 600:] = True
    labels, _ = classify(depth, self_mask=selfm, object_mask=obj)
    assert labels.shape == depth.shape
    assert set(np.unique(labels)) <= set(sm.CLASS_NAMES)
    counts = np.bincount(labels.ravel(), minlength=sm.CLASS_WALL + 1)
    assert int(counts.sum()) == labels.size


# ---------------------------------------------------------------------------
# Self-mask polygons
# ---------------------------------------------------------------------------

def test_polygon_mask_is_fractional_and_resolution_independent():
    poly = [sm.rect_polygon(0.5, 0.5, 1.0, 1.0)]
    small = sm.polygon_mask((100, 200), poly)
    big = sm.polygon_mask((400, 800), poly)
    assert small[75, 150] and not small[25, 50]
    assert big[300, 600] and not big[100, 200]
    assert abs(small.mean() - big.mean()) < 0.02


def test_polygon_mask_of_nothing_is_empty():
    assert not sm.polygon_mask((10, 10), []).any()
    assert not sm.polygon_mask((10, 10), [[(0.0, 0.0), (1.0, 0.0)]]).any()


# ---------------------------------------------------------------------------
# Points and stats
# ---------------------------------------------------------------------------

def test_wall_points_land_on_the_wall_plane():
    depth = wall_depth(2.5)
    labels, _ = classify(depth)
    pts, rows, cols = sm.wall_points(depth, labels, FX, FY, CX, CY, *mount_tf())
    assert pts.shape[0] == np.count_nonzero(labels == sm.CLASS_WALL)
    assert rows.shape == cols.shape == (pts.shape[0],)
    assert np.max(np.abs(pts[:, 0] - 2.5)) < 1e-3


def test_in_strip_takes_only_what_is_ahead_of_the_bumper_and_narrow():
    pts = np.array([[1.0, 0.0, 0.5],     # ahead, centred
                    [1.0, 0.30, 0.5],    # ahead, beside the strip
                    [0.2, 0.0, 0.5],     # behind the bumper
                    [2.0, -0.18, 0.5]])  # ahead, just inside
    assert list(sm.in_strip(pts, 0.405, 0.186)) == [True, False, False, True]


def test_in_strip_of_nothing():
    assert sm.in_strip(np.empty((0, 3)), 0.405, 0.186).shape == (0,)


def test_stats_are_consistent_with_the_classes():
    depth = wall_depth(2.5)
    depth[:30] = np.nan
    obj = np.zeros(depth.shape, dtype=bool)
    obj[200:230, 300:330] = True
    labels, _ = classify(depth, object_mask=obj)
    pts, _, _ = sm.wall_points(depth, labels, FX, FY, CX, CY, *mount_tf())
    stats = sm.class_stats(labels, depth=depth, points=pts,
                           front_x=0.405, strip_half_width_m=0.186)
    assert stats['n_px'] == labels.size
    assert sum(stats['count'].values()) == labels.size
    assert stats['count']['object'] == 30 * 30
    assert stats['count']['invalid'] >= 30 * depth.shape[1]
    assert stats['count']['wall'] == pts.shape[0] == stats['wall_points']
    assert sum(stats['frac'].values()) == pytest.approx(1.0, abs=1e-4)
    assert 0 < stats['wall_points_in_strip'] <= stats['wall_points']


def test_stats_without_points_omit_the_point_counts():
    labels, _ = classify(wall_depth(2.0))
    stats = sm.class_stats(labels)
    assert 'wall_points' not in stats and 'too_close_frac' not in stats


# ---------------------------------------------------------------------------
# Overlay and cost
# ---------------------------------------------------------------------------

def test_the_overlay_colours_every_class_and_draws_the_scan():
    labels, _ = classify(wall_depth(2.0))
    img = np.zeros((HEIGHT, WIDTH, 3), dtype=np.uint8)
    out = sm.overlay(img, labels, scan_px=[(100.0, 50.0), (np.nan, 5.0)])
    assert out.shape == img.shape
    assert out.any()
    assert tuple(out[50, 100]) == (255, 255, 255)


def test_classify_is_fast_enough_for_a_full_frame(capsys):
    """Budget: the tracker already ran at 105 ms median live, so the per-frame
    masking has to stay a small fraction of one 5 Hz tick."""
    depth = wall_depth(3.0)
    selfm = sm.polygon_mask(depth.shape, [sm.rect_polygon(0.75, 0.55, 1.0, 1.0)])
    obj = np.zeros(depth.shape, dtype=bool)
    obj[150:250, 250:400] = True
    best = math.inf
    for _ in range(5):
        t0 = time.perf_counter()
        labels, _ = classify(depth, self_mask=selfm, object_mask=obj)
        pts, _, _ = sm.wall_points(depth, labels, FX, FY, CX, CY, *mount_tf())
        sm.class_stats(labels, depth=depth, points=pts, front_x=0.405,
                       strip_half_width_m=0.186)
        best = min(best, (time.perf_counter() - t0) * 1e3)
    with capsys.disabled():
        print(f'\n  scene_mask classify+points+stats, {WIDTH}x{HEIGHT}: {best:.1f} ms')
    assert best < 60.0


def test_project_to_image_is_the_inverse_of_the_back_projection():
    """A wall pixel, back-projected and projected again, lands where it started."""
    depth = wall_depth(2.5)
    labels, _ = classify(depth)
    pts, rows, cols = sm.wall_points(depth, labels, FX, FY, CX, CY, *mount_tf())
    uv = sm.project_to_image(pts, FX, FY, CX, CY, *mount_tf())
    assert np.max(np.abs(uv[:, 0] - cols)) < 1e-6
    assert np.max(np.abs(uv[:, 1] - rows)) < 1e-6


def test_project_to_image_rejects_points_behind_the_lens():
    uv = sm.project_to_image([[-1.0, 0.0, 0.2], [2.0, 0.0, 0.2]], FX, FY, CX, CY, *mount_tf())
    assert np.all(np.isnan(uv[0]))
    assert np.all(np.isfinite(uv[1]))


def test_project_to_image_of_nothing():
    assert sm.project_to_image(np.empty((0, 3)), FX, FY, CX, CY, *mount_tf()).shape == (0, 2)
