"""inferred_floor.py: the floor the LiDAR can see where the stereo matched nothing.

The camera here is the one test_scene_mask.py builds -- the deployed mount
chain, measured pitch included -- and the LiDAR is the deployed one: 1081
beams over 270 deg from 0.305 m ahead of base_link (description.launch.py).
So "this pixel's floor point is 1.8 m from the laser on beam 540" is a
statement about the real extrinsic pair, not about an idealised rig. Change
either mount and these numbers move, which is the point (CLAUDE.md, "when a
test encodes a config default").

The geometry cases are solved independently of the code under test: a pixel's
floor point is computed from the pinhole model and the TF by hand in
`expected_floor_point`, and the LUT is checked against that.
"""

import math

import numpy as np
import pytest

from f1tenth_perception import inferred_floor as inf
from f1tenth_perception import scene_mask as sm
# The deployed camera chain, built once in test_scene_mask.py: same convention
# test_wall_fusion.py already follows for the same helpers.
from test_scene_mask import (  # noqa: E402
    CX, CY, FX, FY, HEIGHT, WIDTH, floor_depth, mount_tf, wall_depth)

#: base_link -> laser, description.launch.py's static_baselink_to_laser
#: (2026-10-14: x and z MEASURED, y and the rotation are not).
LASER_POSE = (0.305, 0.0, 0.0)
LASER_Z_M = 0.190
#: The deployed Hokuyo layout, read off the run archive's own /scan
#: (corridor10m_20261015T171753): 1081 beams, not the 1080 of the datasheet.
ANGLE_MIN = -2.356194496154785
ANGLE_INCREMENT = 0.004363323096185923
N_BEAMS = 1081
RANGE_MIN = 0.02
RANGE_MAX = 30.0
#: The driver's finite no-return code; above RANGE_MAX, so it counts as FAR.
NO_RETURN_M = 65.533

R_MAX = 3.0
MARGIN = 0.15
FAR_VALUE = R_MAX + MARGIN + 1.0


def build(r_max=R_MAX, self_mask=None, pitch_rad=None, laser_pose=LASER_POSE):
    rotation, translation = mount_tf() if pitch_rad is None else mount_tf(pitch_rad)
    return inf.build_lut((HEIGHT, WIDTH), FX, FY, CX, CY, rotation, translation,
                         laser_pose, ANGLE_MIN, ANGLE_INCREMENT, N_BEAMS,
                         r_max=r_max, self_mask=self_mask)


def expected_floor_point(u, v, pitch_rad=None):
    """(x, y) in base_link where pixel (u, v)'s ray meets the floor z = 0.

    Done from first principles here, not through the module under test: the
    optical ray, the mount rotation, and the one scalar that puts the ray on
    the plane.
    """
    rotation, translation = mount_tf() if pitch_rad is None else mount_tf(pitch_rad)
    ray = np.array([(u - CX) / FX, (v - CY) / FY, 1.0])
    d = rotation @ ray
    s = -translation[2] / d[2]
    p = translation + s * d
    return float(p[0]), float(p[1])


def ranges_at(value, n=N_BEAMS):
    return np.full(n, float(value))


def r_eff_of(ranges, window_k=3, sigma_beams=1.5):
    return inf.effective_ranges(ranges, RANGE_MIN, RANGE_MAX, FAR_VALUE,
                                window_k=window_k, sigma_beams=sigma_beams)


def infer(labels, lut, ranges, margin=MARGIN, **kw):
    return inf.apply(np.array(labels, copy=True), lut, r_eff_of(ranges, **kw),
                     margin=margin)


# ---------------------------------------------------------------------------
# The LUT: ray -> floor -> beam, on the deployed extrinsic pair
# ---------------------------------------------------------------------------

def test_the_lut_puts_each_pixels_floor_point_where_the_pinhole_model_does():
    """r_floor and the beam index agree with the geometry solved by hand.

    Checked on the centre column and two off-centre ones, at rows spread over
    the inferable band, so an error in the pitch, the mount translation or the
    laser offset shows up as a disagreement rather than cancelling.
    """
    lut = build()
    checked = 0
    for u in (int(CX) - 180, int(CX), int(CX) + 200):
        for v in range(220, HEIGHT, 20):
            if not lut.valid[v, u]:
                continue
            px, py = expected_floor_point(u, v)
            # Seen from the LASER, which is 0.305 m ahead of base_link.
            ex, ey = px - LASER_POSE[0], py - LASER_POSE[1]
            assert lut.r_floor[v, u] == pytest.approx(math.hypot(ex, ey), abs=1e-4)
            bearing = math.atan2(ey, ex)
            assert lut.beam[v, u] == round((bearing - ANGLE_MIN) / ANGLE_INCREMENT)
            checked += 1
    assert checked > 20


def test_the_bearing_is_taken_from_the_laser_not_from_base_link():
    """The 0.305 m offset is worth tens of beams, so it cannot be skipped.

    A floor point 1.5 m ahead and 0.5 m to the left is 18.4 deg off the
    heading from base_link and 26.6 deg from the laser: 42 beams apart. This
    is the error the LUT would make if the bearing were taken from base_link,
    and it is why `build_lut` takes a laser pose at all.
    """
    lut = build()
    from_base_link = build(laser_pose=(0.0, 0.0, 0.0))
    both = lut.valid & from_base_link.valid
    assert np.count_nonzero(both) > 1000
    shift = np.abs(lut.beam[both].astype(int) - from_base_link.beam[both].astype(int))
    assert shift.max() > 40
    assert np.median(shift) > 5


def test_only_the_band_below_the_horizon_and_inside_r_max_is_inferable():
    lut = build()
    assert 0 < lut.n_valid < HEIGHT * WIDTH
    rows = np.nonzero(lut.valid.any(axis=1))[0]
    # The floor is below the horizon, which on a nose-up mount is below the
    # principal row; nothing in the upper half of the frame may be inferred.
    assert rows.min() > CY
    assert rows.max() == HEIGHT - 1
    r = lut.r_floor[lut.valid]
    assert np.all(np.isfinite(r))
    assert r.max() <= R_MAX
    assert np.all(np.isnan(lut.r_floor[~lut.valid]))


def test_r_max_is_what_bounds_the_inferable_band():
    near, far = build(r_max=1.0), build(r_max=3.0)
    assert near.n_valid < far.n_valid
    assert np.all(far.valid[near.valid])
    assert near.r_floor[near.valid].max() <= 1.0


def test_a_self_masked_pixel_is_never_inferable():
    """Part C's mask, whichever it ends up being, excludes itself from the LUT."""
    self_mask = np.zeros((HEIGHT, WIDTH), dtype=bool)
    self_mask[HEIGHT // 2:, WIDTH // 2:] = True
    lut = build(self_mask=self_mask)
    assert lut.n_valid > 0
    assert not lut.valid[self_mask].any()
    assert lut.n_valid < build().n_valid


def test_a_bearing_outside_the_lidars_field_of_view_is_not_inferable():
    """A 20-beam scan covers 5 deg: almost nothing the camera sees is in it."""
    rotation, translation = mount_tf()
    narrow = inf.build_lut((HEIGHT, WIDTH), FX, FY, CX, CY, rotation, translation,
                           LASER_POSE, -0.0436, ANGLE_INCREMENT, 20, r_max=R_MAX)
    full = build()
    # 5 deg of 112: a few per cent of the band, all of it near the centre
    # column, and every index inside the shortened scan.
    assert narrow.n_valid < 0.05 * full.n_valid
    assert np.all(narrow.beam[narrow.valid] < 20)
    cols = np.nonzero(narrow.valid.any(axis=0))[0]
    assert cols.max() - cols.min() < 0.25 * WIDTH


def test_the_lut_key_changes_with_every_input_it_depends_on():
    """What `matches` has to notice, or a stale LUT survives a reconfiguration."""
    lut = build()
    assert lut.matches(lut.key)
    assert not lut.matches(build(r_max=2.0).key)
    assert not lut.matches(build(pitch_rad=0.0).key)
    assert not lut.matches(build(laser_pose=(0.12, 0.0, 0.0)).key)
    rotation, translation = mount_tf()
    assert not lut.matches(inf.build_lut(
        (HEIGHT // 2, WIDTH // 2), FX, FY, CX, CY, rotation, translation, LASER_POSE,
        ANGLE_MIN, ANGLE_INCREMENT, N_BEAMS, r_max=R_MAX).key)
    assert not lut.matches(inf.build_lut(
        (HEIGHT, WIDTH), FX * 2, FY, CX, CY, rotation, translation, LASER_POSE,
        ANGLE_MIN, ANGLE_INCREMENT, N_BEAMS, r_max=R_MAX).key)


# ---------------------------------------------------------------------------
# The per-cycle decision
# ---------------------------------------------------------------------------

def test_an_empty_corridor_turns_the_whole_inferable_band_into_floor_inferred():
    lut = build()
    labels = np.full((HEIGHT, WIDTH), sm.CLASS_INVALID, dtype=np.uint8)
    out, n = infer(labels, lut, ranges_at(20.0))
    assert n == lut.n_valid
    assert np.all(out[lut.valid] == sm.CLASS_FLOOR_INFERRED)
    assert np.all(out[~lut.valid] == sm.CLASS_INVALID)


def test_a_no_return_counts_as_far_and_so_does_the_drivers_65_m_code():
    """Nothing was hit, so the floor under that bearing is free."""
    lut = build()
    labels = np.full((HEIGHT, WIDTH), sm.CLASS_INVALID, dtype=np.uint8)
    for value in (np.inf, np.nan, NO_RETURN_M):
        _out, n = infer(labels, lut, ranges_at(value))
        assert n == lut.n_valid, value


def test_an_obstacle_closer_than_the_floor_point_leaves_the_pixel_alone():
    """The whole point: a beam that stopped is not evidence of floor past it."""
    lut = build()
    labels = np.full((HEIGHT, WIDTH), sm.CLASS_INVALID, dtype=np.uint8)
    out, n = infer(labels, lut, ranges_at(1.0))
    inferred = out == sm.CLASS_FLOOR_INFERRED
    assert n > 0
    # Exactly the pixels whose floor point is inside 1.0 m by more than the
    # margin; everything beyond the obstacle stays INVALID.
    assert np.all(lut.r_floor[inferred] < 1.0 - MARGIN + 1e-6)
    assert np.all(out[lut.valid & (lut.r_floor > 1.0)] == sm.CLASS_INVALID)


def test_the_margin_is_what_keeps_the_floor_off_the_foot_of_a_wall():
    lut = build()
    labels = np.full((HEIGHT, WIDTH), sm.CLASS_INVALID, dtype=np.uint8)
    _out, tight = infer(labels, lut, ranges_at(2.0), margin=0.0)
    _out, loose = infer(labels, lut, ranges_at(2.0), margin=0.5)
    assert loose < tight
    band = np.count_nonzero(lut.valid & (lut.r_floor > 1.5) & (lut.r_floor < 2.0))
    assert tight - loose == band


def test_a_window_straddling_an_edge_does_not_leak_floor_past_the_near_side():
    """Why r_eff takes the central +-1 minimum as well as the smoothed mean.

    Half the scan is a wall at 0.8 m and half is empty. The smoothing alone
    would blend the two across the boundary and infer floor several beams into
    the near half; the central minimum pins each beam to what its own
    neighbours saw.
    """
    lut = build()
    ranges = ranges_at(20.0)
    edge = N_BEAMS // 2
    ranges[:edge] = 0.8
    r_eff = r_eff_of(ranges)
    # No beam on the near side is allowed to report past the wall.
    assert r_eff[:edge - 1].max() <= 0.8 + 1e-9
    # Without the central minimum it would: the smoothed mean alone leaks.
    weights = inf.gaussian_beam_weights(3, 1.5)
    smoothed = np.convolve(np.pad(ranges, 3, mode='edge'), weights, mode='valid')
    assert smoothed[edge - 1] > 0.8 + MARGIN
    labels = np.full((HEIGHT, WIDTH), sm.CLASS_INVALID, dtype=np.uint8)
    out, _n = inf.apply(labels, lut, r_eff, margin=MARGIN)
    near_side = lut.valid & (lut.beam < edge) & (lut.r_floor > 0.8)
    assert np.count_nonzero(near_side) > 100
    assert np.all(out[near_side] == sm.CLASS_INVALID)


def test_one_spurious_short_return_does_not_punch_a_hole_in_the_floor():
    """What the smoothing is for, in the only direction it is allowed to act.

    A single short beam among 20 m neighbours still blocks its own bearing
    (the central minimum sees it) but does not block the beams two away.
    """
    ranges = ranges_at(20.0)
    ranges[540] = 0.3
    r_eff = r_eff_of(ranges)
    assert r_eff[540] == pytest.approx(0.3)
    assert r_eff[539] == pytest.approx(0.3)
    assert r_eff[541] == pytest.approx(0.3)
    assert r_eff[537] > R_MAX + MARGIN
    assert r_eff[543] > R_MAX + MARGIN


def test_a_return_below_range_min_refuses_the_inference():
    """Not a measurement and not "far": it must not become free floor."""
    lut = build()
    labels = np.full((HEIGHT, WIDTH), sm.CLASS_INVALID, dtype=np.uint8)
    _out, n = infer(labels, lut, ranges_at(RANGE_MIN * 0.5))
    assert n == 0


def test_only_invalid_and_reflection_pixels_are_candidates():
    """A class decided on evidence is never overwritten by the inference."""
    lut = build()
    for cls in (sm.CLASS_FLOOR, sm.CLASS_WALL, sm.CLASS_CEILING, sm.CLASS_OBJECT,
                sm.CLASS_SELF):
        labels = np.full((HEIGHT, WIDTH), cls, dtype=np.uint8)
        out, n = infer(labels, lut, ranges_at(20.0))
        assert n == 0, sm.CLASS_NAMES[cls]
        assert np.all(out == cls)
    for cls in (sm.CLASS_INVALID, sm.CLASS_REFLECTION):
        labels = np.full((HEIGHT, WIDTH), cls, dtype=np.uint8)
        _out, n = infer(labels, lut, ranges_at(20.0))
        assert n == lut.n_valid, sm.CLASS_NAMES[cls]


def test_the_reflecting_floor_is_what_this_recovers():
    """The real case: this building's tiles mirror the wall, so the floor
    straight ahead comes back REFLECTION and INVALID rather than FLOOR.

    A frame of exactly that -- a floor read 0.8 m BELOW the floor -- is
    reclassified floor_inferred where the LiDAR ran past it, and the classes
    above z = 0 are untouched.
    """
    lut = build()
    depth = floor_depth(z=-0.8)
    rotation, translation = mount_tf()
    labels, _z = sm.classify(depth, FX, FY, CX, CY, rotation, translation)
    assert np.count_nonzero(labels == sm.CLASS_REFLECTION) > 10000
    out, n = infer(labels, lut, ranges_at(20.0))
    assert n > 10000
    assert np.count_nonzero(out == sm.CLASS_FLOOR_INFERRED) == n
    assert np.count_nonzero(out == sm.CLASS_REFLECTION) < \
        np.count_nonzero(labels == sm.CLASS_REFLECTION)


def test_a_wall_in_the_frame_keeps_its_class_and_the_floor_stops_at_its_foot():
    """A 1.5 m wall and an unmatched floor: WALL stays WALL, and the inferred
    floor reaches the wall's foot and no further.
    """
    lut = build()
    rotation, translation = mount_tf()
    depth = wall_depth(1.5)
    labels, _z = sm.classify(depth, FX, FY, CX, CY, rotation, translation)
    n_wall = np.count_nonzero(labels == sm.CLASS_WALL)
    assert n_wall > 1000
    # The LiDAR sees that same wall: 1.5 m in base_link is 1.195 m from the laser.
    out, n = infer(labels, lut, ranges_at(1.5 - LASER_POSE[0]))
    assert np.count_nonzero(out == sm.CLASS_WALL) == n_wall
    assert n > 0
    assert np.all(lut.r_floor[out == sm.CLASS_FLOOR_INFERRED] < 1.195 - MARGIN + 1e-6)


def test_class_stats_counts_floor_inferred_as_its_own_class():
    """Its own % in the logger and the videos, which is the reporting half of
    the feature: a run must be readable as "this much seen, this much inferred".
    """
    labels = np.full((HEIGHT, WIDTH), sm.CLASS_INVALID, dtype=np.uint8)
    labels[:100] = sm.CLASS_FLOOR_INFERRED
    stats = sm.class_stats(labels)
    assert 'floor_inferred' in stats['frac']
    assert stats['count']['floor_inferred'] == 100 * WIDTH
    assert stats['frac']['floor_inferred'] == pytest.approx(100.0 / HEIGHT, abs=1e-4)
    assert sum(stats['count'].values()) == HEIGHT * WIDTH
    assert set(stats['frac']) == set(sm.CLASS_NAMES.values())


def test_floor_inferred_is_never_wall_evidence():
    """Only CLASS_WALL pixels ever become wall points, by construction."""
    labels = np.full((HEIGHT, WIDTH), sm.CLASS_FLOOR_INFERRED, dtype=np.uint8)
    rotation, translation = mount_tf()
    depth = np.full((HEIGHT, WIDTH), 3.0, dtype=np.float32)
    points, rows, cols = sm.wall_points(depth, labels, FX, FY, CX, CY, rotation,
                                        translation)
    assert points.shape == (0, 3)
    assert rows.size == 0 and cols.size == 0


def test_classify_itself_never_produces_floor_inferred():
    """The inference is a separate pass on purpose: scene_mask stays a pure
    statement about the depth image.
    """
    rotation, translation = mount_tf()
    for depth in (floor_depth(), floor_depth(z=-0.8), wall_depth(2.0),
                  np.full((HEIGHT, WIDTH), np.nan, dtype=np.float32)):
        labels, _z = sm.classify(depth, FX, FY, CX, CY, rotation, translation)
        assert not np.any(labels == sm.CLASS_FLOOR_INFERRED)


def test_the_colour_and_the_name_exist_for_every_class():
    """The overlay and the legend enumerate CLASS_NAMES, so a class without a
    colour is a KeyError in the debug video and nowhere else.
    """
    assert set(sm.CLASS_COLOURS) == set(sm.CLASS_NAMES)
    assert sm.CLASS_MAX == max(sm.CLASS_NAMES)
    # Another shade of green: grouped with FLOOR by eye, still distinguishable.
    g = sm.CLASS_COLOURS[sm.CLASS_FLOOR_INFERRED]
    assert g[1] > g[0] and g[1] > g[2]
    assert g != sm.CLASS_COLOURS[sm.CLASS_FLOOR]


def test_an_empty_lut_is_a_no_op_rather_than_an_error():
    rotation, translation = mount_tf()
    lut = inf.build_lut((HEIGHT, WIDTH), FX, FY, CX, CY, rotation, translation,
                        LASER_POSE, ANGLE_MIN, 0.0, 0, r_max=R_MAX)
    assert lut.n_valid == 0
    labels = np.full((HEIGHT, WIDTH), sm.CLASS_INVALID, dtype=np.uint8)
    out, n = inf.apply(labels, lut, np.zeros(1), margin=MARGIN)
    assert n == 0
    assert np.all(out == sm.CLASS_INVALID)


def test_gaussian_weights_are_normalised_and_symmetric():
    w = inf.gaussian_beam_weights(3, 1.5)
    assert w.size == 7
    assert w.sum() == pytest.approx(1.0)
    assert np.allclose(w, w[::-1])
    assert w[3] == w.max()
    # window_k 0 is the raw beam.
    assert inf.gaussian_beam_weights(0, 1.5).tolist() == [1.0]


def test_window_k_zero_uses_the_raw_beams():
    ranges = ranges_at(20.0)
    ranges[540] = 0.3
    r_eff = r_eff_of(ranges, window_k=0)
    assert r_eff[540] == pytest.approx(0.3)
    assert r_eff[539] == pytest.approx(20.0)
