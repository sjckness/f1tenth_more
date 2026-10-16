"""The rectangle verification (wall_fusion, THE RECTANGLE). Synthetic frames only.

The camera is at base (0, 0, 0.3) looking down +x, which makes every test's
geometry checkable by hand: for a fronto-parallel plane at x = X the Z-depth
is X - t_x at EVERY pixel, because Z-depth is the optical-axis coordinate and
not the range. So a wall is one constant in the depth image, and what each
test varies is the class mask and that constant.
"""

import math
import time

import numpy as np
import pytest

from f1tenth_perception import scene_mask, wall_fusion as wf

H, W = 360, 640
FX = FY = 262.4
CX, CY = W / 2.0, H / 2.0
# Columns are the camera's axes in the base frame: optical z -> +x, optical x
# -> -y, optical y -> -z. The convention scene_mask.project_to_image uses.
ROT = np.array([[0.0, 0.0, 1.0], [-1.0, 0.0, 0.0], [0.0, -1.0, 0.0]])
TRANS = np.array([0.0, 0.0, 0.30])
LASER_Z = 0.19
SEG = ((3.0, -1.0), (3.0, 1.0))      # a 2 m wall 3 m ahead, square to the car


def frame(cls, depth_m):
    """A whole frame of one class at one depth."""
    return (np.full((H, W), cls, dtype=np.uint8),
            np.full((H, W), depth_m, dtype=np.float32))


def verify(labels, depth, segments=(SEG,), **kw):
    return wf.verify_rectangles(list(segments), labels, depth, FX, FY, CX, CY,
                                ROT, TRANS, laser_z=LASER_Z, **kw)


def settle(verdict, state=wf.VERIFY_UNVERIFIED, glass=False, p=None, cycles=4):
    """(state, glass) after `cycles` identical verdicts.

    Rejection needs reject_cycles consecutive verifiable cycles, so a test
    about the eventual verdict feeds the same one until it settles.
    """
    streak = 0
    for _ in range(cycles):
        state, glass, streak = wf.verify_step(state, glass, verdict, p, streak)
    return state, glass


# ---------------------------------------------------------------------------
# The grid and the samples
# ---------------------------------------------------------------------------

def test_grid_is_41_rows_peaking_at_the_laser():
    z, w = wf.rect_grid()
    assert z.size == 41 and w.size == 41
    assert z[0] == pytest.approx(-0.19) and z[-1] == pytest.approx(1.81)
    assert w.min() == pytest.approx(0.2)                 # the floor
    assert w[np.argmin(np.abs(z))] == pytest.approx(w.max())   # peak at z = 0


def test_samples_cover_the_wall_from_the_floor_up():
    grid = wf.rect_grid()
    pts, idx, w = wf.rect_samples([SEG], grid, LASER_Z)
    assert pts.shape == (41 * 41, 3)                     # 2 m / 0.05 + 1 rows
    assert np.unique(idx).tolist() == [0]
    assert pts[:, 2].min() == pytest.approx(0.0)         # the floor in base
    assert pts[:, 2].max() == pytest.approx(2.0)
    assert w.min() == pytest.approx(0.2)


def test_the_step_along_a_long_wall_is_capped():
    grid = wf.rect_grid()
    _pts, idx, _w = wf.rect_samples([((5.0, -10.0), (5.0, 10.0))], grid, LASER_Z)
    assert idx.size == 41 * 101           # 20 m at 0.2 m, never 400 steps


# ---------------------------------------------------------------------------
# The four verdicts
# ---------------------------------------------------------------------------

def test_a_real_wall_is_verified():
    labels, depth = frame(scene_mask.CLASS_WALL, 3.0 - TRANS[0])
    v = verify(labels, depth)[0]
    assert v['coverage'] == pytest.approx(1.0)
    assert v['support'] == pytest.approx(1.0)
    assert v['beyond_share'] == pytest.approx(0.0)
    state, glass, streak = wf.verify_step(wf.VERIFY_UNVERIFIED, False, v)
    assert state == wf.VERIFY_VERIFIED and not glass and streak == 0


def test_glass_is_verified_and_flagged():
    """WALL pixels, but the stereo matched the room 3 m BEHIND the pane."""
    labels, depth = frame(scene_mask.CLASS_WALL, 6.0)
    v = verify(labels, depth)[0]
    assert v['support'] == pytest.approx(1.0)
    assert v['beyond_share'] == pytest.approx(1.0)
    state, glass, _streak = wf.verify_step(wf.VERIFY_UNVERIFIED, False, v)
    assert state == wf.VERIFY_VERIFIED and glass


def test_a_chair_filling_the_rectangle_rejects_the_wall():
    labels, depth = frame(scene_mask.CLASS_OBJECT, 1.5)
    v = verify(labels, depth)[0]
    assert v['coverage'] == pytest.approx(1.0) and v['support'] == pytest.approx(0.0)
    # One cycle is not enough: rejection takes the wall away from every
    # consumer, so it needs reject_cycles of them.
    assert wf.verify_step(wf.VERIFY_UNVERIFIED, False, v)[0] == wf.VERIFY_UNVERIFIED
    assert settle(v)[0] == wf.VERIFY_REJECTED


def test_a_chair_over_most_of_the_wall_leaves_it_unverified():
    """Enough of the wall is hidden that support lands in the dead band."""
    labels, depth = frame(scene_mask.CLASS_WALL, 3.0 - TRANS[0])
    chair = np.zeros((H, W), dtype=bool)
    chair[:, : int(0.54 * W)] = True          # most, not all, of the wall
    labels[chair] = scene_mask.CLASS_OBJECT
    depth[chair] = 1.5
    v = verify(labels, depth)[0]
    assert 0.2 < v['support'] < 0.5
    assert settle(v) == (wf.VERIFY_UNVERIFIED, False)


def test_a_surface_nearer_than_the_line_is_against_it():
    labels, depth = frame(scene_mask.CLASS_WALL, 1.5)
    v = verify(labels, depth)[0]
    assert v['support'] == pytest.approx(0.0) and v['against'] > 0.0


# ---------------------------------------------------------------------------
# Coverage: "nothing to say" is never "no wall"
# ---------------------------------------------------------------------------

def test_a_mostly_invalid_rectangle_changes_nothing():
    labels, depth = frame(scene_mask.CLASS_WALL, 3.0 - TRANS[0])
    labels[:, : int(0.8 * W)] = scene_mask.CLASS_INVALID
    v = verify(labels, depth)[0]
    assert v['coverage'] < 0.25
    for state in (wf.VERIFY_UNVERIFIED, wf.VERIFY_VERIFIED, wf.VERIFY_REJECTED):
        assert settle(v, state) == (state, False)


def test_floor_and_ceiling_are_not_informative():
    for cls in (scene_mask.CLASS_FLOOR, scene_mask.CLASS_FLOOR_INFERRED,
                scene_mask.CLASS_CEILING, scene_mask.CLASS_REFLECTION,
                scene_mask.CLASS_SELF, scene_mask.CLASS_INVALID):
        labels, depth = frame(cls, 3.0 - TRANS[0])
        assert verify(labels, depth)[0]['coverage'] == pytest.approx(0.0)


def test_a_wall_behind_the_car_is_never_informative():
    labels, depth = frame(scene_mask.CLASS_WALL, 3.0)
    v = verify(labels, depth, segments=(((-3.0, -1.0), (-3.0, 1.0)),))[0]
    assert v['n_in_image'] == 0 and v['coverage'] == pytest.approx(0.0)


def test_nan_depth_is_not_informative():
    labels, depth = frame(scene_mask.CLASS_WALL, np.nan)
    assert verify(labels, depth)[0]['coverage'] == pytest.approx(0.0)


# ---------------------------------------------------------------------------
# The sticky state machine
# ---------------------------------------------------------------------------

def test_the_dead_band_holds_whatever_state_it_is_given():
    mid = {'coverage': 1.0, 'support': 0.35, 'beyond_share': 0.0}
    for state in (wf.VERIFY_UNVERIFIED, wf.VERIFY_VERIFIED, wf.VERIFY_REJECTED):
        assert settle(mid, state)[0] == state


# ---------------------------------------------------------------------------
# F1: the stereo's own range, the columns' tolerance, and asymmetric rejection
# ---------------------------------------------------------------------------

def test_a_wall_beyond_the_stereos_range_is_not_evidence_either_way():
    """The 15.5 m end wall: the stereo reports ~7 m there, so read literally
    every sample on it is AGAINST it. Past max_range_m nothing is
    informative, so the wall is left UNVERIFIED -- which is lidar_only."""
    labels, depth = frame(scene_mask.CLASS_WALL, 7.0)
    far = ((15.5, -1.5), (15.5, 1.5))
    v = verify(labels, depth, segments=(far,))[0]
    assert v['n_in_image'] > 0          # the samples DO land in the frame
    assert v['coverage'] == pytest.approx(0.0)
    assert v['against'] == pytest.approx(0.0)
    assert settle(v) == (wf.VERIFY_UNVERIFIED, False)


def test_the_range_limit_is_the_parameter_not_the_geometry():
    labels, depth = frame(scene_mask.CLASS_WALL, 7.0)
    far = ((15.5, -1.5), (15.5, 1.5))
    loose = wf.RectParams(max_range_m=20.0)
    v = wf.verify_rectangles([far], labels, depth, FX, FY, CX, CY, ROT, TRANS,
                             laser_z=LASER_Z, p=loose)[0]
    assert v['coverage'] > 0.9 and v['against'] > 0.0
    assert settle(v, p=loose)[0] == wf.VERIFY_REJECTED


def test_a_wall_straddling_the_limit_is_judged_on_the_near_half_only():
    labels, depth = frame(scene_mask.CLASS_WALL, 6.0 - TRANS[0])
    # Square to the car at 6 m, so every sample is at the same optical z,
    # against a limit of 4 m: nothing is informative.
    tight = wf.RectParams(max_range_m=4.0)
    v = wf.verify_rectangles([((6.0, -1.0), (6.0, 1.0))], labels, depth,
                             FX, FY, CX, CY, ROT, TRANS, laser_z=LASER_Z, p=tight)[0]
    assert v['coverage'] == pytest.approx(0.0)


def test_the_tolerance_is_the_columns_quadratic_model():
    """base + quad * d^2, the same two numbers the column check uses: 6 cm at
    1 m and 82 cm at 8 m, where the old max(0.15, 0.05 d) gave 15 and 40."""
    p = wf.RectParams()
    assert p.tol_base_m == pytest.approx(0.05)
    assert p.tol_quad_per_m2 == pytest.approx(0.012)
    near = wf.tolerance([1.0], p.tol_base_m, p.tol_quad_per_m2)[0]
    far = wf.tolerance([8.0], p.tol_base_m, p.tol_quad_per_m2)[0]
    assert near == pytest.approx(0.062) and far == pytest.approx(0.818)
    # A wall 12 cm off the line at 1 m is now OUTSIDE the tolerance.
    labels, depth = frame(scene_mask.CLASS_WALL, 1.12)
    v = verify(labels, depth, segments=(((1.0, -0.4), (1.0, 0.4)),))[0]
    assert v['support'] == pytest.approx(1.0)   # beyond, not at
    assert v['beyond_share'] == pytest.approx(1.0)


def test_rejection_needs_three_verifiable_cycles_and_a_gap_resets_it():
    bad = {'coverage': 1.0, 'support': 0.0, 'beyond_share': 0.0}
    blind = {'coverage': 0.0, 'support': 0.0, 'beyond_share': 0.0}
    state, glass, streak = wf.verify_step(wf.VERIFY_UNVERIFIED, False, bad)
    assert (state, streak) == (wf.VERIFY_UNVERIFIED, 1)
    state, glass, streak = wf.verify_step(state, glass, bad, None, streak)
    assert (state, streak) == (wf.VERIFY_UNVERIFIED, 2)
    # A cycle with no coverage neither advances nor resets the count.
    state, glass, streak = wf.verify_step(state, glass, blind, None, streak)
    assert (state, streak) == (wf.VERIFY_UNVERIFIED, 2)
    state, glass, streak = wf.verify_step(state, glass, bad, None, streak)
    assert state == wf.VERIFY_REJECTED and streak == 3


def test_one_good_cycle_resets_the_rejection_count():
    bad = {'coverage': 1.0, 'support': 0.1, 'beyond_share': 0.0}
    mid = {'coverage': 1.0, 'support': 0.35, 'beyond_share': 0.0}
    streak = 0
    state, glass = wf.VERIFY_UNVERIFIED, False
    for _ in range(2):
        state, glass, streak = wf.verify_step(state, glass, bad, None, streak)
    state, glass, streak = wf.verify_step(state, glass, mid, None, streak)
    assert streak == 0 and state == wf.VERIFY_UNVERIFIED
    state, glass, streak = wf.verify_step(state, glass, bad, None, streak)
    assert state == wf.VERIFY_UNVERIFIED and streak == 1


def test_verification_is_still_immediate():
    good = {'coverage': 1.0, 'support': 1.0, 'beyond_share': 0.0}
    assert wf.verify_step(wf.VERIFY_UNVERIFIED, False, good)[0] == wf.VERIFY_VERIFIED


def test_rejection_clears_glass():
    verdict = {'coverage': 1.0, 'support': 0.0, 'beyond_share': 1.0}
    assert settle(verdict, wf.VERIFY_VERIFIED, True) == (wf.VERIFY_REJECTED, False)


def test_glass_needs_almost_every_sample_beyond():
    half = {'coverage': 1.0, 'support': 1.0, 'beyond_share': 0.75}
    assert settle(half) == (wf.VERIFY_VERIFIED, False)


# ---------------------------------------------------------------------------
# The YOLO mask, the depth encoding, and the cost
# ---------------------------------------------------------------------------

def test_the_object_mask_may_be_supplied_separately():
    labels, depth = frame(scene_mask.CLASS_WALL, 3.0 - TRANS[0])
    yolo = np.ones((H, W), dtype=bool)
    v = verify(labels, depth, object_mask=yolo)[0]
    assert v['support'] == pytest.approx(0.0)


def test_a_mono16_millimetre_depth_reads_the_same_as_metres():
    labels, depth = frame(scene_mask.CLASS_WALL, 3.0 - TRANS[0])
    mm = np.rint(depth * 1000.0).astype(np.uint16)
    a = verify(labels, depth)[0]
    b = verify(labels, mm, depth_scale=0.001)[0]
    assert b['support'] == pytest.approx(a['support'])
    assert b['coverage'] == pytest.approx(a['coverage'], abs=1e-6)


def test_a_zero_in_an_integer_depth_is_no_measurement():
    labels, _d = frame(scene_mask.CLASS_WALL, 3.0)
    zeros = np.zeros((H, W), dtype=np.uint16)
    assert verify(labels, zeros, depth_scale=0.001)[0]['coverage'] == pytest.approx(0.0)


def test_the_cost_is_linear_in_the_samples_and_within_budget():
    """Timed at the track count the tracker actually holds, and at 16.

    The cost is the sample count: 41 z rows times the along-track steps, per
    track. A 3 m wall is 61 steps, so eight of them is 20k samples and
    sixteen is 40k. Eight is what this tracker holds in the corridor (the
    publish cap is 4); 16 is the design ceiling and is printed rather than
    asserted, because at that point the sampling steps are the lever.
    """
    rng = np.random.default_rng(5)
    labels = rng.integers(0, scene_mask.CLASS_MAX + 1, size=(H, W)).astype(np.uint8)
    depth = rng.uniform(0.5, 8.0, size=(H, W)).astype(np.float32)
    grid = wf.rect_grid()

    def timed(n):
        segments = [((2.0 + 0.3 * k, -1.5), (2.0 + 0.3 * k, 1.5)) for k in range(n)]
        ms = []
        for _ in range(20):
            t0 = time.perf_counter()
            wf.verify_rectangles(segments, labels, depth, FX, FY, CX, CY, ROT, TRANS,
                                 laser_z=LASER_Z, grid=grid)
            ms.append((time.perf_counter() - t0) * 1e3)
        return float(np.median(ms)), float(np.percentile(ms, 95))

    eight, sixteen = timed(8), timed(16)
    print(f'\n  wall_fusion verify_rectangles, 640x360: 8 tracks '
          f'{eight[0]:.2f} ms p50 / {eight[1]:.2f} p95, 16 tracks '
          f'{sixteen[0]:.2f} / {sixteen[1]:.2f}')
    assert eight[0] < 5.0
