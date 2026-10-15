"""wall_fusion.py: the per-column LiDAR/stereo cross-check.

One test per status, plus the two properties the stage stands on: the
tolerance grows with the square of range, and a column at the edge of the
camera's field of view is not evidence of disagreement.
"""

import numpy as np
import pytest

from f1tenth_perception import wall_fusion as wf

WIDTH = 64


def check(scan=(), wall=(), width=WIDTH, **kw):
    """cross_check from (col, depth) pairs, for readability in the tests."""
    sc = np.array([c for c, _ in scan], dtype=float) if scan else np.empty(0)
    sd = np.array([d for _, d in scan], dtype=float) if scan else np.empty(0)
    wc = np.array([c for c, _ in wall], dtype=float) if wall else np.empty(0)
    wd = np.array([d for _, d in wall], dtype=float) if wall else np.empty(0)
    return wf.cross_check(sc, sd, wc, wd, width, **kw)


def wall_column(col, depth, n=10):
    """n WALL pixels in one column, all at the same depth."""
    return [(col, depth)] * n


# ---------------------------------------------------------------------------
# One per status
# ---------------------------------------------------------------------------

def test_a_column_both_sensors_agree_on_is_agree():
    out = check(scan=[(10, 2.0)], wall=wall_column(10, 2.03))
    assert out['status'][10] == wf.STATUS_AGREE
    assert out['lidar_m'][10] == pytest.approx(2.0)
    assert out['stereo_m'][10] == pytest.approx(2.03)


def test_a_lidar_return_with_no_stereo_wall_is_lidar_only():
    out = check(scan=[(10, 2.0)])
    assert out['status'][10] == wf.STATUS_LIDAR_ONLY
    assert np.isnan(out['stereo_m'][10])


def test_stereo_wall_with_no_lidar_return_is_stereo_only():
    """Glass and very dark surfaces: the camera sees them, the LiDAR does not."""
    out = check(wall=wall_column(10, 2.0))
    assert out['status'][10] == wf.STATUS_STEREO_ONLY
    assert np.isnan(out['lidar_m'][10])


def test_two_sensors_far_apart_is_disagree():
    out = check(scan=[(10, 2.0)], wall=wall_column(10, 4.0))
    assert out['status'][10] == wf.STATUS_DISAGREE


def test_a_column_with_nothing_is_empty():
    out = check(scan=[(10, 2.0)])
    assert out['status'][30] == wf.STATUS_EMPTY
    assert np.isnan(out['lidar_m'][30]) and np.isnan(out['stereo_m'][30])


def test_every_column_gets_exactly_one_status():
    out = check(scan=[(10, 2.0), (11, 2.0)], wall=wall_column(11, 2.0) + wall_column(20, 3.0))
    assert out['status'].shape == (WIDTH,)
    assert set(np.unique(out['status'])) <= set(wf.STATUS_NAMES)
    assert sum(wf.status_counts(out['status']).values()) == WIDTH


# ---------------------------------------------------------------------------
# The tolerance
# ---------------------------------------------------------------------------

def test_the_tolerance_grows_with_the_square_of_range():
    tol = wf.tolerance([1.0, 2.0, 6.0], base_m=0.05, quad_per_m2=0.012)
    assert tol[0] == pytest.approx(0.062)
    assert tol[1] == pytest.approx(0.098)
    assert tol[2] == pytest.approx(0.482)
    assert tol[2] - tol[1] > tol[1] - tol[0]


def test_the_same_absolute_error_agrees_far_and_disagrees_near():
    """The whole point of a growing tolerance, in one test: 25 cm of stereo
    error is noise at 6 m and a contradiction at 1 m."""
    near = check(scan=[(10, 1.0)], wall=wall_column(10, 1.25))
    far = check(scan=[(10, 6.0)], wall=wall_column(10, 6.25))
    assert near['status'][10] == wf.STATUS_DISAGREE
    assert far['status'][10] == wf.STATUS_AGREE


def test_the_tolerance_is_taken_at_the_lidar_range_when_there_is_one():
    out = check(scan=[(10, 6.0)], wall=wall_column(10, 6.3))
    assert out['tol_m'][10] == pytest.approx(wf.tolerance(6.0, 0.05, 0.012))


# ---------------------------------------------------------------------------
# Field of view
# ---------------------------------------------------------------------------

def test_a_column_at_the_edge_of_the_frame_is_checked_like_any_other():
    out = check(scan=[(0, 2.0), (WIDTH - 1, 2.0)],
                wall=wall_column(0, 2.02) + wall_column(WIDTH - 1, 5.0))
    assert out['status'][0] == wf.STATUS_AGREE
    assert out['status'][WIDTH - 1] == wf.STATUS_DISAGREE


def test_beams_outside_the_frame_are_dropped_not_counted_as_disagreement():
    """The LiDAR sees 270 deg and the camera about 100. A beam the camera
    cannot see is the caller's LIDAR_ONLY, never this stage's DISAGREE."""
    out = check(scan=[(-5, 2.0), (WIDTH + 3, 2.0), (10, 2.0)],
                wall=wall_column(10, 2.0))
    counts = wf.status_counts(out['status'])
    assert counts['disagree'] == 0
    assert counts['agree'] == 1
    assert counts['lidar_only'] == 0


def test_a_beam_projected_behind_the_lens_is_dropped():
    out = check(scan=[(np.nan, 2.0), (10, 2.0)], wall=wall_column(10, 2.0))
    assert wf.status_counts(out['status'])['agree'] == 1


# ---------------------------------------------------------------------------
# The column summary
# ---------------------------------------------------------------------------

def test_a_few_stray_pixels_are_not_stereo_evidence():
    out = check(scan=[(10, 2.0)], wall=[(10, 2.0)] * 2, min_px=5)
    assert out['status'][10] == wf.STATUS_LIDAR_ONLY
    assert out['n_px'][10] == 2


def test_the_column_summary_is_the_near_end_not_the_middle():
    """A column with a near wall low down and a far wall above it: the LiDAR
    reports the near one, so the stereo summary must too."""
    wall = wall_column(10, 2.0, n=10) + wall_column(10, 8.0, n=10)
    out = check(scan=[(10, 2.0)], wall=wall, percentile=20.0)
    assert out['stereo_m'][10] == pytest.approx(2.0)
    assert out['status'][10] == wf.STATUS_AGREE
    assert np.median([d for _, d in wall]) == pytest.approx(5.0)   # the wrong answer


def test_one_bad_pixel_does_not_decide_the_column():
    """Why a percentile and not the minimum."""
    wall = [(10, 0.3)] + wall_column(10, 3.0, n=20)
    out = check(scan=[(10, 3.0)], wall=wall, percentile=20.0)
    assert out['stereo_m'][10] == pytest.approx(3.0)
    assert out['status'][10] == wf.STATUS_AGREE


def test_the_lidar_column_takes_the_nearest_of_several_beams():
    out = check(scan=[(10, 5.0), (10, 2.0)], wall=wall_column(10, 2.0))
    assert out['lidar_m'][10] == pytest.approx(2.0)
    assert out['status'][10] == wf.STATUS_AGREE


def test_column_ranges_of_nothing():
    vals, npx = wf.column_ranges(np.empty(0), np.empty(0), WIDTH)
    assert np.all(np.isnan(vals)) and not npx.any()


def test_non_finite_depths_are_not_evidence():
    out = check(scan=[(10, np.inf)], wall=[(10, np.nan)] * 10)
    assert out['status'][10] == wf.STATUS_EMPTY


# ---------------------------------------------------------------------------
# Spans
# ---------------------------------------------------------------------------

def test_contiguous_spans_finds_runs_and_drops_short_ones():
    mask = np.zeros(20, dtype=bool)
    mask[2:9] = True       # a 7-wide run
    mask[12] = True        # a single column
    assert wf.contiguous_spans(mask, min_width=3) == [(2, 9)]
    assert wf.contiguous_spans(mask, min_width=1) == [(2, 9), (12, 13)]


def test_contiguous_spans_handles_a_run_touching_each_edge():
    mask = np.array([True, True, False, True, True])
    assert wf.contiguous_spans(mask, min_width=2) == [(0, 2), (3, 5)]


def test_contiguous_spans_of_nothing():
    assert wf.contiguous_spans(np.zeros(0, dtype=bool)) == []
    assert wf.contiguous_spans(np.zeros(5, dtype=bool)) == []


# ---------------------------------------------------------------------------
# Cost
# ---------------------------------------------------------------------------

def test_cross_check_is_cheap_on_a_full_frame(capsys):
    import math
    import time
    rng = np.random.default_rng(0)
    wall_cols = rng.integers(0, 640, size=120000)
    wall_depths = rng.uniform(1.0, 8.0, size=120000)
    scan_cols = rng.integers(0, 640, size=1081)
    scan_depths = rng.uniform(1.0, 8.0, size=1081)
    best = math.inf
    for _ in range(5):
        t0 = time.perf_counter()
        wf.cross_check(scan_cols, scan_depths, wall_cols, wall_depths, 640)
        best = min(best, (time.perf_counter() - t0) * 1e3)
    with capsys.disabled():
        print(f'\n  wall_fusion cross_check, 640 columns / 120k WALL px: {best:.1f} ms')
    # Measured 15.3 ms on the Orin. The budget is a regression guard, not a
    # target: the float lexsort this replaced took 40 ms on the same input.
    assert best < 30.0


def test_the_depth_quantisation_of_the_column_sort_is_harmless():
    """The sort packs depth into an integer; the error must stay far below
    the smallest agreement tolerance (5 cm)."""
    depths = np.repeat([1.23456, 2.00049, 7.99951], 10)
    cols = np.repeat([1, 2, 3], 10)
    vals, _ = wf.column_ranges(cols, depths, WIDTH, percentile=50.0, min_px=1)
    for col, want in ((1, 1.23456), (2, 2.00049), (3, 7.99951)):
        assert abs(vals[col] - want) <= 0.0005


def test_a_depth_beyond_the_quantisation_headroom_is_clamped_not_wrapped():
    """A wrapped key would put a 2000 m reading in the wrong column."""
    vals, _ = wf.column_ranges([5] * 10, [5000.0] * 10, WIDTH, min_px=1)
    assert np.isfinite(vals[5]) and vals[5] > 1000.0
    assert np.count_nonzero(np.isfinite(vals)) == 1
