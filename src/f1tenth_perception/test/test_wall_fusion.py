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


# ===========================================================================
# Per-cycle fusion: LiDAR lines + column statuses -> observations
# ===========================================================================

from f1tenth_perception import scene_mask as sm                      # noqa: E402
from f1tenth_perception.scene_labels import WallLine                 # noqa: E402
from test_scene_mask import FX, FY, CX, CY, mount_tf, wall_depth, classify  # noqa: E402,E501

FULL_W = 640


def projector():
    rot, trans = mount_tf()
    return lambda pts: sm.project_to_image(pts, FX, FY, CX, CY, rot, trans)


def front_wall_line(x=3.0, half=2.0, inliers=120, rms=0.01):
    """A wall across the car's path at base_link x, normal pointing forward."""
    return WallLine(1.0, 0.0, x, -half, half, inliers, rms, 'lidar')


def side_wall_line(y=-1.2, t0=-1.0, t1=4.0, inliers=200, rms=0.01):
    """A wall along the car, on its right: the open-corridor case."""
    return WallLine(0.0, -1.0, abs(y), t0, t1, inliers, rms, 'lidar')


def status_array(width=FULL_W, default=wf.STATUS_EMPTY, **spans):
    """A status array with named runs set, e.g. agree=(300, 340)."""
    out = np.full(width, default, dtype=np.uint8)
    for name, (a, b) in spans.items():
        out[a:b] = getattr(wf, f'STATUS_{name.upper()}')
    return out


def check_from(status):
    width = len(status)
    return {'status': status, 'lidar_m': np.full(width, np.nan),
            'stereo_m': np.full(width, np.nan), 'n_px': np.zeros(width, dtype=np.int64),
            'tol_m': np.zeros(width)}


def nan_profile(width=FULL_W):
    return np.full(width, np.nan), np.full(width, np.nan)


def test_a_wall_both_sensors_see_is_sourced_both():
    line = front_wall_line()
    c0, c1 = wf.line_columns(line, projector(), FULL_W)
    assert c1 > c0
    check = check_from(status_array(agree=(c0, c1)))
    obs, rejected = wf.observations_from_lines(
        [line], check, projector(), FULL_W, *nan_profile())
    assert rejected == 0 and len(obs) == 1
    assert obs[0].source == wf.SOURCE_BOTH
    assert obs[0].counts['agree'] == c1 - c0


def test_a_side_wall_outside_the_camera_view_stays_lidar_and_is_not_punished():
    """The open corridor: the LiDAR sees the side wall, the camera cannot. It
    must survive as a lidar observation with a normal confidence."""
    line = side_wall_line()
    check = check_from(status_array())
    obs, rejected = wf.observations_from_lines(
        [line], check, projector(), FULL_W, *nan_profile())
    assert rejected == 0 and len(obs) == 1
    assert obs[0].source == wf.SOURCE_LIDAR
    assert obs[0].counts['agree'] == 0
    # Neutral corroboration, not zero: "the camera cannot see there" is not
    # evidence against the LiDAR.
    assert obs[0].confidence > 0.5


def test_a_contradicted_wall_is_rejected_and_counted():
    line = front_wall_line()
    c0, c1 = wf.line_columns(line, projector(), FULL_W)
    check = check_from(status_array(disagree=(c0, c1)))
    obs, rejected = wf.observations_from_lines(
        [line], check, projector(), FULL_W, *nan_profile())
    assert obs == [] and rejected == 1


def test_a_minority_of_disagreeing_columns_does_not_reject_the_wall():
    line = front_wall_line()
    c0, c1 = wf.line_columns(line, projector(), FULL_W)
    mid = c0 + (c1 - c0) // 5
    check = check_from(status_array(agree=(c0, c1), disagree=(c0, mid)))
    obs, rejected = wf.observations_from_lines(
        [line], check, projector(), FULL_W, *nan_profile())
    assert rejected == 0 and len(obs) == 1 and obs[0].source == wf.SOURCE_BOTH


def test_the_default_height_is_used_when_the_stereo_has_no_heights():
    line = front_wall_line()
    c0, c1 = wf.line_columns(line, projector(), FULL_W)
    check = check_from(status_array(agree=(c0, c1)))
    obs, _ = wf.observations_from_lines(
        [line], check, projector(), FULL_W, *nan_profile(), default_height=2.5)
    assert obs[0].height_measured is False
    assert (obs[0].z_min, obs[0].z_max) == (0.0, 2.5)


def test_a_measured_height_comes_from_the_stereo_columns():
    line = front_wall_line()
    c0, c1 = wf.line_columns(line, projector(), FULL_W)
    zmin, zmax = nan_profile()
    zmin[c0:c1] = 0.02
    zmax[c0:c1] = 1.85
    check = check_from(status_array(agree=(c0, c1)))
    obs, _ = wf.observations_from_lines(
        [line], check, projector(), FULL_W, zmin, zmax)
    assert obs[0].height_measured is True
    assert obs[0].z_min == pytest.approx(0.02) and obs[0].z_max == pytest.approx(1.85)


def test_a_lidar_only_wall_keeps_the_default_height_even_with_stereo_heights():
    """Heights belong to the columns the stereo AGREED on. Borrowing them for
    a wall the stereo never confirmed would invent a measurement."""
    line = front_wall_line()
    c0, c1 = wf.line_columns(line, projector(), FULL_W)
    zmin, zmax = nan_profile()
    zmin[c0:c1], zmax[c0:c1] = 0.0, 1.0
    check = check_from(status_array(lidar_only=(c0, c1)))
    obs, _ = wf.observations_from_lines([line], check, projector(), FULL_W, zmin, zmax)
    assert obs[0].source == wf.SOURCE_LIDAR
    assert obs[0].height_measured is False and obs[0].z_max == 2.5


# --- stereo-only walls -----------------------------------------------------

def glass_points(x=2.0, c0=200, c1=300, n_per_col=20, spread=1.0):
    """Points on a flat pane at base_link x, returned per column span."""
    def column_points(a, b):
        cols = np.arange(a, b)
        ys = np.linspace(-spread, spread, len(cols) * n_per_col)
        return np.column_stack([np.full(ys.size, x), ys, np.zeros(ys.size)])
    return column_points


def test_a_contiguous_stereo_only_span_becomes_a_stereo_wall():
    """Glass: the camera sees it, the LiDAR does not."""
    check = check_from(status_array(stereo_only=(200, 300)))
    obs = wf.stereo_only_observations(check, glass_points(), FULL_W, min_span_columns=8)
    assert len(obs) == 1
    assert obs[0].source == wf.SOURCE_STEREO
    assert obs[0].line.d == pytest.approx(2.0, abs=0.02)
    assert obs[0].confidence < 0.5          # nothing cross-checks it


def test_a_single_stereo_only_column_is_noise_not_a_wall():
    check = check_from(status_array(stereo_only=(200, 203)))
    assert wf.stereo_only_observations(
        check, glass_points(), FULL_W, min_span_columns=8) == []


def test_a_stereo_only_span_with_too_few_points_is_not_a_wall():
    check = check_from(status_array(stereo_only=(200, 300)))
    assert wf.stereo_only_observations(
        check, lambda a, b: np.zeros((5, 3)), FULL_W, min_span_columns=8) == []


def test_a_stereo_only_span_too_short_in_metres_is_not_a_wall():
    check = check_from(status_array(stereo_only=(200, 300)))
    assert wf.stereo_only_observations(
        check, glass_points(spread=0.05), FULL_W, min_span_columns=8,
        min_span_m=0.5) == []


# --- objects ---------------------------------------------------------------

def test_an_object_in_front_of_a_wall_does_not_bend_the_wall_line():
    """The guarantee the whole pipeline rests on: OBJECT pixels never reach a
    wall fit. Classified with the object's mask, the WALL points are the wall
    alone, and the stereo-only fit lands on the wall, not between the two."""
    depth = wall_depth(3.0, width=FULL_W, height=360)
    # A box at 1.2 m, dead centre, as the depth a YOLO mask covers.
    obj = np.zeros(depth.shape, dtype=bool)
    obj[150:280, 280:360] = True
    depth[obj] = 1.2
    labels, _ = classify(depth, object_mask=obj)
    assert np.all(labels[obj] == sm.CLASS_OBJECT)
    pts, _rows, cols = sm.wall_points(depth, labels, FX, FY, CX, CY, *mount_tf())
    assert pts.shape[0] > 0
    assert np.max(np.abs(pts[:, 0] - 3.0)) < 1e-3        # no 1.2 m points at all

    def column_points(a, b):
        sel = (cols >= a) & (cols < b)
        return pts[sel]

    check = check_from(status_array(stereo_only=(280, 360)))
    obs = wf.stereo_only_observations(check, column_points, FULL_W, min_span_columns=8)
    assert len(obs) == 1
    assert obs[0].line.d == pytest.approx(3.0, abs=0.02)


# --- confidence ------------------------------------------------------------

def test_confidence_rises_with_evidence_and_falls_with_a_loose_fit():
    tight = wf.confidence(wf.SOURCE_LIDAR, 120, 0.005, 0, 0)
    loose = wf.confidence(wf.SOURCE_LIDAR, 120, 0.20, 0, 0)
    sparse = wf.confidence(wf.SOURCE_LIDAR, 5, 0.005, 0, 0)
    assert tight > loose and tight > sparse
    assert 0.0 <= loose <= 1.0 and 0.0 <= sparse <= 1.0


def test_agreement_raises_confidence_and_a_stereo_only_wall_is_capped():
    both = wf.confidence(wf.SOURCE_BOTH, 120, 0.01, 40, 40)
    lidar = wf.confidence(wf.SOURCE_LIDAR, 120, 0.01, 0, 40)
    stereo = wf.confidence(wf.SOURCE_STEREO, 0, float('nan'), 40, 40)
    assert both > lidar > stereo
    assert stereo < 0.5


def test_confidence_is_always_in_range():
    for args in ((wf.SOURCE_BOTH, 10 ** 6, 0.0, 10, 10),
                 (wf.SOURCE_LIDAR, 0, 10.0, 0, 0),
                 (wf.SOURCE_STEREO, 0, float('nan'), 0, 0)):
        assert 0.0 <= wf.confidence(*args) <= 1.0


# --- column spans ----------------------------------------------------------

def test_a_wall_entirely_behind_the_car_covers_no_columns():
    behind = WallLine(-1.0, 0.0, 1.0, -1.0, 1.0, 50, 0.01, 'lidar')
    assert wf.line_columns(behind, projector(), FULL_W) == (0, 0)


def test_a_wall_wider_than_the_frame_is_clamped_to_it():
    wide = front_wall_line(x=1.0, half=20.0)
    c0, c1 = wf.line_columns(wide, projector(), FULL_W)
    assert (c0, c1) == (0, FULL_W)


def test_column_extent_gives_the_vertical_span_of_each_column():
    lo, hi = wf.column_extent([3, 3, 3, 7], [0.1, 1.9, 0.5, 2.0], WIDTH)
    assert lo[3] == pytest.approx(0.1) and hi[3] == pytest.approx(1.9)
    assert lo[7] == pytest.approx(2.0) and hi[7] == pytest.approx(2.0)
    assert np.isnan(lo[0]) and np.isnan(hi[0])


def test_column_extent_of_nothing():
    lo, hi = wf.column_extent(np.empty(0), np.empty(0), WIDTH)
    assert np.all(np.isnan(lo)) and np.all(np.isnan(hi))


def test_cross_check_profile_matches_cross_check_on_the_same_columns():
    """The profile entry point is the same decision, with the percentile step
    already done on the camera side."""
    stereo = np.full(WIDTH, np.nan)
    stereo[10] = 2.03
    stereo[11] = 4.0
    stereo[12] = 1.0
    a = wf.cross_check_profile([10, 11], [2.0, 2.0], stereo, WIDTH)
    assert a['status'][10] == wf.STATUS_AGREE
    assert a['status'][11] == wf.STATUS_DISAGREE
    assert a['status'][12] == wf.STATUS_STEREO_ONLY
    assert a['status'][13] == wf.STATUS_EMPTY
    b = check(scan=[(10, 2.0), (11, 2.0)],
              wall=wall_column(10, 2.03) + wall_column(11, 4.0) + wall_column(12, 1.0))
    assert list(a['status'][:14]) == list(b['status'][:14])


def test_cross_check_profile_pads_a_short_profile():
    out = wf.cross_check_profile([2], [1.0], np.array([np.nan, np.nan, 1.0]), WIDTH)
    assert out['status'][2] == wf.STATUS_AGREE
    assert out['status'][WIDTH - 1] == wf.STATUS_EMPTY
