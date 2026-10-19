"""Both backends of extract_segments, on the same inputs.

The Python in extraction.py is the REFERENCE and corridor_perception_c's
libextract_lines.so is the thing under test. Every case here runs twice, once
per backend, and the C has to produce the same segments in the same order to
1e-4 on metres and radians. tools/extract_backends_equiv.py does the same over
all 2292 scans of a recording; this file is the part that runs in CI.

A checkout without corridor_perception_c installed skips the C half rather
than failing: that is the documented fallback, and these tests are the place it
has to stay true.
"""

import math

import numpy as np
import pytest

from corridor_perception import extraction_c
from corridor_perception.extraction import (ExtractionParams, extract_segments,
                                            extract_segments_python)
from corridor_perception.scan import clean_ranges

N_BEAMS = 1081
ANGLE_MIN = math.radians(-135.0)
ANGLE_INC = math.radians(270.0) / (N_BEAMS - 1)
ANGLES = ANGLE_MIN + np.arange(N_BEAMS) * ANGLE_INC

requires_c = pytest.mark.skipif(not extraction_c.available(),
                                reason=f'corridor_perception_c absent: '
                                       f'{extraction_c.load_error()}')


def scan_of(*surfaces, noise=0.0, seed=0):
    """Ranges of a scan holding `surfaces`, each ((x0, y0), (x1, y1))."""
    rng = np.random.default_rng(seed)
    r = np.full(N_BEAMS, np.inf)
    v = np.column_stack([np.cos(ANGLES), np.sin(ANGLES)])
    for (x0, y0), (x1, y1) in surfaces:
        a = np.array([x0, y0], dtype=float)
        d = np.array([x1 - x0, y1 - y0], dtype=float)
        den = v[:, 0] * d[1] - v[:, 1] * d[0]
        with np.errstate(divide='ignore', invalid='ignore'):
            u = (v[:, 0] * a[1] - v[:, 1] * a[0]) / den
            s = np.where(np.abs(v[:, 0]) > 1e-9,
                         (a[0] + u * d[0]) / np.where(np.abs(v[:, 0]) > 1e-9, v[:, 0], 1.0),
                         (a[1] + u * d[1]) / np.where(np.abs(v[:, 1]) > 1e-9, v[:, 1], 1.0))
        hit = np.isfinite(u) & (u >= 0.0) & (u <= 1.0) & (s > 0.05)
        r = np.where(hit & (s < r), s, r)
    if noise:
        r = np.where(np.isfinite(r), r + rng.normal(0.0, noise, r.size), r)
    return clean_ranges(np.where(np.isfinite(r), r, 0.0), 0.06, 20.0)


def both(ranges, p=None):
    """(python segments, c segments) over the same float32 input."""
    p = p or ExtractionParams()
    as32 = np.asarray(ranges, dtype=np.float32)
    same = clean_ranges(as32.astype(np.float64), -math.inf, math.inf)
    py_p = ExtractionParams(**{**vars(p), 'backend': 'python'})
    c_p = ExtractionParams(**{**vars(p), 'backend': 'c'})
    py_stats, c_stats = {}, {}
    py = extract_segments(same, ANGLES, py_p, py_stats)
    c = extract_segments(same, ANGLES, c_p, c_stats)
    return (py, py_stats), (c, c_stats)


def assert_same(py, c, tol=1e-4):
    (py_segs, py_stats), (c_segs, c_stats) = py, c
    assert len(c_segs) == len(py_segs), (
        f'{len(c_segs)} segments from the C against {len(py_segs)} from the reference')
    assert c_stats == py_stats, f'gate stats differ: {c_stats} vs {py_stats}'
    for k, (a, b) in enumerate(zip(py_segs, c_segs)):
        assert abs(a.rho - b.rho) <= tol, f'segment {k} rho'
        assert abs(a.alpha - b.alpha) <= tol, f'segment {k} alpha'
        assert np.allclose(a.p_start, b.p_start, atol=tol), f'segment {k} p_start'
        assert np.allclose(a.p_end, b.p_end, atol=tol), f'segment {k} p_end'
        assert a.n_points == b.n_points, f'segment {k} n_points'
        assert np.allclose(np.asarray(a.cov), np.asarray(b.cov), atol=tol), \
            f'segment {k} cov'


# ---------------------------------------------------------------------------
# the library itself
# ---------------------------------------------------------------------------

@requires_c
def test_the_library_is_the_one_ament_index_points_at():
    assert extraction_c.library_path().endswith('/lib/libextract_lines.so')
    assert extraction_c.version() == '1'


def test_an_absent_library_is_a_fallback_and_a_single_warning(monkeypatch):
    """The documented behaviour when corridor_perception_c is not installed."""
    import corridor_perception.extraction as ex
    monkeypatch.setattr(extraction_c, 'available', lambda: False)
    monkeypatch.setattr(extraction_c, 'load_error', lambda: 'pretend it is absent')
    monkeypatch.setattr(ex, '_C_WARNED', False)
    first = ex.c_backend_warning()
    assert first is not None and 'pretend it is absent' in first
    assert ex.c_backend_warning() is None          # once per process, not per scan
    wall = ((3.0, -1.0), (3.0, 1.0))
    segs = extract_segments(scan_of(wall), ANGLES, ExtractionParams(backend='c'))
    assert segs and segs[0].rho == pytest.approx(3.0, abs=0.02)


@requires_c
def test_the_c_refuses_more_beams_than_its_workspace():
    big = clean_ranges(np.full(4096, 3.0), 0.06, 20.0)
    with pytest.raises(ValueError):
        extraction_c.extract_segments_c(big, np.linspace(-1.0, 1.0, 4096),
                                        ExtractionParams())


@requires_c
def test_an_empty_scan_is_no_segments_on_both():
    empty = clean_ranges(np.zeros(0), 0.06, 20.0)
    assert extract_segments(empty, np.zeros(0), ExtractionParams(backend='c')) == []
    assert extract_segments_python(empty, np.zeros(0), ExtractionParams()) == []


# ---------------------------------------------------------------------------
# the same scenes test_quality_gates.py uses, now on both backends
# ---------------------------------------------------------------------------

@requires_c
def test_a_plain_wall_agrees():
    assert_same(*both(scan_of(((4.0, -1.5), (4.0, 1.5)))))


@requires_c
def test_a_corridor_agrees():
    assert_same(*both(scan_of(((0.0, 1.5), (8.0, 1.5)), ((0.0, -1.5), (8.0, -1.5)),
                              ((8.0, -1.5), (8.0, 1.5)))))


@requires_c
def test_a_person_in_front_of_a_wall_agrees():
    assert_same(*both(scan_of(((3.0, -2.0), (3.0, 2.0)), ((1.5, -0.15), (1.5, 0.15)))))


@requires_c
def test_the_two_merged_faces_agree_and_both_drop_them():
    py, c = both(scan_of(((2.0, -0.80), (2.0, -0.55)), ((2.0, 0.55), (2.0, 0.80))))
    assert_same(py, c)
    assert not [s for s in c[0] if s.length > 1.3]


@requires_c
def test_a_noisy_wall_agrees_through_the_iepf_and_the_merge():
    """2 cm of range noise shatters the wall in the IEPF and the merge
    reunifies it: the path with the most tie-breaks in it."""
    assert_same(*both(scan_of(((2.0, -1.5), (2.0, 1.5)), noise=0.02, seed=7)))


@requires_c
def test_a_cluttered_room_agrees():
    assert_same(*both(scan_of(
        ((0.0, 2.0), (6.0, 2.0)), ((0.0, -2.0), (6.0, -2.0)),
        ((6.0, -2.0), (6.0, 2.0)), ((2.0, -0.4), (2.4, -0.1)),
        ((3.0, 0.5), (3.2, 0.9)), ((1.2, 0.8), (1.5, 0.8)),
        noise=0.01, seed=3)))


@requires_c
@pytest.mark.parametrize('gates', [True, False])
def test_both_agree_with_the_gates_on_and_off(gates):
    assert_same(*both(scan_of(((3.0, -2.0), (3.0, 2.0)), ((1.5, -0.15), (1.5, 0.15)),
                              noise=0.01, seed=5),
                      ExtractionParams(quality_gates=gates)))


@requires_c
@pytest.mark.parametrize('r_med, expected', [(1.0, 3), (3.0, 3), (5.0, 5), (8.0, 6)])
def test_the_seed_gate_agrees_at_every_range(r_med, expected):
    """N(r) decides on the C side too: a wall at r_med whose cluster is just
    under N(r) must be dropped by both."""
    from corridor_perception.extraction import seed_min_count
    assert seed_min_count(r_med, ExtractionParams()) == expected
    assert_same(*both(scan_of(((r_med, -1.0), (r_med, 1.0)))))


@requires_c
def test_an_all_no_return_scan_agrees():
    assert_same(*both(clean_ranges(np.zeros(N_BEAMS), 0.06, 20.0)))


@requires_c
def test_a_glass_corridor_scan_agrees(glass_scans):
    """The five recorded scans, which hold real no-returns and real clutter."""
    for ranges, angles in glass_scans:
        p_py = ExtractionParams(backend='python')
        p_c = ExtractionParams(backend='c')
        as32 = np.asarray(ranges, dtype=np.float32)
        same = clean_ranges(as32.astype(np.float64), -math.inf, math.inf)
        py_stats, c_stats = {}, {}
        py = extract_segments(same, angles, p_py, py_stats)
        c = extract_segments(same, angles, p_c, c_stats)
        assert_same((py, py_stats), (c, c_stats))


# ---------------------------------------------------------------------------
# the stack's merge gap, 0.70 m, at its boundary
# ---------------------------------------------------------------------------

#: The merge gap the stack has run the extraction with since door-detection
#: step 2: f1tenth_perception's multi_wall_tracker.DOOR_WIDTH_MIN_M, deployed
#: as lidar_front_wall_door_width_min_m (f1tenth_perception's tests pin those
#: two to each other). Not ExtractionParams' own 1.5 m, which nothing deployed
#: uses. A retune there leaves these tests valid, just at another number.
STACK_MERGE_GAP = 0.70
CENTRE = (N_BEAMS - 1) // 2


def doorway_scan(gap, k=27, m=102):
    """
    Ranges of a wall square to the centre beam, with a doorway whose edges are ON beams.

    The returns either side stop at beams CENTRE -+ k, and the wall's distance
    puts those two exactly `gap` apart: a raycast doorway's gap runs from the
    last return to the first, up to a beam spacing wider at each side, which
    is useless at a boundary. Each side is m + 1 beams (~1.5 m), long enough
    that the merged wall keeps its fill. Nothing returns through the doorway.
    """
    d = 0.5 * gap / math.tan(k * ANGLE_INC)
    r = np.zeros(N_BEAMS)
    sides = np.r_[CENTRE - k - m:CENTRE - k + 1, CENTRE + k:CENTRE + k + m + 1]
    r[sides] = d / np.cos(ANGLES[sides])
    return clean_ranges(r, 0.06, 20.0)


@pytest.mark.parametrize('backend', ['python', pytest.param('c', marks=requires_c)])
@pytest.mark.parametrize('gap, n_walls', [(0.69, 1), (STACK_MERGE_GAP + 1e-6, 2), (0.71, 2)],
                         ids=['0.69', '0.70', '0.71'])
def test_at_the_stack_merge_gap_0_69_merges_and_0_70_and_0_71_do_not(backend, gap, n_walls):
    """
    A doorway narrower than the merge gap is one wall; one as wide or wider is two.

    The C half must also give exactly the reference's segments. The 0.70
    doorway is built 1 um over the number: a gap fitted from float32 ranges is
    only good to ~3e-8 m (built at exactly 0.70 it measures 0.6999999823 on
    both backends, and merges), so this is the boundary to 40 times the
    measurement's own error. The strict '<' at the exact number is pinned on
    hand-built lines below.
    """
    py, c = both(doorway_scan(gap), ExtractionParams(merge_gap=STACK_MERGE_GAP))
    if backend == 'c':
        assert_same(py, c)
    segs, _stats = py if backend == 'python' else c
    assert len(segs) == n_walls
    assert all(abs(s.alpha) < 1e-3 for s in segs)


@pytest.mark.parametrize('gap, n_clusters', [(STACK_MERGE_GAP - 1e-6, 1), (STACK_MERGE_GAP, 2)],
                         ids=['just_under', 'exactly'])
def test_the_reference_merges_only_strictly_below_merge_gap(gap, n_clusters):
    """
    At exactly merge_gap the two pieces stay apart: '<', not '<='.

    On hand-built lines along x = 3, whose gap is exact in floating point,
    which a fit's is not. The reference only: the C's merge has no entry
    point of its own, so the scan-level test above is its boundary test.
    """
    from corridor_perception.extraction import _merge_clusters
    from corridor_perception.geometry import Line
    half = 0.5 * gap
    a = Line(3.0, 0.0, np.zeros((2, 2)), np.array([3.0, -2.0]), np.array([3.0, -half]), 100)
    b = Line(3.0, 0.0, np.zeros((2, 2)), np.array([3.0, half]), np.array([3.0, 2.0]), 100)
    assert float(b.p_start @ a.tangent - a.p_end @ a.tangent) == gap     # exact
    assert len(_merge_clusters([a, b], ExtractionParams(merge_gap=STACK_MERGE_GAP))) == n_clusters


# ---------------------------------------------------------------------------
# the one thing the C's API cannot express
# ---------------------------------------------------------------------------

def test_a_non_uniform_bearing_grid_goes_to_the_reference():
    """The C takes (angle_min, angle_increment), so it can only ever see a
    uniform grid -- which is what a LaserScan is. A caller that synthesises
    arbitrary bearings must get the reference, not a quietly resampled scan.

    This is the case test_merge_keeps_parallel_offset_surfaces_apart is: it
    builds points, takes their bearings and sorts them.
    """
    pts_a = np.column_stack([np.linspace(0, 2, 60), np.full(60, 1.25)])
    pts_b = np.column_stack([np.linspace(2.3, 4.3, 60), np.full(60, 1.45)])
    pts = np.vstack([pts_a, pts_b])
    r = np.hypot(pts[:, 0], pts[:, 1])
    a = np.arctan2(pts[:, 1], pts[:, 0])
    order = np.argsort(a)
    assert not extraction_c.uniform_angles(a[order])
    ranges = clean_ranges(r[order], 0.02, 30.0)
    segs = extract_segments(ranges, a[order], ExtractionParams(backend='c'))
    assert len(segs) == 2          # the reference's answer, not a resampled one


def test_a_real_scans_grid_is_uniform_to_the_bit():
    assert extraction_c.uniform_angles(ANGLES)
    assert extraction_c.uniform_angles(np.zeros(2))


@requires_c
def test_the_c_refuses_a_grid_it_cannot_rebuild():
    bad = np.concatenate([np.linspace(-1.0, 0.0, 500), np.linspace(0.01, 1.0, 581)])
    with pytest.raises(ValueError, match='uniform grid'):
        extraction_c.extract_segments_c(
            clean_ranges(np.full(bad.size, 3.0), 0.06, 20.0), bad, ExtractionParams())
