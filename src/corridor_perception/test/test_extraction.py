import math

import numpy as np
import pytest

from corridor_perception.scan import clean_ranges
from corridor_perception.extraction import (ExtractionParams, adaptive_breakpoints,
                                            extract_segments, fit_line_tls, iepf_split,
                                            polar_to_cartesian)


def _laterals(segs):
    left = [s for s in segs if abs(math.cos(s.alpha)) < 0.2 and math.sin(s.alpha) > 0]
    right = [s for s in segs if abs(math.cos(s.alpha)) < 0.2 and math.sin(s.alpha) < 0]
    return left, right


def test_tls_recovers_exact_line():
    t = np.linspace(-2, 3, 50)
    alpha, rho = 0.7, 3.0
    n = np.array([math.cos(alpha), math.sin(alpha)])
    tan = np.array([-math.sin(alpha), math.cos(alpha)])
    pts = rho * n + t[:, None] * tan
    line = fit_line_tls(pts, 0.01)
    assert line.rho == pytest.approx(rho) and line.alpha == pytest.approx(alpha)
    assert line.length == pytest.approx(5.0)
    assert line.n_points == 50


def test_tls_covariance_matches_monte_carlo():
    rng = np.random.default_rng(3)
    alpha, rho, sigma = 0.3, 6.0, 0.02
    n = np.array([math.cos(alpha), math.sin(alpha)])
    tan = np.array([-math.sin(alpha), math.cos(alpha)])
    base = rho * n + np.linspace(4.0, 6.0, 40)[:, None] * tan   # far along tangent
    fits = [fit_line_tls(base + rng.normal(0, sigma, size=(40, 1)) * n, sigma)
            for _ in range(3000)]
    vals = np.array([[f.rho, f.alpha] for f in fits])
    emp = np.cov(vals.T)
    model = fits[0].cov
    assert emp[0, 0] == pytest.approx(model[0, 0], rel=0.15)
    assert emp[1, 1] == pytest.approx(model[1, 1], rel=0.15)
    assert emp[0, 1] == pytest.approx(model[0, 1], rel=0.2)
    # The lever arm dominates rho variance far along the tangent.
    assert model[0, 0] > 10 * sigma ** 2 / 40




def test_breakpoints_break_at_invalid_returns():
    r = np.array([1.0, 1.0, np.nan, 1.0, 1.0, 0.0, 1.0, 1.0])
    a = np.linspace(-0.01, 0.01, r.size)
    clusters = adaptive_breakpoints(r, a, ExtractionParams())
    assert [list(c) for c in clusters] == [[0, 1], [3, 4], [6, 7]]


def test_iepf_splits_a_corner():
    leg1 = np.column_stack([np.linspace(0, 2, 40), np.zeros(40)])
    leg2 = np.column_stack([np.full(40, 2.0), np.linspace(0.05, 2, 40)])
    pts = np.vstack([leg1, leg2])
    groups = iepf_split(pts, np.arange(len(pts)), ExtractionParams())
    assert len(groups) == 2


def test_iepf_does_not_shatter_a_noisy_near_wall():
    rng = np.random.default_rng(5)
    x = np.linspace(-3, 3, 400)
    pts = np.column_stack([x, 1.25 + rng.normal(0, 0.0175, x.size)])
    assert len(iepf_split(pts, np.arange(x.size), ExtractionParams())) <= 2






def test_merge_keeps_parallel_offset_surfaces_apart():
    # A 20 cm deep recess is not the same surface as the wall beside it.
    pts_a = np.column_stack([np.linspace(0, 2, 60), np.full(60, 1.25)])
    pts_b = np.column_stack([np.linspace(2.3, 4.3, 60), np.full(60, 1.45)])
    pts = np.vstack([pts_a, pts_b])
    r = np.hypot(pts[:, 0], pts[:, 1])
    a = np.arctan2(pts[:, 1], pts[:, 0])
    order = np.argsort(a)
    # Even a synthesised range array declares the limits it is synthesised
    # against; there is no other way into the pipeline.
    segs = extract_segments(clean_ranges(r[order], 0.02, 30.0), a[order])
    assert len(segs) == 2




def test_polar_to_cartesian():
    pts = polar_to_cartesian([1.0, 2.0], [0.0, math.pi / 2])
    assert np.allclose(pts, [[1.0, 0.0], [0.0, 2.0]])


# ---------------------------------------------------------------------------
# Against recorded scans. No ground truth: these assert that the fit agrees
# with the raw data it came from, and with itself from frame to frame.
# ---------------------------------------------------------------------------

def _mid_frame(frames):
    """A frame from the middle of the run, well inside the corridor."""
    return frames[len(frames) // 2]


def test_real_scan_yields_two_antiparallel_laterals(straight_traverse):
    from corridor_perception.geometry import angdiff
    segs = [s for s in extract_segments(*_frame_scan(_mid_frame(straight_traverse)))
            if s.length > 1.0]
    assert len(segs) >= 2, segs
    # The two longest are NOT necessarily the laterals -- an end wall or a long
    # far surface can outrun them. Require an antiparallel PAIR to exist.
    pairs = [(a, b) for i, a in enumerate(segs) for b in segs[i + 1:]
             if abs(abs(angdiff(a.alpha, b.alpha)) - math.pi) < math.radians(15.0)]
    assert pairs, [(round(s.rho, 2), round(math.degrees(s.alpha), 1)) for s in segs]
    widths = [a.rho + b.rho for a, b in pairs]
    assert any(2.0 < w < 4.5 for w in widths), widths


def test_fitted_frontal_agrees_with_the_raw_forward_beam(straight_traverse):
    """The fitted end wall and the raw beam that sees it must agree.

    This is the one place the naive quantity is allowed to appear, and only as
    a cross-check that the fit is anchored in the same data (P1: it is a test
    baseline, never a predicate).
    """
    checked = 0
    for f in straight_traverse[::100]:
        r, a = _frame_scan(f)
        i = int(round((0.0 - a[0]) / (a[1] - a[0])))
        window = r[i - 3:i + 4]
        window = window[np.isfinite(window)]
        if window.size < 4:
            continue
        raw = float(np.median(window))
        ahead = [s for s in extract_segments(r, a)
                 if abs(math.sin(s.alpha)) < 0.25 and math.cos(s.alpha) > 0
                 and abs(s.rho - raw) < 1.0]
        if not ahead:
            continue
        checked += 1
        assert min(abs(s.rho - raw) for s in ahead) < 0.15
    assert checked >= 3, f'only {checked} frames had both a raw beam and a fit'


def test_no_segment_is_fitted_to_a_no_return(straight_traverse):
    """Replay maps the 65.533 m sentinel to inf; if it ever stopped doing so,
    extraction would happily fit a wall 65 m away."""
    for f in straight_traverse[::200]:
        for s in extract_segments(*_frame_scan(f)):
            assert s.rho < 30.0, f'segment at rho={s.rho:.1f} m'


def test_support_filter_drops_short_segments_on_a_real_scan(straight_traverse):
    p = ExtractionParams()
    for s in extract_segments(*_frame_scan(_mid_frame(straight_traverse)), p):
        assert s.n_points >= p.min_points and s.length >= p.min_length


def _frame_scan(frame):
    return frame.ranges, frame.angles


def test_glass_corridor_scans_yield_walls_not_sentinels(glass_scans):
    """The recorded glazed corridor: five real scans, no motion, no poses.

    Only a static assertion is possible here; T5's continuity-across-the-glazed
    span needs a TRAVERSE of that corridor, which this npz is not.
    """
    for ranges, angles in glass_scans:
        segs = extract_segments(ranges, angles)
        assert segs, 'no surface at all in a corridor scan'
        assert max(s.rho for s in segs) < 30.0, 'fitted to a no-return sentinel'
