"""The extraction's quality gates (extraction.py, WHY THE QUALITY GATES).

Synthetic scans only: every one is built here from geometry, so what each
gate accepts and rejects is stated rather than measured. The UST-10LX's own
sampling (1081 beams over 270 deg) is used so the seed sizes and the fill
ratio are the ones the car actually sees.
"""

import math

import numpy as np
import pytest

from corridor_perception.extraction import (ExtractionParams, extract_segments,
                                            fill_ratio, perp_rms, seed_min_count)
from corridor_perception.scan import clean_ranges

N_BEAMS = 1081
ANGLE_MIN = math.radians(-135.0)
ANGLE_INC = math.radians(270.0) / (N_BEAMS - 1)
ANGLES = ANGLE_MIN + np.arange(N_BEAMS) * ANGLE_INC
NO_RETURN = np.inf


def scan_of(*surfaces):
    """Ranges of a scan holding `surfaces`, each ((x0, y0), (x1, y1)).

    Every beam takes the nearest segment it crosses; a beam that crosses
    nothing is a no-return. Built by intersecting each beam with each segment,
    which is the only way the fill ratio's own geometry is not assumed.
    """
    r = np.full(N_BEAMS, NO_RETURN)
    v = np.column_stack([np.cos(ANGLES), np.sin(ANGLES)])
    for (x0, y0), (x1, y1) in surfaces:
        a = np.array([x0, y0], dtype=float)
        d = np.array([x1 - x0, y1 - y0], dtype=float)
        # a + u d = s v  ->  solve per beam
        den = v[:, 0] * d[1] - v[:, 1] * d[0]
        with np.errstate(divide='ignore', invalid='ignore'):
            u = (v[:, 0] * a[1] - v[:, 1] * a[0]) / den
            s = np.where(np.abs(v[:, 0]) > 1e-9,
                         (a[0] + u * d[0]) / np.where(np.abs(v[:, 0]) > 1e-9, v[:, 0], 1.0),
                         (a[1] + u * d[1]) / np.where(np.abs(v[:, 1]) > 1e-9, v[:, 1], 1.0))
        hit = np.isfinite(u) & (u >= 0.0) & (u <= 1.0) & (s > 0.05)
        r = np.where(hit & (s < r), s, r)
    # Every scan enters the extraction through clean_ranges, bag or synthetic.
    return clean_ranges(np.where(np.isfinite(r), r, 0.0), 0.06, 20.0)


def longest(segs):
    return max(segs, key=lambda s: s.length) if segs else None


# ---------------------------------------------------------------------------
# N(r): the seed size a cluster owes us at its range
# ---------------------------------------------------------------------------

@pytest.mark.parametrize('r_med, expected', [(1.0, 3), (3.0, 3), (5.0, 5), (8.0, 6)])
def test_seed_min_count_at_each_range(r_med, expected):
    assert seed_min_count(r_med, ExtractionParams()) == expected


def test_seed_min_count_is_clipped_both_ways():
    p = ExtractionParams()
    assert seed_min_count(0.1, p) == 3          # never below the 3-point floor
    assert seed_min_count(50.0, p) == 6         # never above the ceiling


# ---------------------------------------------------------------------------
# Fill: a wall owes a return on every unoccluded beam of its own span
# ---------------------------------------------------------------------------

def test_fill_stays_high_with_a_person_in_front_of_the_wall():
    """A person occludes beams; occluded beams are not evidence against a wall."""
    wall = ((3.0, -2.0), (3.0, 2.0))
    person = ((1.5, -0.15), (1.5, 0.15))
    p = ExtractionParams()
    segs = extract_segments(scan_of(wall, person), ANGLES, p)
    seg = longest(segs)
    assert seg is not None, 'the wall itself must survive the gates'
    fill, _n = fill_ratio(seg, scan_of(wall, person), ANGLES, p)
    assert fill >= 0.9


def test_fill_kills_two_faces_merged_across_empty_space():
    """The 1.39 m phantom: two short collinear faces, nothing between them.

    The collinear merge reunifies them (merge_gap 1.5 m) into one segment that
    no surface ever supported across its middle. With the gates off that
    segment is reported; with them on the fill ratio removes it.
    """
    faces = (((2.0, -0.80), (2.0, -0.55)), ((2.0, 0.55), (2.0, 0.80)))
    ranges = scan_of(*faces)
    merged = extract_segments(ranges, ANGLES, ExtractionParams(quality_gates=False))
    assert merged and max(s.length for s in merged) > 1.3, \
        'without the gates the merge produces the phantom this test is about'
    stats = {}
    gated = extract_segments(ranges, ANGLES, ExtractionParams(), stats=stats)
    assert not [s for s in gated if s.length > 1.3]
    assert stats['fill'] >= 1


def test_a_plain_wall_survives_with_the_gates_on():
    wall = ((4.0, -1.5), (4.0, 1.5))
    p = ExtractionParams()
    stats = {}
    segs = extract_segments(scan_of(wall), ANGLES, p, stats=stats)
    seg = longest(segs)
    assert seg is not None and seg.length == pytest.approx(3.0, abs=0.1)
    assert seg.rho == pytest.approx(4.0, abs=0.02)
    assert stats['fill'] == 0 and stats['rms'] == 0


# ---------------------------------------------------------------------------
# RMS: the merge is the one step that can invent a line no cluster supported
# ---------------------------------------------------------------------------

def test_perp_rms_is_the_residual_rms():
    alpha, rho = 0.4, 2.0
    n = np.array([math.cos(alpha), math.sin(alpha)])
    tan = np.array([-n[1], n[0]])
    t = np.linspace(-1.0, 1.0, 11)
    off = np.full(11, 0.05)
    off[::2] = -0.05
    pts = rho * n + t[:, None] * tan + off[:, None] * n
    assert perp_rms(pts, rho, alpha) == pytest.approx(0.05, abs=1e-9)


def test_rms_gate_is_wired_and_measures_the_right_thing():
    """A noisy wall passes at 0.03 m and is dropped when the ceiling is 0.01.

    The gate is deliberately NOT demonstrated through the merge: with
    merge_alpha 6 deg and merge_rho 5 cm checked between the two supports'
    midpoints, the merge cannot refit to a line more than about 5 cm from the
    fragments it joined, so on this extraction the rms ceiling is a backstop
    rather than the gate that fires. The fill ratio above is the one that
    removes merged phantoms. What this test pins is that the ceiling is
    applied, after the merge, to the perpendicular residual.
    """
    rng = np.random.default_rng(7)
    clean = np.asarray(scan_of(((2.0, -1.5), (2.0, 1.5))), dtype=float)
    noisy = clean_ranges(
        np.where(np.isfinite(clean), clean + rng.normal(0.0, 0.02, clean.size), 0.0),
        0.06, 20.0)

    kept = extract_segments(noisy, ANGLES, ExtractionParams(max_rms_m=0.03))
    assert kept and max(seg.length for seg in kept) > 0.9
    stats = {}
    tight = extract_segments(noisy, ANGLES, ExtractionParams(max_rms_m=0.005),
                             stats=stats)
    assert not tight and stats['rms'] >= 1
