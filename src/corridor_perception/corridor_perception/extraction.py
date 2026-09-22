"""Single-scan line extraction: breakpoints, IEPF split, TLS fit, global merge.

Everything here is stateless and in the sensor frame. The pipeline is

  1. adaptive_breakpoints: split the scan where consecutive returns cannot lie
     on one surface (Borges & Aldon). The threshold grows with range and with
     the worst incidence angle we are willing to believe, because a fixed
     euclidean threshold either over-segments far walls or merges near objects.
  2. iepf_split: iterative end-point fit inside each cluster, to separate
     corners.
  3. fit_line_tls: total least squares with a first-order covariance.
  4. a global collinear merge over ALL fitted groups (not only scan-adjacent
     ones): this is what reunifies a wall broken by an occluder or grazed at
     shallow incidence. It is a bucketed union-find in one pass, each surviving
     cluster refitted exactly once. The obvious restart-on-change loop is
     O(n^3) and blows the frame budget in clutter.
  5. a support filter on point count and length.
"""

from __future__ import annotations

import math
from collections import defaultdict
from dataclasses import dataclass

import numpy as np

from .scan import require_clean
from .geometry import Line, normalise_line, wrap_pi

# Groups with fewer returns than this are dropped before fitting: a 2-point
# "line" has no angle information and would only become a merge candidate.
MIN_FIT_POINTS = 3

# Running-mean window, in returns, for the IEPF deviation test; see iepf_split.
IEPF_SMOOTH = 5


@dataclass
class ExtractionParams:
    sigma_0: float = 0.015           # range noise = sigma_0 + sigma_r * range
    sigma_r: float = 0.002
    lambda_abd: float = math.radians(10.0)  # worst incidence angle believed
    abd_sigma_mult: float = 3.0
    split_dist: float = 0.045        # IEPF perpendicular split threshold, m
    merge_alpha: float = math.radians(6.0)
    merge_rho: float = 0.05
    merge_gap: float = 1.50          # max end-to-end gap to consider merging
    min_points: int = 10
    min_length: float = 0.50         # metres
    max_point_gap: float = 0.60


def point_sigma(ranges, p: ExtractionParams) -> np.ndarray:
    return p.sigma_0 + p.sigma_r * np.asarray(ranges, dtype=float)


def polar_to_cartesian(ranges, angles) -> np.ndarray:
    r = np.asarray(ranges, dtype=float)
    a = np.asarray(angles, dtype=float)
    return np.column_stack([r * np.cos(a), r * np.sin(a)])


def adaptive_breakpoints(ranges, angles, p: ExtractionParams) -> list[np.ndarray]:
    """Index arrays of contiguous clusters of valid returns, in scan order.

    d_max = r[i-1] * sin(d_phi) / sin(lambda - d_phi) + k * sigma, clamped to
    max_point_gap. Breaks are forced at invalid or non-finite returns.
    """
    r = np.asarray(ranges, dtype=float)
    a = np.asarray(angles, dtype=float)
    n = r.size
    if n == 0:
        return []
    valid = np.isfinite(r) & (r > 0.0)
    pts = polar_to_cartesian(np.where(valid, r, 0.0), a)

    brk = np.ones(n, dtype=bool)          # brk[i]: break between i-1 and i
    if n > 1:
        dphi = np.abs(np.diff(a))
        rp = np.where(valid[:-1], r[:-1], 0.0)
        den = np.sin(p.lambda_abd - dphi)
        with np.errstate(divide='ignore', invalid='ignore'):
            dmax = rp * np.sin(dphi) / den + p.abd_sigma_mult * point_sigma(rp, p)
        dmax = np.where(den > 0.0, np.minimum(dmax, p.max_point_gap),
                        p.max_point_gap)
        gap = np.linalg.norm(np.diff(pts, axis=0), axis=1)
        brk[1:] = ~(valid[1:] & valid[:-1]) | ~(gap <= dmax)

    starts = np.flatnonzero(brk)
    ends = np.append(starts[1:], n)
    return [np.arange(s, e) for s, e in zip(starts, ends)
            if e - s >= 2 and valid[s]]


def fit_line_tls(pts, sigma, frame: str = 'sensor') -> Line:
    """Total least squares line fit with first-order covariance.

    Covariance is derived in the centroid-anchored frame, where the angle and
    the perpendicular offset e are uncorrelated:
        var(alpha) = sigma^2 / sum(s_i^2),  var(e) = sigma^2 / N
    with s_i the along-line coordinate, then propagated to the origin-anchored
    form through rho = c . n(alpha) + e:
        cov = [[var_e + t_c^2 var_alpha, t_c var_alpha],
               [t_c var_alpha,           var_alpha     ]],   t_c = c . tangent.
    A distant line therefore carries a large rho variance even when the fit is
    tight. That is honest for reporting and WRONG for gating: near where it was
    observed the line is known to ~sigma/sqrt(N). Gate about the observation
    midpoint, not the frame origin.

    sigma is a scalar or per-point array; per-point values enter as their mean
    square.
    """
    pts = np.asarray(pts, dtype=float).reshape(-1, 2)
    n_pts = pts.shape[0]
    if np.ndim(sigma):
        sig = np.asarray(sigma, dtype=float)
        s2 = float(sig @ sig) / max(sig.size, 1)
    else:
        s2 = float(sigma) ** 2
    c = pts.sum(axis=0) / n_pts
    d = pts - c
    (sxx, sxy), (_, syy) = d.T @ d
    alpha = 0.5 * math.atan2(-2.0 * sxy, syy - sxx)
    rho, alpha = normalise_line(c[0] * math.cos(alpha) + c[1] * math.sin(alpha),
                                alpha)
    ca, sa = math.cos(alpha), math.sin(alpha)
    nrm = np.array([ca, sa])
    tan = np.array([-sa, ca])

    # The along-line second moment is the larger eigenvalue of the scatter.
    s_sq = sxx * sa * sa - 2.0 * sxy * sa * ca + syy * ca * ca
    var_alpha = s2 / max(float(s_sq), 1e-12)
    var_e = s2 / n_pts
    t_c = float(c @ tan)
    cov = np.array([[var_e + t_c * t_c * var_alpha, t_c * var_alpha],
                    [t_c * var_alpha, var_alpha]])

    proj = pts @ tan
    p_start = rho * nrm + float(proj.min()) * tan
    p_end = rho * nrm + float(proj.max()) * tan
    return Line(float(rho), alpha, cov, p_start, p_end, n_pts, frame)


def iepf_split(pts, idx, p: ExtractionParams) -> list[np.ndarray]:
    """Iterative end-point fit: split idx until no return is farther than
    split_dist from the chord of its group. Groups share the split point.

    The deviation is measured on a short running mean (IEPF_SMOOTH returns),
    with the chord drawn between the means of the first and last few returns.
    A plain max-deviation test against a chord between two raw returns fires
    on noise: at the default range noise (~1.75 cm at 1.25 m) split_dist is
    only ~2.6 sigma, so among a few hundred returns on a near wall some return
    exceeds it almost every scan. The wall then shatters into 10 cm fragments
    whose angles are uncertain by tens of degrees, and those fragments break
    the merge chain that should reunify it. A real corner is a tent-shaped
    deviation profile, which averaging over a few returns barely lowers.
    """
    pts = np.asarray(pts, dtype=float)
    out: list[np.ndarray] = []
    stack = [np.asarray(idx)]
    while stack:
        g = stack.pop()
        if g.size < 3:
            if g.size >= 2:
                out.append(g)
            continue
        q = pts[g]
        w = min(IEPF_SMOOTH, max(1, g.size // 3))
        a = q[:w].mean(axis=0)
        chord = q[-w:].mean(axis=0) - a
        length = math.hypot(chord[0], chord[1])
        rel = q - a
        if length < 1e-9:
            dist = np.linalg.norm(rel, axis=1)
        else:
            dist = (chord[0] * rel[:, 1] - chord[1] * rel[:, 0]) / length
            if w > 1:
                dist = np.convolve(dist, np.full(w, 1.0 / w), mode='same')
            dist = np.abs(dist)
        k = int(np.argmax(dist))
        if dist[k] > p.split_dist and 0 < k < g.size - 1:
            stack.append(g[k:])
            stack.append(g[:k + 1])
        else:
            out.append(g)
    return out


def _merge_clusters(segs: list[Line], p: ExtractionParams) -> list[list[int]]:
    """One-pass bucketed union-find over collinear, nearby segments.

    Segments are bucketed by quantised alpha, and within a bucket kept sorted
    by rho, so each segment is compared only with its own and the two
    neighbouring alpha buckets (pairs straddling a boundary still meet) over a
    bounded rho window. Mergeable pairs are unioned; the caller refits each
    surviving cluster once.

    A pair merges when all hold:
      * |d_alpha| < merge_alpha;
      * the lines are within merge_rho of each other at the point between
        their supports. Not origin-anchored rho: two fragments of one far wall
        have small angle errors magnified by the tangential lever arm, and
        comparing rho at the origin would keep them apart;
      * the end-to-end gap along the wall is below merge_gap.
    For the same reason the rho window is not a fixed 3-bucket neighbourhood:
    lines that meet at tangential coordinate t differ in origin rho by up to
    merge_rho + |t| * merge_alpha, and the segment with the larger |t| of any
    pair searches wide enough to find the other.
    """
    n = len(segs)
    parent = list(range(n))
    if n < 2:
        return [[i] for i in range(n)]

    def find(i: int) -> int:
        while parent[i] != i:
            parent[i] = parent[parent[i]]
            i = parent[i]
        return i

    alpha = np.array([s.alpha for s in segs])
    rho = np.array([s.rho for s in segs])
    nrm = np.column_stack([np.cos(alpha), np.sin(alpha)])
    tan = np.column_stack([-nrm[:, 1], nrm[:, 0]])
    p0 = np.array([s.p_start for s in segs])
    p1 = np.array([s.p_end for s in segs])
    mid = 0.5 * (p0 + p1)
    reach = 1.05 * (p.merge_rho + np.abs(np.sum(mid * tan, axis=1)) * p.merge_alpha)

    n_alpha = max(1, int(math.floor(2.0 * math.pi / p.merge_alpha + 1e-9)))
    akey = np.floor((alpha + math.pi) / p.merge_alpha).astype(int) % n_alpha
    buckets: dict[int, tuple[np.ndarray, np.ndarray]] = {}
    for k in np.unique(akey):
        members = np.flatnonzero(akey == k)
        order = np.argsort(rho[members])
        buckets[int(k)] = (rho[members][order], members[order])

    for i in range(n):
        cand = []
        for da in (-1, 0, 1):
            b = buckets.get(int((akey[i] + da) % n_alpha))
            if b is None:
                continue
            lo, hi = np.searchsorted(b[0], (rho[i] - reach[i], rho[i] + reach[i]))
            cand.append(b[1][lo:hi])
        j = np.concatenate(cand)
        j = j[j != i]
        if j.size == 0:
            continue
        ok = np.abs(wrap_pi(alpha[j] - alpha[i])) < p.merge_alpha
        m = 0.5 * (mid[i] + mid[j])
        sep = (m @ nrm[i] - rho[i]) - (np.sum(m * nrm[j], axis=1) - rho[j])
        ok &= np.abs(sep) < p.merge_rho
        # Overlap of supports along segment i's tangent; negative is the gap.
        ti = tan[i]
        a0, a1 = sorted((float(p0[i] @ ti), float(p1[i] @ ti)))
        b_s, b_e = p0[j] @ ti, p1[j] @ ti
        gap = np.maximum(np.minimum(b_s, b_e), a0) - np.minimum(np.maximum(b_s, b_e), a1)
        ok &= gap < p.merge_gap
        for jj in j[ok]:
            ri, rj = find(i), find(int(jj))
            if ri != rj:
                parent[rj] = ri

    clusters: dict[int, list[int]] = defaultdict(list)
    for i in range(n):
        clusters[find(i)].append(i)
    return list(clusters.values())


def extract_segments(ranges, angles, p: ExtractionParams | None = None) -> list[Line]:
    """Supported line segments in the sensor frame."""
    p = p or ExtractionParams()
    r = require_clean(ranges, 'extract_segments')
    a = np.asarray(angles, dtype=float)
    valid = np.isfinite(r) & (r > 0.0)
    pts = polar_to_cartesian(np.where(valid, r, 0.0), a)
    sig = point_sigma(np.where(valid, r, 0.0), p)

    groups: list[np.ndarray] = []
    for cluster in adaptive_breakpoints(r, a, p):
        if cluster.size < MIN_FIT_POINTS:
            continue
        groups.extend(g for g in iepf_split(pts, cluster, p)
                      if g.size >= MIN_FIT_POINTS)
    segs = [fit_line_tls(pts[g], sig[g]) for g in groups]

    out: list[Line] = []
    for members in _merge_clusters(segs, p):
        if len(members) == 1:
            line = segs[members[0]]
        else:
            idx = np.unique(np.concatenate([groups[m] for m in members]))
            line = fit_line_tls(pts[idx], sig[idx])
        if line.n_points >= p.min_points and line.length >= p.min_length:
            out.append(line)
    return out
