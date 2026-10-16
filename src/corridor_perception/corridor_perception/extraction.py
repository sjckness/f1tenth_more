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
  6. the QUALITY GATES (quality_gates, on by default): a range-dependent seed
     size, a residual-rms ceiling and an occlusion-aware fill ratio. See
     WHY THE QUALITY GATES below.

WHY THE QUALITY GATES
---------------------
Steps 1-5 ask only "do these returns lie on a line?", and in a room full of
furniture the answer is yes far too often: three returns on a chair leg are
perfectly collinear, and the global merge in step 4 will happily glue two
unrelated 20 cm faces 1.4 m apart into one 1.5 m "wall". That is how a 1.39 m
segment sat 2 m ahead of the car for 30 s of an empty corridor
(corridor10m_20261015T171753, track 8). Three gates, each answering a
question the fit itself cannot:

  SEED SIZE, N(r).    How many returns a real surface OWES us at that range.
      The beam pitch is angular, so a 0.5 m wall is ~50 returns at 1 m and
      ~6 at 8 m: a fixed minimum of 3 is a meaningful demand near the car and
      no demand at all far away, where 3 collinear returns are the normal
      output of noise on any scattered pair of objects. N(r) = clip(3 +
      floor((r_med - 2.0) / 1.5), 3, 6) asks for more returns the further out
      the cluster is -- 3 inside 3.5 m, 6 past 8 m -- which is the range
      where the old floor stopped meaning anything.
  RESIDUAL RMS.       How well the returns actually lie on the line AFTER the
      merge. The merge is the one step that can produce a line no cluster
      ever supported: two collinear-to-6-deg fragments a metre apart refit to
      something neither of them was. The TLS covariance already in Line is
      about the ORIGIN, so it grows with the lever arm and cannot be compared
      against a length; the perpendicular residual rms can, and 0.03 m is the
      same number inlier_distance_m uses for "on this wall".
  FILL RATIO.         Whether the surface was THERE across the span it claims.
      A wall fills its own angular span: every beam in that span either lands
      on it or is stopped by something nearer. So the beams in the span are
      counted, those blocked by a nearer return are EXCLUDED (that is what an
      occluder does, and a person standing in front of a wall must not cost
      the wall its span), and the returns the line actually holds are divided
      by the rest. A real wall scores ~1.0 even with a person in front of it;
      two fragments merged across empty space score about half, because the
      beams in between returned nothing from anywhere near the line.

The gates are a switch (quality_gates) rather than a rewrite because they
change what the extraction REPORTS, and the only honest way to size that
change is to run both on the same bag.
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
# With quality_gates on this is only the FLOOR of N(r) (seed_min_count).
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
    # --- quality gates (see WHY THE QUALITY GATES) ------------------------
    quality_gates: bool = True
    seed_base_points: int = MIN_FIT_POINTS   # N(r) at and below seed_range_m
    seed_max_points: int = 6                 # N(r)'s ceiling
    seed_range_m: float = 2.0                # where N(r) starts to grow
    seed_range_step_m: float = 1.5           # one more return per this much range
    max_rms_m: float = 0.03                  # perpendicular residual ceiling
    min_fill: float = 0.6                    # of the span's unoccluded beams
    occlusion_margin_m: float = 0.10         # nearer than this IS an occluder
    min_incidence: float = 0.05              # |n . v| below this has no s_k
    #: 'c' uses corridor_perception_c's libextract_lines.so and falls back to
    #: 'python' with one warning if it is not installed; 'python' is this file.
    #: The two agree to 1e-4 over every scan of a recording by test, which is
    #: the only reason the default is the one that is harder to read.
    backend: str = 'c'


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


def seed_min_count(r_med, p: ExtractionParams) -> int:
    """N(r): returns a cluster at median range r_med must hold to be fitted.

    clip(3 + floor((r_med - 2.0) / 1.5), 3, 6) -- 3 at 1 and 3 m, 5 at 5 m,
    6 at 8 m and beyond. See WHY THE QUALITY GATES.
    """
    r = float(r_med)
    if not math.isfinite(r):
        return int(p.seed_base_points)
    n = p.seed_base_points + math.floor((r - p.seed_range_m) / p.seed_range_step_m)
    return int(min(max(n, p.seed_base_points), p.seed_max_points))


def perp_rms(pts, rho: float, alpha: float) -> float:
    """Perpendicular residual rms [m] of pts about the line (rho, alpha).

    For the TLS fit of those same points this IS sqrt(lambda_min) of their
    scatter covariance -- the smallest eigenvalue of d.T d / N is the mean
    square perpendicular deviation -- computed as the residual directly
    because that costs one dot product and no eigendecomposition.
    """
    q = np.asarray(pts, dtype=float).reshape(-1, 2)
    if q.shape[0] == 0:
        return math.inf
    resid = q @ np.array([math.cos(alpha), math.sin(alpha)]) - float(rho)
    return math.sqrt(float(resid @ resid) / resid.size)


def fill_ratio(line: Line, ranges, angles, p: ExtractionParams) -> tuple[float, int]:
    """(fill, unoccluded beams) over the beams inside the segment's own span.

    s_k = rho / (n . v_k) is where beam k would meet the line's infinite
    extension (corridor_perception's convention is rho >= 0 with the line in
    the +n half-plane, so a beam pointing at it has n . v_k > 0; the sign is
    the opposite of the usual "n faces the sensor" statement of the same
    formula). A beam whose return is more than occlusion_margin_m NEARER than
    that is blocked by something else and is not evidence against the line;
    one at an incidence below min_incidence has no usable s_k and is left out
    of both counts. Everything else is a beam that should have landed on the
    line.
    """
    r = np.asarray(ranges, dtype=float)
    a = np.asarray(angles, dtype=float)
    b0 = math.atan2(line.p_start[1], line.p_start[0])
    b1 = math.atan2(line.p_end[1], line.p_end[0])
    half = 0.5 * abs(wrap_pi(b1 - b0))
    centre = wrap_pi(b0 + 0.5 * wrap_pi(b1 - b0))
    span = np.abs(wrap_pi(a - centre)) <= half
    if not span.any():
        return math.inf, 0
    av = a[span]
    inc = math.cos(line.alpha) * np.cos(av) + math.sin(line.alpha) * np.sin(av)
    usable = inc > p.min_incidence
    if not usable.any():
        return math.inf, 0
    s = line.rho / inc[usable]
    rv = r[span][usable]
    with np.errstate(invalid='ignore'):
        occluded = rv < s - p.occlusion_margin_m
    n_visible = int(np.count_nonzero(~occluded))
    return line.n_points / max(1, n_visible), n_visible


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


#: Set once, the first time the C backend is asked for and missing, so the
#: warning below is logged once per process rather than once per scan.
_C_WARNED = False


def c_backend_warning() -> str | None:
    """The one-time message when backend 'c' is asked for and unavailable.

    Returns the text the first time and None afterwards, so a caller can log
    it at WARN without a throttle and without importing anything.
    """
    global _C_WARNED
    from . import extraction_c
    if extraction_c.available() or _C_WARNED:
        return None
    _C_WARNED = True
    return ('corridor_perception_c is not installed (' +
            str(extraction_c.load_error()) + '); extract_segments falls back '
            'to the Python backend, which is ~12x slower a scan')


def extract_segments(ranges, angles, p: ExtractionParams | None = None,
                     stats: dict | None = None) -> list[Line]:
    """Supported line segments in the sensor frame.

    `stats`, when given, is filled with what each stage dropped:
    seed/rms/fill/support, and `kept`.

    Dispatches on p.backend. The C is the default and this file is the
    REFERENCE: test_extraction_backends.py runs both over a whole recording
    and over every unit test here, so a disagreement is a bug in the C.
    """
    p = p or ExtractionParams()
    if getattr(p, 'backend', 'python') == 'c':
        from . import extraction_c
        # The C can only express a uniform bearing grid, which is what a
        # LaserScan is; anything else -- a test synthesising points, a replay
        # re-deriving bearings from a cloud -- goes to the reference.
        if extraction_c.available() and extraction_c.uniform_angles(angles):
            require_clean(ranges, 'extract_segments')
            return extraction_c.extract_segments_c(ranges, angles, p, stats)
    return extract_segments_python(ranges, angles, p, stats)


def extract_segments_python(ranges, angles, p: ExtractionParams | None = None,
                            stats: dict | None = None) -> list[Line]:
    """extract_segments' reference implementation. See that function."""
    p = p or ExtractionParams()
    r = require_clean(ranges, 'extract_segments')
    a = np.asarray(angles, dtype=float)
    valid = np.isfinite(r) & (r > 0.0)
    pts = polar_to_cartesian(np.where(valid, r, 0.0), a)
    sig = point_sigma(np.where(valid, r, 0.0), p)
    dropped = {'seed': 0, 'rms': 0, 'fill': 0, 'support': 0}

    def seed_floor(g):
        """N(r) for this group, or the fixed minimum with the gates off.

        Short-circuited on the group's own size before the median is taken:
        a group with seed_max_points returns passes N(r) at ANY range and one
        below seed_base_points fails at any, so the median is only ever
        computed for the few sizes where it decides -- and then over at most
        seed_max_points values, with Python's sort rather than numpy, because
        a scan produces hundreds of these groups and np.median's own overhead
        on a 4-element array is most of this gate's cost (7 ms a scan).
        """
        if not p.quality_gates:
            return MIN_FIT_POINTS
        if g.size >= p.seed_max_points or g.size < p.seed_base_points:
            return p.seed_base_points
        vals = sorted(v for v in (float(r[i]) for i in g) if math.isfinite(v))
        if not vals:
            return p.seed_base_points
        return seed_min_count(vals[len(vals) // 2], p)

    groups: list[np.ndarray] = []
    for cluster in adaptive_breakpoints(r, a, p):
        if cluster.size < seed_floor(cluster):
            dropped['seed'] += 1
            continue
        for g in iepf_split(pts, cluster, p):
            if g.size >= seed_floor(g):
                groups.append(g)
            else:
                dropped['seed'] += 1
    segs = [fit_line_tls(pts[g], sig[g]) for g in groups]

    out: list[Line] = []
    for members in _merge_clusters(segs, p):
        if len(members) == 1:
            line, idx = segs[members[0]], groups[members[0]]
        else:
            idx = np.unique(np.concatenate([groups[m] for m in members]))
            line = fit_line_tls(pts[idx], sig[idx])
        # The cheap support filter runs FIRST, and the two gates only on what
        # it passes. All three are conjunctive, so the order cannot change
        # which segments are reported -- only which counter a doubly-failing
        # one lands in. It changes the cost by a factor of six: a scan
        # produces about 60 merged clusters and reports about 9, and
        # fill_ratio on all 60 is 8 ms a scan.
        if line.n_points < p.min_points or line.length < p.min_length:
            dropped['support'] += 1
            continue
        if p.quality_gates:
            if perp_rms(pts[idx], line.rho, line.alpha) > p.max_rms_m:
                dropped['rms'] += 1
                continue
            if fill_ratio(line, r, a, p)[0] < p.min_fill:
                dropped['fill'] += 1
                continue
        out.append(line)
    if stats is not None:
        stats.update(dropped)
        stats['kept'] = len(out)
    return out
