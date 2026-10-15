"""wall_fusion.py -- per-image-column agreement between the LiDAR and the stereo.

Pure: numpy only, no rclpy. Consumes scene_mask.py's WALL pixels and a /scan
projected into the same image, and says, column by column, whether the two
sensors are looking at the same surface.

WHY A COLUMN. The two sensors fail in opposite directions and the failures are
LOCAL, not global. The LiDAR misses glass and very dark surfaces and sees only
its own scan plane; the stereo misses textureless walls, blows up in sun, and
on this building's glossy tiles reports the wall's reflection below the floor.
A single fused number hides all of that. A column is the finest unit both
sensors can speak about: it is one bearing, which is exactly what a LiDAR beam
is, and a vertical strip of the image, which is what a wall fills.

THE COLUMN SUMMARY IS A LOW PERCENTILE, NOT THE MEDIAN. A LiDAR beam reports
its FIRST return: the nearest surface along that bearing. The honest stereo
counterpart is therefore the near end of that column's WALL depths, not their
middle: in a column holding a near wall low down and a far wall above it (a
doorway, a counter), the median reports a surface that is not there at all,
while a low percentile reports the one the LiDAR also reports. The percentile
rather than the minimum is what keeps a single bad pixel from deciding it.

THE TOLERANCE GROWS WITH THE SQUARE OF RANGE, because stereo depth error does:
sigma_Z = Z^2 * sigma_disparity / (fx * baseline). On this rig (ZED 2i, 0.12 m
baseline, fx 262.4 px at 640x360) half a pixel of disparity error is 1.6 cm at
1 m and 57 cm at 6 m. A fixed tolerance would therefore call every far column
DISAGREE and every near one AGREE, which says more about the tolerance than
about the sensors.

BEAMS OUTSIDE THE CAMERA'S FIELD OF VIEW ARE NOT PENALISED. The LiDAR sees 270
degrees and the camera about 100. A beam the camera cannot possibly see is
LIDAR_ONLY by definition -- it is not evidence of disagreement, and treating it
as such would reject every side wall in the building.
"""

import math

import numpy as np

# scene_mask is the other half of this file's subject and is pure numpy as
# well (its only cv2 use is a local import inside the polygon rasteriser), so
# this does not cost wall_fusion its "numpy only, no rclpy" promise. It is
# imported for the CLASS_* values the rectangle verification reads.
from f1tenth_perception import scene_mask

STATUS_EMPTY = 0        # neither sensor has anything in this column
STATUS_AGREE = 1        # both, within tolerance
STATUS_LIDAR_ONLY = 2   # a LiDAR return, no stereo WALL pixels
STATUS_STEREO_ONLY = 3  # stereo WALL pixels, no LiDAR return (glass, dark)
STATUS_DISAGREE = 4     # both, outside tolerance

STATUS_NAMES = {
    STATUS_EMPTY: 'empty', STATUS_AGREE: 'agree', STATUS_LIDAR_ONLY: 'lidar_only',
    STATUS_STEREO_ONLY: 'stereo_only', STATUS_DISAGREE: 'disagree',
}


def age_weight(age_s, max_age_s, min_weight):
    """How much a profile that old is still worth, linear from 1 to min_weight.

    THE STEREO IS NOT A YES/NO INPUT. Before this it was: a profile paired
    within the old 0.15 s pairing gate counted in full and one a
    millisecond older counted not at all. On corridor10m_20261015T171753 that
    gate passed 44 of 238 live cycles, so 81 % of the run's wall observations
    were LiDAR-only not because the camera had nothing to say but because it
    had said it slightly too early -- and the hard edge meant a 0.14 s profile
    and a 0.01 s one were trusted identically, which is also wrong.

    Age costs accuracy continuously, so the weight is continuous: 1 at zero
    age, min_weight at max_age, linear between, and 0 past it. What it scales
    is in observations_from_lines and observation_covariance -- an old profile
    may still CORROBORATE a LiDAR wall, but it loses most of its power to veto
    one and most of its pull on the fused distance.

    It is NOT a substitute for the ego-motion compensation: the geometry has
    to be carried to the right moment first (the caller does that), and this
    covers what the compensation cannot -- the scene moving, the odometry's own
    error over the interval, and the stereo's own staleness.
    """
    age = abs(float(age_s))
    limit = float(max_age_s)
    if limit <= 0.0:
        return 1.0 if age <= 0.0 else 0.0
    if age >= limit:
        return 0.0
    lo = float(min_weight)
    return 1.0 - (1.0 - lo) * (age / limit)


def tolerance(ranges, base_m, quad_per_m2):
    """Agreement tolerance [m] at `ranges`: base + quad * range^2 (see above)."""
    r = np.asarray(ranges, dtype=float)
    return base_m + quad_per_m2 * np.square(np.where(np.isfinite(r), r, 0.0))


#: Depth quantisation of the column sort, in metres per step. 1 mm is three
#: orders of magnitude finer than the smallest agreement tolerance, and makes
#: the per-column percentile a SINGLE integer sort instead of a two-key
#: lexsort on floats: 40 ms -> 6 ms on a half-filled 640x360 frame.
_DEPTH_STEP_M = 0.001
_DEPTH_BITS = 20   # 1048 m of headroom at 1 mm; depths are capped well below


def _usable(cols, depths, width):
    """(int columns, depths) of the samples that are inside the frame."""
    cols = np.asarray(cols, dtype=float)
    depths = np.asarray(depths, dtype=float)
    ok = np.isfinite(cols) & np.isfinite(depths) & (cols >= -0.5) & (cols < width - 0.5)
    return np.rint(cols[ok]).astype(np.int64), depths[ok]


def column_ranges(cols, depths, width, percentile=20.0, min_px=5):
    """Per-column low percentile of `depths`, NaN where fewer than min_px.

    Vectorised: the (column, depth) pair is packed into one int64 key and
    sorted once, so the runs of a column are contiguous and the percentile is
    an index into them. No Python loop over columns, and no float lexsort.
    """
    width = int(width)
    out = np.full(width, np.nan)
    cols, depths = _usable(cols, depths, width)
    npx = np.bincount(cols, minlength=width).astype(np.int64) if cols.size else \
        np.zeros(width, dtype=np.int64)
    if cols.size == 0:
        return out, npx
    steps = np.clip(np.rint(depths / _DEPTH_STEP_M), 0, (1 << _DEPTH_BITS) - 1)
    keys = np.sort((cols << _DEPTH_BITS) | steps.astype(np.int64))
    starts = np.concatenate([[0], np.cumsum(npx)[:-1]])
    have = npx >= max(1, int(min_px))
    if np.any(have):
        rank = np.floor(np.clip(percentile, 0.0, 100.0) / 100.0
                        * (npx[have] - 1)).astype(np.int64)
        picked = keys[starts[have] + rank] & ((1 << _DEPTH_BITS) - 1)
        out[have] = picked * _DEPTH_STEP_M
    return out, npx


def column_nearest(cols, depths, width):
    """Per-column NEAREST depth, NaN where the column has none.

    The LiDAR's own convention: a beam reports its first return.
    """
    width = int(width)
    out = np.full(width, np.inf)
    cols, depths = _usable(cols, depths, width)
    if cols.size:
        np.minimum.at(out, cols, depths)
    return np.where(np.isfinite(out), out, np.nan)


def column_extent(cols, values, width):
    """Per-column (min, max) of `values`, NaN where the column has none.

    The vertical extent of each column's WALL points, which is what gives an
    observation a real height instead of the default one.
    """
    width = int(width)
    lo = np.full(width, np.inf)
    hi = np.full(width, -np.inf)
    cols, values = _usable(cols, values, width)
    if cols.size:
        np.minimum.at(lo, cols, values)
        np.maximum.at(hi, cols, values)
    return (np.where(np.isfinite(lo), lo, np.nan),
            np.where(np.isfinite(hi), hi, np.nan))


def cross_check(scan_cols, scan_depths, wall_cols, wall_depths, width,
                tol_base_m=0.05, tol_quad_per_m2=0.012, percentile=20.0,
                min_px=5):
    """Per-column agreement between the two sensors.

    `scan_cols`/`scan_depths` are the image column and the optical-axis depth
    of each LiDAR return that falls inside the frame (scene_mask.project_to_
    image gives the columns; beams that miss the frame are the caller's
    business and are LIDAR_ONLY by definition). `wall_cols`/`wall_depths` are
    the same for the WALL pixels.

    Returns a dict of (width,) arrays: status, lidar_m, stereo_m, n_px, tol_m.
    """
    width = int(width)
    lidar_m = column_nearest(scan_cols, scan_depths, width)
    stereo_m, n_px = column_ranges(wall_cols, wall_depths, width,
                                   percentile=percentile, min_px=min_px)
    has_l = np.isfinite(lidar_m)
    has_s = np.isfinite(stereo_m)
    # The tolerance is taken at the LiDAR's range where there is one: it is
    # the better-trusted of the two, and the stereo range is the quantity
    # under test.
    ref = np.where(has_l, lidar_m, np.where(has_s, stereo_m, 0.0))
    tol = tolerance(ref, tol_base_m, tol_quad_per_m2)

    status = np.full(width, STATUS_EMPTY, dtype=np.uint8)
    status[has_l & ~has_s] = STATUS_LIDAR_ONLY
    status[~has_l & has_s] = STATUS_STEREO_ONLY
    both = has_l & has_s
    with np.errstate(invalid='ignore'):
        close = np.abs(stereo_m - lidar_m) <= tol
    status[both & close] = STATUS_AGREE
    status[both & ~close] = STATUS_DISAGREE
    return {'status': status, 'lidar_m': lidar_m, 'stereo_m': stereo_m,
            'n_px': n_px, 'tol_m': tol}


def cross_check_profile(scan_cols, scan_depths, stereo_m, width,
                        tol_base_m=0.05, tol_quad_per_m2=0.012):
    """cross_check against an ALREADY summarised per-column stereo profile.

    StereoColumns.msg carries one distance per column, summarised on the
    camera side where the pixels are (see that message's own rationale), so
    the percentile step has already happened and NaN means "nothing to say".
    """
    width = int(width)
    lidar_m = column_nearest(scan_cols, scan_depths, width)
    stereo_m = np.asarray(stereo_m, dtype=float)
    if stereo_m.size != width:
        padded = np.full(width, np.nan)
        padded[:min(width, stereo_m.size)] = stereo_m[:width]
        stereo_m = padded
    has_l, has_s = np.isfinite(lidar_m), np.isfinite(stereo_m)
    ref = np.where(has_l, lidar_m, np.where(has_s, stereo_m, 0.0))
    tol = tolerance(ref, tol_base_m, tol_quad_per_m2)
    status = np.full(width, STATUS_EMPTY, dtype=np.uint8)
    status[has_l & ~has_s] = STATUS_LIDAR_ONLY
    status[~has_l & has_s] = STATUS_STEREO_ONLY
    both = has_l & has_s
    with np.errstate(invalid='ignore'):
        close = np.abs(stereo_m - lidar_m) <= tol
    status[both & close] = STATUS_AGREE
    status[both & ~close] = STATUS_DISAGREE
    return {'status': status, 'lidar_m': lidar_m, 'stereo_m': stereo_m,
            'n_px': np.zeros(width, dtype=np.int64), 'tol_m': tol}


def status_counts(status):
    """{name: count} of a status array, every status present."""
    s = np.asarray(status)
    return {name: int(np.count_nonzero(s == value)) for value, name in STATUS_NAMES.items()}


def contiguous_spans(mask, min_width=1):
    """[(start, stop)] half-open runs of True in `mask`, at least min_width wide.

    A STEREO_ONLY column on its own is noise; a run of them across the image
    is a pane of glass or a dark surface, which is the only thing that earns a
    stereo-sourced wall.
    """
    m = np.asarray(mask, dtype=bool)
    if m.size == 0:
        return []
    edges = np.diff(np.concatenate([[False], m, [False]]).astype(np.int8))
    starts = np.flatnonzero(edges == 1)
    stops = np.flatnonzero(edges == -1)
    return [(int(a), int(b)) for a, b in zip(starts, stops) if b - a >= min_width]


# ============================================================================
# Per-cycle fusion: LiDAR lines + column statuses -> wall observations
# ============================================================================

SOURCE_LIDAR = 0
SOURCE_STEREO = 1
SOURCE_BOTH = 2
SOURCE_NAMES = {SOURCE_LIDAR: 'lidar', SOURCE_STEREO: 'stereo', SOURCE_BOTH: 'both'}


class Observation:
    """One wall seen in one cycle: a vertical plane, its extent and its source.

    Stateless by construction -- it holds no id, no age and no prediction. The
    tracker is the only thing that remembers anything.
    """

    __slots__ = ('line', 'z_min', 'z_max', 'height_measured', 'source', 'confidence',
                 'col_begin', 'col_end', 'counts', 'stereo_weight')

    def __init__(self, line, z_min, z_max, height_measured, source, confidence,
                 col_begin=0, col_end=0, counts=None, stereo_weight=1.0):
        self.line = line
        self.z_min, self.z_max = float(z_min), float(z_max)
        self.height_measured = bool(height_measured)
        self.source, self.confidence = int(source), float(confidence)
        self.col_begin, self.col_end = int(col_begin), int(col_end)
        self.counts = counts or {}
        #: How much the stereo half of this observation was worth (age_weight).
        #: 1.0 for a LiDAR-only observation, which has no stereo half.
        self.stereo_weight = float(stereo_weight)

    def __repr__(self):   # pragma: no cover - debugging only
        return (f'<Observation {SOURCE_NAMES[self.source]} d={self.line.d:.2f} '
                f'conf={self.confidence:.2f} z={self.z_min:.2f}..{self.z_max:.2f}>')


def confidence(source, inliers, rms, n_agree, n_span, inlier_ref=40, rms_ref=0.03,
               stereo_weight=1.0, rect_support=None):
    """Confidence [0, 1] of an observation. The model, in one place:

      * evidence     how much the LiDAR actually saw, saturating at inlier_ref
                     returns; 0 for a stereo-only wall, which has none.
      * tightness    how well those returns fit the line, 1 at rms 0 and 0.5
                     at rms_ref.
      * corroboration the fraction of the wall's in-frame columns where the
                     stereo agreed, AND -- when the rectangle verification ran
                     over this wall -- its `support`, the weighted fraction of
                     the informative rectangle samples that were at or beyond
                     the line (verify_rectangles). The two are averaged where
                     both exist: they are the same question asked in the
                     LiDAR's own plane and over the wall's whole height, and
                     the second is the one that can tell a wall from a chair
                     standing in front of one. A wall outside the camera's
                     view has neither and is NOT punished for it -- it gets
                     the neutral value, because "the camera cannot see there"
                     is not evidence against the LiDAR.

    SOURCE_STEREO is capped below the others on purpose: a stereo-only wall is
    the one case where nothing cross-checks it, and on this building's floor
    the stereo invents whole surfaces (the reflections below z = 0).
    """
    if source == SOURCE_STEREO:
        base = 0.35
        corro = 1.0
    else:
        evidence = min(1.0, float(inliers) / max(1, inlier_ref))
        tightness = 1.0 / (1.0 + max(0.0, float(rms)) / max(1e-6, rms_ref))
        base = 0.5 * evidence + 0.5 * tightness
        # An old profile's opinion is pulled toward the neutral 0.5 it gets
        # when the camera cannot see the wall at all: it neither vouches for
        # the wall nor argues against it as strongly as a fresh one. That is
        # what "support x age_weight" means here -- the age weight spends the
        # opinion's DISTANCE FROM NEUTRAL, not its magnitude, which is the
        # same convention age_weight's own docstring sets and the only one
        # under which an old profile cannot argue against a wall by default.
        w = min(1.0, max(0.0, float(stereo_weight)))
        terms = []
        if n_span > 0:
            terms.append(0.5 + w * (float(n_agree) / n_span - 0.5))
        if rect_support is not None:
            terms.append(0.5 + w * (float(rect_support) - 0.5))
        corro = sum(terms) / len(terms) if terms else 0.5
    value = base * (0.6 + 0.4 * corro)
    if source == SOURCE_BOTH:
        value = min(1.0, value * 1.15)
    return float(min(1.0, max(0.0, value)))


def line_columns(line, project, width):
    """(col_begin, col_end) half-open image columns a WallLine's extent covers.

    Both endpoints are projected; an endpoint behind the lens (NaN) leaves the
    span open on that side, which is clamped to the frame. (0, 0) means the
    wall never reaches the frame at all.
    """
    (ax, ay), (bx, by) = line.endpoints()
    uv = project(np.array([[ax, ay, 0.0], [bx, by, 0.0]]))
    us = uv[:, 0]
    if not np.any(np.isfinite(us)):
        return 0, 0
    lo, hi = np.nanmin(us), np.nanmax(us)
    if np.isfinite(lo) != np.isfinite(hi):           # one end behind the lens
        lo, hi = (0.0, hi) if np.isfinite(hi) else (lo, float(width))
    lo = int(max(0, math.floor(lo)))
    hi = int(min(int(width), math.ceil(hi) + 1))
    return (lo, hi) if hi > lo else (0, 0)


def _height_from_columns(z_min, z_max, col_begin, col_end, default_height):
    """(z_min, z_max, measured) over a column span, from the stereo profile."""
    if col_end > col_begin:
        lo = np.asarray(z_min, dtype=float)[col_begin:col_end]
        hi = np.asarray(z_max, dtype=float)[col_begin:col_end]
        if np.any(np.isfinite(lo)) and np.any(np.isfinite(hi)):
            return float(np.nanmin(lo)), float(np.nanmax(hi)), True
    return 0.0, float(default_height), False


def observations_from_lines(lines, check, project, width, z_min, z_max,
                            default_height=2.5, disagree_reject_frac=0.5,
                            min_span_columns=8, inlier_ref=40, rms_ref=0.03,
                            stereo_weight=1.0):
    """LiDAR lines + the column cross-check -> [Observation], and the rejects.

    The emit rule, per the wall's own columns:
      AGREE anywhere          -> SOURCE_BOTH
      no stereo opinion       -> SOURCE_LIDAR (including a wall the camera
                                 cannot see at all)
      DISAGREE over more than
      disagree_reject_frac    -> rejected, counted, not emitted

    `stereo_weight` (age_weight) scales what the profile's opinion is worth.
    THE VETO IS WHAT IT MOSTLY SCALES: rejecting a wall the LiDAR fitted is
    the most destructive thing the stereo can do -- it removes the evidence
    entirely rather than adjusting it -- so an old profile needs
    proportionally more disagreement to do it (n_dis * w against the same
    fraction). At w = 1 the rule is exactly the old one. Corroboration is
    pulled toward neutral by the same weight in `confidence`.

    Returns ([Observation], n_rejected).
    """
    status = np.asarray(check['status'])
    out, rejected = [], 0
    for line in lines:
        c0, c1 = line_columns(line, project, width)
        span = status[c0:c1] if c1 > c0 else status[:0]
        n_agree = int(np.count_nonzero(span == STATUS_AGREE))
        n_dis = int(np.count_nonzero(span == STATUS_DISAGREE))
        n_lidar = int(np.count_nonzero(span == STATUS_LIDAR_ONLY))
        n_stereo = int(np.count_nonzero(span == STATUS_STEREO_ONLY))
        n_opinion = n_agree + n_dis
        w = min(1.0, max(0.0, float(stereo_weight)))
        if n_opinion > 0 and n_dis * w >= disagree_reject_frac * n_opinion:
            rejected += 1
            continue
        source = SOURCE_BOTH if n_agree > 0 else SOURCE_LIDAR
        lo, hi, measured = _height_from_columns(
            z_min, z_max, c0 if n_agree else 0, c1 if n_agree else 0, default_height)
        out.append(Observation(
            line, lo, hi, measured, source,
            confidence(source, line.inliers, line.rms, n_agree, n_opinion,
                       inlier_ref=inlier_ref, rms_ref=rms_ref, stereo_weight=w),
            col_begin=c0, col_end=c1,
            counts={'agree': n_agree, 'lidar_only': n_lidar,
                    'stereo_only': n_stereo, 'disagree': n_dis},
            stereo_weight=w))
    return out, rejected


def stereo_only_observations(check, column_points, width, min_span_columns=8,
                             inlier_m=0.06, min_points=40, min_span_m=0.5,
                             default_height=2.5, z_min=None, z_max=None):
    """Walls only the camera sees: glass, and surfaces too dark for the LiDAR.

    Only over a CONTIGUOUS run of at least min_span_columns STEREO_ONLY
    columns. One such column is noise -- a stray pixel, a reflection edge --
    and this is the one path where nothing cross-checks the result, so the
    bar is a shape the LiDAR could not plausibly have missed by accident.

    `column_points` maps a column index to that column's (x, y) points in the
    base frame; the caller supplies it because the points live on the camera
    side. Returns [Observation].
    """
    from f1tenth_perception.scene_labels import WallLine
    from f1tenth_perception.lidar_front_wall import _robust_line
    status = np.asarray(check['status'])
    out = []
    for c0, c1 in contiguous_spans(status == STATUS_STEREO_ONLY, min_span_columns):
        pts = column_points(c0, c1)
        if pts is None or len(pts) < min_points:
            continue
        pts = np.asarray(pts, dtype=float)[:, :2]
        if len(pts) > 400:
            # A fixed sample keeps lidar_front_wall's pair cache to one entry,
            # the same reason scene_labels.fallback_wall samples to 400.
            pts = pts[np.linspace(0, len(pts) - 1, 400).astype(int)]
        fit = _robust_line(pts, inlier_m, 300)
        if fit is None:
            continue
        nx, ny, d, count, rms = fit
        on = np.abs(pts @ np.array([nx, ny]) - d) <= inlier_m
        if np.count_nonzero(on) < 3:
            continue
        t = -ny * pts[on, 0] + nx * pts[on, 1]
        if float(t.max() - t.min()) < min_span_m:
            continue
        line = WallLine(nx, ny, float(d), float(t.min()), float(t.max()),
                        int(count), float(rms), 'stereo')
        lo, hi, measured = _height_from_columns(z_min, z_max, c0, c1, default_height) \
            if z_min is not None else (0.0, float(default_height), False)
        out.append(Observation(
            line, lo, hi, measured, SOURCE_STEREO,
            confidence(SOURCE_STEREO, count, rms, c1 - c0, c1 - c0),
            col_begin=c0, col_end=c1,
            counts={'agree': 0, 'lidar_only': 0, 'stereo_only': int(c1 - c0),
                    'disagree': 0}))
    return out


# ============================================================================
# The rectangle: verifying a wall over its whole HEIGHT, not just its bearing
# ============================================================================
#
# WHY A RECTANGLE AND NOT MORE COLUMNS. The column cross-check above asks one
# question per bearing: "how far is the nearest WALL pixel in this column?".
# That is the LiDAR's own question and it is answered in the LiDAR's own
# plane, which is the whole problem -- a column's answer comes from whatever
# the stereo matched at ANY height in that column, and the classes above and
# below the scan plane are exactly what tells a wall from a chair in front of
# one. A wall is a vertical surface 2 m tall: if it is there, the stereo
# should see WALL at every height, from the floor to above the lens, along its
# whole length. If something is standing in front of it, the stereo sees that
# something NEARER than the line, over part of the rectangle, and the rest of
# the wall is still there. If the LiDAR invented the line, the stereo sees the
# scene BEHIND where the line claims to be, over the whole rectangle.
#
# So the verification samples the wall's own rectangle -- along the segment
# and up it -- and sorts every sample into one of four verdicts:
#
#   at        a WALL pixel at the line's distance. The wall is there.
#   beyond    a WALL pixel FURTHER than the line. Supports the wall as a
#             surface the LiDAR ranged and the stereo saw through: glass, a
#             dark surface, a specular tile. It is still a wall.
#   against   an OBJECT anywhere in the rectangle, or any surface NEARER than
#             the line. Something else is there instead.
#   uninformative  floor, floor_inferred, ceiling, reflection, self, invalid,
#             NaN. The stereo has nothing to say about that sample, and
#             "nothing to say" must never read as "no wall": on this
#             building's floor the stereo matches nothing over large parts of
#             every frame.
#
# THE HEIGHT WEIGHT w(z) PEAKS AT THE LiDAR. A sample at the scan plane is
# where the two sensors are looking at the same thing, so it is worth most; a
# sample at 1.8 m is still evidence but through a different part of the
# extrinsic and the lens, so it is worth 0.2 at the floor of the weight. The
# weight is a Gaussian of 0.4 m about the laser height, floored at 0.2 so no
# sample is ever worth nothing.
#
# COVERAGE GATES EVERYTHING. support and beyond_share are fractions of the
# INFORMATIVE samples, so with 2 % of the rectangle informative they are
# noise. Below coverage_min the verdict does not move the state at all --
# which is the common case for a wall outside the camera's 100 deg, and it
# must leave such a wall UNVERIFIED rather than REJECTED.
#
# THE STATE IS STICKY AND HYSTERETIC, for the same reason the span gate and
# the role are (multi_wall_tracker's own docstrings): one frame of a person
# walking past must not unmake a verified wall, and the cost of the two
# mistakes is not symmetric. support >= support_verify verifies, <=
# support_reject rejects, and in between nothing happens.

VERIFY_UNVERIFIED = 0   # no rectangle verdict yet: lidar_only, and not a doubt
VERIFY_VERIFIED = 1
VERIFY_REJECTED = 2
VERIFY_NAMES = {VERIFY_UNVERIFIED: 'unverified', VERIFY_VERIFIED: 'verified',
                VERIFY_REJECTED: 'rejected'}


class RectParams:
    """The rectangle verification's own numbers. See the section header."""

    __slots__ = ('z_min_lidar', 'z_max_lidar', 'z_step_m', 't_step_min_m', 't_steps_max',
                 'w_sigma_m', 'w_floor', 'tol_base_m', 'tol_frac', 'min_depth_m',
                 'coverage_min', 'support_verify', 'support_reject',
                 'glass_beyond_share', 'glass_at_share')

    def __init__(self, **kwargs):
        self.z_min_lidar = -0.19      # the floor, in the LiDAR's own frame
        self.z_max_lidar = 1.81       # 2.0 m above the floor
        self.z_step_m = 0.05          # 41 rows
        self.t_step_min_m = 0.05      # along the segment
        self.t_steps_max = 100        # ... but never more than this many steps
        self.w_sigma_m = 0.4          # w(z) = max(w_floor, exp(-z^2 / 2 sigma^2))
        self.w_floor = 0.2
        self.tol_base_m = 0.15        # tol = max(base, frac * d_exp)
        self.tol_frac = 0.05
        self.min_depth_m = 0.1        # optical z below this is behind the lens
        self.coverage_min = 0.25
        self.support_verify = 0.5
        self.support_reject = 0.2
        self.glass_beyond_share = 0.7
        self.glass_at_share = 0.2
        for key, value in kwargs.items():
            setattr(self, key, value)


def rect_grid(p=None):
    """(z, w) of the vertical sample rows, in the LiDAR's frame. Precompute once.

    41 rows from -0.19 m (the floor) to +1.81 m every 5 cm, and the height
    weight of each. Both float32 and both constant for the life of the node:
    nothing about them depends on the scan or the wall.
    """
    p = p or RectParams()
    n = int(round((p.z_max_lidar - p.z_min_lidar) / p.z_step_m)) + 1
    z = (p.z_min_lidar + p.z_step_m * np.arange(n)).astype(np.float32)
    w = np.maximum(p.w_floor, np.exp(-np.square(z) / (2.0 * p.w_sigma_m ** 2)))
    return z, w.astype(np.float32)


def rect_samples(segments, grid, laser_z=0.0, p=None):
    """Every segment's rectangle as ONE (N, 3) base-frame array.

    `segments` is [((x0, y0), (x1, y1))] in the base frame. Returns
    (points, segment index, weight): all the rectangles concatenated, so the
    projection and the gather below run once over everything rather than once
    per wall. The only Python loop is over segments.

    Along the segment the step is max(t_step_min_m, length / t_steps_max), so
    a 0.6 m wall is sampled every 5 cm and a 20 m one every 20 cm: the cost
    of a wall does not grow with its length.
    """
    p = p or RectParams()
    z, w = grid
    nz = z.size
    zb = (z + np.float32(laser_z)).astype(np.float32)
    xs, ys, ks = [], [], []
    for k, (a, b) in enumerate(segments):
        ax, ay = float(a[0]), float(a[1])
        bx, by = float(b[0]), float(b[1])
        length = math.hypot(bx - ax, by - ay)
        step = max(p.t_step_min_m, length / max(1, p.t_steps_max))
        nt = max(2, int(math.floor(length / step)) + 1)
        f = np.linspace(0.0, 1.0, nt, dtype=np.float32)
        xs.append(ax + f * np.float32(bx - ax))
        ys.append(ay + f * np.float32(by - ay))
        ks.append(np.full(nt, k, dtype=np.int32))
    if not xs:
        return (np.empty((0, 3), dtype=np.float32), np.empty(0, dtype=np.int32),
                np.empty(0, dtype=np.float32))
    xt, yt, kt = np.concatenate(xs), np.concatenate(ys), np.concatenate(ks)
    pts = np.empty((xt.size * nz, 3), dtype=np.float32)
    pts[:, 0] = np.repeat(xt, nz)
    pts[:, 1] = np.repeat(yt, nz)
    pts[:, 2] = np.tile(zb, xt.size)
    return pts, np.repeat(kt, nz), np.tile(w, xt.size)


def verify_rectangles(segments, labels, depth, fx, fy, cx, cy, rotation, translation,
                      laser_z=0.0, p=None, grid=None, object_mask=None,
                      wall_class=None, object_class=None, depth_scale=1.0):
    """Per-segment (coverage, support, beyond_share) from the class mask and depth.

    `labels` is scene_mask.classify's (H, W) class image and `depth` the
    (H, W) Z-depth it was computed from, in metres -- the ZED's depth IS the
    optical-axis Z (sl::MEASURE::DEPTH, openni_depth_mode false), which is
    what makes d_exp the sample's own optical z and not its range. fx..cy
    must be the intrinsics AT `labels`' resolution; `rotation`/`translation`
    take base-frame points into the camera's optical frame the same way
    scene_mask.project_to_image does.

    `depth_scale` is what a depth SAMPLE must be multiplied by to be metres,
    for an integer depth image: with an integer dtype a 0 is "no measurement"
    and becomes NaN. The scaling is applied to the GATHERED samples, never to
    the image, so a mono16-millimetre frame costs no full-frame copy -- which
    is how it arrives over a topic (front_clearance_node's
    /perception/scene_labels/depth_mm).

    `object_mask`, when given, is the YOLO union mask at the same resolution.
    It is NOT needed: scene_mask.classify already gives OBJECT priority over
    every depth-derived class, so (labels == OBJECT) IS the YOLO mask
    everywhere except where the SELF mask covers it -- and there SELF must
    win, because the car's own housing is not an object in anyone's way.

    Returns [{coverage, support, beyond_share, n_samples, n_in_image, at,
    beyond, against}], one per segment, all floats/ints, never NaN.
    """
    p = p or RectParams()
    if wall_class is None:
        wall_class = scene_mask.CLASS_WALL
    if object_class is None:
        object_class = scene_mask.CLASS_OBJECT
    n_seg = len(segments)
    out = [{'coverage': 0.0, 'support': 0.0, 'beyond_share': 0.0, 'n_samples': 0,
            'n_in_image': 0, 'at': 0.0, 'beyond': 0.0, 'against': 0.0}
           for _ in range(n_seg)]
    if n_seg == 0:
        return out
    grid = grid if grid is not None else rect_grid(p)
    pts, idx, w = rect_samples(segments, grid, laser_z, p)
    if pts.shape[0] == 0:
        return out
    lab = np.asarray(labels)
    dep_img = np.asarray(depth)
    height, width = lab.shape[:2]
    rot = np.asarray(rotation, dtype=np.float32)
    trans = np.asarray(translation, dtype=np.float32).reshape(3)

    # p_opt = R^T (p_base - t), the same convention as
    # scene_mask.project_to_image -- but written out as three elementwise dot
    # products rather than an (N, 3) @ (3, 3) matmul. That matmul is a BLAS
    # call, and an unpinned numpy process on this box gets a 12-thread
    # OpenBLAS pool whose startup cost on a tiny multiply is ~15 ms under
    # load: 16 tracks measured 16 ms through BLAS and 0.6 ms like this.
    dx = pts[:, 0] - trans[0]
    dy = pts[:, 1] - trans[1]
    dz = pts[:, 2] - trans[2]
    ox = dx * rot[0, 0] + dy * rot[1, 0] + dz * rot[2, 0]
    oy = dx * rot[0, 1] + dy * rot[1, 1] + dz * rot[2, 1]
    d_exp = dx * rot[0, 2] + dy * rot[1, 2] + dz * rot[2, 2]
    with np.errstate(divide='ignore', invalid='ignore'):
        u = np.rint(fx * ox / d_exp + cx)
        v = np.rint(fy * oy / d_exp + cy)
    inside = ((d_exp > p.min_depth_m) & (u >= 0.0) & (u < width)
              & (v >= 0.0) & (v < height))
    w_tot = np.bincount(idx, weights=w, minlength=n_seg)
    if not inside.any():
        for k in range(n_seg):
            out[k]['n_samples'] = int(w_tot[k] > 0.0) and int(np.count_nonzero(idx == k))
        return out

    ui = u[inside].astype(np.int32)
    vi = v[inside].astype(np.int32)
    cls = lab[vi, ui]
    raw = dep_img[vi, ui]
    if np.issubdtype(raw.dtype, np.integer):
        dep = raw.astype(np.float32) * np.float32(depth_scale)
        dep[raw == 0] = np.nan
    elif depth_scale != 1.0:
        dep = raw * np.float32(depth_scale)
    else:
        dep = raw
    exp = d_exp[inside]
    wi = w[inside]
    ki = idx[inside]

    tol = np.maximum(p.tol_base_m, p.tol_frac * exp)
    is_wall = cls == wall_class
    if object_mask is None:
        obj = cls == object_class
    else:
        obj = np.asarray(object_mask, dtype=bool)[vi, ui]
    with np.errstate(invalid='ignore'):
        diff = dep - exp                      # NaN propagates; every test below False
        # AGAINST is decided first and wins: with the class mask as the object
        # source the two can never both hold (classify gives OBJECT priority
        # over WALL), but a YOLO mask supplied separately can overlap a WALL
        # pixel, and there the object is the nearer surface and the one that
        # matters. Keeping them exclusive is also what makes at + beyond +
        # against = informative rather than an over-count.
        against = obj | (is_wall & (diff < -tol))
        at = is_wall & ~against & (np.abs(diff) <= tol)
        beyond = is_wall & ~against & (diff > tol)
    informative = at | beyond | against
    supporting = at | beyond

    w_inf = np.bincount(ki, weights=wi * informative, minlength=n_seg)
    w_sup = np.bincount(ki, weights=wi * supporting, minlength=n_seg)
    w_beyond = np.bincount(ki, weights=wi * beyond, minlength=n_seg)
    w_at = np.bincount(ki, weights=wi * at, minlength=n_seg)
    w_against = np.bincount(ki, weights=wi * against, minlength=n_seg)
    n_all = np.bincount(idx, minlength=n_seg)
    n_in = np.bincount(ki, minlength=n_seg)

    for k in range(n_seg):
        out[k] = {
            'coverage': float(w_inf[k] / w_tot[k]) if w_tot[k] > 0.0 else 0.0,
            'support': float(w_sup[k] / w_inf[k]) if w_inf[k] > 0.0 else 0.0,
            'beyond_share': float(w_beyond[k] / w_sup[k]) if w_sup[k] > 0.0 else 0.0,
            'n_samples': int(n_all[k]), 'n_in_image': int(n_in[k]),
            'at': float(w_at[k]), 'beyond': float(w_beyond[k]),
            'against': float(w_against[k])}
    return out


def verify_step(state, glass, verdict, p=None):
    """(state, glass) after one rectangle verdict. Sticky, hysteretic.

    coverage below coverage_min changes nothing -- not the state and not the
    glass flag -- because support is a fraction of the informative samples
    and with a handful of them it is noise. A wall the camera cannot see is
    therefore left exactly as it was, which for a new track is UNVERIFIED,
    i.e. lidar_only, which is not a doubt about the wall.

    GLASS is the one case where the stereo agrees the wall is there and
    reports it FURTHER than the LiDAR does, over almost all of the
    rectangle: the pane is what the LiDAR ranged and the room behind it is
    what the stereo matched. The distance then comes from the LiDAR alone.
    (Note the two glass thresholds as specified overlap: beyond_share > 0.7
    AND 1 - beyond_share < 0.2 means beyond_share > 0.8. Both are kept so the
    rule reads as the pair of statements it is -- mostly beyond, and hardly
    any at -- rather than as one tuned number.)
    """
    p = p or RectParams()
    if verdict['coverage'] < p.coverage_min:
        return int(state), bool(glass)
    support = verdict['support']
    if support >= p.support_verify:
        state = VERIFY_VERIFIED
    elif support <= p.support_reject:
        state = VERIFY_REJECTED
    if state != VERIFY_VERIFIED:
        return int(state), False
    beyond = verdict['beyond_share']
    return int(state), bool(beyond > p.glass_beyond_share
                            and (1.0 - beyond) < p.glass_at_share)


# ============================================================================
# The output contract: WALL / CLEAR / NO_DATA
# ============================================================================

STATE_NO_DATA = 0
STATE_WALL = 1
STATE_CLEAR = 2
STATE_NAMES = {STATE_NO_DATA: 'no_data', STATE_WALL: 'wall', STATE_CLEAR: 'clear'}


def front_wall_output(has_data, d_ray_front, clear_distance_m, min_distance_m=0.01):
    """(state, value) for the front wall distance. The whole contract, in one place.

    THREE STATES, AND THE MIDDLE ONE IS THE POINT:

      WALL     a FRONT track crosses the car's strip; the value is its
               distance from the bumper along the heading.
      CLEAR    the sensors are working and there is NO wall in the way. The
               value is clear_distance_m, a large finite number, NOT a wall at
               that distance. This is the open corridor, and it is the state
               the old pipeline could not express: with nothing in the strip
               the LiDAR branch reported too_few_returns and the stereo plane
               -- the SIDE wall -- became the front wall at 1.1 m
               (corridor_phantom, 2026-10-15). "Nothing ahead" has to be
               sayable, or something will invent an answer.
      NO_DATA  -1.0. The sensors are not telling us: no transform, no scan,
               no odometry to place it with. NOT "clear". The existing hold
               logic applies (check_stop_condition.py, WALL DISTANCE
               FRESHNESS), so the car holds rather than driving on.

    A consumer that cannot tell CLEAR from NO_DATA would have to treat one of
    them as the other, and both choices are wrong: treating NO_DATA as clear
    drives blind, treating CLEAR as no-data stops in an empty corridor.

    Objects never appear here. A wall distance is about WALLS; the generic
    front_clearance safety check is separate and always on.
    """
    if not has_data:
        return STATE_NO_DATA, -1.0
    if d_ray_front is None or not math.isfinite(d_ray_front):
        return STATE_CLEAR, float(clear_distance_m)
    return STATE_WALL, max(float(d_ray_front), float(min_distance_m))
