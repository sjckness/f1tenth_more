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

STATUS_EMPTY = 0        # neither sensor has anything in this column
STATUS_AGREE = 1        # both, within tolerance
STATUS_LIDAR_ONLY = 2   # a LiDAR return, no stereo WALL pixels
STATUS_STEREO_ONLY = 3  # stereo WALL pixels, no LiDAR return (glass, dark)
STATUS_DISAGREE = 4     # both, outside tolerance

STATUS_NAMES = {
    STATUS_EMPTY: 'empty', STATUS_AGREE: 'agree', STATUS_LIDAR_ONLY: 'lidar_only',
    STATUS_STEREO_ONLY: 'stereo_only', STATUS_DISAGREE: 'disagree',
}


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
                 'col_begin', 'col_end', 'counts')

    def __init__(self, line, z_min, z_max, height_measured, source, confidence,
                 col_begin=0, col_end=0, counts=None):
        self.line = line
        self.z_min, self.z_max = float(z_min), float(z_max)
        self.height_measured = bool(height_measured)
        self.source, self.confidence = int(source), float(confidence)
        self.col_begin, self.col_end = int(col_begin), int(col_end)
        self.counts = counts or {}

    def __repr__(self):   # pragma: no cover - debugging only
        return (f'<Observation {SOURCE_NAMES[self.source]} d={self.line.d:.2f} '
                f'conf={self.confidence:.2f} z={self.z_min:.2f}..{self.z_max:.2f}>')


def confidence(source, inliers, rms, n_agree, n_span, inlier_ref=40, rms_ref=0.03):
    """Confidence [0, 1] of an observation. The model, in one place:

      * evidence     how much the LiDAR actually saw, saturating at inlier_ref
                     returns; 0 for a stereo-only wall, which has none.
      * tightness    how well those returns fit the line, 1 at rms 0 and 0.5
                     at rms_ref.
      * corroboration the fraction of the wall's in-frame columns where the
                     stereo agreed. A wall outside the camera's view has no
                     columns and is NOT punished for it -- it gets the neutral
                     value, because "the camera cannot see there" is not
                     evidence against the LiDAR.

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
        corro = (float(n_agree) / n_span) if n_span > 0 else 0.5
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
                            min_span_columns=8, inlier_ref=40, rms_ref=0.03):
    """LiDAR lines + the column cross-check -> [Observation], and the rejects.

    The emit rule, per the wall's own columns:
      AGREE anywhere          -> SOURCE_BOTH
      no stereo opinion       -> SOURCE_LIDAR (including a wall the camera
                                 cannot see at all)
      DISAGREE over more than
      disagree_reject_frac    -> rejected, counted, not emitted

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
        if n_opinion > 0 and n_dis >= disagree_reject_frac * n_opinion:
            rejected += 1
            continue
        source = SOURCE_BOTH if n_agree > 0 else SOURCE_LIDAR
        lo, hi, measured = _height_from_columns(
            z_min, z_max, c0 if n_agree else 0, c1 if n_agree else 0, default_height)
        out.append(Observation(
            line, lo, hi, measured, source,
            confidence(source, line.inliers, line.rms, n_agree, n_opinion,
                       inlier_ref=inlier_ref, rms_ref=rms_ref),
            col_begin=c0, col_end=c1,
            counts={'agree': n_agree, 'lidar_only': n_lidar,
                    'stereo_only': n_stereo, 'disagree': n_dis}))
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
