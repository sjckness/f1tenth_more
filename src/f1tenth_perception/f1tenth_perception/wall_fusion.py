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
