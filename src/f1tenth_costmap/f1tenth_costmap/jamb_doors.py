"""Door jambs as BLOBS of occupied cells, not wall segments.

WHY. Walls here are often glass: the lidar returns a frame post, a handle, a
speck, and nothing in between, so "the end of a wall segment" does not exist
and a line fitted through a glass wall's returns is noise. A door is two
things the sensor does see: occupied evidence on each side of a gap, and a gap
that is clear with free space (and rays) going through it.

DEFINITION (this module; the scan-jump candidate stage is unchanged and still
proposes the gap):
  - A JAMB is the occupied cells bounding the gap on one side, ANY extent: a
    small blob from a glass-wall frame or the end of a long opaque wall.
    Nothing is fitted and nothing has to be a wall segment.
  - Its POSITION is the occupied cell of that cluster closest to the gap
    centre, i.e. the gap edge itself, taken among the occupied cells within
    FLANK_RADIUS_M of the scan's edge point. That radius is also the gap-edge
    SUPPORT test: no occupied cell inside it means that side has no jamb.
  - Per door track the position is kept at low gain (JAMB_ALPHA) and a door
    is only strong after JAMB_PERSIST_SCANS sightings with BOTH jambs present
    (semantic_doors.DoorTracker).
  - STRIP EVIDENCE: the strip beyond the gap (gap-wide, STRIP_NEAR..FAR_M
    deep, on the side away from the sensor) must be free of occupied cells
    (free fraction f, unknown counts as not blocked: a room nobody drove into
    is a fan of rays over unknown), and enough rays must have gone through
    (r). Each side of the gap collects its own (f, r); the crossing direction
    is the side that passes, and flips only on clearly better (30 pp) evidence
    from the other side.

NO rclpy, NO message types: numpy only, testable on a hand-drawn Window.
"""

import math

import numpy as np

#: Occupied evidence this close [m] to each of the scan's gap-edge points is
#: that side's support, and the nearest such cell to the gap centre is the jamb.
FLANK_RADIUS_M = 0.30
#: Sightings with both jambs present before a door may be strong.
JAMB_PERSIST_SCANS = 5
#: Low-gain filter on the jamb positions (and so on the centre and axis).
JAMB_ALPHA = 0.05
#: The strip beyond the gap: depth range [m] measured from the gap line, and
#: the free fraction it must reach.
STRIP_NEAR_M = 0.30
STRIP_FAR_M = 1.00
STRIP_FREE_MIN = 0.85
#: Rays that went through the gap (beams past the threshold) for strong.
RAYS_MIN = 4
#: How much better [fraction of 1.0: 30 percentage points] the other side's
#: free fraction must be before the crossing direction flips.
SIDE_FLIP_MARGIN = 0.30
#: Smoothing of the per-side evidence.
SIDE_EVIDENCE_ALPHA = 0.30


def jamb_blob(window, edge_xy, centre_xy, radius_m=FLANK_RADIUS_M):
    """(position | None, n_cells): the jamb on the side of `edge_xy`.

    Occupied cells within `radius_m` of the edge point; the one closest to
    `centre_xy` is the jamb. None when there is no occupied cell at all.
    """
    r0, c0 = window.to_cell(edge_xy[0], edge_xy[1])
    k = max(1, int(round(float(radius_m) / window.res)))
    ra, rb = max(0, r0 - k), min(window.shape[0], r0 + k + 1)
    ca, cb = max(0, c0 - k), min(window.shape[1], c0 + k + 1)
    if ra >= rb or ca >= cb:
        return None, 0
    rows, cols = np.nonzero(window.occupied[ra:rb, ca:cb])
    if rows.size == 0:
        return None, 0
    xs, ys = window.to_map(rows + ra, cols + ca)
    near = np.hypot(xs - edge_xy[0], ys - edge_xy[1]) <= float(radius_m)
    if not np.any(near):
        return None, 0
    xs, ys = xs[near], ys[near]
    d = np.hypot(xs - centre_xy[0], ys - centre_xy[1])
    i = int(np.argmin(d))
    return (float(xs[i]), float(ys[i])), int(xs.size)


def jamb_blobs(window, a, b, radius_m=FLANK_RADIUS_M):
    """(ja, jb, na, nb) for the gap whose scan edge points are `a` and `b`."""
    centre = ((a[0] + b[0]) / 2.0, (a[1] + b[1]) / 2.0)
    ja, na = jamb_blob(window, a, centre, radius_m)
    jb, nb = jamb_blob(window, b, centre, radius_m)
    return ja, jb, na, nb


def far_normal(ja, jb, laser):
    """Unit normal of the gap line pointing AWAY from the sensor."""
    ex, ey = jb[0] - ja[0], jb[1] - ja[1]
    length = math.hypot(ex, ey)
    if length < 1e-9:
        return (0.0, 0.0)
    nx, ny = -ey / length, ex / length
    s_laser = (laser[0] - ja[0]) * nx + (laser[1] - ja[1]) * ny
    return (-nx, -ny) if s_laser > 0.0 else (nx, ny)


def strip_free_fraction(window, ja, jb, laser, near_m=STRIP_NEAR_M,
                        far_m=STRIP_FAR_M, inset_m=0.05):
    """(free_fraction, far_normal): how clear the strip beyond the gap is.

    The strip is as wide as the gap (inset a little from the jambs, which are
    themselves occupied), `near_m`..`far_m` deep on the side away from
    `laser`. It is cut into COLUMNS across the gap; a column is free when no
    occupied cell lies on it between near_m and far_m, and the fraction is
    free columns / columns. Columns, not cells: a thin wall or a shelf across
    the strip occupies one row of cells (a few percent) but blocks every
    column, which is what "free beyond" has to mean. A chair leg blocks one
    column. Unknown cells are not obstacles (a room nobody drove into is a ray
    fan over unknown). Samples outside the window are skipped; 0.0 when no
    column had any sample.
    """
    n = far_normal(ja, jb, laser)
    ex, ey = jb[0] - ja[0], jb[1] - ja[1]
    length = math.hypot(ex, ey)
    if length < 1e-9 or (n[0] == 0.0 and n[1] == 0.0):
        return 0.0, n
    ux, uy = ex / length, ey / length
    step = max(window.res, 0.05)
    ts = np.arange(float(inset_m), length - float(inset_m) + 1e-9, step)
    ds = np.arange(float(near_m), float(far_m) + 1e-9, step)
    if ts.size == 0 or ds.size == 0:
        return 0.0, n
    columns = free = 0
    for t in ts:
        sampled = blocked = False
        for d in ds:
            x = ja[0] + ux * t + n[0] * d
            y = ja[1] + uy * t + n[1] * d
            if not window.contains(x, y):
                continue
            sampled = True
            if window.is_occupied(x, y):
                blocked = True
                break
        if sampled:
            columns += 1
            free += 0 if blocked else 1
    if columns == 0:
        return 0.0, n
    return free / float(columns), n


def side_passes(ev, free_min=STRIP_FREE_MIN, rays_min=RAYS_MIN):
    """Does one side's evidence {'f', 'r'} pass? None / missing does not."""
    return (ev is not None and ev['f'] >= float(free_min)
            and ev['r'] >= float(rays_min))
