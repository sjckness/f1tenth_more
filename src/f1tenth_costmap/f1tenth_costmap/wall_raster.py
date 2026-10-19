"""wall_raster.py -- confirmed walls as "wall" cells, with the id and role kept.

Pure: numpy only, no rclpy. The ROS glue is semantic_layer_node.

WHY THIS LIVES BESIDE THE SEMANTIC OBJECT LAYER AND NOT IN THE OCCUPANCY GRID.
This package already keeps two distinct sources of truth and combines them only
at render time (semantic_layer.py's own docstring): slam_toolbox's occupancy
grid, which is "something reflected here at some point", and the semantic
layer, which is "a thing of class X is here". A tracked wall is the second
kind: it has an identity that persists across scans, a role relative to the
car, and a provenance. Merging it into the occupancy grid would throw all of
that away and make it indistinguishable from any other reflecting cell.

WHAT A CELL CARRIES. An OccupancyGrid cell is one int8, which cannot hold both
an id and a role, so this returns TWO rasters over the same geometry:

  roles  the published grid. CELL_FREE, or ROLE_VALUE[role] -- so the grid is
         readable on its own and in RViz, and "which kind of wall is this" needs
         no second lookup.
  ids    the track id that claimed each cell, CELL_NO_ID elsewhere. Not an
         OccupancyGrid (ids run past 127); it is what a consumer uses to ask the
         tracker about a cell, and what makes "these cells belong to track 7"
         answerable at all.

THE RASTER IS REBUILT FROM THE CURRENT TRACKS EVERY CYCLE, never accumulated.
That is what makes a deleted or merged track disappear: its cells are simply
not drawn next time. An accumulating layer would need its own deletion logic,
which would be a second lifecycle to keep in step with the tracker's -- and the
one that drifts is the one that leaves a wall on the map after the tracker has
long since decided it was never there.
"""

import math

import numpy as np

CELL_FREE = 0
CELL_NO_ID = -1

#: Grid value per role. Distinct, small, and never 100: these are not
#: occupancy probabilities and must not be read as any.
#: Cell value per wall ROLE. Multiples of ten inside the OccupancyGrid
#: convention's 0..100, so the grid reads on its own in any viewer.
#: 'unknown' (50) is a wall the tracker holds and uses as a reference for
#: NOTHING -- the geometry puts it nowhere near the car's path, or the stereo
#: rectangle rejected it, or it is behind another confirmed wall
#: (multi_wall_tracker, WHAT A ROLE IS). The cells are still a real surface
#: and still occupied; what is NOT true of them is that they are the wall
#: ahead, to the left or to the right. Before this role had a value of its
#: own the .get() below silently filed them as 'back'.
#: 'other' is the same role's name until 2026-10-19 (old bags, old nodes):
#: an alias with the SAME value, so either spelling paints the same cells.
#: Both names are spelled out here rather than imported, because this module
#: is numpy-only (multi_wall_tracker.ROLE_UNKNOWN / LEGACY_ROLE_OTHER).
ROLE_UNKNOWN = 'unknown'
LEGACY_ROLE_OTHER = 'other'
ROLE_VALUE = {'front': 10, 'left': 20, 'right': 30, 'back': 40,
              ROLE_UNKNOWN: 50, LEGACY_ROLE_OTHER: 50}
#: Cell value -> role, under the CURRENT name only: 50 reads back as 'unknown'.
VALUE_ROLE = {v: k for k, v in ROLE_VALUE.items() if k != LEGACY_ROLE_OTHER}


def rasterize_walls(walls, origin_x, origin_y, resolution, width, height,
                    thickness_m=0.06):
    """(roles, ids) rasters of `walls` over a grid, both (height, width).

    Each wall is (id, role, (x0, y0), (x1, y1)) in the GRID's own frame. The
    segment is drawn thickness_m wide -- a wall is a surface, and a
    zero-width line through a cell grid leaves holes wherever it crosses a
    corner, which a consumer asking "is this cell a wall" would read as a gap
    in the wall.

    Where two walls overlap the LOWER id wins, so the raster is deterministic
    and does not flicker between two tracks describing the same surface while
    the tracker is still deciding to merge them.
    """
    width, height = int(width), int(height)
    roles = np.full((height, width), CELL_FREE, dtype=np.int8)
    ids = np.full((height, width), CELL_NO_ID, dtype=np.int32)
    if width <= 0 or height <= 0 or resolution <= 0.0:
        return roles, ids
    # Half the DIAGONAL of a cell, not half its side: a cell counts when its
    # centre is within that of the segment, which is the standard criterion
    # for covering a line without holes. Half a side leaves gaps on a
    # diagonal, and puts an axis-aligned wall exactly on the float boundary --
    # where it painted two columns at x = 1.0 and none at y = -1.2.
    half = max(0.5 * float(thickness_m), 0.5 * math.sqrt(2.0) * float(resolution))
    # Cell CENTRES, the same convention costmap_boundary's own searches use.
    xs = origin_x + (np.arange(width, dtype=np.float64) + 0.5) * resolution
    ys = origin_y + (np.arange(height, dtype=np.float64) + 0.5) * resolution
    gx, gy = np.meshgrid(xs, ys)
    for wall_id, role, p0, p1 in sorted(walls, key=lambda w: int(w[0]), reverse=True):
        mask = _segment_mask(gx, gy, p0, p1, half)
        if not np.any(mask):
            continue
        # A role this module has never heard of is 'unknown', not 'back': it
        # is certainly not a claim about where the wall is.
        roles[mask] = ROLE_VALUE.get(role, ROLE_VALUE[ROLE_UNKNOWN])
        ids[mask] = int(wall_id)
    return roles, ids


def _segment_mask(gx, gy, p0, p1, half):
    """Bool mask of the cells within `half` of the segment p0 -> p1."""
    x0, y0 = float(p0[0]), float(p0[1])
    x1, y1 = float(p1[0]), float(p1[1])
    dx, dy = x1 - x0, y1 - y0
    length2 = dx * dx + dy * dy
    if length2 < 1e-12:
        return (gx - x0) ** 2 + (gy - y0) ** 2 <= half * half
    # Distance to the SEGMENT, not the infinite line: t is clamped, so a wall
    # does not paint cells beyond its own observed extent.
    t = np.clip(((gx - x0) * dx + (gy - y0) * dy) / length2, 0.0, 1.0)
    px, py = x0 + t * dx, y0 + t * dy
    return (gx - px) ** 2 + (gy - py) ** 2 <= half * half


def wall_cells(ids, wall_id):
    """[(row, col)] of the cells claimed by one track id."""
    rows, cols = np.nonzero(np.asarray(ids) == int(wall_id))
    return list(zip(rows.tolist(), cols.tolist()))


def transform_segment(p0, p1, pose_xytheta):
    """A base_link segment moved into the frame `pose_xytheta` is expressed in."""
    x, y, theta = pose_xytheta
    cos_t, sin_t = math.cos(theta), math.sin(theta)

    def to_world(p):
        return (x + cos_t * float(p[0]) - sin_t * float(p[1]),
                y + sin_t * float(p[0]) + cos_t * float(p[1]))
    return to_world(p0), to_world(p1)
