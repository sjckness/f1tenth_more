"""Distance from the car's rectangular footprint to the nearest point -- pure, no ROS.

obstacle_clearance_node.py turns each /scan into base_link points (through
swept_clearance.scan_to_points, the same valid-return filter every /scan
consumer here shares) and calls footprint_clearance() on them. Nothing else
decides the number, and test/test_obstacle_clearance.py checks it against
closed-form cases.

FRAME AND FOOTPRINT
-------------------
base_link, x forward, y left. The footprint is the axis-aligned rectangle

    x in [rear_x, rear_x + length],   |y| <= width / 2

so ``rear_x`` says where the car's tail sits relative to base_link (negative:
behind it). The defaults are swept_corridor.py's body: the URDF chassis mesh
bounds, 0.525 m x 0.272 m with the tail 0.082 m behind the rear axle, which
is where the URDF puts base_link. Not a tape-measure calibration.

WHAT IT RETURNS
---------------
The signed Euclidean distance from the footprint's EDGE to the nearest point:

* outside: the gap, measured to the nearest face or, off a corner, to the
  corner itself (hypot of the two face gaps) -- the rectangle, not a circle
  around it;
* inside: minus the depth to the nearest face, so a negative value means a
  return inside the body outline, i.e. contact at scan height;
* no points at all: +inf (nothing within the sensor's range_max).

This is the convention the test-campaign logger's obstacle_clearance column
uses (robot edge to obstacle, negative = contact). Unlike swept_corridor it
is NOT a distance along the path of travel: an obstacle beside the car counts
as much as one ahead of it.
"""

import math

import numpy as np

__all__ = ['footprint_clearance', 'footprint_signed_distances']


def footprint_signed_distances(points, length, width, rear_x=0.0):
    """Signed distance of every (x, y) point to the footprint's edge, (N,)."""
    if length <= 0.0 or width <= 0.0:
        raise ValueError(f'footprint must have positive size, got {length} x {width}')
    pts = np.asarray(points, dtype=float).reshape(-1, 2)
    half_l = 0.5 * length
    half_w = 0.5 * width
    cx = rear_x + half_l
    # per-axis signed gap: >0 outside that pair of faces, <0 inside
    dx = np.abs(pts[:, 0] - cx) - half_l
    dy = np.abs(pts[:, 1]) - half_w
    outside = np.hypot(np.maximum(dx, 0.0), np.maximum(dy, 0.0))
    inside = np.minimum(np.maximum(dx, dy), 0.0)
    return outside + inside


def footprint_clearance(points, length, width, rear_x=0.0):
    """Smallest signed distance from the footprint to any point; +inf if none."""
    distances = footprint_signed_distances(points, length, width, rear_x)
    if distances.size == 0:
        return math.inf
    return float(distances.min())
