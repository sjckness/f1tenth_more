"""
Synthetic, seeded inputs for scripts/perf/bench_micro.py.

One indoor scene seen by every sensor at once, so that functions which compare
sensors (stereo against LiDAR, masks against depth, the grid against the scan)
get inputs that agree with each other:

    a 2 m wide corridor along +x, a door gap in its left wall with a room behind
    it, a front wall at x = 8 m, a box and a person standing in the corridor.

Everything here is built from the existing test helpers (the 1081-beam ray
caster of f1tenth_perception/test/test_door_detection.py, the pinhole plane
renderer and the camera mount of test_scene_mask.py), at the sizes the car
publishes: scan 1081 beams over 270 deg, depth and masks 640x360 (ZED HD720 with
pub_downscale_factor 2), occupancy grid 0.05 m.

Nothing in this file is production code and nothing in production imports it.
"""

import array
import math

import numpy as np

from corridor_perception.geometry import Pose2D

import test_door_detection as tdd
import test_scene_mask as tsm

# ---------------------------------------------------------------- the world
HALF = 1.0                       # corridor half width [m]
DOOR = (2.0, 2.9)                # door gap in the left wall, x range [m]
FRONT_X = 8.0                    # front wall [m]
ROOM = (1.0, 4.0, 4.0)           # room behind the door: x range and far wall y
CEILING_Z = 2.6
SEGMENTS = [
    ((-3.0, HALF), (DOOR[0], HALF)), ((DOOR[1], HALF), (12.0, HALF)),
    ((-3.0, -HALF), (12.0, -HALF)),
    ((FRONT_X, -HALF), (FRONT_X, HALF)),
    ((ROOM[0], HALF), (ROOM[0], ROOM[2])), ((ROOM[1], HALF), (ROOM[1], ROOM[2])),
    ((ROOM[0], ROOM[2]), (ROOM[1], ROOM[2])),
]
#: (centre xy, radius, height): a box and a person.
OBJECTS = [((3.2, 0.45), 0.15, 0.40), ((5.0, -0.35), 0.22, 1.70)]
CIRCLES = [(c, r) for c, r, _h in OBJECTS]

# ---------------------------------------------------------------- the sensors
N_BEAMS = tdd.N_BEAMS
ANGLE_MIN = tdd.ANGLE_MIN
ANGLE_INC = tdd.INC
RANGE_MIN = 0.02
RANGE_MAX = 30.0
LASER = tdd.LASER                # base_link -> laser (x, y, yaw)
LASER_Z = 0.19

W, H = tsm.WIDTH, tsm.HEIGHT     # 640 x 360
FX, FY, CX, CY = tsm.FX, tsm.FY, tsm.CX, tsm.CY
ROT, TRANS = tsm.mount_tf()      # left optical frame -> base_link
MAX_DEPTH = 10.0

GRID_RES = 0.05
GRID_ORIGIN = (-4.0, -2.5)
GRID_W, GRID_H = 340, 150        # 17 m x 7.5 m = 51 000 cells (the car's map: 173 x 295)


def car_pose(i, n=8, seed=20261111):
    """The i-th of n fixed car poses in the corridor (x, y, yaw)."""
    rng = np.random.default_rng(seed)
    table = np.column_stack([rng.uniform(0.0, 2.0, n), rng.uniform(-0.1, 0.1, n),
                             rng.uniform(-0.05, 0.05, n)])
    x, y, yaw = table[i % n]
    return float(x), float(y), float(yaw)


def drive_pose(i):
    """Pose of step i of a slow drive down the corridor (0.2 m/s at 10 Hz)."""
    return 0.02 * i, 0.05 * math.sin(0.05 * i), 0.03 * math.sin(0.031 * i)


def scan(pose, seed=0):
    """Return the (1081,) float32 ranges of one scan from `pose`."""
    ranges = tdd.raycast(Pose2D(*pose), SEGMENTS, CIRCLES, noise=0.01, seed=seed)
    return np.asarray(ranges, dtype=np.float32)


def _plane_in_base(normal_w, offset_w, pose):
    x, y, yaw = pose
    c, s = math.cos(yaw), math.sin(yaw)
    nx, ny, nz = normal_w
    n_b = (c * nx + s * ny, -s * nx + c * ny, nz)
    return n_b, offset_w - (nx * x + ny * y)


def depth_and_masks(pose, seed=0):
    """
    Return (depth float32 (H, W) with NaN holes, instance mask uint8 (H, W)).

    Depth is the Z-depth of the nearest surface along each pixel ray: floor,
    ceiling, both corridor walls, the front wall and the two objects (each a
    plane patch facing the camera). Noise grows with range as stereo does
    (sigma = 0.004 z^2); 1.5 % of the pixels are holes; beyond MAX_DEPTH is NaN.
    The mask labels the objects' pixels 1 and 2, as yolo's mono8 label image.
    """
    rng = np.random.default_rng(seed)
    planes = [((0.0, 0.0, 1.0), 0.0), ((0.0, 0.0, 1.0), CEILING_Z),
              ((0.0, 1.0, 0.0), HALF), ((0.0, 1.0, 0.0), -HALF),
              ((1.0, 0.0, 0.0), FRONT_X)]
    depth = np.full((H, W), np.nan, dtype=np.float32)
    for n_w, d_w in planes:
        n_b, d_b = _plane_in_base(n_w, d_w, pose)
        depth = np.fmin(depth, tsm.plane_depth(n_b, d_b, ROT, TRANS))
    mask = np.zeros((H, W), dtype=np.uint8)
    rays = tsm.rays() @ ROT.T
    x, y, yaw = pose
    c, s = math.cos(yaw), math.sin(yaw)
    for label, ((ox, oy), r, height) in enumerate(OBJECTS, start=1):
        n_b, d_b = _plane_in_base((1.0, 0.0, 0.0), ox - r, pose)
        d_obj = tsm.plane_depth(n_b, d_b, ROT, TRANS)
        p_b = rays * d_obj[..., None] + TRANS
        y_w = y + s * p_b[..., 0] + c * p_b[..., 1]
        hit = (np.abs(y_w - oy) <= r) & (p_b[..., 2] >= 0.0) & (p_b[..., 2] <= height)
        hit &= np.isfinite(d_obj) & ~(d_obj >= depth)
        depth = np.where(hit, d_obj, depth)
        mask[hit] = label
    depth = depth + rng.normal(0.0, 1.0, depth.shape).astype(np.float32) * 0.004 * depth ** 2
    depth[rng.random(depth.shape) < 0.015] = np.nan
    depth[~(depth < MAX_DEPTH)] = np.nan
    return depth.astype(np.float32), mask


def occupancy_grid():
    """Return the map as an OccupancyGrid's own `data`: array('b'), row-major."""
    grid = np.full((GRID_H, GRID_W), -1, dtype=np.int8)
    ys = GRID_ORIGIN[1] + (np.arange(GRID_H) + 0.5) * GRID_RES
    xs = GRID_ORIGIN[0] + (np.arange(GRID_W) + 0.5) * GRID_RES
    gx, gy = np.meshgrid(xs, ys)
    grid[(gx > -3.0) & (gx < FRONT_X) & (np.abs(gy) < HALF)] = 0
    grid[(gx > ROOM[0]) & (gx < ROOM[1]) & (gy >= HALF) & (gy < ROOM[2])] = 0

    def cell(px, py):
        return (int((py - GRID_ORIGIN[1]) / GRID_RES), int((px - GRID_ORIGIN[0]) / GRID_RES))

    for (ax, ay), (bx, by) in SEGMENTS:
        n = int(math.hypot(bx - ax, by - ay) / (GRID_RES * 0.5)) + 1
        for t in np.linspace(0.0, 1.0, n):
            r, col = cell(ax + t * (bx - ax), ay + t * (by - ay))
            if 0 <= r < GRID_H and 0 <= col < GRID_W:
                grid[r, col] = 100
    for (ox, oy), r, _h in OBJECTS:
        grid[(gx - ox) ** 2 + (gy - oy) ** 2 <= r * r] = 100
    return array.array('b', grid.ravel().tolist())


def segments_in_base(pose, x_max=7.5):
    """Return the corridor's wall segments in base_link, clipped to ahead of the car."""
    x, y, yaw = pose
    c, s = math.cos(yaw), math.sin(yaw)
    out = []
    for (ax, ay), (bx, by) in SEGMENTS[:4]:
        pts = []
        for px, py in ((ax, ay), (bx, by)):
            dx, dy = px - x, py - y
            pts.append((c * dx + s * dy, -s * dx + c * dy))
        (x0, y0), (x1, y1) = pts
        if max(x0, x1) < 0.5:
            continue
        if abs(x1 - x0) > 1e-6:
            lo, hi = sorted((x0, x1))
            lo, hi = max(lo, 0.5), min(hi, x_max)
            if hi - lo < 0.5:
                continue
            f = lambda xx: y0 + (y1 - y0) * (xx - x0) / (x1 - x0)  # noqa: E731
            out.append(((lo, f(lo)), (hi, f(hi))))
        else:
            out.append(((x0, y0), (x1, y1)))
    return out
