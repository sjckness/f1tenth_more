"""
unknown_objects.py: LiDAR obstacles nothing recognised, as "unknown" objects.

Pure: numpy and scipy.ndimage, no rclpy. semantic_layer_node hosts it
(unknown_objects_mode), on a timer at most unknown_rate_hz, and feeds what it
finds into the SAME object tracker the YOLO detections go through
(semantic_layer.update_unknown_batch). Obstacles v2, 2026-10-19.

PER CYCLE
  1. The derived cell view (cell_view). The latest /scan, every valid return
     outside the car's own footprint (+ self_margin_m), within range_m of the
     car, placed in the MAP frame with the global-EKF pose nearest the scan
     stamp and rasterised on a local grid of resolution_m around the car.
     /slam/map is never read or modified: it updates every 5 s, only while
     the car moves, and holds every map artefact the car has ever seen --
     the persistence and decay below need per-scan evidence.
  2. Ownership (cell_ownership), each occupied cell exactly one of:
       WALL    within wall_tol_m of a CONFIRMED wall segment (/perception/
               walls): the wall rows handle it.
       OBJECT  inside a tracked object's footprint (recognised or unknown)
               grown by object_margin_m: that object is already tracked.
       ISLAND  everything else.
  3. Candidates (island_candidates): ISLAND cells, bridged across gaps up to
     cluster_gap_m (a person's two legs are one object), labelled
     8-connected (scipy.ndimage.label); each component with at least
     min_cells cells becomes one observation: centroid, oriented extent
     (principal axes of its cells, plus one cell) and cell count.

No Python loop runs over cells or points; the loops are over walls (a
handful), objects (a handful) and components (a handful).
"""

from dataclasses import dataclass
import math

import numpy as np
from scipy import ndimage

OWNER_FREE = 0
OWNER_WALL = 1
OWNER_OBJECT = 2
OWNER_ISLAND = 3
OWNER_NAMES = {OWNER_FREE: 'free', OWNER_WALL: 'wall', OWNER_OBJECT: 'object',
               OWNER_ISLAND: 'island'}

#: The class an unrecognised LiDAR obstacle carries through the whole stack
#: (semantic tracks, markers, the MPC's obstacle list). A YOLO detection that
#: associates with it upgrades it in place.
UNKNOWN_CLASS = 'unknown'


@dataclass
class UnknownParams:
    """The unknown-object numbers. See the module docstring."""

    resolution_m: float = 0.05
    range_m: float = 4.0
    wall_tol_m: float = 0.10
    object_margin_m: float = 0.10
    cluster_gap_m: float = 0.20
    min_cells: int = 2
    persist_s: float = 0.5
    decay_s: float = 1.0
    #: An unknown object the LiDAR cannot see at all (occluded, behind the
    #: car) is kept this long; one its rays PASS THROUGH goes after decay_s.
    unobserved_decay_s: float = 3.0
    #: A beam passes through an object when it returns this far beyond the
    #: object's far side, or not at all.
    pass_margin_m: float = 0.15
    self_margin_m: float = 0.02
    front_x_m: float = 0.405
    rear_x_m: float = -0.10
    half_width_m: float = 0.136


@dataclass
class Candidate:
    """One island component: an unknown-object observation, map frame."""

    x: float
    y: float
    #: Full extents [m] along the component's principal axes, one cell added.
    length: float
    width: float
    angle: float
    n_cells: int

    @property
    def radius(self):
        """Half the larger extent: the disk the MPC will keep clear of."""
        return 0.5 * max(self.length, self.width)


def scan_points(ranges, angle_min, angle_increment, range_min, range_max, laser_pose):
    """
    Return (points (N, 2) base_link, beam bearings (M,), ranges (M,)) of one scan.

    `laser_pose` is (x, y, yaw) of the laser in base_link. Points are the
    valid returns only; the bearings/ranges are every beam (inf for none),
    base_link, for rays_pass_through.
    """
    r = np.asarray(ranges, dtype=float)
    lx, ly, lyaw = (float(v) for v in laser_pose)
    bearings = lyaw + angle_min + np.arange(r.size) * angle_increment
    valid = np.isfinite(r) & (r >= range_min) & (r <= range_max)
    rr = np.where(valid, r, np.inf)
    pts = np.column_stack([lx + r[valid] * np.cos(bearings[valid]),
                           ly + r[valid] * np.sin(bearings[valid])])
    return pts, bearings, rr


def drop_self_hits(points_bl, p):
    """Drop base_link points inside the car's footprint grown by self_margin_m."""
    pts = np.asarray(points_bl, dtype=float).reshape(-1, 2)
    m = p.self_margin_m
    inside = ((pts[:, 0] >= p.rear_x_m - m) & (pts[:, 0] <= p.front_x_m + m)
              & (np.abs(pts[:, 1]) <= p.half_width_m + m))
    return pts[~inside]


def to_map(points_bl, pose):
    """Base_link points to the map frame, given the car's (x, y, yaw) in map."""
    x, y, yaw = pose
    c, s = math.cos(yaw), math.sin(yaw)
    pts = np.asarray(points_bl, dtype=float).reshape(-1, 2)
    return np.column_stack([x + c * pts[:, 0] - s * pts[:, 1],
                            y + s * pts[:, 0] + c * pts[:, 1]])


def to_base_link(points_map, pose):
    """Map points to base_link, given the car's (x, y, yaw) in map."""
    x, y, yaw = pose
    c, s = math.cos(yaw), math.sin(yaw)
    d = np.asarray(points_map, dtype=float).reshape(-1, 2) - (x, y)
    return np.column_stack([c * d[:, 0] + s * d[:, 1], -s * d[:, 0] + c * d[:, 1]])


class CellView:
    """One cycle's local occupied-cell grid, map frame, around the car."""

    def __init__(self, occupied, origin, resolution):
        self.occupied = occupied          # (n, n) bool
        self.origin = origin              # (x, y) map of cell (0, 0)'s corner
        self.resolution = float(resolution)

    def centres(self, mask=None):
        """Return (rows, cols, (K, 2) map centres) of the cells in mask (default occupied)."""
        rows, cols = np.nonzero(self.occupied if mask is None else mask)
        xy = np.column_stack([self.origin[0] + (cols + 0.5) * self.resolution,
                              self.origin[1] + (rows + 0.5) * self.resolution])
        return rows, cols, xy


def cell_view(points_bl, pose, p):
    """
    Rasterise base_link scan points around the car into a map-frame CellView.

    Self-hits are dropped, and so is everything further than range_m from
    base_link (the grid is 2 * (range_m + 1) wide, centred on the car).
    """
    pts = drop_self_hits(points_bl, p)
    pts = pts[np.hypot(pts[:, 0], pts[:, 1]) <= p.range_m]
    half = p.range_m + 1.0
    n = int(math.ceil(2.0 * half / p.resolution_m))
    origin = (pose[0] - half, pose[1] - half)
    occ = np.zeros((n, n), dtype=bool)
    if pts.size:
        xy = to_map(pts, pose)
        cols = np.floor((xy[:, 0] - origin[0]) / p.resolution_m).astype(int)
        rows = np.floor((xy[:, 1] - origin[1]) / p.resolution_m).astype(int)
        ok = (cols >= 0) & (cols < n) & (rows >= 0) & (rows < n)
        occ[rows[ok], cols[ok]] = True
    return CellView(occ, origin, p.resolution_m)


def _segment_distance(xy, a, b):
    """Distance from each of the (K, 2) points to the segment a-b."""
    a = np.asarray(a, dtype=float)
    ab = np.asarray(b, dtype=float) - a
    den = float(ab @ ab)
    if den < 1e-12:
        return np.hypot(xy[:, 0] - a[0], xy[:, 1] - a[1])
    t = np.clip(((xy - a) @ ab) / den, 0.0, 1.0)
    proj = a + t[:, None] * ab
    return np.hypot(xy[:, 0] - proj[:, 0], xy[:, 1] - proj[:, 1])


def cell_ownership(view, walls, objects, p):
    """
    Return the (n, n) int8 owner grid: OWNER_FREE / WALL / OBJECT / ISLAND.

    `walls` is [((x0, y0), (x1, y1))] of CONFIRMED wall segments, map frame;
    `objects` is [(x, y, radius)] of tracked objects (recognised or unknown),
    map frame. A cell a wall owns is the wall's even if an object is near it.
    """
    owner = np.zeros(view.occupied.shape, dtype=np.int8)
    rows, cols, xy = view.centres()
    if xy.shape[0] == 0:
        return owner
    label = np.full(xy.shape[0], OWNER_ISLAND, dtype=np.int8)
    if objects:
        obj = np.asarray(objects, dtype=float).reshape(-1, 3)
        d = np.hypot(xy[:, None, 0] - obj[None, :, 0], xy[:, None, 1] - obj[None, :, 1])
        hit = np.any(d <= obj[None, :, 2] + p.object_margin_m, axis=1)
        label[hit] = OWNER_OBJECT
    if walls:
        near = np.zeros(xy.shape[0], dtype=bool)
        for a, b in walls:
            near |= _segment_distance(xy, a, b) <= p.wall_tol_m
        label[near] = OWNER_WALL
    owner[rows, cols] = label
    return owner


def _disk(radius_cells):
    r = max(0, int(radius_cells))
    yy, xx = np.mgrid[-r:r + 1, -r:r + 1]
    return (xx * xx + yy * yy) <= r * r


def island_candidates(view, owner, pose, p):
    """
    Return [Candidate] from the ISLAND cells, nearest the car first.

    Islands closer than cluster_gap_m are bridged (a dilation by half the
    gap before 8-connected labelling); each component's cells are the
    ORIGINAL island cells inside it. Components with fewer than min_cells
    cells, or whose centroid is further than range_m from the car, are
    dropped.
    """
    islands = owner == OWNER_ISLAND
    if not islands.any():
        return []
    bridge = int(round(0.5 * p.cluster_gap_m / view.resolution))
    grown = ndimage.binary_dilation(islands, structure=_disk(bridge)) if bridge > 0 \
        else islands
    labels, n = ndimage.label(grown, structure=np.ones((3, 3), dtype=bool))
    labels = np.where(islands, labels, 0)
    rows, cols, xy = view.centres(islands)
    lab = labels[rows, cols]
    counts = np.bincount(lab, minlength=n + 1)
    out = []
    for k in np.flatnonzero(counts >= max(1, int(p.min_cells))):
        if k == 0:
            continue
        pts = xy[lab == k]
        c = pts.mean(axis=0)
        if math.hypot(c[0] - pose[0], c[1] - pose[1]) > p.range_m:
            continue
        if pts.shape[0] >= 2:
            w, v = np.linalg.eigh(np.cov((pts - c).T, bias=True))
            major = v[:, 1]
            along = (pts - c) @ major
            across = (pts - c) @ np.array([-major[1], major[0]])
            length = float(along.max() - along.min()) + view.resolution
            width = float(across.max() - across.min()) + view.resolution
            angle = math.atan2(major[1], major[0])
        else:
            length = width = view.resolution
            angle = 0.0
        out.append(Candidate(float(c[0]), float(c[1]), length, width, angle,
                             int(pts.shape[0])))
    out.sort(key=lambda cd: math.hypot(cd.x - pose[0], cd.y - pose[1]))
    return out


def rays_pass_through(centre_bl, radius, bearings, ranges, laser_xy, p):
    """
    Return (observed, passed) for a disk at base_link centre_bl.

    observed: at least one beam points at the disk. passed: every beam that
    points at it returns more than pass_margin_m beyond its far side, or not
    at all -- the LiDAR sees THROUGH where the object was.
    """
    lx, ly = laser_xy
    dx, dy = float(centre_bl[0]) - lx, float(centre_bl[1]) - ly
    dist = math.hypot(dx, dy)
    if dist <= radius:
        return False, False
    half = math.asin(min(1.0, radius / dist))
    ang = np.angle(np.exp(1j * (np.asarray(bearings, dtype=float) - math.atan2(dy, dx))))
    beam = np.abs(ang) <= half
    if not beam.any():
        return False, False
    r = np.asarray(ranges, dtype=float)[beam]
    passed = bool(np.all(~np.isfinite(r) | (r > dist + radius + p.pass_margin_m)))
    return True, passed
