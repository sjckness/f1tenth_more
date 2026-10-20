"""
unknown_objects.py: LiDAR obstacles nothing recognised, as "unknown" objects.

Pure: numpy and scipy, no rclpy. semantic_layer_node hosts it
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
  3. Candidates (island_candidates): ISLAND cells linked across gaps up to
     cluster_gap_m (a person's two legs are one object; 8-connected cells
     always are), as connected components of a k-d tree's neighbour graph
     (scipy.spatial / scipy.sparse.csgraph); each component with at least
     min_cells cells becomes one observation: centroid, oriented extent
     (principal axes of its cells, plus one cell) and cell count.

No Python loop runs over cells or points; the loops are over walls (a
handful), objects (a handful) and components (a handful).
"""

from dataclasses import dataclass
import math

import numpy as np
from scipy.sparse import coo_matrix
from scipy.sparse.csgraph import connected_components
from scipy.spatial import cKDTree

from f1tenth_costmap.object_memory import TRACKED
from f1tenth_params.object_geometry import passes_obstacle_filter

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


def island_candidates(view, owner, pose, p):
    """
    Return [Candidate] from the ISLAND cells, nearest the car first.

    Island cells within cluster_gap_m of each other are one component
    (single linkage; 8-connected neighbours always are). Components with
    fewer than min_cells cells, or whose centroid is further than range_m
    from the car, are dropped.
    """
    islands = owner == OWNER_ISLAND
    if not islands.any():
        return []
    rows, cols, xy = view.centres(islands)
    # Single linkage at cluster_gap_m (8-connected neighbours are well inside
    # it): one k-d tree query and a sparse connected-components pass, cost
    # set by the island cells only, never by the grid.
    pairs = cKDTree(xy).query_pairs(p.cluster_gap_m, output_type='ndarray')
    graph = coo_matrix((np.ones(len(pairs)), (pairs[:, 0], pairs[:, 1])),
                       shape=(xy.shape[0], xy.shape[0])) if len(pairs) else \
        coo_matrix((xy.shape[0], xy.shape[0]))
    n, lab = connected_components(graph, directed=False)
    # Per-component moments in one pass (np.bincount), so the cost does not
    # grow with a Python loop per component: centroid, the principal axis of
    # the second moments, and the extents along and across it.
    cnt = np.bincount(lab, minlength=n).astype(float)
    mx = np.bincount(lab, xy[:, 0], n) / cnt
    my = np.bincount(lab, xy[:, 1], n) / cnt
    dx, dy = xy[:, 0] - mx[lab], xy[:, 1] - my[lab]
    sxx = np.bincount(lab, dx * dx, n)
    syy = np.bincount(lab, dy * dy, n)
    sxy = np.bincount(lab, dx * dy, n)
    ang = 0.5 * np.arctan2(2.0 * sxy, sxx - syy)
    along = dx * np.cos(ang[lab]) + dy * np.sin(ang[lab])
    across = -dx * np.sin(ang[lab]) + dy * np.cos(ang[lab])
    a_max = np.full(n, -np.inf)
    a_min = np.full(n, np.inf)
    c_max = np.full(n, -np.inf)
    c_min = np.full(n, np.inf)
    np.maximum.at(a_max, lab, along)
    np.minimum.at(a_min, lab, along)
    np.maximum.at(c_max, lab, across)
    np.minimum.at(c_min, lab, across)
    keep = (cnt >= max(1, int(p.min_cells))) & (
        np.hypot(mx - pose[0], my - pose[1]) <= p.range_m)
    res = view.resolution
    out = [Candidate(float(mx[k]), float(my[k]), float(a_max[k] - a_min[k]) + res,
                     float(c_max[k] - c_min[k]) + res, float(ang[k]), int(cnt[k]))
           for k in np.flatnonzero(keep)]
    out.sort(key=lambda cd: math.hypot(cd.x - pose[0], cd.y - pose[1]))
    return out


def drop_behind_walls(candidates, walls, origin, margin_m=0.10):
    """
    Return the candidates NOT behind a wall as seen from `origin` (map, the laser).

    The tracker's first-obstacle rule, applied to unknown objects: a LiDAR
    beam reports its first return, so a cluster whose line of sight from the
    laser crosses a wall segment more than margin_m before reaching it is
    something seen THROUGH a gap in that wall (a doorway, the room beyond) --
    real, but on the far side of a wall the car cannot drive through, so never
    in its way.
    """
    if not candidates or not walls:
        return list(candidates)
    o = np.asarray(origin, dtype=float)
    c = np.array([[cd.x, cd.y] for cd in candidates])
    d = c - o
    dist = np.hypot(d[:, 0], d[:, 1])
    blocked = np.zeros(len(candidates), dtype=bool)
    for a, b in walls:
        a = np.asarray(a, dtype=float)
        e = np.asarray(b, dtype=float) - a
        q = a - o
        den = d[:, 0] * e[1] - d[:, 1] * e[0]
        with np.errstate(divide='ignore', invalid='ignore'):
            s = (q[0] * e[1] - q[1] * e[0]) / den         # along the sight line, 0..1
            u = (q[0] * d[:, 1] - q[1] * d[:, 0]) / den    # along the wall, 0..1
            near = s * dist < dist - margin_m
        blocked |= (np.abs(den) > 1e-12) & (s > 0.0) & near & (u >= 0.0) & (u <= 1.0)
    return [cd for cd, blk in zip(candidates, blocked) if not blk]


def cells_hit(centre, radius, island_xy, resolution):
    """Tell whether one of this cycle's ISLAND cells (map) lies inside the disk, plus a cell."""
    if island_xy.shape[0] == 0:
        return False
    d = np.hypot(island_xy[:, 0] - centre[0], island_xy[:, 1] - centre[1])
    return bool(np.any(d <= radius + resolution))


def fresh_recognised(tracks, now_sec, max_age_s):
    """
    Return the confirmed recognised tracks a detection updated within max_age_s.

    The recognised lifecycle counts detection BATCHES, so when the camera
    stops (output/avoidance/log.md, A3: depth died 0.8 s into both bags) it
    never ticks and a track lives forever. Such a track is not deleted -- the
    lifecycle is unchanged -- but it no longer owns cells (its LiDAR returns
    become islands, i.e. an unknown object, which is what it now is) and it
    never reaches the MPC.
    """
    return [t for t in tracks if t.confirmed and not t.is_unknown
            and now_sec - t.last_update_stamp_sec <= max_age_s]


def plausible(trk, limits=None):
    """
    Return whether a track may be an obstacle at all: the ONE plausibility gate.

    f1tenth_params.object_geometry.passes_obstacle_filter on a recognised
    track's centre height (z_base) and footprint radius, the test
    obstacle_projector_node applies to a detection and mpc_corr to a
    remembered object; the same function and the same stack_params keys
    (obstacle_z_*, *_obstacle_radius), so live and MEMORY agree. An unknown
    track is LiDAR evidence at the LiDAR's own height: always plausible.
    limits None: no gate.
    """
    if limits is None or trk.is_unknown:
        return True
    z = trk.z_base if getattr(trk, 'z_base', None) is not None else math.nan
    return passes_obstacle_filter(z, trk.radius(), limits)


def current_recognised(tracks, now_sec, max_age_s, memories=None, limits=None):
    """
    Return the confirmed recognised tracks that count as SEEN NOW.

    One rule for both of their privileges, so a track is never half in:
    they own their LiDAR cells (cell_ownership) and they are the recognised
    objects on /costmap/semantic_obstacles (obstacles_for_mpc).

    memories None (object memory off): a detection updated the track within
    max_age_s (fresh_recognised, D7). With the memory on
    ({track_id: object_memory.ObjectMemory}): its state is TRACKED. A MEMORY
    object is remembered, not seen: mpc_corr adds it to its obstacle list
    itself, from /costmap/semantic_objects, grown and decaying (Part B), and
    its LiDAR cluster is absorbed by association, never a second track. A
    track the memory has not looked at yet was confirmed since its last
    cycle, i.e. just detected: current. Either way it must pass the
    plausibility gate (plausible, limits): an object that can never be an
    obstacle owns no cells either.
    """
    if memories is None:
        return [t for t in fresh_recognised(tracks, now_sec, max_age_s)
                if plausible(t, limits)]
    out = []
    for t in tracks:
        if not t.confirmed or t.is_unknown or not plausible(t, limits):
            continue
        mem = memories.get(t.track_id)
        if mem is None or mem.state == TRACKED:
            out.append(t)
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


# ============================================================================
# What the MPC receives (semantic_layer_node, B4/B5)
# ============================================================================

MODES = ('off', 'shadow', 'active')

#: Spacing [m] of the disks an elongated unknown object is drawn as.
DISK_SPACING_M = 0.2


def track_disks(trk, spacing_m=DISK_SPACING_M):
    """
    Return [(x, y, r)] map: the disks a track occupies.

    One disk for a recognised object or a compact unknown cluster. An
    ELONGATED unknown cluster (length more than 1.5 x its width -- the run of
    a cabinet, a wall stub the tracker has not confirmed) is a chain of disks
    of its half-width along its length, spacing_m apart, so it is kept clear
    of where it IS rather than of the disk round its whole length: on
    corridor10m_20261019T131815 a 1.4 m run of cells 1 m to the side became
    one 0.65 m disk reaching into the travel strip (output/obstacles_v2,
    D11).
    """
    if not trk.is_unknown or trk.extent is None:
        return [(trk.x_map, trk.y_map, trk.radius())]
    length, width = trk.extent
    if length <= 1.5 * width:
        return [(trk.x_map, trk.y_map, trk.radius())]
    half = 0.5 * max(width, 0.1)
    reach = 0.5 * length - half
    n = int(math.ceil(2.0 * reach / spacing_m)) + 1
    c, s = math.cos(trk.angle), math.sin(trk.angle)
    return [(trk.x_map + u * c, trk.y_map + u * s, half)
            for u in np.linspace(-reach, reach, n)]


def obstacles_for_mpc(tracks, pose, mode, class_margin=None, now_sec=None,
                      recognised_max_age_s=math.inf, current_ids=None, limits=None):
    """
    Return [(x, y, r)] base_link: the disks of every CONFIRMED track the MPC should avoid.

    Recognised objects that are current: in current_ids when given (the
    node passes current_recognised's, memory-aware), else updated within
    recognised_max_age_s of now_sec (fresh_recognised); unknown objects only
    in mode 'active' (shadow computes and shows them, but keeps them off the
    MPC). r is the track's disk radius plus its class's margin
    (obstacle_class_margin_m, 'unknown' included), the rule
    obstacle_projector_node applies.
    """
    margin = class_margin or {}

    def recognised_ok(t):
        if current_ids is not None:
            return t.track_id in current_ids
        return plausible(t, limits) and (
            now_sec is None or now_sec - t.last_update_stamp_sec <= recognised_max_age_s)
    keep = [t for t in tracks if t.confirmed and (
        (t.is_unknown and mode == 'active') or (not t.is_unknown and recognised_ok(t)))]
    disks = [(x, y, r + margin.get(t.class_id, 0.0)) for t in keep
             for x, y, r in track_disks(t)]
    if not disks:
        return []
    bl = to_base_link([(x, y) for x, y, _r in disks], pose)
    return [(float(p[0]), float(p[1]), float(d[2])) for p, d in zip(bl, disks)]


def speed_cap(tracks, pose, mode, enabled, cap_mps, range_m, front_x, strip_half_m):
    """
    Return the speed cap [m/s] an unknown object close ahead imposes, or -1.0 (none).

    A CONFIRMED unknown object whose disk reaches the travel strip
    (|y| <= strip_half_m) between the bumper and range_m ahead of it. Never in
    shadow or off mode, never when disabled.
    """
    if mode != 'active' or not enabled:
        return -1.0
    disks = [d for t in tracks if t.confirmed and t.is_unknown for d in track_disks(t)]
    if not disks:
        return -1.0
    bl = to_base_link([(x, y) for x, y, _r in disks], pose)
    for (x, y), (_xm, _ym, r) in zip(bl, disks):
        if x + r >= front_x and x - r <= front_x + range_m and abs(y) - r <= strip_half_m:
            return float(cap_mps)
    return -1.0
