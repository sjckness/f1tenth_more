"""scene_labels.py -- GROUND / WALL / CEILING / IGNORE / OTHER for the front scene.

Pure: numpy (+ the line fit of lidar_front_wall.py and the floor fit of
ground_plane.py), no rclpy. The ROS glue lives in three nodes, all behind the
one stack switch enable_scene_labels:

  lidar_front_wall_node  the LiDAR half. Wall lines are fitted on the forward
                         /scan with every YOLO detection's bearing sector cut
                         out first (detection_sectors / scan_points), a wall
                         needs a span >= min_span_m (wall_span), and the lines
                         go out on /perception/wall_planes. The same node
                         fuses its LiDAR estimate with the stereo wall
                         distance into the virtual wall (VirtualWall).
  front_clearance_node   the stereo half. Labels a downsampled ZED cloud
                         (label_points) against the floor (fit_ground), the
                         ceiling (CeilingTracker) and the LiDAR wall planes,
                         and measures the stereo wall distance on WALL pixels
                         only (stereo_wall_distance).
  detection_3d_node      measures each detection on its in-mask OTHER pixels.

WHY. M02_approach_wall P002-R005 (2026-10-13): a person stood between the car
and the wall. The LiDAR fit was rejected for 6.5 s (the person's beams made the
wall a minority of the sector: low_inlier_fraction), the stereo background read
the floor and the person's edges at ~2.9 m against a wall at ~5.6 m, and the
mission's 2.0 m wall stop fired with the wall still ~4.1 m from the bumper.
Structure (floor, ceiling, walls) and objects have to be told apart BEFORE any
distance is taken from either.

FRAMES. Everything is base_link: x forward, y left, z up. A wall is a vertical
plane, i.e. a line n . (x, y) = d in the ground plane, with n the unit normal
pointing from the base_link origin toward the wall (d >= 0), exactly the
convention of lidar_front_wall.py. Distances published "from the front" are
from the bumper point (front_x, 0).
"""

import math
from dataclasses import dataclass

import numpy as np

from f1tenth_perception.ground_plane import GroundPlane
from f1tenth_perception.lidar_front_wall import (
    PROVENANCE_DEAD_RECKONED, PROVENANCE_MEASURED, _robust_line)

LABEL_UNKNOWN = 0   # no valid depth at this sample
LABEL_GROUND = 1
LABEL_WALL = 2
LABEL_CEILING = 3
LABEL_IGNORE = 4
LABEL_OTHER = 5
LABEL_NAMES = {
    LABEL_UNKNOWN: 'unknown', LABEL_GROUND: 'ground', LABEL_WALL: 'wall',
    LABEL_CEILING: 'ceiling', LABEL_IGNORE: 'ignore', LABEL_OTHER: 'other',
}


# ============================================================================
# Walls (LiDAR primary)
# ============================================================================

@dataclass(frozen=True)
class WallLine:
    """A vertical wall plane n . (x, y) = d in base_link, with its extent.

    t0 <= t1 are the inliers' coordinates along the line direction (-ny, nx);
    the span is t1 - t0 and is what separates a wall from an object face.
    """

    nx: float
    ny: float
    d: float
    t0: float
    t1: float
    inliers: int
    rms: float
    source: str = 'lidar'

    @property
    def span(self):
        return self.t1 - self.t0

    @property
    def normal_angle(self):
        return math.atan2(self.ny, self.nx)

    def offset(self, x, y):
        """Signed distance [m] from point(s) to the plane, + = toward the car."""
        return self.d - (self.nx * np.asarray(x) + self.ny * np.asarray(y))

    def along(self, x, y):
        return -self.ny * np.asarray(x) + self.nx * np.asarray(y)

    def distance_from(self, x, y=0.0):
        """Perpendicular distance [m] from point (x, y) to the plane."""
        return float(self.d - (self.nx * x + self.ny * y))

    def endpoints(self):
        """((x0, y0), (x1, y1)) of the fitted extent, base_link."""
        px, py = self.nx * self.d, self.ny * self.d
        tx, ty = -self.ny, self.nx
        return ((px + tx * self.t0, py + ty * self.t0),
                (px + tx * self.t1, py + ty * self.t1))

    def to_dict(self):
        return {'nx': round(self.nx, 5), 'ny': round(self.ny, 5), 'd': round(self.d, 4),
                't0': round(self.t0, 3), 't1': round(self.t1, 3), 'inliers': int(self.inliers),
                'rms': round(self.rms, 4), 'source': self.source}

    @staticmethod
    def from_dict(d):
        return WallLine(float(d['nx']), float(d['ny']), float(d['d']), float(d['t0']),
                        float(d['t1']), int(d.get('inliers', 0)), float(d.get('rms', 0.0)),
                        str(d.get('source', 'lidar')))


def detection_sectors(objects_xyr, origin_xy, margin_rad):
    """Bearing sectors [(lo, hi)] [rad, base_link] hidden behind detected objects.

    objects_xyr: (x, y, half_width) of each detection in base_link. Bearings are
    seen from origin_xy, the laser's position: the beams leave from there. The
    half-angle is the object's own angular half-width plus margin_rad.
    """
    sectors = []
    ox, oy = origin_xy
    for x, y, r in objects_xyr:
        dx, dy = float(x) - ox, float(y) - oy
        rng = math.hypot(dx, dy)
        if rng < 1e-3:
            continue
        half = math.atan2(max(float(r), 0.0), rng) + margin_rad
        centre = math.atan2(dy, dx)
        sectors.append((centre - half, centre + half))
    return sectors


def in_sectors(bearings, sectors):
    """Bool mask: bearing inside any of the sectors (wrap-safe)."""
    bearings = np.asarray(bearings, dtype=float)
    hit = np.zeros(bearings.shape, dtype=bool)
    for lo, hi in sectors:
        mid, half = 0.5 * (lo + hi), 0.5 * (hi - lo)
        diff = np.arctan2(np.sin(bearings - mid), np.cos(bearings - mid))
        hit |= np.abs(diff) <= half
    return hit


def scan_points(ranges, angle_min, angle_increment, range_min, range_max, laser_pose,
                half_angle_rad, sectors=(), stride=1, with_bearings=False):
    """Valid returns within +-half_angle_rad of the heading, outside the sectors.

    Returns (points (N, 2) base_link, beams_in_sector, beams_excluded), plus
    the points' bearings [rad] from the laser when with_bearings. A beam
    in an excluded sector is not counted as a sector beam either: it is
    neither evidence for nor against a wall. range_max rejects the driver's
    finite 65.533 m no-return code (see lidar_front_wall_node.py).
    """
    ranges = np.asarray(ranges, dtype=np.float64)
    lx, ly, lyaw = laser_pose
    angles = angle_min + np.arange(ranges.size) * angle_increment + lyaw
    bearings = np.arctan2(np.sin(angles), np.cos(angles))
    in_sector = np.abs(bearings) <= half_angle_rad + 0.5 * abs(angle_increment)
    if stride > 1:
        keep = np.zeros(ranges.size, dtype=bool)
        keep[::stride] = True
        in_sector &= keep
    excluded = in_sector & in_sectors(bearings, sectors) if sectors else np.zeros_like(in_sector)
    usable = in_sector & ~excluded
    with np.errstate(invalid='ignore'):
        valid = usable & np.isfinite(ranges) & (ranges >= range_min) & (ranges <= range_max)
    r, b = ranges[valid], bearings[valid]
    pts = np.column_stack([lx + r * np.cos(b), ly + r * np.sin(b)])
    if with_bearings:
        return pts, int(np.count_nonzero(usable)), int(np.count_nonzero(excluded)), b
    return pts, int(np.count_nonzero(usable)), int(np.count_nonzero(excluded))


def wall_ahead(wall, points, bearings, band_m=0.15, probe_rad=math.radians(2.0)):
    """True when the wall is the surface AHEAD: under the heading, not seen through.

    The heading ray must meet the line inside its inlier extent, and the
    returns within probe_rad of the heading must mostly stop at the line or
    nearer (an object in front of a wall still has the wall behind it). Most
    of them going more than band_m BEYOND the line means an opening: a line
    through two collinear faces either side of a doorway is not a wall ahead
    (2026-09-11 drive_stop_2m_from_wall: faces at 5.7 m either side of a 2.4 m
    doorway, the end wall at 10 m). With fewer than three such returns (an
    object dead ahead was cut out, or nothing returned) the nearest return on
    EACH side of the heading is tested the same way.
    """
    if wall is None or len(points) == 0 or wall.nx <= 1e-6:
        return False
    t_hit = wall.along(wall.d / wall.nx, 0.0)
    if not (wall.t0 <= t_hit <= wall.t1):
        return False
    pts = np.asarray(points, dtype=float)
    short = wall.d - (pts[:, 0] * wall.nx + pts[:, 1] * wall.ny) >= -band_m
    b = np.asarray(bearings, dtype=float)
    near = np.abs(b) <= probe_rad
    if np.count_nonzero(near) >= 3:
        return np.count_nonzero(short & near) * 2 >= np.count_nonzero(near)
    left, right = b > 0.0, b < 0.0
    if not left.any() or not right.any():
        return False
    return bool(short[np.argmin(np.where(left, b, np.inf))]
                and short[np.argmax(np.where(right, b, -np.inf))])


def heading_wall(lines, oblique_max_rad, points, bearings):
    """The nearest line that is the wall AHEAD (wall_ahead), or None.

    A side wall (normal past oblique_max_rad) is not it. Nearest along the
    heading, not longest, so the conservative surface wins when two real walls
    overlap in bearing.
    """
    best = None
    for w in lines:
        if w.nx <= 1e-6 or abs(w.normal_angle) > oblique_max_rad:
            continue
        if not wall_ahead(w, points, bearings):
            continue
        if best is None or w.d / w.nx < best.d / best.nx:
            best = w
    return best


def refit_wall(points, wall, inlier_m, window_m):
    """Track a wall line on a new scan: total least squares on the points within
    window_m of it, then its inliers and span at inlier_m. None if < 3 points.

    Per scan, so the estimate follows the car between the (slower) sequential
    extractions without re-running them.
    """
    pts = np.asarray(points, dtype=float)
    if wall is None or len(pts) < 3:
        return None
    near = np.abs(wall.d - (pts[:, 0] * wall.nx + pts[:, 1] * wall.ny)) <= window_m
    if np.count_nonzero(near) < 3:
        return None
    sel = pts[near]
    centroid = sel.mean(axis=0)
    _, _, vt = np.linalg.svd(sel - centroid, full_matrices=False)
    nx, ny = float(vt[-1][0]), float(vt[-1][1])
    d = nx * centroid[0] + ny * centroid[1]
    if d < 0.0:
        nx, ny, d = -nx, -ny, -d
    res = d - (pts[:, 0] * nx + pts[:, 1] * ny)
    on = np.abs(res) <= inlier_m
    if np.count_nonzero(on) < 3:
        return None
    t = -ny * pts[on, 0] + nx * pts[on, 1]
    return WallLine(nx, ny, float(d), float(t.min()), float(t.max()), int(np.count_nonzero(on)),
                    float(np.sqrt(np.mean(res[on] ** 2))), wall.source)


def sequential_lines(points, inlier_m=0.03, min_span_m=0.8, min_inliers=15, max_lines=3,
                     max_pairs=400, source='lidar'):
    """Up to max_lines wall lines by sequential robust fitting (lidar_front_wall's MSAC).

    Each round fits the dominant line among the remaining points, removes its
    inliers, and keeps it only when it spans >= min_span_m with >= min_inliers
    returns: a person or a chair leg is a short line and is dropped here even
    when no detection cut it out first.
    """
    pts = np.asarray(points, dtype=float)
    lines = []
    for _ in range(2 * max_lines):
        if len(lines) >= max_lines or len(pts) < max(3, min_inliers):
            break
        fit = _robust_line(pts, inlier_m, max_pairs)
        if fit is None:
            break
        nx, ny, d, _count, rms = fit
        on = np.abs(pts @ np.array([nx, ny]) - d) <= inlier_m
        if np.count_nonzero(on) < 3:
            break
        t = -ny * pts[on, 0] + nx * pts[on, 1]
        if np.count_nonzero(on) >= min_inliers and t.max() - t.min() >= min_span_m:
            lines.append(WallLine(nx, ny, d, float(t.min()), float(t.max()),
                                  int(np.count_nonzero(on)), rms, source))
        pts = pts[~on]
    return lines


# ============================================================================
# Ground and ceiling (stereo)
# ============================================================================

def fit_ground(points, prior_z=0.0, candidate_band_m=0.15, inlier_m=0.03,
               max_angle_deg=5.0, max_offset_m=0.05, iters=40, min_inliers=100, rng=None,
               max_candidates=1500):
    """The floor plane, seeded by the TF-predicted plane z = prior_z (base_link).

    RANSAC over the points within candidate_band_m of the prior (at most
    max_candidates of them, evenly spaced), all hypotheses scored at once, then
    a least-squares refit on the winner's inliers. Accepted only within
    max_angle_deg and max_offset_m of the prior; otherwise the prior itself is
    the floor. Returns (GroundPlane, fitted). Same rule as
    ground_plane.fit_ground_plane, vectorised: its per-hypothesis Python loop
    cost ~21 ms per frame on the Orin.
    """
    prior = GroundPlane((0.0, 0.0, 1.0), -float(prior_z))
    pts = np.asarray(points, dtype=float)
    if pts.size == 0:
        return prior, False
    cand = pts[np.abs(pts[:, 2] - prior_z) <= candidate_band_m]
    cand = cand[np.all(np.isfinite(cand), axis=1)]
    n_full = cand.shape[0]
    if n_full < max(3, min_inliers):
        return prior, False
    if n_full > max_candidates:
        cand = cand[np.linspace(0, n_full - 1, max_candidates).astype(int)]
    scale = n_full / float(cand.shape[0])
    rng = np.random.default_rng(0) if rng is None else rng
    idx = rng.integers(0, cand.shape[0], size=(int(iters), 3))
    p0, p1, p2 = cand[idx[:, 0]], cand[idx[:, 1]], cand[idx[:, 2]]
    normals = np.cross(p1 - p0, p2 - p0)
    norm = np.linalg.norm(normals, axis=1)
    ok = norm > 1e-9
    normals = normals[ok] / norm[ok, None]
    p0 = p0[ok]
    normals *= np.where(normals[:, 2] < 0, -1.0, 1.0)[:, None]
    cos_tilt = math.cos(math.radians(max_angle_deg))
    keep = normals[:, 2] >= cos_tilt
    normals, p0 = normals[keep], p0[keep]
    if normals.shape[0] == 0:
        return prior, False
    offsets = np.einsum('ij,ij->i', normals, p0)
    res = np.abs(cand[:, 0, None] * normals[:, 0] + cand[:, 1, None] * normals[:, 1]
                 + cand[:, 2, None] * normals[:, 2] - offsets)
    counts = np.count_nonzero(res < inlier_m, axis=0)
    best = int(np.argmax(counts))
    if counts[best] * scale < min_inliers:
        return prior, False
    inl = cand[res[:, best] < inlier_m]
    centroid = inl.mean(axis=0)
    n = np.linalg.svd(inl - centroid, full_matrices=False)[2][-1]
    if n[2] < 0:
        n = -n
    plane = GroundPlane(n, -float(n @ centroid), int(counts[best] * scale), True)
    if n[2] < cos_tilt or abs(plane.z_at_origin() - prior_z) > max_offset_m:
        return prior, False
    return plane, True


class CeilingTracker:
    """Ceiling height [m above the floor]: histogram peak, EMA, max-step gate.

    Heights in [min_h, max_h] go into bin_m bins; the fullest bin holding at
    least min_count points is this frame's ceiling. The estimate is an EMA of
    those peaks; a peak more than max_step_m from it is rejected (a shelf or a
    door lintel) unless reject_reset frames in a row agree it moved. Until the
    first peak, prior_h is used when > 0 (none otherwise).
    """

    def __init__(self, min_h=2.0, max_h=4.0, bin_m=0.02, min_count=40, alpha=0.3,
                 max_step_m=0.25, reject_reset=10, prior_h=0.0):
        self.min_h, self.max_h, self.bin_m = float(min_h), float(max_h), float(bin_m)
        self.min_count, self.alpha = int(min_count), float(alpha)
        self.max_step_m, self.reject_reset = float(max_step_m), int(reject_reset)
        self.value = float(prior_h) if prior_h and prior_h > 0.0 else None
        self.measured = False
        self._rejects = 0

    def update(self, heights):
        h = np.asarray(heights, dtype=float)
        h = h[(h >= self.min_h) & (h <= self.max_h)]
        if h.size < self.min_count:
            return self.value
        nbins = max(1, int(round((self.max_h - self.min_h) / self.bin_m)))
        counts, edges = np.histogram(h, bins=nbins, range=(self.min_h, self.max_h))
        k = int(np.argmax(counts))
        if counts[k] < self.min_count:
            return self.value
        peak = 0.5 * (edges[k] + edges[k + 1])
        if self.value is None or not self.measured:
            self.value, self.measured, self._rejects = peak, True, 0
        elif abs(peak - self.value) <= self.max_step_m:
            self.value += self.alpha * (peak - self.value)
            self._rejects = 0
        else:
            self._rejects += 1
            if self._rejects >= self.reject_reset:
                self.value, self._rejects = peak, 0
        return self.value


# ============================================================================
# Labelling
# ============================================================================

def label_points(points, in_mask, ground, ceiling_h, walls, ground_band_m=0.04,
                 ceiling_band_m=0.08, max_relevant_height_m=1.0, wall_band_m=0.06,
                 wall_span_margin_m=0.10):
    """Per-point labels for (N, 3) base_link points.

    Order, first match wins:
      GROUND   less than ground_band_m above the floor (or below it).
      CEILING  within ceiling_band_m of the ceiling height, or above it.
      WALL     within wall_band_m of a wall plane and inside its span (+margin).
               Never a YOLO-mask point: a person against a wall is not wall.
      IGNORE   above max_relevant_height_m and NOT a YOLO-mask point (mask
               points are never dropped by height).
      OTHER    the rest.
    Returns (labels uint8 (N,), wall_index int (N,), -1 where not WALL).
    """
    pts = np.asarray(points, dtype=float)
    n = pts.shape[0]
    labels = np.full(n, LABEL_OTHER, dtype=np.uint8)
    wall_idx = np.full(n, -1, dtype=np.int32)
    if n == 0:
        return labels, wall_idx
    mask = np.zeros(n, dtype=bool) if in_mask is None else np.asarray(in_mask, dtype=bool)
    h = ground.height(pts)
    free = np.ones(n, dtype=bool)

    g = h < ground_band_m
    labels[g] = LABEL_GROUND
    free &= ~g
    if ceiling_h is not None:
        c = free & (h > ceiling_h - ceiling_band_m)
        labels[c] = LABEL_CEILING
        free &= ~c
    best = np.full(n, np.inf)
    for i, wall in enumerate(walls):
        off = np.abs(wall.offset(pts[:, 0], pts[:, 1]))
        t = wall.along(pts[:, 0], pts[:, 1])
        on = (free & ~mask & (off <= wall_band_m) & (off < best)
              & (t >= wall.t0 - wall_span_margin_m) & (t <= wall.t1 + wall_span_margin_m))
        best[on] = off[on]
        wall_idx[on] = i
    w = wall_idx >= 0
    labels[w] = LABEL_WALL
    free &= ~w
    labels[free & ~mask & (h > max_relevant_height_m)] = LABEL_IGNORE
    return labels, wall_idx


def fallback_wall(points, candidates, inlier_m=0.06, min_span_m=0.8, min_inliers=40,
                  oblique_max_rad=math.radians(45.0), max_pairs=300):
    """A stereo vertical plane among candidate points, or None. Flagged source.

    Only for when the LiDAR has no wall in the region (glass, a wall above the
    scan plane): one robust line through the candidates' (x, y), accepted
    when it spans min_span_m, holds min_inliers and faces the car.
    """
    pts = np.asarray(points, dtype=float)[np.asarray(candidates, dtype=bool)]
    if len(pts) < min_inliers:
        return None
    if len(pts) > 400:
        # A fixed sample size keeps lidar_front_wall's pair cache to one entry.
        pts = pts[np.linspace(0, len(pts) - 1, 400).astype(int)]
    fit = _robust_line(pts[:, :2], inlier_m, max_pairs)
    if fit is None:
        return None
    nx, ny, d, count, rms = fit
    if count < min_inliers or abs(math.atan2(ny, nx)) > oblique_max_rad:
        return None
    on = np.abs(pts[:, :2] @ np.array([nx, ny]) - d) <= inlier_m
    t = -ny * pts[on, 0] + nx * pts[on, 1]
    if t.max() - t.min() < min_span_m:
        return None
    return WallLine(nx, ny, d, float(t.min()), float(t.max()), int(count), rms, 'stereo')


def stereo_wall_distance(points, labels, wall_idx, walls, in_roi, front_x, min_px=30):
    """Stereo distance [m] from the bumper to the wall seen in the ROI, or None.

    Taken on WALL points inside the ROI only, for the plane holding most of
    them: the median of their offsets along that plane's normal, minus the
    bumper's. Independent of the LiDAR's own d: it is where the stereo points
    of that wall are. Returns (distance, n_px, wall index) or (None, n, None).
    """
    sel = (np.asarray(labels) == LABEL_WALL) & np.asarray(in_roi, dtype=bool)
    if np.count_nonzero(sel) < min_px:
        return None, int(np.count_nonzero(sel)), None
    idx = np.asarray(wall_idx)[sel]
    k = int(np.bincount(idx[idx >= 0]).argmax())
    on = sel & (np.asarray(wall_idx) == k)
    if np.count_nonzero(on) < min_px:
        return None, int(np.count_nonzero(on)), None
    wall = walls[k]
    pts = np.asarray(points, dtype=float)[on]
    along_n = pts[:, 0] * wall.nx + pts[:, 1] * wall.ny
    return float(np.median(along_n) - front_x * wall.nx), int(np.count_nonzero(on)), k


def cluster_other(points, labels, in_mask, cell_m=0.10, x_max=5.0, y_half=2.5,
                  min_points=8):
    """Class-agnostic candidates: OTHER points outside every YOLO mask, gridded.

    Connected 8-neighbour components of the occupied x-y cells. Returns
    [(member index array, summary dict)]. Log only: nothing steers on them.
    """
    import cv2  # local: keep the module importable where cv2 is absent
    pts = np.asarray(points, dtype=float)
    sel = (np.asarray(labels) == LABEL_OTHER) & ~np.asarray(in_mask, dtype=bool)
    sel &= (pts[:, 0] > 0.0) & (pts[:, 0] < x_max) & (np.abs(pts[:, 1]) < y_half)
    idx = np.nonzero(sel)[0]
    if idx.size < min_points:
        return []
    gx = (pts[idx, 0] / cell_m).astype(int)
    gy = ((pts[idx, 1] + y_half) / cell_m).astype(int)
    grid = np.zeros((int(x_max / cell_m) + 1, int(2 * y_half / cell_m) + 1), dtype=np.uint8)
    grid[gx, gy] = 1
    n, comp = cv2.connectedComponents(grid, connectivity=8)
    member = comp[gx, gy]
    out = []
    for c in range(1, n):
        m = idx[member == c]
        if m.size < min_points:
            continue
        p = pts[m]
        cx, cy = float(p[:, 0].mean()), float(p[:, 1].mean())
        out.append((m, {
            'x': round(cx, 3), 'y': round(cy, 3),
            'r': round(float(np.hypot(p[:, 0] - cx, p[:, 1] - cy).max()), 3),
            'x_min': round(float(p[:, 0].min()), 3),
            'z_min': round(float(p[:, 2].min()), 3), 'z_max': round(float(p[:, 2].max()), 3),
            'n': int(m.size)}))
    return out


# ============================================================================
# Virtual wall: LiDAR estimate + stereo WALL distance + odometry
# ============================================================================

@dataclass(frozen=True)
class VirtualStep:
    valid: bool
    distance: float          # from the front bumper [m], NaN when not valid
    source: str              # lidar | lidar_dr | stereo | none, '+stereo' when fused
    disagree: bool           # LiDAR and stereo apart by more than disagree_m
    stereo_fallback: bool    # the stereo distance came from a stereo-only plane
    lidar_m: float           # LiDAR estimate from the front [m] or NaN
    stereo_m: float          # stereo WALL distance from the front [m] or NaN


class VirtualWall:
    """The single wall truth: one distance from the front bumper.

    LiDAR first. Its estimate is lidar_front_wall.WallEstimator's: a measured
    fit, or the last one dead-reckoned with odometry for up to stale_age_sec.
    A SEEDED estimate is not used here: the seed is the nearest occupied map
    cell in a cone and can be a person, which is the failure this exists to
    end. The stereo WALL distance (fresh within stereo_max_age_s) is fused by
    inverse variance when it agrees with the LiDAR; when the two differ by more
    than disagree_m the LiDAR is trusted and the step is flagged.

    WITH NO LIDAR ESTIMATE THERE IS NO WALL (2026-10-15, step 5). The stereo
    distance used to stand in on its own, dead-reckoned between frames. That
    path is removed: an empty LiDAR strip meant the stereo plane became the
    wall, and on an open corridor that plane was the SIDE wall, reported as a
    front wall at 1.1 m (corridor_phantom). Stereo can support a wall only by
    going through the observations and the tracker, where a wall has to cross
    the car's strip to be the front wall at all. An empty strip is CLEAR, and
    saying so is wall_fusion.front_wall_output's job.
    """

    def __init__(self, front_x, disagree_m=0.15, stereo_max_age_s=0.5,
                 lidar_sigma_m=0.02, dr_sigma_per_s=0.05, stereo_sigma_k=0.01,
                 stereo_dr_max_s=1.0):
        self.front_x = float(front_x)
        self.disagree_m = float(disagree_m)
        self.stereo_max_age_s = float(stereo_max_age_s)
        self.lidar_sigma_m = float(lidar_sigma_m)
        self.dr_sigma_per_s = float(dr_sigma_per_s)
        self.stereo_sigma_k = float(stereo_sigma_k)
        self.stereo_dr_max_s = float(stereo_dr_max_s)
        self._stereo = None    # (time, distance, fallback, pose)
        self._pose = None      # (x, y, yaw)

    def update_pose(self, x, y, yaw):
        self._pose = (float(x), float(y), float(yaw))

    def update_stereo(self, now, distance_front, fallback=False):
        if distance_front is None or not math.isfinite(distance_front) or distance_front <= 0.0:
            return
        self._stereo = (float(now), float(distance_front), bool(fallback), self._pose)

    def _stereo_now(self, now):
        """(distance, fallback, dead_reckoned) of the stereo wall, or None."""
        s = self._stereo
        if s is None:
            return None
        age = now - s[0]
        if age <= self.stereo_max_age_s:
            return s[1], s[2], False
        if age > self.stereo_dr_max_s or s[3] is None or self._pose is None:
            return None
        # Closing along the heading at the stereo frame: the stereo distance
        # is taken along the wall normal, ~ the heading for a wall ahead.
        x0, y0, yaw0 = s[3]
        x, y, _ = self._pose
        closing = (x - x0) * math.cos(yaw0) + (y - y0) * math.sin(yaw0)
        return s[1] - closing, s[2], True

    def step(self, now, estimate):
        lidar_m = math.nan
        lidar_sigma = None
        source = 'none'
        if estimate is not None and estimate.valid and estimate.provenance in (
                PROVENANCE_MEASURED, PROVENANCE_DEAD_RECKONED):
            na = estimate.normal_angle if math.isfinite(estimate.normal_angle) else 0.0
            lidar_m = estimate.distance - self.front_x * math.cos(na)
            measured = estimate.provenance == PROVENANCE_MEASURED
            source = 'lidar' if measured else 'lidar_dr'
            lidar_sigma = self.lidar_sigma_m + (
                0.0 if measured else self.dr_sigma_per_s * max(estimate.source_age, 0.0))
        stereo = self._stereo_now(now)
        stereo_m = stereo[0] if stereo is not None else math.nan
        fallback = bool(stereo[1]) if stereo is not None else False

        disagree = False
        if lidar_sigma is not None:
            distance = lidar_m
            if stereo is not None and not stereo[2]:
                if abs(stereo_m - lidar_m) > self.disagree_m:
                    disagree = True
                else:
                    s2 = (self.stereo_sigma_k * max(stereo_m, 0.3) ** 2) ** 2
                    l2 = lidar_sigma ** 2
                    distance = (lidar_m / l2 + stereo_m / s2) / (1.0 / l2 + 1.0 / s2)
                    source += '+stereo'
        else:
            # No LiDAR estimate: no wall. See the class docstring.
            return VirtualStep(False, math.nan, 'none', False, False, lidar_m, stereo_m)
        return VirtualStep(True, float(distance), source, disagree, fallback, lidar_m, stereo_m)
