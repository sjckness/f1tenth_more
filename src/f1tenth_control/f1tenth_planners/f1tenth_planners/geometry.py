"""Planar geometry shared by the planners: frames, distances, polygons, paths.

Pure: numpy and math only, no rclpy. Every frame is the local EKF's odom
frame unless a name says otherwise, the same frame mpc_corr plans in.
"""

import math
from typing import Sequence, Tuple

import numpy as np

Pose = Tuple[float, float, float]


def wrap_pi(angle: float) -> float:
    """Wrap an angle to (-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


def pose_map_to_odom(x, y, psi, tf_x, tf_y, tf_yaw) -> Pose:
    """Express a map-frame pose in odom, given the map -> odom edge.

    The same convention as MPC_corr._pose_map_to_odom: (tf_x, tf_y, tf_yaw) is
    lookup_transform('map', 'odom'), the odom origin expressed in map.
    """
    cos_t, sin_t = math.cos(tf_yaw), math.sin(tf_yaw)
    dx, dy = x - tf_x, y - tf_y
    return (cos_t * dx + sin_t * dy, -sin_t * dx + cos_t * dy, wrap_pi(psi - tf_yaw))


def robot_to_world(pose: Pose, x_r: float, y_r: float) -> Tuple[float, float]:
    """Map a base_link point into the frame ``pose`` is expressed in."""
    x, y, yaw = pose
    c, s = math.cos(yaw), math.sin(yaw)
    return x + c * x_r - s * y_r, y + s * x_r + c * y_r


def world_to_robot(pose: Pose, x_w: float, y_w: float) -> Tuple[float, float]:
    """Inverse of robot_to_world."""
    x, y, yaw = pose
    c, s = math.cos(yaw), math.sin(yaw)
    dx, dy = x_w - x, y_w - y
    return c * dx + s * dy, -s * dx + c * dy


def project_onto_line(point, origin, psi) -> float:
    """Signed distance of ``point`` along the line through ``origin`` at ``psi``."""
    return ((point[0] - origin[0]) * math.cos(psi)
            + (point[1] - origin[1]) * math.sin(psi))


def point_segment_distance(px, py, ax, ay, bx, by):
    """Distance from point(s) (px, py) to the segment a-b; numpy-broadcasting."""
    abx, aby = bx - ax, by - ay
    denom = abx * abx + aby * aby
    if np.ndim(denom) == 0 and denom < 1e-18:
        return np.hypot(px - ax, py - ay)
    t = ((px - ax) * abx + (py - ay) * aby) / np.maximum(denom, 1e-18)
    t = np.clip(t, 0.0, 1.0)
    return np.hypot(px - (ax + t * abx), py - (ay + t * aby))


def _segments_intersect(a, b, c, d) -> np.ndarray:
    """Whether segment a-b properly crosses each segment c[i]-d[i] (arrays Nx2)."""
    def orient(p, q, r):
        return (q[..., 0] - p[..., 0]) * (r[..., 1] - p[..., 1]) - \
            (q[..., 1] - p[..., 1]) * (r[..., 0] - p[..., 0])
    a = np.asarray(a, dtype=float)
    b = np.asarray(b, dtype=float)
    o1 = orient(a, b, c)
    o2 = orient(a, b, d)
    o3 = orient(c, d, a)
    o4 = orient(c, d, b)
    return (o1 * o2 < 0.0) & (o3 * o4 < 0.0)


def segment_segments_distance(a, b, c, d) -> np.ndarray:
    """Distance from segment a-b to each segment c[i]-d[i] (c, d arrays Nx2)."""
    c = np.asarray(c, dtype=float).reshape(-1, 2)
    d = np.asarray(d, dtype=float).reshape(-1, 2)
    if c.shape[0] == 0:
        return np.zeros(0)
    ax, ay = float(a[0]), float(a[1])
    bx, by = float(b[0]), float(b[1])
    d1 = point_segment_distance(c[:, 0], c[:, 1], ax, ay, bx, by)
    d2 = point_segment_distance(d[:, 0], d[:, 1], ax, ay, bx, by)
    d3 = point_segment_distance(ax, ay, c[:, 0], c[:, 1], d[:, 0], d[:, 1])
    d4 = point_segment_distance(bx, by, c[:, 0], c[:, 1], d[:, 0], d[:, 1])
    dist = np.minimum(np.minimum(d1, d2), np.minimum(d3, d4))
    return np.where(_segments_intersect(a, b, c, d), 0.0, dist)


class PreparedPolygon:
    """A closed polygon (Mx2) with its edges precomputed, for repeated containment tests."""

    def __init__(self, polygon):
        polygon = np.asarray(polygon, dtype=float).reshape(-1, 2)
        self.polygon = polygon
        self._x1 = polygon[:, 0][None, :]
        self._y1 = polygon[:, 1][None, :]
        x2 = np.roll(polygon[:, 0], -1)[None, :]
        y2 = np.roll(polygon[:, 1], -1)[None, :]
        dy = y2 - self._y1
        with np.errstate(divide='ignore', invalid='ignore'):
            self._slope = np.where(dy != 0.0, (x2 - self._x1) / dy, 0.0)
        self._y2 = y2

    def contains(self, xs, ys) -> np.ndarray:
        """Even-odd ray-casting containment of the points (xs, ys)."""
        xs = np.atleast_1d(np.asarray(xs, dtype=float))[:, None]
        ys = np.atleast_1d(np.asarray(ys, dtype=float))[:, None]
        crosses = (self._y1 > ys) != (self._y2 > ys)
        x_at = self._x1 + (ys - self._y1) * self._slope
        hits = crosses & (xs < x_at)
        return (np.count_nonzero(hits, axis=1) % 2) == 1


def points_in_polygon(xs, ys, polygon) -> np.ndarray:
    """Even-odd ray-casting containment of points (xs, ys) in a closed polygon (Mx2)."""
    if not isinstance(polygon, PreparedPolygon):
        polygon = PreparedPolygon(polygon)
    return polygon.contains(xs, ys)


def corridor_polygon(corridor: dict) -> np.ndarray:
    """Close a corridor_geometry corridor into one polygon: left wall, then right reversed."""
    left = np.column_stack([corridor['xL'], corridor['yL']])
    right = np.column_stack([corridor['xR'], corridor['yR']])[::-1]
    return np.vstack([left, right])


def path_arclength(path: np.ndarray) -> np.ndarray:
    """Cumulative arc length at each vertex of an Nx2 path (0 at the first)."""
    if len(path) == 0:
        return np.zeros(0)
    seg = np.hypot(np.diff(path[:, 0]), np.diff(path[:, 1]))
    return np.concatenate([[0.0], np.cumsum(seg)])


def resample_path(path: np.ndarray, ds: float) -> np.ndarray:
    """Resample a polyline at a constant spacing ``ds``, keeping both ends."""
    path = np.asarray(path, dtype=float).reshape(-1, 2)
    if len(path) < 2:
        return path.copy()
    s = path_arclength(path)
    if s[-1] < 1e-9:
        return path[:1].copy()
    n = max(int(math.ceil(s[-1] / ds)), 1)
    s_new = np.linspace(0.0, s[-1], n + 1)
    return np.column_stack([np.interp(s_new, s, path[:, 0]),
                            np.interp(s_new, s, path[:, 1])])


def nearest_on_path(path: np.ndarray, x: float, y: float) -> Tuple[int, float, float]:
    """Return (segment index, arc length at the foot point, distance) of (x, y) on ``path``."""
    if len(path) == 1:
        return 0, 0.0, float(math.hypot(x - path[0, 0], y - path[0, 1]))
    a, b = path[:-1], path[1:]
    d = point_segment_distance(x, y, a[:, 0], a[:, 1], b[:, 0], b[:, 1])
    i = int(np.argmin(d))
    ab = b[i] - a[i]
    denom = float(ab @ ab)
    t = 0.0 if denom < 1e-18 else float(np.clip(((x, y) - a[i]) @ ab / denom, 0.0, 1.0))
    s = path_arclength(path)
    return i, float(s[i] + t * math.sqrt(denom)), float(d[i])


def point_at_arclength(path: np.ndarray, s_query: float) -> Tuple[float, float, float]:
    """Point (x, y) and tangent heading at arc length ``s_query``; extrapolates past the end."""
    s = path_arclength(path)
    if len(path) < 2:
        return float(path[0, 0]), float(path[0, 1]), 0.0
    if s_query >= s[-1]:
        tx, ty = path[-1] - path[-2]
        heading = math.atan2(ty, tx)
        extra = s_query - s[-1]
        return (float(path[-1, 0] + extra * math.cos(heading)),
                float(path[-1, 1] + extra * math.sin(heading)), heading)
    s_query = max(s_query, 0.0)
    i = int(np.searchsorted(s, s_query, side='right') - 1)
    i = min(max(i, 0), len(path) - 2)
    seg = s[i + 1] - s[i]
    t = 0.0 if seg < 1e-12 else (s_query - s[i]) / seg
    p = path[i] + t * (path[i + 1] - path[i])
    tx, ty = path[i + 1] - path[i]
    return float(p[0]), float(p[1]), math.atan2(ty, tx)


class Obstacles:
    """What a path must keep clear of: inflated circles and wall segments, odom frame.

    ``circles`` is Nx3 (x, y, r) with r ALREADY inflated by whatever clearance
    the caller wants (car radius + margin); ``walls`` is Mx4 (x0, y0, x1, y1)
    segments that must stay at least ``wall_clearance`` away.
    """

    def __init__(self, circles: Sequence = (), walls: Sequence = (),
                 wall_clearance: float = 0.0):
        self.circles = np.asarray(circles, dtype=float).reshape(-1, 3)
        self.walls = np.asarray(walls, dtype=float).reshape(-1, 4)
        self.wall_clearance = float(wall_clearance)

    def __len__(self):
        return len(self.circles) + len(self.walls)

    def point_free(self, x: float, y: float) -> bool:
        """Whether a point is outside every inflated circle and clear of every wall."""
        if len(self.circles):
            if np.any(np.hypot(self.circles[:, 0] - x, self.circles[:, 1] - y)
                      <= self.circles[:, 2]):
                return False
        if len(self.walls):
            d = point_segment_distance(x, y, self.walls[:, 0], self.walls[:, 1],
                                       self.walls[:, 2], self.walls[:, 3])
            if np.any(d <= self.wall_clearance):
                return False
        return True

    def segment_free(self, a, b) -> bool:
        """Whether the segment a-b keeps clear of every circle and wall."""
        if len(self.circles):
            d = point_segment_distance(self.circles[:, 0], self.circles[:, 1],
                                       float(a[0]), float(a[1]), float(b[0]), float(b[1]))
            if np.any(d <= self.circles[:, 2]):
                return False
        if len(self.walls):
            d = segment_segments_distance(a, b, self.walls[:, 0:2], self.walls[:, 2:4])
            if np.any(d <= self.wall_clearance):
                return False
        return True

    def clearance(self, x: float, y: float) -> float:
        """Signed clearance of a point: smallest distance minus the inflation (inf if empty)."""
        best = math.inf
        if len(self.circles):
            best = min(best, float(np.min(
                np.hypot(self.circles[:, 0] - x, self.circles[:, 1] - y) - self.circles[:, 2])))
        if len(self.walls):
            d = point_segment_distance(x, y, self.walls[:, 0], self.walls[:, 1],
                                       self.walls[:, 2], self.walls[:, 3])
            best = min(best, float(np.min(d)) - self.wall_clearance)
        return best

    def first_blocked_arclength(self, path: np.ndarray, s_from: float = 0.0,
                                s_max: float = math.inf) -> float:
        """Arc length of the first blocked path segment past ``s_from``, or inf."""
        s = path_arclength(path)
        for i in range(len(path) - 1):
            if s[i + 1] < s_from:
                continue
            if s[i] > s_from + s_max:
                break
            if not self.segment_free(path[i], path[i + 1]):
                return max(float(s[i]), s_from)
        return math.inf
