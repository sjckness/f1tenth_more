"""
Forward-only Dubins paths for a car-like robot, plus a footprint check.

Pure Python + math (numpy only in the collision check). No rclpy.

A Dubins path is the shortest forward-only path between two poses for a
vehicle that cannot turn tighter than a radius rho: at most three segments,
each a full-lock arc (L left, R right) or a straight (S). The six candidate
words are LSL, RSR, LSR, RSL, RLR, LRL; shortest_path() solves all six in
the standard normalised form (Shkel & Lumelsky 2001; the same formulas as
A. Walker's reference `dubins.c`) and keeps the shortest that exists.

Normalised form: scale by 1/rho so the turning radius is 1; d = D / rho is the
start-goal distance; alpha and beta are the start and goal yaws measured from
the start->goal bearing. Each word gives segment lengths (t, p, q) in radians
of arc (or units of rho for S); multiplied by rho they are metres.

Frames: q = (x, y, yaw), both poses in the same frame (enter_door uses map),
yaw counter-clockwise positive (REP-103), so L increases the yaw.
"""

from collections import namedtuple
import math

__all__ = [
    'WORDS', 'DubinsPath', 'FreeCheck', 'OccGrid', 'shortest_path', 'all_paths',
    'path_is_free', 'straight_back_is_free', 'mod2pi', 'wrap_pi',
]

WORDS = ('LSL', 'RSR', 'LSR', 'RSL', 'RLR', 'LRL')

#: Below this [m] the start and goal positions coincide, and with matching
#: yaws the path is empty rather than a 2*pi loop.
_EPS_POS = 1e-9
_EPS_YAW = 1e-9
#: A sqrt argument this far below zero is rounding, not infeasibility.
_EPS_SQRT = 1e-12


def mod2pi(a):
    """Return `a` wrapped to [0, 2 pi)."""
    out = math.fmod(a, 2.0 * math.pi)
    if out < 0.0:
        out += 2.0 * math.pi
    if out >= 2.0 * math.pi:        # fmod(-tiny) + 2 pi can round up to 2 pi
        out = 0.0
    return out


def wrap_pi(a):
    """Return `a` wrapped to (-pi, pi] via atan2(sin, cos)."""
    return math.atan2(math.sin(a), math.cos(a))


def _sqrt_or_none(v):
    if v < -_EPS_SQRT:
        return None
    return math.sqrt(max(v, 0.0))


# --------------------------------------------------------------- the six words
# Each takes (alpha, beta, d) and returns (t, p, q) normalised, or None.

def _lsl(a, b, d):
    sa, sb, ca, cb = math.sin(a), math.sin(b), math.cos(a), math.cos(b)
    p = _sqrt_or_none(2.0 + d * d - 2.0 * math.cos(a - b) + 2.0 * d * (sa - sb))
    if p is None:
        return None
    tmp = math.atan2(cb - ca, d + sa - sb)
    return mod2pi(-a + tmp), p, mod2pi(b - tmp)


def _rsr(a, b, d):
    sa, sb, ca, cb = math.sin(a), math.sin(b), math.cos(a), math.cos(b)
    p = _sqrt_or_none(2.0 + d * d - 2.0 * math.cos(a - b) + 2.0 * d * (sb - sa))
    if p is None:
        return None
    tmp = math.atan2(ca - cb, d - sa + sb)
    return mod2pi(a - tmp), p, mod2pi(-b + tmp)


def _lsr(a, b, d):
    sa, sb, ca, cb = math.sin(a), math.sin(b), math.cos(a), math.cos(b)
    p = _sqrt_or_none(-2.0 + d * d + 2.0 * math.cos(a - b) + 2.0 * d * (sa + sb))
    if p is None:
        return None
    tmp = math.atan2(-ca - cb, d + sa + sb) - math.atan2(-2.0, p)
    return mod2pi(-a + tmp), p, mod2pi(-b + tmp)


def _rsl(a, b, d):
    sa, sb, ca, cb = math.sin(a), math.sin(b), math.cos(a), math.cos(b)
    p = _sqrt_or_none(d * d - 2.0 + 2.0 * math.cos(a - b) - 2.0 * d * (sa + sb))
    if p is None:
        return None
    tmp = math.atan2(ca + cb, d - sa - sb) - math.atan2(2.0, p)
    return mod2pi(a - tmp), p, mod2pi(b - tmp)


def _rlr(a, b, d):
    sa, sb, ca, cb = math.sin(a), math.sin(b), math.cos(a), math.cos(b)
    tmp = (6.0 - d * d + 2.0 * math.cos(a - b) + 2.0 * d * (sa - sb)) / 8.0
    if abs(tmp) > 1.0:
        return None
    p = mod2pi(2.0 * math.pi - math.acos(tmp))
    t = mod2pi(a - math.atan2(ca - cb, d - sa + sb) + p / 2.0)
    return t, p, mod2pi(a - b - t + p)


def _lrl(a, b, d):
    sa, sb, ca, cb = math.sin(a), math.sin(b), math.cos(a), math.cos(b)
    tmp = (6.0 - d * d + 2.0 * math.cos(a - b) + 2.0 * d * (sb - sa)) / 8.0
    if abs(tmp) > 1.0:
        return None
    p = mod2pi(2.0 * math.pi - math.acos(tmp))
    t = mod2pi(-a - math.atan2(ca - cb, d + sa - sb) + p / 2.0)
    return t, p, mod2pi(b - a - t + p)


_SOLVERS = {'LSL': _lsl, 'RSR': _rsr, 'LSR': _lsr, 'RSL': _rsl, 'RLR': _rlr, 'LRL': _lrl}


# --------------------------------------------------------------- the path

class DubinsPath:
    """One Dubins word from q0 with turning radius rho; lengths in metres."""

    #: Spacing [m] of the cached samples project() searches.
    PROJECT_STEP_M = 0.02

    def __init__(self, q0, rho, word, lengths):
        self.q0 = (float(q0[0]), float(q0[1]), float(q0[2]))
        self.rho = float(rho)
        self.word = str(word)
        self.lengths = tuple(float(v) for v in lengths)   # (t, p, q) [m]
        self.length = float(sum(self.lengths))
        self._cache = None

    @property
    def t(self):
        return self.lengths[0]

    @property
    def p(self):
        return self.lengths[1]

    @property
    def q(self):
        return self.lengths[2]

    def __repr__(self):
        return (f'DubinsPath({self.word}, t={self.t:.3f}, p={self.p:.3f}, '
                f'q={self.q:.3f}, L={self.length:.3f}, rho={self.rho:.3f})')

    def pose_at(self, s):
        """Return (x, y, yaw) at arc length s [m], clamped to [0, length]."""
        s = min(max(float(s), 0.0), self.length)
        x, y, th = 0.0, 0.0, self.q0[2]
        for kind, seg in zip(self.word, self.lengths):
            step = min(s, seg)
            x, y, th = _advance(kind, x, y, th, step / self.rho)
            s -= step
            if s <= 0.0:
                break
        return (self.q0[0] + self.rho * x, self.q0[1] + self.rho * y, wrap_pi(th))

    def end(self):
        """Return the final pose."""
        return self.pose_at(self.length)

    def sample(self, step=0.05):
        """Return [(x, y, yaw)] every `step` metres, both ends included."""
        step = max(float(step), 1e-4)
        n = max(int(math.ceil(self.length / step)), 1)
        return [self.pose_at(self.length * i / n) for i in range(n + 1)]

    def _samples(self):
        if self._cache is None:
            n = max(int(math.ceil(self.length / self.PROJECT_STEP_M)), 1)
            ss = [self.length * i / n for i in range(n + 1)]
            self._cache = (ss, [self.pose_at(s) for s in ss])
        return self._cache

    def project(self, x, y, s_min=None, s_max=None):
        """
        Return (s, lateral_err) of the closest point of the path to (x, y).

        lateral_err is signed: positive when (x, y) is LEFT of the path's
        heading at s. s_min / s_max restrict the search window (a path that
        loops back near itself would otherwise snap between its two passes).
        """
        ss, poses = self._samples()
        best_i, best_d2 = None, math.inf
        for i, (s, (px, py, _)) in enumerate(zip(ss, poses)):
            if s_min is not None and s < s_min - 1e-9:
                continue
            if s_max is not None and s > s_max + 1e-9:
                continue
            d2 = (px - x) ** 2 + (py - y) ** 2
            if d2 < best_d2:
                best_i, best_d2 = i, d2
        if best_i is None:
            best_i = 0 if s_min is None else len(ss) - 1
        # Refine on the chord to each neighbour.
        s_best = ss[best_i]
        px, py, pyaw = poses[best_i]
        for j in (best_i - 1, best_i + 1):
            if not 0 <= j < len(ss):
                continue
            ax, ay, _ = poses[best_i]
            bx, by, _ = poses[j]
            vx, vy = bx - ax, by - ay
            vv = vx * vx + vy * vy
            if vv <= 0.0:
                continue
            u = max(0.0, min(1.0, ((x - ax) * vx + (y - ay) * vy) / vv))
            cx, cy = ax + u * vx, ay + u * vy
            d2 = (cx - x) ** 2 + (cy - y) ** 2
            if d2 < best_d2 - 1e-15:
                best_d2 = d2
                s_best = ss[best_i] + u * (ss[j] - ss[best_i])
        px, py, pyaw = self.pose_at(s_best)
        lat = -math.sin(pyaw) * (x - px) + math.cos(pyaw) * (y - py)
        return s_best, lat


def _advance(kind, x, y, th, ln):
    """Advance a unit-radius pose by normalised length ln along segment `kind`."""
    if kind == 'L':
        return x + math.sin(th + ln) - math.sin(th), y - math.cos(th + ln) + math.cos(th), th + ln
    if kind == 'R':
        return x - math.sin(th - ln) + math.sin(th), y + math.cos(th - ln) - math.cos(th), th - ln
    return x + ln * math.cos(th), y + ln * math.sin(th), th


def all_paths(q0, q1, rho):
    """Return every feasible word from q0 to q1 as DubinsPath, unsorted."""
    if not rho > 0.0:
        raise ValueError(f'rho must be > 0, got {rho!r}')
    dx, dy = float(q1[0]) - float(q0[0]), float(q1[1]) - float(q0[1])
    big_d = math.hypot(dx, dy)
    if big_d < _EPS_POS and abs(wrap_pi(float(q1[2]) - float(q0[2]))) < _EPS_YAW:
        return [DubinsPath(q0, rho, 'LSL', (0.0, 0.0, 0.0))]
    d = big_d / rho
    theta = math.atan2(dy, dx) if big_d >= _EPS_POS else 0.0
    alpha = mod2pi(float(q0[2]) - theta)
    beta = mod2pi(float(q1[2]) - theta)
    out = []
    for word in WORDS:
        res = _SOLVERS[word](alpha, beta, d)
        if res is None or any(math.isnan(v) for v in res):
            continue
        out.append(DubinsPath(q0, rho, word, tuple(v * rho for v in res)))
    return out


def shortest_path(q0, q1, rho):
    """Return the shortest forward Dubins path from q0 to q1, or None."""
    paths = all_paths(q0, q1, rho)
    if not paths:
        return None
    return min(paths, key=lambda p: p.length)


# --------------------------------------------------------------- the footprint

class OccGrid:
    """
    An occupancy grid, nav_msgs/OccupancyGrid's content (row-major, -1 unknown).

    occupied: value >= occupied_threshold (the door detector's own rule,
    freespace_doors_occupied_threshold); unknown: value < 0; anything else is
    free.
    """

    def __init__(self, data, resolution, origin_x, origin_y, origin_yaw=0.0,
                 occupied_threshold=65):
        import numpy as np
        self.data = np.asarray(data, dtype=np.int16)
        if self.data.ndim != 2:
            raise ValueError('OccGrid data must be 2-D (height, width)')
        self.resolution = float(resolution)
        self.origin_x = float(origin_x)
        self.origin_y = float(origin_y)
        self.origin_yaw = float(origin_yaw)
        self.occupied_threshold = int(occupied_threshold)

    @classmethod
    def from_msg(cls, msg, occupied_threshold=65):
        """Build from a nav_msgs/OccupancyGrid."""
        import numpy as np
        q = msg.info.origin.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        data = np.asarray(msg.data, dtype=np.int16).reshape(msg.info.height, msg.info.width)
        return cls(data, msg.info.resolution, msg.info.origin.position.x,
                   msg.info.origin.position.y, yaw, occupied_threshold)

    def cells_in_box(self, xmin, ymin, xmax, ymax):
        """Return (xs, ys, values) of every cell centre in a map-frame box."""
        import numpy as np
        c, s = math.cos(self.origin_yaw), math.sin(self.origin_yaw)
        corners = [(xmin, ymin), (xmin, ymax), (xmax, ymin), (xmax, ymax)]
        gi, gj = [], []
        for (x, y) in corners:
            dx, dy = x - self.origin_x, y - self.origin_y
            gi.append((c * dx + s * dy) / self.resolution)
            gj.append((-s * dx + c * dy) / self.resolution)
        h, w = self.data.shape
        i0, i1 = max(int(math.floor(min(gi))), 0), min(int(math.ceil(max(gi))), w - 1)
        j0, j1 = max(int(math.floor(min(gj))), 0), min(int(math.ceil(max(gj))), h - 1)
        if i1 < i0 or j1 < j0:
            empty = np.zeros(0)
            return empty, empty, np.zeros(0, dtype=np.int16)
        ii, jj = np.meshgrid(np.arange(i0, i1 + 1), np.arange(j0, j1 + 1))
        gx = (ii + 0.5) * self.resolution
        gy = (jj + 0.5) * self.resolution
        xs = self.origin_x + c * gx - s * gy
        ys = self.origin_y + s * gx + c * gy
        return xs.ravel(), ys.ravel(), self.data[jj, ii].ravel()


#: Result of a footprint check. free: no occupied cell touches the footprint;
#: first_blocked_s: arc length [m] of the first pose that touches one (None if
#: free); unknown_cells: how many distinct UNKNOWN cells the footprint touched
#: (counted as free, reported so the caller can judge how much was a guess).
FreeCheck = namedtuple('FreeCheck', 'free first_blocked_s unknown_cells')


def _footprint_hits(poses, grid, front_m, rear_m, half_width_m, margin_m):
    """Return (first blocked pose index or None, unknown cell count)."""
    import numpy as np
    if grid is None or not poses:
        return None, 0
    pts = np.asarray(poses, dtype=float)
    reach = max(front_m, rear_m) + half_width_m + margin_m + grid.resolution
    xs, ys, vals = grid.cells_in_box(pts[:, 0].min() - reach, pts[:, 1].min() - reach,
                                     pts[:, 0].max() + reach, pts[:, 1].max() + reach)
    occ = vals >= grid.occupied_threshold
    unk = vals < 0
    keep = occ | unk
    if not keep.any():
        return None, 0
    xs, ys, occ, unk = xs[keep], ys[keep], occ[keep], unk[keep]
    # A cell is a square: its centre within the footprint grown by margin plus
    # half a cell diagonal touches it (conservative by up to res * 0.2).
    grow = margin_m + 0.5 * grid.resolution * math.sqrt(2.0)
    lo_x, hi_x = -rear_m - grow, front_m + grow
    lo_y, hi_y = -half_width_m - grow, half_width_m + grow
    first = None
    seen_unknown = np.zeros(xs.shape, dtype=bool)
    chunk = 32                      # poses per block: bounds the P x N temporaries
    for k in range(0, len(pts), chunk):
        blk = pts[k:k + chunk]
        c = np.cos(blk[:, 2])[:, None]
        s = np.sin(blk[:, 2])[:, None]
        dx = xs[None, :] - blk[:, 0:1]
        dy = ys[None, :] - blk[:, 1:2]
        lx = c * dx + s * dy
        ly = -s * dx + c * dy
        inside = (lx >= lo_x) & (lx <= hi_x) & (ly >= lo_y) & (ly <= hi_y)
        seen_unknown |= (inside & unk[None, :]).any(axis=0)
        if first is None:
            rows = np.nonzero((inside & occ[None, :]).any(axis=1))[0]
            if rows.size:
                first = k + int(rows[0])
    return first, int(seen_unknown.sum())


def path_is_free(path, grid, half_width, margin, step=0.05, front_m=0.405, rear_m=0.082):
    """
    Check the car's footprint along `path` against the grid's occupied cells.

    The footprint is the body rectangle in the rear-axle frame the MPC model
    uses (base_link): front_m ahead, rear_m behind, half_width to each side,
    all grown by `margin`. It is placed at every `step` metres of the path.
    Unknown cells are FREE (a room behind a doorway is mostly unknown) but
    are counted. No grid at all is free with 0 unknown cells: the caller has
    to say so itself.

    Returns FreeCheck(free, first_blocked_s, unknown_cells); it unpacks as a
    3-tuple.
    """
    if path is None:
        return FreeCheck(False, 0.0, 0)
    n = max(int(math.ceil(path.length / max(step, 1e-4))), 1)
    ss = [path.length * i / n for i in range(n + 1)]
    poses = [path.pose_at(s) for s in ss]
    first, unknown = _footprint_hits(poses, grid, front_m, rear_m, half_width, margin)
    if first is None:
        return FreeCheck(True, None, unknown)
    return FreeCheck(False, ss[first], unknown)


def straight_back_is_free(q0, distance, grid, half_width, margin, step=0.05, rear_m=0.082):
    """
    Check the strip the body sweeps backing straight up `distance` from q0.

    Only the REAR side: from the rear bumper back by `distance`, half_width +
    margin to each side (the ground the car newly covers; the rest is under
    it already). first_blocked_s is the reverse distance [m] at which the
    rear bumper (grown by margin) would first touch an occupied cell.
    """
    x0, y0, yaw = float(q0[0]), float(q0[1]), float(q0[2])
    if distance <= 0.0:
        return FreeCheck(True, None, 0)
    n = max(int(math.ceil(distance / max(step, 1e-4))), 1)
    ss = [distance * i / n for i in range(n + 1)]
    # A zero-length "front" at the rear bumper of each backed-up pose: the
    # union over the samples is the strip [rear bumper - distance, rear bumper].
    poses = [(x0 - s * math.cos(yaw), y0 - s * math.sin(yaw), yaw) for s in ss]
    first, unknown = _footprint_hits(poses, grid, -rear_m, rear_m, half_width, margin)
    if first is None:
        return FreeCheck(True, None, unknown)
    return FreeCheck(False, ss[first], unknown)
