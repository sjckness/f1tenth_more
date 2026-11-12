"""Point-to-point geometric paths for a car that only drives forward: p2p_geometric's planner.

Pure: numpy and math only. Two path families, both made of circular arcs of
the car's minimum turning radius R and straight segments:

* to_point(): FREE end heading. The shortest forward path to a point is
  turn-then-straight (CS): turn on the circle toward the point until the point
  is dead ahead, then drive straight at it. If the point lies INSIDE that
  circle it cannot be reached that way, and the Dubins path to the point with
  the end heading of the start-to-point bearing is used instead.
* to_pose(): FIXED end heading, the shortest of the six Dubins words (LSL,
  RSR, LSR, RSL, RLR, LRL).

Every candidate is integrated and checked to end where it was asked to; a
word whose closed-form solution does not land on the goal (numerically
degenerate geometry) is discarded rather than trusted.
"""

import math
from typing import List, Optional, Tuple

from f1tenth_planners.geometry import wrap_pi
import numpy as np

#: A path segment: kind 'L' (left arc), 'R' (right arc) or 'S' (straight), and
#: its length in METRES (an arc's length is R times its angle).
Segment = Tuple[str, float]


def _mod2pi(a: float) -> float:
    # A value a hair below 0 (or 2 pi) is a zero-length arc, not a full loop.
    v = a % (2.0 * math.pi)
    return 0.0 if v > 2.0 * math.pi - 1e-7 else v


def _words(alpha: float, beta: float, d: float):
    """Yield the six Dubins words in normalised form (R = 1) as (word, t, p, q)."""
    sa, sb, ca, cb = math.sin(alpha), math.sin(beta), math.cos(alpha), math.cos(beta)
    cab = math.cos(alpha - beta)

    p2 = 2.0 + d * d - 2.0 * cab + 2.0 * d * (sa - sb)
    if p2 >= 0.0:
        tmp = math.atan2(cb - ca, d + sa - sb)
        yield 'LSL', _mod2pi(-alpha + tmp), math.sqrt(p2), _mod2pi(beta - tmp)

    p2 = 2.0 + d * d - 2.0 * cab + 2.0 * d * (sb - sa)
    if p2 >= 0.0:
        tmp = math.atan2(ca - cb, d - sa + sb)
        yield 'RSR', _mod2pi(alpha - tmp), math.sqrt(p2), _mod2pi(-beta + tmp)

    p2 = -2.0 + d * d + 2.0 * cab + 2.0 * d * (sa + sb)
    if p2 >= 0.0:
        p = math.sqrt(p2)
        tmp = math.atan2(-ca - cb, d + sa + sb) - math.atan2(-2.0, p)
        yield 'LSR', _mod2pi(-alpha + tmp), p, _mod2pi(-beta + tmp)

    p2 = d * d - 2.0 + 2.0 * cab - 2.0 * d * (sa + sb)
    if p2 >= 0.0:
        p = math.sqrt(p2)
        tmp = math.atan2(ca + cb, d - sa - sb) - math.atan2(2.0, p)
        yield 'RSL', _mod2pi(alpha - tmp), p, _mod2pi(beta - tmp)

    tmp = (6.0 - d * d + 2.0 * cab + 2.0 * d * (sa - sb)) / 8.0
    if abs(tmp) <= 1.0:
        p = _mod2pi(2.0 * math.pi - math.acos(tmp))
        t = _mod2pi(alpha - math.atan2(ca - cb, d - sa + sb) + p / 2.0)
        yield 'RLR', t, p, _mod2pi(alpha - beta - t + p)

    tmp = (6.0 - d * d + 2.0 * cab + 2.0 * d * (-sa + sb)) / 8.0
    if abs(tmp) <= 1.0:
        p = _mod2pi(2.0 * math.pi - math.acos(tmp))
        t = _mod2pi(-alpha + math.atan2(-ca + cb, d + sa - sb) + p / 2.0)
        yield 'LRL', t, p, _mod2pi(-alpha + beta - t + p)


def integrate(start, segments: List[Segment], radius: float, ds: float = 0.05) -> np.ndarray:
    """Sample a segment list from ``start`` (x, y, yaw); returns Nx3 (x, y, yaw)."""
    x, y, yaw = (float(v) for v in start)
    out = [(x, y, yaw)]
    for kind, length in segments:
        if length <= 0.0:
            continue
        n = max(int(math.ceil(length / ds)), 1)
        h = length / n
        for _ in range(n):
            if kind == 'S':
                x += h * math.cos(yaw)
                y += h * math.sin(yaw)
            else:
                sign = 1.0 if kind == 'L' else -1.0
                dyaw = sign * h / radius
                x += radius * sign * (math.sin(yaw + dyaw) - math.sin(yaw))
                y += radius * sign * (-math.cos(yaw + dyaw) + math.cos(yaw))
                yaw += dyaw
            out.append((x, y, yaw))
    return np.array(out)


def _end_pose(start, segments: List[Segment], radius: float):
    end = integrate(start, segments, radius, ds=max(radius, 1e-3) * 0.02)[-1]
    return float(end[0]), float(end[1]), float(end[2])


def to_pose(start, goal, radius: float) -> Optional[List[Segment]]:
    """Shortest forward Dubins path from ``start`` to ``goal`` (both (x, y, yaw))."""
    dx, dy = goal[0] - start[0], goal[1] - start[1]
    d = math.hypot(dx, dy) / radius
    phi = math.atan2(dy, dx)
    alpha = _mod2pi(start[2] - phi)
    beta = _mod2pi(goal[2] - phi)
    best, best_len = None, math.inf
    tol = max(1e-3, 1e-3 * radius)
    for word, t, p, q in _words(alpha, beta, d):
        length = (t + p + q) * radius
        if length >= best_len:
            continue
        segments = [(word[0], t * radius), (word[1], p * radius), (word[2], q * radius)]
        ex, ey, eyaw = _end_pose(start, segments, radius)
        if (math.hypot(ex - goal[0], ey - goal[1]) > 5.0 * tol
                or abs(wrap_pi(eyaw - goal[2])) > 0.01):
            continue
        best, best_len = segments, length
    return best


def to_point(start, goal_xy, radius: float) -> List[Segment]:
    """Shortest forward path to a point, end heading free; see the module docstring."""
    x, y, yaw = (float(v) for v in start)
    gx, gy = float(goal_xy[0]), float(goal_xy[1])
    best, best_len = None, math.inf
    for kind, sign in (('L', 1.0), ('R', -1.0)):
        cx = x - sign * radius * math.sin(yaw)
        cy = y + sign * radius * math.cos(yaw)
        dc = math.hypot(gx - cx, gy - cy)
        if dc < radius - 1e-9:
            continue
        straight = math.sqrt(max(dc * dc - radius * radius, 0.0))
        # Heading of the tangent line from the circle to the goal.
        base = math.atan2(gy - cy, gx - cx)
        tangent_heading = base + sign * math.asin(min(radius / dc, 1.0)) if dc > 0 else yaw
        arc = _mod2pi(sign * (tangent_heading - yaw))
        length = arc * radius + straight
        if length < best_len:
            best, best_len = [(kind, arc * radius), ('S', straight)], length
    if best is not None:
        ex, ey, _ = _end_pose(start, best, radius)
        if math.hypot(ex - gx, ey - gy) <= max(0.02, 0.02 * radius):
            return best
    # Inside both turning circles: aim the end heading along the start-to-goal
    # bearing; the Dubins path to that pose loops out and comes back.
    bearing = math.atan2(gy - y, gx - x)
    fallback = to_pose((x, y, yaw), (gx, gy, bearing), radius)
    return fallback if fallback is not None else [('S', math.hypot(gx - x, gy - y))]


def path_length(segments: List[Segment]) -> float:
    """Total length [m] of a segment list."""
    return float(sum(length for _, length in segments))
