"""Which side to pass an obstacle on -- the ONE place that decides it.

Pure: no rclpy. Used by blocked.plan_keepouts (the A3 hard keep-out rows)
and, in Part B, by the adaptive corridor (B4), so the corridor that widens
and the constraint that keeps the car out of the obstacle can never disagree
about the side. MPC_corr publishes every tick's decisions on /mpc/pass_sides.

GEOMETRY, in the corridor frame at the obstacle's station: c = the
obstacle's lateral offset from the centreline (+ left), hw = the corridor
half-width there, d = r + car_radius + margin. The REAR AXLE (what the
corridor rows bound) must sit at >= c + d to pass on the left, <= c - d to
pass on the right. A side FITS when that position is inside +-hw.

THE FIRST DECISION for an obstacle: of the sides that fit, the one nearer
the car's own lateral offset (least steering; ties -> the side with more
room, then left).

COMMITMENT (review item 2). The rule above alone can flip between ticks: c
jitters with the detection (the box centre moves, r changes with the box
width) and when the car is still near c both sides cost about the same. So
a decision is COMMITTED to the obstacle and kept while the obstacle is
tracked (an obstacle within match_radius_m of the remembered position, seen
in the last forget_after_s), whatever the costs say. It changes only when
the committed side stops fitting and the other one fits (logged as a
switch); when neither fits the obstacle is reported blocked and the
commitment is kept for when it fits again.
"""

import math
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Sequence, Tuple

LEFT = 1
RIGHT = -1


def side_name(side: Optional[int]) -> Optional[str]:
    return None if side is None else ('left' if side > 0 else 'right')


@dataclass
class SideDecision:
    """One obstacle's decision this tick (corridor frame at its station)."""

    ox: float
    oy: float
    r: float
    c: float
    d: float
    half_width: float
    nx: float
    ny: float
    s_ahead: float            # along-track distance from the car's rear axle
    fits_left: bool
    fits_right: bool
    side: Optional[int]       # LEFT / RIGHT, None when nothing fits
    reason: str               # 'new' | 'committed' | 'switched' | 'no_side_fits'

    def as_dict(self) -> Dict:
        return {'o': [round(self.ox, 3), round(self.oy, 3)], 'r': round(self.r, 3),
                'c': round(self.c, 3), 'd': round(self.d, 3),
                'half_w': round(self.half_width, 3), 's_ahead': round(self.s_ahead, 3),
                'fits': {'left': self.fits_left, 'right': self.fits_right},
                'side': side_name(self.side), 'reason': self.reason}


@dataclass
class _Commitment:
    x: float
    y: float
    side: int
    last_seen: float


@dataclass
class PassSideMemory:
    """The committed side per obstacle, matched by position."""

    match_radius_m: float = 0.5
    forget_after_s: float = 2.0
    entries: List[_Commitment] = field(default_factory=list)

    def reset(self) -> None:
        self.entries = []

    def lookup(self, x: float, y: float) -> Optional[_Commitment]:
        best, best_d = None, self.match_radius_m
        for e in self.entries:
            dist = math.hypot(e.x - x, e.y - y)
            if dist <= best_d:
                best, best_d = e, dist
        return best

    def commit(self, x: float, y: float, side: int, now: float) -> None:
        e = self.lookup(x, y)
        if e is None:
            self.entries.append(_Commitment(x, y, side, now))
        else:
            e.x, e.y, e.side, e.last_seen = x, y, side, now

    def touch(self, x: float, y: float, now: float) -> None:
        e = self.lookup(x, y)
        if e is not None:
            e.x, e.y, e.last_seen = x, y, now

    def expire(self, now: float) -> None:
        self.entries = [e for e in self.entries if now - e.last_seen <= self.forget_after_s]


def nearest_frame(px: float, py: float, corridor) -> Tuple[float, float, float, float, float]:
    """(pcx, pcy, nx, ny, half_w) of the corridor sample nearest (px, py)."""
    xc, yc = corridor['xc'], corridor['yc']
    best, best_d2 = 0, math.inf
    for i in range(len(xc)):
        d2 = (float(xc[i]) - px) ** 2 + (float(yc[i]) - py) ** 2
        if d2 < best_d2:
            best_d2, best = d2, i
    return (float(xc[best]), float(yc[best]), float(corridor['nx'][best]),
            float(corridor['ny'][best]), float(corridor['halfWidth'][best]))


def decide_sides(obstacles: Sequence[Tuple[float, float, float]], corridor,
                 car_xy: Tuple[float, float], car_radius: float, margin: float,
                 relevant, memory: PassSideMemory, now: float,
                 ) -> Tuple[List[SideDecision], List[Dict]]:
    """Decide (and commit) a side for every obstacle `relevant` keeps.

    relevant(s_ahead, c, d, half_w) -> None to decide, or a skip reason
    string (kept in the second return value, for the log). Returns
    (decisions, skipped).
    """
    memory.expire(now)
    decisions: List[SideDecision] = []
    skipped: List[Dict] = []
    if not obstacles:
        return decisions, skipped
    cx, cy = float(car_xy[0]), float(car_xy[1])
    pcx_c, pcy_c, nx_c, ny_c, _ = nearest_frame(cx, cy, corridor)
    y_car = nx_c * (cx - pcx_c) + ny_c * (cy - pcy_c)
    for ox, oy, r in obstacles:
        ox, oy, r = float(ox), float(oy), float(r)
        pcx, pcy, nx, ny, hw = nearest_frame(ox, oy, corridor)
        tx, ty = ny, -nx  # build_straight_corridor: n = (-ty, tx)
        d = r + float(car_radius) + float(margin)
        c = nx * (ox - pcx) + ny * (oy - pcy)
        s_ahead = tx * (ox - cx) + ty * (oy - cy)
        skip = relevant(s_ahead, c, d, hw)
        if skip:
            skipped.append({'o': [round(ox, 3), round(oy, 3)], 'r': round(r, 3),
                            'c': round(c, 3), 's_ahead': round(s_ahead, 3), 'skip': skip})
            continue
        fits_left = c + d <= hw + 1e-9
        fits_right = c - d >= -hw - 1e-9
        committed = memory.lookup(ox, oy)
        if not (fits_left or fits_right):
            side, reason = None, 'no_side_fits'
            memory.touch(ox, oy, now)
        elif committed is not None:
            fits = fits_left if committed.side == LEFT else fits_right
            if fits:
                side, reason = committed.side, 'committed'
            else:
                side, reason = -committed.side, 'switched'
            memory.commit(ox, oy, side, now)
        else:
            if fits_left and fits_right:
                cost_left = abs((c + d) - y_car)
                cost_right = abs((c - d) - y_car)
                if abs(cost_left - cost_right) > 1e-6:
                    side = LEFT if cost_left < cost_right else RIGHT
                else:
                    side = LEFT if hw - (c + d) >= (c - d) + hw else RIGHT
            else:
                side = LEFT if fits_left else RIGHT
            reason = 'new'
            memory.commit(ox, oy, side, now)
        decisions.append(SideDecision(ox, oy, r, c, d, hw, nx, ny, s_ahead,
                                      fits_left, fits_right, side, reason))
    return decisions, skipped
