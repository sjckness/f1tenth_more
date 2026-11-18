"""
The side rule: which way to go round an obstacle that sits on the car's own line. Pure Python.

THE PROBLEM. The MPC's target is always the centreline point and avoidance is the solver's soft
obstacle cost alone. An obstacle exactly on the line ahead of a car aligned with it makes the QP
mirror-symmetric: the obstacle cost has no lateral gradient, the solver has no side to prefer, and
the cheapest plan is to slow down in front of the obstacle -- the car stops (measured closed loop,
test_centerline_target_closed_loop.py). Real detections are never exactly centred, but a few
centimetres of noise is a coin flip, and a coin flip can land differently on the next tick.

THE RULE (a deliberate bias, decided 2026-11-18). An obstacle AHEAD whose centre is within `band`
of the line through the car along the move's heading is handed to the solver displaced to
lateral -side * band (opposite the preferred `side`: +1 left, -1 right), with its radius grown
by the displacement, so the displaced disc still contains the real one: the car never gets less
clearance than without the rule, it only gets a side. Obstacles outside the band are untouched:
the geometry already chooses for them. The car's own swerve towards `side` moves the obstacle
out of the band on the far side, where the displaced and the real disc coincide (continuous).
"""
from __future__ import annotations

import math
from typing import List, Sequence, Tuple

SIDES = {'left': 1.0, 'right': -1.0}


def side_sign(side: str) -> float:
    """Return +1 for left, -1 for right."""
    try:
        return SIDES[str(side)]
    except KeyError:
        raise ValueError(f"obstacle tie side must be 'left' or 'right', got {side!r}") from None


def break_ties(obstacles: Sequence[Tuple[float, float, float]], car_xy: Tuple[float, float],
               heading: float, band: float, side: float, reach: float
               ) -> Tuple[List[Tuple[float, float, float]], int]:
    """
    Return (obstacles with the near-line ones displaced, how many were displaced).

    `obstacles` are (x, y, r) in the same frame as `car_xy`; `heading` is the move's direction
    in that frame; only obstacles 0 < along <= `reach` ahead are considered. `band` <= 0 turns
    the rule off.
    """
    if band <= 0.0 or not obstacles:
        return list(obstacles), 0
    c, s = math.cos(heading), math.sin(heading)
    out, n = [], 0
    for ox, oy, r in obstacles:
        dx, dy = ox - car_xy[0], oy - car_xy[1]
        along = c * dx + s * dy
        lat = -s * dx + c * dy
        if 0.0 < along <= reach and abs(lat) < band:
            target = -side * band
            shift = target - lat
            out.append((ox - s * shift, oy + c * shift, r + abs(shift)))
            n += 1
        else:
            out.append((ox, oy, r))
    return out, n


# ---------------------------------------------------------------- the INFORMED rule (mpc-dodge)
#
# mpc_obstacle_side_mode 'informed' (default 'fixed' = break_ties above, unchanged). Same band and
# the same displacement (never less clearance, only a side), but the side of an obstacle in the
# band is chosen, in order, from:
#   1. its lateral velocity, when |v_lat| > v_min: the side OPPOSITE the sign of v_lat -- pass
#      behind it, on the side it is leaving (Isaac M1: the car dodged the way the cylinder was
#      going and met it there);
#   2. standing still: the side it CAME FROM, if its track remembers one (the side it last stood
#      on outside the band): that side has just been freed;
#   3. otherwise the side with more free space (walls of the costmap, other obstacles beside it);
#   4. last resort: the fixed side (mpc_obstacle_tie_side).
# LATCH: the side chosen for a track is kept while that track stays within `release` of the line
# (hysteresis: band < release), for the rest of the move; a new move clears every latch. Today's
# rule has no memory, so two ticks could pick two sides. An obstacle without a track id (an older
# publisher) is decided on every tick, as before.

SIDE_REASONS = ('velocity', 'came_from', 'space', 'fixed', 'latched')


class SideLatch:
    """Per-track memory of the informed rule: the latched side, the side a track came from."""

    def __init__(self):
        """Start with no latch and no remembered side."""
        self.sides = {}
        self.came_from = {}

    def reset(self):
        """Forget every latch and remembered side (a new move)."""
        self.sides.clear()
        self.came_from.clear()


def choose_side(v_lat, came_from, free_left, free_right, fallback: float, v_min: float,
                space_tol: float = 0.05):
    """
    Return (side, reason): +1 left / -1 right, by the informed rule's order.

    (side, reason): +1 left / -1 right, by the informed rule's order (see above).
    """
    if v_lat is not None and abs(v_lat) > v_min:
        return (-1.0 if v_lat > 0.0 else 1.0), 'velocity'
    if came_from is not None:
        return float(came_from), 'came_from'
    if (free_left is not None and free_right is not None
            and abs(free_left - free_right) > space_tol):
        return (1.0 if free_left > free_right else -1.0), 'space'
    return float(fallback), 'fixed'


def break_ties_informed(obstacles, motions, car_xy, heading, band, release, fallback, reach,
                        latch: SideLatch, v_min=0.1, free_space=None):
    """
    Return (obstacles with the near-line ones displaced, [(index, side, reason)]).

    `motions[i]` is obstacle i's obstacle_motion.Motion or None; `free_space(along, lat)` ->
    (free_left, free_right) metres or (None, None). The displacement is break_ties': the disc goes
    to lateral -side * band with its radius grown by the shift, so it contains the real one.
    """
    if band <= 0.0 or not obstacles:
        return list(obstacles), []
    c, s = math.cos(heading), math.sin(heading)
    out, decided = [], []
    for i, (ox, oy, r) in enumerate(obstacles):
        m = motions[i] if motions is not None and i < len(motions) else None
        tid = m.track_id if m is not None and m.track_id >= 0 else None
        dx, dy = ox - car_xy[0], oy - car_xy[1]
        along = c * dx + s * dy
        lat = -s * dx + c * dy
        if tid is not None and abs(lat) >= band:
            latch.came_from[tid] = 1.0 if lat > 0.0 else -1.0
        latched = tid is not None and tid in latch.sides
        if latched and abs(lat) >= release:
            del latch.sides[tid]
            latched = False
        ahead = 0.0 < along <= reach
        if not ahead or not (abs(lat) < band or latched):
            out.append((ox, oy, r))
            continue
        if latched:
            side, reason = latch.sides[tid], 'latched'
        else:
            v_lat = (-s * m.vx + c * m.vy) if (m is not None and m.valid) else None
            fl, fr = free_space(along, lat) if free_space is not None else (None, None)
            came = latch.came_from.get(tid) if tid is not None else None
            side, reason = choose_side(v_lat, came, fl, fr, fallback, v_min)
            if tid is not None:
                latch.sides[tid] = side
        decided.append((i, side, reason))
        if lat * side <= -band:
            out.append((ox, oy, r))           # already beyond the band on the far side
            continue
        shift = -side * band - lat
        out.append((ox - s * shift, oy + c * shift, r + abs(shift)))
    return out, decided
