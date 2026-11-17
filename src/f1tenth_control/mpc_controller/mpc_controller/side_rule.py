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
