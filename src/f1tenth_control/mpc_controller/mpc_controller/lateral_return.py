"""
lateral_return.py: the strong return onto the move's line after a swerve (mpc-dodge 2b). Pure.

THE PROBLEM. On a straight (goal_distance) move the corridor is rebuilt through the car's CURRENT
position every rebuild ("THE PRICE" in MPC_corr.build_straight_corridor): after an obstacle the car
carries its sideways offset (1.3-1.6 m round a D0.5 in Isaac) for the rest of the move, because the
centreline -- and the lookahead target on it -- moved sideways with the car and w_corr has nothing
to act on. Raising w_corr alone cannot help.

WHAT THE RE-ANCHORING PROTECTED. Its own note: a line anchored in MAP and reprojected into odom
every tick steps whenever a SLAM correction lands in map -> odom, and the corridor jittered. The
heading drift the frozen heading fixes is a map-frame correction and is kept. A line held in the
LOCAL ODOM frame has no map -> odom step in it at all; what it gives up is that an odom POSITION
drift over one move moves the line with the car (EKF wheel + IMU over a few metres: small, but
unmeasured here -- see the report).

THE FIX, behind corridor_lateral_reanchor (true = today's behaviour, the default). False: the
move's LINE stays fixed in odom from the move's start (origin = the car's odom position at the
move's first corridor, direction = the move's frozen, map-corrected heading). Each rebuild plans
from the lateral error e of the car from that line (plan()):
  * |e| < E_ON_M: 'fixed' -- the straight corridor along the line from the foot of the car's
    perpendicular (a rebuild advances it ALONG the line only), its half-width grown to contain the
    car;
  * an obstacle between the car and the line inside the corridor's reach: 'hold' -- today's
    corridor through the car, no return push towards an obstacle (w_obs stays in charge);
  * otherwise, with mpc_return_lookahead_ratio > 0: 'arc' -- a RETURN ARC, the pose corridor
    (corridor_curves_to_pose, the cubic the goal_pose branch uses) from the car's pose to the
    line point D ahead of the foot, arriving along the line. D = max(L_min, ratio * |e|), the
    pure-pursuit lookahead: about atan(1/ratio) of approach, never steeper than the heading cap
    (mpc_return_max_heading_deg, and the angle whose levelling-off at full lock, R_min (1 - cos),
    fits inside mpc_return_overshoot_max_m). An infeasible or over-long arc falls back to 'hold'
    for that rebuild. With ratio 0 the line is 'fixed' at any error (w_corr does the return).
  The centreline, the target on it, w_corr and the stage heading cost then all agree on the
  return; per tick (shape()) w_corr_eff = w_corr * (1 + mpc_return_gain * min(1, |e| / E_SAT_M))
  holds the car tighter to it and the terminal / stage heading weights fall to
  mpc_return_psi_scale at a large error (back to 1 near the line), so the arc's crossing angle is
  not fought by a heading cost aimed along the line.
"""
from __future__ import annotations

import math
from typing import NamedTuple, Optional, Sequence, Tuple

#: Lateral error [m] at which the per-tick shaping reaches full strength.
E_SAT_M = 0.5
#: Below this |e| [m] there is no return to plan.
E_ON_M = 0.10
#: An arc lands at most this fraction of the corridor cap ahead (the pose corridor is longer than
#: its chord and is rejected past the cap).
ARC_D_FRACTION = 0.8

MODES = ('fixed', 'arc', 'hold')


class ReturnShape(NamedTuple):
    """This tick's weight scaling of the return."""

    w_corr_scale: float
    w_psi_scale: float
    active: bool
    blocked: bool
    e: float


def lateral_offset(p: Tuple[float, float], origin: Tuple[float, float], psi: float) -> float:
    """
    Return the signed distance of p from the line through origin along psi, left positive.

    Signed distance of p from the line through origin along psi, left positive.
    """
    return -math.sin(psi) * (p[0] - origin[0]) + math.cos(psi) * (p[1] - origin[1])


def foot_on_line(p, origin, psi):
    """
    Return the point of the line nearest p.

    The point of the line nearest p.
    """
    c, s = math.cos(psi), math.sin(psi)
    a = c * (p[0] - origin[0]) + s * (p[1] - origin[1])
    return origin[0] + a * c, origin[1] + a * s


def max_return_heading(max_heading_deg: float, overshoot_max_m: float,
                       r_min: Optional[float]) -> float:
    """
    Return the approach-angle cap [rad].

    The approach-angle cap [rad]: the configured one, and the one whose levelling-off at full
    lock fits in the allowed overshoot (R_min (1 - cos theta) <= overshoot_max).
    """
    cap = math.radians(max(1.0, min(89.0, max_heading_deg)))
    if r_min and r_min > 0.0 and overshoot_max_m > 0.0:
        c = 1.0 - min(1.0, overshoot_max_m / r_min)
        cap = min(cap, math.acos(max(-1.0, min(1.0, c))))
    return cap


def obstacle_in_return_band(e: float, car_xy, psi: float, obstacles: Sequence, reach: float,
                            clearance: float) -> bool:
    """
    Tell whether an obstacle ahead reaches the band between the car and the line.

    An obstacle ahead (within reach) whose disc reaches the band between the car and the line.
    """
    c, s = math.cos(psi), math.sin(psi)
    lo, hi = min(e, 0.0), max(e, 0.0)
    for ox, oy, r in obstacles:
        dx, dy = ox - car_xy[0], oy - car_xy[1]
        along = c * dx + s * dy
        lat = e + (-s * dx + c * dy)            # obstacle's offset from the LINE
        if -clearance - r < along <= reach and lo - r - clearance < lat < hi + r + clearance:
            return True
    return False


def plan(e: float, ratio: float, lookahead_min: float, max_heading_rad: float, length_cap: float,
         blocked: bool) -> Tuple[str, Optional[float], float]:
    """
    Return (mode, D, target_offset): how this rebuild's corridor returns.

    (mode, D, target_offset): how this rebuild's corridor returns (see the module docstring).

    The arc lands D ahead of the foot at lateral target_offset (left positive): on the line (0)
    when ratio * |e| fits in the corridor cap, else as far towards it as the approach angle allows
    over the D that fits -- the next rebuild, a second later, re-aims from wherever the car is, so
    the car follows a pure pursuit made of arcs and never actually flattens out at the
    intermediate offset.
    """
    if abs(e) < E_ON_M:
        return 'fixed', None, 0.0
    if blocked:
        return 'hold', None, e
    if ratio <= 0.0:
        return 'fixed', None, 0.0
    ratio_eff = max(ratio, 1.0 / math.tan(max_heading_rad))
    d = min(max(lookahead_min, ratio_eff * abs(e)), ARC_D_FRACTION * length_cap)
    step = min(abs(e), d / ratio_eff)
    return 'arc', d, math.copysign(abs(e) - step, e)


def shape(e: float, gain: float, psi_scale: float, blocked: bool) -> ReturnShape:
    """
    Return this tick's weight scaling for lateral error e (see the module docstring).

    This tick's weight scaling for lateral error e (see the module docstring).
    """
    ae = abs(e)
    if blocked or ae < E_ON_M:
        return ReturnShape(1.0, 1.0, False, blocked, e)
    f = min(1.0, ae / E_SAT_M)
    w_corr_scale = 1.0 + max(0.0, gain) * f
    w_psi_scale = 1.0 - (1.0 - min(1.0, max(0.0, psi_scale))) * f
    return ReturnShape(w_corr_scale, w_psi_scale, True, False, e)
