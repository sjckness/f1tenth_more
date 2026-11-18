"""
obstacle_motion.py: moving obstacles in the QP -- predicted discs per stage and the wake. Pure.

mpc-dodge, 2026-11-18 (Andreas's work order, phase 2). Until now every tick handed the solver a
PHOTOGRAPH: the same static discs at every stage of the horizon. A cylinder crossing the car's path
was avoided towards where it was going (Isaac M1: dodged left, the cylinder kept going left,
contact). The semantic layer now estimates each track's ground velocity in odom (f1tenth_costmap
track_velocity.py) and puts it, with the track id, on every Obstacle2D. Two separate knobs use it,
both OFF by default so the QP is today's, operation for operation:

1. PREDICTED DISCS, trust alpha (mpc_obs_vel_trust, 0..1). The disc of a v_valid obstacle at stage
   k is centred at  p + alpha * v * t_k,  t_k the time of the stage's state (control_period + k *
   ts, plus mpc_obs_vel_lead_s for the pipeline latency, capped at mpc_obs_pred_max_s: a constant
   velocity is not a forecast four seconds out). alpha 0 is the static MPC, 1 full constant
   velocity; in between it trusts a noisy estimate less. Nothing else changes: same soft w_obs
   cost, same margins. Passing BEHIND a crossing obstacle becomes cheaper on its own, because the
   predicted disc moves away from that side.

2. THE WAKE (mpc_obs_wake_weight, mpc_obs_wake_horizon_s ~ 1 s). An extra soft cost on horizon
   states inside a half-ellipse AHEAD of the obstacle's motion: from its (predicted) centre forward
   along v, semi-major |v| * T, semi-minor r (the car's R = car_radius + margin is added by the
   cost itself, so the width is r + margin). Drawn as WAKE_DISCS discs whose radii follow the
   ellipse, charged with their own weight through the same softplus-squared term as w_obs. It says
   "not where it is going", never "go the other way": no term pushes the car in a direction; the
   solver picks the side. Only for an obstacle APPROACHING the car (v . (p - car) < 0): one moving
   away adds nothing. Head-on (no lateral velocity) it gives no side either; the side rule decides.
"""
from __future__ import annotations

import math
from typing import List, NamedTuple, Optional, Sequence, Tuple

Circle = Tuple[float, float, float]

#: Discs drawing the half-ellipse of a wake.
WAKE_DISCS = 4


class Motion(NamedTuple):
    """The motion of one solver obstacle (odom frame)."""

    track_id: int
    vx: float
    vy: float
    valid: bool


def stage_times(horizon: int, ts: float, control_period: Optional[float] = None,
                lead_s: float = 0.0, max_s: float = math.inf) -> List[float]:
    """
    Return the time from now of the state each stage's obstacle term is evaluated at.

    Time from now of the state each stage's obstacle term is evaluated at (x_{k+1}).

    Stage 0 spans the control period, every later one the prediction step ts (mpc_solver's
    convention, see rate_weight_factor). lead_s is added to every stage, max_s caps them.
    """
    t_first = ts if control_period is None else float(control_period)
    return [min(max(0.0, lead_s + t_first + k * ts), max_s) for k in range(int(horizon))]


def predicted_stage_obstacles(obstacles: Sequence[Circle], motions: Sequence[Optional[Motion]],
                              times: Sequence[float], trust: float) -> List[List[Circle]]:
    """
    Return, per stage, every obstacle at p + trust * v * t_k (only v_valid ones move).

    Per stage, every obstacle at p + trust * v * t_k (only v_valid ones move).
    """
    out = []
    for t in times:
        stage = []
        for (x, y, r), m in zip(obstacles, motions):
            if m is not None and m.valid and trust > 0.0:
                stage.append((x + trust * m.vx * t, y + trust * m.vy * t, r))
            else:
                stage.append((x, y, r))
        out.append(stage)
    return out


def approaching(obstacle: Circle, motion: Optional[Motion], car_xy: Tuple[float, float]) -> bool:
    """
    Tell whether a v_valid obstacle's velocity points towards the car.

    True when a v_valid obstacle's velocity points towards the car (v . (p - car) < 0).
    """
    if motion is None or not motion.valid:
        return False
    dx, dy = obstacle[0] - car_xy[0], obstacle[1] - car_xy[1]
    return motion.vx * dx + motion.vy * dy < 0.0


def wake_discs(centre: Tuple[float, float], r: float, motion: Motion, horizon_s: float,
               n: int = WAKE_DISCS) -> List[Circle]:
    """
    Return the half-ellipse ahead of a moving obstacle as n discs.

    The half-ellipse ahead of a moving obstacle as n discs (semi-major |v| T, semi-minor r).
    """
    speed = math.hypot(motion.vx, motion.vy)
    length = speed * horizon_s
    if length <= 1e-6 or r <= 0.0:
        return []
    ux, uy = motion.vx / speed, motion.vy / speed
    out = []
    for j in range(1, n + 1):
        f = j / (n + 0.5)
        rj = r * math.sqrt(max(0.0, 1.0 - f * f))
        out.append((centre[0] + ux * f * length, centre[1] + uy * f * length, rj))
    return out


def stage_wake_obstacles(obstacles: Sequence[Circle], motions: Sequence[Optional[Motion]],
                         stage_obstacles: Sequence[Sequence[Circle]], car_xy: Tuple[float, float],
                         horizon_s: float) -> List[List[Circle]]:
    """
    Return, per stage, the wake discs of every approaching v_valid obstacle.

    Per stage, the wake discs of every approaching v_valid obstacle, from its stage centre.
    """
    movers = [i for i, (o, m) in enumerate(zip(obstacles, motions)) if approaching(o, m, car_xy)]
    out = []
    for stage in stage_obstacles:
        discs: List[Circle] = []
        for i in movers:
            x, y, r = stage[i]
            discs.extend(wake_discs((x, y), r, motions[i], horizon_s))
        out.append(discs)
    return out
