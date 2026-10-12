"""Speed floor, obstacle keep-out and the BLOCKED detector (M02 campaign fix A3).

Pure: no rclpy. MPC_corr.control_loop calls these once per tick.

WHAT R011 DID. A 0.3 m plant appeared centred in the corridor ~1.4 m ahead.
The solver steered to full left lock, could not get past inside the 0.87 m
corridor, and settled on a = 0.9 m/s^2 from rest: v + a * 0.1 = 0.09 m/s,
below the ~0.12-0.14 m/s the car needs to move. Nothing noticed that the car
was not moving; the move's 80 s timeout was the only exit, and the operator
walking up ended it first.

THREE RULES replace that:

1. THE OPERATING FLOOR ON EVERY MOVE (gate_speed). A moving move publishes
   max(v_cmd, min_moving_speed_mps); zero comes only from the stop paths
   (goal reached, hold, BLOCKED, an infeasible tick). The QP is told the same
   thing: v_k >= the floor from the first stage the accel limits can reach it
   (first_floor_stage), so the plan it returns is one driven at >= 0.4.

2. A HARD KEEP-OUT PER OBSTACLE (plan_keepouts). The obstacle cost in the QP
   is soft, and for an obstacle dead on the line its lateral gradient is ~0:
   the nominal-model rig drives straight through a centred plant that appears
   late. A keep-out is a hard row on the rear axle's lateral offset, on the
   stages whose along-track position overlaps the obstacle, on ONE side:
   lateral >= c + d (pass left) or <= c - d (pass right), with
   d = r + car_radius + margin. Without it "no feasible solution at 0.4 m/s"
   would mean nothing -- a soft cost is always feasible.

3. BLOCKED, never a crawl (BlockedDetector). Any of, while a move is active:
   - no_progress: commanding >= the floor, odom path < min_progress_m over
     progress_window_s;
   - steering_lock_no_progress: a non-turn move held at >= lock_fraction of
     the steering bound for lock_window_s with < min_progress_m made along
     the corridor heading;
   - no_feasible_plan: the solve failed, no side of an obstacle fits in the
     corridor, or the >= 0.4 plan's body overlaps an obstacle, continuously
     for infeasible_s. Each such tick already commands zero.
   The node then stops, holds, latches BLOCKED for the move and tells the
   mission (/mpc/blocked), which ends it. Reverse stays disabled.
"""

import math
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Sequence, Tuple

from mpc_controller.pass_side import PassSideMemory, decide_sides

__all__ = [
    'BodyBox',
    'BlockedDetector',
    'BlockedVerdict',
    'WallDistanceWait',
    'KeepOut',
    'KeepOutPlan',
    'first_floor_stage',
    'footprint_circles',
    'gate_speed',
    'plan_body_gap',
    'plan_keepouts',
]

REASON_NO_PROGRESS = 'no_progress'
REASON_LOCK = 'steering_lock_no_progress'
REASON_INFEASIBLE = 'no_feasible_plan'


def gate_speed(v_cmd: float, min_moving_speed: float) -> float:
    """The published speed of a MOVING move: never in (0, min_moving_speed).

    A non-finite command is a stop. Everything else is raised to the floor;
    the top of the range is _publish_drive's clamp.
    """
    v_cmd = float(v_cmd)
    if not math.isfinite(v_cmd):
        return 0.0
    return max(v_cmd, float(min_moving_speed))


def first_floor_stage(v0: float, a_prev: float, v_floor: float, a_max: float,
                      da_max: float, dt: float, dt_first: float,
                      horizon: int) -> Optional[int]:
    """First stage index k (state x_k, 1..N) the model can be at >= v_floor.

    Full throttle from (v0, a_prev) under the accel bound a_max and the
    accel-rate bound da_max (per second; dt_first for the first step, dt
    after -- the same spans as the QP's rate rows). None when the floor is
    out of reach within the horizon (then no floor row is added at all: a
    row nothing can satisfy makes the QP infeasible for the wrong reason).
    """
    v = float(v0)
    a = float(a_prev)
    for k in range(int(horizon)):
        span = dt_first if k == 0 else dt
        a = min(float(a_max), a + float(da_max) * span)
        v = v + a * dt
        if v >= float(v_floor) - 1e-9:
            return k + 1
    return None


@dataclass(frozen=True)
class BodyBox:
    """The car rectangle around the rear axle (the MPC state point)."""

    front: float = 0.443
    rear: float = -0.082
    half_width: float = 0.136


def footprint_circles(body: 'BodyBox', n: int = 3) -> Tuple[Tuple[float, float], ...]:
    """n circles covering the body rectangle: (offset ahead of the rear axle, radius).

    The rectangle [rear, front] x [-half_width, half_width] is cut into n equal
    slices along its length; each slice's circumscribed circle covers it, so
    the union covers the whole body, the front bumper corners included. With
    the URDF box (0.525 m x 0.272 m) and n = 3: centres at +0.006, +0.181,
    +0.356 m, radius 0.162 m.
    """
    length = float(body.front) - float(body.rear)
    half = length / (2.0 * n)
    radius = math.hypot(half, float(body.half_width))
    return tuple((float(body.rear) + half * (2 * i + 1), radius) for i in range(n))


@dataclass(frozen=True)
class KeepOut:
    """One obstacle's hard lateral rows, as mpc_solver._solve_rti consumes them.

    Without circles (the A3 form): stage k gets
        side * (n . p_k) >= side * (n . o) + d
    on the REAR AXLE p_k when t . (p_k - o) lies in [s_lo, s_hi].
    With circles (review item 1, the shipped form): one row per footprint
    circle (offset o_i ahead of the axle, radius rc_i) on the circle CENTRE
    p_k + o_i * (cos psi_k, sin psi_k) -- linearised in psi at the stage's
    reference heading --
        side * (n . c_ik) >= side * (n . o) + r + rc_i + margin
    when t . (c_ik - o) lies in [-(r + rc_i + margin), r + rc_i + margin].
    side +1 = the car passes on the obstacle's LEFT, -1 = its right.
    """

    ox: float
    oy: float
    nx: float
    ny: float
    tx: float
    ty: float
    s_lo: float
    s_hi: float
    d: float
    side: int
    r: float = 0.0
    margin: float = 0.0
    circles: Tuple[Tuple[float, float], ...] = ()

    def as_dict(self) -> Dict:
        return {'o': (self.ox, self.oy), 'n': (self.nx, self.ny),
                't': (self.tx, self.ty), 's_lo': self.s_lo, 's_hi': self.s_hi,
                'd': self.d, 'side': self.side, 'r': self.r, 'margin': self.margin,
                'circles': [list(c) for c in self.circles]}


@dataclass
class KeepOutPlan:
    keepouts: List[KeepOut] = field(default_factory=list)
    # Obstacles in the way with no side that fits inside the corridor.
    blocked: List[Dict] = field(default_factory=list)
    # One record per obstacle considered, for the log and /mpc/pass_sides.
    report: List[Dict] = field(default_factory=list)


def keepout_relevance(car_radius: float, margin: float, body: BodyBox, reach_m: float):
    """
    Return relevant(s_ahead, c, d, half_w) -> None (in the way) or a skip reason.

    In the way: ahead of the car's rear bumper and within reach_m + d + the
    nose of the axle along the corridor, and laterally close enough to the
    corridor to matter (|c| < half_w + d). Shared by plan_keepouts and the
    adaptive corridor (adaptive_corridor.py), so both consider the same
    obstacles.
    """
    def relevant(s_ahead, c, d, hw):
        r = d - car_radius - margin
        if s_ahead < -(r - body.rear + margin):
            return 'behind'
        if s_ahead > reach_m + d + body.front:
            return 'beyond_reach'
        if abs(c) >= hw + d:
            return 'outside_corridor'
        return None
    return relevant


def plan_keepouts(obstacles: Sequence[Tuple[float, float, float]],
                  corridor,
                  car_xy: Tuple[float, float],
                  car_radius: float,
                  margin: float,
                  body: BodyBox,
                  reach_m: float,
                  memory: Optional[PassSideMemory] = None,
                  now: float = 0.0,
                  circles: Optional[Sequence[Tuple[float, float]]] = None) -> KeepOutPlan:
    """One KeepOut per obstacle in the way, on the side pass_side decides.

    In the way: see keepout_relevance. The side and its commitment are
    pass_side.decide_sides' -- the single source of truth B4 reuses.
    memory None = a fresh memory (no commitment across calls; tests).
    """
    plan = KeepOutPlan()
    memory = memory if memory is not None else PassSideMemory()
    relevant = keepout_relevance(car_radius, margin, body, reach_m)

    decisions, skipped = decide_sides(obstacles, corridor, car_xy, car_radius, margin,
                                      relevant, memory, now)
    plan.report.extend(skipped)
    for dec in decisions:
        rec = dec.as_dict()
        plan.report.append(rec)
        if dec.side is None:
            plan.blocked.append(rec)
            continue
        s_lo = -(dec.r + body.front + margin)
        s_hi = dec.r - body.rear + margin
        plan.keepouts.append(KeepOut(
            dec.ox, dec.oy, dec.nx, dec.ny, dec.ny, -dec.nx, s_lo, s_hi, dec.d,
            dec.side, r=dec.r, margin=float(margin),
            circles=tuple(tuple(c) for c in (circles or ()))))
    return plan


def plan_body_gap(x_pred, obstacles: Sequence[Tuple[float, float, float]],
                  body: BodyBox) -> float:
    """Smallest gap [m] between the car rectangle and any obstacle disk over a plan.

    Negative = the plan drives the body into an obstacle. inf when there is
    nothing to measure.
    """
    best = math.inf
    if x_pred is None or not obstacles:
        return best
    for state in x_pred:
        px, py, yaw = float(state[0]), float(state[1]), float(state[2])
        c, s = math.cos(yaw), math.sin(yaw)
        for ox, oy, r in obstacles:
            dx, dy = float(ox) - px, float(oy) - py
            lon = dx * c + dy * s
            lat = -dx * s + dy * c
            qx = min(max(lon, body.rear), body.front)
            qy = min(max(lat, -body.half_width), body.half_width)
            best = min(best, math.hypot(lon - qx, lat - qy) - float(r))
    return best


@dataclass(frozen=True)
class BlockedVerdict:
    reason: str
    detail: Dict


class BlockedDetector:
    """Decide, tick by tick, whether the active move is BLOCKED.

    reset() at every new move. update() once per control tick with the
    tick's facts; it returns a BlockedVerdict the first tick a rule fires
    (and keeps returning it), else None.
    """

    def __init__(self, min_moving_speed: float, progress_window_s: float,
                 min_progress_m: float, lock_window_s: float, lock_fraction: float,
                 infeasible_s: float):
        if not (progress_window_s > 0.0 and lock_window_s > 0.0
                and infeasible_s > 0.0 and min_progress_m > 0.0
                and 0.0 < lock_fraction <= 1.0):
            raise ValueError('BlockedDetector: windows and progress must be > 0, '
                             'lock_fraction in (0, 1]')
        self.min_moving_speed = float(min_moving_speed)
        self.progress_window_s = float(progress_window_s)
        self.min_progress_m = float(min_progress_m)
        self.lock_window_s = float(lock_window_s)
        self.lock_fraction = float(lock_fraction)
        self.infeasible_s = float(infeasible_s)
        self.reset()

    def reset(self) -> None:
        self.verdict: Optional[BlockedVerdict] = None
        self._moving: List[Tuple[float, float, float]] = []
        self._lock_since: Optional[Tuple[float, float, float]] = None
        self._infeasible_since: Optional[float] = None
        self._last_infeasible_detail: Dict = {}

    def update(self, t: float, x: float, y: float, v_cmd: float, steer: float,
               steer_min: float, steer_max: float, turning_move: bool,
               heading_ref: Optional[float], feasible: bool,
               infeasible_detail: Optional[Dict] = None) -> Optional[BlockedVerdict]:
        if self.verdict is not None:
            return self.verdict
        t = float(t)

        # -- no_feasible_plan --------------------------------------------
        if not feasible:
            if self._infeasible_since is None:
                self._infeasible_since = t
            self._last_infeasible_detail = dict(infeasible_detail or {})
            if t - self._infeasible_since >= self.infeasible_s - 1e-9:
                detail = dict(self._last_infeasible_detail)
                detail['for_s'] = round(t - self._infeasible_since, 2)
                self.verdict = BlockedVerdict(REASON_INFEASIBLE, detail)
                return self.verdict
        else:
            self._infeasible_since = None

        # -- no_progress -------------------------------------------------
        if float(v_cmd) >= self.min_moving_speed - 1e-9:
            self._moving.append((t, float(x), float(y)))
            while self._moving and t - self._moving[0][0] > self.progress_window_s + 1e-9:
                self._moving.pop(0)
            span = t - self._moving[0][0]
            if span >= self.progress_window_s - 1e-6:
                path = 0.0
                for (_, x0, y0), (_, x1, y1) in zip(self._moving, self._moving[1:]):
                    path += math.hypot(x1 - x0, y1 - y0)
                if path < self.min_progress_m:
                    self.verdict = BlockedVerdict(REASON_NO_PROGRESS, {
                        'path_m': round(path, 3), 'window_s': round(span, 2),
                        'v_cmd': round(float(v_cmd), 3)})
                    return self.verdict
        else:
            # Not commanding motion (an infeasible tick, a stop): the car is
            # meant to be still, so stillness is not evidence of anything.
            self._moving = []

        # -- steering_lock_no_progress --------------------------------------
        bound = steer_max if steer >= 0.0 else steer_min
        at_lock = (abs(bound) > 1e-9
                   and abs(float(steer)) >= self.lock_fraction * abs(float(bound)))
        if turning_move or heading_ref is None or not at_lock:
            self._lock_since = None
        else:
            if self._lock_since is None:
                self._lock_since = (t, float(x), float(y))
            t0, x0, y0 = self._lock_since
            if t - t0 >= self.lock_window_s - 1e-9:
                along = ((float(x) - x0) * math.cos(heading_ref)
                         + (float(y) - y0) * math.sin(heading_ref))
                if along < self.min_progress_m:
                    self.verdict = BlockedVerdict(REASON_LOCK, {
                        'along_m': round(along, 3), 'window_s': round(t - t0, 2),
                        'steer': round(float(steer), 3)})
                    return self.verdict
                # Progress was made: slide the window.
                self._lock_since = (t, float(x), float(y))
        return None


REASON_NO_WALL_DISTANCE = 'no_wall_distance'


class WallDistanceWait:
    """A wall_turn waiting for a valid front distance before it may commit
    (review B8): no distance = no wall decision, the car goes straight; if
    none arrives within timeout_s the move is BLOCKED no_wall_distance.

    update(now, has_distance) -> BlockedVerdict once the wait runs out, else
    None. A valid distance ends the wait; reset() at every new move.
    """

    def __init__(self, timeout_s: float):
        if not timeout_s > 0.0:
            raise ValueError('wall_turn_distance_timeout_s must be > 0')
        self.timeout_s = float(timeout_s)
        self.since: Optional[float] = None

    def reset(self) -> None:
        self.since = None

    def update(self, now: float, has_distance: bool) -> Optional[BlockedVerdict]:
        if has_distance:
            self.since = None
            return None
        if self.since is None:
            self.since = float(now)
        waited = float(now) - self.since
        if waited >= self.timeout_s - 1e-9:
            return BlockedVerdict(REASON_NO_WALL_DISTANCE, {
                'waited_s': round(waited, 2), 'timeout_s': self.timeout_s})
        return None
