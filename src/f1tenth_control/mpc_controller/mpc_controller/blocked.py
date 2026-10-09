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

__all__ = [
    'BodyBox',
    'BlockedDetector',
    'BlockedVerdict',
    'KeepOut',
    'KeepOutPlan',
    'first_floor_stage',
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


@dataclass(frozen=True)
class KeepOut:
    """One hard lateral row, as mpc_solver._solve_rti consumes it.

    Stage k gets   side * (n . p_k) >= side * (n . o) + d
    when its reference position's along-track coordinate t . (p_k - o) lies
    in [s_lo, s_hi]. side +1 = pass with the obstacle on the car's right
    (car to the LEFT of it), -1 = the other way.
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

    def as_dict(self) -> Dict:
        return {'o': (self.ox, self.oy), 'n': (self.nx, self.ny),
                't': (self.tx, self.ty), 's_lo': self.s_lo, 's_hi': self.s_hi,
                'd': self.d, 'side': self.side}


@dataclass
class KeepOutPlan:
    keepouts: List[KeepOut] = field(default_factory=list)
    # Obstacles in the way with no side that fits inside the corridor.
    blocked: List[Dict] = field(default_factory=list)
    # One record per obstacle considered, for the log.
    report: List[Dict] = field(default_factory=list)


def _nearest_frame(px: float, py: float, corridor) -> Tuple[float, float, float, float, float]:
    """(pcx, pcy, nx, ny, half_w) of the corridor sample nearest (px, py)."""
    xc = corridor['xc']
    yc = corridor['yc']
    best = 0
    best_d2 = math.inf
    for i in range(len(xc)):
        d2 = (float(xc[i]) - px) ** 2 + (float(yc[i]) - py) ** 2
        if d2 < best_d2:
            best_d2 = d2
            best = i
    return (float(xc[best]), float(yc[best]), float(corridor['nx'][best]),
            float(corridor['ny'][best]), float(corridor['halfWidth'][best]))


def plan_keepouts(obstacles: Sequence[Tuple[float, float, float]],
                  corridor,
                  car_xy: Tuple[float, float],
                  car_radius: float,
                  margin: float,
                  body: BodyBox,
                  reach_m: float,
                  prev_sides: Optional[Sequence[Tuple[float, float, int]]] = None,
                  side_memory_m: float = 0.5) -> KeepOutPlan:
    """Choose a side and build one KeepOut per obstacle that is in the way.

    In the way: ahead of the car's rear bumper and within reach_m + d of the
    axle along the corridor, and laterally close enough to the corridor to
    matter (|c| < half_w + d, c = the obstacle's offset from the centreline).

    Side: of the sides that fit inside the corridor half-width at the
    obstacle (c + d <= hw to pass left, c - d >= -hw to pass right), the one
    nearer the car's own lateral offset -- the less steering, and
    self-reinforcing once the car starts to move. An exact tie keeps the side
    chosen last tick for an obstacle within side_memory_m (prev_sides), else
    takes the side with more room, else the left. No side fits: listed in
    .blocked and no row is added (the caller treats the tick as infeasible).
    """
    plan = KeepOutPlan()
    if not obstacles:
        return plan
    cx, cy = float(car_xy[0]), float(car_xy[1])
    pcx_c, pcy_c, nx_c, ny_c, _ = _nearest_frame(cx, cy, corridor)
    y_car = nx_c * (cx - pcx_c) + ny_c * (cy - pcy_c)
    for ox, oy, r in obstacles:
        ox, oy, r = float(ox), float(oy), float(r)
        pcx, pcy, nx, ny, hw = _nearest_frame(ox, oy, corridor)
        tx, ty = ny, -nx  # build_straight_corridor: n = (-ty, tx)
        d = r + float(car_radius) + float(margin)
        c = nx * (ox - pcx) + ny * (oy - pcy)
        s_rel = tx * (ox - cx) + ty * (oy - cy)
        rec = {'o': [round(ox, 3), round(oy, 3)], 'r': round(r, 3),
               'c': round(c, 3), 'd': round(d, 3), 'half_w': round(hw, 3),
               's_ahead': round(s_rel, 3)}
        # Along-track window for the AXLE in which the body overlaps the
        # obstacle (plus the margin both ways).
        s_lo = -(r + body.front + margin)
        s_hi = r - body.rear + margin
        if s_rel < -s_hi or s_rel > reach_m + d + body.front:
            rec['skip'] = 'behind' if s_rel < -s_hi else 'beyond_reach'
            plan.report.append(rec)
            continue
        if abs(c) >= hw + d:
            rec['skip'] = 'outside_corridor'
            plan.report.append(rec)
            continue
        fits_left = c + d <= hw + 1e-9
        fits_right = c - d >= -hw - 1e-9
        rec['fits'] = {'left': fits_left, 'right': fits_right}
        if not (fits_left or fits_right):
            rec['side'] = None
            plan.blocked.append(rec)
            plan.report.append(rec)
            continue
        if fits_left and fits_right:
            cost_left = abs((c + d) - y_car)
            cost_right = abs((c - d) - y_car)
            if abs(cost_left - cost_right) > 1e-6:
                side = 1 if cost_left < cost_right else -1
            else:
                side = None
                for px, py, ps in (prev_sides or ()):
                    if math.hypot(px - ox, py - oy) <= side_memory_m:
                        side = int(ps)
                        break
                if side is None:
                    room_left = hw - (c + d)
                    room_right = (c - d) + hw
                    side = 1 if room_left >= room_right else -1
        else:
            side = 1 if fits_left else -1
        rec['side'] = 'left' if side > 0 else 'right'
        plan.keepouts.append(KeepOut(ox, oy, nx, ny, tx, ty, s_lo, s_hi, d, side))
        plan.report.append(rec)
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
