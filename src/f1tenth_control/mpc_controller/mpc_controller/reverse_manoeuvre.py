"""
A short reverse manoeuvre for mpc_corr, taken only when the forward MPC is stuck.

Pure Python (no rclpy). MPC_corr calls ReverseSupervisor.before_forward and
after_forward around its forward solve; test/reverse_manoeuvre_harness.py
drives the same two calls.

WHAT IT IS, AND WHAT IT IS NOT. The forward MPC drives by default and is not
changed. Nothing compares the forward solver's cost with the rear solver's:
the two optimise different problems and their numbers mean different things.
A reverse is taken only when the FORWARD side is demonstrably stuck, the REAR
side is demonstrably ready, and the behaviour tree allows it; it then runs to
completion along a short, frozen rear corridor and hands back to forward.

STUCK, for stuck_time_s CONTINUOUSLY (a tick on which none holds, or a tick
the detector does not see, resets the timer):
  (a) the forward solve failed or fell back;
  (b) the mission is active, |v| < stuck_speed_mps and something is within
      stuck_front_dist_m of the front bumper;
  (c) the forward plan's predicted progress over the horizon is below
      stuck_progress_m.
A stop the mission asked for is never stuck: hold, goal reached, mission not
RUNNING -- the timer is held at 0 (the node passes intentional_stop).

READY: the rear pre-checks pass (the swept strip behind the rear bumper is
FREE and KNOWN in the costmap, and no object -- semantic memory included --
is in it) AND the rear solver was feasible on the latest tick. The rear
solver is LAZY by default: it runs only while the stuck timer runs, so normal
driving never pays for it; "always" runs it every tick (for A/B timing).

SWITCH: stuck AND ready AND /bt/allow_reverse true and fresh, after
forward_min_time_s of forward driving since the last reverse, and fewer than
max_attempts consecutive reverses without reverse_progress_reset_m of forward
progress (then: mode "stuck", stopped until a new move). Both gear changes
command zero until |v| < stuck_speed_mps for gear_change_dwell_s. The reverse
is committed: only the terminations end it (distance done, rear clearance
below rear_stop_margin_m, rear solver infeasible or over budget twice in a
row, timeout = timeout_factor x the expected duration, hold / BT abort).

ONE SOLVER DRIVES. Every tick returns either nothing (the forward solver's
command goes out) or the manoeuvre's command (and the forward solver is not
run). The forward solver keeps its warm start untouched while paused and
re-rolls it from the current state when it resumes.
"""

from dataclasses import dataclass, field
import math
import time
from typing import Dict, List, Optional, Sequence, Tuple

from f1tenth_params.corridor_geometry import corridor_curves, CORRIDOR_WIDTH_RATIO
from mpc_controller.mpc_solver import shift_warm_start, solve_mpc_step
import numpy as np

__all__ = [
    'MODE_FORWARD', 'MODE_REVERSE', 'MODE_STOPPING', 'MODE_STUCK', 'Grid',
    'ManoeuvreConfig', 'RearEnv', 'RearSolver', 'ReverseSupervisor', 'StuckDetector',
    'map_to_odom_point', 'odom_to_map_point', 'rear_corridor', 'rear_horizon',
    'rear_strip_check',
]

MODE_FORWARD = 'forward'
MODE_STOPPING = 'stopping'
MODE_REVERSE = 'reverse'
MODE_STUCK = 'stuck'


@dataclass
class ManoeuvreConfig:
    """The reverse manoeuvre's parameters (stack_params.yaml, reverse_* / rear_* / mpc_rear.*)."""

    auto_enabled: bool = True
    solver_mode: str = 'lazy'           # 'lazy' | 'always'
    corridor_length_m: float = 0.5
    corridor_half_width_m: float = 0.64     # start half-width (at the rear bumper)
    width_ratio: float = CORRIDOR_WIDTH_RATIO   # end / start, as the forward corridor
    v_rev_mps: float = 0.4
    rear_dt_s: float = 0.2
    n_min: int = 5
    n_max: int = 10
    stuck_time_s: float = 0.5
    stuck_speed_mps: float = 0.05
    stuck_front_dist_m: float = 0.6
    stuck_progress_m: float = 0.05
    gear_change_dwell_s: float = 0.3
    forward_min_time_s: float = 2.0
    max_attempts: int = 3
    progress_reset_m: float = 0.3
    rear_stop_margin_m: float = 0.15
    # Braking model for the stop margin: the reverse ends when the free
    # distance behind falls below margin + |v| * delay + v^2 / (2 decel), so
    # the car comes to REST with the margin left. (0.144 s, 1.5 m/s^2) is the
    # fit to the archived forward stops (test_object_approach_closed_loop's
    # BRAKING_MEASURED); reverse braking is unmeasured.
    brake_delay_s: float = 0.144
    brake_decel_mps2: float = 1.5
    rear_check_margin_m: float = 0.10
    bt_allow_max_age_s: float = 0.5
    solver_budget_s: float = 0.05
    timeout_factor: float = 2.0
    # Car geometry: rear bumper behind base_link, body half-width.
    car_rear_offset_m: float = 0.082
    car_half_width_m: float = 0.136
    # Costmap cell values: -1 unknown, [0, free_max) free, >= free_max occupied.
    costmap_free_max: int = 50
    # The /drive clamp's reverse limit: a reverse it would zero is not ready.
    max_reverse_speed_mps: float = 0.0


def rear_horizon(cfg: ManoeuvreConfig) -> int:
    """N_rear = ceil(L / (v_rev * dt)), clamped to [n_min, n_max]."""
    n = int(math.ceil(cfg.corridor_length_m / max(cfg.v_rev_mps * cfg.rear_dt_s, 1e-9)))
    return int(min(max(n, cfg.n_min), cfg.n_max))


# ------------------------------------------------------------------ frames

def map_to_odom_point(p, map_to_odom):
    """Express a map-frame point (x, y) in odom; map_to_odom is the (x, y, yaw) edge."""
    if map_to_odom is None:
        return float(p[0]), float(p[1])
    tx, ty, yaw = map_to_odom
    dx, dy = float(p[0]) - tx, float(p[1]) - ty
    c, s = math.cos(yaw), math.sin(yaw)
    return c * dx + s * dy, -s * dx + c * dy


def odom_to_map_point(p, map_to_odom):
    """Express an odom-frame point (x, y) in map."""
    if map_to_odom is None:
        return float(p[0]), float(p[1])
    tx, ty, yaw = map_to_odom
    c, s = math.cos(yaw), math.sin(yaw)
    return tx + c * float(p[0]) - s * float(p[1]), ty + s * float(p[0]) + c * float(p[1])


# ------------------------------------------------------------------ the corridor

def rear_corridor(x: float, y: float, psi: float, cfg: ManoeuvreConfig,
                  n: int = 30, walls: str = 'bezier') -> Dict:
    """
    Return the rear corridor (odom) for a car at (x, y, psi), as a QP corridor dict.

    It starts at the REAR BUMPER (car_rear_offset_m behind base_link) and runs
    corridor_length_m straight back along the heading. Its half-width grows
    from corridor_half_width_m at the bumper to that times width_ratio at the
    far end -- the forward corridors' rule, not a rectangle. It is built by
    corridor_curves pointing backwards and then FLIPPED, so the arrays run
    from the far end to the bumper and every tangent points the way the car
    faces: the heading costs then hold the car's own heading (the car does not
    turn round to reverse), and psiRef is psi.
    """
    bx = x - cfg.car_rear_offset_m * math.cos(psi)
    by = y - cfg.car_rear_offset_m * math.sin(psi)
    back = psi + math.pi
    w0 = cfg.corridor_half_width_m
    geom = corridor_curves(bx, by, back, back, cfg.corridor_length_m, n,
                           w0=w0, w1=w0 * cfg.width_ratio, walls=walls)
    rev = slice(None, None, -1)
    out = {
        'xc': geom['xc'][rev], 'yc': geom['yc'][rev],
        'tx': -geom['tx'][rev], 'ty': -geom['ty'][rev],
        'nx': -geom['nx'][rev], 'ny': -geom['ny'][rev],
        'xL': geom['xR'][rev], 'yL': geom['yR'][rev],
        'xR': geom['xL'][rev], 'yR': geom['yL'][rev],
        'halfWidth': geom['halfWidth'][rev],
        'psiRef': float(psi), 'psiRefTurn': None, 'L': float(cfg.corridor_length_m),
        'Pend': np.array([geom['xc'][-1], geom['yc'][-1]]),
        'bumper': (bx, by), 'psiStart': float(psi), 'targetLine': None,
    }
    return out


# ------------------------------------------------------------------ the costmap

@dataclass
class Grid:
    """An occupancy grid (nav_msgs/OccupancyGrid's content), map frame."""

    data: np.ndarray            # (height, width) int, row-major as the message
    resolution: float
    origin_x: float
    origin_y: float
    origin_yaw: float = 0.0

    def values_at(self, xs: np.ndarray, ys: np.ndarray) -> np.ndarray:
        """Cell values at map points; -1 (unknown) outside the grid."""
        c, s = math.cos(self.origin_yaw), math.sin(self.origin_yaw)
        dx, dy = xs - self.origin_x, ys - self.origin_y
        gx = (c * dx + s * dy) / self.resolution
        gy = (-s * dx + c * dy) / self.resolution
        i = np.floor(gx).astype(int)
        j = np.floor(gy).astype(int)
        h, w = self.data.shape
        inside = (i >= 0) & (i < w) & (j >= 0) & (j < h)
        out = np.full(xs.shape, -1, dtype=int)
        out[inside] = self.data[j[inside], i[inside]]
        return out


def rear_strip_check(bumper_xy, psi, length, cfg: ManoeuvreConfig, grid: Optional[Grid],
                     map_to_odom, objects_odom: Sequence[Tuple[float, float, float]],
                     unknown_blocks: bool = True) -> Tuple[bool, str, float]:
    """
    Check the strip the body sweeps behind the rear bumper: (ok, reason, clearance).

    The strip runs `length` back from the bumper along -heading and is
    car_half_width_m + rear_check_margin_m to either side. ok needs every
    costmap sample in it FREE and KNOWN and no object disk reaching into it.
    clearance is the free distance behind the bumper before the first
    blocking sample or object (length if none) -- what the rear stop margin
    is measured on during the manoeuvre. No grid is "rear_unknown".
    """
    W = cfg.car_half_width_m + cfg.rear_check_margin_m
    back = np.array([-math.cos(psi), -math.sin(psi)])
    side = np.array([-back[1], back[0]])
    bx, by = float(bumper_xy[0]), float(bumper_xy[1])
    clearance = float(length)
    reason = ''
    for ox, oy, r in objects_odom:
        rel = np.array([ox - bx, oy - by])
        s_along = float(rel @ back)
        d_side = abs(float(rel @ side))
        if d_side <= W + r and -r <= s_along <= length + r:
            clearance = min(clearance, max(s_along - r, 0.0))
            reason = reason or 'rear_object'
    if grid is None:
        return False, reason or 'rear_unknown', clearance
    step = max(grid.resolution * 0.5, 0.01)
    ss = np.arange(0.0, length + 1e-9, step)
    ds = np.arange(-W, W + 1e-9, step)
    S, D = np.meshgrid(ss, ds, indexing='ij')
    px = bx + S * back[0] + D * side[0]
    py = by + S * back[1] + D * side[1]
    if map_to_odom is not None:
        tx, ty, yaw = map_to_odom
        c, s = math.cos(yaw), math.sin(yaw)
        mx, my = tx + c * px - s * py, ty + s * px + c * py
    else:
        mx, my = px, py
    vals = grid.values_at(mx, my)
    occupied = vals >= cfg.costmap_free_max
    unknown = vals < 0
    blocking = occupied | (unknown if unknown_blocks else np.zeros_like(unknown))
    if blocking.any():
        clearance = min(clearance, float(S[blocking].min()))
        if not reason:
            reason = 'rear_occupied' if occupied.any() else 'rear_unknown'
    return reason == '', reason, clearance


# ------------------------------------------------------------------ the rear solver

class RearSolver:
    """
    The rear MPC: the forward bicycle QP at a FIXED speed of -v_rev_mps.

    Same model, same steering limits and steering rate limit, its own horizon
    (rear_horizon), prediction step, weights (mpc_rear.*) and warm start. The
    speed is not a decision: acceleration is pinned at 0 and every stage's
    speed box is [-v_rev, -v_rev], so the QP chooses steering only, along the
    rear corridor toward its far end, with the rear bumper probes in w_obs on
    every stage (the front ones stay: the nose swings when reversing).
    """

    def __init__(self, cfg: ManoeuvreConfig, weights: Dict, limits: Dict, params: Dict,
                 rear_offsets: Sequence[float], boundary_opts: Optional[Dict] = None):
        self.cfg = cfg
        self.weights = dict(weights)
        self.limits = dict(limits)
        self.params = dict(params)
        self.rear_offsets = tuple(rear_offsets)
        self.boundary_opts = dict(boundary_opts or {})
        self.n = rear_horizon(cfg)
        self.warm: Optional[np.ndarray] = None
        self.last_ms = math.nan
        self.calls = 0

    def reset(self):
        """Drop the warm start (the lazy solver stopped)."""
        self.warm = None

    def solve(self, x, steer_prev: float, corridor: Dict, obstacles, boundaries,
              car_radius: float, avoidance_margin: float, control_period: float):
        """Return (steering, info, ok, seconds)."""
        cfg = self.cfg
        n = self.n
        v = -abs(cfg.v_rev_mps)
        x0 = np.array([x[0], x[1], x[2], v], dtype=float)
        lim = dict(self.limits, a_min=0.0, a_max=0.0, dAMin=0.0, dAMax=0.0,
                   vMin=v, vMax=v)
        corr = dict(corridor)
        corr['obstacles_world'] = list(obstacles)
        corr['car_radius'] = car_radius
        corr['avoidance_margin'] = avoidance_margin
        spec = {'v_lo': np.full(n, v), 'v_hi': np.full(n, v), 'vdes': np.full(n, v),
                'rear': np.ones(n, dtype=bool), 'rear_offsets': self.rear_offsets}
        far = np.array([corridor['xc'][0], corridor['yc'][0]])
        warm = self.warm
        if warm is None:
            warm = np.zeros(2 * n)
            warm[0::2] = steer_prev
        t0 = time.perf_counter()
        self.calls += 1
        try:
            u0, info = solve_mpc_step(
                x0=x0, last_u=np.array([steer_prev, 0.0]), pref_nom=far, corridor=corr,
                horizon=n, ts=cfg.rear_dt_s, params=self.params, limits=lim,
                weights=dict(self.weights), obstacles=list(obstacles),
                dmin=car_radius + avoidance_margin, vdes=v, solver='rti',
                boundaries=list(boundaries), warm_start_z=warm,
                control_period=control_period, stage_spec=spec, **self.boundary_opts)
        except Exception as exc:  # noqa: BLE001 -- a failed rear solve is "not ready"
            self.last_ms = (time.perf_counter() - t0) * 1e3
            self.warm = None
            return steer_prev, {'error': repr(exc)}, False, self.last_ms / 1e3
        dt = time.perf_counter() - t0
        self.last_ms = dt * 1e3
        ok = bool(info.get('success', False))
        self.warm = (shift_warm_start(info.get('zopt'), n,
                                      step_fraction=control_period / cfg.rear_dt_s)
                     if ok else None)
        return float(u0[0]), info, ok, dt


# ------------------------------------------------------------------ stuck

class StuckDetector:
    """Any of (a) (b) (c) on every tick for stuck_time_s; see the module docstring."""

    def __init__(self, cfg: ManoeuvreConfig, control_period: float):
        self.cfg = cfg
        self.dt = float(control_period)
        self.timer = 0.0
        self.reason = ''
        self._last_t: Optional[float] = None

    def reset(self):
        self.timer = 0.0
        self.reason = ''
        self._last_t = None

    def update(self, now: float, forward_ok: bool, mission_active: bool, speed: float,
               front_dist: Optional[float], progress_m: Optional[float],
               intentional_stop: bool) -> bool:
        cfg = self.cfg
        # A tick the detector did not see (an early return in the node: hold,
        # goal reached, no state) breaks "continuously".
        if self._last_t is not None and now - self._last_t > 1.5 * self.dt:
            self.timer = 0.0
        self._last_t = now
        if intentional_stop:
            self.timer = 0.0
            self.reason = ''
            return False
        reasons = []
        if not forward_ok:
            reasons.append('forward_infeasible')
        if (mission_active and abs(speed) < cfg.stuck_speed_mps
                and front_dist is not None and front_dist < cfg.stuck_front_dist_m):
            reasons.append('blocked_front')
        if progress_m is not None and progress_m < cfg.stuck_progress_m:
            reasons.append('no_progress')
        if reasons:
            self.timer += self.dt
        else:
            self.timer = 0.0
        self.reason = '+'.join(reasons)
        return self.timer >= cfg.stuck_time_s - 1e-9


# ------------------------------------------------------------------ the supervisor

@dataclass
class RearEnv:
    """What the supervisor reads from the world each tick (odom frame unless noted)."""

    grid: Optional[Grid] = None            # map frame
    map_to_odom: Optional[Tuple[float, float, float]] = None
    objects: List[Tuple[float, float, float]] = field(default_factory=list)
    boundaries: List[Tuple[float, float, float]] = field(default_factory=list)
    bt_allow: Optional[bool] = None
    bt_allow_age_s: float = math.inf
    abort: bool = False                    # e-stop / hold / BT abort


@dataclass
class Command:
    speed: float
    steer: float
    source: str                            # 'stop' | 'rear'


class ReverseSupervisor:
    """The arbitration state machine; see the module docstring."""

    def __init__(self, cfg: ManoeuvreConfig, rear_solver: RearSolver, control_period: float,
                 car_radius: float = 0.20, avoidance_margin: float = 0.12):
        self.cfg = cfg
        self.rear = rear_solver
        self.ts = float(control_period)
        self.car_radius = car_radius
        self.avoidance_margin = avoidance_margin
        self.stuck = StuckDetector(cfg, control_period)
        self.mode = MODE_FORWARD
        self.target = MODE_FORWARD          # where a 'stopping' phase leads
        self.attempts = 0
        self.last_switch_t = 0.0
        self.forward_since: Optional[float] = None
        self.last_termination = ''
        self.rear_ready = False
        self.rear_reason = 'not_running'
        self.rear_running = False
        self.rear_clearance = math.nan
        self.requested = 0.0
        self.travelled = 0.0
        self._stop_since: Optional[float] = None
        self._start: Optional[Tuple[float, float, float]] = None   # map frame
        self._started_t = 0.0
        self._rear_fail = 0
        # Where (and facing which way) the car was stuck when the last reverse
        # was triggered: forward progress is measured PAST this point, so
        # driving back up to the same obstacle after a reverse is not progress.
        self._stuck_pose: Optional[Tuple[float, float, float]] = None
        self._steer = 0.0
        self._is_stuck = False
        self._intentional = False
        self._bt_ok = False
        self._bt_allow = False

    # -------------------------------------------------------------- public

    @property
    def reverse_active(self) -> bool:
        return self.mode == MODE_REVERSE or (self.mode == MODE_STOPPING
                                             and self.target == MODE_REVERSE)

    def new_move(self):
        """Start a new move or mission: forget attempts and a 'stuck' give-up."""
        self.attempts = 0
        self._stuck_pose = None
        if self.mode == MODE_STUCK:
            self.mode = MODE_FORWARD
        self.stuck.reset()

    def abort(self, reason: str):
        """E-stop / hold / BT abort: end a manoeuvre now (zero speed, then forward)."""
        if self.reverse_active:
            self._end_reverse(reason, None)

    def request(self, now: float, x, distance_m: float, env: RearEnv) -> Tuple[bool, str]:
        """/mpc/request_reverse: the same pre-checks and BT permission, no stuck condition."""
        if self.mode != MODE_FORWARD:
            return False, f'busy: mode {self.mode}'
        ok, reason = self._permission(env)
        if not ok:
            return False, reason
        ready, reason = self._precheck(x, env)
        if not ready:
            return False, reason
        dist = self.cfg.corridor_length_m if distance_m <= 0.0 else min(
            float(distance_m), self.cfg.corridor_length_m)
        self._begin_stop_to_reverse(now, dist, 'manual')
        return True, f'accepted: {dist:.2f} m'

    def before_forward(self, now: float, x, env: RearEnv) -> Optional[Command]:
        """Return the manoeuvre's command this tick, or None: the forward solver drives."""
        self._update_bt(env)
        if self.mode == MODE_FORWARD:
            return None
        if self.mode == MODE_STUCK:
            return Command(0.0, self._steer, 'stop')
        if env.abort and self.reverse_active:
            self._end_reverse('abort', x)
        elif self.mode == MODE_REVERSE:
            # The BT withdrawing (or no longer refreshing) its permission is a
            # BT abort as much as a hold is.
            ok, reason = self._permission(env)
            if not ok:
                self._end_reverse('bt_withdrawn: ' + reason, x)
        if self.mode == MODE_STOPPING:
            if self.target == MODE_REVERSE:
                ok, reason = self._permission(env)
                ready, why = self._precheck(x, env)
                if not (ok and ready):
                    self._end_reverse('precheck_lost: ' + (reason or why), x)
                    return Command(0.0, self._steer, 'stop')
            if not self._stopped_long_enough(now, x):
                return Command(0.0, self._steer, 'stop')
            if self.target == MODE_FORWARD:
                self.mode = MODE_FORWARD
                self.forward_since = now
                self.last_switch_t = now
                return None
            self._enter_reverse(now, x, env)
        return self._reverse_tick(now, x, env)

    def after_forward(self, now: float, x, forward_ok: bool, x_pred, mission_active: bool,
                      intentional_stop: bool, front_dist: Optional[float],
                      env: RearEnv) -> Optional[Command]:
        """Feed the stuck detector after a forward solve; maybe start a reverse."""
        cfg = self.cfg
        self._update_bt(env)
        self._intentional = bool(intentional_stop)
        progress = None
        if x_pred is not None and len(x_pred):
            progress = float(math.hypot(x_pred[-1][0] - x[0], x_pred[-1][1] - x[1]))
        stuck = self.stuck.update(now, forward_ok, mission_active, float(x[3]), front_dist,
                                  progress, intentional_stop)
        self._is_stuck = stuck
        # Forward progress PAST the point where the car was last stuck resets
        # the attempts.
        if self._stuck_pose is not None and self.attempts:
            sx, sy, sp = self._stuck_pose
            if ((x[0] - sx) * math.cos(sp) + (x[1] - sy) * math.sin(sp)
                    >= cfg.progress_reset_m):
                self.attempts = 0
                self._stuck_pose = None
        # The rear side: pre-checks always (cheap), the solver only while the
        # stuck timer runs (lazy) or always.
        pre_ok, pre_reason = self._precheck(x, env)
        run_solver = cfg.solver_mode == 'always' or self.stuck.timer > 0.0
        if run_solver:
            self.rear_running = True
            corridor = rear_corridor(x[0], x[1], x[2], cfg)
            _s, _info, ok, dt = self.rear.solve(
                x, self._steer_now(x), corridor, env.objects, env.boundaries,
                self.car_radius, self.avoidance_margin, self.ts)
            solver_ok = ok and dt <= cfg.solver_budget_s
            self.rear_ready = pre_ok and solver_ok
            self.rear_reason = pre_reason if not pre_ok else (
                '' if solver_ok else ('rear_infeasible' if not ok else 'rear_over_budget'))
        else:
            if self.rear_running:
                self.rear.reset()
            self.rear_running = False
            self.rear_ready = False
            self.rear_reason = pre_reason or 'not_running'
        if not stuck or not cfg.auto_enabled:
            return None
        if self.attempts >= cfg.max_attempts:
            self.mode = MODE_STUCK
            self.last_termination = 'max_attempts'
            return Command(0.0, self._steer, 'stop')
        if self.forward_since is not None and now - self.forward_since < cfg.forward_min_time_s:
            return None
        ok, _reason = self._permission(env)
        if not (ok and self.rear_ready):
            return None
        self._stuck_pose = (float(x[0]), float(x[1]), float(x[2]))
        self._begin_stop_to_reverse(now, cfg.corridor_length_m, 'auto')
        return Command(0.0, self._steer, 'stop')

    def status(self) -> Dict:
        return {
            'mode': self.mode, 'reverse_active': self.reverse_active,
            'stuck': self._is_stuck, 'stuck_reason': self.stuck.reason,
            'stuck_timer_s': self.stuck.timer, 'intentional_stop': self._intentional,
            'rear_ready': self.rear_ready, 'rear_reason': self.rear_reason,
            'rear_solver_running': self.rear_running, 'rear_solve_ms': self.rear.last_ms,
            'rear_clearance_m': self.rear_clearance, 'bt_allow_reverse': self._bt_allow,
            'bt_allow_fresh': self._bt_ok, 'attempts': self.attempts,
            'last_switch_time_s': self.last_switch_t,
            'last_termination_reason': self.last_termination,
            'requested_m': self.requested, 'travelled_m': self.travelled,
        }

    # -------------------------------------------------------------- internals

    def _update_bt(self, env: RearEnv):
        self._bt_allow = bool(env.bt_allow)
        self._bt_ok = bool(env.bt_allow) and env.bt_allow_age_s < self.cfg.bt_allow_max_age_s

    def _permission(self, env: RearEnv) -> Tuple[bool, str]:
        if self.cfg.max_reverse_speed_mps < self.cfg.v_rev_mps - 1e-9:
            return False, 'reverse_speed_limit'
        if env.bt_allow is None or env.bt_allow_age_s >= self.cfg.bt_allow_max_age_s:
            return False, 'bt_allow_stale'
        if not env.bt_allow:
            return False, 'bt_disallowed'
        return True, ''

    def _precheck(self, x, env: RearEnv) -> Tuple[bool, str]:
        bx = x[0] - self.cfg.car_rear_offset_m * math.cos(x[2])
        by = x[1] - self.cfg.car_rear_offset_m * math.sin(x[2])
        ok, reason, clearance = rear_strip_check(
            (bx, by), x[2], self.cfg.corridor_length_m, self.cfg, env.grid, env.map_to_odom,
            env.objects)
        if self.mode == MODE_FORWARD:
            self.rear_clearance = clearance
        return ok, reason

    def _stopped_long_enough(self, now, x) -> bool:
        if abs(float(x[3])) >= self.cfg.stuck_speed_mps:
            self._stop_since = None
            return False
        if self._stop_since is None:
            self._stop_since = now
        return now - self._stop_since >= self.cfg.gear_change_dwell_s - 1e-9

    def _begin_stop_to_reverse(self, now, distance, why):
        self.mode = MODE_STOPPING
        self.target = MODE_REVERSE
        self.requested = float(distance)
        self.travelled = 0.0
        self.last_switch_t = now
        self._stop_since = None
        self.last_termination = ''
        self._trigger = why

    def _enter_reverse(self, now, x, env: RearEnv):
        self.mode = MODE_REVERSE
        self.attempts += 1
        self._started_t = now
        self._rear_fail = 0
        self.last_switch_t = now
        # Frozen for the manoeuvre, anchored in MAP like the forward corridor,
        # re-expressed through the current map -> odom every tick.
        px, py = odom_to_map_point((x[0], x[1]), env.map_to_odom)
        yaw = float(x[2]) + (env.map_to_odom[2] if env.map_to_odom is not None else 0.0)
        self._start = (px, py, yaw)

    def _start_odom(self, env: RearEnv):
        sx, sy, syaw = self._start
        ox, oy = map_to_odom_point((sx, sy), env.map_to_odom)
        return ox, oy, syaw - (env.map_to_odom[2] if env.map_to_odom is not None else 0.0)

    def _reverse_tick(self, now, x, env: RearEnv) -> Command:
        cfg = self.cfg
        sx, sy, syaw = self._start_odom(env)
        back = (-math.cos(syaw), -math.sin(syaw))
        self.travelled = max(0.0, (x[0] - sx) * back[0] + (x[1] - sy) * back[1])
        bx = x[0] - cfg.car_rear_offset_m * math.cos(x[2])
        by = x[1] - cfg.car_rear_offset_m * math.sin(x[2])
        remaining = max(self.requested - self.travelled, 0.0)
        _ok, _r, clearance = rear_strip_check(
            (bx, by), syaw, remaining + cfg.rear_stop_margin_m + 0.2, cfg, env.grid,
            env.map_to_odom, env.objects)
        self.rear_clearance = clearance
        expected = self.requested / max(cfg.v_rev_mps, 1e-6)
        if self.travelled >= self.requested - 1e-3:
            return self._end_reverse('distance_done', x)
        v = abs(float(x[3]))
        stopping = v * cfg.brake_delay_s + v * v / (2.0 * max(cfg.brake_decel_mps2, 1e-6))
        if clearance < cfg.rear_stop_margin_m + stopping:
            return self._end_reverse('rear_clearance', x)
        if now - self._started_t > cfg.timeout_factor * expected:
            return self._end_reverse('timeout', x)
        corridor = rear_corridor(sx, sy, syaw, cfg)
        steer, _info, ok, dt = self.rear.solve(
            x, self._steer_now(x), corridor, env.objects, env.boundaries,
            self.car_radius, self.avoidance_margin, self.ts)
        if not ok or dt > cfg.solver_budget_s:
            self._rear_fail += 1
            if self._rear_fail >= 2:
                return self._end_reverse(
                    'rear_infeasible' if not ok else 'rear_over_budget', x)
            steer = self._steer          # hold the last steering for one tick
        else:
            self._rear_fail = 0
        self._steer = steer
        return Command(-abs(cfg.v_rev_mps), steer, 'rear')

    def _end_reverse(self, reason, x) -> Command:
        self.last_termination = reason
        self.mode = MODE_STOPPING
        self.target = MODE_FORWARD
        self._stop_since = None
        self.rear.reset()
        self.rear_running = False
        self.stuck.reset()
        return Command(0.0, self._steer, 'stop')

    def _steer_now(self, x) -> float:
        return self._steer

    def set_steer(self, steer: float):
        """Record the steering the forward solver last published (rate-limit seed)."""
        self._steer = float(steer)
