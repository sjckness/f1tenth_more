"""
The /drive speed law: today's increment, or offset-free MPC on a lag model.

Pure Python (no rclpy). MPC_corr owns one SpeedChannel and calls it around
every tick; test/speed_channel_harness.py drives the same object, call for
call, against a plant that is NOT this model.

WHAT IT REPLACES (output/speed_gap/report.md). speed_command_mode 'legacy'
publishes v_meas + a_0 * ts: the solver re-seeds its speed state from the
measurement every tick and assumes the car tracks the command exactly, so
only its first acceleration ever reaches the wheels. Against a plant that
returns r < 1 of its command that loop has the fixed point a * ts / (1 - r)
-- 0.35 m/s commanded, 0.25 real, against vdes 0.5 on 2026-10-21 -- and
nothing in it can leave the fixed point. 'legacy' is kept, operation for
operation, as the rollback.

THE MODEL ('offset_free'). Two speed states instead of one:

    c   the command in force: the last command the actuator received;
    v   the real speed, seeded from the measurement every tick;

    c+ = c + ts * a                      a is the command RATE: a_min/a_max
                                         and the dAMin/dAMax slew keep their
                                         meaning, they now shape the command
    v+ = v + alpha * (c_eff + d - v)     alpha = 1 - exp(-ts / tau)

c_eff is the command acting during the step -- the new one, c + ts * a, or
with speed_delay_ticks = 1 the one already in force. alpha is the EXACT
zero-order-hold discretisation of the first-order lag dv/dt = (c + d - v) /
tau, not the Euler ts / tau: identical for ts << tau, and stable for any tau
(Euler is not once tau < ts). d is a constant disturbance over the horizon,
expressed in m/s OF COMMAND: the car behaves as if it were sent c + d. The
work order's v+ = v + (ts/tau)(c - v) + d' is the same model with d' =
alpha * d; this scaling is used because it makes d directly comparable with
the speeds it corrects (a car returning 76 % of 0.5 m/s has d = -0.12) and
makes the saturation a speed: |d| <= offset_free_d_max_mps.

The kinematics (X, Y, psi) are driven by the REAL v, the cost w_v * (v -
vdes)^2 acts on the real v, and the published command is c at stage 1, c_0 +
a_0 * ts. x0's c is the command the actuator last received -- what this node
last published, or what the mux forwarded instead when another lane drives
-- NEVER the measurement.

THE ESTIMATOR. A current-form Luenberger observer on (v, d):

    v_pred = (1 - alpha) v_hat + alpha (c_eff + d_hat)      time update
    e      = v_meas - v_pred
    v_hat  = v_pred + L_v e ;  d_hat = d_hat + L_d e        measurement update

with the error dynamics e+ = (I - L C) A e placed at two real poles,
p = exp(-bandwidth * ts) (offset_free_obs_bw_v_rad_s / _bw_d_rad_s):

    L_v = 1 - p_v p_d / (1 - alpha),   L_d = (1 - p_v)(1 - p_d) / alpha.

OFFSET-FREE, and why. The augmented pair A = [[1 - alpha, alpha], [0, 1]],
C = [1, 0] is observable for every alpha > 0 (det [C; CA] = alpha), so the
poles can be placed anywhere; and the offset-free condition (Pannocchia &
Rawlings 2003) rank [[I - A_v, -B_d], [C_v, C_d]] = [[alpha, -alpha], [1,
0]] = 2 = n + n_d holds for the same reason. Consequence: at any closed-loop
steady state the innovation is zero (d_hat stationary), so v_hat = v_meas =
c + d_hat; the plan from a model-consistent rest point moves its speed only
toward the solver's own optimum, so the steady state is where the solver's
plan stops moving -- vdes, the plateau the legacy solver already planned to
(pred_v 0.4996-0.5001 in the report). That holds whatever the true gain,
lag or delay; they change only the transient, which is what
test_speed_channel_matrix.py measures. Pure delay does not enter the steady
state at all.

FREEZE. d_hat integrates only while the lag model describes the plant. It is
HELD (v_hat re-synced to the measurement) on every tick where any of these
holds, and the reasons are reported:
  hold, estop, mission_inactive  the car is being stopped on purpose;
  stopping                       a stop path is publishing zero (ramp
                                 hand-over, goal reached);
  low_command / low_speed        command or measured speed below the lag
                                 model's range: standstill, static friction,
                                 the VESC telemetry deadband (500 ERPM =
                                 0.111 m/s reads exactly 0);
  stall                          command >= offset_free_stall_cmd_mps with
                                 |v_meas| < offset_free_stall_speed_mps for
                                 offset_free_stall_time_s: blocked wheels.
                                 Also caps the command at what it was when the
                                 stall was detected -- never push harder
                                 against a wall;
  odom_stale                     no fresh odometry sample this tick;
  override                       the mux output differs from what this node
                                 published (safety lane, joystick): the plant
                                 is not being driven by this command;
  reverse                        the reverse manoeuvre drives;
  breakaway                      the start-up bound below is active.
plus anti-windup: no integration in the direction that would push a command
already sitting at its ceiling (or a lifted floor) further out.

RESET (d_hat = 0, v_hat = v_meas, stall and start-up state cleared) on every
goal change, hold engage and release, e-stop and forward/reverse switch. The
command in force is re-synced to what the actuator last received rather than
zeroed: it is the actuator's physical state, and zeroing it while the car
moves would command a drop nobody planned (the same reason
_invalidate_move_state keeps last_u). On a hold, an abort or a gear change
the actuator last received zero, so it IS zero.

START-UP. Static friction: from rest the command must jump, not ramp. While
v_meas < offset_free_breakaway_speed_mps and the previous plan wanted to
move, c is LOWER-BOUNDED at v_breakaway_mps on the first
offset_free_breakaway_stages stages (x0's c is lifted to it: the jump is a
step of the command, outside the rate rows). The bound is released the tick
the car moves, and the MPC continues from the command it holds. "Wanted to
move" is read from the previous plan's command beyond the bounded stages, so
a solver that would rather stay put (an obstacle ahead) is never kicked.

BOUNDS. The command box is HARD and feasible by construction (command_box):
  c in [vMin (0 forward, never below -max_reverse_speed_mps),
        max_forward_speed_mps]
with the stop ramp's model-consistent cap (ramp box - d_hat), the stall cap,
the moving floor and the breakaway bound on top. The real speed gets a SOFT
cap at min(max_forward_speed_mps, ramp box): the measurement seeds it, and
noise alone must not make the QP infeasible. The command ceiling is the
/drive clamp it always was (see SpeedChannel.ceiling): a plant that needs
more than max_forward_speed_mps to reach vdes gets the ceiling, and
anti-windup keeps d_hat from integrating past what the ceiling can deliver.
"""

from dataclasses import dataclass
import math
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np

__all__ = [
    'FLOOR_ALL', 'FLOOR_OBJECT', 'FLOOR_OFF', 'SPEED_MODE_LEGACY',
    'SPEED_MODE_OFFSET_FREE', 'SpeedChannel', 'SpeedChannelConfig',
    'SpeedEstimator', 'command_box', 'lag_alpha', 'observer_gains',
    'offset_free_rank',
]

SPEED_MODE_LEGACY = 'legacy'
SPEED_MODE_OFFSET_FREE = 'offset_free'
SPEED_MODES = (SPEED_MODE_LEGACY, SPEED_MODE_OFFSET_FREE)

# Where the operating floor (min_moving_speed_mps) bounds the command.
FLOOR_OBJECT = 'object'     # object approaches while driving: today's rule
FLOOR_ALL = 'all'           # every forward-driving solve
FLOOR_OFF = 'off'
FLOOR_MODES = (FLOOR_OBJECT, FLOOR_ALL, FLOOR_OFF)

# The plan "wants to move" when its command beyond the breakaway stages
# reaches this fraction of the move's reference (or v_breakaway, if lower).
_INTENT_FRAC = 0.8


def lag_alpha(ts: float, tau: float) -> float:
    """Return 1 - exp(-ts / tau): the exact ZOH step of a first-order lag."""
    return 1.0 - math.exp(-float(ts) / float(tau))


def observer_gains(alpha: float, p_v: float, p_d: float) -> Tuple[float, float]:
    """
    Return (L_v, L_d) placing (I - L C) A at the real poles p_v, p_d.

    A = [[1 - alpha, alpha], [0, 1]], C = [1, 0]: det((I - LC) A) =
    (1 - L_v)(1 - alpha) and trace = (1 - L_v)(1 - alpha) + 1 - L_d alpha, so
    matching z^2 - (p_v + p_d) z + p_v p_d gives the closed form below.
    """
    l_v = 1.0 - p_v * p_d / (1.0 - alpha)
    l_d = (1.0 - p_v) * (1.0 - p_d) / alpha
    return l_v, l_d


def offset_free_rank(alpha: float) -> int:
    """Rank of [[I - A_v, -B_d], [C_v, C_d]] for the speed channel (2 = offset-free)."""
    m = np.array([[alpha, -alpha], [1.0, 0.0]], dtype=float)
    return int(np.linalg.matrix_rank(m))


@dataclass
class SpeedChannelConfig:
    """Everything the speed channel reads; MPC_corr fills it from stack_params."""

    mode: str = SPEED_MODE_OFFSET_FREE
    tau_s: float = 0.18
    delay_ticks: int = 0
    obs_bw_v_rad_s: float = 5.0
    obs_bw_d_rad_s: float = 2.0
    d_max_mps: float = 0.35
    est_min_speed_mps: float = 0.12
    est_min_cmd_mps: float = 0.10
    stall_cmd_mps: float = 0.20
    stall_speed_mps: float = 0.05
    stall_time_s: float = 0.8
    odom_max_age_s: float = 0.25
    breakaway_mps: float = 0.25
    breakaway_speed_mps: float = 0.08
    breakaway_stages: int = 3
    moving_floor: str = FLOOR_OBJECT
    speed_slack_weight: float = 1.0e3
    override_tol_mps: float = 0.03
    applied_max_age_s: float = 0.3

    def validate(self) -> None:
        """Raise ValueError on a configuration the channel cannot run."""
        if self.mode not in SPEED_MODES:
            raise ValueError(
                f'speed_command_mode must be one of {SPEED_MODES}, got {self.mode!r}')
        if self.moving_floor not in FLOOR_MODES:
            raise ValueError(
                f'offset_free_moving_floor must be one of {FLOOR_MODES}, '
                f'got {self.moving_floor!r}')
        if not (math.isfinite(self.tau_s) and self.tau_s > 0.0):
            raise ValueError(f'speed_lag_tau_s must be > 0, got {self.tau_s!r}')
        if int(self.delay_ticks) not in (0, 1):
            raise ValueError(f'speed_delay_ticks must be 0 or 1, got {self.delay_ticks!r}')
        for name in ('obs_bw_v_rad_s', 'obs_bw_d_rad_s'):
            if not (getattr(self, name) > 0.0):
                raise ValueError(f'offset_free_{name} must be > 0')
        for name in ('d_max_mps', 'est_min_speed_mps', 'est_min_cmd_mps', 'stall_cmd_mps',
                     'stall_speed_mps', 'stall_time_s', 'odom_max_age_s', 'breakaway_mps',
                     'breakaway_speed_mps', 'speed_slack_weight',
                     'override_tol_mps',
                     'applied_max_age_s'):
            value = float(getattr(self, name))
            if not (math.isfinite(value) and value >= 0.0):
                raise ValueError(f'{name} must be finite and >= 0, got {value!r}')
        if int(self.breakaway_stages) < 1:
            raise ValueError('offset_free_breakaway_stages must be >= 1')


# stack_params.yaml key -> SpeedChannelConfig field. The one mapping both
# MPC_corr (through declare_parameter) and the test harness (through
# get_value) read, so the two cannot configure different channels.
PARAM_KEYS = (
    ('speed_command_mode', 'mode', str),
    ('speed_lag_tau_s', 'tau_s', float),
    ('speed_delay_ticks', 'delay_ticks', int),
    ('offset_free_obs_bw_v_rad_s', 'obs_bw_v_rad_s', float),
    ('offset_free_obs_bw_d_rad_s', 'obs_bw_d_rad_s', float),
    ('offset_free_d_max_mps', 'd_max_mps', float),
    ('offset_free_est_min_speed_mps', 'est_min_speed_mps', float),
    ('offset_free_est_min_cmd_mps', 'est_min_cmd_mps', float),
    ('offset_free_stall_cmd_mps', 'stall_cmd_mps', float),
    ('offset_free_stall_speed_mps', 'stall_speed_mps', float),
    ('offset_free_stall_time_s', 'stall_time_s', float),
    ('offset_free_odom_max_age_s', 'odom_max_age_s', float),
    ('v_breakaway_mps', 'breakaway_mps', float),
    ('offset_free_breakaway_speed_mps', 'breakaway_speed_mps', float),
    ('offset_free_breakaway_stages', 'breakaway_stages', int),
    ('offset_free_moving_floor', 'moving_floor', str),
    ('offset_free_speed_slack_weight', 'speed_slack_weight', float),
    ('offset_free_override_tol_mps', 'override_tol_mps', float),
    ('offset_free_applied_max_age_s', 'applied_max_age_s', float),
)


def config_from(getter) -> SpeedChannelConfig:
    """Build the config from getter(stack_params key) -> value."""
    return SpeedChannelConfig(**{field: kind(getter(key)) for key, field, kind in PARAM_KEYS})


class SpeedEstimator:
    """The (v, d) observer of the module docstring. One update per control tick."""

    def __init__(self, alpha: float, l_v: float, l_d: float, d_max: float):
        self.alpha = float(alpha)
        self.l_v = float(l_v)
        self.l_d = float(l_d)
        self.d_max = float(d_max)
        self.v_hat: Optional[float] = None
        self.d_hat = 0.0
        self.innovation = 0.0
        self.wound = False

    def reset(self, v_meas: Optional[float] = None) -> None:
        """Forget d; start the speed estimate at the measurement."""
        self.v_hat = None if v_meas is None else float(v_meas)
        self.d_hat = 0.0
        self.innovation = 0.0
        self.wound = False

    def update(self, v_meas: Optional[float], c_eff: float, freeze: bool,
               wind: int = 0) -> bool:
        """
        Advance one tick on the command that acted; return whether d integrated.

        freeze holds d and re-syncs v_hat to the measurement. wind = +1 (the
        command sat at its ceiling) blocks d from falling, -1 (at a lifted
        floor) from rising: integrating there could only ask for a command
        the bounds will not give.
        """
        self.wound = False
        if v_meas is None:
            return False
        v_meas = float(v_meas)
        if self.v_hat is None:
            self.v_hat = v_meas
            self.innovation = 0.0
            return False
        v_pred = (1.0 - self.alpha) * self.v_hat + self.alpha * (float(c_eff) + self.d_hat)
        e = v_meas - v_pred
        self.innovation = e
        if freeze:
            self.v_hat = v_meas
            return False
        self.v_hat = v_pred + self.l_v * e
        step = self.l_d * e
        if (wind > 0 and step < 0.0) or (wind < 0 and step > 0.0):
            self.wound = True
            return False
        self.d_hat = float(np.clip(self.d_hat + step, -self.d_max, self.d_max))
        return True


def command_box(c0: float, a_prev: float, lo: Sequence[float], hi: Sequence[float],
                ts: float, t_first: float, limits: Dict) -> Tuple[np.ndarray, np.ndarray]:
    """
    Return (lo, hi) for c_1..c_N, widened just enough to be feasible.

    The rate rows bound a_k in [a_min, a_max] and its change per step by
    dAMin/dAMax * t (t_first for stage 0), and c_{k+1} = c_k + ts * a_k. A
    requested box the command cannot reach -- a cap that drops below where a
    jerk-limited command can get to by then, a floor above it -- is an
    infeasible QP, which _solve_rti discards whole. So a witness command is
    rolled out that holds still inside the requested band and heads for its
    nearest edge, as fast as those rows allow, outside it; each stage's box is
    widened to contain it. The witness satisfies every row, so the QP always
    has at least one solution, and wherever the band was reachable the box is
    exactly the request (the common case -- c0 inside, a_prev ~ 0 -- widens
    nothing). Where lo > hi was requested the cap wins.
    """
    hi = np.asarray(hi, dtype=float).copy()
    lo = np.minimum(np.asarray(lo, dtype=float), hi)
    n = len(lo)
    a_min, a_max = float(limits['a_min']), float(limits['a_max'])
    c, a = float(c0), float(a_prev)
    out_lo, out_hi = lo.copy(), hi.copy()
    for k in range(n):
        t_k = t_first if k == 0 else ts
        a_lo = max(a_min, a + float(limits['dAMin']) * t_k)
        a_hi = min(a_max, a + float(limits['dAMax']) * t_k)
        if a_lo > a_hi:
            # last_u outside the box (not reachable from MPC_corr): hold it.
            a_lo = a_hi = min(max(a, a_min), a_max)
        if c < lo[k]:
            a_des = (lo[k] - c) / ts
        elif c > hi[k]:
            a_des = (hi[k] - c) / ts
        else:
            a_des = 0.0
        a = min(max(a_des, a_lo), a_hi)
        c = c + ts * a
        out_lo[k] = min(lo[k], c)
        out_hi[k] = max(hi[k], c)
    return out_lo, out_hi


class SpeedChannel:
    """The node's speed law: observe, plan the bounds, publish, reset."""

    def __init__(self, cfg: SpeedChannelConfig, control_period: float,
                 prediction_dt: float, horizon: int, max_forward_speed: float,
                 max_reverse_speed: float, moving_floor_speed: float):
        cfg.validate()
        self.cfg = cfg
        self.ts = float(control_period)
        self.ts_pred = float(prediction_dt)
        self.N = int(horizon)
        self.max_forward = float(max_forward_speed)
        self.max_reverse = float(max_reverse_speed)
        self.floor_speed = float(moving_floor_speed)
        self.alpha = lag_alpha(self.ts, cfg.tau_s)
        self.alpha_pred = lag_alpha(self.ts_pred, cfg.tau_s)
        # First stage (0-based index into c_1..c_N / v_1..v_N) whose predicted
        # speed is less than half measurement: see _soft_cap.
        self.cap_first_stage = max(
            0, int(math.ceil(math.log(0.5) / math.log(1.0 - self.alpha_pred))) - 1)
        self.p_v = math.exp(-cfg.obs_bw_v_rad_s * self.ts)
        self.p_d = math.exp(-cfg.obs_bw_d_rad_s * self.ts)
        self.l_v, self.l_d = observer_gains(self.alpha, self.p_v, self.p_d)
        self.est = SpeedEstimator(self.alpha, self.l_v, self.l_d, cfg.d_max_mps)
        # Command in force at each of the last two observe() calls (newest
        # last): c_eff of the step just ended is [-1], or [-2] with one tick
        # of delay.
        self._in_force: List[float] = [0.0, 0.0]
        self._published = 0.0
        self._applied: Optional[float] = None
        self._applied_t: Optional[float] = None
        self.override = False
        self._stall_t = 0.0
        self.stalled = False
        self.stall_cap: Optional[float] = None
        self.breakaway_active = False
        self._intent = False
        self._sat = 0
        self.frozen = True
        self.freeze_reasons: Tuple[str, ...] = ()
        self.v_meas: Optional[float] = None
        self.last_reset = ''
        self.reset_count = 0
        # The latest plan, for command() and the status message.
        self.c0_used = 0.0
        self.speed_slack = 0.0
        self.box_stage1 = (math.nan, math.nan)
        self._req_stage1 = (0.0, math.inf)
        self._actuator_hi = math.inf
        self._vdes = 0.0
        # Whether the command in force is the one the last plan produced: the
        # plan's command rate then carries over into the next rate rows.
        self._planned = False
        self._planned_cmd = math.nan
        self._planned_rate = 0.0

    # ------------------------------------------------------------ properties

    @property
    def offset_free(self) -> bool:
        return self.cfg.mode == SPEED_MODE_OFFSET_FREE

    @property
    def d_hat(self) -> float:
        return self.est.d_hat

    @property
    def ceiling(self) -> float:
        """
        Return the forward command limit: max_forward_speed_mps, in both modes.

        Deliberately NOT raised to make room for the correction (operator
        decision, 2026-10-21): the car delivers ~60-76 % of its command only at
        low speed and about all of it at 0.5 m/s, so the command stays inside
        the /drive clamp it always had. Where vdes / gain would exceed it the
        command sits at the ceiling and anti-windup stops d from asking for
        more.
        """
        return self.max_forward

    @property
    def command_in_force(self) -> float:
        """Return x0's c: what the actuator last received (the mux output under override)."""
        return self._in_force[-1]

    @property
    def last_published(self) -> float:
        return self._published

    # ------------------------------------------------------------ inputs

    def note_applied(self, speed: float, now: float) -> None:
        """Record the mux output (/ackermann_drive): what the VESC was actually sent."""
        if speed is not None and math.isfinite(float(speed)):
            self._applied = float(speed)
            self._applied_t = float(now)

    def applied(self, now: float) -> Optional[float]:
        """Return the mux output if fresh, else None."""
        if self._applied_t is None or now - self._applied_t > self.cfg.applied_max_age_s:
            return None
        return self._applied

    def note_published(self, speed: float) -> None:
        """Record a command as it left _publish_drive (after the gate and the clamp)."""
        if abs(float(speed) - self._planned_cmd) > 1e-9:
            self._planned = False
        self._published = float(speed)

    def rate_in_force(self) -> float:
        """
        Return the command rate the next solve's rate rows start from (last_u[1]).

        The rate the previous plan actually published, (c1 - c0) / ts, when
        that command is the one in force; zero after anything else published
        (a stop path, a hold, the manoeuvre, an override, a clamp): the
        command then jumped, and no planned rate describes it.
        """
        return self._planned_rate if (self._planned and not self.override) else 0.0

    def reset(self, reason: str, v_meas: Optional[float] = None) -> None:
        """Forget d and the start-up/stall state (module docstring, RESET)."""
        self.est.reset(v_meas)
        self._stall_t = 0.0
        self.stalled = False
        self.stall_cap = None
        self.breakaway_active = False
        self._intent = False
        self._sat = 0
        self.last_reset = str(reason)
        self.reset_count += 1

    # ------------------------------------------------------------ per tick

    def observe(self, now: float, v_meas: Optional[float], odom_fresh: bool, *,
                hold: bool = False, estop: bool = False, mission_active: bool = True,
                stopping: bool = False, reverse: bool = False) -> None:
        """Advance the estimator over the step that just ended. Once per tick, first."""
        cfg = self.cfg
        applied = self.applied(now)
        self.override = (applied is not None
                         and abs(applied - self._published) > cfg.override_tol_mps)
        in_force = applied if self.override else self._published
        self._in_force = [self._in_force[-1], float(in_force)]
        c_eff = self._in_force[-1 - int(cfg.delay_ticks)]
        self.v_meas = None if v_meas is None else float(v_meas)

        # The stall detector, on the command that was in force.
        speed = abs(self.v_meas) if self.v_meas is not None else 0.0
        if in_force >= cfg.stall_cmd_mps and speed < cfg.stall_speed_mps:
            self._stall_t += self.ts
            if not self.stalled and self._stall_t >= cfg.stall_time_s - 1e-9:
                self.stalled = True
                self.stall_cap = float(in_force)
        else:
            self._stall_t = 0.0
            self.stalled = False
            self.stall_cap = None

        reasons = []
        for flag, name in ((hold, 'hold'), (estop, 'estop'),
                           (not mission_active, 'mission_inactive'),
                           (stopping, 'stopping'), (reverse, 'reverse'),
                           (not odom_fresh or self.v_meas is None, 'odom_stale'),
                           (self.override, 'override'),
                           (c_eff < cfg.est_min_cmd_mps, 'low_command'),
                           (speed < cfg.est_min_speed_mps, 'low_speed'),
                           (self.stalled, 'stall'),
                           (self.breakaway_active, 'breakaway')):
            if flag:
                reasons.append(name)
        self.freeze_reasons = tuple(reasons)
        integrated = self.est.update(self.v_meas if odom_fresh else None, c_eff,
                                     freeze=bool(reasons), wind=self._sat)
        if self.est.wound:
            self.freeze_reasons = self.freeze_reasons + ('anti_windup',)
        self.frozen = not integrated

    def plan(self, v_meas: float, a_prev: float, limits: Dict, *, vdes: float,
             ramp_box: Optional[float] = None, floor: bool = False,
             may_start: bool = True, t_first: Optional[float] = None
             ) -> Tuple[float, Dict]:
        """
        Return (c0, speed_model) for this tick's solve_mpc_step.

        ramp_box: the stop ramp's speed box (MPC_corr._stop_ramp_limits),
        None when the ramp is not binding. floor: the operating floor applies
        to this move (see offset_free_moving_floor). may_start: the move
        drives forward and no stop path owns the tick -- the only case in
        which the start-up bound may lift the command.
        """
        cfg = self.cfg
        n = self.N
        t_first = self.ts if t_first is None else float(t_first)
        d_hat = self.est.d_hat
        c0 = self.command_in_force

        hi_c = self.ceiling
        if self.stalled and self.stall_cap is not None:
            hi_c = min(hi_c, self.stall_cap)
        # The ACTUATOR limit, for anti-windup. The ramp's cap below is not one:
        # it is box - d_hat, it moves with the estimate, and blocking d at it
        # would freeze d at whatever value first reached it.
        self._actuator_hi = hi_c
        v_hi = self.max_forward
        if ramp_box is not None:
            v_hi = min(v_hi, float(ramp_box))
            hi_c = min(hi_c, max(float(ramp_box) - d_hat, 0.0))
        lo_c = max(float(limits['vMin']), -self.max_reverse)
        lo_all = lo_c
        if floor and self.floor_speed > 0.0:
            lo_all = max(lo_all, self.floor_speed)
        self.breakaway_active = bool(
            may_start and cfg.breakaway_mps > 0.0 and self._intent
            and abs(float(v_meas)) < cfg.breakaway_speed_mps)
        lo = np.full(n, lo_all)
        if self.breakaway_active:
            lo[:min(int(cfg.breakaway_stages), n)] = max(lo_all, cfg.breakaway_mps)
        hi = np.full(n, hi_c)
        # THE JUMP: a lower bound the command is below is a step of the
        # command, not something to ramp to (static friction does not let a
        # ramp start the car, and the floor is a rule about what is sent).
        lift = min(float(lo[0]), hi_c)
        if c0 < lift:
            c0 = lift
        c_lo, c_hi = command_box(c0, a_prev, lo, hi, self.ts_pred, t_first, limits)
        self.c0_used = float(c0)
        self._req_stage1 = (float(min(lo[0], hi[0])), float(hi[0]))
        self.box_stage1 = (float(c_lo[0]), float(c_hi[0]))
        self._vdes = float(vdes)
        speed_model = {
            'alpha': self.alpha_pred,
            'delay': int(cfg.delay_ticks),
            'd': float(d_hat),
            'c_lo': c_lo,
            'c_hi': c_hi,
            'v_hi': self._soft_cap(n, v_hi),
            'slack_weight': cfg.speed_slack_weight,
        }
        return float(c0), speed_model

    def _soft_cap(self, n: int, v_hi: float) -> np.ndarray:
        """
        Return the real-speed cap per stage, inf on the stages the measurement still owns.

        The predicted v_k carries (1 - alpha)^k of x0's MEASURED speed, noise
        and all, and a cap there acts on the noise: with sigma 0.02 m/s a cap
        from stage 1 cut every upward sample hard and left the car 0.012-0.016
        m/s below vdes on average (the rig, 3 seeds). From the first stage
        where that share is below one half (cap_first_stage) the cap acts on
        the plan instead; the hard command cap covers the stages before.
        """
        cap = np.full(n, float(v_hi))
        cap[:min(self.cap_first_stage, n)] = math.inf
        return cap

    def command(self, a0: float, info: Optional[Dict] = None) -> float:
        """
        Return the command to publish: c at stage 1, c0 + a0 * ts, inside the requested box.

        Also reads the plan's intent for the next tick's start-up decision.
        """
        cfg = self.cfg
        c1 = self.c0_used + float(a0) * self.ts
        lo1, hi1 = self._req_stage1
        c1 = min(max(c1, lo1), hi1)
        self._planned = True
        self._planned_cmd = float(c1)
        self._planned_rate = (float(c1) - self.c0_used) / self.ts
        self._sat = 0
        if c1 >= self._actuator_hi - 1e-4:
            self._sat = 1
        elif lo1 > 1e-6 and c1 <= lo1 + 1e-4:
            self._sat = -1
        self.speed_slack = float(info.get('speed_slack', 0.0)) if info else 0.0
        pred = None if info is None else info.get('v_cmd_pred')
        if pred is not None and len(pred):
            tail = np.asarray(pred, dtype=float)[min(int(cfg.breakaway_stages), len(pred) - 1):]
            want = min(cfg.breakaway_mps, _INTENT_FRAC * max(self._vdes, 0.0))
            self._intent = bool(want > 0.0 and float(np.max(tail)) >= want)
        else:
            self._intent = False
        return float(c1)

    def no_plan(self) -> None:
        """End the start-up intent: this tick published without solving."""
        self._intent = False
        self.breakaway_active = False
        self._sat = 0
        self._planned = False

    def model_for(self, ts: float) -> Dict:
        """Return the speed_model dict for one step of `ts` (the ERR log's prediction)."""
        return {'alpha': lag_alpha(ts, self.cfg.tau_s), 'delay': int(self.cfg.delay_ticks),
                'd': float(self.est.d_hat)}

    def status(self) -> Dict:
        """Return everything /mpc/speed_channel carries."""
        applied = self._applied if self._applied is not None else math.nan
        return {
            'mode': self.cfg.mode,
            'v_meas': math.nan if self.v_meas is None else self.v_meas,
            'v_hat': math.nan if self.est.v_hat is None else self.est.v_hat,
            'd_hat': self.est.d_hat,
            'innovation': self.est.innovation,
            'v_cmd': self._published,
            'v_cmd_in_force': self.c0_used if self.offset_free else self.command_in_force,
            'v_cmd_applied': applied,
            'override': self.override,
            'frozen': self.frozen,
            'freeze_reasons': '+'.join(self.freeze_reasons),
            'stalled': self.stalled,
            'stall_cap': math.nan if self.stall_cap is None else self.stall_cap,
            'breakaway_active': self.breakaway_active,
            'v_cmd_ceiling': self.ceiling,
            'speed_slack': self.speed_slack,
            'last_reset': self.last_reset,
            'reset_count': self.reset_count,
            'tau_s': self.cfg.tau_s,
            'alpha': self.alpha,
            'gain_l_v': self.l_v,
            'gain_l_d': self.l_d,
        }
