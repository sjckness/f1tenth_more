"""
Reverse driving chosen by the MPC itself: direction candidates, one controller.

Pure Python (no rclpy). MPC_corr.control_loop calls ReversePlanner.step in
place of solve_mpc_step when allow_reverse is true; with it false step() is
solve_mpc_step, call for call (test_reverse_flag_off_identity.py pins that).

THE IDEA. Each tick solves several convex QPs that are the same problem
except for a per-stage SIGN SCHEDULE on the speed state:

  A        all forward  (v_k >= 0)
  B        all reverse  (v_k <= 0)
  C_T      reverse for the stages up to T seconds, forward after

"v <= 0" includes 0: a reverse stage may stand still, it only forbids going
forward (and vice versa). The candidates are scored, a hysteresis selector
picks one direction, and only the first input of the winner is applied.

FEASIBLE BY CONSTRUCTION. A sign constraint cannot start at stage 0 when the
car is moving the other way: braking takes time under the acceleration box
AND the jerk (rate) rows. So the constraint of each segment starts at the
first stage from which a concrete jerk- and acceleration-limited stop
(stop_witness) already satisfies it for the rest of the segment; the stages
before stay on today's [vMin, vMax] box. The witness is then checked against
every final bound, so a candidate that reaches the QP has at least one
feasible speed profile. The witness ends within +-sign_tol of zero, which is
why a constrained forward stage is v >= -sign_tol rather than v >= 0
(sign_tol = 0.5 * jerk * ts^2, 0.01 m/s at the shipped 0.1 s step: far below
the VESC's 0.11 m/s telemetry deadband, so no direction change hides in it).

LINEARIZED AROUND ITS OWN NOMINAL. RTI linearizes once around the model
rolled out with the warm start. Each candidate gets a warm start whose speed
profile already has the candidate's signs: the previous winner, shifted, for
the candidate that won last tick; for the others the model rolled out with
the candidate's own speed profile. Either way the nominal speed is clipped
into the candidate's per-stage box, so a reverse stage is never linearized
around forward motion.

WHAT THE QPs SHARE. The same stage costs and constraints as today, with
three additions every candidate carries (mpc_solver's stage_spec): a
per-stage speed reference, a linear cost w_rev on reverse speed through a
slack, and rear bumper probes in w_obs on stages that can move backwards.

THE SPEED REFERENCE IS SYMMETRIC: +vdes on forward stages, -vdes on reverse
ones (the box caps reverse at v_rev_max), and on braking stages the sign of
the segment being braked into. A reference of 0 on reverse stages would make
a STANDING reverse candidate w_v * vdes^2 per stage cheaper than a standing
forward one -- about 7.5 at the shipped weights -- and a blocked car would
"reverse" for no reason but that. With the symmetric reference standing still
costs the same in both directions, and w_rev >= 2 * w_v * vdes (the shipped
default) makes reversing at any speed cost at least as much as standing:
reverse then wins only when the obstacle cost, the corridor or the
cost-to-go says it is worth it. The corridor is
extended backwards by corr_back_length_m along its first tangent, heading
unchanged (the car does not turn round to reverse).

Scoring and the terminal cost-to-go: see ReversePlanner._score. Selection:
see DirectionSelector.
"""

from dataclasses import dataclass, field
import math
import time
from typing import Dict, List, Optional, Sequence, Tuple

from mpc_controller.mpc_solver import solve_mpc_step
from mpc_controller.vehicle_model import f1tenth_state_fcn_dt_beta
import numpy as np

__all__ = [
    'FORWARD', 'REVERSE', 'CandidateSpec', 'DirectionSelector', 'ReverseConfig',
    'ReversePlanner', 'build_candidates', 'extend_corridor_backwards',
    'floor_moving_speed_signed', 'nominal_controls', 'rear_probe_offsets',
    'sign_tolerance', 'stop_witness',
]

FORWARD = 1
REVERSE = -1


@dataclass
class ReverseConfig:
    """Everything the reverse planner reads; MPC_corr fills it from params."""

    allow_reverse: bool = False
    # Reverse-to-forward switch times of the C candidates [s]. A time at or
    # past the horizon end is B and is dropped as a duplicate.
    switch_times_s: Tuple[float, ...] = (1.0, 2.0)
    v_rev_max: float = 0.3
    # Nominal reverse speed for the linearization point, as a fraction of
    # v_rev_max. Sets where the reverse stages are linearized, not the plan.
    v_rev_nominal_frac: float = 0.6
    w_rev: float = 4.3
    switch_threshold: float = 1.0
    switch_threshold_rel: float = 0.1
    commit_break_margin: float = 3.0
    min_dwell_s: float = 1.0
    # |v| below this is "stopped" for the commitment rule [m/s].
    stop_speed_mps: float = 0.05
    corr_back_length_m: float = 1.0
    # Rear probe points, metres along the heading from base_link (< 0 behind).
    rear_offsets_m: Tuple[float, ...] = ()
    # Weight on the terminal cost-to-go (V is in metres of path).
    w_ctg: float = 1.0
    # Trust-region curvature of the QP's local V model: (x, y) [1/m], psi.
    ctg_h_xy: float = 2.0
    ctg_h_psi: float = 0.5
    # Clip on the local V gradient magnitude (x, y) [cost/m] and psi.
    ctg_grad_max_xy: float = 6.0
    ctg_grad_max_psi: float = 3.0


def rear_probe_offsets(wheelbase: float, front_x: float, rear_x: float,
                       front_offsets: Sequence[float]) -> Tuple[float, ...]:
    """
    Rear w_obs probes, the front probe pattern mirrored about the bumpers.

    The front probes sit at base_link + L/2 + offset (object_geometry's
    SOLVER_FRONT_POINT_OFFSETS_M), i.e. at (L/2 + o - front_x) from the FRONT
    bumper. The rear ones keep the same spacing from the REAR bumper, mirrored:
    rear_x - (L/2 + o - front_x). With L 0.305, front 0.405, rear -0.082 that
    is (+0.0705, -0.0795, -0.2295) m from base_link. The front probes are not
    moved or changed.
    """
    return tuple(float(rear_x) - (float(wheelbase) / 2.0 + float(o) - float(front_x))
                 for o in front_offsets)


def sign_tolerance(limits: Dict, ts: float) -> float:
    """How far past zero a sign-constrained stage may go [m/s]: 0.5 * j * ts^2."""
    jerk = min(abs(float(limits['dAMin'])), abs(float(limits['dAMax'])))
    return 0.5 * jerk * float(ts) ** 2


def _release_floor(v: float, a: float, ts: float, jerk_up: float) -> float:
    """Lowest speed reached while raising a < 0 back to 0 at jerk_up per step."""
    if a >= 0.0:
        return v
    step = jerk_up * ts
    m = math.ceil(-a / step - 1e-12)
    k = m - 1
    return v + ts * (k * a + step * k * (k + 1) / 2.0)


def stop_witness(v0: float, a0: float, n: int, ts: float, t_first: float,
                 limits: Dict, eps: float) -> np.ndarray:
    """
    Speeds v_1..v_n of a fast, jerk- and acceleration-limited stop.

    Each stage brakes as hard as the acceleration box and the rate rows allow
    (stage 0's rate spans t_first, the control period, like the QP's), but
    never so hard that releasing the brake at full jerk would carry the speed
    more than eps past zero. Once stopped it holds within [-eps, eps]. It is a
    FEASIBILITY WITNESS for build_candidates, not a plan: any profile that
    satisfies the QP's speed rows proves the rows are satisfiable.
    """
    sgn = 1.0 if (v0 > 0.0 or (v0 == 0.0 and a0 >= 0.0)) else -1.0
    v, a = sgn * float(v0), sgn * float(a0)
    if sgn > 0.0:
        a_lo, a_hi = float(limits['a_min']), float(limits['a_max'])
        j_lo, j_hi = float(limits['dAMin']), float(limits['dAMax'])
    else:
        a_lo, a_hi = -float(limits['a_max']), -float(limits['a_min'])
        j_lo, j_hi = -float(limits['dAMax']), -float(limits['dAMin'])
    out = np.empty(n)
    for k in range(n):
        dt_r = t_first if k == 0 else ts
        lo = max(a_lo, a + j_lo * dt_r)
        hi = min(a_hi, a + j_hi * dt_r)
        if lo > hi:
            lo = hi = min(max(a, a_lo), a_hi)

        def ok(ac):
            return _release_floor(v + ac * ts, ac, ts, j_hi) >= -eps

        if ok(lo):
            ac = lo
        elif not ok(hi):
            ac = hi
        else:
            bad, good = lo, hi
            for _ in range(30):
                mid = 0.5 * (bad + good)
                if ok(mid):
                    good = mid
                else:
                    bad = mid
            ac = good
        v = v + ac * ts
        a = ac
        out[k] = sgn * v
    return out


@dataclass
class CandidateSpec:
    """One direction candidate: its per-stage speed box, reference and probes."""

    name: str
    direction: int          # sign of its first segment: what it does now
    final_direction: int    # sign of its last segment: the gear at x_N
    signs: np.ndarray       # +1 / -1 wanted per stage
    constrained: np.ndarray  # bool: the sign row is active at this stage
    v_lo: np.ndarray
    v_hi: np.ndarray
    vdes: np.ndarray
    rear: np.ndarray        # bool: rear probes in w_obs at this stage

    def stage_spec(self, rear_offsets) -> Dict:
        return {'v_lo': self.v_lo, 'v_hi': self.v_hi, 'vdes': self.vdes,
                'rear': self.rear, 'rear_offsets': tuple(rear_offsets)}

    def key(self):
        return (tuple(self.v_lo), tuple(self.v_hi), tuple(self.vdes))


def _segments(signs: np.ndarray) -> List[Tuple[int, int, int]]:
    out = []
    start = 0
    for k in range(1, len(signs) + 1):
        if k == len(signs) or signs[k] != signs[start]:
            out.append((start, k - 1, int(signs[start])))
            start = k
    return out


def build_candidates(v0: float, a0: float, n: int, ts: float, t_first: float,
                     limits: Dict, vdes: float, cfg: ReverseConfig
                     ) -> Tuple[List[CandidateSpec], List[Tuple[str, str]]]:
    """
    Build the tick's candidates, each with a feasible speed box; and the dropped ones.

    Returns (candidates, dropped), dropped being (name, reason) pairs:
    'duplicate' (same box as an earlier candidate, e.g. C_T with T past the
    horizon is B), 'no_reverse_stage' (the car cannot stop and reverse inside
    the horizon, or not before T), 'witness_outside_box' (the car is already
    outside today's [vMin, vMax] box, so no schedule is provably feasible).
    """
    tol = sign_tolerance(limits, ts)
    witness = stop_witness(v0, a0, n, ts, t_first, limits, 0.5 * tol)
    t_stage = (np.arange(n) + 1) * ts
    v_min, v_max = float(limits['vMin']), float(limits['vMax'])

    schedules = [('A', np.ones(n, dtype=int)), ('B', -np.ones(n, dtype=int))]
    for t_sw in cfg.switch_times_s:
        signs = np.where(t_stage <= float(t_sw) + 1e-9, -1, 1).astype(int)
        schedules.append((f'C{float(t_sw):.1f}', signs))

    out, dropped, seen = [], [], set()
    for name, signs in schedules:
        lo_c = np.where(signs > 0, -tol, -float(cfg.v_rev_max))
        hi_c = np.where(signs > 0, v_max, tol)
        constrained = np.zeros(n, dtype=bool)
        for s0, s1, _sg in _segments(signs):
            for k0 in range(s0, s1 + 1):
                seg = slice(k0, s1 + 1)
                if (np.all(witness[seg] >= lo_c[seg] - 1e-12)
                        and np.all(witness[seg] <= hi_c[seg] + 1e-12)):
                    constrained[seg] = True
                    break
        v_lo = np.where(constrained, lo_c, v_min)
        v_hi = np.where(constrained, hi_c, v_max)
        if not (np.all(witness >= v_lo - 1e-9) and np.all(witness <= v_hi + 1e-9)):
            dropped.append((name, 'witness_outside_box'))
            continue
        first_sign = int(signs[0])
        if first_sign < 0 and not np.any(constrained & (signs < 0)):
            dropped.append((name, 'no_reverse_stage'))
            continue
        # Speed reference: +-vdes in the direction of the segment the stage
        # is in, or is braking toward (see the module docstring).
        next_sign = np.empty(n, dtype=int)
        upcoming = int(signs[-1])
        for k in range(n - 1, -1, -1):
            if constrained[k]:
                upcoming = int(signs[k])
            next_sign[k] = upcoming
        vdes_k = np.where(next_sign > 0, float(vdes), -float(vdes))
        rear = (constrained & (signs < 0)) | (~constrained & (witness < 0.0))
        spec = CandidateSpec(
            name=name, direction=first_sign, final_direction=int(signs[-1]),
            signs=signs, constrained=constrained, v_lo=v_lo, v_hi=v_hi,
            vdes=vdes_k, rear=rear)
        if spec.key() in seen:
            dropped.append((name, 'duplicate'))
            continue
        seen.add(spec.key())
        out.append(spec)
    return out, dropped


def nominal_controls(spec: CandidateSpec, v0: float, steer: np.ndarray, ts: float,
                     v_rev_nominal: float, vdes: float, a_track: float = 1.5,
                     accel_seq: Optional[np.ndarray] = None) -> np.ndarray:
    """
    Flat [delta_0, a_0, ...] linearization point consistent with spec's signs.

    The speed profile is either rolled out from accel_seq (the previous
    winner's, shifted) or tracks the candidate's own targets (vdes forward,
    -v_rev_nominal reverse, 0 while braking) at a_track; in both cases each
    stage's speed is clipped into the candidate's box before the
    accelerations are recomputed from it. Nothing here needs to be feasible:
    it is where the QP is linearized, and only its signs matter.
    """
    n = len(spec.v_lo)
    v = float(v0)
    a_out = np.empty(n)
    for k in range(n):
        if accel_seq is not None:
            v_next = v + ts * float(accel_seq[k])
        else:
            if spec.constrained[k]:
                target = float(vdes) if spec.signs[k] > 0 else -float(v_rev_nominal)
            else:
                target = 0.0
            v_next = v + float(np.clip(target - v, -a_track * ts, a_track * ts))
        v_next = float(np.clip(v_next, spec.v_lo[k], spec.v_hi[k]))
        a_out[k] = (v_next - v) / ts
        v = v_next
    z = np.empty(2 * n)
    z[0::2] = np.asarray(steer, dtype=float)[:n]
    z[1::2] = a_out
    return z


def extend_corridor_backwards(corridor: Dict, length: float) -> Dict:
    """
    Return the corridor with a straight stretch of `length` prepended behind its start.

    Along the first tangent, with the first sample's normal and half-width:
    the HEADING REFERENCE IS UNCHANGED (tx, ty point the way the car faces;
    the car does not turn round to reverse). Sample spacing is the corridor's
    own first step. A shallow copy: obstacles and the scalar keys are shared.

    For the QP's nearest-index lateral row this is the same slab index 0
    already gave behind the start; it differs only where a curved corridor's
    far part is nearer to a point behind the car than its start is, and it
    gives compute_local_target-free consumers (markers, the cost-to-go)
    real geometry behind the car.
    """
    if length <= 0.0:
        return corridor
    xc = np.asarray(corridor['xc'], dtype=float)
    yc = np.asarray(corridor['yc'], dtype=float)
    if len(xc) < 2:
        return corridor
    ds = float(math.hypot(xc[1] - xc[0], yc[1] - yc[0])) or 0.05
    n_back = int(math.ceil(length / ds))
    tx0, ty0 = float(corridor['tx'][0]), float(corridor['ty'][0])
    nx0, ny0 = float(corridor['nx'][0]), float(corridor['ny'][0])
    hw0 = float(corridor['halfWidth'][0])
    steps = np.arange(n_back, 0, -1, dtype=float) * ds
    bx = xc[0] - steps * tx0
    by = yc[0] - steps * ty0
    out = dict(corridor)
    out['xc'] = np.concatenate([bx, xc])
    out['yc'] = np.concatenate([by, yc])
    out['tx'] = np.concatenate([np.full(n_back, tx0), corridor['tx']])
    out['ty'] = np.concatenate([np.full(n_back, ty0), corridor['ty']])
    out['nx'] = np.concatenate([np.full(n_back, nx0), corridor['nx']])
    out['ny'] = np.concatenate([np.full(n_back, ny0), corridor['ny']])
    out['halfWidth'] = np.concatenate([np.full(n_back, hw0), corridor['halfWidth']])
    out['xL'] = np.concatenate([bx + hw0 * nx0, corridor['xL']])
    out['yL'] = np.concatenate([by + hw0 * ny0, corridor['yL']])
    out['xR'] = np.concatenate([bx - hw0 * nx0, corridor['xR']])
    out['yR'] = np.concatenate([by - hw0 * ny0, corridor['yR']])
    out['backLength'] = float(n_back * ds)
    return out


def floor_moving_speed_signed(v_cmd: float, floor: float, direction: int,
                              v_meas: float, stop_speed: float) -> float:
    """
    Apply the operating floor in the selected direction only.

    Today (object approaches only) the published speed is max(v_cmd, floor):
    the car is not commanded below min_moving_speed_mps while it moves
    forward. Symmetric for reverse: min(v_cmd, -floor) while reversing. While
    the car still moves AGAINST the selected direction it is braking for a
    direction change, and the floor is not applied at all -- flooring then
    would command a jump straight through zero.
    """
    if direction == FORWARD:
        if v_meas < -stop_speed:
            return float(v_cmd)
        return max(float(v_cmd), float(floor))
    if v_meas > stop_speed:
        return float(v_cmd)
    return min(float(v_cmd), -float(floor))


class DirectionSelector:
    """
    Pick a direction with hysteresis, commitment and dwell (no dithering).

    Candidates are grouped by direction (what they do NOW: A forward, B and
    C_T reverse). Within the current direction the cheapest wins freely. To
    switch, the best of the other direction must beat the best of the
    current one by more than switch_threshold AND by more than
    switch_threshold_rel * |current|, and min_dwell_s must have passed since
    the last change. A switch made while the car still moves the old way
    starts a COMMITMENT: the new direction is kept until the car is stopped
    (|v| < stop_speed_mps), unless the old direction becomes cheaper by
    commit_break_margin (> switch_threshold). If the current direction has
    no feasible candidate at all, the other one is taken immediately.
    """

    def __init__(self, cfg: ReverseConfig):
        self.cfg = cfg
        self.direction = FORWARD
        self.committed = False
        self.last_switch_t = -math.inf
        self.n_switches = 0

    def select(self, results: Sequence['CandidateResult'], v_now: float,
               t_now: float) -> Tuple[Optional['CandidateResult'], str]:
        cfg = self.cfg
        ok = [r for r in results if r.ok]
        same = [r for r in ok if r.spec.direction == self.direction]
        other = [r for r in ok if r.spec.direction != self.direction]
        best_same = min(same, key=lambda r: r.score) if same else None
        best_other = min(other, key=lambda r: r.score) if other else None

        if self.committed:
            still_moving_old_way = (-self.direction * v_now) > cfg.stop_speed_mps
            if not still_moving_old_way:
                self.committed = False

        if best_same is None and best_other is None:
            return None, 'no_feasible_candidate'
        if best_same is None:
            self._switch(v_now, t_now)
            return best_other, 'current_direction_infeasible'
        if best_other is None:
            return best_same, 'only_current_direction'

        gain = best_same.score - best_other.score
        dwell_ok = (t_now - self.last_switch_t) >= cfg.min_dwell_s
        if self.committed:
            if gain > cfg.commit_break_margin and dwell_ok:
                self._switch(v_now, t_now)
                return best_other, 'commit_broken'
            return best_same, 'committed'
        if gain > cfg.switch_threshold and gain > cfg.switch_threshold_rel * abs(best_same.score):
            if not dwell_ok:
                return best_same, 'dwell'
            self._switch(v_now, t_now)
            return best_other, 'switched'
        return best_same, 'threshold_not_met'

    def _switch(self, v_now, t_now):
        old = self.direction
        self.direction = -old
        self.last_switch_t = t_now
        self.n_switches += 1
        # Braking for the change is under way iff the car still moves the old way.
        self.committed = (old * v_now) > self.cfg.stop_speed_mps


@dataclass
class CandidateResult:
    spec: CandidateSpec
    ok: bool
    u0: Optional[np.ndarray] = None
    info: Dict = field(default_factory=dict)
    j_qp: float = math.nan
    v_ctg: float = math.nan
    slack_pen: float = 0.0
    score: float = math.inf
    solve_ms: float = math.nan


class ReversePlanner:
    """
    The tick's multi-candidate solve, scoring and direction selection.

    `ctg` is an optional terminal cost-to-go (cost_to_go.CostToGo): an object
    with a `ready` flag, value(x, y, psi, gear) -> finite float and
    gradient(x, y, psi, gear) -> (dV/dx, dV/dy, dV/dpsi). While it is ready
    EVERY candidate trades the forward terminal pull (w_term to pref_nom,
    w_psi to psiRef) for it, so all of them are still scored on one
    objective. Without one (or not ready: no goal pose, e.g. goal_distance and
    drive moves) the candidates keep today's terminal and are scored on their
    QP cost alone.
    """

    def __init__(self, cfg: ReverseConfig, ctg=None):
        self.cfg = cfg
        self.ctg = ctg
        self.selector = DirectionSelector(cfg)
        self.last_winner: Optional[str] = None
        self.last_debug: Dict = {}

    def step(self, *, now_sec: float = 0.0, **kw) -> Tuple[np.ndarray, Dict]:
        """Solve the tick; a drop-in for solve_mpc_step, `now_sec` drives the dwell."""
        if not self.cfg.allow_reverse:
            return solve_mpc_step(**kw)
        t_tick = time.perf_counter()
        cfg = self.cfg
        x0 = np.asarray(kw['x0'], dtype=float)
        last_u = np.asarray(kw['last_u'], dtype=float)
        n = int(kw['horizon'])
        ts = float(kw['ts'])
        t_first = float(kw.get('control_period') or ts)
        limits = kw['limits']
        vdes = float(kw['vdes'])
        specs, dropped = build_candidates(
            float(x0[3]), float(last_u[1]), n, ts, t_first, limits, vdes, cfg)

        corridor = extend_corridor_backwards(kw['corridor'], cfg.corr_back_length_m)
        warm = kw.get('warm_start_z')
        if warm is not None and np.asarray(warm).size == 2 * n:
            warm = np.asarray(warm, dtype=float)
            steer_guess = warm[0::2]
        else:
            warm = None
            steer_guess = np.full(n, float(last_u[0]))

        results = []
        for spec in specs:
            if warm is not None and spec.name == self.last_winner:
                z_nom = nominal_controls(spec, x0[3], steer_guess, ts, 0.0, vdes,
                                         accel_seq=warm[1::2])
            else:
                z_nom = nominal_controls(
                    spec, x0[3], steer_guess, ts,
                    cfg.v_rev_max * cfg.v_rev_nominal_frac, vdes)
            results.append(self._solve(spec, z_nom, kw, corridor))

        winner, reason = self.selector.select(results, float(x0[3]), float(now_sec))
        fallback = winner is None
        if fallback:
            u0, info = solve_mpc_step(**kw)
            self.last_winner = None
        else:
            u0, info = winner.u0, dict(winner.info)
            self.last_winner = winner.spec.name
        self.last_debug = {
            'winner': None if fallback else winner.spec.name,
            'direction': 'forward' if self.selector.direction == FORWARD else 'reverse',
            'reason': 'fallback_all_failed' if fallback else reason,
            'committed': bool(self.selector.committed),
            'n_switches': int(self.selector.n_switches),
            'candidates': [{
                'name': r.spec.name, 'dir': r.spec.direction, 'ok': bool(r.ok),
                'score': _num(r.score), 'j_qp': _num(r.j_qp), 'v_ctg': _num(r.v_ctg),
                'slack': _num(r.slack_pen), 'solve_ms': _num(r.solve_ms),
                'k_start': int(np.argmax(r.spec.constrained)) if r.spec.constrained.any() else -1,
            } for r in results],
            'dropped': [{'name': nm, 'reason': rs} for nm, rs in dropped],
            'tick_ms': (time.perf_counter() - t_tick) * 1e3,
        }
        info['reverse'] = self.last_debug
        return u0, info

    def _solve(self, spec: CandidateSpec, z_nom: np.ndarray, kw: Dict,
               corridor: Dict) -> CandidateResult:
        cfg = self.cfg
        stage_spec = spec.stage_spec(cfg.rear_offsets_m)
        weights = dict(kw['weights'])
        weights['w_rev'] = cfg.w_rev
        n = int(kw['horizon'])
        if self._ctg_ready():
            x_nom_n = _rollout_last(kw['x0'], z_nom, n, float(kw['ts']), kw['params'])
            g = np.asarray(self.ctg.gradient(
                x_nom_n[0], x_nom_n[1], x_nom_n[2], spec.final_direction), dtype=float)
            gx, gy = _clip_mag(g[0], g[1], cfg.ctg_grad_max_xy)
            g = np.array([gx, gy, float(np.clip(g[2], -cfg.ctg_grad_max_psi,
                                                cfg.ctg_grad_max_psi))])
            weights['w_term'] = 0.0
            weights['w_psi'] = 0.0
            stage_spec['terminal'] = {
                'x_bar': x_nom_n[:3], 'g': g, 'w': cfg.w_ctg,
                'h': np.array([cfg.ctg_h_xy, cfg.ctg_h_xy, cfg.ctg_h_psi])}
        call = dict(kw)
        call.update(weights=weights, corridor=corridor, warm_start_z=z_nom,
                    stage_spec=stage_spec, solver='rti')
        t0 = time.perf_counter()
        try:
            u0, info = solve_mpc_step(**call)
        except Exception as exc:  # noqa: BLE001 -- one bad candidate must not stop the tick
            return CandidateResult(spec=spec, ok=False, info={'error': repr(exc)},
                                   solve_ms=(time.perf_counter() - t0) * 1e3)
        res = CandidateResult(spec=spec, ok=bool(info.get('success', False)), u0=u0,
                              info=info, solve_ms=(time.perf_counter() - t0) * 1e3)
        if not res.ok:
            return res
        self._score(res, kw, weights, stage_spec)
        return res

    def _ctg_ready(self) -> bool:
        return self.ctg is not None and bool(getattr(self.ctg, 'ready', False))

    def _score(self, res: CandidateResult, kw: Dict, weights: Dict, stage_spec: Dict):
        """
        Score = J_QP + V(x_N) (+ the boundary slack the QP bought).

        J_QP is the candidate's true cost (planner_cost_corridor at its
        solution, the number solve_mpc_step reports), so every candidate is
        measured with the same stage costs. With a cost-to-go, its terminal
        pull replaced w_term/w_psi in every candidate, and V is evaluated
        exactly (trilinear) at the predicted x_N in the candidate's FINAL
        gear. The boundary slack penalty is the QP's own: a candidate that
        clips a wall must not look cheap because the true cost does not see
        the slack.
        """
        cfg = self.cfg
        res.j_qp = float(res.info.get('cost', math.nan))
        w_s = float(kw.get('boundary_slack_weight', 0.0) or 0.0)
        res.slack_pen = float(sum(w_s * s * s + w_s * s
                                  for s in res.info.get('boundary_slack', [])))
        res.v_ctg = 0.0
        if 'terminal' in stage_spec and self.ctg is not None:
            x_n = np.asarray(res.info['x_pred'][-1], dtype=float)
            res.v_ctg = float(self.ctg.value(x_n[0], x_n[1], x_n[2],
                                             res.spec.final_direction))
        res.score = res.j_qp + res.slack_pen + cfg.w_ctg * res.v_ctg
        if not math.isfinite(res.score):
            res.ok = False


def _rollout_last(x0, z, n, ts, params):
    x = np.asarray(x0, dtype=float)
    for k in range(n):
        x = np.asarray(f1tenth_state_fcn_dt_beta(
            x, z[2 * k:2 * k + 2], ts, params['L'], params['lr']), dtype=float)
    return x


def _clip_mag(gx, gy, cap):
    mag = math.hypot(gx, gy)
    if mag <= cap or mag == 0.0:
        return float(gx), float(gy)
    return float(gx) * cap / mag, float(gy) * cap / mag


def _num(v):
    v = float(v)
    return v if math.isfinite(v) else None
