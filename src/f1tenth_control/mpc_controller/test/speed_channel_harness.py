"""
Closed-loop rig for the speed law, against a plant that is NOT the solver's model.

Not a test module (no test_ prefix): test_speed_channel_matrix.py and
test_speed_channel_scenarios.py import it, and running it as a script prints
the verification matrix that output/speed_gap/offset_free.md quotes:

    python3 test/speed_channel_harness.py [--out matrix.json]

WHAT IS REAL. solve_mpc_step (the RTI QP, N = 20, mpc_prediction_dt_s and
every weight and limit read from stack_params.yaml, exactly as MPC_corr reads
them), mpc_controller.speed_channel.SpeedChannel (the same object the node
owns, the same calls in the same order: observe, plan, command,
note_published), the stop ramp (stop_profile), and the /drive clamp and
negative-speed gate of MPC_corr._publish_drive. The corridor is the straight
one build_straight_corridor's drive branch produces, rebuilt from the car
every tick, and pref_nom is compute_local_target's lookahead point -- the
same reduction test_stop_ramp_closed_loop.py makes, for the same reason.

WHAT IS NOT THE SOLVER'S MODEL -- THE PLANT (Plant). The command channel of
the car as the identification (output/speed_gap/offset_free.md, section 1)
describes it, each part switchable and each deliberately different from
what the solver assumes:
  * a steady-state gain on the command, constant or speed-dependent (a
    callable: the fitted curve);
  * a first-order lag with ITS OWN tau (the matrix runs 0.5x, 1x, 2x the
    nominal the solver uses);
  * a pure delay of 0..2 control ticks (the solver models none, or one);
  * static friction: from rest the car does not move until the command
    exceeds `stiction` (the matrix's 0.15 m/s deadband), and sticks again
    when it comes to rest below it;
  * Gaussian speed noise on the measurement (sigma 0.02 m/s), seeded;
  * blocked wheels on request (the speed is held at zero, whatever the
    command).
"""

import argparse
import json
import math
import os
import sys
import time

from f1tenth_description.vehicle_frames import base_front_x
from f1tenth_params.param_defaults import get_value
from mpc_controller.drive_limits import clamp_drive_speed
from mpc_controller.MPC_corr import lookahead_clamp_length
from mpc_controller.mpc_solver import shift_warm_start, solve_mpc_step
from mpc_controller.speed_channel import config_from, SpeedChannel
from mpc_controller.stop_profile import StopRampLatch, StopRampProfile
from mpc_controller.vehicle_model import f1tenth_state_fcn_dt_beta
import numpy as np

TS = 0.1
HORIZON = 20
TS_PRED = float(get_value('mpc_prediction_dt_s'))
PARAMS = {'L': float(get_value('mpc_wheelbase_m')), 'lr': 0.17}
MAX_FORWARD = float(get_value('max_forward_speed_mps'))
MAX_REVERSE = float(get_value('max_reverse_speed_mps'))
V_MIN = float(get_value('min_moving_speed_mps'))

# MPC_corr's self.limits (delta from stack_params; the rest are literals there).
LIMITS = {
    'delta_min': float(get_value('mpc_steering_angle_min_rad')),
    'delta_max': float(get_value('mpc_steering_angle_max_rad')),
    'a_min': -2.0, 'a_max': 3.0,
    'dDeltaMin': -0.5, 'dDeltaMax': 0.5,
    'dAMin': -2.0, 'dAMax': 2.0,
    'vMin': 0.0, 'vMax': 3.0,
}

WEIGHTS = {
    key: float(get_value('mpc_' + key))
    for key in ('w_term', 'w_v', 'w_psi', 'w_u_a', 'w_du_delta', 'w_du_a',
                'w_delta0', 'w_obs', 'w_corr', 'w_line', 'w_psi_stage')
}

# The speed-dependent steady-state gain fitted to the archive
# (output/speed_gap/offset_free.md section 1.3): the car falls a roughly
# constant 280-400 ERPM short of its setpoint rather than a fixed fraction, so
# the offset model y_ss = K1 (u - u0) fits better than a constant gain
# (leave-one-run-out 90 vs 116 ERPM). K1 1.04 [1.00, 1.10], u0 0.086 m/s
# [0.068, 0.107] at speed_to_erpm_gain 4516.9 -- 0.77 at 0.33 m/s, 0.86 at 0.5.
# Identified lag (same section): the plant's nominal tau, against which the
# matrix scales 0.5x / 1x / 2x. The model's own tau is speed_lag_tau_s.
TAU_IDENTIFIED = 0.176
FITTED_K1 = 1.042
FITTED_U0 = 0.086


def FITTED_GAIN(u):  # noqa: N802 -- a constant of the rig, named like one
    """Steady speed [m/s] under a constant command u >= 0 (offset model)."""
    return FITTED_K1 * max(float(u) - FITTED_U0, 0.0)


BUMPER_X = base_front_x()
CORR_L = 3.0
CORR_HALF_WIDTH = 0.64
LOOKAHEAD_FRAC = 0.5
LOOKAHEAD_REACH_MARGIN = 1.25


def nominal_config(**overrides):
    """Return the shipped SpeedChannelConfig (stack_params), with overrides."""
    cfg = config_from(get_value)
    for key, value in overrides.items():
        setattr(cfg, key, value)
    return cfg


class Plant:
    """The car's command -> speed channel; see the module docstring."""

    def __init__(self, gain=1.0, tau=0.176, delay=0, stiction=0.0, noise=0.02,
                 seed=0, x0=(0.0, 0.0, 0.0), braking=None):
        self.gain = gain
        self.tau = float(tau)
        self.delay = int(delay)
        self.stiction = float(stiction)
        self.noise = float(noise)
        self.rng = np.random.default_rng(seed)
        self.x = np.array([x0[0], x0[1], x0[2], 0.0], dtype=float)
        # Commands still in flight: `delay` of them, so the one popped each
        # step is the one published `delay` ticks ago (the new one at 0).
        self.queue = [0.0] * self.delay
        self.blocked = False
        self.moving = False
        # (latency s, decel m/s^2): under a zero command the car keeps its
        # speed for the latency, then brakes at the decel -- the fit to the
        # archived LiDAR-measured stops (test_object_approach_closed_loop's
        # BRAKING_MEASURED, 0.144 s / 1.5 m/s^2). None: the lag brakes too.
        self.braking = braking
        self._zero_for = 0.0

    @property
    def v(self):
        return float(self.x[3])

    def steady(self, c):
        """Speed the car settles at under a constant command c."""
        if callable(self.gain):
            return float(np.sign(c)) * float(self.gain(abs(c)))
        return float(self.gain) * float(c)

    def measure(self):
        return self.v + (float(self.rng.normal(0.0, self.noise)) if self.noise else 0.0)

    def step(self, command, steer, ts=TS):
        """Apply `command` (published now) for one control period."""
        self.queue.append(float(command))
        c_eff = self.queue.pop(0)
        v = self.v
        self._zero_for = self._zero_for + ts if c_eff == 0.0 else 0.0
        if self.blocked:
            v_next = 0.0
            self.moving = False
        elif self.braking is not None and c_eff == 0.0:
            latency, decel = self.braking
            braking_for = max(0.0, self._zero_for - latency)
            v_next = max(0.0, abs(v) - decel * min(ts, braking_for)) * (1.0 if v >= 0 else -1.0)
            self.moving = v_next != 0.0
        elif not self.moving and abs(c_eff) <= self.stiction:
            v_next = 0.0
        else:
            self.moving = True
            alpha = 1.0 - math.exp(-ts / self.tau)
            v_next = v + alpha * (self.steady(c_eff) - v)
            if self.stiction > 0.0 and abs(v_next) < 0.02 and abs(c_eff) <= self.stiction:
                v_next = 0.0
                self.moving = False
        # Positions on the step's MEAN speed (exact for a linear change): the
        # model's explicit Euler on v_k would add ~2 cm to every stop.
        x_mid = self.x.copy()
        x_mid[3] = 0.5 * (v + v_next)
        u = np.array([float(steer), 0.0], dtype=float)
        self.x = np.asarray(f1tenth_state_fcn_dt_beta(
            x_mid, u, ts, PARAMS['L'], PARAMS['lr']), dtype=float)
        self.x[3] = v_next


def straight_corridor(origin, psi, length=CORR_L, n=120):
    """build_straight_corridor's drive-branch geometry, from the car."""
    s = np.linspace(0.0, length, n)
    xc = origin[0] + s * math.cos(psi)
    yc = origin[1] + s * math.sin(psi)
    tx = np.full(n, math.cos(psi))
    ty = np.full(n, math.sin(psi))
    return {
        'xc': xc, 'yc': yc, 'tx': tx, 'ty': ty, 'nx': -ty, 'ny': tx,
        'halfWidth': np.full(n, CORR_HALF_WIDTH), 'L': float(length),
        'obstacles_world': [], 'd_safe': 0.32, 'car_radius': 0.20,
        'avoidance_margin': 0.12,
        'Pend': np.array([xc[-1], yc[-1]], dtype=float), 'psiRef': psi,
    }


def pref_nom(corridor, vdes):
    """compute_local_target's lookahead point with no obstacles to deflect."""
    length = float(corridor['L'])
    lookahead = max(LOOKAHEAD_FRAC * length,
                    lookahead_clamp_length(HORIZON, TS_PRED, vdes, LOOKAHEAD_REACH_MARGIN))
    frac = min(lookahead, length) / length
    idx = min(int(round(frac * (len(corridor['xc']) - 1))), len(corridor['xc']) - 1)
    return np.array([corridor['xc'][idx], corridor['yc'][idx]], dtype=float)


def ramp_limits(v_ref, v_now, a_dec):
    """MPC_corr._stop_ramp_limits: the speed box, or None when not binding."""
    v_max = float(LIMITS['vMax'])
    if v_ref is None or not (v_ref < v_max):
        return None
    box = max(float(v_ref), float(v_now) - a_dec * TS_PRED)
    return None if box >= v_max else box


class Trace:
    """Per-tick record of one run."""

    FIELDS = ('t', 'x', 'v_true', 'v_meas', 'v_cmd', 'd_hat', 'frozen', 'stalled',
              'breakaway', 'solve_ms', 'source')

    def __init__(self):
        for f in self.FIELDS:
            setattr(self, f, [])
        self.events = []
        self.rest_x = None

    def add(self, **row):
        for f in self.FIELDS:
            getattr(self, f).append(row.get(f, math.nan))

    def arrays(self):
        return {f: np.asarray(getattr(self, f)) for f in self.FIELDS if f != 'source'}


def run(mode='offset_free', vdes=0.5, plant=None, duration=12.0, cfg=None,
        goal_distance=None, hold=(), block=(), manoeuvre=None, seed=0,
        stop_after=None, weights=None, stuck=None, front_dist=None, wall=None):
    """
    Drive one scenario; return its Trace.

    mode        'offset_free' or 'legacy' (speed_command_mode).
    goal_distance  a goal_distance move with the stop ramp (else open-ended drive).
    wall        (wall_x, target, sense_delay_ticks): the mission's front_clearance
                stop -- the ramp aims at `target` metres between the front
                bumper and a wall at wall_x, on a wall distance that is
                sense_delay_ticks old (MPC_corr._wall_stop_d_rem /
                _wall_stop_tick); trace.rest_gap is the gap at rest.
    hold        (t_on, t_off): /mpc/hold engaged, then released.
    block       (t_on, t_off): wheels blocked.
    manoeuvre   (t_start, [(speed, seconds), ...]): the reverse manoeuvre takes
                over -- MPC_corr's before_forward path -- then hands back.
    """
    plant = plant or Plant(seed=seed)
    cfg = cfg or nominal_config()
    cfg.mode = mode
    channel = SpeedChannel(cfg, TS, TS_PRED, HORIZON, MAX_FORWARD, MAX_REVERSE, V_MIN)
    profile = StopRampProfile(
        a_dec=float(get_value('stop_ramp_a_dec_mps2')),
        a_stop=float(get_value('stop_ramp_a_stop_mps2')),
        tau_latency_s=float(get_value('stop_ramp_tau_latency_s')), v_min=V_MIN)
    latch = StopRampLatch(profile)
    trace = Trace()
    trace.stuck = []
    x_hist = [float(plant.x[0])]
    last_u = np.zeros(2)
    warm = None
    x_start = float(plant.x[0])
    goal_reached = False
    was_held = False
    was_manoeuvre = False
    man_plan = []
    if manoeuvre is not None:
        t_m, steps = manoeuvre
        tick = int(round(t_m / TS))
        for speed, seconds in steps:
            n = int(round(seconds / TS))
            man_plan.extend([(tick + i, speed) for i in range(n)])
            tick += n
    man_by_tick = dict(man_plan)
    n_ticks = int(round(duration / TS))
    for k in range(n_ticks):
        t = k * TS
        if block:
            plant.blocked = block[0] <= t < block[1]
        held = bool(hold) and hold[0] <= t < hold[1]
        in_manoeuvre = k in man_by_tick
        v_meas = plant.measure()
        stopping = goal_reached or latch.latched
        channel.observe(t, v_meas, True, hold=held, stopping=stopping,
                        reverse=in_manoeuvre)
        # MPC_corr's resets: hold engage and release, the forward/reverse switch.
        if held != was_held:
            channel.reset('hold' if held else 'hold_release', v_meas)
            trace.events.append((t, 'hold' if held else 'hold_release'))
        if in_manoeuvre != was_manoeuvre:
            channel.reset('reverse_switch', v_meas)
            trace.events.append((t, 'reverse_switch'))
        was_held, was_manoeuvre = held, in_manoeuvre

        source = 'forward'
        solve_ms = math.nan
        if held or goal_reached:
            cmd, steer = 0.0, 0.0
            channel.no_plan()
            source = 'stop'
        elif in_manoeuvre:
            cmd, steer = float(man_by_tick[k]), 0.0
            last_u = np.array([0.0, 0.0])
            channel.no_plan()
            source = 'manoeuvre'
        else:
            vdes_eff = vdes
            d_rem = None
            if goal_distance is not None:
                d_rem = goal_distance - (float(plant.x[0]) - x_start)
            if wall is not None:
                wall_x, target, delay_ticks = wall
                seen = x_hist[max(0, len(x_hist) - 1 - int(delay_ticks))]
                d_est = wall_x - (seen + BUMPER_X)
                d_wall = profile.predict(d_est, v_meas) - target
                d_rem = d_wall if d_rem is None else min(d_rem, d_wall)
            if d_rem is not None:
                decision = profile.decide(d_rem, vdes)
                if latch.update(decision):
                    goal_reached = True
                    cmd, steer = 0.0, 0.0
                    channel.no_plan()
                    source = 'stop'
                else:
                    vdes_eff = decision.v_ref
            # MPC_corr._stop_ramp_limits runs on EVERY forward tick with the
            # ramp enabled (stack default), stopping point or not: with no
            # stopping point v_ref is vdes, so the box is vdes itself.
            box = ramp_limits(vdes_eff, v_meas, profile.a_dec)
            if source == 'forward':
                x_now = np.array([plant.x[0], plant.x[1], plant.x[2], v_meas])
                corridor = straight_corridor((float(plant.x[0]), float(plant.x[1])),
                                             float(plant.x[2]))
                kw = {'last_u': last_u, 'pref_nom': pref_nom(corridor, vdes),
                      'corridor': corridor, 'horizon': HORIZON, 'ts': TS_PRED,
                      'params': PARAMS, 'weights': weights or WEIGHTS, 'obstacles': [],
                      'dmin': 0.32, 'vdes': vdes_eff, 'solver': 'rti', 'warm_start_z': warm,
                      'control_period': TS}
                t0 = time.perf_counter()
                if channel.offset_free:
                    kw['last_u'] = np.array([last_u[0], channel.rate_in_force()])
                    c0, sm = channel.plan(v_meas, kw['last_u'][1], LIMITS, vdes=vdes_eff,
                                          ramp_box=box)
                    u0, info = solve_mpc_step(
                        x0=np.append(x_now, c0), limits=LIMITS, speed_model=sm, **kw)
                    cmd = channel.command(float(u0[1]), info)
                    last_u = np.array([float(u0[0]), channel.rate_in_force()])
                else:
                    limits = LIMITS if box is None else dict(LIMITS, vMax=box)
                    u0, info = solve_mpc_step(x0=x_now, limits=limits, **kw)
                    cmd = v_meas + float(u0[1]) * TS
                    last_u = np.asarray(u0, dtype=float)
                solve_ms = (time.perf_counter() - t0) * 1e3
                if stuck is not None:
                    # reverse_manoeuvre's StuckDetector, fed exactly as
                    # ReverseSupervisor.after_forward feeds it.
                    xp = info.get('x_pred')
                    progress = (None if xp is None or not len(xp) else
                                float(math.hypot(xp[-1][0] - x_now[0], xp[-1][1] - x_now[1])))
                    fd = front_dist(t) if front_dist is not None else None
                    if stuck.update(t, bool(info.get('success')), True, v_meas, fd,
                                    progress, False):
                        trace.stuck.append(t)
                warm = (shift_warm_start(info['zopt'], HORIZON, step_fraction=TS / TS_PRED)
                        if info.get('success') else None)
                steer = float(u0[0])
        # _publish_drive: the gate, then the clamp (ceiling in offset_free).
        if cmd < 0.0 and source != 'manoeuvre':
            cmd = 0.0
        cmd, _clamped = clamp_drive_speed(cmd, channel.ceiling, MAX_REVERSE)
        channel.note_published(cmd)
        trace.add(t=t, x=float(plant.x[0]) - x_start, v_true=plant.v, v_meas=v_meas,
                  v_cmd=cmd, d_hat=channel.d_hat, frozen=float(channel.frozen),
                  stalled=float(channel.stalled), breakaway=float(channel.breakaway_active),
                  solve_ms=solve_ms, source=source)
        plant.step(cmd, steer)
        x_hist.append(float(plant.x[0]))
        if stop_after is not None and stop_after(trace, plant):
            break
    trace.rest_x = float(plant.x[0]) - x_start
    if wall is not None:
        trace.rest_gap = wall[0] - (float(plant.x[0]) + BUMPER_X)
    return trace


def metrics(trace, vdes, window=(8.0, 12.0), t_start=0.0, target=None):
    """
    Return steady-state error, rise time, overshoot and the command peak of a run.

    The rise is 10 -> 90 % of `target` (default vdes: pass the speed the
    ceiling can deliver where that is lower); the overshoot is above vdes.
    """
    ref = vdes if target is None else float(target)
    a = trace.arrays()
    t = a['t']
    sel = (t >= window[0]) & (t < window[1])
    v_ss = float(np.mean(a['v_true'][sel]))
    after = t >= t_start
    v = a['v_true'][after]
    tt = t[after] - t_start
    rise = math.nan
    lo = np.nonzero(v >= 0.1 * ref)[0]
    hi = np.nonzero(v >= 0.9 * ref)[0]
    if len(lo) and len(hi):
        rise = float(tt[hi[0]] - tt[lo[0]])
    return {
        'v_ss': v_ss,
        'ss_error': v_ss - vdes,
        'rise_s': rise,
        'overshoot': max(0.0, float(np.max(v)) - vdes) / vdes,
        'v_cmd_peak': float(np.max(a['v_cmd'])),
        'v_cmd_ss': float(np.mean(a['v_cmd'][sel])),
        'd_hat_ss': float(np.mean(a['d_hat'][sel])),
    }


def _main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.strip().splitlines()[0])
    parser.add_argument('--out', default=None)
    args = parser.parse_args(argv)
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    import test_speed_channel_matrix as matrix  # noqa: E402
    rows = matrix.run_matrix(verbose=True)
    if args.out:
        with open(args.out, 'w') as fh:
            json.dump(rows, fh, indent=1)


if __name__ == '__main__':
    _main()
