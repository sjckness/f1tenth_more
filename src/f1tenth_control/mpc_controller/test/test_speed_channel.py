"""
Unit tests of the offset-free speed law: model, observer, bounds, freeze, reset.

mpc_controller/speed_channel.py has the design. The closed-loop behaviour
against a plant that is not the model is in test_speed_channel_matrix.py and
test_speed_channel_scenarios.py; this file pins the pieces those runs are made
of, one property at a time.

Run standalone: python3 -m pytest test/test_speed_channel.py -v
"""

import math

from f1tenth_params.param_defaults import get_value
from mpc_controller.mpc_solver import solve_mpc_step
from mpc_controller.speed_channel import (
    command_box, config_from, lag_alpha, observer_gains, offset_free_rank, PARAM_KEYS,
    SPEED_MODE_LEGACY, SPEED_MODE_OFFSET_FREE, SpeedChannel, SpeedChannelConfig,
    SpeedEstimator)
from mpc_controller.vehicle_model import (
    f1tenth_state_fcn_dt_beta, f1tenth_state_fcn_dt_beta_lag,
    f1tenth_state_fcn_dt_beta_lag_jacobian)
import numpy as np
import pytest
from scipy.optimize import approx_fprime

TS = 0.1
L, LR = 0.305, 0.17
LIMITS = {'delta_min': -0.309, 'delta_max': 0.319, 'a_min': -2.0, 'a_max': 3.0,
          'dDeltaMin': -0.5, 'dDeltaMax': 0.5, 'dAMin': -2.0, 'dAMax': 2.0,
          'vMin': 0.0, 'vMax': 3.0}


def _channel(**overrides):
    cfg = SpeedChannelConfig(**overrides)
    return SpeedChannel(cfg, TS, TS, 20, 0.5, 0.4, 0.4)


# --------------------------------------------------------------- the model

class TestTheLagModel:

    @pytest.mark.parametrize('delay', [0, 1])
    def test_the_command_integrates_its_rate_and_the_speed_lags_it(self, delay):
        sm = {'alpha': 0.3, 'delay': delay, 'd': -0.05}
        x = [0.0, 0.0, 0.0, 0.2, 0.4]
        x1 = f1tenth_state_fcn_dt_beta_lag(x, [0.0, 1.0], TS, L, LR, sm)
        assert x1[4] == pytest.approx(0.5)
        c_eff = 0.4 if delay else 0.5
        assert x1[3] == pytest.approx(0.2 + 0.3 * (c_eff - 0.05 - 0.2))

    def test_the_kinematics_are_the_legacy_models_driven_by_the_real_speed(self):
        sm = {'alpha': 0.3, 'delay': 0, 'd': 0.0}
        x = [1.0, 2.0, 0.3, 0.45, 0.6]
        u = [0.1, 0.7]
        legacy = f1tenth_state_fcn_dt_beta(x[:4], u, TS, L, LR)
        lag = f1tenth_state_fcn_dt_beta_lag(x, u, TS, L, LR, sm)
        assert lag[:3] == pytest.approx(legacy[:3])

    @pytest.mark.parametrize('delay', [0, 1])
    @pytest.mark.parametrize('x, u', [
        ([0.0, 0.0, 0.0, 0.3, 0.4], [0.0, 0.5]),
        ([1.0, -0.5, 0.7, 0.5, 0.45], [0.2, -1.0]),
        ([0.3, 0.2, -1.2, 0.1, 0.5], [-0.25, 2.0]),
    ])
    def test_the_jacobian_matches_finite_differences(self, x, u, delay):
        sm = {'alpha': 0.283, 'delay': delay, 'd': -0.1}
        A, B = f1tenth_state_fcn_dt_beta_lag_jacobian(x, u, TS, L, LR, sm)
        eps = 1e-7
        for i in range(5):
            num = approx_fprime(np.asarray(x, float), lambda xx: f1tenth_state_fcn_dt_beta_lag(
                xx, u, TS, L, LR, sm)[i], eps)
            assert A[i] == pytest.approx(num, abs=1e-5)
            numu = approx_fprime(np.asarray(u, float), lambda uu: f1tenth_state_fcn_dt_beta_lag(
                x, uu, TS, L, LR, sm)[i], eps)
            assert B[i] == pytest.approx(numu, abs=1e-5)

    def test_alpha_is_the_exact_zoh_step_not_euler(self):
        assert lag_alpha(0.1, 0.3) == pytest.approx(1.0 - math.exp(-1.0 / 3.0))
        # Stable where Euler's ts / tau would exceed 1.
        assert 0.0 < lag_alpha(0.1, 0.05) < 1.0


# --------------------------------------------------------------- the observer

class TestTheObserver:

    @pytest.mark.parametrize('tau', [0.15, 0.3, 0.6])
    @pytest.mark.parametrize('bw', [(5.0, 2.0), (3.0, 1.0), (8.0, 4.0)])
    def test_the_error_dynamics_sit_on_the_requested_poles(self, tau, bw):
        a = lag_alpha(TS, tau)
        p_v, p_d = math.exp(-bw[0] * TS), math.exp(-bw[1] * TS)
        l_v, l_d = observer_gains(a, p_v, p_d)
        A = np.array([[1.0 - a, a], [0.0, 1.0]])
        LC = np.array([[l_v, 0.0], [l_d, 0.0]])
        eig = np.sort(np.linalg.eigvals((np.eye(2) - LC) @ A).real)
        assert eig == pytest.approx(np.sort([p_v, p_d]), abs=1e-9)

    @pytest.mark.parametrize('alpha', [1e-3, 0.1, 0.283, 0.6, 0.99])
    def test_the_augmented_system_is_observable_and_offset_free(self, alpha):
        A = np.array([[1.0 - alpha, alpha], [0.0, 1.0]])
        C = np.array([[1.0, 0.0]])
        assert np.linalg.matrix_rank(np.vstack([C, C @ A])) == 2
        assert offset_free_rank(alpha) == 2

    @pytest.mark.parametrize('d_true', [-0.19, -0.12, 0.0, 0.07])
    def test_it_converges_to_the_true_disturbance(self, d_true):
        a = lag_alpha(TS, 0.3)
        l_v, l_d = observer_gains(a, math.exp(-0.5), math.exp(-0.2))
        est = SpeedEstimator(a, l_v, l_d, d_max=0.35)
        v, c = 0.0, 0.5
        for _ in range(100):
            v = v + a * (c + d_true - v)
            est.update(v, c, freeze=False)
        assert est.d_hat == pytest.approx(d_true, abs=1e-4)
        assert est.v_hat == pytest.approx(v, abs=1e-4)

    def test_the_estimate_saturates(self):
        a = lag_alpha(TS, 0.3)
        est = SpeedEstimator(a, *observer_gains(a, 0.6, 0.8), d_max=0.1)
        v = 0.0
        for _ in range(200):
            v = v + a * (0.5 - 0.3 - v)
            est.update(v, 0.5, freeze=False)
        assert est.d_hat == pytest.approx(-0.1)

    def test_a_freeze_holds_d_and_resyncs_the_speed(self):
        a = lag_alpha(TS, 0.3)
        est = SpeedEstimator(a, *observer_gains(a, 0.6, 0.8), d_max=0.35)
        est.update(0.2, 0.5, freeze=False)
        est.update(0.25, 0.5, freeze=False)
        d = est.d_hat
        assert not est.update(0.0, 0.5, freeze=True)
        assert est.d_hat == d
        assert est.v_hat == 0.0

    def test_anti_windup_blocks_only_the_direction_that_asks_for_more(self):
        a = lag_alpha(TS, 0.3)
        est = SpeedEstimator(a, *observer_gains(a, 0.6, 0.8), d_max=0.35)
        est.update(0.4, 0.5, freeze=False)
        d0 = est.d_hat
        # Plant slower than predicted, command at its ceiling: d may not fall.
        assert not est.update(0.3, 0.5, freeze=False, wind=+1)
        assert est.d_hat == d0 and est.wound
        # Plant faster than predicted: falling demand is fine at the ceiling.
        assert est.update(0.6, 0.5, freeze=False, wind=+1)
        assert est.d_hat > d0


# --------------------------------------------------------------- bounds

def _witness_ok(c0, a_prev, lo, hi, ts=TS, t_first=TS, limits=LIMITS, trials=0):
    """Return a profile inside [lo, hi] that satisfies every rate row, or None."""
    c, a = c0, a_prev
    out = []
    for k in range(len(lo)):
        t_k = t_first if k == 0 else ts
        a_lo = max(limits['a_min'], a + limits['dAMin'] * t_k)
        a_hi = min(limits['a_max'], a + limits['dAMax'] * t_k)
        if c < lo[k]:
            a_des = (lo[k] - c) / ts
        elif c > hi[k]:
            a_des = (hi[k] - c) / ts
        else:
            a_des = 0.0
        a = min(max(a_des, a_lo), a_hi)
        c = c + ts * a
        if not (lo[k] - 1e-9 <= c <= hi[k] + 1e-9):
            return None
        out.append(c)
    return out


class TestTheCommandBox:

    def test_a_reachable_request_is_returned_unchanged(self):
        lo, hi = command_box(0.45, 0.0, [0.0] * 20, [0.5] * 20, TS, TS, LIMITS)
        assert np.all(lo == 0.0) and np.all(hi == 0.5)

    @pytest.mark.parametrize('seed', range(40))
    def test_the_widened_box_always_admits_a_feasible_command(self, seed):
        rng = np.random.default_rng(seed)
        c0 = float(rng.uniform(0.0, 0.8))
        a_prev = float(rng.uniform(-2.0, 3.0))
        hi = np.full(20, float(rng.uniform(0.0, 0.6)))
        lo = np.full(20, float(rng.uniform(0.0, 0.5)))
        lo[:3] = float(rng.uniform(0.0, 0.5))
        b_lo, b_hi = command_box(c0, a_prev, lo, hi, TS, TS, LIMITS)
        assert np.all(b_lo <= b_hi + 1e-12)
        assert _witness_ok(c0, a_prev, b_lo, b_hi) is not None
        # ...and the QP itself agrees: never an infeasibility certificate.
        sm = {'alpha': lag_alpha(TS, 0.3), 'delay': 0, 'd': 0.0, 'c_lo': b_lo,
              'c_hi': b_hi, 'v_hi': np.full(20, 0.5), 'slack_weight': 1e3}
        _u, info = solve_mpc_step(
            x0=[0.0, 0.0, 0.0, c0, c0], last_u=np.array([0.0, a_prev]),
            pref_nom=np.array([1.5, 0.0]), corridor=_corridor(), horizon=20, ts=TS,
            params={'L': L, 'lr': LR}, limits=LIMITS, weights=WEIGHTS, obstacles=[],
            dmin=0.32, vdes=0.5, control_period=TS, speed_model=sm)
        assert info['success'], info['status_message']

    def test_a_floor_above_the_cap_yields_to_the_cap(self):
        lo, hi = command_box(0.3, 0.0, [0.4] * 5, [0.3] * 5, TS, TS, LIMITS)
        assert np.all(lo <= 0.3 + 1e-12) and np.all(hi == 0.3)


# --------------------------------------------------------------- the channel

class TestTheChannel:

    def _run(self, ch, v, cmd, ticks, t0=0.0, **flags):
        for k in range(ticks):
            ch.note_published(cmd)
            ch.observe(t0 + k * TS, v, True, **flags)

    @pytest.mark.parametrize('flag, reason', [
        ({'hold': True}, 'hold'), ({'estop': True}, 'estop'),
        ({'mission_active': False}, 'mission_inactive'), ({'stopping': True}, 'stopping'),
        ({'reverse': True}, 'reverse')])
    def test_each_mission_stop_freezes_d(self, flag, reason):
        ch = _channel()
        self._run(ch, 0.4, 0.5, 5, **flag)
        assert ch.frozen and reason in ch.freeze_reasons and ch.d_hat == 0.0

    def test_stale_odometry_and_low_speed_freeze_d(self):
        ch = _channel()
        ch.note_published(0.5)
        ch.observe(0.0, 0.4, False)
        assert 'odom_stale' in ch.freeze_reasons
        self._run(ch, 0.05, 0.5, 2)
        assert 'low_speed' in ch.freeze_reasons

    def test_a_blocked_car_stalls_freezes_and_caps_the_command(self):
        ch = _channel(stall_time_s=0.8)
        self._run(ch, 0.0, 0.42, 7)
        assert not ch.stalled
        self._run(ch, 0.0, 0.42, 2, t0=0.7)
        assert ch.stalled and ch.stall_cap == pytest.approx(0.42)
        assert 'stall' in ch.freeze_reasons
        c0, sm = ch.plan(0.0, 0.0, LIMITS, vdes=0.5)
        assert np.all(sm['c_hi'] <= 0.42 + 1e-12)
        self._run(ch, 0.2, 0.42, 1, t0=1.0)
        assert not ch.stalled

    def test_another_lane_driving_is_an_override(self):
        ch = _channel()
        ch.note_applied(0.0, 0.0)
        ch.note_published(0.5)
        ch.observe(0.05, 0.3, True)
        assert ch.override and 'override' in ch.freeze_reasons
        assert ch.command_in_force == 0.0
        assert ch.rate_in_force() == 0.0

    def test_a_reset_forgets_d_but_keeps_the_command_in_force(self):
        ch = _channel()
        for k in range(40):
            ch.note_published(0.5)
            ch.observe(k * TS, 0.38, True)
        assert ch.d_hat < -0.05
        ch.reset('goal_change', 0.38)
        assert ch.d_hat == 0.0 and ch.last_reset == 'goal_change'
        assert ch.command_in_force == 0.5

    def test_the_start_up_bound_needs_rest_intent_and_a_driving_tick(self):
        ch = _channel(breakaway_mps=0.35, breakaway_speed_mps=0.08)
        c0, sm = ch.plan(0.0, 0.0, LIMITS, vdes=0.5)
        assert not ch.breakaway_active and c0 == 0.0       # no previous plan yet
        ch.command(0.2, {'v_cmd_pred': np.linspace(0.02, 0.6, 20)})
        c0, sm = ch.plan(0.0, 0.0, LIMITS, vdes=0.5)
        assert ch.breakaway_active and c0 == pytest.approx(0.35)
        assert np.all(sm['c_lo'][:3] >= 0.35 - 1e-12) and np.all(sm['c_lo'][3:] == 0.0)
        assert ch.command(0.0, {'v_cmd_pred': np.full(20, 0.5)}) >= 0.35
        c0, sm = ch.plan(0.2, 0.0, LIMITS, vdes=0.5)       # moving: released
        assert not ch.breakaway_active
        ch.command(0.0, {'v_cmd_pred': np.full(20, 0.5)})
        ch.plan(0.0, 0.0, LIMITS, vdes=0.5, may_start=False)
        assert not ch.breakaway_active

    def test_a_plan_that_wants_to_stay_put_is_not_kicked(self):
        ch = _channel()
        ch.command(0.0, {'v_cmd_pred': np.full(20, 0.02)})
        ch.plan(0.0, 0.0, LIMITS, vdes=0.5)
        assert not ch.breakaway_active

    def test_the_floor_is_a_bound_with_a_jump(self):
        ch = _channel()
        c0, sm = ch.plan(0.1, 0.0, LIMITS, vdes=0.4, floor=True)
        assert c0 == pytest.approx(0.4) and np.all(sm['c_lo'] >= 0.4 - 1e-12)

    def test_the_ceiling_is_max_forward_speed(self):
        ch = _channel()
        assert ch.ceiling == 0.5
        c0, sm = ch.plan(0.3, 0.0, LIMITS, vdes=0.5)
        assert np.all(sm['c_hi'] <= 0.5) and np.all(sm['v_hi'][ch.cap_first_stage:] == 0.5)
        assert np.all(np.isinf(sm['v_hi'][:ch.cap_first_stage]))

    def test_the_ramp_cap_is_model_consistent(self):
        ch = _channel()
        ch.est.d_hat = -0.1
        c0, sm = ch.plan(0.45, 0.0, LIMITS, vdes=0.4, ramp_box=0.4)
        assert sm['c_hi'][-1] == pytest.approx(0.5)      # 0.4 - (-0.1), at the ceiling
        ch.est.d_hat = 0.05
        c0, sm = ch.plan(0.45, 0.0, LIMITS, vdes=0.4, ramp_box=0.4)
        assert sm['c_hi'][-1] == pytest.approx(0.35)


# --------------------------------------------------------------- the QP

def _corridor(n=60, length=3.0):
    s = np.linspace(0.0, length, n)
    return {'xc': s, 'yc': np.zeros(n), 'tx': np.ones(n), 'ty': np.zeros(n),
            'nx': np.zeros(n), 'ny': np.ones(n), 'halfWidth': np.full(n, 0.64),
            'L': length, 'obstacles_world': [], 'car_radius': 0.2,
            'avoidance_margin': 0.12, 'Pend': np.array([length, 0.0]), 'psiRef': 0.0}


WEIGHTS = {'w_term': 9.0, 'w_v': 4.2857, 'w_psi': 4.5, 'w_psi_stage': 4.7736,
           'w_corr': 3.5714, 'w_line': 1.1662, 'w_u_a': 0.0, 'w_du_delta': 8.33,
           'w_du_a': 0.0, 'w_delta0': 0.0, 'w_obs': 8.0}


def _solve(x0, speed_model=None, solver='rti', stage_spec=None):
    return solve_mpc_step(
        x0=x0, last_u=np.zeros(2), pref_nom=np.array([1.5, 0.0]), corridor=_corridor(),
        horizon=20, ts=TS, params={'L': L, 'lr': LR}, limits=LIMITS, weights=WEIGHTS,
        obstacles=[], dmin=0.32, vdes=0.5, solver=solver, control_period=TS,
        speed_model=speed_model, stage_spec=stage_spec)


class TestTheQp:

    def _model(self, c_hi=0.5, d=0.0):
        return {'alpha': lag_alpha(TS, 0.3), 'delay': 0, 'd': d,
                'c_lo': np.zeros(20), 'c_hi': np.full(20, c_hi),
                'v_hi': np.full(20, 0.5), 'slack_weight': 1e3}

    def test_it_solves_and_reports_the_command_separately(self):
        u0, info = _solve([0.0, 0.0, 0.0, 0.3, 0.3], self._model())
        assert info['success']
        assert info['x_pred'].shape == (20, 4)
        assert len(info['v_cmd_pred']) == 20
        # Inside OSQP's tolerance (~1e-3 relative); the PUBLISHED command is
        # clipped to the requested box exactly by SpeedChannel.command.
        assert np.all(info['v_cmd_pred'] <= 0.5 + 5e-3)

    def test_a_negative_disturbance_raises_the_planned_command(self):
        _u, plain = _solve([0.0, 0.0, 0.0, 0.4, 0.4], self._model(c_hi=0.8))
        _u, lossy = _solve([0.0, 0.0, 0.0, 0.4, 0.4], self._model(c_hi=0.8, d=-0.1))
        assert lossy['v_cmd_pred'][-1] > plain['v_cmd_pred'][-1] + 0.05

    def test_the_wrong_state_size_and_the_unsupported_combinations_raise(self):
        with pytest.raises(ValueError):
            _solve([0.0, 0.0, 0.0, 0.3], self._model())
        with pytest.raises(ValueError):
            _solve([0.0, 0.0, 0.0, 0.3, 0.3], self._model(), solver='slsqp')
        with pytest.raises(ValueError):
            _solve([0.0, 0.0, 0.0, 0.3, 0.3], self._model(), stage_spec={'v_lo': []})


# --------------------------------------------------------------- params

class TestTheParameters:

    def test_every_key_is_in_stack_params_and_builds_a_valid_config(self):
        for key, _field, _kind in PARAM_KEYS:
            assert get_value(key) is not None, key
        cfg = config_from(get_value)
        cfg.validate()
        assert cfg.mode in (SPEED_MODE_OFFSET_FREE, SPEED_MODE_LEGACY)

    def test_the_shipped_mode_is_offset_free(self):
        assert get_value('speed_command_mode') == SPEED_MODE_OFFSET_FREE

    @pytest.mark.parametrize('bad', [
        {'mode': 'fast'}, {'tau_s': 0.0}, {'delay_ticks': 2}, {'moving_floor': 'some'},
        {'d_max_mps': -0.1}, {'breakaway_stages': 0}])
    def test_a_bad_configuration_is_refused(self, bad):
        with pytest.raises(ValueError):
            SpeedChannelConfig(**bad).validate()
