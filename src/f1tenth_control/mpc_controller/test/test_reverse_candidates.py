"""
reverse_mpc's direction candidates: schedules, feasibility, linearization, selection.

Pure Python, no rclpy node. Closed-loop scenarios live in
test_reverse_scenarios.py; the flag-off identity in
test_reverse_flag_off_identity.py.
"""

import math

from f1tenth_params.object_geometry import SOLVER_FRONT_POINT_OFFSETS_M
from mpc_controller import mpc_solver
from mpc_controller.mpc_solver import _d_front_at, solve_mpc_step
from mpc_controller.reverse_mpc import (
    build_candidates, CandidateResult, DirectionSelector, extend_corridor_backwards,
    floor_moving_speed_signed, FORWARD, nominal_controls, rear_probe_offsets, REVERSE,
    ReverseConfig, ReversePlanner, sign_tolerance, stop_witness)
from mpc_controller.vehicle_model import (
    f1tenth_state_fcn_dt_beta, f1tenth_state_fcn_dt_beta_jacobian)
import numpy as np
import pytest

import test_object_approach_closed_loop as rig

LIMITS = rig.LIMITS
N = rig.HORIZON
TS = rig.TS
TOL = sign_tolerance(LIMITS, TS)
REAR = rear_probe_offsets(rig.WHEELBASE, 0.405, -0.082, SOLVER_FRONT_POINT_OFFSETS_M)


def _cfg(**kw):
    base = {'allow_reverse': True, 'v_rev_max': 0.3, 'rear_offsets_m': REAR}
    base.update(kw)
    return ReverseConfig(**base)


def _straight_corridor(length=3.0, n=60, half_width=0.64):
    s = np.linspace(0.0, length, n)
    return {
        'xc': s, 'yc': np.zeros(n), 'xL': s, 'yL': np.full(n, half_width),
        'xR': s, 'yR': np.full(n, -half_width), 'tx': np.ones(n), 'ty': np.zeros(n),
        'nx': np.zeros(n), 'ny': np.ones(n), 'halfWidth': np.full(n, half_width),
        'psiRef': 0.0, 'psiRefTurn': None, 'obstacles_world': [], 'car_radius': 0.2,
        'avoidance_margin': 0.12,
    }


def _solve_kwargs(x0, last_u=(0.0, 0.0), pref=(2.0, 0.0), obstacles=()):
    corridor = _straight_corridor()
    corridor['obstacles_world'] = list(obstacles)
    return {
        'x0': np.asarray(x0, dtype=float), 'last_u': np.asarray(last_u, dtype=float),
        'pref_nom': np.asarray(pref, dtype=float), 'corridor': corridor, 'horizon': N,
        'ts': TS, 'params': rig.PARAMS, 'limits': LIMITS, 'weights': dict(rig.WEIGHTS),
        'obstacles': list(obstacles), 'dmin': rig.DMIN, 'vdes': 0.5, 'solver': 'rti',
        'boundaries': [], 'control_period': TS}


# ------------------------------------------------------------- the witness

class TestStopWitness:

    @pytest.mark.parametrize('v0,a0', [(0.5, 0.0), (0.4, 1.0), (0.5, -1.5), (-0.3, 0.0),
                                       (0.05, -2.0), (0.0, 0.0)])
    def test_respects_the_acceleration_box_and_the_rate_rows(self, v0, a0):
        v = stop_witness(v0, a0, N, TS, TS, LIMITS, 0.5 * TOL)
        a = np.diff(np.concatenate([[v0], v])) / TS
        assert np.all(a >= LIMITS['a_min'] - 1e-9) and np.all(a <= LIMITS['a_max'] + 1e-9)
        jerk = np.diff(np.concatenate([[a0], a])) / TS
        assert np.all(jerk >= LIMITS['dAMin'] - 1e-6)
        assert np.all(jerk <= LIMITS['dAMax'] + 1e-6)

    @pytest.mark.parametrize('v0', [0.5, 0.3, -0.3])
    def test_ends_stopped_within_the_sign_tolerance(self, v0):
        v = stop_witness(v0, 0.0, N, TS, TS, LIMITS, 0.5 * TOL)
        assert abs(v[-1]) <= TOL
        # Never overshoots past zero by more than the band.
        assert np.all(np.sign(v0) * v >= -0.5 * TOL - 1e-12)


# ------------------------------------------------------------- the schedules

class TestBuildCandidates:

    def test_moving_forward_the_reverse_rows_start_only_once_the_car_can_stop(self):
        specs, _ = build_candidates(0.4, 0.0, N, TS, TS, LIMITS, 0.5, _cfg())
        by = {s.name: s for s in specs}
        assert by['A'].constrained.all()
        k_b = int(np.argmax(by['B'].constrained))
        witness = stop_witness(0.4, 0.0, N, TS, TS, LIMITS, 0.5 * TOL)
        assert k_b > 0
        assert np.all(witness[k_b:] <= TOL)            # stopped from there on
        assert witness[k_b - 1] > TOL                  # and not one stage earlier
        assert np.all(by['B'].v_lo[:k_b] == LIMITS['vMin'])  # today's box before

    def test_reversing_the_forward_rows_wait_for_the_stop(self):
        specs, _ = build_candidates(-0.25, 0.0, N, TS, TS, LIMITS, 0.5, _cfg())
        by = {s.name: s for s in specs}
        assert by['B'].constrained.all()
        assert int(np.argmax(by['A'].constrained)) > 0

    def test_a_switch_time_at_the_horizon_end_is_the_all_reverse_candidate(self):
        # N * ts = 2.0 s at the shipped prediction step, so C2.0 IS B.
        specs, dropped = build_candidates(0.0, 0.0, N, TS, TS, LIMITS, 0.5, _cfg())
        assert [s.name for s in specs] == ['A', 'B', 'C1.0']
        assert ('C2.0', 'duplicate') in dropped

    def test_with_a_four_second_horizon_both_c_candidates_exist(self):
        specs, dropped = build_candidates(0.0, 0.0, N, 0.2, 0.1, LIMITS, 0.5, _cfg())
        assert [s.name for s in specs] == ['A', 'B', 'C1.0', 'C2.0']
        assert dropped == []

    def test_a_switch_before_the_car_can_stop_drops_the_candidate(self):
        specs, dropped = build_candidates(
            0.5, 0.0, N, TS, TS, LIMITS, 0.5, _cfg(switch_times_s=(0.3,)))
        assert 'C0.3' not in [s.name for s in specs]
        assert ('C0.3', 'no_reverse_stage') in dropped

    @pytest.mark.parametrize('v0,a0', [(0.5, 0.0), (-0.3, 0.0), (0.2, -1.0), (0.0, 0.0)])
    def test_the_witness_lies_inside_every_candidates_box(self, v0, a0):
        specs, _ = build_candidates(v0, a0, N, TS, TS, LIMITS, 0.5, _cfg())
        w = stop_witness(v0, a0, N, TS, TS, LIMITS, 0.5 * TOL)
        for s in specs:
            assert np.all(w >= s.v_lo - 1e-9) and np.all(w <= s.v_hi + 1e-9), s.name

    def test_speed_reference_is_symmetric_and_rear_probes_follow_reverse_stages(self):
        specs, _ = build_candidates(0.0, 0.0, N, TS, TS, LIMITS, 0.5, _cfg())
        by = {s.name: s for s in specs}
        assert np.all(by['A'].vdes == 0.5) and not by['A'].rear.any()
        assert np.all(by['B'].vdes == -0.5) and by['B'].rear.all()
        c = by['C1.0']
        assert np.all(c.vdes[c.signs < 0] == -0.5) and np.all(c.vdes[c.signs > 0] == 0.5)
        assert np.array_equal(c.rear, c.signs < 0)


# ------------------------------------------------------------- the QP side

class TestStageSpecInTheQp:

    @pytest.mark.parametrize('v0', [0.4, 0.0, -0.2])
    def test_every_candidate_solution_keeps_its_sign_schedule(self, v0):
        kw = _solve_kwargs((0.0, 0.0, 0.0, v0))
        specs, _ = build_candidates(v0, 0.0, N, TS, TS, LIMITS, 0.5, _cfg())
        for s in specs:
            z = nominal_controls(s, v0, np.zeros(N), TS, 0.18, 0.5)
            u0, info = solve_mpc_step(**dict(kw, warm_start_z=z,
                                             weights=dict(kw['weights'], w_rev=4.3),
                                             stage_spec=s.stage_spec(REAR)))
            assert info['success'], (s.name, info['status_message'])
            v = np.asarray(info['x_pred'])[:, 3]
            # 5 mm/s: OSQP's own feasibility tolerance at max_iter 200 (an
            # inaccurate solve misses a row by ~1e-3), far below any motion.
            assert np.all(v >= s.v_lo - 5e-3) and np.all(v <= s.v_hi + 5e-3), s.name

    def test_the_reverse_slack_is_the_reverse_speed(self):
        kw = _solve_kwargs((0.0, 0.0, 0.0, 0.0), pref=(-1.0, 0.0))
        specs, _ = build_candidates(0.0, 0.0, N, TS, TS, LIMITS, 0.5, _cfg())
        b = [s for s in specs if s.name == 'B'][0]
        z = nominal_controls(b, 0.0, np.zeros(N), TS, 0.18, 0.5)
        _u, info = solve_mpc_step(**dict(kw, warm_start_z=z,
                                         weights=dict(kw['weights'], w_rev=0.5),
                                         stage_spec=b.stage_spec(REAR)))
        v = np.asarray(info['x_pred'])[:, 3]
        assert v.min() < -0.05      # it does reverse toward the target behind
        np.testing.assert_allclose(info['rev_slack'], np.maximum(0.0, -v), atol=2e-3)

    def test_stage_spec_needs_the_rti_backend(self):
        kw = _solve_kwargs((0.0, 0.0, 0.0, 0.0))
        specs, _ = build_candidates(0.0, 0.0, N, TS, TS, LIMITS, 0.5, _cfg())
        with pytest.raises(ValueError):
            solve_mpc_step(**dict(kw, solver='slsqp', stage_spec=specs[0].stage_spec(REAR)))

    def test_without_stage_spec_the_obstacle_probes_are_the_front_ones(self):
        obstacles = [(-0.4, 0.0, 0.1)]       # behind the car
        d_front = _d_front_at(0.0, 0.0, 0.0, rig.WHEELBASE, obstacles)
        d_both = _d_front_at(0.0, 0.0, 0.0, rig.WHEELBASE, obstacles, REAR)
        assert d_front == pytest.approx(0.2525 + 0.4 - 0.1)
        assert d_both == pytest.approx(0.4 - 0.2295 - 0.1)

    def test_rear_offsets_mirror_the_front_pattern_about_the_bumpers(self):
        front = [rig.WHEELBASE / 2.0 + o - 0.405 for o in SOLVER_FRONT_POINT_OFFSETS_M]
        rear = [-0.082 - r for r in REAR]
        np.testing.assert_allclose(front, rear)
        np.testing.assert_allclose(REAR, (0.0705, -0.0795, -0.2295))


class TestNominalNeverMixesSigns:

    @pytest.mark.parametrize('v0', [0.45, 0.0, -0.28])
    def test_tracker_and_shifted_winner_nominals_stay_in_the_box(self, v0):
        specs, _ = build_candidates(v0, 0.0, N, TS, TS, LIMITS, 0.5, _cfg())
        # A deliberately wrong-signed warm start: full forward accel.
        wrong = np.full(N, 2.0)
        for s in specs:
            for z in (nominal_controls(s, v0, np.zeros(N), TS, 0.18, 0.5),
                      nominal_controls(s, v0, np.zeros(N), TS, 0.0, 0.5, accel_seq=wrong)):
                v = v0 + TS * np.cumsum(z[1::2])
                assert np.all(v >= s.v_lo - 1e-9) and np.all(v <= s.v_hi + 1e-9), s.name


# ------------------------------------------------------------- the model in reverse

class TestReverseKinematics:

    def test_steering_left_while_reversing_turns_the_heading_right(self):
        x = f1tenth_state_fcn_dt_beta([0.0, 0.0, 0.0, -0.4], [0.2, 0.0], 0.1, 0.305, 0.17)
        assert x[0] < 0.0          # moved backwards
        assert x[2] < 0.0          # yaw rate v/L * tan(delta) < 0
        x_fwd = f1tenth_state_fcn_dt_beta([0.0, 0.0, 0.0, 0.4], [0.2, 0.0], 0.1, 0.305, 0.17)
        assert x_fwd[2] == pytest.approx(-x[2])

    @pytest.mark.parametrize('v', [-0.4, -0.1])
    def test_the_jacobian_matches_finite_differences_in_reverse(self, v):
        xk = np.array([0.3, -0.2, 0.4, v])
        uk = np.array([-0.15, 0.3])
        A, B = f1tenth_state_fcn_dt_beta_jacobian(xk, uk, 0.1, 0.305, 0.17)
        eps = 1e-6
        for j in range(4):
            d = np.zeros(4)
            d[j] = eps
            col = (np.array(f1tenth_state_fcn_dt_beta(xk + d, uk, 0.1, 0.305, 0.17))
                   - np.array(f1tenth_state_fcn_dt_beta(xk - d, uk, 0.1, 0.305, 0.17))) / (2 * eps)
            np.testing.assert_allclose(A[:, j], col, atol=1e-6)
        for j in range(2):
            d = np.zeros(2)
            d[j] = eps
            col = (np.array(f1tenth_state_fcn_dt_beta(xk, uk + d, 0.1, 0.305, 0.17))
                   - np.array(f1tenth_state_fcn_dt_beta(xk, uk - d, 0.1, 0.305, 0.17))) / (2 * eps)
            np.testing.assert_allclose(B[:, j], col, atol=1e-6)


# ------------------------------------------------------------- the selector

def _res(name, direction, score):
    from mpc_controller.reverse_mpc import CandidateSpec
    spec = CandidateSpec(name=name, direction=direction, final_direction=direction,
                         signs=np.full(N, direction), constrained=np.ones(N, bool),
                         v_lo=np.zeros(N), v_hi=np.zeros(N), vdes=np.zeros(N),
                         rear=np.zeros(N, bool))
    return CandidateResult(spec=spec, ok=True, score=score)


class TestDirectionSelector:

    def test_a_gain_below_the_absolute_threshold_keeps_the_direction(self):
        sel = DirectionSelector(_cfg(switch_threshold=1.0, switch_threshold_rel=0.0))
        win, why = sel.select([_res('A', FORWARD, 10.0), _res('B', REVERSE, 9.2)], 0.4, 5.0)
        assert win.spec.name == 'A' and why == 'threshold_not_met'

    def test_a_gain_below_the_relative_threshold_keeps_the_direction(self):
        sel = DirectionSelector(_cfg(switch_threshold=1.0, switch_threshold_rel=0.1))
        win, why = sel.select([_res('A', FORWARD, 100.0), _res('B', REVERSE, 92.0)], 0.4, 5.0)
        assert win.spec.name == 'A' and why == 'threshold_not_met'

    def test_both_thresholds_met_switches_and_commits_while_moving(self):
        sel = DirectionSelector(_cfg(switch_threshold=1.0, switch_threshold_rel=0.1))
        win, why = sel.select([_res('A', FORWARD, 20.0), _res('B', REVERSE, 10.0)], 0.4, 5.0)
        assert win.spec.name == 'B' and why == 'switched'
        assert sel.committed and sel.direction == REVERSE

    def test_committed_holds_until_stopped_unless_the_break_margin_is_beaten(self):
        sel = DirectionSelector(_cfg(switch_threshold=1.0, switch_threshold_rel=0.0,
                                     commit_break_margin=3.0, min_dwell_s=0.0))
        sel.select([_res('A', FORWARD, 20.0), _res('B', REVERSE, 10.0)], 0.4, 5.0)
        win, why = sel.select([_res('A', FORWARD, 10.0), _res('B', REVERSE, 12.0)], 0.3, 5.1)
        assert win.spec.name == 'B' and why == 'committed'
        win, why = sel.select([_res('A', FORWARD, 8.0), _res('B', REVERSE, 12.0)], 0.2, 5.2)
        assert win.spec.name == 'A' and why == 'commit_broken'

    def test_the_commitment_ends_once_the_car_is_stopped(self):
        sel = DirectionSelector(_cfg(switch_threshold=1.0, switch_threshold_rel=0.0,
                                     min_dwell_s=0.0))
        sel.select([_res('A', FORWARD, 20.0), _res('B', REVERSE, 10.0)], 0.4, 5.0)
        assert sel.committed
        sel.select([_res('A', FORWARD, 11.5), _res('B', REVERSE, 12.0)], 0.01, 5.5)
        assert not sel.committed

    def test_dwell_blocks_a_second_change(self):
        sel = DirectionSelector(_cfg(switch_threshold=1.0, switch_threshold_rel=0.0,
                                     min_dwell_s=1.0))
        sel.select([_res('A', FORWARD, 20.0), _res('B', REVERSE, 10.0)], 0.0, 5.0)
        win, why = sel.select([_res('A', FORWARD, 1.0), _res('B', REVERSE, 10.0)], 0.0, 5.5)
        assert win.spec.name == 'B' and why == 'dwell'
        win, why = sel.select([_res('A', FORWARD, 1.0), _res('B', REVERSE, 10.0)], 0.0, 6.1)
        assert win.spec.name == 'A' and why == 'switched'

    def test_an_infeasible_current_direction_switches_at_once(self):
        sel = DirectionSelector(_cfg())
        bad = _res('A', FORWARD, math.inf)
        bad.ok = False
        win, why = sel.select([bad, _res('B', REVERSE, 50.0)], 0.0, 0.0)
        assert win.spec.name == 'B' and why == 'current_direction_infeasible'


# ------------------------------------------------------------- floor and corridor

class TestOperatingFloorBothWays:

    def test_forward_is_todays_floor(self):
        assert floor_moving_speed_signed(0.2, 0.4, FORWARD, 0.3, 0.05) == 0.4
        assert floor_moving_speed_signed(0.5, 0.4, FORWARD, 0.3, 0.05) == 0.5

    def test_reverse_is_the_mirror(self):
        assert floor_moving_speed_signed(-0.1, 0.4, REVERSE, -0.2, 0.05) == -0.4
        assert floor_moving_speed_signed(-0.5, 0.4, REVERSE, -0.2, 0.05) == -0.5

    def test_no_floor_while_braking_for_a_direction_change(self):
        # Still rolling forward with reverse selected: the command passes through.
        assert floor_moving_speed_signed(0.25, 0.4, REVERSE, 0.3, 0.05) == 0.25
        assert floor_moving_speed_signed(-0.2, 0.4, FORWARD, -0.3, 0.05) == -0.2


class TestCorridorBackExtension:

    def test_extension_is_straight_behind_with_the_same_heading_and_width(self):
        c = _straight_corridor()
        e = extend_corridor_backwards(c, 1.0)
        n_back = len(e['xc']) - len(c['xc'])
        assert e['backLength'] >= 1.0 and n_back > 0
        assert np.all(np.diff(e['xc']) > 0)                 # ordered, behind first
        assert e['xc'][0] == pytest.approx(-e['backLength'])
        assert np.all(e['tx'][:n_back] == 1.0) and np.all(e['ty'][:n_back] == 0.0)
        assert np.all(e['halfWidth'][:n_back] == c['halfWidth'][0])
        np.testing.assert_allclose(e['yL'][:n_back], c['halfWidth'][0])

    def test_zero_length_returns_the_same_dict(self):
        c = _straight_corridor()
        assert extend_corridor_backwards(c, 0.0) is c


# ------------------------------------------------------------- the planner

class TestPlanner:

    def test_flag_off_is_solve_mpc_step(self):
        kw = _solve_kwargs((0.0, 0.0, 0.0, 0.4))
        u_a, i_a = ReversePlanner(ReverseConfig(allow_reverse=False)).step(now_sec=1.0, **kw)
        u_b, i_b = solve_mpc_step(**kw)
        assert np.array_equal(u_a, u_b) and np.array_equal(i_a['zopt'], i_b['zopt'])
        assert 'reverse' not in i_a

    def test_free_road_ahead_stays_forward_and_reports_every_candidate(self):
        p = ReversePlanner(_cfg())
        kw = _solve_kwargs((0.0, 0.0, 0.0, 0.4))
        for i in range(3):
            u0, info = p.step(now_sec=0.1 * i, **kw)
        dbg = info['reverse']
        assert dbg['winner'] == 'A' and dbg['direction'] == 'forward'
        assert [c['name'] for c in dbg['candidates']] == ['A', 'B', 'C1.0']
        assert all(c['ok'] for c in dbg['candidates'])
        assert min(c['score'] for c in dbg['candidates']) == pytest.approx(
            [c['score'] for c in dbg['candidates'] if c['name'] == 'A'][0])

    def test_all_candidates_failing_falls_back_to_todays_solve(self, monkeypatch):
        p = ReversePlanner(_cfg())
        kw = _solve_kwargs((0.0, 0.0, 0.0, 0.4))
        real = mpc_solver.solve_mpc_step

        def failing(**k):
            if k.get('stage_spec') is not None:
                raise RuntimeError('boom')
            return real(**k)
        monkeypatch.setattr('mpc_controller.reverse_mpc.solve_mpc_step', failing)
        u0, info = p.step(now_sec=0.0, **kw)
        assert info['reverse']['reason'] == 'fallback_all_failed'
        assert np.array_equal(u0, real(**kw)[0])
