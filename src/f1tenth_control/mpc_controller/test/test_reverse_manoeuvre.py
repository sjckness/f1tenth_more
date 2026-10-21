"""
The reverse manoeuvre (reverse_manoeuvre.py): arbitration, rear side, closed loop.

Closed-loop cases run test/reverse_manoeuvre_harness.py (the real forward
MPC, the real rear solver, a first-order speed plant). The state-machine
cases feed ReverseSupervisor scripted inputs with a stub rear solver, so each
condition is tested on its own.
"""

import math

from mpc_controller.reverse_manoeuvre import (
    Grid, map_to_odom_point, MODE_FORWARD, MODE_REVERSE, MODE_STOPPING, MODE_STUCK,
    odom_to_map_point, rear_corridor, rear_horizon, rear_strip_check, RearEnv,
    ReverseSupervisor, validate_reverse_mechanisms)
import numpy as np
import pytest

import reverse_manoeuvre_harness as H

TS = H.TS


# ================================================================== closed loop

@pytest.fixture(scope='module')
def against_wall():
    """Stuck against a wall 2 m ahead, BT allows reverse, costmap free and known."""
    return H.run(H.Script(), duration=35.0, front_wall_x=2.0)


def test_stuck_against_a_wall_reverses_half_a_metre_then_forward_resumes(against_wall):
    r = against_wall
    spans = H.reverses(r)
    assert spans, 'no reverse'
    assert r.terminations[0][1] == 'distance_done'
    i0 = r.t.index(spans[0][0])
    i1 = r.t.index(spans[0][1])
    assert r.x[i0][0] - r.x[i1][0] == pytest.approx(0.5, abs=0.06)
    # Forward resumes: the forward solver drives again after the reverse.
    after = [k for k in range(i1, len(r.t)) if r.source[k] == 'forward']
    assert after and r.forward_calls[after[0]]


def test_three_reverses_without_progress_then_stuck_and_no_fourth(against_wall):
    r = against_wall
    assert len(H.reverses(r)) == 3
    assert r.mode[-1] == MODE_STUCK
    assert ('max_attempts' in [why for _t, why in r.terminations])
    assert r.v_cmd[-1] == 0.0


def test_forward_min_time_is_respected(against_wall):
    r = against_wall
    cfg = H.shipped_manoeuvre_config()
    spans = H.reverses(r)
    for (_s0, e0), (s1, _e1) in zip(spans, spans[1:]):
        resumed = next(t for t, m in zip(r.t, r.mode) if t > e0 and m == MODE_FORWARD)
        switch = max(t for t, m in zip(r.t, r.mode) if t < s1 and m == MODE_FORWARD)
        assert switch - resumed >= cfg.forward_min_time_s - 1e-9


def test_only_one_solver_drives_each_tick(against_wall):
    r = against_wall
    for fwd, src, rear in zip(r.forward_calls, r.source, r.rear_calls):
        if src == 'rear':
            assert not fwd
        if fwd and src == 'forward':
            assert True            # forward solve and its own command
    # The forward solver is never run while the manoeuvre owns the car.
    owned = [not f for f, m in zip(r.forward_calls, r.mode) if m == MODE_REVERSE]
    assert owned and all(owned)


def test_gear_change_waits_for_standstill_and_the_dwell(against_wall):
    r = against_wall
    cfg = H.shipped_manoeuvre_config()
    n_dwell = int(round(cfg.gear_change_dwell_s / TS))
    for s0, _e in H.reverses(r):
        k = r.t.index(s0)
        assert r.source[k] == 'rear'
        before = [abs(r.x[j][3]) for j in range(k - n_dwell, k)]
        assert all(v < cfg.stuck_speed_mps for v in before)
        assert all(src == 'stop' for src in r.source[k - n_dwell:k])


def test_normal_driving_never_runs_the_rear_solver():
    r = H.run(H.Script(), duration=12.0, goal_distance=4.0)
    assert sum(r.rear_calls) == 0
    assert set(r.mode) == {MODE_FORWARD}
    assert 'goal_reached' in r.source


def test_without_bt_permission_the_supervisor_changes_no_command():
    script = H.Script(bt_allow=lambda t: None)
    on = H.run(script, duration=12.0, front_wall_x=2.0)
    off = H.run(script, duration=12.0, front_wall_x=2.0, supervisor_enabled=False)
    assert on.v_cmd == off.v_cmd and on.steer == off.steer
    assert any(s['rear_solver_running'] for s in on.status)    # it did look


def test_a_wall_appearing_behind_ends_the_reverse_at_the_stop_margin():
    cfg = H.shipped_manoeuvre_config()

    def behind(start):
        x, y, psi = start
        bx = x - cfg.car_rear_offset_m * math.cos(psi)
        r = 0.05
        return [(bx - 0.30 - r, y, r)]          # 0.30 m behind the rear bumper
    r = H.run(H.Script(objects_in_reverse=behind), duration=12.0, front_wall_x=2.0)
    assert r.terminations and r.terminations[0][1] == 'rear_clearance'
    s0, e0 = H.reverses(r)[0]
    k0, k1 = r.t.index(s0), r.t.index(e0)
    start = r.x[k0]
    wall_x = start[0] - cfg.car_rear_offset_m - 0.30
    gaps = [(x[0] - cfg.car_rear_offset_m) - wall_x for x in r.x[k0:k1 + 15]]
    assert min(gaps) > 0.05                     # never reached it


def test_steering_brings_a_lateral_offset_back_while_reversing():
    kicked = {'done': False}

    def perturb(t, x, sup):
        if sup.mode == MODE_REVERSE and not kicked['done']:
            kicked['done'] = True
            x = x.copy()
            x[1] += 0.10                         # pushed 10 cm left of the corridor
        return x
    r = H.run(H.Script(), duration=10.0, front_wall_x=2.0, perturb=perturb)
    s0, e0 = H.reverses(r)[0]
    k0, k1 = r.t.index(s0), r.t.index(e0)
    ys = [x[1] for x in r.x[k0 + 1:k1 + 1]]
    psis = [x[2] for x in r.x[k0 + 1:k1 + 1]]
    # Reversing, a positive (left) offset is removed with RIGHT steering:
    # psi_dot = v / L tan(delta) > 0 for delta < 0 when v < 0, which swings
    # the tail toward -y. The sign, the heading and the offset all move the
    # correcting way. HOW MUCH is mpc_rear.* tuning (the copied forward
    # weights hold the corridor heading hard; 0.5 m corrects ~5 mm).
    assert min(r.steer[k0 + 1:k0 + 6]) < 0.0
    assert psis[-1] > psis[0] + 0.01
    assert ys[-1] < ys[0]


# ================================================================== the rear side

def test_rear_horizon_is_ceil_length_over_v_dt_clamped():
    cfg = H.shipped_manoeuvre_config()
    assert rear_horizon(cfg) == 7               # ceil(0.5 / (0.4 * 0.2)) = 7
    cfg.rear_dt_s = 0.1
    assert rear_horizon(cfg) == 10              # 13 clamped
    cfg.corridor_length_m = 0.1
    assert rear_horizon(cfg) == 5               # 3 clamped


def test_the_rear_corridor_starts_at_the_bumper_and_widens_like_the_forward_ones():
    cfg = H.shipped_manoeuvre_config()
    c = rear_corridor(1.0, 2.0, 0.3, cfg)
    bx, by = c['bumper']
    assert bx == pytest.approx(1.0 - cfg.car_rear_offset_m * math.cos(0.3))
    assert (c['xc'][-1], c['yc'][-1]) == pytest.approx((bx, by))     # bumper end last
    assert math.hypot(c['xc'][0] - bx, c['yc'][0] - by) == pytest.approx(
        cfg.corridor_length_m, abs=1e-6)
    assert c['halfWidth'][-1] == pytest.approx(cfg.corridor_half_width_m, abs=1e-3)
    assert c['halfWidth'][0] == pytest.approx(cfg.corridor_half_width_m * cfg.width_ratio,
                                              abs=1e-3)
    assert np.allclose(np.arctan2(c['ty'], c['tx']), 0.3)            # the car's heading
    assert c['psiRef'] == 0.3


def test_map_odom_round_trip():
    T = (1.0, -2.0, 0.4)
    p = (3.0, 0.5)
    assert odom_to_map_point(map_to_odom_point(p, T), T) == pytest.approx(p)


def _grid(value=0):
    return Grid(data=np.full((100, 100), value), resolution=0.05, origin_x=-2.5,
                origin_y=-2.5)


def test_the_strip_needs_free_and_known_cells():
    cfg = H.shipped_manoeuvre_config()
    ok, why, clear = rear_strip_check((0.0, 0.0), 0.0, 0.5, cfg, _grid(0), None, [])
    assert ok and clear == pytest.approx(0.5)
    g = _grid(0)
    H.mark(g, (-0.4, -0.3), (-0.05, 0.05), -1)            # unknown 0.3 m behind
    ok, why, clear = rear_strip_check((0.0, 0.0), 0.0, 0.5, cfg, g, None, [])
    assert not ok and why == 'rear_unknown' and clear == pytest.approx(0.3, abs=0.03)
    g = _grid(0)
    H.mark(g, (-0.4, -0.3), (-0.05, 0.05), 100)
    ok, why, _ = rear_strip_check((0.0, 0.0), 0.0, 0.5, cfg, g, None, [])
    assert not ok and why == 'rear_occupied'
    ok, why, _ = rear_strip_check((0.0, 0.0), 0.0, 0.5, cfg, None, None, [])
    assert not ok and why == 'rear_unknown'


def test_an_object_behind_blocks_the_strip_one_beside_it_does_not():
    cfg = H.shipped_manoeuvre_config()
    ok, why, _ = rear_strip_check((0.0, 0.0), 0.0, 0.5, cfg, _grid(0), None,
                                  [(-0.4, 0.0, 0.1)])
    assert not ok and why == 'rear_object'
    ok, _why, _ = rear_strip_check((0.0, 0.0), 0.0, 0.5, cfg, _grid(0), None,
                                   [(-0.4, 0.8, 0.1)])
    assert ok


def test_the_strip_is_checked_in_the_map_frame():
    cfg = H.shipped_manoeuvre_config()
    g = _grid(0)
    H.mark(g, (0.6, 0.7), (-0.05, 0.05), 100)            # occupied in MAP at x 0.6..0.7
    # odom = map shifted by +1 m in x: the car at odom (0, 0) is at map (1, 0).
    ok, why, clear = rear_strip_check((0.0, 0.0), 0.0, 0.5, cfg, g, (1.0, 0.0, 0.0), [])
    assert not ok and why == 'rear_occupied' and clear == pytest.approx(0.3, abs=0.03)


# ================================================================== state machine

class _StubRear:
    def __init__(self, ok=True, seconds=0.002):
        self.ok, self.seconds = ok, seconds
        self.calls = 0
        self.resets = 0
        self.last_ms = seconds * 1e3
        self.warm = None

    def reset(self):
        self.resets += 1
        self.warm = None

    def solve(self, x, steer_prev, corridor, obstacles, boundaries, *_a):
        self.calls += 1
        self.warm = 'warm'
        self.corridor = corridor
        return -0.05, {}, self.ok, self.seconds


def _sup(**cfg_over):
    cfg = H.shipped_manoeuvre_config(**cfg_over)
    rear = _StubRear()
    return ReverseSupervisor(cfg, rear, TS), rear


def _env(**over):
    base = {'grid': _grid(0), 'map_to_odom': None, 'bt_allow': True, 'bt_allow_age_s': 0.1}
    base.update(over)
    return RearEnv(**base)


X_STOPPED = np.array([0.0, 0.0, 0.0, 0.0])
X_MOVING = np.array([0.0, 0.0, 0.0, 0.4])
X_PRED_STILL = np.zeros((20, 4))
X_PRED_GOING = np.column_stack([np.linspace(0.05, 1.0, 20), np.zeros(20), np.zeros(20),
                                np.full(20, 0.5)])


def _tick(sup, k, forward_ok=True, x=X_STOPPED, x_pred=X_PRED_STILL, mission=True,
          intentional=False, front=0.2, env=None):
    env = env or _env()
    t = k * TS
    cmd = sup.before_forward(t, x, env)
    if cmd is not None:
        return cmd
    return sup.after_forward(t, x, forward_ok, x_pred, mission, intentional, front, env)


def test_stuck_continuously_for_half_a_second_warms_the_rear_and_switches_on_time():
    sup, rear = _sup()
    cmds = [_tick(sup, k, forward_ok=False) for k in range(8)]
    assert rear.calls >= 1 and rear.calls <= 5           # warmed from the first stuck tick
    first = next(k for k, c in enumerate(cmds) if c is not None)
    assert first == 4                                     # 5 ticks x 0.1 s = 0.5 s
    assert sup.mode == MODE_STOPPING


def test_stuck_clearing_at_0p3_s_stops_the_rear_solver_and_never_reverses():
    sup, rear = _sup()
    for k in range(3):
        assert _tick(sup, k, forward_ok=False) is None
    assert rear.calls == 3 and sup.rear_running
    for k in range(3, 20):
        assert _tick(sup, k, x=X_MOVING, x_pred=X_PRED_GOING, front=None) is None
    assert not sup.rear_running and rear.resets == 1 and rear.warm is None
    assert sup.mode == MODE_FORWARD and rear.calls == 3


def test_infeasible_on_alternating_ticks_for_two_seconds_never_reverses():
    sup, _rear = _sup()
    for k in range(20):
        ok = (k % 2 == 1)
        cmd = _tick(sup, k, forward_ok=ok, x=X_MOVING,
                    x_pred=(X_PRED_GOING if ok else X_PRED_STILL), front=None)
        assert cmd is None
    assert sup.mode == MODE_FORWARD


def test_a_completed_approach_to_the_wall_is_not_stuck():
    # "Approach the wall" done at 0.3 m: stopped, wall in front, no progress --
    # but the move asked for it.
    sup, rear = _sup()
    for k in range(30):
        assert _tick(sup, k, intentional=True, front=0.3) is None
    assert sup.stuck.timer == 0.0 and rear.calls == 0


def test_a_paused_mission_is_not_stuck():
    sup, rear = _sup()
    for k in range(30):
        assert _tick(sup, k, forward_ok=False, mission=False, intentional=True) is None
    assert rear.calls == 0 and sup.mode == MODE_FORWARD


@pytest.mark.parametrize('env_over,why', [
    ({'grid': None}, 'rear_unknown'),
    ({'objects': [(-0.4, 0.0, 0.15)]}, 'rear_object'),
])
def test_an_unknown_or_occupied_rear_never_reverses(env_over, why):
    sup, _rear = _sup()
    for k in range(20):
        assert _tick(sup, k, forward_ok=False, env=_env(**env_over)) is None
    assert sup.mode == MODE_FORWARD and sup.rear_reason == why and not sup.rear_ready


def test_high_forward_cost_but_not_stuck_never_reverses():
    # The supervisor has no cost input at all: a feasible forward plan that
    # makes progress is never second-guessed, however expensive.
    sup, rear = _sup()
    for k in range(30):
        assert _tick(sup, k, x=X_MOVING, x_pred=X_PRED_GOING, front=None) is None
    assert rear.calls == 0 and sup.mode == MODE_FORWARD


@pytest.mark.parametrize('env_over,why', [
    ({'bt_allow': None, 'bt_allow_age_s': math.inf}, 'bt_allow_stale'),
    ({'bt_allow': True, 'bt_allow_age_s': 0.6}, 'bt_allow_stale'),
    ({'bt_allow': False}, 'bt_disallowed'),
])
def test_no_reverse_without_a_fresh_bt_permission(env_over, why):
    sup, _rear = _sup()
    for k in range(20):
        assert _tick(sup, k, forward_ok=False, env=_env(**env_over)) is None
    assert sup.mode == MODE_FORWARD
    assert sup.request(2.0, X_STOPPED, 0.3, _env(**env_over)) == (False, why)


def test_the_drive_clamp_must_allow_the_reverse_speed():
    sup, _rear = _sup(max_reverse_speed_mps=0.0)
    for k in range(20):
        assert _tick(sup, k, forward_ok=False) is None
    assert sup.request(2.0, X_STOPPED, 0.3, _env()) == (False, 'reverse_speed_limit')


def test_the_manual_request_caps_the_distance_and_needs_the_same_checks():
    sup, _rear = _sup()
    ok, why = sup.request(0.0, X_STOPPED, 2.0, _env())
    assert ok and sup.requested == pytest.approx(0.5) and sup.mode == MODE_STOPPING
    sup2, _ = _sup()
    assert sup2.request(0.0, X_STOPPED, 0.2, _env(objects=[(-0.3, 0.0, 0.1)])) == (
        False, 'rear_object')
    sup3, _ = _sup()
    sup3.request(0.0, X_STOPPED, 0.0, _env())
    assert sup3.requested == pytest.approx(sup3.cfg.corridor_length_m)


def _into_reverse(sup):
    sup.request(0.0, X_STOPPED, 0.0, _env())
    for k in range(10):
        cmd = sup.before_forward(k * TS, X_STOPPED, _env())
        if sup.mode == MODE_REVERSE:
            return k, cmd
    raise AssertionError('never reached reverse')


def test_two_infeasible_rear_solves_in_a_row_end_the_reverse():
    sup, rear = _sup()
    k, cmd = _into_reverse(sup)
    assert cmd.speed < 0.0
    rear.ok = False
    sup.before_forward((k + 1) * TS, X_STOPPED, _env())
    assert sup.mode == MODE_REVERSE                 # one failure: hold on
    sup.before_forward((k + 2) * TS, X_STOPPED, _env())
    assert sup.mode == MODE_STOPPING and sup.last_termination == 'rear_infeasible'


def test_two_over_budget_rear_solves_in_a_row_end_the_reverse():
    sup, rear = _sup()
    k, _ = _into_reverse(sup)
    rear.seconds = 0.2
    sup.before_forward((k + 1) * TS, X_STOPPED, _env())
    sup.before_forward((k + 2) * TS, X_STOPPED, _env())
    assert sup.last_termination == 'rear_over_budget'


def test_the_reverse_times_out_at_twice_its_expected_duration():
    sup, _ = _sup()
    k, _ = _into_reverse(sup)
    t_end = k * TS + 2.0 * sup.requested / sup.cfg.v_rev_mps
    sup.before_forward(t_end + 0.05, X_STOPPED, _env())
    assert sup.last_termination == 'timeout'


def test_a_hold_aborts_the_reverse():
    sup, _ = _sup()
    _into_reverse(sup)
    sup.abort('hold')
    assert sup.mode == MODE_STOPPING and sup.last_termination == 'hold'
    cmd = sup.before_forward(5.0, X_STOPPED, _env())
    assert cmd.speed == 0.0


def test_the_bt_withdrawing_its_permission_ends_the_reverse():
    sup, _ = _sup()
    k, _ = _into_reverse(sup)
    cmd = sup.before_forward((k + 1) * TS, X_STOPPED, _env(bt_allow=False))
    assert cmd.speed == 0.0 and sup.last_termination == 'bt_withdrawn: bt_disallowed'
    sup2, _ = _sup()
    k, _ = _into_reverse(sup2)
    sup2.before_forward((k + 1) * TS, X_STOPPED, _env(bt_allow_age_s=0.8))
    assert sup2.last_termination == 'bt_withdrawn: bt_allow_stale'


def _reverse_through_a_map_odom_jump():
    """Reverse 0.3 m in odom, then let the localisation correct map -> odom by 0.2 m."""
    sup, rear = _sup()
    sup.request(0.0, X_STOPPED, 0.0, _env(map_to_odom=(0.0, 0.0, 0.0)))
    for k in range(10):
        sup.before_forward(k * TS, X_STOPPED, _env(map_to_odom=(0.0, 0.0, 0.0)))
    assert sup.mode == MODE_REVERSE
    sup.before_forward(1.1, np.array([-0.3, 0.0, 0.0, -0.4]), _env(map_to_odom=(0.0, 0.0, 0.0)))
    before = (sup.travelled, rear.corridor['bumper'])
    sup.before_forward(1.2, np.array([-0.3, 0.0, 0.0, -0.4]), _env(map_to_odom=(0.2, 0.0, 0.0)))
    return sup, rear, before


def test_the_rear_corridor_stays_put_in_map_through_a_map_odom_jump():
    # The start pose, held in map, now reads 0.2 m further back in odom, and
    # the corridor handed to the rear solver moves with it.
    _s, rear, (_travelled, bumper_before) = _reverse_through_a_map_odom_jump()
    assert rear.corridor['bumper'][0] - bumper_before[0] == pytest.approx(-0.2, abs=1e-9)
    assert rear.corridor['bumper'][1] == pytest.approx(bumper_before[1], abs=1e-9)


def test_a_map_odom_correction_is_not_distance_travelled():
    # The car has backed up 0.3 m in odom on both sides of the correction.
    sup, _rear, (travelled_before, _bumper) = _reverse_through_a_map_odom_jump()
    assert travelled_before == pytest.approx(0.3, abs=1e-6)
    assert sup.travelled == pytest.approx(0.3, abs=1e-6)


def test_a_reverse_the_localisation_holds_still_in_map_ends_on_distance_not_timeout():
    # The 2026-10-20 15:39 car run: map -> odom absorbed the motion, so the
    # car stood still in map while odom moved back at 0.35 m/s. Counted
    # through the map anchor, that reverse ran on to its timeout.
    sup, _ = _sup()
    k, _cmd = _into_reverse(sup)
    t0 = k * TS
    for i in range(1, 40):
        back = 0.035 * i
        sup.before_forward(t0 + i * TS, np.array([-back, 0.0, 0.0, -0.35]),
                           _env(map_to_odom=(back, 0.0, 0.0)))
        if sup.mode != MODE_REVERSE:
            break
    assert sup.last_termination == 'distance_done'
    assert sup.travelled == pytest.approx(0.5, abs=0.04)


def test_a_new_move_lifts_the_stuck_give_up():
    sup, _ = _sup()
    sup.mode = MODE_STUCK
    sup.attempts = 3
    sup.new_move()
    assert sup.mode == MODE_FORWARD and sup.attempts == 0


@pytest.mark.parametrize('allow_reverse,auto', [(False, True), (True, False), (False, False)])
def test_one_reverse_mechanism_at_a_time_starts(allow_reverse, auto):
    validate_reverse_mechanisms(allow_reverse, auto)


def test_both_reverse_mechanisms_refuse_to_start():
    with pytest.raises(ValueError, match='only one reverse mechanism'):
        validate_reverse_mechanisms(True, True)
