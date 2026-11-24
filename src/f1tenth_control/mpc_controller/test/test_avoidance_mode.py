"""
mpc_avoidance_mode (2026-11-24): 'centerline' (today) or 'legacy' (the car up to 2026-11-17).

WHY. Field test 2026-11-23 (turn_left_then_person_then_straight_2m, image 1992797): 0 of 4
straight legs towards a person completed. In the logged run the car stopped facing the person
1 m ahead, backed off with the reverse manoeuvre and swerved 58 deg away. 'legacy' brings back
the avoidance the car ran before branch mpc-centerline-target (7c217d95, 16a69a58, 943f98c6) and
the mpc-dodge car deploy (c19f400e), so a field test can switch back with a profile.

What 'legacy' is, and what this file pins:
  - the lookahead target is shifted off an obstacle (_legacy_deflected_target, the body of
    compute_local_target before 7c217d95), with its 5-tick coast and its smoother;
  - w_obs is mpc_legacy_w_obs, not mpc_w_obs;
  - no side rule, and every mpc-dodge term off whatever stack_params ships for it;
  - compute_local_target itself is untouched: 'centerline' stays exactly today's.

Closed loop (the rig of test_centerline_target_closed_loop.py): a person 1 m ahead of the
bumper, from a standstill, on a mission 'drive: straight' move. Measured 2026-11-24 on the
nominal rig (radius 0.75 = person 0.25 + class margin 0.5): centerline passes in 7.6 s keeping
the margin, legacy in 5.6 s eating 0.13 m into the class margin (0.37 m clear of the body).
The rig does not reproduce the field stall (single static obstacle, perfect model); both pass.

Run standalone: python3 -m pytest test/test_avoidance_mode.py -v
"""

import math

from f1tenth_params.param_defaults import get_value
from mpc_controller.drive_limits import clamp_drive_speed
from mpc_controller.MPC_corr import MPCController
from mpc_controller.mpc_solver import shift_warm_start, solve_mpc_step
from mpc_controller.vehicle_model import f1tenth_state_fcn_dt_beta
import numpy as np
import pytest
import test_centerline_target_closed_loop as cl
import test_object_approach_closed_loop as rig


class _Logger:
    def __init__(self):
        self.lines = []

    def info(self, msg, **kwargs):
        self.lines.append(msg)

    warn = info
    error = info


class _Param:
    def __init__(self, value):
        self.value = value


class _Node:
    """Just what _init_avoidance_mode / _init_dodge_params / the legacy target read."""

    _reset_legacy_target = MPCController._reset_legacy_target

    def __init__(self, **overrides):
        self.overrides = overrides
        self.logger = _Logger()
        self.car_radius = 0.20
        self.avoidance_margin = 0.12

    def declare_parameter(self, name, default):
        return _Param(self.overrides.get(name, default))

    def get_logger(self):
        return self.logger


def _node(mode, **overrides):
    node = _Node(mpc_avoidance_mode=mode, **overrides)
    MPCController._init_avoidance_mode(node)
    return node


def _straight_corridor(length=3.0, n=120):
    s = np.linspace(0.0, length, n)
    return {'xc': s, 'yc': np.zeros(n), 'halfWidth': np.full(n, 0.64), 'L': float(length),
            'obstacles_world': []}


# ---- the parameter ------------------------------------------------------------------------------

def test_the_shipped_default_is_todays_controller():
    assert get_value('mpc_avoidance_mode') == 'centerline'


def test_an_unknown_mode_refuses_to_start():
    with pytest.raises(ValueError):
        _node('old')


def test_legacy_reads_its_two_numbers_from_stack_params():
    node = _node('legacy')
    assert node.obstacle_target_shift == float(get_value('obstacle_target_shift_m')) == 0.30
    assert node.legacy_w_obs == float(get_value('mpc_legacy_w_obs')) == 8.0


def test_the_legacy_shift_fits_inside_the_narrow_corridor():
    assert float(get_value('obstacle_target_shift_m')) < float(get_value('corr_half_width_start_m'))


def test_legacy_turns_every_mpc_dodge_term_off_whatever_stack_params_says():
    node = _node('legacy')
    MPCController._init_dodge_params(node)
    assert node.obs_vel_trust == 0.0
    assert node.obs_wake_weight == 0.0
    assert node.obs_body_probes is False
    assert node.obstacle_side_mode == 'fixed'
    assert node.corridor_lateral_reanchor is True


def test_centerline_keeps_the_shipped_mpc_dodge_set():
    node = _node('centerline')
    MPCController._init_dodge_params(node)
    assert node.obs_vel_trust == float(get_value('mpc_obs_vel_trust'))
    assert node.obs_wake_weight == float(get_value('mpc_obs_wake_weight'))
    assert node.obs_body_probes == bool(get_value('mpc_obs_body_probes'))


def test_legacy_skips_the_side_rule():
    node = _node('legacy')
    node.x, node.y, node.yaw = 0.0, 0.0, 0.0
    obstacles = [(1.5, 0.0, 0.3)]
    corridor = {'psiRef': 0.0, 'L': 3.0}
    out, motions = MPCController._side_ruled_obstacles(node, obstacles, [None], corridor)
    assert out == obstacles
    assert motions == [None]


def test_centerline_still_applies_the_side_rule():
    node = _node('centerline')
    node.x, node.y, node.yaw = 0.0, 0.0, 0.0
    node.obstacle_side_mode = 'fixed'
    node.obstacle_tie_band_m = float(get_value('mpc_obstacle_tie_band_m'))
    node.obstacle_tie_side = 1.0
    node.corr_L_base = 3.0
    out, _ = MPCController._side_ruled_obstacles(node, [(1.5, 0.0, 0.3)], [None],
                                                 {'psiRef': 0.0, 'L': 3.0})
    assert out != [(1.5, 0.0, 0.3)]


def test_control_loop_shifts_the_target_only_in_legacy():
    import inspect
    src = inspect.getsource(MPCController.control_loop)
    i_target = src.index('self.compute_local_target(')
    i_legacy = src.index('self._legacy_deflected_target(')
    i_cache = src.index('self.cached_pref_nom = pref_nom')
    assert i_target < i_legacy < i_cache
    assert "== 'legacy'" in src[i_target:i_legacy]


# ---- the legacy target (the pre-7c217d95 compute_local_target) ----------------------------------

def _legacy_target(node, x, corridor):
    p = MPCController.compute_local_target(_LookaheadFake(), x, corridor)
    return MPCController._legacy_deflected_target(node, x, corridor, p)


class _LookaheadFake:
    corr_L_base = 3.0
    corr_lookahead_frac = 0.5
    corr_lookahead_reach_margin = 1.25
    N = 20
    ts = 0.1
    vdes = 0.5

    def get_logger(self):
        return _Logger()

    def _corridor_lookahead(self, corridor):
        return MPCController._corridor_lookahead(self, corridor)


def test_without_obstacles_the_legacy_target_is_the_centreline_point():
    corridor = _straight_corridor()
    clean = MPCController.compute_local_target(_LookaheadFake(), [0.0, 0.0, 0.0, 0.5], corridor)
    legacy = _legacy_target(_node('legacy'), [0.0, 0.0, 0.0, 0.5], corridor)
    assert np.allclose(legacy, clean)


@pytest.mark.parametrize('shift', [0.30, 0.12])
def test_an_obstacle_on_the_target_shifts_it_by_the_parameter(shift):
    """Lookahead exactly on the obstacle centre: penetration 1, so the full shift, sideways."""
    corridor = _straight_corridor()
    corridor['obstacles_world'] = [(1.5, 0.0, 0.15)]
    node = _node('legacy', obstacle_target_shift_m=shift)
    target = _legacy_target(node, [0.0, 0.0, 0.0, 0.5], corridor)
    r_safe = 0.15 + node.car_radius + node.avoidance_margin
    assert float(target[1]) == pytest.approx(shift, abs=1e-6)
    assert float(target[0]) == pytest.approx(1.5 + r_safe, abs=1e-6)


def test_a_vanished_obstacle_coasts_the_shift_out_over_five_ticks():
    corridor = _straight_corridor()
    node = _node('legacy')
    node.target_smoothing_alpha = 1.0  # the coast alone
    corridor['obstacles_world'] = [(1.5, 0.0, 0.15)]
    first = _legacy_target(node, [0.0, 0.0, 0.0, 0.5], corridor)
    corridor['obstacles_world'] = []
    lateral = [float(_legacy_target(node, [0.0, 0.0, 0.0, 0.5], corridor)[1])
               for _ in range(6)]
    assert lateral == pytest.approx([first[1] * k / 5 for k in (4, 3, 2, 1, 0, 0)], abs=1e-9)


def test_the_smoother_halves_the_step_and_a_new_move_forgets_it():
    corridor = _straight_corridor()
    node = _node('legacy')
    MPCController._legacy_deflected_target(node, [0, 0, 0, 0], corridor, np.array([1.0, 0.0]))
    half = MPCController._legacy_deflected_target(node, [0, 0, 0, 0], corridor,
                                                  np.array([1.0, 1.0]))
    assert half == pytest.approx([1.0, 0.5])
    MPCController._reset_legacy_target(node)
    fresh = MPCController._legacy_deflected_target(node, [0, 0, 0, 0], corridor,
                                                   np.array([1.0, 1.0]))
    assert fresh == pytest.approx([1.0, 1.0])


# ---- closed loop: the field case, a person 1 m ahead of the bumper ------------------------------

# Person: body 0.25 m + class margin 0.5 m (obstacle_class_margin_m) as one disc, centre 1.65 m
# ahead of base_link (bumper ~0.4 m ahead of it, 1.0 m gap) and 0.1 m off the line (outside the
# side rule's band, as in the field run).
PERSON = (1.65, 0.1, 0.75)
PERSON_BODY_R = 0.25


def _drive(mode, obstacle=PERSON, duration=12.0):
    fake = cl._fake_controller()
    fake.goal_distance = None
    fake.goal_anchor_odom = None
    fake.drive_cmd = {'mode': 'straight', 'speed': cl.SPEED_MPS}
    fake.mpc_avoidance_mode = mode
    fake.obstacle_target_shift = float(get_value('obstacle_target_shift_m'))
    fake.target_smoothing_alpha = 0.5
    fake.deflection_decay_ticks = 5
    MPCController._reset_legacy_target(fake)
    weights = cl.shipped_weights()
    if mode == 'legacy':
        weights['w_obs'] = float(get_value('mpc_legacy_w_obs'))
    period = float(get_value('corridor_update_period'))
    x, last_u, warm, corridor, last_build = np.zeros(4), np.zeros(2), None, None, None
    xs = []
    for tick in range(int(round(duration / rig.TS))):
        t = tick * rig.TS
        if corridor is None or t - last_build >= period - 1e-9:
            fake.x, fake.y, fake.yaw = float(x[0]), float(x[1]), float(x[2])
            corridor = MPCController.build_straight_corridor(fake, x)
            last_build = t
        fake.x, fake.y, fake.yaw = float(x[0]), float(x[1]), float(x[2])
        solver_obs, _ = MPCController._side_ruled_obstacles(fake, [obstacle], [None], corridor)
        corridor['obstacles_world'] = solver_obs
        corridor['d_safe'] = rig.DMIN
        corridor['car_radius'] = rig.CAR_RADIUS
        corridor['avoidance_margin'] = rig.AVOIDANCE_MARGIN
        pref_nom = MPCController.compute_local_target(fake, x, corridor)
        if mode == 'legacy':
            pref_nom = MPCController._legacy_deflected_target(fake, x, corridor, pref_nom)
        u0, info = solve_mpc_step(
            x0=x, last_u=last_u, pref_nom=pref_nom, corridor=corridor,
            horizon=rig.HORIZON, ts=rig.TS, params=rig.PARAMS, limits=rig.LIMITS,
            weights=dict(weights), obstacles=solver_obs, dmin=rig.DMIN, vdes=fake.vdes,
            solver='rti', warm_start_z=warm)
        warm = shift_warm_start(info.get('zopt'), rig.HORIZON) if info else None
        v_cmd, _ = clamp_drive_speed(x[3] + u0[1] * rig.TS, rig.MAX_FORWARD_SPEED, 0.0)
        u_plant = np.array([u0[0], (v_cmd - x[3]) / rig.TS])
        x = np.array(f1tenth_state_fcn_dt_beta(x, u_plant, rig.TS, rig.WHEELBASE, rig.LR))
        last_u = np.asarray(u0, dtype=float)
        xs.append(x.copy())
    s = np.array(xs)
    half = rig.WHEELBASE / 2.0
    cx, cy = s[:, 0] + half * np.cos(s[:, 2]), s[:, 1] + half * np.sin(s[:, 2])
    dist = np.hypot(cx - obstacle[0], cy - obstacle[1])
    past = np.nonzero(cx > obstacle[0] + obstacle[2])[0]
    t_pass = float((past[0] + 1) * rig.TS) if len(past) else math.inf
    return {'t_pass': t_pass, 'min_dist': float(np.min(dist))}


@pytest.fixture(scope='module')
def person_runs():
    return {mode: _drive(mode) for mode in ('centerline', 'legacy')}


@pytest.mark.parametrize('mode', ['centerline', 'legacy'])
def test_both_modes_get_past_a_person_1m_ahead_from_a_standstill(person_runs, mode):
    assert person_runs[mode]['t_pass'] < 10.0


def test_legacy_commits_sooner_than_centerline(person_runs):
    assert person_runs['legacy']['t_pass'] < person_runs['centerline']['t_pass']


@pytest.mark.parametrize('mode', ['centerline', 'legacy'])
def test_the_car_disc_stays_clear_of_the_persons_body(person_runs, mode):
    assert person_runs[mode]['min_dist'] > PERSON_BODY_R + rig.CAR_RADIUS + 0.2


def test_centerline_keeps_the_class_margin_and_legacy_does_not(person_runs):
    """The trade the 2026-11-18 branch made, measured: legacy cuts into the person's margin."""
    touch = PERSON[2] + rig.CAR_RADIUS
    assert person_runs['centerline']['min_dist'] > touch
    assert person_runs['legacy']['min_dist'] < touch
