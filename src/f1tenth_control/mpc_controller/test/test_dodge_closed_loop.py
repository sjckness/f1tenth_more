"""
mpc-dodge: moving obstacles, the informed side rule and the return onto the line, closed loop.

THE RIG is test_centerline_target_closed_loop.py's (real build_straight_corridor,
compute_local_target and RTI solve_mpc_step around f1tenth_state_fcn_dt_beta, shipped weights from
stack_params), with the node's own per-tick pieces of mpc-dodge run on the stand-in exactly as
control_loop runs them: _side_ruled_obstacles, _attach_obstacle_motion, _compute_return_shape and
_tick_weights. The obstacle MOVES (kinematic, it does not react to the car); the "perceived" list
is its true position with its true velocity (plus Gaussian noise of sigma in the robustness cases),
the track id its index, v_valid as given.

Cases (the work order asks for few, the necessary ones):
  * a D0.5 cylinder crossing 2.5 m ahead at 0.4 m/s, from the right and from the left (Isaac
    M1/M3): the static MPC against mpc_obs_vel_trust 1.0;
  * a cylinder that walks in from the right and stops on the line (Isaac M4): the informed side
    rule passes on the side it came from;
  * robustness: velocity noise of growing sigma and v_valid false must not do worse than the static
    MPC;
  * the return (corridor_lateral_reanchor false + mpc_return_*): a car 1.0 m off the line with no
    obstacle, and the D0.5 on the line: time back under 0.10 m, overshoot, crossings.

Print the runs: python3 -m pytest test/test_dodge_closed_loop.py -s -k table
"""

import math

from f1tenth_params.corridor_geometry import CORRIDOR_WIDTH_RATIO
from f1tenth_params.param_defaults import get_value
from mpc_controller.drive_limits import clamp_drive_speed
from mpc_controller.MPC_corr import MPCController
from mpc_controller.mpc_solver import shift_warm_start, solve_mpc_step
from mpc_controller.obstacle_motion import Motion
from mpc_controller.side_rule import side_sign, SideLatch
from mpc_controller.vehicle_model import f1tenth_state_fcn_dt_beta
import numpy as np
import pytest
from test_centerline_target_closed_loop import shipped_weights, STEER_DEADBAND_RAD
import test_object_approach_closed_loop as rig

SPEED_MPS = 0.5
R_CYL = 0.25
TOL_M = 0.03                       # the Isaac harness's "good run" tolerance on R_safe
RETURN_TOL_M = 0.10
# The return configuration under test (also what the Isaac A/B starts from).
# Picked on the rig from gain {0, 1} x ratio {1.5, 2.5, 4} x psi {1, 0.5} x L_min {1, 1.5}: ratio
# 1.5 asks for arcs past full lock (no return at all), 4 returns in ~6.5-9 s, 2.5 in 5-7 s.
RETURN_FLAGS = {'corridor_lateral_reanchor': False, 'return_gain': 1.0,
                'return_lookahead_ratio': 2.5, 'return_psi_scale': 0.5,
                'return_lookahead_min_m': 1.0,
                'return_max_heading_deg': 35.0, 'return_overshoot_max_m': 0.15}
FIXED_TRUST = {'obs_vel_trust': 1.0}
# The rig's baseline is the STATIC controller (every mpc-dodge term off), whatever stack_params
# ships: since the car deploy (mpc-dodge-car) the shipped defaults are the Isaac-tuned set, and
# 'static' must stay static. Each case names its flags on top of this; the 'shipped' cases run the
# stack_params defaults themselves.
STATIC = {'obs_vel_trust': 0.0, 'obs_wake_weight': 0.0, 'obs_body_probes': False}


def r_safe(r):
    return r + rig.CAR_RADIUS + rig.AVOIDANCE_MARGIN


def _fake(**flags):
    fake = rig._ObjectMPC(standoff=1.0, speed=SPEED_MPS)
    fake.corr_wmin = float(get_value('corr_half_width_start_m'))
    fake.corr_wmax = fake.corr_wmin * CORRIDOR_WIDTH_RATIO
    fake.corridor_heading_return = bool(get_value('corridor_heading_return'))
    fake.goal_distance = 6.0
    fake.goal_start_xy = (0.0, 0.0)
    fake.goal_anchor_odom = (0.0, 0.0, 0.0)
    fake.psi_init_corridor = 0.0
    fake.obstacle_tie_band_m = float(get_value('mpc_obstacle_tie_band_m'))
    fake.obstacle_tie_side = side_sign(get_value('mpc_obstacle_tie_side'))
    # the mpc-dodge parameters, shipped defaults (MPCController._init_dodge_params) ...
    fake.obs_vel_trust = float(get_value('mpc_obs_vel_trust'))
    fake.obs_vel_lead_s = float(get_value('mpc_obs_vel_lead_s'))
    fake.obs_pred_max_s = float(get_value('mpc_obs_pred_max_s'))
    fake.obs_wake_weight = float(get_value('mpc_obs_wake_weight'))
    fake.obs_wake_horizon_s = float(get_value('mpc_obs_wake_horizon_s'))
    fake.obstacle_side_mode = str(get_value('mpc_obstacle_side_mode'))
    fake.obstacle_side_release_m = float(get_value('mpc_obstacle_side_release_m'))
    fake.obstacle_side_vlat_min = float(get_value('mpc_obstacle_side_vlat_min'))
    fake.corridor_lateral_reanchor = bool(get_value('corridor_lateral_reanchor'))
    fake.return_gain = float(get_value('mpc_return_gain'))
    fake.return_lookahead_ratio = float(get_value('mpc_return_lookahead_ratio'))
    fake.return_lookahead_min_m = float(get_value('mpc_return_lookahead_min_m'))
    fake.return_psi_scale = float(get_value('mpc_return_psi_scale'))
    fake.return_max_heading_deg = float(get_value('mpc_return_max_heading_deg'))
    fake.return_overshoot_max_m = float(get_value('mpc_return_overshoot_max_m'))
    fake._side_latch = SideLatch()
    fake._move_line_origin = None
    fake.costmap_boundaries_world = []
    fake.obs_body_probes = bool(get_value('mpc_obs_body_probes'))
    # ... the static baseline unless the case asks for the shipped defaults, then its flags
    if not flags.pop('shipped', False):
        for k, v in STATIC.items():
            setattr(fake, k, v)
    for k, v in flags.items():
        setattr(fake, k, v)
    weights = shipped_weights()
    fake._weights_for = lambda corridor: weights
    return fake


class Run:
    def __init__(self):
        self.xs, self.deltas, self.obs, self.t = [], [], [], []

    @property
    def states(self):
        return np.array(self.xs)

    def centres(self):
        s = self.states
        h = rig.WHEELBASE / 2.0
        return s[:, 0] + h * np.cos(s[:, 2]), s[:, 1] + h * np.sin(s[:, 2])

    def min_margin(self):
        """
        Return min over time of the car-centre distance to the TRUE obstacle - R_safe.

        min over time of (car-centre distance to the TRUE obstacle) - R_safe, None if none.
        """
        if not self.obs or not self.obs[0]:
            return None
        cx, cy = self.centres()
        best = math.inf
        for i, obs in enumerate(self.obs):
            for ox, oy, r in obs:
                best = min(best, math.hypot(cx[i] - ox, cy[i] - oy) - r_safe(r))
        return best

    def contact(self):
        """
        Return min over time of the car disc's clearance to the obstacle's disc (< 0 = contact).

        min over time of the car disc's clearance to the obstacle's disc (< 0 = contact).
        """
        cx, cy = self.centres()
        best = math.inf
        for i, obs in enumerate(self.obs):
            for ox, oy, r in obs:
                best = min(best, math.hypot(cx[i] - ox, cy[i] - oy) - r - rig.CAR_RADIUS)
        return best

    def side_at(self, x_obs):
        """Sign of the car's lateral offset when it passes x_obs, relative to the obstacle's y."""
        s = self.states
        i = int(np.argmin(np.abs(s[:, 0] - x_obs)))
        oy = self.obs[i][0][1] if self.obs[i] else 0.0
        return float(np.sign(s[i, 1] - oy))

    def sign_changes(self, deadband=STEER_DEADBAND_RAD):
        d = np.asarray(self.deltas)
        d = d[self.states[:, 3] > 0.05]
        signs = np.sign(np.where(np.abs(d) < deadband, 0.0, d))
        signs = signs[signs != 0.0]
        return int(np.sum(signs[1:] != signs[:-1]))

    def return_metrics(self, x_after):
        """
        Return (time back under RETURN_TOL_M, overshoot, crossings of y = 0) after x_after.

        After the car's x passes x_after: time until |y| < RETURN_TOL_M for good (s, None if
        never), the overshoot (largest excursion on the far side of y = 0 after the first
        crossing) and the number of crossings of y = 0.
        """
        s = self.states
        t = np.asarray(self.t)
        k0 = int(np.argmax(s[:, 0] >= x_after)) if np.any(s[:, 0] >= x_after) else len(s)
        y = s[k0:, 1]
        if len(y) == 0:
            return None, None, 0
        start_sign = np.sign(y[0]) if abs(y[0]) > 1e-6 else 1.0
        out = np.abs(y) >= RETURN_TOL_M
        t_back = None
        if not out[-1]:
            last_out = int(np.max(np.nonzero(out)[0])) if np.any(out) else -1
            t_back = float(t[k0 + last_out + 1] - t[k0])
        far = -start_sign * y
        overshoot = float(max(0.0, np.max(far)))
        crossings = int(np.sum(np.sign(y[1:]) != np.sign(y[:-1])))
        return t_back, overshoot, crossings


def drive(obstacles_at, duration, x0=(0.0, 0.0, 0.0), sigma_v=0.0, v_valid=True, seed=0,
          line_origin=None, drive_move=False, **flags):
    """
    Drive the move with obstacles_at(t) -> [(x, y, r, vx, vy)] true obstacles; return a Run.

    obstacles_at(t) -> [(x, y, r, vx, vy)] true obstacles; returns a Run.

    line_origin: the move's line origin when the car starts OFF it (the return cases: the move
    began on y = 0 and the car was pushed off, e.g. by an earlier obstacle).
    """
    rng = np.random.default_rng(seed)
    fake = _fake(**flags)
    if line_origin is not None:
        fake._move_line_origin = tuple(line_origin)
    if drive_move:
        # a mission's 'drive: straight' move: build_straight_corridor's drive branch, not the
        # goal_distance one (what the Isaac harness runs)
        fake.goal_distance = None
        fake.goal_anchor_odom = None
        fake.drive_cmd = {'mode': 'straight', 'speed': SPEED_MPS}
    period = float(get_value('corridor_update_period'))
    x = np.array([x0[0], x0[1], x0[2], 0.0])
    last_u, warm, corridor, last_build = np.zeros(2), None, None, None
    run = Run()
    for tick in range(int(round(duration / rig.TS))):
        t = tick * rig.TS
        if corridor is None or t - last_build >= period - 1e-9:
            fake.x, fake.y, fake.yaw = float(x[0]), float(x[1]), float(x[2])
            corridor = MPCController.build_straight_corridor(fake, x)
            last_build = t
        fake.x, fake.y, fake.yaw = float(x[0]), float(x[1]), float(x[2])
        truth = obstacles_at(t)
        planning = [(ox, oy, r) for ox, oy, r, _vx, _vy in truth]
        motions = [Motion(i, vx + rng.normal(0.0, sigma_v), vy + rng.normal(0.0, sigma_v), v_valid)
                   for i, (_x, _y, _r, vx, vy) in enumerate(truth)]
        solver_obs, motions = MPCController._side_ruled_obstacles(
            fake, planning, motions, corridor)
        corridor['obstacles_world'] = solver_obs
        MPCController._attach_obstacle_motion(fake, corridor, solver_obs, motions)
        fake._return_shape = MPCController._compute_return_shape(fake, corridor, planning)
        fake._planning_obstacles_last = planning
        corridor['d_safe'] = rig.DMIN
        corridor['car_radius'] = rig.CAR_RADIUS
        corridor['avoidance_margin'] = rig.AVOIDANCE_MARGIN
        pref_nom = MPCController.compute_local_target(fake, x, corridor)
        u0, info = solve_mpc_step(
            x0=x, last_u=last_u, pref_nom=pref_nom, corridor=corridor,
            horizon=rig.HORIZON, ts=rig.TS, params=rig.PARAMS, limits=rig.LIMITS,
            weights=MPCController._tick_weights(fake, corridor), obstacles=solver_obs,
            dmin=rig.DMIN, vdes=fake.vdes, solver='rti', warm_start_z=warm)
        warm = shift_warm_start(info.get('zopt'), rig.HORIZON) if info else None
        v_cmd, _ = clamp_drive_speed(x[3] + u0[1] * rig.TS, rig.MAX_FORWARD_SPEED, 0.0)
        u_plant = np.array([u0[0], (v_cmd - x[3]) / rig.TS])
        x = np.array(f1tenth_state_fcn_dt_beta(x, u_plant, rig.TS, rig.WHEELBASE, rig.LR))
        last_u = np.asarray(u0, dtype=float)
        run.xs.append(x.copy())
        run.deltas.append(float(u0[0]))
        run.obs.append([(ox, oy, r) for ox, oy, r, _vx, _vy in obstacles_at(t + rig.TS)])
        run.t.append(t + rig.TS)
    return run


def crossing(side_from, speed=0.4, ahead=2.5, start_lat=1.2, r=R_CYL):
    """Return a cylinder crossing the line `ahead` m ahead, from the right (-1) or left (+1)."""
    vy = -side_from * speed
    return lambda t: [(ahead, side_from * start_lat + vy * t, r, 0.0, vy)]


def walk_in_and_stop(side_from=-1.0, speed=0.4, ahead=2.5, start_lat=1.2, r=R_CYL):
    """
    Return M4: a cylinder that walks in from side_from and stops on the line.

    M4: walks in from `side_from` and stops on the line.
    """
    t_stop = start_lat / speed
    vy = -side_from * speed

    def at(t):
        if t < t_stop:
            return [(ahead, side_from * start_lat + vy * t, r, 0.0, vy)]
        return [(ahead, 0.0, r, 0.0, 0.0)]
    return at


def on_line(r=R_CYL, ahead=1.5):
    return lambda t: [(ahead, 0.0, r, 0.0, 0.0)]


DURATION = 12.0
# Crossings timed to reach the car's line while the nose is already past: the cylinder closes in
# on the flank (ahead m, start lateral m, speed m/s). Isaac phase 3 diagnosis: a third of the
# dodge failures looked like this.
FLANK_CASES = {'right_2.0': (2.0, -1.2, 0.3), 'left_1.8': (1.8, 1.4, 0.35)}
# M4 in the rig: far enough ahead that the static MPC has not swerved yet when the cylinder stops
# on the line (2.5 m ahead it reacts to the cylinder still walking in, and the stop is no longer
# on the car's line, so no side rule applies).
M4_AHEAD = 4.0


@pytest.fixture(scope='module')
def runs():
    out = {}
    for name, side in (('from_right', -1.0), ('from_left', 1.0)):
        out[f'{name}/static'] = drive(crossing(side), DURATION)
        out[f'{name}/trust1'] = drive(crossing(side), DURATION, **FIXED_TRUST)
        out[f'{name}/shipped'] = drive(crossing(side), DURATION, shipped=True)
    out['m4/fixed'] = drive(walk_in_and_stop(ahead=M4_AHEAD), DURATION)
    out['m4/informed'] = drive(walk_in_and_stop(ahead=M4_AHEAD), DURATION,
                               obstacle_side_mode='informed')
    for sigma in (0.1, 0.3, 0.6):
        out[f'from_right/trust1_noise{sigma}'] = drive(crossing(-1.0), DURATION, sigma_v=sigma,
                                                       **FIXED_TRUST)
    out['from_right/trust1_invalid'] = drive(crossing(-1.0), DURATION, v_valid=False,
                                             **FIXED_TRUST)
    for name, (ahead, lat, v) in FLANK_CASES.items():
        side = -1.0 if lat < 0 else 1.0
        out[f'flank/{name}/trust1'] = drive(
            crossing(side, speed=v, ahead=ahead, start_lat=abs(lat)), DURATION, **FIXED_TRUST)
        out[f'flank/{name}/trust1_body'] = drive(
            crossing(side, speed=v, ahead=ahead, start_lat=abs(lat)), DURATION,
            obs_body_probes=True, **FIXED_TRUST)
    out['return/free_offset'] = drive(lambda t: [], 14.0, x0=(0.0, 1.0, 0.0),
                                      line_origin=(0.0, 0.0), **RETURN_FLAGS)
    out['return/free_offset_today'] = drive(lambda t: [], 14.0, x0=(0.0, 1.0, 0.0),
                                            line_origin=(0.0, 0.0))
    out['return/on_line'] = drive(on_line(r=0.5), 14.0, **RETURN_FLAGS)
    out['return/on_line_drive'] = drive(on_line(r=0.5), 14.0, drive_move=True, **RETURN_FLAGS)
    return out


def test_print_the_table(runs, capsys):
    """Not an assertion: the numbers. Run with -s -k table."""
    with capsys.disabled():
        print(f'\n{"run":<34} {"margin":>7} {"contact":>8} {"side":>5} {"peak|y|":>7} '
              f'{"y end":>6} {"x end":>6} {"sgn":>4} {"t_back":>6} {"oversh":>6} {"cross":>5}')
        for name, run in runs.items():
            s = run.states
            m = run.min_margin()
            c = run.contact() if run.obs and run.obs[0] else None
            side = (run.side_at(M4_AHEAD if name.startswith('m4') else 2.5)
                    if run.obs and run.obs[0] else 0.0)
            tb, ov, cr = run.return_metrics(3.5 if 'on_line' in name else 0.0)
            print(f'{name:<34} {"" if m is None else f"{m:+.3f}":>7} '
                  f'{"" if c is None else f"{c:+.3f}":>8} {side:>+5.0f} '
                  f'{np.max(np.abs(s[:, 1])):>7.3f} {s[-1, 1]:>+6.2f} {s[-1, 0]:>6.2f} '
                  f'{run.sign_changes():>4} '
                  f'{"-" if tb is None else f"{tb:.1f}":>6} {ov:>6.3f} {cr:>5}')


# ---- moving obstacle: predicted discs ---------------------------------------------------------

@pytest.mark.parametrize('name, behind', [('from_right', -1.0), ('from_left', 1.0)])
def test_trust1_passes_behind_a_crossing_obstacle_without_contact(runs, name, behind):
    run = runs[f'{name}/trust1']
    assert run.contact() > 0.0
    assert run.min_margin() >= -TOL_M
    # behind it: on the side it came from, i.e. the car's y relative to the cylinder when the car
    # passes its x has the sign of where the cylinder started
    assert run.side_at(2.5) == behind
    assert run.states[-1, 0] > 4.0
    assert run.sign_changes() <= 3


@pytest.mark.parametrize('name', ['from_right', 'from_left'])
def test_trust1_keeps_more_clearance_than_the_static_mpc(runs, name):
    assert runs[f'{name}/trust1'].min_margin() >= runs[f'{name}/static'].min_margin() - 1e-3


# ---- informed side rule -----------------------------------------------------------------------

def test_informed_rule_passes_a_stopped_obstacle_on_the_side_it_came_from(runs):
    run = runs['m4/informed']
    assert run.contact() > 0.0
    assert run.side_at(M4_AHEAD) == -1.0              # came from the right
    assert run.states[-1, 0] > 4.0
    assert run.sign_changes() <= 3


def test_fixed_rule_still_passes_on_the_configured_side(runs):
    assert runs['m4/fixed'].side_at(M4_AHEAD) == side_sign(get_value('mpc_obstacle_tie_side'))


# ---- robustness: a noisy or invalid velocity is never worse than the static MPC ---------------

@pytest.mark.parametrize('case', ['trust1_noise0.1', 'trust1_noise0.3', 'trust1_noise0.6',
                                  'trust1_invalid'])
def test_noisy_or_invalid_velocity_does_no_worse_than_static(runs, case):
    static = runs['from_right/static']
    run = runs[f'from_right/{case}']
    assert run.min_margin() >= min(static.min_margin(), 0.0) - TOL_M
    assert run.states[-1, 0] > 4.0


def test_invalid_velocity_is_exactly_the_static_mpc(runs):
    np.testing.assert_allclose(runs['from_right/trust1_invalid'].states,
                               runs['from_right/static'].states, atol=1e-9)


# ---- the return onto the line -----------------------------------------------------------------

def test_today_the_offset_car_never_comes_back(runs):
    """corridor_lateral_reanchor true (shipped): the corridor follows the car."""
    assert abs(runs['return/free_offset_today'].states[-1, 1]) > 0.9


def test_return_from_1m_offset_settles_with_a_small_overshoot(runs):
    run = runs['return/free_offset']
    t_back, overshoot, crossings = run.return_metrics(0.0)
    assert t_back is not None and t_back < 8.0
    assert overshoot <= max(RETURN_FLAGS['return_overshoot_max_m'], 0.15 * 1.0) + 1e-6
    assert crossings <= 2                 # one crossing, maybe one touch back: no oscillation
    assert run.sign_changes() <= 3


@pytest.mark.parametrize('name', ['return/on_line', 'return/on_line_drive'])
def test_return_after_an_obstacle_on_the_line_comes_back_under_10_cm(runs, name):
    run = runs[name]
    assert run.min_margin() >= -TOL_M
    t_back, overshoot, crossings = run.return_metrics(2.0)
    assert t_back is not None
    assert abs(run.states[-1, 1]) < RETURN_TOL_M
    assert overshoot <= 0.15 + 1e-6


# ---- body probes ------------------------------------------------------------------------------

@pytest.mark.parametrize('name', sorted(FLANK_CASES))
def test_body_probes_keep_r_safe_when_a_crossing_obstacle_closes_on_the_flank(runs, name):
    body = runs[f'flank/{name}/trust1_body']
    assert body.min_margin() >= -TOL_M
    assert body.min_margin() >= runs[f'flank/{name}/trust1'].min_margin()
    assert body.states[-1, 0] > 4.0


# ---- the shipped (car deploy) set: docs/mpc_dodge/mpc_params_tuned.yaml -------------------------

@pytest.mark.parametrize('name', ['from_right', 'from_left'])
def test_shipped_defaults_pass_a_crossing_obstacle_at_least_as_well_as_trust1(runs, name):
    shipped = runs[f'{name}/shipped']
    assert shipped.contact() > 0.0
    assert shipped.min_margin() >= -TOL_M
    assert shipped.min_margin() >= runs[f'{name}/trust1'].min_margin() - TOL_M
    assert shipped.states[-1, 0] > 4.0
