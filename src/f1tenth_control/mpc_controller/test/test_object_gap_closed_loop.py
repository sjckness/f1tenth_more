"""go_to_object gaps in the closed-loop rig, under the 0.4 m/s operating floor.

Rig as test_object_approach_closed_loop.py -- the real corridor, target
selection and solver, the stop latch and the floor exactly as MPC_corr's
object branch applies them, the shipped /drive clamp, and a braking plant
fitted to the archived stops -- with the target also in the obstacle list at
its footprint radius (+ class margin). The controller is sent what GoToObject
sends: centre distance = gap + nose_reach + r_target.

object_reached is judged by the REAL evaluator (f1tenth_behavior's
condition_eval) on every tick's status: live r, the stop latch and the
measured speed, with the stack_params tolerances. So "reached" here is the
mission's own verdict, not a rig approximation of it.

Classes: person 0.50 m wide (r 0.25), chair 0.45 m wide (r 0.225).

Every scenario runs under two braking models: MEASURED (the median archived
stop, 0.111 m from 0.4 m/s) and LONG (the longest archived stop, 0.133 m).
LONG is the one that tests how close a stop can bring the car.

Print the tables:
    python3 -m pytest test/test_object_gap_closed_loop.py -s -k table
"""

import math

import pytest

from f1tenth_behavior.mission.condition_eval import EvalContext, evaluate
from f1tenth_behavior.mission.mission_config import StopCondition
from f1tenth_behavior.mission.runtime import ObjectStatusSample
from f1tenth_params.object_geometry import gap_limits, nose_reach
from f1tenth_params.param_defaults import get_value
from mpc_controller.MPC_corr import MPCController

import test_object_approach_closed_loop as rig

NOSE_REACH_M = nose_reach(float(get_value('mpc_wheelbase_m')))
REST_SPEED_MPS = float(get_value('object_rest_speed_mps'))
CLASSES = {'person': 0.50, 'chair': 0.45}
SCENARIOS = ('ahead 4 m', '45 deg off, 4 m', '70 deg off, 2.5 m', 'jitter 5 cm @ 12.5 Hz')
BRAKING = {'measured': rig.BRAKING_MEASURED, 'long': rig.BRAKING_LONG}
REACHED = StopCondition(type='object_reached', params={})

# (class, label, gap or None for the class default) -- what the floor session runs.
FLOOR_RUNS = (('person', 'default gap', None), ('chair', 'default gap', None),
              ('person', 'gap 1.0', 1.0))


def _reached_tick(res):
    """First tick the mission's object_reached would fire on, or None."""
    for tick, (r, latched) in enumerate(zip(res.live_r, res.latched)):
        status = ObjectStatusSample(
            move_id='rig', r=r, alpha=0.0, target_age_s=0.0,
            target_behind_terminal=False, goal_watchdog=False, received_sec=0.0,
            stop_latched=latched, speed=float(res.xs[tick][3]))
        ctx = EvalContext(
            now=0.0, move_start_time=0.0, move_start_xy=None, current_xy=None,
            detected_classes={}, min_obstacle_distance=None, front_clearance=None,
            default_distance=None, object_status=status, object_move_id='rig',
            object_reach_tol_m=rig.REACH_TOL_M, object_rest_speed_mps=REST_SPEED_MPS)
        if evaluate(REACHED, ctx):
            return tick
    return None


def approach(target_class, gap, scenario='ahead 4 m', braking='measured'):
    """Run one approach and report it the way the floor sheet reads."""
    limits = gap_limits(target_class)
    gap = limits.default_gap if gap is None else gap
    radius = CLASSES[target_class] / 2.0
    res = rig.run_approach(standoff=gap + NOSE_REACH_M + radius,
                           obstacle_r=radius + limits.class_margin,
                           braking=BRAKING[braking], **rig.SCENARIOS[scenario])
    tick = _reached_tick(res)
    end = tick if tick is not None else len(res.xs) - 1
    tx, ty = res.targets[min(end, len(res.targets) - 1)]
    centre = [math.hypot(t[0] - s[0], t[1] - s[1])
              for s, t in zip(res.xs[1:], res.targets)]
    final_gap = math.hypot(tx - res.xs[end][0], ty - res.xs[end][1]) - NOSE_REACH_M - radius
    moving = sum(1 for v in res.published if v > 0.0)
    return {
        'gap': gap, 'limits': limits, 'res': res,
        'reached': tick is not None, 'reached_tick': tick,
        'final_gap': final_gap,
        'overshoot': max(0.0, gap - final_gap),
        'min_front_gap': min(centre) - NOSE_REACH_M - radius,
        'min_body_gap': min(centre) - radius - rig.CAR_RADIUS,
        'peak_speed': max(s[3] for s in res.xs),
        'clamps': sum(res.clamped), 'moving_ticks': moving,
        'latched_r': res.latched_r,
    }


@pytest.fixture(scope='module')
def gap_table():
    """Commanded gap 0 (a probe the loader would reject), gap_min, default gap."""
    out = {}
    for cls in CLASSES:
        limits = gap_limits(cls)
        for label, gap in (('probe (gap 0)', 0.0), ('gap_min', limits.gap_min),
                           ('default gap', limits.default_gap)):
            out[(cls, label)] = approach(cls, gap)
    return out


@pytest.fixture(scope='module')
def floor_table():
    """The floor session's approaches, every scenario, both braking models."""
    return {(cls, label, scen, brk): approach(cls, gap, scen, brk)
            for cls, label, gap in FLOOR_RUNS for scen in SCENARIOS for brk in BRAKING}


def test_print_the_gap_table(gap_table, capsys):
    with capsys.disabled():
        print(f'\n{"class":<7} {"commanded":<14} {"gap m":>6} {"gap_min":>7} '
              f'{"final gap":>9} {"latched r":>9}  object_reached')
        for (cls, label), res in gap_table.items():
            print(f'{cls:<7} {label:<14} {res["gap"]:>6.2f} {res["limits"].gap_min:>7.2f} '
                  f'{res["final_gap"]:>9.3f} {res["latched_r"]:>+9.3f}  {res["reached"]}')


def test_print_the_floor_table(floor_table, capsys):
    with capsys.disabled():
        print(f'\n{"class":<7} {"gap":<12} {"scenario":<22} {"braking":<8} {"reached":>7} '
              f'{"final":>6} {"oversh":>6} {"min frt":>7} {"min body":>8} {"v pk":>5} '
              f'{"clamps":>7}')
        for (cls, label, scen, brk), res in floor_table.items():
            print(f'{cls:<7} {label:<12} {scen:<22} {brk:<8} '
                  f'{("tick " + str(res["reached_tick"])) if res["reached"] else "NO":>7} '
                  f'{res["final_gap"]:>6.3f} {res["overshoot"]:>6.3f} '
                  f'{res["min_front_gap"]:>7.3f} {res["min_body_gap"]:>8.3f} '
                  f'{res["peak_speed"]:>5.2f} {res["clamps"]:>3}/{res["moving_ticks"]:<3}')


# ------------------------------------------------------------------ acceptance

@pytest.mark.parametrize('cls,label,gap', FLOOR_RUNS)
@pytest.mark.parametrize('scenario', SCENARIOS)
@pytest.mark.parametrize('braking', sorted(BRAKING))
def test_object_reached_fires(floor_table, cls, label, gap, scenario, braking):
    """ACCEPTANCE: the mission's own verdict fires on every floor approach."""
    res = floor_table[(cls, label, scenario, braking)]
    assert res['reached'], (cls, label, scenario, braking)


@pytest.mark.parametrize('cls,label,gap', FLOOR_RUNS)
@pytest.mark.parametrize('scenario', SCENARIOS)
@pytest.mark.parametrize('braking', sorted(BRAKING))
def test_the_final_gap_is_never_below_gap_min(floor_table, cls, label, gap, scenario, braking):
    """ACCEPTANCE: including the longest measured stop."""
    res = floor_table[(cls, label, scenario, braking)]
    assert res['final_gap'] >= res['limits'].gap_min, (cls, label, scenario, braking, res)


@pytest.mark.parametrize('cls,label,gap', FLOOR_RUNS)
@pytest.mark.parametrize('scenario', SCENARIOS)
@pytest.mark.parametrize('braking', sorted(BRAKING))
def test_the_closest_the_car_gets_is_outside_gap_min(floor_table, cls, label, gap,
                                                     scenario, braking):
    """Over the whole run, not only at rest.

    The closest point can come BEFORE the rest point: a car braking on an arc
    with its steering held passes nearest the target and then curves slightly
    away (by up to 2.5 cm on the 45 and 70 degree approaches under LONG
    braking). So the bound is checked on the run's minimum.
    """
    res = floor_table[(cls, label, scenario, braking)]
    assert res['min_front_gap'] >= res['limits'].gap_min, (cls, label, scenario, braking)


# ------------------------------------------------------------ what the floor changed

@pytest.mark.parametrize('cls', sorted(CLASSES))
def test_under_the_floor_gap_min_is_a_load_time_limit_not_a_reachability_one(gap_table, cls):
    """A commanded gap of 0 now REACHES, ending about 0.1 m from the object's edge.

    Before the floor, w_obs slowed the car to rest with its front 0.268 m from
    the edge, so a gap below gap_min could not complete. With the command
    floored at 0.4 m/s the solver can no longer slow the car, and the stop
    latch brings it in to about gap + 0.1. The only thing keeping a
    go_to_object move outside gap_min is now the mission loader rejecting
    gap_m < gap_min.
    """
    probe = gap_table[(cls, 'probe (gap 0)')]
    assert probe['reached'] is True
    assert probe['final_gap'] < probe['limits'].gap_min


@pytest.mark.parametrize('cls', sorted(CLASSES))
@pytest.mark.parametrize('label', ['gap_min', 'default gap'])
def test_gap_min_and_the_default_gap_rest_in_the_stop_window(gap_table, cls, label):
    res = gap_table[(cls, label)]
    lo, hi = rig.stop_window()
    assert res['reached']
    assert res['gap'] + lo <= res['final_gap'] <= res['gap'] + hi


def test_the_straight_approach_clamp_rate_is_caused_by_the_lookahead(monkeypatch):
    """The 0.4 m/s approach hits max_forward_speed_mps 0.5 on about a third of its moving ticks.

    Cause, pinned: the object corridor's lookahead (0.5 * L, 1.65 m at the
    start of a 4 m approach) lies about twice as far as the horizon reaches at
    0.4 m/s (N * ts * v = 0.80 m), so the terminal cost w_term pulls the
    horizon's end toward it and the solver asks for up to 0.65 m/s against
    w_v. Capping the lookahead at the horizon's reach -- NOT shipped, a
    diagnostic -- takes the clamp rate to zero. The solver bounds are left as
    they are.
    """
    shipped = approach('person', None, 'ahead 4 m')
    rate = shipped['clamps'] / shipped['moving_ticks']
    assert rate > 0.10, rate

    reach = rig.HORIZON * rig.TS * rig.MIN_MOVING_SPEED
    original = MPCController._corridor_lookahead
    monkeypatch.setattr(rig._ObjectMPC, '_corridor_lookahead',
                        lambda self, c: min(original(self, c), reach))
    capped = approach('person', None, 'ahead 4 m')
    assert capped['clamps'] == 0
