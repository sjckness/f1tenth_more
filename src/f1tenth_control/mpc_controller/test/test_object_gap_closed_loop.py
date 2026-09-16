"""go_to_object gaps in the closed-loop rig: gap_min is reachable, the default gap reaches.

Rig as test_object_approach_closed_loop.py, with the target also in the
obstacle list at its footprint radius (+ class margin, 0: no m* was found in
Part 1) and mpc_corr's /drive clamp on. The controller is sent what GoToObject
sends: centre distance = gap + nose_reach + r_target.

Classes: person 0.50 m wide (r 0.25), chair 0.45 m wide (r 0.225).
object_reached fires on a tick whose live r (mpc_corr's /mpc/object_status r,
the same quantity _assess_object_tick computes) is <= object_reach_tol_m 0.10.

Print the tables:
    python3 -m pytest test/test_object_gap_closed_loop.py -s -k table
"""

import math

import pytest

from f1tenth_params.object_geometry import gap_limits, nose_reach
from f1tenth_params.param_defaults import get_value

import test_object_approach_closed_loop as rig

NOSE_REACH_M = nose_reach(float(get_value('mpc_wheelbase_m')))
REACH_TOL_M = float(get_value('object_reach_tol_m'))
CLASSES = {'person': 0.50, 'chair': 0.45}
SCENARIOS = ('ahead 4 m', '45 deg off, 4 m', '70 deg off, 2.5 m', 'jitter 5 cm @ 12.5 Hz')


def approach(target_class, gap, scenario='ahead 4 m'):
    limits = gap_limits(target_class)
    radius = CLASSES[target_class] / 2.0
    res = rig.run_approach(standoff=gap + NOSE_REACH_M + radius,
                           obstacle_r=radius + limits.class_margin,
                           speed_limits=(1.0, 0.0), **rig.SCENARIOS[scenario])
    live_r = [f.r for f in res.flags if f is not None]
    tx, ty = res.targets[-1]
    x, y = res.xs[-1][0], res.xs[-1][1]
    return {
        'settled_gap': math.hypot(tx - x, ty - y) - NOSE_REACH_M - radius,
        'min_live_r': min(live_r),
        'reached': min(live_r) <= REACH_TOL_M,
        'reached_tick': next((t for t, r in enumerate(live_r) if r <= REACH_TOL_M), None),
        'limits': limits,
    }


@pytest.fixture(scope='module')
def gap_table():
    out = {}
    for cls in CLASSES:
        limits = gap_limits(cls)
        for label, gap in (('rest probe (gap 0)', 0.0), ('gap_min', limits.gap_min),
                           ('default gap', limits.default_gap)):
            out[(cls, label)] = (gap, approach(cls, gap))
    return out


@pytest.fixture(scope='module')
def scenario_table():
    return {(cls, scen): approach(cls, gap_limits(cls).default_gap, scen)
            for cls in CLASSES for scen in SCENARIOS}


def test_print_the_gap_table(gap_table, capsys):
    with capsys.disabled():
        print(f'\n{"class":<7} {"commanded":<20} {"gap m":>6} {"gap_min":>7} '
              f'{"settled gap":>11} {"min live r":>10}  object_reached')
        for (cls, label), (gap, res) in gap_table.items():
            print(f'{cls:<7} {label:<20} {gap:>6.2f} {res["limits"].gap_min:>7.2f} '
                  f'{res["settled_gap"]:>11.3f} {res["min_live_r"]:>+10.3f}  {res["reached"]}')


def test_print_the_scenario_table(scenario_table, capsys):
    with capsys.disabled():
        print(f'\n{"class":<7} {"scenario":<24} {"settled gap":>11} {"min live r":>10} '
              f'{"reached tick":>12}')
        for (cls, scen), res in scenario_table.items():
            print(f'{cls:<7} {scen:<24} {res["settled_gap"]:>11.3f} '
                  f'{res["min_live_r"]:>+10.3f} {str(res["reached_tick"]):>12}')


@pytest.mark.parametrize('cls', sorted(CLASSES))
def test_the_controller_rests_well_inside_gap_min(gap_table, cls):
    """Commanded gap 0: w_obs is what stops the car. gap_min must sit above that rest point
    by more than the reach tolerance, or a gap_min approach could not fire."""
    _gap, probe = gap_table[(cls, 'rest probe (gap 0)')]
    assert probe['reached'] is False
    assert probe['settled_gap'] < probe['limits'].gap_min - REACH_TOL_M


@pytest.mark.parametrize('cls', sorted(CLASSES))
@pytest.mark.parametrize('label', ['gap_min', 'default gap'])
def test_gap_min_and_the_default_gap_reach(gap_table, cls, label):
    gap, res = gap_table[(cls, label)]
    assert res['reached'], (cls, label, res)
    assert res['settled_gap'] == pytest.approx(gap, abs=REACH_TOL_M)


@pytest.mark.parametrize('cls', sorted(CLASSES))
@pytest.mark.parametrize('scenario', SCENARIOS)
def test_object_reached_fires_at_the_default_gap(scenario_table, cls, scenario):
    assert scenario_table[(cls, scenario)]['reached'], (cls, scenario)
