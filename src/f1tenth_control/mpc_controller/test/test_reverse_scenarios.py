"""
Closed-loop reverse scenarios on the nominal model (test/reverse_harness.py).

Each scenario runs once per module; the plots, GIFs and the metrics table in
output/reverse/ come from the same harness (python3 test/reverse_harness.py
--out ../../../output/reverse).

THE DOOR SCENARIOS FAIL and are marked xfail(strict=True): if a change makes
the car go through the door, the marker turns the pass into a failure that
says so. Why they fail is in output/reverse/report.md: the QP's collision
model (nose probes and the rear-axle point) cannot see the body's corners at
a door jamb, which the lattice cost-to-go does, and the selector dithers on
the difference.
"""

import pytest

import reverse_harness as harness

S = harness.scenarios()


def _run(name, allow_reverse=True):
    return harness.metrics(harness.run_scenario(S[name], allow_reverse))


@pytest.fixture(scope='module')
def free():
    return {'pose': _run('a_free_corridor'), 'pose_off': _run('a_free_corridor', False),
            'distance': _run('a2_free_corridor_distance'),
            'distance_off': _run('a2_free_corridor_distance', False)}


@pytest.fixture(scope='module')
def behind():
    return {'near': _run('d_goal_behind_1p5'), 'far': _run('e_goal_far_behind')}


# ------------------------------------------------------------------ a

@pytest.mark.parametrize('key', ['pose', 'distance'])
def test_a_free_corridor_never_reverses(free, key):
    m = free[key]
    assert m['reached'] and m['reverse_m'] == 0.0
    assert m['direction_changes'] == 0 and m['selector_switches'] == 0


@pytest.mark.parametrize('key', ['pose', 'distance'])
def test_a_free_corridor_has_no_speed_dip_against_the_flag_off_run(free, key):
    on, off = free[key], free[key + '_off']
    assert on['min_cmd_speed_after_cruise'] >= off['min_cmd_speed_after_cruise'] - 0.1
    assert on['time_to_goal_s'] <= off['time_to_goal_s'] + 1.0


def test_a_free_corridor_without_a_goal_pose_is_the_flag_off_drive(free):
    # goal_distance: no cost-to-go; the forward candidate differs from today's
    # QP only in v >= 0 and the extra (inert) terms.
    on, off = free['distance'], free['distance_off']
    assert on['path_m'] == pytest.approx(off['path_m'], abs=0.05)


# ------------------------------------------------------------------ d, e

def test_d_goal_straight_behind_reverses(behind):
    m = behind['near']
    assert m['reached'] and m['reverse_m'] >= 1.0
    assert m['direction_changes'] <= 1


def test_e_goal_far_behind_turns_instead_of_a_long_reverse(behind):
    m = behind['far']
    assert m['reached'] and m['reverse_m'] < 0.5


# ------------------------------------------------------------------ f

def test_f_near_tie_changes_direction_at_most_once():
    d_tie, _table = harness.find_near_tie()
    m = harness.metrics(harness.run_scenario(harness.near_tie_scenario(d_tie), True))
    assert m['reached'] and m['direction_changes'] <= 1


# ------------------------------------------------------------------ b, c

@pytest.fixture(scope='module')
def doors():
    return {'b': _run('b_door_late'), 'c': _run('c_door_early')}


@pytest.mark.xfail(strict=True, reason='door entry not achieved; see output/reverse/report.md')
def test_c_door_seen_early_is_entered_directly(doors):
    m = doors['c']
    assert m['reached'] and m['reverse_m'] < 0.05 and not m['wall_contact']


@pytest.mark.xfail(strict=True, reason='door entry not achieved; see output/reverse/report.md')
def test_b_door_seen_late_brakes_reverses_once_and_enters(doors):
    m = doors['b']
    assert m['reached'] and m['reverse_m'] > 0.2 and m['direction_changes'] <= 2
    assert not m['wall_contact']


def test_b_and_c_never_touch_a_wall_while_trying(doors):
    for name, m in doors.items():
        assert not m['wall_contact'], name
