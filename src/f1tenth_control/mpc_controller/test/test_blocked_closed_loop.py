"""R011 and its neighbours in closed loop (M02 campaign fixes A1-A3).

campaign_rig: a 5 m goal_distance move at 0.4 m/s past one static obstacle
3.0 m ahead, nominal model, real corridor / target / solver code, 0.2 s
stages under a 0.1 s loop, the 1.2 m wide fixed corridor, and the A3 speed
floor + keep-outs + BLOCKED detector called as MPC_corr.control_loop calls
them. "appear" is the distance (rear axle to obstacle centre) at which the
obstacle is first published: inf = from the start, 1.4 = R011.

What it shows, and what it cannot: the NOMINAL model, perfect obstacle
positions and sizes, no detection latency or dropout. A pass here is
necessary, not sufficient -- Gazebo and the car are the real checks.

Print the table:
    python3 -m pytest test/test_blocked_closed_loop.py -s -k table
"""

import math

import pytest

import campaign_rig as cr

PLANT_R = 0.15     # R011's ~0.30 m plant
PERSON_R = 0.25    # a 0.5 m person
BIG_R = 0.50       # a 1.0 m object

CASES = [
    # (label, r, lateral offset, appear distance)
    ('plant centred, seen from start', PLANT_R, 0.0, math.inf),
    ('plant centred, seen at 2.0 m', PLANT_R, 0.0, 2.0),
    ('plant 0.3 left, seen at 1.4 m', PLANT_R, 0.3, 1.4),
    ('plant centred, seen at 1.4 m (R011)', PLANT_R, 0.0, 1.4),
    ('person centred, seen from start', PERSON_R, 0.0, math.inf),
    ('person centred, seen at 2.0 m', PERSON_R, 0.0, 2.0),
    ('person 0.3 left, seen from start', PERSON_R, 0.3, math.inf),
    ('1.0 m object centred, from start', BIG_R, 0.0, math.inf),
    ('1.0 m object centred, at 1.4 m', BIG_R, 0.0, 1.4),
    ('pebble centred, seen at 1.4 m', 0.05, 0.0, 1.4),
]


@pytest.fixture(scope='module')
def table():
    out = {}
    for label, r, off, appear in CASES:
        out[label] = cr.run(cr.Config(a3=True), [cr.Obstacle(3.0, off, r, appear)])
    return out


@pytest.fixture(scope='module')
def legacy_r011():
    """R011 on the pre-campaign controller: 0.1 s stages, the 0.87 m funnel, no A3."""
    cfg = cr.Config(a3=False, half_width_start=0.4333, half_width_end=0.7667,
                    lookahead_frac=0.5, dt=0.1)
    return cr.run(cfg, [cr.Obstacle(3.0, 0.0, PLANT_R, 1.4)])


def test_print_the_table(table, legacy_r011, capsys):
    with capsys.disabled():
        print(f'\n{"case":<36} {"outcome":<28} {"body gap":>8} {"x-track":>7}')
        rows = list(table.items()) + [('R011 on the PRE-campaign stack', legacy_r011)]
        for label, res in rows:
            if res.reached_s is not None:
                outcome = f'passed, {res.reached_s:.1f} s'
            elif res.blocked_s is not None:
                outcome = f'BLOCKED {res.blocked_reason} {res.blocked_s:.1f} s'
            else:
                outcome = 'neither'
            print(f'{label:<36} {outcome:<28} {res.min_body_gap:>+8.3f} '
                  f'{res.peak_cross_track:>7.2f}')


@pytest.mark.parametrize('label', [c[0] for c in CASES])
def test_never_a_crawl(table, label):
    """Every published speed is 0 or >= the 0.4 m/s floor."""
    assert all(v == 0.0 or v >= 0.4 - 1e-9 for v in table[label].commanded)
    assert table[label].stalled_s == 0.0


@pytest.mark.parametrize('label', [c[0] for c in CASES])
def test_never_touches_the_obstacle(table, label):
    assert table[label].min_body_gap > 0.0


@pytest.mark.parametrize('label', [
    'plant centred, seen from start', 'plant centred, seen at 2.0 m',
    'plant 0.3 left, seen at 1.4 m', 'person centred, seen from start',
    'person centred, seen at 2.0 m', 'person 0.3 left, seen from start',
    'pebble centred, seen at 1.4 m'])
def test_the_wide_corridor_passes(table, label):
    assert table[label].reached_s is not None
    assert table[label].blocked_s is None


@pytest.mark.parametrize('label', [
    'plant centred, seen at 1.4 m (R011)', '1.0 m object centred, from start',
    '1.0 m object centred, at 1.4 m'])
def test_what_cannot_be_passed_ends_blocked(table, label):
    assert table[label].blocked_s is not None
    assert table[label].blocked_reason == 'no_feasible_plan'


def test_r011_ends_blocked_within_two_seconds_of_appearing(table):
    res = table['plant centred, seen at 1.4 m (R011)']
    # The car's rear axle reaches 3.0 - 1.4 = 1.6 m from the start: find when.
    t_appear = next(i * 0.1 for i, s in enumerate(res.xs) if math.hypot(3.0 - s[0], s[1]) <= 1.4)
    assert res.blocked_s - t_appear <= 2.0


def test_r011_on_the_pre_campaign_stack_neither_passes_cleanly_nor_stops(legacy_r011):
    """The failure the fixes are for, reproduced on the nominal model.

    The old stack never decides: it either crawls below the breakaway speed
    or drives through the plant. (On the car it crawled for 33 s.)
    """
    res = legacy_r011
    crawled = any(0.0 < v < 0.4 for v in res.commanded)
    assert res.blocked_s is None
    assert crawled or res.min_body_gap < 0.0
