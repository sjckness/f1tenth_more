"""The dodge family D(d, seed): ramp, generator, motion formula, feasibility oracle. No ROS.

    python3 -m pytest scripts/sim/isaac_tuning/test -q
"""
import importlib.util
import math
import os
import sys

import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import common  # noqa: E402
import dodge  # noqa: E402

ISAAC_MOTION = os.path.expanduser('~/isaac_ws/sim/motion.py')


def test_ramp_follows_the_spec():
    assert dodge.ramp(0) == {'speed': 0.2, 'diag_frac': 0.0, 'n_levels': 2}
    assert dodge.ramp(3) == {'speed': 0.5, 'diag_frac': 0.3, 'n_levels': 3}
    assert dodge.ramp(13)['speed'] == 1.5 and dodge.ramp(20)['speed'] == 1.5
    assert dodge.ramp(20)['diag_frac'] == 0.5
    assert dodge.ramp(20)['n_levels'] == dodge.MAX_LEVELS


def test_pingpong_goes_back_and_forth():
    a, b = (0.0, 0.0), (2.0, 0.0)
    assert dodge.pingpong_point(a, b, 0.5) == pytest.approx([0.5, 0.0])
    assert dodge.pingpong_point(a, b, 2.5) == pytest.approx([1.5, 0.0])   # bounced at b
    assert dodge.pingpong_point(a, b, 4.5) == pytest.approx([0.5, 0.0])   # one full cycle
    assert dodge.pingpong_point(a, b, -0.5) == pytest.approx([0.5, 0.0])


@pytest.mark.skipif(not os.path.exists(ISAAC_MOTION), reason='~/isaac_ws not checked out')
def test_harness_and_isaac_use_the_same_motion_formula():
    spec = importlib.util.spec_from_file_location('isaac_motion', ISAAC_MOTION)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    pp = {'type': 'pingpong', 'a': [1.0, 0.5], 'b': [1.6, 5.2], 'speed': 0.9, 's0': 1.3, 'delay_s': 0.2}
    lin = {'v': [0.0, 0.4], 'delay_s': 0.0, 'duration_s': 6.0}
    wp = {'type': 'waypoints', 'points': [[3.0, 0.6], [3.0, 3.0], [4.0, 3.0]], 'speed': 1.0,
          'delay_s': 0.5}
    for tau in (0.0, 0.1, 3.3, 7.9, 20.0):
        assert mod.motion_position(pp, [0, 0], tau) == pytest.approx(common.motion_position(pp, [0, 0], tau))
        assert mod.motion_position(lin, [2, 1], tau) == pytest.approx(common.motion_position(lin, [2, 1], tau))
        assert mod.motion_position(wp, None, tau) == pytest.approx(common.motion_position(wp, None, tau))


def test_generated_scenario_is_deterministic_feasible_and_inside_the_room():
    s1 = dodge.generate(4, 1)
    s2 = dodge.generate(4, 1)
    assert s1 == s2
    assert s1['layout'] == 'dodge' and s1['ramp'] == dodge.ramp(4)
    assert len(s1['levels']) == dodge.ramp(4)['n_levels']
    assert dodge.feasible({'obstacles': dodge.oracle_obstacles(s1),
                           'distance_m': s1['mission']['distance_m']})
    ids = [ob['id'] for ob in s1['obstacles']]
    assert len(ids) == len(set(ids))
    inst = common.scenario_instance(s1, 0)
    for ob in inst['obstacle_info']:
        r = ob['radius_m']
        m = ob['motion']
        for t in [0.1 * i for i in range(300)]:
            x, y = common.motion_position(m, None, t)
            assert dodge.ROOM_Y[0] + r <= y + 1e-6 and y - 1e-6 <= dodge.ROOM_Y[1] - r
            assert abs(x - (inst['nominal_start']['x'] + dodge.LEVEL_AHEAD_M[ob['level']])) \
                <= dodge.X_BAND_M + 1e-6
        # placed where the motion starts
        assert common.motion_position(m, None, 0.0) == pytest.approx([ob['x'], ob['y']], abs=1e-3)


def test_a_pair_moves_rigidly_and_never_overlaps():
    for seed in range(40):
        scn = dodge.candidate(6, seed)
        for lv in scn['levels']:
            if lv['n'] != 2:
                continue
            obs = [ob for ob in scn['obstacles'] if ob['level'] == lv['level']]
            for t in [0.2 * i for i in range(100)]:
                p = [dodge.pingpong_point(o['motion']['a'], o['motion']['b'],
                                          o['motion']['s0'] + o['motion']['speed'] * t) for o in obs]
                dist = math.hypot(p[0][0] - p[1][0], p[0][1] - p[1][1])
                assert dist == pytest.approx(lv['spacing_m'], abs=1e-3)   # endpoints rounded to 0.1 mm
            return
    pytest.skip('no pair drawn')


def test_oracle_rejects_a_wall_of_cylinders_and_accepts_an_empty_room():
    assert dodge.feasible({'obstacles': [], 'distance_m': 3.5})
    # static cylinders every 0.6 m across the whole width at 1.5 m: no gap for the car
    wall = [{'diameter': 0.5, 'motion': {'type': 'pingpong', 'a': [1.5, y], 'b': [1.5, y], 'speed': 0.0,
                                          's0': 0.0}} for y in [-2.7 + 0.6 * i for i in range(10)]]
    assert not dodge.feasible({'obstacles': wall, 'distance_m': 3.5})
    # one static cylinder on the line: go round it
    one = [{'diameter': 0.5, 'motion': {'type': 'pingpong', 'a': [1.5, 0.0], 'b': [1.5, 0.0],
                                         'speed': 0.0, 's0': 0.0}}]
    ok, path = dodge.feasible({'obstacles': one, 'distance_m': 3.5}, return_path=True)
    assert ok and path[-1][1] >= 3.5
    need = 0.25 + dodge.CAR_RADIUS_M + dodge.SHIPPED_MARGIN_M
    for _, x, y, psi in path:
        cx, cy = x + 0.5 * dodge.WHEELBASE_M * math.cos(psi), y + 0.5 * dodge.WHEELBASE_M * math.sin(psi)
        assert math.hypot(cx - 1.5, cy) >= need - 1e-9


def test_dodge_ids_round_trip():
    assert dodge.parse_id(dodge.scenario_id(7, 2)) == (7, 2)
    assert common.is_dodge('D07_s2') and not common.is_dodge('S1')
