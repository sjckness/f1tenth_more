"""Unit tests of the tuning harness's pure parts: scenarios, the space, overlays, metrics, score.

    python3 -m pytest scripts/sim/isaac_tuning/test -q      (no ROS needed)
"""
import csv
import json
import math
import os
import sys

import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import common  # noqa: E402
import metrics  # noqa: E402


# ---------------------------------------------------------------- scenarios and space
def test_every_scenario_loads_and_places_its_obstacles_ahead():
    for sid in common.SCENARIO_IDS:
        scn = common.load_scenario(sid)
        inst = common.scenario_instance(scn, 0)
        for ob in inst['obstacle_info']:
            assert ob['x'] == pytest.approx(scn['car_start']['x'] + ob['ahead_m'])
    assert common.scenario_instance(common.load_scenario('S5'), 0)['obstacles'] == {}


def test_jitter_is_seeded_and_bounded():
    scn = common.load_scenario('S1')
    a, b = common.scenario_instance(scn, 3), common.scenario_instance(scn, 3)
    assert a['car'] == b['car']
    assert common.scenario_instance(scn, 4)['car'] != a['car']
    for rep in range(20):
        car = common.scenario_instance(scn, rep)['car']
        assert abs(car['x'] - 0.5) <= 0.05 and abs(car['y'] - 3.0) <= 0.05
        assert abs(car['yaw_deg']) <= 3.0


def test_s3_alternates_the_obstacle_side():
    scn = common.load_scenario('S3')
    ys = [common.scenario_instance(scn, rep)['obstacles']['cyl050'][1] for rep in range(4)]
    assert ys[0] == pytest.approx(3.15) and ys[1] == pytest.approx(2.85)
    assert ys[2] == pytest.approx(3.15) and ys[3] == pytest.approx(2.85)


def test_the_space_rejects_unknown_keys_and_out_of_range_values():
    space = common.load_space()
    assert common.check_params({}, space) == []
    assert common.check_params({'mpc_w_obs': 100.0}, space) == []
    assert common.check_params({'mpc_w_obs': 1e6}, space)
    assert common.check_params({'obstacle_target_shift_m': 0.3}, space)
    assert common.check_params({'mpc_w_term': 'high'}, space)


def test_the_overlays_carry_every_parameter_by_its_route(tmp_path):
    space = common.load_space()
    overlay, extra = common.write_overlays({'mpc_w_obs': 300.0, 'corr_half_width_start_m': 0.7},
                                           space, str(tmp_path))
    o, e = common.load_yaml(overlay), common.load_yaml(extra)
    nav = e['component_supervisor_node']['ros__parameters']['sim_component_args']['navigation']
    assert nav['mpc_w_obs'] == 300.0 and nav['mpc_w_term'] == 9.0
    assert nav['cpu_affinity'] == '6,7'                       # linus's host overlay kept
    assert o['mpc_corr']['ros__parameters']['corr_half_width_start_m'] == 0.7
    assert o['semantic_layer_node']['ros__parameters']['unknown_objects_mode'] == 'active'
    assert 'reset_manager' in o                                # isaac.yaml kept
    exp = common.expected_node_params({'mpc_w_obs': 300.0}, space)
    assert exp['mpc_corr']['mpc_w_obs'] == 300.0
    assert exp['semantic_layer_node']['unknown_objects_mode'] == 'active'


# ---------------------------------------------------------------- metrics
def test_footprint_clearance_is_signed_distance_to_the_circle():
    # car at the origin heading +x, obstacle centre 1.0 m ahead of base_link, r 0.25:
    # front bumper at 0.405 -> clearance 1.0 - 0.405 - 0.25
    assert metrics.footprint_circle_clearance(0, 0, 0, 1.0, 0.0, 0.25) == pytest.approx(0.345)
    assert metrics.footprint_circle_clearance(0, 0, 0, 0.2, 0.0, 0.25) < 0
    # beside the car: half-width 0.136
    assert metrics.footprint_circle_clearance(0, 0, 0, 0.0, 0.5, 0.25) == pytest.approx(0.114)


def test_sign_changes_ignore_the_deadband():
    assert metrics.sign_changes([0.0, 0.1, 0.2, 0.01, -0.01, 0.1]) == 0
    assert metrics.sign_changes([0.1, -0.1, 0.1]) == 2


def _write(path, header, rows):
    with open(path, 'w', newline='', encoding='utf-8') as fh:
        w = csv.writer(fh)
        w.writerow(header)
        w.writerows(rows)


def _run_dir(tmp_path, scenario='S1', swerve=0.9, complete=True, hard_stop=False, end_t=12.0):
    """A synthetic run: straight 4 m at 0.5 m/s, optional sideways swerve around x 1.5-3."""
    d = tmp_path / scenario
    d.mkdir()
    inst = common.scenario_instance(common.load_scenario(scenario), 0)
    inst['car'] = {'x': 0.5, 'y': 3.0, 'yaw_deg': 0.0}
    gt, drive = [], []
    t_run = 2.0
    for k in range(int((end_t - t_run) / 0.02) + 1):
        t = t_run + k * 0.02
        s = min(4.0, 0.5 * (t - t_run))
        y = 3.0 - swerve * math.sin(math.pi * min(s / 3.0, 1.0)) ** 2
        gt.append([t, 0.5 + s, y, 0.0, 0.5 if s < 4.0 else 0.0, 0.0])
    for k in range(int((end_t - t_run) / 0.1)):
        t = t_run + k * 0.1
        s = 0.5 * (t - t_run)
        drive.append([t, 0.5, (-0.2 if 0.1 < s < 1.5 else 0.15 if 1.5 <= s < 3.0 else 0.0)
                      if swerve else 0.0])
    _write(d / 'gt.csv', ['t', 'x', 'y', 'yaw', 'v', 'w'], gt)
    _write(d / 'drive.csv', ['t', 'speed', 'steer'], drive)
    _write(d / 'mission.csv', ['t', 'state'],
           [[t_run, 'RUNNING'], [end_t, 'COMPLETE' if complete else 'ABORTED']])
    _write(d / 'solver.csv', ['t', 'success', 'status', 'n_obstacles', 'cost', 'solve_dt'],
           [[t_run + 0.1 * k, 1, 1, 1, 0.0, 0.0] for k in range(50)])
    _write(d / 'hard_stop.csv', ['t', 'engaged', 'nearest_range_m'],
           [[t_run + 1.0, 1 if hard_stop else 0, 0.2]])
    for name, header in (('status.csv', ['t', 'status', 'solve_time_ms', 'success']),
                         ('bt.csv', ['t', 'active_lane', 'emergency_trip', 'safety_stop_active',
                                     'stop_source']),
                         ('mux.csv', ['t', 'speed', 'steer', 'frame']),
                         ('obstacles.csv', ['t', 'frame', 'n', 'x', 'y', 'r']),
                         ('cpu.csv', ['t_wall', 't', 'mpc_cpu_s'])):
        _write(d / name, header, [])
    (d / 'events.jsonl').write_text('')
    (d / 'meta.json').write_text(json.dumps({'instance': inst, 'exit': {'code': 0},
                                             't_running': t_run, 't_end': end_t}))
    return str(d)


def test_a_clean_swerve_is_a_success_with_a_margin(tmp_path):
    r = metrics.compute(_run_dir(tmp_path, swerve=0.9))
    assert r['outcome'] == 'success'
    assert r['obstacles'][0]['r_safe_ref_m'] == pytest.approx(0.25 + 0.20 + 0.12)
    assert r['min_margin_m'] > 0
    assert r['steer_sign_changes'] == 1
    assert 0 < r['score'] <= 100


def test_driving_through_the_obstacle_is_a_collision_and_scores_zero(tmp_path):
    r = metrics.compute(_run_dir(tmp_path, swerve=0.0))
    assert r['outcome'] == 'collision'
    assert r['score'] == 0.0


def test_a_hard_stop_is_an_estop(tmp_path):
    r = metrics.compute(_run_dir(tmp_path, swerve=0.9, hard_stop=True))
    assert r['outcome'] == 'estop' and r['score'] == 0.0


def test_no_obstacle_scores_tracking(tmp_path):
    r = metrics.compute(_run_dir(tmp_path, scenario='S5', swerve=0.0))
    assert r['outcome'] == 'success'
    assert r['min_margin_m'] is None
    # 4 m at 0.5 m/s is 8 s nominal; the synthetic mission ends after 10 s
    assert r['score_terms'] == {'margin': 1.0, 'time': pytest.approx(0.8, abs=0.01),
                                'osc': 1.0, 'track': 1.0}


def test_aggregate_penalises_inconsistency():
    runs = [{'valid': True, 'outcome': 'success', 'score': s, 'min_margin_m': 0.01,
             'max_lateral_error_m': 0.9, 'steer_sign_changes': 1, 'time_to_goal_s': 9.0}
            for s in (90.0, 90.0)]
    steady = metrics.aggregate(runs)
    runs[1]['score'] = 50.0
    shaky = metrics.aggregate(runs)
    assert steady['score_agg'] == pytest.approx(90.0)
    assert shaky['score_agg'] < shaky['score_mean'] < steady['score_agg']
