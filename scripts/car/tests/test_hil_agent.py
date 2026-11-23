"""Tests of scripts/car/orin/hil_agent.py that need no ROS: the profile and the run judge.

    python3 -m pytest scripts/car/tests/test_hil_agent.py
"""
import csv
import importlib.util
import json
import math
import os
import re
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[3]
spec = importlib.util.spec_from_file_location('hil_agent', ROOT / 'scripts/car/orin/hil_agent.py')
agent = importlib.util.module_from_spec(spec)
spec.loader.exec_module(agent)

# the door_left preset: start (16.464, -12.604) yaw 180, doorway 1.25 m at (11.464, -14.604)
POSE = {'idx': 16, 'start': {'x': 16.464, 'y': -12.604, 'z': 0.02, 'yaw_deg': -180.0},
        'door': {'x': 11.464, 'y': -14.604, 'width_m': 1.25, 'passage_axis': 'y'}}


def _strip_comments(xml):
    return re.sub(r'\s+', ' ', re.sub(r'<!--.*?-->', '', xml, flags=re.S)).strip()


def test_profile_matches_the_sim_side_template():
    win = (ROOT / 'scripts/sim/win/dds/hil.xml.in').read_text()
    win = win.replace('@ASYNC_WRITER@', '')
    assert _strip_comments(agent.PROFILE_TEMPLATE) == _strip_comments(win)


def test_profile_renders_and_parses():
    import xml.etree.ElementTree as ET
    text = agent.render_profile('192.168.200.11', 16)
    root = ET.fromstring(text)
    assert '@' not in text
    assert '192.168.200.11' in text and str(16 * 1024 * 1024) in text
    assert 'flow_controller' not in text      # Jazzy rejects it, and then ignores the whole profile
    ns = {'p': 'http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles'}
    assert root.find('.//p:leaseDuration/p:sec', ns).text == '5'


def _gt(points):
    return [{'t_sim': f'{t:.3f}', 'x': f'{x:.4f}', 'y': f'{y:.4f}', 'yaw': f'{yaw:.5f}', 'v': f'{v:.3f}'}
            for t, x, y, yaw, v in points]


def _result(final_state, outcomes, t0=10.0, t1=60.0, status='ended'):
    return {'run_id': 'p16_corridor_rrt_r1', 'mode': 'corridor_rrt', 'rep': 1, 'pose': POSE,
            'status': status, 'final_state': final_state, 'outcomes': outcomes,
            'start': {'t_sim': t0}, 'end': {'t_sim': t1}}


def _path_into_door():
    """4 m along -x, a 1 m-radius left turn onto the door axis, then down it 0.3 m past the line."""
    pts, t = [], 10.0
    for i in range(41):
        pts.append((t, 16.464 - 0.1 * i, -12.604, math.pi, 0.4))
        t += 0.25
    cx, cy = 12.464, -13.604                    # turn centre: ends at (11.464, -13.604) facing -y
    for k in range(1, 21):
        a = math.pi / 2 + (math.pi / 2) * k / 20
        pts.append((t, cx + math.cos(a), cy + math.sin(a), math.pi + (math.pi / 2) * k / 20, 0.3))
        t += 0.25
    for k in range(1, 14):
        pts.append((t, 11.464, -13.604 - 0.1 * k, -math.pi / 2, 0.2))
        t += 0.25
    return pts


def test_judge_success_through_the_door():
    pts = _path_into_door()
    r = agent.judge_run(_result('COMPLETE', [
        {'move_id': 'move_0_straight_5m', 'stop_reason': 'stop_condition:distance_reached', 'outcome': ''},
        {'move_id': 'move_1_enter_left_door', 'stop_reason': 'stop_condition:door_entered', 'outcome': 'entered'}],
        t1=pts[-1][0]), _gt(pts), [])
    assert r['crossed_door'] is True
    assert r['mission_complete'] is True
    assert r['success'] is True and r['failure'] == 'ok'
    assert r['max_progress_along_start_m'] == pytest.approx(5.0, abs=0.2)


def test_judge_complete_but_wrong_place_is_not_success():
    pts = [(10.0 + 0.25 * i, 16.464 - 0.1 * i, -12.604, math.pi, 0.4) for i in range(60)]
    r = agent.judge_run(_result('COMPLETE', []), _gt(pts), [])
    assert r['mission_complete'] and not r['crossed_door'] and not r['gt_in_door']
    assert not r['success'] and r['failure'] == 'complete_not_in_door'


def test_judge_door_not_found():
    pts = [(10.0 + 0.25 * i, 16.464 - 0.1 * i, -12.604, math.pi, 0.4) for i in range(51)]
    r = agent.judge_run(_result('ABORTED', [
        {'move_id': 'move_0_straight_5m', 'stop_reason': 'stop_condition:distance_reached', 'outcome': ''},
        {'move_id': 'move_1_enter_left_door', 'stop_reason': 'door_not_found', 'outcome': 'door_not_found'}]),
        _gt(pts), [])
    assert not r['success'] and r['failure'] == 'door:door_not_found'


def test_judge_marked_run():
    res = {'run_id': 'p01_corridor_mpc_r1', 'mode': 'corridor_mpc', 'rep': 1, 'pose': POSE,
           'status': 'stack_not_ready', 'final_state': None, 'outcomes': []}
    r = agent.judge_run(res, [], [])
    assert not r['success'] and r['failure'] == 'stack_not_ready'


def test_analyze_writes_tables_and_report(tmp_path):
    sess = tmp_path / '20261123_220000_office_door_left_5m'
    (sess / 'runs').mkdir(parents=True)
    json.dump({'modes': ['corridor_mpc', 'corridor_rrt'], 'reps': 1, 'poses': [POSE],
               'git_branch': 'hil-office-batch', 'git_commit': 'abc'}, open(sess / 'session.json', 'w'))
    pts = _path_into_door()
    for mode, ok in (('corridor_mpc', True), ('corridor_rrt', False)):
        d = sess / 'runs' / f'p16_{mode}_r1'
        d.mkdir()
        res = _result('COMPLETE' if ok else 'ABORTED', [], t1=pts[-1][0])
        res.update({'run_id': d.name, 'mode': mode})
        json.dump(res, open(d / 'result.json', 'w'))
        with open(d / 'gt.csv', 'w', newline='') as f:
            w = csv.DictWriter(f, fieldnames=['t_sim', 'x', 'y', 'yaw', 'v'])
            w.writeheader()
            w.writerows(_gt(pts if ok else pts[:40]))
    class A:
        session = str(sess)
    assert agent.cmd_analyze(A) == 0
    rows = list(csv.DictReader(open(sess / 'analysis' / 'summary_by_mode.csv')))
    assert [(r['mode'], r['success']) for r in rows] == [('corridor_mpc', '1'), ('corridor_rrt', '0')]
    rep = (sess / 'analysis' / 'report.md').read_text()
    assert rep.startswith('---\nid: 2026-11-23_office-door-batch')
    assert '| corridor_mpc | 1 | 1 (100.0 %)' in rep
    assert os.path.exists(sess / 'analysis' / 'by_pose.csv')
