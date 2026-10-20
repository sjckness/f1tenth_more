"""Planned missions: storing RESULT lines, the summary, refusals of load and start."""

import json
import time

from f1tenth_ctl import cli, config, missions
import pytest

MISSION = {
    'mission_id': 'llm_a1b2c3', 'schema_version': '5.0',
    'moves': [
        {'id': 'straight_1', 'drive': {'mode': 'straight', 'speed': 0.5},
         'stop_condition': {'type': 'distance_reached', 'distance': 2.0},
         'timeout_sec': 30, 'on_timeout': 'abort'},
        {'id': 'turn_2', 'drive': {'mode': 'wall_turn', 'turn_sign': -1.0,
                                   'turn_mag_deg': 90.0, 'speed': 0.3},
         'stop_condition': {'type': 'orientation_delta', 'value': 90},
         'timeout_sec': 20, 'on_timeout': 'abort'},
        {'id': 'to_person', 'go_to_object': {'target_class': 'person', 'speed': 0.4,
                                             'acquire_timeout_sec': 5.0, 'lost_grace_sec': 1.5},
         'stop_condition': {'type': 'object_reached'}, 'timeout_sec': 40,
         'on_timeout': 'abort', 'terminal': True},
    ],
}
# Recorded the way llm_planner_node._emit_result prints them (--dry-run).
RESULT_OK = 'RESULT ' + json.dumps({
    'status': 'ok', 'stage': 'delivery', 'attempts': 1, 'mission_id': 'llm_a1b2c3',
    'schema_version': '5.0', 'n_moves': 3, 'plan_seconds': 3.1, 'unsupported': [],
    'notes': [], 'mission': MISSION, 'mission_path': '/x/llm_generated/llm_a1b2c3.json'})
RESULT_ERR = 'RESULT ' + json.dumps({
    'status': 'llm_error', 'stage': 'llm', 'error_type': 'LlamaServerUnreachableError',
    'error_message': 'llama-server not reachable', 'attempts': 0, 'llm_raw': None})


@pytest.fixture
def settings(tmp_path):
    path = tmp_path / 't.env'
    path.write_text('\n'.join(f'F1T_{n.upper()}_CMD="sleep 1"' for n in config.MANAGED)
                    + f'\nF1T_STATE_DIR={tmp_path}/state\nF1T_MONITOR=false\n')
    return config.load(str(path))


def test_summary_of_a_planned_mission():
    s = missions.summarize(MISSION)
    assert s['n_moves'] == 3 and s['max_speed_mps'] == 0.5 and s['reverse'] is False
    assert s['targets'] == ['person']
    assert [m['action'] for m in s['moves']] == [
        'straight', 'turn right 90.0 deg', 'go to the nearest person, stop at the default gap']
    # The last move ends on reaching an object: the length is not known ahead.
    assert s['distance_m'] is None and s['max_duration_s'] == 90.0
    assert missions.summarize({'moves': [MISSION['moves'][0]]})['distance_m'] == 2.0


def test_result_lines_store_their_mission_once(settings):
    lines = ['missione generata in 3.10s', 'comando> ' + RESULT_OK, RESULT_ERR]
    res = cli.planner_results(settings, lines, [0, 30, 900])
    ok, err = res
    assert ok['status'] == 'ok' and ok['stored']['mission_id'] == 'llm_a1b2c3'
    assert ok['mission'] == MISSION
    mid = ok['stored']['id']
    assert len(mid) == len('20261020T153012-abcdef') and mid[15] == '-'
    # Another RESULT line with the same plan in the same second: its own id.
    twin = cli.planner_results(settings, [RESULT_OK], [5000])[0]['stored']['id']
    assert twin != mid and twin.startswith(mid)
    assert err['status'] == 'llm_error' and 'stored' not in err
    assert err['error_message'] == 'llama-server not reachable'
    # The same RESULT line read again (same offset): the same record.
    again = cli.planner_results(settings, ['comando> ' + RESULT_OK], [30])[0]
    assert again['stored']['id'] == mid
    rec = missions.get(settings, mid)
    with open(rec['path']) as f:
        assert json.load(f) == MISSION       # exactly the planner's document
    assert mid in [r['id'] for r in missions.recent(settings)]


def test_unknown_or_odd_ids_are_not_found(settings):
    assert missions.get(settings, 'nope') is None
    assert missions.get(settings, '../state') is None


def test_state_names_and_the_countdown(settings):
    missions.set_current(settings, id='m1', path='/p/m1.json', started_at=1000.0)
    st = missions.describe_state(settings, 'LOADED', '/p/m1.json', now=1005.0)
    assert st['state'] == 'starting' and st['id'] == 'm1' and st['elapsed_s'] == 5.0
    assert missions.describe_state(settings, 'RUNNING', '/p/m1.json', now=1030)['state'] \
        == 'running'
    assert missions.describe_state(settings, 'COMPLETE', '/p/m1.json')['state'] == 'finished'
    other = missions.describe_state(settings, 'ABORTED', '/p/other.json')
    assert other['state'] == 'aborted' and other['id'] is None


def _stored(settings, age_s=0.0):
    res = cli.planner_results(settings, [RESULT_OK], [int(time.time() * 1000) % 100000])
    rec = res[0]['stored']
    if age_s:
        meta = missions.get(settings, rec['id'])
        meta.pop('mission')
        meta['stored_at'] -= age_s
        missions._write_json(f"{missions.store_dir(settings)}/{rec['id']}.json", meta)
    return rec['id']


class Args:
    def __init__(self, **kw):
        self.allow_old = False
        self.id = None
        self.__dict__.update(kw)


def test_load_refuses_an_old_plan(settings):
    mid = _stored(settings, age_s=301)
    out = cli.cmd_mission_load(settings, Args(id=mid))
    assert out['ok'] is False and 'plan it again' in out['refused']


def test_load_refuses_without_a_stack(settings, monkeypatch):
    mid = _stored(settings)
    monkeypatch.setattr(cli, '_stack_up', lambda s: False)
    out = cli.cmd_mission_load(settings, Args(id=mid))
    assert out['ok'] is False and out['refused'] == 'the stack is not running'


def test_load_passes_the_stored_file(settings, monkeypatch):
    mid = _stored(settings)
    seen = {}
    monkeypatch.setattr(cli, '_stack_up', lambda s: True)

    def fake_probe(s, mode, spec, timeout):
        seen.update(spec)
        return {'ok': True, 'ros_state': 'IDLE', 'load': {'ok': True, 'message': 'loaded'}}

    monkeypatch.setattr(cli, 'run_rosprobe', fake_probe)
    out = cli.cmd_mission_load(settings, Args(id=mid))
    assert out['ok'] and seen['op'] == 'load' and seen['path'] == missions.mission_path(
        settings, mid)
    assert missions.current(settings)['id'] == mid


@pytest.mark.parametrize('status, reason', [
    ({'ready': False, 'failing': ['scan'], 'estop_latched': False}, 'not READY: scan'),
    ({'ready': False, 'failing': ['estop_latched'], 'estop_latched': True},
     'the e-stop is latched'),
])
def test_start_refuses_when_not_ready_or_latched(settings, monkeypatch, status, reason):
    mid = _stored(settings)
    missions.set_current(settings, id=mid, path=missions.mission_path(settings, mid))
    monkeypatch.setattr(cli, 'cmd_status', lambda *a: status)
    monkeypatch.setattr(cli, 'run_rosprobe', lambda *a, **k: pytest.fail('must not start'))
    out = cli.cmd_mission_start(settings, Args(), time.monotonic())
    assert out['ok'] is False and out['refused'].startswith(reason)


def test_start_refuses_another_running_mission(settings, monkeypatch):
    mid = _stored(settings)
    missions.set_current(settings, id=mid, path=missions.mission_path(settings, mid))
    monkeypatch.setattr(cli, 'cmd_status', lambda *a: {'ready': True, 'estop_latched': False})
    monkeypatch.setattr(cli, 'run_rosprobe', lambda *a, **k: {
        'ok': False, 'error': 'another mission is running (RUNNING)'})
    out = cli.cmd_mission_start(settings, Args(), time.monotonic())
    assert out['ok'] is False and out['refused'] == 'another mission is running (RUNNING)'
    assert 'started_at' not in missions.current(settings) or \
        missions.current(settings)['started_at'] is None


def test_start_needs_the_loaded_one_and_a_fresh_plan(settings, monkeypatch):
    assert 'no mission loaded' in cli.cmd_mission_start(
        settings, Args(), time.monotonic())['refused']
    old = _stored(settings, age_s=400)
    missions.set_current(settings, id=old, path=missions.mission_path(settings, old))
    assert 'plan it again' in cli.cmd_mission_start(settings, Args(), time.monotonic())['refused']
    assert 'is not the loaded mission' in cli.cmd_mission_start(
        settings, Args(id=_stored(settings)), time.monotonic())['refused']


def test_start_ok_records_the_time(settings, monkeypatch):
    mid = _stored(settings)
    missions.set_current(settings, id=mid, path=missions.mission_path(settings, mid))
    monkeypatch.setattr(cli, 'cmd_status', lambda *a: {'ready': True, 'estop_latched': False})
    monkeypatch.setattr(cli, 'run_rosprobe', lambda *a, **k: {
        'ok': True, 'start': {'ok': True, 'message': 'starting in 15.0 s'}})
    out = cli.cmd_mission_start(settings, Args(), time.monotonic())
    assert out['ok'] and out['countdown_s'] == 15.0
    assert missions.current(settings)['started_at'] > 0
