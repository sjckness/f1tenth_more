"""planner-log --wait: the same answer, as soon as the RESULT line is there."""

import argparse
import json
import os

from f1tenth_ctl import cli, config
import pytest

MIN = '\n'.join(f'F1T_{n.upper()}_CMD="sleep 1"' for n in config.MANAGED) + '\n'
RESULT = 'RESULT ' + json.dumps({'status': 'ok', 'stage': 'translator', 'attempts': 1,
                                 'mission_id': 'llm_x', 'plan_seconds': 0.5,
                                 'mission': {'mission_id': 'llm_x', 'moves': []}})


@pytest.fixture
def settings(tmp_path):
    env = tmp_path / 'ctl.env'
    env.write_text(MIN + f'F1T_STATE_DIR={tmp_path / "state"}\n')
    s = config.load(str(env))
    os.makedirs(s.log_dir)
    open(s.log_path('planner'), 'w').close()
    return s


class Clock:
    """A fake clock whose sleep() advances it and plays a script of log appends."""

    def __init__(self, log_path, script=()):
        self.t = 0.0
        self.path = log_path
        self.script = sorted(script)        # (time, text) appended once the clock passes it
        self.sleeps = []

    def clock(self):
        return self.t

    def sleep(self, s):
        self.sleeps.append(s)
        self.t += s
        while self.script and self.script[0][0] <= self.t:
            with open(self.path, 'a') as fh:
                fh.write(self.script.pop(0)[1])


def _alive(monkeypatch, value=True):
    monkeypatch.setattr(cli.units, 'is_running', lambda *_a, **_k: value)


def test_returns_at_once_when_the_result_is_already_there(settings, monkeypatch):
    path = settings.log_path('planner')
    open(path, 'a').write('missione generata\n' + RESULT + '\n')
    _alive(monkeypatch)
    fake = Clock(path)
    res = cli.wait_for_result(settings, 0, 65536, 8.0, sleep=fake.sleep, clock=fake.clock)
    assert fake.sleeps == [] and res['lines'][-1].startswith('RESULT ')


def test_waits_for_the_result_and_returns_it_with_what_came_before(settings, monkeypatch):
    path = settings.log_path('planner')
    open(path, 'a').write('old line\n')
    since = os.path.getsize(path)
    fake = Clock(path, [(0.10, 'missione generata in 0.5s\n'), (0.40, 'JSON\n'),
                        (0.60, RESULT + '\n')])
    _alive(monkeypatch)
    res = cli.wait_for_result(settings, since, 65536, 8.0, sleep=fake.sleep, clock=fake.clock)
    assert res['lines'] == ['missione generata in 0.5s', 'JSON', RESULT]
    assert res['cursor'] == os.path.getsize(path)
    assert 0.6 <= fake.t < 0.7                       # no sleeping past the line


def test_a_result_line_with_the_planner_prompt_glued_counts(settings, monkeypatch):
    path = settings.log_path('planner')
    fake = Clock(path, [(0.2, 'comando> comando> ' + RESULT + '\n')])
    _alive(monkeypatch)
    res = cli.wait_for_result(settings, 0, 65536, 8.0, sleep=fake.sleep, clock=fake.clock)
    assert cli.is_result_line(res['lines'][0])


def test_a_half_written_line_is_not_a_result(settings, monkeypatch):
    path = settings.log_path('planner')
    fake = Clock(path, [(0.1, RESULT[:20]), (0.5, RESULT[20:] + '\n')])
    _alive(monkeypatch)
    res = cli.wait_for_result(settings, 0, 65536, 8.0, sleep=fake.sleep, clock=fake.clock)
    assert res['lines'] == [RESULT] and fake.t >= 0.5


def test_gives_up_after_the_wait_with_the_partial_answer(settings, monkeypatch):
    path = settings.log_path('planner')
    fake = Clock(path, [(0.3, 'still thinking\n')])
    _alive(monkeypatch)
    res = cli.wait_for_result(settings, 0, 65536, 2.0, sleep=fake.sleep, clock=fake.clock)
    assert res['lines'] == ['still thinking'] and 2.0 <= fake.t <= 2.05


def test_polls_the_size_every_50_ms_at_most(settings, monkeypatch):
    fake = Clock(settings.log_path('planner'))
    _alive(monkeypatch)
    cli.wait_for_result(settings, 0, 65536, 1.0, sleep=fake.sleep, clock=fake.clock)
    assert max(fake.sleeps) <= cli.WAIT_POLL_S + 1e-9 and len(fake.sleeps) >= 19


def test_the_log_is_read_only_when_it_grew(settings, monkeypatch):
    fake = Clock(settings.log_path('planner'), [(0.5, 'x\n')])
    _alive(monkeypatch)
    reads = []
    real = cli.units.read_log_since
    monkeypatch.setattr(cli.units, 'read_log_since',
                        lambda *a, **k: reads.append(1) or real(*a, **k))
    cli.wait_for_result(settings, 0, 65536, 1.0, sleep=fake.sleep, clock=fake.clock)
    assert len(reads) == 2                           # the first look, and after the append


def test_no_cursor_means_no_waiting(settings, monkeypatch):
    fake = Clock(settings.log_path('planner'))
    _alive(monkeypatch)
    res = cli.wait_for_result(settings, None, 65536, 8.0, sleep=fake.sleep, clock=fake.clock)
    assert fake.sleeps == [] and res['lines'] == []


def test_zero_wait_is_todays_read(settings, monkeypatch):
    path = settings.log_path('planner')
    open(path, 'a').write('a\nb\n')
    fake = Clock(path)
    res = cli.wait_for_result(settings, 0, 65536, 0.0, sleep=fake.sleep, clock=fake.clock)
    assert fake.sleeps == []
    assert res == cli.units.read_log_since(path, 0, 65536)


def test_a_dead_planner_ends_the_wait(settings, monkeypatch):
    fake = Clock(settings.log_path('planner'))
    _alive(monkeypatch, False)
    res = cli.wait_for_result(settings, 0, 65536, 30.0, sleep=fake.sleep, clock=fake.clock)
    assert res['lines'] == [] and fake.t < cli.WAIT_ALIVE_S + 0.2


def test_a_chunk_cut_by_max_bytes_returns_for_the_caller_to_continue(settings, monkeypatch):
    path = settings.log_path('planner')
    open(path, 'a').write(('x' * 40 + '\n') * 5)
    fake = Clock(path)
    _alive(monkeypatch)
    res = cli.wait_for_result(settings, 0, 100, 30.0, sleep=fake.sleep, clock=fake.clock)
    assert res['truncated'] and fake.sleeps == []


def _args(**kw):
    ns = dict(since=0, max_bytes=65536, wait=0.0)
    ns.update(kw)
    return argparse.Namespace(**ns)


def _plan_only(monkeypatch):
    monkeypatch.setattr(cli.units, 'is_running', lambda *_a, **_k: True)
    monkeypatch.setattr(cli, 'planner_plan_only', lambda _s: True)


def test_the_answer_with_and_without_wait_is_the_same(settings, monkeypatch):
    path = settings.log_path('planner')
    open(path, 'a').write('missione\n' + RESULT + '\n')
    _plan_only(monkeypatch)
    plain = cli.cmd_planner_log(settings, _args())
    waited = cli.cmd_planner_log(settings, _args(wait=5.0))
    plain.pop('stored', None)
    assert plain['results'][0]['stored']['id'] == waited['results'][0]['stored']['id']
    for answer in (plain, waited):
        for entry in answer['results']:
            entry['stored'] = {k: v for k, v in entry['stored'].items()
                               if k not in ('stored_at',)}
    assert plain == waited
    assert waited['results'][0]['mission'] == {'mission_id': 'llm_x', 'moves': []}


def test_an_args_object_without_wait_still_works(settings, monkeypatch):
    """Older callers build the arguments without the new field."""
    _plan_only(monkeypatch)
    ns = argparse.Namespace(since=0, max_bytes=65536)
    assert cli.cmd_planner_log(settings, ns)['ok'] is True


def test_the_flag_is_accepted_and_defaults_to_no_wait():
    parser = cli.build_parser()
    assert parser.parse_args(['planner-log', '--since', '5']).wait == 0.0
    assert parser.parse_args(['planner-log', '--since', '5', '--wait', '8']).wait == 8.0
