"""mission-status through the daemon: identical to the old path, read-only, never a hang."""

import functools
import re
import threading
import time

from f1tenth_ctl import cli, config, missiond, missions, ready, rosprobe
import pytest

from test_missiond import CHECKS, Daemon, FakeEx, FakeMeter, FakeNode, GOOD, MIN

NOW = 1000.0


@pytest.fixture
def settings(tmp_path):
    env = tmp_path / 'ctl.env'
    env.write_text(MIN + f'F1T_STATE_DIR={tmp_path / "s"}\n')
    return config.load(str(env))


def stage(settings, monkeypatch, current):
    """Fix the clock of describe_state and the stored `current` mission record."""
    monkeypatch.setattr(cli.missions, 'describe_state',
                        functools.partial(missions.describe_state, now=NOW))
    if current is not None:
        missions.set_current(settings, **current)


class Probe:
    """The old path: what the one-shot probe does once discovery is done, on fake state."""

    def __init__(self, statuses):
        self.statuses = statuses
        self.calls = 0

    def __call__(self, settings, mode, spec, timeout):
        self.calls += 1
        return rosprobe.mission_core(spec, FakeEx(), self.statuses, None, None, None)


class NoServices:
    """Load and start clients that fail the test when anything touches them."""

    def __getattr__(self, name):
        raise AssertionError(f'status touched a service client ({name})')


def missiond_ops(statuses, node, measured):
    return missiond.RosOps(node, FakeEx(), statuses, NoServices(), NoServices(), None,
                           FakeMeter(measured), CHECKS,
                           {'status_topic': '/mission/status', 'window_s': 1.0},
                           core=rosprobe.mission_core, ready=ready.evaluate,
                           estop_latched=ready.estop_latched)


def make_ops(statuses, node=None, measured=GOOD):
    ops = missiond_ops(statuses, node or FakeNode(), measured)
    ops.note_status()
    return ops


def normalise(text):
    """Drop what legitimately differs between two runs: the stamp and the elapsed time."""
    return re.sub(r'"(time|elapsed_s)": [^,}]+(, )?', '', text)


def run_cli(args, capsys):
    code = cli.main(args)
    return code, capsys.readouterr().out


STATES = {
    'idle': ([(0.0, False, 'IDLE', '')], None),
    'loaded_ours': ([(0.0, False, 'LOADED', '/m.json')],
                    dict(id='r1', path='/m.json', loaded_at=NOW - 100)),
    'starting': ([(0.0, False, 'LOADED', '/m.json')],
                 dict(id='r1', path='/m.json', started_at=NOW - 2, countdown_s=10.0)),
    'running': ([(0.0, False, 'RUNNING', '/m.json')],
                dict(id='r1', path='/m.json', started_at=NOW - 30)),
    'holding': ([(0.0, False, 'HOLDING', '/m.json')],
                dict(id='r1', path='/m.json', started_at=NOW - 30)),
    'complete': ([(0.0, False, 'COMPLETE', '/m.json')],
                 dict(id='r1', path='/m.json', started_at=NOW - 60)),
    'aborted_estop': ([(0.0, True, 'ABORTED', '/m.json')], None),
    'loaded_not_ours': ([(0.0, False, 'LOADED', '/other.json')],
                        dict(id='r1', path='/m.json')),
    'several_samples': ([(0.0, False, 'IDLE', ''), (1.0, False, 'LOADED', '/m.json')], None),
}


@pytest.mark.parametrize('fmt', [[], ['--human']], ids=['json', 'human'])
@pytest.mark.parametrize('name', sorted(STATES))
def test_the_daemon_path_prints_what_the_old_path_prints(
        name, fmt, settings, monkeypatch, capsys, sock):
    statuses, current = STATES[name]
    stage(settings, monkeypatch, current)
    monkeypatch.setattr(cli, 'load', lambda _e: settings)
    monkeypatch.setattr(missiond, 'sock_path', lambda _d: sock)

    old_probe = Probe(statuses)
    monkeypatch.setattr(cli, 'run_rosprobe', old_probe)
    old_code, old_out = run_cli(['mission-status', *fmt], capsys)       # no daemon: old path
    assert old_probe.calls == 1

    daemon = Daemon(sock, make_ops(list(statuses)).execute)
    try:
        new_probe = Probe(statuses)
        monkeypatch.setattr(cli, 'run_rosprobe', new_probe)
        new_code, new_out = run_cli(['mission-status', *fmt], capsys)
    finally:
        daemon.close()
    assert new_probe.calls == 0                                         # the daemon answered
    assert new_code == old_code
    assert normalise(new_out) == normalise(old_out)


@pytest.mark.parametrize('fmt', [[], ['--human']], ids=['json', 'human'])
def test_no_status_yet_falls_back_and_prints_what_the_old_path_prints(
        fmt, settings, monkeypatch, capsys, sock):
    stage(settings, monkeypatch, None)
    monkeypatch.setattr(cli, 'load', lambda _e: settings)
    monkeypatch.setattr(missiond, 'sock_path', lambda _d: sock)
    probe = Probe([])
    monkeypatch.setattr(cli, 'run_rosprobe', probe)
    old_code, old_out = run_cli(['mission-status', *fmt], capsys)
    daemon = Daemon(sock, make_ops([]).execute)                          # a daemon with no view
    try:
        new_code, new_out = run_cli(['mission-status', *fmt], capsys)
    finally:
        daemon.close()
    assert probe.calls == 2                                              # both ran the old path
    assert (new_code, normalise(new_out)) == (old_code, normalise(old_out))
    assert old_code == 1 and 'no /mission/status' in old_out


# ---- read-only

def test_status_never_touches_a_service_or_the_ready_meter():
    class Meter:
        def measure(self, _w):
            raise AssertionError('status measured READY')

    ops = missiond_ops([(0.0, False, 'RUNNING', '/m.json')], FakeNode(), GOOD)
    ops.meter = Meter()
    ops.note_status()
    out = ops.execute({'op': 'status', 'spec': {}})
    assert out == {'result': {'ros_state': 'RUNNING', 'json_path': '/m.json',
                              'emergency_stop_active': False, 'ok': True}}


def test_a_status_does_not_change_the_state_it_reads():
    statuses = [(0.0, False, 'LOADED', '/m.json')]
    ops = make_ops(statuses)
    before = list(statuses)
    for _ in range(5):
        ops.execute({'op': 'status', 'spec': {}})
    assert statuses == before


def test_a_stale_view_makes_status_fall_back_too():
    node = FakeNode()
    ops = make_ops([(0.0, False, 'LOADED', '/m.json')], node=node)
    node.pubs = []
    assert 'no publisher' in ops.execute({'op': 'status', 'spec': {}})['fallback']
    ops2 = make_ops([])
    assert 'no /mission/status' in ops2.execute({'op': 'status', 'spec': {}})['fallback']


# ---- while a load or a start is in flight, and a dead daemon

def test_status_behind_a_slow_load_takes_the_old_path_fast_and_is_never_run_later(
        settings, monkeypatch, sock):
    ran = []

    def execute(req):
        ran.append(req['op'])
        if req['op'] == 'load':
            time.sleep(1.0)
        return {'result': {'ok': True, 'ros_state': 'IDLE', 'json_path': '', 'op': req['op']}}

    monkeypatch.setattr(missiond, 'sock_path', lambda _d: sock)
    monkeypatch.setattr(missiond, 'STATUS_WAIT_S', 0.3)
    daemon = Daemon(sock, execute)
    probe = Probe([(0.0, False, 'IDLE', '')])
    monkeypatch.setattr(cli, 'run_rosprobe', probe)
    try:
        loader = threading.Thread(
            target=lambda: missiond.request(sock, 'load', {'op': 'load'}, 5))
        loader.start()
        time.sleep(0.15)
        t0 = time.monotonic()
        res = cli.run_mission_op(settings, {'op': 'status'}, 15.0)
        waited = time.monotonic() - t0
        loader.join()
        time.sleep(0.2)
    finally:
        daemon.close()
    assert waited < 0.8 and probe.calls == 1                 # not a hang: the old path answered
    assert res['ros_state'] == 'IDLE'
    assert ran == ['load']                                   # the abandoned status never ran


def test_status_requests_between_a_load_and_a_start_change_nothing(sock):
    ran = []
    daemon = Daemon(sock, lambda r: ran.append(r['op']) or {'result': {'ok': True}})
    try:
        for op in ('status', 'load', 'status', 'start', 'status'):
            kind, _ = missiond.request(sock, op, {'op': op}, 5, read_only=(op == 'status'))
            assert kind == 'result'
    finally:
        daemon.close()
    assert ran == ['status', 'load', 'status', 'start', 'status']


def test_a_read_only_request_never_reports_an_error_only_a_fallback(sock):
    Daemon(sock, lambda r: {'result': {'ok': True}}).close()
    assert missiond.request(sock, 'status', {}, 1, read_only=True)[0] == 'fallback'


def test_the_daemon_dying_between_two_calls_gives_the_old_path_and_the_same_answer(
        settings, monkeypatch, sock):
    monkeypatch.setattr(missiond, 'sock_path', lambda _d: sock)
    statuses = [(0.0, False, 'LOADED', '/m.json')]
    probe = Probe(statuses)
    monkeypatch.setattr(cli, 'run_rosprobe', probe)
    daemon = Daemon(sock, make_ops(list(statuses)).execute)
    via_daemon = cli.run_mission_op(settings, {'op': 'status'}, 15.0)
    assert probe.calls == 0
    daemon.stop = True
    daemon.server._closing = True
    daemon.server._sock.close()                              # killed: the file stays behind
    via_probe = cli.run_mission_op(settings, {'op': 'status'}, 15.0)
    assert probe.calls == 1 and via_probe == via_daemon
