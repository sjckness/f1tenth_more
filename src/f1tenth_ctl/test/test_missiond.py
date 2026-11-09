"""The mission daemon: protocol, fallbacks, serialization, READY re-check, e-stop independence."""

import argparse
import json
import os
import shutil
import socket
import stat
import tempfile
import threading
import time

from f1tenth_ctl import cli, config, missiond, monitor, ready
import pytest

MIN = '\n'.join(f'F1T_{n.upper()}_CMD="sleep 1"' for n in config.MANAGED) + '\n'


@pytest.fixture
def sock():
    """Yield a short socket path (unix paths are limited to ~100 characters)."""
    d = tempfile.mkdtemp(prefix='md')
    yield os.path.join(d, 'm.sock')
    shutil.rmtree(d, ignore_errors=True)


class Daemon:
    """A Server driven by a thread that plays the monitor's loop, with a fake executor."""

    def __init__(self, path, execute, clock=time.monotonic, loop=True):
        self.server = missiond.Server(path, execute, clock=clock)
        assert self.server.start()
        self.stop = False
        self.thread = None
        if loop:
            self.thread = threading.Thread(target=self._loop, daemon=True)
            self.thread.start()

    def _loop(self):
        while not self.stop:
            self.server.poll()
            time.sleep(0.005)

    def close(self):
        self.stop = True
        if self.thread:
            self.thread.join(timeout=2)
        self.server.close()


@pytest.fixture
def daemon(sock):
    made = []

    def make(execute, **kw):
        d = Daemon(sock, execute, **kw)
        made.append(d)
        return d

    yield make
    for d in made:
        d.close()


def ok_exec(req):
    return {'result': {'ok': True, 'op': req['op'], 'echo': req['spec'].get('path')}}


# ---- the protocol

def test_a_load_and_a_start_round_trip(daemon, sock):
    daemon(ok_exec)
    kind, res = missiond.request(sock, 'load', {'op': 'load', 'path': '/m.json'}, 5)
    assert kind == 'result' and res == {'ok': True, 'op': 'load', 'echo': '/m.json'}
    kind, res = missiond.request(sock, 'start', {'op': 'start'}, 5)
    assert kind == 'result' and res['op'] == 'start'


def test_the_socket_is_private(daemon, sock):
    daemon(ok_exec)
    assert stat.S_IMODE(os.stat(sock).st_mode) == 0o600
    assert os.stat(sock).st_uid == os.getuid()


def test_a_second_server_does_not_take_a_live_socket(daemon, sock):
    daemon(ok_exec)
    other = missiond.Server(sock, ok_exec)
    assert other.start() is False
    assert missiond.request(sock, 'ping', {}, 5)[0] == 'result'      # the first one still serves


def test_a_leftover_socket_file_is_replaced(sock):
    s = socket.socket(socket.AF_UNIX)
    s.bind(sock)
    s.close()                                        # a file, nobody listening
    d = Daemon(sock, ok_exec)
    try:
        assert missiond.request(sock, 'ping', {}, 5)[0] == 'result'
    finally:
        d.close()


def test_close_removes_the_socket(daemon, sock):
    d = daemon(ok_exec)
    d.close()
    assert not os.path.exists(sock)


def test_a_bad_request_is_answered_with_a_fallback_and_nothing_runs(daemon, sock):
    ran = []
    daemon(lambda r: ran.append(r) or ok_exec(r))
    s = socket.socket(socket.AF_UNIX)
    s.connect(sock)
    s.makefile().readline()                          # hello
    s.sendall(b'{"op": "abort", "spec": {}}\n')      # abort is not served here
    reply = json.loads(s.makefile().readline())
    s.close()
    assert reply == {'fallback': 'bad request'} and ran == []
    for junk in (b'not json\n', b'[]\n', b'{"op": "load"}\n'):
        s = socket.socket(socket.AF_UNIX)
        s.connect(sock)
        s.makefile().readline()
        s.sendall(junk)
        assert 'fallback' in json.loads(s.makefile().readline())
        s.close()
    assert ran == []


# ---- the client falls back when nothing was sent

def test_no_daemon_means_fallback(sock):
    assert missiond.request(sock, 'load', {}, 5) == ('fallback', 'no daemon socket')


def test_a_dead_daemon_s_socket_file_means_fallback(sock):
    s = socket.socket(socket.AF_UNIX)
    s.bind(sock)
    s.close()
    kind, why = missiond.request(sock, 'load', {}, 5)
    assert kind == 'fallback' and 'cannot connect' in why


def test_a_daemon_that_never_says_hello_means_fallback_fast(sock):
    srv = socket.socket(socket.AF_UNIX)
    srv.bind(sock)
    srv.listen(4)                                    # accepts the connection, says nothing
    t0 = time.monotonic()
    kind, why = missiond.request(sock, 'load', {}, 5)
    srv.close()
    assert (kind, why) == ('fallback', 'no hello') and time.monotonic() - t0 < 1.0


def test_a_hung_monitor_loop_means_fallback_and_nothing_runs(daemon, sock):
    now = {'t': 100.0}
    ran = []
    daemon(lambda r: ran.append(r) or ok_exec(r), clock=lambda: now['t'], loop=False)
    now['t'] = 100.0 + missiond.LOOP_MAX_AGE_S + 1      # the loop has not turned for 2 s
    kind, why = missiond.request(sock, 'start', {}, 5)
    assert kind == 'fallback' and 'stuck' in why and ran == []


def test_a_stale_status_view_makes_the_daemon_refuse_to_answer(daemon, sock):
    ran = []
    daemon(lambda r: ran.append(r) or {'fallback': 'no /mission/status received yet'})
    assert missiond.request(sock, 'start', {}, 5) == ('fallback',
                                                      'no /mission/status received yet')
    assert len(ran) == 1


def test_a_refusal_is_a_result_not_a_fallback(daemon, sock):
    refusal = {'ok': False, 'error': 'another mission is running (RUNNING)'}
    daemon(lambda r: {'result': refusal})
    assert missiond.request(sock, 'load', {}, 5) == ('result', refusal)


def test_no_answer_after_the_request_is_an_error_never_a_second_attempt(daemon, sock):
    ran = []

    def slow(req):
        ran.append(req)
        time.sleep(0.6)
        return ok_exec(req)

    daemon(slow)
    kind, msg = missiond.request(sock, 'start', {'op': 'start'}, 0.2)
    assert kind == 'error' and 'may have happened' in msg
    time.sleep(0.8)
    assert len(ran) == 1


# ---- one at a time

def test_concurrent_requests_are_serialized(daemon, sock):
    spans, lock = [], threading.Lock()

    def slow(req):
        t0 = time.monotonic()
        time.sleep(0.15)
        with lock:
            spans.append((t0, time.monotonic(), req['spec']['n']))
        return ok_exec(req)

    daemon(slow)
    results = []

    def client(n):
        results.append(missiond.request(sock, 'load', {'op': 'load', 'n': n}, 10))

    threads = [threading.Thread(target=client, args=(n,)) for n in range(4)]
    for t in threads:
        t.start()
    for t in threads:
        t.join()
    assert [r[0] for r in results] == ['result'] * 4
    spans.sort()
    assert len(spans) == 4
    for (_, end, _), (start, _, _) in zip(spans, spans[1:]):
        assert start >= end - 1e-3                   # no overlap


def test_a_client_that_left_before_its_turn_is_not_executed(daemon, sock):
    ran = []

    def slow(req):
        ran.append(req['spec']['n'])
        time.sleep(0.4)
        return ok_exec(req)

    daemon(slow)
    first = threading.Thread(target=lambda: missiond.request(sock, 'load', {'n': 1}, 5))
    first.start()
    time.sleep(0.1)
    s = socket.socket(socket.AF_UNIX)
    s.connect(sock)
    s.makefile().readline()
    s.sendall(json.dumps({'op': 'start', 'spec': {'n': 2}}).encode() + b'\n')
    time.sleep(0.05)
    s.close()                                        # gone while the first still runs
    first.join()
    time.sleep(0.6)
    assert ran == [1]


# ---- RosOps: the view of the mission, READY, the e-stop wording

class Pub:
    def __init__(self, gid):
        self.endpoint_gid = bytes([gid])


class FakeNode:
    def __init__(self):
        self.pubs = [Pub(1)]

    def get_publishers_info_by_topic(self, _topic):
        return list(self.pubs)


class FakeEx:
    def spin_once(self, timeout_sec=0):
        pass


class FakeMeter:
    def __init__(self, measured):
        self.measured = measured

    def measure(self, _window):
        return self.measured


CHECKS = [
    {'name': 'scan', 'kind': 'topic_rate', 'required': True, 'topic': '/scan', 'min_hz': 20.0},
    {'name': 'battery', 'kind': 'battery', 'required': True, 'min_voltage': 10.8},
    {'name': 'estop', 'kind': 'estop', 'required': True, 'topic': '/mission/status'},
]
GOOD = {'scan': {'hz': 40.0, 'count': 90},
        'battery': {'received': True, 'has_data': True, 'voltage': 12.1},
        'estop': {'received': True, 'emergency_stop_active': False, 'state': 'LOADED'}}


def make_ops(measured=GOOD, statuses=None, core=None):
    node = FakeNode()
    statuses = [(0.0, False, 'LOADED', '/m.json')] if statuses is None else statuses
    calls = []

    def fake_core(spec, ex, st, load_cli, start_cli, load_type, tick):
        calls.append(spec)
        return {'ok': True, 'ros_state': st[-1][2]}

    ops = missiond.RosOps(node, FakeEx(), statuses, None, None, None, FakeMeter(measured),
                          CHECKS, {'status_topic': '/mission/status', 'window_s': 1.0},
                          core=core or fake_core, ready=ready.evaluate,
                          estop_latched=ready.estop_latched)
    ops.note_status()
    return ops, node, calls


def test_a_good_view_runs_the_core_with_the_op_in_the_spec():
    ops, _, calls = make_ops()
    out = ops.execute({'op': 'load', 'spec': {'path': '/m.json'}})
    assert out == {'result': {'ok': True, 'ros_state': 'LOADED'}}
    assert calls == [{'path': '/m.json', 'op': 'load'}]


def test_no_status_received_yet_is_a_fallback_and_nothing_runs():
    ops, _, calls = make_ops(statuses=[])
    assert ops.execute({'op': 'start', 'spec': {}}) == {
        'fallback': 'no /mission/status received yet'}
    assert calls == []


def test_a_lost_publisher_is_a_fallback():
    ops, node, calls = make_ops()
    node.pubs = []
    out = ops.execute({'op': 'load', 'spec': {}})
    assert 'no publisher' in out['fallback'] and calls == []


def test_a_new_publisher_whose_sample_has_not_arrived_is_a_fallback():
    """The behaviour tree restarted: its latched sample may not be here yet."""
    ops, node, calls = make_ops()
    node.pubs = [Pub(2)]
    out = ops.execute({'op': 'start', 'spec': {}})
    assert 'publishers changed' in out['fallback'] and calls == []
    ops.note_status()                                # the new sample arrives
    assert 'result' in ops.execute({'op': 'start', 'spec': {}})


def test_a_spec_the_daemon_does_not_follow_is_a_fallback():
    ops, _, calls = make_ops()
    out = ops.execute({'op': 'load', 'spec': {'status_topic': '/other/status'}})
    assert 'status_topic differs' in out['fallback'] and calls == []


def test_start_with_a_ready_snapshot_goes_through():
    ops, _, calls = make_ops()
    out = ops.execute({'op': 'start', 'spec': {'require_ready': True, 'path': '/m.json'}})
    assert out['result']['ok'] and len(calls) == 1


@pytest.mark.parametrize('bad, failing, words', [
    ({'battery': {'received': True, 'has_data': True, 'voltage': 7.2}}, ['battery'],
     'not READY: battery'),
    ({'scan': {'hz': 0.0, 'count': 0}}, ['scan'], 'not READY: scan'),
    ({'scan': {'hz': 0.0, 'count': 0},
      'battery': {'received': False, 'has_data': False}}, ['scan', 'battery'],
     'not READY: scan, battery'),
])
def test_a_not_ready_measurement_refuses_start_and_the_core_never_runs(bad, failing, words):
    ops, _, calls = make_ops(measured=dict(GOOD, **bad))
    out = ops.execute({'op': 'start', 'spec': {'require_ready': True}})
    assert out['result']['ok'] is False and out['result']['error'] == words
    assert out['result']['ready_refused'] is True and out['result']['failing'] == failing
    assert calls == []


def test_a_latched_estop_is_refused_with_the_cli_s_own_words():
    latched = dict(GOOD, estop={'received': True, 'emergency_stop_active': True,
                                'state': 'LOADED'})
    ops, _, calls = make_ops(measured=latched)
    out = ops.execute({'op': 'start', 'spec': {'require_ready': True}})
    assert out['result']['error'] == 'the e-stop is latched (estop-reset first)'
    assert calls == []


def test_load_does_not_check_ready_and_a_start_without_the_flag_does_not_either():
    bad = dict(GOOD, battery={'received': False, 'has_data': False})
    ops, _, calls = make_ops(measured=bad)
    assert ops.execute({'op': 'load', 'spec': {'path': '/m.json'}})['result']['ok']
    assert ops.execute({'op': 'start', 'spec': {}})['result']['ok']      # old behaviour kept
    assert len(calls) == 2


def test_the_daemon_serves_only_load_start_and_ping():
    assert missiond.OPS == ('load', 'start', 'ping')
    for forbidden in ('abort', 'abort_mission', 'estop', 'emergency_stop', 'status'):
        assert forbidden not in missiond.OPS


# ---- the CLI: daemon first, same behaviour otherwise

@pytest.fixture
def settings(tmp_path):
    env = tmp_path / 'ctl.env'
    env.write_text(MIN + f'F1T_STATE_DIR={tmp_path / "s"}\n')
    return config.load(str(env))


class Probe:
    """Stands in for run_rosprobe and counts the one-shot probes."""

    def __init__(self, answer=None):
        self.calls = []
        self.answer = answer if answer is not None else {'ok': True, 'one_shot': True}

    def __call__(self, settings, mode, spec, timeout):
        self.calls.append((mode, spec.get('op')))
        return dict(self.answer)


def test_the_cli_uses_the_daemon_s_answer_and_no_probe(settings, monkeypatch, tmp_path):
    probe = Probe()
    monkeypatch.setattr(cli, 'run_rosprobe', probe)
    monkeypatch.setattr(missiond, 'request',
                        lambda *a, **k: ('result', {'ok': True, 'daemon': True}))
    assert cli.run_mission_op(settings, {'op': 'load'}, 25) == {'ok': True, 'daemon': True}
    assert probe.calls == []


@pytest.mark.parametrize('why', ['no daemon socket', 'no hello', 'daemon loop stuck (3 s)',
                                 'no /mission/status received yet'])
def test_the_cli_falls_back_to_the_old_probe_when_the_daemon_cannot_take_it(
        settings, monkeypatch, why):
    probe = Probe()
    monkeypatch.setattr(cli, 'run_rosprobe', probe)
    monkeypatch.setattr(missiond, 'request', lambda *a, **k: ('fallback', why))
    assert cli.run_mission_op(settings, {'op': 'start'}, 25) == {'ok': True, 'one_shot': True}
    assert probe.calls == [('mission', 'start')]


def test_the_cli_never_retries_after_the_request_went_out(settings, monkeypatch):
    probe = Probe()
    monkeypatch.setattr(cli, 'run_rosprobe', probe)
    monkeypatch.setattr(missiond, 'request', lambda *a, **k: ('error', 'no answer'))
    assert cli.run_mission_op(settings, {'op': 'start'}, 25) == {'error': 'no answer'}
    assert probe.calls == []


def test_a_refusal_from_the_daemon_is_returned_as_it_is(settings, monkeypatch):
    probe = Probe()
    monkeypatch.setattr(cli, 'run_rosprobe', probe)
    refusal = {'ok': False, 'error': 'another mission is running (RUNNING)'}
    monkeypatch.setattr(missiond, 'request', lambda *a, **k: ('result', refusal))
    assert cli.run_mission_op(settings, {'op': 'load'}, 25) == refusal
    assert probe.calls == []


def test_only_load_and_start_ever_reach_the_daemon(settings, monkeypatch):
    probe = Probe()
    sent = []
    monkeypatch.setattr(cli, 'run_rosprobe', probe)
    monkeypatch.setattr(missiond, 'request',
                        lambda p, op, *a, **k: sent.append(op) or ('result', {'ok': True}))
    for op in ('status', 'validate', 'abort', 'estop'):
        cli.run_mission_op(settings, {'op': op}, 25)
    assert sent == [] and len(probe.calls) == 4
    cli.run_mission_op(settings, {'op': 'load'}, 25)
    cli.run_mission_op(settings, {'op': 'start'}, 25)
    assert sent == ['load', 'start']


def test_the_daemon_can_be_switched_off_in_the_env_file(tmp_path, monkeypatch):
    env = tmp_path / 'off.env'
    env.write_text(MIN + f'F1T_STATE_DIR={tmp_path / "s"}\nF1T_MISSIOND=0\n')
    s = config.load(str(env))
    probe = Probe()
    monkeypatch.setattr(cli, 'run_rosprobe', probe)
    monkeypatch.setattr(missiond, 'request', lambda *a, **k: pytest.fail('used the daemon'))
    cli.run_mission_op(s, {'op': 'load'}, 25)
    assert probe.calls == [('mission', 'load')]
    assert 'mission_sock' not in monitor.probe_spec(s, {'checks': [], 'window_s': 1.0,
                                                        'use_sim_time': False})


def test_the_monitor_spec_carries_the_socket_and_the_status_topic(settings):
    spec = monitor.probe_spec(settings, {'checks': [], 'window_s': 1.0, 'use_sim_time': False})
    assert spec['mission_sock'] == os.path.join(settings.state_dir, 'missiond.sock')
    assert spec['status_topic'] == settings.status_topic


# ---- mission-start: the same refusal, the same exit code, whichever side refuses

def _start_args():
    return argparse.Namespace(id=None, allow_old=False)


def _stage_start(settings, monkeypatch, ready_now):
    rec = {'id': 'r1', 'stored_at': time.time(), 'path': '/m.json', 'mission_id': 'llm_x'}
    monkeypatch.setattr(cli.missions, 'current', lambda _s: {'id': 'r1'})
    monkeypatch.setattr(cli.missions, 'get', lambda _s, _i: rec)
    monkeypatch.setattr(cli, 'cmd_status', lambda *a, **k: {
        'ready': ready_now, 'failing': [] if ready_now else ['scan', 'battery'],
        'estop_latched': False})


def test_not_ready_on_the_cli_side_and_on_the_daemon_side_refuse_identically(
        settings, monkeypatch):
    monkeypatch.setattr(cli, 'run_rosprobe', Probe())
    _stage_start(settings, monkeypatch, ready_now=False)
    cli_side = cli.cmd_mission_start(settings, _start_args(), time.monotonic())
    _stage_start(settings, monkeypatch, ready_now=True)
    monkeypatch.setattr(missiond, 'request', lambda *a, **k: (
        'result', {'ok': False, 'error': 'not READY: scan, battery', 'ready_refused': True,
                   'failing': ['scan', 'battery']}))
    daemon_side = cli.cmd_mission_start(settings, _start_args(), time.monotonic())
    assert daemon_side == cli_side
    assert cli_side['ok'] is False and cli_side['error'] == 'not READY: scan, battery'
    assert cli_side['refused'] == 'not READY: scan, battery' and cli_side['id'] == 'r1'


def test_the_start_spec_asks_the_daemon_for_its_own_ready_check(settings, monkeypatch):
    _stage_start(settings, monkeypatch, ready_now=True)
    seen = []
    monkeypatch.setattr(cli, 'run_mission_op',
                        lambda s, spec, timeout: seen.append(spec) or {'ok': False, 'error': 'x'})
    cli.cmd_mission_start(settings, _start_args(), time.monotonic())
    assert seen[0]['op'] == 'start' and seen[0]['require_ready'] is True
    assert seen[0]['path'] == '/m.json'


def test_a_refused_start_exits_1_through_main(settings, monkeypatch, capsys):
    _stage_start(settings, monkeypatch, ready_now=True)
    monkeypatch.setattr(cli, 'load', lambda _e: settings)
    monkeypatch.setattr(missiond, 'request', lambda *a, **k: (
        'result', {'ok': False, 'error': 'not READY: scan', 'ready_refused': True,
                   'failing': ['scan']}))
    code = cli.main(['mission-start'])
    out = json.loads(capsys.readouterr().out)
    assert code == 1 and out['ok'] is False and out['error'] == 'not READY: scan'


def test_abort_and_estop_do_not_know_the_daemon():
    """The e-stop and abort paths must not import or call the daemon."""
    import inspect
    for fn in (cli.cmd_estop, cli.cmd_estop_reset, cli.cmd_killall):
        assert 'missiond' not in inspect.getsource(fn)
        assert 'run_mission_op' not in inspect.getsource(fn)
    from f1tenth_ctl import rosprobe
    for fn in (rosprobe.estop, rosprobe.estop_with, rosprobe.estop_reset):
        assert 'missiond' not in inspect.getsource(fn)
