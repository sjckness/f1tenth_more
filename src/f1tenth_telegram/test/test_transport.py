"""Transport: JSON parsing, ssh command lines, timeouts and ssh failures."""

import json
import os
import stat
import time

from f1tenth_telegram.config import Target
from f1tenth_telegram.transport import parse_ctl_output, Transport
import pytest


def test_parse_takes_the_last_json_object():
    out = 'some warning\n{"ok": true, "cmd": "status", "ready": false}\n'
    res = parse_ctl_output(out, '', 0)
    assert res['ok'] is True and res['ready'] is False and res['_rc'] == 0


def test_parse_keeps_a_failed_commands_answer():
    res = parse_ctl_output('{"ok": false, "error": "stack is already running"}', '', 1)
    assert res['ok'] is False and 'already running' in res['error']
    assert 'transport_error' not in res


def test_parse_ssh_failure_without_json():
    res = parse_ctl_output('', 'ssh: connect to host x port 22: Connection timed out\n', 255)
    assert res['transport_error'] == 'ssh' and 'timed out' in res['error']


def test_parse_missing_ctl_and_garbage():
    assert parse_ctl_output('', 'bash: f1tenth_ctl: not found', 127)['transport_error'] \
        == 'not_found'
    res = parse_ctl_output('Traceback ...\nValueError: x\n', '', 1)
    assert res['transport_error'] == 'parse' and res['ok'] is False
    assert parse_ctl_output('[1, 2]\n', '', 0)['transport_error'] == 'parse'


def test_ssh_command_line():
    t = Transport(Target(host='jetson-orin', ssh_key='~/.ssh/k', ssh_options=['Port=22'],
                         ctl_command='~/ws/bin/f1tenth_ctl', ctl_env='/x y/sim.env'))
    argv = t.ctl_argv(['send', 'planner', 'go ahead; rm -rf ~'])
    assert argv[:2] == ['ssh', '-o'] and 'BatchMode=yes' in argv
    assert 'ConnectTimeout=5' in argv and os.path.expanduser('~/.ssh/k') in argv
    assert argv[argv.index('--') - 1] == 'jetson-orin'
    remote = argv[-1]
    # ~ stays for the remote shell; every argument is quoted.
    assert remote.startswith('~/ws/bin/f1tenth_ctl send planner ')
    assert "'go ahead; rm -rf ~'" in remote and "'/x y/sim.env'" in remote
    assert remote.endswith(' --json')


def test_local_command_line():
    t = Transport(Target(host='local', ctl_command='~/bin/f1tenth_ctl'))
    assert t.ctl_argv(['status']) == [os.path.expanduser('~/bin/f1tenth_ctl'),
                                      'status', '--json']


def _fake(tmp_path, name, body):
    path = tmp_path / name
    path.write_text('#!/bin/sh\n' + body + '\n')
    path.chmod(path.stat().st_mode | stat.S_IEXEC)
    return path


@pytest.fixture
def fake_bin(tmp_path, monkeypatch):
    d = tmp_path / 'bin'
    d.mkdir()
    monkeypatch.setenv('PATH', f'{d}:{os.environ["PATH"]}')
    return d


def test_ssh_timeout_is_reported_not_raised(fake_bin):
    _fake(fake_bin, 'ssh', 'sleep 10')
    t = Transport(Target(host='car'))
    t0 = time.monotonic()
    res = t.ctl(['status'], timeout=0.5)
    assert time.monotonic() - t0 < 3.0
    assert res['ok'] is False and res['transport_error'] == 'timeout'


def test_ssh_unreachable_host(fake_bin):
    _fake(fake_bin, 'ssh', 'echo "ssh: connect to host car port 22: No route to host" >&2; '
                           'exit 255')
    res = Transport(Target(host='car')).ctl(['status'], timeout=5)
    assert res['transport_error'] == 'ssh' and 'No route' in res['error']


def test_answer_through_fake_ssh(fake_bin):
    answer = json.dumps({'ok': True, 'cmd': 'status', 'ready': True})
    _fake(fake_bin, 'ssh', f"echo '{answer}'")
    res = Transport(Target(host='car')).ctl(['status'], timeout=5)
    assert res['ok'] and res['ready'] and res['_elapsed_s'] >= 0


def test_reachability_online_and_offline(fake_bin):
    _fake(fake_bin, 'ping', 'exit 1')
    _fake(fake_bin, 'ssh', 'exit 0')
    r = Transport(Target(host='car')).reachability(timeout=3)
    assert r['online'] is True and r['ping'] is False and r['ssh'] is True
    _fake(fake_bin, 'ssh', 'echo "ssh: connect to host car: Connection timed out" >&2; '
                           'exit 255')
    r = Transport(Target(host='car')).reachability(timeout=3)
    assert r['online'] is False and 'timed out' in r['error']


def test_reachability_ssh_hangs(fake_bin):
    _fake(fake_bin, 'ping', 'exit 0')
    _fake(fake_bin, 'ssh', 'sleep 10')
    r = Transport(Target(host='car')).reachability(timeout=0.5)
    assert r['online'] is False and 'no answer' in r['error']


def test_local_target_is_always_online_and_runs_directly(tmp_path):
    ctl = _fake(tmp_path, 'f1tenth_ctl', 'echo "{\\"ok\\": true, \\"args\\": \\"$*\\"}"')
    t = Transport(Target(host='local', ctl_command=str(ctl)))
    assert t.reachability(1)['online'] is True
    res = t.ctl(['start', 'stack'], timeout=5)
    assert res['ok'] and res['args'] == 'start stack --json'
