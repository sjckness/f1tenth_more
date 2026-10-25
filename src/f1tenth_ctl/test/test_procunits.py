"""
The proc backend (procunits.py + runner.py) on its own: no systemd needed.

Dummy processes under real runners, in a temporary state directory: orphans
and the control-group stop, PID reuse, the stdin FIFO, exit status versus
signal, the runner's records, the reset a new container does, and the
backend choice. test_process_manager.py runs the CLI end to end on both
backends; the pure tests here pin the property dicts to systemd's, so
classify() and the JSON stay the same.
"""

import json
import os
import shlex
import signal
import subprocess
import sys
import time

from f1tenth_ctl import cli, config, procunits, runner, units
import pytest

PREFIX = f'f1t-ptest-{os.getpid()}-'


def _env_file(tmp_path, cmds, extra=''):
    lines = ['F1T_PROCESS_BACKEND=proc', f'F1T_UNIT_PREFIX={PREFIX}',
             f'F1T_STATE_DIR={tmp_path}/state', f'F1T_WORKDIR={tmp_path}/cwd',
             'F1T_STOP_SIGINT_WAIT_S=2', 'F1T_STOP_SIGTERM_WAIT_S=1',
             'F1T_GRAPH_CHECK=false', 'F1T_MONITOR=false', 'F1T_PLANNER_STDIN=fifo']
    for name in config.MANAGED:
        lines.append(f'F1T_{name.upper()}_CMD={shlex.quote(cmds[name])}')
    path = tmp_path / 'proc.env'
    path.write_text('\n'.join(lines) + '\n' + extra)
    return str(path)


@pytest.fixture
def make(tmp_path):
    """Return settings for the given commands; every runner is gone after the test."""
    made = []

    def _make(stack='sleep 300', planner='cat', logger='sleep 300', extra=''):
        s = config.load(_env_file(tmp_path, {'stack': stack, 'planner': planner,
                                             'logger': logger}, extra))
        made.append(s)
        return s
    yield _make
    for s in made:
        procunits.sweep(s)


def _wait(pred, timeout=8.0, poll=0.05):
    deadline = time.monotonic() + timeout
    while True:
        value = pred()
        if value or time.monotonic() >= deadline:
            return value
        time.sleep(poll)


def _pids_with(marker):
    """Pids of live processes with `marker` as one of their arguments."""
    out = []
    for name in os.listdir('/proc'):
        if not name.isdigit():
            continue
        try:
            with open(f'/proc/{name}/cmdline', 'rb') as f:
                argv = [a.decode(errors='replace') for a in f.read().split(b'\0')]
        except OSError:
            continue
        st = runner.proc_stat(int(name))
        if marker in argv and st and st['state'] not in ('Z', 'X'):
            out.append(int(name))
    return out


def _state(s, name='stack'):
    return units.unit_states(s, [name])[name]


def _info(s, name='stack'):
    with open(os.path.join(procunits.unit_dir(s, s.procs[name].unit), 'proc.json')) as f:
        return json.load(f)


# ---- start, records, stop --------------------------------------------------------

def test_start_runs_the_command_under_a_detached_runner(make):
    s = make()
    res = units.start(s, s.procs['stack'], settle_s=0.3)
    assert res['ok'] and res['state'] == 'running', res
    assert res['unit'] == f'{PREFIX}stack.service' and res['managed'] is True
    info = _info(s)
    pid = res['pid']
    assert info['pid'] == pid and info['pid_start'] == runner.proc_stat(pid)['start']
    assert info['boot_id'] == runner.boot_id() and info['subreaper'] is True
    # Its own session and group (SIGINT to the group = Ctrl-C), and a runner
    # that is neither our child nor in our session.
    assert os.getsid(pid) == pid and os.getpgid(pid) == pid
    rpid = info['runner_pid']
    assert runner.proc_stat(rpid)['ppid'] != os.getpid()
    assert os.getsid(rpid) != os.getsid(0)
    assert runner.proc_stat(pid)['ppid'] == rpid
    # The working directory is F1T_WORKDIR.
    assert os.readlink(f'/proc/{pid}/cwd') == s.workdir
    out = units.stop(s, s.procs['stack'])
    assert out['ok'] and out['signals'] == ['SIGINT'], out
    assert _state(s)['state'] == 'stopped'
    assert not runner.alive(pid, info['pid_start'])
    assert not runner.alive(rpid, info['runner_start'])
    assert os.listdir(procunits.unit_dir(s, s.procs['stack'].unit)) == []


def test_a_second_start_is_refused_while_it_runs(make):
    s = make()
    assert units.start(s, s.procs['stack'], settle_s=0.0)['ok']
    again = units.start(s, s.procs['stack'], settle_s=0.0)
    assert again['ok'] is False and 'already running' in again['error']


def test_an_orphan_belongs_to_the_unit_and_is_stopped_with_it(make):
    s = make(stack="sh -c '(setsid sleep 3111 &); exec sleep 3112'")
    res = units.start(s, s.procs['stack'], settle_s=0.3)
    assert res['ok'], res
    rpid = _info(s)['runner_pid']
    orphans = _wait(lambda: [p for p in _pids_with('3111')
                             if runner.proc_stat(p)['ppid'] == rpid])
    # Re-parented to the runner (child subreaper), not to init: still the unit's.
    assert len(orphans) == 1, _pids_with('3111')
    assert orphans[0] in units.unit_pids(s.procs['stack'].unit, s)
    assert os.getsid(orphans[0]) != res['pid']       # outside the main's group
    out = units.stop(s, s.procs['stack'])
    # SIGINT ended the main process; the runner stopped the orphan (SIGTERM).
    assert out['ok'] and out['signals'] == ['SIGINT'], out
    assert _pids_with('3111') == [] and _pids_with('3112') == []


def test_what_an_ended_process_leaves_behind_is_stopped(make):
    s = make(stack="sh -c '(setsid sleep 3121 &); sleep 0.5; exit 3'")
    assert units.start(s, s.procs['stack'], settle_s=0.0)['ok']
    assert _wait(lambda: _pids_with('3121'))
    st = _wait(lambda: _state(s) if _state(s)['state'] == 'crashed' else None)
    assert st and st['exit_status'] == 3 and st['result'] == 'exit-code', st
    assert _pids_with('3121') == []


def test_an_orphan_that_ignores_sigterm_gets_sigkill(make):
    code = 'import signal, time; signal.signal(signal.SIGTERM, signal.SIG_IGN); time.sleep(300)'
    s = make(stack=f"sh -c '(setsid {sys.executable} -c \"{code}\" mark3131 &); "
                   "sleep 0.5; exit 0'")
    assert units.start(s, s.procs['stack'], settle_s=0.0)['ok']
    assert _wait(lambda: _pids_with('mark3131'))
    t0 = time.monotonic()
    # Main gone, the orphan still there: stopping (deactivating), then gone.
    assert _wait(lambda: _state(s)['state'] == 'stopping')
    assert _wait(lambda: _state(s)['state'] not in ('running', 'stopping'))
    assert time.monotonic() - t0 < 5.0              # kill timeout 1 s, then SIGKILL
    assert _pids_with('mark3131') == []
    # An exit 0 nobody asked for is still a crash (systemd: inactive + desired running).
    st = _state(s)
    assert st['state'] == 'crashed' and st['result'] == 'exited without a stop request', st


@pytest.mark.parametrize('cmd,expect', [
    ('sh -c "sleep 0.3; exit 4"', {'exit_status': 4, 'result': 'exit-code'}),
    ('sh -c "sleep 0.3; kill -USR1 $$"', {'signal': 'SIGUSR1', 'result': 'signal'}),
    # Clean signals (systemd's SuccessExitStatus) and exit 0: inactive, so
    # crashed only because nobody asked for the stop.
    ('sh -c "sleep 0.3; kill -TERM $$"', {'result': 'exited without a stop request'}),
    ('sh -c "sleep 0.3; exit 0"', {'result': 'exited without a stop request'}),
])
def test_an_end_is_reported_as_exit_status_or_signal(make, cmd, expect):
    s = make(stack=cmd)
    assert units.start(s, s.procs['stack'], settle_s=0.0)['ok']
    st = _wait(lambda: _state(s) if _state(s)['state'] == 'crashed' else None)
    assert st, _state(s)
    for key, value in expect.items():
        assert st.get(key) == value, st
    assert ('exit_status' in st) == ('exit_status' in expect)
    assert ('signal' in st) == ('signal' in expect)
    # stop clears it, as systemctl reset-failed does.
    units.stop(s, s.procs['stack'])
    assert _state(s)['state'] == 'stopped'


def test_the_fifo_is_the_planners_stdin(make):
    s = make()
    assert units.start(s, s.procs['planner'], settle_s=0.2)['ok']
    sent = units.fifo_send(s, 'planner', 'go ahead 2 meters')
    assert sent['ok'] and isinstance(sent['cursor'], int)
    lines = _wait(lambda: 'go ahead 2 meters' in units.log_tail(s, 'planner')
                  and units.log_tail(s, 'planner'))
    assert lines, units.log_tail(s, 'planner')
    # `cat` is still there: the FIFO never reached EOF.
    assert _state(s, 'planner')['state'] == 'running'
    assert units.stop(s, s.procs['planner'])['ok']


def test_sigterm_to_the_runner_stops_the_whole_tree(make):
    """procunits.finish_stop is `systemctl stop`: the runner takes the tree down."""
    s = make(stack="sh -c '(setsid sleep 3141 &); exec sleep 3142'")
    assert units.start(s, s.procs['stack'], settle_s=0.0)['ok']
    assert _wait(lambda: _pids_with('3141'))
    out = procunits.finish_stop(s, s.procs['stack'].unit)
    assert out['forced'] is False
    assert _pids_with('3141') == [] and _pids_with('3142') == []
    assert os.listdir(procunits.unit_dir(s, s.procs['stack'].unit)) == []


def test_a_command_that_cannot_run_is_a_runner_error(tmp_path):
    d = tmp_path / 'unit'
    proc = subprocess.run(
        [sys.executable, '-m', 'f1tenth_ctl.runner', '--unit', 'x.service', '--dir', str(d),
         '--log', str(tmp_path / 'x.log'), '--workdir', str(tmp_path), '--',
         str(tmp_path / 'no_such_program')],
        capture_output=True, text=True, timeout=15,
        env={**os.environ, 'PYTHONPATH': config.SOURCE_ROOT})
    assert proc.returncode == 1 and 'cannot run' in proc.stderr
    ex = json.loads((d / 'exit.json').read_text())
    assert ex['status'] == runner.EXIT_EXEC and ex['result'] == 'exit-code'
    with pytest.raises(SystemExit):
        runner.parse_args(['--unit', 'u', '--dir', 'd', '--log', 'l', '--workdir', 'w', '--'])


# ---- records against PID reuse ------------------------------------------------------

def _fake_record(s, unit, **info):
    d = procunits.unit_dir(s, unit)
    os.makedirs(d, exist_ok=True)
    runner.write_json(os.path.join(d, 'proc.json'), info)


def test_a_reused_pid_is_never_the_process(make):
    s = make()
    me = runner.proc_stat(os.getpid())
    base = {'pid': os.getpid(), 'runner_pid': os.getpid(), 'boot_id': runner.boot_id(),
            'started_at': time.time()}
    unit = f'{PREFIX}fake.service'
    # Same pids, other start times: a reused pid, so nothing of the unit lives.
    _fake_record(s, unit, **base, pid_start=me['start'] + 1, runner_start=me['start'] + 1)
    p = procunits.show(s, [unit])[unit]
    assert p['ActiveState'] == 'failed' and p['Result'] == 'runner-lost'
    # The same record with the true start times: alive.
    _fake_record(s, unit, **base, pid_start=me['start'], runner_start=me['start'])
    p = procunits.show(s, [unit])[unit]
    assert p['ActiveState'] == 'active' and p['MainPID'] == str(os.getpid())
    # From another boot: ignored altogether, like a transient unit after a reboot.
    _fake_record(s, unit, **{**base, 'boot_id': 'another-boot'}, pid_start=me['start'],
                 runner_start=me['start'])
    p = procunits.show(s, [unit])[unit]
    assert p['ActiveState'] == 'inactive' and p['LoadState'] == 'not-found'
    procunits._clear(s, unit)


def test_reset_forgets_ended_processes_and_keeps_live_ones(make):
    s = make(logger="sh -c 'exit 5'")
    assert units.start(s, s.procs['stack'], settle_s=0.0)['ok']
    units.start(s, s.procs['logger'], settle_s=0.0)
    assert _wait(lambda: _state(s, 'logger')['state'] == 'crashed')
    units.write_desired(s, 'planner', 'running')        # a planner of a previous container
    assert _state(s, 'planner')['state'] == 'crashed'
    out = procunits.reset(s)
    assert 'stack' in out['alive'] and {'planner', 'logger', 'discovery'} <= set(out['reset'])
    assert _state(s, 'planner')['state'] == 'stopped'
    assert _state(s, 'logger')['state'] == 'stopped'
    assert _state(s)['state'] == 'running'


# ---- the contract with systemd (pure) ---------------------------------------------

def test_exit_records_follow_systemd():
    assert runner.exit_record(3 << 8) == {'code': 1, 'status': 3, 'result': 'exit-code'}
    assert runner.exit_record(0)['result'] == 'success'
    assert runner.exit_record(signal.SIGTERM) == {'code': 2, 'status': 15, 'result': 'success'}
    assert runner.exit_record(signal.SIGKILL)['result'] == 'signal'
    dumped = {'code': 3, 'status': 11, 'result': 'core-dump'}
    assert runner.exit_record(signal.SIGSEGV | 0x80) == dumped
    assert runner.exit_record(signal.SIGKILL, stop_requested=True,
                              escalated=True)['result'] == 'timeout'


SYSTEMD_CASES = [
    # (record for props_of, what `systemctl show` says in the same case)
    ({'info': {'pid': 42, 'started_at': 0}, 'main_alive': True, 'runner_alive': True},
     {'ActiveState': 'active', 'MainPID': '42', 'ActiveEnterTimestamp': 'x'}),
    ({'info': {'pid': 42}, 'runner_alive': True},
     {'ActiveState': 'deactivating'}),
    ({'info': {'pid': 42}, 'exit': {'code': 1, 'status': 3, 'result': 'exit-code'}},
     {'ActiveState': 'failed', 'Result': 'exit-code', 'ExecMainCode': '1',
      'ExecMainStatus': '3'}),
    ({'info': {'pid': 42}, 'exit': {'code': 2, 'status': 11, 'result': 'signal'}},
     {'ActiveState': 'failed', 'Result': 'signal', 'ExecMainCode': '2', 'ExecMainStatus': '11'}),
    ({'info': {'pid': 42}, 'exit': {'code': 2, 'status': 15, 'result': 'success'}},
     {'ActiveState': 'inactive', 'Result': 'success'}),
    ({}, {'ActiveState': 'inactive', 'LoadState': 'not-found'}),
]


@pytest.mark.parametrize('rec,sysd', SYSTEMD_CASES)
@pytest.mark.parametrize('desired', [{}, {'desired': 'running'}, {'desired': 'stopped'}])
def test_classify_sees_the_same_states_as_with_systemd(rec, sysd, desired):
    unit = 'f1t-stack.service'
    mine = units.classify(procunits.props_of(unit, rec), desired)
    theirs = units.classify({'Id': unit, **sysd}, desired)
    if 'since' in theirs:
        assert mine['since'] and theirs['since']
        mine['since'] = theirs['since']
    assert mine == theirs


# ---- the backend choice ----------------------------------------------------------

@pytest.fixture
def fresh_choice():
    units._auto_choice.clear()
    yield
    units._auto_choice.clear()


def test_auto_picks_proc_in_a_container_or_without_a_user_manager(
        fresh_choice, monkeypatch, make):
    s = make(extra='F1T_PROCESS_BACKEND=auto\n')
    assert s.process_backend == 'auto'
    monkeypatch.setattr(units, 'in_container', lambda: True)
    monkeypatch.setattr(units, 'user_manager_answers', lambda settings=None: True)
    assert units.backend_choice(s) == {'backend': 'proc', 'why': 'auto: inside a container'}
    units._auto_choice.clear()
    monkeypatch.setattr(units, 'in_container', lambda: False)
    assert units.backend(s) == 'systemd'
    units._auto_choice.clear()
    monkeypatch.setattr(units, 'user_manager_answers', lambda settings=None: False)
    assert units.backend(s) == 'proc'
    # Decided once per process.
    monkeypatch.setattr(units, 'user_manager_answers', lambda settings=None: True)
    assert units.backend(s) == 'proc'


def test_an_explicit_backend_is_kept_and_a_wrong_one_refused(fresh_choice, make, tmp_path):
    assert units.backend(make(extra='F1T_PROCESS_BACKEND=systemd\n')) == 'systemd'
    assert units.backend(make()) == 'proc'
    with pytest.raises(config.ConfigError, match='F1T_PROCESS_BACKEND'):
        make(extra='F1T_PROCESS_BACKEND=tmux\n')


def test_check_has_the_backend_items_instead_of_systemds(make, capsys):
    s = make(extra='F1T_SHELL_REF=off\n')
    code = cli.main(['check', '--env', s.env_file])
    out = json.loads(capsys.readouterr().out)
    names = [i['name'] for i in out['items']]
    assert 'process_backend' in names and 'workdir' in names
    assert 'systemd_user' not in names and 'linger' not in names
    assert 'workdir' not in out['failing'] and code in (0, 1)
