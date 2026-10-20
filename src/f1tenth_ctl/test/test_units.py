"""systemd unit handling without systemd: show parsing, states, the stop ladder."""

from f1tenth_ctl import config, units


def _settings(tmp_path, extra=''):
    path = tmp_path / 't.env'
    path.write_text('\n'.join(f'F1T_{n.upper()}_CMD="sleep 1"' for n in config.MANAGED)
                    + f'\nF1T_STATE_DIR={tmp_path}/state\n' + extra)
    return config.load(str(path))


def test_parse_show_splits_units():
    text = ('Id=a.service\nActiveState=active\nMainPID=12\n\n'
            'Id=b.service\nActiveState=failed\nResult=exit-code\n')
    blocks = units.parse_show(text)
    assert [b['Id'] for b in blocks] == ['a.service', 'b.service']
    assert blocks[1]['Result'] == 'exit-code'


def test_classify_running():
    st = units.classify({'Id': 'f1t-stack.service', 'ActiveState': 'active',
                         'MainPID': '42'}, {})
    assert st['state'] == 'running' and st['pid'] == 42 and st['managed']


def test_classify_failed_exit_code_is_crashed():
    st = units.classify({'ActiveState': 'failed', 'Result': 'exit-code',
                         'ExecMainCode': '1', 'ExecMainStatus': '3'}, {})
    assert st['state'] == 'crashed' and st['exit_status'] == 3


def test_classify_failed_by_signal_is_crashed():
    st = units.classify({'ActiveState': 'failed', 'Result': 'signal',
                         'ExecMainCode': '2', 'ExecMainStatus': '11'}, {})
    assert st['state'] == 'crashed' and st['signal'] == 'SIGSEGV'


def test_gone_without_a_stop_request_is_crashed():
    st = units.classify({'ActiveState': 'inactive', 'LoadState': 'not-found'},
                        {'desired': 'running'})
    assert st['state'] == 'crashed'
    st = units.classify({'ActiveState': 'inactive'}, {'desired': 'stopped'})
    assert st['state'] == 'stopped'
    assert units.classify({'ActiveState': 'inactive'}, {})['state'] == 'stopped'


def test_desired_state_round_trip(tmp_path):
    s = _settings(tmp_path)
    assert units.read_desired(s, 'stack') == {}
    units.write_desired(s, 'stack', 'running', cmd='x')
    assert units.read_desired(s, 'stack')['desired'] == 'running'


def test_build_command_keeps_the_command_out_of_execstart(tmp_path):
    s = _settings(tmp_path, 'F1T_PLANNER_STDIN=fifo\n'
                  "F1T_STACK_CMD='ros2 launch x y.launch.py a:=${HOME} b:=$$'\n")
    cmd = units.build_command(s, s.procs['stack'])
    assert cmd[:2] == ['systemd-run', '--user']
    assert '--unit=f1t-stack.service' in cmd
    assert '--setenv=F1T_UNIT_CMD=ros2 launch x y.launch.py a:=${HOME} b:=$$' in cmd
    assert cmd[-3:] == ['/bin/bash', '-c', units.UNIT_SCRIPT]
    # systemd expands "${VAR}" and "$$" in ExecStart; the script has neither.
    assert '${' not in units.UNIT_SCRIPT and '$$' not in units.UNIT_SCRIPT
    assert f'StandardOutput=append:{s.log_path("stack")}' in cmd
    assert '--setenv=F1T_UNIT_FIFO=' in cmd
    planner = units.build_command(s, s.procs['planner'])
    assert f'--setenv=F1T_UNIT_FIFO={s.fifo_path("planner")}' in planner


def _ladder(down_after):
    sent = []

    def send(sig):
        sent.append(sig)

    def is_down(_timeout):
        return sent[-1] in down_after

    res = units.stop_ladder(is_down, send, 0.0, 0.0, 0.0)
    return res, sent


def test_ladder_stops_at_sigint():
    res, sent = _ladder({'SIGINT'})
    assert res['stopped'] and sent == ['SIGINT']


def test_ladder_escalates_to_sigterm_then_sigkill():
    res, sent = _ladder({'SIGTERM'})
    assert res['stopped'] and sent == ['SIGINT', 'SIGTERM']
    res, sent = _ladder({'SIGKILL'})
    assert res['stopped'] and sent == ['SIGINT', 'SIGTERM', 'SIGKILL']


def test_ladder_reports_a_survivor():
    res, sent = _ladder(set())
    assert not res['stopped'] and sent == ['SIGINT', 'SIGTERM', 'SIGKILL']


def test_log_tail_reads_the_end(tmp_path):
    s = _settings(tmp_path)
    (tmp_path / 'state' / 'logs').mkdir(parents=True)
    with open(s.log_path('stack'), 'w') as f:
        f.write('\n'.join(str(i) for i in range(100)))
    assert units.log_tail(s, 'stack', 3) == ['97', '98', '99']
    assert units.log_tail(s, 'planner') == []


def test_check_line_accepts_one_line_up_to_500_chars():
    assert units.check_line('  go ahead 2 meters ') == ('go ahead 2 meters', None)
    assert units.check_line('x' * 500)[0] == 'x' * 500
    line, why = units.check_line('x' * 501)
    assert line is None and '501' in why
    assert units.check_line('go\nahead')[1].startswith('one line only')
    assert units.check_line('go\rahead')[1].startswith('one line only')
    assert units.check_line('   ')[1] == 'empty text'


def test_read_log_since_returns_complete_lines_only(tmp_path):
    log = tmp_path / 'planner.log'
    log.write_bytes(b'old line\n')
    start = units.read_log_since(str(log), None)
    assert start['cursor'] == 9 and start['lines'] == []
    with open(log, 'ab') as f:
        f.write(b'\x1b[32m[INFO] colored\x1b[0m\ncomando> RESULT {"status": "ok"}\npartial')
    res = units.read_log_since(str(log), start['cursor'])
    assert res['lines'] == ['[INFO] colored', 'comando> RESULT {"status": "ok"}']
    # The unfinished line is not consumed: it comes whole next time.
    assert res['cursor'] == log.stat().st_size - len(b'partial')
    with open(log, 'ab') as f:
        f.write(b' line\n')
    assert units.read_log_since(str(log), res['cursor'])['lines'] == ['partial line']


def test_read_log_since_after_a_reset_and_a_cap(tmp_path):
    log = tmp_path / 'planner.log'
    log.write_bytes(b'a\nb\n')
    res = units.read_log_since(str(log), 999)
    assert res['reset'] and res['lines'] == ['a', 'b'] and res['cursor'] == 4
    log.write_bytes(b'0123456789\nxyz\n')
    capped = units.read_log_since(str(log), 0, max_bytes=12)
    assert capped['lines'] == ['0123456789'] and capped['truncated'] and capped['cursor'] == 11
    assert units.read_log_since(str(tmp_path / 'missing.log'), 0)['lines'] == []
