"""
The process manager end to end, with dummy processes instead of ros2 launch.

Real transient units under `systemd --user`, through the CLI's JSON. Every
unit is named f1t-test-<pid>-<name>.service, so these tests can never touch
the real f1t-stack/planner/logger units of a machine they run on. Skipped
where there is no user manager (containers, CI).
"""

import json
import os
import subprocess
import time

from f1tenth_ctl import cli, config
import pytest

PREFIX = f'f1t-test-{os.getpid()}-'


def _user_manager_ok():
    try:
        out = subprocess.run(['systemctl', '--user', 'is-system-running'],
                             capture_output=True, text=True, timeout=5).stdout.strip()
    except (OSError, subprocess.TimeoutExpired):
        return False
    return out in ('running', 'degraded')


pytestmark = pytest.mark.skipif(not _user_manager_ok(), reason='no systemd --user')

IGNORES_SIGINT = ("python3 -c 'import signal, time; "
                  "signal.signal(signal.SIGINT, signal.SIG_IGN); time.sleep(300)'")


@pytest.fixture
def env_file(tmp_path):
    path = tmp_path / 'test.env'
    path.write_text(f"""
F1T_UNIT_PREFIX={PREFIX}
F1T_STATE_DIR={tmp_path}/state
F1T_GRAPH_CHECK=false
F1T_MONITOR=false
F1T_READY_FILE={tmp_path}/ready.yaml
F1T_STACK_CMD="sleep 300"
F1T_STACK_MATCH="sleep 3[0-9]{{2}}$"
F1T_PLANNER_CMD="cat"
F1T_PLANNER_STDIN=fifo
F1T_LOGGER_CMD="{IGNORES_SIGINT}"
F1T_STOP_SIGINT_WAIT_S=1
F1T_STOP_SIGTERM_WAIT_S=1
""")
    (tmp_path / 'ready.yaml').write_text(
        'window_s: 0.5\nchecks:\n  - {name: scan, kind: topic_rate, topic: /scan, '
        'type: sensor_msgs/msg/LaserScan, min_hz: 1}\n')
    yield str(path)
    units = [f'{PREFIX}{n}.service' for n in ('stack', 'planner', 'logger', 'monitor')]
    subprocess.run(['systemctl', '--user', 'stop', *units], capture_output=True)
    subprocess.run(['systemctl', '--user', 'reset-failed', *units], capture_output=True)


def ctl(env_file, *args, capsys):
    code = cli.main([*args, '--env', env_file])
    out = json.loads(capsys.readouterr().out)
    return code, out


def test_start_status_stop(env_file, capsys):
    code, out = ctl(env_file, 'start', 'stack', capsys=capsys)
    assert code == 0 and out['ok'] and out['state'] == 'running'
    assert out['unit'] == f'{PREFIX}stack.service' and out['pid'] > 0

    code, out = ctl(env_file, 'start', 'stack', capsys=capsys)
    assert code == 1 and 'already running' in out['error']

    code, out = ctl(env_file, 'status', capsys=capsys)
    assert code == 0 and out['processes']['stack']['state'] == 'running'
    assert out['processes']['planner']['state'] == 'stopped'
    # No ROS in this env: READY is false and says why, within the budget.
    assert out['ready'] is False and 'scan' in out['failing']
    assert out['elapsed_s'] < 5.0

    code, out = ctl(env_file, 'stop', 'stack', capsys=capsys)
    assert code == 0 and out['signals'] == ['SIGINT'] and out['state'] == 'stopped'

    code, out = ctl(env_file, 'status', capsys=capsys)
    assert out['processes']['stack']['state'] == 'stopped'
    assert 'stack' in out['failing']


def test_stop_escalates_when_sigint_is_ignored(env_file, capsys):
    ctl(env_file, 'start', 'logger', capsys=capsys)
    code, out = ctl(env_file, 'stop', 'logger', capsys=capsys)
    assert code == 0 and out['signals'] == ['SIGINT', 'SIGTERM']
    assert out['state'] == 'stopped'


def test_stop_of_a_stopped_process_is_ok(env_file, capsys):
    code, out = ctl(env_file, 'stop', 'planner', capsys=capsys)
    assert code == 0 and out['already'] == 'stopped' and out['signals'] == []


ROS2_RUN_LIKE = """
import subprocess, sys
# Like `ros2 run`: waits for its child and does not forward SIGINT, because
# in a terminal Ctrl-C reaches the child directly.
p = subprocess.Popen([sys.executable, '-c', 'import time; time.sleep(300)'])
while p.returncode is None:
    try:
        p.communicate()
    except KeyboardInterrupt:
        pass
"""


def test_sigint_reaches_the_child_of_a_ros2_run_like_parent(env_file, capsys, tmp_path):
    script = tmp_path / 'ros2_run_like.py'
    script.write_text(ROS2_RUN_LIKE)
    with open(env_file, 'a') as f:
        f.write(f'F1T_STACK_CMD="python3 {script}"\n')
    code, out = ctl(env_file, 'start', 'stack', capsys=capsys)
    assert code == 0, out
    code, out = ctl(env_file, 'stop', 'stack', capsys=capsys)
    # SIGINT went to the whole process group, as Ctrl-C would: no escalation.
    assert code == 0 and out['signals'] == ['SIGINT'], out


def test_planner_reads_lines_from_its_fifo(env_file, capsys):
    code, out = ctl(env_file, 'send', 'planner', 'go', 'ahead', capsys=capsys)
    assert code == 1 and 'not running' in out['error']
    ctl(env_file, 'start', 'planner', capsys=capsys)
    code, out = ctl(env_file, 'send', 'planner', 'go', 'ahead', '2', 'meters', capsys=capsys)
    assert code == 0 and out['sent'] == 'go ahead 2 meters'
    deadline = time.monotonic() + 3
    lines = []
    while time.monotonic() < deadline:
        code, out = ctl(env_file, 'logs', 'planner', capsys=capsys)
        lines = out['lines']
        if 'go ahead 2 meters' in lines:
            break
        time.sleep(0.2)
    assert 'go ahead 2 meters' in lines
    # `cat` is still there: its stdin never reached EOF.
    code, out = ctl(env_file, 'status', capsys=capsys)
    assert out['processes']['planner']['state'] == 'running'
    ctl(env_file, 'stop', 'planner', capsys=capsys)


def test_send_takes_one_line_and_planner_log_follows_it(env_file, capsys):
    ctl(env_file, 'start', 'planner', capsys=capsys)
    code, out = ctl(env_file, 'send', 'planner', 'two\nlines', capsys=capsys)
    assert code == 1 and 'one line only' in out['error']
    code, out = ctl(env_file, 'send', 'planner', 'x' * 501, capsys=capsys)
    assert code == 1 and 'at most 500' in out['error']
    code, out = ctl(env_file, 'send', 'planner', 'turn left at the wall', capsys=capsys)
    assert code == 0 and isinstance(out['cursor'], int)
    lines = []
    deadline = time.monotonic() + 3
    while time.monotonic() < deadline and 'turn left at the wall' not in lines:
        code, log = ctl(env_file, 'planner-log', '--since', str(out['cursor']), capsys=capsys)
        lines = log['lines']
        time.sleep(0.2)
    # `cat` echoes it: only what came after the cursor.
    assert lines == ['turn left at the wall'] and log['running'] is True
    code, log2 = ctl(env_file, 'planner-log', '--since', str(log['cursor']), capsys=capsys)
    assert log2['lines'] == [] and log2['cursor'] == log['cursor']
    ctl(env_file, 'stop', 'planner', capsys=capsys)
    code, log3 = ctl(env_file, 'planner-log', capsys=capsys)
    assert log3['running'] is False and log3['lines'] == []


def test_planner_log_says_whether_the_planner_is_plan_only(env_file, capsys):
    code, log = ctl(env_file, 'planner-log', capsys=capsys)
    assert log['running'] is False and log['plan_only'] is None
    ctl(env_file, 'start', 'planner', capsys=capsys)        # `cat`: no --dry-run
    code, log = ctl(env_file, 'planner-log', capsys=capsys)
    assert log['running'] is True and log['plan_only'] is False
    ctl(env_file, 'stop', 'planner', capsys=capsys)
    with open(env_file, 'a') as f:
        f.write("F1T_PLANNER_CMD='python3 -c \"import time; time.sleep(300)\" "
                "llm_planner_node --dry-run'\n")
    ctl(env_file, 'start', 'planner', capsys=capsys)
    code, log = ctl(env_file, 'planner-log', capsys=capsys)
    assert log['plan_only'] is True
    ctl(env_file, 'stop', 'planner', capsys=capsys)


def test_a_process_that_dies_is_crashed(env_file, capsys, tmp_path):
    with open(env_file, 'a') as f:
        f.write('F1T_STACK_CMD="sh -c \'sleep 2; exit 3\'"\n')
    code, out = ctl(env_file, 'start', 'stack', capsys=capsys)
    assert code == 0 and out['state'] == 'running'
    time.sleep(2.5)
    code, out = ctl(env_file, 'status', capsys=capsys)
    st = out['processes']['stack']
    assert st['state'] == 'crashed' and st['exit_status'] == 3
    # stop clears it.
    code, out = ctl(env_file, 'stop', 'stack', capsys=capsys)
    code, out = ctl(env_file, 'status', capsys=capsys)
    assert out['processes']['stack']['state'] == 'stopped'


def test_the_command_reaches_bash_unexpanded_by_systemd(env_file, capsys):
    # systemd expands "${VAR}", "$$" and "%n" in ExecStart; the command must
    # only be expanded by bash, after the env file is sourced.
    with open(env_file, 'a') as f:
        f.write('F1T_MARK=from_env_file\n'
                "F1T_STACK_CMD='sh -c \"echo mark=${F1T_MARK} pct=%n; exec sleep 300\"'\n")
    code, out = ctl(env_file, 'start', 'stack', capsys=capsys)
    assert code == 0, out
    deadline = time.monotonic() + 3
    while time.monotonic() < deadline:
        code, out = ctl(env_file, 'logs', 'stack', capsys=capsys)
        if any('mark=' in line for line in out['lines']):
            break
        time.sleep(0.2)
    assert 'mark=from_env_file pct=%n' in out['lines']
    ctl(env_file, 'stop', 'stack', capsys=capsys)


def test_units_and_the_cli_see_the_same_environment(env_file, capsys):
    with open(env_file, 'a') as f:
        f.write('F1T_MARK=from_env_file\n'
                "F1T_STACK_CMD='python3 -c \"import os, json, time; "
                'print(chr(69) + json.dumps(sorted(os.environ)), flush=True); '
                "time.sleep(300)\"'\n")
    code, out = ctl(env_file, 'start', 'stack', capsys=capsys)
    assert code == 0, out
    deadline = time.monotonic() + 3
    keys = None
    while keys is None and time.monotonic() < deadline:
        code, out = ctl(env_file, 'logs', 'stack', capsys=capsys)
        for line in out['lines']:
            if line.startswith('E['):
                keys = set(json.loads(line[1:]))
        time.sleep(0.2)
    ctl(env_file, 'stop', 'stack', capsys=capsys)
    cli_keys = set(config.source_env_file(env_file))
    unit_only = {'F1T_UNIT_NAME', 'F1T_UNIT_CMD', 'F1T_UNIT_FIFO', 'PYTHONUNBUFFERED',
                 'RCUTILS_LOGGING_BUFFERED_STREAM', 'SHLVL', 'PWD', '_', 'OLDPWD'}
    assert keys - unit_only == cli_keys - unit_only
    assert 'F1T_MARK' in keys


def test_a_process_that_exits_right_away_fails_the_start(env_file, capsys):
    with open(env_file, 'a') as f:
        f.write('F1T_STACK_CMD="sh -c \'echo boom; exit 4\'"\n')
    code, out = ctl(env_file, 'start', 'stack', capsys=capsys)
    assert code == 1 and out['state'] == 'crashed' and out['exit_status'] == 4
    assert any('boom' in line for line in out['log_tail'])


def test_start_refuses_a_copy_started_by_hand(env_file, capsys):
    by_hand = subprocess.Popen(['sleep', '301'])
    try:
        time.sleep(0.2)
        code, out = ctl(env_file, 'start', 'stack', capsys=capsys)
        assert code == 1 and 'outside f1tenth_ctl' in out['error']
        assert by_hand.pid in out['pids']
        code, out = ctl(env_file, 'status', capsys=capsys)
        st = out['processes']['stack']
        assert st['state'] == 'running' and st['managed'] is False
        assert by_hand.pid in st['seen']['pids']
        # stop applies the same ladder to it.
        code, out = ctl(env_file, 'stop', 'stack', capsys=capsys)
        assert out['unmanaged']['pids'] == [by_hand.pid]
        assert by_hand.wait(timeout=5) is not None
    finally:
        if by_hand.poll() is None:
            by_hand.kill()


def test_killall_dry_run_signals_nothing(env_file, capsys):
    ctl(env_file, 'start', 'stack', capsys=capsys)
    code, out = ctl(env_file, 'killall', '--dry-run', capsys=capsys)
    assert code == 0 and out['dry_run'] and out['units']['stack'] == 'running'
    code, out = ctl(env_file, 'status', capsys=capsys)
    assert out['processes']['stack']['state'] == 'running'
    ctl(env_file, 'stop', 'stack', capsys=capsys)


def test_bad_env_file_exits_2_with_json(tmp_path, capsys):
    bad = tmp_path / 'bad.env'
    bad.write_text('F1T_STACK_CMD=x\n')
    code = cli.main(['status', '--env', str(bad)])
    out = json.loads(capsys.readouterr().out)
    assert code == 2 and out['ok'] is False and 'F1T_PLANNER_CMD' in out['error']
