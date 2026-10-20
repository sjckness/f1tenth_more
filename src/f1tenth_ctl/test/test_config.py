"""The env file: sourcing, the minimal base environment, process specs."""

import os
import re

from f1tenth_ctl import config
import pytest

PKG = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def _write(tmp_path, text, name='t.env'):
    path = tmp_path / name
    path.write_text(text)
    return str(path)


MINIMAL = '\n'.join(f'F1T_{n.upper()}_CMD="sleep 1"' for n in config.MANAGED) + '\n'


def test_sourcing_exports_plain_assignments(tmp_path):
    env = config.source_env_file(_write(tmp_path, 'FOO=bar\nBAZ="a b"\n'))
    assert env['FOO'] == 'bar' and env['BAZ'] == 'a b'
    assert env['F1T_CTL_ENV'].endswith('t.env')


def test_sourcing_starts_from_a_minimal_environment(tmp_path, monkeypatch):
    monkeypatch.setenv('AMENT_PREFIX_PATH', '/somewhere/else')
    monkeypatch.setenv('F1T_LEAK', 'x')
    env = config.source_env_file(_write(tmp_path, 'FOO=1\n'))
    assert 'AMENT_PREFIX_PATH' not in env and 'F1T_LEAK' not in env
    assert env['PATH'].endswith('/usr/bin:/sbin:/bin')
    assert env['HOME'] == os.environ['HOME']


def test_a_failing_env_file_is_an_error(tmp_path):
    path = _write(tmp_path, 'source /nonexistent/setup.bash || return 1\n')
    with pytest.raises(config.ConfigError, match='exit 97'):
        config.source_env_file(path)
    with pytest.raises(config.ConfigError, match='not found'):
        config.source_env_file(str(tmp_path / 'missing.env'))


def test_settings_read_the_process_specs(tmp_path):
    path = _write(tmp_path, MINIMAL + 'F1T_STACK_NODES="/a /b"\nF1T_PLANNER_STDIN=fifo\n'
                  'ROS_DISCOVERY_SERVER=127.0.0.1:11811\nF1T_STATE_DIR=/tmp/x\n')
    s = config.load(path)
    assert s.procs['stack'].nodes == ['/a', '/b']
    assert s.procs['planner'].stdin_fifo and not s.procs['logger'].stdin_fifo
    assert s.procs['stack'].unit == 'f1t-stack.service'
    assert s.discovery_address == ('127.0.0.1', 11811)
    assert s.state_dir == '/tmp/x'
    assert s.ready_file == str(tmp_path / 'ready.yaml')
    assert s.sigint_wait_s == 10.0 and s.sigterm_wait_s == 5.0
    assert s.estop_topic == '/safety_stop' and s.estop_service_timeout_s == 0.5


def test_unit_prefix_moves_every_unit(tmp_path):
    s = config.load(_write(tmp_path, MINIMAL + 'F1T_UNIT_PREFIX=f1t-test-\n'))
    assert {p.unit for p in s.procs.values()} == {
        'f1t-test-stack.service', 'f1t-test-planner.service', 'f1t-test-logger.service'}


def test_missing_command_is_an_error(tmp_path):
    with pytest.raises(config.ConfigError, match='F1T_LOGGER_CMD'):
        config.load(_write(tmp_path, 'F1T_STACK_CMD=a\nF1T_PLANNER_CMD=b\n'))


def test_no_discovery_server_configured(tmp_path):
    s = config.load(_write(tmp_path, MINIMAL))
    assert s.discovery_server is None and s.discovery_address is None


@pytest.mark.parametrize('name', ['ctl.env', 'sim_linus.env'])
def test_shipped_env_files_define_every_process(name):
    with open(os.path.join(PKG, 'config', name)) as f:
        text = f.read()
    for proc in config.MANAGED:
        assert re.search(rf'^F1T_{proc.upper()}_CMD=".+"', text, re.M), proc
        assert re.search(rf'^F1T_{proc.upper()}_NODES="/', text, re.M), proc
    assert re.search(r'^ROS_DISCOVERY_SERVER=127\.0\.0\.1:11811$', text, re.M)
    # Every source line fails the whole file, so a missing workspace is loud.
    for line in re.findall(r'^source .*$', text, re.M):
        assert line.endswith('|| return 1'), line


def test_car_env_matches_the_stack_defaults():
    with open(os.path.join(PKG, 'config', 'ctl.env')) as f:
        text = f.read()
    stack = re.search(r'^F1T_STACK_CMD="(.+)"', text, re.M).group(1)
    # The stack as started by hand on the car: no overrides of its defaults.
    assert stack == 'ros2 launch f1tenth_bringup supervisor_bringup.launch.py'
    logger = re.search(r'^F1T_LOGGER_CMD="(.+)"', text, re.M).group(1)
    assert 'enable_mission_logger:=true' in logger
