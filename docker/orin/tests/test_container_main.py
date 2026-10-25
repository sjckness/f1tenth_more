"""
f1tenth_container_main.py and healthcheck.sh, off the car (linus).

The pure helpers, the refusals, the harness role, and one full run with
dummy processes: the container's own ctl env file (ctl_container.env) with
the stack, planner and logger replaced by sleeps, a real Fast DDS discovery
server on a free port (domain 91), the gateway in --mock from the gateway's
venv, then the healthcheck, a gateway crash and restart, and the ordered
stop. Needs ROS Jazzy (/opt/ros/jazzy) and a gateway venv with fastapi
(F1T_TEST_GATEWAY_VENV, default ~/.local/share/f1tenth_gateway/venv); the
full run is skipped without them.
"""

import fcntl
import json
import os
import queue
import shlex
import signal
import socket
import subprocess
import sys
import threading
import time
import urllib.request

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
ORIN = os.path.dirname(HERE)
REPO = os.path.dirname(os.path.dirname(ORIN))
MAIN = os.path.join(ORIN, 'container', 'f1tenth_container_main.py')
HEALTH = os.path.join(ORIN, 'container', 'healthcheck.sh')
CTL_ENV = os.path.join(REPO, 'src', 'f1tenth_ctl', 'config', 'ctl_container.env')
CTL_BIN = os.path.join(REPO, 'src', 'f1tenth_ctl', 'bin', 'f1tenth_ctl')
READY = os.path.join(REPO, 'src', 'f1tenth_ctl', 'config', 'ready.yaml')
VENV = os.path.expanduser(os.environ.get('F1T_TEST_GATEWAY_VENV',
                                         '~/.local/share/f1tenth_gateway/venv'))
TOKEN = 'x' * 40
HAVE_ROS = os.path.isfile('/opt/ros/jazzy/setup.bash')
HAVE_VENV = subprocess.run([os.path.join(VENV, 'bin', 'python'), '-c', 'import fastapi, uvicorn'],
                           capture_output=True).returncode == 0 if os.path.isdir(VENV) else False
sys.dont_write_bytecode = True       # no __pycache__ in docker/orin (the build context)
sys.path.insert(0, os.path.join(ORIN, 'container'))
import f1tenth_container_main as main_mod  # noqa: E402, I100


# ---- pure helpers -------------------------------------------------------------------

def test_the_token_file_is_read_like_systemd_does(tmp_path):
    path = tmp_path / 'env'
    path.write_text('# the gateway\nF1T_GATEWAY_TOKEN="abc def"\nexport OTHER=1\n'
                    "QUOTED='q'\n  SPACED = v \nbad line\n1BAD=x\n=x\n")
    assert main_mod.read_env_file(str(path)) == {
        'F1T_GATEWAY_TOKEN': 'abc def', 'OTHER': '1', 'QUOTED': 'q', 'SPACED': 'v'}


def test_a_path_is_on_the_longest_mount_that_contains_it(tmp_path):
    points = ['/', '/home', '/home/fabiocar/f1tenth_archive', '/scratch']
    assert main_mod.mount_of('/home/fabiocar/f1tenth_archive/x', points) == (
        '/home/fabiocar/f1tenth_archive')
    assert main_mod.mount_of('/home/fabiocar/.ros', points) == '/home'
    assert main_mod.mount_of('/opt/f1tenth_ws', points) == '/'
    assert main_mod.mount_of('/home2/x', points) == '/'
    assert main_mod._unescape('/mnt/with\\040space') == '/mnt/with space'
    assert '/proc' in main_mod.mount_points()


def test_bound_ports_reads_proc_net_tables(tmp_path):
    table = tmp_path / 'udp'
    table.write_text(
        '  sl  local_address rem_address   st tx_queue rx_queue tr tm->when retrnsmt   uid  '
        'timeout inode\n'
        '   1: 0100007F:2E23 00000000:0000 07 00000000:00000000 00:00000000 00000000  1004 '
        '       0 4242 2 0000000000000000 0\n'
        '   2: 0100007F:2242 00000000:0000 0A 00000000:00000000 00:00000000 00000000  1004 '
        '       0 4343 1 0000000000000000 100 0 0 10 0\n')
    assert main_mod.bound_ports([str(table)], 11811) == [4242]
    assert main_mod.bound_ports([str(table)], 8770, '0A') == [4343]
    assert main_mod.bound_ports([str(table)], 8770, '01') == []
    assert main_mod.bound_ports([str(tmp_path / 'missing')], 11811) == []


def test_live_shm_is_told_from_stale(tmp_path):
    live = tmp_path / 'fastrtps_port7411_el'
    stale = tmp_path / 'fastrtps_0123456789abcdef_el'
    other = tmp_path / 'fastrtps_port7411'
    for p in (live, stale, other):
        p.write_text('')
    fd = os.open(live, os.O_RDONLY)
    try:
        fcntl.flock(fd, fcntl.LOCK_EX)      # a live participant holds its lock
        found_live, found_stale = main_mod.live_shm(str(tmp_path))
    finally:
        os.close(fd)
    assert found_live == [str(live)] and found_stale == [str(stale)]


def test_the_gateway_port_follows_its_arguments():
    assert main_mod.gateway_port([]) == 8770
    assert main_mod.gateway_port(['--mock', '--port', '18770']) == 18770
    assert main_mod.gateway_port(['--port=18771']) == 18771


# ---- healthcheck.sh ---------------------------------------------------------------

def _free_port(kind):
    with socket.socket(socket.AF_INET, kind) as s:
        s.bind(('127.0.0.1', 0))
        return s.getsockname()[1]


def _health(ds_port, gw_port, role='car'):
    env = dict(os.environ, F1T_CONTAINER_ROLE=role, F1T_HEALTH_DS_PORT=str(ds_port),
               F1T_HEALTH_GATEWAY_PORT=str(gw_port))
    return subprocess.run(['bash', HEALTH], env=env, capture_output=True, text=True, timeout=30)


def test_the_healthcheck_needs_both_ports_held_here():
    ds_port, gw_port = _free_port(socket.SOCK_DGRAM), _free_port(socket.SOCK_STREAM)
    proc = _health(ds_port, gw_port)
    assert proc.returncode == 1 and 'no discovery server' in proc.stdout
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as udp:
        udp.bind(('127.0.0.1', ds_port))
        proc = _health(ds_port, gw_port)
        assert proc.returncode == 1 and 'gateway does not listen' in proc.stdout
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as tcp:
            tcp.bind(('127.0.0.1', gw_port))
            tcp.listen(1)
            proc = _health(ds_port, gw_port)
            assert proc.returncode == 0, proc.stdout
            assert f'pid {os.getpid()}' in proc.stdout
    assert _health(ds_port, gw_port, role='harness').returncode == 0


# ---- the process, without ROS processes ----------------------------------------------

def _home(tmp_path, token=TOKEN):
    home = tmp_path / 'home'
    (home / '.config' / 'f1tenth_gateway').mkdir(parents=True)
    if token is not None:
        env = home / '.config' / 'f1tenth_gateway' / 'env'
        env.write_text(f'F1T_GATEWAY_TOKEN={token}\n')
        env.chmod(0o600)
    return home


def _main_env(home, ctl_env, **extra):
    env = {'HOME': str(home), 'USER': os.environ.get('USER', 'andreas'),
           'PATH': '/usr/local/bin:/usr/bin:/bin', 'LANG': 'C.UTF-8',
           'F1T_CTL_ENV': ctl_env, 'F1T_GATEWAY_VENV': VENV}
    env.update(extra)
    return env


def _run_main(env, timeout=60):
    return subprocess.run([sys.executable, MAIN], env=env, capture_output=True, text=True,
                          timeout=timeout)


def test_the_harness_role_starts_nothing_and_stops_on_sigterm(tmp_path):
    env = _main_env(_home(tmp_path), CTL_ENV, F1T_CONTAINER_ROLE='harness')
    proc = subprocess.Popen([sys.executable, MAIN], env=env, stdout=subprocess.PIPE, text=True)
    time.sleep(1.0)
    proc.send_signal(signal.SIGTERM)
    out, _ = proc.communicate(timeout=10)
    assert proc.returncode == 0 and 'nothing to start' in out and 'SIGTERM: exit' in out


def test_a_missing_env_file_is_a_configuration_error(tmp_path):
    proc = _run_main(_main_env(_home(tmp_path), str(tmp_path / 'none.env')))
    assert proc.returncode == main_mod.EXIT_CONFIG and 'no such file' in proc.stdout


@pytest.mark.skipif(not HAVE_ROS, reason='needs /opt/ros/jazzy (the env file sources it)')
def test_preflight_refuses_without_mounts_or_token(tmp_path):
    home = _home(tmp_path, token=None)
    env = _main_env(home, CTL_ENV, F1T_REQUIRED_MOUNTS=str(home / 'f1tenth_archive_mount'))
    proc = _run_main(env)
    assert proc.returncode == main_mod.EXIT_PREFLIGHT, proc.stdout
    assert 'missing (expected a mount)' in proc.stdout
    assert 'F1T_GATEWAY_TOKEN' in proc.stdout and 'refusing to start' in proc.stdout
    # Nothing was started: no runner record, no discovery server.
    assert not (home / '.local/state/f1tenth_ctl/proc').exists()


# ---- the full run (mock gateway, dummy processes, a real discovery server) ----------------

class Reader:
    """Collect a process's stdout lines in a thread."""

    def __init__(self, stream):
        self.lines, self.q = [], queue.Queue()
        self.thread = threading.Thread(target=self._read, args=(stream,), daemon=True)
        self.thread.start()

    def _read(self, stream):
        for line in stream:
            self.q.put(line.rstrip('\n'))

    def wait_for(self, text, timeout, count=1):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            try:
                self.lines.append(self.q.get(timeout=0.2))
            except queue.Empty:
                if not self.thread.is_alive():
                    return False        # the process is gone, its output read
                continue
            if sum(text in line for line in self.lines) >= count:
                return True
        return False


def _wait(pred, timeout):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if pred():
            return True
        time.sleep(0.25)
    return pred()


def _pids_with(marker):
    out = []
    for name in os.listdir('/proc'):
        if name.isdigit():
            try:
                with open(f'/proc/{name}/cmdline', 'rb') as f:
                    if marker.encode() in f.read().split(b'\0'):
                        out.append(int(name))
            except OSError:
                pass
    return out


@pytest.mark.skipif(not (HAVE_ROS and HAVE_VENV), reason='needs ROS Jazzy and a gateway venv')
def test_the_container_main_runs_the_car_role_with_dummies(tmp_path):
    home = _home(tmp_path)
    ds_port, gw_port = _free_port(socket.SOCK_DGRAM), _free_port(socket.SOCK_STREAM)
    ctl_env = tmp_path / 'test_ctl.env'
    ctl_env.write_text(f""". {shlex.quote(CTL_ENV)} || return 1
# Isolated: own discovery server port, domain 91; dummies for the processes, and
# match patterns that can only ever find these dummies (stop signals what matches).
ROS_DISCOVERY_SERVER=127.0.0.1:{ds_port}
ROS_DOMAIN_ID=91
F1T_SHELL_REF=off
F1T_READY_FILE={shlex.quote(READY)}
F1T_STACK_CMD="sh -c '(setsid sleep 3901 &); exec sleep 3900'"
F1T_STACK_NODES=/w1_fake_stack_node
F1T_STACK_MATCH="sleep 390[0-9]$"
F1T_PLANNER_CMD="sleep 3902"
F1T_PLANNER_MATCH="sleep 3902$"
F1T_LOGGER_CMD="sleep 3903"
F1T_LOGGER_MATCH="sleep 3903$"
""")
    env = _main_env(home, str(ctl_env), F1T_GATEWAY_ARGS=f'--mock --port {gw_port}',
                    F1T_REQUIRED_MOUNTS='', F1T_PREFLIGHT='warn', F1T_AUTOSTART='stack')
    proc = subprocess.Popen([sys.executable, MAIN], env=env, stdout=subprocess.PIPE,
                            stderr=subprocess.STDOUT, text=True)
    log = Reader(proc.stdout)
    ctl_cli = dict(env, PATH='/usr/bin:/bin')
    try:
        assert log.wait_for(' up', 120), '\n'.join(log.lines)
        text = '\n'.join(log.lines)
        assert 'backend proc (F1T_PROCESS_BACKEND=proc)' in text
        assert 'reset: stopped stack,planner,logger,discovery,monitor' in text
        assert f'discovery server 127.0.0.1:{ds_port}' in text
        assert 'autostart stack: ok=True state=running' in text
        # Healthy once the gateway listens.
        assert _wait(lambda: _health(ds_port, gw_port).returncode == 0, 60), \
            _health(ds_port, gw_port).stdout
        req = urllib.request.Request(f'http://127.0.0.1:{gw_port}/api/status',
                                     headers={'Authorization': f'Bearer {TOKEN}'})
        with urllib.request.urlopen(req, timeout=10) as resp:
            assert resp.status == 200 and isinstance(json.load(resp), dict)
        # f1tenth_ctl from "a docker exec": the same processes, managed.
        st = json.loads(subprocess.run([CTL_BIN, 'status', '--env', str(ctl_env)], env=ctl_cli,
                                       capture_output=True, text=True, timeout=60).stdout)
        assert st['processes']['stack']['state'] == 'running'
        assert st['processes']['stack']['managed'] is True
        assert st['processes']['discovery']['managed'] is True
        assert _pids_with('3900') and _pids_with('3901')
        # The gateway dies: restarted (Restart=always, 3 s).
        line = next(x for x in log.lines if 'gateway started (pid' in x)
        gw_pid = int(line.split('(pid ')[1].split(')')[0])
        os.killpg(gw_pid, signal.SIGKILL)
        assert log.wait_for('gateway started (pid', 30, count=2), '\n'.join(log.lines)
        assert _wait(lambda: _health(ds_port, gw_port).returncode == 0, 60)
        # The ordered stop.
        proc.send_signal(signal.SIGTERM)
        assert proc.wait(120) == 0
        log.thread.join(5)
        while not log.q.empty():
            log.lines.append(log.q.get())
        text = '\n'.join(log.lines)
        print(text)                      # the run's log, with -s
        order = [text.index(f'stop {name}:') for name in
                 ('planner', 'logger', 'monitor', 'stack', 'gateway', 'discovery', 'runners')]
        assert order == sorted(order), text
        assert 'stopped (clean)' in text, text
        assert _pids_with('3900') == [] and _pids_with('3901') == []
        assert _health(ds_port, gw_port).returncode == 1
        records = home / '.local/state/f1tenth_ctl/proc'
        assert list(records.rglob('*.json')) == []
    finally:
        if proc.poll() is None:
            proc.send_signal(signal.SIGTERM)
            try:
                proc.wait(60)
            except subprocess.TimeoutExpired:
                proc.kill()
        # Whatever is left of this run's units (never anything else).
        sys.path.insert(0, os.path.join(REPO, 'src', 'f1tenth_ctl'))
        from f1tenth_ctl import config, procunits
        procunits.sweep(config.settings_from_env(
            str(ctl_env), {**env, 'F1T_STATE_DIR': str(home / '.local/state/f1tenth_ctl'),
                           'F1T_STACK_CMD': 'x', 'F1T_PLANNER_CMD': 'x',
                           'F1T_LOGGER_CMD': 'x'}))
