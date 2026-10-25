"""
The car container's host side, on linus with fake docker/systemctl/ss/getent on PATH.

Covers docker/orin/host/f1tenth_ctl_car (the Telegram wrapper), stack_mode.sh
(--dry-run and status), install.sh / uninstall.sh (temporary HOME and state),
bashrc_banner.sh, scripts/orin_stack_guard.sh's command line,
scripts/docker_orin.sh (pin, up --dry-run and its refusals, status, down) and
compose.yaml's rendering. Nothing here talks to the real Docker daemon or the
user's systemd; the compose rendering only parses.
"""

import json
import os
import shutil
import socket
import stat
import subprocess

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
ORIN = os.path.dirname(HERE)
REPO = os.path.dirname(os.path.dirname(ORIN))
HOST = os.path.join(ORIN, 'host')
WRAPPER = os.path.join(HOST, 'f1tenth_ctl_car')
STACK_MODE = os.path.join(HOST, 'stack_mode.sh')
INSTALL = os.path.join(HOST, 'install.sh')
UNINSTALL = os.path.join(HOST, 'uninstall.sh')
DOCKER_ORIN = os.path.join(REPO, 'scripts', 'docker_orin.sh')
GUARD = os.path.join(REPO, 'scripts', 'orin_stack_guard.sh')
IMAGE = 'f1tenth:jazzy-orin-test'

FAKE_DOCKER = r"""#!/bin/bash
# Fake docker: behaviour from FAKE_* variables, every call logged. Failures
# print what Docker 29 (29.5.3, measured on linus) prints; FAKE_DOCKER_STYLE=28
# gives the texts before it. Docker 29's failed `inspect -f` prints an empty
# line on stdout before the error.
echo "docker $*" >> "${FAKE_LOG:-/dev/null}"
state=${FAKE_STATE:-absent}
if [ -n "${FAKE_STATE_FILE:-}" ] && [ -s "$FAKE_STATE_FILE" ]; then
  state=$(cat "$FAKE_STATE_FILE")
fi
role=${FAKE_ROLE:-car}
style=${FAKE_DOCKER_STYLE:-29}
sub=$1; shift
inspect_fail() {
  if [ "$style" = 29 ]; then echo; fi
  echo "$1" >&2
  exit 1
}
# Docker itself does not answer (daemon down, no permission): every call fails.
if [ -n "${FAKE_DOCKER_ERR:-}" ]; then
  [ "$sub" = inspect ] && inspect_fail "$FAKE_DOCKER_ERR"
  echo "$FAKE_DOCKER_ERR" >&2
  exit 1
fi
fmt=""
if [ "$sub" = image ]; then sub="image_$1"; shift; fi
[ "${1:-}" = --type ] && shift 2
[ "${1:-}" = -f ] && { fmt=$2; shift 2; }
case "$sub" in
  inspect)
    # FAKE_INSPECT_ERR: inspect alone fails (the container exists for ps).
    if [ -n "${FAKE_INSPECT_ERR:-}" ]; then inspect_fail "$FAKE_INSPECT_ERR"; fi
    if [ "$state" = absent ]; then
      if [ "$style" = 29 ]; then inspect_fail "error: no such object: $1"; fi
      inspect_fail "Error: No such object: $1"
    fi
    case "$fmt" in
      *'.State.Status}}{{println}}'*)
        printf '%s\nPATH=/usr/bin\nF1T_CONTAINER_ROLE=%s\n' "$state" "$role" ;;
      '{{.State.Status}}') echo "$state" ;;
      *State.Running*) [ "$state" = running ] && echo true || echo false ;;
      *Health.Status*) echo "${FAKE_HEALTH:-healthy}" ;;
      *'range .Config.Env'*) printf 'PATH=/usr/bin\nF1T_CONTAINER_ROLE=%s\n' "$role" ;;
      *'.Id'*) echo c0ffee0123456789 ;;
      *) echo "${FAKE_IMAGE_REF:-f1tenth:jazzy-orin-test}" ;;
    esac ;;
  image_inspect)
    [ -n "${FAKE_IMAGE_ID:-}" ] || inspect_fail "Error response from daemon: No such image: $1"
    case "$fmt" in
      *'.Id'*) echo "$FAKE_IMAGE_ID" ;;
      *'index .Config.Labels'*)
        key=$(echo "$fmt" | sed -n 's/.*Labels "\([^"]*\)".*/\1/p')
        sed -n "s/^$key=//p" "${FAKE_LABELS:-/dev/null}" ;;
    esac ;;
  exec) echo "FAKE-EXEC $*"; exit "${FAKE_EXEC_RC:-0}" ;;
  create) echo fakecid ;;
  cp) cp "${FAKE_CAR_SYNC:?}" "$2" ;;
  ps)
    # `ps -a --filter name=f1tenth_car --format '{{.Names}}[ {{.State}}]'`: the
    # name filter matches substrings, so FAKE_PS_OTHERS (name:state ...) come too.
    with_state=0
    case "$*" in *'{{.State}}'*) with_state=1 ;; esac
    for pair in ${FAKE_PS_OTHERS:-}; do
      if [ "$with_state" = 1 ]; then echo "${pair%%:*} ${pair#*:}"; else echo "${pair%%:*}"; fi
    done
    if [ "$state" != absent ]; then
      if [ "$with_state" = 1 ]; then echo "f1tenth_car $state"; else echo f1tenth_car; fi
    fi ;;
  version) echo 29.5.3 ;;
  *) ;;
esac
exit 0
"""

DOCKER29_ABSENT_DAEMON = ('failed to connect to the docker API at unix:///var/run/docker.sock; '
                          'check if the path is correct and if the daemon is running: dial unix '
                          '/var/run/docker.sock: connect: no such file or directory')
DOCKER_REFUSED_DAEMON = ('Cannot connect to the Docker daemon at unix:///var/run/docker.sock. '
                         'Is the docker daemon running?')
DOCKER29_NO_PERMISSION = ('permission denied while trying to connect to the docker API at '
                          'unix:///var/run/docker.sock')
DOCKER28_NO_PERMISSION = ('permission denied while trying to connect to the Docker daemon socket '
                          'at unix:///var/run/docker.sock: Get "http://%2Fvar%2Frun%2Fdocker.sock/'
                          'v1.47/containers/json": dial unix /var/run/docker.sock: connect: '
                          'permission denied')
DOCKER_OLD_NO_PERMISSION = ('Got permission denied while trying to connect to the Docker daemon '
                            'socket at unix:///var/run/docker.sock: Get "http://%2Fvar%2Frun%2F'
                            'docker.sock/v1.24/containers/json": dial unix /var/run/docker.sock: '
                            'connect: permission denied')

FAKE_SYSTEMCTL = r"""#!/bin/bash
# Fake `systemctl --user`. start/stop are remembered in FAKE_UNIT_DIR (over
# FAKE_ACTIVE) and, as in systemd, `start` of an active unit does nothing. The
# car unit's start (its ExecStart) leaves the container FAKE_START_STATE
# (running) in FAKE_STATE_FILE, its stop (ExecStop) leaves it absent.
echo "systemctl $*" >> "${FAKE_LOG:-/dev/null}"
[ "$1" = --user ] && shift
verb=$1; shift
[ "${1:-}" = --quiet ] && shift
unit=${1:-}
sd=${FAKE_UNIT_DIR:-}
active() {
  if [ -n "$sd" ] && [ -f "$sd/$1" ]; then
    [ "$(cat "$sd/$1")" = active ]
    return
  fi
  case " ${FAKE_ACTIVE:-} " in *" $1 "*) return 0 ;; esac
  return 1
}
known() {
  case " ${FAKE_KNOWN:-} " in *" $1 "*) return 0 ;; esac
  [ -e "$HOME/.config/systemd/user/$1" ]
}
case "$verb" in
  is-active)
    if active "$unit"; then echo active; exit 0; fi
    echo inactive; exit 3 ;;
  is-enabled)
    case " ${FAKE_ENABLED:-} " in *" $unit "*) echo enabled; exit 0 ;; esac
    if known "$unit"; then echo disabled; exit 1; fi
    echo not-found; exit 4 ;;
  cat) known "$unit"; exit ;;
  start)
    active "$unit" && exit 0
    if [ -n "$sd" ]; then echo active > "$sd/$unit"; fi
    if [ "$unit" = f1tenth-car-container.service ] && [ -n "${FAKE_STATE_FILE:-}" ]; then
      echo "${FAKE_START_STATE:-running}" > "$FAKE_STATE_FILE"
    fi
    exit 0 ;;
  stop)
    # FAKE_STOP_STUCK: the unit stays active; FAKE_STOP_RC: the stop's exit code.
    if [ -n "$sd" ] && [ -z "${FAKE_STOP_STUCK:-}" ]; then echo inactive > "$sd/$unit"; fi
    if [ "$unit" = f1tenth-car-container.service ] && [ -n "${FAKE_STATE_FILE:-}" ]; then
      echo absent > "$FAKE_STATE_FILE"
    fi
    exit "${FAKE_STOP_RC:-0}" ;;
  *) exit 0 ;;
esac
"""

FAKE_GETENT = r"""#!/bin/bash
for pair in ${FAKE_GROUPS:-video:44 render:104 dialout:20 input:106 zed:1005 jtop:1003}; do
  if [ "${pair%%:*}" = "$2" ]; then echo "$2:x:${pair#*:}:"; exit 0; fi
done
exit 2
"""

FAKE_NATIVE_CTL = r"""#!/bin/bash
echo "native-ctl $*" >> "${FAKE_LOG:-/dev/null}"
printf '{"ok": true, "side": "native", "args": "%s"}\n' "$*"
exit "${FAKE_NATIVE_RC:-0}"
"""


def script(path, body):
    with open(path, 'w') as f:
        f.write(body)
    os.chmod(path, 0o755)
    return str(path)


class Fake:
    """Paths and the environment of one fake world."""

    def read_log(self):
        return self.log.read_text() if self.log.exists() else ''


@pytest.fixture
def fake(tmp_path):
    """Build a fake bin directory first on PATH, a temporary HOME and state root."""
    fb = tmp_path / 'bin'
    fb.mkdir()
    script(fb / 'docker', FAKE_DOCKER)
    script(fb / 'systemctl', FAKE_SYSTEMCTL)
    script(fb / 'getent', FAKE_GETENT)
    script(fb / 'ss', '#!/bin/bash\nprintf "%s" "${FAKE_SS:-}"\n')
    home = tmp_path / 'home'
    home.mkdir()
    (home / '.bashrc').write_text('# the user\'s own bashrc\nalias ll="ls -l"\n')
    native = script(tmp_path / 'native_ctl', FAKE_NATIVE_CTL)
    log = tmp_path / 'log'
    (tmp_path / 'units').mkdir()
    env = {k: v for k, v in os.environ.items()
           if not k.startswith(('FAKE_', 'F1T_')) and k != 'DOCKER_HOST'}
    env.update(PATH=f'{fb}:{env["PATH"]}', HOME=str(home), FAKE_LOG=str(log),
               FAKE_STATE_FILE=str(tmp_path / 'ctr_state'), FAKE_UNIT_DIR=str(tmp_path / 'units'),
               F1T_NATIVE_CTL=native, F1T_STATE=str(home / 'state'))
    f = Fake()
    f.tmp, f.bin, f.home, f.env, f.log = tmp_path, fb, home, env, log
    f.state = home / 'state'
    return f


def run(cmd, env, **kw):
    return subprocess.run(cmd, capture_output=True, text=True, env=env, timeout=60, **kw)


def with_env(base, **extra):
    env = dict(base)
    env.update(extra)
    return env


# ---- f1tenth_ctl_car -------------------------------------------------------------------

@pytest.mark.parametrize('style', ['29', '28'])
def test_wrapper_runs_the_native_ctl_when_no_container_exists(fake, style):
    r = run([WRAPPER, 'status', '--json'], with_env(fake.env, FAKE_DOCKER_STYLE=style))
    assert r.returncode == 0, r.stdout
    assert json.loads(r.stdout) == {'ok': True, 'side': 'native', 'args': 'status --json'}


def test_wrapper_matches_the_container_name_exactly(fake):
    # `ps --filter name=f1tenth_car` also lists f1tenth_car_old: not ours.
    env = with_env(fake.env, FAKE_PS_OTHERS='f1tenth_car_old:running other_f1tenth_car:exited')
    r = run([WRAPPER, '--which'], env)
    assert (r.returncode, r.stdout.strip()) == (0, 'native')


@pytest.mark.parametrize('state', ['exited', 'created', 'dead'])
def test_wrapper_runs_the_native_ctl_when_the_container_is_stopped(fake, state):
    r = run([WRAPPER, '--which'], with_env(fake.env, FAKE_STATE=state))
    assert (r.returncode, r.stdout.strip()) == (0, 'native')


@pytest.mark.parametrize('err', [DOCKER29_ABSENT_DAEMON, DOCKER_REFUSED_DAEMON,
                                 DOCKER_REFUSED_DAEMON.upper()])
def test_wrapper_runs_the_native_ctl_when_the_daemon_is_down(fake, err):
    r = run([WRAPPER, 'status', '--json'], with_env(fake.env, FAKE_DOCKER_ERR=err))
    assert r.returncode == 0, r.stdout
    assert json.loads(r.stdout)['side'] == 'native'


def test_wrapper_against_the_real_docker_cli_without_the_container(fake):
    """Measured failure of round 1: Docker 29's texts made the wrapper refuse here."""
    docker = shutil.which('docker')
    if docker is None:
        pytest.skip('no docker CLI here')
    env = {k: v for k, v in os.environ.items() if not k.startswith(('FAKE_', 'F1T_'))}
    ps = run([docker, 'ps', '-a', '--filter', 'name=f1tenth_car', '--format', '{{.Names}}'], env)
    if ps.returncode != 0:
        pytest.skip(f'docker does not answer here: {ps.stderr.strip()[:120]}')
    if 'f1tenth_car' in ps.stdout.split():
        pytest.skip('a container f1tenth_car exists on this machine')
    env['F1T_NATIVE_CTL'] = fake.env['F1T_NATIVE_CTL']
    r = run([WRAPPER, '--which'], env)
    assert (r.returncode, r.stdout.strip()) == (0, 'native'), r.stdout + r.stderr
    r = run([WRAPPER, 'status', '--json'], env)
    assert r.returncode == 0 and json.loads(r.stdout)['side'] == 'native', r.stdout


@pytest.mark.parametrize('health', ['healthy', 'starting', 'unhealthy'])
def test_wrapper_execs_the_container_ctl_while_the_car_container_runs(fake, health):
    env = with_env(fake.env, FAKE_STATE='running', FAKE_HEALTH=health)
    r = run([WRAPPER, 'send', 'planner', 'vai dritto', '--json'], env)
    assert r.returncode == 0, r.stderr
    assert r.stdout.strip() == ('FAKE-EXEC f1tenth_car /opt/f1tenth_ws/src/f1tenth_ctl/bin/'
                                'f1tenth_ctl send planner vai dritto --json')
    assert 'native-ctl' not in fake.read_log()


def test_wrapper_passes_the_exit_code_of_the_container_ctl(fake):
    env = with_env(fake.env, FAKE_STATE='running', FAKE_EXEC_RC='3')
    assert run([WRAPPER, 'start', 'stack'], env).returncode == 3


def test_wrapper_refuses_while_the_harness_runs(fake):
    env = with_env(fake.env, FAKE_STATE='running', FAKE_ROLE='harness')
    r = run([WRAPPER, 'start', 'stack', '--json'], env)
    assert r.returncode == 1
    answer = json.loads(r.stdout)
    assert answer['ok'] is False and 'harness' in answer['error']
    assert 'native-ctl' not in fake.read_log() and 'exec' not in fake.read_log()


@pytest.mark.parametrize('err', [DOCKER29_NO_PERMISSION, DOCKER28_NO_PERMISSION,
                                 DOCKER_OLD_NO_PERMISSION])
def test_wrapper_refuses_when_docker_cannot_be_asked(fake, err):
    r = run([WRAPPER, 'status', '--json'], with_env(fake.env, FAKE_DOCKER_ERR=err))
    assert r.returncode == 1
    answer = json.loads(r.stdout)
    assert answer['ok'] is False and 'cannot ask Docker' in answer['error']
    assert 'permission denied' in answer['error'].lower()
    assert 'native-ctl' not in fake.read_log()


def test_wrapper_refuses_when_the_container_exists_but_cannot_be_inspected(fake):
    env = with_env(fake.env, FAKE_STATE='running', FAKE_INSPECT_ERR='error: template: boom')
    r = run([WRAPPER, 'status', '--json'], env)
    assert r.returncode == 1 and 'exists, but its state' in json.loads(r.stdout)['error']
    assert 'native-ctl' not in fake.read_log() and 'FAKE-EXEC' not in r.stdout


def test_wrapper_refuses_when_neither_side_exists(fake):
    r = run([WRAPPER, 'status'], with_env(fake.env, F1T_NATIVE_CTL=str(fake.tmp / 'missing')))
    assert r.returncode == 1 and 'no native f1tenth_ctl' in json.loads(r.stdout)['error']


# ---- stack_mode.sh ---------------------------------------------------------------------

def fake_tools(fake, guard_rc=0):
    guard = script(fake.tmp / 'guard',
                   '#!/bin/bash\necho "guard $*" >> "$FAKE_LOG"\n'
                   f'[ {guard_rc} = 0 ] || echo "  - TCP 8770 is listened on" >&2\n'
                   f'exit {guard_rc}\n')
    # `up` leaves the container FAKE_START_STATE (running), `down` absent.
    orin = script(fake.tmp / 'docker_orin',
                  '#!/bin/bash\necho "docker_orin $*" >> "$FAKE_LOG"\n'
                  'case "$1" in\n'
                  '  up) echo "${FAKE_START_STATE:-running}" > "$FAKE_STATE_FILE" ;;\n'
                  '  down) echo absent > "$FAKE_STATE_FILE" ;;\n'
                  'esac\nexit 0\n')
    return with_env(fake.env, F1T_GUARD=guard, F1T_DOCKER_ORIN=orin, F1T_GUARD_WAIT_S='1')


def test_stack_mode_container_dry_run_lists_the_steps_in_order_and_changes_nothing(fake):
    env = with_env(fake_tools(fake), FAKE_ACTIVE='f1tenth-gateway.service',
                   FAKE_KNOWN='f1tenth-car-container.service')
    r = run([STACK_MODE, 'container', '--archive', 'shared', '--dry-run'], env)
    assert r.returncode == 0, r.stderr
    out = r.stdout
    steps = [out.index('killall --reset-discovery --json'),
             out.index('systemctl --user stop f1tenth-gateway.service'),
             out.index('the guard right now'),
             out.index('F1T_ARCHIVE_MODE=shared F1T_ALLOW_DRIFT=0'),
             out.index('systemctl --user start f1tenth-car-container.service')]
    assert steps == sorted(steps)
    log = fake.read_log()
    assert 'native-ctl' not in log
    assert 'systemctl --user stop' not in log and 'systemctl --user start' not in log
    assert not (fake.home / '.config' / 'f1tenth' / 'car_container.env').exists()


def test_stack_mode_container_without_the_unit_uses_docker_orin_up(fake):
    r = run([STACK_MODE, 'container', '--allow-drift'], fake_tools(fake))
    assert r.returncode == 0, r.stderr
    log = fake.read_log()
    order = [log.index('native-ctl killall --reset-discovery --json'),
             log.index('guard --for jazzy'),
             log.index('docker_orin up --archive separate --allow-drift')]
    assert order == sorted(order)
    unit_env = (fake.home / '.config' / 'f1tenth' / 'car_container.env').read_text()
    assert 'F1T_ARCHIVE_MODE=separate' in unit_env


def test_stack_mode_container_with_the_unit_resets_allow_drift_after_the_start(fake):
    env = with_env(fake_tools(fake), FAKE_KNOWN='f1tenth-car-container.service')
    r = run([STACK_MODE, 'container', '--allow-drift'], env)
    assert r.returncode == 0, r.stderr
    assert 'systemctl --user start f1tenth-car-container.service' in fake.read_log()
    unit_env = (fake.home / '.config' / 'f1tenth' / 'car_container.env').read_text()
    assert 'F1T_ALLOW_DRIFT=0' in unit_env


@pytest.mark.parametrize('state', ['exited', 'absent', 'dead'])
def test_stack_mode_container_stops_a_stale_unit_before_starting_it(fake, state):
    """Round 1: the unit stays "active (exited)" after its container died; start was a no-op."""
    car = 'f1tenth-car-container.service'
    env = with_env(fake_tools(fake), FAKE_STATE=state, FAKE_ACTIVE=car, FAKE_KNOWN=car)
    r = run([STACK_MODE, 'container'], env)
    assert r.returncode == 0, r.stdout + r.stderr
    assert 'still active although f1tenth_car is ' + state in r.stdout
    log = fake.read_log()
    assert log.index('native-ctl killall') < log.index(f'systemctl --user stop {car}') \
        < log.index(f'systemctl --user start {car}')
    assert 'container mode in' in r.stdout


def test_stack_mode_container_judges_the_stale_unit_stop_by_the_unit_state(fake):
    car = 'f1tenth-car-container.service'
    env = with_env(fake_tools(fake), FAKE_STATE='exited', FAKE_ACTIVE=car, FAKE_KNOWN=car)
    # ExecStop (docker_orin down) exit 3 for stale SHM: the unit is down, go on.
    r = run([STACK_MODE, 'container'], with_env(env, FAKE_STOP_RC='3'))
    assert r.returncode == 0 and 'WARN: stopping' in r.stdout, r.stdout + r.stderr
    assert f'systemctl --user start {car}' in fake.read_log()
    # A unit that stays active: never a start that would do nothing.
    fake.log.unlink()
    (fake.tmp / 'ctr_state').unlink()
    for f in (fake.tmp / 'units').iterdir():
        f.unlink()
    r = run([STACK_MODE, 'container'], with_env(env, FAKE_STOP_STUCK='1'))
    assert r.returncode == 1 and 'still active' in r.stderr
    assert f'systemctl --user start {car}' not in fake.read_log()


def test_stack_mode_container_dry_run_lists_the_stale_unit_stop(fake):
    car = 'f1tenth-car-container.service'
    env = with_env(fake_tools(fake), FAKE_STATE='exited', FAKE_ACTIVE=car, FAKE_KNOWN=car)
    r = run([STACK_MODE, 'container', '--dry-run'], env)
    assert r.returncode == 0, r.stderr
    assert r.stdout.index(f'would run: systemctl --user stop {car}') \
        < r.stdout.index(f'would run: systemctl --user start {car}')
    assert 'systemctl --user stop' not in fake.read_log()


@pytest.mark.parametrize('start_state', ['exited', 'absent'])
def test_stack_mode_container_fails_when_the_unit_starts_but_no_container_runs(fake, start_state):
    car = 'f1tenth-car-container.service'
    env = with_env(fake_tools(fake), FAKE_KNOWN=car, FAKE_START_STATE=start_state)
    r = run([STACK_MODE, 'container'], env)
    assert r.returncode == 1
    assert f'after the start f1tenth_car is {start_state}' in r.stderr
    assert 'NO stack' in r.stderr and 'container mode in' not in r.stdout


def test_stack_mode_container_fails_when_the_harness_comes_up_instead(fake):
    car = 'f1tenth-car-container.service'
    env = with_env(fake_tools(fake), FAKE_KNOWN=car, FAKE_ROLE='harness')
    r = run([STACK_MODE, 'container'], env)
    assert r.returncode == 1 and '(role harness), not running as the car' in r.stderr


def test_stack_mode_container_checks_what_docker_orin_up_left(fake):
    env = with_env(fake_tools(fake), FAKE_START_STATE='absent')
    r = run([STACK_MODE, 'container'], env)
    assert r.returncode == 1 and 'after the start f1tenth_car is absent' in r.stderr
    assert 'docker_orin up' in fake.read_log()


def test_stack_mode_container_changes_nothing_when_docker_does_not_answer(fake):
    env = with_env(fake_tools(fake), FAKE_DOCKER_ERR=DOCKER29_ABSENT_DAEMON,
                   FAKE_KNOWN='f1tenth-car-container.service')
    r = run([STACK_MODE, 'container'], env)
    assert r.returncode == 1 and 'nothing was changed' in r.stderr
    assert 'failed to connect to the docker API' in r.stderr
    log = fake.read_log()
    assert 'native-ctl' not in log and 'systemctl --user st' not in log


def test_stack_mode_container_stops_when_the_guard_still_refuses(fake):
    r = run([STACK_MODE, 'container'], fake_tools(fake, guard_rc=1))
    assert r.returncode == 1 and 'guard still refuses' in r.stderr and 'TCP 8770' in r.stderr
    assert 'docker_orin up' not in fake.read_log()


def test_stack_mode_container_is_a_no_op_when_the_container_runs(fake):
    r = run([STACK_MODE, 'container'], with_env(fake_tools(fake), FAKE_STATE='running'))
    assert r.returncode == 0 and 'already in container mode' in r.stdout
    assert 'native-ctl' not in fake.read_log()


def test_stack_mode_native_dry_run(fake):
    env = with_env(fake_tools(fake), FAKE_STATE='running',
                   FAKE_ACTIVE='f1tenth-car-container.service',
                   FAKE_KNOWN='f1tenth-gateway.service')
    r = run([STACK_MODE, 'native', '--dry-run'], env)
    assert r.returncode == 0, r.stderr
    out = r.stdout
    order = [out.index('systemctl --user stop f1tenth-car-container.service'),
             out.index('the guard right now'),
             out.index('systemctl --user start f1tenth-gateway.service')]
    assert order == sorted(order)
    # The unit's ExecStop is the down; no second one is listed.
    assert 'docker_orin' not in out


def test_stack_mode_native_downs_a_container_started_without_the_unit(fake):
    env = with_env(fake_tools(fake), FAKE_STATE='running', FAKE_KNOWN='f1tenth-gateway.service')
    r = run([STACK_MODE, 'native'], env)
    assert r.returncode == 0, r.stderr
    log = fake.read_log()
    order = [log.index('docker_orin down'), log.index('guard --for humble'),
             log.index('systemctl --user start f1tenth-gateway.service')]
    assert order == sorted(order)


def test_stack_mode_status_and_usage(fake):
    tools = fake_tools(fake)
    r = run([STACK_MODE, 'status'], with_env(tools, FAKE_STATE='running'))
    assert r.returncode == 0 and 'mode: container (car)' in r.stdout
    gw = 'f1tenth-gateway.service'
    r = run([STACK_MODE, 'status'], with_env(tools, FAKE_STATE='running', FAKE_ACTIVE=gw))
    assert r.returncode == 1 and 'CONFLICT' in r.stdout
    r = run([STACK_MODE, 'status'], with_env(tools, FAKE_ACTIVE=gw))
    assert 'mode: native' in r.stdout
    assert run([STACK_MODE, 'sideways'], fake.env).returncode == 2
    assert run([STACK_MODE, '--help'], fake.env).returncode == 0


def test_stack_mode_status_unit_line(fake):
    car = 'f1tenth-car-container.service'
    tools = fake_tools(fake)
    # Not installed: one word, one line (round 1 printed "not-found" + "not installed").
    r = run([STACK_MODE, 'status'], tools)
    assert f'  {car}: inactive (not installed)\n' in r.stdout, r.stdout
    r = run([STACK_MODE, 'status'], with_env(tools, FAKE_KNOWN=car))
    assert f'  {car}: inactive (disabled)\n' in r.stdout, r.stdout
    r = run([STACK_MODE, 'status'], with_env(tools, FAKE_KNOWN=car, FAKE_ACTIVE=car,
                                             FAKE_STATE='exited'))
    assert 'STALE: active but f1tenth_car is exited' in r.stdout, r.stdout


# ---- install.sh / uninstall.sh / banner ------------------------------------------------

STATE_DIRS = ('ros', 'archive', 'ctl', 'gateway', 'archive_trigger', 'zed_settings', 'hf_cache',
              'run', 'logs', 'missions/mission_reports', 'missions/mission_videos',
              'missions/llm_generated', 'missions/corridors_jsons')


def test_install_is_idempotent_and_uninstall_keeps_the_data(fake):
    env = fake.env
    r = run([INSTALL, '--dry-run'], env)
    assert r.returncode == 0, r.stderr
    assert not fake.state.exists() and 'dry-run: none made' in r.stdout
    r = run([INSTALL], env)
    assert r.returncode == 0, r.stderr
    for d in STATE_DIRS:
        assert (fake.state / d).is_dir(), d
    bin_ = fake.home / '.local' / 'bin'
    assert os.readlink(bin_ / 'f1tenth_ctl_car') == WRAPPER
    assert os.readlink(bin_ / 'docker_orin') == DOCKER_ORIN
    assert os.readlink(bin_ / 'stack_mode') == STACK_MODE
    units = fake.home / '.config' / 'systemd' / 'user'
    target = os.readlink(units / 'f1tenth-car-container.service')
    assert target.endswith('host/systemd/f1tenth-car-container.service')
    log = fake.read_log()
    assert 'systemctl --user enable --now f1tenth-archive-trigger.path' in log
    assert 'enable f1tenth-car-container' not in log and 'start f1tenth-car-container' not in log
    bashrc = (fake.home / '.bashrc').read_text()
    assert bashrc.count('# f1tenth_car banner') == 1 and 'alias ll' in bashrc
    unit_env = (fake.home / '.config' / 'f1tenth' / 'car_container.env').read_text()
    assert unit_env.count('F1T_ARCHIVE_MODE=separate') == 1
    # Second run: nothing to do.
    path_unit = 'f1tenth-archive-trigger.path'
    r = run([INSTALL], with_env(env, FAKE_ENABLED=path_unit, FAKE_ACTIVE=path_unit))
    assert r.returncode == 0 and 'nothing to do' in r.stdout, r.stdout
    assert (fake.home / '.bashrc').read_text().count('# f1tenth_car banner') == 1
    # A run in the state survives the uninstall.
    (fake.state / 'archive' / 'run1').write_text('data')
    r = run([UNINSTALL], env)
    assert r.returncode == 0, r.stderr
    assert not (bin_ / 'f1tenth_ctl_car').exists()
    assert not (units / 'f1tenth-car-container.service').exists()
    bashrc = (fake.home / '.bashrc').read_text()
    assert '# f1tenth_car banner' not in bashrc and 'alias ll' in bashrc
    assert (fake.state / 'archive' / 'run1').read_text() == 'data'


def test_install_never_replaces_a_real_file(fake):
    bin_ = fake.home / '.local' / 'bin'
    bin_.mkdir(parents=True)
    (bin_ / 'stack_mode').write_text('#!/bin/sh\necho mine\n')
    r = run([INSTALL], fake.env)
    assert r.returncode == 1 and 'not a symlink' in r.stderr
    assert (bin_ / 'stack_mode').read_text() == '#!/bin/sh\necho mine\n'


def test_uninstall_refuses_while_the_container_runs(fake):
    r = run([UNINSTALL], with_env(fake.env, FAKE_STATE='running'))
    assert r.returncode == 1 and 'stack_mode native' in r.stderr


def test_uninstall_keeps_a_symlink_that_is_not_ours(fake):
    bin_ = fake.home / '.local' / 'bin'
    bin_.mkdir(parents=True)
    os.symlink('/usr/bin/true', bin_ / 'docker_orin')
    r = run([UNINSTALL], fake.env)
    assert r.returncode == 0 and os.path.islink(bin_ / 'docker_orin')


@pytest.mark.parametrize('state,warns', [('running', True), ('exited', False), ('absent', False)])
def test_banner_warns_only_in_interactive_shells_while_the_container_runs(fake, state, warns):
    banner = os.path.join(HOST, 'bashrc_banner.sh')
    env = with_env(fake.env, FAKE_STATE=state)
    r = run(['bash', '--norc', '--noprofile', '-i', '-c', f'. {banner}; echo rc=$?'], env)
    assert 'rc=0' in r.stdout
    assert ('f1tenth_car is RUNNING' in r.stderr) == warns
    r = run(['bash', '--norc', '--noprofile', '-c', f'. {banner}; echo rc=$?'], env)
    assert 'rc=0' in r.stdout and 'RUNNING' not in r.stderr


# ---- orin_stack_guard.sh ---------------------------------------------------------------

def test_guard_command_line():
    assert run([GUARD, '--help'], os.environ.copy()).returncode == 0
    assert run([GUARD, '--for', 'thor'], os.environ.copy()).returncode == 2
    assert run([GUARD, '--bogus'], os.environ.copy()).returncode == 2


def test_guard_refuses_humble_while_the_car_container_runs(fake):
    # ss prints nothing here; linus's real /proc may hold ROS processes of
    # other runs, so only the container's line is asserted.
    r = run([GUARD, '--for', 'humble'], with_env(fake.env, FAKE_STATE='running'))
    assert r.returncode == 1 and 'f1tenth_car runs' in r.stderr


# ---- docker_orin.sh --------------------------------------------------------------------

LABELS = {'f1tenth.gid.video': '44', 'f1tenth.gid.render': '104', 'f1tenth.gid.dialout': '20',
          'f1tenth.gid.input': '106', 'f1tenth.gid.zed': '1005', 'f1tenth.gid.jtop': '1003',
          'f1tenth.car_sync.sha256': 'abc', 'f1tenth.car_sync.native_head': '6c329992',
          'org.opencontainers.image.revision': 'be849452'}
DEVICES = ('Character devices:\n 13 input\n 81 video4linux\n166 ttyACM\n188 ttyUSB\n'
           '189 usb_device\n510 hidraw\n')


def orin_world(fake, drift_rc=0, guard_rc=0, labels=None):
    """Make the state, the inputs, a jtop socket, image labels, guard and drift stand-ins."""
    for d in STATE_DIRS:
        (fake.state / d).mkdir(parents=True, exist_ok=True)
    (fake.state / 'zed_settings' / 'SN36866199.conf').write_text('[x]\n')
    for d in ('projects/llm/models', '.local/share/f1tenth_gateway/www', '.config/f1tenth_car',
              '.config/f1tenth', '.config/f1tenth_gateway'):
        (fake.home / d).mkdir(parents=True, exist_ok=True)
    (fake.home / '.config' / 'f1tenth_gateway' / 'env').write_text('F1T_GATEWAY_TOKEN=x\n')
    sock_path = str(fake.tmp / 'jtop.sock')
    if not os.path.exists(sock_path):
        fake.sock = socket.socket(socket.AF_UNIX)
        fake.sock.bind(sock_path)
    lab = dict(LABELS)
    lab.update(labels or {})
    (fake.tmp / 'labels').write_text(''.join(f'{k}={v}\n' for k, v in lab.items()))
    (fake.tmp / 'car_sync.json').write_text('{"schema": 1, "native_head": "6c329992"}\n')
    (fake.tmp / 'devices').write_text(DEVICES)
    guard = script(fake.tmp / 'guard',
                   f'#!/bin/bash\necho "guard $*" >> "$FAKE_LOG"\nexit {guard_rc}\n')
    drift = script(fake.tmp / 'drift',
                   '#!/bin/bash\necho "drift $*" >> "$FAKE_LOG"\n'
                   f'[ {drift_rc} = 3 ] && echo \'{{"drift": ["src/x.py"]}}\'\nexit {drift_rc}\n')
    return with_env(fake.env, FAKE_IMAGE_ID='sha256:feedbeef0123',
                    FAKE_LABELS=str(fake.tmp / 'labels'),
                    FAKE_CAR_SYNC=str(fake.tmp / 'car_sync.json'), F1T_GUARD=guard,
                    F1T_DRIFT_CHECK=drift, F1T_PROC_DEVICES=str(fake.tmp / 'devices'),
                    F1T_JTOP_SOCK=sock_path, F1T_MIN_FREE_GB_STATE='0',
                    F1T_MIN_FREE_GB_ROOT='0')


def pinned(fake, **world):
    env = orin_world(fake, **world)
    assert run([DOCKER_ORIN, 'pin', IMAGE], env).returncode == 0
    return env


def test_pin_records_tag_and_id(fake):
    env = orin_world(fake)
    r = run([DOCKER_ORIN, 'pin', IMAGE], env)
    assert r.returncode == 0, r.stderr
    pin = (fake.home / '.config' / 'f1tenth' / 'car_image').read_text()
    assert f'tag {IMAGE}\n' in pin and 'id sha256:feedbeef0123\n' in pin
    assert f'pin image={IMAGE}' in (fake.state / 'history.log').read_text()


def test_pin_refuses_an_image_not_built_by_build_sh(fake):
    env = orin_world(fake, labels={'f1tenth.car_sync.sha256': ''})
    r = run([DOCKER_ORIN, 'pin', IMAGE], env)
    assert r.returncode == 1 and 'not built by docker/orin/build.sh' in r.stderr


def test_up_dry_run_runs_every_check_then_prints_the_compose_command(fake):
    env = pinned(fake)
    r = run([DOCKER_ORIN, 'up', '--dry-run', '--archive', 'separate'], env)
    assert r.returncode == 0, r.stderr
    assert 'docker compose -p f1tenth_car -f ' in r.stdout
    assert 'up -d --no-build --pull never' in r.stdout
    assert 'HIDRAW_MAJOR=510 GID_RENDER=104 GID_INPUT=106' in r.stdout
    log = fake.read_log()
    assert 'guard --for jazzy' in log and 'drift --car-sync' in log
    assert '--ws ' + str(fake.home / 'dev_ws' / 'f1tenth_more') in log
    assert 'docker compose' not in log  # dry-run: compose never called


def test_up_dry_run_harness_adds_the_override(fake):
    r = run([DOCKER_ORIN, 'up', '--dry-run', '--harness'], pinned(fake))
    if r.returncode == 1 and 'docker/orin/checks missing' in r.stderr:
        pytest.skip('docker/orin/checks arrives with W3 (D3 harness)')
    assert r.returncode == 0, r.stderr
    assert 'compose.harness.yaml' in r.stdout


@pytest.mark.parametrize('world,message', [
    ({'labels': {'f1tenth.gid.render': '994'}}, 'group render: host GID 104, image GID 994'),
    ({'labels': {'f1tenth.gid.input': ''}}, 'no f1tenth.gid.input label'),
    ({'guard_rc': 1}, 'the guard refuses'),
    ({'drift_rc': 3}, 'car-sync and rebuild'),
    ({'drift_rc': 2}, 'the drift check failed'),
])
def test_up_refuses(fake, world, message):
    r = run([DOCKER_ORIN, 'up', '--dry-run'], pinned(fake, **world))
    assert r.returncode == 1, r.stdout
    assert message in r.stderr, r.stderr


def test_up_allow_drift_starts_anyway_and_logs_it(fake):
    r = run([DOCKER_ORIN, 'up', '--dry-run', '--allow-drift'], pinned(fake, drift_rc=3))
    assert r.returncode == 0, r.stderr
    assert 'allow-drift' in (fake.state / 'history.log').read_text()


def test_up_refuses_when_the_tag_moved_or_nothing_is_pinned(fake):
    env = orin_world(fake)
    r = run([DOCKER_ORIN, 'up', '--dry-run'], env)
    assert r.returncode == 1 and 'no pinned image' in r.stderr
    run([DOCKER_ORIN, 'pin', IMAGE], env)
    r = run([DOCKER_ORIN, 'up', '--dry-run'], with_env(env, FAKE_IMAGE_ID='sha256:0ther'))
    assert r.returncode == 1 and 'rebuilt?' in r.stderr


def test_up_refuses_missing_state_inputs_and_devices(fake):
    env = pinned(fake)
    shutil.rmtree(fake.state / 'hf_cache')
    r = run([DOCKER_ORIN, 'up', '--dry-run'], env)
    assert r.returncode == 1 and 'hf_cache missing' in r.stderr
    (fake.state / 'hf_cache').mkdir()
    (fake.home / '.config' / 'f1tenth_gateway' / 'env').unlink()
    r = run([DOCKER_ORIN, 'up', '--dry-run'], env)
    assert r.returncode == 1 and 'gateway token' in r.stderr
    (fake.home / '.config' / 'f1tenth_gateway' / 'env').write_text('x')
    (fake.tmp / 'devices').write_text('Character devices:\n166 ttyACM\n')
    r = run([DOCKER_ORIN, 'up', '--dry-run'], env)
    assert r.returncode == 1 and 'no hidraw major' in r.stderr
    (fake.tmp / 'devices').write_text('Character devices:\n510 hidraw\n167 ttyACM\n')
    r = run([DOCKER_ORIN, 'up', '--dry-run'], env)
    assert r.returncode == 1 and 'ttyACM is major 167' in r.stderr


def test_up_refuses_when_the_container_already_runs(fake):
    env = with_env(pinned(fake), FAKE_STATE='running')
    r = run([DOCKER_ORIN, 'up', '--dry-run'], env)
    assert r.returncode == 1 and 'already runs' in r.stderr


@pytest.mark.parametrize('style', ['29', '28'])
def test_status_exit_codes(fake, style):
    env = with_env(orin_world(fake), FAKE_DOCKER_STYLE=style)
    r = run([DOCKER_ORIN, 'status'], env)
    assert r.returncode == 3, r.stdout + r.stderr
    assert 'container: f1tenth_car absent' in r.stdout
    running = with_env(env, FAKE_STATE='running', FAKE_HEALTH='healthy')
    assert run([DOCKER_ORIN, 'status'], running).returncode == 0
    sick = with_env(env, FAKE_STATE='running', FAKE_HEALTH='unhealthy')
    assert run([DOCKER_ORIN, 'status'], sick).returncode == 4
    stopped = with_env(env, FAKE_STATE='exited', FAKE_HEALTH='unhealthy')
    assert run([DOCKER_ORIN, 'status'], stopped).returncode == 3


@pytest.mark.parametrize('style', ['29', '28'])
def test_status_pin_line(fake, style):
    env = with_env(pinned(fake), FAKE_DOCKER_STYLE=style)
    r = run([DOCKER_ORIN, 'status'], env)
    pin_line = r.stdout.splitlines()[0]
    assert pin_line.startswith(f'pinned:    {IMAGE} feedbeef0123 (pinned ')
    assert 'WARNING' not in pin_line
    r = run([DOCKER_ORIN, 'status'], with_env(env, FAKE_IMAGE_ID='sha256:0123456789abcdef'))
    assert 'WARNING: the tag is now 0123456789ab' in r.stdout.splitlines()[0]
    # The tag is gone: Docker 29's failed `image inspect -f` prints an empty line first.
    r = run([DOCKER_ORIN, 'status'], with_env(env, FAKE_IMAGE_ID=''))
    assert r.returncode == 3, r.stdout + r.stderr
    assert r.stdout.splitlines()[0].endswith('WARNING: the tag is not present (docker image ls)')
    assert r.stdout.splitlines()[1].startswith('container: f1tenth_car absent')


def test_status_and_down_fail_when_docker_does_not_answer(fake):
    env = with_env(orin_world(fake), FAKE_DOCKER_ERR=DOCKER29_ABSENT_DAEMON)
    for cmd in ('status', 'down'):
        r = run([DOCKER_ORIN, cmd], env)
        assert r.returncode == 1, (cmd, r.stdout, r.stderr)
        assert "cannot ask Docker for f1tenth_car's state" in r.stderr
        assert 'does not exist' not in r.stdout and 'absent' not in r.stdout


def test_a_similar_container_name_is_not_f1tenth_car(fake):
    env = with_env(orin_world(fake), FAKE_PS_OTHERS='f1tenth_car_old:running')
    r = run([DOCKER_ORIN, 'status'], env)
    assert r.returncode == 3 and 'f1tenth_car absent' in r.stdout


@pytest.mark.parametrize('style', ['29', '28'])
def test_down_without_a_container_reports_and_logs(fake, style):
    """Round 1: Docker 29's empty stdout line made this take the stop path and fail."""
    r = run([DOCKER_ORIN, 'down'], with_env(orin_world(fake), FAKE_DOCKER_STYLE=style))
    assert r.returncode == 0, r.stdout + r.stderr
    assert 'f1tenth_car does not exist' in r.stdout and 'leftovers: none' in r.stdout
    log = fake.read_log()
    assert 'docker stop' not in log and 'docker rm' not in log
    assert 'down container_state=absent' in (fake.state / 'history.log').read_text()


def test_down_of_an_exited_container_removes_it_without_a_stop(fake):
    r = run([DOCKER_ORIN, 'down'], with_env(orin_world(fake), FAKE_STATE='exited'))
    assert r.returncode == 0, r.stderr
    log = fake.read_log()
    assert 'docker rm f1tenth_car' in log and 'docker stop' not in log


def test_restart_with_nothing_running_goes_on_to_up(fake):
    r = run([DOCKER_ORIN, 'restart', '--dry-run'], pinned(fake))
    assert r.returncode == 0, r.stdout + r.stderr
    assert 'dry-run: f1tenth_car is absent' in r.stdout
    assert 'up -d --no-build --pull never' in r.stdout
    assert 'down container_state' not in (fake.state / 'history.log').read_text()


def test_restart_dry_run_stops_nothing(fake):
    # --dry-run reaches the down half too; up --dry-run then sees the container
    # still running and says so (a dry run cannot take it down first).
    r = run([DOCKER_ORIN, 'restart', '--dry-run'], with_env(pinned(fake), FAKE_STATE='running'))
    assert 'would signal exec-started processes, docker stop -t 120' in r.stdout
    log = fake.read_log()
    assert 'docker stop' not in log and 'docker rm' not in log and 'docker exec' not in log
    assert r.returncode == 1 and 'already runs' in r.stderr


def test_wait_host(fake):
    env = orin_world(fake)
    r = run([DOCKER_ORIN, 'wait-host', '5'], env)
    assert r.returncode == 0 and r.stdout == '', r.stdout + r.stderr
    r = run([DOCKER_ORIN, 'wait-host', '1'],
            with_env(env, F1T_JTOP_SOCK=str(fake.tmp / 'no.sock')))
    assert r.returncode == 1 and 'no.sock is not a socket after' in r.stderr
    r = run([DOCKER_ORIN, 'wait-host', '1'], with_env(env, FAKE_DOCKER_ERR=DOCKER_REFUSED_DAEMON))
    assert r.returncode == 1 and 'dockerd does not answer after' in r.stderr
    assert 'wait-host FAILED' in (fake.state / 'history.log').read_text()
    assert run([DOCKER_ORIN, 'wait-host', 'soon'], env).returncode == 2


def test_down_of_a_car_container_is_docker_stop_then_rm_and_a_clean_report(fake):
    r = run([DOCKER_ORIN, 'down'], with_env(orin_world(fake), FAKE_STATE='running'))
    assert r.returncode == 0, r.stderr
    log = fake.read_log()
    assert 'docker stop -t 120 f1tenth_car' in log and 'docker rm f1tenth_car' in log
    assert 'docker exec' not in log  # role car: the main process stops its own processes
    assert 'guard --for humble --quiet' in log and 'leftovers: none' in r.stdout


def test_down_of_the_harness_signals_the_exec_started_processes_first(fake):
    env = with_env(orin_world(fake), FAKE_STATE='running', FAKE_ROLE='harness')
    r = run([DOCKER_ORIN, 'down'], env)
    assert r.returncode == 0, r.stderr
    log = fake.read_log()
    assert log.index('docker exec -i f1tenth_car python3 -') \
        < log.index('docker stop -t 120 f1tenth_car')


def test_down_reports_leftovers_with_exit_3(fake):
    r = run([DOCKER_ORIN, 'down'], orin_world(fake, guard_rc=1))
    assert r.returncode == 3 and 'leftovers' in r.stdout


def test_usage_errors(fake):
    assert run([DOCKER_ORIN], fake.env).returncode == 2
    assert run([DOCKER_ORIN, 'sideways'], fake.env).returncode == 2
    world = orin_world(fake)
    assert run([DOCKER_ORIN, 'up', '--archive', 'both'], world).returncode == 2
    for args in (['up', '--archive'], ['up', '--autostart'], ['logs', '-n'], ['down', '--now']):
        r = run([DOCKER_ORIN] + args, world)
        assert r.returncode == 2 and r.stderr.strip(), (args, r.stderr)
    assert run([DOCKER_ORIN, '--help'], fake.env).returncode == 0


# ---- compose ---------------------------------------------------------------------------

COMPOSE_ENV = {'F1TENTH_IMAGE': 'f1tenth:jazzy-orin-fake', 'F1TENTH_IMAGE_ID': 'sha256:0',
               'GID_VIDEO': '44', 'GID_RENDER': '104', 'GID_DIALOUT': '20', 'GID_INPUT': '106',
               'GID_ZED': '1005', 'GID_JTOP': '1003', 'HIDRAW_MAJOR': '510',
               'F1T_STATE': '/scratch/fabiocar/f1tenth_container',
               'F1T_ARCHIVE_DIR': '/scratch/fabiocar/f1tenth_container/archive',
               'HOME': '/home/fabiocar', 'F1T_REPO': '/home/fabiocar/dev_ws/f1tenth_car'}


def compose_config(files, env):
    have = shutil.which('docker') is not None
    if not have or run(['docker', 'compose', 'version'], os.environ.copy()).returncode:
        pytest.skip('no docker compose here')
    args = ['docker', 'compose']
    for f in files:
        args += ['-f', os.path.join(ORIN, f)]
    return run(args + ['config', '--format', 'json'], env)


def compose_env(drop=None):
    env = {k: v for k, v in os.environ.items() if k not in COMPOSE_ENV}
    env.update({k: v for k, v in COMPOSE_ENV.items() if k != drop})
    return env


def test_compose_renders_with_every_variable_and_fails_without_one():
    r = compose_config(['compose.yaml'], compose_env())
    assert r.returncode == 0, r.stderr
    car = json.loads(r.stdout)['services']['car']
    assert car['network_mode'] == 'host' and car['ipc'] == 'host' and car['init'] is True
    assert car['runtime'] == 'nvidia' and car['restart'] == 'no' and car['pull_policy'] == 'never'
    assert car['ulimits']['nice'] == {'soft': 40, 'hard': 40}
    assert 'c 510:* rmw' in car['device_cgroup_rules']
    targets = {v['target']: v for v in car['volumes']}
    assert targets['/home/fabiocar/.config/f1tenth_gateway/env']['read_only'] is True
    binds = [v for v in car['volumes'] if v['type'] == 'bind']
    assert all(v.get('bind', {}).get('create_host_path') is False for v in binds)
    for var in ('F1TENTH_IMAGE', 'GID_INPUT', 'HIDRAW_MAJOR', 'F1T_STATE', 'F1T_ARCHIVE_DIR'):
        assert compose_config(['compose.yaml'], compose_env(drop=var)).returncode != 0, var


def test_compose_harness_override():
    r = compose_config(['compose.yaml', 'compose.harness.yaml'], compose_env())
    assert r.returncode == 0, r.stderr
    car = json.loads(r.stdout)['services']['car']
    assert car['command'] == ['sleep', 'infinity']
    assert car['environment']['F1T_CONTAINER_ROLE'] == 'harness'
    assert car['environment']['ROS_DOMAIN_ID'] == '88'
    targets = {v['target']: v for v in car['volumes']}
    assert targets['/home/fabiocar/dev_ws/f1tenth_car/scripts']['read_only'] is True
    assert not targets['/home/fabiocar/dev_ws/f1tenth_car/output'].get('read_only')


def test_executable_bits():
    for path in (WRAPPER, STACK_MODE, DOCKER_ORIN, GUARD, INSTALL, UNINSTALL):
        assert os.stat(path).st_mode & stat.S_IXUSR, path
