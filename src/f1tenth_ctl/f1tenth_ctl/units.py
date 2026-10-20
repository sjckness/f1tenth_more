"""
Managed processes as transient systemd user units (f1t-<name>.service).

Why systemd and not tmux: see output/telegram/recon.md section 7. In short,
the unit's cgroup holds the whole process tree (the supervisor's own
`ros2 launch` children, llama-server), systemd keeps the exit status of a
unit that failed, and the unit outlives the ssh session (Linger is on).

States reported for a managed process:
- running: the unit is active (or starting);
- stopped: no unit, or one stopped by f1tenth_ctl;
- crashed: the unit failed, or it is gone although f1tenth_ctl started it and
  never stopped it (an exit 0 nobody asked for is still a crash).
The "desired" state that tells the last two apart is a small JSON file per
process in the state directory, written on start and on stop.
"""

import json
import os
import re
import signal
import subprocess
import time
from typing import Callable, Dict, List, Optional

from .config import BASE_KEYS, BASE_PATH, ProcSpec, Settings

_SHOW_PROPS = ('Id', 'LoadState', 'ActiveState', 'SubState', 'Result', 'MainPID',
               'ExecMainCode', 'ExecMainStatus', 'ActiveEnterTimestamp')
_RUNNING = ('active', 'activating', 'reloading')

# si_code values systemd reports in ExecMainCode.
_EXIT_CODES = {'1': 'exited', '2': 'killed', '3': 'dumped'}


def systemctl(args: List[str], timeout: float = 10.0, env=None):
    return subprocess.run(['systemctl', '--user', *args], capture_output=True,
                          text=True, timeout=timeout, env=env)


def parse_show(text: str) -> List[Dict[str, str]]:
    """Split `systemctl show` output for several units into one dict each."""
    blocks, cur = [], {}
    for line in text.splitlines():
        if not line.strip():
            if cur:
                blocks.append(cur)
                cur = {}
            continue
        key, _, value = line.partition('=')
        cur[key] = value
    if cur:
        blocks.append(cur)
    return blocks


def show(units: List[str]) -> Dict[str, Dict[str, str]]:
    proc = systemctl(['show', '--no-pager', '-p', ','.join(_SHOW_PROPS), *units])
    out = {}
    for block, unit in zip(parse_show(proc.stdout), units):
        out[block.get('Id') or unit] = block
    return out


def _state_file(settings: Settings, name: str) -> str:
    return os.path.join(settings.state_dir, f'{name}.json')


def read_desired(settings: Settings, name: str) -> dict:
    try:
        with open(_state_file(settings, name)) as f:
            return json.load(f)
    except (OSError, ValueError):
        return {}


def write_desired(settings: Settings, name: str, desired: str, **extra) -> None:
    os.makedirs(settings.state_dir, exist_ok=True)
    data = {'desired': desired, 'time': time.time(), **extra}
    tmp = _state_file(settings, name) + '.tmp'
    with open(tmp, 'w') as f:
        json.dump(data, f)
    os.replace(tmp, _state_file(settings, name))


def classify(props: Dict[str, str], desired: dict) -> dict:
    """Classify one unit's properties as running, stopped or crashed (pure)."""
    active = props.get('ActiveState', 'inactive')
    info = {'unit': props.get('Id', ''), 'managed': True}
    if active in _RUNNING:
        info['state'] = 'running'
        pid = int(props.get('MainPID') or 0)
        if pid:
            info['pid'] = pid
        if props.get('ActiveEnterTimestamp'):
            info['since'] = props['ActiveEnterTimestamp']
        return info
    if active == 'deactivating':
        info['state'] = 'stopping'
        return info
    if active == 'failed':
        info['state'] = 'crashed'
        info['result'] = props.get('Result', '')
        code = _EXIT_CODES.get(props.get('ExecMainCode', ''), props.get('ExecMainCode', ''))
        status = props.get('ExecMainStatus', '')
        if code == 'exited':
            info['exit_status'] = int(status or 0)
        elif code in ('killed', 'dumped'):
            info['signal'] = _signal_name(status)
        return info
    if desired.get('desired') == 'running':
        info['state'] = 'crashed'
        info['result'] = 'exited without a stop request'
        return info
    info['state'] = 'stopped'
    return info


def _signal_name(num: str) -> str:
    try:
        return signal.Signals(int(num)).name
    except (ValueError, TypeError):
        return str(num)


def unit_states(settings: Settings, names) -> Dict[str, dict]:
    specs = [settings.procs[n] if n in settings.procs
             else ProcSpec(n, '', unit_prefix=settings.unit_prefix) for n in names]
    props = show([s.unit for s in specs])
    out = {}
    for spec in specs:
        out[spec.name] = classify(props.get(spec.unit, {'Id': spec.unit}),
                                  read_desired(settings, spec.name))
    return out


def is_running(settings: Settings, name: str) -> bool:
    return unit_states(settings, [name])[name]['state'] in ('running', 'stopping')


# Fixed on purpose: systemd expands "${VAR}" and a bare "$VAR" word in
# ExecStart, and turns "$$" into "$". The command itself travels in
# F1T_UNIT_CMD (environment values are never expanded), so whatever the env
# file puts in F1T_*_CMD reaches bash untouched.
# The unit starts with the user manager's environment; it is cut down to the
# same base the CLI sources the env file on (config.base_environment), so the
# units and the CLI's own helpers see one and the same environment.
_KEEP = '|'.join(BASE_KEYS + ('F1T_CTL_ENV', 'F1T_UNIT_*', 'PYTHONUNBUFFERED',
                              'RCUTILS_LOGGING_BUFFERED_STREAM'))
UNIT_SCRIPT = (
    'for v in $(compgen -e); do case "$v" in ' + _KEEP + ') ;; *) unset "$v" ;; esac; done; '
    f'PATH="$HOME/.local/bin:{BASE_PATH}"; export PATH; '
    'echo "[f1tenth_ctl] $(date -Is) start $F1T_UNIT_NAME: $F1T_UNIT_CMD"; '
    'set -a; . "$F1T_CTL_ENV" || exit 97; set +a; '
    # The FIFO is opened read-write and kept as stdin, so there is always a
    # writer and the reader never sees EOF. `f1tenth_ctl send` writes to it.
    'if [ -n "$F1T_UNIT_FIFO" ]; then exec 3<>"$F1T_UNIT_FIFO"; exec 0<&3 3<&-; fi; '
    'eval "exec $F1T_UNIT_CMD"'
)


def build_command(settings: Settings, spec: ProcSpec) -> List[str]:
    """Build the systemd-run command line for one managed process (pure)."""
    log = settings.log_path(spec.name)
    fifo = settings.fifo_path(spec.name) if spec.stdin_fifo else ''
    return [
        'systemd-run', '--user', '--quiet',
        f'--unit={spec.unit}',
        f'--description=f1tenth_ctl {spec.name}',
        # After the main process exits, systemd sends SIGTERM to whatever is
        # left in the cgroup and SIGKILL after TimeoutStopSec.
        '-p', 'KillSignal=SIGTERM',
        '-p', f'TimeoutStopSec={max(1, int(settings.sigterm_wait_s))}',
        '-p', f'StandardOutput=append:{log}',
        '-p', f'StandardError=append:{log}',
        '-p', f'WorkingDirectory={settings.env.get("HOME", "~")}',
        f'--setenv=F1T_CTL_ENV={settings.env_file}',
        f'--setenv=F1T_UNIT_NAME={spec.name}',
        f'--setenv=F1T_UNIT_CMD={spec.cmd}',
        f'--setenv=F1T_UNIT_FIFO={fifo}',
        '--setenv=PYTHONUNBUFFERED=1',
        '--setenv=RCUTILS_LOGGING_BUFFERED_STREAM=0',
        '/bin/bash', '-c', UNIT_SCRIPT,
    ]


def _ensure_fifo(path: str) -> None:
    if os.path.exists(path):
        import stat
        if stat.S_ISFIFO(os.stat(path).st_mode):
            return
        os.remove(path)
    os.mkfifo(path, 0o600)


def start(settings: Settings, spec: ProcSpec, settle_s: float = 1.5) -> dict:
    os.makedirs(settings.log_dir, exist_ok=True)
    if spec.stdin_fifo:
        _ensure_fifo(settings.fifo_path(spec.name))
    systemctl(['reset-failed', spec.unit])
    cmd = build_command(settings, spec)
    proc = subprocess.run(cmd, capture_output=True, text=True, timeout=15,
                          env=_systemd_env(settings))
    if proc.returncode != 0:
        return {'ok': False, 'error': f'systemd-run failed: {proc.stderr.strip()}'}
    write_desired(settings, spec.name, 'running', cmd=spec.cmd)
    deadline = time.monotonic() + settle_s
    while True:
        state = unit_states(settings, [spec.name])[spec.name]
        if state['state'] != 'running' or time.monotonic() >= deadline:
            break
        time.sleep(0.25)
    if state.get('state') != 'running':
        return {'ok': False, 'error': f'{spec.name} exited right after start',
                **state, 'log_tail': log_tail(settings, spec.name)}
    return {'ok': True, **state, 'log': settings.log_path(spec.name)}


def _systemd_env(settings: Settings) -> dict:
    env = dict(os.environ)
    for key in ('XDG_RUNTIME_DIR', 'DBUS_SESSION_BUS_ADDRESS'):
        if key in settings.env:
            env[key] = settings.env[key]
    return env


def _wait_inactive(settings: Settings, name: str, timeout: float,
                   poll: float = 0.2) -> bool:
    deadline = time.monotonic() + timeout
    while True:
        if unit_states(settings, [name])[name]['state'] not in ('running', 'stopping'):
            return True
        if time.monotonic() >= deadline:
            return False
        time.sleep(poll)


def stop_ladder(is_down: Callable[[float], bool],
                send: Callable[[str], None],
                sigint_wait: float, sigterm_wait: float,
                sigkill_wait: float = 2.0) -> dict:
    """
    Send SIGINT, wait, SIGTERM, wait, SIGKILL (pure apart from the callbacks).

    `send(signame)` delivers a signal; `is_down(timeout)` waits up to
    `timeout` seconds and says whether the target is gone.
    """
    t0 = time.monotonic()
    sent = []
    for signame, wait in (('SIGINT', sigint_wait), ('SIGTERM', sigterm_wait),
                          ('SIGKILL', sigkill_wait)):
        send(signame)
        sent.append(signame)
        if is_down(wait):
            return {'stopped': True, 'signals': sent,
                    'elapsed_s': round(time.monotonic() - t0, 2)}
    return {'stopped': False, 'signals': sent,
            'elapsed_s': round(time.monotonic() - t0, 2)}


def stop(settings: Settings, spec: ProcSpec) -> dict:
    state = unit_states(settings, [spec.name])[spec.name]
    write_desired(settings, spec.name, 'stopped')
    if state['state'] not in ('running', 'stopping'):
        systemctl(['reset-failed', spec.unit])
        return {'ok': True, 'already': state['state'], 'signals': []}
    pid = state.get('pid')

    def send(signame):
        if signame == 'SIGINT':
            # Ctrl-C in a terminal: SIGINT to the main process's process group
            # (systemd makes it the group leader). `ros2 launch` then shuts its
            # tree down itself, and `ros2 run` expects its child to get the
            # signal too (it does not forward it). Process groups the stack
            # made for itself (the supervisor's components) are left to it, as
            # with Ctrl-C.
            if pid:
                try:
                    if os.getpgid(pid) == pid:
                        os.killpg(pid, signal.SIGINT)
                    else:
                        os.kill(pid, signal.SIGINT)
                except ProcessLookupError:
                    pass
        else:
            systemctl(['kill', f'--signal={signame}', spec.unit])

    result = stop_ladder(lambda t: _wait_inactive(settings, spec.name, t), send,
                         settings.sigint_wait_s, settings.sigterm_wait_s)
    systemctl(['stop', spec.unit], timeout=15)
    systemctl(['reset-failed', spec.unit])
    result['ok'] = result['stopped']
    return result


def log_tail(settings: Settings, name: str, lines: int = 15) -> List[str]:
    path = settings.log_path(name)
    try:
        with open(path, 'rb') as f:
            f.seek(0, os.SEEK_END)
            size = f.tell()
            f.seek(max(0, size - 16384))
            data = f.read().decode(errors='replace')
    except OSError:
        return []
    return data.splitlines()[-lines:]


def unit_pids(unit: str) -> List[int]:
    """Every pid in the unit's cgroup (for telling managed from strays)."""
    proc = systemctl(['show', '-p', 'ControlGroup', '--value', unit])
    cg = proc.stdout.strip()
    if not cg:
        return []
    for root in ('/sys/fs/cgroup', '/sys/fs/cgroup/unified', '/sys/fs/cgroup/systemd'):
        path = f'{root}{cg}/cgroup.procs'
        try:
            with open(path) as f:
                return [int(x) for x in f.read().split()]
        except OSError:
            continue
    return []


MAX_LINE = 500
_ANSI = re.compile(r'\x1b\[[0-9;?]*[ -/]*[@-~]|\x1b[@-Z\\-_]')


def check_line(text: str):
    """Return (line, None) for one acceptable stdin line, else (None, why) (pure)."""
    if '\n' in text or '\r' in text:
        return None, 'one line only: the text contains a line break'
    line = text.strip()
    if not line:
        return None, 'empty text'
    if len(line) > MAX_LINE:
        return None, f'too long: {len(line)} characters, at most {MAX_LINE}'
    return line, None


def log_size(settings: Settings, name: str) -> int:
    try:
        return os.path.getsize(settings.log_path(name))
    except OSError:
        return 0


def fifo_send(settings: Settings, name: str, text: str) -> dict:
    """
    Write one line to a running process's stdin FIFO.

    The answer carries `cursor`, the log's size just before the write: a
    `planner-log --since <cursor>` then returns only what came after it.
    """
    line, why = check_line(text)
    if why:
        return {'ok': False, 'error': why}
    path = settings.fifo_path(name)
    if not is_running(settings, name):
        return {'ok': False, 'error': f'{name} is not running', 'running': False}
    cursor = log_size(settings, name)
    try:
        fd = os.open(path, os.O_WRONLY | os.O_NONBLOCK)
    except OSError as exc:
        return {'ok': False, 'error': f'cannot open {path}: {exc.strerror}'}
    try:
        os.write(fd, (line + '\n').encode())
    finally:
        os.close(fd)
    return {'ok': True, 'sent': line, 'cursor': cursor, 'running': True}


def strip_ansi(text: str) -> str:
    return _ANSI.sub('', text)


def read_log_since(path: str, since: Optional[int], max_bytes: int = 65536) -> dict:
    """
    Return the complete lines of a log after byte offset `since` (pure, file only).

    `cursor` is where the next read starts: after the last complete line, so a
    line still being written is returned whole, next time. Without `since`
    nothing is read and `cursor` is the end of the file. A `since` past the
    end means the file was replaced: it is read from the start (`reset`).
    """
    try:
        size = os.path.getsize(path)
    except OSError:
        size = 0
    out = {'size': size, 'lines': [], 'reset': False, 'truncated': False}
    if since is None:
        out['cursor'] = size
        return out
    start = since
    if start > size or start < 0:
        start = 0
        out['reset'] = True
    if size == start:
        out['cursor'] = start
        return out
    with open(path, 'rb') as f:
        f.seek(start)
        chunk = f.read(max_bytes)
    end = chunk.rfind(b'\n')
    if end < 0:
        # No complete line yet; a single line longer than max_bytes is cut.
        if len(chunk) < max_bytes:
            out['cursor'] = start
            return out
        end = len(chunk) - 1
    body = chunk[:end + 1]
    out['cursor'] = start + len(body)
    out['truncated'] = start + len(chunk) < size
    out['lines'] = [strip_ansi(line) for line in
                    body.decode(errors='replace').splitlines()]
    return out


def main_pid(settings: Settings, name: str) -> Optional[int]:
    return unit_states(settings, [name])[name].get('pid')
