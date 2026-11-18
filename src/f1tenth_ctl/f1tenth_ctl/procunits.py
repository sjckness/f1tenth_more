"""
The proc backend of units.py: managed processes without systemd.

Inside the car's container there is no user systemd manager, so
`systemd-run --user` cannot hold the stack. Here one runner process per
managed process (runner.py, double-forked, child subreaper) takes the unit's
place. units.py calls this module when the backend is `proc` and keeps
everything else as it is: the unit names (f1t-<name>.service), the logs, the
stdin FIFOs, the desired-state files, the stop ladder, and the property dicts
classify() reads, so status, start, stop and their JSON are the same as with
systemd.

Per unit, under <state_dir>/proc/:
  <unit>/proc.json   written by the runner once the process runs: runner and
                     main pid with their /proc start times and the boot id
                     (a pid alone could have been reused by now)
  <unit>/exit.json   written by the runner when the main process ended and
                     what it left behind was stopped: how it ended
  <unit>.lock        flock, so two callers (the bot, the gateway) never start
                     the same unit twice

What the systemd calls become:
  systemd-run              launch(): the runner, which returns once the process runs
  systemctl show           show(): ActiveState/Result/MainPID/... from the records
  systemctl kill --signal  kill(): the signal to the runner's whole tree
  systemctl stop           finish_stop(): SIGTERM to the runner, which stops the
                           tree (SIGKILL after TimeoutStopSec), then reset_failed()
  systemctl reset-failed   reset_failed(): the records go
  cgroup.procs             unit_pids(): the runner and every process below it
"""

from contextlib import contextmanager
import fcntl
import json
import os
import signal
import subprocess
import sys
import time
from typing import Dict, List, Optional

from . import config, runner, units
from .config import MANAGED, ProcSpec, Settings

LAUNCH_TIMEOUT_S = 15.0       # as the systemd-run call
LOCK_TIMEOUT_S = 30.0         # a stop holds the lock for up to ~20 s


def proc_dir(settings: Settings) -> str:
    return os.path.join(settings.state_dir, 'proc')


def unit_dir(settings: Settings, unit: str) -> str:
    return os.path.join(proc_dir(settings), unit)


@contextmanager
def _locked(settings: Settings, unit: str, timeout: float = LOCK_TIMEOUT_S):
    """Hold the unit's lock; yields False if it could not be had in `timeout` s."""
    os.makedirs(proc_dir(settings), exist_ok=True)
    fd = os.open(os.path.join(proc_dir(settings), f'{unit}.lock'), os.O_RDWR | os.O_CREAT, 0o600)
    try:
        deadline = time.monotonic() + timeout
        got = False
        while True:
            try:
                fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
                got = True
                break
            except BlockingIOError:
                if time.monotonic() >= deadline:
                    break
                time.sleep(0.05)
        yield got
    finally:
        os.close(fd)


def _read_json(path: str) -> Optional[dict]:
    try:
        with open(path) as f:
            data = json.load(f)
    except (OSError, ValueError):
        return None
    return data if isinstance(data, dict) else None


def _stamp(t: float) -> str:
    """Format a time as systemd's ActiveEnterTimestamp does."""
    return time.strftime('%a %Y-%m-%d %H:%M:%S %Z', time.localtime(t))


def record(settings: Settings, unit: str) -> dict:
    """
    Return what is known of a unit: its records and whether they are alive.

    A record from another boot (or, in a container, about processes that are
    gone) never makes a pid alive: pid and start time must both match.
    """
    d = unit_dir(settings, unit)
    boot = runner.boot_id()
    info = _read_json(os.path.join(d, 'proc.json'))
    if info and info.get('boot_id') != boot:
        info = None
    out = {'info': info, 'runner_alive': False, 'main_alive': False, 'exit': None}
    if info:
        out['runner_alive'] = runner.alive(info.get('runner_pid'), info.get('runner_start'))
        out['main_alive'] = runner.alive(info.get('pid'), info.get('pid_start'))
    if not out['runner_alive']:
        # Read after the liveness check: the runner writes it before it exits.
        ex = _read_json(os.path.join(d, 'exit.json'))
        out['exit'] = ex if ex and ex.get('boot_id') == boot else None
    return out


def props_of(unit: str, rec: dict) -> Dict[str, str]:
    """Turn a record() into the properties `systemctl show` would give (pure)."""
    p = {'Id': unit, 'LoadState': 'not-found', 'ActiveState': 'inactive', 'SubState': 'dead',
         'Result': 'success', 'MainPID': '0', 'ExecMainCode': '0', 'ExecMainStatus': '0',
         'ActiveEnterTimestamp': ''}
    info, ex = rec.get('info') or {}, rec.get('exit')
    if rec.get('main_alive'):
        p.update(LoadState='loaded', ActiveState='active', SubState='running',
                 MainPID=str(info.get('pid', 0)),
                 ActiveEnterTimestamp=_stamp(float(info.get('started_at') or 0)))
    elif rec.get('runner_alive'):
        # The main process ended; the runner stops what it left (stop-sigterm).
        p.update(LoadState='loaded', ActiveState='deactivating', SubState='stop-sigterm')
    elif ex:
        failed = ex.get('result') != 'success'
        p.update(LoadState='loaded', ActiveState='failed' if failed else 'inactive',
                 SubState='failed' if failed else 'dead', Result=str(ex.get('result')),
                 ExecMainCode=str(ex.get('code', 0)), ExecMainStatus=str(ex.get('status', 0)))
    elif rec.get('info'):
        # A runner that left no exit record and nothing alive: it was killed
        # (SIGKILL), so how the process ended is unknown.
        p.update(LoadState='loaded', ActiveState='failed', SubState='failed',
                 Result='runner-lost')
    return p


def show(settings: Settings, unit_names: List[str]) -> Dict[str, Dict[str, str]]:
    return {u: props_of(u, record(settings, u)) for u in unit_names}


def unit_environment(settings: Settings, spec: ProcSpec) -> Dict[str, str]:
    """
    Return the runner's environment: systemd-run's --setenv values on the CLI's base.

    units.UNIT_SCRIPT cuts it down exactly as in a systemd unit, so a process
    sees what it would see there (test_process_manager compares both).
    """
    env = config.base_environment()
    env.update({
        'F1T_CTL_ENV': settings.env_file,
        'F1T_UNIT_NAME': spec.name,
        'F1T_UNIT_CMD': spec.cmd,
        'F1T_UNIT_FIFO': settings.fifo_path(spec.name) if spec.stdin_fifo else '',
        'PYTHONUNBUFFERED': '1',
        'RCUTILS_LOGGING_BUFFERED_STREAM': '0',
        # Only for `-m f1tenth_ctl.runner`: UNIT_SCRIPT unsets it.
        'PYTHONPATH': config.SOURCE_ROOT,
    })
    return env


def kill_timeout(settings: Settings) -> int:
    """Return the systemd backend's TimeoutStopSec: SIGTERM to SIGKILL, for leftovers."""
    return max(1, int(settings.sigterm_wait_s))


def runner_command(settings: Settings, spec: ProcSpec) -> List[str]:
    """Build the runner's command line for one managed process (pure)."""
    return [sys.executable or 'python3', '-m', 'f1tenth_ctl.runner',
            '--unit', spec.unit, '--dir', unit_dir(settings, spec.unit),
            '--log', settings.log_path(spec.name),
            '--workdir', settings.workdir or settings.env.get('HOME', '/'),
            '--kill-timeout', str(kill_timeout(settings)),
            '--', '/bin/bash', '-c', units.UNIT_SCRIPT]


def _clear(settings: Settings, unit: str) -> None:
    d = unit_dir(settings, unit)
    for name in ('proc.json', 'exit.json', 'proc.json.tmp', 'exit.json.tmp'):
        try:
            os.remove(os.path.join(d, name))
        except FileNotFoundError:
            pass


def launch(settings: Settings, spec: ProcSpec) -> Optional[str]:
    """Start the runner (`systemd-run`); return an error text, or None."""
    with _locked(settings, spec.unit) as got:
        if not got:
            return (f'runner failed: {spec.unit} is held by another f1tenth_ctl '
                    f'for more than {LOCK_TIMEOUT_S:.0f} s')
        rec = record(settings, spec.unit)
        if rec['runner_alive'] or rec['main_alive']:
            return f'runner failed: unit {spec.unit} is already running'
        _clear(settings, spec.unit)
        os.makedirs(unit_dir(settings, spec.unit), exist_ok=True)
        try:
            proc = subprocess.run(runner_command(settings, spec),
                                  env=unit_environment(settings, spec),
                                  stdin=subprocess.DEVNULL, capture_output=True, text=True,
                                  timeout=LAUNCH_TIMEOUT_S)
        except (OSError, subprocess.TimeoutExpired) as exc:
            return f'runner failed: {exc}'
        if proc.returncode != 0:
            tail = (proc.stderr.strip() or proc.stdout.strip()).splitlines()[-3:]
            return 'runner failed: ' + (' | '.join(tail) or f'exit {proc.returncode}')
    return None


def _tree(settings: Settings, unit: str, include_runner: bool) -> List[int]:
    rec = record(settings, unit)
    info = rec['info'] or {}
    pids = []
    if rec['runner_alive']:
        if include_runner:
            pids.append(int(info['runner_pid']))
        pids += runner.descendants(int(info['runner_pid']))
    elif rec['main_alive']:
        # The runner was killed: what is still below the main process.
        pids.append(int(info['pid']))
        pids += runner.descendants(int(info['pid']))
    return pids


def unit_pids(settings: Settings, unit: str) -> List[int]:
    """Return the runner and every process below it (the unit's "cgroup")."""
    return _tree(settings, unit, include_runner=True)


def kill(settings: Settings, unit: str, sig: int) -> List[int]:
    """`systemctl kill --signal=SIG`: every process of the unit, not the runner."""
    sent = []
    for pid in _tree(settings, unit, include_runner=False):
        try:
            os.kill(pid, sig)
            sent.append(pid)
        except (ProcessLookupError, PermissionError):
            pass
    return sent


def _wait_gone(pid: int, start, timeout: float) -> bool:
    deadline = time.monotonic() + timeout
    while runner.alive(pid, start):
        if time.monotonic() >= deadline:
            return False
        time.sleep(0.1)
    return True


def finish_stop(settings: Settings, unit: str) -> dict:
    """
    Make sure nothing of the unit is left, then drop its records.

    `systemctl stop` + `reset-failed`: SIGTERM to the runner, which sends it
    to the whole tree and SIGKILL after TimeoutStopSec; the runner itself is
    killed if it is still there after that.
    """
    out = {'forced': False}
    with _locked(settings, unit) as got:
        rec = record(settings, unit)
        info = rec['info'] or {}
        if rec['runner_alive']:
            rpid, rstart = int(info['runner_pid']), info.get('runner_start')
            try:
                os.kill(rpid, signal.SIGTERM)
            except ProcessLookupError:
                pass
            if not _wait_gone(rpid, rstart, kill_timeout(settings) + 3.0):
                out['forced'] = True
                for pid in _tree(settings, unit, include_runner=True):
                    try:
                        os.kill(pid, signal.SIGKILL)
                    except (ProcessLookupError, PermissionError):
                        pass
                _wait_gone(rpid, rstart, 2.0)
        elif rec['main_alive']:
            out['forced'] = True
            kill(settings, unit, signal.SIGKILL)
            _wait_gone(int(info['pid']), info.get('pid_start'), 2.0)
        if got:
            _reset_unlocked(settings, unit)
    return out


def _reset_unlocked(settings: Settings, unit: str) -> bool:
    rec = record(settings, unit)
    if rec['runner_alive'] or rec['main_alive']:
        return False
    _clear(settings, unit)
    return True


def reset_failed(settings: Settings, unit: str) -> bool:
    """`systemctl reset-failed`: forget how a unit that is not running ended."""
    with _locked(settings, unit) as got:
        return _reset_unlocked(settings, unit) if got else False


def _unit_names(settings: Settings) -> List[str]:
    names = set()
    try:
        for entry in os.listdir(proc_dir(settings)):
            if os.path.isdir(os.path.join(proc_dir(settings), entry)):
                names.add(entry)
    except OSError:
        pass
    return sorted(names)


def sweep(settings: Settings) -> Dict[str, dict]:
    """Stop every unit that still has a runner or a main process (the container's last step)."""
    out = {}
    for unit in _unit_names(settings):
        rec = record(settings, unit)
        if rec['runner_alive'] or rec['main_alive']:
            out[unit] = finish_stop(settings, unit)
    return out


def reset(settings: Settings, names=None) -> dict:
    """
    Start from a clean slate (a new container): every named process is "stopped".

    Records of units with nothing alive are dropped and the desired state
    becomes 'stopped', so a process the previous container ran does not show
    as crashed. A unit that is alive is left alone and reported.
    """
    names = list(names) if names is not None else list(MANAGED) + ['discovery', 'monitor']
    out = {'reset': [], 'alive': []}
    for name in names:
        spec = settings.procs.get(name) or ProcSpec(name, '', unit_prefix=settings.unit_prefix)
        if reset_failed(settings, spec.unit):
            units.write_desired(settings, name, 'stopped')
            out['reset'].append(name)
        else:
            out['alive'].append(name)
    return out


def check_items(settings: Settings, why: str) -> List[dict]:
    """Return `f1tenth_ctl check`'s items for this backend (in place of systemd_user, linger)."""
    items = [{'name': 'process_backend', 'ok': True, 'required': False,
              'value': f'proc ({why}): one runner per process, no systemd'}]
    workdir = settings.workdir
    err = None
    try:
        os.makedirs(workdir, exist_ok=True)
        ok = os.access(workdir, os.W_OK | os.X_OK)
    except OSError as exc:
        ok, err = False, exc.strerror
    item = {'name': 'workdir', 'ok': ok, 'value': workdir, 'required': True}
    if not ok:
        item['why'] = f'F1T_WORKDIR is not a writable directory: {err or "permission denied"}'
    items.append(item)
    return items
