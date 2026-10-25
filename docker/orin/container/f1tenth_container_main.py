#!/usr/bin/env python3
"""
The car container's main process: preflight, discovery server, gateway, ordered stop.

docker/orin/compose.yaml runs this as the container's command under tini
(compose `init: true`): PID 1 forwards SIGTERM here and reaps every orphan,
the f1tenth_ctl runners included. Stdlib only. f1tenth_ctl is imported from
the workspace F1T_CTL_ENV belongs to, and every managed process goes through
it (its proc backend), exactly as `f1tenth_ctl` from a shell would run it.

Role `car` (F1T_CONTAINER_ROLE, the default):
1. Preflight; refuses (exit 3) unless every state directory is writable and
   sits on a mount (not in the image's own filesystem, where `down` would
   lose it), the ctl env file loads, the gateway's venv and token are there,
   no discovery server listens on its UDP port, nothing listens on the
   gateway's TCP port, and no live Fast DDS shared memory exists (another
   stack on this host). Warnings only: no ZED calibration file, no GGUF model.
2. Reset: every managed process is "stopped" and the runner records of the
   previous container are dropped (a new container starts nothing it did not
   start, and shows nothing as crashed).
3. The discovery server (`f1tenth_ctl discovery up`); exit 4 if it fails.
4. The gateway (src/f1tenth_gateway/bin/f1tenth_gateway), restart-always like
   its native unit (Restart=always, RestartSec=3; 30 s after 5 starts in a
   minute). The token file is parsed here and its values go into the
   gateway's environment only.
5. F1T_AUTOSTART=stack: `f1tenth_ctl start stack`. Default none: the stack is
   started from the PWA, Telegram or `f1tenth_ctl start stack`.
6. Waits. On SIGTERM/SIGINT the ordered stop: planner, logger, monitor and
   the stack (f1tenth_ctl stop, the usual ladder), then the gateway (SIGINT:
   it revokes the driving lease and publishes a last zero on /teleop; SIGKILL
   after 10 s), the discovery server, any runner left, any ROS process left,
   `fastdds shm clean`. Exit 0.
Any other role (the D3 harness): starts nothing, waits for SIGTERM.

Environment:
  F1T_CTL_ENV           f1tenth_ctl env file (default: the image's ctl_container.env)
  F1T_CONTAINER_ROLE    car | harness (default car)
  F1T_AUTOSTART         none | stack (default none)
  F1T_GATEWAY_VENV      the gateway's venv (default /opt/f1tenth_gateway/venv)
  F1T_GATEWAY_CONFIG    its yaml (default: gateway.yaml next to this file)
  F1T_GATEWAY_ENV_FILE  its token file, KEY=VALUE lines (default ~/.config/f1tenth_gateway/env)
  F1T_GATEWAY_ARGS      extra gateway arguments (off the car: `--mock --port N`)
  F1T_GATEWAY_LOG       its log (default ~/.local/state/f1tenth_gateway/gateway.log)
  F1T_GATEWAY_BIN       its launcher (default: src/f1tenth_gateway/bin/f1tenth_gateway)
  F1T_REQUIRED_MOUNTS   colon-separated paths that must be on a mount (default
                        DEFAULT_MOUNTS below; empty: none, e.g. a test run on linus)
  F1T_PREFLIGHT         strict (default) | warn: failed checks only warn
Exit: 0 after the ordered stop, 2 bad configuration, 3 preflight refused,
4 the discovery server did not start.
"""

from argparse import Namespace
from datetime import datetime
import fcntl
import glob
import os
import shlex
import signal
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
DEFAULT_CTL_ENV = '/opt/f1tenth_ws/src/f1tenth_ctl/config/ctl_container.env'
DEFAULT_VENV = '/opt/f1tenth_gateway/venv'
# State that must survive the container (compose mounts it from
# /scratch/fabiocar/f1tenth_container/ on the car).
DEFAULT_MOUNTS = ('~/f1tenth_archive', '~/.ros', '~/.local/state/f1tenth_ctl',
                  '~/.local/state/f1tenth_gateway', '~/.local/state/f1tenth_archive_trigger',
                  '/usr/local/zed/settings')
ZED_SETTINGS = '/usr/local/zed/settings'
MODELS_DIR = '~/projects/llm/models'
GATEWAY_PORT = 8770
TOKEN_ENV = 'F1T_GATEWAY_TOKEN'
MIN_TOKEN_LEN = 32
GATEWAY_STOP_S = 10.0       # the native unit's TimeoutStopSec (KillSignal=SIGINT)
RESTART_S = 3.0             # the native unit's RestartSec
BURST_STARTS, BURST_WINDOW_S, BURST_WAIT_S = 5, 60.0, 30.0
EXIT_CONFIG, EXIT_PREFLIGHT, EXIT_DISCOVERY = 2, 3, 4

_stop = []


def log(msg: str) -> None:
    stamp = datetime.now().astimezone().isoformat(timespec='seconds')
    print(f'[f1tenth_container_main] {stamp} {msg}', flush=True)


def _on_signal(signum, _frame):
    if not _stop:
        _stop.append(signal.Signals(signum).name)


def wait_for_stop(tick=None, period: float = 0.5) -> str:
    while not _stop:
        if tick is not None:
            tick()
        time.sleep(period)
    return _stop[0]


# ---- files and sockets (pure helpers, tested on linus) -------------------------------

def read_env_file(path: str) -> dict:
    """Parse a systemd EnvironmentFile (KEY=VALUE, # comments, optional quotes)."""
    out = {}
    with open(path) as f:
        for raw in f:
            line = raw.strip()
            if not line or line.startswith('#'):
                continue
            if line.startswith('export '):
                line = line[len('export '):].lstrip()
            key, sep, value = line.partition('=')
            key = key.strip()
            if not sep or not key or not key.replace('_', 'a').isalnum() or key[0].isdigit():
                continue
            value = value.strip()
            if len(value) >= 2 and value[0] == value[-1] and value[0] in '"\'':
                value = value[1:-1]
            out[key] = value
    return out


def _unescape(field: str) -> str:
    r"""Undo /proc/self/mountinfo's octal escapes (\040 is a space)."""
    out, i = [], 0
    while i < len(field):
        if field[i] == '\\' and i + 3 < len(field) and field[i + 1:i + 4].isdigit():
            out.append(chr(int(field[i + 1:i + 4], 8)))
            i += 4
        else:
            out.append(field[i])
            i += 1
    return ''.join(out)


def mount_points(mountinfo: str = '/proc/self/mountinfo') -> list:
    points = []
    try:
        with open(mountinfo) as f:
            for line in f:
                fields = line.split()
                if len(fields) > 4:
                    points.append(_unescape(fields[4]))
    except OSError:
        pass
    return points


def mount_of(path: str, points) -> str:
    """Return the mount point a path lives on (the longest matching prefix)."""
    path = os.path.realpath(path)
    best = '/'
    for point in points:
        if point == '/':
            continue
        if (path == point or path.startswith(point.rstrip('/') + '/')) and len(point) > len(best):
            best = point
    return best


def bound_ports(tables, port: int, state: str = None) -> list:
    """Return the socket inodes bound to `port` in /proc/net tables (TCP listen: '0A')."""
    want = f':{port:04X}'
    inodes = []
    for table in tables:
        try:
            with open(table) as f:
                next(f, None)
                for line in f:
                    cols = line.split()
                    if len(cols) <= 9 or not cols[1].endswith(want):
                        continue
                    if state is None or cols[3] == state:
                        inodes.append(int(cols[9]))
        except OSError:
            continue
    return inodes


def live_shm(shm_dir: str = '/dev/shm') -> tuple:
    """
    Return (live, stale) Fast DDS shared-memory lock files.

    A segment's or port's `_el` file is flocked by its owner for its whole
    life, so a lock we cannot take means a live participant: the test
    `fastdds shm clean` itself makes. The probe holds a free lock for an
    instant only.
    """
    live, stale = [], []
    for path in sorted(glob.glob(os.path.join(shm_dir, 'fastrtps_*_el'))
                       + glob.glob(os.path.join(shm_dir, 'fastdds_*_el'))):
        try:
            fd = os.open(path, os.O_RDONLY)
        except OSError:
            continue
        try:
            fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
            stale.append(path)
        except BlockingIOError:
            live.append(path)
        except OSError:
            pass
        finally:
            os.close(fd)
    return live, stale


def _writable_dir(path: str):
    """Create `path` if needed; return None if it is a writable directory, else why not."""
    try:
        os.makedirs(path, exist_ok=True)
    except OSError as exc:
        return exc.strerror
    if not os.access(path, os.W_OK | os.X_OK):
        return 'not writable'
    return None


def gateway_port(args) -> int:
    for i, arg in enumerate(args):
        if arg == '--port' and i + 1 < len(args):
            return int(args[i + 1])
        if arg.startswith('--port='):
            return int(arg.split('=', 1)[1])
    return GATEWAY_PORT


# ---- the run ------------------------------------------------------------------

class Config:
    """Everything this process reads from its environment, resolved once."""

    def __init__(self, environ):
        home = environ.get('HOME') or os.path.expanduser('~')
        self.home = home

        def expand(p):
            return os.path.join(home, p[2:]) if p.startswith('~/') else p
        self.expand = expand
        self.role = (environ.get('F1T_CONTAINER_ROLE') or 'car').strip()
        self.autostart = (environ.get('F1T_AUTOSTART') or 'none').strip()
        self.ctl_env = expand(environ.get('F1T_CTL_ENV') or DEFAULT_CTL_ENV)
        self.ctl_root = find_ctl_root(self.ctl_env)
        src = os.path.dirname(self.ctl_root)
        self.venv = expand(environ.get('F1T_GATEWAY_VENV') or DEFAULT_VENV)
        self.gw_config = expand(environ.get('F1T_GATEWAY_CONFIG')
                                or os.path.join(HERE, 'gateway.yaml'))
        self.gw_env_file = expand(environ.get('F1T_GATEWAY_ENV_FILE')
                                  or '~/.config/f1tenth_gateway/env')
        self.gw_args = shlex.split(environ.get('F1T_GATEWAY_ARGS') or '')
        self.gw_log = expand(environ.get('F1T_GATEWAY_LOG')
                             or '~/.local/state/f1tenth_gateway/gateway.log')
        self.gw_bin = expand(environ.get('F1T_GATEWAY_BIN')
                             or os.path.join(src, 'f1tenth_gateway', 'bin', 'f1tenth_gateway'))
        mounts = environ.get('F1T_REQUIRED_MOUNTS')
        self.mounts = ([expand(p) for p in mounts.split(':') if p] if mounts is not None
                       else [expand(p) for p in DEFAULT_MOUNTS])
        self.strict = (environ.get('F1T_PREFLIGHT') or 'strict').strip() != 'warn'


def find_ctl_root(ctl_env: str) -> str:
    """
    Return src/f1tenth_ctl of the workspace to run: the env file's, else this file's.

    The env file's own package when it is one of f1tenth_ctl's config files;
    else the workspace this file sits in (docker/orin/container/ of a
    checkout), else the image's /opt/f1tenth_ws.
    """
    candidates = [os.path.dirname(os.path.dirname(os.path.realpath(ctl_env))),
                  os.path.normpath(os.path.join(HERE, '..', '..', '..', 'src', 'f1tenth_ctl')),
                  '/opt/f1tenth_ws/src/f1tenth_ctl']
    for root in candidates:
        if os.path.isfile(os.path.join(root, 'f1tenth_ctl', 'units.py')):
            return root
    return candidates[-1]


def import_ctl(cfg: Config):
    """Import f1tenth_ctl from cfg.ctl_root (else whatever is installed)."""
    if os.path.isdir(os.path.join(cfg.ctl_root, 'f1tenth_ctl')):
        sys.path.insert(0, cfg.ctl_root)
    import f1tenth_ctl  # noqa: F401
    from f1tenth_ctl import cli, config, discovery, monitor, procscan, procunits, units
    return Namespace(cli=cli, config=config, discovery=discovery, monitor=monitor,
                     procscan=procscan, procunits=procunits, units=units)


def preflight(cfg: Config, settings, ctl) -> tuple:
    """Return (problems, warnings): problems refuse the start unless F1T_PREFLIGHT=warn."""
    problems, warnings = [], []
    points = mount_points()
    for path in cfg.mounts:
        if not os.path.isdir(path):
            problems.append(f'{path}: missing (expected a mount)')
        elif mount_of(path, points) == '/':
            problems.append(f'{path}: on the image filesystem, not a mount: data would be lost')
    trigger = settings.env.get('F1TENTH_ARCHIVE_TRIGGER')
    dirs = [settings.state_dir, settings.log_dir, settings.workdir,
            os.path.dirname(cfg.gw_log), os.path.join(cfg.home, '.ros'),
            os.path.join(cfg.home, 'f1tenth_archive')]
    if trigger:
        dirs.append(os.path.dirname(trigger))
    for path in dirs:
        why = _writable_dir(path)
        if why:
            problems.append(f'{path}: {why}')
    if not os.access(os.path.join(cfg.venv, 'bin', 'python'), os.X_OK):
        problems.append(f'gateway venv: no {cfg.venv}/bin/python')
    if not os.access(cfg.gw_bin, os.X_OK):
        problems.append(f'gateway launcher: {cfg.gw_bin} missing or not executable')
    if not os.path.isfile(cfg.gw_config):
        problems.append(f'gateway config: {cfg.gw_config} missing')
    try:
        token = read_env_file(cfg.gw_env_file).get(TOKEN_ENV, '')
        if len(token) < MIN_TOKEN_LEN:
            problems.append(f'{cfg.gw_env_file}: {TOKEN_ENV} unset or shorter than '
                            f'{MIN_TOKEN_LEN} characters')
    except OSError as exc:
        problems.append(f'gateway token file {cfg.gw_env_file}: {exc.strerror} '
                        f'({TOKEN_ENV} is read from it)')
    addr = settings.discovery_address
    if addr is not None and bound_ports(('/proc/net/udp', '/proc/net/udp6'), addr[1]):
        problems.append(f'UDP {addr[1]} is already bound: a discovery server runs (the '
                        'native stack?); the guard should have refused')
    port = gateway_port(cfg.gw_args)
    if bound_ports(('/proc/net/tcp', '/proc/net/tcp6'), port, '0A'):
        problems.append(f'TCP {port} already listens: the native gateway '
                        '(f1tenth-gateway.service)? stack_mode.sh stops it')
    live, stale = live_shm()
    if live:
        problems.append(f'{len(live)} live Fast DDS shared-memory segment(s) in /dev/shm '
                        f'(another DDS stack on this host), e.g. {live[0]}')
    if stale:
        warnings.append(f'{len(stale)} stale Fast DDS shared-memory file(s) in /dev/shm '
                        '(nobody holds them; the supervisor sweeps them)')
    if not glob.glob(os.path.join(ZED_SETTINGS, 'SN*.conf')):
        warnings.append(f'no ZED calibration in {ZED_SETTINGS} (SN*.conf): the SDK will '
                        'try to download it')
    models = cfg.expand(MODELS_DIR)
    if not glob.glob(os.path.join(models, '*.gguf')):
        warnings.append(f'no *.gguf in {models}: enable_intelligence cannot start llama-server')
    backend = ctl.units.backend_choice(settings)
    if backend['backend'] != 'proc':
        warnings.append(f"f1tenth_ctl backend is {backend['backend']} ({backend['why']}), "
                        'not proc')
    return problems, warnings


class Gateway:
    """The gateway as a child with the native unit's Restart=always."""

    def __init__(self, cfg: Config, token_env: dict):
        self.cfg = cfg
        self.env = dict(os.environ)
        self.env.update(F1T_CTL_ENV=cfg.ctl_env, F1T_GATEWAY_VENV=cfg.venv,
                        F1T_GATEWAY_CONFIG=cfg.gw_config)
        self.env.update(token_env)
        self.proc = None
        self.starts = []
        self.next_start = 0.0

    def start(self) -> None:
        os.makedirs(os.path.dirname(self.cfg.gw_log), exist_ok=True)
        with open(self.cfg.gw_log, 'ab') as out:
            out.write(f'[f1tenth_container_main] {datetime.now().isoformat()} start\n'.encode())
            out.flush()
            self.proc = subprocess.Popen(
                [self.cfg.gw_bin, *self.cfg.gw_args], env=self.env, cwd=self.cfg.home,
                stdin=subprocess.DEVNULL, stdout=out, stderr=subprocess.STDOUT,
                start_new_session=True)
        self.starts.append(time.monotonic())
        log(f'gateway started (pid {self.proc.pid}), log {self.cfg.gw_log}')

    def tick(self) -> None:
        now = time.monotonic()
        if self.proc is None:
            if now >= self.next_start:
                self.start()
            return
        code = self.proc.poll()
        if code is None:
            return
        self.proc = None
        self.starts = [t for t in self.starts if now - t < BURST_WINDOW_S]
        delay = BURST_WAIT_S if len(self.starts) >= BURST_STARTS else RESTART_S
        self.next_start = now + delay
        log(f'gateway exited ({code}); restart in {delay:.0f} s (see {self.cfg.gw_log})')

    def stop(self) -> dict:
        if self.proc is None or self.proc.poll() is not None:
            return {'ok': True, 'already': 'stopped', 'signals': []}
        signals = ['SIGINT']
        try:
            os.killpg(self.proc.pid, signal.SIGINT)
            self.proc.wait(GATEWAY_STOP_S)
        except ProcessLookupError:
            self.proc.wait(5.0)
        except subprocess.TimeoutExpired:
            signals.append('SIGKILL')
            try:
                os.killpg(self.proc.pid, signal.SIGKILL)
            except ProcessLookupError:
                pass
            self.proc.wait(5.0)
        return {'ok': True, 'signals': signals, 'exit': self.proc.returncode}


def _leftovers(settings, ctl) -> dict:
    """Signal every ROS process of this user still alive (killall's step 2)."""
    procs = ctl.procscan.ros_processes(ctl.procscan.all_procs(), os.getuid(),
                                       ctl.procscan.own_exclusions(), include_discovery=True)
    pids = [p.pid for p in procs]
    if not pids:
        return {'ok': True, 'pids': []}
    ladder = ctl.procscan.signal_ladder(pids, settings.killall_sigint_wait_s,
                                        settings.killall_sigterm_wait_s)
    return {'ok': not ladder['survivors'], 'pids': pids, **ladder}


def _shm_clean(settings) -> dict:
    try:
        proc = subprocess.run(['bash', '-c', 'fastdds shm clean'], env=settings.env,
                              capture_output=True, text=True, timeout=15)
    except (OSError, subprocess.TimeoutExpired) as exc:
        return {'ok': False, 'error': str(exc)}
    return {'ok': proc.returncode == 0, 'error': proc.stderr.strip()[-160:]}


def ordered_stop(settings, ctl, gateway) -> bool:
    """Stop everything, in the order that keeps the car safe; True if all went."""
    ok = True

    def step(name, fn):
        nonlocal ok
        t0 = time.monotonic()
        try:
            res = fn()
        except Exception as exc:  # noqa: BLE001 - every step must run
            res = {'ok': False, 'error': f'{type(exc).__name__}: {exc}'}
        ok = ok and bool(res.get('ok', True))
        extra = ' '.join(f'{k}={res[k]}' for k in ('already', 'signals', 'pids', 'error')
                         if res.get(k))
        log(f'stop {name}: ok={res.get("ok", True)} {extra} '
            f'({time.monotonic() - t0:.1f} s)')

    for name in ('planner', 'logger'):
        step(name, lambda n=name: ctl.cli.cmd_stop(settings, Namespace(target=n)))
    step('monitor', lambda: ctl.monitor.stop(settings))
    step('stack', lambda: ctl.cli.cmd_stop(settings, Namespace(target='stack')))
    step('gateway', gateway.stop if gateway is not None else (lambda: {'ok': True}))
    step('discovery', lambda: ctl.discovery.down(settings, force=True))
    step('runners', lambda: {'ok': True, 'swept': ctl.procunits.sweep(settings)})
    if ctl.units.in_container():
        # The container's own PID namespace: every ROS process seen is ours.
        step('leftovers', lambda: _leftovers(settings, ctl))
        step('shm', lambda: _shm_clean(settings))
    else:
        log('stop leftovers, shm: skipped (not in a container: the other ROS processes '
            'and segments of this user are not ours)')
    return ok


def run_car(cfg: Config) -> int:
    if not os.path.isfile(cfg.ctl_env):
        log(f'F1T_CTL_ENV {cfg.ctl_env}: no such file')
        return EXIT_CONFIG
    try:
        ctl = import_ctl(cfg)
        settings = ctl.config.load(cfg.ctl_env)
    except Exception as exc:  # noqa: BLE001 - ConfigError, or an import failure
        log(f'f1tenth_ctl configuration: {type(exc).__name__}: {exc}')
        return EXIT_CONFIG
    choice = ctl.units.backend_choice(settings)
    log(f'role car, env {settings.env_file}, backend {choice["backend"]} ({choice["why"]}), '
        f'autostart {cfg.autostart}')
    problems, warnings = preflight(cfg, settings, ctl)
    for text in warnings:
        log(f'preflight WARN: {text}')
    for text in problems:
        log(f'preflight {"FAIL" if cfg.strict else "WARN (F1T_PREFLIGHT=warn)"}: {text}')
    if problems and cfg.strict:
        log('refusing to start: fix the above (the container stops, nothing was started)')
        return EXIT_PREFLIGHT
    reset = ctl.procunits.reset(settings)
    log(f'reset: stopped {",".join(reset["reset"]) or "-"}; still alive '
        f'{",".join(reset["alive"]) or "-"}')
    gateway, code = None, 0
    try:
        disc = ctl.discovery.up(settings)
        if not disc.get('ok'):
            log(f'discovery server did not start: {disc.get("error")} '
                f'{disc.get("log_tail", "")}')
            code = EXIT_DISCOVERY
            return code
        log(f'discovery server {disc.get("address")} (pid {disc.get("pid")})')
        token_env = read_env_file(cfg.gw_env_file) if os.path.isfile(cfg.gw_env_file) else {}
        gateway = Gateway(cfg, token_env)
        gateway.start()
        if cfg.autostart == 'stack':
            try:
                res = ctl.cli.cmd_start(settings, Namespace(target='stack'))
            except Exception as exc:  # noqa: BLE001 - the container stays up without it
                res = {'ok': False, 'error': f'{type(exc).__name__}: {exc}'}
            log(f'autostart stack: ok={res.get("ok")} state={res.get("state")} '
                f'{res.get("error", "")}')
        elif cfg.autostart != 'none':
            log(f'F1T_AUTOSTART={cfg.autostart!r} unknown: nothing started (none | stack)')
        log('up')
        signame = wait_for_stop(gateway.tick)
        log(f'{signame}: ordered stop')
    except Exception as exc:  # noqa: BLE001 - whatever failed, nothing is left running
        log(f'failed: {type(exc).__name__}: {exc}: ordered stop')
        code = 1
    finally:
        ok = ordered_stop(settings, ctl, gateway)
        log(f'stopped ({"clean" if ok else "with errors, see above"})')
    return code


def main() -> int:
    for sig in (signal.SIGTERM, signal.SIGINT, signal.SIGHUP):
        signal.signal(sig, _on_signal)
    cfg = Config(os.environ)
    if cfg.role != 'car':
        log(f'role {cfg.role}: nothing to start, waiting for SIGTERM')
        log(f'{wait_for_stop()}: exit')
        return 0
    return run_car(cfg)


if __name__ == '__main__':
    sys.exit(main())
