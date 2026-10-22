"""
The env file and the process specs it defines.

One env file (config/ctl.env, or another one passed with --env or
F1T_CTL_ENV) is the whole configuration. It is a bash file: it sources ROS
and the workspace, sets the network variables, and defines the F1T_* values
read here. Every process f1tenth_ctl starts sources the same file: the
systemd units source it themselves, and short-lived helpers (the ROS probe,
`ros2 daemon stop`) get the environment it produced.

The file is sourced on top of a minimal base environment, not the caller's,
so a stack started from an ssh session, from the bot or from a terminal that
already sourced something else gets the same environment.
"""

from dataclasses import dataclass, field
import os
import shlex
import subprocess
from typing import Dict, List, Optional

PACKAGE_DIR = os.path.dirname(os.path.abspath(__file__))
SOURCE_ROOT = os.path.dirname(PACKAGE_DIR)

# Kept from the caller; everything else comes from the env file.
BASE_KEYS = ('HOME', 'USER', 'LOGNAME', 'LANG', 'LC_ALL', 'TERM', 'SHELL',
             'XDG_RUNTIME_DIR', 'DBUS_SESSION_BUS_ADDRESS', 'TZ')
BASE_PATH = '/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin'

MANAGED = ('stack', 'planner', 'logger')


class ConfigError(Exception):
    """The env file is missing, fails to source, or lacks a value."""


def default_env_file() -> str:
    """
    Pick the env file: --env, else F1T_CTL_ENV, else config/ctl.env.

    From a source checkout that is src/f1tenth_ctl/config/ctl.env; from an
    install space it is share/f1tenth_ctl/config/ctl.env.
    """
    env = os.environ.get('F1T_CTL_ENV')
    if env:
        return os.path.expanduser(env)
    candidates = [os.path.join(SOURCE_ROOT, 'config', 'ctl.env')]
    try:
        from ament_index_python.packages import get_package_share_directory
        candidates.append(os.path.join(
            get_package_share_directory('f1tenth_ctl'), 'config', 'ctl.env'))
    except Exception:  # noqa: BLE001 - no ament index outside a sourced shell
        pass
    for path in candidates:
        if os.path.isfile(path):
            return path
    return candidates[0]


def base_environment() -> Dict[str, str]:
    env = {k: os.environ[k] for k in BASE_KEYS if k in os.environ}
    home = env.get('HOME', os.path.expanduser('~'))
    env['HOME'] = home
    env['PATH'] = f'{home}/.local/bin:{BASE_PATH}'
    if 'XDG_RUNTIME_DIR' not in env and os.path.isdir(f'/run/user/{os.getuid()}'):
        env['XDG_RUNTIME_DIR'] = f'/run/user/{os.getuid()}'
    return env


def source_env_file(path: str, timeout: float = 20.0) -> Dict[str, str]:
    """Source `path` in bash on the base environment and return the result."""
    if not os.path.isfile(path):
        raise ConfigError(f'env file not found: {path}')
    script = 'set -a; . "$1" >&2 || exit 97; set +a; env -0'
    try:
        proc = subprocess.run(
            ['bash', '-c', script, 'f1tenth_ctl', path], env=base_environment(),
            capture_output=True, timeout=timeout)
    except subprocess.TimeoutExpired as exc:
        raise ConfigError(f'sourcing {path} took more than {timeout:.0f} s') from exc
    if proc.returncode != 0:
        tail = proc.stderr.decode(errors='replace').strip().splitlines()[-3:]
        raise ConfigError(f'sourcing {path} failed (exit {proc.returncode}): '
                          + ' | '.join(tail))
    env = {}
    for item in proc.stdout.split(b'\0'):
        if b'=' in item:
            key, _, value = item.partition(b'=')
            env[key.decode()] = value.decode(errors='replace')
    env['F1T_CTL_ENV'] = os.path.abspath(path)
    return env


@dataclass
class ProcSpec:
    """One managed process: what to run and how to recognise it."""

    name: str
    cmd: str
    nodes: List[str] = field(default_factory=list)
    match: Optional[str] = None      # regex over /proc cmdlines (started by hand)
    stdin_fifo: bool = False
    unit_prefix: str = 'f1t-'

    @property
    def unit(self) -> str:
        return f'{self.unit_prefix}{self.name}.service'


@dataclass
class Settings:
    env_file: str
    env: Dict[str, str]
    procs: Dict[str, ProcSpec]
    unit_prefix: str
    graph_check: bool
    monitor_enabled: bool
    state_dir: str
    ready_file: str
    discovery_server: Optional[str]          # 'address:port', or None
    discovery_id: str
    sigint_wait_s: float
    sigterm_wait_s: float
    killall_sigint_wait_s: float
    killall_sigterm_wait_s: float
    estop_topic: str
    estop_service: str
    estop_service_timeout_s: float
    estop_publish_s: float
    estop_publish_hz: float
    estop_abort_timeout_s: float
    estop_reset_service: str
    restart_service: str
    behavior_component: str
    abort_service: str
    status_topic: str
    estop_reset_wait_s: float
    mission_max_age_s: float
    mission_countdown_s: float
    mission_discover_s: float

    @property
    def log_dir(self) -> str:
        return os.path.join(self.state_dir, 'logs')

    def log_path(self, name: str) -> str:
        return os.path.join(self.log_dir, f'{name}.log')

    def fifo_path(self, name: str) -> str:
        return os.path.join(self.state_dir, f'{name}.stdin')

    @property
    def discovery_address(self):
        """(address, port) of the discovery server, or None."""
        if not self.discovery_server:
            return None
        first = self.discovery_server.split(';')[0].strip()
        host, _, port = first.rpartition(':')
        return host or '127.0.0.1', int(port)


def _float(env, key, default):
    try:
        return float(env.get(key, default))
    except ValueError as exc:
        raise ConfigError(f'{key}={env.get(key)!r} is not a number') from exc


def _flag(env, key, default=True):
    value = env.get(key)
    if value is None or value == '':
        return default
    return value.lower() not in ('0', 'false', 'no', 'off')


def _resolve(env_file: str, path: str) -> str:
    path = os.path.expanduser(path)
    if not os.path.isabs(path):
        path = os.path.join(os.path.dirname(os.path.abspath(env_file)), path)
    return path


def settings_from_env(env_file: str, env: Dict[str, str]) -> Settings:
    # Tests use their own prefix so they can never touch the real units.
    prefix = env.get('F1T_UNIT_PREFIX', 'f1t-')
    procs = {}
    for name in MANAGED:
        key = name.upper()
        cmd = env.get(f'F1T_{key}_CMD', '').strip()
        if not cmd:
            raise ConfigError(f'F1T_{key}_CMD is not set in {env_file}')
        procs[name] = ProcSpec(
            name=name, cmd=cmd,
            nodes=shlex.split(env.get(f'F1T_{key}_NODES', '')),
            match=env.get(f'F1T_{key}_MATCH') or None,
            stdin_fifo=env.get(f'F1T_{key}_STDIN', '') == 'fifo',
            unit_prefix=prefix)
    home = env.get('HOME', os.path.expanduser('~'))
    state_dir = os.path.expanduser(
        env.get('F1T_STATE_DIR') or os.path.join(home, '.local/state/f1tenth_ctl'))
    ready = env.get('F1T_READY_FILE', 'ready.yaml')
    return Settings(
        env_file=os.path.abspath(env_file),
        env=env,
        procs=procs,
        unit_prefix=prefix,
        graph_check=_flag(env, 'F1T_GRAPH_CHECK'),
        monitor_enabled=_flag(env, 'F1T_MONITOR'),
        state_dir=state_dir,
        ready_file=_resolve(env_file, ready),
        discovery_server=env.get('ROS_DISCOVERY_SERVER') or None,
        discovery_id=env.get('F1T_DISCOVERY_SERVER_ID', '0'),
        sigint_wait_s=_float(env, 'F1T_STOP_SIGINT_WAIT_S', 10.0),
        sigterm_wait_s=_float(env, 'F1T_STOP_SIGTERM_WAIT_S', 5.0),
        killall_sigint_wait_s=_float(env, 'F1T_KILLALL_SIGINT_WAIT_S', 5.0),
        killall_sigterm_wait_s=_float(env, 'F1T_KILLALL_SIGTERM_WAIT_S', 3.0),
        estop_topic=env.get('F1T_ESTOP_TOPIC', '/safety_stop'),
        estop_service=env.get('F1T_ESTOP_SERVICE', '/mission/emergency_stop'),
        estop_service_timeout_s=_float(env, 'F1T_ESTOP_SERVICE_TIMEOUT_S', 0.5),
        estop_publish_s=_float(env, 'F1T_ESTOP_PUBLISH_S', 2.0),
        estop_publish_hz=_float(env, 'F1T_ESTOP_PUBLISH_HZ', 50.0),
        estop_abort_timeout_s=_float(env, 'F1T_ESTOP_ABORT_TIMEOUT_S', 1.0),
        # estop-reset: a service that clears the latch in place (none exists
        # today), else the supervisor restarts 'behavior', else a stack restart.
        estop_reset_service=env.get('F1T_ESTOP_RESET_SERVICE', ''),
        restart_service=env.get('F1T_SUPERVISOR_RESTART_SERVICE', '/restart_component'),
        behavior_component=env.get('F1T_BEHAVIOR_COMPONENT', 'behavior'),
        abort_service=env.get('F1T_MISSION_ABORT_SERVICE', '/mission/abort_mission'),
        status_topic=env.get('F1T_MISSION_STATUS_TOPIC', '/mission/status'),
        estop_reset_wait_s=_float(env, 'F1T_ESTOP_RESET_WAIT_S', 60.0),
        # A stored plan older than this is not loaded or started: plan again.
        mission_max_age_s=_float(env, 'F1T_MISSION_MAX_AGE_S', 300.0),
        # f1tenth_behavior's mission_countdown_sec, for telling "starting" apart:
        # 0.0 like stack_params.yaml since 2026-10-21. Only a fallback: a start
        # takes the countdown from the loader's own answer
        # (missions.armed_countdown).
        mission_countdown_s=_float(env, 'F1T_MISSION_COUNTDOWN_S', 0.0),
        # How long a fresh participant gets to receive /mission/status before
        # mission-load/start/status give up. Under the discovery server this is
        # NOT quick: measured on the car 2026-10-21, a new node needed between
        # 6 and 10 s for the latched sample, while the budget was 3 s -- so
        # every mission-load refused with "no /mission/status: is behavior
        # running?" although the BT was publishing it the whole time. The
        # callers' own timeout is 25 s, so this had room to spare.
        mission_discover_s=_float(env, 'F1T_MISSION_DISCOVER_S', 15.0),
    )


def load(env_file: Optional[str] = None) -> Settings:
    path = env_file or default_env_file()
    return settings_from_env(path, source_env_file(path))
