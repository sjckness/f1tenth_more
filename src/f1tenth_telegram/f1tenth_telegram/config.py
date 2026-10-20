"""
The bot's configuration (config/telegram.yaml) and its token.

The YAML holds no secret. The token comes from TELEGRAM_BOT_TOKEN, or from
the env file named by telegram.token_env_file (default
~/.config/f1tenth_telegram/env, a `TELEGRAM_BOT_TOKEN=...` line), which must
not be readable by group or others.
"""

from dataclasses import dataclass, field
import os
import stat
from typing import Dict, List, Optional

PACKAGE_DIR = os.path.dirname(os.path.abspath(__file__))
SOURCE_ROOT = os.path.dirname(PACKAGE_DIR)

DEFAULT_TIMEOUTS = {
    'default': 20.0, 'reach': 8.0, 'status': 12.0, 'connect': 25.0, 'start': 30.0,
    'stop': 30.0, 'killall': 75.0, 'estop': 15.0,
}


class ConfigError(Exception):
    pass


@dataclass
class Target:
    name: str = 'Orin'
    host: str = 'jetson-orin'          # ssh host; 'local' runs f1tenth_ctl here
    ssh_key: Optional[str] = None
    ssh_options: List[str] = field(default_factory=list)
    connect_timeout_s: float = 5.0
    ctl_command: str = '~/dev_ws/f1tenth_more/src/f1tenth_ctl/bin/f1tenth_ctl'
    ctl_env: Optional[str] = None      # passed as --env (None: the ctl's default)
    ping: bool = True

    @property
    def local(self) -> bool:
        return self.host in ('local', '')


@dataclass
class Config:
    target: Target
    timeouts: Dict[str, float]
    allowed_user_ids: List[int]
    max_command_age_s: float = 10.0
    token_env_file: str = '~/.config/f1tenth_telegram/env'

    def timeout(self, cmd: str) -> float:
        return float(self.timeouts.get(cmd, self.timeouts['default']))


def default_config_path() -> str:
    env = os.environ.get('F1T_TELEGRAM_CONFIG')
    if env:
        return os.path.expanduser(env)
    return os.path.join(SOURCE_ROOT, 'config', 'telegram.yaml')


def from_dict(data: dict) -> Config:
    data = data or {}
    t = data.get('target') or {}
    known = set(Target.__dataclass_fields__)
    unknown = set(t) - known
    if unknown:
        raise ConfigError(f'target: unknown keys {sorted(unknown)}')
    target = Target(**t)
    if not target.ctl_command:
        raise ConfigError('target.ctl_command is empty')
    timeouts = dict(DEFAULT_TIMEOUTS)
    timeouts.update({k: float(v) for k, v in (data.get('timeouts_s') or {}).items()})
    tg = data.get('telegram') or {}
    try:
        ids = [int(x) for x in (tg.get('allowed_user_ids') or [])]
    except (TypeError, ValueError) as exc:
        raise ConfigError('telegram.allowed_user_ids must be numeric user ids') from exc
    return Config(target=target, timeouts=timeouts, allowed_user_ids=ids,
                  max_command_age_s=float(tg.get('max_command_age_s', 10.0)),
                  token_env_file=tg.get('token_env_file', '~/.config/f1tenth_telegram/env'))


def load(path: Optional[str] = None) -> Config:
    import yaml
    path = path or default_config_path()
    try:
        with open(path) as f:
            return from_dict(yaml.safe_load(f))
    except OSError as exc:
        raise ConfigError(f'cannot read {path}: {exc.strerror}') from exc


def read_token(cfg: Config, environ=None) -> str:
    """TELEGRAM_BOT_TOKEN from the environment, else from the token env file."""
    environ = os.environ if environ is None else environ
    if environ.get('TELEGRAM_BOT_TOKEN'):
        return environ['TELEGRAM_BOT_TOKEN'].strip()
    path = os.path.expanduser(cfg.token_env_file)
    try:
        st = os.stat(path)
    except OSError as exc:
        raise ConfigError(f'no TELEGRAM_BOT_TOKEN and no token file {path}') from exc
    if st.st_mode & (stat.S_IRWXG | stat.S_IRWXO):
        raise ConfigError(f'{path} is readable by others: chmod 600 {path}')
    with open(path) as f:
        for line in f:
            line = line.strip()
            if line.startswith('export '):
                line = line[len('export '):]
            key, sep, value = line.partition('=')
            if sep and key.strip() == 'TELEGRAM_BOT_TOKEN':
                token = value.strip().strip('"').strip("'")
                if token:
                    return token
    raise ConfigError(f'{path} has no TELEGRAM_BOT_TOKEN=... line')
