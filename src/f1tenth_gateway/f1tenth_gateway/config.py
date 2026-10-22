"""
Gateway configuration: one YAML file, the token from the environment.

Every key has a default here; config/gateway.yaml (the car) and
config/gateway_mock.yaml (linus, no ROS) override what they need. Unknown
keys are an error, so a typo does not silently keep a default.

The bearer token is never in a file of the repo: it is read from the
environment variable named by auth.token_env (F1T_GATEWAY_TOKEN), which the
systemd unit loads from ~/.config/f1tenth_gateway/env (chmod 600).
"""

import copy
import os
from typing import Any, Dict, Optional

import yaml

from .teleop import TeleopConfig

DEFAULTS: Dict[str, Any] = {
    'listen': {
        # tailscale serve proxies https://<orin>.ts.net -> here. Not 8765
        # (foxglove_bridge) nor 8083 (llama-server); verify with `ss -ltn`.
        'host': '127.0.0.1',
        'port': 8770,
    },
    'mock': False,
    'www_dir': '~/.local/share/f1tenth_gateway/www',
    'ctl_env': None,                     # None: f1tenth_ctl's own default (config/ctl.env)
    'auth': {
        'token_env': 'F1T_GATEWAY_TOKEN',
        'min_token_len': 32,
        'max_failures_per_min': 5,
        'ws_auth_timeout_s': 2.0,
        'cookie_secure': True,           # false only for plain http in the mock tests
    },
    'client': {
        'lost_after_s': 1.5,             # no message for this long: the link is gone
        'ping_s': 0.5,
        'telemetry_hz': 10.0,
        'status_period_s': 1.0,
    },
    'teleop': {
        'max_speed_mps': 1.0,
        'default_limit_mps': 0.6,
        'limit_step_mps': 0.1,
        'deadzone': 0.15,
        'max_steer_rad': 0.30,
        'accel_mps2': 1.0,
        'steer_rate_radps': 1.5,
        'watchdog_s': 0.25,
        'zero_tail_s': 0.5,
        'publish_hz': 50.0,
        'max_input_hz': 50.0,
        # Reverse (teleop.py): the gas stick below the dead zone, linear to this
        # (the MPC's max_reverse_speed_mps; 0.0 = no reverse). A change of
        # direction holds 0 for reverse_hold_s and until |measured| is below
        # stop_speed_mps.
        'max_reverse_speed_mps': 0.4,
        'reverse_hold_s': 0.3,
        'stop_speed_mps': 0.05,
    },
    'mission': {
        'auto_abort_s': 2.0,
        'plan_timeout_s': 60.0,
        'plan_poll_s': 0.5,
        'max_text_len': 500,
    },
    'estop': {
        # f1tenth_ctl's own values come from its env file (CtlPort.estop_spec);
        # these are the gateway-only ones.
        'reset_still_s': 1.0,            # measured speed exactly 0 this long before a reset
        'reset_odom_max_age_s': 0.5,
    },
    'transcription': {
        'enabled': True,
        'model': 'small',                # base | small: CPU int8 on the Orin
        'language': 'it',
        'max_duration_s': 30.0,
        'max_bytes': 2_000_000,
        'cpu_threads': 4,
        'nice': 10,
        'timeout_s': 60.0,
        'idle_unload_s': 600.0,
    },
    'archive': {
        'root': '~/f1tenth_archive',
        'workspace': '~/dev_ws/f1tenth_more',
        'max_runs': 50,
    },
    'topics': {
        'teleop': '/teleop',
        'mission_status': '/mission/status',
        'move_outcome': '/mission/move_outcome',
        'battery': '/diagnostics/battery_status',
        'odom': '/odom',
        'pose_map': '/ekf_global/odometry/filtered',
        'pose_odom': '/odometry/filtered',
        'drive_out': '/ackermann_drive',
        'system': '/diagnostics/system_status',
        'tree_status': '/behavior/tree_status',
        'mpc_solver': '/mpc/solver_status',
        'map': '/slam/map',
        'vesc': '/sensors/core',
        'llm2_status': '/llm2/status',
    },
    'llm2': {
        # LLM2's pause / resume (two-llm-arch, output/two_llm/README.md section 2).
        'service': '/llm2/set_active',
        'timeout_s': 2.0,                # the SetBool call; no retry
        'stale_s': 5.0,                  # no /llm2/status this long: LLM2 not running
        'trust_s': 2.0,                  # one status period: the service answer wins
    },
    'teleop_frame_id': 'base_link/pwa_teleop',
    'bt_open_service': '/behavior_executor_node/snapshot_streams/open',
    'bt_close_service': '/behavior_executor_node/snapshot_streams/close',
}


class ConfigError(Exception):
    pass


def _merge(base: Dict, over: Dict, path: str = '') -> Dict:
    out = copy.deepcopy(base)
    for key, value in (over or {}).items():
        where = f'{path}{key}'
        if key not in base:
            raise ConfigError(f'unknown key {where!r}')
        if isinstance(base[key], dict):
            if not isinstance(value, dict):
                raise ConfigError(f'{where} must be a mapping')
            out[key] = _merge(base[key], value, where + '.')
        else:
            out[key] = value
    return out


def load(path: Optional[str] = None, overrides: Optional[Dict] = None) -> Dict:
    raw = {}
    if path:
        try:
            with open(path) as f:
                raw = yaml.safe_load(f) or {}
        except OSError as exc:
            raise ConfigError(f'cannot read {path}: {exc.strerror}') from exc
    cfg = _merge(DEFAULTS, raw)
    if overrides:
        cfg = _merge(cfg, overrides)
    cfg['teleop_cfg'] = TeleopConfig(**cfg['teleop'])
    base = os.path.dirname(os.path.abspath(path)) if path else os.getcwd()

    def resolve(p):
        p = os.path.expanduser(p)
        return p if os.path.isabs(p) else os.path.normpath(os.path.join(base, p))
    cfg['www_dir'] = resolve(cfg['www_dir'])
    cfg['archive']['root'] = resolve(cfg['archive']['root'])
    cfg['archive']['workspace'] = resolve(cfg['archive']['workspace'])
    if cfg['ctl_env']:
        cfg['ctl_env'] = resolve(cfg['ctl_env'])
    port = cfg['listen']['port']
    if not isinstance(port, int) or not 1 <= port <= 65535:
        raise ConfigError(f'listen.port must be 1..65535, got {port!r}')
    if cfg['transcription']['model'] not in ('tiny', 'base', 'small', 'medium'):
        raise ConfigError('transcription.model must be tiny, base, small or medium')
    return cfg


def token(cfg: Dict, environ=os.environ) -> str:
    """Return the bearer token from the environment, or raise if missing or short."""
    name = cfg['auth']['token_env']
    value = environ.get(name, '')
    if len(value) < int(cfg['auth']['min_token_len']):
        raise ConfigError(f'{name} is unset or shorter than {cfg["auth"]["min_token_len"]} '
                          'characters (see the README: read -rs, chmod 600)')
    return value
