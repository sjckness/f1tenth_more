"""
Compare the env file's ROS environment with the target's shell.

A stack started by f1tenth_ctl and one started by hand from an ssh shell
should be the same stack, and `ros2 ...` in that shell should see the graph
the bot sees. Both hold only if the env file and the shell agree on the
variables below. `f1tenth_ctl check` compares them and warns on any
difference (it does not fail: the env file may differ on purpose).

The reference shell is F1T_SHELL_REF in the env file:
- `interactive` (the default): `bash -i`, i.e. the user's ~/.bashrc, which
  is what an ssh login and a terminal get;
- anything else: a bash snippet run with `bash -c` (for the sim: the sim
  scripts' own environment).
"""

import os
import subprocess
from typing import Dict, List, Optional

from .config import base_environment, Settings

# The ROS 2 middleware and workspace variables that decide which graph a
# process joins and which code it runs.
KEYS = ('ROS_DISTRO', 'ROS_DOMAIN_ID', 'ROS_LOCALHOST_ONLY', 'ROS_AUTOMATIC_DISCOVERY_RANGE',
        'ROS_STATIC_PEERS', 'ROS_DISCOVERY_SERVER', 'ROS_SUPER_CLIENT', 'RMW_IMPLEMENTATION',
        'FASTRTPS_DEFAULT_PROFILES_FILE', 'FASTDDS_DEFAULT_PROFILES_FILE', 'CYCLONEDDS_URI',
        'AMENT_PREFIX_PATH', 'LD_LIBRARY_PATH')
PATH_KEYS = ('AMENT_PREFIX_PATH', 'LD_LIBRARY_PATH')
# What an unset variable means (Humble and Jazzy).
DEFAULTS = {'ROS_DOMAIN_ID': '0', 'ROS_LOCALHOST_ONLY': '0',
            'RMW_IMPLEMENTATION': 'rmw_fastrtps_cpp'}
_MARK = '\0__F1T_SHELL_ENV__\0'


def normalize(env: Dict[str, str]) -> Dict[str, str]:
    out = {}
    for key in KEYS:
        value = env.get(key, '')
        if key == 'ROS_SUPER_CLIENT':
            value = 'TRUE' if value.strip().lower() in ('true', '1', 'yes', 'on') else ''
        elif not value and key in DEFAULTS:
            value = DEFAULTS[key]
        out[key] = value
    return out


def diff(env_file: Dict[str, str], shell: Dict[str, str]) -> List[dict]:
    """Return one entry per variable that differs, after normalize() (pure)."""
    a, b = normalize(env_file), normalize(shell)
    out = []
    for key in KEYS:
        if a[key] == b[key]:
            continue
        entry = {'key': key, 'env_file': a[key] or None, 'shell': b[key] or None}
        if key in PATH_KEYS:
            pa = [p for p in a[key].split(':') if p]
            pb = [p for p in b[key].split(':') if p]
            entry['only_env_file'] = [p for p in pa if p not in pb]
            entry['only_shell'] = [p for p in pb if p not in pa]
            if not entry['only_env_file'] and not entry['only_shell']:
                entry['order_differs'] = True
            del entry['env_file'], entry['shell']
        out.append(entry)
    return out


def shell_environment(settings: Settings, timeout: float = 20.0) -> Dict[str, str]:
    """Return the environment of the reference shell (see the module docstring)."""
    ref = settings.env.get('F1T_SHELL_REF', 'interactive').strip() or 'interactive'
    env = base_environment()
    env['TERM'] = 'dumb'
    # Snippets may use the env file's own settings (e.g. F1T_SIM_WS).
    env.update({k: v for k, v in settings.env.items() if k.startswith('F1T_')})
    # printf turns the two-character \\0 into the NUL bytes of _MARK.
    dump = "printf '\\0__F1T_SHELL_ENV__\\0'; env -0"
    if ref == 'interactive':
        argv = ['bash', '-i', '-c', dump]
    else:
        argv = ['bash', '-c', f'set -a; {ref}\nset +a; {dump}']
    proc = subprocess.run(argv, env=env, capture_output=True, timeout=timeout,
                          stdin=subprocess.DEVNULL, cwd=env.get('HOME'))
    raw = proc.stdout.decode(errors='replace')
    if _MARK not in raw:
        tail = proc.stderr.decode(errors='replace').strip().splitlines()[-2:]
        raise RuntimeError(f'reference shell gave no environment: {" | ".join(tail)}')
    out = {}
    for item in raw.split(_MARK, 1)[1].split('\0'):
        if '=' in item:
            key, _, value = item.partition('=')
            out[key] = value
    return out


def describe(settings: Settings) -> str:
    ref = settings.env.get('F1T_SHELL_REF', 'interactive').strip() or 'interactive'
    if ref == 'interactive':
        return f'bash -i (~{os.sep}.bashrc of {settings.env.get("USER", "this user")})'
    return 'F1T_SHELL_REF snippet'


def check(settings: Settings) -> Optional[dict]:
    """Return the check item for `f1tenth_ctl check`, or None if disabled."""
    if settings.env.get('F1T_SHELL_REF', '').strip().lower() in ('off', 'none'):
        return None
    try:
        differences = diff(settings.env, shell_environment(settings))
    except (OSError, RuntimeError, subprocess.TimeoutExpired) as exc:
        return {'name': 'shell_env', 'ok': False, 'required': False,
                'value': describe(settings), 'why': f'could not read it: {exc}'}
    item = {'name': 'shell_env', 'ok': not differences, 'required': False,
            'value': f'same ROS environment as {describe(settings)}' if not differences
            else f'{len(differences)} difference(s) with {describe(settings)}: '
                 + ', '.join(d['key'] for d in differences)}
    if differences:
        item['why'] = ('the bot and a shell on the target do not run the same ROS '
                       'environment: see differences')
        item['differences'] = differences
    return item
