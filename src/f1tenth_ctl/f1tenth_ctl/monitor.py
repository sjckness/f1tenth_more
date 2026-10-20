"""
The READY monitor as its own unit (f1t-monitor.service).

`status` has to answer in < 5 s. A one-shot ROS probe spends most of that
starting Python and rclpy and, on a loaded machine, matching its
subscriptions, so it reported a running stack as silent about half the time
(2026-10-20, Gazebo + stack on linus, load 17 on 8 cores). The monitor is
one long-running node that keeps the measurements current in a JSON
snapshot; `status` reads the file. It needs the discovery server, so
`connect` and `discovery up` start it, `status` restarts it when it is
missing, and `discovery down` and `killall` stop it.
"""

import hashlib
import json
import os
import shlex
import time
from typing import Optional

from . import units
from .config import ProcSpec, Settings, SOURCE_ROOT

NAME = 'monitor'
FRESH_S = 2.0      # an older snapshot means the monitor is stuck
WARMUP_S = 3.0     # matching + one full rate window after a start


def _paths(settings: Settings):
    return (os.path.join(settings.state_dir, 'monitor.json'),
            os.path.join(settings.state_dir, 'monitor.spec.json'))


def probe_spec(settings: Settings, thresholds: dict) -> dict:
    snap, _ = _paths(settings)
    spec = {'checks': thresholds['checks'], 'window_s': thresholds['window_s'],
            'use_sim_time': thresholds['use_sim_time'], 'out': snap, 'period_s': 0.5}
    spec['spec_id'] = hashlib.sha1(
        json.dumps(spec, sort_keys=True).encode()).hexdigest()[:12]
    return spec


def proc_spec(settings: Settings) -> ProcSpec:
    _, spec_path = _paths(settings)
    # A super client, to list the nodes (see cli._probe_env).
    cmd = (f'env ROS_SUPER_CLIENT=TRUE PYTHONPATH={shlex.quote(SOURCE_ROOT)}'
           '${PYTHONPATH:+:$PYTHONPATH} '
           f'python3 -m f1tenth_ctl.rosprobe monitor --spec-file {shlex.quote(spec_path)}')
    return ProcSpec(name=NAME, cmd=cmd, unit_prefix=settings.unit_prefix)


def state(settings: Settings) -> dict:
    return units.unit_states(settings, [NAME])[NAME]


def ensure(settings: Settings, thresholds: dict) -> dict:
    """Start the monitor, or restart it if the thresholds changed."""
    spec = probe_spec(settings, thresholds)
    snap_path, spec_path = _paths(settings)
    st = state(settings)
    if st['state'] == 'running':
        try:
            with open(spec_path) as f:
                if json.load(f).get('spec_id') == spec['spec_id']:
                    return {'ok': True, 'started': False, **st}
        except (OSError, ValueError):
            pass
        units.stop(settings, proc_spec(settings))
    os.makedirs(settings.state_dir, exist_ok=True)
    with open(spec_path, 'w') as f:
        json.dump(spec, f)
    try:
        os.remove(snap_path)
    except OSError:
        pass
    res = units.start(settings, proc_spec(settings), settle_s=0.0)
    return {'ok': res.get('ok', False), 'started': True, **res}


def why_not(settings: Settings) -> str:
    """Say why there is no snapshot: the unit state and its last log line."""
    st = state(settings)
    tail = units.log_tail(settings, NAME, 1)
    detail = f': {tail[-1][:120]}' if tail and st['state'] != 'running' else ''
    return f"monitor {st['state']}{detail}"


def stop(settings: Settings) -> dict:
    return units.stop(settings, proc_spec(settings))


def read(settings: Settings, spec_id: str, wait_s: float = 0.0) -> Optional[dict]:
    """Return the latest snapshot if it is fresh and for these thresholds."""
    snap_path, _ = _paths(settings)
    deadline = time.monotonic() + wait_s
    while True:
        try:
            with open(snap_path) as f:
                snap = json.load(f)
            if (snap.get('spec_id') == spec_id
                    and time.time() - snap.get('time', 0) <= FRESH_S):
                snap['age_s'] = round(time.time() - snap['time'], 2)
                snap['warming_up'] = time.time() - snap.get('started', 0) < WARMUP_S
                return snap
        except (OSError, ValueError):
            pass
        if time.monotonic() >= deadline:
            return None
        time.sleep(0.1)
