"""Fix batch 1, item 1: watch a running system_observer_node for DURATION s.

- logs every /diagnostics/system_status message (receive time, temps, gpu load);
- runs a real IsSystemOverheated (stack_params values) on the same topic and
  ticks it at the BT's 10 Hz, recording every status / tripped_reason and any
  exception;
- once a second, reads the gpu-thermal zone exactly as the node's fallback
  does and records whether it was readable or failed (EAGAIN -> TypeError),
  so the run proves the failing read actually happened while the node lived.

  python3 system_observer_soak_monitor.py OUT_DIR DURATION_S
"""
import collections
import json
import os
import sys
import time

import py_trees
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from f1tenth_behavior.behaviours.is_system_overheated import IsSystemOverheated
from f1tenth_messages.msg import SystemStatus

out, duration = sys.argv[1], float(sys.argv[2])
params = yaml.safe_load(open(os.path.join(
    get_package_share_directory('f1tenth_params'), 'config', 'stack_params.yaml')))
p = {k: params[k]['default'] for k in (
    'sys_obs_max_temp_c', 'sys_obs_max_load_percent', 'enable_sys_obs_load_trip',
    'sys_obs_load_trip_consecutive_samples')}

GPU_ZONE = None
for z in sorted(os.listdir('/sys/devices/virtual/thermal')):
    if z.startswith('thermal_zone'):
        with open(f'/sys/devices/virtual/thermal/{z}/type') as f:
            if 'gpu' in f.read().lower():
                GPU_ZONE = f'/sys/devices/virtual/thermal/{z}/temp'
                break

rclpy.init()
node = rclpy.create_node('system_observer_soak_monitor')
cond = IsSystemOverheated(
    max_temp_c=p['sys_obs_max_temp_c'], max_load_percent=p['sys_obs_max_load_percent'],
    enable_load_trip=p['enable_sys_obs_load_trip'],
    load_trip_consecutive_samples=p['sys_obs_load_trip_consecutive_samples'])
cond.setup(node=node)

msgs = open(os.path.join(out, 'system_status.csv'), 'w')
msgs.write('t_recv,cpu_temp_c,gpu_temp_c,gpu_percent,emc_percent,cpu_percent,n_cores\n')
t0 = time.monotonic()


def on_status(m):
    msgs.write(f'{time.monotonic() - t0:.3f},{m.cpu_temp_c:.3f},{m.gpu_temp_c:.3f},'
               f'{m.gpu_percent:.3f},{m.emc_percent:.3f},{m.cpu_percent:.2f},'
               f'{len(m.cpu_per_core)}\n')
    msgs.flush()


node.create_subscription(SystemStatus, '/diagnostics/system_status', on_status, 10)

statuses = collections.Counter()
reasons = collections.Counter()
exceptions = []
zone = collections.Counter()
zone_log = open(os.path.join(out, 'gpu_zone_probe.csv'), 'w')
zone_log.write('t,result\n')


def tick():
    try:
        s = cond.update()
        statuses[s.name] += 1
        if cond.tripped_reason:
            reasons[cond.tripped_reason] += 1
    except Exception as exc:  # noqa: B902 -- recorded, that is the point
        exceptions.append(repr(exc))


def probe():
    try:
        with open(GPU_ZONE) as f:
            r = str(int(f.read().strip()))
        zone['readable'] += 1
    except TypeError:
        r = 'EAGAIN(TypeError)'
        zone['eagain'] += 1
    except (OSError, ValueError) as exc:
        r = type(exc).__name__
        zone['other_error'] += 1
    zone_log.write(f'{time.monotonic() - t0:.3f},{r}\n')
    zone_log.flush()


node.create_timer(0.1, tick)
node.create_timer(1.0, probe)
while time.monotonic() - t0 < duration:
    rclpy.spin_once(node, timeout_sec=0.1)

summary = {
    'duration_s': duration, 'gpu_zone': GPU_ZONE, 'is_system_overheated_params': p,
    'bt_ticks_by_status': dict(statuses), 'tripped_reasons': dict(reasons),
    'exceptions': exceptions, 'gpu_zone_probe': dict(zone),
}
json.dump(summary, open(os.path.join(out, 'monitor_summary.json'), 'w'), indent=2)
print(json.dumps(summary))
node.destroy_node()
rclpy.shutdown()
