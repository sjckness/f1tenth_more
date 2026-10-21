#!/usr/bin/env python3
"""
f1tenth_comms synthetic run: both nodes as on the car, everything around them faked.

Needs a sourced workspace with f1tenth_comms and f1tenth_messages built. Runs
wifi_coverage_node and link_manager as their own processes (installed
executables, the package's comms.yaml plus overrides), and fakes:
- TF: map -> odom identity, odom -> base_link driving a 10 m x 3 m
  rectangle at 0.5 m/s (parked 40-46 s), at --tf-hz (default 100 Hz, the
  car's /tf rate);
- the Wi-Fi: a /proc/net/wireless file with RSSI = -38 - 4.5 * (distance
  from the map origin) dBm, and no line at all (not associated) for x > 8 m;
- the base: two heartbeat responders (scripts/base/heartbeat_responder.py's
  own code), "tailscale" on 127.0.0.1 and "local" on 127.0.0.2, same port,
  silenced or made lossy on a schedule;
- nmcli: a script that logs every call and answers "disconnected", so each
  reconnect attempt reaches `connection up`.

Schedule (s since start):
    0-20   both paths answer                 -> OK via tailscale
    20-35  tailscale silent                  -> OK via local
    35-50  both silent                       -> LOST, reconnect attempts
    50-65  both answer, 35 % dropped         -> DEGRADED
    65-     both answer                       -> OK, back on tailscale after failback_s
    80     /comms/coverage_reset             -> the grids start again

It checks the state sequence, the reconnect rate limit (from the fake
nmcli's log), the grids (no-link cells where they must be, -1 elsewhere,
publish rate) and the reset, and measures each node's CPU from
/proc/<pid>/stat. Prints a JSON summary; --out writes it to a file.

Only loopback is used. Run it in the f1tenth_more:cpu container with no
network (tools/test_comms_humble.sh ... -- python3 /work/src/tools/...), or
on linus with ROS_DOMAIN_ID and ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST set.
"""

import argparse
import importlib.util
import json
import math
import os
import random
import signal
import socket
import stat
import subprocess
import sys
import tempfile
import threading
import time

import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
RESPONDER = os.path.join(HERE, '..', 'scripts', 'base', 'heartbeat_responder.py')
HEADER = ('Inter-| sta-|   Quality        |   Discarded packets               | Missed | WE\n'
          ' face | tus | link level noise |  nwid  crypt   frag  retry   misc | beacon | 22\n')
FAKE_NMCLI = r"""#!/usr/bin/env python3
import sys, time
args = sys.argv[1:]
with open(__file__ + '.log', 'a') as f:
    f.write('%.3f %s\n' % (time.time(), ' '.join(args)))
if 'device' in args and 'show' in args:
    print('GENERAL.STATE:30 (disconnected)')
    print('GENERAL.CONNECTION:')
elif 'connection' in args and 'show' in args:
    print('802-11-wireless.ssid:f1tenth-base')
elif 'up' in args:
    print('Connection successfully activated')
"""

RECT = (10.0, 3.0)
SPEED = 0.5
STOP = (40.0, 46.0)     # parked, while LOST: only the time trigger can fire
DEAD_X = 8.0
PHASES = [(0.0, 'both'), (20.0, 'tailscale_silent'), (35.0, 'both_silent'),
          (50.0, 'lossy'), (65.0, 'both')]
RESET_AT = 80.0
CLK_TCK = os.sysconf('SC_CLK_TCK')


def pose_at(t):
    """Return (x, y, yaw) on the rectangle after t seconds at SPEED, parked during STOP."""
    w, h = RECT
    moving = t - min(max(t - STOP[0], 0.0), STOP[1] - STOP[0])
    s = (moving * SPEED) % (2 * (w + h))
    if s < w:
        return s, 0.0, 0.0
    if s < w + h:
        return w, s - w, math.pi / 2
    if s < 2 * w + h:
        return w - (s - w - h), h, math.pi
    return 0.0, h - (s - 2 * w - h), -math.pi / 2


def rssi_at(x, y):
    return -38.0 - 4.5 * math.hypot(x, y)


def phase_at(t):
    name = PHASES[0][1]
    for start, n in PHASES:
        if t >= start:
            name = n
    return name


def cpu_seconds(pid):
    with open(f'/proc/{pid}/stat') as f:
        fields = f.read().rsplit(')', 1)[1].split()
    return (int(fields[11]) + int(fields[12])) / CLK_TCK     # utime + stime


def rss_mb(pid):
    with open(f'/proc/{pid}/status') as f:
        for line in f:
            if line.startswith('VmRSS:'):
                return int(line.split()[1]) / 1024.0
    return None


def load_responder():
    spec = importlib.util.spec_from_file_location('heartbeat_responder', RESPONDER)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


class Base:
    """Two responders with the schedule's silence and drops."""

    def __init__(self, mod, port, t0):
        self.mod, self.t0 = mod, t0
        self.stop = False
        self.rng = random.Random(7)
        self.echoed = {'tailscale': 0, 'local': 0}
        self.threads, self.socks = [], []
        for name, addr in (('tailscale', '127.0.0.1'), ('local', '127.0.0.2')):
            socks = mod.open_sockets([addr], port)
            self.socks += socks
            th = threading.Thread(target=mod.serve, daemon=True, kwargs={
                'socks': socks, 'log_every': 0, 'stop': lambda: self.stop,
                'reply': self._reply_fn(name)})
            th.start()
            self.threads.append(th)

    def _reply_fn(self, name):
        def reply(data):
            phase = phase_at(time.monotonic() - self.t0)
            if phase == 'both_silent' or (phase == 'tailscale_silent' and name == 'tailscale'):
                return None
            if phase == 'lossy' and self.rng.random() < 0.35:
                return None
            out = self.mod.reply_for(data)
            if out is not None:
                self.echoed[name] += 1
            return out
        return reply

    def close(self):
        self.stop = True
        for th in self.threads:
            th.join(timeout=2.0)
        for s in self.socks:
            s.close()


def write_params(path, base_yaml, proc, port, nmcli):
    with open(base_yaml) as f:
        params = yaml.safe_load(f)
    cov = params['wifi_coverage_node']['ros__parameters']
    cov.update({'wifi_interface': 'fakewlan0', 'proc_net_wireless': proc,
                'use_iw_fallback': False})
    lm = params['link_manager']['ros__parameters']
    lm.update({'tailscale_host': '127.0.0.1', 'local_host': '127.0.0.2', 'port': port,
               'wifi_interface': 'fakewlan0', 'proc_net_wireless': proc,
               'use_iw_fallback': False, 'nmcli': nmcli})
    with open(path, 'w') as f:
        yaml.safe_dump(params, f)
    return lm


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    ap.add_argument('--duration', type=float, default=95.0)
    ap.add_argument('--tf-hz', type=float, default=100.0,
                    help='odom -> base_link rate; the car had 19-105 /tf messages/s in '
                         'the 2026-10-16/19 bags')
    ap.add_argument('--out', default='')
    ap.add_argument('--work', default='',
                    help='scratch dir for the fake proc file, nmcli and node logs '
                         '(default: a new one under $TMPDIR)')
    args = ap.parse_args()

    import rclpy
    from ament_index_python.packages import get_package_prefix, get_package_share_directory
    from geometry_msgs.msg import TransformStamped
    from nav_msgs.msg import OccupancyGrid
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
    from std_msgs.msg import String
    from std_srvs.srv import Trigger
    from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster

    from f1tenth_messages.msg import LinkStatus

    latched = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                         reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
    work = args.work or tempfile.mkdtemp(prefix='comms_synth_')
    os.makedirs(work, exist_ok=True)
    proc = os.path.join(work, 'wireless')
    nmcli = os.path.join(work, 'nmcli')
    with open(nmcli, 'w') as f:
        f.write(FAKE_NMCLI)
    if os.path.exists(nmcli + '.log'):         # a reused --work: count this run only
        os.remove(nmcli + '.log')
    os.chmod(nmcli, os.stat(nmcli).st_mode | stat.S_IEXEC)
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
        s.bind(('127.0.0.1', 0))
        port = s.getsockname()[1]
    params_file = os.path.join(work, 'params.yaml')
    lm_params = write_params(
        params_file, os.path.join(get_package_share_directory('f1tenth_comms'), 'config',
                                  'comms.yaml'), proc, port, nmcli)

    def write_proc(x, y):
        text = HEADER
        if x <= DEAD_X:
            text += f'fakewlan0: 0000   50.  {rssi_at(x, y):.0f}.  -256   0  0  0  0  0  0\n'
        tmp = proc + '.tmp'
        with open(tmp, 'w') as f:
            f.write(text)
        os.replace(tmp, proc)

    write_proc(0.0, 0.0)
    rclpy.init()
    node = Node('comms_synthetic_run')
    tfb = TransformBroadcaster(node)
    static = StaticTransformBroadcaster(node)
    st = TransformStamped()
    st.header.stamp = node.get_clock().now().to_msg()
    st.header.frame_id, st.child_frame_id = 'map', 'odom'
    st.transform.rotation.w = 1.0
    static.sendTransform(st)

    t0 = time.monotonic()
    rec = {'status': [], 'rssi': [], 'rtt': [], 'cells': []}

    def on_status(m):
        rec['status'].append((time.monotonic() - t0, m))

    node.create_subscription(LinkStatus, '/comms/link_status', on_status, 50)
    node.create_subscription(OccupancyGrid, '/comms/coverage_rssi',
                             lambda m: rec['rssi'].append((time.monotonic() - t0, m)), latched)
    node.create_subscription(OccupancyGrid, '/comms/coverage_rtt',
                             lambda m: rec['rtt'].append((time.monotonic() - t0, m)), latched)
    node.create_subscription(String, '/comms/coverage_cells',
                             lambda m: rec['cells'].append((time.monotonic() - t0,
                                                            json.loads(m.data))), latched)

    def tf_tick():
        t = time.monotonic() - t0
        x, y, yaw = pose_at(t)
        msg = TransformStamped()
        msg.header.stamp = node.get_clock().now().to_msg()
        msg.header.frame_id, msg.child_frame_id = 'odom', 'base_link'
        msg.transform.translation.x, msg.transform.translation.y = x, y
        msg.transform.rotation.z, msg.transform.rotation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        tfb.sendTransform(msg)

    def proc_tick():
        write_proc(*pose_at(time.monotonic() - t0)[:2])

    node.create_timer(1.0 / args.tf_hz, tf_tick)
    node.create_timer(0.1, proc_tick)
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    spin = threading.Thread(target=executor.spin, daemon=True)
    spin.start()

    base = Base(load_responder(), port, t0)
    prefix = os.path.join(get_package_prefix('f1tenth_comms'), 'lib', 'f1tenth_comms')
    procs = {}
    for exe in ('wifi_coverage_node', 'link_manager'):
        procs[exe] = subprocess.Popen(
            [os.path.join(prefix, exe), '--ros-args', '--params-file', params_file],
            stdout=open(os.path.join(work, exe + '.log'), 'w'), stderr=subprocess.STDOUT)
    cpu_marks = []

    def mark(label):
        cpu_marks.append((label, time.monotonic() - t0,
                          {k: cpu_seconds(p.pid) for k, p in procs.items()}))

    time.sleep(3.0)                     # imports and first samples, outside the CPU windows
    mark('start')
    reset_result = None
    while time.monotonic() - t0 < args.duration:
        t = time.monotonic() - t0
        if t >= 20.0 and len(cpu_marks) == 1:
            mark('steady_end')
        if reset_result is None and t >= RESET_AT:
            cli = node.create_client(Trigger, '/comms/coverage_reset')
            cli.wait_for_service(timeout_sec=5.0)
            fut = cli.call_async(Trigger.Request())
            while not fut.done():
                time.sleep(0.05)
            reset_result = (round(t, 2), fut.result().success, fut.result().message)
        time.sleep(0.1)
    mark('end')
    rss = {k: rss_mb(p.pid) for k, p in procs.items()}
    for p in procs.values():
        p.send_signal(signal.SIGINT)
    for p in procs.values():
        try:
            p.wait(timeout=10)
        except subprocess.TimeoutExpired:
            p.kill()
    base.close()
    executor.shutdown()
    node.destroy_node()
    rclpy.shutdown()

    summary = analyse(rec, cpu_marks, rss, reset_result, nmcli + '.log', lm_params, base,
                      args.tf_hz)
    summary['work_dir'] = work
    text = json.dumps(summary, indent=1)
    print(text)
    if args.out:
        with open(args.out, 'w') as f:
            f.write(text + '\n')
    return 0 if all(c['ok'] for c in summary['checks']) else 1


def analyse(rec, cpu_marks, rss, reset_result, nmcli_log, lm, base, tf_hz):
    checks = []

    def check(name, ok, detail):
        checks.append({'name': name, 'ok': bool(ok), 'detail': detail})

    # ---- link state over time
    timeline, last = [], None
    for t, m in rec['status']:
        key = (m.state_name, m.path)
        if key != last:
            timeline.append({'t': round(t, 2), 'state': m.state_name, 'path': m.path,
                             'rtt_ms': None if math.isnan(m.rtt_ms) else round(m.rtt_ms, 2),
                             'loss_pct': None if math.isnan(m.loss_pct)
                             else round(m.loss_pct, 1)})
            last = key

    def states_between(a, b):
        return {(m.state_name, m.path) for t, m in rec['status'] if a <= t < b}

    check('OK via tailscale while both answer (5-20 s)',
          states_between(5, 20) == {('OK', 'tailscale')}, sorted(states_between(5, 20)))
    s = states_between(23, 35)
    check('local takes over when tailscale is silent (23-35 s)',
          ('OK', 'local') in s and not any(p == 'tailscale' for _, p in s), sorted(s))
    s = states_between(38, 49.5)
    check('LOST when both are silent (38-49.5 s)', s == {('LOST', '')}, sorted(s))
    s = states_between(53, 65)
    check('DEGRADED with 35 % loss (53-65 s)',
          any(st == 'DEGRADED' for st, _ in s) and not any(st == 'LOST' for st, _ in s),
          sorted(s))
    end = rec['status'][-1][1]
    check('OK via tailscale at the end', (end.state_name, end.path) == ('OK', 'tailscale'),
          [end.state_name, end.path])

    # ---- reconnect attempts, from the fake nmcli's own log
    lines = [ln.split(' ', 1) for ln in open(nmcli_log).read().splitlines()] \
        if os.path.exists(nmcli_log) else []
    ups = [float(t) for t, cmd in lines if ' up ' in f' {cmd} ']
    gaps = [round(b - a, 2) for a, b in zip(ups, ups[1:])]
    triggers = [m.last_reconnect_trigger for _, m in rec['status'] if m.last_reconnect_trigger]
    attempts_lost = [(t, m.reconnect_attempts) for t, m in rec['status']]
    first_lost = next((t for t, m in rec['status'] if m.state_name == 'LOST'), None)
    last_lost = max((t for t, m in rec['status'] if m.state_name == 'LOST'), default=None)
    outside = [n for t, n in attempts_lost
               if first_lost is not None and (t < first_lost - 0.6)] or [0]
    # The gaps are between the fake nmcli's own process starts: the read-only
    # calls before each `up` (one more on the first attempt) shift them by tens of ms.
    check('reconnect attempts only while LOST, never closer than retry_min_interval_s; '
          'new_cell while moving, time while parked',
          ups and min(gaps or [99]) >= lm['retry_min_interval_s'] - 0.2 and max(outside) == 0
          and {'time', 'new_cell'} <= set(triggers),
          {'nmcli_up_calls': len(ups), 'gaps_s': gaps,
           'triggers_seen': sorted(set(triggers)), 'nmcli_calls_total': len(lines),
           'lost_s': [round(first_lost or 0, 1), round(last_lost or 0, 1)]})
    n_end = rec['status'][-1][1].reconnect_attempts
    check('no attempt after the link came back',
          all(m.reconnect_attempts == n_end for t, m in rec['status']
              if last_lost is not None and t > last_lost + 1.0), n_end)

    # ---- grids
    before = [m for t, m in rec['rssi'] if t < RESET_AT - 0.5]
    g = before[-1]
    data = list(g.data)
    nolink, unknown = data.count(-128), data.count(-1)
    levels = [v for v in data if 0 <= v <= 100]
    w, res = g.info.width, g.info.resolution
    ox, oy = g.info.origin.position.x, g.info.origin.position.y
    dead_cells = [(i % w, i // w) for i, v in enumerate(data)
                  if v == -128 and ox + (i % w + 0.5) * res > DEAD_X]
    check('RSSI grid before the reset: levels, no-link cells, -1 for the rest',
          nolink > 0 and levels and unknown > 0 and len(dead_cells) > 0 and
          all(-128 <= v <= 100 for v in data),
          {'size': [g.info.width, g.info.height], 'origin': [ox, oy],
           'cells_level': len(levels), 'cells_no_link': nolink,
           'cells_no_link_beyond_x8': len(dead_cells), 'cells_unknown': unknown,
           'level_min_max': [min(levels), max(levels)]})
    rtt_g = [m for t, m in rec['rtt'] if t < RESET_AT - 0.5][-1]
    rd = list(rtt_g.data)
    check('RTT grid: same geometry, no-link cells match the RSSI grid',
          (rtt_g.info.width, rtt_g.info.height) == (g.info.width, g.info.height) and
          [v == -128 for v in rd] == [v == -128 for v in data],
          {'cells_level': sum(1 for v in rd if 0 <= v <= 100),
           'cells_no_link': rd.count(-128), 'cells_unknown': rd.count(-1),
           'level_max': max([v for v in rd if v >= 0] or [None])})
    times = [t for t, _ in rec['rssi']]
    gaps = [b - a for a, b in zip(times, times[1:])]
    check('grids at most once per second',
          min(gaps) >= 0.9, {'messages': len(times), 'min_gap_s': round(min(gaps), 3)})
    cells = [c for t, c in rec['cells'] if t < RESET_AT - 0.5][-1]
    lost_cells = [c for c in cells['cells'] if c['no_link'] and c['x'] <= DEAD_X]
    check('cells JSON: every number, no-link samples on the LOST stretch',
          cells['cells'] and lost_cells and all(
              k in cells['cells'][0] for k in ('rssi_mean_dbm', 'rssi_min_dbm', 'rtt_p95_ms',
                                               'loss_pct', 'samples', 'last_t')),
          {'cells': len(cells['cells']), 'counts': cells['counts'],
           'cells_with_no_link_samples_x_le_8': len(lost_cells),
           'example': cells['cells'][0]})

    # ---- reset
    after = [(t, m) for t, m in rec['rssi'] if t >= RESET_AT]
    first_after = after[0][1] if after else None
    final = rec['rssi'][-1][1]
    check('reset empties the grid, then it grows again',
          reset_result and reset_result[1] and first_after is not None and
          first_after.info.width * first_after.info.height <= 1 and
          0 < sum(1 for v in final.data if v != -1) < sum(1 for v in data if v != -1),
          {'service': reset_result,
           'first_after': [first_after.info.width, first_after.info.height]
           if first_after else None,
           'cells_at_end': sum(1 for v in final.data if v != -1)})

    # ---- CPU
    cpu = {}
    marks = {label: (t, c) for label, t, c in cpu_marks}
    for name in marks['start'][1]:
        def pct(a, b):
            (ta, ca), (tb, cb) = marks[a], marks[b]
            return round(100.0 * (cb[name] - ca[name]) / (tb - ta), 2)
        cpu[name] = {'steady_ok_pct_of_one_core': pct('start', 'steady_end'),
                     'whole_run_pct_of_one_core': pct('start', 'end'),
                     'rss_mb': round(rss[name], 1) if rss[name] else None}
    return {'checks': checks, 'timeline': timeline, 'cpu': cpu, 'tf_hz': tf_hz,
            'echoed': base.echoed, 'python': sys.version.split()[0],
            'ros_distro': os.environ.get('ROS_DISTRO')}


if __name__ == '__main__':
    sys.exit(main())
