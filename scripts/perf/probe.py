#!/usr/bin/env python3
"""
One probe process for a stack run (perf baseline, level B).

    probe.py run --root-pid <ros2 launch pid> --duration 60 --out <dir> --label run1
                 [--harness-pid PID ...] [--skip 5] [--no-ros]
    probe.py summarise <dir> [--runs run1 run2]        # nodes.csv / topics.csv + a table

What `run` measures, in ONE process (no `ros2 topic hz` side processes):
  * every process below --root-pid, twice a second: CPU % of one core from the
    delta of psutil cpu_times (user + system, all threads) and RSS;
  * the topics of TOPICS: message count, the period between HEADER stamps
    (p50 / p95 / max) and between arrivals. Subscriptions are raw (no
    deserialisation) and the stamp is read straight from the CDR bytes, so the
    probe pays the executor's per-message cost and little else;
  * /tf, deserialised: rate and stamp period per edge and in aggregate;
  * /mpc/solver_status: solve_dt_sec (the message carries no iteration count);
  * the CPU of short-lived children once their parent reaps them, as
    "<node> (reaped children)" (added after the 20261111_0001 runs, whose CSVs
    do not have these rows: there that CPU is inside OTHER_PROCESSES);
  * its own CPU, the harness processes (--harness-pid: bag player, camera
    feeder) and everything else on the machine, so contention is on record.

A topic on which the middleware reports lost messages (the subscription's
message_lost event) is flagged `lost > 0`; summarise drops its period
statistics, because a period measured across a hole is not the publisher's.

The first --skip seconds are recorded but left out of every statistic.
"""

import argparse
import collections
import csv
import json
import os
import pathlib
import struct
import sys
import time

import psutil

SAMPLE_HZ = 2.0

#: topic -> type. Header stamps are used where the type starts with a std_msgs/Header.
#: Deliberately NOT here: the two 0.92 MB camera topics (copying them into Python at the
#: camera rate cost the probe half a core and made it lose messages on every other topic;
#: their consumers' output rates below say how many frames arrived) and the 40 Hz topics
#: nothing downstream of the MPC reads (/odom, /safety/hard_stop, /obstacle_clearance,
#: /perception/lidar_front_wall).
TOPICS = {
    '/scan': 'sensor_msgs/msg/LaserScan',
    '/odometry/filtered': 'nav_msgs/msg/Odometry',
    '/ekf_global/odometry/filtered': 'nav_msgs/msg/Odometry',
    '/camera/detections': 'vision_msgs/msg/Detection2DArray',
    '/camera/detections_3d': 'vision_msgs/msg/Detection3DArray',
    '/perception/obstacles_2d': 'f1tenth_messages/msg/Obstacle2DArray',
    '/perception/front_distance': 'std_msgs/msg/Float32',
    '/perception/stereo_columns': 'f1tenth_messages/msg/StereoColumns',
    '/perception/scene_labels': 'sensor_msgs/msg/Image',
    '/perception/lidar_front_wall_virtual': 'f1tenth_messages/msg/WallEstimate',
    '/perception/virtual_wall': 'std_msgs/msg/Float32',
    '/perception/walls': 'f1tenth_messages/msg/MultiWallTrackArray',
    '/perception/wall_observations': 'f1tenth_messages/msg/WallObservationArray',
    '/costmap/boundaries': 'f1tenth_messages/msg/BoundaryConstraintArray',
    '/costmap/front_clearance': 'std_msgs/msg/Float32',
    '/costmap/semantic_objects': 'f1tenth_messages/msg/SemanticObjectArray',
    '/costmap/semantic_obstacles': 'f1tenth_messages/msg/Obstacle2DArray',
    '/perception/doors_semantic': 'f1tenth_messages/msg/DoorArray',
    '/behavior/tree_status': 'f1tenth_messages/msg/BehaviorTreeStatus',
    '/drive': 'ackermann_msgs/msg/AckermannDriveStamped',
    '/mpc/speed_channel': 'f1tenth_messages/msg/SpeedChannelStatus',
    '/diagnostics/system_status': 'f1tenth_messages/msg/SystemStatus',
}
DESERIALISED = {
    '/tf': 'tf2_msgs/msg/TFMessage',
    '/mpc/solver_status': 'f1tenth_messages/msg/MpcSolverStatus',
}


def node_name(proc):
    """A stable, readable name for one process of the stack."""
    try:
        cmd = proc.cmdline()
    except (psutil.NoSuchProcess, psutil.AccessDenied):
        return None
    if not cmd:
        return None
    for arg in cmd:
        if arg.startswith('__node:='):
            return arg.split(':=', 1)[1]
    base = [os.path.basename(a) for a in cmd]
    if 'ros2' in base and 'launch' in cmd:
        files = [a for a in cmd if a.endswith('.launch.py')]
        return 'launch:' + (os.path.basename(files[0]) if files else '?')
    if base[0].startswith('python') and len(cmd) > 1:
        if cmd[1] == '-m' and len(cmd) > 2:
            return 'py:' + cmd[2]
        return base[1] if not base[1].startswith('-') else base[0]
    if base[0] in ('taskset', 'env', 'nice') or base[0].startswith('taskset'):
        rest = [b for a, b in zip(cmd[1:], base[1:]) if not a.startswith('-') and '=' not in a
                and not a[0].isdigit()]
        return rest[0] if rest else base[0]
    return base[0]


class ProcSampler:
    def __init__(self, root_pid, harness_pids):
        self.root = psutil.Process(root_pid) if root_pid else None
        self.harness = {}
        for pid in harness_pids:
            try:
                self.harness[pid] = psutil.Process(pid)
            except psutil.NoSuchProcess:
                pass
        self.me = psutil.Process()
        self.prev = {}
        self.prev_t = None
        self.prev_sys = None
        self.rows = []
        self.names = {}

    def _procs(self):
        out = {}
        if self.root is not None:
            try:
                for p in [self.root] + self.root.children(recursive=True):
                    out[p.pid] = ('stack', p)
            except psutil.NoSuchProcess:
                pass
        for pid, p in self.harness.items():
            try:
                for q in [p] + p.children(recursive=True):
                    out[q.pid] = ('harness', q)
            except psutil.NoSuchProcess:
                pass
        out[self.me.pid] = ('probe', self.me)
        return out

    def sample(self, t):
        now = time.monotonic()
        sys_t = psutil.cpu_times()
        busy = sum(sys_t) - sys_t.idle - getattr(sys_t, 'iowait', 0.0)
        cur = {}
        for pid, (kind, p) in self._procs().items():
            try:
                ct = p.cpu_times()
                rss = p.memory_info().rss
                threads = p.num_threads()
            except (psutil.NoSuchProcess, psutil.AccessDenied):
                continue
            if pid not in self.names:
                self.names[pid] = ('probe' if kind == 'probe' else node_name(p)) or f'pid{pid}'
            cur[pid] = (ct.user + ct.system, rss, threads, kind,
                        ct.children_user + ct.children_system)
        if self.prev_t is not None:
            dt = now - self.prev_t
            total = 0.0
            for pid, (cpu, rss, threads, kind, kids) in cur.items():
                if pid not in self.prev:
                    continue
                pct = 100.0 * (cpu - self.prev[pid][0]) / dt
                self.rows.append((round(t, 3), pid, kind, self.names[pid], round(pct, 2),
                                  round(rss / 1e6, 1), threads))
                total += pct
                # CPU of children that lived and died between two samples (the behaviour
                # tree's `dot`, every 0.4 s) only ever shows up here, once reaped. Launch
                # processes are left out: their children are the nodes, counted above.
                kid_pct = 100.0 * (kids - self.prev[pid][4]) / dt
                if kid_pct > 0.0 and not self.names[pid].startswith('launch:') \
                        and self.names[pid] not in ('component_supervisor_node', 'taskset'):
                    self.rows.append((round(t, 3), pid, kind,
                                      self.names[pid] + ' (reaped children)',
                                      round(kid_pct, 2), 0.0, 0))
                    total += kid_pct
            sys_pct = 100.0 * (busy - self.prev_sys) / dt
            self.rows.append((round(t, 3), 0, 'system', 'ALL_CPUS_BUSY', round(sys_pct, 2), 0, 0))
            self.rows.append((round(t, 3), 0, 'system', 'OTHER_PROCESSES',
                              round(sys_pct - total, 2), 0, 0))
        self.prev, self.prev_t, self.prev_sys = cur, now, busy


def _stamp(raw):
    """(sec + nanosec) of the std_msgs/Header at the start of a CDR message."""
    sec, nsec = struct.unpack_from('<iI', raw, 4)
    return sec + nsec * 1e-9


def run(args):
    out = pathlib.Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    sampler = ProcSampler(args.root_pid, args.harness_pid)
    t_start = time.monotonic()
    topics = {}
    tf_edges = collections.defaultdict(list)
    solve = []
    node = None
    if not args.no_ros:
        import rclpy
        from rclpy.event_handler import SubscriptionEventCallbacks
        from rclpy.qos import QoSProfile, QoSReliabilityPolicy
        from rosidl_runtime_py.utilities import get_message
        rclpy.init()
        node = rclpy.create_node('perf_probe', start_parameter_services=False)
        qos = QoSProfile(depth=500, reliability=QoSReliabilityPolicy.BEST_EFFORT)

        def lost_cb(name):
            def cb(info):
                topics[name]['lost'] = int(info.total_count)
            return cb

        def raw_cb(name, header):
            rec = topics[name]

            def cb(raw):
                rec['rx'].append(time.monotonic() - t_start)
                if header:
                    rec['stamp'].append(_stamp(raw))
            return cb

        for name, type_name in TOPICS.items():
            try:
                cls = get_message(type_name)
            except Exception as exc:
                print(f'probe: skipping {name}: {exc!r}', file=sys.stderr)
                continue
            fields = list(cls.get_fields_and_field_types().items())
            header = bool(fields) and fields[0] == ('header', 'std_msgs/Header')
            topics[name] = {'type': type_name, 'rx': [], 'stamp': [], 'lost': 0, 'header': header}
            node.create_subscription(
                cls, name, raw_cb(name, header), qos, raw=True,
                event_callbacks=SubscriptionEventCallbacks(message_lost=lost_cb(name)))

        def tf_cb(msg):
            now = time.monotonic() - t_start
            topics['/tf']['rx'].append(now)
            for tr in msg.transforms:
                tf_edges[f'{tr.header.frame_id}->{tr.child_frame_id}'].append(
                    (now, tr.header.stamp.sec + tr.header.stamp.nanosec * 1e-9))

        def solve_cb(msg):
            now = time.monotonic() - t_start
            rec = topics['/mpc/solver_status']
            rec['rx'].append(now)
            rec['stamp'].append(msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9)
            solve.append((now, float(msg.solve_dt_sec), bool(msg.success), str(msg.solver),
                          int(msg.n_boundary_constraints), int(msg.n_obstacles)))

        for name, cb in (('/tf', tf_cb), ('/mpc/solver_status', solve_cb)):
            topics[name] = {'type': DESERIALISED[name], 'rx': [], 'stamp': [], 'lost': 0,
                            'header': name != '/tf'}
            node.create_subscription(
                get_message(DESERIALISED[name]), name, cb, qos,
                event_callbacks=SubscriptionEventCallbacks(message_lost=lost_cb(name)))

    period = 1.0 / SAMPLE_HZ
    next_sample = time.monotonic()
    end = t_start + args.duration
    while time.monotonic() < end:
        now = time.monotonic()
        if now >= next_sample:
            sampler.sample(now - t_start)
            next_sample += period
        if node is not None:
            rclpy.spin_once(node, timeout_sec=max(0.0, min(0.05, next_sample - time.monotonic())))
        else:
            time.sleep(max(0.0, next_sample - time.monotonic()))

    with open(out / f'proc_{args.label}.csv', 'w', newline='') as fh:
        w = csv.writer(fh)
        w.writerow(['t', 'pid', 'kind', 'name', 'cpu_pct', 'rss_mb', 'threads'])
        w.writerows(sampler.rows)
    raw = {'label': args.label, 'duration': args.duration, 'skip': args.skip,
           'topics': {k: {'type': v['type'], 'header': v['header'], 'lost': v['lost'],
                          'rx': v['rx'], 'stamp': v['stamp']} for k, v in topics.items()},
           'tf_edges': {k: v for k, v in tf_edges.items()}, 'solve': solve,
           'loadavg': os.getloadavg(), 'cpu_count': psutil.cpu_count(),
           'mem_available_mb': psutil.virtual_memory().available / 1e6,
           'swap_used_mb': psutil.swap_memory().used / 1e6}
    (out / f'probe_{args.label}.json').write_text(json.dumps(raw))
    if node is not None:
        node.destroy_node()
        rclpy.shutdown()
    return 0


# ----------------------------------------------------------------- summarise

def _pct(values, q):
    import numpy as np
    return float(np.percentile(np.asarray(values, dtype=float), q)) if len(values) else float('nan')


def _periods(times):
    import numpy as np
    t = np.asarray(times, dtype=float)
    if t.size < 3:
        return None
    d = np.diff(t) * 1e3
    return d[d > 0] if np.any(d > 0) else None


def _one_run(directory, label):
    import numpy as np
    raw = json.loads((directory / f'probe_{label}.json').read_text())
    skip, dur = raw['skip'], raw['duration']
    window = dur - skip
    nodes = collections.defaultdict(lambda: collections.defaultdict(list))
    with open(directory / f'proc_{label}.csv') as fh:
        for row in csv.DictReader(fh):
            if float(row['t']) < skip:
                continue
            key = (row['kind'], row['name'])
            nodes[key][float(row['t'])].append((float(row['cpu_pct']), float(row['rss_mb'])))
    node_stats = {}
    for key, by_t in nodes.items():
        cpu = [sum(c for c, _r in v) for v in by_t.values()]      # same-named processes add up
        rss = [sum(r for _c, r in v) for v in by_t.values()]
        n_proc = max(len(v) for v in by_t.values())
        node_stats[key] = {'cpu_mean': float(np.mean(cpu)), 'cpu_p95': _pct(cpu, 95),
                           'cpu_max': float(np.max(cpu)), 'rss_mean': float(np.mean(rss)),
                           'rss_max': float(np.max(rss)), 'n_proc': n_proc,
                           'samples': len(cpu)}
    topic_stats = {}
    for name, rec in raw['topics'].items():
        rx = [t for t in rec['rx'] if t >= skip]
        n_skip = len(rec['rx']) - len(rx)
        st = {'type': rec['type'], 'n': len(rx), 'rate_hz': len(rx) / window if window > 0 else 0.0,
              'lost': rec['lost']}
        per_rx = _periods(rx)
        per_st = _periods(rec['stamp'][n_skip:]) if rec['header'] else None
        for tag, per in (('rx', per_rx), ('stamp', per_st)):
            for q in (50, 95):
                st[f'{tag}_p{q}_ms'] = _pct(per, q) if per is not None else float('nan')
            st[f'{tag}_max_ms'] = float(np.max(per)) if per is not None else float('nan')
        topic_stats[name] = st
    for edge, pairs in raw['tf_edges'].items():
        pairs = [p for p in pairs if p[0] >= skip]
        per = _periods([p[1] for p in pairs])
        topic_stats[f'/tf[{edge}]'] = {
            'type': 'tf edge', 'n': len(pairs), 'rate_hz': len(pairs) / window, 'lost': 0,
            'rx_p50_ms': float('nan'), 'rx_p95_ms': float('nan'), 'rx_max_ms': float('nan'),
            'stamp_p50_ms': _pct(per, 50) if per is not None else float('nan'),
            'stamp_p95_ms': _pct(per, 95) if per is not None else float('nan'),
            'stamp_max_ms': float(np.max(per)) if per is not None else float('nan')}
    solve = [s for s in raw['solve'] if s[0] >= skip]
    ms = [s[1] * 1e3 for s in solve]
    solve_stats = {'n': len(solve), 'p50_ms': _pct(ms, 50), 'p95_ms': _pct(ms, 95),
                   'max_ms': float(np.max(ms)) if ms else float('nan'),
                   'success_frac': float(np.mean([s[2] for s in solve])) if solve else float('nan'),
                   'solver': solve[0][3] if solve else '',
                   'n_boundary': _pct([s[4] for s in solve], 50),
                   'n_obstacles': _pct([s[5] for s in solve], 50)}
    return {'nodes': node_stats, 'topics': topic_stats, 'solve': solve_stats,
            'host': {k: raw[k] for k in ('loadavg', 'mem_available_mb', 'swap_used_mb',
                                         'cpu_count')}}


def summarise(args):
    directory = pathlib.Path(args.dir)
    labels = args.runs or sorted(p.stem[len('probe_'):] for p in directory.glob('probe_run*.json'))
    runs = {lab: _one_run(directory, lab) for lab in labels}
    keys = sorted({k for r in runs.values() for k in r['nodes']},
                  key=lambda k: -max(r['nodes'].get(k, {}).get('cpu_mean', 0.0)
                                     for r in runs.values()))
    with open(directory / 'nodes.csv', 'w', newline='') as fh:
        w = csv.writer(fh)
        head = ['kind', 'node', 'n_proc']
        for lab in labels:
            head += [f'{lab}_cpu_mean', f'{lab}_cpu_p95', f'{lab}_rss_mean_mb', f'{lab}_rss_max_mb']
        head += ['cpu_mean_all', 'run_diff_pct', 'small_gains_measurable']
        w.writerow(head)
        for key in keys:
            row = [key[0], key[1], max(r['nodes'].get(key, {}).get('n_proc', 0)
                                       for r in runs.values())]
            means = []
            for lab in labels:
                st = runs[lab]['nodes'].get(key)
                if st is None:
                    row += ['', '', '', '']
                    continue
                means.append(st['cpu_mean'])
                row += [round(st['cpu_mean'], 2), round(st['cpu_p95'], 2),
                        round(st['rss_mean'], 1), round(st['rss_max'], 1)]
            avg = sum(means) / len(means) if means else 0.0
            diff = (100.0 * (max(means) - min(means)) / avg) if len(means) > 1 and avg > 0 else ''
            flag = '' if diff == '' else ('no (>5% between runs)' if diff > 5.0 else 'yes')
            row += [round(avg, 2), '' if diff == '' else round(diff, 1), flag]
            w.writerow(row)
    names = sorted({k for r in runs.values() for k in r['topics']})
    with open(directory / 'topics.csv', 'w', newline='') as fh:
        w = csv.writer(fh)
        w.writerow(['topic', 'type', 'run', 'n', 'rate_hz', 'stamp_p50_ms', 'stamp_p95_ms',
                    'stamp_max_ms', 'rx_p50_ms', 'rx_p95_ms', 'lost', 'period_valid'])
        for name in names:
            for lab in labels:
                st = runs[lab]['topics'].get(name)
                if st is None:
                    continue
                valid = st['lost'] == 0
                f = (lambda v: round(v, 2) if valid and v == v else '')  # noqa: E731
                w.writerow([name, st['type'], lab, st['n'], round(st['rate_hz'], 2),
                            f(st['stamp_p50_ms']), f(st['stamp_p95_ms']), f(st['stamp_max_ms']),
                            f(st['rx_p50_ms']), f(st['rx_p95_ms']), st['lost'],
                            'yes' if valid else 'no (probe lost messages)'])
    (directory / 'summary.json').write_text(json.dumps(
        {lab: {'solve': r['solve'], 'host': r['host']} for lab, r in runs.items()}, indent=1))
    print(f'wrote {directory}/nodes.csv, topics.csv, summary.json for runs {labels}')
    for lab, r in runs.items():
        print(lab, 'solve', r['solve'], 'host', r['host'])
    return 0


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    sub = ap.add_subparsers(dest='cmd', required=True)
    r = sub.add_parser('run')
    r.add_argument('--root-pid', type=int, default=0)
    r.add_argument('--harness-pid', type=int, action='append', default=[])
    r.add_argument('--duration', type=float, default=60.0)
    r.add_argument('--skip', type=float, default=5.0)
    r.add_argument('--out', required=True)
    r.add_argument('--label', default='run1')
    r.add_argument('--no-ros', action='store_true')
    s = sub.add_parser('summarise')
    s.add_argument('dir')
    s.add_argument('--runs', nargs='*')
    args = ap.parse_args(argv)
    return run(args) if args.cmd == 'run' else summarise(args)


if __name__ == '__main__':
    sys.exit(main())
