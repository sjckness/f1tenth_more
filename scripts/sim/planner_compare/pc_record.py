#!/usr/bin/env python3
"""Recorder for one planner-controller comparison run (scripts/sim/planner_compare).

Two subcommands, both run in the stack's ROS environment (sim_isaac.sh run):

  pc_record.py ready  --node NAME [--timeout S]
      Wait until the stack can take the command: the controller node NAME is in
      the graph and none of the OTHER controllers is (a leftover from an earlier
      mode would drive the car too), /mission/status, /odometry/filtered and
      /sim/ground_truth/odom have arrived. Exit 0 when ready, 1 on timeout.

  pc_record.py record --node NAME --out DIR [--timeout S] [--tail S] [--sim-time BOOL]
      Record until the mission has gone RUNNING and then left it (COMPLETE /
      ABORTED), plus --tail seconds so the stop is in the data, or --timeout
      (wall) seconds. Writes into DIR:

        gt.csv, odom.csv          t_sim,t_wall,x,y,yaw,v
                                  (/sim/ground_truth/odom, /odometry/filtered)
        drive.csv                 t_sim,t_wall,speed,steer (/drive: the controller's own output)
        mission.csv               t_sim,t_wall,state,json_path (/mission/status, on change)
        paths.jsonl               {"src","t_sim","frame","xy"}: /planner/path (new modes),
                                  the corridor polygon (mpc_corr: /corridor; corridor_rrt:
                                  /planner/markers), the RRT* tree edges (/planner/markers,
                                  src rrt_tree, xy = segment endpoint pairs) and mpc_corr's
                                  predicted horizon (/mpc/solver_status pred_x/pred_y)
        planner_status.jsonl      /planner/status as published (plan_ms per replan)
        solver.csv                t_sim,t_wall,success,solve_dt_sec,cost,solver (mpc_corr)
        cpu.csv                   t_wall,t_sim,cpu_s,rss_mb,sys_busy_pct: the controller
                                  PROCESS read from /proc at 5 Hz (wall), all threads
        meta.json                 node, pid, cmdline, host, counts, mission trail, exit reason

The controller's CPU comes from /proc/<pid>/stat (utime + stime) of the one
process whose command line carries ``__node:=NAME``: the same measurement for
the MPC and for the two new nodes, independent of what each node reports about
itself (mpc_corr's solve_dt_sec is on the ROS clock, 0 in sim time).
"""

import argparse
import csv
import json
import math
import os
import socket
import sys
import threading
import time

from ackermann_msgs.msg import AckermannDriveStamped
from f1tenth_messages.msg import MissionStatus, MpcSolverStatus
from nav_msgs.msg import Odometry, Path
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy,
                       qos_profile_sensor_data)
from std_msgs.msg import String
from visualization_msgs.msg import MarkerArray

CONTROLLERS = ('mpc_corr', 'corridor_rrt', 'p2p_geometric')
ACTIVE_STATES = ('LOADED', 'RUNNING', 'HOLDING')
BEST_EFFORT = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST, depth=50)
LATCHED = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL,
                     history=HistoryPolicy.KEEP_LAST, depth=1)
CLK_TCK = os.sysconf('SC_CLK_TCK')


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def find_pids(node_name):
    """Pids whose command line has ``__node:=node_name`` (the launch_ros remap), not ours."""
    pids = []
    me = os.getpid()
    for d in os.listdir('/proc'):
        if not d.isdigit() or int(d) == me:
            continue
        try:
            with open(f'/proc/{d}/cmdline', 'rb') as f:
                args = f.read().split(b'\0')
        except OSError:
            continue
        if f'__node:={node_name}'.encode() in args:
            pids.append(int(d))
    return pids


def proc_cpu_s(pid):
    with open(f'/proc/{pid}/stat') as f:
        fields = f.read().rsplit(')', 1)[1].split()
    # fields[0] is state (field 3); utime, stime are fields 14, 15
    return (int(fields[11]) + int(fields[12])) / CLK_TCK


def proc_rss_mb(pid):
    with open(f'/proc/{pid}/statm') as f:
        return int(f.read().split()[1]) * os.sysconf('SC_PAGE_SIZE') / 2**20


def sys_cpu():
    with open('/proc/stat') as f:
        vals = [int(v) for v in f.readline().split()[1:]]
    idle = vals[3] + vals[4]
    return sum(vals), idle


class Recorder(Node):

    def __init__(self, node_name, out, sim_time):
        super().__init__('planner_compare_recorder',
                         parameter_overrides=[Parameter('use_sim_time', value=sim_time)])
        self.node_name = node_name
        self.out = out
        self.lock = threading.Lock()
        self.files = {}
        self.writers = {}
        self.counts = {}
        self.mission_trail = []
        self.state = None
        self.seen_running = False
        self.done_wall = None
        self.last = {}
        if out:
            os.makedirs(out, exist_ok=True)
            self._csv('gt', ['t_sim', 't_wall', 'x', 'y', 'yaw', 'v'])
            self._csv('odom', ['t_sim', 't_wall', 'x', 'y', 'yaw', 'v'])
            self._csv('drive', ['t_sim', 't_wall', 'speed', 'steer'])
            self._csv('mission', ['t_sim', 't_wall', 'state', 'json_path'])
            self._csv('solver', ['t_sim', 't_wall', 'success', 'solve_dt_sec', 'cost', 'solver'])
            self._csv('cpu', ['t_wall', 't_sim', 'cpu_s', 'rss_mb', 'sys_busy_pct'])
            self.paths = open(os.path.join(out, 'paths.jsonl'), 'w')
            self.pstatus = open(os.path.join(out, 'planner_status.jsonl'), 'w')
        self.create_subscription(Odometry, '/sim/ground_truth/odom',
                                 lambda m: self._odom('gt', m), qos_profile_sensor_data)
        self.create_subscription(Odometry, '/odometry/filtered',
                                 lambda m: self._odom('odom', m), qos_profile_sensor_data)
        self.create_subscription(MissionStatus, '/mission/status', self._mission, LATCHED)
        if out:
            self.create_subscription(AckermannDriveStamped, '/drive', self._drive, BEST_EFFORT)
            self.create_subscription(Path, '/planner/path', self._path, BEST_EFFORT)
            self.create_subscription(String, '/planner/status', self._pstatus, BEST_EFFORT)
            self.create_subscription(String, '/corridor', self._corridor, BEST_EFFORT)
            self.create_subscription(MpcSolverStatus, '/mpc/solver_status', self._solver,
                                     BEST_EFFORT)
            self.create_subscription(MarkerArray, '/planner/markers', self._markers, BEST_EFFORT)

    # ---- output ----------------------------------------------------------------------
    def _csv(self, name, header):
        f = open(os.path.join(self.out, f'{name}.csv'), 'w', newline='')
        w = csv.writer(f)
        w.writerow(header)
        self.files[name], self.writers[name] = f, w

    def _row(self, name, row):
        self.counts[name] = self.counts.get(name, 0) + 1
        self.last[name] = time.time()
        if self.out:
            with self.lock:
                self.writers[name].writerow(row)

    def t(self):
        return self.get_clock().now().nanoseconds * 1e-9, time.time()

    def close(self):
        if not self.out:
            return
        with self.lock:
            for f in self.files.values():
                f.close()
            self.paths.close()
            self.pstatus.close()

    # ---- callbacks -------------------------------------------------------------------
    def _odom(self, name, m):
        ts, tw = self.t()
        p = m.pose.pose
        v = math.hypot(m.twist.twist.linear.x, m.twist.twist.linear.y)
        self._row(name, [f'{ts:.4f}', f'{tw:.4f}', f'{p.position.x:.4f}', f'{p.position.y:.4f}',
                         f'{yaw_of(p.orientation):.5f}', f'{v:.4f}'])
        if name == 'gt':
            self.last['gt_frame'] = m.header.frame_id

    def _drive(self, m):
        ts, tw = self.t()
        self._row('drive', [f'{ts:.4f}', f'{tw:.4f}', f'{m.drive.speed:.4f}',
                            f'{m.drive.steering_angle:.5f}'])

    def _mission(self, m):
        ts, tw = self.t()
        self.counts['mission'] = self.counts.get('mission', 0) + 1
        self.last['mission'] = tw
        if m.state != self.state:
            self.mission_trail.append({'t_sim': ts, 't_wall': tw, 'state': m.state,
                                       'json_path': m.json_path})
            if self.out:
                with self.lock:
                    self.writers['mission'].writerow(
                        [f'{ts:.4f}', f'{tw:.4f}', m.state, m.json_path])
            self.get_logger().info(f'mission state {self.state} -> {m.state}')
        self.state = m.state
        if m.state == 'RUNNING':
            self.seen_running = True
        elif self.seen_running and m.state not in ACTIVE_STATES and self.done_wall is None:
            self.done_wall = tw

    def _write_path(self, src, frame, xy):
        ts, tw = self.t()
        self.counts[src] = self.counts.get(src, 0) + 1
        with self.lock:
            self.paths.write(json.dumps({'src': src, 't_sim': round(ts, 4), 't_wall': round(tw, 4),
                                         'frame': frame, 'xy': xy}) + '\n')

    def _path(self, m):
        self._write_path('planner_path', m.header.frame_id,
                         [[round(p.pose.position.x, 4), round(p.pose.position.y, 4)]
                          for p in m.poses])

    def _corridor(self, m):
        try:
            payload = json.loads(m.data)
        except ValueError:
            return
        self._write_path('corridor', payload.get('frame_id', 'odom'), payload.get('polygon', []))

    def _markers(self, m):
        walls = {}
        for mk in m.markers:
            pts = [[round(p.x, 4), round(p.y, 4)] for p in mk.points]
            if mk.ns in ('corridor_left', 'corridor_right'):
                walls[mk.ns] = (mk.header.frame_id, pts)
            elif mk.ns == 'rrt_tree' and pts:
                self._write_path('rrt_tree', mk.header.frame_id, pts)
        if len(walls) == 2:
            frame = walls['corridor_left'][0]
            self._write_path('corridor', frame,
                             walls['corridor_left'][1] + walls['corridor_right'][1][::-1])

    def _solver(self, m):
        ts, tw = self.t()
        self._row('solver', [f'{ts:.4f}', f'{tw:.4f}', int(m.success), f'{m.solve_dt_sec:.6f}',
                             f'{m.cost:.4f}', m.solver])
        if len(m.pred_x):
            self._write_path('mpc_pred', m.prediction_frame_id or 'odom',
                             [[round(float(x), 4), round(float(y), 4)]
                              for x, y in zip(m.pred_x, m.pred_y)])

    def _pstatus(self, m):
        ts, tw = self.t()
        self.counts['planner_status'] = self.counts.get('planner_status', 0) + 1
        try:
            d = json.loads(m.data)
        except ValueError:
            d = {'raw': m.data}
        d.update({'t_sim': round(ts, 4), 't_wall': round(tw, 4)})
        with self.lock:
            self.pstatus.write(json.dumps(d) + '\n')


def cpu_sampler(rec, pid, stop, period=0.2):
    tot0, idle0 = sys_cpu()
    while not stop.is_set():
        try:
            cpu = proc_cpu_s(pid)
            rss = proc_rss_mb(pid)
        except OSError:
            rec.counts['cpu_lost'] = rec.counts.get('cpu_lost', 0) + 1
            stop.wait(period)
            continue
        tot, idle = sys_cpu()
        busy = 100.0 * (1.0 - (idle - idle0) / max(tot - tot0, 1))
        tot0, idle0 = tot, idle
        ts, tw = rec.t()
        rec._row('cpu', [f'{tw:.4f}', f'{ts:.4f}', f'{cpu:.3f}', f'{rss:.1f}', f'{busy:.1f}'])
        stop.wait(period)


def graph_nodes(node):
    return {n for n, _ in node.get_node_names_and_namespaces()}


def cmd_ready(args):
    rclpy.init()
    rec = Recorder(args.node, None, sim_time=False)
    t0 = time.time()
    others = [c for c in CONTROLLERS if c != args.node]
    ok = False
    last_report = 0.0
    while time.time() - t0 < args.timeout:
        rclpy.spin_once(rec, timeout_sec=0.2)
        names = graph_nodes(rec)
        missing = []
        if args.node not in names:
            missing.append(f'node {args.node}')
        if rec.counts.get('mission', 0) == 0:
            missing.append('/mission/status')
        for name, topic in (('odom', '/odometry/filtered'), ('gt', '/sim/ground_truth/odom')):
            if time.time() - rec.last.get(name, 0.0) > 2.0:
                missing.append(topic)
        stray = [c for c in others if c in names]
        if stray:
            print(f'[ready] ERROR: other controller(s) in the graph: {stray} (leftover run?)',
                  flush=True)
            break
        if not missing:
            ok = True
            break
        if time.time() - last_report > 10.0:
            print(f'[ready] {time.time() - t0:.0f}s, waiting for: {", ".join(missing)}', flush=True)
            last_report = time.time()
    print(f'[ready] {"READY" if ok else "NOT READY"} after {time.time() - t0:.1f}s '
          f'(mission state {rec.state})', flush=True)
    rec.destroy_node()
    rclpy.shutdown()
    return 0 if ok else 1


def cmd_record(args):
    pids = find_pids(args.node)
    if len(pids) != 1:
        print(f'[record] ERROR: expected one process with __node:={args.node}, found {pids}',
              flush=True)
        return 2
    pid = pids[0]
    with open(f'/proc/{pid}/cmdline', 'rb') as f:
        cmdline = [a.decode(errors='replace') for a in f.read().split(b'\0') if a]
    try:
        affinity = sorted(os.sched_getaffinity(pid))
    except OSError:
        affinity = None
    rclpy.init()
    rec = Recorder(args.node, args.out, sim_time=args.sim_time)
    stop = threading.Event()
    sampler = threading.Thread(target=cpu_sampler, args=(rec, pid, stop), daemon=True)
    sampler.start()
    t0 = time.time()
    with open(os.path.join(args.out, 'RECORDING'), 'w') as f:
        f.write(str(t0))
    print(f'[record] {args.node} pid {pid}, recording into {args.out}', flush=True)
    reason = 'timeout'
    try:
        while time.time() - t0 < args.timeout:
            rclpy.spin_once(rec, timeout_sec=0.05)
            if rec.done_wall is not None and time.time() - rec.done_wall >= args.tail:
                reason = f'mission {rec.state}'
                break
    except KeyboardInterrupt:
        reason = 'interrupted'
    stop.set()
    sampler.join(timeout=2.0)
    meta = {
        'node': args.node, 'pid': pid, 'cmdline': cmdline, 'cpu_affinity': affinity,
        'host': socket.gethostname(), 'nproc': os.cpu_count(),
        'cpu_model': next((ln.split(':', 1)[1].strip() for ln in open('/proc/cpuinfo')
                           if ln.startswith('model name')), None),
        'clk_tck': CLK_TCK, 'sim_time': args.sim_time, 'gt_frame': rec.last.get('gt_frame'),
        'record_wall_start': t0, 'record_wall_end': time.time(), 'exit_reason': reason,
        'final_state': rec.state, 'mission_trail': rec.mission_trail, 'counts': rec.counts,
    }
    rec.close()
    with open(os.path.join(args.out, 'meta.json'), 'w') as f:
        json.dump(meta, f, indent=2)
    print(f'[record] done: {reason}; counts {rec.counts}', flush=True)
    rec.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    return 0 if reason.startswith('mission') else 3


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    sub = ap.add_subparsers(dest='cmd', required=True)
    r = sub.add_parser('ready')
    r.add_argument('--node', required=True, choices=CONTROLLERS)
    r.add_argument('--timeout', type=float, default=240.0)
    c = sub.add_parser('record')
    c.add_argument('--node', required=True, choices=CONTROLLERS)
    c.add_argument('--out', required=True)
    c.add_argument('--timeout', type=float, default=150.0)
    c.add_argument('--tail', type=float, default=4.0)
    c.add_argument('--sim-time', type=lambda v: v.lower() in ('1', 'true'), default=True)
    args = ap.parse_args()
    return cmd_ready(args) if args.cmd == 'ready' else cmd_record(args)


if __name__ == '__main__':
    sys.exit(main())
