#!/usr/bin/env python3
"""The two measurements cpu_test.sh needs that no shell tool on this box gives.

    cpu_test_probe.py procstat --out DIR --duration 60 [--interval 1]
    cpu_test_probe.py rates    --out DIR --duration 60 [--topics a,b,c]

procstat: per-THREAD CPU and per-PROCESS disk I/O, once a second, for every
    process of the stack. This is what `pidstat -u -t -d 1` would give, and
    pidstat is NOT installed here (no sysstat package) and needs a package
    install to be. /proc has all of it without root: utime and stime per
    thread from /proc/<pid>/task/<tid>/stat, read_bytes and write_bytes per
    process from /proc/<pid>/io, and the CPU a thread last ran on and its
    affinity mask from the same files. cpu_test.sh uses pidstat when it IS
    present and this otherwise, and says which in the summary.

rates: how often each topic actually publishes, MEASURED OUTSIDE ANY BAG, with
    one subscription per topic and no deserialisation of the payload beyond
    what rclpy does to hand it over. The point of measuring here rather than
    counting rows in a recording is that a recorder loses messages under load
    -- the nightly audit found 24 % of /scan missing from a bag the LiDAR had
    published at 40 Hz -- so a bag's own rate is a measurement of the
    recorder, not of the stack.
"""

import argparse
import json
import os
import re
import sys
import time
from pathlib import Path

CLK_TCK = os.sysconf('SC_CLK_TCK')
#: Process names that are the stack. Everything else on the box is ignored, so
#: a browser or an editor cannot appear in a phase table.
STACK_PATTERNS = (
    'component_supervisor', 'urg_node', 'zed_wrapper', 'zed_camera',
    'ekf_node', 'slam_toolbox', 'controller_server', 'planner_server',
    'behavior_executor', 'mission_logger', 'mpc_corr', 'MPC_corr',
    'lidar_front_wall', 'front_clearance', 'swept_clearance',
    'obstacle_clearance', 'wall_distance', 'detection_3d', 'yolo_detector',
    'obstacle_projector', 'semantic_layer', 'costmap_', 'safe_corridor',
    'ackermann_mux', 'vesc', 'foxglove', 'system_observer', 'diagnostics',
    'reset_manager', 'robot_state_publisher', 'static_transform',
    'rosbag2', 'ros2', 'python3', 'bt_navigator', 'map_server',
    'lifecycle_manager', 'viz_jpeg', 'throttle', 'llama',
)


def stack_pids():
    """{pid: (name, cmdline)} of the processes that look like the stack."""
    out = {}
    for entry in os.listdir('/proc'):
        if not entry.isdigit():
            continue
        try:
            with open(f'/proc/{entry}/cmdline', 'rb') as fh:
                cmd = fh.read().replace(b'\0', b' ').decode('utf-8', 'replace').strip()
            if not cmd:
                continue
            with open(f'/proc/{entry}/comm') as fh:
                comm = fh.read().strip()
        except OSError:
            continue
        if not any(pat in cmd or pat in comm for pat in STACK_PATTERNS):
            continue
        # the ROS node name if there is one, else the executable
        name = comm
        m = re.search(r'--ros-args.*?-r\s+__node:=(\S+)', cmd)
        if m:
            name = m.group(1)
        else:
            for token in cmd.split():
                base = os.path.basename(token)
                if base and not base.startswith('-') and base not in ('python3', 'ros2'):
                    name = base
                    break
        out[int(entry)] = (name, cmd[:300])
    return out


def thread_times(pid):
    """{tid: (name, utime+stime ticks, last cpu, affinity)} for one process."""
    out = {}
    base = f'/proc/{pid}/task'
    try:
        tids = os.listdir(base)
    except OSError:
        return out
    for tid in tids:
        try:
            with open(f'{base}/{tid}/stat') as fh:
                fields = fh.read()
        except OSError:
            continue
        # comm can hold spaces and parentheses: split on the LAST ')'
        close = fields.rfind(')')
        head, rest = fields[:close], fields[close + 2:].split()
        name = head[head.find('(') + 1:]
        try:
            ticks = int(rest[11]) + int(rest[12])     # utime, stime
            cpu = int(rest[36])                       # processor
        except (IndexError, ValueError):
            continue
        try:
            affinity = sorted(os.sched_getaffinity(int(tid)))
        except OSError:
            affinity = []
        out[int(tid)] = (name, ticks, cpu, affinity)
    return out


def process_io(pid):
    try:
        with open(f'/proc/{pid}/io') as fh:
            vals = dict(line.split(': ') for line in fh.read().splitlines())
        return int(vals.get('read_bytes', 0)), int(vals.get('write_bytes', 0))
    except (OSError, ValueError):
        return 0, 0


def run_procstat(cfg):
    out = Path(cfg.out)
    out.mkdir(parents=True, exist_ok=True)
    samples = []
    pids = stack_pids()
    prev = {pid: thread_times(pid) for pid in pids}
    prev_io = {pid: process_io(pid) for pid in pids}
    t_prev = time.monotonic()
    deadline = t_prev + cfg.duration
    while time.monotonic() < deadline:
        time.sleep(cfg.interval)
        now = time.monotonic()
        dt = now - t_prev
        pids = stack_pids()
        for pid, (name, _cmd) in pids.items():
            cur = thread_times(pid)
            base = prev.get(pid, {})
            for tid, (tname, ticks, cpu, affinity) in cur.items():
                old = base.get(tid)
                if old is None:
                    continue
                pct = 100.0 * (ticks - old[1]) / CLK_TCK / dt
                if pct <= 0.05:
                    continue
                samples.append({'t': round(now, 3), 'pid': pid, 'node': name,
                                'tid': tid, 'thread': tname,
                                'cpu_pct': round(pct, 2), 'last_cpu': cpu,
                                'affinity': affinity})
            prev[pid] = cur
            r, w = process_io(pid)
            pr, pw = prev_io.get(pid, (r, w))
            if r - pr or w - pw:
                samples.append({'t': round(now, 3), 'pid': pid, 'node': name,
                                'read_kbs': round((r - pr) / 1024.0 / dt, 1),
                                'write_kbs': round((w - pw) / 1024.0 / dt, 1)})
            prev_io[pid] = (r, w)
        t_prev = now
    (out / 'procstat.json').write_text(json.dumps(
        {'source': '/proc (pidstat not installed)', 'clk_tck': CLK_TCK,
         'samples': samples}, indent=0))
    print(f'procstat: {len(samples)} samples -> {out}/procstat.json')
    return 0


DEFAULT_TOPICS = ('/scan,/odometry/filtered,/zed2/zed_node/depth/camera_info,'
                  '/perception/stereo_columns,/perception/scene_labels,'
                  '/perception/walls,/mpc/diagnostics,/diagnostics,'
                  '/perception/virtual_wall/debug,/perception/scene_labels/stats,'
                  '/mpc/status')

#: THE TYPE OF EACH TOPIC, WITHOUT ASKING THE GRAPH. Graph introspection is
#: not reliable here: it is empty for the first seconds of a node's life, and
#: under the Discovery Server this box's `ros2 topic list` is routinely blank
#: while the topics are flowing. Subscribing from a static table matches
#: whenever the publisher appears, which is the behaviour a rate counter has to
#: have -- the first dry run of this script measured 0 Hz on every topic for
#: exactly this reason. The graph is still consulted for anything not listed.
KNOWN_TYPES = {
    '/scan': 'sensor_msgs/msg/LaserScan',
    '/odometry/filtered': 'nav_msgs/msg/Odometry',
    '/ekf_global/odometry/filtered': 'nav_msgs/msg/Odometry',
    '/odom': 'nav_msgs/msg/Odometry',
    '/zed2/zed_node/depth/camera_info': 'sensor_msgs/msg/CameraInfo',
    '/camera/camera_info': 'sensor_msgs/msg/CameraInfo',
    '/perception/stereo_columns': 'f1tenth_messages/msg/StereoColumns',
    '/perception/scene_labels': 'sensor_msgs/msg/Image',
    '/perception/scene_labels/classes': 'sensor_msgs/msg/Image',
    '/perception/scene_labels/depth_mm': 'sensor_msgs/msg/Image',
    '/perception/scene_labels/stats': 'std_msgs/msg/String',
    '/perception/walls': 'f1tenth_messages/msg/MultiWallTrackArray',
    '/perception/wall_observations': 'f1tenth_messages/msg/WallObservationArray',
    '/perception/virtual_wall': 'std_msgs/msg/Float32',
    '/perception/virtual_wall/debug': 'std_msgs/msg/String',
    '/perception/front_distance': 'std_msgs/msg/Float32',
    '/perception/debug/front_wall': 'std_msgs/msg/String',
    '/diagnostics': 'diagnostic_msgs/msg/DiagnosticArray',
    '/mpc/status': 'std_msgs/msg/String',
    '/mpc/solver_status': 'f1tenth_messages/msg/MpcSolverStatus',
    '/drive': 'ackermann_msgs/msg/AckermannDriveStamped',
    '/tf': 'tf2_msgs/msg/TFMessage',
}


def run_rates(cfg):
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from rosidl_runtime_py.utilities import get_message

    wanted = [t for t in cfg.topics.split(',') if t]
    out = Path(cfg.out)
    out.mkdir(parents=True, exist_ok=True)

    class Counter(Node):
        def __init__(self):
            super().__init__('cpu_test_rate_counter')
            self.counts = {t: 0 for t in wanted}
            self.first = {}
            self.last = {}
            self.subscribed = {}
            self.unresolved = []
            for topic in wanted:
                self._try_subscribe(topic)

        def _try_subscribe(self, topic):
            if topic in self.subscribed:
                return True
            type_name = KNOWN_TYPES.get(topic)
            if type_name is None:
                graph = dict(self.get_topic_names_and_types())
                types = graph.get(topic)
                type_name = types[0] if types else None
            if type_name is None:
                return False
            try:
                msg_type = get_message(type_name)
            except Exception:                           # noqa: BLE001
                return False
            self.subscribed[topic] = self.create_subscription(
                msg_type, topic, lambda _m, t=topic: self._tick(t),
                qos_profile_sensor_data)
            return True

        def retry(self):
            """Subscribe to anything still unsubscribed: a publisher that
            appears mid-phase has to be counted from then on."""
            for topic in list(self.counts):
                self._try_subscribe(topic)

        def _tick(self, topic):
            now = time.monotonic()
            self.counts[topic] += 1
            self.first.setdefault(topic, now)
            self.last[topic] = now

    rclpy.init()
    node = Counter()
    deadline = time.monotonic() + cfg.duration
    next_retry = time.monotonic() + 2.0
    while rclpy.ok() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.2)
        if time.monotonic() >= next_retry:
            node.retry()
            next_retry = time.monotonic() + 2.0
    node.missing = [t for t in node.counts if t not in node.subscribed]
    rates = {}
    for topic, n in node.counts.items():
        span = node.last.get(topic, 0.0) - node.first.get(topic, 0.0)
        rates[topic] = {'messages': n,
                        'hz': round(n / span, 2) if span > 0.5 and n > 1 else None,
                        'span_s': round(span, 2)}
    payload = {'duration_s': cfg.duration, 'absent_from_the_graph': node.missing,
               'rates': rates}
    (out / 'rates.json').write_text(json.dumps(payload, indent=1))
    for topic, r in rates.items():
        print(f'  {topic:48s} {r["messages"]:6d} msgs  '
              f'{r["hz"] if r["hz"] is not None else "--":>8} Hz')
    if node.missing:
        print(f'  absent from the graph: {node.missing}')
    node.destroy_node()
    rclpy.shutdown()
    return 0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('mode', choices=['procstat', 'rates'])
    ap.add_argument('--out', required=True)
    ap.add_argument('--duration', type=float, default=60.0)
    ap.add_argument('--interval', type=float, default=1.0)
    ap.add_argument('--topics', default=DEFAULT_TOPICS)
    cfg = ap.parse_args()
    return run_procstat(cfg) if cfg.mode == 'procstat' else run_rates(cfg)


if __name__ == '__main__':
    sys.exit(main())
