#!/usr/bin/env python3
r"""CPU of the three R3 TF nodes with the old and the new TF path, under the car's real /tf.

For each node and each setting of its switch (old: a TransformListener; new:
R3's default) this starts the installed node as its own process on a private
localhost-only DDS domain, plays a recorded bag's /tf in real time (and its
/tf_static once, latched) from this process, and reads the node's CPU time
(utime + stime of /proc/<pid>/stat, every thread) over a window after a
warm-up, plus its thread count.

    node                     old                       new (default)
    mpc_corr                 tf_polling:=false         tf_polling:=true
    detection_3d_node        tf_static_only:=false     tf_static_only:=true
    obstacle_projector_node  tf_static_only:=false     tf_static_only:=true

Nothing else is published: the nodes idle except for TF, so the difference is
what the TF path costs. mpc_corr still runs its 10 Hz control loop (no odometry:
the early-return branch) in both settings.

Usage (workspace install sourced; nothing else on the domain):
    python3 tools/r3_tf_cpu.py ~/bags/corridor10m_20261016T144537 \
        --seconds 30 --json output/rust_main/r3_tf_cpu.json
"""

import argparse
import json
import os
import signal
import subprocess
import sys
import time

CASES = [
    ('mpc_controller', 'mpc_corr', 'tf_polling'),
    ('f1tenth_perception', 'detection_3d_node', 'tf_static_only'),
    ('f1tenth_perception', 'obstacle_projector_node', 'tf_static_only'),
]


def read_tf(bag):
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from tf2_msgs.msg import TFMessage
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=bag, storage_id='sqlite3'),
                rosbag2_py.ConverterOptions('', ''))
    reader.set_filter(rosbag2_py.StorageFilter(topics=['/tf', '/tf_static']))
    dynamic, static = [], []
    while reader.has_next():
        topic, data, t_ns = reader.read_next()
        msg = deserialize_message(data, TFMessage)
        (static if topic == '/tf_static' else dynamic).append((t_ns, msg))
    return dynamic, static


def cpu_ticks(pid):
    with open(f'/proc/{pid}/stat') as f:
        fields = f.read().rsplit(')', 1)[1].split()
    return int(fields[11]) + int(fields[12])      # utime + stime (fields 14, 15)


def threads(pid):
    with open(f'/proc/{pid}/status') as f:
        for line in f:
            if line.startswith('Threads:'):
                return int(line.split()[1])
    return -1


def executable(package, exe):
    from ament_index_python.packages import get_package_prefix
    return os.path.join(get_package_prefix(package), 'lib', package, exe)


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('bag')
    ap.add_argument('--seconds', type=float, default=30.0, help='measurement window')
    ap.add_argument('--warmup', type=float, default=8.0)
    ap.add_argument('--domain', type=int, default=97)
    ap.add_argument('--json')
    a = ap.parse_args()

    for key in ('ROS_DISCOVERY_SERVER', 'ROS_SUPER_CLIENT', 'ROS_STATIC_PEERS',
                'FASTRTPS_DEFAULT_PROFILES_FILE'):
        os.environ.pop(key, None)
    os.environ['ROS_DOMAIN_ID'] = str(a.domain)
    os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'

    import rclpy
    from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile
    from tf2_msgs.msg import TFMessage

    dynamic, static = read_tf(a.bag)
    rate = len(dynamic) / ((dynamic[-1][0] - dynamic[0][0]) * 1e-9)
    rclpy.init()
    player = rclpy.create_node('r3_tf_cpu_player')
    tf_pub = player.create_publisher(TFMessage, '/tf', QoSProfile(
        depth=100, durability=DurabilityPolicy.VOLATILE, history=HistoryPolicy.KEEP_LAST))
    static_pub = player.create_publisher(TFMessage, '/tf_static', QoSProfile(
        depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL,
        history=HistoryPolicy.KEEP_LAST))
    for _, msg in static:
        static_pub.publish(msg)
    hz = os.sysconf('SC_CLK_TCK')
    results = []
    for package, exe, switch in CASES:
        for label, value in (('old', 'false'), ('new', 'true')):
            proc = subprocess.Popen(
                [executable(package, exe), '--ros-args', '--log-level', 'warn',
                 '-p', f'{switch}:={value}'],
                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            i, t_play0, bag0 = 0, time.monotonic(), dynamic[0][0]
            sent = 0
            window = None
            deadline = a.warmup + a.seconds
            while True:
                elapsed = time.monotonic() - t_play0
                if elapsed >= deadline or proc.poll() is not None:
                    break
                if window is None and elapsed >= a.warmup:
                    window = (time.monotonic(), cpu_ticks(proc.pid), sent)
                # Real-time playback, looping the bag.
                while (dynamic[i][0] - bag0) * 1e-9 <= elapsed:
                    tf_pub.publish(dynamic[i][1])
                    sent += 1
                    i += 1
                    if i == len(dynamic):
                        i, bag0 = 0, dynamic[0][0] - int(elapsed * 1e9)
                time.sleep(0.002)
            alive = proc.poll() is None
            row = {'node': exe, 'path': label, f'{switch}': value, 'alive': alive}
            if alive and window is not None:
                t1, c1 = time.monotonic(), cpu_ticks(proc.pid)
                row.update({
                    'cpu_percent': round(100.0 * (c1 - window[1]) / hz / (t1 - window[0]), 2),
                    'tf_msgs_per_s': round((sent - window[2]) / (t1 - window[0]), 1),
                    'threads': threads(proc.pid)})
            proc.send_signal(signal.SIGINT)
            try:
                proc.wait(timeout=10)
            except subprocess.TimeoutExpired:
                proc.kill()
                proc.wait()
            results.append(row)
            print(json.dumps(row), flush=True)
    report = {'bag': os.path.abspath(a.bag), 'bag_tf_rate_hz': round(rate, 1),
              'window_s': a.seconds, 'warmup_s': a.warmup, 'clk_tck': hz,
              'host': os.uname().nodename, 'results': results}
    if a.json:
        with open(a.json, 'w') as f:
            json.dump(report, f, indent=1)
            f.write('\n')
    player.destroy_node()
    rclpy.shutdown()
    return 0 if all(r['alive'] for r in results) else 1


if __name__ == '__main__':
    sys.exit(main())
