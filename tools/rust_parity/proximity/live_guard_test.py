#!/usr/bin/env python3
"""Live test: the Rust proximity_guard_node vs the Python IsProximityTooClose.

Runs, on an isolated localhost ROS domain:
  - proximity_guard_node mode:=active  (stops on /parity/rs_stop),
  - proximity_guard_node mode:=shadow  (must publish no command at all),
  - a Python host node with the REAL IsProximityTooClose and its event-driven
    e-stop (cpu-cleanup: immediate_stop_topic, stop from the /scan callback;
    the BT tick path adds up to one 10 Hz tick on top),
and drives them with /tf_static + synthetic /parity/scan at 40 Hz:
  A  clear / trip pattern     -> scan stamp -> stop latency, both sides
  B  scans stop for 2 s       -> Rust: stale stop; Python: nothing
  C  clear scans again        -> Rust stops cease
  D  front sector all NaN     -> Rust: invalid_beams stop; Python: nothing
CPU time per process is read from /proc around phase A.

Usage (workspace sourced): live_guard_test.py [--out output/rust/live_guard.json]
"""

import argparse
import json
import math
import os
import random
import signal
import subprocess
import sys
import threading
import time

import numpy as np

FRONT_X = 0.405
THRESHOLD = 0.15
N = 1081
ANGLE_MIN = -2.356194496154785
INC = 0.004363323096185923


def py_host():
    """--py-host: the real IsProximityTooClose with its event stop, spinning."""
    import rclpy
    from f1tenth_behavior.behaviours.is_proximity_too_close import IsProximityTooClose
    rclpy.init()
    node = rclpy.create_node('py_proximity_host')
    b = IsProximityTooClose(scan_topic='/parity/scan', front_distance_threshold=THRESHOLD,
                            base_front_x_m=FRONT_X, immediate_stop_topic='/parity/py_stop',
                            immediate_stop_frame_id='base_link/emergency')
    b.setup(node=node)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass


def cpu_seconds(pid):
    with open(f'/proc/{pid}/stat') as f:
        fields = f.read().rsplit(')', 1)[1].split()
    return (int(fields[11]) + int(fields[12])) / os.sysconf('SC_CLK_TCK')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--py-host', action='store_true')
    ap.add_argument('--out', default='output/rust/live_guard.json')
    ap.add_argument('--phase-a-s', type=float, default=20.0)
    a = ap.parse_args()
    if a.py_host:
        py_host()
        return 0

    env = dict(os.environ)
    env['ROS_DOMAIN_ID'] = str(random.randint(150, 220))
    env['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'
    env.pop('ROS_DISCOVERY_SERVER', None)
    env.pop('ROS_SUPER_CLIENT', None)
    os.environ.update({k: env[k] for k in ('ROS_DOMAIN_ID', 'ROS_AUTOMATIC_DISCOVERY_RANGE')})
    os.environ.pop('ROS_DISCOVERY_SERVER', None)

    from ament_index_python.packages import get_package_prefix
    exe = os.path.join(get_package_prefix('f1tenth_proximity_guard'), 'lib',
                       'f1tenth_proximity_guard', 'proximity_guard_node')
    common = ['--ros-args', '-p', f'base_front_x_m:={FRONT_X}',
              '-p', f'front_distance_threshold_m:={THRESHOLD}',
              '-p', 'scan_topic:=/parity/scan']
    procs = {
        'rs_active': subprocess.Popen(
            [exe] + common + ['-p', 'mode:=active', '-p', 'stop_topic:=/parity/rs_stop',
                              '-p', 'decision_topic:=/parity/rs_decision'],
            env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL),
        'rs_shadow': subprocess.Popen(
            [exe] + common + ['-r', '__node:=proximity_guard_shadow', '-p', 'mode:=shadow',
                              '-p', 'stop_topic:=/parity/rs_shadow_stop',
                              '-p', 'decision_topic:=/parity/rs_shadow_decision'],
            env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL),
        'py_host': subprocess.Popen([sys.executable, __file__, '--py-host'], env=env,
                                    stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL),
    }

    import rclpy
    from ackermann_msgs.msg import AckermannDriveStamped
    from diagnostic_msgs.msg import DiagnosticStatus
    from geometry_msgs.msg import TransformStamped
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import LaserScan
    from tf2_msgs.msg import TFMessage

    rclpy.init()
    node = rclpy.create_node('guard_driver')
    rx = {k: [] for k in ('rs_stop', 'py_stop', 'rs_shadow_stop', 'rs_decision',
                          'rs_shadow_decision')}
    lock = threading.Lock()

    def recorder(key):
        def cb(msg):
            with lock:
                rx[key].append((time.time_ns(), msg))
        return cb

    for key, typ in (('rs_stop', AckermannDriveStamped), ('py_stop', AckermannDriveStamped),
                     ('rs_shadow_stop', AckermannDriveStamped),
                     ('rs_decision', DiagnosticStatus), ('rs_shadow_decision', DiagnosticStatus)):
        node.create_subscription(typ, f'/parity/{key}', recorder(key), 100)
    scan_pub = node.create_publisher(LaserScan, '/parity/scan', 10)
    tf_pub = node.create_publisher(TFMessage, '/tf_static', QoSProfile(
        depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL,
        reliability=ReliabilityPolicy.RELIABLE))
    spin = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin.start()

    t = TransformStamped()
    t.header.frame_id = 'base_link'
    t.child_frame_id = 'laser'
    t.transform.translation.x = 0.305
    t.transform.translation.z = 0.19
    t.transform.rotation.w = 1.0
    tf_pub.publish(TFMessage(transforms=[t]))

    ang = ANGLE_MIN + np.arange(N) * INC
    clear = [30.0] * N
    trip = list(clear)
    for i in np.flatnonzero(np.abs(ang) < 0.05):
        trip[i] = 0.20      # 0.10 m from the bumper
    nan_front = list(clear)
    for i in np.flatnonzero(np.abs(ang) < math.radians(45)):
        nan_front[i] = math.nan

    def publish(ranges):
        m = LaserScan()
        now = time.time_ns()
        m.header.stamp.sec, m.header.stamp.nanosec = divmod(now, 1_000_000_000)
        m.header.frame_id = 'laser'
        m.angle_min, m.angle_max, m.angle_increment = ANGLE_MIN, -ANGLE_MIN, INC
        m.range_min, m.range_max = 0.02, 30.0
        m.ranges = ranges
        scan_pub.publish(m)
        return now

    # Discovery + /tf_static latching.
    deadline = time.time() + 20
    while time.time() < deadline and (scan_pub.get_subscription_count() < 3):
        time.sleep(0.1)
    for _ in range(80):     # 2 s warm-up of clear scans
        publish(clear)
        time.sleep(0.025)
    with lock:
        for v in rx.values():
            v.clear()

    res = dict(domain=env['ROS_DOMAIN_ID'], subscribers=scan_pub.get_subscription_count())
    # ---- A
    cpu0 = {k: cpu_seconds(p.pid) for k, p in procs.items()}
    trips = []
    t_end = time.time() + a.phase_a_s
    k = 0
    last_a = 0
    while time.time() < t_end:
        is_trip = (k % 5 == 4)
        stamp = last_a = publish(trip if is_trip else clear)
        if is_trip:
            trips.append(stamp)
        k += 1
        time.sleep(0.025)
    time.sleep(0.3)
    cpu1 = {k2: cpu_seconds(p.pid) for k2, p in procs.items()}
    with lock:
        snap = {key: list(v) for key, v in rx.items()}
    scans_a = k

    def latencies(key):
        times = [tt for tt, _ in snap[key]]
        out = []
        for j, s in enumerate(trips):
            nxt = trips[j + 1] if j + 1 < len(trips) else s + 10**9
            hit = [tt for tt in times if s < tt < nxt]
            out.append((hit[0] - s) / 1e6 if hit else None)
        return out

    def summary(vals):
        v = np.array([x for x in vals if x is not None])
        if not len(v):
            return dict(n=0, missing=len(vals))
        return dict(n=int(len(v)), missing=int(len(vals) - len(v)), p50=float(np.percentile(v, 50)),
                    p95=float(np.percentile(v, 95)), max=float(v.max()), mean=float(v.mean()))

    compute = [float(next(kv.value for kv in m.values if kv.key == 'compute_us'))
               for _, m in snap['rs_decision']
               if any(kv.key == 'compute_us' and kv.value != 'none' for kv in m.values)]
    res['A'] = dict(
        scans=scans_a, trip_scans=len(trips),
        rs_stop_latency_ms=summary(latencies('rs_stop')),
        py_event_stop_latency_ms=summary(latencies('py_stop')),
        # Stops within the scan stream (+50 ms); the 0.3 s tail after the
        # last scan already holds the first stale stop (phase B measures it).
        rs_stops=sum(1 for tt, _ in snap['rs_stop'] if tt < last_a + 50_000_000),
        py_stops=sum(1 for tt, _ in snap['py_stop'] if tt < last_a + 50_000_000),
        rs_shadow_stops=len(snap['rs_shadow_stop']),
        rs_decisions=len(snap['rs_decision']),
        rs_compute_us=dict(p50=float(np.percentile(compute, 50)), p95=float(np.percentile(compute, 95)),
                           max=float(max(compute))) if compute else None,
        cpu_ms_per_scan={k2: 1000.0 * (cpu1[k2] - cpu0[k2]) / scans_a for k2 in cpu0},
        tf_fallback_seen=any(kv.key == 'tf_fallback' and kv.value == 'true'
                             for _, m in snap['rs_decision'] for kv in m.values))

    # ---- B: no scans
    with lock:
        for v in rx.values():
            v.clear()
    last = publish(clear)
    time.sleep(2.0)
    with lock:
        snap = {key: list(v) for key, v in rx.items()}
    rs_b = [tt for tt, _ in snap['rs_stop']]
    stale_dec = [(tt, m.message) for tt, m in snap['rs_decision'] if m.message == 'stale']
    res['B'] = dict(
        rs_first_stale_stop_after_last_scan_ms=(rs_b[0] - last) / 1e6 if rs_b else None,
        rs_stale_stops_in_2s=len(rs_b), py_stops_in_2s=len(snap['py_stop']),
        rs_stale_decisions=len(stale_dec),
        rs_max_gap_between_stale_stops_ms=(float(np.max(np.diff(rs_b))) / 1e6
                                           if len(rs_b) > 1 else None),
        rs_shadow_stops=len(snap['rs_shadow_stop']))

    # ---- C: clear again
    with lock:
        for v in rx.values():
            v.clear()
    first = None
    for _ in range(80):
        s = publish(clear)
        first = first or s
        time.sleep(0.025)
    time.sleep(0.2)
    with lock:
        snap = {key: list(v) for key, v in rx.items()}
    late = [tt for tt, _ in snap['rs_stop'] if tt > first + 100_000_000]
    res['C'] = dict(rs_stops_total=len(snap['rs_stop']), rs_stops_after_100ms=len(late),
                    py_stops=len(snap['py_stop']))

    # ---- D: front sector NaN
    with lock:
        for v in rx.values():
            v.clear()
    for _ in range(40):
        publish(nan_front)
        time.sleep(0.025)
    time.sleep(0.2)
    with lock:
        snap = {key: list(v) for key, v in rx.items()}
    res['D'] = dict(scans=40, rs_stops=len(snap['rs_stop']), py_stops=len(snap['py_stop']),
                    rs_reasons=sorted({m.message for _, m in snap['rs_decision']}),
                    rs_shadow_stops=len(snap['rs_shadow_stop']))

    for p in procs.values():
        p.send_signal(signal.SIGINT)
    for p in procs.values():
        try:
            p.wait(timeout=5)
        except subprocess.TimeoutExpired:
            p.kill()
    node.destroy_node()
    rclpy.try_shutdown()
    os.makedirs(os.path.dirname(a.out) or '.', exist_ok=True)
    with open(a.out, 'w') as f:
        json.dump(res, f, indent=1)
    print(json.dumps(res, indent=1))
    return 0


if __name__ == '__main__':
    sys.exit(main())
