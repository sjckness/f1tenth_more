#!/usr/bin/env python3
"""Sonde ROS 2 per l'HIL (rclpy; gira su linus e nell'immagine dell'Orin). L'ambiente DDS
(ROS_DOMAIN_ID, ROS_DISCOVERY_SERVER, ROS_SUPER_CLIENT, FASTRTPS_DEFAULT_PROFILES_FILE) lo
prepara chi chiama.

    ros_probe.py wait --timeout S [--window 3] [--tol 0.2] /topic:HZ ...
        aspetta che ogni topic esista e arrivi ad almeno HZ*(1-tol) messaggi/s (HZ 0 = basta
        un messaggio); una riga per topic, exit 0 se tutti ok
    ros_probe.py sub TOPIC --timeout S
        stampa READY quando la sottoscrizione esiste, GOT al primo std_msgs/String; exit 1 se scade
    ros_probe.py pingpong TOPIC --timeout S -- CMD...
        lancia CMD (es. ssh ... che esegue `sub` sull'altro host), aspetta READY, pubblica su
        TOPIC e misura il tempo fino a GOT: "visto dall'altro host in 0.84 s"
    ros_probe.py drive --seconds S --speed V [--topic /ackermann_drive] [--odom /odom] [--min-dist D]
        pubblica AckermannDriveStamped (sterzo 0) per S secondi, poi zero per 1 s; misura lo
        spostamento su --odom; exit 0 se >= D
"""
import argparse
import math
import subprocess
import sys
import threading
import time


def _rclpy():
    import rclpy  # noqa: PLC0415 - import tardivo: --help funziona senza ROS
    return rclpy


def cmd_wait(a):
    rclpy = _rclpy()
    from rclpy.qos import qos_profile_sensor_data
    from rosidl_runtime_py.utilities import get_message
    rclpy.init()
    node = rclpy.create_node('f1t_hil_wait')
    want = {}
    for spec in a.topics:
        name, _, hz = spec.partition(':')
        want[name] = float(hz or 0)
    stamps = {t: [] for t in want}
    subs = {}
    deadline = time.monotonic() + a.timeout
    done = {}
    while time.monotonic() < deadline and len(done) < len(want):
        types = dict(node.get_topic_names_and_types())
        for t in want:
            if t not in subs and t in types:
                msg = get_message(types[t][0])
                subs[t] = node.create_subscription(
                    msg, t, lambda _m, t=t: stamps[t].append(time.monotonic()),
                    qos_profile_sensor_data)
        rclpy.spin_once(node, timeout_sec=0.1)
        now = time.monotonic()
        for t, hz in want.items():
            if t in done or not stamps[t]:
                continue
            if hz <= 0:
                done[t] = 'visto'
                continue
            if now - stamps[t][0] >= a.window:
                recent = [s for s in stamps[t] if s >= now - a.window]
                rate = len(recent) / a.window
                if rate >= hz * (1 - a.tol):
                    done[t] = f'{rate:.1f} Hz'
    ok = True
    for t, hz in want.items():
        if t in done:
            print(f'OK   {t} {done[t]} (min {hz * (1 - a.tol):.1f})')
        else:
            ok = False
            n = len(stamps[t])
            why = 'mai visto nel grafo' if t not in subs else f'{n} messaggi, sotto {hz * (1 - a.tol):.1f} Hz'
            print(f'FAIL {t}: {why}')
    print(f'{len(done)}/{len(want)} topic ok')
    node.destroy_node()
    rclpy.shutdown()
    return 0 if ok else 1


def cmd_sub(a):
    rclpy = _rclpy()
    from std_msgs.msg import String
    rclpy.init()
    node = rclpy.create_node('f1t_hil_sub')
    got = []
    node.create_subscription(String, a.topic, lambda m: got.append(m.data), 10)
    print('READY', flush=True)
    deadline = time.monotonic() + a.timeout
    while time.monotonic() < deadline and not got:
        rclpy.spin_once(node, timeout_sec=0.05)
    print(f'GOT {got[0]}' if got else 'TIMEOUT', flush=True)
    node.destroy_node()
    rclpy.shutdown()
    return 0 if got else 1


def cmd_pingpong(a):
    cmd = [c for c in a.cmd if c != '--']
    p = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, bufsize=1)
    lines = []

    def reader():
        for line in p.stdout:
            lines.append((time.monotonic(), line.strip()))
    threading.Thread(target=reader, daemon=True).start()
    t_end = time.monotonic() + a.ready_timeout
    while time.monotonic() < t_end and not any(x[1] == 'READY' for x in lines):
        if p.poll() is not None and not any(x[1] == 'READY' for x in lines):
            break
        time.sleep(0.05)
    if not any(x[1] == 'READY' for x in lines):
        p.kill()
        tail = '; '.join(x[1] for x in lines[-3:]) or 'nessun output'
        print(f'il sottoscrittore remoto non e partito: {tail}')
        return 1
    rclpy = _rclpy()
    from std_msgs.msg import String
    rclpy.init()
    node = rclpy.create_node('f1t_hil_pub')
    pub = node.create_publisher(String, a.topic, 10)
    t0 = time.monotonic()
    got = None
    while time.monotonic() - t0 < a.timeout + 2:
        pub.publish(String(data='f1t-precheck'))
        rclpy.spin_once(node, timeout_sec=0.1)
        got = next((x for x in lines if x[1].startswith('GOT')), None)
        if got or p.poll() is not None:
            break
    node.destroy_node()
    rclpy.shutdown()
    try:
        p.wait(timeout=10)
    except subprocess.TimeoutExpired:
        p.kill()
    if not got:
        print(f'non visto dall altro host entro {a.timeout + 2:.0f} s')
        return 1
    dt = got[0] - t0
    print(f'visto dall altro host in {dt:.2f} s')
    return 0 if dt <= a.timeout else 10


def cmd_drive(a):
    rclpy = _rclpy()
    from ackermann_msgs.msg import AckermannDriveStamped
    from nav_msgs.msg import Odometry
    from rclpy.qos import qos_profile_sensor_data
    rclpy.init()
    node = rclpy.create_node('f1t_hil_drive')
    pub = node.create_publisher(AckermannDriveStamped, a.topic, 10)
    pos = []
    node.create_subscription(Odometry, a.odom, lambda m: pos.append(
        (m.pose.pose.position.x, m.pose.pose.position.y)), qos_profile_sensor_data)
    t_wait = time.monotonic() + 10
    while not pos and time.monotonic() < t_wait:
        rclpy.spin_once(node, timeout_sec=0.1)
    if not pos:
        print(f'nessuna {a.odom} in 10 s')
        return 1
    start = pos[-1]

    def send(v, seconds):
        t0 = time.monotonic()
        while time.monotonic() - t0 < seconds:
            m = AckermannDriveStamped()
            m.header.stamp = node.get_clock().now().to_msg()
            m.drive.speed = float(v)
            m.drive.steering_angle = 0.0
            pub.publish(m)
            rclpy.spin_once(node, timeout_sec=0.05)
    send(a.speed, a.seconds)
    send(0.0, 1.0)
    end = pos[-1]
    dist = math.hypot(end[0] - start[0], end[1] - start[1])
    print(f'{dist:.2f} m in {a.seconds:.0f} s a {a.speed} m/s (min {a.min_dist})')
    node.destroy_node()
    rclpy.shutdown()
    return 0 if dist >= a.min_dist else 1


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sp = ap.add_subparsers(dest='cmd', required=True)
    w = sp.add_parser('wait')
    w.add_argument('--timeout', type=float, default=60)
    w.add_argument('--window', type=float, default=3.0)
    w.add_argument('--tol', type=float, default=0.2)
    w.add_argument('topics', nargs='+')
    s = sp.add_parser('sub')
    s.add_argument('topic')
    s.add_argument('--timeout', type=float, default=10)
    pp = sp.add_parser('pingpong')
    pp.add_argument('topic')
    pp.add_argument('--timeout', type=float, default=3)
    pp.add_argument('--ready-timeout', type=float, default=40)
    pp.add_argument('cmd', nargs=argparse.REMAINDER)
    d = sp.add_parser('drive')
    d.add_argument('--seconds', type=float, default=10)
    d.add_argument('--speed', type=float, default=0.3)
    d.add_argument('--topic', default='/ackermann_drive')
    d.add_argument('--odom', default='/odom')
    d.add_argument('--min-dist', type=float, default=1.0)
    a = ap.parse_args()
    return {'wait': cmd_wait, 'sub': cmd_sub, 'pingpong': cmd_pingpong, 'drive': cmd_drive}[a.cmd](a)


if __name__ == '__main__':
    sys.exit(main())
