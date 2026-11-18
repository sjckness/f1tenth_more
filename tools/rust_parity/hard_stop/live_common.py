"""Shared pieces of the live hard-stop tests (live_hard_stop_test.py, timeout_parity.py).

Both run the REAL nodes as processes on an isolated localhost ROS domain
(random ROS_DOMAIN_ID 150-220, ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST, no
Discovery Server): the Python hard_stop_node (f1tenth_control) and the Rust
one (f1tenth_hard_stop_rs), side by side under their own names and topics
(/live/<tag>_stop, /live/<tag>_diag), fed by one driver node in this process:
the same /live/scan messages, the same /drive commands, one /clock.

Processes are stopped by PID (SIGINT, then kill), never by pattern.
"""

import os
import random
import signal
import subprocess
import threading
import time

N = 1081
ANGLE_MIN = -2.356194496154785       # the car's UST: -135 deg as f32
INC = 0.004363323096185923           # 0.25 deg as f32
FRONT_X = 0.405                      # vehicle_frames.base_front_x()
HALF_WIDTH = 0.136                   # stack_params swept_clearance_body_half_width_m
I0 = int(round(-ANGLE_MIN / INC))    # the beam straight ahead


def isolated_env():
    """Isolate this process and its children: a random domain, localhost only."""
    env = dict(os.environ)
    env['ROS_DOMAIN_ID'] = str(random.randint(150, 220))
    env['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'
    for k in ('ROS_DISCOVERY_SERVER', 'ROS_SUPER_CLIENT', 'FASTRTPS_DEFAULT_PROFILES_FILE'):
        env.pop(k, None)
        os.environ.pop(k, None)
    os.environ.update({k: env[k] for k in ('ROS_DOMAIN_ID', 'ROS_AUTOMATIC_DISCOVERY_RANGE')})
    return env


def executables():
    from ament_index_python.packages import get_package_prefix
    return {
        'py': os.path.join(get_package_prefix('f1tenth_control'), 'lib', 'f1tenth_control',
                           'hard_stop_node'),
        'rust': os.path.join(get_package_prefix('f1tenth_hard_stop_rs'), 'lib',
                             'f1tenth_hard_stop_rs', 'hard_stop_node'),
    }


def start_node(impl, tag, env, log_dir, sim=False, params=None):
    """One hard_stop_node process; its output goes to <log_dir>/<tag>.log."""
    exe = executables()[impl]
    args = [exe, '--ros-args', '-r', f'__node:=hs_{tag}',
            '-p', 'scan_topic:=/live/scan', '-p', f'stop_topic:=/live/{tag}_stop',
            '-p', f'diag_topic:=/live/{tag}_diag',
            '-p', f'use_sim_time:={"true" if sim else "false"}']
    if impl == 'rust':
        # What ackermann_mux.launch.py passes (the Python's own defaults).
        args += ['-p', f'footprint_front_x_m:={FRONT_X}',
                 '-p', f'footprint_half_width_m:={HALF_WIDTH}']
    for k, v in (params or {}).items():
        args += ['-p', f'{k}:={v}']
    log = open(os.path.join(log_dir, f'{tag}.log'), 'w')
    proc = subprocess.Popen(args, env=env, stdout=log, stderr=subprocess.STDOUT)
    proc.log_file = log
    return proc


def stop_nodes(procs):
    """SIGINT each node, kill what is still up after 5 s: {tag: how it ended}."""
    out = {}
    for t, p in procs.items():
        out[t] = 'already exited' if p.poll() is not None else None
        if out[t] is None:
            p.send_signal(signal.SIGINT)
    for t, p in procs.items():
        try:
            rc = p.wait(timeout=5)
            out[t] = out[t] or f'SIGINT, exit {rc}'
        except subprocess.TimeoutExpired:
            p.kill()
            p.wait()
            out[t] = 'killed after 5 s'
        p.log_file.close()
    return out


def cpu_seconds(pid):
    """utime + stime of a process, seconds."""
    with open(f'/proc/{pid}/stat') as f:
        fields = f.read().rsplit(')', 1)[1].split()
    return (int(fields[11]) + int(fields[12])) / os.sysconf('SC_CLK_TCK')


def ranges(points=(), fill=30.0):
    """A scan: fill everywhere, (beam index, range) placed."""
    r = [fill] * N
    for i, v in points:
        r[i] = v
    return r


CLEAR = ranges()
TRIP_AHEAD = ranges([(I0 - 1, 0.18), (I0, 0.18), (I0 + 1, 0.18)])


class Driver:
    """The driver node: scans, /drive, /clock out; every stop and diagnostic in."""

    def __init__(self, tags, sim_clock=False):
        import rclpy
        from ackermann_msgs.msg import AckermannDriveStamped
        from rclpy.qos import qos_profile_sensor_data
        from rosgraph_msgs.msg import Clock
        from sensor_msgs.msg import LaserScan
        from std_msgs.msg import String
        self._LaserScan = LaserScan
        self._Drive = AckermannDriveStamped
        self._Clock = Clock
        rclpy.init()
        self.node = rclpy.create_node('hs_live_driver')
        self.lock = threading.Lock()
        self.tags = list(tags)
        self.stops = {t: [] for t in self.tags}
        self.diags = {t: [] for t in self.tags}
        self.scan_pub = self.node.create_publisher(LaserScan, '/live/scan',
                                                   qos_profile_sensor_data)
        self.drive_pub = self.node.create_publisher(AckermannDriveStamped, '/drive', 10)
        self.clock_pub = self.node.create_publisher(Clock, '/clock', 10)

        def on_stop(tag):
            def cb(m):
                with self.lock:
                    self.stops[tag].append((time.monotonic(), m.header.frame_id,
                                            m.drive.speed, m.drive.steering_angle))
            return cb

        def on_diag(tag):
            def cb(m):
                with self.lock:
                    self.diags[tag].append((time.monotonic(), m.data))
            return cb
        for t in self.tags:
            self.node.create_subscription(AckermannDriveStamped, f'/live/{t}_stop', on_stop(t),
                                          1000)
            self.node.create_subscription(String, f'/live/{t}_diag', on_diag(t), 1000)
        self.last_drive = None
        self.sim = {'ns': 1_000_000_000_000, 'frozen': False, 'last': time.monotonic(),
                    'run': True, 'enabled': sim_clock}
        self._spin = threading.Thread(target=rclpy.spin, args=(self.node,), daemon=True)
        self._spin.start()
        if sim_clock:
            threading.Thread(target=self._clock_loop, daemon=True).start()

    def _clock_loop(self):
        while self.sim['run']:
            now = time.monotonic()
            if not self.sim['frozen']:
                self.sim['ns'] += int((now - self.sim['last']) * 1e9)
            self.sim['last'] = now
            c = self._Clock()
            c.clock.sec, c.clock.nanosec = divmod(self.sim['ns'], 1_000_000_000)
            self.clock_pub.publish(c)
            time.sleep(0.005)

    def freeze(self, frozen):
        self.sim['frozen'] = frozen

    def scan(self, r):
        m = self._LaserScan()
        if self.sim['enabled']:
            m.header.stamp.sec, m.header.stamp.nanosec = divmod(self.sim['ns'], 1_000_000_000)
        else:
            m.header.stamp = self.node.get_clock().now().to_msg()
        m.header.frame_id = 'laser'
        m.angle_min, m.angle_max, m.angle_increment = ANGLE_MIN, -ANGLE_MIN, INC
        m.range_min, m.range_max, m.ranges = 0.02, 30.0, r
        self.scan_pub.publish(m)
        return time.monotonic()

    def drive(self, speed):
        m = self._Drive()
        m.header.stamp = self.node.get_clock().now().to_msg()
        m.drive.speed = float(speed)
        self.drive_pub.publish(m)
        self.last_drive = time.monotonic()
        return self.last_drive

    def stream(self, r, seconds, period=0.026, drive=None, drive_every=2):
        """Scans every `period` for `seconds`; with `drive`, a /drive command right
        after every `drive_every`-th scan. Returns the publish times of the scans."""
        times = []
        end = time.monotonic() + seconds
        k = 0
        nxt = time.monotonic()
        while time.monotonic() < end:
            times.append(self.scan(r))
            if drive is not None and k % drive_every == 0:
                self.drive(drive)
            k += 1
            nxt += period
            time.sleep(max(0.0, nxt - time.monotonic()))
        return times

    def wait_ready(self, n_nodes, timeout=30.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            subs = self.scan_pub.get_subscription_count()
            drive = self.drive_pub.get_subscription_count()
            pubs = [self.node.count_publishers(f'/live/{t}_diag') for t in self.tags]
            if subs >= n_nodes and drive >= n_nodes and all(p >= 1 for p in pubs):
                return True
            time.sleep(0.1)
        return False

    def mark(self):
        with self.lock:
            return ({t: len(v) for t, v in self.stops.items()},
                    {t: len(v) for t, v in self.diags.items()})

    def since(self, mark):
        with self.lock:
            return ({t: list(v[mark[0][t]:]) for t, v in self.stops.items()},
                    {t: list(v[mark[1][t]:]) for t, v in self.diags.items()})

    def qos(self, topic, publishers=True):
        """{node name: (reliability, durability, history, depth)} on a topic."""
        infos = (self.node.get_publishers_info_by_topic(topic) if publishers
                 else self.node.get_subscriptions_info_by_topic(topic))
        return {i.node_name: (i.qos_profile.reliability.name, i.qos_profile.durability.name,
                              i.qos_profile.history.name, i.qos_profile.depth) for i in infos}

    def close(self):
        import rclpy
        self.sim['run'] = False
        time.sleep(0.02)
        self.node.destroy_node()
        rclpy.try_shutdown()
