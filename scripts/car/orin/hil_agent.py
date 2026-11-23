#!/usr/bin/env python3
"""car-hil-agent - the ROS side of the office HIL batch, run INSIDE the HIL container.

Installed by scripts/car/deploy.sh as ~/.local/bin/car-hil-agent on the Orin; car-hil (hil.sh) copies
it into the container at `car-hil up` and runs it with `docker exec` in the HIL DDS environment.
It never starts or stops processes of the stack: that is car-hil's job.

    car-hil-agent profile --ip IP [--buffer-mb N] --out FILE
        render the stack side's Fast DDS profile for the HIL link (template below)
    car-hil-agent wait --timeout S TOPIC[:MIN_HZ] ...
        exit 0 once every topic has delivered messages (and, with :MIN_HZ, at that rate over 3 s)
    car-hil-agent ready --node NAME --timeout S
        the stack can take a mission: the controller NAME is in the graph and no other controller
        is, /mission/status, /odometry/filtered and /sim/ground_truth/odom flow
    car-hil-agent run --session DIR --run-id ID --mode MODE --rep N --pose-idx I --mission FILE
                      [--timeout S] [--settle S]
        one repetition: /reset_all + checks, load + start the mission, record until it ends
        (COMPLETE / ABORTED) + 3 s, or abort it at --timeout (wall). Writes DIR/runs/ID/.
    car-hil-agent mark --session DIR --run-id ID --mode MODE --rep N --pose-idx I --status S [--note T]
        a repetition that could not run (stack not ready, Isaac down): its result.json, so the
        analysis counts it as that mode's failure
    car-hil-agent versions --out FILE
        package.xml versions of the installed workspace (the report's `modules:`)
    car-hil-agent analyze --session DIR
        every run of the session -> DIR/analysis/ (runs.csv, summary_by_mode.csv, by_pose.csv,
        report.md, figures/). Pure computation, no ROS: it also runs on linus (python3 + numpy,
        matplotlib optional).

What a run judges, and from what (analysis): the mission's own verdict (/mission/status final
state and the /mission/move_outcome of each move) AND the ground truth (/sim/ground_truth/odom,
Isaac's world frame) against the TRUE doorway of the pose (start poses file): did the car's centre
cross the doorway line inside the opening, and does it end standing in it. The two are reported
separately, because a mission can report door_entered on a wrongly placed door and the car can be
in the doorway while the mission timed out.
"""
from __future__ import annotations

import argparse
import csv
import json
import math
import os
import socket
import sys
import threading
import time

CONTROLLERS = {'corridor_mpc': 'mpc_corr', 'corridor_rrt': 'corridor_rrt',
               'p2p_geometric': 'p2p_geometric'}
ACTIVE_STATES = ('LOADED', 'RUNNING', 'HOLDING')
CAR_HALF_WIDTH_M = 0.186          # office_missions.py / pick_spawn_poses.py footprint
DOORWAY_ALONG_TOL_M = 0.30        # ground-truth "standing in the doorway": |along| <= this
DOORWAY_YAW_TOL_DEG = 20.0
RESET_POS_TOL_M, RESET_YAW_TOL_DEG = 0.05, 1.5     # the HIL office settle (dce14b9e): 5 cm
SPEED_TOL = 0.05
LOCAL_TOL, GLOBAL_TOL, LOC_YAW_TOL = 0.05, 0.15, 0.05
REZERO_TOL_M, REZERO_WAIT_S = 0.02, 5.0     # rezero_local(): /odom and /odometry/filtered back at 0
PATHS_MIN_PERIOD_S = 0.5          # throttle of the path / corridor / door logs (disk)
PACKAGES = ('f1tenth_behavior', 'f1tenth_planners', 'mpc_controller', 'f1tenth_costmap',
            'f1tenth_bringup', 'f1tenth_params', 'f1tenth_navigation', 'f1tenth_perception',
            'f1tenth_localization', 'f1tenth_messages')

# The stack side of the HIL link. Same content as scripts/sim/win/dds/hil.xml.in (the sim side's
# template, from hil-isaac's src/f1tenth_bringup/config/sim/dds/hil.xml.in) with shared memory on:
# a 5 s lease like config/fastdds_profile.xml, 16 MB socket buffers for the camera images, UDP
# bound to the link address. NOT tools/core/hil/dds/hil.xml.in: that copy declares
# flow_controller_descriptors, which Jazzy's Fast DDS rejects, and a rejected profile is ignored
# silently (RTF 0.44, docs/analysis/simulation/2026-11-13_hil-windows-wsl).
# scripts/car/tests/test_hil_agent.py checks this stays the same as the sim side's template.
PROFILE_TEMPLATE = """<?xml version="1.0" encoding="UTF-8"?>
<profiles xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">
  <transport_descriptors>
    <transport_descriptor>
      <transport_id>f1t_shm</transport_id>
      <type>SHM</type>
    </transport_descriptor>
    <transport_descriptor>
      <transport_id>f1t_udp_hil</transport_id>
      <type>UDPv4</type>
      <maxMessageSize>65000</maxMessageSize>
      <sendBufferSize>@BUFFER_BYTES@</sendBufferSize>
      <receiveBufferSize>@BUFFER_BYTES@</receiveBufferSize>
      <interfaceWhiteList>
        <address>@ETH_IP@</address>
        <address>127.0.0.1</address>
      </interfaceWhiteList>
    </transport_descriptor>
  </transport_descriptors>
  <participant profile_name="f1tenth_hil" is_default_profile="true">
    <rtps>
      <builtin>
        <discovery_config>
          <leaseDuration><sec>5</sec></leaseDuration>
          <leaseAnnouncement><sec>1</sec></leaseAnnouncement>
        </discovery_config>
      </builtin>
      <userTransports>
        @SHM_TRANSPORT@
        <transport_id>f1t_udp_hil</transport_id>
      </userTransports>
      <useBuiltinTransports>false</useBuiltinTransports>
    </rtps>
  </participant>
</profiles>
"""


def render_profile(ip: str, buffer_mb: int, shm: bool = True) -> str:
    text = PROFILE_TEMPLATE
    for key, value in (('@ETH_IP@', ip),
                       ('@SHM_TRANSPORT@', '<transport_id>f1t_shm</transport_id>' if shm else ''),
                       ('@BUFFER_BYTES@', str(buffer_mb * 1024 * 1024))):
        text = text.replace(key, value)
    return text


def log(msg: str) -> None:
    print(f'[agent {time.strftime("%H:%M:%S")}] {msg}', flush=True)


def yaw_of(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap(a: float) -> float:
    return (a + math.pi) % (2.0 * math.pi) - math.pi


# ============================================================================ ROS side
def _ros():
    """Import rclpy and the message types lazily: `analyze` runs on machines without ROS."""
    import rclpy  # noqa: F401
    return rclpy


def find_pids(node_name: str) -> list[int]:
    pids, me = [], os.getpid()
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


def proc_cpu_s(pid: int) -> float:
    with open(f'/proc/{pid}/stat') as f:
        fields = f.read().rsplit(')', 1)[1].split()
    return (int(fields[11]) + int(fields[12])) / os.sysconf('SC_CLK_TCK')


def cmd_profile(a) -> int:
    text = render_profile(a.ip, a.buffer_mb)
    import xml.etree.ElementTree as ET
    ET.fromstring(text)      # a malformed profile is ignored silently by Fast DDS: fail here
    with open(a.out, 'w') as f:
        f.write(text)
    log(f'profile {a.out}: UDP on {a.ip} + 127.0.0.1, SHM, buffers {a.buffer_mb} MB, lease 5 s')
    return 0


TOPIC_TYPES = {
    '/clock': ('rosgraph_msgs.msg', 'Clock'),
    '/scan': ('sensor_msgs.msg', 'LaserScan'),
    '/odom': ('nav_msgs.msg', 'Odometry'),
    '/sim/ground_truth/odom': ('nav_msgs.msg', 'Odometry'),
    '/odometry/filtered': ('nav_msgs.msg', 'Odometry'),
    '/sensors/imu/raw': ('sensor_msgs.msg', 'Imu'),
    '/camera/image_raw': ('sensor_msgs.msg', 'Image'),
}


def cmd_wait(a) -> int:
    rclpy = _ros()
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    import importlib
    rclpy.init()
    node = Node('car_hil_wait')
    want = {}
    stamps: dict[str, list[float]] = {}
    for spec in a.topics:
        topic, _, hz = spec.partition(':')
        if topic not in TOPIC_TYPES:
            log(f'unknown topic {topic} (known: {", ".join(TOPIC_TYPES)})')
            return 2
        want[topic] = float(hz) if hz else 0.0
        mod, cls = TOPIC_TYPES[topic]
        mtype = getattr(importlib.import_module(mod), cls)
        stamps[topic] = []
        node.create_subscription(mtype, topic, lambda _m, t=topic: stamps[t].append(time.monotonic()),
                                 qos_profile_sensor_data)
    t0 = time.monotonic()
    last_report = t0
    ok = False
    while time.monotonic() - t0 < a.timeout:
        rclpy.spin_once(node, timeout_sec=0.1)
        now = time.monotonic()
        missing = []
        for topic, hz in want.items():
            recent = [s for s in stamps[topic] if now - s <= 3.0]
            stamps[topic] = recent
            rate = len(recent) / 3.0
            if not recent or (hz and rate < hz):
                missing.append(f'{topic} ({rate:.1f} Hz' + (f' < {hz:g})' if hz else ')'))
        if not missing and now - t0 >= 3.0:
            ok = True
            break
        if now - last_report >= 15.0:
            log(f'{now - t0:.0f} s, waiting for: {", ".join(missing)}')
            last_report = now
    rates = {t: round(len(stamps[t]) / 3.0, 1) for t in want}
    log(f'{"OK" if ok else "TIMEOUT"} after {time.monotonic() - t0:.0f} s: {rates}')
    node.destroy_node()
    rclpy.shutdown()
    return 0 if ok else 1


def cmd_ready(a) -> int:
    rclpy = _ros()
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
    from f1tenth_messages.msg import MissionStatus
    from nav_msgs.msg import Odometry
    rclpy.init()
    node = Node('car_hil_ready')
    last: dict[str, float] = {}
    latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         reliability=ReliabilityPolicy.RELIABLE)
    node.create_subscription(MissionStatus, '/mission/status',
                             lambda m: last.__setitem__('mission', time.monotonic()), latched)
    for t in ('/odometry/filtered', '/sim/ground_truth/odom'):
        node.create_subscription(Odometry, t, lambda m, t=t: last.__setitem__(t, time.monotonic()),
                                 qos_profile_sensor_data)
    t0 = time.monotonic()
    others = [n for m, n in CONTROLLERS.items() if n != a.node]
    ok, last_report, why = False, t0, ''
    while time.monotonic() - t0 < a.timeout:
        rclpy.spin_once(node, timeout_sec=0.2)
        names = {n for n, _ in node.get_node_names_and_namespaces()}
        stray = [c for c in others if c in names]
        if stray:
            why = f'other controller(s) in the graph: {stray} (a leftover stack?)'
            break
        now = time.monotonic()
        missing = []
        if a.node not in names:
            missing.append(f'node {a.node}')
        if 'mission' not in last:
            missing.append('/mission/status')
        for t in ('/odometry/filtered', '/sim/ground_truth/odom'):
            if now - last.get(t, -1e9) > 2.0:
                missing.append(t)
        if not missing:
            ok = True
            break
        why = 'waiting for: ' + ', '.join(missing)
        if now - last_report >= 15.0:
            log(f'{now - t0:.0f} s, {why}')
            last_report = now
    log(f'{"READY" if ok else "NOT READY"} after {time.monotonic() - t0:.0f} s' + ('' if ok else f': {why}'))
    node.destroy_node()
    rclpy.shutdown()
    return 0 if ok else 1


class RunNode:
    """Everything one repetition needs: the reset, the mission services and the recorder."""

    def __init__(self, out: str, controller: str):
        rclpy = _ros()
        from rclpy.node import Node
        from rclpy.parameter import Parameter
        from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy,
                               qos_profile_sensor_data)
        from ackermann_msgs.msg import AckermannDriveStamped
        from f1tenth_messages.msg import DoorArray, MissionStatus, MoveOutcome
        from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
        from nav_msgs.msg import Odometry, Path
        from rosgraph_msgs.msg import Clock
        from std_msgs.msg import Bool, String
        self.rclpy = rclpy
        self.node = Node('car_hil_run', parameter_overrides=[Parameter('use_sim_time', value=True)])
        self.out = out
        self.controller = controller
        self.lock = threading.Lock()
        self.files, self.writers = {}, {}
        self.counts: dict[str, int] = {}
        self.last: dict[str, tuple[float, object]] = {}
        self.recording = False
        self.mission_trail: list[dict] = []
        self.state = None
        self.estop = False
        self.seen_running = False
        self.done_wall = None
        self.outcomes: list[dict] = []
        self.hold = None
        self.drive_window: list[float] = []
        self.gt_window: list[float] = []
        self.sampling = False
        self.throttle: dict[str, float] = {}
        self.clock0 = None
        os.makedirs(out, exist_ok=True)
        self._csv('gt', ['t_sim', 't_wall', 'x', 'y', 'yaw', 'v'])
        self._csv('odom', ['t_sim', 't_wall', 'x', 'y', 'yaw', 'v'])
        self._csv('drive', ['t_sim', 't_wall', 'speed', 'steer'])
        self._csv('mission', ['t_sim', 't_wall', 'state', 'estop'])
        self._csv('cpu', ['t_wall', 't_sim', 'cpu_s'])
        self.jsonl = {k: open(os.path.join(out, f'{k}.jsonl'), 'w')
                      for k in ('outcomes', 'doors', 'paths', 'planner_status')}
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        status_qos = QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                                reliability=ReliabilityPolicy.RELIABLE)
        best_effort = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                                 history=HistoryPolicy.KEEP_LAST, depth=20)
        n = self.node
        n.create_subscription(PoseStamped, '/sim/ground_truth/start_pose',
                              lambda m: self._keep('start', m), latched)
        n.create_subscription(Odometry, '/sim/ground_truth/odom', lambda m: self._odom('gt', m),
                              qos_profile_sensor_data)
        n.create_subscription(Odometry, '/odometry/filtered', lambda m: self._odom('odom', m),
                              qos_profile_sensor_data)
        n.create_subscription(Odometry, '/odom', lambda m: self._keep('wheel', m),
                              qos_profile_sensor_data)
        n.create_subscription(Odometry, '/ekf_global/odometry/filtered',
                              lambda m: self._keep('ekf_global', m), 10)
        n.create_subscription(PoseWithCovarianceStamped, '/slam/pose', lambda m: self._keep('slam', m), 10)
        n.create_subscription(MissionStatus, '/mission/status', self._mission, status_qos)
        n.create_subscription(MoveOutcome, '/mission/move_outcome', self._outcome, 10)
        n.create_subscription(Bool, '/mpc/hold', lambda m: setattr(self, 'hold', m.data), 10)
        n.create_subscription(AckermannDriveStamped, '/drive', self._drive, best_effort)
        n.create_subscription(DoorArray, '/perception/doors_semantic', self._doors, 10)
        n.create_subscription(Path, '/planner/path', self._path, best_effort)
        n.create_subscription(String, '/corridor', self._corridor, best_effort)
        n.create_subscription(String, '/planner/status', self._pstatus, best_effort)
        n.create_subscription(Clock, '/clock', self._clock, qos_profile_sensor_data)

    # ---- output
    def _csv(self, name, header):
        f = open(os.path.join(self.out, f'{name}.csv'), 'w', newline='')
        w = csv.writer(f)
        w.writerow(header)
        self.files[name], self.writers[name] = f, w

    def _row(self, name, row):
        self.counts[name] = self.counts.get(name, 0) + 1
        if self.recording:
            with self.lock:
                self.writers[name].writerow(row)

    def _json(self, name, obj, throttle_key=None):
        if not self.recording:
            return
        if throttle_key is not None:
            now = time.monotonic()
            if now - self.throttle.get(throttle_key, 0.0) < PATHS_MIN_PERIOD_S:
                return
            self.throttle[throttle_key] = now
        self.counts[name] = self.counts.get(name, 0) + 1
        with self.lock:
            self.jsonl[name].write(json.dumps(obj) + '\n')

    def t(self):
        return self.node.get_clock().now().nanoseconds * 1e-9, time.time()

    def close(self):
        with self.lock:
            for f in list(self.files.values()) + list(self.jsonl.values()):
                f.close()

    # ---- callbacks
    def _keep(self, key, m):
        self.last[key] = (time.monotonic(), m)

    def _clock(self, m):
        s = m.clock.sec + m.clock.nanosec * 1e-9
        self.last['clock'] = (time.monotonic(), s)

    def _odom(self, name, m):
        self.last[name] = (time.monotonic(), m)
        p, tw = m.pose.pose, m.twist.twist
        v = math.hypot(tw.linear.x, tw.linear.y)
        if name == 'gt' and self.sampling:
            self.gt_window.append(v)
        ts, twall = self.t()
        self._row(name, [f'{ts:.4f}', f'{twall:.4f}', f'{p.position.x:.4f}', f'{p.position.y:.4f}',
                         f'{yaw_of(p.orientation):.5f}', f'{v:.4f}'])

    def _drive(self, m):
        if self.sampling:
            self.drive_window.append(m.drive.speed)
        ts, tw = self.t()
        self._row('drive', [f'{ts:.4f}', f'{tw:.4f}', f'{m.drive.speed:.4f}',
                            f'{m.drive.steering_angle:.5f}'])

    def _mission(self, m):
        ts, tw = self.t()
        self.last['mission'] = (time.monotonic(), m)
        self.estop = bool(m.emergency_stop_active)
        if m.state != self.state:
            entry = {'t_sim': round(ts, 4), 't_wall': round(tw, 4), 'state': m.state,
                     'estop': self.estop}
            if self.recording:
                self.mission_trail.append(entry)
            self._row('mission', [f'{ts:.4f}', f'{tw:.4f}', m.state, int(self.estop)])
        self.state = m.state
        if self.recording:
            if m.state == 'RUNNING':
                self.seen_running = True
            elif self.seen_running and m.state not in ACTIVE_STATES and self.done_wall is None:
                self.done_wall = time.monotonic()

    def _outcome(self, m):
        ts, tw = self.t()
        d = {'t_sim': round(ts, 4), 't_wall': round(tw, 4), 'mission_id': m.mission_id,
             'move_id': m.move_id, 'move_type': m.move_type, 'stop_reason': m.stop_reason,
             'outcome': m.outcome, 'duration_s': _num(m.duration_s), 'commanded': _num(m.commanded),
             'actual': _num(m.actual), 'score_percent': _num(m.score_percent),
             'mismatch_flagged': bool(m.mismatch_flagged), 'note': m.note}
        if self.recording:
            self.outcomes.append(d)
        self._json('outcomes', d)

    def _doors(self, m):
        ts, tw = self.t()
        doors = [{'id': int(d.id), 'state': int(d.state), 'strong': bool(getattr(d, 'strong', True)),
                  'x': round(d.center.x, 3), 'y': round(d.center.y, 3), 'width': round(d.width, 3),
                  'nx': round(d.normal.x, 3), 'ny': round(d.normal.y, 3)} for d in m.doors]
        self._json('doors', {'t_sim': round(ts, 3), 't_wall': round(tw, 3),
                             'frame': m.header.frame_id, 'doors': doors}, throttle_key='doors')

    def _path(self, m):
        ts, tw = self.t()
        self._json('paths', {'src': 'planner_path', 't_sim': round(ts, 3), 'frame': m.header.frame_id,
                             'xy': [[round(p.pose.position.x, 3), round(p.pose.position.y, 3)]
                                    for p in m.poses]}, throttle_key='planner_path')

    def _corridor(self, m):
        try:
            payload = json.loads(m.data)
        except ValueError:
            return
        ts, _ = self.t()
        self._json('paths', {'src': 'corridor', 't_sim': round(ts, 3),
                             'frame': payload.get('frame_id', 'odom'),
                             'xy': payload.get('polygon', [])}, throttle_key='corridor')

    def _pstatus(self, m):
        ts, tw = self.t()
        try:
            d = json.loads(m.data)
        except ValueError:
            d = {'raw': m.data}
        if isinstance(d, dict):
            d.update({'t_sim': round(ts, 3), 't_wall': round(tw, 3)})
        self._json('planner_status', d)

    # ---- helpers
    def spin_for(self, sec, until=None):
        end = time.monotonic() + sec
        while time.monotonic() < end:
            self.rclpy.spin_once(self.node, timeout_sec=0.05)
            if until is not None and until():
                return True
        return until() if until is not None else True

    def call(self, srv_type, name, request, timeout):
        cli = self.node.create_client(srv_type, name)
        if not cli.wait_for_service(timeout_sec=10.0):
            return None, f'service {name} not available within 10 s'
        fut = cli.call_async(request)
        end = time.monotonic() + timeout
        while not fut.done() and time.monotonic() < end:
            self.rclpy.spin_once(self.node, timeout_sec=0.05)
        if not fut.done():
            return None, f'no answer from {name} within {timeout:.0f} s'
        return fut.result(), ''

    def fresh(self, key, max_age=2.0):
        v = self.last.get(key)
        return v[1] if v is not None and time.monotonic() - v[0] <= max_age else None


def _num(v):
    v = float(v)
    return None if math.isnan(v) or math.isinf(v) else round(v, 4)


def _near_zero(rn, key, after, tol=REZERO_TOL_M):
    """The last `key` message arrived after `after` (monotonic) and sits within tol of 0 0 0."""
    v = rn.last.get(key)
    if v is None or v[0] < after:
        return False
    p = v[1].pose.pose
    return math.hypot(p.position.x, p.position.y) <= tol and abs(wrap(yaw_of(p.orientation))) <= LOC_YAW_TOL


def rezero_local(rn, trigger_type, setpose_type) -> dict:
    """Isaac's /sim/reset, then /set_pose 0 on the local EKF: steps 2 and 3 of /reset_all, again.

    Isaac drifts: with every wheel commanded to 0 the car creeps backwards by itself, ~3.7 mm/s
    of true motion and ~7.3 mm/s on /odom (2026-11-23, three poses, Isaac alone). /reset_all zeroes
    the odometry ~11 s before the checks (its step 2, then the SLAM restart, then --settle), so
    /odometry/filtered was already at ~8 cm against LOCAL_TOL 5 cm and every reset failed. Doing it
    again here, right before the 1 s sampling window, leaves ~1 s of drift and puts the car back at
    its start pose the moment the mission starts. The drift itself is a sim bug, not fixed here.

    /sim/reset alone is not enough: the local EKF fuses /odom differentially, so the jump back to 0
    reaches /odometry/filtered as a velocity spike, not as a reset. Hence the /set_pose, after /odom
    is back at 0 (same order as reset_manager). Both Isaac and fake_sim offer /sim/reset.
    Returns the record; 'error' set when a step failed.
    """
    rec = {}
    t0 = time.monotonic()
    res, err = rn.call(trigger_type, '/sim/reset', trigger_type.Request(), 30.0)
    if res is None or not res.success:
        rec['error'] = f'/sim/reset: {err or res.message}'
        return rec
    if not rn.spin_for(REZERO_WAIT_S, until=lambda: _near_zero(rn, 'wheel', t0)):
        rec['error'] = f'/odom not back within {REZERO_TOL_M} m of 0 in {REZERO_WAIT_S:.0f} s after /sim/reset'
        return rec
    rec['sim_reset_s'] = round(time.monotonic() - t0, 2)
    req = setpose_type.Request()
    req.pose.header.frame_id = 'odom'
    req.pose.header.stamp = rn.node.get_clock().now().to_msg()
    req.pose.pose.pose.orientation.w = 1.0
    cov = [0.0] * 36
    cov[0] = cov[7] = cov[14] = cov[21] = cov[28] = cov[35] = 1.0e-4   # reset_manager's covariance
    req.pose.pose.covariance = cov
    t1 = time.monotonic()
    res, err = rn.call(setpose_type, '/set_pose', req, 10.0)
    if res is None:
        rec['error'] = f'/set_pose: {err}'
        return rec
    if not rn.spin_for(REZERO_WAIT_S, until=lambda: _near_zero(rn, 'odom', t1)):
        rec['error'] = (f'/odometry/filtered not back within {REZERO_TOL_M} m of 0 in '
                        f'{REZERO_WAIT_S:.0f} s after /set_pose')
        return rec
    rec['total_s'] = round(time.monotonic() - t0, 2)
    return rec


def do_reset(rn: RunNode, settle: float) -> dict:
    """/reset_all, settle, re-zero the local odometry (rezero_local), then restart.sh's checks
    (HIL office tolerances). Returns the record."""
    from robot_localization.srv import SetPose
    from std_srvs.srv import Trigger
    rec = {'checks': {}, 'ok': False}
    rn.spin_for(1.0)
    t0 = time.monotonic()
    res, err = rn.call(Trigger, '/reset_all', Trigger.Request(), 150.0)
    rec['service_s'] = round(time.monotonic() - t0, 1)
    if res is None or not res.success:
        rec['error'] = err or res.message
        return rec
    rec['message'] = res.message
    rn.spin_for(settle)
    rec['rezero'] = rezero_local(rn, Trigger, SetPose)
    if 'error' in rec['rezero']:
        rec['error'] = rec['rezero']['error']
        return rec
    rn.sampling = True
    rn.drive_window.clear()
    rn.gt_window.clear()
    rn.spin_for(1.0)
    rn.sampling = False
    checks = rec['checks']
    st, gt = rn.last.get('start'), rn.fresh('gt')
    if st is None or gt is None:
        checks['pose'] = {'ok': False, 'detail': 'no latched /sim/ground_truth/start_pose' if st is None
                          else 'no /sim/ground_truth/odom in 2 s'}
    else:
        sp, gp = st[1].pose, gt.pose.pose
        d = math.hypot(gp.position.x - sp.position.x, gp.position.y - sp.position.y)
        dyaw = math.degrees(abs(wrap(yaw_of(gp.orientation) - yaw_of(sp.orientation))))
        checks['pose'] = {'ok': d <= RESET_POS_TOL_M and dyaw <= RESET_YAW_TOL_DEG,
                          'dist_m': round(d, 4), 'dyaw_deg': round(dyaw, 2),
                          'start': [round(sp.position.x, 3), round(sp.position.y, 3),
                                    round(math.degrees(yaw_of(sp.orientation)), 2)]}
    vmax = max(rn.gt_window) if rn.gt_window else None
    checks['speed'] = {'ok': vmax is not None and vmax <= SPEED_TOL, 'gt_vmax': vmax}
    m = rn.fresh('mission', 1e9)
    checks['mission'] = {'ok': m is not None and m.state not in ACTIVE_STATES
                         and not m.emergency_stop_active,
                         'state': None if m is None else m.state,
                         'estop': None if m is None else bool(m.emergency_stop_active)}
    dmax = max((abs(v) for v in rn.drive_window), default=None)
    checks['drive_zero'] = {'ok': dmax is not None and dmax == 0.0, 'max_speed': dmax,
                            'samples': len(rn.drive_window), 'hold': rn.hold}
    parts, ok = {}, True
    for key, tol in (('odom', LOCAL_TOL), ('ekf_global', GLOBAL_TOL), ('slam', GLOBAL_TOL)):
        v = rn.last.get(key)
        if v is None or (key != 'slam' and time.monotonic() - v[0] > 5.0) or (key == 'slam' and v[0] < t0):
            parts[key] = 'none'
            ok = False
            continue
        p = v[1].pose.pose
        dd, aa = math.hypot(p.position.x, p.position.y), abs(wrap(yaw_of(p.orientation)))
        parts[key] = [round(p.position.x, 3), round(p.position.y, 3), round(aa, 3)]
        ok = ok and dd <= tol and aa <= LOC_YAW_TOL
    checks['localization'] = {'ok': ok, **parts}
    rec['ok'] = all(c['ok'] for c in checks.values())
    return rec


def session_pose(session: str, idx: int) -> dict:
    """Pose idx of the session (session.json 'poses', written by the workstation)."""
    with open(os.path.join(session, 'session.json')) as f:
        poses = json.load(f)['poses']
    for p in poses:
        if int(p['idx']) == int(idx):
            return p
    raise SystemExit(f'pose {idx} is not in {session}/session.json')


def cmd_run(a) -> int:
    rclpy = _ros()
    from f1tenth_messages.srv import LoadMission
    from std_srvs.srv import Trigger
    pose = session_pose(a.session, a.pose_idx)
    node_name = CONTROLLERS[a.mode]
    out = os.path.join(a.session, 'runs', a.run_id)
    if os.path.exists(os.path.join(out, 'result.json')):
        log(f'{a.run_id}: result.json exists, nothing to do')
        return 0
    os.makedirs(out, exist_ok=True)
    rclpy.init()
    rn = RunNode(out, node_name)
    result = {'run_id': a.run_id, 'mode': a.mode, 'rep': a.rep, 'controller': node_name, 'pose': pose,
              'mission_file': a.mission, 'host': socket.gethostname(),
              'started_wall': time.strftime('%Y-%m-%dT%H:%M:%S%z'), 'status': 'started'}

    def finish(status, rc):
        result['status'] = status
        result['ended_wall'] = time.strftime('%Y-%m-%dT%H:%M:%S%z')
        result['final_state'] = rn.state
        result['estop'] = rn.estop
        result['mission_trail'] = rn.mission_trail
        result['outcomes'] = rn.outcomes
        result['counts'] = rn.counts
        rn.close()
        tmp = os.path.join(out, 'result.json.tmp')
        with open(tmp, 'w') as f:
            json.dump(result, f, indent=1)
        os.replace(tmp, os.path.join(out, 'result.json'))
        log(f'{a.run_id}: {status}, mission {rn.state}, '
            + '; '.join(f"{o['move_id']}: {o['stop_reason']}{'/' + o['outcome'] if o['outcome'] else ''}"
                        for o in rn.outcomes))
        rn.node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        return rc

    # 1 reset (one retry: the first /reset_all after a fresh stack sometimes lands SLAM late)
    resets = []
    for attempt in (1, 2):
        rec = do_reset(rn, a.settle)
        rec['attempt'] = attempt
        resets.append(rec)
        log(f'{a.run_id}: reset {attempt}: {"OK" if rec["ok"] else "FAIL"} '
            + json.dumps({k: v.get('ok') for k, v in rec.get('checks', {}).items()} or rec.get('error')))
        if rec['ok']:
            break
        rn.spin_for(5.0)
    result['reset'] = resets
    if not resets[-1]['ok']:
        return finish('reset_failed', 3)
    clk = rn.last.get('clock')
    # 2 load
    rn.recording = True
    t_rec = time.monotonic()
    res, err = rn.call(LoadMission, '/mission/load_mission', LoadMission.Request(path=a.mission), 30.0)
    result['load'] = {'ok': bool(res and res.success), 'message': err or res.message}
    if not result['load']['ok']:
        return finish('load_failed', 4)
    rn.spin_for(0.5)
    # 3 start (+ the controller's CPU from /proc, the same measurement for every mode)
    pids = find_pids(node_name)
    cpu = {'pid': pids[0] if len(pids) == 1 else None, 'pids': pids}
    stop = threading.Event()

    def sampler():
        if cpu['pid'] is None:
            return
        while not stop.is_set():
            try:
                c = proc_cpu_s(cpu['pid'])
            except OSError:
                cpu['lost'] = cpu.get('lost', 0) + 1
                stop.wait(0.2)
                continue
            ts, tw = rn.t()
            rn._row('cpu', [f'{tw:.4f}', f'{ts:.4f}', f'{c:.3f}'])
            stop.wait(0.2)
    th = threading.Thread(target=sampler, daemon=True)
    th.start()
    ts0, tw0 = rn.t()
    res, err = rn.call(Trigger, '/mission/start_mission', Trigger.Request(), 30.0)
    result['start'] = {'ok': bool(res and res.success), 'message': err or res.message,
                       't_sim': round(ts0, 4), 't_wall': round(tw0, 4)}
    result['cpu'] = cpu
    if not result['start']['ok']:
        stop.set()
        return finish('start_failed', 5)
    # 4 wait for the end
    status = 'ended'
    while True:
        rn.spin_for(0.2)
        if rn.done_wall is not None and time.monotonic() - rn.done_wall >= 3.0:
            break
        if time.monotonic() - t_rec > a.timeout:
            status = 'timeout'
            log(f'{a.run_id}: still {rn.state} after {a.timeout:.0f} s: /mission/abort_mission')
            rn.call(Trigger, '/mission/abort_mission', Trigger.Request(), 10.0)
            rn.spin_for(3.0)
            break
        if not rn.seen_running and time.monotonic() - t_rec > 20.0 and rn.state != 'RUNNING':
            status = 'never_running'
            break
    stop.set()
    th.join(timeout=2.0)
    ts1, tw1 = rn.t()
    clk1 = rn.last.get('clock')
    if clk and clk1 and clk1[0] > clk[0]:
        result['rtf'] = round((clk1[1] - clk[1]) / (clk1[0] - clk[0]), 3)
    result['end'] = {'t_sim': round(ts1, 4), 't_wall': round(tw1, 4)}
    return finish(status, 0)


def cmd_mark(a) -> int:
    """A run that could not even start (stack not ready, Isaac down): a result.json saying so, so
    the analysis counts it as a failure of that mode instead of losing it."""
    out = os.path.join(a.session, 'runs', a.run_id)
    if os.path.exists(os.path.join(out, 'result.json')):
        log(f'{a.run_id}: result.json exists, not marking it {a.status}')
        return 0
    os.makedirs(out, exist_ok=True)
    result = {'run_id': a.run_id, 'mode': a.mode, 'rep': a.rep, 'controller': CONTROLLERS[a.mode],
              'pose': session_pose(a.session, a.pose_idx), 'status': a.status, 'note': a.note,
              'started_wall': time.strftime('%Y-%m-%dT%H:%M:%S%z'), 'final_state': None,
              'outcomes': []}
    with open(os.path.join(out, 'result.json'), 'w') as f:
        json.dump(result, f, indent=1)
    log(f'{a.run_id}: marked {a.status} ({a.note})')
    return 0


def cmd_versions(a) -> int:
    import re
    root = os.environ.get('F1T_WS_INSTALL', '/opt/f1tenth_ws/install')
    out = {}
    for pkg in PACKAGES:
        p = os.path.join(root, pkg, 'share', pkg, 'package.xml')
        if not os.path.exists(p):
            p = os.path.join(root, 'share', pkg, 'package.xml')
        try:
            m = re.search(r'<version>([^<]+)</version>', open(p).read())
            out[pkg] = m.group(1) if m else '?'
        except OSError:
            out[pkg] = None
    src = {}
    try:
        for ln in open('/opt/f1tenth_ws/SOURCES'):
            k, _, v = ln.strip().partition(' ')
            if k in ('commit', 'branch', 'describe', 'subject'):
                src[k] = v
    except OSError:
        pass
    with open(a.out, 'w') as f:
        json.dump({'modules': out, 'sources': src}, f, indent=1)
    log(f'versions -> {a.out}: {out}')
    return 0


# ============================================================================ analysis (no ROS)
def _read_csv(path):
    try:
        with open(path, newline='') as f:
            return list(csv.DictReader(f))
    except OSError:
        return []


def door_frame(pose: dict):
    """(door_xy, n, t, width): n = unit passage direction INTO the door from the start's side,
    t = n rotated +90 deg. passage_axis 'y': the opening lies along x and is crossed along y."""
    door = pose['door']
    dx, dy = float(door['x']), float(door['y'])
    sx, sy = float(pose['start']['x']), float(pose['start']['y'])
    if door.get('passage_axis', 'y') == 'y':
        n = (0.0, 1.0 if dy > sy else -1.0)
    else:
        n = (1.0 if dx > sx else -1.0, 0.0)
    t = (-n[1], n[0])
    return (dx, dy), n, t, float(door.get('width_m', 0.0))


def judge_run(result: dict, gt: list[dict], cpu: list[dict]) -> dict:
    """The ground-truth verdict of one run plus the mission's own. Pure: tested offline."""
    pose = result['pose']
    (dx, dy), n, tv, w = door_frame(pose)
    r = {'run_id': result['run_id'], 'mode': result['mode'], 'pose_idx': pose.get('idx'),
         'rep': result.get('rep'), 'status': result.get('status'), 'note': result.get('note', ''),
         'final_state': result.get('final_state'), 'door_width_m': w}
    outs = {o['move_id']: o for o in result.get('outcomes', [])}
    for key, mid in (('move0', 'move_0_straight_5m'), ('move1', 'move_1_enter_left_door')):
        o = outs.get(mid) or {}
        r[f'{key}_stop_reason'] = o.get('stop_reason', '')
        r[f'{key}_outcome'] = o.get('outcome', '')
        r[f'{key}_duration_s'] = o.get('duration_s')
        r[f'{key}_actual'] = o.get('actual')
    r['mission_complete'] = result.get('final_state') == 'COMPLETE'
    start = result.get('start', {}) or {}
    t0 = start.get('t_sim')
    t1 = (result.get('end') or {}).get('t_sim')
    pts = [(float(g['t_sim']), float(g['x']), float(g['y']), float(g['yaw']), float(g['v']))
           for g in gt if g.get('x') not in (None, '')]
    if t0 is not None:
        pts = [p for p in pts if p[0] >= t0 - 0.05 and (t1 is None or p[0] <= t1 + 0.05)]
    r['gt_samples'] = len(pts)
    r['rtf'] = result.get('rtf')
    if len(pts) < 2:
        r.update({'crossed_door': False, 'gt_in_door': False, 'success': False,
                  'failure': failure_label(r, False)})
        return r
    length = sum(math.hypot(b[1] - a[1], b[2] - a[2]) for a, b in zip(pts, pts[1:]))
    r['path_length_m'] = round(length, 3)
    r['duration_s'] = round(pts[-1][0] - pts[0][0], 2)
    sx, sy = float(pose['start']['x']), float(pose['start']['y'])
    syaw = math.radians(float(pose['start']['yaw_deg']))
    # the run-up: how far along the start heading the car got (before turning off it)
    prog = [((p[1] - sx) * math.cos(syaw) + (p[2] - sy) * math.sin(syaw)) for p in pts]
    r['max_progress_along_start_m'] = round(max(prog), 3)
    along = [((p[1] - dx) * n[0] + (p[2] - dy) * n[1]) for p in pts]
    lat = [((p[1] - dx) * tv[0] + (p[2] - dy) * tv[1]) for p in pts]
    half_free = max(0.0, w / 2.0 - CAR_HALF_WIDTH_M)
    crossed = False
    for i in range(1, len(pts)):
        if (along[i - 1] < 0.0) != (along[i] < 0.0):
            f = along[i - 1] / (along[i - 1] - along[i]) if along[i - 1] != along[i] else 0.0
            lc = lat[i - 1] + f * (lat[i] - lat[i - 1])
            if abs(lc) <= w / 2.0:
                crossed = True
                r['cross_lateral_m'] = round(lc, 3)
                break
    r['crossed_door'] = crossed
    r['min_door_dist_m'] = round(min(math.hypot(a_, l_) for a_, l_ in zip(along, lat)), 3)
    r['max_along_m'] = round(max(along), 3)
    fa, fl, fyaw = along[-1], lat[-1], pts[-1][3]
    yerr = math.degrees(abs(wrap(fyaw - math.atan2(n[1], n[0]))))
    r.update({'final_along_m': round(fa, 3), 'final_lateral_m': round(fl, 3),
              'final_yaw_err_deg': round(yerr, 1),
              'final_x': round(pts[-1][1], 3), 'final_y': round(pts[-1][2], 3)})
    in_door = (abs(fa) <= DOORWAY_ALONG_TOL_M and abs(fl) <= max(half_free, 0.05)
               and yerr <= DOORWAY_YAW_TOL_DEG)
    r['gt_in_door'] = in_door
    # stuck: commanded nothing to do with it -- the car stands still for > 3 s while RUNNING
    still = 0.0
    longest = 0.0
    for a_, b_ in zip(pts, pts[1:]):
        if b_[4] < 0.02:
            still += b_[0] - a_[0]
            longest = max(longest, still)
        else:
            still = 0.0
    r['longest_standstill_s'] = round(longest, 1)
    if cpu and len(cpu) >= 2:
        c0, c1 = cpu[0], cpu[-1]
        dsim = float(c1['t_sim']) - float(c0['t_sim'])
        if dsim > 0:
            r['ctrl_cpu_ms_per_sim_s'] = round(1000.0 * (float(c1['cpu_s']) - float(c0['cpu_s'])) / dsim, 1)
    r['success'] = bool(r['mission_complete'] and (in_door or crossed))
    r['failure'] = failure_label(r, crossed)
    return r


def failure_label(r: dict, crossed: bool) -> str:
    """One word per run for the tables: what stopped it first."""
    st = r.get('status')
    if st in ('reset_failed', 'load_failed', 'start_failed', 'never_running', 'stack_not_ready',
              'isaac_failed', 'skipped'):
        return st
    if r.get('mission_complete'):
        if r.get('gt_in_door') or crossed:
            return 'ok'
        return 'complete_not_in_door'
    m0 = r.get('move0_stop_reason') or ''
    m1 = (r.get('move1_outcome') or '') or (r.get('move1_stop_reason') or '')
    if st == 'timeout':
        return 'run_timeout'
    if m0 and not m0.startswith('stop_condition'):
        return f'straight:{m0}'
    if m1:
        return f'door:{m1}'
    return f'mission:{r.get("final_state")}'


def _wilson(k, n, z=1.96):
    if n == 0:
        return (0.0, 0.0)
    p = k / n
    d = 1 + z * z / n
    c = p + z * z / (2 * n)
    h = z * math.sqrt(p * (1 - p) / n + z * z / (4 * n * n))
    return ((c - h) / d, (c + h) / d)


def _median(xs):
    xs = sorted(x for x in xs if x is not None)
    if not xs:
        return None
    m = len(xs) // 2
    return xs[m] if len(xs) % 2 else 0.5 * (xs[m - 1] + xs[m])


def cmd_analyze(a) -> int:
    sess = a.session
    runs_dir = os.path.join(sess, 'runs')
    out = os.path.join(sess, 'analysis')
    os.makedirs(os.path.join(out, 'figures'), exist_ok=True)
    try:
        session = json.load(open(os.path.join(sess, 'session.json')))
    except (OSError, ValueError):
        session = {}
    rows = []
    for rid in sorted(os.listdir(runs_dir)) if os.path.isdir(runs_dir) else []:
        d = os.path.join(runs_dir, rid)
        try:
            result = json.load(open(os.path.join(d, 'result.json')))
        except (OSError, ValueError):
            continue
        rows.append(judge_run(result, _read_csv(os.path.join(d, 'gt.csv')),
                              _read_csv(os.path.join(d, 'cpu.csv'))))
    if not rows:
        log(f'no runs in {runs_dir}')
        return 1
    cols = ['run_id', 'pose_idx', 'mode', 'rep', 'status', 'final_state', 'success', 'failure',
            'mission_complete', 'gt_in_door', 'crossed_door', 'door_width_m',
            'move0_stop_reason', 'move0_duration_s', 'move0_actual', 'move1_stop_reason',
            'move1_outcome', 'move1_duration_s', 'duration_s', 'path_length_m',
            'max_progress_along_start_m', 'min_door_dist_m', 'max_along_m', 'final_along_m',
            'final_lateral_m', 'final_yaw_err_deg', 'cross_lateral_m', 'longest_standstill_s',
            'ctrl_cpu_ms_per_sim_s', 'rtf', 'final_x', 'final_y', 'gt_samples']
    with open(os.path.join(out, 'runs.csv'), 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=cols, extrasaction='ignore')
        w.writeheader()
        for r in rows:
            w.writerow(r)
    modes = [m for m in session.get('modes', CONTROLLERS) if any(r['mode'] == m for r in rows)]
    summary = []
    for m in modes:
        rr = [r for r in rows if r['mode'] == m]
        k = sum(r['success'] for r in rr)
        lo, hi = _wilson(k, len(rr))
        fails: dict[str, int] = {}
        for r in rr:
            if not r['success']:
                fails[r['failure']] = fails.get(r['failure'], 0) + 1
        summary.append({
            'mode': m, 'runs': len(rr), 'success': k, 'success_pct': round(100.0 * k / len(rr), 1),
            'ci95_pct': f'{100 * lo:.0f}-{100 * hi:.0f}',
            'mission_complete': sum(r['mission_complete'] for r in rr),
            'gt_in_door': sum(bool(r.get('gt_in_door')) for r in rr),
            'crossed_door': sum(bool(r.get('crossed_door')) for r in rr),
            'straight_ok': sum((r.get('move0_stop_reason') or '').startswith('stop_condition') for r in rr),
            'median_duration_s': _median([r.get('duration_s') for r in rr if r['success']]),
            'median_path_m': _median([r.get('path_length_m') for r in rr if r['success']]),
            'median_cpu_ms_per_sim_s': _median([r.get('ctrl_cpu_ms_per_sim_s') for r in rr]),
            'failures': '; '.join(f'{k_} {v}' for k_, v in sorted(fails.items(), key=lambda kv: -kv[1])),
        })
    with open(os.path.join(out, 'summary_by_mode.csv'), 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=list(summary[0]))
        w.writeheader()
        w.writerows(summary)
    poses = sorted({r['pose_idx'] for r in rows if r['pose_idx'] is not None})
    bypose = []
    for p in poses:
        row = {'pose_idx': p}
        pr = [r for r in rows if r['pose_idx'] == p]
        row['door_width_m'] = pr[0]['door_width_m']
        for m in modes:
            mr = [r for r in pr if r['mode'] == m]
            row[m] = f"{sum(r['success'] for r in mr)}/{len(mr)}"
        bypose.append(row)
    with open(os.path.join(out, 'by_pose.csv'), 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=['pose_idx', 'door_width_m', *modes])
        w.writeheader()
        w.writerows(bypose)
    figs = make_figures(sess, out, rows, session)
    write_report(sess, out, rows, summary, bypose, modes, session, figs)
    log(f'{len(rows)} runs -> {out} (report.md, runs.csv, summary_by_mode.csv, by_pose.csv, '
        f'{len(figs)} figures)')
    for s in summary:
        log(f"  {s['mode']:<14} {s['success']}/{s['runs']} ({s['success_pct']} %, CI {s['ci95_pct']} %)  "
            f"{s['failures']}")
    return 0


def make_figures(sess, out, rows, session) -> list[str]:
    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
    except ImportError:
        log('no matplotlib: no figures')
        return []
    figs = []
    geo = None
    try:
        geo = json.load(open(os.path.join(sess, 'map_geo.json')))
    except (OSError, ValueError):
        pass
    colors = {'corridor_mpc': '#1f77b4', 'corridor_rrt': '#d62728', 'p2p_geometric': '#2ca02c'}
    # success by mode
    modes = [m for m in session.get('modes', CONTROLLERS) if any(r['mode'] == m for r in rows)]
    fig, ax = plt.subplots(figsize=(6, 3.2))
    for i, m in enumerate(modes):
        rr = [r for r in rows if r['mode'] == m]
        k = sum(r['success'] for r in rr)
        lo, hi = _wilson(k, len(rr))
        p = k / len(rr)
        ax.bar(i, 100 * p, color=colors.get(m, 'grey'))
        ax.errorbar(i, 100 * p, yerr=[[100 * (p - lo)], [100 * (hi - p)]], color='k', capsize=4)
        ax.text(i, 100 * p + 2, f'{k}/{len(rr)}', ha='center', fontsize=9)
    ax.set_xticks(range(len(modes)), modes)
    ax.set_ylabel('success [%] (Wilson 95 %)')
    ax.set_ylim(0, 110)
    fig.tight_layout()
    fig.savefig(os.path.join(out, 'figures', 'success_by_mode.png'), dpi=110)
    plt.close(fig)
    figs.append('figures/success_by_mode.png')
    # one figure per pose: the nine trajectories on the floor plan, the true door
    poses = {}
    for r in rows:
        poses.setdefault(r['pose_idx'], []).append(r)
    for pidx, rr in sorted(poses.items(), key=lambda kv: (kv[0] is None, kv[0])):
        try:
            res0 = json.load(open(os.path.join(sess, 'runs', rr[0]['run_id'], 'result.json')))
        except (OSError, ValueError):
            continue
        pose = res0['pose']
        (dx, dy), n, tv, w = door_frame(pose)
        sx, sy = float(pose['start']['x']), float(pose['start']['y'])
        fig, ax = plt.subplots(figsize=(5.5, 5.5))
        if geo:
            for key, col in (('floor', '#eeeeee'), ('wall', '#444444')):
                for x, y, ww, hh in geo.get(key, []):
                    if abs(x - sx) < 12 and abs(y - sy) < 12:
                        ax.add_patch(plt.Rectangle((x, y), ww, hh, color=col, lw=0))
        ax.plot([dx - tv[0] * w / 2, dx + tv[0] * w / 2], [dy - tv[1] * w / 2, dy + tv[1] * w / 2],
                color='orange', lw=4, label=f'door {w:.2f} m')
        for r in rr:
            gt = _read_csv(os.path.join(sess, 'runs', r['run_id'], 'gt.csv'))
            if not gt:
                continue
            xs = [float(g['x']) for g in gt]
            ys = [float(g['y']) for g in gt]
            ax.plot(xs, ys, color=colors.get(r['mode'], 'grey'), lw=1.2,
                    ls='-' if r['success'] else '--', alpha=0.9)
            ax.plot(xs[-1], ys[-1], 'o' if r['success'] else 'x', color=colors.get(r['mode'], 'grey'))
        ax.plot(sx, sy, 'k^', ms=8)
        for m, c in colors.items():
            if any(r['mode'] == m for r in rr):
                ax.plot([], [], color=c, label=f"{m} {sum(r['success'] for r in rr if r['mode'] == m)}"
                        f"/{sum(r['mode'] == m for r in rr)}")
        ax.set_aspect('equal')
        span = 7.0
        cx, cy = (sx + dx) / 2, (sy + dy) / 2
        ax.set_xlim(cx - span, cx + span)
        ax.set_ylim(cy - span, cy + span)
        ax.set_title(f'pose {pidx}: start ({sx:.2f}, {sy:.2f}) yaw {pose["start"]["yaw_deg"]:.0f}')
        ax.legend(fontsize=7, loc='best')
        fig.tight_layout()
        name = f'figures/pose_{int(pidx):02d}.png' if pidx is not None else 'figures/pose_unknown.png'
        fig.savefig(os.path.join(out, name), dpi=90)
        plt.close(fig)
        figs.append(name)
    return figs


def write_report(sess, out, rows, summary, bypose, modes, session, figs):
    try:
        ver = json.load(open(os.path.join(sess, 'versions.json')))
    except (OSError, ValueError):
        ver = {'modules': {}, 'sources': {}}
    sid = os.path.basename(os.path.normpath(sess))
    mods = ', '.join(f'{k}: "{v}"' for k, v in ver.get('modules', {}).items() if v)
    n = len(rows)
    tot = sum(r['success'] for r in rows)
    res_line = '; '.join(f"{s['mode']} {s['success']}/{s['runs']}" for s in summary)
    date = sid[:8]
    date = f'{date[:4]}-{date[4:6]}-{date[6:8]}' if date.isdigit() else time.strftime('%Y-%m-%d')
    lines = [
        '---',
        f'id: {date}_office-door-batch',
        'title: "Office HIL: 35 pose x 3 planner x 3 ripetizioni, rettilineo 5 m e porta a sinistra"',
        'area: simulation',
        f'date: {date}',
        'tags: [isaac, hil, orin, office-world, enter-door, planner-modes, batch]',
        f'modules: {{{mods}}}',
        f'branch: "{session.get("git_branch", "n/d")}"',
        f'commit: "{session.get("git_commit", "n/d")}"',
        'status: open',
        'hypothesis: "Su tutte le pose fattibili dell\'office (find_door_runup.py), i tre planner_controller_mode '
        'eseguono rettilineo 5 m + enter_door left con lo stesso successo?"',
        f'result: "{tot}/{n} run riusciti; {res_line} (successo = missione COMPLETE e auto nel varco vero, ground truth)"',
        'decision: "n/d"',
        f'raw_data: "Orin: ~/f1tenth_hil_batch/{sid}/runs/ (CSV per run); sessione {sid}"',
        '---',
        f'# Office HIL batch {sid}',
        '',
        '## Setup',
        '',
        f"- Stack: immagine `{ver.get('sources', {}).get('branch', '?')}` "
        f"(commit {ver.get('sources', {}).get('commit', '?')[:12]}), container HIL sull'Orin, "
        '`sim:=isaac`, `enable_intelligence:=false` (missione fissa, niente LLM).',
        f"- Sim: Isaac sulla work, mondo office, layout `{session.get('layout', '?')}` con `scene.car.start` per posa.",
        f"- Missione: `{session.get('mission_name', 'mission.json')}` (rettilineo 5 m a 0.4 m/s, poi `enter_door` left).",
        f"- Pose: {len(bypose)} da `{session.get('poses_file', '?')}`; modi {', '.join(modes)}; "
        f"{session.get('reps', '?')} ripetizioni.",
        '',
        '## Metodo',
        '',
        'Per posa: Isaac riavviato con la posa; per modo: stack avviato da zero; per ripetizione: `/reset_all` '
        '+ i controlli di restart.sh (tolleranze office 5 cm / 1.5 deg), load + start della missione, registrazione '
        'fino a COMPLETE/ABORTED + 3 s. **Successo** = missione COMPLETE **e**, dalla ground truth di Isaac, '
        f'l\'auto finisce nel varco vero (|along| <= {DOORWAY_ALONG_TOL_M} m, |lateral| <= w/2 - {CAR_HALF_WIDTH_M} m, '
        f'yaw entro {DOORWAY_YAW_TOL_DEG:.0f} deg) oppure lo ha attraversato dentro l\'apertura. '
        'Rifare l\'analisi: `python3 scripts/car/orin/hil_agent.py analyze --session <dir>`.',
        '',
        '## Risultati',
        '',
        '| modo | run | successo | IC 95 % | COMPLETE | nel varco (GT) | attraversato | rettilineo ok | durata mediana [s] | CPU ctrl [ms/s sim] | fallimenti |',
        '|---|---|---|---|---|---|---|---|---|---|---|',
    ]
    for s in summary:
        lines.append(f"| {s['mode']} | {s['runs']} | {s['success']} ({s['success_pct']} %) | {s['ci95_pct']} % | "
                     f"{s['mission_complete']} | {s['gt_in_door']} | {s['crossed_door']} | {s['straight_ok']} | "
                     f"{s['median_duration_s'] if s['median_duration_s'] is not None else '-'} | "
                     f"{s['median_cpu_ms_per_sim_s'] if s['median_cpu_ms_per_sim_s'] is not None else '-'} | "
                     f"{s['failures'] or '-'} |")
    lines += ['', '![successo per modo](figures/success_by_mode.png)', '', '### Per posa (successi / ripetizioni)', '',
              '| posa | varco [m] | ' + ' | '.join(modes) + ' |', '|---|---|' + '---|' * len(modes)]
    for b in bypose:
        lines.append(f"| {b['pose_idx']} | {b['door_width_m']} | " + ' | '.join(b[m] for m in modes) + ' |')
    lines += ['', 'Traiettorie per posa: `figures/pose_NN.png` (linea piena = successo, x = fine di un run fallito).',
              '', '## Conclusioni', '', 'n/d (da scrivere dopo aver guardato i fallimenti).', '',
              '## Prossimi passi', '', '- ...', '']
    with open(os.path.join(out, 'report.md'), 'w') as f:
        f.write('\n'.join(lines))


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest='cmd', required=True)
    p = sub.add_parser('profile')
    p.add_argument('--ip', required=True)
    p.add_argument('--buffer-mb', type=int, default=16)
    p.add_argument('--out', required=True)
    p = sub.add_parser('wait')
    p.add_argument('--timeout', type=float, default=240.0)
    p.add_argument('topics', nargs='+')
    p = sub.add_parser('ready')
    p.add_argument('--node', required=True, choices=sorted(CONTROLLERS.values()))
    p.add_argument('--timeout', type=float, default=240.0)
    p = sub.add_parser('run')
    p.add_argument('--session', required=True)
    p.add_argument('--run-id', required=True)
    p.add_argument('--mode', required=True, choices=sorted(CONTROLLERS))
    p.add_argument('--pose-idx', type=int, required=True, help="the pose's idx in session.json")
    p.add_argument('--rep', type=int, default=None)
    p.add_argument('--mission', required=True)
    p.add_argument('--timeout', type=float, default=180.0)
    p.add_argument('--settle', type=float, default=5.0)
    p = sub.add_parser('mark')
    p.add_argument('--session', required=True)
    p.add_argument('--run-id', required=True)
    p.add_argument('--mode', required=True, choices=sorted(CONTROLLERS))
    p.add_argument('--rep', type=int, default=None)
    p.add_argument('--pose-idx', type=int, required=True)
    p.add_argument('--status', required=True)
    p.add_argument('--note', default='')
    p = sub.add_parser('versions')
    p.add_argument('--out', required=True)
    p = sub.add_parser('analyze')
    p.add_argument('--session', required=True)
    a = ap.parse_args()
    return {'profile': cmd_profile, 'wait': cmd_wait, 'ready': cmd_ready, 'run': cmd_run,
            'mark': cmd_mark, 'versions': cmd_versions, 'analyze': cmd_analyze}[a.cmd](a)


if __name__ == '__main__':
    sys.exit(main())
