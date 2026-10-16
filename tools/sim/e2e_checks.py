#!/usr/bin/env python3
"""End-to-end checks of the stack against the Isaac sim (tools/e2e_test.sh runs this).

One rclpy node (run it as a Discovery Server super client). Each subcommand prints one JSON
object on stdout and exits 0 (pass) or 1 (fail):

  hz        rates of the stack's input and output topics, against minimums
  perception  person detected (2D + 3D, 3D position vs ground truth) and the LiDAR front
            wall distance vs ground truth (wall inner face x = --wall-x, base_link origin)
  mission   load + start a mission through the mission manager; record MissionStatus,
            MPC solve times, the car's ground-truth travel; check the state reaches COMPLETE
  reset     /reset_all (reset_manager, which calls /sim/reset): car back at the start pose
            (ground truth) and both EKFs at 0
Ground truth comes from the sim (/sim/ground_truth/*), never from the stack.
"""
import argparse
import json
import math
import statistics
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rosidl_runtime_py.utilities import get_message


def _yaw(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def _stats(xs):
    if not xs:
        return None
    s = sorted(xs)
    return {'n': len(s), 'mean': round(statistics.mean(s), 5), 'p50': round(s[len(s) // 2], 5),
            'p95': round(s[min(len(s) - 1, int(0.95 * len(s)))], 5), 'max': round(s[-1], 5)}


def _find_proc(node_name):
    import psutil
    for p in psutil.process_iter(['cmdline']):
        if f'__node:={node_name}' in ' '.join(p.info['cmdline'] or []):
            return p
    return None


def _cpu_s(p):
    if p is None:
        return None
    try:
        t = p.cpu_times()
        return t.user + t.system
    except Exception:  # noqa: BLE001 - process gone
        return None


class Checker(Node):

    def __init__(self):
        super().__init__('e2e_checks')
        self.gt = None          # (t_wall, x, y, yaw, vx)
        self.gt_person = None   # (x, y)
        from nav_msgs.msg import Odometry
        from geometry_msgs.msg import PoseStamped
        self.create_subscription(Odometry, '/sim/ground_truth/odom', self._on_gt, 50)
        self.create_subscription(PoseStamped, '/sim/ground_truth/person_pose',
                                 self._on_person, 10)

    def _on_gt(self, m):
        p = m.pose.pose
        self.gt = (time.time(), p.position.x, p.position.y, _yaw(p.orientation),
                   m.twist.twist.linear.x)

    def _on_person(self, m):
        self.gt_person = (m.pose.position.x, m.pose.position.y)

    def spin_for(self, sec, until=None):
        end = time.time() + sec
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.05)
            if until is not None and until():
                return True
        return until() if until is not None else True

    def call(self, srv_type, name, req, timeout=30.0):
        cli = self.create_client(srv_type, name)
        if not cli.wait_for_service(timeout_sec=10.0):
            return None
        fut = cli.call_async(req)
        end = time.time() + timeout
        while not fut.done() and time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.05)
        return fut.result() if fut.done() else None


# ---- hz ---------------------------------------------------------------------------------
# topic -> (type, minimum Hz). Minimums: ~80 % of the sim's configured rate for sim
# outputs (isaac_ws config/sensors.yaml), and of the car's rate for stack outputs.
HZ_TOPICS = {
    '/clock': ('rosgraph_msgs/msg/Clock', 50.0),
    '/scan': ('sensor_msgs/msg/LaserScan', 32.0),
    '/odom': ('nav_msgs/msg/Odometry', 40.0),
    '/sensors/imu/raw': ('sensor_msgs/msg/Imu', 40.0),
    '/camera/image_raw': ('sensor_msgs/msg/Image', 10.0),
    '/zed2/zed_node/depth/depth_registered': ('sensor_msgs/msg/Image', 10.0),
    '/odometry/filtered': ('nav_msgs/msg/Odometry', 20.0),
    '/ekf_global/odometry/filtered': ('nav_msgs/msg/Odometry', 10.0),
    # motion-gated (slam_toolbox minimum_travel_distance 0.03 m): informational when parked,
    # its rate while driving is in the mission check
    '/slam/pose': ('geometry_msgs/msg/PoseWithCovarianceStamped', 0.0),
    '/camera/detections': ('vision_msgs/msg/Detection2DArray', 3.0),
    '/camera/detections_3d': ('vision_msgs/msg/Detection3DArray', 3.0),
    '/perception/lidar_front_wall': ('f1tenth_messages/msg/WallLineFit', 32.0),
    '/costmap/front_clearance': ('std_msgs/msg/Float32', 5.0),
    '/ackermann_drive': ('ackermann_msgs/msg/AckermannDriveStamped', 5.0),
    '/diagnostics/system_status': ('f1tenth_messages/msg/SystemStatus', 0.5),
}


def cmd_hz(node, a):
    from rclpy.qos import ReliabilityPolicy as RP
    counts = {t: [] for t in HZ_TOPICS}
    subs, qos_used = [], {}
    node.spin_for(3.0)   # discovery, to read each publisher's QoS
    for t, (typ, _) in HZ_TOPICS.items():
        # Match the publishers' reliability: a best-effort reader of a reliable ~1 MB image
        # stream loses fragments and under-counts (the stack's own readers are reliable).
        pubs = node.get_publishers_info_by_topic(t)
        reliable = bool(pubs) and all(i.qos_profile.reliability == RP.RELIABLE for i in pubs)
        qos = QoSProfile(depth=10, reliability=RP.RELIABLE) if reliable \
            else qos_profile_sensor_data
        qos_used[t] = 'reliable' if reliable else 'best_effort'
        try:
            cls = get_message(typ)
        except Exception as e:  # noqa: BLE001 - report, do not crash the run
            counts[t] = None
            node.get_logger().warn(f'{t}: {e}')
            continue
        subs.append(node.create_subscription(
            cls, t, lambda _m, t=t: counts[t].append(time.time()), qos))
    node.spin_for(2.0)   # discovery
    for t in counts:
        if counts[t] is not None:
            counts[t].clear()
    t0 = time.time()
    node.spin_for(a.window)
    dt = time.time() - t0
    rows, ok = {}, True
    for t, (typ, mn) in HZ_TOPICS.items():
        hz = None if counts[t] is None else round(len(counts[t]) / dt, 2)
        good = hz is not None and hz >= mn
        ok &= good
        rows[t] = {'hz': hz, 'min': mn, 'pass': good, 'qos': qos_used.get(t)}
    return ok, {'window_s': round(dt, 1), 'topics': rows}


# ---- perception -------------------------------------------------------------------------
def cmd_perception(node, a):
    from f1tenth_messages.msg import WallLineFit
    from vision_msgs.msg import Detection2DArray, Detection3DArray
    import tf2_ros
    from tf2_geometry_msgs import do_transform_point  # noqa: F401 (registers PointStamped)
    from geometry_msgs.msg import PointStamped

    buf = tf2_ros.Buffer()
    tf2_ros.TransformListener(buf, node)
    det2d, det3d, walls = [], [], []

    def on2d(m):
        for d in m.detections:
            for h in d.results:
                if h.hypothesis.class_id == 'person':
                    det2d.append(h.hypothesis.score)

    def on3d(m):
        for d in m.detections:
            if any(h.hypothesis.class_id == 'person' for h in d.results):
                p = PointStamped()
                p.header = m.header
                p.point = d.bbox.center.position
                try:
                    q = buf.transform(p, 'base_link', timeout=rclpy.duration.Duration(seconds=0.2))
                except Exception:  # noqa: BLE001 - frame not known yet: skip this one
                    continue
                if node.gt is not None and node.gt_person is not None:
                    _, x, y, yaw, _ = node.gt
                    dx, dy = node.gt_person[0] - x, node.gt_person[1] - y
                    gx = math.cos(yaw) * dx + math.sin(yaw) * dy
                    gy = -math.sin(yaw) * dx + math.cos(yaw) * dy
                    det3d.append((q.point.x, q.point.y, gx, gy))

    def onwall(m):
        if node.gt is not None:
            _, x, _, yaw, _ = node.gt
            walls.append((m.valid, m.distance, (a.wall_x - x) * 1.0, abs(math.degrees(yaw))))

    node.create_subscription(Detection2DArray, '/camera/detections', on2d, 10)
    node.create_subscription(Detection3DArray, '/camera/detections_3d', on3d, 10)
    node.create_subscription(WallLineFit, '/perception/lidar_front_wall', onwall,
                             qos_profile_sensor_data)
    node.spin_for(a.window)
    out = {'person_2d': {'n': len(det2d), 'score': _stats(det2d)}}
    if det3d:
        ex = [d[0] - d[2] for d in det3d]
        ey = [d[1] - d[3] for d in det3d]
        out['person_3d'] = {'n': len(det3d),
                            'detected_base_link_xy': [round(det3d[-1][0], 3),
                                                      round(det3d[-1][1], 3)],
                            'ground_truth_base_link_xy': [round(det3d[-1][2], 3),
                                                          round(det3d[-1][3], 3)],
                            'err_x': _stats([abs(e) for e in ex]),
                            'err_y': _stats([abs(e) for e in ey])}
    else:
        out['person_3d'] = {'n': 0}
    valid = [w for w in walls if w[0]]
    errs = [w[1] - w[2] for w in valid]
    out['front_wall'] = {'scans': len(walls), 'valid': len(valid),
                         'measured_m': _stats([w[1] for w in valid]),
                         'ground_truth_m': _stats([w[2] for w in valid]),
                         'err_m': _stats([abs(e) for e in errs]),
                         'signed_err_mean_m': round(statistics.mean(errs), 4) if errs else None,
                         'tolerance_m': a.wall_tol}
    perr = [math.hypot(d[0] - d[2], d[1] - d[3]) for d in det3d]
    if perr:
        out['person_3d']['err_xy_m'] = _stats(perr)
        out['person_3d']['tolerance_m'] = a.person_tol
    # median, so one bad frame cannot fail the step
    ok_person = bool(det2d) and bool(perr) and statistics.median(perr) <= a.person_tol
    ok_wall = bool(errs) and max(abs(e) for e in errs) <= a.wall_tol
    out['pass'] = {'person': ok_person, 'front_wall': ok_wall}
    return ok_person and ok_wall, out


# ---- mission ----------------------------------------------------------------------------
def cmd_mission(node, a):
    from f1tenth_messages.msg import MissionStatus, MpcSolverStatus
    from f1tenth_messages.srv import LoadMission
    from std_srvs.srv import Trigger

    from geometry_msgs.msg import PoseWithCovarianceStamped
    states, solve, events, slam = [], [], [], []
    qos = QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                     reliability=ReliabilityPolicy.RELIABLE)
    node.create_subscription(MissionStatus, '/mission/status',
                             lambda m: states.append((time.time(), m.state)), qos)
    node.create_subscription(MpcSolverStatus, '/mpc/solver_status',
                             lambda m: solve.append((m.success, m.solve_dt_sec,
                                                     m.control_period_sec)), 50)
    node.create_subscription(PoseWithCovarianceStamped, '/slam/pose',
                             lambda m: slam.append(time.time()), 10)
    node.spin_for(10.0, until=lambda: node.gt is not None)
    start = node.gt
    req = LoadMission.Request()
    req.path = a.mission
    r = node.call(LoadMission, '/mission/load_mission', req)
    events.append({'load': None if r is None else {'success': r.success, 'message': r.message}})
    if r is None or not r.success:
        return False, {'events': events, 'states': [s for _, s in states]}
    r = node.call(Trigger, '/mission/start_mission', Trigger.Request())
    events.append({'start': None if r is None else {'success': r.success, 'message': r.message}})
    if r is None or not r.success:
        return False, {'events': events, 'states': [s for _, s in states]}
    t_start = time.time()
    # the mission manager counts down (mission_countdown_sec) before RUNNING
    node.spin_for(60.0, until=lambda: bool(states) and states[-1][0] > t_start
                  and states[-1][1] != 'LOADED')
    t_run = time.time()
    solve.clear()
    slam.clear()
    mpc_proc = _find_proc('mpc_corr')
    cpu0 = _cpu_s(mpc_proc)
    done = node.spin_for(a.timeout, until=lambda: bool(states) and
                         states[-1][1] in ('COMPLETE', 'ABORTED') and states[-1][0] > t_start)
    end = node.gt
    cpu1 = _cpu_s(mpc_proc)
    seq = []
    for _, s in states:
        if not seq or seq[-1] != s:
            seq.append(s)
    travel = None
    if start and end:
        travel = {'start_xy_yaw': [round(start[1], 3), round(start[2], 3),
                                   round(math.degrees(start[3]), 2)],
                  'end_xy_yaw': [round(end[1], 3), round(end[2], 3),
                                 round(math.degrees(end[3]), 2)],
                  'distance_m': round(math.hypot(end[1] - start[1], end[2] - start[2]), 3)}
    ok_solve = [s[1] for s in solve if s[0]]
    out = {'events': events, 'state_sequence': seq, 'final': states[-1][1] if states else None,
           'duration_s': round(time.time() - t_start, 1), 'ground_truth_travel': travel,
           'mpc': {'messages': len(solve), 'failed': sum(1 for s in solve if not s[0]),
                   'solve_dt_s': _stats(ok_solve),
                   'control_period_s': _stats([s[2] for s in solve])},
           'slam_pose_msgs': len(slam)}
    # MpcSolverStatus.solve_dt_sec is measured with the node clock, i.e. /clock in sim, which
    # does not advance inside the callback (single-threaded executor): it reads 0. The
    # process CPU time per published tick is an upper bound of the solve time instead.
    if cpu0 is not None and cpu1 is not None and solve:
        run_s = max(1e-3, time.time() - t_run)
        out['mpc']['running_s'] = round(run_s, 1)
        out['mpc']['process_cpu_s'] = round(cpu1 - cpu0, 3)
        out['mpc']['process_cpu_pct_of_core'] = round(100.0 * (cpu1 - cpu0) / run_s, 1)
        out['mpc']['cpu_s_per_tick'] = round((cpu1 - cpu0) / len(solve), 4)
    if end and node.gt_person and a.person_gap:
        d = math.hypot(node.gt_person[0] - end[1], node.gt_person[1] - end[2])
        out['person_gap_m'] = round(d - a.base_front_x - a.person_radius, 3)
    ok = bool(done and states and states[-1][1] == 'COMPLETE')
    if a.expect_distance:
        # a drive mission: the car really travelled the distance (ground truth), and SLAM
        # updated while driving (2 m is far beyond its 0.03 m minimum_travel_distance)
        if travel is None:
            ok = False
        else:
            out['distance_err_m'] = round(travel['distance_m'] - a.expect_distance, 3)
            out['distance_tol_m'] = a.distance_tol
            ok = ok and abs(out['distance_err_m']) <= a.distance_tol
        ok = ok and len(slam) > 0
    return ok, out


# ---- reset ------------------------------------------------------------------------------
def cmd_reset(node, a):
    from nav_msgs.msg import Odometry
    from std_srvs.srv import Trigger
    ekf = {}
    node.create_subscription(Odometry, '/odometry/filtered',
                             lambda m: ekf.__setitem__('local', m.pose.pose), 10)
    node.create_subscription(Odometry, '/ekf_global/odometry/filtered',
                             lambda m: ekf.__setitem__('global', m.pose.pose), 10)
    node.spin_for(10.0, until=lambda: node.gt is not None)
    before = node.gt
    t0 = time.time()
    r = node.call(Trigger, '/reset_all', Trigger.Request(), timeout=120.0)
    dt = time.time() - t0
    ekf.clear()
    node.spin_for(3.0, until=lambda: 'local' in ekf and 'global' in ekf)
    first = dict(ekf)
    gt_first = node.gt
    node.spin_for(3.0)
    after = node.gt
    out = {'service': None if r is None else {'success': r.success, 'message': r.message[:400]},
           'duration_s': round(dt, 1)}
    if before and gt_first:
        out['ground_truth_before'] = [round(before[1], 3), round(before[2], 3),
                                      round(math.degrees(before[3]), 2)]
        out['ground_truth_after'] = [round(gt_first[1], 3), round(gt_first[2], 3),
                                     round(math.degrees(gt_first[3]), 2)]
        out['start_pose_err_m'] = round(math.hypot(gt_first[1] - a.start_x,
                                                   gt_first[2] - a.start_y), 3)
    for k, p in first.items():
        out[f'ekf_{k}_first'] = [round(p.position.x, 3), round(p.position.y, 3),
                                 round(math.degrees(_yaw(p.orientation)), 2)]
    for k, p in ekf.items():
        out[f'ekf_{k}_3s'] = [round(p.position.x, 3), round(p.position.y, 3),
                              round(math.degrees(_yaw(p.orientation)), 2)]
    if gt_first and after:
        out['ground_truth_moved_in_3s_m'] = round(math.hypot(after[1] - gt_first[1],
                                                             after[2] - gt_first[2]), 3)
    # Pass: reset_manager reports success (it verifies both EKFs read 0 0 0 itself), the sim
    # is back at its start pose, and the first EKF outputs after the service are near 0. They
    # come ~4 s after the EKF reset (SLAM's reset takes ~3.9 s), during which the parked sim
    # car creeps (isaac_ws D25) and its slipping wheel odometry reports about twice that.
    ok = (r is not None and r.success and 'start_pose_err_m' in out
          and out['start_pose_err_m'] <= a.start_tol
          and all(abs(v) <= 0.05 for k in ('ekf_local_first', 'ekf_global_first')
                  for v in out.get(k, [1, 1])[:2])
          and all(abs(out.get(k, [0, 0, 99])[2]) <= 3.0
                  for k in ('ekf_local_first', 'ekf_global_first')))
    return ok, out


def main():
    ap = argparse.ArgumentParser()
    sub = ap.add_subparsers(dest='cmd', required=True)
    h = sub.add_parser('hz')
    h.add_argument('--window', type=float, default=10.0)
    p = sub.add_parser('perception')
    p.add_argument('--window', type=float, default=15.0)
    p.add_argument('--wall-x', type=float, default=6.0, help='inner face of the +x wall')
    p.add_argument('--wall-tol', type=float, default=0.10)
    p.add_argument('--person-tol', type=float, default=0.30,
                   help='median 3D person position error vs ground truth [m]')
    m = sub.add_parser('mission')
    m.add_argument('--mission', required=True, help='mission file (path)')
    m.add_argument('--timeout', type=float, default=90.0)
    m.add_argument('--expect-distance', type=float, default=0.0)
    m.add_argument('--distance-tol', type=float, default=0.25)
    m.add_argument('--person-gap', action='store_true',
                   help='report the ground-truth gap car front -> person near edge')
    m.add_argument('--base-front-x', type=float, default=0.405, help='base_link -> bumper')
    m.add_argument('--person-radius', type=float, default=0.175, help='sim person collider')
    r = sub.add_parser('reset')
    r.add_argument('--start-x', type=float, default=1.0)
    r.add_argument('--start-y', type=float, default=1.5)
    r.add_argument('--start-tol', type=float, default=0.05)
    a = ap.parse_args()

    rclpy.init()
    node = Checker()
    try:
        ok, out = {'hz': cmd_hz, 'perception': cmd_perception, 'mission': cmd_mission,
                   'reset': cmd_reset}[a.cmd](node, a)
    finally:
        node.destroy_node()
        rclpy.shutdown()
    out = {'check': a.cmd, 'pass': bool(ok), **out}
    print(json.dumps(out))
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
