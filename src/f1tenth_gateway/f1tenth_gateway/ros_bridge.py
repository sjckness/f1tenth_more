"""
The gateway's ROS side: one rclpy node, `pwa_gateway`, spun in its own thread.

What it does:
- publishes manual driving on /teleop (ackermann_mux lane "joystick",
  priority 100, under every /safety_stop source). A timer at publish_hz
  calls Teleop.tick() and publishes what it returns, so the 250 ms watchdog
  runs here, independent of the web server's event loop;
- holds the e-stop ready from start-up: the /safety_stop publisher and the
  /mission/emergency_stop and /mission/abort_mission clients exist and are
  matched before anyone presses EMERGENZA. The sequence itself is
  f1tenth_ctl's (rosprobe.estop_with), run on this node;
- feeds the mission tracker (/mission/status, /mission/move_outcome);
- follows LLM2 (two-llm-arch): /llm2/status always (one JSON every 2 s), and
  pauses / resumes it with /llm2/set_active from a client made at start
  (llm2.py says how the two make one state);
- keeps the latest telemetry. The heavier subscriptions (odometry, VESC,
  MPC horizon, SLAM map, behaviour tree stream) exist only while a phone is
  connected, and are dropped 30 s after the last one leaves.

Topics and types are the ones the stack publishes today (output/pwa_recon.md
section 1.7). Nothing here is Jazzy-only: the car runs Humble.
"""

import math
import threading
import time
from typing import Callable, Dict, List, Optional

from .llm2 import Llm2State

LINGER_S = 30.0
PATH_MIN_STEP_M = 0.05
PATH_MAX_POINTS = 3000

CMD_SOURCES = {
    '': 'mpc', 'base_link/pwa_teleop': 'app', 'base_link/emergency': 'emergenza BT',
    'base_link/hard_stop': 'hard stop LiDAR', 'base_link/obstacle': 'ostacolo',
    'base_link/f1tenth_ctl_estop': 'e-stop', 'base_link': 'reset',
}
BT_STATUS = {1: 'idle', 2: 'running', 3: 'success', 4: 'failure'}
BT_KIND = {2: 'sequence', 3: 'selector', 4: 'parallel', 5: 'chooser', 6: 'decorator',
           7: 'composite'}


def yaw_of(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def uuid_hex(u) -> str:
    return bytes(u.uuid).hex()


def bt_nodes(msg) -> List[Dict]:
    """py_trees_ros_interfaces/BehaviourTree -> [{id, parent, children, name, kind, status}]."""
    out = []
    for b in msg.behaviours:
        out.append({'id': uuid_hex(b.own_id), 'parent': uuid_hex(b.parent_id),
                    'children': [uuid_hex(c) for c in b.child_ids], 'name': b.name,
                    'kind': BT_KIND.get(b.type, 'behaviour'),
                    'status': BT_STATUS.get(b.status, 'idle')})
    ids = {n['id'] for n in out}
    for n in out:
        if n['parent'] not in ids:
            n['parent'] = None
    return out


class RosBridge:
    def __init__(self, cfg: Dict, teleop, estop_spec: Dict,
                 on_status: Callable, on_move_outcome: Callable):
        self.cfg = cfg
        self.teleop = teleop
        self.estop_spec = estop_spec
        self.on_status = on_status
        self.on_move_outcome = on_move_outcome
        self.viewers = 0
        self._lock = threading.Lock()
        self._latest: Dict = {}
        self._path: List = []
        self._path_version = 0
        self._bt = {'version': 0, 'nodes': [], 't': None}
        self._map = {'version': 0, 'msg': None}
        self._heavy = None              # subscriptions while someone watches
        self._last_viewer_t = 0.0
        self._bt_stream = None
        self._thread = None
        self._estop_lock = threading.Lock()
        llm2 = cfg['llm2']
        self.llm2 = Llm2State(llm2['service'], llm2['stale_s'], llm2['trust_s'])

    # ---- life cycle --------------------------------------------------------------------

    def start(self) -> None:
        import rclpy
        from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
        from rclpy.executors import MultiThreadedExecutor
        from rclpy.node import Node
        from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
        from ackermann_msgs.msg import AckermannDriveStamped
        from std_msgs.msg import String
        from std_srvs.srv import SetBool, Trigger
        from f1tenth_messages.msg import BatteryStatus, MissionStatus, MoveOutcome

        rclpy.init()
        self._rclpy = rclpy
        self.node = node = Node('pwa_gateway')
        self._Ackermann = AckermannDriveStamped
        self._latched = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                                   reliability=ReliabilityPolicy.RELIABLE,
                                   durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._sensor = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=5,
                                  reliability=ReliabilityPolicy.BEST_EFFORT,
                                  durability=DurabilityPolicy.VOLATILE)
        reliable = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=10,
                              reliability=ReliabilityPolicy.RELIABLE,
                              durability=DurabilityPolicy.VOLATILE)
        topics = self.cfg['topics']
        self._teleop_group = MutuallyExclusiveCallbackGroup()
        self._teleop_pub = node.create_publisher(
            AckermannDriveStamped, topics['teleop'],
            QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                       reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.VOLATILE))
        # The e-stop, ready now: same QoS as f1tenth_ctl's ZeroDrive.
        self._safety_pub = node.create_publisher(AckermannDriveStamped,
                                                 self.estop_spec['topic'], reliable)
        self._estop_cli = node.create_client(Trigger, self.estop_spec['service'])
        self._abort_cli = node.create_client(Trigger, self.estop_spec['abort_service'])
        self._Trigger = Trigger
        # LLM2: cheap (0.5 Hz), so always on, like the mission status.
        self._llm2_cli = node.create_client(SetBool, self.cfg['llm2']['service'])
        self._SetBool = SetBool
        node.create_subscription(String, topics['llm2_status'], self._llm2_status_cb, reliable)
        node.create_subscription(MissionStatus, topics['mission_status'], self._mission_cb,
                                 self._latched)
        node.create_subscription(MoveOutcome, topics['move_outcome'], self._outcome_cb,
                                 reliable)
        node.create_subscription(BatteryStatus, topics['battery'], self._battery_cb,
                                 reliable)
        period = 1.0 / float(self.teleop.cfg.publish_hz)
        node.create_timer(period, self._teleop_tick, callback_group=self._teleop_group)
        node.create_timer(1.0, self._housekeeping)
        self._executor = MultiThreadedExecutor(num_threads=4)
        self._executor.add_node(node)
        self._thread = threading.Thread(target=self._spin, name='ros', daemon=True)
        self._thread.start()

    def _spin(self) -> None:
        try:
            self._executor.spin()
        except Exception:  # noqa: BLE001 - shutdown races
            pass

    def stop(self) -> None:
        try:
            self._close_bt_stream()
            self._executor.shutdown(timeout_sec=1.0)
            self.node.destroy_node()
        finally:
            if self._rclpy.ok():
                self._rclpy.shutdown()

    # ---- teleop -------------------------------------------------------------------------

    def _teleop_tick(self) -> None:
        cmd = self.teleop.tick(time.monotonic())
        if cmd is not None:
            self.publish_teleop(cmd)

    def publish_teleop(self, cmd) -> None:
        msg = self._Ackermann()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.header.frame_id = self.cfg['teleop_frame_id']
        msg.drive.speed = float(cmd.speed)
        msg.drive.steering_angle = float(cmd.steer)
        self._teleop_pub.publish(msg)

    def teleop_graph(self) -> Dict:
        topic = self.cfg['topics']['teleop']
        subs = self.node.count_subscribers(topic)
        pubs = self.node.count_publishers(topic)
        return {'subscribers': subs, 'other_publishers': max(0, pubs - 1)}

    def estop_graph(self) -> Dict:
        """
        Tell whether the e-stop's two paths are there, for the app's button.

        The e-stop is only as good as what it can reach: the zeros need the
        mux subscribed to /safety_stop, and the latch needs
        /mission/emergency_stop served. Both are made at start-up, so this is
        a graph question, not a question about this node.
        """
        topic = self.estop_spec['topic']
        return {'zero_subscribers': self.node.count_subscribers(topic),
                'service_ready': bool(self._estop_cli.service_is_ready()),
                'abort_ready': bool(self._abort_cli.service_is_ready())}

    # ---- e-stop and abort ------------------------------------------------------------------

    def estop(self) -> Dict:
        """Run f1tenth_ctl's e-stop on this node, with the publisher and clients made at start."""
        from f1tenth_ctl import rosprobe
        with self._estop_lock:
            return rosprobe.estop_with(
                self.node, self.estop_spec, spin=lambda: time.sleep(0.002),
                zero_pub=self._safety_pub,
                clients={'emergency_stop': self._estop_cli, 'abort_mission': self._abort_cli})

    def abort(self, timeout_s: float = 1.0) -> Dict:
        from f1tenth_ctl import rosprobe
        t0 = time.monotonic()
        if not self._abort_cli.service_is_ready():
            return {'ok': False, 'message': 'servizio /mission/abort_mission non disponibile'}
        fut = self._abort_cli.call_async(self._Trigger.Request())
        while not fut.done() and time.monotonic() - t0 < timeout_s:
            time.sleep(0.005)
        if not fut.done():
            fut.cancel()
            return {'ok': False, 'message': f'nessuna risposta entro {timeout_s:g} s'}
        resp = fut.result()
        if resp is None:
            return {'ok': False, 'message': str(fut.exception())}
        ok = bool(resp.success) or rosprobe.abort_ok(resp.message)
        return {'ok': ok, 'message': resp.message,
                'duration_ms': int((time.monotonic() - t0) * 1000)}

    # ---- LLM2 pause / resume -------------------------------------------------------------------

    def llm2_set_active(self, active: bool, timeout_s: float = 2.0) -> Dict:
        """Call /llm2/set_active; the answer is LLM2's state at once (Llm2State.on_response)."""
        t0 = time.monotonic()
        service = self.cfg['llm2']['service']
        if not self._llm2_cli.service_is_ready():
            return {'ok': False, 'code': 409,
                    'message': f'LLM2 non attivo: servizio {service} non disponibile'}
        req = self._SetBool.Request()
        req.data = bool(active)
        fut = self._llm2_cli.call_async(req)
        while not fut.done() and time.monotonic() - t0 < timeout_s:
            time.sleep(0.005)
        if not fut.done():
            fut.cancel()
            return {'ok': False, 'message': f'{service}: nessuna risposta entro {timeout_s:g} s'}
        resp = fut.result()
        if resp is None:
            return {'ok': False, 'message': f'{service}: {fut.exception()}'}
        res = self.llm2.on_response(active, resp.success, resp.message, time.monotonic())
        res['duration_ms'] = int((time.monotonic() - t0) * 1000)
        return res

    def llm2_view(self) -> Dict:
        return self.llm2.view(time.monotonic(), bool(self._llm2_cli.service_is_ready()))

    # ---- always-on callbacks ------------------------------------------------------------------

    def _mission_cb(self, msg) -> None:
        self.on_status(msg.state, msg.json_path, bool(msg.emergency_stop_active),
                       time.monotonic())

    def _outcome_cb(self, msg) -> None:
        self.on_move_outcome(msg.mission_id, msg.move_id)

    def _battery_cb(self, msg) -> None:
        self._put('battery', {'v': round(float(msg.voltage), 2), 'ok': bool(msg.ok),
                              'has_data': bool(msg.has_data),
                              'min_v': round(float(msg.min_voltage), 2)})

    def _llm2_status_cb(self, msg) -> None:
        self.llm2.on_status(msg.data, time.monotonic())

    def _put(self, key: str, value) -> None:
        with self._lock:
            self._latest[key] = (time.monotonic(), value)

    # ---- the subscriptions that exist while someone watches --------------------------------

    def set_viewers(self, n: int) -> None:
        self.viewers = n
        if n > 0:
            self._last_viewer_t = time.monotonic()

    def _housekeeping(self) -> None:
        now = time.monotonic()
        if self.viewers > 0:
            self._last_viewer_t = now
            if self._heavy is None:
                self._open_heavy()
            self._check_bt_stream(now)
        elif self._heavy is not None and now - self._last_viewer_t > LINGER_S:
            self._close_heavy()

    def _open_heavy(self) -> None:
        from nav_msgs.msg import OccupancyGrid, Odometry
        from ackermann_msgs.msg import AckermannDriveStamped
        from f1tenth_messages.msg import BehaviorTreeStatus, MpcSolverStatus, SystemStatus
        t, node, subs = self.cfg['topics'], self.node, []
        subs.append(node.create_subscription(Odometry, t['odom'], self._odom_cb, self._sensor))
        subs.append(node.create_subscription(Odometry, t['pose_map'], self._pose_map_cb,
                                             self._sensor))
        subs.append(node.create_subscription(Odometry, t['pose_odom'], self._pose_odom_cb,
                                             self._sensor))
        subs.append(node.create_subscription(AckermannDriveStamped, t['drive_out'],
                                             self._drive_cb, self._sensor))
        subs.append(node.create_subscription(SystemStatus, t['system'], self._system_cb, 1))
        subs.append(node.create_subscription(BehaviorTreeStatus, t['tree_status'],
                                             self._tree_status_cb, 1))
        subs.append(node.create_subscription(MpcSolverStatus, t['mpc_solver'],
                                             self._solver_cb, self._sensor))
        subs.append(node.create_subscription(OccupancyGrid, t['map'], self._map_cb,
                                             self._latched))
        try:
            from vesc_msgs.msg import VescStateStamped
            subs.append(node.create_subscription(VescStateStamped, t['vesc'], self._vesc_cb,
                                                 self._sensor))
        except ImportError:
            pass
        self._heavy = subs

    def _close_heavy(self) -> None:
        for s in self._heavy or []:
            self.node.destroy_subscription(s)
        self._heavy = None
        self._close_bt_stream()

    def _odom_cb(self, msg) -> None:
        v = float(msg.twist.twist.linear.x)
        self._put('v_meas', {'v': round(v, 3), 'src': 'odom'})
        self.teleop.measured(v, time.monotonic())       # the direction change waits for 0

    def _pose_odom_cb(self, msg) -> None:
        p = msg.pose.pose
        self._put('pose_odom', (p.position.x, p.position.y, yaw_of(p.orientation)))

    def _pose_map_cb(self, msg) -> None:
        p = msg.pose.pose
        x, y, yaw = p.position.x, p.position.y, yaw_of(p.orientation)
        self._put('pose', {'x': round(x, 3), 'y': round(y, 3), 'yaw': round(yaw, 4),
                           'frame': msg.header.frame_id or 'map'})
        with self._lock:
            last = self._path[-1] if self._path else None
            if last is None or math.hypot(x - last[0], y - last[1]) >= PATH_MIN_STEP_M:
                self._path.append((round(x, 3), round(y, 3)))
                del self._path[:-PATH_MAX_POINTS]
                self._path_version += 1

    def _drive_cb(self, msg) -> None:
        fid = msg.header.frame_id
        self._put('cmd', {'v': round(float(msg.drive.speed), 3),
                          'steer': round(float(msg.drive.steering_angle), 4),
                          'source': CMD_SOURCES.get(fid, fid)})

    def _system_cb(self, msg) -> None:
        self._put('system', {'cpu_percent': round(float(msg.cpu_percent), 1),
                             'cpu_temp_c': round(float(msg.cpu_temp_c), 1)})

    def _tree_status_cb(self, msg) -> None:
        self._put('safety', {'active': bool(msg.safety_stop_active or msg.emergency_trip),
                             'source': msg.stop_source, 'lane': msg.active_lane})

    def _vesc_cb(self, msg) -> None:
        s = msg.state
        fields = ('ntc_temp_mos1', 'ntc_temp_mos2', 'ntc_temp_mos3')
        temps = [float(getattr(s, f, 0.0)) for f in fields]
        self._put('vesc_temp_c', round(max(temps), 1))

    def _solver_cb(self, msg) -> None:
        pts = list(zip(msg.pred_x, msg.pred_y))
        self._put('horizon_odom', {'frame': msg.prediction_frame_id, 'pts': pts})

    def _map_cb(self, msg) -> None:
        with self._lock:
            self._map = {'version': self._map['version'] + 1, 'msg': msg}

    # ---- behaviour tree snapshot stream --------------------------------------------------------

    def _check_bt_stream(self, now: float) -> None:
        """Open a snapshot stream of our own; reopen it when it goes quiet (behavior restarted)."""
        try:
            from py_trees_ros_interfaces.msg import BehaviourTree, SnapshotStreamParameters
            from py_trees_ros_interfaces.srv import OpenSnapshotStream
        except ImportError:
            return
        stream = self._bt_stream
        if stream and stream.get('topic') and now - (stream.get('last') or stream['t']) < 5.0:
            return
        if stream and stream.get('pending'):
            if now - stream['t'] < 3.0:
                return
        self._close_bt_stream()
        cli = self.node.create_client(OpenSnapshotStream, self.cfg['bt_open_service'])
        if not cli.service_is_ready():
            self.node.destroy_client(cli)
            return
        req = OpenSnapshotStream.Request()
        req.parameters = SnapshotStreamParameters(blackboard_data=False,
                                                  blackboard_activity=False,
                                                  snapshot_period=1.0)
        fut = cli.call_async(req)
        stream = {'pending': True, 't': now, 'cli': cli}
        self._bt_stream = stream

        def done(f):
            res = f.result()
            stream['pending'] = False
            if res is None or not res.topic_name:
                return
            stream['topic'] = res.topic_name
            stream['sub'] = self.node.create_subscription(BehaviourTree, res.topic_name,
                                                          self._bt_cb, self._latched)
        fut.add_done_callback(done)

    def _close_bt_stream(self) -> None:
        stream, self._bt_stream = self._bt_stream, None
        if not stream:
            return
        if stream.get('sub') is not None:
            self.node.destroy_subscription(stream['sub'])
        if stream.get('topic'):
            try:
                from py_trees_ros_interfaces.srv import CloseSnapshotStream
                cli = self.node.create_client(CloseSnapshotStream,
                                              self.cfg['bt_close_service'])
                if cli.service_is_ready():
                    cli.call_async(CloseSnapshotStream.Request(topic_name=stream['topic']))
            except ImportError:
                pass
        if stream.get('cli') is not None:
            self.node.destroy_client(stream['cli'])

    def _bt_cb(self, msg) -> None:
        nodes = bt_nodes(msg)
        stream = self._bt_stream
        if stream is not None:
            stream['last'] = time.monotonic()
        with self._lock:
            changed = [(n['id'], n['status']) for n in nodes] != \
                [(n['id'], n['status']) for n in self._bt['nodes']]
            self._bt = {'version': self._bt['version'] + (1 if changed else 0),
                        'nodes': nodes, 't': time.monotonic()}

    # ---- for the app ---------------------------------------------------------------------------

    def reset_path(self) -> None:
        with self._lock:
            self._path = []
            self._path_version += 1

    def latest(self, max_age_s: float = 2.0) -> Dict:
        now = time.monotonic()
        with self._lock:
            out = {k: v for k, (t, v) in self._latest.items() if now - t <= max_age_s}
            ages = {k: round(now - t, 2) for k, (t, v) in self._latest.items()}
        out['ages'] = ages
        horizon = out.pop('horizon_odom', None)
        out.pop('pose_odom', None)
        out['horizon'] = self._horizon_in_map(horizon)
        return out

    def _horizon_in_map(self, horizon) -> List:
        """Return the MPC's predicted path in map (odom -> map via the two EKFs' poses)."""
        if not horizon or not horizon['pts']:
            return []
        with self._lock:
            pm = self._latest.get('pose')
            po = self._latest.get('pose_odom')
        if not pm or not po:
            return []
        xm, ym, yawm = pm[1]['x'], pm[1]['y'], pm[1]['yaw']
        xo, yo, yawo = po[1]
        dyaw = yawm - yawo
        c, s = math.cos(dyaw), math.sin(dyaw)
        out = []
        for x, y in horizon['pts']:
            dx, dy = x - xo, y - yo
            out.append((round(xm + c * dx - s * dy, 3), round(ym + s * dx + c * dy, 3)))
        return out

    def path(self, since_version: int = 0) -> Dict:
        with self._lock:
            return {'version': self._path_version, 'points': list(self._path)}

    def bt(self) -> Dict:
        with self._lock:
            return {'version': self._bt['version'], 'nodes': self._bt['nodes']}

    def map_grid(self) -> Optional[Dict]:
        """Return the latest /slam/map as {version, width, height, resolution, origin, data}."""
        with self._lock:
            version, msg = self._map['version'], self._map['msg']
        if msg is None:
            return None
        info = msg.info
        o = info.origin.position
        return {'version': version, 'width': info.width, 'height': info.height,
                'resolution': info.resolution,
                'origin': {'x': o.x, 'y': o.y, 'yaw': yaw_of(info.origin.orientation)},
                'data': msg.data}
