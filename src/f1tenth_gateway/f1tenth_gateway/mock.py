"""
A pretend car for linus: the gateway's two ports (ROS, f1tenth_ctl) without ROS.

MockWorld is one small simulation both ports share: processes that start and
stop with a delay, READY checks, the e-stop latch, a mission that loads,
counts down, runs its moves and completes, a car that moves under teleop or
a mission, a planner that answers. MockRos and MockCtl expose it with the
same methods as RosBridge and CtlPort, so app.py runs unchanged on it.

Everything the gateway would publish on /teleop and /safety_stop is recorded
(MockWorld.published) for the tests: nothing in here touches a network.
"""

import json
import math
import os
import tempfile
import threading
import time
from typing import Dict, List, Optional

from .teleop import Command

MOVES_DEMO = [
    {'id': 'straight_1', 'drive': {'mode': 'straight', 'speed': 0.5},
     'stop_condition': {'type': 'distance_reached', 'distance': 2.0},
     'timeout_sec': 30, 'on_timeout': 'abort'},
    {'id': 'turn_2', 'drive': {'mode': 'wall_turn', 'turn_sign': -1.0, 'turn_mag_deg': 90.0,
                               'speed': 0.4},
     'stop_condition': {'type': 'orientation_delta', 'value': 90}, 'timeout_sec': 20,
     'on_timeout': 'abort'},
    {'id': 'to_person', 'go_to_object': {'target_class': 'person', 'speed': 0.4,
                                         'acquire_timeout_sec': 5.0, 'lost_grace_sec': 1.5},
     'stop_condition': {'type': 'object_reached'}, 'timeout_sec': 40, 'on_timeout': 'abort',
     'terminal': True},
]
CHECK_NAMES = ('odometry', 'scan', 'tf_map_base_link', 'mission_service', 'estop_latched',
               'battery', 'wheel_odom')


class MockWorld:
    def __init__(self, countdown_s: float = 3.0, move_s: float = 4.0, plan_s: float = 1.5):
        self._lock = threading.RLock()
        self.countdown_s = countdown_s
        self.move_s = move_s
        self.plan_s = plan_s
        self.dir = tempfile.mkdtemp(prefix='f1t_mock_')
        now = time.time()
        self.procs = {name: {'state': 'running', 'since': now - 754.0}
                      for name in ('stack', 'planner')}
        self.procs['logger'] = {'state': 'stopped', 'since': None}
        self.pending: Dict[str, tuple] = {}
        self.failing: List[str] = []
        self.estop_latched = False
        self.battery_v = 11.6
        self.ros_state = 'IDLE'
        self.json_path: Optional[str] = None
        self.mission: Optional[Dict] = None
        self.start_t: Optional[float] = None
        self.run_t: Optional[float] = None
        self.moves_done = 0
        self.outcomes: List[tuple] = []
        self.x, self.y, self.yaw, self.v = 0.0, 0.0, 0.0, 0.0
        self.cmd = (0.0, 0.0, 'mpc')
        self.teleop_t = 0.0
        self.published: List[tuple] = []         # (t, topic, speed, steer)
        self.store: Dict[str, Dict] = {}
        self.current: Dict = {}
        self.log: List[tuple] = []               # planner output: (offset, line)
        self.log_size = 0
        self.plan_jobs: List[tuple] = []
        self.status_listeners = []
        self.outcome_listeners = []
        self.teleop_others = 0
        self.mux_listens = True
        self.path: List[tuple] = []
        self.path_version = 0
        self.bt_version = 1

    def reset(self) -> None:
        """Back to a READY, idle car (the e2e tests share one gateway)."""
        with self._lock:
            now = time.time()
            self.procs = {name: {'state': 'running', 'since': now - 754.0}
                          for name in ('stack', 'planner')}
            self.procs['logger'] = {'state': 'stopped', 'since': None}
            self.pending.clear()
            self.failing = []
            self.estop_latched = False
            self.json_path, self.mission, self.start_t, self.run_t = None, None, None, None
            self.moves_done = 0
            self.x, self.y, self.yaw, self.v = 0.0, 0.0, 0.0, 0.0
            self.cmd, self.teleop_t = (0.0, 0.0, 'mpc'), 0.0
            self.teleop_others, self.mux_listens = 0, True
            self.published.clear()
            self.current = {}
            self.plan_jobs = []
            self.path, self.path_version = [], self.path_version + 1
            self._set_state('IDLE')

    # ---- processes ---------------------------------------------------------------------

    def proc_state(self, name: str) -> Dict:
        with self._lock:
            p = dict(self.procs[name])
            if name in self.pending:
                p['state'] = self.pending[name][0]
            return p

    def begin(self, name: str, action: str) -> Dict:
        with self._lock:
            state = self.procs[name]['state']
            if action == 'start' and state == 'running':
                return {'ok': False, 'target': name, 'error': f'{name} is already running'}
            if action == 'stop' and state == 'stopped':
                return {'ok': True, 'target': name, 'state': 'stopped', 'note': 'not running'}
            self.pending[name] = ('starting' if action == 'start' else 'stopping', time.time())
        time.sleep(1.5 if action == 'start' else 1.0)
        with self._lock:
            self.pending.pop(name, None)
            self.procs[name] = {'state': 'running' if action == 'start' else 'stopped',
                                'since': time.time() if action == 'start' else None}
            if name == 'stack' and action == 'stop':
                self.ros_state, self.json_path = 'IDLE', None
                self.estop_latched = False      # the BT went with it
            return {'ok': True, 'target': name, 'state': self.procs[name]['state']}

    # ---- the simulation ------------------------------------------------------------------

    def tick(self, dt: float) -> None:
        now = time.time()
        with self._lock:
            self._finish_plans(now)
            if self.ros_state == 'LOADED' and self.start_t is not None and \
                    now - self.start_t >= self.countdown_s:
                self._set_state('RUNNING')
                self.run_t = now
            if self.ros_state == 'RUNNING' and self.run_t is not None:
                n = len((self.mission or {}).get('moves', []))
                while self.moves_done < n and now - self.run_t >= (self.moves_done + 1) * \
                        self.move_s:
                    move = self.mission['moves'][self.moves_done]
                    self.moves_done += 1
                    for cb in self.outcome_listeners:
                        cb(self.mission.get('mission_id'), move.get('id'))
                if self.moves_done >= n:
                    self._set_state('COMPLETE')
            teleop_fresh = time.monotonic() - self.teleop_t < 0.25
            if self.estop_latched or self.procs['stack']['state'] != 'running':
                target, steer, src = 0.0, 0.0, 'emergenza BT' if self.estop_latched else 'mpc'
            elif teleop_fresh:
                target, steer, src = self.cmd
            elif self.ros_state == 'RUNNING':
                target, steer, src = 0.5, 0.12, 'mpc'
            else:
                target, steer, src = 0.0, 0.0, 'mpc'
            self.cmd = (target, steer, src) if not teleop_fresh else self.cmd
            self.v += max(-2.0 * dt, min(1.0 * dt, target - self.v))
            if abs(self.v) < 0.02 and target == 0.0:
                self.v = 0.0
            self.yaw += self.v / 0.33 * math.tan(steer) * dt
            self.x += self.v * math.cos(self.yaw) * dt
            self.y += self.v * math.sin(self.yaw) * dt
            last = self.path[-1] if self.path else None
            if last is None or math.hypot(self.x - last[0], self.y - last[1]) >= 0.05:
                self.path.append((round(self.x, 3), round(self.y, 3)))
                del self.path[:-3000]
                self.path_version += 1
            self.battery_v = max(10.9, self.battery_v - 0.00002)

    def _set_state(self, state: str) -> None:
        self.ros_state = state
        self.bt_version += 1
        for cb in self.status_listeners:
            cb(state, self.json_path or '', self.estop_latched, time.monotonic())

    def notify(self) -> None:
        with self._lock:
            for cb in self.status_listeners:
                cb(self.ros_state, self.json_path or '', self.estop_latched, time.monotonic())

    # ---- planner ---------------------------------------------------------------------------

    def _append(self, line: str) -> int:
        offset = self.log_size
        self.log.append((offset, line))
        self.log_size += len(line.encode()) + 1
        return offset

    def send(self, text: str) -> Dict:
        with self._lock:
            cursor = self.log_size
            self._append(f'comando> {text}')
            self.plan_jobs.append((time.time() + self.plan_s, text))
            return {'ok': True, 'cursor': cursor}

    def _finish_plans(self, now: float) -> None:
        due = [j for j in self.plan_jobs if j[0] <= now]
        self.plan_jobs = [j for j in self.plan_jobs if j[0] > now]
        for _t, text in due:
            self._append('traduzione...')
            if 'errore' in text.lower():
                self._append('RESULT {"status": "refused", "stage": "translator", '
                             '"error_type": "EmptyPlanError", "error_message": '
                             '"comando ambiguo"}')
                continue
            mid = f'llm_mock{len(self.store) + 1:04d}'
            mission = {'mission_id': mid, 'schema_version': '5.0',
                       'moves': [dict(m) for m in MOVES_DEMO]}
            stamp = time.strftime('%Y%m%dT%H%M%S')
            sid = f'{stamp}-{len(self.store) + 1:06x}'
            path = os.path.join(self.dir, f'{sid}.mission.json')
            with open(path, 'w') as f:
                json.dump(mission, f)
            self.store[sid] = {'id': sid, 'mission': mission, 'path': path, 'stored_at': now,
                               'mission_id': mid}
            self._append(f'RESULT-OK {sid}')

    def results_since(self, since: int) -> List[Dict]:
        out = []
        for offset, line in self.log:
            if offset < since:
                continue
            if line.startswith('RESULT-OK '):
                sid = line.split()[1]
                rec = self.store[sid]
                out.append({'offset': offset, 'status': 'ok', 'stage': 'delivery',
                            'mission_id': rec['mission_id'], 'plan_seconds': self.plan_s,
                            'unsupported': [], 'notes': [], 'mission': rec['mission'],
                            'stored': {'id': sid, 'path': rec['path'],
                                       'stored_at': rec['stored_at'],
                                       'mission_id': rec['mission_id']}})
            elif line.startswith('RESULT '):
                out.append({'offset': offset, 'status': 'refused', 'stage': 'translator',
                            'error_type': 'EmptyPlanError',
                            'error_message': 'comando ambiguo', 'unsupported': [],
                            'notes': []})
        return out


class MockCtl:
    """f1tenth_ctl's methods (CtlPort) on the MockWorld."""

    def __init__(self, world: MockWorld):
        self.w = world
        self.proc_locks = {n: threading.Lock() for n in ('stack', 'planner', 'logger')}
        self.plan_lock = threading.Lock()
        self.plan_max_age_s = 300.0

    def status(self) -> Dict:
        w = self.w
        with w._lock:
            stack_up = w.procs['stack']['state'] == 'running'
            checks = []
            for name in CHECK_NAMES:
                ok = name not in w.failing
                if name == 'estop_latched':
                    ok = not w.estop_latched
                value = {'odometry': 20.0, 'scan': 40.0, 'tf_map_base_link': 0.05,
                         'mission_service': True, 'estop_latched': w.estop_latched,
                         'battery': round(w.battery_v, 2), 'wheel_odom': 50.0}[name]
                want = {'odometry': '>= 10 Hz', 'scan': '>= 20 Hz',
                        'tf_map_base_link': '<= 0.5 s', 'mission_service': 'available',
                        'estop_latched': 'false', 'battery': '>= 10.8 V',
                        'wheel_odom': 'shown'}[name]
                if not ok and name in ('odometry', 'scan'):
                    value = 4.0
                checks.append({'name': name, 'ok': ok if stack_up else False, 'value': value,
                               'want': want, 'required': name != 'wheel_odom',
                               'why': None if ok else 'mock'})
            failing = [c['name'] for c in checks if c['required'] and not c['ok']]
            procs = {'discovery': {'state': 'running', 'listening': True}}
            for name in ('stack', 'planner', 'logger'):
                procs[name] = w.proc_state(name)
            return {'ok': True, 'ready': stack_up and not failing,
                    'failing': failing if stack_up else ['stack'],
                    'estop_latched': w.estop_latched, 'processes': procs,
                    'checks': checks if stack_up else [],
                    'probe': {'ran': stack_up, 'nodes': 63 if stack_up else None}}

    def start(self, name: str) -> Dict:
        return self.w.begin(name, 'start')

    def stop(self, name: str) -> Dict:
        return self.w.begin(name, 'stop')

    def estop(self) -> Dict:
        return MockRos(self.w, None).estop()

    def estop_reset(self) -> Dict:
        w = self.w
        with w._lock:
            if not w.estop_latched:
                return {'ok': True, 'path': 'none', 'message': 'the e-stop is not latched'}
        time.sleep(2.0)
        with w._lock:
            w.estop_latched = False
            if w.ros_state in ('RUNNING', 'LOADED', 'HOLDING'):
                w.ros_state = 'ABORTED'
            w.notify()
        return {'ok': True, 'path': 'restart_component', 'latched_before': True,
                'latched_after': False, 'verified': True}

    def estop_spec(self) -> Dict:
        return {'topic': '/safety_stop', 'service': '/mission/emergency_stop',
                'service_timeout_s': 0.5, 'abort_service': '/mission/abort_mission',
                'abort_timeout_s': 1.0, 'publish_s': 2.0, 'publish_hz': 50.0}

    def planner_log(self, since: Optional[int] = None) -> Dict:
        w = self.w
        with w._lock:
            running = w.procs['planner']['state'] == 'running'
            if since is None:
                return {'ok': True, 'running': running, 'plan_only': True if running else None,
                        'cursor': w.log_size, 'lines': [], 'offsets': [], 'results': []}
            lines = [(o, ln) for o, ln in w.log if o >= since]
            return {'ok': True, 'running': running, 'plan_only': True if running else None,
                    'cursor': w.log_size, 'lines': [ln for _o, ln in lines],
                    'offsets': [o for o, _ln in lines], 'results': w.results_since(since)}

    def send_planner(self, text: str) -> Dict:
        return {'target': 'planner', **self.w.send(text)}

    def mission_record(self, mid: str) -> Optional[Dict]:
        return self.w.store.get(mid)

    def mission_path(self, mid: str) -> str:
        rec = self.w.store.get(mid)
        return rec['path'] if rec else ''

    def summarize(self, mission: Dict) -> Dict:
        try:
            from f1tenth_ctl.missions import summarize
            return summarize(mission)
        except ImportError:
            moves = mission.get('moves', [])
            return {'n_moves': len(moves), 'moves': [{'i': i + 1, 'id': m.get('id'),
                                                      'action': '?'} for i, m in
                                                     enumerate(moves)],
                    'max_speed_mps': None, 'distance_m': None, 'max_duration_s': None,
                    'targets': [], 'reverse': False}

    def validate(self, path: str) -> Dict:
        return {'status': 'ok', 'ok': True, 'message': 'valid (mock)'}

    def mission_load(self, mid: str) -> Dict:
        w = self.w
        with w._lock:
            rec = w.store.get(mid)
            if rec is None:
                return {'ok': False, 'refused': f'unknown mission {mid}'}
            if w.ros_state in ('RUNNING', 'HOLDING'):
                return {'ok': False, 'refused': 'another mission is running'}
            if w.procs['stack']['state'] != 'running':
                return {'ok': False, 'refused': 'the stack is not running'}
            w.mission, w.json_path, w.moves_done = rec['mission'], rec['path'], 0
            w.start_t = w.run_t = None
            w.current = {'id': mid, 'path': rec['path'], 'started_at': None}
            w._set_state('LOADED')
            return {'ok': True, 'id': mid, 'load': {'ok': True, 'message': 'loaded'}}

    def mission_start(self, mid: str) -> Dict:
        w = self.w
        st = self.status()
        with w._lock:
            if w.current.get('id') != mid:
                return {'ok': False, 'refused': f'{mid} is not the loaded mission'}
            if w.estop_latched:
                return {'ok': False, 'refused': 'the e-stop is latched (estop-reset first)'}
            if not st['ready']:
                return {'ok': False, 'refused': 'not READY: ' + ', '.join(st['failing'])}
            w.start_t = time.time()
            w.current['started_at'] = w.start_t
            return {'ok': True, 'id': mid, 'countdown_s': w.countdown_s,
                    'start': {'ok': True, 'message': f'starting in {w.countdown_s:.1f} s'}}

    def describe_state(self, ros_state, json_path, wall_now: float) -> Dict:
        names = {'IDLE': 'idle', 'LOADED': 'loaded', 'RUNNING': 'running',
                 'HOLDING': 'running', 'COMPLETE': 'finished', 'ABORTED': 'aborted'}
        cur = self.w.current
        ours = bool(json_path) and json_path == cur.get('path')
        state = names.get(ros_state or '', 'unknown')
        started = cur.get('started_at') if ours else None
        if state == 'loaded' and started and wall_now - started <= self.w.countdown_s + 5.0:
            state = 'starting'
        return {'state': state, 'ros_state': ros_state, 'json_path': json_path,
                'id': cur.get('id') if ours else None}


class MockRos:
    """RosBridge's methods on the MockWorld; a thread plays the publish loop and the world."""

    def __init__(self, world: MockWorld, teleop, on_status=None, on_move_outcome=None):
        self.w = world
        self.teleop = teleop
        self.viewers = 0
        self._stop = threading.Event()
        self._thread = None
        if on_status is not None:
            world.status_listeners.append(on_status)
        if on_move_outcome is not None:
            world.outcome_listeners.append(on_move_outcome)

    def start(self) -> None:
        self._thread = threading.Thread(target=self._run, name='mock-ros', daemon=True)
        self._thread.start()
        self.w.notify()

    def _run(self) -> None:
        period = 1.0 / float(self.teleop.cfg.publish_hz) if self.teleop else 0.02
        last = time.monotonic()
        while not self._stop.is_set():
            now = time.monotonic()
            if self.teleop is not None:
                cmd = self.teleop.tick(now)
                if cmd is not None:
                    self.publish_teleop(cmd)
            self.w.tick(now - last)
            last = now
            time.sleep(period)

    def stop(self) -> None:
        self._stop.set()

    def publish_teleop(self, cmd: Command) -> None:
        with self.w._lock:
            self.w.published.append((time.monotonic(), '/teleop', cmd.speed, cmd.steer))
            del self.w.published[:-5000]
            if self.w.mux_listens:
                self.w.cmd = (cmd.speed, cmd.steer, 'app')
                self.w.teleop_t = time.monotonic()

    def teleop_graph(self) -> Dict:
        return {'subscribers': 1 if self.w.mux_listens else 0,
                'other_publishers': self.w.teleop_others}

    def estop(self) -> Dict:
        w = self.w
        t0 = time.monotonic()
        with w._lock:
            w.published.append((t0, '/safety_stop', 0.0, 0.0))
            w.estop_latched = True
            if w.ros_state in ('LOADED', 'RUNNING', 'HOLDING'):
                w.ros_state = 'ABORTED'
            w.notify()
        time.sleep(0.3)
        ms = int((time.monotonic() - t0) * 1000)
        return {'steps': {
            'zero_drive': {'status': 'ok', 'ok': True, 'message': '15 msgs to 1 subscriber(s)',
                           'duration_ms': ms},
            'emergency_stop': {'status': 'ok', 'ok': True, 'message': 'latched',
                               'duration_ms': 20},
            'abort_mission': {'status': 'ok', 'ok': True, 'message': 'aborted',
                              'duration_ms': 25}}, 'car_held': True}

    def abort(self, timeout_s: float = 1.0) -> Dict:
        w = self.w
        with w._lock:
            if w.ros_state in ('LOADED', 'RUNNING', 'HOLDING'):
                w._set_state('ABORTED')
                return {'ok': True, 'message': 'aborted'}
            return {'ok': True, 'message': f'no mission to abort (state={w.ros_state})'}

    def set_viewers(self, n: int) -> None:
        self.viewers = n

    def reset_path(self) -> None:
        with self.w._lock:
            self.w.path = []
            self.w.path_version += 1

    def latest(self, max_age_s: float = 2.0) -> Dict:
        w = self.w
        with w._lock:
            if w.procs['stack']['state'] != 'running':
                return {'ages': {}, 'horizon': []}
            horizon = [(round(w.x + math.cos(w.yaw) * d, 3), round(w.y + math.sin(w.yaw) * d, 3))
                       for d in (0.2, 0.4, 0.6, 0.8, 1.0, 1.2)] if w.v > 0.05 else []
            return {'pose': {'x': round(w.x, 3), 'y': round(w.y, 3), 'yaw': round(w.yaw, 4),
                             'frame': 'map'},
                    'v_meas': {'v': round(w.v, 3), 'src': 'odom'},
                    'cmd': {'v': round(w.cmd[0], 3), 'steer': round(w.cmd[1], 4),
                            'source': w.cmd[2]},
                    'battery': {'v': round(w.battery_v, 2), 'ok': w.battery_v >= 10.8,
                                'has_data': True, 'min_v': 10.8},
                    'vesc_temp_c': 38.0, 'system': {'cpu_percent': 41.0, 'cpu_temp_c': 52.0},
                    'safety': {'active': w.estop_latched,
                               'source': 'emergency_stop' if w.estop_latched else '',
                               'lane': 'emergency' if w.estop_latched else 'navigation'},
                    'ages': {'v_meas': 0.02}, 'horizon': horizon}

    def path(self, since_version: int = 0) -> Dict:
        with self.w._lock:
            return {'version': self.w.path_version, 'points': list(self.w.path)}

    def bt(self) -> Dict:
        w = self.w
        running = w.ros_state == 'RUNNING'
        st = (lambda cond: 'running' if cond else 'idle')
        nodes = [
            {'id': 'root', 'parent': None, 'children': ['emergency', 'mission', 'navigation'],
             'name': 'root', 'kind': 'selector', 'status': 'running'},
            {'id': 'emergency', 'parent': 'root', 'children': ['emergency_condition', 'stop'],
             'name': 'emergency', 'kind': 'sequence',
             'status': 'running' if w.estop_latched else 'failure'},
            {'id': 'emergency_condition', 'parent': 'emergency', 'children': [],
             'name': 'emergency_condition', 'kind': 'selector',
             'status': 'success' if w.estop_latched else 'failure'},
            {'id': 'stop', 'parent': 'emergency', 'children': [], 'name': 'Stop',
             'kind': 'behaviour', 'status': 'running' if w.estop_latched else 'idle'},
            {'id': 'mission', 'parent': 'root', 'children': ['active', 'progress'],
             'name': 'mission', 'kind': 'sequence', 'status': st(running)},
            {'id': 'active', 'parent': 'mission', 'children': [], 'name': 'MissionActive',
             'kind': 'behaviour', 'status': 'success' if running else 'failure'},
            {'id': 'progress', 'parent': 'mission', 'children': [], 'name': 'mission_progress',
             'kind': 'sequence', 'status': st(running)},
            {'id': 'navigation', 'parent': 'root', 'children': [], 'name': 'navigation',
             'kind': 'sequence', 'status': 'idle' if running else 'running'},
        ]
        return {'version': w.bt_version, 'nodes': nodes}

    def map_grid(self) -> Optional[Dict]:
        """Return a 12 x 6 m corridor with a doorway and a box, 0.05 m cells."""
        w, h, res = 240, 120, 0.05
        data = [-1] * (w * h)
        for row in range(h):
            for col in range(w):
                y = row * res - 3.0
                if -1.2 <= y <= 1.2:
                    data[row * w + col] = 0
                if abs(abs(y) - 1.25) < 0.05 and not (6.0 < col * res < 7.0 and y > 0):
                    data[row * w + col] = 100
                if 8.0 < col * res < 8.3 and -0.6 < y < -0.3:
                    data[row * w + col] = 100
        return {'version': 1, 'width': w, 'height': h, 'resolution': res,
                'origin': {'x': -1.0, 'y': -3.0, 'yaw': 0.0}, 'data': data}
