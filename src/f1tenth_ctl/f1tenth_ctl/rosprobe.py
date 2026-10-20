"""
The only part of f1tenth_ctl that imports rclpy.

Run as `python3 -m f1tenth_ctl.rosprobe <mode>` in the environment the env
file produced. The spec arrives as JSON in F1T_PROBE_SPEC (or on stdin).
Everything else in f1tenth_ctl is stdlib and runs without ROS sourced.

monitor: the READY measurements, kept up to date. A long-running node (unit
         f1t-monitor) that subscribes to every topic the READY checks name
         and rewrites a JSON snapshot every 0.5 s: rates over a sliding
         window, the TF age, service availability, the mission status, the
         battery and the node list. `status` reads that file. A one-shot
         probe cannot do this reliably: on a loaded machine a fresh
         participant needs seconds to match, longer than the 5 s status
         budget leaves it (2026-10-20, Gazebo + stack on linus).
probe:   the same measurements once, printed as one JSON line (fallback
         while the monitor is not up, and for the graph check of `start`).
estop:   calls the e-stop service and, at the same time, publishes zero
         speed and steering on the mux's highest-priority input for a fixed
         time. Each part reports for itself. Nothing is stopped or killed.
estop_reset: clears the latch (see estop_reset() for the sequence) and
         checks on /mission/status that it is clear.
estop_wait: waits until /mission/status says the latch is clear (after a
         full stack restart, which the CLI does itself).
mission: validate / load / start / status of a stored mission (missions.py)
         through the mission loader's own services and its validator.
"""

from collections import deque
import json
import os
import sys
import time


def _qos(reliable: bool, transient: bool, depth: int):
    from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST, depth=depth,
        reliability=ReliabilityPolicy.RELIABLE if reliable else ReliabilityPolicy.BEST_EFFORT,
        durability=(DurabilityPolicy.TRANSIENT_LOCAL if transient
                    else DurabilityPolicy.VOLATILE))


def _node(prefix: str, use_sim_time: bool):
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    overrides = [Parameter('use_sim_time', Parameter.Type.BOOL, bool(use_sim_time))]
    return Node(f'{prefix}_{os.getpid()}', parameter_overrides=overrides,
                start_parameter_services=False)


def _shutdown():
    import rclpy
    if rclpy.ok():
        rclpy.shutdown()


class Meter:
    """Subscriptions for the READY checks, and their measurements."""

    def __init__(self, node, checks):
        from rosidl_runtime_py.utilities import get_message, get_service
        self.node = node
        self.checks = checks
        self.errors = {}
        self.stamps = {}
        self.latest = {}
        self.clients = {}
        self.tf_buffer = None
        for check in checks:
            name, kind = check['name'], check['kind']
            try:
                if kind == 'topic_rate':
                    self.stamps[name] = deque(maxlen=4000)
                    node.create_subscription(get_message(check['type']), check['topic'],
                                             self._counter(name), _qos(False, False, 10),
                                             raw=True)
                elif kind == 'estop':
                    node.create_subscription(get_message(check['type']), check['topic'],
                                             self._keeper(name), _qos(True, True, 1))
                elif kind == 'battery':
                    node.create_subscription(get_message(check['type']), check['topic'],
                                             self._keeper(name), _qos(False, False, 5))
                elif kind == 'service':
                    self.clients[name] = node.create_client(get_service(check['type']),
                                                            check['service'])
                elif kind == 'tf_age' and self.tf_buffer is None:
                    from tf2_ros import Buffer, TransformListener
                    self.tf_buffer = Buffer()
                    self._tf_listener = TransformListener(self.tf_buffer, node,
                                                          spin_thread=False)
            except Exception as exc:  # noqa: BLE001 - unknown type, bad name: report it
                self.errors[name] = {'error': f'{type(exc).__name__}: {exc}'}

    def _counter(self, name):
        def cb(_msg):
            self.stamps[name].append(time.monotonic())
        return cb

    def _keeper(self, name):
        def cb(msg):
            self.latest[name] = (time.monotonic(), msg)
        return cb

    def complete(self, span: float) -> bool:
        """Say whether every check already has what it needs."""
        from rclpy.time import Time
        for check in self.checks:
            name, kind = check['name'], check['kind']
            if name in self.errors:
                continue
            if kind == 'topic_rate':
                st = self.stamps[name]
                if len(st) < 2 or st[-1] - st[0] < span:
                    return False
            elif kind == 'tf_age':
                if not self.tf_buffer.can_transform(check['parent'], check['child'], Time()):
                    return False
            elif kind == 'service':
                if not self.clients[name].service_is_ready():
                    return False
            elif name not in self.latest:
                return False
        return True

    def measure(self, window: float) -> dict:
        """Measure every check; rates over the last `window` seconds."""
        now = time.monotonic()
        out = dict(self.errors)
        for check in self.checks:
            name, kind = check['name'], check['kind']
            if name in out:
                continue
            if kind == 'topic_rate':
                recent = [t for t in self.stamps[name] if now - t <= window]
                span = recent[-1] - recent[0] if len(recent) >= 2 else 0.0
                hz = (len(recent) - 1) / span if span > 0 else 0.0
                out[name] = {'count': len(recent), 'hz': hz,
                             'publishers': self.node.count_publishers(check['topic'])}
                if self.stamps[name]:
                    out[name]['last_age_s'] = round(now - self.stamps[name][-1], 3)
            elif kind == 'tf_age':
                out[name] = self._tf_age(check['parent'], check['child'])
            elif kind == 'service':
                out[name] = {'available': self.clients[name].service_is_ready()}
            elif kind == 'estop':
                got = self.latest.get(name)
                out[name] = ({'received': False} if got is None else
                             {'received': True, 'state': got[1].state,
                              'json_path': got[1].json_path,
                              'emergency_stop_active': bool(got[1].emergency_stop_active)})
            elif kind == 'battery':
                got = self.latest.get(name)
                stale = got is not None and now - got[0] > max(window, 5.0)
                out[name] = ({'received': False} if got is None or stale else
                             {'received': True, 'has_data': bool(got[1].has_data),
                              'voltage': float(got[1].voltage), 'ok': bool(got[1].ok)})
        return out

    def _tf_age(self, parent, child):
        from rclpy.time import Time
        if self.tf_buffer is None:
            return {'found': False, 'error': 'no tf listener'}
        try:
            tf = self.tf_buffer.lookup_transform(parent, child, Time())
        except Exception as exc:  # noqa: BLE001 - tf2 raises several types
            return {'found': False, 'error': f'{type(exc).__name__}: {str(exc)[:160]}'}
        now = self.node.get_clock().now()
        if now.nanoseconds == 0:
            return {'found': True, 'age_s': None, 'error': 'clock is zero (no /clock?)'}
        return {'found': True,
                'age_s': (now - Time.from_msg(tf.header.stamp)).nanoseconds / 1e9}


def _nodes(node):
    return sorted((ns.rstrip('/') + '/' + n) for n, ns in node.get_node_names_and_namespaces()
                  if not n.startswith('f1tenth_ctl_'))


def probe(spec: dict) -> dict:
    import rclpy
    from rclpy.executors import SingleThreadedExecutor

    rclpy.init()
    t_start = time.monotonic()
    node = _node('f1tenth_ctl_probe', spec.get('use_sim_time', False))
    meter = Meter(node, spec.get('checks', []))
    ex = SingleThreadedExecutor()
    ex.add_node(node)
    # Listen at least window_s; past that, stop as soon as every check has
    # its data, or at the deadline the CLI set.
    window = float(spec.get('window_s', 1.0))
    t_listen = time.monotonic()
    t_min = t_listen + window
    end = t_listen + max(window, float(spec.get('deadline', time.time() + window)) - time.time())
    next_check = t_min
    while time.monotonic() < end:
        ex.spin_once(timeout_sec=0.02)
        if time.monotonic() >= next_check:
            if meter.complete(min(window, 1.0)):
                break
            next_check = time.monotonic() + 0.1
    result = {'measured': meter.measure(time.monotonic() - t_listen + 1.0),
              'window_s': window, 'listened_s': round(time.monotonic() - t_listen, 2)}
    if spec.get('nodes', True):
        result['nodes'] = _nodes(node)
    result['elapsed_s'] = round(time.monotonic() - t_start, 2)
    ex.remove_node(node)
    node.destroy_node()
    _shutdown()
    return result


def monitor(spec: dict) -> int:
    """Rewrite the snapshot file every `period_s` until SIGINT/SIGTERM."""
    import signal

    import rclpy
    from rclpy.executors import SingleThreadedExecutor

    out_path = spec['out']
    stop = {'now': False}

    def on_signal(_sig, _frame):
        stop['now'] = True

    signal.signal(signal.SIGTERM, on_signal)
    signal.signal(signal.SIGINT, on_signal)
    rclpy.init()
    node = _node('f1tenth_ctl_monitor', spec.get('use_sim_time', False))
    meter = Meter(node, spec.get('checks', []))
    ex = SingleThreadedExecutor()
    ex.add_node(node)
    window = max(float(spec.get('window_s', 1.0)), 2.0)
    period = float(spec.get('period_s', 0.5))
    started = time.time()
    next_write = time.monotonic() + period
    while not stop['now'] and rclpy.ok():
        ex.spin_once(timeout_sec=0.02)
        if time.monotonic() < next_write:
            continue
        next_write = time.monotonic() + period
        snap = {'time': time.time(), 'started': started, 'pid': os.getpid(),
                'spec_id': spec.get('spec_id'), 'window_s': window,
                'measured': meter.measure(window), 'nodes': _nodes(node)}
        tmp = out_path + '.tmp'
        with open(tmp, 'w') as f:
            json.dump(snap, f)
        os.replace(tmp, out_path)
    ex.remove_node(node)
    node.destroy_node()
    _shutdown()
    return 0


def step_record(status: str, message: str, duration_s: float, **extra) -> dict:
    """One step of estop / estop-reset: status ok | failed | skipped."""
    return {'status': status, 'ok': status == 'ok', 'message': message,
            'duration_ms': int(round(max(0.0, duration_s) * 1000)), **extra}


ABORT_NOTHING = 'no mission to abort'


def abort_ok(message: str) -> bool:
    """`no mission to abort (state=...)` means nothing can resume: a success."""
    return message.startswith(ABORT_NOTHING)


class ServiceStep:
    """
    One service call inside estop, polled so it never blocks the others.

    The call_timeout counts from the request. Finding the server runs inside
    the zero-drive window, so it delays nothing; an answer that comes after
    call_timeout but inside the window is still taken, marked late.
    """

    def __init__(self, node, srv_type, name: str, call_timeout: float, t0: float,
                 ok_if=None):
        self.srv_type = srv_type
        self.name = name
        self.call_timeout = call_timeout
        self.t0 = t0
        self.ok_if = ok_if
        self.client = node.create_client(srv_type, name) if name else None
        self.future = None
        self.sent_at = None
        self.record = None
        if self.client is None:
            self.record = step_record('skipped', 'no service configured', 0.0, service=name)

    @property
    def done(self) -> bool:
        return self.record is not None

    def poll(self, now: float, window_end: float) -> None:
        if self.done:
            return
        if self.future is None:
            if self.client.service_is_ready():
                self.sent_at = now
                self.future = self.client.call_async(self.srv_type.Request())
            elif now >= window_end:
                self.record = step_record('failed', 'service not available', now - self.t0,
                                          service=self.name)
            return
        waited = now - self.sent_at
        if self.future.done():
            resp = self.future.result()
            extra = {'service': self.name, 'call_ms': int(round(waited * 1000))}
            if resp is None:
                self.record = step_record('failed', str(self.future.exception()),
                                          now - self.t0, **extra)
                return
            ok = bool(resp.success) or bool(self.ok_if and self.ok_if(resp.message))
            if waited > self.call_timeout:
                extra['late'] = True
            self.record = step_record('ok' if ok else 'failed', resp.message, now - self.t0,
                                      **extra)
        elif waited > self.call_timeout and now >= window_end:
            self.record = step_record('failed', f'no answer within {waited:.1f} s '
                                      f'(timeout {self.call_timeout:g} s)', now - self.t0,
                                      service=self.name)

    def give_up(self, now: float) -> None:
        if not self.done:
            self.record = step_record('failed', 'gave up', now - self.t0, service=self.name)


def estop(spec: dict) -> dict:
    """
    Hold the car: three steps at once, each best effort, none waits for another.

    a) zero_drive: zero speed and steering on the mux's top lane (/safety_stop,
       priority 200, 0.2 s timeout) at publish_hz, starting at once, for
       publish_s counted from the moment the mux matched;
    b) emergency_stop: /mission/emergency_stop, which latches the behaviour
       tree's own zero on the same lane (service_timeout_s, 0.5 s);
    c) abort_mission: /mission/abort_mission, which ends a LOADED/RUNNING
       mission and puts mpc_corr on /mpc/hold (abort_timeout_s, 1 s).
       "No mission to abort" is a success.
    car_held = a) published AND (b) ok OR c) ok).
    """
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from std_srvs.srv import Trigger

    rclpy.init()
    t0 = time.monotonic()
    node = _node('f1tenth_ctl_estop', spec.get('use_sim_time', False))
    zero = ZeroDrive(node, spec['topic'], float(spec.get('publish_hz', 50.0)))
    steps = {
        'emergency_stop': ServiceStep(node, Trigger, spec.get('service', ''),
                                      float(spec.get('service_timeout_s', 0.5)), t0),
        'abort_mission': ServiceStep(node, Trigger, spec.get('abort_service', ''),
                                     float(spec.get('abort_timeout_s', 1.0)), t0,
                                     ok_if=abort_ok),
    }
    publish_s = float(spec.get('publish_s', 2.0))
    match_grace = float(spec.get('match_grace_s', 3.0))
    window_end = t0 + match_grace + publish_s
    hard_end = window_end + 2.0

    ex = SingleThreadedExecutor()
    ex.add_node(node)
    while True:
        now = time.monotonic()
        if (now >= window_end and all(st.done for st in steps.values())) or now >= hard_end:
            break
        if now < window_end:
            matched = zero.first_match is not None
            zero.tick(now)
            if not matched and zero.first_match is not None:
                window_end = zero.first_match + publish_s
                hard_end = window_end + 2.0
        for st in steps.values():
            st.poll(now, window_end)
        ex.spin_once(timeout_sec=0.002)
    now = time.monotonic()
    for st in steps.values():
        st.give_up(now)
    out = {'zero_drive': zero.report(now, publish_s)}
    out.update({name: st.record for name, st in steps.items()})
    car_held = out['zero_drive']['ok'] and (out['emergency_stop']['ok']
                                            or out['abort_mission']['ok'])
    _close(ex, node)
    return {'steps': out, 'car_held': bool(car_held)}


def choose_reset_path(reset_service: str, reset_ready: bool, restart_ready: bool) -> str:
    """
    Pick how to clear the e-stop latch: the cleanest path that is there.

    1. reset_service: a service that clears the latch in place. None exists
       today (the latch is documented as cleared only by a restart); the
       name is configurable for when one does.
    2. restart_behavior: the supervisor restarts the 'behavior' component
       only (/restart_component), which is the documented way to clear it.
    3. stack_restart: neither is available (no supervisor, e.g.
       stack_bringup.launch.py); the CLI restarts the whole stack.
    """
    if reset_service and reset_ready:
        return 'reset_service'
    if restart_ready:
        return 'restart_behavior'
    return 'stack_restart'


class ZeroDrive:
    """Zero speed and steering on the mux's top lane, at a fixed rate."""

    def __init__(self, node, topic: str, hz: float):
        from ackermann_msgs.msg import AckermannDriveStamped
        self.node = node
        self.topic = topic
        self.pub = node.create_publisher(AckermannDriveStamped, topic, _qos(True, False, 10))
        self.msg = AckermannDriveStamped()   # every drive field 0.0
        self.msg.header.frame_id = 'base_link/f1tenth_ctl_estop'
        self.period = 1.0 / hz
        self.count = 0
        self.after_match = 0
        self.subscribers = 0
        self.first_match = None
        self._next = None
        self._t0 = None

    def tick(self, now: float) -> None:
        if self._t0 is None:
            self._t0 = self._next = now
        if now >= self._next:
            self.msg.header.stamp = self.node.get_clock().now().to_msg()
            self.pub.publish(self.msg)
            self.count += 1
            if self.first_match is not None:
                self.after_match += 1
            self._next = max(self._next + self.period, now)
        subs = self.pub.get_subscription_count()
        self.subscribers = max(self.subscribers, subs)
        if self.first_match is None and subs > 0:
            self.first_match = now

    def report(self, now: float, window_s: float = None) -> dict:
        span = now - self._t0 if self._t0 is not None else 0.0
        ok = self.first_match is not None and self.after_match > 0
        extra = {'topic': self.topic, 'count': self.count, 'after_match': self.after_match,
                 'subscribers': self.subscribers,
                 'matched_after_ms': (None if self.first_match is None
                                      else int(round((self.first_match - self._t0) * 1000)))}
        if ok and window_s:
            extra['rate_hz'] = round(self.after_match / window_s, 1)
        message = (f'{self.after_match} msgs to {self.subscribers} subscriber(s)' if ok
                   else 'no subscriber matched (is ackermann_mux up?)')
        return step_record('ok' if ok else 'failed', message, span, **extra)


def _status_sub(node, spec, store):
    from rosidl_runtime_py.utilities import get_message

    def cb(msg):
        store.append((time.monotonic(), bool(msg.emergency_stop_active), msg.state,
                      msg.json_path))
    node.create_subscription(get_message(spec.get('status_type',
                                                  'f1tenth_messages/msg/MissionStatus')),
                             spec.get('status_topic', '/mission/status'), cb,
                             _qos(True, True, 1))


def _call(ex, client, request, timeout: float, busy=None, ok_if=None) -> dict:
    """Call a service, spinning (and running `busy`) until the answer or the timeout."""
    t0 = time.monotonic()
    fut = client.call_async(request)
    while time.monotonic() - t0 < timeout and not fut.done():
        ex.spin_once(timeout_sec=0.005)
        if busy:
            busy(time.monotonic())
    took = time.monotonic() - t0
    if not fut.done():
        return step_record('failed', f'no answer within {timeout:g} s', took)
    resp = fut.result()
    if resp is None:
        return step_record('failed', str(fut.exception()), took)
    ok = bool(resp.success) or bool(ok_if and ok_if(resp.message))
    return step_record('ok' if ok else 'failed', resp.message, took)


def estop_reset(spec: dict) -> dict:
    """
    Clear the e-stop latch, without the car moving while it happens.

    Restarting 'behavior' removes the behaviour tree whose emergency lane
    holds the mux at zero, and the e-stop does not touch the mission or
    /mpc/hold: mpc_corr may still hold a move's goal and a RUNNING mission's
    /drive commands would pass the mux the moment the lane goes quiet. So:

    1. read the latch on /mission/status (nothing to do if it is clear);
    2. publish zero drive on the top lane, from now until the end, and go on
       only once the mux has matched it;
    3. /mission/abort_mission: a RUNNING or LOADED mission is aborted and
       mpc_corr is put on /mpc/hold, so nothing resumes afterwards (it
       answers "no mission to abort" when there is none, which is fine);
    4. the reset itself (choose_reset_path);
    5. wait for /mission/status with the latch clear, from the new node.
    """
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from rosidl_runtime_py.utilities import get_service
    from std_srvs.srv import Trigger

    rclpy.init()
    t0 = time.monotonic()
    node = _node('f1tenth_ctl_estop_reset', spec.get('use_sim_time', False))
    ex = SingleThreadedExecutor()
    ex.add_node(node)
    statuses = []
    _status_sub(node, spec, statuses)
    reset_name = spec.get('reset_service') or ''
    reset_cli = node.create_client(Trigger, reset_name) if reset_name else None
    restart_type = get_service('f1tenth_messages/srv/RestartComponent')
    restart_cli = node.create_client(restart_type, spec.get('restart_service',
                                                            '/restart_component'))
    abort_cli = node.create_client(Trigger, spec.get('abort_service', '/mission/abort_mission'))
    out = {}

    # 1. what is the latch now? Services get the same few seconds to appear.
    discover_end = t0 + float(spec.get('discover_s', 4.0))
    while time.monotonic() < discover_end:
        ex.spin_once(timeout_sec=0.02)
        if statuses and restart_cli.service_is_ready() and abort_cli.service_is_ready() and (
                reset_cli is None or reset_cli.service_is_ready()):
            break
    out['latched_before'] = statuses[-1][1] if statuses else None
    if out['latched_before'] is None:
        out.update(ok=False, path=None, error='no /mission/status: is behavior running?')
    elif out['latched_before'] is False and not spec.get('force'):
        out.update(ok=True, path='none', latched_after=False, verified=True,
                   message='the e-stop is not latched: nothing to do')
    if 'ok' in out:
        _close(ex, node)
        return out
    path = choose_reset_path(reset_name, bool(reset_cli and reset_cli.service_is_ready()),
                             restart_cli.service_is_ready())
    out['path'] = path
    if path == 'stack_restart':
        out.update(ok=False, needs='stack_restart',
                   error='no reset service and no supervisor /restart_component')
        _close(ex, node)
        return out

    # 2. zero drive from here to the end. Nothing is reset until the mux is
    #    listening to it: the old tree's own zero ends with its process, and an
    #    unmatched publisher would leave the top lane empty right then.
    zero = ZeroDrive(node, spec.get('safety_topic', '/safety_stop'),
                     float(spec.get('publish_hz', 50.0)))
    match_end = time.monotonic() + float(spec.get('match_timeout_s', 5.0))
    while time.monotonic() < match_end and (zero.first_match is None or zero.after_match < 5):
        ex.spin_once(timeout_sec=0.005)
        zero.tick(time.monotonic())
    if zero.first_match is None:
        out.update(ok=False, verified=False, latched_after=True,
                   zero_drive=zero.report(time.monotonic()),
                   error='the mux never matched the zero drive: nothing was reset')
        _close(ex, node)
        return out
    # 3. abort the interrupted mission: nothing may resume after the reset
    out['abort_mission'] = (
        _call(ex, abort_cli, Trigger.Request(), 3.0, zero.tick, ok_if=abort_ok)
        if abort_cli.service_is_ready() else step_record('failed', 'service not available', 0.0))
    # 4. the reset
    t_action = time.monotonic()
    if path == 'reset_service':
        out['reset'] = _call(ex, reset_cli, Trigger.Request(), 5.0, zero.tick)
    else:
        req = restart_type.Request()
        req.component_name = spec.get('component', 'behavior')
        out['restart'] = _call(ex, restart_cli, req,
                               float(spec.get('restart_timeout_s', 30.0)), zero.tick)
    # 5. the new state, as published after the action
    wait_end = time.monotonic() + float(spec.get('wait_s', 60.0))
    cleared_at = None
    while time.monotonic() < wait_end:
        ex.spin_once(timeout_sec=0.005)
        zero.tick(time.monotonic())
        fresh = [s for s in statuses if s[0] >= t_action]
        if fresh and fresh[-1][1] is False:
            cleared_at = fresh[-1][0]
            break
    # A little longer, so the mux never sees the top lane go quiet while the
    # new tree is still settling.
    hold_end = time.monotonic() + float(spec.get('hold_after_s', 0.5))
    while time.monotonic() < hold_end:
        ex.spin_once(timeout_sec=0.005)
        zero.tick(time.monotonic())
    now = time.monotonic()
    fresh = [s for s in statuses if s[0] >= t_action]
    out['latched_after'] = fresh[-1][1] if fresh else None
    out['mission_state_after'] = fresh[-1][2] if fresh else None
    out['verified'] = cleared_at is not None
    out['cleared_after_s'] = None if cleared_at is None else round(cleared_at - t_action, 2)
    out['zero_drive'] = zero.report(now)
    out['ok'] = out['verified']
    if not out['verified']:
        out['error'] = ('the latch is still set' if out['latched_after']
                        else 'no /mission/status from the new behavior within '
                             f"{spec.get('wait_s', 60.0)} s")
    _close(ex, node)
    return out


def estop_wait(spec: dict) -> dict:
    """Wait until /mission/status says the e-stop is not latched."""
    import rclpy
    from rclpy.executors import SingleThreadedExecutor

    rclpy.init()
    t0 = time.monotonic()
    node = _node('f1tenth_ctl_estop_wait', spec.get('use_sim_time', False))
    ex = SingleThreadedExecutor()
    ex.add_node(node)
    statuses = []
    _status_sub(node, spec, statuses)
    end = t0 + float(spec.get('wait_s', 120.0))
    while time.monotonic() < end:
        ex.spin_once(timeout_sec=0.05)
        if statuses and statuses[-1][1] is False:
            break
    latched = statuses[-1][1] if statuses else None
    out = {'latched_after': latched, 'verified': latched is False,
           'waited_s': round(time.monotonic() - t0, 2),
           'mission_state_after': statuses[-1][2] if statuses else None}
    _close(ex, node)
    return out


def validate_mission_file(path: str) -> dict:
    """Validate with the stack's own parser, the one /mission/load_mission uses."""
    try:
        from f1tenth_behavior.mission.mission_config import load_mission_file
    except ImportError as exc:
        return step_record('failed', f'validator unavailable: {exc}', 0.0)
    t0 = time.monotonic()
    try:
        cfg = load_mission_file(path)
    except Exception as exc:  # noqa: BLE001 - MissionConfigError, OSError, ValueError, ...
        return step_record('failed', f'invalid mission: {type(exc).__name__}: {exc}',
                           time.monotonic() - t0)
    return step_record('ok', f'valid: {cfg.mission_id}, {len(cfg.moves)} move(s)',
                       time.monotonic() - t0)


def mission(spec: dict) -> dict:
    """
    One mission operation: validate | load | start | status.

    load refuses an invalid file, a RUNNING/HOLDING mission (the loader would
    replace it mid-run) and a start that is counting down. start refuses a
    latched e-stop, a running mission, nothing loaded, or a loaded mission
    other than the expected file. Each answer carries the mission status
    as read before the operation.
    """
    op = spec.get('op')
    if op == 'validate':
        return {'validate': validate_mission_file(spec['path'])}

    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from rosidl_runtime_py.utilities import get_service
    from std_srvs.srv import Trigger

    rclpy.init()
    node = _node('f1tenth_ctl_mission', spec.get('use_sim_time', False))
    ex = SingleThreadedExecutor()
    ex.add_node(node)
    statuses = []
    _status_sub(node, spec, statuses)
    load_type = get_service('f1tenth_messages/srv/LoadMission')
    load_cli = node.create_client(load_type, spec.get('load_service', '/mission/load_mission'))
    start_cli = node.create_client(Trigger, spec.get('start_service', '/mission/start_mission'))
    end = time.monotonic() + float(spec.get('discover_s', 3.0))
    while time.monotonic() < end:
        ex.spin_once(timeout_sec=0.02)
        ready = {'load': load_cli, 'start': start_cli}.get(op)
        if statuses and (ready is None or ready.service_is_ready()):
            break
    out = {}
    if statuses:
        _, latched, state, json_path = statuses[-1]
        out.update(ros_state=state, json_path=json_path, emergency_stop_active=latched)
    if op == 'status':
        out['ok'] = bool(statuses)
        if not statuses:
            out['error'] = 'no /mission/status: is behavior running?'
        _close(ex, node)
        return out

    def refuse(why):
        out.update(ok=False, error=why)
        _close(ex, node)
        return out

    if not statuses:
        return refuse('no /mission/status: is behavior running?')
    state = out['ros_state']
    if state in ('RUNNING', 'HOLDING'):
        return refuse(f'another mission is running ({state})')
    if op == 'load':
        out['validate'] = validate_mission_file(spec['path'])
        if not out['validate']['ok']:
            return refuse(out['validate']['message'])
        if spec.get('counting_down'):
            return refuse('a mission start is counting down')
        if not load_cli.service_is_ready():
            return refuse('/mission/load_mission not available')
        req = load_type.Request()
        req.path = spec['path']
        out['load'] = _call(ex, load_cli, req, 5.0)
        target = spec['path']
    elif op == 'start':
        if out['emergency_stop_active']:
            return refuse('the e-stop is latched')
        if state != 'LOADED':
            return refuse(f'nothing loaded (mission state {state})')
        if spec.get('path') and out['json_path'] != spec['path']:
            return refuse(f"the loaded mission is not this one ({out['json_path']})")
        if not start_cli.service_is_ready():
            return refuse('/mission/start_mission not available')
        out['start'] = _call(ex, start_cli, Trigger.Request(), 5.0)
        target = None
    else:
        return refuse(f'unknown op {op!r}')
    step = out.get('load') or out.get('start')
    out['ok'] = bool(step and step['ok'])
    if not out['ok']:
        out['error'] = step['message'] if step else 'failed'
    # The status as published after the call (a load answers with the new path).
    t_after = time.monotonic()
    end = t_after + 2.0
    while time.monotonic() < end:
        ex.spin_once(timeout_sec=0.02)
        if statuses[-1][0] >= t_after - 0.5 and (target is None or statuses[-1][3] == target):
            break
    _, latched, state, json_path = statuses[-1]
    out['after'] = {'ros_state': state, 'json_path': json_path,
                    'emergency_stop_active': latched}
    _close(ex, node)
    return out


def _close(ex, node):
    ex.remove_node(node)
    node.destroy_node()
    _shutdown()


def main(argv=None) -> int:
    argv = sys.argv[1:] if argv is None else argv
    modes = {'probe': probe, 'estop': estop, 'estop_reset': estop_reset,
             'estop_wait': estop_wait, 'mission': mission}
    if not argv or argv[0] not in tuple(modes) + ('monitor',):
        print('usage: F1T_PROBE_SPEC=<json> python3 -m f1tenth_ctl.rosprobe '
              + '|'.join(tuple(modes) + ('monitor',)), file=sys.stderr)
        return 2
    if '--spec-file' in argv:
        with open(argv[argv.index('--spec-file') + 1]) as f:
            spec = json.load(f)
    else:
        raw = os.environ.get('F1T_PROBE_SPEC')
        spec = json.loads(raw if raw is not None else (sys.stdin.read() or '{}'))
    if argv[0] == 'monitor':
        return monitor(spec)
    try:
        result = modes[argv[0]](spec)
    except Exception as exc:  # noqa: BLE001 - always answer in JSON
        result = {'error': f'{type(exc).__name__}: {exc}'}
    sys.stdout.write(json.dumps(result) + '\n')
    sys.stdout.flush()
    return 0


if __name__ == '__main__':
    sys.exit(main())
