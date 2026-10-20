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


def estop(spec: dict) -> dict:
    """
    Zero drive on the top mux lane and the e-stop service, at the same time.

    Publishing runs for publish_s counted from the moment the first
    subscriber (the mux) matched, not from start-up: on a loaded machine
    matching took over a second (2026-10-20), and zeros sent before it reach
    nobody. The service gets service_timeout_s to answer; a later answer
    inside the publish window is still reported, marked late, because the
    latch has engaged all the same.
    """
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from std_srvs.srv import Trigger

    rclpy.init()
    t0 = time.monotonic()
    node = _node('f1tenth_ctl_estop', spec.get('use_sim_time', False))
    zero = ZeroDrive(node, spec['topic'], float(spec.get('publish_hz', 50.0)))
    client = node.create_client(Trigger, spec['service'])
    publish_s = float(spec.get('publish_s', 2.0))
    call_timeout = float(spec.get('service_timeout_s', 0.5))
    match_grace = float(spec.get('match_grace_s', 3.0))
    svc = {'state': 'waiting', 'future': None, 'sent_at': None}
    svc_out = {'service': spec['service']}
    end_publish = t0 + match_grace + publish_s

    ex = SingleThreadedExecutor()
    ex.add_node(node)
    while True:
        now = time.monotonic()
        if now >= end_publish and svc['state'] == 'done':
            break
        if now < end_publish:
            matched = zero.first_match is not None
            zero.tick(now)
            if not matched and zero.first_match is not None:
                end_publish = zero.first_match + publish_s
        if svc['state'] == 'waiting':
            if client.service_is_ready():
                svc_out['found_after_s'] = round(now - t0, 3)
                svc['future'] = client.call_async(Trigger.Request())
                svc['sent_at'] = now
                svc['state'] = 'sent'
            elif now >= end_publish:
                svc['state'] = 'done'
                svc_out.update(ok=False, error='service not found')
        elif svc['state'] == 'sent':
            fut = svc['future']
            waited = now - svc['sent_at']
            if fut.done():
                svc['state'] = 'done'
                resp = fut.result()
                svc_out['call_s'] = round(waited, 3)
                svc_out['within_timeout'] = waited <= call_timeout
                if resp is None:
                    svc_out.update(ok=False, error=str(fut.exception()))
                else:
                    svc_out.update(ok=bool(resp.success), message=resp.message)
                    if waited > call_timeout:
                        svc_out['late'] = True
            elif waited > call_timeout and now >= end_publish:
                svc['state'] = 'done'
                svc_out.update(ok=False, error=f'no response ({waited:.1f} s; timeout '
                                               f'{call_timeout:g} s)')
        ex.spin_once(timeout_sec=0.002)
    publish_out = zero.report(time.monotonic())
    publish_out['rate_hz'] = (round(zero.after_match / publish_s, 1)
                              if zero.first_match is not None else 0.0)
    if not publish_out['ok']:
        publish_out['error'] = 'no subscriber matched (is ackermann_mux up?)'
    _close(ex, node)
    return {'service': svc_out, 'publish': publish_out}


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

    def report(self, now: float) -> dict:
        span = now - self._t0 if self._t0 is not None else 0.0
        return {'topic': self.topic, 'count': self.count, 'after_match': self.after_match,
                'subscribers': self.subscribers,
                'matched_after_s': (None if self.first_match is None
                                    else round(self.first_match - self._t0, 3)),
                'duration_s': round(span, 2),
                'ok': self.first_match is not None and self.after_match > 0}


def _status_sub(node, spec, store):
    from rosidl_runtime_py.utilities import get_message

    def cb(msg):
        store.append((time.monotonic(), bool(msg.emergency_stop_active), msg.state))
    node.create_subscription(get_message(spec.get('status_type',
                                                  'f1tenth_messages/msg/MissionStatus')),
                             spec.get('status_topic', '/mission/status'), cb,
                             _qos(True, True, 1))


def _call(ex, client, request, timeout: float, busy=None) -> dict:
    """Call a service, spinning (and running `busy`) until the answer or the timeout."""
    t0 = time.monotonic()
    fut = client.call_async(request)
    while time.monotonic() - t0 < timeout and not fut.done():
        ex.spin_once(timeout_sec=0.005)
        if busy:
            busy(time.monotonic())
    took = round(time.monotonic() - t0, 3)
    if not fut.done():
        return {'ok': False, 'error': f'no answer within {timeout:g} s', 'call_s': took}
    resp = fut.result()
    if resp is None:
        return {'ok': False, 'error': str(fut.exception()), 'call_s': took}
    return {'ok': bool(resp.success), 'message': resp.message, 'call_s': took}


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
    out['abort'] = (_call(ex, abort_cli, Trigger.Request(), 3.0, zero.tick)
                    if abort_cli.service_is_ready() else {'ok': False, 'error': 'not available'})
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


def _close(ex, node):
    ex.remove_node(node)
    node.destroy_node()
    _shutdown()


def main(argv=None) -> int:
    argv = sys.argv[1:] if argv is None else argv
    modes = {'probe': probe, 'estop': estop, 'estop_reset': estop_reset,
             'estop_wait': estop_wait}
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
