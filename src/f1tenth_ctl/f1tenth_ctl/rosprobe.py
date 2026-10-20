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
                elif kind == 'estop_clear':
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
            elif kind == 'estop_clear':
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
    from ackermann_msgs.msg import AckermannDriveStamped
    from rclpy.executors import SingleThreadedExecutor
    from std_srvs.srv import Trigger

    rclpy.init()
    t0 = time.monotonic()
    node = _node('f1tenth_ctl_estop', spec.get('use_sim_time', False))
    pub = node.create_publisher(AckermannDriveStamped, spec['topic'], _qos(True, False, 10))
    client = node.create_client(Trigger, spec['service'])
    period = 1.0 / float(spec.get('publish_hz', 50.0))
    publish_s = float(spec.get('publish_s', 2.0))
    call_timeout = float(spec.get('service_timeout_s', 0.5))
    match_grace = float(spec.get('match_grace_s', 3.0))

    msg = AckermannDriveStamped()   # every drive field 0.0: zero speed, zero steering
    msg.header.frame_id = 'base_link/f1tenth_ctl_estop'
    pubs = {'count': 0, 'after_match': 0, 'subscribers': 0, 'first_match': None}
    svc = {'state': 'waiting', 'future': None, 'sent_at': None}
    svc_out = {'service': spec['service']}
    end_publish = t0 + match_grace + publish_s
    next_pub = t0

    ex = SingleThreadedExecutor()
    ex.add_node(node)
    while True:
        now = time.monotonic()
        if now >= end_publish and svc['state'] == 'done':
            break
        if now >= next_pub and now < end_publish:
            msg.header.stamp = node.get_clock().now().to_msg()
            pub.publish(msg)
            pubs['count'] += 1
            if pubs['first_match'] is not None:
                pubs['after_match'] += 1
            next_pub = max(next_pub + period, now)
        subs = pub.get_subscription_count()
        pubs['subscribers'] = max(pubs['subscribers'], subs)
        if pubs['first_match'] is None and subs > 0:
            pubs['first_match'] = now
            end_publish = now + publish_s
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
        ex.spin_once(timeout_sec=min(0.002, max(0.0, next_pub - time.monotonic())))
    ex.remove_node(node)

    first = pubs['first_match']
    publish_out = {
        'topic': spec['topic'], 'count': pubs['count'], 'subscribers': pubs['subscribers'],
        'matched_after_s': None if first is None else round(first - t0, 3),
        'after_match': pubs['after_match'],
        'rate_hz': (round(pubs['after_match'] / publish_s, 1) if first is not None else 0.0),
        'duration_s': round(time.monotonic() - t0, 2),
        'ok': first is not None and pubs['after_match'] > 0,
    }
    if first is None:
        publish_out['error'] = 'no subscriber matched (is ackermann_mux up?)'
    node.destroy_node()
    _shutdown()
    return {'service': svc_out, 'publish': publish_out}


def main(argv=None) -> int:
    argv = sys.argv[1:] if argv is None else argv
    if not argv or argv[0] not in ('probe', 'estop', 'monitor'):
        print('usage: F1T_PROBE_SPEC=<json> python3 -m f1tenth_ctl.rosprobe '
              'probe|estop|monitor', file=sys.stderr)
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
        result = probe(spec) if argv[0] == 'probe' else estop(spec)
    except Exception as exc:  # noqa: BLE001 - always answer in JSON
        result = {'error': f'{type(exc).__name__}: {exc}'}
    sys.stdout.write(json.dumps(result) + '\n')
    sys.stdout.flush()
    return 0


if __name__ == '__main__':
    sys.exit(main())
