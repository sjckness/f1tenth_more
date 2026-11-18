"""The Python side of the hard-stop parity protocol (f1tenth_hard_stop_rs::replay).

PySession.run(op) answers the same JSON ops as `hard_stop_replay`, with the
real code:
  - "fn" ops call f1tenth_control.hard_stop's functions and HardStopState;
  - "node" ops build a real HardStopNode (rclpy, never spun) with the op's
    parameters; "scan" / "republish" / "lane" call its _scan_cb / _republish /
    _lane_cb. Only its edges are replaced: _now (the op's clock values, counted),
    get_logger (captured lines), the two publishers' publish (captured), and the
    module's trigger_range_hits is wrapped to report the hits it returned.

Outputs are normalised through JSON exactly as the Rust side writes them
(non-finite floats as "nan" / "inf" / "-inf"), and same() compares floats by
bit pattern.

Imported by parity_hard_stop.py and make_test_cases.py; needs the workspace
sourced (f1tenth_control on PYTHONPATH) and rclpy.init() done by the caller.
"""

import json
import math
import struct

from ackermann_msgs.msg import AckermannDriveStamped
import f1tenth_control.hard_stop as hs
import f1tenth_control.hard_stop_node as hsn
from rclpy.parameter import Parameter
from sensor_msgs.msg import LaserScan

LANE_NAMES = [lane for lane, _prio in hs.LOWER_LANES]

#: ROS parameter name -> Python type of HardStopNode's declaration.
PARAM_TYPES = {
    'hard_stop_enabled': bool, 'hard_stop_range_m': float,
    'hard_stop_debounce_scans': int, 'hard_stop_clear_s': float,
    'hard_stop_republish_hz': float, 'scan_topic': str, 'stop_topic': str,
    'diag_topic': str, 'footprint_front_x_m': float, 'footprint_rear_x_m': float,
    'footprint_half_width_m': float, 'footprint_self_margin_m': float,
    'laser_x_m': float, 'hard_stop_allow_reverse': bool, 'lane_timeout_s': float,
}


# ---------------------------------------------------------------- encoding
def enc_float(x):
    x = float(x)
    if math.isnan(x):
        return 'nan'
    if math.isinf(x):
        return 'inf' if x > 0 else '-inf'
    return x


def dec_float(v):
    if isinstance(v, str):
        return {'nan': math.nan, 'inf': math.inf, '-inf': -math.inf}[v]
    if isinstance(v, bool) or not isinstance(v, (int, float)):
        raise TypeError(f'{v!r} is not a float')
    return float(v)


def enc(obj):
    """JSON-ready: floats via enc_float, tuples as lists."""
    if isinstance(obj, bool) or obj is None or isinstance(obj, (int, str)):
        return obj
    if isinstance(obj, float):
        return enc_float(obj)
    if isinstance(obj, dict):
        return {k: enc(v) for k, v in obj.items()}
    if isinstance(obj, (list, tuple)):
        return [enc(v) for v in obj]
    raise TypeError(f'cannot encode {type(obj)}')


def normalise(obj):
    """What the other side reads back: through JSON text."""
    return json.loads(json.dumps(enc(obj)))


def _bits(x):
    return struct.pack('<d', x)


def same(a, b):
    """Bit-exact equality of two normalised outputs (-0.0 is not 0.0)."""
    if isinstance(a, bool) or isinstance(b, bool):
        return type(a) is type(b) and a == b
    if isinstance(a, float) or isinstance(b, float):
        return type(a) is type(b) and _bits(a) == _bits(b)
    if isinstance(a, list) and isinstance(b, list):
        return len(a) == len(b) and all(same(x, y) for x, y in zip(a, b))
    if isinstance(a, dict) and isinstance(b, dict):
        return a.keys() == b.keys() and all(same(a[k], b[k]) for k in a)
    return type(a) is type(b) and a == b


def first_difference(a, b, path='$'):
    """Path and the two values of the first difference, or None."""
    if same(a, b):
        return None
    if isinstance(a, dict) and isinstance(b, dict) and a.keys() == b.keys():
        for k in a:
            d = first_difference(a[k], b[k], f'{path}.{k}')
            if d:
                return d
    if isinstance(a, list) and isinstance(b, list) and len(a) == len(b):
        for i, (x, y) in enumerate(zip(a, b)):
            d = first_difference(x, y, f'{path}[{i}]')
            if d:
                return d
    return (path, a, b)


# ---------------------------------------------------------------- helpers
def _hit(h):
    return [h[0], h[1], h[2], h[3]]


def _poly(args):
    return tuple(dec_float(v) for v in args['polygon'])


def _floats(args, key):
    return [dec_float(v) for v in args[key]]


class _Clock:
    """Successive _now() reads return the op's values; the last one repeats."""

    def __init__(self, values):
        self.values = [dec_float(v) for v in values]
        self.reads = 0

    def __call__(self):
        v = self.values[min(self.reads, len(self.values) - 1)] if self.values else math.nan
        self.reads += 1
        return v


class _Logger:
    def __init__(self):
        self.lines = []

    def _log(self, level, msg, kw):
        t = kw.get('throttle_duration_sec')
        self.lines.append([level, str(msg), None if t is None else float(t)])

    def warn(self, msg, **kw):
        self._log('warn', msg, kw)

    warning = warn

    def info(self, msg, **kw):
        self._log('info', msg, kw)

    def debug(self, msg, **kw):
        self._log('debug', msg, kw)

    def error(self, msg, **kw):
        self._log('error', msg, kw)


class _CapturedNode(hsn.HardStopNode):
    """The real HardStopNode; only get_logger is replaced (captured lines)."""

    def __init__(self, **kwargs):
        self._capture = _Logger()
        super().__init__(**kwargs)

    def get_logger(self):
        return self._capture


_LAST_HITS = []
_ORIGINAL_TRH = hs.trigger_range_hits


def _recording_trh(*args, **kwargs):
    hits = _ORIGINAL_TRH(*args, **kwargs)
    _LAST_HITS[:] = [list(hits)]
    return hits


# The node calls the name it imported; wrap that one (results unchanged).
hsn.trigger_range_hits = _recording_trh


# ---------------------------------------------------------------- fn ops
def run_fn(name, a):
    if name == 'footprint_polygon':
        return list(hs.footprint_polygon(
            dec_float(a['front_x']), dec_float(a['rear_x']), dec_float(a['half_width']),
            margin=dec_float(a['margin'])))
    if name == 'is_self_hit':
        return hs.is_self_hit(dec_float(a['x']), dec_float(a['y']), _poly(a))
    if name == 'scan_points_base_link':
        return [_hit(h) for h in hs.scan_points_base_link(
            _floats(a, 'ranges'), dec_float(a['angle_min']), dec_float(a['angle_increment']),
            dec_float(a['range_min']), dec_float(a['range_max']), dec_float(a['laser_x']),
            dec_float(a['laser_y']))]
    if name == 'trigger_range_hits':
        return [_hit(h) for h in _ORIGINAL_TRH(
            _floats(a, 'ranges'), dec_float(a['angle_min']), dec_float(a['angle_increment']),
            dec_float(a['range_min']), dec_float(a['range_max']), dec_float(a['laser_x']),
            _poly(a), dec_float(a['trigger_range_m']))]
    if name == 'all_ahead':
        hits = [tuple(dec_float(v) for v in h) for h in a['hits']]
        return hs.all_ahead(hits, _poly(a))
    if name == 'stop_now':
        speed = None if a['speed'] is None else dec_float(a['speed'])
        return hs.stop_now(a['engaged'], a['allow_reverse'], a['ahead_only'], speed)
    if name == 'winning_speed':
        latest = {k: (dec_float(v[0]), dec_float(v[1])) for k, v in a['latest'].items()}
        return hs.winning_speed(latest, dec_float(a['now']), timeout_s=dec_float(a['timeout_s']))
    if name == 'state':
        return _run_state(a)
    if name == 'py_round':
        return round(dec_float(a['x']), a['ndigits'])
    if name == 'py_repr':
        return repr(dec_float(a['x']))
    if name == 'json_float':
        return json.dumps(dec_float(a['x']))
    if name == 'py_fixed':
        return format(dec_float(a['x']), ('+' if a['plus'] else '') + f'.{a["prec"]}f')
    if name == 'degrees':
        return math.degrees(dec_float(a['x']))
    raise ValueError(f'unknown fn {name!r}')


def _state_dict(s):
    return {'engaged': s.engaged, 'consecutive_hits': s.consecutive_hits,
            'last_hit_time': s.last_hit_time, 'trigger_count': s.trigger_count,
            'debounce_scans': s.debounce_scans, 'clear_s': s.clear_s}


def _run_state(a):
    s = hs.HardStopState(debounce_scans=a['debounce_scans'], clear_s=dec_float(a['clear_s']))
    results = []
    for op in a['ops']:
        if op[0] == 'update':
            results.append(s.update(op[1], dec_float(op[2])))
        elif op[0] == 'tick':
            results.append(s.tick(dec_float(op[1])))
        elif op[0] == 'as_dict':
            nearest = None if op[2] is None else (dec_float(op[2][0]), dec_float(op[2][1]))
            results.append(s.as_dict(n_hits=op[1], nearest=nearest))
        else:
            raise ValueError(f'unknown state op {op[0]!r}')
    return {'results': results, 'final': _state_dict(s)}


# ---------------------------------------------------------------- the session
class PySession:
    """One replay session against the real Python code."""

    def __init__(self):
        self.node = None
        self.stops = 0
        self.diags = []

    def close(self):
        if self.node is not None:
            self.node.destroy_node()
            self.node = None

    def _core_state(self):
        n = self.node
        d = _state_dict(n.state)
        d['ahead_only'] = n.ahead_only
        d['lane_latest'] = {lane: (list(n.lane_latest[lane]) if lane in n.lane_latest else None)
                            for lane in LANE_NAMES}
        return d

    def _start(self, params):
        self.close()
        overrides = []
        for k, v in params.items():
            t = PARAM_TYPES[k]
            value = dec_float(v) if t is float else v
            if type(value) is not t:
                raise TypeError(f'{k}: {v!r} is not {t.__name__}')
            overrides.append(Parameter(k, value=value))
        node = _CapturedNode(parameter_overrides=overrides)
        self.node = node
        node.stop_pub.publish = self._on_stop
        node.diag_pub.publish = self._on_diag
        subs = sorted(s.topic_name for s in node.subscriptions)
        startup = [line[1] for line in node._capture.lines if line[1].startswith('HARD STOP')]
        node._capture.lines.clear()
        return {'started': True, 'subscriptions': subs,
                'republish_period_ns': node.republish_timer.timer_period_ns,
                'startup': startup[0] if startup else None,
                'stop_frame_id': hsn.HARD_STOP_FRAME_ID, 'state': self._core_state()}

    def _on_stop(self, msg):
        assert msg.drive.speed == 0.0 and msg.drive.steering_angle == 0.0
        assert msg.header.frame_id == hsn.HARD_STOP_FRAME_ID
        self.stops += 1

    def _on_diag(self, msg):
        self.diags.append(msg.data)

    def _callback(self, op, call):
        clock = _Clock(op['clock'])
        self.node._now = clock
        self.node._capture.lines.clear()
        self.stops = 0
        self.diags = []
        _LAST_HITS[:] = []
        call()
        assert self.stops <= 1 and len(self.diags) <= 1, (self.stops, self.diags)
        return clock

    def run(self, op):
        kind = op['op']
        out = {}
        if kind == 'fn':
            out['out'] = run_fn(op['fn'], op['args'])
        elif kind == 'node':
            try:
                out.update(self._start(op['params']))
            except Exception as exc:  # the Rust side refuses with started: false
                self.close()
                out.update({'started': False, 'error': repr(exc)})
        elif kind == 'scan':
            msg = LaserScan()
            msg.header.frame_id = 'laser'
            msg.angle_min = dec_float(op['angle_min'])
            msg.angle_increment = dec_float(op['angle_increment'])
            msg.range_min = dec_float(op['range_min'])
            msg.range_max = dec_float(op['range_max'])
            msg.ranges = [dec_float(r) for r in op['ranges']]
            clock = self._callback(op, lambda: self.node._scan_cb(msg))
            enabled = bool(self.node.enabled)
            out.update({
                'enabled': enabled, 'stop': self.stops == 1,
                'diag': self.diags[0] if self.diags else None,
                'logs': list(self.node._capture.lines) if enabled else [],
                'hits': [_hit(h) for h in _LAST_HITS[0]] if _LAST_HITS else None,
                'clock_reads': clock.reads, 'state': self._core_state()})
        elif kind == 'republish':
            clock = self._callback(op, self.node._republish)
            out.update({'enabled': bool(self.node.enabled), 'stop': self.stops == 1,
                        'logs': list(self.node._capture.lines),
                        'clock_reads': clock.reads, 'state': self._core_state()})
        elif kind == 'lane':
            msg = AckermannDriveStamped()
            msg.drive.speed = dec_float(op['speed'])
            lane = op['lane']
            clock = self._callback(op, lambda: self.node._lane_cb(lane, msg))
            out.update({'stop': self.stops == 1, 'clock_reads': clock.reads,
                        'state': self._core_state()})
        elif kind == 'set_state':
            t = op['last_hit_time']
            self.node.state.last_hit_time = None if t is None else dec_float(t)
            out['state'] = self._core_state()
        else:
            raise ValueError(f'unknown op {kind!r}')
        out['id'] = op.get('id')
        return normalise(out)


# ---------------------------------------------------------------- rust side
def replay_binary():
    import os
    from ament_index_python.packages import get_package_prefix
    return os.path.join(get_package_prefix('f1tenth_hard_stop_rs'), 'lib',
                        'f1tenth_hard_stop_rs', 'hard_stop_replay')


def run_rust(ops, binary=None):
    """Every op through hard_stop_replay in one process: (outputs, seconds, stderr)."""
    import subprocess
    import time
    payload = ''.join(json.dumps(op, allow_nan=False) + '\n' for op in ops)
    t0 = time.perf_counter()
    p = subprocess.run([binary or replay_binary()], input=payload, capture_output=True,
                       text=True, check=False)
    dt = time.perf_counter() - t0
    out = [json.loads(line) for line in p.stdout.splitlines() if line.strip()]
    if len(out) != len(ops):
        raise RuntimeError(f'replay returned {len(out)} results for {len(ops)} ops '
                           f'(exit {p.returncode}): {p.stderr[-2000:]}')
    return out, dt, p.stderr
