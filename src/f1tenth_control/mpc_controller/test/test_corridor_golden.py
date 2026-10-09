"""
Golden replay of mpc_corr's control loop: corridor, local target, solve.

WHAT THIS PINS. A fixed set of scenarios is replayed through the REAL
MPCController -- its own __init__ (so the deployed parameter defaults), its own
subscription callbacks, and its own control_loop reached through the timer it
registers -- against a recorded input window (golden/corridor_inputs.json.gz,
cut from an archived wall_turn run by tools/make_corridor_golden_inputs.py:
odometry, /scan, map -> odom, base_link -> laser, obstacles, costmap
boundaries). Per control tick it records every solve_mpc_step call (inputs
including the corridor dict and the local target, outputs including (delta,
a)), every message published, every log line with its throttle arguments, and
at the end the snapshot / CSV files the node wrote. golden/corridor_golden.json.gz
holds what the code produced when the goldens were generated; this test
replays and requires the same, bit for bit. It also pins the node's declared
parameters (names and defaults), topics, types, QoS and timer period.

WHAT IS A STAND-IN. Only rclpy plumbing: Node.__init__, parameter
declaration, publishers, subscriptions, the timer, the clock, the logger and
the tf2 buffer. Nothing touches the network, and the node's debug files go to
a temporary directory.

SAME MACHINE ONLY. Bit-exact float equality is only meaningful against the
build that produced it (numpy/BLAS, scipy, OSQP). The golden header records
the generating machine; on a different host or library version the replay
tests SKIP and say why. The interface test does not depend on floating point
and always runs.

Scenarios: goal_distance (fresh map -> odom, a stalled one, an obstacle in the
corridor), goal_pose, goal_turn, goal_object (object_corridor_mode off / arc /
arc_far, and arc_far with a stalled map -> odom), goal_drive (straight with a
d_wall correction, wall_turn with the wall tracker on recorded /scan). Every
one runs twice, with corridor_heading_return forced true and forced false, so
the goldens do not depend on that key's yaml default.

Regenerate (only when a behaviour change is INTENDED, on the same machine):
    python3 test/test_corridor_golden.py --regenerate [--trace FULL.json.gz]
Write the full trace of the current code for diffing:
    python3 test/test_corridor_golden.py --trace FULL.json.gz
"""

import argparse
import array
import base64
import contextlib
import datetime
import gzip
import hashlib
import json
import math
from pathlib import Path
import platform
import re
import socket
import subprocess
import sys
import tempfile
from types import SimpleNamespace
from unittest import mock

from f1tenth_messages.msg import DriveCommand, ObjectGoal, TurnGoal
from geometry_msgs.msg import PoseStamped
import mpc_controller.MPC_corr as mpc_corr
import numpy as np
import pytest
from rclpy.clock import ClockType
import rclpy.node
from rclpy.serialization import deserialize_message
from rclpy.time import Time
from rosidl_runtime_py.convert import message_to_ordereddict
from rosidl_runtime_py.utilities import get_message
from std_msgs.msg import Float32
from tf2_ros import LookupException

_HERE = Path(__file__).resolve().parent
INPUTS_PATH = _HERE / 'golden' / 'corridor_inputs.json.gz'
GOLDEN_PATH = _HERE / 'golden' / 'corridor_golden.json.gz'

# Keys control_loop attaches to the cached corridor every tick; everything
# else in the dict is what the corridor build produced.
_ATTACHED_KEYS = ('obstacles_world', 'd_safe', 'car_radius', 'avoidance_margin')

# map -> odom delivery stalls at this tick (tf2 keeps handing back the last
# edge it received, which is the real failure mode) and resumes at the second.
_STALL = (15, 30)

_GOAL_TICK = 2
_OBJECT_AHEAD_M = 3.5
_OBJECT_DRIFT = (10, 20, 0.04)   # from tick, to tick, metres per tick in map y

_BASE_SCENARIOS = {
    # 1.5 m: reached inside the window, so the termination check and
    # /mpc/goal_reached are covered too.
    'distance_fresh': {'goal': ['distance', 1.5]},
    'distance_stale': {'goal': ['distance', 1.5], 'stall': list(_STALL)},
    'distance_obstacle': {'goal': ['distance', 6.0], 'obstacles': 'synthetic'},
    'pose': {'goal': ['pose', 4.6, -1.6, -1.2]},
    'turn': {'goal': ['turn', -90.0, 0.5, 'full_lock']},
    'object_off': {'goal': ['object', 0.6, 0.5], 'object_corridor_mode': 'off'},
    'object_arc': {'goal': ['object', 0.6, 0.5], 'object_corridor_mode': 'arc'},
    'object_arc_far': {'goal': ['object', 0.6, 0.5], 'object_corridor_mode': 'arc_far'},
    'object_arc_far_stale': {'goal': ['object', 0.6, 0.5],
                             'object_corridor_mode': 'arc_far', 'stall': list(_STALL)},
    'drive_straight': {'goal': ['drive', 'straight', 0.0, 0.0, 0.5, -1.0],
                       'd_wall_correction': 0.03},
    # Issued at tick 20 (dFront 2.73 m): the first rebuild of the move is
    # then inside the 90 degree trigger and commits, so the wall is selected
    # from the recorded /scan and refitted every tick after.
    'drive_wall_turn': {'goal': ['drive', 'wall_turn', -1.0, 90.0, 0.5, -1.0],
                        'goal_tick': 20},
}


def _scenarios():
    out = {}
    for name, spec in _BASE_SCENARIOS.items():
        for heading_return in (True, False):
            out[f'{name}__hr_{str(heading_return).lower()}'] = dict(
                spec, corridor_heading_return=heading_return)
    return out


SCENARIOS = _scenarios()


# ----------------------------------------------------------------------------
# Canonical form: exact, JSON-safe, order-independent for dicts.
# ----------------------------------------------------------------------------
def canon(value):
    """Return an exact JSON-safe form of value (floats kept bit-exact)."""
    if isinstance(value, dict):
        return {str(k): canon(value[k]) for k in sorted(value, key=str)}
    if isinstance(value, (list, tuple)):
        return [canon(v) for v in value]
    if isinstance(value, np.ndarray):
        return {'nd': str(value.dtype), 'shape': list(value.shape),
                'v': [canon(v) for v in value.ravel().tolist()]}
    if isinstance(value, array.array):
        return [canon(v) for v in value.tolist()]
    if isinstance(value, (bool, np.bool_)):
        return bool(value)
    if isinstance(value, (int, np.integer)):
        return int(value)
    if isinstance(value, (float, np.floating)):
        return float(value)
    if isinstance(value, (bytes, bytearray)):
        return {'b64': base64.b64encode(bytes(value)).decode('ascii')}
    if value is None or isinstance(value, str):
        return value
    if hasattr(value, 'get_fields_and_field_types'):
        return {'msg': type(value).__name__, 'f': canon(message_to_ordereddict(value))}
    return {'repr': repr(value)}


def _dumps(value):
    return json.dumps(value, sort_keys=True, allow_nan=True, separators=(',', ':'))


def digest(value):
    """Return a short sha256 of the canonical form of value."""
    return hashlib.sha256(_dumps(canon(value)).encode('utf-8')).hexdigest()[:20]


def _same(a, b):
    if isinstance(a, float) and isinstance(b, float):
        return a == b or (math.isnan(a) and math.isnan(b))
    return a == b


def first_difference(expected, actual, path='$'):
    """Return (path, expected, actual) of the first mismatch, or None."""
    if isinstance(expected, dict) and isinstance(actual, dict):
        for key in sorted(set(expected) | set(actual)):
            if key not in expected or key not in actual:
                return (f'{path}.{key}', expected.get(key, '<absent>'),
                        actual.get(key, '<absent>'))
            diff = first_difference(expected[key], actual[key], f'{path}.{key}')
            if diff:
                return diff
        return None
    if isinstance(expected, list) and isinstance(actual, list):
        if len(expected) != len(actual):
            return (f'{path}.len', len(expected), len(actual))
        for i, (e, a) in enumerate(zip(expected, actual)):
            diff = first_difference(e, a, f'{path}[{i}]')
            if diff:
                return diff
        return None
    if type(expected) is not type(actual) or not _same(expected, actual):
        return (path, expected, actual)
    return None


# ----------------------------------------------------------------------------
# Recorded inputs
# ----------------------------------------------------------------------------
def load_inputs(path=INPUTS_PATH):
    """Load the recorded input window, deserializing each message once."""
    with gzip.open(path, 'rb') as f:
        raw = f.read()
    fixture = json.loads(raw.decode('utf-8'))
    messages = {}
    for key, blob in fixture['blobs'].items():
        messages[key] = deserialize_message(
            base64.b64decode(blob['cdr']), get_message(blob['type']))
    fixture['messages'] = messages
    fixture['sha256'] = hashlib.sha256(raw).hexdigest()
    return fixture


def _yaw(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def _object_target_map(inputs):
    """Point _OBJECT_AHEAD_M ahead of the car at the goal tick, in map."""
    tick = inputs['ticks'][_GOAL_TICK]
    pose = inputs['messages'][tick['odom']].pose.pose
    x, y, yaw = pose.position.x, pose.position.y, _yaw(pose.orientation)
    px = x + _OBJECT_AHEAD_M * math.cos(yaw)
    py = y + _OBJECT_AHEAD_M * math.sin(yaw)
    tf = inputs['messages'][tick['map_odom']].transform
    t_yaw = _yaw(tf.rotation)
    return (tf.translation.x + math.cos(t_yaw) * px - math.sin(t_yaw) * py,
            tf.translation.y + math.sin(t_yaw) * px + math.cos(t_yaw) * py)


# ----------------------------------------------------------------------------
# rclpy stand-ins
# ----------------------------------------------------------------------------
class _Logger:
    def __init__(self, rig):
        self._rig = rig

    def _log(self, level, text, **kwargs):
        self._rig.logs.append([level, self._rig.scrub(str(text)), canon(kwargs)])

    def debug(self, text, **kwargs):
        self._log('debug', text, **kwargs)

    def info(self, text, **kwargs):
        self._log('info', text, **kwargs)

    def warn(self, text, **kwargs):
        self._log('warn', text, **kwargs)

    def warning(self, text, **kwargs):
        self._log('warn', text, **kwargs)

    def error(self, text, **kwargs):
        self._log('error', text, **kwargs)


class _Clock:
    def __init__(self, rig):
        self._rig = rig

    def now(self):
        return Time(nanoseconds=self._rig.now_ns, clock_type=ClockType.ROS_TIME)


class _Publisher:
    def __init__(self, rig, topic):
        self._rig = rig
        self.topic_name = topic

    def publish(self, msg):
        self._rig.published.append([self.topic_name, canon(msg)])

    def get_subscription_count(self):
        return 1


class _TfBuffer:
    def __init__(self):
        self.map_odom = None
        self.laser = None

    def lookup_transform(self, target, source, when):
        if (target, source) == ('map', 'odom') and self.map_odom is not None:
            return self.map_odom
        if (target, source) == ('base_link', 'laser') and self.laser is not None:
            return self.laser
        raise LookupException(f'"{target}" <- "{source}" not available')


def _type_name(msg_type):
    return f'{msg_type.__module__.split(".")[0]}/{msg_type.__name__}'


def _qos_name(qos):
    if isinstance(qos, int):
        return f'depth={qos}'
    return (f'{qos.reliability.name}/{qos.durability.name}/{qos.history.name}'
            f'/depth={qos.depth}')


class Rig:
    """One MPCController built and driven with stand-in rclpy plumbing."""

    def __init__(self, overrides, tmpdir):
        self.overrides = dict(overrides)
        self.tmpdir = Path(tmpdir)
        self.now_ns = 0
        self.logs = []
        self.published = []
        self.solves = []
        self.params = {}
        self.subs = {}
        self.pubs = {}
        self.timers = []
        self.tf = _TfBuffer()
        self._stack = contextlib.ExitStack()

    def scrub(self, text):
        text = text.replace(str(self.tmpdir), '<TMP>')
        return re.sub(r'corridor_debug_\d{8}T\d{6}', 'corridor_debug_<STAMP>', text)

    # -- patches ------------------------------------------------------------
    def __enter__(self):
        rig = self

        def node_init(self, *args, **kwargs):
            pass

        def declare_parameter(self, name, value=None, *args, **kwargs):
            used = rig.overrides.get(name, value)
            rig.params[name] = {'default': canon(value), 'value': canon(used)}
            return SimpleNamespace(value=used)

        def get_parameter(self, name):
            return SimpleNamespace(value=rig.overrides.get(
                name, rig.params[name]['value']))

        def create_subscription(self, msg_type, topic, callback, qos, *args, **kwargs):
            rig.subs[topic] = {'type': _type_name(msg_type), 'qos': _qos_name(qos),
                               'callback': callback}
            return SimpleNamespace(topic_name=topic)

        def create_publisher(self, msg_type, topic, qos, *args, **kwargs):
            rig.pubs[topic] = {'type': _type_name(msg_type), 'qos': _qos_name(qos)}
            return _Publisher(rig, topic)

        def create_timer(self, period, callback, *args, **kwargs):
            rig.timers.append({'period': period, 'callback': callback})
            return SimpleNamespace(period=period)

        logger = _Logger(rig)
        clock = _Clock(rig)
        real_solve = mpc_corr.solve_mpc_step

        def recording_solve(**kwargs):
            entry = {'in': canon(kwargs)}
            u0, info = real_solve(**kwargs)
            entry['u0'] = canon(u0)
            entry['info'] = canon(info)
            rig.solves.append(entry)
            return u0, info

        controller = mpc_corr.MPCController
        for target, name, new in (
                (rclpy.node.Node, '__init__', node_init),
                (controller, 'declare_parameter', declare_parameter),
                (controller, 'get_parameter', get_parameter),
                (controller, 'create_subscription', create_subscription),
                (controller, 'create_publisher', create_publisher),
                (controller, 'create_timer', create_timer),
                (controller, 'get_logger', lambda self: logger),
                (controller, 'get_clock', lambda self: clock),
                (mpc_corr, 'Buffer', lambda *a, **k: rig.tf),
                (mpc_corr, 'TransformListener', lambda *a, **k: None),
                (mpc_corr, '_resolve_debug_output_path',
                 lambda filename: rig.tmpdir / filename),
                (mpc_corr, 'solve_mpc_step', recording_solve)):
            self._stack.enter_context(mock.patch.object(target, name, new))
        return self

    def __exit__(self, *exc):
        self._stack.close()
        return False

    # -- running ------------------------------------------------------------
    def build(self, now_ns):
        self.now_ns = now_ns
        self.node = mpc_corr.MPCController()
        unknown = sorted(set(self.overrides) - set(self.params))
        assert not unknown, f'overrides for undeclared parameters: {unknown}'
        return self.node

    def deliver(self, topic, msg):
        self.subs[topic]['callback'](msg)

    def tick_control(self):
        (timer,) = self.timers
        timer['callback']()

    def interface(self):
        return {
            'parameters': {k: v['default'] for k, v in sorted(self.params.items())},
            'subscriptions': {k: {'type': v['type'], 'qos': v['qos']}
                              for k, v in sorted(self.subs.items())},
            'publishers': {k: dict(v) for k, v in sorted(self.pubs.items())},
            'timers': [t['period'] for t in self.timers],
        }

    def files(self):
        self.node.corridor_log_file.flush()
        self.node.control_log_file.flush()
        self.node.error_log_file.flush()
        out = {}
        for path in sorted(self.tmpdir.iterdir()):
            out[self.scrub(path.name)] = hashlib.sha256(
                self.scrub(path.read_text(encoding='utf-8')).encode('utf-8')).hexdigest()[:20]
        return out

    def close(self):
        for name in ('corridor_log_file', 'control_log_file', 'error_log_file'):
            handle = getattr(self.node, name, None)
            if handle is not None:
                handle.close()
        self.node.model_log.close()


# ----------------------------------------------------------------------------
# One scenario
# ----------------------------------------------------------------------------
def _goal_messages(spec, k, inputs, clock_ns):
    goal = spec['goal']
    kind = goal[0]
    goal_tick = spec.get('goal_tick', _GOAL_TICK)
    if kind == 'object':
        if k < goal_tick:
            return []
        tx, ty = _object_target_map(inputs)
        start, stop, step = _OBJECT_DRIFT
        ty += step * (min(max(k, start), stop) - start)
        msg = ObjectGoal()
        msg.header.frame_id = 'map'
        msg.header.stamp = Time(nanoseconds=clock_ns).to_msg()
        msg.move_id = 'golden-object-1'
        msg.target_class = 'person'
        msg.point.x, msg.point.y = float(tx), float(ty)
        msg.standoff, msg.speed = float(goal[1]), float(goal[2])
        msg.track_id = 'golden-track-1'
        msg.gap_m = float(goal[1])
        return [('/mpc/goal_object', msg)]
    if k != goal_tick:
        return []
    if kind == 'distance':
        return [('/mpc/goal_distance', Float32(data=float(goal[1])))]
    if kind == 'pose':
        msg = PoseStamped()
        msg.header.frame_id = 'odom'
        msg.pose.position.x, msg.pose.position.y = float(goal[1]), float(goal[2])
        msg.pose.orientation.z = math.sin(goal[3] / 2.0)
        msg.pose.orientation.w = math.cos(goal[3] / 2.0)
        return [('/mpc/goal_pose', msg)]
    if kind == 'turn':
        return [('/mpc/goal_turn', TurnGoal(
            heading_delta_deg=float(goal[1]), speed=float(goal[2]), steering=goal[3]))]
    if kind == 'drive':
        return [('/mpc/goal_drive', DriveCommand(
            mode=goal[1], turn_sign=float(goal[2]), turn_mag_deg=float(goal[3]),
            speed=float(goal[4]), d_safe=float(goal[5])))]
    raise ValueError(kind)


def _base_overrides(spec):
    return {
        'corridor_heading_return': bool(spec['corridor_heading_return']),
        'object_corridor_mode': spec.get('object_corridor_mode', 'off'),
        'use_map_frame_goal_anchor': True,
        # '' disables the model-validation CSV, which would otherwise be
        # created relative to the working directory.
        'model_log_path': '',
    }


def _summary(entry, previous_geometry):
    """Readable per-tick record of one solve, plus digests of the rest."""
    kw = entry['in']
    corridor = kw['corridor']
    geometry = {k: v for k, v in corridor.items() if k not in _ATTACHED_KEYS}
    out = {
        'x0': kw['x0'], 'last_u': kw['last_u'], 'pref_nom': kw['pref_nom'],
        'u0': entry['u0'], 'vdes': kw['vdes'], 'dmin': kw['dmin'],
        'solver': kw['solver'], 'weights': kw['weights'],
        'obstacles': kw['obstacles'], 'boundaries': kw['boundaries'],
        'attached': {k: corridor.get(k) for k in _ATTACHED_KEYS},
        'psiRef': corridor.get('psiRef'), 'L': corridor.get('L'),
        'objectShape': corridor.get('objectShape'),
        'success': entry['info'].get('success'), 'status': entry['info'].get('status'),
        'cost': entry['info'].get('cost'), 'iterations': entry['info'].get('iterations'),
        'geometry_digest': digest(geometry),
        'inputs_digest': digest(kw),
        'info_digest': digest(entry['info']),
    }
    full_geometry = geometry if out['geometry_digest'] != previous_geometry else None
    return out, full_geometry


def run_scenario(name, spec, inputs, overrides=None):
    """Replay one scenario; return (golden record, full trace)."""
    messages = inputs['messages']
    ticks = inputs['ticks']
    with tempfile.TemporaryDirectory(prefix='corridor_golden_') as tmp, \
            Rig(overrides if overrides is not None else _base_overrides(spec), tmp) as rig:
        rig.tf.laser = messages[inputs['laser_static']]
        rig.build(ticks[0]['clock_ns'])
        init_logs = list(rig.logs)
        record = {'interface': rig.interface(),
                  'params': {k: v['value'] for k, v in sorted(rig.params.items())},
                  'init_logs': digest(init_logs), 'ticks': []}
        # Full corridor bodies, keyed by geometry digest: one entry each time
        # the build produces a different corridor. Shared across scenarios in
        # the golden file (most corridors repeat between the two heading_return
        # variants), and referenced from each tick by solve.geometry_digest.
        corridors = {}
        trace = {'init_logs': init_logs, 'ticks': []}
        odom_topic = next(t for t in ('/odometry/filtered', '/odom') if t in rig.subs)
        stall = spec.get('stall')
        last_fed = {}
        last_geometry = None
        frozen_tf = None

        def feed(topic, key):
            if key is None or topic not in rig.subs or last_fed.get(topic) == key:
                return
            last_fed[topic] = key
            rig.deliver(topic, messages[key])

        for k, tick in enumerate(ticks):
            rig.now_ns = tick['clock_ns']
            rig.logs, rig.published, rig.solves = [], [], []

            tf = messages[tick['map_odom']] if tick['map_odom'] else None
            if stall and stall[0] <= k < stall[1]:
                if frozen_tf is None:
                    frozen_tf = messages[ticks[stall[0] - 1]['map_odom']]
                tf = frozen_tf
            rig.tf.map_odom = tf

            feed(odom_topic, tick['odom'])
            feed('/costmap/boundaries', tick['boundaries'])
            feed('/perception/obstacles_2d',
                 tick['synthetic_obstacle'] if spec.get('obstacles') == 'synthetic'
                 else tick['obstacles'])
            feed('/scan', tick['scan'])
            rig.deliver('/perception/front_distance',
                        Float32(data=float(tick['front_distance'])))
            if spec.get('d_wall_correction') is not None:
                rig.deliver('/perception/d_wall/psi_correction',
                            Float32(data=float(spec['d_wall_correction'])))
            for topic, msg in _goal_messages(spec, k, inputs, tick['clock_ns']):
                rig.deliver(topic, msg)

            rig.tick_control()

            entry = {'k': k, 'solve': None,
                     'drive': [[p[1]['f']['drive']['speed'], p[1]['f']['drive']['steering_angle']]
                               for p in rig.published if p[0] == '/drive'],
                     'published': [[topic, digest(msg)] for topic, msg in rig.published],
                     'logs': digest(rig.logs), 'n_logs': len(rig.logs)}
            assert len(rig.solves) <= 1
            if rig.solves:
                entry['solve'], full = _summary(rig.solves[0], last_geometry)
                last_geometry = entry['solve']['geometry_digest']
                if full is not None:
                    corridors[last_geometry] = full
            record['ticks'].append(entry)
            trace['ticks'].append({'k': k, 'logs': rig.logs, 'published': rig.published,
                                   'solves': rig.solves})

        record['files'] = rig.files()
        rig.close()
    return record, corridors, trace


# ----------------------------------------------------------------------------
# Header / machine identity
# ----------------------------------------------------------------------------
def _machine():
    import osqp
    import scipy
    model = Path('/proc/device-tree/model')
    return {
        'hostname': socket.gethostname(),
        'machine': platform.machine(),
        'kernel': platform.release(),
        'device_model': (model.read_text(errors='replace').strip('\x00\n')
                         if model.exists() else None),
        'python': platform.python_version(),
        'numpy': np.__version__,
        'scipy': scipy.__version__,
        'osqp': osqp.__version__,
    }


# Identity fields a bit-exact comparison depends on; the rest is information.
_MACHINE_GATE = ('hostname', 'machine', 'numpy', 'scipy', 'osqp')


def _git_head():
    try:
        return subprocess.run(['git', 'rev-parse', '--short', 'HEAD'], cwd=_HERE,
                              capture_output=True, text=True, check=True).stdout.strip()
    except (OSError, subprocess.CalledProcessError):
        return None


def generate(inputs):
    """Replay every scenario; return (golden document, full traces)."""
    golden = {'header': {
        'generated_at': datetime.datetime.now(datetime.timezone.utc).isoformat(),
        'generated_from_commit': _git_head(),
        'machine': _machine(),
        'inputs': {'file': INPUTS_PATH.name, 'sha256': inputs['sha256'],
                   'source_run': inputs['source_run'],
                   'source_bag_sha256': inputs['source_bag_sha256']},
        'note': ('Bit-exact goldens. Compare only on the machine and library '
                 'versions recorded here; regenerate only for an intended change.'),
    }, 'scenarios': {}}
    traces = {}
    interface = None
    corridors = {}
    for name, spec in SCENARIOS.items():
        record, scenario_corridors, trace = run_scenario(name, spec, inputs)
        if interface is None:
            interface = record['interface']
        assert record.pop('interface') == interface, f'{name}: interface differs'
        golden['scenarios'][name] = dict(record, spec=canon(spec))
        corridors.update(scenario_corridors)
        traces[name] = trace
    golden['interface'] = interface
    golden['corridors'] = corridors
    return golden, traces


def _write_gz(path, document):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    with gzip.GzipFile(path, 'wb', mtime=0) as f:
        f.write(_dumps(document).encode('utf-8'))


def load_golden(path=GOLDEN_PATH):
    with gzip.open(path, 'rb') as f:
        return json.loads(f.read().decode('utf-8'))


# ----------------------------------------------------------------------------
# Tests
# ----------------------------------------------------------------------------
@pytest.fixture(scope='module')
def golden():
    return load_golden()


@pytest.fixture(scope='module')
def inputs():
    return load_inputs()


def _skip_unless_same_machine(golden):
    recorded = golden['header']['machine']
    here = _machine()
    differs = [f'{k}: golden {recorded.get(k)!r}, here {here.get(k)!r}'
               for k in _MACHINE_GATE if recorded.get(k) != here.get(k)]
    if differs:
        pytest.skip('bit-exact goldens were generated on a different machine/build ('
                    + '; '.join(differs) + ')')


def test_inputs_are_the_ones_the_goldens_were_generated_from(golden, inputs):
    assert inputs['sha256'] == golden['header']['inputs']['sha256']


def test_declared_interface_is_unchanged(golden, inputs):
    """Parameters (names and defaults), topics, types, QoS and timer period."""
    spec = SCENARIOS['distance_fresh__hr_true']
    with tempfile.TemporaryDirectory(prefix='corridor_golden_') as tmp, \
            Rig(_base_overrides(spec), tmp) as rig:
        rig.tf.laser = inputs['messages'][inputs['laser_static']]
        rig.build(inputs['ticks'][0]['clock_ns'])
        actual = canon(rig.interface())
        rig.close()
    diff = first_difference(golden['interface'], actual)
    assert diff is None, f'interface changed at {diff[0]}: {diff[1]!r} -> {diff[2]!r}'


@pytest.mark.parametrize('name', sorted(SCENARIOS))
def test_replay_matches_golden(name, golden, inputs, tmp_path):
    _skip_unless_same_machine(golden)
    expected = golden['scenarios'][name]
    # The recorded parameter VALUES are fed back, so the replay does not
    # depend on stack_params.yaml having stayed put; the declared NAMES must
    # match exactly (Rig.build rejects overrides nothing declares, and the
    # comparison below catches a newly declared one).
    record, corridors, trace = run_scenario(
        name, SCENARIOS[name], inputs, overrides=expected['params'])
    record.pop('interface')
    actual = canon(dict(record, spec=canon(SCENARIOS[name])))
    # Ticks first, so a failure names the EARLIEST tick that moved rather
    # than whichever top-level key happens to sort first.
    diff = (first_difference(expected['ticks'], actual['ticks'], '$.ticks')
            or first_difference(expected, actual))
    if diff is None:
        # Per-tick geometry digests already matched; the bodies are compared
        # too, so a failure here names the field rather than only a hash.
        for key, body in sorted(corridors.items()):
            diff = first_difference(golden['corridors'].get(key, '<absent>'), canon(body),
                                    f'corridors[{key}]')
            if diff is not None:
                break
    if diff is not None:
        dump = tmp_path / f'{name}.trace.json.gz'
        _write_gz(dump, trace)
        pytest.fail(f'{name}: first difference at {diff[0]}\n  golden: {diff[1]!r}\n'
                    f'  now:    {diff[2]!r}\n  full trace of this run: {dump}')


# ----------------------------------------------------------------------------
# Regeneration
# ----------------------------------------------------------------------------
def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.split('\n')[1])
    parser.add_argument('--regenerate', action='store_true',
                        help=f'overwrite {GOLDEN_PATH.name}')
    parser.add_argument('--trace', help='write the full trace (logs, messages) here')
    args = parser.parse_args(argv)
    inputs = load_inputs()
    document, traces = generate(inputs)
    if args.regenerate:
        _write_gz(GOLDEN_PATH, document)
        print(f'wrote {GOLDEN_PATH} ({GOLDEN_PATH.stat().st_size} bytes)')
    if args.trace:
        _write_gz(args.trace, traces)
        print(f'wrote {args.trace}')
    if not args.regenerate and not args.trace:
        parser.print_help()
    return 0


if __name__ == '__main__':
    sys.exit(main())
