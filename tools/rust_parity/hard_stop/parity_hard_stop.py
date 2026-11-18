#!/usr/bin/env python3
"""Parity: f1tenth_control's hard stop (Python) vs f1tenth_hard_stop_rs (Rust).

The same ops (f1tenth_hard_stop_rs::replay's protocol) go to
  - the Python: py_reference.PySession, i.e. hard_stop.py's functions and a
    real HardStopNode (its _scan_cb / _republish / _lane_cb, with only the
    clock, logger and publishers replaced), and
  - the Rust: hard_stop_replay (decision.rs and core.rs, the code the node runs),
and EVERY output is compared exactly: published stops, the latch state after
each callback, the triggering returns (floats by bit pattern), the
/safety/hard_stop JSON and the log lines byte for byte, and the number of
clock reads per callback.

Sections:
  tests      the 42 tests of test_hard_stop.py as cases (make_test_cases.py):
             the committed file must be current, and Rust must reproduce it;
  format     CPython's round / repr / json / format / degrees vs pyfmt.rs;
  functions  the pure functions on random inputs (edges, NaN/inf, odd layouts);
  sessions   random node sessions: parameters, scan layouts, objects, lanes,
             republish ticks, clocks that jump, stall, go back, advance in a
             callback;
  bags       every recorded /scan of the car bags with the recorded /drive,
             /teleop, /calibration_drive and a 20 Hz republish tick, as the
             node would see them, in four variants: default, reverse gate off,
             no self-hit mask (every self-return becomes a trigger) and
             injected obstacles (+ synthetic reverse commands);
  recorded   bags in which the car's own hard_stop_node stopped the car
             (/safety_stop frame_id base_link/hard_stop): the replay's stop
             episodes against the recorded ones (informative, not gated:
             the parameters of that run are the launch defaults by assumption);
  timing     per-scan compute of the node's callback, Python vs Rust.

Gate: zero differences in every gated section (exit 1 otherwise).

Usage (workspace sourced, isolated DDS: no node is spun, but rclpy creates
participants): parity_hard_stop.py [--quick] [--bags N] [--out FILE.json]
"""

import argparse
import glob
import json
import math
import os
import random
import struct
import sys
import time

import numpy as np
import rclpy

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import make_test_cases as mtc  # noqa: E402
from py_reference import (  # noqa: E402
    dec_float, enc_float, first_difference, PySession, replay_binary, run_rust, same)

ROOTS = ['~/f1tenth_campaigns/first_test_campaign/raw', '/home/andreas/bags']
NODE = dict(mtc.NODE_DEFAULTS)
LANES = ('teleop', 'calibration_drive', 'drive')


# ---------------------------------------------------------------- comparison
class Tally:
    def __init__(self):
        self.groups = {}

    def group(self, name):
        return self.groups.setdefault(name, dict(ops=0, diffs=0, examples=[], py_s=0.0,
                                                 rs_s=0.0, by_op={}, stops=0, engages=0,
                                                 scans=0))

    def total_diffs(self, gated):
        return sum(g['diffs'] for k, g in self.groups.items() if k.split(':')[0] in gated)


def _equal(p, r):
    # A refused node: both must refuse; the messages are each language's own.
    if isinstance(p, dict) and p.get('started') is False:
        return isinstance(r, dict) and r.get('started') is False
    return same(p, r)


def compare(tally, group, ops, binary, py_outs=None):
    """Run ops through both sides; count and keep the first differences."""
    g = tally.group(group)
    t0 = time.perf_counter()
    if py_outs is None:
        s = PySession()
        try:
            py_outs = [s.run(op) for op in ops]
        finally:
            s.close()
    g['py_s'] += time.perf_counter() - t0
    rs_outs, dt, _err = run_rust(ops, binary)
    g['rs_s'] += dt
    for op, p, r in zip(ops, py_outs, rs_outs):
        g['ops'] += 1
        kind = op['op']
        g['by_op'][kind] = g['by_op'].get(kind, 0) + 1
        if kind == 'scan':
            g['scans'] += 1
        if isinstance(p, dict) and p.get('stop'):
            g['stops'] += 1
        if 'error' in r and not (kind == 'node' and r.get('started') is False):
            g['diffs'] += 1
            if len(g['examples']) < 10:
                g['examples'].append(dict(id=op.get('id'), rust_error=r['error']))
            continue
        if not _equal(p, r):
            g['diffs'] += 1
            if len(g['examples']) < 10:
                path, a, b = first_difference(p, r)
                g['examples'].append(dict(id=op.get('id'), path=path, py=a, rs=b))
    return py_outs, rs_outs


def count_engages(outs):
    n, prev = 0, False
    for o in outs:
        st = o.get('state') if isinstance(o, dict) else None
        if st is None:
            continue
        if st['engaged'] and not prev:
            n += 1
        prev = st['engaged']
    return n


# ---------------------------------------------------------------- tests
def section_tests(tally, binary):
    records = mtc.build()
    text = mtc.dumps(records)
    with open(mtc.DEFAULT_OUT) as f:
        committed = f.read()
    current = committed == text
    ops, stored = [], []
    for rec in [json.loads(line) for line in committed.splitlines() if line.strip()]:
        if rec.get('rust_only') == 'deps':
            continue
        ops += rec['ops']
        stored += rec['expected']
    py = [o for rec in records if rec['expected'] for o in rec['expected']]
    compare(tally, 'tests', ops, binary, py_outs=py)
    # The committed expectations against the fresh Python run, too.
    g = tally.group('tests:committed_vs_python')
    g['ops'] = len(stored)
    g['diffs'] = sum(1 for a, b in zip(stored, py) if not same(a, b)) + abs(len(stored) - len(py))
    deps, _dt, _e = run_rust([{'op': 'deps', 'id': 'deps'}], binary)
    forbidden = [d for d in deps[0]['out']
                 for f in ('py_trees', 'f1tenth_behavior', 'behaviours') if f in d]
    return dict(file=os.path.relpath(mtc.DEFAULT_OUT, mtc._REPO), current=current,
                tests=len(records), ops=len(ops), rust_deps=deps[0]['out'],
                forbidden_deps=forbidden)


# ---------------------------------------------------------------- format
def section_format(tally, binary, rng, n):
    vals = [0.0, -0.0, math.nan, math.inf, -math.inf, 5e-324, -5e-324, 2.2250738585072014e-308,
            1.7976931348623157e308, 1e16, 1e15, 9999999999999998.0, 0.0001, 0.00001, 1e-5,
            0.5, 1.5, 2.5, -0.5, 0.125, 0.375, 0.03125, -0.03125, 0.00005, -0.00005]
    for _ in range(n):
        vals.append(struct.unpack('<d', struct.pack('<Q', rng.getrandbits(64)))[0])
        vals.append(rng.uniform(-200.0, 200.0))
        vals.append(rng.uniform(-1.0, 1.0))
        vals.append(float(np.float32(rng.uniform(0.0, 0.5))))
        vals.append(rng.randint(-10**6, 10**6) / 2 ** rng.randint(0, 20))   # exact binary ties
        vals.append(rng.randint(-10**5, 10**5) / 10 ** rng.randint(0, 6))
        e = rng.randint(-30, 30)
        vals.append(rng.uniform(1.0, 10.0) * 10.0 ** e)
    # Every bearing of the car's and the test's layouts, in degrees.
    car = (float(np.float32(-2.356194496154785)), float(np.float32(0.004363323096185923)))
    for amin, inc in (car, (mtc.ANGLE_MIN, mtc.ANGLE_INC)):
        vals += [amin + i * inc for i in range(1081)]
    ops = []
    for k, x in enumerate(vals):
        x = enc_float(x)
        ops.append({'op': 'fn', 'fn': 'py_repr', 'args': {'x': x}, 'id': f'repr{k}'})
        ops.append({'op': 'fn', 'fn': 'json_float', 'args': {'x': x}, 'id': f'json{k}'})
        ops.append({'op': 'fn', 'fn': 'degrees', 'args': {'x': x}, 'id': f'deg{k}'})
        for nd in (0, 1, 2, 3, 4, 6):
            ops.append({'op': 'fn', 'fn': 'py_round', 'args': {'x': x, 'ndigits': nd},
                        'id': f'round{nd}_{k}'})
        for prec, plus in ((1, True), (2, False), (3, False), (3, True)):
            ops.append({'op': 'fn', 'fn': 'py_fixed', 'args': {'x': x, 'prec': prec, 'plus': plus},
                        'id': f'fixed{prec}{plus}_{k}'})
    for i in range(0, len(ops), 50000):
        compare(tally, 'format', ops[i:i + 50000], binary)
    return dict(values=len(vals), ops=len(ops))


# ---------------------------------------------------------------- functions
def _rand_float(rng, lo, hi, special=0.05):
    if rng.random() < special:
        return rng.choice([math.nan, math.inf, -math.inf, 0.0, -0.0, lo, hi])
    return rng.uniform(lo, hi)


def _rand_scan(rng):
    n = rng.choice([0, 1, 3, 10, 100, 1081, 1081, 1081])
    amin = rng.choice([mtc.ANGLE_MIN, float(np.float32(-2.356194496154785)), -math.pi,
                       rng.uniform(-4, 4), 0.0])
    inc = rng.choice([mtc.ANGLE_INC, float(np.float32(0.004363323096185923)), -mtc.ANGLE_INC,
                      rng.uniform(-0.05, 0.05), 0.0])
    fill = rng.choice([5.0, 30.0, math.inf, math.nan, 0.5])
    r = [fill] * n
    for _ in range(rng.randint(0, 12)):
        if not n:
            break
        i = rng.randrange(n)
        w = rng.randint(1, 30)
        v = rng.choice([rng.uniform(0.0, 0.3), rng.uniform(0.1, 0.22), 0.2, 0.12, 0.13,
                        math.nan, math.inf, -math.inf, 0.01, 0.0, -0.0, -0.1])
        r[i:i + w] = [v] * len(r[i:i + w])
    rmin = rng.choice([0.02, float(np.float32(0.02)), 0.0, -1.0, 0.15, math.nan])
    rmax = rng.choice([30.0, 0.19, math.inf, math.nan, 10.0])
    return r, amin, inc, rmin, rmax


def section_functions(tally, binary, rng, n):
    ops = []
    for k in range(n):
        r, amin, inc, rmin, rmax = _rand_scan(rng)
        front = rng.choice([0.405, rng.uniform(0.2, 0.6)])
        poly = [rng.uniform(-0.3, 0.0), front + 0.02, -rng.uniform(0.1, 0.2),
                rng.uniform(0.1, 0.2)]
        if rng.random() < 0.05:
            poly = [math.nan, 0.425, -0.156, 0.156]
        args = {'ranges': [enc_float(v) for v in r], 'angle_min': amin, 'angle_increment': inc,
                'range_min': enc_float(rmin), 'range_max': enc_float(rmax),
                'laser_x': rng.choice([0.305, rng.uniform(0.0, 0.5)]),
                'polygon': [enc_float(v) for v in poly],
                'trigger_range_m': enc_float(rng.choice([0.2, rng.uniform(0.0, 0.6), 0.0,
                                                         math.inf]))}
        ops.append({'op': 'fn', 'fn': 'trigger_range_hits', 'args': args, 'id': f'trh{k}'})
        if k % 5 == 0:
            a2 = dict(args, laser_y=rng.choice([0.0, -0.0, rng.uniform(-0.1, 0.1)]))
            del a2['polygon'], a2['trigger_range_m']
            ops.append({'op': 'fn', 'fn': 'scan_points_base_link', 'args': a2, 'id': f'sp{k}'})
        # A point on, just inside and just outside each edge, and random ones.
        x_min, x_max, y_min, y_max = poly
        for j, (x, y) in enumerate((
                (x_max, 0.0), (x_min, y_max), (np.nextafter(x_max, 1), 0.0),
                (np.nextafter(x_max, -1), y_min), (rng.uniform(-1, 1), rng.uniform(-1, 1)),
                (math.nan, 0.0), (0.0, -0.0))):
            ops.append({'op': 'fn', 'fn': 'is_self_hit',
                        'args': {'x': enc_float(float(x)), 'y': enc_float(float(y)),
                                 'polygon': [enc_float(v) for v in poly]}, 'id': f'sh{k}.{j}'})
        ops.append({'op': 'fn', 'fn': 'footprint_polygon', 'args': {
            'front_x': enc_float(_rand_float(rng, -1, 1)),
            'rear_x': enc_float(_rand_float(rng, -1, 1)),
            'half_width': enc_float(_rand_float(rng, -0.5, 0.5)),
            'margin': enc_float(_rand_float(rng, -0.1, 0.1))}, 'id': f'fp{k}'})
        latest = {}
        now = rng.uniform(0, 10)
        for lane in LANES:
            if rng.random() < 0.6:
                age = rng.choice([0.0, 0.2, rng.uniform(0, 0.4), -0.1, math.nan])
                latest[lane] = [enc_float(now - age),
                                enc_float(rng.choice([0.0, -0.0, -0.4, 0.5, rng.uniform(-1, 1),
                                                      math.nan]))]
        ops.append({'op': 'fn', 'fn': 'winning_speed', 'args': {
            'latest': latest, 'now': now,
            'timeout_s': enc_float(rng.choice([0.2, 0.0, rng.uniform(0, 0.5), math.nan]))},
            'id': f'ws{k}'})
        ops.append({'op': 'fn', 'fn': 'stop_now', 'args': {
            'engaged': rng.random() < 0.7, 'allow_reverse': rng.random() < 0.7,
            'ahead_only': rng.random() < 0.6,
            'speed': rng.choice([None, 0.0, -0.0, -0.4, 0.4, -1e-9, 'nan'])}, 'id': f'sn{k}'})
        sops, t = [], rng.uniform(-5, 5)
        for _ in range(rng.randint(1, 60)):
            t += rng.choice([0.025, 0.0, rng.uniform(0, 0.6), -0.3])
            c = rng.random()
            if c < 0.6:
                sops.append(['update', rng.random() < 0.5, t])
            elif c < 0.85:
                sops.append(['tick', t])
            else:
                sops.append(['as_dict', rng.randint(0, 5),
                             None if rng.random() < 0.3 else
                             [enc_float(rng.uniform(0, 0.3)), enc_float(rng.uniform(-3, 3))]])
        ops.append({'op': 'fn', 'fn': 'state', 'args': {
            'debounce_scans': rng.choice([-1, 0, 1, 2, 2, 3, 5]),
            'clear_s': enc_float(rng.choice([0.5, 0.0, rng.uniform(0, 1.5), -0.1, math.nan])),
            'ops': sops}, 'id': f'st{k}'})
    # all_ahead on the Python's own hits of some of the scans above.
    s = PySession()
    for k, op in enumerate([o for o in ops if o['fn'] == 'trigger_range_hits'][:n // 2]):
        hits = s.run(op)['out']
        ops.append({'op': 'fn', 'fn': 'all_ahead',
                    'args': {'hits': hits, 'polygon': op['args']['polygon']}, 'id': f'aa{k}'})
    s.close()
    compare(tally, 'functions', ops, binary)
    return dict(ops=len(ops))


# ---------------------------------------------------------------- sessions
def _rand_params(rng):
    p = dict(NODE)
    if rng.random() < 0.7:
        p.update({
            'hard_stop_enabled': rng.random() < 0.92,
            'hard_stop_range_m': rng.choice([0.2, 0.15, 0.3, rng.uniform(0.05, 0.5)]),
            'hard_stop_debounce_scans': rng.choice([-1, 0, 1, 2, 2, 3, 4]),
            'hard_stop_clear_s': rng.choice([0.5, 0.0, 0.05, rng.uniform(0.0, 1.5)]),
            'hard_stop_republish_hz': rng.choice([20.0, 10.0, 3.0, 40.0]),
            'footprint_front_x_m': rng.choice([0.405, rng.uniform(0.3, 0.5)]),
            'footprint_rear_x_m': rng.choice([-0.10, rng.uniform(-0.3, 0.0)]),
            'footprint_half_width_m': rng.choice([0.136, rng.uniform(0.05, 0.25)]),
            'footprint_self_margin_m': rng.choice([0.02, 0.0, -0.02, rng.uniform(0, 0.1)]),
            'laser_x_m': rng.choice([0.305, rng.uniform(0.1, 0.45)]),
            'hard_stop_allow_reverse': rng.random() < 0.7,
            'lane_timeout_s': rng.choice([0.2, 0.0, 0.1, rng.uniform(0, 0.5)]),
        })
    if rng.random() < 0.03:
        p['hard_stop_republish_hz'] = rng.choice([0.0, -5.0])     # both refuse
    if rng.random() < 0.03:
        del p['footprint_front_x_m']                               # Rust refuses (launch value)
    return p


def section_sessions(tally, binary, rng, n_sessions, n_events):
    ops = []
    refused_py_only = 0
    for k in range(n_sessions):
        p = _rand_params(rng)
        if 'footprint_front_x_m' not in p:
            refused_py_only += 1
            continue    # The Python has its URDF default here; covered by the launch test.
        ops.append({'op': 'node', 'params': p, 'id': f's{k}'})
        if p['hard_stop_republish_hz'] <= 0:
            continue    # Both refuse to start (compared); no session follows.
        r, amin, inc, rmin, rmax = _rand_scan(rng)
        if rng.random() < 0.5:   # the car's layout most of the time
            r, amin, inc = [5.0] * 1081, mtc.ANGLE_MIN, mtc.ANGLE_INC
            rmin, rmax = 0.02, 30.0
        t = rng.choice([1000.0, 1.792e9, 0.0])
        obj = None
        for e in range(n_events):
            c = rng.random()
            t += rng.choice([0.025, 0.025, 0.025, 0.0, 0.05, rng.uniform(0, 0.7), -0.5])
            clock = [t] if rng.random() < 0.9 else [t + rng.uniform(0, 0.3) * i for i in range(4)]
            if c < 0.6:
                if obj is None and rng.random() < 0.15:
                    obj = (rng.randrange(max(len(r), 1)), rng.randint(1, 8),
                           rng.uniform(0.08, 0.25), rng.randint(1, 40))
                ranges = list(r)
                if obj is not None and ranges:
                    i, w, rr, left = obj
                    ranges[i:i + w] = [rr] * len(ranges[i:i + w])
                    obj = None if left <= 1 else (i, w, rr, left - 1)
                ops.append({'op': 'scan', 'clock': clock, 'ranges': [enc_float(v) for v in ranges],
                            'angle_min': amin, 'angle_increment': inc,
                            'range_min': enc_float(rmin), 'range_max': enc_float(rmax),
                            'id': f's{k}e{e}'})
            elif c < 0.85:
                ops.append({'op': 'republish', 'clock': clock, 'id': f's{k}e{e}'})
            else:
                speed = rng.choice([-0.4, -0.3, 0.0, -0.0, 0.5, float(np.float32(-0.25)),
                                    rng.uniform(-1, 1)])
                ops.append({'op': 'lane', 'lane': rng.choice(LANES), 'speed': speed,
                            'clock': clock, 'id': f's{k}e{e}'})
            if rng.random() < 0.003:
                ops.append({'op': 'set_state', 'last_hit_time': rng.choice([None, t - 10.0]),
                            'id': f's{k}e{e}x'})
    py, _rs = compare(tally, 'sessions', ops, binary)
    tally.group('sessions')['engages'] = count_engages(py)
    return dict(sessions=n_sessions - refused_py_only, ops=len(ops),
                skipped_missing_launch_value=refused_py_only)


# ---------------------------------------------------------------- bags
def find_bags():
    out = []
    import yaml
    for root in ROOTS:
        for meta in sorted(glob.glob(os.path.join(os.path.expanduser(root), '**',
                                                  'metadata.yaml'), recursive=True)):
            try:
                info = yaml.safe_load(open(meta))['rosbag2_bagfile_information']
            except Exception:  # unreadable metadata: not a parity input
                continue
            counts = {t['topic_metadata']['name']: t['message_count']
                      for t in info.get('topics_with_message_count', [])}
            if counts.get('/scan', 0) > 0:
                out.append((os.path.dirname(meta), info.get('storage_identifier', 'sqlite3'),
                            counts))
    return out


def _bag_messages(bag_dir, storage, topics):
    """(topic, data, t_ns) of the topics, in time order. rosbag2, or for a sqlite3
    bag whose metadata.yaml rosbag2 cannot parse, the .db3 files read-only."""
    import rosbag2_py
    try:
        reader = rosbag2_py.SequentialReader()
        reader.open(rosbag2_py.StorageOptions(uri=bag_dir, storage_id=storage),
                    rosbag2_py.ConverterOptions('', ''))
    except RuntimeError:
        if storage != 'sqlite3':
            raise
        import sqlite3
        rows = []
        for db in sorted(glob.glob(os.path.join(bag_dir, '*.db3'))):
            con = sqlite3.connect(f'file:{db}?mode=ro', uri=True)
            try:
                ids = {i: n for i, n in con.execute('SELECT id, name FROM topics')
                       if n in topics}
                marks = ','.join('?' * len(ids))
                rows += [(ids[tid], bytes(data), t) for tid, t, data in con.execute(
                    f'SELECT topic_id, timestamp, data FROM messages WHERE topic_id IN ({marks})',
                    list(ids))]
            finally:
                con.close()
        rows.sort(key=lambda r: r[2])
        yield from rows
        return
    reader.set_filter(rosbag2_py.StorageFilter(topics=topics))
    while reader.has_next():
        yield reader.read_next()


def read_bag(bag_dir, storage):
    """(events [(t_ns, kind, payload)], laser x from /tf_static, recorded hard stops [t_ns])."""
    from ackermann_msgs.msg import AckermannDriveStamped
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import LaserScan
    from tf2_msgs.msg import TFMessage
    topics = ['/scan', '/tf_static', '/safety_stop'] + ['/' + lane for lane in LANES]
    events, laser_x, recorded = [], None, []
    for topic, data, t in _bag_messages(bag_dir, storage, topics):
        if topic == '/scan':
            m = deserialize_message(data, LaserScan)
            events.append((t, 'scan', m))
        elif topic == '/tf_static':
            for tr in deserialize_message(data, TFMessage).transforms:
                if tr.child_frame_id.endswith('laser') and tr.header.frame_id == 'base_link':
                    laser_x = tr.transform.translation.x
        elif topic == '/safety_stop':
            m = deserialize_message(data, AckermannDriveStamped)
            if m.header.frame_id == 'base_link/hard_stop':
                recorded.append(t)
        else:
            m = deserialize_message(data, AckermannDriveStamped)
            events.append((t, topic[1:], float(m.drive.speed)))
    events.sort(key=lambda e: e[0])
    return events, laser_x, recorded


def scan_payload(m):
    return {'ranges': [enc_float(r) for r in m.ranges], 'angle_min': float(m.angle_min),
            'angle_increment': float(m.angle_increment), 'range_min': float(m.range_min),
            'range_max': float(m.range_max)}


def bag_ops(name, events, variant, rng, tick_phase_ns=None):
    """The node's view of one bag: its scans and lanes, plus a 20 Hz tick.

    The ticks start one period after the first message, or, with
    tick_phase_ns, at that phase of the 50 ms grid (the car's own timer).
    """
    if not events:
        return []
    params = dict(NODE)
    if variant == 'no_reverse_gate':
        params['hard_stop_allow_reverse'] = False
    elif variant == 'no_mask':
        params['footprint_self_margin_m'] = -1.0   # an empty polygon: nothing is a self-hit
    ops = [{'op': 'node', 'params': params, 'id': f'{name}/{variant}'}]
    t0 = events[0][0]
    t_end = events[-1][0]
    period = 50_000_000
    tick = t0 + period
    if tick_phase_ns is not None:
        tick = t0 + (tick_phase_ns - t0) % period
    inject = None
    reverse_until = -1
    k = 0
    for t, kind, payload in events + [(t_end + 1, 'end', None)]:
        while tick < t and tick <= t_end:
            ops.append({'op': 'republish', 'clock': [tick * 1e-9], 'id': f'{name}/{variant}/r{k}'})
            k += 1
            tick += period
        if kind == 'end':
            break
        if kind == 'scan':
            p = payload if isinstance(payload, dict) else scan_payload(payload)
            if variant == 'injected':
                if inject is None and rng.random() < 1 / 120:   # one episode every ~3 s at 40 Hz
                    n = len(p['ranges'])
                    deg = rng.choice([0.0, rng.uniform(-25, 25), rng.uniform(-135, 135)])
                    i = int(round((math.radians(deg) - p['angle_min']) / p['angle_increment']))
                    inject = [min(max(i, 0), max(n - 1, 0)), rng.randint(1, 8),
                              float(np.float32(rng.uniform(0.10, 0.23))), rng.randint(1, 80)]
                    if rng.random() < 0.5:
                        reverse_until = t + int(rng.uniform(0.2, 2.0) * 1e9)
                if inject is not None:
                    i, w, rr, left = inject
                    ranges = list(p['ranges'])
                    ranges[i:i + w] = [rr] * len(ranges[i:i + w])
                    p = dict(p, ranges=ranges)
                    inject = None if left <= 1 else [i, w, rr, left - 1]
            ops.append(dict(p, op='scan', clock=[t * 1e-9], id=f'{name}/{variant}/s{k}'))
            if variant == 'injected' and t < reverse_until and k % 2 == 0:
                ops.append({'op': 'lane', 'lane': 'drive', 'speed': float(np.float32(-0.3)),
                            'clock': [t * 1e-9], 'id': f'{name}/{variant}/v{k}'})
        else:
            ops.append({'op': 'lane', 'lane': kind, 'speed': payload, 'clock': [t * 1e-9],
                        'id': f'{name}/{variant}/l{k}'})
        k += 1
    return ops


def self_hit_stats(events, laser_x=0.305, trigger=0.2):
    """Returns per scan: within the radius, inside the mask (self-hits), outside (triggers)."""
    poly = (NODE['footprint_rear_x_m'] - 0.02, NODE['footprint_front_x_m'] + 0.02,
            -(NODE['footprint_half_width_m'] + 0.02), NODE['footprint_half_width_m'] + 0.02)
    near = self_hits = trig = scans_self = scans_trig = 0
    min_self = math.inf
    for _t, kind, m in events:
        if kind != 'scan':
            continue
        r = np.asarray(m.ranges, dtype=np.float64)
        b = m.angle_min + np.arange(len(r)) * m.angle_increment
        ok = np.isfinite(r) & (r >= m.range_min) & (r <= m.range_max) & (r <= trigger)
        if not ok.any():
            continue
        x = laser_x + r[ok] * np.cos(b[ok])
        y = r[ok] * np.sin(b[ok])
        inside = (poly[0] <= x) & (x <= poly[1]) & (poly[2] <= y) & (y <= poly[3])
        near += int(ok.sum())
        self_hits += int(inside.sum())
        trig += int((~inside).sum())
        scans_self += int(inside.any())
        scans_trig += int((~inside).any())
        if inside.any():
            min_self = min(min_self, float(r[ok][inside].min()))
    return dict(returns_within_0p2=near, self_hit_returns=self_hits, trigger_returns=trig,
                scans_with_self_hits=scans_self, scans_with_triggers=scans_trig,
                min_self_hit_range=None if math.isinf(min_self) else min_self)


def car_tick_phase(events, recorded, period=50_000_000):
    """The car's republish phase (ns mod 50 ms) from its recorded stops.

    A recorded hard stop more than 3 ms after the last scan the bag received
    came from the 20 Hz timer, not from a scan callback; the mode of those
    times modulo the period (1 ms bins) is the timer's phase.
    """
    import bisect
    scans = [t for t, k, _p in events if k == 'scan']
    bins = {}
    for r in recorded:
        i = bisect.bisect_right(scans, r) - 1
        if i >= 0 and r - scans[i] > 3_000_000:
            b = (r % period) // 1_000_000
            bins[b] = bins.get(b, 0) + 1
    if not bins:
        return None, 0
    best = max(bins, key=bins.get)
    near = [r % period for r in recorded
            if abs((r % period) // 1_000_000 - best) <= 1]
    return int(sorted(near)[len(near) // 2]), sum(bins.values())


def stop_episodes(times_ns, gap_ns=120_000_000):
    """[(start, end)] of runs of stops no more than gap apart."""
    eps = []
    for t in sorted(times_ns):
        if eps and t - eps[-1][1] <= gap_ns:
            eps[-1][1] = t
        else:
            eps.append([t, t])
    return eps


def section_bags(tally, binary, rng, max_bags, only=None):
    bags = find_bags()
    if only:
        bags = [b for b in bags if only in b[0]]
    if max_bags:
        bags = bags[:max_bags]
    per_bag, recorded_rows = [], []
    for n, (bag, storage, counts) in enumerate(bags):
        name = os.path.relpath(bag, os.path.expanduser('~'))
        try:
            events, laser_x, recorded = read_bag(bag, storage)
        except Exception as exc:  # a broken bag is reported, not fatal
            per_bag.append(dict(bag=name, error=repr(exc)))
            continue
        row = dict(bag=name, scans=sum(1 for e in events if e[1] == 'scan'),
                   lane_msgs={lane: sum(1 for e in events if e[1] == lane) for lane in LANES},
                   negative_drive=sum(1 for e in events if e[1] == 'drive' and e[2] < 0),
                   tf_laser_x=laser_x, recorded_hard_stops=len(recorded))
        row.update(self_hit_stats(events))
        # Scan payloads once, shared by the variants.
        events = [(t, k, scan_payload(p) if k == 'scan' else p) for t, k, p in events]
        for variant in ('default', 'no_reverse_gate', 'no_mask', 'injected'):
            ops = bag_ops(name, events, variant, random.Random(f'{name}/{variant}'))
            py, _rs = compare(tally, f'bags:{variant}', ops, binary)
            g = tally.group(f'bags:{variant}')
            eng = count_engages(py)
            g['engages'] += eng
            stops = [op['clock'][0] for op, o in zip(ops, py) if o.get('stop')]
            row[f'{variant}_engages'] = eng
            row[f'{variant}_stops'] = len(stops)
            if variant == 'default' and recorded:
                def rel(eps, t0=recorded[0]):
                    return [[round((a - t0) / 1e9, 3), round((b - t0) / 1e9, 3)] for a, b in eps]
                phase, n_ticks = car_tick_phase(events, recorded)
                # The same bag again, the ticks on the car's own phase (gated too).
                ops_c = bag_ops(name, events, 'default', None, tick_phase_ns=phase)
                py_c, _rs_c = compare(tally, 'bags:car_tick_phase', ops_c, binary)
                stops_c = [op['clock'][0] for op, o in zip(ops_c, py_c) if o.get('stop')]
                rec = stop_episodes(recorded)
                rep = stop_episodes([int(round(t * 1e9)) for t in stops])
                rep_c = stop_episodes([int(round(t * 1e9)) for t in stops_c])
                recorded_rows.append(dict(
                    bag=name, recorded_msgs=len(recorded), replay_stops=len(stops),
                    replay_stops_car_phase=len(stops_c),
                    car_tick_phase_ms=None if phase is None else round(phase / 1e6, 2),
                    timer_stops_used=n_ticks,
                    recorded_episodes=rel(rec), replay_episodes=rel(rep),
                    replay_episodes_car_phase=rel(rep_c)))
        per_bag.append(row)
        print(f'bag {n + 1}/{len(bags)} {name}: {row["scans"]} scans, self-hit scans '
              f'{row["scans_with_self_hits"]}, diffs so far '
              f'{tally.total_diffs({"bags"})}', flush=True)
    return dict(bags=len(bags), per_bag=per_bag), recorded_rows


# ---------------------------------------------------------------- timing
def section_timing(binary, events_scan):
    """Per-scan callback compute, Python _scan_cb vs Rust on_scan (no ROS)."""
    from sensor_msgs.msg import LaserScan
    out = []
    for label, payload in events_scan:
        s = PySession()
        s.run({'op': 'node', 'params': NODE, 'id': 'b'})
        msg = LaserScan()
        msg.angle_min = payload['angle_min']
        msg.angle_increment = payload['angle_increment']
        msg.range_min = payload['range_min']
        msg.range_max = payload['range_max']
        msg.ranges = [dec_float(v) for v in payload['ranges']]
        node = s.node
        t = [1000.0]
        node._now = lambda: t[0]
        node.stop_pub.publish = lambda m: None
        node.diag_pub.publish = lambda m: None
        reps = 300
        batches = []
        for _ in range(5):
            t0 = time.perf_counter()
            for _ in range(reps):
                t[0] += 0.025
                node._scan_cb(msg)
            batches.append((time.perf_counter() - t0) / reps * 1e9)
        s.close()
        py_ns = sorted(batches)[2]
        rs, _dt, _e = run_rust([{'op': 'fn', 'fn': 'bench_scan', 'id': 'b', 'args': dict(
            payload, params=NODE, repeat=3000)}], binary)
        rs_ns = rs[0]['out']['ns_per_scan']
        out.append(dict(scan=label, py_us=round(py_ns / 1e3, 2), rs_us=round(rs_ns / 1e3, 3),
                        ratio=round(py_ns / rs_ns, 1)))
    return out


# ---------------------------------------------------------------- main
def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--replay', default=None, help='hard_stop_replay binary')
    ap.add_argument('--quick', action='store_true', help='small fuzz, 3 bags')
    ap.add_argument('--bags', type=int, default=0, help='max bags (0: all)')
    ap.add_argument('--seed', type=int, default=20261023)
    ap.add_argument('--out', default='output/rust/parity_hard_stop.json')
    ap.add_argument('--sections', default='tests,format,functions,sessions,bags,timing')
    ap.add_argument('--only-bags', default=None,
                    help='substring: run only the bags whose path contains it (debugging)')
    a = ap.parse_args()
    binary = a.replay or replay_binary()
    rng = random.Random(a.seed)
    rclpy.init()
    tally = Tally()
    report = dict(seed=a.seed, replay=binary,
                  git_head=os.popen('git rev-parse HEAD').read().strip())
    t_start = time.time()
    try:
        _o, _d, err = run_rust([{'op': 'deps', 'id': 0}], binary)
        report['replay_build'] = err.splitlines()[0] if err else None
        run = set(a.sections.split(','))
        if 'tests' in run:
            report['tests'] = section_tests(tally, binary)
            print('tests:', tally.group('tests')['diffs'], flush=True)
        if 'format' in run:
            report['format'] = section_format(tally, binary, rng, 300 if a.quick else 4000)
            print('format diffs:', tally.group('format')['diffs'],
                  tally.group('format')['examples'][:5], flush=True)
        if 'functions' in run:
            report['functions'] = section_functions(tally, binary, rng,
                                                    300 if a.quick else 6000)
            print('functions diffs:', tally.group('functions')['diffs'], flush=True)
        if 'sessions' in run:
            report['sessions'] = section_sessions(tally, binary, rng, 40 if a.quick else 600,
                                                  150 if a.quick else 300)
            print('sessions diffs:', tally.group('sessions')['diffs'], flush=True)
        if 'bags' in run:
            report['bags'], report['recorded'] = section_bags(
                tally, binary, rng, 3 if a.quick and not a.bags else a.bags, a.only_bags)
        if 'timing' in run:
            # A blank scan, a trigger scan and a car scan (the first bag's first).
            blank = {'ranges': [5.0] * 1081,
                     'angle_min': float(np.float32(-2.356194496154785)),
                     'angle_increment': float(np.float32(0.004363323096185923)),
                     'range_min': float(np.float32(0.02)), 'range_max': 30.0}
            trig = dict(blank, ranges=[5.0] * 540 + [0.18] * 3 + [5.0] * 538)
            samples = [('blank 1081 beams', blank), ('trigger ahead (3 beams)', trig)]
            bags = find_bags()
            if bags:
                ev, _x, _r = read_bag(bags[0][0], bags[0][1])
                first = next((p for _t, k, p in ev if k == 'scan'), None)
                if first is not None:
                    samples.append(
                        (f'car scan ({os.path.basename(os.path.dirname(bags[0][0]))})',
                         scan_payload(first)))
            report['timing'] = section_timing(binary, samples)
    finally:
        rclpy.try_shutdown()
    report['groups'] = tally.groups
    gated = {'tests', 'format', 'functions', 'sessions', 'bags'}
    report['total_diffs'] = tally.total_diffs(gated)
    report['total_ops'] = sum(g['ops'] for g in tally.groups.values())
    report['seconds'] = round(time.time() - t_start, 1)
    ok = report['total_diffs'] == 0
    if 'tests' in report:
        ok = ok and report['tests']['current'] and not report['tests']['forbidden_deps']
    report['sections'] = a.sections
    report['gate'] = 'PASS' if ok else 'FAIL'
    os.makedirs(os.path.dirname(a.out) or '.', exist_ok=True)
    with open(a.out, 'w') as f:
        json.dump(report, f, indent=1, default=str)
    for k, g in tally.groups.items():
        print(f'{k:28s} ops {g["ops"]:8d} diffs {g["diffs"]:4d} scans {g["scans"]:7d} '
              f'stops {g["stops"]:7d} engages {g["engages"]:5d} py {g["py_s"]:7.1f}s '
              f'rs {g["rs_s"]:6.1f}s')
    print(f'TOTAL {report["total_ops"]} ops, {report["total_diffs"]} differences; '
          f'test cases current: {report.get("tests", {}).get("current")}; '
          f'gate {report["gate"]}')
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
