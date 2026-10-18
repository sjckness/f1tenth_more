#!/usr/bin/env python3
"""Parity: IsProximityTooClose's front check (Python) vs proximity_guard (Rust).

Feeds IDENTICAL inputs to
  - the Python: f1tenth_behavior.behaviours.is_proximity_too_close.
    IsProximityTooClose._scan_callback (the real class, a stand-in node),
    front decision = latest_lidar_front_min < front_distance_threshold, and
  - the Rust: proximity_guard_replay (f1tenth_proximity_guard::decision, the
    code the node runs),
and diffs the decisions. Cases: synthetic scans (obstacle in/out of the cone,
at the threshold +-1 cm, cone edges, NaN/+-inf, below range_min, empty, no
TF, odd layouts, random fuzz) and every /scan of the recorded bags, each
also with a synthetic obstacle injected near the threshold.

Gate: `tripped` (the Python decision) differs on zero cases. The Rust
`stop` may differ from the Python only through a named fail-safe reason.

Usage (workspace sourced, ~/dev_ws/jc_env.sh first on linus):
  tools/rust_parity/proximity/parity_proximity.py [--bags N] [--out FILE.json]
"""

import argparse
import glob
import json
import math
import os
import random
import subprocess
import sys
import time

import numpy as np

from f1tenth_behavior.behaviours.is_proximity_too_close import IsProximityTooClose
from sensor_msgs.msg import LaserScan

FRONT_X = 0.405          # base_front.xacro base_front_x (2026-10-13)
THRESHOLD = 0.15         # stack_params proximity_front_threshold_m
CONE_DEG = 45.0          # create_root() default front_cone_half_angle_deg
LASER = (0.305, 0.0, 0.0)  # description.launch.py base_link -> laser
UST = dict(angle_min=-2.356194496154785, angle_increment=0.004363323096185923,
           range_min=0.019999999552965164, n=1081)

ARCHIVES = ['~/f1tenth_archive/mission_row', '~/f1tenth_sim_archive/mission_row']


# ---------------------------------------------------------------- Python side
class _Logger:
    def info(self, *a, **k):
        pass

    warn = warning = error = debug = info


class _Node:
    def get_logger(self):
        return _Logger()


_PY = {}


def python_front(case):
    """The real IsProximityTooClose on one case -> (tripped, min_front)."""
    key = (case['threshold_m'], case['cone_half_angle_deg'], case['front_x'])
    b = _PY.get(key)
    if b is None:
        b = IsProximityTooClose(front_distance_threshold=case['threshold_m'],
                                front_cone_half_angle_deg=case['cone_half_angle_deg'],
                                base_front_x_m=case['front_x'])
        b.node = _Node()
        _PY[key] = b
    msg = LaserScan()
    msg.header.frame_id = 'laser'
    msg.angle_min = case['angle_min']
    msg.angle_increment = case['angle_increment']
    msg.range_min = case['range_min']
    msg.range_max = 30.0
    msg.ranges = [_decode(r) for r in case['ranges']]
    b.latest_lidar_front_min = None
    b._frames._poses.clear()
    if case['laser'] is not None:
        # The pose static_frames.StaticFramePoses would have resolved.
        b._frames._poses['laser'] = tuple(case['laser'])
    b._scan_callback(msg)
    m = b.latest_lidar_front_min
    return (m is not None and m < b.front_distance_threshold), m


# ---------------------------------------------------------------- case format
def _encode(r):
    r = float(r)
    if math.isnan(r):
        return 'nan'
    if math.isinf(r):
        return 'inf' if r > 0 else '-inf'
    return r


def _decode(r):
    return float(r)  # float('nan'), float('inf'), float('-inf') included


def f32(x):
    return float(np.float32(x))


def make_case(cid, ranges, laser=LASER, angle_min=UST['angle_min'],
              angle_increment=UST['angle_increment'], range_min=UST['range_min'],
              front_x=FRONT_X, threshold=THRESHOLD, cone=CONE_DEG, group='synthetic'):
    return dict(id=cid, group=group, angle_min=f32(angle_min),
                angle_increment=f32(angle_increment), range_min=f32(range_min),
                ranges=[_encode(np.float32(r)) for r in ranges],
                laser=None if laser is None else [float(v) for v in laser],
                front_x=front_x, threshold_m=threshold, cone_half_angle_deg=cone)


def beam_angles(n=UST['n'], angle_min=UST['angle_min'], inc=UST['angle_increment']):
    return f32(angle_min) + np.arange(n, dtype=np.float64) * f32(inc)


def range_for_bumper_point(theta, d, laser=LASER, front_x=FRONT_X):
    """Range along beam theta whose return is d from (front_x, 0), or None."""
    lx, ly, lyaw = laser
    c, s = math.cos(lyaw + theta), math.sin(lyaw + theta)
    # |(lx + r c - fx, ly + r s)| = d  ->  r^2 + 2 r (ax c + ly s) + (ax^2 + ly^2 - d^2) = 0
    ax = lx - front_x
    b = 2.0 * (ax * c + ly * s)
    cc = ax * ax + ly * ly - d * d
    disc = b * b - 4.0 * cc
    if disc < 0:
        return None
    r = (-b + math.sqrt(disc)) / 2.0
    return r if r > 0 else None


def synthetic_cases(n_fuzz, seed):
    rng = random.Random(seed)
    ang = beam_angles()
    base = np.full(UST['n'], 30.0)
    cases = []

    def add(cid, ranges, **kw):
        cases.append(make_case(cid, ranges, **kw))

    add('open', base)
    # Threshold +-1 cm (and exact), straight ahead and off-axis.
    for deg in (0.0, 10.0, 20.0, 30.0, 40.0, -15.0, -35.0):
        i = int(np.argmin(np.abs(ang - math.radians(deg))))
        for dd, tag in ((-0.01, 'minus1cm'), (0.0, 'exact'), (0.01, 'plus1cm'),
                        (-0.001, 'minus1mm'), (0.001, 'plus1mm')):
            r = range_for_bumper_point(ang[i], THRESHOLD + dd)
            if r is None:
                continue
            x = base.copy()
            x[i] = r
            add(f'threshold_{tag}_beam{deg:+.0f}deg', x)
    # Obstacle in / out of the cone, near and far.
    for deg in (0, 5, 25, 44, 46, 60, 90, 120, -30, -50, -100):
        i = int(np.argmin(np.abs(ang - math.radians(deg))))
        for r in (0.12, 0.20, 0.25, 0.30, 0.5, 1.0, 5.0):
            x = base.copy()
            x[i] = r
            add(f'single_beam_{deg:+d}deg_r{r:.2f}', x)
    # Cone edges, measured from the BUMPER: points at bumper-bearing 45 +- 0.5 deg.
    fx, (lx, ly, _) = FRONT_X, LASER
    for bearing in (44.0, 44.5, 44.9, 45.0, 45.1, 45.5, 46.0, -44.5, -45.5):
        for d in (0.05, 0.10, 0.14):
            px = fx + d * math.cos(math.radians(bearing))
            py = d * math.sin(math.radians(bearing))
            theta = math.atan2(py - ly, px - lx)
            i = int(np.argmin(np.abs(ang - theta)))
            x = base.copy()
            # The range that puts the return on that exact bearing along beam i.
            x[i] = math.hypot(px - lx, py - ly)
            add(f'cone_edge_{bearing:+.1f}deg_d{d:.2f}', x)
    # A wall straight ahead at distances around the threshold (many beams).
    for d in (0.10, 0.14, 0.149, 0.15, 0.151, 0.16, 0.30):
        x = base.copy()
        wall_x = fx + d   # base_link x of the wall plane
        for i, a in enumerate(ang):
            c = math.cos(a)
            if c > 1e-6:
                x[i] = min(x[i], (wall_x - lx) / c)
        add(f'wall_ahead_{d:.3f}', x)
    # Behind the bumper / beside the body.
    for r, deg in ((0.05, 0), (0.09, 0), (0.15, 80), (0.2, 100)):
        i = int(np.argmin(np.abs(ang - math.radians(deg))))
        x = base.copy()
        x[i] = r
        add(f'behind_or_beside_r{r}_{deg}deg', x)
    # Non-finite and sub-range_min values.
    i0 = int(np.argmin(np.abs(ang)))
    for tag, val in (('nan', math.nan), ('pinf', math.inf), ('ninf', -math.inf),
                     ('below_range_min', 0.01), ('zero', 0.0)):
        x = base.copy()
        x[i0] = val
        add(f'one_{tag}_ahead', x)
        x = base.copy()
        x[np.abs(ang) < math.radians(45)] = val
        add(f'front_all_{tag}', x)
        add(f'all_{tag}', np.full(UST['n'], val))
        x = base.copy()
        x[i0] = val
        x[i0 + 3] = 0.20   # a real trip next to it
        add(f'{tag}_next_to_trip', x)
    # Half / just over half the front sector NaN.
    sector = np.flatnonzero(np.abs(ang) < math.radians(45))
    for frac in (0.4, 0.5, 0.6):
        x = base.copy()
        x[sector[:int(len(sector) * frac)]] = math.nan
        add(f'front_nan_fraction_{frac}', x)
    add('empty', [])
    # No /tf_static yet: the Python's (0, 0, 0) stand-in.
    for r in (0.3, 0.45, 0.5, 0.6):
        x = base.copy()
        x[i0] = r
        add(f'no_tf_r{r}', x, laser=None)
    # Odd layouts: reversed scan, rear-facing mount, offset mount, few beams.
    rev = base.copy()[::-1].copy()
    rev[540] = 0.2
    add('reversed_increment_trip', rev, angle_min=-UST['angle_min'],
        angle_increment=-UST['angle_increment'])
    x = base.copy()
    x[0] = 0.2      # beam -135 deg of a laser mounted yaw=pi points... backwards-left
    add('rear_mount_yaw_pi', x, laser=(0.1, 0.0, math.pi))
    x = base.copy()
    x[i0] = 0.21
    add('offset_mount_y', x, laser=(0.305, 0.05, 0.1))
    add('three_beams_ahead', [0.3, 0.2, 0.3], angle_min=-0.01, angle_increment=0.01)
    add('rear_only_layout', [0.1] * 10, angle_min=3.0, angle_increment=0.01)
    # Random fuzz near the threshold.
    for k in range(n_fuzz):
        x = np.full(UST['n'], rng.choice([30.0, 8.0, 2.0]))
        for _ in range(rng.randint(1, 6)):
            i = rng.randrange(UST['n'])
            w = rng.randint(1, 40)
            x[i:i + w] = rng.uniform(0.05, 0.6)
        if rng.random() < 0.3:
            for _ in range(rng.randint(1, 50)):
                x[rng.randrange(UST['n'])] = rng.choice([math.nan, math.inf, 0.0, 0.01])
        laser = None if rng.random() < 0.1 else (rng.uniform(0.2, 0.4), rng.uniform(-0.03, 0.03),
                                                 rng.uniform(-0.05, 0.05))
        add(f'fuzz_{k:05d}', x, laser=laser)
    return cases


# ---------------------------------------------------------------- bags
def bag_scans(bag_dir):
    """(laser pose or None, [LaserScan]) of one bag, pose from its /tf_static."""
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from f1tenth_behavior.behaviours.static_frames import StaticFramePoses
    from tf2_msgs.msg import TFMessage
    meta = os.path.join(bag_dir, 'metadata.yaml')
    storage = 'mcap' if 'mcap' in open(meta).read().split('storage_identifier:')[1][:20] else 'sqlite3'
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=bag_dir, storage_id=storage),
                rosbag2_py.ConverterOptions('', ''))
    reader.set_filter(rosbag2_py.StorageFilter(topics=['/scan', '/tf_static']))
    frames = StaticFramePoses()
    scans = []
    while reader.has_next():
        topic, data, _ = reader.read_next()
        if topic == '/tf_static':
            frames.on_tf_static(deserialize_message(data, TFMessage))
        else:
            scans.append(deserialize_message(data, LaserScan))
    pose = frames.pose(scans[0].header.frame_id) if scans else None
    return pose, scans


def bag_cases(bag_dir, rng):
    pose, scans = bag_scans(bag_dir)
    name = os.path.basename(os.path.dirname(bag_dir))
    out = []
    for k, m in enumerate(scans):
        r = np.asarray(m.ranges, dtype=np.float64)
        common = dict(laser=pose, angle_min=m.angle_min, angle_increment=m.angle_increment,
                      range_min=m.range_min, group='bag')
        out.append(make_case(f'{name}#{k}', r, **common))
        # The same real scan with a small obstacle near the threshold.
        x = r.copy()
        ang = m.angle_min + np.arange(len(x)) * m.angle_increment
        cand = np.flatnonzero(np.abs(ang) < math.radians(50))
        if len(cand):
            i = int(rng.choice(cand))
            rr = range_for_bumper_point(ang[i], THRESHOLD + rng.uniform(-0.02, 0.02),
                                        laser=pose or (0.0, 0.0, 0.0))
            if rr is not None:
                x[i:i + rng.randint(1, 4)] = rr
                common['group'] = 'bag+obstacle'
                out.append(make_case(f'{name}#{k}+obs', x, **common))
    return out


def find_bags(topic='/scan'):
    """Bag directories whose metadata counts at least one `topic` message."""
    import yaml
    bags = []
    for root in ARCHIVES:
        for meta in sorted(glob.glob(os.path.join(os.path.expanduser(root), '*', 'bag',
                                                  'metadata.yaml'))):
            try:
                info = yaml.safe_load(open(meta))['rosbag2_bagfile_information']
            except Exception:  # unreadable metadata: not a parity input
                continue
            counts = {t['topic_metadata']['name']: t['message_count']
                      for t in info.get('topics_with_message_count', [])}
            if counts.get(topic, 0) > 0:
                bags.append(os.path.dirname(meta))
    return bags


# ---------------------------------------------------------------- compare
def run_rust(replay, cases):
    payload = '\n'.join(json.dumps({k: v for k, v in c.items() if k != 'group'})
                        for c in cases) + '\n'
    t0 = time.perf_counter()
    p = subprocess.run([replay], input=payload, capture_output=True, text=True, check=False)
    dt = time.perf_counter() - t0
    if p.returncode != 0:
        raise RuntimeError(f'replay failed: {p.stderr[:2000]}')
    out = [json.loads(line) for line in p.stdout.splitlines() if line.strip()]
    if len(out) != len(cases):
        raise RuntimeError(f'replay returned {len(out)} outcomes for {len(cases)} cases')
    return out, dt


def compare(cases, rust, stats):
    for c, r in zip(cases, rust):
        assert c['id'] == r['id'], (c['id'], r['id'])
        t0 = time.perf_counter()
        py_trip, py_min = python_front(c)
        stats['py_time_s'] += time.perf_counter() - t0
        g = stats['groups'].setdefault(c['group'], dict(cases=0, py_trips=0, rs_trips=0,
                                                        trip_diff=0, stop_ne_py=0))
        g['cases'] += 1
        g['py_trips'] += int(py_trip)
        g['rs_trips'] += int(r['tripped'])
        if py_trip != r['tripped']:
            g['trip_diff'] += 1
            stats['trip_diffs'].append(dict(id=c['id'], py=py_trip, rs=r['tripped'],
                                            py_min=py_min, rs_min=r['min_front']))
        if (py_min is None) != (r['min_front'] is None):
            stats['min_presence_diffs'].append(c['id'])
        elif py_min is not None:
            d = abs(py_min - r['min_front'])
            if d > stats['max_min_abs_diff']:
                stats['max_min_abs_diff'] = d
                stats['max_min_case'] = c['id']
            stats['min_bitexact'] += int(py_min == r['min_front'])
            stats['min_compared'] += 1
        if r['stop'] != py_trip:
            g['stop_ne_py'] += 1
            key = r['reason']
            stats['fail_safe_reasons'][key] = stats['fail_safe_reasons'].get(key, 0) + 1
            if r['reason'] in ('clear', 'tripped'):
                stats['unexplained_stop_diffs'].append(c['id'])
            if len(stats['fail_safe_examples'].setdefault(key, [])) < 5:
                stats['fail_safe_examples'][key].append(c['id'])


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--replay', default=None, help='proximity_guard_replay binary')
    ap.add_argument('--fuzz', type=int, default=3000)
    ap.add_argument('--bags', type=int, default=0, help='max bags (0: all)')
    ap.add_argument('--seed', type=int, default=20261018)
    ap.add_argument('--out', default='output/rust/parity_proximity.json')
    a = ap.parse_args()
    replay = a.replay
    if replay is None:
        from ament_index_python.packages import get_package_prefix
        replay = os.path.join(get_package_prefix('f1tenth_proximity_guard'), 'lib',
                              'f1tenth_proximity_guard', 'proximity_guard_replay')
    stats = dict(groups={}, trip_diffs=[], min_presence_diffs=[], max_min_abs_diff=0.0,
                 max_min_case=None, min_bitexact=0, min_compared=0, fail_safe_reasons={},
                 fail_safe_examples={}, unexplained_stop_diffs=[], py_time_s=0.0,
                 rs_time_s=0.0, bags=[], seed=a.seed)
    cases = synthetic_cases(a.fuzz, a.seed)
    rust, dt = run_rust(replay, cases)
    stats['rs_time_s'] += dt
    compare(cases, rust, stats)
    print(f'synthetic: {len(cases)} cases, trip diffs {len(stats["trip_diffs"])}', flush=True)

    rng = random.Random(a.seed)
    bags = find_bags()
    if a.bags:
        bags = bags[:a.bags]
    for n, bag in enumerate(bags):
        try:
            cases = bag_cases(bag, rng)
        except Exception as exc:  # a broken bag is reported, not fatal
            stats['bags'].append(dict(bag=bag, error=repr(exc)))
            continue
        if not cases:
            stats['bags'].append(dict(bag=bag, cases=0))
            continue
        rust, dt = run_rust(replay, cases)
        stats['rs_time_s'] += dt
        compare(cases, rust, stats)
        stats['bags'].append(dict(bag=bag, cases=len(cases)))
        if n % 20 == 0:
            print(f'bag {n + 1}/{len(bags)}: trip diffs so far {len(stats["trip_diffs"])}',
                  flush=True)
    os.makedirs(os.path.dirname(a.out) or '.', exist_ok=True)
    with open(a.out, 'w') as f:
        json.dump(stats, f, indent=1, default=str)
    total = sum(g['cases'] for g in stats['groups'].values())
    print(json.dumps({k: stats[k] for k in ('groups', 'max_min_abs_diff', 'min_bitexact',
                                             'min_compared', 'fail_safe_reasons')}, indent=1))
    print(f'TOTAL {total} cases, trip diffs {len(stats["trip_diffs"])}, '
          f'unexplained stop diffs {len(stats["unexplained_stop_diffs"])}')
    return 0 if not stats['trip_diffs'] and not stats['unexplained_stop_diffs'] else 1


if __name__ == '__main__':
    sys.exit(main())
