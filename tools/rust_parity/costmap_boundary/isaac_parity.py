#!/usr/bin/env python3
"""costmap_boundary parity on recorded Isaac runs: what the live node published vs the other side.

Input: bags written by tools/rust_parity/isaac/scenario.sh with RECORD=1, of a run with
impl:=py or impl:=rust. For every /costmap/boundaries message the live node published:
  - inputs it had at that tick: the latest /slam/map received before it (and the one before, for
    a map that arrived during the tick), and as pose candidates the last 4
    /ekf_global/odometry/filtered messages stamped at or before the tick's own stamp (+20 ms):
    the node uses the latest pose it received, and the recorder cannot know which one that was;
  - the OTHER implementation, offline, on each candidate (impl:=py run -> the Rust replay;
    impl:=rust run -> the real Python _extraction_tick), and the SAME implementation as a check
    of the candidate method;
  - match: some candidate reproduces count, order, normals and offsets (1e-12) and the paired
    /costmap/front_clearance (0 Float32 ulps). Unmatched ticks are listed.
Also per run: frame_id, stamp spacing (sim time), rate, and the offline tick time of both
implementations on these inputs (p50/p95/max).

Usage (workspace sourced): isaac_parity.py --run py:DIR --run rust:DIR ... [--out FILE]
"""

import argparse
import json
import math
import os
import subprocess
import sys
import time

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from parity_costmap_boundary import PythonRef  # noqa: E402

TOL = 1e-12


def read_bag(bag):
    import rosbag2_py
    from f1tenth_messages.msg import BoundaryConstraintArray
    from nav_msgs.msg import OccupancyGrid, Odometry
    from rclpy.serialization import deserialize_message
    from std_msgs.msg import Float32
    types = {'/slam/map': OccupancyGrid, '/ekf_global/odometry/filtered': Odometry,
             '/costmap/boundaries': BoundaryConstraintArray, '/costmap/front_clearance': Float32}
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=bag, storage_id='mcap'), rosbag2_py.ConverterOptions('', ''))
    r.set_filter(rosbag2_py.StorageFilter(topics=list(types)))
    out = {k: [] for k in types}
    while r.has_next():
        topic, data, t = r.read_next()
        out[topic].append((t, deserialize_message(data, types[topic])))
    return out


def stamp(h):
    return h.stamp.sec + h.stamp.nanosec * 1e-9


def yaw(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def f32_ulps(a, b):
    ia = int(np.float32(a).view(np.int32))
    ib = int(np.float32(b).view(np.int32))
    return abs(ia - ib)


def same(rec_cons, rec_clr, cons, clr):
    if len(rec_cons) != len(cons):
        return False
    for a, b in zip(rec_cons, cons):
        if any(abs(x - y) > TOL for x, y in zip(a, b)):
            return False
    if (rec_clr is None) != (clr is None):
        return False
    return rec_clr is None or f32_ulps(rec_clr, clr) == 0


def analyse(impl, bag_dir, replay, py, params):
    b = read_bag(os.path.join(bag_dir, 'bag'))
    maps, poses = b['/slam/map'], b['/ekf_global/odometry/filtered']
    clr = b['/costmap/front_clearance']
    pose_stamps = [stamp(m.header) for _, m in poses]
    ticks = []
    for t, arr in b['/costmap/boundaries']:
        s = stamp(arr.header)
        cl = [c for tc, c in clr if 0 <= tc - t < 20_000_000]   # same tick: published right after
        mi = [i for i, (tm, _) in enumerate(maps) if tm < t]
        if not mi:
            continue
        cand = [i for i, ps in enumerate(pose_stamps) if ps <= s + 0.02][-4:]
        ticks.append(dict(t=t, stamp=s, frame=arr.header.frame_id,
                          cons=[[c.normal[0], c.normal[1], c.offset] for c in arr.constraints],
                          clr=float(cl[0].data) if cl else None,
                          maps=sorted({mi[-1], mi[-2]} if len(mi) > 1 else {mi[-1]}),
                          poses=cand))
    # Rust offline: one replay line per map with every candidate pose.
    def pose_of(i):
        p = poses[i][1].pose.pose
        return [p.position.x, p.position.y, yaw(p.orientation)]
    rust = {}
    rs_us = []
    for mi in sorted({m for tk in ticks for m in tk['maps']}):
        g = maps[mi][1]
        pids = sorted({p for tk in ticks if mi in tk['maps'] for p in tk['poses']})
        line = json.dumps(dict(id=str(mi), width=g.info.width, height=g.info.height,
                               resolution=float(g.info.resolution),
                               origin_x=g.info.origin.position.x, origin_y=g.info.origin.position.y,
                               data=[int(v) for v in g.data], poses=[pose_of(p) for p in pids],
                               **params))
        out = subprocess.run([replay], input=line + '\n', capture_output=True, text=True,
                             check=True).stdout.splitlines()
        for p, o in zip(pids, out):
            o = json.loads(o)
            rs_us.append(o.get('compute_us', float('nan')))
            rust[(mi, p)] = ([[float(v) for v in c] for c in o['constraints']],
                             None if o['front_clearance'] is None else float(o['front_clearance']))
    pycache = {}

    def py_out(mi, p):
        if (mi, p) not in pycache:
            r = py.run(maps[mi][1], pose_of(p))
            pycache[(mi, p)] = (r['constraints'], r['front_clearance'])
        return pycache[(mi, p)]
    res = dict(impl=impl, ticks=len(ticks), cross_match=0, self_match=0, unmatched=[],
               latest_pose_used=0, matched_rank={}, discriminating_ticks=0,
               discriminating_matched=0)
    for tk in ticks:
        def match(fn):
            for mi in reversed(tk['maps']):
                for rank, p in enumerate(reversed(tk['poses'])):
                    cons, c = fn(mi, p)
                    if same(tk['cons'], tk['clr'], cons, c):
                        return rank
            return None
        other = (lambda mi, p: rust[(mi, p)]) if impl == 'py' else py_out
        mine = py_out if impl == 'py' else (lambda mi, p: rust[(mi, p)])
        r_other, r_self = match(other), match(mine)
        # Discriminating tick: the candidates do not all give the same output, so a match
        # singles out inputs (a moving car), instead of matching any of identical candidates.
        outs = {json.dumps(other(mi, p)) for mi in tk['maps'] for p in tk['poses']}
        if len(outs) > 1:
            res['discriminating_ticks'] += 1
            res['discriminating_matched'] += int(r_other is not None)
        key = 'none' if r_other is None else str(r_other)
        res['matched_rank'][key] = res['matched_rank'].get(key, 0) + 1
        res['cross_match'] += int(r_other is not None)
        res['self_match'] += int(r_self is not None)
        res['latest_pose_used'] += int(r_other == 0)
        if r_other is None and len(res['unmatched']) < 10:
            res['unmatched'].append(dict(stamp=tk['stamp'], recorded=tk['cons'], clr=tk['clr']))
    st = np.array([tk['stamp'] for tk in ticks])
    d = np.diff(st) if len(st) > 1 else np.array([0.0])
    wall = (ticks[-1]['t'] - ticks[0]['t']) / 1e9 if len(ticks) > 1 else 0.0
    res.update(frame_ids=sorted({tk['frame'] for tk in ticks}),
               constraints_per_tick={str(k): int(v) for k, v in
                                     zip(*np.unique([len(tk['cons']) for tk in ticks],
                                                    return_counts=True))},
               stamp_step_s=dict(p50=float(np.median(d)), min=float(d.min()), max=float(d.max())),
               rate_hz_wall=round((len(ticks) - 1) / wall, 2) if wall else None,
               stamps_monotonic=bool(np.all(d > 0)),
               maps=len({m for tk in ticks for m in tk['maps']}),
               rust_offline_us=dict(p50=float(np.percentile(rs_us, 50)),
                                    p95=float(np.percentile(rs_us, 95)), max=float(max(rs_us))))
    return res


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--run', action='append', required=True, help='py:DIR or rust:DIR')
    ap.add_argument('--out', default='output/rust/isaac_costmap_parity.json')
    a = ap.parse_args()
    from ament_index_python.packages import get_package_prefix
    replay = os.path.join(get_package_prefix('f1tenth_costmap_boundary_rs'), 'lib',
                          'f1tenth_costmap_boundary_rs', 'costmap_boundary_replay')
    py = PythonRef()
    params = py.params()
    out = []
    for spec in a.run:
        impl, d = spec.split(':', 1)
        py.tick_s.clear()
        r = analyse(impl, d, replay, py, params)
        t = np.array(py.tick_s) * 1e3
        r['python_offline_tick_ms'] = dict(p50=float(np.percentile(t, 50)),
                                           p95=float(np.percentile(t, 95)), max=float(t.max()),
                                           n=len(t))
        r['run'] = d
        out.append(r)
        print(json.dumps({k: v for k, v in r.items() if k != 'unmatched'}), flush=True)
    with open(a.out, 'w') as f:
        json.dump(out, f, indent=1)
    return 0 if all(r['cross_match'] == r['ticks'] for r in out) else 1


if __name__ == '__main__':
    sys.exit(main())
