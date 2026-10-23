#!/usr/bin/env python3
"""TF tree of the running graph: one publisher per edge, one parent per frame, values.

Run as a Discovery Server super client, e.g.
    ROS_SUPER_CLIENT=TRUE python3 tools/sim/tf_owners.py --json out.json --md out.md

Jazzy's rclpy MessageInfo carries no publisher GID, so the owner of an edge is checked by
its effects instead of by name:
  - /tf_static: every latched publisher delivers its message once (TRANSIENT_LOCAL), so an
    edge carried by more than one /tf_static message has more than one publisher;
  - /tf: two publishers of one edge show up as one stamp with two different values;
  - --odom-check: every odom->base_link on /tf must equal the EKF's /odometry/filtered pose
    of the stamp --odom-offset earlier (robot_localization's transform_time_offset), so
    robot_localization, not the simulator, owns it.
Also: every child frame has exactly one parent, and
  --pose parent:child:x,y,z[,qx,qy,qz,qw] compares an edge's values within --tol.
/tf publisher node names (graph API) are listed for the record. Exit code 1 if a check fails.
"""
import argparse
import json
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from nav_msgs.msg import Odometry
from tf2_msgs.msg import TFMessage


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--listen', type=float, default=8.0)
    ap.add_argument('--odom-check', default='/odometry/filtered',
                    help="EKF output topic odom->base_link must match ('' to skip)")
    ap.add_argument('--odom-offset', type=float, default=0.05,
                    help='robot_localization transform_time_offset of the local EKF '
                         '(ekf.yaml: 0.05 s): the TF is stamped this much after the output')
    ap.add_argument('--pose', nargs='*', default=[])
    ap.add_argument('--tol', type=float, default=1e-3)
    ap.add_argument('--json')
    ap.add_argument('--md')
    a = ap.parse_args()

    rclpy.init()
    node = Node('tf_owners')
    edges = {}   # (parent, child) -> {'static_msgs', 'dyn_msgs', 'stamps': {stamp: value}, ...}
    ekf = {}     # stamp -> (x, y, z, qx, qy, qz, qw) of the EKF output
    odom_tf = {}

    def _val(t):
        tr, q = t.transform.translation, t.transform.rotation
        return tuple(round(v, 6) for v in (tr.x, tr.y, tr.z, q.x, q.y, q.z, q.w))

    def cb(static):
        def _cb(msg):
            for t in msg.transforms:
                key = (t.header.frame_id.lstrip('/'), t.child_frame_id.lstrip('/'))
                e = edges.setdefault(key, {'static_msgs': 0, 'dyn_msgs': 0, 'stamps': {},
                                           'conflicts': 0})
                e['static_msgs' if static else 'dyn_msgs'] += 1
                e['last'] = t.transform
                st = (t.header.stamp.sec, t.header.stamp.nanosec)
                v = _val(t)
                if not static:
                    if st in e['stamps'] and e['stamps'][st] != v:
                        e['conflicts'] += 1
                    e['stamps'][st] = v
                    if key == ('odom', 'base_link'):
                        odom_tf[st] = v
        return _cb

    def on_ekf(m):
        p, q = m.pose.pose.position, m.pose.pose.orientation
        ekf[(m.header.stamp.sec, m.header.stamp.nanosec)] = tuple(
            round(v, 6) for v in (p.x, p.y, p.z, q.x, q.y, q.z, q.w))

    static_qos = QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                            reliability=ReliabilityPolicy.RELIABLE)
    node.create_subscription(TFMessage, '/tf', cb(False), 100)
    node.create_subscription(TFMessage, '/tf_static', cb(True), static_qos)
    if a.odom_check:
        node.create_subscription(Odometry, a.odom_check, on_ekf, 100)
    t_end = time.time() + a.listen
    while time.time() < t_end:
        rclpy.spin_once(node, timeout_sec=0.1)
    tf_pubs = sorted(i.node_namespace.rstrip('/') + '/' + i.node_name
                     for i in node.get_publishers_info_by_topic('/tf'))
    static_pubs = sorted(i.node_namespace.rstrip('/') + '/' + i.node_name
                         for i in node.get_publishers_info_by_topic('/tf_static'))
    node.destroy_node()
    rclpy.shutdown()

    failures = []
    rows = []
    parents = {}
    for (p, c), e in sorted(edges.items()):
        tr, q = e['last'].translation, e['last'].rotation
        rows.append({'parent': p, 'child': c, 'static_msgs': e['static_msgs'],
                     'dyn_msgs': e['dyn_msgs'], 'conflicting_stamps': e['conflicts'],
                     'xyz': [tr.x, tr.y, tr.z], 'q': [q.x, q.y, q.z, q.w]})
        parents.setdefault(c, set()).add(p)
        if e['static_msgs'] > 1:
            failures.append(f'{p}->{c}: carried by {e["static_msgs"]} /tf_static messages')
        if e['static_msgs'] and e['dyn_msgs']:
            failures.append(f'{p}->{c}: on both /tf_static and /tf')
        if e['conflicts']:
            failures.append(f'{p}->{c}: {e["conflicts"]} stamps with two different values')
    for c, ps in parents.items():
        if len(ps) != 1:
            failures.append(f'{c}: {len(ps)} parents {sorted(ps)}')
    odom = {}
    if a.odom_check:
        off = int(round(a.odom_offset * 1e9))

        def back(st):
            ns = st[0] * 1_000_000_000 + st[1] - off
            return (ns // 1_000_000_000, ns % 1_000_000_000)
        pairs = {st: back(st) for st in odom_tf if back(st) in ekf}
        common = sorted(pairs)
        diff = [s_ for s_ in common
                if max(abs(x - y) for x, y in zip(odom_tf[s_], ekf[pairs[s_]])) > 1e-4]
        odom = {'tf_msgs': len(odom_tf), 'ekf_msgs': len(ekf), 'stamp_offset_s': a.odom_offset,
                'matched_stamps': len(common),
                'value_mismatch': len(diff),
                'mismatch_samples': [{'stamp': s_, 'tf': odom_tf[s_], 'ekf': ekf[pairs[s_]]}
                                     for s_ in diff[:3]]}
        if not odom_tf:
            failures.append('odom->base_link: not on /tf')
        elif not common or diff:
            failures.append(f'odom->base_link vs {a.odom_check}: {odom}')
    for spec in a.pose:
        p, c, vals = spec.split(':', 2)
        want = [float(v) for v in vals.split(',')]
        r = next((r for r in rows if (r['parent'], r['child']) == (p, c)), None)
        if r is None:
            failures.append(f'pose {p}->{c}: edge not seen')
            continue
        got = r['xyz'] + (r['q'] if len(want) > 3 else [])
        if any(abs(g - w) > a.tol for g, w in zip(got, want)):
            failures.append(f'pose {p}->{c}: got {[round(g, 6) for g in got]} want {want}')
    out = {'edges': len(rows), 'failures': failures, 'odom_check': odom,
           'tf_publishers': tf_pubs, 'tf_static_publishers': static_pubs, 'rows': rows}
    if a.json:
        with open(a.json, 'w') as f:
            json.dump(out, f, indent=1)
    if a.md:
        with open(a.md, 'w') as f:
            f.write('| parent -> child | /tf_static msgs | /tf msgs | xyz | q (x,y,z,w) |\n'
                    '|---|---|---|---|---|\n')
            for r in rows:
                f.write(f"| {r['parent']} -> {r['child']} | {r['static_msgs']} | "
                        f"{r['dyn_msgs']} | "
                        f"{', '.join(f'{v:.4f}' for v in r['xyz'])} | "
                        f"{', '.join(f'{v:.5f}' for v in r['q'])} |\n")
            f.write(f'\n/tf publishers: {", ".join(tf_pubs)}\n\n'
                    f'/tf_static publishers: {", ".join(static_pubs)}\n\n'
                    f'odom->base_link vs EKF output: {odom}\n')
            f.write('\nchecks: ' + ('PASS' if not failures else 'FAIL: ' + '; '.join(failures))
                    + '\n')
    print(json.dumps({'edges': len(rows), 'odom_check': odom, 'failures': failures}))
    return 1 if failures else 0


if __name__ == '__main__':
    sys.exit(main())
