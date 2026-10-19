#!/usr/bin/env python3
"""
door_replay.py: door_detection replayed offline on a recorded bag.

Usage (no ROS environment needed; numpy, scipy, matplotlib, rosbags):
    python3 tools/door_replay.py [--out DIR] [--every N] [--no-frames] BAG

BAG is a rosbag2 directory (sqlite3). It is read with sqlite3 directly and
decoded with rosbags' CDR deserialisers, the repo's own .msg files registered
for f1tenth_messages. Never rclpy.

WHAT IS REPLAYED. lidar_front_wall_node's door cycle, with the node's own
pure functions: every Nth /scan (default 4, the tracker's own decimation)
in base_link with the laser pose from /tf_static; the car pose in odom from
/odometry/filtered at the scan stamp; the wall tracks the car itself
published on /perception/walls nearest that stamp (rebuilt as
multi_wall_tracker.Track); then door_candidates (tracked walls + frame
pieces) -> pane evidence -> DoorTracker, as on the car. Object sectors are
not cut (no detections are replayed), so 'object' rejects cannot occur.

OUTPUTS in --out: <bag>.doors.json (every door track's life, the reject
codes, the pane memory, per-cycle counts and cost) and <bag>.doors.png (the
accumulated scan in odom with the frame pieces, the doors and the panes).
"""

import argparse
import collections
import json
import math
from pathlib import Path
import sqlite3
import sys
import time

import numpy as np

REPO = Path(__file__).resolve().parents[1]
for sub in ('src/f1tenth_perception', 'src/corridor_perception', 'src/f1tenth_params'):
    sys.path.insert(0, str(REPO / sub))

from corridor_perception.geometry import Pose2D  # noqa: E402, I100
from f1tenth_perception import door_detection as dd  # noqa: E402
from f1tenth_perception import multi_wall_tracker as mwt  # noqa: E402

from rosbags.typesys import get_types_from_msg, get_typestore, Stores  # noqa: E402

TOPICS = ('/scan', '/tf_static', '/odometry/filtered', '/perception/walls')


def typestore():
    store = get_typestore(Stores.ROS2_HUMBLE)
    types = {}
    for path in sorted((REPO / 'src/f1tenth_messages/msg').glob('*.msg')):
        types.update(get_types_from_msg(path.read_text(), f'f1tenth_messages/msg/{path.stem}'))
    store.register(types)
    return store


def read_bag(bag, store):
    out = {t: [] for t in TOPICS}
    for path in sorted(Path(bag).glob('*.db3')):
        con = sqlite3.connect(f'file:{path}?mode=ro', uri=True)
        want = {tid: (name, typ) for tid, name, typ in
                con.execute('select id, name, type from topics') if name in TOPICS}
        for tid, data in con.execute(
                'select topic_id, data from messages order by timestamp'):
            if tid in want:
                name, typ = want[tid]
                out[name].append(store.deserialize_cdr(data, typ))
        con.close()
    return out


def stamp(msg):
    return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def laser_pose(tf_static, frame, base='base_link'):
    """(x, y, yaw) of `frame` in base_link from /tf_static (planar chain)."""
    edges = {}
    for msg in tf_static:
        for tr in msg.transforms:
            edges[tr.child_frame_id] = (tr.header.frame_id, tr.transform)
    x, y, yaw = 0.0, 0.0, 0.0
    while frame != base:
        parent, tf = edges[frame]
        c, s = math.cos(yaw_of(tf.rotation)), math.sin(yaw_of(tf.rotation))
        x, y = tf.translation.x + c * x - s * y, tf.translation.y + s * x + c * y
        yaw += yaw_of(tf.rotation)
        frame = parent
    return x, y, yaw


def tracks_from(msg):
    """Return the published MultiWallTracks of one message as multi_wall_tracker.Track."""
    out = []
    for t in msg.tracks:
        tan = np.array([-math.sin(t.theta), math.cos(t.theta)])
        a = tan @ np.array([t.p0.x, t.p0.y])
        b = tan @ np.array([t.p1.x, t.p1.y])
        out.append(mwt.Track(
            id=int(t.id), x=np.array([t.rho, t.theta], dtype=float), P=np.eye(2) * 1e-3,
            t0=float(min(a, b)), t1=float(max(a, b)), born=0.0, last_seen=0.0,
            ever_confirmed=t.state in (1, 2), state=int(t.state)))
    return out


def nearest(seq, t, key=stamp):
    lo, hi = 0, len(seq) - 1
    while lo < hi:
        mid = (lo + hi) // 2
        if key(seq[mid]) < t:
            lo = mid + 1
        else:
            hi = mid
    best = min((i for i in (lo - 1, lo) if 0 <= i < len(seq)), key=lambda i: abs(key(seq[i]) - t))
    return seq[best]


def replay(bag, out_dir, every=4, frames=True):
    store = typestore()
    data = read_bag(bag, store)
    name = Path(bag).name
    p = dd.DoorParams(frame_jambs=frames)
    lpose = laser_pose(data['/tf_static'], data['/scan'][0].header.frame_id)
    odom = data['/odometry/filtered']
    tracker = dd.DoorTracker(p)
    life = {}
    codes = collections.Counter()
    acc, piece_xy, cycles, ms = [], [], [], []
    t0 = stamp(data['/scan'][0])
    for k, scan in enumerate(data['/scan']):
        if k % every:
            continue
        t = stamp(scan)
        om = nearest(odom, t)
        pp = om.pose.pose
        car = Pose2D(pp.position.x, pp.position.y, yaw_of(pp.orientation))
        walls = (tracks_from(nearest(data['/perception/walls'], t))
                 if data['/perception/walls'] else [])
        sc = mwt.scan_in_base_link(scan.ranges, scan.angle_min, scan.angle_increment,
                                   scan.range_min, scan.range_max, lpose, (), with_cut=True)
        sc = (sc[0], sc[1], lpose[:2], sc[3])
        tic = time.perf_counter()
        rejects = []
        pieces = dd.scan_pieces(sc, p) if frames else []
        cands = dd.door_candidates(walls, car, sc, p, rejects, pieces)
        doors = tracker.step(cands, car, t, scan=sc)
        ms.append((time.perf_counter() - tic) * 1e3)
        for code, _ids, _w in rejects:
            codes[code] += 1
        for dr in doors:
            rec = life.setdefault(dr.id, {'born': round(t - t0, 2), 'states': set()})
            rec['states'].add(dd.STATE_NAMES[dr.state])
            rec['last'] = round(t - t0, 2)
            rec['centre'] = [round(float(v), 2) for v in dr.centre]
            rec['width'] = round(dr.width, 3)
            rec['parents'] = list(dr.parents)
            rec['leaf'] = dd.LEAF_NAMES[dr.leaf_side]
        cycles.append({'t': round(t - t0, 2), 'candidates': len(cands), 'pieces': len(pieces),
                       'doors': len(doors),
                       'confirmed': sum(1 for dr in doors if dr.state == dd.STATE_CONFIRMED)})
        r = np.asarray(sc[0])
        ok = np.isfinite(r)
        pts = np.column_stack([lpose[0] + r[ok] * np.cos(sc[1][ok]),
                               lpose[1] + r[ok] * np.sin(sc[1][ok])])
        pts = pts[np.hypot(pts[:, 0], pts[:, 1]) < 6.0]
        acc.append(car.transform_points(pts))
        for pc in pieces:
            piece_xy.append(car.transform_points(np.vstack([pc.a, pc.b])))
    for rec in life.values():
        rec['states'] = sorted(rec['states'])
    confirmed = {k: v for k, v in life.items() if 'confirmed' in v['states']}
    summary = {
        'bag': name, 'every': every, 'frame_jambs': frames, 'cycles': len(cycles),
        'door_tracks': len(life), 'confirmed_doors': len(confirmed),
        'frame_doors_confirmed': sum(1 for v in confirmed.values() if 0 in v['parents']),
        'panes_remembered': len(tracker.panes), 'pane_kills': tracker.n_pane_kills,
        'reject_codes': dict(codes.most_common()),
        'ms_linus': {'p50': round(float(np.percentile(ms, 50)), 2),
                     'p95': round(float(np.percentile(ms, 95)), 2)},
        'doors': {str(k): v for k, v in sorted(confirmed.items())},
        'panes': [{'centre': [round(float(v), 2) for v in pn.centre], 'width': round(pn.width, 3)}
                  for pn in tracker.panes],
    }
    out = Path(out_dir)
    out.mkdir(parents=True, exist_ok=True)
    (out / f'{name}.doors{"" if frames else "_noframes"}.json').write_text(
        json.dumps(summary, indent=1))
    draw(out / f'{name}.doors{"" if frames else "_noframes"}.png', acc, piece_xy, confirmed,
         tracker.panes, odom)
    return summary


def draw(path, acc, piece_xy, doors, panes, odom):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    P = np.vstack(acc)
    fig, ax = plt.subplots(figsize=(26, 12))
    ax.scatter(P[::3, 0], P[::3, 1], s=0.3, c='#b0b0b0')
    for seg in piece_xy[::2]:
        ax.plot(seg[:, 0], seg[:, 1], color='#3060ff', lw=0.6, alpha=0.15)
    path_xy = np.array([[m.pose.pose.position.x, m.pose.pose.position.y] for m in odom])
    ax.plot(path_xy[:, 0], path_xy[:, 1], 'k-', lw=1)
    for pn in panes:
        ax.plot([pn.jamb0[0], pn.jamb1[0]], [pn.jamb0[1], pn.jamb1[1]], color='#00b0d0', lw=3)
    for k, v in doors.items():
        c = np.array(v['centre'])
        ax.plot(*c, 'o', color='#10c040', ms=9)
        ax.text(c[0] + 0.05, c[1] + 0.08, f'{k} {v["width"]:.2f}', fontsize=8, color='#108030')
    ax.set_aspect('equal')
    ax.grid(True, alpha=0.3)
    ax.set_title('door replay: grey scan (odom), blue frame pieces, green confirmed doors, '
                 'cyan panes')
    fig.tight_layout()
    fig.savefig(path, dpi=60)
    plt.close(fig)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', default='output/perception/door_replay')
    ap.add_argument('--every', type=int, default=4)
    ap.add_argument('--no-frames', action='store_true')
    ap.add_argument('bag')
    args = ap.parse_args()
    s = replay(args.bag, args.out, args.every, not args.no_frames)
    print(json.dumps({k: v for k, v in s.items() if k not in ('doors', 'panes')}, indent=1))


if __name__ == '__main__':
    main()
