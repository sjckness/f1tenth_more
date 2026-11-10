#!/usr/bin/env python3
"""
Timings the car recorded about itself, read from a mission bag (perf baseline).

    bag_reference.py <bag_dir> [--out file.json]

The stack publishes a few of its own stage times: /mpc/solver_status
(solve_dt_sec), the tracker and door cycle in /perception/virtual_wall/debug
(tracker_ms, door_ms, rect_ms), the scene mask tick in
/perception/scene_labels/stats (ms) and the scene labels tick in
/perception/scene_labels_debug (ms). A bag recorded on the car therefore holds
the ON-CAR cost of those stages, for the code the car ran that day -- the
reference for how a linus number transfers to the Orin. Reads the sqlite3 file
directly; plays nothing.
"""

import argparse
import json
import pathlib
import sqlite3
import sys

import numpy as np
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

JSON_TOPICS = {
    '/perception/virtual_wall/debug': ('tracker_ms', 'door_ms', 'rect_ms'),
    '/perception/scene_labels/stats': ('ms',),
    '/perception/scene_labels_debug': ('ms',),
}


def stats(values):
    a = np.asarray([v for v in values if isinstance(v, (int, float)) and v == v], dtype=float)
    if a.size == 0:
        return None
    return {'n': int(a.size), 'p50': round(float(np.percentile(a, 50)), 3),
            'p95': round(float(np.percentile(a, 95)), 3), 'max': round(float(a.max()), 3)}


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    ap.add_argument('bag')
    ap.add_argument('--out')
    args = ap.parse_args(argv)
    bag = pathlib.Path(args.bag).expanduser()
    db_path = next(bag.glob('*.db3'))
    db = sqlite3.connect(f'file:{db_path}?mode=ro', uri=True)
    topics = {n: (i, t) for i, n, t in db.execute('select id, name, type from topics')}
    t0, t1 = db.execute('select min(timestamp), max(timestamp) from messages').fetchone()
    out = {'bag': str(bag), 'duration_s': round((t1 - t0) / 1e9, 1), 'stages_ms': {},
           'rates_hz': {}}

    def rows(name):
        return db.execute('select data from messages where topic_id = ? order by timestamp',
                          (topics[name][0],))

    for name, keys in JSON_TOPICS.items():
        if name not in topics:
            continue
        cls = get_message(topics[name][1])
        payloads = []
        for (data,) in rows(name):
            try:
                payloads.append(json.loads(deserialize_message(data, cls).data))
            except Exception:
                continue
        for key in keys:
            st = stats([p.get(key) for p in payloads])
            if st:
                out['stages_ms'][f'{name}:{key}'] = st
    name = '/mpc/solver_status'
    if name in topics:
        try:
            cls = get_message(topics[name][1])
            msgs = [deserialize_message(data, cls) for (data,) in rows(name)]
            st = stats([m.solve_dt_sec * 1e3 for m in msgs if m.solve_dt_sec > 0.0])
            if st:
                out['stages_ms'][f'{name}:solve_dt_sec'] = st
                out['mpc'] = {'solver': sorted({m.solver for m in msgs}),
                              'ticks': len(msgs),
                              'ticks_with_a_solve': sum(m.solve_dt_sec > 0.0 for m in msgs)}
        except Exception as exc:
            out['mpc'] = {'error': f'{name} not decodable with this workspace: {exc!r}'}
    dur = (t1 - t0) / 1e9
    for name, (tid, _type) in sorted(topics.items()):
        n = db.execute('select count(*) from messages where topic_id = ?', (tid,)).fetchone()[0]
        if n:
            out['rates_hz'][name] = round(n / dur, 2)
    text = json.dumps(out, indent=1)
    if args.out:
        pathlib.Path(args.out).write_text(text + '\n')
    print(json.dumps({k: out[k] for k in ('duration_s', 'stages_ms') if k in out}, indent=1))
    print(out.get('mpc'))
    return 0


if __name__ == '__main__':
    sys.exit(main())
