#!/usr/bin/env python3
"""Sim health: how late /scan arrives in Isaac, from bags recorded by scenario.sh (RECORD=1).

Per bag: the gap between consecutive /scan messages on the RECEIVE clock (the recorder's wall
time) and on the header stamps (simulated time), as histograms; how many wall gaps exceed the
guard's 200 ms timeout and how much simulated time passed in them (a simulator pause: little; lost
scans: as much); the real-time factor from /clock against wall time.

Usage: scan_gaps.py BAG_DIR ... [--out FILE]
"""

import argparse
import json
import sys

import numpy as np

EDGES_MS = [0, 20, 25, 30, 40, 50, 75, 100, 150, 200, 250, 500, 1000, 1e9]


def read(bag):
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosgraph_msgs.msg import Clock
    from sensor_msgs.msg import LaserScan
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=bag, storage_id='mcap'), rosbag2_py.ConverterOptions('', ''))
    r.set_filter(rosbag2_py.StorageFilter(topics=['/scan', '/clock']))
    scan, clock = [], []
    while r.has_next():
        topic, data, t = r.read_next()
        if topic == '/scan':
            h = deserialize_message(data, LaserScan).header.stamp
            scan.append((t, h.sec * 1_000_000_000 + h.nanosec))
        else:
            c = deserialize_message(data, Clock).clock
            clock.append((t, c.sec * 1_000_000_000 + c.nanosec))
    return np.array(scan, dtype=np.int64), np.array(clock, dtype=np.int64)


def hist(gaps_ms):
    counts, _ = np.histogram(gaps_ms, bins=EDGES_MS)
    labels = [f'{int(a)}-{int(b)}' if b < 1e9 else f'>{int(a)}' for a, b in zip(EDGES_MS, EDGES_MS[1:])]
    return {lab: int(c) for lab, c in zip(labels, counts)}


def one(bag):
    scan, clock = read(bag)
    wall = np.diff(scan[:, 0]) / 1e6
    sim = np.diff(scan[:, 1]) / 1e6
    late = np.flatnonzero(wall > 200.0)
    rtf = None
    if len(clock) > 1:
        rtf = float((clock[-1, 1] - clock[0, 1]) / (clock[-1, 0] - clock[0, 0]))
    return dict(
        bag=bag, scans=int(len(scan)), seconds_wall=float((scan[-1, 0] - scan[0, 0]) / 1e9),
        rate_hz_wall=float((len(scan) - 1) / ((scan[-1, 0] - scan[0, 0]) / 1e9)),
        rtf=rtf,
        wall_gap_ms=dict(p50=float(np.median(wall)), p95=float(np.percentile(wall, 95)),
                         p99=float(np.percentile(wall, 99)), max=float(wall.max())),
        sim_gap_ms=dict(p50=float(np.median(sim)), p95=float(np.percentile(sim, 95)),
                        max=float(sim.max())),
        wall_hist=hist(wall), sim_hist=hist(sim),
        over_timeout=[dict(wall_ms=round(float(wall[i]), 1), sim_ms=round(float(sim[i]), 1))
                      for i in late],
        sim_gaps_over_timeout=int(np.count_nonzero(sim > 200.0)))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('bags', nargs='+')
    ap.add_argument('--out', default=None)
    a = ap.parse_args()
    out = [one(b) for b in a.bags]
    for r in out:
        print(json.dumps({k: v for k, v in r.items() if k not in ('wall_hist', 'sim_hist')}))
    if a.out:
        with open(a.out, 'w') as f:
            json.dump(out, f, indent=1)
    return 0


if __name__ == '__main__':
    sys.exit(main())
