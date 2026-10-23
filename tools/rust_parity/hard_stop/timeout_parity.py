#!/usr/bin/env python3
"""Timeout parity: what the Python and the Rust hard_stop_node do when inputs stop.

Like tools/rust_parity/proximity/timeout_parity.py, on an isolated localhost
domain (live_common.py), four real nodes at once:
  py_car / rs_car   use_sim_time false (the car: system clock)
  py_sim / rs_sim   use_sim_time true; /clock comes from this driver at 200 Hz
and the same /live/scan and /drive to all four. /clock advances with wall time
unless frozen (a simulator pause).

scan cases: 0.6 s of scans (the last ones CLEAR or TRIPPING, i.e. engaged),
  a gap of G ms without scans (clock running or frozen), 1 s of clear scans.
  Per node: stops in the gap, the first and the last gap stop after the last
  scan (the republish keeps the lane claimed; the clear hold releases it on
  the node clock), stops after the scans resume.
lane cases: engaged on an object ahead with /drive -0.4 after every scan (the
  reverse gate keeps the node quiet), then /drive goes silent while the
  object stays; the clock is frozen for the first F ms of the silence. Per
  node: when it claims the lane again (lane_timeout_s on the node clock).

The pairs (py_car, rs_car) and (py_sim, rs_sim) must agree: counts within 2
(the republish timers' phases differ), times within 60 ms (30 ms for the lane).

Usage (workspace sourced): timeout_parity.py [--recorded-ms 260] [--out FILE]
"""

import argparse
import json
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import live_common as lc  # noqa: E402

TAGS = ('py_car', 'rs_car', 'py_sim', 'rs_sim')
PAIRS = (('py_car', 'rs_car'), ('py_sim', 'rs_sim'))


def ms(t):
    return None if t is None else round(t * 1e3, 1)


def scan_case(d, pre, clock, gap_ms):
    d.stream(lc.CLEAR, 0.8)
    m = d.mark()
    last = d.stream(lc.TRIP_AHEAD if pre == 'trip' else lc.CLEAR, 0.6)[-1]
    d.freeze(clock == 'frozen')
    time.sleep(gap_ms / 1000.0)
    d.freeze(False)
    resume = d.stream(lc.CLEAR, 1.0)
    time.sleep(0.1)
    stops, diags = d.since(m)
    row = dict(kind='scan', pre=pre, clock=clock, gap_ms=gap_ms)
    for t in TAGS:
        ts = [s[0] for s in stops[t] if s[0] > last + 0.005]
        gap = [x for x in ts if x < resume[0]]
        after = [x for x in ts if x >= resume[0]]
        row[t] = dict(gap_stops=len(gap),
                      first_gap_stop_ms=ms(gap[0] - last) if gap else None,
                      last_gap_stop_ms=ms(gap[-1] - last) if gap else None,
                      after_resume_stops=len(after),
                      last_stop_ms=ms(ts[-1] - last) if ts else None,
                      engaged_at_end=json.loads(diags[t][-1][1])['engaged'] if diags[t] else None)
    return row


def lane_case(d, freeze_ms):
    d.stream(lc.CLEAR, 0.8)
    m = d.mark()
    d.stream(lc.TRIP_AHEAD, 0.3)
    d.stream(lc.TRIP_AHEAD, 0.6, drive=-0.4, drive_every=1)
    last_drive = d.last_drive
    if freeze_ms:
        d.freeze(True)
        d.stream(lc.TRIP_AHEAD, freeze_ms / 1000.0)
        d.freeze(False)
    d.stream(lc.TRIP_AHEAD, 0.6)
    d.stream(lc.CLEAR, 1.0)
    time.sleep(0.1)
    stops, _diags = d.since(m)
    row = dict(kind='lane', freeze_ms=freeze_ms)
    for t in TAGS:
        back = [s[0] for s in stops[t] if s[0] > last_drive + 0.03]
        row[t] = dict(claim_again_ms=ms(back[0] - last_drive) if back else None)
    return row


def compare(row):
    out = {}
    for a, b in PAIRS:
        ra, rb = row[a], row[b]
        ok = True
        for k, va in ra.items():
            vb = rb[k]
            if va is None or vb is None or isinstance(va, bool):
                ok &= va == vb
            elif k.endswith('_stops'):
                ok &= abs(va - vb) <= 2
            else:
                ok &= abs(va - vb) <= (30 if row['kind'] == 'lane' else 60)
        out[f'{a}~{b}'] = bool(ok)
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--recorded-ms', type=float, default=260.0,
                    help='the Isaac pause the guard parity replayed (232-282 ms)')
    ap.add_argument('--out', default='output/rust/timeout_parity_hard_stop.json')
    ap.add_argument('--log-dir', default=None)
    a = ap.parse_args()
    env = lc.isolated_env()
    log_dir = a.log_dir or os.path.join(os.path.dirname(os.path.abspath(a.out)), 'live_logs')
    os.makedirs(log_dir, exist_ok=True)
    procs = {t: lc.start_node('py' if t.startswith('py') else 'rust', t, env, log_dir,
                              sim=t.endswith('sim')) for t in TAGS}
    d = lc.Driver(TAGS, sim_clock=True)
    rows = []
    try:
        if not d.wait_ready(len(TAGS)):
            raise RuntimeError('nodes not discovered within 30 s')
        d.stream(lc.CLEAR, 3.0)       # discovery, /clock
        for pre in ('clear', 'trip'):
            for clock in ('running', 'frozen'):
                for g in (150.0, 250.0, 500.0, 1000.0, float(a.recorded_ms)):
                    row = scan_case(d, pre, clock, g)
                    row['agree'] = compare(row)
                    rows.append(row)
                    print(json.dumps(row), flush=True)
        for f in (0, 150, 400):
            row = lane_case(d, f)
            row['agree'] = compare(row)
            rows.append(row)
            print(json.dumps(row), flush=True)
        alive = {t: procs[t].poll() is None for t in TAGS}
        shutdown = None
    finally:
        d.close()
        shutdown = lc.stop_nodes(procs)
    ok = all(all(r['agree'].values()) for r in rows) and all(alive.values())
    os.makedirs(os.path.dirname(a.out) or '.', exist_ok=True)
    with open(a.out, 'w') as f:
        json.dump(dict(domain=env['ROS_DOMAIN_ID'], recorded_ms=a.recorded_ms, rows=rows,
                       alive_at_end=alive, shutdown=shutdown, pass_=ok), f, indent=1)
    print(f'{sum(all(r["agree"].values()) for r in rows)}/{len(rows)} cases agree; '
          f'{"PASS" if ok else "FAIL"}')
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
