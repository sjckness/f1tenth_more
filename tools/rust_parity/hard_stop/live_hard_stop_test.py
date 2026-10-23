#!/usr/bin/env python3
"""Live: the Python and the Rust hard_stop_node side by side, same inputs, same stops.

An isolated localhost ROS domain (live_common.py), the two real nodes as
processes (car clock: use_sim_time false), one driver publishing the same
/live/scan messages (the car's 1081-beam layout) and /drive commands to both:

  A trigger   clear, then an object 0.18 m ahead, then clear: both engage on
              the same scan (the 2nd, debounce 2), publish the same stops
              (speed 0, steering 0, frame_id base_link/hard_stop) and release
              on the same scan; every /safety/hard_stop JSON identical, scan by scan;
  B dead      the LiDAR stops while engaged: both keep the lane claimed on the
              20 Hz republish and release on the clear hold, ~0.5 s after the
              last hit, with no scan;
  C reverse   engaged ahead, /drive -0.4: both go quiet; /drive 0.0: both re-claim
              at once (in the lane callback);
  D lane      /drive -0.4 then silence: both claim again lane_timeout_s (0.2 s)
              after the last command;
  E cpu       20 s at 40 Hz with an object every 3 s: CPU of each process;
  QoS         the publishers' and subscriptions' QoS of both nodes, from the graph;
  params      the parameter services: all 15 parameters listed and equal on both.

Writes output/rust/live_hard_stop.json (and the nodes' logs to --log-dir).
Exit 1 if a check fails.

Usage (workspace sourced): live_hard_stop_test.py [--out FILE] [--log-dir DIR]
"""

import argparse
import json
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import live_common as lc  # noqa: E402

TAGS = ('py', 'rs')


def parse(diags):
    return [json.loads(d) for _t, d in diags]


def first(seq, pred, start=0):
    return next((i for i in range(start, len(seq)) if pred(seq[i])), None)


def phase_trigger(d):
    m = d.mark()
    clear = d.stream(lc.CLEAR, 0.6)
    trip = d.stream(lc.TRIP_AHEAD, 1.0)
    after = d.stream(lc.CLEAR, 1.2)
    time.sleep(0.3)
    stops, diags = d.since(m)
    n_scans = len(clear) + len(trip) + len(after)
    res = dict(scans_sent=n_scans, trip_scans=len(trip))
    seqs = {}
    for t in TAGS:
        p = parse(diags[t])
        seqs[t] = [x for _tt, x in diags[t]]
        eng = first(p, lambda x: x['engaged'])
        rel = first(p, lambda x: not x['engaged'], eng) if eng is not None else None
        st = stops[t]
        res[t] = dict(
            diags=len(p), engage_scan=eng, release_scan=rel,
            trigger_count=p[-1]['trigger_count'] if p else None, stops=len(st),
            first_stop_after_2nd_trip_scan_ms=(round((st[0][0] - trip[1]) * 1e3, 2)
                                               if st and len(trip) > 1 else None),
            last_stop_after_last_trip_scan_ms=(round((st[-1][0] - trip[-1]) * 1e3, 1)
                                               if st else None),
            frame_ids=sorted({s[1] for s in st}),
            all_zero=all(s[2] == 0.0 and s[3] == 0.0 for s in st))
    same_len = len(seqs['py']) == len(seqs['rs']) == n_scans
    res['diag_sequences_identical'] = same_len and seqs['py'] == seqs['rs']
    res['diag_differences'] = (sum(a != b for a, b in zip(seqs['py'], seqs['rs']))
                               if same_len else None)
    expected_engage = len(clear) + 1
    res['checks'] = dict(
        same_engage_scan=res['py']['engage_scan'] == res['rs']['engage_scan'] == expected_engage,
        same_release_scan=res['py']['release_scan'] == res['rs']['release_scan'] is not None,
        identical_diags=res['diag_sequences_identical'],
        stop_frame=all(res[t]['frame_ids'] == ['base_link/hard_stop'] and res[t]['all_zero']
                       for t in TAGS),
        stop_counts_close=abs(res['py']['stops'] - res['rs']['stops']) <= 3,
        first_stop_on_the_engaging_scan=all(
            res[t]['first_stop_after_2nd_trip_scan_ms'] is not None
            and 0 <= res[t]['first_stop_after_2nd_trip_scan_ms'] < 15 for t in TAGS))
    return res


def phase_dead(d):
    m = d.mark()
    d.stream(lc.CLEAR, 0.4)
    trip = d.stream(lc.TRIP_AHEAD, 0.3)
    last = trip[-1]
    time.sleep(1.5)
    resume = d.stream(lc.CLEAR, 0.6)
    time.sleep(0.2)
    stops, _diags = d.since(m)
    res = {}
    for t in TAGS:
        gap = [s[0] for s in stops[t] if last + 0.005 < s[0] < resume[0]]
        res[t] = dict(gap_stops=len(gap),
                      last_gap_stop_ms=round((gap[-1] - last) * 1e3, 1) if gap else None,
                      stops_after_resume=sum(1 for s in stops[t] if s[0] >= resume[0]))
    res['checks'] = dict(
        republished_through_the_gap=all(8 <= res[t]['gap_stops'] <= 11 for t in TAGS),
        released_on_the_hold=all(res[t]['last_gap_stop_ms'] is not None
                                 and 420 <= res[t]['last_gap_stop_ms'] <= 560 for t in TAGS),
        same_release_time=(res['py']['last_gap_stop_ms'] is not None
                           and res['rs']['last_gap_stop_ms'] is not None
                           and abs(res['py']['last_gap_stop_ms']
                                   - res['rs']['last_gap_stop_ms']) <= 60),
        quiet_after=all(res[t]['stops_after_resume'] == 0 for t in TAGS))
    return res


def phase_reverse(d):
    m = d.mark()
    d.stream(lc.CLEAR, 0.4)
    d.stream(lc.TRIP_AHEAD, 0.3)
    rev = d.stream(lc.TRIP_AHEAD, 1.0, drive=-0.4, drive_every=2)
    t0 = d.drive(0.0)
    time.sleep(0.03)
    d.stream(lc.TRIP_AHEAD, 0.4, drive=0.0)
    d.stream(lc.CLEAR, 1.0)
    time.sleep(0.2)
    stops, diags = d.since(m)
    res = {}
    for t in TAGS:
        quiet = [s[0] for s in stops[t] if rev[0] + 0.03 < s[0] < t0]
        after = [s[0] for s in stops[t] if s[0] >= t0]
        p = parse(diags[t])
        res[t] = dict(stops_while_reversing=len(quiet),
                      reclaim_ms=round((after[0] - t0) * 1e3, 2) if after else None,
                      passing_reverse_diags=sum(1 for x in p if x['passing_reverse']))
    res['checks'] = dict(
        quiet_while_reversing=all(res[t]['stops_while_reversing'] == 0 for t in TAGS),
        reclaimed_at_once=all(res[t]['reclaim_ms'] is not None and res[t]['reclaim_ms'] < 15
                              for t in TAGS),
        same_passing_count=abs(res['py']['passing_reverse_diags']
                               - res['rs']['passing_reverse_diags']) <= 1)
    return res


def phase_lane(d):
    m = d.mark()
    d.stream(lc.CLEAR, 0.4)
    d.stream(lc.TRIP_AHEAD, 0.3)
    d.stream(lc.TRIP_AHEAD, 0.6, drive=-0.4, drive_every=1)
    last_drive = d.last_drive
    d.stream(lc.TRIP_AHEAD, 0.7)
    d.stream(lc.CLEAR, 1.0)
    time.sleep(0.2)
    stops, _diags = d.since(m)
    res = {}
    for t in TAGS:
        back = [s[0] for s in stops[t] if s[0] > last_drive + 0.05]
        res[t] = dict(claim_again_ms=round((back[0] - last_drive) * 1e3, 1) if back else None)
    res['checks'] = dict(
        lane_timeout=all(res[t]['claim_again_ms'] is not None
                         and 195 <= res[t]['claim_again_ms'] <= 260 for t in TAGS),
        same_time=(None not in (res['py']['claim_again_ms'], res['rs']['claim_again_ms'])
                   and abs(res['py']['claim_again_ms'] - res['rs']['claim_again_ms']) <= 30))
    return res


def phase_cpu(d, procs, seconds=20.0):
    c0 = {t: lc.cpu_seconds(procs[t].pid) for t in TAGS}
    w0 = time.monotonic()
    end = w0 + seconds
    k = 0
    while time.monotonic() < end:
        k += 1
        d.stream(lc.TRIP_AHEAD if k % 10 == 0 else lc.CLEAR, 0.3, period=0.025)
    wall = time.monotonic() - w0
    res = {t: dict(cpu_percent=round((lc.cpu_seconds(procs[t].pid) - c0[t]) / wall * 100, 2))
           for t in TAGS}
    res['wall_s'] = round(wall, 1)
    res['scan_hz'] = 40
    return res


def qos_check(d):
    out = {}
    for name, topic, pubs in (('scan_subscription', '/live/scan', False),
                              ('drive_subscription', '/drive', False),
                              ('teleop_subscription', '/teleop', False),
                              ('calibration_drive_subscription', '/calibration_drive', False)):
        q = d.qos(topic, publishers=pubs)
        out[name] = {k: v for k, v in q.items() if k.startswith('hs_')}
    for kind in ('stop', 'diag'):
        out[f'{kind}_publisher'] = {
            t: list(d.qos(f'/live/{t}_{kind}').values()) for t in TAGS}
    checks = {}
    for name in ('scan_subscription', 'drive_subscription', 'teleop_subscription',
                 'calibration_drive_subscription'):
        v = out[name]
        checks[name] = v.get('hs_py') is not None and v.get('hs_py') == v.get('hs_rs')
    for kind in ('stop', 'diag'):
        v = out[f'{kind}_publisher']
        checks[f'{kind}_publisher'] = len(v['py']) == 1 and v['py'] == v['rs']
    out['checks'] = checks
    return out


HS_PARAMS = ('hard_stop_enabled', 'hard_stop_range_m', 'hard_stop_debounce_scans',
             'hard_stop_clear_s', 'hard_stop_republish_hz', 'scan_topic', 'stop_topic',
             'diag_topic', 'footprint_front_x_m', 'footprint_rear_x_m',
             'footprint_half_width_m', 'footprint_self_margin_m', 'laser_x_m',
             'hard_stop_allow_reverse', 'lane_timeout_s')
#: Per-node by construction (each node has its own output topics here).
PER_NODE = {'stop_topic', 'diag_topic'}


def _call(d, srv_type, name, request, timeout=5.0):
    cli = d.node.create_client(srv_type, name)
    try:
        if not cli.wait_for_service(timeout_sec=timeout):
            return None
        fut = cli.call_async(request)
        end = time.monotonic() + timeout
        while not fut.done() and time.monotonic() < end:
            time.sleep(0.01)
        return fut.result() if fut.done() else None
    finally:
        d.node.destroy_client(cli)


def params_check(d):
    """The parameter services: the 15 parameters listed and read back on both nodes."""
    from rcl_interfaces.msg import ParameterType
    from rcl_interfaces.srv import GetParameters, ListParameters
    out = {}
    for t in TAGS:
        lst = _call(d, ListParameters, f'/hs_{t}/list_parameters', ListParameters.Request())
        req = GetParameters.Request()
        req.names = list(HS_PARAMS)
        got = _call(d, GetParameters, f'/hs_{t}/get_parameters', req)
        values = {}
        if got is not None:
            for n, v in zip(HS_PARAMS, got.values):
                values[n] = {ParameterType.PARAMETER_BOOL: v.bool_value,
                             ParameterType.PARAMETER_INTEGER: v.integer_value,
                             ParameterType.PARAMETER_DOUBLE: v.double_value,
                             ParameterType.PARAMETER_STRING: v.string_value}.get(v.type, None)
        out[t] = dict(names=sorted(lst.result.names) if lst is not None else None,
                      values=values)
    same = {n: out['py']['values'].get(n) == out['rs']['values'].get(n)
            for n in HS_PARAMS if n not in PER_NODE}
    out['checks'] = dict(
        all_listed=all(out[t]['names'] is not None and set(HS_PARAMS) <= set(out[t]['names'])
                       for t in TAGS),
        all_read_back=all(len(out[t]['values']) == len(HS_PARAMS)
                          and None not in out[t]['values'].values() for t in TAGS),
        same_values=all(same.values()))
    out['differing'] = sorted(n for n, v in same.items() if not v)
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--out', default='output/rust/live_hard_stop.json')
    ap.add_argument('--log-dir', default=None)
    a = ap.parse_args()
    env = lc.isolated_env()
    log_dir = a.log_dir or os.path.join(os.path.dirname(os.path.abspath(a.out)), 'live_logs')
    os.makedirs(log_dir, exist_ok=True)
    procs = {'py': lc.start_node('py', 'py', env, log_dir),
             'rs': lc.start_node('rust', 'rs', env, log_dir)}
    d = lc.Driver(TAGS)
    report = dict(domain=env['ROS_DOMAIN_ID'])
    try:
        if not d.wait_ready(2):
            raise RuntimeError('nodes not discovered within 30 s')
        d.stream(lc.CLEAR, 2.0)       # discovery settles
        report['qos'] = qos_check(d)
        report['params'] = params_check(d)
        report['A_trigger'] = phase_trigger(d)
        report['B_dead_lidar'] = phase_dead(d)
        report['C_reverse_gate'] = phase_reverse(d)
        report['D_lane_timeout'] = phase_lane(d)
        report['E_cpu'] = phase_cpu(d, procs)
        report['alive_at_end'] = {t: procs[t].poll() is None for t in TAGS}
    finally:
        d.close()
        report['shutdown'] = lc.stop_nodes(procs)
    checks = {f'{k}.{c}': v for k, sec in report.items() if isinstance(sec, dict)
              for c, v in sec.get('checks', {}).items()}
    checks['alive_at_end'] = all(report['alive_at_end'].values())
    checks['clean_shutdown'] = all(v.startswith('SIGINT') for v in report['shutdown'].values())
    report['checks'] = checks
    report['pass'] = all(checks.values())
    os.makedirs(os.path.dirname(a.out) or '.', exist_ok=True)
    with open(a.out, 'w') as f:
        json.dump(report, f, indent=1)
    for k, v in checks.items():
        print(f'{"ok  " if v else "FAIL"} {k}')
    phases = ('A_trigger', 'B_dead_lidar', 'C_reverse_gate', 'D_lane_timeout', 'E_cpu')
    print(json.dumps({k: report[k] for k in phases}, indent=1))
    print('PASS' if report['pass'] else 'FAIL')
    return 0 if report['pass'] else 1


if __name__ == '__main__':
    sys.exit(main())
