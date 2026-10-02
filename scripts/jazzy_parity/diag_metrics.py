#!/usr/bin/env python3
"""Phase 4: ekf_cost_observer_node outputs -- recorded live Humble vs Jazzy
replays -- and the platform-dependent field list of /diagnostics/system_status.

ekf_cost_observer publishes one DiagnosticStatus per EKF probe per 1 s
wall-clock window. Replay windows are not aligned with the recorded ones, so
each field is compared as a distribution over the run (median, p10, p90),
excluding the first and last 2 windows (start-up / tail). The Jazzy-vs-Jazzy
spread of those medians is the noise floor.

Field classes, from the node's own code:
  topic-derived  ticks_inproc (robot_localization's own FrequencyStatus
                 "Events in window", replayed from the bag's /diagnostics),
                 ticks_selfcount, tick_count_agreement, tick_rate_hz,
                 period_ms_* (output header stamps), meas_delivered,
                 meas_per_tick (input topic counts)
  process-derived pid, cpu_ms_per_tick, cpu_ms_per_meas, cpu_percent_of_core
                 (/proc of the live EKF process: platform- and load-dependent,
                 and absent in a replay with no EKF process -- not compared)

Usage: diag_metrics.py --recorded SRC_BAG --runs LABEL=BAG ... --out FILE.json
"""
import argparse
import json

import numpy as np

from bag_read import read_topic

TOPIC_FIELDS = ('ticks_inproc', 'ticks_selfcount', 'tick_count_agreement', 'tick_rate_hz',
                'period_ms_p50', 'period_ms_p90', 'period_ms_max', 'meas_delivered',
                'meas_per_tick')
PROCESS_FIELDS = ('pid', 'cpu_ms_per_tick', 'cpu_ms_per_meas', 'cpu_percent_of_core')


def load_observer(bag):
    out = {}
    for _, msg in read_topic(bag, '/diagnostics'):
        for st in msg.status:
            if not st.name.startswith('ekf_cost_observer'):
                continue
            probe = 'local' if 'local' in st.name else 'global'
            vals = {}
            for kv in st.values:
                try:
                    vals[kv.key] = float(kv.value)
                except ValueError:
                    pass
            vals['_level'] = int.from_bytes(st.level, 'little') if isinstance(st.level, bytes) else int(st.level)
            vals['_message'] = st.message
            out.setdefault(probe, []).append(vals)
    return out


def summary(windows):
    w = windows[2:-2] if len(windows) > 6 else windows
    res = {'n_windows': len(w)}
    for f in TOPIC_FIELDS + PROCESS_FIELDS:
        v = np.array([x[f] for x in w if f in x])
        if v.size:
            res[f] = {'median': float(np.median(v)), 'p10': float(np.percentile(v, 10)),
                      'p90': float(np.percentile(v, 90))}
    res['levels'] = sorted({x['_level'] for x in w})
    res['message_examples'] = sorted({x['_message'].split(':')[0][:60] for x in w})[:3]
    return res


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--recorded', required=True)
    ap.add_argument('--runs', nargs='+', required=True)
    ap.add_argument('--out', required=True)
    args = ap.parse_args()
    res = {'recorded': {}, 'runs': {}}
    rec = load_observer(args.recorded)
    for probe, w in rec.items():
        res['recorded'][probe] = summary(w)
    for spec in args.runs:
        label, bag = spec.split('=', 1)
        res['runs'][label] = {p: summary(w) for p, w in load_observer(bag).items()}
    labels = list(res['runs'])
    table = {}
    for probe in ('local', 'global'):
        for f in TOPIC_FIELDS:
            meds = [res['runs'][lb][probe][f]['median'] for lb in labels
                    if probe in res['runs'][lb] and f in res['runs'][lb][probe]]
            recm = res['recorded'].get(probe, {}).get(f, {}).get('median')
            if not meds:
                continue
            table[f'{probe}.{f}'] = {
                'jazzy_medians': meds, 'floor_spread': float(max(meds) - min(meds)),
                'recorded_median': recm,
                'recorded_minus_jazzy_mean': (recm - float(np.mean(meds))) if recm is not None else None}
    res['comparison'] = table

    # /diagnostics/system_status (system_observer_node): field inventory of the
    # recorded Orin data, for the platform-dependence table.
    ss = [m for _, m in read_topic(args.recorded, '/diagnostics/system_status')]
    if ss:
        res['recorded_system_status'] = {
            'n': len(ss), 'n_cores': len(ss[0].cpu_per_core),
            'cpu_percent': [float(min(m.cpu_percent for m in ss)), float(max(m.cpu_percent for m in ss))],
            'ram_total_mb': float(ss[0].ram_total_mb),
            'cpu_temp_c': [float(min(m.cpu_temp_c for m in ss)), float(max(m.cpu_temp_c for m in ss))],
            'gpu_percent': [float(min(m.gpu_percent for m in ss)), float(max(m.gpu_percent for m in ss))],
            'gpu_temp_c': [float(min(m.gpu_temp_c for m in ss)), float(max(m.gpu_temp_c for m in ss))],
            'emc_percent': [float(min(m.emc_percent for m in ss)), float(max(m.emc_percent for m in ss))]}
    with open(args.out, 'w') as f:
        json.dump(res, f, indent=2)
    for k, v in table.items():
        print('%-28s jazzy %-40s floor %-9.4g recorded %-10s' % (
            k, [round(x, 3) for x in v['jazzy_medians']], v['floor_spread'],
            None if v['recorded_median'] is None else round(v['recorded_median'], 3)))
    for probe in ('local', 'global'):
        print(probe, 'recorded levels', res['recorded'].get(probe, {}).get('levels'),
              res['recorded'].get(probe, {}).get('message_examples'),
              '| replay', res['runs'][labels[0]].get(probe, {}).get('levels'),
              res['runs'][labels[0]].get(probe, {}).get('message_examples'))
    print('system_status', res.get('recorded_system_status'))
    print('wrote', args.out)


if __name__ == '__main__':
    main()
