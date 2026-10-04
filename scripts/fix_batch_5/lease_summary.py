#!/usr/bin/env python3
"""Fix batch 5 (H1): summarise the lease fix's measurements.

  lease_summary.py isolation OUT_JSON BATCH_DIR...
      fix batch 4's isolation_summary.py per batch, plus per run the
      watchdog's alerts (alert mode: it reports, never restarts), and the
      statistics against fix batch 4's baseline (8 of 30): one-sided Fisher
      exact p, and the exact (Clopper-Pearson) 95 % upper bound of the
      batch's own rate.
  lease_summary.py discload OUT_JSON CONDITION_DIR...
      per condition: mean and standard deviation over runs of every
      discovery_load.py figure, idle (60 s, unfed) and fed (30 s), and the
      bringup cost (cumulative CPU of every participant's dds.* threads at
      the settled sample).
"""
import glob
import json
import os
import re
import statistics
import subprocess
import sys

from scipy.stats import beta, fisher_exact

HERE = os.path.dirname(os.path.abspath(__file__))
BASELINE = (8, 30)   # fix batch 4, production default (20 s lease)
ALERT_RE = re.compile(r"\[health\] '(\w+)' LIVENESS FAILURE[^:]*: (.*)")


def upper95(k, n):
    return 1.0 if k == n else float(beta.ppf(0.95, k + 1, n - k))


def isolation(out_json, batches):
    tmp = out_json + '.fb4.json'
    subprocess.run([sys.executable, os.path.join(HERE, '..', 'fix_batch_4', 'isolation_summary.py'),
                    tmp, *batches], check=True, stdout=subprocess.DEVNULL)
    res = json.load(open(tmp))
    os.remove(tmp)
    for b in batches:
        name = os.path.basename(b.rstrip('/'))
        v = res[name]
        alerts = {}
        for r in sorted(glob.glob(os.path.join(b, 'run_*'))):
            log = os.path.join(r, 'launch.log')
            hits = [m.groups() for m in map(ALERT_RE.search, open(log, errors='replace'))
                    if m] if os.path.exists(log) else []
            if hits:
                alerts[os.path.basename(r)] = hits
        k, n = v['runs_with_isolation'], v['runs']
        _, p = fisher_exact([[BASELINE[0], BASELINE[1] - BASELINE[0]], [k, n - k]],
                            alternative='greater')
        v['watchdog_alerts'] = alerts
        v['vs_fb4_baseline'] = {
            'baseline': f'{BASELINE[0]}/{BASELINE[1]}', 'this': f'{k}/{n}',
            'fisher_one_sided_p': float(f'{p:.2g}'),
            'rate': round(k / n, 4) if n else None,
            'rate_upper95': round(upper95(k, n), 4) if n else None,
            'baseline_rate': round(BASELINE[0] / BASELINE[1], 4),
        }
        isolated = {x['run']: x['isolated'] for x in v['rows'] if x.get('isolated')}
        # Did the watchdog see every isolation isolation_check.py saw?
        v['isolated_runs_alerted'] = {r: r in alerts for r in isolated}
        print(f"{name}: {k}/{n} isolated, p={p:.2g} vs {BASELINE[0]}/{BASELINE[1]}, "
              f"upper95 {upper95(k, n):.3f}; watchdog alerts in {len(alerts)} runs: "
              + ', '.join(f'{r}: {[c for c, _ in h]}' for r, h in alerts.items()))
    json.dump(res, open(out_json, 'w'), indent=2)


def _stats(values):
    values = [x for x in values if x is not None]
    if not values:
        return None
    return {'mean': round(statistics.mean(values), 4),
            'sd': round(statistics.stdev(values), 4) if len(values) > 1 else 0.0,
            'n': len(values)}


def _flat(d, prefix=''):
    out = {}
    for k, v in d.items():
        if isinstance(v, dict):
            out.update(_flat(v, f'{prefix}{k}.'))
        elif isinstance(v, (int, float)):
            out[prefix + k] = v
    return out


def discload(out_json, conditions):
    res = {}
    for c in conditions:
        name = os.path.basename(c.rstrip('/'))
        per = {'idle': [], 'fed': [], 'bringup': []}
        for r in sorted(glob.glob(os.path.join(c, 'run_*'))):
            for phase in ('idle', 'fed'):
                p = os.path.join(r, f'load_{phase}.json')
                if os.path.exists(p):
                    per[phase].append(_flat(json.load(open(p))))
            p = os.path.join(r, 'load_settled.json')
            if os.path.exists(p):
                s = json.load(open(p))
                dds = sum(v for pr in s['procs'].values() if not pr['ds']
                          for k, v in pr['cpu'].items() if k.startswith('dds'))
                # Not the Discovery Server: it outlives the run, so its
                # cumulative CPU is not one bringup's.
                per['bringup'].append({'dds_threads_cpu_s': dds})
        res[name] = {phase: {k: _stats([row.get(k) for row in rows])
                             for k in sorted({k for row in rows for k in row})}
                     for phase, rows in per.items()}
        res[name]['runs'] = len(per['idle'])
    json.dump(res, open(out_json, 'w'), indent=2)
    keys = ['cpu_s_per_s.dds.ev', 'cpu_s_per_s.dds.udp', 'cpu_s_per_s.dds.shm', 'ds_cpu_s_per_s',
            'supervisor_cpu_s_per_s', 'udp_per_s.InDatagrams', 'udp_per_s.OutDatagrams',
            'udp_per_s.RcvbufErrors', 'lo_per_s.rx_packets', 'lo_per_s.rx_bytes',
            'cpu_s_per_s.app', 'participants']
    for name, v in res.items():
        print(f'== {name} ({v["runs"]} runs)')
        for phase in ('idle', 'fed'):
            print('  ' + phase + ': ' + ', '.join(
                f'{k}={v[phase][k]["mean"]}±{v[phase][k]["sd"]}' for k in keys if v[phase].get(k)))
        print('  bringup: ' + ', '.join(f'{k}={s["mean"]}±{s["sd"]}'
                                         for k, s in v['bringup'].items() if s))


if __name__ == '__main__':
    {'isolation': isolation, 'discload': discload}[sys.argv[1]](sys.argv[2], sys.argv[3:])
