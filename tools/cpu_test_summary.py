#!/usr/bin/env python3
"""summary.md for a cpu_test.sh run: one table per phase, and the two diffs.

    cpu_test_summary.py --out output/cpu_test/<timestamp>

Reads whatever each phase directory holds -- procstat.json or pidstat.txt,
tegrastats.log, rates.json, and the nodes' own timing JSON if a phase captured
any -- and writes summary.md beside them. Missing inputs are reported as
missing; nothing is inferred.
"""

import argparse
import collections
import json
import re
import sys
from pathlib import Path

import numpy as np

PHASE_TITLE = {
    'a': 'stationary, logger OFF', 'b': 'stationary, logger ON (recording)',
    'c': 'corridor10m mission, logger OFF', 'd': 'corridor10m mission, logger ON'}

TEGRA = re.compile(
    r'RAM (\d+)/(\d+)MB.*?CPU \[([^\]]*)\].*?GR3D_FREQ (\d+)%'
    r'(?:.*?tj@([\d.]+)C)?(?:.*?VDD_CPU_CV (\d+)mW)?(?:.*?VIN_SYS_5V0 (\d+)mW)?')


def read_tegrastats(path):
    if not path.exists():
        return None
    cores, gpu, tj, ram, cpu_mw, sys_mw, throttle = [], [], [], [], [], [], 0
    for line in path.read_text(errors='replace').splitlines():
        if 'throttl' in line.lower():
            throttle += 1
        m = TEGRA.search(line)
        if not m:
            continue
        ram.append(int(m.group(1)))
        per_core = []
        for token in m.group(3).split(','):
            token = token.strip()
            if token in ('off', ''):
                per_core.append(0.0)
                continue
            per_core.append(float(token.split('%')[0]))
        cores.append(per_core)
        gpu.append(float(m.group(4)))
        if m.group(5):
            tj.append(float(m.group(5)))
        if m.group(6):
            cpu_mw.append(float(m.group(6)))
        if m.group(7):
            sys_mw.append(float(m.group(7)))
    if not cores:
        return None
    width = max(len(c) for c in cores)
    grid = np.full((len(cores), width), np.nan)
    for i, c in enumerate(cores):
        grid[i, :len(c)] = c
    return {'samples': len(cores),
            'per_core_median': [round(float(np.nanmedian(grid[:, i])), 1)
                                for i in range(width)],
            'cpu_total_median': round(float(np.nanmedian(np.nansum(grid, axis=1))), 1),
            'gpu_median': round(float(np.median(gpu)), 1),
            'gpu_max': round(float(np.max(gpu)), 1),
            'tj_median': round(float(np.median(tj)), 1) if tj else None,
            'tj_max': round(float(np.max(tj)), 1) if tj else None,
            'ram_max_mb': int(np.max(ram)),
            'cpu_mw_median': round(float(np.median(cpu_mw)), 0) if cpu_mw else None,
            'sys_mw_median': round(float(np.median(sys_mw)), 0) if sys_mw else None,
            'throttle_lines': throttle}


def read_procstat(path):
    if not path.exists():
        return None
    data = json.loads(path.read_text())
    per_node = collections.defaultdict(list)
    io_node = collections.defaultdict(lambda: [0.0, 0.0])
    threads = collections.defaultdict(set)
    affinity = {}
    by_core = collections.defaultdict(float)
    for s in data['samples']:
        if 'cpu_pct' in s:
            per_node[s['node']].append((s['t'], s['cpu_pct']))
            threads[s['node']].add(s['tid'])
            affinity.setdefault(s['node'], s.get('affinity'))
            by_core[s.get('last_cpu')] += s['cpu_pct']
        else:
            io_node[s['node']][0] += s.get('read_kbs', 0.0)
            io_node[s['node']][1] += s.get('write_kbs', 0.0)
    # sum the per-thread percentages within a sample instant, then take the median
    nodes = {}
    for node, rows in per_node.items():
        per_t = collections.defaultdict(float)
        for t, pct in rows:
            per_t[t] += pct
        values = np.array(list(per_t.values()))
        nodes[node] = {'cpu_median': round(float(np.median(values)), 1),
                       'cpu_p95': round(float(np.percentile(values, 95)), 1),
                       'threads': len(threads[node]),
                       'affinity': affinity.get(node),
                       'read_kbs': round(io_node[node][0] / max(len(per_t), 1), 1),
                       'write_kbs': round(io_node[node][1] / max(len(per_t), 1), 1)}
    return {'source': data.get('source', '?'), 'nodes': nodes,
            'by_core': {k: round(v, 1) for k, v in sorted(by_core.items())
                        if k is not None}}


def read_rates(path):
    return json.loads(path.read_text()) if path.exists() else None


def table(rows, headers):
    out = ['| ' + ' | '.join(headers) + ' |',
           '|' + '|'.join(['---'] * len(headers)) + '|']
    for row in rows:
        out.append('| ' + ' | '.join('--' if v is None else str(v) for v in row) + ' |')
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', required=True)
    cfg = ap.parse_args()
    out = Path(cfg.out)
    phases = {}
    for letter in 'abcd':
        d = out / f'phase_{letter}'
        if not d.is_dir():
            continue
        phases[letter] = {
            'tegra': read_tegrastats(d / 'tegrastats.log'),
            'proc': read_procstat(d / 'procstat.json'),
            'rates': read_rates(d / 'rates.json'),
            'cpu_source': (d / 'cpu_source.txt').read_text().strip()
            if (d / 'cpu_source.txt').exists() else '?',
            'pidstat': (d / 'pidstat.txt').exists(),
        }

    L = ['# cpu_test summary', '']
    run = out / 'run.txt'
    if run.exists():
        L += ['```', run.read_text().strip(), '```', '']
    if not phases:
        L += ['**No phase directory held anything.**']
        (out / 'summary.md').write_text('\n'.join(L) + '\n')
        return 0

    for letter, p in phases.items():
        L += [f'## Phase {letter} — {PHASE_TITLE[letter]}', '']
        L += [f'CPU source: `{p["cpu_source"]}`', '']
        t = p['tegra']
        if t:
            L += ['### Board (tegrastats, 1 s)', '']
            L += table([[t['samples'], t['cpu_total_median'], t['gpu_median'],
                         t['gpu_max'], t['tj_median'], t['tj_max'],
                         t['ram_max_mb'], t['cpu_mw_median'], t['sys_mw_median'],
                         t['throttle_lines']]],
                       ['samples', 'CPU total % (median)', 'GPU % med', 'GPU % max',
                        'tj C med', 'tj C max', 'RAM MB max', 'CPU mW', 'SYS mW',
                        'throttle lines']) + ['']
            L += ['Per core, median %: `' + ', '.join(
                f'{i}:{v}' for i, v in enumerate(t['per_core_median'])) + '`', '']
        else:
            L += ['_no tegrastats output_', '']
        pr = p['proc']
        if pr:
            rows = sorted(pr['nodes'].items(), key=lambda kv: -kv[1]['cpu_median'])[:10]
            L += ['### CPU per node, top 10 (sum over its threads)', '']
            L += table([[name, v['cpu_median'], v['cpu_p95'], v['threads'],
                         (','.join(map(str, v['affinity'])) if v['affinity'] else '--'),
                         v['read_kbs'], v['write_kbs']] for name, v in rows],
                       ['node', 'CPU % med', 'CPU % p95', 'threads',
                        'affinity', 'read KB/s', 'write KB/s']) + ['']
            if pr['by_core']:
                L += ['CPU seen per core (sum of the thread samples that last ran '
                      'there): `' + ', '.join(f'{k}:{v}' for k, v in
                                              pr['by_core'].items()) + '`', '']
        elif p['pidstat']:
            L += ['_pidstat.txt written; not parsed here_', '']
        else:
            L += ['_no per-process CPU output_', '']
        r = p['rates']
        if r:
            L += ['### Rates, measured outside any bag', '']
            L += table([[topic, v['messages'], v['hz']]
                        for topic, v in r['rates'].items()],
                       ['topic', 'messages', 'Hz']) + ['']
            if r.get('absent_from_the_graph'):
                L += ['Absent from the graph: `'
                      + ', '.join(r['absent_from_the_graph']) + '`', '']
        else:
            L += ['_no rates output_', '']

    def diff(x, y, label):
        if x not in phases or y not in phases:
            return []
        out_lines = [f'## Diff {x} -> {y} ({label})', '']
        a, b = phases[x], phases[y]
        rows = []
        if a['tegra'] and b['tegra']:
            for key, name in (('cpu_total_median', 'CPU total % (median)'),
                              ('gpu_median', 'GPU % median'),
                              ('tj_median', 'tj C median'),
                              ('ram_max_mb', 'RAM MB max')):
                va, vb = a['tegra'][key], b['tegra'][key]
                if va is None or vb is None:
                    continue
                rows.append([name, va, vb, round(vb - va, 1)])
        if a['proc'] and b['proc']:
            names = set(a['proc']['nodes']) | set(b['proc']['nodes'])
            for name in sorted(names):
                va = a['proc']['nodes'].get(name, {}).get('cpu_median', 0.0)
                vb = b['proc']['nodes'].get(name, {}).get('cpu_median', 0.0)
                if abs(vb - va) >= 1.0:
                    rows.append([f'CPU % {name}', va, vb, round(vb - va, 1)])
                wa = a['proc']['nodes'].get(name, {}).get('write_kbs', 0.0)
                wb = b['proc']['nodes'].get(name, {}).get('write_kbs', 0.0)
                if abs(wb - wa) >= 50.0:
                    rows.append([f'write KB/s {name}', wa, wb, round(wb - wa, 1)])
        if not rows:
            return out_lines + ['_nothing comparable in both phases_', '']
        return out_lines + table(rows, ['quantity', x, y, 'delta']) + ['']

    L += diff('a', 'b', 'what recording costs at rest')
    L += diff('c', 'd', 'what recording costs under load')
    (out / 'summary.md').write_text('\n'.join(L) + '\n')
    print(f'wrote {out}/summary.md')
    return 0


if __name__ == '__main__':
    sys.exit(main())
