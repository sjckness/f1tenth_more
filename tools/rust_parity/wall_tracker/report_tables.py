#!/usr/bin/env python3
"""Markdown tables from the R6 JSON outputs (output/rust/wall_tracker/).

    python3 tools/rust_parity/wall_tracker/report_tables.py [timing|stages|parity|all]

timing   tracker_ms py against rust (timing_{py,rust}_{car,every,full}.json)
stages   R6.0's per-stage exclusive times (stage_profile_py_car.json), the two
         largest bags of each kind
parity   the bag parity runs (parity_{car,every,full,doors}.json)
"""

import json
import sys
from pathlib import Path

D = Path(__file__).resolve().parents[3] / 'output' / 'rust' / 'wall_tracker'
NAMES = {'corridor10m_20261015T171753': 'corridor10m_20261015T171753 (c1015)',
         'corridor10m_20261016T144537': 'corridor10m_20261016T144537 (c1016)',
         'corridor10m_20261019T131815': 'corridor10m_20261019T131815 (c1019)',
         'corridor_obstacles_20261019T135307': 'corridor_obstacles_20261019T135307 (obst1019)'}


def short(path):
    b = Path(path).name
    return NAMES.get(b, b)


def load(name):
    p = D / name
    return json.loads(p.read_text()) if p.is_file() else None


def timing():
    py, rs = {}, {}
    for tag, store in (('py', py), ('rust', rs)):
        for kind in ('car', 'every', 'full'):
            doc = load(f'timing_{tag}_{kind}.json')
            for r in (doc or {}).get('results', []):
                store[(short(r['bag']), r['cycles_mode'], r['stereo'])] = r
    out = ['| bag | cycles | camera | steps | py p50 | py p95 | rust p50 | rust p95 | p95 py/rust |',
           '|---|---|---|---:|---:|---:|---:|---:|---:|']
    for k, a in py.items():
        b = rs.get(k)
        if b is None:
            continue
        ta, tb = a['tracker_ms'], b['tracker_ms']
        out.append(f"| {k[0]} | {k[1]} | {k[2]} | {ta['n']} | {ta['p50']:.2f} | {ta['p95']:.2f} "
                   f"| {tb['p50']:.2f} | {tb['p95']:.2f} | {ta['p95'] / tb['p95']:.1f}x |")
    return '\n'.join(out)


def stages(bags=('c1016', 'c1019')):
    doc = load('stage_profile_py_car.json')
    out = []
    for r in doc['results']:
        name = short(r['bag'])
        if not any(f'({b})' in name for b in bags):
            continue
        t = r['tracker_ms']
        out += ['', f"**{name}** -- tracker_ms p50 {t['p50']:.2f} / p95 {t['p95']:.2f} ms over "
                    f"{t['n']} steps (instrumented p50 {r['tracker_ms_instrumented']['p50']:.2f})",
                '', '| stage (exclusive ms) | p50 | p95 | mean | calls / cycle |',
                '|---|---:|---:|---:|---:|']
        st = sorted(r['stages_exclusive_ms'].items(), key=lambda kv: -kv[1].get('mean', 0))
        for k, s in st[:16]:
            out.append(f"| `{k}` | {s['p50']:.3f} | {s['p95']:.3f} | {s['mean']:.3f} "
                       f"| {s['calls_per_cycle']} |")
    return '\n'.join(out)


def parity():
    out = ['| bag | cycles | camera | doors | tracker steps | track states | messages '
           '| cycles differing | max abs diff | front steps | rect cycles |',
           '|---|---|---|---|---:|---:|---:|---:|---:|---:|---:|']
    for kind in ('car', 'every', 'full', 'doors'):
        doc = load(f'parity_{kind}.json')
        for r in (doc or {}).get('runs', []):
            out.append(f"| {short(r['bag'])} | {r['cycles_mode']} | {r['stereo']} "
                       f"| {'on' if r['doors'] else 'off'} | {r['stepped']} "
                       f"| {r['tracks_compared']} | {r['messages_compared']} "
                       f"| {r['cycles_with_diff']} | {r['max_abs_diff']} | {r['front_steps']} "
                       f"| {r['rect_cycles']} |")
    return '\n'.join(out)


def main():
    what = sys.argv[1] if len(sys.argv) > 1 else 'all'
    if what in ('timing', 'all'):
        print(timing())
    if what in ('stages', 'all'):
        print(stages())
    if what in ('parity', 'all'):
        print(parity())


if __name__ == '__main__':
    main()
