#!/usr/bin/env python3
"""Markdown table of evaluate.py variants: dodge per level, M / S good runs, the return.

    ab_table.py name=<variant dir> [name=<variant dir> ...]

A variant may be split over several A/B directories: give the same name twice
(alpha_wake=<D runs dir> alpha_wake=<M runs dir>) and the runs are pooled.
d_fail = the first level whose runs are not all good (metrics.good_enough), over every seed run.
"""
import collections
import glob
import json
import os
import statistics
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import metrics  # noqa: E402

HEAD = ('| variante | d_fail | dodge per livello (buoni/run) | M buoni | S buoni | '
        'S1–S3 scarto lat. max medio [m] | S1–S3 rientrati < 0.10 m | S1–S3 scarto finale medio [m] | '
        'overshoot max [m] |\n|---|---|---|---|---|---|---|---|---|')


def runs_of(vdir):
    out = []
    for rj in sorted(glob.glob(os.path.join(vdir, '*_r*', 'run.json'))):
        with open(rj, encoding='utf-8') as fh:
            out.append(json.load(fh))
    return out


def fmt(v):
    return '–' if v is None else (f'{v:.2f}' if isinstance(v, float) else str(v))


def row(name, runs):
    levels = {}
    per = collections.defaultdict(list)
    for r in runs:
        sid = r['scenario']
        if sid.startswith('D'):
            levels.setdefault(int(sid[1:3]), []).append(metrics.good_enough(r))
        elif sid[0] in 'MS':
            per[sid].append(r)
    d_fail = next((d for d in sorted(levels) if not all(levels[d])), None) if levels else None
    dtxt = ' '.join(f'd{d}: {sum(v)}/{len(v)}' for d, v in sorted(levels.items())) or '–'
    dsum = (f'**{sum(sum(v) for v in levels.values())}/{sum(len(v) for v in levels.values())}**'
            if levels else '–')

    def good(sids):
        rs = [r for s in sids for r in per.get(s, [])]
        if not rs:
            return '–'
        return f'{sum(metrics.good_enough(r) for r in rs)}/{len(rs)}'
    mtxt = good(['M1', 'M2', 'M3', 'M4'])
    if mtxt != '–':
        mtxt += ' (' + ' '.join(f'{s} {good([s])}' for s in ('M1', 'M2', 'M3', 'M4')) + ')'
    stxt = good(['S1', 'S2', 'S3', 'S4', 'S5'])
    if stxt != '–':
        stxt += ' (' + ' '.join(f'{s} {good([s])}' for s in ('S1', 'S2', 'S3', 'S4', 'S5')) + ')'
    s_runs = [r for s in ('S1', 'S2', 'S3') for r in per.get(s, []) if r.get('valid')]
    lat = statistics.fmean([r['max_lateral_error_m'] for r in s_runs]) if s_runs else None
    rets = [r.get('return') or {} for r in s_runs]
    passed = [x for x in rets if x.get('passed')]
    back = [x for x in passed if x.get('return_time_s') is not None]
    fin = statistics.fmean([abs(x['final_lateral_m']) for x in passed]) if passed else None
    ov = max((x.get('overshoot_m', 0.0) for x in passed), default=None)
    return (f'| {name} | **{fmt(d_fail)}** | {dtxt} = {dsum} | {mtxt} | {stxt} | {fmt(lat)} | '
            f'{len(back)}/{len(passed)} | {fmt(fin)} | {fmt(ov)} |')


def main(argv):
    pooled = collections.OrderedDict()
    for spec in argv:
        name, _, d = spec.partition('=')
        pooled.setdefault(name, []).extend(runs_of(d))
    print(HEAD)
    for name, runs in pooled.items():
        print(row(name, runs))
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))
