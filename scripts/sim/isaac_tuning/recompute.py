#!/usr/bin/env python3
"""Recompute run.json (metrics.compute) for every run under a directory, and an A/B's summaries.

    recompute.py output/isaac_tuning/dodge/ab_phase2 [--ab]

Keeps the runner's own fields (recorder_code, frames, wall_recorder_s, isaac_alive_after). With
--ab the directory is an evaluate.py output: each variant's summary.json families (D, M, S) and
ab.json / ab.md are rebuilt from the recomputed runs; the ramp's d_fail is recomputed from them.
"""
from __future__ import annotations

import argparse
import glob
import json
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import evaluate  # noqa: E402
import metrics  # noqa: E402

KEEP = ('recorder_code', 'frames', 'wall_recorder_s', 'isaac_alive_after', 'teardown_leftovers')


def recompute_runs(root: str) -> int:
    n = 0
    for rj in glob.glob(os.path.join(root, '**', 'run.json'), recursive=True):
        d = os.path.dirname(rj)
        if not os.path.exists(os.path.join(d, 'meta.json')):
            continue
        with open(rj, encoding='utf-8') as fh:
            old = json.load(fh)
        new = metrics.compute(d)
        for k in KEEP:
            if k in old:
                new[k] = old[k]
        with open(rj, 'w', encoding='utf-8') as fh:
            json.dump(new, fh, indent=1, default=str)
        n += 1
    return n


def rebuild_ab(ab: str) -> None:
    results = []
    with open(os.path.join(ab, 'ab.json'), encoding='utf-8') as fh:
        order = [r['variant'] for r in json.load(fh)]
    for name in order:
        p = os.path.join(ab, name, 'summary.json')
        with open(p, encoding='utf-8') as fh:
            res = json.load(fh)
        vdir = os.path.join(ab, name)
        for fam in ('D', 'M', 'S'):
            if fam in res:
                res[fam] = evaluate.family_summary(evaluate._load_runs(vdir, fam))
        if 'ramp' in res:
            levels = {}
            for r in evaluate._load_runs(vdir, 'D'):
                d = int(r['scenario'][1:3])
                levels.setdefault(d, []).append(metrics.good_enough(r))
            lv_sorted = sorted(levels)
            res['ramp']['levels_recomputed'] = {d: f'{sum(v)}/{len(v)}' for d, v in levels.items()}
            res['ramp']['d_fail'] = next((d for d in lv_sorted if not all(levels[d])), None)
        with open(p, 'w', encoding='utf-8') as fh:
            json.dump(res, fh, indent=1, default=str)
        results.append(res)
    evaluate.write_table(results, ab)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.strip().splitlines()[0])
    ap.add_argument('root')
    ap.add_argument('--ab', action='store_true')
    args = ap.parse_args(argv)
    print(f'{recompute_runs(args.root)} runs recomputed')
    if args.ab:
        rebuild_ab(args.root)
        print(open(os.path.join(args.root, 'ab.md'), encoding='utf-8').read())
    return 0


if __name__ == '__main__':
    sys.exit(main())
