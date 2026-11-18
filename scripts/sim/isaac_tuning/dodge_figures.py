#!/usr/bin/env python3
"""The mpc-dodge report's figures, from an evaluate.py A/B directory (and optionally a loop).

    dodge_figures.py --ab output/isaac_tuning/dodge/ab_phase2 [--out output/isaac_tuning/figures]

Writes (PNG, light surface, plots.py's palette):
  dodge_dfail_varianti.png      d_fail of the ramp and good M / S runs per variant (small multiples)
  dodge_<variant>_d<d>.png      top views of a variant's dodge runs at one level (+ its d_fail level)
  dodge_ritorno.png             lateral offset along S1-S3 for baseline / fixed line / return
  dodge_mappa_M_<a>_vs_<b>.png  M1-M4 trajectories, two variants overlaid
  screens_<run>.jpg             contact sheets of runs with spectator frames
"""
from __future__ import annotations

import argparse
import json
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import common  # noqa: E402
import plots  # noqa: E402


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.strip().splitlines()[0])
    ap.add_argument('--ab', required=True)
    ap.add_argument('--out', default=os.path.join(common.OUT_ROOT, 'figures'))
    ap.add_argument('--compare', default='baseline,alpha', help='two variants for the M map')
    ap.add_argument('--return-variants', default='baseline,fixed_line,return')
    args = ap.parse_args(argv)
    os.makedirs(args.out, exist_ok=True)
    with open(os.path.join(args.ab, 'ab.json'), encoding='utf-8') as fh:
        results = json.load(fh)
    made = [plots.fig_dfail(results, os.path.join(args.out, 'dodge_dfail_varianti.png'))]
    for r in results:
        name = r['variant']
        vdir = os.path.join(args.ab, name)
        ramp = r.get('ramp') or {}
        levels = sorted({lv['d'] for lv in ramp.get('levels', [])})
        for d in levels[-2:]:
            runs = plots.load_runs_glob(vdir, f'D{d:02d}_s*_r0')
            if runs:
                made.append(plots.fig_dodge_runs(
                    runs, os.path.join(args.out, f'dodge_{name}_d{d:02d}.png'),
                    f'{name}: dodge level d = {d} (d_fail {ramp.get("d_fail")})'))
    names = {r['variant'] for r in results}
    sets = [(n, plots.load_set(os.path.join(args.ab, n))) for n in args.return_variants.split(',')
            if n in names]
    if sets:
        made.append(plots.fig_return(sets, os.path.join(args.out, 'dodge_ritorno.png')))
    pair = [n for n in args.compare.split(',') if n in names]
    if len(pair) == 2:
        sets = [(n, plots.load_set(os.path.join(args.ab, n))) for n in pair]
        made.append(plots.fig_map(sets, os.path.join(args.out, f'dodge_mappa_M_{pair[0]}_vs_{pair[1]}.png'),
                                  scenarios=common.MOVING_IDS))
    for m in made:
        if m:
            print(m)
    return 0


if __name__ == '__main__':
    sys.exit(main())
