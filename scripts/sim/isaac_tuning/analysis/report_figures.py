#!/usr/bin/env python3
"""The figures of docs/mpc_dodge/REPORT.md, from the A/B directories of the night 2026-11-18.

    report_figures.py [--root output/isaac_tuning/dodge] [--out output/isaac_tuning/figures]
"""
import argparse
import collections
import glob
import json
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import common  # noqa: E402
import metrics  # noqa: E402
import plots  # noqa: E402
from plots import INK, INK2, SERIES, plt  # noqa: E402


def runs_of(dirs):
    out = []
    for d in dirs:
        for rj in sorted(glob.glob(os.path.join(d, '*_r*', 'run.json'))):
            with open(rj, encoding='utf-8') as fh:
                out.append(json.load(fh))
    return out


def fig_levels(variants, out):
    """Per variant: good dodge runs per level (6 seeds each) and good M runs (12)."""
    names = list(variants)[::-1]
    fig, (a1, a2) = plt.subplots(1, 2, figsize=(12, 0.62 * len(names) + 1.8), sharey=True,
                                 gridspec_kw={'width_ratios': [2.2, 1]})
    h = 0.26
    for k, d in enumerate((0, 1, 2)):
        vals = []
        for n in names:
            rs = [r for r in runs_of(variants[n]['D']) if r['scenario'].startswith(f'D{d:02d}')]
            vals.append((sum(metrics.good_enough(r) for r in rs), len(rs)))
        y = [i + (1 - k) * h for i in range(len(names))]
        a1.barh(y, [g for g, _ in vals], height=h - 0.03, color=SERIES[k], label=f'd = {d}', zorder=2)
        for yy, (g, n) in zip(y, vals):
            a1.text(g + 0.08, yy, f'{g}/{n}', va='center', fontsize=8, color=INK2)
    a1.set_xlim(0, 7)
    a1.set_title('Dodge: run buoni per livello (6 seed per livello)', loc='left')
    a1.set_xlabel('run buoni')
    a1.legend(loc='lower right', ncol=3)
    mvals = []
    for n in names:
        rs = [r for r in runs_of(variants[n]['M']) if r['scenario'].startswith('M')]
        mvals.append((sum(metrics.good_enough(r) for r in rs), len(rs)))
    a2.barh(range(len(names)), [g for g, _ in mvals], height=0.6, color=SERIES[0], zorder=2)
    for i, (g, n) in enumerate(mvals):
        a2.text(g + 0.15, i, f'{g}/{n}' if n else '–', va='center', fontsize=9, color=INK)
    a2.set_xlim(0, 13)
    from matplotlib.ticker import MaxNLocator
    a2.xaxis.set_major_locator(MaxNLocator(integer=True))
    a2.set_title('M1–M4: run buoni', loc='left')
    a2.set_xlabel('run buoni')
    a1.set_yticks(range(len(names)))
    a1.set_yticklabels(names)
    for a in (a1, a2):
        a.grid(axis='y', visible=False)
    fig.tight_layout()
    fig.savefig(out, dpi=130)
    plt.close(fig)
    return out


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.strip().splitlines()[0])
    ap.add_argument('--root', default=os.path.join(common.OUT_ROOT, 'dodge'))
    ap.add_argument('--out', default=os.path.join(common.OUT_ROOT, 'figures'))
    args = ap.parse_args(argv)
    R, C = args.root, os.path.join(args.root, 'ab_phase2c')
    os.makedirs(args.out, exist_ok=True)
    variants = collections.OrderedDict([
        ('baseline', {'D': [f'{C}/D_only/baseline'], 'M': [f'{R}/ab_phase2/baseline']}),
        ('α = 1', {'D': [f'{C}/D_M/alpha'], 'M': [f'{C}/D_M/alpha']}),
        ('α + scia', {'D': [f'{C}/D_M/alpha_wake'], 'M': [f'{C}/M_rest/alpha_wake']}),
        ('regola informata', {'D': [f'{C}/D_M_rest/informed'], 'M': [f'{C}/D_M_rest/informed']}),
        ('tutte (α + scia + informata)', {'D': [f'{C}/D_M_rest/all'], 'M': [f'{C}/D_M_rest/all']}),
        ('α + ritorno', {'D': [f'{C}/D_M_S/alpha_return'], 'M': [f'{C}/D_M_S/alpha_return']}),
        ('α + scia + sonde sul corpo', {'D': [f'{C}/body/alpha_wake_body'],
                                        'M': [f'{C}/body/alpha_wake_body']}),
    ])
    made = [fig_levels(variants, os.path.join(args.out, 'dodge_livelli_varianti.png'))]
    for name, d in (('baseline', f'{C}/D_only/baseline'), ('alpha_wake_body', f'{C}/body/alpha_wake_body')):
        runs = plots.load_runs_glob(d, 'D01_s*_r0')
        made.append(plots.fig_dodge_runs(runs, os.path.join(args.out, f'dodge_d01_{name}.png'),
                                         f'{name}: livello d = 1, seed 0–5'))
    sets = [('baseline', plots.load_set(f'{R}/ab_phase2/baseline')),
            ('linea fissa', plots.load_set(f'{R}/ab_phase2b/fixed_line')),
            ('α + ritorno (arco)', plots.load_set(f'{C}/D_M_S/alpha_return'))]
    made.append(plots.fig_return(sets, os.path.join(args.out, 'dodge_ritorno.png')))
    sets = [('baseline', plots.load_set(f'{R}/ab_phase2/baseline')),
            ('α + scia', plots.load_set(f'{C}/M_rest/alpha_wake'))]
    made.append(plots.fig_map(sets, os.path.join(args.out, 'dodge_mappa_M_baseline_vs_alpha_scia.png'),
                              scenarios=common.MOVING_IDS))
    for m in made:
        print(m)
    return 0


if __name__ == '__main__':
    sys.exit(main())
