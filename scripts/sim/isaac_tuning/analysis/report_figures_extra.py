#!/usr/bin/env python3
"""More figures for docs/mpc_dodge/REPORT.md: the velocity estimate, failure modes, the persons.

    report_figures_extra.py [--root output/isaac_tuning/dodge] [--out output/isaac_tuning/figures]

  dodge_velocita_stimata.png   estimated vs true speed of the M1 cylinder (alpha + wake run)
  dodge_modi_di_fallimento.png where the cylinder is at the closest approach, baseline vs best
  persone_P1_P4.png            top views of the person scenarios, car setting vs unknown active
  screen_P1_persona.jpg        spectator frames of a P1 run
"""
import argparse
import bisect
import collections
import csv
import glob
import json
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import common  # noqa: E402
import failure_modes  # noqa: E402
import plots  # noqa: E402
from plots import CRITICAL, GRID, INK, INK2, SERIES, plt  # noqa: E402


def fig_velocity(run_dir, out):
    """Speed of the moving cylinder: ground truth vs the semantic layer's estimate per message."""
    t0 = json.load(open(os.path.join(run_dir, 'meta.json')))['t_running']
    gt = [(float(x['t']), float(x['x']), float(x['y']), float(x['yaw']))
          for x in csv.DictReader(open(os.path.join(run_dir, 'gt.csv'))) if x['x'] != 'nan']
    gts = [g[0] for g in gt]
    tr = sorted((float(x['t']), float(x['x']), float(x['y']))
                for x in csv.DictReader(open(os.path.join(run_dir, 'movers.csv'))))
    ts = [p[0] for p in tr]
    true_t, true_v = [], []
    for i in range(3, len(tr) - 3):
        dt = tr[i + 3][0] - tr[i - 3][0]
        if dt > 0:
            true_t.append(tr[i][0] - t0)
            true_v.append(math.hypot(tr[i + 3][1] - tr[i - 3][1], tr[i + 3][2] - tr[i - 3][2]) / dt)
    est = {True: ([], []), False: ([], [])}
    for x in csv.DictReader(open(os.path.join(run_dir, 'obstacles.csv'))):
        if not x['x'] or x.get('id') in (None, '', '-1'):
            continue
        t = float(x['t'])
        _, cx, cy, cyaw = gt[min(len(gt) - 1, bisect.bisect_left(gts, t))]
        ox, oy = float(x['x']), float(x['y'])
        gx = cx + math.cos(cyaw) * ox - math.sin(cyaw) * oy
        gy = cy + math.sin(cyaw) * ox + math.cos(cyaw) * oy
        j = max(0, min(bisect.bisect_right(ts, t), len(tr)) - 1)
        if math.hypot(tr[j][1] - gx, tr[j][2] - gy) > 0.45:
            continue
        valid = x['v_valid'] == '1'
        est[valid][0].append(t - t0)
        est[valid][1].append(math.hypot(float(x['vx']), float(x['vy'])))
    fig, ax = plt.subplots(figsize=(9, 3.8))
    ax.plot(true_t, true_v, color=INK, linewidth=2.0, label='velocità vera (ground truth)', zorder=3)
    ax.plot(est[True][0], est[True][1], 'o', color=SERIES[0], markersize=4, alpha=0.8,
            label='stima del semantic layer, v_valid', zorder=4)
    ax.plot(est[False][0], est[False][1], 'o', color=SERIES[1], markersize=4, alpha=0.8,
            label='stima, non valida (traccia giovane o ferma)', zorder=4)
    ax.set_xlabel('tempo dalla partenza [s]')
    ax.set_ylabel('velocità del cilindro [m/s]')
    ax.set_title('M1: il cilindro attraversa a 0.4 m/s per 6 s, poi si ferma', loc='left')
    ax.set_ylim(-0.05, 0.8)
    ax.legend(loc='upper right')
    fig.tight_layout()
    fig.savefig(out, dpi=130)
    plt.close(fig)
    return out


def fig_failures(groups, out):
    """Counts of the failure modes (failure_modes.analyse) per variant, one hue per variant."""
    cats = [('front', True), ('side', True), ('rear', True), ('front', False), ('side', False),
            ('rear', False)]
    labels = {('front', True): 'davanti, collisione / e-stop', ('side', True): 'fianco, collisione / e-stop',
              ('rear', True): 'coda, collisione', ('front', False): 'davanti, sfioro',
              ('side', False): 'fianco, sfioro', ('rear', False): 'coda, sfioro (auto in corsa)'}
    data = []
    for name, dirs in groups:
        rows = [a for d in dirs for a in (failure_modes.analyse(p) for p in
                                          sorted(glob.glob(os.path.join(d, 'D*_r*')))
                                          if os.path.exists(os.path.join(p, 'run.json'))) if a]
        c = collections.Counter((x['where'], x['outcome'] in ('collision', 'estop')) for x in rows)
        data.append((name, [c.get(k, 0) for k in cats], len(rows)))
    fig, ax = plt.subplots(figsize=(9, 4.2))
    h = 0.8 / len(data)
    for k, (name, vals, n) in enumerate(data):
        y = [i + k * h - 0.4 + h / 2 for i in range(len(cats))]
        ax.barh(y, vals, height=h - 0.04, color=SERIES[k], label=f'{name} ({n} run non buoni su 18)',
                zorder=2)
        for yy, v in zip(y, vals):
            if v:
                ax.text(v + 0.08, yy, str(v), va='center', fontsize=9, color=INK2)
    ax.set_yticks(range(len(cats)))
    ax.set_yticklabels([labels[k] for k in cats])
    ax.invert_yaxis()
    ax.set_xlabel('run')
    ax.set_title('Dodge (livelli 0–2, 6 seed): dove sta il cilindro nel punto più vicino', loc='left')
    from matplotlib.ticker import MaxNLocator
    ax.xaxis.set_major_locator(MaxNLocator(integer=True))
    ax.grid(axis='y', visible=False)
    ax.legend(loc='center right')
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
    made = [fig_velocity(f'{C}/M_rest/alpha_wake/M1_r0',
                         os.path.join(args.out, 'dodge_velocita_stimata.png')),
            fig_failures([('MPC di oggi', [f'{C}/D_only/baseline']),
                          ('α + scia + sonde sul corpo', [f'{C}/body/alpha_wake_body'])],
                         os.path.join(args.out, 'dodge_modi_di_fallimento.png'))]
    p_car = [r for r in plots.load_set(f'{R}/phase4_people') if not r['scenario'].endswith('u')]
    p_unk = [r for r in plots.load_set(f'{R}/phase4_people') if r['scenario'].endswith('u')]
    for r in p_unk:
        r['scenario'] = r['scenario'][:-1]
    made.append(plots.fig_map([('auto (unknown off)', p_car), ('unknown active', p_unk)],
                              os.path.join(args.out, 'persone_P1_P4.png'),
                              scenarios=common.PEOPLE_IDS, xlim=(0.0, 6.0), ylim=(0.0, 6.0)))
    made.append(plots.contact_sheet(f'{R}/phase4_people/P1_r1',
                                    os.path.join(args.out, 'screen_P1_persona.jpg')))
    for m in made:
        print(m)
    return 0


if __name__ == '__main__':
    sys.exit(main())
