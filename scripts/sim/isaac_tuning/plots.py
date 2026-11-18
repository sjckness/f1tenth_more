#!/usr/bin/env python3
"""Explanatory figures of a tuning night: trajectories on the room map, progress, margins.

    plots.py --night output/isaac_tuning/night_<date> [--compare <set dir> ...] [--out DIR]

Figures (PNG, light surface):
  map_<label>.png     top view of the 6 x 6 room per scenario: true obstacle, its R_safe ring,
                      every repetition's ground-truth path, per set compared (first = before,
                      last = after)
  progress.png        set score per iteration, and good runs / runs per iteration (two panels,
                      one y-axis each)
  margins.png         car-centre clearance - R_safe per run, per scenario, before vs after
  screens_<run>.png   a contact sheet of a run's spectator frames, if it has any
"""
from __future__ import annotations

import argparse
import csv
import glob
import json
import math
import os
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.patches import Circle, Rectangle  # noqa: E402

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import common  # noqa: E402
from metrics import CLASS_MARGIN_M  # noqa: E402

# Reference palette (dataviz skill, light mode): surface, ink, categorical slots, status.
SURFACE, INK, INK2, GRID = '#fcfcfb', '#0b0b0b', '#52514e', '#e4e3df'
SERIES = ['#2a78d6', '#eb6834', '#1baf7a', '#eda100', '#e87ba4']
CRITICAL, OBST = '#d03b3b', '#9b9a95'
plt.rcParams.update({
    'figure.facecolor': SURFACE, 'axes.facecolor': SURFACE, 'savefig.facecolor': SURFACE,
    'axes.edgecolor': GRID, 'axes.labelcolor': INK2, 'xtick.color': INK2, 'ytick.color': INK2,
    'text.color': INK, 'axes.grid': True, 'grid.color': GRID, 'grid.linewidth': 0.8,
    'font.size': 10, 'axes.titlesize': 11, 'axes.titleweight': 'bold', 'legend.frameon': False,
    'axes.spines.top': False, 'axes.spines.right': False,
})
SCN_TITLE = {'M1': 'M1  D0.5 crossing from the right, 0.4 m/s',
             'M2': 'M2  D0.5 head-on, 0.25 m/s', 'M3': 'M3  D0.5 crossing from the left, 0.5 m/s',
             'M4': 'M4  D0.5 walks in and stops on the line',
             **{'S1': 'S1  D0.5 on the line, 1.5 m', 'S2': 'S2  D1.0 on the line, 1.5 m',
             'S3': 'S3  D0.5, +-0.15 m off, 1.5 m', 'S4': 'S4  D0.5 on the line, 1.0 m',
             'S5': 'S5  no obstacle',
             'P1': 'P1  persona attraversa da destra, 1 m/s',
             'P2': 'P2  persona verso l\'auto, 0.5 m/s',
             'P3': 'P3  persona entra e si ferma',
             'P4': 'P4  persona ferma sulla linea'}}


def load_set(set_dir: str) -> list[dict]:
    """Every run of a set directory: run.json + its ground-truth path."""
    runs = []
    for rj in sorted(glob.glob(os.path.join(set_dir, '[SMP]?*_r*', 'run.json'))):
        d = os.path.dirname(rj)
        with open(rj, encoding='utf-8') as fh:
            run = json.load(fh)
        with open(os.path.join(d, 'instance.json'), encoding='utf-8') as fh:
            run['instance'] = json.load(fh)
        xs, ys = [], []
        t0, t1 = None, None
        meta_p = os.path.join(d, 'meta.json')
        if os.path.exists(meta_p):
            with open(meta_p, encoding='utf-8') as fh:
                meta = json.load(fh)
            t0, t1 = meta.get('t_running'), meta.get('t_end')
        if os.path.exists(os.path.join(d, 'gt.csv')):
            with open(os.path.join(d, 'gt.csv'), newline='', encoding='utf-8') as fh:
                for row in csv.DictReader(fh):
                    t = float(row['t'])
                    if t0 is None or (t0 - 0.2 <= t <= (t1 or 1e9) + 1.0):
                        # stop at a physics blow-up after a hard contact (metrics.compute does too)
                        if row['x'] == 'nan' or abs(float(row.get('v') or 0.0)) > 20.0:
                            break
                        xs.append(float(row['x']))
                        ys.append(float(row['y']))
        run['path'] = (xs, ys)
        run['dir'] = d
        mv = os.path.join(d, 'movers.csv')
        run['movers'] = {}
        if os.path.exists(mv):
            with open(mv, newline='', encoding='utf-8') as fh:
                for row in csv.DictReader(fh):
                    run['movers'].setdefault(row['id'], []).append((float(row['x']), float(row['y'])))
        runs.append(run)
    return runs


def fig_map(sets: list[tuple[str, list]], out: str,
            scenarios=common.SCENARIO_IDS + common.MOVING_IDS, xlim=(0.0, 5.6),
            ylim=(1.2, 4.8)) -> str:
    """Top view per scenario, the sets overlaid (one color per set, fixed order)."""
    present = [s for s in scenarios if any(r['scenario'] == s for _, runs in sets for r in runs)]
    tall = (ylim[1] - ylim[0]) > 4.5
    fig, axes = plt.subplots(1, len(present), figsize=(4.0 * len(present), 5.4 if tall else 4.4),
                             squeeze=False)
    for ax, sid in zip(axes[0], present):
        ax.add_patch(Rectangle((0, 0), 6, 6, fill=False, edgecolor=INK2, linewidth=1.5))
        drawn = False
        for k, (label, runs) in enumerate(sets):
            color = SERIES[k % len(SERIES)]
            for r in (r for r in runs if r['scenario'] == sid):
                if not drawn:
                    for ob in r['instance'].get('obstacle_info', []):
                        rr = ob['radius_m']
                        track = r.get('movers', {}).get(ob['id'])
                        if ob.get('motion') and track:
                            # moving: its path, faint discs along it, the final disc solid
                            ax.plot([p[0] for p in track], [p[1] for p in track], ':', color=INK2,
                                    linewidth=1.5, zorder=2)
                            for p in track[::max(1, len(track) // 6)]:
                                ax.add_patch(Circle(p, rr, color=OBST, alpha=0.15, zorder=2))
                            ax.add_patch(Circle(track[-1], rr, color=OBST, alpha=0.55, zorder=2))
                            ax.annotate('', xy=track[-1], xytext=track[0],
                                        arrowprops=dict(arrowstyle='->', color=INK2, lw=1.2))
                            continue
                        ax.add_patch(Circle((ob['x'], ob['y']), rr, color=OBST, alpha=0.55, zorder=2))
                        ax.add_patch(Circle((ob['x'], ob['y']), rr + 0.32 + CLASS_MARGIN_M.get(
                            ob.get('class'), 0.0), fill=False, ls='--',
                                            edgecolor=OBST, linewidth=1.0, zorder=2))
                    drawn = True
                xs, ys = r['path']
                if not xs:
                    continue
                ax.plot(xs, ys, color=color, linewidth=2.0, alpha=0.85, zorder=3,
                        solid_capstyle='round')
                ax.plot(xs[0], ys[0], 'o', color=color, markersize=5, zorder=4)
                if r.get('outcome') in ('collision', 'estop'):
                    ax.plot(xs[-1], ys[-1], 'x', color=CRITICAL, markersize=10, mew=2.5, zorder=5)
        ax.set_xlim(*xlim)
        ax.set_ylim(*ylim)
        ax.set_aspect('equal')
        ax.set_title(SCN_TITLE.get(sid, sid), loc='left')
        ax.set_xlabel('x [m]  (car drives +x)')
    axes[0][0].set_ylabel('y [m]')
    handles = [plt.Line2D([0], [0], color=SERIES[k % len(SERIES)], lw=2.5, label=label)
               for k, (label, _) in enumerate(sets)]
    handles += [Circle((0, 0), 1, color=OBST, alpha=0.55, label='obstacle'),
                plt.Line2D([0], [0], color=OBST, ls='--', label='R_safe (r + 0.32 m, + 0.5 persona)'),
                plt.Line2D([0], [0], marker='x', color=CRITICAL, ls='', mew=2.5, label='collision / e-stop')]
    ncol = len(handles) if len(present) >= 4 else min(3, len(handles))
    rows = math.ceil(len(handles) / ncol)
    fig.legend(handles=handles, loc='upper center', ncol=ncol, bbox_to_anchor=(0.5, 1.0), fontsize=9)
    fig.tight_layout(rect=(0, 0, 1, 1.0 - 0.07 * rows - (0.0 if len(present) >= 4 else 0.04)))
    fig.savefig(out, dpi=130)
    plt.close(fig)
    return out


def fig_progress(history_rows: list[dict], out: str) -> str:
    rows = [r for r in history_rows if r.get('phase') == 'screen']
    if not rows:
        return ''
    it = list(range(1, len(rows) + 1))
    score = [float(r['set_score']) for r in rows]
    good = [sum(int(r.get(f'{s}_good') or 0) for s in common.SCENARIO_IDS) for r in rows]
    n = [sum(int(r.get(f'{s}_n') or 0) for s in common.SCENARIO_IDS) for r in rows]
    fig, (a1, a2) = plt.subplots(2, 1, figsize=(8, 5.6), sharex=True)
    a1.plot(it, score, '-o', color=SERIES[0], linewidth=2, markersize=7)
    best = max(range(len(score)), key=lambda i: score[i])
    a1.annotate(f'best {score[best]:.0f}', (it[best], score[best]), textcoords='offset points',
                xytext=(6, 8), color=INK)
    a1.set_title('Set score per iteration (mean - std per scenario, averaged)', loc='left')
    a1.set_ylim(0, 100)
    frac = [g / m if m else 0 for g, m in zip(good, n)]
    a2.bar(it, frac, color=SERIES[0], width=0.6)
    for i, (g, m) in enumerate(zip(good, n)):
        a2.text(it[i], frac[i] + 0.03, f'{g}/{m}', ha='center', color=INK2, fontsize=9)
    a2.set_ylim(0, 1.15)
    a2.set_title('Good runs per iteration (success, clearance >= R_safe - 3 cm)', loc='left')
    a2.set_xlabel('iteration')
    a2.set_xticks(it)
    fig.tight_layout()
    fig.savefig(out, dpi=130)
    plt.close(fig)
    return out


def fig_margins(sets: list[tuple[str, list]], out: str) -> str:
    scen = ['S1', 'S2', 'S3', 'S4']
    fig, ax = plt.subplots(figsize=(8, 4.2))
    ax.axhspan(-0.03, 0.0, color=GRID, alpha=0.8, zorder=0)
    ax.axhline(0.0, color=INK2, linewidth=1)
    w = 0.8 / max(1, len(sets))
    for k, (label, runs) in enumerate(sets):
        for i, sid in enumerate(scen):
            vals = [(r.get('min_margin_m'), r.get('outcome')) for r in runs
                    if r['scenario'] == sid and r.get('min_margin_m') is not None]
            for j, (m, o) in enumerate(vals):
                x = i + (k - (len(sets) - 1) / 2) * w + (j - (len(vals) - 1) / 2) * 0.04
                bad = o in ('collision', 'estop')
                ax.plot(x, m, 'x' if bad else 'o', color=CRITICAL if bad else SERIES[k % len(SERIES)],
                        markersize=8, mew=2, zorder=3)
    ax.set_xticks(range(len(scen)))
    ax.set_xticklabels([SCN_TITLE[s] for s in scen], fontsize=8)
    ax.set_ylabel('clearance - R_safe [m]')
    ax.set_title('Car-centre clearance to R_safe per run (above 0 = outside the safety disc)', loc='left')
    handles = [plt.Line2D([0], [0], marker='o', ls='', color=SERIES[k % len(SERIES)], label=label)
               for k, (label, _) in enumerate(sets)]
    handles.append(plt.Line2D([0], [0], marker='x', ls='', color=CRITICAL, mew=2, label='collision / e-stop'))
    ax.legend(handles=handles, loc='lower right')
    ax.text(-0.45, -0.015, '3 cm tolerance', color=INK2, fontsize=8, ha='left', va='center')
    fig.tight_layout()
    fig.savefig(out, dpi=130)
    plt.close(fig)
    return out


def contact_sheet(run_dir: str, out: str, n: int = 6) -> str:
    from PIL import Image, ImageDraw
    frames = sorted(glob.glob(os.path.join(run_dir, 'frames', '*.jpg')))
    if not frames:
        return ''
    pick = [frames[round(i * (len(frames) - 1) / (n - 1))] for i in range(n)]
    ims = [Image.open(p).convert('RGB').resize((640, 360)) for p in pick]
    sheet = Image.new('RGB', (640 * 3, 360 * ((n + 2) // 3)), SURFACE)
    for i, (im, p) in enumerate(zip(ims, pick)):
        d = ImageDraw.Draw(im)
        d.rectangle((0, 0, 150, 26), fill=(252, 252, 251))
        d.text((8, 6), f'sim t {float(os.path.basename(p)[1:-4]):.1f} s', fill=(11, 11, 11))
        sheet.paste(im, ((i % 3) * 640, (i // 3) * 360))
    sheet.save(out, quality=90)
    return out


# ---------------------------------------------------------------- mpc-dodge figures
def load_runs_glob(set_dir: str, pattern: str) -> list[dict]:
    """Every run under set_dir whose directory matches pattern (dodge runs: D<dd>_s<seed>_r<k>)."""
    runs = []
    for rj in sorted(glob.glob(os.path.join(set_dir, pattern, 'run.json'))):
        runs.extend(_load_named(set_dir, os.path.basename(os.path.dirname(rj))))
    return runs


def _load_named(set_dir: str, name: str) -> list[dict]:
    d = os.path.join(set_dir, name)
    with open(os.path.join(d, 'run.json'), encoding='utf-8') as fh:
        run = json.load(fh)
    with open(os.path.join(d, 'instance.json'), encoding='utf-8') as fh:
        run['instance'] = json.load(fh)
    with open(os.path.join(d, 'meta.json'), encoding='utf-8') as fh:
        meta = json.load(fh)
    t0, t1 = meta.get('t_running'), meta.get('t_end')
    xs, ys = [], []
    with open(os.path.join(d, 'gt.csv'), newline='', encoding='utf-8') as fh:
        for row in csv.DictReader(fh):
            t = float(row['t'])
            if t0 is None or (t0 - 0.2 <= t <= (t1 or 1e9) + 0.5):
                xs.append(float(row['x']))
                ys.append(float(row['y']))
    run['path'] = (xs, ys)
    run['dir'] = d
    run['movers'] = {}
    mv = os.path.join(d, 'movers.csv')
    if os.path.exists(mv):
        with open(mv, newline='', encoding='utf-8') as fh:
            for row in csv.DictReader(fh):
                tt = float(row['t'])
                if t0 is None or (t0 - 0.2 <= tt <= (t1 or 1e9) + 0.5):
                    run['movers'].setdefault(row['id'], []).append((float(row['x']), float(row['y'])))
    return [run]


def fig_dfail(results: list[dict], out: str) -> str:
    """Small multiples, one hue: d_fail of the dodge ramp, good M runs, good S runs per variant."""
    names = [r['variant'] for r in results][::-1]
    rs = results[::-1]
    panels = [('dodge ramp: d_fail (first level that fails)',
               [((r.get('ramp') or {}).get('d_fail')) for r in rs], None),
              ('M1-M4: good runs', [(r.get('M') or {}).get('good') for r in rs],
               [(r.get('M') or {}).get('n') for r in rs]),
              ('S1-S5: good runs', [(r.get('S') or {}).get('good') for r in rs],
               [(r.get('S') or {}).get('n') for r in rs])]
    fig, axes = plt.subplots(1, 3, figsize=(12, 0.45 * len(names) + 1.6), sharey=True)
    for ax, (title, vals, ns) in zip(axes, panels):
        y = range(len(names))
        v = [0 if x is None else x for x in vals]
        ax.barh(list(y), v, color=SERIES[0], height=0.62, zorder=2)
        for i, x in enumerate(vals):
            lab = '-' if x is None else (f'{x}/{ns[i]}' if ns and ns[i] is not None else str(x))
            ax.text((0 if x is None else x) + 0.05 * max(1, max(v) or 1), i, lab, va='center',
                    color=INK, fontsize=9)
        ax.set_title(title, loc='left')
        ax.grid(axis='y', visible=False)
        top = max(v) if v else 1
        ax.set_xlim(0, max(1, top) * 1.25 + (0.5 if ns is None else 0))
        from matplotlib.ticker import MaxNLocator
        ax.xaxis.set_major_locator(MaxNLocator(integer=True))
    axes[0].set_yticks(range(len(names)))
    axes[0].set_yticklabels(names)
    fig.tight_layout()
    fig.savefig(out, dpi=130)
    plt.close(fig)
    return out


def fig_dodge_runs(runs: list[dict], out: str, title: str = '') -> str:
    """Top views of dodge runs (room 8.5 x 6 m): the car's path, every cylinder's track."""
    n = len(runs)
    cols = min(3, n)
    rows_n = math.ceil(n / cols)
    fig, axes = plt.subplots(rows_n, cols, figsize=(5.2 * cols, 3.9 * rows_n), squeeze=False)
    for ax in axes.ravel()[n:]:
        ax.axis('off')
    for ax, r in zip(axes.ravel(), runs):
        ax.add_patch(Rectangle((0, 0), 8.5, 6, fill=False, edgecolor=INK2, linewidth=1.5))
        for ob in r['instance'].get('obstacle_info', []):
            tr = r['movers'].get(ob['id'])
            if not tr:
                continue
            ax.plot([p[0] for p in tr], [p[1] for p in tr], '-', color=OBST, linewidth=1.0,
                    alpha=0.6, zorder=2)
            ax.add_patch(Circle(tr[-1], ob['radius_m'], color=OBST, alpha=0.6, zorder=2))
        xs, ys = r['path']
        ax.plot(xs, ys, color=SERIES[0], linewidth=2.0, zorder=3)
        if r.get('outcome') in ('collision', 'estop'):
            ax.plot(xs[-1], ys[-1], 'x', color=CRITICAL, markersize=11, mew=2.5, zorder=5)
        f = r.get('failure') or {}
        sub = (f'{r["outcome"]}' + (f', level {f.get("level")}, margin {f["margin_m"]:+.2f} m'
                                    if f else ''))
        ax.set_title(f'{r["scenario"]}: {sub}', loc='left', fontsize=10)
        ax.set_xlim(0, 8.5)
        ax.set_ylim(0, 6)
        ax.set_aspect('equal')
    if title:
        fig.suptitle(title, x=0.01, ha='left', fontweight='bold')
    fig.tight_layout()
    fig.savefig(out, dpi=120)
    plt.close(fig)
    return out


def fig_return(sets: list[tuple[str, list]], out: str, scenarios=('S1', 'S2', 'S3')) -> str:
    """Lateral offset from the start line along the drive, per variant: the return after a swerve."""
    fig, axes = plt.subplots(1, len(scenarios), figsize=(4.6 * len(scenarios), 3.6), sharey=True,
                             squeeze=False)
    for ax, sid in zip(axes[0], scenarios):
        ax.axhspan(-0.10, 0.10, color=GRID, alpha=0.8, zorder=0)
        for k, (label, runs) in enumerate(sets):
            for r in (r for r in runs if r['scenario'] == sid):
                xs, ys = r['path']
                if not xs:
                    continue
                st = r['instance']['nominal_start']
                yaw = math.radians(r.get('start_pose_map_sim', {}).get('yaw_deg', st['yaw_deg']))
                x0 = r.get('start_pose_map_sim', {}).get('x', xs[0])
                y0 = r.get('start_pose_map_sim', {}).get('y', ys[0])
                c, s_ = math.cos(yaw), math.sin(yaw)
                a = [c * (x - x0) + s_ * (y - y0) for x, y in zip(xs, ys)]
                b = [-s_ * (x - x0) + c * (y - y0) for x, y in zip(xs, ys)]
                ax.plot(a, b, color=SERIES[k % len(SERIES)], linewidth=2.0, alpha=0.85)
        ax.set_title(SCN_TITLE.get(sid, sid), loc='left')
        ax.set_xlabel('along the start heading [m]')
    axes[0][0].set_ylabel('lateral offset [m]')
    handles = [plt.Line2D([0], [0], color=SERIES[k % len(SERIES)], lw=2.5, label=label)
               for k, (label, _) in enumerate(sets)]
    handles.append(Rectangle((0, 0), 1, 1, color=GRID, label='+-0.10 m'))
    fig.legend(handles=handles, loc='upper center', ncol=len(handles), bbox_to_anchor=(0.5, 1.0))
    fig.tight_layout(rect=(0, 0, 1, 0.9))
    fig.savefig(out, dpi=130)
    plt.close(fig)
    return out


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.strip().splitlines()[0])
    ap.add_argument('--night', default=None, help='night_<date> directory (iterations it*)')
    ap.add_argument('--compare', nargs='*', default=[], help='set dirs to overlay (label=dir)')
    ap.add_argument('--history', default=os.path.join(common.TUNING_DIR, 'history.csv'))
    ap.add_argument('--out', default=None)
    args = ap.parse_args(argv)
    sets = []
    for spec in args.compare:
        label, _, d = spec.partition('=')
        sets.append((label, load_set(d)))
    if args.night and not sets:
        its = sorted(d for d in glob.glob(os.path.join(args.night, 'it*')) if not d.endswith('_confirm'))
        if its:
            sets.append(('before (iteration 1)', load_set(its[0])))
            if len(its) > 1:
                best = max(its, key=lambda d: json.load(open(os.path.join(d, 'set.json')))['set_score']
                           if os.path.exists(os.path.join(d, 'set.json')) else -1)
                sets.append((f'after (best: {os.path.basename(best).split("_")[0]})', load_set(best)))
    out = args.out or os.path.join(args.night or common.OUT_ROOT, 'plots')
    os.makedirs(out, exist_ok=True)
    made = []
    if sets:
        made.append(fig_map(sets, os.path.join(out, 'map.png')))
        made.append(fig_margins(sets, os.path.join(out, 'margins.png')))
        for _, runs in sets:
            for r in runs:
                if os.path.isdir(os.path.join(r['dir'], 'frames')):
                    made.append(contact_sheet(r['dir'], os.path.join(
                        out, f'screens_{os.path.basename(os.path.dirname(r["dir"]))}_{r["scenario"]}_r{r["rep"]}.jpg')))
    if os.path.exists(args.history):
        with open(args.history, newline='', encoding='utf-8') as fh:
            hist = list(csv.DictReader(fh))
        if args.night:
            tag = os.path.relpath(args.night, common.REPO)
            hist = [r for r in hist if r.get('out', '').startswith(tag)]
        made.append(fig_progress(hist, os.path.join(out, 'progress.png')))
    for m in made:
        if m:
            print(m)
    return 0


if __name__ == '__main__':
    sys.exit(main())
