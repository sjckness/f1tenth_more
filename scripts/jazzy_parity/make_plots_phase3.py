#!/usr/bin/env python3
"""Phase 3 plots.

  mpc_commands_overlay.png   steering + speed of every Jazzy replay run (and a
                             Humble replay when given), with the recorded
                             live Humble commands dashed for the sanity check
  mpc_floor_diff.png         steering/speed difference between Jazzy runs (ZOH
                             50 Hz), i.e. the noise floor over time
  mpc_solve_time.png         solve-time distributions: recorded Humble/Orin
                             (live, loaded, /mpc/solver_status solve_dt) and
                             each Thor capture.npz given with --timing
                             (perf_counter around the same call; a replay's own
                             solve_dt is 0 under sim time)

Usage:
  make_plots_phase3.py --out DIR --recorded SRC_BAG --jazzy B1 B2 B3
      [--humble HB ...] [--timing LABEL=CAPTURE_NPZ ...]
"""
import argparse
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

import mpc_frozen_io  # noqa: E402
import mpc_metrics as mm  # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', required=True)
    ap.add_argument('--recorded', required=True)
    ap.add_argument('--jazzy', nargs='+', required=True)
    ap.add_argument('--humble', nargs='*', default=[])
    ap.add_argument('--timing', nargs='*', default=[])
    args = ap.parse_args()
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)

    rec = mm.load_mpc_run(args.recorded)
    jz = [mm.load_mpc_run(b) for b in args.jazzy]
    hb = [mm.load_mpc_run(b) for b in args.humble]
    t0 = rec['drive'][0, 0]

    fig, ax = plt.subplots(2, 1, figsize=(12, 7), sharex=True)
    for i, r in enumerate(jz):
        ax[0].plot(r['drive'][:, 0] - t0, r['drive'][:, 1], lw=1, label=f'Jazzy replay {i+1}')
        ax[1].plot(r['drive'][:, 0] - t0, r['drive'][:, 2], lw=1, label=f'Jazzy replay {i+1}')
    for i, r in enumerate(hb):
        ax[0].plot(r['drive'][:, 0] - t0, r['drive'][:, 1], lw=1, ls=':', label=f'Humble replay {i+1}')
        ax[1].plot(r['drive'][:, 0] - t0, r['drive'][:, 2], lw=1, ls=':', label=f'Humble replay {i+1}')
    ax[0].plot(rec['drive'][:, 0] - t0, rec['drive'][:, 1], 'k--', lw=1,
               label='recorded live Humble (different reference heading)')
    ax[1].plot(rec['drive'][:, 0] - t0, rec['drive'][:, 2], 'k--', lw=1, label='recorded live Humble')
    ax[0].set_ylabel('steering angle [rad]')
    ax[1].set_ylabel('speed [m/s]')
    ax[1].set_xlabel('bag time [s]')
    ax[0].legend(fontsize=7)
    ax[0].set_title('/drive: open-loop replays of humble_obstacle_run, injected goal at t=0.5 s')
    fig.tight_layout()
    fig.savefig(out / 'mpc_commands_overlay.png', dpi=110)
    plt.close(fig)

    fig, ax = plt.subplots(2, 1, figsize=(12, 6), sharex=True)
    a = jz[0]
    grid = np.arange(a['drive'][0, 0] + 1.0, a['drive'][-1, 0], 0.02)
    for i, b in enumerate(jz[1:], start=2):
        ds = mm.zoh(a['drive'][:, 0], a['drive'][:, 1], grid) - mm.zoh(b['drive'][:, 0], b['drive'][:, 1], grid)
        dv = mm.zoh(a['drive'][:, 0], a['drive'][:, 2], grid) - mm.zoh(b['drive'][:, 0], b['drive'][:, 2], grid)
        ax[0].plot(grid - t0, ds, lw=0.8, label=f'run1 - run{i}')
        ax[1].plot(grid - t0, dv, lw=0.8, label=f'run1 - run{i}')
    ax[0].set_ylabel('d steering [rad]')
    ax[1].set_ylabel('d speed [m/s]')
    ax[1].set_xlabel('bag time [s]')
    ax[0].legend(fontsize=7)
    ax[0].set_title('Jazzy-vs-Jazzy noise floor (identical inputs and goal; ZOH 50 Hz)')
    fig.tight_layout()
    fig.savefig(out / 'mpc_floor_diff.png', dpi=110)
    plt.close(fig)

    fig, ax = plt.subplots(figsize=(10, 5))
    series = [('recorded Humble/Orin (live, loaded)', rec)]
    for spec in args.timing:
        label, path = spec.split('=', 1)
        ticks, _, _ = mpc_frozen_io.load_ticks(path, ('wall_dt',))
        series.append((label, np.array([t['wall_dt'] for t in ticks])))
    bins = np.linspace(0, 130, 66)
    for label, r in series:
        dt = (np.array([s['solve_dt'] for s in r['status']]) if isinstance(r, dict) else r) * 1e3
        ax.hist(dt, bins=bins, histtype='step', lw=1.3,
                label=f'{label}: p50 {np.percentile(dt, 50):.1f} / p99 {np.percentile(dt, 99):.1f} ms')
    ax.set_xlabel('solve_mpc_step wall time [ms]')
    ax.set_ylabel('ticks')
    ax.set_title('MPC solve time -- Thor numbers are INDICATIVE only (target is the Orin)')
    ax.legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(out / 'mpc_solve_time.png', dpi=110)
    plt.close(fig)
    print('wrote plots to', out)


if __name__ == '__main__':
    main()
