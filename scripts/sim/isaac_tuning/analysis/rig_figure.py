#!/usr/bin/env python3
"""The mechanism on the closed-loop rig: a cylinder crossing 2.5 m ahead at 0.4 m/s from the right.

    rig_figure.py --mpc-test-dir <mpc_controller/test> [--out output/isaac_tuning/figures]

Runs mpc_controller's test_dodge_closed_loop rig (real corridor, target and RTI solver, nominal
model) three times -- static obstacles, mpc_obs_vel_trust 1, trust 1 + wake 150 -- and draws the
car's path with the cylinder at a few instants. Needs the ROS / workspace environment of the
mpc_controller tests (scripts: worktree-test-env).
"""
import argparse
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import common  # noqa: E402
import numpy as np  # noqa: E402
from plots import CRITICAL, INK2, OBST, SERIES, plt  # noqa: E402
from matplotlib.patches import Circle  # noqa: E402


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.strip().splitlines()[0])
    ap.add_argument('--mpc-test-dir', required=True)
    ap.add_argument('--out', default=os.path.join(common.OUT_ROOT, 'figures'))
    args = ap.parse_args(argv)
    sys.path.insert(0, args.mpc_test_dir)
    import test_dodge_closed_loop as T
    runs = [('MPC statico (oggi)', {}), ('α = 1 (dischi predetti)', {'obs_vel_trust': 1.0}),
            ('α = 1 + scia', {'obs_vel_trust': 1.0, 'obs_wake_weight': 150.0})]
    fig, axes = plt.subplots(1, 3, figsize=(15, 4.6), sharey=True)
    for ax, (label, flags) in zip(axes, runs):
        r = T.drive(T.crossing(-1.0), 10.0, **flags)
        s = r.states
        t = np.asarray(r.t)
        for k, ti in enumerate((1.0, 3.0, 5.0, 7.0)):
            i = int(np.argmin(np.abs(t - ti)))
            ox, oy, rr = r.obs[i][0]
            ax.add_patch(Circle((ox, oy), rr, color=OBST, alpha=0.25 + 0.15 * k, zorder=2))
            ax.annotate(f'{ti:.0f} s', (ox + rr + 0.05, oy), fontsize=8, color=INK2, va='center')
            ax.plot(s[i, 0], s[i, 1], 'o', color=SERIES[0], markersize=4, zorder=4)
            ax.annotate(f'{ti:.0f} s', (s[i, 0], s[i, 1] + 0.12), fontsize=8, color=SERIES[0],
                        ha='center')
        ax.plot(s[:, 0], s[:, 1], color=SERIES[0], linewidth=2.2, zorder=3)
        ax.annotate('', xy=(2.5, 1.0), xytext=(2.5, -1.2),
                    arrowprops=dict(arrowstyle='->', color=INK2, lw=1.2))
        m = r.min_margin()
        c = r.contact()
        ax.set_title(f'{label}\nmargine al R_safe {m:+.2f} m' + ('  — CONTATTO' if c < 0 else ''),
                     loc='left', color=CRITICAL if c < 0 else INK2)
        ax.set_aspect('equal')
        ax.set_xlim(-0.2, 5.8)
        ax.set_ylim(-1.5, 2.6)
        ax.set_xlabel('x [m] (l\'auto va verso +x)')
    axes[0].set_ylabel('y [m]')
    fig.suptitle('Rig closed-loop: cilindro Ø0.5 che attraversa da destra a 0.4 m/s, 2.5 m avanti',
                 x=0.01, ha='left', fontweight='bold')
    fig.tight_layout()
    out = os.path.join(args.out, 'rig_attraversamento.png')
    fig.savefig(out, dpi=130)
    plt.close(fig)
    print(out)
    return 0


if __name__ == '__main__':
    sys.exit(main())
