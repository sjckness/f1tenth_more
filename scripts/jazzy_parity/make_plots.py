#!/usr/bin/env python3
"""Generate the Phase 1 report's plots from the recorded run bags."""
import math
import sys
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

sys.path.insert(0, str(Path(__file__).parent))
from compare_runs import load_pose_series, load_tf_series, nearest_match, wrap

REPO = Path(__file__).resolve().parents[2]
HUMBLE_BAG = '/home/andre/bags/humble_reference/humble_obstacle_run/bag'
RUNS = REPO / 'output/phase1/runs'
OUT = REPO / 'output/phase1/plots'
OUT.mkdir(parents=True, exist_ok=True)

jazzy_bags = [str(RUNS / f'jazzy_ekf_global_{i}/bag') for i in (1, 2, 3)]
jazzy_relay_bags = [str(RUNS / f'jazzy_relay_{i}/bag') for i in (1, 2, 3)]

# ---- 1. trajectories overlaid ----
humble = load_pose_series(HUMBLE_BAG, '/ekf_global/odometry/filtered')
jazzy = load_pose_series(jazzy_bags[0], '/ekf_global/odometry/filtered')

fig, ax = plt.subplots(figsize=(8, 6))
ax.plot([r[1] for r in humble], [r[2] for r in humble], label='Humble (recorded)', lw=2)
ax.plot([r[1] for r in jazzy], [r[2] for r in jazzy], label='Jazzy (replay run 1)', lw=1, ls='--')
ax.scatter([humble[0][1]], [humble[0][2]], c='green', marker='o', s=60, label='Humble start', zorder=5)
ax.scatter([jazzy[0][1]], [jazzy[0][2]], c='red', marker='x', s=60, label='Jazzy start (cold seed)', zorder=5)
ax.set_xlabel('map x (m)'); ax.set_ylabel('map y (m)')
ax.set_title('/ekf_global/odometry/filtered trajectory: Jazzy replay vs Humble recording')
ax.legend(); ax.axis('equal'); ax.grid(alpha=0.3)
fig.tight_layout()
fig.savefig(OUT / 'trajectory_overlay.png', dpi=130)
plt.close(fig)

# ---- 2. position/yaw error over time (full window, showing the transient) ----
t0 = humble[0][0]
matched = nearest_match(jazzy, humble, max_dt=0.05)
ts, pos_errs, yaw_errs = [], [], []
for r, o, dt in matched:
    if o is None:
        continue
    ts.append(r[0] - t0)
    pos_errs.append(math.hypot(r[1] - o[1], r[2] - o[2]))
    yaw_errs.append(abs(math.degrees(wrap(r[3] - o[3]))))

fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(9, 6), sharex=True)
ax1.plot(ts, pos_errs)
ax1.axvline(20, color='gray', ls=':', label='t=20s: steady-state cutoff used in report')
ax1.set_ylabel('position error (m)')
ax1.set_title('ekf_global: Jazzy-vs-Humble error over time (cold-start convergence then steady state)')
ax1.legend(); ax1.grid(alpha=0.3)
ax2.plot(ts, yaw_errs)
ax2.axvline(20, color='gray', ls=':')
ax2.set_xlabel('t (s), relative to bag start')
ax2.set_ylabel('yaw error (deg)')
ax2.grid(alpha=0.3)
fig.tight_layout()
fig.savefig(OUT / 'error_over_time.png', dpi=130)
plt.close(fig)

# ---- 3. map->odom over time (translation + yaw), Humble vs all 3 Jazzy runs ----
humble_mo = load_tf_series(HUMBLE_BAG, 'map', 'odom')
jazzy_mos = [load_tf_series(b, 'map', 'odom') for b in jazzy_bags]

fig, axes = plt.subplots(3, 1, figsize=(9, 8), sharex=True)
t0 = humble_mo[0][0]
axes[0].plot([r[0] - t0 for r in humble_mo], [r[1] for r in humble_mo], label='Humble', lw=2)
axes[1].plot([r[0] - t0 for r in humble_mo], [r[2] for r in humble_mo], label='Humble', lw=2)
axes[2].plot([r[0] - t0 for r in humble_mo],
             [math.degrees(r[3]) for r in humble_mo], label='Humble', lw=2)
for i, mo in enumerate(jazzy_mos):
    tj0 = mo[0][0]
    axes[0].plot([r[0] - tj0 for r in mo], [r[1] for r in mo], label=f'Jazzy run {i+1}', lw=1, alpha=0.8)
    axes[1].plot([r[0] - tj0 for r in mo], [r[2] for r in mo], label=f'Jazzy run {i+1}', lw=1, alpha=0.8)
    axes[2].plot([r[0] - tj0 for r in mo],
                 [math.degrees(r[3]) for r in mo], label=f'Jazzy run {i+1}', lw=1, alpha=0.8)
axes[0].set_ylabel('map->odom x (m)')
axes[1].set_ylabel('map->odom y (m)')
axes[2].set_ylabel('map->odom yaw (deg)')
axes[2].set_xlabel('t (s), relative to each run\'s own start')
axes[0].set_title('map->odom over time: Humble vs 3 Jazzy replay runs')
axes[0].legend(fontsize=8); axes[0].grid(alpha=0.3)
axes[1].grid(alpha=0.3); axes[2].grid(alpha=0.3)
fig.tight_layout()
fig.savefig(OUT / 'map_odom_over_time.png', dpi=130)
plt.close(fig)

# ---- 4. correction-step sizes at each pose0 event, Humble vs Jazzy ----
humble_pose0 = load_pose_series(HUMBLE_BAG, '/slam/pose_calibrated')
from compare_runs import correction_steps
humble_steps = correction_steps(humble_pose0, humble_mo)
jazzy_steps = [correction_steps(humble_pose0, mo) for mo in jazzy_mos]

fig, ax = plt.subplots(figsize=(10, 5))
ax.plot([s['t'] - t0 for s in humble_steps], [s['d_yaw_deg'] for s in humble_steps],
        'o-', label='Humble', ms=4)
for i, js in enumerate(jazzy_steps):
    ax.plot([s['t'] - t0 for s in js], [s['d_yaw_deg'] for s in js],
            'x--', label=f'Jazzy run {i+1}', ms=4, alpha=0.7)
ax.set_xlabel('t (s)'); ax.set_ylabel('map->odom yaw step at pose0 correction (deg)')
ax.set_title('Per-correction yaw step size: Humble vs Jazzy')
ax.legend(fontsize=8); ax.grid(alpha=0.3)
fig.tight_layout()
fig.savefig(OUT / 'correction_steps.png', dpi=130)
plt.close(fig)

# ---- 5. relay layer: confirm zero error (sanity plot) ----
humble_relay = load_pose_series(HUMBLE_BAG, '/slam/pose_calibrated')
jazzy_relay = load_pose_series(jazzy_relay_bags[0], '/slam/pose_calibrated')
matched = nearest_match(jazzy_relay, humble_relay, max_dt=0.05)
ts_r = [r[0] - t0 for r, o, dt in matched if o is not None]
errs_r = [math.hypot(r[1]-o[1], r[2]-o[2]) for r, o, dt in matched if o is not None]
fig, ax = plt.subplots(figsize=(9, 3))
ax.plot(ts_r, errs_r, 'o', ms=3)
ax.set_xlabel('t (s)'); ax.set_ylabel('position error (m)')
ax.set_title('slam_pose_relay_node: Jazzy-vs-Humble per-message error (bit-exact expected)')
ax.grid(alpha=0.3)
fig.tight_layout()
fig.savefig(OUT / 'relay_error.png', dpi=130)
plt.close(fig)

print('wrote plots to', OUT)
for p in sorted(OUT.glob('*.png')):
    print(' ', p.name)
