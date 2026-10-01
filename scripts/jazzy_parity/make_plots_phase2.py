#!/usr/bin/env python3
"""Phase 2 report plots: SLAM trajectory + map diff, boundary constraint
error over time, front_clearance overlay. Reuses compare_runs.py's own
data-loading/metric functions rather than reimplementing them."""
import math
import sys
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

sys.path.insert(0, str(Path(__file__).parent))
from compare_runs import (
    load_pose_series, load_boundaries_series, load_scalar_series,
    load_map, nearest_match, resample_zoh)

REPO = Path(__file__).resolve().parents[2]
HUMBLE_BAG = '/home/andre/bags/humble_reference/humble_obstacle_run/bag'
RUNS = REPO / 'output/phase2/runs'
OUT = REPO / 'output/phase2/plots'
OUT.mkdir(parents=True, exist_ok=True)

slam_bags = [str(RUNS / f'jazzy_slam_{i}/bag') for i in (1, 2, 3)]
boundary_bags = [str(RUNS / f'jazzy_costmap_boundary_{i}/bag') for i in (1, 2, 3)]

# ---- 1. SLAM trajectory overlay (resampled 1Hz, since raw event times
# are themselves non-deterministic -- see resample_zoh's own docstring) ----
humble_pose = load_pose_series(HUMBLE_BAG, '/slam/pose')
jazzy_poses = [load_pose_series(b, '/slam/pose') for b in slam_bags]
t0 = max(humble_pose[0][0], min(j[0][0] for j in jazzy_poses))
t1 = min(humble_pose[-1][0], min(j[-1][0] for j in jazzy_poses))
sample_times = np.arange(t0, t1, 1.0)
h_rs = [r for r in resample_zoh(humble_pose, sample_times) if r is not None]

fig, ax = plt.subplots(figsize=(8, 6))
ax.plot([r[1] for r in h_rs], [r[2] for r in h_rs], label='Humble (recorded)', lw=2, marker='o', ms=4)
for i, j in enumerate(jazzy_poses):
    j_rs = [r for r in resample_zoh(j, sample_times) if r is not None]
    ax.plot([r[1] for r in j_rs], [r[2] for r in j_rs],
             label=f'Jazzy run {i+1}', lw=1, ls='--', marker='x', ms=4, alpha=0.8)
ax.set_xlabel('map x (m)'); ax.set_ylabel('map y (m)')
ax.set_title('/slam/pose trajectory (1Hz resampled): Jazzy replay vs Humble')
ax.legend(); ax.axis('equal'); ax.grid(alpha=0.3)
fig.tight_layout()
fig.savefig(OUT / 'slam_trajectory_overlay.png', dpi=130)
plt.close(fig)

# ---- 2. map diff image: Humble final map vs Jazzy run 1 final map ----
humble_map = load_map(HUMBLE_BAG)
jazzy_map = load_map(slam_bags[0])


def grid_rgb(mp):
    arr = np.array(mp.data, dtype=np.int16).reshape(mp.info.height, mp.info.width)
    rgb = np.zeros((mp.info.height, mp.info.width, 3), dtype=np.uint8)
    rgb[arr == -1] = (128, 128, 128)       # unknown: gray
    rgb[(arr >= 0) & (arr < 65)] = (255, 255, 255)  # free: white
    rgb[arr >= 65] = (0, 0, 0)             # occupied: black
    return rgb


fig, axes = plt.subplots(1, 2, figsize=(12, 5))
axes[0].imshow(grid_rgb(humble_map), origin='lower')
axes[0].set_title(f'Humble final map ({humble_map.info.width}x{humble_map.info.height})')
axes[1].imshow(grid_rgb(jazzy_map), origin='lower')
axes[1].set_title(f'Jazzy run 1 final map ({jazzy_map.info.width}x{jazzy_map.info.height})')
for a in axes:
    a.axis('off')
fig.suptitle('Final /slam/map: black=occupied, white=free, gray=unknown')
fig.tight_layout()
fig.savefig(OUT / 'slam_map_diff.png', dpi=130)
plt.close(fig)

# ---- 3. boundary constraint error over time ----
humble_b = load_boundaries_series(HUMBLE_BAG)
jazzy_b = load_boundaries_series(boundary_bags[0])
matched = nearest_match(jazzy_b, humble_b, max_dt=0.1)
tb0 = humble_b[0][0]
ts, angs = [], []
for r, o, dt in matched:
    if o is None or len(r[1]) != len(o[1]):
        continue
    for (rnx, rny, roff), (onx, ony, ooff) in zip(r[1], o[1]):
        dot = max(-1.0, min(1.0, rnx * onx + rny * ony))
        ts.append(r[0] - tb0)
        angs.append(math.degrees(math.acos(dot)))

fig, ax = plt.subplots(figsize=(10, 4))
ax.plot(ts, angs, '.', ms=3, alpha=0.5)
ax.axhline(5, color='gray', ls=':', label='5deg (outlier threshold used in report)')
ax.set_xlabel('t (s)'); ax.set_ylabel('constraint normal angle error (deg)')
ax.set_title('/costmap/boundaries: per-constraint normal error, Jazzy run 1 vs Humble\n'
             '(median agreement is tight; occasional large jumps are nearest-cell-method\n'
             'discontinuities near decision boundaries, amplified by periodic-timer sampling)')
ax.legend(); ax.grid(alpha=0.3)
fig.tight_layout()
fig.savefig(OUT / 'boundary_error_over_time.png', dpi=130)
plt.close(fig)

# ---- 4. front_clearance overlay ----
humble_c = load_scalar_series(HUMBLE_BAG, '/costmap/front_clearance')
jazzy_c = load_scalar_series(boundary_bags[0], '/costmap/front_clearance')
tc0 = humble_c[0][0]
fig, ax = plt.subplots(figsize=(10, 4))
ax.plot([r[0] - tc0 for r in humble_c], [r[1] for r in humble_c], label='Humble', lw=1.5)
ax.plot([r[0] - tc0 for r in jazzy_c], [r[1] for r in jazzy_c], label='Jazzy run 1', lw=1, ls='--', alpha=0.8)
ax.set_xlabel('t (s)'); ax.set_ylabel('front_clearance (m)')
ax.set_title('/costmap/front_clearance: Jazzy run 1 vs Humble')
ax.legend(); ax.grid(alpha=0.3)
fig.tight_layout()
fig.savefig(OUT / 'front_clearance_overlay.png', dpi=130)
plt.close(fig)

print('wrote plots to', OUT)
for p in sorted(OUT.glob('*.png')):
    print(' ', p.name)
