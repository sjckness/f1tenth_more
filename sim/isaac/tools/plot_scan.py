"""Polar plot of a /scan sample with the room outline (from the scene YAML) overlaid.

    /isaac-sim/python.sh tools/plot_scan.py <run_dir | scan.npz> <scene_yaml> <car_spec_yaml> <out_png> [title]
A .npz is a sim snapshot (main.py --snapshot-at: ranges, angles, pose); a run dir holds frames.npz.
"""
from __future__ import annotations

import math
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
import yaml  # noqa: E402


def main(run: str, scene_yaml: str, car_yaml: str, out: str, title: str = "") -> int:
    d = np.load(run if run.endswith(".npz") else f"{run}/frames.npz")
    sc = yaml.safe_load(open(scene_yaml))
    cs = yaml.safe_load(open(car_yaml))
    r, a = d["ranges"].astype(float), d["angles"].astype(float)
    x, y, yaw = d["pose"]
    lx, ly, _ = cs["sensors"]["laser"]["xyz"]["value"]
    ox, oy = x + math.cos(yaw) * lx - math.sin(yaw) * ly, y + math.sin(yaw) * lx + math.cos(yaw) * ly
    fig = plt.figure(figsize=(7, 7))
    ax = fig.add_subplot(111, projection="polar")
    fin = np.isfinite(r)
    ax.scatter(a[fin], r[fin], s=2, c="tab:blue", label="/scan (sim)")
    if "room" in sc:
        sx, sy = sc["room"]["size_x"], sc["room"]["size_y"]
        corners = np.array([[0, 0], [sx, 0], [sx, sy], [0, sy], [0, 0]], dtype=float)
        pts = []
        for i in range(4):
            for s_ in np.linspace(0, 1, 400):
                pts.append(corners[i] + s_ * (corners[i + 1] - corners[i]))
        pts = np.array(pts) - [ox, oy]
        ang = np.arctan2(pts[:, 1], pts[:, 0]) - yaw
        ang = np.arctan2(np.sin(ang), np.cos(ang))
        ax.plot(ang[np.argsort(ang)], np.hypot(pts[:, 0], pts[:, 1])[np.argsort(ang)], "r-", lw=0.8,
                label="room walls (scene yaml)")
    if sc.get("person", {}).get("enabled"):
        px, py = sc["person"]["position"][:2]
        ax.plot([math.atan2(py - oy, px - ox) - yaw], [math.hypot(px - ox, py - oy)], "g^", ms=9,
                label="person centre")
    ax.set_theta_zero_location("N")
    ax.set_thetamin(-135)
    ax.set_thetamax(135)
    n10 = int(np.sum(r < 10.0))
    ax.set_title(f"{title + chr(10) if title else ''}/scan at pose x={x:.2f} y={y:.2f} yaw={math.degrees(yaw):.1f} deg "
                 f"(0 deg = car forward, CCW +)\n{n10}/{r.size} beams < 10 m ({100.0 * n10 / r.size:.1f} %), "
                 f"{int(fin.sum())} finite", fontsize=9)
    ax.legend(loc="lower left", fontsize=8)
    fig.savefig(out, dpi=110, bbox_inches="tight")
    return 0


if __name__ == "__main__":
    sys.exit(main(*sys.argv[1:6]))
