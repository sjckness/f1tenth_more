#!/usr/bin/env python3
"""
plot_corridor.py

Standalone visualizer for mpc_corr's corridor debug log. Reads
<ws_root>/src/f1tenth_control/corridors_jsons/corridor_debug.jsonl (written by
MPCController.save_corridor_snapshot whenever self.save_corridor_debug = True,
via MPC_corr.py's _resolve_debug_output_path()) and plots the corridor
geometry with matplotlib. No ROS import, no Foxglove -- just reads the JSONL
file directly, so it works even while mpc_corr is running and still writing
to it.

DEFAULT_PATH below previously hardcoded ~/ros2_f110_ws/corridor_debug.jsonl --
a stale reference to this project's old workspace name AND the wrong location
even under that old name (MPC_corr.py itself migrated corridor_log_path off
that same hardcoded pattern a while back; this script's own default was never
updated to match). Old-workspace-name cleanup pass: now resolved the same
portable way MPC_corr.py itself uses, walking up from this file's own location
for the src/ ancestor rather than assuming a fixed home-relative path.

Usage
-----
    python3 plot_corridor.py                    # plot the most recent snapshot
    python3 plot_corridor.py --index 42         # plot snapshot #42 (0-indexed)
    python3 plot_corridor.py --animate          # step through every snapshot in the file
    python3 plot_corridor.py --live             # keep watching the file, plot new
                                                # snapshots as mpc_corr appends them
    python3 plot_corridor.py --path /other/file.jsonl

Dependencies
------------
    pip install matplotlib --break-system-packages
"""

import argparse
import json
import time
from pathlib import Path

import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np


# This script lives at the workspace root (next to src/), always run from
# source (never colcon-installed), so -- unlike MPC_corr.py's own
# _resolve_debug_output_path(), which has to handle being copied into
# install/ -- a plain __file__-relative anchor is sufficient here.
DEFAULT_PATH = (Path(__file__).resolve().parent / "src" / "f1tenth_control"
                 / "corridors_jsons" / "corridor_debug.jsonl")


def read_snapshots(path: Path):
    """Read every JSON line in the file, skipping any that fail to parse
    (e.g. a partially-written last line while mpc_corr is mid-write)."""
    snapshots = []
    if not path.exists():
        return snapshots
    with open(path, "r", encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            try:
                snapshots.append(json.loads(line))
            except json.JSONDecodeError:
                continue
    return snapshots


def draw_snapshot(ax, snap, title_suffix=""):
    """Draw one corridor snapshot onto the given axes. Clears the axes first."""
    ax.clear()

    corridor = snap["corridor"]
    robot = snap["robot"]
    target = snap["target"]
    obstacles = snap.get("obstacles_world", [])

    xc = np.array(corridor["xc"])
    yc = np.array(corridor["yc"])
    xL = np.array(corridor["xL"])
    yL = np.array(corridor["yL"])
    xR = np.array(corridor["xR"])
    yR = np.array(corridor["yR"])
    Pend = corridor["Pend"]

    # corridor width envelope, shaded
    ax.fill(
        np.concatenate([xL, xR[::-1]]),
        np.concatenate([yL, yR[::-1]]),
        color="tab:cyan", alpha=0.15, label="corridor width", zorder=1,
    )

    ax.plot(xc, yc, "--", color="teal", linewidth=2, label="centerline (arc)", zorder=2)
    ax.plot(xL, yL, "-", color="tab:blue", linewidth=1, label="left boundary", zorder=2)
    ax.plot(xR, yR, "-", color="tab:blue", linewidth=1, label="right boundary", zorder=2)

    # robot pose: position + heading arrow
    rx, ry, ryaw = robot["x"], robot["y"], robot["yaw"]
    if rx is not None and ry is not None:
        ax.plot(rx, ry, "o", color="tab:purple", markersize=9, label="robot", zorder=4)
        if ryaw is not None:
            arrow_len = 0.5
            ax.arrow(
                rx, ry,
                arrow_len * np.cos(ryaw), arrow_len * np.sin(ryaw),
                head_width=0.12, head_length=0.15,
                fc="tab:purple", ec="tab:purple", zorder=4,
            )

    # local target (lookahead point, post obstacle-deflection)
    ax.plot(target["x"], target["y"], "*", color="tab:orange", markersize=16,
             label="local target", zorder=4)

    # corridor end point
    ax.plot(Pend[0], Pend[1], "x", color="tab:gray", markersize=10,
             label="corridor end (Pend)", zorder=3)

    # obstacles
    for i, obs in enumerate(obstacles):
        circle = plt.Circle(
            (obs["x"], obs["y"]), obs["r"],
            color="tab:red", alpha=0.35, zorder=3,
            label="obstacle" if i == 0 else None,
        )
        ax.add_patch(circle)

    ax.set_aspect("equal", adjustable="datalim")
    ax.grid(True, alpha=0.3)
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_title(f"Corridor snapshot{title_suffix}")
    ax.legend(loc="upper left", fontsize=8, framealpha=0.9)


def mode_static(snapshots, index):
    if not snapshots:
        print("No snapshots found in file.")
        return
    if index is None:
        index = len(snapshots) - 1
    index = max(0, min(index, len(snapshots) - 1))

    fig, ax = plt.subplots(figsize=(8, 8))
    draw_snapshot(ax, snapshots[index], title_suffix=f"  (#{index}/{len(snapshots) - 1})")
    plt.tight_layout()
    plt.show()


def mode_animate(snapshots, interval_ms=150):
    if not snapshots:
        print("No snapshots found in file.")
        return

    fig, ax = plt.subplots(figsize=(8, 8))

    def update(i):
        draw_snapshot(ax, snapshots[i], title_suffix=f"  (#{i}/{len(snapshots) - 1})")

    anim = animation.FuncAnimation(
        fig, update, frames=len(snapshots), interval=interval_ms, repeat=True
    )
    plt.tight_layout()
    plt.show()
    return anim  # keep a reference so it isn't garbage-collected mid-animation


def mode_live(path: Path, poll_sec=0.3):
    """Keep tailing the file; redraw whenever a new complete line appears."""
    fig, ax = plt.subplots(figsize=(8, 8))
    plt.ion()
    plt.show()

    last_count = 0
    print(f"Watching {path} -- close the plot window or Ctrl+C to stop.")
    try:
        while plt.fignum_exists(fig.number):
            snapshots = read_snapshots(path)
            if len(snapshots) > last_count:
                last_count = len(snapshots)
                draw_snapshot(ax, snapshots[-1], title_suffix=f"  (#{last_count - 1}, live)")
                fig.canvas.draw()
            fig.canvas.flush_events()
            time.sleep(poll_sec)
    except KeyboardInterrupt:
        pass


def main():
    parser = argparse.ArgumentParser(description="Plot mpc_corr corridor debug snapshots.")
    parser.add_argument("--path", type=Path, default=DEFAULT_PATH,
                         help=f"Path to corridor_debug.jsonl (default: {DEFAULT_PATH})")
    parser.add_argument("--index", type=int, default=None,
                         help="Plot a specific snapshot by index (default: most recent)")
    parser.add_argument("--animate", action="store_true",
                         help="Step through every snapshot currently in the file")
    parser.add_argument("--live", action="store_true",
                         help="Keep watching the file and plot new snapshots as they arrive")
    parser.add_argument("--interval-ms", type=int, default=150,
                         help="Frame interval in ms for --animate (default: 150)")
    args = parser.parse_args()

    if args.live:
        mode_live(args.path)
        return

    snapshots = read_snapshots(args.path)
    print(f"Loaded {len(snapshots)} snapshot(s) from {args.path}")

    if args.animate:
        mode_animate(snapshots, interval_ms=args.interval_ms)
    else:
        mode_static(snapshots, args.index)


if __name__ == "__main__":
    main()