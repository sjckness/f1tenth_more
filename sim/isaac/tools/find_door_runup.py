#!/usr/bin/env python3
"""Find a start pose whose straight run-up ENDS BESIDE A DOORWAY, on a chosen side.

    sim/isaac/tools/find_door_runup.py [--run 5.0] [--side left|right] [--grid 0.05] [--json FILE]

"go straight N m then enter the door on the left" needs a geometry no hand-picked pose had: the
car must drive N m in a straight line and arrive with a doorway that far to one side. config/
scenes/office.yaml's `door` preset starts 2 m from its doorway facing it, which is the END of
such a command, not its start.

This reads the SDF and nothing else -- no simulator, no USD -- by reusing pick_spawn_poses'
occupancy grid (every COLLISION geometry projected to the lidar plane) and its doorways() finder,
so the poses it prints are in the same metres as Gazebo and the generated USD.

How a candidate is built, for each doorway the finder returns:

* the doorway's own approach point A sits `standoff` (2.0 m) out along the passage axis, facing
  the opening, and doorways() has already checked that A is on drivable floor;
* u is the unit vector from A through the door. A heading v has the door on its LEFT when
  left(v) == u, i.e. v = (u_y, -u_x); on its RIGHT when v = (-u_y, u_x);
* the start is S = A - run * v, so driving `run` metres along v arrives exactly at A.

A candidate is kept only when S is on floor with more than a footprint radius of clearance, the
straight run from S is free for at least `run` (plus --beyond), and the clearance never drops to
the footprint radius anywhere along it. The grid over-estimates occupancy (axis-aligned boxes per
triangle), so a pose this tool calls free is free; one it rejects may not be.

`free_beyond_arrival_m` is worth reading before picking: corridor_rrt builds its corridor
rrt_corridor_length_drive_m past the car on an open-ended drive and reports `no path` once that
end is blocked, so a wall close behind the door stalls the drive phase before the door phase
starts (stack_params.yaml, rrt_target_retreat).
"""
from __future__ import annotations

import argparse
import json
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import numpy as np  # noqa: E402
from pick_spawn_poses import (clearance_map, doorways, FOOTPRINT_R, free_run,  # noqa: E402
                              occupancy, world_xy)  # noqa: F401
from sdf_to_usd import DEFAULT_FUEL, DEFAULT_MODELS, DEFAULT_SDF, parse_world  # noqa: E402


def main() -> int:
    """Print the start poses whose run-up ends beside a doorway."""
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--sdf", default=DEFAULT_SDF)
    ap.add_argument("--models-dir", default=DEFAULT_MODELS)
    ap.add_argument("--fuel-cache", default=DEFAULT_FUEL)
    ap.add_argument("--run", type=float, default=5.0, help="straight run-up [m]")
    ap.add_argument("--side", choices=("left", "right"), default="left")
    ap.add_argument("--grid", type=float, default=0.05, help="grid resolution [m]")
    ap.add_argument("--beyond", type=float, default=0.0,
                    help="also require this much free space past the arrival point [m]")
    ap.add_argument("--json", default="", help="write the result here")
    args = ap.parse_args()

    models, _meta = parse_world(args.sdf)
    occ, origin, res, floor, _raster = occupancy(models, args.models_dir, args.fuel_cache,
                                                 args.grid)
    nx, ny = occ.shape

    def floor_at(x: float, y: float):
        i, j = int((x - origin[0]) / res), int((y - origin[1]) / res)
        if not (0 <= i < floor.shape[0] and 0 <= j < floor.shape[1]):
            return None
        z = float(floor[i, j])
        return None if not math.isfinite(z) else round(z, 3)

    clear = clearance_map(occ, res)
    inside = np.isfinite(floor) & (np.abs(np.where(np.isfinite(floor), floor, 1e3)) < 0.05)
    clear = np.where(inside, clear, 0.0)

    doors = doorways(occ, clear, origin, res, floor_at)
    print(f"grid {nx} x {ny} at {res} m; doorways: {len(doors)}")

    want_left = args.side == "left"
    out = []
    for d in doors:
        use = d.get("use_approach")
        if not use:
            continue
        app = d[f"approach_{use}"]
        yaw_app = math.radians(app["yaw_deg"])
        ux, uy = math.cos(yaw_app), math.sin(yaw_app)
        vx, vy = (uy, -ux) if want_left else (-uy, ux)
        sx, sy = app["x"] - args.run * vx, app["y"] - args.run * vy
        i, j = int((sx - origin[0]) / res), int((sy - origin[1]) / res)
        if not (0 <= i < nx and 0 <= j < ny):
            continue
        fz = floor_at(sx, sy)
        start_clear = float(clear[i, j])
        if fz is None or start_clear <= FOOTPRINT_R:
            continue
        di, dj = int(round(vx)), int(round(vy))
        if (di, dj) == (0, 0):
            continue
        ahead = free_run(occ, i, j, di, dj, res, limit=40.0)
        if ahead < args.run + args.beyond:
            continue
        lateral = []
        for k in range(max(int(args.run / res), 1) + 1):
            px, py = sx + vx * k * res, sy + vy * k * res
            pi, pj = int((px - origin[0]) / res), int((py - origin[1]) / res)
            if 0 <= pi < nx and 0 <= pj < ny:
                lateral.append(float(clear[pi, pj]))
        min_lateral = min(lateral) if lateral else 0.0
        if min_lateral <= FOOTPRINT_R:
            continue
        out.append({
            "start": {"x": round(sx, 3), "y": round(sy, 3), "z": 0.02,
                      "yaw_deg": round(math.degrees(math.atan2(vy, vx)), 1)},
            "run_m": args.run,
            "side": args.side,
            "door": {"x": d["x"], "y": d["y"], "width_m": d["width_m"],
                     "passage_axis": d["passage_axis"]},
            "free_ahead_m": round(ahead, 2),
            "free_beyond_arrival_m": round(ahead - args.run, 2),
            "min_lateral_clearance_m": round(min_lateral, 3),
            "start_clearance_m": round(start_clear, 3),
            "floor_z_m": fz,
        })

    out.sort(key=lambda c: (-min(c["free_beyond_arrival_m"], 6.0), -c["door"]["width_m"],
                            -c["min_lateral_clearance_m"]))
    print(f"start poses with {args.run:.1f} m of run-up and a door on the {args.side}: {len(out)}")
    for c in out:
        s, dr = c["start"], c["door"]
        print(f"  ({s['x']:+8.3f}, {s['y']:+8.3f}) yaw {s['yaw_deg']:+6.1f}  ->  door "
              f"({dr['x']:+8.3f}, {dr['y']:+8.3f}) {dr['width_m']:.2f} m  |  free ahead "
              f"{c['free_ahead_m']:5.2f} m ({c['free_beyond_arrival_m']:5.2f} past arrival)  |  "
              f"lateral min {c['min_lateral_clearance_m']:.2f} m")
    if args.json:
        with open(args.json, "w") as fh:
            json.dump({"run_m": args.run, "side": args.side, "candidates": out,
                       "doorways": doors}, fh, indent=2)
        print(f"wrote {args.json}")
    return 0 if out else 1


if __name__ == "__main__":
    sys.exit(main())
