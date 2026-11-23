#!/usr/bin/env python3
"""Find a start pose with a straight run-up that ends with a DOORWAY ON THE LEFT.

For "go straight N m then enter the door on the left": the car must start somewhere
it can drive N m in a straight line and arrive beside a doorway, with the opening to
its left. Reuses pick_spawn_poses' occupancy grid and doorway finder, so this needs
no simulator -- only the SDF and the model meshes.

    python find_door_runup.py [--run 5.0] [--side left|right] [--json OUT]
"""
from __future__ import annotations

import argparse
import json
import math
import os
import sys

TOOLS = os.path.join(os.path.dirname(os.path.abspath(__file__)))
# the repo's tools directory, passed in by --tools or guessed from this file's location
DEFAULT_TOOLS = r"F:\Users\Andreas Steffens\dev_ws\f1tenth_hil\sim\isaac\tools"


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--tools", default=DEFAULT_TOOLS)
    ap.add_argument("--run", type=float, default=5.0, help="straight run-up [m]")
    ap.add_argument("--side", choices=("left", "right"), default="left")
    ap.add_argument("--grid", type=float, default=0.05)
    ap.add_argument("--beyond", type=float, default=0.0,
                    help="also require this much free space past the arrival point")
    ap.add_argument("--json", default="")
    args = ap.parse_args()

    sys.path.insert(0, args.tools)
    import numpy as np
    from pick_spawn_poses import (clearance_map, doorways, FOOTPRINT_R, free_run,  # noqa: E402
                                  occupancy, world_xy)
    from sdf_to_usd import DEFAULT_FUEL, DEFAULT_MODELS, DEFAULT_SDF, parse_world  # noqa: E402

    print(f"SDF: {DEFAULT_SDF}")
    models, _meta = parse_world(DEFAULT_SDF)
    occ, origin, res, floor, _raster = occupancy(models, DEFAULT_MODELS, DEFAULT_FUEL, args.grid)
    nx, ny = occ.shape
    print(f"griglia {nx} x {ny} a {res} m")

    def floor_at(x, y):
        i, j = int((x - origin[0]) / res), int((y - origin[1]) / res)
        if not (0 <= i < floor.shape[0] and 0 <= j < floor.shape[1]):
            return None
        z = float(floor[i, j])
        return None if not math.isfinite(z) else round(z, 3)

    clear = clearance_map(occ, res)
    inside = np.isfinite(floor) & (np.abs(np.where(np.isfinite(floor), floor, 1e3)) < 0.05)
    clear = np.where(inside, clear, 0.0)

    doors = doorways(occ, clear, origin, res, floor_at)
    print(f"vani trovati: {len(doors)}")
    for d in doors:
        print(f"  ({d['x']:+.3f}, {d['y']:+.3f})  largh {d['width_m']:.2f} m  "
              f"passaggio lungo {d['passage_axis']}  libero oltre {d['free_through_m']}")

    want_left = args.side == "left"
    out = []
    for d in doors:
        use = d.get("use_approach")
        if not use:
            continue
        app = d[f"approach_{use}"]
        yaw_app = math.radians(app["yaw_deg"])
        ux, uy = math.cos(yaw_app), math.sin(yaw_app)      # approach -> through the door
        # heading whose LEFT is u:  v = (uy, -ux);  whose RIGHT is u: v = (-uy, ux)
        vx, vy = (uy, -ux) if want_left else (-uy, ux)
        sx, sy = app["x"] - args.run * vx, app["y"] - args.run * vy
        i, j = int((sx - origin[0]) / res), int((sy - origin[1]) / res)
        if not (0 <= i < nx and 0 <= j < ny):
            continue
        fz = floor_at(sx, sy)
        cl = float(clear[i, j])
        if fz is None or cl <= FOOTPRINT_R:
            continue
        di, dj = int(round(vx)), int(round(vy))
        if (di, dj) == (0, 0):
            continue
        ahead = free_run(occ, i, j, di, dj, res, limit=40.0)
        if ahead < args.run + args.beyond:
            continue
        # lateral room all along the run
        lat = []
        steps = max(int(args.run / res), 1)
        for k in range(steps + 1):
            px, py = sx + vx * k * res, sy + vy * k * res
            pi, pj = int((px - origin[0]) / res), int((py - origin[1]) / res)
            if 0 <= pi < nx and 0 <= pj < ny:
                lat.append(float(clear[pi, pj]))
        min_lat = min(lat) if lat else 0.0
        if min_lat <= FOOTPRINT_R:
            continue
        out.append({
            "start": {"x": round(sx, 3), "y": round(sy, 3), "z": 0.02,
                      "yaw_deg": round(math.degrees(math.atan2(vy, vx)), 1)},
            "run_m": args.run,
            "door": {"x": d["x"], "y": d["y"], "width_m": d["width_m"],
                     "passage_axis": d["passage_axis"]},
            "side": args.side,
            "free_ahead_m": round(ahead, 2),
            "free_beyond_arrival_m": round(ahead - args.run, 2),
            "min_lateral_clearance_m": round(min_lat, 3),
            "start_clearance_m": round(cl, 3),
            "floor_z_m": fz,
        })

    out.sort(key=lambda c: (-c["min_lateral_clearance_m"], -c["free_beyond_arrival_m"]))
    print(f"\ncandidati con {args.run:.1f} m di rincorsa e porta a {args.side}: {len(out)}")
    for c in out:
        s = c["start"]
        print(f"  start ({s['x']:+.3f}, {s['y']:+.3f}) yaw {s['yaw_deg']:+.1f} deg"
              f" -> porta ({c['door']['x']:+.3f}, {c['door']['y']:+.3f}) largh {c['door']['width_m']:.2f}"
              f" | libero avanti {c['free_ahead_m']:.2f} m (oltre l arrivo {c['free_beyond_arrival_m']:.2f})"
              f" | laterale min {c['min_lateral_clearance_m']:.2f} m")
    if args.json:
        with open(args.json, "w") as fh:
            json.dump({"run_m": args.run, "side": args.side, "candidates": out,
                       "doorways": doors}, fh, indent=2)
        print(f"\nscritto {args.json}")
    return 0 if out else 1


if __name__ == "__main__":
    sys.exit(main())
