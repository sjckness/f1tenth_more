#!/usr/bin/env python3
"""check_drive_path.py -- validate a hand-written drive OFFLINE, before touching Isaac.

    sim/isaac/tools/check_drive_path.py                        # the built-in office U-turn route
    sim/isaac/tools/check_drive_path.py --segments my.json     # your own segment list
    sim/isaac/tools/check_drive_path.py --layout open_space --json out.json

WHY THIS EXISTS. The Leatherback's USD velocity drives have no force limit, so a car that stalls
against an obstacle while still being commanded builds unbounded wheel torque until PhysX goes NaN
(sim/isaac/config/car_spec.yaml:68, "office, 170 s stalled at 0.4 m/s"). A hand-written drive that
runs into furniture does not just stop: it can corrupt the simulation. So the path is checked
against the world's collision geometry FIRST, and only then driven.

HOW. The segment list is integrated with the same kinematics the sim applies (bicycle model plus
the first-order lag and rate limits of sim/isaac/sim/car.py), from the scene's `car.start`. Every
pose is tested against the occupancy grid of pick_spawn_poses.py -- the collision geometry that
crosses the lidar band, rasterised triangle by triangle -- for

  * a free disc of FOOTPRINT_R around base_link, and
  * a drivable floor underneath (a cell can be "free" because it is OUTSIDE the building).

No Isaac, no USD libraries, no scipy: numpy only. Read-only.

LIMITS. The grid is the lidar band (0.05-0.35 m), so it cannot see an obstacle that is only higher
or only lower than that. The occupancy of each triangle is its axis-aligned bounding box, which
OVER-estimates: a path this tool calls clear is clear, but it may reject a tight one that would
actually fit.
"""
from __future__ import annotations

import argparse
import json
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

from pick_spawn_poses import FOOTPRINT_R, occupancy  # noqa: E402
from sdf_to_usd import DEFAULT_FUEL, DEFAULT_MODELS, DEFAULT_SDF, parse_world  # noqa: E402

ISAAC_DIR = os.path.dirname(HERE)
SCENES = os.path.join(ISAAC_DIR, "config", "scenes")

# sim/isaac/sim/car.py + config/car_spec.yaml (leatherback): the sim does not jump to the command.
WHEELBASE = 0.320           # leatherback, measured
STEER_MAX_LEFT = 0.31975
STEER_MAX_RIGHT = 0.30935
SPEED_TAU = 0.15
STEER_TAU = 0.08
ACCEL = 3.0
STEER_RATE = 3.0
DT = 0.02                   # finer than the 60 Hz physics step

# The routes are DATA, not code: sim/isaac/config/routes/*.json, editable segment by segment, and
# each file carries its own `notes` explaining why it is shaped the way it is. The default one is
# the validated office corridor loop.
ROUTES = os.path.join(ISAAC_DIR, "config", "routes")
DEFAULT_ROUTE = os.path.join(ROUTES, "office_corridor_loop.json")


def load_route(path: str):
    """A route file is either a bare list of segments or {"segments": [...], ...}."""
    with open(path, encoding="utf-8") as f:
        data = json.load(f)
    if isinstance(data, dict):
        return data.get("segments") or [], data
    return data, {}


def _lag_rate(cur: float, tgt: float, tau: float, rate: float, dt: float) -> float:
    """First-order lag with a rate limit -- the same shape as car.py's _lag_rate."""
    step = (tgt - cur) * (1.0 - math.exp(-dt / tau)) if tau > 1e-6 else (tgt - cur)
    return cur + max(-rate * dt, min(rate * dt, step))


def start_pose(scene: str, layout: str) -> tuple[float, float, float]:
    """(x, y, yaw_rad) of the layout's car.start, read from the scene yaml."""
    import yaml
    path = os.path.join(SCENES, f"{scene}.yaml")
    with open(path, encoding="utf-8") as f:
        data = yaml.safe_load(f) or {}
    car = data.get("car") or {}
    if layout:
        presets = data.get("presets") or {}
        preset = presets.get(layout) or {}
        if not preset:
            raise SystemExit(f"{path}: no preset '{layout}' (have: {sorted(presets.keys())})")
        car = preset.get("car") or car
    s = car.get("start") or {}
    return (float(s.get("x", 0.0)), float(s.get("y", 0.0)),
            math.radians(float(s.get("yaw_deg", 0.0))))


def integrate(segments, x: float, y: float, yaw: float):
    """Bicycle model with the sim's lag. Returns a list of (t, name, x, y, yaw, v)."""
    poses = []
    v = d = 0.0
    t = 0.0
    for seg in segments:
        tgt_v = float(seg["speed"])
        tgt_d = max(-STEER_MAX_RIGHT, min(STEER_MAX_LEFT, float(seg["steer"])))
        for _ in range(int(round(float(seg["dur"]) / DT))):
            v = _lag_rate(v, tgt_v, SPEED_TAU, ACCEL, DT)
            d = _lag_rate(d, tgt_d, STEER_TAU, STEER_RATE, DT)
            yaw += (v / WHEELBASE) * math.tan(d) * DT
            x += v * math.cos(yaw) * DT
            y += v * math.sin(yaw) * DT
            t += DT
            poses.append((t, seg["name"], x, y, yaw, v))
    return poses


def clearance_at(occ, inside, origin, res, x: float, y: float, search_m: float = 1.5):
    """(distance to the nearest occupied cell, is there drivable floor under the centre).

    Brute force in a local window: exact enough, and needs no scipy.
    """
    nx, ny = occ.shape
    ci = int((x - origin[0]) / res)
    cj = int((y - origin[1]) / res)
    if not (0 <= ci < nx and 0 <= cj < ny):
        return 0.0, False
    on_floor = bool(inside[ci, cj])
    r = int(math.ceil(search_m / res))
    i0, i1 = max(0, ci - r), min(nx, ci + r + 1)
    j0, j1 = max(0, cj - r), min(ny, cj + r + 1)
    hits = np.argwhere(occ[i0:i1, j0:j1])
    if hits.size == 0:
        return search_m, on_floor
    dx = (hits[:, 0] + i0 - ci) * res
    dy = (hits[:, 1] + j0 - cj) * res
    return float(np.sqrt(dx * dx + dy * dy).min()), on_floor


def _require_real_grid(occ, inside, a, raster=None) -> None:
    """Refuse to judge a path against a grid that saw nothing.

    An empty grid makes every pose look 0.00 m from an obstacle AND off the floor, which reads
    like a terrible route when in fact the world was never loaded. That happens when the mesh
    files do not resolve: `src/husarion_gz_worlds` is a submodule, and 20 of the office's meshes
    come from the Fuel cache, which only exists on the machine that converted the world. A verdict
    from such a grid is worthless in both directions, so it is an error, never a rejection.
    """
    tri = None if raster is None else raster.get("triangles_rasterised", 0)
    if int(inside.sum()) > 0 and (tri is None or tri > 0) and bool(occ.any()):
        return
    print("\nERROR: the occupancy grid is empty -- the world geometry did not load, so this check"
          "\n       cannot say anything about the route.", file=sys.stderr)
    if tri is not None:
        print(f"       triangles rasterised: {tri}", file=sys.stderr)
    print(f"       floor cells: {int(inside.sum())}\n"
          f"       meshes: {a.models_dir}\n"
          f"       fuel cache: {a.fuel_cache}\n"
          "       Check that the submodule is initialised (git submodule update --init"
          " src/husarion_gz_worlds)\n"
          "       and that the Fuel cache is present. On a machine without them, validate the"
          " route where\n"
          "       the world lives and pass the resulting json to hydra_run.sh with --path-check.",
          file=sys.stderr)
    if a.cache and os.path.isfile(a.cache):
        print(f"       (delete the poisoned cache first: {a.cache})", file=sys.stderr)
    raise SystemExit(2)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--scene", default="office")
    ap.add_argument("--layout", default="corridor")
    ap.add_argument("--segments", default=DEFAULT_ROUTE,
                    help="route JSON (a list of {name,speed,steer,dur}, or an object with "
                         "`segments`). Default: config/routes/office_corridor_loop.json")
    ap.add_argument("--sdf", default=DEFAULT_SDF)
    ap.add_argument("--models-dir", default=DEFAULT_MODELS)
    ap.add_argument("--fuel-cache", default=DEFAULT_FUEL)
    ap.add_argument("--grid", type=float, default=0.05)
    ap.add_argument("--margin", type=float, default=0.25,
                    help="fail if the footprint comes closer than this to an obstacle [m]")
    ap.add_argument("--stride", type=int, default=5, help="check every Nth integrated pose")
    ap.add_argument("--json", default="", help="write the result here")
    ap.add_argument("--cache", default="", help="npz to hold the occupancy grid; built once, "
                                                "then reused (iterating on a route is cheap)")
    a = ap.parse_args()

    segments, meta = load_route(a.segments)
    if not segments:
        raise SystemExit(f"{a.segments}: no segments")
    print(f"route {meta.get('name', os.path.basename(a.segments))} ({len(segments)} segments)")

    x0, y0, yaw0 = start_pose(a.scene, a.layout)
    print(f"scene {a.scene}/{a.layout}: start ({x0:.3f}, {y0:.3f}) "
          f"yaw {math.degrees(yaw0):.1f} deg")
    poses = integrate(segments, x0, y0, yaw0)
    total = sum(math.dist(poses[i - 1][2:4], poses[i][2:4]) for i in range(1, len(poses)))
    print(f"{len(segments)} segments, {poses[-1][0]:.1f} s, path length {total:.2f} m")

    if a.cache and os.path.isfile(a.cache):
        z = np.load(a.cache)
        occ, origin, res, inside = z["occ"], z["origin"], float(z["res"]), z["inside"]
        if abs(res - a.grid) > 1e-9:
            raise SystemExit(f"{a.cache} was built at grid {res}, not {a.grid}: delete it")
        print(f"occupancy grid from cache {a.cache}: {occ.shape}")
        _require_real_grid(occ, inside, a)
    else:
        print(f"building the occupancy grid from {os.path.basename(a.sdf)} ...")
        models, _meta = parse_world(a.sdf)
        occ, origin, res, floor, raster = occupancy(models, a.models_dir, a.fuel_cache, a.grid)
        inside = np.isfinite(floor) & (np.abs(np.where(np.isfinite(floor), floor, 1e3)) < 0.05)
        print(f"  grid {occ.shape}, {raster['triangles_rasterised']} triangles, "
              f"{int(inside.sum())} cells with drivable floor")
        _require_real_grid(occ, inside, a, raster)
        if a.cache:
            np.savez_compressed(a.cache, occ=occ, origin=origin, res=res, inside=inside)
            print(f"  cached in {a.cache}")

    worst = {"clear": 1e9}
    off_floor = []
    per_seg: dict = {}
    for t, name, x, y, yaw, _v in poses[::a.stride]:
        clear, on_floor = clearance_at(occ, inside, origin, res, x, y)
        s = per_seg.setdefault(name, {"clear": 1e9, "at": None})
        if clear < s["clear"]:
            s.update(clear=clear, at=(round(x, 3), round(y, 3), round(t, 1)))
        if clear < worst["clear"]:
            worst = {"clear": clear, "t": round(t, 1), "name": name, "x": round(x, 3),
                     "y": round(y, 3), "yaw_deg": round(math.degrees(yaw), 1)}
        if not on_floor:
            off_floor.append((round(t, 1), name, round(x, 3), round(y, 3)))

    need = FOOTPRINT_R + a.margin
    print("\nper segment, closest approach to collision geometry:")
    for name, s in per_seg.items():
        print(f"  {name:15} {s['clear']:5.2f} m  at {s['at']}"
              f"{'  <-- TIGHT' if s['clear'] < need else ''}")

    print(f"\nworst: {worst['clear']:.2f} m at t={worst.get('t')}s in '{worst.get('name')}' "
          f"({worst.get('x')}, {worst.get('y')})")
    print(f"needed: footprint {FOOTPRINT_R:.2f} + margin {a.margin:.2f} = {need:.2f} m")
    print(f"end pose: ({poses[-1][2]:.3f}, {poses[-1][3]:.3f}) "
          f"yaw {math.degrees(poses[-1][4]):.1f} deg; "
          f"{math.dist(poses[-1][2:4], (x0, y0)):.2f} m from the start")

    ok = worst["clear"] >= need and not off_floor
    if off_floor:
        print(f"\nNO DRIVABLE FLOOR under {len(off_floor)} checked poses, first {off_floor[0]}")
    print(f"\n{'PATH OK' if ok else 'PATH REJECTED'}")

    if a.json:
        with open(a.json, "w", encoding="utf-8") as f:
            json.dump({"scene": a.scene, "layout": a.layout, "segments": segments,
                       "start": [x0, y0, math.degrees(yaw0)],
                       "end": [poses[-1][2], poses[-1][3], math.degrees(poses[-1][4])],
                       "path_length_m": total, "duration_s": poses[-1][0],
                       "worst": worst, "per_segment": per_seg,
                       "footprint_r_m": FOOTPRINT_R, "margin_m": a.margin,
                       "off_floor": off_floor[:20], "ok": ok}, f, indent=2)
        print(f"wrote {a.json}")
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
