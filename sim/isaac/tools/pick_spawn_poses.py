#!/usr/bin/env python3
"""Pick the office world's car spawn poses from the SDF, with no simulator involved.

    sim/isaac/tools/pick_spawn_poses.py [--sdf PATH] [--grid 0.05] [--json FILE]

The HIL office world offers three start poses by argument (``--layout corridor|door|open_space``,
config/scenes/office.yaml). Choosing them by eye in the Isaac GUI is neither reproducible nor
possible on a machine without Isaac, so they are computed from the same SDF the world is built
from: every COLLISION geometry is projected to an occupancy grid at the lidar's height
(base_link + 0.19 m, the UST-10LX plane), and the poses are read off that grid.

  corridor    narrow free space (clearance 0.5-1.5 m) with a long clear run ahead
  door        2.5 m in front of a Door model, facing it
  open_space  the cell with the largest clearance

The projection uses each geometry's axis-aligned bounding box, which OVER-estimates occupancy for
round and diagonal furniture: a pose this tool calls free is free, a pose it rejects may not be.
Mesh bounds come from the mesh files themselves (sdf_to_usd.mesh_info), so the numbers are in the
same metres as Gazebo and the generated USD.
"""
from __future__ import annotations

import argparse
import json
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from sdf_to_usd import (DEFAULT_FUEL, DEFAULT_MODELS, DEFAULT_SDF, mesh_info,  # noqa: E402
                        mesh_triangles, parse_world, resolve_uri)

LIDAR_Z = (0.05, 0.35)          # the scan plane is 0.19 m up; a 0.3 m band around it
CAR_HALF_WIDTH = 0.136          # f1tenth_params footprint
FOOTPRINT_R = 0.30              # conservative circle around base_link (0.405 m nose, 0.136 half width)


def occupancy(models, models_dir: str, fuel_cache: str, res: float):
    """Occupancy grid of the collision geometry that intersects the lidar plane.

    Mesh geometry is rasterised TRIANGLE by triangle (each triangle's own bounding box), because
    the office's walls and floors are single building-wide meshes; primitive geometry uses its
    oriented box. Only what crosses the scan band counts.
    """
    boxes = []        # (lo, hi, model) per rasterised piece, for the report
    tri_boxes: list = []
    floor_boxes: list = []
    cache: dict = {}
    tri_cache: dict = {}
    for m in models:
        for link in m.links:
            for g in link.geoms:
                if g.kind != "collision":
                    continue
                tf = m.pose @ link.pose @ g.pose
                if g.gtype == "mesh":
                    path = None
                    if g.uri not in tri_cache:
                        path, _ = resolve_uri(g.uri, models_dir, fuel_cache)
                        tri_cache[g.uri] = mesh_triangles(path) if os.path.isfile(path) else None
                    tris = tri_cache[g.uri]
                    if tris is not None:
                        v = tris * np.array(g.scale, dtype=float)          # (N, 3, 3)
                        w = (tf[:3, :3] @ v.reshape(-1, 3).T).T + tf[:3, 3]
                        w = w.reshape(-1, 3, 3)
                        lo_t, hi_t = w.min(axis=1), w.max(axis=1)
                        keep = (hi_t[:, 2] >= LIDAR_Z[0]) & (lo_t[:, 2] <= LIDAR_Z[1])
                        if keep.any():
                            tri_boxes.append((lo_t[keep], hi_t[keep], m.name))
                            boxes.append((lo_t[keep].min(axis=0), hi_t[keep].max(axis=0), m.name))
                        low = (hi_t[:, 2] < LIDAR_Z[0]) & (hi_t[:, 2] > -1.0)    # drivable surface
                        if low.any():
                            floor_boxes.append((lo_t[low], hi_t[low]))
                        continue
                    if g.uri not in cache:
                        p2, _ = resolve_uri(g.uri, models_dir, fuel_cache)
                        cache[g.uri] = mesh_info(p2) if os.path.isfile(p2) else {}
                    info = cache[g.uri]
                    if not info.get("bbox_min"):
                        continue
                    mn = np.array(info["bbox_min"]) * np.array(g.scale, dtype=float)
                    mx = np.array(info["bbox_max"]) * np.array(g.scale, dtype=float)
                elif g.gtype == "box":
                    s = np.array(g.size, dtype=float) / 2.0
                    mn, mx = -s, s
                elif g.gtype == "plane":
                    s = np.array([g.size[0] if g.size else 20.0, g.size[1] if len(g.size) > 1 else 20.0]) / 2.0
                    mn, mx = np.array([-s[0], -s[1], -0.01]), np.array([s[0], s[1], 0.01])
                elif g.gtype in ("cylinder", "sphere"):
                    r = float(g.size[0]) if g.size else 0.5
                    h = float(g.size[1]) if g.gtype == "cylinder" and len(g.size) > 1 else 2 * r
                    mn, mx = np.array([-r, -r, -h / 2]), np.array([r, r, h / 2])
                else:
                    continue
                corners = np.array([[x, y, z] for x in (mn[0], mx[0]) for y in (mn[1], mx[1])
                                    for z in (mn[2], mx[2])])
                w = (tf[:3, :3] @ corners.T).T + tf[:3, 3]
                lo, hi = w.min(axis=0), w.max(axis=0)
                if hi[2] < LIDAR_Z[0] or lo[2] > LIDAR_Z[1]:
                    continue                     # below the scan plane (floor) or above it
                boxes.append((lo, hi, m.name))
    if not boxes:
        raise SystemExit("no collision geometry intersects the lidar plane")
    all_lo = np.min([b[0] for b in boxes], axis=0)
    all_hi = np.max([b[1] for b in boxes], axis=0)
    nx = int(math.ceil((all_hi[0] - all_lo[0]) / res)) + 2
    ny = int(math.ceil((all_hi[1] - all_lo[1]) / res)) + 2
    occ = np.zeros((nx, ny), dtype=bool)

    def mark(lo, hi) -> None:
        i0 = max(0, int((lo[0] - all_lo[0]) / res))
        i1 = min(nx - 1, int(math.ceil((hi[0] - all_lo[0]) / res)))
        j0 = max(0, int((lo[1] - all_lo[1]) / res))
        j1 = min(ny - 1, int(math.ceil((hi[1] - all_lo[1]) / res)))
        if i1 >= i0 and j1 >= j0:
            occ[i0:i1 + 1, j0:j1 + 1] = True

    # height of the drivable surface: the highest geometry BELOW the scan band per cell
    floor = np.full((nx, ny), -np.inf)

    def mark_floor(lo, hi) -> None:
        i0 = max(0, int((lo[0] - all_lo[0]) / res))
        i1 = min(nx - 1, int(math.ceil((hi[0] - all_lo[0]) / res)))
        j0 = max(0, int((lo[1] - all_lo[1]) / res))
        j1 = min(ny - 1, int(math.ceil((hi[1] - all_lo[1]) / res)))
        if i1 >= i0 and j1 >= j0:
            np.maximum(floor[i0:i1 + 1, j0:j1 + 1], hi[2], out=floor[i0:i1 + 1, j0:j1 + 1])

    for lo_t, hi_t in floor_boxes:
        for lo, hi in zip(lo_t, hi_t):
            mark_floor(lo, hi)
    n_tri = 0
    for lo_t, hi_t, _name in tri_boxes:
        for lo, hi in zip(lo_t, hi_t):
            mark(lo, hi)
            n_tri += 1
    for lo, hi, name in boxes:
        if any(name == b[2] for b in tri_boxes):
            continue                      # already rasterised per triangle
        mark(lo, hi)
    return occ, all_lo[:2], res, floor, {"pieces": len(boxes), "triangles_rasterised": n_tri,
                                         "floor_cells": int(np.isfinite(floor).sum())}


def clearance_map(occ: np.ndarray, res: float) -> np.ndarray:
    from scipy import ndimage

    return ndimage.distance_transform_edt(~occ) * res


def free_run(occ: np.ndarray, i: int, j: int, di: int, dj: int, res: float, limit: float = 30.0) -> float:
    n = 0
    while n * res < limit:
        i += di
        j += dj
        if not (0 <= i < occ.shape[0] and 0 <= j < occ.shape[1]) or occ[i, j]:
            break
        n += 1
    return n * res


def world_xy(origin, res, i, j):
    return float(origin[0] + (i + 0.5) * res), float(origin[1] + (j + 0.5) * res)


def doorways(occ: np.ndarray, clear: np.ndarray, origin, res: float, floor_at,
             width_range=(0.7, 1.7), min_passage: float = 1.5, standoff: float = 2.0) -> list[dict]:
    """Find DOORWAYS: a gap in the structure with a jamb on each side and free space through it.

    This is the geometry the door detector works on (f1tenth_costmap: two jambs, a door-wide gap
    between them), and in this office it is not the ``Door`` model: that leaf is CLOSED and a
    flight of stairs stands 0.6 m in front of it (measured: a Stairs collision box at
    x 10.72..10.93, y -7.41..-6.41). The doorways that a car can actually drive at are the
    openings left between the wall meshes.
    """
    out: dict = {}
    nx, ny = occ.shape
    free = (~occ) & (clear > 0.0)
    for i in range(1, nx - 1):
        for j in range(1, ny - 1):
            if not free[i, j]:
                continue
            for axis in (0, 1):
                # jambs across `axis`, passage along the other axis
                d_pos = free_run(occ, i, j, *(1, 0) if axis == 0 else (0, 1), res=res, limit=1.2)
                d_neg = free_run(occ, i, j, *(-1, 0) if axis == 0 else (0, -1), res=res, limit=1.2)
                width = d_pos + d_neg + res
                if not (width_range[0] <= width <= width_range[1]):
                    continue
                if d_pos >= 1.2 or d_neg >= 1.2:          # nothing found within the limit: not a gap
                    continue
                through = (0, 1) if axis == 0 else (1, 0)
                f1 = free_run(occ, i, j, through[0], through[1], res=res, limit=6.0)
                f2 = free_run(occ, i, j, -through[0], -through[1], res=res, limit=6.0)
                if min(f1, f2) < min_passage:
                    continue
                # centre of the gap
                ci = i + (d_pos - d_neg) / 2.0 / res * (1 if axis == 0 else 0)
                cj = j + (d_pos - d_neg) / 2.0 / res * (0 if axis == 0 else 1)
                x, y = world_xy(origin, res, int(round(ci)), int(round(cj)))
                key = (round(x * 2) / 2, round(y * 2) / 2, axis)
                cand = {"x": round(x, 3), "y": round(y, 3), "width_m": round(width, 2),
                        "passage_axis": "y" if axis == 0 else "x",
                        "free_through_m": [round(f1, 2), round(f2, 2)],
                        "floor_z_m": floor_at(x, y)}
                for sign, name in ((1.0, "a"), (-1.0, "b")):
                    dx, dy = (through[0] * sign * standoff * 1.0, through[1] * sign * standoff * 1.0)
                    sx, sy = x + dx * 1.0, y + dy * 1.0
                    i2, j2 = int((sx - origin[0]) / res), int((sy - origin[1]) / res)
                    ok = 0 <= i2 < nx and 0 <= j2 < ny
                    cand[f"approach_{name}"] = {
                        "x": round(sx, 3), "y": round(sy, 3),
                        "yaw_deg": round(math.degrees(math.atan2(-dy, -dx)), 1),
                        "clearance_m": round(float(clear[i2, j2]), 3) if ok else 0.0,
                        "floor_z_m": floor_at(sx, sy) if ok else None}
                prev = out.get(key)
                if prev is None or max(cand["free_through_m"]) > max(prev["free_through_m"]):
                    out[key] = cand
    best = sorted(out.values(), key=lambda d: -(min(d["free_through_m"])))
    usable = [d for d in best
              if d["floor_z_m"] is not None
              and any((d[f"approach_{n}"]["floor_z_m"] is not None
                       and d[f"approach_{n}"]["clearance_m"] > FOOTPRINT_R) for n in ("a", "b"))]
    for d in usable:
        d["use_approach"] = "a" if (d["approach_a"]["floor_z_m"] is not None
                                    and d["approach_a"]["clearance_m"] > FOOTPRINT_R) else "b"
    return usable


def door_poses(models, clear, origin, res, floor_at, standoff: float = 2.5) -> list[dict]:
    out = []
    for m in models:
        if "door" not in m.name.lower():
            continue
        p = m.pose[:3, 3]
        yaw = math.atan2(m.pose[1, 0], m.pose[0, 0])
        for sign, label in ((1.0, "front"), (-1.0, "back")):
            # a door's local x is across the leaf, so its normal is local y
            nx, ny = -math.sin(yaw) * sign, math.cos(yaw) * sign
            x, y = float(p[0] + nx * standoff), float(p[1] + ny * standoff)
            i, j = int((x - origin[0]) / res), int((y - origin[1]) / res)
            if not (0 <= i < clear.shape[0] and 0 <= j < clear.shape[1]):
                continue
            out.append({"model": m.name, "side": label, "x": round(x, 3), "y": round(y, 3),
                        "floor_z_m": floor_at(x, y),
                        "yaw_deg": round(math.degrees(math.atan2(-ny, -nx)), 1),
                        "clearance_m": round(float(clear[i, j]), 3),
                        "usable": bool(clear[i, j] > FOOTPRINT_R),
                        "door_xy": [round(float(p[0]), 3), round(float(p[1]), 3)],
                        "door_yaw_deg": round(math.degrees(yaw), 1)})
    return sorted(out, key=lambda d: -d["clearance_m"])


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--sdf", default=DEFAULT_SDF)
    ap.add_argument("--models-dir", default=DEFAULT_MODELS)
    ap.add_argument("--fuel-cache", default=DEFAULT_FUEL)
    ap.add_argument("--grid", type=float, default=0.05, help="grid resolution [m]")
    ap.add_argument("--json", default="", help="write the result here")
    args = ap.parse_args()
    models, meta = parse_world(args.sdf)
    occ, origin, res, floor, raster = occupancy(models, args.models_dir, args.fuel_cache, args.grid)

    def floor_at(x: float, y: float):
        i, j = int((x - origin[0]) / res), int((y - origin[1]) / res)
        if not (0 <= i < floor.shape[0] and 0 <= j < floor.shape[1]):
            return None
        z = float(floor[i, j])
        return None if not math.isfinite(z) else round(z, 3)
    clear = clearance_map(occ, res)
    # A cell with no drivable surface under it is not a candidate however free it looks: the
    # largest "clearance" in this world is the empty space OUTSIDE the building (measured: the
    # unmasked maximum was (7.56, 3.00), 0.11 m past the outer wall, where the car would fall).
    inside = np.isfinite(floor) & (np.abs(np.where(np.isfinite(floor), floor, 1e3)) < 0.05)
    clear = np.where(inside, clear, 0.0)
    free_area = float(((~occ) & inside).sum()) * res * res

    # open space: the largest free circle
    i, j = np.unravel_index(int(np.argmax(clear)), clear.shape)
    x, y = world_xy(origin, res, i, j)
    open_space = {"x": round(x, 3), "y": round(y, 3), "yaw_deg": 0.0,
                  "clearance_m": round(float(clear[i, j]), 3), "floor_z_m": floor_at(x, y)}

    # corridor: narrow (0.5-1.5 m clearance), long clear run along one axis, car fits
    best = None
    cand = np.argwhere((clear > max(0.5, FOOTPRINT_R)) & (clear < 1.5))
    for i, j in cand[:: max(1, len(cand) // 4000)]:
        for (di, dj), yaw in (((1, 0), 0.0), ((-1, 0), 180.0), ((0, 1), 90.0), ((0, -1), -90.0)):
            run = free_run(occ, int(i), int(j), di, dj, res)
            side = min(free_run(occ, int(i), int(j), -dj, di, res, 5.0),
                       free_run(occ, int(i), int(j), dj, -di, res, 5.0))
            score = run + 2.0 * min(side, 1.5)
            if run >= 5.0 and side >= CAR_HALF_WIDTH + 0.1 and (best is None or score > best["score"]):
                x, y = world_xy(origin, res, int(i), int(j))
                best = {"x": round(x, 3), "y": round(y, 3), "yaw_deg": yaw, "score": round(score, 2),
                        "floor_z_m": floor_at(x, y),
                        "clear_ahead_m": round(run, 2), "clear_sides_m": round(side, 2),
                        "clearance_m": round(float(clear[i, j]), 3)}
    doors = door_poses(models, clear, origin, res, floor_at)
    usable_doors = [d for d in doors if d["usable"]]
    ways = doorways(occ, clear, origin, res, floor_at)
    out = {"meta": meta, "grid": {"res": res, "origin_xy": [round(float(v), 3) for v in origin],
                                  "cells": list(occ.shape), "free_area_m2": round(free_area, 1),
                                  "lidar_band_m": list(LIDAR_Z), "rasterised": raster},
           "open_space": open_space, "corridor": best,
           "door_model_poses": usable_doors[:4],
           "doorways": ways[:10],
           "door": (lambda w: {**w[f"approach_{w['use_approach']}"], "doorway": [w["x"], w["y"]],
                               "doorway_width_m": w["width_m"]})(ways[0]) if ways else None,
           "doors": doors[:8],
           "door_models": sorted({m.name for m in models if "door" in m.name.lower()}),
           "glass_models": sorted({m.name for m in models if "glass" in m.name.lower()})}
    print(json.dumps(out, indent=1))
    if args.json:
        with open(args.json, "w") as f:
            json.dump(out, f, indent=1)
    return 0


if __name__ == "__main__":
    sys.exit(main())
