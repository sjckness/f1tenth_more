"""Collider audit of a referenced NVIDIA environment (default: the office scene), as the sim builds it.

    /isaac-sim/python.sh tools/office_colliders.py [scene] [out_json] [--set scene.KEY=VALUE ...]

Builds the scene exactly like sim/main.py (config/scenes/<scene>.yaml, scene_builder._build_environment,
including the collider rules of that YAML), then lists every mesh of the vertical structure (walls,
partitions, doors, glass, columns, large furniture) within `radius` m of the car start, instanced
or not, with: collision yes/no, approximation, size. The lidar (PhysX raycast) only sees colliders.
"""
from __future__ import annotations

import fnmatch
import json
import os
import sys

from isaacsim import SimulationApp

app = SimulationApp({"headless": True})
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import omni.usd  # noqa: E402
from isaacsim.core.api import World  # noqa: E402
from isaacsim.storage.native import get_assets_root_path  # noqa: E402
from pxr import Usd, UsdGeom, UsdPhysics  # noqa: E402

from sim.config import load_config  # noqa: E402
from sim.scene_builder import VERTICAL_CATEGORIES, WORLD, _build_environment  # noqa: E402

overrides, pos = [], []
it = iter(sys.argv[1:])
for a in it:
    if a == "--set":
        overrides.append(next(it))
    elif a.startswith("--set="):
        overrides.append(a[len("--set="):])
    else:
        pos.append(a)
scene = pos[0] if pos else "office"
out = pos[1] if len(pos) > 1 else f"/home/isaac_ws/output/sim_next/{scene}_colliders.json"
cfg = load_config(scene, overrides)
world = World(stage_units_in_meters=1.0)
stage = omni.usd.get_context().get_stage()
UsdGeom.Xform.Define(stage, WORLD)
report = _build_environment(stage, cfg, get_assets_root_path(), app)
for _ in range(10):
    app.update()
sc = cfg.scene
cx, cy = float(sc["car"]["start"]["x"]), float(sc["car"]["start"]["y"])
radius = float(sc.get("environment", {}).get("audit_radius", 15.0))
bbc = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_, UsdGeom.Tokens.render])
rows, other_without = [], []
for prim in Usd.PrimRange(stage.GetPrimAtPath(f"{WORLD}/env"), Usd.TraverseInstanceProxies()):
    if not prim.IsA(UsdGeom.Mesh):
        continue
    name = prim.GetName().lower()
    cat = next((c for c, pats in VERTICAL_CATEGORIES.items() if any(fnmatch.fnmatch(name, p) for p in pats)), None)
    if cat is None:
        if not prim.HasAPI(UsdPhysics.CollisionAPI):
            r_ = bbc.ComputeWorldBound(prim).ComputeAlignedRange()
            other_without.append({"path": str(prim.GetPath()), "size": None if r_.IsEmpty() else
                                  [round(r_.GetMax()[i] - r_.GetMin()[i], 2) for i in range(3)]})
        continue
    rng = bbc.ComputeWorldBound(prim).ComputeAlignedRange()
    if rng.IsEmpty():
        continue
    mn, mx = rng.GetMin(), rng.GetMax()
    # distance from the car start to the mesh's footprint box
    dx = max(mn[0] - cx, 0.0, cx - mx[0])
    dy = max(mn[1] - cy, 0.0, cy - mx[1])
    if (dx * dx + dy * dy) ** 0.5 > radius:
        continue
    approx = None
    if prim.HasAPI(UsdPhysics.MeshCollisionAPI):
        approx = UsdPhysics.MeshCollisionAPI(prim).GetApproximationAttr().Get()
    rows.append({"path": str(prim.GetPath()), "category": cat, "instance_proxy": prim.IsInstanceProxy(),
                 "collision": prim.HasAPI(UsdPhysics.CollisionAPI), "approximation": approx,
                 "size": [round(mx[i] - mn[i], 3) for i in range(3)],
                 "z": [round(mn[2], 2), round(mx[2], 2)],
                 "dist_from_start_m": round((dx * dx + dy * dy) ** 0.5, 2)})
summary: dict = {}
for r in rows:
    s = summary.setdefault(r["category"], {"meshes": 0, "with_collision": 0, "without": 0})
    s["meshes"] += 1
    s["with_collision" if r["collision"] else "without"] += 1
res = {"scene": scene, "overrides": overrides, "car_start": [cx, cy], "radius_m": radius,
       "build_report": report, "summary": summary, "other_meshes_without_collision": other_without, "meshes": rows}
with open(out, "w") as fh:
    json.dump(res, fh, indent=1, default=str)
print("OTHER_NOCOLL", json.dumps(other_without), flush=True)
print("AUDIT", json.dumps({"summary": summary, "build": {k: report.get(k) for k in
                                                          ("colliders_added", "vertical_colliders_added")}},
                          default=str), flush=True)
for r in sorted(rows, key=lambda r: r["dist_from_start_m"]):
    if not r["collision"]:
        print("NOCOLL", r["category"], r["path"], r["size"], r["dist_from_start_m"], flush=True)
app.close()
