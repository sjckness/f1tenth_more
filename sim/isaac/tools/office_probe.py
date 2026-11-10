"""Probe an NVIDIA environment (default Office): units, bbox, labels, colliders and a top-down
height map from PhysX raycasts, to choose car/person poses.

    /isaac-sim/python.sh tools/office_probe.py [/Isaac/Environments/Office/office.usd] [out_prefix]
Writes <out_prefix>.json and <out_prefix>_heightmap.png.
"""
from __future__ import annotations

import json
import logging
import sys

from isaacsim import SimulationApp

app = SimulationApp({"headless": True})

import numpy as np  # noqa: E402
import omni.usd  # noqa: E402
from isaacsim.core.api import World  # noqa: E402
from isaacsim.core.utils.semantics import get_labels, get_semantics  # noqa: E402
from isaacsim.core.utils.stage import add_reference_to_stage  # noqa: E402
from isaacsim.storage.native import get_assets_root_path  # noqa: E402
from omni.physx import get_physx_scene_query_interface  # noqa: E402
from pxr import Usd, UsdGeom, UsdPhysics  # noqa: E402

logging.basicConfig(level=logging.INFO)
log = logging.getLogger("probe")
USD = sys.argv[1] if len(sys.argv) > 1 else "/Isaac/Environments/Office/office.usd"
OUT = sys.argv[2] if len(sys.argv) > 2 else "/home/isaac_ws/output/office_probe"


def main() -> None:
    root = get_assets_root_path()
    world = World(stage_units_in_meters=1.0)
    stage = omni.usd.get_context().get_stage()
    add_reference_to_stage(f"{root}{USD}", "/World/env")
    for _ in range(30):
        app.update()
    env = stage.GetPrimAtPath("/World/env")
    info: dict = {"usd": USD, "meters_per_unit_env_layer": None}
    ref_stage = Usd.Stage.Open(f"{root}{USD}")
    info["meters_per_unit_env_layer"] = UsdGeom.GetStageMetersPerUnit(ref_stage)
    info["up_axis_env_layer"] = UsdGeom.GetStageUpAxis(ref_stage)
    bb = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_, UsdGeom.Tokens.render])
    r = bb.ComputeWorldBound(env).ComputeAlignedRange()
    info["bbox_min"], info["bbox_max"] = list(r.GetMin()), list(r.GetMax())
    labels: dict = {}
    old_sem: dict = {}
    names: dict = {}
    n_mesh = n_coll = 0
    n_inst = 0
    for prim in Usd.PrimRange(env):
        if prim.IsInstanceable():
            prim.SetInstanceable(False)      # colliders cannot be authored on instance proxies
            n_inst += 1
    info["instanceable_prims_deinstanced"] = n_inst
    for _ in range(5):
        app.update()
    for prim in Usd.PrimRange(env):
        for _i, labs in (get_labels(prim) or {}).items():
            for lab in labs:
                labels[lab] = labels.get(lab, 0) + 1
        try:
            for _k, (_t, d) in (get_semantics(prim) or {}).items():
                old_sem[d] = old_sem.get(d, 0) + 1
        except Exception:  # noqa: BLE001
            pass
        if prim.IsA(UsdGeom.Mesh):
            n_mesh += 1
            nm = prim.GetName().lower()
            for key in ("floor", "wall", "ceiling", "door", "window", "glass", "carpet", "column", "pillar"):
                if key in nm:
                    names[key] = names.get(key, 0) + 1
            if prim.HasAPI(UsdPhysics.CollisionAPI):
                n_coll += 1
            else:
                UsdPhysics.CollisionAPI.Apply(prim)
                UsdPhysics.MeshCollisionAPI.Apply(prim).CreateApproximationAttr("none")
    info.update(meshes=n_mesh, meshes_with_collision=n_coll, labels=labels, old_semantics=old_sem,
                mesh_name_keywords=names)
    top = [str(c.GetPath()) for c in env.GetChildren()]
    info["top_children"] = top[:50]
    log.info("probe info: %s", json.dumps({k: info[k] for k in ("bbox_min", "bbox_max", "meshes",
                                                                 "meshes_with_collision", "meters_per_unit_env_layer")}))
    with open(f"{OUT}.json", "w") as f:
        json.dump(info, f, indent=1, default=str)
    world.reset()
    for _ in range(5):
        world.step(render=False)
    sq = get_physx_scene_query_interface()
    x0, y0, z0 = r.GetMin()
    x1, y1, z1 = r.GetMax()
    res = float(sys.argv[3]) if len(sys.argv) > 3 else 0.1
    # clamp to a sane window (very large bboxes come from sky/backdrop meshes)
    x0, y0 = max(x0, -26.0), max(y0, -36.0)
    x1, y1 = min(x1, 9.0), min(y1, 41.0)
    z1 = float(sys.argv[4]) if len(sys.argv) > 4 else 1.2     # slice height: start rays below the ceiling
    log.info("raycast grid %.1f..%.1f x %.1f..%.1f res %.2f", x0, x1, y0, y1, res)
    xs = np.arange(x0, x1, res)
    ys = np.arange(y0, y1, res)
    hm = np.full((len(ys), len(xs)), np.nan, dtype=np.float32)
    for j, y in enumerate(ys):
        for i, x in enumerate(xs):
            hit = sq.raycast_closest((float(x), float(y), float(z1)), (0.0, 0.0, -1.0), float(z1) + 0.5)
            if hit["hit"]:
                hm[j, i] = hit["position"][2]
    np.save(f"{OUT}_heightmap.npy", hm)
    info["heightmap"] = {"x0": float(x0), "y0": float(y0), "res": res, "shape": list(hm.shape)}
    vals = hm[np.isfinite(hm)]
    hist, edges = np.histogram(vals, bins=40)
    info["height_hist"] = {"counts": hist.tolist(), "edges": edges.tolist()}
    floor_z = float(edges[np.argmax(hist)])
    info["floor_z_mode"] = floor_z
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(figsize=(12, 12 * len(ys) / len(xs) + 1))
    im = ax.imshow(hm, origin="lower", extent=[x0, x1, y0, y1], cmap="viridis", vmin=floor_z - 0.05, vmax=floor_z + 1.25)
    fig.colorbar(im, ax=ax, label="top surface z (m)")
    ax.set_xticks(np.arange(np.floor(x0), np.ceil(x1), 1.0))
    ax.set_yticks(np.arange(np.floor(y0), np.ceil(y1), 1.0))
    ax.grid(True, lw=0.3)
    ax.set_title(f"{USD} top-down heightmap (PhysX raycasts)")
    fig.savefig(f"{OUT}_heightmap.png", dpi=90, bbox_inches="tight")
    with open(f"{OUT}.json", "w") as f:
        json.dump(info, f, indent=1, default=str)
    log.info("probe done: %s", json.dumps({k: info[k] for k in ("bbox_min", "bbox_max", "meshes", "floor_z_mode")}))


if __name__ == "__main__":
    try:
        main()
    finally:
        app.close()
