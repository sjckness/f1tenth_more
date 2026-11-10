"""Inspect asset-server USDs: folder listings, world bbox, the largest child prims, skeleton/animation.

    /isaac-sim/python.sh tools/asset_probe.py [--list FOLDER ...] [--usd ASSET ...]
Asset paths are relative to the NVIDIA asset root (e.g. /Isaac/People/Characters/...).
"""
import argparse

from isaacsim import SimulationApp

app = SimulationApp({"headless": True})
import omni.client  # noqa: E402
import omni.usd  # noqa: E402
from isaacsim.core.utils.stage import add_reference_to_stage  # noqa: E402
from isaacsim.storage.native import get_assets_root_path  # noqa: E402
from pxr import Usd, UsdGeom, UsdSkel  # noqa: E402

ap = argparse.ArgumentParser()
ap.add_argument("--list", nargs="*", default=[])
ap.add_argument("--usd", nargs="*", default=[])
ap.add_argument("--depth", type=int, default=3)
a = ap.parse_args()
root = get_assets_root_path()
for sub in a.list:
    res, entries = omni.client.list(f"{root}{sub}")
    print("LIST", sub, res, flush=True)
    for e in entries:
        print("  ", e.relative_path, e.size, flush=True)

stage = omni.usd.get_context().get_stage()
UsdGeom.Xform.Define(stage, "/World")
bbc = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_, UsdGeom.Tokens.render])


def size(prim):
    r = bbc.ComputeWorldBound(prim).ComputeAlignedRange()
    if r.IsEmpty():
        return None, None
    mn, mx = r.GetMin(), r.GetMax()
    return [round(mx[i] - mn[i], 3) for i in range(3)], [round(v, 3) for v in mn] + [round(v, 3) for v in mx]


for i, usd in enumerate(a.usd):
    path = f"/World/a{i}"
    try:
        add_reference_to_stage(f"{root}{usd}", path)
    except Exception as exc:  # noqa: BLE001
        print("USD", usd, "FAILED", exc, flush=True)
        continue
    for _ in range(20):
        app.update()
    bbc.Clear()
    prim = stage.GetPrimAtPath(path)
    sz, rng = size(prim)
    print("USD", usd, "size xyz", sz, "range", rng, "metersPerUnit", UsdGeom.GetStageMetersPerUnit(stage), flush=True)
    for p in Usd.PrimRange(prim):
        d = str(p.GetPath()).count("/") - path.count("/")
        if d > a.depth:
            continue
        sz, _ = size(p)
        extra = ""
        if p.IsA(UsdSkel.Skeleton):
            extra = f" joints={len(UsdSkel.Skeleton(p).GetJointsAttr().Get() or [])}"
        if p.IsA(UsdSkel.Root):
            extra += " SkelRoot"
        if p.HasAPI(UsdSkel.BindingAPI):
            b = UsdSkel.BindingAPI(p)
            anim = b.GetAnimationSourceRel().GetTargets()
            extra += f" anim={list(map(str, anim))}" if anim else ""
        print("   " + "  " * d + f"{p.GetName()} [{p.GetTypeName()}] {sz}{extra}", flush=True)
    stage.RemovePrim(path)
app.close()
