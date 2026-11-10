"""Posed footprint of a people asset for several arm angles (no rendering).
    /isaac-sim/python.sh tools/person_extent.py <asset> <deg> [<deg> ...]
"""
import functools
import os
import sys

from isaacsim import SimulationApp

app = SimulationApp({"headless": True})
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import omni.usd  # noqa: E402
from isaacsim.core.utils.stage import add_reference_to_stage  # noqa: E402
from isaacsim.storage.native import get_assets_root_path  # noqa: E402
from pxr import UsdGeom  # noqa: E402

from sim.person_pose import pose_arms_down, skinned_extent  # noqa: E402

print = functools.partial(print, flush=True)  # noqa: A001
stage = omni.usd.get_context().get_stage()
UsdGeom.Xform.Define(stage, "/World")
for k, deg in enumerate(sys.argv[2:]):
    path = f"/World/p{k}"
    UsdGeom.Xform.Define(stage, path)
    add_reference_to_stage(f"{get_assets_root_path()}{sys.argv[1]}", f"{path}/visual")
    for _ in range(5):
        app.update()
    if float(deg) >= 0:
        pose_arms_down(stage, f"{path}/visual", float(deg))
    mn, mx = skinned_extent(stage, f"{path}/visual", path)
    sz = mx - mn
    print(f"EXTENT deg={deg} size {sz[0]:.3f} x {sz[1]:.3f} x {sz[2]:.3f}  min {mn.round(3)} max {mx.round(3)}")
app.close()
