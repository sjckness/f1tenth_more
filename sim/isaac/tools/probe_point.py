"""List every collider hit by a vertical ray and every mesh whose bbox contains (x, y) near the floor.
    /isaac-sim/python.sh tools/probe_point.py <env_usd> x y
"""
import sys
from isaacsim import SimulationApp
app = SimulationApp({"headless": True})
import omni.usd
from isaacsim.core.api import World
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.storage.native import get_assets_root_path
from omni.physx import get_physx_scene_query_interface
from pxr import Usd, UsdGeom, UsdPhysics

usd, X, Y = sys.argv[1], float(sys.argv[2]), float(sys.argv[3])
world = World(stage_units_in_meters=1.0)
stage = omni.usd.get_context().get_stage()
add_reference_to_stage(f"{get_assets_root_path()}{usd}", "/World/env")
for _ in range(30):
    app.update()
for prim in Usd.PrimRange(stage.GetPrimAtPath("/World/env")):
    if prim.IsA(UsdGeom.Mesh) and not prim.HasAPI(UsdPhysics.CollisionAPI):
        UsdPhysics.CollisionAPI.Apply(prim)
        UsdPhysics.MeshCollisionAPI.Apply(prim).CreateApproximationAttr("none")
world.reset()
for _ in range(5):
    world.step(render=False)
sq = get_physx_scene_query_interface()
hits = []
def cb(h):
    hits.append((round(h.position[2], 4), h.collision))
    return True
sq.raycast_all((X, Y, 3.0), (0.0, 0.0, -1.0), 6.0, cb, True)
print("NHITS", len(hits), flush=True)
for h in sorted(hits, reverse=True):
    print("RAYHIT", h, flush=True)
bb = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_, UsdGeom.Tokens.render])
for prim in Usd.PrimRange(stage.GetPrimAtPath("/World/env"), Usd.TraverseInstanceProxies()):
    if prim.IsA(UsdGeom.Mesh):
        r = bb.ComputeWorldBound(prim).ComputeAlignedRange()
        mn, mx = r.GetMin(), r.GetMax()
        if mn[0] <= X <= mx[0] and mn[1] <= Y <= mx[1] and mn[2] < 0.6 and mx[2] > -0.6:
            print("MESH", prim.GetPath(), "z %.3f..%.3f" % (mn[2], mx[2]), "coll", prim.HasAPI(UsdPhysics.CollisionAPI),
                  "vis", UsdGeom.Imageable(prim).ComputeVisibility(), flush=True)
app.close()
