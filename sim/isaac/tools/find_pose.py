"""Find car/person start poses in an NVIDIA environment: open floor, a WALL straight ahead at 2-4 m,
side clearance, and a free spot for the person ahead-left. Uses PhysX horizontal raycasts at lidar height.

    /isaac-sim/python.sh tools/find_pose.py <env_usd> <out_json> x0 x1 y0 y1 [step]
"""
from __future__ import annotations

import json
import math
import sys

from isaacsim import SimulationApp

app = SimulationApp({"headless": True})

import numpy as np  # noqa: E402
import omni.usd  # noqa: E402
from isaacsim.core.api import World  # noqa: E402
from isaacsim.core.utils.stage import add_reference_to_stage  # noqa: E402
from isaacsim.storage.native import get_assets_root_path  # noqa: E402
from omni.physx import get_physx_scene_query_interface  # noqa: E402
from pxr import Usd, UsdGeom, UsdPhysics  # noqa: E402

USD, OUT = sys.argv[1], sys.argv[2]
X0, X1, Y0, Y1 = map(float, sys.argv[3:7])
STEP = float(sys.argv[7]) if len(sys.argv) > 7 else 0.5
H = 0.19


def main() -> None:
    world = World(stage_units_in_meters=1.0)
    stage = omni.usd.get_context().get_stage()
    add_reference_to_stage(f"{get_assets_root_path()}{USD}", "/World/env")
    for _ in range(30):
        app.update()
    added = 0
    for prim in Usd.PrimRange(stage.GetPrimAtPath("/World/env")):
        if prim.IsA(UsdGeom.Mesh) and not prim.HasAPI(UsdPhysics.CollisionAPI):
            UsdPhysics.CollisionAPI.Apply(prim)
            UsdPhysics.MeshCollisionAPI.Apply(prim).CreateApproximationAttr("none")
            added += 1
    world.reset()
    for _ in range(5):
        world.step(render=False)
    sq = get_physx_scene_query_interface()

    def ray(x, y, ang, z=H, maxd=10.0):
        h = sq.raycast_closest((x, y, z), (math.cos(ang), math.sin(ang), 0.0), maxd, True)   # both sides: walls are single-sided meshes
        return (h["distance"], h["collision"]) if h["hit"] else (maxd, "")

    def floor_ok(x, y):
        """A real office floor mesh (not the invisible GroundPlane) right below, nothing above it up to 0.5 m."""
        h = sq.raycast_closest((x, y, 0.5), (0.0, 0.0, -1.0), 1.0, True)
        name = h["collision"].lower() if h["hit"] else ""
        return (h["hit"] and abs(h["position"][2]) < 0.05 and h["distance"] > 0.44
                and ("floor" in name or "carpet" in name) and "groundplane" not in name)

    cands = []
    for x in np.arange(X0, X1, STEP):
        for y in np.arange(Y0, Y1, STEP):
            x, y = float(x), float(y)
            if not floor_ok(x, y):
                continue
            ring = [ray(x, y, a)[0] for a in np.linspace(0, 2 * math.pi, 24, endpoint=False)]
            if min(ring) < 0.8:
                continue
            # indoors: a ceiling mesh above (the floor check already excludes the outdoor GroundPlane)
            up = sq.raycast_closest((x, y, 1.0), (0.0, 0.0, 1.0), 4.0, True)
            if not up["hit"] or "ceiling" not in up["collision"].lower():
                continue
            for yaw_deg in (0, 90, 180, 270):
                yaw = math.radians(yaw_deg)
                d, path = ray(x, y, yaw)
                if not (2.0 <= d <= 4.0 and "wall" in path.lower()):
                    continue
                # straight wall: rays at +-10 deg should hit the same kind of surface at d/cos
                dl, pl = ray(x, y, yaw + math.radians(10))
                dr, pr = ray(x, y, yaw - math.radians(10))
                flat = abs(dl * math.cos(math.radians(10)) - d) < 0.1 and abs(dr * math.cos(math.radians(10)) - d) < 0.1
                # person spot ahead-left: 1.8 m ahead, 0.8 m left
                px = x + 1.8 * math.cos(yaw) - 0.8 * math.sin(yaw)
                py = y + 1.8 * math.sin(yaw) + 0.8 * math.cos(yaw)
                pring = [ray(px, py, a, z=0.9)[0] for a in np.linspace(0, 2 * math.pi, 16, endpoint=False)]
                los = ray(x, y, math.atan2(py - y, px - x), z=0.9)[0] >= math.hypot(px - x, py - y) - 0.3
                person_ok = floor_ok(px, py) and min(pring) > 0.5 and los
                cands.append({"x": x, "y": y, "yaw_deg": yaw_deg, "wall_dist": d, "wall_prim": path, "flat": flat,
                              "person": [px, py], "person_ok": person_ok, "clearance": min(ring)})
    good = [c for c in cands if c["flat"] and c["person_ok"]]
    good.sort(key=lambda c: (-c["clearance"], abs(c["wall_dist"] - 3.0)))
    with open(OUT, "w") as f:
        json.dump({"colliders_added": added, "n_candidates": len(cands), "good": good[:30]}, f, indent=1)
    print("FIND_POSE", len(cands), len(good), json.dumps(good[:5]))


if __name__ == "__main__":
    try:
        main()
    finally:
        app.close()
