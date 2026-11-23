"""Find walls the PhysX lidar cannot see: single-sided or missing colliders, at the car's start pose.

    /isaac-sim/python.sh tools/lidar_passthrough.py <scene> <out_prefix> [--set scene.KEY=VALUE ...]

Builds the environment as the sim does (scene_builder._build_environment with the scene YAML's collider
rules), then for every real lidar beam (sensors.yaml angles, laser height from car_spec) at the start pose:
  forward ray  laser -> out to 30 m: what the lidar's raycast returns (PhysX scene query = same colliders);
  reverse ray  from 30 m back to the laser: the first surface seen from the far side.
If the reverse ray finds a surface CLOSER to the laser than the forward return, the forward ray went
through it: a collider whose faces point away from the laser (single-sided triangle mesh) - a wall the
car's lidar would miss. Writes <out_prefix>.json (counts, the meshes passed through) and <out_prefix>.png.
"""
from __future__ import annotations

import collections
import json
import math
import os
import sys

from isaacsim import SimulationApp

app = SimulationApp({"headless": True})
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import matplotlib  # noqa: E402
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
import omni.usd  # noqa: E402
from isaacsim.core.api import World  # noqa: E402
from isaacsim.storage.native import get_assets_root_path  # noqa: E402
from omni.physx import get_physx_scene_query_interface  # noqa: E402
from pxr import UsdGeom  # noqa: E402

from sim.config import get, load_config  # noqa: E402
from sim.scene_builder import WORLD, _build_environment  # noqa: E402

overrides, pos = [], []
it = iter(sys.argv[1:])
for a in it:
    if a == "--set":
        overrides.append(next(it))
    else:
        pos.append(a)
scene = pos[0] if pos else "office"
out = pos[1] if len(pos) > 1 else f"/home/isaac_ws/output/sim_next/{scene}_passthrough"
cfg = load_config(scene, overrides)
world = World(stage_units_in_meters=1.0)
stage = omni.usd.get_context().get_stage()
UsdGeom.Xform.Define(stage, WORLD)
report = _build_environment(stage, cfg, get_assets_root_path(), app)
world.reset()
for _ in range(10):
    world.step(render=False)
sq = get_physx_scene_query_interface()
st = cfg.scene["car"]["start"]
yaw = math.radians(float(st["yaw_deg"]))
lx_b, _, lz_b = get(cfg.car, "sensors.laser.xyz")
ox, oy = float(st["x"]) + lx_b * math.cos(yaw), float(st["y"]) + lx_b * math.sin(yaw)
oz = float(st.get("z", 0.0)) + lz_b
lc = cfg.sensors["lidar"]
n = int(round((float(lc["angle_max"]) - float(lc["angle_min"])) / float(lc["angle_increment"]))) + 1
angles = float(lc["angle_min"]) + np.arange(n) * float(lc["angle_increment"])
R = 30.0
fwd = np.full(n, np.inf)
rev_near = np.full(n, np.inf)      # nearest surface found from the far side (distance from the laser)
through = collections.Counter()
pts_f, pts_t = [], []
for i, a in enumerate(angles):
    d = (math.cos(yaw + a), math.sin(yaw + a), 0.0)
    h = sq.raycast_closest((ox, oy, oz), d, R)
    rf = h["distance"] if h["hit"] else np.inf
    fwd[i] = rf
    # walk back from the far end: every surface between the laser and the forward return seen from behind
    start_s = R
    for _ in range(20):
        p0 = (ox + d[0] * start_s, oy + d[1] * start_s, oz)
        hb = sq.raycast_closest(p0, (-d[0], -d[1], 0.0), start_s)
        if not hb["hit"]:
            break
        s = start_s - hb["distance"]          # distance of that surface from the laser
        if s < rf - 0.05:
            rev_near[i] = min(rev_near[i], s)
            through[hb["collision"]] += 1
            pts_t.append((ox + d[0] * s, oy + d[1] * s))
        if s <= 0.05:
            break
        start_s = s - 0.01
    if np.isfinite(rf):
        pts_f.append((ox + d[0] * rf, oy + d[1] * rf))
pass_beams = int(np.isfinite(rev_near).sum())
res = {"scene": scene, "overrides": overrides, "laser": [ox, oy, oz], "beams": n,
       "forward_finite": int(np.isfinite(fwd).sum()), "forward_below_10m": int((fwd < 10).sum()),
       "forward_below_10m_pct": round(100.0 * (fwd < 10).sum() / n, 1),
       "pass_through_beams": pass_beams, "pass_through_pct": round(100.0 * pass_beams / n, 1),
       "passed_through_colliders": through.most_common(40),
       "forward_ranges": [None if not np.isfinite(v) else round(float(v), 3) for v in fwd],
       "reverse_nearest": [None if not np.isfinite(v) else round(float(v), 3) for v in rev_near],
       "build": {k: report.get(k) for k in ("colliders_added", "vertical_colliders_added", "vertical_approximation")}}
with open(f"{out}.json", "w") as fh:
    json.dump(res, fh, indent=1, default=str)
print("PASSTHROUGH", json.dumps({k: v for k, v in res.items()
                                 if k not in ("passed_through_colliders", "forward_ranges", "reverse_nearest")},
                                default=str),
      flush=True)
for c, k in through.most_common(25):
    print("THROUGH", k, c, flush=True)
fig, ax = plt.subplots(figsize=(9, 9))
if pts_f:
    f = np.array(pts_f)
    ax.scatter(f[:, 0], f[:, 1], s=3, c="tab:blue", label="lidar return (forward raycast)")
if pts_t:
    t = np.array(pts_t)
    ax.scatter(t[:, 0], t[:, 1], s=5, c="tab:red", label="surface the beam went THROUGH (seen from behind)")
ax.plot([ox], [oy], "k^", ms=8, label="laser")
ax.set_xlim(ox - 12, ox + 12)
ax.set_ylim(oy - 12, oy + 12)
ax.set_aspect("equal")
ax.legend(loc="lower left", fontsize=8)
ax.set_title(f"{scene} {' '.join(overrides)}\n{res['forward_below_10m']}/{n} beams < 10 m "
             f"({res['forward_below_10m_pct']} %), {pass_beams} beams pass through a surface "
             f"({res['pass_through_pct']} %)", fontsize=9)
fig.savefig(f"{out}.png", dpi=110, bbox_inches="tight")
app.close()
