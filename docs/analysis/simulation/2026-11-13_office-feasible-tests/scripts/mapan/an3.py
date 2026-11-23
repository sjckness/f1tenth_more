"""Surfaces model: per-link / per-geom bboxes -> rooms (floor patches) and walls."""
import sys, os, numpy as np
REPO = r"f:/Users/Andreas Steffens/dev_ws/f1tenth_hil"
sys.path.insert(0, os.path.join(REPO, "sim/isaac/tools"))
from sdf_to_usd import parse_world, DEFAULT_SDF, DEFAULT_MODELS, DEFAULT_FUEL, resolve_uri, mesh_info, mesh_triangles

models, meta = parse_world(DEFAULT_SDF)
S = next(m for m in models if m.name == "Surfaces")
print("links:", len(S.links))
for link in S.links:
    for g in link.geoms:
        if g.kind != "collision":
            continue
        tf = S.pose @ link.pose @ g.pose
        p, _ = resolve_uri(g.uri, DEFAULT_MODELS, DEFAULT_FUEL) if g.gtype == "mesh" else ("", "")
        info = mesh_info(p) if (g.gtype == "mesh" and os.path.isfile(p)) else {}
        if info.get("bbox_min"):
            mn = np.array(info["bbox_min"])*np.array(g.scale); mx = np.array(info["bbox_max"])*np.array(g.scale)
        elif g.gtype == "box":
            s = np.array(g.size)/2; mn, mx = -s, s
        else:
            print("  ??", link.name, g.name, g.gtype, g.uri); continue
        corners = np.array([[x,y,z] for x in (mn[0],mx[0]) for y in (mn[1],mx[1]) for z in (mn[2],mx[2])])
        w = (tf[:3,:3]@corners.T).T + tf[:3,3]
        lo, hi = w.min(axis=0), w.max(axis=0)
        print(f"  {link.name:14s} {g.name:20s} x {lo[0]:7.2f}..{hi[0]:7.2f}  y {lo[1]:8.2f}..{hi[1]:8.2f}  z {lo[2]:6.2f}..{hi[2]:6.2f}  area {(hi[0]-lo[0])*(hi[1]-lo[1]):7.1f}  uri {os.path.basename(g.uri)}")
