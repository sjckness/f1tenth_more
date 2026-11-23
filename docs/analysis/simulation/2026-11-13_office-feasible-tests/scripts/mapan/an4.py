"""Wall-only mask + per-room floor rasterisation -> clean doorway inventory."""
import sys, os, json, math
import numpy as np
REPO = r"f:/Users/Andreas Steffens/dev_ws/f1tenth_hil"
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(REPO, "sim/isaac/tools"))
from sdf_to_usd import (parse_world, DEFAULT_SDF, DEFAULT_MODELS, DEFAULT_FUEL,
                        resolve_uri, mesh_triangles)

RES = 0.05
LZ = (0.05, 0.35)
models, meta = parse_world(DEFAULT_SDF)
S = next(m for m in models if m.name == "Surfaces")

# global grid from the structure bbox
OX, OY = -1.70, -24.40
NX, NY = int((27.2-OX)/RES)+1, int((3.1-OY)/RES)+1
def idx(x, y): return int((x-OX)/RES), int((y-OY)/RES)
def wxy(i, j): return OX+(i+0.5)*RES, OY+(j+0.5)*RES

wall = np.zeros((NX, NY), bool)
room = np.zeros((NX, NY), np.int16)          # 0 = no floor
rooms = {}
for link in S.links:
    for g in link.geoms:
        if g.kind != "collision" or g.gtype != "mesh":
            continue
        tf = S.pose @ link.pose @ g.pose
        p, _ = resolve_uri(g.uri, DEFAULT_MODELS, DEFAULT_FUEL)
        tris = mesh_triangles(p)
        if tris is None:
            print("missing", p); continue
        v = tris * np.array(g.scale)
        w = (tf[:3,:3] @ v.reshape(-1,3).T).T + tf[:3,3]
        w = w.reshape(-1,3,3)
        lo, hi = w.min(axis=1), w.max(axis=1)
        if link.name.startswith("wall"):
            keep = (hi[:,2] >= LZ[0]) & (lo[:,2] <= LZ[1])
            tgt, val = wall, True
        else:
            keep = np.ones(len(lo), bool)
            rid = int(link.name.split("_")[1]); rooms[rid] = link.name
            tgt, val = room, rid
        for l, h in zip(lo[keep], hi[keep]):
            i0, j0 = idx(l[0], l[1]); i1, j1 = idx(h[0], h[1])
            i0 = max(0, i0); j0 = max(0, j0); i1 = min(NX-1, i1+1); j1 = min(NY-1, j1+1)
            if i1 >= i0 and j1 >= j0:
                if val is True:
                    tgt[i0:i1+1, j0:j1+1] = True
                else:
                    sub = tgt[i0:i1+1, j0:j1+1]
                    sub[sub == 0] = val
# glass walls + the Door model + Stairs also block
for m in models:
    if m.name not in ("Glass_wall_1", "Glass_wall_2", "OpenRobotics/Door", "Stairs"):
        continue
    for link in m.links:
        for g in link.geoms:
            if g.kind != "collision": continue
            tf = m.pose @ link.pose @ g.pose
            if g.gtype == "box":
                s = np.array(g.size)/2; mn, mx = -s, s
            elif g.gtype == "mesh":
                p, _ = resolve_uri(g.uri, DEFAULT_MODELS, DEFAULT_FUEL)
                tris = mesh_triangles(p)
                if tris is None: continue
                v = tris*np.array(g.scale); mn = v.reshape(-1,3).min(axis=0); mx = v.reshape(-1,3).max(axis=0)
            else: continue
            c = np.array([[x,y,z] for x in (mn[0],mx[0]) for y in (mn[1],mx[1]) for z in (mn[2],mx[2])])
            w = (tf[:3,:3]@c.T).T + tf[:3,3]
            lo, hi = w.min(axis=0), w.max(axis=0)
            if hi[2] < LZ[0] or lo[2] > LZ[1]: continue
            i0, j0 = idx(lo[0], lo[1]); i1, j1 = idx(hi[0], hi[1])
            wall[max(0,i0):min(NX-1,i1+1)+1, max(0,j0):min(NY-1,j1+1)+1] = True
            print("blocker", m.name, g.name, np.round(lo,2), np.round(hi,2))

np.savez_compressed(os.path.join(HERE, "walls.npz"), wall=wall, room=room,
                    origin=np.array([OX, OY]), res=RES)
print("rooms", rooms)
print("wall cells", wall.sum(), "floor cells", (room>0).sum(), "area", (room>0).sum()*RES*RES)

# ---- doorway detection on the WALL mask: a gap with a jamb on each side
def run(i, j, di, dj, limit):
    n = 0
    while n*RES < limit:
        i += di; j += dj
        if not (0 <= i < NX and 0 <= j < NY) or wall[i, j]: break
        n += 1
    return n*RES

free = (~wall) & (room > 0)
cands = {}
for axis in (0, 1):                    # 0: jambs along x (passage along y), 1: the other way
    jd = (1, 0) if axis == 0 else (0, 1)
    td = (0, 1) if axis == 0 else (1, 0)
    for i in range(1, NX-1):
        for j in range(1, NY-1):
            if not free[i, j]: continue
            dp = run(i, j, *jd, limit=2.6); dn = run(i, j, -jd[0], -jd[1], limit=2.6)
            if dp >= 2.6 or dn >= 2.6: continue
            width = dp + dn + RES
            if not (0.5 <= width <= 2.5): continue
            f1 = run(i, j, *td, limit=4.0); f2 = run(i, j, -td[0], -td[1], limit=4.0)
            if min(f1, f2) < 1.0: continue
            # the centre cell of the gap
            ci = i + int(round((dp-dn)/2/RES)) * jd[0]
            cj = j + int(round((dp-dn)/2/RES)) * jd[1]
            # rooms on the two sides of the passage (1.0 m through)
            s = int(1.0/RES)
            a = (ci+td[0]*s, cj+td[1]*s); b = (ci-td[0]*s, cj-td[1]*s)
            if not (0 <= a[0] < NX and 0 <= a[1] < NY and 0 <= b[0] < NX and 0 <= b[1] < NY): continue
            ra, rb = int(room[a]), int(room[b])
            x, y = wxy(ci, cj)
            key = (round(x*4)/4, round(y*4)/4, axis)
            rec = {"x": round(x,2), "y": round(y,2), "w": round(width,2), "axis": "y" if axis==0 else "x",
                   "rooms": sorted({ra, rb}), "through": [round(f1,2), round(f2,2)]}
            if key not in cands or rec["w"] < cands[key]["w"]:
                cands[key] = rec
out = list(cands.values())
# merge near-duplicates (same wall opening detected at several cells)
merged = []
for r in sorted(out, key=lambda d: (d["axis"], d["x"], d["y"])):
    for m2 in merged:
        if m2["axis"] == r["axis"] and abs(m2["x"]-r["x"]) < 1.0 and abs(m2["y"]-r["y"]) < 1.0:
            m2["n"] += 1
            m2["w"] = min(m2["w"], r["w"])
            break
    else:
        r = dict(r); r["n"] = 1; merged.append(r)
print(f"\n{len(merged)} wall openings (merged from {len(out)} cells):")
for r in sorted(merged, key=lambda d: (d["x"], d["y"])):
    print(json.dumps(r))
with open(os.path.join(HERE, "openings.json"), "w") as f:
    json.dump(merged, f, indent=1)
