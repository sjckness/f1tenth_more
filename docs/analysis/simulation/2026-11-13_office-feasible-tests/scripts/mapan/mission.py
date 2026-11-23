"""Mission primitive simulator on the office occupancy grid + start-pose search."""
import sys, os, json, math
import numpy as np
REPO = r"f:/Users/Andreas Steffens/dev_ws/f1tenth_hil"
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(REPO, "sim/isaac/tools"))
from sdf_to_usd import (parse_world, DEFAULT_SDF, DEFAULT_MODELS, DEFAULT_FUEL,
                        resolve_uri, mesh_triangles)

RES = 0.05
LZ = (0.05, 0.35)            # lidar band
OX, OY = -1.70, -24.40
NX, NY = int((27.2-OX)/RES)+1, int((3.1-OY)/RES)+1
# car geometry (f1tenth_params)
HALF_W = 0.136 + 0.05        # body half width + beside margin
FRONT_X = 0.405              # base_link -> front bumper
REAR_X = -0.15
RMIN = 0.96                  # worst-side minimum turning radius

def ij(x, y):
    return np.clip(((np.asarray(x)-OX)/RES).astype(int), 0, NX-1), \
           np.clip(((np.asarray(y)-OY)/RES).astype(int), 0, NY-1)

def build():
    models, meta = parse_world(DEFAULT_SDF)
    wall = np.zeros((NX, NY), bool)      # structure at the lidar plane
    hmax = np.zeros((NX, NY), float)     # top height of any collision geometry
    obst = np.zeros((NX, NY), bool)      # everything at the lidar plane
    room = np.zeros((NX, NY), np.int16)
    objs = []
    tcache = {}
    for m in models:
        is_struct = m.name in ("Surfaces", "Glass_wall_1", "Glass_wall_2", "OpenRobotics/Door", "Stairs")
        mlo = np.array([1e9]*3); mhi = np.array([-1e9]*3)
        for link in m.links:
            for g in link.geoms:
                if g.kind != "collision":
                    continue
                tf = m.pose @ link.pose @ g.pose
                if g.gtype == "mesh":
                    if g.uri not in tcache:
                        p, _ = resolve_uri(g.uri, DEFAULT_MODELS, DEFAULT_FUEL)
                        tcache[g.uri] = mesh_triangles(p) if os.path.isfile(p) else None
                    tris = tcache[g.uri]
                    if tris is None:
                        continue
                    v = tris*np.array(g.scale)
                    w = (tf[:3,:3] @ v.reshape(-1,3).T).T + tf[:3,3]
                    w = w.reshape(-1,3,3)
                    lo, hi = w.min(axis=1), w.max(axis=1)
                else:
                    if g.gtype == "box":
                        s = np.array(g.size)/2; mn, mx = -s, s
                    elif g.gtype in ("cylinder", "sphere"):
                        r = float(g.size[0]); h = float(g.size[1]) if g.gtype=="cylinder" and len(g.size)>1 else 2*r
                        mn = np.array([-r,-r,-h/2]); mx = np.array([r,r,h/2])
                    else:
                        continue
                    c = np.array([[a,b,cc] for a in (mn[0],mx[0]) for b in (mn[1],mx[1]) for cc in (mn[2],mx[2])])
                    w = (tf[:3,:3]@c.T).T + tf[:3,3]
                    lo = w.min(axis=0)[None,:]; hi = w.max(axis=0)[None,:]
                mlo = np.minimum(mlo, lo.min(axis=0)); mhi = np.maximum(mhi, hi.max(axis=0))
                floorish = is_struct and link.name.startswith("floor")
                if not floorish:
                    hi_pos = hi[:, 2] > 0.02
                    if hi_pos.any():
                        a0, b0 = ij(lo[hi_pos,0], lo[hi_pos,1]); a1, b1 = ij(hi[hi_pos,0], hi[hi_pos,1])
                        for a,b,c2,d,h in zip(a0,b0,a1,b1,hi[hi_pos,2]):
                            np.maximum(hmax[a:c2+1, b:d+1], h, out=hmax[a:c2+1, b:d+1])
                keep = np.ones(len(lo), bool) if floorish else ((hi[:,2] >= LZ[0]) & (lo[:,2] <= LZ[1]))
                if not keep.any():
                    continue
                i0, j0 = ij(lo[keep,0], lo[keep,1]); i1, j1 = ij(hi[keep,0], hi[keep,1])
                for a,b,c2,d in zip(i0,j0,i1,j1):
                    if floorish:
                        rid = int(link.name.split("_")[1])
                        sub = room[a:c2+1, b:d+1]; sub[sub == 0] = rid
                    else:
                        obst[a:c2+1, b:d+1] = True
                        if is_struct:
                            wall[a:c2+1, b:d+1] = True
        if mlo[0] < 1e8 and not is_struct:
            objs.append({"name": m.name, "lo": mlo.round(3).tolist(), "hi": mhi.round(3).tolist(),
                         "c": [(mlo[0]+mhi[0])/2, (mlo[1]+mhi[1])/2]})
    return wall, obst, room, objs, meta, hmax

def edt(mask):
    INF = 1e12
    def dt1(fr):
        n = len(fr); d = np.empty(n); v = np.zeros(n, int); z = np.empty(n+1)
        k = 0; z[0] = -INF; z[1] = INF
        for q in range(1, n):
            while True:
                s = ((fr[q]+q*q)-(fr[v[k]]+v[k]*v[k]))/(2.0*q-2.0*v[k])
                if s <= z[k]: k -= 1
                else: break
            k += 1; v[k] = q; z[k] = s; z[k+1] = INF
        k = 0
        for q in range(n):
            while z[k+1] < q: k += 1
            d[q] = (q-v[k])**2 + fr[v[k]]
        return d
    f = np.where(mask, 0.0, INF)
    for i in range(f.shape[0]): f[i] = dt1(f[i])
    for j in range(f.shape[1]): f[:,j] = dt1(f[:,j])
    return np.sqrt(f)*RES

if __name__ == "__main__":
    wall, obst, room, objs, meta, hmax = build()
    inside = room > 0
    print("wall", wall.sum(), "obst", obst.sum(), "floor", inside.sum(), "objs", len(objs))
    cl_o = edt(obst); cl_w = edt(wall)
    np.savez_compressed(os.path.join(HERE, "map.npz"), wall=wall, obst=obst, room=room,
                        cl_o=cl_o, cl_w=cl_w, inside=inside, hmax=hmax,
                        meta=np.array([OX, OY, RES, NX, NY]))
    with open(os.path.join(HERE, "objects.json"), "w") as f:
        json.dump(objs, f, indent=1)
    print("saved. free area", (inside & ~obst).sum()*RES*RES)
