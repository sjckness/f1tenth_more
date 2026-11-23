"""Office map analysis: occupancy at the lidar plane, clearance, doorways, objects."""
import sys, os, json, math
import numpy as np
REPO = r"f:/Users/Andreas Steffens/dev_ws/f1tenth_hil"
sys.path.insert(0, os.path.join(REPO, "sim/isaac/tools"))
import pick_spawn_poses as P
from sdf_to_usd import parse_world, DEFAULT_SDF, DEFAULT_MODELS, DEFAULT_FUEL

def edt(mask, res):
    """Exact euclidean distance transform of ~mask (distance to nearest True), Felzenszwalb."""
    INF = 1e12
    f = np.where(mask, 0.0, INF)
    # 1D squared distance transform along each axis
    def dt1(fr):
        n = len(fr); d = np.empty(n); v = np.zeros(n, dtype=int); z = np.empty(n+1)
        k = 0; v[0] = 0; z[0] = -INF; z[1] = INF
        for q in range(1, n):
            while True:
                s = ((fr[q] + q*q) - (fr[v[k]] + v[k]*v[k])) / (2.0*q - 2.0*v[k])
                if s <= z[k]:
                    k -= 1
                else:
                    break
            k += 1; v[k] = q; z[k] = s; z[k+1] = INF
        k = 0
        for q in range(n):
            while z[k+1] < q:
                k += 1
            d[q] = (q - v[k])**2 + fr[v[k]]
        return d
    out = np.empty_like(f)
    for i in range(f.shape[0]):
        out[i] = dt1(f[i])
    for j in range(f.shape[1]):
        out[:, j] = dt1(out[:, j])
    return np.sqrt(out) * res

models, meta = parse_world(DEFAULT_SDF)
occ, origin, res, floor, raster = P.occupancy(models, DEFAULT_MODELS, DEFAULT_FUEL, 0.05)
print("grid", occ.shape, "origin", origin, "res", res, raster)
clear = edt(occ, res)
inside = np.isfinite(floor) & (np.abs(np.where(np.isfinite(floor), floor, 1e3)) < 0.05)
clear_in = np.where(inside, clear, 0.0)
np.savez_compressed(os.path.join(os.path.dirname(__file__), "grid.npz"),
                    occ=occ, floor=floor, clear=clear, inside=inside, origin=np.array(origin), res=res)
print("free area m2", float(((~occ)&inside).sum())*res*res)

def floor_at(x, y):
    i, j = int((x-origin[0])/res), int((y-origin[1])/res)
    if not (0 <= i < floor.shape[0] and 0 <= j < floor.shape[1]):
        return None
    z = float(floor[i, j])
    return None if not math.isfinite(z) else round(z, 3)

ways = P.doorways(occ, clear_in, origin, res, floor_at, width_range=(0.6, 2.2), min_passage=1.5, standoff=2.0)
print("doorways found:", len(ways))
for w in ways:
    print(json.dumps(w))
with open(os.path.join(os.path.dirname(__file__), "doorways.json"), "w") as f:
    json.dump(ways, f, indent=1)
