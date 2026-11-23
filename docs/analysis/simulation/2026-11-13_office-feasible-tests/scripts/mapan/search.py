"""Start-pose search for the four mission patterns."""
import os, sys, json, math
import numpy as np
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import prim
from prim import *
HMAX = np.load(os.path.join(HERE, "map.npz"))["hmax"]
objs = json.load(open(os.path.join(HERE, "objects2.json")))
chairs = [o for o in objs if o["class"] == "chair"]

WALL_T_TURN = 3.0     # thresh before a turn
WALL_T = 2.0          # "approach the wall"
CAM_Z = 0.22
GO_TO_GAP = 0.4

# ---------- candidate start poses: free floor cells on a 0.5 m lattice, 4 cardinal headings
step = int(0.5/RES)
ii, jj = np.meshgrid(np.arange(0, NX, step), np.arange(0, NY, step), indexing="ij")
free = INSIDE[ii, jj] & (CL_O[ii, jj] > 0.45)        # car fits with margin
ii, jj = ii[free], jj[free]
xs = OX + (ii+0.5)*RES; ys = OY + (jj+0.5)*RES
X = np.concatenate([xs]*4); Y = np.concatenate([ys]*4)
YAW = np.concatenate([np.full(xs.shape, a) for a in (0.0, math.pi/2, math.pi, -math.pi/2)])
print("candidate poses:", len(X))

def visible_chairs(px, py, pyaw, maxrange=6.0, fov=math.radians(45)):
    """Chairs visible from one pose: (obj, range, bearing)."""
    out = []
    for o in chairs:
        tx, ty = o["c"]
        d = math.hypot(tx-px, ty-py)
        if not (0.6 < d <= maxrange):
            continue
        b = math.atan2(ty-py, tx-px) - pyaw
        b = (b + math.pi) % (2*math.pi) - math.pi
        if abs(b) > fov:
            continue
        # height aware LOS: eye 0.22 -> chair back 1.00
        n = int(d/RES); blocked = False
        for k in range(1, n):
            f = k*RES/d
            if f > 0.90:
                break
            qx, qy = px + f*(tx-px), py + f*(ty-py)
            i2, j2 = gij(np.array([qx]), np.array([qy]))
            h = HMAX[i2[0], j2[0]]
            if h > CAM_Z + (1.00-CAM_Z)*f + 0.02:
                blocked = True; break
        if blocked:
            continue
        out.append((o, round(d, 2), round(math.degrees(b), 1)))
    return sorted(out, key=lambda t: t[1])

# ---------- Mission B: straight at the wall -> turn left -> approach the wall
def mission_B(sign=+1):
    d1 = ray(WALL, X, Y, YAW, maxd=25.0)
    d1o = ray(OBST, X, Y, YAW, maxd=25.0)
    run = d1 - WALL_T_TURN
    ok = np.isfinite(d1) & (run > 1.0) & (d1o >= d1 - 0.05)   # nothing but wall ahead
    x1 = X + np.where(ok, run, 0)*np.cos(YAW); y1 = Y + np.where(ok, run, 0)*np.sin(YAW)
    ok &= strip_free(OBST, X, Y, YAW, np.where(ok, run, 0).max()*0+0.0) if False else ok
    # strip check per pose (variable length) - do it with the max then verify individually later
    okt, x2, y2, yaw2 = turn90(x1, y1, YAW, sign)
    ok &= okt
    d3 = ray(WALL, x2, y2, yaw2, maxd=25.0)
    d3o = ray(OBST, x2, y2, yaw2, maxd=25.0)
    ok &= np.isfinite(d3) & (d3 - WALL_T > 0.5) & (d3o >= d3 - 0.05)
    idx = np.flatnonzero(ok)
    out = []
    for k in idx:
        L = float(d1[k]-WALL_T_TURN)
        if not strip_free(OBST, np.array([X[k]]), np.array([Y[k]]), np.array([YAW[k]]), L)[0]:
            continue
        L2 = float(d3[k]-WALL_T)
        if not strip_free(OBST, np.array([x2[k]]), np.array([y2[k]]), np.array([yaw2[k]]), L2)[0]:
            continue
        i2, j2 = gij(np.array([X[k]]), np.array([Y[k]]))
        out.append({"x": round(float(X[k]),2), "y": round(float(Y[k]),2),
                    "yaw": round(math.degrees(YAW[k]),0), "room": int(ROOM[i2[0],j2[0]]),
                    "wall_ahead": round(float(d1[k]),2), "leg1": round(L,2),
                    "after_turn": [round(float(x2[k]),2), round(float(y2[k]),2), round(math.degrees(yaw2[k]),0)],
                    "wall_ahead2": round(float(d3[k]),2), "leg2": round(L2,2),
                    "score": round(min(L,4.0)+min(L2,4.0),2)})
    return sorted(out, key=lambda d: -d["score"])

if __name__ == "__main__":
    for sign, nm in ((+1, "LEFT"), (-1, "RIGHT")):
        res = mission_B(sign)
        print(f"\n===== M-B  wall -> turn {nm} -> wall : {len(res)} feasible start poses")
        for r in res[:12]:
            print(json.dumps(r))
        json.dump(res, open(os.path.join(HERE, f"mB_{nm}.json"), "w"), indent=1)
