"""wall -> turn left/right -> go_to <class> [-> straight 2 m] feasibility."""
import os, sys, json, math
import numpy as np
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from prim import *
HMAX = np.load(os.path.join(HERE, "map.npz"))["hmax"]
objs = json.load(open(os.path.join(HERE, "objects2.json")))
ROOMNAME = {1:"corridoio SW",2:"sala SW",3:"ufficio SE",4:"corridoio centrale N",5:"bagno NE",
            6:"cucina/sala NE",7:"stanza NW",8:"sala riunioni W",9:"passaggio W",10:"stanza W",
            11:"open space centrale",12:"ufficio E",13:"ripostiglio N",14:"stanza N-centro"}
WALL_T_TURN, WALL_T = 3.0, 2.0
GAP = {"chair": 0.5, "person": 1.0}

def vis(px, py, pyaw, cls, maxrange=6.0, fov=math.radians(45), top=1.0):
    out = []
    for o in objs:
        if o["class"] != cls: continue
        tx, ty = o["c"]; d = math.hypot(tx-px, ty-py)
        if not (0.8 < d <= maxrange): continue
        b = (math.atan2(ty-py, tx-px) - pyaw + math.pi) % (2*math.pi) - math.pi
        if abs(b) > fov: continue
        blocked = False
        for k in range(1, int(d/RES)):
            f = k*RES/d
            if f > 0.85: break
            i2, j2 = gij(np.array([px+f*(tx-px)]), np.array([py+f*(ty-py)]))
            if HMAX[i2[0], j2[0]] > CAMZ + (top-CAMZ)*f + 0.02:
                blocked = True; break
        if blocked: continue
        out.append({"obj": o["name"].split("/")[-1], "d": round(d,2), "bearing": round(math.degrees(b),1),
                    "xy": [round(tx,2), round(ty,2)]})
    return sorted(out, key=lambda t: t["d"])
CAMZ = 0.22

step = int(0.5/RES)
ii, jj = np.meshgrid(np.arange(0, NX, step), np.arange(0, NY, step), indexing="ij")
m = INSIDE[ii, jj] & (CL_O[ii, jj] > 0.45)
xs = OX+(ii[m]+0.5)*RES; ys = OY+(jj[m]+0.5)*RES
X = np.concatenate([xs]*4); Y = np.concatenate([ys]*4)
YAW = np.concatenate([np.full(xs.shape, a) for a in (0.0, math.pi/2, math.pi, -math.pi/2)])

d1 = ray(WALL, X, Y, YAW, maxd=22.0)
d1o = ray(OBST, X, Y, YAW, maxd=22.0)
leg1 = d1 - WALL_T_TURN
base = np.isfinite(d1) & (leg1 > 1.0) & (leg1 < 8.0) & (d1o >= d1-0.05)
print("poses with a usable wall leg:", int(base.sum()))

res = {"left": [], "right": []}
for sign, side in ((+1, "left"), (-1, "right")):
    okt, x2, y2, yaw2 = turn90(np.where(base, X, 1e4), np.where(base, Y, 1e4), YAW, sign)
    x1 = X + leg1*np.cos(YAW); y1 = Y + leg1*np.sin(YAW)
    okt, x2, y2, yaw2 = turn90(x1, y1, YAW, sign)
    ok = base & okt
    for k in np.flatnonzero(ok):
        if not strip_free(OBST, np.array([X[k]]), np.array([Y[k]]), np.array([YAW[k]]), float(leg1[k]))[0]:
            continue
        ch = vis(float(x2[k]), float(y2[k]), float(yaw2[k]), "chair")
        d3 = float(ray(WALL, np.array([x2[k]]), np.array([y2[k]]), np.array([yaw2[k]]), maxd=22.0)[0])
        i2, j2 = gij(np.array([X[k]]), np.array([Y[k]]))
        rec = {"start": [round(float(X[k]),2), round(float(Y[k]),2)], "yaw": round(math.degrees(YAW[k])),
               "room": int(ROOM[i2[0], j2[0]]), "wall1": round(float(d1[k]),2), "leg1": round(float(leg1[k]),2),
               "after_turn": [round(float(x2[k]),2), round(float(y2[k]),2), round(math.degrees(yaw2[k]))],
               "wall2": round(d3,2), "chairs": ch[:3]}
        if ch:
            # can the car then still do 2 m straight from the go_to stop point?
            c0 = ch[0]; tx, ty = c0["xy"]
            dd = math.hypot(tx-x2[k], ty-y2[k])
            stop_d = max(0.0, dd - 0.30 - GAP["chair"] - FRONT_X)   # base_link travel
            sx = float(x2[k]) + stop_d*math.cos(float(yaw2[k])); sy = float(y2[k]) + stop_d*math.sin(float(yaw2[k]))
            byaw = math.atan2(ty-sy, tx-sx)
            free2 = bool(strip_free(OBST, np.array([sx]), np.array([sy]), np.array([byaw]), 2.0)[0])
            rec["go_to_stop"] = [round(sx,2), round(sy,2)]
            rec["straight2m_after_chair_free"] = free2
        res[side].append(rec)

for side in ("left", "right"):
    withch = [r for r in res[side] if r["chairs"]]
    print(f"\n===== wall -> turn {side.upper()} : {len(res[side])} poses, {len(withch)} with a chair visible after the turn")
    withch.sort(key=lambda r: (abs(r['leg1']-4.0), r["chairs"][0]["d"]))
    for r in withch[:14]:
        c = r["chairs"][0]
        print(f"start({r['start'][0]:6.2f},{r['start'][1]:7.2f}) yaw{r['yaw']:4d} room{r['room']:3d} "
              f"{ROOMNAME.get(r['room'],'?'):20s} wall1 {r['wall1']:5.2f} leg1 {r['leg1']:4.1f} -> "
              f"({r['after_turn'][0]:6.2f},{r['after_turn'][1]:7.2f}) yaw{r['after_turn'][2]:4d} "
              f"| chair {c['obj']:22s} {c['d']:4.1f}m {c['bearing']:6.1f}deg | wall2 {r['wall2']:5.2f} "
              f"| +2m free: {r['straight2m_after_chair_free']}")
    json.dump(res[side], open(os.path.join(HERE, f"turn_{side}.json"), "w"), indent=1)
