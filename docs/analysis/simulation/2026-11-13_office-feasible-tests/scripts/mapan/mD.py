"""wall -> turn left -> (person on the leg) -> first door on the right."""
import os, sys, json, math
import numpy as np
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from prim import *
ops = [o for o in json.load(open(os.path.join(HERE, "openings2.json"))) if o["detectable"] and o["drivable"]]
ROOMNAME = {1:"corridoio SW",2:"sala SW",3:"ufficio SE",4:"corridoio N",5:"bagno NE",6:"cucina NE",
            7:"stanza NW",8:"sala riunioni W",9:"passaggio W",10:"stanza W",11:"open space",
            12:"ufficio E",13:"ripostiglio N",14:"stanza N-centro"}
step = int(0.5/RES)
ii, jj = np.meshgrid(np.arange(0, NX, step), np.arange(0, NY, step), indexing="ij")
m = INSIDE[ii, jj] & (CL_O[ii, jj] > 0.45)
xs = OX+(ii[m]+0.5)*RES; ys = OY+(jj[m]+0.5)*RES
X = np.concatenate([xs]*4); Y = np.concatenate([ys]*4)
YAW = np.concatenate([np.full(xs.shape, a) for a in (0.0, math.pi/2, math.pi, -math.pi/2)])
d1 = ray(WALL, X, Y, YAW, maxd=22.0); d1o = ray(OBST, X, Y, YAW, maxd=22.0)
leg1 = d1 - 3.0
base = np.isfinite(d1) & (leg1 > 1.0) & (leg1 < 7.0) & (d1o >= d1-0.05)
x1 = np.where(base, X + leg1*np.cos(YAW), 1e4); y1 = np.where(base, Y + leg1*np.sin(YAW), 1e4)
out = []
for sign, side_turn in ((+1, "left"), (-1, "right")):
    okt, x2, y2, yaw2 = turn90(x1, y1, YAW, sign)
    ok = base & okt
    for k in np.flatnonzero(ok):
        if not strip_free(OBST, np.array([X[k]]), np.array([Y[k]]), np.array([YAW[k]]), float(leg1[k]))[0]:
            continue
        px, py, ph = float(x2[k]), float(y2[k]), float(yaw2[k])
        hv = (math.cos(ph), math.sin(ph)); rv = (math.sin(ph), -math.cos(ph))
        for door_side, sv in (("right", rv), ("left", (-rv[0], -rv[1]))):
            for op in ops:
                s = (op["x"]-px)*hv[0] + (op["y"]-py)*hv[1]
                t = (op["x"]-px)*sv[0] + (op["y"]-py)*sv[1]
                if not (2.5 <= s <= 6.5 and 0.8 <= t <= 2.5):
                    continue
                if not strip_free(OBST, np.array([px]), np.array([py]), np.array([ph]), float(s)+0.3)[0]:
                    continue
                # first on that side?
                earlier = [o2 for o2 in ops if o2 is not op
                           and 0.5 <= ((o2["x"]-px)*hv[0]+(o2["y"]-py)*hv[1]) < s-0.3
                           and 0.5 <= ((o2["x"]-px)*sv[0]+(o2["y"]-py)*sv[1]) <= 3.5]
                if earlier:
                    continue
                # person on the leg, 1.5 m before the abeam point, lateral clearance check
                sp = max(1.6, s-1.6)
                gx, gy = px + sp*hv[0], py + sp*hv[1]
                i3, j3 = gij(np.array([gx]), np.array([gy]))
                freew = float(CL_W[i3[0], j3[0]])
                i4, j4 = gij(np.array([op["x"]+sv[0]*0]), np.array([op["y"]]))
                inx = op["x"] - sv[0]*1.0; iny = op["y"] - sv[1]*1.0
                out.append({"turn": side_turn, "door_side": door_side,
                            "start": [round(float(X[k]),2), round(float(Y[k]),2)], "yaw": round(math.degrees(YAW[k])),
                            "wall1": round(float(d1[k]),2), "leg1": round(float(leg1[k]),2),
                            "after_turn": [round(px,2), round(py,2), round(math.degrees(ph))],
                            "door": [op["x"], op["y"]], "w": op["w"], "rooms": op["rooms"],
                            "ahead": round(float(s),2), "lat": round(float(t),2),
                            "person": [round(gx,2), round(gy,2)], "person_wall_clear": round(freew,2)})
def sc(r):
    return (abs(r["leg1"]-4.0) + abs(r["ahead"]-4.0) + abs(r["lat"]-1.5))
best = {}
for r in out:
    k = (tuple(r["door"]), r["turn"], r["door_side"], tuple(r["after_turn"][2:]))
    if k not in best or sc(r) < sc(best[k]):
        best[k] = r
print(f"{len(out)} chains, {len(best)} distinct cases\n")
for r in sorted(best.values(), key=sc)[:20]:
    print(f"turn {r['turn']:5s} door-{r['door_side']:5s} start({r['start'][0]:6.2f},{r['start'][1]:7.2f}) yaw{r['yaw']:4d} "
          f"wall1 {r['wall1']:5.2f} leg1 {r['leg1']:4.1f} -> ({r['after_turn'][0]:6.2f},{r['after_turn'][1]:7.2f}) "
          f"yaw{r['after_turn'][2]:4d} | door({r['door'][0]:5.2f},{r['door'][1]:7.2f}) w{r['w']:.2f} rooms{r['rooms']} "
          f"ahead {r['ahead']:4.1f} lat {r['lat']:4.1f} | person({r['person'][0]:6.2f},{r['person'][1]:7.2f}) "
          f"wallclear {r['person_wall_clear']:.2f}")
json.dump(list(best.values()), open(os.path.join(HERE, "mD.json"), "w"), indent=1)
