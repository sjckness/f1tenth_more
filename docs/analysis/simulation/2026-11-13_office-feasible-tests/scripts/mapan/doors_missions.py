"""Missions with a door step: 'go straight, enter the first door on the left/right, then <target>'."""
import os, sys, json, math
import numpy as np
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from prim import *
HMAX = np.load(os.path.join(HERE, "map.npz"))["hmax"]
ops = json.load(open(os.path.join(HERE, "openings2.json")))
objs = json.load(open(os.path.join(HERE, "objects2.json")))
CAM_Z = 0.22
ROOMNAME = {1:"corridoio SW",2:"sala SW",3:"ufficio SE",4:"corridoio centrale N",5:"bagno NE",
            6:"cucina/sala NE",7:"stanza NW",8:"sala riunioni W",9:"passaggio W",10:"stanza W",
            11:"open space centrale",12:"ufficio E",13:"ripostiglio N",14:"stanza N-centro"}

def vis(px, py, pyaw, cls, maxrange=7.0, fov=math.radians(45), top=1.0):
    out = []
    for o in objs:
        if o["class"] != cls: continue
        tx, ty = o["c"]
        d = math.hypot(tx-px, ty-py)
        if not (0.6 < d <= maxrange): continue
        b = (math.atan2(ty-py, tx-px) - pyaw + math.pi) % (2*math.pi) - math.pi
        if abs(b) > fov: continue
        blocked = False
        for k in range(1, int(d/RES)):
            f = k*RES/d
            if f > 0.88: break
            i2, j2 = gij(np.array([px+f*(tx-px)]), np.array([py+f*(ty-py)]))
            if HMAX[i2[0], j2[0]] > CAM_Z + (top-CAM_Z)*f + 0.02:
                blocked = True; break
        if blocked: continue
        out.append({"obj": o["name"].split("/")[-1], "d": round(d,2), "bearing": round(math.degrees(b),1),
                    "xy": [round(tx,2), round(ty,2)], "h": o["hi"][2]})
    return sorted(out, key=lambda t: t["d"])

HEAD = {"+x": 0.0, "+y": math.pi/2, "-x": math.pi, "-y": -math.pi/2}
def left_of(h):      # unit vector 90 deg CCW from heading
    return (-math.sin(h), math.cos(h))

results = []
for op in ops:
    if not (op["detectable"] and op["drivable"]):
        continue
    # passage direction candidates
    pdirs = [(1.0, 0.0), (-1.0, 0.0)] if op["through"] == "x" else [(0.0, 1.0), (0.0, -1.0)]
    for pd in pdirs:
        for side, sgn in (("left", +1), ("right", -1)):
            # heading h such that sgn*left_of(h) == pd  ->  h = atan2 of pd rotated -sgn*90
            ang = math.atan2(pd[1], pd[0])
            h = ang - sgn*math.pi/2
            hv = (math.cos(h), math.sin(h)); lv = left_of(h)
            # candidate corridor lines: start poses on the lattice with heading h
            step = int(0.5/RES)
            ii, jj = np.meshgrid(np.arange(0, NX, step), np.arange(0, NY, step), indexing="ij")
            m = INSIDE[ii, jj] & (CL_O[ii, jj] > 0.45)
            xs = OX+(ii[m]+0.5)*RES; ys = OY+(jj[m]+0.5)*RES
            dx = op["x"]-xs; dy = op["y"]-ys
            s = dx*hv[0] + dy*hv[1]            # ahead
            t = (dx*lv[0] + dy*lv[1])*sgn      # toward the door side
            sel = (s >= 2.0) & (s <= 8.0) & (t >= 0.8) & (t <= 3.0)
            if not sel.any(): continue
            for x0, y0, ss, tt in zip(xs[sel], ys[sel], s[sel], t[sel]):
                # straight leg free, and the car passes abeam the door
                if not strip_free(OBST, np.array([x0]), np.array([y0]), np.array([h]), float(ss)+0.5)[0]:
                    continue
                # the doorway must be laterally visible: clear line from abeam point to the door centre
                ax, ay = x0+ss*hv[0], y0+ss*hv[1]
                if not los(np.array([ax]), np.array([ay]), np.array([op["x"]]), np.array([op["y"]]))[0]:
                    continue
                # is it the FIRST opening on that side along the leg? count other same-side openings nearer
                earlier = []
                for o2 in ops:
                    if o2 is op or not o2["detectable"]:
                        continue
                    s2 = (o2["x"]-x0)*hv[0] + (o2["y"]-y0)*hv[1]
                    t2 = ((o2["x"]-x0)*lv[0] + (o2["y"]-y0)*lv[1])*sgn
                    if 0.5 <= s2 < ss-0.3 and 0.5 <= t2 <= 3.5:
                        earlier.append((o2["x"], o2["y"], round(s2,2)))
                # target visible from the doorway, looking through it
                thr = math.atan2(pd[1], pd[0])
                inx, iny = op["x"]+pd[0]*0.8, op["y"]+pd[1]*0.8
                i3, j3 = gij(np.array([inx]), np.array([iny]))
                room_in = int(ROOM[i3[0], j3[0]])
                chairs = vis(inx, iny, thr, "chair")
                results.append({"door": [op["x"], op["y"]], "w": op["w"], "through": op["through"],
                                "rooms": op["rooms"], "side": side, "start": [round(float(x0),2), round(float(y0),2)],
                                "yaw": round(math.degrees(h)), "ahead": round(float(ss),2),
                                "lateral": round(float(tt),2), "room_beyond": room_in,
                                "earlier_same_side": earlier, "chairs_from_door": chairs[:3]})
# keep the best per (door, side): first-on-side, then a chair visible, then ahead ~3-5 m
def score(r):
    return (0 if not r["earlier_same_side"] else 1,
            0 if r["chairs_from_door"] else 1,
            abs(r["ahead"]-4.0), abs(r["lateral"]-1.5))
best = {}
for r in results:
    k = (tuple(r["door"]), r["side"], r["through"], r["room_beyond"])
    if k not in best or score(r) < score(best[k]):
        best[k] = r
print(f"{len(results)} (door,start) combinations; {len(best)} distinct door/side/direction cases\n")
for r in sorted(best.values(), key=score):
    rb = ROOMNAME.get(r["room_beyond"], "?")
    flag = "FIRST" if not r["earlier_same_side"] else f"NOT-first ({len(r['earlier_same_side'])} before)"
    ch = ", ".join(f"{c['obj']}@{c['d']}m/{c['bearing']}deg" for c in r["chairs_from_door"]) or "-"
    print(f"door({r['door'][0]:6.2f},{r['door'][1]:7.2f}) w{r['w']:.2f} thr-{r['through']}  "
          f"{r['side']:5s}  start=({r['start'][0]:6.2f},{r['start'][1]:7.2f}) yaw{r['yaw']:4d}  "
          f"ahead {r['ahead']:4.1f} lat {r['lateral']:4.1f}  -> room {r['room_beyond']:2d} {rb:20s} {flag:22s} chairs: {ch}")
json.dump(list(best.values()), open(os.path.join(HERE, "door_missions.json"), "w"), indent=1)
