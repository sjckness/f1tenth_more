"""Wall openings from the WALL mask of map.npz: width, axis, rooms, passability."""
import os, sys, json, math
import numpy as np
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from prim import *

def runlen(i, j, di, dj, limit):
    n = 0
    while n*RES < limit:
        i += di; j += dj
        if not (0 <= i < NX and 0 <= j < NY) or WALL[i, j]:
            break
        n += 1
    return n*RES

free = (~WALL) & INSIDE
cands = {}
for axis in (0, 1):
    jd = (1, 0) if axis == 0 else (0, 1)     # jambs along this
    td = (0, 1) if axis == 0 else (1, 0)     # passage along this
    for i in range(1, NX-1):
        for j in range(1, NY-1):
            if not free[i, j]:
                continue
            dp = runlen(i, j, *jd, limit=3.0); dn = runlen(i, j, -jd[0], -jd[1], limit=3.0)
            if dp >= 3.0 or dn >= 3.0:
                continue
            w = dp + dn + RES
            if not (0.5 <= w <= 2.6):
                continue
            f1 = runlen(i, j, *td, limit=3.0); f2 = runlen(i, j, -td[0], -td[1], limit=3.0)
            if min(f1, f2) < 1.2:
                continue
            ci = i + int(round((dp-dn)/2/RES))*jd[0]
            cj = j + int(round((dp-dn)/2/RES))*jd[1]
            x, y = OX+(ci+0.5)*RES, OY+(cj+0.5)*RES
            s = int(1.2/RES)
            a = (ci+td[0]*s, cj+td[1]*s); b = (ci-td[0]*s, cj-td[1]*s)
            if not all(0 <= v < n for v, n in ((a[0], NX), (a[1], NY))) or \
               not all(0 <= v < n for v, n in ((b[0], NX), (b[1], NY))):
                continue
            key = (round(x*2)/2, round(y*2)/2, axis)
            rec = {"x": round(x,2), "y": round(y,2), "w": round(w,2),
                   "through": "y" if axis == 0 else "x",
                   "rooms": sorted({int(ROOM[a]), int(ROOM[b])}),
                   "clear_in": round(float(min(CL_O[a], CL_O[b])),2)}
            if key not in cands or rec["w"] < cands[key]["w"]:
                cands[key] = rec
out = []
for r in sorted(cands.values(), key=lambda d: (d["through"], d["x"], d["y"])):
    for m in out:
        if m["through"] == r["through"] and abs(m["x"]-r["x"]) < 1.2 and abs(m["y"]-r["y"]) < 1.2:
            m["w"] = min(m["w"], r["w"]); m["n"] += 1
            m["clear_in"] = max(m["clear_in"], r["clear_in"]); break
    else:
        r = dict(r); r["n"] = 1; out.append(r)
for r in out:
    r["detectable"] = bool(0.6 <= r["w"] <= 1.5)
    r["nominal"] = bool(0.7 <= r["w"] <= 1.4)
    r["drivable"] = bool(r["w"] >= 0.55 and r["clear_in"] >= 0.30)
print(f"{len(out)} openings")
for r in sorted(out, key=lambda d: (d["x"], d["y"])):
    print(json.dumps(r))
json.dump(out, open(os.path.join(HERE, "openings2.json"), "w"), indent=1)
