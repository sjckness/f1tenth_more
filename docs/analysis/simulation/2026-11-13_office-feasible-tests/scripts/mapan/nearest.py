import os, sys, json, math
import numpy as np
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from prim import *
ops = json.load(open(os.path.join(HERE, "openings2.json")))
A = lambda v: np.array([float(v)])
LIDAR_FOV = math.radians(135)
cases = [
 ("M-A1 at the door move (after the 2.8 m leg)", 9.33, -13.07, 90, (8.08, -13.07)),
 ("M-D2 after left turn, SW", 5.88, -22.18, 90, (8.08, -19.42)),
 ("M-D2 at the door move", 5.88, -20.58, 90, (8.08, -19.42)),
 ("M-A4 at the door move (after 3.9 m)", 17.33, -2.52, 90, (18.68, -2.52)),
 ("M-D3 after left turn, open space south", 11.63, -11.07, -90, (8.08, -13.07)),
 ("M-A1 start", 9.33, -15.87, 90, (8.08, -13.07)),
 ("M-A2 start", 9.83, -22.88, 90, (8.08, -19.42)),
 ("M-A3 door preset", 10.064, -19.404, 180, (8.08, -19.42)),
 ("M-A4 kitchen", 17.33, -6.37, 90, (18.68, -2.52)),
 ("M-A5 corridor preset", 16.814, 0.946, -90, (18.68, -2.52)),
 ("M-D after turn", 16.48, -5.67, 90, (18.68, -2.52)),
 ("M-D' after turn (left door)", 16.48, -5.67, 90, (15.23, -2.27)),
 ("M-A6 south into sala riunioni", 10.33, -9.37, -90, (8.08, -13.07)),
]
for nm, x, y, yawd, target in cases:
    yaw = math.radians(yawd)
    rows = []
    for o in ops:
        if not o["detectable"]:
            continue
        d = math.hypot(o["x"]-x, o["y"]-y)
        b = (math.atan2(o["y"]-y, o["x"]-x) - yaw + math.pi) % (2*math.pi) - math.pi
        if d > 9.0 or abs(b) > LIDAR_FOV:
            continue
        vis = bool(los(A(x), A(y), A(o["x"]), A(o["y"]))[0])
        if not vis:
            continue
        rows.append((round(d,2), round(math.degrees(b)), o["x"], o["y"], o["w"]))
    rows.sort()
    tgt = [r for r in rows if abs(r[2]-target[0])<0.4 and abs(r[3]-target[1])<0.4]
    rank = rows.index(tgt[0])+1 if tgt else None
    print(f"\n{nm}  pose ({x},{y}) yaw {yawd}  -> target door {target} rank {rank} of {len(rows)} visible")
    for i, r in enumerate(rows[:5]):
        mark = " <== target" if tgt and r == tgt[0] else ""
        print(f"   {i+1}. ({r[2]:6.2f},{r[3]:7.2f}) w{r[4]:.2f}  d {r[0]:4.2f} m  bearing {r[1]:4d} deg{mark}")
