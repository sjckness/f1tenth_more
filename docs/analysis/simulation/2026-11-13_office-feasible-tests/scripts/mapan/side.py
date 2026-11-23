"""Which door does enter_door{side} pick? nearest STRONG door with |bearing| in 15..135 deg on that side."""
import os, sys, json, math
import numpy as np
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from prim import *
ops = [o for o in json.load(open(os.path.join(HERE, "openings2.json"))) if o["detectable"]]
A = lambda v: np.array([float(v)])
def pick(x, y, yawd, side):
    yaw = math.radians(yawd); rows = []
    for o in ops:
        d = math.hypot(o["x"]-x, o["y"]-y)
        b = math.degrees((math.atan2(o["y"]-y, o["x"]-x) - yaw + math.pi) % (2*math.pi) - math.pi)
        if d > 9.0 or abs(b) > 135:
            continue
        if not bool(los(A(x), A(y), A(o["x"]), A(o["y"]))[0]):
            continue
        rows.append((round(d,2), round(b), o["x"], o["y"], o["w"]))
    rows.sort()
    sector = [r for r in rows if (15 <= r[1] <= 135 if side == "left" else -135 <= r[1] <= -15)]
    return rows, sector
CASES = [
 ("M-A  door move start (9.33,-13.27) yaw 90", 9.33, -13.27, 90, "left", (8.08,-13.07)),
 ("M-A' door move start (17.33,-2.77) yaw 90", 17.33, -2.77, 90, "right", (18.68,-2.52)),
 ("M-A2 door move start (9.83,-19.38) yaw 90", 9.83, -19.38, 90, "left", (8.08,-19.42)),
 ("M-D  door move start (11.63,-11.59) yaw -90", 11.63, -11.59, -90, "right", (8.08,-13.07)),
 ("M-D' corridor after turn (16.48,-4.65) yaw 90", 16.48, -4.65, 90, "right", (18.68,-2.52)),
 ("M-D' corridor after turn (16.48,-4.65) yaw 90", 16.48, -4.65, 90, "left", (15.23,-2.27)),
 ("corridor preset (16.814,0.946) yaw -90", 16.814, 0.946, -90, "left", (18.68,-2.52)),
]
for nm, x, y, yd, side, target in CASES:
    rows, sect = pick(x, y, yd, side)
    got = sect[0] if sect else None
    ok = got and abs(got[2]-target[0]) < 0.4 and abs(got[3]-target[1]) < 0.4
    print(f"\n{nm}  side={side}")
    for r in rows[:5]:
        tag = ""
        if 15 <= abs(r[1]) <= 135:
            tag = "  [" + ("left" if r[1] > 0 else "right") + " sector]"
        print(f"   ({r[2]:6.2f},{r[3]:7.2f}) w{r[4]:.2f}  d {r[0]:4.2f}  bearing {r[1]:5d} deg{tag}")
    print(f"   -> enter_door side={side} picks "
          f"{('(%.2f,%.2f)' % (got[2], got[3])) if got else 'NOTHING (door_not_found)'}"
          f"   {'CORRECT' if ok else 'WRONG / none'}  (wanted {target})")
