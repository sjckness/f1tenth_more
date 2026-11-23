"""Where a person can be put on each nominal path, and whether the car can get past."""
import os, sys, json, math
import numpy as np
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from prim import *
A = lambda v: np.array([float(v)])
R_PERS = 0.175        # capsule radius
CLS_MARGIN = 0.5      # obstacle_class_margin_m person
CAR_R, AVOID = 0.20, 0.12
KEEP = R_PERS + CLS_MARGIN + CAR_R + AVOID        # centre-to-centre the solver keeps
HALF = 0.186
CORR_HALF_NEAR, CORR_HALF_FAR = 0.64, 0.64*2.3/1.3

def side_room(x, y, nx, ny, maxd=4.0):
    """free distance from (x,y) along (nx,ny) until an obstacle."""
    d = 0.0
    while d < maxd:
        d += RES
        if look(OBST, A(x+nx*d), A(y+ny*d))[0] or not look(INSIDE, A(x+nx*d), A(y+ny*d))[0]:
            return round(d-RES, 2)
    return maxd

def leg(p0, yaw, length, label, step=1.0):
    print(f"  leg {label}: from ({p0[0]:.2f},{p0[1]:.2f}) heading {math.degrees(yaw):.0f} deg, {length:.1f} m")
    hv = (math.cos(yaw), math.sin(yaw)); lv = (-math.sin(yaw), math.cos(yaw))
    s = step
    while s <= length-0.4:
        gx, gy = p0[0]+s*hv[0], p0[1]+s*hv[1]
        dl = side_room(gx, gy, lv[0], lv[1]); dr = side_room(gx, gy, -lv[0], -lv[1])
        need = KEEP + HALF
        pl = dl - need; pr = dr - need
        v = []
        if pl >= 0: v.append(f"pass LEFT ({pl:.2f} m spare)")
        if pr >= 0: v.append(f"pass RIGHT ({pr:.2f} m spare)")
        verdict = "; ".join(v) if v else "BLOCKS (no gap >= %.2f m)" % need
        corr = "inside MPC corridor" if (max(pl, pr) >= 0 and min(need, 9) <= CORR_HALF_FAR) else "needs more than the corridor half-width"
        print(f"    person at ({gx:6.2f},{gy:7.2f})  s={s:4.1f} m  free L {dl:4.2f} / R {dr:4.2f}  -> {verdict}")
        s += step

print("person keep-out (centre to car centre) =", round(KEEP, 3), "m; + car half width", HALF,
      "=> a gap of", round(KEEP+HALF, 2), "m to the wall is needed to slip past")
print("mpc corridor hard half-width: %.2f m at the car, %.2f m at the far end\n" % (CORR_HALF_NEAR, CORR_HALF_FAR))

print("## M-D / M-B  start (11.83,-6.87) yaw 0  (corridoio centrale)")
leg((11.83, -6.87), 0.0, 3.45, "1 east to the wall")
leg((16.48, -5.67), math.pi/2, 5.0, "2 north after the left turn")

print("\n## M-B1  start (16.83,-9.87) yaw 180  (open space)")
leg((16.83, -9.87), math.pi, 4.0, "1 west to the wall")
leg((11.63, -11.07), -math.pi/2, 8.0, "2 south after the left turn", step=1.5)

print("\n## M-A1  start (9.33,-15.87) yaw 90 -> door (8.08,-13.07) -> chair (3.85,-14.01)")
leg((9.33, -15.87), math.pi/2, 2.8, "1 north", step=0.8)
leg((7.78, -13.07), math.pi, 3.4, "2 west through the door", step=1.0)

print("\n## M-A3  door preset (10.064,-19.404) yaw 180 -> chair (3.84,-19.50)")
leg((10.064, -19.404), math.pi, 6.0, "1 west through the doorway", step=1.0)

print("\n## M-C2  start (10.33,-19.88) yaw 0 -> turn left -> DeskChair_6")
leg((10.33, -19.88), 0.0, 3.9, "1 east to the wall")
leg((15.43, -18.68), math.pi/2, 2.1, "2 north to the chair", step=0.7)

print("\n## M-A4  start (17.33,-6.37) yaw 90 -> door right (18.68,-2.52) -> WoodenChair_4")
leg((17.33, -6.37), math.pi/2, 3.9, "1 north")
leg((19.48, -2.52), 0.0, 3.5, "2 east inside the kitchen", step=1.0)
