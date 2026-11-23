"""Simulate an explicit primitive chain from a start pose and report each step."""
import os, sys, json, math
import numpy as np
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from prim import *
HMAX = np.load(os.path.join(HERE, "map.npz"))["hmax"]
objs = json.load(open(os.path.join(HERE, "objects2.json")))
BY = {o["name"].split("/")[-1]: o for o in objs}
CAMZ, GAP = 0.22, {"chair": 0.5, "person": 1.0}
A = lambda v: np.array([float(v)])

def seen(px, py, pyaw, tx, ty, top=1.0):
    d = math.hypot(tx-px, ty-py)
    b = (math.atan2(ty-py, tx-px) - pyaw + math.pi) % (2*math.pi) - math.pi
    blocked = None
    for k in range(1, max(2, int(d/RES))):
        f = k*RES/d
        if f > 0.85: break
        i2, j2 = gij(A(px+f*(tx-px)), A(py+f*(ty-py)))
        h = float(HMAX[i2[0], j2[0]])
        if h > CAMZ + (top-CAMZ)*f + 0.02:
            blocked = round(h, 2); break
    return round(d, 2), round(math.degrees(b), 1), blocked

def run(name, start, steps):
    x, y, yaw = start[0], start[1], math.radians(start[2])
    print(f"\n### {name}\nstart ({x:.2f}, {y:.2f}) yaw {math.degrees(yaw):.0f} deg")
    ok_all = True
    for st in steps:
        kind = st[0]
        if kind == "straight_wall":
            thr = st[1]
            dw = float(ray(WALL, A(x), A(y), A(yaw), maxd=25)[0])
            do = float(ray(OBST, A(x), A(y), A(yaw), maxd=25)[0])
            leg = dw - thr
            free = bool(strip_free(OBST, A(x), A(y), A(yaw), max(leg, 0.0))[0]) if leg > 0 else False
            ok = leg > 0.5 and free
            print(f"  straight -> wall at {thr} m : wall {dw:.2f} m, nearest obstacle {do:.2f} m, "
                  f"drive {leg:.2f} m, path clear {free}  => {'OK' if ok else 'FAIL'}")
            if not ok: ok_all = False; break
            x, y = x + leg*math.cos(yaw), y + leg*math.sin(yaw)
        elif kind == "straight_dist":
            n = st[1]
            free = bool(strip_free(OBST, A(x), A(y), A(yaw), n)[0])
            do = float(ray(OBST, A(x), A(y), A(yaw), maxd=25)[0])
            print(f"  straight {n} m : nearest obstacle ahead {do:.2f} m, corridor clear {free} "
                  f" => {'OK' if free else 'FAIL'}")
            if not free: ok_all = False; break
            x, y = x + n*math.cos(yaw), y + n*math.sin(yaw)
        elif kind == "turn":
            sg = +1 if st[1] == "left" else -1
            okt, x2, y2, y2w = turn90(A(x), A(y), A(yaw), sg)
            print(f"  turn {st[1]} 90 deg (R={RMIN*1.25:.2f} m): swept area clear {bool(okt[0])} "
                  f"-> ({float(x2[0]):.2f}, {float(y2[0]):.2f}) yaw {math.degrees(float(y2w[0])):.0f}"
                  f"  => {'OK' if bool(okt[0]) else 'FAIL'}")
            if not bool(okt[0]): ok_all = False; break
            x, y, yaw = float(x2[0]), float(y2[0]), float(y2w[0])
        elif kind == "go_to":
            cls, tx, ty = st[1], st[2], st[3]
            top = 1.0 if cls == "chair" else 1.6
            d, b, bl = seen(x, y, yaw, tx, ty, top)
            infov = abs(b) <= 50.0
            okv = infov and bl is None and d <= 7.0
            gap = GAP[cls]
            travel = max(0.0, d - 0.30 - gap - FRONT_X)
            free = bool(strip_free(OBST, A(x), A(y), A(math.atan2(ty-y, tx-x)), travel)[0])
            print(f"  go_to {cls} @({tx},{ty}) : range {d} m, bearing {b} deg (FOV +-50: {infov}), "
                  f"occluder {bl}, approach {travel:.2f} m clear {free}  => {'OK' if okv and free else 'FAIL'}")
            if not (okv and free): ok_all = False
            yaw = math.atan2(ty-y, tx-x)
            x, y = x + travel*math.cos(yaw), y + travel*math.sin(yaw)
        elif kind == "enter_door":
            dx, dy, thr_dir = st[1], st[2], st[3]
            d = math.hypot(dx-x, dy-y)
            b = (math.atan2(dy-y, dx-x) - yaw + math.pi) % (2*math.pi) - math.pi
            tyaw = {"+x": 0.0, "-x": math.pi, "+y": math.pi/2, "-y": -math.pi/2}[thr_dir]
            lv = (-math.sin(yaw), math.cos(yaw))
            side = "left" if ((dx-x)*lv[0] + (dy-y)*lv[1]) > 0 else "right"
            print(f"  enter_door @({dx},{dy}) through {thr_dir} : {d:.2f} m, bearing {math.degrees(b):.0f} deg, "
                  f"side {side}")
            x, y, yaw = dx + math.cos(tyaw)*0.3, dy + math.sin(tyaw)*0.3, tyaw
        print(f"       pose now ({x:.2f}, {y:.2f}) yaw {math.degrees(yaw):.0f}")
    print(f"  == {'FEASIBLE' if ok_all else 'NOT feasible as written'}")
    return (x, y, math.degrees(yaw)), ok_all

if __name__ == "__main__":
    C = lambda n: (BY[n]["c"][0], BY[n]["c"][1])
    run("M-A1  go straight, enter the first door on the LEFT, approach the chair  (sala riunioni)",
        (9.33, -15.87, 90), [("straight_dist", 2.8), ("enter_door", 8.08, -13.07, "-x"),
                             ("go_to", "chair") + C("ConfOfficeChairBlack5")])
    run("M-A2  same, SW room (DeskChair_2)",
        (9.83, -22.88, 90), [("straight_dist", 3.5), ("enter_door", 8.08, -19.42, "-x"),
                             ("go_to", "chair") + C("DeskChair_2")])
    run("M-A3  head-on variant from the `door` preset: straight, through the door, chair",
        (10.064, -19.404, 180), [("straight_dist", 2.0), ("enter_door", 8.08, -19.42, "-x"),
                                 ("go_to", "chair") + C("DeskChair_2")])
    run("M-B1  go straight at the wall, turn LEFT, approach the wall (open space)",
        (16.83, -9.87, 180), [("straight_wall", 3.0), ("turn", "left"), ("straight_wall", 2.0)])
    run("M-B2  same, SW->E leg",
        (8.83, -19.88, 0), [("straight_wall", 3.0), ("turn", "left"), ("straight_wall", 2.0)])
    run("M-C1  at the wall turn LEFT, go to the chair, then straight 2 m",
        (16.83, -9.87, 180), [("straight_wall", 3.0), ("turn", "left"),
                              ("go_to", "chair") + C("DeskChair_7"), ("straight_dist", 2.0)])
    run("M-C2  variant in the open space (DeskChair_6)",
        (10.33, -19.88, 0), [("straight_wall", 3.0), ("turn", "left"),
                             ("go_to", "chair") + C("DeskChair_6"), ("straight_dist", 2.0)])
    run("M-D1  at the wall turn LEFT, go to the PERSON, enter the first door on the RIGHT",
        (16.83, -9.87, 180), [("straight_wall", 3.0), ("turn", "left"),
                              ("go_to", "person", 11.6, -13.6), ("enter_door", 8.08, -13.07, "-x")])
