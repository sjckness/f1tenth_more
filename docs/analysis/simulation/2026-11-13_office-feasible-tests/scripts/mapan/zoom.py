import os, sys, math, json
import numpy as np
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from prim import *
def show(x0, x1, y0, y1, marks=(), cell=0.25):
    st = max(1, int(cell/RES))
    i0, j0 = gij(np.array([x0]), np.array([y0])); i1, j1 = gij(np.array([x1]), np.array([y1]))
    i0, j0, i1, j1 = int(i0[0]), int(j0[0]), int(i1[0]), int(j1[0])
    mk = {}
    for (mx, my, ch) in marks:
        a, b = gij(np.array([mx]), np.array([my]))
        mk[(int(a[0])//st, int(b[0])//st)] = ch
    print(f"  x {x0}..{x1}  y {y0}..{y1}   # wall  o furniture  . floor  ' ' outside   cell {cell} m")
    hdr = "       " + "".join(("|" if (int(OX+(i+0.5)*RES))%5==0 else " ") for i in range(i0, i1, st))
    xs = "       " + "".join((str(int(abs(OX+(i+0.5)*RES))%10)) for i in range(i0, i1, st))
    print(xs); print(hdr)
    for j in range(j1, j0, -st):
        row = []
        for i in range(i0, i1, st):
            k = (i//st, j//st)
            if k in mk: row.append(mk[k]); continue
            w = WALL[i:i+st, j:j+st].any(); o = OBST[i:i+st, j:j+st].any()
            f = INSIDE[i:i+st, j:j+st].any()
            row.append('#' if w else ('o' if o else ('.' if f else ' ')))
        print(f"{OY+(j+0.5)*RES:7.1f}" + "".join(row))
    print(xs)
if __name__ == "__main__":
    which = sys.argv[1]
    if which == "sw":
        show(5.0, 14.5, -24.3, -15.0, marks=[(9.83,-22.88,'S'),(8.08,-19.42,'D'),(3.84,-19.5,'C')])
    elif which == "sw2":
        show(-1.5, 14.5, -24.3, -15.0, marks=[(9.83,-22.88,'S'),(8.08,-19.42,'D'),(3.84,-19.5,'C'),(1.37,-19.47,'c')])
    elif which == "ne":
        show(14.0, 27.0, -8.0, 3.0, marks=[(17.33,-6.37,'S'),(18.68,-2.52,'D'),(23.36,-2.26,'C'),(16.81,0.95,'P')])
    elif which == "mid":
        show(5.0, 19.0, -18.0, -7.0, marks=[(10.33,-9.37,'S'),(8.08,-13.07,'D'),(3.85,-14.01,'C')])
    elif which == "conf":
        show(-1.5, 10.0, -18.0, -8.0, marks=[(9.33,-15.87,'S'),(8.08,-13.07,'D'),(3.85,-14.01,'C')])
