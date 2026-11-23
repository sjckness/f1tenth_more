"""Vectorised mission primitives on the office grid."""
import os, math, json
import numpy as np
HERE = os.path.dirname(os.path.abspath(__file__))
D = np.load(os.path.join(HERE, "map.npz"))
WALL, OBST, ROOM, CL_O, CL_W, INSIDE = D["wall"], D["obst"], D["room"], D["cl_o"], D["cl_w"], D["inside"]
OX, OY, RES, NX, NY = D["meta"]
NX, NY = int(NX), int(NY)
HALF_W = 0.186
FRONT_X = 0.405
REAR_X = -0.15
RMIN = 0.96

def gij(x, y):
    i = np.clip(((x-OX)/RES).astype(np.int32), 0, NX-1)
    j = np.clip(((y-OY)/RES).astype(np.int32), 0, NY-1)
    return i, j

def look(mask, x, y):
    i, j = gij(np.asarray(x, float), np.asarray(y, float))
    return mask[i, j]

def ray(mask, X, Y, YAW, maxd=20.0, half=HALF_W, from_x=FRONT_X, noff=7):
    """Distance from the FRONT BUMPER to the first `mask` cell inside the swept corridor."""
    X, Y, YAW = map(lambda a: np.asarray(a, float), (X, Y, YAW))
    c, s = np.cos(YAW), np.sin(YAW)
    offs = np.linspace(-half, half, noff)
    K = int(maxd/RES)
    d = np.full(X.shape, np.inf)
    done = np.zeros(X.shape, bool)
    for k in range(K+1):
        r = from_x + k*RES
        hit = np.zeros(X.shape, bool)
        for o in offs:
            px = X + r*c - o*s
            py = Y + r*s + o*c
            hit |= look(mask, px, py)
        new = hit & ~done
        d[new] = k*RES
        done |= hit
        if done.all():
            break
    return d

def strip_free(mask, X, Y, YAW, length, half=HALF_W, from_x=REAR_X):
    """Is the rectangle from from_x to from_x+length (body frame) free of mask?"""
    X, Y, YAW = map(lambda a: np.asarray(a, float), (X, Y, YAW))
    c, s = np.cos(YAW), np.sin(YAW)
    ok = np.ones(X.shape, bool)
    for k in range(int(length/RES)+1):
        r = from_x + k*RES
        for o in np.linspace(-half, half, 7):
            ok &= ~look(mask, X + r*c - o*s, Y + r*s + o*c)
    return ok

def on_floor(X, Y):
    return look(INSIDE, X, Y)

def turn90(X, Y, YAW, sign, R=RMIN*1.25):
    """90 deg turn of radius R. sign +1 left. Returns (feasible, x1, y1, yaw1)."""
    X, Y, YAW = map(lambda a: np.asarray(a, float), (X, Y, YAW))
    c, s = np.cos(YAW), np.sin(YAW)
    cx, cy = X - sign*R*s, Y + sign*R*c            # arc centre (left of the car for sign +1)
    ok = np.ones(X.shape, bool)
    for t in np.linspace(0, math.pi/2, 19):
        a = YAW + sign*t
        px = cx + sign*R*np.sin(a); py = cy - sign*R*np.cos(a)
        ca, sa = np.cos(a), np.sin(a)
        for r in np.linspace(REAR_X, FRONT_X, 6):
            for o in np.linspace(-HALF_W, HALF_W, 5):
                qx = px + r*ca - o*sa; qy = py + r*sa + o*ca
                ok &= ~look(OBST, qx, qy) & on_floor(qx, qy)
    a = YAW + sign*math.pi/2
    return ok, cx + sign*R*np.sin(a), cy - sign*R*np.cos(a), a

def los(X, Y, TX, TY, mask=None):
    """Line of sight clear of walls from (X,Y) to (TX,TY) (stop 0.3 m short)."""
    mask = WALL if mask is None else mask
    X, Y, TX, TY = map(lambda a: np.asarray(a, float), (X, Y, TX, TY))
    d = np.hypot(TX-X, TY-Y)
    n = int(np.nanmax(d)/RES)+1 if d.size else 1
    ok = np.ones(X.shape, bool)
    for k in range(n):
        f = np.clip((k*RES)/np.maximum(d, 1e-6), 0, 1)
        f = np.minimum(f, np.maximum(0.0, (d-0.3)/np.maximum(d, 1e-6)))
        ok &= ~look(mask, X + f*(TX-X), Y + f*(TY-Y))
    return ok
