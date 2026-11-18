#!/usr/bin/env python3
"""The "dodge" scenario family D(d, seed): levels of cylinders sliding across the car's path.

    dodge.py --d 0 3 6 --seeds 0 1 2          # generate (and check) scenarios/dodge/Ddd_sN.json
    dodge.py --show D04_s1                    # print one, with its oracle path

Pure Python (numpy, stdlib). The car drives straight at 0.5 m/s from (0.5, 3.0) in the Isaac layout
`dodge` (~/isaac_ws base_room, 8.5 x 6 m: the tuning room made longer, same width). LEVELS are rows
of cylinders at fixed distances ahead of the nominal start (1.5, 2.5, ... m). On each level 1-2
cylinders (D0.3, now and then D0.5) slide back and forth from wall to wall ("pingpong" motion of
~/isaac_ws sim/motion.py, started by /sim/scenario_go when the mission goes RUNNING), the initial
direction alternating between levels; some levels move DIAGONALLY (15-35 deg off the lateral axis),
their segment clipped to +-X_BAND_M around the level so they never wander into the next one. Two
cylinders on one level move as a rigid pair (same segment, same phase, 1.2-2.0 m apart), so they
never pass through each other.

DIFFICULTY RAMP d = 0, 1, 2, ...: cylinder speed 0.2 + 0.1 d m/s (cap 1.5), share of diagonal
levels min(0.5, 0.1 d), levels 2 + d // 3 (cap MAX_LEVELS). The mission ends 1.0 m past the last
level.

FEASIBILITY ORACLE (feasible()). A forward search on a lattice of (x, y, heading, steering) states:
kinematic bicycle at the car's constant speed, steering within +-0.319 rad and at most 0.5 rad/s
(the MPC's own limits), starting after a start lag. A state survives a step when the car's disc
centre (axle midpoint) stays >= R_safe_ref + ORACLE_BUFFER_M from every cylinder, whose position is
checked at t and at t +- ORACLE_TIME_SLACK_S (a path that only works with perfect timing is not one
a controller reacting to a tracker can fly), and >= WALL_CLEARANCE_M from the side walls. A scenario
whose lattice never reaches the mission distance is rejected and the next candidate seed is drawn:
D(d, seed) is the first FEASIBLE candidate of the sequence seed * 1000 + attempt. Nobody tunes on an
impossible scenario. The oracle knows the future exactly, so feasible here means "possible for a
clairvoyant", an upper bound on what any controller can do.
"""
from __future__ import annotations

import argparse
import json
import math
import os
import random
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
DODGE_DIR = os.path.join(HERE, 'scenarios', 'dodge')

# Geometry of the Isaac `dodge` layout (~/isaac_ws config/scenes/base_room.yaml presets.dodge).
ROOM_X = (0.0, 8.5)
ROOM_Y = (0.0, 6.0)
CAR_START = {'x': 0.5, 'y': 3.0, 'yaw_deg': 0.0}
CAR_SPEED = 0.5
LEVEL_AHEAD_M = (1.5, 2.5, 3.5, 4.5, 5.5)
MAX_LEVELS = 5
PAST_LAST_LEVEL_M = 1.0
POOL = {0.3: [f'c030_{i}' for i in range(10)], 0.5: [f'c050_{i}' for i in range(10)]}
WALL_GAP_M = 0.05             # a cylinder's surface turns this far from a wall
X_BAND_M = 0.6                # a diagonal segment stays within +-this of its level
P_PAIR = 0.35                 # chance of two cylinders on a level
P_BIG = 0.25                  # chance a cylinder is D0.5 rather than D0.3
PAIR_SPACING_M = (1.2, 2.0)   # centre to centre, along the motion
DIAG_DEG = (15.0, 35.0)

# Car / safety numbers (common.py, metrics.py, stack_params).
WHEELBASE_M = 0.305
CAR_RADIUS_M = 0.20
SHIPPED_MARGIN_M = 0.12
DELTA_MAX = 0.319
DELTA_RATE = 0.5
ORACLE_DT = 0.2
ORACLE_START_LAG_S = 0.6
ORACLE_TIME_SLACK_S = 0.25
ORACLE_BUFFER_M = 0.0
WALL_CLEARANCE_M = 0.45
ORACLE_MAX_PSI = math.radians(60.0)
ORACLE_MAX_STATES = 6000


def ramp(d: int) -> dict:
    """The difficulty knobs of level d."""
    return {'speed': round(min(1.5, 0.2 + 0.1 * d), 3), 'diag_frac': round(min(0.5, 0.1 * d), 3),
            'n_levels': int(min(MAX_LEVELS, 2 + d // 3))}


def pingpong_point(a, b, s: float):
    """Same formula as ~/isaac_ws sim/motion.py (and common.motion_position)."""
    ax, ay = float(a[0]), float(a[1])
    bx, by = float(b[0]), float(b[1])
    length = math.hypot(bx - ax, by - ay)
    if length <= 1e-9:
        return [ax, ay]
    u = math.fmod(s, 2.0 * length)
    if u < 0.0:
        u += 2.0 * length
    f = (u if u <= length else 2.0 * length - u) / length
    return [ax + f * (bx - ax), ay + f * (by - ay)]


def _clip_segment(cx, cy, ux, uy, x_lo, x_hi, y_lo, y_hi):
    """The segment of the line (cx, cy) + s (ux, uy) inside the box; (s_min, s_max)."""
    lo, hi = -1e9, 1e9
    for c, u, a, b in ((cx, ux, x_lo, x_hi), (cy, uy, y_lo, y_hi)):
        if abs(u) < 1e-12:
            if not (a <= c <= b):
                return None
            continue
        s1, s2 = (a - c) / u, (b - c) / u
        lo, hi = max(lo, min(s1, s2)), min(hi, max(s1, s2))
    return (lo, hi) if hi > lo else None


def candidate(d: int, cand_seed: int) -> dict:
    """One random scenario of level d (not checked). Coordinates in the NOMINAL start frame:
    ahead (x) and lateral (y, left-positive) from the car's nominal start."""
    rng = random.Random(cand_seed)
    rp = ramp(d)
    half_w = (ROOM_Y[1] - ROOM_Y[0]) / 2.0
    y_off = CAR_START['y'] - (ROOM_Y[0] + half_w)       # 0 in the dodge room (car on the middle)
    first_dir = rng.choice((1.0, -1.0))
    pool = {k: list(v) for k, v in POOL.items()}
    levels, obstacles = [], []
    for i in range(rp['n_levels']):
        x_i = LEVEL_AHEAD_M[i]
        n = 2 if rng.random() < P_PAIR else 1
        diams = [0.5 if rng.random() < P_BIG else 0.3 for _ in range(n)]
        diagonal = rng.random() < rp['diag_frac']
        theta = math.radians(rng.uniform(*DIAG_DEG)) * rng.choice((1.0, -1.0)) if diagonal else 0.0
        direction = first_dir * (1.0 if i % 2 == 0 else -1.0)   # alternate between levels
        # unit vector along the motion (ahead, lateral), lateral component signed by `direction`
        ux, uy = math.sin(theta), direction * math.cos(theta)
        rmax = max(diams) / 2.0
        lat_lo = -half_w - y_off + rmax + WALL_GAP_M
        lat_hi = half_w - y_off - rmax - WALL_GAP_M
        seg = _clip_segment(x_i, 0.0, ux, uy, x_i - X_BAND_M, x_i + X_BAND_M, lat_lo, lat_hi)
        if seg is None:
            raise RuntimeError('level segment empty')
        s_a, s_b = seg
        spacing = rng.uniform(*PAIR_SPACING_M) if n == 2 else 0.0
        if s_b - s_a <= spacing + 0.2:       # a short diagonal cannot hold the pair: one cylinder
            n, diams, spacing = 1, diams[:1], 0.0
        a = (x_i + s_a * ux, s_a * uy)
        length = (s_b - s_a) - spacing
        s0 = rng.uniform(0.0, length)          # heading from a to b at the go
        speed = rp['speed']
        level = {'level': i, 'ahead_m': x_i, 'n': n, 'diameters': diams, 'theta_deg': math.degrees(theta),
                 'direction': direction, 'speed': speed, 'segment_m': s_b - s_a, 'spacing_m': spacing}
        for j in range(n):
            off = j * spacing
            pa = (a[0] + off * ux, a[1] + off * uy)
            pb = (pa[0] + length * ux, pa[1] + length * uy)
            oid = pool[diams[j]].pop(0)
            start = pingpong_point(pa, pb, s0)
            obstacles.append({
                'id': oid, 'level': i, 'diameter': diams[j],
                'ahead_m': round(start[0], 4), 'lateral_m': round(start[1], 4),
                'motion': {'type': 'pingpong', 'a': [round(pa[0], 4), round(pa[1], 4)],
                           'b': [round(pb[0], 4), round(pb[1], 4)], 'speed': speed,
                           's0': round(s0, 4), 'delay_s': 0.0}})
        levels.append(level)
    distance = LEVEL_AHEAD_M[rp['n_levels'] - 1] + PAST_LAST_LEVEL_M
    return {'levels': levels, 'obstacles': obstacles, 'distance_m': distance, 'ramp': rp}


# ---------------------------------------------------------------- oracle
def _obstacle_positions(obstacles, times):
    """(T, K, 2) cylinder centres (nominal frame) at `times` after the go, and (K,) radii."""
    pos = np.zeros((len(times), len(obstacles), 2))
    for k, ob in enumerate(obstacles):
        m = ob['motion']
        for i, t in enumerate(times):
            pos[i, k] = pingpong_point(m['a'], m['b'], float(m['s0']) + float(m['speed']) * max(t, 0.0))
    return pos, np.array([ob['diameter'] / 2.0 for ob in obstacles])


def feasible(scn: dict, start_lag: float = ORACLE_START_LAG_S, dt: float = ORACLE_DT,
             speed: float = CAR_SPEED, buffer: float = ORACLE_BUFFER_M,
             time_slack: float = ORACLE_TIME_SLACK_S, return_path: bool = False):
    """True when a clairvoyant car at constant `speed` can reach the mission distance (see module doc).

    With return_path, (ok, path) where path is [(t, x, y, psi)] of one solution (nominal frame).
    """
    obstacles = scn['obstacles']
    distance = float(scn['distance_m'])
    half_w = (ROOM_Y[1] - ROOM_Y[0]) / 2.0
    y_lo, y_hi = -half_w + WALL_CLEARANCE_M, half_w - WALL_CLEARANCE_M
    ds = speed * dt
    d_steps = np.array([-DELTA_RATE * dt, -0.5 * DELTA_RATE * dt, 0.0, 0.5 * DELTA_RATE * dt,
                        DELTA_RATE * dt])
    n_max = int(math.ceil((distance / speed) * 2.0 / dt)) + 10
    times = [start_lag + dt * (i + 1) for i in range(n_max)]
    slack = [-time_slack, 0.0, time_slack] if time_slack > 0 else [0.0]
    pos_s = [_obstacle_positions(obstacles, [t + s for t in times])[0] for s in slack]
    radii = np.array([ob['diameter'] / 2.0 for ob in obstacles]) if obstacles else np.zeros(0)
    need = radii + CAR_RADIUS_M + SHIPPED_MARGIN_M + buffer
    # the car waits still for start_lag: the cylinders must leave it alone meanwhile
    if obstacles:
        cx0 = WHEELBASE_M / 2.0
        wait_t = np.arange(0.0, start_lag + 1e-9, 0.1)
        p_wait, _ = _obstacle_positions(obstacles, list(wait_t))
        dist0 = np.hypot(p_wait[:, :, 0] - cx0, p_wait[:, :, 1])
        if np.any(dist0 < need[None, :]):
            return (False, []) if return_path else False
    # states: x, y, psi, delta ; parent index per step for the path
    X = np.zeros(1)
    Y = np.zeros(1)
    P = np.zeros(1)
    D = np.zeros(1)
    parents = []
    hist = []
    for i in range(n_max):
        nX = (X[:, None] + 0 * d_steps[None, :]).ravel()
        nD = np.clip(D[:, None] + d_steps[None, :], -DELTA_MAX, DELTA_MAX).ravel()
        Pp = np.repeat(P, len(d_steps))
        Yp = np.repeat(Y, len(d_steps))
        par = np.repeat(np.arange(len(X)), len(d_steps))
        # exact kinematic bicycle (rear axle) over ds with the new steering held
        k = np.tan(nD) / WHEELBASE_M
        nP = Pp + k * ds
        small = np.abs(k) < 1e-9
        kk = np.where(small, 1.0, k)
        nX = np.where(small, nX + ds * np.cos(Pp), nX + (np.sin(nP) - np.sin(Pp)) / kk)
        nY = np.where(small, Yp + ds * np.sin(Pp), Yp - (np.cos(nP) - np.cos(Pp)) / kk)
        ok = (np.abs(nP) <= ORACLE_MAX_PSI)
        cx = nX + 0.5 * WHEELBASE_M * np.cos(nP)
        cy = nY + 0.5 * WHEELBASE_M * np.sin(nP)
        ok &= (cy >= y_lo) & (cy <= y_hi)
        if len(radii):
            for ps in pos_s:
                dd = np.hypot(cx[:, None] - ps[i, None, :, 0], cy[:, None] - ps[i, None, :, 1])
                ok &= np.all(dd >= need[None, :], axis=1)
        if not np.any(ok):
            return (False, []) if return_path else False
        nX, nY, nP, nD, par = nX[ok], nY[ok], nP[ok], nD[ok], par[ok]
        # dedup on a grid, then cap the count (keep the furthest ahead)
        key = np.stack([np.round(nX / 0.04), np.round(nY / 0.04), np.round(nP / math.radians(3.0)),
                        np.round(nD / (0.25 * DELTA_RATE * dt))], axis=1)
        _, idx = np.unique(key, axis=0, return_index=True)
        if len(idx) > ORACLE_MAX_STATES:
            idx = idx[np.argsort(-nX[idx])[:ORACLE_MAX_STATES]]
        X, Y, P, D = nX[idx], nY[idx], nP[idx], nD[idx]
        parents.append(par[idx])
        hist.append((X.copy(), Y.copy(), P.copy()))
        if np.any(X >= distance):
            if not return_path:
                return True
            j = int(np.argmax(X))
            path = []
            for step in range(len(hist) - 1, -1, -1):
                hx, hy, hp = hist[step]
                path.append((times[step], float(hx[j]), float(hy[j]), float(hp[j])))
                j = int(parents[step][j])
            return True, path[::-1]
    return (False, []) if return_path else False


# ---------------------------------------------------------------- scenarios
def scenario_id(d: int, seed: int) -> str:
    return f'D{int(d):02d}_s{int(seed)}'


def generate(d: int, seed: int, max_attempts: int = 200) -> dict:
    """D(d, seed): the first feasible candidate of the seed's sequence, as a harness scenario."""
    rejected = 0
    for attempt in range(max_attempts):
        cand_seed = int(seed) * 1000 + int(d) * 100000 + attempt
        scn = candidate(d, cand_seed)
        if feasible(scn):
            break
        rejected += 1
    else:
        raise RuntimeError(f'D({d}, {seed}): no feasible candidate in {max_attempts}')
    sid = scenario_id(d, seed)
    rp = scn['ramp']
    desc = (f'dodge d={d}: {rp["n_levels"]} levels, cylinders at {rp["speed"]:.1f} m/s, '
            f'{sum(1 for lv in scn["levels"] if lv["theta_deg"])} diagonal')
    return {
        'id': sid, 'family': 'D', 'd': int(d), 'seed': int(seed), 'candidate_seed': cand_seed,
        'rejected_candidates': rejected, 'description': desc, 'scene': 'base_room', 'layout': 'dodge',
        'car_start': dict(CAR_START),
        'mission': {'distance_m': scn['distance_m'], 'speed_mps': CAR_SPEED,
                    'timeout_s': round(scn['distance_m'] / CAR_SPEED + 8.0, 1)},
        'ramp': rp, 'levels': scn['levels'],
        'obstacles': [{'id': ob['id'], 'level': ob['level'], 'ahead_m': ob['ahead_m'],
                       'lateral_m': ob['lateral_m'], 'motion': ob['motion']} for ob in scn['obstacles']],
        'jitter': {'seed': 7000 + int(d) * 10 + int(seed), 'xy_m': 0.05, 'yaw_deg': 3.0},
        'oracle': {'start_lag_s': ORACLE_START_LAG_S, 'time_slack_s': ORACLE_TIME_SLACK_S,
                   'buffer_m': ORACLE_BUFFER_M, 'dt_s': ORACLE_DT, 'wall_clearance_m': WALL_CLEARANCE_M},
    }


def path_for(sid: str) -> str:
    return os.path.join(DODGE_DIR, f'{sid}.json')


def ensure(d: int, seed: int) -> dict:
    """The scenario D(d, seed), from scenarios/dodge/ or generated and written there."""
    p = path_for(scenario_id(d, seed))
    if os.path.exists(p):
        with open(p, encoding='utf-8') as fh:
            return json.load(fh)
    scn = generate(d, seed)
    os.makedirs(DODGE_DIR, exist_ok=True)
    with open(p, 'w', encoding='utf-8') as fh:
        json.dump(scn, fh, indent=1)
    return scn


def parse_id(sid: str):
    """'D04_s1' -> (4, 1); None for anything else."""
    if not (sid.startswith('D') and '_s' in sid):
        return None
    a, b = sid[1:].split('_s', 1)
    return int(a), int(b)


def oracle_obstacles(scn: dict) -> list:
    """The scenario's obstacles in the oracle's form (with diameters from the pool ids)."""
    out = []
    for ob in scn['obstacles']:
        diam = 0.5 if ob['id'].startswith('c050') else 0.3
        out.append({'diameter': diam, 'motion': ob['motion']})
    return out


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.strip().splitlines()[0])
    ap.add_argument('--d', type=int, nargs='*', default=[])
    ap.add_argument('--seeds', type=int, nargs='*', default=[0, 1, 2])
    ap.add_argument('--show', default=None)
    args = ap.parse_args(argv)
    if args.show:
        d, seed = parse_id(args.show)
        scn = ensure(d, seed)
        ok, path = feasible({'obstacles': oracle_obstacles(scn), 'distance_m': scn['mission']['distance_m']},
                            return_path=True)
        print(json.dumps({k: scn[k] for k in ('id', 'description', 'ramp', 'levels', 'rejected_candidates')},
                         indent=1))
        print('oracle path (t, x, y):', [(round(t, 1), round(x, 2), round(y, 2)) for t, x, y, _ in path][::5])
        return 0
    for d in args.d:
        for seed in args.seeds:
            scn = ensure(d, seed)
            print(f'{scn["id"]}: {scn["description"]}; rejected {scn["rejected_candidates"]}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
