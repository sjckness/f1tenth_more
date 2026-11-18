"""Metrics and score of one tuning run, from the recorder's CSV streams. Pure Python, no ROS.

Everything is measured on Isaac's ground truth (/sim/ground_truth/odom, map_sim) in the START FRAME:
the car's pose when the mission went RUNNING, x along its heading, y to its left. Obstacles are the
true ones the scenario placed. Times are sim seconds.

Outcome, first match wins:
  collision  the car's footprint rectangle comes within CONTACT_M of an obstacle's circle
  estop      a safety layer stopped the car: hard_stop engaged, the BT's emergency trip / safety
             stop, or the mux forwarding a safety lane (hard_stop, emergency, proximity_guard,
             obstacle)
  success    the mission reached COMPLETE
  stuck      the car stood still (< STUCK_SPEED) for STUCK_S while the mission was running
  timeout    anything else (ABORTED on its timeout, or never ended)

Score (0..100): 0 for collision / estop (hard failures); 10 x progress for stuck / timeout; for a
success 100 x (W_MARGIN f_margin + W_TIME f_time + W_OSC f_osc + W_TRACK f_track), each in 0..1:
  f_margin  1 when the car-centre clearance >= R_safe_ref (no obstacle: 1), falling linearly to 0
            at MARGIN_FLOOR_M inside it
  f_time    nominal time (distance / speed) / time to goal, capped at 1
  f_osc     1 for <= 1 steering sign change (one swerve and back), then 1 / (1 + (n - 1) / 2)
  f_track   no obstacle: 1 - max |y| / TRACK_TOL_M; with an obstacle: 1 - |final heading error| /
            HEADING_TOL_RAD (the corridor does not home laterally, so parallel-offset is by design)
R_safe_ref = r + car_radius 0.20 + 0.12, the SHIPPED margin, whatever obstacle_safety_margin_m the set
under test uses: a set cannot pass by shrinking its own margin.
"""
from __future__ import annotations

import bisect
import csv
import json
import math
import os
import statistics

from common import CAR_RADIUS_M, FOOTPRINT_HALF_WIDTH_M, FOOTPRINT_X_M, WHEELBASE_M

SHIPPED_MARGIN_M = 0.12
# obstacle_class_margin_m as shipped ({person: 0.5}): R_safe_ref of a person includes it (phase 4).
CLASS_MARGIN_M = {'person': 0.5}
CONTACT_M = 0.01
STUCK_SPEED = 0.02
STUCK_S = 3.0
STEER_DEADBAND_RAD = 0.02
MARGIN_FLOOR_M = 0.15
TRACK_TOL_M = 0.15
HEADING_TOL_RAD = 0.30
W_MARGIN, W_TIME, W_OSC, W_TRACK = 0.4, 0.2, 0.2, 0.2
SAFETY_FRAMES = ('base_link/hard_stop', 'base_link/emergency', 'base_link/proximity_guard',
                 'base_link/obstacle')
HARD_FAILURES = ('collision', 'estop')
# Steering saturation (mpc_corr's limits: delta_max 0.319 rad, dDeltaMax 0.5 rad/s), counted at
# SAT_FRAC of the limit on the /drive commands.
DELTA_MAX_RAD = 0.319
DELTA_RATE_MAX = 0.5
SAT_FRAC = 0.97
SOLVE_DEADLINE_MS = 100.0


def read_csv(path: str) -> list[dict]:
    if not os.path.exists(path):
        return []
    with open(path, newline='', encoding='utf-8') as fh:
        return list(csv.DictReader(fh))


def _f(v, default=math.nan) -> float:
    try:
        return float(v)
    except (TypeError, ValueError):
        return default


def footprint_circle_clearance(x, y, yaw, cx, cy, r) -> float:
    """Distance from the car's footprint rectangle (base_link pose) to a circle; <0 = overlap."""
    dx, dy = cx - x, cy - y
    lx = math.cos(yaw) * dx + math.sin(yaw) * dy
    ly = -math.sin(yaw) * dx + math.cos(yaw) * dy
    qx = min(max(lx, FOOTPRINT_X_M[0]), FOOTPRINT_X_M[1])
    qy = min(max(ly, -FOOTPRINT_HALF_WIDTH_M), FOOTPRINT_HALF_WIDTH_M)
    return math.hypot(lx - qx, ly - qy) - r


def sign_changes(values, deadband=STEER_DEADBAND_RAD) -> int:
    signs = [1 if v > deadband else -1 for v in values if abs(v) > deadband]
    return sum(1 for a, b in zip(signs, signs[1:]) if a != b)


def compute(run_dir: str) -> dict:
    """The run.json content for one recorded run directory."""
    with open(os.path.join(run_dir, 'meta.json'), encoding='utf-8') as fh:
        meta = json.load(fh)
    inst = meta['instance']
    out = {'scenario': inst['scenario'], 'rep': inst['rep'], 'recorder_exit': meta.get('exit'),
           'end_reason': meta.get('end_reason'), 'phases_wall_s': meta.get('phases')}
    if (meta.get('exit') or {}).get('code') != 0:
        out.update(outcome='invalid', score=None, valid=False)
        return out
    out['valid'] = True
    m = inst['mission']
    dist, speed, timeout = float(m['distance_m']), float(m['speed_mps']), float(m['timeout_s'])
    t_run = _f(meta.get('t_running'))
    gt = [r for r in read_csv(os.path.join(run_dir, 'gt.csv'))]
    states = [(_f(r['t']), r['state']) for r in read_csv(os.path.join(run_dir, 'mission.csv'))]
    t_end = next((t for t, s in states if t >= t_run and s in ('COMPLETE', 'ABORTED', 'IDLE')),
                 _f(meta.get('t_end')))
    final_state = next((s for t, s in states if t >= t_run and s in ('COMPLETE', 'ABORTED', 'IDLE')),
                       None)
    # finite only: after a hard contact with a kinematic cylinder Isaac's car can go NaN (seen in
    # dodge runs); the run is a collision already, its NaN tail must not become a 20 m lateral error
    # The window ends at the first non-physical sample: after a hard contact with a kinematic
    # cylinder Isaac's car can blow up (|v| of 1e3..1e7 m/s, then NaN; seen in dodge runs). The run
    # is a collision already; its blown-up tail must not become a 20 m lateral error.
    window = []
    out['gt_blowup_t'] = None
    for r in gt:
        if not (t_run <= _f(r['t']) <= t_end + 1e-6):
            continue
        if not (math.isfinite(_f(r['x'])) and math.isfinite(_f(r['y'])) and abs(_f(r['v'], 0.0)) < 20.0):
            out['gt_blowup_t'] = _f(r['t']) - t_run
            break
        window.append(r)
    if not window:
        out.update(outcome='invalid', score=None, valid=False, problem='no ground truth in the run')
        return out
    p0 = window[0]
    x0, y0, yaw0 = _f(p0['x']), _f(p0['y']), _f(p0['yaw'])
    c0, s0 = math.cos(yaw0), math.sin(yaw0)

    def to_start(r):
        dx, dy = _f(r['x']) - x0, _f(r['y']) - y0
        return c0 * dx + s0 * dy, -s0 * dx + c0 * dy

    along = [to_start(r) for r in window]
    lat = [abs(b) for _, b in along]
    out['start_pose_map_sim'] = {'x': x0, 'y': y0, 'yaw_deg': math.degrees(yaw0)}
    out['progress_m'] = max(a for a, _ in along)
    out['max_lateral_error_m'] = max(lat)
    out['final_lateral_error_m'] = lat[-1]
    yaw_end = _f(window[-1]['yaw'])
    out['final_heading_error_rad'] = abs(math.atan2(math.sin(yaw_end - yaw0), math.cos(yaw_end - yaw0)))
    out['duration_s'] = t_end - t_run
    out['time_to_goal_s'] = (t_end - t_run) if final_state == 'COMPLETE' else None

    # obstacles: true positions (moving ones from /sim/ground_truth/movers, 20 Hz), car-centre
    # (axle midpoint) clearance and footprint contact
    obstacles = inst.get('obstacle_info') or []
    movers: dict = {}
    for row in read_csv(os.path.join(run_dir, 'movers.csv')):
        movers.setdefault(row['id'], []).append((_f(row['t']), _f(row['x']), _f(row['y'])))
    clearances, contact = [], math.inf
    for ob in obstacles:
        r = float(ob['radius_m'])
        track = sorted(movers.get(ob['id'], [])) if ob.get('motion') else []
        times = [p[0] for p in track]

        def where(t, ob=ob, track=track, times=times):
            if not track:
                return ob['x'], ob['y']
            i = bisect.bisect_right(times, t)
            p = track[max(0, min(i, len(track)) - 1)]
            return p[1], p[2]
        dmin, t_min, body = math.inf, None, math.inf
        for row in window:
            x, y, yaw = _f(row['x']), _f(row['y']), _f(row['yaw'])
            ox, oy = where(_f(row['t']))
            cx, cy = x + 0.5 * WHEELBASE_M * math.cos(yaw), y + 0.5 * WHEELBASE_M * math.sin(yaw)
            dc = math.hypot(cx - ox, cy - oy)
            if dc < dmin:
                dmin, t_min = dc, _f(row['t']) - t_run
            body = min(body, footprint_circle_clearance(x, y, yaw, ox, oy, r))
        contact = min(contact, body)
        r_safe = r + CAR_RADIUS_M + SHIPPED_MARGIN_M + CLASS_MARGIN_M.get(ob.get('class'), 0.0)
        clearances.append({'id': ob['id'], 'r': r, 'min_centre_dist_m': dmin, 'r_safe_ref_m': r_safe,
                           'margin_m': dmin - r_safe, 't_min_s': t_min, 'body_clearance_m': body,
                           'level': ob.get('level')})
    out['obstacles'] = clearances
    out['min_margin_m'] = min((c['margin_m'] for c in clearances), default=None)
    out['min_body_clearance_m'] = contact if obstacles else None

    # steering, solver, perception, safety
    drive = [r for r in read_csv(os.path.join(run_dir, 'drive.csv')) if t_run <= _f(r['t']) <= t_end]
    out['steer_sign_changes'] = sign_changes([_f(r['steer'], 0.0) for r in drive])
    out['steer_sat'] = steering_saturation(drive)
    solver = [r for r in read_csv(os.path.join(run_dir, 'solver.csv')) if t_run <= _f(r['t']) <= t_end]
    out['solver_ticks'] = len(solver)
    out['solve_failures'] = sum(1 for r in solver if r['success'] in ('0', 'False'))
    out['max_n_obstacles_seen'] = max((int(_f(r['n_obstacles'], 0)) for r in solver), default=0)
    status = [_f(r['solve_time_ms']) for r in read_csv(os.path.join(run_dir, 'status.csv'))
              if t_run <= _f(r['t']) <= t_end]
    status = [v for v in status if v == v]
    out['solve_time_ms_p95'] = (sorted(status)[int(0.95 * (len(status) - 1))] if status else None)
    out['solve_over_deadline'] = sum(1 for v in status if v > SOLVE_DEADLINE_MS)
    cpu = [r for r in read_csv(os.path.join(run_dir, 'cpu.csv'))
           if r['t'] and t_run <= _f(r['t']) <= t_end and r['mpc_cpu_s']]
    if len(cpu) >= 2 and solver:
        out['mpc_cpu_ms_per_tick'] = 1e3 * (_f(cpu[-1]['mpc_cpu_s']) - _f(cpu[0]['mpc_cpu_s'])) / len(solver)
    else:
        out['mpc_cpu_ms_per_tick'] = None
    obs_rows = [r for r in read_csv(os.path.join(run_dir, 'obstacles.csv'))
                if t_run <= _f(r['t']) <= t_end and r['r']]
    out['perceived_r_max_m'] = max((_f(r['r']) for r in obs_rows), default=None)
    hard = any(r['engaged'] == '1' for r in read_csv(os.path.join(run_dir, 'hard_stop.csv'))
               if t_run <= _f(r['t']) <= t_end)
    bt = [r for r in read_csv(os.path.join(run_dir, 'bt.csv')) if t_run <= _f(r['t']) <= t_end]
    bt_trip = any(r['emergency_trip'] or r['safety_stop_active'] == '1' for r in bt)
    mux = [r for r in read_csv(os.path.join(run_dir, 'mux.csv')) if t_run <= _f(r['t']) <= t_end]
    mux_safety = sorted({r['frame'] for r in mux if r['frame'] in SAFETY_FRAMES})
    events = []
    ev_path = os.path.join(run_dir, 'events.jsonl')
    if os.path.exists(ev_path):
        with open(ev_path, encoding='utf-8') as fh:
            for line in fh:
                try:
                    e = json.loads(line)
                    if t_run <= _f(e.get('t')) <= t_end:
                        events.append(json.loads(e['data']) if e['data'].startswith('{') else e['data'])
                except ValueError:
                    continue
    contact_event = any(isinstance(e, dict) and e.get('event') == 'contact' for e in events)
    out['safety'] = {'hard_stop_engaged': hard, 'bt_trip': bt_trip, 'mux_safety_lanes': mux_safety,
                     'contact_event': contact_event, 'events': events[:20]}

    # stuck: standing still for STUCK_S while running (after the first second)
    stuck, t_still = False, None
    for row in window:
        t = _f(row['t'])
        if t - t_run < 1.0:
            continue
        if abs(_f(row['v'], 0.0)) < STUCK_SPEED:
            t_still = t if t_still is None else t_still
            if t - t_still >= STUCK_S and final_state != 'COMPLETE':
                stuck = True
                break
        else:
            t_still = None

    if (obstacles and contact <= CONTACT_M) or contact_event:
        outcome = 'collision'
    elif hard or bt_trip or mux_safety:
        outcome = 'estop'
    elif final_state == 'COMPLETE':
        outcome = 'success'
    elif stuck:
        outcome = 'stuck'
    else:
        outcome = 'timeout'
    out['outcome'] = outcome
    out['mission_final_state'] = final_state
    out['failure'] = failure_point(out, clearances, drive, t_run)
    out['return'] = return_metrics(window, along, obstacles, inst, (x0, y0, yaw0), t_run)
    if any(ob.get('class') == 'person' for ob in obstacles):
        out['perception'] = perception_metrics(run_dir, window, obstacles, movers, t_run, t_end)
    out.update(score_run(out, dist, speed, bool(obstacles)))
    out['nominal'] = {'distance_m': dist, 'speed_mps': speed, 'timeout_s': timeout}
    return out


RETURN_TOL_M = 0.10
CAMERA_HFOV_DEG = 101.0       # semantic_layer memory hfov (stack_params), the ZED left camera
CAMERA_RANGE_M = (0.8, 5.0)   # its depth window
DETECT_GATE_M = 0.6           # an obstacle disc centre this close to the person = the person


def perception_metrics(run_dir, window, obstacles, movers, t_run, t_end):
    """Phase 4: how the stack's perception sees a person (YOLO -> 3D -> tracker -> obstacle list).

    For every recorded /costmap/semantic_obstacles message, the person counts as DETECTED when a
    disc centre lies within DETECT_GATE_M of its true position at that time (base_link -> map_sim
    through the ground-truth car pose). Reported: when it was first in the camera's view
    (hfov, depth window), first detected, the latency between the two, the share of the in-view
    messages that carried it, the share after it left the view (memory / LiDAR), and the mean
    position error of the matching disc.
    """
    import bisect as _b
    gt_t = [_f(r['t']) for r in window]
    rows = [r for r in read_csv(os.path.join(run_dir, 'obstacles.csv'))
            if t_run <= _f(r['t']) <= t_end]
    msgs = {}
    for r in rows:
        msgs.setdefault(_f(r['t']), []).append(r)
    out = {}
    for ob in obstacles:
        if ob.get('class') != 'person':
            continue
        track = sorted(movers.get(ob['id'], []))
        times = [p[0] for p in track]

        def where(t):
            if not track:
                return ob['x'], ob['y']
            i = _b.bisect_right(times, t)
            p = track[max(0, min(i, len(track)) - 1)]
            return p[1], p[2]
        t_view = t_det = None
        in_view_n = in_view_det = after_n = after_det = 0
        errs = []
        for t in sorted(msgs):
            k = min(len(window) - 1, _b.bisect_left(gt_t, t))
            if k < 0 or not window:
                continue
            cx, cy, cyaw = _f(window[k]['x']), _f(window[k]['y']), _f(window[k]['yaw'])
            px, py = where(t)
            dx, dy = px - cx, py - cy
            bx = math.cos(cyaw) * dx + math.sin(cyaw) * dy
            by = -math.sin(cyaw) * dx + math.cos(cyaw) * dy
            rng = math.hypot(bx, by)
            visible = (bx > 0 and abs(math.degrees(math.atan2(by, bx))) <= CAMERA_HFOV_DEG / 2
                       and CAMERA_RANGE_M[0] <= rng <= CAMERA_RANGE_M[1])
            best = None
            for r in msgs[t]:
                if not r['x']:
                    continue
                ox, oy = _f(r['x']), _f(r['y'])
                gx = cx + math.cos(cyaw) * ox - math.sin(cyaw) * oy
                gy = cy + math.sin(cyaw) * ox + math.cos(cyaw) * oy
                d = math.hypot(gx - px, gy - py)
                best = d if best is None else min(best, d)
            det = best is not None and best <= DETECT_GATE_M
            if visible:
                in_view_n += 1
                in_view_det += int(det)
                t_view = t if t_view is None else t_view
            elif t_view is not None:
                after_n += 1
                after_det += int(det)
            if det:
                errs.append(best)
                t_det = t if t_det is None else t_det
        out[ob['id']] = {
            't_first_in_view_s': None if t_view is None else t_view - t_run,
            't_first_detected_s': None if t_det is None else t_det - t_run,
            'latency_s': (None if t_view is None or t_det is None else t_det - t_view),
            'detected_in_view_frac': (in_view_det / in_view_n) if in_view_n else None,
            'detected_out_of_view_frac': (after_det / after_n) if after_n else None,
            'mean_error_m': statistics.fmean(errs) if errs else None}
    return out




def return_metrics(window, along, obstacles, inst, start, t_run) -> dict | None:
    """The return onto the line after the last obstacle (mpc-dodge phase 2b), start frame.

    x_pass: the furthest obstacle's along-coordinate + its radius (moving ones: where they were
    placed; dodge levels: the level's distance). From the first ground-truth sample past x_pass:
    return_time_s, until |y| stays under RETURN_TOL_M for good (None: not back by the end of the
    mission); overshoot_m, the largest excursion on the side OPPOSITE the swerve (the side of the
    largest |y| before x_pass); crossings of y = 0 after x_pass; final |y|. None without obstacles
    or if the car never got past them."""
    if not obstacles or not window:
        return None
    x0, y0, yaw0 = start
    c0, s0 = math.cos(yaw0), math.sin(yaw0)
    ends = []
    for ob in obstacles:
        dx, dy = float(ob['x']) - x0, float(ob['y']) - y0
        ends.append(c0 * dx + s0 * dy + float(ob['radius_m']))
    x_pass = max(ends)
    k = next((i for i, (a, _b) in enumerate(along) if a >= x_pass), None)
    if k is None:
        return {'x_pass_m': x_pass, 'passed': False}
    before = [b for _a, b in along[:k + 1]]
    side = 1.0 if before and max(before, key=abs) >= 0.0 else -1.0
    after = along[k:]
    ts = [_f(r['t']) for r in window[k:]]
    ys = [b for _a, b in after]
    out_idx = [i for i, y in enumerate(ys) if abs(y) >= RETURN_TOL_M]
    if not out_idx:
        t_back = 0.0
    elif out_idx[-1] == len(ys) - 1:
        t_back = None
    else:
        t_back = ts[out_idx[-1] + 1] - ts[0]
    crossings = sum(1 for a, b in zip(ys, ys[1:]) if (a > 0.0) != (b > 0.0))
    return {'x_pass_m': x_pass, 'passed': True, 'swerve_side': side,
            'lateral_at_pass_m': ys[0], 'return_time_s': t_back,
            'overshoot_m': max(0.0, max(-side * y for y in ys)), 'crossings': crossings,
            'final_lateral_m': ys[-1], 'time_after_pass_s': ts[-1] - ts[0]}


def steering_saturation(drive_rows, t0=None, t1=None) -> dict:
    """Share of /drive commands at the steering limit (|delta|) and at the rate limit (|d delta/dt|)."""
    rows = [(_f(r['t']), _f(r['steer'], 0.0)) for r in drive_rows]
    rows = [(t, d) for t, d in rows if (t0 is None or t >= t0) and (t1 is None or t <= t1)]
    if not rows:
        return {'n': 0, 'delta_frac': None, 'rate_frac': None, 'delta_any': False, 'rate_any': False}
    n_delta = sum(1 for _, d in rows if abs(d) >= SAT_FRAC * DELTA_MAX_RAD)
    rates = []
    for (ta, da), (tb, db) in zip(rows, rows[1:]):
        if tb - ta > 0.02:
            rates.append(abs(db - da) / (tb - ta))
    n_rate = sum(1 for v in rates if v >= SAT_FRAC * DELTA_RATE_MAX)
    return {'n': len(rows), 'delta_frac': n_delta / len(rows),
            'rate_frac': (n_rate / len(rates)) if rates else None,
            'delta_any': n_delta > 0, 'rate_any': n_rate > 0}


def failure_point(run: dict, clearances: list, drive_rows, t_run: float):
    """Where a run that is not good failed: the obstacle (and its level) of the worst margin, that
    margin, the body clearance, when (s after RUNNING), and the steering saturation in the 1.5 s
    before it. None for a good run or a run without obstacles."""
    if not clearances or good_enough(run):
        return None
    worst = min(clearances, key=lambda c: c['margin_m'])
    t = worst.get('t_min_s')
    sat = steering_saturation(drive_rows, t_run + t - 1.5, t_run + t) if t is not None else None
    return {'id': worst['id'], 'level': worst.get('level'), 'margin_m': worst['margin_m'],
            'body_clearance_m': worst.get('body_clearance_m'), 't_s': t, 'steer_sat_before': sat}


def score_run(r: dict, dist: float, speed: float, has_obstacle: bool) -> dict:
    outcome = r['outcome']
    if outcome in HARD_FAILURES:
        return {'score': 0.0, 'score_terms': None}
    if outcome != 'success':
        return {'score': 10.0 * max(0.0, min(1.0, r['progress_m'] / dist)), 'score_terms': None}
    m = r.get('min_margin_m')
    f_margin = 1.0 if (m is None or m >= 0.0) else max(0.0, 1.0 + m / MARGIN_FLOOR_M)
    t_goal = r.get('time_to_goal_s') or math.inf
    f_time = min(1.0, (dist / speed) / t_goal) if t_goal > 0 else 0.0
    n = r['steer_sign_changes']
    f_osc = 1.0 if n <= 1 else 1.0 / (1.0 + (n - 1) / 2.0)
    if has_obstacle:
        f_track = max(0.0, 1.0 - r['final_heading_error_rad'] / HEADING_TOL_RAD)
    else:
        f_track = max(0.0, 1.0 - r['max_lateral_error_m'] / TRACK_TOL_M)
    terms = {'margin': f_margin, 'time': f_time, 'osc': f_osc, 'track': f_track}
    score = 100.0 * (W_MARGIN * f_margin + W_TIME * f_time + W_OSC * f_osc + W_TRACK * f_track)
    return {'score': score, 'score_terms': terms}


def good_enough(r: dict, margin_tol_m: float = 0.03) -> bool:
    """The night loop's per-run bar: success, and (with an obstacle) within margin_tol of R_safe."""
    if r.get('outcome') != 'success':
        return False
    m = r.get('min_margin_m')
    return m is None or m >= -margin_tol_m


def aggregate(runs: list[dict]) -> dict:
    """Per-scenario statistics: mean and std of the score, consistency = mean - std."""
    valid = [r for r in runs if r.get('valid')]
    scores = [r['score'] for r in valid]
    n = len(valid)
    mean = statistics.fmean(scores) if scores else 0.0
    std = statistics.pstdev(scores) if len(scores) > 1 else 0.0
    count = lambda o: sum(1 for r in valid if r['outcome'] == o)  # noqa: E731
    margins = [r['min_margin_m'] for r in valid if r.get('min_margin_m') is not None]
    return {
        'n': n, 'invalid': len(runs) - n,
        'success': count('success'), 'collision': count('collision'), 'estop': count('estop'),
        'stuck': count('stuck'), 'timeout': count('timeout'),
        'good': sum(1 for r in valid if good_enough(r)),
        'score_mean': mean, 'score_std': std, 'score_agg': mean - std,
        'min_margin_m': min(margins) if margins else None,
        'max_lateral_m': max((r['max_lateral_error_m'] for r in valid), default=None),
        'max_sign_changes': max((r['steer_sign_changes'] for r in valid), default=None),
        'solve_failures': sum(r.get('solve_failures', 0) for r in valid),
        'time_to_goal_mean_s': statistics.fmean([r['time_to_goal_s'] for r in valid
                                                 if r.get('time_to_goal_s')] or [math.nan]),
    }
