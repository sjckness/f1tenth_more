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
    window = [r for r in gt if t_run <= _f(r['t']) <= t_end + 1e-6]
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
        dmin = math.inf
        for row in window:
            x, y, yaw = _f(row['x']), _f(row['y']), _f(row['yaw'])
            ox, oy = where(_f(row['t']))
            cx, cy = x + 0.5 * WHEELBASE_M * math.cos(yaw), y + 0.5 * WHEELBASE_M * math.sin(yaw)
            dmin = min(dmin, math.hypot(cx - ox, cy - oy))
            contact = min(contact, footprint_circle_clearance(x, y, yaw, ox, oy, r))
        r_safe = r + CAR_RADIUS_M + SHIPPED_MARGIN_M
        clearances.append({'id': ob['id'], 'r': r, 'min_centre_dist_m': dmin, 'r_safe_ref_m': r_safe,
                           'margin_m': dmin - r_safe})
    out['obstacles'] = clearances
    out['min_margin_m'] = min((c['margin_m'] for c in clearances), default=None)
    out['min_body_clearance_m'] = contact if obstacles else None

    # steering, solver, perception, safety
    drive = [r for r in read_csv(os.path.join(run_dir, 'drive.csv')) if t_run <= _f(r['t']) <= t_end]
    out['steer_sign_changes'] = sign_changes([_f(r['steer'], 0.0) for r in drive])
    solver = [r for r in read_csv(os.path.join(run_dir, 'solver.csv')) if t_run <= _f(r['t']) <= t_end]
    out['solver_ticks'] = len(solver)
    out['solve_failures'] = sum(1 for r in solver if r['success'] in ('0', 'False'))
    out['max_n_obstacles_seen'] = max((int(_f(r['n_obstacles'], 0)) for r in solver), default=0)
    status = [_f(r['solve_time_ms']) for r in read_csv(os.path.join(run_dir, 'status.csv'))
              if t_run <= _f(r['t']) <= t_end]
    status = [v for v in status if v == v]
    out['solve_time_ms_p95'] = (sorted(status)[int(0.95 * (len(status) - 1))] if status else None)
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
    out.update(score_run(out, dist, speed, bool(obstacles)))
    out['nominal'] = {'distance_m': dist, 'speed_mps': speed, 'timeout_s': timeout}
    return out


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
