"""
Missions planned by the LLM planner, stored, then loaded and started on request.

The planner runs plan-only (llm_planner_node --dry-run): it prints the
mission and one `RESULT {json}` line per command, whose `mission` field is
the full mission document, and it never loads or starts anything. When
`planner-log` reads such a RESULT line, the mission is stored here under an
id (timestamp + short hash), exactly as the planner made it, once per RESULT
line. `mission-load <id>` loads THAT file; nothing is ever regenerated.

Layout under <state_dir>/missions/:
    <id>.mission.json   the mission document, the file /mission/load_mission reads
    <id>.json           what the bot shows: when, the summary, the RESULT fields
    index.json          RESULT line byte offset in the planner log -> id
    current.json        the mission f1tenth_ctl loaded / started last
"""

import hashlib
import json
import os
import time
from typing import List, Optional

from .config import Settings

ID_STATES = ('IDLE', 'LOADED', 'RUNNING', 'HOLDING', 'COMPLETE', 'ABORTED')
# /mission/status state -> what mission-status says
STATE_NAMES = {'IDLE': 'idle', 'LOADED': 'loaded', 'RUNNING': 'running', 'HOLDING': 'running',
               'COMPLETE': 'finished', 'ABORTED': 'aborted'}
ACTIVE = ('RUNNING', 'HOLDING')


def store_dir(settings: Settings) -> str:
    return os.path.join(settings.state_dir, 'missions')


def mission_path(settings: Settings, mission_id: str) -> str:
    return os.path.join(store_dir(settings), f'{mission_id}.mission.json')


def _read_json(path: str, default=None):
    try:
        with open(path) as f:
            return json.load(f)
    except (OSError, ValueError):
        return default


def _write_json(path: str, data) -> None:
    os.makedirs(os.path.dirname(path), exist_ok=True)
    tmp = path + '.tmp'
    with open(tmp, 'w') as f:
        json.dump(data, f, indent=2, ensure_ascii=False)
        f.write('\n')
    os.replace(tmp, path)


def _num(value) -> Optional[float]:
    try:
        return float(value)
    except (TypeError, ValueError):
        return None


def summarize(mission: dict) -> dict:
    """
    Summarize what matters for safety, from the mission document itself (pure).

    Schema: f1tenth_behavior/mission/mission_config.py. A move is a `drive`
    (mode straight | wall_turn, speed, turn), a `go_to_object` (target class,
    gap, speed), or an older goal_distance / goal_pose / turn; each ends on its
    stop_condition or its timeout_sec. There is no reverse in this schema.
    """
    moves = mission.get('moves') or []
    out_moves, speeds, targets = [], [], []
    distance, open_ended, duration = 0.0, False, 0.0
    for i, move in enumerate(moves):
        drive = move.get('drive') or {}
        target = move.get('go_to_object')
        stop = move.get('stop_condition') or {}
        if target is not None:
            gap = target.get('gap_m')
            action = (f"go to the nearest {target.get('target_class')}, "
                      + (f'stop {gap} m before it' if gap is not None
                         else 'stop at the default gap'))
            speed = _num(target.get('speed'))
            targets.append(target.get('target_class'))
        elif drive.get('mode') == 'wall_turn' and (_num(drive.get('turn_mag_deg')) or 0) > 0:
            side = 'left' if (_num(drive.get('turn_sign')) or 0) > 0 else 'right'
            action = f"turn {side} {drive.get('turn_mag_deg')} deg"
            speed = _num(drive.get('speed'))
        elif drive:
            action = 'straight'
            speed = _num(drive.get('speed'))
        elif 'goal_distance' in move:
            action = f"drive {move.get('goal_distance')} m"
            speed = _num(move.get('vdes'))
        elif 'turn' in move:
            action = 'turn'
            speed = None
        else:
            action = 'move'
            speed = None
        if speed:
            speeds.append(speed)
        until = stop.get('type', '?')
        for key in ('distance', 'value', 'seconds', 'degrees'):
            if key in stop:
                until += f' {stop[key]}'
                break
        if stop.get('type') == 'distance_reached' and _num(stop.get('distance')) is not None:
            distance += _num(stop.get('distance'))
        elif 'goal_distance' in move and _num(move.get('goal_distance')) is not None:
            distance += _num(move.get('goal_distance'))
        else:
            open_ended = True
        duration += _num(move.get('timeout_sec')) or 0.0
        out_moves.append({'i': i, 'id': move.get('id'), 'action': action,
                          'speed_mps': speed, 'until': until,
                          'timeout_s': _num(move.get('timeout_sec')),
                          'on_timeout': move.get('on_timeout'),
                          'terminal': bool(move.get('terminal'))})
    return {'mission_id': mission.get('mission_id'),
            'schema_version': mission.get('schema_version'),
            'n_moves': len(moves), 'moves': out_moves,
            'max_speed_mps': max(speeds) if speeds else None,
            'speed_note': None if speeds and len(speeds) == len(moves)
            else 'some moves use the controller default speed',
            'targets': targets, 'reverse': False,
            'distance_m': None if open_ended or not moves else round(distance, 2),
            'max_duration_s': round(duration, 1) if duration else None}


def store_result(settings: Settings, result: dict, log_offset: int,
                 now: float = None) -> Optional[dict]:
    """Store the mission of one RESULT line; return its record (or the existing one)."""
    mission = result.get('mission')
    if result.get('status') != 'ok' or not isinstance(mission, dict):
        return None
    index_path = os.path.join(store_dir(settings), 'index.json')
    index = _read_json(index_path, {})
    known = index.get(str(log_offset))
    if known:
        rec = _read_json(os.path.join(store_dir(settings), f'{known}.json'))
        if rec:
            return rec
    now = time.time() if now is None else now
    body = json.dumps(mission, indent=2, ensure_ascii=False) + '\n'
    digest = hashlib.sha1(body.encode()).hexdigest()
    base = time.strftime('%Y%m%dT%H%M%S', time.localtime(now)) + '-' + digest[:6]
    os.makedirs(store_dir(settings), exist_ok=True)
    mid, n = base, 1
    # The same plan twice in one second (another RESULT line): its own id.
    while os.path.exists(os.path.join(store_dir(settings), f'{mid}.json')):
        n += 1
        mid = f'{base}-{n}'
    with open(mission_path(settings, mid), 'w') as f:
        f.write(body)
    rec = {'id': mid, 'stored_at': now, 'sha1': digest, 'log_offset': log_offset,
           'path': mission_path(settings, mid),
           'mission_id': mission.get('mission_id'),
           'unsupported': list(result.get('unsupported') or []),
           'notes': list(result.get('notes') or []),
           'plan_seconds': result.get('plan_seconds'),
           'summary': summarize(mission)}
    _write_json(os.path.join(store_dir(settings), f'{mid}.json'), rec)
    index[str(log_offset)] = mid
    _write_json(index_path, index)
    return rec


def get(settings: Settings, mid: str) -> Optional[dict]:
    if not mid or os.sep in mid or mid.startswith('.'):
        return None
    rec = _read_json(os.path.join(store_dir(settings), f'{mid}.json'))
    if rec is None:
        return None
    rec['mission'] = _read_json(mission_path(settings, mid))
    return rec


def recent(settings: Settings, n: int = 10) -> List[dict]:
    d = store_dir(settings)
    try:
        names = [x for x in os.listdir(d) if x.endswith('.json')
                 and not x.endswith('.mission.json') and x not in ('index.json', 'current.json')]
    except OSError:
        return []
    recs = [_read_json(os.path.join(d, x)) for x in names]
    recs = [r for r in recs if r]
    recs.sort(key=lambda r: r.get('stored_at', 0), reverse=True)
    return recs[:n]


def current(settings: Settings) -> dict:
    return _read_json(os.path.join(store_dir(settings), 'current.json'), {}) or {}


def set_current(settings: Settings, **fields) -> dict:
    data = current(settings)
    data.update(fields)
    _write_json(os.path.join(store_dir(settings), 'current.json'), data)
    return data


def describe_state(settings: Settings, ros_state: Optional[str], json_path: Optional[str],
                   now: float = None, countdown_s: float = 15.0) -> dict:
    """mission-status from /mission/status and current.json (pure apart from the read)."""
    now = time.time() if now is None else now
    cur = current(settings)
    ours = bool(json_path) and json_path == cur.get('path')
    state = STATE_NAMES.get(ros_state or '', 'unknown' if ros_state is None else ros_state)
    started = cur.get('started_at') if ours else None
    if state == 'loaded' and started and now - started <= countdown_s + 5.0:
        state = 'starting'
    out = {'state': state, 'ros_state': ros_state, 'json_path': json_path,
           'id': cur.get('id') if ours else None}
    if started:
        out['elapsed_s'] = round(now - started, 1)
    return out
