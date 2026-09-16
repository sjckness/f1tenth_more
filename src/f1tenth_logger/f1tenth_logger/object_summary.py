"""Run-summary sections for go_to_object moves and the /drive clamp. ROS-free.

mission_extract.read_bag collects the raw streams (/mpc/object_status,
/mpc/goal_object, /mpc/goal_object_end, /mission/move_outcome,
/mpc/drive_clamp); this turns them into the compact per-move record the
extract's meta row carries, so a run can be judged without replaying it.

Every sample is a dict keyed by field name (see mission_extract.read_bag), and
each stream is anything with parallel `t` and `v` lists (mission_render.Stream).
"""

import math

__all__ = ['summarize_drive_clamp', 'summarize_object_approach']


def _finite(value):
    return value if isinstance(value, (int, float)) and math.isfinite(value) else None


def summarize_object_approach(object_status, goal_object, goal_object_end, move_outcome):
    """Per ObjectGoal move_id: goals sent, the last status, flag counts, the end, the outcome."""
    moves = {}

    def entry(move_id):
        return moves.setdefault(move_id, {
            'goals': 0, 'first_goal_t': None, 'last_goal_t': None, 'end_t': None,
            'status_samples': 0, 'final': None,
            'min_r': None, 'max_target_age_s': None,
            'inside_turn_radius_samples': 0, 'target_behind_samples': 0,
            'target_behind_terminal': False, 'goal_watchdog_samples': 0,
            'target_stale_samples': 0, 'outcome': None, 'stop_reason': None,
            'final_gap_or_range': None, 'arrival_bearing_error_deg': None,
            'track_gap_m': None,
        })

    for t, goal in zip(goal_object.t, goal_object.v):
        e = entry(goal['move_id'])
        e['goals'] += 1
        e['first_goal_t'] = t if e['first_goal_t'] is None else e['first_goal_t']
        e['last_goal_t'] = t

    for t, status in zip(object_status.t, object_status.v):
        if not status['move_id']:
            continue
        e = entry(status['move_id'])
        e['status_samples'] += 1
        r = _finite(status['r'])
        if r is not None:
            e['min_r'] = r if e['min_r'] is None else min(e['min_r'], r)
        age = _finite(status['target_age_s'])
        if age is not None:
            e['max_target_age_s'] = (age if e['max_target_age_s'] is None
                                     else max(e['max_target_age_s'], age))
        e['inside_turn_radius_samples'] += int(bool(status['inside_turn_radius']))
        e['target_behind_samples'] += int(bool(status['target_behind']))
        e['target_behind_terminal'] |= bool(status['target_behind_terminal'])
        e['goal_watchdog_samples'] += int(bool(status['goal_watchdog']))
        e['target_stale_samples'] += int(bool(status['target_stale']))
        e['final'] = {'t': t, 'r': r, 'alpha': _finite(status['alpha']),
                      'target_age_s': age, 'psi_c': _finite(status['psi_c'])}

    for t, end in zip(goal_object_end.t, goal_object_end.v):
        entry(end['move_id'])['end_t'] = t

    for _t, outcome in zip(move_outcome.t, move_outcome.v):
        if outcome.get('move_type') != 'go_to_object' or not outcome.get('wire_move_id'):
            continue
        e = entry(outcome['wire_move_id'])
        e['outcome'] = outcome.get('outcome') or None
        e['stop_reason'] = outcome.get('stop_reason') or None
        e['final_gap_or_range'] = _finite(outcome.get('actual'))
        e['arrival_bearing_error_deg'] = _finite(outcome.get('arrival_bearing_error_deg'))
        e['track_gap_m'] = _finite(outcome.get('track_gap_m'))
    return moves


def summarize_drive_clamp(drive_clamp):
    """How often /drive was clamped, and the extremes that were asked for."""
    requested = [_finite(v['requested_speed']) for v in drive_clamp.v]
    requested = [r for r in requested if r is not None]
    return {
        'events': len(drive_clamp.v),
        'max_requested': max(requested) if requested else None,
        'min_requested': min(requested) if requested else None,
        'first_t': drive_clamp.t[0] if drive_clamp.t else None,
    }
