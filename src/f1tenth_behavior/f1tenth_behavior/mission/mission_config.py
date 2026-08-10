"""Mission config schema, parser, and validator.

See f1tenth_behavior/README.md's "Mission config" section for the authoritative
field-by-field table this mirrors -- keep the two in sync if either changes.

Deliberately has no rclpy/ROS dependency at all: this is pure JSON-in,
dataclasses-out, so it's usable/testable standalone. mission/loader.py is the
ROS-facing wrapper (parameter + topic) that calls load_mission_file() below.
"""

import json
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, List, Optional

STOP_CONDITION_TYPES = {
    'distance_reached', 'goal_reached', 'time_elapsed', 'object_seen',
    'object_cleared', 'obstacle_distance_below', 'manual',
}
ON_OBJECT_ACTIONS = {
    'stop_and_hold', 'reduce_speed', 'reduce_speed_for', 'abort_mission',
    'skip_to_move', 'log_only',
}
ON_TIMEOUT_VALUES = {'abort', 'skip'}

# stop_condition.type values that are explicitly out of scope for this pass
# (manual advance -- no mechanism planned). condition_eval.evaluate() returns
# None for these; every consumer of that function has to treat None as "never
# satisfies on its own", not crash. goal_reached is NOT a stub anymore -- since
# mpc_corr gained a /mpc/goal_pose input (and publishes /mpc/goal_reached on
# arrival for both its distance- and pose-mode goals), CheckStopCondition
# tracks that signal for real; see condition_eval.evaluate()'s own goal_reached
# branch and CheckStopCondition's docstring. It still evaluates to None when
# used as HandleObjectAction's resume_condition, though (that caller doesn't
# track the live signal) -- flagged there, not silent.
STUB_STOP_CONDITION_TYPES = {'manual'}
# on_object actions whose control effect depends on a vdes-override mechanism
# that doesn't exist yet.
STUB_ON_OBJECT_ACTIONS = {'reduce_speed', 'reduce_speed_for'}


class MissionConfigError(ValueError):
    """Raised on any schema violation. Callers must treat this (and, from
    load_mission_file(), OSError/json.JSONDecodeError too) as "reject the whole
    mission" -- never install a partially-parsed MissionConfig over whatever
    mission was already loaded and running.
    """


@dataclass(frozen=True)
class GoalPose:
    x: float
    y: float
    yaw: float


@dataclass(frozen=True)
class StopCondition:
    type: str
    # Every extra field from the JSON verbatim, rather than one dataclass per
    # type -- the field set genuinely differs per type (see the config table),
    # and a 7-way tagged union of near-empty dataclasses wouldn't be any clearer
    # than each evaluator reading params[...] for the fields its own type needs.
    params: Dict[str, object] = field(default_factory=dict)


@dataclass(frozen=True)
class OnObjectAction:
    # JSON key is "class" -- "class" itself is a reserved word, hence "cls".
    cls: str
    action: str
    params: Dict[str, object] = field(default_factory=dict)


@dataclass(frozen=True)
class Move:
    id: str
    stop_condition: StopCondition
    timeout_sec: float
    goal_distance: Optional[float] = None
    goal_pose: Optional[GoalPose] = None
    vdes: Optional[float] = None
    on_object: List[OnObjectAction] = field(default_factory=list)
    on_timeout: str = 'abort'


@dataclass(frozen=True)
class MissionConfig:
    mission_id: str
    moves: List[Move]

    def index_of(self, move_id: str) -> Optional[int]:
        for i, m in enumerate(self.moves):
            if m.id == move_id:
                return i
        return None


def _require(cond: bool, message: str) -> None:
    if not cond:
        raise MissionConfigError(message)


def _parse_stop_condition(raw: object, where: str) -> StopCondition:
    _require(isinstance(raw, dict), f'{where}: stop_condition must be an object')
    t = raw.get('type')
    _require(
        t in STOP_CONDITION_TYPES,
        f'{where}: stop_condition.type={t!r} not in {sorted(STOP_CONDITION_TYPES)}',
    )
    params = {k: v for k, v in raw.items() if k != 'type'}

    if t == 'time_elapsed':
        _require('duration_sec' in params, f'{where}: time_elapsed requires duration_sec')
    elif t == 'object_seen':
        _require('class' in params, f'{where}: object_seen requires class')
    elif t == 'object_cleared':
        _require('class' in params, f'{where}: object_cleared requires class')
    elif t == 'obstacle_distance_below':
        _require('distance' in params, f'{where}: obstacle_distance_below requires distance')

    return StopCondition(type=t, params=params)


def _parse_on_object(raw: object, where: str) -> OnObjectAction:
    _require(isinstance(raw, dict), f'{where}: on_object entry must be an object')
    cls = raw.get('class')
    _require(isinstance(cls, str) and cls, f'{where}: on_object entry requires a non-empty class')
    action = raw.get('action')
    _require(
        action in ON_OBJECT_ACTIONS,
        f'{where}: on_object action={action!r} not in {sorted(ON_OBJECT_ACTIONS)}',
    )
    params = {k: v for k, v in raw.items() if k not in ('class', 'action')}

    if action == 'reduce_speed':
        _require('factor' in params, f'{where}: reduce_speed requires factor')
    elif action == 'reduce_speed_for':
        _require(
            'factor' in params and 'duration_sec' in params,
            f'{where}: reduce_speed_for requires factor and duration_sec',
        )
    elif action == 'abort_mission':
        _require('reason' in params, f'{where}: abort_mission requires reason')
    elif action == 'skip_to_move':
        _require('move_id' in params, f'{where}: skip_to_move requires move_id')
    elif action == 'log_only':
        _require('message' in params, f'{where}: log_only requires message')

    if 'resume_condition' in params and params['resume_condition'] is not None:
        params = dict(params)
        params['resume_condition'] = _parse_stop_condition(
            params['resume_condition'], f'{where} (on_object {cls}/{action}).resume_condition')

    return OnObjectAction(cls=cls, action=action, params=params)


def _parse_move(raw: object, where: str) -> Move:
    _require(isinstance(raw, dict), f'{where}: move must be an object')
    move_id = raw.get('id')
    _require(isinstance(move_id, str) and move_id, f'{where}: move requires a non-empty id')
    where = f'{where} (id={move_id})'

    has_distance = raw.get('goal_distance') is not None
    has_pose = raw.get('goal_pose') is not None
    _require(
        has_distance != has_pose,
        f'{where}: exactly one of goal_distance/goal_pose is required',
    )

    goal_distance = float(raw['goal_distance']) if has_distance else None
    goal_pose = None
    if has_pose:
        gp = raw['goal_pose']
        _require(
            isinstance(gp, dict) and {'x', 'y', 'yaw'} <= gp.keys(),
            f'{where}: goal_pose requires x, y, yaw',
        )
        goal_pose = GoalPose(x=float(gp['x']), y=float(gp['y']), yaw=float(gp['yaw']))

    _require('stop_condition' in raw, f'{where}: stop_condition is required')
    stop_condition = _parse_stop_condition(raw['stop_condition'], where)

    if stop_condition.type == 'distance_reached' and 'distance' not in stop_condition.params:
        _require(
            has_distance,
            f'{where}: distance_reached with no explicit distance needs goal_distance set',
        )

    _require('timeout_sec' in raw, f'{where}: timeout_sec is required')
    timeout_sec = float(raw['timeout_sec'])
    _require(timeout_sec > 0.0, f'{where}: timeout_sec must be > 0')

    on_timeout = raw.get('on_timeout', 'abort')
    _require(
        on_timeout in ON_TIMEOUT_VALUES,
        f'{where}: on_timeout={on_timeout!r} not in {sorted(ON_TIMEOUT_VALUES)}',
    )

    vdes = float(raw['vdes']) if raw.get('vdes') is not None else None

    on_object_raw = raw.get('on_object', [])
    _require(isinstance(on_object_raw, list), f'{where}: on_object must be a list')
    on_object = [_parse_on_object(o, where) for o in on_object_raw]

    return Move(
        id=move_id, goal_distance=goal_distance, goal_pose=goal_pose, vdes=vdes,
        stop_condition=stop_condition, on_object=on_object,
        timeout_sec=timeout_sec, on_timeout=on_timeout,
    )


def parse_mission(raw: object) -> MissionConfig:
    """Parse+validate an already-json.loads()'d mission dict into a MissionConfig.
    Raises MissionConfigError with a specific, actionable message on the first
    violation found. Never returns a partially-built MissionConfig -- either the
    whole thing validates or nothing is returned at all.
    """
    _require(isinstance(raw, dict), 'mission root must be a JSON object')
    mission_id = raw.get('mission_id')
    _require(isinstance(mission_id, str) and mission_id, 'mission_id is required')

    moves_raw = raw.get('moves')
    _require(isinstance(moves_raw, list) and len(moves_raw) > 0, 'moves must be a non-empty list')

    moves = [_parse_move(m, f'moves[{i}]') for i, m in enumerate(moves_raw)]

    ids = [m.id for m in moves]
    dupes = sorted({i for i in ids if ids.count(i) > 1})
    _require(not dupes, f'duplicate move id(s): {dupes}')

    id_set = set(ids)
    for m in moves:
        for oo in m.on_object:
            if oo.action == 'skip_to_move':
                target = oo.params['move_id']
                _require(
                    target in id_set,
                    f'move {m.id!r}: skip_to_move target {target!r} is not a known move id',
                )

    return MissionConfig(mission_id=mission_id, moves=moves)


def load_mission_file(path: str) -> MissionConfig:
    """Read and parse a mission JSON file from disk.

    Raises MissionConfigError (schema violation), OSError (file not found/
    unreadable), or json.JSONDecodeError (malformed JSON) -- callers must catch
    all three the same way: log clearly, leave whatever mission was already
    loaded untouched, do not install a partial result.
    """
    text = Path(path).read_text(encoding='utf-8')
    raw = json.loads(text)
    return parse_mission(raw)
