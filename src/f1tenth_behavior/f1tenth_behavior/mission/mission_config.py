"""Mission config schema, parser, and validator.

See f1tenth_behavior/README.md's "Mission config" section for the authoritative
field-by-field table this mirrors -- keep the two in sync if either changes.

Deliberately has no rclpy/ROS dependency at all: this is pure JSON-in,
dataclasses-out, so it's usable/testable standalone. mission/loader.py is the
ROS-facing wrapper (parameter + topic) that calls load_mission_file() below.
"""

import json
import math
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, List, Optional

STOP_CONDITION_TYPES = {
    'distance_reached', 'goal_reached', 'time_elapsed', 'object_seen',
    'object_cleared', 'obstacle_distance_below', 'front_clearance',
    'orientation_delta', 'manual',
}
ON_OBJECT_ACTIONS = {
    'stop_and_hold', 'reduce_speed', 'reduce_speed_for', 'abort_mission',
    'skip_to_move', 'log_only',
}
# 'stop' added alongside the "turn" step type / schema_version 2.0 -- a third
# on_timeout behavior distinct from the two that already existed: neither
# aborts the whole mission (like 'abort') nor jumps to the next move (like
# 'skip'), it just halts the car in place and leaves the mission sitting on
# the current (now-timed-out) move indefinitely. See CheckStopCondition's own
# timeout-handling branch for the implementation.
ON_TIMEOUT_VALUES = {'abort', 'skip', 'stop'}
# Only one reference frame for a turn step's heading tracking is supported
# today (see TurnSpec.reference) -- kept as a real field, not hardcoded,
# specifically so a future reference (e.g. a fused/filtered heading source
# distinct from whichever /odom-equivalent topic localization_source
# currently selects) can be added without a schema change.
VALID_TURN_REFERENCES = {'odometry_orientation'}

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
#
# front_clearance is likewise NOT a stub -- added alongside f1tenth_perception's
# wall_detector_node (RANSAC plane segmentation), which was the FIRST real
# backing sensor source for it: this type did not exist in this schema at all
# before that integration (there was no prior stub, real or otherwise, for the
# name "front_clearance" specifically -- confirmed by reading this set before
# adding it). wall_detector_node has since been retired in favor of
# f1tenth_costmap's costmap_boundary_node (see the dual-EKF + costmap-
# derived-MPC-boundaries pass) -- a source swap, not a second integration;
# front_clearance itself has been a real (non-stub) type continuously since
# the original addition. See CheckStopCondition's own docstring for how the
# live /costmap/front_clearance value reaches condition_eval.evaluate().
#
# orientation_delta (schema_version 2.0, alongside the "turn" step type) is
# also real, not a stub -- backed by CheckStopCondition's own /odom
# subscription (already owned there for distance_reached; extended to also
# extract yaw) rather than a new sensor. Turn-exclusive by construction (see
# _parse_move's own validation below): using it as any other step's
# stop_condition, or as a resume_condition, is rejected/never-satisfies
# respectively -- see condition_eval.py's own orientation_delta branch for why
# the resume_condition case specifically returns None rather than raising.
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
class TurnSpec:
    """A "turn" step's own fields -- see Move.turn. Sibling to (not nesting)
    Move's own stop_condition, same shape convention goal_distance/goal_pose
    already use (stop_condition always lives at the Move level, not inside a
    per-type sub-object) -- the task description that introduced this showed
    stop_condition nested under "turn" in its own field list; kept it a
    sibling instead for schema consistency with every other step type."""
    heading_delta_deg: float  # signed: + = left/CCW, - = right/CW (right-hand rule)
    speed: float               # linear speed while turning [m/s]
    steering: str               # "full_lock" or "partial:<deg>" -- format validated
                                 # at load time, see _parse_turn_spec below
    reference: str = 'odometry_orientation'  # only supported value today, see
                                              # VALID_TURN_REFERENCES


@dataclass(frozen=True)
class Move:
    id: str
    stop_condition: StopCondition
    goal_distance: Optional[float] = None
    goal_pose: Optional[GoalPose] = None
    turn: Optional[TurnSpec] = None
    vdes: Optional[float] = None
    on_object: List[OnObjectAction] = field(default_factory=list)
    # Optional as of schema_version 2.0 (previously always required) --
    # None means no per-move timeout is enforced (CheckStopCondition's own
    # timeout branch skips the check entirely rather than treating None as
    # "already expired"). Existing 1.x missions always set this explicitly,
    # so relaxing the requirement doesn't change how any of them parse or
    # behave -- see parse_mission()'s own backward-compat note.
    timeout_sec: Optional[float] = None
    on_timeout: str = 'abort'


@dataclass(frozen=True)
class MissionConfig:
    mission_id: str
    moves: List[Move]
    # "1.0" (the implicit, pre-existing behavior) if the JSON omits this field
    # entirely -- see parse_mission(). "2.0" is the first version that knows
    # about "turn" steps / orientation_delta / optional timeout_sec, but
    # nothing here is actually gated on this value (a schema_version: "1.0"
    # mission that happened to use a "turn" step would still parse and run
    # exactly the same as a "2.0" one) -- it's informational bookkeeping, not
    # an enforced feature flag, since none of the new fields are structurally
    # incompatible with 1.x missions that simply don't use them.
    schema_version: str = '1.0'

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
    elif t == 'front_clearance':
        _require('distance' in params, f'{where}: front_clearance requires distance')
    elif t == 'orientation_delta':
        _require('value' in params, f'{where}: orientation_delta requires value')
        _require(
            isinstance(params['value'], (int, float)) and not isinstance(params['value'], bool),
            f'{where}: orientation_delta value must be numeric',
        )
        # The turn-exclusivity check (this type may only appear on a "turn"
        # step) and the heading_delta_deg/value consistency check both need
        # the sibling `turn` field, which isn't visible from here -- see
        # _parse_move's own validation, after both this and the turn spec
        # have been parsed.

    return StopCondition(type=t, params=params)


def _parse_turn_spec(raw: object, where: str) -> TurnSpec:
    _require(isinstance(raw, dict), f'{where}: turn must be an object')

    heading_delta_deg = raw.get('heading_delta_deg')
    _require(
        isinstance(heading_delta_deg, (int, float)) and not isinstance(heading_delta_deg, bool),
        f'{where}: turn.heading_delta_deg is required and must be numeric',
    )

    speed = raw.get('speed')
    _require(
        isinstance(speed, (int, float)) and not isinstance(speed, bool) and speed > 0.0,
        f'{where}: turn.speed is required and must be a positive number',
    )

    steering = raw.get('steering')
    _require(
        isinstance(steering, str) and _is_valid_steering(steering),
        f'{where}: turn.steering must be "full_lock" or "partial:<deg>" (got {steering!r})',
    )

    reference = raw.get('reference', 'odometry_orientation')
    _require(
        reference in VALID_TURN_REFERENCES,
        f'{where}: turn.reference={reference!r} not in {sorted(VALID_TURN_REFERENCES)}',
    )

    return TurnSpec(
        heading_delta_deg=float(heading_delta_deg), speed=float(speed),
        steering=steering, reference=reference,
    )


def _is_valid_steering(value: str) -> bool:
    """"full_lock", or "partial:<deg>" where <deg> parses as a real number
    (sign allowed, e.g. "partial:15" or "partial:15.5") -- mirrors the exact
    format mpc_corr.py's own goal_turn_callback expects to parse back out on
    the wire (see that method's docstring), so a mission that passes this
    check is guaranteed not to hit its "unparseable steering" fallback later.
    """
    if value == 'full_lock':
        return True
    if not value.startswith('partial:'):
        return False
    try:
        float(value.split(':', 1)[1])
    except ValueError:
        return False
    return True


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
    has_turn = raw.get('turn') is not None
    _require(
        sum((has_distance, has_pose, has_turn)) == 1,
        f'{where}: exactly one of goal_distance/goal_pose/turn is required',
    )

    goal_distance = None
    if has_distance:
        gd = raw['goal_distance']
        _require(
            isinstance(gd, (int, float)) and not isinstance(gd, bool),
            f'{where}: goal_distance must be numeric (got {type(gd).__name__})',
        )
        goal_distance = float(gd)
    goal_pose = None
    if has_pose:
        gp = raw['goal_pose']
        _require(
            isinstance(gp, dict) and {'x', 'y', 'yaw'} <= gp.keys(),
            f'{where}: goal_pose requires x, y, yaw',
        )
        goal_pose = GoalPose(x=float(gp['x']), y=float(gp['y']), yaw=float(gp['yaw']))
    turn = _parse_turn_spec(raw['turn'], where) if has_turn else None

    _require('stop_condition' in raw, f'{where}: stop_condition is required')
    stop_condition = _parse_stop_condition(raw['stop_condition'], where)

    if stop_condition.type == 'distance_reached' and 'distance' not in stop_condition.params:
        _require(
            has_distance,
            f'{where}: distance_reached with no explicit distance needs goal_distance set',
        )

    # orientation_delta is turn-exclusive (see STUB_STOP_CONDITION_TYPES'
    # comment on this type) -- reject it on any step that isn't a turn...
    if stop_condition.type == 'orientation_delta':
        _require(
            has_turn,
            f'{where}: orientation_delta stop_condition is only valid on a "turn" step',
        )
        # ...and, on a turn step, its magnitude must agree with the turn's own
        # heading_delta_deg -- validated here (load time) rather than trusted
        # separately at evaluation time, so a mismatched pair can never
        # silently mean two different things (e.g. the turn drives 90 deg but
        # the mission advances at 45 deg because someone edited one field and
        # not the other).
        _require(
            math.isclose(
                abs(float(stop_condition.params['value'])), abs(turn.heading_delta_deg),
                abs_tol=1e-6,
            ),
            f'{where}: orientation_delta value ({stop_condition.params["value"]}) must equal '
            f'abs(turn.heading_delta_deg) ({abs(turn.heading_delta_deg)})',
        )

    timeout_sec = None
    if raw.get('timeout_sec') is not None:
        ts = raw['timeout_sec']
        _require(
            isinstance(ts, (int, float)) and not isinstance(ts, bool),
            f'{where}: timeout_sec must be numeric (got {type(ts).__name__})',
        )
        timeout_sec = float(ts)
        _require(timeout_sec > 0.0, f'{where}: timeout_sec must be > 0')

    on_timeout_explicit = 'on_timeout' in raw
    on_timeout = raw.get('on_timeout', 'abort')
    _require(
        on_timeout in ON_TIMEOUT_VALUES,
        f'{where}: on_timeout={on_timeout!r} not in {sorted(ON_TIMEOUT_VALUES)}',
    )
    # timeout_sec is optional (see Move.timeout_sec's own comment) precisely
    # so a move can opt out of the safety cap entirely -- an explicit
    # on_timeout with nothing for it to ever apply to is almost always a
    # mistake (a field the author meant to pair with a timeout_sec that got
    # dropped/renamed), so this is rejected at load time rather than silently
    # ignored. The reverse (timeout_sec with no explicit on_timeout) is NOT
    # an error -- on_timeout's own default ('abort') already covers that
    # case exactly as it did before schema_version 2.0.
    _require(
        not (on_timeout_explicit and timeout_sec is None),
        f'{where}: on_timeout is set but timeout_sec is not -- on_timeout has nothing to '
        'apply to (either add timeout_sec or remove on_timeout)',
    )

    vdes = None
    if raw.get('vdes') is not None:
        vd = raw['vdes']
        _require(
            isinstance(vd, (int, float)) and not isinstance(vd, bool),
            f'{where}: vdes must be numeric (got {type(vd).__name__})',
        )
        vdes = float(vd)

    on_object_raw = raw.get('on_object', [])
    _require(isinstance(on_object_raw, list), f'{where}: on_object must be a list')
    on_object = [_parse_on_object(o, where) for o in on_object_raw]

    return Move(
        id=move_id, goal_distance=goal_distance, goal_pose=goal_pose, turn=turn, vdes=vdes,
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

    # Optional, defaults to "1.0" -- every mission written before this field
    # existed omits it entirely, and that must keep parsing exactly as it did
    # before (see MissionConfig.schema_version's own comment: this is
    # informational, not an enforced gate on which fields below are legal).
    schema_version = raw.get('schema_version', '1.0')
    _require(isinstance(schema_version, str) and schema_version, 'schema_version must be a non-empty string if present')

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

    return MissionConfig(mission_id=mission_id, moves=moves, schema_version=schema_version)


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
