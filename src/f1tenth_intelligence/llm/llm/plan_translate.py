"""Translates an LLM phase list (llm_planner_node.py's own vocabulary --
mode/guard/thresh/turn_sign/stop_at/stop_at_distance) into a mission dict
matching f1tenth_behavior's REAL mission schema (mission/mission_config.py's
parse_mission()), ready to json.dump() to a file and hand to
/mission/load_mission.

Deliberately pure Python, no rclpy/ROS import of any kind -- fully
unit-testable in isolation (see test/test_plan_translate.py) the same way
mission_config.py itself is. llm_planner_node.py (this package's ROS-facing
node) is the only caller.

Schema facts this module depends on -- verified by reading mission/
mission_config.py, mission/condition_eval.py, and f1tenth_behavior/README.md's
own "Mission config" table before writing a single line here, not assumed
from the task description's own (partly incorrect) proposed mapping:

  - Top-level key is "moves", not "plan".
  - A move is discriminated by exactly one of goal_distance/goal_pose/turn
    being set (mission_config._parse_move: `sum(...) == 1`).
  - stop_condition is always a SIBLING field on the move, never nested under
    goal_distance/turn.
  - stop_condition.type spellings actually in STOP_CONDITION_TYPES:
    "distance_reached" (not "fixed_distance"), "front_clearance" (param key
    "distance", not "value"), "orientation_delta" (param key "value").
  - "orientation_delta" is turn-exclusive (mission_config rejects it on any
    move without a `turn` field) and its `value` must be in DEGREES, equal to
    abs(turn.heading_delta_deg) to within math.isclose(abs_tol=1e-6) --
    checked at load time. Confirmed live against the two real missions that
    already use "turn" (missions/test_03_bounce_walls.json,
    missions/test_04_turn_accuracy.json): both set stop_condition.value equal
    to the turn's own heading_delta_deg magnitude, e.g. `"turn":
    {"heading_delta_deg": 90, ...}, "stop_condition": {"type":
    "orientation_delta", "value": 90}`. This is why `value` below is always
    derived FROM heading_delta_deg (never recomputed independently from
    thresh/turn_sign a second time) -- two independent computations of the
    "same" number risk a floating-point mismatch the loader would then
    reject at load time.
  - turn.speed is REQUIRED and must be > 0.0 (mission_config._parse_turn_spec)
    -- the LLM's own phase vocabulary (llm_planner_node.SYSTEM_PROMPT) has no
    speed field at all, so this module supplies DEFAULT_TURN_SPEED_MPS (see
    its own comment) rather than leaving it unset, which mission_config.py
    would reject outright.
  - turn.heading_delta_deg sign convention (mission_config.py's own docstring
    on TurnSpec): "+ = left/CCW, - = right/CW". The LLM's own turn_sign
    convention (SYSTEM_PROMPT: "-1.0 = DESTRA, +1.0 = SINISTRA") already
    matches that sign exactly, so heading_delta_deg = degrees(thresh) *
    turn_sign directly -- no separate sign()/copysign needed, and
    validate_plan() already guarantees turn_sign is exactly -1.0 or 1.0
    before a phase list ever reaches this module.
  - The task's proposed mapping table only ever set `turn: {...}` with no
    stop_condition at all, and suggested schema_version "1.0" -- both
    corrected here: EVERY move requires a stop_condition
    (mission_config._parse_move: `_require('stop_condition' in raw, ...)`),
    and every real mission already using "turn" sets schema_version "2.0"
    (2.0 is the version that introduced the "turn" step type/orientation_delta
    at all -- see mission_config.py's own STOP_CONDITION_TYPES/TurnSpec
    comments). Both are flagged again in llm_planner_node's own module
    docstring and in the implementation summary.

Deliberately narrow scope: this module only implements the guards
SYSTEM_PROMPT actually teaches the 3B model to produce -- "wall"/
"front_object"/"distance" (-> a goal_distance move) and "turned" (-> a turn
move). Any other guard raises PlanTranslationError rather than guessing a
mapping the LLM was never taught to produce; mission_config.py's schema
supports plenty of stop_condition types (object_seen, time_elapsed,
obstacle_distance_below, ...) this translator has no reason to ever emit.

IMPORTANT correction vs. the task's own proposed mapping table, found only by
actually reading SYSTEM_PROMPT's example 1 closely (llm_planner_node.py):
    {"plan":[{"mode":"straight","guard":"wall","thresh":3.0},
             {"mode":"wall_turn","turn_sign":-1.0,"guard":"turned","thresh":1.3},
             {"mode":"wall_turn","turn_sign":-1.0,"guard":"distance","thresh":2.0,
              "stop_at_distance":2.0}]}
The THIRD phase is mode "wall_turn" with guard "distance" -- not "turned".
REGOLA 1 in SYSTEM_PROMPT explains why: "after a turn, to keep going, use
wall_turn with the SAME turn_sign, never straight" -- in the OLD, now-dead
/corridor_cmd system this file used to target, "mode" selected which fixed
heading the SLSQP corridor realigned to (straight -> the ORIGINAL mission-
start heading, wall_turn -> stay on the current post-turn heading), so
re-using "straight" after a turn would have silently steered the car back
toward its pre-turn direction. The REAL mission schema (mission_config.py)
has **no such "mode"/realignment concept at all** -- mpc_corr.py's own
goal_distance handling always continues from whatever heading is current
when THAT move starts (MPC_corr.py's goal_distance_callback re-anchors
psi_init_corridor to self.yaw every time, fixed by the "straight-after-turn
reference frame" commit already in this repo's history), so the old trick is
both unnecessary and irrelevant here. Consequently: **the mapping below is
keyed on `guard`, not `mode`** -- "turned" always produces a turn move
(requires mode=="wall_turn" with turn_sign, exactly as validate_plan()
itself only requires turn_sign in that case); every other guard
("wall"/"front_object"/"distance") produces a goal_distance move regardless
of whether `mode` says "straight" or "wall_turn". `mode` is otherwise
unused by this translator -- it has no equivalent in the real schema.
"""

import math
import time
from typing import Optional

# The LLM phase vocabulary (SYSTEM_PROMPT in llm_planner_node.py) has no
# speed field at all, but mission_config.TurnSpec.speed is required (> 0.0).
# 0.5 m/s matches mpc_corr.py's own self.vdes default (MPC_corr.py) and is
# also the exact value missions/test_03_bounce_walls.json's own "turn" moves
# already use in practice -- a defensible, already-precedented default, not
# an arbitrary number. Judgment call, flagged in the implementation summary;
# revisit if turn moves this pass generates turn out too fast/slow live.
DEFAULT_TURN_SPEED_MPS = 0.5

# "some large cap" per the task's own proposed mapping for a wall/front_object
# -guarded straight phase (these guards stop the move on CLEARANCE, not on
# distance traveled, so goal_distance here is just an upper bound the move
# should never actually reach). Set to THRESH_RANGE['distance']'s own upper
# bound in llm_planner_node.py (the largest distance validate_plan() would
# ever accept for ANY phase) so this cap can never be the effective binding
# constraint ahead of a distance-guarded move.
STRAIGHT_GOAL_DISTANCE_CAP_M = 50.0

# Mission-level schema_version -- see module docstring's own paragraph on why
# this is "2.0", not the task's proposed "1.0": every real mission already
# using a "turn" step sets this, and 2.0 is specifically the version that
# introduced "turn"/orientation_delta into the schema at all.
MISSION_SCHEMA_VERSION = '2.0'


class PlanTranslationError(ValueError):
    """Raised when `phases` reaches phases_to_mission() malformed/unvalidated
    (see _sanity_check), or contains a (mode, guard) pairing outside the
    mapping this module implements (see module docstring)."""


def _sanity_check(phases) -> None:
    """Defensive re-check of the invariants this module's own mapping logic
    depends on -- NOT a reimplementation of llm_planner_node.validate_plan()'s
    full contract (its THRESH_RANGE plausibility warnings, for instance, don't
    matter for translation correctness). Deliberately does not import
    validate_plan() from llm_planner_node.py: that module imports rclpy at
    load time, and pulling it in here (even lazily) would break this module's
    own "no ROS imports, fully unit-testable in isolation" property that the
    task explicitly asked for. The REAL first line of defense is and stays
    LLMPlannerNode.process_command() calling validate_plan() before this
    function is ever reached -- this is only the second line, so a phase list
    that skips that step fails loudly and specifically here instead of
    crashing on a bare KeyError/TypeError somewhere in the mapping loop below,
    or silently producing a bogus mission.
    """
    if not isinstance(phases, list) or not phases:
        raise PlanTranslationError('phases must be a non-empty list (did validate_plan() run?)')
    for i, ph in enumerate(phases):
        if not isinstance(ph, dict):
            raise PlanTranslationError(f'phase {i} is not an object (did validate_plan() run?)')
        if 'mode' not in ph or 'guard' not in ph or 'thresh' not in ph:
            raise PlanTranslationError(
                f'phase {i} is missing mode/guard/thresh (did validate_plan() run?)')
        if not isinstance(ph['thresh'], (int, float)) or isinstance(ph['thresh'], bool):
            raise PlanTranslationError(
                f'phase {i}: thresh must be numeric, got {ph["thresh"]!r} '
                '(did normalize_plan()+validate_plan() run?)')
        if ph['mode'] == 'wall_turn' and 'turn_sign' not in ph:
            raise PlanTranslationError(
                f'phase {i}: wall_turn with no turn_sign (did validate_plan() run?)')


def _phase_to_move(i: int, ph: dict) -> dict:
    # Keyed on `guard`, not `mode` -- see module docstring's own correction
    # paragraph for why: the real mission schema has no "mode" concept, and
    # SYSTEM_PROMPT's own example 1 has a mode="wall_turn"/guard="distance"
    # phase (post-turn continuation) that is NOT a turn move.
    mode = ph['mode']
    guard = ph['guard']
    thresh = float(ph['thresh'])
    move_id = f'move_{i}'

    if guard == 'turned':
        if mode != 'wall_turn' or 'turn_sign' not in ph:
            raise PlanTranslationError(
                f"phase {i}: guard 'turned' requires mode 'wall_turn' with a turn_sign "
                f'(got mode={mode!r}); no direction to derive heading_delta_deg from.'
            )
        turn_sign = float(ph['turn_sign'])
        heading_delta_deg = math.degrees(thresh) * turn_sign
        return {
            'id': move_id,
            'turn': {
                'heading_delta_deg': heading_delta_deg,
                'speed': DEFAULT_TURN_SPEED_MPS,
                'steering': 'full_lock',
            },
            'stop_condition': {
                'type': 'orientation_delta',
                'value': abs(heading_delta_deg),
            },
        }

    if guard == 'distance':
        return {
            'id': move_id,
            'goal_distance': thresh,
            'stop_condition': {'type': 'distance_reached', 'distance': thresh},
        }

    if guard in ('wall', 'front_object'):
        return {
            'id': move_id,
            'goal_distance': STRAIGHT_GOAL_DISTANCE_CAP_M,
            'stop_condition': {'type': 'front_clearance', 'distance': thresh},
        }

    raise PlanTranslationError(
        f'phase {i}: unsupported guard {guard!r} -- outside the mapping SYSTEM_PROMPT '
        'teaches the LLM to produce; refusing to guess.'
    )


def phases_to_mission(phases: list, mission_id: Optional[str] = None) -> dict:
    """Translate a phase list (already normalize_plan()+validate_plan()'d by
    the caller -- see _sanity_check's own docstring for why that's still not
    fully trusted here) into a mission dict matching mission_config.
    parse_mission()'s schema.

    Raises PlanTranslationError -- never returns a partial/best-effort
    result, same "all or nothing" discipline mission_config.parse_mission()
    itself uses.
    """
    _sanity_check(phases)

    if mission_id is None:
        mission_id = f'llm_plan_{int(time.time())}'

    moves = [_phase_to_move(i, ph) for i, ph in enumerate(phases)]

    # Last-phase stop_at/stop_at_distance OVERRIDES that move's base
    # stop_condition, regardless of its mode -- both branches use an EXPLICIT
    # `distance` param (never relying on goal_distance) so the override is
    # valid even when the last phase is a wall_turn (whose move has no
    # goal_distance field at all to fall back on). validate_plan() already
    # guarantees at most one of stop_at/stop_at_distance is present, and only
    # on the last phase.
    last = phases[-1]
    if 'stop_at' in last:
        moves[-1]['stop_condition'] = {
            'type': 'front_clearance', 'distance': float(last['stop_at'])}
    elif 'stop_at_distance' in last:
        moves[-1]['stop_condition'] = {
            'type': 'distance_reached', 'distance': float(last['stop_at_distance'])}

    return {
        'mission_id': mission_id,
        'schema_version': MISSION_SCHEMA_VERSION,
        'moves': moves,
    }
