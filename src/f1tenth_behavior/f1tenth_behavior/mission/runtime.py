"""Mutable mission runtime state -- the blackboard-held object the mission BT
nodes read/write every tick.

Kept separate from mission_config.py's parsed (frozen) MissionConfig
deliberately: MissionConfig is what was loaded from JSON and never changes once
parsed; MissionRuntimeState is where execution currently stands and changes
every tick. Also has no rclpy dependency -- same reasoning as mission_config.py.
"""

import time
from dataclasses import dataclass, field
from enum import Enum
from typing import NamedTuple, Optional, Set, Tuple

from f1tenth_behavior.mission.mission_config import MissionConfig, Move, StopCondition

# Blackboard key every mission BT node registers (READ or WRITE) to access the
# shared MissionRuntimeState instance -- one key, one object, mutated in place
# via its own methods (start(), log_stub_once()) rather than reassigned per
# field, so a single py_trees.blackboard.Client.register_key(MISSION_KEY, ...)
# is enough for any node that needs it.
MISSION_KEY = 'mission'

# Live-sensor blackboard keys CheckStopCondition (behaviours/check_stop_
# condition.py) owns the WRITE side of -- defined here, not there, so
# mission/preflight.py's liveness check can read the same key names without
# mission/ (ROS-independent by convention -- see this module's own docstring)
# importing from behaviours/ (ROS-facing), which would invert the existing
# dependency direction (behaviours -> mission, never mission -> behaviours).
# check_stop_condition.py imports these from here rather than defining them
# locally -- same names, same values, one definition. See that module's own
# docstring for what populates each and the staleness caveats that apply.
CURRENT_XY_KEY = 'mission_current_xy'
CURRENT_YAW_KEY = 'mission_current_yaw'
MIN_OBSTACLE_DISTANCE_KEY = 'mission_min_obstacle_distance'
FRONT_CLEARANCE_KEY = 'mission_front_clearance'
# Global (map-frame) EKF pose -- /ekf_global/odometry/filtered, a SEPARATE
# subscription from local /odom above (see CheckStopCondition's own docstring
# for why one node owning two independent topics is fine here, unlike
# duplicating the fusion itself would be). Used only for closed-loop move
# scoring (mission/move_scoring.py) -- deliberately NOT used for stop_
# condition evaluation, which stays on the low-latency local source. Global
# EKF is "the smoothest available reference" for post-hoc grading (confirmed
# with the user), not for real-time control decisions.
GLOBAL_XY_KEY = 'mission_global_xy'
GLOBAL_YAW_KEY = 'mission_global_yaw'
# Signed, unwrapped, accumulated rotation (degrees) since the current move
# started, from the GLOBAL EKF source -- CheckStopCondition's own second
# accumulator, parallel to (not sharing state with) the local-odom one that
# drives orientation_delta itself (see that module's "Turn accumulation"
# docstring paragraph for why an unwrapped accumulator is required at all,
# not a wrapped current-minus-start delta -- the exact same reasoning
# applies here, just against a different data source, for a different
# purpose: grading a finished turn, not deciding when to stop it).
GLOBAL_TURN_ACCUM_KEY = 'mission_global_turn_accum_deg'


class MissionState(Enum):
    IDLE = 'IDLE'
    # Parsed and installed via /mission/load_mission (or mission_file_name /
    # /mission/load_path), but not yet driving -- MissionActive only succeeds
    # on RUNNING/HOLDING, so the mission subtree does not tick (and therefore
    # never publishes a goal) while LOADED. /mission/start_mission is the only
    # way out of this state (see MissionRuntimeState.begin()).
    LOADED = 'LOADED'
    RUNNING = 'RUNNING'
    HOLDING = 'HOLDING'
    COMPLETE = 'COMPLETE'
    ABORTED = 'ABORTED'


class DetectionInfo(NamedTuple):
    """One entry of the detected_classes blackboard value: when a class was last
    seen (time.monotonic()) and its most recent confidence score. Written by
    DetectedClassesBridge for every message on /camera/detections, regardless of
    score -- min_confidence filtering happens at evaluation time (object_seen's
    own min_confidence is per-stop-condition, not a single global cutoff)."""
    last_seen: float
    score: float


@dataclass
class HoldContext:
    """Set by HandleObjectAction when a stop_and_hold fires, read by the same
    behaviour on every later tick to decide whether to resume.

    triggering_class is what to fall back to (an implicit object_cleared on it)
    when the JSON's on_object entry omitted resume_condition. hold_start_time/xy
    are captured when the hold begins, separate from the underlying move's own
    move_start_time/xy -- a resume_condition of type time_elapsed/distance_reached
    is evaluated relative to when the *hold* started, not when the move that was
    interrupted started (those are generally different moments, and re-using the
    move's own start would make e.g. a 5s resume timer actually fire immediately
    if the move had already been running for a while)."""
    triggering_class: str
    resume_condition: Optional[StopCondition]
    hold_start_time: float
    hold_start_xy: Optional[Tuple[float, float]]


@dataclass
class MissionRuntimeState:
    config: Optional[MissionConfig] = None
    current_index: int = 0
    move_start_time: float = 0.0
    move_start_xy: Optional[Tuple[float, float]] = None
    # Lazily captured the same way move_start_xy is (see CheckStopCondition's
    # own lazy-capture comment) -- only actually consumed by a "turn" step's
    # orientation_delta stop_condition, but captured unconditionally for
    # every move for the same reason move_start_xy is: simpler, uniform code,
    # harmless for move types that never read it.
    move_start_yaw: Optional[float] = None
    # Global-EKF counterparts of move_start_xy/move_start_yaw above -- same
    # lazy-capture pattern, same reasoning, different (map-frame, /ekf_global/
    # odometry/filtered) source. Consumed only by mission/move_scoring.py's
    # closed-loop verification/precision scoring (3.2/3.5), never by
    # condition_eval.py -- stop_condition evaluation stays on the local
    # source (see runtime.py's own GLOBAL_XY_KEY comment for why).
    move_start_global_xy: Optional[Tuple[float, float]] = None
    move_start_global_yaw: Optional[float] = None
    # Set by whichever behaviour ends the current move (CheckStopCondition's
    # own condition-satisfied/timeout branches, HandleObjectAction's
    # abort_mission/skip_to_move) just before the transition, read by
    # mission/move_scoring.py.record_move_outcome() at that same moment --
    # see each setter's own comment for the exact tag used. None only before
    # the very first move has ever ended.
    last_stop_reason: Optional[str] = None
    # Per-mission-run move-by-move scoring record (mission/move_scoring.py) --
    # appended to by record_move_outcome(), read by write_mission_summary() at
    # mission end. Reset (to empty) only in load() -- NOT in goto_move(),
    # unlike move_start_xy/yaw above, since this is exactly the accumulating
    # history load()'s own reset must not wipe mid-mission.
    move_outcomes: list = field(default_factory=list)
    # Real wall-clock (time.time(), not time.monotonic() like move_start_time
    # above) timestamp of when THIS mission run actually started -- captured
    # in begin(), used only for the summary file's own timestamp/filename
    # (mission/move_scoring.py.write_mission_summary()), where a real
    # calendar time is what makes runs comparable across restarts of this
    # node (time.monotonic() is meaningless across process boundaries).
    mission_start_wall_time: Optional[float] = None
    state: MissionState = MissionState.IDLE
    # Set by AdvanceMove/HandleObjectAction's skip_to_move, cleared by
    # PublishMoveGoal once it has actually published for the new move -- guards
    # against republishing /mpc/goal_distance every tick (see PublishMoveGoal's
    # own docstring for why that matters to mpc_corr specifically).
    goal_dirty: bool = False
    hold_context: Optional[HoldContext] = None

    # Per-current-move dedup for "stub feature used" warnings (goal_pose,
    # vdes-override, manual stop_condition, reduce_speed/_for) so they log once
    # per move, not once per tick. Reset automatically on move change -- see
    # log_stub_once().
    _stub_logged_tags: Set[str] = field(default_factory=set)
    _stub_logged_move_id: Optional[str] = None

    @property
    def current_move(self) -> Optional[Move]:
        if self.config is None:
            return None
        if not (0 <= self.current_index < len(self.config.moves)):
            return None
        return self.config.moves[self.current_index]

    def log_stub_once(self, logger, tag: str, message: str) -> None:
        """Log `message` (logger.warn) the first time `tag` is requested for the
        CURRENT move; a no-op on every later tick for that same move. Automatically
        resets when current_index changes, so a stub used again on a later move
        logs again.
        """
        move = self.current_move
        move_id = move.id if move is not None else None
        if move_id != self._stub_logged_move_id:
            self._stub_logged_tags = set()
            self._stub_logged_move_id = move_id
        if tag in self._stub_logged_tags:
            return
        self._stub_logged_tags.add(tag)
        logger.warn(message)

    def load(self, config: MissionConfig, now: float) -> None:
        """Install a newly-loaded, already-validated mission WITHOUT starting it
        -- state becomes LOADED, not RUNNING. Called by MissionLoader after a
        successful /mission/load_mission (or mission_file_name / load_path) call
        -- never called with a partially-valid config (mission_config.py's parser
        already guarantees that). The mission subtree will not tick -- and
        therefore never publish a goal -- until begin() transitions LOADED ->
        RUNNING (see /mission/start_mission).

        move_start_xy (and move_start_yaw, for a turn step's orientation_delta)
        is deliberately left None (not captured here): MissionLoader doesn't
        track odom (nothing outside CheckStopCondition does -- see its own
        docstring), and odom may not even be available yet at load time. Same
        reasoning as AdvanceMove/HandleObjectAction's skip_to_move -- see
        CheckStopCondition's lazy-capture comment for how it actually gets set.
        """
        self.config = config
        self.current_index = 0
        self.move_start_time = now
        self.move_start_xy = None
        self.move_start_yaw = None
        self.move_start_global_xy = None
        self.move_start_global_yaw = None
        self.last_stop_reason = None
        self.move_outcomes = []
        self.mission_start_wall_time = None
        self.state = MissionState.LOADED
        self.goal_dirty = False
        self.hold_context = None
        self._stub_logged_tags = set()
        self._stub_logged_move_id = None

    def begin(self, now: float) -> None:
        """LOADED -> RUNNING, called by /mission/start_mission (MissionLoader
        checks state == LOADED before calling this; see
        MissionLoader._on_start_mission_service()). move_start_time is reset here
        (not in load()) so a move's time_elapsed-style stop conditions are
        measured from when the mission actually starts moving, not from whenever
        it happened to be loaded -- those can be arbitrarily far apart."""
        self.move_start_time = now
        self.mission_start_wall_time = time.time()
        self.state = MissionState.RUNNING
        self.goal_dirty = True

    def goto_move(self, index: int, now: float) -> None:
        """Jump to move `index` -- a normal +1 advance (AdvanceMove) or an
        arbitrary skip_to_move target (HandleObjectAction): same transition
        either way. Resets move_start_time and marks goal_dirty so
        PublishMoveGoal republishes for the new move; move_start_xy/
        move_start_yaw (and their global-EKF counterparts) reset to None
        rather than being carried over, for the same lazy-capture reason as
        start() -- see CheckStopCondition's own docstring for how they get
        set. move_outcomes is NOT reset here -- see its own field comment.
        """
        self.current_index = index
        self.move_start_time = now
        self.move_start_xy = None
        self.move_start_yaw = None
        self.move_start_global_xy = None
        self.move_start_global_yaw = None
        self.goal_dirty = True

    def complete(self) -> None:
        self.state = MissionState.COMPLETE
        self.goal_dirty = False

    def abort(self) -> None:
        self.state = MissionState.ABORTED
        self.goal_dirty = False
