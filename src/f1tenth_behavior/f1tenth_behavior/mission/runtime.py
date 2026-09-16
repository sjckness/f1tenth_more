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
# Forward-half-plane counterpart of the key above (/mpc/min_obstacle_distance_
# forward -- MPC_corr.compute_forward_obstacle_distance's dot > 0 filter). A
# SECOND key, not a replacement: an obstacle_distance_below stop_condition
# picks which of the two it reads via its own `forward_only` param, which
# defaults to false so every mission written before the distinction existed
# keeps reading the omnidirectional value. See condition_eval.py's own
# obstacle_distance_below branch.
MIN_OBSTACLE_DISTANCE_FORWARD_KEY = 'mission_min_obstacle_distance_forward'
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
# Latest /mpc/object_status, as an ObjectStatusSample. CheckStopCondition owns
# the subscription (object_reached reads it); GoToObject reads it for the
# terminal target_behind flag. One subscription, one value, like every key
# above.
OBJECT_STATUS_KEY = 'mission_object_status'


class ObjectStatusSample(NamedTuple):
    """The fields of one ObjectApproachStatus the mission acts on.

    received_sec is time.monotonic() at receipt -- the same clock as
    EvalContext.now -- so a status that stopped arriving can be told apart
    from one that is merely unchanged.
    """

    move_id: str
    r: float
    alpha: float
    target_age_s: float
    target_behind_terminal: bool
    goal_watchdog: bool
    received_sec: float


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
class ObjectApproachRecord:
    """What the current go_to_object move has done so far, for scoring.

    Written by GoToObject (the handler's phase, target point and track) and by
    CheckStopCondition (the last status for THIS move), read by
    move_scoring.record_move_outcome when the move ends. One per move.
    """

    wire_move_id: str
    phase: str = 'ACQUIRE'
    target_xy: Optional[Tuple[float, float]] = None
    track_id: Optional[str] = None
    target_radius: Optional[float] = None
    gap_m: Optional[float] = None
    nose_reach_m: Optional[float] = None
    last_status: Optional[ObjectStatusSample] = None
    outcome: Optional[str] = None


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
    # Monotonic counter of mission RUNS, bumped by load() and by begin().
    # Exists because a move id is not unique across runs, and behaviours that
    # cache per-move state keyed on the move id alone therefore carry it from
    # one run of a mission into the next.
    #
    # WHAT THAT COST. go_to_person.json is a single-move mission whose move is
    # always `move_0_go_to_person`. CheckStopCondition latches mpc_corr's
    # /mpc/goal_reached (which is only ever published True, never False) and
    # reset that latch only when the move id changed -- so re-running the
    # mission in a live executor left the latch True from the previous run and
    # the first tick of run 2 saw its own goal as already reached. Three live
    # runs on 2026-09-15 recorded it exactly: run 1 drove 0.548 m, runs 2 and 3
    # completed in 0.082 s having moved 0.0 m, both mismatch_flagged.
    #
    # An int rather than a bool "is this a new run" flag: only INEQUALITY is
    # ever tested, so a consumer compares (run_generation, move.id) against
    # what it last saw and needs no reset hook of its own, no ordering
    # guarantee against the run's first tick, and no way to miss an edge.
    run_generation: int = 0
    state: MissionState = MissionState.IDLE
    # Set by AdvanceMove/HandleObjectAction's skip_to_move, cleared by
    # PublishMoveGoal once it has actually published for the new move -- guards
    # against republishing /mpc/goal_distance every tick (see PublishMoveGoal's
    # own docstring for why that matters to mpc_corr specifically).
    goal_dirty: bool = False
    hold_context: Optional[HoldContext] = None
    # The current go_to_object move's record (see ObjectApproachRecord); None
    # for every other move type and between moves.
    object_record: Optional[ObjectApproachRecord] = None

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
        self.object_record = None
        self._stub_logged_tags = set()
        self._stub_logged_move_id = None
        # New run: see run_generation's own field comment. Bumped here AND in
        # begin() on purpose -- load() alone would miss a begin() that follows
        # some other path to LOADED, and begin() alone would leave a reloaded
        # mission sharing the previous run's generation for every tick between
        # load and start. Two bumps per normal run is harmless: only
        # inequality is ever tested.
        self.run_generation += 1

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
        self.object_record = None
        # LOADED -> RUNNING is the other half of "a new run started" -- see
        # run_generation's own field comment and load()'s bump.
        self.run_generation += 1

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
        self.object_record = None

    def complete(self) -> None:
        self.state = MissionState.COMPLETE
        self.goal_dirty = False

    def abort(self) -> None:
        self.state = MissionState.ABORTED
        self.goal_dirty = False
