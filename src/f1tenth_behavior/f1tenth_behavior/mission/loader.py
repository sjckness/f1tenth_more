"""ROS-facing mission loader: the `mission_file_name` parameter (initial/default
load), a /mission/load_path (std_msgs/String) topic for fire-and-forget runtime reloads,
and four services for callers that want a synchronous result:

- /mission/load_mission (f1tenth_messages/srv/LoadMission): same load path as
  the topic, but returns success/message instead of only logging. Installs the
  mission and sets state -> LOADED; it does NOT start it (see start_mission
  below) -- this is a deliberate split from this service's original behavior,
  which went straight to RUNNING. Splitting it means a caller can load a
  mission well ahead of time (e.g. while doing pre-flight checks) without the
  car moving the instant the file parses.
- /mission/start_mission (std_srvs/Trigger): LOADED -> RUNNING. Fails
  (success=false) unless state is currently LOADED -- in particular, it does
  NOT re-arm an ABORTED/COMPLETE mission; load it again first. Also fails
  (success=false, state left at LOADED, not consumed) if mission/preflight.py's
  liveness check finds a dependency THIS mission needs is not alive and/or not
  yet publishing (mpc_corr, ackermann_to_vesc_node, localization, and
  conditionally front_clearance_node/yolo_detector_node depending on what the
  mission's moves/stop_conditions/on_object entries actually use) -- see that
  module's own docstring for the battery-precheck race this replaces "the car
  silently didn't move" with an explicit, actionable failure at start time.
  Also publishes /mpc/hold(False) exactly once, right when this transition
  actually succeeds -- see this method's own extended comment below for why:
  a real bug found via live testing, where load_mission + start_mission on a
  fresh mission after an abort both reported success and the mission genuinely
  reached RUNNING, but mpc_corr silently drove nothing because its OWN
  self.hold flag (not any state this file owns) was still latched True from
  the prior abort and nothing had ever released it.
- /mission/abort_mission (std_srvs/Trigger): abort whatever mission is
  currently loaded-or-running (LOADED, RUNNING, or HOLDING) -- no request
  payload, no mission_id to get right; there is only ever one mission slot
  at a time in this system, so "abort the current one" is unambiguous.
  (Used to require a mission_id, matching f1tenth_messages/srv/AbortMission
  -- dropped per a later simplification request; AbortMission.srv was
  removed from f1tenth_messages since this was its only consumer.) Fails
  (success=false) only if there's nothing to abort (state is
  IDLE/COMPLETE/ABORTED already). Stops the car via the existing
  /mpc/hold(True) publish mpc_corr already reacts to (same transition
  HandleObjectAction's on_object abort_mission action performs) -- not a
  new burst-publish onto /drive: that would be a second, uncoordinated
  writer racing mpc_corr's own /drive publishes, where /mpc/hold is already
  a race-free, one-writer mechanism mpc_corr itself honors every tick.
- /mission/emergency_stop (std_srvs/Trigger): sets a flag that is latched for
  this node's lifetime -- no reset service exists on purpose (restart the
  node/BT process to clear it). Deliberately does NOT touch mission state or
  /mpc/hold: its whole effect is feeding the emergency lane's
  IsEmergencyStopTriggered condition (see that behaviour and
  behavior_executor_node.create_root()), which stops the car the exact same
  way IsBatteryLow/IsSystemOverheated already do -- publishing onto the
  safety_stop ackermann_mux lane (priority 200), which outranks anything
  mpc_corr or the mission subtree publish regardless of their own internal
  state. This is intentionally a different mechanism from the mission-level
  abort above: a hardware/operator emergency stop, not a mission control flow.

State changes here are also published on /mission/status
(f1tenth_messages/msg/MissionStatus, transient-local QoS so a late subscriber
still gets the current value immediately) -- state, json_path, and the
emergency_stop flag folded into the same message rather than a separate topic
(simpler: one topic, one message, one thing to subscribe to for "what's
mission/emergency status right now"). IsEmergencyStopTriggered subscribes to
it; the mission subtree itself does not -- it already reads MissionRuntimeState
straight off the blackboard (MISSION_KEY, same process), so round-tripping
through this topic would be redundant for that.

NOT a py_trees.behaviour.Behaviour, same reasoning as DetectedClassesBridge --
loading/starting/aborting a mission (or tripping e-stop) is an event (parameter
read once at startup, or a message/service call arriving), not a per-tick
action. Instantiated once in behavior_executor_node.main() right after
tree.setup().

Interface choice for loading, flagged explicitly (see the deliberation in
chat): the task description offered two options for the load trigger -- a
custom .srv, or a plain String topic -- and asked me not to invent new
message/interface types without confirming first. A custom .srv would mean
adding a new file to f1tenth_messages; std_msgs/String is already a dependency
used elsewhere in the stack, so it was the smaller addition and what got
implemented first. /mission/load_mission (added in a later pass, per the
requester's follow-up) is exactly that custom-srv swap, now that synchronous
request/response semantics turned out to matter -- see
f1tenth_messages/srv/LoadMission.srv. It still needs a real request field
(the path), so it kept its own .srv; abort/start/emergency_stop don't need
one, so they're plain std_srvs/Trigger (abort_mission was itself a custom
.srv -- AbortMission.srv, mission_id-guarded -- until a later simplification
dropped the guard; see /mission/abort_mission's own note above).
/mission/load_path stays; it was not removed.

Locking: all mutating entry points this class owns (the load_path callback,
and the load/start/abort/emergency_stop services) take self._lock around the
read-then-mutate sequence on the shared MissionRuntimeState. Checked before
adding this: no lock existed anywhere in the mission package before this pass.
It is *not* fixing an active race under the current architecture --
behavior_executor_node.main() drives the tree ticks (via tree.tick_tock()'s
wall timer, added to `node`), this class's own subscription, and all four
services through one `rclpy.spin(node)` call with rclpy's default
SingleThreadedExecutor, so no two ROS callbacks -- including a BT tick and a
service call -- can ever actually execute concurrently; each runs to
completion before the next one starts. The lock here is explicit, local
documentation of that invariant for the entry points this file owns, and a
safety net if this node's executor or callback-group setup ever changes
without this file being revisited. It deliberately does NOT reach into
HandleObjectAction's abort_mission/skip_to_move or AdvanceMove's goto_move --
those mutate the same MissionRuntimeState but were out of scope for this pass
(BT node tick logic was explicitly not to be touched); they remain safe today
for the same single-threaded-executor reason, not because they share this
lock.
"""

import json
import math
import os
import threading
import time
from typing import Tuple

import py_trees
from ament_index_python.packages import get_package_share_directory
from f1tenth_messages.msg import MissionStatus, MoveOutcome
from f1tenth_messages.srv import LoadMission
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger

from f1tenth_behavior.mission.mission_config import MissionConfigError, load_mission_file
from f1tenth_behavior.mission.preflight import (
    PREFLIGHT_BLACKBOARD_KEYS,
    check_liveness,
    required_dependencies,
)
from f1tenth_behavior.mission.runtime import (
    MISSION_KEY,
    MissionRuntimeState,
    MissionState,
)

# Shared by the /mission/status publisher here and IsEmergencyStopTriggered's
# subscriber -- durability must match on both ends for the "late subscriber
# still gets the current value" latch behavior to actually work.
MISSION_STATUS_QOS = QoSProfile(
    depth=1,
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    history=QoSHistoryPolicy.KEEP_LAST,
)


# MissionState -> the /test/mission_event name the campaign logger keys off
# (f1tenth_logger/TEST_CAMPAIGN.md). IDLE and HOLDING deliberately have none:
# the first is not a mission, the second is not a lifecycle edge -- the
# mission is still live and the logger must keep recording.
_TEST_EVENTS = {
    MissionState.LOADED: 'mission_loaded',
    MissionState.RUNNING: 'mission_started',
    MissionState.COMPLETE: 'mission_finished',
    MissionState.ABORTED: 'mission_aborted',
}


def _float_or_nan(value):
    return float(value) if value is not None and math.isfinite(float(value)) else math.nan


def _move_outcome_msg(outcome, mission_id, stamp):
    """move_scoring.MoveOutcome -> f1tenth_messages/MoveOutcome (None -> NaN / '')."""
    msg = MoveOutcome()
    msg.header.stamp = stamp
    msg.mission_id = mission_id
    msg.move_id = outcome.move_id
    msg.move_type = outcome.move_type
    msg.stop_reason = outcome.stop_reason or ''
    msg.outcome = getattr(outcome, 'outcome', None) or ''
    msg.wire_move_id = getattr(outcome, 'wire_move_id', None) or ''
    msg.duration_s = _float_or_nan(outcome.end_time - outcome.start_time)
    msg.commanded = _float_or_nan(outcome.commanded)
    msg.actual = _float_or_nan(outcome.actual)
    msg.score_percent = _float_or_nan(outcome.score_percent)
    msg.mismatch_flagged = bool(outcome.mismatch_flagged)
    msg.arrival_bearing_error_deg = _float_or_nan(
        getattr(outcome, 'arrival_bearing_error_deg', None))
    msg.track_gap_m = _float_or_nan(getattr(outcome, 'track_gap_m', None))
    msg.note = outcome.note or ''
    return msg


class MissionLoader:

    def __init__(self, node, load_path_topic='/mission/load_path', hold_topic='/mpc/hold',
                 status_topic='/mission/status', move_outcome_topic='/mission/move_outcome',
                 test_event_topic='/test/mission_event'):
        self._node = node
        self._lock = threading.Lock()
        self.blackboard = py_trees.blackboard.Client(name='MissionLoader')
        self.blackboard.register_key(key=MISSION_KEY, access=py_trees.common.Access.WRITE)
        setattr(self.blackboard, MISSION_KEY, MissionRuntimeState())
        # READ-only -- CheckStopCondition owns writing these (see mission/
        # runtime.py's own comment on why the key constants live there).
        # Read here only by _on_start_mission_service's preflight check
        # (mission/preflight.py) to confirm real data has actually arrived,
        # not merely that a node exists -- see that module's own docstring.
        # The key list lives there too, so the two cannot drift apart.
        for key in PREFLIGHT_BLACKBOARD_KEYS:
            self.blackboard.register_key(key=key, access=py_trees.common.Access.READ)

        # Not part of MissionRuntimeState: unrelated to mission progress (can be
        # true whether or not a mission is even loaded), and unlike everything
        # else on the blackboard it never resets on a new load -- keeping it
        # there would risk it getting cleared by a future load()/goto_move()
        # touching the dataclass, which must never happen (see /mission/
        # emergency_stop's own docstring: no reset except a node restart).
        self._emergency_stop_active = False
        self._current_json_path = ''

        self.hold_pub = node.create_publisher(Bool, hold_topic, 10)
        self.status_pub = node.create_publisher(MissionStatus, status_topic, MISSION_STATUS_QOS)
        # Each finished move's scored outcome, once, so a bag carries it (the
        # mission_reports JSON is written on the car, not recorded). Same
        # watcher as /mission/status below; see _publish_new_outcomes.
        self.move_outcome_pub = node.create_publisher(MoveOutcome, move_outcome_topic, 10)
        self._outcomes_published = (None, 0)

        # The mission lifecycle as JSON, for the test-campaign logger
        # (f1tenth_logger/test_campaign/). Published from _publish_status() and nowhere
        # else -- see _publish_test_event for why.
        self.test_event_pub = node.create_publisher(String, test_event_topic, 10)
        self._last_event_key = None
        self._pending_event_reason = ''

        # Countdown between /mission/start_mission and the mission actually
        # beginning, so every test starts from a measured standstill. The car
        # cannot move during it: the state stays LOADED, and MissionActive only
        # succeeds on RUNNING/HOLDING, so the mission subtree never ticks and no
        # goal is ever published; /mpc/hold stays engaged until the timer fires.
        # /mission/emergency_stop and /mission/abort_mission both cancel it.
        self._countdown_s = float(
            node.declare_parameter('mission_countdown_sec', 3.0).value)
        # Read live at every start (see _countdown_seconds) so a campaign can
        # retune it with `ros2 param set` between tests, without a restart.
        self._countdown_timer = None
        self._start_cancelled = False
        # The countdown the last accepted start actually armed (0.0 when it
        # was switched off). mission_started reports this, not a fresh read
        # of the parameter, which may have been retuned in between.
        self._armed_countdown_s = None

        self._sub = node.create_subscription(
            String, load_path_topic, self._on_load_path, 10)
        self._load_srv = node.create_service(
            LoadMission, '/mission/load_mission', self._on_load_mission_service)
        self._start_srv = node.create_service(
            Trigger, '/mission/start_mission', self._on_start_mission_service)
        self._abort_srv = node.create_service(
            Trigger, '/mission/abort_mission', self._on_abort_mission_service)
        self._estop_srv = node.create_service(
            Trigger, '/mission/emergency_stop', self._on_emergency_stop_service)

        mission_file_name = str(node.declare_parameter('mission_file_name', '').value)
        if mission_file_name:
            missions_dir = os.path.join(
                get_package_share_directory('f1tenth_behavior'), 'missions')
            self._load(os.path.join(missions_dir, mission_file_name))
        else:
            node.get_logger().info(
                '[mission] mission_file_name parameter not set -- waiting for a path on '
                f'{load_path_topic}, or a call to /mission/load_mission '
                '(mission.state stays IDLE until then).'
            )

        # MISSION-END EVENT GAP FIX (automatic-mission-logger pass). Before
        # this, /mission/status was only ever published from the four service/
        # load paths in this file -- but three of the FOUR ways a mission can
        # actually reach a terminal state don't go through any of them, because
        # they happen inside a BT tick against the shared MissionRuntimeState
        # object, which holds no node or publisher reference at all (see
        # mission/runtime.py: complete()/abort() just assign self.state):
        #   - advance_move.py's state.complete()          -- NORMAL SUCCESS
        #   - handle_object_action.py's state.abort()     -- on_object abort
        #   - check_stop_condition.py's state.abort()     -- stop-condition abort
        # Only _on_abort_mission_service's own state.abort() (an operator
        # calling /mission/abort_mission) republished. So /mission/status
        # latched RUNNING forever through a mission that had actually finished
        # or aborted itself, and MissionStatus.msg's own documented contract --
        # "Republished whenever mission state changes" -- was not true for the
        # success path or either autonomous abort path. Any consumer keying off
        # mission lifecycle (f1tenth_diagnostics' mission_logger_node is the
        # first) would therefore start on RUNNING and then never stop, losing
        # exactly the failure-run data it exists to capture.
        #
        # Fixed by WATCHING the state rather than adding publish calls to each
        # of the three BT behaviours: those run inside the tree and have no
        # publisher, and enumerating call sites is exactly the pattern that let
        # three of them drift out of sync in the first place. This timer diffs
        # the live state against the last value actually published and emits on
        # any change, so every terminal transition is covered, including ones
        # added later that this file never learns about. Still NOT a per-tick
        # publish (the .msg's other documented promise): it publishes on CHANGE
        # only, so a steady RUNNING mission emits nothing.
        self._last_published_state = None
        state_watch_period_sec = float(
            node.declare_parameter('mission_state_watch_period_sec', 0.1).value)
        self._state_watch_timer = node.create_timer(
            state_watch_period_sec, self._on_state_watch_tick)

        # Establishes the initial latched value immediately (state=IDLE or
        # LOADED depending on the auto-load above, emergency_stop_active=False)
        # so a subscriber that attaches before any service is ever called still
        # gets a well-defined value rather than nothing at all.
        self._publish_status()

    def _on_state_watch_tick(self):
        """Republish /mission/status if the mission state changed without one
        of this file's own service paths doing it -- see the MISSION-END EVENT
        GAP FIX comment in __init__ for which transitions those are and why
        this is a watcher rather than three extra publish calls."""
        state: MissionRuntimeState = getattr(self.blackboard, MISSION_KEY)
        self._publish_new_outcomes(state)
        if state.state is not self._last_published_state:
            self._publish_status()

    def _publish_new_outcomes(self, state):
        """Publish move_outcomes entries appended since the last tick, once each.

        Keyed on the LIST's identity as well as its length: load() replaces the
        list, so a new run starts counting from zero instead of skipping its
        first outcomes or republishing the previous run's.
        """
        outcomes = state.move_outcomes
        list_id, count = self._outcomes_published
        if list_id != id(outcomes):
            count = 0
        mission_id = state.config.mission_id if state.config is not None else ''
        for outcome in outcomes[count:]:
            self.move_outcome_pub.publish(_move_outcome_msg(
                outcome, mission_id, self._node.get_clock().now().to_msg()))
        self._outcomes_published = (id(outcomes), len(outcomes))

    def _on_load_path(self, msg: String):
        self._load(str(msg.data))

    def _on_load_mission_service(
        self, request: LoadMission.Request, response: LoadMission.Response
    ) -> LoadMission.Response:
        success, message = self._load(str(request.path))
        response.success = success
        response.message = message
        return response

    def _on_start_mission_service(
        self, request: Trigger.Request, response: Trigger.Response
    ) -> Trigger.Response:
        with self._lock:
            state: MissionRuntimeState = getattr(self.blackboard, MISSION_KEY)

            if state.state is not MissionState.LOADED:
                response.success = False
                response.message = (
                    f'cannot start: state is {state.state.value}, not LOADED '
                    '(load a mission via /mission/load_mission first)'
                )
                return response

            # The countdown leaves the state at LOADED, so without this a
            # second call would arm a second timer against the same mission.
            if self._countdown_timer is not None:
                response.success = False
                response.message = (
                    f'cannot start: already starting in up to '
                    f'{self._countdown_seconds():.1f} s '
                    '(call /mission/abort_mission to cancel it)'
                )
                return response

            # Refused rather than armed-and-then-refused: the emergency stop
            # latches for the lifetime of the process, so this start could
            # never be allowed to fire anyway (see _on_countdown_elapsed's own
            # check, which is what stops one already in flight).
            if self._emergency_stop_active:
                response.success = False
                response.message = (
                    'cannot start: emergency stop is latched -- restart the '
                    'node to clear it'
                )
                return response

            # Preflight liveness check (mission/preflight.py) -- confirms every
            # node/data source THIS mission actually needs is alive AND
            # publishing, not just that the service call itself is well-formed.
            # See that module's own docstring for the battery-precheck race
            # this replaces "did it silently not move" with "here's exactly
            # what's missing." Leaves state at LOADED on failure (not
            # consumed/aborted) so the caller can fix the dependency and retry
            # the same /mission/start_mission call.
            # sim mode (behavior node on use_sim_time): the car's
            # ackermann_to_vesc_node is replaced by f1tenth_sim's drive_bridge,
            # so the actuation requirement swaps to it (mission/preflight.py).
            reqs = required_dependencies(
                state.config,
                sim=bool(self._node.get_parameter('use_sim_time').value),
            )
            failures = check_liveness(
                reqs,
                self._node.get_node_names(),
                lambda key: getattr(self.blackboard, key),
            )
            if failures:
                response.success = False
                response.message = (
                    'cannot start: preflight liveness check failed -- '
                    + '; '.join(failures)
                )
                self._node.get_logger().error(
                    f"[mission] '{state.config.mission_id}' preflight FAILED, staying "
                    f'LOADED: {response.message}'
                )
                return response

            mission_id = state.config.mission_id if state.config else ''
            # Arm, do not begin: the mission starts in _on_countdown_elapsed,
            # countdown_s from now, and only if nothing has asked it to stop by
            # then. Deliberately NOT a sleep here -- this callback holds
            # self._lock and blocks the executor, so sleeping in it would make
            # /mission/emergency_stop unanswerable for exactly the window in
            # which the operator is most likely to need it.
            self._start_cancelled = False
            countdown_s = self._countdown_seconds()
            self._armed_countdown_s = max(countdown_s, 0.0)
            if countdown_s <= 0.0:
                # Countdown switched off: identical to the behaviour before
                # there was one. No timer exists, so there is nothing a stop
                # could arrive in the middle of.
                self._begin_mission(state, mission_id)
                response.success = True
                response.message = f"started '{mission_id}'"
            else:
                self._countdown_timer = self._node.create_timer(
                    countdown_s, self._on_countdown_elapsed)
                self._node.get_logger().info(
                    f"[mission] '{mission_id}' starting in "
                    f'{countdown_s:.1f} s -- state stays LOADED until '
                    f'then, /mpc/hold still engaged, emergency stop armed.'
                )
                response.success = True
                response.message = f'starting in {countdown_s:.1f} s'
        self._publish_status()
        return response

    def _on_countdown_elapsed(self):
        """The mission begins here, or it never begins at all.

        Re-checks everything rather than trusting that the timer was cancelled:
        rclpy can have this callback already queued in the executor when
        _cancel_countdown() runs, so a stop that arrives in that window would
        otherwise still start the car. ``_start_cancelled`` is the flag that
        closes it.
        """
        with self._lock:
            stop_requested = self._start_cancelled
            self._cancel_countdown()        # one-shot: never fire twice
            state: MissionRuntimeState = getattr(self.blackboard, MISSION_KEY)
            mission_id = state.config.mission_id if state.config else ''

            blocked = None
            if stop_requested:
                blocked = 'a stop was requested during the countdown'
            elif self._emergency_stop_active:
                blocked = 'the emergency stop is active'
            elif state.state is not MissionState.LOADED:
                blocked = f'state is {state.state.value}, not LOADED'

            if blocked is not None:
                self._node.get_logger().error(
                    f"[mission] '{mission_id}' NOT started -- {blocked}. The car "
                    'stays where it is.'
                )
                self._pending_event_reason = 'cancelled before start'
                if state.state is MissionState.LOADED:
                    # Do not leave it sitting at LOADED as though the start had
                    # never been asked for: it was asked for and refused, and
                    # the campaign logger needs that to be an outcome.
                    state.abort()
                    self.hold_pub.publish(Bool(data=True))
            else:
                self._begin_mission(state, mission_id)
        self._publish_status()

    def _begin_mission(self, state: MissionRuntimeState, mission_id: str):
        """The actual start: RUNNING, /mpc/hold released, one log line.

        Called from _on_countdown_elapsed, and directly from
        _on_start_mission_service when mission_countdown_sec is 0 -- one body,
        so the two paths cannot drift apart on something this safety-relevant.
        Caller holds self._lock and publishes status afterwards.
        """
        state.begin(now=time.monotonic())
        # Fixes a real "load+start after abort doesn't work" bug found via
        # live testing. Traced end-to-end, not assumed: neither this
        # file's own state.state check above nor MissionRuntimeState.load()
        # (called by _load(), unconditionally overwrites state -> LOADED
        # regardless of what it was before) ever rejects a load/start
        # sequence because of a PRIOR abort -- both correctly reset and
        # succeed. The actual blocker lives entirely in mpc_corr, a
        # different process: its own self.hold flag (MPC_corr.py's
        # hold_callback/control_loop) is set True by abort_mission (both
        # this service's own abort path and HandleObjectAction's
        # on_object one), by AdvanceMove on mission COMPLETE, and by
        # CheckStopCondition on_timeout='stop' -- and NOTHING, across this
        # entire package, ever published hold(False) again except
        # HandleObjectAction._resume() (the stop_and_hold-specific resume,
        # unrelated to starting a brand new mission). So a fresh mission
        # could load, start, reach RUNNING, and PublishMoveGoal a real
        # goal to mpc_corr -- which would then silently zero its own
        # output every tick regardless, because control_loop() checks
        # self.hold before ever looking at the goal. From the operator's
        # side this reads as "start_mission doesn't work", not as any
        # kind of error, since every service call along the way
        # genuinely succeeds.
        #
        # Fix, deliberately placed HERE rather than in abort_mission
        # itself: releasing the hold is exactly correct at the moment a
        # NEW mission is deliberately, successfully started (this is
        # precisely the point the system should commit to actively
        # driving again), not automatically during abort's own settling.
        # Clearing it inside abort_mission instead would need a "the car
        # has actually stopped" signal that does not exist anywhere in
        # this stack today (no ERPM/velocity feedback loop watches for
        # that), and publishing hold(False) there without one would risk
        # releasing it before the abort has actually taken effect -- a
        # worse safety regression than the bug being fixed. Doing it here
        # instead also uniformly covers all three latch sources above
        # (abort, mission-complete, timeout-stop) with one change, rather
        # than three separate "wait for stopped, then release" additions.
        self.hold_pub.publish(Bool(data=False))
        self._node.get_logger().info(
            f"[mission] '{mission_id}' STARTED -- state=RUNNING from move 0 "
            f'({state.current_move.id!r}) -- /mpc/hold released.'
        )

    def _countdown_seconds(self) -> float:
        """The countdown as it is set RIGHT NOW, in seconds.

        Falls back to the value read at construction when the node has no
        get_parameter -- the unit tests drive this class with a minimal stub,
        and a stub missing one accessor must not change the behaviour under
        test.
        """
        getter = getattr(self._node, 'get_parameter', None)
        if getter is None:
            return self._countdown_s
        try:
            return float(getter('mission_countdown_sec').value)
        except Exception:  # noqa: BLE001 -- an undeclared/odd stub parameter
            return self._countdown_s

    def _cancel_countdown(self) -> bool:
        """Disarm a pending start. True if one was actually armed.

        Every stop path calls this, and the caller must hold self._lock. It is
        safe to call when nothing is armed -- the flag it sets is what stops an
        already-queued _on_countdown_elapsed from starting the car anyway.
        """
        self._start_cancelled = True
        timer, self._countdown_timer = self._countdown_timer, None
        if timer is None:
            return False
        timer.cancel()
        self._node.destroy_timer(timer)
        return True

    def _on_abort_mission_service(
        self, request: Trigger.Request, response: Trigger.Response
    ) -> Trigger.Response:
        with self._lock:
            state: MissionRuntimeState = getattr(self.blackboard, MISSION_KEY)

            # LOADED included (not just RUNNING/HOLDING): a mission that was
            # loaded ahead of time but never started should still be cancellable
            # without ever having to start it first.
            if state.state not in (
                    MissionState.LOADED, MissionState.RUNNING, MissionState.HOLDING):
                response.success = False
                response.message = f'no mission to abort (state={state.state.value})'
                return response

            mission_id = state.config.mission_id if state.config else ''

            # A start that is counting down is cancelled here, before anything
            # else: the car must never begin driving after a stop was asked for.
            was_counting = self._cancel_countdown()

            # Same transition HandleObjectAction's on_object abort_mission action
            # performs (state.abort() + /mpc/hold(True)) -- reused, not
            # reimplemented. See that behaviour's _dispatch() for the other caller.
            # Harmless no-op if the mission was only LOADED (never actually
            # driving): mpc_corr just holds at zero, which it already was.
            state.abort()
            self.hold_pub.publish(Bool(data=True))
            self._pending_event_reason = (
                'cancelled before start' if was_counting
                else 'aborted via /mission/abort_mission'
            )
            self._node.get_logger().error(
                f"[mission] '{mission_id}' ABORTED via /mission/abort_mission service call"
                + (f' -- the pending {self._countdown_seconds():.1f} s start '
                   f'was cancelled.'
                   if was_counting else '.')
            )
            response.success = True
            response.message = f"aborted '{mission_id}'"
        self._publish_status()
        return response

    def _on_emergency_stop_service(
        self, request: Trigger.Request, response: Trigger.Response
    ) -> Trigger.Response:
        with self._lock:
            already_active = self._emergency_stop_active
            self._emergency_stop_active = True
            # Nothing here waits or blocks, so this service stays answerable at
            # any time -- including during a start countdown, which it cancels.
            was_counting = self._cancel_countdown()
            if was_counting:
                state: MissionRuntimeState = getattr(self.blackboard, MISSION_KEY)
                if state.state is MissionState.LOADED:
                    state.abort()
                self.hold_pub.publish(Bool(data=True))
                self._pending_event_reason = 'cancelled before start'
        if was_counting:
            self._node.get_logger().error(
                '[mission] the pending start countdown was CANCELLED by the '
                'emergency stop -- the mission will not begin.'
            )
        self._node.get_logger().error(
            '[mission] EMERGENCY STOP triggered via /mission/emergency_stop -- '
            'latched for the rest of this process\'s lifetime, restart to clear.'
        )
        response.success = True
        response.message = (
            'emergency stop already active' if already_active
            else 'emergency stop engaged'
        )
        self._publish_status()
        return response

    def _load(self, path: str) -> Tuple[bool, str]:
        try:
            config = load_mission_file(path)
        except (MissionConfigError, OSError) as exc:
            # OSError covers file-not-found/unreadable; json.JSONDecodeError is a
            # ValueError subclass, already caught alongside MissionConfigError.
            self._node.get_logger().error(
                f'[mission] REJECTED {path!r}: {exc} -- previous mission (if any) '
                'left running unchanged.'
            )
            return False, str(exc)
        except ValueError as exc:  # json.JSONDecodeError
            self._node.get_logger().error(
                f'[mission] REJECTED {path!r}: malformed JSON: {exc} -- previous '
                'mission (if any) left running unchanged.'
            )
            return False, str(exc)
        except Exception as exc:  # noqa: BLE001 -- deliberate last-resort net, see below
            # Found live: a mission JSON with a field of the wrong TYPE in a spot
            # _parse_move/_parse_stop_condition/etc. don't (or didn't yet) run an
            # explicit isinstance() _require() against -- e.g. `"goal_distance":
            # {"stop_condition": ...}` (an object where a plain number was
            # expected) raised a bare `TypeError: float() argument must be a
            # string or a real number, not 'dict'` straight out of load_mission_
            # file(), which is neither MissionConfigError/OSError nor ValueError,
            # so it propagated out of this method entirely and crashed the whole
            # rclpy spin loop -- taking down the ENTIRE BT node (including the
            # emergency-stop/obstacle-stop lanes, unrelated to mission loading)
            # over a single malformed mission file. mission_config.py's own
            # validation was tightened in response (goal_distance/vdes/
            # timeout_sec now get an explicit numeric-type _require() before
            # float() ever sees them), but this catch-all stays regardless: a
            # service that loads user-authored JSON from disk must never be able
            # to crash this node over *any* future gap in that validation, known
            # or not. Every branch below already promises "reject and leave the
            # previous mission running" -- this is that same promise, just for
            # exception types the two branches above don't happen to name.
            self._node.get_logger().error(
                f'[mission] REJECTED {path!r}: unexpected {type(exc).__name__}: {exc} -- '
                'previous mission (if any) left running unchanged. This likely means a '
                'mission field has the wrong JSON type in a spot mission_config.py does not '
                'yet validate explicitly -- worth tightening there, but this service must '
                'never crash the node over it regardless.'
            )
            return False, f'{type(exc).__name__}: {exc}'

        with self._lock:
            state: MissionRuntimeState = getattr(self.blackboard, MISSION_KEY)
            if self._cancel_countdown():
                self._node.get_logger().warn(
                    '[mission] a new mission was loaded while the previous one '
                    'was counting down -- that start has been cancelled.'
                )
            state.load(config, now=time.monotonic())
            self._current_json_path = path
        self._node.get_logger().info(
            f'[mission] Loaded {path!r}: mission_id={config.mission_id!r} '
            f'{len(config.moves)} move(s) -- state=LOADED, waiting for '
            '/mission/start_mission.'
        )
        self._publish_status()
        return True, f"loaded '{config.mission_id}', {len(config.moves)} moves"

    def _publish_status(self):
        state: MissionRuntimeState = getattr(self.blackboard, MISSION_KEY)
        msg = MissionStatus()
        msg.header.stamp = self._node.get_clock().now().to_msg()
        msg.state = state.state.value
        msg.json_path = self._current_json_path
        msg.emergency_stop_active = self._emergency_stop_active
        self.status_pub.publish(msg)
        self._publish_test_event(state)
        # Read by _on_state_watch_tick to detect transitions this file's own
        # service paths did NOT publish -- see __init__'s own comment.
        self._last_published_state = state.state

    def _publish_test_event(self, state: MissionRuntimeState):
        """One /test/mission_event per lifecycle edge, from here and nowhere else.

        Called only by _publish_status(), which every service path and the
        state watcher already call -- so the three terminal transitions that
        happen inside a BT tick are covered without this file having to
        enumerate them. That is the same reasoning as the MISSION-END EVENT GAP
        FIX in __init__, and the reason there is no second publish call
        anywhere: adding one per call site is exactly how three of them drifted
        out of sync before.

        plan_id is the mission_id. The planner puts that same string in its
        /test/plan_result, so the campaign logger can check that these events
        belong to the test it currently has open.
        """
        reason, self._pending_event_reason = self._pending_event_reason, ''
        event = _TEST_EVENTS.get(state.state)
        if event is None:
            return
        mission_id = state.config.mission_id if state.config else ''
        key = (state.state, mission_id)
        if key == self._last_event_key:
            return              # this edge has already been announced
        self._last_event_key = key
        payload = {'event': event, 'plan_id': mission_id, 'reason': reason}
        if event == 'mission_loaded':
            payload['countdown_s'] = self._countdown_seconds()
        elif event == 'mission_started':
            # Repeated here so a logger that missed mission_loaded still
            # learns the countdown this start actually waited out.
            payload['countdown_s'] = (
                self._armed_countdown_s if self._armed_countdown_s is not None
                else self._countdown_seconds())
        self.test_event_pub.publish(String(data=json.dumps(payload)))
