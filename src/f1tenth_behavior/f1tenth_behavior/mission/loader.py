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
  conditionally costmap_boundary_node/yolo_detector_node depending on what the
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

import os
import threading
import time
from typing import Tuple

import py_trees
from ament_index_python.packages import get_package_share_directory
from f1tenth_messages.msg import MissionStatus
from f1tenth_messages.srv import LoadMission
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger

from f1tenth_behavior.mission.mission_config import MissionConfigError, load_mission_file
from f1tenth_behavior.mission.preflight import check_liveness, required_dependencies
from f1tenth_behavior.mission.runtime import (
    CURRENT_XY_KEY,
    FRONT_CLEARANCE_KEY,
    MIN_OBSTACLE_DISTANCE_KEY,
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


class MissionLoader:

    def __init__(self, node, load_path_topic='/mission/load_path', hold_topic='/mpc/hold',
                 status_topic='/mission/status'):
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
        self.blackboard.register_key(key=CURRENT_XY_KEY, access=py_trees.common.Access.READ)
        self.blackboard.register_key(
            key=MIN_OBSTACLE_DISTANCE_KEY, access=py_trees.common.Access.READ)
        self.blackboard.register_key(
            key=FRONT_CLEARANCE_KEY, access=py_trees.common.Access.READ)

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

        # Establishes the initial latched value immediately (state=IDLE or
        # LOADED depending on the auto-load above, emergency_stop_active=False)
        # so a subscriber that attaches before any service is ever called still
        # gets a well-defined value rather than nothing at all.
        self._publish_status()

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

            # Preflight liveness check (mission/preflight.py) -- confirms every
            # node/data source THIS mission actually needs is alive AND
            # publishing, not just that the service call itself is well-formed.
            # See that module's own docstring for the battery-precheck race
            # this replaces "did it silently not move" with "here's exactly
            # what's missing." Leaves state at LOADED on failure (not
            # consumed/aborted) so the caller can fix the dependency and retry
            # the same /mission/start_mission call.
            reqs = required_dependencies(state.config)
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
                f"[mission] '{mission_id}' STARTED via /mission/start_mission -- "
                f'state=RUNNING from move 0 ({state.current_move.id!r}) -- '
                '/mpc/hold released.'
            )
            response.success = True
            response.message = f"started '{mission_id}'"
        self._publish_status()
        return response

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

            # Same transition HandleObjectAction's on_object abort_mission action
            # performs (state.abort() + /mpc/hold(True)) -- reused, not
            # reimplemented. See that behaviour's _dispatch() for the other caller.
            # Harmless no-op if the mission was only LOADED (never actually
            # driving): mpc_corr just holds at zero, which it already was.
            state.abort()
            self.hold_pub.publish(Bool(data=True))
            self._node.get_logger().error(
                f"[mission] '{mission_id}' ABORTED via /mission/abort_mission service call."
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
