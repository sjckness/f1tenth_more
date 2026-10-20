"""The behaviour tree's side of mpc_corr's reverse manoeuvre (Part D).

mpc_corr (mpc_controller/reverse_manoeuvre.py) reverses by itself when it is
stuck, but only while the BT allows it: /bt/allow_reverse, a Bool it must
receive at least every bt_allow_reverse_max_age_s. Everything here exists only
with bt_reverse_integration true (stack_params.yaml). With it false nothing
publishes /bt/allow_reverse and the MPC never starts a reverse, as before.

D1  ReverseIntegration.post_tick publishes /bt/allow_reverse once per tick.
    True only while the mission is RUNNING, no emergency condition other than
    the front stop is tripped (battery, operator e-stop, overheat), and no
    proximity reading blocks the rear: blocks_reverse() on IsProximityTooClose
    and IsObstacleDetected (a side/rear trip, or a front trip under
    front_hard_floor_m). The front trip above the floor does NOT withdraw it:
    that trip is what got the car stuck, and the reverse is the way out.

D2  ReverseModeMonitor holds the MPC's mode from /mpc/reverse_status. While
    it says the MPC is stopping for a reverse or reversing (reverse_active),
    IsProximityTooClose's front cone and IsObstacleDetected's corridor do not
    stop the car for anything at or beyond front_hard_floor_m (their
    should_stop()). Side and rear always stop. A status older than
    max_age_s, or none at all, reads as forward: the front stops are back.

D3  The MPC's 'stuck' (reverse_max_attempts used up) FAILS the current move
    with reason stuck_after_reverse: hold, outcome recorded, summary ABORTED,
    mission ABORTED -- GoToObject's failure path, the only thing this stack
    does with a failed move. Done here, after the tick, rather than in the
    mission subtree: while the front stop trips, the emergency lane wins
    every tick and the mission subtree is never ticked, so a behaviour there
    would never see the give-up.

D4  The mode is written to the blackboard (REVERSE_MODE_KEY), shown by
    MpcReverseMode, a pass-through node in the mission subtree named after
    it, and logged once per change.
"""

import time

from f1tenth_behavior.behaviours.is_battery_low import IsBatteryLow
from f1tenth_behavior.behaviours.is_emergency_stop_triggered import IsEmergencyStopTriggered
from f1tenth_behavior.behaviours.is_obstacle_detected import IsObstacleDetected
from f1tenth_behavior.behaviours.is_proximity_too_close import IsProximityTooClose
from f1tenth_behavior.behaviours.is_system_overheated import IsSystemOverheated
from f1tenth_behavior.mission.move_scoring import record_move_outcome, write_mission_summary
from f1tenth_behavior.mission.object_handler import object_move_wire_id
from f1tenth_behavior.mission.runtime import (
    GLOBAL_TURN_ACCUM_KEY,
    GLOBAL_XY_KEY,
    GLOBAL_YAW_KEY,
    MISSION_KEY,
    MissionState,
)
from f1tenth_messages.msg import ReverseStatus
import py_trees
from std_msgs.msg import Bool, String

REVERSE_MODE_KEY = 'mpc_reverse_mode'
STUCK_OUTCOME = 'stuck_after_reverse'
MODE_STUCK = 'stuck'            # reverse_manoeuvre.MODE_STUCK
MODE_UNKNOWN = 'unknown'        # no fresh /mpc/reverse_status: treated as forward


class ReverseModeMonitor:
    """Hold the MPC's reverse mode as the tree sees it (D2).

    One instance, shared by every behaviour that gates on it; setup()
    subscribes once.
    """

    def __init__(self, max_age_s=0.5, clock=time.monotonic):
        self.max_age_s = float(max_age_s)
        self._clock = clock
        self.msg = None
        self.received = None
        # When 'stuck' was first received after any other mode (D3).
        self.stuck_since = None
        self.sub = None

    def setup(self, node, topic='/mpc/reverse_status'):
        self.sub = node.create_subscription(ReverseStatus, topic, self.on_status, 10)

    def on_status(self, msg):
        now = self._clock()
        if msg.mode == MODE_STUCK:
            if self.stuck_since is None:
                self.stuck_since = now
        else:
            self.stuck_since = None
        self.msg, self.received = msg, now

    def fresh(self) -> bool:
        return self.received is not None and self._clock() - self.received < self.max_age_s

    @property
    def mode(self) -> str:
        return self.msg.mode if self.fresh() else MODE_UNKNOWN

    def reverse_active(self) -> bool:
        """Return True while stopping toward a reverse or reversing; False when stale."""
        return self.fresh() and bool(self.msg.reverse_active)


class MpcReverseMode(py_trees.behaviour.Behaviour):
    """Show the MPC's mode in the BT visualization (D4).

    A pass-through node named after the mode: always SUCCESS, decides nothing.
    """

    def __init__(self, monitor, name='mpc_mode'):
        super().__init__(name=f'{name}: {MODE_UNKNOWN}')
        self._prefix = name
        self.monitor = monitor

    def update(self):
        self.name = f'{self._prefix}: {self.monitor.mode}'
        return py_trees.common.Status.SUCCESS


class ReverseIntegration:
    """Run D1, D3 and D4's blackboard mirror; registered as a post-tick handler."""

    def __init__(self, monitor, root, clock=time.monotonic):
        self.monitor = monitor
        self._clock = clock
        nodes = list(root.iterate())
        self.emergency = [b for b in nodes if isinstance(
            b, (IsBatteryLow, IsEmergencyStopTriggered, IsSystemOverheated))]
        self.front_stops = [b for b in nodes if isinstance(
            b, (IsProximityTooClose, IsObstacleDetected))]
        self.blackboard = py_trees.blackboard.Client(name='ReverseIntegration')
        self.blackboard.register_key(key=MISSION_KEY, access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(key=REVERSE_MODE_KEY, access=py_trees.common.Access.WRITE)
        for key in (GLOBAL_XY_KEY, GLOBAL_YAW_KEY, GLOBAL_TURN_ACCUM_KEY):
            self.blackboard.register_key(key=key, access=py_trees.common.Access.READ)
        self.node = None
        self.allow_pub = self.hold_pub = self.end_pub = None
        self.allowed = False
        self.allow_reason = 'not ticked yet'
        self._logged_mode = MODE_UNKNOWN
        self._logged_allow = None

    def setup(self, node):
        self.node = node
        self.allow_pub = node.create_publisher(Bool, '/bt/allow_reverse', 10)
        self.hold_pub = node.create_publisher(Bool, '/mpc/hold', 10)
        self.end_pub = node.create_publisher(String, '/mpc/goal_object_end', 10)
        self.monitor.setup(node)

    def __call__(self, tree):
        self.post_tick()

    def _bb(self, key):
        try:
            return getattr(self.blackboard, key)
        except (KeyError, AttributeError):
            return None

    def decide(self, state):
        """Return (allowed, reason) from this tick's statuses (D1)."""
        if state is None or state.state is not MissionState.RUNNING:
            return False, 'mission not RUNNING'
        for cond in self.emergency:
            if cond.status == py_trees.common.Status.SUCCESS:
                return False, cond.name
        for stop in self.front_stops:
            if stop.blocks_reverse():
                return False, f'{stop.name} blocks the reverse'
        return True, ''

    def post_tick(self):
        state = self._bb(MISSION_KEY)
        mode = self.monitor.mode
        setattr(self.blackboard, REVERSE_MODE_KEY, mode)
        if mode != self._logged_mode:
            msg = self.monitor.msg
            detail = (f' (attempts {msg.attempts}, last end '
                      f'{msg.last_termination_reason or "-"})' if mode != MODE_UNKNOWN else
                      ' (no fresh /mpc/reverse_status: front stops in force)')
            self.node.get_logger().info(
                f'[reverse] MPC mode {self._logged_mode} -> {mode}{detail}')
            self._logged_mode = mode

        if self._stuck_this_move(state):
            self._fail_move_stuck(state)

        self.allowed, self.allow_reason = self.decide(state)
        self.allow_pub.publish(Bool(data=self.allowed))
        if self.allowed != self._logged_allow:
            self.node.get_logger().info(
                f'[reverse] /bt/allow_reverse {self.allowed}'
                + (f': {self.allow_reason}' if self.allow_reason else ''))
            self._logged_allow = self.allowed

    def _stuck_this_move(self, state) -> bool:
        """Return True if the MPC gave up during the current move.

        A 'stuck' left over from an earlier move (it clears only when mpc_corr
        gets the next goal) started before this move did and does not count.
        """
        since = self.monitor.stuck_since
        return (state is not None and state.state is MissionState.RUNNING
                and self.monitor.mode == MODE_STUCK and since is not None
                and since >= state.move_start_time)

    def _fail_move_stuck(self, state):
        """D3: the same steps as GoToObject's failure outcomes."""
        move = state.current_move
        now = self._clock()
        logger = self.node.get_logger()
        if getattr(move, 'go_to_object', None) is not None:
            record = state.object_record
            if record is not None and record.outcome is None:
                record.outcome = STUCK_OUTCOME
            self.end_pub.publish(String(data=object_move_wire_id(
                state.config.mission_id, state.run_generation, move.id)))
        self.hold_pub.publish(Bool(data=True))
        state.last_stop_reason = f'failed:{STUCK_OUTCOME}'
        logger.error(
            f"[mission] Move '{move.id}' FAILED: {STUCK_OUTCOME} (mpc_corr used up its "
            'reverse attempts without forward progress) -- aborting mission.')
        record_move_outcome(
            state, logger, move, state.last_stop_reason, now,
            end_global_xy=self._bb(GLOBAL_XY_KEY),
            end_global_yaw=self._bb(GLOBAL_YAW_KEY),
            global_turn_accum_deg=self._bb(GLOBAL_TURN_ACCUM_KEY))
        write_mission_summary(
            state.config.mission_id, state.move_outcomes, 'ABORTED',
            state.mission_start_wall_time, logger)
        state.abort()
