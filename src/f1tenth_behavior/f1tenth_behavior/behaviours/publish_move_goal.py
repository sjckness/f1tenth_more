"""py_trees Action: publish the current move's goal to mpc_corr, exactly once
per move (guarded by mission.goal_dirty).

Structural note, flagged explicitly: the task's tree diagram lists this as the
*last* child of the progress Sequence (CheckStopCondition -> AdvanceMove ->
PublishMoveGoal). Taken literally, that has no path to ever publish move 0's
goal at mission start -- PublishMoveGoal is only reached after AdvanceMove
succeeds, which is only reached after CheckStopCondition succeeds, which
mission.begin() correctly can't have happened yet for a move that hasn't been
published a goal for at all. behavior_executor_node.create_root() places this
behaviour *first* in the progress Sequence instead (see its own comment for the
full tick-by-tick reasoning) -- functionally identical to the diagram for every
tick except the very first one, since this behaviour always returns SUCCESS
(published or a harmless no-op) and therefore never blocks CheckStopCondition/
AdvanceMove from running immediately after it in the same tick.

Guard against republishing every tick matters because mpc_corr.goal_distance_
callback resets goal_start_xy (progress tracking) on every message it receives
-- see that method's own docstring -- so a PublishMoveGoal that fired
unconditionally would make mpc_corr's own distance-traveled tracking restart
every single tick and the move would never appear to progress. The same is
true for goal_pose_callback and goal_start_xy's pose-mode counterpart.

Pose-mode moves (move.goal_pose set) publish straight to mpc_corr's own
/mpc/goal_pose input -- deliberately NOT via Nav2/NavigateThroughPoses. Those
are two separate, enable_nav2-gated navigation backends (see
behavior_executor_node's module docstring); only the mpc_corr path keeps
/mpc/hold and mpc_corr's obstacle corridor in effect for mission moves, and
Nav2's own controller has no concept of either. Confirmed with the requester
before wiring this (see chat log) rather than assumed. Position-only, same
scope limit as mpc_corr.py's own goal_pose_callback: yaw is sent (mpc_corr
stores it) but not consumed for arrival.

Turn-mode moves (move.turn set, schema_version 2.0) publish
f1tenth_messages/TurnGoal to mpc_corr's own /mpc/goal_turn input -- the third
of the four mutually-exclusive goal shapes mission_config.py's
Move.turn/goal_distance/goal_pose/drive enforce. Same "publish once per move,
guarded by goal_dirty" shape as the other two; mpc_corr owns everything about
how it actually gets the car turning (see MPC_corr.py's own goal_turn_
callback docstring) -- this behaviour's only job is forwarding the already-
validated turn spec across the ROS boundary, exactly like it does for the
other two goal types.

Drive-mode moves (move.drive set, schema_version 3.0) publish
f1tenth_messages/DriveCommand to /mpc/goal_drive -- the fourth shape, and the
only OPEN-ENDED one. The other three all name a target mpc_corr can decide it
has REACHED, and mpc_corr says so on /mpc/goal_reached; a drive command names
only a MODE, so mpc_corr drives it indefinitely and NEVER publishes
goal_reached for it. The move ends when this tree's own CheckStopCondition
says it ends, and nothing else. That is deliberate and is the whole reason the
shape exists -- see mission_config.DriveSpec.

THE SENTINEL CONVERSION HAPPENS HERE, and only here. DriveSpec models "not
set" as Python None (approach_d_safe) because 0.0 is a legal, meaningful
value; a ROS msg float32 field has no null, so DriveCommand.msg spends a
NEGATIVE value as its "use the node default" sentinel. Translating between the
two is this behaviour's job as the ROS boundary -- mission_config.py stays
ROS-free and never sees the sentinel, and mpc_corr never sees a None. The
other two sentinels (turn_mag_deg == 0, speed == 0) need no conversion: 0 is
already how DriveSpec spells "unset" for both, because neither a zero-degree
turn nor a zero speed is a request anyone can act on.
"""

import math

import py_trees
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32

from f1tenth_messages.msg import DriveCommand, TurnGoal

from f1tenth_behavior.mission.runtime import MISSION_KEY, MissionRuntimeState


class PublishMoveGoal(py_trees.behaviour.Behaviour):

    def __init__(
        self, name='PublishMoveGoal',
        goal_distance_topic='/mpc/goal_distance',
        goal_pose_topic='/mpc/goal_pose',
        goal_turn_topic='/mpc/goal_turn',
        goal_drive_topic='/mpc/goal_drive',
    ):
        super().__init__(name=name)
        self._goal_distance_topic = goal_distance_topic
        self._goal_pose_topic = goal_pose_topic
        self._goal_turn_topic = goal_turn_topic
        self._goal_drive_topic = goal_drive_topic
        self.node = None
        self.goal_distance_pub = None
        self.goal_pose_pub = None
        self.goal_turn_pub = None
        self.goal_drive_pub = None
        self.blackboard = self.attach_blackboard_client(name=name)
        self.blackboard.register_key(key=MISSION_KEY, access=py_trees.common.Access.WRITE)

    def setup(self, **kwargs):
        try:
            self.node = kwargs['node']
        except KeyError as e:
            raise KeyError("PublishMoveGoal.setup() didn't find 'node' in kwargs") from e
        self.goal_distance_pub = self.node.create_publisher(
            Float32, self._goal_distance_topic, 10)
        self.goal_pose_pub = self.node.create_publisher(
            PoseStamped, self._goal_pose_topic, 10)
        self.goal_turn_pub = self.node.create_publisher(
            TurnGoal, self._goal_turn_topic, 10)
        self.goal_drive_pub = self.node.create_publisher(
            DriveCommand, self._goal_drive_topic, 10)

    def update(self):
        state: MissionRuntimeState = getattr(self.blackboard, MISSION_KEY)
        move = state.current_move
        if move is None:
            return py_trees.common.Status.SUCCESS

        if not state.goal_dirty:
            return py_trees.common.Status.SUCCESS

        if move.goal_distance is not None:
            self.goal_distance_pub.publish(Float32(data=float(move.goal_distance)))
            self.node.get_logger().info(
                f"[mission] Move '{move.id}': published goal_distance={move.goal_distance} "
                'to /mpc/goal_distance.'
            )
        elif move.goal_pose is not None:
            # publish straight to mpc_corr's own /mpc/goal_pose input (see
            # module docstring for why this goes to mpc_corr, not Nav2/
            # NavigateThroughPoses).
            gp = move.goal_pose
            pose_msg = PoseStamped()
            pose_msg.header.stamp = self.node.get_clock().now().to_msg()
            pose_msg.header.frame_id = 'map'
            pose_msg.pose.position.x = gp.x
            pose_msg.pose.position.y = gp.y
            pose_msg.pose.orientation.z = math.sin(gp.yaw / 2.0)
            pose_msg.pose.orientation.w = math.cos(gp.yaw / 2.0)
            self.goal_pose_pub.publish(pose_msg)
            self.node.get_logger().info(
                f"[mission] Move '{move.id}': published goal_pose=(x={gp.x}, y={gp.y}, "
                f'yaw={gp.yaw}) to /mpc/goal_pose.'
            )
        elif move.drive is not None:
            # The fourth, open-ended shape (schema_version 3.0) -- see module
            # docstring for why mpc_corr will never report this one reached,
            # and for why the approach_d_safe sentinel is converted here.
            d = move.drive
            drive_msg = DriveCommand()
            drive_msg.mode = d.mode
            drive_msg.turn_sign = float(d.turn_sign)
            drive_msg.turn_mag_deg = float(d.turn_mag_deg)
            drive_msg.speed = float(d.speed)
            # None -> -1.0: "leave mpc_corr's own standoff alone". Any
            # negative value is the sentinel; -1.0 is simply the one this
            # side spends. 0.0 must survive as a real value (no standoff at
            # all), which is exactly why the sentinel is not 0.
            drive_msg.d_safe = (
                -1.0 if d.approach_d_safe is None else float(d.approach_d_safe))
            self.goal_drive_pub.publish(drive_msg)
            self.node.get_logger().info(
                f"[mission] Move '{move.id}': published drive mode={d.mode!r} "
                f'turn_sign={d.turn_sign:+.1f} turn_mag_deg={d.turn_mag_deg:.1f} '
                f'speed={d.speed:.2f} d_safe={drive_msg.d_safe:+.2f} '
                'to /mpc/goal_drive (open-ended -- the stop_condition ends it).'
            )
        else:
            # move.turn is set (mission_config.py guarantees exactly one of
            # the four goal shapes) -- see module docstring.
            t = move.turn
            turn_msg = TurnGoal()
            turn_msg.heading_delta_deg = float(t.heading_delta_deg)
            turn_msg.speed = float(t.speed)
            turn_msg.steering = t.steering
            self.goal_turn_pub.publish(turn_msg)
            self.node.get_logger().info(
                f"[mission] Move '{move.id}': published turn heading_delta_deg="
                f'{t.heading_delta_deg:+.1f} speed={t.speed:.2f} steering={t.steering!r} '
                'to /mpc/goal_turn.'
            )

        state.goal_dirty = False
        return py_trees.common.Status.SUCCESS
