"""Custom navigate_through_poses action-client behaviour.

NOT py_trees_ros.action_clients.FromBlackboard: that library behaviour's "has the
goal changed since last sent" comparison is internal to py_trees_ros and wasn't
verifiable in this environment (py_trees_ros isn't installed in the dev sandbox this
was written in) -- since this behaviour needs to explicitly compare whole pose arrays
and explicitly cancel an in-flight goal before resending, both are implemented
directly here against a real rclpy.action.ActionClient instead of trusting/extending
an opaque library internal.

Reads a NavigateThroughPoses.Goal (poses: PoseStamped[]) from the blackboard at
goal_key -- written by HasGoalPose from the single /goal_pose input, wrapped as a
one-element poses list (see its own docstring). Each tick:
  - No value on the blackboard, or an empty poses list -> FAILURE. (Doesn't happen in
    practice once the navigation Sequence's HasGoalPose gate has already passed, since
    that gate makes the same check -- kept here too so this behaviour is safe even if
    ticked standalone.)
  - The poses list differs (by per-pose frame_id + position + orientation, NOT
    header.stamp or object identity) from the array last sent -> cancel the in-flight
    goal if one exists, then send the new array as one navigate_through_poses call.
  - Identical to the last-sent array -> don't resend; report RUNNING/SUCCESS/FAILURE
    from the in-flight (or just-completed) goal's own status.

Known limitation: if a second, different goal arrives while the first is still waiting
on goal-acceptance (send_goal_async hasn't resolved yet), the in-flight cancel only
covers an already-accepted goal_handle -- a goal still in the accept-pending window
isn't explicitly cancelled before the new send. Not addressed here; flagged as a
narrow race, not expected to matter at this BT's tick rate vs. typical Nav2 accept
latency.
"""

import py_trees
from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateThroughPoses
from rclpy.action import ActionClient


def _pose_key(pose_stamped):
    """Comparable tuple of a PoseStamped's frame_id + position + orientation.
    Deliberately excludes header.stamp -- a goal re-published with a fresh timestamp
    but identical pose is still logically the same goal, not a new one."""
    p = pose_stamped.pose.position
    o = pose_stamped.pose.orientation
    return (
        pose_stamped.header.frame_id,
        p.x, p.y, p.z,
        o.x, o.y, o.z, o.w,
    )


def _poses_key(poses):
    return tuple(_pose_key(p) for p in poses)


class NavigateThroughPosesClient(py_trees.behaviour.Behaviour):

    def __init__(self, name='NavigateThroughPoses', goal_key='goal_pose_goal',
                 action_name='navigate_through_poses'):
        super().__init__(name=name)
        self.goal_key = goal_key
        self.action_name = action_name
        self.node = None
        self.action_client = None
        self.blackboard = self.attach_blackboard_client(name=name)
        self.blackboard.register_key(key=goal_key, access=py_trees.common.Access.READ)

        self._last_sent_key = None
        self._goal_handle = None
        self._latest_status = None  # action_msgs/GoalStatus.STATUS_* once known

    def setup(self, **kwargs):
        try:
            self.node = kwargs['node']
        except KeyError as e:
            raise KeyError(
                "NavigateThroughPosesClient.setup() didn't find 'node' in kwargs") from e
        self.action_client = ActionClient(self.node, NavigateThroughPoses, self.action_name)

    def update(self):
        if not self.blackboard.exists(self.goal_key):
            return py_trees.common.Status.FAILURE
        poses = getattr(self.blackboard, self.goal_key).poses
        if not poses:
            return py_trees.common.Status.FAILURE

        new_key = _poses_key(poses)

        if new_key != self._last_sent_key:
            self._cancel_in_flight()
            self._send_goal(poses)
            self._last_sent_key = new_key
            return py_trees.common.Status.RUNNING

        return self._current_status()

    def _cancel_in_flight(self):
        if self._goal_handle is not None:
            self.node.get_logger().info(
                f'{self.action_name}: new goal array differs from in-flight one -- '
                'cancelling before resending.')
            self._goal_handle.cancel_goal_async()
            self._goal_handle = None
        self._latest_status = None

    def _send_goal(self, poses):
        goal = NavigateThroughPoses.Goal(poses=list(poses))
        send_goal_future = self.action_client.send_goal_async(goal)
        send_goal_future.add_done_callback(self._on_goal_response)

    def _on_goal_response(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.node.get_logger().error(
                f'{self.action_name} goal rejected by the action server.')
            self._latest_status = GoalStatus.STATUS_ABORTED
            return
        self._goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_result)

    def _on_result(self, future):
        self._latest_status = future.result().status

    def _current_status(self):
        if self._latest_status is None:
            return py_trees.common.Status.RUNNING
        if self._latest_status == GoalStatus.STATUS_SUCCEEDED:
            return py_trees.common.Status.SUCCESS
        if self._latest_status in (GoalStatus.STATUS_CANCELED, GoalStatus.STATUS_ABORTED):
            return py_trees.common.Status.FAILURE
        return py_trees.common.Status.RUNNING
