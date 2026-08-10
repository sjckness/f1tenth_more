"""py_trees Condition + single-pose goal-forwarding behaviour.

Subscribes to goal_pose_topic (default /goal_pose, matching RViz2's "2D Goal Pose"
tool) and returns SUCCESS once a PoseStamped has actually been received, FAILURE
otherwise -- so the navigation lane (see behavior_executor_node.create_root(),
Sequence: HasGoalPose -> NavigateThroughPosesClient) never sends anything to Nav2
until a real goal has arrived. Before that, both root Selector children fail and the
tree does nothing, same as "no obstacle, no goal" being a legitimate idle state.

Every tick it has a pose, (re-)writes NavigateThroughPoses.Goal(poses=[<latest>]) onto
the blackboard at goal_key for the sibling NavigateThroughPosesClient action-client
behaviour to pick up -- it only sends a new navigate_through_poses goal when the
poses array actually changed, cancelling any in-flight goal first (see its own
docstring). Single-pose only: this behaviour is the sole goal input, wrapped as a
one-element poses list.
"""

import py_trees
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateThroughPoses


class HasGoalPose(py_trees.behaviour.Behaviour):

    def __init__(self, name='HasGoalPose', goal_pose_topic='/goal_pose',
                 goal_key='goal_pose_goal'):
        super().__init__(name=name)
        self.goal_pose_topic = goal_pose_topic
        self.goal_key = goal_key
        self.node = None
        self.sub = None
        self.latest_pose = None
        self.blackboard = self.attach_blackboard_client(name=name)
        self.blackboard.register_key(key=goal_key, access=py_trees.common.Access.WRITE)

    def setup(self, **kwargs):
        try:
            self.node = kwargs['node']
        except KeyError as e:
            raise KeyError("HasGoalPose.setup() didn't find 'node' in kwargs") from e
        self.sub = self.node.create_subscription(
            PoseStamped, self.goal_pose_topic, self._goal_pose_callback, 10)

    def _goal_pose_callback(self, msg):
        self.latest_pose = msg

    def update(self):
        if self.latest_pose is None:
            return py_trees.common.Status.FAILURE
        setattr(self.blackboard, self.goal_key,
                NavigateThroughPoses.Goal(poses=[self.latest_pose]))
        return py_trees.common.Status.SUCCESS
