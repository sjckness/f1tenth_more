"""py_trees Condition leaf for the navigation lane's MPC-mode branch (enable_nav2:=
false -- see behavior_executor_node.create_root()).

Mirrors HasGoalPose's role for the Nav2-mode branch (see that module's docstring) but
for mpc_corr.py's own goal mechanism: subscribes to goal_distance_topic (default
/mpc/goal_distance, std_msgs/Float32 -- see f1tenth_params/config/stack_params.yaml's
own documentation of that runtime-only interface, not a launch parameter) and returns
SUCCESS once any message has been received, FAILURE otherwise.

Presence-based, no staleness/expiry check -- matching mpc_corr.py's own definition of
a valid goal (self.goal_distance is not None), which has no timeout of its own either
(unlike e.g. odom_stale_timeout_sec, a genuinely different continuous-stream staleness
concept for /odom). Same permanent-latch behavior as HasGoalPose: once a goal has ever
arrived, this stays SUCCESS even after mpc_corr reports the goal reached and stops --
mpc_corr itself is what actually stops the car; this leaf only reports "is the
navigation lane meant to be active," not "is the car currently moving."

No blackboard write, unlike HasGoalPose: there is no MPC-mode action-client sibling in
the navigation Sequence -- mpc_corr subscribes to /mpc/goal_distance directly and
drives itself, entirely out of band from the BT (see mpc_corr.launch.py). This leaf
exists purely so the navigation lane's Sequence -- and the BT snapshot log -- correctly
reflect "an MPC goal is active" instead of always failing, which is what happened
before this leaf existed (HasGoalPose was the only condition wired in, and it listens
on the wrong topic/message type entirely for MPC mode -- see create_root()'s enable_nav2
branch).
"""

import py_trees
from std_msgs.msg import Float32


class HasMpcGoal(py_trees.behaviour.Behaviour):

    def __init__(self, name='HasMpcGoal', goal_distance_topic='/mpc/goal_distance'):
        super().__init__(name=name)
        self.goal_distance_topic = goal_distance_topic
        self.node = None
        self.sub = None
        self.received = False

    def setup(self, **kwargs):
        try:
            self.node = kwargs['node']
        except KeyError as e:
            raise KeyError("HasMpcGoal.setup() didn't find 'node' in kwargs") from e
        self.sub = self.node.create_subscription(
            Float32, self.goal_distance_topic, self._goal_distance_callback, 10)

    def _goal_distance_callback(self, msg):
        self.received = True

    def update(self):
        return (py_trees.common.Status.SUCCESS if self.received
                else py_trees.common.Status.FAILURE)
