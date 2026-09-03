"""py_trees port of f1tenth_behavior's old BT.CPP StopAction.

Publishes a zero-speed AckermannDriveStamped every tick, onto the ackermann_mux's
highest-priority lane (see f1tenth_bringup/config/mux.yaml: topic "safety_stop",
priority 200, timeout 0.2s). update() always returns SUCCESS -- the "keep stopping
while blocked" behavior comes from the root Selector re-ticking every cycle, not from
this behaviour holding RUNNING. Publishing simply stops (no explicit "resume" message)
once the tree stops ticking this node; the mux lane's own timeout (owned entirely by
mux.yaml, not by this behaviour) is what lets the next-highest lane take back over.

frame_id IS LOAD-BEARING FOR ANALYSIS, not decoration. The ackermann_mux
republishes the winning source message VERBATIM onto /ackermann_drive, so
header.frame_id is what a recorded bag can use to attribute an actuated
command to its source. Two conventions depend on it (see
behavior_executor_node.create_root()):

  ''                    -- MPC_corr._publish_drive (no frame_id set)
  'base_link/emergency' -- the emergency lane's Stop
  'base_link/obstacle'  -- the handle_obstacle lane's Stop

The 2026-09-01 mission analysis could separate MPC commands from BT stops this
way (35% of all actuated samples turned out to be BT stops), but NOT emergency
from obstacle stops, because both instances then stamped a bare 'base_link' --
so all nine stop episodes had to be re-bucketed offline by replaying each
condition's predicate against recorded sensor data. Any new Stop instance
should take its own distinct frame_id for the same reason.

Note the default here is still the bare 'base_link': a Stop constructed
without an explicit frame_id is not attributable, and callers are expected to
pass one.
"""

import py_trees
from ackermann_msgs.msg import AckermannDriveStamped


class Stop(py_trees.behaviour.Behaviour):

    def __init__(self, name='Stop', output_topic='safety_stop', frame_id='base_link'):
        super().__init__(name=name)
        self.output_topic = output_topic
        self.frame_id = frame_id
        self.node = None
        self.pub = None

    def setup(self, **kwargs):
        try:
            self.node = kwargs['node']
        except KeyError as e:
            raise KeyError("Stop.setup() didn't find 'node' in kwargs") from e
        self.pub = self.node.create_publisher(AckermannDriveStamped, self.output_topic, 10)

    def update(self):
        msg = AckermannDriveStamped()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.drive.speed = 0.0
        msg.drive.steering_angle = 0.0
        self.pub.publish(msg)
        return py_trees.common.Status.SUCCESS
