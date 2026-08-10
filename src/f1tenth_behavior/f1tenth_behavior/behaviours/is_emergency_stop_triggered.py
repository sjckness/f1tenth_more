"""py_trees Condition: has /mission/emergency_stop been triggered?

Subscribes to /mission/status (f1tenth_messages/msg/MissionStatus), published
by f1tenth_behavior's own MissionLoader (mission/loader.py) with transient-local
QoS -- even though publisher and subscriber currently live in the same process
(behavior_executor_node), that QoS is still what guarantees this behaviour picks
up the correct value immediately in setup(), regardless of the exact ordering
between MissionLoader's construction and this behaviour's own setup() -- the
same guarantee IsBatteryLow/IsSystemOverheated get from their own (cross-process)
publishers. QoS must match MissionLoader's MISSION_STATUS_QOS on both ends for
that latch behavior to actually work -- see mission/loader.py.

emergency_stop_active is latched for the node's lifetime by MissionLoader -- no
reset service exists on purpose (see /mission/emergency_stop's own docstring)
-- so once this condition trips SUCCESS, it stays SUCCESS until the BT process
is restarted. Wired into the emergency lane's inner Selector alongside
IsBatteryLow/IsSystemOverheated (see behavior_executor_node.create_root()),
unconditionally (not gated behind any enable_* toggle, same reasoning as
IsBatteryLow: an emergency stop is never gateable) -- tripping it produces the
exact same Stop (safety_stop mux, priority 200) those two already do.

Returns FAILURE (not SUCCESS) until at least one MissionStatus message has been
received -- same "no data yet must not mean tripped" reasoning as IsBatteryLow's
has_data check.
"""

import py_trees

from f1tenth_messages.msg import MissionStatus


class IsEmergencyStopTriggered(py_trees.behaviour.Behaviour):

    def __init__(self, name='IsEmergencyStopTriggered',
                 mission_status_topic='/mission/status'):
        super().__init__(name=name)
        self.mission_status_topic = mission_status_topic
        self.node = None
        self.sub = None
        self.latest = None

    def setup(self, **kwargs):
        try:
            self.node = kwargs['node']
        except KeyError as e:
            raise KeyError(
                "IsEmergencyStopTriggered.setup() didn't find 'node' in kwargs") from e
        # Local import -- avoids behavior_executor_node importing
        # mission/loader.py (and everything it pulls in) just for this one QoS
        # profile constant when the emergency lane doesn't otherwise depend on
        # the mission subtree at all.
        from f1tenth_behavior.mission.loader import MISSION_STATUS_QOS
        self.sub = self.node.create_subscription(
            MissionStatus, self.mission_status_topic, self._callback, MISSION_STATUS_QOS)

    def _callback(self, msg):
        self.latest = msg

    def update(self):
        if self.latest is None:
            return py_trees.common.Status.FAILURE
        return (
            py_trees.common.Status.SUCCESS
            if self.latest.emergency_stop_active
            else py_trees.common.Status.FAILURE
        )
