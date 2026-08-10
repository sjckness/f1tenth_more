"""py_trees Condition: is the battery critically low?

Subscribes to /diagnostics/battery_status (f1tenth_messages/BatteryStatus), published
continuously by f1tenth_diagnostics' diagnostics_server_node -- independent of
enable_sys_obs, battery safety monitoring is never gateable (see that node's own
module docstring). The ok/has_data verdict is computed entirely in
diagnostics_server_node against min_battery_voltage; this behaviour does not duplicate
that threshold logic, it only reads the verdict.

Returns FAILURE (not SUCCESS) until at least one BatteryStatus message has been
received (has_data=False) -- a startup race where the BT starts ticking before
diagnostics_server_node has ever seen a /sensors/core sample must NOT be treated as
"battery low," or the emergency lane would trip on every stack startup before the VESC
driver has published anything.
"""

import py_trees

from f1tenth_messages.msg import BatteryStatus


class IsBatteryLow(py_trees.behaviour.Behaviour):

    def __init__(self, name='IsBatteryLow',
                 battery_status_topic='/diagnostics/battery_status'):
        super().__init__(name=name)
        self.battery_status_topic = battery_status_topic
        self.node = None
        self.sub = None
        self.latest = None

    def setup(self, **kwargs):
        try:
            self.node = kwargs['node']
        except KeyError as e:
            raise KeyError("IsBatteryLow.setup() didn't find 'node' in kwargs") from e
        self.sub = self.node.create_subscription(
            BatteryStatus, self.battery_status_topic, self._callback, 10)

    def _callback(self, msg):
        self.latest = msg

    def update(self):
        if self.latest is None or not self.latest.has_data:
            return py_trees.common.Status.FAILURE
        return (
            py_trees.common.Status.FAILURE
            if self.latest.ok
            else py_trees.common.Status.SUCCESS
        )
