"""py_trees Condition: is the Jetson's CPU or GPU over its temp/load threshold?

Subscribes to /diagnostics/system_status (f1tenth_messages/SystemStatus), published by
f1tenth_diagnostics' system_observer_node -- only constructed and added to the tree at
all when enable_sys_obs is true (see behavior_executor_node.create_root()). If sys_obs
is disabled, this behaviour must not exist in the tree at all, not merely report no
data: nothing publishes system_status in that case either, so "no data" and "sys_obs is
off" would be indistinguishable if this behaviour were left in the tree unconditionally
-- the enable_sys_obs check has to happen at tree-construction time, not inside
update().

One shared threshold for both CPU and GPU temp (max_temp_c) and both CPU and GPU load
(max_load_percent) -- both rails are the same silicon-throttle/thermal-shutdown concern
on the same board, a single conservative threshold for each is simpler than four
independent ones and easy to split later if per-rail tuning turns out to matter.
gpu_temp_c/gpu_percent read 0.0 when jtop isn't connected (system_observer_node's
documented fallback) -- 0.0 never exceeds a positive threshold, so a missing jtop
connection fails safe here (no false trip), it just means this condition can no longer
detect GPU-specific overheating until jtop is reconnected.
"""

import py_trees

from f1tenth_messages.msg import SystemStatus


class IsSystemOverheated(py_trees.behaviour.Behaviour):

    def __init__(self, name='IsSystemOverheated',
                 system_status_topic='/diagnostics/system_status',
                 max_temp_c=85.0, max_load_percent=95.0):
        super().__init__(name=name)
        self.system_status_topic = system_status_topic
        self.max_temp_c = max_temp_c
        self.max_load_percent = max_load_percent
        self.node = None
        self.sub = None
        self.latest = None

    def setup(self, **kwargs):
        try:
            self.node = kwargs['node']
        except KeyError as e:
            raise KeyError(
                "IsSystemOverheated.setup() didn't find 'node' in kwargs") from e
        self.sub = self.node.create_subscription(
            SystemStatus, self.system_status_topic, self._callback, 10)

    def _callback(self, msg):
        self.latest = msg

    def update(self):
        if self.latest is None:
            return py_trees.common.Status.FAILURE
        msg = self.latest
        tripped = (
            msg.cpu_temp_c > self.max_temp_c
            or msg.gpu_temp_c > self.max_temp_c
            or msg.cpu_percent > self.max_load_percent
            or msg.gpu_percent > self.max_load_percent
        )
        return py_trees.common.Status.SUCCESS if tripped else py_trees.common.Status.FAILURE
