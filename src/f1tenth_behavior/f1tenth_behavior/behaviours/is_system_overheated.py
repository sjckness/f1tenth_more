"""py_trees Condition: is the Jetson's CPU or GPU over its temp/load threshold?

Subscribes to /diagnostics/system_status (f1tenth_messages/SystemStatus), published by
f1tenth_diagnostics' system_observer_node -- only constructed and added to the tree at
all when enable_sys_obs is true (see behavior_executor_node.create_root()). If sys_obs
is disabled, this behaviour must not exist in the tree at all, not merely report no
data: nothing publishes system_status in that case either, so "no data" and "sys_obs is
off" would be indistinguishable if this behaviour were left in the tree unconditionally
-- the enable_sys_obs check has to happen at tree-construction time, not inside
update().

gpu_temp_c/gpu_percent read 0.0 when jtop isn't connected (system_observer_node's
documented fallback) -- 0.0 never exceeds a positive threshold, so a missing jtop
connection fails safe here (no false trip), it just means this condition can no longer
detect GPU-specific overheating until jtop is reconnected.

=============================================================================
LOAD vs. TEMPERATURE ARE NOW SEPARATE TRIGGERS (2026-09-01 mission-analysis fix)
=============================================================================
The 2026-09-01 bag analysis found this behaviour was the third-largest cause of
unexpected stops -- 3 of 9 stop episodes across the session -- and every one of
those trips was a FALSE POSITIVE fired by the *load* half of the check:

    run 15-06-45  gpu_percent 99.0  -> stop 0.05s later, 0.91s long
    run 15-08-12  gpu_percent 99.1  -> stop 0.05s later, 2.11s long
    run 15-09-43  gpu_percent 96.8  -> stop 0.01s later, 1.00s long

In all three the car had 2.65-4.69 m of clear space and neither perception
condition held for a single sample. Meanwhile temperatures across the WHOLE
session stayed at 55 C CPU / 49 C GPU against a 100 C limit -- the thermal
guard never fired once, and had nothing to do with these stops.

Two things made the load half misfire:

  1. /diagnostics/system_status is published at 1 Hz, and this behaviour
     latches the newest sample. So ONE spiking sample stops the car for a
     full second, and the observed stop durations were exactly 1-2 sample
     periods -- the signature of a latched single sample, not of sustained
     distress.
  2. gpu_percent is genuinely bursty under YOLO inference: consecutive 1 Hz
     samples in run 15-08-12 ranged from 3.0% to 99.1%. A single-sample
     threshold on a signal with that variance is guaranteed to trip on
     normal operation.

The fix deliberately makes the check ACCURATE rather than removing its ability
to act -- this guard protects against hardware self-damage (thermal), which is
a different risk category from collision avoidance and does not get to be
downgraded to a warning just because the car is slow and soft. Concretely:

  - TEMPERATURE (max_temp_c) trips exactly as before, on a single sample, with
    no debounce. It never false-positived, and adding latency to a genuine
    thermal event would be strictly worse.
  - LOAD (max_load_percent) now requires load_trip_consecutive_samples
    CONSECUTIVE over-threshold samples before it can trip (default 3, i.e.
    ~3s at the 1 Hz publish rate). All three false positives above were
    1-2 samples long and would not have tripped under this default.
  - LOAD can also be disabled outright (enable_load_trip=False) without
    touching the thermal guard, for anyone who concludes utilization is simply
    not a safety signal on this board.

The consecutive counter is reset by ANY under-threshold sample, and counts
SAMPLES (system_status callbacks), not BT ticks -- the tree ticks at 10 Hz
against a 1 Hz publisher, so counting ticks would make the debounce depend on
the tree's rate rather than on how long the load actually stayed high.

`tripped_reason` is exposed for behavior_executor_node's BehaviorTreeStatus
publisher, so a bag records WHICH half tripped instead of leaving it to be
re-derived offline.
"""

import py_trees

from f1tenth_messages.msg import SystemStatus


class IsSystemOverheated(py_trees.behaviour.Behaviour):

    def __init__(self, name='IsSystemOverheated',
                 system_status_topic='/diagnostics/system_status',
                 max_temp_c=85.0, max_load_percent=95.0,
                 enable_load_trip=True, load_trip_consecutive_samples=3):
        super().__init__(name=name)
        self.system_status_topic = system_status_topic
        self.max_temp_c = max_temp_c
        self.max_load_percent = max_load_percent
        # See module docstring: load is debounced/disable-able independently of
        # temperature, which is neither.
        self.enable_load_trip = bool(enable_load_trip)
        self.load_trip_consecutive_samples = max(1, int(load_trip_consecutive_samples))
        self.node = None
        self.sub = None
        self.latest = None
        # Counts CONSECUTIVE over-threshold system_status SAMPLES (not ticks).
        self._load_over_streak = 0
        # Set by update(); read by behavior_executor_node for BT telemetry.
        self.tripped_reason = ''

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
        # Streak is advanced here, per received sample, so the debounce measures
        # real elapsed time at the publisher's rate rather than the BT's.
        if self.enable_load_trip and (
                msg.cpu_percent > self.max_load_percent
                or msg.gpu_percent > self.max_load_percent):
            self._load_over_streak += 1
        else:
            self._load_over_streak = 0

    def update(self):
        self.tripped_reason = ''
        if self.latest is None:
            return py_trees.common.Status.FAILURE
        msg = self.latest

        # Temperature: single-sample, undebounced, unchanged. This is the half
        # that actually guards against hardware damage.
        if msg.cpu_temp_c > self.max_temp_c:
            self.tripped_reason = f'cpu_temp_c={msg.cpu_temp_c:.1f}>{self.max_temp_c:.1f}'
            return py_trees.common.Status.SUCCESS
        if msg.gpu_temp_c > self.max_temp_c:
            self.tripped_reason = f'gpu_temp_c={msg.gpu_temp_c:.1f}>{self.max_temp_c:.1f}'
            return py_trees.common.Status.SUCCESS

        # Load: debounced, and only if enabled at all.
        if (self.enable_load_trip
                and self._load_over_streak >= self.load_trip_consecutive_samples):
            self.tripped_reason = (
                f'load cpu={msg.cpu_percent:.1f}/gpu={msg.gpu_percent:.1f} '
                f'>{self.max_load_percent:.1f} for {self._load_over_streak} samples')
            return py_trees.common.Status.SUCCESS

        return py_trees.common.Status.FAILURE
