"""
Per-update COST metric for the two robot_localization EKF instances.

WHY THIS NODE EXISTS. robot_localization's own overload signal is
`ros_filter.cpp:2206`:

    const double loop_elapsed = (this->now() - cur_time).seconds();
    if (loop_elapsed > 1. / frequency_) { "Failed to meet update rate!" }

That test is SELF-REFERENTIAL: it judges the loop against the very parameter
that sets the loop's period. Lowering `frequency` from 50 to 20 raises the bar
from 20ms to 50ms, so a filter whose per-tick cost is unchanged stops warning.
ekf_global was last measured at ~27.5ms of CPU per tick against the 20ms
budget, and that cost has never been explained. Once the budget widens to 50ms
the warning goes quiet and the regression becomes invisible. This node measures
the cost DIRECTLY so it survives the threshold change.

WHAT IS MEASURED, AND WHAT IS NOT. `loop_elapsed` is a wall-clock duration
computed inside a vendored C++ binary this workspace does not build; there is
no way to read it from outside the process. What this node reports instead:

  cpu_ms_per_tick   -- delta(utime + stime) from /proc/<pid> divided by the
                       number of filter ticks over the same window.
  cpu_ms_per_meas   -- the same CPU delta divided by the number of input
                       measurements delivered over that window.
  meas_per_tick     -- input measurements divided by ticks.

cpu_ms_per_tick is deliberately the SAME quantity the 27.5ms figure was derived
from (81.7% of one core divided by 29.7Hz), so a before/after comparison against
that number is apples-to-apples. It is CPU time, not wall time: on a loaded box
it is a LOWER BOUND on loop_elapsed, because preemption inflates the wall
duration without consuming CPU. `period_ms_p90`/`period_ms_max`, computed from
consecutive published header stamps, are the wall-clock companion -- when those
exceed 1/frequency the loop really did overrun.

Splitting cpu_ms_per_tick from cpu_ms_per_meas is the point of the exercise.
Prediction in robot_localization is measurement-driven (FilterBase::
processMeasurement runs one predict + correct per QUEUED MEASUREMENT, not per
timer tick), so total cost behaves as `f * A + B * M` for tick rate f, fixed
per-tick overhead A, per-measurement cost B and measurement rate M. Changing
`frequency` moves only `f * A`. Reporting both terms is what makes it possible
to say afterwards whether a cost change came from the rate change or from input
rate drift -- which a single aggregate CPU percentage cannot distinguish.

TWO INDEPENDENT TICK COUNTS, ON PURPOSE. A readiness probe in this workspace
once reported map->odom at 8.4Hz when the true rate was 29.7Hz: it subscribed
BEST_EFFORT at a shallow depth, could not drain the stream, and reported its own
drop rate as a publisher rate. A cost metric that divides by a tick count is
exactly as vulnerable, so this node never trusts a single source:

  ticks_inproc  -- summed "Events in window" from robot_localization's OWN
                   FrequencyStatus diagnostic (ros_filter.cpp:1992 constructs a
                   HeaderlessTopicDiagnostic named "odometry/filtered", ticked
                   at ros_filter.cpp:2173). Counted INSIDE the filter process,
                   so no subscriber of ours can undercount it.
  ticks_selfcount -- this node's own count of received output messages, taken
                   over a RELIABLE queue 500 deep (the same shape as the
                   recorder fix, for the same reason).

The cost divisions use ticks_inproc when it is available and fall back to
ticks_selfcount when it is not. `tick_count_agreement` reports the ratio; a
value far from 1.0 means one of the two counts is lying and the cost figures in
that same message should not be trusted. Note the two are NOT expected to agree
perfectly: freq_diag_->tick() fires even when `corrected_data` suppresses the
actual publish, so ticks_inproc runs slightly HIGH against ticks_selfcount by
design, not by fault. This needs `print_diagnostics: true` on the filter (both
ekf.yaml and ekf_global.yaml set it); without it only the self-count exists.

MEASUREMENT COUNT IS "DELIVERED", NOT "PROCESSED". This node counts messages
published on each filter's input topics. robot_localization may decline to fuse
some of them (preparePose returning false, the first differential reading, an
out-of-sequence stamp), so the true processed count is a subset. Stated plainly
because it biases cpu_ms_per_meas LOW and meas_per_tick HIGH by whatever that
rejection fraction is; it is the right denominator to first order and the bias
is identical before and after a rate change, so the comparison still holds.

OBSERVER EFFECT. Counting inputs requires subscribing to them, which costs CPU
on a box that has been sitting at 84-93%. Keep this node OFF cores 0,1 (the EKF
pair's reserved budget) -- localization.launch.py pins it away from them for
that reason. The load it adds is the same before and after, so before/after
comparisons stay valid even though absolute numbers carry its overhead.
"""

import time
from typing import Dict, List, Optional

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

import psutil

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)

from rosidl_runtime_py.utilities import get_message


# Matches mission_logger_node's own deep-queue depth, chosen for the same
# reason: a RELIABLE queue this deep does not drop under a burst, so a count
# taken over it is a real count rather than a drop rate. See the module
# docstring's "TWO INDEPENDENT TICK COUNTS" paragraph.
_DEEP_QUEUE_DEPTH = 500

# robot_localization names its output-frequency diagnostic after the topic,
# not the node (ros_filter.cpp:1992-1998), and diagnostic_updater prefixes the
# node name -- so the published status name looks like
# "ekf_filter_node: odometry/filtered". Match on both halves.
_FREQ_DIAG_SUFFIX = 'odometry/filtered'

# diagnostic_updater's FrequencyStatus key. Matched by substring so a
# diagnostic_updater version that repunctuates the label does not silently
# zero the in-process count.
_EVENTS_IN_WINDOW_KEY = 'Events in window'


def _reliable_deep_qos(depth: int = _DEEP_QUEUE_DEPTH) -> QoSProfile:
    """RELIABLE + KEEP_LAST at a depth chosen not to drop under burst."""
    return QoSProfile(
        reliability=QoSReliabilityPolicy.RELIABLE,
        history=QoSHistoryPolicy.KEEP_LAST,
        depth=depth,
        durability=QoSDurabilityPolicy.VOLATILE,
    )


def find_ekf_pid(node_name: str) -> Optional[int]:
    """
    PID of the running `ekf_node` whose remapped node name is `node_name`.

    Both EKF instances are the SAME vendored executable (robot_localization's
    `ekf_node`), launched under a `taskset -c` prefix, so neither the process
    name nor the executable path tells them apart. ROS 2 launch passes the
    node name through as `-r __node:=<name>`, which is the only discriminator
    present in the command line. taskset exec()s into ekf_node rather than
    forking, so the cmdline seen here is ekf_node's own.
    """
    want = f'__node:={node_name}'
    for proc in psutil.process_iter(['pid', 'cmdline']):
        cmdline = proc.info.get('cmdline') or []
        if any(want == arg or arg.endswith(want) for arg in cmdline):
            return proc.info['pid']
    return None


def summarize_periods(stamps_sec: List[float]) -> Dict[str, float]:
    """
    Wall-clock period stats (ms) from consecutive publish header stamps.

    Header stamps, never receive times: a receive time measures when THIS
    node's executor got around to the message, which is precisely the
    measurement artifact this file's docstring warns about. The stamps are
    written by the filter itself.
    """
    if len(stamps_sec) < 2:
        return {'period_ms_p50': 0.0, 'period_ms_p90': 0.0, 'period_ms_max': 0.0}
    deltas = sorted(
        (b - a) * 1e3 for a, b in zip(stamps_sec, stamps_sec[1:]) if b > a)
    if not deltas:
        return {'period_ms_p50': 0.0, 'period_ms_p90': 0.0, 'period_ms_max': 0.0}
    return {
        'period_ms_p50': deltas[len(deltas) // 2],
        'period_ms_p90': deltas[min(len(deltas) - 1, int(0.9 * len(deltas)))],
        'period_ms_max': deltas[-1],
    }


class _FilterProbe:
    """Per-EKF-instance accounting: CPU, ticks, delivered measurements."""

    def __init__(self, owner: Node, label: str, node_name: str,
                 output_topic: str, input_topics: List[str]):
        self._owner = owner
        self.label = label
        self.node_name = node_name
        self.output_topic = output_topic
        self.input_topics = list(input_topics)

        self.pid: Optional[int] = None
        self._proc: Optional[psutil.Process] = None
        self._last_cpu_sec: Optional[float] = None

        self.ticks_selfcount = 0
        self.ticks_inproc = 0
        self.have_inproc = False
        self.meas_count = 0
        self.stamps: List[float] = []

        self._input_subs: Dict[str, object] = {}
        self._output_sub = None

    def try_bind(self) -> None:
        """
        Resolve the PID and create any subscriptions not yet made.

        Called every publish tick rather than once at startup: the filters are
        supervised and can restart underneath this node, which changes their
        PID, and topics do not all exist at bring-up time.
        """
        if self._proc is None or not self._proc.is_running():
            self.pid = find_ekf_pid(self.node_name)
            self._proc = psutil.Process(self.pid) if self.pid else None
            self._last_cpu_sec = None

        if self._output_sub is None:
            self._output_sub = self._subscribe(
                self.output_topic, self._on_output)

        for topic in self.input_topics:
            if topic not in self._input_subs:
                sub = self._subscribe(topic, self._on_input)
                if sub is not None:
                    self._input_subs[topic] = sub

    def _subscribe(self, topic: str, callback):
        """Subscribe using the type the graph reports, or None if absent yet."""
        for name, types in self._owner.get_topic_names_and_types():
            if name == topic and types:
                return self._owner.create_subscription(
                    get_message(types[0]), topic, callback,
                    _reliable_deep_qos())
        return None

    def _on_output(self, msg) -> None:
        self.ticks_selfcount += 1
        header = getattr(msg, 'header', None)
        if header is not None:
            self.stamps.append(
                header.stamp.sec + header.stamp.nanosec * 1e-9)

    def _on_input(self, _msg) -> None:
        self.meas_count += 1

    def note_inproc_events(self, events: int) -> None:
        """Fold one FrequencyStatus window's event count into this interval."""
        self.ticks_inproc += events
        self.have_inproc = True

    def sample(self) -> Dict[str, float]:
        """Consume the interval's counters and return this window's metrics."""
        cpu_delta_sec = 0.0
        if self._proc is not None:
            try:
                times = self._proc.cpu_times()
                cpu_now = times.user + times.system
            except (psutil.NoSuchProcess, psutil.AccessDenied):
                self._proc = None
                self.pid = None
                cpu_now = None
            else:
                if self._last_cpu_sec is not None:
                    cpu_delta_sec = cpu_now - self._last_cpu_sec
                self._last_cpu_sec = cpu_now

        ticks_auth = self.ticks_inproc if self.have_inproc else self.ticks_selfcount
        agreement = (
            self.ticks_inproc / self.ticks_selfcount
            if self.have_inproc and self.ticks_selfcount else 0.0)

        metrics = {
            'pid': float(self.pid or 0),
            'cpu_ms_per_tick': (cpu_delta_sec * 1e3 / ticks_auth) if ticks_auth else 0.0,
            'cpu_ms_per_meas': (
                cpu_delta_sec * 1e3 / self.meas_count) if self.meas_count else 0.0,
            'meas_per_tick': (self.meas_count / ticks_auth) if ticks_auth else 0.0,
            'cpu_percent_of_core': cpu_delta_sec * 100.0,
            'ticks_inproc': float(self.ticks_inproc),
            'ticks_selfcount': float(self.ticks_selfcount),
            'tick_count_agreement': agreement,
            'meas_delivered': float(self.meas_count),
        }
        metrics.update(summarize_periods(self.stamps))

        self.ticks_selfcount = 0
        self.ticks_inproc = 0
        self.have_inproc = False
        self.meas_count = 0
        self.stamps = []
        return metrics


class EkfCostObserverNode(Node):

    def __init__(self):
        super().__init__('ekf_cost_observer_node')

        self.publish_rate_hz = float(
            self.declare_parameter('publish_rate_hz', 1.0).value)

        probes = [
            ('local', 'local_ekf_node_name', 'ekf_filter_node',
             'local_output_topic', '/odometry/filtered',
             'local_input_topics', ['/odom', '/sensors/imu/raw']),
            ('global', 'global_ekf_node_name', 'ekf_global_filter_node',
             'global_output_topic', '/ekf_global/odometry/filtered',
             'global_input_topics', ['/odometry/filtered',
                                     '/slam/pose_calibrated']),
        ]
        self._probes = [
            _FilterProbe(
                self, label,
                str(self.declare_parameter(name_key, name_default).value),
                str(self.declare_parameter(out_key, out_default).value),
                list(self.declare_parameter(in_key, in_default).value))
            for (label, name_key, name_default, out_key, out_default,
                 in_key, in_default) in probes
        ]

        self._pub = self.create_publisher(DiagnosticArray, '/diagnostics', 10)
        # Subscribing to the same topic we publish on: our own messages come
        # back, and are ignored because _on_diagnostics only matches statuses
        # carrying an EKF node name plus the FrequencyStatus suffix.
        self.create_subscription(
            DiagnosticArray, '/diagnostics', self._on_diagnostics,
            _reliable_deep_qos(100))

        self._wall_last = time.monotonic()
        period = 1.0 / max(self.publish_rate_hz, 1e-3)
        self.create_timer(period, self._publish_tick)

        self.get_logger().info(
            f'[ekf_cost] per-update cost metric up at '
            f'{self.publish_rate_hz:.2f} Hz on /diagnostics; watching '
            + ', '.join(p.node_name for p in self._probes))

    def _on_diagnostics(self, msg: DiagnosticArray) -> None:
        for status in msg.status:
            if _FREQ_DIAG_SUFFIX not in status.name:
                continue
            for probe in self._probes:
                if probe.node_name not in status.name:
                    continue
                for kv in status.values:
                    if _EVENTS_IN_WINDOW_KEY in kv.key:
                        try:
                            probe.note_inproc_events(int(float(kv.value)))
                        except ValueError:
                            pass

    def _publish_tick(self) -> None:
        now = time.monotonic()
        window_sec = max(now - self._wall_last, 1e-6)
        self._wall_last = now

        out = DiagnosticArray()
        out.header.stamp = self.get_clock().now().to_msg()

        for probe in self._probes:
            probe.try_bind()
            metrics = probe.sample()
            metrics['window_sec'] = window_sec
            metrics['tick_rate_hz'] = (
                max(metrics['ticks_inproc'], metrics['ticks_selfcount'])
                / window_sec)
            # cpu_percent_of_core was accumulated as CPU-seconds * 100; divide
            # by the real window to make it a percentage of one core.
            metrics['cpu_percent_of_core'] /= window_sec

            status = DiagnosticStatus()
            status.name = f'ekf_cost_observer: {probe.label} EKF per-update cost'
            status.hardware_id = 'none'
            if probe.pid is None:
                status.level = DiagnosticStatus.WARN
                status.message = (
                    f'{probe.node_name} not found in /proc: cost unavailable')
            elif metrics['ticks_inproc'] == 0.0 and metrics['ticks_selfcount'] == 0.0:
                status.level = DiagnosticStatus.WARN
                status.message = f'{probe.node_name} published nothing this window'
            else:
                status.level = DiagnosticStatus.OK
                status.message = (
                    f"{metrics['cpu_ms_per_tick']:.2f}ms/tick, "
                    f"{metrics['cpu_ms_per_meas']:.2f}ms/meas, "
                    f"{metrics['meas_per_tick']:.2f} meas/tick")
            status.values = [
                KeyValue(key=key, value=f'{value:.6g}')
                for key, value in sorted(metrics.items())
            ]
            out.status.append(status)

        self._pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = EkfCostObserverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
