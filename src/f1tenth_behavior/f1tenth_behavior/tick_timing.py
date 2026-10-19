"""Drives the BT from one rclpy timer at bt_tick_hz and measures every tick.

WHY THIS EXISTS (cpu-cleanup, 2026-10-16): behavior_executor_node used
py_trees_ros' tick_tock(), which is the same rclpy timer underneath, but
nothing measured how long a tick took. When the node showed 86% of a core on
the Orin with the car parked, there was no way to tell from the stack itself
whether the tick, a post-tick handler or a subscription callback was paying
for it. BtTicker is that measurement:

  * one timer at 1 / bt_tick_hz calls tree.tick(), timed end to end with
    every pre/post-tick handler included (the tree status publisher, the
    snapshot logger and the visualizer all run inside tree.tick());
  * a WARN (throttled) whenever a tick takes more than warn_fraction of the
    period -- a tick that long delays every subscription callback queued
    behind it on the single-threaded executor, the e-stop's scan included;
  * once per report_period_s, a DiagnosticStatus on /diagnostics (the
    stack's existing diagnostics topic, recorded by the mission logger):
    tick duration p50/p95/max, the measured tick rate and the overrun count
    for that window.

TickTimingStats holds the numbers and has no ROS dependency, so the
percentile and overrun logic is tested without rclpy.
"""

import math
import time

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

DIAGNOSTIC_NAME = 'behavior_executor_node: BT tick timing'


def percentile(sorted_values, q):
    """Nearest-rank percentile of an already sorted, non-empty list."""
    rank = max(1, math.ceil(q / 100.0 * len(sorted_values)))
    return sorted_values[rank - 1]


class TickTimingStats:
    """Tick durations and start times for one reporting window."""

    def __init__(self, period_s, warn_fraction=0.5):
        self.period_s = float(period_s)
        self.budget_s = float(warn_fraction) * self.period_s
        self._durations = []
        self._starts = []
        self._overruns = 0

    def record(self, start_s, duration_s):
        """Fold one tick in; True if it took longer than the budget."""
        self._starts.append(start_s)
        self._durations.append(duration_s)
        over = duration_s > self.budget_s
        if over:
            self._overruns += 1
        return over

    def report_and_reset(self):
        """Summary of the window so far, or None if no tick ran in it."""
        if not self._durations:
            return None
        durations = sorted(self._durations)
        starts = self._starts
        span = starts[-1] - starts[0]
        rate_hz = (len(starts) - 1) / span if len(starts) > 1 and span > 0.0 else 0.0
        report = {
            'ticks': len(durations),
            'p50_ms': 1e3 * percentile(durations, 50),
            'p95_ms': 1e3 * percentile(durations, 95),
            'max_ms': 1e3 * durations[-1],
            'rate_hz': rate_hz,
            'period_ms': 1e3 * self.period_s,
            'budget_ms': 1e3 * self.budget_s,
            'overruns': self._overruns,
        }
        self._durations = []
        self._starts = []
        self._overruns = 0
        return report


def make_diagnostic_array(report, stamp):
    """The /diagnostics message for one TickTimingStats report."""
    status = DiagnosticStatus()
    status.name = DIAGNOSTIC_NAME
    status.hardware_id = 'behavior_executor_node'
    if report['overruns']:
        status.level = DiagnosticStatus.WARN
        status.message = (f"{report['overruns']} tick(s) over the "
                          f"{report['budget_ms']:.0f} ms budget")
    else:
        status.level = DiagnosticStatus.OK
        status.message = 'ticks within budget'
    status.values = [
        KeyValue(key='tick_ms_p50', value=f"{report['p50_ms']:.3f}"),
        KeyValue(key='tick_ms_p95', value=f"{report['p95_ms']:.3f}"),
        KeyValue(key='tick_ms_max', value=f"{report['max_ms']:.3f}"),
        KeyValue(key='ticks', value=str(report['ticks'])),
        KeyValue(key='rate_hz_measured', value=f"{report['rate_hz']:.2f}"),
        KeyValue(key='period_ms', value=f"{report['period_ms']:.1f}"),
        KeyValue(key='budget_ms', value=f"{report['budget_ms']:.1f}"),
        KeyValue(key='overruns', value=str(report['overruns'])),
    ]
    array = DiagnosticArray()
    array.header.stamp = stamp
    array.status = [status]
    return array


class BtTicker:
    """Ticks `tree` from a timer on `node` and reports the tick timing."""

    def __init__(self, node, tree, tick_hz, warn_fraction=0.5, report_period_s=1.0,
                 diagnostics_topic='/diagnostics', clock=time.monotonic):
        if not tick_hz > 0.0:
            raise ValueError(f'bt_tick_hz must be > 0, got {tick_hz!r}')
        self._node = node
        self._tree = tree
        self._clock = clock
        self.period_s = 1.0 / float(tick_hz)
        self.stats = TickTimingStats(self.period_s, warn_fraction)
        self._diag_pub = node.create_publisher(DiagnosticArray, diagnostics_topic, 10)
        self.timer = node.create_timer(self.period_s, self.tick)
        self.report_timer = node.create_timer(report_period_s, self.report)

    def tick(self):
        start = self._clock()
        self._tree.tick()
        duration = self._clock() - start
        if self.stats.record(start, duration):
            self._node.get_logger().warn(
                f'BT tick took {1e3 * duration:.1f} ms, over the '
                f'{1e3 * self.stats.budget_s:.0f} ms budget '
                f'({1e3 * self.period_s:.0f} ms period)', throttle_duration_sec=5.0)

    def report(self):
        report = self.stats.report_and_reset()
        if report is None:
            return
        self._diag_pub.publish(
            make_diagnostic_array(report, self._node.get_clock().now().to_msg()))

    def shutdown(self):
        for timer in (self.timer, self.report_timer):
            timer.cancel()
