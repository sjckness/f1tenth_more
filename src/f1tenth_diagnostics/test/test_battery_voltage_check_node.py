"""Tests for battery_voltage_check_node's three advisory outcome branches.

Same convention test_gyro_bias_calibration_node.py already uses: construct a
real (never spun) rclpy Node via parameter_overrides and call its
callbacks/_poll()/_finish() directly -- no live topics/hardware, no
executor, pure in-process method calls. Elapsed-time conditions are
simulated by backdating node._start_time/_first_sample_time with
rclpy.duration.Duration (same "age injection" technique
test_costmap_boundary_node.py's TestPeriodicPublishRegardlessOfAge uses on
_map_last_time/_pose_last_time), not real sleeps -- keeps the suite fast and
deterministic.

THE PROPERTY THESE TESTS EXIST TO HOLD: this check is advisory. Every branch
exits 0 and lets the drive stack launch. The node used to exit 1 on both bad
branches, which made vesc.launch.py skip ackermann_to_vesc_node/
vesc_to_odom_node for that entire boot with no retry -- 323 archived boots hit
that, 308 of them because VESC telemetry had not started yet rather than
because the pack was flat. See the node's own module docstring for the full
account, and test_battery_low_runtime_estop.py in f1tenth_behavior for the
runtime cutout that is what actually protects the pack.

Covered here:
  - reading arrives and is fine       -> branch reading_ok, level OK
  - reading arrives and is low        -> branch reading_low, level WARN
  - no reading before the timeout     -> branch no_reading_before_timeout,
                                         level WARN
  - none of the three aborts (exit_code stays 0 in all of them)
  - every branch names itself in its log line and on /diagnostics
  - the /diagnostics status really reaches a subscriber (live pub/sub)
  - the pre-existing slow-connect and averaging-window behaviour, unchanged

Run standalone: python3 -m pytest test/test_battery_voltage_check_node.py -v
"""
import time

import pytest
import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from rclpy.duration import Duration
from rclpy.parameter import Parameter
from vesc_msgs.msg import VescState, VescStateStamped

from f1tenth_diagnostics.battery_voltage_check_node import (
    BRANCH_NO_READING,
    BRANCH_READING_LOW,
    BRANCH_READING_OK,
    BatteryVoltageCheckNode,
)


@pytest.fixture(scope='module', autouse=True)
def _rclpy_context():
    rclpy.init()
    yield
    if rclpy.ok():
        rclpy.shutdown()


def _construct_with_params(param_dict=None):
    overrides = [Parameter(k, value=v) for k, v in (param_dict or {}).items()]
    return BatteryVoltageCheckNode(parameter_overrides=overrides)


def _fake_state(voltage):
    msg = VescStateStamped()
    msg.state = VescState()
    msg.state.voltage_input = voltage
    return msg


class _CapturedLogger:
    """Stand-in for the node's rclpy logger, recording level and text."""

    def __init__(self):
        self.lines = []

    def info(self, text):
        self.lines.append(('info', text))

    def warning(self, text):
        self.lines.append(('warning', text))

    def error(self, text):
        self.lines.append(('error', text))

    def text_at(self, level):
        return ' '.join(t for lvl, t in self.lines if lvl == level)


def _instrument(node):
    """Capture the node's outcome log lines and /diagnostics publishes."""
    logger = _CapturedLogger()
    published = []
    node.get_logger = lambda: logger
    node._diagnostics_pub.publish = published.append
    return logger, published


def _only_status(published):
    """Return the single DiagnosticStatus from a single published array."""
    assert len(published) == 1, f'expected exactly one publish, got {len(published)}'
    array = published[0]
    assert isinstance(array, DiagnosticArray)
    assert len(array.status) == 1
    return array.status[0]


def _values(status):
    """Return a DiagnosticStatus' KeyValue list as a plain dict."""
    return {kv.key: kv.value for kv in status.values}


class TestReadingArrivesAndIsFine:
    """Branch 1: a real reading at or above the threshold."""

    def test_branch_is_reading_ok_and_does_not_abort(self):
        node = _construct_with_params(
            {'min_battery_voltage': 10.8, 'sample_window_sec': 2.0})
        try:
            logger, published = _instrument(node)
            node._state_callback(_fake_state(12.0))
            node._state_callback(_fake_state(12.4))
            node._first_sample_time = node._first_sample_time - Duration(seconds=2.1)
            node._poll()

            assert node.done is True
            assert node.branch == BRANCH_READING_OK
            assert node.exit_code == 0
            assert node.measured_voltage == pytest.approx(12.2)
        finally:
            node.destroy_node()

    def test_diagnostics_status_is_ok_level_and_carries_the_numbers(self):
        node = _construct_with_params(
            {'min_battery_voltage': 10.8, 'sample_window_sec': 2.0})
        try:
            _, published = _instrument(node)
            node._state_callback(_fake_state(12.0))
            node._state_callback(_fake_state(12.4))
            node._first_sample_time = node._first_sample_time - Duration(seconds=2.1)
            node._poll()

            status = _only_status(published)
            assert status.level == DiagnosticStatus.OK
            values = _values(status)
            assert values['branch'] == BRANCH_READING_OK
            assert float(values['voltage_v']) == pytest.approx(12.2)
            assert float(values['min_battery_voltage_v']) == pytest.approx(10.8)
            assert values['sample_count'] == '2'
        finally:
            node.destroy_node()

    def test_log_line_names_the_branch(self):
        node = _construct_with_params(
            {'min_battery_voltage': 10.8, 'sample_window_sec': 2.0})
        try:
            logger, _ = _instrument(node)
            node._state_callback(_fake_state(12.0))
            node._first_sample_time = node._first_sample_time - Duration(seconds=2.1)
            node._poll()

            assert f'branch={BRANCH_READING_OK}' in logger.text_at('info')
        finally:
            node.destroy_node()


class TestReadingArrivesAndIsLow:
    """Branch 2: a real reading below the threshold. Warns, never aborts."""

    def test_low_reading_warns_and_still_exits_zero(self):
        node = _construct_with_params(
            {'min_battery_voltage': 10.8, 'sample_window_sec': 2.0})
        try:
            logger, _ = _instrument(node)
            node._state_callback(_fake_state(9.5))
            node._state_callback(_fake_state(9.7))
            node._first_sample_time = node._first_sample_time - Duration(seconds=2.1)
            node._poll()

            assert node.done is True
            assert node.branch == BRANCH_READING_LOW
            # THE regression this file exists for: a flat pack must no longer
            # take the drive stack down with it at boot.
            assert node.exit_code == 0
            assert node.measured_voltage == pytest.approx(9.6)
            assert logger.lines and logger.lines[-1][0] == 'warning'
        finally:
            node.destroy_node()

    def test_warning_states_measured_voltage_against_the_threshold(self):
        node = _construct_with_params(
            {'min_battery_voltage': 10.8, 'sample_window_sec': 2.0})
        try:
            logger, _ = _instrument(node)
            node._state_callback(_fake_state(9.6))
            node._first_sample_time = node._first_sample_time - Duration(seconds=2.1)
            node._poll()

            warned = logger.text_at('warning')
            assert f'branch={BRANCH_READING_LOW}' in warned
            assert '9.60V' in warned
            assert '10.80V' in warned
        finally:
            node.destroy_node()

    def test_diagnostics_status_is_warn_level_and_carries_the_numbers(self):
        node = _construct_with_params(
            {'min_battery_voltage': 10.8, 'sample_window_sec': 2.0})
        try:
            _, published = _instrument(node)
            node._state_callback(_fake_state(9.6))
            node._first_sample_time = node._first_sample_time - Duration(seconds=2.1)
            node._poll()

            status = _only_status(published)
            assert status.level == DiagnosticStatus.WARN
            assert status.message.startswith(f'{BRANCH_READING_LOW}:')
            values = _values(status)
            assert values['branch'] == BRANCH_READING_LOW
            assert float(values['voltage_v']) == pytest.approx(9.6)
            assert float(values['min_battery_voltage_v']) == pytest.approx(10.8)
        finally:
            node.destroy_node()


class TestNoReadingBeforeTimeout:
    """Branch 3: telemetry never arrived, so nothing was measured at all."""

    def test_before_the_timeout_nothing_is_decided_yet(self):
        node = _construct_with_params({'max_wait_for_first_sample_sec': 10.0})
        try:
            _instrument(node)
            # Only 1s "elapsed" (well under the 10s ceiling), no sample received.
            node._start_time = node.get_clock().now() - Duration(seconds=1.0)
            node._poll()

            assert node.done is False
            assert node.branch is None
        finally:
            node.destroy_node()

    def test_at_the_timeout_it_warns_and_still_exits_zero(self):
        node = _construct_with_params({'max_wait_for_first_sample_sec': 10.0})
        try:
            logger, _ = _instrument(node)
            node._start_time = node.get_clock().now() - Duration(seconds=10.1)
            node._poll()

            assert node.done is True
            assert node.branch == BRANCH_NO_READING
            # The branch that caused 308 of the 323 archived aborted boots.
            assert node.exit_code == 0
            # No reading was taken, so there is deliberately no voltage to report.
            assert node.measured_voltage is None
            assert logger.lines and logger.lines[-1][0] == 'warning'
        finally:
            node.destroy_node()

    def test_warning_names_the_branch_and_the_timeout_that_expired(self):
        node = _construct_with_params({'max_wait_for_first_sample_sec': 10.0})
        try:
            logger, _ = _instrument(node)
            node._start_time = node.get_clock().now() - Duration(seconds=10.1)
            node._poll()

            warned = logger.text_at('warning')
            assert f'branch={BRANCH_NO_READING}' in warned
            assert 'max_wait_for_first_sample_sec' in warned
            assert '10.0s' in warned
        finally:
            node.destroy_node()

    def test_diagnostics_status_is_warn_level_with_no_voltage_claimed(self):
        node = _construct_with_params({'max_wait_for_first_sample_sec': 10.0})
        try:
            _, published = _instrument(node)
            node._start_time = node.get_clock().now() - Duration(seconds=10.1)
            node._poll()

            status = _only_status(published)
            assert status.level == DiagnosticStatus.WARN
            values = _values(status)
            assert values['branch'] == BRANCH_NO_READING
            assert values['sample_count'] == '0'
            # Reporting a voltage here would be inventing a measurement.
            assert 'voltage_v' not in values
            assert float(values['max_wait_for_first_sample_sec']) == pytest.approx(10.0)
        finally:
            node.destroy_node()


class TestNoBranchAborts:
    """Cross-branch invariant: none of the three ever gates the drive stack."""

    def test_all_three_branches_exit_zero(self):
        seen = {}

        low = _construct_with_params({'min_battery_voltage': 10.8})
        try:
            _instrument(low)
            low._state_callback(_fake_state(9.0))
            low._first_sample_time = low._first_sample_time - Duration(seconds=2.1)
            low._poll()
            seen[low.branch] = low.exit_code
        finally:
            low.destroy_node()

        ok = _construct_with_params({'min_battery_voltage': 10.8})
        try:
            _instrument(ok)
            ok._state_callback(_fake_state(12.5))
            ok._first_sample_time = ok._first_sample_time - Duration(seconds=2.1)
            ok._poll()
            seen[ok.branch] = ok.exit_code
        finally:
            ok.destroy_node()

        none = _construct_with_params({'max_wait_for_first_sample_sec': 10.0})
        try:
            _instrument(none)
            none._start_time = none.get_clock().now() - Duration(seconds=10.1)
            none._poll()
            seen[none.branch] = none.exit_code
        finally:
            none.destroy_node()

        assert seen == {
            BRANCH_READING_OK: 0,
            BRANCH_READING_LOW: 0,
            BRANCH_NO_READING: 0,
        }


class TestDiagnosticsReallyReachesASubscriber:
    """The publish is not just constructed -- it goes out over the topic."""

    def test_low_reading_status_is_delivered_on_diagnostics(self):
        node = _construct_with_params(
            {'min_battery_voltage': 10.8, 'sample_window_sec': 2.0})
        try:
            received = []
            node.create_subscription(
                DiagnosticArray, '/diagnostics', received.append, 10)

            node._state_callback(_fake_state(9.6))
            node._first_sample_time = node._first_sample_time - Duration(seconds=2.1)
            node._poll()

            # Same spin-with-timeout shape main()'s linger loop uses; no sleep,
            # so callbacks keep being serviced while we wait.
            deadline = time.monotonic() + 5.0
            while time.monotonic() < deadline and not received:
                rclpy.spin_once(node, timeout_sec=0.05)

            assert received, '/diagnostics status was never delivered'
            statuses = [s for msg in received for s in msg.status
                        if s.name.endswith('battery startup precheck')]
            assert statuses, 'no battery precheck status among the received arrays'
            assert statuses[0].level == DiagnosticStatus.WARN
            assert BRANCH_READING_LOW in statuses[0].message
        finally:
            node.destroy_node()


class TestSlowConnectStillCompletesTheWindow:
    """A late first sample still gets a full averaging window, not a stub."""

    def test_first_sample_after_old_2s_window_still_completes(self):
        node = _construct_with_params({
            'min_battery_voltage': 10.8,
            'sample_window_sec': 2.0,
            'max_wait_for_first_sample_sec': 10.0,
        })
        try:
            _instrument(node)
            # Simulate 5s having already elapsed since startup (well past the
            # OLD 2.0s deadline, comfortably under the 10.0s ceiling) before
            # the very first sample arrives.
            node._start_time = node.get_clock().now() - Duration(seconds=5.0)
            node._state_callback(_fake_state(12.5))
            assert node.done is False  # averaging window just started

            # Ceiling-check branch must be a no-op now that a sample exists,
            # no matter how much total time has passed.
            node._poll()
            assert node.done is False

            node._first_sample_time = node._first_sample_time - Duration(seconds=2.1)
            node._poll()

            assert node.done is True
            assert node.branch == BRANCH_READING_OK
        finally:
            node.destroy_node()

    def test_averaging_window_measured_from_first_sample_not_startup(self):
        node = _construct_with_params({'sample_window_sec': 2.0})
        try:
            _instrument(node)
            node._start_time = node.get_clock().now() - Duration(seconds=8.0)
            node._state_callback(_fake_state(12.0))  # first sample at "elapsed=8.0s"

            # Only 1s since the first sample (< 2.0s window) -- must not be
            # done yet even though 9s have passed since node startup.
            node._poll()
            assert node.done is False

            node._first_sample_time = node._first_sample_time - Duration(seconds=2.1)
            node._poll()
            assert node.done is True
        finally:
            node.destroy_node()


if __name__ == '__main__':
    import sys
    sys.exit(pytest.main([__file__, '-v']))
