"""battery_voltage_check_node.py tests -- battery-check-startup-race fix.

Same convention test_gyro_bias_calibration_node.py already uses: construct a
real (never spun) rclpy Node via parameter_overrides and call its
callbacks/_poll()/_finish() directly -- no live topics/hardware, no
executor, pure in-process method calls. Elapsed-time conditions are
simulated by backdating node._start_time/_first_sample_time with
rclpy.duration.Duration (same "age injection" technique
test_costmap_boundary_node.py's TestPeriodicPublishRegardlessOfAge uses on
_map_last_time/_pose_last_time), not real sleeps -- keeps the suite fast and
deterministic.

Covers the fix itself (see battery_voltage_check_node.py's own module
docstring for the full "startup-race" reasoning):
  - The bug this closes: a slow-to-connect VESC (first sample arrives after
    the OLD fixed sample_window_sec, but before the new, longer
    max_wait_for_first_sample_sec ceiling) must now PASS once its averaging
    window completes, not fail-close on a timing artifact.
  - The still-preserved safety intent: genuinely zero samples through the
    whole max_wait_for_first_sample_sec ceiling must still fail closed,
    exactly as before.
  - The averaging window itself only starts counting from the first sample,
    not from node startup -- a slow-to-connect VESC still gets the full
    sample_window_sec of real averaging, not a truncated remainder.
  - Existing pass/fail-on-voltage logic (unchanged) still works once a
    first sample has landed.

Run standalone: python3 -m pytest test/test_battery_voltage_check_node.py -v
"""
import pytest
import rclpy
from rclpy.duration import Duration
from rclpy.parameter import Parameter
from vesc_msgs.msg import VescState, VescStateStamped

from f1tenth_diagnostics.battery_voltage_check_node import BatteryVoltageCheckNode


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


class TestFirstSampleCeiling:
    """The new phase: nothing to average yet, just waiting for telemetry to
    start at all."""

    def test_no_sample_before_ceiling_does_not_fail_yet(self):
        node = _construct_with_params({'max_wait_for_first_sample_sec': 10.0})
        try:
            # Only 1s "elapsed" (well under the 10s ceiling), no sample received.
            node._start_time = node.get_clock().now() - Duration(seconds=1.0)
            node._poll()
            assert node.done is False
        finally:
            node.destroy_node()

    def test_no_sample_at_ceiling_fails_closed(self):
        node = _construct_with_params({'max_wait_for_first_sample_sec': 10.0})
        try:
            node._start_time = node.get_clock().now() - Duration(seconds=10.1)
            node._poll()
            assert node.done is True
            assert node.exit_code == 1
        finally:
            node.destroy_node()


class TestSlowConnectNoLongerFalsePositives:
    """The actual bug this pass fixes: on the old code, ANY sample arriving
    after the old fixed 2.0s window was too late -- _finish() had already
    fired and given up. Reproduces that exact shape (first sample lands well
    after where the OLD window would have expired) and confirms the new code
    still completes a full, real averaging window and passes."""

    def test_first_sample_after_old_2s_window_still_completes_and_passes(self):
        node = _construct_with_params({
            'min_battery_voltage': 10.8,
            'sample_window_sec': 2.0,
            'max_wait_for_first_sample_sec': 10.0,
        })
        try:
            # Simulate 5s having already elapsed since startup (well past the
            # OLD 2.0s deadline, comfortably under the new 10.0s ceiling)
            # before the very first sample arrives.
            node._start_time = node.get_clock().now() - Duration(seconds=5.0)
            node._state_callback(_fake_state(12.5))
            assert node.done is False  # averaging window just started, not done yet

            # Ceiling-check branch must be a no-op now that a sample exists,
            # no matter how much total time has passed.
            node._poll()
            assert node.done is False

            # Averaging window (measured from the first sample, not from
            # startup) completes.
            node._first_sample_time = node._first_sample_time - Duration(seconds=2.1)
            node._poll()

            assert node.done is True
            assert node.exit_code == 0
        finally:
            node.destroy_node()

    def test_first_sample_never_arrives_still_fails_closed(self):
        """The safety intent this pass must NOT weaken: a genuinely
        unconnected VESC still fail-closes, just against the new (longer,
        real-margin) ceiling instead of the old short one."""
        node = _construct_with_params({'max_wait_for_first_sample_sec': 10.0})
        try:
            node._start_time = node.get_clock().now() - Duration(seconds=10.5)
            node._poll()
            assert node.done is True
            assert node.exit_code == 1
        finally:
            node.destroy_node()


class TestAveragingWindowUnchanged:

    def test_mean_above_threshold_passes(self):
        node = _construct_with_params({'min_battery_voltage': 10.8, 'sample_window_sec': 2.0})
        try:
            node._state_callback(_fake_state(12.0))
            node._state_callback(_fake_state(12.4))
            node._first_sample_time = node._first_sample_time - Duration(seconds=2.1)
            node._poll()

            assert node.done is True
            assert node.exit_code == 0
        finally:
            node.destroy_node()

    def test_mean_below_threshold_fails(self):
        node = _construct_with_params({'min_battery_voltage': 10.8, 'sample_window_sec': 2.0})
        try:
            node._state_callback(_fake_state(9.5))
            node._state_callback(_fake_state(9.7))
            node._first_sample_time = node._first_sample_time - Duration(seconds=2.1)
            node._poll()

            assert node.done is True
            assert node.exit_code == 1
        finally:
            node.destroy_node()

    def test_averaging_window_measured_from_first_sample_not_startup(self):
        """A slow-to-connect VESC (first sample well after startup) still
        gets the FULL sample_window_sec of averaging, not a truncated
        remainder -- confirms the window start is _first_sample_time, not
        _start_time."""
        node = _construct_with_params({'sample_window_sec': 2.0})
        try:
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
