"""Advisory startup battery-voltage report for the VESC drive stack.

Samples VescStateStamped.state.voltage_input on state_topic (default
/sensors/core, published by vesc_driver_node) over a short averaging window
(default 2.0s at vesc_driver_node's 50Hz telemetry poll rate -> ~100
samples), compares the mean against min_battery_voltage (default 10.8V),
reports the result on /diagnostics AND in the log, and ALWAYS exits 0.

ADVISORY, NOT A GATE -- this is the whole point of this node's current
shape, so read this before "restoring" a non-zero exit. It used to fail
closed: any outcome other than a confirmed-good reading exited 1, and
f1tenth_hardware/launch/vesc.launch.py's OnProcessExit handler read that as
"do not launch the drive stack", permanently, with no retry for that whole
boot. ackermann_to_vesc_node and vesc_to_odom_node never started, so nothing
translated /ackermann_drive into VESC commands -- while MPC, twist_mux and
both EKFs came up healthy and looked entirely normal. That failure signature
("the stack is up but the motor gets nothing") was repeatedly misdiagnosed as
a Fast-DDS/discovery problem, because nothing downstream of the abort named
the battery precheck as the cause.

It is not a theoretical risk. Counted over this node's own archived logs
(~/.ros/log, 2026-07-14 to 2026-09-08): 323 aborted boots against 688 passing
ones -- roughly one boot in three -- and 308 of the 323 were the no-telemetry
branch, not a low reading. Raising the first-sample ceiling to 10.0s helped
but did not close it: 40 of those no-telemetry aborts happened after that
change landed, the most recent on 2026-09-08. Confirmed live on the most
recent passing boot, the first telemetry sample landed 7.42s in -- i.e.
routinely within a few seconds of the ceiling.

The no-telemetry aborts are a startup race, not a battery fault: battery
voltage is only observable THROUGH the VESC, so the precheck's data source is
the very thing that is still coming up. Even the low readings deserve
suspicion in that light -- 14 of the 15 recorded low aborts read exactly 9.6V
off n=2 samples, where a healthy 2.0s window at 50Hz yields ~100, so those
means came from a link that had barely started or had already stopped.

What still protects the pack is the RUNTIME battery cutout, which is
unchanged and still fail-closed: f1tenth_diagnostics' diagnostics_server_node
publishes BatteryStatus on /diagnostics/battery_status at 2 Hz from the same
/sensors/core samples, and f1tenth_behavior's IsBatteryLow condition sits
unconditionally in the BT's emergency lane, tripping a Stop onto the
ackermann_mux safety_stop lane (priority 200). That path monitors the battery
continuously, while the car is moving -- which is when a low pack actually
matters -- rather than once, at boot, before the sensor exists. See
test_battery_low_runtime_estop.py in f1tenth_behavior, which fires that path
end to end.

WAITING FOR A READING -- ON MESSAGE ARRIVAL, NOT ON GRAPH DISCOVERY. _poll()
waits for the first VescStateStamped to actually be delivered to
_state_callback, up to max_wait_for_first_sample_sec (default 10.0s); only
then does the sample_window_sec averaging window start. It deliberately does
NOT consult get_node_names()/get_publishers_info_by_topic(): this stack runs
under a Fast-DDS Discovery Server, where the rclpy graph API has been observed
returning nothing while real nodes are alive and publishing, so "is the driver
there yet" is not answerable that way. A delivered message is the only
trustworthy evidence, and it is also the evidence we actually need.

The averaging window (rather than a single sample) is unchanged: VESC UART
telemetry can have occasional dropped/garbled packets, and averaging ~100
samples over 2s is cheap.

THREE OUTCOMES, each named in its own log line by the branch id it fired, and
each published as one DiagnosticStatus on /diagnostics (a topic already in
mission_logger_node's recorded topic list, so a run made on a low battery
carries that fact in its own bag):

  reading_ok                 mean >= min_battery_voltage.  INFO  / level OK.
  reading_low                mean <  min_battery_voltage.  WARN  / level WARN.
  no_reading_before_timeout  no sample at all within
                             max_wait_for_first_sample_sec. WARN / level WARN.

All three exit 0. The branch id is in the log text, in DiagnosticStatus.message
and as the 'branch' KeyValue, because the failure this node's old shape caused
was invisible for months precisely for want of a line naming the cause.

Caveat on the bag, stated rather than implied: this node runs once at boot and
exits, long before mission_logger_node starts recording for a mission. The
/diagnostics status therefore reaches a bag only if a recording happens to be
active at boot. The always-available record of the outcome is this node's log
line; diagnostics_server_node's continuous BatteryStatus is what carries
battery state into the run itself.
"""

import statistics
import sys
import time

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rclpy.node import Node
from vesc_msgs.msg import VescStateStamped

_POLL_PERIOD_SEC = 0.2

# Branch ids. These are the strings that appear in the log line, in
# DiagnosticStatus.message and as the 'branch' KeyValue -- one vocabulary, so
# grepping a log and filtering a bag use the same word.
BRANCH_READING_OK = 'reading_ok'
BRANCH_READING_LOW = 'reading_low'
BRANCH_NO_READING = 'no_reading_before_timeout'

# DiagnosticStatus.name/hardware_id. Namespaced because /diagnostics is a
# shared topic (ekf_cost_observer_node publishes there too).
_DIAG_NAME = 'f1tenth_diagnostics: battery startup precheck'
_DIAG_HARDWARE_ID = 'vesc'


class BatteryVoltageCheckNode(Node):

    def __init__(self, **kwargs):
        # **kwargs forwarded to rclpy.node.Node -- lets tests pass
        # parameter_overrides directly (see gyro_bias_calibration_node.py's
        # own __init__ for the same pattern already established in this
        # package), not otherwise used by main() below.
        super().__init__('battery_voltage_check_node', **kwargs)

        self.state_topic = str(self.declare_parameter('state_topic', '/sensors/core').value)
        self.min_battery_voltage = float(
            self.declare_parameter('min_battery_voltage', 10.8).value)
        self.sample_window_sec = float(
            self.declare_parameter('sample_window_sec', 2.0).value)
        self.max_wait_for_first_sample_sec = float(
            self.declare_parameter('max_wait_for_first_sample_sec', 10.0).value)
        # Kept spinning after the verdict so the /diagnostics publish above
        # actually leaves the process -- publish() only queues, and this node
        # exits immediately afterwards. Costs this much startup latency to the
        # whole drive stack (vesc.launch.py waits for this process to exit
        # before releasing it), so it is deliberately short.
        self.publish_linger_sec = float(
            self.declare_parameter('publish_linger_sec', 0.5).value)

        # ALWAYS 0. Retained rather than removed because vesc.launch.py's
        # OnProcessExit handler still reads the returncode, and a genuine
        # crash (non-zero from an unhandled exception) should still be
        # distinguishable there from a completed advisory check.
        self.exit_code = 0
        self.done = False
        # Which of the three outcomes fired -- one of the BRANCH_* ids above,
        # or None while still waiting. Asserted directly by the tests.
        self.branch = None
        # Mean voltage the verdict was made on, or None on the no-reading
        # branch (where there is genuinely no measurement to report).
        self.measured_voltage = None

        self._samples = []
        self._start_time = self.get_clock().now()
        self._first_sample_time = None

        self._diagnostics_pub = self.create_publisher(
            DiagnosticArray, '/diagnostics', 10)
        self._sub = self.create_subscription(
            VescStateStamped, self.state_topic, self._state_callback, 50)
        # Polls rather than a single one-shot timer: the deadline for "first
        # sample ever" and the deadline for "averaging window complete" are
        # two different, sequential conditions -- _poll() checks whichever one
        # is currently active every tick instead of needing two separately-
        # scheduled timer objects.
        self._poll_timer = self.create_timer(_POLL_PERIOD_SEC, self._poll)

        self.get_logger().info(
            f'[battery_precheck] advisory check starting: waiting up to '
            f'{self.max_wait_for_first_sample_sec:.1f}s for the first telemetry '
            f'sample on "{self.state_topic}", then averaging over '
            f'{self.sample_window_sec:.1f}s (min_battery_voltage='
            f'{self.min_battery_voltage:.1f}V). This check reports; it never '
            'blocks the drive stack.')

    def _state_callback(self, msg: VescStateStamped):
        if self._first_sample_time is None:
            self._first_sample_time = self.get_clock().now()
            elapsed = (self._first_sample_time - self._start_time).nanoseconds / 1e9
            self.get_logger().info(
                f'[battery_precheck] first telemetry sample received after '
                f'{elapsed:.2f}s -- averaging over the next '
                f'{self.sample_window_sec:.1f}s.')
        self._samples.append(msg.state.voltage_input)

    def _poll(self):
        now = self.get_clock().now()

        if self._first_sample_time is None:
            elapsed = (now - self._start_time).nanoseconds / 1e9
            if elapsed >= self.max_wait_for_first_sample_sec:
                self._no_reading(elapsed)
            return

        elapsed_since_first = (now - self._first_sample_time).nanoseconds / 1e9
        if elapsed_since_first >= self.sample_window_sec:
            self._finish()

    def _no_reading(self, elapsed_sec):
        """Report branch 3: nothing was ever measured, so nothing is compared."""
        # Names the timeout that expired and proceeds. Deliberately not phrased
        # as a battery verdict -- no reading was taken, and reading it as "the
        # battery is bad" is precisely the mistake the old fail-closed gate made
        # 308 times.
        self._stop_sampling()
        self.measured_voltage = None
        self.get_logger().warning(
            f'[battery_precheck] branch={BRANCH_NO_READING}: no telemetry '
            f'received on "{self.state_topic}" within '
            f'{self.max_wait_for_first_sample_sec:.1f}s '
            f'(max_wait_for_first_sample_sec), {elapsed_sec:.2f}s elapsed. '
            'Battery voltage was NOT measured, so this is not a battery '
            'verdict -- PROCEEDING with drive stack startup anyway (this check '
            'is advisory). Usual cause is the VESC serial handshake still '
            'being in progress; check vesc_driver_node\'s own log for a '
            '"Connected to VESC" line, and raise '
            'max_wait_for_first_sample_sec if it connected but late. Runtime '
            'battery protection is unaffected: diagnostics_server_node + the '
            'BT IsBatteryLow emergency lane still monitor the pack '
            'continuously.')
        self._publish_outcome(
            BRANCH_NO_READING, DiagnosticStatus.WARN,
            'no telemetry before timeout; battery not measured, proceeding',
            [('state_topic', self.state_topic),
             ('max_wait_for_first_sample_sec',
              f'{self.max_wait_for_first_sample_sec:.3f}'),
             ('elapsed_sec', f'{elapsed_sec:.3f}'),
             ('sample_count', '0')])
        self.done = True

    def _finish(self):
        self._stop_sampling()

        # Always non-empty by construction: _finish() is only reached via
        # _poll()'s second branch, which requires _first_sample_time to
        # already be set, which _state_callback only sets after appending
        # that same first sample.
        voltage = statistics.mean(self._samples)
        self.measured_voltage = voltage
        n = len(self._samples)
        values = [
            ('state_topic', self.state_topic),
            ('voltage_v', f'{voltage:.3f}'),
            ('min_battery_voltage_v', f'{self.min_battery_voltage:.3f}'),
            ('sample_count', str(n)),
        ]

        if voltage >= self.min_battery_voltage:
            self.get_logger().info(
                f'[battery_precheck] branch={BRANCH_READING_OK}: measured '
                f'{voltage:.2f}V >= {self.min_battery_voltage:.2f}V minimum '
                f'(n={n} samples) -- proceeding.')
            self._publish_outcome(
                BRANCH_READING_OK, DiagnosticStatus.OK,
                f'battery {voltage:.2f}V >= {self.min_battery_voltage:.2f}V minimum',
                values)
        else:
            self.get_logger().warning(
                f'[battery_precheck] branch={BRANCH_READING_LOW}: measured '
                f'{voltage:.2f}V is BELOW the {self.min_battery_voltage:.2f}V '
                f'minimum (n={n} samples, deficit '
                f'{self.min_battery_voltage - voltage:.2f}V) -- PROCEEDING with '
                'drive stack startup anyway (this check is advisory). Charge or '
                'replace the pack. Runtime protection is still armed: '
                'diagnostics_server_node publishes BatteryStatus against this '
                'same threshold and the BT IsBatteryLow emergency lane will '
                'stop the car if it stays low.')
            self._publish_outcome(
                BRANCH_READING_LOW, DiagnosticStatus.WARN,
                f'LOW BATTERY {voltage:.2f}V < '
                f'{self.min_battery_voltage:.2f}V minimum, proceeding anyway',
                values)
        self.done = True

    def _stop_sampling(self):
        self._poll_timer.cancel()
        self._sub.destroy()

    def _publish_outcome(self, branch, level, message, values):
        """Record the verdict on /diagnostics as one DiagnosticStatus."""
        self.branch = branch
        status = DiagnosticStatus()
        status.level = level
        status.name = _DIAG_NAME
        status.hardware_id = _DIAG_HARDWARE_ID
        status.message = f'{branch}: {message}'
        status.values = (
            [KeyValue(key='branch', value=branch)]
            + [KeyValue(key=k, value=v) for k, v in values])

        array = DiagnosticArray()
        array.header.stamp = self.get_clock().now().to_msg()
        array.status = [status]
        self._diagnostics_pub.publish(array)


def main():
    rclpy.init()
    node = BatteryVoltageCheckNode()
    try:
        # NOT rclpy.spin(node): calling rclpy.shutdown() from inside a callback
        # running under the executor deadlocks (executor.shutdown() waits for that
        # same callback to finish -- confirmed live). Callbacks only set node.done;
        # shutdown happens here, in the main thread, once the loop notices it.
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.2)
        # Keep spinning briefly so the /diagnostics publish is actually handed
        # to the middleware before destroy_node() below tears the publisher
        # down. Spins rather than sleeps, deliberately -- the executor has to
        # keep running for delivery to happen at all.
        linger_deadline = time.monotonic() + node.publish_linger_sec
        while rclpy.ok() and time.monotonic() < linger_deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
    except KeyboardInterrupt:
        pass
    finally:
        exit_code = node.exit_code
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    sys.exit(exit_code)


if __name__ == '__main__':
    main()
