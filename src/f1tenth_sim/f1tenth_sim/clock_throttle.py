"""Clock throttle: republishes Gazebo's 1 kHz clock on /clock at a lower rate.

Why (output/sim_clock_fanout.md): Gazebo publishes one clock message per
physics step, 1000/s with the 1 ms step. Every node on use_sim_time subscribes
to /clock, and between two machines Fast DDS (Discovery Server, unicast) sends
one UDP packet per remote subscriber per message: ~40 subscribers on the Thor
x 1000 Hz = ~40,000 packets/s for /clock alone. The bridge's writer could not
keep up, /clock reached the Thor in waves (seconds with no clock, then
catch-up bursts), and every sim-time timer on the Thor froze and burst with it.

So ros_gz_bridge now puts Gazebo's clock on /sim/clock_raw (subscribed only
here, on the sim PC, through shared memory) and this node republishes it on
/clock at rate_hz (default 200 Hz, 5 ms resolution, 5x less network traffic)
while physics keeps its 1 ms step.

Decimation is on SIM time, not wall time, and snapped to a grid: a message is
forwarded when its time enters a new rate_hz bin (floor(t / period) grew).
  - With the 1 ms step and 200 Hz the forwarded stamps are exact multiples of
    5 ms, so a 50 Hz (20 ms) sim-time timer fires exactly on its tick.
  - The average output rate is rate_hz in sim time at any real-time factor
    (fewer messages per wall second when the sim runs slower, as before).
  - The forwarded values are Gazebo's own, never interpolated or invented: a
    paused sim (same value repeated, or nothing) forwards nothing new, so the
    supervisor's "sim clock paused" gate keeps working.
  - Time going backwards (world reset) is forwarded at once and re-anchors.
  - rate_hz <= 0 forwards every message (A/B against the old behaviour).

Messages are handled serialized (CDR bytes in, the same bytes out): no
deserialize/serialize per message, only the 8-byte time is read.

This node itself must run on WALL time (use_sim_time false): on sim time it
would subscribe to the /clock it publishes.
"""
import signal
import struct

# CDR encapsulation header (2 bytes representation id + 2 bytes options),
# then builtin_interfaces/Time {int32 sec, uint32 nanosec}.
_CDR_LE = struct.Struct('<iI')
_CDR_BE = struct.Struct('>iI')


def clock_ns_from_cdr(data: bytes) -> int:
    """Return the time in a serialized rosgraph_msgs/Clock as integer ns."""
    if len(data) < 12:
        raise ValueError(f'Clock CDR too short: {len(data)} bytes')
    # representation id 0x0001 = CDR_LE, 0x0000 = CDR_BE (also PL_CDR_* 0x3/0x2)
    little = data[1] & 0x01
    sec, nsec = (_CDR_LE if little else _CDR_BE).unpack_from(data, 4)
    return sec * 1_000_000_000 + nsec


class ClockDecimator:
    """Decides which clock values to forward. Pure logic, no ROS."""

    def __init__(self, rate_hz: float):
        self.period_ns = int(round(1e9 / rate_hz)) if rate_hz > 0 else 0
        self._last_bin = None
        self._last_ns = None

    def accept(self, t_ns: int) -> bool:
        """Return True if the clock value t_ns should be published."""
        if self.period_ns == 0:                     # passthrough
            self._last_ns = t_ns
            return True
        b = t_ns // self.period_ns
        last_ns = self._last_ns
        self._last_ns = t_ns
        if last_ns is not None and t_ns < last_ns:  # backwards: reset, re-anchor
            self._last_bin = b
            return True
        if self._last_bin is None or b > self._last_bin:
            self._last_bin = b
            return True
        return False


def main():
    import rclpy
    from rclpy.executors import ExternalShutdownException
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                           QoSReliabilityPolicy)
    from rosgraph_msgs.msg import Clock

    class ClockThrottle(Node):

        def __init__(self):
            # Wall time, whatever the launch file says (see module docstring).
            super().__init__(
                'sim_clock_throttle',
                parameter_overrides=[
                    Parameter('use_sim_time', Parameter.Type.BOOL, False)])
            self.declare_parameter('rate_hz', 200.0)
            self.declare_parameter('input_topic', '/sim/clock_raw')
            self.declare_parameter('output_topic', '/clock')
            self.declare_parameter('stats_period_sec', 30.0)
            p = self.get_parameter
            rate = float(p('rate_hz').value)
            self.decimator = ClockDecimator(rate)
            self.n_in = 0
            self.n_out = 0

            # Input: local only (same PC), keep only the newest.
            in_qos = QoSProfile(
                depth=1, history=QoSHistoryPolicy.KEEP_LAST,
                reliability=QoSReliabilityPolicy.BEST_EFFORT,
                durability=QoSDurabilityPolicy.VOLATILE)
            # Output: RELIABLE so reliable readers (probes, ros2 topic echo,
            # bag recorders) still match; the stack's TimeSource readers are
            # best effort, so they cost no ACK traffic. Depth 1: a stale clock
            # value is worthless, never queue a backlog to flush later.
            out_qos = QoSProfile(
                depth=1, history=QoSHistoryPolicy.KEEP_LAST,
                reliability=QoSReliabilityPolicy.RELIABLE,
                durability=QoSDurabilityPolicy.VOLATILE)
            self.pub = self.create_publisher(Clock, p('output_topic').value, out_qos)
            self.create_subscription(
                Clock, p('input_topic').value, self.on_clock, in_qos, raw=True)

            period = float(p('stats_period_sec').value)
            if period > 0:
                self.create_timer(period, self.report)
            self.get_logger().info(
                f"{p('input_topic').value} -> {p('output_topic').value} at "
                + (f'{rate:g} Hz of sim time' if rate > 0 else 'full rate (passthrough)'))

        def on_clock(self, data: bytes):
            self.n_in += 1
            try:
                t_ns = clock_ns_from_cdr(data)
            except ValueError as exc:
                self.get_logger().warn(str(exc), throttle_duration_sec=5.0)
                return
            if self.decimator.accept(t_ns):
                self.pub.publish(data)
                self.n_out += 1

        def report(self):
            last = self.decimator._last_ns
            self.get_logger().info(
                f'clock in {self.n_in} / out {self.n_out} msgs in the last period, '
                f'sim time {last / 1e9:.3f} s' if last is not None
                else 'no clock received yet')
            self.n_in = self.n_out = 0

    rclpy.init()
    node = ClockThrottle()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        # Same teardown as drive_bridge.main(): ignore the second SIGINT that
        # ros2 launch forwards, and don't shut down an already shut context.
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
