#!/usr/bin/env python3
"""hard_stop_node.py: the last-resort LiDAR stop. One sensor, one decision, one lane.

See hard_stop.py's module docstring for WHY this exists and why it is not a
behaviour-tree leaf. This file is the ROS glue and nothing else: the trigger
rule, the self-hit mask, the debounce and the latch are all pure functions
there.

WHAT IT SUBSCRIBES TO, AND AT WHAT RATE
  /scan, qos_profile_sensor_data, NEVER decimated and never throttled. The whole
  value of this node is latency: one scan period, 25 ms at the 40 Hz this rig
  measures. A throttle or a timer-driven "latest scan" poll would add a period of
  its own for no benefit, and this node does so little per scan (one pass over
  1081 floats) that the full rate costs nothing worth measuring.

WHAT IT PUBLISHES
  /safety_stop (ackermann_msgs/AckermannDriveStamped), speed 0, steering 0 --
  the EXISTING mux lane at priority 200 (f1tenth_bringup/config/mux.yaml), the
  same one the behaviour tree's Stop behaviours use.

  Sharing that lane rather than adding a new one is deliberate. A second lane at
  priority 200 would be a tie, and the mux's behaviour between equal priorities
  is not something to bet a safety stop on; a lane at 210 would rank this above
  the operator's own joystick, which is the wrong answer for a reflex with no
  model of intent. And both publishers on this lane mean exactly the same thing
  -- stop -- so they cannot contradict each other, which is what usually makes a
  shared topic a bad idea.

  header.frame_id is 'base_link/hard_stop'. That is load-bearing, not
  decoration: the mux republishes the winning message VERBATIM onto
  /ackermann_drive, so frame_id is the only thing that lets a recorded bag
  attribute an actuated command to its source. The existing conventions are ''
  (MPC_corr), 'base_link/emergency' and 'base_link/obstacle' (the BT's two
  Stops); stop.py's own docstring asks every new Stop source to take a distinct
  one, after a 2026-09-01 analysis could not separate emergency from obstacle
  stops because both stamped a bare 'base_link'.

  /safety/hard_stop (std_msgs/String, JSON) -- state and trigger count every
  scan. A String of JSON rather than a custom message for the same reason
  /costmap/safe_corridor_report is one: it is a diagnostic whose field set is
  still being shaped, nothing in the stack parses it, and a .msg would both
  freeze it and make every bag recorded against an older definition
  undeserialisable (CDR is positional).

THE REPUBLISH TIMER, which is the one piece of real subtlety here. The mux lane
has a 0.2 s timeout: a lane that stops publishing loses its claim and the
next-highest lane takes over. While the stop is ENGAGED, this node therefore has
to keep publishing, and it must not depend on scans to do it -- because "the
LiDAR stopped publishing" is one of the ways the situation this node exists for
actually arises. A scan-driven republish would mean a dead LiDAR silently
RELEASES a latched stop after 0.2 s. So the republish runs on its own timer at
hard_stop_republish_hz (default 20, i.e. 4x the mux's timeout), and the latch's
clear hold is re-evaluated there too (HardStopState.tick) so the release still
happens on time when scans are flowing and never happens for want of one.

DIRECTION-AWARE (hard_stop_allow_reverse, default true, 2026-10-20). With
every triggering return ahead of the bumper, the stop blocks only motion
towards it: it subscribes to the lanes under its own (/teleop,
/calibration_drive, /drive), works out which one the mux would pass, and
stays quiet while that command is negative, so the MPC's reverse manoeuvre
can back the car out. Any non-negative command re-claims the lane at once
(in the lane's own callback, not on the next timer tick); a return beside
the nose keeps the stop total. hard_stop.stop_now has the rule. Quiet is
never a release: the latch and its clear hold run as before, and the other
/safety_stop sources (the BT's e-stop among them) are not affected.

WHAT IT DOES NOT TOUCH: the 0.40 m behaviour-tree proximity e-stop
(IsProximityTooClose) is unchanged and still runs; /costmap/front_clearance is
unchanged; the BT stop logic is unchanged. This is an addition, not a
replacement -- two independent stops at two distances, neither relying on the
other.
"""

import json
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from ackermann_msgs.msg import AckermannDriveStamped
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

from f1tenth_control.hard_stop import (
    all_ahead, HardStopState, LANE_TIMEOUT_S, LOWER_LANES, footprint_polygon, stop_now,
    trigger_range_hits, winning_speed)

#: frame_id this node stamps every stop with -- see the module docstring.
HARD_STOP_FRAME_ID = 'base_link/hard_stop'


class HardStopNode(Node):

    def __init__(self, **kwargs):
        super().__init__('hard_stop_node', **kwargs)

        self.declare_parameter('hard_stop_enabled', True)
        self.declare_parameter('hard_stop_range_m', 0.20)
        self.declare_parameter('hard_stop_debounce_scans', 2)
        self.declare_parameter('hard_stop_clear_s', 0.5)
        # 20 Hz is 4x the mux lane's own 0.2 s timeout, so three consecutive
        # missed republishes still leave the claim alive.
        self.declare_parameter('hard_stop_republish_hz', 20.0)
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('stop_topic', '/safety_stop')
        self.declare_parameter('diag_topic', '/safety/hard_stop')
        # The footprint. Defaults come from the URDF and stack_params via
        # vehicle_frames, NOT from numbers invented here -- declared as
        # parameters anyway so a rig whose shell differs can say so without a
        # code change, and so a test can shrink the mask to prove the self-hit
        # path is what is doing the discarding.
        self.declare_parameter('footprint_front_x_m', _default_front_x())
        self.declare_parameter('footprint_rear_x_m', -0.10)
        self.declare_parameter('footprint_half_width_m', _default_half_width())
        self.declare_parameter('footprint_self_margin_m', 0.02)
        # base_link -> laser is a pure translation on this rig; the node reads
        # it as a parameter rather than through TF so it has no dependency on a
        # transform being available at the instant a stop is needed.
        self.declare_parameter('laser_x_m', 0.305)
        # Direction-aware stop (hard_stop.stop_now): an obstacle ahead of the
        # bumper blocks only forward motion. The lanes it reads are the mux's
        # own (f1tenth_bringup/config/mux.yaml).
        self.declare_parameter('hard_stop_allow_reverse', True)
        self.declare_parameter('lane_timeout_s', LANE_TIMEOUT_S)

        p = self.get_parameter
        self.enabled = bool(p('hard_stop_enabled').value)
        self.allow_reverse = bool(p('hard_stop_allow_reverse').value)
        self.lane_timeout_s = float(p('lane_timeout_s').value)
        self.ahead_only = False
        self.lane_latest = {}
        self.trigger_range_m = float(p('hard_stop_range_m').value)
        self.laser_x = float(p('laser_x_m').value)
        self.polygon = footprint_polygon(
            front_x=float(p('footprint_front_x_m').value),
            rear_x=float(p('footprint_rear_x_m').value),
            half_width=float(p('footprint_half_width_m').value),
            margin=float(p('footprint_self_margin_m').value))
        self.state = HardStopState(
            debounce_scans=int(p('hard_stop_debounce_scans').value),
            clear_s=float(p('hard_stop_clear_s').value))

        self.stop_pub = self.create_publisher(
            AckermannDriveStamped, str(p('stop_topic').value), 10)
        self.diag_pub = self.create_publisher(String, str(p('diag_topic').value), 10)
        self.scan_sub = self.create_subscription(
            LaserScan, str(p('scan_topic').value), self._scan_cb, qos_profile_sensor_data)

        if self.allow_reverse:
            for lane, _prio in LOWER_LANES:
                self.create_subscription(
                    AckermannDriveStamped, '/' + lane,
                    lambda msg, lane=lane: self._lane_cb(lane, msg), 10)

        republish_hz = float(p('hard_stop_republish_hz').value)
        self.republish_timer = self.create_timer(1.0 / republish_hz, self._republish)

        self.get_logger().info(
            f'HARD STOP {"enabled" if self.enabled else "DISABLED"}: '
            f'range <= {self.trigger_range_m:.3f} m from the laser at x={self.laser_x:.3f} '
            f'(bumper {float(p("footprint_front_x_m").value):.3f}), '
            f'debounce {self.state.debounce_scans} scans, clear hold {self.state.clear_s:.2f} s, '
            f'self-mask x[{self.polygon[0]:+.3f},{self.polygon[1]:+.3f}] '
            f'y[{self.polygon[2]:+.3f},{self.polygon[3]:+.3f}], '
            f'stop -> {p("stop_topic").value} as {HARD_STOP_FRAME_ID}, '
            f'reverse {"allowed" if self.allow_reverse else "blocked"} past an obstacle ahead')

    # ------------------------------------------------------------------
    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _scan_cb(self, msg: LaserScan):
        """One scan: find triggering returns, fold them in, act."""
        if not self.enabled:
            return
        hits = trigger_range_hits(
            msg.ranges, msg.angle_min, msg.angle_increment,
            msg.range_min, msg.range_max, self.laser_x,
            self.polygon, self.trigger_range_m)
        was_engaged = self.state.engaged
        engaged = self.state.update(bool(hits), self._now())
        if hits:
            self.ahead_only = all_ahead(hits, self.polygon)

        if self._claim(engaged):
            self._publish_stop()
        if hits:
            # Throttled, but the FIRST trigger of each event is not: a stop
            # nobody can attribute afterwards is a stop that gets argued about.
            nearest = hits[0]
            message = (f'HARD STOP {"ENGAGED" if engaged else "pending"} '
                       f'({self.state.consecutive_hits}/{self.state.debounce_scans}): '
                       f'{len(hits)} return(s) within {self.trigger_range_m:.2f} m, '
                       f'nearest {nearest[0]:.3f} m at {math.degrees(nearest[1]):+.1f} deg '
                       f'(base_link {nearest[2]:+.3f},{nearest[3]:+.3f})')
            if engaged and not was_engaged:
                self.get_logger().warn(f'{message} [trigger #{self.state.trigger_count}]')
            else:
                self.get_logger().warn(message, throttle_duration_sec=1.0)
        elif was_engaged and not engaged:
            self.get_logger().info(
                f'hard stop RELEASED after {self.state.clear_s:.2f} s clear '
                f'(trigger #{self.state.trigger_count})')

        diag = self.state.as_dict(n_hits=len(hits), nearest=(hits[0][:2] if hits else None))
        diag.update(allow_reverse=self.allow_reverse, ahead_only=self.ahead_only,
                    passing_reverse=bool(engaged and not self._claim(engaged)))
        self.diag_pub.publish(String(data=json.dumps(diag, sort_keys=True)))

    def _republish(self):
        """Keep the mux lane claimed while engaged, and age the clear hold.

        See the module docstring: the lane times out in 0.2 s, and a
        scan-driven republish would let a dead LiDAR release a latched stop.
        """
        if not self.enabled:
            return
        was_engaged = self.state.engaged
        engaged = self.state.tick(self._now())
        if self._claim(engaged):
            self._publish_stop()
        elif was_engaged and not engaged:
            self.get_logger().info(
                f'hard stop RELEASED on the clear hold with no further scans '
                f'(trigger #{self.state.trigger_count})')

    def _speed(self):
        return winning_speed(self.lane_latest, self._now(), timeout_s=self.lane_timeout_s)

    def _claim(self, engaged):
        return stop_now(engaged, self.allow_reverse, self.ahead_only, self._speed())

    def _lane_cb(self, lane, msg):
        """A command on a lane under ours: re-claim at once if it is not a reverse."""
        self.lane_latest[lane] = (self._now(), float(msg.drive.speed))
        if self.enabled and self._claim(self.state.engaged):
            self._publish_stop()

    def _publish_stop(self):
        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = HARD_STOP_FRAME_ID
        msg.drive.speed = 0.0
        msg.drive.steering_angle = 0.0
        self.stop_pub.publish(msg)


def _default_front_x():
    """The URDF's bumper x, or the known value if the share dir is unavailable."""
    try:
        from f1tenth_description.vehicle_frames import base_front_x
        return float(base_front_x())
    except Exception:  # pragma: no cover - only when the share dir is missing
        return 0.405


def _default_half_width():
    """The stack's own body half-width, or the known value."""
    try:
        from f1tenth_params.param_defaults import get_value
        return float(get_value('swept_clearance_body_half_width_m'))
    except Exception:  # pragma: no cover - only when stack_params is unavailable
        return 0.136


def main(args=None):
    rclpy.init(args=args)
    node = HardStopNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
