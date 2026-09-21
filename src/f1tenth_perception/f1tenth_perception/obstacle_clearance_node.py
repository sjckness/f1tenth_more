#!/usr/bin/env python3
"""obstacle_clearance_node: footprint-to-nearest-return distance, once per /scan.

PUBLISHES
  <clearance_topic>   Float32  signed distance [m] from the car's rectangular
                               footprint to the nearest valid /scan return, one
                               message per scan (default /obstacle_clearance).
                               Negative: a return inside the body outline.
                               +inf: no valid return at all.
  <safety_event_topic> String  JSON {event: "contact", cause, clearance_m,
                               source}, once when the clearance first drops to
                               contact_threshold_m, re-armed only after it
                               rises contact_rearm_m above it (default
                               /safety/event). publish_contact_events false
                               turns it off.

A DATA SOURCE, NOT A CONTROLLER. Nothing in the stack subscribes to either
topic to drive the car; the consumer is f1tenth_logger's test-campaign
logger, started by hand. The geometry is obstacle_clearance.py's, the scan
filter swept_clearance.scan_to_points' (inf, nan, below range_min and above
range_max dropped -- which is what removes urg_node's finite 65.533 m
no-return code, since the scan advertises range_max 30 m).

WHAT "CONTACT" MEANS HERE, AND WHAT IT MISSES. The only sensor is the 2D
lidar, 0.20 m above base_link: a return inside the footprint is something at
that height inside the body outline -- a wall or a person the car has
reached. Anything lower than the scan plane (a bottle, a low box) is
invisible to it, and so is its contact. There is no bumper, no collision
check and no IMU impact detector in this stack; this is the only contact
signal there is.

SELF-RETURNS. If a part of the car itself (a mast, a cable) sits in the scan
plane inside the footprint, the clearance reads negative at a standstill in
open space and one contact event fires at startup. The calibration run shows
it; shrink the footprint or fix the mount before trusting contact events.

The base_link <- scan frame transform comes from /tf_static into a tf2 Buffer
this node fills itself, as swept_clearance_node does: no hardcoded offset, so
the car's real forward mount (+0.12 m, yaw 0) is used, not the URDF's stale
rear-facing one. Scans arriving before the transform are skipped.
"""

import json
import math

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32, String
from tf2_msgs.msg import TFMessage
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer

from f1tenth_perception.obstacle_clearance import footprint_clearance
from f1tenth_perception.swept_clearance import quaternion_to_rotation, scan_to_points
from f1tenth_perception.swept_corridor import (
    BODY_FRONT_X_M, BODY_HALF_WIDTH_M, BODY_REAR_X_M)

_TF_STATIC_QOS = QoSProfile(
    depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL, history=HistoryPolicy.KEEP_LAST)

#: swept_corridor's body rectangle, as length/width/tail position
DEFAULT_LENGTH_M = BODY_FRONT_X_M - BODY_REAR_X_M
DEFAULT_WIDTH_M = 2.0 * BODY_HALF_WIDTH_M
DEFAULT_REAR_X_M = BODY_REAR_X_M


class ContactEdge:
    """Rising edge of clearance <= threshold, re-armed above threshold + rearm."""

    def __init__(self, threshold, rearm):
        if rearm < 0.0:
            raise ValueError(f'contact_rearm_m must be >= 0, got {rearm}')
        self.threshold = float(threshold)
        self.rearm = float(rearm)
        self.in_contact = False

    def update(self, clearance):
        """True exactly once per contact episode."""
        if not math.isfinite(clearance):
            if clearance > 0.0:
                self.in_contact = False
            return False
        if self.in_contact:
            if clearance > self.threshold + self.rearm:
                self.in_contact = False
            return False
        if clearance <= self.threshold:
            self.in_contact = True
            return True
        return False


class ObstacleClearanceNode(Node):

    def __init__(self, **kwargs):
        super().__init__('obstacle_clearance_node', **kwargs)

        def param(name, default):
            return self.declare_parameter(name, default).value

        self.scan_topic = str(param('scan_topic', '/scan'))
        self.clearance_topic = str(param('clearance_topic', '/obstacle_clearance'))
        self.safety_event_topic = str(param('safety_event_topic', '/safety/event'))
        self.base_frame = str(param('base_frame', 'base_link'))
        self.length = float(param('footprint_length_m', DEFAULT_LENGTH_M))
        self.width = float(param('footprint_width_m', DEFAULT_WIDTH_M))
        self.rear_x = float(param('footprint_rear_x_m', DEFAULT_REAR_X_M))
        if self.length <= 0.0 or self.width <= 0.0:
            raise ValueError(
                f'footprint_length_m/footprint_width_m must be positive, got '
                f'{self.length} x {self.width}')
        self.publish_contact_events = bool(param('publish_contact_events', True))
        self.contact = ContactEdge(float(param('contact_threshold_m', 0.0)),
                                   float(param('contact_rearm_m', 0.05)))

        self._tf_buffer = Buffer()
        self._transform = None   # (frame_id, rotation 3x3, translation 3)

        self.clearance_pub = self.create_publisher(Float32, self.clearance_topic, 10)
        self.safety_pub = self.create_publisher(String, self.safety_event_topic, 10)
        self.tf_static_sub = self.create_subscription(
            TFMessage, '/tf_static', self._tf_static_cb, _TF_STATIC_QOS)
        # best-effort, like every other /scan consumer here: this node must
        # not be able to back-pressure urg_node, the e-stop's only /scan source
        self.scan_sub = self.create_subscription(
            LaserScan, self.scan_topic, self._scan_cb, qos_profile_sensor_data)

        self.get_logger().info(
            f'obstacle_clearance_node up: {self.scan_topic} -> {self.clearance_topic}; '
            f'footprint {self.length:.3f} x {self.width:.3f} m, x in '
            f'[{self.rear_x:+.3f}, {self.rear_x + self.length:+.3f}] of {self.base_frame}; '
            f'contact events on {self.safety_event_topic} '
            f'{"at <= %.3f m" % self.contact.threshold if self.publish_contact_events else "OFF"}')

    def _tf_static_cb(self, msg):
        for transform in msg.transforms:
            self._tf_buffer.set_transform_static(transform, 'obstacle_clearance_node')

    def _transform_for(self, frame_id):
        if self._transform is not None and self._transform[0] == frame_id:
            return self._transform[1:]
        try:
            stamped = self._tf_buffer.lookup_transform(self.base_frame, frame_id, Time())
        except TransformException:
            self.get_logger().warn(
                f'no {self.base_frame} <- {frame_id!r} transform on /tf_static yet; '
                'skipping scans until it arrives', throttle_duration_sec=5.0)
            return None
        t = stamped.transform
        rotation = quaternion_to_rotation(t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w)
        translation = np.array([t.translation.x, t.translation.y, t.translation.z])
        self._transform = (frame_id, rotation, translation)
        self.get_logger().info(
            f'{self.base_frame} <- {frame_id}: t=({translation[0]:.3f}, {translation[1]:.3f}, '
            f'{translation[2]:.3f}) m')
        return rotation, translation

    def _scan_cb(self, msg):
        transform = self._transform_for(msg.header.frame_id)
        if transform is None:
            return
        points = scan_to_points(msg.ranges, msg.angle_min, msg.angle_increment,
                                msg.range_min, msg.range_max, *transform)
        clearance = footprint_clearance(points, self.length, self.width, self.rear_x)
        self.clearance_pub.publish(Float32(data=clearance))
        if self.contact.update(clearance) and self.publish_contact_events:
            self._publish_contact(clearance)

    def _publish_contact(self, clearance):
        cause = (f'lidar return {-clearance:.3f} m inside the footprint' if clearance < 0.0
                 else f'lidar return {clearance:.3f} m from the footprint')
        self.get_logger().warn(f'CONTACT: {cause}')
        self.safety_pub.publish(String(data=json.dumps({
            'event': 'contact',
            'cause': cause,
            'clearance_m': round(float(clearance), 4),
            'source': 'obstacle_clearance_node',
        })))


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleClearanceNode()
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
