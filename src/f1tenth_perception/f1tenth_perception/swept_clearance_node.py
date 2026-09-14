#!/usr/bin/env python3
"""swept_clearance_node: forward clearance along the steering arc, LiDAR + ZED depth.

PUBLISHES
  <clearance_topic>            Float32  fused clearance [m], at publish_rate_hz
                                        (default /perception/swept_clearance)
  <clearance_topic>/lidar      Float32  LiDAR-only clearance, once per scan
  <clearance_topic>/camera     Float32  depth-only clearance, once per depth frame
  <clearance_topic>/steering   Float32  steering angle [rad] the corridor used
                                        (the lagged one in envelope mode), at
                                        publish_rate_hz; nan before any command

The value is the rear-axle arc length before the body touches something, NOT a
sensor range: straight ahead it is the gap to the front bumper. Geometry,
footprint and every exclusion are in swept_corridor.py's module docstring; the
steering estimate and fusion rules in swept_clearance.py's.

NOTHING SUBSCRIBES TO THIS YET. It is a new topic next to, not a replacement
for, /perception/front_clearance (camera ROI), /costmap/front_clearance (map)
and IsProximityTooClose's own /scan cone, whose consumers are unchanged.

FUSION
------
min(lidar, camera) over the sensors whose last value is younger than their
timeout. A stale sensor is left out with a throttled warning, never read as
clear; with both stale the topic carries 0.0. A sensor turned off by use_lidar
or use_camera is not a sensor here at all: it is neither fused nor warned
about, and with both off the topic carries 0.0. The fused value is published
from a timer, not from a sensor callback, precisely so that silence from both
sensors still produces a 0.0 on the wire.

SENSOR PATHS -- points in base_link, then swept_corridor.clearance(), nothing else
  * LiDAR: every return of the full scan (inf/nan/below range_min dropped),
    through base_link <- scan frame.
  * Camera: ZED depth_registered unprojected with its camera_info, through
    base_link <- optical frame (the full static chain, including the ZED's own
    ~3 deg mount pitch), kept only inside [camera_z_min_m, camera_z_max_m].
    This is the DEPTH case: the ZED2 is a stereo camera. The webcam source is
    monocular and has no depth topic; with camera_source 'webcam' the launch
    file sets use_camera false rather than guess a ground plane.

Both transforms come from /tf_static into a tf2 Buffer this node fills itself
(lidar_front_wall_node's approach: no /tf subscription, no hardcoded offset).
Until a sensor's transform arrives its frames are skipped and it stays stale.

rear_axle_x_m is where the rear axle sits in base_frame; 0 per the URDF. Points
are shifted by it before the corridor test, whose frame is the rear axle.
"""

import math

import numpy as np
import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image, LaserScan
from std_msgs.msg import Float32
from tf2_msgs.msg import TFMessage
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer

from f1tenth_perception.cpu_affinity import apply_nice, declare_nice_param
from f1tenth_perception.swept_clearance import (
    STEERING_MODES,
    SensorReading,
    SteeringHistory,
    depth_to_points,
    fuse,
    quaternion_to_rotation,
    scan_to_points,
)
from f1tenth_perception.swept_corridor import clearance

_TF_STATIC_QOS = QoSProfile(
    depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL, history=HistoryPolicy.KEEP_LAST)


def depth_image_to_metres(msg):
    """sensor_msgs/Image -> float64 (height, width) depth in metres, or None for
    an unsupported encoding. 32FC1 is metres with ZED's -inf/nan/+inf kept as
    they are; 16UC1 is millimetres, where 0 means no data and becomes nan.
    """
    if msg.encoding == '32FC1':
        dtype, bytes_per = np.dtype('>f4' if msg.is_bigendian else '<f4'), 4
    elif msg.encoding == '16UC1':
        dtype, bytes_per = np.dtype('>u2' if msg.is_bigendian else '<u2'), 2
    else:
        return None
    row_len = msg.step // bytes_per
    data = np.frombuffer(bytes(msg.data), dtype=dtype, count=msg.height * row_len)
    image = data.reshape(msg.height, row_len)[:, :msg.width].astype(float)
    if msg.encoding == '16UC1':
        image = np.where(image == 0.0, np.nan, image / 1000.0)
    return image


class SweptClearanceNode(Node):

    def __init__(self, **kwargs):
        super().__init__('swept_clearance_node', **kwargs)

        def param(name, default):
            return self.declare_parameter(name, default).value

        self.scan_topic = str(param('scan_topic', '/scan'))
        self.depth_topic = str(param('depth_topic', '/zed2/zed_node/depth/depth_registered'))
        self.depth_info_topic = str(param('depth_info_topic', '/zed2/zed_node/depth/camera_info'))
        self.steering_topic = str(param('steering_topic', '/ackermann_drive'))
        self.clearance_topic = str(param('clearance_topic', '/perception/swept_clearance'))
        self.base_frame = str(param('base_frame', 'base_link'))
        self.use_lidar = bool(param('use_lidar', True))
        self.use_camera = bool(param('use_camera', True))

        self.geometry = {
            'wheelbase': float(param('wheelbase_m', 0.305)),
            'front_x': float(param('body_front_x_m', 0.443)),
            'rear_x': float(param('body_rear_x_m', -0.082)),
            'half_width': float(param('body_half_width_m', 0.136)),
            'margin': float(param('margin_m', 0.10)),
            'max_range': float(param('max_range_m', 5.0)),
            'absolute_min_clearance': float(param('absolute_min_clearance_m', 0.15)),
        }
        self.rear_axle_x = float(param('rear_axle_x_m', 0.0))

        self.steering_mode = str(param('steering_estimate', 'lagged_command'))
        if self.steering_mode not in STEERING_MODES:
            raise ValueError(
                f'steering_estimate {self.steering_mode!r} is not one of {STEERING_MODES}')
        self.steering = SteeringHistory(float(param('steering_lag_sec', 0.3)))

        self.lidar_timeout = float(param('lidar_timeout_sec', 0.25))
        self.camera_timeout = float(param('camera_timeout_sec', 0.5))
        publish_rate_hz = float(param('publish_rate_hz', 20.0))

        self.camera_stride = int(param('camera_stride_px', 4))
        self.camera_max_points = int(param('camera_max_points', 4000))
        self.camera_min_depth = float(param('camera_min_depth_m', 0.2))
        self.camera_max_depth = float(param('camera_max_depth_m', 6.0))
        self.camera_z_min = float(param('camera_z_min_m', 0.08))
        self.camera_z_max = float(param('camera_z_max_m', 0.35))

        declare_nice_param(self)
        apply_nice(self)

        self._tf_buffer = Buffer()
        self._transforms = {}   # frame_id -> (rotation 3x3, translation 3)
        self._depth_info = None
        self._lidar = (None, None)    # (receipt time, clearance)
        self._camera = (None, None)

        self.clearance_pub = self.create_publisher(Float32, self.clearance_topic, 10)
        self.lidar_pub = self.create_publisher(Float32, self.clearance_topic + '/lidar', 10)
        self.camera_pub = self.create_publisher(Float32, self.clearance_topic + '/camera', 10)
        self.steering_pub = self.create_publisher(Float32, self.clearance_topic + '/steering', 10)

        self.tf_static_sub = self.create_subscription(
            TFMessage, '/tf_static', self._tf_static_cb, _TF_STATIC_QOS)
        self.steering_sub = self.create_subscription(
            AckermannDriveStamped, self.steering_topic, self._steering_cb, 10)
        # Sensor subscriptions are best-effort, like lidar_front_wall_node's:
        # this node must not be able to back-pressure urg_node, the e-stop's
        # only /scan publisher.
        self.scan_sub = None
        if self.use_lidar:
            self.scan_sub = self.create_subscription(
                LaserScan, self.scan_topic, self._scan_cb, qos_profile_sensor_data)
        self.depth_info_sub = None
        self.depth_sub = None
        if self.use_camera:
            self.depth_info_sub = self.create_subscription(
                CameraInfo, self.depth_info_topic, self._depth_info_cb, qos_profile_sensor_data)
            self.depth_sub = self.create_subscription(
                Image, self.depth_topic, self._depth_cb, qos_profile_sensor_data)

        self.timer = self.create_timer(1.0 / publish_rate_hz, self._publish_fused)

        sensors = [name for name, on in (('lidar ' + self.scan_topic, self.use_lidar),
                                         ('depth ' + self.depth_topic, self.use_camera)) if on]
        self.get_logger().info(
            f'swept_clearance_node up: {", ".join(sensors) or "NO SENSORS (publishing 0.0)"}; '
            f'steering {self.steering_topic} as {self.steering_mode} '
            f'(lag {self.steering.lag_sec:.2f} s) -> {self.clearance_topic}')

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    # ---- inputs that only cache -------------------------------------------

    def _tf_static_cb(self, msg):
        for transform in msg.transforms:
            self._tf_buffer.set_transform_static(transform, 'swept_clearance_node')

    def _steering_cb(self, msg):
        if not self.steering.add(self._now(), float(msg.drive.steering_angle)):
            self.get_logger().warn(
                f'ignoring non-finite steering_angle on {self.steering_topic}',
                throttle_duration_sec=5.0)

    def _depth_info_cb(self, msg):
        self._depth_info = msg

    def _transform_for(self, frame_id):
        cached = self._transforms.get(frame_id)
        if cached is not None:
            return cached
        try:
            stamped = self._tf_buffer.lookup_transform(self.base_frame, frame_id, Time())
        except TransformException:
            self.get_logger().warn(
                f'no {self.base_frame} <- {frame_id!r} transform on /tf_static yet; '
                'skipping this sensor until it arrives', throttle_duration_sec=5.0)
            return None
        t = stamped.transform
        rotation = quaternion_to_rotation(t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w)
        translation = np.array([t.translation.x, t.translation.y, t.translation.z])
        self._transforms[frame_id] = (rotation, translation)
        self.get_logger().info(
            f'{self.base_frame} <- {frame_id}: t=({translation[0]:.3f}, {translation[1]:.3f}, '
            f'{translation[2]:.3f}) m')
        return rotation, translation

    # ---- the one clearance computation both sensors share ------------------

    def _clearance(self, points_base, now):
        points = points_base - np.array([self.rear_axle_x, 0.0])
        angles = self.steering.angles(now, self.steering_mode)
        if not angles:
            self.get_logger().warn(
                f'no steering command on {self.steering_topic} yet; using a straight corridor',
                throttle_duration_sec=5.0)
            angles = [0.0]
        return min(clearance(points, angle, **self.geometry) for angle in angles)

    def _scan_cb(self, msg):
        now = self._now()
        transform = self._transform_for(msg.header.frame_id)
        if transform is None:
            return
        points = scan_to_points(msg.ranges, msg.angle_min, msg.angle_increment,
                                msg.range_min, msg.range_max, *transform)
        value = self._clearance(points, now)
        self._lidar = (now, value)
        self.lidar_pub.publish(Float32(data=value))

    def _depth_cb(self, msg):
        now = self._now()
        info = self._depth_info
        if info is None:
            self.get_logger().warn(
                f'no camera_info on {self.depth_info_topic} yet; skipping depth',
                throttle_duration_sec=5.0)
            return
        transform = self._transform_for(msg.header.frame_id or info.header.frame_id)
        if transform is None:
            return
        depth = depth_image_to_metres(msg)
        if depth is None:
            self.get_logger().warn(
                f'unsupported depth encoding {msg.encoding!r} (need 32FC1 or 16UC1)',
                throttle_duration_sec=5.0)
            return
        fx, fy, cx, cy = info.k[0], info.k[4], info.k[2], info.k[5]
        if info.width and info.height and (info.width, info.height) != (msg.width, msg.height):
            sx, sy = msg.width / info.width, msg.height / info.height
            fx, cx, fy, cy = fx * sx, cx * sx, fy * sy, cy * sy
        points = depth_to_points(
            depth, fx, fy, cx, cy, *transform, stride=self.camera_stride,
            min_depth=self.camera_min_depth, max_depth=self.camera_max_depth,
            z_min=self.camera_z_min, z_max=self.camera_z_max,
            max_points=self.camera_max_points)
        value = self._clearance(points, now)
        self._camera = (now, value)
        self.camera_pub.publish(Float32(data=value))

    # ---- fusion -----------------------------------------------------------

    def _publish_fused(self):
        now = self._now()
        readings = []
        if self.use_lidar:
            readings.append(SensorReading('lidar', *self._lidar, self.lidar_timeout))
        if self.use_camera:
            readings.append(SensorReading('camera', *self._camera, self.camera_timeout))
        value, stale = fuse(now, readings)
        self.clearance_pub.publish(Float32(data=float(value)))

        lagged = self.steering.lagged(now)
        self.steering_pub.publish(Float32(data=math.nan if lagged is None else float(lagged)))

        if stale:
            ages = ', '.join(
                r.name + (' never received' if r.stamp is None else f' {now - r.stamp:.2f} s old')
                for r in readings if r.name in stale)
            if len(stale) == len(readings):
                self.get_logger().warn(
                    f'every clearance sensor is stale ({ages}); publishing 0.0',
                    throttle_duration_sec=2.0)
            else:
                self.get_logger().warn(
                    f'ignoring stale clearance sensor ({ages})', throttle_duration_sec=2.0)
        elif not readings:
            self.get_logger().warn(
                'use_lidar and use_camera are both false; publishing 0.0',
                throttle_duration_sec=10.0)


def main(args=None):
    rclpy.init(args=args)
    node = SweptClearanceNode()
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
