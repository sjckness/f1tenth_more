#!/usr/bin/env python3
"""2D-to-3D detection fusion node for the F1TENTH perception stack.

Synchronizes YOLO 2D detections (yolo_detector_node, vision_msgs/Detection2DArray,
pixel coords) with the ZED depth image (sensor_msgs/Image, 32FC1 meters) and its
camera_info, back-projects each 2D box into a 3D pose + size using the pinhole
model, and publishes:
  * vision_msgs/Detection3DArray on `detections_3d_topic` (default /camera/detections_3d)
  * visualization_msgs/MarkerArray on `markers_topic` (default /camera/detection_markers)
    -- CUBE markers, color-coded per class, short lifetime so stale boxes vanish
       in Foxglove/RViz if detections stop.

Back-projection math happens in the depth image's own frame (ZED left camera
*optical* frame, e.g. `zed2_left_camera_optical_frame`), since that's the
convention camera_info/pinhole intrinsics use. Before publishing, each pose is
transformed via tf2 into `output_frame` (default `zed2_left_camera_frame`, the
ZED's non-optical camera frame) so downstream consumers (Foxglove 3D panel,
TF-tree-aligned tooling) get a REP-103-style axis convention (x-forward,
y-left, z-up) instead of the optical one (z-forward, x-right, y-down). The
optical -> camera_frame transform is a fixed joint already broadcast by the
ZED wrapper's own robot_state_publisher -- this node only looks it up, it does
not (re)publish it.

Detections with hypothesis.score below `confidence_threshold` are dropped
before either output is built.

Depth-axis (Z) extent of each box is NOT observable from a single monocular
depth view, so `default_depth_extent` is a configurable placeholder, not a
measurement -- see the comment at its use site below.
"""

import hashlib

import message_filters
import numpy as np
import rclpy
import tf2_geometry_msgs  # noqa: F401 - registers Pose transform support
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Pose
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Header
from tf2_ros import (
    ConnectivityException,
    ExtrapolationException,
    LookupException,
)
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from vision_msgs.msg import (
    BoundingBox3D,
    Detection2DArray,
    Detection3D,
    Detection3DArray,
    ObjectHypothesisWithPose,
)
from visualization_msgs.msg import Marker, MarkerArray


class Detection3DNode(Node):
    def __init__(self):
        super().__init__('detection_3d_node')

        # ---- parameters ------------------------------------------------
        self.detections_topic = str(
            self.declare_parameter('detections_topic', '/camera/detections').value)
        self.depth_topic = str(
            self.declare_parameter(
                'depth_topic', '/zed2/zed_node/depth/depth_registered').value)
        self.depth_info_topic = str(
            self.declare_parameter(
                'depth_info_topic', '/zed2/zed_node/depth/camera_info').value)
        self.detections_3d_topic = str(
            self.declare_parameter('detections_3d_topic', '/camera/detections_3d').value)
        self.markers_topic = str(
            self.declare_parameter('markers_topic', '/camera/detection_markers').value)
        self.sync_slop = float(
            self.declare_parameter('sync_slop', 0.1).value)
        # message_filters keeps this many of the *faster* topic's (depth, ~25Hz)
        # messages around while waiting for a match on the slower one
        # (detections -- header stamp is copied from the input frame, but the
        # message itself isn't published until inference finishes, ~1s later
        # on this hardware). The default queue_size=10 in message_filters only
        # covers ~0.4s of depth history at 25Hz, which is shorter than that
        # inference latency, so every detection's matching depth frame had
        # already been evicted before the detection arrived -- nothing ever
        # matched. 60 covers ~2.4s, comfortably above observed latency.
        self.sync_queue_size = int(
            self.declare_parameter('sync_queue_size', 60).value)
        self.default_depth_extent = float(
            self.declare_parameter('default_depth_extent', 0.3).value)
        self.marker_lifetime = float(
            self.declare_parameter('marker_lifetime', 0.2).value)
        self.center_fraction = float(
            self.declare_parameter('center_fraction', 0.5).value)
        # Non-optical ZED camera frame (REP-103 axes) that published poses are
        # transformed into. Confirmed against zed_macro.urdf.xacro: with
        # camera_name='zed2' the wrapper's robot_state_publisher broadcasts a
        # fixed zed2_left_camera_frame -> zed2_left_camera_optical_frame joint,
        # so this is the parent of the optical frame the depth image is in.
        self.output_frame = str(
            self.declare_parameter('output_frame', 'zed2_left_camera_frame').value)
        # Detections with hypothesis.score below this are dropped before
        # either output (Detection3DArray, MarkerArray) is built.
        self.confidence_threshold = float(
            self.declare_parameter('confidence_threshold', 0.3).value)

        # ---- intrinsics (cached from the latest camera_info) -----------
        self.fx = None
        self.fy = None
        self.cx = None
        self.cy = None

        self.bridge = CvBridge()

        # ---- tf2 (optical frame -> output_frame) ------------------------
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # ---- ROS interfaces ---------------------------------------------
        self.det3d_pub = self.create_publisher(
            Detection3DArray, self.detections_3d_topic, 10)
        self.marker_pub = self.create_publisher(
            MarkerArray, self.markers_topic, 10)

        self.info_sub = self.create_subscription(
            CameraInfo, self.depth_info_topic, self._camera_info_callback, 10)

        self.det_sub = message_filters.Subscriber(
            self, Detection2DArray, self.detections_topic)
        self.depth_sub = message_filters.Subscriber(self, Image, self.depth_topic)
        self.sync = message_filters.ApproximateTimeSynchronizer(
            [self.det_sub, self.depth_sub], queue_size=self.sync_queue_size,
            slop=self.sync_slop)
        self.sync.registerCallback(self._synced_callback)

        self.get_logger().info(
            f'detection_3d_node up: syncing "{self.detections_topic}" + '
            f'"{self.depth_topic}" (slop={self.sync_slop}s), intrinsics from '
            f'"{self.depth_info_topic}", publishing 3D detections on '
            f'"{self.detections_3d_topic}" and markers on "{self.markers_topic}" '
            f'in frame "{self.output_frame}", confidence_threshold='
            f'{self.confidence_threshold}')

    def _camera_info_callback(self, msg: CameraInfo):
        # Intrinsics are effectively static per session; cache and move on.
        self.fx = msg.k[0]
        self.fy = msg.k[4]
        self.cx = msg.k[2]
        self.cy = msg.k[5]

    def _synced_callback(self, det_msg, depth_msg):
        if self.fx is None:
            self.get_logger().warn(
                'No camera_info received yet on '
                f'"{self.depth_info_topic}" -- skipping frame', throttle_duration_sec=5.0)
            return

        try:
            depth = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='32FC1')
        except CvBridgeError as exc:
            self.get_logger().error(f'cv_bridge depth conversion failed: {exc}')
            return

        h, w = depth.shape[:2]

        # This is a static fixed joint (see zed_macro.urdf.xacro), so look up
        # the latest available transform (Time()) rather than the depth
        # frame's exact stamp -- avoids extrapolation failures if /tf_static
        # hasn't been received for that exact timestamp yet.
        try:
            optical_to_output = self.tf_buffer.lookup_transform(
                self.output_frame, depth_msg.header.frame_id,
                rclpy.time.Time())
        except (LookupException, ConnectivityException, ExtrapolationException) as exc:
            self.get_logger().warn(
                f'tf2 lookup "{depth_msg.header.frame_id}" -> '
                f'"{self.output_frame}" failed: {exc} -- skipping frame',
                throttle_duration_sec=5.0)
            return

        out_header = Header(stamp=depth_msg.header.stamp, frame_id=self.output_frame)

        det3d_array = Detection3DArray()
        det3d_array.header = out_header

        markers = MarkerArray()

        for i, det in enumerate(det_msg.detections):
            class_id = det.results[0].hypothesis.class_id if det.results else ''
            score = det.results[0].hypothesis.score if det.results else 0.0
            if score < self.confidence_threshold:
                continue

            cx_px = det.bbox.center.position.x
            cy_px = det.bbox.center.position.y
            box_w = det.bbox.size_x
            box_h = det.bbox.size_y

            z = self._median_depth(depth, cx_px, cy_px, box_w, box_h, w, h)
            if z is None:
                continue

            x3d = (cx_px - self.cx) * z / self.fx
            y3d = (cy_px - self.cy) * z / self.fy
            width_3d = box_w * z / self.fx
            height_3d = box_h * z / self.fy

            # Position + orientation, transformed from the optical frame into
            # output_frame. Orientation starts identity (axis-aligned in the
            # optical frame); after the transform it carries the optical ->
            # camera_frame rotation, so scale.x/y/z (still the box's optical
            # width/height/depth-extent) render correctly oriented in
            # output_frame without needing to permute which axis is which.
            optical_pose = Pose()
            optical_pose.position.x = float(x3d)
            optical_pose.position.y = float(y3d)
            optical_pose.position.z = float(z)
            optical_pose.orientation.w = 1.0
            out_pose = tf2_geometry_msgs.do_transform_pose(
                optical_pose, optical_to_output)

            det3d = self._build_detection3d(
                header=out_header, pose=out_pose,
                width=width_3d, height=height_3d,
                class_id=class_id, score=score)
            det3d_array.detections.append(det3d)

            markers.markers.append(self._build_marker(
                header=out_header, marker_id=i, pose=out_pose,
                width=width_3d, height=height_3d, class_id=class_id))

        self.det3d_pub.publish(det3d_array)
        self.marker_pub.publish(markers)

    def _median_depth(self, depth, cx_px, cy_px, box_w, box_h, img_w, img_h):
        # Sample only the center `center_fraction` of the box (in each axis) to
        # avoid depth bleed from background pixels near the box edges.
        half_w = (box_w * self.center_fraction) / 2.0
        half_h = (box_h * self.center_fraction) / 2.0

        x1 = int(np.clip(cx_px - half_w, 0, img_w - 1))
        x2 = int(np.clip(cx_px + half_w, 0, img_w - 1))
        y1 = int(np.clip(cy_px - half_h, 0, img_h - 1))
        y2 = int(np.clip(cy_px + half_h, 0, img_h - 1))

        if x2 <= x1 or y2 <= y1:
            return None

        roi = depth[y1:y2, x1:x2]
        valid = roi[np.isfinite(roi) & (roi > 0.0)]
        if valid.size == 0:
            return None

        return float(np.median(valid))

    def _build_detection3d(self, header, pose, width, height, class_id, score):
        det3d = Detection3D()
        det3d.header = header

        hyp = ObjectHypothesisWithPose()
        hyp.hypothesis.class_id = str(class_id)
        hyp.hypothesis.score = float(score)
        hyp.pose.pose = pose
        det3d.results.append(hyp)

        bbox = BoundingBox3D()
        bbox.center = pose
        bbox.size.x = float(width)
        bbox.size.y = float(height)
        # Depth-axis (camera Z) extent is not observable from a single view;
        # `default_depth_extent` is a fixed placeholder, not a measurement.
        bbox.size.z = self.default_depth_extent
        det3d.bbox = bbox

        return det3d

    def _build_marker(self, header, marker_id, pose, width, height, class_id):
        marker = Marker()
        marker.header = header
        marker.ns = 'detection_3d'
        marker.id = marker_id
        marker.type = Marker.CUBE
        marker.action = Marker.ADD

        marker.pose = pose

        marker.scale.x = max(float(width), 0.01)
        marker.scale.y = max(float(height), 0.01)
        marker.scale.z = self.default_depth_extent

        r, g, b = self._class_color(class_id)
        marker.color.r = r
        marker.color.g = g
        marker.color.b = b
        marker.color.a = 0.5

        marker.lifetime.sec = int(self.marker_lifetime)
        marker.lifetime.nanosec = int((self.marker_lifetime % 1.0) * 1e9)

        return marker

    @staticmethod
    def _class_color(class_id):
        # Deterministic per-class color: hash the class name into a hue-ish RGB
        # triple so the same class always renders the same color across frames.
        digest = hashlib.md5(str(class_id).encode('utf-8')).digest()
        return (digest[0] / 255.0, digest[1] / 255.0, digest[2] / 255.0)


def main(args=None):
    rclpy.init(args=args)
    node = Detection3DNode()
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
