#!/usr/bin/env python3
"""Raw ZED-depth front-distance monitor -- last-resort hardware proximity input.

Subscribes DIRECTLY to the ZED depth image (same raw topic detection_3d_node
uses, default /zed2/zed_node/depth/depth_registered) and publishes the closest
plausible distance in a fixed, centered forward ROI as std_msgs/Float32 on
/perception/front_distance -- the SAME topic MPC_corr.py already subscribes to
(front_distance_callback) but that had no publisher anywhere in the stack
before this node existed (MPC_corr.py's own self.front_distance default of
10.0 was therefore always what it saw).

Deliberately has NO dependency on yolo_detector_node/detection_3d_node/
obstacle_projector_node's output (Detection2DArray/Detection3DArray/
Obstacle2DArray) -- this is meant to keep working as a last-resort hardware
proximity signal even if the YOLO/classification pipeline is degraded,
lagging, or misconfigured, per f1tenth_behavior's IsProximityTooClose BT
condition (the emergency lane), which consumes this topic together with raw
/scan for the side/rear check. See that behaviour's own docstring for the
full design.

Consumed also by MPC_corr.py -- as a side effect of finally having a real
publisher, MPC_corr's own front_distance-based corridor-length logic
(previously always seeing the hardcoded 10.0 "clear" fallback) now sees real
data. Flagged explicitly, not silent: this is a behavior change to MPC_corr
beyond what the emergency-stop leaf itself needed, chosen deliberately (see
the chat deliberation) over inventing a second, dedicated topic.

ROI + statistic: a `center_fraction`-sized box (default 0.5, same convention/
default as detection_3d_node's own per-detection ROI sampling) centered in the
depth frame, treated as "what's directly ahead." Within it, invalid pixels
(non-finite, or below min_valid_depth -- the same ~0.2-0.3 m ZED
minimum-stereo-range plausibility floor detection_3d_node itself applies to
its own per-detection depth reads) are dropped; the
`front_distance_percentile`-th percentile (default 5.0, NOT the bare minimum)
of what's left is published -- a robust stand-in for "closest point" that
isn't derailed by one or two noisy near-zero pixels, while still being
conservative (front_distance_percentile=0 would be the literal min).

If a frame has no valid pixels in the ROI at all (depth completely unreadable
that frame), nothing is published for it -- silently reporting a fabricated
"safe" or "danger" value on real sensor dropout would be worse than just not
updating. IsProximityTooClose (like IsBatteryLow) treats "no message received
yet" as FAILURE (not tripped), not as a green light, so a cold start or a
sensor glitch does not itself trigger the emergency lane.
"""

import numpy as np
import rclpy
from cv_bridge import CvBridge, CvBridgeError
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32


class FrontDepthMonitorNode(Node):
    def __init__(self):
        super().__init__('front_depth_monitor_node')

        self.depth_topic = str(
            self.declare_parameter(
                'depth_topic', '/zed2/zed_node/depth/depth_registered').value)
        self.front_distance_topic = str(
            self.declare_parameter(
                'front_distance_topic', '/perception/front_distance').value)
        # Same convention/default as detection_3d_node's own center_fraction --
        # fraction of the frame (in each axis) sampled around the image center.
        self.center_fraction = float(
            self.declare_parameter('center_fraction', 0.5).value)
        # Same ~0.2-0.3 m ZED minimum-stereo-range plausibility floor
        # detection_3d_node applies to its own per-detection depth reads.
        self.min_valid_depth = float(
            self.declare_parameter('min_valid_depth', 0.2).value)
        # Percentile (0-100) of valid ROI pixels published as "front distance"
        # -- 0 would be the bare minimum (most conservative, most noise-prone);
        # this default trades a little conservatism for robustness against a
        # handful of spurious near-zero pixels.
        self.front_distance_percentile = float(
            self.declare_parameter('front_distance_percentile', 5.0).value)

        self.bridge = CvBridge()
        self.front_distance_pub = self.create_publisher(
            Float32, self.front_distance_topic, 10)
        self.depth_sub = self.create_subscription(
            Image, self.depth_topic, self._depth_callback, 10)

        self.get_logger().info(
            f'front_depth_monitor_node up: "{self.depth_topic}" -> '
            f'"{self.front_distance_topic}", center_fraction={self.center_fraction}, '
            f'min_valid_depth={self.min_valid_depth} m, '
            f'percentile={self.front_distance_percentile}')

    def _depth_callback(self, msg: Image):
        try:
            depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')
        except CvBridgeError as exc:
            self.get_logger().error(f'cv_bridge depth conversion failed: {exc}')
            return

        h, w = depth.shape[:2]
        half_w = (w * self.center_fraction) / 2.0
        half_h = (h * self.center_fraction) / 2.0
        cx, cy = w / 2.0, h / 2.0

        x1 = int(np.clip(cx - half_w, 0, w - 1))
        x2 = int(np.clip(cx + half_w, 0, w - 1))
        y1 = int(np.clip(cy - half_h, 0, h - 1))
        y2 = int(np.clip(cy + half_h, 0, h - 1))
        if x2 <= x1 or y2 <= y1:
            return

        roi = depth[y1:y2, x1:x2]
        valid = roi[np.isfinite(roi) & (roi >= self.min_valid_depth)]
        if valid.size == 0:
            self.get_logger().warn(
                'No valid depth pixels in front ROI this frame -- not publishing.',
                throttle_duration_sec=5.0)
            return

        front_distance = float(np.percentile(valid, self.front_distance_percentile))
        self.front_distance_pub.publish(Float32(data=front_distance))


def main(args=None):
    rclpy.init(args=args)
    node = FrontDepthMonitorNode()
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
