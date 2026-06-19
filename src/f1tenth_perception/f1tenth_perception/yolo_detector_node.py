#!/usr/bin/env python3
"""YOLO 2D object detector node for the F1TENTH perception stack.

Subscribes to a ZED2 RGB image stream, (eventually) runs a YOLO model, and
publishes detections as vision_msgs/Detection2DArray on /yolo/2D_detections.

Until a model is wired in, the node runs in **passthrough mode**: it converts
each incoming frame with cv_bridge (to prove the pipeline is alive) and
publishes an *empty* Detection2DArray, stamped with the source image header.

Message field reference (vision_msgs, Humble/Jazzy == "4.x" layout):
    Detection2DArray.detections[]            -> Detection2D
    Detection2D.results[]                     -> ObjectHypothesisWithPose
    ObjectHypothesisWithPose.hypothesis       -> ObjectHypothesis
        .hypothesis.class_id  (string)        -> class name
        .hypothesis.score     (float)         -> confidence
    Detection2D.bbox                          -> BoundingBox2D
        .bbox.center          (vision_msgs/Pose2D)
        .bbox.center.position.x / .y          -> bbox center in pixels
        .bbox.size_x / .bbox.size_y           -> bbox width / height in pixels

NOTE: this is the modern API. In old (Foxy/Galactic) vision_msgs the center was
a geometry_msgs/Pose2D accessed as bbox.center.x/.y. On Humble/Jazzy it is
vision_msgs/Pose2D and the access is bbox.center.position.x/.y (used below).
"""

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from vision_msgs.msg import (
    Detection2D,
    Detection2DArray,
    ObjectHypothesisWithPose,
)

from cv_bridge import CvBridge, CvBridgeError


class YoloDetectorNode(Node):
    def __init__(self):
        super().__init__('yolo_detector_node')

        # ---- parameters ----------------------------------------------------
        # image_topic defaults to the conceptual /zed2/rgb name; the launch file
        # wires it to the real ZED topic (/zed2/zed_node/rgb/image_rect_color),
        # since the ZED wrapper cannot rename its published topics itself.
        self.image_topic = str(
            self.declare_parameter('image_topic', '/zed2/rgb').value)
        self.detections_topic = str(
            self.declare_parameter('detections_topic', '/yolo/2D_detections').value)
        self.model_path = str(self.declare_parameter('model_path', '').value)

        # ---- model (placeholder) ------------------------------------------
        self.model = None
        if not self.model_path:
            self.get_logger().warn(
                'No YOLO model loaded — running in passthrough mode')
        else:
            # TODO: load model here, e.g.
            #   from ultralytics import YOLO
            #   self.model = YOLO(self.model_path)
            self.get_logger().info(
                f'model_path set to "{self.model_path}" but model loading is '
                'not implemented yet — running in passthrough mode')

        # ---- ROS interfaces -----------------------------------------------
        self.bridge = CvBridge()
        self.pub = self.create_publisher(Detection2DArray, self.detections_topic, 10)
        self.sub = self.create_subscription(
            Image, self.image_topic, self.image_callback, 10)

        self.get_logger().info(
            f'yolo_detector_node up: subscribing "{self.image_topic}", '
            f'publishing "{self.detections_topic}"')

    def image_callback(self, msg: Image):
        # Convert ROS Image -> OpenCV (BGR). ZED rgb/image_rect_color is bgra8;
        # desired_encoding='bgr8' gives a standard 3-channel image for inference.
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except CvBridgeError as exc:
            self.get_logger().error(f'cv_bridge conversion failed: {exc}')
            return

        out = Detection2DArray()
        out.header = msg.header  # keep detections time/frame aligned to the image

        if self.model is None:
            # Passthrough: confirm the frame arrived, publish an empty array.
            _ = cv_image  # frame is decoded; no inference yet
            self.pub.publish(out)
            return

        # TODO: real inference path (kept ready for when a model is loaded):
        #   results = self.model(cv_image)
        #   for det in results:
        #       out.detections.append(self._build_detection(
        #           class_id=det.name, score=det.conf,
        #           cx=det.cx, cy=det.cy, w=det.w, h=det.h))
        self.pub.publish(out)

    @staticmethod
    def _build_detection(class_id, score, cx, cy, w, h):
        """Build one Detection2D using the Humble/Jazzy vision_msgs API.

        cx, cy are the bbox center in pixels; w, h are width/height in pixels.
        """
        det = Detection2D()

        hyp = ObjectHypothesisWithPose()
        hyp.hypothesis.class_id = str(class_id)
        hyp.hypothesis.score = float(score)
        det.results.append(hyp)

        det.bbox.center.position.x = float(cx)
        det.bbox.center.position.y = float(cy)
        det.bbox.center.theta = 0.0
        det.bbox.size_x = float(w)
        det.bbox.size_y = float(h)
        return det


def main(args=None):
    rclpy.init(args=args)
    node = YoloDetectorNode()
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
