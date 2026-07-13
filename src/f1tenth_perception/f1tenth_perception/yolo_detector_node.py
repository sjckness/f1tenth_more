#!/usr/bin/env python3
"""YOLO 2D object detector node for the F1TENTH perception stack.

Subscribes to a raw RGB image stream, runs an Ultralytics YOLO model, and
publishes:
  * vision_msgs/Detection2DArray on `detections_topic` (default /camera/detections)
  * sensor_msgs/Image           on `annotated_topic`  (default /camera/image_annotated)
    with, for every detection, the bounding box + class name + confidence drawn
    on the frame.

The input topic defaults to **/camera/image_raw** so the node is agnostic to the
active camera source (v4l2 webcam or ZED2 wrapper); the bringup launch remaps
whichever camera is selected onto that canonical topic.

If no model is configured (`model_path` empty) OR Ultralytics is not importable,
the node runs in **passthrough mode**: it publishes an empty Detection2DArray and
re-publishes the incoming frame on the annotated topic (so both topics stay live
and downstream tooling still sees a stream).

The `device` parameter (default `cuda`) selects the torch device YOLO runs on.
If `cuda` is requested but `torch.cuda.is_available()` is False (e.g. a CPU-only
torch wheel on a CUDA-capable host), the node raises at startup rather than
silently running inference on the CPU -- pass `device:=cpu` to opt into CPU
explicitly (the CPU path is still fully supported, just never used implicitly).

`model_path` may point at a native `*.pt` checkpoint (runs via torch/cuBLAS) or
a `*.engine` file built for this exact machine via TensorRT:
    yolo export model=yolo26s.pt format=onnx device=cpu simplify=True
    trtexec --onnx=yolo26s.onnx --saveEngine=yolo26s.engine --fp16
(ONNX export runs on CPU deliberately -- it only traces the graph once, and
doing so avoids depending on GPU torch/cuBLAS for the export step itself.)
Engines are NOT portable across devices/TensorRT versions, even same-model
Jetsons, and must be rebuilt per host -- see .gitignore, these are never
committed. TensorRT engines are CUDA-only and already bound to a device at
build time, so `Model.to(device)` is skipped for them (it would raise) and
`device:='cpu'` is rejected outright for a `*.engine` model_path.

Message field reference (vision_msgs, Humble == "4.x" layout):
    Detection2DArray.detections[]            -> Detection2D
    Detection2D.results[]                     -> ObjectHypothesisWithPose
    ObjectHypothesisWithPose.hypothesis       -> ObjectHypothesis
        .hypothesis.class_id  (string)        -> class name
        .hypothesis.score     (float)         -> confidence
    Detection2D.bbox                          -> BoundingBox2D
        .bbox.center          (vision_msgs/Pose2D)
        .bbox.center.position.x / .y          -> bbox center in pixels
        .bbox.size_x / .bbox.size_y           -> bbox width / height in pixels
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
        # image_topic defaults to the canonical /camera/image_raw so the same
        # node works whether the webcam (v4l2_camera) or the ZED2 wrapper is the
        # active source; the bringup launch remaps the selected camera onto it.
        self.image_topic = str(
            self.declare_parameter('image_topic', '/camera/image_raw').value)
        self.detections_topic = str(
            self.declare_parameter('detections_topic', '/camera/detections').value)
        self.annotated_topic = str(
            self.declare_parameter('annotated_topic', '/camera/image_annotated').value)
        self.model_path = str(self.declare_parameter('model_path', '').value)
        # Torch device YOLO runs inference on. Default 'cuda' matches the
        # Jetson Orin deployment target; pass device:=cpu to explicitly opt
        # into the (still fully supported) CPU path.
        self.device = str(self.declare_parameter('device', 'cuda').value)

        # ---- model ---------------------------------------------------------
        self.model = None
        if not self.model_path:
            self.get_logger().warn(
                'No YOLO model_path set — running in passthrough mode '
                '(empty detections, annotated == raw frame)')
        else:
            try:
                from ultralytics import YOLO
                import torch
            except Exception as exc:  # noqa: BLE001 - stay alive in passthrough
                self.get_logger().error(
                    f'Failed to import Ultralytics/torch: {exc} — falling '
                    'back to passthrough mode')
            else:
                is_engine = self.model_path.endswith('.engine')
                if is_engine and self.device == 'cpu':
                    raise RuntimeError(
                        f'model_path="{self.model_path}" is a TensorRT engine, '
                        'which is CUDA-only and already bound to a device at '
                        'build time -- device:="cpu" is not valid for it. Use '
                        'a *.pt checkpoint for CPU inference instead.')
                if self.device == 'cuda' and not torch.cuda.is_available():
                    # Deliberately NOT caught below / not a passthrough
                    # fallback: a CPU-only torch wheel silently serving
                    # "working" but far slower inference is worse than a
                    # loud startup failure. See module docstring.
                    raise RuntimeError(
                        f'device="cuda" requested but torch.cuda.is_available() '
                        f'is False (torch {torch.__version__}). On Jetson this '
                        'usually means a generic CPU-only wheel is installed '
                        'instead of a JetPack-matched CUDA build. Pass '
                        'device:=cpu to explicitly run on CPU instead.')
                try:
                    self.model = YOLO(self.model_path, task='detect')
                    if not is_engine:
                        # TensorRT engines are already device-bound at build
                        # time; Model.to() only supports native *.pt models
                        # and raises TypeError otherwise.
                        self.model.to(self.device)
                    self.get_logger().info(
                        f'Loaded YOLO model "{self.model_path}" on '
                        f'device="{self.device}"')
                except Exception as exc:  # noqa: BLE001 - stay alive in passthrough
                    self.get_logger().error(
                        f'Failed to load Ultralytics YOLO ("{self.model_path}") '
                        f'on device="{self.device}": {exc} — falling back to '
                        'passthrough mode')
                    self.model = None

        # ---- ROS interfaces -----------------------------------------------
        self.bridge = CvBridge()
        self.det_pub = self.create_publisher(
            Detection2DArray, self.detections_topic, 10)
        self.annotated_pub = self.create_publisher(
            Image, self.annotated_topic, 10)
        self.sub = self.create_subscription(
            Image, self.image_topic, self.image_callback, 10)

        self.get_logger().info(
            f'yolo_detector_node up: subscribing "{self.image_topic}", '
            f'publishing detections "{self.detections_topic}" and annotated '
            f'"{self.annotated_topic}"')

    def image_callback(self, msg: Image):
        # Convert ROS Image -> OpenCV BGR (3-channel) for inference/drawing.
        # cv_bridge handles rgb8 (webcam) and bgra8 (ZED) -> bgr8 conversion.
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except CvBridgeError as exc:
            self.get_logger().error(f'cv_bridge conversion failed: {exc}')
            return

        out = Detection2DArray()
        out.header = msg.header  # keep detections time/frame aligned to the image

        if self.model is None:
            # Passthrough: empty detections, re-publish the frame unmodified so
            # the annotated topic stays live.
            self.det_pub.publish(out)
            self._publish_annotated(cv_image, msg.header)
            return

        # ---- real inference ------------------------------------------------
        results = self.model(cv_image, verbose=False, device=self.device)
        result = results[0]
        names = result.names

        for box in result.boxes:
            x1, y1, x2, y2 = (float(v) for v in box.xyxy[0].tolist())
            cls_id = int(box.cls[0].item())
            score = float(box.conf[0].item())
            cx = (x1 + x2) / 2.0
            cy = (y1 + y2) / 2.0
            out.detections.append(self._build_detection(
                class_id=names.get(cls_id, str(cls_id)), score=score,
                cx=cx, cy=cy, w=(x2 - x1), h=(y2 - y1)))

        self.det_pub.publish(out)

        # results.plot() renders boxes + "class conf" labels (BGR ndarray).
        annotated = result.plot()
        self._publish_annotated(annotated, msg.header)

    def _publish_annotated(self, cv_image, header):
        try:
            annotated_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')
        except CvBridgeError as exc:
            self.get_logger().error(f'cv_bridge annotated encode failed: {exc}')
            return
        annotated_msg.header = header
        self.annotated_pub.publish(annotated_msg)

    @staticmethod
    def _build_detection(class_id, score, cx, cy, w, h):
        """Build one Detection2D using the Humble vision_msgs API.

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
