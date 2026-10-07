"""JPEG-encode the throttled raw images for foxglove_bridge.

Part of the /viz relays (see config/viz_relays.yaml and
launch/viz_relays.launch.py). A topic_tools throttle has already cut each
raw Image down to its viz rate on /viz_internal/<source>; this node encodes
only those frames and publishes sensor_msgs/CompressedImage (format 'jpeg')
on /viz/<source>/compressed. The raw intermediate stays outside /viz/, so the
bridge whitelist can never expose it.

Lazy, like the throttles in front of it: each output publisher exists from
startup, so Foxglove can list it, but the node subscribes to its input only
while that output has a subscriber. With no subscription on
/viz_internal/<source>, the lazy throttle feeding it drops its own
subscription to the full-rate source as well, so nothing on the chain runs
while no client is watching.

Not image_transport's republish: in Humble (3.1.12) it is a standalone
executable, not a component, and it subscribes whether or not anyone is
watching.

QoS: both image sources (yolo_detector_node's /camera/image_annotated,
costmap_renderer_node's /costmap/visualization) publish RELIABLE + VOLATILE,
and the throttles copy that, so this node's endpoints are RELIABLE + VOLATILE
too. Depth 1: a viewer wants the newest frame, never a backlog.
"""
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage, Image

# encoding -> (channels, cv2 conversion to BGR/gray, or None if already so)
_ENCODINGS = {
    'bgr8': (3, None),
    'rgb8': (3, cv2.COLOR_RGB2BGR),
    'bgra8': (4, cv2.COLOR_BGRA2BGR),
    'rgba8': (4, cv2.COLOR_RGBA2BGR),
    'mono8': (1, None),
    '8UC1': (1, None),
    '8UC3': (3, None),
}

_QOS = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                  reliability=ReliabilityPolicy.RELIABLE,
                  durability=DurabilityPolicy.VOLATILE)


def encode_jpeg(encoding, width, height, step, data, quality):
    """Raw 8-bit image buffer -> JPEG bytes. Raises ValueError on anything else."""
    if encoding not in _ENCODINGS:
        raise ValueError(f'unsupported encoding {encoding!r}')
    channels, conversion = _ENCODINGS[encoding]
    rows = np.frombuffer(data, dtype=np.uint8)
    if rows.size < step * height:
        raise ValueError(f'{rows.size} bytes for {height} rows of step {step}')
    # step can carry row padding past width * channels; slice it off.
    image = rows[:step * height].reshape(height, step)[:, :width * channels]
    image = image.reshape(height, width, channels) if channels > 1 else image
    if conversion is not None:
        image = cv2.cvtColor(image, conversion)
    ok, jpeg = cv2.imencode('.jpg', image, [cv2.IMWRITE_JPEG_QUALITY, int(quality)])
    if not ok:
        raise ValueError('cv2.imencode failed')
    return jpeg.tobytes()


class VizJpegNode(Node):

    def __init__(self):
        super().__init__('viz_jpeg_node')
        inputs = list(self.declare_parameter('input_topics', ['']).value)
        outputs = list(self.declare_parameter('output_topics', ['']).value)
        self.quality = int(self.declare_parameter('jpeg_quality', 75).value)
        period = float(self.declare_parameter('check_period_sec', 1.0).value)
        pairs = [(i, o) for i, o in zip(inputs, outputs) if i and o]
        if len(inputs) != len(outputs) or not pairs:
            raise ValueError(f'input_topics {inputs} and output_topics {outputs} '
                             'must be non-empty lists of the same length')
        self._chains = [
            {'input': i, 'output': o, 'sub': None,
             'pub': self.create_publisher(CompressedImage, o, _QOS)}
            for i, o in pairs]
        self.create_timer(period, self._update_subscriptions)
        self.get_logger().info(
            f'JPEG q{self.quality}: ' + ', '.join(f'{i} -> {o}' for i, o in pairs))

    def _update_subscriptions(self):
        for chain in self._chains:
            watched = chain['pub'].get_subscription_count() > 0
            if watched and chain['sub'] is None:
                chain['sub'] = self.create_subscription(
                    Image, chain['input'],
                    lambda msg, c=chain: self._on_image(msg, c), _QOS)
                self.get_logger().info(f'{chain["output"]} watched: subscribed {chain["input"]}')
            elif not watched and chain['sub'] is not None:
                self.destroy_subscription(chain['sub'])
                chain['sub'] = None
                self.get_logger().info(f'{chain["output"]} unwatched: dropped {chain["input"]}')

    def _on_image(self, msg, chain):
        try:
            jpeg = encode_jpeg(msg.encoding, msg.width, msg.height, msg.step,
                               msg.data, self.quality)
        except ValueError as exc:
            self.get_logger().warn(f'{chain["input"]}: {exc}', throttle_duration_sec=10.0)
            return
        out = CompressedImage()
        out.header = msg.header
        out.format = 'jpeg'
        out.data = jpeg
        chain['pub'].publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = VizJpegNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
