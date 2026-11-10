#!/usr/bin/env python3
"""
Camera feeder for a bag replay (perf baseline, level B). Harness only.

The car's mission bags do not hold the camera's raw output: they hold a clean
JPEG of the left image at 480x270 and ~5 Hz and the registered depth at ~5 Hz.
The stack's camera consumers work per frame, at the camera's rate, on 640x360
frames. This node turns the two bag streams back into the two topics the ZED
wrapper publishes, at the car's frame rate:

    /perf/image_clean/compressed (CompressedImage) -> /camera/image_raw (bgra8 640x360)
    /perf/depth_in (Image 32FC1)                   -> /zed2/zed_node/depth/depth_registered

Each tick (--rate, default 30 Hz, on the node's clock = /clock in a replay) the
LATEST image and depth are published again with the tick's stamp, both with the
same one. So the consumers see the car's rates, sizes and encodings, but the
content only changes five times a second and the image was upscaled from a JPEG:
what is measured downstream is the CPU of the per-frame work, not detection
quality.

--goal-distance D (optional) publishes /mpc/goal_distance --goal-delay seconds
of replayed time after the first frame, so that mpc_corr solves during the run
(a stack that is only replayed has no mission and its MPC would idle). It is
sent again every 6 s, at most three times, for as long as no
/mpc/solver_status has been seen: mpc_corr ignores a goal that arrives before
its first odometry.
"""

import argparse
import sys

import cv2
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import CompressedImage, Image
from rosidl_runtime_py.utilities import get_message
from std_msgs.msg import Float32


class Feeder(Node):
    def __init__(self, args):
        super().__init__('perf_replay_feeder', parameter_overrides=[
            Parameter('use_sim_time', Parameter.Type.BOOL, True)])
        self.args = args
        self.size = (args.width, args.height)
        self.image = None      # (frame_id, bgra bytes)
        self.depth = None      # (frame_id, height, width, step, bytes)
        self.n_pub = 0
        best = QoSProfile(depth=2, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        out = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE)
        self.create_subscription(CompressedImage, '/perf/image_clean/compressed', self._image, best)
        self.create_subscription(Image, '/perf/depth_in', self._depth, best)
        self.image_pub = self.create_publisher(Image, '/camera/image_raw', out)
        self.depth_pub = self.create_publisher(
            Image, '/zed2/zed_node/depth/depth_registered', out)
        self.create_timer(1.0 / args.rate, self._tick)
        # The goal is timed in published frames, not with a ROS timer: on /clock a timer
        # created before the bag starts fires at once when the time jumps to the bag's.
        self.goal_pub = self.create_publisher(Float32, '/mpc/goal_distance', 10)
        self.goal_due = int(args.goal_delay * args.rate) if args.goal_distance > 0.0 else None
        self.goals_sent = 0
        self.solving = False
        self.create_subscription(
            get_message('f1tenth_messages/msg/MpcSolverStatus'), '/mpc/solver_status',
            self._solver_status, best, raw=True)

    def _solver_status(self, _raw):
        self.solving = True

    def _image(self, msg):
        img = cv2.imdecode(np.frombuffer(bytes(msg.data), np.uint8), cv2.IMREAD_COLOR)
        if img is None:
            return
        if (img.shape[1], img.shape[0]) != self.size:
            img = cv2.resize(img, self.size, interpolation=cv2.INTER_LINEAR)
        self.image = (msg.header.frame_id, cv2.cvtColor(img, cv2.COLOR_BGR2BGRA).tobytes())

    def _depth(self, msg):
        self.depth = (msg.header.frame_id, msg.height, msg.width, msg.step, msg.encoding,
                      bytes(msg.data))

    def _tick(self):
        stamp = self.get_clock().now().to_msg()
        if self.image is not None:
            frame, data = self.image
            msg = Image(height=self.size[1], width=self.size[0], encoding='bgra8',
                        is_bigendian=0, step=self.size[0] * 4)
            msg.header.stamp, msg.header.frame_id = stamp, frame
            msg.data = data
            self.image_pub.publish(msg)
            self.n_pub += 1
            if self.goal_due is not None and self.n_pub >= self.goal_due:
                self._goal()
        if self.depth is not None:
            frame, h, w, step, enc, data = self.depth
            msg = Image(height=h, width=w, encoding=enc, is_bigendian=0, step=step)
            msg.header.stamp, msg.header.frame_id = stamp, frame
            msg.data = data
            self.depth_pub.publish(msg)

    def _goal(self):
        if self.solving or self.goals_sent >= 3:
            self.goal_due = None
            return
        self.goals_sent += 1
        self.goal_due = self.n_pub + int(6.0 * self.args.rate)
        self.goal_pub.publish(Float32(data=float(self.args.goal_distance)))
        self.get_logger().info(
            f'published /mpc/goal_distance {self.args.goal_distance} m (#{self.goals_sent})')


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    ap.add_argument('--rate', type=float, default=30.0)
    ap.add_argument('--width', type=int, default=640)
    ap.add_argument('--height', type=int, default=360)
    ap.add_argument('--goal-distance', type=float, default=0.0)
    ap.add_argument('--goal-delay', type=float, default=8.0)
    args = ap.parse_args(argv)
    rclpy.init()
    node = Feeder(args)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        print(f'feeder: {node.n_pub} image frames published, {node.goals_sent} goal(s) sent, '
              f'solver_status seen: {node.solving}', file=sys.stderr)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
