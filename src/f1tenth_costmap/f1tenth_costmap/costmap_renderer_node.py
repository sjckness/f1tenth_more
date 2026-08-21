#!/usr/bin/env python3
"""
costmap_renderer_node.py

Combines the occupancy layer (slam_toolbox's own /slam/map, consumed
directly -- no logic of this node's own re-derives what slam_toolbox already
produces, per the task's own "don't rebuild occupancy logic from scratch"
instruction) and the semantic layer (semantic_layer_node.py's own
/costmap/semantic_markers) into ONE rgb8 sensor_msgs/Image, published on
/costmap/visualization for a Foxglove Image panel to render directly with no
extra configuration. See costmap_renderer.py's own module docstring for why
that's sensor_msgs/Image (raw), not sensor_msgs/CompressedImage.

Both native topics stay available on their own regardless of whether this
node is even running (/slam/map straight from slam_toolbox, /costmap/
semantic_markers from semantic_layer_node.py) -- this node is a pure,
additional visualization convenience on top of them, not a replacement for
either (see the task's own Part D.4 instruction) -- e.g. a future MPC-
constraints-from-costmap consumer would read the native OccupancyGrid/
MarkerArray topics directly, not this rendered image.

Render rate: a fixed 2Hz timer (render_rate_hz, default 2.0), NOT tied to
/slam/map's own (irregular, event-driven -- see slam_toolbox_params.yaml's
map_update_interval) publish rate, and NOT on-change. This is deliberately
the simpler of the two options the task itself offered ("2-5Hz or on-change,
your call") -- this output is a visualization convenience for a human
watching a Foxglove panel, explicitly NOT safety-relevant/real-time (per the
task's own framing), so a plain periodic timer at the low end of the
suggested range keeps the rendering cost (numpy work over a potentially
large, growing occupancy grid) infrequent without adding on-change-detection
complexity that would buy nothing a human eye could actually perceive.
Renders (and publishes) only once at least one /slam/map message has been
received -- no synthetic/placeholder image before then.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import Image
from visualization_msgs.msg import Marker, MarkerArray

from f1tenth_costmap.costmap_renderer import rgb_array_to_image_msg, render_costmap_rgb


class CostmapRendererNode(Node):
    def __init__(self):
        super().__init__('costmap_renderer_node')

        self.declare_parameter('map_topic', '/slam/map')
        self.declare_parameter('semantic_topic', '/costmap/semantic_markers')
        self.declare_parameter('output_topic', '/costmap/visualization')
        self.declare_parameter('render_rate_hz', 2.0)

        p = self.get_parameter
        render_rate_hz = p('render_rate_hz').value

        # nav_msgs/OccupancyGrid map publishers conventionally use transient-
        # local (latched) durability -- confirmed against slam_toolbox's own
        # installed source convention (map_saver/map servers across the ROS 2
        # ecosystem uniformly latch /map so a late-joining subscriber gets the
        # current map immediately, not just future updates) -- matched here
        # explicitly rather than assumed, same discipline semantic_layer_
        # node.py's own /slam/pose QoS match already follows.
        map_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.map_sub = self.create_subscription(
            OccupancyGrid, p('map_topic').value, self._map_cb, map_qos)
        self.semantic_sub = self.create_subscription(
            MarkerArray, p('semantic_topic').value, self._semantic_cb, 10)
        self.image_pub = self.create_publisher(Image, p('output_topic').value, 10)

        self._latest_grid: OccupancyGrid = None
        self._latest_semantic_objects: list = []  # [(x, y, (r, g, b)), ...]

        self.timer = self.create_timer(1.0 / render_rate_hz, self._render_tick)

        self.get_logger().info('costmap_renderer_node started')

    # ------------------------------------------------------------------
    def _map_cb(self, msg: OccupancyGrid):
        self._latest_grid = msg

    def _semantic_cb(self, msg: MarkerArray):
        # Only the CYLINDER markers carry a real position -- the paired
        # TEXT_VIEW_FACING label markers (same ns+id, see semantic_layer_
        # node.py's own _publish()) are for direct RViz/Foxglove readability
        # only and are skipped here, not double-drawn.
        objects = []
        for marker in msg.markers:
            if marker.type != Marker.CYLINDER:
                continue
            color = (marker.color.r, marker.color.g, marker.color.b)
            objects.append((marker.pose.position.x, marker.pose.position.y, color))
        self._latest_semantic_objects = objects

    # ------------------------------------------------------------------
    def _render_tick(self):
        grid = self._latest_grid
        if grid is None:
            return  # nothing to render yet -- see module docstring.

        rgb = render_costmap_rgb(
            grid.data, grid.info.width, grid.info.height, grid.info.resolution,
            grid.info.origin.position.x, grid.info.origin.position.y,
            self._latest_semantic_objects)
        image_msg = rgb_array_to_image_msg(
            rgb, frame_id=grid.header.frame_id, stamp=self.get_clock().now().to_msg())
        self.image_pub.publish(image_msg)


def main(args=None):
    rclpy.init(args=args)
    node = CostmapRendererNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
