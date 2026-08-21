"""Costmap PNG-for-Foxglove render -- pure functions, no rclpy dependency,
independently unit-testable (same "pure logic separate from ROS glue"
convention this whole codebase's other render/geometry modules already use).

Combines the two layers Part C keeps genuinely independent (see semantic_
layer.py's own module docstring) into ONE rgb8 image, only at render time --
this module is the one place they actually meet.

"PNG for Foxglove" (the task's own framing) means sensor_msgs/Image with
encoding='rgb8' -- a RAW bitmap message, not a literal PNG-encoded byte
blob (sensor_msgs/CompressedImage would be that) -- Foxglove's own Image
panel renders raw sensor_msgs/Image directly with zero extra configuration,
which is the actual requirement ("renders directly... with no extra
configuration"). No PIL/opencv/cv_bridge dependency needed for this: a
sensor_msgs/Image's own `data` field is just the raw row-major byte buffer,
built directly from a numpy array here.
"""

import numpy as np
from sensor_msgs.msg import Image

# Occupancy-grid rendering palette. nav_msgs/OccupancyGrid.data values: -1
# (unknown), 0 (free) .. 100 (occupied), everything in between a probability.
_UNKNOWN_COLOR = (128, 128, 128)  # mid-gray, distinct from both the free/
# occupied grayscale ramp's own endpoints (255=free white, 0=occupied black)
# so "never observed" reads as visually distinct from "observed and mostly
# clear", not silently folded into the free end of the same ramp.


def occupancy_to_rgb(grid_data, width: int, height: int) -> np.ndarray:
    """(height, width, 3) uint8 -- one row-major nav_msgs/OccupancyGrid.data
    array rendered as a standard grayscale costmap: free (0) -> white,
    occupied (100) -> black, linear interpolation between, unknown (-1) ->
    _UNKNOWN_COLOR. `grid_data` may be any int-convertible sequence (a real
    OccupancyGrid.data array/array.array, or a plain Python list in tests)."""
    arr = np.asarray(grid_data, dtype=np.int16).reshape(height, width)
    unknown_mask = arr < 0
    known = np.clip(arr, 0, 100).astype(np.float32)
    gray = (255.0 * (1.0 - known / 100.0)).astype(np.uint8)
    rgb = np.stack([gray, gray, gray], axis=-1)
    rgb[unknown_mask] = _UNKNOWN_COLOR
    return rgb


def world_to_pixel(
        x_map: float, y_map: float, origin_x: float, origin_y: float,
        resolution: float, height: int) -> tuple:
    """Map-frame (x, y) -> (col, row) pixel indices in the image occupancy_
    to_rgb produces for the SAME grid (same origin/resolution/height).
    origin_x/origin_y: the OccupancyGrid's own `info.origin.position` (the
    map-frame pose of grid cell (0, 0)) -- NOT the robot's own pose.

    Row is flipped (image row 0 = grid's own highest-Y row) so the rendered
    image reads with +Y toward the TOP -- the conventional "north-up" map-
    image orientation (matches how RViz/Nav2's own costmap displays render,
    +X right/+Y up), not raw row-major top-to-bottom (which would render the
    map upside down relative to that convention)."""
    col = int(round((x_map - origin_x) / resolution))
    grid_row = int(round((y_map - origin_y) / resolution))
    row = height - 1 - grid_row
    return col, row


def draw_semantic_objects(
        rgb: np.ndarray, objects_255: list, origin_x: float, origin_y: float,
        resolution: float, radius_px: int = 3) -> np.ndarray:
    """Draws each (x_map, y_map, (r, g, b)) in `objects_255` ((r, g, b) already
    0-255 uint8-range ints, see render_costmap_rgb) as a filled square onto
    `rgb` -- mutated in place AND returned, matching numpy's own in-place-
    slice-assignment idiom rather than pretending this is side-effect-free.
    A plain filled square (not a circle) deliberately -- no PIL/opencv
    dependency for a antialiased circle primitive, and at this render's own
    scale (radius_px single digits) the visual difference from a true circle
    is negligible. Objects outside the image bounds are silently skipped,
    not an error -- a semantic object can legitimately sit outside the
    CURRENT occupancy grid's own bounds (slam_toolbox's map grows over time;
    an object seen before the grid expanded to cover that area is not a bug)."""
    h, w = rgb.shape[0], rgb.shape[1]
    for x_map, y_map, color in objects_255:
        col, row = world_to_pixel(x_map, y_map, origin_x, origin_y, resolution, h)
        if not (0 <= col < w and 0 <= row < h):
            continue
        r0, r1 = max(0, row - radius_px), min(h, row + radius_px + 1)
        c0, c1 = max(0, col - radius_px), min(w, col + radius_px + 1)
        rgb[r0:r1, c0:c1] = color
    return rgb


def render_costmap_rgb(
        grid_data, width: int, height: int, resolution: float,
        origin_x: float, origin_y: float, semantic_objects: list) -> np.ndarray:
    """Full combined render -- occupancy base (occupancy_to_rgb) + semantic
    overlay (draw_semantic_objects), one (height, width, 3) uint8 array.
    `semantic_objects`: list of (x_map, y_map, (r, g, b)) with (r, g, b) each
    a float in [0, 1] (the same shape semantic_layer_node.py's own
    _color_for_class already produces -- converted to the 0-255 range here,
    once, rather than every caller doing its own scaling)."""
    rgb = occupancy_to_rgb(grid_data, width, height)
    objects_255 = [
        (x, y, tuple(int(round(c * 255)) for c in color))
        for x, y, color in semantic_objects
    ]
    return draw_semantic_objects(rgb, objects_255, origin_x, origin_y, resolution)


def rgb_array_to_image_msg(rgb: np.ndarray, frame_id: str, stamp) -> Image:
    """Packs an (H, W, 3) uint8 array into a sensor_msgs/Image, encoding=
    'rgb8' -- see module docstring's own "PNG for Foxglove" section for why
    this (not CompressedImage) is the right message type. `stamp`: an
    already-built builtin_interfaces/Time (e.g. node.get_clock().now().
    to_msg()) -- not computed here, so this stays a pure function of its
    arguments."""
    msg = Image()
    msg.header.frame_id = frame_id
    msg.header.stamp = stamp
    msg.height = rgb.shape[0]
    msg.width = rgb.shape[1]
    msg.encoding = 'rgb8'
    msg.is_bigendian = 0
    msg.step = rgb.shape[1] * 3
    msg.data = np.ascontiguousarray(rgb, dtype=np.uint8).tobytes()
    return msg
