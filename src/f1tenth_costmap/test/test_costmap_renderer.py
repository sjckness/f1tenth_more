"""costmap_renderer.py tests -- mirrors f1tenth_perception/test/test_lidar_
boundary.py's own convention: static/synthetic only, no live hardware, no
rclpy Node instantiation required.

Run standalone: python3 -m pytest test/test_costmap_renderer.py -v
"""

import numpy as np
import pytest
from builtin_interfaces.msg import Time

from f1tenth_costmap.costmap_renderer import (
    _UNKNOWN_COLOR,
    draw_semantic_objects,
    occupancy_to_rgb,
    render_costmap_rgb,
    rgb_array_to_image_msg,
    world_to_pixel,
)


# ==============================================================================
# occupancy_to_rgb
# ==============================================================================

class TestOccupancyToRgb:

    def test_free_cell_renders_white(self):
        rgb = occupancy_to_rgb([0], width=1, height=1)
        assert tuple(rgb[0, 0]) == (255, 255, 255)

    def test_occupied_cell_renders_black(self):
        rgb = occupancy_to_rgb([100], width=1, height=1)
        assert tuple(rgb[0, 0]) == (0, 0, 0)

    def test_unknown_cell_renders_distinct_gray(self):
        rgb = occupancy_to_rgb([-1], width=1, height=1)
        assert tuple(rgb[0, 0]) == _UNKNOWN_COLOR
        # Distinct from BOTH ramp endpoints -- see module docstring on why
        # unknown must not silently read as "probably free".
        assert _UNKNOWN_COLOR not in ((255, 255, 255), (0, 0, 0))

    def test_intermediate_value_interpolates(self):
        rgb = occupancy_to_rgb([50], width=1, height=1)
        gray = rgb[0, 0, 0]
        assert 0 < gray < 255

    def test_shape_and_dtype(self):
        rgb = occupancy_to_rgb([0, 100, -1, 50], width=2, height=2)
        assert rgb.shape == (2, 2, 3)
        assert rgb.dtype == np.uint8

    def test_row_major_layout_preserved(self):
        # 2x1 grid: row0=[free, occupied] -- confirms reshape uses the same
        # row-major convention OccupancyGrid.data itself documents.
        rgb = occupancy_to_rgb([0, 100], width=2, height=1)
        assert tuple(rgb[0, 0]) == (255, 255, 255)
        assert tuple(rgb[0, 1]) == (0, 0, 0)


# ==============================================================================
# world_to_pixel
# ==============================================================================

class TestWorldToPixel:

    def test_origin_maps_to_bottom_left_of_a_square_grid(self):
        # origin=(0,0), resolution=1.0, height=10 -- grid cell (0,0) (map-
        # frame (0,0)) is grid_row 0, which the Y-flip (see module docstring)
        # puts at image row height-1 (the BOTTOM of the image).
        col, row = world_to_pixel(0.0, 0.0, origin_x=0.0, origin_y=0.0, resolution=1.0, height=10)
        assert (col, row) == (0, 9)

    def test_max_y_maps_to_top_of_image(self):
        col, row = world_to_pixel(0.0, 9.0, origin_x=0.0, origin_y=0.0, resolution=1.0, height=10)
        assert (col, row) == (0, 0)

    def test_resolution_scales_correctly(self):
        col, row = world_to_pixel(
            1.0, 0.0, origin_x=0.0, origin_y=0.0, resolution=0.5, height=10)
        assert col == 2  # 1.0m / 0.5m-per-cell

    def test_origin_offset_shifts_the_result(self):
        col, row = world_to_pixel(
            5.0, 5.0, origin_x=5.0, origin_y=5.0, resolution=1.0, height=10)
        assert (col, row) == (0, 9)  # (5,5) IS the origin here -- same as (0,0) test above


# ==============================================================================
# draw_semantic_objects
# ==============================================================================

class TestDrawSemanticObjects:

    def test_draws_a_colored_square_at_the_right_pixel(self):
        rgb = np.zeros((20, 20, 3), dtype=np.uint8)
        draw_semantic_objects(
            rgb, [(2.0, 2.0, (255, 0, 0))], origin_x=0.0, origin_y=0.0,
            resolution=1.0, radius_px=1)
        col, row = world_to_pixel(2.0, 2.0, 0.0, 0.0, 1.0, height=20)
        assert tuple(rgb[row, col]) == (255, 0, 0)

    def test_out_of_bounds_object_is_silently_skipped(self):
        rgb = np.zeros((10, 10, 3), dtype=np.uint8)
        result = draw_semantic_objects(
            rgb, [(1000.0, 1000.0, (255, 0, 0))], origin_x=0.0, origin_y=0.0,
            resolution=1.0, radius_px=1)
        # No exception, image unchanged (still all zeros).
        assert np.all(result == 0)

    def test_mutates_in_place_and_returns_the_same_array(self):
        rgb = np.zeros((10, 10, 3), dtype=np.uint8)
        result = draw_semantic_objects(
            rgb, [(5.0, 5.0, (1, 2, 3))], origin_x=0.0, origin_y=0.0, resolution=1.0)
        assert result is rgb

    def test_radius_controls_square_size(self):
        rgb = np.zeros((20, 20, 3), dtype=np.uint8)
        draw_semantic_objects(
            rgb, [(10.0, 10.0, (9, 9, 9))], origin_x=0.0, origin_y=0.0,
            resolution=1.0, radius_px=2)
        col, row = world_to_pixel(10.0, 10.0, 0.0, 0.0, 1.0, height=20)
        # A radius_px=2 square is 5x5 -- corner offsets within that square
        # must be painted, one step further out must not be.
        assert tuple(rgb[row - 2, col - 2]) == (9, 9, 9)
        assert tuple(rgb[row - 3, col]) == (0, 0, 0)


# ==============================================================================
# render_costmap_rgb (full combined render)
# ==============================================================================

class TestRenderCostmapRgb:

    def test_combines_occupancy_and_semantic_layers(self):
        grid = [0] * 100  # 10x10, all free
        semantic = [(5.0, 5.0, (1.0, 0.0, 0.0))]  # float [0,1] color, as
        # semantic_layer_node.py's own _color_for_class produces.
        rgb = render_costmap_rgb(
            grid, width=10, height=10, resolution=1.0, origin_x=0.0, origin_y=0.0,
            semantic_objects=semantic)
        col, row = world_to_pixel(5.0, 5.0, 0.0, 0.0, 1.0, height=10)
        assert tuple(rgb[row, col]) == (255, 0, 0)
        # An untouched cell must stay the plain occupancy-layer white.
        assert tuple(rgb[0, 0]) == (255, 255, 255)

    def test_empty_semantic_list_leaves_pure_occupancy_render(self):
        grid = [100] * 4
        rgb = render_costmap_rgb(
            grid, width=2, height=2, resolution=1.0, origin_x=0.0, origin_y=0.0,
            semantic_objects=[])
        assert np.all(rgb == 0)

    def test_color_scaling_from_float_0_1_to_uint8_0_255(self):
        grid = [0] * 4
        rgb = render_costmap_rgb(
            grid, width=2, height=2, resolution=1.0, origin_x=0.0, origin_y=0.0,
            semantic_objects=[(0.0, 0.0, (0.5, 0.5, 0.5))])
        col, row = world_to_pixel(0.0, 0.0, 0.0, 0.0, 1.0, height=2)
        assert tuple(rgb[row, col]) == (128, 128, 128)  # round(0.5*255)


# ==============================================================================
# rgb_array_to_image_msg
# ==============================================================================

class TestRgbArrayToImageMsg:

    def test_packs_shape_and_encoding_correctly(self):
        rgb = np.zeros((4, 8, 3), dtype=np.uint8)
        stamp = Time(sec=123, nanosec=456)
        msg = rgb_array_to_image_msg(rgb, frame_id='map', stamp=stamp)
        assert msg.height == 4
        assert msg.width == 8
        assert msg.encoding == 'rgb8'
        assert msg.step == 8 * 3
        assert msg.header.frame_id == 'map'
        assert msg.header.stamp == stamp
        assert len(msg.data) == 4 * 8 * 3

    def test_data_round_trips_correctly(self):
        rgb = np.zeros((2, 2, 3), dtype=np.uint8)
        rgb[0, 0] = (10, 20, 30)
        rgb[1, 1] = (200, 100, 50)
        msg = rgb_array_to_image_msg(rgb, frame_id='map', stamp=Time())
        recovered = np.frombuffer(bytes(msg.data), dtype=np.uint8).reshape(2, 2, 3)
        assert np.array_equal(recovered, rgb)


if __name__ == '__main__':
    import sys
    sys.exit(pytest.main([__file__, '-v']))
