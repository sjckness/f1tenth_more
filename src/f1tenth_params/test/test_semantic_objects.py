"""
semantic_objects.py: map-frame semantic objects as odom-frame obstacle circles.

Pure, no ROS. The map -> odom tuple is MPC_corr's convention: (x, y, yaw) of
the odom origin in map, as lookup_transform('map', 'odom') returns it.
"""

import math
from types import SimpleNamespace

from f1tenth_params.semantic_objects import (
    map_point_to_odom,
    map_to_odom_from_poses,
    MEMORY,
    objects_to_odom_circles,
    odom_point_to_map,
    TRACKED,
)
import pytest


def _obj(x, y, r, state=TRACKED, confidence=1.0, class_id='chair'):
    return SimpleNamespace(x=x, y=y, effective_radius_m=r, state=state,
                           confidence=confidence, class_id=class_id)


class TestPointTransforms:

    def test_identity_edge_changes_nothing(self):
        assert map_point_to_odom(3.0, -1.0, (0.0, 0.0, 0.0)) == pytest.approx((3.0, -1.0))

    def test_translation_only(self):
        # odom origin at map (1, 2): a map point there is odom (0, 0).
        assert map_point_to_odom(1.0, 2.0, (1.0, 2.0, 0.0)) == pytest.approx((0.0, 0.0))
        assert map_point_to_odom(4.0, 2.0, (1.0, 2.0, 0.0)) == pytest.approx((3.0, 0.0))

    def test_rotation_only(self):
        # odom rotated +90 deg in map: map +y is odom +x.
        assert map_point_to_odom(0.0, 2.0, (0.0, 0.0, math.pi / 2)) == pytest.approx((2.0, 0.0))

    @pytest.mark.parametrize('edge', [(0.3, -0.7, 0.4), (-5.0, 2.0, -2.9), (0.0, 0.0, math.pi)])
    def test_round_trip(self, edge):
        x_o, y_o = map_point_to_odom(2.5, 1.5, edge)
        assert odom_point_to_map(x_o, y_o, edge) == pytest.approx((2.5, 1.5))

    def test_matches_the_mpc_pose_convention(self):
        """Same arithmetic as MPC_corr._pose_map_to_odom (not imported: no dependency)."""
        tf_x, tf_y, tf_yaw = 0.5, -0.25, 0.3
        x, y = 2.0, 1.0
        c, s = math.cos(tf_yaw), math.sin(tf_yaw)
        expected = (c * (x - tf_x) + s * (y - tf_y), -s * (x - tf_x) + c * (y - tf_y))
        assert map_point_to_odom(x, y, (tf_x, tf_y, tf_yaw)) == pytest.approx(expected)


class TestEdgeFromPoses:

    def test_recovers_the_edge(self):
        edge = (0.4, -1.1, 0.25)
        base_in_odom = (2.0, 0.5, 0.6)
        bx, by = odom_point_to_map(base_in_odom[0], base_in_odom[1], edge)
        base_in_map = (bx, by, base_in_odom[2] + edge[2])
        assert map_to_odom_from_poses(base_in_map, base_in_odom) == pytest.approx(edge)

    def test_yaw_is_wrapped(self):
        _x, _y, yaw = map_to_odom_from_poses((0.0, 0.0, 3.0), (0.0, 0.0, -3.0))
        assert -math.pi <= yaw <= math.pi


class TestCircles:

    def test_effective_radius_and_odom_position(self):
        circles = objects_to_odom_circles([_obj(3.0, 0.0, 0.85, MEMORY, 0.4, 'person')],
                                          (1.0, 0.0, 0.0))
        assert circles == [pytest.approx((2.0, 0.0, 0.85))]

    def test_memory_can_be_left_out(self):
        objs = [_obj(1.0, 0.0, 0.2), _obj(2.0, 0.0, 0.3, MEMORY, 0.5)]
        assert len(objects_to_odom_circles(objs, (0.0, 0.0, 0.0))) == 2
        only = objects_to_odom_circles(objs, (0.0, 0.0, 0.0), include_memory=False)
        assert only == [pytest.approx((1.0, 0.0, 0.2))]

    def test_low_confidence_can_be_dropped(self):
        objs = [_obj(1.0, 0.0, 0.2), _obj(2.0, 0.0, 0.3, MEMORY, 0.05)]
        assert len(objects_to_odom_circles(objs, (0.0, 0.0, 0.0), min_confidence=0.1)) == 1

    def test_class_margins_dict_or_json(self):
        objs = [_obj(1.0, 0.0, 0.25, class_id='person'), _obj(2.0, 0.0, 0.2)]
        as_dict = objects_to_odom_circles(objs, (0.0, 0.0, 0.0), class_margins={'person': 0.3})
        as_json = objects_to_odom_circles(objs, (0.0, 0.0, 0.0), class_margins='{"person": 0.3}')
        assert as_dict == as_json
        assert as_dict[0][2] == pytest.approx(0.55) and as_dict[1][2] == pytest.approx(0.2)

    def test_empty(self):
        assert objects_to_odom_circles([], (0.0, 0.0, 0.0)) == []

    def test_a_map_odom_jump_moves_the_circle_in_odom_not_the_object_in_map(self):
        obj = _obj(3.0, 1.0, 0.25, MEMORY, 0.6)
        before = objects_to_odom_circles([obj], (0.0, 0.0, 0.0))[0]
        after_edge = (0.5, -0.3, 0.15)
        after = objects_to_odom_circles([obj], after_edge)[0]
        assert before[:2] != pytest.approx(after[:2])
        assert odom_point_to_map(after[0], after[1], after_edge) == pytest.approx((3.0, 1.0))
        assert (obj.x, obj.y) == (3.0, 1.0)
