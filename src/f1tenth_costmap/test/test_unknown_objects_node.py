"""
semantic_layer_node, obstacles v2 (B4/B5): the MPC's obstacle list, the speed cap, the modes.

A real (never spun) rclpy node, callbacks called directly and publishers
mocked -- test_semantic_layer_node.py's convention. Synthetic /scan of a
persistent cluster; the laser at base_link's origin (identity transform).
"""

import json
import math
from unittest.mock import MagicMock

from f1tenth_costmap import unknown_objects as uo
from f1tenth_costmap.semantic_layer import SemanticObject
from f1tenth_costmap.semantic_layer_node import SemanticLayerNode

from geometry_msgs.msg import Transform, TransformStamped

from nav_msgs.msg import Odometry

import numpy as np

import pytest

import rclpy
from rclpy.parameter import Parameter

from sensor_msgs.msg import LaserScan

from vision_msgs.msg import Detection3D, Detection3DArray, ObjectHypothesisWithPose

N_BEAMS = 1081
ANGLE_MIN = -math.radians(135.0)
INC = math.radians(0.25)


@pytest.fixture(scope='module', autouse=True)
def _rclpy_context():
    rclpy.init()
    yield
    if rclpy.ok():
        rclpy.shutdown()


def _stamp(msg, t):
    msg.header.stamp.sec = int(t)
    msg.header.stamp.nanosec = int(round((t - int(t)) * 1e9))
    return msg


def _scan(t, objects=((1.5, 0.0, 0.15),)):
    """Return a scan from base_link's origin of disks (x, y, r); nothing else returns."""
    msg = _stamp(LaserScan(), t)
    msg.header.frame_id = 'laser'
    msg.angle_min, msg.angle_increment = ANGLE_MIN, INC
    msg.angle_max = ANGLE_MIN + (N_BEAMS - 1) * INC
    msg.range_min, msg.range_max = 0.05, 30.0
    ang = ANGLE_MIN + np.arange(N_BEAMS) * INC
    d = np.stack([np.cos(ang), np.sin(ang)], axis=1)
    best = np.full(N_BEAMS, np.inf)
    for cx, cy, r in objects:
        oc = -np.array([cx, cy])
        b = d @ oc
        disc = b * b - (oc @ oc - r * r)
        t_hit = -b - np.sqrt(np.maximum(disc, 0.0))
        best = np.where((disc >= 0) & (t_hit > 0) & (t_hit < best), t_hit, best)
    msg.ranges = [float(v) for v in best]
    return msg


def _pose(t):
    msg = _stamp(Odometry(), t)
    msg.pose.pose.orientation.w = 1.0
    return msg


def _node(**params):
    node = SemanticLayerNode(parameter_overrides=[
        Parameter(k, value=v) for k, v in params.items()])
    ts = TransformStamped()
    ts.transform = Transform()
    ts.transform.rotation.w = 1.0
    node.tf_buffer.lookup_transform = MagicMock(return_value=ts)
    for pub in ('marker_pub', 'tracks_pub', 'obstacles_pub', 'speed_cap_pub',
                'unknown_debug_pub', 'cell_owners_pub', 'wall_cells_pub', 'wall_ids_pub'):
        setattr(node, pub, MagicMock())
    node.cell_owners_pub.get_subscription_count.return_value = 0
    return node


def _drive(node, cycles=10, t0=10.0, objects=((1.5, 0.0, 0.15),)):
    for i in range(cycles):
        t = t0 + 0.1 * i
        node._pose_cb(_pose(t))
        node._scan_cb(_scan(t, objects))
        node._cycle()


def _last(pub):
    return pub.publish.call_args[0][0]


class TestModes:

    def test_active_puts_the_unknown_object_on_the_mpcs_list(self):
        node = _node(unknown_objects_mode='active')
        try:
            _drive(node, objects=((2.0, 0.0, 0.15),))
            obs = _last(node.obstacles_pub)
            assert obs.header.frame_id == 'base_link'
            assert len(obs.obstacles) == 1
            o = obs.obstacles[0]
            assert (o.x, o.y) == pytest.approx((1.95, 0.0), abs=0.08)
            assert 0.1 <= o.r <= 0.3
            # Its near edge is ~1.4 m past the bumper: beyond the 1.0 m cap.
            assert _last(node.speed_cap_pub).data == pytest.approx(-1.0)
            dbg = json.loads(_last(node.unknown_debug_pub).data)
            assert dbg['unknown_confirmed'] == 1 and dbg['cells']['island'] > 0
        finally:
            node.destroy_node()

    def test_shadow_computes_everything_but_the_mpc_input_is_unchanged(self):
        node = _node(unknown_objects_mode='shadow')
        try:
            _drive(node, objects=((1.2, 0.0, 0.15),))
            assert _last(node.obstacles_pub).obstacles == []
            assert _last(node.speed_cap_pub).data == pytest.approx(-1.0)
            tracks = _last(node.tracks_pub)
            assert [d.results[0].hypothesis.class_id for d in tracks.detections] == [
                uo.UNKNOWN_CLASS]
            assert json.loads(_last(node.unknown_debug_pub).data)['unknown_confirmed'] == 1
        finally:
            node.destroy_node()

    def test_off_computes_nothing_and_still_publishes_the_list(self):
        node = _node(unknown_objects_mode='off')
        try:
            assert not hasattr(node, 'scan_sub')
            _drive(node)
            assert _last(node.obstacles_pub).obstacles == []
            assert node._objects == []
        finally:
            node.destroy_node()

    def test_a_bad_mode_refuses_to_start(self):
        with pytest.raises(ValueError):
            _node(unknown_objects_mode='maybe')


class TestSpeedCap:

    def test_an_unknown_object_close_ahead_caps_the_speed(self):
        node = _node(unknown_objects_mode='active')
        try:
            _drive(node, objects=((1.1, 0.05, 0.12),))
            assert _last(node.speed_cap_pub).data == pytest.approx(0.4)
        finally:
            node.destroy_node()

    def test_beside_the_strip_it_does_not(self):
        node = _node(unknown_objects_mode='active')
        try:
            _drive(node, objects=((1.1, 0.9, 0.12),))
            assert _last(node.speed_cap_pub).data == pytest.approx(-1.0)
        finally:
            node.destroy_node()

    def test_disabled_never_caps(self):
        node = _node(unknown_objects_mode='active', unknown_speed_cap_enabled=False)
        try:
            _drive(node, objects=((1.1, 0.05, 0.12),))
            assert _last(node.speed_cap_pub).data == pytest.approx(-1.0)
        finally:
            node.destroy_node()


class TestTheUpgradeOnTheWire:

    def test_a_detection_over_the_unknown_keeps_its_id_and_takes_the_class(self):
        node = _node(unknown_objects_mode='active')
        try:
            _drive(node)
            (unknown,) = [t for t in node._objects if t.confirmed]
            det_msg = _stamp(Detection3DArray(), 11.05)
            det_msg.header.frame_id = 'base_link'
            det = Detection3D()
            det.bbox.center.position.x = 1.5
            det.bbox.size.x = 0.3
            hyp = ObjectHypothesisWithPose()
            hyp.hypothesis.class_id = 'person'
            hyp.hypothesis.score = 0.9
            det.results.append(hyp)
            det_msg.detections.append(det)
            node._detections_cb(det_msg)
            assert [(t.track_id, t.class_id) for t in node._objects] == [
                (unknown.track_id, 'person')]
            assert node._n_upgrades == 1 and node._n_dedupe == 0
            # The person's own LiDAR returns are now freed, not a new unknown.
            _drive(node, cycles=10, t0=11.1)
            assert [t.class_id for t in node._objects] == ['person']
        finally:
            node.destroy_node()


class TestPureMpcHelpers:

    @staticmethod
    def _track(cls, x, y, confirmed=True, width=0.3):
        t = SemanticObject(1, cls, x, y, 0.9, 0.0, 1, width=width)
        t.confirmed = confirmed
        if cls == uo.UNKNOWN_CLASS:
            t.extent = (width, width / 2)
        return t

    def test_the_class_margin_and_the_frame(self):
        tracks = [self._track('person', 2.0, 1.0), self._track(uo.UNKNOWN_CLASS, 1.0, 0.0)]
        out = uo.obstacles_for_mpc(tracks, (1.0, 1.0, math.pi / 2), 'active',
                                   {'person': 0.3})
        assert out[0] == pytest.approx((0.0, -1.0, 0.15 + 0.3))
        assert out[1] == pytest.approx((-1.0, 0.0, 0.15))

    def test_tentative_tracks_never_reach_the_mpc(self):
        tracks = [self._track(uo.UNKNOWN_CLASS, 1.0, 0.0, confirmed=False)]
        assert uo.obstacles_for_mpc(tracks, (0.0, 0.0, 0.0), 'active') == []
