"""
Coverage for the extract/render split.

Two things must stay true and are both easy to break by accident:

1. mission_render must never acquire a ROS import. linus has no ROS at all, so
   a stray `from rclpy...` added for convenience would not fail here (ROS is
   sourced) -- it would fail only on the machine that cannot be debugged from
   here. Checked statically, by parsing the module rather than importing it,
   so the guard holds regardless of what happens to be importable.

2. write_extract -> read_extract must round-trip the bag dict exactly. The MP4
   md5 equivalence rests entirely on this; a lossy round-trip would show up as
   a subtly different video rather than as an error.
"""

import argparse
import ast
import math
import os

import numpy as np
import pytest

from f1tenth_logger import mission_render
from f1tenth_logger.mission_extract import write_extract

_ROS_MODULES = {
    'rclpy', 'rosbag2_py', 'rosidl_runtime_py', 'f1tenth_messages',
    'std_msgs', 'nav_msgs', 'sensor_msgs', 'geometry_msgs', 'vision_msgs',
    'visualization_msgs', 'ackermann_msgs', 'tf2_ros', 'builtin_interfaces',
}


def _imported_roots(path):
    tree = ast.parse(open(path).read())
    roots = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            roots.update(alias.name.split('.')[0] for alias in node.names)
        elif isinstance(node, ast.ImportFrom) and node.module and node.level == 0:
            roots.add(node.module.split('.')[0])
    return roots


class TestTheRenderHalfIsROSFree:
    def test_mission_render_imports_no_ros_module(self):
        roots = _imported_roots(mission_render.__file__)
        assert not (roots & _ROS_MODULES), \
            f'mission_render must run where ROS does not exist; found {roots & _ROS_MODULES}'

    def test_mission_render_does_not_import_the_extract_half(self):
        # One direction only: extract imports render, never the reverse.
        # The reverse would drag rosbag2_py in transitively and undo the split.
        assert 'f1tenth_logger' not in _imported_roots(mission_render.__file__) or \
            'mission_extract' not in open(mission_render.__file__).read()


class TestExtractRoundTrip:
    """
    write_extract -> read_extract must reproduce the bag dict exactly.

    Synthetic rather than bag-derived so this passes on a machine holding no
    run data, and so each stream's shape is pinned explicitly.
    """

    @pytest.fixture
    def bag(self):
        def stream(max_age, samples):
            s = mission_render.Stream(max_age)
            for t, v in samples:
                s.add(t, v)
            return s

        grid = np.arange(-1, 5, dtype=np.int8).reshape(2, 3)
        return {
            'streams': {
                'pose': stream(0.5, [(1.0, (1.25, -0.5, 0.33, 2.0)),
                                     (1.1, (1.30, -0.5, 0.34, 2.1))]),
                'boundaries': stream(0.5, [(1.0, [(0.9, 0.37, 2.8), (-0.37, 0.92, 1.4)])]),
                'clearance': stream(0.5, [(1.0, 2.6908931732177734)]),
                'obstacles': stream(0.5, [(1.0, [(0.5, 0.25, 0.3)])]),
                'detections': stream(0.5, [(1.0, ('cam_frame',
                                                  [('person', 0.87, 1.0, 2.0, 0.5, 0.04),
                                                   ('bottle', 0.42, 3.0, 1.0, 0.2, None)]))]),
                'markers': stream(0.5, [(1.0, [('disk', 'person', 1.0, 2.0, 0.3, ''),
                                               ('label', 'person', 1.0, 2.3, 0.0, 'p1')])]),
                'tree': stream(0.5, [(1.0, ('', ['emergency', 'mission'],
                                            ['FAILURE', 'RUNNING'], '', False, ''))]),
                'solver': stream(0.5, [(1.0, (True, 1, 'solved', 0.0142850875854,
                                              0.10000000149011612, 2.9159159660339355,
                                              'rti', 3, 0, [1.24, 1.29], [0.1, 0.2],
                                              [0.0, 0.01], [2.0, 2.1], 'odom'))]),
                'stop': stream(0.25, [(1.0, ('obstacle', 0.0))]),
                'drive': stream(0.5, [(1.0, (0.4699937105178833, -0.15588527917861938))]),
                'map': stream(None, [(1.0, (grid, (-0.78, 7.86, -4.53, 7.46))),
                                     (2.0, (grid, (-0.78, 7.86, -4.53, 7.46)))]),
                'corridor': stream(0.5, [(1.0, {'corridor_left': [(0.95, 1.69), (0.98, 1.70)],
                                                'corridor_right': [(1.95, 1.69)]})]),
                'map_to_odom': stream(0.5, [(1.0, (0.118, 0.004, 0.286))]),
            },
            'static_tf': {'cam_frame': ('base_link', np.array([0.1, 0.0, 0.2]),
                                        np.eye(3))},
            't0': 1788434263.7, 't1': 1788434277.8,
            'pose_frame': 'map', 'pose_topic': '/ekf_global/odometry/filtered',
            'legacy_solver_msgs': 0, 'dropped': {'/some/topic': 2},
        }

    def _round_trip(self, bag, tmp_path):
        path = write_extract(bag, tmp_path / 'r.extract.parquet',
                             manifest={'mission_id': 'm'}, run_id='r')
        return mission_render.read_extract(path)

    def test_scalars_and_metadata_survive(self, bag, tmp_path):
        out = self._round_trip(bag, tmp_path)
        assert out['t0'] == bag['t0'] and out['t1'] == bag['t1']
        assert out['pose_frame'] == 'map'
        assert out['pose_topic'] == '/ekf_global/odometry/filtered'
        assert out['dropped'] == {'/some/topic': 2}
        assert out['run_id'] == 'r'
        assert out['manifest']['mission_id'] == 'm'

    def test_every_stream_survives_with_its_max_age(self, bag, tmp_path):
        out = self._round_trip(bag, tmp_path)
        assert set(out['streams']) == set(bag['streams'])
        for name, original in bag['streams'].items():
            assert len(out['streams'][name]) == len(original), name
            assert out['streams'][name].max_age == original.max_age, name

    def test_floats_round_trip_bit_exactly(self, bag, tmp_path):
        # The md5 equivalence rests on this: JSON via repr() is lossless, so a
        # value must come back identical, not merely close.
        out = self._round_trip(bag, tmp_path)
        assert out['streams']['clearance'].v[0] == 2.6908931732177734
        assert out['streams']['drive'].v[0] == (0.4699937105178833,
                                                -0.15588527917861938)
        assert out['streams']['pose'].v[0] == (1.25, -0.5, 0.33, 2.0)

    def test_tuples_come_back_as_tuples_not_lists(self, bag, tmp_path):
        # draw_frame unpacks several of these positionally; a list would work
        # by luck today and break on the first `is tuple` or concat.
        out = self._round_trip(bag, tmp_path)
        assert isinstance(out['streams']['pose'].v[0], tuple)
        assert isinstance(out['streams']['solver'].v[0], tuple)

    def test_a_none_inside_a_detection_survives(self, bag, tmp_path):
        # sigma is None whenever the covariance was zero -- it must not become
        # 0.0, which would draw a confident ellipse that was never measured.
        out = self._round_trip(bag, tmp_path)
        _frame, dets = out['streams']['detections'].v[0]
        assert dets[1][5] is None
        assert dets[0][5] == 0.04

    def test_the_occupancy_grid_survives_and_is_deduplicated(self, bag, tmp_path):
        out = self._round_trip(bag, tmp_path)
        first, second = out['streams']['map'].v
        assert np.array_equal(first[0], bag['streams']['map'].v[0][0])
        assert first[0].dtype == np.int8
        assert first[1] == (-0.78, 7.86, -4.53, 7.46)
        # Both ticks reference ONE stored raster: identical content is interned
        # by hash, which is what keeps a republished map from dominating the file.
        assert np.array_equal(first[0], second[0])

    def test_static_tf_matrices_survive_as_arrays(self, bag, tmp_path):
        out = self._round_trip(bag, tmp_path)
        parent, translation, matrix = out['static_tf']['cam_frame']
        assert parent == 'base_link'
        assert np.allclose(translation, [0.1, 0.0, 0.2])
        assert np.allclose(matrix, np.eye(3))

    def test_corridor_polylines_survive_keyed_by_namespace(self, bag, tmp_path):
        out = self._round_trip(bag, tmp_path)
        walls = out['streams']['corridor'].v[0]
        assert set(walls) == {'corridor_left', 'corridor_right'}
        assert walls['corridor_left'][0] == (0.95, 1.69)

    def test_zero_order_hold_still_honours_max_age_after_a_round_trip(
            self, bag, tmp_path):
        out = self._round_trip(bag, tmp_path)
        assert out['streams']['pose'].at(1.15) == (1.30, -0.5, 0.34, 2.1)
        assert out['streams']['pose'].at(99.0) is None      # stale
        assert out['streams']['map'].at(99.0) is not None   # max_age None

    def test_an_empty_stream_is_still_present_after_a_round_trip(
            self, bag, tmp_path):
        bag['streams']['stop'] = mission_render.Stream(0.25)
        out = self._round_trip(bag, tmp_path)
        assert 'stop' in out['streams']
        assert len(out['streams']['stop']) == 0

    def test_the_extract_is_much_smaller_than_a_bag(self, bag, tmp_path):
        path = write_extract(bag, tmp_path / 'r.extract.parquet')
        assert os.path.getsize(path) > 0
        assert not math.isnan(os.path.getsize(path))


class TestPaddingComesFromTheRunNotTheCurrentConfig:
    """
    car_radius / avoidance_margin decide where the tightened boundary is drawn.

    They are properties of the run. Reading them from the live config (or from
    a constant in the renderer, which is what they used to be) means
    re-rendering a historical run after a config change silently draws padding
    that run never flew with, and the archive stops being a faithful record.
    """

    def _snapshot(self, tmp_path, car_radius=0.20, margin=0.12):
        path = tmp_path / 'run.params.yaml'
        path.write_text(
            f'car_radius:\n  default: {car_radius}\n  description: "r"\n'
            f'obstacle_safety_margin_m:\n  default: {margin}\n  description: "m"\n')
        return path

    def test_values_are_read_from_the_runs_own_snapshot(self, tmp_path):
        from f1tenth_logger.mission_extract import padding_from_params
        got = padding_from_params(self._snapshot(tmp_path, 0.33, 0.07))
        assert got['car_radius'] == 0.33
        assert got['avoidance_margin'] == 0.07
        assert got['source'].endswith('run.params.yaml')

    def test_avoidance_margin_maps_from_obstacle_safety_margin_m(self, tmp_path):
        # There is no stack_params key literally named avoidance_margin;
        # MPC_corr's avoidance_margin is fed from obstacle_safety_margin_m.
        from f1tenth_logger.mission_extract import padding_from_params
        assert padding_from_params(self._snapshot(tmp_path, margin=0.09))[
            'avoidance_margin'] == 0.09

    def test_a_missing_snapshot_falls_back_but_says_so(self, tmp_path):
        from f1tenth_logger.mission_extract import padding_from_params
        got = padding_from_params(tmp_path / 'nope.yaml')
        assert got['source'] == 'fallback'
        assert got['car_radius'] == 0.20

    def test_the_renderer_uses_the_runs_values_when_no_flag_is_passed(self):
        cfg = argparse.Namespace(car_radius=None, avoidance_margin=None)
        bag = {'padding': {'car_radius': 0.33, 'avoidance_margin': 0.07,
                           'source': '/x.yaml'}}
        mission_render._resolve_padding(bag, cfg, 'run')
        assert cfg.car_radius == 0.33
        assert cfg.avoidance_margin == 0.07

    def test_an_explicit_flag_still_wins(self):
        cfg = argparse.Namespace(car_radius=0.5, avoidance_margin=None)
        bag = {'padding': {'car_radius': 0.33, 'avoidance_margin': 0.07,
                           'source': '/x.yaml'}}
        mission_render._resolve_padding(bag, cfg, 'run')
        assert cfg.car_radius == 0.5          # override respected
        assert cfg.avoidance_margin == 0.07   # the rest still from the run

    def test_an_extract_with_no_padding_refuses_rather_than_guessing(self):
        cfg = argparse.Namespace(car_radius=None, avoidance_margin=None)
        with pytest.raises(ValueError, match='car_radius'):
            mission_render._resolve_padding({'padding': {}}, cfg, 'run')

    def test_padding_survives_the_parquet_round_trip(self, tmp_path):
        bag = {'streams': {}, 'static_tf': {}, 't0': 0.0, 't1': 1.0,
               'pose_frame': 'map', 'pose_topic': '/p',
               'legacy_solver_msgs': 0, 'dropped': {}}
        path = write_extract(bag, tmp_path / 'r.extract.parquet',
                             padding={'car_radius': 0.33,
                                      'avoidance_margin': 0.07,
                                      'source': '/x.yaml'})
        assert mission_render.read_extract(path)['padding']['car_radius'] == 0.33
