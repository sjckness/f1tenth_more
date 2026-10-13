"""
perception_debug.py tests: the pure half of the perception debug video.

No rclpy, no message packages, no ROS: numpy and cv2 only, so this file runs
in a plain venv as well as under colcon test.
"""

import json

from f1tenth_perception import perception_debug as pd
import numpy as np
import pytest


def _wall_payload(**over):
    kwargs = {
        'stamp_sec': 100.25, 'img_hw': (360, 640), 'roi': (230, 145, 410, 215),
        'counts': {'roi': 12600, 'housing': 0, 'too_close': 0, 'invalid': 400,
                   'object': 2300, 'bg': 9900, 'min_bg': 60},
        'stereo': {'raw_m': 2.31, 'front_distance_m': 2.35, 'front_wall': True},
        'clearance': {'obstacle_m': 1.02, 'corridor_half_width_m': 0.35,
                      'raw_m': 1.02, 'front_clearance_m': 1.05, 'front_blocked': False},
        'objects_in_roi': [('person', 0.91, 2300)], 'object_source': 'mask',
        'det_stamp_sec': 99.875,
        'lidar': pd.lidar_fit_summary(True, 0, 2.65, 60, 75, 100.2, 100.25),
        'too_close': False}
    kwargs.update(over)
    return pd.build_wall_debug(**kwargs)


class TestDebugSize:

    def test_480_wide_keeps_the_zed_640x360_aspect(self):
        assert pd.debug_size(640, 360, 480) == (480, 270)

    def test_never_upscales(self):
        assert pd.debug_size(640, 360, 1280) == (640, 360)

    def test_zero_width_means_source_size(self):
        assert pd.debug_size(640, 360, 0) == (640, 360)


class TestTickDue:

    @staticmethod
    def _ticks(stamps, period):
        due_at, next_due = [], None
        for stamp in stamps:
            due, next_due = pd.tick_due(stamp, next_due, period)
            if due:
                due_at.append(stamp)
        return due_at

    def test_first_frame_is_a_tick(self):
        assert pd.tick_due(10.0, None, 0.2)[0]

    def test_15hz_input_at_5hz_ticks_every_third_frame(self):
        stamps = [i / 15.0 for i in range(150)]
        assert len(self._ticks(stamps, 0.2)) == 50

    def test_15hz_input_at_10hz_averages_10hz_not_7_5(self):
        stamps = [i / 15.0 for i in range(150)]
        assert len(self._ticks(stamps, 0.1)) == pytest.approx(100, abs=1)

    def test_a_gap_restarts_the_schedule_instead_of_bursting(self):
        stamps = [0.0, 0.2, 5.0, 5.0667, 5.1333, 5.2]
        assert self._ticks(stamps, 0.2) == [0.0, 0.2, 5.0, 5.2]

    def test_time_going_backwards_restarts_instead_of_stalling(self):
        assert self._ticks([100.0, 100.2, 50.0, 50.0667, 50.2], 0.2) == \
            [100.0, 100.2, 50.0, 50.2]

    def test_zero_period_never_ticks(self):
        assert not pd.tick_due(1.0, None, 0.0)[0]


class TestMatchByClassScore:

    def test_exact_key_pairs_and_unmatched_is_none(self):
        out = pd.match_by_class_score(
            [('person', 0.91), ('tv', 0.42), ('chair', 0.33)],
            [('tv', 0.42, 'B'), ('person', 0.91, 'A')])
        assert out == ['A', 'B', None]

    def test_same_class_different_score_does_not_pair(self):
        assert pd.match_by_class_score([('person', 0.91)], [('person', 0.9100001, 'X')]) == [None]

    def test_duplicate_keys_pair_in_arrival_order_each_used_once(self):
        out = pd.match_by_class_score(
            [('cup', 0.5), ('cup', 0.5)], [('cup', 0.5, 'first'), ('cup', 0.5, 'second')])
        assert out == ['first', 'second']


class TestDistanceFromFront:

    def test_camera_origin_is_moved_to_the_front(self):
        # Made-up geometry: lens 0.1 m, bumper 0.5 m ahead of base_link.
        assert pd.distance_from_front(2.0, 0.1, 0.5) == pytest.approx(1.6)

    def test_a_point_behind_the_bumper_stays_negative(self):
        assert pd.distance_from_front(0.2, 0.1, 0.5) == pytest.approx(-0.2)

    def test_missing_input_is_none(self):
        assert pd.distance_from_front(None, 0.1, 0.4) is None
        assert pd.distance_from_front(2.0, None, 0.4) is None


class TestWallPayload:

    def test_round_trips_through_json(self):
        text = pd.to_json(_wall_payload())
        back = pd.parse_wall_debug(text)
        assert back['roi'] == [230, 145, 410, 215]
        assert back['px']['bg'] == 9900
        assert back['objects_in_roi'] == [['person', 0.91, 2300]]
        assert back['det_age_s'] == pytest.approx(0.375)
        assert back['lidar']['origin'] == 'base_link'

    def test_names_the_source_the_wall_guard_uses_and_no_fused_value(self):
        payload = _wall_payload()
        assert payload['source'] == 'stereo'
        assert payload['fused_m'] is None

    def test_default_origins_are_front_except_the_raw_lens_value(self):
        payload = _wall_payload(front_offset_m=0.333)
        assert payload['stereo_origin'] == 'front'
        assert payload['obstacle_origin'] == 'front'
        assert payload['clearance_origin'] == 'front'
        assert payload['stereo_raw_origin'] == 'camera'
        assert payload['front_offset_m'] == pytest.approx(0.333)

    def test_clearance_is_mixed_while_stereo_and_obstacles_differ_in_origin(self):
        payload = _wall_payload(stereo_origin=pd.ORIGIN_CAMERA,
                                obstacle_origin=pd.ORIGIN_BASE_LINK)
        assert payload['clearance_origin'] == 'mixed'

    def test_carries_the_depth_frame_id(self):
        assert _wall_payload(frame_id='zed2_left_camera_optical_frame')['frame_id'] == \
            'zed2_left_camera_optical_frame'

    def test_nan_and_inf_become_null_never_invalid_json(self):
        payload = _wall_payload(
            stereo={'raw_m': float('nan'), 'front_distance_m': -1.0, 'front_wall': False},
            lidar=pd.lidar_fit_summary(False, 2, float('nan'), 0, 3, 1.0, 1.1))
        text = pd.to_json(payload)
        assert 'NaN' not in text and 'Infinity' not in text
        assert json.loads(text)['stereo_raw_m'] is None
        assert json.loads(text)['lidar']['distance_m'] is None

    def test_unknown_version_and_garbage_are_rejected(self):
        payload = _wall_payload()
        payload['v'] = pd.WALL_DEBUG_VERSION + 1
        assert pd.parse_wall_debug(pd.to_json(payload)) is None
        assert pd.parse_wall_debug('not json') is None
        assert pd.parse_wall_debug(None) is None


class TestBoxOverlap:

    def test_inside_partial_and_outside(self):
        roi = (10, 10, 20, 20)
        assert pd.box_overlap_px((0, 0, 100, 100), roi) == 100
        assert pd.box_overlap_px((15, 15, 30, 30), roi) == 25
        assert pd.box_overlap_px((30, 30, 40, 40), roi) == 0


class TestDrawAndEncode:

    @staticmethod
    def _frame(**over):
        frame = {
            'scale': 0.75,
            'hi': [{'box': (100, 50, 200, 300), 'class_id': 'person', 'score': 0.91,
                    'color': (0, 0, 255), 'track': '12', 'd_front_m': 1.0,
                    'contours': None}],
            'lo': [{'box': (400, 100, 450, 160), 'class_id': 'tv', 'score': 0.12}],
            'threshold': 0.3, 'min_confidence': 0.1,
            'wall': pd.parse_wall_debug(pd.to_json(_wall_payload())),
            'wall_front': {'used_m': 2.0, 'lidar_m': 2.2, 'obstacle_m': 0.6},
            'mission': 'RUNNING test', 'stamp_text': '09:33:05.123Z', 'lag_s': 0.45,
            'front_label': '@front'}
        frame.update(over)
        return frame

    def test_text_strip_is_appended_below_not_drawn_over_the_image(self):
        img = np.zeros((270, 480, 3), dtype=np.uint8)
        out = pd.draw_debug_frame(img, self._frame())
        assert out.shape[1] == 480 and out.shape[0] > 270
        assert out.shape[0] - 270 == pd.STRIP_PAD_PX * 2 + pd.LINE_HEIGHT_PX * 5

    def test_strip_reports_threshold_floor_wall_source_clearance_and_roi_object(self):
        lines = [t for t, _c in pd.strip_lines(self._frame())]
        assert 'conf>0.30 grey>=0.10' in lines[0]
        assert lines[1].startswith('WALL[stereo] 2.00@front raw 2.31@camera')
        assert lines[2].startswith('LIDAR 2.20@front ok') and lines[2].endswith('fused --')
        assert lines[3].startswith('CLEAR 1.05@front free obj 0.60@front')
        assert lines[-1].startswith('ROI OBJ person 0.91')

    def test_unknown_bumper_is_labelled_and_shows_no_distance_from_it(self):
        lines = [t for t, _c in pd.strip_lines(self._frame(
            wall_front={}, front_label='@front?'))]
        assert lines[0].endswith('d@front?')
        assert lines[1].startswith('WALL[stereo] --@front? raw 2.31@camera')

    def test_roi_is_drawn_red_when_an_object_overlaps_it(self):
        img = np.zeros((270, 480, 3), dtype=np.uint8)
        out = pd.draw_debug_frame(img, self._frame(hi=[], lo=[]))
        # ROI (230, 145) at 640 wide -> (172, 108) at 480 wide.
        assert tuple(out[120, 172]) == pd.COLOR_ROI_HIT

    def test_roi_is_amber_when_nothing_overlaps_it(self):
        payload = _wall_payload(objects_in_roi=[])
        payload['px']['object'] = 0
        img = np.zeros((270, 480, 3), dtype=np.uint8)
        out = pd.draw_debug_frame(img, self._frame(hi=[], lo=[], wall=payload))
        assert tuple(out[120, 172]) == pd.COLOR_ROI

    def test_label_carries_track_distance_and_corridor_flag(self, monkeypatch):
        drawn = []
        monkeypatch.setattr(pd, '_text', lambda img, text, *a, **k: drawn.append(text))
        det = dict(self._frame()['hi'][0], in_corridor=True)
        pd.draw_debug_frame(np.zeros((270, 480, 3), dtype=np.uint8),
                            self._frame(hi=[det], lo=[]))
        assert 'person 0.91 T12 1.00m C' in drawn

    def test_below_threshold_box_is_drawn_grey(self):
        img = np.zeros((270, 480, 3), dtype=np.uint8)
        out = pd.draw_debug_frame(img, self._frame(hi=[], wall=None))
        x0, y0 = int(400 * 0.75), int(100 * 0.75)
        assert tuple(out[y0 + 5, x0]) == pd.COLOR_LOW

    def test_missing_wall_payload_still_draws(self):
        img = np.zeros((270, 480, 3), dtype=np.uint8)
        out = pd.draw_debug_frame(img, self._frame(wall=None))
        assert out.shape[0] > 270

    def test_jpeg_is_a_decodable_jpeg(self):
        import cv2
        img = np.random.default_rng(0).integers(0, 255, (300, 480, 3), dtype=np.uint8)
        data = pd.encode_jpeg(img, 50)
        assert data[:2] == b'\xff\xd8'
        assert cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_COLOR).shape == img.shape

    def test_mask_contour_follows_the_mask(self):
        mask = np.zeros((360, 640), dtype=bool)
        mask[100:200, 100:300] = True
        contours = pd.mask_contours(mask, (480, 270))
        assert len(contours) == 1
        xs, ys = contours[0][:, 0, 0], contours[0][:, 0, 1]
        assert xs.min() == 75 and ys.min() == 75
