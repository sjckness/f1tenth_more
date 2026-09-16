"""The /drive speed clamp: drive_limits.py and MPC_corr._publish_drive.

Pure function tests, one duck-typed node test of the choke point itself, and
the coupling test the defaults were chosen by: no speed any mission, the LLM
translator or mpc_corr requests is changed by them.

Run standalone: python3 -m pytest test/test_drive_limits.py -v
"""

import json
import math
from pathlib import Path
from types import SimpleNamespace

import pytest

from f1tenth_params.param_defaults import get_value
from mpc_controller.MPC_corr import MPCController
from mpc_controller.drive_limits import clamp_drive_speed, validate_speed_limits

REPO_SRC = Path(__file__).resolve().parents[3]
MISSIONS = REPO_SRC / 'f1tenth_behavior' / 'missions'


class TestClamp:

    @pytest.mark.parametrize('speed, expected, clamped', [
        (0.4, 0.4, False),
        (1.0, 1.0, False),
        (1.2, 1.0, True),
        (2.07, 1.0, True),
        (0.0, 0.0, False),
        (-0.01, 0.0, True),
        (-1.04, 0.0, True),
    ])
    def test_default_limits(self, speed, expected, clamped):
        assert clamp_drive_speed(speed, 1.0, 0.0) == (pytest.approx(expected), clamped)

    def test_a_reverse_allowance_is_a_magnitude(self):
        assert clamp_drive_speed(-0.2, 1.0, 0.3) == (pytest.approx(-0.2), False)
        assert clamp_drive_speed(-0.5, 1.0, 0.3) == (pytest.approx(-0.3), True)

    @pytest.mark.parametrize('bad', [math.nan, math.inf, -math.inf])
    def test_a_non_finite_request_is_a_stop(self, bad):
        assert clamp_drive_speed(bad, 1.0, 0.0) == (0.0, True)

    @pytest.mark.parametrize('fwd, rev', [(0.0, 0.0), (-1.0, 0.0), (1.0, -0.1), (math.nan, 0.0)])
    def test_invalid_limits_are_rejected(self, fwd, rev):
        with pytest.raises(ValueError):
            validate_speed_limits(fwd, rev)


class _Pub:
    def __init__(self):
        self.msgs = []

    def publish(self, msg):
        self.msgs.append(msg)

    def get_subscription_count(self):
        return 1


def _node(fwd=1.0, rev=0.0):
    from builtin_interfaces.msg import Time
    logger = SimpleNamespace(info=lambda *a, **k: None, warn=lambda *a, **k: None)
    return SimpleNamespace(
        max_forward_speed=fwd, max_reverse_speed=rev, pub=_Pub(), drive_clamp_pub=_Pub(),
        get_logger=lambda: logger,
        get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(to_msg=lambda: Time())))


class TestPublishDriveIsTheChokePoint:

    def test_an_overspeed_command_goes_out_clamped_with_an_event(self):
        node = _node()
        MPCController._publish_drive(node, 2.07, 0.1)
        (drive,) = node.pub.msgs
        assert drive.drive.speed == pytest.approx(1.0)
        assert drive.drive.steering_angle == pytest.approx(0.1)
        (event,) = node.drive_clamp_pub.msgs
        assert event.requested_speed == pytest.approx(2.07)
        assert event.applied_speed == pytest.approx(1.0)
        assert (event.max_forward_speed, event.max_reverse_speed) == (1.0, 0.0)

    def test_reverse_goes_out_as_zero(self):
        node = _node()
        MPCController._publish_drive(node, -0.23, -0.2)
        assert node.pub.msgs[0].drive.speed == 0.0
        assert len(node.drive_clamp_pub.msgs) == 1

    def test_an_in_range_command_is_untouched_and_silent(self):
        node = _node()
        MPCController._publish_drive(node, 0.4, 0.0)
        assert node.pub.msgs[0].drive.speed == pytest.approx(0.4)
        assert node.drive_clamp_pub.msgs == []

    def test_the_stop_paths_still_publish_zero(self):
        node = _node()
        MPCController._publish_drive(node, 0.0, 0.0)
        assert node.pub.msgs[0].drive.speed == 0.0
        assert node.drive_clamp_pub.msgs == []


class TestDefaultsChangeNoRequestedSpeed:
    """How the defaults were chosen, pinned: the clamp must never bind on a request."""

    def _requested_speeds(self):
        speeds = []
        for path in sorted(MISSIONS.glob('*.json')):
            raw = json.loads(path.read_text())
            for move in raw['moves']:
                for key in ('drive', 'turn', 'go_to_object'):
                    if isinstance(move.get(key), dict) and 'speed' in move[key]:
                        speeds.append((path.name, key, float(move[key]['speed'])))
        for key in ('mission_translator_speed_straight', 'mission_translator_speed_turn',
                    'mission_translator_speed_go_to'):
            speeds.append(('stack_params', key, float(get_value(key))))
        return speeds

    def test_every_requested_speed_is_inside_the_limits(self):
        fwd = float(get_value('max_forward_speed_mps'))
        rev = float(get_value('max_reverse_speed_mps'))
        for source, key, speed in self._requested_speeds():
            assert -rev <= speed <= fwd, (source, key, speed)

    def test_the_defaults_are_forward_one_and_no_reverse(self):
        assert float(get_value('max_forward_speed_mps')) == 1.0
        assert float(get_value('max_reverse_speed_mps')) == 0.0
