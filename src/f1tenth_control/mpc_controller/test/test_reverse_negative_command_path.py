"""
A negative speed survives every hop it has to survive, with allow_reverse on.

Why this file exists: the bag check on the newest run in ~/bags
(corridor_obstacles_20261019T135307, and the three before it) found no
negative speed ANYWHERE along /drive -> /ackermann_drive ->
/commands/motor/speed -> /sensors/core, because max_reverse_speed_mps was
0.0 and no reverse could be commanded at all. Reading the chain instead
showed it is sign-transparent end to end (ackermann_mux republishes the
message unchanged, correctVelocity is sign-symmetric on the magnitude,
vesc_driver's erpm_deadband touches only the published TELEMETRY, and its
command clip is speed_min -23250). The one thing that did block a reverse
was higher up and invisible to those topics: /mpc/reverse_status was never
published on the allow_reverse path, so the BT's ReverseModeMonitor sat at
"unknown", which reverse_integration.py D1 treats as forward -- the front
cone then zeroes the command on the safety lane (mux priority 200) exactly
when the car has something in front of it, which is when a reverse is
wanted.

So the coverage here is: the status the BT reads (the fixed node), and the
two numeric hops either side of it, pinned against a regression to a
magnitude-only limit.

Run standalone: python3 -m pytest test/test_reverse_negative_command_path.py -v
"""

import math

from builtin_interfaces.msg import Time

from f1tenth_params.param_defaults import get_value

from mpc_controller.MPC_corr import MPCController
from mpc_controller.drive_limits import clamp_drive_speed, validate_speed_limits
from mpc_controller.reverse_manoeuvre import (
    MODE_FORWARD, MODE_REVERSE, MODE_STOPPING, MODE_STUCK)
from mpc_controller.reverse_mpc import FORWARD, REVERSE

import pytest


class _FakeClock:
    """header.stamp is a real builtin_interfaces Time: the setter type-checks it."""

    def now(self):
        class _Now:
            def to_msg(self):
                return Time(sec=0, nanosec=0)
        return _Now()


class _FakePub:
    def __init__(self):
        self.sent = []

    def publish(self, msg):
        self.sent.append(msg)


class _FakeSelector:
    def __init__(self, direction):
        self.direction = direction


class _FakeCfg:
    stop_speed_mps = 0.05


class _FakePlanner:
    def __init__(self, direction):
        self.selector = _FakeSelector(direction)
        self.cfg = _FakeCfg()


class _FakeMPC:
    """Just enough instance state for _publish_reverse_status_from_planner."""

    def __init__(self, direction, v):
        self.reverse_status_pub = _FakePub()
        self.reverse_planner = _FakePlanner(direction)
        self.v = v
        # DELIBERATELY ABSENT: _bt_allow, _bt_allow_t and everything else
        # _init_reverse_manoeuvre sets only AFTER its allow_reverse early
        # return. The first version of this stand-in defined _bt_allow, so the
        # test passed while the real node died with AttributeError on its first
        # tick. The fake must have exactly the state this path really has.

    def get_clock(self):
        return _FakeClock()


def _status(direction, v):
    mpc = _FakeMPC(direction, v)
    MPCController._publish_reverse_status_from_planner(mpc)
    assert len(mpc.reverse_status_pub.sent) == 1
    return mpc.reverse_status_pub.sent[0]


class TestTheStatusTheBtReads:
    """Without these fields the safety lane zeroes the reverse (D1)."""

    def test_it_touches_no_attribute_the_allow_reverse_path_lacks(self):
        """The publish must not reach for manoeuvre-only state (AttributeError)."""
        msg = _status(REVERSE, v=0.0)
        assert msg.bt_allow_reverse is False
        assert msg.bt_allow_fresh is False

    def test_a_selected_reverse_at_rest_reports_reverse_and_active(self):
        msg = _status(REVERSE, v=0.0)
        assert msg.mode == MODE_REVERSE
        assert msg.reverse_active is True

    def test_a_selected_reverse_while_still_rolling_forward_reports_stopping(self):
        """
        Still moving the old way, so "stopping" toward the reverse.

        D1 must already suppress the front stops here: the car is braking for
        the direction change with the obstacle still in front of it.
        """
        msg = _status(REVERSE, v=0.30)
        assert msg.mode == MODE_STOPPING
        assert msg.reverse_active is True

    def test_forward_reports_forward_and_not_active(self):
        msg = _status(FORWARD, v=0.45)
        assert msg.mode == MODE_FORWARD
        assert msg.reverse_active is False

    def test_this_path_never_reports_stuck(self):
        """
        The mode "stuck" belongs to the manoeuvre alone.

        It is that mechanism's attempt counter, and D3 turns it into the
        stuck_after_reverse outcome. allow_reverse has no attempt counter and
        must never trip it.
        """
        for direction in (FORWARD, REVERSE):
            for v in (-0.4, 0.0, 0.5):
                assert _status(direction, v).mode != MODE_STUCK

    def test_the_manoeuvre_only_fields_are_neutral(self):
        msg = _status(REVERSE, v=0.0)
        assert msg.stuck is False
        assert msg.attempts == 0
        assert msg.rear_ready is False
        assert math.isnan(msg.rear_solve_ms)
        assert math.isnan(msg.rear_clearance_m)


class TestTheDriveClampKeepsTheSign:
    """The /drive clamp is a two-sided box, not a magnitude limit."""

    def test_the_commanded_reverse_speed_passes_unclamped(self):
        rev = float(get_value('max_reverse_speed_mps'))
        v, clamped = clamp_drive_speed(-rev, 0.5, rev)
        assert v == pytest.approx(-rev)
        assert clamped is False

    def test_a_deeper_reverse_clamps_to_the_limit_not_to_zero(self):
        rev = float(get_value('max_reverse_speed_mps'))
        v, clamped = clamp_drive_speed(-10.0, 0.5, rev)
        assert v == pytest.approx(-rev)
        assert clamped is True

    def test_the_limit_is_a_magnitude_so_a_negative_param_is_refused(self):
        with pytest.raises(ValueError):
            validate_speed_limits(0.5, -0.4)


def _vesc_params():
    """Read vesc.yaml's own values, not a copy of them (they are live calibration)."""
    import pathlib
    import yaml
    here = pathlib.Path(__file__).resolve()
    root = next(p for p in here.parents if (p / 'src' / 'f1tenth_bringup').is_dir())
    cfg = yaml.safe_load(
        (root / 'src/f1tenth_bringup/config/vesc.yaml').read_text(encoding='utf-8'))
    for node in cfg.values():
        pr = node.get('ros__parameters', {}) if isinstance(node, dict) else {}
        if 'speed_to_erpm_gain' in pr:
            return pr
    raise AssertionError('speed_to_erpm_gain not found in vesc.yaml')


class TestTheErpmHopKeepsTheSign:
    """ackermann_to_vesc: erpm = speed_to_erpm_gain * speed + offset."""

    def test_a_reverse_command_becomes_negative_erpm_inside_the_driver_limits(self):
        pr = _vesc_params()
        gain = float(pr['speed_to_erpm_gain'])
        offset = float(pr.get('speed_to_erpm_offset', 0.0))
        rev = float(get_value('max_reverse_speed_mps'))
        erpm = gain * -rev + offset
        assert erpm < 0.0, 'the conversion must carry the sign, not the magnitude'
        assert erpm >= float(pr['speed_min']), 'vesc_driver would clip it'

    def test_the_reverse_command_clears_the_telemetry_deadband(self):
        """
        A reverse must come back visible in the telemetry.

        erpm_deadband only zeroes the PUBLISHED state, but a reverse under it
        would read back as a standstill and look like the command never landed.
        """
        pr = _vesc_params()
        rev = float(get_value('max_reverse_speed_mps'))
        erpm = abs(float(pr['speed_to_erpm_gain']) * -rev)
        assert erpm > float(pr['erpm_deadband'])
