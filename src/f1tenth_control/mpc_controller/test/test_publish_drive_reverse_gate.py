"""
A negative speed leaves mpc_corr only while the reverse manoeuvre drives.

Why: raising max_reverse_speed_mps from 0.0 to 0.4 widened the /drive clamp,
and the forward solver's own braking undershoot (v + a*ts dipping below zero
near a stop) started reaching the wheels -- on 2026-10-20, 64 of 568 forward
ticks commanded down to -0.153 m/s in five windows, carrying FORWARD steering.
The clamp had been hiding that, not preventing it. Two changes close it: the
forward solver's vMin is 0.0, and _publish_drive refuses a negative from any
source other than the committed manoeuvre.

Run standalone: python3 -m pytest test/test_publish_drive_reverse_gate.py -v
"""

import math

from builtin_interfaces.msg import Time

from mpc_controller.MPC_corr import MPCController

import pytest


class _Pub:
    def __init__(self, subs=1):
        self.sent = []
        self._subs = subs

    def publish(self, msg):
        self.sent.append(msg)

    def get_subscription_count(self):
        return self._subs


class _Logger:
    def __init__(self):
        self.warns = []

    def warn(self, msg, **kw):
        self.warns.append(msg)

    def info(self, msg, **kw):
        pass


class _Clock:
    def now(self):
        class _Now:
            def to_msg(self):
                return Time(sec=0, nanosec=0)
        return _Now()


class _Supervisor:
    """Only the one field _reverse_manoeuvre_active reads."""

    def __init__(self, reverse_active):
        self._active = reverse_active

    def status(self):
        return {'reverse_active': self._active}


class _FakeMPC:
    """Just enough instance state for _publish_drive."""

    def __init__(self, supervisor=None):
        self.reverse_supervisor = supervisor
        self.reverse_suppressed = 0
        self.max_forward_speed = 0.5
        self.max_reverse_speed = 0.4
        self.drive_clamp_pub = _Pub()
        self.pub = _Pub()
        self._logger = _Logger()
        self._last_published_steer = 0.0

    def get_clock(self):
        return _Clock()

    def get_logger(self):
        return self._logger

    # The REAL predicate, not a stand-in for it: the gate and
    # /mpc/reverse_status must never disagree about what the car is doing, so
    # the test exercises the same code the node runs.
    _reverse_manoeuvre_active = MPCController._reverse_manoeuvre_active


def _publish(mpc, speed, steer=0.05):
    MPCController._publish_drive(mpc, speed, steer)
    assert len(mpc.pub.sent) == 1
    return mpc.pub.sent[0].drive.speed


class TestTheForwardPathCannotGoBackwards:
    """No reverse manoeuvre running: every negative becomes a stop."""

    def test_a_braking_undershoot_is_published_as_zero(self):
        mpc = _FakeMPC(supervisor=None)
        assert _publish(mpc, -0.153) == 0.0
        assert mpc.reverse_suppressed == 1

    def test_the_whole_observed_undershoot_range_is_suppressed(self):
        for v in (-0.010, -0.060, -0.079, -0.134, -0.153):
            mpc = _FakeMPC(supervisor=None)
            assert _publish(mpc, v) == 0.0

    def test_a_supervisor_that_is_not_reversing_does_not_excuse_it(self):
        mpc = _FakeMPC(supervisor=_Supervisor(reverse_active=False))
        assert _publish(mpc, -0.4) == 0.0
        assert mpc.reverse_suppressed == 1

    def test_the_suppression_is_counted_and_reported_on_drive_clamp(self):
        mpc = _FakeMPC(supervisor=None)
        _publish(mpc, -0.2)
        assert len(mpc.drive_clamp_pub.sent) == 1
        event = mpc.drive_clamp_pub.sent[0]
        assert event.requested_speed == pytest.approx(-0.2)
        assert event.applied_speed == 0.0
        assert any('DRIVE/no-reverse' in w for w in mpc._logger.warns)


class TestTheManoeuvreStillReverses:
    """The one path that may command a negative speed."""

    def test_the_manoeuvre_speed_passes_unchanged(self):
        mpc = _FakeMPC(supervisor=_Supervisor(reverse_active=True))
        assert _publish(mpc, -0.4) == pytest.approx(-0.4)
        assert mpc.reverse_suppressed == 0

    def test_a_deeper_request_still_clamps_to_the_limit_not_to_zero(self):
        mpc = _FakeMPC(supervisor=_Supervisor(reverse_active=True))
        assert _publish(mpc, -10.0) == pytest.approx(-0.4)

    def test_the_manoeuvres_own_stop_is_published_as_zero(self):
        mpc = _FakeMPC(supervisor=_Supervisor(reverse_active=True))
        assert _publish(mpc, 0.0) == 0.0
        assert mpc.reverse_suppressed == 0


class TestForwardIsUntouched:
    """The gate must be invisible to every forward command."""

    def test_forward_speeds_pass_through(self):
        for v in (0.0, 0.05, 0.4, 0.5):
            mpc = _FakeMPC(supervisor=None)
            assert _publish(mpc, v) == pytest.approx(v)
            assert mpc.reverse_suppressed == 0

    def test_forward_overshoot_still_clamps_to_the_forward_limit(self):
        mpc = _FakeMPC(supervisor=None)
        assert _publish(mpc, 0.52) == pytest.approx(0.5)

    def test_a_nan_request_is_not_mistaken_for_a_reverse(self):
        mpc = _FakeMPC(supervisor=None)
        assert _publish(mpc, math.nan) == 0.0
        assert mpc.reverse_suppressed == 0


def test_the_forward_solver_may_not_plan_a_negative_speed():
    """The forward box's floor is vMin; the rear solver overrides it with -v_rev."""
    import inspect
    import re
    src = inspect.getsource(MPCController.__init__)
    m = re.search(r'"vMin":\s*([-0-9.]+)', src)
    assert m, 'vMin is gone from the forward limits'
    assert float(m.group(1)) == 0.0
