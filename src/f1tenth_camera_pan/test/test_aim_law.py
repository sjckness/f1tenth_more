"""Aim-law unit tests -- pure, no ROS, no built workspace needed.

Covers the behaviours the spec calls out: straight -> 0, left/right sign,
clamp, kappa->0 continuity, reverse/stopped -> 0, the measured/steering blend,
and the PanSmoother conditioning.
"""
import math

from f1tenth_camera_pan.aim_law import (
    aim_pan,
    AimParams,
    curvature,
    lookahead_point,
    PanSmoother,
)
import pytest

P = AimParams()  # defaults (must match stack_params.yaml)


def test_straight_cruise_aims_zero():
    assert aim_pan(2.0, 0.0, 0.0, P) == pytest.approx(0.0, abs=1e-9)


def test_left_turn_pans_left_positive():
    # low speed -> steering-only branch; positive steering = left = positive pan
    pan = aim_pan(0.4, 0.0, 0.25, P)
    assert pan > 0.0


def test_right_turn_pans_right_negative():
    pan = aim_pan(0.4, 0.0, -0.25, P)
    assert pan < 0.0


def test_sign_is_symmetric():
    assert aim_pan(0.4, 0.0, 0.25, P) == pytest.approx(-aim_pan(0.4, 0.0, -0.25, P))


def test_hard_turn_is_clamped_to_max_pan():
    # a huge curvature would aim far to the side; must saturate at +-max_pan
    pan = aim_pan(3.0, 10.0, 0.0, P)        # measured kappa huge and left
    assert pan == pytest.approx(P.max_pan_rad, abs=1e-9)
    pan_r = aim_pan(3.0, -10.0, 0.0, P)
    assert pan_r == pytest.approx(-P.max_pan_rad, abs=1e-9)


def test_stopped_aims_zero():
    assert aim_pan(0.0, 5.0, 0.5, P) == 0.0            # |v| < v_stop
    assert aim_pan(0.05, 5.0, 0.5, P) == 0.0


def test_reverse_aims_zero():
    assert aim_pan(-1.0, 0.0, 0.3, P) == 0.0


def test_track_when_stopped_follows_steering():
    p = AimParams(track_when_stopped=True)
    assert aim_pan(0.0, 0.0, 0.3, p) > 0.0        # parked, steer left -> pan left
    assert aim_pan(0.0, 0.0, -0.3, p) < 0.0       # steer right -> pan right
    assert aim_pan(0.0, 0.0, 0.0, p) == pytest.approx(0.0)   # centred -> 0


def test_track_when_stopped_is_clamped():
    p = AimParams(track_when_stopped=True)
    assert aim_pan(0.0, 0.0, 0.4, p) == pytest.approx(p.max_pan_rad, abs=1e-9)


def test_track_when_stopped_reverse_still_zero():
    p = AimParams(track_when_stopped=True)
    assert aim_pan(-1.0, 0.0, 0.3, p) == 0.0      # reversing -> 0 even so


def test_default_stopped_ignores_steering():
    assert aim_pan(0.0, 0.0, 0.3, P) == 0.0       # track_when_stopped off (default)


def test_curvature_kappa_zero_limit_is_a_straight_line():
    # lookahead point is continuous as kappa -> 0 (series form, no div0)
    for k in (1e-3, 1e-6, 1e-9, 0.0):
        px, py = lookahead_point(k, 2.0)
        assert px == pytest.approx(2.0, abs=1e-2)
        assert py == pytest.approx(k * 2.0 ** 2 / 2.0, abs=1e-2)  # ~kappa s^2/2 -> 0


def test_lookahead_point_matches_closed_form_for_a_real_arc():
    kappa, s = 0.5, 2.0          # R = 2 m
    px, py = lookahead_point(kappa, s)
    assert px == pytest.approx(math.sin(kappa * s) / kappa)
    assert py == pytest.approx((1.0 - math.cos(kappa * s)) / kappa)


def test_blend_prefers_measured_at_high_speed():
    # delta says straight, omega_z says turning left; above v_curv_full the
    # measured curvature wins, so we still pan left.
    k = curvature(2.0, 1.0, 0.0, P)          # kappa_meas = 0.5, w = 1
    assert k == pytest.approx(0.5, abs=1e-9)


def test_blend_prefers_steering_at_low_speed():
    # below v_curv_min the measured omega_z/v is ignored (would be noisy/huge)
    k = curvature(0.3, 5.0, 0.2, P)          # w = 0 -> steering only
    assert k == pytest.approx(math.tan(0.2) / P.wheelbase_m, abs=1e-9)


def test_blend_is_continuous_across_the_speed_window():
    # no jump: sample kappa across v_curv_min..v_curv_full, consecutive diffs small
    ks = [curvature(v / 100.0, 1.0, 0.1, P)
          for v in range(int(P.v_curv_min_mps * 100) - 20, int(P.v_curv_full_mps * 100) + 20)]
    for a, b in zip(ks, ks[1:]):
        assert abs(b - a) < 0.5


def test_lookahead_arc_length_is_clamped():
    # very slow but moving -> s floored at s_min (not ~0), very fast -> capped
    # (checked indirectly: pan at tiny speed uses a non-degenerate arc)
    slow = aim_pan(0.2, 0.0, 0.3, P)
    assert slow != 0.0


class TestPanSmoother:

    def test_rate_limit_caps_the_step(self):
        s = PanSmoother(deadband_rad=0.0, lp_tau_s=0.0, rate_max_radps=1.0, initial=0.0)
        out = s.update(1.0, 0.1)             # want +1 rad, allowed 0.1 rad in 0.1 s
        assert out == pytest.approx(0.1, abs=1e-9)

    def test_deadband_holds_small_changes(self):
        s = PanSmoother(deadband_rad=0.05, lp_tau_s=0.0, rate_max_radps=100.0, initial=0.2)
        out = s.update(0.23, 0.1)            # change 0.03 < deadband -> hold 0.2
        assert out == pytest.approx(0.2, abs=1e-9)

    def test_lowpass_moves_partway(self):
        s = PanSmoother(deadband_rad=0.0, lp_tau_s=0.1, rate_max_radps=100.0, initial=0.0)
        out = s.update(1.0, 0.1)             # alpha = 0.1/0.2 = 0.5
        assert out == pytest.approx(0.5, abs=1e-9)

    def test_converges_to_target_over_time(self):
        s = PanSmoother(deadband_rad=0.0, lp_tau_s=0.05, rate_max_radps=10.0, initial=0.0)
        for _ in range(200):
            out = s.update(0.2, 0.02)
        assert out == pytest.approx(0.2, abs=1e-3)
