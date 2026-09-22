"""The motion-noise model, and the scale measurement that says it cannot yet
be calibrated."""

import math

import numpy as np
import pytest

from corridor_perception.geometry import Pose2D
from corridor_perception.process_noise import (ProcessNoiseParams, compound,
                                               measure_odom_scale)


def test_defaults_declare_themselves_uncalibrated():
    """Every recording predates the drivetrain gain fix, so no honest numbers
    exist yet and the defaults must not pretend otherwise."""
    assert ProcessNoiseParams().calibrated is False


def test_step_cov_grows_with_motion_and_vanishes_at_rest():
    p = ProcessNoiseParams()
    assert np.allclose(p.step_cov(0.0, 0.0), 0.0)
    near, far = p.step_cov(1.0, 0.0), p.step_cov(4.0, 0.0)
    assert far[1, 1] == pytest.approx(4.0 * near[1, 1])


def test_a_straight_step_still_carries_heading_uncertainty():
    """A model with only sigma_yaw_per_rad would give a straight traverse zero
    heading uncertainty, and heading error is what turns into cross-track error
    while a track coasts. See the module docstring's note on the brief."""
    assert ProcessNoiseParams().step_cov(1.0, 0.0)[2, 2] > 0.0


def test_compounding_beats_summing_when_the_heading_is_uncertain():
    """Summing per-step covariances drops the term where an early heading error
    rotates later translation. The compounded result must be larger."""
    p = ProcessNoiseParams()
    step = Pose2D(0.1, 0.0, 0.0)
    q = p.step_cov(0.1, 0.0)

    delta, cov = Pose2D(), np.zeros((3, 3))
    for _ in range(20):                       # 2 m in 10 cm steps
        delta, cov = compound(delta, cov, step, q)

    summed = 20.0 * q
    assert delta.x == pytest.approx(2.0)
    assert cov[1, 1] > summed[1, 1]
    assert cov[2, 2] == pytest.approx(summed[2, 2])   # heading itself just adds


def test_compounding_a_turn_then_a_drive_moves_the_uncertainty_sideways():
    p = ProcessNoiseParams()
    delta, cov = compound(Pose2D(), np.zeros((3, 3)),
                          Pose2D(0.0, 0.0, math.pi / 2), p.step_cov(0.0, math.pi / 2))
    delta, cov = compound(delta, cov, Pose2D(1.0, 0.0, 0.0), p.step_cov(1.0, 0.0))
    assert delta.y == pytest.approx(1.0)
    # After a left turn the step's along-track axis is the world's y.
    assert cov[0, 0] > cov[1, 1]


def test_measured_scale_matches_the_uncommitted_gain_change(straight_traverse_path):
    """The lidar measures the odometry scale error without SLAM or a filter.

    The end wall recedes by what the vehicle really travelled; /odom reports
    what it thought. On this recording that ratio is ~1.22, which is the same
    factor vesc.yaml's uncommitted change was derived from by a completely
    different route (SLAM displacement over 322 intervals).
    """
    from corridor_perception.replay import BagReplay
    m = measure_odom_scale(BagReplay(straight_traverse_path))
    assert m.odom_distance_m == pytest.approx(15.0, abs=0.5)
    assert m.lidar_closing_m == pytest.approx(18.3, abs=0.5)
    assert 1.15 < m.scale < 1.30
    # ... and therefore the gain vesc.yaml now carries, uncommitted.
    assert m.implied_gain == pytest.approx(4516.9, rel=0.05)
