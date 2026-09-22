import math

import numpy as np
import pytest

from corridor_perception.axis import (AxisEstimate, SurfaceClass, classify, classify_all,
                                      estimate_axis)
from corridor_perception.extraction import extract_segments
from corridor_perception.geometry import Line, Pose2D, angdiff


def _seg(p0, p1):
    p0, p1 = np.asarray(p0, float), np.asarray(p1, float)
    d = p1 - p0
    alpha = math.atan2(d[0], -d[1])
    rho = p0 @ np.array([math.cos(alpha), math.sin(alpha)])
    if rho < 0:
        rho, alpha = -rho, alpha + math.pi
    return Line(rho, math.atan2(math.sin(alpha), math.cos(alpha)), np.eye(2) * 1e-4,
                p0, p1, 50, 'world')


CORRIDOR = [_seg((0, 1.25), (10, 1.25)), _seg((0, -1.25), (10, -1.25)),
            _seg((10, -1.25), (10, 1.25)), _seg((0, -1.25), (0, 1.25))]


def test_axis_of_clean_corridor():
    ax = estimate_axis(CORRIDOR, forward_hint=0.0)
    assert ax.strength == pytest.approx(1.0)
    assert ax.theta == pytest.approx(0.0, abs=1e-9)
    assert ax.lateral_length == pytest.approx(20.0)
    assert ax.frontal_length == pytest.approx(5.0)
    assert ax.is_corridor


def test_forward_hint_selects_direction():
    assert abs(angdiff(estimate_axis(CORRIDOR, forward_hint=math.pi - 0.2).theta,
                       math.pi)) < 1e-9


def test_unstructured_scene_is_flagged_not_a_corridor():
    rng = np.random.default_rng(0)
    segs = []
    for _ in range(30):
        c = rng.uniform(-5, 5, 2)
        ang = rng.uniform(0, math.pi)
        d = 0.5 * np.array([math.cos(ang), math.sin(ang)])
        segs.append(_seg(c - d, c + d))
    ax = estimate_axis(segs)
    assert ax.strength < 0.75
    assert not ax.is_corridor


def test_no_segments_gives_zero_strength():
    ax = estimate_axis([], forward_hint=0.4)
    assert ax.strength == 0.0 and not ax.is_corridor and ax.theta == pytest.approx(0.4)


def test_classification_is_relative_to_axis_not_heading():
    ax = AxisEstimate(0.0, 1.0, 4, 20.0, 5.0)
    labels = [classify_all(CORRIDOR, ax, Pose2D(5.0, 0.0, yaw))
              for yaw in np.radians([0, 45, 89, 135, 180, -120])]
    expected = [SurfaceClass.LATERAL_LEFT, SurfaceClass.LATERAL_RIGHT,
                SurfaceClass.FRONTAL, SurfaceClass.REAR]
    assert all(lab == expected for lab in labels)


def test_diagonal_segment_is_unstructured():
    ax = AxisEstimate(0.0, 1.0, 4, 20.0, 5.0)
    assert classify(_seg((2, -1), (3, 0)), ax, Pose2D()) == SurfaceClass.UNSTRUCTURED


@pytest.mark.xfail(strict=True, reason=(
    'estimate_axis swaps the lateral/frontal families by ~90 deg partway down a '
    'real corridor: it picks whichever family has more support, and when the end '
    'wall and a side opening outweigh the side walls that choice flips. '
    'forward_hint cannot help -- it resolves the 180 deg ambiguity only. Holding '
    'the family across frames is Step 2/3 work (structural rejection, axis '
    'validity), so this records the requirement instead of building ahead.'))
def test_axis_is_stable_along_a_real_traverse_with_the_seed_carried(straight_traverse_filtered):
    """The seed is set once at entry and carried; the axis must not drift or
    flip along a real run down a real corridor.

    No ground truth: the corridor's true direction is unknown. What IS known by
    construction is that the vehicle drove down ONE straight corridor, so the
    axis estimated in odom must stay put. Every frame is compared against the
    first accepted estimate, which is the only self-consistent datum available.
    """
    hint, first, seen = None, None, 0
    for f in straight_traverse_filtered[::25]:
        segs = [s.transform(f.odom_sensor, 'odom')
                for s in extract_segments(f.ranges, f.angles)]
        ax = estimate_axis(segs, forward_hint=hint)
        if not ax.is_corridor:
            continue                      # not corridor-like here; P4, not a failure
        seen += 1
        if first is None:
            first, hint = ax.theta, ax.theta
            continue
        # A flip would show as ~180 deg; drift as a slow creep. Both fail here.
        assert abs(angdiff(ax.theta, first)) < math.radians(10.0), f.index
        hint = ax.theta
    assert seen > 20, f'only {seen} corridor-like frames; the test proved little'


def test_raw_odom_yaw_drift_would_break_the_axis(straight_traverse):
    """The same traverse off raw /odom, which cannot hold the axis.

    Not a test of this package: a test of the motion source it is given. Over
    15 m of straight driving /odom accumulates ~19 deg of yaw that SLAM, both
    EKFs and the lidar all disagree with, so every surface tracked in that
    frame rotates out from under its own association gate. Asserted here so
    that if the drivetrain calibration is ever fixed, this test fails loudly
    and the filtered-source workaround above can be reconsidered.
    """
    thetas = []
    for f in straight_traverse[::25]:
        segs = [s.transform(f.odom_sensor, 'odom')
                for s in extract_segments(f.ranges, f.angles)]
        ax = estimate_axis(segs, forward_hint=thetas[-1] if thetas else None)
        if ax.is_corridor:
            thetas.append(ax.theta)
    spread = max(abs(angdiff(t, thetas[0])) for t in thetas)
    assert spread > math.radians(15.0), (
        f'raw /odom held the axis to {math.degrees(spread):.1f} deg: has the '
        'drivetrain yaw calibration been fixed? Re-read this test.')
