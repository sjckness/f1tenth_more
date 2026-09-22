"""Bag replay, against a real recording.

Every assertion here is about consistency with the recording itself: there is
no ground truth in a bag. Where a number is checked absolutely it comes from
the recording's own metadata or from a second, independent quantity in the
same bag.
"""

import math

import numpy as np
import pytest

from corridor_perception.geometry import Pose2D
from corridor_perception.replay import BagReplay
from corridor_perception.scan import NO_RETURN_MIN_M


def test_replay_plays_the_recording_end_to_end(straight_traverse):
    frames = straight_traverse
    assert len(frames) > 1000
    stamps = [f.stamp for f in frames]
    assert stamps == sorted(stamps)
    assert 25.0 < stamps[-1] - stamps[0] < 40.0


def test_frames_carry_matching_ranges_and_angles(straight_traverse):
    f = straight_traverse[0]
    assert f.ranges.shape == f.angles.shape == (1081,)
    # The UST sweep is symmetric about straight ahead, 270 degrees wide.
    assert f.angles[0] == pytest.approx(-math.radians(135), abs=1e-3)
    assert f.angles[-1] == pytest.approx(math.radians(135), abs=1e-3)


def test_no_return_sentinel_becomes_infinite(straight_traverse):
    """The driver reports no-return as 65.533 m, a finite number that would be
    fitted as a wall. Nothing at or beyond the sentinel may survive replay."""
    saw_one = False
    for f in straight_traverse[::100]:
        finite = f.ranges[np.isfinite(f.ranges)]
        assert finite.size, 'a scan with no valid return at all'
        assert finite.max() < NO_RETURN_MIN_M
        saw_one |= bool((~np.isfinite(f.ranges)).any())
    # And the recording really does contain them, or this proves nothing.
    assert saw_one, 'no invalid returns in the sample: the filter is untested here'


def test_odometry_is_interpolated_to_the_scan_stamp(straight_traverse_path):
    """/scan runs at 40 Hz and /odom at ~10 Hz, so a pose per scan can only
    come from interpolation; consecutive frames must not repeat a pose."""
    frames = list(BagReplay(straight_traverse_path))
    moving = [f for f in frames if f.ds > 1e-4]
    assert len(moving) > 500
    poses = {(round(f.odom.x, 6), round(f.odom.y, 6)) for f in moving}
    assert len(poses) > 0.9 * len(moving)


def test_odom_step_composes_back_to_the_absolute_pose(straight_traverse):
    """odom_step is the motion in the PREVIOUS base frame: composing it onto
    the previous pose must reproduce the current one exactly."""
    for prev, cur in zip(straight_traverse[:200], straight_traverse[1:201]):
        rebuilt = prev.odom.compose(cur.odom_step)
        assert rebuilt.x == pytest.approx(cur.odom.x, abs=1e-9)
        assert rebuilt.y == pytest.approx(cur.odom.y, abs=1e-9)
        assert rebuilt.theta == pytest.approx(cur.odom.theta, abs=1e-9)


def test_extrinsic_comes_from_the_bag_not_the_urdf(straight_traverse):
    """The URDF says the laser is rear-facing at x = -0.12, yaw = pi. Every
    recording says forward-facing at x = +0.12, and the recordings are what the
    scans were taken under. See replay.SENSOR_IN_BASE_NOTE."""
    s = straight_traverse[0].sensor_in_base
    assert s.x == pytest.approx(0.12, abs=1e-6)
    assert s.theta == pytest.approx(0.0, abs=1e-6)


def test_sensor_pose_composes_base_and_extrinsic(straight_traverse):
    f = straight_traverse[0]
    assert f.odom_sensor.x == pytest.approx(f.odom.compose(f.sensor_in_base).x)


def test_caller_can_override_the_extrinsic(straight_traverse_path):
    override = Pose2D(-0.12, 0.0, math.pi)
    f = next(iter(BagReplay(straight_traverse_path, sensor_in_base=override)))
    assert f.sensor_in_base.theta == pytest.approx(math.pi)


def test_info_reports_what_the_recording_contains(straight_traverse_path):
    info = BagReplay(straight_traverse_path).info()
    assert info.n_frames > 1000
    assert info.n_dropped < 0.05 * info.n_scan
    assert info.path_length_m > 10.0
    assert info.no_return_fraction > 0.1
    # The header overstates what the sensor returns. Asserted as a RELATION,
    # not a band: the effective horizon is surface-dependent (pale plaster far,
    # glazed or dark short), so a figure written in here would be wrong on the
    # next corridor. Nothing in this package may treat either number as a
    # horizon; acquirability is decided by evidence.
    assert info.observed_max_range < info.header_range_max
    assert info.observed_max_range < NO_RETURN_MIN_M


def test_a_bag_without_the_topic_says_so(straight_traverse_path):
    with pytest.raises(KeyError, match='no /nope'):
        BagReplay(straight_traverse_path, scan_topic='/nope')
