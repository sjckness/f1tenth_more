"""track_velocity.py: per-track velocity in the local odom frame (mpc-dodge). Pure, no rclpy."""
import math

from f1tenth_costmap import track_velocity as tv
import numpy as np
import pytest


def _feed(est, tid, t0, n, dt, pos_fn, noise=0.0, seed=0):
    rng = np.random.default_rng(seed)
    for i in range(n):
        t = t0 + i * dt
        x, y = pos_fn(t)
        est.update(tid, t, x + rng.normal(0, noise), y + rng.normal(0, noise))


def test_constant_velocity_converges_and_is_valid():
    est = tv.TrackVelocityEstimator()
    _feed(est, 7, 0.0, 20, 0.1, lambda t: (2.0 + 0.0 * t, -1.0 + 0.6 * t), noise=0.02)
    e = est.estimate(7)
    assert e.vx == pytest.approx(0.0, abs=0.12)
    assert e.vy == pytest.approx(0.6, abs=0.12)
    assert e.valid and e.sigma_v < 0.25
    assert e.heading == pytest.approx(math.pi / 2, abs=0.25)


def test_static_object_is_not_valid_and_young_track_is_not_valid():
    est = tv.TrackVelocityEstimator()
    _feed(est, 1, 0.0, 20, 0.1, lambda t: (2.0, 0.5), noise=0.01)
    assert not est.estimate(1).valid          # |v| below the floor
    est2 = tv.TrackVelocityEstimator()
    _feed(est2, 2, 0.0, 3, 0.1, lambda t: (2.0, 0.5 + t))
    assert not est2.estimate(2).valid         # 0.2 s old < min_age_s


def test_new_id_and_long_gap_restart_the_filter():
    est = tv.TrackVelocityEstimator()
    _feed(est, 3, 0.0, 15, 0.1, lambda t: (t, 0.0))
    assert est.estimate(3).valid
    est.update(3, 5.0, 10.0, 0.0)             # 3.6 s gap > max_gap_s: a fresh filter
    assert est.estimate(3).age_s == 0.0 and not est.estimate(3).valid
    assert est.estimate(4) is None
    est.retain([4])
    assert est.estimate(3) is None


def test_out_of_order_measurement_is_ignored():
    est = tv.TrackVelocityEstimator()
    assert est.update(1, 1.0, 0.0, 0.0)
    assert not est.update(1, 0.9, 5.0, 5.0)
    assert not est.update(1, 1.0, 5.0, 5.0)


def test_ego_motion_compensated_static_object_has_no_velocity():
    """
    Show a static object seen from a moving car has ~zero velocity once in odom.

    The car drives at 0.5 m/s and turns; a static object measured in base_link, brought into
    odom at each measurement's stamp, has ~zero velocity -- differencing base_link positions
    would have shown the car's own -0.5 m/s.
    """
    est = tv.TrackVelocityEstimator()
    obj = (3.0, 1.0)
    hist = []
    for i in range(60):
        t = 0.05 * i
        yaw = 0.2 * t
        hist.append((t, (0.5 * t * math.cos(yaw), 0.5 * t * math.sin(yaw), yaw)))
    for i in range(1, 25):
        t = 0.1 * i + 0.013                   # scan stamps between odom samples
        pose = tv.interpolate_pose(hist, t)
        c, s = math.cos(pose[2]), math.sin(pose[2])
        dx, dy = obj[0] - pose[0], obj[1] - pose[1]
        bl = (c * dx + s * dy, -s * dx + c * dy)
        xo = pose[0] + c * bl[0] - s * bl[1]
        yo = pose[1] + s * bl[0] + c * bl[1]
        est.update(9, t, xo, yo)
    e = est.estimate(9)
    assert e.speed < 0.05 and not e.valid


def test_map_to_odom_cancels_the_map_pose():
    # the same base_link point placed with a map pose, taken back and put in odom
    pose_map = (10.0, -3.0, 1.0)
    pose_odom = (0.5, 0.2, 0.3)
    bl = (1.2, -0.4)
    c, s = math.cos(pose_map[2]), math.sin(pose_map[2])
    p_map = (pose_map[0] + c * bl[0] - s * bl[1], pose_map[1] + s * bl[0] + c * bl[1])
    xo, yo = tv.map_to_odom_at(p_map, pose_map, pose_odom)
    c, s = math.cos(pose_odom[2]), math.sin(pose_odom[2])
    assert (xo, yo) == pytest.approx((pose_odom[0] + c * bl[0] - s * bl[1],
                                      pose_odom[1] + s * bl[0] + c * bl[1]))


def test_interpolate_pose_bounds_and_wrap():
    hist = [(0.0, (0.0, 0.0, 3.1)), (1.0, (1.0, 2.0, -3.1))]
    x, y, yaw = tv.interpolate_pose(hist, 0.5)
    assert (x, y) == pytest.approx((0.5, 1.0))
    assert abs(math.atan2(math.sin(yaw - math.pi), math.cos(yaw - math.pi))) < 0.01
    assert tv.interpolate_pose(hist, -0.1) is None
    assert tv.interpolate_pose(hist, 1.1) == hist[-1][1]
    assert tv.interpolate_pose(hist, 2.0) is None


def test_arc_correction_moves_the_centroid_away_from_the_sensor():
    x, y = tv.arc_centre_correction((2.0, 0.0), 0.25, (0.0, 0.0))
    assert x == pytest.approx(2.0 + 0.5 / math.pi) and y == pytest.approx(0.0)
    assert tv.arc_centre_correction((2.0, 0.0), 0.0) == (2.0, 0.0)
