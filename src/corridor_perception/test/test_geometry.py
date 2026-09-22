import math

import numpy as np
import pytest

from corridor_perception.geometry import Line, Pose2D, angdiff, normalise_line, wrap_pi


def _line(rho, alpha, cov=None, t0=-1.0, t1=1.0):
    n = np.array([math.cos(alpha), math.sin(alpha)])
    t = np.array([-math.sin(alpha), math.cos(alpha)])
    return Line(rho, alpha, np.eye(2) * 1e-4 if cov is None else cov,
                rho * n + t0 * t, rho * n + t1 * t, 20, 'test')


def test_wrap_pi_lands_in_half_open_interval():
    a = np.linspace(-20, 20, 4001)
    w = wrap_pi(a)
    assert np.all(w >= -math.pi) and np.all(w < math.pi)
    assert np.allclose(np.cos(w), np.cos(a)) and np.allclose(np.sin(w), np.sin(a))
    assert wrap_pi(math.pi) == pytest.approx(-math.pi)


def test_angdiff_is_smallest_signed_difference():
    assert angdiff(math.radians(179), math.radians(-179)) == pytest.approx(math.radians(-2))
    assert angdiff(0.1, 0.3) == pytest.approx(-0.2)


def test_normalise_line_flips_negative_rho():
    rho, alpha = normalise_line(-2.0, 0.3)
    assert rho == 2.0 and alpha == pytest.approx(0.3 - math.pi)


def test_compose_and_inverse_round_trip_points():
    rng = np.random.default_rng(0)
    a = Pose2D(1.0, -2.0, 0.7)
    b = Pose2D(-0.3, 0.5, -2.1)
    pts = rng.normal(size=(10, 2))
    assert np.allclose(a.compose(b).transform_points(pts),
                       a.transform_points(b.transform_points(pts)))
    ident = a.compose(a.inverse())
    assert abs(ident.x) < 1e-12 and abs(ident.y) < 1e-12 and abs(ident.theta) < 1e-12


def test_line_transform_matches_transformed_points():
    line = _line(2.0, 0.4)
    pose = Pose2D(3.0, -1.0, 2.5)
    out = line.transform(pose, frame='odom')
    assert out.rho >= 0
    for p in pose.transform_points(np.vstack([line.p_start, line.p_end])):
        assert abs(out.signed_distance(p)) < 1e-9
    assert out.frame == 'odom'


def test_yaw_in_place_changes_alpha_only():
    line = _line(2.0, 0.4)
    out = line.transform(Pose2D(0.0, 0.0, 0.9))
    assert out.rho == pytest.approx(2.0)
    assert out.alpha == pytest.approx(1.3)


@pytest.mark.parametrize('pose', [Pose2D(3.0, -1.0, 2.5), Pose2D(-4.0, 6.0, -0.3),
                                  Pose2D(0.5, 0.2, 1.0)])
def test_line_transform_covariance_matches_numeric_jacobian(pose):
    # Includes a case where the transformed rho goes negative and flips.
    cov = np.array([[4e-4, 1e-4], [1e-4, 9e-5]])
    line = _line(1.5, -0.8, cov)
    out = line.transform(pose)

    def f(v):
        return line.__class__(v[0], v[1], cov, line.p_start, line.p_end).transform(pose)

    J = np.zeros((2, 2))
    eps = 1e-6
    for k in range(2):
        dv = np.zeros(2)
        dv[k] = eps
        hi, lo = f(line.as_vector() + dv), f(line.as_vector() - dv)
        J[:, k] = [(hi.rho - lo.rho) / (2 * eps), angdiff(hi.alpha, lo.alpha) / (2 * eps)]
    assert np.allclose(out.cov, J @ cov @ J.T, rtol=1e-4, atol=1e-10)


def test_innovation_reads_opposite_side_observation_as_same_surface():
    # A line passing close to the origin can come back from a fit as
    # (small rho, alpha + pi). It must not look like a different surface.
    a = _line(0.02, 0.5)
    b = _line(0.01, 0.5 + math.pi - 0.001)
    innov = a.innovation(b)
    assert abs(innov[0]) < 0.05 and abs(innov[1]) < 0.01


def test_overlap_is_minus_doorway_width_for_collinear_stretches():
    left = _line(1.25, math.pi / 2, t0=0.0, t1=-4.0)      # x from 0 to 4
    right = _line(1.25, math.pi / 2, t0=-5.2, t1=-9.0)    # x from 5.2 to 9
    assert left.overlap(right) == pytest.approx(-1.2)
    assert left.overlap(_line(1.25, math.pi / 2, t0=-3.0, t1=-6.0)) == pytest.approx(1.0)


def test_distance_and_bearing_from_pose():
    wall = _line(8.0, 0.0)
    pose = Pose2D(3.0, 0.4, 0.6)
    assert wall.distance_from(pose) == pytest.approx(5.0)
    assert wall.bearing_from(pose) == pytest.approx(-0.6)
