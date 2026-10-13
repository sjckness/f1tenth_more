"""ground_plane: the floor is removed, the wall behind it is kept.

Synthetic Z-depth image from the car's real ZED mount (base_link ->
zed2_left_camera_optical_frame, z 0.165, level): a floor at z 0 and a wall
5 m ahead -- the P002-R005 scene, where the floor read as a 2.9 m "wall".
"""

import numpy as np

from f1tenth_perception.ground_plane import (
    back_project, fit_ground_plane, ground_mask, quat_to_matrix)

FX = FY = 260.0
CX, CY = 320.0, 180.0
W, H = 640, 360
CAM_Z = 0.165
WALL_X = 5.0
ROT = quat_to_matrix(0.5, -0.5, 0.5, -0.5)   # optical -> base_link
TRANS = np.array([0.23, 0.06, CAM_Z])


def _scene(pitch_rows=0.0):
    """Z depth of floor below the horizon, wall where the floor is farther."""
    v = np.arange(H, dtype=float)[:, None] - CY + pitch_rows
    with np.errstate(divide='ignore'):
        floor_z = np.where(v > 0, CAM_Z * FY / v, np.inf)
    wall_z = WALL_X - TRANS[0]
    return np.broadcast_to(np.minimum(floor_z, wall_z), (H, W)).copy()


def _points(depth):
    rows, cols = np.mgrid[0:H:2, 0:W:2]
    rows, cols = rows.ravel(), cols.ravel()
    return rows, cols, back_project(depth, rows, cols, FX, FY, CX, CY, ROT, TRANS)


def test_back_projection_puts_the_floor_at_z_zero_and_the_wall_at_five_metres():
    depth = _scene()
    rows, _, pts = _points(depth)
    floor = rows > CY + 20
    assert np.allclose(pts[floor & (pts[:, 0] < WALL_X - 0.01), 2], 0.0, atol=1e-9)
    assert np.allclose(pts[rows < CY, 0], WALL_X, atol=1e-9)


def test_the_floor_plane_is_found_and_the_wall_survives():
    depth = _scene()
    rows, _, pts = _points(depth)
    plane = fit_ground_plane(pts, prior_z=0.0)
    assert plane.fitted
    assert plane.tilt_deg() < 0.5
    assert abs(plane.z_at_origin()) < 0.01
    mask = ground_mask(pts, plane, 0.06)
    wall = pts[:, 0] > WALL_X - 0.01
    assert not mask[wall & (pts[:, 2] > 0.06)].any()
    assert mask[~wall].all()
    # What front_clearance_node's band statistic sees once the floor is gone.
    assert np.allclose(pts[~mask, 0], WALL_X, atol=1e-9)


def test_a_wall_only_frame_falls_back_to_the_prior_and_keeps_the_wall():
    depth = np.full((H, W), WALL_X - TRANS[0])
    _, _, pts = _points(depth)
    plane = fit_ground_plane(pts[pts[:, 2] > 0.2], prior_z=0.0)
    assert not plane.fitted
    assert not ground_mask(pts[pts[:, 2] > 0.06], plane, 0.06).any()


def test_the_wall_is_never_fitted_as_the_floor():
    """Candidates are near prior_z only; a vertical plane fails the tilt gate."""
    depth = np.full((H, W), WALL_X - TRANS[0])
    _, _, pts = _points(depth)
    plane = fit_ground_plane(pts, prior_z=0.0, band_m=0.15)
    assert plane.tilt_deg() < 15.0
