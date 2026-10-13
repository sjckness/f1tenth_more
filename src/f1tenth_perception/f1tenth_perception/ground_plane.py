"""ground_plane.py -- floor detection for the depth front-wall estimate.

Pure: numpy only, no rclpy. Same "pure logic separate from ROS glue" split as
lidar_front_wall.py.

WHY. front_clearance_node's background distance is a low percentile band of
the ROI's unmasked depth. The ROI reaches below the horizon, so floor pixels
2-3 m ahead are always the nearest consistent surface, and the "wall" read
~2.9 m with the real wall at 5.65 m (run P002-R005, 2026-10-13). Removing the
floor leaves the pixels that stand up from it: the wall, and anything the
object mask missed.

HOW. Depth pixels are back-projected through the camera intrinsics and moved
into base_frame (z up). A plane is RANSAC-fitted to the points near the
expected floor height, with its normal constrained near vertical, and refined
by least squares on its inliers. A pixel whose point lies less than
``height_m`` above that plane is ground. When no plane is found the expected
floor (z = prior_z, horizontal) is used instead, so a frame of pure wall still
loses nothing it should keep.
"""

import math

import numpy as np

__all__ = ['GroundPlane', 'back_project', 'fit_ground_plane', 'ground_mask',
           'quat_to_matrix']


class GroundPlane:
    """n . p + d = 0 in base_frame, n unit with n_z > 0 (up)."""

    __slots__ = ('normal', 'd', 'inliers', 'fitted')

    def __init__(self, normal, d, inliers=0, fitted=False):
        self.normal = np.asarray(normal, dtype=float)
        self.d = float(d)
        self.inliers = int(inliers)
        self.fitted = bool(fitted)

    def height(self, points):
        """Signed height [m] of (N, 3) points above the plane."""
        return points @ self.normal + self.d

    def tilt_deg(self):
        return math.degrees(math.acos(min(1.0, abs(float(self.normal[2])))))

    def z_at_origin(self):
        """Floor height [m] under the base_frame origin."""
        return -self.d / float(self.normal[2])


def quat_to_matrix(x, y, z, w):
    """3x3 rotation matrix of a unit quaternion."""
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def back_project(depth, rows, cols, fx, fy, cx, cy, rotation, translation):
    """(N, 3) base_frame points of pixels (rows, cols) of a Z-depth image.

    ``depth`` holds the optical-axis distance Z (the ZED's depth convention),
    so X = (u - cx) Z / fx and Y = (v - cy) Z / fy in the optical frame;
    ``rotation``/``translation`` take optical-frame points into base_frame.
    """
    z = depth[rows, cols].astype(float)
    x = (cols - cx) * z / fx
    y = (rows - cy) * z / fy
    pts = np.stack((x, y, z), axis=1)
    return pts @ np.asarray(rotation).T + np.asarray(translation)


def fit_ground_plane(points, prior_z=0.0, band_m=0.15, inlier_m=0.03,
                     max_tilt_deg=15.0, iters=60, min_inliers=150, rng=None):
    """RANSAC floor plane among (N, 3) base_frame points, or the prior.

    Only points within ``band_m`` of ``prior_z`` are candidates, so a wall
    can never be taken for the floor. Returns a GroundPlane; ``fitted`` is
    False when the prior had to be used.
    """
    prior = GroundPlane((0.0, 0.0, 1.0), -float(prior_z))
    pts = np.asarray(points, dtype=float)
    if pts.size == 0:
        return prior
    pts = pts[np.all(np.isfinite(pts), axis=1)]
    cand = pts[np.abs(pts[:, 2] - prior_z) <= band_m]
    if cand.shape[0] < max(3, min_inliers):
        return prior
    rng = np.random.default_rng(0) if rng is None else rng
    cos_tilt = math.cos(math.radians(max_tilt_deg))
    best_mask, best_count = None, 0
    for _ in range(int(iters)):
        p0, p1, p2 = cand[rng.choice(cand.shape[0], 3, replace=False)]
        n = np.cross(p1 - p0, p2 - p0)
        norm = np.linalg.norm(n)
        if norm < 1e-9:
            continue
        n = n / norm
        if n[2] < 0:
            n = -n
        if n[2] < cos_tilt:
            continue
        mask = np.abs(cand @ n - n @ p0) < inlier_m
        count = int(np.count_nonzero(mask))
        if count > best_count:
            best_mask, best_count = mask, count
    if best_mask is None or best_count < min_inliers:
        return prior
    # Least-squares refinement: the smallest singular vector of the centred
    # inliers is the normal.
    inl = cand[best_mask]
    centroid = inl.mean(axis=0)
    n = np.linalg.svd(inl - centroid, full_matrices=False)[2][-1]
    if n[2] < 0:
        n = -n
    if n[2] < cos_tilt:
        return prior
    return GroundPlane(n, -float(n @ centroid), best_count, True)


def ground_mask(points, plane, height_m=0.05):
    """Boolean (N,): True where a point is less than height_m above the floor."""
    return plane.height(np.asarray(points, dtype=float)) < height_m
