"""scene_mask.py -- one class for every depth pixel, whole frame, no ROI.

Pure: numpy only (cv2 for polygon rasterisation, imported locally), no rclpy.
The ROS glue is front_clearance_node's scene-mask tick.

WHY. The ROI-based front distance reads whatever fills a fixed rectangle, and
on an open corridor that is the FLOOR: /perception/front_distance sat at ~2.0 m
for a whole run with nothing ahead for more than 10 m (corridor_phantom,
2026-10-15), because the ROI's lower edge looks at the ground a couple of
metres in front of the bumper. A distance is only as good as the question
"what surface is this?", so that question is answered first, per pixel, for
the whole frame, and every later stage consumes the classes rather than raw
depth.

CLASSES, in strict priority order -- the first rule that matches wins, which
is what makes "an object standing on the floor is OBJECT, not FLOOR" true by
construction:

  SELF        static self-mask polygons in image coordinates. The car's own
              LiDAR housing is in the ZED's field of view (bottom-right) and
              returns real, small depths; a second, depth-only polygon covers
              the region where the housing occludes the right lens, where the
              stereo match is against the housing rather than the scene.
              Image-space, so it wins even where the depth is invalid.
  OBJECT      YOLO SEGMENTATION instance masks (not boxes). Also image-space.
  FLOOR       |z| < floor_tol.
  REFLECTION  z < -floor_tol. The glossy floor tiles of this building mirror
              the wall: on the calib clips of 2026-10-14 the "floor" points
              are the wall's reflection, z -0.2..-1.2 m. They are not floor
              and they are certainly not wall.
  CEILING     z > ceiling_min_z.
  WALL        anything else that has a usable depth. This is the only class
              any wall distance is ever taken from.
  INVALID     no usable depth, and not SELF or OBJECT.

FRAMES. z is the height above base_link's own z = 0, which is the floor
(base.xacro), through the live TF -- so the mount pitch (-2.083 deg since
2026-10-15) is already in it and FLOOR is flat at z = 0 only when that pitch
is right. There is no separate floor-plane fit here on purpose: a RANSAC fit
on a reflecting floor fits the reflection.

-INF IS NOT MISSING DATA. The ZED encodes "closer than the minimum measurable
range" as -Inf (front_clearance_node's own module docstring). It has no usable
3D position, so it is INVALID here, but it is counted separately in the stats
(`too_close_frac`) rather than being quietly merged into "no data".
"""

import numpy as np

from f1tenth_perception.ground_plane import back_project

CLASS_INVALID = 0
CLASS_SELF = 1
CLASS_OBJECT = 2
CLASS_FLOOR = 3
CLASS_REFLECTION = 4
CLASS_CEILING = 5
CLASS_WALL = 6

CLASS_NAMES = {
    CLASS_INVALID: 'invalid', CLASS_SELF: 'self', CLASS_OBJECT: 'object',
    CLASS_FLOOR: 'floor', CLASS_REFLECTION: 'reflection', CLASS_CEILING: 'ceiling',
    CLASS_WALL: 'wall',
}

#: BGR overlay colours, for the debug image only.
CLASS_COLOURS = {
    CLASS_INVALID: (40, 40, 40), CLASS_SELF: (0, 0, 255), CLASS_OBJECT: (0, 165, 255),
    CLASS_FLOOR: (0, 160, 0), CLASS_REFLECTION: (200, 0, 200),
    CLASS_CEILING: (160, 120, 0), CLASS_WALL: (255, 255, 0),
}


def polygon_mask(shape_hw, polygons):
    """Bool (H, W) mask of `polygons`, each [(fx, fy), ...] in FRACTIONAL coords.

    Fractions of width/height rather than pixels, for the same reason
    lidar_exclusion_x_min is fractional: one calibration survives a resolution
    change with no rescaling. An empty list gives an all-False mask.
    """
    import cv2  # local: keep the module importable where cv2 is absent
    h, w = int(shape_hw[0]), int(shape_hw[1])
    mask = np.zeros((h, w), dtype=np.uint8)
    polys = []
    for poly in polygons or ():
        pts = np.asarray(poly, dtype=float).reshape(-1, 2)
        if pts.shape[0] >= 3:
            polys.append(np.column_stack([pts[:, 0] * w, pts[:, 1] * h]).round().astype(np.int32))
    if polys:
        cv2.fillPoly(mask, polys, 1)
    return mask.astype(bool)


def rect_polygon(x_min, y_min, x_max, y_max):
    """The fractional rectangle as a 4-vertex polygon, for the params."""
    return [(x_min, y_min), (x_max, y_min), (x_max, y_max), (x_min, y_max)]


def pixel_z(depth, fx, fy, cx, cy, rotation, translation):
    """Base-frame height z [m] of EVERY pixel of a Z-depth image, as (H, W).

    Only the z row of the rotation is needed, so this is three multiplies per
    pixel and no (H, W, 3) intermediate: at 640x360 the full back-projection
    is 5.5 MB and ~4x the arithmetic, and every class but WALL is decided on z
    alone. The WALL pixels get their full xyz from `wall_points`.

    Invalid depths propagate as NaN (the arithmetic does it: inf * 0 is the
    only trap, and k_z is never 0 for a forward-looking camera).
    """
    depth = np.asarray(depth, dtype=np.float32)
    h, w = depth.shape[:2]
    rot = np.asarray(rotation, dtype=float)
    ku = (np.arange(w, dtype=np.float32) - cx) / fx
    kv = (np.arange(h, dtype=np.float32) - cy) / fy
    kz = (rot[2, 0] * ku)[None, :] + (rot[2, 1] * kv)[:, None] + rot[2, 2]
    with np.errstate(invalid='ignore'):
        return (kz * depth + translation[2]).astype(np.float32)


def valid_depth(depth, min_depth=0.2, max_depth=20.0):
    """Bool (H, W): the depth is a usable measurement of a surface."""
    d = np.asarray(depth)
    with np.errstate(invalid='ignore'):
        return np.isfinite(d) & (d >= min_depth) & (d <= max_depth)


def classify(depth, fx, fy, cx, cy, rotation, translation, self_mask=None,
             object_mask=None, floor_tol=0.05, ceiling_min_z=2.0, min_depth=0.2,
             max_depth=20.0):
    """Per-pixel classes (H, W) uint8 and their base-frame z (H, W).

    Strict priority, first match wins (module docstring):
    SELF, OBJECT, FLOOR, REFLECTION, CEILING, WALL, INVALID.
    `self_mask` and `object_mask` are bool (H, W) image-space masks and win
    over every depth-derived class, including where the depth is invalid.
    """
    depth = np.asarray(depth, dtype=np.float32)
    shape = depth.shape[:2]
    z = pixel_z(depth, fx, fy, cx, cy, rotation, translation)
    ok = valid_depth(depth, min_depth, max_depth)

    labels = np.full(shape, CLASS_INVALID, dtype=np.uint8)
    # Lowest priority first, then overwrite: one pass per class, no per-pixel
    # branching, and the final value is the highest-priority match.
    labels[ok] = CLASS_WALL
    labels[ok & (z > ceiling_min_z)] = CLASS_CEILING
    labels[ok & (z < -floor_tol)] = CLASS_REFLECTION
    labels[ok & (np.abs(z) < floor_tol)] = CLASS_FLOOR
    if object_mask is not None:
        labels[np.asarray(object_mask, dtype=bool)] = CLASS_OBJECT
    if self_mask is not None:
        labels[np.asarray(self_mask, dtype=bool)] = CLASS_SELF
    return labels, z


def wall_points(depth, labels, fx, fy, cx, cy, rotation, translation):
    """(N, 3) base-frame points of the WALL pixels, and their (rows, cols)."""
    rows, cols = np.nonzero(np.asarray(labels) == CLASS_WALL)
    if rows.size == 0:
        return np.empty((0, 3), dtype=float), rows, cols
    pts = back_project(np.asarray(depth, dtype=np.float32), rows, cols,
                       fx, fy, cx, cy, rotation, translation)
    return pts, rows, cols


def in_strip(points, front_x, strip_half_width_m):
    """Bool (N,): the point is ahead of the bumper and inside the car's strip.

    The one strip the whole pipeline shares (vehicle_frames.front_side_limit_m):
    a surface the car would hit going straight.
    """
    pts = np.asarray(points, dtype=float).reshape(-1, 3)
    if pts.shape[0] == 0:
        return np.zeros(0, dtype=bool)
    return (pts[:, 0] > front_x) & (np.abs(pts[:, 1]) <= strip_half_width_m)


def class_stats(labels, depth=None, points=None, front_x=0.0, strip_half_width_m=0.0):
    """Per-class pixel fractions, the WALL point count inside the strip, and
    the -Inf (too close to measure) fraction -- the /perception/scene_labels/
    stats payload.
    """
    lab = np.asarray(labels)
    n = int(lab.size)
    counts = np.bincount(lab.ravel(), minlength=CLASS_WALL + 1)
    out = {'n_px': n, 'frac': {CLASS_NAMES[i]: (round(float(counts[i]) / n, 5) if n else 0.0)
                               for i in range(CLASS_WALL + 1)}}
    out['count'] = {CLASS_NAMES[i]: int(counts[i]) for i in range(CLASS_WALL + 1)}
    if points is not None:
        out['wall_points'] = int(np.asarray(points).reshape(-1, 3).shape[0])
        out['wall_points_in_strip'] = int(np.count_nonzero(
            in_strip(points, front_x, strip_half_width_m)))
    if depth is not None:
        d = np.asarray(depth)
        out['too_close_frac'] = round(float(np.count_nonzero(
            np.isneginf(d))) / n, 5) if n else 0.0
    return out


def project_to_image(points, fx, fy, cx, cy, rotation, translation):
    """(u, v) pixel coordinates of base-frame points, NaN for anything behind.

    The inverse of the back-projection: p_opt = R^T (p_base - t). Points with
    a non-positive optical z are behind the lens and come back NaN rather than
    wrapping round to a plausible-looking pixel.
    """
    pts = np.asarray(points, dtype=float).reshape(-1, 3)
    if pts.shape[0] == 0:
        return np.empty((0, 2), dtype=float)
    opt = (pts - np.asarray(translation, dtype=float)) @ np.asarray(rotation, dtype=float)
    with np.errstate(divide='ignore', invalid='ignore'):
        u = fx * opt[:, 0] / opt[:, 2] + cx
        v = fy * opt[:, 1] / opt[:, 2] + cy
    behind = ~(opt[:, 2] > 1e-6)
    u[behind] = np.nan
    v[behind] = np.nan
    return np.column_stack([u, v])


def overlay(image_bgr, labels, scan_px=None, alpha=0.5):
    """The class-coloured debug overlay, with the projected /scan drawn on top.

    `scan_px` is an (M, 2) array of (u, v) pixel coordinates of the LiDAR
    returns that fall in the frame; they are drawn last so the comparison the
    cross-check makes is visible by eye.
    """
    import cv2  # local: keep the module importable where cv2 is absent
    img = np.ascontiguousarray(image_bgr)
    tint = np.zeros_like(img)
    lab = np.asarray(labels)
    for cls, colour in CLASS_COLOURS.items():
        tint[lab == cls] = colour
    out = cv2.addWeighted(img, 1.0 - alpha, tint, alpha, 0.0)
    if scan_px is not None:
        for u, v in np.asarray(scan_px, dtype=float).reshape(-1, 2):
            if np.isfinite(u) and np.isfinite(v):
                cv2.circle(out, (int(round(u)), int(round(v))), 1, (255, 255, 255), -1)
    return out
