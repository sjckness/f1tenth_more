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

FLOOR_INFERRED is added AFTER this classification, not inside it: it is the
floor the LiDAR can see where the stereo matched nothing, and it comes from a
different sensor through a different extrinsic. inferred_floor.py owns it and
its own module docstring says why it is a separate class rather than more
FLOOR. Nothing here produces it; `classify` never returns it.

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
CLASS_FLOOR_INFERRED = 7

CLASS_NAMES = {
    CLASS_INVALID: 'invalid', CLASS_SELF: 'self', CLASS_OBJECT: 'object',
    CLASS_FLOOR: 'floor', CLASS_REFLECTION: 'reflection', CLASS_CEILING: 'ceiling',
    CLASS_WALL: 'wall', CLASS_FLOOR_INFERRED: 'floor_inferred',
}

#: The highest class value, for anything sizing an array by class (class_stats'
#: bincount, a legend, a per-class loop). Deliberately derived rather than
#: written out: adding a class and forgetting one of those is a silent
#: truncation, which is how a new class disappears from the stats it was added
#: to produce.
CLASS_MAX = max(CLASS_NAMES)

#: BGR overlay colours, for the debug image only.
CLASS_COLOURS = {
    CLASS_INVALID: (40, 40, 40), CLASS_SELF: (0, 0, 255), CLASS_OBJECT: (0, 165, 255),
    CLASS_FLOOR: (0, 160, 0), CLASS_REFLECTION: (200, 0, 200),
    CLASS_CEILING: (160, 120, 0), CLASS_WALL: (255, 255, 0),
    # Another shade of green: it IS floor, and the eye should group it with
    # FLOOR at a glance while still being able to tell the two apart.
    CLASS_FLOOR_INFERRED: (90, 230, 120),
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
    counts = np.bincount(lab.ravel(), minlength=CLASS_MAX + 1)
    out = {'n_px': n, 'frac': {CLASS_NAMES[i]: (round(float(counts[i]) / n, 5) if n else 0.0)
                               for i in CLASS_NAMES}}
    out['count'] = {CLASS_NAMES[i]: int(counts[i]) for i in CLASS_NAMES}
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


def scan_to_base(ranges, angle_min, angle_increment, range_min, range_max,
                 rotation, translation):
    """Return the (N, 3) base-frame points of a scan's valid returns and their (N,) ranges.

    `rotation`/`translation` take laser-frame points into the base frame. The
    scan plane is a plane in base_link, so the z of every return is the
    laser's own mount height; the cross-check only ever uses x and y, and the
    overlay wants the real height to land on the right rows. One definition
    for front_clearance_node's live overlay and the run report's offline one
    (f1tenth_logger.mission_overlay).
    """
    r = np.asarray(ranges, dtype=float)
    a = angle_min + np.arange(r.size) * angle_increment
    with np.errstate(invalid='ignore'):
        ok = np.isfinite(r) & (r >= range_min) & (r <= range_max)
    r, a = r[ok], a[ok]
    local = np.column_stack([r * np.cos(a), r * np.sin(a), np.zeros(r.size)])
    pts = local @ np.asarray(rotation, dtype=float).T + np.asarray(translation, dtype=float)
    return pts, r


#: The classes a run report tints (f1tenth_logger.mission_overlay): the
#: surfaces every wall and floor decision is made from, and the objects.
#: INVALID, SELF, REFLECTION and CEILING stay clear, so the camera image reads
#: through them.
REPORT_CLASSES = (CLASS_WALL, CLASS_FLOOR, CLASS_FLOOR_INFERRED, CLASS_OBJECT)

#: BGR ramp of LiDAR points coloured by range (range_colours), near to far:
#: red, yellow, white. None of the three is a class colour, so a point stays
#: readable on every fill.
RANGE_RAMP = ((0, 0, 255), (0, 255, 255), (255, 255, 255))

#: No outline round a tinted region smaller than this fraction of the frame:
#: the stereo mask is speckled, and a ring round every speckle hides the image
#: instead of outlining a surface.
OUTLINE_MIN_FRAC = 0.002


def range_colours(ranges, range_max_m):
    """Return (N, 3) uint8 BGR of finite ranges on RANGE_RAMP over [0, range_max_m], clamped."""
    ramp = np.asarray(RANGE_RAMP, dtype=float)
    t = np.clip(np.asarray(ranges, dtype=float).reshape(-1) / max(float(range_max_m), 1e-6),
                0.0, 1.0) * (len(ramp) - 1)
    i = np.minimum(t.astype(int), len(ramp) - 2)
    f = (t - i)[:, None]
    return np.round(ramp[i] * (1.0 - f) + ramp[i + 1] * f).astype(np.uint8)


def overlay(image_bgr, labels, scan_px=None, alpha=0.5, classes=None, outline_px=0,
            scan_colours=None, point_px=1, legend_range_m=None):
    """The class-coloured debug overlay, with the projected /scan drawn on top.

    `scan_px` is an (M, 2) array of (u, v) pixel coordinates of the LiDAR
    returns that fall in the frame; they are drawn last so the comparison the
    cross-check makes is visible by eye. `labels` is resized (nearest) to the
    image when the two differ, and None draws the scan alone.

    The defaults are the live debug image's (front_clearance_node,
    /perception/scene_labels/image): every class tinted over the whole frame,
    white dots. A run report (f1tenth_logger.mission_overlay) draws with this
    same function and asks for less ink:

      classes         tint only these (REPORT_CLASSES); every other pixel
                      stays exactly as the camera saw it
      outline_px      > 0: a full-colour outline this thick round each tinted
                      region of at least OUTLINE_MIN_FRAC of the frame
      scan_colours    (M, 3) BGR per point (range_colours), drawn as square
                      dots point_px wide instead of the white circles; later
                      points cover earlier ones
      legend_range_m  the corner legend (draw_legend): the tinted classes and
                      the range scale from 0 to this range
    """
    import cv2  # local: keep the module importable where cv2 is absent
    img = np.ascontiguousarray(image_bgr)
    h, w = img.shape[:2]
    lab = None if labels is None else np.asarray(labels)
    if lab is not None and lab.shape[:2] != (h, w):
        lab = cv2.resize(lab, (w, h), interpolation=cv2.INTER_NEAREST)
    if lab is None:
        out = img.copy()
    elif classes is None:
        tint = np.zeros_like(img)
        for cls, colour in CLASS_COLOURS.items():
            tint[lab == cls] = colour
        out = cv2.addWeighted(img, 1.0 - alpha, tint, alpha, 0.0)
    else:
        out = _tint_classes(img, lab, classes, alpha, outline_px)
    if scan_px is not None and scan_colours is None:
        for u, v in np.asarray(scan_px, dtype=float).reshape(-1, 2):
            if np.isfinite(u) and np.isfinite(v):
                cv2.circle(out, (int(round(u)), int(round(v))), 1, (255, 255, 255), -1)
    elif scan_px is not None:
        _draw_dots(out, scan_px, scan_colours, point_px)
    if legend_range_m is not None:
        draw_legend(out, CLASS_COLOURS if classes is None else classes, legend_range_m)
    return out


def _tint_classes(img, lab, classes, alpha, outline_px):
    """Blend only the pixels of `classes`, then outline them (overlay's report path)."""
    import cv2  # local: keep the module importable where cv2 is absent
    out = img.copy()
    tint = np.zeros_like(img)
    sel = np.zeros(lab.shape[:2], dtype=bool)
    for cls in classes:
        hit = lab == cls
        tint[hit] = CLASS_COLOURS[cls]
        sel |= hit
    blend = cv2.addWeighted(img, 1.0 - alpha, tint, alpha, 0.0)
    out[sel] = blend[sel]
    if outline_px > 0:
        min_area = OUTLINE_MIN_FRAC * lab.shape[0] * lab.shape[1]
        for cls in classes:
            contours, _ = cv2.findContours((lab == cls).astype(np.uint8),
                                           cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            big = [c for c in contours if cv2.contourArea(c) >= min_area]
            if big:
                cv2.drawContours(out, big, -1, CLASS_COLOURS[cls], int(outline_px))
    return out


def _draw_dots(out, scan_px, colours, size):
    """Draw square dots `size` px wide at (u, v), one colour each, clipped to the image."""
    uv = np.asarray(scan_px, dtype=float).reshape(-1, 2)
    col = np.asarray(colours, dtype=np.uint8).reshape(-1, 3)
    ok = np.isfinite(uv).all(axis=1)
    u = np.round(uv[ok, 0]).astype(int)
    v = np.round(uv[ok, 1]).astype(int)
    col = col[ok]
    h, w = out.shape[:2]
    s = max(1, int(size))
    lo = -(s // 2)
    for dy in range(lo, lo + s):
        for dx in range(lo, lo + s):
            x, y = u + dx, v + dy
            inside = (x >= 0) & (x < w) & (y >= 0) & (y < h)
            out[y[inside], x[inside]] = col[inside]


def draw_legend(img, classes, range_max_m):
    """Draw the report's corner legend in place: a swatch per class, the range scale.

    Top-left, on a half-transparent dark box, in type a few pixels high, and
    sized from the image width so it stays a corner at any resolution (a
    480 px frame, the debug image's width, is the reference).
    """
    import cv2  # local: keep the module importable where cv2 is absent
    h, w = img.shape[:2]
    k = min(1.5, max(0.6, w / 480.0))
    font, scale = cv2.FONT_HERSHEY_SIMPLEX, 0.3 * k
    row, sw, pad = int(round(10 * k)), int(round(7 * k)), int(round(3 * k))
    bar_w = 3 * sw
    lines = [(cls, CLASS_NAMES[cls].replace('_', ' ')) for cls in classes]
    lines.append((None, 'lidar 0-{:g} m'.format(float(range_max_m))))
    text_w = max(cv2.getTextSize(text, font, scale, 1)[0][0] for _, text in lines)
    box_w = min(w, pad * 3 + bar_w + text_w)
    box_h = min(h, pad * 2 + row * len(lines))
    x0 = y0 = pad
    x1, y1 = min(w, x0 + box_w), min(h, y0 + box_h)
    img[y0:y1, x0:x1] = (img[y0:y1, x0:x1] * 0.45).astype(img.dtype)
    for i, (cls, text) in enumerate(lines):
        top = y0 + pad + i * row
        mid = top + row // 2
        if cls is None:
            bar = img[mid - 1:mid + 2, x0 + pad:x0 + pad + bar_w]
            ramp = range_colours(np.linspace(0.0, range_max_m, bar_w), range_max_m)
            bar[:] = ramp[None, :bar.shape[1], :]
        else:
            img[mid - sw // 2:mid - sw // 2 + sw, x0 + pad:x0 + pad + sw] = CLASS_COLOURS[cls]
        cv2.putText(img, text, (x0 + pad * 2 + bar_w, mid + int(round(3 * k))), font, scale,
                    (235, 235, 235), 1, cv2.LINE_AA)
