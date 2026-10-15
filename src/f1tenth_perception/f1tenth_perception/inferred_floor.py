"""inferred_floor.py -- the floor the LiDAR can see and the stereo cannot.

Pure: numpy only, no rclpy. The ROS glue is front_clearance_node's scene-mask
tick, right after scene_mask.classify.

WHY. On this building's floor the stereo has nothing to match: in the lower
centre quarter of the frame -- the floor straight ahead, the one region a
"what is in front of me" question is actually about -- 74 % of pixels are
INVALID and 38 % of the rest are REFLECTION (corridor10m_20261015T171753,
2026-10-15; see the replay's own `scene mask classes` table). FLOOR is 1.1 %
there. So the class that says "free space" is almost never assigned where the
free space is, and INVALID means two completely different things in one image:
"nothing matched on the open floor" and "something is there that I cannot
measure".

The LiDAR can tell those apart. It sees the same ground plane from 0.19 m up,
and a beam that runs 20 m down an empty corridor is positive evidence that the
patch of floor 2 m ahead is empty floor -- the stereo's failure to match it is
a stereo problem, not an obstacle. So: for every pixel with no usable depth,
ask where its ray MEETS THE FLOOR, find the beam that covers that bearing, and
if the beam went past that point the pixel is floor.

floor_inferred IS ITS OWN CLASS, NOT FLOOR. It is an inference from a
different sensor at a different height, through two extrinsics, and it must
never be mistaken for a measurement:

  * it is excluded from every plane and pitch fit (those exist to MEASURE the
    extrinsic this inference already assumes -- feeding it back would be
    circular, and it would fit the assumption perfectly);
  * it is never wall evidence (only CLASS_WALL pixels ever are);
  * it carries its own colour, its own legend entry and its own percentage in
    the debug video and in /perception/scene_labels/stats, so a run can be
    read as "this much floor was seen and this much was inferred".

Wherever FLOOR means free space, floor_inferred means free space too.

THE LUT. The ray -> floor -> beam geometry depends only on the intrinsics, the
mask resolution and the two extrinsics, all of which are static, so it is
computed ONCE (`build_lut`) and recomputed only when one of them changes. Per
pixel it stores the beam index of the floor point's bearing SEEN FROM THE
LASER (not from base_link: the beams leave from the laser, 0.305 m ahead of
base_link, and at 2 m that difference is 9 degrees, 21 beams), the 2D range
from the laser to that floor point, and whether the pixel can be inferred at
all -- below the horizon, inside r_max, inside the LiDAR's own field of view,
and not SELF.

THE PER-CYCLE RANGE IS NOT THE RAW BEAM. Two corrections, both conservative:

  r_w     a Gaussian-weighted mean over +-window_k beams, so one spurious
          short return does not carve a hole in the inferred floor. A
          no-return (inf, NaN, or past range_max -- the driver's finite
          65.533 m code) counts as FAR here, because it is: nothing was hit.
  r_eff   = min(r_w, the minimum over the central +-1 beams). The smoothing
          alone would let the floor leak past a wall or object edge, where the
          window straddles the boundary and averages a 1 m return with a 20 m
          one. The central minimum pins the result to what the beams actually
          nearest this bearing saw. r_eff <= the smoothed value always, so
          every correction here can only REFUSE to infer floor, never invent
          it.

A return BELOW range_min is not "far" and not a measurement either; it is
treated as zero range, which refuses the inference for that bearing.

STALENESS. The decision is made on the most recent scan. If that scan is more
than max_scan_age from the mask's own stamp the inference is SKIPPED for this
cycle and the classes stay exactly as classify left them: an old scan
describes a different place, and a wrong floor is worse than no floor.
"""

import numpy as np

from f1tenth_perception.scene_mask import (
    CLASS_FLOOR_INFERRED, CLASS_INVALID, CLASS_REFLECTION)

#: Classes a pixel may be inferred FROM. Everything else -- FLOOR, WALL,
#: CEILING, OBJECT, SELF -- is a decision already made on evidence this
#: inference does not improve on.
CANDIDATE_CLASSES = (CLASS_INVALID, CLASS_REFLECTION)


class FloorLut:
    """Per-pixel floor geometry for one (resolution, intrinsics, extrinsic, scan).

    The whole-image arrays are the readable form, for the tests and for
    anything inspecting the geometry:

    beam: (H, W) int32 index into the scan's ranges for the floor point's
          bearing, 0 where the pixel is not inferable (`valid` says which).
    r_floor: (H, W) float32 2D distance [m] from the LASER to that floor
          point, NaN where not inferable.
    valid: (H, W) bool, the lut_valid flag of the module docstring.

    `apply` works on the COMPACT form instead -- the same three quantities
    gathered down to the inferable pixels alone (flat_idx, flat_beam,
    flat_r_floor). Only about 40 % of a 640x360 frame is below the horizon and
    inside r_max, so every per-cycle gather and compare is on 95k values
    rather than 230k: 2.1 ms -> 0.6 ms per cycle on the Orin.

    key: what it was built for; `matches` compares a candidate against it.
    """

    __slots__ = ('beam', 'r_floor', 'valid', 'key', 'n_valid',
                 'flat_idx', 'flat_beam', 'flat_r_floor')

    def __init__(self, beam, r_floor, valid, key):
        self.beam = beam
        self.r_floor = r_floor
        self.valid = valid
        self.key = key
        self.n_valid = int(np.count_nonzero(valid))
        self.flat_idx = np.flatnonzero(valid.ravel())
        self.flat_beam = beam.ravel()[self.flat_idx]
        self.flat_r_floor = r_floor.ravel()[self.flat_idx].astype(float)

    def matches(self, key):
        return self.key == key


def lut_key(shape_hw, k, rotation, translation, laser_pose, scan_geometry, r_max):
    """A hashable identity for a LUT: everything build_lut's result depends on.

    The mask resolution, the intrinsics, both extrinsics, the scan's own
    angular layout and r_max. The self mask is deliberately NOT in it: it is
    static per resolution and already keyed by the shape.
    """
    return (tuple(int(v) for v in shape_hw), tuple(round(float(v), 6) for v in k),
            tuple(np.round(np.asarray(rotation, dtype=float).ravel(), 9)),
            tuple(np.round(np.asarray(translation, dtype=float).ravel(), 9)),
            tuple(round(float(v), 9) for v in laser_pose),
            tuple(round(float(v), 12) if isinstance(v, float) else int(v)
                  for v in scan_geometry),
            round(float(r_max), 6))


def build_lut(shape_hw, fx, fy, cx, cy, rotation, translation, laser_pose,
              angle_min, angle_increment, n_beams, r_max=3.0, self_mask=None,
              floor_z=0.0):
    """The ray -> floor -> beam lookup table (module docstring, THE LUT).

    `rotation`/`translation` take depth-frame points into the base frame, the
    same pair scene_mask.classify is given, so the mount pitch is already in
    them. `laser_pose` is (x, y, yaw) of the laser in the base frame.
    `floor_z` is the floor's height in that frame, which for this stack is 0:
    base_link sits on the ground (base.xacro).

    Everything is one vectorised pass over the pixel grid; nothing here is
    per-pixel Python.
    """
    h, w = int(shape_hw[0]), int(shape_hw[1])
    rot = np.asarray(rotation, dtype=float)
    trans = np.asarray(translation, dtype=float)

    # Ray direction of every pixel, in the base frame.
    ku = (np.arange(w, dtype=float) - cx) / fx
    kv = (np.arange(h, dtype=float) - cy) / fy
    dx = rot[0, 0] * ku[None, :] + rot[0, 1] * kv[:, None] + rot[0, 2]
    dy = rot[1, 0] * ku[None, :] + rot[1, 1] * kv[:, None] + rot[1, 2]
    dz = rot[2, 0] * ku[None, :] + rot[2, 1] * kv[:, None] + rot[2, 2]

    # Where that ray meets the floor plane z = floor_z. Below the horizon means
    # dz < 0 for a camera above the floor, and the hit must be ahead of the
    # lens (s > 0); both are the same condition, kept explicit.
    below = dz < -1e-9
    with np.errstate(divide='ignore', invalid='ignore'):
        s = (float(floor_z) - trans[2]) / dz
    hit = below & np.isfinite(s) & (s > 0.0)
    # Zero the ray parameter where there is no hit BEFORE using it: a pixel
    # exactly on the horizon has dz = 0, so s is +-inf, and inf * 0 on the
    # component that happens to be zero is a NaN and a numpy warning from
    # inside the scene-mask tick (which treats any warning raised as an error
    # as a failed tick).
    s = np.where(hit, s, 0.0)
    px = trans[0] + s * dx
    py = trans[1] + s * dy

    # The bearing and range of that floor point SEEN FROM THE LASER.
    lx, ly, lyaw = (float(laser_pose[0]), float(laser_pose[1]), float(laser_pose[2]))
    ex, ey = px - lx, py - ly
    r_floor = np.hypot(ex, ey)
    bearing = np.arctan2(ey, ex) - lyaw
    bearing = np.arctan2(np.sin(bearing), np.cos(bearing))

    inc = float(angle_increment)
    beam = np.zeros((h, w), dtype=np.int32)
    valid = hit & np.isfinite(r_floor) & (r_floor <= float(r_max))
    if inc != 0.0 and int(n_beams) > 0:
        idx = np.rint((bearing - float(angle_min)) / inc)
        in_fov = np.isfinite(idx) & (idx >= 0) & (idx <= int(n_beams) - 1)
        valid &= in_fov
        beam = np.where(valid, idx, 0).astype(np.int32)
    else:
        valid = np.zeros((h, w), dtype=bool)
    if self_mask is not None:
        valid &= ~np.asarray(self_mask, dtype=bool)
    return FloorLut(beam, np.where(valid, r_floor, np.nan).astype(np.float32), valid,
                    lut_key((h, w), (fx, fy, cx, cy), rot, trans, (lx, ly, lyaw),
                            (angle_min, inc, int(n_beams)), r_max))


def gaussian_beam_weights(window_k, sigma_beams):
    """Normalised Gaussian weights over offsets -window_k .. +window_k beams."""
    k = max(0, int(window_k))
    offsets = np.arange(-k, k + 1, dtype=float)
    sigma = float(sigma_beams)
    if sigma <= 0.0:
        w = (offsets == 0.0).astype(float)
    else:
        w = np.exp(-0.5 * (offsets / sigma) ** 2)
    return w / w.sum()


def effective_ranges(ranges, range_min, range_max, far_value, window_k=3,
                     sigma_beams=1.5):
    """The per-beam effective range r_eff (module docstring, THE PER-CYCLE RANGE).

    `far_value` is what "nothing was hit" counts as; the caller sets it from
    r_max and the margin, so a no-return always clears the floor test. A
    return below range_min becomes 0.0, which never does.
    """
    r = np.asarray(ranges, dtype=float)
    with np.errstate(invalid='ignore'):
        far = ~np.isfinite(r) | (r > float(range_max))
        near = np.isfinite(r) & (r < float(range_min))
    clipped = np.where(far, float(far_value), np.where(near, 0.0, r))
    clipped = np.nan_to_num(clipped, nan=float(far_value), posinf=float(far_value),
                            neginf=0.0)

    k = max(0, int(window_k))
    if k > 0 and clipped.size > 1:
        weights = gaussian_beam_weights(k, sigma_beams)
        padded = np.pad(clipped, k, mode='edge')
        smoothed = np.convolve(padded, weights, mode='valid')
        # The central +-1 minimum: the guard against the floor leaking past an
        # edge the smoothing window straddles.
        one = np.pad(clipped, 1, mode='edge')
        central = np.minimum(np.minimum(one[:-2], one[1:-1]), one[2:])
        return np.minimum(smoothed, central)
    return clipped


def apply(labels, lut, r_eff, margin=0.15, candidate_classes=CANDIDATE_CLASSES):
    """Relabel the inferable pixels floor_inferred. Returns (labels, n_applied).

    `labels` is modified in place and returned, the same discipline
    scene_mask.classify's own single-pass relabelling follows. One gather by
    beam index and one comparison, both over the LUT's inferable pixels only:
    no per-pixel loop and nothing touching the other 60 % of the frame.
    """
    lab = np.asarray(labels)
    if lut.flat_idx.size == 0:
        return lab, 0
    flat = lab.reshape(-1)
    here = flat[lut.flat_idx]
    cand = np.zeros(here.shape, dtype=bool)
    for cls in candidate_classes:
        cand |= here == cls
    if not cand.any():
        return lab, 0
    reach = np.asarray(r_eff, dtype=float)[lut.flat_beam]
    with np.errstate(invalid='ignore'):
        free = cand & (reach > lut.flat_r_floor + float(margin))
    n = int(np.count_nonzero(free))
    if n:
        flat[lut.flat_idx[free]] = CLASS_FLOOR_INFERRED
    return lab, n
