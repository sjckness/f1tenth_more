"""
door_detection.py: open doorways in the multi-wall tracker's walls, from /scan.

Pure: numpy and this package's own modules, no rclpy. lidar_front_wall_node
hosts it behind door_detection_enabled, on the wall tracker's own cycle, and
publishes /perception/doors (f1tenth_messages/DoorArray) and /viz/doors.
PERCEPTION ONLY: nothing in the control path, the costmap, the BT or the
missions reads either topic.

WHAT A DOOR IS HERE
-------------------
An OPEN doorway: a gap between two pieces of one wall line that the LiDAR
sees through. A candidate must satisfy ALL of (door_candidates):

  1. COLLINEAR. Both pieces lie on wall tracks that are confirmed or coasting
     (never tentative), and when they are two different tracks those are the
     same line within collinear_angle_rad and collinear_offset_m, measured at
     the facing ends. A corner is not a doorway.
  2. WIDTH. Jamb to jamb in [width_min_m, width_max_m].
  3. THROUGH THE GAP. Every beam whose ray crosses the gap returns at least
     behind_margin_m BEYOND the wall line, or not at all. A return ON the line
     (within wall_tol_m, the tracker's negative_tol_m) is a pane or a dark
     panel the LiDAR did see: not a door. A return between wall_tol_m and
     behind_margin_m beyond is too shallow to call. And at least
     n_required beams must cross it (BEAMS, below), or it is undecidable.
  4. REAL JAMBS. A beam just inside the gap at either end that returns SHORT
     of the line means the wall end is a shadow cast by something in front
     of it -- a box, a bin -- not a jamb.
  5. OBJECTS. A beam cut out for a YOLO detection (scan_in_base_link's cut
     mask) has no opinion, so a gap holding one is undecidable: a detected
     object can never create or confirm a door. A PERSON inside an already
     CONFIRMED door is the opposite case and is DoorTracker's business: it
     sets `occupied` and keeps the door alive.
  6. STEREO, optional. stereo_gap_support: no WALL pixels at the line's depth
     inside the gap's image rectangle, and floor (or inferred floor)
     continuing beyond the line. It raises the confidence; it is never
     required, so a LiDAR-only door is a door.

Closed doors are wall to the LiDAR and are out of scope; so are double doors
(they fail the width).

WHY THE GAP IS FOUND ON THE BEAMS, NOT BETWEEN TRACK ENDPOINTS
--------------------------------------------------------------
The extraction's collinear merge (merge_max_gap_m, 1.5 m) exists to reunify a
wall broken by an occluder, and it does exactly that to the two faces either
side of a 0.7-1.0 m doorway: they reach the tracker as ONE observation, so a
doorway in a wall is usually INSIDE one track's extent rather than between
two tracks. The tracker's own collinear gap (assoc_max_gap_m) keeps two
tracks apart only across an opening wider than 0.5 m that the beams saw open.
Both cases happen, so the gap is found the same way in both: the tracks give
the LINE and the stretch of it to look at, and the beams along that stretch
give the pieces of wall (runs of ON returns at least jamb_min_length_m long)
and the gaps between them. A piece's end is the last ON return; a jamb is
that return's foot on its wall's line. Each jamb must lie on (within
jamb_extent_tol_m of) a tracked wall's extent: the door is built on the
tracker's walls, not on raw returns.

BEAMS
-----
n_required = max(min_beams, ceil(min_beam_fraction * n_expected)), where
n_expected is the gap's angular width seen from the laser over the scan's
angle increment: what the geometry promises at that range and incidence.
The floor makes a far or grazing gap undecidable (at 0.25 deg a 0.85 m gap
square-on has 8 beams at 24 m, at 60 deg incidence at 12 m); the fraction
makes a near gap be crossed by nearly all the beams it should be, so beams
dropped as grazing cannot be made up for by a handful that were not.

IDENTITY (DoorTracker)
----------------------
Wall track ids re-spawn often (a merge, a re-birth after a coast), so a door's
identity never depends on them: doors are associated across cycles by
centre, normal and width in the tracker's fixed frame (odom, the frame every
wall track lives in), and the parent ids are only reported.
"""

import collections
from dataclasses import dataclass, field
import math

from f1tenth_perception import multi_wall_tracker as mwt
from f1tenth_perception import scene_mask
from f1tenth_perception import wall_fusion

import numpy as np

STATE_TENTATIVE = 0
STATE_CONFIRMED = 1
STATE_COASTING = 2
STATE_NAMES = {STATE_TENTATIVE: 'tentative', STATE_CONFIRMED: 'confirmed',
               STATE_COASTING: 'coasting'}

#: What one beam says about the wall line it crosses (classify_beams).
BEAM_ON = 0        # within wall_tol_m of the line: the wall
BEAM_BEYOND = 1    # at least behind_margin_m past it: through an opening
BEAM_NONE = 2      # no return at all (and not cut): through, as far as it knows
BEAM_SHORT = 3     # more than wall_tol_m short of it: something in front
BEAM_SHALLOW = 4   # past the line, but by less than behind_margin_m
BEAM_CUT = 5       # cut out for a YOLO detection: no opinion

#: The reject codes door_candidates reports, in the order they are checked.
REJECT_CODES = ('not_collinear', 'untracked_jamb', 'width', 'on_line',
                'jamb_occluded', 'blocked', 'object', 'shallow', 'few_beams')


@dataclass
class DoorParams:
    """The door detector's and tracker's numbers. See the module docstring."""

    # --- the candidate (door_candidates) -----------------------------------
    collinear_angle_rad: float = math.radians(5.0)
    collinear_offset_m: float = 0.10
    width_min_m: float = 0.70
    width_max_m: float = 1.00
    behind_margin_m: float = 0.30
    #: ON the line: the tracker's own negative_tol_m, so a beam the negative
    #: evidence would call a hit on the wall is a hit here too.
    wall_tol_m: float = 0.15
    min_beams: int = 8
    min_beam_fraction: float = 0.8
    #: A run of ON returns shorter than this is not a piece of wall: one
    #: stray return inside an opening is an ON return in the gap, not a jamb.
    jamb_min_length_m: float = 0.10
    #: How far past a tracked wall's measured extent a jamb may lie.
    jamb_extent_tol_m: float = 0.30
    #: |cos| between a beam and the line's normal below which the beam runs
    #: along the line and its crossing point is meaningless.
    min_incidence: float = 0.05
    #: Beams at each end of the gap whose SHORT return marks a shadow edge.
    edge_beams: int = 2
    # --- the tracker (DoorTracker) -----------------------------------------
    confirm_hits: int = 3
    confirm_window: int = 5
    delete_after_s: float = 2.0
    assoc_dist_m: float = 0.30
    assoc_angle_rad: float = math.radians(15.0)
    assoc_width_m: float = 0.20
    #: Weight of a new observation in the jambs' running average.
    smoothing: float = 0.3
    #: Confidence kept per cycle a door goes unseen.
    coast_confidence_decay: float = 0.9
    person_classes: tuple = ('person',)
    #: A person counts as IN a door within this far either side of its line.
    person_depth_m: float = 0.6
    # --- stereo support (stereo_gap_support) --------------------------------
    stereo_max_wall_share: float = 0.2
    stereo_min_floor_share: float = 0.5
    stereo_min_coverage: float = 0.25
    stereo_max_range_m: float = 8.0
    stereo_tol_base_m: float = 0.05
    stereo_tol_quad_per_m2: float = 0.012
    stereo_bonus: float = 0.2


@dataclass
class DoorCandidate:
    """One cycle's doorway, base_link at the scan stamp."""

    jamb0: np.ndarray
    jamb1: np.ndarray
    #: Unit normal of the wall line, pointing AWAY from the laser: the
    #: direction the car would cross the doorway in.
    normal: np.ndarray
    parents: tuple
    n_beams: int
    n_required: int
    stereo_support: object = None    # True / False, or None: not evaluated
    confidence: float = 0.0

    @property
    def centre(self):
        return 0.5 * (self.jamb0 + self.jamb1)

    @property
    def width(self):
        return float(np.hypot(*(self.jamb1 - self.jamb0)))


@dataclass
class _Wall:
    track_id: int
    a: np.ndarray
    b: np.ndarray
    n: np.ndarray
    d: float


def _wall_bl(trk, car_pose):
    """One track as base_link endpoints and its line n . x = d."""
    a, b = (np.asarray(p, dtype=float) for p in mwt.segment_in_base_link(trk, car_pose))
    th = trk.theta - car_pose.theta
    n = np.array([math.cos(th), math.sin(th)])
    return _Wall(int(trk.id), a, b, n, float(n @ (0.5 * (a + b))))


def eligible(trk):
    """Whether a wall track may carry a door: confirmed or coasting, never tentative."""
    return bool(trk.ever_confirmed) and trk.state in (mwt.STATE_CONFIRMED,
                                                      mwt.STATE_COASTING)


def _facing_ends(wa, wb):
    """(pa, pb): the endpoint of each wall nearest the other wall's endpoints."""
    best = None
    for pa in (wa.a, wa.b):
        for pb in (wb.a, wb.b):
            dist = math.hypot(pa[0] - pb[0], pa[1] - pb[1])
            if best is None or dist < best[0]:
                best = (dist, pa, pb)
    return best[1], best[2]


def collinear(wa, wb, p):
    """
    Return (ok, angle_rad, offset_m) for two base_link walls.

    The angle is between the lines, unoriented. The offset is how far each
    wall's FACING end lies off the other wall's line, the larger of the two:
    two pieces either side of a doorway line up exactly there.
    """
    angle = math.acos(min(1.0, abs(float(wa.n @ wb.n))))
    pa, pb = _facing_ends(wa, wb)
    offset = max(abs(float(wa.n @ pb) - wa.d), abs(float(wb.n @ pa) - wb.d))
    ok = angle <= p.collinear_angle_rad and offset <= p.collinear_offset_m
    return ok, angle, offset


def _along_gap(wa, wb):
    """Gap [m] between two walls' extents along wa's tangent; negative overlap."""
    u = np.array([-wa.n[1], wa.n[0]])
    a = sorted((float(u @ wa.a), float(u @ wa.b)))
    b = sorted((float(u @ wb.a), float(u @ wb.b)))
    return max(b[0] - a[1], a[0] - b[1])


def _groups(walls, p, rejects):
    """Union-find of walls that are one line with at most a doorway between them."""
    parent = list(range(len(walls)))

    def find(i):
        while parent[i] != i:
            parent[i] = parent[parent[i]]
            i = parent[i]
        return i

    reach = p.width_max_m + p.jamb_extent_tol_m
    for i in range(len(walls)):
        for j in range(i + 1, len(walls)):
            wa, wb = walls[i], walls[j]
            ok, _angle, _offset = collinear(wa, wb, p)
            gap = _along_gap(wa, wb)
            if ok and gap <= reach:
                parent[find(j)] = find(i)
                continue
            if ok and gap <= 2.0 * p.width_max_m:
                # One line, but too far apart to hold a single doorway.
                _reject(rejects, 'width', (wa.track_id, wb.track_id), gap)
                continue
            if not ok:
                pa, pb = _facing_ends(wa, wb)
                dist = float(np.hypot(*(pa - pb)))
                if 0.5 * p.width_min_m <= dist <= reach:
                    _reject(rejects, 'not_collinear', (wa.track_id, wb.track_id), dist)
    out = collections.defaultdict(list)
    for i, w in enumerate(walls):
        out[find(i)].append(w)
    return list(out.values())


_MAX_REJECTS = 16


def _reject(rejects, code, parents, width):
    if rejects is not None and len(rejects) < _MAX_REJECTS:
        rejects.append((code, tuple(int(x) for x in parents), round(float(width), 3)))


def classify_beams(r, cut, s, p):
    """BEAM_* per beam for returns r on rays meeting the wall line at s."""
    r = np.asarray(r, dtype=float)
    s = np.asarray(s, dtype=float)
    out = np.full(r.shape, BEAM_SHALLOW, dtype=np.int8)
    finite = np.isfinite(r)
    with np.errstate(invalid='ignore'):
        diff = np.where(finite, r - s, np.inf)
    out[finite & (np.abs(diff) <= p.wall_tol_m)] = BEAM_ON
    out[finite & (diff >= p.behind_margin_m)] = BEAM_BEYOND
    out[finite & (diff < -p.wall_tol_m)] = BEAM_SHORT
    out[~finite] = BEAM_NONE
    out[np.asarray(cut, dtype=bool)] = BEAM_CUT
    return out


def _angle_increment(bearings):
    b = np.asarray(bearings, dtype=float)
    if b.size < 2:
        return math.nan
    return float(np.median(np.abs(np.diff(b))))


def _foot(pt, wall):
    """Project pt onto the wall's line."""
    return pt - (float(wall.n @ pt) - wall.d) * wall.n


def _group_candidates(group, beams, inc, p, rejects):
    """
    Return the doorways along one collinear group of walls (module docstring, BEAMS).

    `beams` is (ranges, unit ray directions (N, 2), laser xy, cut), computed
    once per cycle for every group.
    """
    r, v, lxy, cut_all = beams
    ref = max(group, key=lambda w: float(np.hypot(*(w.b - w.a))))
    # Every member's line with its normal on the same side as the reference,
    # so the crossing distances below are the same quantity for all of them.
    members = []
    for w in group:
        if float(w.n @ ref.n) < 0.0:
            w = _Wall(w.track_id, w.a, w.b, -w.n, -w.d)
        members.append(w)
    n, d = ref.n, ref.d
    u = np.array([-n[1], n[0]])
    spans = np.array([sorted((float(u @ w.a), float(u @ w.b))) for w in members])
    if float(spans[:, 1].max() - spans[:, 0].min()) < p.width_min_m + 2.0 * p.jamb_min_length_m:
        return []   # too short to hold a doorway and a piece of wall either side
    t_lo = float(spans[:, 0].min()) - p.jamb_extent_tol_m
    t_hi = float(spans[:, 1].max()) + p.jamb_extent_tol_m

    nv = v @ n
    with np.errstate(divide='ignore', invalid='ignore'):
        s_ref = (d - float(n @ lxy)) / nv
        t_beam = float(u @ lxy) + s_ref * (v @ u)
        sel = (np.abs(nv) > p.min_incidence) & (s_ref > 0.05) & \
            (t_beam >= t_lo) & (t_beam <= t_hi)
    idx = np.flatnonzero(sel)
    if idx.size == 0:
        return []
    idx = idx[np.argsort(t_beam[idx])]
    tb = t_beam[idx]
    # Each beam is judged against the member whose extent it falls in (or
    # the nearest one): within the collinear tolerance the members are not
    # quite one line, and 5 deg over a long wall is more than wall_tol_m.
    dist = np.maximum(spans[None, :, 0] - tb[:, None], tb[:, None] - spans[None, :, 1])
    k = np.argmin(np.maximum(dist, 0.0), axis=1)
    mem_n = np.array([w.n for w in members])[k]
    mem_d = np.array([w.d for w in members])[k]
    vk = v[idx]
    inc_k = np.sum(vk * mem_n, axis=1)
    with np.errstate(divide='ignore', invalid='ignore'):
        s = (mem_d - np.sum(mem_n * lxy, axis=1)) / inc_k
    usable = np.abs(inc_k) > p.min_incidence
    idx, tb, k, vk, s = idx[usable], tb[usable], k[usable], vk[usable], s[usable]
    if idx.size == 0:
        return []
    cls = classify_beams(r[idx], cut_all[idx], s, p)
    with np.errstate(invalid='ignore'):
        pts = lxy + np.where(np.isfinite(r[idx]), r[idx], 0.0)[:, None] * vk

    # The pieces of wall: maximal runs of ON beams, long enough to be wall.
    edges = np.flatnonzero(np.diff(np.concatenate(
        [[0], (cls == BEAM_ON).astype(np.int8), [0]])))
    starts, ends = edges[0::2], edges[1::2] - 1
    length = np.abs((pts[ends] - pts[starts]) @ u)
    keep = (ends > starts) & (length >= p.jamb_min_length_m)
    pieces = list(zip(starts[keep].tolist(), ends[keep].tolist()))

    out = []
    for (_a0, a1), (b0, _b1) in zip(pieces, pieces[1:]):
        wa, wb = members[k[a1]], members[k[b0]]
        j0, j1 = _foot(pts[a1], wa), _foot(pts[b0], wb)
        width = float(np.hypot(*(j1 - j0)))
        parents = (wa.track_id, wb.track_id)
        if width < 0.5 * p.width_min_m:
            # A dropout between two returns of one wall: not worth a reject.
            continue
        code, n_open, n_req = _judge_gap(
            cls[a1 + 1:b0], (j0, j1), (spans[k[a1]], spans[k[b0]]), u, (wa, wb), lxy, inc, p)
        if code:
            _reject(rejects, code, parents, width)
            continue
        centre = 0.5 * (j0 + j1)
        normal = n if float(n @ (centre - lxy)) >= 0.0 else -n
        out.append(DoorCandidate(j0, j1, normal.copy(), parents, n_open, n_req))
    return out


def _judge_gap(gap, jambs, spans, u, walls, lxy, inc, p):
    """
    Return (reject code or '', open beams, required beams) for one gap.

    `gap` is the BEAM_* classes strictly between the two pieces, in order;
    `jambs`, `spans` and `walls` are the two jambs, their walls' extents along
    `u` and the walls themselves.
    """
    (j0, j1), (wa, wb) = jambs, walls
    for jamb, (lo, hi) in zip(jambs, spans):
        t = float(u @ jamb)
        if not (lo - p.jamb_extent_tol_m <= t <= hi + p.jamb_extent_tol_m):
            return 'untracked_jamb', 0, 0
    if wa.track_id != wb.track_id and not collinear(wa, wb, p)[0]:
        return 'not_collinear', 0, 0
    width = float(np.hypot(*(j1 - j0)))
    if not (p.width_min_m <= width <= p.width_max_m):
        return 'width', 0, 0
    if np.any(gap == BEAM_ON):
        return 'on_line', 0, 0
    edge = max(1, int(p.edge_beams))
    if np.any(gap[:edge] == BEAM_SHORT) or np.any(gap[-edge:] == BEAM_SHORT):
        return 'jamb_occluded', 0, 0
    if np.any(gap == BEAM_SHORT):
        return 'blocked', 0, 0
    if np.any(gap == BEAM_CUT):
        return 'object', 0, 0
    if np.any(gap == BEAM_SHALLOW):
        return 'shallow', 0, 0
    n_open = int(np.count_nonzero((gap == BEAM_BEYOND) | (gap == BEAM_NONE)))
    va, vb = j0 - lxy, j1 - lxy
    span = abs(math.atan2(float(va[0] * vb[1] - va[1] * vb[0]), float(va @ vb)))
    n_expected = span / inc if math.isfinite(inc) and inc > 0.0 else 0.0
    n_req = max(int(p.min_beams), int(math.ceil(p.min_beam_fraction * n_expected)))
    if n_open < n_req:
        return 'few_beams', n_open, n_req
    return '', n_open, n_req


def door_candidates(tracks, car_pose, scan, p=None, rejects=None):
    """
    Return every doorway of this cycle, base_link at the scan stamp.

    `tracks` are the wall tracker's tracks (all of them: the eligible ones
    are picked here). `scan` is (ranges, base_link bearings, laser xy, cut),
    exactly what MultiWallTracker.step was given. `rejects`, when a list,
    collects (code, (parent ids), width) for the gaps that failed, capped at
    16: a debug trail, not a log.
    """
    p = p or DoorParams()
    walls = [_wall_bl(t, car_pose) for t in tracks if eligible(t)]
    if not walls or scan is None:
        return []
    ranges, bearings = np.asarray(scan[0], dtype=float), np.asarray(scan[1], dtype=float)
    cut = (np.asarray(scan[3], dtype=bool) if len(scan) > 3 and scan[3] is not None
           else np.zeros(ranges.shape, dtype=bool))
    beams = (ranges, np.column_stack([np.cos(bearings), np.sin(bearings)]),
             np.asarray(scan[2], dtype=float).reshape(2), cut)
    inc = _angle_increment(bearings)
    out = []
    for group in _groups(walls, p, rejects):
        out += _group_candidates(group, beams, inc, p, rejects)
    for cand in out:
        cand.confidence = lidar_confidence(cand)
    return out


def lidar_confidence(cand):
    """
    Return the LiDAR-only confidence in [0.5, 0.8].

    0.5 for a gap crossed by exactly the beams it needed, rising to 0.8 at
    twice that. The stereo can add stereo_bonus on top (DoorTracker).
    """
    if cand.n_required <= 0:
        return 0.5
    extra = (cand.n_beams - cand.n_required) / float(cand.n_required)
    return 0.5 + 0.3 * min(1.0, max(0.0, extra))


# ============================================================================
# Stereo support (optional)
# ============================================================================

def stereo_gap_support(cand, labels, depth, fx, fy, cx, cy, rotation, translation,
                       p=None, depth_scale=1.0):
    """
    Return (support, detail) of the class mask and depth for one base_link candidate.

    support is None when the camera cannot say (gap out of the image, out of
    the stereo's range, too few informative samples), else True when no WALL
    pixel sits at the line's depth inside the gap's rectangle (share at most
    stereo_max_wall_share) AND the floor -- or the inferred floor -- carries
    on beyond the line (share at least stereo_min_floor_share). `rotation` /
    `translation` take base-frame points into the optical frame the way
    scene_mask.project_to_image does.
    """
    p = p or DoorParams()
    detail = {'coverage': 0.0, 'wall_share': math.nan, 'floor_share': math.nan}
    j0, j1, normal = cand.jamb0, cand.jamb1, cand.normal
    width = cand.width
    if width <= 0.2:
        return None, detail
    tang = (j1 - j0) / width
    ts = np.arange(0.1, width - 0.1 + 1e-9, 0.05)
    zs = np.arange(0.3, 1.81, 0.1)
    xy = j0 + ts[:, None] * tang
    gap = np.column_stack([np.repeat(xy, zs.size, axis=0), np.tile(zs, ts.size)])
    beyond = np.array([[*(j0 + t * tang + k * normal), 0.0]
                       for t in ts for k in (0.3, 0.6, 0.9, 1.2)])
    lab = np.asarray(labels)
    dep = np.asarray(depth)
    h, w = lab.shape[:2]
    rot = np.asarray(rotation, dtype=float)
    trans = np.asarray(translation, dtype=float).reshape(3)

    def sample(points):
        uv = scene_mask.project_to_image(points, fx, fy, cx, cy, rot, trans)
        exp = ((points - trans) @ rot)[:, 2]
        with np.errstate(invalid='ignore'):
            ok = (np.isfinite(uv[:, 0]) & (uv[:, 0] >= 0) & (uv[:, 0] < w)
                  & (uv[:, 1] >= 0) & (uv[:, 1] < h) & (exp <= p.stereo_max_range_m))
        ui = np.rint(uv[ok, 0]).clip(0, w - 1).astype(int)
        vi = np.rint(uv[ok, 1]).clip(0, h - 1).astype(int)
        raw = dep[vi, ui]
        if np.issubdtype(raw.dtype, np.integer):
            dz = raw.astype(float) * depth_scale
            dz[raw == 0] = np.nan
        else:
            dz = raw.astype(float) * depth_scale
        return lab[vi, ui], dz, exp[ok], int(np.count_nonzero(ok)), points.shape[0]

    cls, dz, exp, n_in, n_all = sample(gap)
    informative = (cls != scene_mask.CLASS_INVALID) & (cls != scene_mask.CLASS_SELF) & \
        (cls != scene_mask.CLASS_OBJECT)
    detail['coverage'] = float(np.count_nonzero(informative)) / max(n_all, 1)
    if n_in == 0 or detail['coverage'] < p.stereo_min_coverage:
        return None, detail
    tol = wall_fusion.tolerance(exp, p.stereo_tol_base_m, p.stereo_tol_quad_per_m2)
    with np.errstate(invalid='ignore'):
        wall_at = (cls == scene_mask.CLASS_WALL) & (np.abs(dz - exp) <= tol)
    detail['wall_share'] = float(np.count_nonzero(wall_at)) / max(
        int(np.count_nonzero(informative)), 1)
    fcls, _fdz, _fexp, f_in, _f_all = sample(beyond)
    if f_in == 0:
        return None, detail
    floor = (fcls == scene_mask.CLASS_FLOOR) | (fcls == scene_mask.CLASS_FLOOR_INFERRED)
    detail['floor_share'] = float(np.count_nonzero(floor)) / f_in
    support = (detail['wall_share'] <= p.stereo_max_wall_share
               and detail['floor_share'] >= p.stereo_min_floor_share)
    return bool(support), detail


# ============================================================================
# Tracking
# ============================================================================

def _perp(vec):
    return np.array([-vec[1], vec[0]])


@dataclass
class DoorTrack:
    """One door, in the tracker's fixed frame (odom)."""

    id: int  # noqa: A003 -- the wire name (Door.id), as Track.id is
    jamb0: np.ndarray
    jamb1: np.ndarray
    #: Unit, odom: the crossing direction, from the side the car first saw
    #: the door from. Kept perpendicular to jamb1 - jamb0.
    normal: np.ndarray
    born: float
    #: Last cycle the LiDAR saw the door as a candidate.
    last_seen: float
    #: Last cycle a person inside it kept it alive (occupied).
    last_held: float = -math.inf
    state: int = STATE_TENTATIVE
    hits: int = 1
    history: collections.deque = field(default_factory=lambda: collections.deque(maxlen=5))
    ever_confirmed: bool = False
    occupied: bool = False
    confidence: float = 0.0
    stereo_support: bool = False
    parents: tuple = (0, 0)
    n_beams: int = 0

    @property
    def centre(self):
        return 0.5 * (self.jamb0 + self.jamb1)

    @property
    def width(self):
        return float(np.hypot(*(self.jamb1 - self.jamb0)))


def _unoriented_angle(a, b):
    return math.acos(min(1.0, abs(float(np.asarray(a) @ np.asarray(b)))))


class DoorTracker:
    """Door tracks across cycles; see the module docstring, IDENTITY."""

    def __init__(self, params=None):
        self.p = params or DoorParams()
        self.doors = []
        self._next_id = 1
        #: (code, (parent ids), width) of the last cycle's failed gaps.
        self.rejects = []

    def _in_odom(self, cand, car_pose):
        j0, j1 = car_pose.transform_points(np.vstack([cand.jamb0, cand.jamb1]))
        normal = car_pose.R @ cand.normal
        return j0, j1, normal / max(float(np.hypot(*normal)), 1e-9)

    def _cost(self, door, j0, j1, normal):
        dc = float(np.hypot(*(door.centre - 0.5 * (j0 + j1))))
        da = _unoriented_angle(door.normal, normal)
        dw = abs(door.width - float(np.hypot(*(j1 - j0))))
        if dc > self.p.assoc_dist_m or da > self.p.assoc_angle_rad \
                or dw > self.p.assoc_width_m:
            return math.inf
        return dc / self.p.assoc_dist_m + da / self.p.assoc_angle_rad \
            + dw / self.p.assoc_width_m

    def person_inside(self, door, persons_odom):
        """Whether any (x, y, half width) person lies within the door's gap."""
        if not persons_odom:
            return False
        tang = door.jamb1 - door.jamb0
        width = float(np.hypot(*tang))
        if width <= 0.0:
            return False
        tang = tang / width
        for x, y, half in persons_odom:
            rel = np.array([x, y]) - door.jamb0
            along = float(tang @ rel)
            margin = max(float(half), 0.0)
            if (-margin <= along <= width + margin
                    and abs(float(door.normal @ rel)) <= self.p.person_depth_m):
                return True
        return False

    def step(self, candidates, car_pose, stamp, persons=()):
        """
        Fold one cycle's base_link candidates in; return every live door.

        `persons` are (x, y, half width) of this cycle's person detections in
        base_link at the scan stamp (class already filtered). They never
        create or confirm a door; inside a CONFIRMED door they set occupied
        and keep it alive.
        """
        p = self.p
        stamp = float(stamp)
        obs = [self._in_odom(c, car_pose) + (c,) for c in candidates]
        persons_odom = []
        if persons:
            pts = car_pose.transform_points(np.array([[x, y] for x, y, _ in persons]))
            persons_odom = [(float(q[0]), float(q[1]), float(h))
                            for q, (_x, _y, h) in zip(pts, persons)]
        pairs = []
        for i, door in enumerate(self.doors):
            for j, (j0, j1, normal, _c) in enumerate(obs):
                cost = self._cost(door, j0, j1, normal)
                if math.isfinite(cost):
                    pairs.append((cost, i, j))
        used_d, used_o = set(), set()
        for _cost, i, j in sorted(pairs):
            if i in used_d or j in used_o:
                continue
            used_d.add(i)
            used_o.add(j)
            self._update(self.doors[i], *obs[j], stamp)
        for i, door in enumerate(self.doors):
            if i in used_d:
                door.occupied = False
                continue
            door.history.append(False)
            door.occupied = door.ever_confirmed and self.person_inside(door, persons_odom)
            if door.occupied:
                door.last_held = stamp
                door.state = STATE_CONFIRMED
            else:
                door.state = STATE_COASTING if door.ever_confirmed else STATE_TENTATIVE
                door.confidence *= p.coast_confidence_decay
        for j, (j0, j1, normal, cand) in enumerate(obs):
            if j not in used_o:
                self._spawn(j0, j1, normal, cand, stamp)
        self._merge()
        self.doors = [d for d in self.doors if not self._expired(d, stamp)]
        return list(self.doors)

    def _update(self, door, j0, j1, normal, cand, stamp):
        a = self.p.smoothing
        # Jamb order by the door's own tangent, so a candidate seen from the
        # other side (or with its jambs listed the other way) averages onto
        # the right jamb.
        tang = _perp(door.normal)
        if float(tang @ (j1 - j0)) * float(tang @ (door.jamb1 - door.jamb0)) < 0.0:
            j0, j1 = j1, j0
        door.jamb0 = (1.0 - a) * door.jamb0 + a * j0
        door.jamb1 = (1.0 - a) * door.jamb1 + a * j1
        new_normal = _perp(door.jamb1 - door.jamb0)
        norm = float(np.hypot(*new_normal))
        if norm > 1e-9:
            new_normal = new_normal / norm
            door.normal = new_normal if float(new_normal @ door.normal) >= 0.0 \
                else -new_normal
        door.hits += 1
        door.history.append(True)
        door.last_seen = stamp
        door.parents = tuple(int(x) for x in cand.parents)
        door.n_beams = int(cand.n_beams)
        door.confidence = (1.0 - a) * door.confidence + a * self._confidence(cand)
        if cand.stereo_support is not None:
            door.stereo_support = bool(cand.stereo_support)
        if sum(door.history) >= self.p.confirm_hits:
            door.ever_confirmed = True
        door.state = STATE_CONFIRMED if door.ever_confirmed else STATE_TENTATIVE

    def _confidence(self, cand):
        bonus = self.p.stereo_bonus if cand.stereo_support else 0.0
        return min(1.0, float(cand.confidence) + bonus)

    def _spawn(self, j0, j1, normal, cand, stamp):
        door = DoorTrack(self._next_id, j0.copy(), j1.copy(), normal.copy(), stamp, stamp,
                         parents=tuple(int(x) for x in cand.parents),
                         n_beams=int(cand.n_beams))
        door.history = collections.deque([True], maxlen=max(1, int(self.p.confirm_window)))
        door.confidence = self._confidence(cand)
        door.stereo_support = bool(cand.stereo_support)
        if self.p.confirm_hits <= 1:
            door.ever_confirmed, door.state = True, STATE_CONFIRMED
        self._next_id += 1
        self.doors.append(door)

    def _merge(self):
        """Fold any two doors that associate with each other into the older one."""
        keep = []
        for door in sorted(self.doors, key=lambda d: d.id):
            if any(math.isfinite(self._cost(k, door.jamb0, door.jamb1, door.normal))
                   for k in keep):
                continue
            keep.append(door)
        self.doors = keep

    def _expired(self, door, stamp):
        if stamp - max(door.last_seen, door.last_held) > self.p.delete_after_s:
            return True
        window = max(1, int(self.p.confirm_window))
        return (not door.ever_confirmed and len(door.history) >= window
                and sum(door.history) < self.p.confirm_hits)
