"""
door_detection.py: open doorways in the tracked walls and the door frames of /scan.

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
     the facing ends. A corner is not a doorway. Or they lie on a FRAME LINE
     (FRAME JAMBS, below).
  2. WIDTH. Jamb to jamb in [width_min_m, width_max_m].
  3. THROUGH THE GAP. Every beam whose ray crosses the central
     core_fraction of the gap returns at least behind_margin_m BEYOND the
     wall line, or not at all. A return ON the line (within wall_tol_m, the
     tracker's negative_tol_m, short of it; on_behind_tol_m, past it)
     anywhere in the gap is a pane or a dark panel the LiDAR did see: not a
     door. A return between on_behind_tol_m and behind_margin_m beyond is
     too shallow to call -- except in the OUTER
     parts, where it may be THE LEAF (below). And at least n_required beams
     must cross it (BEAMS, below), or it is undecidable.
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
     required, so a LiDAR-only door is a door. With stereo_glass_veto on
     (default OFF), stereo_glass_check can also REJECT a gap: WALL pixels
     whose valid depth puts them ON the line over at least
     stereo_wall_veto_frac of the gap ('stereo_wall' -- a pane the LiDAR saw
     through); a gap whose depth is mostly invalid is kept and flagged
     glass_suspect.

FRAME JAMBS (frame_jambs; scan_pieces)
--------------------------------------
A glass partition is never a tracked wall: the tracker births a wall at
1.0 m of returns, and the LiDAR sees a glass wall only as its frames --
mullions, door frames, posts, 4-60 cm of returns each -- plus the glass
itself where a beam meets it square-on (GLASS). So the scan's own short
straight pieces (scan_pieces: the extraction's breakpoints and split,
refined at the few-centimetre corner where a frame's face meets its side)
are walls here too, with parent id FRAME_ID (0). A piece on a tracked wall
is that wall (_uncovered); a piece as long as a track needs to be born is a
wall the tracker has not confirmed, never a frame. Pieces too short to have
a direction of their own join a line on where they lie (_collinear_short),
up to frame_line_max_gap_m apart; a line of frame pieces alone must hold
frame_min_pieces separate frames (_stations) and is refitted to every return
on it (_refine_line), the line its doors' normals come from. Along it the
doorways are found on the beams exactly as along a track (BEAMS), every
criterion above unchanged except that a jamb is any run of at least
frame_jamb_min_length_m of ON returns on the line -- a frame is a piece one
scan and three returns the next -- shorter than a wall.

GLASS: SEEN SQUARE-ON, and PANES (DoorTracker._glass_evidence)
-------------------------------------------------------------
Glass is invisible to the LiDAR obliquely -- the beams return from the room
behind it, exactly like an open doorway -- and returns them only within a
few degrees of its normal (corridor10m_20261016T144537, one pane between two
frames: ON on 100 / 94 / 51 / 19 / 0 % of its beams at 0-1 / 1-2 / 2-3 /
3-4 / 4-5 deg). Its panes are 1.05-1.15 m between frames there: every pane
is a door-wide gap. So:

  * SEEN SQUARE-ON. No door is confirmed before square_looks cycles on which
    THE FAN -- every beam within square_incidence_rad of the door's normal
    -- crossed its core, at least square_min_beams of them, all through, and
    none of the core's beams returned on its line (gap_evidence,
    is_square_look). The fan is wider than the pane's return zone by what a
    door's normal can be off the glass's, so a pane never passes; a doorway
    always does once the car drives past it or faces it. A door only ever
    seen obliquely stays TENTATIVE. Every door, on tracks or on frames: a
    tracked glass wall has panes too (door 8 of that bag).
  * PANES. A door not yet seen square-on whose core returns at least
    pane_min_on_beams ON its line on pane_min_cycles consecutive cycles is
    a pane: deleted, and remembered pane_memory_s after its glass was last
    seen ON; a candidate that associates with a remembered pane is dropped.
    A door once seen square-on is not glass: if it later returns on its
    line it has been closed or blocked, and simply goes unseen.

Both need the cycle's scan (DoorTracker.step(scan=...)). A closed glass DOOR
is a pane to the LiDAR and is never reported: closed doors are out of scope.

THE LEAF (open door leaf, is_leaf)
---------------------------------
An open leaf hinged at a jamb is seen through the doorway as a short surface
starting AT the jamb and running away from the wall line: near the hinge its
returns are less than behind_margin_m past the line. So the strict rule of 3
holds only in the central core_fraction of the gap; in each OUTER part the
shallow returns are allowed when they all lie on one straight segment
(rms <= leaf_max_rms_m) whose line passes within leaf_hinge_tol_m of that
part's jamb, which starts at the jamb (not through it) and is at most
leaf_max_len_m long. Anything else shallow in an outer part -- a box, a bin,
a corner -- is 'not_a_leaf'. Leaf returns count as decided beams for BEAMS.
leaf_side says which jamb, left or right looking along the crossing normal.
The price, deliberately kept: a leaf open too little still crosses into the
core shallower than behind_margin_m and is rejected ('shallow'). On a
0.85 m door seen square-on from 2.7 m (test_door_detection) the smallest
opening accepted is 65 deg at core_fraction 0.6, 55 at 0.5, 50 at 0.4 and
45 at 0.3: a smaller core admits a less open leaf, and more clutter near the
jambs.

THE REVEAL (is_reveal). A jamb seen at an angle also shows the face running
back from the line: the reveal of a frame in a wall, the side of a post.
Shallow returns at most reveal_tol_m from their jamb along the line and
reveal_max_depth_m past it are that face, allowed at both jambs and never a
leaf; a 90 deg leaf reaches the margin there, a wall's thickness does not.

Closed doors are wall to the LiDAR and are out of scope; so are double doors
(they fail the width).

WHY THE GAP IS FOUND ON THE BEAMS, NOT BETWEEN TRACK ENDPOINTS
--------------------------------------------------------------
Since THE COLLINEAR GAP RULE (multi_wall_tracker) the extraction and the
tracker join collinear wall pieces only across a gap shorter than
width_min_m -- this very parameter -- so a doorway normally lies BETWEEN two
tracks. The search still runs on the beams along the whole collinear group,
INSIDE a track's extent too, as a fallback: the tracker measures a gap by
its open part sampled every 5 cm, so a borderline doorway can still be read
as just under the rule and joined into one track. Either way the tracks give
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

from corridor_perception.extraction import (
    adaptive_breakpoints, ExtractionParams, iepf_split, polar_to_cartesian)
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
BEAM_ON = 0        # on the line: wall_tol_m short of it .. on_behind_tol_m past
BEAM_BEYOND = 1    # at least behind_margin_m past it: through an opening
BEAM_NONE = 2      # no return at all (and not cut): through, as far as it knows
BEAM_SHORT = 3     # more than wall_tol_m short of it: something in front
BEAM_SHALLOW = 4   # past the line by on_behind_tol_m .. behind_margin_m
BEAM_CUT = 5       # cut out for a YOLO detection: no opinion

#: The reject codes door_candidates reports, in the order they are checked.
REJECT_CODES = ('not_collinear', 'untracked_jamb', 'width', 'on_line',
                'jamb_occluded', 'blocked', 'object', 'shallow', 'not_a_leaf',
                'few_beams', 'stereo_wall')

#: Which jamb an open leaf hangs on, looking along the crossing normal
#: (Door.LEAF_* on the wire).
LEAF_NONE = 0
LEAF_LEFT = 1
LEAF_RIGHT = 2
LEAF_NAMES = {LEAF_NONE: 'none', LEAF_LEFT: 'left', LEAF_RIGHT: 'right'}


@dataclass
class DoorParams:
    """The door detector's and tracker's numbers. See the module docstring."""

    # --- the candidate (door_candidates) -----------------------------------
    collinear_angle_rad: float = math.radians(5.0)
    collinear_offset_m: float = 0.10
    #: THE COLLINEAR GAP RULE's number too (multi_wall_tracker.DOOR_WIDTH_MIN_M).
    width_min_m: float = mwt.DOOR_WIDTH_MIN_M
    width_max_m: float = 1.00
    behind_margin_m: float = 0.30
    #: ON the line, on the near side: the tracker's own negative_tol_m.
    wall_tol_m: float = 0.15
    #: ...and on the far side only this much (THE LEAF): a return further
    #: past the line is not the wall's own surface. An open leaf's returns
    #: next to its hinge are a few cm past the line, and counted as wall they
    #: moved the jamb along the leaf and left stray ON returns in the gap.
    on_behind_tol_m: float = 0.05
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
    # --- the leaf (module docstring, THE LEAF) ------------------------------
    #: The central share of the gap where EVERY beam must be open.
    core_fraction: float = 0.6
    leaf_max_len_m: float = 1.0
    leaf_hinge_tol_m: float = 0.10
    #: A leaf is flat: the rms of its returns about their own line.
    leaf_max_rms_m: float = 0.03
    #: THE REVEAL (is_reveal): shallow returns within reveal_tol_m of their
    #: jamb along the line and at most reveal_max_depth_m past it are the
    #: jamb's own side, allowed at both jambs. A 90 deg leaf reaches the
    #: margin there; a wall, a frame and its post are thinner.
    reveal_tol_m: float = 0.05
    reveal_max_depth_m: float = 0.20
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
    # --- the stereo glass veto (stereo_glass_check), OFF by default ---------
    stereo_glass_veto: bool = False
    stereo_plane_tol_m: float = 0.30
    stereo_wall_veto_frac: float = 0.5
    #: Below this share of valid depth over the gap the stereo cannot say:
    #: no veto, and the door is flagged glass_suspect.
    stereo_min_valid_frac: float = 0.5
    # --- frame jambs (module docstring, FRAME JAMBS) ------------------------
    #: Also look for doorways between the short straight pieces the scan
    #: itself shows (door frames, mullions, panel edges), not only between
    #: wall tracks.
    frame_jambs: bool = True
    piece_min_points: int = 3
    piece_min_length_m: float = 0.04
    piece_max_rms_m: float = 0.015
    piece_max_range_m: float = 6.0
    #: A straight piece at least this long is a WALL, and a wall carries a
    #: door only once the tracker has confirmed it (criterion 1): 1.0 m is
    #: the length the tracker needs to start one (wall_min_span_birth_m).
    piece_max_length_m: float = 1.0
    #: A piece shorter than this has no trusted direction of its own: it
    #: joins a line on where its points lie, and its direction is checked
    #: (within frame_angle_rad) only if it is at least twice
    #: piece_min_length_m long.
    piece_angle_min_length_m: float = 0.30
    #: How many times a short run that is not straight is split again at its
    #: corner (scan_pieces): a frame and its side, then a reveal.
    piece_refine_depth: int = 3
    frame_angle_rad: float = math.radians(20.0)
    #: Two frame pieces this far apart along one line [m] are still one
    #: partition: its panes can be wider than a door (1.05-1.15 m in
    #: corridor10m_20261016T144537). Track to track it stays one doorway's
    #: width_max_m + jamb_extent_tol_m.
    frame_line_max_gap_m: float = 2.0
    #: A line made of frame pieces alone needs at least this many of them, at
    #: separate places along it (_stations).
    frame_min_pieces: int = 3
    #: The shortest run of ON returns that is a jamb on a frame line (a door
    #: frame seen square-on is 4-10 cm of the wall line).
    frame_jamb_min_length_m: float = 0.04
    # --- glass (module docstring, GLASS: SEEN SQUARE-ON, PANES) --------------
    #: SEEN SQUARE-ON: a door is confirmed only after this many SQUARE LOOKS,
    #: cycles on which THE FAN -- every beam within square_incidence_rad of
    #: its normal -- crossed its core, at least square_min_beams of them,
    #: all through, and none of its core's beams returned on its line.
    #: 0 disables (the step 1-2 rule).
    square_looks: int = 1
    #: A pane returns the beams within ~2 deg of its own normal (100 / 94 /
    #: 51 / 19 / 0 % at 0-1 / 1-2 / 2-3 / 3-4 / 4-5 deg); the fan is wider by
    #: what a door's normal can be off the glass's (~1-2 deg on a refined
    #: frame line or a track), so it always holds the beams the glass returns.
    square_incidence_rad: float = math.radians(3.0)
    square_min_beams: int = 8
    #: PANES: a door not yet seen square-on whose core returns at least this
    #: many beams ON its line, on pane_min_cycles consecutive cycles, is a
    #: pane of glass seen at last...
    pane_min_on_beams: int = 2
    pane_min_cycles: int = 2
    #: ...and no candidate is accepted there again for this long [s].
    pane_memory_s: float = 60.0


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
    leaf_side: int = LEAF_NONE
    glass_suspect: object = None     # True / False, or None: not evaluated

    @property
    def centre(self):
        return 0.5 * (self.jamb0 + self.jamb1)

    @property
    def width(self):
        return float(np.hypot(*(self.jamb1 - self.jamb0)))


#: The parent id of a jamb that lies on a LiDAR FRAME PIECE, not on a wall
#: track (Door.parent_wall_id0/1 on the wire). Track ids start at 1.
FRAME_ID = 0


@dataclass
class _Wall:
    track_id: int
    a: np.ndarray
    b: np.ndarray
    n: np.ndarray
    d: float

    @property
    def is_frame(self):
        return self.track_id == FRAME_ID

    @property
    def length(self):
        return float(np.hypot(*(self.b - self.a)))


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
    short_a = wa.length < p.piece_angle_min_length_m
    short_b = wb.length < p.piece_angle_min_length_m
    if short_a or short_b:
        return _collinear_short(wa, wb, p)
    angle = math.acos(min(1.0, abs(float(wa.n @ wb.n))))
    pa, pb = _facing_ends(wa, wb)
    offset = max(abs(float(wa.n @ pb) - wa.d), abs(float(wb.n @ pa) - wb.d))
    ok = angle <= p.collinear_angle_rad and offset <= p.collinear_offset_m
    return ok, angle, offset


def _collinear_short(wa, wb, p):
    """
    Return (ok, angle_rad, offset_m) when at least one piece is too short to trust.

    A 5-20 cm piece's own direction is noise (a few returns), so the line is
    the longer piece's when one is long enough, else the line through the
    two pieces' midpoints. Both pieces' endpoints must lie within
    collinear_offset_m of it, and a piece of at least twice
    piece_min_length_m must run along it within frame_angle_rad -- which is
    what keeps out the office partitions that start at a frame and run
    AWAY from the corridor.
    """
    if wa.length >= p.piece_angle_min_length_m or wb.length >= p.piece_angle_min_length_m:
        ref = wa if wa.length >= wb.length else wb
        n, d = ref.n, ref.d
    else:
        ma, mb = 0.5 * (wa.a + wa.b), 0.5 * (wb.a + wb.b)
        t = mb - ma
        norm = float(np.hypot(*t))
        if norm < 1e-6:
            return False, math.pi / 2, math.inf
        t = t / norm
        n = np.array([-t[1], t[0]])
        d = float(n @ ma)
    offset = max(abs(float(n @ q) - d) for q in (wa.a, wa.b, wb.a, wb.b))
    angle = 0.0
    for w in (wa, wb):
        if w.length >= 2.0 * p.piece_min_length_m:
            angle = max(angle, math.acos(min(1.0, abs(float(w.n @ n)))))
    ok = offset <= p.collinear_offset_m and angle <= p.frame_angle_rad
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
    # Only pairs whose segments come within reach of each other can share a
    # doorway: one vectorised point-to-segment pass instead of a collinearity
    # test for every pair of a few dozen pieces.
    near = np.triu(_near_pairs(walls, max(2.0 * p.width_max_m, p.frame_line_max_gap_m)), k=1)
    frame = np.array([w.is_frame for w in walls])
    ii, jj = np.nonzero(near)
    # Two tracks: the scalar test, with its reject trail.
    for i, j in zip(*(ii[~(frame[ii] | frame[jj])], jj[~(frame[ii] | frame[jj])])):
        wa, wb = walls[i], walls[j]
        ok, _angle, _offset = collinear(wa, wb, p)
        gap = _along_gap(wa, wb)
        if ok and gap <= reach:
            parent[find(j)] = find(i)
        elif ok and gap <= 2.0 * p.width_max_m:
            # One line, but too far apart to hold a single doorway.
            _reject(rejects, 'width', (wa.track_id, wb.track_id), gap)
        elif not ok:
            pa, pb = _facing_ends(wa, wb)
            dist = float(np.hypot(*(pa - pb)))
            if 0.5 * p.width_min_m <= dist <= reach:
                _reject(rejects, 'not_collinear', (wa.track_id, wb.track_id), dist)
    # A frame piece in the pair: the same test vectorised over all such
    # pairs, and no rejects (frame pieces are everything straight in the
    # scan; a reject for every pair that is not one line would drown the
    # trail). The pieces of one partition may lie a pane apart, wider than
    # a door: frame_line_max_gap_m.
    sel = frame[ii] | frame[jj]
    fi, fj = ii[sel], jj[sel]
    if fi.size:
        ok = (_collinear_pairs(walls, fi, fj, p)
              & (_along_gap_pairs(walls, fi, fj) <= max(reach, p.frame_line_max_gap_m)))
        for i, j in zip(fi[ok].tolist(), fj[ok].tolist()):
            parent[find(j)] = find(i)
    out = collections.defaultdict(list)
    for i, w in enumerate(walls):
        out[find(i)].append(w)
    return list(out.values())


def _near_pairs(walls, reach):
    """(N, N) bool: segments i and j come within `reach` of each other."""
    a = np.array([w.a for w in walls])
    b = np.array([w.b for w in walls])
    pts = np.concatenate([a, b, 0.5 * (a + b)])            # (3N, 2)
    ab = b - a
    den = np.maximum(np.sum(ab * ab, axis=1), 1e-12)
    t = np.clip(np.einsum('kij,ij->ki', pts[:, None, :] - a[None, :, :], ab)
                / den[None, :], 0.0, 1.0)                  # (3N, N)
    proj = a[None, :, :] + t[:, :, None] * ab[None, :, :]
    dist = np.hypot(pts[:, None, 0] - proj[..., 0], pts[:, None, 1] - proj[..., 1])
    n = len(walls)
    d = dist.reshape(3, n, n).min(axis=0)                  # point set of i vs segment j
    return np.minimum(d, d.T) <= reach


def _arrays(walls):
    a = np.array([w.a for w in walls])
    b = np.array([w.b for w in walls])
    n = np.array([w.n for w in walls])
    d = np.array([w.d for w in walls])
    return a, b, n, d, np.hypot(*(b - a).T)


def _collinear_pairs(walls, ii, jj, p):
    """collinear(walls[i], walls[j], p)[0] for every pair (i, j), vectorised."""
    a, b, n, d, length = _arrays(walls)
    ai, bi, ni, di, li = a[ii], b[ii], n[ii], d[ii], length[ii]
    aj, bj, nj, dj, lj = a[jj], b[jj], n[jj], d[jj], length[jj]
    ok = np.zeros(ii.size, dtype=bool)

    def dot(u, v):
        return np.sum(u * v, axis=-1)

    # Both long enough to trust their own lines: the facing-ends test.
    lng = (li >= p.piece_angle_min_length_m) & (lj >= p.piece_angle_min_length_m)
    if lng.any():
        ends_i = np.stack([ai[lng], bi[lng]], axis=1)                # (K, 2, 2)
        ends_j = np.stack([aj[lng], bj[lng]], axis=1)
        dist = np.linalg.norm(ends_i[:, :, None, :] - ends_j[:, None, :, :], axis=-1)
        best = dist.reshape(-1, 4).argmin(axis=1)
        rows = np.arange(best.size)
        pa, pb = ends_i[rows, best // 2], ends_j[rows, best % 2]
        angle = np.arccos(np.minimum(1.0, np.abs(dot(ni[lng], nj[lng]))))
        offset = np.maximum(np.abs(dot(ni[lng], pb) - di[lng]),
                            np.abs(dot(nj[lng], pa) - dj[lng]))
        ok[lng] = (angle <= p.collinear_angle_rad) & (offset <= p.collinear_offset_m)
    # At least one too short (_collinear_short): the longer piece's line when
    # it is long enough, else the line through the two midpoints.
    sh = ~lng
    if sh.any():
        use_i = li[sh] >= lj[sh]
        ref_n = np.where(use_i[:, None], ni[sh], nj[sh])
        ref_d = np.where(use_i, di[sh], dj[sh])
        by_mid = np.maximum(li[sh], lj[sh]) < p.piece_angle_min_length_m
        mi, mj = 0.5 * (ai[sh] + bi[sh]), 0.5 * (aj[sh] + bj[sh])
        tm = mj - mi
        norm = np.hypot(tm[:, 0], tm[:, 1])
        tm = tm / np.maximum(norm, 1e-12)[:, None]
        mid_n = np.column_stack([-tm[:, 1], tm[:, 0]])
        ref_n = np.where(by_mid[:, None], mid_n, ref_n)
        ref_d = np.where(by_mid, dot(mid_n, mi), ref_d)
        offset = np.max(np.abs(np.column_stack(
            [dot(ref_n, q) for q in (ai[sh], bi[sh], aj[sh], bj[sh])]) - ref_d[:, None]), axis=1)
        angle = np.zeros(offset.size)
        for nw, lw in ((ni[sh], li[sh]), (nj[sh], lj[sh])):
            ang = np.arccos(np.minimum(1.0, np.abs(dot(nw, ref_n))))
            angle = np.maximum(angle, np.where(lw >= 2.0 * p.piece_min_length_m, ang, 0.0))
        ok[sh] = ((offset <= p.collinear_offset_m) & (angle <= p.frame_angle_rad)
                  & ~(by_mid & (norm < 1e-6)))
    return ok


def _along_gap_pairs(walls, ii, jj):
    """_along_gap(walls[i], walls[j]) for every pair, vectorised."""
    a, b, n, _d, _length = _arrays(walls)
    u = np.column_stack([-n[ii, 1], n[ii, 0]])
    ta = np.sort(np.column_stack([np.sum(u * a[ii], axis=1), np.sum(u * b[ii], axis=1)]), axis=1)
    tb = np.sort(np.column_stack([np.sum(u * a[jj], axis=1), np.sum(u * b[jj], axis=1)]), axis=1)
    return np.maximum(tb[:, 0] - ta[:, 1], ta[:, 0] - tb[:, 1])


_MAX_REJECTS = 16


def _reject(rejects, code, parents, width):
    if rejects is not None and len(rejects) < _MAX_REJECTS:
        rejects.append((code, tuple(int(x) for x in parents), round(float(width), 3)))


def record_reject(rejects, code, cand):
    """Record a candidate refused after door_candidates (the stereo glass veto)."""
    _reject(rejects, code, cand.parents, cand.width)


def classify_beams(r, cut, s, p):
    """BEAM_* per beam for returns r on rays meeting the wall line at s."""
    r = np.asarray(r, dtype=float)
    s = np.asarray(s, dtype=float)
    out = np.full(r.shape, BEAM_SHALLOW, dtype=np.int8)
    finite = np.isfinite(r)
    with np.errstate(invalid='ignore'):
        diff = np.where(finite, r - s, np.inf)
    out[finite & (diff >= -p.wall_tol_m) & (diff <= p.on_behind_tol_m)] = BEAM_ON
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
    framed = any(w.is_frame for w in group)
    pure_frame = framed and all(w.is_frame for w in group)
    if framed:
        # FRAME JAMBS: the pieces are too short for their own lines to be
        # trusted, so every member is judged against ONE line fitted through
        # all of them, each weighted by its length -- then refitted to every
        # return on it (_refine_line): a door's normal is what a SQUARE LOOK
        # is measured against.
        min_frames = max(2, int(p.frame_min_pieces))
        if pure_frame and len(group) < min_frames:
            return []       # too few frames to be a partition line
        n, d = _group_line(group)
        if pure_frame and _stations(group, n, p.jamb_min_length_m) < min_frames:
            return []
        n, d = _refine_line(n, d, group, beams, p)
        line = _Wall(FRAME_ID, group[0].a, group[0].b, n, d)
        members = [_Wall(w.track_id, _foot(w.a, line), _foot(w.b, line), n, d) for w in group]
        jamb_min = p.frame_jamb_min_length_m
    else:
        ref = max(group, key=lambda w: float(np.hypot(*(w.b - w.a))))
        # Every member's line with its normal on the same side as the
        # reference, so the crossing distances below are the same quantity
        # for all of them.
        members = []
        for w in group:
            if float(w.n @ ref.n) < 0.0:
                w = _Wall(w.track_id, w.a, w.b, -w.n, -w.d)
            members.append(w)
        n, d = ref.n, ref.d
        jamb_min = p.jamb_min_length_m
    u = np.array([-n[1], n[0]])
    spans = np.array([sorted((float(u @ w.a), float(u @ w.b))) for w in members])
    if float(spans[:, 1].max() - spans[:, 0].min()) < p.width_min_m + 2.0 * jamb_min:
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
    keep = (ends > starts) & (length >= jamb_min)
    if pure_frame:
        # A partition line no track holds: a run of wall as long as a track
        # needs to be born is a wall the tracker has not confirmed, never a
        # jamb (criterion 1). Its returns stay ON the line, inside the gap.
        keep &= length < p.piece_max_length_m
    pieces = list(zip(starts[keep].tolist(), ends[keep].tolist()))
    # On a partition line no track holds, the line itself is the evidence
    # (frame_min_pieces frames on it): a jamb is any run of ON returns along
    # it, whether or not this scan made a piece of that frame -- a 6 cm
    # frame is a piece one scan and three returns the next.
    whole = (float(spans[:, 0].min()), float(spans[:, 1].max()))

    out = []
    for (_a0, a1), (b0, _b1) in zip(pieces, pieces[1:]):
        wa, wb = members[k[a1]], members[k[b0]]
        j0, j1 = _foot(pts[a1], wa), _foot(pts[b0], wb)
        width = float(np.hypot(*(j1 - j0)))
        parents = (wa.track_id, wb.track_id)
        if width < 0.5 * p.width_min_m:
            # A dropout between two returns of one wall: not worth a reject.
            continue
        gap = slice(a1 + 1, b0)
        code, n_open, n_req, leaf = _judge_gap(
            cls[gap], (j0, j1), (whole, whole) if pure_frame else (spans[k[a1]], spans[k[b0]]),
            u, (wa, wb), lxy, inc, p, tb[gap], pts[gap])
        if code:
            _reject(rejects, code, parents, width)
            continue
        centre = 0.5 * (j0 + j1)
        normal = n if float(n @ (centre - lxy)) >= 0.0 else -n
        cand = DoorCandidate(j0, j1, normal.copy(), parents, n_open, n_req)
        if leaf is not None:
            cand.leaf_side = side_of(leaf, centre, normal)
        out.append(cand)
    return out


def _refine_line(n, d, group, beams, p, bands=(0.10, 0.04)):
    """
    Return (n, d) refitted (TLS) to the scan's returns on the line, inside the group.

    The group line through a few 5-10 cm pieces is off by a degree or more
    (their endpoints are a few returns each); the returns along it -- every
    frame, and any glass ranged square-on -- span metres. Each pass keeps the
    uncut returns within `band` of the current line and within
    jamb_extent_tol_m of the group's extent along it; fewer than
    piece_min_points * 3 of them, or a spread under width_min_m, and the
    line is kept as it was.
    """
    r, v, lxy, cut = beams
    ok = np.isfinite(r) & ~np.asarray(cut, dtype=bool)
    q = lxy + np.where(ok, r, 0.0)[:, None] * v
    q = q[ok]
    u = np.array([-n[1], n[0]])
    ends = np.array([[float(u @ w.a), float(u @ w.b)] for w in group])
    lo, hi = float(ends.min()) - p.jamb_extent_tol_m, float(ends.max()) + p.jamb_extent_tol_m
    along = q @ u
    inside = (along >= lo) & (along <= hi)
    for band in bands:
        on = inside & (np.abs(q @ n - d) <= band)
        if (np.count_nonzero(on) < 3 * p.piece_min_points
                or float(np.ptp(along[on])) < p.width_min_m):
            break
        c = q[on].mean(axis=0)
        _vals, vecs = np.linalg.eigh(np.cov((q[on] - c).T, bias=True))
        t = vecs[:, 1]
        new_n = np.array([-t[1], t[0]])
        if float(new_n @ n) < 0.0:
            new_n = -new_n
        n, d = new_n, float(new_n @ c)
    return n, d


def _stations(members, n, merge_gap):
    """
    Return how many separate places along the line (normal n) the members occupy.

    Members whose extents along it overlap or lie within merge_gap of each
    other are one place: a post's face and the side running back from it
    are one frame, not two.
    """
    u = np.array([-n[1], n[0]])
    spans = sorted(sorted((float(u @ w.a), float(u @ w.b))) for w in members)
    count, end = 0, -math.inf
    for lo, hi in spans:
        if lo - end >= merge_gap:
            count += 1
        end = max(end, hi)
    return count


def _group_line(group):
    """Return (n, d) of the line through a group's segments, length-weighted TLS."""
    pts, wts = [], []
    for w in group:
        for q in (w.a, 0.5 * (w.a + w.b), w.b):
            pts.append(q)
            wts.append(max(w.length, 1e-3))
    pts, wts = np.array(pts), np.array(wts)
    c = (pts * wts[:, None]).sum(axis=0) / wts.sum()
    q = pts - c
    cov = (q * wts[:, None]).T @ q
    _vals, vecs = np.linalg.eigh(cov)
    t = vecs[:, 1]
    n = np.array([-t[1], t[0]])
    return n, float(n @ c)


def _run_fits(pts, cs, s, e):
    """
    Return (centre, tangent, rms, lo, hi) of the TLS line through each run [s, e).

    Laser-centred `pts`; `cs` are their prefix sums of (x, y, xx, xy, yy).
    Every run is a contiguous block of returns, so its moments come from the
    prefix sums: one vectorised 2x2 fit for all of them. The returns of a
    straight piece are in scan order along it, so its first and last returns
    are its ends (lo, hi: along the tangent from the centre).
    """
    n = (e - s).astype(float)
    m = (cs[e] - cs[s]) / n[:, None]
    cx, cy = m[:, 0], m[:, 1]
    sxx, sxy, syy = m[:, 2] - cx * cx, m[:, 3] - cx * cy, m[:, 4] - cy * cy
    phi = 0.5 * np.arctan2(2.0 * sxy, sxx - syy)          # the major axis
    t = np.column_stack([np.cos(phi), np.sin(phi)])
    lam_min = 0.5 * (sxx + syy) - np.hypot(0.5 * (sxx - syy), sxy)
    c = np.column_stack([cx, cy])
    ta = np.sum((pts[s] - c) * t, axis=1)
    tb = np.sum((pts[e - 1] - c) * t, axis=1)
    return c, t, np.sqrt(np.maximum(lam_min, 0.0)), np.minimum(ta, tb), np.maximum(ta, tb)


def _split_at_corner(pts, s, e):
    """Return the run [s, e) as two runs sharing its farthest return from its end-to-end chord."""
    q = pts[s:e]
    chord = q[-1] - q[0]
    norm = math.hypot(chord[0], chord[1])
    if norm < 1e-9:
        return None
    dev = np.abs(chord[0] * (q[:, 1] - q[0, 1]) - chord[1] * (q[:, 0] - q[0, 0])) / norm
    k = int(np.argmax(dev[1:-1])) + 1 if q.shape[0] > 2 else 0
    if k <= 0:
        return None
    return (s, s + k + 1), (s + k, e)


def scan_pieces(scan, p=None):
    """
    Return the short straight pieces of one scan as _Wall frames, base_link.

    The extraction's own adaptive breakpoints and iterative end-point split
    (corridor_perception.extraction, the same split_dist), then a straight
    piece is every run of at least piece_min_points returns, at least
    piece_min_length_m and less than piece_max_length_m long (a piece as long
    as a track needs to be born is a wall: criterion 1), with an rms about
    its own line of at most piece_max_rms_m, within piece_max_range_m of the
    laser. Unlike a wall observation there is NO minimum of 0.5 m and 10
    points: a door frame seen in a glass partition is a few centimetres of
    returns. The extraction's split smooths over a few returns and so never
    breaks a corner only centimetres across -- a frame's face and the side
    running back from it, or a jamb and its reveal -- so a short run that is
    not straight is split again at its farthest return from its end-to-end
    chord, piece_refine_depth times at most. Cut beams (object sectors) are
    no-returns here, as everywhere.
    """
    p = p or DoorParams()
    ranges, bearings = np.asarray(scan[0], dtype=float), np.asarray(scan[1], dtype=float)
    lxy = np.asarray(scan[2], dtype=float).reshape(2)
    cut = (np.asarray(scan[3], dtype=bool) if len(scan) > 3 and scan[3] is not None
           else np.zeros(ranges.shape, dtype=bool))
    # Returns past any piece that could pass the range test are dropped
    # before the split: most of a corridor scan is far wall, and the split is
    # the one per-cluster Python loop here.
    reach = p.piece_max_range_m + 0.5 * p.piece_max_length_m
    with np.errstate(invalid='ignore'):
        r = np.where(cut | ~np.isfinite(ranges) | (ranges > reach), np.inf, ranges)
    ep = ExtractionParams()
    pts = polar_to_cartesian(np.where(np.isfinite(r), r, 0.0), bearings)   # laser-centred
    runs = [(int(g[0]), int(g[-1]) + 1)
            for cluster in adaptive_breakpoints(r, bearings, ep)
            if cluster.size >= p.piece_min_points
            for g in iepf_split(pts, cluster, ep) if g.size >= p.piece_min_points]
    if not runs:
        return []
    x, y = pts[:, 0], pts[:, 1]
    cs = np.vstack([np.zeros(5), np.cumsum(np.column_stack([x, y, x * x, x * y, y * y]),
                                           axis=0)])
    s, e = (np.array(v) for v in zip(*runs))
    fit = _run_fits(pts, cs, s, e)
    for _depth in range(max(0, int(p.piece_refine_depth))):
        c, t, rms, lo, hi = fit
        bent = ((hi - lo < p.piece_max_length_m) & (rms > p.piece_max_rms_m)
                & (e - s >= 2 * p.piece_min_points - 1))
        if not bent.any():
            break
        keep_s, keep_e = list(s[~bent]), list(e[~bent])
        for k in np.flatnonzero(bent):
            halves = _split_at_corner(pts, int(s[k]), int(e[k]))
            for hs, he in halves or ():
                if he - hs >= p.piece_min_points:
                    keep_s.append(hs)
                    keep_e.append(he)
        s, e = np.array(keep_s, dtype=int), np.array(keep_e, dtype=int)
        if s.size == 0:
            return []
        fit = _run_fits(pts, cs, s, e)
    c, t, rms, lo, hi = fit
    length = hi - lo
    keep = ((np.hypot(c[:, 0], c[:, 1]) <= p.piece_max_range_m)
            & (length >= p.piece_min_length_m) & (length < p.piece_max_length_m)
            & (rms <= p.piece_max_rms_m))
    out = []
    for k in np.flatnonzero(keep):
        nrm = np.array([-t[k, 1], t[k, 0]])
        ck = c[k] + lxy
        out.append(_Wall(FRAME_ID, ck + lo[k] * t[k], ck + hi[k] * t[k], nrm,
                         float(nrm @ ck)))
    return out


def _uncovered(pieces, walls, p):
    """
    Return the pieces no eligible wall track already accounts for.

    A piece lying on a tracked wall (its points within collinear_offset_m of
    the line, within jamb_extent_tol_m of its extent) is a fragment of that
    wall: the track is the better line, so where tracks exist the result is
    exactly the track-only detector's.
    """
    if not walls or not pieces:
        return list(pieces)
    wa, wb, wn, wd, _length = _arrays(walls)
    u = np.column_stack([-wn[:, 1], wn[:, 0]])                       # (W, 2)
    lo = np.minimum(np.sum(u * wa, axis=1), np.sum(u * wb, axis=1)) - p.jamb_extent_tol_m
    hi = np.maximum(np.sum(u * wa, axis=1), np.sum(u * wb, axis=1)) + p.jamb_extent_tol_m
    covered = np.ones((len(pieces), len(walls)), dtype=bool)
    for q in (np.array([pc.a for pc in pieces]), np.array([pc.b for pc in pieces])):
        along = q @ u.T                                              # (P, W)
        covered &= ((np.abs(q @ wn.T - wd[None, :]) <= p.collinear_offset_m)
                    & (along >= lo[None, :]) & (along <= hi[None, :]))
    return [pc for pc, cov in zip(pieces, covered.any(axis=1)) if not cov]


def gap_evidence(jamb0, jamb1, scan, p=None):
    """
    Return what this scan's beams say across a door: counts of its core's beams.

    Base_link jambs. The beams crossing the CENTRAL core_fraction of the
    segment, classified against its line exactly as door_candidates does
    (classify_beams): 'on', 'open' (beyond or no return), 'short', 'cut' and
    'n' (all of them). THE FAN is every beam of the scan within
    square_incidence_rad of the line's normal, wherever it crosses the line:
    'fan' counts them, 'square' those crossing the core, 'square_open' those
    of them through. A SQUARE LOOK is the whole fan inside the core, at least
    square_min_beams of it, all through, and no 'on' at all (module
    docstring, GLASS); 'on' is also what PANES count -- glass the LiDAR has
    finally ranged, which an open doorway never returns.
    """
    p = p or DoorParams()
    j0, j1 = np.asarray(jamb0, dtype=float), np.asarray(jamb1, dtype=float)
    width = float(np.hypot(*(j1 - j0)))
    out = {'on': 0, 'open': 0, 'short': 0, 'cut': 0, 'n': 0,
           'fan': 0, 'square': 0, 'square_open': 0}
    if width < 1e-6:
        return out
    t = (j1 - j0) / width
    n = np.array([-t[1], t[0]])
    d = float(n @ j0)
    ranges, bearings = np.asarray(scan[0], dtype=float), np.asarray(scan[1], dtype=float)
    lxy = np.asarray(scan[2], dtype=float).reshape(2)
    cut = (np.asarray(scan[3], dtype=bool) if len(scan) > 3 and scan[3] is not None
           else np.zeros(ranges.shape, dtype=bool))
    v = np.column_stack([np.cos(bearings), np.sin(bearings)])
    nv = v @ n
    with np.errstate(divide='ignore', invalid='ignore'):
        s = (d - float(n @ lxy)) / nv
        along = (lxy @ t - float(j0 @ t)) + s * (v @ t)
    outer = 0.5 * (1.0 - min(max(float(p.core_fraction), 0.0), 1.0)) * width
    towards = (np.abs(nv) > p.min_incidence) & (s > 0.05)
    in_core = (along >= outer) & (along <= width - outer)
    fan = towards & (np.abs(nv) >= math.cos(p.square_incidence_rad))
    out['fan'] = int(np.count_nonzero(fan))
    sel = towards & in_core
    if not sel.any():
        return out
    cls = classify_beams(ranges[sel], cut[sel], s[sel], p)
    is_open = (cls == BEAM_BEYOND) | (cls == BEAM_NONE)
    square = fan[sel]
    out['on'] = int(np.count_nonzero(cls == BEAM_ON))
    out['open'] = int(np.count_nonzero(is_open))
    out['short'] = int(np.count_nonzero(cls == BEAM_SHORT))
    out['cut'] = int(np.count_nonzero(cls == BEAM_CUT))
    out['n'] = int(cls.size)
    out['square'] = int(np.count_nonzero(square))
    out['square_open'] = int(np.count_nonzero(square & is_open))
    return out


def is_square_look(ev, p):
    """Return whether gap_evidence's counts are a SQUARE LOOK (module docstring, GLASS)."""
    return (ev['on'] == 0 and ev['square'] >= p.square_min_beams
            and ev['square'] == ev['fan'] and ev['square_open'] == ev['square'])


def side_of(jamb, centre, normal):
    """Return LEAF_LEFT or LEAF_RIGHT: the jamb's side looking along `normal`."""
    left = np.array([-normal[1], normal[0]])
    return LEAF_LEFT if float(left @ (np.asarray(jamb) - centre)) > 0.0 else LEAF_RIGHT


def is_leaf(points, jamb, p):
    """
    Return whether shallow returns `points` form an open leaf hung on `jamb`.

    One straight segment (rms about its own line at most leaf_max_rms_m)
    whose line passes within leaf_hinge_tol_m of the jamb, which starts at
    the jamb rather than running through it, and which is at most
    leaf_max_len_m long measured from the jamb (module docstring, THE LEAF).
    """
    pts = np.asarray(points, dtype=float).reshape(-1, 2)
    jamb = np.asarray(jamb, dtype=float)
    if pts.shape[0] == 0:
        return False
    if pts.shape[0] == 1:
        return float(np.hypot(*(pts[0] - jamb))) <= p.leaf_max_len_m
    c = pts.mean(axis=0)
    _w, vecs = np.linalg.eigh(np.cov((pts - c).T, bias=True))
    e = vecs[:, 1]
    normal = np.array([-e[1], e[0]])
    resid = (pts - c) @ normal
    if float(np.sqrt(np.mean(resid ** 2))) > p.leaf_max_rms_m:
        return False
    if abs(float(normal @ (jamb - c))) > p.leaf_hinge_tol_m:
        return False
    along = (pts - c) @ e
    s_jamb = float(e @ (jamb - c))
    lo, hi = float(along.min()), float(along.max())
    if not (s_jamb <= lo + p.leaf_hinge_tol_m or s_jamb >= hi - p.leaf_hinge_tol_m):
        return False      # the jamb is in the middle: a surface THROUGH it
    return max(abs(hi - s_jamb), abs(lo - s_jamb)) <= p.leaf_max_len_m


def is_reveal(points, jamb, wall, p):
    """
    Return whether shallow returns `points` are the jamb's own side: THE REVEAL.

    A jamb seen at an angle shows the face running back from the wall line
    -- the reveal of a door frame in a wall, the side of a glass
    partition's post. Its returns lie at the jamb along the line (within
    reveal_tol_m) and at most reveal_max_depth_m past it: a wall's thickness.
    Both jambs may show one; neither is a leaf (module docstring, THE LEAF).
    """
    pts = np.asarray(points, dtype=float).reshape(-1, 2)
    if pts.shape[0] == 0:
        return False
    tang = np.array([-wall.n[1], wall.n[0]])
    along = np.abs(pts @ tang - float(tang @ np.asarray(jamb, dtype=float)))
    depth = np.abs(pts @ wall.n - wall.d)
    return bool(np.all(along <= p.reveal_tol_m) and np.all(depth <= p.reveal_max_depth_m))


def _judge_gap(gap, jambs, spans, u, walls, lxy, inc, p, t_gap=None, pts_gap=None):
    """
    Return (reject code or '', decided beams, required beams, leaf jamb or None).

    `gap` is the BEAM_* classes strictly between the two pieces, in order;
    `jambs`, `spans` and `walls` are the two jambs, their walls' extents along
    `u` and the walls themselves; `t_gap` and `pts_gap` are those beams'
    crossings along `u` and return points (THE LEAF).
    """
    (j0, j1), (wa, wb) = jambs, walls
    for jamb, (lo, hi) in zip(jambs, spans):
        t = float(u @ jamb)
        if not (lo - p.jamb_extent_tol_m <= t <= hi + p.jamb_extent_tol_m):
            return 'untracked_jamb', 0, 0, None
    if wa.track_id != wb.track_id and not collinear(wa, wb, p)[0]:
        return 'not_collinear', 0, 0, None
    width = float(np.hypot(*(j1 - j0)))
    if not (p.width_min_m <= width <= p.width_max_m):
        return 'width', 0, 0, None
    if np.any(gap == BEAM_ON):
        return 'on_line', 0, 0, None
    edge = max(1, int(p.edge_beams))
    if np.any(gap[:edge] == BEAM_SHORT) or np.any(gap[-edge:] == BEAM_SHORT):
        return 'jamb_occluded', 0, 0, None
    if np.any(gap == BEAM_SHORT):
        return 'blocked', 0, 0, None
    if np.any(gap == BEAM_CUT):
        return 'object', 0, 0, None
    shallow = gap == BEAM_SHALLOW
    leaf, n_leaf = None, 0
    if np.any(shallow):
        if t_gap is None or pts_gap is None:
            return 'shallow', 0, 0, None
        # THE LEAF: strict in the core, a leaf hung on a jamb in the outer parts.
        t0, t1 = float(u @ j0), float(u @ j1)
        frac = (np.asarray(t_gap, dtype=float) - t0) / (t1 - t0)
        outer = 0.5 * (1.0 - min(max(float(p.core_fraction), 0.0), 1.0))
        if np.any(shallow & (frac >= outer) & (frac <= 1.0 - outer)):
            return 'shallow', 0, 0, None
        pts_gap = np.asarray(pts_gap, dtype=float)
        hung = []
        for mask, jamb, wall in ((shallow & (frac < outer), j0, wa),
                                 (shallow & (frac > 1.0 - outer), j1, wb)):
            if not np.any(mask):
                continue
            if is_reveal(pts_gap[mask], jamb, wall, p):
                n_leaf += int(np.count_nonzero(mask))   # the jamb's own side
            else:
                hung.append((mask, jamb))
        if len(hung) > 1:
            return 'not_a_leaf', 0, 0, None     # one leaf per doorway
        if hung:
            mask, jamb = hung[0]
            if not is_leaf(pts_gap[mask], jamb, p):
                return 'not_a_leaf', 0, 0, None
            leaf = jamb
            n_leaf += int(np.count_nonzero(mask))
    n_open = int(np.count_nonzero((gap == BEAM_BEYOND) | (gap == BEAM_NONE))) + n_leaf
    va, vb = j0 - lxy, j1 - lxy
    span = abs(math.atan2(float(va[0] * vb[1] - va[1] * vb[0]), float(va @ vb)))
    n_expected = span / inc if math.isfinite(inc) and inc > 0.0 else 0.0
    n_req = max(int(p.min_beams), int(math.ceil(p.min_beam_fraction * n_expected)))
    if n_open < n_req:
        return 'few_beams', n_open, n_req, None
    return '', n_open, n_req, leaf


def door_candidates(tracks, car_pose, scan, p=None, rejects=None, pieces=None):
    """
    Return every doorway of this cycle, base_link at the scan stamp.

    `tracks` are the wall tracker's tracks (all of them: the eligible ones
    are picked here). `scan` is (ranges, base_link bearings, laser xy, cut),
    exactly what MultiWallTracker.step was given. `rejects`, when a list,
    collects (code, (parent ids), width) for the gaps that failed, capped at
    16: a debug trail, not a log. `pieces` are this scan's frame pieces
    (scan_pieces); computed here when frame_jambs is on and none are given.
    """
    p = p or DoorParams()
    if scan is None:
        return []
    walls = [_wall_bl(t, car_pose) for t in tracks if eligible(t)]
    if pieces is None and p.frame_jambs:
        pieces = scan_pieces(scan, p)
    walls = walls + _uncovered(pieces or [], walls, p)
    if not walls:
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

def _gap_samples(cand):
    """
    Return (gap rectangle (N, 3), floor beyond the line (M, 3)) samples, or None.

    The rectangle runs between the jambs, 0.1 m in from each, every 5 cm, at
    0.3 .. 1.8 m above the floor; the floor samples lie 0.3 .. 1.2 m past the
    line along the candidate's normal.
    """
    j0, normal = cand.jamb0, cand.normal
    width = cand.width
    if width <= 0.2:
        return None
    tang = (cand.jamb1 - j0) / width
    ts = np.arange(0.1, width - 0.1 + 1e-9, 0.05)
    zs = np.arange(0.3, 1.81, 0.1)
    xy = j0 + ts[:, None] * tang
    gap = np.column_stack([np.repeat(xy, zs.size, axis=0), np.tile(zs, ts.size)])
    beyond = np.array([[*(j0 + t * tang + k * normal), 0.0]
                       for t in ts for k in (0.3, 0.6, 0.9, 1.2)])
    return gap, beyond


def _sample_mask(points, labels, depth, fx, fy, cx, cy, rotation, translation, max_range,
                 depth_scale):
    """(classes, depth m (NaN invalid), expected optical z, n in image, n) of points."""
    lab = np.asarray(labels)
    dep = np.asarray(depth)
    h, w = lab.shape[:2]
    rot = np.asarray(rotation, dtype=float)
    trans = np.asarray(translation, dtype=float).reshape(3)
    uv = scene_mask.project_to_image(points, fx, fy, cx, cy, rot, trans)
    exp = ((points - trans) @ rot)[:, 2]
    with np.errstate(invalid='ignore'):
        ok = (np.isfinite(uv[:, 0]) & (uv[:, 0] >= 0) & (uv[:, 0] < w)
              & (uv[:, 1] >= 0) & (uv[:, 1] < h) & (exp <= max_range))
    ui = np.rint(uv[ok, 0]).clip(0, w - 1).astype(int)
    vi = np.rint(uv[ok, 1]).clip(0, h - 1).astype(int)
    raw = dep[vi, ui]
    dz = raw.astype(float) * depth_scale
    if np.issubdtype(raw.dtype, np.integer):
        dz[raw == 0] = np.nan
    dz[~np.isfinite(dz) | (dz <= 0.0)] = np.nan
    return lab[vi, ui], dz, exp[ok], int(np.count_nonzero(ok)), points.shape[0]


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
    samples = _gap_samples(cand)
    if samples is None:
        return None, detail
    gap, beyond = samples
    args = (labels, depth, fx, fy, cx, cy, rotation, translation, p.stereo_max_range_m,
            depth_scale)
    cls, dz, exp, n_in, n_all = _sample_mask(gap, *args)
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
    fcls, _fdz, _fexp, f_in, _f_all = _sample_mask(beyond, *args)
    if f_in == 0:
        return None, detail
    floor = (fcls == scene_mask.CLASS_FLOOR) | (fcls == scene_mask.CLASS_FLOOR_INFERRED)
    detail['floor_share'] = float(np.count_nonzero(floor)) / f_in
    support = (detail['wall_share'] <= p.stereo_max_wall_share
               and detail['floor_share'] >= p.stereo_min_floor_share)
    return bool(support), detail


def stereo_glass_check(cand, labels, depth, fx, fy, cx, cy, rotation, translation,
                       p=None, depth_scale=1.0):
    """
    Return (veto, glass_suspect, detail) for one base_link candidate.

    The stereo glass veto (stereo_glass_veto; the caller decides whether to
    act on it). Over the gap's image rectangle, WALL pixels whose VALID depth
    is within stereo_plane_tol_m of the wall line's own depth at that pixel
    are a surface ON the line: at least stereo_wall_veto_frac of the valid
    samples and the gap is a pane, veto True ('stereo_wall'). Wall pixels
    farther than that -- the room behind an open door -- never count. With
    less than stereo_min_valid_frac of the samples carrying valid depth the
    stereo cannot say: no veto, glass_suspect True. veto is None (and
    glass_suspect False) when the camera does not cover the gap at all.
    Depths are optical z, the ZED's own depth convention.
    """
    p = p or DoorParams()
    detail = {'coverage': 0.0, 'valid_frac': math.nan, 'on_line_frac': math.nan}
    samples = _gap_samples(cand)
    if samples is None:
        return None, False, detail
    cls, dz, exp, n_in, n_all = _sample_mask(
        samples[0], labels, depth, fx, fy, cx, cy, rotation, translation,
        p.stereo_max_range_m, depth_scale)
    detail['coverage'] = n_in / max(n_all, 1)
    if n_in == 0 or detail['coverage'] < p.stereo_min_coverage:
        return None, False, detail
    valid = np.isfinite(dz)
    n_valid = int(np.count_nonzero(valid))
    detail['valid_frac'] = n_valid / n_in
    if detail['valid_frac'] < p.stereo_min_valid_frac:
        return False, True, detail
    with np.errstate(invalid='ignore'):
        on_line = (cls == scene_mask.CLASS_WALL) & valid & \
            (np.abs(dz - exp) <= p.stereo_plane_tol_m)
    detail['on_line_frac'] = float(np.count_nonzero(on_line)) / n_valid
    return detail['on_line_frac'] >= p.stereo_wall_veto_frac, False, detail


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
    #: LEAF_*, of the last sighting, relative to `normal`.
    leaf_side: int = LEAF_NONE
    #: The last glass check said the gap's depth was mostly invalid.
    glass_suspect: bool = False
    #: Consecutive cycles whose beams returned ON this door's line (PANES).
    pane_hits: int = 0
    #: Cycles it was seen square-on and clear (SEEN SQUARE-ON).
    square_looks: int = 0

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
        #: PANES: where a "door" turned out to be glass, kept pane_memory_s
        #: after the glass was last seen (DoorTrack records, odom).
        self.panes = []
        self.n_pane_kills = 0
        self.n_pane_suppressed = 0

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

    def step(self, candidates, car_pose, stamp, persons=(), scan=None):
        """
        Fold one cycle's base_link candidates in; return every live door.

        `persons` are (x, y, half width) of this cycle's person detections in
        base_link at the scan stamp (class already filtered). They never
        create or confirm a door; inside a CONFIRMED door they set occupied
        and keep it alive. `scan` is the cycle's scan exactly as
        door_candidates had it: what its beams say across every live door is
        the glass evidence (module docstring, GLASS). Without it no door is
        ever seen square-on, so with square_looks > 0 none confirms.
        """
        p = self.p
        stamp = float(stamp)
        self.panes = [pn for pn in self.panes if stamp - pn.last_seen <= p.pane_memory_s]
        obs = [self._in_odom(c, car_pose) + (c,) for c in candidates]
        # A candidate where a pane was seen is that pane again, not a door.
        kept = []
        for o in obs:
            if any(math.isfinite(self._cost(pn, o[0], o[1], o[2])) for pn in self.panes):
                self.n_pane_suppressed += 1
            else:
                kept.append(o)
        obs = kept
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
        if scan is not None:
            self._glass_evidence(car_pose, stamp, scan)
        self.doors = [d for d in self.doors if not self._expired(d, stamp)]
        return list(self.doors)

    def _glass_evidence(self, car_pose, stamp, scan):
        """
        Fold this cycle's beams across every door in: square looks and panes.

        Module docstring, GLASS. A cycle with no ON return in a door's core
        and at least square_min_beams of its square-on beams through is a
        SQUARE LOOK. A door with no square look yet whose core returns at
        least pane_min_on_beams ON its line on pane_min_cycles consecutive
        cycles is deleted and remembered as a pane for pane_memory_s,
        refreshed whenever the glass is seen again. A door once seen
        square-on is not glass: if it later returns on its line it has been
        closed or blocked, and simply goes unseen.
        """
        to_bl = car_pose.inverse()
        p = self.p

        def look(rec):
            j0, j1 = to_bl.transform_points(np.vstack([rec.jamb0, rec.jamb1]))
            return gap_evidence(j0, j1, scan, p)

        for pn in self.panes:
            if look(pn)['on'] >= p.pane_min_on_beams:
                pn.last_seen = stamp
        keep = []
        for door in self.doors:
            ev = look(door)
            if is_square_look(ev, p):
                door.square_looks += 1
                if door.history and door.history[-1]:
                    self._maybe_confirm(door)
            if door.square_looks == 0:
                door.pane_hits = door.pane_hits + 1 if ev['on'] >= p.pane_min_on_beams else 0
                if door.pane_hits >= max(1, int(p.pane_min_cycles)):
                    door.last_seen = stamp
                    self.panes.append(door)
                    self.n_pane_kills += 1
                    continue
            keep.append(door)
        self.doors = keep

    def _maybe_confirm(self, door):
        """Confirm a door seen on confirm_hits of its window and square_looks times square-on."""
        if (sum(door.history) >= self.p.confirm_hits
                and door.square_looks >= self.p.square_looks):
            door.ever_confirmed = True
        door.state = STATE_CONFIRMED if door.ever_confirmed else STATE_TENTATIVE

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
        # The leaf side is relative to the crossing normal, which this door
        # keeps from birth: a sighting from the other side mirrors it.
        flipped = float(normal @ door.normal) < 0.0
        door.leaf_side = ({LEAF_LEFT: LEAF_RIGHT, LEAF_RIGHT: LEAF_LEFT}.get(
            cand.leaf_side, cand.leaf_side) if flipped else cand.leaf_side)
        if cand.glass_suspect is not None:
            door.glass_suspect = bool(cand.glass_suspect)
        door.confidence = (1.0 - a) * door.confidence + a * self._confidence(cand)
        if cand.stereo_support is not None:
            door.stereo_support = bool(cand.stereo_support)
        self._maybe_confirm(door)

    def _confidence(self, cand):
        bonus = self.p.stereo_bonus if cand.stereo_support else 0.0
        return min(1.0, float(cand.confidence) + bonus)

    def _spawn(self, j0, j1, normal, cand, stamp):
        door = DoorTrack(self._next_id, j0.copy(), j1.copy(), normal.copy(), stamp, stamp,
                         parents=tuple(int(x) for x in cand.parents),
                         n_beams=int(cand.n_beams), leaf_side=int(cand.leaf_side),
                         glass_suspect=bool(cand.glass_suspect))
        door.history = collections.deque([True], maxlen=max(1, int(self.p.confirm_window)))
        door.confidence = self._confidence(cand)
        door.stereo_support = bool(cand.stereo_support)
        self._maybe_confirm(door)
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
