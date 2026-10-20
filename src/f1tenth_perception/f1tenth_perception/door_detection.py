"""
door_detection.py: open doorways in the confirmed walls, as an overlay.

Pure: numpy and this package's own modules, no rclpy. lidar_front_wall_node
hosts it behind door_detection_enabled, on the wall tracker's own cycle, and
publishes /perception/doors (f1tenth_messages/DoorArray) and /viz/doors.
PERCEPTION ONLY: nothing in the control path, the costmap, the BT or the
missions reads either topic.

THE OVERLAY RULE
----------------
Doors are an OVERLAY on the walls. Door detection never modifies, splits or
removes a wall track or any LiDAR data -- the walls it looks at, the scan it
reads and everything downstream of them are exactly what they were without
it -- and nothing may treat a doorway's gap as free space because a door
exists there. A door is a label on a gap the walls already have, nothing
more. (The Manhattan SNAP below rotates COPIES of the host lines inside
door_candidates for one cycle; the tracks are never touched.)

WHAT A DOOR IS HERE
-------------------
An OPEN doorway: a gap between two stretches of one confirmed wall line that
the LiDAR sees through. door_candidates evaluates every such gap of the cycle
and returns ALL of them as DoorCandidate records, each with the code of the
first test it failed in the order below ('' when it passed them all: a door).

  1. GEOMETRY -- not a candidate at all, no effect on any door:
     not_collinear  two host walls meet at a door-wide gap but are not one
                    line (a corner);
     untracked_jamb a jamb more than jamb_extent_tol_m off its wall's extent;
     width          jamb to jamb outside [width_min_m, width_max_m] (also two
                    collinear groups too far apart to hold a doorway).
  2. HOST (strong). A HOST is a wall track that is CONFIRMED (not tentative,
     not coasting) with role front, left or right. Only hosts form the
     collinear groups the gaps are searched along: nothing else is ever a
     jamb, and an UNKNOWN segment (a tentative track, a track with role
     unknown/other/back or none, a scan fragment no track holds) never hosts
     a door and is never evidence for or against one.
     off_manhattan  the host is more than manhattan_tol_deg off the room's
                    dominant direction (MANHATTAN PRIOR, below);
     short_line     the union of the group's members along the line, less
                    this gap when it lies inside one member, is shorter than
                    min_line_extent_m;
     short_side     a jamb lies less than min_side_len_m from the far end of
                    the member it is on.
  3. VISIBILITY (weak: the frame is UNDECIDABLE, not a rejection):
     oblique_view   the ray to the gap's centre is more than
                    max_view_angle_deg off the line's normal;
     jamb_occluded  a SHORT return among the first/last edge_beams beams of
                    the gap: the wall end is a shadow edge, not a jamb;
     unknown_in_gap something unknown in the gap -- detail 'blocked' (a
                    SHORT return elsewhere in it), 'object' (a beam cut out
                    for a YOLO detection) or 'unknown_on_line' (ON-band
                    returns in a gap BETWEEN two members: no host claims that
                    stretch of line, so whatever returned there -- a closed
                    leaf, a pane seen square-on, a panel -- is unknown);
     few_beams      fewer decided beams than n_required (BEAMS, below), or
                    an open leaf covers the whole core.
  4. GAP (strong). ALL DISTANCES ARE PERPENDICULAR to the (snapped) host
     line, never along the ray: a return r on a ray that meets the line at s
     with incidence cos(phi) lies (r - s) cos(phi) behind it.
     on_line        at least on_line_min_beams ON-band returns in a gap that
                    lies INSIDE one member's extent: the tracker holds wall
                    across that stretch, so the returns are the host wall
                    (fewer are just not counted as through);
     not_a_leaf     shallow returns in an outer part of the gap that are not
                    an open leaf or a reveal (THE LEAF, below);
     behind         in the central core_fraction of the gap, fewer than
                    behind_frac of the beams not on the leaf return at least
                    behind_margin_m behind the line or not at all;
     not_boundary   along both host segments, from the jamb to the member's
                    far end and boundary_end_margin_m clear of both, more
                    than boundary_max_hits finite returns lie more than
                    boundary_behind_min_m behind the line -- not counting
                    returns on another confirmed wall perpendicular to the
                    host: the host is not a boundary the LiDAR cannot see
                    through (a glass partition's panes).
  5. STEREO (strong), only for a gap that passed 1-4 and only when a
     StereoFrame is available (fetched at most once per call, lazily):
     stereo_wall    stereo_glass_veto on, and the class mask's WALL pixels
                    with valid depth sit ON the line over at least
                    stereo_wall_veto_frac of the gap: a pane the LiDAR saw
                    through (mostly invalid depth: kept, glass_suspect);
     furniture      jamb_height_check on, and a jamb's wall surface ends
                    below furniture_max_height_m with the camera seeing
                    above it (stereo_jamb_heights): desks, cabinets.
     A positive with a frame also gets stereo_gap_support (the confidence
     bonus; never required).
Every gap except a GEOMETRY one also gets `visible`: view within
max_view_angle_deg, no jamb_occluded, no unknown_in_gap and at least
n_required decided beams -- whatever its code.

CODE CLASSES (code_class): 'strong' (STRONG_CODES), 'weak' (WEAK_CODES),
'geometry' (GEOMETRY_CODES) or 'ok' (''). Only a STRONG code on a VISIBLE
gap is evidence against a door (LIFECYCLE); WEAK and GEOMETRY records, and
STRONG ones that are not visible, change nothing.

MANHATTAN PRIOR (ManhattanPrior, manhattan_prior)
-------------------------------------------------
Rooms are rectangular. The dominant direction is a length-weighted histogram
of the confirmed walls of at least manhattan_min_wall_len_m (normal angle
mod 90 deg, 2 deg bins, circular), its peak refined by the weighted circular
mean of the walls within manhattan_tol_deg of it, smoothed over time (EMA,
manhattan_tau_s, on the 4-fold circle). A host further than
manhattan_tol_deg from it is off_manhattan; the others are SNAPPED before
the beam profile is built: each member rotated about its own midpoint onto
the dominant axis, so a wall the tracker fitted a few degrees off is judged
on the room's true line. Long stable off-axis walls stay walls; they simply
cannot host doors. No dominant direction yet: no check, no snap.

THE LEAF (open door leaf) AND THE REVEAL
----------------------------------------
Shallow returns (on_behind_tol_m .. behind_margin_m behind the line) in the
OUTER parts of the gap (the (1 - core_fraction) / 2 next to each jamb), plus
ON-band returns past the line in an outer part that already holds shallow
ones (a leaf's returns straddle the ON band next to its hinge), are judged
per jamb side, grown with the shallow returns continuing contiguously from
that side into the core:
  * THE REVEAL: all within reveal_tol_m of the jamb along the line and at
    most reveal_max_depth_m deep (or a single such return at any depth) --
    the jamb's own side face. Allowed at both jambs.
  * THE LEAF: one straight segment (rms <= leaf_max_rms_m, at least two
    returns: a single return is never a leaf) whose line passes within
    leaf_hinge_tol_m of the jamb, which STARTS at the jamb (its near end
    within leaf_hinge_tol_m of it, and no open beam between the jamb and
    it) rather than running through it, and is at most leaf_max_len_m long.
    If the grown set is not a leaf but the outer part's own returns are,
    the leaf is those.
  * Two faces perpendicular to the line at both jambs, deeper than
    reveal_max_depth_m, are the two reveals of a thick wall: a door with no
    leaf. A perpendicular face at one jamb and a leaf at the other: the face
    is the reveal.
  * Anything else -- shallow returns at both jambs, a box, a pole, a segment
    detached from the jamb -- is 'not_a_leaf'.
Leaf and reveal returns count as decided beams and are left out of the core
when its through-share is taken. leaf_side says which jamb, left or right
looking along the crossing normal (LEFT: the normal rotated +90 deg about
+z), per sighting: NONE also means "no leaf visible from here".

BEAMS
-----
n_required = max(min_beams, ceil(min_beam_fraction * n_expected)), where
n_expected is the gap's angular width seen from the laser over the scan's
angle increment: what the geometry promises at that range and incidence.
Decided beams are those returning beyond, shallow (leaf and reveal
included) or not at all.

WHY THE GAP IS FOUND ON THE BEAMS
---------------------------------
Since THE COLLINEAR GAP RULE (multi_wall_tracker) the tracker joins
collinear wall pieces only across a gap shorter than width_min_m, so a
doorway normally lies BETWEEN two host tracks; the search still runs on the
beams inside a track's extent as a fallback (the in-track case). Along the
group's line the beams give the pieces of wall -- runs of ON returns at
least jamb_min_length_m long that lie on a host's extent -- and the gaps
between them. A jamb is the foot of a piece's last return on its member's
line.

LIFECYCLE (DoorTracker)
-----------------------
A positive record (code '') associates with a door by centre, normal and
width (cheapest first) or spawns a CANDIDATE. Each positive adds the car's
position and the observed width to the door's evidence (the last
confirm_window). A door is CONFIRMED once its evidence holds at least
min_viewpoints car positions viewpoint_sep_m apart (greedy clusters) and the
widths' population std is below width_std_max_m (and, with the optional
square_looks > 0, that many SQUARE LOOKS: gap_evidence). A car that stands
still never confirms a door. A STRONG record on a VISIBLE gap that
associates with a door not seen positive this cycle STRIKES it: evidence
cleared, CONFIRMED back to CANDIDATE; a second strike deletes it. WEAK and
GEOMETRY records, and STRONG ones not visible, have no effect at all. A
CANDIDATE unseen for candidate_max_age_s and a CONFIRMED door unseen for
confirmed_max_age_s are dropped -- unless a person detected inside a
CONFIRMED door (occupied) holds it. Door identity never depends on the wall
track ids (they re-spawn); the parent ids are only reported.
"""

import collections
from dataclasses import dataclass, field
import math

from f1tenth_perception import multi_wall_tracker as mwt
from f1tenth_perception import scene_mask
from f1tenth_perception import wall_fusion

import numpy as np

STATE_CANDIDATE = 0
#: The old name of STATE_CANDIDATE (Door.STATE_TENTATIVE on the wire).
STATE_TENTATIVE = STATE_CANDIDATE
STATE_CONFIRMED = 1
#: Legacy (Door.STATE_COASTING): doors no longer coast; never produced.
STATE_COASTING = 2
STATE_NAMES = {STATE_CANDIDATE: 'candidate', STATE_CONFIRMED: 'confirmed',
               STATE_COASTING: 'coasting'}

#: The roles of a wall track that may host a door (module docstring, HOST).
HOST_ROLES = ('front', 'left', 'right')

#: What one beam says about the wall line it crosses (classify_beams), by
#: its PERPENDICULAR depth behind the line.
BEAM_ON = 0        # on the line: wall_tol_m in front of it .. on_behind_tol_m behind
BEAM_BEYOND = 1    # at least behind_margin_m behind it: through an opening
BEAM_NONE = 2      # no return at all (and not cut): through, as far as it knows
BEAM_SHORT = 3     # more than wall_tol_m in front of it: something in front
BEAM_SHALLOW = 4   # behind it by on_behind_tol_m .. behind_margin_m
BEAM_CUT = 5       # cut out for a YOLO detection: no opinion

#: The code classes (module docstring, CODE CLASSES).
STRONG_CODES = ('on_line', 'behind', 'not_boundary', 'stereo_wall', 'furniture',
                'short_line', 'short_side', 'off_manhattan', 'not_a_leaf')
WEAK_CODES = ('oblique_view', 'few_beams', 'jamb_occluded', 'unknown_in_gap')
GEOMETRY_CODES = ('not_collinear', 'untracked_jamb', 'width')
#: Every code, in the order door_candidates checks them.
REJECT_CODES = ('untracked_jamb', 'not_collinear', 'width',
                'off_manhattan', 'short_line', 'short_side',
                'oblique_view', 'jamb_occluded', 'unknown_in_gap', 'few_beams',
                'on_line', 'not_a_leaf', 'behind', 'not_boundary',
                'stereo_wall', 'furniture')

#: Which jamb an open leaf hangs on, looking along the crossing normal
#: (Door.LEAF_* on the wire).
LEAF_NONE = 0
LEAF_LEFT = 1
LEAF_RIGHT = 2
LEAF_NAMES = {LEAF_NONE: 'none', LEAF_LEFT: 'left', LEAF_RIGHT: 'right'}

#: JAMB HEIGHT (stereo_jamb_heights): the samples on the wall plane beside
#: each jamb, outward along the host [m] and above the floor [m].
JAMB_SAMPLE_OFFSETS_M = (0.05, 0.10, 0.15, 0.20, 0.25, 0.30)
JAMB_SAMPLE_HEIGHTS_M = tuple(round(0.1 * k, 1) for k in range(1, 25))
#: The camera must have seen this far above a low wall surface's top, and
#: not wall there, before the surface is called furniture.
JAMB_SEEN_ABOVE_M = 0.30
#: A wall track is perpendicular to the host (not_boundary) within this.
PERPENDICULAR_TOL_RAD = math.radians(20.0)
#: The Manhattan histogram's bin width.
MANHATTAN_BIN_RAD = math.radians(2.0)


def code_class(code):
    """Return 'strong', 'weak', 'geometry' or 'ok' ('' is a door) for a record's code."""
    if code == '':
        return 'ok'
    if code in STRONG_CODES:
        return 'strong'
    if code in WEAK_CODES:
        return 'weak'
    if code in GEOMETRY_CODES:
        return 'geometry'
    raise ValueError(f'unknown door code {code!r}')


@dataclass
class DoorParams:
    """The door detector's and tracker's numbers. See the module docstring."""

    # --- the gap (door_candidates) ------------------------------------------
    collinear_angle_rad: float = math.radians(5.0)
    collinear_offset_m: float = 0.10
    #: THE COLLINEAR GAP RULE's number too (multi_wall_tracker.DOOR_WIDTH_MIN_M).
    width_min_m: float = mwt.DOOR_WIDTH_MIN_M
    width_max_m: float = 1.00
    #: Perpendicular depth behind the line from which a return is THROUGH.
    behind_margin_m: float = 1.0
    #: ON the line, in front of it: the tracker's own negative_tol_m.
    wall_tol_m: float = 0.15
    #: ...and behind it only this much: a return further behind is not the
    #: wall's own surface (an open leaf's returns next to its hinge are a
    #: few cm behind the line).
    on_behind_tol_m: float = 0.05
    min_beams: int = 8
    min_beam_fraction: float = 0.8
    #: A run of ON returns shorter than this is not a piece of wall: one
    #: stray return inside an opening is an ON return in the gap, not a jamb.
    jamb_min_length_m: float = 0.10
    #: How far past a host's measured extent a jamb may lie.
    jamb_extent_tol_m: float = 0.30
    #: |cos| between a beam and the line's normal below which the beam runs
    #: along the line and its crossing point is meaningless.
    min_incidence: float = 0.05
    #: Beams at each end of the gap whose SHORT return marks a shadow edge.
    edge_beams: int = 2
    # --- the host (module docstring, HOST and MANHATTAN PRIOR) ---------------
    manhattan_prior: bool = True
    manhattan_tol_deg: float = 10.0
    manhattan_min_wall_len_m: float = 1.5
    manhattan_tau_s: float = 2.0
    min_line_extent_m: float = 2.0
    min_side_len_m: float = 0.5
    # --- the gap tests (module docstring, VISIBILITY and GAP) ----------------
    max_view_angle_deg: float = 45.0
    #: The share of the core's beams (not on the leaf) that must be through.
    behind_frac: float = 0.7
    boundary_behind_min_m: float = 0.30
    boundary_max_hits: int = 2
    boundary_end_margin_m: float = 0.10
    on_line_min_beams: int = 2
    # --- the leaf (module docstring, THE LEAF) -------------------------------
    #: The central share of the gap where the beams must be through.
    core_fraction: float = 0.6
    leaf_max_len_m: float = 1.0
    leaf_hinge_tol_m: float = 0.10
    #: A leaf is flat: the rms of its returns about their own line.
    leaf_max_rms_m: float = 0.03
    #: THE REVEAL: within this of the jamb along the line...
    reveal_tol_m: float = 0.05
    #: ...and at most this deep: a wall's thickness. Two perpendicular faces
    #: deeper than this are a thick wall's two reveals.
    reveal_max_depth_m: float = 0.20
    # --- the lifecycle (DoorTracker) -----------------------------------------
    min_viewpoints: int = 2
    viewpoint_sep_m: float = 0.3
    width_std_max_m: float = 0.05
    #: The evidence window: the last this many positive observations.
    confirm_window: int = 20
    candidate_max_age_s: float = 10.0
    confirmed_max_age_s: float = 60.0
    assoc_dist_m: float = 0.30
    assoc_angle_rad: float = math.radians(15.0)
    assoc_width_m: float = 0.20
    #: Weight of a new observation in the jambs' running average.
    smoothing: float = 0.3
    person_classes: tuple = ('person',)
    #: A person counts as IN a door within this far either side of its line.
    person_depth_m: float = 0.6
    # --- SEEN SQUARE-ON, optional (gap_evidence, is_square_look) -------------
    #: Extra confirmation requirement: this many SQUARE LOOKS (cycles on which
    #: THE FAN -- every beam within square_incidence_rad of the door's normal
    #: -- crossed its core, at least square_min_beams of them, all through,
    #: and none of its core's beams returned on its line). 0: off.
    square_looks: int = 0
    square_incidence_rad: float = math.radians(3.0)
    square_min_beams: int = 8
    # --- stereo support (stereo_gap_support) ---------------------------------
    stereo_max_wall_share: float = 0.2
    stereo_min_floor_share: float = 0.5
    stereo_min_coverage: float = 0.25
    stereo_max_range_m: float = 8.0
    stereo_tol_base_m: float = 0.05
    stereo_tol_quad_per_m2: float = 0.012
    stereo_bonus: float = 0.2
    # --- the stereo glass veto (stereo_glass_check), OFF by default ----------
    stereo_glass_veto: bool = False
    stereo_plane_tol_m: float = 0.30
    stereo_wall_veto_frac: float = 0.5
    #: Below this share of valid depth over the gap the stereo cannot say:
    #: no veto, and the door is flagged glass_suspect.
    stereo_min_valid_frac: float = 0.5
    # --- the jamb height (stereo_jamb_heights) -------------------------------
    jamb_height_check: bool = True
    jamb_min_height_m: float = 1.8
    furniture_max_height_m: float = 1.3


@dataclass
class DoorCandidate:
    """One evaluated gap of one cycle, base_link at the scan stamp (module docstring)."""

    jamb0: np.ndarray
    jamb1: np.ndarray
    #: Unit normal of the wall line, pointing AWAY from the laser: the
    #: direction the car would cross the doorway in.
    normal: np.ndarray
    parents: tuple
    #: Decided beams across the gap, and how many it needed (BEAMS).
    n_beams: int = 0
    n_required: int = 0
    #: '' for a door, else the first test it failed (REJECT_CODES).
    code: str = ''
    #: code_class(code).
    cls: str = 'ok'
    #: The frame could decide (module docstring): only a visible STRONG
    #: record strikes a door.
    visible: bool = False
    #: unknown_in_gap: 'blocked' | 'object' | 'unknown_on_line'; else a hint.
    detail: str = ''
    view_deg: float = math.nan
    #: Through share of the core's beams not on the leaf (NaN: none).
    behind_frac: float = math.nan
    boundary_hits: int = 0
    leaf_side: int = LEAF_NONE
    glass_suspect: object = None     # True / False, or None: not evaluated
    #: True unless the stereo measured the jambs (stereo_jamb_heights).
    jamb_height_unknown: bool = True
    #: The wall-surface top [m] beside (jamb0, jamb1), None where unmeasured.
    jamb_tops: tuple = (None, None)
    #: Signed offset [deg] of the host from the dominant direction, None
    #: without a Manhattan prior.
    host_offset_deg: object = None
    confidence: float = 0.0
    stereo_support: object = None    # True / False, or None: not evaluated

    @property
    def centre(self):
        return 0.5 * (self.jamb0 + self.jamb1)

    @property
    def width(self):
        return float(np.hypot(*(self.jamb1 - self.jamb0)))


@dataclass
class StereoFrame:
    """
    One class mask + depth pair and how to project into it.

    `rotation` and `translation` take base-frame points into the optical
    frame exactly as scene_mask.project_to_image does: p_opt = R^T (p - t).
    `depth` is metres times depth_scale (0.001 for mono16 millimetres;
    0 or non-finite is invalid).
    """

    labels: np.ndarray
    depth: np.ndarray
    fx: float
    fy: float
    cx: float
    cy: float
    rotation: np.ndarray
    translation: np.ndarray
    depth_scale: float = 1.0

    def projection(self):
        """Return (labels, depth, fx, fy, cx, cy, rotation, translation), stereo_*'s order."""
        return (self.labels, self.depth, self.fx, self.fy, self.cx, self.cy,
                self.rotation, self.translation)


# ============================================================================
# Walls, hosts and the Manhattan prior
# ============================================================================

@dataclass
class _Wall:
    track_id: int
    a: np.ndarray
    b: np.ndarray
    n: np.ndarray
    d: float
    #: The track's normal angle in odom (ManhattanPrior.offset).
    theta: float = 0.0
    #: Its offset [rad] from the dominant direction as fitted (None: no prior).
    offset: object = None

    @property
    def length(self):
        return float(np.hypot(*(self.b - self.a)))

    @property
    def mid(self):
        return 0.5 * (self.a + self.b)

    def flipped(self):
        return _Wall(self.track_id, self.a, self.b, -self.n, -self.d, self.theta + math.pi,
                     self.offset)

    def snapped(self):
        """Return this wall turned by -offset about its own midpoint: THE SNAP."""
        c, s = math.cos(-self.offset), math.sin(-self.offset)
        rot = np.array([[c, -s], [s, c]])
        m = self.mid
        n = rot @ self.n
        return _Wall(self.track_id, m + rot @ (self.a - m), m + rot @ (self.b - m), n,
                     float(n @ m), self.theta - self.offset, self.offset)


def is_host(trk):
    """Return whether a wall track may host a door: CONFIRMED, role front/left/right."""
    return trk.state == mwt.STATE_CONFIRMED and getattr(trk, 'role', '') in HOST_ROLES


def _wall_bl(trk, car_pose):
    """One track as base_link endpoints and its line n . x = d."""
    a, b = (np.asarray(q, dtype=float) for q in mwt.segment_in_base_link(trk, car_pose))
    th = trk.theta - car_pose.theta
    n = np.array([math.cos(th), math.sin(th)])
    return _Wall(int(trk.id), a, b, n, float(n @ (0.5 * (a + b))), float(trk.theta))


def _wrap_quarter(angle):
    """Return angle wrapped into [-45, 45) deg: the offset from the nearest axis."""
    q = 0.5 * math.pi
    return float((angle + 0.5 * q) % q - 0.5 * q)


class ManhattanPrior:
    """
    The room's dominant wall direction, mod 90 deg (module docstring, MANHATTAN PRIOR).

    Stateful: update(tracks, stamp) once per cycle with the wall tracker's
    tracks, then pass it to door_candidates. Off (dominant_rad stays None,
    offset returns None) when params.manhattan_prior is False.
    """

    def __init__(self, params=None):
        self.p = params or DoorParams()
        #: The EMA on the 4-fold circle: sum w e^{i 4 a}, normalised.
        self._z = 0j
        self._stamp = None
        #: Dominant normal direction [rad] in [0, pi/2), odom; None until the
        #: first update with an eligible wall.
        self.dominant_rad = None
        #: |z|: ~1 when the confirmed walls all agree, lower when part of
        #: their length is off-axis or the direction moved recently.
        self.strength = 0.0
        #: Length [m] of the walls that agreed with the last update's peak.
        self.weight = 0.0
        #: Eligible walls (confirmed, long enough) in the last update.
        self.n_walls = 0

    def update(self, tracks, stamp):
        """Fold this cycle's confirmed walls of at least manhattan_min_wall_len_m in."""
        p = self.p
        if not p.manhattan_prior:
            return
        walls = [(float(t.theta), float(t.length)) for t in tracks
                 if t.state == mwt.STATE_CONFIRMED
                 and t.length >= p.manhattan_min_wall_len_m]
        if not walls:
            return      # no eligible wall this cycle: no update
        quarter = 0.5 * math.pi
        ang = np.mod(np.array([w[0] for w in walls]), quarter)
        wts = np.array([w[1] for w in walls])
        nbins = int(round(quarter / MANHATTAN_BIN_RAD))
        k = np.minimum((ang / MANHATTAN_BIN_RAD).astype(int), nbins - 1)
        hist = np.bincount(k, weights=wts, minlength=nbins)
        # The peak of the circular histogram, read through a 3-bin window so a
        # direction on a bin edge is not split between two bins.
        peak = int(np.argmax(hist + np.roll(hist, 1) + np.roll(hist, -1)))
        centre = (peak + 0.5) * MANHATTAN_BIN_RAD
        dist = np.mod(ang - centre + 0.5 * quarter, quarter) - 0.5 * quarter
        sel = np.abs(dist) <= math.radians(p.manhattan_tol_deg)
        u = complex(np.sum(wts[sel] * np.exp(4j * ang[sel]))) / float(np.sum(wts))
        stamp = float(stamp)
        if self._stamp is None or stamp < self._stamp or p.manhattan_tau_s <= 0.0:
            alpha = 1.0
        else:
            alpha = 1.0 - math.exp(-(stamp - self._stamp) / p.manhattan_tau_s)
        self._z = (1.0 - alpha) * self._z + alpha * u
        self._stamp = stamp
        self.n_walls = len(walls)
        self.weight = float(np.sum(wts[sel]))
        self.strength = abs(self._z)
        self.dominant_rad = (float(np.mod(np.angle(self._z) / 4.0, quarter))
                             if self.strength > 1e-12 else None)

    def offset(self, theta):
        """Return the signed offset [rad, -45..45 deg) of a normal angle from the nearest axis."""
        if not self.p.manhattan_prior or self.dominant_rad is None:
            return None
        return _wrap_quarter(float(theta) - self.dominant_rad)

    def to_debug(self):
        """Return the prior for the debug topic."""
        return {'dominant_deg': (None if self.dominant_rad is None
                                 else round(math.degrees(self.dominant_rad), 2) % 90.0),
                'strength': round(float(self.strength), 3),
                'weight': round(float(self.weight), 2),
                'n_walls': int(self.n_walls)}


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


def _away(n, point, lxy):
    """Return n oriented away from the laser at `point`."""
    return n if float(n @ (point - lxy)) >= 0.0 else -n


def _view_deg(centre, normal, lxy):
    ray = np.asarray(centre, dtype=float) - lxy
    norm = float(np.hypot(*ray))
    if norm < 1e-9:
        return 90.0
    return math.degrees(math.acos(min(1.0, abs(float(ray @ normal)) / norm)))


def _pair_record(code, wa, wb, lxy):
    """Return a GEOMETRY record between two hosts: jambs = their facing ends."""
    pa, pb = _facing_ends(wa, wb)
    centre = 0.5 * (pa + pb)
    normal = _away(wa.n, centre, lxy)
    return DoorCandidate(pa.copy(), pb.copy(), normal.copy(), (wa.track_id, wb.track_id),
                         code=code, cls='geometry',
                         view_deg=_view_deg(centre, normal, lxy))


def _groups(walls, p, lxy):
    """Return (collinear groups of hosts, GEOMETRY records between hosts)."""
    parent = list(range(len(walls)))

    def find(i):
        while parent[i] != i:
            parent[i] = parent[parent[i]]
            i = parent[i]
        return i

    reach = p.width_max_m + p.jamb_extent_tol_m
    pairs = []
    for i in range(len(walls)):
        for j in range(i + 1, len(walls)):
            wa, wb = walls[i], walls[j]
            ok, _angle, _offset = collinear(wa, wb, p)
            gap = _along_gap(wa, wb)
            if ok and gap <= reach:
                parent[find(j)] = find(i)
            elif ok and gap <= 2.0 * p.width_max_m:
                # One line, but too far apart to hold a single doorway.
                pairs.append(('width', i, j))
            elif not ok:
                pa, pb = _facing_ends(wa, wb)
                dist = float(np.hypot(*(pa - pb)))
                if 0.5 * p.width_min_m <= dist <= reach:
                    pairs.append(('not_collinear', i, j))
    # Only between hosts that did not end up in one group (a chain of
    # collinear walls is one line, whatever two of its members say).
    records = [_pair_record(code, walls[i], walls[j], lxy) for code, i, j in pairs
               if find(i) != find(j)]
    out = collections.defaultdict(list)
    for i, w in enumerate(walls):
        out[find(i)].append(w)
    return list(out.values()), records


# ============================================================================
# The beams
# ============================================================================

def beam_depth(r, s, cos_inc=None):
    """Return the PERPENDICULAR depth (r - s) |cos_inc| behind the line; inf for no return."""
    r = np.asarray(r, dtype=float)
    scale = 1.0 if cos_inc is None else np.abs(np.asarray(cos_inc, dtype=float))
    with np.errstate(invalid='ignore'):
        return np.where(np.isfinite(r), (r - np.asarray(s, dtype=float)) * scale, np.inf)


def classify_depth(depth, finite, cut, p):
    """Return BEAM_* per beam from its perpendicular depth behind the line."""
    depth = np.asarray(depth, dtype=float)
    finite = np.asarray(finite, dtype=bool)
    out = np.full(depth.shape, BEAM_SHALLOW, dtype=np.int8)
    with np.errstate(invalid='ignore'):
        out[finite & (depth >= -p.wall_tol_m) & (depth <= p.on_behind_tol_m)] = BEAM_ON
        out[finite & (depth >= p.behind_margin_m)] = BEAM_BEYOND
        out[finite & (depth < -p.wall_tol_m)] = BEAM_SHORT
    out[~finite] = BEAM_NONE
    out[np.asarray(cut, dtype=bool)] = BEAM_CUT
    return out


def classify_beams(r, cut, s, p, cos_inc=None):
    """BEAM_* per beam for returns r on rays meeting the line at s with incidence cos_inc."""
    r = np.asarray(r, dtype=float)
    return classify_depth(beam_depth(r, s, cos_inc), np.isfinite(r), cut, p)


def _angle_increment(bearings):
    b = np.asarray(bearings, dtype=float)
    if b.size < 2:
        return math.nan
    return float(np.median(np.abs(np.diff(b))))


def _foot(pt, wall):
    """Project pt onto the wall's line."""
    return pt - (float(wall.n @ pt) - wall.d) * wall.n


def _scan_arrays(scan):
    ranges = np.asarray(scan[0], dtype=float)
    bearings = np.asarray(scan[1], dtype=float)
    lxy = np.asarray(scan[2], dtype=float).reshape(2)
    cut = (np.asarray(scan[3], dtype=bool) if len(scan) > 3 and scan[3] is not None
           else np.zeros(ranges.shape, dtype=bool))
    return ranges, bearings, lxy, cut


def _union_length(spans):
    total, end = 0.0, -math.inf
    for lo, hi in sorted((float(a), float(b)) for a, b in spans):
        if hi <= end:
            continue
        total += hi - max(lo, end)
        end = hi
    return total


def side_of(jamb, centre, normal):
    """Return LEAF_LEFT or LEAF_RIGHT: the jamb's side looking along `normal`."""
    left = np.array([-normal[1], normal[0]])
    return LEAF_LEFT if float(left @ (np.asarray(jamb) - centre)) > 0.0 else LEAF_RIGHT


def is_leaf(points, jamb, p, tangent=None):
    """
    Return whether returns `points` form an open leaf hung on `jamb`.

    At least two returns (a single return is never a leaf) on one straight
    segment (rms about its own line at most leaf_max_rms_m) whose line
    passes within leaf_hinge_tol_m of the jamb, which STARTS at the jamb --
    the jamb not between its ends, and its nearest return within
    leaf_hinge_tol_m of it, plus the spacing of its own first two returns
    (at most leaf_hinge_tol_m more): beams grazing a leaf seen nearly
    edge-on sample it only every few centimetres, the first of them that far
    from the hinge -- and which is at most leaf_max_len_m long measured from
    the jamb (module docstring, THE LEAF). For a face perpendicular to the
    host line, pass the host line's unit `tangent`: its nearest return is
    then measured along the line (edge-on, a 90 deg leaf's first return
    may lie 0.2 m behind the hinge). That no open beam lies between the
    jamb and the leaf is the caller's to check (it needs the beam order).
    """
    pts = np.asarray(points, dtype=float).reshape(-1, 2)
    jamb = np.asarray(jamb, dtype=float)
    if pts.shape[0] < 2:
        return False
    if tangent is None:
        dist = np.sort(np.hypot(*(pts - jamb).T))
        near = float(dist[0]) - min(float(dist[1] - dist[0]), p.leaf_hinge_tol_m)
    else:
        near = float(np.min(np.abs((pts - jamb) @ np.asarray(tangent, dtype=float))))
    if near > p.leaf_hinge_tol_m:
        return False      # detached: it does not start at the jamb
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
    -- the reveal of a door frame in a wall, the side of a post. Its returns
    lie at the jamb along the line (within reveal_tol_m) and at most
    reveal_max_depth_m behind it: a wall's thickness.
    """
    pts = np.asarray(points, dtype=float).reshape(-1, 2)
    if pts.shape[0] == 0:
        return False
    tang = np.array([-wall.n[1], wall.n[0]])
    along = np.abs(pts @ tang - float(tang @ np.asarray(jamb, dtype=float)))
    depth = np.abs(pts @ wall.n - wall.d)
    return bool(np.all(along <= p.reveal_tol_m) and np.all(depth <= p.reveal_max_depth_m))


# ============================================================================
# The candidates
# ============================================================================

class _Stereo:
    """The cycle's StereoFrame, fetched at most once and only when first needed."""

    def __init__(self, source):
        self.source = source
        self.fetched = False
        self.frame = None

    def get(self):
        if not self.fetched:
            self.fetched = True
            src = self.source
            self.frame = src() if callable(src) else src
        return self.frame


class _Cycle:
    """What every group of one door_candidates call shares."""

    def __init__(self, p, tracks, car_pose, scan, stereo, manhattan):
        self.p = p
        self.tracks = tracks
        self.car_pose = car_pose
        r, bearings, lxy, cut = _scan_arrays(scan)
        self.r, self.lxy, self.cut = r, lxy, cut
        self.v = np.column_stack([np.cos(bearings), np.sin(bearings)])
        self.inc = _angle_increment(bearings)
        self.stereo = _Stereo(stereo)
        self.manhattan = manhattan if p.manhattan_prior else None
        self._confirmed = None

    def confirmed_walls(self):
        """Every CONFIRMED wall track (any role), base_link: what not_boundary may discount."""
        if self._confirmed is None:
            self._confirmed = [_wall_bl(t, self.car_pose) for t in self.tracks
                               if t.state == mwt.STATE_CONFIRMED]
        return self._confirmed


def door_candidates(tracks, car_pose, scan, p=None, stereo=None, manhattan=None):
    """
    Return every gap evaluated this cycle as a DoorCandidate, base_link at the scan stamp.

    `tracks` are the wall tracker's tracks, all of them (the HOSTS are picked
    here). `scan` is (ranges, base_link bearings, laser xy, cut), exactly
    what MultiWallTracker.step was given. `stereo` is a StereoFrame, or a
    zero-argument callable returning one or None: called at most once, and
    only when some gap reaches the STEREO stage. `manhattan` is a
    ManhattanPrior already updated for this cycle (its update(tracks, stamp)
    is the caller's), or None: no off_manhattan, no snap. code '' is a
    door; every other record is a gap that is not one this cycle, and why
    (module docstring). Nothing passed in is modified (THE OVERLAY RULE).
    """
    p = p or DoorParams()
    if scan is None:
        return []
    hosts = [_wall_bl(t, car_pose) for t in tracks if is_host(t)]
    if not hosts:
        return []
    cyc = _Cycle(p, tracks, car_pose, scan, stereo, manhattan)
    # THE SNAP, before the hosts are grouped: on the room's axis, two pieces
    # of one wall the tracker fitted a few degrees off each are one line
    # again. An off-Manhattan host is judged as fitted (off_manhattan).
    tol = math.radians(p.manhattan_tol_deg)
    for i, w in enumerate(hosts):
        if cyc.manhattan is not None:
            w.offset = cyc.manhattan.offset(w.theta)
        if w.offset is not None and abs(w.offset) <= tol:
            hosts[i] = w.snapped()
    groups, out = _groups(hosts, p, cyc.lxy)
    for group in groups:
        out += _GroupScan(group, cyc).candidates()
    return out


class _GroupScan:
    """One collinear group of hosts and the beams along its line."""

    def __init__(self, group, cyc):
        self.cyc = cyc
        ref = max(group, key=lambda w: w.length)
        # Every member with its normal on the reference's side, so the
        # crossing distances are the same quantity for all of them.
        self.members = [w if float(w.n @ ref.n) >= 0.0 else w.flipped() for w in group]
        self.offsets = [w.offset for w in self.members]
        lead = int(np.argmax([w.length for w in self.members]))
        self.n, self.d = self.members[lead].n, self.members[lead].d
        self.u = np.array([-self.n[1], self.n[0]])
        self.spans = np.array([sorted((float(self.u @ w.a), float(self.u @ w.b)))
                               for w in self.members])

    def candidates(self):
        """Return the records of every gap along this group's line."""
        p = self.cyc.p
        spans = self.spans
        if (float(spans[:, 1].max() - spans[:, 0].min())
                < p.width_min_m + 2.0 * p.jamb_min_length_m):
            return []   # too short to hold a doorway and a piece of wall either side
        if not self._profile():
            return []
        cls, tb, pts, k = self.cls, self.tb, self.pts, self.k
        # The pieces of wall: maximal runs of ON beams, long enough to be wall
        # and lying on a host's extent (nothing else is a jamb).
        edges = np.flatnonzero(np.diff(np.concatenate(
            [[0], (cls == BEAM_ON).astype(np.int8), [0]])))
        starts, ends = edges[0::2], edges[1::2] - 1
        length = np.abs((pts[ends] - pts[starts]) @ self.u)
        on_host = np.concatenate([[0], np.cumsum(self.inside)])
        on_host = (on_host[ends + 1] - on_host[starts]) > 0
        keep = (length >= p.jamb_min_length_m) & on_host
        pieces = list(zip(starts[keep].tolist(), ends[keep].tolist()))
        out = []
        for (_a0, a1), (b0, _b1) in zip(pieces, pieces[1:]):
            ka, kb = int(k[a1]), int(k[b0])
            j0 = _foot(pts[a1], self.members[ka])
            j1 = _foot(pts[b0], self.members[kb])
            if float(np.hypot(*(j1 - j0))) < 0.5 * p.width_min_m:
                continue    # a dropout between two returns of one wall
            out.append(self._judge(a1, b0, ka, kb, j0, j1, tb))
        return out

    def _profile(self):
        """Classify every usable beam crossing the group's stretch of line, in order along it."""
        cyc, p = self.cyc, self.cyc.p
        n, d, u, lxy, v = self.n, self.d, self.u, cyc.lxy, cyc.v
        t_lo = float(self.spans[:, 0].min()) - p.jamb_extent_tol_m
        t_hi = float(self.spans[:, 1].max()) + p.jamb_extent_tol_m
        nv = v @ n
        with np.errstate(divide='ignore', invalid='ignore'):
            s_ref = (d - float(n @ lxy)) / nv
            t_beam = float(u @ lxy) + s_ref * (v @ u)
            sel = (np.abs(nv) > p.min_incidence) & (s_ref > 0.05) & \
                (t_beam >= t_lo) & (t_beam <= t_hi)
        idx = np.flatnonzero(sel)
        if idx.size == 0:
            return False
        idx = idx[np.argsort(t_beam[idx], kind='stable')]
        tb = t_beam[idx]
        # Each beam is judged against the member whose extent it falls in
        # (or the nearest one): within the collinear tolerance the members
        # are not quite one line unless snapped.
        dist = np.maximum(self.spans[None, :, 0] - tb[:, None],
                          tb[:, None] - self.spans[None, :, 1])
        k = np.argmin(np.maximum(dist, 0.0), axis=1)
        inside = dist[np.arange(k.size), k] <= 0.0
        mem_n = np.array([w.n for w in self.members])[k]
        mem_d = np.array([w.d for w in self.members])[k]
        vk = v[idx]
        inc_k = np.sum(vk * mem_n, axis=1)
        with np.errstate(divide='ignore', invalid='ignore'):
            s = (mem_d - mem_n @ lxy) / inc_k
        usable = (np.abs(inc_k) > p.min_incidence) & (s > 0.0)
        if not usable.any():
            return False
        idx, tb, k, vk, s, inc_k = (x[usable] for x in (idx, tb, k, vk, s, inc_k))
        r = cyc.r[idx]
        finite = np.isfinite(r)
        self.depth = beam_depth(r, s, inc_k)
        self.cut = cyc.cut[idx]
        self.cls = classify_depth(self.depth, finite, self.cut, p)
        self.finite = finite
        self.tb, self.k, self.inside = tb, k, inside[usable]
        self.pts = lxy + np.where(finite, r, 0.0)[:, None] * vk
        return True

    # ---- one gap ----------------------------------------------------------------

    def _judge(self, a1, b0, ka, kb, j0, j1, tb):
        cyc, p = self.cyc, self.cyc.p
        lxy, u = cyc.lxy, self.u
        wa, wb = self.members[ka], self.members[kb]
        centre = 0.5 * (j0 + j1)
        normal = _away(self.n, centre, lxy)
        cand = DoorCandidate(j0, j1, normal.copy(), (wa.track_id, wb.track_id))
        cand.view_deg = _view_deg(centre, normal, lxy)
        width = cand.width
        # 1. GEOMETRY.
        code = ''
        for jamb, (lo, hi) in ((j0, self.spans[ka]), (j1, self.spans[kb])):
            t = float(u @ jamb)
            if not (lo - p.jamb_extent_tol_m <= t <= hi + p.jamb_extent_tol_m):
                code = 'untracked_jamb'
        if not code and ka != kb and not collinear(wa, wb, p)[0]:
            code = 'not_collinear'
        if not code and not (p.width_min_m <= width <= p.width_max_m):
            code = 'width'
        if code:
            cand.code, cand.cls = code, 'geometry'
            return cand
        in_track = ka == kb
        t0, t1 = float(u @ j0), float(u @ j1)
        g = slice(a1 + 1, b0)
        gcls, gdep, gpts = self.cls[g], self.depth[g], self.pts[g]
        frac = (tb[g] - t0) / (t1 - t0) if t1 > t0 else np.full(gcls.size, 0.5)
        codes, details = [], {}
        # 2. HOST.
        offs = [o for o in (self.offsets[ka], self.offsets[kb]) if o is not None]
        if offs:
            worst = max(offs, key=abs)
            cand.host_offset_deg = round(math.degrees(worst), 2)
            if abs(worst) > math.radians(p.manhattan_tol_deg):
                codes.append('off_manhattan')
        extent = _union_length(self.spans) - (width if in_track else 0.0)
        if extent < p.min_line_extent_m:
            codes.append('short_line')
        if min(t0 - self.spans[ka][0], self.spans[kb][1] - t1) < p.min_side_len_m:
            codes.append('short_side')
        # 4 (first: VISIBILITY needs the leaf). THE LEAF and the reveals.
        leaf_code, leaf_jamb, claimed, leaf_detail = self._leaf(gcls, gdep, gpts, frac,
                                                                j0, j1)
        is_open = (gcls == BEAM_BEYOND) | (gcls == BEAM_NONE)
        on_free = (gcls == BEAM_ON) & ~claimed
        decided = int(np.count_nonzero(is_open | (gcls == BEAM_SHALLOW) | claimed))
        va, vb = j0 - lxy, j1 - lxy
        span = abs(math.atan2(float(va[0] * vb[1] - va[1] * vb[0]), float(va @ vb)))
        n_expected = span / cyc.inc if math.isfinite(cyc.inc) and cyc.inc > 0.0 else 0.0
        n_req = max(int(p.min_beams), int(math.ceil(p.min_beam_fraction * n_expected)))
        cand.n_beams, cand.n_required = decided, n_req
        outer = 0.5 * (1.0 - min(max(float(p.core_fraction), 0.0), 1.0))
        core = (frac >= outer) & (frac <= 1.0 - outer) & ~claimed & (gcls != BEAM_CUT)
        n_core = int(np.count_nonzero(core))
        if n_core:
            cand.behind_frac = float(np.count_nonzero(core & is_open)) / n_core
        # 3. VISIBILITY.
        edge = np.zeros(gcls.size, dtype=bool)
        n_edge = max(1, int(p.edge_beams))
        edge[:n_edge] = True
        edge[-n_edge:] = True
        short = gcls == BEAM_SHORT
        jamb_occluded = bool(np.any(short & edge))
        unknown = ''
        if np.any(short & ~edge):
            unknown = 'blocked'
        elif np.any(gcls == BEAM_CUT):
            unknown = 'object'
        elif not in_track and np.any(on_free):
            unknown = 'unknown_on_line'
        oblique = cand.view_deg > p.max_view_angle_deg
        cand.visible = (not oblique and not jamb_occluded and not unknown
                        and decided >= n_req)
        if oblique:
            codes.append('oblique_view')
        if jamb_occluded:
            codes.append('jamb_occluded')
        if unknown:
            codes.append('unknown_in_gap')
            details['unknown_in_gap'] = unknown
        if decided < n_req or n_core == 0:
            codes.append('few_beams')
            if decided >= n_req:
                details['few_beams'] = 'leaf_covers_core'
        # 4. GAP.
        if in_track and int(np.count_nonzero(on_free)) >= p.on_line_min_beams:
            codes.append('on_line')
        if leaf_code:
            codes.append(leaf_code)
            details[leaf_code] = leaf_detail
        if n_core and cand.behind_frac < p.behind_frac:
            codes.append('behind')
        cand.boundary_hits = self._boundary_hits(ka, kb, t0, t1)
        if cand.boundary_hits > p.boundary_max_hits:
            codes.append('not_boundary')
        if leaf_jamb is not None:
            cand.leaf_side = side_of(leaf_jamb, centre, normal)
        code = min(codes, key=REJECT_CODES.index) if codes else ''
        # 5. STEREO.
        if not code:
            code = self._stereo(cand)
        cand.code, cand.cls = code, code_class(code)
        cand.detail = details.get(code, '')
        if not code:
            cand.confidence = lidar_confidence(cand)
        return cand

    def _leaf(self, gcls, gdep, gpts, frac, j0, j1):
        """Return (code, leaf jamb or None, claimed returns (mask), detail) of THE LEAF."""
        p = self.cyc.p
        n = gcls.size
        claimed = np.zeros(n, dtype=bool)
        if n == 0:
            return '', None, claimed, ''
        outer = 0.5 * (1.0 - min(max(float(p.core_fraction), 0.0), 1.0))
        sides = (frac < outer, frac > 1.0 - outer)
        core = ~(sides[0] | sides[1])
        shallow = gcls == BEAM_SHALLOW
        on_past = (gcls == BEAM_ON) & (gdep > 0.0)
        is_open = (gcls == BEAM_BEYOND) | (gcls == BEAM_NONE)
        found, softs = [], []
        for side, jamb in ((0, j0), (1, j1)):
            mine, other = sides[side], sides[1 - side]
            soft = (shallow | on_past) & mine if np.any(shallow & mine) else mine & False
            softs.append(soft)
            if not soft.any():
                found.append(('clear', None))
                continue
            order = np.arange(n) if side == 0 else np.arange(n)[::-1]
            first = int(np.argmax(soft[order]))
            starts = not bool(np.any(is_open[order[:first]]))
            run = np.zeros(n, dtype=bool)
            if starts:
                # The contiguous run from the jamb, through this side's outer
                # part, on into the core and -- a leaf seen from the far side
                # -- into the other outer part.
                for b in order[first:]:
                    if soft[b] or shallow[b] or (on_past[b] and other[b]):
                        run[b] = True
                    else:
                        break
            sets = [soft | run, soft | (run & core), soft]
            found.append(self._side(sets, jamb, gpts, gdep, starts))
        # A leaf whose run covers every soft return of the other jamb's outer
        # part leaves that jamb clear: seen from the far side, one leaf.
        for side in (0, 1):
            kind, mask = found[side]
            rest = softs[1 - side]
            if kind in ('leaf', 'perp') and rest.any() and not np.any(rest & ~mask):
                found[1 - side] = ('clear', None)
                break
        kinds = [kind for kind, _mask in found]
        # Only accepted returns are claimed: a reveal always, a leaf only when
        # the doorway has one.
        for kind, mask in found:
            if kind == 'reveal':
                claimed |= mask
        if 'hung' in kinds:
            return 'not_a_leaf', None, claimed, ''
        leafy = [side for side in (0, 1) if kinds[side] in ('leaf', 'perp')]
        if len(leafy) == 2 and 'leaf' in kinds and 'perp' not in kinds:
            return 'not_a_leaf', None, claimed, 'two_leaves'
        for side in leafy:
            claimed |= found[side][1]
        if len(leafy) == 2:
            if 'leaf' not in kinds:
                return '', None, claimed, ''         # a thick wall's two reveals
            # A perpendicular face and a leaf: the face is the reveal.
            return '', (j0, j1)[kinds.index('leaf')], claimed, ''
        if leafy:
            return '', (j0, j1)[leafy[0]], claimed, ''
        return '', None, claimed, ''

    def _side(self, sets, jamb, gpts, gdep, starts):
        """
        Return (kind, returns) of one jamb's side: reveal, perp, leaf or hung.

        `sets` are the side's candidate returns, largest first (grown into
        the other outer part, into the core, its own outer part only); the
        first that is a reveal or a leaf is the side's.
        """
        p = self.cyc.p
        tried = []
        for mask in sets:
            if not mask.any() or any(np.array_equal(mask, t) for t in tried):
                continue
            tried.append(mask)
            pts = gpts[mask]
            along = np.abs((pts - jamb) @ self.u)
            perpendicular = bool(np.all(along <= p.reveal_tol_m))
            if perpendicular and (float(np.max(gdep[mask])) <= p.reveal_max_depth_m
                                  or pts.shape[0] == 1):
                return 'reveal', mask
            if starts and is_leaf(pts, jamb, p, self.u if perpendicular else None):
                return ('perp' if perpendicular else 'leaf'), mask
        return 'hung', sets[1]

    def _boundary_hits(self, ka, kb, t0, t1):
        """Return how many returns lie well behind the host segments (not_boundary)."""
        cyc, p = self.cyc, self.cyc.p
        m = p.boundary_end_margin_m
        tb = self.tb
        sel = np.zeros(tb.size, dtype=bool)
        for lo, hi in ((self.spans[ka][0] + m, t0 - m), (t1 + m, self.spans[kb][1] - m)):
            if hi > lo:
                sel |= (tb >= lo) & (tb <= hi)
        with np.errstate(invalid='ignore'):
            hit = sel & self.finite & ~self.cut & (self.depth > p.boundary_behind_min_m)
        if not hit.any():
            return 0
        q = self.pts[hit]
        excluded = np.zeros(q.shape[0], dtype=bool)
        sin_tol = math.sin(PERPENDICULAR_TOL_RAD)
        for w in cyc.confirmed_walls():
            if abs(float(w.n @ self.n)) > sin_tol:
                continue    # not perpendicular to the host
            tw = np.array([-w.n[1], w.n[0]])
            lo, hi = sorted((float(tw @ w.a), float(tw @ w.b)))
            along = q @ tw
            excluded |= ((np.abs(q @ w.n - w.d) <= p.collinear_offset_m)
                         & (along >= lo - p.jamb_extent_tol_m)
                         & (along <= hi + p.jamb_extent_tol_m))
        return int(np.count_nonzero(~excluded))

    def _stereo(self, cand):
        """Run the STEREO stage on a gap that passed the LiDAR; return its code."""
        p = self.cyc.p
        frame = self.cyc.stereo.get() if self.cyc.stereo.source is not None else None
        if frame is None:
            return ''
        proj = frame.projection()
        if p.stereo_glass_veto:
            veto, cand.glass_suspect, _detail = stereo_glass_check(
                cand, *proj, p=p, depth_scale=frame.depth_scale)
            if veto:
                return 'stereo_wall'
        if p.jamb_height_check:
            heights = stereo_jamb_heights(cand, frame, p)
            cand.jamb_tops = (heights['jamb0']['wall_top'], heights['jamb1']['wall_top'])
            cand.jamb_height_unknown = heights['height_unknown']
            if heights['decision'] == 'furniture':
                return 'furniture'
        cand.stereo_support, _detail = stereo_gap_support(cand, *proj, p=p,
                                                          depth_scale=frame.depth_scale)
        return ''


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


def gap_evidence(jamb0, jamb1, scan, p=None):
    """
    Return what this scan's beams say across a door: counts of its core's beams.

    Base_link jambs. The beams crossing the CENTRAL core_fraction of the
    segment, classified against its line by perpendicular depth
    (classify_beams): 'on', 'open' (beyond or no return), 'short', 'cut' and
    'n' (all of them). THE FAN is every beam of the scan within
    square_incidence_rad of the line's normal, wherever it crosses the line:
    'fan' counts them, 'square' those crossing the core, 'square_open' those
    of them through. A SQUARE LOOK (is_square_look) is the whole fan inside
    the core, at least square_min_beams of it, all through, and no 'on'.
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
    ranges, bearings, lxy, cut = _scan_arrays(scan)
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
    cls = classify_beams(ranges[sel], cut[sel], s[sel], p, nv[sel])
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
    """Return whether gap_evidence's counts are a SQUARE LOOK (module docstring, LIFECYCLE)."""
    return (ev['on'] == 0 and ev['square'] >= p.square_min_beams
            and ev['square'] == ev['fan'] and ev['square_open'] == ev['square'])


# ============================================================================
# Stereo: support, the glass veto and the jamb height
# ============================================================================

def _gap_samples(cand):
    """
    Return (gap rectangle (N, 3), floor beyond the line (M, 3)) samples, or None.

    The rectangle runs between the jambs, 0.1 m in from each, every 5 cm, at
    0.3 .. 1.8 m above the floor; the floor samples lie 0.3 .. 1.2 m past the
    line along the candidate's normal.
    """
    j0, normal = np.asarray(cand.jamb0, dtype=float), np.asarray(cand.normal, dtype=float)
    width = cand.width
    if width <= 0.2:
        return None
    tang = (np.asarray(cand.jamb1, dtype=float) - j0) / width
    ts = np.arange(0.1, width - 0.1 + 1e-9, 0.05)
    zs = np.arange(0.3, 1.81, 0.1)
    xy = j0 + ts[:, None] * tang
    gap = np.column_stack([np.repeat(xy, zs.size, axis=0), np.tile(zs, ts.size)])
    ks = np.array([0.3, 0.6, 0.9, 1.2])
    floor = (xy[:, None, :] + ks[None, :, None] * normal).reshape(-1, 2)
    return gap, np.column_stack([floor, np.zeros(floor.shape[0])])


def _project(points, labels, depth, fx, fy, cx, cy, rotation, translation, max_range,
             depth_scale):
    """Return (in image (N,), classes, depth m (NaN invalid), expected optical z) of the points."""
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
    return ok, lab[vi, ui], dz, exp[ok]


def _sample_mask(points, labels, depth, fx, fy, cx, cy, rotation, translation, max_range,
                 depth_scale):
    """(classes, depth m (NaN invalid), expected optical z, n in image, n) of points."""
    ok, cls, dz, exp = _project(points, labels, depth, fx, fy, cx, cy, rotation,
                                translation, max_range, depth_scale)
    return cls, dz, exp, int(np.count_nonzero(ok)), points.shape[0]


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

    The stereo glass veto (stereo_glass_veto; door_candidates acts on it).
    Over the gap's image rectangle, WALL pixels whose VALID depth is within
    stereo_plane_tol_m of the wall line's own depth at that pixel are a
    surface ON the line: at least stereo_wall_veto_frac of the valid samples
    and the gap is a pane, veto True ('stereo_wall'). Wall pixels farther
    than that -- the room behind an open door -- never count. With less than
    stereo_min_valid_frac of the samples carrying valid depth the stereo
    cannot say: no veto, glass_suspect True. When the camera does not cover
    the gap, both are None: no opinion either way, so a door keeps the flag
    an earlier cycle set. Depths are optical z, the ZED's own convention.
    """
    p = p or DoorParams()
    detail = {'coverage': 0.0, 'valid_frac': math.nan, 'on_line_frac': math.nan}
    samples = _gap_samples(cand)
    if samples is None:
        return None, None, detail
    cls, dz, exp, n_in, n_all = _sample_mask(
        samples[0], labels, depth, fx, fy, cx, cy, rotation, translation,
        p.stereo_max_range_m, depth_scale)
    detail['coverage'] = n_in / max(n_all, 1)
    if n_in == 0 or detail['coverage'] < p.stereo_min_coverage:
        return None, None, detail
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


def stereo_jamb_heights(cand, stereo, p=None):
    """
    Return how high the wall surface beside each jamb reaches, and what that says.

    For each jamb ('jamb0', 'jamb1'), samples ON the wall plane beside it
    (JAMB_SAMPLE_OFFSETS_M outward along the host, JAMB_SAMPLE_HEIGHTS_M
    above the floor, base_link), projected into the StereoFrame. A sample
    is WALL-AT when its class is WALL and its valid depth lies within
    stereo_plane_tol_m of the plane's own optical depth there. Per side:
    'valid' (any in-image sample with valid depth), 'wall_top' (the highest
    WALL-AT sample, None if none) and 'seen_top' (the highest in-image
    sample with valid depth, None if none). 'decision':
      'tall'      both sides reach jamb_min_height_m;
      'furniture' a side's wall surface ends below furniture_max_height_m
                  and the camera saw at least JAMB_SEEN_ABOVE_M above it
                  without wall there (a desk, a cabinet);
      'unknown'   anything else: no decision.
    'height_unknown' is True when a side has no valid depth or no WALL-AT
    sample, or the view was clipped (a side's seen_top below
    jamb_min_height_m) without a decision; False otherwise.
    """
    p = p or DoorParams()
    j0 = np.asarray(cand.jamb0, dtype=float)
    j1 = np.asarray(cand.jamb1, dtype=float)
    width = float(np.hypot(*(j1 - j0)))
    out = {'decision': 'unknown', 'height_unknown': True}
    if width < 1e-6:
        for key in ('jamb0', 'jamb1'):
            out[key] = {'valid': False, 'wall_top': None, 'seen_top': None}
        return out
    tang = (j1 - j0) / width
    offs = np.asarray(JAMB_SAMPLE_OFFSETS_M, dtype=float)
    zs = np.asarray(JAMB_SAMPLE_HEIGHTS_M, dtype=float)
    sides = []
    for key, jamb, outward in (('jamb0', j0, -tang), ('jamb1', j1, tang)):
        xy = jamb + offs[:, None] * outward
        pts = np.column_stack([np.repeat(xy, zs.size, axis=0), np.tile(zs, offs.size)])
        ok, cls, dz, exp = _project(pts, *stereo.projection(), p.stereo_max_range_m,
                                    stereo.depth_scale)
        z_in = pts[ok, 2]
        valid = np.isfinite(dz)
        with np.errstate(invalid='ignore'):
            wall_at = (cls == scene_mask.CLASS_WALL) & valid & \
                (np.abs(dz - exp) <= p.stereo_plane_tol_m)
        side = {'valid': bool(valid.any()),
                'wall_top': float(z_in[wall_at].max()) if wall_at.any() else None,
                'seen_top': float(z_in[valid].max()) if valid.any() else None}
        out[key] = side
        sides.append(side)
    if any(not s['valid'] or s['wall_top'] is None for s in sides):
        return out
    if all(s['wall_top'] >= p.jamb_min_height_m for s in sides):
        out['decision'], out['height_unknown'] = 'tall', False
        return out
    if any(s['wall_top'] < p.furniture_max_height_m
           and s['seen_top'] >= s['wall_top'] + JAMB_SEEN_ABOVE_M for s in sides):
        out['decision'], out['height_unknown'] = 'furniture', False
        return out
    out['height_unknown'] = any(s['seen_top'] < p.jamb_min_height_m for s in sides)
    return out


# ============================================================================
# Tracking
# ============================================================================

def _perp(vec):
    return np.array([-vec[1], vec[0]])


@dataclass
class DoorTrack:
    """One door, in the tracker's fixed frame (odom). Module docstring, LIFECYCLE."""

    id: int  # noqa: A003 -- the wire name (Door.id), as Track.id is
    jamb0: np.ndarray
    jamb1: np.ndarray
    #: Unit, odom: the crossing direction, from the side the car first saw
    #: the door from. Kept perpendicular to jamb1 - jamb0.
    normal: np.ndarray
    born: float
    #: Last cycle with a positive observation of it.
    last_positive: float
    state: int = STATE_CANDIDATE
    #: Positive observations, ever.
    hits: int = 1
    #: STRONG visible rejections since it was last confirmed (2: deleted).
    strikes: int = 0
    #: (car x, car y, observed width, stamp) of its last confirm_window
    #: positive observations, odom. Cleared by a strike.
    evidence: collections.deque = field(default_factory=lambda: collections.deque(maxlen=20))
    #: Separate car positions in the evidence (viewpoint_sep_m apart, greedy).
    n_viewpoints: int = 0
    #: Population std [m] of the evidence widths.
    width_std: float = 0.0
    ever_confirmed: bool = False
    occupied: bool = False
    confidence: float = 0.0
    stereo_support: bool = False
    parents: tuple = (0, 0)
    n_beams: int = 0
    #: LEAF_*, of the last positive sighting, relative to `normal`.
    leaf_side: int = LEAF_NONE
    #: The last stereo glass check that could say found the gap's depth
    #: mostly invalid.
    glass_suspect: bool = False
    #: False once a positive sighting had both jambs measured (stereo).
    jamb_height_unknown: bool = True
    #: SQUARE LOOKS (only counted while DoorParams.square_looks > 0).
    square_looks: int = 0
    #: Last cycle a person inside it held it (occupied).
    last_held: float = -math.inf

    @property
    def centre(self):
        return 0.5 * (self.jamb0 + self.jamb1)

    @property
    def width(self):
        return float(np.hypot(*(self.jamb1 - self.jamb0)))


def _unoriented_angle(a, b):
    return math.acos(min(1.0, abs(float(np.asarray(a) @ np.asarray(b)))))


def viewpoints(positions, sep):
    """Return how many greedy clusters `positions` form: each new one >= sep from every earlier."""
    reps = []
    for q in positions:
        if all(math.hypot(q[0] - r[0], q[1] - r[1]) >= sep for r in reps):
            reps.append(q)
    return len(reps)


class DoorTracker:
    """Door tracks across cycles: the module docstring's LIFECYCLE."""

    def __init__(self, params=None):
        self.p = params or DoorParams()
        self.doors = []
        self._next_id = 1
        #: Every record door_candidates gave the last step (the debug topic's).
        self.last_candidates = []
        #: Cumulative: strikes applied, doors deleted by a second strike, and
        #: CANDIDATE -> CONFIRMED transitions.
        self.n_strikes = 0
        self.n_deleted_by_strikes = 0
        self.n_confirmed = 0

    def _in_odom(self, cand, car_pose):
        j0, j1 = car_pose.transform_points(np.vstack([cand.jamb0, cand.jamb1]))
        normal = car_pose.R @ np.asarray(cand.normal, dtype=float)
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

    def _associate(self, doors, obs):
        """Return [(door index, obs index)], cheapest first, one-to-one."""
        pairs = []
        for i, door in enumerate(doors):
            for j, (j0, j1, normal, _c) in enumerate(obs):
                cost = self._cost(door, j0, j1, normal)
                if math.isfinite(cost):
                    pairs.append((cost, i, j))
        used_d, used_o, out = set(), set(), []
        for _cost, i, j in sorted(pairs):
            if i in used_d or j in used_o:
                continue
            used_d.add(i)
            used_o.add(j)
            out.append((i, j))
        return out

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
        Fold one cycle's door_candidates records in; return every live door (both states).

        `persons` are (x, y, half width) of this cycle's person detections in
        base_link at the scan stamp (class already filtered). They never
        create or confirm a door; inside a CONFIRMED door they set occupied
        and keep it alive. `scan` (the cycle's scan exactly as door_candidates
        had it) is needed only for the optional SQUARE LOOKS.
        """
        p = self.p
        stamp = float(stamp)
        self.last_candidates = list(candidates)
        car_xy = (float(car_pose.x), float(car_pose.y))
        positive, strong = [], []
        for cand in candidates:
            kind = code_class(cand.code)
            if kind == 'ok':
                positive.append(self._in_odom(cand, car_pose) + (cand,))
            elif kind == 'strong' and cand.visible:
                strong.append(self._in_odom(cand, car_pose) + (cand,))
        seen = set()
        matched = self._associate(self.doors, positive)
        for i, j in matched:
            self._update(self.doors[i], *positive[j], stamp, car_xy)
            seen.add(self.doors[i].id)
        taken = {j for _i, j in matched}
        for j, (j0, j1, normal, cand) in enumerate(positive):
            if j not in taken:
                seen.add(self._spawn(j0, j1, normal, cand, stamp, car_xy).id)
        # STRONG, visible: a strike on a door no positive saw this cycle.
        others = [d for d in self.doors if d.id not in seen]
        struck = set()
        for i, _j in self._associate(others, strong):
            door = others[i]
            door.strikes += 1
            door.evidence.clear()
            door.n_viewpoints, door.width_std = 0, 0.0
            door.state = STATE_CANDIDATE
            self.n_strikes += 1
            if door.strikes >= 2:
                struck.add(door.id)
                self.n_deleted_by_strikes += 1
        self.doors = [d for d in self.doors if d.id not in struck]
        if scan is not None and p.square_looks > 0:
            to_bl = car_pose.inverse()
            for door in self.doors:
                j0, j1 = to_bl.transform_points(np.vstack([door.jamb0, door.jamb1]))
                if is_square_look(gap_evidence(j0, j1, scan, p), p):
                    door.square_looks += 1
        for door in self.doors:
            if door.id in seen:
                self._maybe_confirm(door)
        persons_odom = []
        if persons:
            pts = car_pose.transform_points(np.array([[x, y] for x, y, _ in persons]))
            persons_odom = [(float(q[0]), float(q[1]), float(h))
                            for q, (_x, _y, h) in zip(pts, persons)]
        for door in self.doors:
            door.occupied = (door.state == STATE_CONFIRMED
                             and self.person_inside(door, persons_odom))
            if door.occupied:
                door.last_held = stamp
        self._merge()
        self.doors = [d for d in self.doors if not self._expired(d, stamp)]
        return list(self.doors)

    def _maybe_confirm(self, door):
        """CONFIRM a candidate seen from enough viewpoints with a steady width."""
        p = self.p
        if door.state == STATE_CONFIRMED:
            return
        if (door.n_viewpoints >= p.min_viewpoints and door.width_std < p.width_std_max_m
                and (p.square_looks <= 0 or door.square_looks >= p.square_looks)):
            door.state = STATE_CONFIRMED
            door.ever_confirmed = True
            door.strikes = 0
            self.n_confirmed += 1

    def _evidence(self, door, car_xy, width, stamp):
        door.evidence.append((car_xy[0], car_xy[1], float(width), float(stamp)))
        door.n_viewpoints = viewpoints([(e[0], e[1]) for e in door.evidence],
                                       self.p.viewpoint_sep_m)
        door.width_std = float(np.std([e[2] for e in door.evidence]))

    def _update(self, door, j0, j1, normal, cand, stamp, car_xy):
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
        door.last_positive = stamp
        door.parents = tuple(int(x) for x in cand.parents)
        door.n_beams = int(cand.n_beams)
        # The leaf side is relative to the crossing normal, which this door
        # keeps from birth: a sighting from the other side mirrors it.
        flipped = float(normal @ door.normal) < 0.0
        door.leaf_side = ({LEAF_LEFT: LEAF_RIGHT, LEAF_RIGHT: LEAF_LEFT}.get(
            cand.leaf_side, cand.leaf_side) if flipped else cand.leaf_side)
        if cand.glass_suspect is not None:
            door.glass_suspect = bool(cand.glass_suspect)
        door.jamb_height_unknown = door.jamb_height_unknown and bool(cand.jamb_height_unknown)
        door.confidence = (1.0 - a) * door.confidence + a * self._confidence(cand)
        if cand.stereo_support is not None:
            door.stereo_support = bool(cand.stereo_support)
        self._evidence(door, car_xy, cand.width, stamp)

    def _confidence(self, cand):
        bonus = self.p.stereo_bonus if cand.stereo_support else 0.0
        return min(1.0, float(cand.confidence) + bonus)

    def _spawn(self, j0, j1, normal, cand, stamp, car_xy):
        door = DoorTrack(self._next_id, j0.copy(), j1.copy(), normal.copy(), stamp, stamp,
                         parents=tuple(int(x) for x in cand.parents),
                         n_beams=int(cand.n_beams), leaf_side=int(cand.leaf_side),
                         glass_suspect=bool(cand.glass_suspect),
                         jamb_height_unknown=bool(cand.jamb_height_unknown))
        door.evidence = collections.deque(maxlen=max(1, int(self.p.confirm_window)))
        door.confidence = self._confidence(cand)
        door.stereo_support = bool(cand.stereo_support)
        self._evidence(door, car_xy, cand.width, stamp)
        self._next_id += 1
        self.doors.append(door)
        return door

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
        max_age = (self.p.confirmed_max_age_s if door.state == STATE_CONFIRMED
                   else self.p.candidate_max_age_s)
        return stamp - max(door.last_positive, door.last_held) > max_age
