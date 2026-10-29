"""semantic_doors.py -- open doorways found in the SEMANTIC LAYER's walls.

Pure: numpy only, no rclpy. The ROS glue is semantic_layer_node, behind
semantic_doors_enabled and door_source == 'semantic'.

WHY HERE AND NOT BESIDE door_detection.py. f1tenth_perception's
door_detection.py is the PROFILE detector: it reads the raw per-beam depth
profile perpendicular to a host wall line, plus the stereo class mask, and
lives next to both. This one reads the SEMANTIC LAYER'S WALLS AND NOTHING
ELSE geometric -- the CONFIRMED tracks of /perception/walls as
semantic_layer_node already re-expresses them into the map frame -- so it
lives next to those. The scan is used for exactly two liveness questions
(do the beams cross the opening, and is a wall end a real end or a shadow
edge); no beam profile, no bands, no stereo. The two detectors are selected
by door_source and never run together.

THE OVERLAY RULE, unchanged from door_detection.py and binding here too: a
door never modifies, splits or removes a wall track or any LiDAR data, and
NOTHING may treat a doorway's gap as free space because a door is published
there. A door is a label on a gap the walls already have.

WHAT A DOOR IS HERE
-------------------
A gap between two WALL ENDS, of width_min_m .. width_max_m, that the LiDAR
sees through and that nothing stands in. Two shapes:

  CASE A (parallel)   two walls that are one line across the gap: their
                      directions within parallel_angle_deg, and the lateral
                      (across-line) offset between the two facing ends under
                      parallel_offset_m. The classic doorway in a long wall,
                      held by the tracker as two tracks because
                      door_width_min_m is also the tracker's collinear merge
                      limit (stack_params, lidar_front_wall_door_width_min_m).

  CASE B (90 degrees) one wall's END against a PERPENDICULAR wall: the
                      directions within perp_tol_deg of square, and the gap
                      measured from that end PERPENDICULARLY TO THE OTHER
                      WALL'S PLANE. A corridor mouth, or a doorway whose far
                      jamb is the return wall itself. The foot of that
                      perpendicular must lie strictly inside the other wall's
                      extent (foot_inset_m) -- a foot at that wall's own end
                      is two unknown ends facing each other, which is a
                      corner, not an opening.

Both jambs must be REAL WALL ENDS (real_end below). Only OPEN openings are
reported: there is no closed-door state here, and a gap the LiDAR does not
see through is simply not a door this cycle.

THE FOUR GATES, in the order they are applied (first failure is the code)
------------------------------------------------------------------------
  level       a wall below door_min_wall_level is not a wall for this
              purpose -- see wall_level(). Applied before pairing, so a
              tentative track is not even a candidate jamb.
  geometry    'angle', 'offset', 'width', 'overlap' (case A: the walls are
              not on opposite sides of the gap -- they overlap along the
              line), 'foot_outside' (case B).
  ends        'end_fov'       a jamb's bearing is within end_fov_margin_rad
                              of the scan's own edge, or its range is within
                              end_range_margin_m of range_max: the wall may
                              simply continue outside what the scan saw;
              'end_occluded'  a beam just past the jamb, on the gap side,
                              returns end_occlusion_margin_m or more SHORTER
                              than the jamb itself: the end is the edge of
                              something nearer, a shadow boundary, not a jamb.
  passage     'few_beams'     fewer than min_beams beams aimed into the gap;
              'not_through'   fewer than through_frac of them return more
                              than behind_margin_m PERPENDICULARLY BEYOND the
                              opening line (or do not return at all): what is
                              behind the gap is too near to be a room;
              'on_segment'    a valid return lies within gap_clear_m of the
                              opening segment, the jambs' own returns
                              excluded by shrinking the segment
                              jamb_exclude_m at each end;
              'object'        a semantic-layer object (a recognised YOLO
                              track or a confirmed unknown) overlaps the same
                              shrunk segment.

A candidate that passes all four is a door for this cycle. candidate_doors()
returns EVERY gap it evaluated with its code, so a replay or a debug topic
can say why a doorway was not reported.

TRACKING (DoorTracker)
----------------------
Ids are stable and never reused. A cycle's doors are associated to the live
tracks by CENTRE DISTANCE under assoc_max_m, nearest first; an unmatched
candidate starts a new track. A track is CONFIRMED on its confirm_hits'th
hit and stays confirmed. Centre, width and axis are smoothed (EMA,
smooth_alpha) so a door does not jitter with the wall fit.

THE CROSSING DIRECTION IS FIXED AT FIRST CONFIRMATION. `normal` is the unit
vector perpendicular to the jamb line, oriented at that one moment so it
points from the car's side of the opening to the far side, and it is NEVER
recomputed -- that is what keeps the door frame's x axis from flipping when
the car drives through and ends up on the other side. Before confirmation
the normal follows the car, because there is not yet a door to have a
direction.

A CONFIRMED DOOR IS KEPT IN THE MAP after it leaves view: nothing times out
unless forget_unseen_s is set above 0. An UNCONFIRMED track is dropped after
candidate_forget_s without a hit, so a one-off geometric coincidence does
not sit in the list forever.
"""

import itertools
import math

import numpy as np

# ---------------------------------------------------------------------------
# Wall confirmation levels
# ---------------------------------------------------------------------------
#: The two independent axes MultiWallTrack carries -- `state` (tentative /
#: confirmed / coasting) and `source` (the best evidence the track EVER had:
#: lidar / stereo / both) -- ordered into the one scale door_min_wall_level
#: compares against. The names are what stack_params.yaml spells, because
#: 'on'/'off'-style words are what YAML mangles (CLAUDE.md) and a bare integer
#: in a config file says nothing to the person reading it.
#: Every value door_source may take, and the module that owns each:
#:   'freespace'  f1tenth_costmap/freespace_doors.py   -- the occupancy grid
#:   'semantic'   f1tenth_costmap/semantic_doors.py    -- this module's walls
#:   'profile'    f1tenth_perception/door_detection.py -- the beam profile
#: Lives here rather than in a config module because all three nodes that
#: validate door_source already import one of the door modules, and a fourth
#: place to keep the list in step is a fourth place to forget.
DOOR_SOURCES = ('freespace', 'semantic', 'profile')

LEVEL_TENTATIVE = 0        # any live track, confirmed or not
LEVEL_CONFIRMED = 1        # confirmed (or coasting) on ANY evidence, stereo-only included
LEVEL_LIDAR_CONFIRMED = 2  # confirmed, and the LiDAR has seen it -- THE DEFAULT
LEVEL_STEREO_CHECKED = 3   # confirmed, and both sensors have seen it

LEVEL_NAMES = {LEVEL_TENTATIVE: 'tentative',
               LEVEL_CONFIRMED: 'confirmed',
               LEVEL_LIDAR_CONFIRMED: 'lidar_confirmed',
               LEVEL_STEREO_CHECKED: 'stereo_checked'}
LEVEL_BY_NAME = {v: k for k, v in LEVEL_NAMES.items()}

#: MultiWallTrack.STATE_* / SOURCE_*, spelled out rather than imported: this
#: module is numpy-only and must stay importable without f1tenth_messages
#: (the tests and tools/semantic_door_replay.py both rely on that).
STATE_TENTATIVE, STATE_CONFIRMED, STATE_COASTING = 0, 1, 2
SOURCE_LIDAR, SOURCE_STEREO, SOURCE_BOTH = 0, 1, 2


def wall_level(state, source):
    """Return the LEVEL_* of a wall track with this `state` and `source`.

    COASTING counts as confirmed: the track was confirmed and is being held
    through a few scans it did not match, which is the tracker saying the
    wall is still there, not that it has become doubtful.
    """
    if int(state) == STATE_TENTATIVE:
        return LEVEL_TENTATIVE
    if int(source) == SOURCE_BOTH:
        return LEVEL_STEREO_CHECKED
    if int(source) == SOURCE_LIDAR:
        return LEVEL_LIDAR_CONFIRMED
    return LEVEL_CONFIRMED  # confirmed, stereo only


class Wall:
    """One semantic-layer wall: a map-frame segment with its track identity."""

    __slots__ = ('id', 'role', 'p0', 'p1', 'level')

    def __init__(self, id, p0, p1, level=LEVEL_LIDAR_CONFIRMED, role='unknown'):
        self.id = int(id)
        self.role = str(role)
        self.p0 = (float(p0[0]), float(p0[1]))
        self.p1 = (float(p1[0]), float(p1[1]))
        self.level = int(level)

    @property
    def length(self):
        return math.hypot(self.p1[0] - self.p0[0], self.p1[1] - self.p0[1])

    def direction(self):
        """Unit vector p0 -> p1, or None for a degenerate segment."""
        n = self.length
        if n < 1e-9:
            return None
        return ((self.p1[0] - self.p0[0]) / n, (self.p1[1] - self.p0[1]) / n)

    def __repr__(self):
        return (f'Wall(id={self.id}, p0={self.p0}, p1={self.p1}, '
                f'level={LEVEL_NAMES.get(self.level, self.level)})')


class DoorParams:
    """Every threshold of this module, one object, all SI.

    Defaults are the shipped ones; stack_params.yaml is the single source for
    what the car actually runs (semantic_layer_node builds this from it).
    """

    __slots__ = ('min_wall_level', 'width_min_m', 'width_max_m',
                 'parallel_angle_deg', 'parallel_offset_m', 'perp_tol_deg',
                 'foot_inset_m', 'end_fov_margin_rad', 'end_range_margin_m',
                 'end_occlusion_margin_m', 'end_probe_beams', 'min_beams',
                 'through_frac', 'behind_margin_m', 'gap_clear_m',
                 'jamb_exclude_m', 'assoc_max_m', 'confirm_hits',
                 'confirm_pose_spread_m', 'confirm_same_pose_hits', 'max_tracks',
                 'merge_tracks_overlap', 'merge_tracks_parallel_m',
                 'merge_tracks_yaw_deg',
                 'strong_min_hits', 'strong_min_spread_m',
                 'smooth_alpha', 'candidate_forget_s', 'forget_unseen_s',
                 'max_walls')

    def __init__(self, **kw):
        self.min_wall_level = LEVEL_LIDAR_CONFIRMED
        self.width_min_m = 0.70
        self.width_max_m = 1.40
        self.parallel_angle_deg = 10.0
        self.parallel_offset_m = 0.15
        self.perp_tol_deg = 15.0
        self.foot_inset_m = 0.10
        self.end_fov_margin_rad = math.radians(2.0)
        self.end_range_margin_m = 0.50
        self.end_occlusion_margin_m = 0.30
        self.end_probe_beams = 6
        self.min_beams = 4
        self.through_frac = 0.60
        self.behind_margin_m = 0.30
        self.gap_clear_m = 0.12
        self.jamb_exclude_m = 0.08
        self.assoc_max_m = 0.30
        self.confirm_hits = 5
        # POSE-DIVERSITY CONFIRMATION, off unless confirm_pose_spread_m > 0.
        # The plain hit count asks "seen this often"; a detector whose input
        # is a single scan needs "seen from somewhere else", because a ray
        # artefact is perfectly repeatable from one standing position and
        # counting it five times proves nothing. With the spread set, a track
        # confirms when two of its hits came from robot positions at least
        # that far apart, OR after confirm_same_pose_hits hits from wherever
        # (so a stationary car still eventually confirms a real door).
        self.confirm_pose_spread_m = 0.0
        self.confirm_same_pose_hits = 0
        # A BUDGET, not a detection limit. A confirmed door is never
        # forgotten (forget_unseen_s 0), which is what lets a mission drive
        # back to one -- but it also means the list only grows, and every
        # confirmed door costs a TF and five markers on every cycle. A
        # permissive detector on a 19 m run through a cluttered room
        # produced 98 of them (measured, output/doors/freespace.md), i.e.
        # about a thousand transforms a second into the tree. 0 is unbounded,
        # which is what the wall detectors keep: they are capped upstream by
        # /perception/walls carrying at most four tracks.
        self.max_tracks = 0
        # MERGING TWO TRACKS THAT ARE THE SAME DOOR. Association
        # (assoc_max_m) only ever folds a candidate into a track; two tracks,
        # once both exist, never meet again. On corridor10m_20261019T131815
        # that left 23 overlapping pairs among 30 strong doors -- ids 5 and 15
        # sitting 0.30 m apart with the same axis and 307 against 48 hits,
        # i.e. one doorway published twice, because one candidate once landed
        # a hair over assoc_max_m from the track that already had it.
        #
        # Two tests, because the two failure shapes differ:
        #
        # merge_tracks_overlap -- TWO DOORWAYS CANNOT OVERLAP IN SPACE. Two
        # openings this wide whose centres are closer than the mean of their
        # widths would have to share a hole in the wall, so one of them is
        # false whatever the axes say, and dropping the weaker hypothesis is
        # strictly better than publishing both. Scale-free, which a fixed
        # radius is not: it allows 1.2 m between two 1.2 m doorways and only
        # 0.7 m between two narrow ones. The measured clusters were exactly
        # this shape -- four tracks inside 0.6 m at four different angles, up
        # to 97 deg apart.
        #
        # merge_tracks_parallel_m -- a duplicate of a SIDE door drifts ALONG
        # the wall rather than across it, latching onto one real jamb and
        # something else, so its copy can sit further away than the width
        # (measured pairs at 0.89, 0.90 and 0.91 m) and is only recognisable
        # together with its axis. Requiring the axes to agree is what keeps
        # this from fusing two genuinely different openings a metre apart.
        #
        # Both at 0 is the wall detectors' setting, where /perception/walls
        # carries at most four walls and cannot produce either shape.
        self.merge_tracks_overlap = 0.0
        self.merge_tracks_parallel_m = 0.0
        self.merge_tracks_yaw_deg = 25.0
        # WHAT MAKES A CONFIRMED DOOR WORTH DRIVING AT. Confirmed means
        # publishable; STRONG means a mission may select it (enter_door
        # resolves over strong doors only). Both zero -- the wall detectors'
        # setting -- makes every confirmed door strong, which is exactly the
        # behaviour they had before the field existed. A detector whose
        # evidence is one scan sets them, because there a doorway and a gap
        # between two pieces of furniture look nearly identical and only
        # being seen from somewhere else tells them apart.
        self.strong_min_hits = 0
        self.strong_min_spread_m = 0.0
        self.smooth_alpha = 0.30
        self.candidate_forget_s = 5.0
        self.forget_unseen_s = 0.0   # 0 = a confirmed door is never forgotten
        self.max_walls = 12
        for k, v in kw.items():
            if k not in self.__slots__:
                raise TypeError(f'DoorParams got an unknown parameter {k!r}')
            setattr(self, k, v)


#: Candidate codes, grouped the way the docstring's four gates are. '' is a
#: door. A consumer that wants "was this geometry at all plausible" asks for
#: the group, not the code, so a new code never breaks it.
GEOMETRY_CODES = ('angle', 'offset', 'width', 'overlap', 'foot_outside')
END_CODES = ('end_fov', 'end_occluded')
PASSAGE_CODES = ('few_beams', 'not_through', 'on_segment', 'object')


def code_group(code):
    """'ok' | 'geometry' | 'ends' | 'passage' for a candidate's code."""
    if not code:
        return 'ok'
    if code in GEOMETRY_CODES:
        return 'geometry'
    if code in END_CODES:
        return 'ends'
    return 'passage'


class DoorCandidate:
    """One evaluated gap: its geometry, where it came from, and its verdict."""

    __slots__ = ('jamb0', 'jamb1', 'centre', 'width', 'axis', 'case',
                 'wall_id0', 'wall_id1', 'code', 'n_beams', 'through_frac',
                 'flags')

    def __init__(self, jamb0, jamb1, case, wall_id0, wall_id1, code='',
                 n_beams=0, through_frac=0.0, flags=()):
        self.jamb0 = (float(jamb0[0]), float(jamb0[1]))
        self.jamb1 = (float(jamb1[0]), float(jamb1[1]))
        self.centre = ((self.jamb0[0] + self.jamb1[0]) / 2.0,
                       (self.jamb0[1] + self.jamb1[1]) / 2.0)
        dx, dy = self.jamb1[0] - self.jamb0[0], self.jamb1[1] - self.jamb0[1]
        self.width = math.hypot(dx, dy)
        self.axis = (dx / self.width, dy / self.width) if self.width > 1e-9 else (1.0, 0.0)
        self.case = case          # 'A' (parallel) or 'B' (90 degrees)
        self.wall_id0 = int(wall_id0)
        self.wall_id1 = int(wall_id1)
        self.code = code
        self.n_beams = int(n_beams)
        self.through_frac = float(through_frac)
        # Accepted-but-noteworthy, as opposed to `code` which is rejected-
        # and-why. A flagged candidate IS a door ('marginal' width, a
        # 'one_jamb' opening whose second jamb the sensor never saw) and is
        # tracked and published like any other; the flag rides along so the
        # markers can say so and the debug dict can count it.
        self.flags = tuple(flags)

    @property
    def ok(self):
        return self.code == ''

    def normal(self):
        """A unit vector perpendicular to the jamb line (sign not decided here)."""
        return (-self.axis[1], self.axis[0])

    def __repr__(self):
        return (f'DoorCandidate(case={self.case}, w={self.width:.2f}, '
                f'walls=({self.wall_id0}, {self.wall_id1}), code={self.code!r}'
                + (f', flags={self.flags}' if self.flags else '') + ')')


# ---------------------------------------------------------------------------
# Geometry
# ---------------------------------------------------------------------------
def _angle_between_lines(u, v):
    """Smallest angle [deg] between two undirected directions, 0..90."""
    dot = abs(u[0] * v[0] + u[1] * v[1])
    return math.degrees(math.acos(max(-1.0, min(1.0, dot))))


def _facing_ends(a, b):
    """The closest endpoint pair ((pa, other_a), (pb, other_b)) of two walls."""
    best = None
    for pa, oa in ((a.p0, a.p1), (a.p1, a.p0)):
        for pb, ob in ((b.p0, b.p1), (b.p1, b.p0)):
            d = math.hypot(pb[0] - pa[0], pb[1] - pa[1])
            if best is None or d < best[0]:
                best = (d, (pa, oa), (pb, ob))
    return best[1], best[2]


def _case_a(a, b, p):
    """Evaluate the parallel/collinear pair (a, b); return a DoorCandidate.

    The collinearity is measured AT THE GAP, not between the two infinite
    lines: the across-line component of the vector between the two facing
    ends. Two long walls a degree apart are collinear where they meet and
    metres apart at their far ends, and it is where they meet that decides
    whether a doorway sits between them.
    """
    ua, ub = a.direction(), b.direction()
    if ua is None or ub is None:
        return None
    angle = _angle_between_lines(ua, ub)
    (pa, oa), (pb, ob) = _facing_ends(a, b)
    cand = DoorCandidate(pa, pb, 'A', a.id, b.id)
    if angle > p.parallel_angle_deg:
        cand.code = 'angle'
        return cand
    # One direction for the pair, b's flipped onto a's if it runs the other way.
    sign = 1.0 if (ua[0] * ub[0] + ua[1] * ub[1]) >= 0.0 else -1.0
    ux, uy = ua[0] + sign * ub[0], ua[1] + sign * ub[1]
    n = math.hypot(ux, uy)
    if n < 1e-9:
        cand.code = 'angle'
        return cand
    ux, uy = ux / n, uy / n
    gx, gy = pb[0] - pa[0], pb[1] - pa[1]
    along = gx * ux + gy * uy
    lateral = abs(gx * -uy + gy * ux)
    if lateral > p.parallel_offset_m:
        cand.code = 'offset'
        return cand
    # The two walls must lie on OPPOSITE sides of the gap: each one's far end
    # must be behind its own facing end, looking along the gap. Otherwise the
    # extents overlap and there is no gap between them at all.
    if ((pa[0] - oa[0]) * ux + (pa[1] - oa[1]) * uy) * along <= 0.0:
        cand.code = 'overlap'
        return cand
    if ((pb[0] - ob[0]) * ux + (pb[1] - ob[1]) * uy) * along >= 0.0:
        cand.code = 'overlap'
        return cand
    if not (p.width_min_m <= cand.width <= p.width_max_m):
        cand.code = 'width'
    return cand


def _case_b(a, b, p):
    """Evaluate every end of `a` against the perpendicular wall `b`.

    Yields a DoorCandidate per end of `a`. The gap is the PERPENDICULAR
    distance from that end to b's plane, and the far jamb is the foot of that
    perpendicular -- a point on b's surface, which is why the foot must lie
    strictly inside b's extent rather than pass real_end(): a foot at b's own
    end means two unknown ends facing each other across a diagonal, which is
    a corner.
    """
    ua, ub = a.direction(), b.direction()
    if ua is None or ub is None:
        return
    angle = _angle_between_lines(ua, ub)
    for end in (a.p0, a.p1):
        # The foot of the perpendicular from `end` onto b's infinite line.
        t = ((end[0] - b.p0[0]) * ub[0] + (end[1] - b.p0[1]) * ub[1])
        foot = (b.p0[0] + t * ub[0], b.p0[1] + t * ub[1])
        cand = DoorCandidate(end, foot, 'B', a.id, b.id)
        if abs(angle - 90.0) > p.perp_tol_deg:
            cand.code = 'angle'
            yield cand
            continue
        if not (p.foot_inset_m <= t <= b.length - p.foot_inset_m):
            cand.code = 'foot_outside'
            yield cand
            continue
        if not (p.width_min_m <= cand.width <= p.width_max_m):
            cand.code = 'width'
        yield cand


# ---------------------------------------------------------------------------
# The scan: real ends, and the passage
# ---------------------------------------------------------------------------
class ScanView:
    """One scan, ready for the two questions this module asks of it.

    Everything is BASE_LINK, because that is the frame the beams are cheap in
    (unknown_objects.scan_points already puts them there) and the only frame
    where a bearing means anything. The caller converts the walls and the
    objects into it with unknown_objects.to_base_link.
    """

    __slots__ = ('bearings', 'ranges', 'laser', 'range_max', 'points')

    def __init__(self, bearings, ranges, laser_xy, range_max):
        self.bearings = np.asarray(bearings, dtype=float)
        self.ranges = np.asarray(ranges, dtype=float)
        self.laser = (float(laser_xy[0]), float(laser_xy[1]))
        self.range_max = float(range_max)
        finite = np.isfinite(self.ranges)
        self.points = np.column_stack([
            self.laser[0] + self.ranges[finite] * np.cos(self.bearings[finite]),
            self.laser[1] + self.ranges[finite] * np.sin(self.bearings[finite])])

    def polar(self, xy):
        """(bearing, range) of a base_link point as the LASER sees it."""
        dx, dy = xy[0] - self.laser[0], xy[1] - self.laser[1]
        return math.atan2(dy, dx), math.hypot(dx, dy)


def real_end(point_bl, inward_bl, scan, p):
    """Is this wall end a real end, or the edge of the scan / of a shadow?

    `point_bl` is the end, `inward_bl` a point on the GAP side of it (the
    opening's centre): the probe looks just past the end, into the gap, where
    an opening must show either nothing or something FURTHER away. A nearer
    return there is the giveaway of a shadow edge -- the wall carries on
    behind whatever is closer, and what looks like a jamb is that nearer
    thing's own silhouette.

    Returns '' for a real end, or the failing code.
    """
    bearing, rng = scan.polar(point_bl)
    if scan.bearings.size < 2:
        return 'end_fov'
    lo, hi = float(scan.bearings.min()), float(scan.bearings.max())
    if bearing - lo < p.end_fov_margin_rad or hi - bearing < p.end_fov_margin_rad:
        return 'end_fov'
    if rng > scan.range_max - p.end_range_margin_m:
        return 'end_fov'
    gap_bearing, _ = scan.polar(inward_bl)
    step = math.atan2(math.sin(gap_bearing - bearing), math.cos(gap_bearing - bearing))
    if abs(step) < 1e-9:
        return ''
    # The first end_probe_beams beams past the end, on the gap side.
    delta = np.angle(np.exp(1j * (scan.bearings - bearing)))
    side = delta * (1.0 if step > 0 else -1.0)
    probe = np.argsort(np.where(side > 0.0, side, np.inf))[:p.end_probe_beams]
    probe = probe[np.isfinite(np.where(side[probe] > 0.0, side[probe], np.inf))]
    if probe.size == 0:
        return 'end_fov'
    r = scan.ranges[probe]
    nearer = np.isfinite(r) & (r < rng - p.end_occlusion_margin_m)
    return 'end_occluded' if bool(np.any(nearer)) else ''


def _shrunk(j0, j1, margin):
    """The jamb segment with `margin` taken off each end (the jambs' own returns)."""
    dx, dy = j1[0] - j0[0], j1[1] - j0[1]
    n = math.hypot(dx, dy)
    if n <= 2.0 * margin:
        return None
    ux, uy = dx / n, dy / n
    return ((j0[0] + ux * margin, j0[1] + uy * margin),
            (j1[0] - ux * margin, j1[1] - uy * margin))


def _point_segment_distance(pts, a, b):
    """Distance from each row of `pts` (N, 2) to the segment a -> b."""
    pts = np.asarray(pts, dtype=float).reshape(-1, 2)
    a = np.asarray(a, dtype=float)
    e = np.asarray(b, dtype=float) - a
    n2 = float(e @ e)
    if n2 < 1e-12:
        return np.hypot(pts[:, 0] - a[0], pts[:, 1] - a[1])
    t = np.clip(((pts - a) @ e) / n2, 0.0, 1.0)
    foot = a + t[:, None] * e
    return np.hypot(pts[:, 0] - foot[:, 0], pts[:, 1] - foot[:, 1])


def _on_segment(pts, a, b, tol):
    """Rows of `pts` lying ON the segment a -> b: within `tol` PERPENDICULARLY
    and projecting inside it.

    Perpendicular and along-track are separated deliberately, and this is the
    whole reason there is a function here rather than a call to
    _point_segment_distance: the JAMBS' OWN RETURNS sit at the segment's two
    ends, and a plain point-to-segment distance cannot tell "0.1 m out along
    the wall, past the jamb" (a wall return, expected) from "0.1 m into the
    opening, off the line" (something standing in the doorway). The along
    axis is cut by jamb_exclude_m before this is called; the perpendicular
    one is gap_clear_m.
    """
    pts = np.asarray(pts, dtype=float).reshape(-1, 2)
    if pts.shape[0] == 0:
        return np.zeros(0, dtype=bool)
    a = np.asarray(a, dtype=float)
    e = np.asarray(b, dtype=float) - a
    n = math.hypot(e[0], e[1])
    if n < 1e-9:
        return np.zeros(pts.shape[0], dtype=bool)
    u = e / n
    d = pts - a
    along = d @ u
    perp = np.abs(d[:, 0] * -u[1] + d[:, 1] * u[0])
    return (perp <= tol) & (along >= 0.0) & (along <= n)


def passage(j0_bl, j1_bl, scan, objects_bl, p):
    """Judge the opening between two base_link jambs; return (code, n, frac).

    FOUR THINGS, in the docstring's order: enough beams aimed into the gap,
    enough of them reaching BEYOND the opening line, nothing returning ON the
    line, and no semantic-layer object standing in it. All depths are
    PERPENDICULAR to the opening line, never along the ray: a return at range
    r on a ray meeting the line at s, at incidence cos(phi), lies
    (r - s) cos(phi) behind it.
    """
    seg = _shrunk(j0_bl, j1_bl, p.jamb_exclude_m)
    if seg is None:
        return 'few_beams', 0, 0.0
    a, b = seg
    b0, r0 = scan.polar(j0_bl)
    b1, r1 = scan.polar(j1_bl)
    # The beams whose bearing falls inside the gap's own angular span.
    half = abs(math.atan2(math.sin(b1 - b0), math.cos(b1 - b0))) / 2.0
    mid = b0 + math.atan2(math.sin(b1 - b0), math.cos(b1 - b0)) / 2.0
    delta = np.angle(np.exp(1j * (scan.bearings - mid)))
    inside = np.abs(delta) <= half
    n_beams = int(inside.sum())
    if n_beams < p.min_beams:
        return 'few_beams', n_beams, 0.0

    # Perpendicular depth of each in-gap beam past the opening line.
    ax, ay = j1_bl[0] - j0_bl[0], j1_bl[1] - j0_bl[1]
    an = math.hypot(ax, ay)
    nx, ny = -ay / an, ax / an
    # Orient the normal away from the laser, so "beyond" is positive.
    if (j0_bl[0] - scan.laser[0]) * nx + (j0_bl[1] - scan.laser[1]) * ny < 0.0:
        nx, ny = -nx, -ny
    line_offset = (j0_bl[0] - scan.laser[0]) * nx + (j0_bl[1] - scan.laser[1]) * ny
    r = scan.ranges[inside]
    cos_phi = np.cos(scan.bearings[inside]) * nx + np.sin(scan.bearings[inside]) * ny
    with np.errstate(divide='ignore', invalid='ignore'):
        depth = r * cos_phi - line_offset
    through = ~np.isfinite(r) | (depth > p.behind_margin_m)
    frac = float(through.mean())
    if frac < p.through_frac:
        return 'not_through', n_beams, frac

    if scan.points.shape[0] and bool(np.any(_on_segment(
            scan.points, a, b, p.gap_clear_m))):
        return 'on_segment', n_beams, frac

    for ox, oy, orad in objects_bl:
        if float(_point_segment_distance([[ox, oy]], a, b)[0]) <= float(orad):
            return 'object', n_beams, frac
    return '', n_beams, frac


# ---------------------------------------------------------------------------
# One cycle
# ---------------------------------------------------------------------------
def eligible_walls(walls, p):
    """The walls at or above door_min_wall_level, longest first, capped.

    The cap (max_walls) bounds the pair loop: the gates below are cheap, but
    the loop is quadratic, and /perception/walls plus the semantic layer's
    long raw observations can put more segments in front of it than any room
    has walls. The longest are the ones a doorway sits in.
    """
    ok = [w for w in walls if w.level >= p.min_wall_level and w.length > 1e-6]
    ok.sort(key=lambda w: -w.length)
    return ok[:max(0, int(p.max_walls))]


def candidate_doors(walls, scan, objects_bl, p, to_base_link=None):
    """Every gap of this cycle, each with its code; the doors are the `ok` ones.

    `walls` are MAP-frame Wall records (the semantic layer's own), `scan` a
    base_link ScanView, `objects_bl` an iterable of (x, y, radius) in
    base_link. `to_base_link` maps a map point to base_link; without it the
    walls are taken to be in base_link already (what the unit tests do).
    """
    tf = to_base_link or (lambda xy: xy)
    out = []
    for a, b in itertools.combinations(eligible_walls(walls, p), 2):
        cands = [c for c in (_case_a(a, b, p),) if c is not None]
        cands += list(_case_b(a, b, p)) + list(_case_b(b, a, p))
        for cand in cands:
            if cand.code:
                out.append(cand)
                continue
            j0, j1 = tf(cand.jamb0), tf(cand.jamb1)
            centre = tf(cand.centre)
            # Case A: both jambs are wall ends. Case B: only the first is --
            # the second is the foot on the perpendicular wall's surface,
            # already required to lie inside its extent.
            ends = (j0, j1) if cand.case == 'A' else (j0,)
            code = ''
            for e in ends:
                code = real_end(e, centre, scan, p)
                if code:
                    break
            if code:
                cand.code = code
                out.append(cand)
                continue
            cand.code, cand.n_beams, cand.through_frac = passage(
                j0, j1, scan, objects_bl, p)
            out.append(cand)
    return out


# ---------------------------------------------------------------------------
# Tracking
# ---------------------------------------------------------------------------
STATE_CANDIDATE, STATE_DOOR_CONFIRMED = 0, 1


class DoorTrack:
    """One tracked door: a stable id, a smoothed pose, and a fixed direction."""

    __slots__ = ('id', 'centre', 'axis', 'width', 'normal', 'hits', 'state',
                 'born_sec', 'last_seen_sec', 'confirmed_sec', 'case',
                 'wall_id0', 'wall_id1', 'spread_pair', 'spread', 'flags',
                 'strong')

    def __init__(self, track_id, cand, now_sec):
        self.id = int(track_id)
        self.centre = cand.centre
        self.axis = cand.axis
        self.width = cand.width
        self.normal = None            # decided at first confirmation
        self.hits = 1
        self.state = STATE_CANDIDATE
        self.born_sec = float(now_sec)
        self.last_seen_sec = float(now_sec)
        self.confirmed_sec = None
        self.case = cand.case
        self.wall_id0, self.wall_id1 = cand.wall_id0, cand.wall_id1
        self.flags = tuple(cand.flags)
        # The two poses furthest apart that this track has been seen from,
        # and their distance. A RUNNING DIAMETER, not a list of poses: the
        # first version kept the first 16 poses and the distance between
        # them, which reported 0.00 m spread for a track with 300 hits --
        # born while the car was standing still, the list filled with 16
        # copies of one position, and every later pose, 19 m up the
        # corridor, discarded by the cap. Measured on a real bag.
        self.spread_pair = None
        self.spread = 0.0
        self.strong = False      # recomputed by the tracker on every sighting

    def remember_pose(self, robot_xy):
        """Fold one sighting's robot position into the running diameter."""
        if robot_xy is None:
            return
        xy = (float(robot_xy[0]), float(robot_xy[1]))
        if self.spread_pair is None:
            self.spread_pair = (xy, xy)
            return
        a, b = self.spread_pair
        da = math.hypot(xy[0] - a[0], xy[1] - a[1])
        db = math.hypot(xy[0] - b[0], xy[1] - b[1])
        if da >= db and da > self.spread:
            self.spread_pair, self.spread = (a, xy), da
        elif db > self.spread:
            self.spread_pair, self.spread = (b, xy), db

    def pose_spread(self):
        """Largest distance between any two poses this track was seen from."""
        return self.spread

    @property
    def confirmed(self):
        return self.state == STATE_DOOR_CONFIRMED

    def jambs(self):
        """The two jamb points, from the smoothed centre, axis and width."""
        hx, hy = self.axis[0] * self.width / 2.0, self.axis[1] * self.width / 2.0
        return ((self.centre[0] - hx, self.centre[1] - hy),
                (self.centre[0] + hx, self.centre[1] + hy))

    def yaw(self):
        """Heading [rad] of the door frame's x axis (the crossing direction)."""
        n = self.normal or (-self.axis[1], self.axis[0])
        return math.atan2(n[1], n[0])

    def frame_id(self):
        return f'door_{self.id}'

    def __repr__(self):
        return (f'DoorTrack(id={self.id}, w={self.width:.2f}, hits={self.hits}, '
                f'{"CONFIRMED" if self.confirmed else "candidate"})')


def _blend_axis(old, new, alpha):
    """EMA of two undirected axes, `new` flipped onto `old` first.

    An axis has no sign: the wall fit may hand back p0/p1 the other way round
    on any scan, and averaging that naively collapses the axis to zero.
    """
    sign = 1.0 if (old[0] * new[0] + old[1] * new[1]) >= 0.0 else -1.0
    x = (1.0 - alpha) * old[0] + alpha * sign * new[0]
    y = (1.0 - alpha) * old[1] + alpha * sign * new[1]
    n = math.hypot(x, y)
    return old if n < 1e-9 else (x / n, y / n)


class DoorTracker:
    """The door list: association, the confirm lifecycle, the fixed direction."""

    def __init__(self, params=None, first_id=1):
        self.p = params or DoorParams()
        self.tracks = []
        self._next_id = int(first_id)

    def confirmed(self):
        return [t for t in self.tracks if t.confirmed]

    def by_id(self, door_id):
        for t in self.tracks:
            if t.id == door_id:
                return t
        return None

    def update(self, candidates, robot_xy, now_sec):
        """Fold this cycle's candidates in; return the confirmed doors.

        `robot_xy` is the car in the same (map) frame as the candidates: it is
        read ONLY to orient a door's normal at the moment it is confirmed.
        """
        p = self.p
        doors = [c for c in candidates if c.ok]
        pairs = []
        for ci, cand in enumerate(doors):
            for ti, trk in enumerate(self.tracks):
                d = math.hypot(cand.centre[0] - trk.centre[0],
                               cand.centre[1] - trk.centre[1])
                if d <= p.assoc_max_m or self._same_door(cand, trk, d):
                    pairs.append((d, ci, ti))
        pairs.sort()
        taken_c, taken_t = set(), set()
        for d, ci, ti in pairs:
            if ci in taken_c or ti in taken_t:
                continue
            taken_c.add(ci)
            taken_t.add(ti)
            # Inside assoc_max_m the candidate is this door seen again and is
            # blended in. Outside it, it only passed the SAME-DOOR test
            # (DoorParams' two merge thresholds), so it is one opening seen
            # badly: it counts as a sighting but must not drag the track's
            # geometry a third of the way towards itself, which is what the
            # EMA would do with a candidate a metre off.
            #
            # Associating over the wider test at all is what keeps ids
            # stable. Letting the track be born and then merged away instead
            # churned 1024 ids over 506 cycles of
            # corridor10m_20261019T131815 for a final 19 doors -- a thousand
            # door_<id> frames in the TF tree, and nothing a mission could
            # name twice.
            self._hit(self.tracks[ti], doors[ci], robot_xy, now_sec,
                      blend=d <= p.assoc_max_m)
        for ci, cand in enumerate(doors):
            if ci in taken_c:
                continue
            trk = DoorTrack(self._next_id, cand, now_sec)
            trk.remember_pose(robot_xy)
            trk.strong = self.is_strong(trk)
            self._next_id += 1
            self.tracks.append(trk)
            if self.should_confirm(trk):
                self._confirm(trk, robot_xy, now_sec)
        self._forget(now_sec)
        return self.confirmed()

    def _hit(self, trk, cand, robot_xy, now_sec, blend=True):
        a = self.p.smooth_alpha
        if blend:
            trk.centre = ((1.0 - a) * trk.centre[0] + a * cand.centre[0],
                          (1.0 - a) * trk.centre[1] + a * cand.centre[1])
            trk.axis = _blend_axis(trk.axis, cand.axis, a)
            trk.width = (1.0 - a) * trk.width + a * cand.width
            trk.case = cand.case
            trk.wall_id0, trk.wall_id1 = cand.wall_id0, cand.wall_id1
            trk.flags = cand.flags
        trk.hits += 1
        trk.last_seen_sec = float(now_sec)
        trk.remember_pose(robot_xy)
        trk.strong = self.is_strong(trk)
        if not trk.confirmed and self.should_confirm(trk):
            self._confirm(trk, robot_xy, now_sec)

    def is_strong(self, trk):
        """Has this track earned being driven at? See DoorParams.

        No flags, enough sightings, and enough distance between the two robot
        positions furthest apart that saw it. With both thresholds at zero
        (the wall detectors) this is just "confirmed and unflagged", and the
        wall detectors never set a flag.
        """
        p = self.p
        if trk.flags:
            return False
        if p.strong_min_hits and trk.hits < p.strong_min_hits:
            return False
        return trk.pose_spread() >= p.strong_min_spread_m

    def should_confirm(self, trk):
        """Is this track ready to be published? See DoorParams' two knobs.

        Plain hit counting unless confirm_pose_spread_m is set, which is what
        the wall-based detectors use and what this tracker did before the
        grid/scan one existed. With the spread set: two hits from robot
        positions that far apart, OR confirm_same_pose_hits hits from
        anywhere -- "seen from somewhere else", which a repeatable one-scan
        artefact cannot satisfy while the car is moving, with a plain count
        as the fallback for a car that is not.
        """
        p = self.p
        if p.confirm_pose_spread_m > 0.0:
            if p.confirm_same_pose_hits and trk.hits >= p.confirm_same_pose_hits:
                return True
            return trk.pose_spread() >= p.confirm_pose_spread_m
        return trk.hits >= p.confirm_hits

    def _confirm(self, trk, robot_xy, now_sec):
        """Confirm a track and FIX its crossing direction, once and for all."""
        trk.state = STATE_DOOR_CONFIRMED
        trk.confirmed_sec = float(now_sec)
        nx, ny = -trk.axis[1], trk.axis[0]
        if robot_xy is not None:
            # From the car's side of the opening to the far side, at this one
            # moment. Never recomputed: see the module docstring.
            away = ((trk.centre[0] - robot_xy[0]) * nx
                    + (trk.centre[1] - robot_xy[1]) * ny)
            if away < 0.0:
                nx, ny = -nx, -ny
        trk.normal = (nx, ny)

    def _forget(self, now_sec):
        keep = []
        for t in self.tracks:
            unseen = float(now_sec) - t.last_seen_sec
            if t.confirmed:
                # Kept in the map after leaving view, unless asked otherwise.
                if self.p.forget_unseen_s > 0.0 and unseen > self.p.forget_unseen_s:
                    continue
            elif unseen > self.p.candidate_forget_s:
                continue
            keep.append(t)
        self.tracks = self._capped(self._merged(keep))

    def _merged(self, tracks):
        """Collapse tracks that are the same door. See DoorParams' two tests.

        The survivor is the better-evidence one (confirmed, then strong, then
        sightings, then pose spread, then the older id) and it keeps its own
        geometry, flags and fixed direction: the loser is a worse view of the
        same opening, not half of a better one. It absorbs only what is
        strictly evidence -- the loser's pose-diameter endpoints, its hit
        count if that is higher (MAX, not sum: in a cycle where both tracks
        took a hit from the same scan, summing would count that scan twice),
        the earlier born_sec and the later last_seen_sec.

        Closest pair first, repeatedly, because a cluster of four collapses
        pairwise and merging the wrong two first would average across the
        cluster.
        """
        p = self.p
        if p.merge_tracks_overlap <= 0.0 and p.merge_tracks_parallel_m <= 0.0:
            return tracks
        tracks = list(tracks)
        while True:
            best = None
            for i in range(len(tracks)):
                for j in range(i + 1, len(tracks)):
                    a, b = tracks[i], tracks[j]
                    d = math.hypot(a.centre[0] - b.centre[0],
                                   a.centre[1] - b.centre[1])
                    if best is not None and d >= best[0]:
                        continue
                    if not self._same_door(a, b, d):
                        continue
                    best = (d, i, j)
            if best is None:
                return tracks
            keep, drop = sorted((tracks[best[1]], tracks[best[2]]),
                                key=self._evidence, reverse=True)
            self._absorb(keep, drop)
            tracks = [t for t in tracks if t is not drop]

    def _same_door(self, a, b, d):
        """Are these two tracks one opening? Either test may say so."""
        p = self.p
        if (p.merge_tracks_overlap > 0.0
                and d <= p.merge_tracks_overlap * (a.width + b.width) / 2.0):
            return True
        return (p.merge_tracks_parallel_m > 0.0
                and d <= p.merge_tracks_parallel_m
                and _angle_between_lines(a.axis, b.axis) <= p.merge_tracks_yaw_deg)

    @staticmethod
    def _evidence(trk):
        """Merge ranking: who survives. Older id last, so ids stay stable."""
        return (trk.confirmed, trk.strong, trk.hits, trk.pose_spread(), -trk.id)

    def _absorb(self, keep, drop):
        """Fold the losing track's evidence into the surviving one.

        Not its geometry: `keep` already has the better view of it. Strength
        is recomputed because the pose diameter can only grow here; the
        confirm decision is left to the next sighting, which brings a robot
        position with it and so can orient the crossing direction.
        """
        if drop.spread_pair is not None:
            for xy in drop.spread_pair:
                keep.remember_pose(xy)
        keep.hits = max(keep.hits, drop.hits)
        keep.born_sec = min(keep.born_sec, drop.born_sec)
        keep.last_seen_sec = max(keep.last_seen_sec, drop.last_seen_sec)
        keep.strong = self.is_strong(keep)

    def _capped(self, tracks):
        """Enforce max_tracks, dropping the LONGEST UNSEEN first.

        RECENCY FIRST, not evidence first. The obvious ranking -- confirmed
        before unconfirmed, then most sightings -- was measured to be the
        wrong one: a long-lived door with 300 hits always outranked a young
        but genuine candidate, so the budget evicted candidates BEFORE they
        could earn strength, and on corridor10m_20261019T131815 a cap of 32
        cost 10 of the 29 strong doors the uncapped run found. The budget was
        fighting the detector.

        Keeping what the car can currently see fixes that: a door being
        driven past has a fresh last_seen_sec and survives long enough to
        earn its strength, while the eviction falls on doors long out of
        view -- which is what a budget should sacrifice, and makes "kept in
        the map after leaving view" a convenience rather than a guarantee.
        Evidence still breaks ties, so of two equally stale doors the
        better-seen one stays.

        A cap that is hit is a tuning signal, not silent corruption:
        door_candidates on the debug topic sits pinned at the limit.
        """
        limit = int(self.p.max_tracks)
        if limit <= 0 or len(tracks) <= limit:
            return tracks
        ranked = sorted(tracks, key=lambda t: (
            t.last_seen_sec, t.confirmed, t.hits, t.pose_spread()), reverse=True)
        return ranked[:limit]


def nearest_confirmed(tracker, xy):
    """The confirmed door nearest `xy`, or None. What "door": "nearest" means."""
    best, best_d = None, None
    for t in tracker.confirmed():
        d = math.hypot(t.centre[0] - xy[0], t.centre[1] - xy[1])
        if best_d is None or d < best_d:
            best, best_d = t, d
    return best
