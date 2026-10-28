"""Doors from what the sensor SEES THROUGH, not from a shape in the map.

WHY THIS, AFTER A SKELETON DETECTOR. The first version of this module cropped
slam_toolbox's grid, took the distance transform of the free space, thinned it
to its medial axis and looked for a bottleneck whose clearance rose on both
sides. Replayed on the real map it found nothing, and the reason is structural:
A ROOM BEHIND A DOOR IS NOT A ROOM IN THE GRID. It is a ray fan -- a handful of
beams that went through the opening, each marking a thin line of free cells,
with UNKNOWN between them, because nothing has ever driven in there to fill it.
So the far side never becomes a wide region, "clearance rises on both sides"
never holds, and the few skeleton cells that do reach through the doorway are
short branches off the corridor's axis, which the spur pruning throws away. The
detector was asking the map a question the map cannot answer until after you
have already driven through the door.

THE SCAN CAN ANSWER IT IMMEDIATELY. A doorway in a range profile is a
near-far-near pattern: a jamb at A, a run of beams that suddenly reach much
further (through the opening -- into the room, or to nothing at all), and a
second jamb at B. That signature exists on the FIRST scan that sees the
opening, needs no accumulated map, and is exactly as strong for a door in the
side wall of a corridor as for one straight ahead, which matters because most
of ours are to the side.

SO THE GRID IS DEMOTED TO A CROSS-CHECK, and its UNKNOWN cells are read as
UNKNOWN rather than as obstacles -- the opposite of what the skeleton version
did, and for the same reason the skeleton version failed. It is asked three
cheap questions: is the line A-B clear of real obstacle, is there ANY free
cell behind it (one ray's worth is enough -- there is deliberately no "wide
room" requirement), and is there occupancy at A and at B where the jambs
should be. The medial-axis code is still here, behind `use_skeleton`, default
off; `skeleton_candidates` is the old entry point, unchanged.

PERMISSIVE ON PURPOSE. A missed door is a mission that cannot run; a false
door is a marker in Foxglove and, at worst, one aborted approach. So every
gate has slack, and all of it is parameterised:
  - a short occupied run across the opening (a chair leg, one bad return, a
    map artefact) does not veto it -- only a CONTINUOUS occupied run of
    `occupied_run_veto_m` does, and up to `occupied_frac_max` of the segment
    may be occupied in pieces smaller than `noise_max_m`;
  - a few near beams inside the opening do not split it, up to
    `short_run_max` in a row or `short_frac_max` of them overall;
  - a missing jamb (glass, a corner reflection, a jamb seen edge-on) keeps
    the candidate with a `one_jamb` flag rather than rejecting it;
  - the detection width band is wider than the nominal one, and a door
    outside the nominal band is published with a `marginal` flag rather than
    dropped.
A flag is NOT a rejection: a flagged candidate is a door, tracked and
published like any other, and the flag rides along so the markers can say so.

WHAT STOPS A ONE-SCAN ARTEFACT, then, if the gates are this loose: the
CONFIRMATION, which is about where the car was standing rather than how many
times it looked. A track confirms on two hits from robot positions
`confirm_pose_spread_m` apart, or `confirm_same_pose_hits` hits from anywhere.
A reflection, a sunbeam or a beam that slipped past a corner does not survive
being looked at from a different place; a doorway does.

WHAT IT SHARES WITH THE OTHER TWO DETECTORS. Everything downstream, exactly as
before: the candidates are semantic_doors.DoorCandidate and they go to
semantic_doors.DoorTracker, so stable ids, association, the smoothed pose, the
direction fixed once at first confirmation and "kept in the map after leaving
view" are that tracker's behaviour, not a second copy. `case` is 'F' and the
parent wall ids are -1, because there are no parent walls.

FRAMES. Jumps are found in beam order, which only means anything in the laser
frame; every POINT is carried in MAP, because that is the frame the grid, the
tracker and the door TF all live in. `ScanProfile` holds both.

NO rclpy, NO message types, NO TF: numpy and scipy.ndimage only, so every gate
below is testable on a synthetic range profile and a hand-drawn grid.
scikit-image is deliberately NOT a dependency -- see package.xml.
"""

import math

import numpy as np

from scipy import ndimage

from f1tenth_costmap.semantic_doors import DoorCandidate

#: OccupancyGrid.data sentinel for a cell that was never observed.
UNKNOWN = -1

#: `case` on every DoorCandidate this module makes. 'A'/'B' belong to
#: semantic_doors' two wall rules; this detector has one rule and no walls.
CASE_FREESPACE = 'F'

#: No parent walls exist, and Door.parent_wall_id* is an integer field.
NO_WALL_ID = -1


# ---------------------------------------------------------------------------
# Rejection codes, and flags
# ---------------------------------------------------------------------------
#: Why an opening was not a door, from the SCAN half. '' is a door.
#:
#:   width         |AB| outside [width_min_detect_m, width_max_detect_m]
#:   no_interior   no beams at all between the two jambs
#:   few_beams     fewer than beyond_beams_min beams reached beyond_m past AB
#:   split         the near beams inside A..B are too many, or too many in a
#:                 row: something is standing in the opening, not beside it
#:   edge          A or B is an isolated return, not the end of a surface --
#:                 a jamb is part of something, a dust mote is not
SCAN_CODES = ('width', 'no_interior', 'few_beams', 'split', 'edge')

#: ... and from the GRID half, the cross-check.
#:
#:   blocked        a CONTINUOUS occupied run of occupied_run_veto_m across AB
#:   occupied_frac  more than occupied_frac_max of AB occupied, in pieces
#:   no_behind      not one free cell behind AB: the beams that made this
#:                  opening left no trace in the map at all
#:   no_jamb        neither A nor B has any occupancy near it
GRID_CODES = ('blocked', 'occupied_frac', 'no_behind', 'no_jamb')

#: ... and from the medial-axis detector, which is still here behind
#: `use_skeleton` and still has its own gates. See skeleton_candidates.
SKELETON_CODES = ('junction', 'not_min', 'no_rise', 'spur', 'asymmetric',
                  'thin_jamb', 'not_beyond')

#: ACCEPTED-BUT-NOTEWORTHY. Not rejections: a flagged candidate is a door and
#: is tracked and published like any other.
#:
#:   marginal   width outside [width_min_m, width_max_m] but inside the wider
#:              detection band -- probably a door, measure it before trusting
#:   one_jamb   only one end had occupancy near it. Glass, a corner
#:              reflection and a jamb seen edge-on all do this, and dropping
#:              the opening for it loses more doors than it saves
#:   no_grid    no occupancy grid was available, so none of GRID_CODES was
#:              checked at all. The scan alone said door
FLAG_NAMES = ('marginal', 'one_jamb', 'no_grid')


def code_group(code):
    """'ok' | 'scan' | 'grid' | 'skeleton' for a candidate's code.

    Raises on a code it does not know rather than guessing a group: the
    groups are what a debug dict is read by, and silently filing a scan
    failure under grid would send someone to look at the map for an answer
    that is in the range profile.
    """
    if not code:
        return 'ok'
    if code in SCAN_CODES:
        return 'scan'
    if code in GRID_CODES:
        return 'grid'
    if code in SKELETON_CODES:
        return 'skeleton'
    raise ValueError(f'not a freespace_doors code: {code!r}')


# ---------------------------------------------------------------------------
# Parameters
# ---------------------------------------------------------------------------
class FreespaceParams:
    """Every threshold of this module, one object, all SI.

    Defaults are the shipped ones and are deliberately PERMISSIVE (see the
    module docstring); stack_params.yaml is the single source for what the
    car actually runs, and freespace_door_node builds this from it.
    """

    __slots__ = (
        # the window
        'window_m', 'occupied_threshold', 'cleanup_iterations',
        # widths, shared by both detectors
        'width_min_m', 'width_max_m', 'width_min_detect_m', 'width_max_detect_m',
        # the scan half
        'smooth_beams', 'jump_min_m', 'max_edges', 'min_interior_beams',
        'beyond_m', 'beyond_beams_min', 'block_margin_m',
        'short_run_max', 'short_frac_max',
        'edge_beams', 'edge_tol_m',
        # the grid half
        'noise_max_m', 'occupied_frac_max', 'occupied_run_veto_m',
        'behind_min_m', 'behind_max_m', 'behind_free_min_cells',
        'scan_jamb_radius_m', 'scan_jamb_min_cells',
        # assembly
        'merge_max_m', 'max_candidates',
        # tracking
        'assoc_max_m', 'confirm_hits', 'confirm_pose_spread_m',
        'confirm_same_pose_hits', 'max_tracks', 'smooth_alpha',
        'candidate_forget_s', 'forget_unseen_s',
        # the medial-axis detector, off by default
        'use_skeleton', 'rise_ratio', 'rise_within_m', 'spur_prune_m',
        'local_min_radius_m', 'tangent_span_m', 'symmetry_tol_m',
        'jamb_radius_m', 'jamb_min_cells', 'min_beams', 'jamb_exclude_m')

    def __init__(self, **kw):
        # --- the window
        self.window_m = 8.0
        self.occupied_threshold = 65
        self.cleanup_iterations = 1
        # --- widths. The DETECT band is what is looked for; a door outside
        #     the nominal band but inside this one is published, flagged
        #     'marginal', rather than dropped.
        self.width_min_m = 0.70
        self.width_max_m = 1.40
        self.width_min_detect_m = 0.60
        self.width_max_detect_m = 1.50
        # --- the scan half
        self.smooth_beams = 3
        self.jump_min_m = 0.50
        self.max_edges = 64
        self.min_interior_beams = 1
        self.beyond_m = 0.50
        self.beyond_beams_min = 2
        self.block_margin_m = 0.05
        self.short_run_max = 2
        self.short_frac_max = 0.20
        self.edge_beams = 3
        self.edge_tol_m = 0.20
        # --- the grid half
        self.noise_max_m = 0.10
        self.occupied_frac_max = 0.25
        self.occupied_run_veto_m = 0.20
        self.behind_min_m = 0.30
        self.behind_max_m = 1.00
        self.behind_free_min_cells = 3
        self.scan_jamb_radius_m = 0.25
        self.scan_jamb_min_cells = 1
        # --- assembly
        self.merge_max_m = 0.30
        self.max_candidates = 16
        # --- tracking (handed to semantic_doors.DoorTracker)
        self.assoc_max_m = 0.30
        self.confirm_hits = 3
        self.confirm_pose_spread_m = 0.15
        self.confirm_same_pose_hits = 3
        self.max_tracks = 32
        self.smooth_alpha = 0.30
        self.candidate_forget_s = 12.0
        self.forget_unseen_s = 0.0   # 0 = a confirmed door is never forgotten
        # --- the medial-axis detector (skeleton_candidates), default OFF
        self.use_skeleton = False
        self.rise_ratio = 1.3
        self.rise_within_m = 1.0
        self.spur_prune_m = 0.30
        self.local_min_radius_m = 0.30
        self.tangent_span_m = 0.25
        self.symmetry_tol_m = 0.20
        self.jamb_radius_m = 0.15
        self.jamb_min_cells = 4
        self.min_beams = 4
        self.jamb_exclude_m = 0.08
        for k, v in kw.items():
            if k not in self.__slots__:
                raise TypeError(f'FreespaceParams got an unknown parameter {k!r}')
            setattr(self, k, v)

    def door_params(self):
        """A semantic_doors.DoorParams carrying only the tracking knobs.

        DoorTracker reads exactly these off its params object and nothing
        else; the rest of DoorParams is the wall detector's geometry and is
        irrelevant here. Built rather than subclassed so there is one
        tracker, with one behaviour, for all three door sources.
        """
        from f1tenth_costmap.semantic_doors import DoorParams
        return DoorParams(
            assoc_max_m=self.assoc_max_m,
            confirm_hits=self.confirm_hits,
            confirm_pose_spread_m=self.confirm_pose_spread_m,
            confirm_same_pose_hits=self.confirm_same_pose_hits,
            max_tracks=self.max_tracks,
            smooth_alpha=self.smooth_alpha,
            candidate_forget_s=self.candidate_forget_s,
            forget_unseen_s=self.forget_unseen_s)


# ---------------------------------------------------------------------------
# The window
# ---------------------------------------------------------------------------
class Window:
    """One cropped patch of the grid, in THREE states, plus a clearance field.

    THE THREE STATES MATTER, and getting them wrong is what sank the first
    version of this module:
      `occupied`  value >= occupied_threshold. A real obstacle.
      `unknown`   value < 0. Never observed. NOT an obstacle -- a room behind
                  a door is mostly this, and calling it obstacle is exactly
                  how the medial-axis detector concluded there was nothing
                  there (see the module docstring).
      `free`      known free, i.e. neither of the above. This is also what
                  `clearance` is the distance transform of, and what
                  `is_free`/`march_to_jamb` test, so the medial-axis detector
                  keeps its old "unknown is impassable" reading unchanged --
                  which is right for the question IT asks.

    `clearance` is only built when asked for (the scan detector never looks at
    it and the distance transform is the single most expensive thing here).
    `origin` is the map-frame position of the (0, 0) cell's CORNER, so a cell
    centre is origin + (index + 0.5) * res -- the convention
    costmap_boundary.py uses.
    """

    __slots__ = ('free', 'occupied', 'unknown', 'clearance', 'res', 'origin')

    def __init__(self, free, occupied, unknown, clearance, res, origin):
        self.free = free
        self.occupied = occupied
        self.unknown = unknown
        self.clearance = clearance
        self.res = float(res)
        self.origin = (float(origin[0]), float(origin[1]))

    @property
    def shape(self):
        return self.free.shape

    def to_map(self, row, col):
        """Map-frame (x, y) of a cell CENTRE. Accepts arrays."""
        return (self.origin[0] + (np.asarray(col) + 0.5) * self.res,
                self.origin[1] + (np.asarray(row) + 0.5) * self.res)

    def to_cell(self, x, y):
        """(row, col) of the cell containing a map-frame point. May be out of range."""
        return (int(math.floor((y - self.origin[1]) / self.res)),
                int(math.floor((x - self.origin[0]) / self.res)))

    def inside(self, row, col):
        return 0 <= row < self.free.shape[0] and 0 <= col < self.free.shape[1]

    def _at(self, mask, x, y, outside=False):
        r, c = self.to_cell(x, y)
        return bool(mask[r, c]) if self.inside(r, c) else outside

    def is_free(self, x, y):
        """Known free AND inside the window. Outside reads as not free."""
        return self._at(self.free, x, y, outside=False)

    def is_occupied(self, x, y):
        """A real obstacle. Outside the window reads as NOT occupied.

        Outside-is-not-occupied on purpose: the window is cropped around the
        car, and a door at its edge must not be vetoed by the crop.
        """
        return self._at(self.occupied, x, y, outside=False)

    def is_unknown(self, x, y):
        """Never observed, or outside the window -- the same thing to a caller."""
        return self._at(self.unknown, x, y, outside=True)

    def count_occupied(self, xy, radius_m):
        """Occupied cells within `radius_m` (square) of a map point."""
        r, c = self.to_cell(xy[0], xy[1])
        k = max(1, int(round(float(radius_m) / self.res)))
        r0, r1 = max(0, r - k), min(self.shape[0], r + k + 1)
        c0, c1 = max(0, c - k), min(self.shape[1], c + k + 1)
        if r0 >= r1 or c0 >= c1:
            return 0
        return int(np.count_nonzero(self.occupied[r0:r1, c0:c1]))


def _cell_structure():
    """The 3x3 all-ones structuring element: 8-connectivity, one cell of reach."""
    return np.ones((3, 3), dtype=bool)


def occupancy_window(grid_data, width, height, resolution, origin_x, origin_y,
                     robot_xy, p, with_clearance=None):
    """Crop the grid around the robot into the three states Window documents.

    The window is always exactly the requested size: anything outside the
    grid's own extent is filled UNKNOWN -- which for the scan detector means
    "no opinion" (a door at the edge of the map is still a door) and for the
    medial-axis one means impassable, since that one folds unknown into
    not-free. Both readings come out of the same crop.

    CLEANUP, one closing of the OBSTACLES, which can only ever make them more
    solid: it fills a one- or two-cell pinhole in a wall, so a missed ray is
    not a 0.10 m doorway. It is applied to the obstacles ALONE and never
    grows them into unknown space. `free` additionally gets the opening the
    medial-axis detector needs (isolated free specks dropped) -- it is the
    only consumer of `free`, and the scan detector reads `occupied` directly.

    `with_clearance` defaults to p.use_skeleton: the distance transform costs
    about 9 ms on the default window and nothing but the medial axis reads it.
    """
    if with_clearance is None:
        with_clearance = bool(p.use_skeleton)
    res = float(resolution)
    half = int(math.ceil(float(p.window_m) / res))
    rc, cc = (int(math.floor((robot_xy[1] - origin_y) / res)),
              int(math.floor((robot_xy[0] - origin_x) / res)))
    r0, c0 = rc - half, cc - half
    n = 2 * half + 1

    arr = np.asarray(grid_data, dtype=np.int16).reshape(int(height), int(width))
    sub = np.full((n, n), UNKNOWN, dtype=np.int16)
    sr0, sc0 = max(0, r0), max(0, c0)
    sr1, sc1 = min(int(height), r0 + n), min(int(width), c0 + n)
    if sr0 < sr1 and sc0 < sc1:
        sub[sr0 - r0:sr1 - r0, sc0 - c0:sc1 - c0] = arr[sr0:sr1, sc0:sc1]

    unknown = sub < 0
    occupied = sub >= int(p.occupied_threshold)
    free = ~unknown & ~occupied
    it = int(p.cleanup_iterations)
    st = _cell_structure()
    if it > 0:
        occupied = ndimage.binary_closing(occupied, structure=st, iterations=it,
                                          border_value=0)
        # The medial axis's own reading: unknown impassable, specks removed.
        free = ndimage.binary_opening(
            ndimage.binary_closing(free, structure=st, iterations=it,
                                   border_value=0) & ~occupied,
            structure=st, iterations=it, border_value=0)
        unknown = unknown & ~occupied
    clearance = (np.asarray(ndimage.distance_transform_edt(free, sampling=res),
                            dtype=float)
                 if with_clearance else None)
    origin = (origin_x + c0 * res, origin_y + r0 * res)
    return Window(free, occupied, unknown, clearance, res, origin)


# ---------------------------------------------------------------------------
# The scan: where the range profile steps out and back in again
# ---------------------------------------------------------------------------
class ScanProfile:
    """One scan, in beam order, with every point already in MAP.

    Beam ORDER is the laser frame's business -- a jump is a thing that
    happens between two adjacent beams and means nothing anywhere else -- but
    every POINT here is map-frame, because the grid, the tracker and the door
    TF all are, and carrying two copies of the geometry to transform at the
    end is how frames get mixed up.

    `ranges` is already smoothed and already has no-return beams filled in at
    `range_max`: a beam that goes through a doorway and hits nothing is the
    strongest evidence the doorway is open, and dropping it (which is right
    for unknown_objects.scan_points, whose question is "does a return LIE ON
    something") would make an open door look shut.
    """

    __slots__ = ('bearings', 'ranges', 'points', 'laser', 'range_max', 'no_return')

    def __init__(self, bearings, ranges, points, laser_xy, range_max, no_return):
        self.bearings = np.asarray(bearings, dtype=float)
        self.ranges = np.asarray(ranges, dtype=float)
        self.points = np.asarray(points, dtype=float).reshape(-1, 2)
        self.laser = (float(laser_xy[0]), float(laser_xy[1]))
        self.range_max = float(range_max)
        self.no_return = np.asarray(no_return, dtype=bool)

    def __len__(self):
        return int(self.ranges.size)


def smooth_ranges(ranges, window):
    """A light median filter over the range profile. Edge-preserving.

    A MEDIAN and not a mean, because every gate below is about an EDGE: a
    mean of three smears a 0.5 m jamb step across three beams and shifts
    where A and B are, a median of three removes a single-beam spike and
    leaves the step exactly where it was.
    """
    w = int(window)
    if w < 3 or ranges.size < w:
        return np.asarray(ranges, dtype=float)
    return ndimage.median_filter(np.asarray(ranges, dtype=float), size=w,
                                 mode='nearest')


def scan_profile(bearings, ranges, range_min, range_max, laser_map, p):
    """Build a ScanProfile: fill no-returns, smooth, and place every beam in map.

    `laser_map` is (x, y, yaw) of the LASER in the map frame.
    """
    r = np.asarray(ranges, dtype=float)
    b = np.asarray(bearings, dtype=float)
    no_return = ~np.isfinite(r) | (r >= float(range_max))
    r = np.where(no_return, float(range_max), r)
    too_near = (~no_return) & (r < float(range_min))
    if np.any(too_near):
        # Below range_min is not a measurement at all. Treated as no-return
        # rather than dropped, so beam ORDER (which every jump test depends
        # on) survives, and a dropped beam cannot silently become a jump.
        r = np.where(too_near, float(range_max), r)
        no_return = no_return | too_near
    r = smooth_ranges(r, p.smooth_beams)
    th = float(laser_map[2]) + b
    pts = np.column_stack([laser_map[0] + r * np.cos(th),
                           laser_map[1] + r * np.sin(th)])
    return ScanProfile(b, r, pts, laser_map[:2], range_max, no_return)


def jump_edges(scan, p):
    """(rising, falling): beam indices where the profile steps out, and back.

    `rising[i] = a` means beam a is NEAR and beam a+1 is at least
    `jump_min_m` further: a is the last beam on the near surface, i.e. a
    jamb point. `falling[j] = b` means beam b is NEAR and beam b-1 is at
    least that much further: b is the FIRST beam back on a near surface, the
    other jamb. A doorway is a rising edge followed by a falling one.

    Capped at `max_edges` each, largest jumps kept, so a scan full of foliage
    cannot turn the pairing below into a quadratic blow-up.
    """
    r = scan.ranges
    if r.size < 3:
        return np.zeros(0, dtype=int), np.zeros(0, dtype=int)
    step = r[1:] - r[:-1]
    rising = np.flatnonzero(step >= float(p.jump_min_m))
    falling = np.flatnonzero(step <= -float(p.jump_min_m)) + 1
    cap = int(p.max_edges)
    if rising.size > cap:
        rising = np.sort(rising[np.argsort(step[rising])[-cap:]])
    if falling.size > cap:
        falling = np.sort(falling[np.argsort(-step[falling - 1])[-cap:]])
    return rising, falling


def _beyond(points, a, b, laser, beyond_m):
    """Signed distance of each point past line AB, positive AWAY from `laser`.

    Returns (signed, mask) where mask is `signed >= beyond_m`. The sign
    convention is the whole point: "beyond" means on the far side of the
    threshold from the sensor, so a beam that stops short of the opening and
    a beam that sails through it are told apart by a sign, not a magnitude.
    """
    ax, ay = float(a[0]), float(a[1])
    ex, ey = float(b[0]) - ax, float(b[1]) - ay
    length = math.hypot(ex, ey)
    if length < 1e-9:
        return np.zeros(len(points)), np.zeros(len(points), dtype=bool)
    nx, ny = -ey / length, ex / length
    s_laser = (laser[0] - ax) * nx + (laser[1] - ay) * ny
    sign = -1.0 if s_laser > 0.0 else 1.0
    signed = ((points[:, 0] - ax) * nx + (points[:, 1] - ay) * ny) * sign
    return signed, signed >= float(beyond_m)


def _is_surface_end(scan, index, direction, p):
    """Does the near surface CONTINUE on `direction` side of beam `index`?

    A jamb is the end of something: the beams just outside the opening come
    back at about the same range. An isolated return -- one dust mote, one
    reflection, the tip of a chair back -- has nothing beside it at its own
    range, and pairing two of those makes a doorway out of thin air.
    """
    n = len(scan)
    r0 = scan.ranges[index]
    for k in range(1, int(p.edge_beams) + 1):
        i = index + direction * k
        if not (0 <= i < n):
            break
        if scan.no_return[i]:
            continue
        if abs(scan.ranges[i] - r0) <= float(p.edge_tol_m):
            return True
    return False


def _run_lengths(mask):
    """The lengths of the True runs in a boolean array, longest first."""
    if mask.size == 0:
        return []
    idx = np.flatnonzero(np.diff(np.concatenate(([0], mask.view(np.int8), [0]))))
    return sorted(((idx[1::2] - idx[0::2]).tolist()), reverse=True)


def opening_from_scan(scan, a, b, p):
    """Evaluate the beams between jamb beams `a` and `b`; return (code, info).

    code '' means the SCAN half accepts this opening. `info` carries what the
    grid half and the candidate need: the two jamb points, the width, how
    many beams went beyond, and any flags earned here.
    """
    pa, pb = scan.points[a], scan.points[b]
    width = float(math.hypot(pb[0] - pa[0], pb[1] - pa[1]))
    info = {'a': (float(pa[0]), float(pa[1])), 'b': (float(pb[0]), float(pb[1])),
            'width': width, 'n_beyond': 0, 'flags': [], 'beyond_pts': None}
    if not (float(p.width_min_detect_m) <= width <= float(p.width_max_detect_m)):
        return 'width', info
    if not (float(p.width_min_m) <= width <= float(p.width_max_m)):
        info['flags'].append('marginal')

    interior = scan.points[a + 1:b]
    if interior.shape[0] < max(1, int(p.min_interior_beams)):
        return 'no_interior', info
    signed, beyond = _beyond(interior, pa, pb, scan.laser, p.beyond_m)
    info['n_beyond'] = int(np.count_nonzero(beyond))
    info['beyond_pts'] = interior[beyond]
    if info['n_beyond'] < int(p.beyond_beams_min):
        return 'few_beams', info

    # BLOCKING beams: ones that stopped AT or BEFORE the threshold. A few
    # are a chair leg standing in the doorway, or noise; many, or many in a
    # row, are something filling it.
    #
    # Not "beams that failed to reach beyond_m", which is what this tested
    # first and which rejected every side door into a small room: a beam
    # through a 0.9 m opening into a 3 m room seen at a shallow angle clips
    # the room's far wall only 0.3 m past the threshold, so a whole run of
    # perfectly good beams read as "short" and split the opening. A beam
    # that is PAST the threshold went through, however little further it
    # got; only one that stopped at or before it is in the way.
    blocking = signed <= float(p.block_margin_m)
    runs = _run_lengths(blocking)
    longest = runs[0] if runs else 0
    frac = float(np.count_nonzero(blocking)) / float(blocking.size)
    if longest > int(p.short_run_max) and frac > float(p.short_frac_max):
        return 'split', info

    # BOTH ends, not either: a doorway has a jamb on each side. The first
    # version accepted an opening when EITHER end looked like a surface,
    # which on a real bag rejected exactly ONE opening out of ten thousand
    # -- a gate that never fires is not a gate.
    if not (_is_surface_end(scan, a, -1, p) and _is_surface_end(scan, b, +1, p)):
        return 'edge', info
    return '', info


def scan_openings(scan, p):
    """Every near-far-near opening in one scan, evaluated. Rejections included.

    Returns [(code, info), ...]. The rejected ones are kept because they are
    what the debug markers draw and the debug dict counts -- "there is a jump
    pair there and here is why it is not a door" is the question this
    detector gets asked.
    """
    rising, falling = jump_edges(scan, p)
    if rising.size == 0 or falling.size == 0:
        return []
    out = []
    for a in rising:
        for b in falling:
            if b <= a + 1:
                continue
            code, info = opening_from_scan(scan, int(a), int(b), p)
            if code == 'width':
                # Too wide here means every further falling edge is wider
                # still only if the profile were monotonic, which it is not;
                # but a too-wide pair is the common case and cheap to skip.
                if info['width'] > float(p.width_max_detect_m):
                    continue
            out.append((code, info))
    return out


# ---------------------------------------------------------------------------
# The grid: a cross-check, never the primary evidence
# ---------------------------------------------------------------------------
def _samples_along(a, b, step):
    """Points sampled along the segment a -> b at `step`, both ends included."""
    length = math.hypot(b[0] - a[0], b[1] - a[1])
    n = max(1, int(math.ceil(length / step)))
    ts = np.linspace(0.0, 1.0, n + 1)
    return (np.column_stack([a[0] + (b[0] - a[0]) * ts,
                             a[1] + (b[1] - a[1]) * ts]), length / n)


def check_segment(window, a, b, p):
    """Is the line A-B clear enough of real obstacle to be an opening?

    TOLERANT BY DESIGN, three numbers deep:
      - an occupied run shorter than `noise_max_m` is ignored outright (one
        or two cells: a chair leg, one bad return, a map artefact);
      - what survives may still cover up to `occupied_frac_max` of the
        segment, as long as it is in pieces;
      - only a CONTINUOUS surviving run of `occupied_run_veto_m` vetoes the
        opening, because that is a thing standing across the doorway rather
        than in it.
    """
    pts, step = _samples_along(a, b, window.res / 2.0)
    occ = np.array([window.is_occupied(x, y) for x, y in pts], dtype=bool)
    runs = _run_lengths(occ)
    noise_cells = max(1, int(round(float(p.noise_max_m) / step)))
    real = [n for n in runs if n > noise_cells]
    if real and real[0] * step >= float(p.occupied_run_veto_m):
        return 'blocked'
    if real and (sum(real) / float(occ.size)) > float(p.occupied_frac_max):
        return 'occupied_frac'
    return ''


def check_behind(window, info, laser, p):
    """Did the beams that went through this opening leave a trace in the map?

    ALONG THE RAYS, not along the door's own perpendicular -- which is the
    whole correction, and it only shows up on a SIDE door. A beam does not
    turn to face the doorway after passing it: seen at a shallow angle, the
    beams through a 0.9 m opening 3 m away have fanned more than a metre
    sideways by the time they are 1 m past the threshold, so a box sampled
    perpendicular to the opening lands where no beam ever went and reports
    'no_behind' for a perfectly good door. (First version did exactly that
    and failed every side-door test.)

    NO "WIDE ROOM" REQUIREMENT, deliberately: a room behind a door that has
    never been driven into is a fan of thin free lines with unknown between
    them, and asking for an open region there is precisely the test that made
    the medial-axis detector report nothing. All this asks is that SOMETHING
    got through -- which the scan already said, so this is really a check
    that the map agrees with the scan rather than an independent gate.
    """
    pts = info.get('beyond_pts')
    if pts is None or len(pts) == 0:
        return 'no_behind'
    a, b = info['a'], info['b']
    ex, ey = b[0] - a[0], b[1] - a[1]
    length = math.hypot(ex, ey)
    if length < 1e-9:
        return 'no_behind'
    nx, ny = -ey / length, ex / length
    s_laser = (laser[0] - a[0]) * nx + (laser[1] - a[1]) * ny
    if abs(s_laser) < 1e-9:
        return 'no_behind'
    step = max(window.res, 0.05)
    depths = np.arange(float(p.behind_min_m), float(p.behind_max_m) + 1e-9, step)
    found = 0
    for px, py in pts:
        dx, dy = px - laser[0], py - laser[1]
        ray_len = math.hypot(dx, dy)
        if ray_len < 1e-9:
            continue
        s_p = ((px - a[0]) * nx + (py - a[1]) * ny)
        if (s_p - s_laser) == 0.0:
            continue
        # Where this ray crosses the threshold, as a distance from the laser.
        cross = ray_len * (-s_laser) / (s_p - s_laser)
        if not (0.0 < cross < ray_len):
            continue
        ux, uy = dx / ray_len, dy / ray_len
        for d in depths:
            reach = cross + d
            if reach >= ray_len:
                break
            if window.is_free(laser[0] + ux * reach, laser[1] + uy * reach):
                found += 1
                if found >= int(p.behind_free_min_cells):
                    return ''
    return 'no_behind'


def check_jambs(window, a, b, p):
    """(code, flags): is there occupancy where each jamb should be?

    A MISSING JAMB IS A FLAG, NOT A REJECTION. Glass returns nothing, a
    corner can reflect a beam away, and a jamb seen edge-on puts almost no
    cells in the grid; dropping the opening for any of those loses more real
    doors than it saves false ones, and the confirmation is what filters
    artefacts here. Both ends missing is still a rejection: that is not a
    doorway, it is two numbers.
    """
    need = int(p.scan_jamb_min_cells)
    na = window.count_occupied(a, p.scan_jamb_radius_m)
    nb = window.count_occupied(b, p.scan_jamb_radius_m)
    if na < need and nb < need:
        return 'no_jamb', []
    if na < need or nb < need:
        return '', ['one_jamb']
    return '', []


def check_grid(window, info, laser, p):
    """The whole grid cross-check for one opening: (code, flags).

    With no window at all -- the first scans of a run, before slam_toolbox
    has published anything -- NOTHING here is checked and the candidate is
    flagged `no_grid`. The scan is the primary evidence and it has already
    spoken; refusing to see doors until a map exists would make the detector
    useless exactly when a mission is most likely to be starting.
    """
    if window is None:
        return '', ['no_grid']
    a, b = info['a'], info['b']
    code = check_segment(window, a, b, p)
    if code:
        return code, []
    code = check_behind(window, info, laser, p)
    if code:
        return code, []
    return check_jambs(window, a, b, p)


# ---------------------------------------------------------------------------
# Assembly
# ---------------------------------------------------------------------------
def _candidate(info, code, flags):
    return DoorCandidate(info['a'], info['b'], CASE_FREESPACE,
                         NO_WALL_ID, NO_WALL_ID, code=code,
                         n_beams=info['n_beyond'],
                         flags=tuple(dict.fromkeys(list(info['flags']) + list(flags))))


def merge_candidates(cands, max_m):
    """Collapse candidates whose centres are within `max_m` into one.

    One doorway produces several jump pairs: the median filter, a jamb two
    beams wide, a far surface at two different depths all give neighbouring
    rising and falling edges that pair up into nearly the same opening. The
    tracker associates each candidate to AT MOST ONE track, so handing it
    five copies of one door spawns four spurious tracks. The survivor of a
    cluster is the one with the most beams through it -- the best-seen
    version of that door -- and it inherits the union of the cluster's flags,
    so a merge never quietly drops a `marginal` or `one_jamb` warning.
    """
    out = []
    for cand in sorted(cands, key=lambda c: (-c.n_beams, c.centre)):
        for kept in out:
            if math.hypot(cand.centre[0] - kept.centre[0],
                          cand.centre[1] - kept.centre[1]) < float(max_m):
                kept.flags = tuple(dict.fromkeys(kept.flags + cand.flags))
                break
        else:
            out.append(cand)
    return out


def scan_candidates(scan, window, robot_xy, p):
    """THE ENTRY POINT: one scan (and optionally the grid) in, candidates out.

    Returns semantic_doors.DoorCandidate, accepted ones with code '' and
    rejected ones carrying why, ready for a semantic_doors.DoorTracker.
    Accepted candidates are merged and capped; rejected ones are returned
    whole, because they are what the debug view is made of.
    """
    accepted, rejected = [], []
    for code, info in scan_openings(scan, p):
        if code:
            rejected.append(_candidate(info, code, ()))
            continue
        grid_code, flags = check_grid(window, info, scan.laser, p)
        cand = _candidate(info, grid_code, flags)
        (rejected if grid_code else accepted).append(cand)
    accepted = merge_candidates(accepted, p.merge_max_m)
    accepted.sort(key=lambda c: -c.n_beams)
    return accepted[:int(p.max_candidates)] + rejected


def jump_points(scan, p):
    """(n, 2) map-frame points of every jump edge -- for the debug markers."""
    rising, falling = jump_edges(scan, p)
    idx = np.concatenate([rising, falling]) if (rising.size or falling.size) \
        else np.zeros(0, dtype=int)
    if idx.size == 0:
        return np.zeros((0, 2), dtype=float)
    return scan.points[idx]


# ---------------------------------------------------------------------------
# The medial axis
# ---------------------------------------------------------------------------
#: The eight neighbour offsets in Zhang-Suen's own P2..P9 order: N, NE, E, SE,
#: S, SW, W, NW. The order is the algorithm, not a convention -- A(P1) counts
#: 0 -> 1 transitions AROUND the ring, so shuffling it breaks the test.
_RING = ((-1, 0), (-1, 1), (0, 1), (1, 1), (1, 0), (1, -1), (0, -1), (-1, -1))


def _pad(mask):
    """`mask` in a zero border, so every neighbour view below is in range."""
    out = np.zeros((mask.shape[0] + 2, mask.shape[1] + 2), dtype=np.uint8)
    out[1:-1, 1:-1] = mask
    return out


def _neighbour_code(pad, shape):
    """(rows, cols) uint8: bit k set when the _RING[k] neighbour is set.

    The eight neighbours as SLICES of the padded array -- numpy views, no
    copies -- packed into one byte per cell, so the four Zhang-Suen
    conditions become a single 256-entry table lookup instead of thirty
    array operations. This is what makes `thin` affordable at 2 Hz: the
    first draft of it built an (8, rows, cols) stack and an np.roll per
    sub-iteration and cost 284 ms on an 8 m window, measured.
    """
    h, w = shape
    code = np.zeros(shape, dtype=np.uint8)
    for k, (dr, dc) in enumerate(_RING):
        code |= pad[1 + dr:1 + dr + h, 1 + dc:1 + dc + w] << np.uint8(k)
    return code


def _zhang_suen_tables():
    """(step1, step2) bool tables over the 256 neighbour codes.

    For each code: B is the number of set neighbours, A the number of 0 -> 1
    transitions going once around the ring. Both sub-iterations want
    2 <= B <= 6 and A == 1; they differ only in which three-neighbour
    products must be zero, which is the asymmetry that keeps the thinning
    from eating a two-cell-wide line from one side.
    """
    t1 = np.zeros(256, dtype=bool)
    t2 = np.zeros(256, dtype=bool)
    for code in range(256):
        bits = [(code >> k) & 1 for k in range(8)]
        b = sum(bits)
        a = sum(1 for k in range(8) if bits[k] == 0 and bits[(k + 1) % 8] == 1)
        if not (2 <= b <= 6 and a == 1):
            continue
        p2, p4, p6, p8 = bits[0], bits[2], bits[4], bits[6]
        t1[code] = (p2 * p4 * p6 == 0) and (p4 * p6 * p8 == 0)
        t2[code] = (p2 * p4 * p8 == 0) and (p2 * p6 * p8 == 0)
    return t1, t2


_STEP1, _STEP2 = _zhang_suen_tables()


def thin(mask, max_iterations=400):
    """Zhang-Suen thinning: `mask` reduced to a one-cell-wide skeleton.

    The classic two-sub-iteration algorithm, vectorised: each sub-iteration
    marks every cell that passes its four conditions and deletes them all at
    once, which is what makes the result independent of scan order.
    It stops when a full iteration deletes nothing.

    Topology-preserving, which is the whole reason to use it rather than a
    distance-transform ridge: the skeleton of a room-door-room scene is
    CONNECTED THROUGH THE DOORWAY, and that connectivity is what `_walk`
    below follows to ask whether the clearance rises on both sides.

    `max_iterations` only bounds the work on a pathological input -- one cell
    layer is peeled per iteration, so a window of radius R needs at most
    R / resolution of them. Reaching the cap returns the partially thinned
    mask rather than raising: a thicker-than-one-cell skeleton produces
    'junction' rejections, not wrong doors.
    """
    cur = np.asarray(mask, dtype=bool).copy()
    pad = _pad(cur.view(np.uint8))
    shape = cur.shape
    for _ in range(int(max_iterations)):
        changed = False
        for table in (_STEP1, _STEP2):
            doomed = cur & table[_neighbour_code(pad, shape)]
            if doomed.any():
                cur &= ~doomed
                pad[1:-1, 1:-1] = cur.view(np.uint8)
                changed = True
        if not changed:
            break
    return cur


def neighbour_count(skel):
    """(rows, cols) int: 8-connected skeleton neighbours of each skeleton cell."""
    skel = np.asarray(skel, dtype=bool)
    code = _neighbour_code(_pad(skel.view(np.uint8)), skel.shape)
    counts = np.unpackbits(code.reshape(-1, 1), axis=1).sum(axis=1).reshape(skel.shape)
    return np.where(skel, counts, 0).astype(np.int16)


def prune_spurs(skel, prune_cells):
    """Erode `prune_cells` cells off every skeleton TIP.

    Thinning leaves a short hair wherever the free-space boundary had a bump
    -- a corner, a single noisy cell -- and each hair ends in a tip that
    would read as a dead end. Repeatedly deleting every degree <= 1 cell
    removes a hair of length <= prune_cells entirely, and shortens a real
    branch by prune_cells, which costs nothing: a bottleneck within
    `prune_cells` of a genuine tip is a dead end and is meant to be
    rejected anyway (code 'spur').
    """
    cur = np.asarray(skel, dtype=bool).copy()
    for _ in range(max(0, int(prune_cells))):
        tips = cur & (neighbour_count(cur) <= 1)
        if not tips.any():
            break
        cur &= ~tips
    return cur


def skeleton(window, p):
    """The pruned medial axis of `window`'s free space, as a bool array."""
    skel = thin(window.free)
    return prune_spurs(skel, int(round(float(p.spur_prune_m) / window.res)))


# ---------------------------------------------------------------------------
# Walking the skeleton
# ---------------------------------------------------------------------------
_SQRT2 = math.sqrt(2.0)


def _walk(skel, start, blocked, max_m, res):
    """Geodesic BFS along the skeleton from `start`, `blocked` removed.

    Returns {(row, col): distance_m} for every skeleton cell within `max_m`
    of `start` along the skeleton, `start` itself at 0. Diagonal steps cost
    sqrt(2) cells, orthogonal 1, so the distance is a real arc length and
    not a step count -- rise_within_m is in metres and has to mean metres.

    A dict rather than an array because the walk is local (a 1 m radius is
    ~20 cells of skeleton) while the window is 320 x 320: filling an array
    per candidate would cost more than the walk.
    """
    h, w = skel.shape
    out = {start: 0.0}
    frontier = [start]
    while frontier:
        nxt = []
        for (r, c) in frontier:
            d0 = out[(r, c)]
            for (dr, dc) in _RING:
                rr, cc = r + dr, c + dc
                if not (0 <= rr < h and 0 <= cc < w) or not skel[rr, cc]:
                    continue
                if (rr, cc) == blocked:
                    continue
                step = (1.0 if dr == 0 or dc == 0 else _SQRT2) * res
                d = d0 + step
                if d > max_m:
                    continue
                if (rr, cc) not in out or d < out[(rr, cc)] - 1e-12:
                    out[(rr, cc)] = d
                    nxt.append((rr, cc))
        frontier = nxt
    return out


def _sides(skel, cell, p, res):
    """The two skeleton walks away from `cell`, or None if it is not a curve cell.

    `cell` is removed from both walks, so each one explores exactly one side
    of the bottleneck. Returns (walk_a, walk_b) keyed by the two skeleton
    neighbours; None when `cell` has anything other than two neighbours
    (a tip, an isolated cell, or a branch point -- all code 'junction').
    """
    h, w = skel.shape
    nbrs = [(cell[0] + dr, cell[1] + dc) for (dr, dc) in _RING
            if 0 <= cell[0] + dr < h and 0 <= cell[1] + dc < w
            and skel[cell[0] + dr, cell[1] + dc]]
    if len(nbrs) != 2:
        return None
    a, b = nbrs
    walk_a = _walk(skel, a, cell, float(p.rise_within_m), res)
    walk_b = _walk(skel, b, cell, float(p.rise_within_m), res)
    # Two neighbours that are themselves adjacent are the same side seen
    # twice (the skeleton turning a tight corner through this cell), not two
    # sides of a passage.
    if b in walk_a:
        return None
    return walk_a, walk_b


def _tangent(window, cell, walk_a, walk_b, p):
    """Unit skeleton tangent at `cell`: from the side-A sample to the side-B one.

    Each side contributes the cell nearest `tangent_span_m` along its own
    walk, so the tangent is a chord of the skeleton over ~2 * tangent_span_m
    rather than the direction to one adjacent cell (which can only ever be
    one of eight, i.e. quantised to 45 degrees -- far too coarse for a door
    frame's yaw).
    """
    span = float(p.tangent_span_m)

    def sample(walk):
        best, best_err = None, None
        for rc, d in walk.items():
            err = abs(d - span)
            if best_err is None or err < best_err:
                best, best_err = rc, err
        return best

    ra, rb = sample(walk_a), sample(walk_b)
    if ra is None or rb is None:
        return None
    xa, ya = window.to_map(*ra)
    xb, yb = window.to_map(*rb)
    dx, dy = float(xb - xa), float(yb - ya)
    n = math.hypot(dx, dy)
    if n < 1e-9:
        return None
    return (dx / n, dy / n)


# ---------------------------------------------------------------------------
# Jambs
# ---------------------------------------------------------------------------
def march_to_jamb(window, start_xy, direction, max_m, bisections=4):
    """Walk from `start_xy` along `direction` to the FACE of the first obstacle.

    Returns the map-frame point where the free space ends, or None if the
    march leaves the window or runs `max_m` without hitting anything.

    Steps half a cell at a time, so a diagonal march cannot tunnel through a
    one-cell-thick wall, and then BISECTS the last free / first occupied pair
    `bisections` times. Without that refinement the returned point sits up to
    half a cell inside the obstacle, both jambs overshoot outward, and the
    measured width is up to a whole cell (0.05 m) too wide -- which both
    inflates the width against its band and spoils the `asymmetric` check
    against 2d. Four bisections put it within res / 32, about 1.5 mm.
    """
    step = window.res / 2.0
    n = int(math.ceil(float(max_m) / step))
    x, y = float(start_xy[0]), float(start_xy[1])
    if not window.is_free(x, y):
        return None
    lo = 0.0                               # known free
    for i in range(1, n + 1):
        hi = step * i
        px, py = x + direction[0] * hi, y + direction[1] * hi
        r, c = window.to_cell(px, py)
        if not (0 <= r < window.shape[0] and 0 <= c < window.shape[1]):
            return None
        if not window.free[r, c]:
            for _ in range(int(bisections)):
                mid = (lo + hi) / 2.0
                if window.is_free(x + direction[0] * mid, y + direction[1] * mid):
                    lo = mid
                else:
                    hi = mid
            return (x + direction[0] * hi, y + direction[1] * hi)
        lo = hi
    return None


def jamb_cells(window, jamb_xy, radius_m):
    """How many non-free cells lie within `radius_m` of a jamb point.

    "Nearest occupied cell" is all the jamb march can tell you, and one cell
    is enough to stop it. This counts how much obstacle is actually there.
    """
    r, c = window.to_cell(jamb_xy[0], jamb_xy[1])
    k = max(1, int(round(float(radius_m) / window.res)))
    r0, r1 = max(0, r - k), min(window.shape[0], r + k + 1)
    c0, c1 = max(0, c - k), min(window.shape[1], c + k + 1)
    if r0 >= r1 or c0 >= c1:
        return 0
    return int(np.count_nonzero(~window.free[r0:r1, c0:c1]))


# ---------------------------------------------------------------------------
# The live scan confirmation
# ---------------------------------------------------------------------------
class ScanBeams:
    """One scan as MAP-frame beam endpoints, plus the laser's map position.

    The caller builds this: a beam with no return (inf, NaN, or a range at or
    past range_max) must be placed AT range_max rather than dropped, because
    a beam that goes through a doorway and hits nothing is the strongest
    evidence the doorway is open -- dropping it would make an open door look
    like a closed one. semantic_doors.ScanView drops those beams; it asks a
    different question (does a return LIE ON the gap) where a missing return
    carries no information.
    """

    __slots__ = ('points', 'laser')

    def __init__(self, points_map, laser_xy_map):
        self.points = np.asarray(points_map, dtype=float).reshape(-1, 2)
        self.laser = (float(laser_xy_map[0]), float(laser_xy_map[1]))


def beam_hits(jamb0, jamb1, beams, p):
    """How many beams cross this opening and reach `beyond_m` past it.

    THE TEST. Let L be the laser and the jamb line the infinite line through
    jamb0 and jamb1. For each beam endpoint P:
      - P must lie at least `beyond_m` beyond the line, on the far side from L
        (so a beam that stops short of the threshold, or one that ends just
        past it inside the door reveal, does not count);
      - the point where L -> P crosses the line must fall INSIDE the opening,
        shrunk by `jamb_exclude_m` at each end (so a beam passing beside the
        jamb through a different gap is not credited to this one).
    Returns the number of beams passing both.

    Counting beams rather than testing a single ray is what makes this robust
    to one stray return: `min_beams` of them have to agree.
    """
    j0 = np.asarray(jamb0, dtype=float)
    j1 = np.asarray(jamb1, dtype=float)
    e = j1 - j0
    length = float(np.hypot(e[0], e[1]))
    if length < 1e-9 or beams.points.size == 0:
        return 0
    u = e / length                      # along the jamb line
    nrm = np.array([-u[1], u[0]])       # across it
    lx, ly = beams.laser
    s_l = float((lx - j0[0]) * nrm[0] + (ly - j0[1]) * nrm[1])
    if abs(s_l) < 1e-9:
        # The laser is ON the jamb line: there is no "far side" to reach.
        return 0
    sign = -1.0 if s_l > 0.0 else 1.0   # positive = away from the laser
    d = beams.points - j0
    s = (d[:, 0] * nrm[0] + d[:, 1] * nrm[1]) * sign
    far = s >= float(p.beyond_m)
    if not np.any(far):
        return 0
    # Where each far beam crosses the line, as a fraction along L -> P.
    pts = beams.points[far]
    s_far = s[far]
    s_laser = abs(s_l)
    t = s_laser / (s_laser + s_far)     # in (0, 1]: s_far > 0, s_laser > 0
    cross_x = lx + (pts[:, 0] - lx) * t
    cross_y = ly + (pts[:, 1] - ly) * t
    along = (cross_x - j0[0]) * u[0] + (cross_y - j0[1]) * u[1]
    margin = float(p.jamb_exclude_m)
    inside = (along >= margin) & (along <= length - margin)
    return int(np.count_nonzero(inside))


# ---------------------------------------------------------------------------
# The detector
# ---------------------------------------------------------------------------
def _bottleneck_cells(window, skel, p):
    """The skeleton cells worth evaluating as doorways, narrowest first.

    THREE FILTERS, and the order is the point. All three only ever remove
    cells that `evaluate` would have rejected anyway, so the set of ACCEPTED
    doors is unchanged by them -- what changes is which candidates get to
    spend the `max_candidates` budget, and that turned out to matter a great
    deal on a real grid (see below).

      1. A CURVE CELL, degree exactly 2. A tip has no two sides and a branch
         point has three, so neither can be a doorway pinch; `_sides` already
         refuses both with code 'junction'. On a real slam_toolbox grid this
         is most of the skeleton: the free-space boundary is ragged where it
         meets unexplored space, and the medial axis of that is bushy --
         measured on corridor10m_20261019T131815's last grid, 814 of 2303
         skeleton cells were branch points and only 1476 were curve cells.

      2. A LOCAL MINIMUM of clearance along the skeleton, within
         `local_min_radius_m`. A square minimum filter over the clearance
         with non-skeleton cells set to +inf, so a cell survives only if no
         skeleton cell near it is narrower.

      3. IN THE WIDTH BAND, on 2 x clearance.

    THE BAND MUST COME LAST. Filtering by the band first and then taking
    each cluster's narrowest cell -- which is what this function did at
    first -- does not find local minima at all: it finds cells sitting on
    the band's own lower EDGE. Wherever the skeleton descends through
    0.70 m on its way into a tight corner, the in-band cluster is cut off
    at exactly 0.70 and the cluster's argmin is that cut, not a pinch.
    Measured on the same real grid: all 38 candidates came back at
    2d = 0.700 exactly, every one of them rejected ('junction', 'not_min',
    'asymmetric'), 22 of the 38 clusters never looked at because
    max_candidates is 16 -- and a genuine 0.90 m doorway, whose pinch sorts
    AFTER every 0.70 m band-edge artefact in a narrowest-first ordering,
    would have been cut from the list before it was ever evaluated. The
    accepted set was right; the recall under the cap was not.

    Clusters are then reduced to one cell each -- the least clearance -- so
    a plateau of equally narrow cells produces ONE candidate rather than
    twenty. Without that the tracker, which associates each candidate to at
    most one track, would spawn a track per cell and a single doorway would
    come out as a row of doors.
    """
    d = window.clearance
    curve = np.asarray(skel, dtype=bool) & (neighbour_count(skel) == 2)
    if not curve.any():
        return []
    radius = max(1, int(round(float(p.local_min_radius_m) / window.res)))
    on_skel = np.where(skel, d, np.inf)
    floor = ndimage.minimum_filter(on_skel, size=2 * radius + 1, mode='constant',
                                   cval=np.inf)
    keep = (curve & (d <= floor + 1e-9)
            & (2.0 * d >= float(p.width_min_m))
            & (2.0 * d <= float(p.width_max_m)))
    if not keep.any():
        return []
    labels, n = ndimage.label(keep, structure=_cell_structure())
    out = []
    for lab in range(1, n + 1):
        rows, cols = np.nonzero(labels == lab)
        k = int(np.argmin(d[rows, cols]))
        out.append((int(rows[k]), int(cols[k])))
    # Deterministic order, narrowest first, so max_candidates keeps the
    # tightest real pinches rather than the top-left ones.
    out.sort(key=lambda rc: (round(float(d[rc]), 6), rc[0], rc[1]))
    return out


def evaluate(window, skel, cell, beams, robot_xy, p):
    """Run every gate on one bottleneck cell; return a coded DoorCandidate.

    Order matters: cheap shape tests first, then the jamb marches, then the
    scan. The returned candidate always carries real geometry when it has
    any, so a rejected one is still worth drawing as a debug marker.
    """
    d0 = float(window.clearance[cell])
    width_d = 2.0 * d0
    cx, cy = window.to_map(*cell)
    cx, cy = float(cx), float(cy)

    def coded(code, j0=None, j1=None, n_beams=0):
        if j0 is None:
            # No jamb geometry yet: a degenerate pair centred on the cell,
            # width_d apart along x, purely so the candidate has a position
            # to be drawn and counted at.
            h = max(width_d, 1e-3) / 2.0
            j0, j1 = (cx - h, cy), (cx + h, cy)
        return DoorCandidate(j0, j1, CASE_FREESPACE, NO_WALL_ID, NO_WALL_ID,
                             code=code, n_beams=n_beams)

    if not (float(p.width_min_m) <= width_d <= float(p.width_max_m)):
        return coded('width')

    sides = _sides(skel, cell, p, window.res)
    if sides is None:
        return coded('junction')
    walk_a, walk_b = sides
    if not walk_a or not walk_b:
        return coded('junction')

    d = window.clearance
    rise = float(p.rise_ratio) * d0
    reach = float(p.rise_within_m) - window.res * 1.5
    for walk in (walk_a, walk_b):
        vals = np.array([d[rc] for rc in walk])
        # A dead end: the walk could not get `rise_within_m` away because the
        # skeleton ended. Checked before the rise, so a short branch reports
        # 'spur' and not 'no_rise' -- the difference between "nowhere to go"
        # and "it got no wider".
        if max(walk.values()) < reach:
            return coded('spur')
        if np.any(vals < d0 - 1e-9):
            return coded('not_min')
        if float(vals.max()) < rise:
            return coded('no_rise')

    tangent = _tangent(window, cell, walk_a, walk_b, p)
    if tangent is None:
        return coded('junction')
    across = (-tangent[1], tangent[0])
    limit = float(p.width_max_m)
    j0 = march_to_jamb(window, (cx, cy), across, limit)
    j1 = march_to_jamb(window, (cx, cy), (-across[0], -across[1]), limit)
    if j0 is None or j1 is None:
        return coded('asymmetric')
    width = math.hypot(j1[0] - j0[0], j1[1] - j0[1])
    if not (float(p.width_min_m) <= width <= float(p.width_max_m)):
        return coded('width', j0, j1)
    # The marches and the distance transform must agree. They only can if the
    # skeleton cell really sits mid-gap AND the nearest obstacle is one of the
    # two jambs -- i.e. if this is a clean bottleneck and not, say, a cell
    # whose clearance is set by a wall it is running alongside.
    if abs(width - width_d) > float(p.symmetry_tol_m):
        return coded('asymmetric', j0, j1)
    # BOTH JAMBS MUST BE PART OF SOMETHING. The march stops at the first
    # non-free cell, and one cell is enough to stop it -- so without this a
    # chair leg, a single stray return or a two-cell map artefact standing
    # the right distance from a wall is a jamb, and the pair is a doorway.
    # Found on a real grid: corridor10m_20261019T131815 confirmed one
    # 1.11 m "door" whose far jamb was a 2-cell blob (the near one was a
    # solid wall), after the bottleneck, rise, symmetry AND 12-beam scan
    # gates had all passed. Nothing earlier in the chain asks how much
    # obstacle is behind a jamb.
    min_cells = int(p.jamb_min_cells)
    if min_cells > 0:
        for jamb in (j0, j1):
            if jamb_cells(window, jamb, p.jamb_radius_m) < min_cells:
                return coded('thin_jamb', j0, j1)

    n_beams = beam_hits(j0, j1, beams, p) if beams is not None else 0
    if n_beams == 0:
        return coded('not_beyond', j0, j1, n_beams)
    if n_beams < int(p.min_beams):
        return coded('few_beams', j0, j1, n_beams)
    return coded('', j0, j1, n_beams)


def skeleton_candidates(window, skel, beams, robot_xy, p):
    """Every bottleneck of `window`, evaluated. The MEDIAL-AXIS entry point.

    Kept, and kept tested, but off by default (`use_skeleton`): on the real
    map it finds nothing, because a room behind a door is a ray fan rather
    than a region and the rise test it is built around cannot hold there.
    See the module docstring. `scan_candidates` is what runs.

    Returns a list of semantic_doors.DoorCandidate -- the accepted ones with
    code '' and the rejected ones carrying why, because the rejections are
    what the debug dict and the debug markers are made of. Hand the list
    straight to a semantic_doors.DoorTracker.
    """
    cells = _bottleneck_cells(window, skel, p)[:int(p.max_candidates)]
    return [evaluate(window, skel, c, beams, robot_xy, p) for c in cells]


def skeleton_points(window, skel):
    """(n, 2) map-frame centres of every skeleton cell -- for the debug markers."""
    rows, cols = np.nonzero(skel)
    if rows.size == 0:
        return np.zeros((0, 2), dtype=float)
    x, y = window.to_map(rows, cols)
    return np.column_stack([x, y])
