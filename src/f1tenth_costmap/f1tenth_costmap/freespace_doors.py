"""A third door source: the OCCUPANCY GRID, with no walls involved at all.

WHY A THIRD ONE. The other two detectors both start from a wall. The profile
detector (f1tenth_perception/door_detection.py) walks a beam profile along a
host wall line; the semantic one (semantic_doors.py) pairs two tracked wall
segments. Both inherit the wall extractor's own limits: /perception/walls
carries at most four tracks, a wall has to be long and straight enough to be
fitted at all, and a doorway whose jambs were never fitted as two separate
walls is invisible to both. This one asks a different question entirely --
"where does the free space pinch?" -- and needs no wall, no fit and no track.

THE IDEA, in one paragraph. Crop slam_toolbox's own /slam/map to a window
around the car and call everything that is not KNOWN FREE occupied (unknown
included: an unexplored cell is not a passage). Take the Euclidean distance
transform of the free space: every free cell now carries its clearance d, the
distance to the nearest thing. Thin the free space to its medial axis -- a
one-cell-wide curve running down the middle of every corridor and through
every doorway. A doorway is then a LOCAL MINIMUM of d along that curve whose
2d falls in the door width band, with the clearance rising on both sides: a
bottleneck between two wider regions. That last condition is what separates a
doorway from a corridor that merely narrows, and from the car's own
half-explored surroundings.

WHAT IT DOES NOT DO. It never decides a door exists from the grid alone. The
grid is a 5-second-old accumulation (see output/doors/freespace.md); a gap in
it can be a gap in the MAP rather than a gap in the WALL -- two scans that
never quite joined up, a doorway-sized hole punched by a moving person who has
since left. So every candidate also has to be confirmed LIVE from /scan:
beams must go through the opening and reach `beyond_m` past the jamb line.
A candidate that never gets that never confirms, and so never reaches a
mission (see `beam_hits`).

WHAT IT SHARES WITH semantic_doors.py. Everything downstream: this module
produces semantic_doors.DoorCandidate objects and hands them to
semantic_doors.DoorTracker. Stable ids, 0.3 m association, the confirm count,
the smoothed pose, the direction fixed once at first confirmation, "kept in
the map after leaving view" -- all of it is that tracker's behaviour, not a
second copy of it. Only the way a candidate is FOUND is new. `case` is 'F' so
a reader can tell the three sources apart (markers colour it amber), and the
parent wall ids are -1 because there are no parent walls.

FRAMES. The grid is in map, so this module is in MAP throughout -- the
candidates, the jambs, the scan beams. The semantic detector works in
base_link and transforms the walls in; this one has nothing to gain by it.

GEOMETRY CONVENTIONS, so the tracker reuse is exact:
  - `axis` of a DoorCandidate is JAMB TO JAMB, and the tracker builds the door
    frame's x from the perpendicular to it. Here the perpendicular to the jamb
    line IS the skeleton tangent (the jambs are found by marching
    perpendicular to that tangent), so the tracker's own "point the normal
    away from the car" rule yields exactly "the skeleton tangent oriented from
    the robot's side to the far side", with no extra code.
  - cell (row, col) -> map: x = origin_x + (col + 0.5) * res, same cell-CENTRE
    convention costmap_boundary.py's own _car_frame_cells uses.

NO rclpy, NO message types, NO TF: numpy and scipy.ndimage only, so every
gate below is testable on a hand-drawn grid. scikit-image is deliberately NOT
a dependency -- its medial_axis/skeletonize would be the obvious tool, but
python3-skimage is not installed on this workspace's machines and adding it to
the car's image for one function is a worse trade than the 40 lines of
Zhang-Suen thinning in `thin` below (which is deterministic and needs nothing
beyond numpy).
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
# Rejection codes
# ---------------------------------------------------------------------------
#: Why a bottleneck was not a door. '' is a door. Deliberately the SAME WORDS
#: as semantic_doors where the question is the same ('width', 'few_beams'), so
#: a debug dict from either source reads the same way, and different words
#: where the question is different -- there is no 'angle' or 'offset' here
#: because there is no wall pair to be parallel.
#:
#:   width        2d outside [width_min_m, width_max_m]
#:   junction     the skeleton cell is a tip or a branch point, not a curve
#:   not_min      clearance is not a local minimum along the skeleton
#:   no_rise      clearance does not rise to rise_ratio x d on BOTH sides
#:   spur         the skeleton ran out within rise_within_m on one side: a
#:                dead end, not a passage between two regions
#:   asymmetric   the two jamb marches disagree with 2d: the skeleton is not
#:                centred in this gap, so it is not a clean bottleneck
#:   thin_jamb    a jamb has too little obstacle behind it: a doorway's jamb
#:                is part of a wall, and a two-cell blob is a chair leg, a map
#:                artefact or one stray return
#:   few_beams    fewer than min_beams beams went through the opening
#:   not_beyond   beams reached the opening but not beyond_m past it: the
#:                opening is a dent in a wall, not a hole through it
SHAPE_CODES = ('width', 'junction', 'not_min', 'no_rise', 'spur', 'asymmetric',
               'thin_jamb')
PASSAGE_CODES = ('few_beams', 'not_beyond')

ALL_CODES = SHAPE_CODES + PASSAGE_CODES


def code_group(code):
    """'ok' | 'shape' | 'passage' for a candidate's code.

    Mirrors semantic_doors.code_group so a consumer can ask "was the geometry
    plausible at all" without knowing the individual codes. Unlike that one
    this raises on a code it does not know, rather than guessing a group: the
    two code sets are disjoint and mixing them up would read as a passage
    failure when it was a shape failure.
    """
    if not code:
        return 'ok'
    if code in SHAPE_CODES:
        return 'shape'
    if code in PASSAGE_CODES:
        return 'passage'
    raise ValueError(f'not a freespace_doors code: {code!r}')


# ---------------------------------------------------------------------------
# Parameters
# ---------------------------------------------------------------------------
class FreespaceParams:
    """Every threshold of this module, one object, all SI.

    Defaults are the shipped ones; stack_params.yaml is the single source for
    what the car actually runs (freespace_door_node builds this from it).
    """

    __slots__ = ('window_m', 'occupied_threshold', 'cleanup_iterations',
                 'width_min_m', 'width_max_m', 'rise_ratio', 'rise_within_m',
                 'spur_prune_m', 'local_min_radius_m', 'tangent_span_m',
                 'symmetry_tol_m', 'jamb_radius_m', 'jamb_min_cells',
                 'min_beams', 'beyond_m', 'jamb_exclude_m',
                 'assoc_max_m', 'confirm_hits', 'smooth_alpha',
                 'candidate_forget_s', 'forget_unseen_s', 'max_candidates')

    def __init__(self, **kw):
        # --- the window
        self.window_m = 8.0
        self.occupied_threshold = 65
        self.cleanup_iterations = 1
        # --- the bottleneck
        self.width_min_m = 0.70
        self.width_max_m = 1.40
        self.rise_ratio = 1.3
        self.rise_within_m = 1.0
        self.spur_prune_m = 0.30
        self.local_min_radius_m = 0.30
        self.tangent_span_m = 0.25
        self.symmetry_tol_m = 0.20
        self.jamb_radius_m = 0.15
        self.jamb_min_cells = 4
        # --- the live confirmation
        self.min_beams = 4
        self.beyond_m = 0.50
        self.jamb_exclude_m = 0.08
        # --- tracking (handed straight to semantic_doors.DoorTracker)
        self.assoc_max_m = 0.30
        self.confirm_hits = 3
        self.smooth_alpha = 0.30
        self.candidate_forget_s = 12.0
        self.forget_unseen_s = 0.0   # 0 = a confirmed door is never forgotten
        self.max_candidates = 16
        for k, v in kw.items():
            if k not in self.__slots__:
                raise TypeError(f'FreespaceParams got an unknown parameter {k!r}')
            setattr(self, k, v)

    def door_params(self):
        """A semantic_doors.DoorParams carrying only the tracking knobs.

        DoorTracker reads exactly these six off its params object and nothing
        else; the rest of DoorParams is the semantic detector's geometry and
        is irrelevant here. Built rather than subclassed so there is one
        tracker, with one behaviour, for all three door sources.
        """
        from f1tenth_costmap.semantic_doors import DoorParams
        return DoorParams(
            assoc_max_m=self.assoc_max_m,
            confirm_hits=self.confirm_hits,
            smooth_alpha=self.smooth_alpha,
            candidate_forget_s=self.candidate_forget_s,
            forget_unseen_s=self.forget_unseen_s)


# ---------------------------------------------------------------------------
# The window
# ---------------------------------------------------------------------------
class Window:
    """One cropped, cleaned patch of the grid, plus its clearance field.

    `free` is True where the cell is KNOWN FREE after cleanup; `clearance` is
    the distance [m] from each cell to the nearest non-free cell (0 on an
    occupied cell). `origin` is the map-frame position of the (0, 0) cell's
    own CORNER, so a cell centre is origin + (index + 0.5) * res -- the same
    convention costmap_boundary.py uses.
    """

    __slots__ = ('free', 'clearance', 'res', 'origin')

    def __init__(self, free, clearance, res, origin):
        self.free = free
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

    def is_free(self, x, y):
        """Is the map-frame point inside the window AND known free?"""
        r, c = self.to_cell(x, y)
        if not (0 <= r < self.free.shape[0] and 0 <= c < self.free.shape[1]):
            return False
        return bool(self.free[r, c])


def _cell_structure():
    """The 3x3 all-ones structuring element: 8-connectivity, one cell of reach."""
    return np.ones((3, 3), dtype=bool)


def occupancy_window(grid_data, width, height, resolution, origin_x, origin_y,
                     robot_xy, p):
    """Crop the grid around the robot, mark free space, and clean it up.

    UNKNOWN IS OCCUPIED. Every cell that is not strictly below
    `occupied_threshold` and not UNKNOWN is free; -1 and >= threshold are
    both obstacles. This is the opposite of costmap_boundary.py's own rule
    (where unknown is never occupied) and deliberately so: that node asks
    "is there definitely something there", which must not be answered yes by
    ignorance; this one asks "may the car drive here", which must not be
    answered yes by ignorance either. Both pick the conservative answer and
    they differ because the questions are opposite.

    The window is always exactly the requested size: anything outside the
    grid's own extent is filled UNKNOWN, i.e. occupied, so a door is never
    invented at the edge of the map.

    CLEANUP, two steps, both of which can only SHRINK the free space:
      1. binary_closing on the occupied mask -- fills pinholes in a wall, so
         a one- or two-cell hole from a missed ray is not a 0.10 m doorway.
      2. binary_opening on the free mask -- drops isolated free specks that
         would otherwise seed a skeleton of their own.
    At 0.05 m/cell one iteration reaches 0.10 m, an order below width_min_m,
    so neither step can close or open a real doorway.
    """
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

    free = (sub >= 0) & (sub < int(p.occupied_threshold))
    it = int(p.cleanup_iterations)
    if it > 0:
        st = _cell_structure()
        occ = ndimage.binary_closing(~free, structure=st, iterations=it,
                                     border_value=1)
        free = ndimage.binary_opening(~occ, structure=st, iterations=it,
                                      border_value=0)
    clearance = ndimage.distance_transform_edt(free, sampling=res)
    origin = (origin_x + c0 * res, origin_y + r0 * res)
    return Window(free, np.asarray(clearance, dtype=float), res, origin)


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


def candidate_doors(window, skel, beams, robot_xy, p):
    """Every bottleneck of `window`, evaluated. The module's one entry point.

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
