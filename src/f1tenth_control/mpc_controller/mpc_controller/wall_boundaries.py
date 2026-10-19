"""wall_boundaries.py: MPC half-plane rows from the multi-wall tracker's tracks.

Pure: math and numpy only, no rclpy and no f1tenth_messages import, so every
rule below is unit-testable without a node or a built message package -- the
same split mpc_solver.py's own pad_boundary_constraints/boundary_constraint_
bounds block already uses, and the reason MPCController never has to be
constructed in a test (its __init__ opens log files and starts a timer).

WHY THIS EXISTS. /costmap/boundaries' rows come from costmap_boundary.py's
nearest-OCCUPIED-CELL extraction on /slam/map: one radial tangent line per
direction, at the single closest occupied cell in an angular window. A small
island of occupied cells -- a stale map artefact, a person already handled by
w_obs, one spurious return integrated once -- is indistinguishable from a wall
to that extraction, and it becomes a half-plane row across the whole horizon,
which the solver then cannot satisfy and the car stops. A TRACKED wall has
what an island does not: a length, a lifecycle (it had to be confirmed), a
measured extent with ends, an evidence source, and a role saying whether it is
anywhere near the car's path. Those are exactly the admission tests below.

FRAME. Everything here is the ODOM frame, which is where the tracker publishes
(MultiWallTrackArray.header.frame_id, from lidar_front_wall_node's own
/odometry/filtered -- the SAME local-EKF odom MPC_corr's x/y/yaw live in) and
where the solver's rows are evaluated. There is deliberately no map -> odom
lookup anywhere in this module: a wall track is odom-native, so pushing it
through map and back would inject the global EKF's drift into a quantity that
never had any. What IS re-derived every cycle, which is the thing that matters,
is the half-plane's ORIENTATION against the car's CURRENT position
(half_plane_from_track) -- nothing is frozen in base_link, and no snapshot of
the car pose at the scan stamp is reused.

OFFSET AND THE SAFETY MARGIN -- read this before changing a number. The margin
is applied ONCE, and not here: mpc_solver.boundary_constraint_bounds turns an
(normal, offset) row into `normal . p <= offset - car_radius - avoidance_margin`
at every stage. So `offset` on a row this module produces is the RAW geometric
odom-frame offset of the wall line (n . p for p on the wall), exactly as
f1tenth_messages/BoundaryConstraint.msg's own note requires ("offset is NOT
pre-shrunk by any safety margin -- consumers subtract their own"). Subtracting
car_radius + avoidance_margin here as well would double it and stop the car a
footprint early. The ONLY producer-side shrink is the extra coasting margin
(wall_row_coast_margin_m), which is additional conservatism for a wall nobody
is currently looking at, not a restatement of the car's own footprint.

ROW IDENTITY IS GEOMETRY, NOT A TRACK ID. Tracker ids are never reused and
re-spawn often (a wall the association loses and re-finds is a new id with the
same surface), so a diagnostic keyed on them flickers and a slack figure keyed
on them looks like a new violation every re-birth. RowIdentity matches a row to
the previous cycle's rows by NORMAL ANGLE and OFFSET, with the same tolerances
that merge near-duplicates, and only mints a new id when nothing matches. The
track ids are still reported, as provenance.

COASTING AGE IS DERIVED, NOT PUBLISHED. MultiWallTrack carries `state` and
`age` (since birth) but no coasting age, and appending a field to that message
is off the table: CDR is positional, so it would make every bag already
recorded undeserialisable, and those recordings are this project's evidence
base (lidar_front_wall_node._publish_walls says so explicitly). CoastClock
therefore derives it on this side, from the stamps at which a track id was last
seen CONFIRMED. A track first seen already coasting has no such stamp -- the
clock starts at that first sighting instead, which under-estimates its true
coast age, so such a wall is admitted for at most wall_row_max_coast_s from
when THIS process first saw it rather than from when the tracker lost it. That
is the safe direction (it expires sooner than a longer estimate would), and it
only ever happens for tracks that were already coasting when the MPC started.
"""

import math
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np

# MultiWallTrack.msg's own enumerations, mirrored rather than imported so this
# module stays free of f1tenth_messages (see the module docstring). The tests
# assert these stay equal to the message's constants.
STATE_TENTATIVE = 0
STATE_CONFIRMED = 1
STATE_COASTING = 2

SOURCE_LIDAR = 0
SOURCE_STEREO = 1
SOURCE_BOTH = 2

STATE_NAMES = {
    STATE_TENTATIVE: 'tentative',
    STATE_CONFIRMED: 'confirmed',
    STATE_COASTING: 'coasting',
}
SOURCE_NAMES = {SOURCE_LIDAR: 'lidar', SOURCE_STEREO: 'stereo', SOURCE_BOTH: 'both'}

# The roles a row may ever come from. 'back' is excluded along with 'other':
# it is behind the car by construction, so a half-plane from it can only ever
# constrain a reversing horizon, and the tracker's own role hysteresis makes a
# wall the car has just driven past flip front -> back rather than disappear.
ROW_ROLES = ('front', 'left', 'right')


class WallTrackView:
    """The fields of one MultiWallTrack this module reads, and nothing else.

    A plain object rather than the message type so a test can build a wall in
    one line, and so this module imports no ROS package. `from_msg` adapts a
    real MultiWallTrack; the field names are identical to the message's.
    """

    __slots__ = ('id', 'role', 'state', 'source', 'rho', 'theta', 'p0', 'p1')

    def __init__(self, track_id, role, state, source, rho, theta, p0, p1):
        self.id = int(track_id)
        self.role = str(role)
        self.state = int(state)
        self.source = int(source)
        self.rho = float(rho)
        self.theta = float(theta)
        self.p0 = (float(p0[0]), float(p0[1]))
        self.p1 = (float(p1[0]), float(p1[1]))

    @classmethod
    def from_msg(cls, track):
        """One MultiWallTrack -> WallTrackView, reading the ODOM-frame fields.

        `role` is preferred over `label` per the message's own note (they carry
        the same value; role is the documented one). p0/p1 are the odom
        endpoints, NOT p0_bl/p1_bl: those are base_link at the scan stamp, i.e.
        a frozen snapshot of a pose that has since moved.
        """
        return cls(
            track_id=track.id,
            role=(getattr(track, 'role', '') or getattr(track, 'label', '')),
            state=track.state,
            source=track.source,
            rho=track.rho,
            theta=track.theta,
            p0=(track.p0.x, track.p0.y),
            p1=(track.p1.x, track.p1.y),
        )


class WallRow:
    """One admitted half-plane row, plus everything /mpc/wall_constraints_diag reports.

    `offset` is the RAW odom-frame offset (see the module docstring): the
    solver, not this class, subtracts car_radius + avoidance_margin.
    """

    __slots__ = ('row_id', 'normal_x', 'normal_y', 'offset', 'track_ids', 'role',
                 'state', 'coast_age_s', 'clearance_m', 'active', 'reason')

    def __init__(self, row_id, normal_x, normal_y, offset, track_ids, role, state,
                 coast_age_s, clearance_m, active=True, reason=''):
        self.row_id = row_id
        self.normal_x = float(normal_x)
        self.normal_y = float(normal_y)
        self.offset = float(offset)
        self.track_ids = tuple(int(t) for t in track_ids)
        self.role = str(role)
        self.state = str(state)
        self.coast_age_s = float(coast_age_s)
        self.clearance_m = float(clearance_m)
        self.active = bool(active)
        self.reason = str(reason)

    @property
    def constraint(self) -> Tuple[float, float, float]:
        """Return the (normal_x, normal_y, offset) triple solve_mpc_step wants."""
        return (self.normal_x, self.normal_y, self.offset)

    def as_dict(self) -> Dict:
        """JSON-ready diagnostic record, one per row (see MPC_corr's wall diag)."""
        return {
            'row_id': self.row_id,
            'track_ids': list(self.track_ids),
            'role': self.role,
            'state': self.state,
            'normal': [round(self.normal_x, 6), round(self.normal_y, 6)],
            'offset': round(self.offset, 4),
            'clearance_m': round(self.clearance_m, 4),
            'coast_age_s': round(self.coast_age_s, 3),
            'active': self.active,
            'reason': self.reason,
        }

    def __repr__(self):
        return (f'WallRow(id={self.row_id}, n=({self.normal_x:.3f},{self.normal_y:.3f}), '
                f'offset={self.offset:.3f}, role={self.role}, active={self.active}, '
                f'reason={self.reason!r})')


# ---------------------------------------------------------------------------
# STEP 2 -- admission
# ---------------------------------------------------------------------------

def admit_track(track: WallTrackView, coast_age_s: float, max_coast_s: float,
                allow_stereo_only: bool) -> Tuple[bool, str]:
    """(admitted, reason) for one track under the wall-row admission rule.

    `reason` is '' when admitted and names the FIRST failing test otherwise, in
    the order checked -- the same first-failure-wins convention
    WallLineFit.msg's own `reason` and the tracker's own front_reason use, so a
    rejected wall reports one cause rather than a set.

    The order is deliberate: role before evidence before lifecycle. A wall with
    role 'other' is a reference wall for nothing no matter how well observed
    (the tracker's own words), so asking anything else about it first would
    report a cause that is not why it was rejected.
    """
    if track.role not in ROW_ROLES:
        return False, f'role_{track.role or "unset"}'
    if track.source == SOURCE_STEREO and not allow_stereo_only:
        # Nothing has ever cross-checked this surface (the message's own note
        # on SOURCE_STEREO). A stereo-only plane became the front wall at 1.1 m
        # on an open corridor once already (corridor_phantom, 2026-10-15); it
        # does not get to be a hard-ish row on that evidence.
        return False, 'stereo_only'
    if track.state == STATE_TENTATIVE:
        return False, 'tentative'
    if track.state == STATE_COASTING and coast_age_s > max_coast_s:
        return False, 'coast_expired'
    return True, ''


def coast_margin_for(track: WallTrackView, coast_margin_m: float) -> float:
    """Extra producer-side shrink for a coasting track, 0.0 for a confirmed one.

    A coasting wall is a PREDICTION carried on odometry, not an observation
    (the tracker's own coast_expired comment), so the row it produces is held
    further away than the surface was last seen. This is the one shrink this
    module applies; car_radius + avoidance_margin is the solver's, applied once
    there (module docstring).
    """
    return float(coast_margin_m) if track.state == STATE_COASTING else 0.0


class CoastClock:
    """Coasting age per track id, derived from the states seen on /perception/walls.

    See the module docstring's "COASTING AGE IS DERIVED" paragraph for why this
    is not simply read off the message. Keyed on the track id, which IS stable
    for the lifetime of one track -- a coast is the same track going unmatched,
    and only a re-birth changes the id, at which point the wall is CONFIRMED
    again and the clock restarts, which is correct.
    """

    def __init__(self, forget_after_s: float = 5.0):
        self.forget_after_s = float(forget_after_s)
        #: track id -> stamp it was last seen CONFIRMED, or first seen at all.
        self._last_confirmed: Dict[int, float] = {}
        #: track id -> stamp it was last seen in any state, for pruning.
        self._last_seen: Dict[int, float] = {}

    def update(self, tracks: Sequence[WallTrackView], stamp: float) -> None:
        """Record this cycle's states. Call once per /perception/walls message."""
        for trk in tracks:
            self._last_seen[trk.id] = stamp
            if trk.state == STATE_CONFIRMED or trk.id not in self._last_confirmed:
                self._last_confirmed[trk.id] = stamp
        stale = [tid for tid, seen in self._last_seen.items()
                 if stamp - seen > self.forget_after_s]
        for tid in stale:
            self._last_seen.pop(tid, None)
            self._last_confirmed.pop(tid, None)

    def age(self, track: WallTrackView, stamp: float) -> float:
        """Seconds this track has been coasting; 0.0 while it is confirmed."""
        if track.state != STATE_COASTING:
            return 0.0
        last = self._last_confirmed.get(track.id)
        if last is None:
            return 0.0
        return max(0.0, float(stamp) - float(last))


# ---------------------------------------------------------------------------
# STEP 3 -- geometry
# ---------------------------------------------------------------------------

def half_plane_from_track(track: WallTrackView, car_xy: Tuple[float, float],
                          extra_margin_m: float = 0.0) -> Tuple[float, float, float]:
    """(normal_x, normal_y, offset) in odom for one wall, normal AWAY from the car.

    The tracker's (rho, theta) is `x cos(theta) + y sin(theta) = rho`, rho >= 0,
    so (cos theta, sin theta) points from the ODOM ORIGIN toward the line -- not
    from the car. Which side the car is on is therefore a question that has to
    be asked, every cycle, against the car's CURRENT position: the car sits at
    the origin of nothing, and a car that drives past a wall's plane (through a
    doorway, round the end of a partition) flips sides without the track
    changing at all.

    `normal . car <= offset` holds by construction after the flip, so the car's
    own position always satisfies its own rows and the half-plane never
    encloses it from the wrong side. On the measure-zero case of the car
    exactly on the line, the unflipped orientation is kept, deterministically.
    """
    nx, ny = math.cos(track.theta), math.sin(track.theta)
    offset = float(track.rho)
    if nx * car_xy[0] + ny * car_xy[1] > offset:
        nx, ny, offset = -nx, -ny, -offset
    return nx, ny, offset - float(extra_margin_m)


def segment_span(p0: Tuple[float, float], p1: Tuple[float, float],
                 normal_x: float, normal_y: float) -> Tuple[float, float]:
    """Return (s_min, s_max) of the segment ALONG the wall.

    Measured in the tangent direction (-normal_y, normal_x) -- the same frame
    the activity test projects path points into, so the two can never disagree
    about where the wall's ends are.
    """
    tx, ty = -normal_y, normal_x
    s0 = p0[0] * tx + p0[1] * ty
    s1 = p1[0] * tx + p1[1] * ty
    return (s0, s1) if s0 <= s1 else (s1, s0)


def row_is_active(p0, p1, normal_x: float, normal_y: float,
                  path_xy: Sequence[Tuple[float, float]], end_margin_m: float) -> bool:
    """Report whether the horizon overlaps this wall's MEASURED extent along it.

    A half-plane is infinite; a wall is not. Without this test the row from a
    2 m partition closes the whole 10 m of free space in line with it, and the
    gap PAST the wall's end -- the doorway, the corridor mouth, the way round --
    becomes infeasible. The tracker learned the same lesson the other way round
    (THE MEASURED EXTENT, corridor10m_20261016T144537): a track whose extent
    was the union of two walls across a corridor mouth held FRONT for 5 s and
    stopped the car 5 m early.

    The test is along-wall only, deliberately: lateral position is what the
    constraint is FOR, so gating on it would switch the row off exactly when
    the car is about to need it. A path point whose projection lands inside
    [s_min - end_margin, s_max + end_margin] activates the row.
    """
    if len(path_xy) == 0:
        return False
    s_min, s_max = segment_span(p0, p1, normal_x, normal_y)
    tx, ty = -normal_y, normal_x
    pts = np.asarray(path_xy, dtype=float).reshape(-1, 2)
    s = pts[:, 0] * tx + pts[:, 1] * ty
    lo, hi = s_min - float(end_margin_m), s_max + float(end_margin_m)
    return bool(np.any((s >= lo) & (s <= hi)))


def clearance_to_path(normal_x: float, normal_y: float, offset: float,
                      path_xy: Sequence[Tuple[float, float]]) -> float:
    """Smallest `offset - normal . p` over the path: how close it comes to the wall.

    Positive everywhere means the whole path is on the safe side. This is the
    ranking key for max_wall_constraints (nearest wall first) and the number
    the diagnostic reports, because "the plan passes 0.08 m from row 3" is the
    question a human actually asks of a wall row. inf for an empty path.
    """
    if len(path_xy) == 0:
        return math.inf
    pts = np.asarray(path_xy, dtype=float).reshape(-1, 2)
    return float(np.min(offset - (pts[:, 0] * normal_x + pts[:, 1] * normal_y)))


def _normals_close(a: WallRow, b: WallRow, angle_rad: float) -> bool:
    dot = a.normal_x * b.normal_x + a.normal_y * b.normal_y
    return math.acos(max(-1.0, min(1.0, dot))) < angle_rad


def merge_rows(rows: Sequence[WallRow], merge_angle_deg: float,
               merge_offset_m: float) -> List[WallRow]:
    """Collapse near-duplicate rows, keeping the MOST RESTRICTIVE of each group.

    The tracker legitimately holds two tracks for one physical surface: the two
    faces of a doorway's reveal, a wall re-found under a new id before the old
    one times out, a long wall split by a person's cut sector. Two rows then say
    nearly the same thing with slightly different offsets, which costs two of
    the four slots and two slack variables for one wall.

    Most restrictive = SMALLEST offset, because the row is `n . p <= offset`.
    The surviving row keeps the union of the group's track ids (so the
    diagnostic still names both sources), the tightest offset, and the state/
    role of the row that offset came from. Both normals are already oriented
    away from the car (half_plane_from_track), so the angle between them is
    directly comparable without any pi-folding.
    """
    angle_rad = math.radians(float(merge_angle_deg))
    out: List[WallRow] = []
    for row in rows:
        for i, kept in enumerate(out):
            if (_normals_close(row, kept, angle_rad)
                    and abs(row.offset - kept.offset) < float(merge_offset_m)):
                tighter, looser = ((row, kept) if row.offset < kept.offset
                                   else (kept, row))
                merged = WallRow(
                    row_id=tighter.row_id,
                    normal_x=tighter.normal_x, normal_y=tighter.normal_y,
                    offset=tighter.offset,
                    track_ids=tuple(sorted(set(kept.track_ids) | set(row.track_ids))),
                    role=tighter.role, state=tighter.state,
                    coast_age_s=max(kept.coast_age_s, row.coast_age_s),
                    clearance_m=min(kept.clearance_m, row.clearance_m),
                    # A merged row is active if EITHER half overlapped the path:
                    # they are one physical surface, so the more restrictive
                    # offset and the wider extent are both the conservative
                    # reading of the same wall.
                    active=kept.active or row.active,
                    reason=('' if (kept.active or row.active)
                            else (tighter.reason or looser.reason)),
                )
                out[i] = merged
                break
        else:
            out.append(row)
    return out


class RowIdentity:
    """Stable row ids across cycles, matched on geometry rather than on track ids.

    See the module docstring's "ROW IDENTITY IS GEOMETRY" paragraph. The match
    tolerances are the SAME ones merge_rows uses, on purpose: two rows close
    enough to be merged within one cycle are close enough to be the same row
    across two of them, and keeping one pair of numbers means a tuning change
    cannot make the two disagree.
    """

    def __init__(self, merge_angle_deg: float, merge_offset_m: float,
                 forget_after_s: float = 5.0):
        self.merge_angle_deg = float(merge_angle_deg)
        self.merge_offset_m = float(merge_offset_m)
        self.forget_after_s = float(forget_after_s)
        self._next_id = 1
        #: row_id -> (normal_x, normal_y, offset, last_seen_stamp)
        self._known: Dict[int, Tuple[float, float, float, float]] = {}

    def assign(self, normal_x: float, normal_y: float, offset: float,
               stamp: float) -> int:
        """Id of the previously-seen row this geometry matches, or a fresh one."""
        angle_rad = math.radians(self.merge_angle_deg)
        best_id, best_delta = None, None
        for row_id, (nx, ny, off, _seen) in self._known.items():
            dot = max(-1.0, min(1.0, normal_x * nx + normal_y * ny))
            if math.acos(dot) >= angle_rad:
                continue
            delta = abs(offset - off)
            if delta >= self.merge_offset_m:
                continue
            if best_delta is None or delta < best_delta:
                best_id, best_delta = row_id, delta
        if best_id is None:
            best_id = self._next_id
            self._next_id += 1
        self._known[best_id] = (normal_x, normal_y, offset, float(stamp))
        return best_id

    def prune(self, stamp: float) -> None:
        """Forget rows not seen for forget_after_s, so ids do not accumulate."""
        for row_id in [r for r, v in self._known.items()
                       if float(stamp) - v[3] > self.forget_after_s]:
            self._known.pop(row_id, None)


# ---------------------------------------------------------------------------
# STEP 2 (the 'other' half) -- confirmed 'other' walls as soft obstacles
# ---------------------------------------------------------------------------

def other_wall_obstacles(tracks: Sequence[WallTrackView], spacing_m: float,
                         radius_m: float) -> List[Tuple[float, float, float]]:
    """Return confirmed role-'other' segments as (x, y, r) odom disks for w_obs.

    'other' means the geometry puts the track nowhere near the car's path, or
    the rectangle rejected it, or it is behind another confirmed wall -- real,
    tracked, and a reference wall for nothing. A row from it would be wrong (it
    is not a bound on where the car may go), but silently discarding it loses a
    surface the car can still reach by turning. The soft w_obs path is the
    right strength for that: it deflects and it never makes a QP infeasible.

    TENTATIVE 'other' tracks are left out entirely -- an unconfirmed track
    nowhere near the path has cleared no test at all, and the w_obs cost is
    exactly where a phantom would quietly bend every trajectory.

    `radius_m` is ADDITIONAL inflation on top of the car_radius +
    avoidance_margin the solver already applies around each disk
    (_linearize_obstacle_cost_rti), so 0.0 -- the default -- is the existing
    unified margin and nothing more, which is what "radius from existing
    margins" means here.
    """
    disks: List[Tuple[float, float, float]] = []
    spacing = max(float(spacing_m), 1e-3)
    for trk in tracks:
        if trk.role in ROW_ROLES or trk.state != STATE_CONFIRMED:
            continue
        (x0, y0), (x1, y1) = trk.p0, trk.p1
        length = math.hypot(x1 - x0, y1 - y0)
        n_steps = max(int(math.ceil(length / spacing)), 1)
        for i in range(n_steps + 1):
            t = i / float(n_steps)
            disks.append((x0 + t * (x1 - x0), y0 + t * (y1 - y0), float(radius_m)))
    return disks


# ---------------------------------------------------------------------------
# The whole rule, in one call
# ---------------------------------------------------------------------------

def build_wall_rows(
        tracks: Sequence[WallTrackView],
        car_xy: Tuple[float, float],
        path_xy: Sequence[Tuple[float, float]],
        stamp: float,
        coast_clock: CoastClock,
        identity: RowIdentity,
        max_rows: int,
        max_coast_s: float,
        coast_margin_m: float,
        allow_stereo_only: bool,
        end_margin_m: float,
        merge_angle_deg: float,
        merge_offset_m: float) -> Tuple[List[WallRow], List[Dict]]:
    """(rows_for_the_solver, diagnostic_records) for one cycle.

    Order: admit, build the half-plane against the car's current position,
    test the row against the path's own along-wall extent, merge
    near-duplicates, then keep the `max_rows` ACTIVE rows whose clearance to
    the path is smallest. Rejected tracks and inactive rows are not dropped
    silently -- they come back in the second return value with their reason,
    which is what /mpc/wall_constraints_diag publishes.

    The caller owns `coast_clock` and `identity` across cycles and has already
    called coast_clock.update() for this message (the clock has to see every
    cycle's states, including the ones this function rejects).
    """
    rows: List[WallRow] = []
    rejected: List[Dict] = []

    for trk in tracks:
        coast_age = coast_clock.age(trk, stamp)
        ok, reason = admit_track(trk, coast_age, max_coast_s, allow_stereo_only)
        if not ok:
            rejected.append({
                'row_id': None, 'track_ids': [trk.id], 'role': trk.role,
                'state': STATE_NAMES.get(trk.state, str(trk.state)),
                'source': SOURCE_NAMES.get(trk.source, str(trk.source)),
                'normal': None, 'offset': None, 'clearance_m': None,
                'coast_age_s': round(coast_age, 3), 'active': False,
                'reason': reason,
            })
            continue
        nx, ny, offset = half_plane_from_track(
            trk, car_xy, coast_margin_for(trk, coast_margin_m))
        active = row_is_active(trk.p0, trk.p1, nx, ny, path_xy, end_margin_m)
        rows.append(WallRow(
            row_id=identity.assign(nx, ny, offset, stamp),
            normal_x=nx, normal_y=ny, offset=offset,
            track_ids=(trk.id,), role=trk.role,
            state=STATE_NAMES.get(trk.state, str(trk.state)),
            coast_age_s=coast_age,
            clearance_m=clearance_to_path(nx, ny, offset, path_xy),
            active=active,
            reason='' if active else 'path_past_segment_end',
        ))

    rows = merge_rows(rows, merge_angle_deg, merge_offset_m)
    identity.prune(stamp)

    active_rows = sorted((r for r in rows if r.active), key=lambda r: r.clearance_m)
    kept = active_rows[:max(int(max_rows), 0)]
    for row in active_rows[len(kept):]:
        row.active = False
        row.reason = 'over_max_wall_constraints'

    diag = [r.as_dict() for r in kept]
    diag += [r.as_dict() for r in rows if r not in kept]
    diag += rejected
    return kept, diag


def check_row_capacity(boundary_source: str, max_wall_constraints: int,
                       boundary_max_sources: int) -> str:
    """'' when the QP has room for every wall row, else the startup error text.

    mpc_solver.pad_boundary_constraints pads OR TRUNCATES to
    boundary_max_sources, and a truncated set of wall rows still looks like a
    complete one -- on the wire, in the diagnostic, and in the solve: four
    walls admitted, three enforced, and the missing one is whichever the
    distance sort happened to put last. The polytope path documented this risk
    in a comment (boundary_max_sources' own description in stack_params.yaml);
    here it is a refusal to start, which is what the task asks for and the only
    form that cannot be missed.

    A pure function rather than an inline check in MPCController.__init__ so it
    is testable: that constructor opens real log files and starts a real
    control-loop timer, so this suite never builds one.
    """
    if boundary_source != 'walls':
        return ''
    if int(max_wall_constraints) <= int(boundary_max_sources):
        return ''
    return (f'boundary_source=walls needs boundary_max_sources (QP rows per '
            f'stage, currently {int(boundary_max_sources)}) >= '
            f'max_wall_constraints ({int(max_wall_constraints)}); '
            f'pad_boundary_constraints would otherwise TRUNCATE '
            f'{int(max_wall_constraints) - int(boundary_max_sources)} wall '
            f'row(s) silently')


def stale_reason(frame_id: str, odom_frame: str, age_s: Optional[float],
                 max_age_s: float) -> str:
    """'' when this cycle's wall message may be used, else why it may not.

    Two separable failures, and the distinction matters because only one of
    them is a fault. An age past max_age_s means the tracker (or the LiDAR, or
    the pose it needs at the scan stamp) has gone quiet, and withholding every
    row is the fail-safe: no rows is "I am not constraining you", which the
    solver already treats as unconstrained, whereas a held-over row is a wall
    asserted at a place the car has since driven through. A frame mismatch is a
    configuration error -- somebody remapped the tracker's odometry or the
    MPC's -- and it must never be papered over by transforming, because the
    whole point is that these two agree about odom by construction.
    """
    if age_s is None:
        return 'walls: no message ever received'
    if frame_id and odom_frame and frame_id != odom_frame:
        return f'walls: frame {frame_id!r} is not the MPC odom frame {odom_frame!r}'
    if age_s > float(max_age_s):
        return f'walls: {age_s:.2f} s old, past {float(max_age_s):.2f} s'
    return ''
