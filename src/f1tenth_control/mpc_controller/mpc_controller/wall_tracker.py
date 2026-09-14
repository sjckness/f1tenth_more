"""
The wall a wall_turn was triggered by, tracked as a line in the odom frame,
and d_wall: the front bumper's perpendicular distance to it.

WHY THE INCREMENT RULE CANNOT KEEP RUNNING ON dFront. wall_turn.py sizes
each corridor increment from d_avail, the distance left in which to finish
rotating, and that distance is governed by the wall that triggered the turn.
dFront is a forward measurement in the car's own frame. Once the car bends,
the sensor axis (or a swept corridor) turns away from that wall: the wall
leaves the measurement's field of view and dFront jumps to whatever lies down
the new heading -- optimistic exactly when the rule needs it honest. Any
forward cone has the same defect, and so does a cone aimed perpendicular to
the wall: both are defined relative to a heading that is rotating throughout
the turn, so they change for reasons unrelated to the car actually moving.

So the wall is held as a geometric object in the odom frame, n . p = c with
n pointing from the car toward the wall, and reported as

    d_wall = |c - n . p_bumper|,     p_bumper = p_car + bumper_x * heading

which has no cone, no field of view and no dependence on the current
heading: correct at every point in the turn, including with the wall fully
abeam. From the bumper, not the rear axle, because mid-turn the nose is
closer to the wall than the axle, and the nose is what contacts.

SELECTION -- ONCE, AT COMMIT. Candidate lines are extracted from the scan
in hand when the turn commits: sequential MSAC over beam-adjacent two-point
hypotheses, a total-least-squares refit on the inliers, the next line from
what is left. Without gating the fit grabs the corridor's side walls, which
are far closer than the front wall and would make d_avail tiny -- committing
the full turn at once, which is exactly the defect being fixed. A candidate
is the front wall only if ALL of these hold, evaluated against psi_commit,
the heading frozen at commit:

  1. Perpendicularity. A line fit gives n only up to sign, so the angle is
     folded into [0, pi/2]: err = |wrap(atan2(ny, nx) - psi_commit)|,
     err = min(err, pi - err), accept if err < normal_tol. A front wall's
     normal lies along the heading; a side wall's is 90 degrees off. The
     tolerance is wide enough for an oblique approach, narrow enough that a
     side wall can never qualify.
  2. Distance window. Reject anything nearer than the distance the turn
     was decided on less dfront_slack (too_close), or farther than it plus
     dfront_slack (too_far). A wall at 0.4 m cannot be the wall the car
     decided to turn for at 4 m, whatever its normal -- and neither can one
     at 10 m: on an archived run that started 1.25 m behind an obstacle,
     the corridor's genuine end wall 10 m away passed every other gate and
     would have removed the distance cap from a turn decided on 1.25 m.
     The reference is the dFront the commit was made WITH, not the nominal
     trigger distance: a wall_turn issued once dFront already reads below
     the trigger (the mission pattern: straight until 4.0 m, then a 90
     degree turn whose trigger is 4.27 m) commits on its first rebuild with
     the wall INSIDE the nominal trigger, and a floor at the trigger would
     reject it. With dFront unknown at commit there is no window, and the
     caller logs that.
  3. Ahead-ness. The closest wall point must be ahead: (p_closest - p_car)
     . heading_commit > 0. Kills anything abeam or behind.
  4. Span. At least min_span of inliers, and at least min_inliers of them.
     A short cluster fits any line you like: a doorframe or a box corner
     makes a perfect two-point wall.

Of several passing candidates the smallest angle error wins. A rejected
candidate reports the FIRST gate it failed, in that order, so the log shows
what the fit landed on and why it was refused.

TRACKING -- NEVER RE-GATED. After selection the line is refitted, on every
control tick that has a fresh scan, from returns ASSOCIATED to it by
proximity (within assoc_dist of the predicted line), never by orientation,
and none of the four gates run again. The car is rotating, so a correctly
tracked front wall fails the perpendicularity test partway through the turn
-- precisely when it is still needed. psi_commit and the line's identity are
frozen; only its parameters refresh. The association gate is tight so the
next corridor's wall is not absorbed into the fit, and the refit tightens to
the LiDAR's own inlier distance so the few returns of an adjoining wall near
the corner cannot lever the normal. Each corridor rebuild then reads a fit
at most one tick old.

FALLBACK. When the refit degrades (too few associated returns, too short a
span, no fresh scan) d_wall is dead-reckoned: the stored line is already in
the odom frame, so the bumper's distance to it from the live pose IS the last
good value carried forward by odometry. Never an ungated refit -- that
reintroduces the side-wall problem. The odometry scale bias documented in
f1tenth_perception's lidar_front_wall_node.py (distance ~17-20% short, so the
wall reads farther than it is) applies to the dead-reckoned stretch and is
not compensated here, for the reasons given there. It is also why the refit
is per tick and not per rebuild: replayed on the archive with 1 s between
refits, d_wall ran a 0.15 m sawtooth from exactly that bias.

FRAMES. Scan returns are unprojected in base_link (x forward, y left) with
the laser's static pose, then placed in odom with the car pose the scan was
taken at. Lines, poses and d_wall are all odom frame -- the frame MPC_corr's
own state and corridor already live in.

Pure logic, no rclpy: MPC_corr.py is the ROS glue (scan, odometry, TF,
publishing), on the model of lidar_front_wall.py / lidar_front_wall_node.py.
The MSAC line fit is a local re-implementation rather than an import of
f1tenth_perception.lidar_front_wall: a control package reaching into a
perception package's internals for forty lines of numpy is a worse coupling
than the duplication.
"""

import math
from dataclasses import dataclass
from typing import Optional, Tuple

import numpy as np

# Mirror f1tenth_messages/WallTrack.msg's PROVENANCE_* constants.
PROVENANCE_NONE = 0
PROVENANCE_MEASURED = 1
PROVENANCE_DEAD_RECKONED = 2
PROVENANCE_NAMES = {
    PROVENANCE_NONE: 'none',
    PROVENANCE_MEASURED: 'measured',
    PROVENANCE_DEAD_RECKONED: 'dead_reckoned',
}

# Two-point hypotheses pair each return with the return this many places
# later in beam order. Beam-adjacent pairs lie on one surface far more often
# than random ones, so a few offsets per return cover every surface in view
# with a fraction of the hypotheses an all-pairs scheme needs. 6 and 12
# beams of a 0.25 deg grid are 1.5 and 3 degrees: 10 and 21 cm of baseline
# at 4 m, 2.6 and 5.2 cm at 1 m.
_HYPOTHESIS_BEAM_OFFSETS = (6, 12)
# A hypothesis through two returns closer than this is dominated by range
# noise in its direction (2.8 mm median residual on this Hokuyo).
_MIN_HYPOTHESIS_BASELINE_M = 0.02
# Lines extracted per selection before giving up. The front wall is the
# first or second surface by return count in any corridor; six leaves room
# for the two side walls and some clutter to be pulled out ahead of it.
_MAX_CANDIDATES = 6
# Refit gates, as multiples of the LiDAR inlier distance, applied after the
# association gate: a coarse pass that drops an adjoining wall's corner
# returns, then the inlier distance itself.
_REFIT_GATE_MULTIPLES = (3.0, 1.0)
# Inlier projections along the line further apart than this start a new
# run, and a candidate's span is its LONGEST run, not max - min. Without
# this, returns scattered across a room that happen to line up read as one
# long wall: on an archived scan 23 returns over 16 m passed as a 16 m wall
# with a better angle than the real one (315 returns, 3.6 m). Consecutive
# returns on a wall are a few cm apart; a doorway (~0.9 m) splits a wall
# into panels, each of which then has to carry min_span on its own.
_SPAN_MAX_GAP_M = 0.25

# Gate verdicts, in the order they are tested.
REASON_ACCEPTED = 'accepted'
REASON_OBLIQUE = 'oblique'
REASON_TOO_CLOSE = 'too_close'
REASON_TOO_FAR = 'too_far'
REASON_BEHIND = 'behind'
REASON_SHORT_SPAN = 'short_span'
REASON_FEW_INLIERS = 'few_inliers'


def wrap_to_pi(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def scan_to_odom_points(ranges, angle_min, angle_increment, range_min, range_max,
                        laser_pose, car_pose) -> np.ndarray:
    """Valid returns of one scan as (N, 2) odom-frame points, in beam order.

    laser_pose is (x, y, yaw) of the scan frame in base_link; car_pose is
    (x, y, yaw) of base_link in odom when the scan was taken. NaN, +-Inf and
    anything outside [range_min, range_max] is not a return.
    """
    ranges = np.asarray(ranges, dtype=np.float64)
    laser_x, laser_y, laser_yaw = laser_pose
    car_x, car_y, psi = car_pose
    angles = angle_min + np.arange(ranges.size) * angle_increment + laser_yaw
    with np.errstate(invalid='ignore'):
        valid = np.isfinite(ranges) & (ranges >= range_min) & (ranges <= range_max)
    r = ranges[valid]
    a = angles[valid]
    bx = laser_x + r * np.cos(a)
    by = laser_y + r * np.sin(a)
    c, s = math.cos(psi), math.sin(psi)
    return np.column_stack([car_x + c * bx - s * by, car_y + s * bx + c * by])


@dataclass(frozen=True)
class Line:
    """n . p = c in the odom frame, |n| = 1, n pointing from the car toward
    the wall at selection (so offset() is positive on the car's side)."""

    nx: float
    ny: float
    c: float

    @property
    def normal(self) -> np.ndarray:
        return np.array([self.nx, self.ny])

    @property
    def tangent(self) -> np.ndarray:
        return np.array([-self.ny, self.nx])

    @property
    def normal_yaw(self) -> float:
        return math.atan2(self.ny, self.nx)

    def offset(self, px: float, py: float) -> float:
        """Signed distance from (px, py) to the line, positive before it."""
        return self.c - (self.nx * px + self.ny * py)


@dataclass(frozen=True)
class Candidate:
    line: Line
    inlier_count: int
    span_m: float
    rms_m: float
    angle_err_rad: float
    distance_m: float   # front bumper to the line, perpendicular
    ahead: bool
    reason: str         # REASON_ACCEPTED, or the first gate it failed


@dataclass(frozen=True)
class Selection:
    accepted: Optional[Candidate]
    candidates: Tuple[Candidate, ...]   # every line extracted, in extraction order
    window_m: Optional[Tuple[float, float]]   # (floor, ceiling); None with dFront unknown
    n_points: int       # returns ahead of the car offered to the extractor


@dataclass(frozen=True)
class TrackStep:
    """One tick's refit verdict. d_wall is from the live pose either way."""

    valid: bool
    provenance: int
    d_wall: float
    reason: str          # 'measured', or why this rebuild dead-reckons
    inlier_count: int
    span_m: float
    since_last_fit_sec: float


def _tls_line(points: np.ndarray) -> Tuple[float, float, float]:
    """Total-least-squares line through `points`: (nx, ny, c), |n| = 1."""
    centroid = points.mean(axis=0)
    _, _, vt = np.linalg.svd(points - centroid, full_matrices=False)
    normal = vt[-1]
    return float(normal[0]), float(normal[1]), float(normal @ centroid)


def _msac_line(points: np.ndarray, inlier_distance_m: float):
    """Best line through `points` (beam order) by MSAC over beam-adjacent
    two-point hypotheses, then two TLS refits on the inliers. Returns
    (nx, ny, c, inlier_mask) or None when no three returns agree."""
    n = len(points)
    pairs_i, pairs_j = [], []
    for k in _HYPOTHESIS_BEAM_OFFSETS:
        if n > k:
            idx = np.arange(n - k)
            pairs_i.append(idx)
            pairs_j.append(idx + k)
    if not pairs_i:
        return None
    i = np.concatenate(pairs_i)
    j = np.concatenate(pairs_j)
    delta = points[j] - points[i]
    length = np.hypot(delta[:, 0], delta[:, 1])
    keep = length > _MIN_HYPOTHESIS_BASELINE_M
    if not np.any(keep):
        return None
    i, delta, length = i[keep], delta[keep], length[keep]
    normals = np.column_stack([-delta[:, 1], delta[:, 0]]) / length[:, None]
    offsets = np.einsum('ij,ij->i', normals, points[i])
    residuals = np.abs(points @ normals.T - offsets)
    # Truncated cost, not an inlier count: a tie between two surfaces goes
    # to the one the returns sit tighter on.
    best = int(np.argmin(np.minimum(residuals, inlier_distance_m).sum(axis=0)))
    inliers = residuals[:, best] <= inlier_distance_m

    nx = ny = c = 0.0
    for _ in range(2):
        if np.count_nonzero(inliers) < 3:
            return None
        nx, ny, c = _tls_line(points[inliers])
        inliers = np.abs(points @ np.array([nx, ny]) - c) <= inlier_distance_m
    if np.count_nonzero(inliers) < 3:
        return None
    return nx, ny, c, inliers


def _span(points: np.ndarray, line: Line) -> Tuple[float, float]:
    """(s_min, s_max) along the line's tangent of the longest run of
    `points` with no gap wider than _SPAN_MAX_GAP_M. See that constant."""
    s = np.sort(points @ line.tangent)
    if s.size == 0:
        return 0.0, 0.0
    breaks = np.flatnonzero(np.diff(s) > _SPAN_MAX_GAP_M)
    starts = np.concatenate(([0], breaks + 1))
    ends = np.concatenate((breaks, [s.size - 1]))
    k = int(np.argmax(s[ends] - s[starts]))
    return float(s[starts[k]]), float(s[ends[k]])


class WallTracker:
    """Owns the commit references, the selected line and the refit state.

    Lifecycle per wall_turn: reset() -> commit(psi, dFront) on the rebuild
    the turn commits -> select(points, pose) until a wall is accepted ->
    update(points, pose) on every later tick -> d_wall(pose) whenever a
    distance is wanted. `now` is always passed in so replay controls the
    clock.
    """

    def __init__(self, *, normal_tol_rad: float, min_span_m: float, min_inliers: int,
                 assoc_dist_m: float, dfront_slack_m: float, bumper_x_m: float,
                 inlier_distance_m: float):
        self.normal_tol_rad = float(normal_tol_rad)
        self.min_span_m = float(min_span_m)
        self.min_inliers = int(min_inliers)
        self.assoc_dist_m = float(assoc_dist_m)
        self.dfront_slack_m = float(dfront_slack_m)
        self.bumper_x_m = float(bumper_x_m)
        self.inlier_distance_m = float(inlier_distance_m)
        self.reset()

    def reset(self):
        self.psi_commit: Optional[float] = None
        self.d_front_commit: Optional[float] = None
        self.line: Optional[Line] = None
        self.selected: Optional[Candidate] = None
        self.last_fit_time: Optional[float] = None
        self.last_inlier_count = 0
        self.last_span_m = math.nan
        self.provenance = PROVENANCE_NONE

    @property
    def armed(self) -> bool:
        return self.psi_commit is not None

    @property
    def has_wall(self) -> bool:
        return self.line is not None

    def commit(self, psi_commit: float, d_front_commit: Optional[float]):
        """Freeze the gating references. A second call within the same turn
        changes nothing: the references belong to the commit instant."""
        if self.psi_commit is None:
            self.psi_commit = float(psi_commit)
            self.d_front_commit = (
                float(d_front_commit) if d_front_commit is not None else None)

    @property
    def window_m(self) -> Optional[Tuple[float, float]]:
        """(floor, ceiling) a candidate's bumper distance must fall in, or
        None when dFront was unknown at commit."""
        if self.d_front_commit is None:
            return None
        return (max(self.d_front_commit - self.dfront_slack_m, 0.0),
                self.d_front_commit + self.dfront_slack_m)

    def bumper(self, car_pose) -> np.ndarray:
        x, y, psi = car_pose
        return np.array([x + self.bumper_x_m * math.cos(psi),
                         y + self.bumper_x_m * math.sin(psi)])

    def d_wall(self, car_pose) -> float:
        """Bumper-to-line perpendicular distance from the live pose; NaN
        without a wall. This is also the dead-reckoned value: the line is
        fixed in odom, so only the pose moves."""
        if self.line is None:
            return math.nan
        bx, by = self.bumper(car_pose)
        return abs(self.line.offset(bx, by))

    # ------------------------------------------------------------------
    # Selection
    # ------------------------------------------------------------------

    def _gate(self, line: Line, inliers: np.ndarray, car_pose) -> Candidate:
        x, y, _psi = car_pose
        # Orient n from the car toward the line before anything reads it.
        if line.offset(x, y) < 0.0:
            line = Line(-line.nx, -line.ny, -line.c)
        heading = np.array([math.cos(self.psi_commit), math.sin(self.psi_commit)])

        angle_err = abs(wrap_to_pi(line.normal_yaw - self.psi_commit))
        angle_err = min(angle_err, math.pi - angle_err)

        bx, by = self.bumper(car_pose)
        distance = abs(line.offset(bx, by))

        s_min, s_max = _span(inliers, line)
        span = s_max - s_min
        # Closest wall point: the foot of the car's perpendicular, clamped to
        # the inlier segment.
        foot = np.array([x, y]) + line.offset(x, y) * line.normal
        s_car = float(np.array([x, y]) @ line.tangent)
        closest = foot + (min(max(s_car, s_min), s_max) - s_car) * line.tangent
        ahead = bool((closest - np.array([x, y])) @ heading > 0.0)

        count = int(len(inliers))
        rms = float(np.sqrt(np.mean((inliers @ line.normal - line.c) ** 2)))

        window = self.window_m
        if angle_err >= self.normal_tol_rad:
            reason = REASON_OBLIQUE
        elif window is not None and distance < window[0]:
            reason = REASON_TOO_CLOSE
        elif window is not None and distance > window[1]:
            reason = REASON_TOO_FAR
        elif not ahead:
            reason = REASON_BEHIND
        elif span < self.min_span_m:
            reason = REASON_SHORT_SPAN
        elif count < self.min_inliers:
            reason = REASON_FEW_INLIERS
        else:
            reason = REASON_ACCEPTED
        return Candidate(line, count, span, rms, angle_err, distance, ahead, reason)

    def select(self, points: np.ndarray, car_pose, now: float) -> Selection:
        """Gate this scan's candidate lines against the commit references and
        adopt the best passing one. Requires commit() first."""
        if self.psi_commit is None:
            raise RuntimeError('select() before commit(): no psi_commit to gate against')
        x, y, _psi = car_pose
        heading = np.array([math.cos(self.psi_commit), math.sin(self.psi_commit)])
        # Only returns ahead of the car can belong to a front wall; halving
        # the set here also halves every hypothesis's residual row.
        remaining = points[(points - np.array([x, y])) @ heading > 0.0]
        n_points = int(len(remaining))

        candidates = []
        for _ in range(_MAX_CANDIDATES):
            if len(remaining) < max(self.min_inliers, 3):
                break
            fit = _msac_line(remaining, self.inlier_distance_m)
            if fit is None:
                break
            nx, ny, c, mask = fit
            candidates.append(self._gate(Line(nx, ny, c), remaining[mask], car_pose))
            remaining = remaining[~mask]

        passing = [cand for cand in candidates if cand.reason == REASON_ACCEPTED]
        accepted = min(passing, key=lambda cand: cand.angle_err_rad) if passing else None
        if accepted is not None:
            self.line = accepted.line
            self.selected = accepted
            self.last_fit_time = float(now)
            self.last_inlier_count = accepted.inlier_count
            self.last_span_m = accepted.span_m
            self.provenance = PROVENANCE_MEASURED
        return Selection(accepted, tuple(candidates), self.window_m, n_points)

    # ------------------------------------------------------------------
    # Tracking
    # ------------------------------------------------------------------

    def _refit(self, points: np.ndarray):
        """TLS refit from returns associated to the stored line. Returns
        (line, count, span) or (None, count, span) with the failing reason
        encoded by the caller from count/span."""
        line = self.line
        gates = (self.assoc_dist_m,) + tuple(
            m * self.inlier_distance_m for m in _REFIT_GATE_MULTIPLES)
        count = 0
        for gate in gates:
            mask = np.abs(points @ line.normal - line.c) <= gate
            count = int(np.count_nonzero(mask))
            if count < self.min_inliers:
                return None, count, math.nan
            nx, ny, c = _tls_line(points[mask])
            # Keep the stored orientation: the refit's sign is arbitrary.
            if nx * line.nx + ny * line.ny < 0.0:
                nx, ny, c = -nx, -ny, -c
            line = Line(nx, ny, c)
        inliers = points[np.abs(points @ line.normal - line.c) <= gates[-1]]
        s_min, s_max = _span(inliers, line)
        span = s_max - s_min
        if span < self.min_span_m:
            return None, count, span
        return line, count, span

    def update(self, points: Optional[np.ndarray], car_pose, now: float) -> TrackStep:
        """One tick: refit from `points` (None when no fresh scan), or
        dead-reckon. d_wall is always from `car_pose` against the stored
        line, which the refit may just have refreshed."""
        now = float(now)
        if self.line is None:
            return TrackStep(False, PROVENANCE_NONE, math.nan, 'no_wall', 0, math.nan,
                             math.nan)
        if points is None or len(points) == 0:
            reason = 'no_scan'
            count, span = 0, math.nan
        else:
            line, count, span = self._refit(points)
            if line is not None:
                self.line = line
                self.last_fit_time = now
                self.last_inlier_count = count
                self.last_span_m = span
                reason = 'measured'
            else:
                reason = 'few_inliers' if count < self.min_inliers else 'short_span'
        self.provenance = (PROVENANCE_MEASURED if reason == 'measured'
                           else PROVENANCE_DEAD_RECKONED)
        return TrackStep(True, self.provenance, self.d_wall(car_pose), reason, count, span,
                         now - self.last_fit_time)
