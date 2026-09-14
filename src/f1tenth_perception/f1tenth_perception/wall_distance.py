"""Tracked-wall distance, the wall_turn phase machine, and the corridor
correction -- pure logic, no rclpy, no I/O.

wall_distance_node.py is the ROS glue and every number it publishes is decided
here, so it can be unit-tested and replayed against archived scans without a
running graph (the same split glass_detect.py / a node, lidar_front_wall.py /
lidar_front_wall_node.py and f1tenth_costmap's costmap_boundary.py already
use).

Read docs/wall_turn_investigation.md first. It records why the correction goes
where it goes, and the two findings that shape everything below.


WHAT IS ACTUALLY CONTROLLABLE, AND WHAT IS NOT
==============================================
A wall_turn move ends when the BEHAVIOUR TREE sees 90 degrees of accumulated
yaw (f1tenth_behavior condition_eval.py's orientation_delta), not when the MPC
decides anything. So the turn's EXIT HEADING is start_heading + 90 deg and no
edit inside mpc_corr can change it. What an edit can change is the PATH taken
to accumulate those 90 degrees, and therefore WHERE the car ends up.

That is why this module computes a lateral correction and hands it to the
STRAIGHT corridor the car drives after the turn (Option B in the
investigation), and why injecting at the turn's own dpsi_this was rejected:
dpsi_this is bounded by min(|dpsi_rem|, ...) in wall_turn.py, so its authority
goes to zero exactly as the turn completes, which is precisely where the
correction is wanted -- and the ratchet suppresses the one sign that would
help.


THE CONTROL LAW, AND ITS SIGN
=============================
Lateral dynamics for a car following a wall are

    edot = v * sin(dpsi) ~= v * dpsi

where e is the car's own signed lateral offset from the line it should be on,
in the SAME sense as dpsi (left positive). Setting dpsi = -k * e gives
edot = -v * k * e: first order, unconditionally stable, no oscillation, and
the closed-loop behaviour in SPACE is independent of speed. Convergence length
is L = 1/k, which is why convergence_length_m is the parameter a human touches
and k is derived from it. DO NOT TUNE k EMPIRICALLY -- pick the distance over
which the error should wash out and invert it.

NOW THE SIGN, AND THIS IS THE ONE THING IN THE FILE THAT WILL DRIVE A CAR INTO
GLASS IF IT IS WRONG. d_wall is defined (WallDistance.msg) as the WALL's
signed offset from the CAR, left positive. That is the opposite sense to e:
steering left DECREASES d_wall (the car closes on a left-hand wall) while it
INCREASES the car's own leftward offset. So

    d_ref_signed = copysign(d_ref, d_wall)       # d_ref is a magnitude
    e            = d_ref_signed - d_wall          # car's offset, left positive
    dpsi         = -k * e = k * (d_wall - d_ref_signed)

Worked both ways, because one worked example per side is the only thing that
catches an inversion:

  * Wall on the LEFT at 0.40 m, d_ref 0.60. d_wall = +0.40, d_ref_signed =
    +0.60, e = +0.20 (the car is 0.20 m further left than it should be),
    dpsi = -0.20k < 0 = RIGHT = away from the wall. Correct.
  * Wall on the RIGHT at 0.40 m. d_wall = -0.40, d_ref_signed = -0.60,
    e = -0.20, dpsi = +0.20k > 0 = LEFT = away from the wall. Correct.

A NOTE FOR ANYONE COMPARING THIS TO THE DESIGN PROMPT: the prompt writes
"e = d_wall - d_ref" and "dpsi = -k*e". Those two together are stable only if
d_wall is read as the CAR's offset from the wall. With d_wall as the WALL's
offset from the car -- which is what the stack's left-positive convention gives
and what the message documents -- the same pair is unstable, and the fix is the
one sign flip above. test_wall_distance.TestSignOnBothSides pins it on both
sides. Only Stage 2 of docs/bringup_checklist.md can pin the convention itself
against physical left and right.


STALENESS PUSHES THE TWO OUTPUTS IN OPPOSITE DIRECTIONS
=======================================================
This is spelled out because the next reader will assume the conservative
direction is the same for both. It is not.

  * GATE / clearance: stale -> INFLATE the required margin. A wall IS there;
    confidence in WHERE is decaying; back off. gate_margin().
  * SOFT / centering: stale -> FADE the correction toward zero. A stale d_wall
    commands a confident heading toward a position no longer trusted, and
    neutral is safer than wrong. soft_fade().

Same input, opposite direction, and test_wall_distance.TestStaleAsymmetry
asserts they never move the same way.


COASTING IS BOUNDED BY ODOMETRY QUALITY, NOT BY TASTE
=====================================================
The tracked wall is carried through the turn even when no beams land on it,
because for glass that is the NORMAL state -- at oblique incidence the pane is
invisible and absence is not evidence the wall moved. But the carry is dead
reckoning, and this car's dead reckoning is measured wrong:

  * distance under-reports ~17-20% (/odom and the local EKF, since the
    speed_to_erpm_gain change of 2026-08-10), and
  * gyro yaw gain is 0.93-1.09.

So a 90 degree turn can misorient the stored wall by ~6 degrees and a 2 m coast
can misplace it laterally by ~0.4 m -- both larger than any margin in this
design. Hence the hard caps in WallDistanceTracker: past max_coast_distance of
odometry TRAVEL (path length, not displacement -- an arc is longer than its
chord and the bias is already optimistic) or max_coast_yaw of rotation,
provenance goes NONE and valid goes false, with one WARN per coast.

RAISE THE CAPS ONLY AFTER THE VESC SPEED SCALE AND THE GYRO GAIN ARE FIXED.
They are not conservative guesses; they are the measured bias divided into an
acceptable error. Their defaults live in stack_params.yaml with the same note.
"""

import math
from dataclasses import dataclass
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np

# Mirrors f1tenth_messages/WallDistance.msg's own constants.
# test_wall_distance.TestConstantsMirrorTheMessage asserts they stay equal --
# wall_distance_node casts one straight onto the other.
PROVENANCE_NONE = 0
PROVENANCE_GEOMETRIC = 1
PROVENANCE_GLASS_CONFIRMED = 2
PROVENANCE_PREDICTED = 3
PROVENANCE_NAMES = {
    PROVENANCE_NONE: 'NONE',
    PROVENANCE_GEOMETRIC: 'GEOMETRIC',
    PROVENANCE_GLASS_CONFIRMED: 'GLASS_CONFIRMED',
    PROVENANCE_PREDICTED: 'PREDICTED',
}

PHASE_UNKNOWN = 0
PHASE_WALL_TURN = 1
PHASE_COMMITTED = 2
PHASE_CORRIDOR = 3
PHASE_NAMES = {
    PHASE_UNKNOWN: 'UNKNOWN',
    PHASE_WALL_TURN: 'WALL_TURN',
    PHASE_COMMITTED: 'COMMITTED',
    PHASE_CORRIDOR: 'CORRIDOR',
}


def wrap_to_pi(angle: float) -> float:
    """Map `angle` into (-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


def fold_to_half_pi(angle: float) -> float:
    """Map `angle` into [-pi/2, pi/2].

    A line fit gives its direction only up to a flip, so "heading minus wall
    direction" is only ever meaningful modulo pi. Folding here rather than at
    each call site is what keeps a wall the car is driving along from reading
    as 180 degrees off half the time.
    """
    a = wrap_to_pi(angle)
    if a > math.pi / 2:
        return a - math.pi
    if a < -math.pi / 2:
        return a + math.pi
    return a


# ===========================================================================
# Geometry
# ===========================================================================

def signed_wall_offset(nx: float, ny: float, c: float,
                       car_pose: Tuple[float, float, float]) -> float:
    """The wall's signed perpendicular offset from the car, LEFT POSITIVE.

    `n . p = c` is the wall in the same frame as `car_pose` (odom). Returns a
    value whose MAGNITUDE is the perpendicular distance from the car's origin
    to the line and whose SIGN is + when the wall lies to the car's left.

    TWO THINGS THIS GETS RIGHT ON PURPOSE.

    1. It is invariant to the fit's arbitrary normal flip. `tls_line` and
       `ransac_line` return `n` up to a sign, so (nx, ny, c) and
       (-nx, -ny, -c) are the same wall; both factors below negate together,
       so the answer does not move. glass_detect does NO orientation
       normalisation of its own -- unlike wall_tracker._gate, which reorients
       n from the car toward the line before anything reads it. Do not assume
       either.

    2. The magnitude is the PERPENDICULAR distance, not its projection onto
       the car's lateral axis. For an oblique wall those differ by
       cos(obliquity), and the obliquity is reported separately as
       heading_rel so the consumer can use it or gate on it. Shrinking the
       magnitude here would silently fold two different quantities together.

    DEGENERATE CASE: a wall exactly dead ahead (its normal along the heading)
    is neither left nor right, and the sign is then whatever copysign makes of
    a zero. That is deterministic but meaningless, and it is why heading_rel
    is published: at |heading_rel| near pi/2 the wall is in front, not beside,
    and a centering correction against it is not defined. The correction only
    ever runs in phase CORRIDOR, by which point a wall_turn's front wall is
    abeam.
    """
    x, y, psi = car_pose
    # Signed perpendicular displacement from the car to the line, along +n.
    # NOTE the sense: this matches mpc_controller/wall_tracker.py's
    # Line.offset() ("positive before it") and is the NEGATION of
    # f1tenth_perception/glass_detect.py's Line.signed(). The two Line classes
    # in this workspace use opposite conventions and mixing them is one sign
    # flip away from steering into the pane -- see
    # docs/wall_turn_investigation.md.
    along_n = c - (nx * x + ny * y)
    # The car's left unit normal, dotted with the wall's normal: its sign says
    # which side of the car the wall's perpendicular foot falls on.
    n_dot_left = -nx * math.sin(psi) + ny * math.cos(psi)
    return along_n * math.copysign(1.0, n_dot_left)


def wall_heading_rel(nx: float, ny: float, psi: float) -> float:
    """Vehicle heading minus the wall's direction, folded into [-pi/2, pi/2].

    0 means driving parallel to the wall. +-pi/2 means driving straight at it.
    """
    # The line's tangent is (-ny, nx); its bearing is atan2(nx, -ny).
    return fold_to_half_pi(psi - math.atan2(nx, -ny))


# ===========================================================================
# Candidate sources
# ===========================================================================

# Widest gap [m] along a fitted line that is still one surface rather than two.
# Same value and same reason as glass_detect's own _MULLION_MAX_GAP_M: a window
# mullion or a doorframe interrupts a wall without ending it.
MULLION_MAX_GAP_M = 0.6

# Rejection reasons are glass_detect's own REASON_* constants, not a parallel
# set: three of the four gates here are the same gates with the same meaning,
# and a second spelling of "too_few_inliers" is how two modules start
# disagreeing about what a number means.


def geometric_candidates(ranges, angle_min: float, angle_increment: float,
                         range_min: float, range_max: float, laser_pose, car_pose, *,
                         min_range_m: float, max_range_m: float,
                         inlier_distance_m: float, min_inliers: int,
                         min_span_m: float, min_distance_m: float,
                         max_candidates: int):
    """Plain line fits over one scan's returns, as glass_detect Candidates.

    WHY THIS EXISTS, AND IT IS NOT AN OPTIMISATION. glass_detect.detect() is not
    a wall fitter and was never meant to be: it returns a surface only when that
    surface carries TRANSPARENCY evidence -- a spike, returns on the line inside
    an opening, or a see-through band -- because its job is the marking-only
    glass observation a costmap must never clear. It begins with
    `if not voids: return []`. An ordinary opaque wall produces no voids and
    therefore no candidate at all.

    MEASURED ON THE ARCHIVE, because this was found the hard way. Over 40 real
    scans from 2026-09-10T13-34-09_mission-wall_turn (an office corridor, opaque
    walls) glass_detect found 690 voids and 160 candidates but ACCEPTED ONLY 4 --
    roughly one per ten scans, against a confirmation bar of persistence_hits *
    geometry_only_multiplier = 6 hits inside a window of 8 scans. It can never
    confirm at that rate, so wall_distance_node tracked nothing for the whole
    run. A plain MSAC over the same first scan found a 5.29 m wall with 220
    inliers at 2.48 m immediately.

    So the node runs BOTH sources every scan, and that is what
    WallDistance.msg's provenance field has always distinguished:
    GLASS_CONFIRMED for a track whose evidence says "glass", GEOMETRIC for one
    fitted from returns with no such evidence. Glass-confirmed candidates are
    strictly better when they exist -- they survive incidence angles a geometric
    fit never sees -- but they are the exception on this car in this building,
    not the rule.

    THE GATES, in the order a rejection reports them:
      inliers >= min_inliers          a short cluster fits any line you like
      span    >= min_span_m           measured along the line, mullion-tolerant
      perp    >= min_distance_m       a line passing through the car's own
                                      footprint is the car, or near-field
                                      clutter, not a wall. Note this CANNOT be
                                      done by filtering returns by range: an
                                      oblique line through points all beyond
                                      min_range_m can still pass 0.62 m from the
                                      origin, which is exactly what the archive
                                      produced.

    Returns glass_detect.Candidate objects with evidence=0 (hence GEOMETRIC) so
    GlassTracker.update() consumes them through the same association and
    persistence path as glass ones, with no second code path to keep in step.
    """
    from f1tenth_perception.glass_detect import (
        REASON_ACCEPTED, REASON_TOO_FEW_INLIERS, REASON_TOO_NEAR, REASON_TOO_SHORT,
        Candidate, contiguous_extent, no_return_mask, ransac_line, scan_points)

    ranges = np.asarray(ranges, dtype=np.float64)
    n = ranges.size
    angles = angle_min + np.arange(n) * angle_increment
    usable = (~no_return_mask(ranges, range_min, range_max)
              & (ranges >= float(min_range_m)) & (ranges <= float(max_range_m)))
    if np.count_nonzero(usable) < max(int(min_inliers), 2):
        return []
    points = scan_points(ranges, angles, laser_pose, car_pose)[usable]

    car = np.array([float(car_pose[0]), float(car_pose[1])])
    out = []
    remaining = np.ones(len(points), dtype=bool)
    for _ in range(max(int(max_candidates), 0)):
        if np.count_nonzero(remaining) < 2:
            break
        subset = points[remaining]
        fit = ransac_line(subset, float(inlier_distance_m))
        if fit is None:
            break
        line, sub_mask = fit
        chosen = np.flatnonzero(remaining)[sub_mask]
        remaining[chosen] = False
        inliers = points[chosen]
        count = int(len(inliers))
        proj = line.project(inliers)
        lo, hi = contiguous_extent(proj, MULLION_MAX_GAP_M)
        span = float(hi - lo)
        rms = float(np.sqrt(np.mean((inliers @ line.normal - line.c) ** 2)))
        # Endpoints put back ON the line rather than at the outermost inliers,
        # so the published segment is the fitted geometry and not a noisy
        # sample. Identical construction to glass_detect._verify, so a
        # geometric and a glass candidate for the same surface land in the same
        # place and re-associate to each other.
        base = line.normal * line.c
        p0, p1 = base + lo * line.tangent, base + hi * line.tangent
        perp = abs(float(line.c - line.normal @ car))

        if count < int(min_inliers):
            reason = REASON_TOO_FEW_INLIERS
        elif span < float(min_span_m):
            reason = REASON_TOO_SHORT
        elif perp < float(min_distance_m):
            reason = REASON_TOO_NEAR
        else:
            reason = REASON_ACCEPTED
        out.append(Candidate(
            line=line, p0=p0, p1=p1, length_m=span, inlier_count=count,
            n_voids=0, evidence=0, reason=reason, rms_m=rms,
            spike_angle_err_rad=math.nan))
    return out


def to_base_link(point, car_pose) -> Tuple[float, float]:
    """An odom-frame point in base_link (x forward, y left)."""
    x, y, psi = car_pose
    dx, dy = float(point[0]) - x, float(point[1]) - y
    c, s = math.cos(psi), math.sin(psi)
    return (c * dx + s * dy, -s * dx + c * dy)


# ===========================================================================
# Odometry travel
# ===========================================================================

class Odometer:
    """Monotone path length and absolute yaw travel from a pose stream.

    THE COAST CAPS ARE MEASURED AGAINST PATH LENGTH, NOT DISPLACEMENT, and the
    difference is not academic: during a 90 degree turn at R_min 1.07 m the arc
    is 1.68 m while the chord is 1.51 m, an 11% under-read in the optimistic
    direction -- stacked on top of odometry that is already 17-20% optimistic.
    A displacement-based cap would let the wall drift further than the cap
    claims to allow.

    yaw_travel is the integral of |dyaw|, so a there-and-back wobble counts
    twice rather than cancelling. That is deliberate: what degrades a stored
    wall's orientation is total rotation through the gyro's gain error, not net
    rotation.
    """

    def __init__(self) -> None:
        self.s = 0.0
        self.yaw_travel = 0.0
        self._last: Optional[Tuple[float, float, float]] = None

    def update(self, pose: Tuple[float, float, float]) -> None:
        x, y, psi = float(pose[0]), float(pose[1]), float(pose[2])
        if self._last is not None:
            px, py, ppsi = self._last
            self.s += math.hypot(x - px, y - py)
            # wrap_to_pi, so a pose stream crossing +-pi does not book a full
            # turn of travel it never made.
            self.yaw_travel += abs(wrap_to_pi(psi - ppsi))
        self._last = (x, y, psi)


# ===========================================================================
# Phase, from /mpc/wall_track alone
# ===========================================================================

@dataclass(frozen=True)
class Transition:
    """One phase change, with the reason, for the log line."""

    old: int
    new: int
    reason: str

    def __str__(self) -> str:
        return f'{PHASE_NAMES[self.old]} -> {PHASE_NAMES[self.new]}: {self.reason}'


class PhaseMachine:
    """Phase of an active wall_turn, inferred from /mpc/wall_track alone.

    SILENCE IS OVERLOADED AND MUST NEVER MAP TO CORRIDOR. Silence on
    /mpc/wall_track means any of: no wall_turn active, /mpc/hold engaged
    (MPC_corr.control_loop returns before _wall_track_tick), no odometry yet,
    wall_track_enable false (no publisher exists at all), or a dead node. And a
    TERMINAL wall_turn -- which is every wall_turn mission in the repo today --
    has no exit edge whatsoever: drive_cmd is never cleared by a hold, so the
    topic simply goes quiet. Exit is observable as silence, not as a final
    message.

    So entering corridor-following requires a POSITIVE signal: a clean
    COMMITTED -> silence transition with a valid track held throughout. Every
    other route to silence goes to UNKNOWN, which produces no correction and no
    gating.

    STARTUP IS ALSO UNKNOWN, and for a structural reason rather than caution:
    /mpc/wall_track is published with volatile durability, so a subscriber that
    starts between two turns sees nothing at all until the next one begins.
    There is no latched state to recover.

    Driven once per publish tick, not once per message: `tick` takes the
    psi_commit of the most recent message received SINCE THE LAST TICK, or None
    if none arrived. That is what makes wall_track_silence_ticks mean
    "consecutive ticks with no message" rather than something wall-clock and
    jittery.
    """

    def __init__(self, *, silence_ticks: int) -> None:
        self.silence_ticks = max(int(silence_ticks), 1)
        self.phase = PHASE_UNKNOWN
        self.silent_ticks = 0
        # Whether a valid track has been held for every tick since this turn
        # entered COMMITTED. "Throughout" means throughout: a track that
        # dropped out mid-commit may have re-associated to a different surface,
        # so it does not earn CORRIDOR.
        self.track_held_throughout = False

    def tick(self, psi_commit: Optional[float], *,
             track_valid: bool) -> Optional[Transition]:
        """Advance one publish tick. Returns a Transition or None.

        psi_commit: the value from the most recent /mpc/wall_track message
        received since the last tick (NaN before the turn commits, finite
        after), or None if no message arrived.
        """
        if psi_commit is not None:
            return self._on_message(float(psi_commit), track_valid)
        return self._on_silence(track_valid)

    def _on_message(self, psi_commit: float, track_valid: bool) -> Optional[Transition]:
        self.silent_ticks = 0
        committed = math.isfinite(psi_commit)
        new = PHASE_COMMITTED if committed else PHASE_WALL_TURN
        old = self.phase
        if new == PHASE_COMMITTED:
            if old == PHASE_COMMITTED:
                self.track_held_throughout = self.track_held_throughout and track_valid
            else:
                self.track_held_throughout = track_valid
        else:
            # An uncommitted turn has no corridor to earn yet.
            self.track_held_throughout = False
        if new == old:
            return None
        self.phase = new
        reason = ('/mpc/wall_track with psi_commit finite: the turn has committed'
                  if committed else
                  '/mpc/wall_track with psi_commit NaN: wall_turn issued, not yet bending')
        return Transition(old, new, reason)

    def _on_silence(self, track_valid: bool) -> Optional[Transition]:
        self.silent_ticks += 1
        old = self.phase
        if old not in (PHASE_WALL_TURN, PHASE_COMMITTED):
            # UNKNOWN stays UNKNOWN; CORRIDOR stays CORRIDOR, because silence
            # is what CORRIDOR means -- it is not a degraded state.
            return None
        # The track must be held through the silence window too.
        self.track_held_throughout = self.track_held_throughout and track_valid
        if self.silent_ticks < self.silence_ticks:
            # One dropped message on a reliable topic is not a turn ending.
            return None
        if old == PHASE_COMMITTED and self.track_held_throughout and track_valid:
            self.phase = PHASE_CORRIDOR
            return Transition(
                old, PHASE_CORRIDOR,
                f'{self.silent_ticks} silent ticks after COMMITTED with a valid track '
                'held throughout: the turn exited and the car is now in the corridor')
        if old == PHASE_WALL_TURN:
            why = ('the turn never committed, so there is no corridor to follow '
                   '(hold, lost goal, or wall_track_enable false)')
        elif not track_valid:
            why = 'no valid track at exit, so there is nothing to be proportional to'
        else:
            why = 'the track did not hold throughout the commit'
        self.phase = PHASE_UNKNOWN
        self.track_held_throughout = False
        return Transition(old, PHASE_UNKNOWN,
                          f'{self.silent_ticks} silent ticks: {why}')


# ===========================================================================
# Tracking, with a stable identity and the coast caps
# ===========================================================================

@dataclass
class _Record:
    """Per-track bookkeeping the glass tracker does not keep."""

    track_id: int
    obs_time: float
    obs_s: float
    obs_yaw: float
    fit_rms: float
    inlier_count: int
    glass_confirmed: bool
    # WHICH update() LAST SAW THIS TRACK DIRECTLY, as a scan sequence number
    # rather than a timestamp. GEOMETRIC/GLASS_CONFIRMED vs PREDICTED is the
    # question "did returns land on this track in the latest scan", and a
    # timestamp cannot answer it: scans arrive at 40 Hz and the node publishes
    # at 10 Hz, so the newest scan is always a few milliseconds old at tick
    # time and an `age <= 0` test reports PREDICTED for every tick of a
    # perfectly healthy track. Measured on the archive before this was a
    # counter: 69 of 71 ticks came out PREDICTED with the wall in plain view.
    obs_seq: int = -1
    cap_warned: bool = False


@dataclass
class Observation:
    """One tick's answer. Everything wall_distance_node puts on the wire."""

    valid: bool
    track_id: int
    d_wall: float
    heading_rel: float
    fit_rms: float
    inlier_count: int
    age: float
    provenance: int
    p0: Optional[np.ndarray] = None      # base_link
    p1: Optional[np.ndarray] = None      # base_link
    coast_distance: float = 0.0
    coast_yaw: float = 0.0
    # True on the single tick a coast first exceeds a cap, so the node can WARN
    # once per coast instead of ten times a second.
    coast_cap_first_hit: bool = False

    @staticmethod
    def none(reason_provenance: int = PROVENANCE_NONE) -> 'Observation':
        return Observation(
            valid=False, track_id=0, d_wall=math.nan, heading_rel=math.nan,
            fit_rms=math.nan, inlier_count=0, age=math.nan,
            provenance=reason_provenance)


class WallDistanceTracker:
    """glass_detect.GlassTracker plus a stable published identity and the
    odometry-bounded coast caps.

    WHAT IS REUSED AND WHY. GlassTracker already maintains odom-frame tracks,
    re-associates by endpoint proximity AND line angle, and -- the part that
    matters -- never decays a track on absence alone, because absence is the
    normal state for glass. Rewriting that would reintroduce the clearing bug
    the whole design exists to prevent. What it does not have is a stable
    identity (its tracks are distinguished by Python object identity) or any
    notion of how far a track has been coasted, and those are what this adds.

    WHAT IS NOT REUSED. GlassTracker.drop_stale() forgets a track after a
    timeout and defaults to never forgetting, which is right for glass. The
    coast caps here are a DIFFERENT mechanism: they stop TRUSTING a track while
    keeping it, so that re-acquiring the same pane keeps its track_id instead
    of looking like a new wall. Do not collapse the two.

    SELECTION IS STICKY. A track_id change is published, not hidden, so the
    consumer can reject the resulting step in d_wall instead of following it --
    but it should only happen when the wall genuinely changes. So the currently
    reported track is kept for as long as it is still confirmed, and only then
    does the nearest confirmed track take over.
    """

    def __init__(self, *, glass_tracker, max_coast_distance: float,
                 max_coast_yaw: float, track_timeout_sec: float = 0.0) -> None:
        self.glass = glass_tracker
        self.max_coast_distance = float(max_coast_distance)
        self.max_coast_yaw = float(max_coast_yaw)
        self.track_timeout_sec = float(track_timeout_sec)
        self.odometer = Odometer()
        self._records: Dict[int, _Record] = {}
        self._next_id = 1
        self._selected: Optional[int] = None
        # Incremented once per update(), i.e. once per scan. See _Record.obs_seq.
        self._seq = 0

    # -- internals ---------------------------------------------------------

    def _record_for(self, track, now: float) -> _Record:
        key = id(track)
        rec = self._records.get(key)
        if rec is None:
            rec = _Record(track_id=self._next_id, obs_time=now,
                          obs_s=self.odometer.s, obs_yaw=self.odometer.yaw_travel,
                          fit_rms=math.nan, inlier_count=0, glass_confirmed=False,
                          obs_seq=self._seq)
            self._next_id += 1
            self._records[key] = rec
        return rec

    def _live(self) -> Dict[int, object]:
        return {id(t): t for t in self.glass.tracks}

    # -- the tick ----------------------------------------------------------

    def update(self, candidates: Sequence, car_pose, now: float,
               use_intensity: bool) -> List[object]:
        """Fold one scan's candidates in. Returns the confirmed track list.

        Call Odometer.update() with the pose BEFORE this, every time a pose
        arrives -- the coast caps are measured against it and it must not be
        sampled only when a scan happens to land.
        """
        self._seq += 1
        confirmed = self.glass.update(candidates, car_pose, now, use_intensity)

        # Which tracks were DIRECTLY observed on this scan, and by which
        # candidate. GlassTracker.update assigns `track.line = cand.line` for
        # the track it matched, so the line object identity is an exact
        # mapping -- no need to re-run the association and risk disagreeing
        # with it.
        by_line = {id(c.line): c for c in candidates if c.accepted}
        for track in self.glass.tracks:
            rec = self._record_for(track, now)
            cand = by_line.get(id(track.line))
            if cand is None or track.last_seen != now:
                continue
            rec.obs_time = now
            rec.obs_seq = self._seq
            rec.obs_s = self.odometer.s
            rec.obs_yaw = self.odometer.yaw_travel
            rec.fit_rms = float(cand.rms_m)
            rec.inlier_count = int(cand.inlier_count)
            # EVIDENCE_TRANSPARENCY is the OR of the three evidence kinds that
            # actually argue "glass" rather than "a gap" -- see glass_detect.
            rec.glass_confirmed = bool(cand.evidence & _transparency_mask())
            # A fresh observation ends the coast, and re-arms the one-shot WARN.
            rec.cap_warned = False

        if self.track_timeout_sec > 0.0:
            self.glass.drop_stale(now, self.track_timeout_sec)
        live = self._live()
        for key in [k for k in self._records if k not in live]:
            del self._records[key]
        if self._selected is not None and self._selected not in live:
            self._selected = None

        # SELECTION IS STICKY ON LIVENESS, NOT ON CONFIRMATION, and the
        # difference is the whole point of tracking rather than re-fitting.
        # GlassTracker.confirmed() is a PUBLICATION GATE FOR NEW DETECTIONS --
        # "has this candidate been seen often enough to believe it exists" --
        # not a liveness test for a wall already acquired. Re-selecting
        # whenever the selected track fell out of the confirmation window would
        # drop the wall after persistence_window unseen scans, which is exactly
        # the clearing behaviour glass tracking exists to prevent: through a
        # turn the pane sweeps into incidence angles where it returns nothing,
        # and absence there is the normal state.
        #
        # So the selected track is kept while its object is alive. It is given
        # up only when the glass tracker has forgotten it outright, or when its
        # own coast cap has fired -- at which point a confirmed track with
        # fresher evidence is strictly better than one we have stopped
        # trusting. A change of track_id is then PUBLISHED, not hidden, so the
        # consumer can reject the step in d_wall instead of following it.
        if self._selected is None or self._selected not in live:
            self._selected = self._nearest(confirmed, car_pose)
        elif (self._over_cap(self._records[self._selected])
                and self._records[self._selected].cap_warned):
            # ONE TICK OF "NOT TRUSTED" BEFORE THE HANDOVER, and the ordering
            # is the point. cap_warned is set by observation() on the first
            # over-cap tick, so this branch cannot fire until that tick has
            # been published: the consumer sees valid=false with
            # coast_cap_first_hit, and only then does a fresher confirmed track
            # take over with a new track_id.
            #
            # Swapping immediately -- which is what this did first -- means
            # observation() never sees the over-cap record, so the WARN never
            # fires and the operator gets an unexplained track_id change
            # instead of "the coast cap fired". Measured on
            # 2026-09-10T15-33-46_mission-wall_turn: 3 track_ids over one 90
            # degree turn and ZERO cap warnings.
            replacement = self._nearest(confirmed, car_pose)
            if replacement is not None and replacement != self._selected:
                self._selected = replacement
        return confirmed

    def _over_cap(self, rec: '_Record') -> bool:
        return (self.odometer.s - rec.obs_s > self.max_coast_distance
                or self.odometer.yaw_travel - rec.obs_yaw > self.max_coast_yaw)

    def _nearest(self, confirmed: Sequence, car_pose) -> Optional[int]:
        best, best_d = None, math.inf
        for track in confirmed:
            d = abs(signed_wall_offset(track.line.nx, track.line.ny,
                                       track.line.c, car_pose))
            if d < best_d:
                best, best_d = id(track), d
        return best

    def observation(self, car_pose, now: float) -> Observation:
        """The current tick's tracked-wall geometry, or an invalid Observation.

        valid=false means "no usable track" and is NOT the same as d_wall==0.
        """
        if self._selected is None:
            return Observation.none()
        track = self._live().get(self._selected)
        if track is None:
            self._selected = None
            return Observation.none()
        rec = self._records[self._selected]

        coast_s = max(self.odometer.s - rec.obs_s, 0.0)
        coast_yaw = max(self.odometer.yaw_travel - rec.obs_yaw, 0.0)
        age = max(float(now) - rec.obs_time, 0.0)

        # THE COAST CAPS. Named here, with the reason, so they can be raised
        # after the odometry calibration lands and not before: /odom and the
        # local EKF under-report distance ~17-20% (speed_to_erpm_gain,
        # 2026-08-10) and the gyro yaw gain is 0.93-1.09, so beyond these the
        # stored line is misplaced by more than any margin in this design. See
        # stack_params.yaml's wall_distance_max_coast_{distance,yaw}.
        if self._over_cap(rec):
            first = not rec.cap_warned
            rec.cap_warned = True
            return Observation(
                valid=False, track_id=rec.track_id, d_wall=math.nan,
                heading_rel=math.nan, fit_rms=rec.fit_rms,
                inlier_count=rec.inlier_count, age=age,
                provenance=PROVENANCE_NONE,
                coast_distance=coast_s, coast_yaw=coast_yaw,
                coast_cap_first_hit=first)

        line = track.line
        if rec.obs_seq == self._seq:
            provenance = (PROVENANCE_GLASS_CONFIRMED if rec.glass_confirmed
                          else PROVENANCE_GEOMETRIC)
        else:
            # No returns landed on this track in the latest scan. For glass
            # that is the NORMAL state, not evidence the wall moved -- at
            # oblique incidence the pane is invisible.
            provenance = PROVENANCE_PREDICTED
        return Observation(
            valid=True,
            track_id=rec.track_id,
            d_wall=signed_wall_offset(line.nx, line.ny, line.c, car_pose),
            heading_rel=wall_heading_rel(line.nx, line.ny, car_pose[2]),
            fit_rms=rec.fit_rms,
            inlier_count=rec.inlier_count,
            age=age,
            provenance=provenance,
            p0=np.asarray(to_base_link(track.p0, car_pose), dtype=float),
            p1=np.asarray(to_base_link(track.p1, car_pose), dtype=float),
            coast_distance=coast_s,
            coast_yaw=coast_yaw,
        )


def _transparency_mask() -> int:
    # Imported lazily so this module stays importable if glass_detect's
    # constant set changes name, and so the dependency is visible at the one
    # place it is used rather than at the top of the file.
    from f1tenth_perception.glass_detect import EVIDENCE_TRANSPARENCY
    return int(EVIDENCE_TRANSPARENCY)


# ===========================================================================
# Staleness -- the two directions
# ===========================================================================

def soft_fade(age: float, fade_start_age: float, fade_zero_age: float) -> float:
    """Multiplier in [0, 1] for the SOFT output (the centering correction).

    1.0 until fade_start_age, then linear to 0.0 at fade_zero_age. A stale
    d_wall commands a confident heading toward a position no longer trusted;
    neutral is safer than wrong.

    THE OPPOSITE DIRECTION FROM gate_margin(), deliberately. See the module
    docstring.
    """
    age = float(age)
    if not math.isfinite(age) or age <= float(fade_start_age):
        return 1.0
    span = float(fade_zero_age) - float(fade_start_age)
    if span <= 0.0:
        return 0.0
    return max(0.0, min(1.0, 1.0 - (age - float(fade_start_age)) / span))


def gate_margin(age: float, stale_inflate_per_s: float) -> float:
    """Extra required margin [m] for the GATE output, growing with age.

    A wall IS there and confidence in WHERE is decaying, so a gate must back
    off. THE OPPOSITE DIRECTION FROM soft_fade(). Returns metres to ADD to
    whatever margin the consumer already requires; never negative.
    """
    age = float(age)
    if not math.isfinite(age) or age <= 0.0:
        return 0.0
    return max(0.0, age * float(stale_inflate_per_s))


# ===========================================================================
# The correction
# ===========================================================================

@dataclass(frozen=True)
class CorrectionConfig:
    """Everything the control law needs, from stack_params.yaml."""

    d_ref: float
    convergence_length_m: float
    max_psi_correction: float
    max_psi_rate: float
    deadband_floor: float
    deadband_k: float
    fade_start_age: float
    fade_zero_age: float
    stale_inflate_per_s: float

    @property
    def k(self) -> float:
        """Proportional gain [rad/m], DERIVED from the convergence length.

        L = 1/k is the distance over which a lateral error washes out, so the
        parameter a human touches is the one with physical meaning. Tuning k
        directly is how a first-order law with a speed-independent spatial
        response turns into a number nobody can explain.
        """
        return 1.0 / max(float(self.convergence_length_m), 1e-6)


def deadband_for(fit_rms: float, deadband_floor: float, deadband_k: float) -> float:
    """Deadband [m], DERIVED FROM THE FIT QUALITY rather than constant.

    deadband = max(deadband_floor, deadband_k * fit_rms). The 17-19 mm rms
    measured at 1.1 m will grow substantially at longer range with fewer
    inliers, and a fixed deadband would steer on noise exactly when the
    estimate is worst. A non-finite rms (no fit yet) falls back to the floor.
    """
    rms = float(fit_rms)
    scaled = 0.0 if not math.isfinite(rms) else abs(rms) * float(deadband_k)
    return max(float(deadband_floor), scaled)


def psi_correction(d_wall: float, *, cfg: CorrectionConfig, fit_rms: float,
                   age: float, prev_psi: float, dt: float, valid: bool) -> float:
    """The heading correction [rad] for the corridor the car is in.

    Left positive, same sense as psi. Applied by mpc_corr to the STRAIGHT
    drive corridor's psi_base (Option B in docs/wall_turn_investigation.md).

    Order of operations, and it matters:
      deadband -> gain -> saturate -> fade -> rate-limit.
    The rate limit is last precisely so it absorbs everything upstream of it,
    including the handoff discontinuity at a phase transition and the step in
    d_wall when a track re-associates.

    NO TRACKED WALL MEANS ZERO, NEVER THE LAST KNOWN VALUE. The natural
    implementation holds the last value and that is the bug: at turn exit with
    no wall in hand, a held correction is a confident heading toward a position
    nobody can see. Zero is the only defensible fallback, and it is returned
    WITHOUT passing through the rate limit -- the rate limit exists to smooth
    corrections, not to ration their withdrawal. Snapping to zero returns the
    corridor to exactly the geometry it had before this node existed, which is
    the safe direction by construction.
    """
    if not valid or not math.isfinite(float(d_wall)):
        return 0.0

    d_wall = float(d_wall)
    # d_ref is a MAGNITUDE and cannot come from corridor width: the corridor is
    # synthetic (corr_wmin/corr_wmax are bare literals in MPC_corr.py, not
    # parameters) and only one wall is ever sensed. Signed here by whichever
    # side the wall is actually on.
    d_ref_signed = math.copysign(abs(float(cfg.d_ref)), d_wall)

    # e is the CAR's signed lateral offset from the reference line, left
    # positive -- the OPPOSITE sense to d_wall. Read the module docstring's
    # sign section before touching this line.
    e = d_ref_signed - d_wall

    band = deadband_for(fit_rms, cfg.deadband_floor, cfg.deadband_k)
    magnitude = max(abs(e) - band, 0.0)
    if magnitude <= 0.0:
        target = 0.0
    else:
        e_db = math.copysign(magnitude, e)
        target = -cfg.k * e_db
        limit = abs(float(cfg.max_psi_correction))
        # Beyond the small-angle regime edot = v*sin(dpsi) ~ v*dpsi fails and
        # the stability argument with it -- and the steering cannot deliver it
        # anyway (delta_max is 0.278 rad at the wheels).
        target = max(-limit, min(limit, target))
        target *= soft_fade(age, cfg.fade_start_age, cfg.fade_zero_age)

    step = abs(float(cfg.max_psi_rate)) * max(float(dt), 0.0)
    prev = float(prev_psi) if math.isfinite(float(prev_psi)) else 0.0
    return max(prev - step, min(prev + step, target))
