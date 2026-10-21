"""The front-wall lock of a front_clearance stop, with no ROS in it.

behaviours/check_stop_condition.py owns the messages, the clock and the move;
this owns WHICH wall the stop may fire on. Design:
output/front_wall_lock/design.md (2026-10-21, M02 P002-R029 / R031 review).

THE GAP IT CLOSES
-----------------
A front_clearance stop asked one question of /perception/virtual_wall: is
the front wall closer than N metres, now? That topic is whichever wall the
travel strip meets on this scan (multi_wall_tracker.front_track). Nothing tied
it to the wall the move was sent to approach. Swerve round an obstacle and the
strip meets a side wall; the side wall becomes FRONT, and the stop fires on it.
The mission then records "arrived" metres from the wall it was asked to reach.

THE LOCK
--------
The same shape as the go_to_object handler's target lock (object_handler.py:
FOLLOW and its follow_gate_m), on the wall tracker's track ids, which are never
reused:

  unlocked    no lockable FRONT wall seen yet in this move. The stop reads the
              raw wall distance, exactly as it did before the lock: nothing
              has been chosen, so there is nothing to protect. CLEAR (50 m)
              never fires.
  locked      the first LOCKABLE FRONT track of the move is THE wall (see
              WHAT MAY BE LOCKED). The stop reads
              the contract distance (/perception/virtual_wall's value, from
              the debug payload that names its track) only while that track
              is the FRONT one. The deceleration ramp lands the car on that
              same value, so a move that never changes front wall behaves
              exactly as it did before the lock.
  not_front   the locked track is still tracked but something else is FRONT,
              or nothing is (CLEAR). The stop reads "not yet": it never fires
              on a wall it was not sent to. This is the swerve case.
  lost        the locked track is gone: absent from /perception/walls,
              coasting longer than the tracker lets a FRONT coast, or turned
              'unknown'. The one way back is RE-ACQUISITION, below. Until then
              the stop reads "not yet", and the move stays bounded by its own
              timeout_sec -- the fail-safe direction condition_eval.py already
              argues for an absent reading.
  no_reading  no fresh debug payload, so no way to know which track the
              contract value belongs to. "Not yet". (A stale or missing wall
              topic is held by WALL DISTANCE FRESHNESS before this runs.)

WHAT MAY BE LOCKED: a FRONT track on /perception/walls whose evidence includes
the LiDAR (MultiWallTrack.source LIDAR or BOTH). A lock is a strong claim --
every other wall is refused from then on -- and a STEREO-only track is one
"nothing has ever cross-checked". Measured on the M02 mission of 2026-10-21
12:25 (bf7938, COMPLETE): a stereo-only track 23 deg oblique was FRONT for two
tracker steps at 4.6 m and vanished; locking it would have refused the real
wall (LiDAR + stereo, 0.88 m / 25.7 deg off it) twelve seconds later, and the
mission would have timed out instead of completing. A stereo-only FRONT leaves
the lock open and the stop on the raw value, as before.

RE-ACQUISITION. A lost lock takes the current FRONT track only if it is
CONFIRMED and lies on the locked wall's line: its offset from the bumper
within gate_m of where the locked wall is now, and its normal within gate_rad
of the locked wall's. Anything else is refused, logged, and NOT adopted.

  * "Where the locked wall is now" is the dead-reckoned prediction
    lidar_front_wall.WallEstimator._predict computes, d = d0 - (p - p0) . n:
    the tracker's lines live in the odom frame, where a static wall does not
    move, so the anchor line evaluated at the current bumper IS that
    prediction (same_wall).
  * gate_m starts at 0.30 m, the WallEstimator's wrong-surface gate: the
    valley of the recorded innovation histogram (same-wall reacquisitions
    -16/+6 cm p10/p90, 28 % of fit losses back on a surface more than 2 m
    off).
  * gate_rad is what the design could not have with one scalar: a side wall
    at the same distance as the front one is the case the lock exists for,
    and only the normal tells them apart. 15 deg is NOT measured. It is far
    above the line-normal noise (0.4-0.8 deg 1-sigma) and the odometry yaw
    drift over one move, and far below the 90 deg of a side wall.

THE ANCHOR FOLLOWS THE LOCKED TRACK, NOT THE MOVE START (deviation from the
design, which roots the prediction at the move-start anchor). The drift the
design fixes is WallEstimator re-anchoring on ANY accepted fit, with no
identity. Here the anchor is only ever refreshed by the locked track's own
CONFIRMED estimate -- the tracker's association has already decided it is the
same wall -- or by a candidate that has just passed the gate. A frozen
move-start anchor would refuse the right wall: /odometry/filtered reads
distance ~17 % short (lidar_front_wall_odom_topic), so a static wall's
odom-frame line moves 0.17 m per metre driven, and after ~1.8 m of approach
the same wall re-observed sits outside a 0.30 m gate around the move-start
line.

COASTING. A locked track the tracker still holds but has not matched stays
the locked wall for at most coast_max_sec and coast_max_m of odometry since
it was last CONFIRMED -- the tracker's own FRONT coast caps
(lidar_front_wall_wall_tracker_front_max_coast_sec / _m), read from the same
stack_params keys. Past them it is lost, and a gated FRONT may replace it.
"""

import math
from dataclasses import dataclass
from typing import Dict, Optional, Tuple

# f1tenth_messages/MultiWallTrack.STATE_* and SOURCE_*; test_wall_lock.py
# asserts they stay equal.
STATE_TENTATIVE = 0
STATE_CONFIRMED = 1
STATE_COASTING = 2
SOURCE_LIDAR = 0
SOURCE_STEREO = 1
SOURCE_BOTH = 2

# wall_fusion.STATE_NAMES as /perception/virtual_wall/debug spells them.
FRONT_WALL = 'wall'
FRONT_CLEAR = 'clear'
FRONT_NO_DATA = 'no_data'

UNLOCKED = 'unlocked'
LOCKED = 'locked'
NOT_FRONT = 'not_front'
LOST = 'lost'
NO_READING = 'no_reading'

EVENT_ACQUIRED = 'acquired'
EVENT_REACQUIRED = 'reacquired'
EVENT_REFUSED = 'refused'

# multi_wall_tracker.is_unknown_role: 'unknown', its pre-2026-10-19 name
# 'other', and '' (not classified yet). Such a wall is a reference for nothing.
_UNKNOWN_ROLES = ('', 'unknown', 'other')


def _wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


@dataclass(frozen=True)
class WallTrack:
    """One /perception/walls track: the fields the lock reads. Lines are in odom."""

    track_id: int
    state: int
    role: str
    rho: float
    theta: float
    source: int = SOURCE_LIDAR      # the best evidence the track has ever had


@dataclass(frozen=True)
class FrontReading:
    """One /perception/virtual_wall/debug payload: the contract verdict, its
    value and the id of the FRONT track it came from, all from ONE publish."""

    state: str
    distance: Optional[float]
    front_id: Optional[int]


@dataclass(frozen=True)
class LockParams:
    gate_m: float = 0.30
    gate_rad: float = math.radians(15.0)
    coast_max_sec: float = 0.5
    coast_max_m: float = 0.3


@dataclass(frozen=True)
class LockStep:
    """One tick's decision. distance is what the stop reads; None is "not yet"."""

    phase: str
    distance: Optional[float]
    track_id: Optional[int]
    event: Optional[str] = None
    detail: str = ''


def same_wall(anchor: Tuple[float, float], candidate: Tuple[float, float],
              point: Tuple[float, float], gate_m: float,
              gate_rad: float) -> Tuple[bool, float, float]:
    """Is `candidate` the anchored wall? Returns (ok, offset_m, angle_rad).

    Both lines are (rho, theta) in odom, rho = x cos(theta) + y sin(theta).
    offset_m compares their signed offsets from `point` (the bumper) along
    each normal, so it is the candidate's distance minus the anchor's
    dead-reckoned one, measured where the car is. A line passing near the
    odom origin can come back normalised the other way (theta + pi, -rho);
    the candidate is flipped onto the anchor's side first.
    """
    rho_a, th_a = anchor
    rho_c, th_c = candidate
    if abs(_wrap(th_c - th_a)) > math.pi / 2.0:
        rho_c, th_c = -rho_c, th_c + math.pi
    angle = abs(_wrap(th_c - th_a))
    x, y = point
    off_a = rho_a - (x * math.cos(th_a) + y * math.sin(th_a))
    off_c = rho_c - (x * math.cos(th_c) + y * math.sin(th_c))
    offset = abs(off_c - off_a)
    return offset <= gate_m and angle <= gate_rad, offset, angle


class FrontWallLock:
    """The per-move lock. One instance per behaviour; reset() on every move."""

    def __init__(self, params: Optional[LockParams] = None):
        self.params = params or LockParams()
        self.reset()

    def reset(self) -> None:
        """Forget the locked wall: a new move approaches its own."""
        self.track_id: Optional[int] = None
        self.anchor: Optional[Tuple[float, float]] = None
        self._confirmed_at: Optional[Tuple[float, Optional[float]]] = None

    def update(self, now: float, raw_distance: Optional[float],
               front: Optional[FrontReading],
               tracks: Optional[Dict[int, WallTrack]],
               bumper_xy: Optional[Tuple[float, float]],
               odometer: Optional[float]) -> LockStep:
        """Advance one tick.

        raw_distance is the wall topic's own value, read while unlocked. front
        is None without a fresh debug payload, tracks None without a fresh
        /perception/walls (by id), bumper_xy and odometer None before the
        first odometry message. The caller decides freshness.
        """
        if self.track_id is None:
            return self._acquire(now, raw_distance, front, tracks, odometer)

        trk = tracks.get(self.track_id) if tracks is not None else None
        # Only a matched, still-meaningful estimate moves the anchor: not a
        # coast (a prediction) and not a track the tracker has declared a
        # reference for nothing ('unknown': rect-rejected, occluded).
        if (trk is not None and trk.state == STATE_CONFIRMED
                and trk.role not in _UNKNOWN_ROLES):
            self._seen(trk, now, odometer)
        if front is None:
            return LockStep(NO_READING, None, self.track_id,
                            detail='no fresh /perception/virtual_wall/debug, so no '
                                   'way to tell which wall the distance belongs to')
        if front.state == FRONT_WALL and front.front_id == self.track_id:
            return LockStep(LOCKED, front.distance, self.track_id)
        if self._alive(trk, now, odometer):
            other = (f'track {front.front_id} is the front wall'
                     if front.state == FRONT_WALL else f'the front is {front.state}')
            return LockStep(NOT_FRONT, None, self.track_id,
                            detail=f'locked wall {self.track_id} is not in the way ({other})')
        return self._reacquire(now, front, tracks, bumper_xy, odometer)

    def _acquire(self, now, raw_distance, front, tracks, odometer):
        if front is None or front.state != FRONT_WALL or front.front_id is None:
            return LockStep(UNLOCKED, raw_distance, None)
        # WHAT MAY BE LOCKED. The track must be on a fresh /perception/walls
        # too: that is where its evidence and its anchor line are.
        trk = tracks.get(front.front_id) if tracks is not None else None
        if trk is None:
            return LockStep(UNLOCKED, raw_distance, None,
                            detail=f'front track {front.front_id} is not on a fresh '
                                   '/perception/walls; not locked')
        if trk.source == SOURCE_STEREO:
            return LockStep(UNLOCKED, raw_distance, None,
                            detail=f'front track {front.front_id} is stereo-only; not locked')
        self.track_id = trk.track_id
        self.anchor = (trk.rho, trk.theta)
        if trk.state == STATE_CONFIRMED:
            self._confirmed_at = (now, odometer)
        return LockStep(LOCKED, front.distance, self.track_id, event=EVENT_ACQUIRED)

    def _seen(self, trk, now, odometer):
        self.anchor = (trk.rho, trk.theta)
        self._confirmed_at = (now, odometer)

    def _alive(self, trk, now, odometer):
        """Is the locked track still the locked wall? See COASTING."""
        if trk is None or trk.role in _UNKNOWN_ROLES:
            return False
        if trk.state == STATE_CONFIRMED:
            return True
        if trk.state != STATE_COASTING or self._confirmed_at is None:
            return False
        t0, m0 = self._confirmed_at
        if now - t0 > self.params.coast_max_sec:
            return False
        if odometer is not None and m0 is not None and odometer - m0 > self.params.coast_max_m:
            return False
        return True

    def _reacquire(self, now, front, tracks, bumper_xy, odometer):
        lost = f'locked wall {self.track_id} lost'
        if front.state != FRONT_WALL or front.front_id is None:
            return LockStep(LOST, None, self.track_id,
                            detail=f'{lost}; the front is {front.state}')
        cand = tracks.get(front.front_id) if tracks is not None else None
        if cand is None or cand.state != STATE_CONFIRMED:
            return LockStep(LOST, None, self.track_id,
                            detail=f'{lost}; front track {front.front_id} is not a '
                                   'confirmed track on /perception/walls')
        if self.anchor is None or bumper_xy is None:
            return LockStep(LOST, None, self.track_id, event=EVENT_REFUSED,
                            detail=f'{lost}; front track {cand.track_id} cannot be '
                                   'gated (no anchor line or no pose)')
        p = self.params
        ok, offset, angle = same_wall(self.anchor, (cand.rho, cand.theta), bumper_xy,
                                      p.gate_m, p.gate_rad)
        off = (f'{offset:.2f} m / {math.degrees(angle):.1f} deg off it (gate '
               f'{p.gate_m:.2f} m / {math.degrees(p.gate_rad):.1f} deg)')
        if not ok:
            at = '' if front.distance is None else f' at {front.distance:.2f} m'
            return LockStep(LOST, None, self.track_id, event=EVENT_REFUSED,
                            detail=f'{lost}; front track {cand.track_id}{at} is '
                                   f'{off} -- not adopted')
        previous = self.track_id
        self.track_id = cand.track_id
        self._seen(cand, now, odometer)
        return LockStep(LOCKED, front.distance, self.track_id, event=EVENT_REACQUIRED,
                        detail=f'track {previous} -> {cand.track_id}, {off}')
