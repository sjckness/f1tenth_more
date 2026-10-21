"""Which wall distance the deceleration ramp aims at, under the front-wall lock.

Pure: no rclpy, no ROS types. MPC_corr feeds RampWallSource the two topics
below and asks it for a distance on every tick of a wall stop
(MPC_corr._ramp_wall_distance, read by _wall_stop_d_rem). Design:
output/front_wall_lock/design.md, "The ramp aims at the locked wall".

THE GAP IT CLOSES. f1tenth_behavior's front-wall lock (mission/wall_lock.py)
decides which wall a front_clearance stop may fire on: the first LiDAR-backed
FRONT track of the move, by the wall tracker's id. It made the STOP fire only
on that wall, but the ramp kept reading the raw /perception/virtual_wall,
which is whichever wall the travel strip meets on this scan. Swerve round a
person, the strip meets a side wall nearer than the target, and the ramp
brakes the car to rest by the side wall (the P002-R029 case): the lock turned
a false "arrived" into a timeout, and the car still stopped in the wrong place.

THE WIRE. Two inputs, each kept with its receipt time:
  * /mpc/stop_wall_id (std_msgs/Int32), published by CheckStopCondition on
    every tick it runs the lock: the locked track's id while the lock holds,
    NO_LOCK (-1) while it does not (wall_lock.ramp_wall_id has the mapping).
    The BT owns every rule of the lock -- acquisition, the stereo-only rule,
    the coast caps, the re-acquisition gates; this module only reads its
    answer, so the two can never disagree on which wall is locked.
  * /perception/virtual_wall/debug (JSON), the payload the lock itself reads:
    `state`, `distance_m` and `front_id` from ONE publish, so the value is
    known to belong to that track. Pairing the id with the Float32 instead
    would mix two scans across a front switch, which is the tick that matters.

THE CASES (select_ramp_distance):
  wall_lock_enabled false   MPC_corr builds no RampWallSource and subscribes
                            to neither topic; the ramp reads
                            _fresh_front_distance, exactly as before.
  not locked yet (-1)       the raw wall distance, as before.
  locked, its wall FRONT    the payload's distance_m: the locked track's
                            contract value, the number the stop reads. A move
                            that never changes front wall ramps on the same
                            value as before.
  locked, its wall not      None: no wall ramp this tick. The ramp is
  FRONT                     deceleration toward the wall the move was sent
                            to, and that wall is not in the strip. Another
                            wall or obstacle in the way stays with what
                            already handles any wall that is not FRONT: the
                            solver's wall rows, the obstacle avoidance, the
                            stop-and-reverse logic, the LiDAR stop. The
                            mission's stop reads "not yet" on the same tick.
  locked wall lost (-1)     the raw wall distance, until the BT re-acquires
                            the lock under its gates. Conservative: the ramp
                            then brakes for whatever wall is FRONT, so a lost
                            lock can only brake more than a held one.

Degraded inputs fall back to the raw distance, as if unlocked (conservative):
  * no fresh lock id -- none since the move's /mpc/stop_target, or older than
    max_age_s (wall_distance_max_age_s, five BT ticks): the BT is not running
    the lock (holding the car, dead, or between moves);
  * the lock holds but there is no usable payload -- missing, stale,
    unreadable, or a FRONT of the locked track whose distance_m is not a
    distance -- so nothing says which wall the raw value belongs to.

A value that is not a distance (not finite, <= 0, or at/above the contract's
CLEAR distance) never comes out of the locked branch: the same filter
_fresh_front_distance applies to the raw one.
"""

import json
import math
from typing import NamedTuple, Optional

# /mpc/stop_wall_id when no lock is in force (f1tenth_behavior's
# wall_lock.NO_LOCK_ID; test_ramp_wall_source.py asserts they stay equal).
NO_LOCK = -1

# wall_fusion.STATE_NAMES['wall'], as the debug payload spells it.
FRONT_WALL = 'wall'

# Where this tick's distance came from (RampWallPick.source).
SOURCE_RAW = 'raw'                      # no lock in force: unlocked or lost
SOURCE_RAW_NO_LOCK = 'raw_no_lock_id'   # no fresh /mpc/stop_wall_id
SOURCE_RAW_NO_DEBUG = 'raw_no_debug'    # locked, but no usable debug payload
SOURCE_LOCKED = 'locked'                # the locked wall is FRONT: its value
SOURCE_NOT_FRONT = 'not_front'          # the locked wall is not FRONT: no ramp


class FrontReading(NamedTuple):
    """One /perception/virtual_wall/debug payload, all from ONE publish.

    The contract verdict, its value and the id of the FRONT track it came from.
    """

    state: str
    distance: Optional[float]
    front_id: Optional[int]


class RampWallPick(NamedTuple):
    """One tick's answer. distance None means no wall ramp this tick."""

    distance: Optional[float]
    source: str
    track_id: Optional[int] = None      # the locked track, while the lock holds
    front_id: Optional[int] = None      # the payload's FRONT track


def front_from_debug(text: str) -> Optional[FrontReading]:
    """Return the FrontReading in a /perception/virtual_wall/debug payload, or None.

    Reads the three keys lidar_front_wall_node writes from one publish and
    nothing else -- the same contract as f1tenth_behavior's
    check_stop_condition.front_from_debug, which the lock reads. None for a
    payload that is not a JSON object with `state`, or whose numbers do not
    parse: an unreadable verdict is no verdict.
    """
    try:
        payload = json.loads(text)
        state = str(payload['state'])
        distance = payload.get('distance_m')
        front_id = payload.get('front_id')
        return FrontReading(
            state=state,
            distance=float(distance) if distance is not None else None,
            front_id=int(front_id) if front_id is not None else None)
    except (ValueError, TypeError, KeyError, AttributeError):
        return None


def _is_distance(value, clear_distance_m) -> bool:
    return (value is not None and math.isfinite(value) and value > 0.0
            and value < clear_distance_m)


def select_ramp_distance(raw: Optional[float], locked_id: Optional[int],
                         front: Optional[FrontReading],
                         clear_distance_m: float) -> RampWallPick:
    """Return the wall distance the ramp aims at this tick (module docstring, THE CASES).

    raw         MPC_corr._fresh_front_distance(): None when absent, stale,
                NO_DATA or CLEAR.
    locked_id   the fresh /mpc/stop_wall_id: a track id, NO_LOCK, or None when
                there is no fresh one.
    front       the fresh debug payload, or None.
    """
    if locked_id is None:
        return RampWallPick(raw, SOURCE_RAW_NO_LOCK)
    if locked_id < 0:
        return RampWallPick(raw, SOURCE_RAW)
    if front is None:
        return RampWallPick(raw, SOURCE_RAW_NO_DEBUG, locked_id)
    if front.state == FRONT_WALL and front.front_id == locked_id:
        if _is_distance(front.distance, clear_distance_m):
            return RampWallPick(front.distance, SOURCE_LOCKED, locked_id, front.front_id)
        return RampWallPick(raw, SOURCE_RAW_NO_DEBUG, locked_id, front.front_id)
    return RampWallPick(None, SOURCE_NOT_FRONT, locked_id, front.front_id)


def describe(pick: RampWallPick) -> str:
    """One log phrase for a pick: what the ramp aims at, and why."""
    if pick.source == SOURCE_LOCKED:
        return f'locked wall {pick.track_id} at {pick.distance:.2f} m'
    if pick.source == SOURCE_NOT_FRONT:
        other = ('the front is not a wall' if pick.front_id is None
                 else f'track {pick.front_id} is the front wall')
        return (f'no wall ramp: locked wall {pick.track_id} is not in the way '
                f'({other})')
    why = {
        SOURCE_RAW: 'no wall locked',
        SOURCE_RAW_NO_LOCK: 'no fresh /mpc/stop_wall_id',
        SOURCE_RAW_NO_DEBUG: (f'wall {pick.track_id} locked, but no usable '
                              '/perception/virtual_wall/debug'),
    }.get(pick.source, pick.source)
    return f'the raw wall topic ({why})'


class RampWallSource:
    """The two inputs with their receipt times, and the selection over them.

    One per MPC_corr, built only with wall_lock_enabled. Times are whatever
    clock the caller passes, as long as it is one clock (MPC_corr: its node
    clock, so sim time in the simulator).
    """

    def __init__(self, max_age_s: float, clear_distance_m: float):
        self.max_age_s = float(max_age_s)
        self.clear_distance_m = float(clear_distance_m)
        self._debug_text: Optional[str] = None
        self._debug_rx: Optional[float] = None
        self.reset()

    def reset(self) -> None:
        """Forget the lock id: a new move starts unlocked until the BT says."""
        self._lock_id: Optional[int] = None
        self._lock_rx: Optional[float] = None

    def on_lock_id(self, value: int, now: float) -> None:
        self._lock_id = int(value)
        self._lock_rx = float(now)

    def on_debug(self, text: str, now: float) -> None:
        # Parsed only when a locked tick needs it: the payload arrives at the
        # tracker rate whether or not a wall stop is running.
        self._debug_text = text
        self._debug_rx = float(now)

    def _fresh(self, rx, now) -> bool:
        return rx is not None and now - rx <= self.max_age_s

    def select(self, raw: Optional[float], now: float) -> RampWallPick:
        locked_id = self._lock_id if self._fresh(self._lock_rx, now) else None
        front = None
        if locked_id is not None and locked_id >= 0 and self._fresh(self._debug_rx, now):
            front = front_from_debug(self._debug_text)
        return select_ramp_distance(raw, locked_id, front, self.clear_distance_m)
