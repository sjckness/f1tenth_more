"""The go_to_object handler's decisions, with no ROS in it.

behaviours/go_to_object.py owns the messages, the clock and the blackboard;
this owns which track is the target and when the move has failed. Replaces
object_goal_bridge, which republished /mpc/goal_pose at 20 Hz outside any
mission and re-anchored the controller on every message.

PHASES
------
ACQUIRE  no target yet. The nearest confirmed track of target_class (nearest
         to the vehicle) becomes the target. None within acquire_timeout_sec
         -> target_not_found.
FOLLOW   the nearest confirmed track of target_class within follow_gate_m of
         the LAST TARGET POINT is the target, REGARDLESS OF track_id.
         semantic_layer_node re-issues ids across short dropouts; a person
         who is re-acquired under a new id 10 cm from where they were is
         still the person being approached, and a different person 2 m away
         with the right class is not.
GRACE    no gated track. STOP AND WAIT: the last point keeps being
         published, at speed 0, for lost_grace_sec -- mpc_corr holds the car
         stopped with its steering where it was -- and a gated track brings the
         handler back to FOLLOW at the move's speed. When the grace expires ->
         target_lost. Not a reduced speed: the car is not operated below
         min_moving_speed_mps (0.4 m/s), and the halved speed this used to send
         (0.4 * 0.5 = 0.2) was exactly such a speed.

Every confirmed track in a tracks message counts: semantic_layer_node
publishes confirmed tracks only. A tracks message older than tracks_max_gap_sec
(by RECEIPT time) counts as no tracks at all, so a stalled perception pipeline
reads as a lost target rather than as a target frozen in place.

OUTCOMES decided here: target_not_found, target_lost, target_unreachable (the
controller reported target_behind_terminal for this move). reached and
timeout are decided by CheckStopCondition, like every other move's stop.
"""

import math
from dataclasses import dataclass
from typing import Iterable, Optional, Sequence, Tuple

from f1tenth_params.object_geometry import nominal_footprint_radius

ACQUIRE = 'ACQUIRE'
FOLLOW = 'FOLLOW'
GRACE = 'GRACE'
ENDED = 'ENDED'

OUTCOME_REACHED = 'reached'
OUTCOME_NOT_FOUND = 'target_not_found'
OUTCOME_LOST = 'target_lost'
OUTCOME_UNREACHABLE = 'target_unreachable'
OUTCOME_TIMEOUT = 'timeout'
OUTCOMES = (OUTCOME_REACHED, OUTCOME_NOT_FOUND, OUTCOME_LOST,
            OUTCOME_UNREACHABLE, OUTCOME_TIMEOUT)


@dataclass(frozen=True)
class Track:
    """One confirmed semantic track, map frame."""

    track_id: str
    class_id: str
    x: float
    y: float
    score: float
    stamp_sec: float   # the tracks message's capture stamp
    width: float = 0.0  # fused footprint width [m]; <= 0 means unknown


@dataclass(frozen=True)
class HandlerParams:
    """One move's handler configuration: the move's spec plus stack tuning."""

    target_class: str
    gap_m: float          # car front to object near edge
    nose_reach_m: float   # base_link to car front, as the solver measures it
    speed: float
    acquire_timeout_sec: float
    lost_grace_sec: float
    follow_gate_m: float = 0.5
    tracks_max_gap_sec: float = 0.5


@dataclass(frozen=True)
class HandlerStep:
    """One tick's decision."""

    phase: str
    # The point to publish, map frame, and its capture stamp; None while
    # nothing has been acquired.
    target_xy: Optional[Tuple[float, float]]
    target_stamp_sec: Optional[float]
    track_id: Optional[str]
    speed: float
    outcome: Optional[str] = None
    # The target's footprint radius and the centre distance the controller is
    # sent (ObjectGoal.standoff): gap_m + nose_reach_m + target_radius.
    target_radius: Optional[float] = None
    centre_standoff: Optional[float] = None


def track_radius(track: Track) -> float:
    """Return the target's footprint radius [m]: fused width / 2, else the class nominal.

    THE r_target RULE. The handler sends mpc_corr a centre distance built on it,
    and anything that reports a gap to the same object (the scoring, the
    watch_objects debug script) must use it too, or two tools disagree about
    one car and one object.
    """
    if track.width and track.width > 0.0:
        return track.width / 2.0
    return nominal_footprint_radius(track.class_id)


def centre_distance_for_gap(gap_m: float, nose_reach_m: float, radius_m: float) -> float:
    """Return the base_link-to-centre distance at which the front-to-edge gap is gap_m."""
    return float(gap_m) + float(nose_reach_m) + float(radius_m)


def gap_for_centre_distance(distance_m: float, nose_reach_m: float, radius_m: float) -> float:
    """Return the front-to-edge gap [m] at a base_link-to-centre distance (the inverse)."""
    return float(distance_m) - float(nose_reach_m) - float(radius_m)


def nearest_track(tracks: Iterable[Track], target_class: str,
                  point: Tuple[float, float],
                  within: Optional[float] = None) -> Optional[Track]:
    """Nearest track of target_class to `point`, optionally within a radius."""
    best, best_d = None, math.inf
    for track in tracks:
        if track.class_id != target_class:
            continue
        d = math.hypot(track.x - point[0], track.y - point[1])
        if within is not None and d > within:
            continue
        if d < best_d:
            best, best_d = track, d
    return best


class ObjectHandler:
    """The per-move state machine. One instance per move; update() is the interface."""

    def __init__(self, params: HandlerParams, start_sec: float):
        """Start in ACQUIRE at start_sec."""
        self.params = params
        self.start_sec = float(start_sec)
        self.phase = ACQUIRE
        self.target_xy: Optional[Tuple[float, float]] = None
        self.target_stamp_sec: Optional[float] = None
        self.track_id: Optional[str] = None
        self.target_radius: Optional[float] = None
        self.grace_since: Optional[float] = None
        self.outcome: Optional[str] = None
        self.transitions = []

    def _go(self, phase, now_sec):
        if phase != self.phase:
            self.transitions.append((now_sec, self.phase, phase))
            self.phase = phase

    def _step(self, speed, outcome=None):
        standoff = (None if self.target_radius is None else centre_distance_for_gap(
            self.params.gap_m, self.params.nose_reach_m, self.target_radius))
        return HandlerStep(self.phase, self.target_xy, self.target_stamp_sec,
                           self.track_id, speed, outcome, self.target_radius, standoff)

    def end(self, outcome: str, now_sec: float) -> HandlerStep:
        """End the move from outside (reached, timeout) or from inside."""
        self.outcome = outcome
        self._go(ENDED, now_sec)
        return self._step(0.0, outcome)

    def update(self, now_sec: float, tracks: Sequence[Track],
               tracks_received_sec: Optional[float],
               vehicle_xy: Optional[Tuple[float, float]],
               behind_terminal: bool = False) -> HandlerStep:
        """Advance one tick. `tracks` is the latest message's confirmed tracks."""
        p = self.params
        if self.phase == ENDED:
            return self._step(0.0, self.outcome)
        if behind_terminal and self.phase != ACQUIRE:
            return self.end(OUTCOME_UNREACHABLE, now_sec)

        fresh = (tracks_received_sec is not None
                 and now_sec - tracks_received_sec <= p.tracks_max_gap_sec)
        usable = list(tracks) if fresh else []

        if self.phase == ACQUIRE:
            track = (nearest_track(usable, p.target_class, vehicle_xy)
                     if vehicle_xy is not None else None)
            if track is None:
                if now_sec - self.start_sec >= p.acquire_timeout_sec:
                    return self.end(OUTCOME_NOT_FOUND, now_sec)
                return self._step(0.0)
            self._take(track)
            self._go(FOLLOW, now_sec)
            return self._step(p.speed)

        track = nearest_track(usable, p.target_class, self.target_xy, within=p.follow_gate_m)
        if track is not None:
            self._take(track)
            self.grace_since = None
            self._go(FOLLOW, now_sec)
            return self._step(p.speed)

        if self.phase == FOLLOW:
            self.grace_since = now_sec
            self._go(GRACE, now_sec)
        if now_sec - self.grace_since >= p.lost_grace_sec:
            return self.end(OUTCOME_LOST, now_sec)
        # Stop and wait: see GRACE in the module docstring.
        return self._step(0.0)

    def _take(self, track: Track):
        self.target_xy = (track.x, track.y)
        self.target_stamp_sec = track.stamp_sec
        self.track_id = track.track_id
        self.target_radius = track_radius(track)


def object_move_wire_id(mission_id: str, run_generation: int, move_id: str) -> str:
    """ObjectGoal.move_id: stable for the move, distinct across runs.

    A mission's move ids repeat across runs, and mpc_corr remembers ended ids,
    so the bare move id would make run 2 of a mission un-startable (and, before
    ended ids were remembered, silently continue run 1's anchor).
    """
    return f'{mission_id}#{run_generation}/{move_id}'
