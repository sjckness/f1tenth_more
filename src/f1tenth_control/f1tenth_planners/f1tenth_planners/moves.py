"""The mission's goal shapes as plain objects: where to plan to, and when the move is done.

Pure: math only. One class per goal topic mpc_corr takes, with mpc_corr's
semantics, so corridor_rrt and p2p_geometric honour the same mission files:

  DistanceMove  /mpc/goal_distance  "drive d metres made good along the heading
                the move started with". Progress is the along-line projection
                (MPC_corr._project_onto_line), not the straight-line
                displacement, so a dodge does not count as progress.
  PoseMove      /mpc/goal_pose      position-only arrival within a tolerance
                (mpc_corr: pose_goal_tolerance); the yaw is carried, not used.
  TurnMove      /mpc/goal_turn      the end of the requested steering's turn
                circle (R = wheelbase / tan(steering), full_lock or
                partial:<deg> as mpc_corr parses them) after the heading
                change, plus TURN_LEAD_M straight on along the new heading,
                with that heading as the end heading. mpc_corr instead puts
                its synthetic point one chord along the NEW heading's ray,
                which works as a corridor bearing but, as a point to plan to,
                sits beside the car and asks a forward-only planner for a
                loop. The mission's orientation_delta stop ends the move, as
                with mpc_corr -- and because that stop is `accumulated turn >=
                value`, the target heading is TURN_OVERSHOOT_DEG past the
                requested one: a tracker that settles exactly on 90.0 deg
                leaves the mission's accumulator at 89.98 and the move times
                out (seen in the first end-to-end run of test_01.json).
  DriveMove     /mpc/goal_drive     OPEN-ENDED: never reached, the mission's
                stop_condition ends it. 'straight' holds the heading the move
                started with; 'wall_turn' does that until the front wall is
                within the turn's trigger distance, then turns by turn_mag_deg
                toward turn_sign once and holds the new heading.
  ObjectMove    /mpc/goal_object    the standoff point short of a tracked
                target, refreshed every tick. Arrival is mpc_corr's
                ObjectStopLatch, reported on /mpc/object_status; the mission's
                object_reached decides, exactly as with mpc_corr.

A move answers two questions: target(pose) -- where to plan to now (a
PlanTarget) -- and progress(pose) -- how far is left and whether it is over.
"""

from dataclasses import dataclass
import math
from typing import Optional, Tuple

from f1tenth_planners.geometry import project_onto_line, wrap_pi


@dataclass
class PlanTarget:
    """Where to plan to: a point, an optional end heading, and whether the move ends there."""

    x: float
    y: float
    yaw: Optional[float]
    terminal: bool


@dataclass
class Progress:
    """How far is left [m] (inf when open-ended) and whether the move has arrived."""

    remaining: float
    reached: bool


class Move:
    """Base: a move with a speed reference [m/s]; None means the node's cruise speed."""

    kind = 'none'
    open_ended = False

    def __init__(self, speed: Optional[float] = None):
        self.speed = speed if (speed is not None and speed > 0.0) else None

    def target(self, pose) -> Optional[PlanTarget]:
        """Where to plan to from ``pose`` (x, y, yaw)."""
        return None

    def progress(self, pose) -> Progress:
        """Distance still to go and arrival."""
        return Progress(math.inf, False)

    def describe(self) -> str:
        """One line for the log."""
        return self.kind


class DistanceMove(Move):
    """Drive ``distance`` metres made good along the start heading."""

    kind = 'distance'

    def __init__(self, start_pose, distance: float, stop_lead: float, speed=None):
        super().__init__(speed)
        self.anchor = (float(start_pose[0]), float(start_pose[1]))
        self.psi = float(start_pose[2])
        self.distance = float(distance)
        self.stop_lead = float(stop_lead)

    def traveled(self, pose) -> float:
        """Along-line progress from the anchor."""
        return project_onto_line((pose[0], pose[1]), self.anchor, self.psi)

    def target(self, pose):
        """Return the point ``distance`` along the start heading, ending on that heading."""
        return PlanTarget(self.anchor[0] + self.distance * math.cos(self.psi),
                          self.anchor[1] + self.distance * math.sin(self.psi),
                          self.psi, True)

    def progress(self, pose):
        """Remaining distance along the line; arrived one stop_lead short (the car coasts)."""
        remaining = self.distance - self.traveled(pose)
        return Progress(remaining, remaining <= self.stop_lead)

    def describe(self):
        """One line for the log."""
        return (f'distance {self.distance:.2f} m from ({self.anchor[0]:+.2f},'
                f'{self.anchor[1]:+.2f}) heading {self.psi:+.3f}')


class PoseMove(Move):
    """Reach a position (odom), within tolerance, or pass it within pass_tolerance."""

    kind = 'pose'

    def __init__(self, start_pose, goal_xy, goal_yaw: Optional[float],
                 tolerance: float, pass_tolerance: float, stop_lead: float = 0.0,
                 speed=None, end_heading: Optional[float] = None):
        super().__init__(speed)
        self.goal = (float(goal_xy[0]), float(goal_xy[1]))
        self.goal_yaw = goal_yaw
        self.end_heading = end_heading
        self.tolerance = float(tolerance)
        self.pass_tolerance = float(pass_tolerance)
        self.stop_lead = float(stop_lead)
        dx, dy = self.goal[0] - start_pose[0], self.goal[1] - start_pose[1]
        self.approach = math.atan2(dy, dx) if math.hypot(dx, dy) > 1e-6 else float(start_pose[2])

    def target(self, pose):
        """Return the goal point; an end heading only for a turn."""
        return PlanTarget(self.goal[0], self.goal[1], self.end_heading, True)

    def progress(self, pose):
        """Distance to the goal; arrived inside the tolerance or having passed it."""
        dist = math.hypot(pose[0] - self.goal[0], pose[1] - self.goal[1])
        if dist <= max(self.tolerance, self.stop_lead):
            return Progress(dist, True)
        heading = self.end_heading if self.end_heading is not None else self.approach
        passed = project_onto_line((pose[0], pose[1]), self.goal, heading) >= 0.0
        return Progress(dist, bool(passed and dist <= self.pass_tolerance))

    def describe(self):
        """One line for the log."""
        return f'pose ({self.goal[0]:+.2f},{self.goal[1]:+.2f}) tol {self.tolerance:.2f} m'


#: Straight run [m] past the end of a turn's arc that the turn's target adds.
TURN_LEAD_M = 0.5
#: How far past the requested heading change a turn aims [deg], so the
#: mission's orientation_delta (accumulated >= value) is crossed, not grazed.
TURN_OVERSHOOT_DEG = 5.0


def turn_radius(steering: str, wheelbase: float, max_steering: float) -> float:
    """Radius [m] of the turn a TurnGoal asks for; mpc_corr's steering parsing."""
    if steering == 'full_lock':
        steering_rad = max_steering
    else:
        try:
            steering_rad = min(math.radians(abs(float(steering.split(':', 1)[1]))),
                               max_steering)
        except (IndexError, ValueError):
            steering_rad = max_steering
    steering_rad = max(steering_rad, math.radians(1.0))
    return wheelbase / math.tan(steering_rad)


def turn_reach(heading_delta: float, steering: str, wheelbase: float,
               max_steering: float, reach_max: float = 3.0) -> float:
    """mpc_corr's _resolve_turn_reach: the chord of the requested steering's circle."""
    radius = turn_radius(steering, wheelbase, max_steering)
    chord = 2.0 * radius * math.sin(abs(heading_delta) / 2.0)
    return float(min(max(chord, 0.3), reach_max))


class TurnMove(PoseMove):
    """A heading change: the end of the requested turn circle, then TURN_LEAD_M straight."""

    kind = 'turn'

    def __init__(self, start_pose, heading_delta: float, steering: str, wheelbase: float,
                 max_steering: float, tolerance: float, pass_tolerance: float,
                 speed=None):
        x0, y0, yaw0 = (float(v) for v in start_pose)
        sign = 1.0 if heading_delta >= 0.0 else -1.0
        target_yaw = yaw0 + float(heading_delta) + sign * math.radians(TURN_OVERSHOOT_DEG)
        radius = turn_radius(steering, wheelbase, max_steering)
        # Centre of the circle, then the point on it at the new heading.
        cx, cy = x0 - sign * radius * math.sin(yaw0), y0 + sign * radius * math.cos(yaw0)
        ax = cx + sign * radius * math.sin(target_yaw)
        ay = cy - sign * radius * math.cos(target_yaw)
        goal = (ax + TURN_LEAD_M * math.cos(target_yaw), ay + TURN_LEAD_M * math.sin(target_yaw))
        super().__init__(start_pose, goal, target_yaw, tolerance, pass_tolerance,
                         speed=speed, end_heading=target_yaw)
        self.heading_delta = float(heading_delta)
        self.radius = radius

    def describe(self):
        """One line for the log."""
        return (f'turn {math.degrees(self.heading_delta):+.1f} deg on R {self.radius:.2f} m '
                f'-> ({self.goal[0]:+.2f},{self.goal[1]:+.2f})')


class DriveMove(Move):
    """Open-ended 'straight' or 'wall_turn' drive; see the module docstring."""

    kind = 'drive'
    open_ended = True

    def __init__(self, start_pose, mode: str, turn_sign: float, turn_mag: float,
                 horizon: float, trigger_distance: float, speed=None):
        super().__init__(speed)
        self.mode = mode if mode in ('straight', 'wall_turn') else 'straight'
        if self.mode == 'wall_turn' and turn_sign == 0.0:
            self.mode = 'straight'
        self.psi = float(start_pose[2])
        self.turn_sign = math.copysign(1.0, turn_sign) if turn_sign else 0.0
        self.turn_mag = abs(float(turn_mag))
        self.horizon = float(horizon)
        self.trigger_distance = float(trigger_distance)
        self.committed = False

    def update_front_distance(self, front_distance: Optional[float]) -> bool:
        """Commit the wall turn once the wall is close enough; True on the commit tick."""
        if self.mode != 'wall_turn' or self.committed or front_distance is None:
            return False
        if 0.0 < front_distance <= self.trigger_distance:
            self.committed = True
            self.psi = wrap_pi(self.psi + self.turn_sign * self.turn_mag)
            return True
        return False

    def target(self, pose):
        """Return a point ``horizon`` ahead of the car along the held heading (rolling)."""
        return PlanTarget(pose[0] + self.horizon * math.cos(self.psi),
                          pose[1] + self.horizon * math.sin(self.psi),
                          self.psi, False)

    def describe(self):
        """One line for the log."""
        extra = ''
        if self.mode == 'wall_turn':
            extra = (f' turn {self.turn_sign * math.degrees(self.turn_mag):+.0f} deg at '
                     f'{self.trigger_distance:.2f} m'
                     + (' (committed)' if self.committed else ''))
        return f'drive {self.mode} heading {self.psi:+.3f}{extra} (open-ended)'


class ObjectMove(Move):
    """Approach a tracked target to ``standoff`` metres; target refreshed every tick."""

    kind = 'object'

    def __init__(self, move_id: str, target_class: str, track_id: str,
                 target_map_xy, standoff: float, speed: float, gap_m: float):
        super().__init__(speed)
        self.move_id = move_id
        self.target_class = target_class
        self.track_id = track_id
        self.target_map_xy = (float(target_map_xy[0]), float(target_map_xy[1]))
        self.target_odom_xy: Optional[Tuple[float, float]] = None
        self.standoff = float(standoff)
        self.raw_speed = float(speed)
        self.gap_m = float(gap_m)
        self.stamp_sec: Optional[float] = None
        self.psi_c: Optional[float] = None

    def r(self, pose) -> float:
        """Range from the car to the standoff point (negative inside it)."""
        if self.target_odom_xy is None:
            return math.inf
        return math.hypot(self.target_odom_xy[0] - pose[0],
                          self.target_odom_xy[1] - pose[1]) - self.standoff

    def target(self, pose):
        """Return the standoff point on the car-to-target line."""
        if self.target_odom_xy is None:
            return None
        tx, ty = self.target_odom_xy
        bearing = math.atan2(ty - pose[1], tx - pose[0])
        return PlanTarget(tx - self.standoff * math.cos(bearing),
                          ty - self.standoff * math.sin(bearing), None, True)

    def progress(self, pose):
        """Range to the standoff point; arrival is the stop latch's call, not this."""
        return Progress(self.r(pose), False)

    def describe(self):
        """One line for the log."""
        return (f'object {self.target_class!r} move_id={self.move_id!r} '
                f'standoff {self.standoff:.2f} m')
