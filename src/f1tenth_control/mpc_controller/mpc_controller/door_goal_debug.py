"""door_goal_debug.py -- pure helpers for the live door -> MPC goal diagnostic.

No rclpy. MPC_corr feeds them the DoorArray it already could see, the goal it
is driving at (odom, as the solver uses it) and the live map -> odom edge, and
publishes what comes back as Foxglove markers plus one log line.

PERCEPTION-SIDE ONLY: nothing here is read by the controller.
"""

import math
from typing import Dict, Optional, Tuple

#: The pre-point enter_door aims at first sits this far back on the door axis
#: (enter_door.DEFAULT_APPROACH_DIST_M). The matcher tries the pre-point and
#: the centre, so a goal in either phase finds its door.
PRE_POINT_DIST_M = 0.8

#: A door further than this from the goal [m] (by the nearer of the two
#: candidate points) is not "the door this goal is for".
MATCH_MAX_M = 1.5


def wrap_pi(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


def goal_odom_to_map(goal_odom: Tuple[float, float, float],
                     tf_map_odom: Tuple[float, float, float]
                     ) -> Tuple[float, float, float]:
    """The goal exactly as the MPC uses it (odom), expressed in map through
    the live map -> odom edge (x, y, yaw). Same maths as MPC_corr's
    _pose_odom_to_map, repeated here so this module stays import-light."""
    x, y, psi = goal_odom
    tx, ty, tyaw = tf_map_odom
    c, s = math.cos(tyaw), math.sin(tyaw)
    return (tx + c * x - s * y, ty + s * x + c * y, wrap_pi(psi + tyaw))


def door_axis_yaw(normal_x: float, normal_y: float) -> float:
    """Door.normal's bearing -- the crossing direction, enter_door's tyaw."""
    return math.atan2(normal_y, normal_x)


def match_door(goal_map_xy: Tuple[float, float],
               doors: Dict[int, Tuple[Tuple[float, float], float]],
               approach_dist_m: float = PRE_POINT_DIST_M,
               max_m: float = MATCH_MAX_M):
    """(door_id, kind, expected_xy, door_yaw) of the door this goal belongs to.

    `doors` is {id: ((cx, cy), yaw)} in the map frame. `kind` is 'centre' or
    'pre_point': which of the two points enter_door publishes is nearer to the
    goal. None when no door is within max_m.
    """
    best = None
    for door_id, ((cx, cy), yaw) in doors.items():
        pre = (cx - math.cos(yaw) * approach_dist_m,
               cy - math.sin(yaw) * approach_dist_m)
        for kind, pt in (('centre', (cx, cy)), ('pre_point', pre)):
            d = math.hypot(goal_map_xy[0] - pt[0], goal_map_xy[1] - pt[1])
            if d <= max_m and (best is None or d < best[0]):
                best = (d, door_id, kind, pt, yaw)
    if best is None:
        return None
    return best[1], best[2], best[3], best[4]


def gap(goal_map: Tuple[float, float, float], expected_xy: Tuple[float, float],
        door_yaw: float) -> Tuple[float, float]:
    """(metres, signed degrees) between the goal the MPC drives at (in map)
    and the door as detected NOW. Degrees are wrapped to (-180, 180]; a value
    near +-180 is an axis flip, not a small error."""
    dist = math.hypot(goal_map[0] - expected_xy[0], goal_map[1] - expected_xy[1])
    ddeg = math.degrees(wrap_pi(goal_map[2] - door_yaw))
    return dist, ddeg


def evaluate(goal_odom: Tuple[float, float, float],
             tf_map_odom: Tuple[float, float, float],
             doors: Dict[int, Tuple[Tuple[float, float], float]],
             honour_yaw: bool = True) -> Optional[dict]:
    """Everything the markers and the log line need, or None with no door."""
    goal_map = goal_odom_to_map(goal_odom, tf_map_odom)
    hit = match_door((goal_map[0], goal_map[1]), doors)
    if hit is None:
        return None
    door_id, kind, expected_xy, door_yaw = hit
    dist, ddeg = gap(goal_map, expected_xy, door_yaw)
    return {'door_id': door_id, 'kind': kind, 'expected_xy': expected_xy,
            'door_yaw': door_yaw, 'goal_map': goal_map, 'dist_m': dist,
            'dyaw_deg': ddeg, 'honour_yaw': honour_yaw}
