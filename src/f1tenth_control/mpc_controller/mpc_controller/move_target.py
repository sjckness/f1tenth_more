"""
The current object move's TARGET, which is never an obstacle. Pure Python, no ROS.

A go_to_object move drives TO an object, so mpc_corr leaves that object out of
everything that keeps the car away from things:

- the obstacle list the solver's w_obs and compute_local_target's deflection
  read (live /perception/obstacles_2d circles and remembered semantic objects);
- the reverse manoeuvre's rear check (RearEnv.objects) and the front distance
  its stuck detection reads, so the target can neither make the car "stuck"
  nor block a reverse.

Live and remembered alike: a person the detector loses close in is neither
steered around nor grown by the memory's person growth. What still stops the
car at the target is the approach's own stop latch, and the BT's front stops,
which read the sensors directly.

HOW THE TARGET IS RECOGNISED
- A semantic object (/costmap/semantic_objects, map frame) by its track id
  when the ObjectGoal carries one (ObjectGoal.track_id is str(SemanticObject.id),
  go_to_object.py). With no id, by class and a centre within gate_m of the goal
  point.
- A live circle (odom (x, y, r)) has neither id nor class: it is the target
  when its centre lies within gate_m of the goal point. A different object
  that close to the target is left out with it (report, Part B fixes).
"""

from dataclasses import dataclass
import math
from typing import Iterable, List, Optional, Sequence, Tuple

__all__ = [
    'MoveTarget',
    'is_target_object',
    'without_target_circles',
    'without_target_objects',
]


@dataclass(frozen=True)
class MoveTarget:
    """The active object move's target, as mpc_corr holds it."""

    track_id: str                              # '' when the ObjectGoal carried none
    class_id: str
    map_xy: Tuple[float, float]                # the goal point, map frame
    odom_xy: Optional[Tuple[float, float]]     # reprojected this tick; None before the first


def is_target_object(obj, target: Optional[MoveTarget], gate_m: float) -> bool:
    """Tell whether a SemanticObject-like `obj` (id, class_id, x, y in map) is the target."""
    if target is None:
        return False
    if target.track_id:
        return str(obj.id) == target.track_id
    return (str(obj.class_id) == target.class_id
            and math.hypot(float(obj.x) - target.map_xy[0],
                           float(obj.y) - target.map_xy[1]) <= float(gate_m))


def without_target_objects(objects: Iterable, target: Optional[MoveTarget],
                           gate_m: float) -> list:
    """Return the semantic objects that are not the target."""
    return [o for o in objects if not is_target_object(o, target, gate_m)]


def without_target_circles(circles: Sequence[Tuple[float, float, float]],
                           target: Optional[MoveTarget],
                           gate_m: float) -> List[Tuple[float, float, float]]:
    """
    Return the odom (x, y, r) circles whose centre is not within gate_m of the target.

    The input list itself when there is no target (or no odom point yet), so a
    tick outside an object move sees exactly the live list.
    """
    if target is None or target.odom_xy is None:
        return circles
    tx, ty = target.odom_xy
    return [c for c in circles if math.hypot(c[0] - tx, c[1] - ty) > float(gate_m)]
