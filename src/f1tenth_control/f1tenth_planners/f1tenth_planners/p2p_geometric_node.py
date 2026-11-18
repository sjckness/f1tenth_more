"""p2p_geometric: no corridor, a geometric point-to-point path and pure pursuit.

planner_controller_mode 'p2p_geometric'. Every replan draws the path from the
car to the move's target out of the car's own turning geometry (dubins.py),
with R = mpc_wheelbase_m / tan(steering bound):

* target without an end heading (goal_pose, the object's standoff point):
  turn-then-straight, the shortest forward path to a point;
* target with one (goal_distance's line end, a turn's synthetic pose, a drive
  move's rolling point): the shortest Dubins path to the pose -- unless that
  needs a loop (it is longer than the free-heading path by more than R), in
  which case the free-heading path is taken. A car a few centimetres off its
  line near the end of a move must not circle to fix its heading.

There is nothing to plan around: obstacles and wall tracks only cap the speed.
The first blocked point along the path within p2p_obstacle_lookahead_m slows
the car (the operating floor min_moving_speed_mps still holds while it moves)
and at p2p_obstacle_stop_m stops it, until the path clears or the mission's
own stop condition (obstacle_distance_below, a timeout) decides. The hard
stop (hard_stop_node) is underneath, as for every mode.
"""

import math

from f1tenth_params.param_defaults import get_value
from f1tenth_planners import dubins
from f1tenth_planners.base_node import PlannerControllerNode, relaxed_obstacles, run_node
from f1tenth_planners.geometry import Obstacles
from f1tenth_planners.moves import PlanTarget
import numpy as np

#: Sample spacing [m] of the planned path.
_PATH_DS = 0.1


def geometric_path(pose, target: PlanTarget, radius: float) -> np.ndarray:
    """Return the Nx2 path from the car to ``target``; the module docstring says which."""
    free = dubins.to_point(pose, (target.x, target.y), radius)
    segments = free
    if target.yaw is not None:
        fixed = dubins.to_pose(pose, (target.x, target.y, target.yaw), radius)
        if fixed is not None and dubins.path_length(fixed) <= dubins.path_length(free) + radius:
            segments = fixed
    return dubins.integrate(pose, segments, radius, ds=_PATH_DS)[:, :2]


def blocked_cap(obstacles: Obstacles, path: np.ndarray, s_car: float, lookahead: float,
                stop: float, cruise: float) -> float:
    """Speed cap from the first blocked path point ahead: linear from lookahead to stop."""
    s_block = obstacles.first_blocked_arclength(path, s_car, lookahead)
    if not math.isfinite(s_block):
        return math.inf
    gap = s_block - s_car
    if gap <= stop:
        return 0.0
    return cruise * (gap - stop) / max(lookahead - stop, 1e-6)


class P2pGeometricNode(PlannerControllerNode):
    """The p2p_geometric planner-controller node."""

    REPLAN_WHEN_BLOCKED = False

    def __init__(self, **node_kwargs):
        super().__init__('p2p_geometric', **node_kwargs)
        self.obstacle_lookahead = float(self.declare_parameter(
            'obstacle_lookahead_m', get_value('p2p_obstacle_lookahead_m')).value)
        self.obstacle_stop = float(self.declare_parameter(
            'obstacle_stop_m', get_value('p2p_obstacle_stop_m')).value)
        self._blocked_logged = False
        self.get_logger().info(
            f'p2p_geometric | turn radius {self.r_min:.2f} m, obstacles slow from '
            f'{self.obstacle_lookahead:.2f} m and stop at {self.obstacle_stop:.2f} m')

    def plan(self, pose, target, obstacles):
        """Turn-then-straight or Dubins path to the target."""
        return geometric_path(pose, target, self.r_min)

    def blocked_speed_cap(self, pose, obstacles, s_car):
        """Slow, then stop, for the first blocked point on the path ahead."""
        if self.path is None or not len(obstacles):
            return math.inf
        relaxed = relaxed_obstacles(obstacles, [pose[:2]])
        cap = blocked_cap(relaxed, self.path, s_car, self.obstacle_lookahead,
                          self.obstacle_stop, self.cruise_speed)
        blocked = cap == 0.0
        if blocked != self._blocked_logged:
            self.get_logger().info('P2P | path blocked: stopped' if blocked
                                   else 'P2P | path clear: driving')
            self._blocked_logged = blocked
        return cap


def main(args=None):
    """Spin p2p_geometric."""
    run_node(P2pGeometricNode, args)
