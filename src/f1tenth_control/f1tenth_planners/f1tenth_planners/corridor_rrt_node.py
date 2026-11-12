"""corridor_rrt: mpc_corr's corridor, with RRT* in the MPC's place.

planner_controller_mode 'corridor_rrt'. The corridor is built with the same
geometry module mpc_corr uses (f1tenth_params.corridor_geometry), from the car
to the move's target, every replan:

* a target with an end heading (goal_distance's line end, a turn's synthetic
  pose, a drive move's rolling point): corridor_curves_to_pose(), the
  boundary-value corridor that ends on the pose;
* a target without one (goal_pose, the object's standoff point): the same,
  ending on the car-to-target bearing.

Half-widths are mpc_corr's: corr_half_width_start_m at the car, times
CORRIDOR_WIDTH_RATIO at the far end. The RRT* (rrt.py) then samples inside
that corridor, keeps the car (inflated by car_radius + obstacle_safety_margin_m)
clear of the live obstacle circles and the LiDAR wall tracks, and returns a
shortcut path that pure pursuit tracks. Where mpc_corr bends a soft reference
around an obstacle with a cost, this plans an explicit detour inside the same
corridor; where the corridor is blocked from wall to wall, no path exists and
the car stops and replans until one does.
"""

import math

from f1tenth_params.corridor_geometry import corridor_curves_to_pose, CORRIDOR_WIDTH_RATIO
from f1tenth_params.param_defaults import get_value
from f1tenth_planners.base_node import PlannerControllerNode, relaxed_obstacles, run_node
from f1tenth_planners.geometry import corridor_polygon, Obstacles, PreparedPolygon
from f1tenth_planners.moves import PlanTarget
from f1tenth_planners.rrt import corridor_sampler, rrt_star, RRTConfig
import numpy as np
from visualization_msgs.msg import Marker

#: Points this close to the car or to the target count as inside the
#: corridor: its start edge passes through the car and its end edge through
#: the target, and a polygon test calls a point on an edge ambiguous.
_END_SLACK_M = 0.3
#: Spacing [m] of the containment samples along an edge.
_EDGE_SAMPLE_M = 0.1
#: Below this distance to the target no corridor is built: a straight leg.
_MIN_CORRIDOR_M = 0.35
#: Corridor samples (mpc_corr uses 120 for drawing; 60 is plenty to sample in).
_CORRIDOR_SAMPLES = 60


def build_corridor(pose, target: PlanTarget, half_width_start: float, r_min: float):
    """Build the corridor (corridor_geometry keys) from the car to ``target``, or None."""
    x, y, yaw = pose
    dist = math.hypot(target.x - x, target.y - y)
    if dist < _MIN_CORRIDOR_M:
        return None
    psi_end = target.yaw if target.yaw is not None else math.atan2(target.y - y, target.x - x)
    return corridor_curves_to_pose(
        x, y, yaw, target.x, target.y, psi_end, _CORRIDOR_SAMPLES,
        w0=half_width_start, w1=half_width_start * CORRIDOR_WIDTH_RATIO, r_min=r_min)


def plan_in_corridor(pose, target: PlanTarget, obstacles: Obstacles, corridor,
                     config: RRTConfig):
    """RRT* from the car to the target inside ``corridor``; returns (RRTResult, polygon)."""
    start = np.array(pose[:2], dtype=float)
    goal = np.array([target.x, target.y], dtype=float)
    relaxed = relaxed_obstacles(obstacles, [start, goal])
    polygon = corridor_polygon(corridor)
    prepared = PreparedPolygon(polygon)

    def inside(xs, ys):
        xs = np.atleast_1d(xs)
        ys = np.atleast_1d(ys)
        near_ends = ((np.hypot(xs - start[0], ys - start[1]) <= _END_SLACK_M)
                     | (np.hypot(xs - goal[0], ys - goal[1]) <= _END_SLACK_M))
        return near_ends | prepared.contains(xs, ys)

    def point_free(px, py):
        return bool(inside(px, py)[0]) and relaxed.point_free(px, py)

    def segment_free(a, b):
        length = math.hypot(b[0] - a[0], b[1] - a[1])
        k = max(int(math.ceil(length / _EDGE_SAMPLE_M)), 1)
        t = np.linspace(0.0, 1.0, k + 1)[1:]
        if not np.all(inside(a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1]))):
            return False
        return relaxed.segment_free(a, b)

    result = rrt_star(start, pose[2], goal, corridor_sampler(corridor), point_free,
                      segment_free, config)
    return result, polygon


class CorridorRrtNode(PlannerControllerNode):
    """The corridor_rrt planner-controller node."""

    def __init__(self, **node_kwargs):
        super().__init__('corridor_rrt', **node_kwargs)

        def p(name, key=None):
            return self.declare_parameter(name, get_value(key or name)).value

        seed = int(p('rrt_seed'))
        self.rrt_config = RRTConfig(
            max_iterations=int(p('rrt_max_iterations')),
            time_budget_s=float(p('rrt_time_budget_s')),
            step=float(p('rrt_step_m')),
            goal_bias=float(p('rrt_goal_bias')),
            rewire_radius=float(p('rrt_rewire_radius_m')),
            root_cone=math.radians(float(p('rrt_root_cone_deg'))),
            seed=None if seed < 0 else seed)
        self.half_width_start = float(p('corr_half_width_start_m'))
        self.drive_length = float(p('rrt_corridor_length_drive_m'))
        self.corridor = None
        self.polygon = None
        self.last_result = None
        self.get_logger().info(
            f'corridor_rrt | corridor half-width {self.half_width_start:.2f} -> '
            f'{self.half_width_start * CORRIDOR_WIDTH_RATIO:.2f} m, RRT* step '
            f'{self.rrt_config.step:.2f} m, budget {self.rrt_config.time_budget_s * 1e3:.0f} ms')

    def drive_horizon(self) -> float:
        """Return an open-ended drive's corridor length (rrt_corridor_length_drive_m)."""
        return self.drive_length

    def plan(self, pose, target, obstacles):
        """Corridor from the car to the target, then RRT* inside it."""
        corridor = build_corridor(pose, target, self.half_width_start, self.r_min)
        if corridor is None:
            self.corridor, self.polygon, self.last_result = None, None, None
            return np.array([pose[:2], [target.x, target.y]], dtype=float)
        result, polygon = plan_in_corridor(pose, target, obstacles, corridor, self.rrt_config)
        self.corridor, self.polygon, self.last_result = corridor, polygon, result
        if not result.solved:
            self.get_logger().warn(
                f'RRT* | no path in the corridor after {result.iterations} iterations, '
                f'{len(result.nodes)} nodes', throttle_duration_sec=1.0)
            return None
        self.get_logger().info(
            f'RRT* | {"direct" if result.direct else "tree"} path {result.cost:.2f} m, '
            f'{len(result.path)} vertices, {result.iterations} iterations, '
            f'corridor {corridor["length"]:.2f} m', throttle_duration_sec=0.5)
        return result.path

    def extra_markers(self, stamp):
        """Corridor walls, centreline and the RRT* tree."""
        out = []
        c = self.corridor
        if c is not None:
            out.append(self.line_marker(1, 'corridor_left', np.column_stack([c['xL'], c['yL']]),
                                        stamp, (0.2, 0.5, 1.0, 0.9), 0.03))
            out.append(self.line_marker(2, 'corridor_right', np.column_stack([c['xR'], c['yR']]),
                                        stamp, (0.2, 0.5, 1.0, 0.9), 0.03))
            out.append(self.line_marker(3, 'corridor_centre', np.column_stack([c['xc'], c['yc']]),
                                        stamp, (0.6, 0.6, 0.6, 0.6), 0.015))
        r = self.last_result
        if r is not None and not r.direct:
            edges = r.tree_edges()[:800]
            pts = edges.reshape(-1, 2)
            out.append(self.line_marker(4, 'rrt_tree', pts, stamp, (1.0, 0.6, 0.1, 0.35),
                                        0.01, kind=Marker.LINE_LIST))
        return out


def main(args=None):
    """Spin corridor_rrt."""
    run_node(CorridorRrtNode, args)
