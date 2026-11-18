"""RRT* in the plane, with the corridor as its sampling region: corridor_rrt's planner.

Pure: numpy and math only. The search space and the obstacles come in as three
callables, so the same planner runs inside a corridor (corridor_rrt) or in any
other region a test hands it:

    sample(rng)          -> (x, y)   a candidate point in the region
    point_free(x, y)     -> bool     the point is in the region and clear
    segment_free(a, b)   -> bool     the straight edge a-b is in the region and clear

WHAT IT PLANS. A geometric path for the car's reference point (base_link). It
is not kinodynamic; two things keep the result drivable by pure pursuit:

* the ROOT CONE: an edge that leaves the car must point within ``root_cone``
  of its heading. An Ackermann car cannot start a path sideways, and without
  this the tree happily begins with a 90 degree corner at the car.
* SHORTCUTTING: the raw tree path zig-zags at the step size; greedy
  shortcutting (furthest visible vertex first, root cone respected) leaves the
  few straight legs a tracker can follow, and the tracker's lookahead rounds
  their corners. Clearance is the obstacles' inflation, which the caller sets.

THE BUDGET. One plan runs inside a 0.1 s control tick, so it is bounded by
``time_budget_s`` as well as ``max_iterations``. Three things keep it inside:

* an unobstructed straight line to the goal is returned at once, without
  growing a tree: in an empty corridor that is the plan, and the common case;
* every new node also tries the straight line to the goal, not only nodes
  within one step of it (as RRT-Connect does), so a solution exists as soon
  as the tree has a node that sees past the obstacle;
* after the first solution only ``refine_iterations`` more are spent
  rewiring: the shortcut pass straightens the result anyway, and a
  near-optimal tree is not worth a late control tick.
"""

from dataclasses import dataclass
import math
import time
from typing import Callable, Optional

from f1tenth_planners.geometry import wrap_pi
import numpy as np


@dataclass
class RRTConfig:
    """RRT* settings; the defaults are stack_params.yaml's rrt_* values."""

    max_iterations: int = 1500
    time_budget_s: float = 0.08
    step: float = 0.3
    goal_bias: float = 0.15
    rewire_radius: float = 0.8
    root_cone: float = math.radians(50.0)
    seed: Optional[int] = None
    refine_iterations: int = 100


@dataclass
class RRTResult:
    """A plan (None when none was found) and the tree that produced it."""

    path: Optional[np.ndarray]
    nodes: np.ndarray
    parents: np.ndarray
    iterations: int
    solved: bool
    direct: bool = False

    @property
    def cost(self) -> float:
        """Length of the returned path (inf without one)."""
        if self.path is None:
            return math.inf
        return float(np.sum(np.hypot(np.diff(self.path[:, 0]), np.diff(self.path[:, 1]))))

    def tree_edges(self):
        """Every (parent, child) edge as an Nx4 array, for visualisation."""
        idx = np.nonzero(self.parents >= 0)[0]
        if len(idx) == 0:
            return np.zeros((0, 4))
        return np.column_stack([self.nodes[self.parents[idx]], self.nodes[idx]])


def _within_cone(start_xy, start_yaw, point, cone) -> bool:
    dx, dy = point[0] - start_xy[0], point[1] - start_xy[1]
    if math.hypot(dx, dy) < 1e-9:
        return True
    return abs(wrap_pi(math.atan2(dy, dx) - start_yaw)) <= cone


def rrt_star(start_xy, start_yaw: float, goal_xy,
             sample: Callable, point_free: Callable, segment_free: Callable,
             config: RRTConfig = RRTConfig(),
             clock: Callable[[], float] = time.perf_counter) -> RRTResult:
    """Plan from start_xy (heading start_yaw) to goal_xy; see the module docstring."""
    start = np.asarray(start_xy, dtype=float)
    goal = np.asarray(goal_xy, dtype=float)
    cone = float(config.root_cone)

    def edge_ok(i_parent, a, b):
        if i_parent == 0 and not _within_cone(start, start_yaw, b, cone):
            return False
        return segment_free(a, b)

    cap = int(config.max_iterations) + 2
    nodes = np.zeros((cap, 2))
    parents = np.full(cap, -1, dtype=int)
    cost = np.zeros(cap)
    nodes[0] = start
    n = 1

    if edge_ok(0, start, goal):
        return RRTResult(path=np.vstack([start, goal]), nodes=nodes[:1].copy(),
                         parents=parents[:1].copy(), iterations=0, solved=True,
                         direct=True)

    rng = np.random.default_rng(config.seed)
    t_end = clock() + float(config.time_budget_s)
    step = float(config.step)
    radius = float(config.rewire_radius)
    best_goal_parent, best_goal_cost = -1, math.inf
    iterations = 0
    stop_at = int(config.max_iterations)

    for iterations in range(1, int(config.max_iterations) + 1):
        if clock() > t_end or iterations > stop_at:
            break
        if rng.random() < config.goal_bias:
            target = goal
        else:
            target = np.asarray(sample(rng), dtype=float)
        d2 = np.sum((nodes[:n] - target) ** 2, axis=1)
        i_near = int(np.argmin(d2))
        dist = math.sqrt(float(d2[i_near]))
        if dist < 1e-9:
            continue
        if dist > step:
            new = nodes[i_near] + (target - nodes[i_near]) * (step / dist)
        else:
            new = target.copy()
        if not point_free(float(new[0]), float(new[1])):
            continue

        # Choose the cheapest reachable parent in the neighbourhood.
        dn = np.sqrt(np.sum((nodes[:n] - new) ** 2, axis=1))
        near = np.nonzero(dn <= radius)[0]
        if i_near not in near:
            near = np.append(near, i_near)
        order = near[np.argsort(cost[near] + dn[near])]
        parent = -1
        for j in order:
            if edge_ok(int(j), nodes[j], new):
                parent = int(j)
                break
        if parent < 0:
            continue
        k = n
        nodes[k] = new
        parents[k] = parent
        cost[k] = cost[parent] + dn[parent]
        n += 1

        # Rewire the neighbourhood through the new node where that is cheaper.
        for j in near:
            j = int(j)
            if j == 0 or j == parent:
                continue
            c_new = cost[k] + dn[j]
            if c_new + 1e-9 < cost[j] and segment_free(new, nodes[j]):
                parents[j] = k
                cost[j] = c_new

        d_goal = float(np.hypot(*(goal - new)))
        if cost[k] + d_goal < best_goal_cost and segment_free(new, goal):
            if best_goal_parent < 0:
                stop_at = iterations + int(config.refine_iterations)
            best_goal_parent, best_goal_cost = k, cost[k] + d_goal

        if n >= cap:
            break

    nodes, parents = nodes[:n].copy(), parents[:n].copy()
    if best_goal_parent < 0:
        return RRTResult(path=None, nodes=nodes, parents=parents,
                         iterations=iterations, solved=False)

    chain = [goal]
    i = best_goal_parent
    guard = 0
    while i >= 0 and guard <= n:
        chain.append(nodes[i])
        i = int(parents[i])
        guard += 1
    raw = np.array(chain[::-1])
    path = shortcut(raw, start_yaw, segment_free, cone)
    return RRTResult(path=path, nodes=nodes, parents=parents,
                     iterations=iterations, solved=True)


def shortcut(path: np.ndarray, start_yaw: float, segment_free: Callable,
             root_cone: float = math.pi) -> np.ndarray:
    """Greedy visibility shortcut: from each kept vertex jump to the furthest visible one."""
    path = np.asarray(path, dtype=float)
    if len(path) <= 2:
        return path.copy()
    out = [path[0]]
    i = 0
    while i < len(path) - 1:
        j_best = i + 1
        for j in range(len(path) - 1, i + 1, -1):
            if i == 0 and not _within_cone(path[0], start_yaw, path[j], root_cone):
                continue
            if segment_free(path[i], path[j]):
                j_best = j
                break
        out.append(path[j_best])
        i = j_best
    return np.array(out)


def corridor_sampler(corridor: dict, shrink: float = 0.95):
    """Return a uniform sampler over a corridor: a centreline sample plus a lateral offset."""
    xc = np.asarray(corridor['xc'], dtype=float)
    yc = np.asarray(corridor['yc'], dtype=float)
    nx = np.asarray(corridor['nx'], dtype=float)
    ny = np.asarray(corridor['ny'], dtype=float)
    hw = np.asarray(corridor['halfWidth'], dtype=float) * float(shrink)
    count = len(xc)

    def sample(rng):
        i = int(rng.integers(0, count))
        lat = float(rng.uniform(-hw[i], hw[i]))
        return xc[i] + lat * nx[i], yc[i] + lat * ny[i]

    return sample
