"""The planner-controller modes: which node turns a mission move into /drive.

One stack-wide switch, ``planner_controller_mode`` (stack_params.yaml), picks
the process that backs the ackermann_mux "navigation" lane when enable_nav2 is
false. All three speak the same contract with the behaviour tree: the goal
topics PublishMoveGoal / GoToObject / EnterDoor publish (/mpc/goal_distance,
/mpc/goal_pose, /mpc/goal_turn, /mpc/goal_drive, /mpc/goal_object,
/mpc/goal_object_end, /mpc/hold, /mpc/stop_target) and the feedback the stop
conditions read (/mpc/goal_reached, /mpc/object_status,
/mpc/min_obstacle_distance, /mpc/min_obstacle_distance_forward). A mission
file therefore runs unchanged in every mode; only the planner and the
controller behind the topics differ.

  corridor_mpc   mpc_corr (mpc_controller/MPC_corr.py): the corridor is the
                 MPC's soft reference, the OSQP solve is the controller. The
                 default, and what every campaign so far ran.
  corridor_rrt   corridor_rrt (f1tenth_planners): the SAME corridor geometry
                 (corridor_geometry.py), RRT* inside it around the obstacles
                 and wall tracks, a pure-pursuit tracker on the result. The
                 RRT takes the MPC's place.
  p2p_geometric  p2p_geometric (f1tenth_planners): no corridor. A Dubins /
                 turn-then-straight path from the car to the target, tracked
                 by the same pure pursuit; obstacles slow and stop the car
                 rather than being planned around.

Lives here, in the dependency-free leaf package, because two consumers that
must agree on it do not depend on each other: f1tenth_navigation's launch
file (which node to start) and f1tenth_behavior's mission preflight (which
node must be up before a mission may start).
"""

from f1tenth_params.param_defaults import get_value

#: The accepted values, default first.
PLANNER_CONTROLLER_MODES = ('corridor_mpc', 'corridor_rrt', 'p2p_geometric')

#: The ROS node name that drives the navigation lane in each mode. The
#: preflight looks this name up in the graph.
CONTROLLER_NODE_NAMES = {
    'corridor_mpc': 'mpc_corr',
    'corridor_rrt': 'corridor_rrt',
    'p2p_geometric': 'p2p_geometric',
}


def validate_planner_controller_mode(mode) -> str:
    """Return ``mode`` normalised (stripped, lower case), or raise ValueError."""
    value = str(mode).strip().lower()
    if value not in PLANNER_CONTROLLER_MODES:
        raise ValueError(
            f'planner_controller_mode must be one of {PLANNER_CONTROLLER_MODES}, '
            f'got {mode!r}')
    return value


def controller_node_name(mode) -> str:
    """Return the node name that drives the car in ``mode``."""
    return CONTROLLER_NODE_NAMES[validate_planner_controller_mode(mode)]


def default_planner_controller_mode() -> str:
    """Return stack_params.yaml's planner_controller_mode, validated."""
    return validate_planner_controller_mode(get_value('planner_controller_mode'))
