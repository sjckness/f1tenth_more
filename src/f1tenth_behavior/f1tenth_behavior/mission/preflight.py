"""Mission-start liveness preflight -- what a mission actually needs alive
before /mission/start_mission is allowed to transition LOADED -> RUNNING.

Motivated directly by a race found this session: the launch-time VESC
battery precheck (f1tenth_hardware/launch/vesc.launch.py +
battery_voltage_check_node.py) gates whether ackermann_to_vesc_node's whole
launch group even starts. When that race went the wrong way,
ackermann_to_vesc_node ended up silently absent from the ROS graph while
mpc_corr, the ackermann_mux, and the EKF all came up and looked completely
healthy -- there was no signal anywhere in this stack that the car
physically could not move until a mission was started and nothing happened.
/mission/start_mission's own check today is `state == LOADED` and nothing
else (see mission/loader.py's own docstring) -- a service-call-level check,
not a "can this mission's dependencies actually do their job" one. This
module is that check.

Split the same way mission_config.py/condition_eval.py already are:
`required_dependencies()` is pure (a MissionConfig in, a list of
Requirement out) and independently testable with zero rclpy dependency;
`check_liveness()` is the ROS-facing half a caller with a live node/
blackboard runs the result against. mission/loader.py is that caller.

Deliberately NOT a new subscribe-and-wait-N-seconds mechanism: that would
change /mission/start_mission's latency/blocking behavior, which wasn't
asked for and wasn't confirmed. Instead this reuses two things that already
exist:
  - `node.get_node_names()` -- a direct ROS graph query, no new
    subscription, answers "is a node by this exact name currently up" (the
    ackermann_to_vesc_node race above is exactly this kind of failure: the
    node never existed at all, not merely quiet).
  - The live blackboard values CheckStopCondition (behaviours/check_stop_
    condition.py) already tracks for its own stop_condition evaluation
    (CURRENT_XY_KEY, MIN_OBSTACLE_DISTANCE_KEY, FRONT_CLEARANCE_KEY -- see
    mission/runtime.py for why the key constants live there) -- these are
    None until at least one real message has actually arrived, which is
    "alive AND publishing," not just "alive." Reused here rather than
    re-subscribed, per this pass's own "extend rather than parallel-build"
    instruction.

This does NOT catch a dependency that dies *after* passing preflight and
*before* first use mid-mission -- that is 3.2/3.5's job (closed-loop
per-move verification), a different failure window entirely. It also does
not (yet) close the staleness gap documented in f1tenth_behavior/README.md's
"Dependency failure behavior" section -- a node that was alive and
publishing at start_mission time but died a minute into a long mission
still reads as fine here, by design (this only ever runs once, at start).
"""

from dataclasses import dataclass
from typing import Callable, List, Optional

from f1tenth_behavior.mission.mission_config import MissionConfig
from f1tenth_behavior.mission.runtime import (
    CURRENT_XY_KEY,
    FRONT_CLEARANCE_KEY,
    MIN_OBSTACLE_DISTANCE_KEY,
)


@dataclass(frozen=True)
class Requirement:
    name: str  # human-readable, used in the failure message
    reason: str  # why THIS mission needs it, also used in the failure message
    # Exact ROS node name expected in the graph (node.get_node_names()), or
    # None if this requirement is data-liveness-only (see blackboard_key).
    node_name: Optional[str] = None
    # Blackboard key that must hold a non-None value (i.e. at least one real
    # message has arrived), or None if this requirement is node-existence-only.
    blackboard_key: Optional[str] = None


# Every move type (goal_distance/goal_pose/turn) is driven through mpc_corr
# (see PublishMoveGoal's own docstring) -- always required, unconditionally.
_MPC_CORR = Requirement(
    name='mpc_corr',
    reason='every move type (goal_distance/goal_pose/turn) is driven through it',
    node_name='mpc_corr',
)
# THE specific node a prior battery-precheck launch race let go silently
# missing while mpc_corr/ackermann_mux/EKF all looked healthy -- see module
# docstring. Always required: without it nothing mpc_corr commands ever
# reaches the VESC, for any mission.
_ACKERMANN_TO_VESC = Requirement(
    name='ackermann_to_vesc_node',
    reason=(
        'converts mpc_corr\'s drive command into actual VESC commands -- the exact '
        'node a prior battery-precheck launch race left silently missing while '
        'mpc_corr/ackermann_mux/EKF all looked healthy'
    ),
    node_name='ackermann_to_vesc_node',
)
# CheckStopCondition's own /odom subscription (distance_reached/
# orientation_delta tracking) -- always required, every move needs a
# position/heading to measure progress against.
_LOCALIZATION = Requirement(
    name='localization (/odom)',
    reason="CheckStopCondition's distance/heading tracking needs it for every move",
    blackboard_key=CURRENT_XY_KEY,
)


def required_dependencies(config: MissionConfig) -> List[Requirement]:
    """Pure function: what must be alive for THIS mission, specifically --
    not a blanket "everything the stack could ever need" list. Testable
    without rclpy (see test/test_preflight.py)."""
    reqs = [_MPC_CORR, _ACKERMANN_TO_VESC, _LOCALIZATION]

    stop_condition_types = {m.stop_condition.type for m in config.moves}
    for m in config.moves:
        for oo in m.on_object:
            if oo.params.get('resume_condition') is not None:
                stop_condition_types.add(oo.params['resume_condition'].type)

    if 'front_clearance' in stop_condition_types:
        reqs.append(Requirement(
            name='costmap_boundary_node',
            reason="a 'front_clearance' stop_condition (or resume_condition) is used",
            node_name='costmap_boundary_node',
            blackboard_key=FRONT_CLEARANCE_KEY,
        ))

    if 'obstacle_distance_below' in stop_condition_types:
        # Published by mpc_corr itself (see check_stop_condition.py's own
        # docstring) -- no separate node, just a stronger liveness bar on the
        # one already required above: it must have published this specific
        # value at least once, not merely be present in the graph.
        reqs.append(Requirement(
            name='mpc_corr (/mpc/min_obstacle_distance)',
            reason="an 'obstacle_distance_below' stop_condition (or resume_condition) is used",
            blackboard_key=MIN_OBSTACLE_DISTANCE_KEY,
        ))

    uses_perception = bool(stop_condition_types & {'object_seen', 'object_cleared'}) or any(
        m.on_object for m in config.moves
    )
    if uses_perception:
        reqs.append(Requirement(
            name='yolo_detector_node',
            reason=(
                "an 'object_seen'/'object_cleared' stop_condition, or an on_object "
                'entry, is used'
            ),
            node_name='yolo_detector_node',
        ))

    return reqs


def check_liveness(
    reqs: List[Requirement],
    live_node_names: List[str],
    blackboard_get: Callable[[str], object],
) -> List[str]:
    """Returns a list of human-readable failure messages, one per unmet
    Requirement -- empty means every requirement is currently satisfied.

    live_node_names: node.get_node_names() -- passed in rather than a node
    object so this stays testable without rclpy (see test/test_preflight.py).
    blackboard_get: a `lambda key: getattr(blackboard, key)`-shaped callable,
    same reasoning -- avoids this module depending on py_trees directly.
    """
    live = set(live_node_names)
    failures = []
    for req in reqs:
        if req.node_name is not None and req.node_name not in live:
            failures.append(
                f'{req.name}: node {req.node_name!r} not found in the ROS graph '
                f'(needed because {req.reason})'
            )
            continue  # blackboard_key check below would be redundant/misleading
        if req.blackboard_key is not None and blackboard_get(req.blackboard_key) is None:
            failures.append(
                f'{req.name}: alive but no data received yet on its expected topic '
                f'(needed because {req.reason})'
            )
    return failures
