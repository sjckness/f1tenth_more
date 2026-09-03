"""Owns the outer BT (emergency > obstacle-stop > navigate): builds a
py_trees_ros.trees.BehaviourTree and ticks it forever as an ambient supervisor via
tick_tock() -- this tree is not a one-shot "run until SUCCESS/FAILURE" helper, it runs
for the life of the process, mirroring the old BT.CPP behavior_executor_node's own
wall-timer-driven tickRoot() loop.

Also publishes a live, colored dot-graph of the tree's current status (each
behaviour's SUCCESS/FAILURE/RUNNING/INVALID) as a PNG sensor_msgs/CompressedImage
on /bt/tree_visualization (viewable in Foxglove's Image panel), gated by
enable_bt_visualization -- see make_tree_visualizer/_status_dot_graph below for
why this builds its own dot-graph rather than using py_trees.display's.

Priority structure (root Selector, first child that succeeds wins -- same as the old
safety_stop_and_navigate.xml's ReactiveFallback):
  1. emergency: (IsBatteryLow OR IsEmergencyStopTriggered OR IsProximityTooClose OR
                 IsSystemOverheated) -> Stop
  2. handle_obstacle: IsObstacleDetected -> Stop
  3. mission: MissionActive? -> Selector[on-object response, move progression] -- see
     "Mission subtree" below. Sits above navigation deliberately (see that section).
  4. navigation: enable_nav2 true  -> HasGoalPose -> NavigateThroughPosesClient
                 enable_nav2 false -> HasMpcGoal (mpc_corr.py drives itself, entirely
                 out of band from the BT, once /mpc/goal_distance arrives -- see
                 has_mpc_goal.py's own docstring)

Mission subtree: a scripted sequence of moves (mission/mission_config.py's JSON
schema -- see f1tenth_behavior/README.md's config table, and missions/*.json for
examples), loaded via the mission_file_name parameter (a bare filename under this
package's installed missions/ dir, see mission/loader.py) or the /mission/load_path
topic (mission/loader.py), executed against mpc_corr the same way navigation's HasMpcGoal
lane does (/mpc/goal_distance) but with move-by-move stop conditions, per-move
reactions to detected object classes (mission/condition_eval.py, detected_classes_
bridge.py), and a hold mechanism (/mpc/hold, received by mpc_corr.py) instead of
Nav2/direct-MPC's single-shot goal. Positioned above navigation in the root Selector
(not below) so that whenever a mission is actively RUNNING/HOLDING it wins outright
and navigation's HasMpcGoal -- whose SUCCESS just means "a goal was EVER received",
a permanent latch, see has_mpc_goal.py -- never gets ticked that cycle at all. Both
lanes ultimately publish to the same /mpc/goal_distance; this ordering is what keeps
that a single writer per tick, not any coordination between the two lanes themselves.
See mission_progress's own construction below for why PublishMoveGoal is ticked
before CheckStopCondition/AdvanceMove, not after as literally diagrammed in the
original task -- and check_stop_condition.py/handle_object_action.py/object_seen.py/
mission_active.py's own docstrings for the other deliberate deviations from a fully
literal reading (all flagged there, not silent).

Single goal input: HasGoalPose subscribes to /goal_pose (geometry_msgs/PoseStamped)
and is both the navigation lane's gate (SUCCESS only once a pose has actually been
received, FAILURE otherwise) and the writer of a one-element poses list onto the
shared blackboard key. Before any goal has arrived, it fails, so does handle_obstacle
(no obstacle) and emergency (nothing wrong), the root Selector fails, and the tree
does nothing -- Nav2 never gets sent a goal on its own. NavigateThroughPosesClient
(custom action-client behaviour, not py_trees_ros.action_clients.FromBlackboard -- see
its own docstring for why) only sends a new navigate_through_poses goal when the poses
array actually changed, cancelling any in-flight goal first.

Emergency lane (added after Phase 6's audit flagged that no emergency branch existed
here despite the old BT.CPP XML never having had one either -- this is a deliberate,
new addition, not a restoration): reuses the same generic Stop behaviour handle_obstacle
already uses, publishing onto the same safety_stop mux lane (priority 200, see
f1tenth_bringup/config/mux.yaml) -- the mux doesn't care which BT branch published,
only that something did. IsBatteryLow is unconditional (battery safety is never
gateable). IsSystemOverheated is only constructed and added at all if enable_sys_obs
(f1tenth_params/config/stack_params.yaml) is true, read directly via get_value() here
-- the same single source of truth f1tenth_diagnostics/system_observer.launch.py reads
to decide whether system_observer_node even runs, so both sides of that toggle always
agree. This is why the check happens here, at tree-construction time, rather than
inside IsSystemOverheated.update() checking some "is sys_obs enabled" flag on every
tick: sys_obs disabled must mean the condition is structurally absent, not just always
FAILURE, since "no data received" and "disabled" would otherwise be indistinguishable.
IsEmergencyStopTriggered is unconditional too (same reasoning as IsBatteryLow) --
it reads the latched flag MissionLoader (mission/loader.py) sets via
/mission/emergency_stop, off /mission/status; see that behaviour's own docstring
for why this is a deliberately different, simpler mechanism than the mission
subtree's own /mission/abort_mission (a hardware/operator stop, not mission
control flow -- feeds this lane, not the mission one).

IsProximityTooClose is also unconditional (same reasoning) -- a last-resort
hardware proximity check built directly from raw /scan alone (front cone +
sides/rear, two angular windows over the same LaserScan -- front used to read
raw ZED depth via f1tenth_perception's front_depth_monitor_node, REPLACED by
a LiDAR front-cone check to remove ZED stereo depth's known unreliability
against flat/featureless walls as an e-stop trigger, see is_proximity_too_
close.py's own docstring for the full rationale/trade-off), deliberately
independent of the YOLO/obstacle-array pipeline IsObstacleDetected (the
handle_obstacle lane, below) uses, so it keeps working even if perception/
classification is degraded or lagging. See that behaviour's own docstring
for the two thresholds/zones and the raw-lidar-frame angle convention (angle
0 is physically the car's front, post-remount).

Safety-margin unification pass (following safety_stop_controller's retirement):
IsProximityTooClose's two thresholds and IsObstacleDetected's corridor half-
width/height are no longer hardcoded here -- create_root() derives all of them
from three shared stack_params.yaml keys (car_radius, obstacle_safety_margin_m,
proximity_front_extra_margin_m), bootstrap-read in main() the same way
sys_obs_max_temp_c/sys_obs_max_load_percent already are. See
stack_params.yaml's car_radius comment for the full 4-mechanism picture
(MPC_corr.py's own car_radius/avoidance_margin are the other two consumers)
and why the geometry differs enough that this is a base-value-plus-derived-
formula scheme, not one scalar reused everywhere.
"""

import os
import sys
import time

import py_trees
import py_trees.display
import py_trees_ros
import rclpy

from f1tenth_behavior.behaviours.advance_move import AdvanceMove
from f1tenth_behavior.behaviours.check_stop_condition import CheckStopCondition
from f1tenth_behavior.behaviours.handle_object_action import HandleObjectAction
from f1tenth_behavior.behaviours.has_goal_pose import HasGoalPose
from f1tenth_behavior.behaviours.has_mpc_goal import HasMpcGoal
from f1tenth_behavior.behaviours.is_battery_low import IsBatteryLow
from f1tenth_behavior.behaviours.is_emergency_stop_triggered import IsEmergencyStopTriggered
from f1tenth_behavior.behaviours.is_obstacle_detected import IsObstacleDetected
from f1tenth_behavior.behaviours.is_proximity_too_close import IsProximityTooClose
from f1tenth_behavior.behaviours.is_system_overheated import IsSystemOverheated
from f1tenth_behavior.behaviours.mission_active import MissionActive
from f1tenth_behavior.behaviours.navigate_through_poses_client import (
    NavigateThroughPosesClient,
)
from f1tenth_behavior.behaviours.object_seen import ObjectSeen
from f1tenth_behavior.behaviours.publish_move_goal import PublishMoveGoal
from f1tenth_behavior.behaviours.stop import Stop

from f1tenth_behavior.mission.detected_classes_bridge import DetectedClassesBridge
from f1tenth_behavior.mission.loader import MissionLoader

from f1tenth_params.param_defaults import get_odom_topic, get_value

import pydot
from f1tenth_messages.msg import BehaviorTreeStatus
from sensor_msgs.msg import CompressedImage

GOAL_POSE_KEY = 'goal_pose_goal'


def make_snapshot_logger(min_interval_sec=1.0):
    """Returns a py_trees post_tick_handler that logs a full ascii-tree snapshot
    (every node's SUCCESS/FAILURE/RUNNING/INVALID status) at INFO level, at most
    once every min_interval_sec -- the tree ticks much faster than that
    (bt_loop_duration_ms, default 100ms), so this guard is what keeps the log to
    ~1Hz regardless of tick rate. ascii_tree (not unicode_tree) deliberately: no
    box-drawing/ANSI-colour characters that could turn into garbage in a rosout/
    journald sink that isn't a real TTY.

    Prepends a one-line lane summary (each root child's name=status) ahead of the
    tree dump so it's obvious at a glance which priority lane -- emergency,
    handle_obstacle, or navigation, see module docstring -- is currently active,
    without having to mentally re-derive Selector semantics from the indented tree
    every time.
    """
    state = {'last_log_time': 0.0}

    def _log_snapshot(tree):
        now = time.monotonic()
        if now - state['last_log_time'] < min_interval_sec:
            return
        state['last_log_time'] = now

        root = tree.root
        lane_summary = ' | '.join(
            f'{child.name}={child.status.name}' for child in root.children)
        snapshot = py_trees.display.ascii_tree(root, show_status=True)
        tree.node.get_logger().info(
            f'BT snapshot -- root={root.status.name}, lanes: {lane_summary}\n{snapshot}')

    return _log_snapshot


_STATUS_FILLCOLOR = {
    py_trees.common.Status.SUCCESS: '#4CAF50',  # green
    py_trees.common.Status.FAILURE: '#F44336',  # red
    py_trees.common.Status.RUNNING: '#FFC107',  # amber
    py_trees.common.Status.INVALID: '#BDBDBD',  # gray -- not yet ticked
}


def _shape_for(behaviour: py_trees.behaviour.Behaviour) -> str:
    # Mirrors py_trees.display.dot_tree's own shape convention (see that
    # function's get_node_attributes) so this graph reads the same way to
    # anyone already used to py_trees's own static-structure export.
    if isinstance(behaviour, py_trees.composites.Selector):
        return 'octagon'
    if isinstance(behaviour, py_trees.composites.Sequence):
        return 'box'
    if isinstance(behaviour, py_trees.composites.Parallel):
        return 'parallelogram'
    if isinstance(behaviour, py_trees.decorators.Decorator):
        return 'ellipse'
    return 'ellipse'


def _status_dot_graph(root: py_trees.behaviour.Behaviour) -> pydot.Dot:
    """Build a pydot graph of the live tree, colored by each behaviour's
    current .status -- NOT py_trees.display.dot_tree()/render_dot_tree(): the
    installed py_trees version (2.4.0, checked via its own source before
    assuming otherwise) colors nodes by TYPE (Selector/Sequence/Decorator),
    not by tick status, and has no status-colored export in its public API.
    Post-processing dot_tree()'s own pydot.Dot output was considered and
    rejected -- it de-duplicates same-named nodes internally (this tree has
    two behaviours both named "Stop") via a local variable not exposed to
    callers, so matching graph nodes back to live behaviours from outside
    that function can't be done reliably. Building the (small, ~40-line)
    graph directly from the live tree via .children/.status/.id sidesteps
    that entirely -- every Behaviour (leaf, composite, or decorator) exposes
    all three uniformly (confirmed in Behaviour.__init__/Decorator.__init__),
    so this recursion needs no per-type special-casing beyond the shape
    chosen for display.

    RUNNING edges (parent -> a child currently RUNNING) are bolded/colored
    orange, so the tree's current execution path is visible at a glance --
    the most useful thing to see live, since a Selector/Sequence's own
    status alone doesn't say which child put it there.
    """
    graph = pydot.Dot(graph_type='digraph', ordering='out')
    graph.set_graph_defaults(fontname='times-roman', bgcolor='white')
    graph.set_node_defaults(fontname='times-roman')
    graph.set_edge_defaults(fontname='times-roman')

    def add(behaviour: py_trees.behaviour.Behaviour) -> None:
        graph.add_node(pydot.Node(
            name=str(behaviour.id),
            label=f'{behaviour.name}\n{behaviour.status.name}',
            shape=_shape_for(behaviour),
            style='filled',
            fillcolor=_STATUS_FILLCOLOR.get(behaviour.status, '#FFFFFF'),
            fontsize=9,
        ))
        for child in behaviour.children:
            add(child)
            running = child.status == py_trees.common.Status.RUNNING
            graph.add_edge(pydot.Edge(
                str(behaviour.id), str(child.id),
                color='orange' if running else 'black',
                penwidth='3' if running else '1',
            ))

    add(root)
    return graph


def make_tree_visualizer(node, publisher, min_interval_sec=0.4):
    """Returns a py_trees post_tick_handler that renders the tree's live
    status (see _status_dot_graph) straight to PNG bytes in memory (pydot's
    create_png(), no temp file) and publishes it as a sensor_msgs/
    CompressedImage (format='png') on `publisher`.

    Throttled to at most once every min_interval_sec, independent of the
    tree's own tick rate (bt_loop_duration_ms, default 100ms) -- same
    stored-last-publish-time pattern as make_snapshot_logger above, so
    graphviz layout/PNG-render cost never adds latency to BT ticking itself.
    """
    state = {'last_publish_time': 0.0}

    def _visualize(tree):
        now = time.monotonic()
        if now - state['last_publish_time'] < min_interval_sec:
            return
        state['last_publish_time'] = now

        try:
            png_bytes = _status_dot_graph(tree.root).create_png()
        except Exception as exc:  # noqa: BLE001 - never let visualization crash the BT
            node.get_logger().warn(
                f'BT visualization render failed: {exc!r}', throttle_duration_sec=5.0)
            return

        msg = CompressedImage()
        msg.header.stamp = node.get_clock().now().to_msg()
        msg.format = 'png'
        msg.data = png_bytes
        publisher.publish(msg)

    return _visualize


def make_tree_status_publisher(node, publisher):
    """Returns a py_trees post_tick_handler publishing f1tenth_messages/
    BehaviorTreeStatus on /behavior/tree_status, once per tick.

    Unthrottled, deliberately -- unlike make_snapshot_logger/make_tree_visualizer
    above (both of which are throttled because ascii-tree formatting and
    graphviz PNG rendering are expensive), this builds a handful of short
    strings. At the default 10 Hz tick rate that is far cheaper than the BT
    snapshot log it replaces for analysis purposes, and per-tick resolution is
    the entire point: a stop lasting 0.9 s (the observed duration of a
    single-sample IsSystemOverheated trip) spans ~9 ticks, and a 1 Hz throttle
    could easily miss it altogether.

    WHY THIS EXISTS: see BehaviorTreeStatus.msg. Which lane won, and which
    emergency condition tripped, previously lived only in the per-component
    supervisor log, which is overwritten on every supervisor restart -- during
    the 2026-09-01 analysis it covered only the window after the last mission,
    so all nine stop episodes had to be re-bucketed by replaying each
    condition's predicate against recorded sensor data.
    """
    def _publish(tree):
        root = tree.root
        msg = BehaviorTreeStatus()
        msg.header.stamp = node.get_clock().now().to_msg()

        lane_names = []
        lane_statuses = []
        active_lane = ''
        for child in root.children:
            lane_names.append(child.name)
            lane_statuses.append(child.status.name)
            # First succeeding child is the one a memory=False Selector stopped
            # at -- i.e. the lane that actually ran this tick.
            if not active_lane and child.status == py_trees.common.Status.SUCCESS:
                active_lane = child.name
        msg.lane_names = lane_names
        msg.lane_statuses = lane_statuses
        msg.active_lane = active_lane

        # Which emergency child tripped. Walked by name so this keeps working
        # when the emergency lane's composition changes with the
        # enable_lidar_safety_stop/enable_sys_obs flags.
        emergency_trip = ''
        for child in root.children:
            if child.name != 'emergency':
                continue
            for cond in child.children:
                if cond.name != 'emergency_condition':
                    continue
                for leaf in cond.children:
                    if leaf.status == py_trees.common.Status.SUCCESS:
                        # tripped_reason is optional -- only IsSystemOverheated
                        # currently exposes it (see that behaviour).
                        reason = getattr(leaf, 'tripped_reason', '')
                        emergency_trip = (
                            f'{leaf.name}: {reason}' if reason else leaf.name)
                        break
        msg.emergency_trip = emergency_trip

        # stop_source mirrors the frame_id the corresponding Stop stamps, so
        # this topic can be cross-checked against /safety_stop itself.
        if active_lane == 'emergency':
            msg.stop_source = 'emergency'
        elif active_lane == 'handle_obstacle':
            msg.stop_source = 'obstacle'
        else:
            msg.stop_source = ''
        msg.safety_stop_active = bool(msg.stop_source)

        publisher.publish(msg)

    return _publish


def create_root(
    max_temp_c=85.0, max_load_percent=95.0,
    car_radius=0.20, obstacle_safety_margin_m=0.12, proximity_front_extra_margin_m=0.08,
    enable_camera_obstacle_stop=False,
    enable_lidar_safety_stop=True,
    proximity_front_threshold_m=0.15,
    proximity_side_threshold_m=0.15,
    enable_sys_obs_load_trip=False,
    sys_obs_load_trip_consecutive_samples=3,
) -> py_trees.behaviour.Behaviour:
    """Build the root Selector.

    ==========================================================================
    OBSTACLE-AVOIDANCE TEST CONFIGURATION (2026-09-01 mission-analysis follow-up)
    ==========================================================================
    Three of this tree's stop paths were retuned after the 2026-09-01 bag
    analysis, which found that 100% of unexpected stops came from this tree
    preempting the MPC on the ackermann_mux safety_stop lane (priority 200),
    NOT from the MPC failing -- the solver converged on 1285/1285 solves with
    zero missed deadlines. See mission_analysis_2026-09-01.md.

    enable_camera_obstacle_stop (DEFAULT FALSE -- this is the behaviour change):
        Gates the whole handle_obstacle lane (IsObstacleDetected + Stop).
        Camera-seen obstacles are no longer supposed to halt the car outright;
        they reach the controller as SOFT AVOIDANCE instead, through the path
        that already exists and is unaffected by this flag:

            /camera/detections_3d
              -> obstacle_projector_node  (f1tenth_perception)
              -> /perception/obstacles_2d
              -> MPC_corr.obstacles_global_live
              -> solve_mpc_step(obstacles=...)   (w_obs cost term)

        That path is why disabling this lane does not leave camera obstacles
        unhandled -- it is a change of RESPONSE (steer around) rather than a
        removal of the input. NOTE it is a soft cost term, not a hard
        constraint, which is exactly why the lidar floor below stays on.

        The lane is left constructed-but-gated rather than deleted so it can
        be flipped back on as a comparison point / fallback in one config
        change. Structurally absent when disabled (same pattern as
        enable_sys_obs), not a lane that is ticked and always fails: a lane
        that exists but never succeeds is indistinguishable in a BT snapshot
        from one whose condition is simply not tripping.

        WHAT THIS COSTS, stated plainly: in run 14-35-50 this lane was the
        only thing stopping the car from driving into a chair 0.66 m ahead,
        and it held for the entire 31.8 s run. With it off, that scenario is
        handled by MPC avoidance or not at all.

    enable_lidar_safety_stop / proximity_*_threshold_m:
        IsProximityTooClose stays ON but is tightened from its old
        derived thresholds (0.40 m front, 0.20 m side/rear) to a genuine
        last-resort contact range (0.15 m / 0.15 m by default). It is NOT
        redundant with MPC avoidance -- it is the floor that catches the case
        where avoidance itself is wrong (stale costmap, dropped track, or a
        logic bug of the kind the 2026-09-01 analysis already found once).
        Kept as its own flag so it is a one-line change to remove, but the
        default is ON: the first run that evaluates whether avoidance works is
        the wrong moment to remove the thing that reports when it does not.

        These are now DIRECT thresholds rather than being derived from
        car_radius + margins. The old derivation deliberately composed a
        safety margin on top of the car's radius; a last-resort contact
        threshold is the opposite intent (deliberately inside that margin),
        so deriving it from the same terms would be misleading. car_radius
        (0.20 m) is now larger than these thresholds, which is correct and
        intentional: this trips only once something is nearer than the car's
        own nominal radius.

    enable_sys_obs_load_trip (DEFAULT FALSE):
        See is_system_overheated.py's own docstring. Temperature still trips
        undebounced; CPU/GPU *utilization* no longer trips by default, because
        all three of its observed trips were false positives on bursty YOLO
        inference load while temperatures sat 45 C below their limit.
    """
    root = py_trees.composites.Selector(name='root', memory=False)

    # OR of the emergency conditions -- any one tripping is enough to stop.
    # IsSystemOverheated is only added at all if enable_sys_obs is true (see module
    # docstring for why this check happens here, not inside the behaviour itself).
    # IsEmergencyStopTriggered and IsProximityTooClose are unconditional too,
    # same as IsBatteryLow -- an operator/hardware emergency stop and raw
    # hardware proximity safety are never gateable behind a config toggle.
    # Safety-margin unification pass: lidar_distance_threshold/front_distance_threshold
    # are derived from the shared car_radius/obstacle_safety_margin_m/
    # proximity_front_extra_margin_m stack params rather than hardcoded -- see
    # stack_params.yaml's car_radius comment for the full 4-mechanism picture, and
    # is_proximity_too_close.py's own docstring for the derivation reasoning.
    # Tightened last-resort contact thresholds, passed straight through rather
    # than derived from car_radius + margins -- see this function's docstring
    # for why the old derivation no longer expresses the right intent.
    emergency_condition = py_trees.composites.Selector(
        name='emergency_condition', memory=False)
    emergency_children = [
        IsBatteryLow(),
        IsEmergencyStopTriggered(),
    ]
    if enable_lidar_safety_stop:
        emergency_children.append(IsProximityTooClose(
            front_distance_threshold=proximity_front_threshold_m,
            lidar_distance_threshold=proximity_side_threshold_m,
        ))
    if get_value('enable_sys_obs'):
        emergency_children.append(IsSystemOverheated(
            max_temp_c=max_temp_c, max_load_percent=max_load_percent,
            enable_load_trip=enable_sys_obs_load_trip,
            load_trip_consecutive_samples=sys_obs_load_trip_consecutive_samples))
    emergency_condition.add_children(emergency_children)

    # frame_id distinguishes THIS Stop from handle_obstacle's below. Both
    # publish on the same safety_stop mux lane, and the ackermann_mux
    # republishes the winning message verbatim, so frame_id is what lets a bag
    # tell an emergency stop from an obstacle stop -- during the 2026-09-01
    # analysis both stamped 'base_link' and were indistinguishable, which is
    # why every stop had to be re-bucketed by replaying predicates offline.
    emergency = py_trees.composites.Sequence(name='emergency', memory=False)
    emergency.add_children([
        emergency_condition,
        Stop(output_topic='safety_stop', frame_id='base_link/emergency'),
    ])

    # Safety-margin unification pass: corridor_half_width/corridor_half_height are
    # both derived from car_radius + obstacle_safety_margin_m -- this is the fix for
    # the corridor half-width/height drift the Phase 1 audit flagged (this
    # behaviour's own old 0.25 vs. safety_stop_controller's separate 0.4, now retired)
    # -- see stack_params.yaml's car_radius comment for the full derivation.
    obstacle_corridor_half = car_radius + obstacle_safety_margin_m
    handle_obstacle = None
    if enable_camera_obstacle_stop:
        handle_obstacle = py_trees.composites.Sequence(
            name='handle_obstacle', memory=False)
        handle_obstacle.add_children([
            IsObstacleDetected(
                detections_topic='/camera/detections_3d',
                stop_distance=1.0,
                corridor_half_width=obstacle_corridor_half,
                corridor_half_height=obstacle_corridor_half,
            ),
            Stop(output_topic='safety_stop', frame_id='base_link/obstacle'),
        ])

    # Structurally branched on enable_nav2 -- same "absent, not just failing" pattern
    # as emergency_condition's enable_sys_obs check above: HasGoalPose listens on
    # /goal_pose (PoseStamped), a completely different topic/message type from what
    # mpc_corr.py actually consumes (/mpc/goal_distance, Float32), so leaving it wired
    # in unconditionally meant this lane silently never succeeded at all with
    # enable_nav2:=false (see has_mpc_goal.py's own docstring for the fix).
    navigation = py_trees.composites.Sequence(name='navigation', memory=False)
    if get_value('enable_nav2'):
        navigation.add_children([
            HasGoalPose(goal_pose_topic='/goal_pose', goal_key=GOAL_POSE_KEY),
            NavigateThroughPosesClient(
                goal_key=GOAL_POSE_KEY, action_name='navigate_through_poses'),
        ])
    else:
        # No action-client sibling needed here -- mpc_corr.py drives itself directly
        # off /mpc/goal_distance, entirely out of band from the BT (see
        # mpc_corr.launch.py). This condition alone is enough for the Sequence/root
        # Selector and BT snapshot log to correctly report the navigation lane as
        # active while an MPC goal is in progress.
        navigation.add_children([
            HasMpcGoal(goal_distance_topic='/mpc/goal_distance'),
        ])

    # -- mission -----------------------------------------------------------------
    # See module docstring's "Mission subtree" section for the priority-ordering
    # rationale and pointers to each behaviour's own deviation notes.
    mission_object_response = py_trees.composites.Sequence(
        name='mission_object_response', memory=False)
    mission_object_response.add_children([ObjectSeen(), HandleObjectAction()])

    # PublishMoveGoal first, not last as in the original task's tree diagram --
    # see that behaviour's own docstring for the full tick-by-tick reasoning
    # (the literal ordering never publishes move 0's goal at mission start).
    # odom_topic follows localization_source (get_odom_topic(), see param_defaults.py)
    # instead of CheckStopCondition's own '/odom' default -- so the mission's
    # distance-travelled stop condition reads the EKF-fused pose once localization_source
    # is 'ekf', not silently-still-raw wheel/steering dead reckoning.
    mission_progress = py_trees.composites.Sequence(name='mission_progress', memory=False)
    mission_progress.add_children(
        [PublishMoveGoal(), CheckStopCondition(odom_topic=get_odom_topic()), AdvanceMove()])

    mission_selector = py_trees.composites.Selector(name='mission_selector', memory=False)
    mission_selector.add_children([mission_object_response, mission_progress])

    mission = py_trees.composites.Sequence(name='mission', memory=False)
    mission.add_children([MissionActive(), mission_selector])

    # handle_obstacle is omitted entirely when enable_camera_obstacle_stop is
    # false. Ordering among the remaining lanes is unchanged.
    #
    # NOTE ON SELECTOR SEMANTICS, worth stating because it caused a real bug:
    # this is a memory=False Selector, so it stops at its first succeeding
    # child and the lanes AFTER that child are not ticked at all. In run
    # 14-35-50 handle_obstacle succeeded on 100% of frames (a chair 0.66 m
    # ahead), which meant `mission` was never ticked for the entire run --
    # PublishMoveGoal never ran, no goal was ever published, and the MPC sat
    # in its "no goal received" branch for 31.8 s while the mission still
    # reported COMPLETE. Disabling this lane removes that particular starvation
    # path, but the same shape exists for `emergency`: any lane that succeeds
    # persistently starves the mission subtree behind it.
    lanes = [emergency]
    if handle_obstacle is not None:
        lanes.append(handle_obstacle)
    lanes += [mission, navigation]
    root.add_children(lanes)
    return root


# nice -- own inline copy (own copy, not a cross-package import of
# f1tenth_perception.cpu_affinity: same precedent MPC_corr.py already
# established -- a package with a single consumer of this logic keeps its
# own copy rather than adding a dependency on an unrelated package just for
# a few lines). Takes `node` explicitly (there's no self here: this node has
# no custom Node subclass/__init__, tree.node is only constructed inside
# main() below, well after module load).
#
# CPU AFFINITY REMOVED FROM HERE (thread-pinning-leak fix, Step 6
# reintroduction investigation): this used to also call
# os.sched_setaffinity(0, cores) on a declared cpu_affinity param, applied
# once, in-process, from behavior_executor_node's main() (see git history) --
# confirmed live to only restrict the ONE thread executing that call, not
# the process: 21 of this node's 22 threads showed full 0-11 affinity, with
# 2 actually caught executing on cpu1 (one of the EKF pair's own reserved
# cores) under real load. Worse, that call ran AFTER tree.setup() (py_trees_
# ros' own executor/action-client machinery), so most of this node's
# threads already existed, unpinned, before it ever fired. Affinity is now
# an external `taskset -c <cores>` launch prefix instead (see
# behavior_bringup.launch.py's own matching comment) -- it sets the mask
# before this process's first instruction runs, so every thread this node
# or any library it uses ever spawns inherits it, with no in-process code
# needed at all. nice stays here (same self-applied, main-thread-only
# mechanism as before) -- it was never the leak; only affinity was. Renamed
# _apply_cpu_affinity_and_priority -> _apply_nice to match what this
# function actually does now (same reasoning as f1tenth_perception/
# cpu_affinity.py's own matching rename).
def _apply_nice(node):
    node.declare_parameter('nice', 0)
    nice_val = int(node.get_parameter('nice').value)
    if nice_val != 0:
        try:
            os.nice(nice_val)
            node.get_logger().info(f'Process nice set to {nice_val:+d}')
        except Exception as exc:  # noqa: BLE001 - best-effort, never fatal
            node.get_logger().warn(
                f'Could not set nice {nice_val:+d} (need CAP_SYS_NICE/root): {exc}')


def main():
    rclpy.init()

    # bt_setup_timeout_sec must be known BEFORE tree.setup() runs (it's the timeout
    # argument to that very call) -- but py_trees_ros.trees.BehaviourTree.node is None
    # until .setup() actually creates it, so tree.node isn't usable yet at this point.
    # A throwaway bootstrap node reads just this one parameter first; launch's
    # parameters=[...] overrides are process-global -p flags (not node-name-scoped),
    # so this picks up the same override behavior_bringup.launch.py passes, same as
    # tree.node itself would once it exists.
    bootstrap_node = rclpy.create_node('behavior_executor_node_bootstrap')
    # Was hardcoded to 15.0 -- Nav2's lifecycle chain (map_server -> full stack ->
    # lifecycle_manager_navigation) can take longer than that under load, especially on
    # Jetson. behavior_bringup.launch.py's readiness gate (wait_for_trigger_service_node
    # polling lifecycle_manager_navigation's is_active) is the primary fix for the race;
    # this timeout is a second, independent safety margin on top of that, not a
    # replacement for it -- kept generous rather than assuming sequencing alone makes
    # the exact value irrelevant.
    bt_setup_timeout_sec = float(
        bootstrap_node.declare_parameter('bt_setup_timeout_sec', 60.0).value)
    # IsSystemOverheated's shared CPU/GPU thresholds (see create_root()'s docstring) --
    # read the same way and for the same reason as bt_setup_timeout_sec above: needed
    # before tree.node exists.
    max_temp_c = float(
        bootstrap_node.declare_parameter('sys_obs_max_temp_c', 85.0).value)
    max_load_percent = float(
        bootstrap_node.declare_parameter('sys_obs_max_load_percent', 95.0).value)
    # Safety-margin unification pass -- same bootstrap-read pattern as the two
    # sys_obs params above, needed before tree.node exists (see create_root()'s
    # own comments and stack_params.yaml's car_radius comment for the derivation
    # these three feed IsObstacleDetected/IsProximityTooClose).
    car_radius = float(
        bootstrap_node.declare_parameter('car_radius', 0.20).value)
    obstacle_safety_margin_m = float(
        bootstrap_node.declare_parameter('obstacle_safety_margin_m', 0.12).value)
    proximity_front_extra_margin_m = float(
        bootstrap_node.declare_parameter('proximity_front_extra_margin_m', 0.08).value)
    # Obstacle-avoidance test configuration -- same bootstrap-read pattern as
    # everything above (needed before tree.node exists). See create_root()'s
    # docstring for what each of these changes and why the defaults are what
    # they are.
    enable_camera_obstacle_stop = bool(
        bootstrap_node.declare_parameter('enable_camera_obstacle_stop', False).value)
    enable_lidar_safety_stop = bool(
        bootstrap_node.declare_parameter('enable_lidar_safety_stop', True).value)
    proximity_front_threshold_m = float(
        bootstrap_node.declare_parameter('proximity_front_threshold_m', 0.15).value)
    proximity_side_threshold_m = float(
        bootstrap_node.declare_parameter('proximity_side_threshold_m', 0.15).value)
    enable_sys_obs_load_trip = bool(
        bootstrap_node.declare_parameter('enable_sys_obs_load_trip', False).value)
    sys_obs_load_trip_consecutive_samples = int(
        bootstrap_node.declare_parameter(
            'sys_obs_load_trip_consecutive_samples', 3).value)
    bootstrap_node.destroy_node()

    root = create_root(
        max_temp_c=max_temp_c, max_load_percent=max_load_percent,
        car_radius=car_radius, obstacle_safety_margin_m=obstacle_safety_margin_m,
        proximity_front_extra_margin_m=proximity_front_extra_margin_m,
        enable_camera_obstacle_stop=enable_camera_obstacle_stop,
        enable_lidar_safety_stop=enable_lidar_safety_stop,
        proximity_front_threshold_m=proximity_front_threshold_m,
        proximity_side_threshold_m=proximity_side_threshold_m,
        enable_sys_obs_load_trip=enable_sys_obs_load_trip,
        sys_obs_load_trip_consecutive_samples=sys_obs_load_trip_consecutive_samples,
    )
    tree = py_trees_ros.trees.BehaviourTree(root=root)

    try:
        tree.setup(timeout=bt_setup_timeout_sec)
    except Exception as exc:  # noqa: BLE001 - convert into a diagnosable exit, not a
        # bare traceback from py_trees's own signal handler. NOT using tree.node here
        # -- same reason as above, it may still be None/partially-built on a failed
        # setup(), so a module-level logger is used instead of a Node method.
        rclpy.logging.get_logger('behavior_executor_node').error(
            f'STARTUP ABORTED: tree.setup() did not complete within '
            f'{bt_setup_timeout_sec:.1f}s -- NavigateThroughPoses action server not '
            'available. Is bt_navigator active? check: ros2 lifecycle get /bt_navigator '
            f'-- underlying error: {exc!r}')
        rclpy.shutdown()
        sys.exit(1)

    node = tree.node

    # nice -- see this module's own _apply_nice() docstring/comment. CPU
    # affinity is set externally now (taskset -c launch prefix), before this
    # process even starts, so there's no equivalent "apply as early as
    # possible" concern for it anymore -- only nice still needs a real node.
    _apply_nice(node)

    node.declare_parameter('bt_loop_duration_ms', 100)
    bt_loop_duration_ms = node.get_parameter('bt_loop_duration_ms').value

    # Not py_trees behaviours -- see their own module docstrings for why
    # (subscription/parameter-driven, not per-tick actions). Built here, not
    # inside create_root(), because both need a real node (tree.node), which
    # doesn't exist until tree.setup() has already succeeded above. Held in
    # local variables deliberately -- both keep the node's subscriptions alive
    # via their own self._sub references, but an explicit reference here too
    # makes that lifetime obvious rather than relying on rclpy's internal
    # subscription bookkeeping being the only thing keeping them alive.
    detected_classes_bridge = DetectedClassesBridge(node)
    mission_loader = MissionLoader(node)

    node.get_logger().info(
        'No goal pose received yet -- waiting on /goal_pose (see HasGoalPose).')

    tree.add_post_tick_handler(make_snapshot_logger(min_interval_sec=1.0))

    # Per-tick lane/condition telemetry -- NOT gated behind a flag, unlike the
    # visualization below. This is the signal that makes a stop diagnosable
    # from the bag alone, so it must be present on exactly the runs where
    # something goes wrong; a config toggle is how it would come to be off on
    # the one run that mattered.
    tree_status_pub = node.create_publisher(
        BehaviorTreeStatus, '/behavior/tree_status', 10)
    tree.add_post_tick_handler(make_tree_status_publisher(node, tree_status_pub))

    node.get_logger().info(
        f'BT safety config: camera_obstacle_stop={enable_camera_obstacle_stop} '
        f'lidar_safety_stop={enable_lidar_safety_stop} '
        f'(front={proximity_front_threshold_m:.2f}m side={proximity_side_threshold_m:.2f}m) '
        f'sys_obs_load_trip={enable_sys_obs_load_trip} '
        f'(needs {sys_obs_load_trip_consecutive_samples} consecutive samples); '
        f'camera obstacles reach the MPC via /perception/obstacles_2d regardless.')

    # Structurally absent when disabled -- same "not just idling it" pattern
    # as enable_sys_obs/enable_foxglove (see stack_params.yaml's own comment
    # on this param): no publisher, no post_tick_handler registered at all,
    # not a handler that no-ops every tick.
    if get_value('enable_bt_visualization'):
        bt_visualization_pub = node.create_publisher(
            CompressedImage, '/bt/tree_visualization', 10)
        tree.add_post_tick_handler(make_tree_visualizer(node, bt_visualization_pub))

    tree.tick_tock(period_ms=bt_loop_duration_ms)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        tree.shutdown()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
