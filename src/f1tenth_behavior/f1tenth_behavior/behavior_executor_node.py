"""Owns the outer BT (obstacle-stop > navigate): loads the waypoint list once at
startup, builds a py_trees_ros.trees.BehaviourTree, and ticks it forever as an ambient
supervisor via tick_tock() -- this tree is not a one-shot "run until SUCCESS/FAILURE"
helper, it runs for the life of the process, mirroring the old BT.CPP
behavior_executor_node's own wall-timer-driven tickRoot() loop.

Priority structure (root Selector, first child that succeeds wins -- same as the old
safety_stop_and_navigate.xml's ReactiveFallback):
  1. handle_obstacle: IsObstacleDetected -> Stop
  2. navigation: NavigateThroughPoses (py_trees_ros action client, static waypoint goal)

No emergency-stop branch: the old BT.CPP XML never had one either (there is no
IsNoEmergency signal anywhere in this stack), so it is intentionally not invented here.
"""

import math

import py_trees
import py_trees_ros
import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateThroughPoses

from f1tenth_behavior.behaviours.is_obstacle_detected import IsObstacleDetected
from f1tenth_behavior.behaviours.stop import Stop

WAYPOINTS_GOAL_KEY = 'navigate_through_poses_goal'


def create_root() -> py_trees.behaviour.Behaviour:
    root = py_trees.composites.Selector(name='root', memory=False)

    handle_obstacle = py_trees.composites.Sequence(name='handle_obstacle', memory=False)
    handle_obstacle.add_children([
        IsObstacleDetected(
            detections_topic='/camera/detections_3d',
            stop_distance=1.0,
            corridor_half_width=0.25,
            corridor_half_height=0.25,
        ),
        Stop(output_topic='safety_stop', frame_id='base_link'),
    ])

    navigation = py_trees_ros.action_clients.FromBlackboard(
        name='NavigateThroughPoses',
        action_type=NavigateThroughPoses,
        action_name='navigate_through_poses',
        key=WAYPOINTS_GOAL_KEY,
    )

    root.add_children([handle_obstacle, navigation])
    return root


def load_waypoints(node) -> list:
    node.declare_parameter('waypoint_frame', 'map')
    node.declare_parameter('waypoint_x', [])
    node.declare_parameter('waypoint_y', [])
    node.declare_parameter('waypoint_yaw', [])

    frame = node.get_parameter('waypoint_frame').value
    xs = node.get_parameter('waypoint_x').value
    ys = node.get_parameter('waypoint_y').value
    yaws = node.get_parameter('waypoint_yaw').value
    if not yaws:
        yaws = [0.0] * len(xs)

    if not (len(xs) == len(ys) == len(yaws)):
        raise RuntimeError(
            'waypoint_x/waypoint_y/waypoint_yaw parallel arrays must be the same length')

    waypoints = []
    for x, y, yaw in zip(xs, ys, yaws):
        pose = PoseStamped()
        pose.header.frame_id = frame
        pose.pose.position.x = x
        pose.pose.position.y = y
        # Yaw-only orientation (2D navigation goal): no roll/pitch, so the quaternion
        # reduces to (0, 0, sin(yaw/2), cos(yaw/2)).
        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)
        waypoints.append(pose)
    return waypoints


def main():
    rclpy.init()

    root = create_root()
    tree = py_trees_ros.trees.BehaviourTree(root=root)
    tree.setup(timeout=15.0)
    node = tree.node

    node.declare_parameter('bt_loop_duration_ms', 100)
    bt_loop_duration_ms = node.get_parameter('bt_loop_duration_ms').value

    waypoints = load_waypoints(node)
    node.get_logger().info(f'Loaded {len(waypoints)} waypoints')

    blackboard = py_trees.blackboard.Client(name='WaypointGoal')
    blackboard.register_key(key=WAYPOINTS_GOAL_KEY, access=py_trees.common.Access.WRITE)
    setattr(blackboard, WAYPOINTS_GOAL_KEY, NavigateThroughPoses.Goal(poses=waypoints))

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
