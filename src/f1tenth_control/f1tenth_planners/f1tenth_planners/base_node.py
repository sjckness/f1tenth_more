"""The ROS side both new planner-controllers share: mpc_corr's contract with the mission.

corridor_rrt and p2p_geometric differ only in how they plan (plan()) and how a
blocked path limits the speed (blocked_speed_cap()). Everything the behaviour
tree sees is here, and it is mpc_corr's interface, topic for topic:

  in   /mpc/goal_distance, /mpc/goal_pose, /mpc/goal_turn, /mpc/goal_drive,
       /mpc/goal_object, /mpc/goal_object_end (moves.py, one class per shape),
       /mpc/hold (absolute priority: zero and nothing else; ends an object
       move), /mpc/stop_target + the wall distance (the front_clearance ramp),
       odometry (get_odom_topic()), the obstacle list (mpc_obstacles_topic,
       base_link circles), /perception/walls (odom wall segments),
       /costmap/unknown_speed_cap.
  out  /drive (the mux's navigation lane, published every tick as mpc_corr
       does: zero while idle, held or stale), /mpc/goal_reached (latched True
       while an arrived distance / pose / turn move stays current),
       /mpc/min_obstacle_distance and /mpc/min_obstacle_distance_forward (from
       the front bumper, same definitions), /mpc/object_status (every tick of an
       object move: r, the stop latch, target_behind persistence, the
       refresh watchdog), and for viewing only /planner/path (nav_msgs/Path)
       and /planner/markers.

Speed: the move's speed (else planner_cruise_speed_mps), capped by the pure
pursuit curvature limit, the arrival ramp (sqrt(2 a d) toward the stopping
point, never below min_moving_speed_mps while moving), the front_clearance
ramp, the unknown-object cap and the subclass's blocked-path cap, then clamped
to max_forward_speed_mps. Never negative: neither mode reverses.
"""

import json
import math
import time
from typing import List, Optional

from ackermann_msgs.msg import AckermannDriveStamped
from f1tenth_comms.tf_poll import make_tf
from f1tenth_description.vehicle_frames import base_front_x
from f1tenth_messages.msg import (
    DriveCommand, MultiWallTrack, MultiWallTrackArray, ObjectApproachStatus, ObjectGoal,
    Obstacle2DArray, TurnGoal)
from f1tenth_params.param_defaults import get_odom_topic, get_value, get_wall_distance_topic
from f1tenth_planners.geometry import (
    Obstacles, point_segment_distance, pose_map_to_odom, robot_to_world)
from f1tenth_planners.moves import (
    DistanceMove, DriveMove, Move, ObjectMove, PlanTarget, PoseMove, TurnMove)
from f1tenth_planners.pure_pursuit import (
    min_turning_radius, PurePursuit, PurePursuitConfig, rate_limit)
from geometry_msgs.msg import Point, PoseStamped
from mpc_controller.drive_limits import clamp_drive_speed, validate_speed_limits
from mpc_controller.object_approach import (
    assess_object_approach, heading_margin_for, object_speed_decision, ObjectStopLatch,
    SPEED_DRIVE, TargetBehindPersistence)
from mpc_controller.object_guard import RefreshWatchdog
from nav_msgs.msg import Odometry, Path
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy)
from std_msgs.msg import Bool, Float32, String
from tf2_ros import ConnectivityException, ExtrapolationException, LookupException
from visualization_msgs.msg import Marker, MarkerArray

#: Wall tracks the planners keep clear of: matched now or briefly coasting,
#: with LiDAR evidence. A stereo-only wall is not trusted to block a path.
_WALL_STATES = (MultiWallTrack.STATE_CONFIRMED, MultiWallTrack.STATE_COASTING)
_WALL_SOURCES = (MultiWallTrack.SOURCE_LIDAR, MultiWallTrack.SOURCE_BOTH)


def _yaw_from_quaternion(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def relaxed_obstacles(obstacles: Obstacles, points, slack: float = 0.02) -> Obstacles:
    """Shrink inflations just enough that each of ``points`` is free.

    A car that already sits inside an inflation (a side wall 0.3 m away
    against a 0.32 m clearance, an object goal whose standoff is tighter than
    the margin) would otherwise make every edge from the start, or to the
    goal, collide, and the planner would find nothing. The inflation is
    lowered to the clearance those points actually have, so a path may not
    come closer than the car already is, and nothing else changes.
    """
    circles = obstacles.circles.copy()
    wall_clearance = obstacles.wall_clearance
    for px, py in points:
        if len(circles):
            d = np.hypot(circles[:, 0] - px, circles[:, 1] - py)
            circles[:, 2] = np.minimum(circles[:, 2], np.maximum(d - slack, 0.0))
        if len(obstacles.walls):
            w = obstacles.walls
            d = point_segment_distance(px, py, w[:, 0], w[:, 1], w[:, 2], w[:, 3])
            wall_clearance = min(wall_clearance, max(float(np.min(d)) - slack, 0.0))
    return Obstacles(circles, obstacles.walls, wall_clearance)


class PlannerControllerNode(Node):
    """Goal handling, state, safety and /drive for a planner + pure-pursuit controller."""

    def __init__(self, node_name: str, **node_kwargs):
        super().__init__(node_name, **node_kwargs)

        def p(name, key=None):
            return self.declare_parameter(name, get_value(key or name)).value

        self.ts = float(p('control_period_s', 'planner_control_period_s'))
        self.cruise_speed = float(p('cruise_speed_mps', 'planner_cruise_speed_mps'))
        self.replan_period = float(p('replan_period_s', 'planner_replan_period_s'))
        self.path_deviation = float(p('path_deviation_m', 'planner_path_deviation_m'))
        self.goal_tolerance = float(p('goal_tolerance_m', 'planner_goal_tolerance_m'))
        self.goal_pass_tolerance = float(p('goal_pass_tolerance_m',
                                           'planner_goal_pass_tolerance_m'))
        self.car_radius = float(p('car_radius'))
        self.avoidance_margin = float(p('avoidance_margin', 'obstacle_safety_margin_m'))
        self.wheelbase = float(p('wheelbase_m', 'mpc_wheelbase_m'))
        self.delta_min = float(p('delta_min', 'mpc_steering_angle_min_rad'))
        self.delta_max = float(p('delta_max', 'mpc_steering_angle_max_rad'))
        self.max_forward_speed = float(p('max_forward_speed_mps'))
        self.max_reverse_speed = float(p('max_reverse_speed_mps'))
        validate_speed_limits(self.max_forward_speed, self.max_reverse_speed)
        self.min_moving_speed = float(p('min_moving_speed_mps'))
        self.stop_lead = float(p('object_stop_distance_m'))
        self.ramp_enabled = bool(p('enable_stop_ramp'))
        self.ramp_decel = float(p('stop_ramp_a_dec_mps2'))
        self.steering_rate_max = float(p('steering_rate_max_rad_s',
                                         'pp_steering_rate_max_rad_s'))
        self.odom_stale_timeout = float(p('odom_stale_timeout_sec'))
        self.obstacles_topic = str(p('obstacles_topic', 'mpc_obstacles_topic'))
        self.obstacles_max_age = float(p('obstacles_max_age_s', 'mpc_obstacles_max_age_s'))
        self.target_exclusion_gate = float(p('target_exclusion_gate_m',
                                             'mpc_target_exclusion_gate_m'))
        self.wall_max_age = float(p('walls_max_age_s', 'mpc_obstacles_max_age_s'))
        self.front_distance_max_age = float(p('corr_wall_turn_front_distance_max_age_sec'))
        self.wall_turn_margin = float(p('wall_turn_safety_margin_m',
                                        'corr_wall_turn_safety_margin_m'))
        self.map_frame = str(self.declare_parameter('map_frame', 'map').value)
        self.r_min = min_turning_radius(self.wheelbase, self.delta_min, self.delta_max)

        self.tracker = PurePursuit(PurePursuitConfig(
            wheelbase=self.wheelbase, delta_min=self.delta_min, delta_max=self.delta_max,
            lookahead_min=float(p('pp_lookahead_min_m')),
            lookahead_gain=float(p('pp_lookahead_gain_s')),
            lookahead_max=float(p('pp_lookahead_max_m')),
            lateral_accel_max=float(p('pp_lateral_accel_max_mps2'))))

        # Object approach: mpc_corr's own helpers and keys, so the mission's
        # object_reached sees the same latch at the same distances.
        self.object_latch = ObjectStopLatch(float(p('object_reach_tol_m')), self.stop_lead)
        self.object_r_freeze = float(p('object_r_freeze_m'))
        self.object_heading_margin = heading_margin_for(self.object_r_freeze)
        self.object_behind = TargetBehindPersistence(float(p('object_behind_persist_sec')))
        self.object_watchdog = RefreshWatchdog(float(p('object_goal_timeout_sec')))
        self.object_watchdog_tripped = False
        self.object_behind_terminal = False
        self.object_behind_for_s = 0.0
        self.object_target_held = False
        self.object_ended_ids = set()

        # State.
        self.pose = None            # (x, y, yaw) odom, base_link
        self.v = 0.0
        self.odom_frame = 'odom'
        self.odom_time: Optional[float] = None
        self.hold = False
        self.move: Optional[Move] = None
        self.reached = False
        self.path: Optional[np.ndarray] = None
        self.plan_time: Optional[float] = None
        self.plan_failures = 0
        self.last_steer = 0.0
        self.last_tick: Optional[float] = None
        self.obstacles_odom: List = []
        self.obstacles_time: Optional[float] = None
        self.walls_odom = np.zeros((0, 4))
        self.walls_time: Optional[float] = None
        self.front_distance: Optional[float] = None
        self.front_distance_time: Optional[float] = None
        self.stop_target = -1.0
        self.unknown_cap: Optional[float] = None
        self.unknown_cap_time: Optional[float] = None

        self.tf = make_tf(self)

        odom_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                              durability=DurabilityPolicy.VOLATILE,
                              history=HistoryPolicy.KEEP_LAST, depth=10)
        self.create_subscription(Odometry, get_odom_topic(), self._odom_cb, odom_qos)
        self.create_subscription(Float32, '/mpc/goal_distance', self._goal_distance_cb, 10)
        self.create_subscription(PoseStamped, '/mpc/goal_pose', self._goal_pose_cb, 10)
        self.create_subscription(TurnGoal, '/mpc/goal_turn', self._goal_turn_cb, 10)
        self.create_subscription(DriveCommand, '/mpc/goal_drive', self._goal_drive_cb, 10)
        self.create_subscription(ObjectGoal, '/mpc/goal_object', self._goal_object_cb, 10)
        self.create_subscription(String, '/mpc/goal_object_end', self._goal_object_end_cb, 10)
        self.create_subscription(Bool, '/mpc/hold', self._hold_cb, 10)
        self.create_subscription(
            Float32, '/mpc/stop_target', self._stop_target_cb,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(Float32, get_wall_distance_topic(), self._front_cb, 10)
        self.create_subscription(Obstacle2DArray, self.obstacles_topic, self._obstacles_cb, 10)
        self.create_subscription(MultiWallTrackArray, '/perception/walls', self._walls_cb, 10)
        self.create_subscription(Float32, '/costmap/unknown_speed_cap',
                                 self._unknown_cap_cb, 10)

        self.drive_pub = self.create_publisher(AckermannDriveStamped, '/drive', 10)
        self.goal_reached_pub = self.create_publisher(Bool, '/mpc/goal_reached', 10)
        self.min_obs_pub = self.create_publisher(Float32, '/mpc/min_obstacle_distance', 10)
        self.min_obs_fwd_pub = self.create_publisher(
            Float32, '/mpc/min_obstacle_distance_forward', 10)
        self.object_status_pub = self.create_publisher(
            ObjectApproachStatus, '/mpc/object_status', 10)
        self.path_pub = self.create_publisher(Path, '/planner/path', 10)
        self.markers_pub = self.create_publisher(MarkerArray, '/planner/markers', 10)
        self.status_pub = self.create_publisher(String, '/planner/status', 10)

        self.timer = self.create_timer(self.ts, self.control_loop)
        self.get_logger().info(
            f'{node_name}: planner-controller up | control {self.ts:.2f} s, cruise '
            f'{self.cruise_speed:.2f} m/s, R_min {self.r_min:.2f} m, inflation '
            f'{self.car_radius + self.avoidance_margin:.2f} m, odom {get_odom_topic()}, '
            f'obstacles {self.obstacles_topic}')

    # ------------------------------------------------------------------ hooks
    def plan(self, pose, target: PlanTarget, obstacles: Obstacles) -> Optional[np.ndarray]:
        """Return an Nx2 odom path from the car toward ``target``, or None. Subclasses."""
        raise NotImplementedError

    def blocked_speed_cap(self, pose, obstacles: Obstacles, s_car: float) -> float:
        """Speed cap [m/s] from obstacles on the current path. Subclasses may override."""
        return math.inf

    def extra_markers(self, stamp) -> List[Marker]:
        """Mode-specific markers (corridor, tree) for /planner/markers."""
        return []

    # -------------------------------------------------------------- callbacks
    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _odom_cb(self, msg: Odometry):
        p = msg.pose.pose
        self.pose = (float(p.position.x), float(p.position.y),
                     _yaw_from_quaternion(p.orientation))
        self.v = float(msg.twist.twist.linear.x)
        self.odom_frame = msg.header.frame_id or self.odom_frame
        self.odom_time = self._now()

    def _new_move(self, move: Move):
        """Make ``move`` current: a new goal of any shape replaces the last one."""
        if isinstance(self.move, ObjectMove) and not isinstance(move, ObjectMove):
            self.object_ended_ids.add(self.move.move_id)
        self.move = move
        self.reached = False
        self.path = None
        self.plan_time = None
        self.plan_failures = 0
        self.tracker.set_path(None)
        self.get_logger().info(f'MOVE | {move.describe()}')

    def _goal_distance_cb(self, msg: Float32):
        if self.pose is None:
            self.get_logger().warn('goal_distance before odometry: ignored')
            return
        self._new_move(DistanceMove(self.pose, float(msg.data), self.stop_lead))

    def _goal_pose_cb(self, msg: PoseStamped):
        if self.pose is None:
            self.get_logger().warn('goal_pose before odometry: ignored')
            return
        # Coordinates taken as odom, exactly as mpc_corr's goal_pose_callback
        # takes them, so a mission behaves the same in every mode.
        goal = (float(msg.pose.position.x), float(msg.pose.position.y))
        self._new_move(PoseMove(self.pose, goal, _yaw_from_quaternion(msg.pose.orientation),
                                self.goal_tolerance, self.goal_pass_tolerance))

    def _goal_turn_cb(self, msg: TurnGoal):
        if self.pose is None:
            self.get_logger().warn('goal_turn before odometry: ignored')
            return
        self._new_move(TurnMove(
            self.pose, math.radians(float(msg.heading_delta_deg)), str(msg.steering),
            self.wheelbase, min(abs(self.delta_min), abs(self.delta_max)),
            self.goal_tolerance, self.goal_pass_tolerance, speed=float(msg.speed)))

    def _goal_drive_cb(self, msg: DriveCommand):
        if self.pose is None:
            self.get_logger().warn('goal_drive before odometry: ignored')
            return
        mag = abs(float(msg.turn_mag_deg)) or 90.0
        self._new_move(DriveMove(
            self.pose, str(msg.mode), float(msg.turn_sign), math.radians(mag),
            horizon=self.drive_horizon(), speed=float(msg.speed),
            trigger_distance=self.r_min + self.wall_turn_margin))

    def drive_horizon(self) -> float:
        """How far ahead an open-ended drive move's rolling target sits [m]."""
        return 3.0

    def _goal_object_cb(self, msg: ObjectGoal):
        move_id = str(msg.move_id)
        if move_id and move_id in self.object_ended_ids:
            return
        if self.pose is None:
            self.get_logger().warn('goal_object before odometry: ignored')
            return
        standoff = float(msg.standoff)
        if not standoff > 0.0:
            self.get_logger().error(f'goal_object standoff {standoff:.3f} <= 0 refused')
            return
        now = self._now()
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        target = (float(msg.point.x), float(msg.point.y))
        move = self.move
        if isinstance(move, ObjectMove) and move_id and move.move_id == move_id:
            move.target_map_xy = target
            move.stamp_sec = stamp
            move.standoff = standoff
            move.raw_speed = float(msg.speed)
            move.speed = float(msg.speed) if msg.speed > 0.0 else None
            self.object_watchdog.note(now)
            return
        move = ObjectMove(move_id, str(msg.target_class), str(getattr(msg, 'track_id', '')),
                          target, standoff, float(msg.speed),
                          float(getattr(msg, 'gap_m', math.nan)))
        move.stamp_sec = stamp
        self._new_move(move)
        self.object_latch.reset()
        self.object_behind.reset()
        self.object_behind_terminal, self.object_behind_for_s = False, 0.0
        self.object_watchdog.reset()
        self.object_watchdog.note(now)
        self.object_watchdog_tripped = False
        self._refresh_object_target()
        tx, ty = move.target_odom_xy
        move.psi_c = math.atan2(ty - self.pose[1], tx - self.pose[0])

    def _goal_object_end_cb(self, msg: String):
        move_id = str(msg.data)
        self.object_ended_ids.add(move_id)
        if isinstance(self.move, ObjectMove) and self.move.move_id == move_id:
            self.get_logger().info(f'OBJECT | move {move_id!r} ended by the mission')
            self.move = None
            self.path = None
            self.tracker.set_path(None)

    def _hold_cb(self, msg: Bool):
        if bool(msg.data) != self.hold:
            self.get_logger().info(f'HOLD | {"engaged" if msg.data else "released"}')
        self.hold = bool(msg.data)
        if self.hold and isinstance(self.move, ObjectMove):
            self.object_ended_ids.add(self.move.move_id)
            self.move = None
            self.path = None
            self.tracker.set_path(None)

    def _stop_target_cb(self, msg: Float32):
        self.stop_target = float(msg.data)

    def _front_cb(self, msg: Float32):
        self.front_distance = float(msg.data)
        self.front_distance_time = self._now()

    def _obstacles_cb(self, msg: Obstacle2DArray):
        if self.pose is None:
            return
        self.obstacles_odom = [(*robot_to_world(self.pose, o.x, o.y), float(o.r))
                               for o in msg.obstacles]
        self.obstacles_time = self._now()

    def _walls_cb(self, msg: MultiWallTrackArray):
        if msg.header.frame_id and msg.header.frame_id != self.odom_frame:
            self.get_logger().warn(
                f'/perception/walls in {msg.header.frame_id!r}, not {self.odom_frame!r}: '
                'ignored', throttle_duration_sec=5.0)
            return
        segs = [(t.p0.x, t.p0.y, t.p1.x, t.p1.y) for t in msg.tracks
                if t.state in _WALL_STATES and t.source in _WALL_SOURCES]
        self.walls_odom = np.asarray(segs, dtype=float).reshape(-1, 4)
        self.walls_time = self._now()

    def _unknown_cap_cb(self, msg: Float32):
        self.unknown_cap = float(msg.data)
        self.unknown_cap_time = self._now()

    # ---------------------------------------------------------------- helpers
    def _fresh(self, stamp: Optional[float], max_age: float, now: float) -> bool:
        return stamp is not None and now - stamp <= max_age

    def live_circles(self, now: float) -> List:
        """Fresh obstacle circles (odom, raw radius), without the move's target."""
        if not self._fresh(self.obstacles_time, self.obstacles_max_age, now):
            return []
        circles = self.obstacles_odom
        move = self.move
        if isinstance(move, ObjectMove) and move.target_odom_xy is not None:
            tx, ty = move.target_odom_xy
            circles = [c for c in circles
                       if math.hypot(c[0] - tx, c[1] - ty) > self.target_exclusion_gate]
        return circles

    def obstacles(self, now: float) -> Obstacles:
        """Everything a path must keep clear of, inflated by car_radius + margin."""
        inflate = self.car_radius + self.avoidance_margin
        circles = [(x, y, r + inflate) for x, y, r in self.live_circles(now)]
        walls = (self.walls_odom if self._fresh(self.walls_time, self.wall_max_age, now)
                 else np.zeros((0, 4)))
        return Obstacles(circles, walls, inflate)

    def _bumper(self):
        x, y, yaw = self.pose
        front = base_front_x()
        return x + front * math.cos(yaw), y + front * math.sin(yaw)

    def _publish_obstacle_distances(self, now: float):
        d_all, d_fwd = 1e6, 1e6
        if self.pose is not None:
            bx, by = self._bumper()
            c, s = math.cos(self.pose[2]), math.sin(self.pose[2])
            for ox, oy, r in self.live_circles(now):
                d = math.hypot(bx - ox, by - oy) - r
                d_all = min(d_all, d)
                if (ox - bx) * c + (oy - by) * s > 0.0:
                    d_fwd = min(d_fwd, d)
        self.min_obs_pub.publish(Float32(data=float(d_all)))
        self.min_obs_fwd_pub.publish(Float32(data=float(d_fwd)))

    def _lookup_map_odom(self):
        try:
            tf = self.tf.lookup(self.map_frame, self.odom_frame)
        except (LookupException, ConnectivityException, ExtrapolationException):
            return None
        t = tf.transform
        return (float(t.translation.x), float(t.translation.y),
                _yaw_from_quaternion(t.rotation))

    def _refresh_object_target(self):
        move = self.move
        if not isinstance(move, ObjectMove):
            return
        edge = self._lookup_map_odom()
        if edge is None:
            # As mpc_corr: hold the last reprojection; with none yet, take the
            # map point as odom (identical while map and odom coincide).
            self.object_target_held = move.target_odom_xy is not None
            if move.target_odom_xy is None:
                move.target_odom_xy = move.target_map_xy
            return
        x, y, _ = pose_map_to_odom(move.target_map_xy[0], move.target_map_xy[1], 0.0, *edge)
        move.target_odom_xy = (x, y)
        self.object_target_held = False

    def publish_drive(self, speed: float, steering: float):
        """Clamp, rate-limit the steering, publish on /drive."""
        now = self._now()
        dt = self.ts if self.last_tick is None else max(now - self.last_tick, 0.0)
        self.last_tick = now
        steering = rate_limit(self.last_steer, float(steering), self.steering_rate_max, dt)
        steering = float(np.clip(steering, self.delta_min, self.delta_max))
        speed, _ = clamp_drive_speed(max(float(speed), 0.0), self.max_forward_speed,
                                     self.max_reverse_speed)
        self.last_steer = steering
        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.drive.speed = float(speed)
        msg.drive.steering_angle = steering
        self.drive_pub.publish(msg)

    def _publish_object_status(self, flags, speed_ref: float):
        move = self.move
        msg = ObjectApproachStatus()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.odom_frame
        msg.move_id = move.move_id
        msg.target_class = move.target_class
        msg.track_id = move.track_id
        msg.gap = math.nan
        if move.psi_c is not None:
            msg.psi_c = float(move.psi_c)
        if flags is not None:
            msg.r = float(flags.r)
            msg.bearing = float(flags.bearing)
            msg.e = float(flags.e)
            msg.alpha = float(flags.alpha)
            msg.inside_turn_radius = bool(flags.inside_turn_radius)
            msg.target_behind = bool(flags.target_behind)
            msg.gap = float(flags.r) + move.gap_m
        msg.target_behind_for_s = float(self.object_behind_for_s)
        msg.target_behind_terminal = bool(self.object_behind_terminal)
        msg.goal_watchdog = bool(self.object_watchdog_tripped)
        msg.speed_ref = float(speed_ref)
        msg.stop_latched = bool(self.object_latch.latched)
        msg.speed = float(self.v)
        msg.target_stale = bool(self.object_target_held)
        if move.stamp_sec is not None:
            msg.target_age_s = float(self._now() - move.stamp_sec)
        self.object_status_pub.publish(msg)

    def _object_flags(self, now: float):
        move = self.move
        if move.target_odom_xy is None or move.psi_c is None:
            return None
        flags = assess_object_approach(
            move.psi_c, move.target_odom_xy, (self.pose[0], self.pose[1]), self.pose[2],
            move.standoff, r_freeze=self.object_r_freeze,
            heading_margin=self.object_heading_margin, wheelbase=self.wheelbase,
            delta_min=self.delta_min, delta_max=self.delta_max)
        was = self.object_behind_terminal
        self.object_behind_terminal, self.object_behind_for_s = self.object_behind.update(
            flags.target_behind, now)
        if self.object_behind_terminal and not was:
            self.get_logger().warn(f'OBJECT | target behind for {self.object_behind_for_s:.2f} s:'
                                   ' terminal')
        return flags

    def _ramp_cap(self, distance: float) -> float:
        """Cap the speed by the braking parabola to a stop ``distance`` ahead (floored)."""
        if not self.ramp_enabled or not math.isfinite(distance):
            return math.inf
        if distance <= 0.0:
            return 0.0
        return max(math.sqrt(2.0 * self.ramp_decel * distance), self.min_moving_speed)

    #: Replan at once when an obstacle appears on the path ahead. corridor_rrt
    #: plans around it; p2p_geometric would plan the same path again, so it
    #: slows and stops instead (blocked_speed_cap).
    REPLAN_WHEN_BLOCKED = True
    #: How far ahead along the path [m] a new obstacle triggers that replan.
    BLOCKED_CHECK_M = 2.0

    def _needs_replan(self, now: float, obstacles: Obstacles, s_car: float,
                      cross_track: float) -> Optional[str]:
        if self.path is None:
            return 'no path'
        if self.plan_time is None or now - self.plan_time >= self.replan_period:
            return 'period'
        if cross_track > self.path_deviation:
            return f'off path {cross_track:.2f} m'
        if self.REPLAN_WHEN_BLOCKED and len(obstacles):
            relaxed = relaxed_obstacles(obstacles, [self.pose[:2]])
            s_block = relaxed.first_blocked_arclength(self.path, s_car, self.BLOCKED_CHECK_M)
            if math.isfinite(s_block):
                return f'blocked {s_block - s_car:.2f} m ahead'
        return None

    # ------------------------------------------------------------------- loop
    def control_loop(self):
        """One control tick; see the module docstring for the order of the caps."""
        now = self._now()
        self.tf.poll()
        self._publish_obstacle_distances(now)

        if (self.pose is None or self.odom_time is None
                or now - self.odom_time > self.odom_stale_timeout):
            self.get_logger().warn('odometry missing or stale: zero', throttle_duration_sec=2.0)
            self.publish_drive(0.0, 0.0)
            return
        if self.hold or self.move is None:
            self.publish_drive(0.0, 0.0)
            return

        move = self.move
        pose = self.pose
        flags = None
        speed_ref = move.speed if move.speed is not None else self.cruise_speed

        if isinstance(move, ObjectMove):
            self._refresh_object_target()
            reason = self.object_watchdog.tripped(now)
            if reason is not None:
                if not self.object_watchdog_tripped:
                    self.get_logger().error(f'OBJECT/watchdog | {reason}: hard stop')
                self.object_watchdog_tripped = True
                self.publish_drive(0.0, 0.0)
                self._publish_object_status(self._object_flags(now), 0.0)
                return
            self.object_watchdog_tripped = False
            self.object_latch.update(move.r(pose))
            decision = object_speed_decision(
                move.raw_speed, self.object_latch.latched, self.min_moving_speed)
            flags = self._object_flags(now)
            if decision.mode != SPEED_DRIVE:
                self.publish_drive(0.0, self.last_steer)
                self._publish_object_status(flags, 0.0)
                return
            speed_ref = decision.speed_ref
        elif isinstance(move, DriveMove):
            front = (self.front_distance
                     if self._fresh(self.front_distance_time, self.front_distance_max_age, now)
                     else None)
            if move.update_front_distance(front):
                self.get_logger().info(f'DRIVE | wall_turn committed at {front:.2f} m: '
                                       f'{move.describe()}')
                self.path = None

        progress = move.progress(pose)
        if not move.open_ended and not isinstance(move, ObjectMove):
            if self.reached or progress.reached:
                if not self.reached:
                    self.get_logger().info(
                        f'GOAL | reached: {move.describe()} (remaining '
                        f'{progress.remaining:+.3f} m)')
                self.reached = True
                self.publish_drive(0.0, 0.0)
                self.goal_reached_pub.publish(Bool(data=True))
                return

        obstacles = self.obstacles(now)
        if self.path is not None:
            cmd = self.tracker.step(pose, self.v)
            why = self._needs_replan(now, obstacles, cmd.s_car, cmd.cross_track)
        else:
            why = 'no path'
        if why is not None:
            self._replan(now, pose, move, obstacles, why)
        if self.path is None:
            self.publish_drive(0.0, self.last_steer)
            if isinstance(move, ObjectMove):
                self._publish_object_status(flags, 0.0)
            return

        cmd = self.tracker.step(pose, self.v)
        speed = min(speed_ref, max(cmd.speed_cap, self.min_moving_speed))
        target = move.target(pose)
        if target is not None and target.terminal:
            speed = min(speed, self._ramp_cap(progress.remaining - self.stop_lead))
        if (self.stop_target >= 0.0 and self.front_distance is not None
                and self.front_distance > 0.0
                and self._fresh(self.front_distance_time, self.front_distance_max_age, now)):
            speed = min(speed, self._ramp_cap(self.front_distance - self.stop_target))
        if (self.unknown_cap is not None and self.unknown_cap > 0.0
                and self._fresh(self.unknown_cap_time, self.obstacles_max_age, now)):
            speed = min(speed, self.unknown_cap)
        speed = min(speed, self.blocked_speed_cap(pose, obstacles, cmd.s_car))
        if 0.0 < speed < self.min_moving_speed:
            speed = self.min_moving_speed
        self.publish_drive(speed, cmd.steering)
        if isinstance(move, ObjectMove):
            self._publish_object_status(flags, speed)
        self.get_logger().info(
            f'CTRL | {move.kind} v_ref={speed_ref:.2f} v={speed:.2f} steer={cmd.steering:+.3f} '
            f'xte={cmd.cross_track:.2f} rem={progress.remaining:.2f}',
            throttle_duration_sec=1.0)

    def _replan(self, now: float, pose, move: Move, obstacles: Obstacles, why: str):
        target = move.target(pose)
        if target is None:
            self.path = None
            return
        t0 = time.perf_counter()
        path = self.plan(pose, target, obstacles)
        dt_ms = (time.perf_counter() - t0) * 1e3
        self.plan_time = now
        if path is None or len(path) < 2:
            self.plan_failures += 1
            keep = (self.path is not None and math.isinf(
                relaxed_obstacles(obstacles, [pose[:2]]).first_blocked_arclength(self.path)))
            self.get_logger().warn(
                f'PLAN | failed ({why}, {dt_ms:.0f} ms, {self.plan_failures} in a row): '
                + ('keeping the previous path' if keep else 'stopping'),
                throttle_duration_sec=1.0)
            if not keep:
                self.path = None
                self.tracker.set_path(None)
            self._publish_status(why, dt_ms, ok=False)
            return
        self.plan_failures = 0
        self.path = np.asarray(path, dtype=float)
        self.tracker.set_path(self.path)
        if isinstance(move, ObjectMove) and move.target_odom_xy is not None:
            tx, ty = move.target_odom_xy
            move.psi_c = math.atan2(ty - pose[1], tx - pose[0])
        self.get_logger().info(
            f'PLAN | {why}: {len(self.path)} pts, {self.tracker.length:.2f} m, '
            f'{dt_ms:.1f} ms, {len(obstacles)} obstacles', throttle_duration_sec=0.5)
        self._publish_status(why, dt_ms, ok=True)
        self._publish_path()

    def _publish_status(self, why: str, dt_ms: float, ok: bool):
        self.status_pub.publish(String(data=json.dumps({
            'node': self.get_name(), 'move': self.move.kind if self.move else None,
            'reason': why, 'ok': ok, 'plan_ms': round(dt_ms, 2),
            'path_m': round(self.tracker.length, 3), 'failures': self.plan_failures})))

    def _publish_path(self):
        stamp = self.get_clock().now().to_msg()
        msg = Path()
        msg.header.stamp = stamp
        msg.header.frame_id = self.odom_frame
        for i, (x, y) in enumerate(self.path):
            ps = PoseStamped()
            ps.header = msg.header
            ps.pose.position.x, ps.pose.position.y = float(x), float(y)
            if i + 1 < len(self.path):
                yaw = math.atan2(self.path[i + 1][1] - y, self.path[i + 1][0] - x)
            else:
                yaw = 0.0
            ps.pose.orientation.z = math.sin(yaw / 2.0)
            ps.pose.orientation.w = math.cos(yaw / 2.0)
            msg.poses.append(ps)
        self.path_pub.publish(msg)
        markers = MarkerArray()
        markers.markers.append(self.line_marker(0, 'path', self.path, stamp,
                                                (0.1, 0.9, 0.2, 1.0), 0.04))
        markers.markers.extend(self.extra_markers(stamp))
        self.markers_pub.publish(markers)

    def line_marker(self, marker_id: int, ns: str, points, stamp, rgba, width: float,
                    kind: int = Marker.LINE_STRIP) -> Marker:
        """Build a LINE_STRIP (or LINE_LIST) marker in the odom frame."""
        m = Marker()
        m.header.stamp = stamp
        m.header.frame_id = self.odom_frame
        m.ns = ns
        m.id = marker_id
        m.type = kind
        m.action = Marker.ADD
        m.scale.x = width
        m.color.r, m.color.g, m.color.b, m.color.a = rgba
        m.pose.orientation.w = 1.0
        m.lifetime.sec = 2
        m.points = [Point(x=float(x), y=float(y), z=0.02) for x, y in points]
        return m

    def destroy_node(self):
        """Close the polled TF before the node."""
        try:
            self.tf.close()
        finally:
            super().destroy_node()


def run_node(node_cls, args=None):
    """Spin a planner-controller until shutdown; send a last zero if the context allows."""
    rclpy.init(args=args)
    node = node_cls()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception:  # noqa: BLE001 -- a signal mid-spin invalidates the context
        if rclpy.ok():
            raise
    finally:
        if rclpy.ok():
            node.publish_drive(0.0, 0.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
