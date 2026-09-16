"""Drive to a semantically tracked object: tracks in, MPC goal pose out.

This is the piece that was missing, and it is small because everything either
side of it already existed and was simply not connected:

  /costmap/semantic_tracks   confirmed, map-frame, class-labelled tracks from
                             semantic_layer_node -- whole-frame Hungarian
                             association against predicted positions, EMA
                             position and velocity, hit/miss lifecycle.
  /mpc/goal_pose             mpc_corr drives to a map-frame point and reports
                             /mpc/goal_reached on arrival.

So "go to the person" needs no new controller, no new mission primitive and
no new tracker. It needs the track carried across to the goal topic.

Why tracks and not /camera/detections_3d
----------------------------------------
An earlier version of this node took the nearest raw detection of the target
class each frame. That is precisely the greedy scheme semantic_layer.py was
rewritten to remove: matching detections one at a time in arrival order lets
one detection steal the nearest track before a better-matching one is
considered, and matching against a last-seen position rather than a predicted
one makes genuine motion indistinguishable from a new object. It produced
duplicate and multiplying objects in live testing there, and it would produce
a goal that jumps between instances here.

It was also simply broken: it looked up a `map` transform through tf2, and
**no map->odom edge exists in the TF tree** -- getting to the map frame is a
two-hop job that semantic_layer_node already does. Consuming its output means
this node needs no transform at all: the tracks are map-frame, and so is the
pose topic.

What this node does NOT do, deliberately
----------------------------------------
* **No tracking.** semantic_layer_node owns that. One tracker in the system.
* **No steering.** mpc_corr does the driving; this only supplies the goal.
* **No mission gating.** While enabled it republishes the goal continuously,
  overriding whatever goal a mission move published. Run it for a go-to
  mission and nothing else.

Standoff
--------
The goal is placed `standoff` metres SHORT of the track, along the line from
the vehicle. mpc_corr's arrival tolerance is ~0.15 m, so a goal on the target
itself means driving to within 15 cm of a person. The standoff is what makes
this safe to point at a human.
"""

from __future__ import annotations

import math

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles
from std_msgs.msg import Bool, Float32, String
from vision_msgs.msg import Detection3DArray

_WARN_THROTTLE_S = 2.0


def standoff_goal(vehicle_xy, target_xy, standoff):
    """Goal point ``standoff`` metres short of the target, and the heading to it.

    Pure, so the one piece of geometry that can hurt someone is testable
    without a camera. Two properties matter and both are asserted in the
    tests: the goal never lands past the target (that would drive into it),
    and when the vehicle is already inside the standoff the goal collapses to
    the vehicle's own position rather than a point behind it -- mpc_corr
    would otherwise be asked to reverse toward a person.
    """
    vx, vy = vehicle_xy
    tx, ty = target_xy
    dx, dy = tx - vx, ty - vy
    distance = math.hypot(dx, dy)
    yaw = math.atan2(dy, dx) if distance > 1e-9 else 0.0

    if distance <= standoff or distance <= 1e-9:
        return (vx, vy), yaw, distance

    scale = (distance - standoff) / distance
    return (vx + dx * scale, vy + dy * scale), yaw, distance


def nearest_track(tracks, vehicle_xy, target_class, min_confidence):
    """Nearest track of the wanted class. ``tracks`` are (id, class, x, y, score).

    Nearest is a tie-break among already-associated tracks, not an
    association step -- semantic_layer_node has done that. With two people in
    view this still prefers whichever is closer, so the track id is published
    for diagnostics: an id that changes mid-approach means the goal jumped.
    """
    best = None
    for track_id, class_id, x, y, score in tracks:
        if class_id != target_class or score < min_confidence:
            continue
        distance = math.hypot(x - vehicle_xy[0], y - vehicle_xy[1])
        if best is None or distance < best[0]:
            best = (distance, track_id, (x, y))
    return best


class ObjectGoalBridge(Node):

    def __init__(self) -> None:
        super().__init__('object_goal_bridge')

        # Shadow by default: the first run should be a manual drive that
        # records what this WOULD have commanded, with nothing reaching
        # mpc_corr.
        self.declare_parameter('enabled', False)
        self.declare_parameter('target_class', 'person')
        self.declare_parameter('standoff', 1.0)
        self.declare_parameter('min_confidence', 0.5)
        self.declare_parameter('max_target_age', 0.8)
        self.declare_parameter('publish_rate_hz', 20.0)
        self.declare_parameter('tracks_topic', '/costmap/semantic_tracks')
        self.declare_parameter('pose_topic', '/ekf_global/odometry/filtered')
        self.declare_parameter('goal_topic', '/mpc/goal_pose')
        self.declare_parameter('goal_frame', 'map')

        gp = self.get_parameter
        self._enabled = bool(gp('enabled').value)
        self._target_class = str(gp('target_class').value)
        self._standoff = float(gp('standoff').value)
        self._min_confidence = float(gp('min_confidence').value)
        self._max_target_age = float(gp('max_target_age').value)
        self._goal_frame = str(gp('goal_frame').value)

        if self._standoff <= 0.0:
            raise ValueError('standoff must be > 0: a zero standoff drives '
                             'the vehicle into the target')

        self._tracks: list = []
        self._tracks_stamp: float = 0.0
        self._vehicle_xy: tuple[float, float] | None = None
        self._last_distance: float | None = None
        self._locked_id: str | None = None

        sensor_qos = QoSPresetProfiles.SENSOR_DATA.value
        self.create_subscription(
            Detection3DArray, gp('tracks_topic').value, self._on_tracks, 10)
        self.create_subscription(
            Odometry, gp('pose_topic').value, self._on_pose, sensor_qos)

        self._pub_goal = self.create_publisher(PoseStamped, gp('goal_topic').value, 10)
        self._pub_locked = self.create_publisher(Bool, '~/target_locked', 10)
        self._pub_distance = self.create_publisher(Float32, '~/target_distance', 10)
        self._pub_track_id = self.create_publisher(String, '~/target_track_id', 10)
        self._pub_state = self.create_publisher(String, '~/state', 10)

        rate = float(gp('publish_rate_hz').value)
        self.create_timer(1.0 / rate, self._on_timer)

        if not self._enabled:
            self.get_logger().warn(
                'SHADOW MODE: tracking the target and computing the goal, '
                'publishing NOTHING to the MPC. Set enabled:=true to actuate.')
        self.get_logger().info(
            f'object_goal_bridge up: target_class="{self._target_class}", '
            f'standoff={self._standoff:.2f} m, tracks '
            f'"{gp("tracks_topic").value}" + pose "{gp("pose_topic").value}" '
            f'-> "{gp("goal_topic").value}" at {rate:.1f} Hz')

    # -- ingest -----------------------------------------------------------

    def _on_tracks(self, msg: Detection3DArray) -> None:
        if msg.header.frame_id and msg.header.frame_id != self._goal_frame:
            # These are supposed to arrive already in the goal frame. If they
            # do not, transforming here would silently reintroduce the two-hop
            # map problem this node exists to avoid.
            self.get_logger().warn(
                f'tracks are in frame {msg.header.frame_id!r}, expected '
                f'{self._goal_frame!r}; ignoring them',
                throttle_duration_sec=_WARN_THROTTLE_S)
            return

        self._tracks = [
            (det.id,
             det.results[0].hypothesis.class_id,
             det.bbox.center.position.x,
             det.bbox.center.position.y,
             det.results[0].hypothesis.score)
            for det in msg.detections if det.results
        ]
        self._tracks_stamp = self.get_clock().now().nanoseconds * 1e-9

    def _on_pose(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        self._vehicle_xy = (p.x, p.y)

    # -- output -----------------------------------------------------------

    def _on_timer(self) -> None:
        now = self.get_clock().now().nanoseconds * 1e-9

        if self._vehicle_xy is None:
            return self._report('NO_POSE', locked=False)
        if not self._tracks:
            return self._report('SEARCHING', locked=False)
        if now - self._tracks_stamp > self._max_target_age:
            # Stale tracks: stop refreshing the goal. mpc_corr keeps the last
            # one it was given, so the mission's own timeout ends the move --
            # this node does not invent a goal from a stale track.
            return self._report('TRACKS_STALE', locked=False)

        best = nearest_track(self._tracks, self._vehicle_xy,
                             self._target_class, self._min_confidence)
        if best is None:
            return self._report('NO_TARGET_OF_CLASS', locked=False)

        distance, track_id, target_xy = best
        if self._locked_id is not None and track_id != self._locked_id:
            self.get_logger().warn(
                f'target track changed {self._locked_id} -> {track_id}: the '
                'goal just jumped to a different instance',
                throttle_duration_sec=_WARN_THROTTLE_S)
        self._locked_id = track_id

        (goal_x, goal_y), yaw, distance = standoff_goal(
            self._vehicle_xy, target_xy, self._standoff)
        self._last_distance = distance

        if self._enabled:
            message = PoseStamped()
            message.header.stamp = self.get_clock().now().to_msg()
            message.header.frame_id = self._goal_frame
            message.pose.position.x = goal_x
            message.pose.position.y = goal_y
            message.pose.orientation.z = math.sin(yaw / 2.0)
            message.pose.orientation.w = math.cos(yaw / 2.0)
            self._pub_goal.publish(message)

        self._report('TRACKING' if self._enabled else 'TRACKING_SHADOW',
                     locked=True)

    def _report(self, state: str, locked: bool) -> None:
        self._pub_state.publish(String(data=state))
        self._pub_locked.publish(Bool(data=locked))
        self._pub_track_id.publish(String(data=self._locked_id or ''))
        if self._last_distance is not None:
            self._pub_distance.publish(Float32(data=float(self._last_distance)))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ObjectGoalBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
