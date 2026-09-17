#!/usr/bin/env python3
"""Watch the confirmed semantic tracks, and a go_to approach, from a second terminal.

For the 2026-09-18 floor session (docs/procedures/2026-09-18_go_to_floor.md).
Not in any launch file: run it by hand, next to the stack.

At 2 Hz it prints one line per confirmed track on /costmap/semantic_tracks:

    #12   person  0.91  range 2.34 m  bearing +12.3 deg  gap 1.52 m  age 0.18 s  TARGET

  range, bearing  from base_link to the track's centre; bearing is relative to
                  the car's heading, positive to the left.
  gap             the car's FRONT to the object's NEAR EDGE -- the quantity a
                  go_to_object move's gap_m commands and its scoring reports.
  age             now minus the tracks message's capture stamp.
  TARGET          the track the running go_to move is driving at.

During a go_to move it adds one line with mpc_corr's live view of the approach
from /mpc/object_status (r, gap, alpha, target_age, speed_ref, flags), plus the
move_id and target_class of the latest /mpc/goal_object.

THE SAME NUMBERS AS THE HANDLER, BY IMPORT. Tracks are parsed by GoToObject's
own tracks_from_message; the target radius is object_handler.track_radius (the
fused width / 2, else the class nominal); the gap is
object_handler.gap_for_centre_distance with the handler's nose_reach
(object_geometry via gap_limits). Nothing is re-derived here.

THE CAR POSE IS THE HANDLER'S: /ekf_global/odometry/filtered, the map-frame
global EKF that CheckStopCondition writes to the blackboard and GoToObject
reads. The handler does not look up TF, so neither does this. With no pose in
the last POSE_MAX_AGE_S the track lines print NO POSE instead of numbers.

Works under the Fast-DDS Discovery Server: it is a plain rclpy node, which
discovers the graph where the ros2 CLI does not. Give it ~10 s after starting
before trusting an empty track list.

Options:
    --class person          only tracks of that class
    --csv /tmp/run1.csv     every printed row, timestamped; refused under
                            ~/f1tenth_archive (the archive is not written to)
    --markers               also publish one RViz text label per confirmed track
                            on /debug/semantic_track_labels, e.g.
                            "person #12 gap 0.93 m"

RViz, to see the labels (Fixed Frame: map), add this display to the .rviz file
under Visualization Manager -> Displays, or Add -> By topic ->
/debug/semantic_track_labels -> MarkerArray:

    - Class: rviz_default_plugins/MarkerArray
      Enabled: true
      Name: semantic track labels
      Namespaces:
        semantic_track_labels: true
      Topic:
        Depth: 5
        Durability Policy: Volatile
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /debug/semantic_track_labels
      Value: true

Usage:
    source install/setup.bash
    python3 scripts/watch_objects.py [--class person] [--csv /tmp/x.csv] [--markers]
"""

import argparse
import csv
import math
import os
import sys
import time
import zlib
from dataclasses import dataclass
from typing import Optional

PRINT_PERIOD_S = 0.5
POSE_MAX_AGE_S = 0.5
GOAL_MAX_AGE_S = 1.0
STATUS_MAX_AGE_S = 1.0
LABEL_TOPIC = '/debug/semantic_track_labels'
LABEL_NS = 'semantic_track_labels'
ARCHIVE_ROOT = os.path.expanduser('~/f1tenth_archive')

CSV_FIELDS = (
    'wall_time', 'kind', 'track_id', 'class', 'score', 'range_m', 'bearing_deg',
    'gap_m', 'age_s', 'target', 'move_id', 'r', 'status_gap_m', 'alpha_deg',
    'target_age_s', 'speed_ref', 'flags')


# ------------------------------------------------------------------ pure part

@dataclass(frozen=True)
class Pose:
    """The car in the map frame, and when that estimate arrived."""

    x: float
    y: float
    yaw: float
    received_sec: float


@dataclass(frozen=True)
class TrackRow:
    """One printed track line. Geometry fields are None without a pose."""

    track_id: str
    class_id: str
    score: float
    age_s: float
    range_m: Optional[float]
    bearing_deg: Optional[float]
    gap_m: Optional[float]
    target: bool
    x: float
    y: float


def _wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def locked_track_id(tracks, goal, follow_gate_m):
    """Return the id of the track the go_to move is driving at, or None.

    The goal's own track_id when it carries one; otherwise the nearest track of
    the goal's class within the handler's follow gate of the goal point -- the
    handler's FOLLOW rule, by import.
    """
    if goal is None:
        return None
    if goal.track_id:
        return goal.track_id
    from f1tenth_behavior.mission.object_handler import nearest_track
    track = nearest_track(tracks, goal.target_class, (goal.x, goal.y), within=follow_gate_m)
    return track.track_id if track is not None else None


def track_rows(tracks, pose, now_sec, nose_reach_m, class_filter=None, target_id=None):
    """Build the printed rows: one per track, nearest first when there is a pose."""
    from f1tenth_behavior.mission.object_handler import gap_for_centre_distance, track_radius
    rows = []
    for track in tracks:
        if class_filter and track.class_id != class_filter:
            continue
        range_m = bearing = gap = None
        if pose is not None:
            dx, dy = track.x - pose.x, track.y - pose.y
            range_m = math.hypot(dx, dy)
            bearing = math.degrees(_wrap(math.atan2(dy, dx) - pose.yaw))
            gap = gap_for_centre_distance(range_m, nose_reach_m, track_radius(track))
        rows.append(TrackRow(
            track_id=str(track.track_id), class_id=track.class_id, score=track.score,
            age_s=now_sec - track.stamp_sec, range_m=range_m, bearing_deg=bearing,
            gap_m=gap, target=target_id is not None and str(track.track_id) == str(target_id),
            x=track.x, y=track.y))
    rows.sort(key=lambda row: (row.range_m is None, row.range_m or 0.0, row.track_id))
    return rows


def format_track_row(row):
    """Return one track line; NO POSE in place of the geometry when there is none."""
    head = f'  #{row.track_id:<5} {row.class_id:<14} {row.score:4.2f}'
    if row.range_m is None:
        geometry = '  NO POSE'
    else:
        geometry = (f'  range {row.range_m:5.2f} m  bearing {row.bearing_deg:+6.1f} deg'
                    f'  gap {row.gap_m:5.2f} m')
    return f'{head}{geometry}  age {row.age_s:4.2f} s' + ('  TARGET' if row.target else '')


# Status flags worth a word, in the order they are printed.
STATUS_FLAGS = ('stop_latched', 'inside_turn_radius', 'target_behind',
                'target_behind_terminal', 'goal_watchdog', 'target_stale')


def status_flags(status):
    """Return the names of the status flags that are set."""
    return [name for name in STATUS_FLAGS if bool(getattr(status, name, False))]


def _num(value, fmt):
    return 'nan' if value is None or not math.isfinite(value) else format(value, fmt)


def format_status_line(status, goal):
    """Return the go_to line: the goal's move_id and class, the status's live values."""
    move_id = goal.move_id if goal is not None else status.move_id
    target_class = goal.target_class if goal is not None else status.target_class
    flags = ','.join(status_flags(status)) or '-'
    return (f'  GO_TO {move_id} [{target_class}]  r {_num(status.r, "+.3f")}'
            f'  gap {_num(status.gap, ".3f")} m'
            f'  alpha {_num(math.degrees(status.alpha), "+.1f")} deg'
            f'  target_age {_num(status.target_age_s, ".2f")} s'
            f'  speed_ref {_num(status.speed_ref, ".2f")}  flags {flags}')


def label_text(row):
    """Return an RViz label, e.g. 'person #12 gap 0.93 m'."""
    base = f'{row.class_id} #{row.track_id}'
    return base + (' NO POSE' if row.gap_m is None else f' gap {row.gap_m:.2f} m')


def marker_id(track_id):
    """Return a stable int marker id: the track id itself when numeric."""
    text = str(track_id)
    return int(text) if text.isdigit() else zlib.crc32(text.encode()) & 0x7FFFFFFF


def csv_path_allowed(path):
    """Refuse any path that resolves inside ~/f1tenth_archive."""
    real = os.path.realpath(os.path.expanduser(path))
    archive = os.path.realpath(ARCHIVE_ROOT)
    return not (real == archive or real.startswith(archive + os.sep))


def csv_track_record(wall_time, row):
    """Return the CSV dict for a printed track line."""
    return {'wall_time': f'{wall_time:.3f}', 'kind': 'track', 'track_id': row.track_id,
            'class': row.class_id, 'score': f'{row.score:.3f}',
            'range_m': '' if row.range_m is None else f'{row.range_m:.3f}',
            'bearing_deg': '' if row.bearing_deg is None else f'{row.bearing_deg:.2f}',
            'gap_m': '' if row.gap_m is None else f'{row.gap_m:.3f}',
            'age_s': f'{row.age_s:.3f}', 'target': int(row.target)}


def csv_status_record(wall_time, status, goal):
    """Return the CSV dict for a printed go_to line."""
    return {'wall_time': f'{wall_time:.3f}', 'kind': 'status',
            'move_id': goal.move_id if goal is not None else status.move_id,
            'class': goal.target_class if goal is not None else status.target_class,
            'track_id': status.track_id, 'r': f'{status.r:.3f}',
            'status_gap_m': '' if not math.isfinite(status.gap) else f'{status.gap:.3f}',
            'alpha_deg': f'{math.degrees(status.alpha):.2f}',
            'target_age_s': f'{status.target_age_s:.3f}',
            'speed_ref': f'{status.speed_ref:.3f}', 'flags': ','.join(status_flags(status))}


@dataclass(frozen=True)
class GoalView:
    """The fields of the latest /mpc/goal_object the watcher uses."""

    move_id: str
    target_class: str
    track_id: str
    x: float
    y: float
    received_sec: float


# ------------------------------------------------------------------ ROS part

def _build_node(args):
    import rclpy
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from vision_msgs.msg import Detection3DArray
    from visualization_msgs.msg import Marker, MarkerArray

    from f1tenth_messages.msg import ObjectApproachStatus, ObjectGoal

    from f1tenth_behavior.behaviours.go_to_object import tracks_from_message
    from f1tenth_params.object_geometry import gap_limits
    from f1tenth_params.param_defaults import get_value

    nose_reach_m = gap_limits(args.target_class or 'person').nose_reach
    follow_gate_m = float(get_value('object_follow_gate_m'))

    class WatchObjects(Node):

        def __init__(self):
            super().__init__('watch_objects')
            self.tracks, self.tracks_frame_ok = [], True
            self.pose = self.status = self.goal = None
            self.status_received = None
            self.create_subscription(
                Detection3DArray, '/costmap/semantic_tracks', self._on_tracks, 10)
            self.create_subscription(
                Odometry, '/ekf_global/odometry/filtered', self._on_pose, 10)
            self.create_subscription(
                ObjectApproachStatus, '/mpc/object_status', self._on_status, 10)
            self.create_subscription(ObjectGoal, '/mpc/goal_object', self._on_goal, 10)
            self.labels = (self.create_publisher(MarkerArray, LABEL_TOPIC, 5)
                           if args.markers else None)
            self.csv_file = self.csv_writer = None
            if args.csv:
                self.csv_file = open(os.path.expanduser(args.csv), 'a', newline='')
                self.csv_writer = csv.DictWriter(self.csv_file, fieldnames=CSV_FIELDS)
                if self.csv_file.tell() == 0:
                    self.csv_writer.writeheader()
            self.create_timer(PRINT_PERIOD_S, self._on_timer)
            print(f'watch_objects: nose_reach {nose_reach_m:.4f} m, follow gate '
                  f'{follow_gate_m:.2f} m, pose from /ekf_global/odometry/filtered'
                  + (f', class {args.target_class}' if args.target_class else '')
                  + (f', csv {args.csv}' if args.csv else '')
                  + (f', labels on {LABEL_TOPIC}' if args.markers else ''), flush=True)

        def _now(self):
            return self.get_clock().now().nanoseconds * 1e-9

        def _on_tracks(self, msg):
            self.tracks_frame_ok = (not msg.header.frame_id) or msg.header.frame_id == 'map'
            self.tracks = tracks_from_message(msg) if self.tracks_frame_ok else []

        def _on_pose(self, msg):
            p, q = msg.pose.pose.position, msg.pose.pose.orientation
            yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
            self.pose = Pose(p.x, p.y, yaw, self._now())

        def _on_status(self, msg):
            self.status, self.status_received = msg, self._now()

        def _on_goal(self, msg):
            self.goal = GoalView(msg.move_id, msg.target_class, msg.track_id,
                                 msg.point.x, msg.point.y, self._now())

        def _on_timer(self):
            now = self._now()
            wall = time.time()
            pose = (self.pose if self.pose is not None
                    and now - self.pose.received_sec <= POSE_MAX_AGE_S else None)
            goal = (self.goal if self.goal is not None
                    and now - self.goal.received_sec <= GOAL_MAX_AGE_S else None)
            target = locked_track_id(self.tracks, goal, follow_gate_m)
            rows = track_rows(self.tracks, pose, now, nose_reach_m,
                              args.target_class, target)
            stamp = time.strftime('%H:%M:%S', time.localtime(wall)) + f'.{int(wall * 10) % 10}'
            print(f'{stamp}  tracks {len(rows)}'
                  + ('' if self.tracks_frame_ok else '  (tracks not in map frame: ignored)')
                  + ('' if pose is not None else '  NO POSE'), flush=True)
            for row in rows:
                print(format_track_row(row), flush=True)
                if self.csv_writer:
                    self.csv_writer.writerow(csv_track_record(wall, row))
            status_fresh = (self.status is not None and self.status_received is not None
                            and now - self.status_received <= STATUS_MAX_AGE_S)
            if goal is not None and status_fresh:
                print(format_status_line(self.status, goal), flush=True)
                if self.csv_writer:
                    self.csv_writer.writerow(csv_status_record(wall, self.status, goal))
            if self.csv_file:
                self.csv_file.flush()
            if self.labels is not None:
                self.labels.publish(self._markers(rows, Marker, MarkerArray))

        def _markers(self, rows, marker_cls, array_cls):
            array = array_cls()
            for row in rows:
                marker = marker_cls()
                marker.header.frame_id = 'map'
                marker.header.stamp = self.get_clock().now().to_msg()
                marker.ns, marker.id = LABEL_NS, marker_id(row.track_id)
                marker.type, marker.action = marker_cls.TEXT_VIEW_FACING, marker_cls.ADD
                marker.pose.position.x, marker.pose.position.y = row.x, row.y
                marker.pose.position.z = 1.2
                marker.pose.orientation.w = 1.0
                marker.scale.z = 0.25
                marker.color.a = 1.0
                marker.color.r, marker.color.g, marker.color.b = (
                    (1.0, 0.85, 0.0) if row.target else (1.0, 1.0, 1.0))
                marker.lifetime.sec, marker.lifetime.nanosec = 1, 0
                marker.text = label_text(row) + ('  TARGET' if row.target else '')
                array.markers.append(marker)
            return array

        def close(self):
            if self.csv_file:
                self.csv_file.close()

    return rclpy, WatchObjects


def main(argv=None):
    """Parse the options, refuse an archive CSV path, and spin the watcher."""
    parser = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    parser.add_argument('--class', dest='target_class', default=None,
                        help='only tracks of this class')
    parser.add_argument('--csv', default=None,
                        help='append every printed row to this CSV (not under ~/f1tenth_archive)')
    parser.add_argument('--markers', action='store_true',
                        help=f'publish RViz text labels on {LABEL_TOPIC}')
    args = parser.parse_args(argv)
    if args.csv and not csv_path_allowed(args.csv):
        print(f'watch_objects: refusing --csv {args.csv}: it is inside {ARCHIVE_ROOT}, '
              'which is read-only for tools. Write it somewhere else, e.g. /tmp.',
              file=sys.stderr)
        return 2

    rclpy, node_cls = _build_node(args)
    from rclpy.executors import ExternalShutdownException
    rclpy.init()
    node = node_cls()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        # Ctrl-C: Humble's signal handler shuts the context down first, so
        # spin() raises ExternalShutdownException rather than KeyboardInterrupt.
        pass
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
