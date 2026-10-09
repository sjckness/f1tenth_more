"""
The corridor's visual and logged forms: /mpc/corridor_markers and snapshots.

Message and record construction only. Publishing, the snapshot file handles
and the clock stay with the node; nothing here imports rclpy.

/mpc/corridor_markers is the MPC's own soft reference corridor (the corridor
build's xL/yL/xR/yR wall polylines + xc/yc centerline) as a MarkerArray for
Foxglove/RViz -- see the publisher's construction-site comment in MPC_corr's
__init__ for why MarkerArray (not PolygonStamped/Path) and why this is NOT the
same thing as /costmap/boundaries.

Frame: 'odom', not 'map' -- and this REMAINS correct after the map-frame
anchor fix, which is worth spelling out because that fix makes the
corridor's heading originate in map and so reads, at a glance, like it should
have moved these markers to 'map' too. It should not. Every number in the
corridor is an odom-frame coordinate by the time it gets here: the origin
X0/Y0 is the live pose from get_odom_topic()'s source (raw /odom or the LOCAL
EKF's /odometry/filtered -- see MPC_corr's _update_active_odom(), never the
global map-frame EKF), and the heading is the anchor CorridorManager has
already reprojected OUT of map and INTO this tick's odom frame. The anchor is
STORED in map and CONSUMED in odom; only the stored form is map-frame, and it
never reaches this module. Publishing under 'map' would therefore be wrong by
the entire map -> odom offset -- which reached 1.88 m of translation and 32.3
deg of yaw by the end of the 2026-09-07T12-48-40_mission-bottle_then_person
run that motivated the fix, so it is not a rounding-level distinction.

WHAT THIS MEANS FOR REPLAY, since that is the part that bit: these markers
line up with map-frame data (the costmap, the occupancy grid,
/ekf_global/odometry/filtered) only once the recorded map -> odom edge is
applied, which RViz/Foxglove do automatically from /tf -- present in the bag
at ~13.7 Hz. Comparing the raw marker coordinates against map-frame ones
WITHOUT applying that edge is exactly the measurement that made the pre-fix
corridor look 2.12 m adrift; the frame tag here is what tells a viewer (or an
analyst) to apply it.
"""

from pathlib import Path
from typing import Any, Dict, Optional

from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray

MARKER_FRAME = 'odom'

# (marker id, namespace, x key, y key, rgba) per polyline.
_POLYLINES = (
    (0, 'corridor_left', 'xL', 'yL', (0.2, 0.6, 1.0, 0.9)),
    (1, 'corridor_right', 'xR', 'yR', (1.0, 0.6, 0.2, 0.9)),
    (2, 'corridor_centerline', 'xc', 'yc', (0.8, 0.8, 0.8, 0.6)),
)


def line_marker(marker_id, ns, xs, ys, stamp_msg, rgba, update_period) -> Marker:
    """
    Build one LINE_STRIP Marker from parallel x/y arrays.

    A corridor wall or the centerline. Points are already absolute
    odom-frame coordinates (see this module's docstring), so pose stays
    identity -- no marker-local transform to apply. stamp_msg is a
    builtin_interfaces/Time; update_period is the corridor rebuild period
    the lifetime is derived from.
    """
    m = Marker()
    m.header.frame_id = MARKER_FRAME
    m.header.stamp = stamp_msg
    m.ns = ns
    m.id = marker_id
    m.type = Marker.LINE_STRIP
    m.action = Marker.ADD
    m.pose.orientation.w = 1.0
    m.scale.x = 0.03  # line width, meters
    m.color.r, m.color.g, m.color.b, m.color.a = rgba
    # Slightly longer than corridor_update_period so a normal rebuild
    # refreshes each marker well before it would expire, but a stalled
    # mpc_corr (crashed, or stuck on a slow tick) makes the corridor
    # visibly disappear from Foxglove/RViz instead of silently showing a
    # stale, no-longer-true corridor forever -- same "don't let a viz
    # element imply liveness it doesn't have" reasoning as this repo's
    # own front_clearance/min_obstacle_distance staleness gaps (see
    # f1tenth_behavior/README.md's "Dependency failure behavior"
    # section) -- except here the marker's own lifetime enforces it
    # directly, rather than relying on a consumer to notice.
    # builtin_interfaces/Duration splits into sec + nanosec, and nanosec is
    # a uint32 the message class asserts on: anything >= 4294967296 (~4.295s)
    # raises "The 'nanosec' field must be an unsigned integer in
    # [0, 4294967295]". Packing the WHOLE lifetime into nanosec therefore
    # crashed mpc_corr outright for any corridor_update_period above about
    # 1.43s -- and stack_params.yaml's own default is 10.0, so this fired on
    # the very first _publish_corridor_markers() call, every run: the
    # AssertionError propagates out of control_loop() through rclpy's
    # executor and kills the process, navigation crash-loops until the
    # supervisor's restart budget is gone, and mpc_corr then stays absent
    # from the graph -- which /mission/start_mission's own preflight
    # (f1tenth_behavior/mission/preflight.py) correctly reports as "node
    # 'mpc_corr' not found in the ROS graph", i.e. start_mission stops
    # working with no obvious connection to a marker-lifetime line.
    lifetime_sec = 3.0 * update_period
    m.lifetime.sec = int(lifetime_sec)
    m.lifetime.nanosec = int(round((lifetime_sec - int(lifetime_sec)) * 1e9))
    m.points = [Point(x=float(px), y=float(py), z=0.0) for px, py in zip(xs, ys)]
    return m


def corridor_marker_array(corridor, make_line) -> MarkerArray:
    """
    Assemble the three corridor polylines into one MarkerArray.

    make_line(marker_id, ns, xs, ys, rgba) builds each Marker -- the node
    passes its own _corridor_line_marker bound to the corridor's stamp, so
    the stamp and lifetime come from one place.
    """
    markers = MarkerArray()
    for marker_id, ns, x_key, y_key, rgba in _POLYLINES:
        markers.markers.append(make_line(
            marker_id, ns, corridor[x_key], corridor[y_key], rgba))
    return markers


def snapshot_record(t, robot, corridor, target, obstacles_global) -> Dict[str, Any]:
    """
    Build one corridor_debug.jsonl record.

    t is seconds since the epoch, so a snapshot can be lined up with the
    test folder's own streams (there was no timestamp at all before, which
    made these records impossible to align with anything). robot is
    (x, y, yaw, v); target the lookahead target the solve used.
    """
    x, y, yaw, v = robot
    return {
        "t": t,
        "robot": {
            "x": x,
            "y": y,
            "yaw": yaw,
            "v": v
        },
        "corridor": {
            "xc": corridor["xc"].tolist(),
            "yc": corridor["yc"].tolist(),
            "xL": corridor["xL"].tolist(),
            "yL": corridor["yL"].tolist(),
            "xR": corridor["xR"].tolist(),
            "yR": corridor["yR"].tolist(),
            "Pend": corridor["Pend"].tolist(),
        },
        "target": {
            "x": float(target[0]),
            "y": float(target[1])
        },
        "obstacles_world": [
            {"x": ox, "y": oy, "r": r}
            for (ox, oy, r) in obstacles_global
        ]
    }


def campaign_test_dir(status: Dict[str, Any]) -> Optional[Path]:
    """
    Return the open test's folder named by a logger status, or None.

    The test-campaign logger's status carries campaign_dir / mission /
    test_id, which is the test folder. None when no test is open (state not
    'open', or any of the three missing).
    """
    state = status.get('state')
    campaign_dir = status.get('campaign_dir')
    mission = status.get('mission')
    test_id = status.get('test_id')
    if (state == 'open' and campaign_dir and mission and test_id):
        return Path(campaign_dir) / str(mission) / str(test_id)
    return None
