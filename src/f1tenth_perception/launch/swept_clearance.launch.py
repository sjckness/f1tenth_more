"""swept_clearance_node -> /perception/swept_clearance.

REGISTERED as the `swept_clearance` component as of the d_wall pass. It is its
OWN component, never under `perception` and never folded into lidar.launch.py:
it reads /scan, and a component restart SIGINTs every launch file under it --
see lidar_front_wall.launch.py's module docstring for why that would leave the
e-stop without urg_node. f1tenth_bringup/test/test_wall_distance_component.py
fails if that placement changes.

REGISTERING IT IS NOT VALIDATING IT. Until that pass this file appeared in no
component and in no other launch file, so the node had NEVER RUN ON THE CAR --
and after that pass it still has not, because the LiDAR was powered down
throughout. Treat its first powered run as validation, not a formality; it is on
f1tenth_perception/README.md's UNVALIDATED list and it comes up in Stage 0 of
docs/bringup_checklist.md.

WHAT THE VALUE IS, because three nearby signals are all called something like
"clearance" and getting it wrong is a documented recurring error in this stack
(front_clearance_node.py:36-56). /perception/swept_clearance is the ARC LENGTH
THE REAR AXLE TRAVELS before the body RECTANGLE first contacts a point: metres
of TRAVEL, not a perpendicular gap, and not a sensor range. Straight ahead it is
the gap to the front bumper (0.443 m ahead of the axle), not to the axle. A wall
abeam at 0.6 m reports the 5 m horizon, not 0.6 m. ANY CONSUMER READING IT AS A
DISTANCE-TO-WALL IS WRONG -- for that, read /perception/d_wall (signed lateral,
wall_distance_node) or /perception/lidar_front_wall (unsigned perpendicular).
Nothing subscribes to this topic yet.

IT IS POINT-BASED, AND THAT IS ITS LIMIT. It only ever sees where beams
currently land. wall_distance_node's tracked wall is a fitted line that
deliberately extends past that -- which is the whole reason the wall is tracked
rather than re-fitted -- so that extension does not reach this node. It reaches
the same swept_corridor.clearance() geometry instead, from inside
wall_distance_node, by sampling the segment at glass_point_spacing and
publishing the result on /perception/d_wall/swept_arc. Sampling was chosen over
extending clearance() to accept a segment: sample_segment() already exists and
is unit-tested, while a segment overload would mean new geometry inside the one
module the stack's swept-clearance correctness rests on.

Every swept_clearance_* default and its description comes from
f1tenth_params/config/stack_params.yaml, passed with an explicit value_type
(see detection.launch.py for why launch_ros's string inference is not used).

The sensor switches are not stack params of their own: use_lidar is the
stack-wide LiDAR switch, and use_camera follows camera_source, because only
the ZED publishes depth. The webcam is monocular; a ground-plane homography
path for it is not built, so with camera_source 'webcam' the node runs on
the LiDAR alone.

cpu_affinity/nice follow lidar_front_wall.launch.py: core 5, off the behavior
executor's core 4 that hosts the e-stop, and shared with lidar_front_wall_node.
"""

from f1tenth_params.param_defaults import get_default, get_value

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

# (name without the swept_clearance_ prefix, value type).
SWEPT_CLEARANCE_ARGS = [
    ('wheelbase_m', float),
    ('body_front_x_m', float),
    ('body_rear_x_m', float),
    ('body_half_width_m', float),
    ('margin_m', float),
    ('max_range_m', float),
    ('absolute_min_clearance_m', float),
    ('rear_axle_x_m', float),
    ('steering_estimate', str),
    ('steering_lag_sec', float),
    ('steering_topic', str),
    ('lidar_timeout_sec', float),
    ('camera_timeout_sec', float),
    ('publish_rate_hz', float),
    ('camera_stride_px', int),
    ('camera_max_points', int),
    ('camera_min_depth_m', float),
    ('camera_max_depth_m', float),
    ('camera_z_min_m', float),
    ('camera_z_max_m', float),
]


def generate_launch_description():
    use_lidar_default, use_lidar_desc = get_default('use_lidar')
    declared = [DeclareLaunchArgument(
        'use_lidar', default_value=str(use_lidar_default), description=use_lidar_desc)]

    params = {
        'use_lidar': ParameterValue(LaunchConfiguration('use_lidar'), value_type=bool),
        'use_camera': get_value('camera_source') == 'zed',
        'depth_topic': '/zed2/zed_node/depth/depth_registered',
        'depth_info_topic': '/zed2/zed_node/depth/camera_info',
    }
    for name, value_type in SWEPT_CLEARANCE_ARGS:
        default, description = get_default('swept_clearance_' + name)
        declared.append(DeclareLaunchArgument(
            'swept_clearance_' + name, default_value=str(default), description=description))
        params[name] = ParameterValue(
            LaunchConfiguration('swept_clearance_' + name), value_type=value_type)

    declared.append(DeclareLaunchArgument(
        'swept_clearance_cpu_affinity', default_value='5',
        description="Comma-separated core ids to pin swept_clearance_node to via a "
                    "'taskset -c' launch prefix. Must stay a non-empty core list. Keep it "
                    "off behavior_executor_node's core (4), which hosts the e-stop."))
    declared.append(DeclareLaunchArgument(
        'swept_clearance_nice', default_value='0',
        description='Process niceness for swept_clearance_node. 0: no-op.'))
    params['nice'] = ParameterValue(LaunchConfiguration('swept_clearance_nice'), value_type=int)

    node = Node(
        package='f1tenth_perception',
        executable='swept_clearance_node',
        name='swept_clearance_node',
        output='screen',
        prefix=['taskset -c ', LaunchConfiguration('swept_clearance_cpu_affinity')],
        parameters=[params],
    )

    return LaunchDescription([*declared, node])
