"""obstacle_clearance_node -> /obstacle_clearance (+ contact events on /safety/event).

OPTIONAL. Registered as the `obstacle_clearance` component in
f1tenth_bringup/config/components.yaml and listed in the supervisor's
_NEVER_AUTO_START: it is never launched at boot, and starts only on request,

    ./scripts/stackctl.py start obstacle_clearance

It exists for the test-campaign logger (f1tenth_logger test_campaign) and
nothing in the stack controls off it, so there is no reason to run it
outside a campaign session. Its OWN component for the same reason as
lidar_front_wall/wall_distance/swept_clearance: it reads /scan, and a
restart of `perception` would take urg_node -- the e-stop's only /scan --
down with it. f1tenth_bringup/test/test_obstacle_clearance_component.py
fails if that placement changes.

THE VALUE is the signed distance from the car's footprint RECTANGLE to the
nearest valid return, in any direction: negative inside the body outline,
+inf with no return at all. Not swept_clearance's arc length along the path.

The footprint defaults to the same body swept_clearance uses, read from
stack_params.yaml's swept_clearance_body_* keys (URDF chassis mesh bounds,
not a tape measure), so the two nodes cannot disagree about the car's size
unless someone overrides one of them here.

cpu_affinity follows lidar_front_wall.launch.py and swept_clearance.launch.py:
core 5, off behavior_executor_node's core 4, which hosts the e-stop.
"""

from f1tenth_params.param_defaults import get_default, get_value

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    use_lidar_default, use_lidar_desc = get_default('use_lidar')
    front_x = float(get_value('swept_clearance_body_front_x_m'))
    rear_x = float(get_value('swept_clearance_body_rear_x_m'))
    half_width = float(get_value('swept_clearance_body_half_width_m'))

    # (name, default, value type, description)
    args = [
        ('footprint_length_m', front_x - rear_x, float,
         'Footprint length [m] along base_link x. Default: swept_clearance body '
         '(URDF chassis mesh bounds, not measured).'),
        ('footprint_width_m', 2.0 * half_width, float,
         'Footprint width [m] along base_link y, centred on y = 0.'),
        ('footprint_rear_x_m', rear_x, float,
         'Where the footprint tail sits in base_link x [m]; negative is behind '
         'base_link (the rear axle, per the URDF).'),
        ('scan_topic', '/scan', str, 'LaserScan input.'),
        ('clearance_topic', '/obstacle_clearance', str, 'Float32 output, one per scan.'),
        ('publish_contact_events', True, bool,
         "Publish {event: 'contact'} on /safety/event when the clearance first "
         'drops to contact_threshold_m.'),
        ('contact_threshold_m', 0.0, float, 'Clearance at or below which it is contact.'),
        ('contact_rearm_m', 0.05, float,
         'The clearance must rise this far above the threshold before another '
         'contact event can fire.'),
    ]
    declared = [DeclareLaunchArgument(
        'use_lidar', default_value=str(use_lidar_default), description=use_lidar_desc)]
    params = {}
    for name, default, value_type, description in args:
        text = str(default).lower() if isinstance(default, bool) else str(default)
        declared.append(DeclareLaunchArgument(
            'obstacle_clearance_' + name, default_value=text, description=description))
        params[name] = ParameterValue(
            LaunchConfiguration('obstacle_clearance_' + name), value_type=value_type)

    declared.append(DeclareLaunchArgument(
        'obstacle_clearance_cpu_affinity', default_value='5',
        description="Comma-separated core ids to pin obstacle_clearance_node to via a "
                    "'taskset -c' launch prefix. Keep it off behavior_executor_node's "
                    'core (4), which hosts the e-stop.'))

    node = Node(
        condition=IfCondition(LaunchConfiguration('use_lidar')),
        package='f1tenth_perception',
        executable='obstacle_clearance_node',
        name='obstacle_clearance_node',
        output='screen',
        prefix=['taskset -c ', LaunchConfiguration('obstacle_clearance_cpu_affinity')],
        parameters=[params],
    )

    return LaunchDescription([*declared, node])
