"""Nav2 orchestrator -- replaces the old monolithic nav2_bringup.launch.py.

Each Nav2 server now lives in its own single-responsibility launch file
(map.launch.py, controller.launch.py, planner.launch.py,
behavior_server.launch.py, bt_navigator.launch.py); this file conditionally
includes each one based on its own enable_nav2_<component> value (see
stack_params.yaml -- all default true, so the current "everything up" behavior
is unchanged), plus the LiDAR include, exactly as the old monolithic file did.
This makes it possible to launch e.g. just the map by setting the other
enable_nav2_* keys false, without touching anything else.

Like enable_nav2 itself (the master on/off switch for this whole file, read by
stack_bringup.launch.py/component_supervisor_node.py -- unaffected by this
phase), the 5 enable_nav2_<component> values are read here as plain Python
values via get_value(), NOT DeclareLaunchArgument/LaunchConfiguration: they
decide which IncludeLaunchDescription actions even get constructed and which
names enter the lifecycle manager's node_names list below, both parse-time
decisions, not CLI-overridable through this file's own arguments.

Lifecycle management: kept as ONE shared lifecycle_manager_navigation here
(not one per component) -- Nav2's lifecycle_manager already supports a
dynamic managed_nodes list, so there's no need for N separate managers just to
get N independently-toggleable components. node_names below is built from
exactly the components that got included above, so e.g. running with only
enable_nav2_map:=true still lifecycle-manages (configures/activates) just
map_server correctly. The tradeoff: a component's own launch file, run
completely standalone (bypassing this file), has no lifecycle_manager of its
own and will sit `unconfigured` forever -- see each component file's own
docstring. That's judged acceptable since the point of the split is
selectability *through this orchestrator*, not turning each component into a
fully independent lifecycle-managed entry point.

Moved here from f1tenth_behavior/launch/navigation.launch.py (Nav2 nodes) and
f1tenth_navigation/launch/map_server_launch_old.py (map_server + its own standalone
lifecycle manager, merged in, then later split back out into map.launch.py) so
that f1tenth_behavior's own launch file (behavior_bringup.launch.py) no longer
needs to declare any Nav2 nodes itself -- it only depends on this file already
having been included. Enablement is controlled by the stack-wide `enable_nav2`
launch argument, not by use_behavior_tree: Nav2 idles with no active goal and
produces no drive command on its own, so it's safe to run independently of
which drive-command source is active.

TF ownership: map -> odom is published dynamically by the EKF (f1tenth_bringup/config/
ekf.yaml, world_frame: map). The static odom -> base_link (identity) TF the EKF needs
to compose that correction against used to live here, gated behind enable_nav2 -- a
real bug (found while making f1tenth_behavior's BT work with enable_nav2:=false, see
that package's behavior_bringup.launch.py): with Nav2 off, NOTHING published
odom -> base_link at all, not even map_only.launch.py's leaner /map-only path, so the
map -> odom -> base_link chain was broken regardless of localization_source, not just
for the EKF. It now lives in f1tenth_localization/launch/localization.launch.py
instead, unconditionally, alongside that file's other static sensor TFs -- this file no
longer publishes it. There is no static map -> odom transform here -- if the EKF node
isn't running, map is disconnected from odom/base_link entirely.
"""

import os

from ament_index_python.packages import get_package_share_directory

from f1tenth_params.param_defaults import get_default, get_path_default, get_value

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    navigation_share = get_package_share_directory('f1tenth_navigation')
    perception_share = get_package_share_directory('f1tenth_perception')

    default_map, map_desc = get_path_default('map', package='f1tenth_navigation')
    map_yaml_arg = DeclareLaunchArgument(
        'map', default_value=default_map, description=map_desc,
    )
    # use_sim_time is deliberately NOT sourced from stack_params.yaml -- sim and real
    # hardware need different defaults (see stack_params.yaml's header comment), so it
    # stays a local, per-file literal.
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true.',
    )
    autostart_default, autostart_desc = get_default('autostart')
    autostart_arg = DeclareLaunchArgument(
        'autostart', default_value=str(autostart_default), description=autostart_desc,
    )
    nav2_params_default, nav2_params_desc = get_path_default(
        'nav2_params_config', package='f1tenth_navigation')
    nav2_params_arg = DeclareLaunchArgument(
        'nav2_params', default_value=nav2_params_default, description=nav2_params_desc,
    )

    def include(package, launch_file, **launch_arguments):
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory(package), 'launch', launch_file)
            ),
            launch_arguments=launch_arguments.items(),
        )

    # map_server listed first: the lifecycle_manager below configures/activates managed
    # nodes in node_names order, so map_server reaches active before the costmaps'
    # static_layer (map_subscribe_transient_local: True) needs /map -- though
    # transient-local QoS means a late subscriber would still get the latched map
    # either way.
    component_includes = []
    lifecycle_nodes = []

    if get_value('enable_nav2_map'):
        component_includes.append(include(
            'f1tenth_navigation', 'map.launch.py',
            map=LaunchConfiguration('map'), use_sim_time=LaunchConfiguration('use_sim_time'),
        ))
        lifecycle_nodes.append('map_server')
    if get_value('enable_nav2_controller'):
        component_includes.append(include(
            'f1tenth_navigation', 'controller.launch.py',
            nav2_params=LaunchConfiguration('nav2_params'),
        ))
        lifecycle_nodes.append('controller_server')
    if get_value('enable_nav2_planner'):
        component_includes.append(include(
            'f1tenth_navigation', 'planner.launch.py',
            nav2_params=LaunchConfiguration('nav2_params'),
        ))
        lifecycle_nodes.append('planner_server')
    if get_value('enable_nav2_behavior_server'):
        component_includes.append(include(
            'f1tenth_navigation', 'behavior_server.launch.py',
            nav2_params=LaunchConfiguration('nav2_params'),
        ))
        lifecycle_nodes.append('behavior_server')
    if get_value('enable_nav2_bt_navigator'):
        component_includes.append(include(
            'f1tenth_navigation', 'bt_navigator.launch.py',
            nav2_params=LaunchConfiguration('nav2_params'),
        ))
        lifecycle_nodes.append('bt_navigator')

    lidar_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(perception_share, 'launch', 'lidar.launch.py')
        ),
    )

    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'autostart': LaunchConfiguration('autostart'),
            'node_names': lifecycle_nodes,
        }],
    )

    return LaunchDescription([
        map_yaml_arg,
        use_sim_time_arg,
        autostart_arg,
        nav2_params_arg,
        lidar_bringup,
        *component_includes,
        lifecycle_manager,
    ])
