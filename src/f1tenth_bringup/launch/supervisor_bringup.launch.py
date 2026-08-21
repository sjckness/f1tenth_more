"""Parallel bringup path: launches component_supervisor_node, which spawns every
component itself via subprocess (see component_supervisor_node.py's own docstring)
and exposes /restart_component (f1tenth_messages/srv/RestartComponent) and
~/control_component (f1tenth_messages/srv/ComponentControl) to restart, shut down,
or start any one of them independently. This file's only job is starting that one
node -- it is NOT a rewrite of stack_bringup.launch.py's grouping logic, and
stack_bringup.launch.py is untouched and still works as a single-process fallback.

Example:
  ros2 launch f1tenth_bringup supervisor_bringup.launch.py
  ros2 service call /restart_component f1tenth_messages/srv/RestartComponent \\
      "{component_name: 'navigation'}"
  ros2 service call /component_supervisor_node/control_component \\
      f1tenth_messages/srv/ComponentControl "{component_name: 'navigation', action: 0}"

Fast-DDS Discovery Server (EKF-pair-stall investigation follow-up -- see
stack_params.yaml's own discovery_server_address/_port comment for the full
root-cause writeup): started here, first, as a plain standalone process --
deliberately NOT a components.yaml entry, so component_supervisor_node's own
/restart_component has no way to touch it; it must outlive whatever component
churns, not get restarted along with it. ROS_DISCOVERY_SERVER is set via
SetEnvironmentVariable before component_supervisor_node's own Node() action
below, so component_supervisor_node itself picks it up, and -- since component_
supervisor_node.py's own subprocess.Popen() call never passes env= (confirmed
by reading it: plain Python default, inherit the full parent environment) --
every component IT spawns inherits it too, with zero changes needed there.
Also set in ~/.bashrc (shell-level, covers any node started by hand outside
this launch tree -- e.g. the manual `ros2 run`/`ros2 launch` commands this
whole investigation's own diagnostic scripts use) and in stack_bringup.launch.py
(the OTHER bringup path) -- deliberately in all three places: a partial
rollout, where only some nodes see this env var, would leave the untouched
ones still on SIMPLE discovery, defeating the point (they just wouldn't be
found by/wouldn't find the Discovery-Server nodes at all, a much worse failure
than the stall this is meant to fix).
"""

import os

from ament_index_python.packages import get_package_share_directory

from f1tenth_params.param_defaults import get_default, get_path_default

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    discovery_server_address_default, discovery_server_address_desc = get_default(
        'discovery_server_address')
    discovery_server_address_la = DeclareLaunchArgument(
        'discovery_server_address', default_value=str(discovery_server_address_default),
        description=discovery_server_address_desc)
    discovery_server_port_default, discovery_server_port_desc = get_default(
        'discovery_server_port')
    discovery_server_port_la = DeclareLaunchArgument(
        'discovery_server_port', default_value=str(discovery_server_port_default),
        description=discovery_server_port_desc)
    discovery_server_address = LaunchConfiguration('discovery_server_address')
    discovery_server_port = LaunchConfiguration('discovery_server_port')

    # Every subsequent action in this launch tree sees this -- see module
    # docstring's own Discovery Server paragraph for why this must come
    # before component_supervisor_node's own Node() action below.
    discovery_server_env = SetEnvironmentVariable(
        'ROS_DISCOVERY_SERVER', [discovery_server_address, ':', discovery_server_port])

    # respawn=True is this action's OWN resilience (if the server process
    # itself ever dies) -- unrelated to component_supervisor_node's restart
    # machinery, which never sees this process at all (see module docstring).
    #
    # cmd invokes scripts/ensure_discovery_server.py, NOT `fastdds discovery`
    # directly -- REAL BUG FOUND LIVE, twice (idempotency-fix pass): (1) the
    # very first version of this action passed cmd=['fastdds', 'discovery',
    # ...] directly and failed every time with OSError: [Errno 8] Exec
    # format error (/opt/ros/humble/bin/fastdds is a shebang-less shell
    # script; ExecuteProcess execs directly, no shell, unlike bash's own
    # silent ENOEXEC->/bin/sh fallback -- fixed at the time by wrapping in
    # '/bin/sh', '-c', '...'). (2) THAT fix was itself insufficient on its
    # own: this server is deliberately built to survive component_
    # supervisor_node restarts and outlive a launch session (see this
    # file's own module docstring) -- but nothing checked whether one was
    # ALREADY alive before starting a new one, so relaunching the stack
    # while an earlier session's server (still running, exactly as
    # designed) held the port produced "Discovery Server wasn't able to
    # allocate the specified listening port", then respawn=True retried
    # against that same still-occupied port forever. Confirmed NOT a PATH/
    # environment gap (a genuinely fresh interactive shell resolves fastdds/
    # fast-discovery-server identically to an already-working session, both
    # via /opt/ros/humble/bin per .bashrc) -- ensure_discovery_server.py is
    # the actual fix: checks once whether address:port already has a live
    # listener, reuses it if so (idles harmlessly, no error, no respawn
    # loop), or execs the real server if not -- see that script's own
    # module docstring for the full writeup, including the SAME Exec-
    # format-error bug it independently hit and fixed in its own exec call.
    ensure_discovery_server_path = os.path.join(
        get_package_share_directory('f1tenth_bringup'), 'scripts', 'ensure_discovery_server.py')
    discovery_server = ExecuteProcess(
        cmd=['python3', ensure_discovery_server_path,
             discovery_server_address, discovery_server_port],
        name='fastdds_discovery_server',
        output='screen',
        respawn=True,
        respawn_delay=2.0,
    )

    # calibration-single-source-of-truth pass: stack_params.yaml's own
    # `calibration` key is now the ONLY declaration of this value for this
    # bringup path -- components.yaml's `hardware` entry used to hardcode a
    # 'true' literal here independently, with no live way to override it (see
    # that file's own `hardware` entry comment). This is a real
    # DeclareLaunchArgument (unlike the 5 stack-wide branching args, which are
    # plain-Python get_value() reads with no CLI override -- see f1tenth_params/
    # param_defaults.py's own docstring) so `calibration:=false`/`:=true` on the
    # CLI actually reaches component_supervisor_node below, same as stack_
    # bringup.launch.py's own separate `calibration` DeclareLaunchArgument does
    # for its path.
    calibration_default, calibration_desc = get_default('calibration')
    calibration_la = DeclareLaunchArgument(
        'calibration', default_value=str(calibration_default).lower(),
        description=calibration_desc)

    components_config, components_config_desc = get_path_default('components_config')
    components_config_la = DeclareLaunchArgument(
        'components_config', default_value=components_config,
        description=components_config_desc)
    restart_timeout_default, restart_timeout_desc = get_default('restart_timeout_sec')
    restart_timeout_la = DeclareLaunchArgument(
        'restart_timeout_sec', default_value=str(restart_timeout_default),
        description=restart_timeout_desc)
    log_dir_default, log_dir_desc = get_default('log_dir')
    log_dir_la = DeclareLaunchArgument(
        'log_dir', default_value=str(log_dir_default), description=log_dir_desc)
    watchdog_period_default, watchdog_period_desc = get_default('watchdog_period_sec')
    watchdog_period_la = DeclareLaunchArgument(
        'watchdog_period_sec', default_value=str(watchdog_period_default),
        description=watchdog_period_desc)
    max_auto_restarts_default, max_auto_restarts_desc = get_default('max_auto_restarts')
    max_auto_restarts_la = DeclareLaunchArgument(
        'max_auto_restarts', default_value=str(max_auto_restarts_default),
        description=max_auto_restarts_desc)
    restart_budget_window_default, restart_budget_window_desc = get_default(
        'restart_budget_window_sec')
    restart_budget_window_la = DeclareLaunchArgument(
        'restart_budget_window_sec', default_value=str(restart_budget_window_default),
        description=restart_budget_window_desc)

    component_supervisor_node = Node(
        package='f1tenth_bringup',
        executable='component_supervisor_node',
        name='component_supervisor_node',
        output='screen',
        parameters=[{
            'calibration': LaunchConfiguration('calibration'),
            'components_config': LaunchConfiguration('components_config'),
            'restart_timeout_sec': LaunchConfiguration('restart_timeout_sec'),
            'log_dir': LaunchConfiguration('log_dir'),
            'watchdog_period_sec': LaunchConfiguration('watchdog_period_sec'),
            'max_auto_restarts': LaunchConfiguration('max_auto_restarts'),
            'restart_budget_window_sec': LaunchConfiguration('restart_budget_window_sec'),
        }],
    )

    return LaunchDescription([
        discovery_server_address_la,
        discovery_server_port_la,
        discovery_server_env,
        discovery_server,
        calibration_la,
        components_config_la,
        restart_timeout_la,
        log_dir_la,
        watchdog_period_la,
        max_auto_restarts_la,
        restart_budget_window_la,
        component_supervisor_node,
    ])
