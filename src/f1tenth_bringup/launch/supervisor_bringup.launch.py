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

FASTRTPS_DEFAULT_PROFILES_FILE (old jazzy line, fix batch 5, backlog H1) is set
right after ROS_DISCOVERY_SERVER, the same way and for the same reach:
config/fastdds_profile.xml (5 s participant lease) for the Discovery Server,
the supervisor and every component. Launch argument fastdds_profile overrides
the file; scripts/sim/sim_isaac.sh passes its own sim profile (same lease)
through it. scripts/env/jazzy.sh sets the same file for hand-started tools.

Sim mode (sim:=isaac; default sim:=none = the car, unchanged): the SAME stack drives
the Isaac Sim car. The supervisor drops the device-driver components' launch files
(vesc.launch.py, lidar.launch.py; camera.launch.py -> sim_camera_tf.launch.py, the
ZED frames only) and starts every component through sim_component.launch.py, which
puts all its nodes on /clock (use_sim_time true) and loads config/sim/isaac.yaml
(sim_overlay) and sim_overlay_extra after the normal parameters. use_sim_time:=true
alone (sim:=none) only switches the clock. The supervisor itself stays on wall
time (restart timers, watchdog). Normally started by scripts/sim/sim_isaac.sh;
output/jazzy_port/phase3.md has the design and the default-path identity evidence.
"""

import os

from ament_index_python.packages import get_package_share_directory

from f1tenth_params.param_defaults import get_default, get_path_default

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    LogInfo,
    OpaqueFunction,
    SetEnvironmentVariable,
)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


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

    # Fast DDS participant profile (5 s lease) for every participant this
    # launch tree starts: the Discovery Server below, the supervisor, and every
    # component it spawns (they inherit this environment, like
    # ROS_DISCOVERY_SERVER). Overrides whatever the shell had, so the stack
    # cannot run without it. See config/fastdds_profile.xml for why.
    fastdds_profile_default, fastdds_profile_desc = get_path_default('fastdds_profile')
    fastdds_profile_la = DeclareLaunchArgument(
        'fastdds_profile', default_value=fastdds_profile_default,
        description=fastdds_profile_desc)
    fastdds_profile_env = SetEnvironmentVariable(
        'FASTRTPS_DEFAULT_PROFILES_FILE', LaunchConfiguration('fastdds_profile'))

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
    # DeclareLaunchArgument (unlike the 4 stack-wide branching args, which are
    # plain-Python get_value() reads with no CLI override -- see f1tenth_params/
    # param_defaults.py's own docstring) so `calibration:=false`/`:=true` on the
    # CLI actually reaches component_supervisor_node below, same as stack_
    # bringup.launch.py's own separate `calibration` DeclareLaunchArgument does
    # for its path.
    calibration_default, calibration_desc = get_default('calibration')
    calibration_la = DeclareLaunchArgument(
        'calibration', default_value=str(calibration_default).lower(),
        description=calibration_desc)

    # component-auto-start pass: same "real DeclareLaunchArgument, not one of the
    # 4 stack-wide branching values" treatment as calibration directly above --
    # stack_params.yaml's own enable_intelligence key only supplies the default;
    # enable_intelligence:=true/false on the CLI actually reaches component_
    # supervisor_node below (see that node's own self.enable_intelligence and
    # stack_params.yaml's enable_intelligence comment for the full reasoning,
    # including why this isn't just a baked-in-at-parse-time stack-wide value).
    # mission-logger-opt-in pass: the same treatment again, for the same
    # reason -- the supervisor forwards only components.yaml's args, so this
    # has to arrive as a declared parameter or `enable_mission_logger:=true`
    # on this command line would do nothing at all.
    enable_mission_logger_default, enable_mission_logger_desc = get_default(
        'enable_mission_logger')
    enable_mission_logger_la = DeclareLaunchArgument(
        'enable_mission_logger',
        default_value=str(enable_mission_logger_default).lower(),
        description=enable_mission_logger_desc)

    # Rust ports (output/rust/): the same "declared argument -> supervisor
    # parameter -> component launch argument" path as enable_mission_logger,
    # for the same reason. guard -> behavior_bringup.launch.py proximity_guard,
    # impl -> costmap.launch.py costmap_boundary_impl. Defaults keep the stack
    # exactly as it is without them: no guard, the Python costmap_boundary_node.
    guard_la = DeclareLaunchArgument(
        'guard', default_value='off', choices=['off', 'shadow', 'active'],
        description='Rust front-proximity guard (f1tenth_proximity_guard): off (not '
                    'launched), shadow (decisions only), active (also stops on the '
                    'safety_stop lane). The emergency lane of the BT is unchanged in every mode.')
    impl_la = DeclareLaunchArgument(
        'impl', default_value='py', choices=['py', 'rust'],
        description='costmap_boundary_node implementation: py (f1tenth_costmap) or rust '
                    '(f1tenth_costmap_boundary_rs). Never both: same node name and topics.')
    # corridor_impl -> navigation.launch.py corridor_impl -> mpc_corr.launch.py:
    # which implementation builds MPC_corr's corridor geometry (R1). Default
    # from stack_params.yaml (py), the same as mpc_corr.launch.py's own.
    corridor_impl_default, corridor_impl_desc = get_default('corridor_impl')
    corridor_impl_la = DeclareLaunchArgument(
        'corridor_impl', default_value=str(corridor_impl_default), choices=['py', 'rust'],
        description=corridor_impl_desc)
    # mpc_solver_impl -> navigation.launch.py mpc_solver_impl -> mpc_corr.launch.py:
    # which implementation solves the MPC's RTI tick (R2). Default from
    # stack_params.yaml (py), the same as mpc_corr.launch.py's own.
    mpc_solver_impl_default, mpc_solver_impl_desc = get_default('mpc_solver_impl')
    mpc_solver_impl_la = DeclareLaunchArgument(
        'mpc_solver_impl', default_value=str(mpc_solver_impl_default), choices=['py', 'rust'],
        description=mpc_solver_impl_desc)
    # hard_stop_impl -> ackermann_mux.launch.py hard_stop_impl (the `control`
    # component): which implementation runs hard_stop_node (R4). Default from
    # stack_params.yaml (py), the same as ackermann_mux.launch.py's own.
    hard_stop_impl_default, hard_stop_impl_desc = get_default('hard_stop_impl')
    hard_stop_impl_la = DeclareLaunchArgument(
        'hard_stop_impl', default_value=str(hard_stop_impl_default), choices=['py', 'rust'],
        description=hard_stop_impl_desc)

    enable_intelligence_default, enable_intelligence_desc = get_default('enable_intelligence')
    enable_intelligence_la = DeclareLaunchArgument(
        'enable_intelligence', default_value=str(enable_intelligence_default).lower(),
        description=enable_intelligence_desc)
    # f1tenth_comms: the same treatment, so `enable_comms:=true` here starts
    # the `comms` component with the stack (default from stack_params.yaml).
    enable_comms_default, enable_comms_desc = get_default('enable_comms')
    enable_comms_la = DeclareLaunchArgument(
        'enable_comms', default_value=str(enable_comms_default).lower(),
        description=enable_comms_desc)

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

    # Simulation mode. The defaults (sim:=none use_sim_time:=false) are the car: the
    # supervisor Node below is then built with exactly the parameters it always had, and no
    # sim parameter is passed (see _supervisor_node). With sim:=isaac the supervisor drops
    # the device drivers, swaps in the sim frame publishers and runs every component on
    # use_sim_time with the sim overlay (component_supervisor_node.apply_sim_mode,
    # sim_component.launch.py). Run it through scripts/sim/sim_isaac.sh.
    sim_la = DeclareLaunchArgument(
        'sim', default_value='none', choices=['none', 'isaac'],
        description="'none': the car (default). 'isaac': Isaac Sim (~/isaac_ws) replaces the "
                    'VESC, LiDAR and ZED drivers; every other component runs as on the car, on '
                    'use_sim_time, with config/sim/isaac.yaml loaded after the normal parameters.')
    use_sim_time_la = DeclareLaunchArgument(
        'use_sim_time', default_value='false', choices=['true', 'false'],
        description='Run every component on /clock (sim_component.launch.py). Forced true with '
                    'sim:=isaac. The supervisor itself stays on wall time (its watchdog).')
    sim_overlay_la = DeclareLaunchArgument(
        'sim_overlay',
        default_value=os.path.join(
            get_package_share_directory('f1tenth_bringup'), 'config', 'sim', 'isaac.yaml'),
        description='Sim parameter overlay, loaded after the normal parameters (sim:=isaac only).')
    sim_overlay_extra_la = DeclareLaunchArgument(
        'sim_overlay_extra', default_value='',
        description='Optional host overlay (e.g. config/sim/hosts/linus.yaml: CPU pinning for '
                    'an 8-core machine). Only its sim_component_args are used (component launch '
                    'arguments; node parameters belong in sim_overlay). sim:=isaac only.')

    def _supervisor_node(context):
        params = {
            'calibration': LaunchConfiguration('calibration'),
            'enable_mission_logger': LaunchConfiguration('enable_mission_logger'),
            # value_type=str: the supervisor declares these as STRING
            # parameters, and an untyped substitution is read back as YAML 1.1,
            # where guard's default `off` is the boolean false (rclpy then
            # refuses the override and the supervisor dies in __init__;
            # CLAUDE.md, "never on/off as string values").
            'proximity_guard': ParameterValue(LaunchConfiguration('guard'), value_type=str),
            'costmap_boundary_impl': ParameterValue(
                LaunchConfiguration('impl'), value_type=str),
            'corridor_impl': ParameterValue(
                LaunchConfiguration('corridor_impl'), value_type=str),
            'mpc_solver_impl': ParameterValue(
                LaunchConfiguration('mpc_solver_impl'), value_type=str),
            'hard_stop_impl': ParameterValue(
                LaunchConfiguration('hard_stop_impl'), value_type=str),
            'enable_intelligence': LaunchConfiguration('enable_intelligence'),
            'enable_comms': LaunchConfiguration('enable_comms'),
            'components_config': LaunchConfiguration('components_config'),
            'restart_timeout_sec': LaunchConfiguration('restart_timeout_sec'),
            'log_dir': LaunchConfiguration('log_dir'),
            'watchdog_period_sec': LaunchConfiguration('watchdog_period_sec'),
            'max_auto_restarts': LaunchConfiguration('max_auto_restarts'),
            'restart_budget_window_sec': LaunchConfiguration('restart_budget_window_sec'),
        }
        actions = []
        sim = LaunchConfiguration('sim').perform(context)
        use_sim_time = LaunchConfiguration('use_sim_time').perform(context) == 'true'
        if sim != 'none':
            if not use_sim_time:
                actions.append(LogInfo(
                    msg=f'[supervisor_bringup] sim:={sim} forces use_sim_time:=true'))
            params.update({
                'sim': sim,
                'component_use_sim_time': True,
                'sim_overlay': LaunchConfiguration('sim_overlay'),
                'sim_overlay_extra': LaunchConfiguration('sim_overlay_extra'),
            })
        elif use_sim_time:
            params['component_use_sim_time'] = True
        actions.append(Node(
            package='f1tenth_bringup',
            executable='component_supervisor_node',
            name='component_supervisor_node',
            output='screen',
            parameters=[params],
        ))
        return actions

    component_supervisor_node = OpaqueFunction(function=_supervisor_node)

    return LaunchDescription([
        discovery_server_address_la,
        discovery_server_port_la,
        discovery_server_env,
        fastdds_profile_la,
        fastdds_profile_env,
        discovery_server,
        calibration_la,
        enable_mission_logger_la,
        guard_la,
        impl_la,
        corridor_impl_la,
        mpc_solver_impl_la,
        hard_stop_impl_la,
        enable_intelligence_la,
        enable_comms_la,
        components_config_la,
        restart_timeout_la,
        log_dir_la,
        watchdog_period_la,
        max_auto_restarts_la,
        restart_budget_window_la,
        sim_la,
        use_sim_time_la,
        sim_overlay_la,
        sim_overlay_extra_la,
        component_supervisor_node,
    ])
