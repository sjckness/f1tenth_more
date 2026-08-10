"""F1TENTH stack bringup -- thin orchestrator.

Every node lives in its owning package's own launch file; this file only includes
those launch files -- including foxglove_bridge.launch.py and
startup_sequence.launch.py, both of which live in this package itself (f1tenth_bringup)
rather than being owned by any other f1tenth_* package.

The 5 stack-wide branching args (camera_source, localization_source, enable_llm,
use_behavior_tree, enable_nav2) are NOT DeclareLaunchArgument/LaunchConfiguration
anywhere in the workspace -- they're plain Python values read directly from
f1tenth_params/config/stack_params.yaml (via param_defaults.get_value) at parse
time below, and the return list is built with plain Python if/else instead of
IfCondition/UnlessCondition for them. The ONLY way to change one of these 5 is
editing stack_params.yaml; passing e.g. `enable_nav2:=false` on the CLI to this
file (or to any of the owning launch files that used to declare one of these 5
themselves) is silently ignored, since no launch argument by that name exists
anymore to receive it.

(enable_safety_stop / safety_stop_controller retired -- the BT's own
handle_obstacle lane, unconditional, already provides the same corridor-stop
protection without needing an opt-in flag or a second obstacle-stop mechanism
that could disagree with it. See f1tenth_behavior/behaviours/is_obstacle_detected.py.)

Every other param (owned by one node/launch file) still works as a normal
DeclareLaunchArgument with full CLI-override behavior -- only its default value
and description are sourced from stack_params.yaml instead of being hardcoded.
"""

import os

from f1tenth_params.param_defaults import get_value

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import UnlessCondition
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # ================================================================
    # 1. STACK-WIDE (the 5 branching args -- see module docstring)
    # ================================================================
    camera_source = get_value('camera_source')
    enable_llm = get_value('enable_llm')
    use_behavior_tree = get_value('use_behavior_tree')
    # enable_nav2 is NOT read here -- like localization_source (see section 3 below),
    # its branch now lives entirely in the file that owns it: f1tenth_navigation/
    # navigation.launch.py, included unconditionally in section 6 below.

    # calibration is NOT one of the 5 -- it's declared in f1tenth_hardware/launch/
    # vesc.launch.py (included via vesc_bringup below, unconditionally and first),
    # stays a real DeclareLaunchArgument/LaunchConfiguration with normal CLI-override
    # behavior. is_calibration_disabled is still a runtime condition (calibration's
    # value isn't known until launch time, unlike the 6 above).
    is_calibration_disabled = UnlessCondition(LaunchConfiguration('calibration'))

    def include(package, launch_file, condition=None, **launch_arguments):
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory(package), 'launch', launch_file)
            ),
            launch_arguments=launch_arguments.items(),
            condition=condition,
        )

    # ================================================================
    # 2. HARDWARE
    # Ackermann->VESC chain, odometry, driver, IMU TF. Must launch
    # first: registers the `calibration` arg that section 3 depends on.
    # ================================================================
    vesc_bringup = include('f1tenth_hardware', 'vesc.launch.py')

    # ================================================================
    # 3. LOCALIZATION / TF
    # Establishes map -> odom (EKF or raw fallback) and static sensor
    # TFs. Deferred during calibration -- see vesc_bringup above.
    # localization_source branching now lives entirely in
    # f1tenth_localization/launch/localization.launch.py -- this file no
    # longer keeps its own duplicate copy of that if/else.
    # ================================================================
    localization_bringup = include(
        'f1tenth_localization', 'localization.launch.py', condition=is_calibration_disabled)
    # NOTE: localization_bringup (above) already includes description.launch.py
    # itself as part of f1tenth_localization/localization.launch.py's own "one
    # restartable unit" bundle (map -> odom + all sensor TFs). This second, direct
    # include is therefore redundant whenever calibration is disabled -- pre-existing
    # from when this was a lighter-weight sensor_tf_launch.py-only include (kept
    # unconditional so the static TF is still available during calibration:=true,
    # when localization_bringup itself is deferred -- see is_calibration_disabled
    # above). Now that description.launch.py also starts robot_state_publisher, the
    # redundancy is heavier than before; flagged here rather than silently resolved,
    # since removing it would drop TF availability during the calibration window.
    description_bringup = include('f1tenth_description', 'description.launch.py')

    # ================================================================
    # 4. PERCEPTION
    # Camera source (ZED2 or webcam) and YOLO 2D/3D detection fusion.
    # camera_source is read directly from stack_params.yaml by camera.launch.py /
    # detection.launch.py themselves now, so it's not forwarded as a
    # launch_arguments override here anymore.
    # ================================================================
    camera_bringup = include('f1tenth_perception', 'camera.launch.py')
    detection_bringup = include('f1tenth_perception', 'detection.launch.py')

    # ================================================================
    # 5. COMMAND / CONTROL
    # Ackermann mux. The MPC drive source (mpc_corr, enable_nav2:=false) is owned
    # by section 6's navigation_bringup now, alongside its Nav2 counterpart -- see
    # that section. safety_stop_controller (the opt-in reactive corridor-stop layer
    # that used to live here) was retired -- superseded by the BT's own
    # unconditional handle_obstacle lane (see section 6 / f1tenth_behavior).
    # ================================================================
    command_control_bringup = [include('f1tenth_control', 'ackermann_mux.launch.py')]

    # ================================================================
    # 6. AUTONOMY
    # navigation_bringup owns the enable_nav2 branch itself -- Nav2 (idles with no
    # goal) vs. mpc_corr, see f1tenth_navigation/navigation.launch.py -- mirroring
    # section 3's localization_bringup consolidation; this file no longer keeps its
    # own duplicate copy of that if/else. Deferred during calibration same as
    # localization_bringup. The BT supervisor turns Nav2's plans into drive commands
    # via the mux; independent of which process navigation_bringup itself started.
    # ================================================================
    autonomy_bringup = [
        include('f1tenth_navigation', 'navigation.launch.py', condition=is_calibration_disabled),
    ]
    if use_behavior_tree:
        autonomy_bringup.append(include('f1tenth_behavior', 'behavior_bringup.launch.py'))

    # ================================================================
    # 7. DIAGNOSTICS & INTELLIGENCE
    # system_observer.launch.py is self-gated behind enable_sys_obs (see that file's
    # own docstring) -- always included here, but a no-op Node-wise when disabled.
    # diagnostics_server.launch.py (continuous battery monitoring + on-demand
    # run_diagnostics service) is never gated, always-on. Plus the opt-in LLM stack.
    # ================================================================
    diagnostics_bringup = [
        include('f1tenth_diagnostics', 'system_observer.launch.py'),
        include('f1tenth_diagnostics', 'diagnostics_server.launch.py'),
    ]
    if enable_llm:
        diagnostics_bringup.append(include('llm', 'llm.launch.py'))

    # ================================================================
    # 8. DEV TOOLS / VISUALIZATION
    # foxglove_bridge.launch.py is self-toggled via enable_foxglove (default true) --
    # no separate inline Node() copy kept here anymore (previously duplicated
    # verbatim; now this is the one definition, same file the component supervisor's
    # dev_tools component also runs).
    # ================================================================
    foxglove_bringup = include('f1tenth_bringup', 'foxglove_bridge.launch.py')

    # ================================================================
    # 9. STARTUP SELF-CHECK
    # Brief steer-sweep (right -> left -> center) shortly after boot as a visual
    # "the stack is alive and the VESC is responding" check. Unconditional, same as
    # every other section here -- no enable/disable toggle was asked for on this one
    # (unlike foxglove_bridge above), so none was added.
    # ================================================================
    startup_sequence_bringup = include('f1tenth_bringup', 'startup_sequence.launch.py')

    return LaunchDescription([
        vesc_bringup,
        localization_bringup,
        description_bringup,
        camera_bringup,
        detection_bringup,
        *command_control_bringup,
        *autonomy_bringup,
        *diagnostics_bringup,
        foxglove_bringup,
        startup_sequence_bringup,
    ])
