"""VESC hardware chain: ackermann->VESC conversion, odometry, and the VESC
driver itself, plus the static base_link->imu TF for the VESC's onboard IMU.

All three sensor-driver nodes share one config file (vesc_config, defaults to
f1tenth_bringup/config/vesc.yaml -- the source of truth for VESC calibration
across the stack EXCEPT the 5 steering-calibration keys called out next).
vesc_driver_node and ackermann_to_vesc_node additionally load
steering_calibration_config (defaults to f1tenth_hardware/config/
steering_calibration.yaml) AFTER vesc_config in their own `parameters=[...]`
list -- ROS 2 merges multiple params files in the order given, later files
winning on a key collision, so this file's servo_min/servo_max/
steering_angle_to_servo_offset/steering_angle_to_servo_gain_left/_right win
over vesc_config's (which no longer defines them at all, to avoid two files
disagreeing -- see vesc.yaml's own note). This is the file vesc_tuning's
steering_calibration_node.py reads at startup and live-writes to during
calibration; it is NOT hot-reloaded by either running node (params are read
once at startup, no set-parameters callback in either), so a change here
only takes effect on that node's next restart. vesc_to_odom_node also loads
steering_calibration_config (for steering_angle_to_servo_offset -- it has no
default and the node aborts on startup without it); it reads
steering_angle_to_servo_gain (singular average, not the left/right pair)
straight from vesc_config, untouched by any of this -- see that key's
comment in vesc.yaml. If
vesc_driver_node dies during normal operation, the whole launch tree is shut
down (drive-by-wire has no meaning without it) -- see _build_driver_group()'s
with_crash_handler.

calibration:=true sequencing (stationary calibration, see
f1tenth_diagnostics' sensor_covariance_calibration_node and
gyro_bias_calibration_node -- run together here, as of the automatic-
calibration pass; previously this file only ran the covariance node):
  1. Driver group v1 (vesc_driver_node, vesc_to_odom_node, static IMU TF)
     launches immediately, unconditionally -- calibration needs LIVE sensor
     data to measure, so the drivers cannot be gated behind calibration
     completion (that was the circular bug in the first version of this).
  2. sensor_covariance_calibration_node AND gyro_bias_calibration_node
     launch concurrently (with each other, and with driver group v1),
     subscribing to driver group v1's topics. Both are stationary samplers
     that now gate their own sampling on a live stationary check (raw ERPM
     telemetry near zero for a short confirmation window -- see
     f1tenth_diagnostics.calibration_common.StationaryGate) before ever
     sampling, and both now exit cleanly with a real, distinguishable exit
     code either way (f1tenth_diagnostics.calibration_common.EXIT_*) --
     gyro_bias_calibration_node used to block forever (rclpy.spin(node),
     manual Ctrl+C required), which made it impossible to sequence
     automatically; that's fixed as part of the same pass that added this
     wiring.
  3. On BOTH calibration nodes' exit (closure-captured counter across 2
     OnProcessExit watchers, same technique as step 4 below): log each
     node's result (success, or the specific failure reason from its exit
     code) then -- regardless of success or failure -- shut down driver
     group v1's specific actions ONLY (matches_action), not a whole-launch
     Shutdown(), and not the same handler as the crash-Shutdown() below (v1
     deliberately has no crash handler; its exit here is expected, not a
     failure -- see _build_driver_group()). A failed/timed-out calibration
     does NOT block startup: neither node writes vesc.yaml on failure (each
     only calls its write step from its own success path), so vesc.yaml is
     left exactly as it already was -- the "last-known-good" fallback is
     structural, not a separate code path. This file's only job on failure
     is to log clearly WHY, then proceed exactly as it would on success.
  4. Once all 3 v1 processes have exited (a closure-captured counter across
     3 separate OnProcessExit watchers -- deterministic, unlike trusting a
     single representative process or guessing a bare timer for the async
     shutdown itself), a fixed 2.0s TimerAction buffer covers OS-level
     process-teardown latency (e.g. the kernel fully releasing
     vesc_driver_node's serial port) before driver group v2 launches. This
     buffer is NOT standing in for calibration duration (that stays
     event-driven, via calibration_duration_sec/gyro_sample_duration_sec) --
     it only bridges the gap between "OS reports the process exited" and
     "the device node is actually free to reopen."
  5. Driver group v2 (fresh instances -- launch actions are single-use, so
     _build_driver_group() is a factory called twice) launches WITH the
     crash handler this time, now reading vesc_config with whatever values
     calibration wrote into it (or the pre-existing values, on a fallback).
  6. Once v2's vesc_driver_node starts, ekf.launch.py (f1tenth_localization)
     and f1tenth_navigation's navigation.launch.py (Nav2 or mpc_corr, branching
     on enable_nav2 itself -- see that file) are included -- both are normally
     included unconditionally by stack_bringup.launch.py, but it defers them
     (UnlessCondition(calibration)) specifically so this file can release them
     at the right moment instead when calibration:=true.

vesc_yaml_path is passed explicitly to both calibration nodes here (via
f1tenth_diagnostics.calibration_common.resolve_source_vesc_yaml_path(), the
same helper calibration.launch.py already used) instead of left to each
node's own default. This is a real fix, not cosmetic: each node's own
default only resolves to the real source-tree vesc.yaml when the workspace
is built with --symlink-install, which this one is not (confirmed) -- left
unset, this path previously had both calibration nodes patch the
INSTALL-space copy of vesc.yaml instead of the source-tree file, which
happened to still work for that immediate run (vesc_config also resolves to
the same install-space copy, so v2 picked up the freshly-written values) but
silently discarded the calibration on the next `colcon build` (which
overwrites install/ from src/ again). This is very likely the exact bug
referenced by vesc.yaml's own comment history ("a prior calibration:=true
run's output wasn't reliably reaching the live driver process").

calibration default: as of the automatic-calibration pass, calibration
defaults to true (f1tenth_params/config/stack_params.yaml's calibration key)
-- both stationary calibrations now run automatically on every normal boot,
not just when explicitly requested. components.yaml's `hardware` component
(the one that auto-starts at supervisor startup) was updated the same way,
for the same reason components.yaml's `calibrate_hardware` component
(on-demand only, unaffected by this) already had `calibration: 'true'`
hardcoded. calibration:=false: driver group launches once (with the crash
handler, identical to the old always-on behavior), no calibration nodes, no
v1/v2 split -- stack_bringup.launch.py's own ekf_bringup/navigation_bringup
launch immediately, completely unaffected by anything in this file.

Battery voltage pre-flight REPORT (see f1tenth_diagnostics'
battery_voltage_check_node): runs BEFORE any of the above, regardless of
calibration:=true/false -- everything described above (the whole
calibration:=true/false split) is built unchanged and simply handed to this
step as "the full stack" to launch once the report is in, rather than
returned directly. Circular-dependency note: battery_voltage_check_node needs
a live vesc_driver_node to sample /sensors/core from, so it can't be gated
behind its own check (same shape as the calibration bug this file already
fixes once). Resolved with a standalone, temporary precheck vesc_driver_node
instance: once the check exits, ShutdownProcess just that instance, and once
IT exits, launch "the full stack" (built above, completely untouched) after
the usual OS-teardown buffer.

THIS IS NO LONGER A GATE, in any branch. It used to be: a non-zero exit from
battery_voltage_check_node meant the full stack was never launched for that
boot, with no retry, so ackermann_to_vesc_node and vesc_to_odom_node simply
did not exist while everything else came up healthy. That state was also
invisible from outside: the precheck vesc_driver_node was deliberately left
running, so this `ros2 launch` kept a live process to track and never exited
-- component_supervisor_node's watchdog polls for process exit, so it saw the
'hardware' component as up, logged nothing, and never restarted it. 323
archived boots took that path (308 of them because VESC telemetry had not
started yet, not because the pack was flat -- see battery_voltage_check_node's
own module docstring for the counts). The check is now advisory and always
exits 0; the returncode is logged here but no longer decides anything, so a
crash in the checker cannot silently cost the car its drive-by-wire either.
Battery protection lives in diagnostics_server_node + the BT's IsBatteryLow
emergency lane, which monitor continuously and are unchanged.
"""

import os

from ament_index_python.packages import get_package_share_directory

from f1tenth_diagnostics.calibration_common import resolve_source_vesc_yaml_path
from f1tenth_params.param_defaults import get_default, get_path_default

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    GroupAction,
    IncludeLaunchDescription,
    LogInfo,
    RegisterEventHandler,
    Shutdown,
    TimerAction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.events import matches_action
from launch.events.process import ShutdownProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# Fixed OS-level process-teardown buffer between "all v1 driver processes
# confirmed exited" and "launch v2" -- see module docstring step 4.
_TEARDOWN_BUFFER_SEC = 2.0

# Local copy of f1tenth_diagnostics.calibration_common.EXIT_REASONS -- launch
# files here treat launched nodes as opaque subprocesses (communicating only
# via exit code/params/topics), not via importing node internals, so this is
# kept as its own small mapping rather than imported. Keep in sync if the
# node-side exit codes ever change.
_CALIBRATION_EXIT_REASONS = {
    0: 'success',
    1: 'insufficient samples (no/too little data on the sampled topic(s) -- '
       'is the sensor publishing?)',
    2: 'stationary check failed or timed out (car never confirmed still, or '
       'the state topic never published)',
    3: 'ruamel.yaml not installed -- computed values were logged but vesc.yaml '
       'was NOT written',
    # Added by the calibration-safety-gates pass (Phase A) -- keep in sync with
    # calibration_common.EXIT_REASONS, per this dict's own comment above.
    4: 'measured value failed a post-sampling sanity bound (implausible '
       'magnitude or implausible delta from the value already in effect) -- '
       'see the node\'s own ERROR log line for both values. vesc.yaml was '
       'NOT written.',
    5: 'motion or vibration detected partway through the sampling window '
       '(ERPM speed or accelerometer deviation) -- see the node\'s own ERROR '
       'log line for when. vesc.yaml was NOT written.',
}


def _build_driver_group(vesc_config, steering_calibration_config, with_crash_handler):
    """Fresh instances of the 3 calibration-cycled sensor-driver actions
    (vesc_driver_node, vesc_to_odom_node, static_imu_tf_node). Called once
    per instance (v1: concurrent with calibration; v2: steady-state,
    launched after calibration writes fresh values) -- launch actions are
    single-use. Node names are kept identical across both calls
    deliberately: vesc.yaml's vesc_to_odom_node: block is keyed by that
    exact name (wheelbase has no code-side default -- a renamed node would
    fail to find it and crash at startup), and by construction v1 is fully
    torn down before v2 exists, so there's never two live nodes with the
    same name.

    with_crash_handler=False on v1: its ShutdownProcess-triggered exit
    (step 3) is expected, not a crash -- attaching the whole-launch
    Shutdown() handler there would tear down the entire stack on every
    calibration run. Only v2 (and the calibration:=false instance) runs for
    the rest of the launch's life and should trigger it.

    vesc_driver_node loads steering_calibration_config AFTER vesc_config (see
    module docstring) so servo_min/servo_max come from the single-source-of-
    truth file. vesc_to_odom_node also loads it (for
    steering_angle_to_servo_offset) -- see module docstring's last paragraph.

    Returns (actions, vesc_driver_node, vesc_to_odom_node, static_imu_tf_node).
    """
    vesc_to_odom_node = Node(
        package='vesc_ackermann',
        executable='vesc_to_odom_node_backup',
        name='vesc_to_odom_node',
        # Needs steering_calibration_config too: use_servo_cmd_to_calc_angular_velocity
        # requires BOTH steering_angle_to_servo_gain (still in vesc_config) AND
        # steering_angle_to_servo_offset (moved to steering_calibration_config -- see
        # module docstring). Without this, the node throws
        # UninitializedStaticallyTypedParameterException on 'steering_angle_to_servo_offset'
        # and aborts on startup (exit code -6), silently leaving /odom with no publisher
        # while vesc_driver_node/static_imu_tf keep running right next to it.
        parameters=[vesc_config, steering_calibration_config],
    )
    vesc_driver_node = Node(
        package='vesc_driver',
        executable='vesc_driver_node',
        name='vesc_driver_node',
        parameters=[vesc_config, steering_calibration_config],
        sigterm_timeout='5',
        sigkill_timeout='2',
    )
    # VESC IMU is rigidly mounted on the chassis; treat it as coincident with
    # base_link. The EKF needs this transform to bring sensors/imu/raw (frame
    # 'imu') into base_link. Adjust the offset if the IMU is calibrated.
    static_imu_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_baselink_to_imu',
        arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 'base_link', 'imu'],
    )
    actions = [vesc_to_odom_node, vesc_driver_node, static_imu_tf_node]
    if with_crash_handler:
        actions.append(RegisterEventHandler(
            OnProcessExit(target_action=vesc_driver_node, on_exit=[Shutdown()])
        ))
    return actions, vesc_driver_node, vesc_to_odom_node, static_imu_tf_node


def generate_launch_description():
    vesc_config_path, vesc_config_desc = get_path_default('vesc_config')
    vesc_la = DeclareLaunchArgument(
        'vesc_config', default_value=vesc_config_path, description=vesc_config_desc)
    steering_calibration_config_path, steering_calibration_config_desc = get_path_default(
        'steering_calibration_config', package='f1tenth_hardware')
    steering_calibration_la = DeclareLaunchArgument(
        'steering_calibration_config', default_value=steering_calibration_config_path,
        description=steering_calibration_config_desc)
    calibration_default, calibration_desc = get_default('calibration')
    calibration_la = DeclareLaunchArgument(
        'calibration', default_value=str(calibration_default), description=calibration_desc)
    calibration_duration_default, calibration_duration_desc = get_default(
        'calibration_duration_sec')
    calibration_duration_la = DeclareLaunchArgument(
        'calibration_duration_sec', default_value=str(calibration_duration_default),
        description=calibration_duration_desc)
    gyro_sample_duration_default, gyro_sample_duration_desc = get_default(
        'gyro_sample_duration_sec')
    gyro_sample_duration_la = DeclareLaunchArgument(
        'gyro_sample_duration_sec', default_value=str(gyro_sample_duration_default),
        description=gyro_sample_duration_desc)
    # calibration-safety-gates pass (Phase A): min_samples/gyro_bias_absolute_
    # bound/gyro_bias_delta_bound/gyro_bias_vibration_threshold were previously
    # NOT declared/forwarded here at all -- gyro_bias_calibration_node ran on
    # its own node-level defaults on this (automatic, boot-time) path, silently
    # disagreeing with calibration.launch.py's (manual) path, which already
    # forwarded min_samples from this same stack_params.yaml key. All four now
    # forwarded explicitly, same pattern as gyro_sample_duration_sec above --
    # see stack_params.yaml's own comments on each key for the full reasoning.
    min_samples_default, min_samples_desc = get_default('min_samples')
    min_samples_la = DeclareLaunchArgument(
        'min_samples', default_value=str(min_samples_default), description=min_samples_desc)
    gyro_bias_absolute_bound_default, gyro_bias_absolute_bound_desc = get_default(
        'gyro_bias_absolute_bound')
    gyro_bias_absolute_bound_la = DeclareLaunchArgument(
        'gyro_bias_absolute_bound', default_value=str(gyro_bias_absolute_bound_default),
        description=gyro_bias_absolute_bound_desc)
    gyro_bias_delta_bound_default, gyro_bias_delta_bound_desc = get_default(
        'gyro_bias_delta_bound')
    gyro_bias_delta_bound_la = DeclareLaunchArgument(
        'gyro_bias_delta_bound', default_value=str(gyro_bias_delta_bound_default),
        description=gyro_bias_delta_bound_desc)
    gyro_bias_vibration_threshold_default, gyro_bias_vibration_threshold_desc = get_default(
        'gyro_bias_vibration_threshold')
    gyro_bias_vibration_threshold_la = DeclareLaunchArgument(
        'gyro_bias_vibration_threshold',
        default_value=str(gyro_bias_vibration_threshold_default),
        description=gyro_bias_vibration_threshold_desc)
    # Explicit, reliably-source-anchored override -- see module docstring's
    # "vesc_yaml_path" paragraph for why this is needed (both calibration
    # nodes' own defaults only resolve to the source tree under
    # --symlink-install, which this workspace does not use).
    vesc_yaml_path_la = DeclareLaunchArgument(
        'vesc_yaml_path', default_value=resolve_source_vesc_yaml_path(),
        description='Real source-tree vesc.yaml that both calibration nodes back up '
                     'and patch in place with their measured values.')
    release_downstream_default, release_downstream_desc = get_default('release_downstream')
    release_downstream_la = DeclareLaunchArgument(
        'release_downstream', default_value=str(release_downstream_default),
        description=release_downstream_desc)

    vesc_config = LaunchConfiguration('vesc_config')
    steering_calibration_config = LaunchConfiguration('steering_calibration_config')
    is_calibration_enabled = IfCondition(LaunchConfiguration('calibration'))
    is_calibration_disabled = UnlessCondition(LaunchConfiguration('calibration'))
    # enable_nav2 is NOT read here anymore -- navigation.launch.py (included below via
    # navigation_include) now owns that branch itself, same as stack_bringup.launch.py's
    # own navigation_bringup (see that file's section 6). See navigation.launch.py's
    # own docstring.

    # Always launched immediately, unconditionally, outside the whole calibration
    # cycle -- it's a command-path node, not a sensor; nothing measures or restarts it.
    ackermann_to_vesc_node = Node(
        package='vesc_ackermann',
        executable='ackermann_to_vesc_node',
        name='ackermann_to_vesc_node',
        parameters=[vesc_config, steering_calibration_config],
        # ackermann_mux.launch.py remaps its output ackermann_cmd_out ->
        # ackermann_drive; this node's own subscription topic ("ackermann_cmd") is
        # hardcoded in vesc_ackermann's source, not configurable via param, so it
        # must be remapped here to actually receive the mux's arbitrated output.
        remappings=[('ackermann_cmd', 'ackermann_drive')],
    )

    ekf_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('f1tenth_localization'), 'launch', 'ekf.launch.py'))
    )
    navigation_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('f1tenth_navigation'), 'launch',
            'navigation.launch.py')))

    # ---- calibration:=false: today's behavior, unchanged, no added delay ----
    false_path_actions, _, _, _ = _build_driver_group(
        vesc_config, steering_calibration_config, with_crash_handler=True)
    false_path_group = GroupAction(false_path_actions, condition=is_calibration_disabled)

    # ---- calibration:=true ----
    # Steps 1+2: driver group v1 launches concurrently with BOTH calibration nodes, so
    # there's live sensor data for them to actually measure.
    driver_group_v1_actions, vesc_driver_node_v1, vesc_to_odom_node_v1, static_imu_tf_node_v1 = \
        _build_driver_group(vesc_config, steering_calibration_config, with_crash_handler=False)
    driver_group_v1 = GroupAction(
        [
            LogInfo(msg='[vesc_launch] calibration:=true -- driver group v1 up, '
                        'starting calibration measurement (covariance + gyro-bias).'),
            *driver_group_v1_actions,
        ],
        condition=is_calibration_enabled,
    )
    calibration_node = Node(
        package='f1tenth_diagnostics',
        executable='sensor_covariance_calibration_node',
        name='sensor_covariance_calibration_node',
        output='screen',
        parameters=[{
            'sample_duration_sec': LaunchConfiguration('calibration_duration_sec'),
            'vesc_yaml_path': LaunchConfiguration('vesc_yaml_path'),
        }],
        condition=is_calibration_enabled,
    )
    gyro_calibration_node = Node(
        package='f1tenth_diagnostics',
        executable='gyro_bias_calibration_node',
        name='gyro_bias_calibration_node',
        output='screen',
        parameters=[{
            'sample_duration_sec': LaunchConfiguration('gyro_sample_duration_sec'),
            'vesc_yaml_path': LaunchConfiguration('vesc_yaml_path'),
            # calibration-safety-gates pass (Phase A) -- see this file's own
            # DeclareLaunchArgument block above for why these weren't
            # forwarded here before.
            'min_samples': LaunchConfiguration('min_samples'),
            'gyro_bias_absolute_bound': LaunchConfiguration('gyro_bias_absolute_bound'),
            'gyro_bias_delta_bound': LaunchConfiguration('gyro_bias_delta_bound'),
            'gyro_bias_vibration_threshold':
                LaunchConfiguration('gyro_bias_vibration_threshold'),
        }],
        condition=is_calibration_enabled,
    )

    # Step 3: on BOTH calibration nodes' exit, log each result then shut down ONLY
    # driver group v1's 3 processes -- regardless of success or failure (see module
    # docstring: a failed/timed-out calibration falls back to the vesc.yaml already
    # on disk rather than blocking startup, since neither node writes on failure).
    _calib_exit_state = {'count': 0, 'results': {}}

    def _make_calibration_exit_handler(label):
        def _on_exit(event, context):
            _calib_exit_state['count'] += 1
            _calib_exit_state['results'][label] = event.returncode
            if _calib_exit_state['count'] < 2:
                return None

            actions = []
            for node_label, code in _calib_exit_state['results'].items():
                reason = _CALIBRATION_EXIT_REASONS.get(code, f'unknown exit code {code}')
                if code == 0:
                    actions.append(LogInfo(
                        msg=f'[vesc_launch] {node_label} calibration: {reason}.'))
                else:
                    actions.append(LogInfo(
                        msg=f'[vesc_launch] {node_label} calibration FAILED ({reason}) -- '
                            'falling back to the value already in vesc.yaml (no write '
                            'happened). Startup is NOT blocked.'))
            actions += [
                LogInfo(msg='[vesc_launch] calibration complete -- shutting down '
                            'driver group v1 for fresh-config relaunch.'),
                EmitEvent(event=ShutdownProcess(
                    process_matcher=matches_action(vesc_driver_node_v1))),
                EmitEvent(event=ShutdownProcess(
                    process_matcher=matches_action(vesc_to_odom_node_v1))),
                EmitEvent(event=ShutdownProcess(
                    process_matcher=matches_action(static_imu_tf_node_v1))),
            ]
            return actions
        return _on_exit

    calibration_exit_handler = RegisterEventHandler(OnProcessExit(
        target_action=calibration_node,
        on_exit=_make_calibration_exit_handler('covariance')))
    gyro_calibration_exit_handler = RegisterEventHandler(OnProcessExit(
        target_action=gyro_calibration_node,
        on_exit=_make_calibration_exit_handler('gyro-bias')))

    # Step 4: wait for all 3 v1 processes to exit -- a closure-captured counter shared
    # across 3 independent OnProcessExit watchers, not a single representative process
    # (their exit order isn't guaranteed -- the serial-port-holding driver node vs. the
    # lightweight TF publisher may not release in the same order every run) and not a
    # bare timer standing in for the exit detection itself. Only once the counter
    # confirms all 3 are gone does the _TEARDOWN_BUFFER_SEC timer (step 4's OS-cleanup
    # allowance) run, then step 5+6 (fresh driver group, then release downstream).
    _exit_state = {'count': 0}

    def _on_v1_process_exit(event, context):
        _exit_state['count'] += 1
        if _exit_state['count'] < 3:
            return None

        driver_group_v2_actions, vesc_driver_node_v2, _, _ = _build_driver_group(
            vesc_config, steering_calibration_config, with_crash_handler=True)

        # release_downstream is a normal per-file DeclareLaunchArgument (not one of the
        # 5 stack-wide branching args), resolved here via .perform(context) since this
        # runs inside a runtime callback, not at parse time. Default true preserves
        # stack_bringup.launch.py's exact existing behavior; component_supervisor_node's
        # calibrate_hardware registry entry passes release_downstream:=false so
        # localization/navigation stay independently-tracked components instead of
        # becoming nested children of this launch tree's own process group.
        release_downstream = (
            LaunchConfiguration('release_downstream').perform(context).lower() == 'true')

        if release_downstream:
            on_start = [
                LogInfo(msg='[vesc_launch] fresh driver group v2 up -- '
                            'releasing ekf_node/navigation (Nav2 or mpc_corr).'),
                ekf_include,
                navigation_include,
            ]
        else:
            on_start = [
                LogInfo(msg='[vesc_launch] fresh driver group v2 up -- '
                            'release_downstream:=false, NOT releasing ekf_node/Nav2 here. '
                            'Expected to be started/restarted independently (e.g. via '
                            'component_supervisor_node\'s localization/navigation '
                            'components).'),
            ]

        release_downstream_handler = RegisterEventHandler(
            OnProcessStart(
                target_action=vesc_driver_node_v2,
                on_start=on_start,
            )
        )
        return [
            LogInfo(msg='[vesc_launch] all driver group v1 processes exited -- '
                        f'relaunching in {_TEARDOWN_BUFFER_SEC:.1f}s '
                        '(OS process-teardown buffer).'),
            TimerAction(period=_TEARDOWN_BUFFER_SEC, actions=[
                *driver_group_v2_actions,
                release_downstream_handler,
            ]),
        ]

    v1_exit_watchers = [
        RegisterEventHandler(OnProcessExit(target_action=action, on_exit=_on_v1_process_exit))
        for action in (vesc_driver_node_v1, vesc_to_odom_node_v1, static_imu_tf_node_v1)
    ]

    # "The full stack" -- everything above, completely unmodified from the
    # pre-battery-check version of this file. The battery gate below only changes
    # WHEN this gets launched, never what's in it.
    full_stack_actions = [
        ackermann_to_vesc_node,
        false_path_group,
        driver_group_v1,
        calibration_node,
        gyro_calibration_node,
        calibration_exit_handler,
        gyro_calibration_exit_handler,
        *v1_exit_watchers,
    ]

    # ---- Battery voltage pre-flight gate (see module docstring) ----
    min_battery_voltage_default, min_battery_voltage_desc = get_default('min_battery_voltage')
    min_battery_voltage_la = DeclareLaunchArgument(
        'min_battery_voltage', default_value=str(min_battery_voltage_default),
        description=min_battery_voltage_desc)
    # battery-check-startup-race fix: ceiling on the separate "wait for the
    # first telemetry sample" phase -- see battery_voltage_check_node.py's own
    # module docstring for why this had to be split out from the fixed
    # sample_window_sec averaging window it used to double as.
    max_wait_for_first_sample_default, max_wait_for_first_sample_desc = get_default(
        'max_wait_for_first_sample_sec')
    max_wait_for_first_sample_la = DeclareLaunchArgument(
        'max_wait_for_first_sample_sec', default_value=str(max_wait_for_first_sample_default),
        description=max_wait_for_first_sample_desc)

    # Standalone, temporary instance: exists only to give battery_voltage_check_node
    # something to sample /sensors/core from before "the full stack" (which includes
    # its own vesc_driver_node, per calibration:=true/false above) is allowed to exist.
    vesc_driver_node_precheck = Node(
        package='vesc_driver',
        executable='vesc_driver_node',
        name='vesc_driver_node',
        parameters=[vesc_config, steering_calibration_config],
        sigterm_timeout='5',
        sigkill_timeout='2',
    )
    battery_check_node = Node(
        package='f1tenth_diagnostics',
        executable='battery_voltage_check_node',
        name='battery_voltage_check_node',
        output='screen',
        parameters=[{
            'min_battery_voltage': LaunchConfiguration('min_battery_voltage'),
            'max_wait_for_first_sample_sec':
                LaunchConfiguration('max_wait_for_first_sample_sec'),
        }],
    )

    # Set once the battery check has exited, whatever it reported -- lets
    # _on_precheck_driver_exit tell "precheck driver released intentionally after the
    # check finished" apart from "precheck driver crashed/disconnected on its own,"
    # which must NOT be treated as permission to launch the full stack. It is no
    # longer a pass/fail flag: the check's verdict does not gate anything (see module
    # docstring), only its completion sequences the driver handover.
    _battery_check_state = {'completed': False}

    def _on_battery_check_exit(event, context):
        _battery_check_state['completed'] = True
        # Advisory: battery_voltage_check_node always exits 0, and even a
        # non-zero exit (i.e. the checker itself crashed) must not cost the car
        # its drive-by-wire -- that is the failure this whole pass removes. Say
        # so out loud rather than silently proceeding, so a crashed checker is
        # still visible in this log.
        actions = []
        if event.returncode != 0:
            actions.append(LogInfo(
                msg=f'[vesc_launch] battery_voltage_check_node exited '
                    f'{event.returncode} (it is advisory and should always exit 0 '
                    '-- a non-zero exit means the checker itself failed, not that '
                    'the battery is bad). Proceeding with the drive stack; see its '
                    'log above, and note battery protection is the BT IsBatteryLow '
                    'emergency lane, not this check.'))
        actions += [
            LogInfo(msg='[vesc_launch] battery check reported -- releasing precheck '
                        'driver, launching the full stack.'),
            EmitEvent(event=ShutdownProcess(
                process_matcher=matches_action(vesc_driver_node_precheck))),
        ]
        return actions

    def _on_precheck_driver_exit(event, context):
        if not _battery_check_state['completed']:
            return [LogInfo(
                msg='[vesc_launch] precheck vesc_driver_node exited before the battery '
                    'check completed -- treating as a failure, NOT launching the drive '
                    'stack. (Hardware disconnected? Check the log above.)')]
        return [
            LogInfo(msg='[vesc_launch] precheck driver released -- launching full '
                        f'stack in {_TEARDOWN_BUFFER_SEC:.1f}s (OS process-teardown '
                        'buffer).'),
            TimerAction(period=_TEARDOWN_BUFFER_SEC, actions=full_stack_actions),
        ]

    battery_check_exit_handler = RegisterEventHandler(
        OnProcessExit(target_action=battery_check_node, on_exit=_on_battery_check_exit))
    precheck_driver_exit_handler = RegisterEventHandler(
        OnProcessExit(target_action=vesc_driver_node_precheck,
                       on_exit=_on_precheck_driver_exit))

    return LaunchDescription([
        vesc_la,
        steering_calibration_la,
        calibration_la,
        calibration_duration_la,
        gyro_sample_duration_la,
        min_samples_la,
        gyro_bias_absolute_bound_la,
        gyro_bias_delta_bound_la,
        gyro_bias_vibration_threshold_la,
        vesc_yaml_path_la,
        release_downstream_la,
        min_battery_voltage_la,
        max_wait_for_first_sample_la,
        LogInfo(msg='[vesc_launch] battery pre-flight check starting.'),
        vesc_driver_node_precheck,
        battery_check_node,
        battery_check_exit_handler,
        precheck_driver_exit_handler,
    ])
