"""Launches gyro_bias_calibration_node and sensor_covariance_calibration_node together
-- run with the car stationary and level; see f1tenth_diagnostics/README.md for how to
read and apply each node's result.

Consolidated from the two previously-separate launch files
(gyro_bias_calibration.launch.py, sensor_covariance_calibration.launch.py) into one,
since both are stationary samplers meant to be run together as a single calibration
pass rather than two separate manual steps. The two nodes are independent (different
node names, no shared state, no ordering dependency) and simply run concurrently here.

Note: merging the two files' launch arguments required renaming what was previously
`sample_duration_sec` in each (identical name, no collision only because they lived in
separate LaunchDescriptions) to two distinct names, gyro_sample_duration_sec (its own
stack_params.yaml key) and calibration_duration_sec (reconciled here, see below), so
both can be set independently from one command.

calibration_duration_sec (not a separately-named covariance_sample_duration_sec
anymore -- reconciled during the safety-margin/dead-config cleanup pass): this launch
file's own covariance sample-duration arg previously read a second,
independently-editable stack_params.yaml key (covariance_sample_duration_sec) that
happened to default to the same 60.0s as f1tenth_hardware/vesc.launch.py's
calibration_duration_sec (used for the exact same underlying
sensor_covariance_calibration_node parameter, sample_duration_sec, just reached via a
different launch path -- calibrate-then-launch vs. this standalone entry point) --
two sources of truth for one value, in agreement only by coincidence. Both launch
files now declare a launch argument literally named calibration_duration_sec, sourced
from the single stack_params.yaml key of that name.

This launch file only ever runs sensor_covariance_calibration_node in its 'stationary'
mode (the node's own declare_parameter default) -- calibration_mode and the three
light_motion_* knobs are deliberately NOT exposed as launch arguments here. That mode
also exists ('light_motion' -- a human-confirmed constant-velocity drive to get a real,
non-zero vx_variance, see that node's own module docstring for why a stationary sample
structurally can't), but its confirm_light_motion_start() blocks on a real input() and
`ros2 launch` does not reliably forward stdin to a launched node's process -- offering it
here previously caused a live run to hang forever right after "Stop command
published -- call confirm_light_motion_start() to begin driving" with no further output
and no way to respond. light_motion mode must be started with `ros2 run` directly
instead (see f1tenth_diagnostics/README.md for the exact invocation and
sensor_covariance_calibration_node.py's own module docstring / confirm_light_motion_
start() docstring for why). gyro_bias_calibration_node is entirely unaffected by any of
this -- it has no calibration_mode of its own.

vesc_yaml_path: passed explicitly here (via resolve_source_vesc_yaml_path(), imported
from the node module itself) rather than left to the node's own default. The node's own
default follows the INSTALLED vesc.yaml's symlink back to source -- only correct when
the workspace was built with --symlink-install, which this one is not, so left alone it
silently patches the install-space copy instead of src/f1tenth_bringup/config/vesc.yaml.
resolve_source_vesc_yaml_path() anchors off the node module's own __file__, which
ament_python always installs in editable mode regardless of --symlink-install (confirmed
empirically), so it reliably reaches the real source file. The node's own resolution
logic is left completely intact as a fallback for anyone invoking it directly via
`ros2 run` instead of through this launch file. Override either way via
`vesc_yaml_path:=<path>` on this launch command.

For automatic sequencing with VESC bringup (calibrate-then-launch in one run), use
f1tenth_hardware/launch/vesc.launch.py's calibration:=true argument instead -- that
path runs sensor_covariance_calibration_node only (not gyro-bias), invoking the node
directly rather than through this launch file; see that file's own module docstring.
"""

from f1tenth_diagnostics.sensor_covariance_calibration_node import (
    resolve_source_vesc_yaml_path,
)
from f1tenth_params.param_defaults import get_default

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    imu_topic_default, imu_topic_desc = get_default('imu_topic')
    imu_topic_arg = DeclareLaunchArgument(
        'imu_topic', default_value=str(imu_topic_default), description=imu_topic_desc)
    odom_topic_default, odom_topic_desc = get_default('odom_topic')
    odom_topic_arg = DeclareLaunchArgument(
        'odom_topic', default_value=str(odom_topic_default), description=odom_topic_desc)
    gyro_sample_duration_default, gyro_sample_duration_desc = get_default(
        'gyro_sample_duration_sec')
    gyro_sample_duration_arg = DeclareLaunchArgument(
        'gyro_sample_duration_sec', default_value=str(gyro_sample_duration_default),
        description=gyro_sample_duration_desc)
    min_samples_default, min_samples_desc = get_default('min_samples')
    min_samples_arg = DeclareLaunchArgument(
        'min_samples', default_value=str(min_samples_default), description=min_samples_desc)
    calibration_duration_default, calibration_duration_desc = get_default(
        'calibration_duration_sec')
    calibration_duration_arg = DeclareLaunchArgument(
        'calibration_duration_sec',
        default_value=str(calibration_duration_default),
        description=calibration_duration_desc)
    # Explicit, reliably-source-anchored override -- see module docstring's
    # "vesc_yaml_path" paragraph for why this is needed instead of the node's own
    # (symlink-following, install-space-in-this-workspace) default.
    vesc_yaml_path_arg = DeclareLaunchArgument(
        'vesc_yaml_path', default_value=resolve_source_vesc_yaml_path(),
        description='Real source-tree vesc.yaml that sensor_covariance_calibration_node '
                     'backs up and patches in place with its measured variances.')
    # calibration_mode / light_motion_* are deliberately NOT declared as launch
    # arguments here -- see module docstring. sensor_covariance_calibration_node
    # stays on its own 'stationary' default in this launch file; light_motion mode
    # is `ros2 run`-only.

    gyro_bias_calibration_node = Node(
        package='f1tenth_diagnostics',
        executable='gyro_bias_calibration_node',
        name='gyro_bias_calibration_node',
        output='screen',
        parameters=[{
            'imu_topic': LaunchConfiguration('imu_topic'),
            'sample_duration_sec': LaunchConfiguration('gyro_sample_duration_sec'),
            'min_samples': LaunchConfiguration('min_samples'),
        }],
    )
    sensor_covariance_calibration_node = Node(
        package='f1tenth_diagnostics',
        executable='sensor_covariance_calibration_node',
        name='sensor_covariance_calibration_node',
        # 'stationary' only (the node's own default) -- calibration_mode is not
        # wired to a launch argument here on purpose, see module docstring.
        # emulate_tty=True was tried here previously as a fix for light_motion
        # mode's input() hang; it does NOT forward stdin (it only affects
        # whether the subprocess's stdout is treated as a TTY for log
        # formatting), so it did not fix anything and has been removed.
        output='screen',
        parameters=[{
            'imu_topic': LaunchConfiguration('imu_topic'),
            'odom_topic': LaunchConfiguration('odom_topic'),
            'sample_duration_sec': LaunchConfiguration('calibration_duration_sec'),
            'vesc_yaml_path': LaunchConfiguration('vesc_yaml_path'),
        }],
    )

    return LaunchDescription([
        imu_topic_arg,
        odom_topic_arg,
        gyro_sample_duration_arg,
        min_samples_arg,
        calibration_duration_arg,
        vesc_yaml_path_arg,
        gyro_bias_calibration_node,
        sensor_covariance_calibration_node,
    ])
