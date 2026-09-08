"""Standalone launch for steering_offset_calibration_node -- the open-loop
steering-offset / effective-wheelbase calibration drive.

RUN THIS BY HAND, FROM A SECOND TERMINAL, WITH THE STACK ALREADY UP AND A HAND
ON THE JOYSTICK. Deliberately NOT included from supervisor_bringup.launch.py or
stack_bringup.launch.py: every other node in f1tenth_diagnostics is passive,
this one drives the car. It is also not part of vesc.launch.py's
calibration:=true path -- that path runs stationary samplers before
localization even exists, and this node needs SLAM up and localised.

    ros2 launch f1tenth_diagnostics steering_offset_calibration.launch.py

Add write_enabled:=false for a dry run: it drives, fits, gates and reports
exactly as normal but changes no config file. Worth doing first on a car whose
space you have not measured.

THIS LAUNCH FILE ONLY EXPOSES MODE B (the drive). Mode A -- the static sweep,
which is the GROUND TRUTH for gain/offset/backlash -- is deliberately not
offered here: it blocks on a real input() to collect each measured wheel
angle, and `ros2 launch` does not reliably forward stdin to a launched node.
Offering it here would reproduce exactly the hang that made
sensor_covariance_calibration_node's light_motion mode unusable through launch
(see calibration.launch.py's own note). Run mode A directly instead:

    ros2 run f1tenth_diagnostics steering_offset_calibration_node \
        --ros-args -p calibration_mode:=static

and do that FIRST -- mode B is validation of mode A plus whatever slip a
static sweep cannot see, not a replacement for it.

DO NOT confuse this with vesc.launch.py's calibration:=true argument. That is a
different, unrelated path (stationary gyro/covariance sampling) with a known TF
race, and nothing here needs it enabled.

The write target is passed explicitly, resolved through
calibration_common.resolve_source_config_path() rather than left to a share-
directory default: this workspace is NOT built with --symlink-install, so an
installed config path is a plain copy and patching it would silently do
nothing to the source tree. Same reasoning as calibration.launch.py's own
vesc_yaml_path argument -- see that file's docstring.

Only steering_calibration.yaml is written now. The wheelbase is PINNED at the
F1TENTH spec value and never fitted (see the node's own module docstring on
why fitting L_eff was wrong), so vesc.yaml is no longer a write target -- the
node reports the discrepancy against vesc.yaml's wheelbase and changes
nothing.
"""

import math

from f1tenth_diagnostics.calibration_common import resolve_source_config_path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# (launch argument, default, description) -- kept as a table rather than
# thirty near-identical DeclareLaunchArgument blocks. These are node-local
# tuning knobs for a manually-run tool, so they live here rather than in
# stack_params.yaml, which is the single source of truth for values the
# PRODUCTION stack shares between launch files. Nothing in the running stack
# reads any of these.
ARGUMENTS = (
    ('pose_topic', '/slam/pose',
     'SLAM pose used for psi and s. Do NOT point this at /odometry/filtered or '
     '/ekf_global/odometry/filtered -- they fuse the gyro and wheel odometry being '
     'calibrated, which makes the fit circular and clean-looking. See the node docstring.'),
    ('map_topic', '/slam/map',
     'Latched SLAM map, used as the standstill READINESS proof that slam_toolbox is up '
     'and holds a map. Preflight cannot wait for a /slam/pose instead: slam_toolbox is '
     'distance-gated (minimum_travel_distance 0.03) so a parked car never produces one. '
     'Do NOT substitute the map -> odom transform -- ekf_global publishes that at 50 Hz '
     'with or without any SLAM input, so it proves nothing.'),
    ('drive_topic', '/calibration_drive',
     'Drive command topic. Must match the calibration lane in mux.yaml (priority 50).'),
    ('nudge_distance_m', '0.18',
     'PREFLIGHT NUDGE: how far the car creeps forward, steering centred, to get past '
     'slam_toolbox\'s 0.03 m travel gate and prove the whole chain (mux lane -> VESC -> '
     'wheels -> odometry -> scan match -> pose) works BEFORE committing to ~3.6 m of '
     'open-loop drive. This is the only part of preflight that moves the car.'),
    ('nudge_speed_mps', '0.1', 'Speed of the preflight nudge (m/s).'),
    ('nudge_pose_timeout_sec', '4.0',
     'How long to keep waiting for a /slam/pose after the nudge has finished and the car '
     'is stopped again. The scan match lags the motion and /slam/pose runs at ~2 Hz, so '
     'this is not zero. No pose within it = refuse to drive.'),
    ('speed_mps', '0.25', 'Drive speed for the calibration run (m/s).'),
    ('amplitude_rad', str(round(math.radians(10.0), 6)),
     'Steering amplitude a (rad). Checked against min/max_steering_angle before moving.'),
    ('segment_plan', 'short',
     "Drive profile per repetition. 'short' = 3 segments (straight, +a, -a), 2.4 m; "
     "'full' = 5 segments (straight, +a, -a, +a, straight), 3.6 m. Both give the fit "
     'what it needs -- a near-zero segment to pin the offset and both steering signs '
     'once the per-repetition alternation is applied. Prefer short: a fourth repetition '
     'buys more than two extra segments, and a profile that runs out of room mid-run '
     'is worth nothing to the fit at all.'),
    ('repetitions', '3',
     'Number of S-curve repetitions. The starting sign alternates each repetition, '
     'which is what lets the fit separate gain from offset.'),
    ('pinned_wheelbase_m', '0.3302',
     'Wheelbase used as a FIXED divisor in the fit -- never fitted. F1TENTH spec '
     '(lf 0.15875 + lr 0.17145 = 13 in), ASSUMED NOT MEASURED on this car; published '
     'F1TENTH figures disagree (another set gives 0.265 m). An error here becomes a '
     'proportional error in the fitted gain.'),
    ('turning_point_sigma_t', '0.26',
     'Timing uncertainty (s) propagated into each turning-point offset estimate. '
     'Half a /slam/pose interval at the ~1.9 Hz this stack publishes.'),
    ('write_enabled', 'true',
     'false = drive, fit, gate and report, but change no config file (dry run).'),
    ('require_estop_publisher', 'true',
     'Refuse to run when nothing publishes /safety_stop. Set false only if you accept '
     'driving with the joystick and this node\'s own clearance check as the only stops.'),
    ('min_front_clearance_m', '0.9', 'Abort if /costmap/front_clearance drops below this.'),
    ('max_lateral_excursion_m', '0.8',
     'Abort if the car strays further than this from the repetition start line.'),
    ('hard_timeout_sec', '300.0', 'Abort unconditionally after this long.'),
    ('inter_rep_pause_sec', '6.0',
     'Stopped pause between repetitions, for repositioning the car. Three repetitions '
     'driven back-to-back need ~10.8 m of clear space; repositioning is the alternative.'),
    ('min_pose_samples_per_segment', '4',
     'Refuse to write if any segment was fit from fewer SLAM fixes than this. /slam/pose '
     'runs at ~2 Hz here, so a 0.6 m segment yields only about 4-5 after settling.'),
)


def generate_launch_description():
    declared = [
        DeclareLaunchArgument(name, default_value=default, description=description)
        for name, default, description in ARGUMENTS
    ]
    declared.append(DeclareLaunchArgument(
        'steering_calibration_yaml_path',
        default_value=resolve_source_config_path(
            'f1tenth_hardware', 'f1tenth_hardware', 'config', 'steering_calibration.yaml'),
        description='Source-tree steering_calibration.yaml -- the single source of truth '
                    'for steering_angle_to_servo_offset and the two gains. NOT '
                    'vesc.yaml: vesc.launch.py loads this file after vesc.yaml, so a '
                    'value written into vesc.yaml for these keys is silently '
                    'overridden.'))
    parameters = {name: LaunchConfiguration(name) for name, _, _ in ARGUMENTS}
    parameters['steering_calibration_yaml_path'] = LaunchConfiguration(
        'steering_calibration_yaml_path')

    calibration_node = Node(
        package='f1tenth_diagnostics',
        executable='steering_offset_calibration_node',
        name='steering_offset_calibration_node',
        output='screen',
        # emulate_tty so the diff and the relaunch/commit warnings come out
        # readable rather than as one run-on block. This node never calls
        # input(), so it does not hit the stdin-forwarding problem that makes
        # sensor_covariance_calibration_node's light_motion mode ros2-run-only.
        emulate_tty=True,
        parameters=[parameters],
    )
    return LaunchDescription(declared + [calibration_node])
