"""Offline replay of a raw clip (tools/record_raw.sh) through the perception stack.

    ros2 launch tools/replay_raw.launch.py clip:=$HOME/raw_data/<date>_<name> \
        [enable_scene_labels:=true] [use_wall_tracker:=true] [domain_id:=88]

The clip folder holds <name>.svo2, bag/ and notes.txt. Nothing touches hardware:

  - the ZED wrapper plays the SVO (svo.use_svo_timestamps true, svo.svo_realtime
    true), with the same remaps and zed2_perception.yaml as camera.launch.py,
    and without its URDF publisher: every TF comes from the bag's /tf_static;
  - ros2 bag play --clock replays the bag's /scan, /tf, /tf_static,
    /odometry/filtered and /sensors/imu/raw. The bag's own camera_info and ZED
    IMU are left out: the wrapper republishes them from the SVO;
  - yolo_detector, detection_3d, obstacle_projector and front_clearance
    (detection.launch.py) and lidar_front_wall (lidar_front_wall.launch.py)
    run as in the stack, all with use_sim_time true. No control, BT, MPC or
    logger.

enable_scene_labels and use_wall_tracker are stack-wide get_value switches with
no launch argument. This file overrides them for its own process only, by
patching f1tenth_params' cached stack_params.yaml before the two includes read
it; stack_params.yaml on disk is not touched.

SYNC. record_raw.sh starts the SVO ~1.7 s before the bag. The SVO starts at
svo.play_from_frame = LEAD_S before the bag's first stamp, the player starts
paused, and tools/replay_clock_sync.py resumes it when the ZED's settled
playback timeline reaches the bag's first stamp (see there for why neither
the bag's own /clock nor the first ZED frame is the trigger). Images then
reach their subscribers at sim time = stamp + the wrapper's publish latency.

Outputs go to <clip>/replay_<timestamp>/: bag/ (perception outputs, the
debug overlay, depth for offline analysis, /clock and /scan, recorded with
--use-sim-time) and replay_<timestamp>.debug.mp4 rendered by
f1tenth_logger.mission_debug_video once the bag has played out. The launch
then shuts itself down.

Isolation: own ROS_DOMAIN_ID, localhost only, no discovery server, so a live
stack on this machine never sees the replay.
"""

import glob
import os
import signal
import time

import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, ExecuteProcess, GroupAction,
                            IncludeLaunchDescription, OpaqueFunction, RegisterEventHandler,
                            SetEnvironmentVariable, TimerAction, UnsetEnvironmentVariable)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown, matches_action
from launch.events.process import SignalProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import SetParameter, SetRemap

# Before anything in this process (or its children) creates a ROS node.
os.environ.pop('ROS_DISCOVERY_SERVER', None)
os.environ.pop('ROS_SUPER_CLIENT', None)
os.environ['ROS_LOCALHOST_ONLY'] = '1'

PLAY_TOPICS = ['/scan', '/tf', '/tf_static', '/odometry/filtered', '/sensors/imu/raw']
LEAD_S = 1.5   # SVO playback starts this long before the bag (settling, see the sync node)
RECORD_TOPICS = [
    '/perception/walls', '/perception/virtual_wall', '/perception/virtual_wall/debug',
    '/perception/wall_planes', '/perception/stereo_wall',
    '/camera/detections', '/camera/detections_3d', '/perception/front_distance',
    '/camera/debug_image/compressed',
    '/zed2/zed_node/depth/depth_registered', '/zed2/zed_node/depth/camera_info',
    '/camera/camera_info', '/zed2/zed_node/imu/data', '/scan', '/clock', '/tf_static',
]


def _svo_skip_frames(svo, bag_dir):
    """SVO frames to skip so playback starts LEAD_S before the bag's first stamp."""
    import pyzed.sl as sl
    with open(os.path.join(bag_dir, 'metadata.yaml'), encoding='utf-8') as f:
        info = yaml.safe_load(f)['rosbag2_bagfile_information']
    bag0 = info['starting_time']['nanoseconds_since_epoch']
    cam = sl.Camera()
    init = sl.InitParameters()
    init.set_from_svo_file(svo)
    init.depth_mode = sl.DEPTH_MODE.NONE
    if cam.open(init) != sl.ERROR_CODE.SUCCESS or cam.grab() != sl.ERROR_CODE.SUCCESS:
        raise RuntimeError(f'cannot read {svo}')
    svo0 = cam.get_timestamp(sl.TIME_REFERENCE.IMAGE).get_nanoseconds()
    fps = cam.get_camera_information().camera_configuration.fps
    cam.close()
    return max(0, round(((bag0 - svo0) * 1e-9 - LEAD_S) * fps)), (bag0 - svo0) * 1e-9, bag0


def _setup(context):
    clip = os.path.abspath(os.path.expanduser(LaunchConfiguration('clip').perform(context)))
    svos = glob.glob(os.path.join(clip, '*.svo2'))
    bag_dir = os.path.join(clip, 'bag')
    if len(svos) != 1 or not os.path.isdir(bag_dir):
        raise RuntimeError(f'{clip}: need exactly one *.svo2 and a bag/ folder')
    domain_id = LaunchConfiguration('domain_id').perform(context)
    os.environ['ROS_DOMAIN_ID'] = domain_id
    out = os.path.join(clip, 'replay_' + time.strftime('%Y%m%dT%H%M%S'))
    os.makedirs(out)

    skip = LaunchConfiguration('svo_skip_frames').perform(context)
    if skip == 'auto':
        skip, offset, _ = _svo_skip_frames(svos[0], bag_dir)
        print(f'[replay_raw] bag starts {offset:.3f} s after the SVO: play_from_frame {skip}')
    skip = int(skip)

    # zed2_perception.yaml plus the playback keys, as the wrapper's override file.
    perception_share = get_package_share_directory('f1tenth_perception')
    with open(os.path.join(perception_share, 'config', 'zed2_perception.yaml'), encoding='utf-8') as f:
        zed_params = yaml.safe_load(f)
    zed_params['/**']['ros__parameters']['svo'] = {
        'use_svo_timestamps': True, 'svo_realtime': True, 'svo_loop': False,
        'play_from_frame': skip}
    zed_yaml = os.path.join(out, 'zed_replay_params.yaml')
    with open(zed_yaml, 'w', encoding='utf-8') as f:
        yaml.safe_dump(zed_params, f)

    # Stack-wide switches for this launch process only (see the docstring).
    from f1tenth_params import param_defaults
    table = param_defaults._load()
    for name in ('enable_scene_labels', 'use_wall_tracker'):
        value = LaunchConfiguration(name).perform(context).lower() in ('1', 'true', 'yes', 'on')
        table[name] = dict(table[name], default=value)

    def include(package, launch_file, args=None):
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory(package), 'launch', launch_file)),
            launch_arguments=(args or {}).items())

    zed = GroupAction([
        SetRemap(src='/zed2/zed_node/rgb/image_rect_color', dst='/camera/image_raw'),
        SetRemap(src='/zed2/zed_node/rgb/camera_info', dst='/camera/camera_info'),
        include('zed_wrapper', 'zed_camera.launch.py', {
            'camera_model': 'zed2i', 'camera_name': 'zed2',
            'ros_params_override_path': zed_yaml, 'svo_path': svos[0],
            'use_sim_time': 'true', 'publish_urdf': 'false', 'publish_tf': 'false',
            'publish_map_tf': 'false', 'publish_imu_tf': 'false'}),
    ])

    record = ExecuteProcess(
        cmd=['ros2', 'bag', 'record', '--use-sim-time', '-o', os.path.join(out, 'bag'),
             *RECORD_TOPICS],
        name='replay_record', output='screen')
    with open(os.path.join(bag_dir, 'metadata.yaml'), encoding='utf-8') as f:
        bag0 = yaml.safe_load(f)['rosbag2_bagfile_information']['starting_time'][
            'nanoseconds_since_epoch']
    sync = ExecuteProcess(
        cmd=['python3', os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                     'replay_clock_sync.py'), str(bag0)],
        name='replay_clock_sync', output='screen')
    play = ExecuteProcess(
        cmd=['ros2', 'bag', 'play', bag_dir, '--start-paused', '--clock', '100',
             '--disable-keyboard-controls',
             '--remap', '/clock:=/clock_bag', '--topics', *PLAY_TOPICS],
        name='replay_play', output='screen')
    video = ExecuteProcess(
        cmd=['python3', '-m', 'f1tenth_logger.mission_debug_video', out, '--out-dir', out],
        name='replay_video', output='screen')

    return [
        # The launch context may hold its own copy of the environment.
        SetEnvironmentVariable('ROS_DOMAIN_ID', domain_id),
        SetEnvironmentVariable('ROS_LOCALHOST_ONLY', '1'),
        UnsetEnvironmentVariable('ROS_DISCOVERY_SERVER'),
        UnsetEnvironmentVariable('ROS_SUPER_CLIENT'),
        SetParameter(name='use_sim_time', value=True),
        record,
        sync,
        play,
        zed,
        include('f1tenth_perception', 'detection.launch.py'),
        include('f1tenth_perception', 'lidar_front_wall.launch.py'),
        # Bag done -> let the pipeline drain, stop the recorder, render, quit.
        RegisterEventHandler(OnProcessExit(target_action=play, on_exit=[TimerAction(
            period=3.0, actions=[EmitEvent(event=SignalProcess(
                signal_number=signal.SIGINT, process_matcher=matches_action(record)))])])),
        RegisterEventHandler(OnProcessExit(target_action=record, on_exit=[video])),
        RegisterEventHandler(OnProcessExit(target_action=video, on_exit=[
            EmitEvent(event=Shutdown(reason='replay finished'))])),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('clip', description='raw clip folder (<name>.svo2 + bag/)'),
        DeclareLaunchArgument('enable_scene_labels', default_value='true'),
        DeclareLaunchArgument('use_wall_tracker', default_value='true'),
        DeclareLaunchArgument('svo_skip_frames', default_value='auto',
                              description="SVO frames to skip, 'auto': align to the bag start"),
        DeclareLaunchArgument('domain_id', default_value='88'),
        OpaqueFunction(function=_setup),
    ])
