"""F1TENTH robot description + real-hardware sensor TFs -- single entry point.

Merged from two previously-separate files (description_launch.py: robot_state_publisher
only; sensor_tf_launch.py: static base_link->laser only) so there's one place that
answers "where do all the frames come from."

robot_state_publisher (roboracer.urdf.xacro) is used by:
  * f1tenth_sim (sim_bringup.launch.py includes this with use_sim:=true,
    enable_sensors:=true) -- the URDF's own sensors.xacro is the sole source of
    laser/imu/zed2_camera_link frames in sim.
  * f1tenth_bringup / f1tenth_localization (real hardware, use_sim:=false) -- to
    publish the vehicle body TF tree (base_footprint/chassis/wheels via core.xacro).
    enable_sensors defaults to false here (see f1tenth_params/config/stack_params.yaml)
    specifically so the URDF's laser/imu/zed2_camera_link links are NOT also emitted
    on real hardware -- those real frames come from static_transform_publisher nodes
    instead (this file's own base_link->laser below, plus
    f1tenth_perception/camera.launch.py's base_link->zed2_camera_link and
    f1tenth_hardware/vesc.launch.py's base_link->imu), which would otherwise fight the
    URDF for authority over the exact same parent->child transforms.

Static base_link->laser TF: gated to real hardware only (UnlessCondition(use_sim)) --
in sim, enable_sensors:=true already gives sensors.xacro's own laser frame; running
this static publisher there too would be a second, conflicting authority for the same
transform.

Frame layout (measured from base_link, which is centered between the axles and sits
0.07m above the ground -- see f1tenth_hardware/vesc.launch.py's static_baselink_to_imu
for the identity base_link->imu TF, unaffected by this file):
  zed2_camera_link (f1tenth_perception/camera.launch.py): 0.12m ahead, 0.15m higher
    (confirmed LIVE via `ros2 run tf2_ros tf2_echo base_link zed2_camera_link`
    at the time of the remount below, not just read from that file's own
    source -- matches its own documented values exactly).
  laser (below): PHYSICAL REMOUNT (rear-facing -> front-facing), done alongside
    the first slam_toolbox integration pass. NEW values are approximate,
    ruler-measured placeholders, NOT a real calibration -- same "flag
    explicitly, don't pretend it's precise" discipline the lidar-camera
    extrinsic gap already follows elsewhere in this codebase (see
    lidar_boundary_node.py's own module docstring and the SLAM feasibility
    reports' own repeated flagging of that gap). x=0.12/y=0.0 chosen to
    match zed2_camera_link's own confirmed-live forward offset (mounted
    close to the camera per the remount's own physical description);
    z=0.20 is the one genuinely new placeholder number here (was 0.15,
    matching the camera -- now higher, per the physical remount).
    yaw=0.0 (front-facing, was pi/rear-facing) -- roll=pitch=0.0 (level
    mount) is an ASSUMPTION, not a confirmed measurement (no way to verify
    this from code/TF alone before the physical remount actually happens)
    -- needs live verification once the physical remount is complete, same
    as the position numbers above.
"""
from ament_index_python.packages import get_package_share_directory

from f1tenth_params.param_defaults import get_default

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import UnlessCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = get_package_share_directory('f1tenth_description')

    use_sim_time = LaunchConfiguration('use_sim_time')
    use_sim = LaunchConfiguration('use_sim')
    enable_sensors = LaunchConfiguration('enable_sensors')
    control_config = LaunchConfiguration('control_config')

    # use_sim_time is deliberately NOT sourced from stack_params.yaml -- sim and real
    # hardware need different defaults (see stack_params.yaml's header comment), so it
    # stays a local, per-file literal.
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use the simulation clock if true.')
    use_sim_default, use_sim_desc = get_default('use_sim')
    declare_use_sim = DeclareLaunchArgument(
        'use_sim', default_value=str(use_sim_default), description=use_sim_desc)
    enable_sensors_default, enable_sensors_desc = get_default('enable_sensors')
    declare_enable_sensors = DeclareLaunchArgument(
        'enable_sensors', default_value=str(enable_sensors_default),
        description=enable_sensors_desc)
    control_config_default, control_config_desc = get_default('control_config')
    declare_control_config = DeclareLaunchArgument(
        'control_config', default_value=str(control_config_default),
        description=control_config_desc)

    xacro_file = PathJoinSubstitution([
        FindPackageShare('f1tenth_description'), 'urdf', 'roboracer.urdf.xacro'])

    robot_description_content = Command([
        'xacro ', xacro_file,
        ' use_sim:=', use_sim,
        ' enable_sensors:=', enable_sensors,
        ' pkg_share:=', pkg_share,
        ' control_config:=', control_config,
    ])

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[
            {'robot_description': ParameterValue(robot_description_content,
                                                 value_type=str)},
            {'use_sim_time': use_sim_time},
        ],
    )

    # Real-hardware-only: see module docstring for the front-facing-remount
    # rationale, the placeholder/unconfirmed status of these numbers, and why
    # this is skipped in sim. Positional args are x y z YAW PITCH ROLL
    # (verified empirically against the --roll/--pitch/--yaw named form --
    # NOT roll pitch yaw).
    static_baselink_to_laser = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_baselink_to_laser',
        arguments=['0.12', '0.0', '0.20', '0.0', '0.0', '0.0',
                   'base_link', 'laser'],
        condition=UnlessCondition(use_sim),
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_use_sim,
        declare_enable_sensors,
        declare_control_config,
        robot_state_publisher,
        static_baselink_to_laser,
    ])
