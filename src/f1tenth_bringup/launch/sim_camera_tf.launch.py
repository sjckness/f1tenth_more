"""sim:=isaac stand-in for the camera part of the 'perception' component (camera.launch.py).

On the car, camera.launch.py starts the ZED wrapper (images, depth and the camera's own
robot_state_publisher) and the static base_link -> zed2_camera_link transform. In sim the
images and depth come from Isaac Sim (/camera/image_raw, /camera/camera_info and
/zed2/zed_node/depth/* under the car's names), so only the frames are needed:

- static_baselink_to_zed2 (same node name as camera.launch.py): base_link -> zed2_camera_link
  with the DEPLOYED mount (bag 2026-10-15T15-18-27 /tf_static, commit e3793e47 on the Orin):
  (0.2399, 0, 0.205) and pitch -2.083 deg (q = 0, -0.01817399939, 0, 0.99983483923). The
  jazzy-campaign branch is older than the car and its camera.launch.py still says z 0.15, no
  pitch; the sim must use what the car runs (output/jazzy_port/phase0.md).
- zed2/zed2_state_publisher, a robot_state_publisher like the wrapper's own (namespace zed2,
  robot_description remapped to zed2_description so it never competes with the car's
  /robot_description): the ZED 2i camera tree with the frame names the car's humble-v4.2.5
  wrapper publishes and Isaac stamps (zed2_left_camera_optical_frame, not v5's
  zed2_left_camera_frame_optical). Values from the same bag.

Every edge is static (fixed joints / static_transform_publisher): front_clearance,
lidar_front_wall, swept_clearance, obstacle_clearance, wall_distance, yolo and the BT read
/tf_static only. Started by component_supervisor_node only in sim:=isaac; never on the car.
The mount is overridable (zed_x, zed_y, zed_z, zed_qx, zed_qy, zed_qz, zed_qw).
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# Deployed car /tf_static, bag 2026-10-15T15-18-27_mission-llm_86cff518e42d.
_DEPLOYED_MOUNT = {
    'zed_x': '0.2399', 'zed_y': '0.0', 'zed_z': '0.205',
    'zed_qx': '0.0', 'zed_qy': '-0.01817399939', 'zed_qz': '0.0', 'zed_qw': '0.99983483923',
}

# ZED 2i camera tree as published by the car's wrapper (humble-v4.2.5, zed2i,
# camera_name zed2): link -> center (0, 0, 0.015); center -> left/right camera frame
# (-0.01, +-0.06, 0); camera frame -> optical frame rpy (-pi/2, 0, -pi/2).
_ZED_URDF = """<?xml version="1.0"?>
<robot name="zed2_sim">
  <link name="zed2_camera_link"/>
  <link name="zed2_camera_center"/>
  <link name="zed2_left_camera_frame"/>
  <link name="zed2_left_camera_optical_frame"/>
  <link name="zed2_right_camera_frame"/>
  <link name="zed2_right_camera_optical_frame"/>
  <joint name="zed2_camera_center_joint" type="fixed">
    <parent link="zed2_camera_link"/><child link="zed2_camera_center"/>
    <origin xyz="0 0 0.015" rpy="0 0 0"/>
  </joint>
  <joint name="zed2_left_camera_joint" type="fixed">
    <parent link="zed2_camera_center"/><child link="zed2_left_camera_frame"/>
    <origin xyz="-0.01 0.06 0" rpy="0 0 0"/>
  </joint>
  <joint name="zed2_left_camera_optical_joint" type="fixed">
    <parent link="zed2_left_camera_frame"/><child link="zed2_left_camera_optical_frame"/>
    <origin xyz="0 0 0" rpy="-1.5707963267948966 0 -1.5707963267948966"/>
  </joint>
  <joint name="zed2_right_camera_joint" type="fixed">
    <parent link="zed2_camera_center"/><child link="zed2_right_camera_frame"/>
    <origin xyz="-0.01 -0.06 0" rpy="0 0 0"/>
  </joint>
  <joint name="zed2_right_camera_optical_joint" type="fixed">
    <parent link="zed2_right_camera_frame"/><child link="zed2_right_camera_optical_frame"/>
    <origin xyz="0 0 0" rpy="-1.5707963267948966 0 -1.5707963267948966"/>
  </joint>
</robot>
"""


def generate_launch_description():
    args = [DeclareLaunchArgument(k, default_value=v,
                                  description='deployed ZED mount (bag 2026-10-15)')
            for k, v in _DEPLOYED_MOUNT.items()]
    lc = {k: LaunchConfiguration(k) for k in _DEPLOYED_MOUNT}
    mount = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_baselink_to_zed2',
        arguments=['--x', lc['zed_x'], '--y', lc['zed_y'], '--z', lc['zed_z'],
                   '--qx', lc['zed_qx'], '--qy', lc['zed_qy'], '--qz', lc['zed_qz'],
                   '--qw', lc['zed_qw'],
                   '--frame-id', 'base_link', '--child-frame-id', 'zed2_camera_link'],
    )
    zed_tree = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace='zed2',
        name='zed2_state_publisher',
        output='screen',
        parameters=[{'robot_description': _ZED_URDF}],
        remappings=[('robot_description', 'zed2_description')],
    )
    return LaunchDescription(args + [mount, zed_tree])
