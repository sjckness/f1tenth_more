import os
from ament_index_python.packages import get_package_share_directory #used for colcon build to find the package in installation directory
from launch import LaunchDescription #the core ros2 container
from launch.actions import ExecuteProcess #allows us to spawn system processes not just ros2 nodes. used for terminal commands directly from py script

#this runs when we launch the ros2 launch command. it finds the def and uses it to find needed folder/files
def generate_launch_description():
    world_path = os.path.join(
        get_package_share_directory('f1tenth_minimal'),
        'worlds',
        'empty_room.sdf'
    )
#runs the command through terminal
    return LaunchDescription([
        ExecuteProcess(
            cmd = ['ign', 'gazebo', '-r', world_path],
            output = 'screen'
        )
    ])