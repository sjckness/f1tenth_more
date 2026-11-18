import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess
def generate_launch_description():
    d = os.path.dirname(os.path.abspath(__file__))
    return LaunchDescription([ExecuteProcess(
        cmd=['python3', os.path.join(d, 'probe_node.py'), os.environ['PROBE_MARKER']],
        output='screen')])
