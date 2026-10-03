"""The slam.launch.py lifecycle pattern on a minimal node: configure on start,
activate on reaching 'inactive', both through launch_ros's change_state client."""
import os
import sys
import launch
from launch.actions import EmitEvent, RegisterEventHandler
from launch_ros.actions import LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition


def generate_launch_description():
    here = os.path.dirname(os.path.abspath(__file__))
    node = LifecycleNode(
        name='syn_lifecycle', namespace='', executable=sys.executable,
        arguments=[os.path.join(here, 'lc_node.py')], output='screen')
    return launch.LaunchDescription([
        node,
        EmitEvent(event=ChangeState(
            lifecycle_node_matcher=lambda a: a is node,
            transition_id=Transition.TRANSITION_CONFIGURE)),
        RegisterEventHandler(OnStateTransition(
            target_lifecycle_node=node, goal_state='inactive',
            entities=[EmitEvent(event=ChangeState(
                lifecycle_node_matcher=lambda a: a is node,
                transition_id=Transition.TRANSITION_ACTIVATE))])),
    ])
