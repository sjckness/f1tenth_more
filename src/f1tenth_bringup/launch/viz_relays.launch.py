"""Rate-limited /viz/* copies of the Foxglove layout's topics.

Started by the supervisor's dev_tools component beside foxglove_bridge.launch.py
(see components.yaml), gated on enable_viz_relays. The relay list is
config/viz_relays.yaml; the rates are stack_params.yaml's viz_rate_* keys, each
also a launch argument here.

One component container holds a topic_tools::ThrottleNode per relay. They are
lazy (subscribe to the source only while the /viz topic has a subscriber) and
copy the source's reliability and durability onto the /viz publisher, both
checked live on 2026-09-22. Raw images go to /viz_internal/<source> instead,
and viz_jpeg_node turns those throttled frames into /viz/<source>/compressed.

ROS_SUPER_CLIENT: a throttle learns its source's type and QoS from the graph
before it can subscribe. A plain Discovery Server client only hears about
topics it already has endpoints on, so without this the throttles would never
find their sources. Scoped to this launch tree, as foxglove_bridge.launch.py
does for the bridge.

cpu_affinity: every core is already assigned (EKFs 0,1, slam 2, bridge 3,
behavior 4, lidar perception 5, detection 6,7, YOLO 8,9, mpc_corr 10,11), so the
relays share core 3 with foxglove_bridge: visualization only, like the bridge,
and off MPC and perception. scripts/check_cpu_pinning.py checks both processes.
"""
import os

import yaml
from ament_index_python.packages import get_package_share_directory
from f1tenth_params.param_defaults import get_default

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode

IMAGE_TYPE = 'sensor_msgs/msg/Image'
RATE_KEYS = ('viz_rate_scan', 'viz_rate_costmap', 'viz_rate_image_annotated',
             'viz_rate_default')


def load_relays(path=None):
    """The relay list from config/viz_relays.yaml."""
    if path is None:
        path = os.path.join(get_package_share_directory('f1tenth_bringup'),
                            'config', 'viz_relays.yaml')
    with open(path) as f:
        return yaml.safe_load(f)['relays']


def throttle_output(relay):
    """Where the throttle publishes: the dest, or /viz_internal/<source> for a raw image."""
    if relay['type'] == IMAGE_TYPE:
        return '/viz_internal' + relay['source']
    return relay['dest']


def _node_name(topic):
    return 'viz_throttle' + topic.replace('/', '_')


def _relays(context):
    if LaunchConfiguration('enable_viz_relays').perform(context).lower() != 'true':
        return []
    rates = {k: float(LaunchConfiguration(k).perform(context)) for k in RATE_KEYS}
    affinity = LaunchConfiguration('viz_relays_cpu_affinity').perform(context)
    relays = load_relays()

    throttles = [
        ComposableNode(
            package='topic_tools', plugin='topic_tools::ThrottleNode',
            name=_node_name(relay['source']),
            parameters=[{
                'input_topic': relay['source'],
                'output_topic': throttle_output(relay),
                'throttle_type': 'messages',
                'msgs_per_sec': rates[relay['rate']],
                'lazy': True,
            }])
        for relay in relays]
    container = ComposableNodeContainer(
        name='viz_relay_container', namespace='',
        package='rclcpp_components', executable='component_container',
        composable_node_descriptions=throttles,
        prefix=f'taskset -c {affinity}', output='screen')

    images = [r for r in relays if r['type'] == IMAGE_TYPE]
    actions = [container]
    if images:
        actions.append(Node(
            package='f1tenth_bringup', executable='viz_jpeg_node', name='viz_jpeg_node',
            prefix=f'taskset -c {affinity}', output='screen',
            parameters=[{
                'input_topics': [throttle_output(r) for r in images],
                'output_topics': [r['dest'] for r in images],
                'jpeg_quality': int(LaunchConfiguration('viz_jpeg_quality').perform(context)),
            }]))
    return actions


def generate_launch_description():
    args = []
    for key in ('enable_viz_relays',) + RATE_KEYS + ('viz_jpeg_quality',):
        default, description = get_default(key)
        args.append(DeclareLaunchArgument(key, default_value=str(default),
                                          description=description))
    args.append(DeclareLaunchArgument(
        'viz_relays_cpu_affinity', default_value='3',
        description="Cores for 'taskset -c' on the relay container and viz_jpeg_node. "
                    "Shares foxglove_bridge's core; see this file's docstring."))
    return LaunchDescription([
        SetEnvironmentVariable('ROS_SUPER_CLIENT', 'TRUE'),
        *args,
        OpaqueFunction(function=_relays),
    ])
