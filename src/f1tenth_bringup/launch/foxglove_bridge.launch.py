"""foxglove_bridge -- a proper, independently-supervisable component: its own launch
file (this one), a param toggle (enable_foxglove), and controllable via
component_supervisor_node's generic ~/control_component / /restart_component services
using component_name='dev_tools' (see components.yaml's dev_tools entry) -- the same
generic mechanism every other component uses, nothing foxglove-specific needed there.

Included by stack_bringup.launch.py's own DEV TOOLS section (no separate inline copy
kept there anymore -- previously duplicated verbatim, now this is the one definition).

Image throttle nodes (perception-optimization pass, following the foxglove CPU
investigation): foxglove_bridge 3.3.0's own parameter set (read directly from
the installed foxglove_bridge_launch.xml) has no per-topic/per-client rate
control at all -- topic_whitelist/client_topic_whitelist are regex NAME
filters, not Hz limiters. /camera/image_raw and /camera/image_annotated
publish at 30Hz for the real perception pipeline (yolo_detector_node,
detection_3d_node); a visualization client has no need for the full rate.
topic_tools' throttle node (package topic_tools, exec throttle) republishes a
slowed-down COPY on a separate *_viz topic -- the real 30Hz topics feeding the
pipeline are untouched, only the visualization-only copy is throttled. Note
from the same investigation: most of foxglove_bridge's own CPU cost turned out
to be a steady-state cost roughly independent of whether a client is
connected (38.5% with a client connected vs. 35-37% without, measured live) --
almost certainly from topic_whitelist's default '.*' continuously introspecting
the whole ROS graph, not from per-client data streaming. This throttle
addresses the image-serialization/bandwidth cost a client incurs when actually
viewing an image panel; it does not address that separate baseline cost.

throttle's CLI is positional argv (messages|bytes, in_topic, rate, [out_topic]),
NOT ROS parameters -- despite "throttle_type"/"input_topic"/etc. appearing as
strings inside the compiled binary (a red herring: those are internal/log
names, not declared parameter names). Confirmed empirically: `ros2 run
topic_tools throttle --ros-args -p throttle_type:=messages ...` fails with
"Throttle type is missing", while `ros2 run topic_tools throttle messages
<topic> <rate> <out_topic>` runs correctly. Use `arguments=[...]` here, not
`parameters=[{...}]`.
"""

from f1tenth_params.param_defaults import get_default

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    enable_default, enable_desc = get_default('enable_foxglove')
    enable_la = DeclareLaunchArgument(
        'enable_foxglove', default_value=str(enable_default), description=enable_desc)

    throttle_hz_default, throttle_hz_desc = get_default('foxglove_image_throttle_hz')
    throttle_hz_la = DeclareLaunchArgument(
        'foxglove_image_throttle_hz', default_value=str(throttle_hz_default),
        description=throttle_hz_desc)

    foxglove_bridge_node = Node(
        package='foxglove_bridge',
        executable='foxglove_bridge',
        name='foxglove_bridge',
        condition=IfCondition(LaunchConfiguration('enable_foxglove')),
        parameters=[{
            'port': 8765,
            'address': '0.0.0.0',
            # connectionGraph capability dropped deliberately -- confirmed
            # unrelated to the real ERROR/WARN diagnostics. Trade-off:
            # Foxglove's connection-graph panel stops working.
            'capabilities': [
                'clientPublish', 'parameters', 'parametersSubscribe',
                'services', 'assets',
            ],
        }]
    )

    # Both gated on enable_foxglove -- no point throttling a viz-only copy
    # nobody's bridging. output='screen' so a failure here is actually visible
    # (its own past absence is why the first version of this code silently
    # exited with no diagnosable error at all).
    image_raw_throttle_node = Node(
        package='topic_tools',
        executable='throttle',
        name='image_raw_viz_throttle',
        output='screen',
        condition=IfCondition(LaunchConfiguration('enable_foxglove')),
        arguments=[
            'messages', '/camera/image_raw',
            LaunchConfiguration('foxglove_image_throttle_hz'),
            '/camera/image_raw/viz',
        ],
    )

    image_annotated_throttle_node = Node(
        package='topic_tools',
        executable='throttle',
        name='image_annotated_viz_throttle',
        output='screen',
        condition=IfCondition(LaunchConfiguration('enable_foxglove')),
        arguments=[
            'messages', '/camera/image_annotated',
            LaunchConfiguration('foxglove_image_throttle_hz'),
            '/camera/image_annotated/viz',
        ],
    )

    return LaunchDescription([
        enable_la, throttle_hz_la, foxglove_bridge_node,
        image_raw_throttle_node, image_annotated_throttle_node,
    ])
