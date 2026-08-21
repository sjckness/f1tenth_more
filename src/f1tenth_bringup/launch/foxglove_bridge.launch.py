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

cpu_affinity (stack-wide CPU-budget investigation): foxglove_bridge is a
vendored binary, pinned via a 'taskset -c' launch prefix (same mechanism/
reasoning as f1tenth_localization/launch/ekf.launch.py's own ekf_node --
see that file's own comment). Previously documented here as "~5% CPU,
genuinely light... shares a pair with ekf_node/behavior_executor_node" --
that figure was WRONG, and this file's own paragraph above already had the
evidence: the 38.5%-with-client/35-37%-without measurement from the
foxglove CPU investigation was dismissed as predating "this particular
measurement environment" rather than reconciled, and the light-load
assumption stuck.

CORRECTED (core-remap pass, following a live CPU-contention investigation --
see that pass's own report): a live `ps -o %cpu` measurement (at REST, no
motion, no client connected) found foxglove_bridge at 52.2% CPU -- much
closer to this file's own earlier 35-38% figure than the "~5%" this
comment carried, confirming that was the anomaly, not the other way
round. Sharing cores 0,1 with ekf_filter_node/ekf_global_filter_node/
behavior_executor_node put ~152% combined demand on a 200% (2-core)
budget -- confirmed saturated live (cpu0/cpu1 both 99%+ busy), and
produced ekf_node's own "Failed to meet update rate!" warnings (see
ekf.launch.py's own matching comment for the full picture). foxglove_
bridge is viz-only and already latency-tolerant (the image throttle nodes
above exist specifically because a viz client doesn't need the pipeline's
full rate) -- moved to its own core (3) instead, off the EKF pair
entirely, same reasoning slam_toolbox already gets a dedicated core
(f1tenth_navigation/launch/slam.launch.py) rather than sharing.
"""

from f1tenth_params.param_defaults import get_default

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # ROS_SUPER_CLIENT (Discovery-Server EDP-visibility investigation --
    # confirmed root cause of "foxglove shows zero topics"/"ros2 topic list
    # shows only /parameter_events+/rosout"): under ROS_DISCOVERY_SERVER,
    # Fast-DDS registers every participant as a plain CLIENT by default --
    # a plain client only gets endpoint (topic/pub/sub) data relayed for
    # topics it's actively trying to match against its OWN registered
    # publishers/subscribers, not the whole graph. PDP (participant/node
    # discovery) has no such restriction, which is exactly why `ros2 node
    # list` worked while `ros2 topic list`/foxglove's topic browser didn't --
    # confirmed live: this env var, set TRUE for a standalone foxglove_bridge
    # test instance, took it from 2 advertised channels (/parameter_events,
    # /rosout -- the two every node matches trivially via its own default
    # rclcpp machinery) to 75 (the full real graph: /scan, /tf, /drive,
    # /slam/pose, everything). Confirmed present in this exact installed
    # Fast-DDS version (2.6.11) via ldd/strings, not XML-profile-only.
    # Scoped to just this launch tree (foxglove_bridge + its throttle
    # nodes), not stack-wide -- the real pub/sub nodes (ekf_node, mpc_corr,
    # slam_toolbox, ...) already know exactly which topics they need and
    # don't need full-graph visibility; making every participant a super
    # client would add needless discovery-cache overhead stack-wide for no
    # benefit. Also set in ~/.bashrc (covers ad-hoc `ros2 topic list`/the
    # ros2 CLI daemon, the same class of "introspection tool" as
    # foxglove_bridge) -- same reasoning as ROS_DISCOVERY_SERVER's own
    # three-places rollout (see supervisor_bringup.launch.py's own module
    # docstring).
    super_client_env = SetEnvironmentVariable('ROS_SUPER_CLIENT', 'TRUE')

    enable_default, enable_desc = get_default('enable_foxglove')
    enable_la = DeclareLaunchArgument(
        'enable_foxglove', default_value=str(enable_default), description=enable_desc)

    throttle_hz_default, throttle_hz_desc = get_default('foxglove_image_throttle_hz')
    throttle_hz_la = DeclareLaunchArgument(
        'foxglove_image_throttle_hz', default_value=str(throttle_hz_default),
        description=throttle_hz_desc)

    foxglove_cpu_affinity_la = DeclareLaunchArgument(
        'foxglove_cpu_affinity', default_value='3',
        description="Comma-separated core ids to pin foxglove_bridge to via "
                    "a 'taskset -c' launch prefix (vendored binary, can't "
                    "self-pin). Own dedicated core (moved off the EKF pair, "
                    "0,1 -- core-remap pass, see this file's own module "
                    "docstring for the live measurement that motivated it). "
                    "Must stay a valid, non-empty core list -- unlike this "
                    "stack's self-pinning nodes, an empty value here is a "
                    "shell-level 'taskset -c' error, not a graceful no-op "
                    "(see ekf.launch.py's own matching comment); remove "
                    "this Node's prefix= argument instead to fully disable "
                    "pinning.")

    foxglove_bridge_node = Node(
        package='foxglove_bridge',
        executable='foxglove_bridge',
        name='foxglove_bridge',
        condition=IfCondition(LaunchConfiguration('enable_foxglove')),
        prefix=['taskset -c ', LaunchConfiguration('foxglove_cpu_affinity')],
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
        super_client_env,
        enable_la, throttle_hz_la, foxglove_cpu_affinity_la, foxglove_bridge_node,
        image_raw_throttle_node, image_annotated_throttle_node,
    ])
