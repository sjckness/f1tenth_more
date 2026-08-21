"""Navigation/drive-command group: the Nav2 stack (nav2.launch.py) when enable_nav2
is true, or mpc_controller's mpc_corr (f1tenth_control/launch/mpc_corr.launch.py)
plus a standalone map_server (map_only.launch.py) when false -- one restartable
unit for component_supervisor_node, and the single site stack_bringup.launch.py/
vesc.launch.py both defer to instead of each keeping their own copy of this branch.

Mirrors f1tenth_localization/launch/localization.launch.py's branch-ownership
pattern (Phase 2): self-resolves enable_nav2 the same way localization.launch.py
self-resolves localization_source -- a plain Python value read directly from
stack_params.yaml, not a DeclareLaunchArgument, so this file needs no args passed
in and stays consistent regardless of which caller includes it.

enable_nav2 previously only toggled whether Nav2 came up at all (nothing filled in
for the "false" case -- see stack_bringup.launch.py's old mpc_bringup, assigned but
never added to its returned LaunchDescription). It now also decides which process
backs the ackermann_mux "navigation" lane; see stack_params.yaml's own enable_nav2
entry.

The false branch also brings up map_only.launch.py alongside mpc_corr: /map is
still worth publishing (RViz/Foxglove map display, anything else that expects it)
even with the rest of the Nav2 stack down. map_only.launch.py is map.launch.py
paired with its own single-node lifecycle_manager -- map_server is a Nav2 lifecycle
node and needs one to ever leave `unconfigured` -- see that file's own docstring
for why it's a separate lifecycle_manager_map rather than reusing
lifecycle_manager_navigation's name.
"""

import os

from ament_index_python.packages import get_package_share_directory

from f1tenth_params.param_defaults import get_value

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    enable_nav2 = get_value('enable_nav2')

    def include(package, launch_file):
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory(package), 'launch', launch_file))
        )

    if enable_nav2:
        actions = [include('f1tenth_navigation', 'nav2.launch.py')]
    else:
        actions = [
            include('f1tenth_control', 'mpc_corr.launch.py'),
            include('f1tenth_navigation', 'map_only.launch.py'),
        ]

    return LaunchDescription(actions)
