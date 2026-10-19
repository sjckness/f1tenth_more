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

FORWARDED ARGUMENTS (avoidance pass, 2026-10-19). That "needs no args passed in"
design had a sharp edge: this file included mpc_corr.launch.py with a bare
IncludeLaunchDescription and NO launch_arguments, so every mpc_corr argument
passed to stack_bringup -- or to the component supervisor's `navigation` entry --
was accepted at the top level and SILENTLY DROPPED here. mpc_corr came up on its
stack_params defaults whatever the command line said, with no warning, which is
exactly the kind of thing that costs an afternoon: the launch line looks right
and does nothing.

_FORWARDED below is the explicit, narrow fix: the boundary/wall-row keys this
pass needs to be able to A/B on the car, declared here with the SAME
stack_params defaults mpc_corr.launch.py uses, and passed through. Narrow on
purpose -- this is not a blanket re-export of mpc_corr's ~40 arguments, which
would duplicate every default in a second place and make this file a maintenance
mirror of that one. Everything not listed still self-resolves from
stack_params.yaml exactly as before, so adding these changes no default.

  ros2 launch f1tenth_navigation navigation.launch.py boundary_source:=costmap

and, through the supervisor, an `args:` entry under `navigation` in
f1tenth_bringup/config/components.yaml.

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

from f1tenth_params.param_defaults import get_default, get_value

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


#: mpc_corr arguments this file declares and forwards. See the module
#: docstring: narrow on purpose, and every default comes from stack_params.yaml
#: via get_default, so it is the same number mpc_corr.launch.py would have used.
_FORWARDED = (
    'boundary_source',
    'boundary_max_sources',
    'boundary_hard',
    'boundary_slack_weight',
    'boundary_slack_max',
    'max_wall_constraints',
    'wall_row_max_coast_s',
    'wall_row_coast_margin_m',
    'wall_row_allow_stereo_only',
    'other_walls_as_obstacles',
    'wall_row_end_margin_m',
    'wall_row_slack_weight',
    'wall_row_max_tf_age_s',
)


def generate_launch_description():
    enable_nav2 = get_value('enable_nav2')

    declarations = []
    for name in _FORWARDED:
        default, description = get_default(name)
        declarations.append(DeclareLaunchArgument(
            name, default_value=str(default), description=description))

    def include(package, launch_file, forward=False):
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory(package), 'launch', launch_file)),
            # Strings on purpose: launch_arguments values are command-line-shaped
            # text, and mpc_corr.launch.py is what types them (boundary_source
            # through ParameterValue(value_type=str), the rest through their own
            # DeclareLaunchArgument). Typing them here as well would be a second
            # place for the coercion to go wrong.
            launch_arguments=[(n, LaunchConfiguration(n)) for n in _FORWARDED]
            if forward else None,
        )

    if enable_nav2:
        # Nav2 owns the drive lane in this branch, mpc_corr is not launched, and
        # the forwarded keys reach nothing. Declared anyway (above) so the same
        # command line is accepted either way rather than erroring on an unknown
        # argument depending on a stack_params value the caller cannot see.
        actions = [include('f1tenth_navigation', 'nav2.launch.py')]
    else:
        actions = [
            include('f1tenth_control', 'mpc_corr.launch.py', forward=True),
            include('f1tenth_navigation', 'map_only.launch.py'),
        ]

    return LaunchDescription(declarations + actions)
