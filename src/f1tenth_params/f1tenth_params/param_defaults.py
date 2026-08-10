"""Shared launch-parameter defaults, single source of truth for every default value
and description across the f1tenth_more workspace's launch files -- see
f1tenth_params/config/stack_params.yaml.

This package is deliberately dependency-free (no exec_depend on any other f1tenth_*
package): f1tenth_bringup's stack_bringup.launch.py includes almost every other
package's launch file, and nearly every launch file in the workspace imports from
here -- if this lived inside f1tenth_bringup itself (as it originally did), every
consumer package would need to depend on f1tenth_bringup, which already depends on
them, a build-order cycle colcon refuses to resolve. Keeping this as its own leaf
package avoids that entirely.

Two ways to use this from a launch file:
  - get_default(name) -> (value, description): for a normal DeclareLaunchArgument,
    pass value (str()-ed) as default_value and description straight through.
  - get_value(name): just the value -- used for the 5 stack-wide branching args
    (camera_source, localization_source, enable_llm, use_behavior_tree,
    enable_nav2), which are plain Python variables at parse time, not
    DeclareLaunchArgument/LaunchConfiguration.
  - get_path_default(name, package='f1tenth_bringup') -> (absolute_path,
    description): for path-type entries, whose yaml `default` is a path relative
    to a package's install share directory (package defaults to f1tenth_bringup,
    since that's where the actual config/*.yaml files this resolves still live;
    see stack_params.yaml's `map` entry for the one exception).
"""

import functools
import os

import yaml

from ament_index_python.packages import get_package_share_directory


@functools.lru_cache(maxsize=1)
def _load():
    path = os.path.join(
        get_package_share_directory('f1tenth_params'), 'config', 'stack_params.yaml')
    with open(path) as f:
        return yaml.safe_load(f) or {}


def get_default(name):
    """Return (value, description) for `name`, sourced from stack_params.yaml."""
    entry = _load()[name]
    return entry['default'], entry['description']


def get_value(name):
    """Return just the yaml default value for `name` -- for the 5 branching args,
    this call itself IS the value: there is no DeclareLaunchArgument/CLI override
    for these, so any `name:=...` passed on the CLI is silently ignored.
    """
    return _load()[name]['default']


def get_path_default(name, package='f1tenth_bringup'):
    """Return (absolute_path, description) for a path-type entry: yaml's relative
    `default` (e.g. "config/vesc.yaml") joined onto `package`'s install share
    directory, resolved eagerly to a plain string (DeclareLaunchArgument's
    default_value needs a real path, not a lazy Substitution, for these).
    """
    entry = _load()[name]
    absolute_path = os.path.join(get_package_share_directory(package), entry['default'])
    return absolute_path, entry['description']


def get_odom_topic():
    """The active odometry topic given the current localization_source (one of the
    5 stack-wide branching args, see stack_params.yaml): '/odometry/filtered'
    (EKF-fused x/y/yaw + gyro yaw rate, see f1tenth_bringup/config/ekf.yaml) when
    localization_source is 'ekf', '/odom' (raw wheel/steering dead reckoning from
    vesc_to_odom_node_backup, no IMU at all) when 'raw_odom'.

    Single source of truth so every odometry consumer stays in lockstep with
    whichever map -> odom source f1tenth_localization/launch/localization.launch.py
    actually brought up -- before this, several consumers (mpc_corr, the BT's
    CheckStopCondition, Nav2's bt_navigator) hardcoded '/odom' independently of
    localization_source, so switching to 'ekf' silently left them still reading the
    unfused topic. Callable both at launch-parse time (launch files, same as
    get_value()) and from plain node code (e.g. behavior_executor_node.py already
    imports get_value() the same way) since this package is dependency-free.
    """
    return '/odometry/filtered' if get_value('localization_source') == 'ekf' else '/odom'
