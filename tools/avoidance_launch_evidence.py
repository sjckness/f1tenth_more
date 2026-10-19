#!/usr/bin/env python3
"""Resolve mpc_corr's boundary parameters as a real launch would, and diff the modes.

Why this exists. The avoidance pass moved `boundary_source` to 'walls' by
default and taught navigation.launch.py to forward the wall-row arguments it
used to drop silently. Both claims are about what the LAUNCH SYSTEM produces,
not about what a yaml file says, so asserting them needs the launch description
evaluated rather than read: a default can be right in stack_params.yaml and
still never reach the node (which is precisely the bug that was fixed here).

This performs the substitutions mpc_corr.launch.py's Node would perform, for a
given set of launch arguments, and prints the resulting parameter dict. Run it
with no arguments for the shipped defaults, or with key:=value overrides:

    python3 tools/avoidance_launch_evidence.py
    python3 tools/avoidance_launch_evidence.py boundary_source:=costmap
    python3 tools/avoidance_launch_evidence.py --diff boundary_source:=costmap

--diff prints only the keys that differ from the shipped defaults, which is the
A/B evidence: flipping one argument must change `boundary_source` and nothing
else.

No node is constructed and nothing is spun -- this is the launch description,
evaluated.
"""

import argparse
import importlib.util
import os
import sys

from ament_index_python.packages import get_package_share_directory

from launch import LaunchContext
from launch.actions import DeclareLaunchArgument
from launch.utilities import perform_substitutions
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


BOUNDARY_KEYS = (
    'boundary_source', 'boundary_max_sources', 'boundary_hard',
    'boundary_slack_weight', 'boundary_slack_max', 'max_wall_constraints',
    'wall_row_max_coast_s', 'wall_row_coast_margin_m',
    'wall_row_allow_stereo_only', 'other_walls_as_obstacles',
    'wall_obstacle_spacing_m', 'wall_obstacle_radius_m',
    'wall_row_end_margin_m', 'wall_row_merge_angle_deg',
    'wall_row_merge_offset_m', 'wall_row_slack_weight', 'wall_row_max_tf_age_s',
    'use_hard_boundary_constraints', 'car_radius', 'avoidance_margin',
)


def _load(path, name):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


def _launch_file(package, name):
    """The INSTALLED launch file -- what ros2 launch would actually run."""
    return os.path.join(get_package_share_directory(package), 'launch', name)


def _resolve(text, context):
    """One substitution-bearing value -> plain text."""
    if isinstance(text, ParameterValue):
        text = text.value
    if isinstance(text, (str, int, float, bool)) or text is None:
        return text
    if not isinstance(text, (list, tuple)):
        text = [text]
    return perform_substitutions(context, list(text))


def mpc_corr_parameters(overrides):
    """{param: resolved value} for mpc_corr's Node under `overrides`."""
    module = _load(_launch_file('f1tenth_control', 'mpc_corr.launch.py'), 'mpc_l')
    ld = module.generate_launch_description()
    context = LaunchContext()
    for entity in ld.entities:
        if isinstance(entity, DeclareLaunchArgument):
            value = overrides.get(entity.name)
            if value is None:
                value = _resolve(entity.default_value, context)
            context.launch_configurations[entity.name] = str(value)
    out = {}
    for entity in ld.entities:
        if not isinstance(entity, Node):
            continue
        for block in entity._Node__parameters or []:
            if isinstance(block, dict):
                for key, value in block.items():
                    out[_resolve(key, context)] = _resolve(value, context)
    return out


def navigation_forwarding():
    """(declared defaults, forwarded names) of navigation.launch.py."""
    module = _load(_launch_file('f1tenth_navigation', 'navigation.launch.py'), 'nav_l')
    ld = module.generate_launch_description()
    context = LaunchContext()
    declared = {}
    for entity in ld.entities:
        if isinstance(entity, DeclareLaunchArgument):
            declared[entity.name] = _resolve(entity.default_value, context)
    forwarded = set()
    for entity in ld.entities:
        for name, _value in (getattr(entity, 'launch_arguments', None) or []):
            forwarded.add(_resolve(name, context))
    return declared, forwarded


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('overrides', nargs='*', help='key:=value launch overrides')
    parser.add_argument('--diff', action='store_true',
                        help='print only what differs from the shipped defaults')
    args = parser.parse_args()

    overrides = {}
    for item in args.overrides:
        if ':=' not in item:
            parser.error(f'expected key:=value, got {item!r}')
        key, value = item.split(':=', 1)
        overrides[key] = value

    declared, forwarded = navigation_forwarding()
    print('navigation.launch.py -- declared and forwarded to mpc_corr.launch.py')
    missing = [k for k in forwarded if k not in declared]
    for key in sorted(forwarded):
        print(f'  {key:30s} default {declared.get(key, "<undeclared>")}')
    print(f'  ({len(forwarded)} forwarded, {len(missing)} forwarded-but-undeclared)')
    if missing:
        print(f'  WARNING undeclared: {sorted(missing)}')

    shipped = mpc_corr_parameters({})
    print('\nmpc_corr parameters, SHIPPED DEFAULTS (no launch overrides)')
    for key in BOUNDARY_KEYS:
        if key in shipped:
            print(f'  {key:30s} = {shipped[key]}')

    if overrides:
        other = mpc_corr_parameters(overrides)
        label = ' '.join(f'{k}:={v}' for k, v in overrides.items())
        print(f'\nmpc_corr parameters WITH {label}')
        for key in BOUNDARY_KEYS:
            if key not in other:
                continue
            changed = other[key] != shipped.get(key)
            if args.diff and not changed:
                continue
            mark = '  <-- CHANGED' if changed else ''
            print(f'  {key:30s} = {other[key]}{mark}')
        changed_keys = sorted(k for k in set(shipped) | set(other)
                              if shipped.get(k) != other.get(k))
        print(f'\n  keys that differ in total: {changed_keys}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
