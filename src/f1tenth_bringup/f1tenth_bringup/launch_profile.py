"""Launch profiles: ``supervisor_bringup.launch.py profile:=<name> [profiles_dir:=DIR]``.

A profile is ``<profiles_dir>/<name>.yaml``. The default directory is the
package's ``config/profiles``. On the car it is ``~/.config/f1tenth_car/profiles``
(``scripts/car/profiles push`` copies the repo's files there, and
``car-stack start <name>`` points ``profiles_dir`` at it). That folder is
mounted read-only into the container, so a profile can change without a
rebuild or a container restart.

Two sections, both optional::

    launch_arguments:      # arguments of supervisor_bringup.launch.py
      planner_controller_mode: corridor_rrt
    component_args:        # {component of components.yaml: {launch argument: value}}
      perception: {yolo_backend: engine}

``launch_arguments`` become defaults: a ``name:=value`` on the command line
(or in ``~/.config/f1tenth_car/stack_args``) wins. ``component_args`` go to the
launch files of that component that declare the argument, after
``components.yaml`` and after the supervisor's own injections.

Anything a profile names must exist: an unknown launch argument, an unknown
component, or a component argument that none of the component's launch files
declares is an error. Today ``ros2 launch`` would accept and ignore it.

The arguments the supervisor writes itself from its own parameters (see
``SUPERVISOR_INJECTED_ARGS``) are refused in ``component_args``: set them in
``launch_arguments``.

CLI, used by ``car-stack`` and ``scripts/car/ready`` inside the container::

    python3 -m f1tenth_bringup.launch_profile check NAME [--profiles-dir DIR]
    python3 -m f1tenth_bringup.launch_profile show  NAME [--profiles-dir DIR]
    python3 -m f1tenth_bringup.launch_profile list  [--profiles-dir DIR]

``check`` exits 0 and prints ``OK <name> <sha256>``; otherwise it exits 1 and
prints one line per error.
"""

import argparse
import hashlib
import json
import os
import re
import sys

import yaml

SECTIONS = ('launch_arguments', 'component_args')

# Component launch arguments that component_supervisor_node.py writes itself
# from its own parameters (_apply_calibration_override,
# _apply_mission_logger_override, apply_planner_controller_mode,
# apply_rust_switches). A profile sets them through launch_arguments, where
# the supervisor reads them. test_launch_profile.py keeps this list in step
# with the supervisor's.
SUPERVISOR_INJECTED_ARGS = frozenset({
    'calibration',
    'enable_mission_logger',
    'planner_controller_mode',
    'proximity_guard',
    'costmap_boundary_impl',
    'corridor_impl',
    'mpc_solver_impl',
    'hard_stop_impl',
    'wall_tracker_impl',
})

# A bare file name: no path, so profile:= can never point outside profiles_dir.
_NAME_RE = re.compile(r'^[A-Za-z0-9_][A-Za-z0-9_.-]*$')


class ProfileError(RuntimeError):
    """A profile that is missing, malformed, or names something that does not exist."""


def default_profiles_dir():
    from ament_index_python.packages import get_package_share_directory
    return os.path.join(get_package_share_directory('f1tenth_bringup'), 'config', 'profiles')


def default_components_config():
    from ament_index_python.packages import get_package_share_directory
    return os.path.join(get_package_share_directory('f1tenth_bringup'), 'config',
                        'components.yaml')


def known_profiles(profiles_dir):
    try:
        return sorted(f[:-5] for f in os.listdir(profiles_dir) if f.endswith('.yaml'))
    except OSError:
        return []


def _str(value):
    # Values are strings, as they would be written after `name:=`. YAML
    # booleans become 'true'/'false', the form every launch file compares to.
    return str(value).lower() if isinstance(value, bool) else str(value)


def read_profile(name, profiles_dir=None):
    """Return the profile as a dict.

    The keys are ``name``, ``path``, ``sha256``, ``launch_arguments`` ({arg: str})
    and ``component_args`` ({component: {arg: str}}). Name ``none`` or '' gives an
    empty profile with path None.
    """
    empty = {'name': 'none', 'path': None, 'sha256': None,
             'launch_arguments': {}, 'component_args': {}}
    if name in ('', 'none', None):
        return empty
    profiles_dir = profiles_dir or default_profiles_dir()
    if not _NAME_RE.match(name):
        raise ProfileError(f'profile {name!r}: a bare name is expected (no path, no .yaml)')
    path = os.path.join(profiles_dir, f'{name}.yaml')
    if not os.path.isfile(path):
        raise ProfileError(f'profile {name!r}: no {path} '
                           f'(known: {known_profiles(profiles_dir)})')
    with open(path, 'rb') as f:
        raw = f.read()
    try:
        data = yaml.safe_load(raw) or {}
    except yaml.YAMLError as e:
        raise ProfileError(f'profile {name!r}: {path} is not valid YAML: {e}') from e
    if not isinstance(data, dict):
        raise ProfileError(f'profile {name!r}: {path} must be a mapping')
    unknown = sorted(set(data) - set(SECTIONS))
    if unknown:
        raise ProfileError(f'profile {name!r}: unknown sections {unknown} '
                           f'(known: {list(SECTIONS)})')
    launch_arguments = data.get('launch_arguments') or {}
    component_args = data.get('component_args') or {}
    if not isinstance(launch_arguments, dict):
        raise ProfileError(f'profile {name!r}: launch_arguments must be a mapping')
    if not isinstance(component_args, dict) or not all(
            isinstance(v, dict) or v is None for v in component_args.values()):
        raise ProfileError(f'profile {name!r}: component_args must be '
                           '{component: {argument: value}}')
    return {
        'name': name,
        'path': path,
        'sha256': hashlib.sha256(raw).hexdigest(),
        'launch_arguments': {str(k): _str(v) for k, v in launch_arguments.items()},
        'component_args': {str(c): {str(k): _str(v) for k, v in (args or {}).items()}
                           for c, args in component_args.items()},
    }


def load_profile(name, profiles_dir=None):
    """Return the launch arguments of profile ``name`` ({} for 'none')."""
    return read_profile(name, profiles_dir)['launch_arguments']


def load_registry(components_config=None):
    """components.yaml as {component: [{package, launch_file, args}, ...]}."""
    with open(components_config or default_components_config()) as f:
        raw = yaml.safe_load(f)['components']
    return {name: [dict(e, args=dict(e.get('args') or {})) for e in entries]
            for name, entries in raw.items()}


def launch_arguments_of(package, launch_file):
    """Names of the launch arguments ``ros2 launch package launch_file`` declares.

    Includes the arguments of included launch files, which also receive a
    ``name:=value`` given on the command line.
    """
    from ament_index_python.packages import get_package_share_directory
    from launch.launch_description_sources import get_launch_description_from_python_launch_file
    path = os.path.join(get_package_share_directory(package), 'launch', launch_file)
    description = get_launch_description_from_python_launch_file(path)
    try:
        declared = description.get_launch_arguments_with_include_launch_description_actions()
        return {action.name for action, _includes in declared}
    except Exception:  # noqa: BLE001 - an include that cannot be resolved offline
        return {action.name for action in description.get_launch_arguments()}


def resolve_component_args(component_args, registry, launch_args_of=launch_arguments_of):
    """Map {component: {arg: value}} to {component: {launch_file: {arg: value}}}.

    Each argument goes to the entries of the component whose launch file declares
    it. Raises ProfileError, listing every problem, when a component is unknown,
    an argument is the supervisor's own, or no launch file of the component
    declares it.
    """
    errors = []
    resolved = {}
    cache = {}
    for component, args in sorted(component_args.items()):
        if component not in registry:
            errors.append(f'component {component!r} is not in components.yaml '
                          f'(known: {sorted(registry)})')
            continue
        for arg, value in sorted(args.items()):
            if arg in SUPERVISOR_INJECTED_ARGS:
                errors.append(f'{component}.{arg}: set by the supervisor itself; '
                              f'put it in launch_arguments')
                continue
            targets, unreadable = [], []
            for entry in registry[component]:
                key = (entry['package'], entry['launch_file'])
                if key not in cache:
                    try:
                        cache[key] = launch_args_of(*key)
                    except Exception as e:  # noqa: BLE001
                        cache[key] = e
                if isinstance(cache[key], Exception):
                    unreadable.append(f'{key[0]}/{key[1]}: {cache[key]}')
                elif arg in cache[key]:
                    targets.append(entry['launch_file'])
            if not targets:
                # A launch file that cannot be loaded here (e.g. the ZED wrapper
                # off the car) matters only when no other one takes the argument.
                files = [e['launch_file'] for e in registry[component]]
                errors.append(f'{component}.{arg}: declared by none of {files}'
                              + (f" (unreadable: {'; '.join(unreadable)})" if unreadable else ''))
                continue
            for launch_file in targets:
                resolved.setdefault(component, {}).setdefault(launch_file, {})[arg] = value
    if errors:
        raise ProfileError('; '.join(errors))
    return resolved


def validate_profile(profile, declared, registry, launch_args_of=launch_arguments_of):
    """Raise ProfileError unless every name in the profile exists.

    Returns the resolved component args ({component: {launch_file: {arg: value}}}).
    ``declared`` is the set of supervisor_bringup launch arguments.
    """
    unknown = sorted(set(profile['launch_arguments']) - set(declared))
    errors = []
    if unknown:
        errors.append(f'not arguments of supervisor_bringup.launch.py: {unknown}')
    resolved = {}
    try:
        resolved = resolve_component_args(profile['component_args'], registry, launch_args_of)
    except ProfileError as e:
        errors.append(str(e))
    if errors:
        raise ProfileError(f"profile {profile['name']!r}: " + '; '.join(errors))
    return resolved


def supervisor_declared_args():
    return launch_arguments_of('f1tenth_bringup', 'supervisor_bringup.launch.py')


def main(argv=None):
    parser = argparse.ArgumentParser(prog='launch_profile', description=__doc__.split('\n')[0])
    parser.add_argument('command', choices=('check', 'show', 'list'))
    parser.add_argument('name', nargs='?')
    parser.add_argument('--profiles-dir', default=None)
    parser.add_argument('--components-config', default=None)
    args = parser.parse_args(argv)
    profiles_dir = args.profiles_dir or default_profiles_dir()
    if args.command == 'list':
        for name in known_profiles(profiles_dir):
            print(name)
        return 0
    if not args.name:
        parser.error(f'{args.command} needs a profile name')
    try:
        profile = read_profile(args.name, profiles_dir)
        resolved = validate_profile(profile, supervisor_declared_args(),
                                    load_registry(args.components_config))
    except ProfileError as e:
        print(f'ERROR {e}')
        return 1
    if args.command == 'show':
        print(json.dumps(dict(profile, resolved_component_args=resolved), indent=2))
    else:
        print(f"OK {profile['name']} {profile['sha256']}")
    return 0


if __name__ == '__main__':
    sys.exit(main())
