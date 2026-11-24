"""What /viz/stack_info says: which stack is running, from which image or tree (pure).

stack_info_node.py gathers the inputs (environment, files, /proc, git) and
publishes to_status(collect(...)) as a diagnostic_msgs/DiagnosticStatus, so
Foxglove shows one line: "<profile> | <image or 'native'> | <branch>@<commit7>".

Every function here takes its inputs as arguments: the tests build them from a
dict and a few fake files. A value that is not known is the string 'unknown',
never a guess.

WHAT THE STACK'S PROCESSES SEE (checked 2026-11-24 on main). f1tenth_ctl starts
the stack through units.UNIT_SCRIPT, the proc backend included: it unsets every
variable but config.BASE_KEYS, F1T_CTL_ENV and F1T_UNIT_*, then sources the env
file with `set -a`. So in the car's container the stack sees what
ctl_container.env exports (F1T_CAR_PROFILE_ARGS, F1T_STACK_ARGS_STATUS,
F1TENTH_IMAGE_SOURCES, F1TENTH_CAR_SYNC) but NOT what compose.yaml gives the
container (F1T_CONTAINER_ROLE, F1TENTH_IMAGE_REF, F1TENTH_IMAGE_ID). Those are
still in the environment of the container's PID 1, which the node reads
(/proc/1/environ, same user) when its own environment lacks them: pid1_env.
"""

import os
import shlex

UNKNOWN = 'unknown'
OK, WARN = 0, 1
#: stack_args tokens that only select a launch profile (scripts/car car-stack writes
#: them on every start): not a trial.
PROFILE_TOKENS = ('profile', 'profiles_dir')
#: F1T_CTL_ENV basenames that name the profile when nothing better does.
CTL_ENV_PROFILES = {'ctl_container.env': 'car', 'ctl.env': 'native'}
_SIM_OFF = ('', 'none', 'false', '0')


def launch_args(text):
    """The name:=value tokens of a command line or a stack_args line, in order, as pairs."""
    if not text:
        return []
    try:
        words = shlex.split(text)
    except ValueError:
        words = text.split()
    out = []
    for w in words:
        name, sep, value = w.partition(':=')
        if sep and name and not name.startswith('-'):
            out.append((name, value))
    return out


def launch_arg(launch_cmd, name):
    """The last name:= value on the launch command line (the one ros2 launch uses), else None."""
    value = None
    for n, v in launch_args(launch_cmd):
        if n == name:
            value = v
    return value


def stack_args_trial(stack_args_text):
    """The stack_args tokens that change the stack beyond picking a profile ([] = no trial)."""
    return [f'{n}:={v}' for n, v in launch_args(stack_args_text) if n not in PROFILE_TOKENS]


def stack_profile(env, pid1_env, launch_cmd, hostname):
    """car / sim-<host> / native (or the container role), from what is known; else 'unknown'."""
    sim = launch_arg(launch_cmd, 'sim')
    if sim is not None and sim.lower() not in _SIM_OFF:
        return f'sim-{hostname}'
    role = env.get('F1T_CONTAINER_ROLE') or pid1_env.get('F1T_CONTAINER_ROLE')
    if role:
        return role
    ctl_env = os.path.basename(env.get('F1T_CTL_ENV', ''))
    if ctl_env.startswith('sim_'):
        return f'sim-{hostname}'
    if ctl_env in CTL_ENV_PROFILES:
        return CTL_ENV_PROFILES[ctl_env]
    return UNKNOWN


def _or_unknown(value):
    return UNKNOWN if value in (None, '') else str(value)


def collect(env, pid1_env, hostname, container, launch_cmd, stack_args_text, git,
            use_sim_time, node_start_utc):
    """
    Return the stack info as a dict (insertion order = the order Foxglove shows).

    env: the node's environment. pid1_env: the container's PID 1 environment ({} outside a
    container, or unreadable). container: running in a container. launch_cmd: the stack's
    top-level launch command line (stack_launch_cmd), or None. stack_args_text: the content
    of ~/.config/f1tenth_car/stack_args, None when the file is absent. git: commit, branch,
    describe, subject, dirty (read_image_provenance in an image, git in a checkout; missing
    keys are unknown).
    """
    image_ref = env.get('F1TENTH_IMAGE_REF') or pid1_env.get('F1TENTH_IMAGE_REF')
    image_id = env.get('F1TENTH_IMAGE_ID') or pid1_env.get('F1TENTH_IMAGE_ID')
    if image_id:
        image_id = image_id.split(':', 1)[-1][:12]
    profile_args = env.get('F1T_CAR_PROFILE_ARGS')
    if profile_args is None and launch_cmd:
        profile_args = ' '.join(f'{n}:={v}' for n, v in launch_args(launch_cmd))
    dirty = git.get('dirty')
    info = {
        'profile': stack_profile(env, pid1_env, launch_cmd, hostname),
        'launch_profile': _or_unknown(launch_arg(launch_cmd, 'profile')),
        'container': 'true' if container else 'false',
        'image_ref': _or_unknown(image_ref) if container else 'native',
        'image_id': _or_unknown(image_id) if container else 'native',
        'branch': _or_unknown(git.get('branch')),
        'commit': _or_unknown(git.get('commit')),
        'describe': _or_unknown(git.get('describe')),
        'subject': _or_unknown(git.get('subject')),
        'dirty': UNKNOWN if dirty is None else ('true' if dirty else 'false'),
        'profile_args': _or_unknown(profile_args),
        'stack_args': ('absent' if stack_args_text is None
                       else (stack_args_text.strip() or 'empty')),
        'stack_args_trial': ' '.join(stack_args_trial(stack_args_text)) or 'none',
        'launch_cmd': _or_unknown(launch_cmd),
        'ros_distro': _or_unknown(env.get('ROS_DISTRO')),
        'rmw': _or_unknown(env.get('RMW_IMPLEMENTATION')),
        'domain_id': env.get('ROS_DOMAIN_ID') or 'unset (0)',
        'discovery_server': env.get('ROS_DISCOVERY_SERVER') or 'unset',
        'use_sim_time': 'true' if use_sim_time else 'false',
        'node_start_utc': node_start_utc,
    }
    return info


def warnings(info):
    """Why the level is WARN: a stack_args trial, a dirty tree, an unknown commit ([] = OK)."""
    out = []
    if info['stack_args_trial'] != 'none':
        out.append(f'stack_args trial: {info["stack_args_trial"]}')
    if info['dirty'] == 'true':
        out.append('dirty tree')
    if info['commit'] == UNKNOWN:
        out.append('commit unknown')
    return out


def summary(info):
    """The one line: '<profile> | <image ref or native> | <branch>@<commit7>'."""
    commit = info['commit'][:7] if info['commit'] != UNKNOWN else UNKNOWN
    return f'{info["profile"]} | {info["image_ref"]} | {info["branch"]}@{commit}'


def to_status(info, hostname, name_prefix=''):
    """Build the diagnostic_msgs/DiagnosticStatus of `info`."""
    from diagnostic_msgs.msg import DiagnosticStatus, KeyValue
    warn = warnings(info)
    status = DiagnosticStatus()
    status.level = bytes([WARN if warn else OK])
    status.name = name_prefix + info['profile']
    status.hardware_id = hostname
    status.message = summary(info) + (f' | WARN: {"; ".join(warn)}' if warn else '')
    status.values = [KeyValue(key=k, value=str(v)) for k, v in info.items()]
    return status
