"""stack_info.collect / to_status: /viz/stack_info from an environment and fake files.

Four cases: the car's container, a sim on linus, a native stack, and the car with a
stack_args trial. The image provenance comes from fake SOURCES / CAR_SYNC.json files
through f1tenth_params.stack_provenance.read_image_provenance, as in the node.
"""

import json

from f1tenth_diagnostics import stack_info
from f1tenth_params.stack_provenance import read_image_provenance

CAR_CMD = ('/usr/bin/python3 /opt/ros/jazzy/bin/ros2 launch f1tenth_bringup '
           'supervisor_bringup.launch.py enable_intelligence:=true enable_mission_logger:=true '
           'enable_comms:=false profile:=test_orin_mpc '
           'profiles_dir:=/home/fabiocar/.config/f1tenth_car/profiles')
CAR_PROFILE_ARGS = 'enable_intelligence:=true enable_mission_logger:=true enable_comms:=false'
IMAGE = 'f1tenth:jazzy-orin-e804ac2-norust'
IMAGE_ID = 'sha256:3f2a9c1d77e0b5a1c0ffee0000000000'
COMMIT = 'e804ac20a7a6f402adb5ac49b45536d65b700bcb'


def _image_git(tmp_path):
    sources = tmp_path / 'SOURCES'
    sources.write_text(f'commit {COMMIT}\nsubject mpc_controller 0.2.0: legacy\n'
                       'branch campaign3-rrt\ndescribe mpc_controller/v0.1.0-12-ge804ac2\n')
    car_sync = tmp_path / 'CAR_SYNC.json'
    car_sync.write_text(json.dumps({'captured_at': '2026-11-23', 'native_branch': 'x'}))
    return read_image_provenance(str(sources), str(car_sync))


def _car_env():
    # What ctl_container.env exports to the stack: no F1T_CONTAINER_ROLE / F1TENTH_IMAGE_*.
    return {'F1T_CTL_ENV': '/opt/f1tenth_ws/src/f1tenth_ctl/config/ctl_container.env',
            'F1T_CAR_PROFILE_ARGS': CAR_PROFILE_ARGS, 'ROS_DISTRO': 'jazzy',
            'RMW_IMPLEMENTATION': 'rmw_fastrtps_cpp',
            'ROS_DISCOVERY_SERVER': '127.0.0.1:11811'}


def _car_pid1():
    # compose.yaml's variables, in the container's PID 1.
    return {'F1T_CONTAINER_ROLE': 'car', 'F1TENTH_IMAGE_REF': IMAGE,
            'F1TENTH_IMAGE_ID': IMAGE_ID}


def _collect(env, pid1, host, container, cmd, stack_args, git, sim_time=False):
    return stack_info.collect(env, pid1, host, container, cmd, stack_args, git, sim_time,
                              '2026-11-24T11:00:00+00:00')


def test_car_container(tmp_path):
    info = _collect(_car_env(), _car_pid1(), 'jetson-orin', True, CAR_CMD,
                    'profile:=test_orin_mpc profiles_dir:=/home/fabiocar/.config/f1tenth_car/'
                    'profiles\n', _image_git(tmp_path))
    status = stack_info.to_status(info, 'jetson-orin')
    values = {kv.key: kv.value for kv in status.values}
    assert status.level == bytes([stack_info.OK])
    assert status.name == 'car'
    assert status.hardware_id == 'jetson-orin'
    assert status.message == f'car | {IMAGE} | campaign3-rrt@e804ac2'
    assert values['container'] == 'true'
    assert values['image_ref'] == IMAGE
    assert values['image_id'] == '3f2a9c1d77e0'
    assert values['launch_profile'] == 'test_orin_mpc'
    assert values['profile_args'] == CAR_PROFILE_ARGS
    assert values['stack_args_trial'] == 'none'
    assert values['dirty'] == 'false'
    assert values['discovery_server'] == '127.0.0.1:11811'
    assert list(values)[:4] == ['profile', 'launch_profile', 'container', 'image_ref']


def test_car_with_a_stack_args_trial_warns(tmp_path):
    info = _collect(_car_env(), _car_pid1(), 'jetson-orin', True, CAR_CMD + ' guard:=rust',
                    'profile:=test_orin_mpc guard:=rust\n', _image_git(tmp_path))
    status = stack_info.to_status(info, 'jetson-orin')
    assert status.level == bytes([stack_info.WARN])
    assert 'WARN: stack_args trial: guard:=rust' in status.message


def test_sim_on_linus_is_sim_host_and_reads_profile_args_from_the_launch_command():
    cmd = ('ros2 launch f1tenth_bringup supervisor_bringup.launch.py sim:=isaac '
           'use_sim_time:=true fastdds_profile:=/x/loopback.xml')
    git = {'commit': COMMIT, 'branch': 'main', 'describe': 'v1', 'subject': 's', 'dirty': False}
    info = _collect({'ROS_DISTRO': 'jazzy'}, {}, 'linus', False, cmd, None, git, sim_time=True)
    status = stack_info.to_status(info, 'linus')
    values = {kv.key: kv.value for kv in status.values}
    assert status.name == 'sim-linus'
    assert status.message == 'sim-linus | native | main@e804ac2'
    assert values['profile_args'] == ('sim:=isaac use_sim_time:=true '
                                      'fastdds_profile:=/x/loopback.xml')
    assert values['stack_args'] == 'absent'
    assert values['use_sim_time'] == 'true'
    assert values['domain_id'] == 'unset (0)'
    assert status.level == bytes([stack_info.OK])


def test_native_dirty_tree_without_a_known_profile_says_unknown_and_warns():
    git = {'commit': COMMIT, 'branch': 'jazzy', 'dirty': True}
    info = _collect({}, {}, 'tpad', False, None, None, git)
    status = stack_info.to_status(info, 'tpad')
    values = {kv.key: kv.value for kv in status.values}
    assert status.name == 'unknown'
    assert values['launch_cmd'] == 'unknown'
    assert values['describe'] == 'unknown'
    assert status.level == bytes([stack_info.WARN])
    assert status.message.endswith('WARN: dirty tree')


def test_native_through_f1tenth_ctl_is_native_and_an_unknown_commit_warns():
    info = _collect({'F1T_CTL_ENV': '/home/a/f1tenth_more/src/f1tenth_ctl/config/ctl.env'},
                    {}, 'thor', False, None, None, {})
    status = stack_info.to_status(info, 'thor')
    assert status.name == 'native'
    assert status.message == 'native | native | unknown@unknown | WARN: commit unknown'
