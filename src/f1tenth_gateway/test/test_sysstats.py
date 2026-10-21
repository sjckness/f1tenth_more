"""
sysstats.py on fixture files: /proc/stat, cpu lists, the GPU load file, /proc/<pid>.

fixtures/sysstats holds real files from the Orin (L4T R36.5, 2026-10-22):
two /proc/stat samples 1 s apart, the GPU's load file (idle: 0), a status and
a stat file. The /proc and /sys trees are put together under tmp_path.
"""

import io
import os
import shutil
import subprocess
import time

from f1tenth_gateway import sysstats
from f1tenth_gateway.sysstats import (
    cpu_percent, format_cpu_list, friendly_name, parse_cpu_list, parse_gpu_load,
    parse_nvidia_smi, parse_proc_stat, scan_procs, SysStats)
import pytest

FIX = os.path.join(os.path.dirname(__file__), 'fixtures', 'sysstats')


def fixture(name):
    with open(os.path.join(FIX, name)) as f:
        return f.read()


def write(path, text, mode='w'):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, mode) as f:
        f.write(text)


def add_proc(proc, pid, allowed, cmdline=b'', ppid=1, ticks=(0, 0), start=1000,
             state='R (running)'):
    """Put a process under proc/<pid>: the Orin's status and stat with our values in."""
    status = fixture('status_cat').replace('Cpus_allowed_list:\t0-11',
                                           f'Cpus_allowed_list:\t{allowed}')
    status = status.replace('PPid:\t187791', f'PPid:\t{ppid}')
    status = status.replace('State:\tR (running)', f'State:\t{state}')
    write(f'{proc}/{pid}/status', status)
    fields = fixture('stat_cat').split()
    fields[13], fields[14], fields[21] = str(ticks[0]), str(ticks[1]), str(start)
    write(f'{proc}/{pid}/stat', ' '.join(fields) + '\n')
    write(f'{proc}/{pid}/cmdline', cmdline, 'wb')


def test_cpu_list_ranges_and_singles_both_ways():
    assert parse_cpu_list('0-3,6') == [0, 1, 2, 3, 6]
    assert parse_cpu_list('0-11\n') == list(range(12))
    assert parse_cpu_list('10,11') == [10, 11]
    assert parse_cpu_list('5') == [5]
    assert parse_cpu_list('\n') == []
    with pytest.raises(ValueError):
        parse_cpu_list('3-1')
    assert format_cpu_list([6, 0, 1, 2, 3]) == '0-3,6'
    assert format_cpu_list([10, 11]) == '10-11'
    assert format_cpu_list([4]) == '4'


def test_proc_stat_deltas_give_the_per_core_percent_with_iowait_as_idle():
    a = parse_proc_stat(fixture('proc_stat_1'))
    b = parse_proc_stat(fixture('proc_stat_2'))
    assert sorted(a) == ['cpu'] + sorted(f'cpu{i}' for i in range(12))
    # cpu1: user +20, system +2, idle +87 -> 22 busy of 109.
    assert cpu_percent(a['cpu1'], b['cpu1']) == 20.2
    # cpu0: user +1, softirq +1, idle +97 -> 2 of 99.
    assert cpu_percent(a['cpu0'], b['cpu0']) == 2.0
    # all: busy 68 of 1203.
    assert cpu_percent(a['cpu'], b['cpu']) == 5.7
    assert cpu_percent(None, b['cpu']) is None
    assert cpu_percent(b['cpu'], b['cpu']) is None
    # user +10 (guest +5 is inside it, not on top), iowait +30, idle +60: 10 % busy, not 40.
    a = parse_proc_stat('cpu0 100 0 0 1000 50 0 0 0 20 0\n')
    b = parse_proc_stat('cpu0 110 0 0 1060 80 0 0 0 25 0\n')
    assert cpu_percent(a['cpu0'], b['cpu0']) == 10.0


def test_sampler_reports_offline_cores_without_a_percent(tmp_path):
    proc, sys = str(tmp_path / 'proc'), str(tmp_path / 'sys')
    write(f'{proc}/stat', fixture('proc_stat_1'))
    write(f'{sys}/devices/system/cpu/present', '0-11\n')
    write(f'{sys}/devices/system/cpu/online', '0-9,11\n')
    st = SysStats(proc=proc, sys=sys, gpu_paths=(), nvidia_smi=None)
    st.sample()
    write(f'{proc}/stat', fixture('proc_stat_2'))
    view = st.sample()
    cores = {c['id']: c for c in view['cores']}
    assert len(cores) == 12
    assert cores[1] == {'id': 1, 'online': True, 'percent': 20.2}
    assert cores[10] == {'id': 10, 'online': False, 'percent': None}
    assert view['cpu_percent'] == 5.7
    assert view['gpu'] == {'percent': None, 'freq_mhz': None, 'max_mhz': None, 'source': None}


def test_gpu_load_file_is_per_mille_and_the_first_readable_path_wins(tmp_path):
    assert parse_gpu_load(fixture('gpu_load_idle')) == 0.0
    assert parse_gpu_load('347\n') == 34.7
    assert parse_gpu_load('1000') == 100.0
    sys = str(tmp_path / 'sys')
    load = f'{sys}/devices/platform/bus@0/17000000.gpu/load'
    later = f'{sys}/devices/platform/17000000.gpu/load'
    write(load, '347\n')
    write(later, '100\n')
    write(f'{sys}/class/devfreq/15340000.vic/cur_freq', '729600000\n')
    write(f'{sys}/class/devfreq/17000000.gpu/cur_freq', '306000000\n')
    write(f'{sys}/class/devfreq/17000000.gpu/max_freq', '816000000\n')
    st = SysStats(proc=str(tmp_path / 'proc'), sys=sys,
                  gpu_paths=(f'{sys}/devices/gpu.0/load', load, later), nvidia_smi=None)
    assert st.sample()['gpu'] == {'percent': 34.7, 'freq_mhz': 306, 'max_mhz': 816,
                                  'source': load}
    # nvidia-smi (linus): '[N/A]' is n/a, not an error.
    assert parse_nvidia_smi('6, 1050, 3090\n') == {'percent': 6.0, 'freq_mhz': 1050,
                                                   'max_mhz': 3090}
    assert parse_nvidia_smi('[N/A], 1050, [N/A]\n')['percent'] is None


def test_pinned_scan_skips_a_process_that_vanishes_or_exits_mid_read(tmp_path, monkeypatch):
    proc = str(tmp_path / 'proc')
    write(f'{proc}/stat', fixture('proc_stat_1'))      # cpu0..cpu11: all 12 online
    online = list(range(12))
    add_proc(proc, 4100, '10-11', b'/usr/bin/python3\0/ws/lib/mpc/mpc_corr\0--ros-args\0'
             b'-r\0__node:=mpc_corr\0', ticks=(300, 100))
    add_proc(proc, 4200, '0-11', b'/usr/bin/bash\0')                  # unpinned
    add_proc(proc, 15, '3', ppid=2)                                     # ksoftirqd/3
    add_proc(proc, 4300, '8-9', b'/opt/ros/humble/lib/x/yolo\0')
    os.remove(f'{proc}/4300/stat')       # gone after its status was read
    add_proc(proc, 4400, '6-7', b'/usr/bin/python3\0/ws/lib/p/obstacle_projector_node\0')
    shutil.rmtree(f'{proc}/4400')        # gone before anything was read
    add_proc(proc, 4500, '8-9', b'', state='Z (zombie)')
    add_proc(proc, 4510, '0-11', b'', state='Z (zombie)')              # not unpinned either
    add_proc(proc, 4600, '8-9', b'')     # exiting: still R, its command line already gone
    pinned, unpinned, kthreads = scan_procs(proc, online)
    assert [(p['pid'], p['name'], p['cores'], p['ticks']) for p in pinned] == \
        [(4100, 'mpc_corr', [10, 11], 400)]
    assert (unpinned, kthreads) == (1, 1)

    # Its own CPU %: ticks over the scan's period, 100 = one core.
    t = [0.0]
    st = SysStats(proc=proc, sys=str(tmp_path / 'sys'), gpu_paths=(), nvidia_smi=None,
                  clock=lambda: t[0])
    monkeypatch.setattr(sysstats, 'CLK_TCK', 100)
    assert st.sample()['pinned'][0]['cpu_percent'] is None
    add_proc(proc, 4100, '10-11', b'/usr/bin/python3\0x\0__node:=mpc_corr\0', ticks=(500, 150))
    t[0] = 5.0
    view = st.sample()
    assert view['pinned'] == [{'pid': 4100, 'name': 'mpc_corr', 'cores': [10, 11],
                               'cores_text': '10-11', 'cpu_percent': 50.0}]


def test_friendly_name_prefers_the_ros_node_then_the_launch_then_the_program():
    assert friendly_name(['/opt/ros/humble/lib/robot_localization/ekf_node', '--ros-args',
                          '-r', '__node:=ekf_global_filter_node'], 'ekf_node') == \
        'ekf_global_filter_node'
    assert friendly_name(['/usr/bin/python3', '/opt/ros/humble/bin/ros2', 'launch',
                          'f1tenth_bringup', 'stack_bringup.launch.py'], 'ros2') == \
        'stack_bringup.launch.py'
    assert friendly_name(['/usr/bin/python3', '-u', '/ws/lib/f1tenth_behavior/'
                          'behavior_executor_node'], 'behavior_execu') == 'behavior_executor_node'
    assert friendly_name(['python3', '-m', 'f1tenth_gateway'], 'python3') == 'f1tenth_gateway'
    assert friendly_name(['/opt/ros/humble/lib/foxglove_bridge/foxglove_bridge'], 'foxglove') \
        == 'foxglove_bridge'
    assert friendly_name([], 'kworker/3:1') == 'kworker/3:1'
    # An interpreter option's value is not the script; a command string is not a name.
    assert friendly_name(['/usr/bin/python3', '-W', 'ignore', '-X', 'faulthandler',
                          '/ws/lib/p/relay.py'], 'python3') == 'relay.py'
    assert friendly_name(['/bin/sh', '-c', '/opt/ros/humble/lib/pkg/exe --ros-args -p a:=1'],
                         'sh') == 'exe'
    assert friendly_name(['/bin/bash', '-lc', 'exec ros2 bag record -a'], 'bash') == 'ros2'
    assert friendly_name(['python3', '-c', 'import time; time.sleep(9)'], 'python3') == 'python3'


class Wedged:
    """An nvidia-smi stuck in the driver (D state): it times out and outlives its SIGKILL."""

    calls = 0

    def __init__(self, *args, **kwargs):
        Wedged.calls += 1
        self.stdout = io.StringIO()

    def communicate(self, timeout=None):
        raise subprocess.TimeoutExpired('nvidia-smi', timeout)

    def kill(self):
        pass

    def wait(self, timeout=None):
        raise subprocess.TimeoutExpired('nvidia-smi', timeout)


def test_nvidia_smi_is_read_and_a_hung_one_never_blocks_the_sample(tmp_path, monkeypatch):
    smi = tmp_path / 'nvidia-smi'
    write(str(smi), '#!/bin/sh\necho "6, 1050, 3090"\n')
    smi.chmod(0o755)
    t = [0.0]
    st = SysStats(proc=str(tmp_path / 'proc'), sys=str(tmp_path / 'sys'), gpu_paths=(),
                  nvidia_smi=str(smi), clock=lambda: t[0])
    assert st.sample()['gpu'] == {'percent': 6.0, 'freq_mhz': 1050, 'max_mhz': 3090,
                                  'source': 'nvidia-smi'}
    # Slow: killed at the timeout, n/a, tried again after NVIDIA_SMI_RETRY_S.
    write(str(smi), '#!/bin/sh\nexec sleep 30\n')
    monkeypatch.setattr(sysstats, 'NVIDIA_SMI_TIMEOUT_S', 0.2)
    t0 = time.monotonic()
    assert st.sample()['gpu']['percent'] is None
    assert time.monotonic() - t0 < 5.0
    assert st.smi == str(smi)
    # Wedged in the kernel: left behind once, then no more nvidia-smi.
    monkeypatch.setattr(Wedged, 'calls', 0)
    monkeypatch.setattr(sysstats.subprocess, 'Popen', Wedged)
    t[0] = sysstats.NVIDIA_SMI_RETRY_S + 1
    assert st.sample()['gpu']['percent'] is None
    t[0] += sysstats.NVIDIA_SMI_RETRY_S + 1
    assert st.sample()['gpu']['percent'] is None
    assert (Wedged.calls, st.smi) == (1, None)
