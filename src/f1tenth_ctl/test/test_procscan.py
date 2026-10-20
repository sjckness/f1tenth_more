"""/proc matching, the signal ladder on real processes, UDP port owners."""

import os
import socket
import subprocess
import sys
import time

from f1tenth_ctl import procscan


def _p(argv, comm=None, uid=None, pid=1000, ppid=1):
    return procscan.Proc(pid, ppid, os.getuid() if uid is None else uid,
                         comm or os.path.basename(argv[0])[:15], argv)


def test_ros_processes_are_recognised():
    assert procscan.is_ros(_p(['/usr/bin/python3', '/opt/ros/humble/bin/ros2', 'launch',
                               'f1tenth_bringup', 'supervisor_bringup.launch.py']))
    assert procscan.is_ros(_p(['/home/u/ws/install/llm/lib/llm/llm_planner_node']))
    assert procscan.is_ros(_p(['/x/bin/node', '--ros-args', '-r', '__node:=n']))
    assert procscan.is_ros(_p(['/opt/ros/jazzy/lib/rclcpp_components/component_container']))
    assert procscan.is_ros(_p(['python3', '-c', 'x'], comm='_ros2_daemon'))


def test_other_processes_are_not():
    assert not procscan.is_ros(_p(['bash']))
    assert not procscan.is_ros(_p(['python3', '-m', 'http.server']))
    assert not procscan.is_ros(_p(['/scratch/llama.cpp/build/bin/llama-server', '-m', 'x']))


def test_discovery_server_is_told_apart():
    ds = _p(['/opt/ros/humble/bin/fast-discovery-server', '--server-id', '0'])
    sh = _p(['sh', '/opt/ros/humble/bin/fastdds', 'discovery', '--server-id', '0'])
    ens = _p(['python3', '/i/share/f1tenth_bringup/scripts/ensure_discovery_server.py',
              '127.0.0.1', '11811'])
    assert all(procscan.is_discovery(p) for p in (ds, sh, ens))
    procs = [ds, _p(['/opt/ros/humble/bin/ros2', 'run', 'x', 'y'], pid=1001)]
    assert [p.pid for p in procscan.ros_processes(procs, os.getuid(), set(), False)] == [1001]
    assert len(procscan.ros_processes(procs, os.getuid(), set(), True)) == 2


def test_other_users_and_excluded_pids_are_skipped():
    mine = _p(['/opt/ros/humble/bin/ros2', 'launch', 'a', 'b'], pid=2000)
    theirs = _p(['/opt/ros/humble/bin/ros2', 'launch', 'a', 'b'], pid=2001,
                uid=os.getuid() + 1)
    excluded = _p(['/opt/ros/humble/bin/ros2', 'launch', 'a', 'b'], pid=2002)
    out = procscan.ros_processes([mine, theirs, excluded], os.getuid(), {2002}, False)
    assert [p.pid for p in out] == [2000]


def test_own_exclusions_cover_this_process_and_its_parent():
    excl = procscan.own_exclusions()
    assert os.getpid() in excl and os.getppid() in excl


def test_read_proc_of_this_process():
    me = procscan.read_proc(os.getpid())
    assert me.uid == os.getuid() and me.ppid == os.getppid() and me.argv


_IGNORES_SIGINT = ('import signal, time; signal.signal(signal.SIGINT, signal.SIG_IGN); '
                   'print("up", flush=True); time.sleep(60)')


def _spawn(code):
    p = subprocess.Popen([sys.executable, '-c', code], stdout=subprocess.PIPE, text=True)
    p.stdout.readline()
    return p


def test_signal_ladder_sigint_is_enough_for_a_normal_process():
    p = _spawn('import time; print("up", flush=True); time.sleep(60)')
    res = procscan.signal_ladder([p.pid], 3.0, 3.0, is_alive=lambda pid: p.poll() is None)
    assert res['signals'] == ['SIGINT'] and res['survivors'] == []


def test_signal_ladder_escalates_past_an_ignored_sigint():
    p = _spawn(_IGNORES_SIGINT)
    t0 = time.monotonic()
    res = procscan.signal_ladder([p.pid], 0.5, 3.0, is_alive=lambda pid: p.poll() is None)
    assert res['signals'] == ['SIGINT', 'SIGTERM'] and res['survivors'] == []
    assert time.monotonic() - t0 < 3.0


def test_udp_owner_of_a_bound_port():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(('127.0.0.1', 0))
    port = sock.getsockname()[1]
    try:
        inodes = procscan.udp_inodes('127.0.0.1', port)
        assert inodes
        assert os.getpid() in procscan.pids_with_inodes(inodes)
    finally:
        sock.close()
    assert procscan.udp_inodes('127.0.0.1', port) == []
