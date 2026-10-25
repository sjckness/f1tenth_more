"""
Unit tests for the pure helpers of scripts/jazzy_parity (no ROS graph needed).

Run: python3 -m pytest scripts/jazzy_parity/test docker/orin/checks/test
"""
import json
import os
import subprocess
import sys

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))

import compare_snapshots  # noqa: E402, I100
import mpc_frozen_io  # noqa: E402
import parity_gates  # noqa: E402
import phase5_components  # noqa: E402
import phase5_summary  # noqa: E402
import proc_audit  # noqa: E402
import run_context  # noqa: E402
import sys_sample  # noqa: E402


# ---- proc_audit -------------------------------------------------------------

def p(pid, ppid, args, pgid=None, et=10):
    return {'pid': pid, 'pgid': pgid if pgid is not None else pid, 'ppid': ppid,
            'etimes': et, 'args': args}


LAUNCH = ('/usr/bin/python3 /opt/ros/jazzy/bin/ros2 launch f1tenth_bringup '
          'supervisor_bringup.launch.py')
NODE = ('/usr/bin/python3 /ws/install/mpc_controller/lib/mpc_controller/mpc_corr --ros-args '
        '-r __node:=mpc_corr --params-file /tmp/launch_params_ab12cd')
DS = '/opt/ros/jazzy/bin/fast-discovery-server --server-id 0 --udp-port 11811'


def test_format_parse_round_trip():
    procs = [p(10, 1, LAUNCH), p(11, 10, NODE, pgid=10)]
    lines = [proc_audit.format_line(x) + '\n' for x in procs]
    assert proc_audit.parse_lines(lines) == procs


def test_parse_skips_malformed_lines():
    lines = ['12 12 1 5 ok args\n', 'p continuation of a multi-line argument\n', '\n']
    assert [x['pid'] for x in proc_audit.parse_lines(lines)] == [12]


def test_node_name_and_namespace():
    assert proc_audit.node_name(NODE) == '/mpc_corr'
    assert proc_audit.node_name('x -r __node:=ekf_global_filter_node -r __ns:=/ekf_global') \
        == '/ekf_global/ekf_global_filter_node'
    assert proc_audit.node_name(LAUNCH) is None


def test_is_stack_excludes_harness_and_build_tools(monkeypatch):
    monkeypatch.delenv('PARITY_IGNORE', raising=False)
    assert proc_audit.is_stack(LAUNCH)
    assert proc_audit.is_stack(NODE)
    assert proc_audit.is_stack(DS)
    assert not proc_audit.is_stack('python3 /x/scripts/jazzy_parity/stack_probe.py --out a nodes')
    assert not proc_audit.is_stack('/home/u/.rustup/bin/rustc --crate-name x -L /ws/install/lib')
    assert not proc_audit.is_stack('/usr/bin/cmake --build /ws/build/pkg -- -I/ws/install/include')
    monkeypatch.setenv('PARITY_IGNORE', 'f1tenth_more_r0b')
    assert not proc_audit.is_stack('/home/u/f1tenth_more_r0b/install/x/lib/x/node')


def test_audit_clean_tree(monkeypatch):
    monkeypatch.delenv('PARITY_IGNORE', raising=False)
    procs = [p(10, 1, LAUNCH), p(11, 10, NODE, pgid=10), p(12, 10, DS, pgid=10)]
    res = proc_audit.audit(procs, launch_pid=10)
    assert res['clean'] and res['n_named_nodes'] == 1
    assert len(res['discovery_server_processes']) == 1


def test_audit_finds_duplicates_and_orphans(monkeypatch):
    monkeypatch.delenv('PARITY_IGNORE', raising=False)
    other = NODE.replace('ab12cd', 'zz99yy')
    procs = [p(10, 1, LAUNCH), p(11, 10, NODE, pgid=10), p(20, 1, other)]
    res = proc_audit.audit(procs, launch_pid=10)
    assert res['duplicate_node_names'] == {'/mpc_corr': [11, 20]}
    assert len(res['orphans']) == 1 and res['orphans'][0].startswith('20 ')
    assert not res['clean']


def test_audit_identical_command_lines_are_duplicates(monkeypatch):
    monkeypatch.delenv('PARITY_IGNORE', raising=False)
    cmd = 'ros2 launch f1tenth_navigation navigation.launch.py'
    res = proc_audit.audit([p(10, 1, LAUNCH), p(11, 10, cmd), p(12, 10, cmd)], launch_pid=10)
    assert list(res['duplicate_command_lines'].values()) == [[11, 12]]


def test_audit_after_shutdown(monkeypatch):
    monkeypatch.delenv('PARITY_IGNORE', raising=False)
    # A Discovery Server reparented to init is reported, not an orphan or a leftover.
    res = proc_audit.audit([p(30, 1, DS)], expect_none=True)
    assert res['clean'] and res['leftovers'] == [] and res['orphans'] == []
    res = proc_audit.audit([p(30, 1, DS), p(31, 1, NODE)], expect_none=True)
    assert not res['clean'] and len(res['leftovers']) == 1


def test_sweep_plan(monkeypatch):
    monkeypatch.delenv('PARITY_IGNORE', raising=False)
    procs = [p(10, 1, LAUNCH), p(11, 10, NODE, pgid=10), p(40, 1, 'bash -c editor', pgid=40)]
    plan = proc_audit.sweep_plan(procs, [10, 40, 99])
    assert plan[10] == 'kill' and plan[99] == 'gone' and plan[40].startswith('foreign')


# ---- sys_sample -------------------------------------------------------------

def test_proc_stat_and_busy():
    a = sys_sample.parse_proc_stat('cpu  100 0 100 800 0 0 0 0 0 0\ncpu0 50 0 50 400 0 0 0 0\n')
    b = sys_sample.parse_proc_stat('cpu  150 0 150 850 50 0 0 0 0 0\ncpu0 100 0 50 450 0 0 0 0\n')
    assert sys_sample.busy_pct(a['cpu'], b['cpu']) == 50.0
    assert sys_sample.busy_pct(a['cpu0'], b['cpu0']) == 50.0
    assert sys_sample.busy_pct(a['cpu'], a['cpu']) is None


def test_meminfo_and_proc_ticks():
    m = sys_sample.parse_meminfo('MemTotal:  1000 kB\nMemAvailable:   400 kB\n'
                                 'HugePages_Total: 0\n')
    assert m == {'MemTotal': 1000, 'MemAvailable': 400, 'HugePages_Total': 0}
    stat = '123 (my (odd) name) S 1 123 123 0 -1 4194560 1 0 0 0 7 5 0 0 20 0 1 0 999 1 1'
    assert sys_sample.proc_ticks(stat) == 12


def test_label_of():
    assert sys_sample.label_of(NODE) == '/mpc_corr'
    assert sys_sample.label_of(LAUNCH) == 'supervisor_bringup.launch.py'


# ---- run_context ------------------------------------------------------------

def test_parse_sources():
    text = ('commit abc\nsubject s here\nsubmodule src/x 1234 https://u\n'
            'tree_sha256 ff\n')
    res = run_context.parse_sources(text)
    assert res['commit'] == 'abc' and res['submodules'] == {'src/x': '1234'}
    assert res['tree_sha256'] == 'ff' and run_context.parse_sources(None) is None


def test_clocks_from_a_fake_sysfs(tmp_path):
    for i, (lo, hi) in enumerate(((100, 200), (200, 200))):
        d = tmp_path / ('cpu%d' % i) / 'cpufreq'
        d.mkdir(parents=True)
        (d / 'scaling_min_freq').write_text('%d\n' % lo)
        (d / 'scaling_max_freq').write_text('%d\n' % hi)
        (d / 'scaling_cur_freq').write_text('%d\n' % hi)
        (d / 'scaling_governor').write_text('schedutil\n')
    cpus = run_context.cpu_clocks(str(tmp_path))
    assert cpus['cpu0']['min_khz'] == 100 and cpus['cpu1']['governor'] == 'schedutil'
    assert run_context.clocks_pinned(cpus, {}) is False
    cpus['cpu0']['min_khz'] = 200
    assert run_context.clocks_pinned(cpus, {}) is True
    assert run_context.clocks_pinned({}, {}) is None


# ---- phase5_components ------------------------------------------------------

def registry():
    return {'components': {
        'hardware': [{'package': 'h', 'launch_file': 'vesc.launch.py'}],
        'perception': [{'package': 'p', 'launch_file': 'camera.launch.py'},
                       {'package': 'p', 'launch_file': 'detection.launch.py'}],
        'diagnostics': [{'package': 'l', 'launch_file': 'mission_logger.launch.py'}]}}


def test_components_derive():
    reg = registry()
    changes = phase5_components.derive(reg, ['hardware'], ['detection.launch.py'], '/runs')
    comps = reg['components']
    assert comps['hardware'] == []
    assert [e['launch_file'] for e in comps['perception']] == ['camera.launch.py']
    assert comps['diagnostics'][0]['args'] == {'mission_logger_runs_dir': '/runs'}
    assert len(changes) == 3


def test_components_unknown_and_health():
    with pytest.raises(SystemExit):
        phase5_components.derive(registry(), ['nope'])
    reg = registry()
    reg['health'] = {'perception': {'checks': [{'launch_file': 'detection.launch.py'}]}}
    phase5_components.derive(reg, [], ['detection.launch.py'])
    assert 'perception' not in reg['health'] and 'perception' in reg['unwatched']


# ---- phase5_summary + parity_gates smoke -----------------------------------

def make_run(tmp_path, respawns=0, leftover='', lock='', audit_clean=True, mission=True):
    run = tmp_path / 'run'
    (run / 'supervisor').mkdir(parents=True)
    (run / 'nodes_startup.json').write_text(json.dumps(
        {'final': ['/a', '/b'], 'seen': {'/a': {'first': 1.0}, '/b': {'first': 2.5}}}))
    log = '[component_supervisor] Shutting down -- stopping all components.\n'
    log += ("x 'navigation' (navigation.launch.py) crashed (exit -9) -- "
            'auto-respawning.\n') * respawns
    (run / 'launch.log').write_text(log)
    (run / 'shutdown.txt').write_text('launch exit code: 0\nshutdown duration: 3.20 s\n')
    (run / 'leftover_procs.txt').write_text(leftover)
    (run / 'leftover_locks.txt').write_text(lock)
    (run / 'discovery_errors.txt').write_text(
        'Matching unexisting participant: 0\nDISCOVERY_DATABASE (any): 0\n'
        '[RTPS_ / [DISCOVERY / [PARTICIPANT errors (any): 0\n')
    (run / 'audit_settled.json').write_text(json.dumps(
        {'clean': audit_clean, 'duplicate_node_names': {}, 'duplicate_command_lines': {},
         'orphans': [] if audit_clean else ['x']}))
    if mission:
        (run / 'mission.json').write_text(json.dumps(
            {'load': {'success': True}, 'start': {'success': True},
             'states': [{'state': 'RUNNING'}, {'state': 'COMPLETE'}]}))
        d = run / 'runs' / 'complete' / 'r1' / 'bag'
        d.mkdir(parents=True)
        (d / 'metadata.yaml').write_text(
            'rosbag2_bagfile_information:\n  message_count: 42\n')
    return run


def test_summary_and_smoke_pass(tmp_path):
    run = make_run(tmp_path)
    s = phase5_summary.summarise(str(run))
    assert s['nodes'] == 2 and s['startup_total_s'] == 2.5 and s['shutdown_path'] == 'graceful'
    assert s['logger_runs'][0]['bag_messages'] == 42
    assert parity_gates.smoke_problems(s, 0) == []


def test_smoke_failures(tmp_path):
    run = make_run(tmp_path, respawns=2, leftover='5 5 1 3 /ws/install/x/lib/x/node\n',
                   lock='/tmp/mission_logger.lock 77 alive\n', audit_clean=False)
    s = phase5_summary.summarise(str(run))
    bad = ' | '.join(parity_gates.smoke_problems(s, 0))
    for word in ('audit', 'left after shutdown', 'live process', 'respawns 2'):
        assert word in bad


def test_smoke_dead_lock_is_not_gated(tmp_path):
    run = make_run(tmp_path, lock='/tmp/mission_logger.lock 77 dead\n')
    s = phase5_summary.summarise(str(run))
    assert s['leftover_locks'] == ['/tmp/mission_logger.lock'] and not s['leftover_locks_live']
    assert parity_gates.smoke_problems(s, 0) == []


def test_rate_problems():
    native = {'/scan': 40.0, '/tf': 100.0, '/drive': 10.0}
    container = {'/scan': 39.0, '/tf': 90.0}
    bad, rows = parity_gates.rate_problems(native, container, ('/scan', '/tf', '/drive'),
                                           0.05)
    assert any('/tf' in b for b in bad) and any('/drive' in b for b in bad)
    assert not any('/scan' in b for b in bad) and len(rows) == 3


def test_parse_golden():
    text = ('ex1  equal=True sha256=aa\nex2  equal=True sha256=bb\n'
            'plan_translate: /x\npython 3.12.3: 2/2 golden missions reproduced exactly\n')
    rows, summary = parity_gates.parse_golden(text)
    assert rows == {'ex1': (True, 'aa'), 'ex2': (True, 'bb')} and summary == (2, 2)


def test_lease_gate_band(tmp_path, capsys):
    lease = tmp_path / 'lease.json'
    lease.write_text(json.dumps({'removal_after_kill_s': [4.47, 4.5], 'profile': 'p'}))
    assert parity_gates.main(['lease', str(lease)]) == 0
    assert 'note' not in capsys.readouterr().out
    # the manager's linus runs with the profile: inside the gate, the plan's
    # 4.4-4.7 s band only noted
    lease.write_text(json.dumps({'removal_after_kill_s': [4.156, 4.428, 4.456, 5.010],
                                 'profile': 'p'}))
    assert parity_gates.main(['lease', str(lease)]) == 0
    assert "2 of 4 trials outside the plan's 4.4-4.7 s band" in capsys.readouterr().out
    lease.write_text(json.dumps({'removal_after_kill_s': [4.47, 19.4], 'profile': 'p'}))
    assert parity_gates.main(['lease', str(lease)]) == 1
    for bad in (3.7, 5.6, None):
        lease.write_text(json.dumps({'removal_after_kill_s': [4.5, bad], 'profile': 'p'}))
        assert parity_gates.main(['lease', str(lease)]) == 1
    lease.write_text(json.dumps({'removal_after_kill_s': [], 'profile': 'p'}))
    assert parity_gates.main(['lease', str(lease)]) == 1
    capsys.readouterr()


def test_feed_live_refuses_with_a_vesc_attached(tmp_path):
    # refused before any precheck or stack: nothing ROS starts here
    (tmp_path / 'ttyACM0').write_text('')
    env = dict(os.environ)
    env.pop('LIVE_VESC_OK', None)
    env.update({'FEED': 'live', 'LIVE_VESC_DEVS': str(tmp_path / 'ttyACM*')})
    r = subprocess.run(['bash', os.path.join(os.path.dirname(HERE), 'phase5_bringup.sh'),
                        'snapshot', str(tmp_path / 'out')], env=env, stdout=subprocess.PIPE,
                       stderr=subprocess.STDOUT, text=True, timeout=60)
    assert r.returncode == 2 and 'refusing FEED=live: %s' % (tmp_path / 'ttyACM0') in r.stdout


def test_archive_requests(tmp_path):
    assert phase5_summary.archive_requests(str(tmp_path)) == []
    (tmp_path / 'archive_trigger').mkdir()
    (tmp_path / 'archive_trigger' / 'request').write_text('{"run_id": "r1"}\n')
    (tmp_path / 'systemctl_calls.txt').write_text(
        '2026-10-25T10:00:00+02:00 --user --no-block start f1tenth-archive.service\n')
    got = phase5_summary.archive_requests(str(tmp_path))
    assert got == ['trigger {"run_id": "r1"}',
                   'systemctl 2026-10-25T10:00:00+02:00 --user --no-block start '
                   'f1tenth-archive.service']


def test_mpc_timing_gate_median_over_replays(tmp_path, capsys):
    def side(path, p95s, solve=None):
        res = {'r%d' % i: {'kind': 'wallrx', 'period_ms': {'n': 130, 'p95': v}}
               for i, v in enumerate(p95s)}
        if solve:
            res['cap'] = {'kind': 'capture', 'solve_ms': {'n': 130, 'p95': solve}}
        path.write_text(json.dumps(res))
        return str(path)
    tn = side(tmp_path / 'tn.json', [100.0, 102.0, 98.0], solve=12.0)
    tc = side(tmp_path / 'tc.json', [104.0, 103.0, 112.0], solve=13.0)
    # medians 104 vs 100: pass, although one container replay alone would fail
    assert parity_gates.main(['mpc-timing', tn, tc]) == 0
    out = capsys.readouterr().out
    assert 'ratio 1.040' in out and 'spread max/min 1.087' in out
    assert 'solve_ms median p95 (not gated): native 12.00 (1), container 13.00 (1)' in out
    tc = side(tmp_path / 'tc.json', [106.0, 106.0, 103.0])
    assert parity_gates.main(['mpc-timing', tn, tc]) == 1
    # fewer than 3 replays on a side fails; --min-runs / --label select
    tc = side(tmp_path / 'tc.json', [101.0, 101.0])
    assert parity_gates.main(['mpc-timing', tn, tc]) == 1
    assert 'container: 2 replay(s) carry period_ms, need >= 3' in capsys.readouterr().out
    assert parity_gates.main(['mpc-timing', tn, tc, '--min-runs', '2']) == 0
    assert parity_gates.main(['mpc-timing', tn, tc, '--min-runs', '1', '--label', 'r0']) == 0
    capsys.readouterr()


def test_gate_commands_on_files(tmp_path, capsys):
    tr = tmp_path / 'tracker.json'
    tr.write_text(json.dumps({'topic': 't', 'key': 'tracker_ms',
                              'stats': {'n': 5, 'p50': 8, 'p95': 12, 'max': 20}}))
    assert parity_gates.main(['tracker', str(tr)]) == 0
    zn, zc = tmp_path / 'zn.json', tmp_path / 'zc.json'
    zn.write_text(json.dumps({'rois': [{'name': 'r0c0', 'median_m': 2.0}]}))
    zc.write_text(json.dumps({'rois': [{'name': 'r0c0', 'median_m': 2.03}]}))
    assert parity_gates.main(['zed-depth', str(zn), str(zc)]) == 0
    zc.write_text(json.dumps({'rois': [{'name': 'r0c0', 'median_m': 2.1}]}))
    assert parity_gates.main(['zed-depth', str(zn), str(zc)]) == 1
    ra, rb, al = tmp_path / 'a.txt', tmp_path / 'b.txt', tmp_path / 'allow.txt'
    ra.write_text('failure p::x::t1\nskipped p::x::t2\n')
    rb.write_text('failure p::x::t1\nskipped p::x::t3\n')
    assert parity_gates.main(['tests', str(ra), str(rb)]) == 1
    al.write_text('# documented\nskipped p::x::t2\nskipped p::x::t3\n')
    assert parity_gates.main(['tests', str(ra), str(rb), '--allow', str(al)]) == 0
    capsys.readouterr()


# ---- mpc_frozen_io ----------------------------------------------------------

def test_frozen_io_round_trip(tmp_path):
    np = pytest.importorskip('numpy')

    class Opaque:
        def __repr__(self):
            return 'Opaque()'
    ticks = [{'wall_dt': 0.0123, 'sim_ns': 5, 'info': {'x': np.arange(3.0), 'o': Opaque(),
                                                       't': (1, 2.5), 'flag': True}}]
    f = str(tmp_path / 'c.npz')
    mpc_frozen_io.save_ticks(f, ticks, {'python': 'x'})
    got, meta, _ = mpc_frozen_io.load_ticks(f, ('wall_dt', 'sim_ns', 'info'))
    assert got[0]['wall_dt'] == 0.0123 and got[0]['sim_ns'] == 5
    assert list(got[0]['info']['x']) == [0.0, 1.0, 2.0] and got[0]['info']['t'] == (1, 2.5)
    assert got[0]['info']['o'] == 'Opaque: Opaque()' and meta == {'python': 'x'}


# ---- compare_snapshots ------------------------------------------------------

def test_compare_snapshots(tmp_path):
    for side, prefix, value in (('a', '/home/f/ws/install', 1),
                                ('b', '/opt/f1tenth_ws/install', 2)):
        d = tmp_path / side
        d.mkdir()
        (d / 'procs_settled.txt').write_text(
            '1 1 1 1 python3 %s/x/lib/x/node --params-file /tmp/launch_params_%s\n'
            % (prefix, 'q' * value))
        (d / 'params.json').write_text(json.dumps(
            {'params': {'/n': {'k': value, 'same': 1}, '/transform_listener_impl_ab': {'z': 1}}}))
    res = compare_snapshots.compare(str(tmp_path / 'a'), str(tmp_path / 'b'),
                                    '/home/f/ws/install', '/opt/f1tenth_ws/install')
    assert res['processes_only_in_a'] == [] and res['processes_only_in_b'] == []
    assert res['param_differences'] == {'/n': {'k': [1, 2]}}
    assert res['generated_name_nodes_param_sets_equal']
