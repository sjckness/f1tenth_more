"""
Tests of tools/bench/planner_to_start.py.

The pure parts (parsing, deltas, statistics, the safety refusal) need no ROS
and no stack:

    python3 -m pytest tools/bench/test_planner_to_start.py

test_live_oneshot_reaches_running is marked `stack`: it STARTS REAL MISSIONS
on the live stack. It is skipped unless F1TENTH_CAR_SAFE=1 is set (wheels
lifted / car on blocks, or motor output disabled), llama-server answers on
127.0.0.1:8083 and /mission/abort_mission is reachable. It runs inside the
stack's ROS environment (the car container, under ctl_container.env):

    F1TENTH_CAR_SAFE=1 F1TENTH_START_BUDGET_S=30 python3 -m pytest -m stack \\
        tools/bench/test_planner_to_start.py
"""

import importlib.util
import os
import socket
import sys
from pathlib import Path

import pytest

TOOL = Path(__file__).with_name('planner_to_start.py')
_spec = importlib.util.spec_from_file_location('planner_to_start', TOOL)
bench = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(bench)

# Real lines, as the planner prints them (ROS logger prefix included).
LATENZA_LINE = ('[INFO] [1794212863.18] [llm_planner_node]: latenza: llm=0.51s '
                'traduzione=0.02s servizi=0.26s totale=0.79s')
RESULT_LINE = ('comando> comando> RESULT {"status": "ok", "stage": "translator", '
               '"attempts": 1, "mission_id": "llm_2b356aac445f", "latency": {"llm_s": 0.509, '
               '"translate_s": 0.019, "services_s": 0.259, "total_s": 0.789}}')


class TestParsing:

    def test_latenza_line(self):
        assert bench.parse_latenza(LATENZA_LINE) == {
            'llm_s': 0.51, 'translate_s': 0.02, 'services_s': 0.26, 'total_s': 0.79}

    def test_not_a_latenza_line(self):
        assert bench.parse_latenza('[INFO] [1.0] [llm_planner_node]: start_mission OK') is None

    def test_result_line_with_glued_prompts_and_latency(self):
        res = bench.parse_result(RESULT_LINE)
        assert res['status'] == 'ok' and res['attempts'] == 1
        assert res['latency'] == {'llm_s': 0.509, 'translate_s': 0.019,
                                  'services_s': 0.259, 'total_s': 0.789}

    def test_unparsable_result_line(self):
        assert bench.parse_result('RESULT {not json') == {'status': 'unparsed'}
        assert bench.parse_result('missione generata in 0.53s (v2), 1 move:') is None

    @pytest.mark.parametrize('line, event', [
        # b6c0062 (the image today) and 18202006 spell the ready line differently.
        ('[INFO] [1.0] [llm_planner_node]: Server raggiunto e scaldato in 1.5s.', 'server_ready'),
        ('[INFO] [1.0] [llm_planner_node]: Server raggiunto, modello e prefisso del prompt '
         'scaldati in 0.2s.', 'server_ready'),
        ('[INFO] [1.0] [llm_planner_node]: LLM Planner (llama-server @ '
         'http://127.0.0.1:8083/completion) pronto -- consegna via /mission/load_mission',
         'ready'),
        ('missione generata in 0.53s (v2), 1 move:', 'planned'),
        ("[INFO] [1.0] [llm_planner_node]: load_mission OK in 0.00s: loaded 'llm_x', 1 moves",
         'loaded_log'),
        ("[INFO] [1.0] [llm_planner_node]: start_mission OK: started 'llm_x'", 'started_log'),
        ('[ERROR] [1.0] [llm_planner_node]: start_mission FALLITO: no', 'delivery_failed'),
        ('[INFO] [1.0] [llm_planner_node]: Scrivi un comando e premi invio (Ctrl-D per uscire).',
         'interactive_ready'),
        (LATENZA_LINE, 'latenza'),
        (RESULT_LINE, 'result'),
    ])
    def test_classify_line(self, line, event):
        assert bench.classify_line(line)[0] == event

    def test_classify_ignores_other_lines(self):
        assert bench.classify_line('{') is None
        assert bench.classify_line('  "mission_id": "llm_x",') is None


class TestDeltas:

    T = {'t_process_up': 1.2, 't_server_ready': 1.9, 't_ready': 2.0, 't_planned': 2.6,
         't_start_returned': 2.9, 't_running': 2.85}
    LAT = {'llm_s': 0.5, 'translate_s': 0.03, 'services_s': 0.25, 'total_s': 0.8}

    def test_oneshot_with_the_planner_numbers(self):
        d = bench.compute_deltas(self.T, self.LAT, 'oneshot')
        assert d['process_start'] == 2.0
        assert d['llm'] == 0.5 and d['translate'] == 0.03
        assert d['services'] == pytest.approx(0.3)
        # RUNNING can be published before the planner prints start_mission OK.
        assert d['start_to_running'] == pytest.approx(-0.05)
        assert d['total_to_start'] == 2.9 and d['total_to_running'] == 2.85

    def test_oneshot_without_latenza_falls_back_to_the_planned_line(self):
        d = bench.compute_deltas(self.T, None, 'oneshot')
        assert d['llm'] == pytest.approx(0.6)      # LLM + translate, seen from outside
        assert d['translate'] is None

    def test_process_start_falls_back_to_server_ready_then_first_line(self):
        t = dict(self.T)
        del t['t_ready']
        assert bench.compute_deltas(t, self.LAT, 'oneshot')['process_start'] == 1.9
        del t['t_server_ready']
        assert bench.compute_deltas(t, self.LAT, 'oneshot')['process_start'] == 1.2

    def test_persistent_has_no_per_run_process_start(self):
        t = dict(self.T, t_ready=0.0)
        d = bench.compute_deltas(t, self.LAT, 'persistent')
        assert d['process_start'] is None
        assert d['total_to_running'] == 2.85

    def test_missing_events_give_none(self):
        d = bench.compute_deltas({'t_ready': 2.0}, None, 'oneshot')
        assert d['services'] is None and d['start_to_running'] is None
        assert d['total_to_running'] is None


class TestStats:

    def test_percentile_nearest_rank(self):
        xs = list(range(1, 21))                  # 1..20
        assert bench.percentile(xs, 95) == 19
        assert bench.percentile(xs, 50) == 10
        assert bench.percentile(xs, 100) == 20
        assert bench.percentile([3.0], 95) == 3.0
        assert bench.percentile([], 95) is None

    def test_stats_ignores_none(self):
        expected = {'n': 3, 'median': 2.0, 'p95': 3.0, 'max': 3.0}
        assert bench.stats([1.0, None, 3.0, 2.0]) == expected
        assert bench.stats([None]) is None


def _run(cmd, i, total, attempts=1, status='ok', **extra):
    res = {'status': status, 'attempts': attempts}
    res.update(extra)
    return {'command': cmd, 'run': i, 'result': res, 'error': None,
            't': {'t_running': total},
            'deltas': {'process_start': 2.0, 'llm': 0.5, 'translate': 0.03, 'services': 0.3,
                       'start_to_running': 0.0, 'total_to_start': total,
                       'total_to_running': total}}


class TestSummary:

    def test_unrepresentative_runs_are_excluded(self):
        runs = [_run('a', 0, 3.0), _run('a', 1, 3.2),
                _run('a', 2, 9.0, attempts=2),
                _run('b', 0, 4.0, status='refused'),
                _run('b', 1, 4.0, unsupported=['vai da Marco']),
                _run('b', 2, 4.0, ambiguous=True)]
        never = _run('b', 3, None)
        never['t'] = {}
        runs.append(never)
        s = bench.summarize(runs)
        assert s['kept'] == 2
        assert s['overall']['total_to_running']['median'] == pytest.approx(3.1)
        why = {(e['command'], e['run']): e['why'] for e in s['excluded']}
        assert why[('a', 2)] == 'attempts 2'
        assert why[('b', 0)] == 'status refused'
        assert why[('b', 1)].startswith('unsupported')
        assert why[('b', 2)] == 'ambiguous'
        assert why[('b', 3)] == 'never RUNNING'

    def test_markdown_bolds_the_largest_stage(self):
        table = bench.markdown_table(bench.summarize([_run('a', 0, 3.0), _run('a', 1, 3.0)]))
        row = next(ln for ln in table.splitlines() if ln.startswith('| a |'))
        cells = row.split('|')[2:-1]
        bold = [c.strip() for c in cells if c.strip().startswith('**')]
        assert bold == ['**2.000 / 2.000 / 2.000**']          # process_start, 2.0 s
        assert cells[0].strip() == bold[0]


class TestSafety:

    def test_refuses_without_the_flag_and_calls_nothing(self, monkeypatch, capsys):
        called = []
        monkeypatch.setattr(bench, 'Stack', lambda *a, **k: called.append('Stack'))
        monkeypatch.setattr(bench, 'Planner', lambda *a, **k: called.append('Planner'))
        rc = bench.main(['--suite', '--runs', '1'],
                        session_fn=lambda args: called.append('session'))
        assert rc != 0
        assert called == []
        assert 'REFUSED' in capsys.readouterr().err

    def test_refusal_text(self):
        assert bench.check_safety(False) == bench.SAFETY_REFUSAL
        assert bench.check_safety(True) is None

    def test_with_the_flag_the_session_runs(self):
        seen = []

        def fake_session(args):
            seen.append(args)
            return {'summary': bench.summarize([_run('a', 0, 3.0)]), 'runs': [_run('a', 0, 3.0)],
                    'complete': True}

        assert bench.main(['--car-is-safe', '--cmd', 'a', '--runs', '1', '--budget-s', '5'],
                          session_fn=fake_session) == 0
        assert seen[0].car_is_safe

    def test_budget_exceeded_exits_1(self):
        def fake_session(args):
            return {'summary': bench.summarize([_run('a', 0, 9.0)]), 'runs': [_run('a', 0, 9.0)],
                    'complete': True}

        assert bench.main(['--car-is-safe', '--cmd', 'a', '--budget-s', '5'],
                          session_fn=fake_session) == 1

    def test_unconfirmed_abort_exits_non_zero(self):
        def fake_session(args):
            return {'summary': bench.summarize([]), 'runs': [], 'complete': False}

        assert bench.main(['--car-is-safe', '--cmd', 'a'], session_fn=fake_session) == 3


class TestInvocation:

    def test_environment_is_f1tenth_ctls_base(self):
        env = bench.planner_environment('/x/ctl.env', base={
            'HOME': '/home/u', 'USER': 'u', 'ROS_DOMAIN_ID': '42', 'PYTHONPATH': '/junk'})
        assert env['F1T_CTL_ENV'] == '/x/ctl.env'
        assert env['PATH'] == '/home/u/.local/bin:' + bench.BASE_PATH
        assert env['PYTHONUNBUFFERED'] == '1' and env['RCUTILS_LOGGING_BUFFERED_STREAM'] == '0'
        assert 'ROS_DOMAIN_ID' not in env and 'PYTHONPATH' not in env

    def test_argv_is_the_production_executable_without_dry_run(self):
        one = bench.planner_argv('oneshot', 'vai dalla persona')
        assert one[-5:] == ['ros2', 'run', 'llm', 'llm_planner_node', 'vai dalla persona']
        assert '--dry-run' not in one
        assert '. "$F1T_CTL_ENV"' in one[2]
        per = bench.planner_argv('persistent', 'ignored')
        assert per[-4:] == ['ros2', 'run', 'llm', 'llm_planner_node']

    def test_pythonpath_overlay_goes_first(self):
        argv = bench.planner_argv('oneshot', 'x', pythonpath='/tmp/llm_src')
        assert 'PYTHONPATH="/tmp/llm_src:$PYTHONPATH"' in argv[2]

    def test_default_suite_comes_from_the_live_evals(self):
        evals = (Path(__file__).resolve().parents[2] / 'src' / 'f1tenth_intelligence' / 'llm'
                 / 'test' / 'test_intent_evals_live.py').read_text()
        for cmd in bench.DEFAULT_SUITE:
            assert repr(cmd) in evals or f'"{cmd}"' in evals, cmd


class TestProductionFlow:
    """--flow production: the pure parts (no ctl, no ROS)."""

    T = {'t_cmd': 0.0, 't_send_ret': 0.9, 't_plan_done': 12.0, 't_plan_fetched': 12.8,
         't_load_call': 12.8, 't_loaded': 14.1, 't_load_ret': 14.6, 't_start_call': 14.6,
         't_running': 16.0, 't_start_ret': 16.9, 't_first_move': 16.3}

    def test_deltas(self):
        d = bench.compute_prod_deltas(self.T)
        assert d['send_cli'] == 0.9
        assert d['llm_and_ctl'] == 12.0
        assert d['fetch_cli'] == pytest.approx(0.8)
        assert d['pause'] == pytest.approx(0.0)
        assert d['load_cli'] == pytest.approx(1.3)
        assert d['start_cli'] == pytest.approx(1.4)
        assert d['running_to_move'] == pytest.approx(0.3)
        assert d['total_to_running'] == 16.0
        assert d['total_to_move'] == 16.3
        assert d['total_to_move_no_pause'] == pytest.approx(16.3)

    def test_pauses_are_summed_and_taken_out(self):
        t = dict(self.T, t_load_call=15.8, t_loaded=17.1, t_load_ret=17.6, t_start_call=20.6,
                 t_running=22.0, t_first_move=22.3)
        d = bench.compute_prod_deltas(t)
        assert d['pause'] == pytest.approx(6.0)
        assert d['total_to_move_no_pause'] == pytest.approx(16.3)

    def test_missing_stages_give_none(self):
        d = bench.compute_prod_deltas({'t_cmd': 0.0, 't_send_ret': 0.9, 't_plan_done': 2.0})
        assert d['fetch_cli'] is None and d['pause'] is None
        assert d['total_to_move'] is None and d['total_to_move_no_pause'] is None

    def test_refused_start_is_timed_as_ready_cli(self):
        t = {k: v for k, v in self.T.items() if k not in ('t_running', 't_first_move')}
        t['t_start_ret'] = 15.4
        d = bench.compute_prod_deltas(t)
        assert d['ready_cli'] == pytest.approx(0.8)          # the CLI up to its refusal
        assert d['start_cli'] is None and d['total_to_move'] is None

    def test_totals_for_the_plan_and_for_a_refused_start(self):
        t = {k: v for k, v in self.T.items() if k not in ('t_running', 't_first_move')}
        t['t_start_ret'] = 15.4
        d = bench.compute_prod_deltas(t)
        assert d['cmd_to_plan'] == 12.8 and d['total_to_refusal'] == pytest.approx(15.4)
        assert bench.compute_prod_deltas(self.T)['total_to_refusal'] is None

    def test_plan_poll_flag(self):
        assert bench.build_parser().parse_args(['--car-is-safe', '--plan-poll', '2']
                                               ).plan_poll == 2.0

    def test_plan_wait_flag(self):
        args = bench.build_parser().parse_args(['--car-is-safe', '--plan-wait', '8'])
        assert args.plan_wait == 8.0
        assert bench.build_parser().parse_args(['--car-is-safe']).plan_wait == 0.0

    def test_ready_cli_only_without_running(self):
        assert bench.compute_prod_deltas(self.T)['ready_cli'] is None

    def test_a_run_ended_at_the_ready_refusal_is_representative(self):
        run = {'command': 'a', 'run': 0, 'result': {'status': 'ok', 'attempts': 1},
               'error': None, 't': {}, 'end': 'ready_refused: not READY: battery'}
        assert bench.representative(run) == (True, '')

    def test_not_ready_start_has_no_bypass(self):
        action = next(a for a in bench.build_parser()._actions if a.dest == 'not_ready_start')
        assert set(action.choices) == {'fail', 'stop'}

    @pytest.mark.parametrize('mode, rule', [
        ('straight', 'speed'), ('go_to_object', 'speed'), ('turn', 'steering'),
        ('wall_turn', 'steering')])
    def test_first_move_rule(self, mode, rule):
        assert bench.first_move_rule({'moves': [{'drive': {'mode': mode}}]}) == rule

    def test_first_move_rule_defaults_to_speed(self):
        assert bench.first_move_rule(None) == 'speed'
        assert bench.first_move_rule({'moves': []}) == 'speed'

    def test_is_move(self):
        assert bench.is_move(0.4, 0.0, 'speed')
        assert not bench.is_move(0.0, 0.3, 'speed')
        assert bench.is_move(0.0, -0.3, 'steering')
        assert not bench.is_move(0.0005, 0.0, 'speed')

    def test_ctl_answer_is_the_last_json_line(self):
        out = 'noise\n{"ok": false}\n{"ok": true, "cursor": 42}\n'
        assert bench.parse_ctl_json(out) == {'ok': True, 'cursor': 42}
        assert bench.parse_ctl_json('no json') is None

    def test_stored_mission_from_planner_log(self):
        answer = {'results': [
            {'offset': 10, 'status': 'refused'},
            {'offset': 90, 'status': 'ok', 'stored': {'id': '20261109T101010-abc123'},
             'mission': {'mission_id': 'llm_x', 'moves': []}}]}
        sid, mission = bench.stored_mission(answer)
        assert sid == '20261109T101010-abc123' and mission['mission_id'] == 'llm_x'
        assert bench.stored_mission({'results': [{'status': 'refused'}]}) == (None, None)
        assert bench.stored_mission(None) == (None, None)

    def test_cold_after_five_idle_minutes(self):
        assert bench.is_cold(300.0) and bench.is_cold(1000.0)
        assert not bench.is_cold(299.0) and not bench.is_cold(None)

    def test_production_table_uses_its_own_stages(self):
        run = {'command': 'a', 'run': 0, 'result': {'status': 'ok', 'attempts': 1},
               'error': None, 't': {'t_running': 16.0},
               'deltas': bench.compute_prod_deltas(TestProductionFlow.T)}
        doc = bench.prod_session_doc({}, [run, dict(run, run=1, variant='pause3')])
        assert set(doc['summaries']) == {'pause0', 'pause3'}
        assert doc['summary']['kept'] == 1
        table = bench.markdown_table(doc['summary'], bench.PROD_DELTAS, bench.PROD_STAGES)
        row = next(ln for ln in table.splitlines() if ln.startswith('| a |'))
        assert '**12.000 / 12.000 / 12.000**' in row          # llm_and_ctl is the largest

    def test_flow_production_parses(self):
        args = bench.build_parser().parse_args(
            ['--car-is-safe', '--flow', 'production', '--variant-runs', '3'])
        assert args.flow == 'production' and args.variant_pause_s == 3.0

    def test_production_refuses_without_the_flag_too(self, monkeypatch):
        called = []
        monkeypatch.setattr(bench, 'run_prod_session', lambda *a, **k: called.append(1))
        assert bench.main(['--flow', 'production']) != 0
        assert called == []


# ---------------------------------------------------------------- live

def _port_open(port: int) -> bool:
    try:
        with socket.create_connection(('127.0.0.1', port), timeout=1.0):
            return True
    except OSError:
        return False


@pytest.mark.stack
def test_live_oneshot_reaches_running():
    """3 oneshot runs of one simple command: RUNNING every time, aborted at the end."""
    if os.environ.get('F1TENTH_CAR_SAFE') != '1':
        pytest.skip('F1TENTH_CAR_SAFE=1 not set: this test starts real missions '
                    '(set it only with the wheels lifted or the motor output disabled)')
    if not _port_open(8083):
        pytest.skip('no llama-server on 127.0.0.1:8083')
    try:
        import rclpy  # noqa: F401
        import f1tenth_messages.msg  # noqa: F401
    except ImportError as exc:
        pytest.skip(f'not in the stack ROS environment: {exc}')
    probe = bench.Stack()
    try:
        if not probe.services_available(5.0):
            pytest.skip(f'{bench.ABORT_SERVICE} not available: is the stack up?')
    finally:
        probe.close()
    budget = float(os.environ.get('F1TENTH_START_BUDGET_S', '30'))
    args = bench.build_parser().parse_args(
        ['--car-is-safe', '--cmd', bench.DEFAULT_SUITE[0], '--runs', '3'])
    session = bench.run_session(args, log=lambda msg: sys.stdout.write(msg + '\n'))
    assert session['complete'], 'the final abort was not confirmed'
    assert session['final_state'] in bench.STOPPED_STATES
    assert len(session['runs']) == 3
    for run in session['runs']:
        assert run['t'].get('t_running') is not None, run.get('error')
    median = session['summary']['overall']['total_to_running']['median']
    assert median < budget, f'median total_to_running {median:.2f} s > budget {budget} s'


@pytest.mark.stack
def test_live_production_flow_reaches_move():
    """3 runs through f1tenth_ctl (send, planner-log, mission-load, mission-start): first move."""
    if os.environ.get('F1TENTH_CAR_SAFE') != '1':
        pytest.skip('F1TENTH_CAR_SAFE=1 not set: this test starts real missions '
                    '(set it only with the wheels lifted or the motor output disabled)')
    if not _port_open(8083):
        pytest.skip('no llama-server on 127.0.0.1:8083')
    try:
        import rclpy  # noqa: F401
        import f1tenth_messages.msg  # noqa: F401
    except ImportError as exc:
        pytest.skip(f'not in the stack ROS environment: {exc}')
    if not os.path.exists(bench.CTL_BIN_DEFAULT):
        pytest.skip(f'no f1tenth_ctl at {bench.CTL_BIN_DEFAULT}')
    unit = bench.planner_unit(os.path.expanduser(bench.PLANNER_LOG_DEFAULT))
    if not unit['cmdline'] or not unit['dry_run']:
        pytest.skip('no --dry-run planner unit running (`f1tenth_ctl start planner`)')
    budget = float(os.environ.get('F1TENTH_START_BUDGET_S', '30'))
    args = bench.build_parser().parse_args(
        ['--car-is-safe', '--flow', 'production', '--cmd', bench.DEFAULT_SUITE[0],
         '--runs', '3'])
    args.planner_log = os.path.expanduser(args.planner_log)
    session = bench.run_session(args, log=lambda msg: sys.stdout.write(msg[:300] + '\n'))
    assert session['complete'], 'the final abort was not confirmed'
    assert session['final_state'] in bench.STOPPED_STATES
    assert len(session['runs']) == 3
    for run in session['runs']:
        assert run['t'].get('t_running') is not None, run.get('error')
        assert run['t'].get('t_first_move') is not None, run.get('error')
    median = session['summary']['overall']['total_to_move']['median']
    assert median < budget, f'median total_to_move {median:.2f} s > budget {budget} s'


if __name__ == '__main__':
    sys.exit(pytest.main([__file__, '-v']))
