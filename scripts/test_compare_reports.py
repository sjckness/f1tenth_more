"""Tests for scripts/compare_reports.py, the comparison.md renderer.

Pure dicts and tmp_path: no ROS, no server, no llm_analytics import. The two
that matter most are test_render_flags_differences_and_missing (a position that
differs is never rendered as if it matched) and test_render_charges_a_body_
mismatch (byte-identical request bodies are what make the numbers comparable,
so a mismatch has to reach the page).

Run:
    pytest scripts/test_compare_reports.py
"""

import hashlib
import importlib.util
import json
import os

HERE = os.path.dirname(os.path.abspath(__file__))


def _load():
    spec = importlib.util.spec_from_file_location(
        'compare_reports', os.path.join(HERE, 'compare_reports.py'))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


compare_reports = _load()
PROMPT_SHA = 'f' * 64


def sha(tag):
    """A 64-hex stand-in for a real sha256, so fixtures stay readable."""
    return (tag * 64)[:64]


def record(command, *, body='b', verdict='accepted: 2 moves', valid=True,
           wall_ms=1000.0, tps=6.0, status=200):
    """One completion exchange, shaped like llm_analytics writes them."""
    return {
        'kind': 'completion',
        'request': {'body_sha256': sha(body)},
        'input': {'command': command, 'system_prompt_sha256': PROMPT_SHA},
        'response': {'http_status': status},
        'thought': {'json_valid': valid, 'airlock': verdict},
        'wall_ms': wall_ms,
        'timings': {'predicted_per_second': tps},
    }


def report(records, *, session_sha='s', peak=3600.0, summary=True):
    """A report.json-shaped dict; summary=False exercises the fallbacks."""
    data = {
        'session': {'commands_sha256': sha(session_sha)},
        'server': {'rss_peak_mb': peak},
        'exchanges': records,
    }
    if not summary:
        return data
    walls = [item['wall_ms'] for item in records]
    speeds = [item['timings']['predicted_per_second'] for item in records]
    accepted = sum(1 for item in records
                   if str(item['thought']['airlock']).startswith('accepted'))
    rejected = sum(1 for item in records
                   if str(item['thought']['airlock']).startswith('rejected'))
    data['summary'] = {
        'exchanges': len(records),
        'by_kind': {'warmup': 0, 'completion': len(records), 'other': 0},
        'failures': sum(1 for item in records
                        if item['response']['http_status'] != 200),
        'json_valid': sum(1 for item in records if item['thought']['json_valid']),
        'airlock': {'accepted': accepted, 'rejected': rejected, 'by_error': {}},
        'wall_ms': {'median': sorted(walls)[len(walls) // 2],
                    'mean': sum(walls) / len(walls)},
        'predicted_per_second': {'mean': sum(speeds) / len(speeds)},
    }
    return data


def render_pair(tmp_path, report_a, report_b, commands=None,
                capture_commands=None):
    """The rendered comparison.md as text, through the real CLI/build_context."""
    dir_a = tmp_path / 'model-a'
    dir_b = tmp_path / 'model-b'
    dir_a.mkdir(exist_ok=True)
    dir_b.mkdir(exist_ok=True)
    if capture_commands is not None:
        for folder in (dir_a, dir_b):
            (folder / 'commands.txt').write_bytes(capture_commands)
    args = ['--a', str(dir_a), '--b', str(dir_b),
            '--out', str(tmp_path / 'sequential' / 'comparison.md'),
            '--name-a', 'model-a', '--name-b', 'model-b',
            '--exit-a', '0', '--exit-b', '0']
    if commands is not None:
        args += ['--commands', str(commands)]
    opts = compare_reports.build_parser().parse_args(args)
    context = compare_reports.build_context(opts, report_a, report_b, [])
    return compare_reports.render(
        compare_reports.aggregate(report_a),
        compare_reports.aggregate(report_b),
        'model-a', 'model-b', context)


def test_aggregate_falls_back_to_exchanges():
    data = report([record('go', wall_ms=1000.0),
                   record('stop', wall_ms=3000.0)], summary=False)
    data['server'] = {}
    metrics = compare_reports.aggregate(data)
    assert metrics['exchanges'] == 2
    assert metrics['completions'] == 2
    assert metrics['failures'] == 0
    assert metrics['json_valid'] == 2
    assert metrics['accepted'] == 2 and metrics['rejected'] == 0
    assert metrics['airlock_ran'] is True
    assert metrics['wall']['median'] == 2000.0
    assert metrics['wall']['mean'] == 2000.0
    assert metrics['decode']['mean'] == 6.0
    assert metrics['rss_peak_mb'] is None


def test_aggregate_prefers_the_summary_over_the_records():
    data = report([record('go')])
    data['summary'].update({
        'exchanges': 7,
        'by_kind': {'warmup': 1, 'completion': 5, 'other': 1},
        'failures': 2,
        'json_valid': 3,
        'airlock': {'accepted': 4, 'rejected': 1,
                    'by_error': {'IntentSchemaError': 1}},
        'wall_ms': {'median': 111.0, 'mean': 222.0},
        'predicted_per_second': {'mean': 9.5},
    })
    metrics = compare_reports.aggregate(data)
    assert metrics['exchanges'] == 7
    assert metrics['completions'] == 5
    assert metrics['failures'] == 2
    assert metrics['json_valid'] == 3
    assert metrics['accepted'] == 4 and metrics['rejected'] == 1
    assert metrics['by_error'] == {'IntentSchemaError': 1}
    assert metrics['wall']['median'] == 111.0
    assert metrics['decode']['mean'] == 9.5
    assert len(metrics['records']) == 1

def test_render_flags_differences_and_missing(tmp_path):
    left = report([record('go'), record('turn')])
    right = report([record('go'),
                    record('turn', verdict='rejected: IntentSchemaError: bad',
                           valid=False)])
    text = render_pair(tmp_path, left, right)
    assert '\u2260 json+airlock' in text
    assert 'Rejected: IntentSchemaError: bad' in text
    # Model B recorded nothing for the second command at all.
    shorter = render_pair(tmp_path, left, report([record('go')]))
    assert '| missing |' in shorter
    assert '1/2 identical' in shorter


def test_render_charges_a_body_mismatch(tmp_path):
    left = report([record('go'), record('turn')])
    right = report([record('go'), record('turn', body='c')])
    text = render_pair(tmp_path, left, right)
    assert '1/2 identical' in text
    assert 'DIFFERS' in text
    assert '**\u2260 ' not in text  # the commands and the verdicts still agree


def test_render_survives_a_stripped_report(tmp_path):
    text = render_pair(tmp_path, report([record('go')], summary=False),
                       report([record('go')], summary=False))
    assert '## Aggregate' in text
    assert '## Per command (1 positions)' in text
    assert 'not run' not in text  # the verdict came from the record itself


def test_command_file_check_falls_back_to_the_capture_snapshot(tmp_path):
    commands = tmp_path / 'commands.txt'
    commands.write_text('go\nstop\n', encoding='utf-8')
    left = report([record('go')])
    right = report([record('go')])
    for data in (left, right):
        data['session'].pop('commands_sha256')  # a pre-2026-11-19 capture
    text = render_pair(tmp_path, left, right, commands=commands,
                       capture_commands=commands.read_bytes())
    assert '-> match |' in text
    assert '(capture commands.txt)' in text


def test_main_writes_the_comparison_and_leaves_no_tmp(tmp_path):
    commands = tmp_path / 'commands.txt'
    commands.write_text('go\nstop\n', encoding='utf-8')
    digest = hashlib.sha256(commands.read_bytes()).hexdigest()

    for name in ('model-a', 'model-b'):
        folder = tmp_path / name
        folder.mkdir()
        data = report([record('go'), record('stop')], session_sha=name)
        data['session']['commands_sha256'] = digest
        (folder / 'report.json').write_text(json.dumps(data), encoding='utf-8')

    out = tmp_path / 'sequential' / 'comparison.md'
    rc = compare_reports.main([
        '--a', str(tmp_path / 'model-a'), '--b', str(tmp_path / 'model-b'),
        '--out', str(out), '--commands', str(commands),
        '--name-a', 'model-a', '--name-b', 'model-b'])
    assert rc == 0
    text = out.read_text(encoding='utf-8')
    assert text.startswith('# Sequential comparison: model-a vs model-b')
    assert '## Checks' in text
    assert '2/2 identical' in text
    assert 'DIFFERS' not in text
    assert not (tmp_path / 'sequential' / 'comparison.md.tmp').exists()


def test_main_refuses_a_missing_report(tmp_path, capsys):
    good = tmp_path / 'model-a'
    good.mkdir()
    (good / 'report.json').write_text('{}', encoding='utf-8')
    out = tmp_path / 'comparison.md'
    rc = compare_reports.main([
        '--a', str(good), '--b', str(tmp_path / 'nowhere'), '--out', str(out)])
    assert rc == 2
    assert 'ERRORE' in capsys.readouterr().err
    assert not out.exists()
