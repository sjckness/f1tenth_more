"""StageTimer: one report per period, percentiles per stage, reset after reporting."""

import json

from f1tenth_perception.stage_timing import StageTimer


def test_first_call_only_starts_the_period():
    timer = StageTimer('n', period_s=30.0)
    timer.add('age.entry', 10.0)
    assert timer.report(100.0) is None
    assert timer.report(129.9) is None


def test_report_has_percentiles_counts_and_resets():
    timer = StageTimer('yolo', period_s=30.0)
    timer.report(0.0)
    for v in range(1, 101):
        timer.add('dur.inference', float(v))
    timer.count('frames_processed', 100)
    timer.count('frames_skipped', 3)
    report = timer.report(30.0)
    stage = report['stages']['dur.inference']
    assert stage['n'] == 100
    assert stage['p50'] == 50.5
    assert stage['p95'] == 95.05
    assert stage['max'] == 100.0
    assert report['counts'] == {'frames_processed': 100, 'frames_skipped': 3}
    assert report['node'] == 'yolo' and report['period_s'] == 30.0
    assert timer.report(45.0) is None
    assert timer.report(60.0) == {'node': 'yolo', 'period_s': 30.0, 'stages': {}, 'counts': {}}


def test_non_finite_samples_are_dropped():
    timer = StageTimer('n', period_s=1.0)
    timer.report(0.0)
    timer.add('x', float('nan'))
    timer.add('x', float('inf'))
    timer.add('x', 2.0)
    assert timer.report(1.0)['stages']['x']['n'] == 1


def test_format_is_one_parseable_line():
    timer = StageTimer('n', period_s=1.0)
    timer.report(0.0)
    timer.add('age.exit', 1.5)
    line = StageTimer.format(timer.report(1.0))
    assert line.startswith('STAGE_TIMING ') and '\n' not in line
    assert json.loads(line[len('STAGE_TIMING '):])['stages']['age.exit']['p50'] == 1.5
