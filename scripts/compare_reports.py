#!/usr/bin/env python3
"""Render one comparison.md from two llm_analytics captures of the same commands.

`scripts/analytics.sh compare A B` runs one model per folder and then calls this
script with explicit paths; nothing here guesses at a layout or a file name. It
is a reader only -- argparse, hashlib, json, os, statistics, sys, time, all
stdlib -- so it also runs on a Jetson with no ROS, no PyYAML and no venv.

What makes the numbers comparable is checked, not assumed: the command file's
sha256, the system prompt's sha256 and every request body's sha256 have to match
between the two captures (planner.md §2.1). A mismatch is printed with the
position it first showed up at, because a latency difference caused by a
different prompt is not a difference between models.

Usage:
    scripts/compare_reports.py --a A/ --b B/ --out comparison.md \
        --name-a qwen25_3b_instruct --name-b other \
        [--commands commands.txt] [--airlock on|off] \
        [--go-to enabled|disabled] [--exit-a 0] [--exit-b 0]

Exit status: 0 written · 2 a report could not be read. A missing optional
section (summary, server) is not an error: the aggregate falls back to the
per-exchange records, and says so by leaving that cell as '-'.
"""

import argparse
import hashlib
import json
import os
import statistics
import sys
import time

# scripts/ lives one level under the workspace root, next to src/.
REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PROMPTS_DIR = os.path.join(
    REPO_ROOT, 'src', 'f1tenth_intelligence', 'llm', 'prompts')

DASH = '-'


def as_dict(value):
    """The value if it is a dict, else an empty one (reports have holes)."""
    return value if isinstance(value, dict) else {}


def as_list(value):
    """The value if it is a list, else an empty one."""
    return value if isinstance(value, list) else []


def sha256_file(path):
    """Hex sha256 of a file, or '' when it cannot be read."""
    try:
        with open(path, 'rb') as handle:
            return hashlib.sha256(handle.read()).hexdigest()
    except OSError:
        return ''


def short_sha(value):
    """First 12 hex digits: enough to compare by eye, short enough for a cell."""
    if not isinstance(value, str) or not value:
        return ''
    return value[:12]


def _report_path(path):
    """`/x/a` -> `/x/a/report.json`; a report path stays as it is."""
    resolved = os.path.abspath(path)
    if os.path.isdir(resolved):
        return os.path.join(resolved, 'report.json')
    return resolved


def default_name(path):
    """Folder name for a capture path: `<dir>/report.json` -> `<dir>`."""
    return os.path.basename(os.path.dirname(_report_path(path))) or 'model'


def local_commands_sha(report, path):
    """sha256 of the capture's own commands.txt, so the check survives old reports.

    Older captures have no session.commands_sha256 (llm_analytics gained it
    later), but they still wrote the snapshot next to the report: same bytes,
    same answer.
    """
    directory = os.path.dirname(_report_path(path))
    candidates = []
    snapshot = as_dict(report.get('session')).get('commands_snapshot')
    if isinstance(snapshot, str) and snapshot:
        candidates.append(snapshot)
    candidates.append(os.path.join(directory, 'commands.txt'))
    for candidate in candidates:
        if os.path.isfile(candidate):
            digest = sha256_file(candidate)
            if digest:
                return digest
    return None


def load_report(path):
    """(report dict, resolved path, problem text) for a folder or a report.json."""
    resolved = _report_path(path)
    try:
        with open(resolved, encoding='utf-8') as handle:
            data = json.load(handle)
    except OSError as exc:
        return {}, resolved, f'cannot read {resolved}: {exc}'
    except ValueError as exc:
        return {}, resolved, f'{resolved} is not valid JSON: {exc}'
    if not isinstance(data, dict):
        return {}, resolved, f'{resolved} is not a report object'
    return data, resolved, ''


def completions(report):
    """The records that measured a model, in the order they were run."""
    records = [as_dict(record) for record in as_list(report.get('exchanges'))]
    measured = [record for record in records if record.get('kind') == 'completion']
    if measured:
        return measured
    # Older captures have no `kind`: keep everything that is not a warmup.
    return [record for record in records if record.get('kind') != 'warmup']


def stats(values):
    """mean/median/n for the numbers among `values`; {} when there are none."""
    ordered = sorted(value for value in values
                     if isinstance(value, (int, float)) and not isinstance(value, bool))
    if not ordered:
        return {}
    return {
        'n': len(ordered),
        'mean': statistics.fmean(ordered),
        'median': statistics.median(ordered),
    }


def wall_ms(record):
    """Wall time of one exchange in ms, or None."""
    value = as_dict(record).get('wall_ms')
    if isinstance(value, (int, float)) and not isinstance(value, bool):
        return value
    return None


def thought_of(record):
    return as_dict(as_dict(record).get('thought'))


def verdict_of(record):
    """The airlock verdict string (`accepted:...` / `rejected:...`), or ''."""
    value = thought_of(record).get('airlock')
    return value if isinstance(value, str) else ''


def json_valid_of(record):
    return bool(thought_of(record).get('json_valid'))


def command_text(record):
    """The exact command the model was asked to execute, or None."""
    value = as_dict(as_dict(record).get('input')).get('command')
    return value if isinstance(value, str) else None


def body_sha(record):
    value = as_dict(as_dict(record).get('request')).get('body_sha256')
    return value if isinstance(value, str) and value else None


def prompt_sha(record):
    value = as_dict(as_dict(record).get('input')).get('system_prompt_sha256')
    return value if isinstance(value, str) and value else None

def aggregate(report):
    """Everything the aggregate table needs, from summary.* or from the records.

    summary.* is preferred -- it is the tool's own reduction -- but a stripped
    or older report must still render: every value falls back to the exchanges.
    """
    summary = as_dict(report.get('summary'))
    server = as_dict(report.get('server'))
    records = completions(report)
    kinds = as_dict(summary.get('by_kind'))

    total = summary.get('exchanges')
    if not isinstance(total, int):
        total = len(as_list(report.get('exchanges')))
    done = kinds.get('completion')
    if not isinstance(done, int):
        done = len(records)
    failures = summary.get('failures')
    if not isinstance(failures, int):
        failures = sum(1 for record in as_list(report.get('exchanges'))
                       if as_dict(record.get('response')).get('http_status') != 200)
    json_valid = summary.get('json_valid')
    if not isinstance(json_valid, int):
        json_valid = sum(1 for record in records if json_valid_of(record))

    verdicts = sum(1 for record in records if verdict_of(record))
    airlock = as_dict(summary.get('airlock'))
    accepted = airlock.get('accepted')
    rejected = airlock.get('rejected')
    by_error = as_dict(airlock.get('by_error'))
    if not isinstance(accepted, int) and not isinstance(rejected, int):
        accepted = rejected = 0
        by_error = {}
        for record in records:
            verdict = verdict_of(record)
            if not verdict:
                continue
            state, _, reason = verdict.partition(':')
            if state.strip().startswith('accepted'):
                accepted += 1
            elif state.strip().startswith('rejected'):
                rejected += 1
                name = reason.split(':', 1)[0].strip() or 'rejected'
                by_error[name] = by_error.get(name, 0) + 1

    wall = as_dict(summary.get('wall_ms'))
    if not wall:
        wall = stats([wall_ms(record) for record in records])
    decode = as_dict(summary.get('predicted_per_second'))
    if not decode:
        decode = stats([as_dict(record.get('timings')).get('predicted_per_second')
                        for record in records])

    peak = server.get('rss_peak_mb')
    if not isinstance(peak, (int, float)) or isinstance(peak, bool):
        peak = None

    return {
        'exchanges': total,
        'completions': done,
        'failures': failures,
        'json_valid': json_valid,
        'accepted': accepted,
        'rejected': rejected,
        'by_error': by_error,
        'airlock_ran': bool(airlock) or verdicts > 0,
        'wall': wall,
        'decode': decode,
        'rss_peak_mb': peak,
        'records': records,
    }


def prompt_names_by_sha():
    """{sha256: file name} for every prompt in prompts/, so nothing is hardcoded.

    The file name has changed before (llm/memory.md) and `--no-go-to` swaps it,
    so the comparison asks "which file has this hash" instead of assuming one.
    """
    table = {}
    try:
        names = sorted(os.listdir(PROMPTS_DIR))
    except OSError:
        return table
    for name in names:
        if not name.endswith('.txt'):
            continue
        digest = sha256_file(os.path.join(PROMPTS_DIR, name))
        if digest:
            table.setdefault(digest, name)
    return table


def session_commands_sha(report):
    value = as_dict(report.get('session')).get('commands_sha256')
    return value if isinstance(value, str) and value else None


def first_prompt_sha(records):
    for record in records:
        value = prompt_sha(record)
        if value:
            return value
    return None


def body_check(records_a, records_b):
    """How many positions have the same request body and the same command text."""
    total = max(len(records_a), len(records_b))
    same_body = 0
    same_command = 0
    problems = []
    for index in range(total):
        left = records_a[index] if index < len(records_a) else None
        right = records_b[index] if index < len(records_b) else None
        sha_a = body_sha(left)
        sha_b = body_sha(right)
        text_a = command_text(left)
        text_b = command_text(right)
        if sha_a and sha_a == sha_b:
            same_body += 1
        if text_a is not None and text_a == text_b:
            same_command += 1
        if sha_a != sha_b or text_a != text_b:
            problems.append((index + 1, sha_a, sha_b, text_a != text_b))
    return {
        'total': total,
        'same_body': same_body,
        'same_command': same_command,
        'problems': problems,
    }

def number(value, digits=1):
    if not isinstance(value, (int, float)) or isinstance(value, bool):
        return DASH
    return f'{value:.{digits}f}'


def delta(left, right, digits=1):
    if not isinstance(left, (int, float)) or not isinstance(right, (int, float)):
        return DASH
    return f'{right - left:+.{digits}f}'


def ratio(part, whole):
    if not isinstance(whole, int) or whole <= 0:
        return DASH
    return f'{part}/{whole} ({100.0 * part / whole:.0f}%)'


def clip(text, limit=120):
    if not isinstance(text, str):
        return text
    if len(text) <= limit:
        return text
    return text[:limit - 3].rstrip() + '...'


def cell(value):
    """One markdown table cell: no raw pipe, no raw newline, no empty cell."""
    if value is None or value == '':
        return DASH
    text = str(value).replace('\\', '\\\\').replace('|', '\\|')
    for newline in ('\r\n', '\r', '\n'):
        text = text.replace(newline, '<br>')
    return text or DASH


def verdict_text(record):
    """`Accepted: 2 moves` / `Rejected: IntentSchemaError: ...` / `not run`."""
    verdict = verdict_of(record)
    if not verdict.strip():
        return 'not run'
    state, _, reason = verdict.partition(':')
    state = state.strip().capitalize()
    reason = clip(reason.strip(), 70)
    return f'{state}: {reason}' if reason else state


def verdict_state(record):
    """'accepted' / 'rejected' / 'none' -- what the per-command diff compares."""
    verdict = verdict_of(record).lower()
    if verdict.startswith('accepted'):
        return 'accepted'
    if verdict.startswith('rejected'):
        return 'rejected'
    return 'none'


def diff_cell(left, right):
    """`missing` when a model recorded nothing, else the flags that differ."""
    if left is None or right is None:
        return 'missing'
    flags = []
    if json_valid_of(left) != json_valid_of(right):
        flags.append('json')
    if verdict_state(left) != verdict_state(right):
        flags.append('airlock')
    if not flags:
        return DASH
    return '**\u2260 ' + '+'.join(flags) + '**'

def render(metrics_a, metrics_b, name_a, name_b, ctx):
    """The whole comparison.md as one string (pure: no disk, no clock but `now`)."""
    records_a = metrics_a['records']
    records_b = metrics_b['records']
    lines = []
    add = lines.append

    add(f'# Sequential comparison: {name_a} vs {name_b}')
    add('')
    add('One command file, two models, one llama-server at a time: the second')
    add('starts only after the first is down and its RAM is released. Both')
    add('captures come from `llm_analytics`; this page only reduces them.')
    add('')
    add('## Metadata')
    add('')
    add(f'- generated: {ctx["now"]}')
    add(f'- comparison: `{ctx["dir"]}` (this folder holds both captures and this file)')
    add(f'- model A: `{name_a}` - {metrics_a["exchanges"]} exchanges, '
        f'{metrics_a["completions"]} completions, tool exit {ctx["exit_a"]}')
    add(f'  - capture `{ctx["sub_a"]}`: report.json, report.md, events.jsonl, '
        'commands.txt, llama-server.log')
    add(f'- model B: `{name_b}` - {metrics_b["exchanges"]} exchanges, '
        f'{metrics_b["completions"]} completions, tool exit {ctx["exit_b"]}')
    add(f'  - capture `{ctx["sub_b"]}`: report.json, report.md, events.jsonl, '
        'commands.txt, llama-server.log')
    add(f'- settings: airlock {ctx["airlock"]}, go_to {ctx["go_to"]}')
    add('')

    add('## Checks')
    add('')
    add('Same command file, same prompt and byte-identical request bodies are what')
    add('make a latency or tok/s difference the model\'s rather than the harness\'s.')
    add('')
    add('| Check | Result |')
    add('|---|---|')

    file_sha = ctx['file_sha']
    sha_a = ctx['session_a']
    sha_b = ctx['session_b']
    if file_sha:
        parts = []
        for label, session, local in (('A', sha_a, ctx['local_a']),
                                      ('B', sha_b, ctx['local_b'])):
            value = session or local
            source = '' if session else ' (capture commands.txt)'
            parts.append(f'{label} `{short_sha(value) or DASH}` '
                         f'{_same(value, file_sha)}{source}')
        known = [value for value in (sha_a or ctx['local_a'],
                                     sha_b or ctx['local_b']) if value]
        if not known:
            verdict = 'n/a (no capture recorded one)'
        elif len(known) == 2 and all(value == file_sha for value in known):
            verdict = 'match'
        else:
            verdict = 'DIFFERENT'
        add(f'| command file `{ctx["commands_name"]}` sha256 `{short_sha(file_sha)}` '
            f'| {" · ".join(parts)} -> {verdict} |')
    else:
        add('| command file | not passed (`--commands`): the input cannot be checked |')

    prompt_a = first_prompt_sha(records_a)
    prompt_b = first_prompt_sha(records_b)
    prompt_name = ctx['prompts'].get(prompt_a) or ctx['prompts'].get(prompt_b) or ''
    prompt_label = f'`{prompt_name}` ' if prompt_name else ''
    add(f'| system prompt {prompt_label}sha256 `{short_sha(prompt_a)}` '
        f'| A `{short_sha(prompt_a)}` · B `{short_sha(prompt_b)}` -> '
        f'{_match_word(prompt_a == prompt_b and bool(prompt_a))} |')

    check = ctx['bodies']
    add(f'| request bodies (position by position) '
        f'| {check["same_body"]}/{check["total"]} identical -> '
        f'{_match_word(check["same_body"] == check["total"] and check["total"] > 0)} |')
    add(f'| command text (position by position) '
        f'| {check["same_command"]}/{check["total"]} identical -> '
        f'{_match_word(check["same_command"] == check["total"] and check["total"] > 0)} |')
    add('')

    for position, left, right, text_differs in check['problems'][:20]:
        tail = ' and the command text differs' if text_differs else ''
        add(f'- request {position}: body A `{short_sha(left) or DASH}` vs '
            f'B `{short_sha(right) or DASH}`{tail}')
    if len(check['problems']) > 20:
        add(f'- ... {len(check["problems"]) - 20} more positions differ')
    for note in ctx['notes']:
        add(f'- {note}')
    if check['problems'] or ctx['notes']:
        add('')

    add('## Aggregate')
    add('')
    add(f'| Metric | {name_a} | {name_b} | B - A |')
    add('|---|---|---|---|')
    a = metrics_a
    b = metrics_b
    rows = [
        ('Exchanges / failures',
         f'{a["exchanges"]} / {a["failures"]}',
         f'{b["exchanges"]} / {b["failures"]}',
         f'{b["exchanges"] - a["exchanges"]:+d} / {b["failures"] - a["failures"]:+d}'),
        ('Completions',
         str(a['completions']), str(b['completions']),
         f'{b["completions"] - a["completions"]:+d}'),
        ('JSON valid',
         ratio(a['json_valid'], a['completions']),
         ratio(b['json_valid'], b['completions']),
         f'{b["json_valid"] - a["json_valid"]:+d}'),
        ('Airlock accepted / rejected',
         airlock_cell(a), airlock_cell(b), DASH),
        ('Airlock errors', error_cell(a), error_cell(b), DASH),
        ('Median wall latency (ms)',
         number(a['wall'].get('median'), 0), number(b['wall'].get('median'), 0),
         delta(a['wall'].get('median'), b['wall'].get('median'), 0)),
        ('Mean wall latency (ms)',
         number(a['wall'].get('mean'), 0), number(b['wall'].get('mean'), 0),
         delta(a['wall'].get('mean'), b['wall'].get('mean'), 0)),
        ('Decode speed (tok/s, mean)',
         number(a['decode'].get('mean')), number(b['decode'].get('mean')),
         delta(a['decode'].get('mean'), b['decode'].get('mean'))),
        ('Peak server RSS (MiB)',
         number(a['rss_peak_mb'], 0), number(b['rss_peak_mb'], 0),
         delta(a['rss_peak_mb'], b['rss_peak_mb'], 0)),
    ]
    for row in rows:
        add('| ' + ' | '.join(row) + ' |')
    add('')

    add(f'## Per command ({check["total"]} positions)')
    add('')
    add(f'| # | Command | {name_a} | {name_b} | Diff | A ms | B ms |')
    add('|---|---|---|---|---|---|---|')
    for index in range(check['total']):
        left = records_a[index] if index < len(records_a) else None
        right = records_b[index] if index < len(records_b) else None
        add('| {} | {} | {} | {} | {} | {} | {} |'.format(
            index + 1,
            cell(clip(command_text(left)) if left is not None else None),
            cell(verdict_text(left) if left is not None else 'missing'),
            cell(verdict_text(right) if right is not None else 'missing'),
            diff_cell(left, right),
            cell(number(wall_ms(left), 0)),
            cell(number(wall_ms(right), 0)),
        ))
    add('')
    add('**Diff legend** - `\u2260 airlock`: the verdict itself differs (accept vs')
    add('reject); `\u2260 json`: only one produced parseable JSON; `\u2260 json+airlock`:')
    add('both; `missing`: that model has no exchange at this position, so the row is')
    add('padded with `-` rather than dropped.')
    add('')
    add('## How to read this')
    add('')
    add('- The median is the comparison. The first command pays the cold prefill for')
    add('  whichever model ran first, which drags its mean; the median barely moves.')
    add('- Identical `body_sha256` above means both models were asked byte-identical')
    add('  questions, so latency and tok/s are theirs to explain, not the harness\'s.')
    add('- Peak RSS is the server\'s high-water mark: it is the number that decides')
    add('  whether two models could ever coexist, which is why this run does not.')
    return '\n'.join(lines) + '\n'


def _same(value, reference):
    """'match' / 'DIFFERS' / 'n/a' for one sha against a reference sha."""
    if not value or not reference:
        return 'n/a'
    return 'match' if value == reference else 'DIFFERS'


def _match_word(flag):
    return 'match' if flag else 'DIFFERS'


def airlock_cell(metrics):
    """Return 'not run' when the airlock never ran: 0/0 would be a lie."""
    if not metrics['airlock_ran']:
        return 'not run'
    return f'{metrics["accepted"]} accepted / {metrics["rejected"]} rejected'


def error_cell(metrics):
    """`IntentSchemaError 3, EmptyPlanError 1` -- what the airlock said no about."""
    if not metrics['airlock_ran']:
        return 'not run'
    if not metrics['by_error']:
        return DASH
    return ', '.join(f'{name} {count}'
                     for name, count in sorted(metrics['by_error'].items()))

def build_context(opts, report_a, report_b, notes):
    """Everything render() needs that is not derived from the reports themselves."""
    commands_path = os.path.abspath(opts.commands) if opts.commands else ''
    records_a = completions(report_a)
    records_b = completions(report_b)
    return {
        'now': time.strftime('%Y-%m-%dT%H:%M:%SZ', time.gmtime()),
        'dir': os.path.basename(os.path.dirname(os.path.abspath(opts.out))),
        'sub_a': default_name(opts.a),
        'sub_b': default_name(opts.b),
        'commands_name': os.path.basename(commands_path) if commands_path else DASH,
        'file_sha': sha256_file(commands_path) if commands_path else '',
        'session_a': session_commands_sha(report_a),
        'session_b': session_commands_sha(report_b),
        'local_a': local_commands_sha(report_a, opts.a),
        'local_b': local_commands_sha(report_b, opts.b),
        'prompts': prompt_names_by_sha(),
        'bodies': body_check(records_a, records_b),
        'airlock': opts.airlock,
        'go_to': opts.go_to,
        'exit_a': DASH if opts.exit_a is None else opts.exit_a,
        'exit_b': DASH if opts.exit_b is None else opts.exit_b,
        'notes': notes,
    }


def main(argv=None):
    """CLI: read two captures, write one comparison.md, return the exit code."""
    opts = build_parser().parse_args(argv)
    report_a, resolved_a, problem_a = load_report(opts.a)
    report_b, resolved_b, problem_b = load_report(opts.b)
    if problem_a or problem_b:
        for problem in (problem_a, problem_b):
            if problem:
                print(f'ERRORE: {problem}', file=sys.stderr)
        return 2

    metrics_a = aggregate(report_a)
    metrics_b = aggregate(report_b)
    notes = []
    if not metrics_a['records']:
        notes.append(f'model A recorded no exchanges in `{resolved_a}`: its side is empty')
    if not metrics_b['records']:
        notes.append(f'model B recorded no exchanges in `{resolved_b}`: its side is empty')

    text = render(metrics_a, metrics_b,
                  opts.name_a or default_name(resolved_a),
                  opts.name_b or default_name(resolved_b),
                  build_context(opts, report_a, report_b, notes))

    directory = os.path.dirname(os.path.abspath(opts.out))
    tmp_path = opts.out + '.tmp'
    try:
        os.makedirs(directory, exist_ok=True)
        # Written beside its destination and renamed: a reader never sees half a
        # comparison, and a crash leaves the previous one intact.
        with open(tmp_path, 'w', encoding='utf-8') as handle:
            handle.write(text)
        os.replace(tmp_path, opts.out)
    except OSError as exc:
        print(f'ERRORE: cannot write {opts.out}: {exc}', file=sys.stderr)
        return 2
    print(f'comparison    {opts.out}')
    return 0

def build_parser():
    parser = argparse.ArgumentParser(
        description='Side-by-side report for two llm_analytics captures of the '
                    'same commands (scripts/analytics.sh compare calls this).')
    parser.add_argument('--a', required=True, metavar='DIR',
                        help='model A capture: a run folder or its report.json')
    parser.add_argument('--b', required=True, metavar='DIR',
                        help='model B capture: a run folder or its report.json')
    parser.add_argument('--name-a', default='',
                        help='label for A (default: the folder name)')
    parser.add_argument('--name-b', default='',
                        help='label for B (default: the folder name)')
    parser.add_argument('--out', required=True, metavar='FILE',
                        help='the comparison.md to write')
    parser.add_argument('--commands', default='',
                        help='command file both runs were fed (its sha256 is checked)')
    parser.add_argument('--airlock', choices=('on', 'off'), default='on',
                        help='whether the runs used --airlock (default: on)')
    parser.add_argument('--go-to', dest='go_to',
                        choices=('enabled', 'disabled'), default='enabled',
                        help='which prompt the runs used (default: enabled)')
    parser.add_argument('--exit-a', type=int, default=None,
                        help="model A's llm_analytics exit code")
    parser.add_argument('--exit-b', type=int, default=None,
                        help="model B's llm_analytics exit code")
    return parser


if __name__ == '__main__':
    sys.exit(main())
