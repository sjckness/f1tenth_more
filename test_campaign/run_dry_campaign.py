#!/usr/bin/env python3
"""Dry-run test campaign for the LLM planner: prompt -> LLM -> translator -> JSON.

Runs every prompt in prompts.json through

    ros2 run llm llm_planner_node '<prompt>' --dry-run

one at a time (there is a single llama-server; parallel runs would distort both
the results and the latencies), and records for each attempt exactly one
outcome: the JSON command produced, an LLM error, or a translator error.

NOTHING REACHES THE CAR. --dry-run returns in llm_planner_node.process_command()
before the /mission/abort_mission, /mission/load_mission and
/mission/start_mission calls, and the llm package has no publisher at all. The
one side effect that does happen is a mission JSON written under
f1tenth_behavior/missions/llm_generated/ (deliberate, for inspection); nothing
reads that directory on its own.

The node prints one machine-readable `RESULT {json}` line per command, which is
what classify() reads. The Italian prose markers are kept as a fallback so a
run against an older node still classifies -- see classify() for both rule sets.
"""

import argparse
import csv
import datetime
import hashlib
import json
import os
import pathlib
import shlex
import shutil
import signal
import subprocess
import sys
import time

HERE = pathlib.Path(__file__).resolve().parent
WORKSPACE = HERE.parent
PROMPTS_FILE = HERE / 'prompts.json'
RUNS_DIR = HERE / 'runs'
LLM_PKG = WORKSPACE / 'src' / 'f1tenth_intelligence' / 'llm'
SETUP_BASH = WORKSPACE / 'install' / 'setup.bash'

# The node's own marker, see llm_planner_node._emit_result.
RESULT_PREFIX = 'RESULT '

# Every status this campaign can assign. `refused` is NOT in the original
# brief's table: it is the node's EmptyPlanError path, where the LLM answered,
# the translator worked, and the correct answer was "no executable plan" (an
# ambiguous or unperformable command). Folding it into translator_error would
# hide what turns out to be the most common outcome of the whole campaign.
STATUSES = ('ok', 'llm_error', 'translator_error', 'refused',
            'node_error', 'timeout', 'unclassified')


# --------------------------------------------------------------------------
# Running one prompt
# --------------------------------------------------------------------------
def build_command(prompt, ros_params):
    """Build the argv for one dry run. Never a shell string.

    `ros2` on PATH is used directly. Otherwise the call is wrapped as
    `bash -c "source <setup.bash> && exec ros2 run ..."` with every piece
    shlex.quote()d, so a prompt's quotes and commas survive either way.
    """
    node_args = ['ros2', 'run', 'llm', 'llm_planner_node', prompt, '--dry-run']
    if ros_params:
        node_args += ['--ros-args']
        for param in ros_params:
            node_args += ['-p', param]

    if shutil.which('ros2'):
        return node_args, False
    if SETUP_BASH.exists():
        inner = ' '.join(shlex.quote(a) for a in node_args)
        return ['bash', '-c', f'source {shlex.quote(str(SETUP_BASH))} && exec {inner}'], True
    sys.exit(
        f"ERROR: 'ros2' is not on PATH and {SETUP_BASH} does not exist.\n"
        "Source the ROS 2 workspace first:\n"
        f"    source /opt/ros/humble/setup.bash && source {SETUP_BASH}")


def run_once(prompt, timeout_s, ros_params):
    """Run one prompt, capture everything, never leave a process behind.

    Popen rather than subprocess.run: run()'s own timeout kills only the direct
    child, which here is the `ros2` wrapper -- the node itself would survive as
    an orphan and keep the llama-server busy for the next prompt. start_new_
    session puts the whole run in its own process group so the timeout path can
    kill the group (TERM, then KILL if it is still there).
    """
    argv, via_shell = build_command(prompt, ros_params)
    t0 = time.monotonic()
    proc = subprocess.Popen(
        argv, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
        text=True, start_new_session=True)
    timed_out = False
    try:
        stdout, stderr = proc.communicate(timeout=timeout_s)
    except subprocess.TimeoutExpired:
        timed_out = True
        _kill_group(proc)
        stdout, stderr = proc.communicate()
    duration = time.monotonic() - t0
    return {
        'stdout': stdout or '',
        'stderr': stderr or '',
        'exit_code': proc.returncode,
        'duration_s': round(duration, 2),
        'timed_out': timed_out,
        'command': argv,
        'via_shell_wrapper': via_shell,
    }


def _kill_group(proc):
    """TERM the run's process group, then KILL whatever is still alive."""
    try:
        pgid = os.getpgid(proc.pid)
    except ProcessLookupError:
        return
    for sig, grace in ((signal.SIGTERM, 5.0), (signal.SIGKILL, 5.0)):
        try:
            os.killpg(pgid, sig)
        except ProcessLookupError:
            return
        deadline = time.monotonic() + grace
        while time.monotonic() < deadline:
            if proc.poll() is not None:
                return
            time.sleep(0.1)


# --------------------------------------------------------------------------
# Classification -- one function, one place to adjust
# --------------------------------------------------------------------------
# error_type (as the node reports it) -> (status, cause, error_kind).
# cause is only meaningful for translator_error, per the brief:
#   rejected_llm_output = the translator correctly refused bad LLM output
#                         -> the prompt/LLM needs work
#   translator_bug      = the translator itself misbehaved
#                         -> the translator needs work
ERROR_TYPE_RULES = {
    # -- LLM stage: raised inside get_intent_from_llm / _completion ---------
    # The model answered but the text was not parseable JSON (or was cut off).
    # Filed at the stage where it actually happened: the translator never ran.
    'JSONDecodeError': ('llm_error', None, 'invalid_json'),
    'ReadTimeout': ('llm_error', None, 'timeout'),
    'ConnectTimeout': ('llm_error', None, 'timeout'),
    'Timeout': ('llm_error', None, 'timeout'),
    'HTTPError': ('llm_error', None, 'http_error'),
    'ConnectionError': ('llm_error', None, 'connection_lost'),
    'LlamaServerUnreachableError': ('llm_error', None, 'unreachable'),
    # -- translator stage: raised inside plan_translate.translate() ---------
    'IntentSchemaError': ('translator_error', 'rejected_llm_output', 'schema_violation'),
    'UnsupportedIntentModeError': ('translator_error', 'rejected_llm_output', 'unsupported_mode'),
    'IntentRangeError': ('translator_error', 'rejected_llm_output', 'range_violation'),
    'RetriesExhausted': ('translator_error', 'rejected_llm_output', 'retries_exhausted'),
    'PlanTranslationError': ('translator_error', 'rejected_llm_output', 'plan_rejected'),
    'InvalidPlan': ('translator_error', 'rejected_llm_output', 'schema_violation'),
    # The translator built a document its own output check refuses. Ours, not
    # the model's -- the one error_kind that means the translator needs fixing.
    'TranslatorOutputError': ('translator_error', 'translator_bug', 'invalid_mission_emitted'),
    # -- node stage --------------------------------------------------------
    'OSError': ('node_error', None, 'mission_write_failed'),
    'FileExistsError': ('node_error', None, 'mission_id_collision'),
    'Unhandled': ('node_error', None, 'no_outcome_recorded'),
}

# Fallback prose markers, used only when the node emitted no RESULT line.
# (status, cause, error_kind, stream, marker), first match wins.
PROSE_RULES = [
    ('ok', None, None, 'stdout', 'missione generata in '),
    ('llm_error', None, 'unreachable', 'stderr', 'ERRORE: Impossibile raggiungere llama-server'),
    ('llm_error', None, 'other', 'stderr', 'LLM: generazione fallita'),
    ('llm_error', None, 'other', 'stderr', 'LLM: traduzione fallita'),
    ('translator_error', 'translator_bug', 'invalid_mission_emitted',
     'stderr', 'BUG DEL TRADUTTORE'),
    ('translator_error', 'rejected_llm_output', 'retries_exhausted',
     'stdout', 'RICHIESTA NON SUPPORTATA -- nessuna missione emessa'),
    ('refused', None, 'unsupported_request', 'stdout',
     "RICHIESTA NON SUPPORTATA -- il robot non sa farlo"),
    ('refused', None, 'ambiguous', 'stdout', 'nessun piano eseguibile'),
    ('node_error', None, 'mission_write_failed', 'stderr',
     'scrittura della missione fallita'),
]


def parse_result_line(stdout):
    """Return the node's RESULT payload, or None if it printed no such line."""
    payload = None
    for line in stdout.splitlines():
        if line.startswith(RESULT_PREFIX):
            try:
                payload = json.loads(line[len(RESULT_PREFIX):])
            except ValueError:
                payload = None          # truncated line: fall back to prose
    return payload


def classify(run):
    """Attribute one run to exactly one status. Never guesses.

    Order of the rules, most certain first:

    1. TIMEOUT wins over everything: a killed process may have printed a
       marker for an attempt that was not the one that hung.
    2. The node's own RESULT line, when present. It carries the stage the
       outcome came from and the exception class, so the attribution is the
       node's own knowledge rather than our reading of its prose.
    3. A Python traceback (no RESULT line, non-zero exit): a frame in
       plan_translate.py means the translator crashed; anything else is the
       node.
    4. The Italian prose markers, for a node without the RESULT line.
    5. Anything left over is `unclassified`, with the raw output kept.
    """
    stdout, stderr = run['stdout'], run['stderr']

    # 1. Timeout. Which stage it hung in is only claimed when the logs show it.
    if run['timed_out']:
        started = 'planner_path=' in stderr
        # After startup, the only slow thing this node does is wait on the LLM:
        # the readiness/warm-up POST, then one /completion per attempt. So a
        # hang with the node up is an LLM wait; a hang before that is not
        # attributable and stays `timeout`.
        if started:
            return _record('llm_error', None, 'timeout',
                           f'killed after {run["duration_s"]}s while waiting on the LLM')
        return _record('timeout', None, None,
                       f'killed after {run["duration_s"]}s before the node logged its startup')

    # 2. The node's own structured outcome.
    result = parse_result_line(stdout)
    if result:
        status = result.get('status')
        error_type = result.get('error_type')
        message = result.get('error_message')

        if status == 'ok':
            return _record('ok', None, None, None, result=result)

        if status == 'refused':
            # Two sub-cases, and the difference matters: a refusal on the first
            # attempt is the model genuinely declining, while a refusal that
            # follows a schema rejection is often the model parroting the
            # validator's feedback back as "ambiguo: schema non valido ...".
            # Same status, different lesson, so they get different kinds.
            if not result.get('ambiguous'):
                kind = 'unsupported_request'
            elif result.get('rejections'):
                kind = 'ambiguous_after_rejection'
            else:
                kind = 'ambiguous'
            return _record('refused', None, kind, message, result=result)

        rule = ERROR_TYPE_RULES.get(error_type)
        if rule:
            status_r, cause, kind = rule
            # A RuntimeError from _completion is the mid-session connection
            # loss; any other RuntimeError is not, so it is matched on text.
            return _record(status_r, cause, kind, message, result=result)
        if error_type == 'RuntimeError' and 'Impossibile contattare llama-server' in (message or ''):
            return _record('llm_error', None, 'connection_lost', message, result=result)
        if status in STATUSES:
            # The node was certain of the stage even though we do not know the
            # exception: keep its status, admit the kind is unknown.
            cause = 'rejected_llm_output' if status == 'translator_error' else None
            return _record(status, cause, f'unmapped:{error_type}', message, result=result)
        return _record('unclassified', None, f'unmapped_status:{status}', message, result=result)

    # 3. A traceback, i.e. an exception nobody caught.
    if 'Traceback (most recent call last)' in stderr:
        last = stderr.strip().splitlines()[-1]
        if 'plan_translate.py' in stderr:
            return _record('translator_error', 'translator_bug', 'crash', last)
        return _record('node_error', None, 'crash', last)

    # 4. Prose markers (node without a RESULT line).
    for status, cause, kind, stream, marker in PROSE_RULES:
        if marker in (stdout if stream == 'stdout' else stderr):
            return _record(status, cause, kind, _first_error_log(stderr))

    # 5. Nothing matched.
    return _record('unclassified', None, None,
                   f'no marker found (exit {run["exit_code"]})')


def _record(status, cause, error_kind, error_message, result=None):
    """Assemble the classification half of a result record."""
    return {
        'status': status,
        'cause': cause,
        'error_kind': error_kind,
        'error_message': error_message,
        'node_result': result,
    }


def _first_error_log(stderr):
    """First [ERROR] line, the readable summary of a prose-matched failure."""
    for line in stderr.splitlines():
        if '[ERROR]' in line:
            return line.split(']', 3)[-1].strip()
    return None


# --------------------------------------------------------------------------
# Record, outputs
# --------------------------------------------------------------------------
def make_record(run_id, spec, attempt, run, verdict):
    """One results.jsonl line: what was asked, what happened, what it means."""
    result = verdict['node_result'] or {}
    return {
        'run_id': run_id,
        'id': spec['id'],
        'category': spec['category'],
        'attempt': attempt,
        'prompt': spec['prompt'],
        'timestamp': datetime.datetime.now().astimezone().isoformat(timespec='seconds'),
        'status': verdict['status'],
        'error_kind': verdict['error_kind'],
        'error_message': verdict['error_message'],
        'cause': verdict['cause'],
        # Only when ok, and parsed -- an object, not a string.
        'output_json': result.get('mission') if verdict['status'] == 'ok' else None,
        # What the model actually said, whenever the node let us see it: the
        # accepted intent, the rejected intent, or the unparseable raw text.
        'llm_raw_output': result.get('llm_raw'),
        'llm_attempts': result.get('attempts'),
        'rejections': result.get('rejections'),
        'mission_id': result.get('mission_id'),
        'mission_path': result.get('mission_path'),
        'stdout': run['stdout'],
        'stderr': run['stderr'],
        'exit_code': run['exit_code'],
        'duration_s': run['duration_s'],
    }


def write_summary(path, records):
    """counts per status, per status:cause:error_kind, per category, per prompt."""
    per_status, per_combo, per_category, per_prompt = {}, {}, {}, {}
    for r in records:
        per_status[r['status']] = per_status.get(r['status'], 0) + 1
        combo = f"{r['status']}:{r['cause'] or '-'}:{r['error_kind'] or '-'}"
        per_combo[combo] = per_combo.get(combo, 0) + 1
        cat = per_category.setdefault(r['category'], {})
        cat[r['status']] = cat.get(r['status'], 0) + 1
        p = per_prompt.setdefault(r['id'], {'category': r['category'], 'prompt': r['prompt'],
                                            'attempts': 0, 'ok': 0, 'statuses': {}})
        p['attempts'] += 1
        p['ok'] += 1 if r['status'] == 'ok' else 0
        p['statuses'][r['status']] = p['statuses'].get(r['status'], 0) + 1
    for p in per_prompt.values():
        p['score'] = f"{p['ok']}/{p['attempts']} ok"
    summary = {
        'total': len(records),
        'per_status': dict(sorted(per_status.items(), key=lambda kv: -kv[1])),
        'per_status_cause_kind': dict(sorted(per_combo.items(), key=lambda kv: -kv[1])),
        'per_category': per_category,
        'per_prompt': per_prompt,
        'duration_s': {
            'total': round(sum(r['duration_s'] for r in records), 1),
            'mean': round(sum(r['duration_s'] for r in records) / max(len(records), 1), 1),
            'max': max((r['duration_s'] for r in records), default=0),
        },
    }
    path.write_text(json.dumps(summary, indent=2, ensure_ascii=False) + '\n', encoding='utf-8')
    return summary


def write_csv(path, records):
    """Flat view for a spreadsheet."""
    with open(path, 'w', newline='', encoding='utf-8') as fh:
        writer = csv.writer(fh)
        writer.writerow(['id', 'category', 'attempt', 'status', 'cause', 'error_kind',
                         'error_message', 'output_json', 'duration_s'])
        for r in records:
            message = (r['error_message'] or '').replace('\n', ' ')
            writer.writerow([
                r['id'], r['category'], r['attempt'], r['status'], r['cause'] or '',
                r['error_kind'] or '', message[:200],
                json.dumps(r['output_json'], ensure_ascii=False) if r['output_json'] else '',
                r['duration_s'],
            ])


def git_state():
    """Commit of the llm package, and whether it has uncommitted changes."""
    def git(*args):
        try:
            return subprocess.run(['git', '-C', str(WORKSPACE), *args],
                                  capture_output=True, text=True, timeout=15).stdout.strip()
        except (OSError, subprocess.SubprocessError):
            return None
    return {
        'llm_package_commit': git('log', '-1', '--format=%H', '--', str(LLM_PKG)),
        'llm_package_commit_subject': git('log', '-1', '--format=%s', '--', str(LLM_PKG)),
        'workspace_head': git('rev-parse', 'HEAD'),
        'branch': git('rev-parse', '--abbrev-ref', 'HEAD'),
        'llm_package_dirty': bool(git('status', '--porcelain', '--', str(LLM_PKG))),
        'llm_package_dirty_files': (git('status', '--porcelain', '--', str(LLM_PKG)) or '').splitlines(),
    }


def llm_settings():
    """Model and server settings, read from the llm package's own config."""
    settings = {'source': str(LLM_PKG / 'config')}
    try:
        import yaml                      # ships with ROS 2; optional here
        interrogations = yaml.safe_load((LLM_PKG / 'config' / 'interrogations.yaml').read_text())
        models = yaml.safe_load((LLM_PKG / 'config' / 'models.yaml').read_text())
        model_name = (interrogations.get('planner') or {}).get('default_model')
        settings['model_name'] = model_name
        settings['model'] = models.get(model_name)
    except Exception as exc:             # config is evidence, not a dependency
        settings['error'] = f'{type(exc).__name__}: {exc}'
    # The defaults the node itself compiles in, straight from the source.
    node_src = (LLM_PKG / 'llm' / 'llm_planner_node.py').read_text(encoding='utf-8')
    for key in ('LLAMA_URL', 'LLAMA_TIMEOUT', 'DEFAULT_PLANNER_PATH', 'MAX_INTENT_RETRIES'):
        for line in node_src.splitlines():
            if line.startswith(key + ' ='):
                settings[key] = line.split('=', 1)[1].split('#')[0].strip()
                break
    return settings


def print_summary_table(summary):
    """The end-of-run table."""
    print('\n' + '=' * 72)
    print(f"SUMMARY -- {summary['total']} runs, "
          f"{summary['duration_s']['total']}s total, "
          f"{summary['duration_s']['mean']}s mean, "
          f"{summary['duration_s']['max']}s max")
    print('=' * 72)
    print(f"{'status':<18}{'cause':<22}{'error_kind':<26}{'count':>5}")
    print('-' * 72)
    for combo, count in summary['per_status_cause_kind'].items():
        status, cause, kind = combo.split(':', 2)
        print(f'{status:<18}{cause:<22}{kind:<26}{count:>5}')
    print('-' * 72)
    print(f"{'TOTAL':<66}{summary['total']:>5}")
    print('\nper category:')
    for cat, counts in sorted(summary['per_category'].items()):
        line = ', '.join(f'{k} {v}' for k, v in sorted(counts.items()))
        print(f'  {cat:<22}{line}')


# --------------------------------------------------------------------------
def main():
    parser = argparse.ArgumentParser(
        description='Dry-run test campaign for the LLM planner (nothing reaches the car).')
    parser.add_argument('--repeat', type=int, default=1,
                        help='attempts per prompt (the LLM is not deterministic)')
    parser.add_argument('--only', nargs='+', metavar='ID',
                        help='run only these prompt ids')
    parser.add_argument('--timeout', type=float, default=120.0,
                        help='per-run timeout in seconds (default 120)')
    parser.add_argument('--resume', metavar='RUN_DIR',
                        help='continue that run: skip prompt/attempt pairs already recorded')
    parser.add_argument('--ros-param', action='append', default=[], metavar='NAME:=VALUE',
                        help='extra ROS parameter for every run, e.g. llm_timeout_sec:=3.0 '
                             '(fault injection; recorded in run_info.json)')
    args = parser.parse_args()

    prompts = json.loads(PROMPTS_FILE.read_text(encoding='utf-8'))
    if args.only:
        wanted = set(args.only)
        unknown = wanted - {p['id'] for p in prompts}
        if unknown:
            sys.exit(f'ERROR: unknown prompt id(s): {", ".join(sorted(unknown))}')
        prompts = [p for p in prompts if p['id'] in wanted]

    # --resume continues the given run in place; otherwise a new timestamped dir.
    done = set()
    records = []
    if args.resume:
        run_dir = pathlib.Path(args.resume).resolve()
        if not (run_dir / 'results.jsonl').exists():
            sys.exit(f'ERROR: no results.jsonl in {run_dir}')
        for line in (run_dir / 'results.jsonl').read_text(encoding='utf-8').splitlines():
            if line.strip():
                rec = json.loads(line)
                records.append(rec)
                done.add((rec['id'], rec['attempt']))
        run_id = run_dir.name
        print(f'resuming {run_dir} -- {len(done)} attempts already recorded')
    else:
        run_id = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
        run_dir = RUNS_DIR / run_id
        run_dir.mkdir(parents=True, exist_ok=False)

    todo = [(spec, attempt) for spec in prompts
            for attempt in range(1, args.repeat + 1)
            if (spec['id'], attempt) not in done]

    (run_dir / 'run_info.json').write_text(json.dumps({
        'run_id': run_id,
        'started': datetime.datetime.now().astimezone().isoformat(timespec='seconds'),
        'command_template': ' '.join(build_command('<prompt>', args.ros_param)[0]),
        'via_shell_wrapper': build_command('<prompt>', args.ros_param)[1],
        'cli_options': vars(args),
        'prompts_file': str(PROMPTS_FILE),
        'prompts_sha256': hashlib.sha256(PROMPTS_FILE.read_bytes()).hexdigest(),
        'prompts_count': len(prompts),
        'planned_runs': len(todo),
        'git': git_state(),
        'llm': llm_settings(),
    }, indent=2, ensure_ascii=False) + '\n', encoding='utf-8')

    print(f'run {run_id}: {len(todo)} runs '
          f'({len(prompts)} prompts x {args.repeat}), timeout {args.timeout}s, one at a time')
    if args.ros_param:
        print(f'  ROS params injected: {" ".join(args.ros_param)}')

    results_path = run_dir / 'results.jsonl'
    with open(results_path, 'a', encoding='utf-8') as out:
        for i, (spec, attempt) in enumerate(todo, start=1):
            run = run_once(spec['prompt'], args.timeout, args.ros_param)
            verdict = classify(run)
            record = make_record(run_id, spec, attempt, run, verdict)
            records.append(record)
            # Flushed and fsynced per run: a crash must never cost a result.
            out.write(json.dumps(record, ensure_ascii=False) + '\n')
            out.flush()
            os.fsync(out.fileno())

            detail = verdict['error_kind'] or ''
            if verdict['cause']:
                detail = f"{verdict['cause']}: {detail}"
            detail = f' ({detail})' if detail else ''
            print(f"[{i}/{len(todo)}] {spec['id']} #{attempt}: "
                  f"{verdict['status']}{detail} {run['duration_s']}s", flush=True)

    summary = write_summary(run_dir / 'summary.json', records)
    write_csv(run_dir / 'results.csv', records)
    print_summary_table(summary)
    print(f'\nwritten to {run_dir}')


if __name__ == '__main__':
    main()
