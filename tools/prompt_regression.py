#!/usr/bin/env python3
"""Old vs new planner prompt: same commands, same LLM, diff the missions.

Run where the llama-server runs (the Orin; linus has no LLM):
    source install/setup.bash
    python3 tools/prompt_regression.py [--url http://127.0.0.1:8083/completion]
        [--old-rev 1d00853] [--prompts first_test_campaing/prompts.yaml]
        [--command "..."]... [--out prompt_regression.json]

Commands: every `text` of the campaign prompts file (except "calibration"),
every `comando:` example in the NEW prompt, and any --command. For each
command and each prompt (old = the file at --old-rev, new = the working
tree's), the planner's own path runs: llm_planner_node.get_intent_from_llm
(same request, temperature 0) with up to MAX_INTENT_RETRIES retries on a
rejected intent, then plan_translate.translate(). Both variants of the
prompt (go_to / no_go_to) are compared.

Verdict per command:
  same                  identical mission (mission_id and the new
                        stop_condition.distance_source ignored)
  default_distance      only a wall stop distance differs and the new one
                        came from the translator default -- the intended
                        change (A4), listed so it can be checked
  REGRESSION            any other difference (move types, guards, turns,
                        a mission on one side only, a failure)
"""

import argparse
import copy
import json
import pathlib
import subprocess
import sys

import yaml

from llm import llm_planner_node as planner
from llm.plan_translate import (
    EmptyPlanError, IntentRangeError, IntentSchemaError, UnsupportedIntentModeError,
    intent_prompt_examples, translate)

REPO = pathlib.Path(__file__).resolve().parents[1]
PROMPT_DIR = 'src/f1tenth_intelligence/llm/prompts'
VARIANTS = {
    'go_to': ('planner_system_prompt.v2.it.txt', True),
    'no_go_to': ('planner_system_prompt.v2.no_go_to.it.txt', False),
}


def plan(command, system_prompt, go_to_enabled):
    feedback = None
    attempts = []
    for attempt in range(1, planner.MAX_INTENT_RETRIES + 2):
        try:
            intent = planner.get_intent_from_llm(command, system_prompt, feedback)
        except Exception as exc:  # noqa: BLE001 - recorded, not raised
            return {'status': 'llm_error', 'error': f'{type(exc).__name__}: {exc}',
                    'attempts': attempts}
        try:
            result = translate(intent, go_to_enabled=go_to_enabled)
        except EmptyPlanError:
            return {'status': 'refused', 'intent': intent, 'attempts': attempts}
        except (UnsupportedIntentModeError, IntentSchemaError, IntentRangeError) as exc:
            feedback = str(exc)
            attempts.append({'attempt': attempt, 'intent': intent, 'error': str(exc)})
            continue
        return {'status': 'ok', 'intent': intent, 'mission': result.mission,
                'notes': list(result.notes), 'attempts': attempts}
    return {'status': 'rejected', 'attempts': attempts}


def _strip(mission):
    m = copy.deepcopy(mission)
    m.pop('mission_id', None)
    for move in m.get('moves', []):
        move.get('stop_condition', {}).pop('distance_source', None)
    return m


def verdict(old, new):
    if old['status'] != 'ok' or new['status'] != 'ok':
        if old['status'] == new['status'] and old.get('intent') == new.get('intent'):
            return 'same'
        return 'REGRESSION'
    a, b = _strip(old['mission']), _strip(new['mission'])
    if a == b:
        return 'same'
    if len(a['moves']) != len(b['moves']):
        return 'REGRESSION'
    for ma, mb, raw_b in zip(a['moves'], b['moves'], new['mission']['moves']):
        if ma == mb:
            continue
        sa, sb = ma.get('stop_condition', {}), mb.get('stop_condition', {})
        others_a = {k: v for k, v in ma.items() if k not in ('stop_condition', 'timeout_sec')}
        others_b = {k: v for k, v in mb.items() if k not in ('stop_condition', 'timeout_sec')}
        only_distance = (others_a == others_b and sa.get('type') == 'front_clearance'
                         and sb.get('type') == 'front_clearance'
                         and {k: v for k, v in sa.items() if k != 'distance'}
                         == {k: v for k, v in sb.items() if k != 'distance'})
        defaulted = raw_b.get('stop_condition', {}).get('distance_source', 'llm') != 'llm'
        if not (only_distance and defaulted):
            return 'REGRESSION'
    return 'default_distance'


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--url', default=planner.LLAMA_URL)
    ap.add_argument('--old-rev', default='1d00853')
    ap.add_argument('--prompts', default=str(REPO / 'first_test_campaing' / 'prompts.yaml'))
    ap.add_argument('--command', action='append', default=[])
    ap.add_argument('--out', default='prompt_regression.json')
    args = ap.parse_args()
    planner.LLAMA_URL = args.url

    commands = []
    try:
        for entry in yaml.safe_load(open(args.prompts)) or []:
            text = str(entry.get('text', ''))
            if text and text != 'calibration':
                commands.append(text)
    except OSError as exc:
        print(f'[regression] {args.prompts}: {exc}', file=sys.stderr)
    commands += [c for c, _ in intent_prompt_examples()]
    commands += args.command
    commands = list(dict.fromkeys(commands))

    results = []
    counts = {}
    for variant, (fname, go_to) in VARIANTS.items():
        old_prompt = subprocess.run(
            ['git', '-C', str(REPO), 'show', f'{args.old_rev}:{PROMPT_DIR}/{fname}'],
            check=True, capture_output=True, text=True).stdout
        new_prompt = (REPO / PROMPT_DIR / fname).read_text()
        for cmd in commands:
            old = plan(cmd, old_prompt, go_to)
            new = plan(cmd, new_prompt, go_to)
            v = verdict(old, new)
            counts[v] = counts.get(v, 0) + 1
            results.append({'variant': variant, 'command': cmd, 'verdict': v,
                            'old': old, 'new': new})
            print(f'[{variant:8s}] {v:16s} {cmd!r}')
            if v != 'same':
                for side, r in (('old', old), ('new', new)):
                    moves = ([(m['id'], m.get('stop_condition')) for m in
                              r.get('mission', {}).get('moves', [])] if r['status'] == 'ok'
                             else r['status'])
                    print(f'      {side}: {json.dumps(moves, ensure_ascii=False)}')
    pathlib.Path(args.out).write_text(json.dumps(results, indent=2, ensure_ascii=False))
    print(f'\n{counts} -- details in {args.out}')
    return 1 if counts.get('REGRESSION') else 0


if __name__ == '__main__':
    sys.exit(main())
