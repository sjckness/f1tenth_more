"""Run the go_to live evals and print a markdown table.

Same cases and the same retry loop as
src/f1tenth_intelligence/llm/test/test_intent_evals_live.py (imported from
there, not copied). Needs llama-server on llm_planner_node.LLAMA_URL and a
sourced workspace.

    python3 docs/analysis/go_to_live_eval.py
"""

import json
import pathlib
import sys
import time

HERE = pathlib.Path(__file__).resolve()
sys.path.insert(0, str(HERE.parents[2] / 'src' / 'f1tenth_intelligence' / 'llm' / 'test'))

import test_intent_evals_live as evals  # noqa: E402
from llm.plan_translate import INTENT_PROMPT_NO_GO_TO_FILENAME, load_intent_prompt  # noqa: E402


def _table(title, prompt, cases, go_to_enabled):
    print(f'\n### {title}\n')
    print('| # | command | intent (accepted) | retries | result | s |')
    print('|---|---|---|---|---|---|')
    passed = 0
    for i, (command, expected, why) in enumerate(cases, 1):
        t0 = time.time()
        r = evals.run_case(prompt, command, expected, go_to_enabled)
        dt = time.time() - t0
        passed += r['passed']
        intent = json.dumps(r['intent'], ensure_ascii=False, separators=(',', ':'))
        verdict = 'pass' if r['passed'] else f'FAIL: {r["reason"]}'
        print(f'| {i} | {command} | `{intent}` | {r["retries"]} | {verdict} | {dt:.1f} |')
    print(f'\n{passed}/{len(cases)} passed')
    return passed


def main():
    if not evals._server_is_up():
        raise SystemExit(f'no llama-server on {evals.llm_planner_node.LLAMA_URL}')
    _table('go_to_enabled = true (planner_system_prompt.v2.it.txt)',
           load_intent_prompt(), evals.CASES, True)
    _table('go_to_enabled = false (planner_system_prompt.v2.no_go_to.it.txt)',
           load_intent_prompt(INTENT_PROMPT_NO_GO_TO_FILENAME), evals.NO_GO_TO_CASES, False)


if __name__ == '__main__':
    main()
