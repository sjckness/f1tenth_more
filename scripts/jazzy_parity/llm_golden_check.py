#!/usr/bin/env python3
"""
Translate every golden intent fixture and compare with the committed mission.

Per fixture: whether translate(intent) equals the committed mission, plus a
sha256 of the canonical JSON of what it produced.

Runs on: the Orin host with native Humble sourced (REPO = the native
checkout, ~/dev_ws/f1tenth_more; Python 3.10), inside the harness container
(REPO = /opt/f1tenth_ws, the image's sources; Python 3.12), linus.
Compares: run on both sides, parity_gates.py golden checks that every
fixture is reproduced on each side (P9.5 "LLM golden check 10/10") and that
both sides produced the same missions (same sha256), independent of any
llama-server.

plan_translate.py is pure Python, but it imports f1tenth_params,
f1tenth_description and f1tenth_behavior lazily (ament index), so the
side's install must be sourced. llm itself is imported from REPO's source
tree (--installed-llm: from the sourced install instead).

Usage: llm_golden_check.py REPO [--installed-llm]
Exit 0 when every fixture with a mission is reproduced exactly.
"""
import argparse
import hashlib
import json
from pathlib import Path
import sys


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.strip().split('\n')[0])
    ap.add_argument('repo', help='checkout or image root holding src/f1tenth_intelligence/llm')
    ap.add_argument('--installed-llm', action='store_true',
                    help='import llm from the sourced install, not from REPO/src')
    a = ap.parse_args(argv)
    repo = Path(a.repo).expanduser().resolve()
    llm_src = repo / 'src' / 'f1tenth_intelligence' / 'llm'
    if not a.installed_llm:
        sys.path.insert(0, str(llm_src))
    from llm.plan_translate import translate
    import llm.plan_translate as pt

    golden_dir = llm_src / 'test' / 'golden'
    n_ok = n_total = 0
    for path in sorted(golden_dir.glob('*.json')):
        case = json.loads(path.read_text())
        if 'mission' not in case:
            continue
        n_total += 1
        result = translate(case['intent'])
        produced = json.dumps(result.mission, sort_keys=True, separators=(',', ':'))
        equal = result.mission == case['mission']
        n_ok += equal
        sha = hashlib.sha256(produced.encode()).hexdigest()[:16]
        print('%-45s equal=%s sha256=%s' % (path.stem, equal, sha))
    print('plan_translate: %s' % pt.__file__)
    print('python %s: %d/%d golden missions reproduced exactly'
          % (sys.version.split()[0], n_ok, n_total))
    return 0 if n_total and n_ok == n_total else 1


if __name__ == '__main__':
    sys.exit(main())
