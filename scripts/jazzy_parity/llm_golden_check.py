#!/usr/bin/env python3
"""Phase 4: translate every golden intent fixture and report, per fixture,
whether translate(intent) equals the committed mission, plus a sha256 of the
canonical JSON of what it produced.

plan_translate is pure Python (jsonschema + stdlib), so this runs with only
the source tree on sys.path -- no ROS, no install space. Running it under two
interpreters (Jazzy/Python 3.12 on Thor, the Humble/Python 3.10 container)
and diffing the output is a direct cross-version check of the translation
path, independent of any llama-server.

Usage: llm_golden_check.py REPO
"""
import hashlib
import json
import sys
from pathlib import Path

repo = Path(sys.argv[1]).resolve()
sys.path.insert(0, str(repo / 'src' / 'f1tenth_intelligence' / 'llm'))
sys.path.insert(0, str(repo / 'src' / 'f1tenth_params'))
from llm.plan_translate import translate  # noqa: E402

golden_dir = repo / 'src' / 'f1tenth_intelligence' / 'llm' / 'test' / 'golden'
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
    print('%-45s equal=%s sha256=%s' % (path.stem, equal,
                                         hashlib.sha256(produced.encode()).hexdigest()[:16]))
print('python %s: %d/%d golden missions reproduced exactly' % (sys.version.split()[0], n_ok, n_total))
