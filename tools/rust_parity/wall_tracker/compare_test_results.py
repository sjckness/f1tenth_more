#!/usr/bin/env python3
"""Compare two colcon --test-result-base trees test by test (R6: py against rust).

Usage:
    python3 tools/rust_parity/wall_tracker/compare_test_results.py PY_BASE RUST_BASE \
        [--json OUT]

Every <base>/<package>/pytest.xml (colcon's JUnit output) is read; a test ID is
<package>::<classname>::<name> and its outcome passed / failure / error /
skipped. The gate is "identical outcomes per test": the same ID set and the same
outcome for every ID. Prints the per-package totals and every difference; exit
1 on any.
"""

import argparse
import glob
import json
import os
import sys
import xml.etree.ElementTree as ET


def outcomes(base):
    out = {}
    for path in sorted(glob.glob(os.path.join(base, '*', 'pytest.xml'))):
        pkg = os.path.basename(os.path.dirname(path))
        for case in ET.parse(path).getroot().iter('testcase'):
            tid = f'{pkg}::{case.get("classname")}::{case.get("name")}'
            state = 'passed'
            for child in case:
                if child.tag in ('failure', 'error', 'skipped'):
                    state = child.tag
                    break
            out[tid] = state
    return out


def totals(res):
    t = {}
    for tid, state in res.items():
        pkg = tid.split('::', 1)[0]
        t.setdefault(pkg, {}).setdefault(state, 0)
        t[pkg][state] += 1
    return t


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('py_base')
    ap.add_argument('rust_base')
    ap.add_argument('--json', default=None)
    cfg = ap.parse_args()
    a, b = outcomes(cfg.py_base), outcomes(cfg.rust_base)
    diff = sorted((t, a.get(t), b.get(t)) for t in set(a) | set(b) if a.get(t) != b.get(t))
    ta, tb = totals(a), totals(b)
    for pkg in sorted(set(ta) | set(tb)):
        print(f'{pkg:32s} py {ta.get(pkg)}  rust {tb.get(pkg)}')
    print(f'{len(a)} py test IDs, {len(b)} rust test IDs, {len(diff)} differ')
    for t, x, y in diff:
        print(f'  DIFF {t}: py {x} rust {y}')
    if cfg.json:
        with open(cfg.json, 'w') as f:
            json.dump({'py': ta, 'rust': tb, 'n_py': len(a), 'n_rust': len(b),
                       'differences': diff,
                       'failing_py': sorted(t for t, s in a.items() if s in ('failure', 'error')),
                       'failing_rust': sorted(t for t, s in b.items()
                                              if s in ('failure', 'error'))}, f, indent=1)
    return 1 if diff else 0


if __name__ == '__main__':
    sys.exit(main())
