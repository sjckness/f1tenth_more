#!/usr/bin/env python3
"""Compare the test outcomes of the corridor suites across implementation modes.

    python3 junit_compare.py <out dir>      (the layout run_corridor_tests.sh writes)

<out dir>/<mode>/<package>.xml are pytest JUnit files, one per mode (py, rust,
substitute, capture). Every test ID must have the same outcome (passed /
failure / error / skipped) in every mode: the Rust corridor may not change what
any test sees. Writes <out dir>/outcomes.json and prints a table; exit 1 on
any difference.
"""

import json
from pathlib import Path
import sys
import xml.etree.ElementTree as ET


def outcomes(xml_path):
    out = {}
    for case in ET.parse(xml_path).getroot().iter('testcase'):
        tid = f'{case.get("classname")}::{case.get("name")}'
        state = 'passed'
        for child in case:
            if child.tag in ('failure', 'error', 'skipped'):
                state = child.tag
                break
        out[tid] = state
    return out


def main(argv):
    root = Path(argv[1])
    modes = sorted(p.name for p in root.iterdir() if p.is_dir())
    packages = sorted({x.stem for m in modes for x in (root / m).glob('*.xml')})
    table = {}
    differences = []
    for pkg in packages:
        per_mode = {}
        for mode in modes:
            path = root / mode / f'{pkg}.xml'
            per_mode[mode] = outcomes(path) if path.is_file() else None
        ref_mode = 'py' if 'py' in per_mode else modes[0]
        ref = per_mode[ref_mode] or {}
        row = {}
        for mode, res in per_mode.items():
            if res is None:
                row[mode] = None
                differences.append((pkg, mode, 'missing JUnit file'))
                continue
            counts = {}
            for state in res.values():
                counts[state] = counts.get(state, 0) + 1
            diff = sorted(t for t in set(ref) | set(res) if ref.get(t) != res.get(t))
            row[mode] = {'tests': len(res), 'counts': counts, 'differs_from_py': len(diff),
                         'failing': sorted(t for t, s in res.items()
                                           if s in ('failure', 'error'))}
            for t in diff:
                differences.append((pkg, mode, t, ref.get(t), res.get(t)))
        table[pkg] = row
    result = {'modes': modes, 'packages': table, 'differences': differences,
              'identical_outcomes': not differences}
    (root / 'outcomes.json').write_text(json.dumps(result, indent=1) + '\n')
    for pkg, row in table.items():
        for mode, r in row.items():
            if r is None:
                print(f'{pkg:16s} {mode:11s} MISSING')
            else:
                c = r['counts']
                print(f'{pkg:16s} {mode:11s} {r["tests"]:5d} tests  '
                      f'passed {c.get("passed", 0):5d}  '
                      f'failure {c.get("failure", 0):3d}  error {c.get("error", 0):3d}  '
                      f'skipped {c.get("skipped", 0):3d}  differs from py {r["differs_from_py"]}')
    print('IDENTICAL OUTCOMES' if not differences else f'{len(differences)} DIFFERENCES')
    for d in differences[:40]:
        print('  ', d)
    return 0 if not differences else 1


if __name__ == '__main__':
    sys.exit(main(sys.argv))
