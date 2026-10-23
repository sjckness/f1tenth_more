#!/usr/bin/env python3
"""List the failing test IDs of a colcon --test-result-base and compare them with a baseline.

IDs in the format of output/jazzy_main/test_failing_ids.txt
("<kind> <package>::<classname>::<name>", kind failure/error/skipped); failures and
errors are compared with that baseline.

Usage: python3 tools/r3_failing_ids.py <test-result-base> [baseline] [--out ids.txt]
Exit 1 when a failure/error ID is not in the baseline.
"""
import argparse
import glob
import os
import sys
import xml.etree.ElementTree as ET


def ids(base):
    out, totals = [], {}
    for path in sorted(glob.glob(os.path.join(base, '*', 'pytest.xml'))):
        pkg = os.path.basename(os.path.dirname(path))
        n = f = e = s = 0
        for tc in ET.parse(path).getroot().iter('testcase'):
            n += 1
            kind = None
            if tc.find('failure') is not None:
                kind, f = 'failure', f + 1
            elif tc.find('error') is not None:
                kind, e = 'error', e + 1
            elif tc.find('skipped') is not None:
                kind, s = 'skipped', s + 1
            if kind:
                out.append(f"{kind} {pkg}::{tc.get('classname', '')}::{tc.get('name', '')}")
        totals[pkg] = (n, n - f - e - s, f, e, s)
    return out, totals


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('base')
    ap.add_argument('baseline', nargs='?', default='output/jazzy_main/test_failing_ids.txt')
    ap.add_argument('--out')
    a = ap.parse_args()
    got, totals = ids(a.base)
    if a.out:
        with open(a.out, 'w') as fh:
            fh.write('\n'.join(got) + '\n')
    with open(a.baseline) as fh:
        base = {line.strip() for line in fh if line.strip()}
    print('| Package | Tests | Pass | Failures | Errors | Skipped |')
    print('|---|---:|---:|---:|---:|---:|')
    for pkg, (n, p, f, e, s) in totals.items():
        print(f'| {pkg} | {n} | {p} | {f} | {e} | {s} |')
    bad = [i for i in got if not i.startswith('skipped')]
    new = [i for i in bad if i not in base]
    pkgs = set(totals)
    gone = sorted(i for i in base if not i.startswith('skipped')
                  and i.split(' ', 1)[1].split('::', 1)[0] in pkgs and i not in bad)
    print(f'\nfailures+errors: {len(bad)}; NEW (not in baseline): {len(new)}')
    for i in new:
        print('  NEW', i)
    print(f'baseline failures of these packages that now pass: {len(gone)}')
    for i in gone:
        print('  FIXED', i)
    return 1 if new else 0


if __name__ == '__main__':
    sys.exit(main())
