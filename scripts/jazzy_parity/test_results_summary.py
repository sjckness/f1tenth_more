#!/usr/bin/env python3
"""Per-package pass/skip/fail/error counts and failing test IDs from a
`colcon test --test-result-base DIR` tree (Phase 5d: the Thor reference at
083c8e8 and, in D3 #2, the Orin container run). Counts come from colcon's own
`colcon test-result --all` listing when it is given (the authoritative
totals); the failing/erroring/skipped test IDs come from the JUnit XML.

Runs on: anywhere (reads files only). P9.1: colcon_test_ids.sh calls it in the
harness container and on linus (the reference at the same commit);
parity_gates.py tests compares the two --ids files.

usage: test_results_summary.py RESULT_DIR [TEST_RESULT_ALL.txt] [--ids OUT.txt]
       test_results_summary.py --compare A_IDS.txt B_IDS.txt
"""
import argparse
import collections
import glob
import os
import re
import xml.etree.ElementTree as ET


def counts_from_colcon(path, base):
    tot = collections.defaultdict(lambda: [0, 0, 0, 0])
    pat = re.compile(r'^(\S+): (\d+) tests?, (\d+) errors?, (\d+) failures?, (\d+) skipped')
    for line in open(path):
        m = pat.match(line.strip())
        if not m or m.group(1) == 'Summary':  # colcon's own grand-total line
            continue
        rel = os.path.relpath(m.group(1), base) if os.path.isabs(m.group(1)) else m.group(1)
        parts = rel.split(os.sep)
        pkg = parts[parts.index('test_results') + 1] if 'test_results' in parts else parts[0]
        for i in range(4):
            tot[pkg][i] += int(m.group(i + 2))
    return tot


def ids_from_xml(base):
    ids = []
    for f in sorted(glob.glob(os.path.join(base, '**', '*.xml'), recursive=True)):
        pkg = os.path.relpath(f, base).split(os.sep)[0]
        try:
            root = ET.parse(f).getroot()
        except ET.ParseError:
            ids.append(('unparsable', pkg, os.path.relpath(f, base)))
            continue
        for tc in root.iter('testcase'):
            tid = f"{pkg}::{tc.get('classname', '')}::{tc.get('name', '')}"
            for kind in ('error', 'failure', 'skipped'):
                if tc.find(kind) is not None:
                    ids.append((kind, pkg, tid))
                    break
    return ids


ap = argparse.ArgumentParser()
ap.add_argument('result_dir', nargs='?')
ap.add_argument('colcon_all', nargs='?')
ap.add_argument('--ids')
ap.add_argument('--compare', nargs=2)
a = ap.parse_args()

if a.compare:
    A = set(open(a.compare[0]).read().split('\n')) - {''}
    B = set(open(a.compare[1]).read().split('\n')) - {''}
    print(f'only in {a.compare[0]}: {len(A - B)}')
    for x in sorted(A - B):
        print('  -', x)
    print(f'only in {a.compare[1]}: {len(B - A)}')
    for x in sorted(B - A):
        print('  +', x)
    raise SystemExit(0)

ids = ids_from_xml(a.result_dir)
if a.colcon_all:
    tot = counts_from_colcon(a.colcon_all, os.path.dirname(os.path.abspath(a.result_dir)))
else:
    tot = collections.defaultdict(lambda: [0, 0, 0, 0])
print('| Package | Tests | Pass | Errors | Failures | Skipped |')
print('|---|---:|---:|---:|---:|---:|')
for p in sorted(tot):
    n, e, f, s = tot[p]
    print(f'| {p} | {n} | {n - e - f - s} | {e} | {f} | {s} |')
T = [sum(t[i] for t in tot.values()) for i in range(4)]
print(f'| **total** | **{T[0]}** | **{T[0] - T[1] - T[2] - T[3]}** | **{T[1]}** | **{T[2]}** | **{T[3]}** |')
if a.ids:
    with open(a.ids, 'w') as out:
        for kind, pkg, tid in ids:
            out.write(f'{kind} {tid}\n')
