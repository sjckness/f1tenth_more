#!/usr/bin/env python3
"""Phase 4: compare two mission_extract outputs (*.extract.parquet) of the SAME
bag -- e.g. extracted on the Orin (Humble, e47e646) and on Thor (Jazzy).

The file bytes are not compared: parquet embeds writer metadata (library
versions). What is compared is the content: schema (names + types), row count,
and every column, cell by cell -- exact equality, with NaN == NaN. Float
columns that differ also report their max abs difference. Table-level
key/value metadata is listed separately (run_id, manifest-derived fields).

Usage: compare_extracts.py A.extract.parquet B.extract.parquet
"""
import sys

import numpy as np
import pyarrow.parquet as pq


def main():
    a = pq.read_table(sys.argv[1])
    b = pq.read_table(sys.argv[2])
    ok = True
    if a.schema.remove_metadata() != b.schema.remove_metadata():
        ok = False
        print('SCHEMA DIFFERS')
        print(a.schema.remove_metadata())
        print(b.schema.remove_metadata())
    print('rows: %d vs %d' % (a.num_rows, b.num_rows))
    ok &= a.num_rows == b.num_rows
    for name in a.column_names:
        if name not in b.column_names:
            continue
        ca, cb = a.column(name).to_pylist(), b.column(name).to_pylist()
        if ca == cb:
            continue
        same = len(ca) == len(cb) and all(
            (x == y) or (isinstance(x, float) and isinstance(y, float) and np.isnan(x) and np.isnan(y))
            for x, y in zip(ca, cb))
        if same:
            continue
        ok = False
        try:
            d = np.nanmax(np.abs(np.array(ca, dtype=float) - np.array(cb, dtype=float)))
            print('column %-30s DIFFERS  max|d|=%g' % (name, d))
        except (TypeError, ValueError):
            n = sum(1 for x, y in zip(ca, cb) if x != y)
            print('column %-30s DIFFERS  %d cells' % (name, n))
    ma = {k: v for k, v in (a.schema.metadata or {}).items() if not k.startswith(b'ARROW')}
    mb = {k: v for k, v in (b.schema.metadata or {}).items() if not k.startswith(b'ARROW')}
    for k in sorted(set(ma) | set(mb)):
        if ma.get(k) != mb.get(k):
            print('metadata %s differs (%d vs %d bytes)' % (k.decode(errors='replace'),
                                                           len(ma.get(k, b'')), len(mb.get(k, b''))))
    print('RESULT %s' % ('IDENTICAL CONTENT' if ok else 'DIFFERENT'))
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
