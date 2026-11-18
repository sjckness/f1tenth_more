#!/usr/bin/env python3
"""
Check that a Rust build did not change the resolved crates of Cargo.lock.

usage: cargo_lock_check.py <committed Cargo.lock> <Cargo.lock after the build>

The Orin image builds the ament_cargo packages with colcon-ros-cargo, which
writes a [patch.crates-io] section for every message crate it finds installed.
Cargo records the patches no member uses as [[patch.unused]] and rewrites
Cargo.lock with them, so that list depends on the machine's install prefix
(linus's lock lists linus's message packages). It does not change what is
compiled. The resolved [[package]] entries (name, version, source, checksum,
dependencies) do, and must equal the committed ones: then the image built
exactly the reviewed dependency set.

Exit 0: the packages are equal (a changed [[patch.unused]] list is printed as a
note). Exit 1: a package was added, removed or changed (each one printed).
Exit 2: usage or parse error.
"""

import sys

import tomllib  # stdlib since 3.11

PACKAGE_KEYS = ('name', 'version', 'source', 'checksum', 'dependencies')


def packages(lock):
    """Return {(name, version, source): entry} for the [[package]] entries."""
    out = {}
    for entry in lock.get('package', []):
        key = (entry.get('name'), entry.get('version'), entry.get('source'))
        out[key] = {k: entry.get(k) for k in PACKAGE_KEYS}
    return out


def unused_patches(lock):
    """Return the sorted (name, version) pairs of [[patch.unused]]."""
    return sorted((p.get('name'), p.get('version'))
                  for p in lock.get('patch', {}).get('unused', []))


def compare(before, after):
    """Return (errors, notes) for two parsed Cargo.lock documents."""
    errors, notes = [], []
    pb, pa = packages(before), packages(after)
    for key in sorted(set(pb) - set(pa), key=str):
        errors.append('removed: %s %s (%s)' % key)
    for key in sorted(set(pa) - set(pb), key=str):
        errors.append('added: %s %s (%s)' % key)
    for key in sorted(set(pb) & set(pa), key=str):
        if pb[key] != pa[key]:
            diff = [k for k in PACKAGE_KEYS if pb[key][k] != pa[key][k]]
            errors.append('changed: %s %s (%s): ' % key + ', '.join(diff))
    if before.get('version') != after.get('version'):
        errors.append('lock format version %s -> %s'
                      % (before.get('version'), after.get('version')))
    ub, ua = unused_patches(before), unused_patches(after)
    if ub != ua:
        notes.append('[[patch.unused]] differs (machine-specific, not compiled): '
                     '%d committed, %d after the build' % (len(ub), len(ua)))
    return errors, notes


def main(argv):
    """Compare the two lock files named in argv; return the exit code."""
    if len(argv) != 3:
        print(__doc__.strip().splitlines()[2], file=sys.stderr)
        return 2
    try:
        docs = []
        for path in argv[1:]:
            with open(path, 'rb') as f:
                docs.append(tomllib.load(f))
    except (OSError, tomllib.TOMLDecodeError) as exc:
        print('cargo_lock_check: %s' % exc, file=sys.stderr)
        return 2
    errors, notes = compare(*docs)
    for note in notes:
        print('cargo_lock_check: note: ' + note)
    for err in errors:
        print('cargo_lock_check: ' + err, file=sys.stderr)
    if errors:
        print('cargo_lock_check: FAILED, the build resolved other crates than the '
              'committed Cargo.lock', file=sys.stderr)
        return 1
    print('cargo_lock_check: %d packages, equal to the committed Cargo.lock'
          % len(packages(docs[1])))
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv))
