#!/usr/bin/env python3
"""
Check `docker diff f1tenth_car` against the list of intended changes.

Runs on: the Orin host (reads `docker diff` output; stdlib only). Plan P8:
"`docker diff` after a 10 min run lists only intended paths". Everything the
stack writes should land on a volume or a bind mount (not in `docker diff`);
what remains in the container's writable layer must match a glob of the
allow list (docker_diff_allow.txt next to this file, one fnmatch glob per
line, `#` comments). A directory that only shows as changed (C) because
something below it changed is accepted when an allowed entry of the same
diff lies beneath it. Bind-mount and volume contents never show; an entry
below a mount point means that mount is missing (see the allow file).

  docker_diff_check.py DIFF.txt [--allow FILE] [--out FILE.json]
  docker diff f1tenth_car | docker_diff_check.py - [--allow FILE]

Exit 0 when every entry is allowed. A first run on a new image: review each
violation; extend the allow list only with paths that are meant to change.
"""
import argparse
import fnmatch
import json
import os
import sys

DEFAULT_ALLOW = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'docker_diff_allow.txt')


def parse_diff(text):
    """[(kind, path)] from `docker diff` lines ('A /x', 'C /y', 'D /z')."""
    out = []
    for line in text.splitlines():
        parts = line.strip().split(None, 1)
        if len(parts) == 2 and parts[0] in ('A', 'C', 'D'):
            out.append((parts[0], parts[1]))
    return out


def read_allow(path):
    with open(path) as f:
        return [line.strip() for line in f if line.strip() and not line.startswith('#')]


def matches(path, globs):
    """Tell whether path matches one of the globs."""
    return any(fnmatch.fnmatch(path, g) for g in globs)


def check(entries, globs):
    """
    Split entries into (allowed entries, violations), in input order.

    Deepest paths first, so that a changed (C) parent is decided after
    everything below it: it passes if it matches a glob itself or an allowed
    entry lies beneath it.
    """
    verdict = {}
    passed = []
    for i in sorted(range(len(entries)), key=lambda k: -entries[k][1].rstrip('/').count('/')):
        kind, path = entries[i]
        ok = matches(path, globs)
        if not ok and kind == 'C':
            base = path.rstrip('/') + '/'
            ok = any(p.startswith(base) for p in passed)
        verdict[i] = ok
        if ok:
            passed.append(path)
    ok, bad = [], []
    for i, (kind, path) in enumerate(entries):
        (ok if verdict[i] else bad).append('%s %s' % (kind, path))
    return ok, bad


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.strip().split('\n')[0],
                                 formatter_class=argparse.RawDescriptionHelpFormatter,
                                 epilog=__doc__[__doc__.index('  docker_diff_check.py'):])
    ap.add_argument('diff', help="`docker diff` output file, or '-' for stdin")
    ap.add_argument('--allow', default=DEFAULT_ALLOW)
    ap.add_argument('--out')
    a = ap.parse_args(argv)
    text = sys.stdin.read() if a.diff == '-' else open(a.diff).read()
    entries = parse_diff(text)
    ok, bad = check(entries, read_allow(a.allow))
    res = {'entries': len(entries), 'allowed': len(ok), 'violations': bad,
           'result': 'PASS' if not bad else 'FAIL'}
    if a.out:
        with open(a.out, 'w') as f:
            json.dump(res, f, indent=1)
    print('%s docker diff: %d entries, %d allowed, %d not in %s'
          % (res['result'], len(entries), len(ok), len(bad), a.allow))
    for v in bad[:50]:
        print('  ' + v)
    return 0 if not bad else 1


if __name__ == '__main__':
    sys.exit(main())
