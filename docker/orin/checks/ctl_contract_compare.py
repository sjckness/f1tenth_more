#!/usr/bin/env python3
"""
Compare two ctl_contract.sh runs: native f1tenth_ctl answers vs the container's.

Runs on: anywhere (stdlib; reads the two run directories). Plan P8: "JSON +
exit codes = native". The bot and the gateway read these answers, so per
step (matched by name; the repeated `status_wait` polls are skipped):
  - the exit codes are equal;
  - both answers parse as JSON (or neither does);
  - the key structure is equal (dicts recursively, the first element of a
    list of dicts), apart from keys that name backend or timing details
    (BACKEND_KEYS: unit, pid, since, age_s, ...);
  - the semantic values are equal: ok, ready, estop_latched, the stack's and
    the discovery server's state, whether the probe ran, whether there is an
    error.

  ctl_contract_compare.py NATIVE_DIR CONTAINER_DIR [--out FILE.json]
Exit 0 when every common step agrees and no step is missing on one side.
"""
import argparse
import glob
import json
import os
import re
import sys

SKIP_STEPS = {'status_wait'}
BACKEND_KEYS = {'unit', 'pid', 'since', 'age_s', 'listened_s', 'warming_up', 'seen', 'previous',
                'result', 'exit_status', 'signal', 'log', 'path', 'started', 'elapsed_s',
                'duration_s', 't', 'time', 'stamp', 'note', 'error'}
SEMANTIC = ('ok', 'ready', 'estop_latched', 'processes.stack.state',
            'processes.discovery.state', 'probe.ran')


def load(run):
    """{step name: (rc, parsed JSON or None, raw text)}; last one wins per name."""
    out = {}
    for f in sorted(glob.glob(os.path.join(run, '[0-9][0-9]_*.json'))):
        name = re.sub(r'^\d+_', '', os.path.basename(f)[:-5])
        try:
            with open(f[:-5] + '.rc') as fh:
                rc = int(fh.read().strip() or -1)
        except (OSError, ValueError):
            rc = None
        with open(f) as fh:
            raw = fh.read()
        try:
            data = json.loads(raw)
        except ValueError:
            data = None
        out[name] = (rc, data, raw)
    return out


def key_paths(d, prefix=''):
    """Set of dotted key paths of a JSON value (backend / timing keys left out)."""
    out = set()
    if isinstance(d, dict):
        for k, v in d.items():
            if k in BACKEND_KEYS:
                continue
            p = prefix + k
            out.add(p)
            out |= key_paths(v, p + '.')
    elif isinstance(d, list) and d and isinstance(d[0], dict):
        out |= key_paths(d[0], prefix + '[].')
    return out


def get(d, dotted):
    for k in dotted.split('.'):
        d = d.get(k) if isinstance(d, dict) else None
    return d


def compare_step(a, b):
    """Problems of one step: (rc, json, raw) pairs."""
    bad = []
    if a[0] != b[0]:
        bad.append('exit code %s vs %s' % (a[0], b[0]))
    if (a[1] is None) != (b[1] is None):
        bad.append('JSON on one side only')
        return bad
    if a[1] is None:
        return bad
    ka, kb = key_paths(a[1]), key_paths(b[1])
    if ka - kb:
        bad.append('keys only native: %s' % sorted(ka - kb)[:12])
    if kb - ka:
        bad.append('keys only container: %s' % sorted(kb - ka)[:12])
    for path in SEMANTIC:
        va, vb = get(a[1], path), get(b[1], path)
        if va != vb:
            bad.append('%s: %r vs %r' % (path, va, vb))
    ea = isinstance(a[1], dict) and 'error' in a[1]
    eb = isinstance(b[1], dict) and 'error' in b[1]
    if ea != eb:
        bad.append('error on one side only: %r / %r' % (get(a[1], 'error'), get(b[1], 'error')))
    return bad


def compare(native, container):
    a, b = load(native), load(container)
    res = {'steps': {}, 'only_native': sorted(set(a) - set(b) - SKIP_STEPS),
           'only_container': sorted(set(b) - set(a) - SKIP_STEPS)}
    for name in sorted(set(a) & set(b) - SKIP_STEPS):
        res['steps'][name] = compare_step(a[name], b[name])
    res['ok'] = (not res['only_native'] and not res['only_container']
                 and not any(res['steps'].values()))
    return res


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.strip().split('\n')[0],
                                 formatter_class=argparse.RawDescriptionHelpFormatter,
                                 epilog=__doc__[__doc__.index('  ctl_contract_compare.py'):])
    ap.add_argument('native')
    ap.add_argument('container')
    ap.add_argument('--out')
    a = ap.parse_args(argv)
    res = compare(a.native, a.container)
    if a.out:
        with open(a.out, 'w') as f:
            json.dump(res, f, indent=1)
    for name, bad in res['steps'].items():
        detail = (': ' + '; '.join(bad)) if bad else ''
        print('%s %s%s' % ('FAIL' if bad else 'PASS', name, detail))
    for side in ('only_native', 'only_container'):
        if res[side]:
            print('FAIL steps %s: %s' % (side.replace('_', ' '), res[side]))
    print('RESULT %s' % ('PASS' if res['ok'] else 'FAIL'))
    return 0 if res['ok'] else 1


if __name__ == '__main__':
    sys.exit(main())
