#!/usr/bin/env python3
"""
Check that the car's native tree carries nothing the container image lacks.

Runs on the Orin HOST (Python 3.10, no ROS), read-only (GIT_OPTIONAL_LOCKS=0):

    scripts/orin/drift_check.sh --car-sync <CAR_SYNC.json> [--ws ~/dev_ws/f1tenth_more]

<CAR_SYNC.json> is docker/orin/car_sync.json of the commit the image was built
from (in the image: /opt/f1tenth_ws/CAR_SYNC.json, also a label). It records
what of the native tree went into main: the car's commits beyond `base`, the
sha256 of the uncommitted diff under src/, and every untracked file under
src/ with its sha256 (or why it was left out). This script captures the same
three things exactly as tools/car_sync/capture_orin.sh does and compares:

  drift (exit 3)   a commit in base..HEAD that car_sync does not list; an
                   uncommitted src/ diff with another sha256 than recorded
                   (none recorded: any diff); an untracked file under src/
                   that is not recorded, or whose content changed
  warnings only    HEAD is not car_sync's native_head although every commit
                   is known (the car went back: the image has more); a
                   recorded untracked file the car no longer has

One JSON object on stdout either way. Exit 0 clean, 3 drift, 2 error (bad
arguments, no checkout, base unknown). Each new car change: capture, pick it
into main, a new car_sync.json, rebuild (docs: the plan's P1).
"""

import argparse
from datetime import datetime
import hashlib
import json
import os
import subprocess
import sys

EXIT_CLEAN, EXIT_ERROR, EXIT_DRIFT = 0, 2, 3
# capture_orin.sh hashes the diff file even when it is empty.
EMPTY_SHA256 = hashlib.sha256(b'').hexdigest()


class DriftError(Exception):
    """The check could not run (no checkout, bad car_sync, unknown base)."""


def _git(ws, *args, check=True) -> bytes:
    env = dict(os.environ, GIT_OPTIONAL_LOCKS='0')
    proc = subprocess.run(['git', '-C', ws, *args], capture_output=True, env=env)
    if check and proc.returncode != 0:
        raise DriftError(f'git {" ".join(args)}: '
                         f'{proc.stderr.decode(errors="replace").strip()}')
    return proc.stdout


def _sha256_file(path) -> str:
    h = hashlib.sha256()
    with open(path, 'rb') as f:
        for chunk in iter(lambda: f.read(1 << 20), b''):
            h.update(chunk)
    return h.hexdigest()


def capture(ws: str, base: str) -> dict:
    """Read the native tree the way capture_orin.sh does (read-only)."""
    if not os.path.isdir(ws):
        raise DriftError(f'{ws}: no such directory')
    head = _git(ws, 'rev-parse', 'HEAD').decode().strip()
    base_sha = _git(ws, 'rev-parse', '--verify', f'{base}^{{commit}}').decode().strip()
    log = _git(ws, 'log', '--format=%H %s', f'{base_sha}..HEAD').decode(errors='replace')
    commits = []
    for line in log.splitlines():
        sha, _, subject = line.partition(' ')
        commits.append({'sha': sha, 'subject': subject})
    diff = _git(ws, 'diff', 'HEAD', '--', 'src')
    names = _git(ws, 'diff', '--name-only', 'HEAD', '--', 'src').decode().split()
    listed = _git(ws, 'ls-files', '--others', '--exclude-standard', '-z', '--', 'src')
    untracked = {}
    for rel in (p.decode(errors='surrogateescape') for p in listed.split(b'\0') if p):
        path = os.path.join(ws, rel)
        untracked[rel] = _sha256_file(path) if os.path.isfile(path) else None
    return {'ws': ws, 'head': head,
            'branch': _git(ws, 'rev-parse', '--abbrev-ref', 'HEAD').decode().strip(),
            'base': base_sha, 'commits': commits,
            'src_diff_sha256': hashlib.sha256(diff).hexdigest() if diff else None,
            'src_diff_files': names, 'untracked': untracked}


def compare(car_sync: dict, native: dict) -> dict:
    """Return {'drift': [...], 'warnings': [...]} for a car_sync and a capture (pure)."""
    drift, warnings = [], []
    known = {c.get('car') for c in car_sync.get('commits') or []}
    for c in native['commits']:
        if c['sha'] not in known:
            drift.append({'kind': 'commit', 'sha': c['sha'], 'subject': c['subject']})
    if native['head'] != car_sync.get('native_head') and not any(
            d['kind'] == 'commit' for d in drift):
        warnings.append({'kind': 'head', 'head': native['head'],
                         'expected': car_sync.get('native_head'),
                         'why': 'HEAD is not the captured one, but every commit is known: '
                                'the image has commits the car does not run'})
    expected = car_sync.get('uncommitted_src_patch_sha256') or None
    if expected == EMPTY_SHA256:
        expected = None
    if native['src_diff_sha256'] != expected:
        drift.append({'kind': 'src_diff', 'sha256': native['src_diff_sha256'],
                      'expected': expected, 'files': native['src_diff_files']})
    shipped = car_sync.get('untracked') or {}
    excluded = {k: (v.get('sha256') if isinstance(v, dict) else v)
                for k, v in (car_sync.get('untracked_excluded') or {}).items()}
    for path, sha in sorted(native['untracked'].items()):
        known_sha = shipped.get(path, excluded.get(path))
        if path not in shipped and path not in excluded:
            drift.append({'kind': 'untracked', 'path': path, 'sha256': sha,
                          'why': 'not in car_sync'})
        elif sha != known_sha:
            drift.append({'kind': 'untracked', 'path': path, 'sha256': sha,
                          'expected': known_sha, 'why': 'content changed'})
    for path in sorted(set(shipped) - set(native['untracked'])):
        warnings.append({'kind': 'untracked_missing', 'path': path,
                         'why': 'in the image, no longer untracked on the car '
                                '(deleted, or committed: then a commit drifts)'})
    return {'drift': drift, 'warnings': warnings}


def check(car_sync_path: str, ws: str) -> dict:
    try:
        with open(car_sync_path) as f:
            car_sync = json.load(f)
    except (OSError, ValueError) as exc:
        raise DriftError(f'{car_sync_path}: {exc}') from exc
    if not isinstance(car_sync, dict) or not car_sync.get('base'):
        raise DriftError(f'{car_sync_path}: no `base` commit: not a car_sync.json')
    native = capture(ws, car_sync['base'])
    result = compare(car_sync, native)
    return {
        'ok': not result['drift'], **result,
        'native': {k: native[k] for k in ('ws', 'head', 'branch', 'base', 'src_diff_sha256')},
        'native_untracked': len(native['untracked']),
        'car_sync': {'path': os.path.abspath(car_sync_path),
                     'captured_at': car_sync.get('captured_at'),
                     'native_head': car_sync.get('native_head'),
                     'main_commit': car_sync.get('main_commit')},
        'checked_at': datetime.now().astimezone().isoformat(timespec='seconds'),
    }


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    ap.add_argument('--car-sync', required=True, help='CAR_SYNC.json of the image')
    ap.add_argument('--ws', default=os.path.expanduser('~/dev_ws/f1tenth_more'),
                    help='the native checkout (default ~/dev_ws/f1tenth_more)')
    args = ap.parse_args(argv)
    try:
        out = check(args.car_sync, os.path.expanduser(args.ws))
    except DriftError as exc:
        print(json.dumps({'ok': False, 'error': str(exc)}))
        return EXIT_ERROR
    print(json.dumps(out))
    return EXIT_CLEAN if out['ok'] else EXIT_DRIFT


if __name__ == '__main__':
    sys.exit(main())
