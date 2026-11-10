#!/usr/bin/env python3
"""Fail if the Hydra submodules and the pinned .repos file disagree.

`src/f1tenth_hydra/config/hydra/hydra_jazzy.repos` stays the single source of truth for the
Hydra version. The submodules under `third_party/hydra_ros/` must point at exactly the commits
(and the URLs) it names: same set of repositories, same SHA, same remote.

    scripts/hydra/check_hydra_pins.py [--quiet]

Exit 0 when everything matches, 1 on any divergence (and the differences are printed).
Checks the commits recorded in the repository (the index), so it works on a clone whose
submodules were never initialized; for an initialized submodule the checked-out HEAD is
checked too, which catches a local commit or an accidental bump left in the working tree.
"""

import argparse
import os
import subprocess
import sys

import yaml

REPOS = 'src/f1tenth_hydra/config/hydra/hydra_jazzy.repos'
PREFIX = 'third_party/hydra_ros'


def _run(*args, cwd=None):
    out = subprocess.run(args, cwd=cwd, capture_output=True, text=True, check=True)
    return out.stdout.strip()


def repo_root():
    here = os.path.dirname(os.path.abspath(__file__))
    return _run('git', 'rev-parse', '--show-toplevel', cwd=here)


def pinned(root):
    """{name: (url, sha)} from the .repos file."""
    with open(os.path.join(root, REPOS)) as handle:
        data = yaml.safe_load(handle)
    return {name: (spec['url'], spec['version'])
            for name, spec in data['repositories'].items()}


def recorded(root):
    """{name: sha} gitlinks recorded for third_party/hydra_ros in the index."""
    out = _run('git', 'ls-files', '--stage', '--', PREFIX, cwd=root)
    found = {}
    for line in out.splitlines():
        meta, path = line.split('\t', 1)
        mode, sha, _stage = meta.split()
        if mode == '160000':
            found[os.path.basename(path)] = sha
    return found


def urls(root):
    """{name: url} from .gitmodules, for the paths under third_party/hydra_ros."""
    try:
        out = _run('git', 'config', '-f', '.gitmodules', '--get-regexp', r'submodule\..*\.url',
                   cwd=root)
    except subprocess.CalledProcessError:
        return {}
    found = {}
    for line in out.splitlines():
        key, url = line.split(' ', 1)
        name = key[len('submodule.'):-len('.url')]
        try:
            path = _run('git', 'config', '-f', '.gitmodules',
                        'submodule.{}.path'.format(name), cwd=root)
        except subprocess.CalledProcessError:
            continue
        if path.startswith(PREFIX + '/'):
            found[os.path.basename(path)] = url
    return found


def checked_out(root, name):
    """The submodule's HEAD, or None when it is not initialized."""
    path = os.path.join(root, PREFIX, name)
    if not os.path.exists(os.path.join(path, '.git')):
        return None
    try:
        return _run('git', 'rev-parse', 'HEAD', cwd=path)
    except subprocess.CalledProcessError:
        return None


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--quiet', action='store_true', help='print only the differences')
    args = parser.parse_args()

    root = repo_root()
    want, have, remotes = pinned(root), recorded(root), urls(root)
    problems = []

    for name in sorted(set(want) | set(have)):
        if name not in have:
            problems.append('{}: in {} but no submodule {}/{}'.format(name, REPOS, PREFIX, name))
            continue
        if name not in want:
            problems.append('{}: submodule {}/{} is not in {}'.format(name, PREFIX, name, REPOS))
            continue
        url, sha = want[name]
        if have[name] != sha:
            problems.append('{}: .repos {} != recorded {}'.format(name, sha[:12], have[name][:12]))
        if remotes.get(name) != url:
            problems.append('{}: .repos url {} != .gitmodules url {}'
                            .format(name, url, remotes.get(name)))
        live = checked_out(root, name)
        if live is not None and live != sha:
            problems.append('{}: working tree at {} != .repos {} (submodule not at its pin)'
                            .format(name, live[:12], sha[:12]))
        if not args.quiet and not problems:
            print('  {:<18} {} ok'.format(name, sha[:12]))

    if problems:
        print('hydra pins DIVERGE from {}:'.format(REPOS), file=sys.stderr)
        for line in problems:
            print('  ' + line, file=sys.stderr)
        print('fix: check out the .repos SHA in the submodule and commit it, or bump both '
              'together (docs/hydra_hil.md)', file=sys.stderr)
        return 1
    if not args.quiet:
        print('{} submodules match {}'.format(len(want), REPOS))
    return 0


if __name__ == '__main__':
    sys.exit(main())
