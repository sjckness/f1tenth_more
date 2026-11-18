#!/usr/bin/env python3
"""Versioni dei moduli (package ROS 2 nostri) a un ref git, o confronto tra due elenchi. Solo stdlib.

    modules.py ref REF                    "nome versione percorso" per ogni modulo al ref REF
    modules.py diff A B                   righe "nome a b" dove le versioni differiscono; A e B sono
                                          file "nome versione" oppure ref:REF
    modules.py json A                     l'elenco A come oggetto JSON {nome: versione}

Un modulo e' una cartella con package.xml sotto src/, esclusi i percorsi che corrispondono a
F1T_MODULES_EXCLUDE_RE (submodule esterni). La radice del repo e' F1T_ROOT (o la cwd).
"""
import json
import os
import re
import subprocess
import sys

ROOT = os.environ.get('F1T_ROOT') or os.getcwd()
EXCLUDE = re.compile(os.environ.get('F1T_MODULES_EXCLUDE_RE')
                     or r'/(f1tenth_external|vesc|husarion_gz_worlds)/')
TAG = re.compile(r'<(name|version)>\s*([^<\s]+)\s*</\1>')


def git(*args):
    return subprocess.run(['git', '-C', ROOT, *args], check=True, capture_output=True,
                          text=True).stdout


def at_ref(ref):
    """{nome: (versione, cartella)} al ref."""
    out = {}
    paths = [p for p in git('ls-tree', '-r', '--name-only', ref, '--', 'src').splitlines()
             if p.endswith('/package.xml') and not EXCLUDE.search('/' + p)]
    if not paths:
        return out
    # Un solo processo git per tutti i blob.
    spec = ''.join(f'{ref}:{p}\n' for p in paths)
    blob = subprocess.run(['git', '-C', ROOT, 'cat-file', '--batch'], input=spec, check=True,
                          capture_output=True, text=True).stdout
    pos = 0
    for p in paths:
        header_end = blob.index('\n', pos)
        size = int(blob[pos:header_end].split()[2])
        body = blob[header_end + 1:header_end + 1 + size]
        pos = header_end + 1 + size + 1
        tags = dict(TAG.findall(body))
        if 'name' in tags:
            out[tags['name']] = (tags.get('version', '?'), os.path.dirname(p))
    return out


def load(spec):
    if spec.startswith('ref:'):
        return {k: v[0] for k, v in at_ref(spec[4:]).items()}
    out = {}
    with open(spec, encoding='utf-8') as fh:
        for line in fh:
            parts = line.split()
            if len(parts) >= 2:
                out[parts[0]] = parts[1]
    return out


def main(argv):
    if len(argv) >= 2 and argv[0] == 'ref':
        for name, (ver, d) in sorted(at_ref(argv[1]).items()):
            print(name, ver, d)
        return 0
    if len(argv) >= 3 and argv[0] == 'diff':
        a, b = load(argv[1]), load(argv[2])
        for name in sorted(set(a) | set(b)):
            if a.get(name) != b.get(name):
                print(name, a.get(name, '-'), b.get(name, '-'))
        return 0
    if len(argv) >= 2 and argv[0] == 'json':
        print(json.dumps(load(argv[1]), sort_keys=True))
        return 0
    print(__doc__, file=sys.stderr)
    return 2


if __name__ == '__main__':
    try:
        sys.exit(main(sys.argv[1:]))
    except subprocess.CalledProcessError as e:
        print(f'git: {e.stderr.strip()}', file=sys.stderr)
        sys.exit(1)
