#!/usr/bin/env python3
# desc: cos'e' una versione di un modulo: CHANGELOG, analisi che la citano, parametri cambiati
# alias: whatis
"""f1t whatis <package> [versione] | f1t whatis <package> --deployed

Stampa in pochi secondi (solo stdlib, legge il checkout corrente):
  1. la voce del CHANGELOG.md del modulo per quella versione e per le 2 precedenti;
  2. i report in docs/analysis/ il cui front matter `modules:` cita il modulo, prima quelli su
     quella versione: id, stato, risultato e decisione su una riga;
  3. i parametri (yaml sotto src/) cambiati tra il tag della versione precedente e quello della
     versione chiesta: git diff <pkg>/vA <pkg>/vB.
Senza versione: quella di package.xml. --deployed: la versione che gira sull'Orin (letta dal
container via ssh, come f1t orin status).
"""
import argparse
import glob
import importlib.util
import os
import re
import subprocess
import sys

CORE = os.environ.get('F1T_CORE') or os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
ROOT = os.environ.get('F1T_ROOT') or os.path.abspath(os.path.join(CORE, '..', '..'))
EXCLUDE = re.compile(os.environ.get('F1T_MODULES_EXCLUDE_RE')
                     or r'/(f1tenth_external|vesc|husarion_gz_worlds)/')
HEAD = re.compile(r'^## \[([^\]]+)\](.*)$')


def load_index_module():
    spec = importlib.util.spec_from_file_location('f1t_index', os.path.join(CORE, 'analysis', 'index.py'))
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def git(*args, check=True):
    r = subprocess.run(['git', '-C', ROOT, *args], capture_output=True, text=True)
    if check and r.returncode:
        raise RuntimeError(r.stderr.strip())
    return r.stdout


def find_package(name):
    for xml in glob.glob(os.path.join(ROOT, 'src', '**', 'package.xml'), recursive=True):
        rel = os.path.relpath(xml, ROOT)
        if EXCLUDE.search('/' + rel):
            continue
        text = open(xml, encoding='utf-8').read()
        m = re.search(r'<name>\s*([^<\s]+)\s*</name>', text)
        if m and m.group(1) == name:
            v = re.search(r'<version>\s*([^<\s]+)\s*</version>', text)
            return os.path.dirname(rel), v.group(1) if v else '?'
    return None, None


def changelog_sections(path):
    """[(versione, intestazione, righe)] in ordine di file (dalla piu' recente)."""
    out, cur = [], None
    if not os.path.exists(path):
        return out
    for line in open(path, encoding='utf-8'):
        m = HEAD.match(line.rstrip('\n'))
        if m:
            cur = (m.group(1), line.rstrip('\n'), [])
            out.append(cur)
        elif cur is not None:
            cur[2].append(line.rstrip('\n'))
    return [s for s in out if s[0].lower() != 'unreleased']


def deployed_version(pkg):
    cfg = subprocess.run(
        ['bash', '-c', 'source "$F1T_CORE/lib/config.sh"; cfg_load; '
         'echo "$F1T_ORIN_USER@$F1T_ORIN_HOST"; echo "$F1T_ORIN_CONTAINER"; echo "$F1T_ORIN_SSH_OPTS"'],
        capture_output=True, text=True, env={**os.environ, 'F1T_CORE': CORE, 'F1T_ROOT': ROOT,
                                              'F1T_LOG_DIR': os.devnull})
    lines = cfg.stdout.splitlines()
    if len(lines) < 3:
        raise RuntimeError(f'config non letta: {cfg.stderr.strip()}')
    target, ctr, opts = lines[0], lines[1], lines[2].split()
    script = os.path.join(CORE, 'orin', 'remote', 'modules.sh')
    with open(script) as fh:
        r = subprocess.run(['ssh', *opts, target, 'bash -s -- --running %s %s' % (
            ctr, "'" + EXCLUDE.pattern + "'")], stdin=fh, capture_output=True, text=True, timeout=60)
    if r.returncode:
        raise RuntimeError(f'Orin: {r.stderr.strip() or "nessuna risposta"}')
    for line in r.stdout.splitlines():
        p = line.split()
        if len(p) >= 2 and p[0] == pkg:
            return p[1]
    raise RuntimeError(f'{pkg} non e\' nel container dell\'Orin')


def one(s, n=110):
    s = ' '.join(str(s or '').split())
    return s if len(s) <= n else s[:n - 3] + '...'


def main():
    ap = argparse.ArgumentParser(prog='f1t whatis', description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('package')
    ap.add_argument('version', nargs='?')
    ap.add_argument('--deployed', action='store_true', help='la versione che gira sull\'Orin')
    ap.add_argument('--max-diff', type=int, default=60, help='righe massime del diff dei parametri')
    a = ap.parse_args()

    pdir, current = find_package(a.package)
    if not pdir:
        print(f'modulo sconosciuto: {a.package} (package.xml sotto src/, esclusi i submodule)', file=sys.stderr)
        return 2
    version = a.version or current
    if a.deployed:
        try:
            version = deployed_version(a.package)
        except (RuntimeError, subprocess.TimeoutExpired) as e:
            print(f'--deployed: {e}', file=sys.stderr)
            return 1
    print(f'== {a.package} {version}   ({pdir}, package.xml {current}'
          f'{", sull Orin" if a.deployed else ""})')

    # 1. CHANGELOG
    secs = changelog_sections(os.path.join(ROOT, pdir, 'CHANGELOG.md'))
    idx = next((i for i, s in enumerate(secs) if s[0] == version), None)
    print('\n-- CHANGELOG')
    if not secs:
        print(f'   nessun {pdir}/CHANGELOG.md')
    elif idx is None:
        print(f'   la versione {version} non ha una voce (versioni: {", ".join(s[0] for s in secs[:6])})')
    else:
        for ver, head, body in secs[idx:idx + 3]:
            print(head)
            for line in [x for x in body if x.strip()][:14]:
                print('   ' + one(line, 150))
    prev = secs[idx + 1][0] if idx is not None and idx + 1 < len(secs) else None

    # 2. analisi che citano il modulo
    print('\n-- Analisi (docs/analysis, modules:)')
    try:
        ix = load_index_module()
        hits = []
        for rep in glob.glob(os.path.join(ROOT, 'docs', 'analysis', '*', '*', 'report.md')):
            fm = ix.parse_front_matter(open(rep, encoding='utf-8').read()) or {}
            mods = fm.get('modules') or {}
            if isinstance(mods, dict) and a.package in mods:
                hits.append((str(mods[a.package]) == version, str(mods[a.package]), fm, os.path.relpath(rep, ROOT)))
    except Exception as e:  # noqa: BLE001 - un front matter rotto non deve fermare il resto
        hits = []
        print(f'   errore leggendo i report: {e}')
    hits.sort(key=lambda h: str(h[2].get('date') or h[2].get('id') or ''), reverse=True)
    hits.sort(key=lambda h: not h[0])  # stabile: prima questa versione, poi le altre, dalle piu' recenti
    if not hits:
        print('   nessun report cita questo modulo')
    for same, ver, fm, rel in hits[:12]:
        mark = '*' if same else ' '
        print(f' {mark} {fm.get("id", "?")} [{ver}, {fm.get("status", "?")}] {one(fm.get("title"), 70)}')
        print(f'     risultato: {one(fm.get("result"))}')
        print(f'     decisione: {one(fm.get("decision"))}')
    if len(hits) > 12:
        print(f'   ... altri {len(hits) - 12} (python3 tools/core/analysis/index.py, docs/analysis/INDEX.csv)')
    print('   (* = cita proprio questa versione)')

    # 3. parametri tra le versioni
    print('\n-- Parametri (yaml sotto src/) tra le versioni')
    tag_b = f'{a.package}/v{version}'
    tag_a = f'{a.package}/v{prev}' if prev else None
    have = set(git('tag', '-l', f'{a.package}/v*').split())
    if tag_b not in have:
        print(f'   tag {tag_b} assente (tag: {" ".join(sorted(have)) or "nessuno"})')
    elif not tag_a or tag_a not in have:
        print(f'   nessun tag della versione precedente ({tag_a or "nessuna versione precedente"}): niente da confrontare')
    else:
        files = [f for f in git('diff', '--name-only', tag_a, tag_b, '--', 'src/*.yaml', 'src/*.yml').split()
                 if not EXCLUDE.search('/' + f)]
        if not files:
            print(f'   nessun yaml cambiato tra {tag_a} e {tag_b}')
        else:
            print(git('diff', '--stat=100', tag_a, tag_b, '--', *files).rstrip())
            diff = git('diff', '--no-color', '-U0', tag_a, tag_b, '--', *files).splitlines()
            for line in diff[:a.max_diff]:
                print('   ' + line)
            if len(diff) > a.max_diff:
                print(f'   ... {len(diff) - a.max_diff} righe in piu\': git diff {tag_a} {tag_b} -- {" ".join(files)}')
    return 0


if __name__ == '__main__':
    try:
        sys.exit(main())
    except RuntimeError as e:
        print(f'errore: {e}', file=sys.stderr)
        sys.exit(1)
