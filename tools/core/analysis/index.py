#!/usr/bin/env python3
# desc: rigenera gli indici del database analisi (INDEX.csv, tabelle d'area, docs/modules) [--check]
# alias: exp index
"""Rigenera gli indici del database analisi e la tabella dei moduli.

Legge il front matter YAML di ogni docs/analysis/<area>/<id>/report.md e:

- riscrive docs/analysis/INDEX.csv (tutti gli esperimenti);
- riscrive, in ogni README d'area, la tabella tra <!-- AUTO:START --> e <!-- AUTO:END -->;
- riscrive la tabella delle aree in docs/analysis/README.md (stessi marker);
- riscrive docs/modules/README.md: modulo, versione, ultimo cambio, aree collegate, CHANGELOG;
- segnala front matter incompleti e report che citano un modulo o una versione inesistente
  (la versione deve essere quella di package.xml o un titolo `## [x.y.z]` del CHANGELOG).

Idempotente: un file viene riscritto solo se il contenuto cambia.
Solo stdlib, piu' PyYAML se installato (senza, usa un parser minimale per il front matter).

    f1t exp index             # rigenera (= python3 tools/core/analysis/index.py)
    f1t exp index --check     # rigenera, esce con 1 se ci sono segnalazioni

(tools/analysis_index.py resta come nome vecchio, con un avviso.)
"""
import argparse
import csv
import io
import json
import os
import re
import subprocess
import sys

try:
    import yaml
except ImportError:  # pragma: no cover - fallback parser below
    yaml = None

ROOT = os.environ.get('F1T_ROOT') or os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..', '..'))
ANALYSIS = os.path.join(ROOT, 'docs', 'analysis')
MODULES_README = os.path.join(ROOT, 'docs', 'modules', 'README.md')
REQUIRED = ('id', 'title', 'area', 'tags', 'modules', 'branch', 'commit', 'status',
            'hypothesis', 'result', 'decision', 'raw_data')
STATUSES = ('open', 'done', 'abandoned')
START, END = '<!-- AUTO:START -->', '<!-- AUTO:END -->'
UNSORTED = '_unsorted'

warnings = []


def warn(msg):
    warnings.append(msg)


# ------------------------------------------------------------------ front matter
def _scalar(v):
    v = v.strip()
    if len(v) >= 2 and v[0] == v[-1] == '"':
        return json.loads(v)
    if len(v) >= 2 and v[0] == v[-1] == "'":
        return v[1:-1].replace("''", "'")
    return v


def _split_flow(s):
    out, depth, cur, quote = [], 0, '', None
    for ch in s:
        if quote:
            cur += ch
            if ch == quote:
                quote = None
            continue
        if ch in '"\'':
            quote = ch
        elif ch in '[{':
            depth += 1
        elif ch in ']}':
            depth -= 1
        elif ch == ',' and depth == 0:
            out.append(cur)
            cur = ''
            continue
        cur += ch
    if cur.strip():
        out.append(cur)
    return out


def _value(v):
    v = v.strip()
    if v.startswith('[') and v.endswith(']'):
        return [_scalar(x) for x in _split_flow(v[1:-1])]
    if v.startswith('{') and v.endswith('}'):
        d = {}
        for item in _split_flow(v[1:-1]):
            k, _, val = item.partition(':')
            d[_scalar(k)] = _scalar(val)
        return d
    return _scalar(v)


def parse_front_matter(text):
    if not text.startswith('---\n'):
        return None
    end = text.find('\n---\n', 4)
    if end < 0:
        return None
    block = text[4:end]
    if yaml is not None:
        data = yaml.safe_load(block) or {}
    else:
        data = {}
        for line in block.splitlines():
            if not line.strip() or line.lstrip().startswith('#'):
                continue
            k, _, v = line.partition(':')
            data[k.strip()] = _value(v)
    # dates, numbers and booleans back to plain strings
    for k, v in list(data.items()):
        if isinstance(v, dict):
            data[k] = {str(a): str(b) for a, b in v.items()}
        elif isinstance(v, list):
            data[k] = [str(x) for x in v]
        elif v is None:
            data[k] = ''
        else:
            data[k] = str(v)
    return data


# ------------------------------------------------------------------ modules
def submodule_paths():
    paths = set()
    gm = os.path.join(ROOT, '.gitmodules')
    if os.path.exists(gm):
        for line in open(gm, encoding='utf-8'):
            m = re.match(r'\s*path\s*=\s*(\S+)', line)
            if m:
                paths.add(m.group(1))
    return paths


def find_modules():
    subs = submodule_paths()
    mods = {}
    for dirpath, dirnames, filenames in os.walk(os.path.join(ROOT, 'src')):
        rel = os.path.relpath(dirpath, ROOT)
        if any(rel == s or rel.startswith(s + os.sep) for s in subs):
            dirnames[:] = []
            continue
        dirnames[:] = sorted(d for d in dirnames if not d.startswith('.'))
        if 'package.xml' in filenames:
            xml = open(os.path.join(dirpath, 'package.xml'), encoding='utf-8').read()
            name = re.search(r'<name>\s*([^<\s]+)\s*</name>', xml)
            ver = re.search(r'<version>\s*([^<\s]+)\s*</version>', xml)
            if not name:
                continue
            cl = os.path.join(dirpath, 'CHANGELOG.md')
            released, unreleased = [], False
            if os.path.exists(cl):
                for line in open(cl, encoding='utf-8'):
                    m = re.match(r'##\s*\[([^\]]+)\](?:\s*-\s*(.*))?', line)
                    if m:
                        if m.group(1).lower() == 'unreleased':
                            unreleased = True
                        else:
                            released.append((m.group(1), (m.group(2) or '').strip()))
            mods[name.group(1)] = dict(dir=rel, version=ver.group(1) if ver else '?',
                                       changelog=os.path.relpath(cl, ROOT) if os.path.exists(cl) else None,
                                       released=released)
    return mods


def last_change(path):
    try:
        out = subprocess.run(['git', '-C', ROOT, 'log', '-1', '--format=%cs', '--', path],
                             capture_output=True, text=True, timeout=30)
        return out.stdout.strip() or '?'
    except (OSError, subprocess.SubprocessError):
        return '?'


# ------------------------------------------------------------------ reports
def areas():
    out = []
    for d in sorted(os.listdir(ANALYSIS)):
        full = os.path.join(ANALYSIS, d)
        if os.path.isdir(full) and (not d.startswith(('_', '.')) or d == UNSORTED):
            out.append(d)
    return sorted(out, key=lambda a: (a == UNSORTED, a))


def load_reports(mods):
    reps = []
    for area in areas():
        adir = os.path.join(ANALYSIS, area)
        for eid in sorted(os.listdir(adir)):
            rp = os.path.join(adir, eid, 'report.md')
            if not os.path.isfile(rp):
                continue
            fm = parse_front_matter(open(rp, encoding='utf-8').read())
            rel = os.path.relpath(rp, ROOT)
            if fm is None:
                warn(f'{rel}: front matter mancante')
                continue
            for k in REQUIRED:
                if k not in fm:
                    warn(f'{rel}: manca il campo `{k}`')
            fm.setdefault('modules', {})
            fm.setdefault('tags', [])
            if fm.get('id') != eid:
                warn(f'{rel}: id `{fm.get("id")}` diverso dalla cartella `{eid}`')
            if fm.get('area') != area:
                warn(f'{rel}: area `{fm.get("area")}` diversa dalla cartella `{area}`')
            if fm.get('status') not in STATUSES:
                warn(f'{rel}: status `{fm.get("status")}` non in {STATUSES}')
            if not isinstance(fm['modules'], dict):
                warn(f'{rel}: `modules` deve essere una mappa modulo: versione')
                fm['modules'] = {}
            for name, ver in fm['modules'].items():
                m = mods.get(name)
                if m is None:
                    warn(f'{rel}: modulo `{name}` inesistente nel workspace')
                elif ver != m['version'] and ver not in [v for v, _ in m['released']]:
                    warn(f'{rel}: `{name}` {ver} non esiste (package.xml {m["version"]}, '
                         f'CHANGELOG: {", ".join(v for v, _ in m["released"]) or "nessuna"})')
            fm['_area'], fm['_id'], fm['_path'] = area, eid, rel
            fm.setdefault('date', eid[:10])
            reps.append(fm)
    return reps


def cell(s, n=None):
    s = ' '.join(str(s).split()).replace('|', '\\|')
    if n and len(s) > n:
        s = s[:n - 1].rstrip() + '…'
    return s


def replace_auto(path, table):
    text = open(path, encoding='utf-8').read()
    if START not in text or END not in text:
        warn(f'{os.path.relpath(path, ROOT)}: marker {START} / {END} mancanti, tabella non scritta')
        return
    a, rest = text.split(START, 1)
    _, b = rest.split(END, 1)
    write_if_changed(path, a + START + '\n' + table + END + b)


def write_if_changed(path, content):
    old = open(path, encoding='utf-8').read() if os.path.exists(path) else None
    if old != content:
        os.makedirs(os.path.dirname(path), exist_ok=True)
        with open(path, 'w', encoding='utf-8', newline='') as f:
            f.write(content)


def one_liner(readme):
    if not os.path.exists(readme):
        return ''
    lines = open(readme, encoding='utf-8').read().splitlines()
    for i, line in enumerate(lines):
        if re.match(r'^##\s.*Stato attuale', line):
            for nxt in lines[i + 1:]:
                if nxt.startswith('#'):
                    break
                if nxt.strip():
                    return nxt.strip().lstrip('>').strip()
    return ''


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--check', action='store_true', help='esce con 1 se ci sono segnalazioni')
    args = ap.parse_args()

    mods = find_modules()
    reps = load_reports(mods)

    # INDEX.csv
    buf = io.StringIO()
    w = csv.writer(buf, lineterminator='\n')
    w.writerow(['id', 'date', 'area', 'title', 'status', 'result', 'decision', 'modules', 'tags',
                'branch', 'commit', 'path'])
    for r in sorted(reps, key=lambda r: (r['_id'], r['_area'])):
        w.writerow([r['_id'], r['date'], r['_area'], r.get('title', ''), r.get('status', ''),
                    r.get('result', ''), r.get('decision', ''),
                    ' '.join(f'{k}={v}' for k, v in r['modules'].items()), ' '.join(r['tags']),
                    r.get('branch', ''), r.get('commit', ''), r['_path']])
    write_if_changed(os.path.join(ANALYSIS, 'INDEX.csv'), buf.getvalue())

    # area READMEs
    tmpl = os.path.join(ANALYSIS, '_template', 'area_README.md')
    summary = []
    for area in areas():
        readme = os.path.join(ANALYSIS, area, 'README.md')
        if not os.path.exists(readme) and os.path.exists(tmpl):
            write_if_changed(readme, open(tmpl, encoding='utf-8').read().replace('<area>', area))
        rows = sorted((r for r in reps if r['_area'] == area), key=lambda r: r['_id'], reverse=True)
        t = ['| data | esperimento | status | risultato | moduli (versione) |',
             '|---|---|---|---|---|']
        for r in rows:
            mv = ', '.join(f'{k} {v}' for k, v in r['modules'].items()) or '—'
            t.append(f"| {r['date']} | [{cell(r.get('title', r['_id']))}]({r['_id']}/report.md) | "
                     f"{r.get('status', '')} | {cell(r.get('result', ''), 220)} | {cell(mv)} |")
        if not rows:
            t.append('| | nessun esperimento | | | |')
        replace_auto(readme, '\n'.join(t) + '\n')
        st = {s: sum(1 for r in rows if r.get('status') == s) for s in STATUSES}
        summary.append((area, len(rows), st, rows[0]['date'] if rows else '—', one_liner(readme)))

    t = ['| area | esperimenti | open / done / abandoned | ultimo | stato in una riga |',
         '|---|---|---|---|---|']
    for area, n, st, last, line in summary:
        t.append(f"| [{area}]({area}/README.md) | {n} | {st['open']} / {st['done']} / {st['abandoned']} | "
                 f"{last} | {cell(line, 240)} |")
    t.append('')
    t.append(f'Totale: {len(reps)} esperimenti. Elenco completo: [INDEX.csv](INDEX.csv).')
    replace_auto(os.path.join(ANALYSIS, 'README.md'), '\n'.join(t) + '\n')

    # docs/modules/README.md
    linked = {}
    for r in reps:
        for k in r['modules']:
            linked.setdefault(k, set()).add(r['_area'])
    t = ['| modulo | versione | ultimo cambio (git) | ultima release (CHANGELOG) | aree analisi collegate | changelog |',
         '|---|---|---|---|---|---|']
    for name in sorted(mods):
        m = mods[name]
        rel = m['released'][0] if m['released'] else None
        cl = f"[CHANGELOG](../../{m['changelog']})" if m['changelog'] else '—'
        if not m['changelog']:
            warn(f"{m['dir']}: CHANGELOG.md mancante")
        elif rel and rel[0] != m['version']:
            warn(f"{m['dir']}: package.xml {m['version']} ma l'ultima release del CHANGELOG e' {rel[0]}")
        ar = ', '.join(f'[{a}](../analysis/{a}/README.md)' for a in sorted(linked.get(name, ()))) or '—'
        t.append(f"| `{name}` | {m['version']} | {last_change(m['dir'])} | "
                 f"{(rel[0] + ' (' + rel[1] + ')') if rel else '—'} | {ar} | {cl} |")
    if not os.path.exists(MODULES_README):
        write_if_changed(MODULES_README, '# Moduli\n\n' + START + '\n' + END + '\n')
    replace_auto(MODULES_README, '\n'.join(t) + '\n')

    for wmsg in warnings:
        print('ATTENZIONE:', wmsg, file=sys.stderr)
    print(f'{len(reps)} esperimenti in {len(areas())} aree, {len(mods)} moduli, '
          f'{len(warnings)} segnalazioni')
    return 1 if (args.check and warnings) else 0


if __name__ == '__main__':
    sys.exit(main())
