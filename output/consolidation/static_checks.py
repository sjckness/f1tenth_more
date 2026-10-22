#!/usr/bin/env python3
"""Static checks for the consolidation merges (no build, no ROS, no node runs).

Run from the repository root:  python3 -I output/consolidation/static_checks.py [BASE]

BASE (default campaign-final) only selects which files count as "touched" for
the bare '=======' marker check; every other check covers the whole tree.

Checks, each over the files git tracks (submodules excluded):
  markers   conflict markers: '<<<<<<< ', '>>>>>>> ', '||||||| ' anywhere;
            a bare '=======' line only in files changed since BASE
  python    every *.py compiles (compile(), nothing written), parsed with
            feature_version=(3, 10), PLUS the Python 3.12-only f-string forms
            that compile on linus (3.12) but not on the Orin (Humble, 3.10):
            a nested string reusing the enclosing f-string's quote, a
            backslash or a comment inside a replacement field (PEP 701)
  launch    every *.launch.py defines generate_launch_description()
  yaml      every *.yaml / *.yml parses (all documents), with duplicate
            mapping keys reported (safe_load keeps the last one silently)
  json      every *.json parses
  xml       every *.xml / *.launch.xml / *.urdf / *.sdf parses (expat also
            rejects '--' inside a comment, the package.xml trap in CLAUDE.md)
  params    stack_params.yaml: every top-level key is a mapping with
            'default'; every get_default/get_value/get_path_default('<key>')
            literal outside output/ names a key that exists (in a test
            file only a note: a test may assert that a key is gone)
  entry     every setup.py console_scripts target 'mod.sub:func' resolves to
            a file in the package that defines func at module level
  rosidl    every "msg/X.msg" / "srv/X.srv" / "action/X.action" listed in a
            CMakeLists.txt exists next to it

Exit status 0 when nothing failed. Python 3.12 is assumed (tokenize's
FSTRING_* tokens).
"""

import ast
import io
import json
import os
import re
import subprocess
import sys
import tokenize
import xml.etree.ElementTree as ET

import yaml

BASE = sys.argv[1] if len(sys.argv) > 1 else 'campaign-final'
FAIL = []
NOTE = []


def fail(check, where, msg):
    FAIL.append(f'[{check}] {where}: {msg}')


def git(*args):
    return subprocess.run(['git', *args], check=True, capture_output=True,
                          text=True).stdout


def tracked():
    out = git('ls-files', '-z', '--cached', '-s')
    files = []
    for rec in out.split('\0'):
        if not rec:
            continue
        meta, name = rec.split('\t', 1)
        if meta.split()[0] == '160000':   # submodule pointer
            continue
        files.append(name)
    return files


FILES = [f for f in tracked() if os.path.isfile(f)]


# --- markers ----------------------------------------------------------------
def check_markers():
    touched = set(git('diff', '--name-only', f'{BASE}...HEAD').split())
    touched |= set(git('diff', '--name-only', 'HEAD').split())
    hard = re.compile(rb'^(<<<<<<< |>>>>>>> |\|\|\|\|\|\|\| )')
    for f in FILES:
        try:
            data = open(f, 'rb').read()
        except OSError as e:
            fail('markers', f, str(e))
            continue
        if b'\0' in data[:8000]:
            continue
        for n, line in enumerate(data.splitlines(), 1):
            if hard.match(line):
                fail('markers', f'{f}:{n}', line[:60].decode(errors='replace'))
            elif line == b'=======' and f in touched:
                fail('markers', f'{f}:{n}', "bare '=======' line in a touched file")


# --- python -----------------------------------------------------------------
def fstring_310_problems(src):
    """PEP 701 forms Python 3.10 rejects. Returns [(line, msg)]."""
    probs = []
    stack = []   # (quote, depth of replacement-field nesting)
    try:
        toks = list(tokenize.generate_tokens(io.StringIO(src).readline))
    except (tokenize.TokenError, SyntaxError, IndentationError):
        return probs
    for t in toks:
        if t.type == tokenize.FSTRING_START:
            q = t.string.lstrip('rRfFbBuU')
            if stack and stack[-1][1] > 0:
                outer = stack[-1][0]
                if q[0] == outer[0] and (len(outer) == 1 or q == outer):
                    probs.append((t.start[0], f'nested f-string reuses the quote {outer}'))
            stack.append([q, 0])
            continue
        if t.type == tokenize.FSTRING_END:
            if stack:
                stack.pop()
            continue
        if not stack:
            continue
        if t.type == tokenize.OP and t.string == '{':
            stack[-1][1] += 1
            continue
        if t.type == tokenize.OP and t.string == '}':
            stack[-1][1] = max(0, stack[-1][1] - 1)
            continue
        if stack[-1][1] == 0 or t.type == tokenize.FSTRING_MIDDLE:
            continue
        outer = stack[-1][0]
        if t.type == tokenize.STRING:
            body = t.string.lstrip('rRbBuU')
            if body[:1] == outer[0] and (len(outer) == 1 or body.startswith(outer)):
                probs.append((t.start[0], f'string inside an f-string reuses its quote {outer}'))
        if t.type == tokenize.COMMENT:
            probs.append((t.start[0], 'comment inside an f-string replacement field'))
        if '\\' in t.string and t.type in (tokenize.STRING, tokenize.OP, tokenize.NAME):
            probs.append((t.start[0], 'backslash inside an f-string replacement field'))
    return probs


def check_python():
    for f in FILES:
        if not f.endswith('.py'):
            continue
        try:
            src = open(f, encoding='utf-8').read()
        except (OSError, UnicodeDecodeError) as e:
            fail('python', f, str(e))
            continue
        try:
            compile(src, f, 'exec', dont_inherit=True)
            tree = ast.parse(src, f, feature_version=(3, 10))
        except SyntaxError as e:
            fail('python', f'{f}:{e.lineno}', e.msg)
            continue
        for line, msg in fstring_310_problems(src):
            fail('python', f'{f}:{line}', f'Python 3.10 (Humble) would reject: {msg}')
        if f.endswith('.launch.py'):
            if not any(isinstance(n, ast.FunctionDef) and n.name == 'generate_launch_description'
                       for n in tree.body):
                fail('launch', f, 'no generate_launch_description()')


# --- yaml / json / xml ------------------------------------------------------
class DupKeyLoader(yaml.SafeLoader):
    pass


def _construct_mapping(loader, node, deep=False):
    seen = {}
    for key_node, _ in node.value:
        key = loader.construct_object(key_node, deep=deep)
        try:
            hash(key)
        except TypeError:
            continue
        if key in seen:
            loader.dups.append((key, seen[key], key_node.start_mark.line + 1))
        seen[key] = key_node.start_mark.line + 1
    return yaml.SafeLoader.construct_mapping(loader, node, deep)


DupKeyLoader.add_constructor(yaml.resolver.BaseResolver.DEFAULT_MAPPING_TAG, _construct_mapping)


def load_yaml(f):
    loader = DupKeyLoader(open(f, encoding='utf-8').read())
    loader.dups = []
    try:
        docs = []
        while loader.check_data():
            docs.append(loader.get_data())
        return docs, loader.dups
    finally:
        loader.dispose()


def check_data_files():
    for f in FILES:
        low = f.lower()
        if low.endswith(('.yaml', '.yml')):
            try:
                _, dups = load_yaml(f)
            except yaml.YAMLError as e:
                fail('yaml', f, str(e).replace('\n', ' ')[:200])
                continue
            except UnicodeDecodeError as e:
                fail('yaml', f, str(e))
                continue
            for key, first, again in dups:
                fail('yaml', f'{f}:{again}', f'duplicate key {key!r} (first at line {first})')
        elif low.endswith('.json'):
            try:
                json.load(open(f, encoding='utf-8'))
            except (ValueError, UnicodeDecodeError) as e:
                fail('json', f, str(e)[:200])
        elif low.endswith(('.xml', '.urdf', '.sdf', '.launch')) and not low.endswith('.xacro'):
            try:
                ET.parse(f)
            except ET.ParseError as e:
                fail('xml', f, str(e))


# --- stack_params keys ------------------------------------------------------
PARAMS = 'src/f1tenth_params/config/stack_params.yaml'
KEY_CALL = re.compile(r"""\bget_(?:default|value|path_default)\(\s*['"]([A-Za-z0-9_]+)['"]\s*[,)]""")


def check_params():
    if not os.path.isfile(PARAMS):
        fail('params', PARAMS, 'missing')
        return
    docs, _ = load_yaml(PARAMS)
    params = docs[0] if docs else {}
    for k, v in params.items():
        if not isinstance(v, dict) or 'default' not in v:
            fail('params', PARAMS, f'key {k!r} has no default')
    for f in FILES:
        if not f.endswith('.py') or f.startswith('output/'):
            continue
        try:
            src = open(f, encoding='utf-8').read()
        except (OSError, UnicodeDecodeError):
            continue
        for m in KEY_CALL.finditer(src):
            if m.group(1) not in params:
                line = src.count('\n', 0, m.start()) + 1
                msg = f'{m.group(1)!r} is not a key of stack_params.yaml'
                if '/test/' in f:   # a test may assert that a key is gone
                    NOTE.append(f'[params] {f}:{line}: {msg} (test file)')
                else:
                    fail('params', f'{f}:{line}', msg)


# --- console_scripts --------------------------------------------------------
def check_entry_points():
    for f in FILES:
        if os.path.basename(f) != 'setup.py':
            continue
        pkg_dir = os.path.dirname(f)
        try:
            tree = ast.parse(open(f, encoding='utf-8').read(), f)
        except SyntaxError:
            continue   # reported by check_python
        for node in ast.walk(tree):
            if not (isinstance(node, ast.Constant) and isinstance(node.value, str)):
                continue
            m = re.fullmatch(r'\s*([\w.-]+)\s*=\s*([\w.]+):(\w+)\s*', node.value)
            if not m:
                continue
            mod, func = m.group(2), m.group(3)
            base = os.path.join(pkg_dir, *mod.split('.'))
            target = base + '.py' if os.path.isfile(base + '.py') else os.path.join(base, '__init__.py')
            if not os.path.isfile(target):
                fail('entry', f, f'{m.group(1)} -> {mod}:{func}: no module file')
                continue
            try:
                body = ast.parse(open(target, encoding='utf-8').read()).body
            except SyntaxError:
                continue
            names = {n.name for n in body if isinstance(n, (ast.FunctionDef, ast.AsyncFunctionDef, ast.ClassDef))}
            names |= {t.id for n in body if isinstance(n, ast.Assign) for t in n.targets if isinstance(t, ast.Name)}
            names |= {a.asname or a.name for n in body if isinstance(n, ast.ImportFrom) for a in n.names}
            if func not in names:
                fail('entry', f, f'{m.group(1)} -> {mod}:{func}: {func} not defined in {target}')


# --- rosidl lists -----------------------------------------------------------
def check_rosidl():
    for f in FILES:
        if os.path.basename(f) != 'CMakeLists.txt':
            continue
        src = open(f, encoding='utf-8', errors='replace').read()
        for m in re.finditer(r'"((?:msg|srv|action)/[\w]+\.(?:msg|srv|action))"', src):
            if not os.path.isfile(os.path.join(os.path.dirname(f), m.group(1))):
                fail('rosidl', f, f'{m.group(1)} listed but missing')


def main():
    for check in (check_markers, check_python, check_data_files, check_params,
                  check_entry_points, check_rosidl):
        check()
    n_py = sum(f.endswith('.py') for f in FILES)
    n_yaml = sum(f.lower().endswith(('.yaml', '.yml')) for f in FILES)
    print(f'static_checks: HEAD {git("rev-parse", "--short", "HEAD").strip()}, '
          f'{len(FILES)} tracked files ({n_py} .py, {n_yaml} .yaml/.yml), base {BASE}')
    for line in NOTE:
        print('note:', line)
    for line in FAIL:
        print('FAIL', line)
    print('result:', 'PASS' if not FAIL else f'{len(FAIL)} FAIL')
    return 1 if FAIL else 0


if __name__ == '__main__':
    sys.exit(main())
