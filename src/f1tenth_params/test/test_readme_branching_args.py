"""README's branching-arg table must agree with stack_params.yaml.

Two failures this file exists to catch.

1. A WRONG DEFAULT in README's "## The 4 stack-wide branching args" table. That
   table claimed `localization_source` defaulted to `raw_odom` and `enable_nav2`
   to `true` while the YAML said `ekf` and `false`. Those two values decide which
   controller drives the car, so a reader editing the stack from the README was
   pointed at the wrong half of it.

2. A FIFTH BRANCHING ARG, or one of the four gaining a DeclareLaunchArgument,
   without the README following. What makes these four "branching" is not being
   declared anywhere: they are read as plain Python at parse time, so a CLI
   override is silently ignored. Give one a DeclareLaunchArgument and the
   README's silent-ignore warning becomes false.

Both files are read from the source tree, not the install space, so this runs
standalone: python3 -m pytest test/test_readme_branching_args.py -v  (no colcon
build -- it is broken workspace-wide, see CLAUDE.md). Nothing is imported from
this package or from ament, so no RMW initialisation is needed.
"""

from pathlib import Path
import re

import yaml

# src/f1tenth_params/test/this_file.py -> the workspace root.
WORKSPACE_ROOT = Path(__file__).resolve().parents[3]
README_PATH = WORKSPACE_ROOT / 'README.md'
STACK_PARAMS_PATH = (
    WORKSPACE_ROOT / 'src' / 'f1tenth_params' / 'config' / 'stack_params.yaml')

SECTION_HEADING = '## The 4 stack-wide branching args'

# The four names, as spelled in param_defaults.get_value's docstring and in
# stack_params.yaml's own "Stack-Wide" block. Hardcoded deliberately: this tuple
# IS the specification the table is checked against, so deriving it from either
# file under test would let one drift quietly take the other with it.
BRANCHING_ARGS = (
    'camera_source',
    'localization_source',
    'use_behavior_tree',
    'enable_nav2',
)

# | `name` | `default` | meaning ... |  -- anchored on the two cells that matter,
# because the meaning cell contains escaped pipes and cannot be split on '|'.
_TABLE_ROW = re.compile(
    r'^\|\s*`(?P<name>[A-Za-z0-9_]+)`\s*\|\s*`(?P<default>[^`]*)`\s*\|')

# "**Default is raw**" and friends: prose that restates the Default column.
_DEFAULT_IS = re.compile(
    r'default\s+is\s+\*{0,2}(?P<claimed>[A-Za-z0-9_]+)', re.IGNORECASE)

# DeclareLaunchArgument('name', ...), including the multi-line spelling.
_DECLARED_ARG = re.compile(
    r"DeclareLaunchArgument\(\s*['\"](?P<name>[A-Za-z0-9_]+)['\"]")


def _readme_rows():
    """[(name, default cell, whole row)] for the branching-arg table."""
    lines = README_PATH.read_text(encoding='utf-8').splitlines()
    assert SECTION_HEADING in lines, (
        f'{README_PATH} has no {SECTION_HEADING!r} heading -- the branching-arg '
        'section moved, so this test would otherwise check nothing.')
    start = lines.index(SECTION_HEADING)
    end = next((i for i in range(start + 1, len(lines))
                if lines[i].startswith('## ')), len(lines))

    rows = []
    for line in lines[start + 1:end]:
        match = _TABLE_ROW.match(line)
        if match:
            rows.append((match.group('name'), match.group('default'), line))
    assert rows, (f'no table rows found under {SECTION_HEADING!r} in '
                  f'{README_PATH} -- the table was reformatted, not removed.')
    return rows


def _stack_params():
    """stack_params.yaml, loaded from the source tree."""
    return yaml.safe_load(STACK_PARAMS_PATH.read_text(encoding='utf-8')) or {}


def _readme_text(value):
    """Render a yaml default the way the table writes it: bools lower-case."""
    if isinstance(value, bool):
        return 'true' if value else 'false'
    return str(value)


def test_readme_table_lists_exactly_the_branching_args():
    names = [name for name, _, _ in _readme_rows()]
    assert len(names) == len(BRANCHING_ARGS), (
        f'README table has {len(names)} row(s) {names}, expected exactly '
        f'{len(BRANCHING_ARGS)} {list(BRANCHING_ARGS)}.')
    missing = set(BRANCHING_ARGS) - set(names)
    extra = set(names) - set(BRANCHING_ARGS)
    assert not missing and not extra, (
        f'README table is out of step with the 4 branching args: missing '
        f'{sorted(missing)}, extra {sorted(extra)}. A fifth branching arg needs '
        "a stack_params.yaml key, param_defaults.get_value's docstring, this "
        'tuple and the README table updated together.')


def test_readme_defaults_match_stack_params():
    params = _stack_params()
    table = {name: (default, line) for name, default, line in _readme_rows()}
    wrong = []
    for name in BRANCHING_ARGS:
        assert name in params, f'stack_params.yaml has no {name!r} key'
        assert name in table, f'README table has no {name!r} row'
        actual = _readme_text(params[name]['default'])
        if table[name][0] != actual:
            wrong.append(f'{name}: README says {table[name][0]!r}, '
                         f'stack_params.yaml says {actual!r}')
    assert not wrong, (
        'README branching-arg defaults disagree with stack_params.yaml: '
        + '; '.join(wrong))


def test_readme_prose_does_not_restate_a_wrong_default():
    """Catches the "Default is raw, not EKF" sentence, not just the cell."""
    params = _stack_params()
    table = {name: (default, line) for name, default, line in _readme_rows()}
    wrong = []
    for name in BRANCHING_ARGS:
        if name not in params or name not in table:
            continue
        actual = _readme_text(params[name]['default'])
        for claimed in _DEFAULT_IS.findall(table[name][1]):
            if claimed.lower() != actual.lower():
                wrong.append(f'{name}: row claims "Default is {claimed}", '
                             f'stack_params.yaml says {actual!r}')
    assert not wrong, '; '.join(wrong)


def test_branching_args_are_not_launch_arguments():
    """Not being declared anywhere is exactly what makes them branching."""
    launch_files = sorted(
        path for path in WORKSPACE_ROOT.glob('src/**/launch/*.py')
        if 'f1tenth_external' not in path.parts)
    assert len(launch_files) > 10, (
        f'only {len(launch_files)} launch file(s) matched {WORKSPACE_ROOT}/src: '
        'the glob is wrong, so this test would pass without checking anything.')

    declared = {}
    for path in launch_files:
        for match in _DECLARED_ARG.finditer(path.read_text(encoding='utf-8')):
            declared.setdefault(match.group('name'), []).append(path)

    offenders = [
        f'{name} declared in '
        + ', '.join(str(p.relative_to(WORKSPACE_ROOT)) for p in declared[name])
        for name in BRANCHING_ARGS if name in declared]
    assert not offenders, (
        'a stack-wide branching arg gained a DeclareLaunchArgument, so a CLI '
        'override would now take effect while README still warns it is silently '
        'ignored: ' + '; '.join(offenders))
