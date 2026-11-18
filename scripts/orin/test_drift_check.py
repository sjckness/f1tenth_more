"""
drift_check.{sh,py} on a fixture repository shaped like the car's checkout.

A base commit, one car commit beyond it, an uncommitted src/ change and
untracked src/ files (one shipped, one excluded), recorded in a car_sync.json
the way docker/orin/car_sync.json records the real car; then every kind of
change the car can make after the image was built. Runs anywhere with git:
    python3 -m pytest scripts/orin/test_drift_check.py -q
"""

from argparse import Namespace
import hashlib
import json
import os
import subprocess
import sys

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(os.path.dirname(HERE))
sys.dont_write_bytecode = True       # no __pycache__ next to the Orin scripts
sys.path.insert(0, HERE)
import drift_check  # noqa: E402, I100


@pytest.fixture(autouse=True)
def plain_git(monkeypatch):
    """Run git without the user's config, in the fixture and in the check."""
    for key, value in (('GIT_CONFIG_GLOBAL', '/dev/null'), ('GIT_CONFIG_NOSYSTEM', '1'),
                       ('GIT_AUTHOR_NAME', 'car'), ('GIT_AUTHOR_EMAIL', 'car@example.org'),
                       ('GIT_COMMITTER_NAME', 'car'),
                       ('GIT_COMMITTER_EMAIL', 'car@example.org')):
        monkeypatch.setenv(key, value)


def git(repo, *args):
    return subprocess.run(['git', '-C', str(repo), *args], check=True, capture_output=True,
                          text=True).stdout.strip()


def write(path, text):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, 'w') as f:
        f.write(text)


def sha(path):
    with open(path, 'rb') as f:
        return hashlib.sha256(f.read()).hexdigest()


@pytest.fixture
def car(tmp_path):
    repo = tmp_path / 'f1tenth_more'
    subprocess.run(['git', 'init', '-q', '-b', 'campaign', str(repo)], check=True)
    write(repo / 'src/pkg/turn.py', 'MARGIN = 0.5\n')
    write(repo / 'src/params/stack_params.yaml', 'margin:\n  default: 0.5\n')
    write(repo / '.gitignore', '*.pyc\n')
    write(repo / 'README.md', 'car\n')
    git(repo, 'add', '-A')
    git(repo, 'commit', '-q', '-m', 'base')
    base = git(repo, 'rev-parse', 'HEAD')
    write(repo / 'src/pkg/turn.py', 'MARGIN = 0.5\nRAMP = True\n')
    git(repo, 'commit', '-q', '-am', 'wall_turn: speed ramp')
    head = git(repo, 'rev-parse', 'HEAD')
    # What the car has beyond its commits: a param change, two new missions.
    write(repo / 'src/params/stack_params.yaml', 'margin:\n  default: 0.7\n')
    write(repo / 'src/missions/good.json', '{"moves": []}\n')
    write(repo / 'src/missions/bad.json', '{"moves": [{"stop_condition": " "}]}\n')
    # Neither of these is the car's code: ignored, and outside src/.
    write(repo / 'src/pkg/turn.pyc', 'bytecode')
    write(repo / 'notes.txt', 'scratch\n')
    native = drift_check.capture(str(repo), base)
    car_sync = {
        'schema': 1, 'base': base, 'native_head': head, 'native_branch': 'campaign',
        'commits': [{'car': head, 'picked_as': 'f' * 40}],
        'uncommitted_src_patch_sha256': native['src_diff_sha256'],
        'untracked': {'src/missions/good.json': sha(repo / 'src/missions/good.json')},
        'untracked_excluded': {'src/missions/bad.json': {
            'sha256': sha(repo / 'src/missions/bad.json'), 'reason': 'loader refuses it'}},
    }
    path = tmp_path / 'CAR_SYNC.json'
    path.write_text(json.dumps(car_sync))
    return Namespace(repo=repo, base=base, head=head, path=str(path), car_sync=car_sync)


def run(car, capsys, *extra):
    code = drift_check.main(['--car-sync', car.path, '--ws', str(car.repo), *extra])
    return code, json.loads(capsys.readouterr().out)


def kinds(out):
    return sorted(d['kind'] for d in out['drift'])


def test_the_tree_the_image_was_synced_from_is_clean(car, capsys):
    index = car.repo / '.git' / 'index'
    before = index.stat().st_mtime_ns
    code, out = run(car, capsys)
    assert code == 0 and out['ok'] is True and out['drift'] == [] and out['warnings'] == []
    assert out['native']['head'] == car.head and out['native_untracked'] == 2
    # Read-only: git did not even refresh the index.
    assert index.stat().st_mtime_ns == before
    assert not (car.repo / '.git' / 'index.lock').exists()


def test_the_shell_wrapper_passes_exit_codes_and_json(car):
    sh = os.path.join(HERE, 'drift_check.sh')
    proc = subprocess.run([sh, '--car-sync', car.path, '--ws', str(car.repo)],
                          capture_output=True, text=True)
    assert proc.returncode == 0 and json.loads(proc.stdout)['ok'] is True
    write(car.repo / 'src/missions/new.json', '{}\n')
    proc = subprocess.run([sh, '--car-sync', car.path, '--ws', str(car.repo)],
                          capture_output=True, text=True)
    assert proc.returncode == 3 and json.loads(proc.stdout)['ok'] is False


def test_a_new_commit_is_drift(car, capsys):
    write(car.repo / 'src/pkg/turn.py', 'MARGIN = 0.5\nRAMP = False\n')
    git(car.repo, 'commit', '-q', '-m', 'wall_turn: ramp off', '--', 'src/pkg/turn.py')
    code, out = run(car, capsys)
    assert code == 3 and kinds(out) == ['commit']
    assert out['drift'][0]['subject'] == 'wall_turn: ramp off'
    # Committing the param change too: a commit AND a diff that went away.
    git(car.repo, 'commit', '-q', '-am', 'margin 0.7')
    code, out = run(car, capsys)
    assert code == 3 and kinds(out) == ['commit', 'commit', 'src_diff']


def test_a_changed_or_reverted_src_diff_is_drift(car, capsys):
    write(car.repo / 'src/params/stack_params.yaml', 'margin:\n  default: 0.8\n')
    code, out = run(car, capsys)
    assert code == 3 and kinds(out) == ['src_diff']
    assert out['drift'][0]['files'] == ['src/params/stack_params.yaml']
    # Back to the committed value: the image has 0.7, the car 0.5 again.
    write(car.repo / 'src/params/stack_params.yaml', 'margin:\n  default: 0.5\n')
    code, out = run(car, capsys)
    assert code == 3 and out['drift'][0]['sha256'] is None
    assert out['drift'][0]['expected'] == car.car_sync['uncommitted_src_patch_sha256']


def test_a_new_or_changed_untracked_file_is_drift(car, capsys):
    write(car.repo / 'src/missions/third.json', '{}\n')
    code, out = run(car, capsys)
    assert code == 3 and out['drift'] == [{
        'kind': 'untracked', 'path': 'src/missions/third.json',
        'sha256': hashlib.sha256(b'{}\n').hexdigest(), 'why': 'not in car_sync'}]
    os.remove(car.repo / 'src/missions/third.json')
    write(car.repo / 'src/missions/good.json', '{"moves": [1]}\n')
    code, out = run(car, capsys)
    assert code == 3 and out['drift'][0]['why'] == 'content changed'


def test_an_excluded_file_is_drift_only_when_it_changes(car, capsys):
    write(car.repo / 'src/missions/bad.json', '{"moves": []}\n')
    code, out = run(car, capsys)
    assert code == 3 and out['drift'][0]['path'] == 'src/missions/bad.json'


def test_ignored_files_and_files_outside_src_are_not_the_cars_code(car, capsys):
    write(car.repo / 'src/pkg/other.pyc', 'x')
    write(car.repo / 'docs/new.md', 'x')
    code, out = run(car, capsys)
    assert code == 0 and out['drift'] == []


def test_less_on_the_car_than_in_the_image_is_a_warning(car, capsys):
    os.remove(car.repo / 'src/missions/good.json')
    code, out = run(car, capsys)
    assert code == 0 and [w['kind'] for w in out['warnings']] == ['untracked_missing']
    # HEAD is not the captured one, yet every commit is known (the car went back).
    car.car_sync['native_head'] = 'e' * 40
    car.car_sync['commits'].append({'car': 'e' * 40, 'picked_as': 'd' * 40})
    with open(car.path, 'w') as f:
        json.dump(car.car_sync, f)
    code, out = run(car, capsys)
    assert code == 0 and 'head' in [w['kind'] for w in out['warnings']]


def test_an_empty_diff_recorded_as_the_empty_sha_is_clean(car, capsys):
    write(car.repo / 'src/params/stack_params.yaml', 'margin:\n  default: 0.5\n')
    car.car_sync['uncommitted_src_patch_sha256'] = drift_check.EMPTY_SHA256
    with open(car.path, 'w') as f:
        json.dump(car.car_sync, f)
    code, out = run(car, capsys)
    assert code == 0, out


def test_errors_are_exit_2_with_json(car, capsys, tmp_path):
    code, out = run(car, capsys, '--ws', str(tmp_path / 'missing'))
    assert code == 2 and 'no such directory' in out['error']
    car.car_sync['base'] = 'f' * 40
    with open(car.path, 'w') as f:
        json.dump(car.car_sync, f)
    code, out = run(car, capsys)
    assert code == 2 and out['ok'] is False
    with open(car.path, 'w') as f:
        f.write('not json')
    code, out = run(car, capsys)
    assert code == 2 and car.path in out['error']


def test_the_recorded_car_sync_matches_its_capture():
    """docker/orin/car_sync.json against the capture it was made from."""
    with open(os.path.join(REPO, 'docker/orin/car_sync.json')) as f:
        car_sync = json.load(f)
    capture = os.path.join(REPO, car_sync['capture'])
    assert sha(os.path.join(capture, 'src.diff')) == car_sync['uncommitted_src_patch_sha256']
    recorded = {}
    with open(os.path.join(capture, 'untracked.sha256')) as f:
        for line in f:
            digest, path = line.split(None, 1)
            recorded[path.strip()] = digest
    shipped = car_sync['untracked']
    excluded = {k: v['sha256'] for k, v in car_sync['untracked_excluded'].items()}
    assert {**shipped, **excluded} == recorded
    with open(os.path.join(capture, 'commits.txt')) as f:
        car_commits = [line.split()[0] for line in f if line.strip()]
    assert sorted(c['car'] for c in car_sync['commits']) == sorted(car_commits)
