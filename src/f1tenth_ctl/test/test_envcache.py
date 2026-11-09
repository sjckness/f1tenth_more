"""envcache: the sourced environment is reused only while nothing it depends on changed."""

import json
import os
import stat
import subprocess
import time

from f1tenth_ctl import config, envcache
import pytest

PKG = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
MIN = '\n'.join(f'F1T_{n.upper()}_CMD="sleep 1"' for n in config.MANAGED) + '\n'


@pytest.fixture(autouse=True)
def cache_home(tmp_path, monkeypatch):
    """Every test gets its own cache directory; HOME stays real (the shipped env files need it)."""
    d = tmp_path / 'cache'
    monkeypatch.setenv('F1T_ENV_CACHE_DIR', str(d))
    monkeypatch.delenv('F1T_ENV_CACHE', raising=False)     # conftest switches it off
    monkeypatch.delenv('F1T_ENV_CACHE_TTL_S', raising=False)
    return d


class Counter:
    """Wraps config.source_env_file and counts the real sourcings."""

    def __init__(self, monkeypatch):
        self.calls = 0
        real = config.source_env_file

        def counting(path, *a, **k):
            if 'trace_path' not in k:           # the traced twin of a miss is not counted
                self.calls += 1
            return real(path, *a, **k)

        monkeypatch.setattr(config, 'source_env_file', counting)


def _write(tmp_path, text, name='t.env'):
    path = tmp_path / name
    path.write_text(text)
    return str(path)


def _load(path):
    return config.load(path).env


def test_second_load_does_not_source(tmp_path, monkeypatch):
    path = _write(tmp_path, MIN + 'FOO=bar\n')
    count = Counter(monkeypatch)
    first = _load(path)
    second = _load(path)
    assert count.calls == 1
    assert first == second and second['FOO'] == 'bar'


def test_cached_environment_equals_the_sourced_one(tmp_path):
    path = _write(tmp_path, MIN + 'A=1\nB="x y"\nexport C=$A$A\n')
    sourced = config.source_env_file(path)
    _load(path)                                     # fills the cache
    cached = _load(path)
    assert cached == sourced


def test_pwd_is_the_callers(tmp_path, monkeypatch):
    path = _write(tmp_path, MIN)
    elsewhere = tmp_path / 'elsewhere'
    elsewhere.mkdir()
    monkeypatch.chdir(tmp_path)
    _load(path)
    monkeypatch.chdir(elsewhere)
    sourced = config.source_env_file(path)
    cached = _load(path)
    assert cached['PWD'] == sourced['PWD'] == os.path.realpath(str(elsewhere))
    assert cached == sourced


def test_editing_the_env_file_invalidates(tmp_path, monkeypatch):
    path = _write(tmp_path, MIN + 'FOO=1\n')
    count = Counter(monkeypatch)
    assert _load(path)['FOO'] == '1'
    _write(tmp_path, MIN + 'FOO=2\n')               # same size, new mtime
    assert _load(path)['FOO'] == '2'
    assert count.calls == 2


def test_same_mtime_but_new_size_invalidates(tmp_path, monkeypatch):
    path = _write(tmp_path, MIN + 'FOO=1\n')
    count = Counter(monkeypatch)
    _load(path)
    mtime = os.stat(path).st_mtime_ns
    _write(tmp_path, MIN + 'FOO=22\n')
    os.utime(path, ns=(mtime, mtime))
    assert _load(path)['FOO'] == '22'
    assert count.calls == 2


def test_a_sourced_chain_is_followed(tmp_path, monkeypatch):
    inner = _write(tmp_path, 'export DEEP=one\n', 'inner.sh')
    mid = _write(tmp_path, f'. {inner} || return 1\n', 'mid.sh')
    path = _write(tmp_path, MIN + f'source {mid} || return 1\n')
    count = Counter(monkeypatch)
    assert _load(path)['DEEP'] == 'one'
    assert _load(path)['DEEP'] == 'one'
    assert count.calls == 1
    _write(tmp_path, 'export DEEP=two\n', 'inner.sh')     # two levels down
    assert _load(path)['DEEP'] == 'two'
    assert count.calls == 2


def test_a_file_read_through_a_FILE_variable_is_watched(tmp_path, monkeypatch):
    args = tmp_path / 'args'
    path = _write(tmp_path, MIN + f'MY_ARGS_FILE={args}\n'
                  'if [ -r "$MY_ARGS_FILE" ]; then export EXTRA=$(cat "$MY_ARGS_FILE"); fi\n')
    count = Counter(monkeypatch)
    assert 'EXTRA' not in _load(path)               # absent when cached
    args.write_text('hello\n')
    assert _load(path)['EXTRA'] == 'hello'          # appeared: sourced again
    assert count.calls == 2
    args.write_text('hello world\n')
    assert _load(path)['EXTRA'] == 'hello world'


def test_a_new_package_in_a_prefix_share_dir_invalidates(tmp_path, monkeypatch):
    prefix = tmp_path / 'install'
    (prefix / 'share').mkdir(parents=True)
    setup = prefix / 'setup.bash'
    setup.write_text('export PREFIX_SET=1\n')
    path = _write(tmp_path, MIN + f'. {setup} || return 1\n')
    count = Counter(monkeypatch)
    _load(path)
    _load(path)
    assert count.calls == 1
    (prefix / 'share' / 'new_pkg').mkdir()
    _load(path)
    assert count.calls == 2


def test_the_entry_expires(tmp_path, monkeypatch):
    path = _write(tmp_path, MIN)
    count = Counter(monkeypatch)
    _load(path)
    monkeypatch.setenv('F1T_ENV_CACHE_TTL_S', '0.05')
    time.sleep(0.1)
    _load(path)
    assert count.calls == 2


def test_the_cache_can_be_switched_off(tmp_path, monkeypatch):
    path = _write(tmp_path, MIN)
    monkeypatch.setenv('F1T_ENV_CACHE', '0')
    count = Counter(monkeypatch)
    _load(path)
    _load(path)
    assert count.calls == 2
    assert not os.path.exists(envcache.cache_dir())


def test_a_different_base_environment_is_a_different_entry(tmp_path, monkeypatch):
    path = _write(tmp_path, MIN + 'FOO=$LANG\n')
    count = Counter(monkeypatch)
    monkeypatch.setenv('LANG', 'C')
    first = _load(path)
    monkeypatch.setenv('LANG', 'en_US.UTF-8')
    second = _load(path)
    assert (first['FOO'], second['FOO']) == ('C', 'en_US.UTF-8')
    assert count.calls == 2
    monkeypatch.setenv('LANG', 'C')
    assert _load(path)['FOO'] == 'C'
    assert count.calls == 2


@pytest.mark.parametrize('damage', [
    lambda p: open(p, 'w').write(''),
    lambda p: open(p, 'w').write('{not json'),
    lambda p: open(p, 'w').write('[]'),
    lambda p: open(p, 'w').write(json.dumps({'version': 1})),
    lambda p: open(p, 'w').write(json.dumps({'version': 99, 'env': {}})),
    lambda p: os.unlink(p),
], ids=['empty', 'bad-json', 'list', 'missing-keys', 'other-version', 'deleted'])
def test_a_damaged_cache_means_source_normally(tmp_path, monkeypatch, damage):
    path = _write(tmp_path, MIN + 'FOO=ok\n')
    count = Counter(monkeypatch)
    _load(path)
    base = config.base_environment()
    damage(envcache.cache_file(path, base))
    assert _load(path)['FOO'] == 'ok'
    assert count.calls == 2
    assert _load(path)['FOO'] == 'ok'              # and the entry is rewritten
    assert count.calls == 2


def test_an_unwritable_cache_dir_still_works(tmp_path, monkeypatch):
    blocker = tmp_path / 'blocker'
    blocker.write_text('a file where the directory should be')
    monkeypatch.setenv('F1T_ENV_CACHE_DIR', str(blocker / 'cache'))
    path = _write(tmp_path, MIN + 'FOO=ok\n')
    assert _load(path)['FOO'] == 'ok'
    assert _load(path)['FOO'] == 'ok'


def test_the_cache_is_private_and_atomic(tmp_path):
    path = _write(tmp_path, MIN + 'SECRET_TOKEN=abc\n')
    _load(path)
    directory = envcache.cache_dir()
    assert stat.S_IMODE(os.stat(directory).st_mode) == 0o700
    files = os.listdir(directory)
    assert len(files) == 1 and files[0].startswith('env-')
    assert stat.S_IMODE(os.stat(os.path.join(directory, files[0])).st_mode) == 0o600


def test_a_failing_env_file_is_still_an_error_and_caches_nothing(tmp_path):
    path = _write(tmp_path, 'source /nonexistent/setup.bash || return 1\n')
    with pytest.raises(config.ConfigError, match='exit 97'):
        config.load(path)
    assert not [f for f in os.listdir(envcache.cache_dir()) if f.startswith('env-')]


def test_an_untraceable_source_line_is_not_cached(tmp_path, monkeypatch):
    path = _write(tmp_path, MIN + 'f=/dev/null; source "$f"\n. ./relative-that-does-not-exist '
                  '2>/dev/null || true\n')
    count = Counter(monkeypatch)
    _load(path)
    _load(path)
    assert count.calls == 2


def test_traced_files_parsing():
    trace = '\n'.join([
        '+F1TSRC set -a', '+F1TSRC . /etc/hostname', '++F1TSRC source /etc/hostname arg',
        "+F1TSRC . '/etc/os-release'", '+F1TSRC echo . /nope'])
    assert envcache.traced_files(trace) == ['/etc/hostname', '/etc/os-release']
    assert envcache.traced_files('+F1TSRC . /definitely/not/here') is None
    assert envcache.traced_files('+F1TSRC . "unbalanced') is None
    assert envcache.traced_files('') == []


def test_tracing_leaves_no_trace_in_the_exported_environment(tmp_path):
    path = _write(tmp_path, MIN + 'A=1\n')
    plain = config.source_env_file(path)
    trace = tmp_path / 'trace'
    traced = config.source_env_file(path, trace_path=str(trace))
    assert traced == plain
    assert 'BASH_XTRACEFD' not in traced and 'PS4' not in traced
    assert '+F1TSRC' in trace.read_text()


def test_a_script_that_sees_the_tracing_is_not_cached_and_gets_the_plain_result(
        tmp_path, monkeypatch):
    """A file that exports PS4 differs between the two runs: no entry, plain result."""
    path = _write(tmp_path, MIN + 'export SEEN_PS4="$PS4"\n')
    count = Counter(monkeypatch)
    first = _load(path)
    second = _load(path)
    assert first == second == config.source_env_file(path)
    assert first['SEEN_PS4'] == '+ '
    assert count.calls == 3                         # nothing was reused


# ---- the real env files (they source ROS: only where it is installed) ----

needs_jazzy = pytest.mark.skipif(not os.path.isfile('/opt/ros/jazzy/setup.bash'),
                                 reason='the shipped env files source ROS Jazzy')


@needs_jazzy
@pytest.mark.parametrize('name', ['ctl_container.env', 'ctl.env'])
def test_cached_equals_sourced_on_the_real_env_files(name, tmp_path, monkeypatch):
    """The cached environment is, entry by entry, what sourcing the shipped file gives."""
    path = os.path.join(PKG, 'config', name)
    try:
        sourced = config.source_env_file(path)
    except config.ConfigError as exc:
        pytest.skip(f'{name} does not source here: {exc}')
    count = Counter(monkeypatch)
    first = _load(path)
    cached = _load(path)
    assert count.calls == 1
    assert first == sourced
    assert cached == sourced
    # and from another cwd, as the bot's ssh call would be
    monkeypatch.chdir(tmp_path)
    assert _load(path) == config.source_env_file(path)
    # the sourced chain really was followed: ROS's setup scripts are in the key
    entry = json.load(open(envcache.cache_file(path, config.base_environment())))
    watched = [f[0] for f in entry['files']]
    assert any(p.endswith('/jazzy/setup.bash') for p in watched), watched


@needs_jazzy
def test_a_cache_hit_is_much_faster_than_sourcing_on_the_real_env_file():
    path = os.path.join(PKG, 'config', 'ctl_container.env')
    try:
        t0 = time.perf_counter()
        config.source_env_file(path)
        cold = time.perf_counter() - t0
    except config.ConfigError as exc:
        pytest.skip(f'does not source here: {exc}')
    _load(path)
    t0 = time.perf_counter()
    _load(path)
    hit = time.perf_counter() - t0
    assert hit < cold / 5, (hit, cold)


def test_cli_wiring_uses_the_cache(tmp_path):
    """A second `f1tenth_ctl` process finds the entry the first one wrote."""
    path = _write(tmp_path, MIN + 'FOO=1\n')
    env = dict(os.environ, F1T_CTL_ENV=path, PYTHONPATH=PKG)
    for _ in range(2):
        out = subprocess.run(['python3', '-m', 'f1tenth_ctl', 'missions', '-n', '1'], env=env,
                             capture_output=True, text=True, timeout=60)
        assert json.loads(out.stdout.strip().splitlines()[-1])['env_file'] == path
    assert len([f for f in os.listdir(envcache.cache_dir()) if f.startswith('env-')]) == 1
