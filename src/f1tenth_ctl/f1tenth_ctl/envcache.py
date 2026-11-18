"""
A cache for the result of sourcing the env file (config.source_env_file).

Every CLI call sources ctl_container.env in bash (0.7 s on the Orin: ROS's
setup.bash and the workspace's), and the answer is the same unless a sourced
file changed. This keeps the resulting environment under the user's cache
dir and reuses it while nothing it depends on moved:

  * the env file and every file it sources, found by tracing the sourcing
    (`set -x`, PS4 marker) and following `. file` / `source file` chains;
    each one by (mtime_ns, size), or "absent";
  * every path in the result under a key ending in `_FILE` (the env file reads
    F1T_STACK_ARGS_FILE without sourcing it), the same way;
  * the `share/` directory of each sourced ROS/colcon prefix script (a
    package added to an install tree changes it, the scripts do not);
  * the minimal base environment the sourcing started from (config.BASE_KEYS);
  * the age of the entry: at most F1T_ENV_CACHE_TTL_S (default 600 s), a bound
    on whatever a sourced script reads that is not listed above (an apt
    upgrade, a changed python3). F1T_ENV_CACHE=0 switches the cache off.

ANY problem (no cache file, bad JSON, a key mismatch, a file that cannot be
stat'd, an untraceable `source` line, a write error) means: source normally,
exactly as before. The cache file is mode 0600 in a 0700 directory, written
atomically, because an environment can hold secrets.

PWD is the one value that depends on the caller (bash sets it from the cwd):
a hit gives it the caller's cwd, as sourcing would.
"""

import hashlib
import json
import os
import re
import shlex
import tempfile
import threading
import time
from typing import Callable, Dict, List, Optional

VERSION = 1
DEFAULT_TTL_S = 600.0
PREFIX_SCRIPTS = ('setup.bash', 'local_setup.bash', 'setup.sh', 'local_setup.sh')
_TRACE_RE = re.compile(r'^\++F1TSRC (?:\.|source) (.+)$')


def enabled(env=None) -> bool:
    """Return False when F1T_ENV_CACHE is 0/false/no/off."""
    value = (os.environ if env is None else env).get('F1T_ENV_CACHE', '1')
    return value.strip().lower() not in ('0', 'false', 'no', 'off')


def ttl_s(env=None) -> float:
    """Return the entry lifetime in seconds (F1T_ENV_CACHE_TTL_S, default 600)."""
    raw = (os.environ if env is None else env).get('F1T_ENV_CACHE_TTL_S')
    try:
        return float(raw) if raw not in (None, '') else DEFAULT_TTL_S
    except ValueError:
        return DEFAULT_TTL_S


def cache_dir() -> str:
    """Return the cache directory: F1T_ENV_CACHE_DIR (tests), else ~/.cache/f1tenth_ctl."""
    return (os.environ.get('F1T_ENV_CACHE_DIR')
            or os.path.join(os.path.expanduser('~'), '.cache', 'f1tenth_ctl'))


def cache_file(env_file: str, base: Dict[str, str]) -> str:
    """Return the cache file of this env file on this base environment."""
    key = json.dumps([os.path.abspath(env_file), sorted(base.items())])
    digest = hashlib.sha1(key.encode()).hexdigest()[:16]
    return os.path.join(cache_dir(), f'env-{digest}.json')


def traced_files(trace_text: str, cwd: str = '/') -> Optional[List[str]]:
    """
    Return the files a traced sourcing sourced, in order, or None when one is untraceable.

    A `source` argument that is not an existing file once resolved (relative to
    cwd, then PATH) cannot be put in the key, so the cache must not be used.
    """
    seen, out = set(), []
    for line in trace_text.splitlines():
        m = _TRACE_RE.match(line)
        if not m:
            continue
        try:
            words = shlex.split(m.group(1))
        except ValueError:
            return None
        if not words:
            return None
        path = words[0]
        if not os.path.isabs(path):
            found = [p for p in [os.path.join(cwd, path)] + [
                os.path.join(d, path) for d in os.environ.get('PATH', '').split(':') if d]
                if os.path.isfile(p)]
            if not found:
                return None
            path = found[0]
        path = os.path.abspath(path)
        if not os.path.isfile(path):
            return None
        if path not in seen:
            seen.add(path)
            out.append(path)
    return out


def watched(env_file: str, sourced: List[str], env: Dict[str, str]) -> List[str]:
    """Every path whose state goes in the key: sourced files, *_FILE values, prefix share/."""
    paths = [os.path.abspath(env_file)] + list(sourced)
    for key, value in env.items():
        if key.endswith('_FILE') and value.startswith('/'):
            paths.append(value)
    for path in sourced:
        if os.path.basename(path) in PREFIX_SCRIPTS:
            paths.append(os.path.join(os.path.dirname(path), 'share'))
    seen, out = set(), []
    for p in paths:
        if p not in seen:
            seen.add(p)
            out.append(p)
    return out


def stat_entry(path: str):
    """Return [path, mtime_ns, size], or [path, None, None] when absent."""
    try:
        st = os.stat(path)
    except FileNotFoundError:
        return [path, None, None]
    return [path, st.st_mtime_ns, st.st_size]


def valid(entry: dict, env_file: str, base: Dict[str, str], now: float, ttl: float) -> bool:
    """Return True when a stored entry still describes the world (see the module docstring)."""
    try:
        if entry['version'] != VERSION or entry['env_file'] != os.path.abspath(env_file):
            return False
        if entry['base'] != sorted(base.items()) and entry['base'] != [
                list(kv) for kv in sorted(base.items())]:
            return False
        age = now - float(entry['created'])
        if age < 0 or age > ttl:
            return False
        if not isinstance(entry['env'], dict) or not entry['files']:
            return False
        for path, mtime_ns, size in entry['files']:
            if stat_entry(path) != [path, mtime_ns, size]:
                return False
        return True
    except (KeyError, TypeError, ValueError):
        return False


def read(env_file: str, base: Dict[str, str], now: float = None) -> Optional[Dict[str, str]]:
    """Return the cached environment, or None (any problem: miss)."""
    try:
        with open(cache_file(env_file, base), 'r') as fh:
            entry = json.load(fh)
        if not valid(entry, env_file, base, time.time() if now is None else now, ttl_s()):
            return None
        env = {str(k): str(v) for k, v in entry['env'].items()}
    except (OSError, ValueError, TypeError, AttributeError):
        return None
    if 'PWD' in env:
        env['PWD'] = os.getcwd()
    return env


def write(env_file: str, base: Dict[str, str], env: Dict[str, str], sourced: List[str],
          now: float = None) -> bool:
    """Store an entry atomically (0600 file, 0700 directory); return False on any error."""
    try:
        files = [stat_entry(p) for p in watched(env_file, sourced, env)]
        entry = {'version': VERSION, 'env_file': os.path.abspath(env_file),
                 'base': sorted(base.items()), 'created': time.time() if now is None else now,
                 'files': files, 'env': env}
        directory = cache_dir()
        os.makedirs(directory, mode=0o700, exist_ok=True)
        os.chmod(directory, 0o700)
        fd, tmp = tempfile.mkstemp(prefix='.env-', suffix='.tmp', dir=directory)
        try:
            os.fchmod(fd, 0o600)
            with os.fdopen(fd, 'w') as fh:
                json.dump(entry, fh)
                fh.flush()
                os.fsync(fh.fileno())
            os.replace(tmp, cache_file(env_file, base))
        except BaseException:
            try:
                os.unlink(tmp)
            except OSError:
                pass
            raise
        return True
    except (OSError, TypeError, ValueError):
        return False


def same_environment(a: Dict[str, str], b: Dict[str, str]) -> bool:
    """Compare two sourced environments, ignoring `_`, the last-command variable bash sets."""
    return ({k: v for k, v in a.items() if k != '_'}
            == {k: v for k, v in b.items() if k != '_'})


def load_env(env_file: str, base: Dict[str, str],
             source: Callable[..., Dict[str, str]]) -> Dict[str, str]:
    """
    Return the sourced environment of env_file: from the cache when valid, else sourced.

    `source` is config.source_env_file (`source(path)` plain, `source(path,
    trace_path=...)` traced); ConfigError from the plain one propagates exactly
    as without the cache.
    """
    if not enabled():
        return source(env_file)
    hit = read(env_file, base)
    if hit is not None:
        return hit
    trace = None
    try:
        os.makedirs(cache_dir(), mode=0o700, exist_ok=True)
        fd, trace = tempfile.mkstemp(prefix='.trace-', dir=cache_dir())
        os.close(fd)
    except OSError:
        trace = None
    if trace is None:
        return source(env_file)
    try:
        # The plain sourcing is the one whose result is returned and cached,
        # exactly what today's CLI gets. A second, traced sourcing runs beside
        # it (same wall time: bash is single-threaded) only to find the files
        # to watch; it is cached only if it produced the same environment.
        traced = {}

        def run_traced():
            try:
                traced['env'] = source(env_file, trace_path=trace)
            except Exception as exc:  # noqa: BLE001 - a failed trace only means "no cache"
                traced['error'] = exc

        worker = threading.Thread(target=run_traced, daemon=True)
        worker.start()
        try:
            env = source(env_file)
        finally:
            worker.join(timeout=30.0)
        if 'env' in traced and same_environment(env, traced['env']):
            try:
                with open(trace, 'r', errors='replace') as fh:
                    files = traced_files(fh.read(), cwd=os.getcwd())
            except OSError:
                files = None
            if files is not None:
                write(env_file, base, env, files)
        return env
    finally:
        try:
            os.unlink(trace)
        except OSError:
            pass
