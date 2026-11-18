"""pytest setup for f1tenth_ctl's tests: the env cache is off unless a test turns it on."""

import os
import shutil
import tempfile

import pytest


@pytest.fixture(autouse=True)
def _no_env_cache(monkeypatch):
    """Keep tests off the real cache directory: they source their own env files."""
    monkeypatch.setenv('F1T_ENV_CACHE', '0')


@pytest.fixture
def sock():
    """Yield a short unix socket path (unix paths are limited to ~100 characters)."""
    d = tempfile.mkdtemp(prefix='md')
    yield os.path.join(d, 'm.sock')
    shutil.rmtree(d, ignore_errors=True)
