"""Where the MATLAB database lives: one resolver, used by every writer.

Order, first hit wins:

1. ``--db-root`` / ``--out`` on the command line;
2. ``$F1TENTH_MATLAB_DATA``;
3. ``matlab_export.db_root`` in the logger YAML
   (``src/f1tenth_logger/config/test_campaign_logger.yaml``, under the
   node's ``ros__parameters``; an empty string means "not set");
4. ``~/matlab_data``.

``tools/matlab/+f1db/default_root.m`` mirrors steps 2 and 4.
"""

import os
import tempfile
from pathlib import Path

import yaml

ENV_VAR = "F1TENTH_MATLAB_DATA"
DEFAULT_DB_ROOT = "~/matlab_data"
YAML_RELPATH = Path("src") / "f1tenth_logger" / "config" / "test_campaign_logger.yaml"
YAML_NODE = "test_campaign_logger"


def yaml_db_root(yaml_path):
    """``matlab_export.db_root`` from the logger YAML, or None."""
    try:
        with open(yaml_path, encoding="utf-8") as fh:
            config = yaml.safe_load(fh) or {}
    except OSError:
        return None
    params = (config.get(YAML_NODE) or {}).get("ros__parameters") or {}
    section = params.get("matlab_export") or {}
    value = section.get("db_root") if isinstance(section, dict) else None
    if value is None:  # also accept the flattened ROS spelling
        value = params.get("matlab_export.db_root")
    return str(value) if value not in (None, "") else None


def resolve_db_root(cli_value=None, env=None, yaml_path=None):
    """``(path, how)`` per the order in the module docstring. Creates nothing."""
    env = os.environ if env is None else env
    if cli_value:
        return Path(cli_value).expanduser().resolve(), "--db-root"
    if env.get(ENV_VAR):
        return Path(env[ENV_VAR]).expanduser().resolve(), f"${ENV_VAR}"
    if yaml_path is not None:
        value = yaml_db_root(yaml_path)
        if value:
            return Path(value).expanduser().resolve(), f"{yaml_path} matlab_export.db_root"
    return Path(DEFAULT_DB_ROOT).expanduser().resolve(), "default"


def get_db_root(cli_value=None, env=None, yaml_path=None):
    """Resolve, create ``runs/`` and ``index/``, prove it is writable.

    Raises SystemExit with the reason when the folder cannot be created or
    written to: a database that silently writes nowhere is worse than none.
    """
    root, how = resolve_db_root(cli_value, env, yaml_path)
    try:
        for sub in ("runs", "index"):
            (root / sub).mkdir(parents=True, exist_ok=True)
        with tempfile.NamedTemporaryFile(dir=root / "runs", prefix=".write_test_"):
            pass
    except OSError as exc:
        raise SystemExit(f"MATLAB db root {root} (from {how}) is not writable: {exc}")
    return root, how


def atomic_write_bytes(path, write):
    """Call ``write(fileobj)`` on ``<path>.tmp``, fsync, then os.replace it."""
    path = Path(path)
    tmp = path.with_name(path.name + ".tmp")
    try:
        with open(tmp, "wb") as fh:
            write(fh)
            fh.flush()
            os.fsync(fh.fileno())
        os.replace(tmp, path)
    finally:
        if tmp.exists():
            tmp.unlink()
    return path
