"""
SQLite index over mission-run manifests -- a CACHE, never a record.

THE ONE INVARIANT THIS MODULE EXISTS TO PROTECT: every byte in runs.db is
derivable from the *.manifest.json files on disk. The database is never the
authority for anything, is never merged back into a manifest, and can be
deleted at any moment and rebuilt with `runs import`. That is what makes a
run portable to another machine (and what keeps a binary blob out of git --
see the runs-repo .gitignore).

Data flows in exactly ONE direction:

    manifest.json  ->  runs.db          (import_dir / sync_manifest)
    manifest.json  <-  runs note/rename (write_manifest_fields, then re-sync)

`runs note` and `runs rename` therefore write the MANIFEST first and only
then refresh the row from it. Notes especially: everything else here
regenerates (bags re-render, reports rebuild, the db re-imports), but a human
being's interpretation of why a run went the way it did does not, so it lives
in the versioned artifact rather than in the disposable one. A DB-only note
would never be committed anywhere.

NOTHING HERE MAY ASSUME A BAG EXISTS. The archive machine holds manifests and
usually no bags at all; import/list/show/note/rename must all work against a
directory of pure manifests. bag_present is therefore a computed, per-call
property (see bag_status) and deliberately NOT a stored column: it describes
this machine right now, not the run, and caching it in the row would make the
db wrong the moment a bag is archived off or rsync'd in.
"""

import json
import os
import sqlite3
from datetime import datetime, timezone

# Columns exist to make `runs list` and lookup fast, NOT to restate the
# manifest. Anything not needed for listing/filtering stays in the manifest and
# is read from there on demand (resolved_params, storage_id, the git dirty
# rationale, ...). Adding a column is a decision to duplicate, so add sparingly.
SCHEMA = """
CREATE TABLE IF NOT EXISTS runs (
  run_id               TEXT PRIMARY KEY,   -- immutable, == manifest run_id
  name                 TEXT UNIQUE,        -- renameable label, defaults to run_id
  mission_id           TEXT,
  mission_json_path    TEXT,
  git_branch           TEXT,
  git_commit           TEXT,
  git_dirty            INTEGER,            -- 0/1; a commit alone names the wrong
                                           -- tree whenever this is 1
  start_time           TEXT,
  end_time             TEXT,
  duration_s           REAL,
  preroll_s            REAL,
  postroll_s           REAL,
  outcome              TEXT,
  stop_reason          TEXT,
  bag_path             TEXT,
  params_snapshot_path TEXT,
  manifest_path        TEXT,
  run_dir              TEXT,
  notes                TEXT DEFAULT ''     -- rendered copy of the manifest's
                                           -- notes list, for listing only
);
"""

_COLUMNS = (
    'run_id', 'name', 'mission_id', 'mission_json_path', 'git_branch', 'git_commit',
    'git_dirty', 'start_time', 'end_time', 'duration_s', 'preroll_s', 'postroll_s',
    'outcome', 'stop_reason', 'bag_path', 'params_snapshot_path', 'manifest_path',
    'run_dir', 'notes',
)


def default_runs_dir():
    return os.path.expanduser(os.environ.get('F1TENTH_RUNS_DIR', '~/f1tenth_runs'))


def connect(runs_dir):
    """
    Open (creating if needed) runs.db under runs_dir, in WAL mode.

    WAL so a reader (`runs list`, analyze_run) never blocks on the logger
    writing a lifecycle transition, and so a crash mid-write leaves a
    recoverable file rather than a truncated one.
    """
    os.makedirs(runs_dir, exist_ok=True)
    conn = sqlite3.connect(os.path.join(runs_dir, 'runs.db'))
    conn.row_factory = sqlite3.Row
    conn.execute('PRAGMA journal_mode=WAL')
    conn.executescript(SCHEMA)
    conn.commit()
    return conn


# ----------------------------------------------------------------------------
# manifest -> row


def _iso_to_dt(value):
    if not value:
        return None
    try:
        return datetime.fromisoformat(value)
    except ValueError:
        return None


def _duration_s(manifest):
    """
    Mission duration in seconds, or None.

    Prefers the explicit mission_start_time/mission_end_time markers the
    continuous-recording logger writes (which exclude pre/post-roll) and falls
    back to start_time/end_time, which is what every manifest recorded before
    that change carries. A manifest with neither pair complete yields None
    rather than 0.0, so "unknown" stays distinguishable from "instantaneous".
    """
    for first, last in (('mission_start_time', 'mission_end_time'),
                        ('start_time', 'end_time')):
        a, b = _iso_to_dt(manifest.get(first)), _iso_to_dt(manifest.get(last))
        if a and b:
            return (b - a).total_seconds()
    return None


def render_notes(manifest):
    """
    Flatten the manifest's notes list into one text blob for the row.

    Notes live in the manifest as a list of {timestamp, text} so they stay
    append-only and individually attributable; the row carries only a rendered
    copy, because the row is a search index, not the record.
    """
    notes = manifest.get('notes') or []
    if isinstance(notes, str):            # tolerate a hand-edited scalar
        return notes
    return '\n'.join(f"[{n.get('timestamp', '?')}] {n.get('text', '')}" for n in notes)


def row_from_manifest(manifest, manifest_path, runs_dir):
    run_id = manifest.get('run_id') or os.path.basename(manifest_path).split('.')[0]
    git = manifest.get('git') or {}
    return {
        'run_id': run_id,
        'name': manifest.get('name') or run_id,
        'mission_id': manifest.get('mission_id'),
        'mission_json_path': manifest.get('mission_json_path'),
        'git_branch': git.get('branch'),
        'git_commit': git.get('commit'),
        'git_dirty': 1 if git.get('dirty') else 0,
        'start_time': manifest.get('mission_start_time') or manifest.get('start_time'),
        'end_time': manifest.get('mission_end_time') or manifest.get('end_time'),
        'duration_s': _duration_s(manifest),
        'preroll_s': manifest.get('preroll_s'),
        'postroll_s': manifest.get('postroll_s'),
        'outcome': manifest.get('outcome') or 'UNKNOWN',
        'stop_reason': manifest.get('stop_reason'),
        'bag_path': manifest.get('bag_path'),
        'params_snapshot_path': manifest.get('params_snapshot_path'),
        'manifest_path': os.path.abspath(manifest_path),
        'run_dir': os.path.join(runs_dir, run_id),
        'notes': render_notes(manifest),
    }


def read_manifest(path):
    with open(path) as f:
        return json.load(f)


def write_manifest_fields(path, updates):
    """
    Merge `updates` into the manifest at `path`, atomically.

    Atomic because a manifest is the irreplaceable artifact here: a partial
    write during a note edit would destroy the one thing that cannot be
    regenerated. Written to a sibling temp file and renamed, so a reader either
    sees the old complete file or the new complete file, never a half one.
    """
    manifest = read_manifest(path)
    manifest.update(updates)
    tmp = f'{path}.tmp'
    with open(tmp, 'w') as f:
        json.dump(manifest, f, indent=2, sort_keys=True)
        f.write('\n')
        f.flush()
        os.fsync(f.fileno())
    os.replace(tmp, path)
    return manifest


def sync_manifest(conn, manifest_path, runs_dir):
    """Upsert one manifest's row. Overwrites every column: the manifest wins."""
    row = row_from_manifest(read_manifest(manifest_path), manifest_path, runs_dir)
    placeholders = ', '.join('?' * len(_COLUMNS))
    conn.execute(
        f'INSERT OR REPLACE INTO runs ({", ".join(_COLUMNS)}) VALUES ({placeholders})',
        [row[c] for c in _COLUMNS])
    conn.commit()
    return row


def import_dir(conn, source_dir, runs_dir):
    """
    Import every *.manifest.json under source_dir. Idempotent.

    Re-running re-syncs each row FROM its manifest, which is exactly why name
    and notes must live in the manifest: anything held only in the row would be
    silently overwritten here.
    """
    imported, failed = [], []
    for entry in sorted(os.listdir(source_dir)):
        if not entry.endswith('.manifest.json'):
            continue
        path = os.path.join(source_dir, entry)
        try:
            imported.append(sync_manifest(conn, path, runs_dir))
        except (OSError, ValueError, KeyError) as exc:
            failed.append((entry, str(exc)))
    return imported, failed


# ----------------------------------------------------------------------------
# lookup


def find(conn, key):
    """Resolve a run by run_id first, then by name. Returns a row or None."""
    for column in ('run_id', 'name'):
        row = conn.execute(
            f'SELECT * FROM runs WHERE {column} = ?', (key,)).fetchone()
        if row:
            return row
    return None


def all_runs(conn):
    """Newest first. start_time is ISO-8601 UTC, so it sorts lexicographically."""
    return conn.execute('SELECT * FROM runs ORDER BY start_time DESC').fetchall()


def bag_status(row):
    """
    ('present'|'absent'|'unknown', path) for this machine, computed live.

    Never stored: whether a bag is on THIS disk is a property of the machine,
    not of the run, and a stored copy would be wrong the moment a bag is
    archived off or synced in.
    """
    path = row['bag_path']
    if not path:
        return 'unknown', None
    return ('present' if os.path.isdir(path) else 'absent'), path


def append_note(conn, row, text, runs_dir):
    """Append a timestamped note TO THE MANIFEST, then refresh the row from it."""
    manifest = read_manifest(row['manifest_path'])
    notes = manifest.get('notes') or []
    if isinstance(notes, str):
        notes = [{'timestamp': 'legacy', 'text': notes}]
    notes.append({
        'timestamp': datetime.now(timezone.utc).isoformat(),
        'text': text,
    })
    write_manifest_fields(row['manifest_path'], {'notes': notes})
    return sync_manifest(conn, row['manifest_path'], runs_dir)


def rename(conn, row, new_name, runs_dir):
    """Set the manifest's `name`, then refresh the row. run_id never changes."""
    clash = conn.execute(
        'SELECT run_id FROM runs WHERE name = ? AND run_id != ?',
        (new_name, row['run_id'])).fetchone()
    if clash:
        raise ValueError(f'name {new_name!r} is already used by {clash["run_id"]}')
    write_manifest_fields(row['manifest_path'], {'name': new_name})
    return sync_manifest(conn, row['manifest_path'], runs_dir)
