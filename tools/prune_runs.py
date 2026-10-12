#!/usr/bin/env python3
"""Prune recorded mission runs: the bag directory AND its runs.db row, together.

Stdlib only, same reason runs_db/runs_cli are: this has to work on a machine
with no ROS sourced.

WHAT A "RUN" IS HERE. A directory named YYYY-MM-DDTHH-MM-SS_mission-<name>
directly under one of the scoped roots. This archive has THREE such roots and
they are near-disjoint (only one run appears in two of them), so all three are
scoped by default:

    ~/f1tenth_archive/complete/     the main archive
    ~/f1tenth_archive/mission_row/  the pre-consolidation layout
    ~/f1tenth_archive/active/       recordings never consolidated into complete/

THE REGISTRATION is a row in ~/f1tenth_runs/runs.db, keyed by run_id. Read
runs_db's module docstring before touching it: that database is a CACHE over
the *.manifest.json files, never a record, and `runs import` rebuilds it. That
is why deleting a row is safe here, and why this tool does not try to repair
the ones it did not break.

ORDER OF OPERATIONS, and it is not arbitrary: the db row is deleted and
committed FIRST, then the directory tree. The two failure modes are not
symmetric --

    a directory with no row   an unregistered run. Harmless, and already the
                              state of 121 of the 122 runs in complete/.
    a row with no directory   a registration pointing at a bag that does not
                              exist. This is the one the caller forbade.

so the write order is chosen to make the second one unreachable. A run whose
row cannot be deleted (locked or read-only db) is skipped whole and reported,
rather than half-deleted.
"""

import argparse
import os
import re
import shutil
import sqlite3
import sys
from datetime import datetime, timedelta

DEFAULT_ARCHIVE = '~/f1tenth_archive'
DEFAULT_RUNS_DB = '~/f1tenth_runs/runs.db'
DEFAULT_DIRS = ('complete', 'mission_row', 'active')
DEFAULT_MAX_DELETE = 50

# YYYY-MM-DDTHH-MM-SS_mission-<name>. Anything that does not match is not a run
# directory and is never a deletion candidate -- see collect().
RUN_RE = re.compile(r'^(\d{4}-\d{2}-\d{2})T(\d{2})-(\d{2})-(\d{2})_mission-(?P<mission>.+)$')

# A bag whose last write was this recent is treated as possibly still being
# written even with no other evidence. Cheap insurance against a recorder that
# has closed its WAL between our two checks.
FRESH_WRITE_SEC = 120


class Run:
    """One run directory, its parsed name, and its size on disk."""

    def __init__(self, root_name, path, name, started, mission):
        self.root_name = root_name
        self.path = path
        self.name = name
        self.started = started
        self.mission = mission
        self.bytes = _tree_bytes(path)
        self.in_progress = _in_progress_reason(path)

    def __repr__(self):
        return f'<Run {self.root_name}/{self.name}>'


def _tree_bytes(path):
    """Bytes on disk under path, not following symlinks out of it."""
    total = 0
    for dirpath, dirnames, filenames in os.walk(path, followlinks=False):
        for fn in filenames:
            fp = os.path.join(dirpath, fn)
            try:
                st = os.lstat(fp)
            except OSError:
                continue
            total += st.st_size
    return total


def _open_fds_under(path):
    """Processes holding an open file under path, via /proc. Linux only.

    Cheaper and more dependable than shelling out to lsof, and this only ever
    reads -- a process we cannot inspect (permissions) is simply not counted,
    which is why it is one signal among several rather than the only one.
    """
    holders = set()
    real = os.path.realpath(path) + os.sep
    try:
        pids = [p for p in os.listdir('/proc') if p.isdigit()]
    except OSError:
        return holders
    for pid in pids:
        fd_dir = f'/proc/{pid}/fd'
        try:
            for fd in os.listdir(fd_dir):
                try:
                    target = os.readlink(os.path.join(fd_dir, fd))
                except OSError:
                    continue
                if target.startswith(real):
                    holders.add(pid)
                    break
        except OSError:
            continue
    return holders


def _in_progress_reason(path):
    """Why this run looks like it is still being recorded, or None.

    Four signals, any one of which is enough. A bag being written is the one
    thing here that is genuinely unrecoverable if deleted, so this errs toward
    calling a run live.
    """
    bag = os.path.join(path, 'bag')
    if os.path.isdir(bag):
        try:
            entries = os.listdir(bag)
        except OSError as e:
            return f'bag/ unreadable ({e})'
        # rosbag2/sqlite3 keeps -wal/-shm alongside the .db3 only while open.
        sidecars = [e for e in entries if e.endswith(('.db3-wal', '.db3-shm'))]
        if sidecars:
            return f'sqlite sidecars present ({", ".join(sorted(sidecars))})'
        # metadata.yaml is written when the recorder closes the bag.
        if not any(e == 'metadata.yaml' for e in entries):
            return 'bag/metadata.yaml missing (recorder never closed it?)'
        newest = 0
        for e in entries:
            try:
                newest = max(newest, os.lstat(os.path.join(bag, e)).st_mtime)
            except OSError:
                continue
        age = datetime.now().timestamp() - newest
        if age < FRESH_WRITE_SEC:
            return f'bag written {age:.0f}s ago (< {FRESH_WRITE_SEC}s)'
    holders = _open_fds_under(path)
    if holders:
        return f'open file handles held by pid(s) {", ".join(sorted(holders))}'
    return None


def human(n):
    """Bytes as a short human string."""
    v = float(n)
    for unit in ('B', 'K', 'M', 'G', 'T'):
        if v < 1024.0:
            return f'{v:.1f}{unit}'
        v /= 1024.0
    return f'{v:.1f}P'


def parse_run_name(name):
    """(datetime, mission) for a run directory name, or (None, None)."""
    m = RUN_RE.match(name)
    if not m:
        return None, None
    date, hh, mm, ss = m.group(1), m.group(2), m.group(3), m.group(4)
    try:
        started = datetime.strptime(f'{date}T{hh}:{mm}:{ss}', '%Y-%m-%dT%H:%M:%S')
    except ValueError:
        return None, None
    return started, m.group('mission')


def _assert_contained(path, root):
    """Refuse any path that is not strictly inside root, symlinks resolved.

    Requirement 7 in one function: everything this tool deletes goes through
    here first, so a crafted or symlinked run name cannot reach outside the
    scoped roots.
    """
    real_root = os.path.realpath(root)
    real_path = os.path.realpath(path)
    if real_path == real_root:
        raise ValueError(f'refusing to operate on the root itself: {path}')
    if os.path.commonpath([real_root, real_path]) != real_root:
        raise ValueError(f'path escapes its root: {path} not inside {root}')
    if os.path.dirname(real_path) != real_root:
        raise ValueError(f'not a direct child of {root}: {path}')


def collect(archive, dir_names):
    """Every run directory under the scoped roots, plus anything unrecognised.

    Returns (runs, strays, empty_roots). `strays` are entries that do not parse
    as run names: they are reported and NEVER deleted, because an unrecognised
    name is exactly the case where the tool's model of the layout is wrong.
    """
    runs, strays, empty_roots = [], [], []
    for dname in dir_names:
        root = os.path.join(archive, dname)
        if not os.path.isdir(root):
            raise SystemExit(
                f'ERRORE: {root} non esiste. Mi rifiuto di continuare -- una '
                'directory mancante non significa "cancella tutto".')
        entries = sorted(os.listdir(root))
        if not entries:
            empty_roots.append(root)
            continue
        for entry in entries:
            path = os.path.join(root, entry)
            if not os.path.isdir(path):
                strays.append((dname, entry, 'not a directory'))
                continue
            started, mission = parse_run_name(entry)
            if started is None:
                strays.append((dname, entry, 'name does not parse as a run'))
                continue
            runs.append(Run(dname, path, entry, started, mission))
    return runs, strays, empty_roots


def read_keep_list(path):
    """One run name per line; blank lines and # comments ignored."""
    names = []
    with open(os.path.expanduser(path), encoding='utf-8') as fh:
        for line in fh:
            line = line.split('#', 1)[0].strip()
            if line:
                names.append(line)
    return names


def select(runs, keep_names, keep_days, keep_last, keep_newer):
    """Decide what to keep. Returns (keep_reason_by_run_name, unmatched_names).

    The three modes UNION rather than intersect: a run kept by any active mode
    is kept. That is the safe direction -- combining modes can only ever spare
    more runs, never delete something a mode wanted to keep.
    """
    keep = {}

    if keep_names is not None:
        wanted = set(keep_names)
        for run in runs:
            if run.name in wanted:
                keep.setdefault(run.name, 'on keep-list')
        matched = {r.name for r in runs}
        unmatched = sorted(n for n in wanted if n not in matched)

        # A static list goes stale the moment a new run is recorded, and the
        # caller must not lose tomorrow's data to yesterday's list. Anything
        # newer than the newest thing the list mentions is kept.
        if keep_newer and keep_names:
            newest = None
            for n in keep_names:
                started, _ = parse_run_name(n)
                if started and (newest is None or started > newest):
                    newest = started
            if newest is not None:
                for run in runs:
                    if run.started > newest:
                        keep.setdefault(
                            run.name, f'newer than keep-list (> {newest:%Y-%m-%dT%H-%M-%S})')
    else:
        unmatched = []

    if keep_days is not None:
        cutoff = datetime.now() - timedelta(days=keep_days)
        for run in runs:
            if run.started >= cutoff:
                keep.setdefault(run.name, f'within --keep-days {keep_days}')

    if keep_last is not None:
        by_mission = {}
        for run in runs:
            by_mission.setdefault(run.mission, []).append(run)
        for mission, group in by_mission.items():
            group.sort(key=lambda r: r.started, reverse=True)
            for run in group[:keep_last]:
                keep.setdefault(run.name, f'in --keep-last {keep_last} for {mission!r}')

    return keep, unmatched


def load_registrations(db_path):
    """{run_id: True} for rows present in runs.db, or None if there is no db."""
    if not os.path.isfile(db_path):
        return None
    conn = sqlite3.connect(db_path)
    try:
        rows = conn.execute('select run_id from runs').fetchall()
    except sqlite3.Error as e:
        raise SystemExit(f'ERRORE: {db_path} non leggibile come runs.db ({e})')
    finally:
        conn.close()
    return {r[0] for r in rows}


def db_is_writable(db_path):
    """Can we actually delete a row? Checked BEFORE deleting any directory."""
    if not os.path.isfile(db_path):
        return False, 'no runs.db'
    if not os.access(db_path, os.W_OK):
        return False, 'runs.db is not writable'
    try:
        conn = sqlite3.connect(db_path, timeout=5.0)
        conn.execute('begin immediate')
        conn.rollback()
        conn.close()
    except sqlite3.Error as e:
        return False, f'runs.db not lockable ({e})'
    return True, ''


def delete_run(run, db_path, has_row):
    """Row first (committed), then the tree. See the module docstring."""
    if has_row:
        conn = sqlite3.connect(db_path, timeout=10.0)
        try:
            with conn:
                conn.execute('delete from runs where run_id = ?', (run.name,))
        finally:
            conn.close()
    shutil.rmtree(run.path)


def main(argv=None):
    """Entry point."""
    ap = argparse.ArgumentParser(
        prog='prune_runs',
        description='Delete old mission runs and their runs.db registrations. '
                    'Dry-run unless --apply is given.')
    ap.add_argument('--archive', default=DEFAULT_ARCHIVE,
                    help=f'archive root (default {DEFAULT_ARCHIVE})')
    ap.add_argument('--runs-db', default=DEFAULT_RUNS_DB,
                    help=f'registration database (default {DEFAULT_RUNS_DB})')
    ap.add_argument('--dirs', default=','.join(DEFAULT_DIRS),
                    help=f'comma-separated run roots (default {",".join(DEFAULT_DIRS)})')
    ap.add_argument('--keep-list', help='file of run names to keep, one per line')
    ap.add_argument('--keep-days', type=int, help='keep runs newer than N days')
    ap.add_argument('--keep-last', type=int,
                    help='keep the N most recent runs of each mission name')
    ap.add_argument('--no-keep-newer', action='store_true',
                    help='do NOT automatically keep runs newer than the keep-list')
    ap.add_argument('--max-delete', type=int, default=DEFAULT_MAX_DELETE,
                    help=f'abort if more than this many runs would go '
                         f'(default {DEFAULT_MAX_DELETE})')
    ap.add_argument('--limit', type=int,
                    help='process at most N candidates, oldest first (for batching)')
    ap.add_argument('--apply', action='store_true', help='actually delete')
    args = ap.parse_args(argv)

    archive = os.path.expanduser(args.archive)
    db_path = os.path.expanduser(args.runs_db)
    dir_names = [d.strip() for d in args.dirs.split(',') if d.strip()]

    if not os.path.isdir(archive):
        raise SystemExit(f'ERRORE: archivio non trovato: {archive}')
    if args.keep_list is None and args.keep_days is None and args.keep_last is None:
        raise SystemExit(
            'ERRORE: nessun criterio di conservazione. Serve almeno uno fra '
            '--keep-list, --keep-days, --keep-last -- senza, questo strumento '
            'cancellerebbe ogni run.')

    runs, strays, empty_roots = collect(archive, dir_names)
    if empty_roots:
        raise SystemExit(
            'ERRORE: root vuoti: ' + ', '.join(empty_roots)
            + '. Mi rifiuto di continuare: un root vuoto non significa '
              '"cancella tutto il resto".')
    if not runs:
        raise SystemExit('ERRORE: nessun run trovato. Nulla da fare.')

    keep_names = read_keep_list(args.keep_list) if args.keep_list else None
    keep, unmatched = select(runs, keep_names, args.keep_days, args.keep_last,
                             not args.no_keep_newer)

    registered = load_registrations(db_path)
    db_ok, db_why = db_is_writable(db_path)

    candidates = sorted((r for r in runs if r.name not in keep),
                        key=lambda r: r.started)

    skipped = []
    deletable = []
    for run in candidates:
        if run.in_progress:
            skipped.append((run, f'REGISTRAZIONE IN CORSO: {run.in_progress}'))
            continue
        has_row = registered is not None and run.name in registered
        if has_row and not db_ok:
            skipped.append((run, f'ha una riga in runs.db ma {db_why} -- '
                                 'cancellarne solo un lato lascerebbe una '
                                 'registrazione senza bag'))
            continue
        if not os.access(run.path, os.W_OK):
            skipped.append((run, 'directory non scrivibile'))
            continue
        try:
            _assert_contained(run.path, os.path.join(archive, run.root_name))
        except ValueError as e:
            skipped.append((run, f'controllo di contenimento fallito: {e}'))
            continue
        deletable.append((run, has_row))

    if args.limit is not None:
        deletable = deletable[:args.limit]

    # A run_id can appear under more than one root (this archive has exactly
    # one such run, in complete/ AND mission_row/), and both copies share the
    # ONE registration row keyed by that id. Only the first copy deleted is
    # the one that removes the row; the second would otherwise be reported as
    # deleting a row that is already gone. Re-labelled rather than reordered:
    # dropping the row with the first copy briefly leaves the second copy
    # unregistered, which is the safe direction (the forbidden one is a row
    # outliving its bag).
    seen_ids = set()
    labelled = []
    for run, has_row in deletable:
        shared = has_row and run.name in seen_ids
        seen_ids.add(run.name)
        labelled.append((run, has_row and not shared, shared))
    deletable = labelled

    # ---- report ---------------------------------------------------------
    print(f'archivio      {archive}')
    print(f'root          {", ".join(dir_names)}')
    print(f'runs.db       {db_path}'
          + ('' if registered is not None else '  (ASSENTE: nessuna registrazione)'))
    if registered is not None:
        print(f'              {len(registered)} righe, scrivibile={db_ok}'
              + ('' if db_ok else f' ({db_why})'))
    print(f'run trovati   {len(runs)}   da conservare {len(keep)}   '
          f'candidati {len(candidates)}')
    print()

    if strays:
        print(f'IGNORATI (nome non riconosciuto, mai cancellati): {len(strays)}')
        for dname, entry, why in strays:
            print(f'  {dname}/{entry} -- {why}')
        print()

    if unmatched:
        print(f'KEEP-LIST SENZA CORRISPONDENZA: {len(unmatched)} '
              '(la lista e i nomi su disco sono divergenti)')
        for name in unmatched:
            print(f'  {name}')
        print()

    if skipped:
        print(f'SALTATI: {len(skipped)}')
        for run, why in skipped:
            print(f'  {run.root_name}/{run.name}  [{human(run.bytes)}]  {why}')
        print()

    total = sum(r.bytes for r, _, _ in deletable)
    verb = 'CANCELLO' if args.apply else 'CANCELLEREI'
    print(f'{verb} {len(deletable)} run, liberando {human(total)}:')
    for run, has_row, shared in deletable:
        if shared:
            reg = 'riga runs.db gia\' rimossa con l\'altra copia'
        elif has_row:
            reg = 'riga runs.db'
        else:
            reg = 'nessuna riga'
        print(f'  {run.root_name}/{run.name}  [{human(run.bytes)}]  ({reg})')
    print()
    print(f'TOTALE: {len(deletable)} run, {human(total)}')

    if len(deletable) > args.max_delete:
        message = (f'{len(deletable)} cancellazioni superano --max-delete '
                   f'{args.max_delete}. Usa --limit {args.max_delete} per '
                   'procedere a lotti, oppure alza --max-delete '
                   'consapevolmente.')
        if args.apply:
            raise SystemExit(f'\nABORT: {message}')
        print(f'\nATTENZIONE: {message}')
        print('(dry-run: la lista sopra e\' comunque completa)')
        return 0

    if not args.apply:
        print('\ndry-run: nulla e\' stato cancellato. Aggiungi --apply.')
        return 0

    done, failed = 0, []
    for run, has_row, _shared in deletable:
        try:
            delete_run(run, db_path, has_row)
            done += 1
        except Exception as e:                      # noqa: BLE001 - reported, not swallowed
            failed.append((run, e))
    print(f'\ncancellati {done} run.')
    for run, e in failed:
        print(f'  FALLITO {run.root_name}/{run.name}: {e}', file=sys.stderr)
    return 1 if failed else 0


if __name__ == '__main__':
    sys.exit(main())
