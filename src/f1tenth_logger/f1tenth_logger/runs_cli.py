"""
`runs` -- list, annotate and inspect recorded missions.

Stdlib only (argparse + sqlite3 via runs_db): this has to work on a machine
that holds nothing but manifests, with no ROS sourced and no bags on disk.
Every subcommand here is therefore bag-independent; the only thing that needs
a bag is analyze_run, which says so in one line rather than failing deep in a
reader.

See runs_db's own module docstring for the one-directional
manifest -> db rule that `note` and `rename` are built around.
"""

import argparse
import os
import sys

from f1tenth_logger import runs_db


def _resolve(conn, key):
    row = runs_db.find(conn, key)
    if row is None:
        print(f"no run matching {key!r} (try `runs list`)", file=sys.stderr)
        raise SystemExit(1)
    return row


def cmd_import(args, conn):
    source = os.path.expanduser(args.source)
    if not os.path.isdir(source):
        print(f'not a directory: {source}', file=sys.stderr)
        return 1
    imported, failed = runs_db.import_dir(conn, source, args.runs_dir)
    for name, err in failed:
        print(f'  SKIPPED {name}: {err}', file=sys.stderr)
    print(f'imported {len(imported)} run(s) from {source}'
          + (f', {len(failed)} skipped' if failed else ''))
    return 0


def cmd_consolidate(args, conn):
    """
    Migrate loose bags into the per-run archive layout, then index them.

    Separate subcommand rather than a flag on `import`: this MOVES real run
    data, and a destructive-shaped operation should have to be named, not
    ridden in on a routine one. Extraction needs ROS and is imported lazily by
    runs_migrate, so every other subcommand stays usable without it.
    """
    from f1tenth_logger import runs_migrate

    source = os.path.expanduser(args.source)
    archive = os.path.expanduser(args.archive_dir)
    videos = os.path.expanduser(args.video_dir) if args.video_dir else None
    if not os.path.isdir(source):
        print(f'not a directory: {source}', file=sys.stderr)
        return 1

    reports, orphans = runs_migrate.consolidate_dir(
        source, archive, videos, extract=not args.no_extract)

    total_bag = total_extract = 0
    problems = []
    for report in reports:
        actions = ', '.join(report['actions']) or 'nothing to do'
        print(f'{report["run_id"]}: {actions}')
        total_bag += report.get('bag_bytes', 0)
        total_extract += report.get('extract_bytes', 0)
        for problem in report['problems']:
            problems.append(f'{report["run_id"]}: {problem}')

    imported, failed = runs_db.import_dir(
        conn, os.path.join(archive, 'complete'), args.runs_dir)
    # Manifests now live one directory deeper (one folder per run), so index
    # each run folder as well as the top level.
    for report in reports:
        manifest = os.path.join(report['dest'], f'{report["run_id"]}.manifest.json')
        if os.path.isfile(manifest):
            runs_db.sync_manifest(conn, manifest, args.runs_dir)

    print(f'\nconsolidated {len(reports)} run(s) into {archive}/complete/')
    if total_bag:
        ratio = (total_bag / total_extract) if total_extract else float('inf')
        print(f'  bags {total_bag / 1e9:.2f} GB, extracts '
              f'{total_extract / 1e6:.1f} MB ({ratio:.0f}x smaller)')
    print(f'  indexed {len(runs_db.all_runs(conn))} run(s) in runs.db')

    # Reported, never guessed at: pairing these up by mtime or name similarity
    # would silently attach one run's artifacts to another run's record.
    if orphans:
        print(f'\nUNMATCHED videos ({len(orphans)}) -- no run_id matches these, '
              'left where they are:')
        for name in orphans:
            print(f'  {name}')
    if problems:
        print(f'\nPROBLEMS ({len(problems)}):')
        for problem in problems:
            print(f'  {problem}')
    if failed:
        for name, err in failed:
            print(f'  SKIPPED {name}: {err}', file=sys.stderr)
    return 0


def cmd_list(args, conn):
    rows = runs_db.all_runs(conn)
    if not rows:
        print('no runs (try `runs import <dir of *.manifest.json>`)')
        return 0
    print(f'{"NAME":42s} {"MISSION":22s} {"OUTCOME":14s} {"DUR":>7s} {"BAG":8s} STARTED')
    print('-' * 118)
    for row in rows:
        state, _ = runs_db.bag_status(row)
        dur = f'{row["duration_s"]:.1f}s' if row['duration_s'] is not None else '-'
        # An outcome still reading RECORDING means the logger died mid-run
        # rather than finalizing; flagged rather than shown as a normal state.
        outcome = row['outcome'] or '?'
        if outcome == 'RECORDING':
            outcome += ' (!)'
        print(f'{row["name"][:42]:42s} {(row["mission_id"] or "-")[:22]:22s} '
              f'{outcome[:14]:14s} {dur:>7s} {state:8s} {row["start_time"] or "-"}')
    stale = sum(1 for r in rows if r['outcome'] == 'RECORDING')
    if stale:
        print(f'\n(!) {stale} run(s) left at RECORDING: the logger did not finalize '
              'them, so their data may be incomplete.')
    return 0


def cmd_show(args, conn):
    row = _resolve(conn, args.key)
    state, bag_path = runs_db.bag_status(row)
    print(f'run_id       {row["run_id"]}')
    print(f'name         {row["name"]}')
    print(f'mission      {row["mission_id"]}  ({row["mission_json_path"]})')
    dirty = ' +dirty' if row['git_dirty'] else ''
    print(f'code         {row["git_branch"]} @ {row["git_commit"]}{dirty}')
    print(f'started      {row["start_time"]}')
    print(f'ended        {row["end_time"]}')
    if row['duration_s'] is not None:
        print(f'duration     {row["duration_s"]:.2f}s')
    for label, key in (('pre-roll', 'preroll_s'), ('post-roll', 'postroll_s')):
        if row[key] is not None:
            print(f'{label:12s} {row[key]:.2f}s')
    print(f'outcome      {row["outcome"]}'
          + (f'  ({row["stop_reason"]})' if row['stop_reason'] else ''))
    print(f'bag          {state}: {bag_path}')
    print(f'params       {row["params_snapshot_path"]}')
    print(f'manifest     {row["manifest_path"]}')
    print(f'run_dir      {row["run_dir"]}')
    if row['notes']:
        print('notes')
        for line in row['notes'].splitlines():
            print(f'  {line}')
    return 0


def cmd_note(args, conn):
    row = _resolve(conn, args.key)
    updated = runs_db.append_note(conn, row, args.text, args.runs_dir)
    print(f'noted on {updated["run_id"]} (written to {row["manifest_path"]})')
    return 0


def cmd_rename(args, conn):
    row = _resolve(conn, args.key)
    try:
        updated = runs_db.rename(conn, row, args.new_name, args.runs_dir)
    except ValueError as exc:
        print(str(exc), file=sys.stderr)
        return 1
    print(f'{updated["run_id"]} is now named {updated["name"]!r} '
          f'(written to {row["manifest_path"]})')
    return 0


def build_parser():
    ap = argparse.ArgumentParser(prog='runs', description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--runs-dir', default=runs_db.default_runs_dir(),
                    help='holds runs.db and per-run artifact folders '
                         '(default: %(default)s)')
    sub = ap.add_subparsers(dest='command', required=True)

    p = sub.add_parser('import', help='build/refresh rows from a directory of '
                                      '*.manifest.json (idempotent)')
    p.add_argument('source')
    p.set_defaults(func=cmd_import)

    p = sub.add_parser('consolidate',
                       help='MOVE loose bags/manifests into one folder per run '
                            'under <archive>/complete/, extract each, and index')
    p.add_argument('source', help='directory holding the loose *.manifest.json')
    p.add_argument('--archive-dir', default='~/f1tenth_archive')
    p.add_argument('--video-dir', default=None,
                   help='copy an already-rendered <run_id>.mp4 in from here')
    p.add_argument('--no-extract', action='store_true',
                   help='skip parquet extraction (it needs ROS sourced)')
    p.set_defaults(func=cmd_consolidate)

    p = sub.add_parser('list', help='every run, newest first')
    p.set_defaults(func=cmd_list)

    p = sub.add_parser('show', help='one run in full')
    p.add_argument('key', help='run_id or name')
    p.set_defaults(func=cmd_show)

    p = sub.add_parser('note', help='append a timestamped note (to the manifest)')
    p.add_argument('key')
    p.add_argument('text')
    p.set_defaults(func=cmd_note)

    p = sub.add_parser('rename', help='set the renameable label (to the manifest)')
    p.add_argument('key')
    p.add_argument('new_name')
    p.set_defaults(func=cmd_rename)
    return ap


def main(argv=None):
    args = build_parser().parse_args(argv)
    args.runs_dir = os.path.expanduser(args.runs_dir)
    conn = runs_db.connect(args.runs_dir)
    try:
        return args.func(args, conn)
    finally:
        conn.close()


if __name__ == '__main__':
    sys.exit(main())
