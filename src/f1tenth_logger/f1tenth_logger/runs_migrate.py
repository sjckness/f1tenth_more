"""
One-time consolidation of loose bags into the per-run archive layout.

NO LONGER PART OF THE NORMAL FLOW. mission_logger_node now writes the archive
layout directly -- recording into <runs_dir>/active/<run_id>/ and renaming the
whole folder to <runs_dir>/complete/<run_id>/ at finalize, where it also writes
the checksums, extract and snapshots this module used to add afterwards. A run
recorded today needs no consolidate step at all.

What is left here is a one-off migration tool for stragglers still sitting in
the old flat ~/.ros/mission_bags/ location from before that change. Kept rather
than deleted precisely because those stragglers are real run data that cannot
be re-collected; reach for it when adopting an old directory, not as routine
housekeeping.

BEFORE (what mission_logger_node wrote until that change): three siblings per
run scattered in one flat directory, plus videos in a fourth place entirely.

    ~/.ros/mission_bags/<run_id>/                 the bag
    ~/.ros/mission_bags/<run_id>.manifest.json
    ~/.ros/mission_bags/<run_id>.params.yaml
    <workspace>/mission_videos/<run_id>.mp4

AFTER: one folder per run, so a run is copyable whole or not at all.

    ~/f1tenth_archive/complete/<run_id>/bag/
                              /<run_id>.manifest.json
                              /<run_id>.params.yaml
                              /<run_id>.extract.parquet
                              /<run_id>.mp4

NOTHING IS DELETED HERE. Bags are MOVED (they are large and there is no point
holding two copies), but videos are COPIED and the originals left in place
until the migration has been verified -- per the work order's "leave
mission_videos/ until verified, then remove separately". No bag is removed by
any mechanism, here or elsewhere, until a `runs archive` verifies far-end
copies against the manifest checksums.

UNMATCHED THINGS ARE REPORTED, NEVER GUESSED AT. A video whose run_id matches
no manifest, a manifest with no bag, a bag with no manifest: each is listed at
the end rather than being paired up by mtime or by name similarity. Guessing
here would silently attach one run's video to another run's record, which is
exactly the kind of error nobody would catch by looking.

Extraction needs ROS, so this module imports mission_extract lazily: `runs`
itself stays stdlib-only and usable on a machine with no ROS (see runs_cli).
"""

import hashlib
import json
import os
import shutil
from pathlib import Path


def _write_manifest(path, manifest):
    """
    Write a manifest atomically: temp file, fsync, rename.

    Same discipline runs_db uses for note/rename. A manifest is the one
    artifact here that cannot be regenerated from anything else, and this
    function runs immediately after the bag it describes has been MOVED -- a
    torn write at that instant would leave run data on disk with no record of
    what it was.
    """
    tmp = Path(f'{path}.tmp')
    with open(tmp, 'w') as handle:
        json.dump(manifest, handle, indent=2, sort_keys=True)
        handle.write('\n')
        handle.flush()
        os.fsync(handle.fileno())
    os.replace(tmp, path)


def _sha256_and_size(path):
    """Checksum and byte count for one file, streamed rather than slurped."""
    digest = hashlib.sha256()
    size = 0
    with open(path, 'rb') as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b''):
            digest.update(chunk)
            size += len(chunk)
    return digest.hexdigest(), size


def bag_checksum(bag_dir):
    """
    ({relative path: sha256}, total bytes) over every file in a bag directory.

    Per-file rather than one digest over a concatenation: a bag is a directory
    whose file set can legitimately grow (multi-file bags split by size), and a
    per-file map lets a later verification say WHICH file differs rather than
    just that something did.
    """
    checksums = {}
    total = 0
    for path in sorted(Path(bag_dir).rglob('*')):
        if path.is_file():
            digest, size = _sha256_and_size(path)
            checksums[str(path.relative_to(bag_dir))] = digest
            total += size
    return checksums, total


def consolidate_run(manifest_path, source_dir, archive_dir, video_dir=None,
                    extract=True):
    """
    Move one run's artifacts into archive_dir/complete/<run_id>/.

    Returns a report dict. Idempotent: a run already consolidated is detected
    by its destination existing and is re-reported, not re-moved, so a run
    interrupted half way can be re-run safely.
    """
    from f1tenth_logger import runs_db          # stdlib only, safe here

    manifest_path = Path(manifest_path)
    manifest = runs_db.read_manifest(manifest_path)
    run_id = manifest.get('run_id') or manifest_path.name.split('.')[0]
    dest = Path(archive_dir) / 'complete' / run_id
    report = {'run_id': run_id, 'dest': str(dest), 'actions': [], 'problems': []}

    dest.mkdir(parents=True, exist_ok=True)

    # --- bag -------------------------------------------------------------
    source_bag = Path(manifest.get('bag_path') or (Path(source_dir) / run_id))
    dest_bag = dest / 'bag'
    if dest_bag.is_dir():
        report['actions'].append('bag already in place')
    elif source_bag.is_dir():
        shutil.move(str(source_bag), str(dest_bag))
        report['actions'].append('bag moved')
    else:
        report['problems'].append(f'no bag directory at {source_bag}')

    # --- manifest + params ------------------------------------------------
    dest_manifest = dest / f'{run_id}.manifest.json'
    params_source = Path(manifest.get('params_snapshot_path')
                         or (Path(source_dir) / f'{run_id}.params.yaml'))
    dest_params = dest / f'{run_id}.params.yaml'
    if params_source.is_file() and not dest_params.exists():
        shutil.move(str(params_source), str(dest_params))
        report['actions'].append('params snapshot moved')
    elif not dest_params.exists():
        report['problems'].append(f'no params snapshot at {params_source}')

    # --- checksum + size, recorded before the paths are rewritten ---------
    if dest_bag.is_dir():
        checksums, total = bag_checksum(dest_bag)
        manifest['bag_bytes'] = total
        manifest['bag_sha256'] = checksums
        report['bag_bytes'] = total

    # Paths inside the manifest must point at where things now ARE. The old
    # absolute paths would resolve to nothing after the move.
    manifest['bag_path'] = str(dest_bag)
    manifest['params_snapshot_path'] = str(dest_params)
    manifest['manifest_path'] = str(dest_manifest)
    manifest['run_dir'] = str(dest)
    _write_manifest(dest_manifest, manifest)
    if manifest_path.resolve() != dest_manifest.resolve() and manifest_path.exists():
        manifest_path.unlink()
        report['actions'].append('manifest moved')

    # --- video ------------------------------------------------------------
    if video_dir:
        candidate = Path(video_dir) / f'{run_id}.mp4'
        dest_video = dest / f'{run_id}.mp4'
        if dest_video.exists():
            report['actions'].append('video already in place')
        elif candidate.is_file():
            # COPIED, not moved: the originals stay until the migration is
            # verified, per the work order.
            shutil.copy2(candidate, dest_video)
            report['actions'].append('video copied')
            report['video'] = str(candidate)
        else:
            report['problems'].append('no rendered video for this run')

    # --- extract ----------------------------------------------------------
    dest_extract = dest / f'{run_id}.extract.parquet'
    if not extract:
        pass
    elif dest_extract.exists():
        report['actions'].append('extract already in place')
    elif not dest_bag.is_dir():
        report['problems'].append('cannot extract: no bag')
    else:
        try:
            from f1tenth_logger.mission_extract import extract_bag
        except ImportError as exc:
            report['problems'].append(
                f'cannot extract without ROS sourced ({exc})')
        else:
            try:
                extract_bag(dest_bag, dest_extract)
                report['actions'].append('extract written')
                report['extract_bytes'] = dest_extract.stat().st_size
            except Exception as exc:            # noqa: BLE001 - one bad bag
                # A bag that cannot be extracted is still archived; losing the
                # whole migration over one unreadable run would be worse.
                report['problems'].append(f'extraction failed: {exc}')

    return report


def find_orphan_videos(video_dir, known_run_ids):
    """MP4s in video_dir whose stem matches no known run_id. Reported, not guessed."""
    if not video_dir or not Path(video_dir).is_dir():
        return []
    return sorted(p.name for p in Path(video_dir).glob('*.mp4')
                  if p.stem not in known_run_ids)


def consolidate_dir(source_dir, archive_dir, video_dir=None, extract=True):
    """Consolidate every *.manifest.json in source_dir. Returns (reports, orphans)."""
    source_dir = Path(source_dir)
    reports = []
    for entry in sorted(os.listdir(source_dir)):
        if entry.endswith('.manifest.json'):
            reports.append(consolidate_run(source_dir / entry, source_dir,
                                           archive_dir, video_dir, extract))
    orphans = find_orphan_videos(video_dir, {r['run_id'] for r in reports})
    return reports, orphans
