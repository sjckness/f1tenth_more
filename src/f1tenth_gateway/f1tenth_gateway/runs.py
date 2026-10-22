"""
The Run tab: the mission logger's archive, read-only.

mission_logger_node writes one folder per mission under the archive root
(stack_params mission_logger_runs_dir, ~/f1tenth_archive):

    active/<run_id>/       still recording, or never finalized
    complete/<run_id>/     finalized: manifest, params, bag/, extract, snapshots/
    incomplete/<run_id>/   a bag without metadata.yaml

run_id = <UTC %Y-%m-%dT%H-%M-%S>_mission-<mission name>. Each folder has
<run_id>.manifest.json with mission_id, start/end times and outcome
(COMPLETE, ABORTED, EMERGENCY_STOP, INTERRUPTED, NOT_STARTED, LOADED, IDLE).

No report exists per run, so the report is a page built here from the
manifest, the mission file, the behaviour tree's scoring JSON
(<ws>/src/f1tenth_behavior/mission_reports/<mission_id>_<stamp>.json) and the
snapshots. Videos are made by hand (mission_render, test_campaign_video):
one is offered only when an mp4 exists.
"""

from datetime import datetime, timezone
import glob
import html
import json
import os
import re
from typing import Dict, List, Optional

RUN_ID = re.compile(r'^[0-9]{4}-[0-9]{2}-[0-9]{2}T[0-9]{2}-[0-9]{2}-[0-9]{2}_[A-Za-z0-9_.-]+$')
FOLDERS = ('complete', 'active', 'incomplete')
OUTCOME_BADGE = {'COMPLETE': 'OK', 'ABORTED': 'ABORT', 'EMERGENCY_STOP': 'E-STOP'}
NO_VIDEO = 'nessun video per questa run (si genera a mano: mission_render)'


def _time(s: Optional[str]) -> Optional[datetime]:
    if not s:
        return None
    try:
        t = datetime.fromisoformat(s)
    except ValueError:
        return None
    return t if t.tzinfo else t.replace(tzinfo=timezone.utc)


def _stamp_from_id(run_id: str) -> Optional[datetime]:
    try:
        return datetime.strptime(run_id[:19], '%Y-%m-%dT%H-%M-%S').replace(tzinfo=timezone.utc)
    except ValueError:
        return None


class Archive:
    def __init__(self, root: str, workspace: str):
        self.root = os.path.expanduser(root)
        self.workspace = os.path.expanduser(workspace)

    # ---- finding runs ------------------------------------------------------------

    def run_dir(self, run_id: str) -> Optional[str]:
        """Return the folder of one run, or None (also for any id that is not a run id)."""
        if not isinstance(run_id, str) or not RUN_ID.match(run_id):
            return None
        for folder in FOLDERS:
            d = os.path.join(self.root, folder, run_id)
            if os.path.isdir(d):
                real = os.path.realpath(d)
                if real.startswith(os.path.realpath(self.root) + os.sep):
                    return real
        return None

    def manifest(self, run_id: str) -> Optional[Dict]:
        d = self.run_dir(run_id)
        if d is None:
            return None
        try:
            with open(os.path.join(d, f'{run_id}.manifest.json')) as f:
                data = json.load(f)
        except (OSError, ValueError):
            return {}
        return data if isinstance(data, dict) else {}

    def list_runs(self, limit: int = 20) -> List[Dict]:
        entries = []
        for folder in FOLDERS:
            base = os.path.join(self.root, folder)
            try:
                names = os.listdir(base)
            except OSError:
                continue
            for name in names:
                if RUN_ID.match(name) and os.path.isdir(os.path.join(base, name)):
                    entries.append((name, folder))
        entries.sort(key=lambda e: e[0], reverse=True)
        return [self._entry(name, folder) for name, folder in entries[:max(1, int(limit))]]

    def _entry(self, run_id: str, folder: str) -> Dict:
        m = self.manifest(run_id)
        start = _time((m or {}).get('start_time')) or _stamp_from_id(run_id)
        end = _time((m or {}).get('end_time'))
        outcome = (m or {}).get('outcome')
        video = self.video_path(run_id)
        return {
            'id': run_id,
            'folder': folder,
            'finalized': folder == 'complete',
            'mission_id': (m or {}).get('mission_id') or run_id.split('_mission-', 1)[-1],
            'outcome': outcome,
            'badge': OUTCOME_BADGE.get(outcome or '', 'INCOMPLETA' if folder != 'complete'
                                       else (outcome or '?')),
            'started': start.isoformat() if start else None,
            'duration_s': round((end - start).total_seconds(), 1) if start and end else None,
            'report': ({'ok': True} if m is not None and m != {} else
                       {'ok': False, 'why': 'manifest assente o illeggibile'}),
            'video': {'ok': True} if video else {'ok': False, 'why': NO_VIDEO},
        }

    # ---- files ------------------------------------------------------------------------

    def video_path(self, run_id: str) -> Optional[str]:
        d = self.run_dir(run_id)
        if d is None:
            return None
        found = sorted(glob.glob(os.path.join(d, '*.mp4')))
        found += sorted(glob.glob(os.path.join(self.workspace, 'mission_videos',
                                               glob.escape(run_id) + '*.mp4')))
        return found[0] if found else None

    def snapshots(self, run_id: str) -> List[str]:
        d = self.run_dir(run_id)
        return sorted(glob.glob(os.path.join(d, 'snapshots', '*.jpg'))) if d else []

    def snapshot_path(self, run_id: str, n: int) -> Optional[str]:
        snaps = self.snapshots(run_id)
        return snaps[n] if 0 <= n < len(snaps) else None

    def scoring(self, mission_id: str, start: Optional[datetime]) -> Optional[Dict]:
        """Return the BT's report for this run: same mission, closest stamp after start."""
        if not mission_id or not re.match(r'^[A-Za-z0-9_.-]+$', mission_id):
            return None
        pattern = os.path.join(self.workspace, 'src', 'f1tenth_behavior', 'mission_reports',
                               glob.escape(mission_id) + '_*.json')
        best = None
        for path in glob.glob(pattern):
            stamp = os.path.basename(path)[len(mission_id) + 1:-5]
            try:
                t = datetime.strptime(stamp, '%Y%m%dT%H%M%S').astimezone(timezone.utc)
            except ValueError:
                continue
            if start is not None and not (0 <= (t - start).total_seconds() <= 3600):
                continue
            if best is None or t < best[0]:
                best = (t, path)
        if best is None:
            return None
        try:
            with open(best[1]) as f:
                return json.load(f)
        except (OSError, ValueError):
            return None

    # ---- the report page ----------------------------------------------------------------

    def report_html(self, run_id: str) -> Optional[str]:
        m = self.manifest(run_id)
        if m is None:
            return None
        e = html.escape
        start = _time(m.get('start_time')) or _stamp_from_id(run_id)
        keys = ('run_id', 'mission_id', 'outcome', 'start_time', 'end_time', 'preroll_s',
                'bag_bytes', 'storage_id')
        rows = [(k, m.get(k)) for k in keys]
        git = m.get('git') or {}
        rows.append(('git', f"{git.get('branch')} {str(git.get('commit'))[:10]}"
                            f"{' (dirty)' if git.get('dirty') else ''}"))
        mission = None
        path = m.get('mission_json_path')
        if isinstance(path, str) and not os.path.isabs(path):
            path = os.path.join(self.run_dir(run_id), path)
        if isinstance(path, str):
            try:
                with open(path) as f:
                    mission = json.load(f)
            except (OSError, ValueError):
                mission = None
        score = self.scoring(m.get('mission_id'), start)
        parts = [f'<h1>{e(run_id)}</h1>', '<table>']
        parts += [f'<tr><th>{e(str(k))}</th><td>{e(str(v))}</td></tr>' for k, v in rows]
        parts.append('</table>')
        if mission and isinstance(mission.get('moves'), list):
            parts.append('<h2>Move</h2><ol>')
            for mv in mission['moves']:
                if isinstance(mv, dict):
                    kind = next((k for k in ('drive', 'go_to_object', 'turn', 'goal_distance',
                                             'goal_pose') if k in mv), '?')
                    stop = (mv.get('stop_condition') or {}).get('type', '?')
                    mid = e(str(mv.get('id')))
                    parts.append(f'<li><b>{mid}</b> {e(kind)}, fino a {e(str(stop))}</li>')
            parts.append('</ol>')
        else:
            parts.append('<p>File della missione non leggibile.</p>')
        moves = (score or {}).get('moves') if isinstance(score, dict) else None
        if isinstance(moves, list) and moves:
            parts.append('<h2>Esito per move</h2><table><tr><th>move</th><th>esito</th>'
                         '<th>motivo</th><th>durata s</th></tr>')
            for mv in moves:
                if isinstance(mv, dict):
                    parts.append('<tr>' + ''.join(
                        f'<td>{e(str(mv.get(k, "")))}</td>'
                        for k in ('move_id', 'outcome', 'stop_reason', 'duration_s')) + '</tr>')
            parts.append('</table>')
        else:
            parts.append('<p>Nessun report di scoring del BT per questa run.</p>')
        snaps = self.snapshots(run_id)
        if snaps:
            parts.append('<h2>Snapshot</h2>')
            parts += [f'<img src="snapshots/{i}" alt="snapshot {i}" loading="lazy">'
                      for i in range(len(snaps))]
        style = ('body{background:#0A0C0F;color:#ECEEF1;font:15px Inter,system-ui,sans-serif;'
                 'margin:16px}th{color:#9099A6;text-align:left;padding-right:12px}'
                 'td,th{padding:4px 8px 4px 0;vertical-align:top}img{max-width:100%;'
                 'border-radius:12px;margin:6px 0}h1{font-size:18px;word-break:break-all}')
        return ('<!doctype html><html lang="it"><head><meta charset="utf-8">'
                '<meta name="viewport" content="width=device-width,initial-scale=1">'
                f'<title>Run {e(run_id)}</title><style>{style}</style></head><body>'
                + ''.join(parts) + '</body></html>')
