// Run (design/Run.dc.html): the mission logger's archive, newest first, with the report
// page and the video when one exists (else the link is off, with the reason).
import { useEffect, useState } from 'preact/hooks';
import { api } from '../api';
import { online } from '../store';

interface RunRow {
  id: string; folder: string; finalized: boolean; mission_id: string; outcome: string | null;
  badge: string; started: string | null; duration_s: number | null;
  report: { ok: boolean; why?: string }; video: { ok: boolean; why?: string };
}

const BADGE: Record<string, [string, string]> = {
  OK: ['#34D27B', 'rgba(52,210,123,0.14)'],
  ABORT: ['#F5A524', 'rgba(245,165,36,0.14)'],
  'E-STOP': ['#FF5A50', 'rgba(255,90,80,0.14)'],
};

function when(iso: string | null): string {
  if (!iso) return '—';
  const d = new Date(iso);
  const months = ['gen', 'feb', 'mar', 'apr', 'mag', 'giu', 'lug', 'ago', 'set', 'ott', 'nov', 'dic'];
  return `${d.getDate()} ${months[d.getMonth()]} ${String(d.getHours()).padStart(2, '0')}:${String(d.getMinutes()).padStart(2, '0')}`;
}

function dur(s: number | null): string {
  if (s === null) return 'in corso / non finalizzata';
  return `${String(Math.floor(s / 60)).padStart(2, '0')}:${String(Math.round(s % 60)).padStart(2, '0')}`;
}

const linkStyle = {
  display: 'flex', alignItems: 'center', gap: '6px', minHeight: '44px', padding: '0 12px',
  borderRadius: '10px', background: 'var(--surface-2)', color: 'var(--text)', textDecoration: 'none',
  fontSize: '13px', fontWeight: 600,
};

export function Run() {
  const [runs, setRuns] = useState<RunRow[] | null>(null);
  const [err, setErr] = useState<string | null>(null);
  const load = () => {
    api<{ runs: RunRow[] }>('/api/runs?limit=20').then((r) => { setRuns(r.runs); setErr(null); })
      .catch((e) => setErr(e.message));
  };
  useEffect(load, [online.value]);
  return (
    <>
      <div class="titlerow"><h1>Run</h1>
        <button type="button" class="mono muted" onClick={load} style={{ background: 'none', border: 0, fontSize: '12px' }}>
          {runs ? `ultime ${runs.length} · aggiorna` : 'aggiorna'}</button></div>
      {err && <div class="card pad" style={{ color: 'var(--red)', fontSize: '14px' }}>{err}</div>}
      <ul style={{ listStyle: 'none', margin: 0, padding: 0, display: 'flex', flexDirection: 'column', gap: '8px' }} data-testid="runs">
        {(runs ?? []).map((r) => {
          const [fg, bg] = BADGE[r.badge] ?? ['#9099A6', 'rgba(144,153,166,0.14)'];
          return (
            <li key={r.id} class="card" style={{ borderRadius: '16px', padding: '12px 14px', display: 'flex', flexDirection: 'column', gap: '10px' }}>
              <div style={{ display: 'flex', alignItems: 'center', gap: '10px' }}>
                <span class="mono" style={{ fontSize: '15px', fontWeight: 700 }}>{r.id.slice(11, 16).replace('-', ':')}</span>
                <span style={{ flex: 1, fontSize: '13px', color: 'var(--muted)', whiteSpace: 'nowrap', overflow: 'hidden', textOverflow: 'ellipsis' }}>{r.mission_id}</span>
                <span class="mono" style={{ fontSize: '11px', fontWeight: 800, letterSpacing: '0.08em', padding: '4px 8px', borderRadius: '6px', color: fg, background: bg }}>{r.badge}</span>
              </div>
              <div style={{ display: 'flex', alignItems: 'center', gap: '8px' }}>
                <span class="mono muted" style={{ flex: 1, fontSize: '12px' }}>{when(r.started)} · {dur(r.duration_s)}</span>
                {r.report.ok
                  ? <a href={`/api/runs/${encodeURIComponent(r.id)}/report`} target="_blank" rel="noopener" style={linkStyle}>Report</a>
                  : <span style={{ ...linkStyle, opacity: 0.4 }} title={r.report.why}>Report</span>}
                {r.video.ok
                  ? <a href={`/api/runs/${encodeURIComponent(r.id)}/video`} target="_blank" rel="noopener" style={linkStyle}>Video</a>
                  : <span style={{ ...linkStyle, opacity: 0.4 }} title={r.video.why} aria-disabled="true">Video</span>}
              </div>
              {!r.video.ok && <span class="mono muted" style={{ fontSize: '11px' }}>{r.video.why}</span>}
            </li>);
        })}
      </ul>
      {runs && runs.length === 0 && <p class="muted">Nessuna run nell'archivio.</p>}
    </>
  );
}
