// Missione (design/Missione.dc.html, MissioneInCorso.dc.html): command by text or voice,
// the plan card, slide-to-start, the mission under way, ABORT on a 1 s hold.
import { signal } from '@preact/signals';
import { useEffect, useRef, useState } from 'preact/hooks';
import { api, ApiError, post, waitJob } from '../api';
import { drive, latched, mission, missionActive, online, status, tel, toast, token } from '../store';
import { fmt, HoldButton, Icon, SlideToStart } from '../components/ui';
import { capture } from '../components/pointer';

interface Step { i: number; id: string; action: string; speed_mps?: number; until?: string;
  timeout_s?: number; on_timeout?: string; terminal?: boolean }
interface Plan {
  ok: boolean; plan_id?: string; text: string; valid: boolean; errors: string[];
  summary?: { n_moves: number; max_speed_mps: number | null; distance_m: number | null;
              max_duration_s: number | null; targets: string[] };
  steps: Step[]; json?: unknown; unsupported: string[]; notes: string[]; created?: number;
  plan_seconds?: number; error?: string;
}

const text = signal('');
const plan = signal<Plan | null>(null);
const planning = signal(false);
const shownPlanId = signal<string | null>(null);
const PLAN_MAX_AGE_S = 300;

async function makePlan(): Promise<void> {
  planning.value = true;
  plan.value = null;
  try {
    const job = await post('/api/mission/plan', { text: text.value });
    const j = await waitJob<Plan>(job, 120_000);
    const r = j.result as Plan;
    plan.value = r ?? { ok: false, text: text.value, valid: false, errors: ['nessuna risposta'],
      steps: [], unsupported: [], notes: [] };
    if (!r?.ok && r?.error) toast('error', `Piano: ${r.error}`);
  } catch (e: any) {
    toast('error', `Piano: ${e.message}`);
  } finally {
    planning.value = false;
  }
}

async function start(): Promise<void> {
  const p = plan.value;
  if (!p?.plan_id) return;
  try {
    const r = await post<{ countdown_s: number }>('/api/mission/start', { plan_id: p.plan_id });
    shownPlanId.value = p.plan_id;
    toast('warn', r.countdown_s > 0 ? `Avvio tra ${r.countdown_s.toFixed(0)} s` : 'Missione avviata');
  } catch (e: any) {
    toast('error', `Avvio rifiutato: ${e.message}`);
  }
}

async function abort(): Promise<void> {
  try {
    const r = await post<{ message: string }>('/api/mission/abort');
    toast('warn', `ABORT: ${r.message}`);
  } catch (e: any) {
    toast('error', `ABORT fallito: ${e.message}`);
  }
}

function Mic({ disabled }: { disabled: boolean }) {
  const [state, setState] = useState<'idle' | 'rec' | 'busy'>('idle');
  const rec = useRef<MediaRecorder | null>(null);
  const chunks = useRef<Blob[]>([]);
  const t0 = useRef(0);
  const down = async (e: PointerEvent) => {
    if (disabled || state !== 'idle') return;
    capture(e);
    if (typeof MediaRecorder === 'undefined' || !navigator.mediaDevices?.getUserMedia) {
      toast('warn', 'Registrazione non disponibile qui: usa la dettatura della tastiera');
      return;
    }
    try {
      const stream = await navigator.mediaDevices.getUserMedia({ audio: true });
      const r = new MediaRecorder(stream);
      chunks.current = [];
      r.ondataavailable = (ev) => { if (ev.data.size) chunks.current.push(ev.data); };
      r.onstop = () => {
        stream.getTracks().forEach((t) => t.stop());
        void upload(new Blob(chunks.current, { type: r.mimeType || 'audio/mp4' }));
      };
      r.start();
      rec.current = r;
      t0.current = performance.now();
      setState('rec');
    } catch {
      toast('error', 'Microfono negato: abilitalo nelle impostazioni o usa la dettatura');
    }
  };
  const up = () => {
    if (state !== 'rec' || !rec.current) return;
    if (performance.now() - t0.current < 400) toast('info', 'Tieni premuto mentre parli');
    rec.current.stop();
    rec.current = null;
    setState('busy');
  };
  const upload = async (blob: Blob) => {
    try {
      const form = new FormData();
      form.append('audio', blob, 'command' + (blob.type.includes('webm') ? '.webm' : '.m4a'));
      const res = await fetch('/api/mission/transcribe', {
        method: 'POST', body: form, headers: { Authorization: `Bearer ${token.value}` } });
      const body = await res.json().catch(() => ({}));
      if (!res.ok) throw new ApiError(res.status, body.detail ?? res.statusText);
      text.value = body.text ?? '';
      if (!text.value) toast('warn', 'Non ho capito: riprova o scrivi');
    } catch (e: any) {
      toast('error', `Trascrizione: ${e.message}`);
    } finally {
      setState('idle');
    }
  };
  return (
    <button type="button" aria-label="Tieni premuto per parlare" disabled={disabled || state === 'busy'}
      onPointerDown={down} onPointerUp={up} onPointerCancel={up} onContextMenu={(e) => e.preventDefault()}
      data-testid="mic"
      style={{ alignSelf: 'flex-end', width: '72px', height: '72px', borderRadius: '20px',
        border: `1px solid ${state === 'rec' ? 'var(--red)' : 'var(--border)'}`,
        background: state === 'rec' ? 'rgba(255,90,80,0.16)' : 'var(--surface-2)', display: 'flex',
        flexDirection: 'column', alignItems: 'center', justifyContent: 'center', gap: '2px',
        touchAction: 'none', opacity: disabled ? 0.4 : 1 }}>
      {Icon.mic()}
      <span class="mono" style={{ fontSize: '9px', color: 'var(--muted)' }}>
        {state === 'rec' ? 'REC' : state === 'busy' ? '…' : 'HOLD'}</span>
    </button>
  );
}

function fmtTime(s?: number): string {
  if (s === undefined || s === null || !Number.isFinite(s)) return '00:00.0';
  const m = Math.floor(s / 60);
  return `${String(m).padStart(2, '0')}:${(s - m * 60).toFixed(1).padStart(4, '0')}`;
}

function PlanCard({ p }: { p: Plan }) {
  const [open, setOpen] = useState(false);
  const [, tick] = useState(0);
  useEffect(() => {
    const t = setInterval(() => tick((x) => x + 1), 1000);
    return () => clearInterval(t);
  }, []);
  const age = p.created ? Date.now() / 1000 - p.created : 0;
  const left = PLAN_MAX_AGE_S - age;
  const s = p.summary;
  const length = s?.distance_m !== null && s?.distance_m !== undefined
    ? `${s.distance_m.toFixed(1)}` : s?.targets?.length ? s.targets.join(', ') : '—';
  return (
    <section aria-label="Piano generato" class="card pad" data-testid="plan-card"
      style={{ display: 'flex', flexDirection: 'column', gap: '12px' }}>
      <div style={{ display: 'flex', alignItems: 'center', justifyContent: 'space-between' }}>
        <span class="label">PIANO</span>
        {p.valid
          ? <span class="mono" style={{ display: 'flex', gap: '6px', alignItems: 'center', fontSize: '12px', fontWeight: 700, color: 'var(--green)' }}>{Icon.check()}VALIDATO</span>
          : <span class="mono" style={{ fontSize: '12px', fontWeight: 700, color: 'var(--red)' }}>ERRORI</span>}
      </div>
      {p.valid && s ? (
        <div class="grid3">
          <div style={{ display: 'flex', flexDirection: 'column' }}><span class="mono" style={{ fontSize: '28px', fontWeight: 700 }}>{s.n_moves}</span><span class="muted" style={{ fontSize: '12px' }}>move</span></div>
          <div style={{ display: 'flex', flexDirection: 'column' }}><span class="mono" style={{ fontSize: '28px', fontWeight: 700 }}>{fmt(s.max_speed_mps, 1)}</span><span class="muted" style={{ fontSize: '12px' }}>v max m/s</span></div>
          <div style={{ display: 'flex', flexDirection: 'column', minWidth: 0 }}><span class="mono" style={{ fontSize: s.distance_m != null ? '28px' : '18px', fontWeight: 700, overflow: 'hidden', textOverflow: 'ellipsis', whiteSpace: 'nowrap', lineHeight: s.distance_m != null ? undefined : '34px' }}>{length}</span><span class="muted" style={{ fontSize: '12px' }}>{s.distance_m != null ? 'lunghezza m' : 'fino a'}</span></div>
        </div>
      ) : (
        <ul style={{ margin: 0, paddingLeft: '18px', color: 'var(--red)', fontSize: '14px' }}>
          {(p.errors.length ? p.errors : [p.error ?? 'piano non valido']).map((e) => <li key={e}>{e}</li>)}
        </ul>)}
      {p.steps.length > 0 && (
        <ol style={{ margin: 0, padding: '0 0 0 20px', display: 'flex', flexDirection: 'column', gap: '4px', fontSize: '14px', color: 'var(--text-2)' }}>
          {p.steps.map((st) => (
            <li key={st.i}>{st.action}{st.speed_mps ? ` a ${st.speed_mps} m/s` : ''}{st.until ? <>, fino a <span class="mono">{st.until}</span></> : ''}</li>))}
        </ol>)}
      {[...p.unsupported.map((u) => `non supportato: ${u}`), ...p.notes].map((n) => (
        <div key={n} style={{ display: 'flex', gap: '8px', fontSize: '13px', color: 'var(--amber)' }}>{Icon.warn()}<span>{n}</span></div>))}
      {p.json !== undefined && (
        <>
          <button type="button" onClick={() => setOpen(!open)} aria-expanded={open}
            style={{ display: 'flex', alignItems: 'center', justifyContent: 'space-between', minHeight: '44px',
              padding: '0 12px', borderRadius: '12px', border: '1px solid var(--border)', background: 'var(--bar)',
              fontFamily: 'var(--mono)', fontSize: '13px' }}>
            <span>{open ? 'Nascondi JSON' : 'Mostra JSON'}</span>{Icon.chevron(open)}
          </button>
          {open && (
            <pre class="mono" style={{ margin: 0, padding: '12px', borderRadius: '12px', background: 'var(--bar)',
              fontSize: '12px', lineHeight: 1.5, color: 'var(--text-2)', whiteSpace: 'pre', overflowX: 'auto',
              WebkitUserSelect: 'text', userSelect: 'text' }}>{JSON.stringify(p.json, null, 2)}</pre>)}
        </>)}
      {p.valid && (
        <span class="mono muted" style={{ fontSize: '11px' }}>
          {left > 0 ? `avviabile ancora ${Math.floor(left / 60)}:${String(Math.floor(left % 60)).padStart(2, '0')}` : 'scaduto: ripianifica'}</span>)}
    </section>
  );
}

function Running() {
  const m = mission.value;
  const t = tel.value;
  const p = plan.value && plan.value.plan_id === (m.id ?? shownPlanId.value) ? plan.value : null;
  const n = m.n_moves ?? p?.steps.length ?? 0;
  const done = m.moves_done ?? 0;
  const active = ['starting', 'running', 'holding'].includes(m.state);
  const head = m.state === 'starting'
    ? `AVVIO TRA ${Math.ceil(m.countdown_left_s ?? 0)} s`
    : m.state === 'holding' ? 'IN ATTESA' : m.state === 'finished' ? 'COMPLETATA'
      : m.state === 'aborted' ? 'INTERROTTA' : 'IN CORSO';
  const colour = m.state === 'aborted' ? 'var(--amber)' : 'var(--green)';
  return (
    <>
      <section aria-label="Missione in corso" data-testid="mission-running" style={{
        border: `1.5px solid ${m.state === 'aborted' ? 'rgba(245,165,36,0.55)' : 'rgba(52,210,123,0.55)'}`,
        background: m.state === 'aborted' ? 'rgba(245,165,36,0.06)' : 'rgba(52,210,123,0.06)',
        borderRadius: '20px', padding: '18px', display: 'flex', flexDirection: 'column', gap: '16px' }}>
        <div style={{ display: 'flex', alignItems: 'center', justifyContent: 'space-between' }}>
          <span class="mono" style={{ display: 'flex', alignItems: 'center', gap: '8px', fontSize: '13px', fontWeight: 800, color: colour, letterSpacing: '0.1em' }}>
            <span style={{ width: '10px', height: '10px', borderRadius: '50%', background: colour,
              boxShadow: active ? '0 0 0 4px rgba(52,210,123,0.25)' : 'none' }} />{head}</span>
          <span class="mono muted" style={{ fontSize: '12px' }}>{m.mission_id ?? m.id ?? ''}</span>
        </div>
        <div style={{ display: 'flex', alignItems: 'flex-end', justifyContent: 'space-between' }}>
          <div style={{ display: 'flex', flexDirection: 'column' }}><span class="label">TEMPO</span>
            <span class="mono" style={{ fontSize: '52px', lineHeight: 1, fontWeight: 800 }}>{fmtTime(m.elapsed_s)}</span></div>
          <div style={{ display: 'flex', flexDirection: 'column', alignItems: 'flex-end' }}><span class="label">PASSO</span>
            <span class="mono" style={{ fontSize: '34px', lineHeight: 1, fontWeight: 700 }}>{m.move_index ?? '—'}<span class="muted">/{n || '—'}</span></span></div>
        </div>
        <div style={{ display: 'flex', flexDirection: 'column', gap: '6px' }}>
          <div style={{ height: '8px', borderRadius: '999px', background: 'var(--border)', overflow: 'hidden' }}>
            <div style={{ width: `${n ? Math.round((done / n) * 100) : 0}%`, height: '100%', background: colour, borderRadius: '999px' }} /></div>
          <div class="mono muted" style={{ display: 'flex', justifyContent: 'space-between', fontSize: '11px' }}>
            <span>inizio</span><span>{n ? Math.round((done / n) * 100) : 0}%</span><span>fine</span></div>
        </div>
        <div class="grid2">
          <div class="stat"><span class="k">VEL REALE m/s</span><span class="v">{fmt(t?.v_meas, 2)}</span></div>
          <div class="stat"><span class="k">v CMD m/s</span><span class="v">{fmt(t?.v_cmd, 2)}</span></div>
        </div>
      </section>
      <section aria-label="Passi" class="card" style={{ padding: '6px 0' }}>
        {Array.from({ length: n }, (_, k) => {
          const idx = k + 1;
          const st = p?.steps[k];
          const state = idx <= done ? 'done' : idx === m.move_index && active ? 'active' : 'pending';
          return (
            <div key={idx} style={{ display: 'flex', alignItems: 'center', gap: '12px', padding: '12px 16px',
              background: state === 'active' ? 'var(--surface-2)' : 'transparent' }}>
              <span style={{ width: '20px', display: 'flex', justifyContent: 'center' }}>
                {state === 'done' ? Icon.check(20, '#34D27B')
                  : <span style={{ width: '10px', height: '10px', borderRadius: '50%',
                    background: state === 'active' ? 'var(--text)' : 'transparent',
                    border: state === 'pending' ? '2px solid var(--disabled)' : 'none' }} />}</span>
              <span style={{ flex: 1, fontSize: '15px', fontWeight: state === 'active' ? 700 : 400,
                color: state === 'active' ? 'var(--text)' : 'var(--muted)' }}>{st ? st.action : `Passo ${idx}`}</span>
              <span class="mono" style={{ fontSize: '12px', color: state === 'active' ? 'var(--text)' : 'var(--muted)' }}>
                {state === 'active' ? 'attivo' : state === 'done' ? 'fatto' : '—'}</span>
            </div>);
        })}
      </section>
      {active ? (
        <HoldButton ms={1000} label="TIENI PREMUTO · ABORT" onDone={abort} disabled={!online.value}
          testid="abort" icon={Icon.square()} />
      ) : (
        <button type="button" class="btn" onClick={() => { shownPlanId.value = null; plan.value = null; }}
          data-testid="new-command">NUOVO COMANDO</button>)}
    </>
  );
}

export function Missione() {
  const m = mission.value;
  const showRunning = missionActive.value ||
    (shownPlanId.value !== null && ['finished', 'aborted'].includes(m.state));
  const p = plan.value;
  const ready = !!status.value?.ready;
  const expired = !!p?.created && Date.now() / 1000 - p.created > PLAN_MAX_AGE_S;
  const canStart = online.value && ready && !!p?.valid && !latched.value && !expired &&
    !missionActive.value && drive.value.state !== 'granted';
  return (
    <>
      <h1>Missione</h1>
      {showRunning ? <Running /> : (
        <>
          <div style={{ display: 'flex', gap: '10px', alignItems: 'stretch' }}>
            <label style={{ flex: 1, display: 'flex', flexDirection: 'column', gap: '6px' }}>
              <span class="label">COMANDO</span>
              <textarea rows={3} value={text.value} maxLength={500} data-testid="command"
                onInput={(e) => { text.value = (e.target as HTMLTextAreaElement).value; }}
                placeholder="es. vai dritto per due metri e fermati"
                style={{ resize: 'none', background: 'var(--surface)', border: '1px solid var(--border)',
                  borderRadius: '16px', color: 'var(--text)', fontFamily: 'var(--sans)', fontSize: '16px',
                  lineHeight: 1.4, padding: '12px 14px', outline: 'none' }} />
            </label>
            <Mic disabled={!online.value || missionActive.value || drive.value.state === 'granted'} />
          </div>
          <button type="button" class="btn primary" data-testid="plan"
            disabled={!online.value || planning.value || !text.value.trim()} onClick={makePlan}>
            {planning.value ? 'PIANIFICAZIONE…' : 'PIANIFICA'}</button>
          {p && <PlanCard p={p} />}
          {p?.valid && (
            <>
              <SlideToStart disabled={!canStart} onDone={start} />
              {!canStart && (
                <span class="mono muted" style={{ fontSize: '12px', textAlign: 'center' }} data-testid="start-why">
                  {!online.value ? 'offline' : latched.value ? 'E-STOP attivo' : !ready ? 'NOT READY'
                    : expired ? 'piano scaduto' : drive.value.state === 'granted' ? 'chiudi la Guida' : ''}</span>)}
            </>)}
        </>)}
    </>
  );
}

// Used by the e2e tests to read what the app holds.
export const _debug = { plan, text, api };
