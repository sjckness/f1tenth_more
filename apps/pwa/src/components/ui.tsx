// Small shared pieces: icons, the top bar, EMERGENZA, the tab bar, banners, switches,
// the hold and slide gestures, toasts.
import { useEffect, useRef, useState } from 'preact/hooks';
import type { ComponentChildren } from 'preact';
import { post } from '../api';
import {
  battery, conn, estopReach, gatewayUp, lastMsgAt, latched, online, rttMs, status, tel, toast,
  toasts,
} from '../store';
import { send } from '../ws';
import { capture } from './pointer';

export const Icon = {
  octagon: (s = 26) => (
    <svg width={s} height={s} viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2.2"
      stroke-linejoin="round" stroke-linecap="round" aria-hidden="true">
      <path d="M8 3h8l5 5v8l-5 5H8l-5-5V8z" /><path d="M12 8v5M12 16.5v.01" /></svg>),
  error: () => (
    <svg width="22" height="22" viewBox="0 0 24 24" fill="none" stroke="#FF5A50" stroke-width="2.2"
      stroke-linecap="round" role="img" aria-label="Errore">
      <circle cx="12" cy="12" r="9" /><path d="M9 9l6 6M15 9l-6 6" /></svg>),
  warn: () => (
    <svg width="22" height="22" viewBox="0 0 24 24" fill="none" stroke="#F5A524" stroke-width="2.2"
      stroke-linecap="round" stroke-linejoin="round" role="img" aria-label="Avviso">
      <path d="M12 3l9.5 17h-19z" /><path d="M12 10v4M12 17v.01" /></svg>),
  check: (s = 14, c = 'currentColor') => (
    <svg width={s} height={s} viewBox="0 0 24 24" fill="none" stroke={c} stroke-width="3"
      stroke-linecap="round" stroke-linejoin="round" aria-hidden="true"><path d="M4 12l5 5L20 6" /></svg>),
  wifiOff: () => (
    <svg width="26" height="26" viewBox="0 0 24 24" fill="none" stroke="#FF5A50" stroke-width="2.2"
      stroke-linecap="round" stroke-linejoin="round" aria-hidden="true">
      <path d="M2 8.5a15 15 0 0 1 20 0M5 12a10.5 10.5 0 0 1 14 0M8.5 15.5a5.5 5.5 0 0 1 7 0M12 19v.01" />
      <path d="M3 3l18 18" /></svg>),
  back: () => (
    <svg width="22" height="22" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2.2"
      stroke-linecap="round" stroke-linejoin="round" aria-hidden="true"><path d="M15 5l-7 7 7 7" /></svg>),
  mic: () => (
    <svg width="26" height="26" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2"
      stroke-linecap="round" aria-hidden="true">
      <rect x="9" y="3" width="6" height="11" rx="3" /><path d="M5 11a7 7 0 0 0 14 0M12 18v3" /></svg>),
  arrow: () => (
    <svg width="26" height="26" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2.6"
      stroke-linecap="round" stroke-linejoin="round" aria-hidden="true"><path d="M5 12h14M13 6l6 6-6 6" /></svg>),
  square: () => (
    <svg width="20" height="20" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2.4"
      stroke-linecap="round" aria-hidden="true"><rect x="6" y="6" width="12" height="12" rx="1.5" /></svg>),
  chevron: (open: boolean) => (
    <svg width="18" height="18" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2"
      stroke-linecap="round" stroke-linejoin="round" aria-hidden="true"
      style={{ transform: open ? 'rotate(180deg)' : 'none' }}><path d="M6 9l6 6 6-6" /></svg>),
};

const TABS: [string, string, preact.JSX.Element][] = [
  ['cockpit', 'Cockpit', <svg width="24" height="24" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round" aria-hidden="true"><path d="M4 16a8 8 0 1 1 16 0" /><path d="M12 16l4-5" /></svg>],
  ['missione', 'Missione', <svg width="24" height="24" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round" aria-hidden="true"><path d="M5 21V4" /><path d="M5 4h12l-2.5 4 2.5 4H5" /></svg>],
  ['guida', 'Guida', <svg width="24" height="24" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round" aria-hidden="true"><circle cx="12" cy="12" r="9" /><circle cx="12" cy="12" r="2" /><path d="M3.5 10.5H10M14 10.5h6.5M12 14v7" /></svg>],
  ['telemetria', 'Telemetria', <svg width="24" height="24" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round" aria-hidden="true"><path d="M3 12h4l3-8 4 16 3-8h4" /></svg>],
  ['run', 'Run', <svg width="24" height="24" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round" aria-hidden="true"><rect x="3" y="5" width="18" height="14" rx="2" /><path d="M10 9l5 3-5 3z" /></svg>],
];

export function TabBar({ route }: { route: string }) {
  return (
    <nav class="tabs" aria-label="Sezioni">
      {TABS.map(([id, label, icon]) => (
        <a href={`#/${id}`} aria-current={route === id ? 'page' : undefined} key={id}>
          {icon}{label}</a>))}
    </nav>
  );
}

export function fmt(v: number | null | undefined, digits = 2, unit = ''): string {
  return v === null || v === undefined || !Number.isFinite(v) ? '—' : `${v.toFixed(digits)}${unit}`;
}

export function ConnPill({ readyMode = false }: { readyMode?: boolean }) {
  if (!online.value) {
    return <div class="pill bad"><span class="dot" />OFFLINE</div>;
  }
  if (readyMode) {
    const r = status.value?.ready;
    return <div class={`pill ${r ? 'ok' : 'warn'}`}><span class="dot" />{r ? 'READY' : 'NOT READY'}</div>;
  }
  return <div class="pill ok"><span class="dot" />ONLINE</div>;
}

export function TopBar({ readyMode = false }: { readyMode?: boolean }) {
  const b = battery.value;
  return (
    <div class="topbar">
      <ConnPill readyMode={readyMode} />
      <div class="topstats">
        <span><span class="k">LAT </span>{online.value && rttMs.value !== null ? `${rttMs.value} ms` : '—'}</span>
        <span><span class="k">BATT </span>{b ? `${b.v.toFixed(1)} V` : '—'}</span>
      </div>
    </div>
  );
}

export async function emergency(): Promise<void> {
  // Both ways at once; the gateway runs one e-stop for both.
  send({ type: 'estop' });
  try {
    const r = await post<{ car_held: boolean }>('/api/estop');
    toast(r.car_held ? 'error' : 'warn',
      r.car_held ? 'E-STOP: auto ferma' : 'E-STOP inviato, conferma incompleta: guarda l\'auto');
  } catch (e: any) {
    toast('error', `E-STOP via REST fallito: ${e.message}`);
  }
}

export function EstopButton({ compact = false }: { compact?: boolean }) {
  // NOT gated on the WebSocket. /api/estop is REST, a separate connection, and
  // the gateway runs the e-stop itself (f1tenth_ctl is its own fallback), so a
  // shaky socket is no reason to take the button away -- which is what used to
  // happen, at exactly the moment the button was most wanted. It goes only on
  // what the gateway reports about its reach (store.estopReach).
  const reach = estopReach.value;
  if (!reach.ok) {
    return (
      <button type="button" class="estop unreachable" disabled
        style={compact ? { height: '52px' } : undefined} data-testid="estop-unavailable">
        <span>E-STOP NON DISPONIBILE</span>
        <small>{reach.why ?? 'arresto affidato al watchdog'}</small>
      </button>
    );
  }
  return (
    <button type="button" class="estop" onClick={emergency}
      style={compact ? { height: '52px', fontSize: '18px' } : undefined} data-testid="estop">
      {Icon.octagon(compact ? 22 : 26)}EMERGENZA
    </button>
  );
}

export function LostBanner() {
  const [, tick] = useState(0);
  useEffect(() => {
    const t = setInterval(() => tick((x) => x + 1), 200);
    return () => clearInterval(t);
  }, []);
  if (conn.value !== 'lost' && conn.value !== 'connecting') return null;
  const ago = lastMsgAt.value ? (performance.now() - lastMsgAt.value) / 1000 : null;
  return (
    <div class="banner lost" role="alert" data-testid="lost-banner">
      {Icon.wifiOff()}
      <div class="col">
        <span class="t">CONNESSIONE PERSA</span>
        <span class="s">{ago !== null && lastMsgAt.value ? `ultimo messaggio ${ago.toFixed(1)} s fa · ` : ''}riprovo…</span>
      </div>
    </div>
  );
}

export function LatchedBanner() {
  // gatewayUp, not online: a latched e-stop is the one thing that must not be
  // hidden because the WebSocket dropped. The REST poll keeps the status fresh
  // in that state (ws.ts), so the latch is still known.
  if (!latched.value || !gatewayUp.value) return null;
  const allowed = tel.value?.reset_allowed ?? status.value?.reset_allowed;
  return (
    <div class="banner latched" role="alert" data-testid="estop-banner">
      {Icon.octagon(26)}
      <div class="col">
        <span class="t" style={{ color: '#FFFFFF' }}>E-STOP ATTIVO</span>
        <span class="s">missioni e guida bloccate · {allowed?.ok ? 'sblocco: riavvia il BT' : (allowed?.why ?? 'riavvia il BT')}</span>
      </div>
      <HoldButton ms={2000} label="SBLOCCA" disabled={!allowed?.ok} small onDone={resetEstop}
        color="#FFFFFF" testid="estop-reset" />
    </div>
  );
}

async function resetEstop(): Promise<void> {
  const { waitJob } = await import('../api');
  try {
    const job = await post('/api/estop/reset');
    toast('warn', 'Sblocco: riavvio del BT…', 6000);
    const j = await waitJob(job, 120_000);
    if (j.state === 'done') toast('info', 'E-STOP sbloccato');
    else toast('error', `Sblocco fallito: ${j.result?.error ?? 'vedi f1tenth_ctl'}`);
  } catch (e: any) {
    toast('error', `Sblocco rifiutato: ${e.message}`);
  }
}

export function Switch({ on, busy, disabled, label, onToggle }: {
  on: boolean; busy?: boolean; disabled?: boolean; label: string; onToggle: () => void;
}) {
  return (
    <button type="button" role="switch" aria-checked={on ? 'true' : 'false'} aria-label={label}
      class={`switch${busy ? ' busy' : ''}`} disabled={disabled || busy} onClick={onToggle}>
      <span class="knob" />
    </button>
  );
}

/** Press and hold for `ms`; a ring fills; release early and nothing happens. */
export function HoldButton({ ms, label, onDone, disabled, color = '#F5A524', small, testid, icon }: {
  ms: number; label: string; onDone: () => void; disabled?: boolean; color?: string;
  small?: boolean; testid?: string; icon?: ComponentChildren;
}) {
  const [p, setP] = useState(0);
  const start = useRef<number | null>(null);
  const raf = useRef<number>(0);
  const stop = () => {
    start.current = null;
    cancelAnimationFrame(raf.current);
    setP(0);
  };
  const frame = () => {
    if (start.current === null) return;
    const f = Math.min(1, (performance.now() - start.current) / ms);
    setP(f);
    if (f >= 1) {
      start.current = null;
      onDone();
      setTimeout(() => setP(0), 300);
      return;
    }
    raf.current = requestAnimationFrame(frame);
  };
  const down = (e: PointerEvent) => {
    if (disabled) return;
    capture(e);
    start.current = performance.now();
    raf.current = requestAnimationFrame(frame);
  };
  useEffect(() => () => cancelAnimationFrame(raf.current), []);
  const r = small ? 14 : 16;
  const c = 2 * Math.PI * r;
  return (
    <button type="button" disabled={disabled} data-testid={testid}
      onPointerDown={down} onPointerUp={stop} onPointerCancel={stop} onPointerLeave={stop}
      onContextMenu={(e) => e.preventDefault()}
      style={{
        display: 'flex', alignItems: 'center', justifyContent: 'center', gap: '10px',
        minHeight: small ? '44px' : '60px', padding: small ? '0 10px' : '0 16px', borderRadius: '16px',
        border: `2px solid ${color}`, color, background: 'transparent', fontFamily: 'var(--mono)',
        fontSize: small ? '12px' : '15px', fontWeight: 800, letterSpacing: '0.1em', flexShrink: 0,
        width: small ? undefined : '100%', opacity: disabled ? 0.4 : 1, touchAction: 'none',
      }}>
      <svg width={2 * r + 6} height={2 * r + 6} viewBox={`0 0 ${2 * r + 6} ${2 * r + 6}`} aria-hidden="true">
        <circle cx={r + 3} cy={r + 3} r={r} fill="none" stroke={color} stroke-opacity="0.25" stroke-width="3" />
        <circle cx={r + 3} cy={r + 3} r={r} fill="none" stroke={color} stroke-width="3"
          stroke-dasharray={`${c * p} ${c}`} transform={`rotate(-90 ${r + 3} ${r + 3})`} />
      </svg>
      {icon}{label}
    </button>
  );
}

/** Drag the thumb to the end to start; a tap is not enough. */
export function SlideToStart({ disabled, onDone, label = 'SCORRI PER AVVIARE' }: {
  disabled?: boolean; onDone: () => void; label?: string;
}) {
  const track = useRef<HTMLDivElement>(null);
  const [x, setX] = useState(0);
  const drag = useRef<{ id: number; x0: number } | null>(null);
  const max = () => (track.current ? track.current.clientWidth - 68 : 200);
  const down = (e: PointerEvent) => {
    if (disabled) return;
    capture(e);
    drag.current = { id: e.pointerId, x0: e.clientX - x };
  };
  const move = (e: PointerEvent) => {
    if (!drag.current || drag.current.id !== e.pointerId) return;
    setX(Math.max(0, Math.min(max(), e.clientX - drag.current.x0)));
  };
  const up = () => {
    if (!drag.current) return;
    drag.current = null;
    if (x >= max() - 4) onDone();
    setX(0);
  };
  const green = disabled ? 'var(--disabled)' : 'var(--green)';
  return (
    <div ref={track} role="slider" aria-label="Scorri per avviare la missione" aria-disabled={disabled}
      aria-valuemin={0} aria-valuemax={100} aria-valuenow={Math.round((x / max()) * 100)}
      data-testid="slide-to-start"
      style={{
        position: 'relative', height: '68px', borderRadius: '999px', flexShrink: 0,
        background: disabled ? 'rgba(94,102,114,0.12)' : 'rgba(52,210,123,0.10)',
        border: `1.5px solid ${disabled ? 'var(--border)' : 'rgba(52,210,123,0.5)'}`,
        display: 'flex', alignItems: 'center', color: green, touchAction: 'none',
      }}>
      <span style={{ flex: 1, textAlign: 'center', paddingLeft: '50px', fontFamily: 'var(--mono)',
        fontSize: '14px', fontWeight: 700, letterSpacing: '0.12em' }}>{label}</span>
      <span onPointerDown={down} onPointerMove={move} onPointerUp={up} onPointerCancel={up}
        data-testid="slide-thumb"
        style={{
          position: 'absolute', left: `${6 + x}px`, top: '5px', width: '56px', height: '56px',
          borderRadius: '50%', background: green, color: 'var(--bg)', display: 'flex',
          alignItems: 'center', justifyContent: 'center', cursor: disabled ? 'not-allowed' : 'grab',
        }}>{Icon.arrow()}</span>
    </div>
  );
}

/** The newest message only: a stack of them would cover the screen's controls. */
export function Toasts({ bottom = false }: { bottom?: boolean }) {
  return (
    <div class={bottom ? 'toasts bottom' : 'toasts'} aria-live="polite">
      {toasts.value.slice(-1).map((t) => <div class={`toast ${t.level}`} key={t.id}>{t.text}</div>)}
    </div>
  );
}
