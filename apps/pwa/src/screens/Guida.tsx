// Guida (design/Guida.dc.html): landscape only, two thumbs. Gas = dead-man: the car moves
// only while that thumb is down; lifting it sends a zero at once. 20 Hz while held.
// Gas below the dead zone = reverse (gateway teleop.py): RETROMARCIA in amber while it lasts,
// and the pad's ends read AVANTI / RETRO (the lower half is no longer a safe place to rest).
import { useEffect, useRef, useState } from 'preact/hooks';
import { battery, drive, hello, online, rttMs, tel, wakeLockMissing } from '../store';
import { send } from '../ws';
import { EstopButton, Icon, MockBadge } from '../components/ui';
import { Joystick } from '../components/Joystick';

function useLandscape(): boolean {
  const q = () => window.matchMedia('(orientation: landscape)').matches;
  const [land, setLand] = useState(q());
  useEffect(() => {
    const mq = window.matchMedia('(orientation: landscape)');
    const f = () => setLand(mq.matches);
    mq.addEventListener('change', f);
    window.addEventListener('resize', f);
    return () => { mq.removeEventListener('change', f); window.removeEventListener('resize', f); };
  }, []);
  return land;
}

export function Guida() {
  const landscape = useLandscape();
  const cfg = hello.value?.teleop;
  const minV = cfg?.min_speed_mps ?? 0.4;
  const maxV = cfg?.max_speed_mps ?? 1.0;
  const step = cfg?.limit_step_mps ?? 0.1;
  const [limit, setLimit] = useState(cfg?.default_limit_mps ?? 0.6);
  const [thr, setThr] = useState(0);
  const [steer, setSteer] = useState(0);
  const [held, setHeld] = useState(false);
  const st = useRef({ thr: 0, steer: 0, held: false, seq: 0, limit });
  st.current.limit = limit;
  const granted = drive.value.state === 'granted' || drive.value.state === 'released';
  const canDrive = online.value && granted && landscape;

  // The lease: asked on entry (and after a reconnect), given back on exit.
  useEffect(() => {
    if (online.value && landscape) {
      drive.value = { state: 'acquiring' };
      send({ type: 'drive_acquire' });
    }
    return () => {
      if (st.current.held) send({ type: 'teleop', deadman: false });
      send({ type: 'drive_release' });
      drive.value = { state: 'idle' };
    };
  }, [online.value, landscape]);

  // 20 Hz while the thumb is on the gas.
  useEffect(() => {
    const t = setInterval(() => {
      const s = st.current;
      if (!s.held) return;
      send({ type: 'teleop', seq: ++s.seq, thr: s.thr, steer: -s.steer, limit: s.limit,
             deadman: true });
    }, 50);
    const hide = () => { if (document.hidden) release(); };
    document.addEventListener('visibilitychange', hide);
    window.addEventListener('pagehide', release);
    return () => {
      clearInterval(t);
      document.removeEventListener('visibilitychange', hide);
      window.removeEventListener('pagehide', release);
    };
  }, []);

  function release() {
    const s = st.current;
    if (s.held) {
      s.held = false;
      s.thr = 0;
      send({ type: 'teleop', deadman: false });
      setHeld(false);
      setThr(0);
    }
  }

  const onGas = (v: number, active: boolean) => {
    const s = st.current;
    if (!active) { release(); return; }
    s.thr = v;
    setThr(v);
    if (!s.held) {
      s.held = true;
      setHeld(true);
      send({ type: 'teleop', seq: ++s.seq, thr: v, steer: -s.steer, limit: s.limit, deadman: true });
    }
  };
  const onSteer = (v: number) => { st.current.steer = v; setSteer(v); };

  if (!landscape) {
    return (
      <div style={{ height: '100%', display: 'flex', flexDirection: 'column', alignItems: 'center',
        justifyContent: 'center', gap: '18px', padding: '24px', textAlign: 'center' }} data-testid="rotate">
        <svg width="64" height="64" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="1.6" aria-hidden="true">
          <rect x="7" y="2" width="10" height="20" rx="2" /><path d="M2 14a8 8 0 0 0 8 8M22 10a8 8 0 0 0-8-8" /></svg>
        <span style={{ fontSize: '22px', fontWeight: 800 }}>Ruota il telefono</span>
        <span class="muted" style={{ fontSize: '14px' }}>La guida manuale funziona solo in orizzontale, con due pollici.</span>
        <a href="#/cockpit" class="btn" style={{ display: 'inline-flex', alignItems: 'center', textDecoration: 'none' }}>Torna al Cockpit</a>
      </div>
    );
  }
  const t = tel.value;
  const vReal = t?.v_meas ?? 0;
  const vCmd = t?.v_cmd ?? 0;
  // Reverse: what this phone sends (at once, no round trip) or what the gateway still
  // commands. reverse_wait: held at 0 until the car has stopped (direction change).
  const maxRev = cfg?.max_reverse_speed_mps ?? 0;
  const reverse = (held && maxRev > 0 && thr < -(cfg?.deadzone ?? 0.15))
    || (t?.drive?.speed_cmd ?? 0) < 0;
  const revWait = reverse && !!t?.drive?.reverse_wait;
  const maxSteerDeg = ((cfg?.max_steer_rad ?? 0.3) * 180) / Math.PI;
  const b = battery.value;
  const reason = drive.value.state === 'acquiring' ? 'richiesta della guida…'
    : !online.value ? 'offline' : !granted ? (drive.value.reason ?? 'guida non disponibile') : null;
  return (
    <div style={{ height: '100%', boxSizing: 'border-box', display: 'flex', flexDirection: 'column', gap: '12px',
      padding: 'calc(12px + var(--safe-top)) calc(24px + var(--safe-right)) calc(14px + var(--safe-bottom)) calc(24px + var(--safe-left))',
      overflow: 'hidden' }} data-testid="guida">
      <div style={{ display: 'flex', alignItems: 'center', gap: '12px', height: '52px' }}>
        <a href="#/cockpit" aria-label="Torna al Cockpit" style={{ width: '48px', height: '48px', borderRadius: '14px',
          background: 'var(--surface)', border: '1px solid var(--border)', display: 'flex', alignItems: 'center',
          justifyContent: 'center', color: 'var(--text)' }}>{Icon.back()}</a>
        <div class="mono" style={{ display: 'flex', flexDirection: 'column', gap: '2px', fontSize: '12px' }}>
          <span style={{ display: 'flex', alignItems: 'center', gap: '6px', fontWeight: 700,
            color: online.value ? 'var(--green)' : 'var(--red)' }}>
            <span style={{ width: '7px', height: '7px', borderRadius: '50%', background: 'currentColor' }} />
            {online.value ? `ONLINE · ${rttMs.value ?? '—'} ms` : 'OFFLINE'}<MockBadge /></span>
          <span><span class="muted">BATT </span>{b ? `${b.v.toFixed(1)} V` : '—'}</span>
          {wakeLockMissing.value && (
            <span style={{ color: 'var(--amber)' }} data-testid="wake-warning">schermo: può spegnersi (= stop)</span>)}
        </div>
        <div style={{ flex: 1 }} />
        <div style={{ width: '240px' }}><EstopButton compact /></div>
        <div style={{ flex: 1 }} />
        <div role="group" aria-label="Limite di velocità" style={{ display: 'flex', alignItems: 'center', gap: '4px',
          background: 'var(--surface)', border: '1px solid var(--border)', borderRadius: '14px', padding: '4px' }}>
          <button type="button" aria-label="Riduci limite" data-testid="limit-down"
            onClick={() => setLimit(Math.max(minV, Math.round((limit - step) * 10) / 10))}
            style={{ width: '44px', height: '44px', borderRadius: '10px', border: 0, background: 'var(--surface-2)', fontSize: '22px' }}>−</button>
          <div style={{ display: 'flex', flexDirection: 'column', alignItems: 'center', minWidth: '74px' }}>
            <span class="mono muted" style={{ fontSize: '10px', letterSpacing: '0.1em' }}>LIMITE</span>
            <span class="mono" style={{ fontSize: '17px', fontWeight: 700 }} data-testid="limit">{limit.toFixed(1)} m/s</span>
          </div>
          <button type="button" aria-label="Aumenta limite" data-testid="limit-up"
            onClick={() => setLimit(Math.min(maxV, Math.round((limit + step) * 10) / 10))}
            style={{ width: '44px', height: '44px', borderRadius: '10px', border: 0, background: 'var(--surface-2)', fontSize: '22px' }}>+</button>
        </div>
      </div>

      <div style={{ flex: 1, display: 'flex', alignItems: 'center', gap: '28px' }}>
        <div style={{ display: 'flex', alignItems: 'center', gap: '12px' }}>
          <Joystick axis="v" label="Gas" disabled={!canDrive} glow={held} reverse={maxRev > 0}
            onChange={onGas} testid="gas" />
          <div class="mono" style={{ display: 'flex', flexDirection: 'column', gap: '2px' }}>
            <span class="muted" style={{ fontSize: '11px' }}>THR</span>
            <span style={{ fontSize: '20px', fontWeight: 700, color: reverse ? 'var(--amber)' : undefined }}>
              {thr >= 0 ? '+' : ''}{Math.round(thr * 100)}%</span>
            <span style={{ marginTop: '8px', fontSize: '10px', color: 'var(--green)', lineHeight: 1.3 }}>pollice<br />= dead-man</span>
          </div>
        </div>
        <div style={{ flex: 1, display: 'flex', flexDirection: 'column', alignItems: 'center', gap: '14px' }}>
          {/* Always laid out, shown only in reverse: nothing moves under the thumbs when it appears. */}
          <div role="status" aria-hidden={!reverse} class="mono" data-testid="reverse" style={{
            visibility: reverse ? 'visible' : 'hidden', display: 'flex', alignItems: 'center', gap: '8px',
            padding: '6px 14px', borderRadius: '999px', background: 'var(--amber)', color: 'var(--bg)',
            fontSize: '14px', fontWeight: 800, letterSpacing: '0.08em' }}>
            <svg width="14" height="14" viewBox="0 0 24 24" fill="currentColor" aria-hidden="true">
              <path d="M12 21l-8-9h5V3h6v9h5z" /></svg>
            RETROMARCIA
            <span style={{ fontSize: '11px', fontWeight: 700, letterSpacing: 0 }}>
              {revWait ? 'attesa arresto' : `max ${maxRev.toFixed(1)} m/s`}</span>
          </div>
          <div style={{ display: 'flex', gap: '22px', alignItems: 'flex-end' }}>
            <div style={{ display: 'flex', flexDirection: 'column', alignItems: 'center' }}>
              <span class="label" style={{ fontSize: '11px' }}>REALE</span>
              <span class="mono" style={{ fontSize: '54px', lineHeight: 1, fontWeight: 800 }} data-testid="v-real">{vReal.toFixed(2)}</span></div>
            <div style={{ display: 'flex', flexDirection: 'column', alignItems: 'center', paddingBottom: '4px' }}>
              <span class="label" style={{ fontSize: '11px' }}>CMD</span>
              <span class={`mono${vCmd < 0 ? '' : ' muted'}`} data-testid="v-cmd" style={{ fontSize: '30px', lineHeight: 1, fontWeight: 700,
                color: vCmd < 0 ? 'var(--amber)' : undefined }}>{vCmd.toFixed(2)}</span></div>
          </div>
          <div style={{ width: '220px', display: 'flex', flexDirection: 'column', gap: '4px' }}>
            <div style={{ position: 'relative', height: '8px', borderRadius: '999px', background: 'var(--border)' }}>
              <div style={{ position: 'absolute', left: 0, top: 0, bottom: 0, width: `${Math.min(100, (Math.max(0, vReal) / limit) * 100)}%`, borderRadius: '999px', background: 'var(--text)' }} />
              <div style={{ position: 'absolute', left: `${Math.min(100, (Math.max(0, vCmd) / limit) * 100)}%`, top: '-4px', width: '2px', height: '16px', background: 'var(--muted)' }} />
            </div>
            <div class="mono muted" style={{ display: 'flex', justifyContent: 'space-between', fontSize: '10px' }}>
              <span>0</span><span>m/s</span><span>{limit.toFixed(1)}</span></div>
          </div>
          {reason ? (
            <div role="status" class="mono" data-testid="drive-reason" style={{ padding: '12px 18px', borderRadius: '16px',
              border: '1.5px solid var(--amber)', background: 'rgba(245,165,36,0.10)', color: 'var(--amber)',
              fontSize: '13px', fontWeight: 700, textAlign: 'center', maxWidth: '300px',
              display: 'flex', flexDirection: 'column', gap: '8px', alignItems: 'center' }}>
              <span>{reason}</span>
              {online.value && ['refused', 'busy', 'revoked'].includes(drive.value.state) && (
                <button type="button" class="btn" data-testid="drive-retry"
                  onClick={() => { drive.value = { state: 'acquiring' }; send({ type: 'drive_acquire' }); }}>
                  RIPROVA</button>)}
            </div>
          ) : (
            <div role="status" class="mono" data-testid="deadman" style={{ display: 'flex', flexDirection: 'column', alignItems: 'center', gap: '4px',
              padding: '12px 18px', borderRadius: '16px', border: `1.5px solid ${held ? 'var(--green)' : 'var(--border)'}`,
              background: held ? 'rgba(52,210,123,0.12)' : 'transparent' }}>
              <span style={{ display: 'flex', alignItems: 'center', gap: '8px', fontSize: '13px', fontWeight: 800, letterSpacing: '0.08em',
                color: held ? 'var(--green)' : 'var(--muted)' }}>
                <span style={{ width: '9px', height: '9px', borderRadius: '50%', background: 'currentColor' }} />
                {held ? 'DEAD-MAN ATTIVO' : 'DEAD-MAN RILASCIATO'}</span>
              <span style={{ fontSize: '11px', color: 'var(--text-2)' }}>{held ? 'rilascia il gas = stop' : 'pollice sul gas per muovere'}</span>
            </div>)}
        </div>
        <div style={{ display: 'flex', flexDirection: 'column', alignItems: 'center', gap: '10px' }}>
          <div class="mono" style={{ display: 'flex', gap: '8px', alignItems: 'baseline' }}>
            <span class="muted" style={{ fontSize: '11px' }}>STERZO</span>
            <span style={{ fontSize: '20px', fontWeight: 700 }}>{steer >= 0 ? '+' : ''}{Math.round(steer * maxSteerDeg)}°</span>
          </div>
          <Joystick axis="h" label="Sterzo" disabled={!canDrive} onChange={(v) => onSteer(v)} testid="steer" />
        </div>
      </div>
    </div>
  );
}
