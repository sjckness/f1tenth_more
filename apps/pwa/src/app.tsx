// The shell: pairing, the hash router, the portrait layout with EMERGENZA always visible,
// the banners (connection lost, E-STOP), the screen wake lock during missions and Guida.
import { useEffect, useState } from 'preact/hooks';
import { openSession } from './api';
import { conn, cpuFrom, missionActive, online, setToken, token, toast, wakeLockMissing } from './store';
import { connect, disconnect } from './ws';
import { EstopButton, LatchedBanner, LostBanner, TabBar, Toasts, TopBar } from './components/ui';
import { Cockpit } from './screens/Cockpit';
import { Cpu } from './screens/Cpu';
import { Guida } from './screens/Guida';
import { Missione } from './screens/Missione';
import { Run } from './screens/Run';
import { Telemetria } from './screens/Telemetria';

// cpu: the CPU view, no tab of its own (the CPU tile opens it, its arrow goes back).
const ROUTES = ['cockpit', 'missione', 'guida', 'telemetria', 'run', 'cpu'];

function useRoute(): string {
  const get = () => {
    const r = location.hash.replace(/^#\/?/, '');
    return ROUTES.includes(r) ? r : 'cockpit';
  };
  const [route, setRoute] = useState(get());
  useEffect(() => {
    const f = () => setRoute(get());
    window.addEventListener('hashchange', f);
    return () => window.removeEventListener('hashchange', f);
  }, []);
  return route;
}

let lock: any = null;
async function holdScreen(want: boolean): Promise<void> {
  const nav: any = navigator;
  if (!want) {
    try { await lock?.release(); } catch { /* already gone */ }
    lock = null;
    return;
  }
  if (!nav.wakeLock?.request) {
    wakeLockMissing.value = true;
    return;
  }
  if (lock && !lock.released) return;
  try {
    lock = await nav.wakeLock.request('screen');
    wakeLockMissing.value = false;
  } catch {
    wakeLockMissing.value = true;
  }
}

function Pair() {
  const [value, setValue] = useState('');
  const pair = async () => {
    setToken(value.trim());
    try {
      await openSession();
      connect();
    } catch (e: any) {
      setToken(null);
      toast('error', e.status === 401 ? 'Token errato' : `Gateway: ${e.message}`);
    }
  };
  return (
    <main class="scroll" style={{ justifyContent: 'center', paddingTop: 'calc(40px + var(--safe-top))' }}>
      <h1>F1Tenth Lab Control</h1>
      <p class="muted">Incolla il token del gateway (F1T_GATEWAY_TOKEN sull'Orin). Resta su questo iPhone.</p>
      <p class="muted">Oppure inquadra il QR stampato sull'Orin (<span style={{ fontFamily: 'var(--mono)' }}>tools/pairing_qr.py</span>): abbina da solo.</p>
      <input type="password" value={value} onInput={(e) => setValue((e.target as HTMLInputElement).value)}
        autoComplete="off" placeholder="token" data-testid="token"
        style={{ minHeight: '52px', borderRadius: '14px', border: '1px solid var(--border)', background: 'var(--surface)',
          color: 'var(--text)', padding: '0 14px', fontSize: '16px', fontFamily: 'var(--mono)' }} />
      <button type="button" class="btn primary" disabled={value.trim().length < 32} onClick={pair} data-testid="pair">ABBINA</button>
    </main>
  );
}

export function App() {
  const route = useRoute();
  useEffect(() => {
    if (token.value) {
      openSession().catch(() => undefined);
      connect();
    }
    return disconnect;
  }, [token.value]);
  useEffect(() => {
    void holdScreen(online.value && (missionActive.value || route === 'guida'));
  }, [online.value, missionActive.value, route]);
  useEffect(() => {
    const f = () => {
      if (!document.hidden && (missionActive.value || route === 'guida')) void holdScreen(true);
    };
    document.addEventListener('visibilitychange', f);
    return () => document.removeEventListener('visibilitychange', f);
  }, [route]);

  if (!token.value || conn.value === 'unauthorized') {
    return (<div class="shell"><Pair />{conn.value === 'unauthorized' && (
      <p style={{ color: 'var(--red)', textAlign: 'center' }}>Token rifiutato dal gateway.</p>)}<Toasts bottom /></div>);
  }
  if (route === 'guida') {
    return (<div class="shell"><LostBanner /><Guida /><Toasts bottom /></div>);
  }
  const Screen = { cockpit: Cockpit, missione: Missione, telemetria: Telemetria, run: Run, cpu: Cpu }[route] ?? Cockpit;
  const needWake = wakeLockMissing.value && missionActive.value;
  return (
    <div class="shell">
      <LostBanner />
      <LatchedBanner />
      {conn.value !== 'lost' && conn.value !== 'connecting' && <TopBar readyMode={route === 'missione'} />}
      {needWake && (
        <div class="banner" style={{ background: 'rgba(245,165,36,0.12)', border: '1.5px solid var(--amber)' }}>
          <div class="col"><span class="t" style={{ color: 'var(--amber)' }}>SCHERMO</span>
            <span class="s">Blocco schermo non impedibile su questo browser: tieni l'app aperta, o la missione si interrompe dopo 2 s.</span></div>
        </div>)}
      <div class="stage">
        <main class="scroll" aria-disabled={!online.value}><Screen /></main>
        <Toasts />
      </div>
      <div class="estop-wrap"><EstopButton /></div>
      <TabBar route={route === 'cpu' ? (cpuFrom.value ?? 'cockpit') : route} />
    </div>
  );
}
