// lamacchinina, the view once called Cockpit: route and code keep "cockpit"
// (docs/pwa/design/Main.dc.html). READY, the failing checks, the three processes,
// speed / VESC temperature / CPU.
import { post, waitJob } from '../api';
import { online, status, tel, toast } from '../store';
import type { Check, Proc } from '../store';
import { fmt, Icon, Switch } from '../components/ui';

const CHECK_LABEL: Record<string, string> = {
  odometry: 'Odometria (EKF)',
  scan: 'LiDAR /scan',
  tf_map_base_link: 'Localizzazione (TF map→base_link)',
  mission_service: 'Servizio missioni',
  estop_latched: 'E-STOP',
  battery: 'Batteria',
  wheel_odom: 'Odometria ruote',
  stack: 'Stack',
};

function detail(c: Check): string {
  if (c.name === 'estop_latched') return 'latched: riavvia il BT (SBLOCCA)';
  const v = typeof c.value === 'number' ? Number(c.value).toFixed(2) : String(c.value ?? '—');
  return `${v} · soglia ${c.want}`;
}

function procLine(name: string, p: Proc | undefined): [string, string] {
  if (!p) return ['?', 'var(--muted)'];
  if (p.pending === 'starting') return ['in avvio…', 'var(--amber)'];
  if (p.pending === 'stopping') return ['in arresto…', 'var(--amber)'];
  if (p.state !== 'running') return [p.state === 'crashed' ? 'crashato' : 'fermo',
    p.state === 'crashed' ? 'var(--red)' : 'var(--muted)'];
  const parts = ['running'];
  if (name === 'stack' && p.nodes) parts.push(`${p.nodes} nodi`);
  if (name === 'planner') parts.push('plan-only');
  if (p.uptime_s) {
    const s = Math.round(p.uptime_s);
    parts.push(`${String(Math.floor(s / 3600)).padStart(2, '0')}:${String(Math.floor(s / 60) % 60).padStart(2, '0')}:${String(s % 60).padStart(2, '0')}`);
  }
  if (p.managed === false) parts.push('avviato a mano');
  return [parts.join(' · '), 'var(--green)'];
}

async function toggle(name: string, running: boolean): Promise<void> {
  if (name === 'stack' && running && !confirm('Fermare lo stack? La macchina perde il controllo.')) return;
  try {
    const job = await post(`/api/process/${name}`, { action: running ? 'stop' : 'start' });
    const j = await waitJob(job, 120_000);
    if (j.state !== 'done') toast('error', `${name}: ${j.result?.error ?? 'fallito'}`);
  } catch (e: any) {
    toast('error', `${name}: ${e.message}`);
  }
}

export function Cockpit() {
  const s = status.value;
  const ready = !!s?.ready;
  const failing = (s?.checks ?? []).filter((c) => c.required && !c.ok);
  const stackDown = s && s.processes?.stack?.state !== 'running';
  const t = tel.value;
  return (
    <>
      <div class="titlerow"><h1>lamacchinina</h1><span class="mono muted" style={{ fontSize: '13px' }}>F1T-01</span></div>
      {!online.value ? (
        <section aria-label="Stato di prontezza" data-testid="ready-card" style={{
          border: '1.5px dashed #3A404B', borderRadius: '20px', padding: '18px',
          display: 'flex', flexDirection: 'column', gap: '10px' }}>
          <span class="label">STATO SISTEMA</span>
          <span class="mono" style={{ fontSize: '46px', lineHeight: 1, fontWeight: 800, color: 'var(--disabled)' }}>— — —</span>
          <span style={{ fontSize: '14px', color: 'var(--muted)' }}>Stato sconosciuto: i dati mostrati sono fermi all'ultimo aggiornamento.</span>
        </section>
      ) : (
      <section aria-label="Stato di prontezza" data-testid="ready-card" style={{
        border: `1.5px solid ${ready ? 'rgba(52,210,123,0.55)' : 'rgba(245,165,36,0.55)'}`,
        background: ready ? 'rgba(52,210,123,0.07)' : 'rgba(245,165,36,0.07)',
        borderRadius: '20px', padding: '18px', display: 'flex', flexDirection: 'column', gap: '14px',
      }}>
        <div style={{ display: 'flex', alignItems: 'flex-end', justifyContent: 'space-between' }}>
          <div style={{ display: 'flex', flexDirection: 'column', gap: '4px' }}>
            <span class="label">STATO SISTEMA</span>
            <span class="mono" style={{ fontSize: '46px', lineHeight: 1, fontWeight: 800,
              color: ready ? 'var(--green)' : 'var(--amber)', letterSpacing: '-0.02em' }}>
              {ready ? 'READY' : 'NOT READY'}</span>
          </div>
          <div class="mono" style={{ textAlign: 'right' }}>
            <div style={{ fontSize: '26px', fontWeight: 700 }}>{s ? `${s.checks_ok}/${s.checks_total}` : '—'}</div>
            <div style={{ fontSize: '11px', color: 'var(--muted)', letterSpacing: '0.08em' }}>CHECK OK</div>
          </div>
        </div>
        {(failing.length > 0 || stackDown) && (
          <ul style={{ listStyle: 'none', margin: 0, padding: 0, display: 'flex', flexDirection: 'column', gap: '8px' }}>
            {stackDown && (
              <li class="checkrow">{Icon.warn()}<div><b>Stack</b><span class="mono muted">processo non attivo: nessun check misurato</span></div></li>)}
            {failing.map((c) => (
              <li class="checkrow" key={c.name}>
                {c.name === 'battery' || c.name === 'estop_latched' ? Icon.error() : Icon.warn()}
                <div><b>{CHECK_LABEL[c.name] ?? c.name}</b><span class="mono muted">{detail(c)}</span></div>
              </li>))}
          </ul>)}
      </section>)}

      <section aria-label="Processi" style={{ display: 'flex', flexDirection: 'column', gap: '8px' }}>
        <span class="label">PROCESSI{online.value ? '' : ' · COMANDI DISATTIVATI'}</span>
        <div class={`card${online.value ? '' : ' disabled-zone'}`} style={{ overflow: 'hidden' }}>
          {(['stack', 'planner', 'logger'] as const).map((name, i) => {
            const p = s?.processes?.[name];
            const [line, colour] = procLine(name, p);
            const running = p?.state === 'running';
            return (
              <div key={name} style={{ display: 'flex', alignItems: 'center', gap: '12px', padding: '14px 16px',
                borderBottom: i < 2 ? '1px solid var(--border)' : '0' }}>
                <div style={{ flex: 1, display: 'flex', flexDirection: 'column', gap: '3px' }}>
                  <span style={{ fontSize: '17px', fontWeight: 700 }}>{name[0].toUpperCase() + name.slice(1)}</span>
                  <span class="mono" style={{ fontSize: '12px', color: colour }} data-testid={`proc-${name}`}>{online.value ? line : '?'}</span>
                </div>
                <Switch on={running} busy={!!p?.pending} disabled={!online.value || !p}
                  label={`Attiva ${name}`} onToggle={() => toggle(name, running)} />
              </div>);
          })}
        </div>
      </section>

      <div class="grid3">
        <div class="stat"><span class="k">VEL m/s</span><span class="v">{fmt(t?.v_meas, 2)}</span></div>
        <div class="stat"><span class="k">VESC °C</span><span class="v">{fmt(s?.vesc_temp_c, 0)}</span></div>
        <div class="stat"><span class="k">CPU %</span><span class="v">{fmt(s?.cpu_percent, 0)}</span></div>
      </div>
      {!online.value && (
        <div class="card pad" style={{ display: 'flex', gap: '12px', fontSize: '14px', color: 'var(--text-2)' }}>
          La macchina si ferma da sola se perde la guida (watchdog di 250 ms); una missione avviata
          dall'app viene interrotta dopo 2 s senza app. I comandi tornano attivi al ricollegamento.
        </div>)}
    </>
  );
}
