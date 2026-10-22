// The CPU view, opened by the CPU tile of lamacchinina or Telemetria: one bar per core with
// its load, the processes pinned to each core above it (the launch files' taskset -c), a
// legend with each one's cores and CPU %, and how many processes are not pinned. All from
// status.system (the gateway's sysstats.py: cores at 1 Hz, processes every 5 s).
import { cpuFrom, online, status } from '../store';
import type { PinnedProc, SysCore } from '../store';
import { fmt, Icon } from '../components/ui';

// Identity of a pinned process: six of the dataviz reference's dark categorical steps. Its
// yellow and red are left out: above the bars they would read as the amber and red load
// states. In this order the six pass the validator on --surface (CVD and normal-vision
// separation, contrast). The 7th to 12th process take the same hues with a dashed border,
// then dotted; the chip's name tells them apart in any case.
const PALETTE = ['#3987e5', '#d95926', '#199e70', '#9085e9', '#008300', '#d55181'];
const BORDERS = ['solid', 'dashed', 'dotted'];
const slots = new Map<string, number>();
const keyOf = (p: PinnedProc) => `${p.name}|${p.cores_text}`;

/**
 * Give each process pinned now a style that stays with it (name and cores), never its rank:
 * one that is gone frees its slot, a new one takes the lowest free slot. So two live
 * processes never share a style, and stack restarts do not use up the palette.
 */
function assignSlots(pinned: PinnedProc[]): void {
  const now = new Set(pinned.map(keyOf));
  for (const k of [...slots.keys()]) if (!now.has(k)) slots.delete(k);
  const used = new Set(slots.values());
  for (const p of pinned) {
    if (slots.has(keyOf(p))) continue;
    let slot = 0;
    while (used.has(slot)) slot++;
    slots.set(keyOf(p), slot);
    used.add(slot);
  }
}

function chipStyle(p: PinnedProc) {
  const slot = slots.get(keyOf(p)) ?? 0;
  const c = PALETTE[slot % PALETTE.length];
  const border = BORDERS[Math.floor(slot / PALETTE.length) % BORDERS.length];
  return { borderColor: c, borderStyle: border, background: `${c}33` };
}

// A chip's width on the iPhone (390 px, 4 bars per row), in characters.
const CHIP_CHARS = 10;

/** A chip's name: no _node, cut at an underscore to fit (ekf_global_filter_node: ekf_global,
 *  slam_toolbox: slam). The legend has the full name. */
function chipLabel(name: string): string {
  const base = name.replace(/_node$/, '');
  if (base.length <= CHIP_CHARS) return base;
  const words = base.split('_');
  let out = words[0];
  for (const w of words.slice(1)) {
    if (out.length + 1 + w.length > CHIP_CHARS) break;
    out += `_${w}`;
  }
  return out;
}

/** Load as a state, the app's way: data grey while normal, amber high, red near saturation. */
function loadColour(pct: number | null): string {
  if (pct === null) return 'var(--disabled)';
  if (pct >= 85) return 'var(--red)';
  if (pct >= 60) return 'var(--amber)';
  return 'var(--text-2)';
}

function back(): void {
  if (cpuFrom.value) {
    cpuFrom.value = null;
    history.back();
  } else {
    location.hash = '#/cockpit';
  }
}

function CoreBar({ core, pins }: { core: SysCore; pins: PinnedProc[] }) {
  const pct = core.online ? core.percent : null;
  const names = pins.map((p) => p.name).join(', ');
  const label = `core ${core.id}: ${core.online ? `${fmt(pct, 0)} %` : 'offline'}` +
    (names ? `, vincolati: ${names}` : '');
  return (
    <div class={`core${core.online ? '' : ' off'}`} role="img" aria-label={label} data-testid={`core-${core.id}`}>
      <div class="chips">
        {pins.map((p) => (
          <span class="chip" key={p.pid} style={chipStyle(p)}>{chipLabel(p.name)}</span>))}
      </div>
      <div class="bar">
        {pct !== null && <div class="fill" style={{ height: `${Math.max(2, pct)}%`, background: loadColour(pct) }} />}
      </div>
      <span class="pct mono">{core.online ? fmt(pct, 0) : 'off'}</span>
      <span class="id mono">cpu{core.id}</span>
    </div>
  );
}

export function Cpu() {
  const sys = status.value?.system ?? null;
  const cores = sys?.cores ?? [];
  const pinned = sys?.pinned ?? [];
  assignSlots(pinned);              // new ones in the gateway's order (by first core)
  // Rows of at most 4 bars: 12 cores are 3 rows, and a chip has room for a readable name.
  const rows = Math.max(1, Math.ceil(cores.length / 4));
  const cols = Math.max(1, Math.ceil(cores.length / rows));
  const nOnline = cores.filter((c) => c.online).length;
  return (
    <>
      <div class="titlerow" style={{ alignItems: 'center', gap: '12px' }}>
        <button type="button" class="backbtn" onClick={back} aria-label="Indietro" data-testid="cpu-back">{Icon.back()}</button>
        <h1 style={{ flex: 1 }}>CPU</h1>
        <span class="mono" style={{ fontSize: '22px', fontWeight: 700 }}>{fmt(sys?.cpu_percent, 0)}<span class="muted" style={{ fontSize: '14px' }}> %</span></span>
      </div>
      {!online.value && <p class="muted" style={{ margin: 0, fontSize: '13px' }}>Offline: i dati sono fermi all'ultimo aggiornamento.</p>}
      {!sys ? (
        <section class="card pad" aria-label="Core">
          <span class="label">CORE</span>
          <p class="muted" style={{ fontSize: '13px', margin: '8px 0 0' }}>Nessun dato dal gateway: arriva entro un secondo (gateway non aggiornato?).</p>
        </section>
      ) : (
        <>
          <section class="card" aria-label="Carico per core" style={{ padding: '12px 12px 14px', display: 'flex', flexDirection: 'column', gap: '10px' }}>
            <span class="label">CORE · {nOnline}/{cores.length} ONLINE</span>
            <div class="cores" style={{ gridTemplateColumns: `repeat(${cols}, minmax(0, 1fr))` }} data-testid="cores">
              {cores.map((c) => <CoreBar key={c.id} core={c} pins={pinned.filter((p) => p.cores.includes(c.id))} />)}
            </div>
            <div style={{ display: 'flex', gap: '12px', flexWrap: 'wrap', fontSize: '11px', color: 'var(--muted)' }}>
              {[['var(--text-2)', '< 60 %'], ['var(--amber)', '60–85 %'], ['var(--red)', '≥ 85 %']].map(([c, t]) => (
                <span key={t} style={{ display: 'flex', alignItems: 'center', gap: '5px' }}>
                  <span style={{ width: '10px', height: '10px', borderRadius: '3px', background: c }} />{t}</span>))}
              <span style={{ display: 'flex', alignItems: 'center', gap: '5px' }}>
                <span style={{ width: '10px', height: '10px', borderRadius: '3px', border: '1.5px dashed var(--disabled)' }} />offline</span>
            </div>
          </section>
          <section class="card pad" aria-label="Processi vincolati" style={{ display: 'flex', flexDirection: 'column', gap: '10px' }}>
            <span class="label">PROCESSI VINCOLATI · {pinned.length}</span>
            {pinned.length === 0 ? (
              <span class="muted" style={{ fontSize: '13px' }}>Nessun processo vincolato a core precisi (stack spento?).</span>
            ) : (
              <ul class="legend" data-testid="pinned">
                <li class="head mono"><span /><span>processo</span><span>core</span><span>CPU %</span></li>
                {pinned.map((p) => (
                  <li key={p.pid}>
                    <span class="sw" style={chipStyle(p)} />
                    <span class="name">{p.name}</span>
                    <span class="mono muted">{p.cores_text}</span>
                    <span class="mono">{fmt(p.cpu_percent, 0)}</span>
                  </li>))}
              </ul>)}
            <span style={{ fontSize: '14px' }} data-testid="unpinned">
              <b class="mono">{sys.unpinned ?? '—'}</b> processi non vincolati</span>
            <span class="muted" style={{ fontSize: '12px' }}>
              CPU %: 100 = un core, media degli ultimi 5 s. Thread del kernel esclusi
              {sys.kernel_threads != null && ` (${sys.kernel_threads})`}.</span>
          </section>
        </>)}
    </>
  );
}
