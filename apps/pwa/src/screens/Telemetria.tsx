// Telemetria (design/Telemetria.dc.html): the SLAM map with the path driven and the pose,
// the MPC's predicted horizon dashed (there is no raceline), speed over 30 s, the BT.
import { useEffect, useRef, useState } from 'preact/hooks';
import { api } from '../api';
import { bt, mapVersion, missionActive, path, speedHist, tel } from '../store';
import type { BtNode } from '../store';
import { fmt } from '../components/ui';

interface MapMeta { version: number; width: number; height: number; resolution: number;
  origin: { x: number; y: number } }

function Trajectory() {
  const [meta, setMeta] = useState<MapMeta | null>(null);
  useEffect(() => {
    if (mapVersion.value === null) return;
    api<MapMeta>('/api/map').then(setMeta).catch(() => setMeta(null));
  }, [mapVersion.value]);
  const W = 326;
  const H = 150;
  const pose = tel.value?.pose;
  const pts = path.value.points;
  const hz = path.value.horizon;
  const all = [...pts, ...hz, ...(pose ? [[pose.x, pose.y] as [number, number]] : [])];
  let x0 = -3; let x1 = 3; let y0 = -1.5; let y1 = 1.5;
  if (all.length) {
    x0 = Math.min(...all.map((p) => p[0])); x1 = Math.max(...all.map((p) => p[0]));
    y0 = Math.min(...all.map((p) => p[1])); y1 = Math.max(...all.map((p) => p[1]));
  }
  const pad = 0.8;
  x0 -= pad; x1 += pad; y0 -= pad; y1 += pad;
  // Same scale on both axes, the box widened to the SVG's aspect.
  let k = Math.min(W / (x1 - x0), H / (y1 - y0));
  if (x1 - x0 < 4 && y1 - y0 < 2) k = Math.min(W / 4, H / 2);
  const cx = (x0 + x1) / 2; const cy = (y0 + y1) / 2;
  const sx = (x: number) => W / 2 + (x - cx) * k;
  const sy = (y: number) => H / 2 - (y - cy) * k;
  const line = (p: [number, number][]) => p.map(([x, y]) => `${sx(x).toFixed(1)},${sy(y).toFixed(1)}`).join(' ');
  const yawDeg = pose ? Math.round((pose.yaw * 180) / Math.PI) : null;
  return (
    <section aria-label="Traiettoria" class="card" style={{ padding: '12px 14px', display: 'flex', flexDirection: 'column', gap: '6px' }}>
      <div class="mono" style={{ display: 'flex', justifyContent: 'space-between', fontSize: '12px' }}>
        <span class="muted" style={{ letterSpacing: '0.1em' }}>TRAIETTORIA 2D</span>
        <span>{pose ? `x ${pose.x.toFixed(2)} · y ${pose.y.toFixed(2)} · ψ ${yawDeg}°` : 'posa assente'}</span>
      </div>
      <svg viewBox={`0 0 ${W} ${H}`} width="100%" role="img" data-testid="trajectory"
        aria-label="Mappa con il percorso fatto, l'orizzonte MPC e la posa">
        <rect x="0" y="0" width={W} height={H} rx="10" fill="#0E1014" />
        {meta && (
          <image href={`/api/map.png?v=${meta.version}`} preserveAspectRatio="none"
            x={sx(meta.origin.x)} y={sy(meta.origin.y + meta.height * meta.resolution)}
            width={meta.width * meta.resolution * k} height={meta.height * meta.resolution * k} />)}
        {hz.length > 1 && <polyline points={line(hz)} fill="none" stroke="#7D8693" stroke-width="1.5" stroke-dasharray="5 4" />}
        {pts.length > 1 && <polyline points={line(pts)} fill="none" stroke="#ECEEF1" stroke-width="3" stroke-linecap="round" stroke-linejoin="round" />}
        {pose && (
          <g>
            <circle cx={sx(pose.x)} cy={sy(pose.y)} r="12" fill="none" stroke="#34D27B" stroke-opacity="0.35" stroke-width="2" />
            <circle cx={sx(pose.x)} cy={sy(pose.y)} r="7" fill="#34D27B" />
            <line x1={sx(pose.x)} y1={sy(pose.y)} x2={sx(pose.x + 0.5 * Math.cos(pose.yaw))}
              y2={sy(pose.y + 0.5 * Math.sin(pose.yaw))} stroke="#34D27B" stroke-width="2" />
          </g>)}
      </svg>
      <div style={{ display: 'flex', gap: '14px', fontSize: '11px', color: 'var(--muted)' }}>
        <span style={{ display: 'flex', alignItems: 'center', gap: '6px' }}><span style={{ width: '14px', height: '3px', background: '#ECEEF1' }} />percorsa</span>
        <span style={{ display: 'flex', alignItems: 'center', gap: '6px' }}><span style={{ width: '14px', height: 0, borderTop: '2px dashed #7D8693' }} />orizzonte MPC</span>
        {meta && <span style={{ display: 'flex', alignItems: 'center', gap: '6px' }}><span style={{ width: '10px', height: '10px', background: '#D8D8D8' }} />mappa SLAM</span>}
      </div>
    </section>
  );
}

function SpeedChart() {
  const hist = speedHist.value;
  const W = 326; const H = 104; const L = 18;
  const tEnd = hist.length ? hist[hist.length - 1].t : 0;
  const vmax = Math.max(1, ...hist.map((h) => Math.max(h.v ?? 0, h.cmd ?? 0)));
  const ceil = Math.ceil(vmax);
  const sx = (t: number) => L + ((t - (tEnd - 30)) / 30) * (W - L);
  const sy = (v: number) => 100 - (v / ceil) * 90;
  const poly = (key: 'v' | 'cmd') => hist.filter((h) => h[key] !== null)
    .map((h) => `${sx(h.t).toFixed(1)},${sy(h[key] as number).toFixed(1)}`).join(' ');
  const last = hist[hist.length - 1];
  return (
    <section aria-label="Velocità" class="card" style={{ padding: '12px 14px', display: 'flex', flexDirection: 'column', gap: '6px' }}>
      <div class="mono" style={{ display: 'flex', justifyContent: 'space-between', alignItems: 'baseline', fontSize: '12px' }}>
        <span class="muted" style={{ letterSpacing: '0.1em' }}>VELOCITÀ · 30 s</span>
        <span><span style={{ fontSize: '18px', fontWeight: 700 }}>{fmt(last?.v, 2)}</span><span class="muted"> / {fmt(last?.cmd, 2)} m/s</span></span>
      </div>
      <svg viewBox={`0 0 ${W} ${H}`} width="100%" role="img" aria-label="Velocità reale e comandata negli ultimi 30 secondi" data-testid="speed-chart">
        {Array.from({ length: ceil + 1 }, (_, i) => (
          <g key={i}>
            <line x1={L} y1={sy(i)} x2={W} y2={sy(i)} stroke={i === 0 ? '#2C323C' : '#20252D'} />
            {i > 0 && <text x="0" y={sy(i) + 4} fill="#9099A6" font-family="JetBrains Mono, monospace" font-size="10">{i}</text>}
          </g>))}
        <polyline points={poly('cmd')} fill="none" stroke="#7D8693" stroke-width="1.5" stroke-dasharray="5 4" />
        <polyline points={poly('v')} fill="none" stroke="#ECEEF1" stroke-width="2.5" stroke-linejoin="round" />
      </svg>
      <div style={{ display: 'flex', gap: '14px', fontSize: '11px', color: 'var(--muted)' }}>
        <span style={{ display: 'flex', alignItems: 'center', gap: '6px' }}><span style={{ width: '14px', height: '3px', background: '#ECEEF1' }} />reale</span>
        <span style={{ display: 'flex', alignItems: 'center', gap: '6px' }}><span style={{ width: '14px', height: 0, borderTop: '2px dashed #7D8693' }} />comandata</span>
      </div>
    </section>
  );
}

const STATUS_STYLE: Record<string, { stroke: string; fill: string; color: string; dash?: string }> = {
  success: { stroke: '#34D27B', fill: 'rgba(52,210,123,0.12)', color: '#34D27B' },
  running: { stroke: '#ECEEF1', fill: 'rgba(236,238,241,0.10)', color: '#ECEEF1' },
  failure: { stroke: '#FF5A50', fill: 'rgba(255,90,80,0.12)', color: '#FF5A50' },
  // A question answered "no", and anything failing on an idle car: not a
  // fault, so not red. Readable as "false", distinct from idle (never ticked).
  no: { stroke: '#5E6672', fill: 'rgba(94,102,114,0.10)', color: '#9099A6' },
  idle: { stroke: '#5E6672', fill: 'transparent', color: '#9099A6', dash: '4 3' },
};
const STATUS_LABEL: Record<string, string> = {
  success: 'successo', running: 'in corso', failure: 'errore', no: 'falso', idle: 'inattivo',
};

// The stack names every condition Is*, Has*, or one of these three
// (f1tenth_behavior/behaviours); any other leaf is an action.
const CONDITIONS = new Set(['ObjectSeen', 'CheckStopCondition', 'MissionActive']);

function isCondition(n: BtNode): boolean {
  return !n.children.length && (/^(Is|Has)[A-Z]/.test(n.name) || CONDITIONS.has(n.name));
}

/**
 * The style for a node, with red kept for failures that are really faults.
 *
 * Most of this tree answers questions, and FAILURE is how a condition says
 * "no": IsProximityTooClose fails on every tick of a clear corridor,
 * IsBatteryLow fails on a healthy battery, and the root Selector itself sits
 * in FAILURE whenever the car is simply idle. Painting all of that red made a
 * parked, healthy car look broken -- and left nothing to distinguish it from
 * the one case that matters, an action or a composite giving up mid-mission.
 */
function styleOf(n: BtNode, running: boolean) {
  if (n.status !== 'failure') return STATUS_STYLE[n.status] ?? STATUS_STYLE.idle;
  return running && !isCondition(n) ? STATUS_STYLE.failure : STATUS_STYLE.no;
}
const KIND_MARK: Record<string, string> = { sequence: '→ ', selector: '? ', parallel: '⇉ ' };

const BW = 104; const BH = 32; const GX = 12; const GY = 26;
const px = (x: number) => x * (BW + GX);
const py = (d: number) => d * (BH + GY);

/** Leaves side by side, each parent centred over its children. */
function layout(nodes: BtNode[]) {
  const byId = new Map(nodes.map((n) => [n.id, n]));
  const root = nodes.find((n) => !n.parent || !byId.has(n.parent)) as BtNode;
  const pos = new Map<string, { x: number; d: number }>();
  let leaf = 0;
  let depth = 0;
  const walk = (n: BtNode, d: number): number => {
    depth = Math.max(depth, d);
    const kids = n.children.map((c) => byId.get(c)).filter(Boolean) as BtNode[];
    let x: number;
    if (!kids.length) { x = leaf++; } else {
      const xs = kids.map((k) => walk(k, d + 1));
      x = (xs[0] + xs[xs.length - 1]) / 2;
    }
    pos.set(n.id, { x, d });
    return x;
  };
  walk(root, 0);
  // What the tree is doing now: the deepest running node (the root when nothing runs).
  let focus = root.id;
  for (const n of nodes) {
    const p = pos.get(n.id);
    if (n.status === 'running' && p && p.d >= (pos.get(focus)?.d ?? 0)) focus = n.id;
  }
  return { pos, focus, W: Math.max(leaf, 1) * (BW + GX), H: (depth + 1) * (BH + GY) };
}

function BtGraph() {
  const nodes = bt.value;
  const scroller = useRef<HTMLDivElement>(null);
  const shown = useRef<string | null>(null);
  const tree = nodes.length ? layout(nodes) : null;
  // The tree is wider than the phone: bring the active node into view when it changes,
  // not on every snapshot (the user may be scrolling).
  useEffect(() => {
    const el = scroller.current;
    const p = tree?.pos.get(tree.focus);
    if (!el || !p || !tree || shown.current === tree.focus) return;
    shown.current = tree.focus;
    el.scrollLeft = px(p.x) + BW / 2 - el.clientWidth / 2;
  });
  if (!tree) {
    return <section aria-label="Behavior tree" class="card pad"><span class="label">BEHAVIOR TREE</span>
      <p class="muted" style={{ fontSize: '13px', margin: '8px 0 0' }}>Nessuno snapshot del BT (stack spento?).</p></section>;
  }
  const { pos, W, H } = tree;
  return (
    <section aria-label="Behavior tree" class="card" style={{ padding: '12px 14px', display: 'flex', flexDirection: 'column', gap: '8px' }}>
      <span class="label">BEHAVIOR TREE</span>
      <div ref={scroller} style={{ overflowX: 'auto' }} data-testid="bt-scroll">
        <svg width={W} height={H} viewBox={`0 0 ${W} ${H}`} data-testid="bt-graph" role="img" aria-label="Behavior tree e stato dei nodi">
          {nodes.map((n) => {
            const p = pos.get(n.id);
            const q = n.parent ? pos.get(n.parent) : undefined;
            if (!p || !q) return null;
            return <path key={`e${n.id}`} d={`M${px(q.x) + BW / 2},${py(q.d) + BH} V${py(q.d) + BH + GY / 2} H${px(p.x) + BW / 2} V${py(p.d)}`}
              fill="none" stroke="#3A404B" stroke-width="1.5" />;
          })}
          {nodes.map((n) => {
            const p = pos.get(n.id);
            if (!p) return null;
            const s = styleOf(n, missionActive.value);
            const name = (KIND_MARK[n.kind] ?? '') + n.name;
            return (
              <g key={n.id}>
                <rect x={px(p.x) + 1} y={py(p.d) + 1} width={BW - 2} height={BH - 2} rx="8" fill={s.fill}
                  stroke={s.stroke} stroke-width="1.5" stroke-dasharray={s.dash} />
                <text x={px(p.x) + BW / 2} y={py(p.d) + BH / 2 + 4} text-anchor="middle" fill={s.color}
                  font-family="JetBrains Mono, monospace" font-size="10" font-weight={n.children.length ? 700 : 400}>
                  {name.length > 15 ? name.slice(0, 14) + '…' : name}</text>
              </g>);
          })}
        </svg>
      </div>
      <div style={{ display: 'flex', gap: '12px', flexWrap: 'wrap', fontSize: '11px', color: 'var(--muted)' }}>
        {Object.entries(STATUS_STYLE)
          .filter(([k]) => k !== 'failure' || missionActive.value)
          .map(([k, s]) => (
            <span key={k} style={{ display: 'flex', alignItems: 'center', gap: '5px' }}>
              <span style={{ width: '10px', height: '10px', borderRadius: '3px', border: `1.5px ${s.dash ? 'dashed' : 'solid'} ${s.stroke}`, background: s.fill }} />{STATUS_LABEL[k] ?? k}</span>))}
      </div>
    </section>
  );
}

export function Telemetria() {
  return (<><h1>Telemetria</h1><Trajectory /><SpeedChart /><BtGraph /></>);
}
