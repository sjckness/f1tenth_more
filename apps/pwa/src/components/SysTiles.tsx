// The CPU and GPU tiles of lamacchinina and Telemetria. The CPU tile opens the CPU view
// (screens/Cpu.tsx); the GPU tile has its frequency under the %, and n/a when the gateway
// has no GPU to read. Both from status.system (the gateway's sysstats.py).
import { cpuFrom, status } from '../store';
import { fmt } from './ui';

export function CpuTile() {
  const open = () => { cpuFrom.value = location.hash.replace(/^#\/?/, '') || 'cockpit'; };
  return (
    <a href="#/cpu" class="stat tap" onClick={open} data-testid="cpu-tile"
      aria-label={`CPU ${fmt(status.value?.cpu_percent, 0)} %: apri il dettaglio per core`}>
      <span class="k">CPU %<span aria-hidden="true"> ›</span></span>
      <span class="v">{fmt(status.value?.cpu_percent, 0)}</span>
    </a>
  );
}

export function GpuTile() {
  const sys = status.value?.system;
  const g = sys?.gpu;
  // No sample yet: unknown (—). A sample without a GPU reading: there is none (n/a).
  const value = !sys ? '—' : g?.percent === null || g?.percent === undefined ? 'n/a' : fmt(g.percent, 0);
  return (
    <div class="stat" data-testid="gpu-tile">
      <span class="k">GPU %</span>
      <span class="v">{value}</span>
      {g?.freq_mhz !== null && g?.freq_mhz !== undefined && <span class="s">{g.freq_mhz} MHz</span>}
    </div>
  );
}
