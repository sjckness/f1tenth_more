// The app's state: everything the gateway says, as signals the screens read.
import { computed, signal } from '@preact/signals';

export type Conn = 'idle' | 'connecting' | 'online' | 'lost' | 'unauthorized';

export interface Check {
  name: string; ok: boolean; value: unknown; want: string; required: boolean; why?: string | null;
}
export interface Proc {
  state: string; pending?: string; uptime_s?: number; nodes?: number | null; managed?: boolean;
}
export interface MissionView {
  state: string; id?: string | null; mission_id?: string | null; n_moves?: number | null;
  moves_done?: number; move_index?: number | null; move_id?: string | null;
  elapsed_s?: number; countdown_left_s?: number; started_by_app?: boolean;
  estop_latched?: boolean;
}
export interface Status {
  ok: boolean; error?: string | null; ready: boolean; failing: string[]; checks: Check[];
  checks_ok: number; checks_total: number; estop_latched: boolean;
  processes: Record<string, Proc>; mission: MissionView;
  battery?: { v: number; ok: boolean } | null; vesc_temp_c?: number | null;
  cpu_percent?: number | null; drive: { busy: boolean; blockers: string[] };
  reset_allowed: { ok: boolean; why: string | null };
  transcription: { state: string; reason?: string | null };
  estop?: { available: boolean; why: string | null };
}
export interface Telemetry {
  t: number; pose?: { x: number; y: number; yaw: number } | null;
  v_meas?: number | null; v_cmd?: number | null; steer_cmd?: number | null;
  cmd_source?: string | null; safety?: { active: boolean; source: string } | null;
  mission: MissionView; battery?: { v: number; ok: boolean } | null; estop_latched: boolean;
  drive: { holder: string | null; armed: boolean; tripped: boolean; limit_mps: number;
           speed_cmd: number; steer_cmd: number };
  reset_allowed: { ok: boolean; why: string | null };
}
export interface Hello {
  client_id: string;
  teleop: { min_speed_mps: number; max_speed_mps: number; default_limit_mps: number;
            limit_step_mps: number; deadzone: number; max_steer_rad: number; watchdog_s: number };
  mission: { auto_abort_s: number };
  client: { lost_after_s: number; ping_s: number };
  mock: boolean;
}
export interface BtNode { id: string; parent: string | null; children: string[]; name: string;
  kind: string; status: string }
export interface Toast { id: number; level: 'info' | 'warn' | 'error'; text: string }

const TOKEN_KEY = 'f1t-token';

function loadToken(): string | null {
  try { return localStorage.getItem(TOKEN_KEY); } catch { return null; }
}

export const token = signal<string | null>(loadToken());
export function setToken(value: string | null): void {
  token.value = value;
  try {
    if (value) localStorage.setItem(TOKEN_KEY, value); else localStorage.removeItem(TOKEN_KEY);
  } catch { /* private mode: the token lives for this session only */ }
}

// The gateway runs the pretend car (--mock). Kept across reloads, so the badge shows
// while connecting or lost too; every hello sets it again.
const MOCK_KEY = 'f1t-mock';
export const mockMode = signal<boolean>((() => {
  try { return localStorage.getItem(MOCK_KEY) === '1'; } catch { return false; }
})());
export function setMockMode(value: boolean): void {
  mockMode.value = value;
  try {
    if (value) localStorage.setItem(MOCK_KEY, '1'); else localStorage.removeItem(MOCK_KEY);
  } catch { /* private mode */ }
}

/**
 * The token out of a pairing link's fragment (`#token=...`), or null.
 *
 * The FRAGMENT, never the query string: a fragment is not sent with the
 * request, so the token stays out of the gateway's log, out of tailscale's and
 * out of any proxy in between. The QR code printed on the Orin
 * (src/f1tenth_gateway/tools/pairing_qr.py) encodes exactly this link.
 *
 * Same floor as the ABBINA button and the gateway's auth.min_token_len: under
 * 32 characters is not a token, so the link is ignored and the pairing screen
 * asks for one by hand.
 */
export function tokenFromHash(hash: string): string | null {
  const m = /^#\/?token=(.*)$/.exec(hash);
  if (!m) return null;
  let value = m[1];
  try { value = decodeURIComponent(value); } catch { /* not encoded: take it as it came */ }
  value = value.trim();
  return value.length >= 32 ? value : null;
}

/**
 * Read a pairing link and clear it from the address bar, at module load.
 *
 * The fragment goes whether or not it carried a usable token: a link left in
 * the address bar ends up in a screenshot, in the back stack and in whatever
 * the next person to pick up the phone sees. replaceState rather than
 * assigning location.hash, which would push a history entry and fire
 * hashchange.
 */
function pairFromLink(): void {
  let hash = '';
  try { hash = location.hash; } catch { return; }
  if (!/^#\/?token=/.test(hash)) return;
  const value = tokenFromHash(hash);
  try {
    history.replaceState(null, '', location.pathname + location.search);
  } catch { /* no history API: the token still pairs, the link lingers */ }
  if (value) setToken(value);
}

// Before the first render, so App() already sees a paired car: a scanned link
// wins over whatever this phone had stored.
pairFromLink();

export const conn = signal<Conn>('idle');
export const rttMs = signal<number | null>(null);
export const lastMsgAt = signal<number>(0);
export const hello = signal<Hello | null>(null);
export const status = signal<Status | null>(null);
export const tel = signal<Telemetry | null>(null);
export const bt = signal<BtNode[]>([]);
export const path = signal<{ points: [number, number][]; horizon: [number, number][] }>(
  { points: [], horizon: [] });
export const mapVersion = signal<number | null>(null);
export const speedHist = signal<{ t: number; v: number | null; cmd: number | null }[]>([]);
export const drive = signal<{ state: string; reason?: string | null }>({ state: 'idle' });
export const toasts = signal<Toast[]>([]);
export const wakeLockMissing = signal(false);

export const online = computed(() => conn.value === 'online');

/**
 * Whether the gateway answers at all, over either connection.
 *
 * The WebSocket is not the measure: REST is a separate connection and keeps
 * working when the socket is down (ws.ts polls /api/status then). This is
 * what EMERGENZA goes by, so it is driven by events rather than by a clock:
 * any message from the gateway sets it, a failed poll clears it.
 */
export const gatewayUp = signal(true);

/**
 * Whether an E-STOP would reach the car, and why not.
 *
 * NOT a function of the WebSocket: /api/estop is REST, and the gateway runs
 * the e-stop itself with f1tenth_ctl as its fallback. Only two things take
 * the button away -- the gateway not answering, or the gateway reporting that
 * it cannot reach /mission/emergency_stop or /safety_stop.
 *
 * With no report yet it fails OPEN: an older gateway sends no status.estop,
 * and a button that works beats a truthful label.
 */
export const estopReach = computed<{ ok: boolean; why: string | null }>(() => {
  if (!gatewayUp.value) return { ok: false, why: 'gateway non raggiungibile' };
  const e = status.value?.estop;
  if (!e) return { ok: true, why: null };
  return e.available ? { ok: true, why: null } : { ok: false, why: e.why ?? null };
});
export const latched = computed(() =>
  tel.value?.estop_latched ?? status.value?.estop_latched ?? false);
export const mission = computed<MissionView>(() =>
  tel.value?.mission ?? status.value?.mission ?? { state: 'unknown' });
export const missionActive = computed(() =>
  ['starting', 'running', 'holding'].includes(mission.value.state));
export const battery = computed(() => tel.value?.battery ?? status.value?.battery ?? null);

let toastId = 0;
export function toast(level: Toast['level'], text: string, ms = 4000): void {
  const t = { id: ++toastId, level, text };
  toasts.value = [...toasts.value.slice(-3), t];
  setTimeout(() => { toasts.value = toasts.value.filter((x) => x.id !== t.id); }, ms);
}

export function pushSpeed(t: number, v: number | null, cmd: number | null): void {
  const keep = speedHist.value.filter((s) => t - s.t <= 30);
  keep.push({ t, v, cmd });
  speedHist.value = keep;
}
