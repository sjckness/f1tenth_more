// The one WebSocket: auth first, ping every 500 ms, "connection lost" after 1.5 s of silence,
// reconnect with backoff. Everything the gateway pushes lands in the store.
import {
  bt, conn, drive, hello, lastMsgAt, mapVersion, path, pushSpeed, rttMs, setMockMode, status, tel,
  toast, token,
} from './store';

let ws: WebSocket | null = null;
let pingTimer: number | undefined;
let watchTimer: number | undefined;
let retryTimer: number | undefined;
let backoff = 500;
let pingId = 0;
const sentAt = new Map<number, number>();

function lostAfterMs(): number {
  return (hello.value?.client.lost_after_s ?? 1.5) * 1000;
}

export function send(msg: object): boolean {
  if (ws && ws.readyState === WebSocket.OPEN) {
    ws.send(JSON.stringify(msg));
    return true;
  }
  return false;
}

function onMessage(ev: MessageEvent): void {
  let m: any;
  try { m = JSON.parse(ev.data); } catch { return; }
  lastMsgAt.value = performance.now();
  if (conn.value === 'lost') conn.value = 'online';
  switch (m.type) {
    case 'hello':
      hello.value = m;
      setMockMode(m.mock === true);
      conn.value = 'online';
      backoff = 500;
      break;
    case 'status': status.value = m; break;
    case 'telemetry':
      tel.value = m;
      pushSpeed(m.t, m.v_meas ?? null, m.v_cmd ?? null);
      break;
    case 'bt': bt.value = m.nodes; break;
    case 'path': path.value = { points: m.points, horizon: m.horizon }; break;
    case 'map': mapVersion.value = m.version; break;
    case 'drive':
      drive.value = { state: m.state, reason: m.reason };
      if (m.state === 'refused' || m.state === 'busy' || m.state === 'revoked') {
        toast('warn', `Guida: ${m.reason}`);
      }
      break;
    case 'pong': {
      const t0 = sentAt.get(m.id);
      if (t0 !== undefined) {
        rttMs.value = Math.round(performance.now() - t0);
        sentAt.delete(m.id);
      }
      break;
    }
    case 'event': toast(m.level === 'info' ? 'info' : m.level === 'warn' ? 'warn' : 'error', m.text);
      break;
    default: break;
  }
}

export function connect(): void {
  if (!token.value) return;
  clearTimeout(retryTimer);
  if (ws && (ws.readyState === WebSocket.OPEN || ws.readyState === WebSocket.CONNECTING)) return;
  conn.value = conn.value === 'online' ? 'lost' : 'connecting';
  const scheme = location.protocol === 'https:' ? 'wss' : 'ws';
  const sock = new WebSocket(`${scheme}://${location.host}/ws`);
  ws = sock;
  sock.onopen = () => sock.send(JSON.stringify({ type: 'auth', token: token.value }));
  sock.onmessage = onMessage;
  sock.onclose = (ev) => {
    if (ws !== sock) return;
    ws = null;
    if (ev.code === 4401) {
      conn.value = 'unauthorized';
      return;
    }
    if (conn.value === 'online') conn.value = 'lost';
    retryTimer = window.setTimeout(connect, backoff);
    backoff = Math.min(backoff * 2, 4000);
  };
  clearInterval(pingTimer);
  pingTimer = window.setInterval(() => {
    const id = ++pingId;
    sentAt.set(id, performance.now());
    if (sentAt.size > 20) sentAt.delete(sentAt.keys().next().value as number);
    send({ type: 'ping', id, t: Date.now() / 1000 });
  }, (hello.value?.client.ping_s ?? 0.5) * 1000);
  clearInterval(watchTimer);
  watchTimer = window.setInterval(() => {
    const silent = performance.now() - lastMsgAt.value;
    if (conn.value === 'online' && silent > lostAfterMs()) conn.value = 'lost';
    // Long silence on an open socket: start again (iOS keeps dead sockets open).
    if (silent > 3 * lostAfterMs() && ws && ws.readyState === WebSocket.OPEN) ws.close();
  }, 250);
}

export function disconnect(): void {
  clearInterval(pingTimer);
  clearInterval(watchTimer);
  clearTimeout(retryTimer);
  const sock = ws;
  ws = null;
  sock?.close();
  conn.value = 'idle';
}
