// REST to the gateway: Bearer token on every call, a session cookie for media links.
import { token } from './store';

export class ApiError extends Error {
  constructor(public status: number, message: string) { super(message); }
}

export async function api<T = any>(path: string, init: RequestInit = {}): Promise<T> {
  const headers = new Headers(init.headers);
  if (token.value) headers.set('Authorization', `Bearer ${token.value}`);
  if (init.body && typeof init.body === 'string') headers.set('Content-Type', 'application/json');
  let res: Response;
  try {
    res = await fetch(path, { ...init, headers, credentials: 'same-origin' });
  } catch {
    throw new ApiError(0, 'gateway non raggiungibile');
  }
  const text = await res.text();
  let body: any = null;
  try { body = text ? JSON.parse(text) : null; } catch { body = text; }
  if (!res.ok) {
    const detail = (body && typeof body === 'object' && body.detail) || res.statusText;
    throw new ApiError(res.status, String(detail));
  }
  return body as T;
}

export const post = <T = any>(path: string, body?: unknown) =>
  api<T>(path, { method: 'POST', body: body === undefined ? undefined : JSON.stringify(body) });

export interface Job<R = any> { id: string; kind: string; state: 'running' | 'done' | 'failed';
  result?: R }

/** Follow a job (202 answers) until it ends; never retries what it started. */
export async function waitJob<R = any>(job: Job<R>, timeoutMs = 90_000): Promise<Job<R>> {
  const end = Date.now() + timeoutMs;
  let j = job;
  while (j.state === 'running' && Date.now() < end) {
    await new Promise((r) => setTimeout(r, 400));
    j = await api<Job<R>>(`/api/jobs/${job.id}`);
  }
  return j;
}

/** The session cookie: lets <video> and report links through without a header. */
export async function openSession(): Promise<void> {
  await post('/api/session');
}
