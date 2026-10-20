import { expect, type APIRequestContext, type Page } from '@playwright/test';
import { TOKEN } from '../playwright.config';

export const AUTH = { Authorization: `Bearer ${TOKEN}` };

export async function mock(request: APIRequestContext, op: string, body: object = {}) {
  const r = await request.post(`/api/_mock/${op}`, { headers: AUTH, data: body });
  expect(r.ok(), `mock ${op}`).toBeTruthy();
  return r.json();
}

/** A fresh, READY car; mission timing short so the tests are quick. */
export async function resetCar(request: APIRequestContext) {
  await mock(request, 'reset');
  await mock(request, 'set', { countdown_s: 1.5, move_s: 2.0 });
}

/** Open the app paired (token in localStorage), on a route, connected. */
export async function openApp(page: Page, route = 'cockpit') {
  await page.addInitScript((t) => localStorage.setItem('f1t-token', t), TOKEN);
  await page.goto(`/#/${route}`);
  await expect(page.locator('.pill', { hasText: /ONLINE|READY/ })
    .or(page.getByTestId('guida')).or(page.getByTestId('rotate')))
    .toBeVisible({ timeout: 10_000 });
}

export async function published(request: APIRequestContext) {
  const s = await mock(request, 'state');
  return s.published as [number, string, number, number][];
}

/** Drag the slide-to-start thumb to the end, the way a finger does. */
export async function slide(page: Page, fraction = 1.0) {
  const thumb = page.getByTestId('slide-thumb');
  const track = page.getByTestId('slide-to-start');
  const tb = (await thumb.boundingBox())!;
  const kb = (await track.boundingBox())!;
  const x0 = tb.x + tb.width / 2;
  const y = tb.y + tb.height / 2;
  const x1 = kb.x + 6 + (kb.width - 68) * fraction + tb.width / 2;
  await thumb.dispatchEvent('pointerdown', { pointerId: 7, clientX: x0, clientY: y, isPrimary: true, pointerType: 'touch' });
  for (let i = 1; i <= 10; i++) {
    await thumb.dispatchEvent('pointermove', { pointerId: 7, clientX: x0 + ((x1 - x0) * i) / 10, clientY: y, isPrimary: true, pointerType: 'touch' });
  }
  await thumb.dispatchEvent('pointerup', { pointerId: 7, clientX: x1, clientY: y, isPrimary: true, pointerType: 'touch' });
}

/** Press and hold a HoldButton for `ms`. */
export async function hold(page: Page, testid: string, ms: number) {
  const el = page.getByTestId(testid);
  await el.dispatchEvent('pointerdown', { pointerId: 9, isPrimary: true, pointerType: 'touch' });
  await page.waitForTimeout(ms);
  // The button may be gone already: a completed hold changes the screen.
  await el.dispatchEvent('pointerup', { pointerId: 9, isPrimary: true, pointerType: 'touch' },
    { timeout: 500 }).catch(() => undefined);
}

export async function planAndStart(page: Page, text = 'vai dritto per due metri e fermati') {
  await page.getByTestId('command').fill(text);
  await page.getByTestId('plan').click();
  await expect(page.getByTestId('plan-card')).toContainText('VALIDATO', { timeout: 15_000 });
  await slide(page);
}
