// Guida in landscape (iPhone 844x390, WebKit): the dead-man, the immediate zero, the
// watchdog when the app goes away, one driver at a time.
import { expect, test, type Page } from '@playwright/test';
import { mock, openApp, published, resetCar } from './helpers';

test.beforeEach(async ({ request }) => { await resetCar(request); });

async function press(page: Page, testid: string, id: number, fx: number, fy: number) {
  const el = page.getByTestId(testid);
  const b = (await el.boundingBox())!;
  const at = { clientX: b.x + b.width * fx, clientY: b.y + b.height * fy };
  await el.dispatchEvent('pointerdown', { pointerId: id, pointerType: 'touch', isPrimary: id === 1, ...at });
  await el.dispatchEvent('pointermove', { pointerId: id, pointerType: 'touch', isPrimary: id === 1, ...at });
  return at;
}

async function lift(page: Page, testid: string, id: number) {
  await page.getByTestId(testid).dispatchEvent('pointerup', { pointerId: id, pointerType: 'touch' });
}

test('gas held: the car gets 0.4..limit on /teleop; released: a zero at once', async ({ page, request }) => {
  await openApp(page, 'guida');
  await expect(page.getByTestId('deadman')).toContainText('DEAD-MAN RILASCIATO');
  const t0 = Date.now();
  await press(page, 'gas', 1, 0.5, 0.05);                  // full gas
  await expect(page.getByTestId('deadman')).toContainText('DEAD-MAN ATTIVO');
  await page.waitForTimeout(600);
  let pub = (await published(request)).filter((p) => p[1] === '/teleop');
  const moving = pub.filter((p) => p[2] > 0);
  expect(moving.length).toBeGreaterThan(5);
  expect(Math.min(...moving.map((p) => p[2]))).toBeGreaterThanOrEqual(0.4);
  expect(Math.max(...moving.map((p) => p[2]))).toBeLessThanOrEqual(0.6 + 1e-9);   // default limit
  const before = pub.length;
  await lift(page, 'gas', 1);
  await page.waitForTimeout(150);
  pub = (await published(request)).filter((p) => p[1] === '/teleop');
  expect(pub.slice(before).some((p) => p[2] === 0)).toBeTruthy();
  await expect(page.getByTestId('deadman')).toContainText('DEAD-MAN RILASCIATO');
  expect(Date.now() - t0).toBeLessThan(10_000);
});

test('gas pulled back: reverse down to -0.4 with RETROMARCIA shown; released: a zero at once', async ({ page, request }) => {
  await openApp(page, 'guida');
  await expect(page.getByTestId('reverse')).toBeHidden();
  await press(page, 'gas', 1, 0.5, 0.95);                  // full reverse
  await expect(page.getByTestId('reverse')).toBeVisible();
  await expect(page.getByTestId('reverse')).toContainText('RETROMARCIA');
  await page.waitForTimeout(800);
  let pub = (await published(request)).filter((p) => p[1] === '/teleop');
  const back = pub.filter((p) => p[2] < 0);
  expect(back.length).toBeGreaterThan(5);
  expect(Math.min(...back.map((p) => p[2]))).toBeGreaterThanOrEqual(-0.4 - 1e-9);   // max_reverse_speed_mps
  expect(pub.some((p) => p[2] > 0)).toBeFalsy();
  const before = pub.length;
  await lift(page, 'gas', 1);
  await page.waitForTimeout(150);
  pub = (await published(request)).filter((p) => p[1] === '/teleop');
  expect(pub.slice(before).some((p) => p[2] === 0)).toBeTruthy();
  await expect(page.getByTestId('reverse')).toBeHidden();
});

test('the limit stepper is clamped to 0.4..1.0 and the server enforces it', async ({ page, request }) => {
  await openApp(page, 'guida');
  for (let i = 0; i < 10; i++) await page.getByTestId('limit-up').click();
  await expect(page.getByTestId('limit')).toHaveText('1.0 m/s');
  for (let i = 0; i < 10; i++) await page.getByTestId('limit-down').click();
  await expect(page.getByTestId('limit')).toHaveText('0.4 m/s');
  await page.getByTestId('limit-up').click();
  await press(page, 'gas', 1, 0.5, 0.05);
  await page.waitForTimeout(500);
  await lift(page, 'gas', 1);
  const moving = (await published(request)).filter((p) => p[1] === '/teleop' && p[2] > 0);
  expect(Math.max(...moving.map((p) => p[2]))).toBeLessThanOrEqual(0.5 + 1e-9);
});

test('closing the app while driving: the watchdog stops the car', async ({ page, request }) => {
  await openApp(page, 'guida');
  await press(page, 'gas', 1, 0.5, 0.05);
  await page.waitForTimeout(400);
  const before = (await published(request)).length;
  await page.close();                                       // no release message: just gone
  await new Promise((r) => setTimeout(r, 800));
  const after = (await published(request)).slice(before).filter((p) => p[1] === '/teleop');
  expect(after.some((p) => p[2] === 0)).toBeTruthy();
  const st = await mock(request, 'state');
  expect(st.teleop.holder).toBeNull();
});

test('a second phone gets "guida occupata"', async ({ page, browser }) => {
  await openApp(page, 'guida');
  await expect(page.getByTestId('deadman')).toBeVisible();
  const ctx = await browser.newContext({ viewport: { width: 844, height: 390 }, hasTouch: true, isMobile: true });
  const other = await ctx.newPage();
  await openApp(other, 'guida');
  await expect(other.getByTestId('drive-reason')).toContainText('occupata');
  await ctx.close();
});

test('E-STOP while driving revokes the guida', async ({ page }) => {
  await openApp(page, 'guida');
  await expect(page.getByTestId('deadman')).toBeVisible();
  await page.getByTestId('estop').click();
  await expect(page.getByTestId('drive-reason')).toBeVisible({ timeout: 5_000 });
});

test('no Guida during a mission or with a joystick on /teleop', async ({ page, request }) => {
  await mock(request, 'set', { teleop_others: 1 });
  await openApp(page, 'guida');
  await expect(page.getByTestId('drive-reason')).toContainText('joystick');
});

test('no Screen Wake Lock: Guida says the screen may go off', async ({ page }) => {
  await page.addInitScript(() => Object.defineProperty(Navigator.prototype, 'wakeLock',
    { get: () => undefined, configurable: true }));
  await openApp(page, 'guida');
  await expect(page.getByTestId('wake-warning')).toContainText('schermo');
});

test('with the Wake Lock, no warning', async ({ page }) => {
  await openApp(page, 'guida');
  await expect(page.getByTestId('deadman')).toBeVisible();
  await expect(page.getByTestId('wake-warning')).toHaveCount(0);
});

test('MOCK badge in Guida too', async ({ page }) => {
  await openApp(page, 'guida');
  await expect(page.getByTestId('deadman')).toBeVisible();
  await expect(page.getByTestId('mock-badge')).toHaveText('MOCK');
});
