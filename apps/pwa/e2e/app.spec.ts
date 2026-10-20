// The PWA in portrait (iPhone 390x844, WebKit) against the pretend car.
import { expect, test } from '@playwright/test';
import { TOKEN } from '../playwright.config';
import { AUTH, hold, mock, openApp, planAndStart, resetCar, slide } from './helpers';

test.beforeEach(async ({ request }) => { await resetCar(request); });

test('pairing: a wrong token is refused, the right one connects', async ({ page }) => {
  await page.goto('/');
  await page.getByTestId('token').fill('w'.repeat(40));
  await page.getByTestId('pair').click();
  await expect(page.getByText('Token errato')).toBeVisible();
  await page.getByTestId('token').fill(TOKEN);
  await page.getByTestId('pair').click();
  await expect(page.locator('.pill', { hasText: 'ONLINE' })).toBeVisible();
  await expect(page.getByRole('heading', { name: 'Cockpit' })).toBeVisible();
});

test('cockpit: READY, then NOT READY with only the failing checks', async ({ page, request }) => {
  await openApp(page);
  const card = page.getByTestId('ready-card');
  await expect(card).toContainText('READY');
  await expect(card).toContainText('6/6');
  await mock(request, 'set', { failing: ['scan', 'tf_map_base_link'] });
  await expect(card).toContainText('NOT READY');
  await expect(card).toContainText('4/6');
  await expect(card).toContainText('LiDAR /scan');
  await expect(card).toContainText('Localizzazione');
  await expect(card).not.toContainText('Batteria');
  await expect(page.getByTestId('proc-stack')).toContainText('63 nodi');
});

test('a process switch shows its transition and cannot be pressed meanwhile', async ({ page }) => {
  await openApp(page);
  const sw = page.getByRole('switch', { name: 'Attiva logger' });
  await expect(sw).toHaveAttribute('aria-checked', 'false');
  await sw.click();
  await expect(page.getByTestId('proc-logger')).toContainText('in avvio…');
  await expect(sw).toBeDisabled();
  await expect(sw).toHaveAttribute('aria-checked', 'true', { timeout: 10_000 });
  await expect(page.getByTestId('proc-logger')).toContainText('running');
});

test('mission: plan, a tap is not a start, slide to start, steps, ABORT on a hold', async ({ page, request }) => {
  await openApp(page, 'missione');
  await mock(request, 'set', { move_s: 20.0 });
  await page.getByTestId('command').fill('vai dritto per due metri e fermati');
  await page.getByTestId('plan').click();
  const card = page.getByTestId('plan-card');
  await expect(card).toContainText('VALIDATO', { timeout: 15_000 });
  await expect(card).toContainText('3');                  // moves
  await expect(card).toContainText('0.5');                // v max
  await card.getByRole('button', { name: 'Mostra JSON' }).click();
  await expect(card).toContainText('"straight_1"');
  // A tap on the thumb, and a half slide, start nothing.
  await page.getByTestId('slide-thumb').click();
  await slide(page, 0.5);
  await expect(page.getByTestId('mission-running')).toHaveCount(0);
  await slide(page);
  const run = page.getByTestId('mission-running');
  await expect(run).toContainText('AVVIO TRA');
  await expect(run).toContainText('IN CORSO', { timeout: 10_000 });
  await expect(run).toContainText('1/3');
  // A short press is not an abort.
  await hold(page, 'abort', 300);
  await expect(run).toContainText('IN CORSO');
  await hold(page, 'abort', 1300);
  await expect(run).toContainText('INTERROTTA', { timeout: 5_000 });
  const st = await mock(request, 'state');
  expect(st.ros_state).toBe('ABORTED');
  expect(st.estop_latched).toBe(false);                   // abort is not the e-stop
});

test('mission: the steps advance and it completes', async ({ page }) => {
  await openApp(page, 'missione');
  await planAndStart(page);
  const run = page.getByTestId('mission-running');
  await expect(run).toContainText('2/3', { timeout: 10_000 });
  await expect(run).toContainText('COMPLETATA', { timeout: 15_000 });
  await page.getByTestId('new-command').click();
  await expect(page.getByTestId('command')).toBeVisible();
});

test('NOT READY: the slide is off and says why', async ({ page, request }) => {
  await openApp(page, 'missione');
  await mock(request, 'set', { failing: ['scan'] });
  await page.getByTestId('command').fill('vai dritto');
  await page.getByTestId('plan').click();
  await expect(page.getByTestId('plan-card')).toContainText('VALIDATO', { timeout: 15_000 });
  await expect(page.getByTestId('start-why')).toHaveText('NOT READY');
  await slide(page);
  await expect(page.getByTestId('mission-running')).toHaveCount(0);
});

test('EMERGENZA: banner, missions blocked, unlock only when still', async ({ page, request }) => {
  await openApp(page, 'missione');
  await page.getByTestId('command').fill('vai dritto');
  await page.getByTestId('plan').click();
  await expect(page.getByTestId('plan-card')).toContainText('VALIDATO', { timeout: 15_000 });
  await page.getByTestId('estop').click();
  const banner = page.getByTestId('estop-banner');
  await expect(banner).toContainText('E-STOP ATTIVO');
  const pub = (await mock(request, 'state')).published as [number, string][];
  expect(pub.some((p) => p[1] === '/safety_stop')).toBeTruthy();
  await expect(page.getByTestId('start-why')).toHaveText('E-STOP attivo');
  // Unlock: a 2 s hold, enabled with the car still for 1 s.
  await expect(page.getByTestId('estop-reset')).toBeEnabled({ timeout: 5_000 });
  await hold(page, 'estop-reset', 2300);
  await expect(banner).toHaveCount(0, { timeout: 10_000 });
});

test('EMERGENZA reset is disabled while the car moves', async ({ page, request }) => {
  await openApp(page);
  await page.getByTestId('estop').click();
  await expect(page.getByTestId('estop-banner')).toBeVisible();
  await mock(request, 'set', { v: 0.6 });
  // The mock decelerates the car; keep it moving for the check.
  await mock(request, 'set', { v: 0.6 });
  await expect(page.getByTestId('estop-reset')).toBeDisabled();
});

test('connection lost: red banner, commands off, EMERGENZA unreachable, then back', async ({ page, request }) => {
  await openApp(page);
  await mock(request, 'freeze', { seconds: 4 });
  await expect(page.getByTestId('lost-banner')).toBeVisible({ timeout: 3_000 });
  await expect(page.getByText('EMERGENZA NON RAGGIUNGIBILE')).toBeVisible();
  await expect(page.getByRole('switch', { name: 'Attiva stack' })).toBeDisabled();
  await expect(page.getByTestId('ready-card')).toContainText('— — —');
  // Tabs still work.
  await page.getByRole('link', { name: 'Run' }).click();
  await expect(page.getByRole('heading', { name: 'Run' })).toBeVisible();
  await expect(page.getByTestId('lost-banner')).toHaveCount(0, { timeout: 8_000 });
  await expect(page.getByTestId('estop')).toBeVisible();
});

test('run tab: outcomes, report link, video only when there is one', async ({ page }) => {
  await openApp(page, 'run');
  const runs = page.getByTestId('runs');
  await expect(runs.locator('li')).toHaveCount(5);
  await expect(runs).toContainText('OK');
  await expect(runs).toContainText('ABORT');
  await expect(runs).toContainText('E-STOP');
  await expect(runs).toContainText('INCOMPLETA');
  await expect(runs).toContainText('mission_render');
  const report = runs.getByRole('link', { name: 'Report' }).first();
  await expect(report).toHaveAttribute('href', /\/api\/runs\/.+\/report$/);
  await expect(runs.getByRole('link', { name: 'Video' })).toHaveCount(1);
});

test('telemetry: map, path and pose after driving; the BT', async ({ page }) => {
  await openApp(page, 'missione');
  await planAndStart(page);
  await expect(page.getByTestId('mission-running')).toContainText('IN CORSO', { timeout: 10_000 });
  await page.waitForTimeout(2500);
  await page.getByRole('link', { name: 'Telemetria' }).click();
  const svg = page.getByTestId('trajectory');
  await expect(svg.locator('image')).toHaveCount(1, { timeout: 10_000 });
  await expect(svg.locator('polyline').first()).toBeVisible();
  await expect(page.getByTestId('bt-graph').locator('rect')).not.toHaveCount(0);
  await expect(page.getByTestId('speed-chart').locator('polyline')).toHaveCount(2);
});

test('no Screen Wake Lock: a warning while a mission runs', async ({ page, request }) => {
  await page.addInitScript(() => Object.defineProperty(Navigator.prototype, 'wakeLock',
    { get: () => undefined, configurable: true }));
  await mock(request, 'set', { move_s: 20.0 });
  await openApp(page, 'missione');
  await expect(page.getByText('SCHERMO')).toHaveCount(0);
  await planAndStart(page);
  await expect(page.getByText('Blocco schermo non impedibile')).toBeVisible({ timeout: 10_000 });
});

test('Guida in portrait asks to rotate the phone', async ({ page }) => {
  await openApp(page, 'guida');
  await expect(page.getByTestId('rotate')).toContainText('Ruota il telefono');
});

test('MOCK badge: on every screen with the pretend car, also when the link is lost', async ({ page, request }) => {
  await openApp(page);
  await expect(page.getByTestId('mock-badge')).toHaveText('MOCK');
  await page.getByRole('link', { name: 'Missione' }).click();
  await expect(page.getByTestId('mock-badge')).toBeVisible();
  await page.getByRole('link', { name: 'Telemetria' }).click();
  await expect(page.getByTestId('mock-badge')).toBeVisible();
  await mock(request, 'freeze', { seconds: 4 });
  await expect(page.getByTestId('lost-banner').getByTestId('mock-badge')).toBeVisible({ timeout: 3_000 });
  // A reload while the gateway is silent: still MOCK (remembered), before any hello.
  await page.reload();
  await expect(page.getByTestId('lost-banner').getByTestId('mock-badge')).toBeVisible();
});

test('API: nothing without the token', async ({ request }) => {
  expect((await request.get('/api/status')).status()).toBe(401);
  expect((await request.get('/api/status', { headers: AUTH })).status()).toBe(200);
});
