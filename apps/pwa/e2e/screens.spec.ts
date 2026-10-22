// Screenshots of every screen and state (WebKit, iPhone 390x844 / 844x390) into
// output/pwa_screens/, each next to its mockup from docs/pwa/design when there is one.
// Run: npm run screens (the build in dist/ is what the gateway serves: npm run build first).
import { expect, test, type Page } from '@playwright/test';
import * as fs from 'node:fs';
import * as path from 'node:path';
import { fileURLToPath } from 'node:url';
import { hold, mock, openApp, planAndStart, resetCar } from './helpers';

const HERE = path.dirname(fileURLToPath(import.meta.url));
const OUT = path.resolve(HERE, '../../../output/pwa_screens');
const DESIGN = path.resolve(HERE, '../../../docs/pwa/design');
const PORTRAIT = { width: 390, height: 844 };
const LANDSCAPE = { width: 844, height: 390 };

test.describe.configure({ mode: 'serial' });
test.beforeAll(() => fs.mkdirSync(OUT, { recursive: true }));
test.beforeEach(async ({ request }) => { await resetCar(request); });

async function shot(page: Page, name: string, { toasts = false } = {}) {
  // An event toast ("piano richiesto…") covers the screen title; wait it out unless wanted.
  if (!toasts) await expect(page.locator('.toast')).toHaveCount(0, { timeout: 10_000 });
  await page.evaluate(() => document.fonts.ready);
  await page.waitForTimeout(300);                        // let the 10 Hz telemetry land
  await page.screenshot({ path: path.join(OUT, `app_${name}.png`) });
}

/** The mockup rendered at its own size, then both side by side in cmp_<name>.png. */
async function compare(page: Page, name: string, mockup: string, size = PORTRAIT) {
  // A desktop context at the iPhone's pixel ratio: the mockups have no viewport meta, so
  // a mobile one would lay them out 980 px wide and shrink them.
  const ctx = await page.context().browser()!.newContext({ viewport: size, deviceScaleFactor: 3 });
  const m = await ctx.newPage();
  await m.goto(`file://${path.join(DESIGN, mockup)}`);
  await m.evaluate(() => document.fonts.ready);
  await m.waitForTimeout(300);
  // The artboard itself: a clipped page screenshot at 3x comes out at a third of the size
  // in WebKit.
  const mockPng = await m.locator('x-dc > div').first().screenshot();
  const appPng = fs.readFileSync(path.join(OUT, `app_${name}.png`));
  const c = await ctx.newPage();
  const wide = size.width > size.height;
  await c.setViewportSize(wide ? { width: size.width + 40, height: size.height * 2 + 100 }
    : { width: size.width * 2 + 60, height: size.height + 50 });
  const img = (b: Buffer) => `data:image/png;base64,${b.toString('base64')}`;
  const cell = (title: string, b: Buffer) => `<figure><figcaption>${title}</figcaption>` +
    `<img src="${img(b)}" width="${size.width}" height="${size.height}"></figure>`;
  await c.setContent(`<!doctype html><html><body style="margin:0;background:#1d2128;font:600 14px sans-serif;color:#ECEEF1">
    <div style="display:flex;flex-direction:${wide ? 'column' : 'row'};gap:20px;padding:10px 20px">
    ${cell(`Mockup · ${mockup}`, mockPng)}${cell(`App · ${name} (WebKit)`, appPng)}</div>
    <style>figure{margin:0;display:flex;flex-direction:column;gap:6px}img{border:1px solid #3A404B}</style>
    </body></html>`);
  await c.screenshot({ path: path.join(OUT, `cmp_${name}.png`), fullPage: true });
  await ctx.close();
}

test('pairing', async ({ page }) => {
  await page.goto('/');
  await expect(page.getByTestId('token')).toBeVisible();
  await shot(page, '01_pairing');
});

test('cockpit READY and NOT READY', async ({ page, request }) => {
  await openApp(page);
  await expect(page.getByTestId('ready-card')).toContainText('6/6');
  await shot(page, '02_cockpit_ready');
  await mock(request, 'set', { failing: ['scan', 'tf_map_base_link'] });
  await expect(page.getByTestId('ready-card')).toContainText('NOT READY');
  await shot(page, '03_cockpit_not_ready');
  await compare(page, '03_cockpit_not_ready', 'Main.dc.html');
});

test('missione: the validated plan', async ({ page }) => {
  await openApp(page, 'missione');
  await page.getByTestId('command').fill('vai dritto per due metri, gira a destra e raggiungi la persona più vicina');
  await page.getByTestId('plan').click();
  await expect(page.getByTestId('plan-card')).toContainText('VALIDATO', { timeout: 15_000 });
  await shot(page, '04_missione_piano');
  await compare(page, '04_missione_piano', 'Missione.dc.html');
});

test('missione: countdown, running, completed', async ({ page, request }) => {
  await mock(request, 'set', { countdown_s: 5.0, move_s: 8.0 });
  await openApp(page, 'missione');
  await planAndStart(page);
  const run = page.getByTestId('mission-running');
  await expect(run).toContainText('AVVIO TRA');
  await shot(page, '05_missione_countdown', { toasts: true });
  await expect(run).toContainText('IN CORSO', { timeout: 10_000 });
  await page.waitForTimeout(2500);
  await shot(page, '06_missione_in_corso');
  await compare(page, '06_missione_in_corso', 'MissioneInCorso.dc.html');
  await expect(run).toContainText('COMPLETATA', { timeout: 40_000 });
  await shot(page, '07_missione_completata');
});

test('missione: aborted', async ({ page, request }) => {
  await mock(request, 'set', { move_s: 20.0 });
  await openApp(page, 'missione');
  await planAndStart(page);
  const run = page.getByTestId('mission-running');
  await expect(run).toContainText('IN CORSO', { timeout: 10_000 });
  await hold(page, 'abort', 1300);
  await expect(run).toContainText('INTERROTTA', { timeout: 5_000 });
  await shot(page, '08_missione_interrotta');
});

test('telemetria', async ({ page, request }) => {
  await mock(request, 'set', { move_s: 20.0 });
  await openApp(page, 'missione');
  await planAndStart(page);
  await expect(page.getByTestId('mission-running')).toContainText('IN CORSO', { timeout: 10_000 });
  await page.waitForTimeout(4000);
  await page.getByRole('link', { name: 'Telemetria' }).click();
  await expect(page.getByTestId('trajectory').locator('image')).toHaveCount(1, { timeout: 10_000 });
  await page.waitForTimeout(1500);
  await shot(page, '09_telemetria');
  await compare(page, '09_telemetria', 'Telemetria.dc.html');
});

test('run', async ({ page }) => {
  await openApp(page, 'run');
  await expect(page.getByTestId('runs').locator('li')).toHaveCount(5);
  await shot(page, '10_run');
  await compare(page, '10_run', 'Run.dc.html');
});

test('guida: released, held, portrait', async ({ page }) => {
  await openApp(page, 'guida');
  await expect(page.getByTestId('rotate')).toBeVisible();
  await shot(page, '11_guida_verticale');
  await page.setViewportSize(LANDSCAPE);
  await expect(page.getByTestId('deadman')).toContainText('DEAD-MAN RILASCIATO');
  await shot(page, '12_guida_rilasciato');
  await compare(page, '12_guida_rilasciato', 'Guida.dc.html', LANDSCAPE);
  const gas = page.getByTestId('gas');
  const steer = page.getByTestId('steer');
  const g = (await gas.boundingBox())!;
  const s = (await steer.boundingBox())!;
  const at = (b: typeof g, fx: number, fy: number) => ({ clientX: b.x + b.width * fx, clientY: b.y + b.height * fy });
  await gas.dispatchEvent('pointerdown', { pointerId: 1, pointerType: 'touch', isPrimary: true, ...at(g, 0.5, 0.2) });
  await gas.dispatchEvent('pointermove', { pointerId: 1, pointerType: 'touch', isPrimary: true, ...at(g, 0.5, 0.2) });
  await steer.dispatchEvent('pointerdown', { pointerId: 2, pointerType: 'touch', ...at(s, 0.5, 0.5) });
  await steer.dispatchEvent('pointermove', { pointerId: 2, pointerType: 'touch', ...at(s, 0.75, 0.5) });
  await expect(page.getByTestId('deadman')).toContainText('DEAD-MAN ATTIVO');
  await page.waitForTimeout(1200);
  await shot(page, '13_guida_attivo');
  await steer.dispatchEvent('pointerup', { pointerId: 2, pointerType: 'touch' });
  await gas.dispatchEvent('pointerup', { pointerId: 1, pointerType: 'touch' });
});

test('guida: occupied by a joystick', async ({ page, request }) => {
  await mock(request, 'set', { teleop_others: 1 });
  await page.setViewportSize(LANDSCAPE);
  await openApp(page, 'guida');
  await expect(page.getByTestId('drive-reason')).toContainText('joystick');
  await shot(page, '14_guida_non_disponibile');
});

test('connessione persa', async ({ page, request }) => {
  await openApp(page);
  await mock(request, 'freeze', { seconds: 6 });
  await expect(page.getByTestId('lost-banner')).toBeVisible({ timeout: 3_000 });
  await page.waitForTimeout(1500);
  await shot(page, '15_connessione_persa', { toasts: true });
  await compare(page, '15_connessione_persa', 'ConnessionePersa.dc.html');
  await expect(page.getByTestId('lost-banner')).toHaveCount(0, { timeout: 10_000 });
});

test('e-stop attivo', async ({ page }) => {
  await openApp(page);
  await page.getByTestId('estop').click();
  await expect(page.getByTestId('estop-banner')).toContainText('E-STOP ATTIVO');
  await expect(page.getByTestId('estop-reset')).toBeEnabled({ timeout: 5_000 });
  await shot(page, '16_estop_attivo', { toasts: true });
});
