// Two thumbs at once, as real touches (CDP Input.dispatchTouchEvent: Chromium only).
import { expect, test } from '@playwright/test';
import { mock, openApp, published, resetCar } from './helpers';

test('gas and steering together: speed and steering in the same commands', async ({ page, request }) => {
  await resetCar(request);
  await openApp(page, 'guida');
  await expect(page.getByTestId('deadman')).toContainText('DEAD-MAN RILASCIATO');
  const gas = (await page.getByTestId('gas').boundingBox())!;
  const steer = (await page.getByTestId('steer').boundingBox())!;
  const cdp = await page.context().newCDPSession(page);
  const g = { x: gas.x + gas.width / 2, y: gas.y + gas.height * 0.08, id: 1 };
  const s0 = { x: steer.x + steer.width / 2, y: steer.y + steer.height / 2, id: 2 };
  const s1 = { ...s0, x: steer.x + steer.width * 0.95 };   // full right
  await cdp.send('Input.dispatchTouchEvent', { type: 'touchStart', touchPoints: [g] });
  await cdp.send('Input.dispatchTouchEvent', { type: 'touchStart', touchPoints: [g, s0] });
  for (let i = 1; i <= 10; i++) {
    const s = { ...s0, x: s0.x + ((s1.x - s0.x) * i) / 10 };
    await cdp.send('Input.dispatchTouchEvent', { type: 'touchMove', touchPoints: [g, s] });
    await page.waitForTimeout(40);
  }
  await expect(page.getByTestId('deadman')).toContainText('DEAD-MAN ATTIVO');
  await page.waitForTimeout(600);
  const pub = (await published(request)).filter((p) => p[1] === '/teleop');
  const both = pub.filter((p) => p[2] >= 0.4 && p[3] < -0.05);       // right = negative (ROS)
  expect(both.length).toBeGreaterThan(3);
  expect(Math.min(...pub.map((p) => p[3]))).toBeGreaterThanOrEqual(-0.30 - 1e-9);
  // Lift only the steering thumb (Chromium's touchEnd lifts the points it lists, all of
  // them when the list is empty): still driving, steering back to 0.
  await cdp.send('Input.dispatchTouchEvent', { type: 'touchEnd', touchPoints: [s1] });
  await expect(page.getByTestId('deadman')).toContainText('DEAD-MAN ATTIVO');
  await page.waitForTimeout(300);
  const last = (await published(request)).filter((p) => p[1] === '/teleop').at(-1)!;
  expect(last[2]).toBeGreaterThanOrEqual(0.4);
  expect(Math.abs(last[3])).toBeLessThan(0.05);   // back to 0 (rate limit 1.5 rad/s: 0.2 s)
  // Lift the gas: zero.
  await cdp.send('Input.dispatchTouchEvent', { type: 'touchEnd', touchPoints: [] });
  await expect(page.getByTestId('deadman')).toContainText('DEAD-MAN RILASCIATO');
  const st = await mock(request, 'state');
  expect(st.teleop.armed).toBe(false);
});
