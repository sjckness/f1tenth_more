import { defineConfig, devices } from '@playwright/test';

// The PWA against the gateway's pretend car (f1tenth_gateway --mock): no ROS, no car.
// F1T_GATEWAY_PYTHON: a Python with the gateway's requirements (README, "Test").
const PORT = 8790;
export const TOKEN = 'e2e-token-' + 'x'.repeat(40);
const python = process.env.F1T_GATEWAY_PYTHON ?? 'python3';
const iphone = devices['iPhone 13'];

export default defineConfig({
  testDir: './e2e',
  timeout: 60_000,
  workers: 1,
  fullyParallel: false,
  reporter: [['list']],
  use: { baseURL: `http://127.0.0.1:${PORT}`, trace: 'off' },
  webServer: {
    command: `cd ../../src/f1tenth_gateway && PYTHONPATH=.:../f1tenth_ctl:../f1tenth_telegram ` +
      `F1T_GATEWAY_TOKEN=${TOKEN} ${python} -m f1tenth_gateway --mock ` +
      `--config config/gateway_mock.yaml --port ${PORT}`,
    url: `http://127.0.0.1:${PORT}/`,
    reuseExistingServer: false,
    timeout: 30_000,
  },
  projects: [
    // iPhone in portrait (390x844) and landscape (844x390), WebKit: Safari's engine.
    { name: 'webkit-portrait', testIgnore: /guida\.spec|multitouch|screens/,
      use: { ...iphone, browserName: 'webkit', viewport: { width: 390, height: 844 } } },
    { name: 'webkit-landscape', testMatch: /guida/,
      use: { ...iphone, browserName: 'webkit', viewport: { width: 844, height: 390 } } },
    // Two real simultaneous touches need CDP: Chromium only, same iPhone viewport.
    { name: 'chromium-multitouch', testMatch: /multitouch/,
      use: { ...iphone, browserName: 'chromium', viewport: { width: 844, height: 390 } } },
    { name: 'screens', testMatch: /screens/,
      use: { ...iphone, browserName: 'webkit', viewport: { width: 390, height: 844 } } },
  ],
});
