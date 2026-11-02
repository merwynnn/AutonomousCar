import { mkdir } from 'node:fs/promises';
import { chromium } from 'playwright';

const url = process.env.URL ?? 'http://localhost:8000';
const out = process.env.OUT ?? '../../docs/assets';

const browser = await chromium.launch({ args: ['--use-gl=swiftshader', '--enable-webgl'] });
const page = await browser.newPage({ viewport: { width: 1600, height: 1000 }, deviceScaleFactor: 1 });
page.on('pageerror', (err) => console.log('PAGEERROR', err.message));
await page.goto(url, { waitUntil: 'networkidle' });
await page.waitForTimeout(1000);

await page.getByRole('button', { name: 'Reset', exact: true }).click();
await page.waitForTimeout(800);
await mkdir(out, { recursive: true });
await page.screenshot({ path: `${out}/dashboard-track.png` });

await page.getByRole('button', { name: 'Run', exact: true }).click();
await page.waitForTimeout(5000);
await page.screenshot({ path: `${out}/dashboard-track-live.png` });

await page.getByRole('button', { name: 'Car only' }).click();
await page.waitForTimeout(1200);
await page.screenshot({ path: `${out}/dashboard-car-studio.png` });

await browser.close();
