import { chromium } from 'playwright';

const browser = await chromium.launch({ headless: true });
const page = await browser.newPage({ viewport: { width: 1600, height: 1000 } });
await page.goto('http://localhost:5173');
await page.getByRole('button', { name: 'Pursuit' }).click();
await page.getByRole('button', { name: 'Reset' }).click();
await page.getByRole('button', { name: 'Start' }).click();
await page.waitForTimeout(12000);
await page.screenshot({ path: '../docs/assets/2026-10-30-m3-slam.png' });
await browser.close();
console.log('screenshot written');
