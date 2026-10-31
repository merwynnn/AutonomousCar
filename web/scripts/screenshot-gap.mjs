import { chromium } from 'playwright';

const browser = await chromium.launch({ headless: true });
const page = await browser.newPage({ viewport: { width: 1600, height: 1000 } });
await page.goto('http://localhost:5173');
await page.getByRole('button', { name: 'Gap', exact: true }).click();
await page.waitForTimeout(500);
await page.getByRole('button', { name: 'Reset', exact: true }).click({ timeout: 60000 });
await page.getByRole('button', { name: 'Start' }).click();
await page.waitForTimeout(6000);
await page.screenshot({ path: '../docs/assets/2026-10-30-gap-benchmark.png' });
await browser.close();
console.log('screenshot written');
