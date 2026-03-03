import { chromium } from 'playwright';

const browser = await chromium.launch({
  headless: false,
  channel: 'chrome',
  args: ['--enable-unsafe-webgpu'],
});
const context = await browser.newContext();
const page = await context.newPage();

const messages = [];
page.on('console', msg => {
  messages.push(`[${msg.type()}] ${msg.text()}`);
});
page.on('pageerror', err => {
  messages.push(`[pageerror] ${err.message}`);
});

await page.goto('http://localhost:8004', { waitUntil: 'domcontentloaded' });
await page.waitForTimeout(6000);

for (const m of messages) {
  console.log(m);
}

await browser.close();
