import { chromium } from 'playwright';

const URL = process.argv[2] || 'http://localhost:8004';
const SCREENSHOT_PATH = process.argv[3] || 'screenshots/current.png';
const WAIT_MS = parseInt(process.argv[4] || '6000', 10);
const HEADLESS = process.argv.includes('--headless');
const CLICK_HD = process.argv.includes('--hd');

async function main() {
  const browser = await chromium.launch({
    headless: HEADLESS,
    channel: 'chrome',
    args: ['--enable-unsafe-webgpu', '--enable-features=Vulkan,UseSkiaRenderer', '--use-angle=metal'],
  });

  const context = await browser.newContext({
    viewport: { width: 800, height: 1000 },
    deviceScaleFactor: 2,
  });
  const page = await context.newPage();
  page.on('console', msg => {
    if (msg.type() === 'error') console.log(`[PAGE ERROR] ${msg.text()}`);
  });

  console.log(`Opening ${URL}...`);
  await page.goto(URL, { waitUntil: 'domcontentloaded', timeout: 15000 });
  await page.waitForTimeout(WAIT_MS);

  if (CLICK_HD) {
    console.log('Clicking SD->HD...');
    await page.click('text=SD', { timeout: 2000 }).catch(() => {});
    await page.waitForTimeout(2000);
  }

  await page.screenshot({ path: SCREENSHOT_PATH, fullPage: false });
  console.log(`Screenshot saved to ${SCREENSHOT_PATH}`);
  await browser.close();
}

main().catch(e => { console.error(e); process.exit(1); });
