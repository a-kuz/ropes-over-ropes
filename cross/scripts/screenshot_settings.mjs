import { chromium } from 'playwright';

async function main() {
  const browser = await chromium.launch({
    headless: false,
    channel: 'chrome',
    args: ['--enable-unsafe-webgpu', '--enable-features=Vulkan,UseSkiaRenderer', '--use-angle=metal'],
  });

  const context = await browser.newContext({
    viewport: { width: 900, height: 1000 },
    deviceScaleFactor: 2,
  });
  const page = await context.newPage();

  await page.goto('http://localhost:8004', { waitUntil: 'domcontentloaded', timeout: 15000 });
  await page.waitForTimeout(8000);

  // Click settings gear button (rightmost button in top bar)
  await page.click('canvas', { position: { x: 855, y: 34 } });
  await page.waitForTimeout(1000);

  await page.screenshot({ path: 'screenshots/settings.png' });
  console.log('Settings screenshot saved');

  await browser.close();
}

main().catch(e => { console.error(e); process.exit(1); });
