import { chromium } from 'playwright';

const browser = await chromium.launch({ headless: false });
const page = await browser.newPage();

const errors = [];
page.on('console', msg => {
    const type = msg.type();
    const text = msg.text();
    if (type === 'error') errors.push(text);
    console.log(`[${type}] ${text}`);
});
page.on('pageerror', err => {
    errors.push(err.message);
    console.log(`[pageerror] ${err.message}`);
});

await page.goto('http://localhost:8004');
console.log('Page loaded, waiting 5s for WebGPU init...');
await page.waitForTimeout(5000);

const screenshot = await page.screenshot();
const fs = await import('fs');
fs.writeFileSync('cross/web/screenshot.png', screenshot);
console.log('Screenshot saved to cross/web/screenshot.png');

const canvas = await page.$('canvas');
if (canvas) {
    const box = await canvas.boundingBox();
    console.log(`Canvas size: ${box.width}x${box.height}`);
} else {
    console.log('No canvas found!');
}

const jsErrors = errors.filter(e => !e.includes('favicon'));
if (jsErrors.length > 0) {
    console.log(`\nErrors found:`);
    jsErrors.forEach(e => console.log(`  - ${e}`));
} else {
    console.log('\nNo JS errors!');
}

await page.waitForTimeout(2000);
await browser.close();
