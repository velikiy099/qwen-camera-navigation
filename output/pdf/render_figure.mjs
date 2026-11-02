import { readFile } from 'node:fs/promises';
import path from 'node:path';
import { launchBrowser } from './browser.mjs';

const [source, output] = process.argv.slice(2);
if (!source || !output) throw new Error('Usage: node render_figure.mjs source.svg output.png');
const svg = await readFile(source, 'utf8');
const browser = await launchBrowser();
try {
  const page = await browser.newPage({ viewport: { width: 1020, height: 560 }, deviceScaleFactor: 2.5 });
  await page.setContent(`<html lang="ja"><head><style>body { margin: 0; } svg { display: block; }</style></head><body>${svg.replace(/<\?xml[\s\S]*?\?>|<!DOCTYPE[\s\S]*?>/g, '')}</body></html>`);
  await page.evaluate(() => document.fonts.ready);
  await page.locator('svg').screenshot({ path: path.resolve(output) });
} finally {
  await browser.close();
}
