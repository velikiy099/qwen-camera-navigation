import { createHash } from 'node:crypto';
import { readFile, readdir, mkdir, writeFile } from 'node:fs/promises';
import path from 'node:path';
import { fileURLToPath } from 'node:url';
import { parseArgs } from 'node:util';
import { createRequire } from 'node:module';
import MarkdownIt from 'markdown-it';
import { launchBrowser, minchoFonts } from './browser.mjs';
import { mathPlugin } from './math.mjs';

const directory = path.dirname(fileURLToPath(import.meta.url));
const require = createRequire(import.meta.url);
const { values } = parseArgs({ options: {
  source: { type: 'string' }, output: { type: 'string' },
  account: { type: 'string' }, preview: { type: 'string' },
} });
if (values.account && /[/\\\r\n]/.test(values.account)) {
  throw new Error('Account must be a filename-safe account name or ID.');
}
let source;
if (values.source) {
  source = path.resolve(values.source);
} else if (values.account) {
  source = path.join(directory, `説明資料_PAI最終課題_${values.account}.md`);
} else {
  const candidates = (await readdir(directory)).filter(name => /^説明資料_PAI最終課題.*\.md$/.test(name));
  if (candidates.length !== 1) throw new Error('Specify the Markdown source with --source.');
  source = path.join(directory, candidates[0]);
}
const output = values.output ? path.resolve(values.output) : source.replace(/\.md$/i, '') + '.pdf';
if (source === output) throw new Error('Source and output must be different files.');
const markdown = await readFile(source, 'utf8');
const md = new MarkdownIt({ html: false, typographer: false, linkify: false });
md.use(mathPlugin);
const diagrams = [];
const fence = md.renderer.rules.fence.bind(md.renderer.rules);
md.renderer.rules.fence = (tokens, index, options, env, self) => {
  if (tokens[index].info.trim() !== 'mermaid') return fence(tokens, index, options, env, self);
  diagrams.push(tokens[index].content);
  return `<div class="mermaid-diagram" data-diagram="${diagrams.length - 1}"></div>\n`;
};
const tokens = md.parse(markdown, {});
// Embed the exact referenced image bytes. No external paths enter the PDF/preview.
let imageCount = 0;
for (const token of tokens) {
  for (const child of token.children ?? []) {
    if (child.type !== 'image') continue;
    const src = child.attrGet('src');
    if (/^(https?:|data:)/i.test(src)) throw new Error('Use local image assets for reproducible report builds.');
    const imagePath = path.resolve(path.dirname(source), decodeURIComponent(src));
    const mime = { '.png': 'image/png', '.jpg': 'image/jpeg', '.jpeg': 'image/jpeg', '.svg': 'image/svg+xml' }[path.extname(imagePath).toLowerCase()];
    if (!mime) throw new Error(`Unsupported image format: ${src}`);
    child.attrSet('src', `data:${mime};base64,${(await readFile(imagePath)).toString('base64')}`);
    imageCount++;
  }
}
const body = md.renderer.render(tokens, md.options, {});
const headingIndex = tokens.findIndex(token => token.type === 'heading_open' && token.tag === 'h1');
const title = headingIndex >= 0 ? tokens[headingIndex + 1].content : path.basename(source, '.md');
const css = await readFile(path.join(directory, 'report.css'), 'utf8');
const mathDirectory = path.join(path.dirname(require.resolve('katex/package.json')), 'dist');
let mathCss = await readFile(path.join(mathDirectory, 'katex.min.css'), 'utf8');
// Embed math fonts so printing and the saved preview need no network connection.
const fontUrls = [...new Set([...mathCss.matchAll(/url\(([^)]+)\)/g)].map(match => match[1]))];
for (const url of fontUrls) {
  const filename = url.replace(/["']/g, '');
  const mime = filename.endsWith('.woff2') ? 'font/woff2' : filename.endsWith('.woff') ? 'font/woff' : 'font/ttf';
  const bytes = await readFile(path.join(mathDirectory, filename));
  mathCss = mathCss.replaceAll(`url(${url})`, `url(data:${mime};base64,${bytes.toString('base64')})`);
}
const mathCount = (body.match(/class="report-math/g) ?? []).length;
const html = `<!doctype html><html lang="ja"><head><meta charset="utf-8"><title>${md.utils.escapeHtml(title)}</title><style>${mathCss}\n:root { --report-font: ${minchoFonts}; }\n${css}</style></head><body><main>${body}</main></body></html>`;
const browser = await launchBrowser();
let diagramSizes;
try {
  const page = await browser.newPage({ viewport: { width: 794, height: 1123 }, deviceScaleFactor: 1 });
  await page.setContent(html);
  await page.addScriptTag({ path: require.resolve('mermaid/dist/mermaid.min.js') });
  diagramSizes = await page.evaluate(async ({ definitions, font }) => {
    await document.fonts.ready;
    const diagramConfig = {
      startOnLoad: false, securityLevel: 'strict', theme: 'base', fontFamily: font,
      themeVariables: {
        fontFamily: font, fontSize: '18px', primaryColor: '#f5f7fa',
        primaryTextColor: '#111111', primaryBorderColor: '#5a6570',
        lineColor: '#505860', edgeLabelBackground: '#ffffff',
      },
      flowchart: { htmlLabels: true, useMaxWidth: true, nodeSpacing: 24, rankSpacing: 28, padding: 9, wrappingWidth: 360 },
    };
    const sizes = [];
    for (const [index, definition] of definitions.entries()) {
      const hasGroups = /^\s*subgraph\b/m.test(definition);
      window.mermaid.initialize({
        ...diagramConfig, layout: hasGroups ? 'elk' : 'dagre',
        themeVariables: { ...diagramConfig.themeVariables, fontSize: hasGroups ? '28px' : '18px' },
        flowchart: { ...diagramConfig.flowchart, wrappingWidth: hasGroups ? 500 : 360 },
      });
      const { svg } = await window.mermaid.render(`report-diagram-${index}`, definition);
      const container = document.querySelector(`[data-diagram="${index}"]`);
      container.innerHTML = svg;
      const rendered = container.querySelector('svg');
      const viewBox = rendered.viewBox.baseVal;
      sizes.push({ index, width: viewBox.width, height: viewBox.height });
      // Leave room for the caption and heading, including on a full figure page.
      const maxHeight = (container.querySelectorAll('.node').length <= 6 ? 116 : 210) * 96 / 25.4;
      rendered.style.maxWidth = `${Math.min(viewBox.width, viewBox.width * maxHeight / viewBox.height)}px`;
      rendered.style.width = '100%';
      rendered.style.height = 'auto';
    }
    // Keep each diagram/image together with its existing Markdown caption.
    for (const element of [...document.querySelectorAll('.mermaid-diagram, main > p, main > table')]) {
      const isImage = element.tagName === 'P' && element.children.length === 1 && element.firstElementChild.tagName === 'IMG';
      const isImageTable = element.tagName === 'TABLE' && element.querySelector('img');
      if (!isImage && !isImageTable && !element.classList.contains('mermaid-diagram')) continue;
      const caption = element.nextElementSibling;
      const figure = document.createElement('figure');
      element.before(figure);
      figure.append(element);
      if (caption?.tagName === 'P' && /^図\d/.test(caption.textContent.trim())) {
        caption.classList.add('caption');
        figure.append(caption);
      }
    }
    await Promise.all([...document.images].map(image => image.decode()));
    await document.fonts.ready;
    return sizes;
  }, { definitions: diagrams, font: minchoFonts });
  await page.emulateMedia({ media: 'print' });
  await mkdir(path.dirname(output), { recursive: true });
  await page.pdf({
    path: output, format: 'A4', printBackground: true, preferCSSPageSize: true,
    displayHeaderFooter: true, headerTemplate: '<div></div>',
    footerTemplate: `<div style='font-family:${minchoFonts};font-size:9pt;color:#333;text-align:center;width:100%'><span class="pageNumber"></span></div>`,
    margin: { top: '17mm', right: '17mm', bottom: '18mm', left: '17mm' },
  });
  if (values.preview) {
    await page.evaluate(() => document.querySelectorAll('script').forEach(script => script.remove()));
    await mkdir(path.dirname(path.resolve(values.preview)), { recursive: true });
    await writeFile(path.resolve(values.preview), await page.content());
  }
} finally {
  await browser.close();
}
const sha = data => createHash('sha256').update(data).digest('hex');
await writeFile(path.join(path.dirname(output), 'report_build.json'), JSON.stringify({
  source: path.basename(source), source_sha256: sha(markdown),
  pdf: path.basename(output), pdf_sha256: sha(await readFile(output)),
  renderer: 'markdown-it + KaTeX + Mermaid + Playwright Chromium',
  font_family: minchoFonts, header: null, footer: 'page number only',
  mermaid_diagrams: diagramSizes, image_count: imageCount,
  latex_expressions: mathCount,
  wording: 'Read directly from Markdown; no duplicated report prose in the renderer.',
  visual_qa: 'Pending inspection of this build.',
}, null, 2) + '\n');
console.log(`Created ${path.basename(output)} (${diagrams.length} Mermaid diagrams, ${imageCount} images).`);
