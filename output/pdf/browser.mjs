import { chromium } from 'playwright';

// Resolve an installed Mincho font on each host; do not depend on a font file path.
export const minchoFonts = '"Noto Serif CJK JP", "Noto Serif JP", "Hiragino Mincho ProN", "Yu Mincho", "YuMincho", "MS PMincho", serif';

export async function launchBrowser() {
  return chromium.launch({
    headless: true,
    ...(process.env.REPORT_BROWSER_EXECUTABLE
      ? { executablePath: process.env.REPORT_BROWSER_EXECUTABLE }
      : {}),
  });
}
