import katex from 'katex';

// Keep LaTeX intact before Markdown consumes backslashes or underscores.
export function mathPlugin(md) {
  md.block.ruler.before('fence', 'math_block', (state, startLine, endLine, silent) => {
    const line = n => state.src.slice(state.bMarks[n] + state.tShift[n], state.eMarks[n]).trim();
    if (line(startLine) !== '$$') return false;
    let end = startLine + 1;
    while (end < endLine && line(end) !== '$$') end++;
    if (end === endLine) throw new Error(`Unclosed display math at line ${startLine + 1}`);
    if (silent) return true;
    const token = state.push('math_block', 'div', 0);
    token.block = true;
    token.content = state.getLines(startLine + 1, end, state.blkIndent, false).trim();
    token.map = [startLine, end + 1];
    state.line = end + 1;
    return true;
  }, { alt: ['paragraph', 'reference', 'blockquote', 'list'] });

  md.inline.ruler.before('escape', 'math_inline', (state, silent) => {
    const start = state.pos;
    if (state.src[start] !== '$' || state.src[start + 1] === '$') return false;
    let end = start + 1;
    while (end < state.posMax) {
      if (state.src[end] === '\n') return false;
      if (state.src[end] === '\\') { end += 2; continue; }
      if (state.src[end] === '$') break;
      end++;
    }
    if (end >= state.posMax || !state.src.slice(start + 1, end).trim()) return false;
    if (!silent) state.push('math_inline', 'span', 0).content = state.src.slice(start + 1, end);
    state.pos = end + 1;
    return true;
  });

  const render = (token, displayMode) => {
    const tag = displayMode ? 'div' : 'span';
    const html = katex.renderToString(token.content, {
      displayMode, throwOnError: true, trust: false, output: 'htmlAndMathml',
    });
    return `<${tag} class="report-math${displayMode ? ' math-block' : ''}" data-latex="${md.utils.escapeHtml(token.content)}">${html}</${tag}>`;
  };
  md.renderer.rules.math_inline = (tokens, index) => render(tokens[index], false);
  md.renderer.rules.math_block = (tokens, index) => render(tokens[index], true) + '\n';
}
