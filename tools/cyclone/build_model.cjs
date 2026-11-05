#!/usr/bin/env node
/* Build Markdown + TeX as offline HTML. No runtime CDN/fonts/JavaScript. */
const fs = require('node:fs');
const path = require('node:path');
const {spawnSync} = require('node:child_process');
const root = path.resolve(__dirname, '../..');
const dependencies = path.resolve(process.argv[2] || path.join(__dirname, 'node_modules'));
const {marked} = require(path.join(dependencies, 'marked'));
const source = path.join(root, 'docs/CYCLONE_ROTOR_MODEL.md');
const output = path.join(root, 'docs/reports/cyclone-rotor-model.html');
const expressions = [];
let markdown = fs.readFileSync(source, 'utf8').replace(/\\\[\s*([\s\S]*?)\\\]/g, (_, tex) => {
  const id = expressions.push(tex.trim()) - 1;
  return `\n<div class="equation" data-equation="${id}">MATH_${id}</div>\n`;
});
const render = spawnSync(process.execPath, [path.join(root, 'tools/yaw/render_report_math.cjs'),
  path.join(dependencies, 'mathjax-full')], {
  input: JSON.stringify(expressions), encoding: 'utf8', maxBuffer: 32 * 1024 * 1024,
});
if (render.status !== 0) throw new Error(render.stderr || 'Math renderer failed');
const svg = JSON.parse(render.stdout);
if (svg.length !== expressions.length || !svg.length) throw new Error('Formula count mismatch');
const headings = [];
const renderer = new marked.Renderer();
renderer.heading = function ({tokens, depth}) {
  const text = this.parser.parseInline(tokens);
  const id = `section-${headings.length}`;
  if (depth === 2) headings.push({id, text});
  else headings.push({id, text, hidden: true});
  return `<h${depth} id="${id}">${text}</h${depth}>\n`;
};
let body = marked.parse(markdown, {renderer});
body = body.replace(/MATH_(\d+)/g, (_, id) => svg[Number(id)]);
body = body.replaceAll('href="reports/cyclone-rotor-model.html"', 'href="cyclone-rotor-model.html"');
if (body.includes('MATH_') || body.includes('data-mjx-error')) throw new Error('Unrendered formula');
const toc = headings.filter(h => !h.hidden).map(h => `<a href="#${h.id}">${h.text}</a>`).join('');
const html = `<!doctype html>
<html lang="zh-CN"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>普通与低速自旋三旋翼 · 动力学与控制</title>
<style>
:root{color-scheme:light;--ink:#172b3b;--accent:#006c78;--line:#d7e3e7}
*{box-sizing:border-box}body{margin:0;background:#f2f6f7;color:var(--ink);font:16px/1.85 -apple-system,BlinkMacSystemFont,"PingFang SC","Microsoft YaHei",sans-serif}
.bar{background:#123240;color:#eff8fa;padding:18px max(24px,calc((100% - 1140px)/2));font-size:14px;letter-spacing:.04em}
.layout{display:grid;grid-template-columns:228px minmax(0,900px);gap:28px;max-width:1180px;margin:28px auto;padding:0 20px}
nav{align-self:start;position:sticky;top:20px;max-height:94vh;overflow:auto;font-size:13px}nav a{display:block;padding:6px 10px;border-left:2px solid var(--line);text-decoration:none}
main{background:#fff;padding:30px 38px;border:1px solid var(--line);border-radius:8px;min-width:0}
h1{font-size:30px;line-height:1.5;margin:0 0 20px}h2{font-size:23px;border-top:2px solid var(--line);padding-top:28px;margin-top:38px;line-height:1.5}h3{font-size:18px;color:#14535c;line-height:1.6;margin-top:26px}
p{margin:14px 0}a{color:var(--accent)}code{font:13px/1.7 ui-monospace,SFMono-Regular,monospace;background:#edf3f5;padding:2px 4px;border-radius:3px;overflow-wrap:anywhere}
table{border-collapse:collapse;width:100%;font-size:14px;margin:18px 0}th,td{padding:9px 11px;border:1px solid var(--line);vertical-align:top;overflow-wrap:anywhere}th{background:#eaf2f4;text-align:left}
.equation{overflow-x:auto;overflow-y:hidden;padding:14px 8px;margin:20px 0;background:#f7fafb;border-left:3px solid #76b9bf;text-align:center;font-size:96%}mjx-container{display:inline-block;max-width:none}mjx-container svg{max-width:none}strong{font-weight:650}
footer{margin-top:36px;padding-top:14px;border-top:1px solid var(--line);font-size:13px;color:#526b78}
@media(max-width:900px){.layout{display:block;padding:0 12px;margin:12px auto}nav{position:static;max-height:none;display:flex;overflow-x:auto;gap:12px;margin:0 0 14px;white-space:nowrap}main{padding:22px 18px}h1{font-size:25px}}
@media print{body{background:white;font-size:11pt}.bar,nav{display:none}.layout{display:block;margin:0;padding:0}main{border:0;padding:0}h2,h3{break-after:avoid}table,.equation{break-inside:avoid}.equation{overflow:visible;font-size:75%}a{color:inherit}}
</style></head><body><div class="bar">dRehmFlight · Cyclone Rotor / 工程模型与调试准备 · 2026-11-05</div>
<div class="layout"><nav aria-label="目录">${toc}</nav><main>${body}
<footer>由 <a href="../CYCLONE_ROTOR_MODEL.md">Markdown 源文档</a>构建。公式已在构建时转换为SVG，断网可读；本文没有本版实机飞行结果。</footer>
</main></div></body></html>\n`;
fs.writeFileSync(output, html);
process.stdout.write(JSON.stringify({output, equations: svg.length, bytes: Buffer.byteLength(html)}) + '\n');
