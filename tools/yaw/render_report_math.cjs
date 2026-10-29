#!/usr/bin/env node
/* Pre-render TeX to self-contained SVG. The delivered report needs no JS library. */
const fs = require('node:fs');
const path = require('node:path');
const root = path.resolve(process.argv[2] || path.join(__dirname, 'node_modules/mathjax-full'));
const {mathjax} = require(path.join(root, 'js/mathjax.js'));
const {TeX} = require(path.join(root, 'js/input/tex.js'));
const {SVG} = require(path.join(root, 'js/output/svg.js'));
const {liteAdaptor} = require(path.join(root, 'js/adaptors/liteAdaptor.js'));
const {RegisterHTMLHandler} = require(path.join(root, 'js/handlers/html.js'));
const {AllPackages} = require(path.join(root, 'js/input/tex/AllPackages.js'));
const adaptor = liteAdaptor();
RegisterHTMLHandler(adaptor);
const document = mathjax.document('', {
  InputJax: new TeX({packages: AllPackages}),
  OutputJax: new SVG({fontCache: 'none'}),
});
const expressions = JSON.parse(fs.readFileSync(0, 'utf8'));
const rendered = expressions.map(tex => {
  const result = adaptor.outerHTML(document.convert(tex, {display: true}));
  if (result.includes('data-mjx-error') || result.includes('data-mml-node="merror"')) {
    throw new Error(`Invalid report formula: ${tex}`);
  }
  return result;
});
process.stdout.write(JSON.stringify(rendered));
