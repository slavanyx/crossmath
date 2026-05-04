'use strict';

const fs = require('fs');
const path = require('path');
const test = require('node:test');
const assert = require('node:assert');
const { JSDOM } = require('jsdom');

/* --------------------------------------------------------------------------
   Setup: extract the IIFE body from crossmath.html and inject an export hook.
   The single-file app wraps everything in (function(){ ... })() so its internals
   are not reachable. We replace the four init lines at the bottom with an
   assignment to globalThis.__exp, exposing the symbols we need.
-------------------------------------------------------------------------- */
const HTML_PATH = path.join(__dirname, 'crossmath.html');
const HTML = fs.readFileSync(HTML_PATH, 'utf8');

const SCRIPT_BODY = (function () {
  const m = HTML.match(/<script>([\s\S]*?)<\/script>/);
  if (!m) throw new Error('crossmath.html: <script> tag not found');
  const exposeList = [
    'generatePuzzle', 'genLayout', 'assignValues', 'pickGivens', 'pickDecoys',
    'isDeducible', 'countSolutions',
    'applyOp', 'applyChain', 'backCompute',
    'seedRng', 'dailySeed',
    'computeViewportLayoutCap', 'buildVisualGrid',
    'state', 'render', 'startPuzzle',
    'replaceArcadeEquation', 'simulateAndValidateArcadeReplacement',
    'detectEquationClears',
    'DIFFICULTY'
  ];
  const replaced = m[1].replace(
    /\/\* Init \*\/\s*loadStats\(\);\s*applyTheme\(\);\s*startPuzzle\([^)]*\);\s*registerOfflineServiceWorker\(\);/,
    `globalThis.__exp = { ${exposeList.join(', ')} };`
  );
  if (replaced === m[1]) {
    throw new Error('init block not found — check the regex against crossmath.html');
  }
  return replaced;
})();

/* --------------------------------------------------------------------------
   Stub globals (no jsdom). Used for math + generation + score/arcade tests
   where we never call render(). A few document.querySelector calls fire from
   inside setTimeouts (animations) — the stubs return safely-noopable shapes.
-------------------------------------------------------------------------- */
function makeNoopEl() {
  const el = {
    classList: { add() {}, remove() {}, toggle() {}, contains() { return false; } },
    style: { setProperty() {} },
    setAttribute() {}, getAttribute() { return null; }, removeAttribute() {},
    appendChild() {}, removeChild() {}, addEventListener() {}, remove() {},
    parentNode: null, children: [],
    getBoundingClientRect() { return { left: 0, top: 0, width: 30, height: 30 }; },
    closest() { return null; },
    querySelector() { return null; },
    querySelectorAll() { return []; },
    innerHTML: ''
  };
  return el;
}

// Node 20+ exposes a built-in `navigator` (and friends) as read-only getters on
// globalThis. Plain assignment throws — use defineProperty to override.
function setGlobal(name, value) {
  Object.defineProperty(globalThis, name, {
    value, writable: true, configurable: true, enumerable: true
  });
}

function installStubGlobals() {
  setGlobal('window', {
    innerWidth: 393, innerHeight: 852,
    addEventListener() {}, removeEventListener() {},
    AudioContext: undefined, webkitAudioContext: undefined
  });
  const noopEl = makeNoopEl();
  setGlobal('document', {
    getElementById() { return noopEl; },
    documentElement: { classList: { toggle() {} } },
    querySelector() { return null; },
    querySelectorAll() { return []; },
    addEventListener() {},
    body: noopEl,
    createElement() { return makeNoopEl(); },
    createTextNode() { return {}; }
  });
  setGlobal('navigator', { serviceWorker: undefined, clipboard: undefined });
  const data = {};
  setGlobal('localStorage', {
    getItem(k) { return Object.prototype.hasOwnProperty.call(data, k) ? data[k] : null; },
    setItem(k, v) { data[k] = String(v); },
    removeItem(k) { delete data[k]; }
  });
  setGlobal('URL', { createObjectURL() { return 'blob:mock'; } });
  setGlobal('Blob', function () {});
}

function loadStub() {
  installStubGlobals();
  delete globalThis.__exp;
  // Eval in indirect-call form to use global scope (var declarations stay scoped to the IIFE inside the body)
  (0, eval)(SCRIPT_BODY);
  return globalThis.__exp;
}

/* --------------------------------------------------------------------------
   jsdom variant: same eval trick, but with a real DOM. Used for layout-fit
   rendering tests at multiple viewport sizes.
-------------------------------------------------------------------------- */
function loadJsdom(viewportW, viewportH) {
  const dom = new JSDOM(HTML, { runScripts: 'outside-only', pretendToBeVisual: true });
  Object.defineProperty(dom.window, 'innerWidth', { value: viewportW, configurable: true });
  Object.defineProperty(dom.window, 'innerHeight', { value: viewportH, configurable: true });
  setGlobal('window', dom.window);
  setGlobal('document', dom.window.document);
  setGlobal('navigator', dom.window.navigator);
  setGlobal('localStorage', dom.window.localStorage);
  setGlobal('URL', dom.window.URL);
  setGlobal('Blob', dom.window.Blob);
  delete globalThis.__exp;
  (0, eval)(SCRIPT_BODY);
  return { exp: globalThis.__exp, dom };
}

/* --------------------------------------------------------------------------
   Helpers shared across tests
-------------------------------------------------------------------------- */
function buildBank(data) {
  const bank = [];
  for (const k of data.layout.cells) if (!data.givens[k]) bank.push(data.puzzle.values[k]);
  if (data.decoys) for (const d of data.decoys) bank.push(d);
  bank.sort((a, b) => a - b);
  return bank;
}

function eqsConnected(layout) {
  if (layout.equations.length === 0) return true;
  const adj = new Array(layout.equations.length).fill(null).map(() => new Set());
  for (let i = 0; i < layout.equations.length; i++) {
    for (let j = i + 1; j < layout.equations.length; j++) {
      const a = new Set(layout.equations[i].cells);
      for (const c of layout.equations[j].cells) {
        if (a.has(c)) { adj[i].add(j); adj[j].add(i); break; }
      }
    }
  }
  const seen = new Set([0]); const queue = [0];
  while (queue.length) {
    const v = queue.shift();
    for (const n of adj[v]) if (!seen.has(n)) { seen.add(n); queue.push(n); }
  }
  return seen.size === layout.equations.length;
}

function noLongRuns(layout) {
  const occ = new Set(layout.cells);
  for (const k of layout.cells) {
    const [r, c] = k.split(',').map(Number);
    // horizontal run starting at the leftmost cell of a sequence
    if (!occ.has(r + ',' + (c - 1))) {
      let len = 0, x = c;
      while (occ.has(r + ',' + x)) { len++; x++; }
      if (len > 3) return false;
    }
    // vertical run starting at the topmost cell of a sequence
    if (!occ.has((r - 1) + ',' + c)) {
      let len = 0, y = r;
      while (occ.has(y + ',' + c)) { len++; y++; }
      if (len > 3) return false;
    }
  }
  return true;
}

function countGivensInEq(eq, givens) {
  let g = 0;
  for (const c of eq.cells) if (givens[c]) g++;
  return g;
}

function hasBackComputeStep(layout, givens) {
  // At least one equation has 2 givens that are NOT both operands (i.e., result is given).
  for (const eq of layout.equations) {
    const g0 = !!givens[eq.cells[0]], g1 = !!givens[eq.cells[1]], g2 = !!givens[eq.cells[2]];
    const total = (g0 ? 1 : 0) + (g1 ? 1 : 0) + (g2 ? 1 : 0);
    if (total === 2 && !(g0 && g1)) return true;
  }
  return false;
}

function bankSatisfiesNeed(data) {
  const need = {};
  for (const k of data.layout.cells) {
    if (!data.givens[k]) need[data.puzzle.values[k]] = (need[data.puzzle.values[k]] || 0) + 1;
  }
  const have = {};
  for (const v of buildBank(data)) have[v] = (have[v] || 0) + 1;
  for (const v in need) if ((have[v] || 0) < need[v]) return false;
  return true;
}

/* ==========================================================================
   1. MATH PRIMITIVES
========================================================================== */
test('applyOp — all four operators', () => {
  const { applyOp } = loadStub();
  assert.equal(applyOp(3, '+', 4), 7);
  assert.equal(applyOp(10, '−', 3), 7);
  assert.equal(applyOp(4, '×', 5), 20);
  assert.equal(applyOp(20, '÷', 5), 4);
  assert.equal(applyOp(20, '÷', 6), 3, 'integer floor division');
  assert.equal(applyOp(0, '+', 0), 0);
});

test('backCompute — addition / subtraction', () => {
  const { backCompute } = loadStub();
  // 3 + 4 = 7: solve for each position
  assert.equal(backCompute('+', [undefined, 4, 7], 0), 3);
  assert.equal(backCompute('+', [3, undefined, 7], 1), 4);
  assert.equal(backCompute('+', [3, 4, undefined], 2), 7);
  // 10 − 3 = 7
  assert.equal(backCompute('−', [undefined, 3, 7], 0), 10);
  assert.equal(backCompute('−', [10, undefined, 7], 1), 3);
  assert.equal(backCompute('−', [10, 3, undefined], 2), 7);
});

test('backCompute — multiplication exact', () => {
  const { backCompute } = loadStub();
  // 4 × 5 = 20
  assert.equal(backCompute('×', [undefined, 5, 20], 0), 4);
  assert.equal(backCompute('×', [4, undefined, 20], 1), 5);
  assert.equal(backCompute('×', [4, 5, undefined], 2), 20);
});

test('backCompute — multiplication inexact returns null', () => {
  const { backCompute } = loadStub();
  // 7 × ? = 20 → not an integer; reject
  assert.equal(backCompute('×', [7, undefined, 20], 1), null);
  // ? × 7 = 20 → not an integer; reject
  assert.equal(backCompute('×', [undefined, 7, 20], 0), null);
});

test('backCompute — division exact', () => {
  const { backCompute } = loadStub();
  // 20 ÷ 5 = 4
  assert.equal(backCompute('÷', [undefined, 5, 4], 0), 20);
  assert.equal(backCompute('÷', [20, undefined, 4], 1), 5);
  assert.equal(backCompute('÷', [20, 5, undefined], 2), 4);
});

test('backCompute — division inexact returns null', () => {
  const { backCompute } = loadStub();
  // 21 ÷ 4 ≠ 5 (a/c not integer); solving for b given a=21,c=4 should fail
  assert.equal(backCompute('÷', [21, undefined, 4], 1), null);
});

test('backCompute — division by zero handled', () => {
  const { backCompute } = loadStub();
  // c = 0 with op '÷' on missingPos=1: would divide a by 0
  assert.equal(backCompute('÷', [10, undefined, 0], 1), null);
  // missingPos=2 with b=0: applyOp does Math.floor(a/0) = Infinity → not integer → null
  assert.equal(backCompute('÷', [10, 0, undefined], 2), null);
});

test('backCompute — negative or zero results rejected (cells must be ≥1)', () => {
  const { backCompute } = loadStub();
  // 3 + ? = 3 → b = 0, rejected
  assert.equal(backCompute('+', [3, undefined, 3], 1), null);
  // 5 − ? = 7 → b = -2, rejected
  assert.equal(backCompute('−', [5, undefined, 7], 1), null);
});

test('applyChain — single op (only real triplet case)', () => {
  const { applyChain } = loadStub();
  assert.equal(applyChain([3, 4], ['+']), 7);
  assert.equal(applyChain([10, 3], ['−']), 7);
  assert.equal(applyChain([4, 5], ['×']), 20);
  assert.equal(applyChain([20, 5], ['÷']), 4);
});

/* ==========================================================================
   2. GENERATION CORRECTNESS — sample 20 puzzles per difficulty
========================================================================== */
const SAMPLES_PER_DIFF = 20;

for (const diff of ['easy', 'medium', 'hard']) {
  test(`generation invariants — ${diff} (${SAMPLES_PER_DIFF} samples)`, () => {
    const exp = loadStub();
    const { generatePuzzle, isDeducible, countSolutions, applyOp, DIFFICULTY } = exp;
    const cfg = DIFFICULTY[diff];

    for (let i = 0; i < SAMPLES_PER_DIFF; i++) {
      const data = generatePuzzle(diff);
      assert.ok(data, `sample ${i}: generatePuzzle returned null`);
      const { layout, puzzle, givens, decoys } = data;

      // Triplets only
      for (const eq of layout.equations) {
        assert.equal(eq.cells.length, 3, `sample ${i}: equation has ${eq.cells.length} cells`);
      }

      // No row/col with 4+ consecutive
      assert.ok(noLongRuns(layout), `sample ${i}: row/col with 4+ consecutive cells`);

      // Each equation has 1 or 2 givens
      for (let j = 0; j < layout.equations.length; j++) {
        const g = countGivensInEq(layout.equations[j], givens);
        assert.ok(g === 1 || g === 2, `sample ${i}: eq ${j} has ${g} givens (need 1 or 2)`);
      }

      // Uniquely solvable (with decoys present)
      const sols = countSolutions(layout, puzzle, givens, 2, decoys);
      assert.equal(sols, 1, `sample ${i}: countSolutions=${sols}, want 1`);

      // Deducible
      const ded = isDeducible(layout, puzzle, givens);
      assert.ok(ded.ok, `sample ${i}: not deducible (depth=${ded.depth})`);

      // At least one back-compute step
      assert.ok(hasBackComputeStep(layout, givens), `sample ${i}: no back-compute step`);

      // Cell values: positive integers within maxNum
      for (const k of layout.cells) {
        const v = puzzle.values[k];
        assert.ok(Number.isInteger(v), `sample ${i}: ${k} value is not int (${v})`);
        assert.ok(v >= 1 && v <= cfg.maxNum, `sample ${i}: ${k}=${v} out of [1, ${cfg.maxNum}]`);
      }

      // Each equation evaluates correctly
      for (let j = 0; j < layout.equations.length; j++) {
        const eq = layout.equations[j];
        const v0 = puzzle.values[eq.cells[0]];
        const v1 = puzzle.values[eq.cells[1]];
        const v2 = puzzle.values[eq.cells[2]];
        const op = puzzle.eqOps[j][0];
        assert.equal(applyOp(v0, op, v1), v2,
          `sample ${i}: eq ${j} (${v0} ${op} ${v1}) ≠ ${v2}`);
      }

      // Connected graph
      assert.ok(eqsConnected(layout), `sample ${i}: equations not connected`);
    }
  });
}

/* ==========================================================================
   3. BANK SUPPLY — every needed value exists in the bank with sufficient count
========================================================================== */
for (const diff of ['easy', 'medium', 'hard']) {
  test(`bank supply guarantee — ${diff} (${SAMPLES_PER_DIFF} samples)`, () => {
    const { generatePuzzle } = loadStub();
    for (let i = 0; i < SAMPLES_PER_DIFF; i++) {
      const data = generatePuzzle(diff);
      assert.ok(data, `sample ${i}: null puzzle`);
      assert.ok(bankSatisfiesNeed(data), `sample ${i}: bank does not supply all needed values`);
    }
  });
}

/* ==========================================================================
   4. DAILY DETERMINISM
========================================================================== */
for (const diff of ['easy', 'medium', 'hard']) {
  test(`daily mode is deterministic — ${diff}`, () => {
    const { generatePuzzle, seedRng } = loadStub();
    const seed = 12345;
    const a = generatePuzzle(diff, seedRng(seed));
    const b = generatePuzzle(diff, seedRng(seed));
    assert.ok(a && b, 'one of the runs returned null');
    assert.deepEqual(a.layout.cells, b.layout.cells, 'layout cells differ');
    assert.deepEqual(a.layout.equations, b.layout.equations, 'equations differ');
    assert.deepEqual(a.puzzle.values, b.puzzle.values, 'values differ');
    assert.deepEqual(a.puzzle.eqOps, b.puzzle.eqOps, 'ops differ');
    assert.deepEqual(a.givens, b.givens, 'givens differ');
    assert.deepEqual(a.decoys, b.decoys, 'decoys differ');
  });
}

/* ==========================================================================
   5. MUL/DIV MODE
========================================================================== */
test('mulDiv off — only + and − appear', () => {
  const exp = loadStub();
  exp.state.settings.mulDiv = false;
  for (let i = 0; i < 10; i++) {
    const data = exp.generatePuzzle('medium');
    assert.ok(data, `sample ${i}: null`);
    for (const ops of data.puzzle.eqOps) {
      const op = ops[0];
      assert.ok(op === '+' || op === '−', `unexpected op ${op}`);
    }
  }
});

test('mulDiv on — all four operators eventually appear, and ÷ stays integer', () => {
  const exp = loadStub();
  exp.state.settings.mulDiv = true;
  const seenOps = new Set();
  for (let i = 0; i < 25; i++) {
    const data = exp.generatePuzzle('medium');
    assert.ok(data, `sample ${i}: null`);
    for (let j = 0; j < data.puzzle.eqOps.length; j++) {
      const op = data.puzzle.eqOps[j][0];
      seenOps.add(op);
      // Verify ÷ is exact integer
      if (op === '÷') {
        const eq = data.layout.equations[j];
        const a = data.puzzle.values[eq.cells[0]];
        const b = data.puzzle.values[eq.cells[1]];
        const c = data.puzzle.values[eq.cells[2]];
        assert.equal(a % b, 0, `sample ${i} eq ${j}: ${a} ÷ ${b} not exact`);
        assert.equal(a / b, c, `sample ${i} eq ${j}: ${a} ÷ ${b} = ${a / b} ≠ ${c}`);
      }
    }
  }
  for (const op of ['+', '−', '×', '÷']) {
    assert.ok(seenOps.has(op), `op ${op} never appeared across 25 medium puzzles`);
  }
});

/* ==========================================================================
   6. SCORE-MODE BEHAVIOR
========================================================================== */
test('score mode — wrong-but-arithmetic placements do NOT mark cleared', () => {
  const exp = loadStub();
  // Hand-craft a one-equation scenario: 10 + 2 = 12
  exp.state.settings.scoreMode = true;
  exp.state.data = {
    layout: { cells: ['0,0', '0,1', '0,2'], equations: [{ cells: ['0,0', '0,1', '0,2'], orientation: 'h' }] },
    puzzle: { values: { '0,0': 10, '0,1': 2, '0,2': 12 }, eqOps: [['+']] },
    givens: { '0,2': true },
    decoys: []
  };
  exp.state.placed = { '0,0': 7, '0,1': 5 }; // 7+5=12 (math works) but not the intended 10+2
  exp.state.clearedEqs = {};
  exp.state.settledCells = {};
  exp.state.combo = 0; exp.state.multiplier = 1; exp.state.comboExpiry = 0;
  exp.state.score = 0; exp.state.arcadeEqsSolved = 0;

  exp.detectEquationClears('0,1');
  assert.equal(Object.keys(exp.state.clearedEqs).length, 0,
    'arithmetically valid but wrong values should NOT clear the equation');
  assert.equal(exp.state.score, 0, 'no score awarded');
});

test('score mode — correct values DO mark cleared and award points', () => {
  const exp = loadStub();
  exp.state.settings.scoreMode = true;
  exp.state.data = {
    layout: { cells: ['0,0', '0,1', '0,2'], equations: [{ cells: ['0,0', '0,1', '0,2'], orientation: 'h' }] },
    puzzle: { values: { '0,0': 10, '0,1': 2, '0,2': 12 }, eqOps: [['+']] },
    givens: { '0,2': true },
    decoys: []
  };
  exp.state.placed = { '0,0': 10, '0,1': 2 };
  exp.state.clearedEqs = {};
  exp.state.settledCells = {};
  exp.state.combo = 0; exp.state.multiplier = 1; exp.state.comboExpiry = 0;
  exp.state.score = 0; exp.state.arcadeEqsSolved = 0;

  exp.detectEquationClears('0,1');
  assert.equal(exp.state.clearedEqs[0], true, 'should be cleared');
  assert.equal(exp.state.score, 100, 'base 100 × multiplier 1 = 100');
  assert.equal(exp.state.arcadeEqsSolved, 1, 'arcadeEqsSolved increments');

  // detectEquationClears scheduled a setTimeout(render, 900); after the test
  // ends, that render would access state.visual (null) and throw. Null out
  // state.data so the deferred render hits the early-exit branch.
  exp.state.data = null;
});

/* ==========================================================================
   7. ARCADE REPLACEMENT — 50-move loop stays consistent
========================================================================== */
test('arcade replacement loop — 50 moves keep state deducible & consistent', () => {
  const exp = loadStub();
  const { generatePuzzle, isDeducible, replaceArcadeEquation, state } = exp;
  state.settings.scoreMode = true;
  state.settings.arcadeMode = true;

  const data = generatePuzzle('easy');
  assert.ok(data, 'initial generation failed');
  state.data = data;
  state.bank = buildBank(data);
  state.placed = {};
  state.clearedEqs = {};
  state.settledCells = {};
  state.arcadeEqsSolved = 0;

  let replacementsAttempted = 0;
  for (let move = 0; move < 50; move++) {
    // Find an active, uncleared equation
    let eqIdx = -1;
    for (let i = 0; i < data.layout.equations.length; i++) {
      const eq = data.layout.equations[i];
      if (!eq.cells || eq.cells.length === 0) continue;
      if (state.clearedEqs[i]) continue;
      eqIdx = i; break;
    }
    if (eqIdx === -1) break;

    // Fill non-given cells in this equation with their correct values
    const eq = data.layout.equations[eqIdx];
    for (const k of eq.cells) {
      if (!data.givens[k] && state.placed[k] == null) {
        state.placed[k] = data.puzzle.values[k];
      }
    }

    // Mark cleared & invoke replacement (mirrors what detectEquationClears would do)
    state.clearedEqs[eqIdx] = true;
    replaceArcadeEquation(eqIdx);
    replacementsAttempted++;

    // After replacement, verify deducibility from the current "known" state
    // (originals givens + currently placed cells are treated as known).
    const knownGivens = {};
    for (const k in data.givens) if (data.givens[k]) knownGivens[k] = true;
    for (const k in state.placed) if (state.placed[k] != null) knownGivens[k] = true;
    const activeEqs = [];
    const activeOps = [];
    for (let i = 0; i < data.layout.equations.length; i++) {
      const e = data.layout.equations[i];
      if (state.clearedEqs[i]) continue;
      if (!e.cells || e.cells.length === 0) continue;
      activeEqs.push(e);
      activeOps.push(data.puzzle.eqOps[i]);
    }
    if (activeEqs.length === 0) continue; // nothing left to verify

    const snapLayout = { cells: data.layout.cells, equations: activeEqs };
    const snapPuzzle = { values: data.puzzle.values, eqOps: activeOps };
    const ded = isDeducible(snapLayout, snapPuzzle, knownGivens);
    assert.ok(ded.ok,
      `move ${move}: state not deducible after replacement (depth=${ded.depth})`);

    // Bank-supply: all required (non-known) values must be available
    const need = {};
    for (const k of data.layout.cells) {
      if (!knownGivens[k]) need[data.puzzle.values[k]] = (need[data.puzzle.values[k]] || 0) + 1;
    }
    const have = {};
    for (const v of state.bank) have[v] = (have[v] || 0) + 1;
    // Subtract what's already placed (they consumed bank tiles)
    for (const k in state.placed) {
      if (state.placed[k] != null && have[state.placed[k]]) have[state.placed[k]]--;
    }
    for (const v in need) {
      assert.ok((have[v] || 0) >= need[v],
        `move ${move}: bank short of value ${v} (need ${need[v]}, have ${have[v] || 0})`);
    }
  }

  assert.ok(replacementsAttempted >= 5,
    `expected ≥5 replacement attempts in 50 moves, got ${replacementsAttempted}`);
});

/* ==========================================================================
   8. JSDOM RENDERING — grid fits within multiple viewports
========================================================================== */
const VIEWPORTS = [
  { w: 360, h: 640, label: '360×640' },
  { w: 375, h: 667, label: '375×667' },
  { w: 393, h: 852, label: '393×852' },
  { w: 768, h: 1024, label: '768×1024' }
];

function parseGridDims(grid) {
  // Grid inline style: gridTemplateColumns: 'repeat(N, Mpx)'
  const cols = grid.style.gridTemplateColumns.match(/repeat\((\d+),\s*(\d+(?:\.\d+)?)px\)/);
  const rows = grid.style.gridTemplateRows.match(/repeat\((\d+),\s*(\d+(?:\.\d+)?)px\)/);
  if (!cols || !rows) throw new Error('grid template parse failed: ' + grid.style.gridTemplateColumns + ' / ' + grid.style.gridTemplateRows);
  return {
    cols: +cols[1], colSize: +cols[2],
    rows: +rows[1], rowSize: +rows[2]
  };
}

for (const vp of VIEWPORTS) {
  for (const diff of ['easy', 'medium', 'hard']) {
    test(`render fits viewport ${vp.label} — ${diff}`, () => {
      const { exp } = loadJsdom(vp.w, vp.h);
      const data = exp.generatePuzzle(diff);
      assert.ok(data, 'puzzle generation failed');
      exp.state.data = data;
      exp.state.visual = exp.buildVisualGrid(data.layout, data.puzzle);
      exp.state.bank = buildBank(data);
      exp.state.placed = {};
      exp.state.generating = false;
      exp.state.history = [];
      exp.state.elapsed = 0; exp.state.startTime = Date.now();
      exp.state.clearedEqs = {}; exp.state.settledCells = {}; exp.state.arcadeIncomingCells = {};
      exp.render();

      const grid = globalThis.document.querySelector('.grid');
      assert.ok(grid, 'no .grid rendered');
      const dims = parseGridDims(grid);
      const totalW = dims.cols * dims.colSize;
      const totalH = dims.rows * dims.rowSize;

      assert.ok(totalW <= vp.w,
        `${vp.label} ${diff}: grid width ${totalW}px > viewport ${vp.w}px`);
      // Vertical overflow is harder to bound exactly (bank reserve estimate is rough),
      // but cells should never be smaller than the floor.
      assert.ok(dims.colSize >= 22,
        `${vp.label} ${diff}: cell size ${dims.colSize}px < 22px floor`);
      assert.equal(dims.colSize, dims.rowSize,
        `${vp.label} ${diff}: cells not square (${dims.colSize}×${dims.rowSize})`);
      // Sanity: vertical shouldn't massively overflow viewport either
      assert.ok(totalH <= vp.h,
        `${vp.label} ${diff}: grid height ${totalH}px > viewport ${vp.h}px`);
    });
  }
}

test('bank wraps via flex-wrap (no horizontal scroll)', () => {
  const { exp } = loadJsdom(360, 640);
  const data = exp.generatePuzzle('hard');
  assert.ok(data, 'hard generation failed');
  exp.state.data = data;
  exp.state.visual = exp.buildVisualGrid(data.layout, data.puzzle);
  exp.state.bank = buildBank(data);
  exp.state.placed = {};
  exp.state.generating = false; exp.state.history = [];
  exp.state.elapsed = 0; exp.state.startTime = Date.now();
  exp.state.clearedEqs = {}; exp.state.settledCells = {}; exp.state.arcadeIncomingCells = {};
  exp.render();

  // Verify CSS rule for .bank-wrap includes flex-wrap: wrap
  // (jsdom doesn't compute styles, so we check the source CSS in the loaded HTML)
  assert.match(HTML, /\.bank-wrap\s*\{[^}]*flex-wrap:\s*wrap/,
    'bank-wrap CSS missing flex-wrap: wrap');
  const bank = globalThis.document.querySelector('.bank-wrap');
  assert.ok(bank, 'no .bank-wrap rendered');
});

test('build marker — meta tag matches build footer', () => {
  const { exp } = loadJsdom(393, 852);
  const data = exp.generatePuzzle('easy');
  exp.state.data = data;
  exp.state.visual = exp.buildVisualGrid(data.layout, data.puzzle);
  exp.state.bank = buildBank(data);
  exp.state.placed = {};
  exp.state.generating = false; exp.state.history = [];
  exp.state.elapsed = 0; exp.state.startTime = Date.now();
  exp.state.clearedEqs = {}; exp.state.settledCells = {}; exp.state.arcadeIncomingCells = {};
  exp.render();

  const meta = globalThis.document.querySelector('meta[name="build"]');
  const footer = globalThis.document.querySelector('.build-footer');
  assert.ok(meta, 'meta[name="build"] not found');
  assert.ok(footer, '.build-footer not rendered');
  const metaVal = meta.getAttribute('content');
  const footerVal = footer.textContent.trim();
  assert.equal(metaVal, footerVal,
    `meta build "${metaVal}" ≠ footer "${footerVal}"`);
});
