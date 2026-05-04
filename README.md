# Crossmath

A math crossword puzzle. Solve interlocking arithmetic equations laid out crossword-style. Plays offline. No ads. No tracking.

**[Play it →](https://slavanyx.github.io/crossmath/)**

## What it is

Each puzzle is a procedurally-generated crossword of triplet equations like `12 + 7 = 19`. Some cells are pre-filled clues; others you fill in by dragging numbers from a bank at the bottom. Equations cross at shared cells, so a number you place affects both its row and column equations.

Every puzzle is verified to be:
- **Uniquely solvable** — there's exactly one valid arrangement
- **Solvable by deduction** — you never have to guess; logic always finds the next step
- **Bank-complete** — every needed number is available in the tile bank

## Modes

- **Easy / Medium / Hard** — puzzle size, clue density, number range, and operator mix scale up
- **Daily** — same puzzle for everyone each day; deterministic from the date
- **Score Mode** — equations "settle" in place when correctly solved; combo multipliers (×2 to ×5) for chained clears
- **Arcade Mode** — endless play; solved equations collapse and new ones slide in to take their place
- **Multiply & divide** — toggle on for ×/÷ operators (off by default; division is integer-only)
- **Dark mode**

## Features

- Drag-and-drop with snap-to-nearest tolerance — even imprecise drops land cleanly
- Hint button highlights the correct number for the selected cell
- Undo, reset, clear-wrong-cells
- Live equation feedback (green when correct, red when wrong) as you place
- Streak tracker (🔥 days in a row solving)
- Personal best times per difficulty
- Sound effects (toggleable)
- Fully offline after first load (PWA service worker)

## Tech

Pure HTML/CSS/JavaScript. No frameworks, no bundlers, no build step. Works in any modern browser. The single `crossmath.html` file is fully self-contained — fonts as system, icons as data URIs, manifest inline.

~88KB total. Loads instantly.

## Running locally

```bash
# Open directly in any browser
open crossmath.html

# Or serve from any static host
python3 -m http.server 8000
# → http://localhost:8000/crossmath.html
```

## Running tests

```bash
npm install
npm test
```

Tests verify generation invariants, math primitives, daily determinism, mode behaviors, and rendering across phone viewports using jsdom. Run on every push via GitHub Actions.

## Installing as an app

Open the live site in mobile Safari (iOS) or Chrome (Android), then:
- **iOS**: Share → Add to Home Screen
- **Android**: ⋮ menu → Install app / Add to Home screen

Get a real app icon, full-screen launch, and offline play.

## Project layout

```
crossmath.html          # single-file build (production)
crossmath/              # split version
├── index.html
├── style.css
└── app.js
test.js                 # node --test suite (jsdom-based)
package.json            # dev deps only (jsdom)
.github/workflows/      # CI
```

## License

MIT — do whatever, just don't blame me if your puzzle doesn't fit on screen.
