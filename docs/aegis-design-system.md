# Aegis Kinetic UI — Design System

This document describes the Aegis design system used by the Open Omnibot dashboard: design tokens, layout patterns, and how to extend the UI.

## What Changed

The UI was consolidated from two separate HTML pages (`index.html` + `calibration.html`) into a single-page app with sidebar navigation, following the "Aegis Kinetic" design spec from Google Stitch.

### New Files

| File | Purpose |
|------|---------|
| `server/src/web/public/aegis.css` | Full Aegis Kinetic design system — color tokens, typography (Space Grotesk + JetBrains Mono), shell layout, component classes, responsive breakpoints |
| `server/src/web/public/app.js` | SPA controller — single WebSocket connection, event bus (`App.on/emit`), view routing, E-STOP (unified keyboard + motor cleanup), `get_info` polling, connection status |
| `server/src/web/public/dashboard.js` | Dashboard view — joystick, keyboard WASD+QE, map with layer toggles, orientation compass, sensor displays, tier-based conditional rendering, demo mode, UWB/Robot IP modals |
| `server/src/web/public/calibration.js` | Calibration view — motor cards (4x PWM sliders + run/stop), uniformity test, IMU diagnostics + traffic light cal, motor calibration (auto-cal/save/load/manual), lockup detection |
| `server/src/web/public/diagnostics.js` | Diagnostics view — latency & clock sync (per-hop, sparklines, auto-ping), WiFi RSSI + sparkline, robot info, debug log |

### Modified Files

| File | Change |
|------|--------|
| `server/src/web/public/index.html` | Rewritten as unified SPA shell (TopAppBar + SideNavBar + 3 view sections) |
| `server/src/web/public/map.js` | Added multi-layer trail support (deadReckoning/imuFusion/uwbPath/groundTruth), Aegis color palette, dynamic legend |

### Preserved Unchanged

| File | Status |
|------|--------|
| `server/src/web/public/joystick.js` | Untouched — clean module, same API |
| `server/src/web/public/calibration.html` | Kept as rollback backup |
| All server-side files | Zero changes — same WebSocket protocol, same static file serving |

## Architecture

```
index.html (SPA shell — HTML only)
├── aegis.css (design system)
├── joystick.js (unchanged)
├── map.js (enhanced with LayerManager)
├── app.js (loads first — WS, event bus, view routing)
├── dashboard.js (subscribes to App events)
├── calibration.js (subscribes to App events)
└── diagnostics.js (subscribes to App events)
```

**Event bus pattern:** `app.js` owns the WebSocket and dispatches typed events via `App.emit()`. Each view module registers listeners via `App.on()`. Views don't know about each other — they communicate only through the event bus.

**Element ID convention:** All IDs are prefixed by view: `d-` (dashboard), `c-` (calibration), `g-` (diagnostics). This prevents conflicts since all views exist in the same DOM.

**View switching:** `App.switchView('dashboard'|'calibration'|'diagnostics')` toggles `.view.active` class. Only one view is visible at a time, but all JS modules run continuously — motor test intervals persist across view switches.

## Design System (Aegis Kinetic)

| Token | Hex | Usage |
|-------|-----|-------|
| `--ak-bg-deep` | `#0b1326` | App background |
| `--ak-surface-l1` | `#131b2e` | Cards, sidebars, top bar |
| `--ak-surface-l2` | `#1c253d` | Active states, nested panels |
| `--ak-accent` | `#3B82F6` | Primary buttons, active elements |
| `--ak-success` | `#10b981` | Connected, cal level 3, run state |
| `--ak-warning` | `#f59e0b` | Latency spikes, calibration active |
| `--ak-error` | `#ef4444` | E-STOP, disconnects, lockup |
| `--ak-text` | `#dae2fd` | Primary text |
| `--ak-data` | `#adc6ff` | Key telemetry values, brand logo |
| `--ak-text-muted` | `#dae2fd/50%` | Labels, secondary text |

**Typography:** `Space Grotesk` for headings/data, `JetBrains Mono` for body/logs. Loaded via Google Fonts CDN.

## Map Layers

The trajectory map now supports 4 simultaneous trail layers:

| Layer | Color | Default | Purpose |
|-------|-------|---------|---------|
| `deadReckoning` | Amber `#f59e0b` | Visible | Raw encoder path (Tier 1) |
| `imuFusion` | Emerald `#10b981` | Hidden | IMU-corrected path (Tier 2+) |
| `uwbPath` | Cyan `#06b6d4` | Hidden | UWB EKF path (Tier 3, dashed) |
| `groundTruth` | White `#ffffff` | Visible | Baseline (placeholder, fed externally) |

**API:**
- `robotMap.updateLayerPose(layerName, x, y)` — add point to specific layer
- `robotMap.setLayerVisible(layerName, bool)` — toggle layer visibility
- `robotMap.clearLayerTrail(layerName)` / `robotMap.clearAllTrails()`
- Backward-compatible: `robotMap.trail` aliases `deadReckoning.trail`

## Known Issues / TODO

1. **Ground truth layer has no data source yet** — the `groundTruth` layer exists but nothing feeds it. When ground truth data becomes available (e.g. from manual measurement or motion capture), feed it via `robotMap.updateLayerPose('groundTruth', x, y)` from `dashboard.js`.

2. **Tier-based layer routing not fully wired** — currently all pose data goes to the `deadReckoning` layer. When Tier 2 is active, `imuFusion` layer should get the fused pose separately. This requires the server to broadcast both raw odometry and fused pose in the `state` message (currently only the active tier's pose is sent).

3. **Canvas sparklines need view-activation redraw** — sparklines in the diagnostics view may render at zero dimensions if the view was never shown. The `viewChanged` event triggers a redraw, but if the robot connects while on a different view, the first few data points may be lost.

4. **Calibration.html still exists** — kept as rollback backup. Delete it once the unified UI is validated in production use.

5. **Mobile responsive** — the sidebar collapses on screens < 900px, but the layout isn't fully optimized for phone use. The joystick works with touch events.

## How to Continue

**Adding a new sensor panel:** Create the HTML in the appropriate `<section class="view">` in `index.html` with `g-`/`c-`/`d-` prefixed IDs. Subscribe to `App.on('state', ...)` or `App.on('robotInfo', ...)` in the corresponding JS file.

**Adding a new WebSocket message type:**
1. Add the case to `app.js` `_handleMessage()` → emit a named event
2. Subscribe in the view JS file via `App.on('newEvent', handler)`
3. Send via `App.send({ type: 'new_type', ... })`

**Adding a new map layer:** Add an entry to `this.layers` in `map.js` constructor. Add a checkbox in the `#d-layer-toggles` section of `index.html`. The layer toggle wiring in `dashboard.js` already handles arbitrary `data-layer` attributes.

**Modifying the design system:** All visual tokens are in `:root` in `aegis.css`. Component classes follow BEM-lite naming. The glass-morphism effect uses `backdrop-filter: blur()`.
