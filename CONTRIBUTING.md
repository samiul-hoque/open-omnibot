# Contributing to Open Omnibot

Thanks for your interest. Open Omnibot is an open-source mobile-robot
platform released under the MIT license. Contributions, bug reports,
hardware photos, and reproductions are all welcome.

## Quick orientation

| Layer | Where it lives | What to expect |
|---|---|---|
| Mechanical / electrical | `hardware/` | BOM, pinout, and short component-level READMEs. CAD/PCB artefacts are placeholders in this release. |
| Firmware (ESP32) | `firmware/esp32-omni/` | PlatformIO project. Cooperative main loop, per-wheel PID, motor-calibration state machine, OTA. Native unit tests under `test/`. |
| UWB firmware (DWM1001) | `firmware/dwm1001-uwb/` | Stub. The DWM1001 reader was removed from the runtime on 2026-04-14; see `localization/tier3-ekf.md` for status. |
| Server (Node.js) | `server/` | WebSocket bridge to the robot, browser dashboard, experiment runner, ~170 unit/integration tests. ES modules. |
| Localization | `localization/` | Algorithm theory + MATLAB reference for Tiers 1 (operational), 2 (operational), and 3 (planned). |
| Evaluation tooling | `evaluation/` | Experiment runners and overhead-ArUco ground-truth helpers. |
| Docs | `docs/` | User-facing manual rendered by the dashboard's docs viewer. The structure is defined by `docs/manifest.json`. |

## Reporting bugs

Open an issue with:

1. What you observed.
2. What you expected.
3. How to reproduce (firmware version / commit, server commit,
   hardware variant if non-stock).
4. Any logs from `server/logs/` (gitignored, so OK to paste).

## Submitting changes

1. Fork and create a feature branch off `main`.
2. Run the existing tests:
   ```bash
   # Server
   cd server && npm install && npm test

   # Firmware native tests
   cd firmware/esp32-omni && pio test -e native
   ```
3. Open a pull request with a clear description of the change and
   why. For non-trivial changes, open an issue first to discuss.

## Code style

- **Firmware**: Arduino / C++ conventions, 2-space indent.
- **Server**: ESLint (`npm run lint`), Node.js conventions, ES modules.
- **Python**: PEP 8.
- **Markdown**: keep one sentence per line in long-form prose if you can.

## Local configuration

Sensitive or machine-specific values do not belong in the tree.
Override them locally:

- Firmware: copy `firmware/esp32-omni/src/config.h` to
  `config.local.h` and edit there.
- Server: create `server/src/config.local.js` exporting a partial
  config object — it is deep-merged at startup.

Both `config.local.*` paths are gitignored.

## Things that are out of scope for this release

- The full undergraduate thesis manuscript is not part of this
  repository. The published paper(s) deriving from this work will
  be linked from the README once available.
- The 602 MB overhead-camera snapshot dataset referenced by parts
  of the codebase lives in an external Zenodo deposit (link TBD).
- Tier 3 (UWB + EKF) is reference theory only and was removed from
  the runtime on 2026-04-14. See `localization/tier3-ekf.md`.
