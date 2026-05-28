# Overhead ArUco Ground-Truth Snapshot Service

On-demand snapshot service that sits beside the server and takes one ArUco
photo when asked. Used as a sanity check on operator-measured ground truth
at experiment waypoints — **not** as a continuous pose stream.

## Fresh session

Two pieces need to be running:

1. The **snapshot service** (this script). Start it in a terminal:
   ```bash
   ./evaluation/.venv/bin/python evaluation/ground_truth/aruco_detector.py
   ```
2. The **server** (`npm start` in `server/`). Its Calibration page has
   a "Ground-Truth Camera" panel — hit Recalibrate whenever the phone
   has been moved or recharged. It spawns `calibrate_homography.py`,
   writes a fresh `calibration.json`, and nudges the snapshot service
   to reload, all in one click.

**Tip:** move the robot OFF the grid during the Recalibrate click —
every intersection it occludes is a correspondence lost.

**Reprints:** use `generate_marker.py` — do not use online marker
generators (they occasionally output inverted polarity that the
detector silently rejects).

## Manual calibration (fallback)

If you want to recalibrate from the shell without the UI:

```bash
./evaluation/.venv/bin/python evaluation/ground_truth/calibrate_homography.py --save-visual
```

Then either restart the snapshot service or
`curl -X POST http://localhost:5055/reload_calibration` to pick up the
new homography without a restart. Typical quality on this rig:
≥ 20 inlier points, ≤ 15 mm median reprojection error.

## Running the snapshot service

```bash
./evaluation/.venv/bin/python evaluation/ground_truth/aruco_detector.py
```

Listens on `http://localhost:5055`. Endpoints:

| Endpoint | Method | Purpose |
|---|---|---|
| `/snapshot?trajectory=X&run_id=Y&label=Z` | GET | Grab a fresh frame, detect marker, save annotated PNG to `evaluation/snapshots/X/Y/Z.png`, return JSON pose |
| `/health` | GET | Liveness + last-frame timestamp |
| `/reload_calibration` | POST | Re-read `calibration.json` from disk (use after recalibrating) |

Response from `/snapshot` (200 on success, 404 if no marker):

```json
{
  "trajectory": "straight_2m",
  "run_id": "exp_straight_2m_0.10_tier1_rep1_2026-04-18T14-03-10",
  "label": "end",
  "image_path": "evaluation/snapshots/straight_2m/exp_.../end.png",
  "detect_ms": 1776523456789,
  "detected": true,
  "x": 1.378, "y": 0.138,
  "theta": 0.2147, "theta_deg": 12.3,
  "side_px": 78.5
}
```

## Diagnosing a bad reprint

If the snapshot service returns `detected: false` and you're sure the
marker is in frame, run the one-shot diagnostic:

```bash
./evaluation/.venv/bin/python evaluation/ground_truth/aruco_detector.py --diagnose
```

Scans `DICT_4X4_{50,100,250,1000}` across 60 frames and reports which
dict decoded the marker (or "no dict decoded at real size" — typically
means polarity inversion or bit-bleed in the print).

## Layout constants

| Flag | Default | Meaning |
|---|---|---|
| `--stream` | RTSP from phone | Video source URL |
| `--marker-id` | 0 | ID in DICT_4X4_50 |
| `--marker-dict` | `4X4_50` | ArUco dictionary name |
| `--min-side-px` | 50 | Detections smaller than this are rejected as false positives |
| `--yaw-off-deg` | 90 | Added to detected heading. Marker-on-robot orientation offset |
| `--port` | 5055 | HTTP port |
| `--snapshot-root` | `evaluation/snapshots` | Where annotated PNGs land |

All calibration-driven parameters (origin, pixel scale, tilt) load from
`calibration.json` automatically.

## How GT values reach the data

Ground truth is captured **once per run**, not streamed. At the end of
an experiment (in the `awaiting_ground_truth` phase of the trajectory
runner), either the operator enters `(x, y, θ)` manually or triggers a
camera snapshot that auto-fills those fields, then submits. The values
land in the run's `meta.json` under `experiment.groundTruth` — not in
the CSV.

## `drive_straight.py`

Standalone drive test that uses the snapshot service for start/end GT
capture (no trajectory runner). See `drive_straight.py --help`.
