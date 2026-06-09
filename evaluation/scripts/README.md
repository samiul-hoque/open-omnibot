# evaluation/scripts

## Ground-truth data

Ground truth is recorded **per-run** via the experiment runner's
`submitGroundTruth` path — either typed manually or auto-filled from
a single ArUco snapshot at the end of the run. The values live in the
per-run `meta.json` sidecar (under the `experiment.groundTruth` block),
not in CSV columns. See `../ground_truth/README.md`.

## Data Pipeline (Python)

The canonical data pipeline for thesis Chapter 7 figures and tables.
See `docs/data-pipeline.md` for the full end-to-end documentation.

| Script | Input | Output |
|---|---|---|
| `replay.py` | — (library) | Shared replay functions, constants, metrics |
| `capture.py` | `server/logs/exp_*.csv` | `evaluation/data/<trajectory>/<date>/` |
| `plot_trajectories.py` | `evaluation/data/` | Thesis PDFs + LaTeX tables |
| `plot_motor_cal_clean.py` | `evaluation/data/motor_calibration/` | Motor cal thesis figure |

### Typical workflow

```bash
source evaluation/.venv/bin/activate

# After running experiments — capture new data:
python evaluation/scripts/capture.py server/logs/

# After running motor cal:
python evaluation/scripts/capture.py --motor-cal \
  --csv server/logs/robot_data_tier1_<ts>.csv \
  --phases evaluation/experiments/motor_cal_<ts>/phases.json

# Generate thesis figures:
python evaluation/scripts/plot_trajectories.py
python evaluation/scripts/plot_motor_cal_clean.py
```

---

## Experiment Runners (Node.js)

Standalone scripts that connect directly to the robot over WebSocket and
own the command stream — **stop the main server (`npm start`) before
running any of these**, otherwise you'll race the server's `cmd`
broadcasts at 50 Hz and get nonsense data.

All scripts take `ROBOT_IP` from the environment (default
`robot.local`) and drop results under
`evaluation/experiments/<script>_<iso-timestamp>/`.

| Script | Robot state | Duration | Purpose |
|---|---|---|---|
| `recalibrate_motor_gains.mjs` | stand (free wheels) | ~20 s | Trigger firmware auto-cal + save to NVS |
| `motion_sweep_experiment.mjs` | stand | ~3-5 min | Per-wheel tracking error across directions x speeds x repeats |
| `ground_imu_experiment.mjs` | floor | ~2 min | FK vs IMU comparison under each movement command |
| `strafe_debug_experiment.mjs` | stand | ~10 s | Single-press PID introspection (historical) |
| `motor_cal_experiment.mjs` | stand | ~minutes | Before/after motor-cal campaign for thesis plot |

### `recalibrate_motor_gains.mjs`

Thin wrapper that sends `start_motor_cal` to the firmware, waits for
`motor_cal_result`, and persists with `save_motor_cal`. No CLI flags.

```bash
ROBOT_IP=robot.local node evaluation/scripts/recalibrate_motor_gains.mjs
```

**When to run**: after any change that might affect per-wheel friction
or the motor-wheel pairing — chassis redesign, tyre replacement,
motor-pin reassignment in `config.h`.

### `motion_sweep_experiment.mjs`

Sweeps all six movement commands (W/S/A/D/Q/E) at multiple speeds with
multiple repeats, reports mean & std tracking error per wheel.

```bash
node evaluation/scripts/motion_sweep_experiment.mjs \
  --runs 3 --duration-ms 2000 \
  --lin-speeds 0.05,0.10,0.20 --ang-speeds 0.20,0.40,0.80
```

**Baseline reference**: baseline runs captured on the reference robot
are not distributed in this repository — capture your own baseline
before making chassis or motor changes.

### `ground_imu_experiment.mjs`

Drives the robot through each command on the floor, captures sensor
stream (encoders + IMU), prints FK-derived body velocity next to
IMU-measured yaw drift. Supports heading-hold A/B.

```bash
node evaluation/scripts/ground_imu_experiment.mjs \
  --duration-ms 5000 --lin 0.10 --ang 0.30 \
  --heading-hold both
```

**Baseline reference**: baseline runs captured on the reference robot
are not distributed in this repository — capture your own baseline
before making chassis or motor changes.

### `strafe_debug_experiment.mjs`

Single-press PID introspection. Historical — used on 2026-04-15 to
trace the strafe-becomes-rotation bug.

### `motor_cal_experiment.mjs`

Multi-run campaign for the thesis motor-calibration figure.

```bash
node evaluation/scripts/motor_cal_experiment.mjs \
  --runs 10 --pwm 160 --window-ms 10000
```

---

## Deprecated Scripts

Old scripts that have been superseded by the pipeline above live in
`deprecated/`. They still work but are no longer maintained:

| Deprecated | Replaced by |
|---|---|
| `analyze_experiments.py` | `capture.py` + `plot_trajectories.py` |
| `extract_trajectory_data.py` | `capture.py` |
| `extract_motor_cal_data.py` | `capture.py --motor-cal` |
| `plot_motor_cal.py` | `plot_motor_cal_clean.py` |
| `analyze_trajectory.py` | `replay.py` + `capture.py` |
| `compute_metrics.py` | `replay.py` |
