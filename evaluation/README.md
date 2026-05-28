# Evaluation

Offline analysis tooling for Open Omnibot experiment runs.

The control server's experiment runner emits a CSV stream plus a JSON
metadata sidecar for each run; the scripts in this directory ingest
those logs, replay the raw encoder + IMU stream through the
localization tiers, compute per-run metrics, and emit plots and
summary tables.

## Setup (one-time)

```bash
cd evaluation
python3 -m venv venv
source venv/bin/activate
pip install -r requirements.txt
```

Some of the Node-based experiment runners (`scripts/*.mjs`) need the
`ws` package — the simplest way is to run them from a checkout where
`server/node_modules/ws` is installed:

```bash
cd ../server && npm install && cd ../evaluation
```

## Scripts

- `scripts/extended_metrics.py` — replay logs through tier 1 / 2 and
  compute closure error, drift, and inter-rep variance.
- `scripts/plot_trajectories.py` — overlay commanded vs estimated
  trajectories per run.
- `scripts/plot_motor_cal_clean.py` — generate the inter-wheel
  uniformity bar chart (before vs after calibration).
- `scripts/motion_sweep_experiment.mjs`,
  `scripts/strafe_debug_experiment.mjs`,
  `scripts/motor_cal_experiment.mjs`,
  `scripts/recalibrate_motor_gains.mjs`,
  `scripts/ground_imu_experiment.mjs`,
  `scripts/self_test_runner.mjs`,
  `scripts/verify_motor_mapping.mjs` — Node scripts that drive the
  robot through specific test patterns. Each takes `ROBOT_IP` as an
  environment variable; default is `robot.local`.

`scripts/README.md` has per-script details and example invocations.

## Ground truth

The `ground_truth/` subdirectory contains the overhead-ArUco rig
toolchain: camera calibration (`calibrate_homography.py`), ArUco
detection (`aruco_detector.py`), a snapshot HTTP service, and the
grid annotation tool. See `ground_truth/calibration.json` for the
default configuration template (the camera-host placeholder needs
to be edited for your setup).

## What makes a run usable

A "run" is a `(csv, meta.json)` pair produced by the experiment
runner. For analysis it needs:

- A non-trivial trajectory (more than just idle dwell).
- A populated `meta.json.experiment.groundTruth` block with operator-
  submitted end-pose measurements, OR an inline ArUco snapshot
  ingested by the server before the run ended.
- A `tier` field in the meta sidecar so per-tier comparisons can
  group runs correctly.

Runs that fail any of these are flagged and skipped by the analysis
scripts.
