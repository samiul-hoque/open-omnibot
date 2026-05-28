#!/usr/bin/env python3
"""Capture new experiment data into the clean data pipeline.

Trigger this after running experiments to extract clean, inspectable CSVs
from the raw server logs into dated collection folders.

Usage:
    # Capture new trajectory experiments:
    python capture.py server/logs/

    # Capture motor calibration data:
    python capture.py --motor-cal \\
        --csv server/logs/robot_data_tier1_2026-04-14T21-35-43.csv \\
        --phases evaluation/experiments/motor_cal_2026-04-14T21-36-52/phases.json

    # Re-capture everything (ignore manifest):
    python capture.py --force server/logs/

Output structure:
    evaluation/data/
      straight_2m/
        2026-04-15T08-15/
          notes.md              session notes (auto-created, user-editable)
          ground_truth.csv      per-run operator measurements
          metrics.csv           per-run per-method error metrics
          trajectories/
            rep01.csv           replayed x,y,theta for all 4 methods
            ...
      motor_calibration/
        2026-04-14T21-36/
          notes.md
          cycles.csv
          representative_before.csv
          representative_after.csv
          metadata.json

A manifest at evaluation/data/.manifest.json tracks which source files
have already been captured, so re-running is safe and incremental.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import re
import sys
from collections import defaultdict
from datetime import datetime
from pathlib import Path

import numpy as np

from replay import (
    WHEEL_RADIUS, METERS_PER_COUNT, COMPLEMENTARY_ALPHA, L_SUM,
    KNOWN_TRAJECTORIES, TRAJ_KIND, TRAJ_LABEL, TRAJ_PATH_M,
    REQUIRED_COLS, METHODS, STRAIGHT_METRIC_COLS, LOOP_METRIC_COLS,
    REPLAY_FNS, compute_metrics,
)

# Motor cal constants
WHEELS = ['L1', 'R1', 'R2', 'L2']
STEADY_STATE_TRIM_MS = 1500
VEL_OUTLIER_THRESHOLD = 80.0


# ---------------------------------------------------------------------------
# Manifest
# ---------------------------------------------------------------------------

def load_manifest(data_dir: Path) -> dict:
    path = data_dir / '.manifest.json'
    if path.exists():
        return json.loads(path.read_text())
    return {'captured_files': {}, 'motor_cal_captures': []}


def save_manifest(data_dir: Path, manifest: dict) -> None:
    path = data_dir / '.manifest.json'
    path.write_text(json.dumps(manifest, indent=2, sort_keys=True) + '\n')


# ---------------------------------------------------------------------------
# CSV loading
# ---------------------------------------------------------------------------

def load_run(csv_path: Path) -> tuple[dict, dict[str, np.ndarray]] | None:
    meta_path = csv_path.with_suffix('.meta.json')
    if not meta_path.exists():
        return None
    with open(meta_path) as f:
        meta = json.load(f)
    exp = meta.get('experiment') or {}
    if exp.get('aborted'):
        return None
    gt = exp.get('groundTruth')
    if not gt:
        return None

    with open(csv_path, newline='') as f:
        reader = csv.reader(f)
        header = next(reader)
        col_idx = {name: i for i, name in enumerate(header)}
        missing = [c for c in REQUIRED_COLS if c not in col_idx]
        if missing:
            return None
        raw_rows = list(reader)

    if len(raw_rows) < 5:
        return None

    def col_floats(name: str) -> np.ndarray:
        idx = col_idx[name]
        out = np.empty(len(raw_rows), dtype=float)
        for i, r in enumerate(raw_rows):
            v = r[idx] if idx < len(r) else ''
            try:
                out[i] = float(v) if v != '' else math.nan
            except ValueError:
                out[i] = math.nan
        return out

    cols = {name: col_floats(name) for name in REQUIRED_COLS}
    return meta, cols



# Replay methods and metrics are imported from replay.py (single source of truth).


# ---------------------------------------------------------------------------
# CSV writers
# ---------------------------------------------------------------------------

def write_trajectory_csv(path, trajectories):
    header = ['t_sec']
    suffixes = [('enc','Encoder-only'),('imu','IMU-only'),
                ('comp','Complementary'),('bno','BNO055 Fusion')]
    for short, _ in suffixes:
        header.extend([f'x_{short}', f'y_{short}', f'th_{short}'])
    ref = trajectories['Encoder-only']; n = len(ref)
    with open(path, 'w', newline='') as f:
        w = csv.writer(f); w.writerow(header)
        for i in range(n):
            row = [f'{ref[i,0]:.4f}']
            for short, method in suffixes:
                t = trajectories[method]
                row.extend([f'{t[i,1]:.6f}', f'{t[i,2]:.6f}', f'{t[i,3]:.6f}'])
            w.writerow(row)

def write_ground_truth_csv(path, entries):
    # start_x_m / start_y_m / start_theta_deg hold the preflight
    # `measuredStartPose` captured before the trajectory began. These are
    # empty strings for pre-preflight historical runs. Downstream analysis
    # rotates (x_meas, y_meas, theta_meas) into the body frame defined by
    # the start pose when those columns are populated — see
    # replay.compute_metrics.
    with open(path, 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow([
            'rep', 'source_csv',
            'x_meas_m', 'y_meas_m', 'theta_meas_deg',
            'start_x_m', 'start_y_m', 'start_theta_deg',
        ])
        for e in sorted(entries, key=lambda e: e['rep']):
            sp = e.get('start_pose') or {}
            w.writerow([
                e['rep'], e['source_csv'],
                e['x'], e['y'], e['theta_deg'],
                sp.get('x', ''), sp.get('y', ''), sp.get('thetaDeg', ''),
            ])

def write_metrics_csv(path, entries, metric_cols):
    with open(path, 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['rep','method'] + metric_cols)
        for e in sorted(entries, key=lambda e: (e['rep'], METHODS.index(e['method']))):
            w.writerow([e['rep'], e['method']] + [f'{e[c]:.4f}' for c in metric_cols])


# ---------------------------------------------------------------------------
# Notes template
# ---------------------------------------------------------------------------

def write_notes(path: Path, traj_id: str, n_valid: int, n_total: int,
                speed: float, date_str: str, extra: str = '') -> None:
    """Write a notes.md template. Never overwrites an existing file."""
    if path.exists():
        return
    label = TRAJ_LABEL.get(traj_id, traj_id)
    content = f"""# {label} — {date_str}

| Field | Value |
|---|---|
| Trajectory | {label} |
| Date | {date_str} |
| Valid runs | {n_valid} |
| Total runs (incl. aborted) | {n_total} |
| Speed | {speed} m/s |

## Conditions
<!-- Surface type, battery level, lighting, any setup notes -->

## Observations
<!-- How did the runs look? Any drift patterns, consistent errors? -->

## Issues
<!-- Aborted runs, sensor glitches, mechanical problems -->
{extra}
"""
    path.write_text(content)


def write_motor_cal_notes(path: Path, n_cycles: int, pwm: int,
                          date_str: str) -> None:
    if path.exists():
        return
    content = f"""# Motor Calibration — {date_str}

| Field | Value |
|---|---|
| Date | {date_str} |
| Cycles | {n_cycles} |
| PWM | {pwm} |

## Conditions
<!-- Robot on stand? Battery level? -->

## Observations
<!-- Calibration convergence, any outlier cycles -->

## Issues
<!-- Failed cycles, unusual readings -->
"""
    path.write_text(content)


# ---------------------------------------------------------------------------
# Timestamp helpers
# ---------------------------------------------------------------------------

def extract_collection_timestamp(start_time_iso: str) -> str:
    """Convert '2026-04-15T08:15:07.069Z' → '2026-04-15T08-15'."""
    # Parse and reformat to minute-level, filesystem-safe
    dt = datetime.fromisoformat(start_time_iso.replace('Z', '+00:00'))
    return dt.strftime('%Y-%m-%dT%H-%M')


def count_total_files(logs_dir: Path, traj_id: str) -> int:
    """Count all exp CSVs for a trajectory (including aborted)."""
    pattern = f'exp_{traj_id}_*.csv'
    return len(list(logs_dir.glob(pattern)))


# ---------------------------------------------------------------------------
# Trajectory capture
# ---------------------------------------------------------------------------

def capture_trajectories(logs_dir: Path, data_dir: Path, force: bool = False) -> int:
    manifest = load_manifest(data_dir)
    captured = manifest['captured_files']

    csvs = sorted(logs_dir.glob('exp_*.csv'))
    if not csvs:
        print(f'No exp_*.csv files in {logs_dir}')
        return 1

    # Collect valid, uncaptured runs
    new_runs: dict[str, list] = defaultdict(list)  # traj_id -> [(csv_path, meta, cols), ...]
    skipped = aborted = already = 0

    for csv_path in csvs:
        fname = csv_path.name
        if not force and fname in captured:
            already += 1
            continue

        result = load_run(csv_path)
        if result is None:
            # Still mark as seen so we don't re-scan
            captured[fname] = {'status': 'skipped'}
            skipped += 1
            continue
        meta, cols = result
        exp = meta.get('experiment', {})
        traj_id = exp.get('trajectory')
        if traj_id not in KNOWN_TRAJECTORIES:
            captured[fname] = {'status': 'skipped', 'reason': f'unknown trajectory {traj_id}'}
            skipped += 1
            continue
        new_runs[traj_id].append((csv_path, meta, cols))

    if not new_runs:
        if already:
            print(f'All {already} files already captured. Use --force to re-capture.')
        else:
            print(f'No valid new runs found ({skipped} skipped).')
        save_manifest(data_dir, manifest)
        return 0

    # Process each trajectory group
    for traj_id, runs in sorted(new_runs.items()):
        # Sort by start time
        runs.sort(key=lambda r: r[1].get('startTime', ''))
        first_time = runs[0][1].get('startTime', '')
        collection_ts = extract_collection_timestamp(first_time) if first_time else 'unknown'

        coll_dir = data_dir / traj_id / collection_ts
        trajs_dir = coll_dir / 'trajectories'
        trajs_dir.mkdir(parents=True, exist_ok=True)

        # Check for existing reps in this folder (for incremental adds)
        existing_reps = len(list(trajs_dir.glob('rep*.csv')))
        rep_offset = existing_reps

        ground_truth = []
        metrics_entries = []
        speed = 0.15

        for idx, (csv_path, meta, cols) in enumerate(runs):
            rep = rep_offset + idx + 1
            exp = meta.get('experiment', {})
            gt = exp['groundTruth']
            # Preflight captures the physical start pose for every run
            # from 2026-04-19 onward; older meta files won't have this
            # key and compute_metrics falls through to body-frame-gt
            # assumption (matches the pre-rig manual-measurement workflow).
            start_pose = exp.get('measuredStartPose') or None
            speed = exp.get('speed', speed)

            ground_truth.append({
                'rep': rep, 'source_csv': csv_path.name,
                'x': gt['xMeas'], 'y': gt['yMeas'], 'theta_deg': gt['thetaDegMeas'],
                'start_pose': start_pose,
            })

            # Replay all methods
            trajectories = {}
            for method, fn in REPLAY_FNS.items():
                try:
                    traj = fn(cols)
                except Exception as e:
                    print(f'  WARN: {csv_path.name} {method} replay failed: {e}', file=sys.stderr)
                    continue
                trajectories[method] = traj
                m = compute_metrics(traj_id, traj, gt, start_pose=start_pose)
                m['rep'] = rep
                m['method'] = method
                metrics_entries.append(m)

            if len(trajectories) == len(METHODS):
                write_trajectory_csv(trajs_dir / f'rep{rep:02d}.csv', trajectories)

            captured[csv_path.name] = {'status': 'captured', 'collection': f'{traj_id}/{collection_ts}', 'rep': rep}

        # Write / append ground truth
        gt_path = coll_dir / 'ground_truth.csv'
        if gt_path.exists() and rep_offset > 0:
            # Append to existing — keep column count in sync with the
            # header that write_ground_truth_csv emits.
            with open(gt_path, 'a', newline='') as f:
                w = csv.writer(f)
                for e in sorted(ground_truth, key=lambda e: e['rep']):
                    sp = e.get('start_pose') or {}
                    w.writerow([
                        e['rep'], e['source_csv'],
                        e['x'], e['y'], e['theta_deg'],
                        sp.get('x', ''), sp.get('y', ''), sp.get('thetaDeg', ''),
                    ])
        else:
            write_ground_truth_csv(gt_path, ground_truth)

        # Write / append metrics
        kind = TRAJ_KIND[traj_id]
        mcols = STRAIGHT_METRIC_COLS if kind == 'straight' else LOOP_METRIC_COLS
        metrics_path = coll_dir / 'metrics.csv'
        if metrics_path.exists() and rep_offset > 0:
            with open(metrics_path, 'a', newline='') as f:
                w = csv.writer(f)
                for e in sorted(metrics_entries, key=lambda e: (e['rep'], METHODS.index(e['method']))):
                    w.writerow([e['rep'], e['method']] + [f'{e[c]:.4f}' for c in mcols])
        else:
            write_metrics_csv(metrics_path, metrics_entries, mcols)

        # Notes
        n_total = count_total_files(logs_dir, traj_id)
        write_notes(coll_dir / 'notes.md', traj_id, len(runs), n_total, speed, collection_ts)

        print(f'  {traj_id}/{collection_ts}: {len(runs)} runs captured')

    save_manifest(data_dir, manifest)

    # Auto-update thesis_datasets.json — set each trajectory to this collection
    # only if there isn't already a pinned entry
    _auto_update_thesis_datasets(data_dir, new_runs, {
        tid: extract_collection_timestamp(runs[0][1].get('startTime', ''))
        for tid, runs in new_runs.items()
        if runs[0][1].get('startTime')
    })

    print(f'\nDone. {sum(len(r) for r in new_runs.values())} new runs captured.')
    return 0


def _auto_update_thesis_datasets(data_dir: Path, new_runs: dict, timestamps: dict) -> None:
    config_path = data_dir / 'thesis_datasets.json'
    if config_path.exists():
        config = json.loads(config_path.read_text())
    else:
        config = {}
    changed = False
    for traj_id, ts in timestamps.items():
        if traj_id not in config:
            config[traj_id] = ts
            changed = True
    if changed:
        config_path.write_text(json.dumps(config, indent=2, sort_keys=True) + '\n')
        print(f'  Updated {config_path}')


# ---------------------------------------------------------------------------
# Motor cal capture
# ---------------------------------------------------------------------------

def capture_motor_cal(csv_path: Path, phases_path: Path, data_dir: Path) -> int:
    import pandas as pd

    phases = json.loads(phases_path.read_text())
    df = pd.read_csv(csv_path, on_bad_lines='skip', engine='python')

    if 'runs' not in phases:
        print('ERROR: phases.json lacks a `runs` array.', file=sys.stderr)
        return 2

    # Derive collection timestamp from phases.json
    started = phases.get('started_at_iso', '')
    if started:
        collection_ts = extract_collection_timestamp(started)
    else:
        collection_ts = 'unknown'

    out_dir = data_dir / 'motor_calibration' / collection_ts
    out_dir.mkdir(parents=True, exist_ok=True)

    # Per-cycle aggregation
    cycle_rows = []
    for run in phases['runs']:
        before = _slice_window(df, run['before']['start_ms'], run['before']['end_ms'])
        after = _slice_window(df, run['after']['start_ms'], run['after']['end_ms'])
        if before.empty or after.empty:
            print(f'  WARN: cycle {run["index"]+1} empty window — skipping', file=sys.stderr)
            continue
        bs = _motor_stats(before)
        as_ = _motor_stats(after)
        cycle_rows.append({
            'cycle': run['index'] + 1,
            'direction': run['direction'],
            'before_mean_rad_s': bs['grand_mean'],
            'after_mean_rad_s': as_['grand_mean'],
            'before_spread_pct': bs['spread_range'] * 100,
            'after_spread_pct': as_['spread_range'] * 100,
            'before_cv_pct': bs['spread_cv'] * 100,
            'after_cv_pct': as_['spread_cv'] * 100,
        })

    if not cycle_rows:
        print('ERROR: no usable cycles', file=sys.stderr)
        return 1

    runs_df = pd.DataFrame(cycle_rows)
    runs_df['improvement_ratio'] = runs_df['before_spread_pct'] / runs_df['after_spread_pct'].replace(0, np.nan)

    # Write cycles.csv
    runs_df.to_csv(out_dir / 'cycles.csv', index=False, float_format='%.4f')
    print(f'  cycles.csv ({len(runs_df)} cycles)')

    # Representative run
    rep_idx = _pick_representative(runs_df)
    rep_run = phases['runs'][rep_idx]

    before_df = _slice_window(df, rep_run['before']['start_ms'], rep_run['before']['end_ms'])
    after_df = _slice_window(df, rep_run['after']['start_ms'], rep_run['after']['end_ms'])

    vel_cols = [f'vel_{w}' for w in WHEELS]
    for label, window_df in [('representative_before', before_df), ('representative_after', after_df)]:
        path = out_dir / f'{label}.csv'
        with open(path, 'w', newline='') as f:
            w = csv.writer(f)
            w.writerow(['t_s'] + vel_cols)
            for _, row in window_df.iterrows():
                w.writerow([f'{row["t_s"]:.4f}'] + [f'{row[c]:.4f}' for c in vel_cols])
        print(f'  {label}.csv ({len(window_df)} samples)')

    # Metadata
    meta = {
        'pwm': phases['pwm'], 'window_ms': phases['window_ms'],
        'steady_state_trim_ms': STEADY_STATE_TRIM_MS,
        'effective_window_ms': phases['window_ms'] - STEADY_STATE_TRIM_MS,
        'total_cycles': len(runs_df),
        'representative_cycle': rep_idx + 1,
        'representative_direction': rep_run['direction'],
        'source_csv': str(csv_path), 'source_phases': str(phases_path),
    }
    (out_dir / 'metadata.json').write_text(json.dumps(meta, indent=2) + '\n')

    # Notes
    write_motor_cal_notes(out_dir / 'notes.md', len(runs_df), phases['pwm'], collection_ts)

    # Update thesis_datasets.json
    config_path = data_dir / 'thesis_datasets.json'
    config = json.loads(config_path.read_text()) if config_path.exists() else {}
    if 'motor_calibration' not in config:
        config['motor_calibration'] = collection_ts
        config_path.write_text(json.dumps(config, indent=2, sort_keys=True) + '\n')

    print(f'\n  motor_calibration/{collection_ts}: {len(runs_df)} cycles captured')
    return 0


def _slice_window(df, start_ms, end_ms):
    t0 = start_ms + STEADY_STATE_TRIM_MS
    window = df[(df['timestamp'] >= t0) & (df['timestamp'] <= end_ms)].copy()
    window['t_s'] = (window['timestamp'] - t0) / 1000.0
    for wheel in WHEELS:
        col = f'vel_{wheel}'
        window.loc[window[col].abs() > VEL_OUTLIER_THRESHOLD, col] = np.nan
    return window

def _motor_stats(df):
    data = df[[f'vel_{w}' for w in WHEELS]].abs()
    means = data.mean().to_numpy()
    grand = float(np.mean(means))
    return {
        'grand_mean': grand,
        'spread_cv': float(np.std(means)/grand) if grand else float('nan'),
        'spread_range': float((means.max()-means.min())/grand) if grand else float('nan'),
    }

def _pick_representative(runs_df):
    subset = runs_df[runs_df['direction'] == 'forward']
    if subset.empty: subset = runs_df
    target = subset['before_spread_pct'].median()
    idx = (subset['before_spread_pct'] - target).abs().idxmin()
    return int(runs_df.loc[idx, 'cycle']) - 1


# ---------------------------------------------------------------------------
# Driver
# ---------------------------------------------------------------------------

def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('logs_dir', nargs='?', type=Path, default=Path('server/logs'),
                        help='Directory containing exp_*.csv + .meta.json (default: server/logs)')
    parser.add_argument('--out', type=Path, default=Path('evaluation/data'),
                        help='Output data directory (default: evaluation/data)')
    parser.add_argument('--force', action='store_true',
                        help='Re-capture all files (ignore manifest)')
    parser.add_argument('--motor-cal', action='store_true',
                        help='Capture motor calibration data instead of trajectories')
    parser.add_argument('--csv', type=Path,
                        help='Server CSV for motor cal (required with --motor-cal)')
    parser.add_argument('--phases', type=Path,
                        help='phases.json for motor cal (required with --motor-cal)')
    args = parser.parse_args(argv)

    args.out.mkdir(parents=True, exist_ok=True)

    if args.motor_cal:
        if not args.csv or not args.phases:
            print('error: --motor-cal requires --csv and --phases', file=sys.stderr)
            return 1
        return capture_motor_cal(args.csv, args.phases, args.out)
    else:
        return capture_trajectories(args.logs_dir, args.out, force=args.force)


if __name__ == '__main__':
    sys.exit(main())
