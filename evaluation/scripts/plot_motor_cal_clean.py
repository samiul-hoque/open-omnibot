#!/usr/bin/env python3
"""Generate thesis motor-calibration figure from clean extracted data.

Stage 2 of the thesis data pipeline (motor calibration).

Usage:
    python3 plot_motor_cal_clean.py [--data <data_dir>] [--out <figure_path>]
    python3 plot_motor_cal_clean.py --use 2026-04-14T21-36  # override dataset

Reads thesis_datasets.json to pick the motor_calibration dated folder
(falls back to the latest). Expects the directory layout from capture.py:

    evaluation/data/motor_calibration/
      2026-04-14T21-36/
        cycles.csv
        representative_before.csv
        representative_after.csv
        metadata.json
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd


WHEELS = ['L1', 'R1', 'R2', 'L2']
WHEEL_COLORS = {
    'L1': '#1f77b4',
    'R1': '#d62728',
    'R2': '#ff7f0e',
    'L2': '#2ca02c',
}


def plot_time_series(ax, df: pd.DataFrame, title: str, y_range: tuple[float, float]) -> None:
    for wheel in WHEELS:
        vals = df[f'vel_{wheel}'].abs()
        ax.plot(df['t_s'], vals, color=WHEEL_COLORS[wheel], label=wheel, linewidth=1.2)
    means = df[[f'vel_{w}' for w in WHEELS]].abs().mean()
    grand = float(means.mean())
    spread = float((means.max() - means.min()) / grand) * 100 if grand != 0 else 0
    cv = float(means.std() / grand) * 100 if grand != 0 else 0
    ax.axhline(grand, color='grey', linestyle='--', linewidth=0.8, alpha=0.7)
    ax.set_title(title)
    ax.set_xlabel('Time (s)')
    ax.set_ylabel(r'$|$Wheel velocity$|$ (rad/s)')
    ax.set_ylim(*y_range)
    ax.grid(True, alpha=0.3)
    ax.legend(loc='lower right', ncol=4, fontsize=8, framealpha=0.9)
    textbox = (
        f'mean = {grand:.2f} rad/s\n'
        f'spread (max-min)/mean = {spread:.1f}%\n'
        f'CV of per-wheel means = {cv:.1f}%'
    )
    ax.text(0.02, 0.97, textbox, transform=ax.transAxes,
            fontsize=8, va='top', ha='left',
            bbox=dict(boxstyle='round,pad=0.3', fc='white', ec='lightgrey', alpha=0.9))


def plot_per_run_spread(ax, runs_df: pd.DataFrame) -> None:
    x = np.arange(1, len(runs_df) + 1)
    width = 0.38
    ax.bar(x - width / 2, runs_df['before_spread_pct'], width=width,
           color='#aa4444', alpha=0.85, label='Before (unity gains)',
           edgecolor='#702020', linewidth=0.4)
    ax.bar(x + width / 2, runs_df['after_spread_pct'], width=width,
           color='#3d8f3d', alpha=0.85, label='After (calibrated)',
           edgecolor='#254825', linewidth=0.4)
    bm = runs_df['before_spread_pct'].mean()
    am = runs_df['after_spread_pct'].mean()
    ax.axhline(bm, color='#aa4444', linestyle=':', linewidth=1.0, alpha=0.9)
    ax.axhline(am, color='#3d8f3d', linestyle=':', linewidth=1.0, alpha=0.9)
    ax.text(len(runs_df) + 0.35, bm, f'mean={bm:.2f}%', color='#aa4444',
            fontsize=7, va='center', ha='left')
    ax.text(len(runs_df) + 0.35, am, f'mean={am:.2f}%', color='#3d8f3d',
            fontsize=7, va='center', ha='left')
    dirs = ['F' if d == 'forward' else 'R' for d in runs_df['direction']]
    labels = [f'{i}\n({d})' for i, d in zip(x, dirs)]
    ax.set_xticks(x)
    ax.set_xticklabels(labels, fontsize=8)
    ax.set_xlabel('Run index (F = forward, R = reverse)')
    ax.set_ylabel('Inter-motor spread $(v_{max} - v_{min})/\\bar{v}$ (%)')
    ax.set_title(f'Per-run spread across {len(runs_df)} cycles')
    ax.grid(True, axis='y', alpha=0.3)
    ax.legend(loc='upper right', fontsize=8, framealpha=0.9)
    ax.set_xlim(0.4, len(runs_df) + 1.6)


def resolve_motor_cal_dir(data_dir: Path, use_override: str | None) -> Path | None:
    """Find the dated motor_calibration folder.

    Priority: --use override > thesis_datasets.json > latest folder.
    """
    mcal_root = data_dir / 'motor_calibration'
    if not mcal_root.is_dir():
        return None

    if use_override:
        candidate = mcal_root / use_override
        if candidate.is_dir():
            return candidate
        print(f'  WARN: override {use_override} not found', file=sys.stderr)

    config_path = data_dir / 'thesis_datasets.json'
    if config_path.exists():
        config = json.loads(config_path.read_text())
        pinned = config.get('motor_calibration')
        if pinned:
            candidate = mcal_root / pinned
            if candidate.is_dir():
                return candidate

    dated = sorted([d for d in mcal_root.iterdir() if d.is_dir()])
    return dated[-1] if dated else None


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--data', type=Path, default=Path('evaluation/data'),
                    help='Clean data root directory (default: evaluation/data)')
    ap.add_argument('--out', type=Path,
                    default=Path('figures/motor_calibration_before_after.pdf'),
                    help='Output figure path')
    ap.add_argument('--use', type=str, default=None,
                    help='Override dataset, e.g. --use 2026-04-14T21-36')
    ap.add_argument('--also-png', action='store_true')
    args = ap.parse_args(argv)

    dataset_dir = resolve_motor_cal_dir(args.data, args.use)
    if dataset_dir is None:
        print('error: no motor_calibration data found — run capture.py --motor-cal first', file=sys.stderr)
        return 1
    print(f'Using dataset: {dataset_dir}')

    cycles_path = dataset_dir / 'cycles.csv'
    before_path = dataset_dir / 'representative_before.csv'
    after_path = dataset_dir / 'representative_after.csv'
    meta_path = dataset_dir / 'metadata.json'

    for p in (cycles_path, before_path, after_path, meta_path):
        if not p.exists():
            print(f'error: {p} not found — run capture.py --motor-cal first', file=sys.stderr)
            return 1

    runs_df = pd.read_csv(cycles_path)
    before_df = pd.read_csv(before_path)
    after_df = pd.read_csv(after_path)
    meta = json.loads(meta_path.read_text())

    rep_cycle = meta['representative_cycle']
    rep_dir = meta['representative_direction']

    # Shared Y range
    vel_cols = [f'vel_{w}' for w in WHEELS]
    all_vals = pd.concat([
        before_df[vel_cols].abs().stack(),
        after_df[vel_cols].abs().stack(),
    ]).dropna()
    if len(all_vals) == 0:
        print('ERROR: no valid velocity data in representative run', file=sys.stderr)
        return 1
    lo = float(np.nanpercentile(all_vals, 2))
    hi = float(np.nanpercentile(all_vals, 98))
    pad = max(0.3, (hi - lo) * 0.35)
    y_range = (lo - pad, hi + pad)

    # Figure layout
    fig = plt.figure(figsize=(9.0, 6.2))
    gs = fig.add_gridspec(2, 2, height_ratios=[1.0, 1.1], hspace=0.48, wspace=0.12)
    ax_before = fig.add_subplot(gs[0, 0])
    ax_after = fig.add_subplot(gs[0, 1], sharey=ax_before)
    ax_sum = fig.add_subplot(gs[1, :])

    plot_time_series(ax_before, before_df,
                     f'Before calibration \u2014 run {rep_cycle} ({rep_dir})',
                     y_range)
    plot_time_series(ax_after, after_df,
                     f'After calibration \u2014 run {rep_cycle} ({rep_dir})',
                     y_range)
    plot_per_run_spread(ax_sum, runs_df)

    fig.suptitle(f"Motor gain calibration repeatability over N={len(runs_df)} cycles at PWM {meta['pwm']}",
                 fontsize=11)
    fig.tight_layout(rect=[0, 0, 1, 0.96])

    args.out.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(args.out, bbox_inches='tight')
    print(f'Wrote {args.out}')

    if args.also_png:
        png = args.out.with_suffix('.png')
        fig.savefig(png, dpi=200, bbox_inches='tight')
        print(f'Wrote {png}')

    plt.close(fig)

    # Summary JSON
    summary = {
        'pwm': meta['pwm'],
        'window_ms_effective': meta['effective_window_ms'],
        'run_count': len(runs_df),
        'representative_run_index': rep_cycle,
        'aggregate': {
            'before_spread_pct': {
                'mean': float(runs_df['before_spread_pct'].mean()),
                'std':  float(runs_df['before_spread_pct'].std()),
                'min':  float(runs_df['before_spread_pct'].min()),
                'max':  float(runs_df['before_spread_pct'].max()),
            },
            'after_spread_pct': {
                'mean': float(runs_df['after_spread_pct'].mean()),
                'std':  float(runs_df['after_spread_pct'].std()),
                'min':  float(runs_df['after_spread_pct'].min()),
                'max':  float(runs_df['after_spread_pct'].max()),
            },
            'improvement_ratio': {
                'mean': float(runs_df['improvement_ratio'].mean()),
                'std':  float(runs_df['improvement_ratio'].std()),
            },
        },
        'per_run': runs_df.to_dict(orient='records'),
    }
    summary_path = args.out.with_suffix('.summary.json')
    summary_path.write_text(json.dumps(summary, indent=2) + '\n')
    print(f'Wrote {summary_path}')

    # Console summary
    print(f'\n=== Aggregate across N={len(runs_df)} cycles ===')
    print(f"BEFORE spread (%):  {runs_df['before_spread_pct'].mean():.3f} \u00b1 {runs_df['before_spread_pct'].std():.3f}")
    print(f"AFTER  spread (%):  {runs_df['after_spread_pct'].mean():.3f} \u00b1 {runs_df['after_spread_pct'].std():.3f}")
    print(f"Improvement ratio:  {runs_df['improvement_ratio'].mean():.3f} \u00b1 {runs_df['improvement_ratio'].std():.3f}")

    return 0


if __name__ == '__main__':
    raise SystemExit(main())
