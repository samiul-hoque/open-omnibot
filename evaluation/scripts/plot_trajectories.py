#!/usr/bin/env python3
"""Generate thesis trajectory figures and tables from clean extracted data.

Stage 2 of the thesis data pipeline.

Usage:
    python3 plot_trajectories.py [--data <data_dir>] [--out <thesis_dir>]

Reads thesis_datasets.json to determine which dated collection folder to
use for each trajectory type (falls back to the latest folder).  Override
a specific dataset on the CLI:

    python3 plot_trajectories.py --use straight_2m=2026-04-15T08-15

Expects the directory layout produced by capture.py:

    evaluation/data/
      thesis_datasets.json          pinned dataset selections
      straight_2m/
        2026-04-15T08-15/           dated collection folder
          metrics.csv
          trajectories/rep01.csv ...
      ...
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import sys
from pathlib import Path

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


# ---------------------------------------------------------------------------
# Trajectory metadata
# ---------------------------------------------------------------------------

TRAJ_IDS = ['straight_2m', 'circle_0_5m_strafe', 'square_0m8_rotate', 'square_0m8_strafe']
# NOTE: `yaw_roundtrip` from the trajectory catalog is INTENTIONALLY absent —
# it's a debug trajectory (per the description in trajectories.js) used
# to investigate yaw symmetry during platform bring-up, not a thesis
# metric. If you add it here, also add a per-trajectory row to
# TRAJ_KIND / TRAJ_LABEL / TRAJ_AXIS_LIMITS / _draw_reference below,
# otherwise plot_trajectory_overlay will raise a KeyError.

TRAJ_LABEL = {
    'straight_2m':          'Straight (2 m)',
    'circle_0_5m_strafe':   'Circle (R=0.5 m)',
    'square_0m8_rotate':    'Square rotate (0.8 m)',
    'square_0m8_strafe':    'Square strafe (0.8 m)',
}

TRAJ_KIND = {
    'straight_2m':          'straight',
    'circle_0_5m_strafe':   'loop',
    'square_0m8_rotate':    'loop',
    'square_0m8_strafe':    'loop',
}

METHODS = ['Encoder-only', 'IMU-only', 'Complementary', 'BNO055 Fusion']

METHOD_STYLE = {
    'Encoder-only':    ('tab:blue',    '-'),
    'IMU-only':        ('tab:red',     '--'),
    'Complementary':   ('tab:green',   '-'),
    'BNO055 Fusion':   ('tab:orange',  '-.'),
}

# Column name prefix → method name mapping for trajectory CSVs
COL_PREFIX_METHOD = {
    'enc': 'Encoder-only',
    'imu': 'IMU-only',
    'comp': 'Complementary',
    'bno': 'BNO055 Fusion',
}

TRAJ_AXIS_LIMITS = {
    'straight_2m':         ((-0.15, 2.15), (-0.6, 0.6)),
    'circle_0_5m_strafe':  ((-0.15, 1.15), (-0.65, 0.65)),
    'square_0m8_rotate':   ((-0.2, 1.0), (-0.2, 1.0)),
    'square_0m8_strafe':   ((-0.2, 1.0), (-1.0, 0.2)),
}

OVERLAY_FIGSIZE = (6.5, 3.5)


# ---------------------------------------------------------------------------
# Dataset resolution
# ---------------------------------------------------------------------------

def resolve_dataset_dir(data_dir: Path, traj_id: str,
                        config: dict, overrides: dict) -> Path | None:
    """Find the dated collection folder for a trajectory type.

    Priority: CLI override > thesis_datasets.json > latest folder.
    """
    traj_root = data_dir / traj_id
    if not traj_root.is_dir():
        return None

    # CLI override
    if traj_id in overrides:
        candidate = traj_root / overrides[traj_id]
        if candidate.is_dir():
            return candidate
        print(f'  WARN: override {traj_id}={overrides[traj_id]} not found', file=sys.stderr)

    # thesis_datasets.json
    if traj_id in config:
        candidate = traj_root / config[traj_id]
        if candidate.is_dir():
            return candidate
        print(f'  WARN: pinned dataset {traj_id}={config[traj_id]} not found', file=sys.stderr)

    # Fall back to latest (alphabetically last dated folder)
    dated = sorted([d for d in traj_root.iterdir() if d.is_dir()])
    return dated[-1] if dated else None


def load_thesis_config(data_dir: Path) -> dict:
    path = data_dir / 'thesis_datasets.json'
    if path.exists():
        return json.loads(path.read_text())
    return {}


def parse_use_overrides(use_args: list[str] | None) -> dict:
    """Parse --use key=value pairs."""
    if not use_args:
        return {}
    result = {}
    for item in use_args:
        if '=' not in item:
            print(f'  WARN: ignoring --use {item} (expected key=value)', file=sys.stderr)
            continue
        k, v = item.split('=', 1)
        result[k] = v
    return result


# ---------------------------------------------------------------------------
# Data loading from clean CSVs
# ---------------------------------------------------------------------------

def load_metrics(dataset_dir: Path) -> list[dict]:
    """Load metrics.csv → list of {rep, method, metric: value} dicts."""
    path = dataset_dir / 'metrics.csv'
    if not path.exists():
        return []
    rows = []
    with open(path, newline='') as f:
        reader = csv.DictReader(f)
        for row in reader:
            d = {'rep': int(row['rep']), 'method': row['method']}
            for k, v in row.items():
                if k not in ('rep', 'method'):
                    d[k] = float(v)
            rows.append(d)
    return rows


def load_trajectories(dataset_dir: Path) -> dict[str, list[np.ndarray]]:
    """Load trajectory CSVs → {method: [Nx2 array (x,y), ...]} per rep."""
    traj_dir = dataset_dir / 'trajectories'
    if not traj_dir.exists():
        return {}
    result: dict[str, list[np.ndarray]] = {m: [] for m in METHODS}
    for csv_path in sorted(traj_dir.glob('rep*.csv')):
        with open(csv_path, newline='') as f:
            reader = csv.DictReader(f)
            rows = list(reader)
        if not rows:
            continue
        for prefix, method in COL_PREFIX_METHOD.items():
            x_col, y_col = f'x_{prefix}', f'y_{prefix}'
            xs = np.array([float(r[x_col]) for r in rows])
            ys = np.array([float(r[y_col]) for r in rows])
            traj = np.column_stack([xs, ys])
            result[method].append(traj)
    return result


# ---------------------------------------------------------------------------
# Aggregation
# ---------------------------------------------------------------------------

def aggregate_metrics(rows: list[dict]) -> dict[str, dict[str, tuple[float, float]]]:
    """Group by method → {metric: (mean, std)}."""
    from collections import defaultdict
    by_method: dict[str, list[dict]] = defaultdict(list)
    for r in rows:
        by_method[r['method']].append(r)
    result = {}
    for method, entries in by_method.items():
        keys = [k for k in entries[0] if k not in ('rep', 'method')]
        result[method] = {
            k: (float(np.mean([e[k] for e in entries])),
                float(np.std([e[k] for e in entries], ddof=1 if len(entries) > 1 else 0)))
            for k in keys
        }
    return result


# ---------------------------------------------------------------------------
# Plots
# ---------------------------------------------------------------------------

def _draw_reference(ax, trajectory_id: str) -> None:
    if trajectory_id == 'straight_2m':
        ax.plot([0, 2], [0, 0], 'k:', linewidth=1.2, label='Commanded')
    elif trajectory_id == 'circle_0_5m_strafe':
        # Body-frame circle: centre (+0.5, 0), radius 0.5, traversed CW
        # starting at the west pole (0, 0). This matches the integration of
        # trajectories.js `quarterArcSegments` from (0, 0, 0): with initial
        # velocity (sin(dθ/2), cos(dθ/2))·speed ≈ (0, speed) (i.e. body-left
        # at start), the robot swings around centre (+0.5, 0) clockwise.
        # Previous parametrisation drew the same circle in the opposite
        # direction — shape identical on a dotted overlay, but inconsistent
        # if anyone ever adds direction arrows or animation.
        t = np.linspace(0, 2 * math.pi, 200)
        ax.plot(0.5 - 0.5 * np.cos(t), 0.5 * np.sin(t),
                'k:', linewidth=1.2, label='Commanded')
    elif trajectory_id == 'square_0m8_rotate':
        ax.plot([0, 0.8, 0.8, 0, 0], [0, 0, 0.8, 0.8, 0], 'k:', linewidth=1.2, label='Commanded')
    elif trajectory_id == 'square_0m8_strafe':
        ax.plot([0, 0.8, 0.8, 0, 0], [0, 0, -0.8, -0.8, 0], 'k:', linewidth=1.2, label='Commanded')


def plot_trajectory_overlay(traj_id: str, per_method: dict[str, list[np.ndarray]], out_path: Path) -> None:
    fig, ax = plt.subplots(figsize=OVERLAY_FIGSIZE)
    for method, traj_list in per_method.items():
        if method == 'IMU-only' or not traj_list:
            continue
        color, ls = METHOD_STYLE.get(method, ('gray', '-'))
        for i, t in enumerate(traj_list):
            alpha = 1.0 if i == 0 else 0.25
            lw = 1.8 if i == 0 else 0.9
            label = method if i == 0 else None
            ax.plot(t[:, 0], t[:, 1], color=color, linestyle=ls, alpha=alpha, linewidth=lw, label=label)
    _draw_reference(ax, traj_id)
    xlim, ylim = TRAJ_AXIS_LIMITS.get(traj_id, (None, None))
    if xlim is not None:
        ax.set_xlim(*xlim)
    if ylim is not None:
        ax.set_ylim(*ylim)
    ax.set_xlabel('X (m)', fontsize=10)
    ax.set_ylabel('Y (m)', fontsize=10)
    ax.set_aspect('equal', adjustable='box')
    ax.tick_params(labelsize=9)
    ax.grid(True, linestyle=':', alpha=0.4)
    ax.legend(loc='best', fontsize=8, framealpha=0.9)
    ax.set_title(TRAJ_LABEL[traj_id], fontsize=11)
    fig.tight_layout()
    fig.savefig(out_path)
    plt.close(fig)


def plot_summary_bar(agg_all: dict[str, dict], out_path: Path) -> None:
    methods = [m for m in METHODS if m != 'IMU-only']
    width = 0.26
    x = np.arange(len(TRAJ_IDS))
    fig, ax = plt.subplots(figsize=(6.5, 3.5))
    offsets = np.linspace(-(len(methods) - 1) / 2, (len(methods) - 1) / 2, len(methods))
    for i, method in enumerate(methods):
        means, stds = [], []
        for traj_id in TRAJ_IDS:
            m = agg_all.get(traj_id, {}).get(method, {})
            if not m:
                means.append(np.nan)
                stds.append(0)
                continue
            key = 'pos_err_cm' if TRAJ_KIND[traj_id] == 'straight' else 'loop_err_cm'
            mean, std = m[key]
            means.append(abs(mean))
            stds.append(std)
        color, _ = METHOD_STYLE.get(method, ('gray', '-'))
        ax.bar(x + offsets[i] * width, means, width, yerr=stds, label=method, color=color, capsize=2)
    ax.set_xticks(x)
    ax.set_xticklabels([TRAJ_LABEL[t] for t in TRAJ_IDS], fontsize=9)
    ax.set_ylabel('Error (cm)', fontsize=10)
    ax.tick_params(labelsize=9)
    ax.grid(True, axis='y', linestyle=':', alpha=0.5)
    ax.legend(loc='upper left', fontsize=8, framealpha=0.9)
    ax.set_title('Primary error metric per trajectory and method', fontsize=10)
    fig.tight_layout()
    fig.savefig(out_path)
    plt.close(fig)


# ---------------------------------------------------------------------------
# LaTeX tables
# ---------------------------------------------------------------------------

def _fmt(mean: float, std: float, digits: int = 2) -> str:
    return f'${mean:.{digits}f} \\pm {std:.{digits}f}$'


def write_straight_table(agg: dict, out_path: Path, n_runs: int) -> None:
    rows = []
    for method in METHODS:
        m = agg.get(method, {})
        if not m:
            rows.append(f'{method} & -- & -- & -- & -- \\\\')
            continue
        rows.append(' & '.join([
            method,
            _fmt(*m['forward_err_cm']),
            _fmt(*m['lateral_drift_cm']),
            _fmt(*m['heading_err_deg']),
            _fmt(*m['pos_err_cm']),
        ]) + ' \\\\')
    body = '\n'.join(rows)
    tex = (
        '% AUTOGENERATED by evaluation/scripts/plot_trajectories.py\n'
        '\\begin{table}[h]\n\\centering\n'
        '\\resizebox{\\textwidth}{!}{%\n'
        '\\begin{tabular}{|l|c|c|c|c|}\n\\hline\n'
        '\\textbf{Method} & \\textbf{Forward err (cm)} & \\textbf{Lateral drift (cm)} & \\textbf{Heading err (\\textdegree)} & \\textbf{Pos err (cm)} \\\\\n\\hline\n'
        f'{body}\n'
        '\\hline\n\\end{tabular}}\n'
        f'\\caption{{Straight-line test results (2~m, mean $\\pm$ std over $N={n_runs}$ runs).}}\n'
        '\\label{tab:straight_results}\n\\end{table}\n'
    )
    out_path.write_text(tex)


def write_loop_table(agg: dict, out_path: Path, caption: str, label: str, n_runs: int) -> None:
    rows = []
    for method in METHODS:
        m = agg.get(method, {})
        if not m:
            rows.append(f'{method} & -- & -- & -- & -- \\\\')
            continue
        rows.append(' & '.join([
            method,
            _fmt(*m['loop_err_cm']),
            _fmt(*m['max_dev_cm']),
            _fmt(*m['heading_err_deg']),
            _fmt(*m['drift_pct']),
        ]) + ' \\\\')
    body = '\n'.join(rows)
    tex = (
        '% AUTOGENERATED by evaluation/scripts/plot_trajectories.py\n'
        '\\begin{table}[h]\n\\centering\n'
        '\\resizebox{\\textwidth}{!}{%\n'
        '\\begin{tabular}{|l|c|c|c|c|}\n\\hline\n'
        '\\textbf{Method} & \\textbf{Loop closure (cm)} & \\textbf{Max deviation (cm)} & \\textbf{Heading err (\\textdegree)} & \\textbf{Drift (\\%)} \\\\\n\\hline\n'
        f'{body}\n'
        '\\hline\n\\end{tabular}}\n'
        f'\\caption{{{caption} Mean $\\pm$ std over $N={n_runs}$ runs.}}\n'
        f'\\label{{{label}}}\n\\end{{table}}\n'
    )
    out_path.write_text(tex)


def write_summary_table(agg_all: dict, out_path: Path) -> None:
    lines = []
    for method in METHODS:
        row_entries = [method]
        for traj_id in TRAJ_IDS:
            m = agg_all.get(traj_id, {}).get(method, {})
            if not m:
                row_entries.append('--')
                continue
            if TRAJ_KIND[traj_id] == 'straight':
                mean, std = m['pos_err_cm']
            else:
                mean, std = m['loop_err_cm']
            row_entries.append(_fmt(mean, std))
        lines.append(' & '.join(row_entries) + ' \\\\')
    body = '\n'.join(lines)
    header = ' & '.join(['\\textbf{Method}'] + [f'\\textbf{{{TRAJ_LABEL[t]}}}' for t in TRAJ_IDS]) + ' \\\\'
    cols = '|l|' + '|'.join(['c'] * len(TRAJ_IDS)) + '|'
    tex = (
        '% AUTOGENERATED by evaluation/scripts/plot_trajectories.py\n'
        '\\begin{table}[h]\n\\centering\n'
        '\\resizebox{\\textwidth}{!}{%\n'
        f'\\begin{{tabular}}{{{cols}}}\n\\hline\n'
        f'{header}\n\\hline\n{body}\n\\hline\n'
        '\\end{tabular}}\n'
        '\\caption{Primary error metric per trajectory across methods: position error for the straight line, loop-closure error for the closed-path trajectories. All values in cm, mean $\\pm$ std.}\n'
        '\\label{tab:results_summary}\n\\end{table}\n'
    )
    out_path.write_text(tex)


# ---------------------------------------------------------------------------
# Driver
# ---------------------------------------------------------------------------

def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--data', type=Path, default=Path('evaluation/data'),
                        help='Clean data directory (default: evaluation/data)')
    parser.add_argument('--out', type=Path, default=Path('output'),
                        help='Output directory for generated tables and figures (default: output/)')
    parser.add_argument('--use', nargs='*', metavar='TRAJ=DATE',
                        help='Override dataset selection, e.g. --use straight_2m=2026-04-15T08-15')
    args = parser.parse_args(argv)

    tables_dir = args.out / 'chapters' / 'generated'
    figs_dir = args.out / 'figures' / 'results'
    tables_dir.mkdir(parents=True, exist_ok=True)
    figs_dir.mkdir(parents=True, exist_ok=True)

    config = load_thesis_config(args.data)
    overrides = parse_use_overrides(args.use)

    agg_all: dict[str, dict] = {}
    run_counts: dict[str, int] = {}

    print('Dataset selection:')
    for traj_id in TRAJ_IDS:
        dataset_dir = resolve_dataset_dir(args.data, traj_id, config, overrides)
        if dataset_dir is None:
            print(f'  {traj_id:<24} — no data')
            continue
        print(f'  {traj_id:<24} <- {dataset_dir.name}')

        metrics = load_metrics(dataset_dir)
        if not metrics:
            print(f'    skip: no metrics.csv')
            continue
        n_runs = len(set(r['rep'] for r in metrics))
        run_counts[traj_id] = n_runs
        agg = aggregate_metrics(metrics)
        agg_all[traj_id] = agg

        # Overlay plot
        trajs = load_trajectories(dataset_dir)
        if trajs:
            trimmed = {m: ts[:3] for m, ts in trajs.items()}
            pdf = figs_dir / f'{traj_id}_overlay.pdf'
            plot_trajectory_overlay(traj_id, trimmed, pdf)
            print(f'    plot -> {pdf}')

    if not agg_all:
        print('error: no data found', file=sys.stderr)
        return 1

    # Tables
    if 'straight_2m' in agg_all:
        write_straight_table(agg_all['straight_2m'], tables_dir / 'table_straight.tex', run_counts['straight_2m'])
    if 'circle_0_5m_strafe' in agg_all:
        write_loop_table(agg_all['circle_0_5m_strafe'], tables_dir / 'table_circular.tex',
                         'Circular path test results (0.5~m radius, one revolution).',
                         'tab:circular_results', run_counts['circle_0_5m_strafe'])
    if 'square_0m8_rotate' in agg_all:
        write_loop_table(agg_all['square_0m8_rotate'], tables_dir / 'table_square_rotate.tex',
                         'Square path (rotate-at-corners variant, 0.8~m sides) results.',
                         'tab:square_rotate_results', run_counts['square_0m8_rotate'])
    if 'square_0m8_strafe' in agg_all:
        write_loop_table(agg_all['square_0m8_strafe'], tables_dir / 'table_square_strafe.tex',
                         'Square path (holonomic-strafe variant, 0.8~m sides) results.',
                         'tab:square_strafe_results', run_counts['square_0m8_strafe'])
    write_summary_table(agg_all, tables_dir / 'table_summary.tex')

    # Summary bar chart
    plot_summary_bar(agg_all, figs_dir / 'comparison_bar_chart.pdf')
    print(f'  plot -> {figs_dir / "comparison_bar_chart.pdf"}')

    # Console summary
    print(f'\nPer-trajectory run counts:')
    for tid, n in run_counts.items():
        print(f'  {tid:<24} {n} run(s)')

    print('\nAggregated metrics:')
    for traj_id in TRAJ_IDS:
        agg = agg_all.get(traj_id)
        if not agg:
            continue
        print(f'\n  {TRAJ_LABEL[traj_id]} (N = {run_counts[traj_id]}):')
        for method in METHODS:
            m = agg.get(method)
            if not m:
                continue
            bits = ', '.join(f'{k}={v[0]:.2f}\u00b1{v[1]:.2f}' for k, v in m.items())
            print(f'    {method:<16} {bits}')

    print('\nDone. Tables + figures written.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
