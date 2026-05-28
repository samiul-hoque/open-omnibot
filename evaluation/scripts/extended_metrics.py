#!/usr/bin/env python3
"""Compute extended evaluation metrics from the frozen thesis campaign.

Reads every `.meta.json` under
`data/runs/`, groups runs by
(trajectory, tier), and computes per-cell:

  - Position RMSE (cm)       : sqrt(mean(pos_err^2)) over reps
  - Max Error (cm)           : max(pos_err) over reps
  - Heading RMSE (deg)       : sqrt(mean(heading_err^2)) over reps
  - Heading Drift Rate (deg/m): mean(|heading_err|) / path_length
  - Drift Over Distance (%)  : mean(pos_err) / path_length * 100

Position error is computed in the camera-world frame:
  - Straight: Euclidean distance between the parallax-corrected
    measured end pose and the expected end pose (start + 2 m along
    start heading). The raw camera-world delta is divided by
    k = 1.077 (marker-height parallax factor from Ch. 5) before the
    error is computed; the raw values match those reported in the
    Ch. 7 straight-line narrative.
  - Loops (circle, square): closure error = distance between
    measured end pose and measured start pose. No parallax
    correction is applied: the bias cancels when start and end are
    co-located (stated in Ch. 7 Table summary note).

Heading error is `wrap(gt_theta - start_theta)` in both cases
(trajectories end in the same orientation they started in).

Writes a LaTeX table to stdout and, if given `--out <path>`, to that
path as a standalone .tex file.
"""

from __future__ import annotations

import argparse
import json
import math
from collections import defaultdict
from pathlib import Path

# ---------------------------------------------------------------------------
# Config
# ---------------------------------------------------------------------------

TRAJ_ORDER = ['straight_2m', 'circle_0_5m_strafe',
              'square_0m8_rotate', 'square_0m8_strafe']

TRAJ_LABEL = {
    'straight_2m':         'Straight',
    'circle_0_5m_strafe':  'Circle',
    'square_0m8_rotate':   'Sq.\\ rotate',
    'square_0m8_strafe':   'Sq.\\ strafe',
}

TRAJ_KIND = {
    'straight_2m':        'straight',
    'circle_0_5m_strafe': 'loop',
    'square_0m8_rotate':  'loop',
    'square_0m8_strafe':  'loop',
}

TRAJ_PATH_LENGTH_M = {
    'straight_2m':        2.0,
    'circle_0_5m_strafe': 2 * math.pi * 0.5,   # ~3.142
    'square_0m8_rotate':  4 * 0.8,             # 3.2
    'square_0m8_strafe':  4 * 0.8,             # 3.2
}

# Marker-height parallax factor (Ch. 5). Applied to the raw
# camera-world displacement of the straight-line trajectory because
# its start and end are not co-located, so the bias does not cancel.
PARALLAX_K = 1.077

TIER_LABEL = {
    0: 'T0',
    1: 'T1',
    2: 'T2',
}


# ---------------------------------------------------------------------------
# Math helpers
# ---------------------------------------------------------------------------

def wrap_deg(x: float) -> float:
    """Wrap degrees into (-180, 180]."""
    x = (x + 180.0) % 360.0 - 180.0
    if x == -180.0:
        x = 180.0
    return x


def rmse(vals):
    return math.sqrt(sum(v * v for v in vals) / len(vals))


def mean(vals):
    return sum(vals) / len(vals)


# ---------------------------------------------------------------------------
# Per-run metrics
# ---------------------------------------------------------------------------

def run_errors(meta: dict):
    """Return (pos_err_m, heading_err_deg) for a single run."""
    exp = meta['experiment']
    traj = exp['trajectory']
    start = exp['measuredStartPose']
    gt = exp['groundTruth']

    sx, sy, sth = start['x'], start['y'], start['thetaDeg']
    ex, ey, eth = gt['xMeas'], gt['yMeas'], gt['thetaDegMeas']

    if TRAJ_KIND[traj] == 'loop':
        exp_x, exp_y = sx, sy
        # Loops: closure error, no parallax correction (bias cancels).
        pos_err = math.hypot(ex - exp_x, ey - exp_y)
    else:
        # Straight: 2 m along the starting heading, with the raw
        # camera-world delta scaled by 1/k before comparison.
        dist = 2.0
        exp_x = sx + dist * math.cos(math.radians(sth))
        exp_y = sy + dist * math.sin(math.radians(sth))
        corr_ex = sx + (ex - sx) / PARALLAX_K
        corr_ey = sy + (ey - sy) / PARALLAX_K
        pos_err = math.hypot(corr_ex - exp_x, corr_ey - exp_y)

    heading_err = wrap_deg(eth - sth)
    return pos_err, heading_err


# ---------------------------------------------------------------------------
# Aggregation
# ---------------------------------------------------------------------------

def aggregate(runs_dir: Path):
    """Load all meta.json, group by (traj, tier), compute metrics."""
    by_cell = defaultdict(list)
    for p in sorted(runs_dir.glob('*.meta.json')):
        meta = json.loads(p.read_text())
        traj = meta['experiment']['trajectory']
        tier = meta['experiment']['tier']
        by_cell[(traj, tier)].append(run_errors(meta))

    results = {}
    for (traj, tier), errs in by_cell.items():
        pos_errs = [e[0] for e in errs]
        head_errs = [e[1] for e in errs]
        abs_head_errs = [abs(h) for h in head_errs]
        pl = TRAJ_PATH_LENGTH_M[traj]
        results[(traj, tier)] = {
            'n': len(errs),
            'pos_rmse_cm':   rmse(pos_errs) * 100.0,
            'pos_max_cm':    max(pos_errs) * 100.0,
            'pos_mean_cm':   mean(pos_errs) * 100.0,
            'head_rmse_deg': rmse(head_errs),
            'head_drift_rate_deg_per_m': mean(abs_head_errs) / pl,
            'drift_pct':     mean(pos_errs) / pl * 100.0,
        }
    return results


# ---------------------------------------------------------------------------
# LaTeX output
# ---------------------------------------------------------------------------

def format_cell(v, digits=2):
    return f'{v:.{digits}f}'


def render_latex_table(results: dict) -> str:
    """One row per (trajectory, tier) cell."""
    lines = [
        '% AUTOGENERATED by evaluation/scripts/extended_metrics.py',
        '\\begin{table}[htbp]',
        '\\centering',
        '\\caption{Extended error metrics per trajectory and tier '
        '($N=3$ reps per cell). Tiers: T0 open-loop, T1 encoder '
        'odometry, T2 complementary filter. Position errors in cm, '
        'heading errors in degrees, drift as \\% of path length.}',
        '\\label{tab:extended_metrics}',
        '\\begin{tabular}{llrrrrr}',
        '\\toprule',
        '\\textbf{Trajectory} & \\textbf{Tier} & '
        '\\textbf{RMSE} & \\textbf{Max} & '
        '\\textbf{Hdg RMSE} & \\textbf{Hdg drift} & '
        '\\textbf{Drift} \\\\',
        ' & & (cm) & (cm) & (\\textdegree) & (\\textdegree/m) & (\\%) \\\\',
        '\\midrule',
    ]
    for traj_idx, traj in enumerate(TRAJ_ORDER):
        first = True
        for tier in (0, 1, 2):
            r = results.get((traj, tier))
            if r is None:
                continue
            traj_cell = TRAJ_LABEL[traj] if first else ''
            first = False
            lines.append(
                f'{traj_cell} & {TIER_LABEL[tier]} & '
                f'{format_cell(r["pos_rmse_cm"])} & '
                f'{format_cell(r["pos_max_cm"])} & '
                f'{format_cell(r["head_rmse_deg"])} & '
                f'{format_cell(r["head_drift_rate_deg_per_m"])} & '
                f'{format_cell(r["drift_pct"])} \\\\'
            )
        if traj_idx < len(TRAJ_ORDER) - 1:
            lines.append('\\addlinespace')
    lines += ['\\bottomrule', '\\end{tabular}', '\\end{table}', '']
    return '\n'.join(lines)


# ---------------------------------------------------------------------------
# Driver
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--runs', type=Path,
                        default=Path('data/runs'),
                        help='Directory holding exp_*.meta.json files')
    parser.add_argument('--out', type=Path, default=None,
                        help='Optional output path for LaTeX table')
    args = parser.parse_args()

    if not args.runs.is_dir():
        raise SystemExit(f'runs dir not found: {args.runs}')

    results = aggregate(args.runs)

    # Console summary
    print('Per-cell extended metrics:')
    print(f'{"trajectory":<22} {"tier":<20} {"N":>2}  '
          f'{"RMSE(cm)":>9} {"Max(cm)":>8} {"HdgRMSE":>8} '
          f'{"HdgDrift":>9} {"Drift%":>7}')
    for traj in TRAJ_ORDER:
        for tier in (0, 1, 2):
            r = results.get((traj, tier))
            if r is None:
                continue
            print(f'{traj:<22} {TIER_LABEL[tier]:<20} {r["n"]:>2}  '
                  f'{r["pos_rmse_cm"]:>9.2f} {r["pos_max_cm"]:>8.2f} '
                  f'{r["head_rmse_deg"]:>8.2f} '
                  f'{r["head_drift_rate_deg_per_m"]:>9.2f} '
                  f'{r["drift_pct"]:>7.2f}')

    tex = render_latex_table(results)
    print('\n' + tex)
    if args.out:
        args.out.parent.mkdir(parents=True, exist_ok=True)
        args.out.write_text(tex)
        print(f'wrote {args.out}')


if __name__ == '__main__':
    main()
