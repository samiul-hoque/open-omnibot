"""Fetch a frame from the overhead camera and overlay world-frame labels
using the current calibration homography. For workspace-bounds verification
after a grid edit — run after a successful recalibration so the pixel →
world mapping matches the physical grid.

Default mode draws solid yellow discs at each trajectory's start pose (for
initial tape placement). `--verify` draws hollow rings instead, so existing
yellow tape underneath is visible and alignment is easy to eyeball.

Output: /tmp/grid_labeled.jpg

MANUAL-SYNC WARNING
-------------------
Several constants here (workspace bounds, 0.8 m square corners, circle
centre/radius, per-trajectory startHints) are hard-coded to match the
authoritative runtime catalogue in
`server/src/experiments/trajectories.js`. Python and JS don't share a
module system, so if you edit one file you MUST manually edit the
other. Cross-file invariants are enumerated at each constant below so a
grep like `grep -rn "startHint" evaluation/ server/` surfaces every
site that has to move together.

If this drifts often enough to be painful, generate a small JSON
manifest from `trajectories.js` (e.g.\\ via
`node server/src/experiments/dump_startHints.js`) and load it here
instead — but resist doing that preemptively, because the constants
change only when the physical grid does, which is once every few
sessions.
"""
from __future__ import annotations

import argparse
import json
import math
import os
import sys
import urllib.request
from pathlib import Path

import cv2
import numpy as np

CALIB = Path(__file__).parent / 'calibration.json'
STILL_URL = 'http://<camera-host>:8080/video/jpeg'
OUT = Path('/tmp/grid_labeled.jpg')


def fetch_still() -> np.ndarray:
    # Pull a single JPEG from the CamON still-image endpoint. Fail with a
    # friendly message (and non-zero exit) if the phone is off / app
    # isn't streaming / network has dropped — the raw urllib.error
    # traceback that used to come out was not helpful for operators who
    # just forgot to launch CamON.
    try:
        with urllib.request.urlopen(STILL_URL, timeout=5) as r:
            data = r.read()
    except (urllib.error.URLError, TimeoutError, OSError) as e:
        print(
            f'FAIL: could not reach CamON at {STILL_URL}:\n  {e}\n'
            'Is the phone on, charged, on the workspace WiFi, and '
            'running CamON Live with streaming enabled? Check http://'
            + STILL_URL.split("/")[2] + '/status in a browser to verify.',
            file=sys.stderr,
        )
        sys.exit(1)
    arr = np.frombuffer(data, dtype=np.uint8)
    img = cv2.imdecode(arr, cv2.IMREAD_COLOR)
    if img is None:
        print(
            f'FAIL: CamON returned {len(data)} bytes but cv2 could not '
            'decode them as JPEG. Most likely CamON sent an HTML error '
            f'page. Check {STILL_URL} in a browser.',
            file=sys.stderr,
        )
        sys.exit(1)
    return img


def world_to_px(H_w2p: np.ndarray, wx: float, wy: float) -> tuple[int, int]:
    p = cv2.perspectiveTransform(np.array([[[wx, wy]]], dtype=np.float32), H_w2p).reshape(2)
    return int(round(p[0])), int(round(p[1]))


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--verify', action='store_true',
                    help='Draw hollow rings at start poses instead of filled '
                         'discs so existing yellow tape is visible through them')
    args = ap.parse_args()
    if not CALIB.exists():
        print('FAIL: no calibration.json')
        sys.exit(1)
    cal = json.loads(CALIB.read_text())
    H_p2w = np.array(cal['homography_px_to_m'], dtype=np.float64)
    H_w2p = np.linalg.inv(H_p2w)

    img = fetch_still()
    H_img, W_img = img.shape[:2]
    vis = img.copy()

    # Decimetre grid overlay across the full detected workspace.
    # Sweep in 10 cm steps along X and Y, only drawing labels where the
    # projected pixel lands inside the frame.
    for xi in range(-20, 21):   # ±2.0 m in 10 cm steps
        for yi in range(-10, 11):  # ±1.0 m in 10 cm steps
            wx = xi * 0.10
            wy = yi * 0.10
            px, py = world_to_px(H_w2p, wx, wy)
            if 0 <= px < W_img and 0 <= py < H_img:
                # Bright green on integer-metre intersections, faint on others.
                if xi % 10 == 0 and yi % 10 == 0:
                    cv2.circle(vis, (px, py), 6, (0, 255, 0), -1)
                    label = f'({wx:+.0f}, {wy:+.0f})'
                    cv2.putText(vis, label, (px + 8, py - 8),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1, cv2.LINE_AA)
                else:
                    cv2.circle(vis, (px, py), 2, (0, 180, 180), -1)

    # World origin crosshair + axis arrows.
    ox, oy = world_to_px(H_w2p, 0, 0)
    # +X axis → 0.5 m long in world
    px_x, py_x = world_to_px(H_w2p, 0.5, 0)
    cv2.arrowedLine(vis, (ox, oy), (px_x, py_x), (0, 0, 255), 3, tipLength=0.05)
    cv2.putText(vis, '+X (0.5 m)', (px_x + 5, py_x + 5),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2, cv2.LINE_AA)
    # +Y axis → 0.5 m long in world
    px_y, py_y = world_to_px(H_w2p, 0, 0.5)
    cv2.arrowedLine(vis, (ox, oy), (px_y, py_y), (255, 100, 0), 3, tipLength=0.05)
    cv2.putText(vis, '+Y (0.5 m)', (px_y + 5, py_y - 5),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 100, 0), 2, cv2.LINE_AA)
    cv2.drawMarker(vis, (ox, oy), (255, 255, 255), cv2.MARKER_CROSS, 30, 3)
    cv2.putText(vis, 'world (0,0)', (ox + 10, oy + 25),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2, cv2.LINE_AA)

    # Confirmed workspace (2026-04-19 rebuild): X ∈ [−1.30, +1.10],
    # Y ∈ [−0.50, +0.60]. Origin is offset (−0.10, +0.05) from the grid's
    # geometric mid-point.
    corners_world = [(-1.30, -0.50), (+1.10, -0.50),
                     (+1.10, +0.60), (-1.30, +0.60)]
    corners_px = [world_to_px(H_w2p, wx, wy) for wx, wy in corners_world]
    for i in range(4):
        cv2.line(vis, corners_px[i], corners_px[(i + 1) % 4],
                 (255, 0, 255), 2, cv2.LINE_AA)
    for (wx, wy), (px, py) in zip(corners_world, corners_px):
        cv2.circle(vis, (px, py), 8, (255, 0, 255), 2)
        cv2.putText(vis, f'({wx:+.2f}, {wy:+.2f})', (px + 10, py + 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 255), 2, cv2.LINE_AA)

    # 0.8 m square envelope (rotate + strafe share the same footprint),
    # shifted up by +0.05 m to sit at the Y centroid of the post-2026-04-19
    # workspace. Cyan outline so it doesn't fight the yellow tape markers.
    sq_world = [(+0.4, -0.35), (+0.4, +0.45), (-0.4, +0.45), (-0.4, -0.35)]
    sq_px = [world_to_px(H_w2p, wx, wy) for wx, wy in sq_world]
    for i in range(4):
        cv2.line(vis, sq_px[i], sq_px[(i + 1) % 4],
                 (200, 200, 0), 1, cv2.LINE_AA)

    # Yellow-tape placement markers for each experiment's start pose.
    # Each entry: (world_pos, heading_deg, label, label_offset_px).
    # Labels are offset per-point to avoid overlapping the grid or each other.
    TAPE_YELLOW = (0, 255, 255)      # BGR: pure yellow
    BLACK       = (0, 0, 0)
    START_POSES = [
        ((-1.00,  0.00),   0, 'straight_2m START',        (20,   -50)),
        (( 0.00, -0.40),  90, 'circle_0_5m_strafe START', (20,   -20)),
        ((+0.40, -0.35),  90, 'square_0m8_rotate START',  (20,    35)),
        ((-0.40, -0.35),  90, 'square_0m8_strafe START',  (-340, 35)),
    ]

    # Actual circle path (r=0.5 m, centred at (0, +0.10) per the updated
    # circle_0_5m_strafe definition). Draw 96 world points as a light-blue
    # polyline so the operator can see how close the arc gets to each edge.
    circle_pts_world = [(0.5 * math.cos(2 * math.pi * i / 96),
                         0.10 + 0.5 * math.sin(2 * math.pi * i / 96))
                        for i in range(97)]
    circle_pts_px = [world_to_px(H_w2p, wx, wy) for wx, wy in circle_pts_world]
    for i in range(len(circle_pts_px) - 1):
        cv2.line(vis, circle_pts_px[i], circle_pts_px[i + 1],
                 (255, 180, 60), 1, cv2.LINE_AA)
    for (wx, wy), heading_deg, label, (lx, ly) in START_POSES:
        cx, cy = world_to_px(H_w2p, wx, wy)
        # Heading arrow: 0.2 m forward in world along the heading.
        h = math.radians(heading_deg)
        hx, hy = world_to_px(H_w2p,
                             wx + 0.2 * math.cos(h),
                             wy + 0.2 * math.sin(h))
        # Filled yellow disc with black outline (tape-marker look) in
        # placement mode; hollow ring in --verify so user's actual tape is
        # visible through it.
        if args.verify:
            cv2.circle(vis, (cx, cy), 18, TAPE_YELLOW, 2, cv2.LINE_AA)
            cv2.circle(vis, (cx, cy), 20, BLACK,       1, cv2.LINE_AA)
            cv2.drawMarker(vis, (cx, cy), TAPE_YELLOW, cv2.MARKER_CROSS, 12, 2)
        else:
            cv2.circle(vis, (cx, cy), 16, TAPE_YELLOW, -1)
            cv2.circle(vis, (cx, cy), 16, BLACK, 2, cv2.LINE_AA)
        # Heading arrow — black for contrast against the yellow disc.
        cv2.arrowedLine(vis, (cx, cy), (hx, hy), BLACK, 3, tipLength=0.25)
        cv2.arrowedLine(vis, (cx, cy), (hx, hy), TAPE_YELLOW, 2, tipLength=0.25)
        # Text label with black shadow for legibility over the grid.
        text_pos = (cx + lx, cy + ly)
        text_full = f'{label}  ({wx:+.2f}, {wy:+.2f})'
        cv2.putText(vis, text_full, text_pos,
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, BLACK, 4, cv2.LINE_AA)
        cv2.putText(vis, text_full, text_pos,
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, TAPE_YELLOW, 2, cv2.LINE_AA)

    # Legend at top-left.
    legend = [
        ('green dot + label  = integer-metre grid intersection', (0, 255, 0)),
        ('white X at origin  = world (0, 0)', (255, 255, 255)),
        ('red arrow          = +X axis (0.5 m long)', (0, 0, 255)),
        ('blue arrow         = +Y axis (0.5 m long)', (255, 100, 0)),
        ('magenta rectangle  = confirmed workspace bounds', (255, 0, 255)),
        ('dim cyan square    = 0.8 m experiment envelope (corners at (±0.4, -0.35)/(±0.4, +0.45))', (200, 200, 0)),
        ('light-blue circle  = 0.5 m circle path, centred at (0, +0.10)', (255, 180, 60)),
        ('yellow discs       = yellow-tape placement markers + heading arrow (robot START)', (0, 255, 255)),
    ]
    y0 = 30
    for i, (txt, col) in enumerate(legend):
        cv2.putText(vis, txt, (15, y0 + i * 24),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, col, 2, cv2.LINE_AA)

    cv2.imwrite(str(OUT), vis, [cv2.IMWRITE_JPEG_QUALITY, 92])
    print(f'wrote {OUT}  ({W_img}x{H_img}, origin at px=({ox},{oy}))')


if __name__ == '__main__':
    main()
