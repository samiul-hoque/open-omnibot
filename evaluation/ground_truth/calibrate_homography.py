"""
Calibrate pixel -> world homography from a single snapshot of the floor grid.

Detects grid intersections via Hough lines, assigns each to the nearest
known 10 cm grid node using the current rough transform as a bootstrap,
then solves cv2.findHomography. Writes the result to
evaluation/ground_truth/calibration.json.

Run once after the camera is locked into place. If the camera moves, rerun.

    python evaluation/ground_truth/calibrate_homography.py
"""
from __future__ import annotations

import argparse
import json
import math
import os
import sys
import time
import urllib.error
import urllib.request
from pathlib import Path
from urllib.parse import urlparse

import cv2
import numpy as np

DEFAULTS_ORIGIN_PX = (1160, 539)
DEFAULTS_PX_PER_M = 830.0
DEFAULTS_TILT_DEG = 0.87
DEFAULT_STREAM = "rtsp://<camera-host>:8080/video/h264"
CELL_M = 0.10  # 10 cm grid spacing
OUT = Path(__file__).parent / "calibration.json"
AF_SETTLE_S = 2.0
AF_CONTROL_PORT = 8080


def trigger_autofocus(stream_url: str, settle_s: float = AF_SETTLE_S) -> None:
    """Ask CamON Live to autofocus on frame centre, then wait for AF to settle.

    Called before opening the RTSP capture — AF converges in ~1-2 s on this
    phone, and frames captured mid-hunt tank the Laplacian-variance pick.
    Soft-fails: a warning is printed if the POST doesn't land, and the
    caller proceeds with whatever focus the phone had.
    """
    host = urlparse(stream_url).hostname
    if not host:
        print(f"WARNING: cannot parse host from {stream_url}; skipping autofocus")
        return
    url = f"http://{host}:{AF_CONTROL_PORT}/control"
    req = urllib.request.Request(
        url, data=b"autofocus=start\n", method="POST",
        headers={"Content-Type": "text/plain"},
    )
    try:
        with urllib.request.urlopen(req, timeout=3.0) as resp:
            resp.read()
    except (urllib.error.URLError, TimeoutError, OSError) as e:
        print(f"WARNING: autofocus POST to {url} failed ({e}); continuing")
        return
    print(f"autofocus triggered on frame centre, settling {settle_s:.1f}s")
    time.sleep(settle_s)


def bootstrap_world(px: float, py: float, origin_px, px_per_m, tilt_deg):
    """Apply the rough linear model — same as detector's PixelToWorld."""
    dx = px - origin_px[0]
    dy = py - origin_px[1]
    t = -math.radians(tilt_deg)
    c, s = math.cos(t), math.sin(t)
    xr = c * dx - s * dy
    yr = s * dx + c * dy
    return xr / px_per_m, -yr / px_per_m


def detect_grid_intersections(frame, origin_px, px_per_m, min_dist_px=40):
    """Find grid intersections via HoughLinesP + line crossings.

    Returns Nx2 pixel coords of intersections inside the workspace region,
    deduplicated to at least `min_dist_px` apart.
    """
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    # Grid is dark (black tape) on light floor — threshold to get dark pixels
    _, binary = cv2.threshold(gray, 100, 255, cv2.THRESH_BINARY_INV)
    # Thin lines slightly to separate adjacent cells
    edges = cv2.Canny(gray, 50, 150)

    # Find line segments
    lines = cv2.HoughLinesP(edges, 1, np.pi / 360,
                            threshold=80, minLineLength=200, maxLineGap=15)
    if lines is None:
        raise RuntimeError("no grid lines found")

    # Split into horizontal (|slope|<~5°) vs vertical (>85°)
    h_lines = []
    v_lines = []
    for line in lines:
        x1, y1, x2, y2 = line[0]
        ang = math.degrees(math.atan2(y2 - y1, x2 - x1))
        a = ((ang + 90) % 180) - 90  # wrap to [-90,90]
        if abs(a) < 10:
            h_lines.append((x1, y1, x2, y2))
        elif abs(abs(a) - 90) < 10:
            v_lines.append((x1, y1, x2, y2))
    print(f"  found {len(h_lines)} horizontal + {len(v_lines)} vertical line segments")

    def line_abc(x1, y1, x2, y2):
        # Returns coefficients for ax + by + c = 0
        return (y2 - y1, x1 - x2, x2 * y1 - x1 * y2)

    # Cluster horizontal lines by their average Y; vertical by average X.
    # Since Hough returns many segments per true grid line, this collapses them.
    def cluster(lines, axis_y=True, tol=15):
        if not lines:
            return []
        # Compute representative coord for each line
        keys = [((y1 + y2) / 2 if axis_y else (x1 + x2) / 2,
                 x1, y1, x2, y2) for x1, y1, x2, y2 in lines]
        keys.sort()
        clusters = []
        cur = [keys[0]]
        for k in keys[1:]:
            if k[0] - cur[-1][0] < tol:
                cur.append(k)
            else:
                clusters.append(cur)
                cur = [k]
        clusters.append(cur)
        # Return one representative line per cluster (average)
        out = []
        for cl in clusters:
            xs1, ys1, xs2, ys2 = zip(*[(c[1], c[2], c[3], c[4]) for c in cl])
            out.append((float(np.mean(xs1)), float(np.mean(ys1)),
                        float(np.mean(xs2)), float(np.mean(ys2))))
        return out

    h_rep = cluster(h_lines, axis_y=True)
    v_rep = cluster(v_lines, axis_y=False)
    print(f"  after clustering: {len(h_rep)} H-lines, {len(v_rep)} V-lines")

    # Compute pairwise intersections
    intersections = []
    for hl in h_rep:
        a1, b1, c1 = line_abc(*hl)
        for vl in v_rep:
            a2, b2, c2 = line_abc(*vl)
            det = a1 * b2 - a2 * b1
            if abs(det) < 1e-6: continue
            x = (b1 * c2 - b2 * c1) / det
            y = (a2 * c1 - a1 * c2) / det
            if 0 <= x < frame.shape[1] and 0 <= y < frame.shape[0]:
                intersections.append((x, y))

    # Dedupe
    out = []
    for p in intersections:
        if all(math.hypot(p[0] - q[0], p[1] - q[1]) >= min_dist_px for q in out):
            out.append(p)
    return np.array(out, dtype=np.float32)


def snap_to_grid(intersections, origin_px, px_per_m, tilt_deg, max_err_m=0.03):
    """For each pixel intersection, bootstrap-world it, snap to nearest 10 cm
    node, and keep only those within `max_err_m` of the snapped node."""
    kept_px = []
    kept_world = []
    for px, py in intersections:
        wx, wy = bootstrap_world(px, py, origin_px, px_per_m, tilt_deg)
        sx = round(wx / CELL_M) * CELL_M
        sy = round(wy / CELL_M) * CELL_M
        err = math.hypot(wx - sx, wy - sy)
        if err <= max_err_m:
            kept_px.append((px, py))
            kept_world.append((sx, sy))
    return np.array(kept_px, dtype=np.float32), np.array(kept_world, dtype=np.float32)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--stream", default=DEFAULT_STREAM)
    ap.add_argument("--origin-x", type=float, default=DEFAULTS_ORIGIN_PX[0])
    ap.add_argument("--origin-y", type=float, default=DEFAULTS_ORIGIN_PX[1])
    ap.add_argument("--px-per-m", type=float, default=DEFAULTS_PX_PER_M)
    ap.add_argument("--tilt-deg", type=float, default=DEFAULTS_TILT_DEG)
    ap.add_argument("--frames", type=int, default=8,
                    help="average this many frames for sharpness")
    ap.add_argument("--save-visual", action="store_true",
                    help="also write an annotated calibration image")
    ap.add_argument("--no-autofocus", action="store_true",
                    help="skip the CamON autofocus trigger before capture")
    args = ap.parse_args()

    origin_px = (args.origin_x, args.origin_y)

    os.environ.setdefault("OPENCV_FFMPEG_CAPTURE_OPTIONS",
                          "rtsp_transport;tcp|allowed_media_types;video")

    if not args.no_autofocus:
        trigger_autofocus(args.stream)

    cap = cv2.VideoCapture(args.stream, cv2.CAP_FFMPEG)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    if not cap.isOpened():
        print(f"FAIL: cannot open {args.stream}"); sys.exit(1)

    # Grab a few frames, keep the sharpest
    best = None; best_s = -1
    t0 = time.time()
    while time.time() - t0 < 4.0:
        ok, f = cap.read()
        if not ok: continue
        s = cv2.Laplacian(cv2.cvtColor(f, cv2.COLOR_BGR2GRAY), cv2.CV_64F).var()
        if s > best_s: best_s = s; best = f
    cap.release()
    if best is None:
        print("FAIL: no frame captured"); sys.exit(1)
    H_px, W_px = best.shape[:2]
    print(f"captured {W_px}x{H_px}, sharpness={best_s:.0f}")

    # Soft sharpness floor. Typical values on this rig with a good AF
    # lock are 350–500 (Laplacian variance on a 2336×1080 grid frame).
    # Below ~200 we consistently see a p50 reprojection of >20 mm and
    # the 2026-04-19 session lost a good hour to a stale-focus
    # calibration (p50 17 mm → 12 mm once AF fired correctly). We warn
    # rather than fail because the autofocus path already soft-fails
    # without blocking, and because the threshold is empirical — better
    # to surface the warning in the operator's terminal than silently
    # accept a blurred solve. If calibration persistently reprojects
    # badly despite good inlier counts, this is your first suspect.
    SHARPNESS_WARN_THRESHOLD = 200.0
    if best_s < SHARPNESS_WARN_THRESHOLD:
        print(
            f"WARNING: captured sharpness {best_s:.0f} is below the "
            f"expected floor (~{SHARPNESS_WARN_THRESHOLD:.0f}). The "
            "phone camera may be mis-focused. Try:\n"
            "  1. Open CamON preview on the phone, tap a grid "
            "intersection near the frame centre to force AF, then "
            "re-run this script.\n"
            "  2. Check for smudges on the phone lens.\n"
            "  3. Verify the phone hasn't auto-dimmed / gone to a "
            "power-save mode that softens the image."
        )

    intersections = detect_grid_intersections(best, origin_px, args.px_per_m)
    print(f"detected {len(intersections)} candidate grid intersections")

    pix, world = snap_to_grid(intersections, origin_px, args.px_per_m, args.tilt_deg)
    print(f"kept {len(pix)} intersections after snapping (≤3cm error from nearest 10cm node)")

    if len(pix) < 20:
        print("WARNING: few correspondences — calibration may be low-quality")

    H, mask = cv2.findHomography(pix, world, cv2.RANSAC, ransacReprojThreshold=2.0)
    n_inliers = int(mask.sum())
    print(f"homography RANSAC inliers: {n_inliers}/{len(pix)}")

    # Evaluate: project inliers and compare
    proj = cv2.perspectiveTransform(pix.reshape(-1, 1, 2), H).reshape(-1, 2)
    errs = [math.hypot(world[i, 0] - proj[i, 0], world[i, 1] - proj[i, 1])
            for i in range(len(world)) if mask[i]]
    errs.sort()
    if errs:
        p50 = errs[len(errs) // 2]
        p95 = errs[int(len(errs) * 0.95)]
        print(f"reprojection error (m): p50={p50*1000:.1f}mm p95={p95*1000:.1f}mm "
              f"max={max(errs)*1000:.1f}mm")

    # Verify origin maps to ~(0,0)
    origin_world = cv2.perspectiveTransform(
        np.array([[origin_px]], dtype=np.float32), H).reshape(2)
    print(f"origin px {origin_px} -> world {origin_world[0]:+.3f},{origin_world[1]:+.3f} "
          f"(should be ~0,0)")

    # Save
    payload = {
        "calibrated_at": time.strftime("%Y-%m-%dT%H:%M:%S"),
        "stream": args.stream,
        "frame_shape": [W_px, H_px],
        "sharpness": float(best_s),
        "homography_px_to_m": H.tolist(),
        "correspondences": len(pix),
        "inliers": n_inliers,
        "reproj_error_p50_mm": float(p50 * 1000) if errs else None,
        "reproj_error_p95_mm": float(p95 * 1000) if errs else None,
        "bootstrap_origin_px": list(origin_px),
        "bootstrap_px_per_m": args.px_per_m,
        "bootstrap_tilt_deg": args.tilt_deg,
    }
    OUT.write_text(json.dumps(payload, indent=2))
    print(f"wrote {OUT}")

    if args.save_visual:
        vis = best.copy()
        for i, (px, py) in enumerate(pix):
            col = (0, 255, 0) if mask[i] else (0, 0, 255)
            cv2.circle(vis, (int(px), int(py)), 5, col, -1)
        cv2.drawMarker(vis, (int(origin_px[0]), int(origin_px[1])), (255, 255, 0),
                       cv2.MARKER_CROSS, 40, 3)
        path = Path("/tmp/calibration_vis.jpg")
        cv2.imwrite(str(path), vis, [cv2.IMWRITE_JPEG_QUALITY, 92])
        print(f"visual: {path}")


if __name__ == "__main__":
    main()
