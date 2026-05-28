"""
Overhead ArUco ground-truth snapshot service.

Runs an HTTP server that, on demand, grabs a single frame from the overhead
phone camera, detects the robot's ArUco marker, converts (pixel, pixel_heading)
to world (x, y, theta), and writes an annotated PNG under
`evaluation/snapshots/<trajectory>/<run_id>/<label>.png`.

This replaces the earlier streaming-WS detector. GT is now captured once per
waypoint (start/end of a run, or manual points during a drive), not at 30 Hz.

Usage:
    # Start the service
    python evaluation/ground_truth/aruco_detector.py

    # Take a snapshot
    curl 'http://localhost:5055/snapshot?trajectory=straight_2m&run_id=exp_...&label=endpoint'

    # One-shot diagnostic (no server): auto-scan which dict the marker is in
    python evaluation/ground_truth/aruco_detector.py --diagnose
"""
from __future__ import annotations

import argparse
import http.server
import json
import math
import os
import signal
import socketserver
import sys
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from urllib.parse import parse_qs, urlparse

import cv2
import numpy as np


# Layout defaults for this rig (2026-04-18). Override via CLI or calibration.json.
DEFAULTS = {
    "stream_url":       "rtsp://<camera-host>:8080/video/h264",
    "origin_px":        (1160, 539),
    "px_per_meter":     830.0,
    "tilt_deg":         0.87,
    "marker_yaw_off":   90.0,
    "marker_id":        0,
    "min_side_px":      50.0,
    "snapshot_port":    5055,
}

REPO_ROOT = Path(__file__).resolve().parent.parent.parent  # <repo>/evaluation/ground_truth/ → <repo>
DEFAULT_SNAPSHOT_ROOT = REPO_ROOT / "evaluation" / "snapshots"


@dataclass
class PoseMsg:
    x: float          # meters, world frame
    y: float          # meters
    theta: float      # radians, CCW-positive, world frame
    marker_id: int
    side_px: float
    detect_ms: int    # wall-clock at cap.read() return


class DirectCapture:
    """Background-thread RTSP grabber that always keeps only the most
    recent frame.

    Why not a simple `cap.read()` at snapshot time: FFmpeg's RTSP
    demuxer + decoder buffers frames the application doesn't consume.
    On this phone camera (50 fps) a 5-second idle between snapshots
    leaves ~250 queued frames, and `cap.read()` returns the OLDEST
    one. Setting CAP_PROP_BUFFERSIZE=1 helps the decoder-output queue
    but not the full demuxer pipeline — measured stream lag climbed
    to 15 s across a 20 s experiment run (2026-04-19 straight_2m smoke
    test).

    Why a background thread: reading continuously keeps the entire
    pipeline drained so the latest available frame is always
    <1 frame old. An earlier iteration of this file avoided the
    thread on the theory that it would contend with the snapshot
    read; in practice the two barely touch (grabber holds cv2, the
    snapshot handler only copies the latest frame out of a Python
    buffer under a short lock). Throughput is irrelevant here — the
    snapshot path just needs a fresh frame at request time.
    """

    # How long to wait for the first frame after start() / after a
    # previously-stale grabber catches up.
    _READY_TIMEOUT_S = 5.0

    def __init__(self, url: str):
        self.url = url
        self._cap = None
        self._thread = None
        self._stop = threading.Event()
        self._lock = threading.Lock()
        self._latest_frame = None
        self._latest_ts = 0.0
        self._frame_count = 0

    def start(self):
        if self.url.startswith("rtsp://"):
            os.environ.setdefault(
                "OPENCV_FFMPEG_CAPTURE_OPTIONS",
                "rtsp_transport;tcp|allowed_media_types;video",
            )
        self._cap = cv2.VideoCapture(self.url, cv2.CAP_FFMPEG)
        if not self._cap.isOpened():
            raise RuntimeError(f"could not open stream: {self.url}")
        try:
            self._cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        except Exception:
            pass
        self._stop.clear()
        self._thread = threading.Thread(target=self._loop, daemon=True,
                                        name="rtsp-grabber")
        self._thread.start()

    def _loop(self):
        """Continuously drain the capture. Short sleep on transient
        read failures so we don't hot-spin if the stream flaps."""
        while not self._stop.is_set():
            ok, frame = self._cap.read()
            if not ok or frame is None:
                time.sleep(0.02)
                continue
            ts = time.time()
            with self._lock:
                self._latest_frame = frame
                self._latest_ts = ts
                self._frame_count += 1

    def read(self):
        """Return the most recent frame (copy), or (None, 0.0) if the
        grabber hasn't produced a frame yet."""
        with self._lock:
            if self._latest_frame is None:
                return None, 0.0
            # Copy so the caller can mutate without racing the grabber.
            return self._latest_frame.copy(), self._latest_ts

    def read_fresh(self, flush: int = 0):
        """Return the next frame captured AFTER this call was made.

        Semantics: blocks until the grabber thread delivers a frame
        whose capture timestamp postdates the call. This is what the
        snapshot path wants — it guarantees we're not returning a
        buffered-from-before frame. On a 50 fps stream the wait is
        typically <20 ms.

        `flush` is ignored; the background grabber already keeps the
        pipeline drained. The parameter is retained so callers that
        used the old DirectCapture API still type-check.
        """
        del flush
        request_ts = time.time()
        deadline = request_ts + self._READY_TIMEOUT_S
        while time.time() < deadline:
            with self._lock:
                ts = self._latest_ts
                if self._latest_frame is not None and ts >= request_ts:
                    return self._latest_frame.copy(), ts
            time.sleep(0.005)
        return None, 0.0

    def stop(self):
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            self._thread = None
        if self._cap is not None:
            self._cap.release()
            self._cap = None

    def stats(self) -> dict:
        with self._lock:
            return {
                "frames_grabbed": self._frame_count,
                "latest_frame_age_ms": (
                    int((time.time() - self._latest_ts) * 1000)
                    if self._latest_ts > 0 else None
                ),
            }


class PixelToWorld:
    """Transform pixel coords to world meters.

    Two modes:
    - `homography`: a 3x3 matrix from calibrate_homography.py. Handles
      translation, rotation, perspective, and non-uniform scale.
    - `linear`: fallback bootstrap model (origin + tilt + uniform scale).
    """
    def __init__(self, origin_px, px_per_m, tilt_deg, homography=None):
        self.ox, self.oy = origin_px
        self.s = px_per_m
        t = -math.radians(tilt_deg)
        self._cos = math.cos(t)
        self._sin = math.sin(t)
        self._H = np.array(homography, dtype=np.float64) if homography is not None else None

    def point(self, px: float, py: float) -> tuple[float, float]:
        if self._H is not None:
            v = self._H @ np.array([px, py, 1.0])
            return float(v[0] / v[2]), float(v[1] / v[2])
        dx = px - self.ox
        dy = py - self.oy
        xr = self._cos * dx - self._sin * dy
        yr = self._sin * dx + self._cos * dy
        return xr / self.s, -yr / self.s

    def direction(self, vx: float, vy: float, base_px=None, base_py=None) -> tuple[float, float]:
        if self._H is not None and base_px is not None and base_py is not None:
            p0 = self.point(base_px, base_py)
            p1 = self.point(base_px + vx, base_py + vy)
            return p1[0] - p0[0], p1[1] - p0[1]
        xr = self._cos * vx - self._sin * vy
        yr = self._sin * vx + self._cos * vy
        return xr, -yr


_DICT_BY_NAME = {
    "4X4_50":   cv2.aruco.DICT_4X4_50,
    "4X4_100":  cv2.aruco.DICT_4X4_100,
    "4X4_250":  cv2.aruco.DICT_4X4_250,
    "4X4_1000": cv2.aruco.DICT_4X4_1000,
    "5X5_50":   cv2.aruco.DICT_5X5_50,
    "6X6_50":   cv2.aruco.DICT_6X6_50,
}


def _detector_params():
    # Three adaptive-threshold passes at winSize 3/23/43. Covers the window
    # sizes that actually decoded our marker during bench (bench shows default
    # narrow range misses it entirely, fine-step aggressive hits 96% but runs
    # 2 fps; this coarse-wide config hits 82% at 20 fps at 1080p).
    p = cv2.aruco.DetectorParameters()
    p.adaptiveThreshWinSizeMin = 3
    p.adaptiveThreshWinSizeMax = 53
    p.adaptiveThreshWinSizeStep = 20
    p.minMarkerPerimeterRate = 0.005
    p.polygonalApproxAccuracyRate = 0.08
    p.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
    return p


def build_detector(dict_name="4X4_50"):
    """Build a single-dict detector."""
    if dict_name not in _DICT_BY_NAME:
        raise ValueError(f"unknown aruco dict {dict_name!r}; known: {list(_DICT_BY_NAME)}")
    d = cv2.aruco.getPredefinedDictionary(_DICT_BY_NAME[dict_name])
    return cv2.aruco.ArucoDetector(d, _detector_params())


def build_multi_detectors(dict_names):
    """Build one detector per dict for --diagnose auto-scan."""
    return [(n, cv2.aruco.ArucoDetector(
        cv2.aruco.getPredefinedDictionary(_DICT_BY_NAME[n]), _detector_params()))
            for n in dict_names if n in _DICT_BY_NAME]


def detect_pose(frame, detector, xform: PixelToWorld, target_id: int,
                min_side_px: float, marker_yaw_off_rad: float):
    """Return (PoseMsg, corners_4x2) or None."""
    corners, ids, _ = detector.detectMarkers(frame)
    if ids is None:
        return None
    best = None
    for i, c in enumerate(corners):
        if int(ids[i][0]) != target_id:
            continue
        cc = c.reshape(4, 2)
        sides = [float(np.linalg.norm(cc[k] - cc[(k + 1) % 4])) for k in range(4)]
        side = sum(sides) / 4.0
        if side < min_side_px:
            continue
        if best is None or side > best[1]:
            best = (cc, side)
    if best is None:
        return None
    cc, side = best
    cx, cy = float(cc[:, 0].mean()), float(cc[:, 1].mean())
    vx, vy = float(cc[1, 0] - cc[0, 0]), float(cc[1, 1] - cc[0, 1])
    x, y = xform.point(cx, cy)
    dx, dy = xform.direction(vx, vy, base_px=cc[0, 0], base_py=cc[0, 1])
    theta = math.atan2(dy, dx) + marker_yaw_off_rad
    theta = ((theta + math.pi) % (2.0 * math.pi)) - math.pi
    return PoseMsg(x=x, y=y, theta=theta, marker_id=target_id, side_px=side, detect_ms=0), cc


def _annotate(frame, corners, pose: PoseMsg):
    vis = frame.copy()
    cv2.aruco.drawDetectedMarkers(
        vis, [corners.reshape(1, 4, 2).astype(np.float32)],
        np.array([[pose.marker_id]]),
    )
    txt = (f"x={pose.x:+.3f}m  y={pose.y:+.3f}m  "
           f"th={math.degrees(pose.theta):+.1f}deg  side={pose.side_px:.0f}px")
    cv2.putText(vis, txt, (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)
    return vis


def _safe_path_component(s: str, fallback: str) -> str:
    """Sanitize a single path component for use under snapshot_root.

    Filters to [A-Za-z0-9_-.] and rejects the exact literals "." and ".."
    — those two would otherwise combine across trajectory/run_id/label
    to escape snapshot_root via `root/../../foo`. Dots are still allowed
    anywhere else because valid run_ids contain speed values like "0.10".
    An empty result falls back to the caller-supplied default.
    """
    cleaned = "".join(c for c in (s or "") if c.isalnum() or c in "-_.")
    if not cleaned or cleaned in (".", ".."):
        return fallback
    return cleaned


def _load_calibration():
    """Load calibration.json alongside this script; return (xform-args, meta)."""
    cal_path = Path(__file__).parent / "calibration.json"
    if not cal_path.exists():
        return None, {"error": "calibration.json missing"}
    with open(cal_path) as fh:
        cal = json.load(fh)
    return {
        "origin_px": tuple(cal.get("bootstrap_origin_px", DEFAULTS["origin_px"])),
        "px_per_m": cal.get("bootstrap_px_per_m", DEFAULTS["px_per_meter"]),
        "tilt_deg": cal.get("bootstrap_tilt_deg", DEFAULTS["tilt_deg"]),
        "homography": cal.get("homography_px_to_m"),
    }, {
        "calibrated_at": cal.get("calibrated_at"),
        "p50_mm": cal.get("reproj_error_p50_mm"),
        "p95_mm": cal.get("reproj_error_p95_mm"),
        "inliers": cal.get("inliers"),
    }


# ---------------------------------------------------------------------------
# HTTP snapshot service
# ---------------------------------------------------------------------------

class SnapshotService:
    """Stream-warm snapshot service. Thread-safe."""
    def __init__(self, grabber: DirectCapture, detector, xform: PixelToWorld,
                 marker_id: int, min_side_px: float, yaw_off_rad: float,
                 snapshot_root: Path):
        self.grabber = grabber
        self.detector = detector
        self.xform = xform
        self.marker_id = marker_id
        self.min_side_px = min_side_px
        self.yaw_off_rad = yaw_off_rad
        self.snapshot_root = snapshot_root
        self.snapshot_root.mkdir(parents=True, exist_ok=True)
        self._lock = threading.Lock()
        self._last_frame_ms = 0

    def health(self) -> dict:
        return {
            "ok": True,
            "last_frame_ms": self._last_frame_ms,
            "grabber": self.grabber.stats(),
            "snapshot_root": str(self.snapshot_root),
            "marker_id": self.marker_id,
            "min_side_px": self.min_side_px,
        }

    def reload_calibration(self) -> dict:
        xform_args, meta = _load_calibration()
        if xform_args is None:
            return {"ok": False, **meta}
        with self._lock:
            self.xform = PixelToWorld(
                xform_args["origin_px"], xform_args["px_per_m"],
                xform_args["tilt_deg"], homography=xform_args["homography"],
            )
        return {"ok": True, **meta}

    def snapshot(self, trajectory: str, run_id: str, label: str) -> tuple[dict, int]:
        """Grab a fresh frame, detect marker, save annotated PNG. Returns (body, status)."""
        traj = _safe_path_component(trajectory, "uncategorized")
        run = _safe_path_component(run_id, f"snap_{int(time.time())}")
        lbl = _safe_path_component(label, "snapshot")

        # Defense in depth: even though _safe_path_component rejects "." and
        # "..", any future loosening of that filter must not escape the
        # snapshot root. Resolve the final path and reject if it sits outside.
        out_dir = (self.snapshot_root / traj / run).resolve()
        root_resolved = self.snapshot_root.resolve()
        if not str(out_dir).startswith(str(root_resolved) + os.sep) and out_dir != root_resolved:
            return {"error": f"refusing to write outside snapshot root: {out_dir}"}, 400
        out_dir.mkdir(parents=True, exist_ok=True)
        out_path = out_dir / f"{lbl}.png"

        # Hold the lock only for the camera+detector I/O. Disk writes happen
        # below, outside the critical section, so a ~50 ms PNG encode doesn't
        # block concurrent /health or /snapshot requests.
        with self._lock:
            frame, ts = self.grabber.read_fresh(flush=3)
            if frame is None:
                return {"error": "stream read failed"}, 503
            detect_ms = int(ts * 1000)
            self._last_frame_ms = detect_ms
            result = detect_pose(frame, self.detector, self.xform,
                                 self.marker_id, self.min_side_px, self.yaw_off_rad)

        response = {
            "trajectory": traj,
            "run_id": run,
            "label": lbl,
            "image_path": str(out_path.relative_to(REPO_ROOT)),
            "detect_ms": detect_ms,
        }

        if result is None:
            cv2.imwrite(str(out_path), frame)
            response.update({"detected": False, "error": "no marker detected"})
            return response, 404

        pose, corners = result
        pose.detect_ms = detect_ms
        cv2.imwrite(str(out_path), _annotate(frame, corners, pose))
        response.update({
            "detected": True,
            "x": pose.x,
            "y": pose.y,
            "theta": pose.theta,
            "theta_deg": math.degrees(pose.theta),
            "side_px": pose.side_px,
        })
        return response, 200


class _Handler(http.server.BaseHTTPRequestHandler):
    """Snapshot/health/reload endpoints. `service_ref` set on the server."""
    service_ref: SnapshotService = None  # type: ignore

    def log_message(self, fmt, *args):
        print(f"[http] {self.command} {self.path} — {fmt % args}", flush=True)

    def do_GET(self):
        u = urlparse(self.path)
        q = parse_qs(u.query)
        if u.path == "/snapshot":
            body, status = self.service_ref.snapshot(
                (q.get("trajectory") or [""])[0],
                (q.get("run_id") or [""])[0],
                (q.get("label") or [""])[0],
            )
            self._send_json(status, body)
        elif u.path == "/health":
            self._send_json(200, self.service_ref.health())
        else:
            self._send_json(404, {"error": "unknown endpoint", "path": u.path})

    def do_POST(self):
        u = urlparse(self.path)
        if u.path == "/reload_calibration":
            body = self.service_ref.reload_calibration()
            self._send_json(200 if body.get("ok") else 500, body)
        else:
            self._send_json(404, {"error": "unknown endpoint", "path": u.path})

    def _send_json(self, status: int, obj: dict):
        data = json.dumps(obj).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(data)))
        self.end_headers()
        self.wfile.write(data)


class _ThreadingServer(socketserver.ThreadingMixIn, http.server.HTTPServer):
    allow_reuse_address = True
    daemon_threads = True


# ---------------------------------------------------------------------------
# Diagnostic: auto-scan which dict the marker is in (no HTTP server)
# ---------------------------------------------------------------------------

def run_diagnose(grabber: DirectCapture, marker_id: int, min_side_px: float):
    dicts = ["4X4_50", "4X4_100", "4X4_250", "4X4_1000"]
    multi = build_multi_detectors(dicts)
    print(f"[diagnose] scanning {dicts} for marker id={marker_id}, "
          f"min side {min_side_px:.0f} px", flush=True)
    hits = {n: 0 for n in dicts}
    for i in range(60):
        frame, _ = grabber.read()
        if frame is None:
            time.sleep(0.1)
            continue
        for name, det in multi:
            corners, ids, _ = det.detectMarkers(frame)
            if ids is None:
                continue
            for idx, mid in enumerate(ids.flatten().tolist()):
                if int(mid) != marker_id:
                    continue
                c = corners[idx].reshape(4, 2)
                side = sum(float(np.linalg.norm(c[k] - c[(k+1)%4])) for k in range(4)) / 4.0
                if side >= min_side_px:
                    hits[name] += 1
                    break
    print("[diagnose] hits over 60 frames (real-size only):")
    for n in dicts:
        print(f"  {n}: {hits[n]}")
    best = max(hits, key=lambda k: hits[k])
    if hits[best] > 0:
        print(f"[diagnose] marker most likely in DICT_{best}", flush=True)
    else:
        print("[diagnose] no dict decoded the marker at real size. "
              "Verify marker visibility, focus, and print polarity "
              "(black border + white interior data).", flush=True)


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[1])
    ap.add_argument("--stream", default=DEFAULTS["stream_url"])
    ap.add_argument("--marker-id", type=int, default=DEFAULTS["marker_id"])
    ap.add_argument("--marker-dict", default="4X4_50",
                    help="ArUco dict name for pose detection (default: 4X4_50)")
    ap.add_argument("--min-side-px", type=float, default=DEFAULTS["min_side_px"])
    ap.add_argument("--yaw-off-deg", type=float, default=DEFAULTS["marker_yaw_off"],
                    help="Added to detected heading. Marker-on-robot orientation offset.")
    ap.add_argument("--port", type=int, default=DEFAULTS["snapshot_port"])
    ap.add_argument("--snapshot-root", default=str(DEFAULT_SNAPSHOT_ROOT),
                    help="Where to save annotated snapshot PNGs.")
    ap.add_argument("--diagnose", action="store_true",
                    help="One-shot scan across 4X4_{50,100,250,1000} to identify "
                         "which dict the marker decodes in. Exits after 60 frames.")
    ap.add_argument("--no-calibration", action="store_true",
                    help="Ignore calibration.json; use the linear bootstrap model.")
    args = ap.parse_args()

    # Load calibration (or fall back to linear bootstrap).
    xform_args, meta = (None, None) if args.no_calibration else _load_calibration()
    if xform_args is None:
        print(f"[detector] no homography — using linear bootstrap "
              f"(origin={DEFAULTS['origin_px']}, scale={DEFAULTS['px_per_meter']}, "
              f"tilt={DEFAULTS['tilt_deg']}°)", flush=True)
        xform = PixelToWorld(DEFAULTS["origin_px"], DEFAULTS["px_per_meter"],
                             DEFAULTS["tilt_deg"])
    else:
        print(f"[detector] loaded calibration (p50={meta.get('p50_mm')}mm, "
              f"{meta.get('inliers')} inliers)", flush=True)
        xform = PixelToWorld(xform_args["origin_px"], xform_args["px_per_m"],
                             xform_args["tilt_deg"],
                             homography=xform_args["homography"])

    print(f"[detector] stream={args.stream}", flush=True)
    print(f"[detector] marker id={args.marker_id} dict={args.marker_dict} "
          f"min_side={args.min_side_px}px yaw_off={args.yaw_off_deg}°", flush=True)

    grabber = DirectCapture(args.stream)
    grabber.start()

    try:
        if args.diagnose:
            run_diagnose(grabber, args.marker_id, args.min_side_px)
            return 0

        detector = build_detector(args.marker_dict)
        service = SnapshotService(
            grabber, detector, xform,
            args.marker_id, args.min_side_px, math.radians(args.yaw_off_deg),
            Path(args.snapshot_root),
        )
        _Handler.service_ref = service
        # Bind to loopback only. The server is a same-host helper; exposing it
        # to the LAN would let any client on the WiFi trigger captures (and
        # pre-fix, escape the snapshot root via path traversal).
        httpd = _ThreadingServer(("127.0.0.1", args.port), _Handler)
        stop = threading.Event()
        def _sig(*_):
            print("[detector] shutting down", flush=True)
            stop.set()
            threading.Thread(target=httpd.shutdown, daemon=True).start()
        signal.signal(signal.SIGINT, _sig)
        signal.signal(signal.SIGTERM, _sig)

        print(f"[detector] snapshot service listening on http://127.0.0.1:{args.port}", flush=True)
        print(f"[detector]   GET  /snapshot?trajectory=X&run_id=Y&label=Z", flush=True)
        print(f"[detector]   GET  /health", flush=True)
        print(f"[detector]   POST /reload_calibration", flush=True)
        print(f"[detector] snapshot root: {service.snapshot_root}", flush=True)
        httpd.serve_forever()
        return 0
    finally:
        grabber.stop()


if __name__ == "__main__":
    sys.exit(main())
