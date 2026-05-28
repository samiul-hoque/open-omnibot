"""
Forward-drive test with snapshot-based GT + guaranteed-stop safety.

Drives vx at constant speed until encoder distance hits a target or a
wall-clock timeout fires, then stops. Captures two ground-truth snapshots
via the aruco_detector HTTP service — one before the drive, one after —
so the start→end GT delta can be compared against the encoder delta.

Crucially: the cmd socket is wrapped so that ANY exit path — exception,
timeout, ctrl-C — sends a stop command via a FRESH connection before
exiting. The 2026-04-18 Tier-2 crash happened because an earlier run
script had no such guard: a ConnectionResetError mid-drive left the
server's 100 ms keepalive replaying the last cmd forever, and the robot
drove into a wall.

Prerequisite: the snapshot service must be running:
    python evaluation/ground_truth/aruco_detector.py &

Usage:
    python evaluation/ground_truth/drive_straight.py --vx 0.15 --target-m 2.0
"""
from __future__ import annotations

import argparse
import json
import math
import signal
import sys
import threading
import time
import urllib.error   # explicit — urllib.request pulls it transitively in CPython but that isn't guaranteed
import urllib.parse
import urllib.request

import websocket

DEFAULT_SERVER = "ws://localhost:3000/ws"
DEFAULT_SNAPSHOT = "http://localhost:5055"


class CmdClient:
    """WS client that can be used from the main thread and always stops on close."""
    def __init__(self, url: str):
        self.url = url
        self._ws = None
        self._open()

    def _open(self):
        self._ws = websocket.create_connection(self.url, timeout=3.0)

    def send(self, obj):
        try:
            self._ws.send(json.dumps(obj))
            return True
        except Exception:
            try:
                self._ws.close()
            except Exception:
                pass
            try:
                self._open()
                self._ws.send(json.dumps(obj))
                return True
            except Exception:
                return False

    def close_with_stop(self):
        """Attempt to deliver stop over this socket, then over a fresh
        socket as a last resort. Never raises."""
        delivered = False
        try:
            if self._ws is not None:
                self._ws.send(json.dumps({"type": "stop"}))
                delivered = True
        except Exception:
            pass
        try:
            if self._ws is not None:
                self._ws.close()
        except Exception:
            pass
        if not delivered:
            try:
                fresh = websocket.create_connection(self.url, timeout=2.0)
                fresh.send(json.dumps({"type": "stop"}))
                fresh.close()
                delivered = True
            except Exception:
                pass
        if not delivered:
            print("[safety] WARNING: could not deliver stop — relying on server-side "
                  "750ms cmd-stale timeout", flush=True)


def pose_reader_loop(url: str, latest: dict, stop_evt: threading.Event):
    """Reader thread — only tracks encoder pose. GT comes from HTTP snapshots."""
    ws = websocket.create_connection(url, timeout=3.0)
    try:
        while not stop_evt.is_set():
            try:
                ws.settimeout(0.5)
                m = json.loads(ws.recv())
            except Exception:
                continue
            if m.get("type") != "state":
                continue
            if m.get("pose"):
                latest["pose"] = dict(m["pose"])
    finally:
        try: ws.close()
        except Exception: pass


def take_snapshot(snapshot_url: str, trajectory: str, run_id: str, label: str):
    """Call the aruco snapshot service. Returns (pose_dict_or_none, image_path_or_none, err_or_none)."""
    params = urllib.parse.urlencode({
        "trajectory": trajectory, "run_id": run_id, "label": label,
    })
    url = f"{snapshot_url}/snapshot?{params}"
    try:
        with urllib.request.urlopen(url, timeout=6.0) as resp:
            body = json.loads(resp.read().decode("utf-8"))
    except urllib.error.HTTPError as e:
        try: body = json.loads(e.read().decode("utf-8"))
        except Exception: body = {"error": str(e)}
    except Exception as e:
        return None, None, str(e)
    if not body.get("detected"):
        return None, body.get("image_path"), body.get("error", "no detection")
    return {
        "x": body["x"], "y": body["y"], "theta": body["theta"],
        "theta_deg": body["theta_deg"], "side_px": body["side_px"],
    }, body.get("image_path"), None


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[1])
    ap.add_argument("--server", default=DEFAULT_SERVER)
    ap.add_argument("--snapshot-url", default=DEFAULT_SNAPSHOT,
                    help="Base URL of the aruco_detector snapshot service")
    ap.add_argument("--vx", type=float, default=0.15)
    ap.add_argument("--target-m", type=float, default=2.0,
                    help="encoder forward distance to stop at")
    ap.add_argument("--timeout-s", type=float, default=0.0,
                    help="wall-clock timeout (default: 1.5x expected duration)")
    ap.add_argument("--zero-imu", action="store_true", default=True)
    ap.add_argument("--no-zero-imu", action="store_false", dest="zero_imu")
    ap.add_argument("--tier", type=int, default=0,
                    help="switch to tier before drive (0 = don't switch)")
    ap.add_argument("--trajectory", default="straight_2m",
                    help="Trajectory name — used to bucket snapshots on disk")
    ap.add_argument("--run-id", default="",
                    help="Run id — used as the snapshot subdir. Defaults to "
                         "drive_YYYY-MM-DDTHH-MM-SS.")
    args = ap.parse_args()

    if not args.run_id:
        args.run_id = "drive_" + time.strftime("%Y-%m-%dT%H-%M-%S")

    timeout_s = args.timeout_s or (args.target_m / max(args.vx, 0.01)) * 1.5

    latest = {"pose": None}
    stop_reader = threading.Event()
    reader_thread = threading.Thread(
        target=pose_reader_loop, args=(args.server, latest, stop_reader),
        daemon=True,
    )
    reader_thread.start()

    interrupted = threading.Event()
    signal.signal(signal.SIGINT, lambda *_: interrupted.set())

    cmd = CmdClient(args.server)
    try:
        # --- setup ---
        if args.tier in (1, 2):
            cmd.send({"type": "setTier", "tier": args.tier})
            print(f"switched to tier {args.tier}", flush=True)
            time.sleep(0.5)
        cmd.send({"type": "stop"})
        if args.zero_imu:
            cmd.send({"type": "zero_imu"})
            time.sleep(0.5)
        cmd.send({"type": "resetPose"})
        time.sleep(1.5)

        # wait for fresh encoder pose from state broadcast
        t0 = time.time()
        while latest["pose"] is None and time.time() - t0 < 3 and not interrupted.is_set():
            time.sleep(0.1)
        if latest["pose"] is None:
            print("FAIL: no pose broadcast received", flush=True)
            return
        p0 = dict(latest["pose"])

        # --- start snapshot ---
        print("capturing start snapshot...", flush=True)
        g0, img0, err0 = take_snapshot(args.snapshot_url, args.trajectory,
                                       args.run_id, "start")
        if err0:
            print(f"WARN: start snapshot failed — {err0} (img={img0})", flush=True)
        else:
            print(f"START: enc=({p0['x']:+.3f},{p0['y']:+.3f},"
                  f"{math.degrees(p0['theta']):+.1f}°)  "
                  f"gt=({g0['x']:+.3f},{g0['y']:+.3f},{g0['theta_deg']:+.1f}°)",
                  flush=True)
            print(f"  snapshot → {img0}", flush=True)

        # --- drive ---
        print(f"driving vx={args.vx} m/s until encoder ≥ {args.target_m} m "
              f"(timeout {timeout_s:.1f}s)", flush=True)
        t_start = time.time()
        last_print = 0
        stop_reason = "timeout"
        while not interrupted.is_set():
            now = time.time()
            elapsed = now - t_start
            ok = cmd.send({"type": "cmd", "vx": args.vx, "vy": 0, "w": 0})
            if not ok:
                stop_reason = "cmd-send-failed"; break
            p = latest["pose"]
            if p and now - last_print > 0.5:
                print(f"  t={elapsed:4.1f}s  enc=({p['x']:+.3f},{p['y']:+.3f},"
                      f"{math.degrees(p['theta']):+.0f}°)", flush=True)
                last_print = now
            if p and p["x"] >= args.target_m:
                stop_reason = "target-reached"; break
            if elapsed > timeout_s:
                stop_reason = "timeout"; break
            time.sleep(0.05)
        if interrupted.is_set():
            stop_reason = "ctrl-c"

        # --- stop + settle ---
        cmd.send({"type": "stop"})
        time.sleep(2.0)
        p1 = dict(latest["pose"]) if latest["pose"] else None

        # --- end snapshot ---
        print("capturing end snapshot...", flush=True)
        g1, img1, err1 = take_snapshot(args.snapshot_url, args.trajectory,
                                       args.run_id, "end")

        print(f"\nEND (stop reason: {stop_reason}):", flush=True)
        if p1:
            print(f"  enc=({p1['x']:+.3f},{p1['y']:+.3f},"
                  f"{math.degrees(p1['theta']):+.1f}°)", flush=True)
        if err1:
            print(f"  WARN: end snapshot failed — {err1} (img={img1})", flush=True)
        elif g1:
            print(f"  gt=({g1['x']:+.3f},{g1['y']:+.3f},{g1['theta_deg']:+.1f}°)",
                  flush=True)
            print(f"  snapshot → {img1}", flush=True)

        # --- summary ---
        if p1 and g0 and g1:
            dx_enc = p1["x"] - p0["x"]; dy_enc = p1["y"] - p0["y"]
            dx_gt = g1["x"] - g0["x"]; dy_gt = g1["y"] - g0["y"]
            dth_gt = g1["theta_deg"] - g0["theta_deg"]
            if dth_gt > 180: dth_gt -= 360
            if dth_gt < -180: dth_gt += 360
            print(f"encoder Δ: ({dx_enc:+.3f}, {dy_enc:+.3f}) m  "
                  f"Δθ={math.degrees(p1['theta']-p0['theta']):+.1f}°", flush=True)
            print(f"GT Δ:      ({dx_gt:+.3f}, {dy_gt:+.3f}) m  "
                  f"Δθ={dth_gt:+.1f}°", flush=True)

    finally:
        cmd.close_with_stop()
        stop_reader.set()
        reader_thread.join(timeout=1.0)


if __name__ == "__main__":
    main()
