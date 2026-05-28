"""Shared offline replay methods and physical constants.

Single source of truth for the four estimation methods used to replay
experiment data offline.  All thesis analysis scripts import from here.

The FK formulas and constants MUST stay in sync with:
  - firmware:  firmware/esp32-omni/src/config.h, mecanum.cpp
  - server:    server/src/localization/mecanumKinematics.js
  - docs:      docs/data-pipeline.md §11 "Physical Constants"
"""

from __future__ import annotations

import math

import numpy as np


# ---------------------------------------------------------------------------
# Physical constants (mirror firmware/esp32-omni/src/config.h and
# server/src/config.js).
# ---------------------------------------------------------------------------
WHEEL_RADIUS = 0.04
LX = 0.1175
LY = 0.0953
L_SUM = LX + LY
COUNTS_PER_WHEEL_REV = 1092
METERS_PER_COUNT = (2 * math.pi * WHEEL_RADIUS) / COUNTS_PER_WHEEL_REV  # ~0.00023 m
COMPLEMENTARY_ALPHA = 0.98


# ---------------------------------------------------------------------------
# Trajectory metadata
# ---------------------------------------------------------------------------
KNOWN_TRAJECTORIES = {
    'straight_2m', 'circle_0_5m_strafe', 'square_0m8_rotate', 'square_0m8_strafe',
}

TRAJ_KIND = {
    'straight_2m':        'straight',
    'circle_0_5m_strafe': 'loop',
    'square_0m8_rotate':  'loop',
    'square_0m8_strafe':  'loop',
}

TRAJ_LABEL = {
    'straight_2m':        'Straight (2 m)',
    'circle_0_5m_strafe': 'Circle (R = 0.5 m)',
    'square_0m8_rotate':  'Square rotate (0.8 m)',
    'square_0m8_strafe':  'Square strafe (0.8 m)',
}

TRAJ_PATH_M = {
    'straight_2m':        2.0,
    'circle_0_5m_strafe': 2 * math.pi * 0.5,
    'square_0m8_rotate':  3.2,
    'square_0m8_strafe':  3.2,
}

REQUIRED_COLS = (
    'elapsed_ms',
    'enc_L1', 'enc_R1', 'enc_R2', 'enc_L2',
    'imu_yaw', 'imu_gyro_z', 'imu_accel_x', 'imu_accel_y',
)

METHODS = ['Encoder-only', 'IMU-only', 'Complementary', 'BNO055 Fusion']

STRAIGHT_METRIC_COLS = ['forward_err_cm', 'lateral_drift_cm', 'heading_err_deg', 'pos_err_cm']
LOOP_METRIC_COLS = ['loop_err_cm', 'max_dev_cm', 'heading_err_deg', 'drift_pct']


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _wrap_pi(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


def _time_s(cols: dict[str, np.ndarray]) -> np.ndarray:
    t = cols['elapsed_ms']
    return (t - t[0]) / 1000.0


def _enc_array(cols: dict[str, np.ndarray], i: int) -> np.ndarray:
    return np.array([cols['enc_L1'][i], cols['enc_R1'][i],
                     cols['enc_R2'][i], cols['enc_L2'][i]])


def _polyline_min_distance(points: np.ndarray, polyline: np.ndarray) -> np.ndarray:
    """Minimum perpendicular distance from each point to a polyline.

    `points` is (N, 2), `polyline` is (K, 2) with K ≥ 2 vertices defining
    K-1 connected line segments (the first vertex is NOT implicitly
    reconnected to the last — pass a closed polyline by repeating the
    first vertex as the last one).

    Returns an (N,) array of non-negative distances. Vectorised over N;
    loops over K-1 segments, which is fine because trajectories have ≤5.
    """
    n = points.shape[0]
    best = np.full(n, np.inf)
    for i in range(len(polyline) - 1):
        a = polyline[i]
        b = polyline[i + 1]
        ab = b - a
        ab_sq = float(ab @ ab)
        if ab_sq == 0.0:
            # Degenerate zero-length segment — distance to the endpoint.
            d = np.hypot(points[:, 0] - a[0], points[:, 1] - a[1])
        else:
            # Project each point onto the segment line, clamp the
            # parameter to [0, 1] so we get the nearest point on the
            # segment (not the infinite line).
            t = ((points - a) @ ab) / ab_sq
            t = np.clip(t, 0.0, 1.0)
            proj = a + t[:, None] * ab
            d = np.hypot(points[:, 0] - proj[:, 0], points[:, 1] - proj[:, 1])
        best = np.minimum(best, d)
    return best


def path_deviation(traj_id: str, traj: np.ndarray) -> np.ndarray:
    """Perpendicular distance of each replayed pose to the commanded path.

    `traj` is an Nx4 [t, x, y, θ] array in the run's body frame (starts
    at origin, +x = initial forward). Returns an (N,) array of
    non-negative distances in metres — "how far off the commanded
    geometry the robot was at each sample".

    Commanded-path geometry, all in body frame:
      - straight_2m          → line segment (0, 0) → (2, 0)
      - circle_0_5m_strafe   → circle centred at (+0.5, 0), radius 0.5
                               (centre and direction from integrating
                                quarterArcSegments from (0, 0, 0))
      - square_0m8_rotate    → closed polyline at (±0.4·2, ±0.4·2)
                               corners unfurled into body-frame corners
                               (0,0)→(0.8,0)→(0.8,0.8)→(0,0.8)→(0,0)
      - square_0m8_strafe    → mirrored in Y: corners
                               (0,0)→(0.8,0)→(0.8,−0.8)→(0,−0.8)→(0,0)
      - unknown trajectory   → fall back to distance from start (so
                               max() of this is comparable to the old
                               "max distance from start" metric).

    Used by `compute_metrics` for the `max_dev_cm` column on loop
    trajectories. Prior implementation computed distance from the
    body-frame origin, which conflates "path reach" (structural) with
    "off-path deviation" (actual error). A perfect 0.8 m square returns
    0 here; the old metric returned 113 cm (the diagonal).
    """
    xy = traj[:, 1:3]
    if traj_id == 'straight_2m':
        # One segment (0, 0) → (2, 0). In-line is |y| inside x ∈ [0, 2];
        # outside that interval we fall back to endpoint distance via
        # the parameter-clamp in _polyline_min_distance.
        poly = np.array([[0.0, 0.0], [2.0, 0.0]])
        return _polyline_min_distance(xy, poly)
    if traj_id == 'circle_0_5m_strafe':
        r = np.hypot(xy[:, 0] - 0.5, xy[:, 1])
        return np.abs(r - 0.5)
    if traj_id == 'square_0m8_rotate':
        poly = np.array([[0.0, 0.0], [0.8, 0.0], [0.8, 0.8], [0.0, 0.8], [0.0, 0.0]])
        return _polyline_min_distance(xy, poly)
    if traj_id == 'square_0m8_strafe':
        poly = np.array([[0.0, 0.0], [0.8, 0.0], [0.8, -0.8], [0.0, -0.8], [0.0, 0.0]])
        return _polyline_min_distance(xy, poly)
    # Unknown — fall back to distance from start pose.
    return np.hypot(xy[:, 0], xy[:, 1])


def world_to_body_pose(wx: float, wy: float, w_theta_rad: float,
                       start_x: float, start_y: float,
                       start_theta_rad: float,
                       ) -> tuple[float, float, float]:
    """Transform a world-frame pose into the run's body frame.

    The replayed trajectories (enc, imu, complementary, bno) are all
    integrated from (0, 0, 0) — i.e. they live in the frame whose origin
    is the robot's physical start pose and whose +x axis is the initial
    body-forward direction. Ground-truth snapshots arrive in the overhead
    camera's world frame (calibrated via the floor-grid homography), so a
    comparison `traj[-1] - gt` is only meaningful if we first move `gt`
    into the same body frame.

    Implements Option C (2026-04-19 decision): operator tape placement
    errors rotate/shift the measurement, not the physical trajectory, so
    we compensate in analysis using the pre-trajectory snapshot of the
    actual start pose (captured as `measuredStartPose` in meta.json by
    the runner's preflight step).

    Inputs are radians for headings; output heading is wrapped to (−π, π].
    """
    dx = wx - start_x
    dy = wy - start_y
    c = math.cos(-start_theta_rad)
    s = math.sin(-start_theta_rad)
    body_x = c * dx - s * dy
    body_y = s * dx + c * dy
    body_theta = _wrap_pi(w_theta_rad - start_theta_rad)
    return body_x, body_y, body_theta


# ---------------------------------------------------------------------------
# Replay methods
#
# Each takes a dict of {column_name: np.ndarray} and returns an Nx4
# array of [t_sec, x, y, theta_rad].
# ---------------------------------------------------------------------------

def replay_encoder_only(cols: dict[str, np.ndarray]) -> np.ndarray:
    """Pure mecanum FK from encoder deltas."""
    t = _time_s(cols); n = len(t)
    out = np.zeros((n, 4)); out[:, 0] = t
    x = y = theta = 0.0; prev = _enc_array(cols, 0)
    for i in range(1, n):
        cur = _enc_array(cols, i)
        d = (cur - prev) * METERS_PER_COUNT; prev = cur
        om = d / WHEEL_RADIUS
        dx_b = (WHEEL_RADIUS / 4) * (om[0] + om[1] + om[2] + om[3])
        dy_b = (WHEEL_RADIUS / 4) * (om[0] - om[1] + om[2] - om[3])
        dth  = (WHEEL_RADIUS / (4 * L_SUM)) * (-om[0] + om[1] + om[2] - om[3])
        c, s = math.cos(theta + dth / 2), math.sin(theta + dth / 2)
        x += c * dx_b - s * dy_b
        y += s * dx_b + c * dy_b
        theta = _wrap_pi(theta + dth)
        out[i, 1:] = [x, y, theta]
    return out


def replay_imu_only(cols: dict[str, np.ndarray]) -> np.ndarray:
    """Pure inertial dead reckoning from BNO055 linear accel + gyro_z."""
    t = _time_s(cols); n = len(t)
    out = np.zeros((n, 4)); out[:, 0] = t
    x = y = theta = vx = vy = 0.0
    for i in range(1, n):
        dt = t[i] - t[i - 1]
        if not (0 < dt < 0.5):
            dt = 0.05
        gz = cols['imu_gyro_z'][i]
        ax_b = cols['imu_accel_x'][i]
        ay_b = cols['imu_accel_y'][i]
        if math.isnan(gz) or math.isnan(ax_b) or math.isnan(ay_b):
            out[i, 1:] = [x, y, theta]; continue
        theta = _wrap_pi(theta + gz * dt)
        c, s = math.cos(theta), math.sin(theta)
        ax_w, ay_w = c * ax_b - s * ay_b, s * ax_b + c * ay_b
        x += vx * dt + 0.5 * ax_w * dt * dt
        y += vy * dt + 0.5 * ay_w * dt * dt
        vx += ax_w * dt; vy += ay_w * dt
        out[i, 1:] = [x, y, theta]
    return out


def replay_complementary(cols: dict[str, np.ndarray], alpha: float = COMPLEMENTARY_ALPHA) -> np.ndarray:
    """Encoder position + complementary-filter heading (gyro-dominant)."""
    t = _time_s(cols); n = len(t)
    out = np.zeros((n, 4)); out[:, 0] = t
    x = y = theta = theta_enc = 0.0; prev = _enc_array(cols, 0)
    for i in range(1, n):
        dt = t[i] - t[i - 1]
        if not (0 < dt < 0.5):
            dt = 0.05
        cur = _enc_array(cols, i)
        d = (cur - prev) * METERS_PER_COUNT; prev = cur
        om = d / WHEEL_RADIUS
        dx_b = (WHEEL_RADIUS / 4) * (om[0] + om[1] + om[2] + om[3])
        dy_b = (WHEEL_RADIUS / 4) * (om[0] - om[1] + om[2] - om[3])
        dth_enc = (WHEEL_RADIUS / (4 * L_SUM)) * (-om[0] + om[1] + om[2] - om[3])
        theta_enc = _wrap_pi(theta_enc + dth_enc)
        gz = cols['imu_gyro_z'][i]
        if math.isnan(gz):
            gz = 0.0
        theta = _wrap_pi(alpha * (theta + gz * dt) + (1 - alpha) * theta_enc)
        c, s = math.cos(theta), math.sin(theta)
        x += c * dx_b - s * dy_b
        y += s * dx_b + c * dy_b
        out[i, 1:] = [x, y, theta]
    return out


def replay_bno055(cols: dict[str, np.ndarray]) -> np.ndarray:
    """Encoder position with heading from BNO055 fused yaw."""
    t = _time_s(cols); n = len(t)
    out = np.zeros((n, 4)); out[:, 0] = t
    x = y = 0.0
    yaw0 = next((v for v in cols['imu_yaw'] if not math.isnan(v)), 0.0)
    prev = _enc_array(cols, 0)
    for i in range(1, n):
        cur = _enc_array(cols, i)
        d = (cur - prev) * METERS_PER_COUNT; prev = cur
        om = d / WHEEL_RADIUS
        dx_b = (WHEEL_RADIUS / 4) * (om[0] + om[1] + om[2] + om[3])
        dy_b = (WHEEL_RADIUS / 4) * (om[0] - om[1] + om[2] - om[3])
        yaw = cols['imu_yaw'][i]
        theta = _wrap_pi(math.radians(yaw - yaw0)) if not math.isnan(yaw) else 0.0
        c, s = math.cos(theta), math.sin(theta)
        x += c * dx_b - s * dy_b
        y += s * dx_b + c * dy_b
        out[i, 1:] = [x, y, theta]
    return out


REPLAY_FNS = {
    'Encoder-only':  replay_encoder_only,
    'IMU-only':      replay_imu_only,
    'Complementary': replay_complementary,
    'BNO055 Fusion': replay_bno055,
}


# ---------------------------------------------------------------------------
# Metrics
# ---------------------------------------------------------------------------

def compute_metrics(traj_id: str, traj: np.ndarray, gt: dict,
                    start_pose: dict | None = None) -> dict:
    """Compute trajectory-specific error metrics for one method/run.

    `traj` is a replayed pose history in the run's body frame (starts at
    (0, 0, 0)). `gt` is the operator / camera ground-truth reading in
    either body frame (pre-GT-rig manual measurements) or world frame
    (overhead-camera snapshots). `start_pose`, when supplied, is the
    world-frame pose of the robot at experiment_start (preflight
    snapshot; stored as `measuredStartPose` in meta.json with keys
    `x` / `y` / `thetaDeg`). When present we use it to rotate `gt` into
    the body frame so the comparison against `traj[-1]` is apples-to-
    apples regardless of tape-placement error. When absent — historical
    data captured before preflight existed — we fall through to the old
    behaviour and assume `gt` is already body-frame, which is accurate
    for the pre-rig manual-measurement workflow.
    """
    x, y, th = float(traj[-1, 1]), float(traj[-1, 2]), float(traj[-1, 3])
    gx, gy, gth = gt['xMeas'], gt['yMeas'], math.radians(gt['thetaDegMeas'])
    if start_pose is not None:
        gx, gy, gth = world_to_body_pose(
            gx, gy, gth,
            start_pose['x'], start_pose['y'],
            math.radians(start_pose['thetaDeg']),
        )
    if TRAJ_KIND[traj_id] == 'straight':
        return {
            'forward_err_cm': (x - gx) * 100,
            'lateral_drift_cm': (y - gy) * 100,
            'heading_err_deg': math.degrees(_wrap_pi(th - gth)),
            'pos_err_cm': math.hypot(x - gx, y - gy) * 100,
        }
    loop_err = math.hypot(x - gx, y - gy) * 100
    # max_dev_cm is the max perpendicular deviation of the replayed path
    # from the commanded geometry (not "max distance from start"). For a
    # perfect run this is 0 cm; on a drifting encoder it's the worst
    # off-path excursion in cm. See `path_deviation` for the geometry
    # used per trajectory.
    max_dev = float(np.max(path_deviation(traj_id, traj))) * 100
    return {
        'loop_err_cm': loop_err,
        'max_dev_cm': max_dev,
        'heading_err_deg': math.degrees(_wrap_pi(th - gth)),
        'drift_pct': loop_err / (TRAJ_PATH_M[traj_id] * 100) * 100 if TRAJ_PATH_M[traj_id] > 0 else 0,
    }
