# Localization Algorithms

This directory contains documentation and reference implementations for the localization system.

## Tiered Approach

The localization system uses a tiered approach, from simple to complex:

| Tier | Method | Complexity | Status |
|------|--------|------------|----------|
| 1 | [Odometry](tier1-odometry.md) | Low | Operational. Drifts over time. |
| 2 | [Sensor Fusion](tier2-fusion.md) | Medium | Operational. Reduced heading drift. |
| 3 | [EKF](tier3-ekf.md) | High | Reference theory only — not currently integrated in the runtime. |

## Sensors Used

- **Wheel Encoders** — Tiers 1 & 2.
- **IMU (BNO055)** — Tier 2. Gyroscope + accelerometer only; the
  magnetometer is deliberately disabled (motor magnetic interference
  near the IMU mount).
- **UWB (DWM1001)** — Planned for Tier 3. The reader, EKF fusion,
  and anchor handling were removed from the runtime on 2026-04-14
  pending validation of the lower tiers. The reference math is
  documented here for future work.

## Directory Structure

```
localization/
├── README.md              # This file
├── tier1-odometry.md      # Dead reckoning theory
├── tier2-fusion.md        # Complementary filter approach
├── tier3-ekf.md           # Extended Kalman Filter
└── matlab/                # MATLAB reference implementations
    ├── odometry.m
    ├── complementary_filter.m
    └── ekf_localization.m
```

## Quick Comparison

### Tier 1: Odometry Only
- Uses only wheel encoders
- Fast computation
- Accumulates drift over time
- Good for short-term motion

### Tier 2: Complementary Filter
- Fuses odometry + IMU
- Simple weighted combination
- Reduces heading drift
- Still accumulates position error

### Tier 3: Extended Kalman Filter (reference theory)
- Documented as the planned tier — not currently integrated in the
  firmware/server runtime. The UWB reader, EKF fusion, and anchor
  handling were removed on 2026-04-14 pending validation of Tiers 1
  and 2.
- When integrated, would fuse encoders, IMU, and UWB range
  measurements for bounded position error.
- See `tier3-ekf.md` and `matlab/ekf_localization.m` for the planned
  algorithm and reference math.

## Getting Started

1. Start with Tier 1 to verify basic motion (encoders only).
2. Switch to Tier 2 for improved heading via IMU fusion.
3. Use the Tier 3 reference math (`tier3-ekf.md`,
   `matlab/ekf_localization.m`) as a starting point for your own
   UWB + EKF integration — it is not a drop-in working module.

See individual markdown files for theory and reference math.
