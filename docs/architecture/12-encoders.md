---
last-verified: 2026-04-09
sources:
  - firmware/esp32-omni/src/sensors.cpp
  - firmware/esp32-omni/src/config.h
  - firmware/esp32-omni/test/test_overflow_logic/test_overflow_logic.cpp
---

# Encoders

## TL;DR

Four quadrature encoders read by the ESP32's hardware **PCNT** (Pulse Counter) peripheral, one PCNT unit per motor. The peripheral does the quadrature decoding, edge counting, and digital glitch filtering for free, leaving the firmware to handle two software concerns: detecting **16-bit counter overflow** and applying **per-motor gain correction** (see `11-motor-calibration.md`). At 1092 counts per wheel revolution and a wheel radius of 4 cm, one count ≈ **0.23 mm** of wheel travel.

## Why hardware PCNT instead of interrupts

The first prototype used GPIO interrupts to count encoder edges in software. At top speed each wheel produces ~1100 edges per second × 4 wheels = ~4400 ISRs/sec, plus encoder noise glitches that can multiply that. Two problems:

1. **ISR latency budget.** The ESP32 has a deep interrupt path; servicing 4400+ ISRs/sec consumed enough CPU that the WebSocket loop occasionally missed packets.
2. **Quadrature decoding in software is fragile.** You need to read both A and B pins in each ISR and update a state machine. We saw missed transitions during fast motion.

The PCNT peripheral solves both: it runs in hardware, decodes quadrature with two channels per unit, and only interrupts the CPU on **overflow** (which we choose not to handle as an interrupt — see below). Net CPU cost is ~zero regardless of speed.

`firmware/esp32-omni/src/sensors.cpp:42-73` — `setupSingleEncoder()`:

```cpp
pcnt_config.pos_mode   = PCNT_COUNT_DEC;     // count down on positive edge
pcnt_config.neg_mode   = PCNT_COUNT_INC;     // count up on negative edge
pcnt_config.lctrl_mode = PCNT_MODE_REVERSE;  // B low → reverse counting direction
pcnt_config.hctrl_mode = PCNT_MODE_KEEP;     // B high → keep counting direction
```

This is the standard 1× quadrature mode using one channel. With the encoder at 13 PPR and the 42:1 gearbox:

```
COUNTS_PER_MOTOR_REV  = ENCODER_PPR × 2  = 26      // 2× from edge counting
COUNTS_PER_WHEEL_REV  = 26 × GEAR_RATIO  = 1092
WHEEL_CIRCUMFERENCE   = 2π × 0.04 m       ≈ 0.251 m
METERS_PER_COUNT      = 0.251 / 1092      ≈ 2.3 × 10⁻⁴ m
```

(`config.h:49-56`.)

## The 16-bit overflow problem

PCNT counters on the ESP32 are **16-bit signed**: range `[-32768, +32767]`. At full speed the robot generates ~1100 counts/sec/wheel, so a counter wraps in roughly 30 seconds of constant motion. If the firmware just sampled the raw counter and computed `delta = current - previous`, a wrap from +32760 to -32760 would look like a delta of −65520 — a huge negative spike that would confuse:
- the velocity estimate (PID would slam motors backward),
- dead reckoning (the robot would teleport),
- the calibration accumulators (a single bad sample could ruin a measurement).

The PCNT peripheral can fire an interrupt on overflow but using the interrupt would mean coordinating shared state between an ISR and the main loop. We chose a simpler approach: **detect wrap in software at sample time.**

## How wrap detection works

`firmware/esp32-omni/src/sensors.cpp:91-111` — `getEncoderCount()`:

```cpp
int16_t rawCount = 0;
pcnt_get_counter_value(pcntUnits[motorIndex], &rawCount);

int16_t diff = rawCount - lastRawCounts[motorIndex];
if (diff > PCNT_OVERFLOW_THRESHOLD) {
    // counter wrapped from −32768 down to +32767 (underflow)
    overflowCounts[motorIndex] -= 65536;
} else if (diff < -PCNT_OVERFLOW_THRESHOLD) {
    // counter wrapped from +32767 up to −32768 (overflow)
    overflowCounts[motorIndex] += 65536;
}
lastRawCounts[motorIndex] = rawCount;

return (overflowCounts[motorIndex] + rawCount) * encDirs[motorIndex];
```

The trick is the **threshold**, defined in `config.h:63`:

```cpp
#define PCNT_OVERFLOW_THRESHOLD (PCNT_H_LIM / 2)   // = 16383
```

Half the counter range. The reasoning:
- **Normal motion** between two samples is small. Even at full speed, ~1100 counts/sec sampled at 50 Hz = 22 counts/sample. Always far below 16383.
- **A wrap** produces an apparent diff of roughly ±65536 (the full counter range). Always far above 16383.
- **No realistic motion** can produce a per-sample delta between 16383 and 65535. So the threshold cleanly separates "real motion" from "wrap event".

The accumulator `overflowCounts[]` holds the cumulative correction in 32-bit, and the returned value `overflowCounts + rawCount` is the true 32-bit position. That position is then sign-corrected by the per-motor `encDirs[]` constant.

### Why ±65536, not ±65535

The 16-bit counter wraps from +32767 to −32768 (a step of −65535, i.e. one short of 65536). But the wrap math is symmetric around the wrap boundary, and using **65536** (i.e. `2^16`) keeps the accumulator in clean modular arithmetic so subsequent reads behave correctly. The off-by-one doesn't accumulate because each wrap event resets the relationship between `rawCount` and `overflowCount`.

### Why this isn't an interrupt

PCNT *can* fire an ISR on overflow. We don't use it because:
- It would need a critical section or atomic add to share `overflowCounts` between the ISR and `getEncoderCount()`.
- Sample-time detection has zero ISR cost and is testable in isolation (see below).
- The 50 Hz sample rate is fast enough that we'll never miss two wraps between samples (would require >65k counts in 20 ms = 3.27 M counts/sec, which is ~3000× the hardware capability).

## Glitch filter

`sensors.cpp:62-64`:

```cpp
pcnt_set_filter_value(pcntUnits[index], 100);
pcnt_filter_enable(pcntUnits[index]);
```

The PCNT has a built-in digital glitch filter. The value is in **APB clock cycles** (80 MHz), so 100 = 1.25 µs. Edges shorter than this are ignored. Real encoder edges from a 1100-count/sec wheel are ~900 µs apart, so we have a >700× safety margin while still rejecting fast electrical noise from motor brushes.

We added the filter after seeing phantom counts during high-current motor draw. Without the filter, brushed-motor commutation noise would couple into the encoder lines and produce false edges that the PCNT would faithfully count.

## Per-direction gain application

After overflow correction, the count is fed into `readEncoders()` which converts it to wheel angular velocity. The per-direction gain (from motor calibration) is applied here:

`sensors.cpp:140-156`:

```cpp
int32_t deltaCounts = data.counts[i] - lastCounts[i];
float gain = (deltaCounts >= 0) ? motorGainFwd[i] : motorGainRev[i];
float wheelRevs = (float)deltaCounts * gain / COUNTS_PER_WHEEL_REV;
float radians   = wheelRevs * 2.0f * 3.14159265f;
data.velocities[i] = radians / dt_sec;
```

The gain is selected by the **sign of the delta** (the actual measured direction of rotation), not by the commanded direction. This is consistent with how `motor_calibration.cpp` accumulates samples and how the server-side dead reckoning consumes the counts. See `11-motor-calibration.md` for the rationale.

## Testing the overflow logic

`firmware/esp32-omni/test/test_overflow_logic/test_overflow_logic.cpp` is a Unity test suite that runs on `pio test -e native`. It reproduces the overflow algorithm in pure C and asserts a battery of cases:

- **Basic counting:** forward, backward, direction inversion, incremental.
- **No false positives:** normal motion of 1000 counts in 10-count steps must not trigger overflow correction.
- **Wrap-around boundary:** crossing +32767 → -32768 must produce a continuous 32-bit count.
- **Oscillation:** rapid sign changes within the wrap zone must not double-correct.

Why these tests exist: during early development, a tuning commit changed the threshold value and silently broke the wrap detection. The bug was invisible at low speeds (the encoder never wrapped during normal demos) but catastrophic at high speeds during a thesis demo. The tests now run on every commit so a regression is caught immediately.

The test file uses a **simulated** counter — it doesn't talk to PCNT — because the algorithm we care about is the software wrap detector, and PCNT itself is hardware that we trust.

Run with:

```bash
cd firmware/esp32-omni
pio test -e native
```

## Tuning / configuration

Located in `firmware/esp32-omni/src/config.h`:

| Constant | Value | Notes |
|---|---|---|
| `ENCODER_PPR` | 13 | Pulses per shaft revolution. Encoder-specific. |
| `GEAR_RATIO` | 42 | Gearbox ratio. Match your gearmotor spec sheet. |
| `COUNTS_PER_MOTOR_REV` | 26 | Derived: PPR × 2 (1× quadrature edge counting) |
| `COUNTS_PER_WHEEL_REV` | 1092 | Derived: above × GEAR_RATIO |
| `PCNT_H_LIM` | 32767 | Hardware ceiling; do not change |
| `PCNT_L_LIM` | -32768 | Hardware floor; do not change |
| `PCNT_OVERFLOW_THRESHOLD` | 16383 | Half the counter range. Lower means false positives on fast motion; higher means missed wraps at extreme speeds. |
| Glitch filter (in code) | 100 cycles ≈ 1.25 µs | Raise if you see phantom counts under load; lower if you have a high-PPR encoder where real edges might be filtered out. |

## Known limitations

- **Single-channel quadrature.** We use one PCNT channel (A as pulse, B as control). Two-channel mode would give 4× resolution but our PPR is high enough that we don't need it.
- **No skew detection.** If the encoder cable detaches mid-run, the count just stops changing. We don't detect "encoder dead" — the velocity will read zero and the PID will pile on PWM. Future work: a "stall" detector that compares commanded PWM to encoder activity.
- **No NVS persistence of encoder counts.** Counts reset to 0 on every boot. Pose state is held only in the server, which means a server restart loses the dead-reckoning state too. This is intentional (the server is the authoritative state holder) but easy to forget.
- **Filter is a fixed value.** A faster encoder (high PPR) on a slow gearbox could have real edges below 1.25 µs apart. If you change to a different motor, recompute the filter value.

## Source

- `firmware/esp32-omni/src/sensors.cpp:42-73` — PCNT initialization and glitch filter
- `firmware/esp32-omni/src/sensors.cpp:91-111` — wrap detection algorithm
- `firmware/esp32-omni/src/sensors.cpp:131-156` — `readEncoders()` velocity conversion
- `firmware/esp32-omni/src/config.h:47-63` — encoder constants and overflow threshold
- `firmware/esp32-omni/test/test_overflow_logic/test_overflow_logic.cpp` — regression suite
- Related: `10-motor-control.md`, `11-motor-calibration.md`, `50-dead-reckoning.md`
