# Hardware

This directory contains all hardware-related files for the Open Omnibot platform.

## Where to look

| If you want… | Read |
|---|---|
| A user-facing component reference (what each part does, why it was chosen) | [`docs/hardware/00-components.md`](../docs/hardware/00-components.md) |
| A spreadsheet of parts and quantities | [`bom.csv`](bom.csv) |
| The authoritative pin map (single source of truth) | [`firmware/esp32-omni/src/config.h`](../firmware/esp32-omni/src/config.h) |
| 3D-printable chassis parts | [`cad/`](cad/) (TODO — not yet committed) |
| Schematic / PCB | [`pcb/`](pcb/) (TODO — not yet committed) |
| Wiring diagrams | [`wiring/`](wiring/) (TODO — not yet committed) |

## Bill of Materials

See [bom.csv](bom.csv) for the complete parts list. Costs and sourcing links are still TODO; component identities are accurate as of the firmware revision in `config.h`.

## Directory Structure

```
hardware/
├── README.md          # This file
├── bom.csv            # Bill of materials spreadsheet
├── cad/               # 3D printable parts
│   ├── *.stl          # Print-ready files
│   └── source/        # Editable source (STEP, F3D, etc.)
├── pcb/               # Custom PCB designs (KiCad)
│   ├── *.kicad_pcb
│   └── gerbers/
└── wiring/            # Wiring diagrams
    └── *.png/svg
```

## CAD Files

STL files are provided for 3D printing. Source files in STEP or Fusion 360 format are in `cad/source/`.

### Print Settings

- Material: PLA or PETG
- Layer height: 0.2mm
- Infill: 20-30%
- Supports: As needed

## Wiring / pinout

The authoritative pin map lives in [`firmware/esp32-omni/src/config.h`](../firmware/esp32-omni/src/config.h). The table below mirrors that file at the time of writing — if it falls out of sync, `config.h` is correct.

### ESP32 GPIO

| Function | Motor L1 (RL) | Motor R1 (RR) | Motor R2 (FR) | Motor L2 (FL) |
|---|---|---|---|---|
| PWM (LEDC channel = motor index) | GPIO 14 | GPIO 25 | GPIO 26 | GPIO 27 |
| Encoder A | GPIO 35 | GPIO 36 | GPIO 33 | GPIO 19 |
| Encoder B | GPIO 34 | GPIO 39 | GPIO 32 | GPIO 18 |

| Function | ESP32 GPIO |
|---|---|
| I²C SDA (BNO055 + MCP23017) | GPIO 21 (default) |
| I²C SCL (BNO055 + MCP23017) | GPIO 22 (default) |

UWB (DWM1001) modules are **not connected to the ESP32** — the tag attaches to the server PC over USB serial, and the four anchors are stand-alone at the room corners.

### MCP23017 (I²C 0x20) — direction + standby

| Pin | Function |
|---|---|
| 0 | L2 IN2 (front-left direction) |
| 1 | L2 IN1 |
| 2 | R2 IN2 (front-right direction) |
| 3 | R2 IN1 |
| 4 | R1 IN2 (rear-right direction) |
| 5 | R1 IN1 |
| 6 | L1 IN2 (rear-left direction) |
| 7 | L1 IN1 |
| 8 | STBY_FRONT (enable L2, R2) |
| 9 | STBY_REAR (enable L1, R1) |

### I²C addresses

| Device | Address |
|---|---|
| MCP23017 | `0x20` |
| BNO055 IMU | `0x29` |

See [`docs/hardware/00-components.md`](../docs/hardware/00-components.md) for the *why* behind each connection (e.g. why direction pins live on an expander, why R2/L2 indices are swapped in firmware).

## Sourcing

Components can be sourced from:
- Electronics: DigiKey, Mouser, LCSC
- Mechanical: Amazon, AliExpress, McMaster-Carr
- 3D Printing: Local printer or services like JLCPCB, PCBWay

## Assembly Notes

<!-- Add assembly instructions or link to detailed guide -->
See [docs/getting-started.md](../docs/getting-started.md) for assembly instructions.
