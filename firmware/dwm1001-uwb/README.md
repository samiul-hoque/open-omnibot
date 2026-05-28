# DWM1001 UWB Firmware

Custom firmware for DWM1001 Ultra-Wideband modules.

## Overview

The DWM1001 modules can operate with:
1. **Factory firmware** - PANS (Positioning and Networking Stack)
2. **Custom firmware** - For advanced features

## Factory Firmware (Recommended for Start)

The DWM1001 comes with PANS firmware that works out of the box:
- Configure via UART shell or SPI
- Set as anchor or tag
- Use TWR (Two-Way Ranging) for positioning

### Shell Commands

```
# Enter shell mode
[Enter] twice

# Show position
les

# Configure as anchor
nma

# Configure as tag
nmt

# Set anchor position
aps x y z
```

## Custom Firmware

For advanced features, custom firmware can be developed using:
- Segger Embedded Studio
- nRF5 SDK
- DWM1001 API

### Building

```bash
# Instructions for custom firmware build
```

## Configuration

### Anchor Setup

Place anchors at known positions and configure coordinates.

### Tag Setup

The tag (on robot) queries anchors for range measurements.

## Protocol

<!-- Document the data format sent to ESP32 -->
Data format: `$UWB,anchor_id,distance_mm,timestamp\n`
