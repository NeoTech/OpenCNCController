# ESP32-S3 HAL Firmware

OpenCNC motion controller firmware for ESP32-S3.

## Features

- Dual-core 240MHz Xtensa LX7
- USB-OTG for CDC communication
- Hardware timer step generation
- FreeRTOS task architecture

## Requirements

- ESP-IDF v5.0 or later
- ESP32-S3 DevKit or custom board

## Building

```bash
# Set up ESP-IDF environment
. ~/esp/esp-idf/export.sh

# Configure target
idf.py set-target esp32s3

# Build
idf.py build

# Flash
idf.py -p /dev/ttyUSB0 flash monitor
```

## Pin Configuration

| Function | GPIO | Notes |
|----------|------|-------|
| Step X | GPIO1 | |
| Dir X | GPIO2 | |
| Step Y | GPIO3 | |
| Dir Y | GPIO4 | |
| Step Z | GPIO5 | |
| Dir Z | GPIO6 | |
| Limit X | GPIO7 | Active low |
| Limit Y | GPIO8 | Active low |
| Limit Z | GPIO9 | Active low |
| Probe | GPIO10 | Active low |
| E-Stop | GPIO11 | Active low |
| Spindle PWM | GPIO12 | 5kHz |
| Spindle Enable | GPIO13 | |
| Coolant Mist | GPIO14 | |
| Coolant Flood | GPIO15 | |

## Architecture

```
Core 0:                     Core 1:
┌─────────────────┐        ┌─────────────────┐
│ Communication   │        │ Motion Control  │
│ Task            │        │ Task            │
│                 │        │                 │
│ - USB CDC Rx/Tx │        │ - Queue Mgmt    │
│ - Status Send   │        │ - Step Timer    │
│ - Command Parse │        │ - Position Track│
└─────────────────┘        └─────────────────┘
```

The dual-core architecture separates real-time motion from communication for deterministic step timing.
