# Raspberry Pi Pico HAL Firmware

OpenCNC motion controller firmware for RP2040 (Raspberry Pi Pico).

## Features

- Dual-core ARM Cortex-M0+ @ 133 MHz
- PIO-based hardware step generation
- USB CDC communication
- Core separation (motion vs comm)

## Requirements

- Pico SDK v1.5.0 or later
- CMake 3.13+
- arm-none-eabi-gcc

## Building

```bash
# Set SDK path
export PICO_SDK_PATH=/path/to/pico-sdk

# Build
mkdir build && cd build
cmake ..
make -j

# Output: opencnc_hal.uf2
```

## Flashing

1. Hold BOOTSEL button and connect USB
2. Pico appears as USB mass storage
3. Drag and drop `opencnc_hal.uf2`

Or using picotool:
```bash
picotool load opencnc_hal.uf2
```

## Pin Configuration

| Function | GPIO | Notes |
|----------|------|-------|
| Step X | GP0 | |
| Dir X | GP1 | |
| Step Y | GP2 | |
| Dir Y | GP3 | |
| Step Z | GP4 | |
| Dir Z | GP5 | |
| Limit X | GP10 | Pull-up |
| Limit Y | GP11 | Pull-up |
| Limit Z | GP12 | Pull-up |
| Probe | GP13 | Pull-up |
| E-Stop | GP14 | Pull-up |
| Spindle PWM | GP15 | 5kHz |
| Spindle Enable | GP16 | |
| Coolant Mist | GP17 | |
| Coolant Flood | GP18 | |

## Architecture

```
        Core 0                      Core 1
┌─────────────────────┐    ┌─────────────────────┐
│   Communication     │    │   Motion Control    │
│                     │    │                     │
│ - USB CDC Rx/Tx     │    │ - Queue Management  │
│ - TinyUSB Tasks     │    │ - Step Generation   │
│ - Status Broadcast  │    │ - Position Tracking │
│                     │    │                     │
└─────────────────────┘    └─────────────────────┘
         │                           │
         └───────── Shared ──────────┘
              Motion Queue
              Position State
```

## PIO Step Generator

The RP2040's PIO (Programmable I/O) enables deterministic step pulse generation:

- Hardware-timed pulses
- No CPU involvement during stepping
- Up to 200 kHz step rate
- Precise pulse timing (8ns resolution)

## Performance

| Config | Max Step Rate |
|--------|---------------|
| Timer-based | 50 kHz |
| PIO-based | 200 kHz |
| PIO + DMA | 250 kHz |
