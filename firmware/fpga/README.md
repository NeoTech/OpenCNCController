# FPGA HAL Firmware

OpenCNC motion controller in Verilog HDL for FPGA targets.

## Target Devices

| Device | Speed | LUTs | Notes |
|--------|-------|------|-------|
| iCE40UP5K | 48 MHz | 5280 | Low-cost, open toolchain |
| iCE40HX8K | 100 MHz | 7680 | Higher capacity |
| ECP5-25 | 150 MHz | 24K | High performance |
| ECP5-85 | 200 MHz | 84K | Maximum performance |

## Features

- Hardware DDA step generation
- Up to 1 MHz+ step rate
- Zero-jitter timing
- SPI interface to host MCU
- On-chip motion queue
- PWM spindle control

## Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                        FPGA                                      │
├─────────────────────────────────────────────────────────────────┤
│  ┌─────────────┐  ┌─────────────┐  ┌─────────────────────────┐  │
│  │ SPI Slave   │  │ Command     │  │ Status                  │  │
│  │ Interface   │──│ Decoder     │──│ Register                │  │
│  └─────────────┘  └─────────────┘  └─────────────────────────┘  │
│         │                                     │                  │
│         ▼                                     ▼                  │
│  ┌─────────────────────────────────────────────────────────┐    │
│  │                  Motion Segment FIFO                     │    │
│  │                     (32 entries)                         │    │
│  └─────────────────────────────────────────────────────────┘    │
│                              │                                   │
│                              ▼                                   │
│  ┌─────────────────────────────────────────────────────────┐    │
│  │                   Step Generator                         │    │
│  │   ┌─────────┐  ┌─────────┐  ┌─────────┐                 │    │
│  │   │ Axis 0  │  │ Axis 1  │  │ Axis 2  │  ...            │    │
│  │   │   DDA   │  │   DDA   │  │   DDA   │                 │    │
│  │   └────┬────┘  └────┬────┘  └────┬────┘                 │    │
│  └────────│────────────│────────────│──────────────────────┘    │
│           ▼            ▼            ▼                           │
│     ┌──────────┐ ┌──────────┐ ┌──────────┐                     │
│     │Step/Dir X│ │Step/Dir Y│ │Step/Dir Z│                     │
│     └──────────┘ └──────────┘ └──────────┘                     │
├─────────────────────────────────────────────────────────────────┤
│                     I/O Pins                                     │
└─────────────────────────────────────────────────────────────────┘
```

## Building (Open Source Toolchain)

### iCE40 (with Yosys + nextpnr)

```bash
# Synthesize
yosys -p "synth_ice40 -top cnc_top -json cnc_top.json" rtl/*.v

# Place and route
nextpnr-ice40 --up5k --package sg48 --json cnc_top.json \
    --pcf constraints/ice40up5k.pcf --asc cnc_top.asc

# Pack bitstream
icepack cnc_top.asc cnc_top.bin

# Program
iceprog cnc_top.bin
```

### ECP5 (with Yosys + nextpnr)

```bash
# Synthesize
yosys -p "synth_ecp5 -top cnc_top -json cnc_top.json" rtl/*.v

# Place and route
nextpnr-ecp5 --25k --package CABGA381 --json cnc_top.json \
    --lpf constraints/ecp5.lpf --textcfg cnc_top.config

# Pack bitstream
ecppack cnc_top.config cnc_top.bit

# Program
openFPGALoader -b colorlight cnc_top.bit
```

## Pin Constraints

See `constraints/` directory for device-specific pin mappings.

## Host Interface

The FPGA communicates with a host MCU (ESP32, STM32, etc.) via SPI:

| Signal | Direction | Description |
|--------|-----------|-------------|
| SCLK | Input | SPI clock (up to 20 MHz) |
| MOSI | Input | Commands/data to FPGA |
| MISO | Output | Status/position from FPGA |
| CS_N | Input | Chip select (active low) |

## Performance

| Metric | iCE40UP5K | ECP5-25 |
|--------|-----------|---------|
| Max step rate | 500 kHz | 2 MHz |
| Position resolution | 32-bit | 32-bit |
| Motion queue | 16 segments | 64 segments |
| Axes | 4 | 6+ |
| Timing jitter | <20 ns | <5 ns |
