# RP2040 CAN-FD Axis Node

CANopen slave node firmware for Raspberry Pi Pico (RP2040). Uses external MCP2518FD CAN-FD controller via SPI since RP2040 lacks native CAN.

## Hardware Requirements

- **MCU**: RP2040 (Raspberry Pi Pico or compatible)
- **CAN Controller**: MCP2518FD external CAN-FD controller
- **CAN Transceiver**: TJA1051T/3
- **Stepper Driver**: Compatible with step/direction interface

## Wiring

### MCP2518FD SPI Connection

| RP2040 Pin | MCP2518FD | Function      |
|------------|-----------|---------------|
| GP2        | SCK       | SPI Clock     |
| GP3        | SI        | MOSI          |
| GP4        | SO        | MISO          |
| GP5        | nCS       | Chip Select   |
| GP6        | INT       | Interrupt     |
| 3.3V       | VDD       | Power         |
| GND        | VSS       | Ground        |

### Stepper Control

| RP2040 Pin | Function      | Description            |
|------------|---------------|------------------------|
| GP16       | STEP          | Step pulse output      |
| GP17       | DIR           | Direction output       |
| GP18       | ENABLE        | Driver enable (act low)|
| GP19       | LIMIT_MIN     | Min limit switch       |
| GP20       | LIMIT_MAX     | Max limit switch       |
| GP21       | HOME          | Home switch            |
| GP25       | LED           | Status LED (onboard)   |

### CAN Bus

| MCP2518FD | TJA1051 | CAN Bus   |
|-----------|---------|-----------|
| TXCAN     | TXD     | -         |
| RXCAN     | RXD     | -         |
| -         | CANH    | CAN High  |
| -         | CANL    | CAN Low   |

## CAN-FD Configuration

- **Oscillator**: 40 MHz crystal on MCP2518FD
- **Arbitration**: 500 kbps
- **Data**: 2 Mbps
- **Frame Size**: Up to 64 bytes

## Building

### Prerequisites

- Pico SDK (set `PICO_SDK_PATH` environment variable)
- CMake 3.13+
- ARM GCC toolchain

### Build Commands

```bash
mkdir build && cd build
cmake ..
make
```

### Flash

Drag and drop `axis_node_rp2040.uf2` onto the Pico's USB mass storage device while holding BOOTSEL.

Or use picotool:
```bash
picotool load build/axis_node_rp2040.uf2
picotool reboot
```

## CANopen Features

Same as STM32G4 node:
- Auto-assign node ID
- NMT slave
- SDO server
- PDO processing
- CiA 402 drive profile

## Operation

### Boot Sequence

1. Initialize GPIO and SPI
2. Initialize MCP2518FD at 500kbps/2Mbps
3. Load configuration from flash
4. Start 1 kHz timer
5. Wait for auto-assign discovery
6. Receive node ID, send boot-up message
7. Process CANopen commands

### LED Patterns

| Pattern             | Meaning                         |
|--------------------|---------------------------------|
| Slow blink (500ms) | Waiting for node ID assignment  |
| Medium blink (250ms)| Pre-operational                |
| Fast blink (100ms) | Operational                     |

### USB Serial Debug

Connect to Pico's USB serial port at 115200 baud for debug output.

## MCP2518FD Notes

The MCP2518FD is connected via SPI and provides:
- Full CAN-FD support (64-byte frames, BRS)
- Hardware TX/RX FIFOs
- Configurable filters
- Interrupt-driven operation

Key registers:
- CiCON: Control register (mode selection)
- CiNBTCFG/CiDBTCFG: Bit timing
- CiFIFOCON: FIFO configuration
- CiFLTOBJ/CiMASK: Receive filters

## Differences from STM32G4 Node

| Feature            | STM32G4          | RP2040           |
|-------------------|------------------|------------------|
| CAN               | Native FDCAN     | MCP2518FD (SPI)  |
| Core              | Cortex-M4F       | Dual Cortex-M0+  |
| Clock             | 170 MHz          | 125 MHz          |
| Step generation   | Timer interrupt  | Timer callback   |

## License

MIT License - See project root for details
