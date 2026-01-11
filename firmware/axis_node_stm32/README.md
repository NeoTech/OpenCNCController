# STM32G4 Axis Node

CANopen slave node firmware for STM32G474RE Nucleo board. Implements CiA 402 drive profile for stepper motor control.

## Hardware Requirements

- **MCU**: STM32G474RE (170 MHz Cortex-M4F)
- **CAN Transceiver**: TJA1051T/3 on FDCAN1
- **Stepper Driver**: Compatible with step/direction interface (A4988, DRV8825, TMC2209, etc.)

## Pin Configuration

| Function        | Pin   | Description                      |
|-----------------|-------|----------------------------------|
| CAN RX          | PA11  | FDCAN1_RX to TJA1051            |
| CAN TX          | PA12  | FDCAN1_TX to TJA1051            |
| Step            | PA0   | Step pulse output               |
| Direction       | PA1   | Direction output                |
| Enable          | PA2   | Driver enable (active low)      |
| Limit Min       | PA3   | Minimum limit switch input      |
| Limit Max       | PA4   | Maximum limit switch input      |
| Home            | PA5   | Home switch input               |
| Encoder A       | PB6   | Encoder channel A (optional)    |
| Encoder B       | PB7   | Encoder channel B (optional)    |
| LED             | PC13  | Status LED                      |

## CAN-FD Configuration

- **Arbitration**: 500 kbps (17 prescaler @ 170 MHz)
- **Data**: 2 Mbps (5 prescaler @ 170 MHz)
- **Frame Size**: Up to 64 bytes

## CANopen Features

- **Auto-assign**: Automatically receives node ID from master
- **NMT**: Full network management support
- **SDO**: Object dictionary access
- **PDO**: Process data for real-time control
- **Heartbeat**: 1 second producer interval

## CiA 402 Drive Profile

Implemented objects:

| Index  | Subindex | Description           |
|--------|----------|-----------------------|
| 0x6040 | 0        | Controlword           |
| 0x6041 | 0        | Statusword            |
| 0x6060 | 0        | Modes of operation    |
| 0x6061 | 0        | Modes of operation display |
| 0x607A | 0        | Target position       |
| 0x6064 | 0        | Position actual value |
| 0x60FF | 0        | Target velocity       |
| 0x606C | 0        | Velocity actual value |
| 0x60FD | 0        | Digital inputs        |
| 0x60FE | 0        | Digital outputs       |

## Building

### Prerequisites

- ARM GCC toolchain
- CMake 3.20+
- STM32CubeG4 HAL drivers

### Build Commands

```bash
mkdir build && cd build
cmake -DCMAKE_TOOLCHAIN_FILE=../toolchain-arm-none-eabi.cmake ..
make
```

### Flash

```bash
# Using ST-Link
st-flash write build/axis_node_stm32.bin 0x08000000

# Using OpenOCD
openocd -f interface/stlink.cfg -f target/stm32g4x.cfg -c "program build/axis_node_stm32.elf verify reset exit"
```

## Operation

### Boot Sequence

1. System clock configured to 170 MHz from 8 MHz HSE
2. FDCAN1 initialized at 500 kbps / 2 Mbps
3. GPIO configured for stepper, limits, and LED
4. Timer 2 configured for step pulse generation
5. Timer 6 configured for 1 ms system tick
6. CANopen slave stack initialized
7. Wait for auto-assign discovery from master
8. Receive node ID assignment
9. Send NMT boot-up message
10. Wait for NMT operational command
11. Process PDOs and motion commands

### LED Patterns

| Pattern              | Meaning                          |
|---------------------|----------------------------------|
| Slow blink (500ms)  | Pre-operational, waiting for ID  |
| Fast blink (100ms)  | Operational, ready               |
| Solid ON            | Motion in progress               |
| Double pulse        | Homing in progress               |
| Triple pulse        | Error condition                  |

### Limit Switch Handling

- Limit switches are active low (NC switches)
- Motion stops immediately when limit hit in direction of travel
- Homing sequence uses limit for zero position
- Digital inputs object (0x60FD) reports limit states

## Configuration

Node configuration is stored in flash and can be modified via SDO or TOML file.

### TOML Example

```toml
[node]
type = "stepper"
axis = "X"

[motion]
max_velocity = 50000      # steps/s
acceleration = 100000     # steps/s^2
steps_per_mm = 200        # 1mm/rev, 200 steps/rev

[homing]
velocity = 10000          # steps/s
velocity_slow = 1000      # steps/s
backoff = 200             # steps
direction = -1            # -1 = min, +1 = max
```

## Troubleshooting

### No CAN Communication

1. Check CAN transceiver wiring and termination
2. Verify 120Ω termination at each end of bus
3. Check TJA1051 power supply (5V)
4. Use oscilloscope to verify CAN signals

### Motor Not Moving

1. Verify driver enable signal
2. Check step/direction wiring
3. Confirm NMT operational state
4. Check controlword for enabled state

### Position Drift

1. Check encoder feedback if installed
2. Verify no missed steps (reduce velocity/acceleration)
3. Check for electrical noise on step line

## License

MIT License - See project root for details
