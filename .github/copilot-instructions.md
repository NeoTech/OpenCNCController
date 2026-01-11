# OpenCNC Copilot Instructions

## Architecture Overview

OpenCNC is a **modular real-time CNC control system** with clear separation between Windows-side components (soft real-time) and firmware (hard real-time). Data flows: `HMI → G-code Parser → Trajectory Planner → Comm Layer → USB → Firmware → Step Pulses`.

**Key architectural decision**: The Windows side buffers 100+ motion segments ahead; firmware holds 16-64 segments. This decouples non-RT Windows from RT motion control.

## Component Map

| Directory | Library/Target | Purpose |
|-----------|----------------|---------|
| `hmi/` | `opencnc_hmi.h` (header-only) | UI-agnostic controller interface |
| `gcode_parser/` | `ngc_parser` | RS274/NGC G-code → canonical commands |
| `trajectory_planner/` | `traj_planner` | Motion planning, velocity profiles |
| `comm/` | `cnc_comm` | USB/serial protocol, packet framing |
| `firmware/` | Platform-specific | ESP32/STM32/Pico/FPGA step generation |
| `config/` | `cnc_config` | TOML machine configuration |

## Build Commands

### Windows Host (HMI + Libraries)
```bash
# Full build (MinGW64 + Ninja) - run from project root
mkdir build && cd build
cmake -G Ninja -DCMAKE_BUILD_TYPE=Release ..
ninja

# With Clang (preferred for better diagnostics)
cmake -G Ninja -DCMAKE_C_COMPILER=clang -DCMAKE_CXX_COMPILER=clang++ ..

# Run tests
ctest --output-on-failure
```

### Component Build Order
When building incrementally or debugging dependencies:
1. `config/` → `cnc_config` (no dependencies)
2. `gcode_parser/` → `ngc_parser` (no dependencies)
3. `trajectory_planner/` → `traj_planner` (no dependencies)
4. `comm/` → `cnc_comm` (no dependencies)
5. `hmi/` → header-only (depends on comm)
6. `examples/` → (depends on all above)
7. `tests/` → (depends on all libraries)

### Firmware Toolchains (FreeRTOS-based)
All firmware targets use **FreeRTOS** for consistent task/timing model:

```bash
# ESP32-S3 (ESP-IDF + FreeRTOS built-in)
cd firmware/esp32
idf.py set-target esp32s3
idf.py build
idf.py -p COM3 flash monitor

# STM32 (STM32CubeIDE or arm-none-eabi-gcc + FreeRTOS)
cd firmware/stm32
mkdir build && cd build
cmake -DCMAKE_TOOLCHAIN_FILE=../cmake/arm-none-eabi.cmake ..
make -j
st-flash write opencnc_hal.bin 0x8000000

# Raspberry Pi Pico (Pico SDK + FreeRTOS port)
cd firmware/pico
export PICO_SDK_PATH=/path/to/pico-sdk
mkdir build && cd build
cmake ..
make -j
# Drag-drop opencnc_hal.uf2 to Pico in BOOTSEL mode

# FPGA (Yosys + nextpnr open toolchain)
cd firmware/fpga
yosys -p "synth_ice40 -top cnc_top -json cnc_top.json" rtl/*.v
nextpnr-ice40 --up5k --json cnc_top.json --pcf constraints/pins.pcf --asc cnc_top.asc
icepack cnc_top.asc cnc_top.bin
iceprog cnc_top.bin
```

### Test Fixture (Pico Display 2.0)
Hardware-in-the-loop test device for HMI development without CNC hardware:

```bash
# Build test fixture firmware
cd firmware/pico_test_fixture
mkdir build && cd build
cmake -G Ninja ..
ninja

# Flash: hold BOOTSEL, plug USB, copy pico_test_fixture.uf2 to RPI-RP2 drive
```

**USB Identity**: VID `0xCAFE`, PID `0x4001`, appears as "OpenCNC Test Fixture"

**Button Controls**:
- **A**: Cycle mode (NORMAL → HOMING → FAULT → JOG)
- **B**: Mode-specific action (cycle state, start homing, inject alarm, cycle axis)
- **X/Y**: Jog or fault selection depending on mode

See [pico_test_fixture/README.md](firmware/pico_test_fixture/README.md) for full documentation.

## Code Conventions

### C++ Style
- **Standard**: C++17 required (`std::optional`, `std::variant`, structured bindings)
- **Namespaces**: Match directory - `ngc_parser::`, `cnc_comm::`, `opencnc::`
- **Header guards**: `#ifndef NGC_PARSER_H` style (not `#pragma once`)
- **Constants**: `constexpr` in namespaces, e.g., `namespace MotionFlags { constexpr uint8_t RAPID = 0x01; }`

### Firmware (C)
- **Standard**: C11 with `extern "C"` wrappers
- **Types**: Use `stdint.h` types (`int32_t`, `uint8_t`) - see [hal_common.h](firmware/common/hal_common.h)
- **Naming**: `snake_case` for functions/types (`machine_state_t`, `hal_motion_start`)

### Protocol Encoding
- Positions encoded as **nanometers** (int32_t): `mm * 1_000_000`
- Packet structure: `0xAA | CMD | SEQ | LEN | PAYLOAD | CRC16 | 0x55`
- CRC: CRC-16-CCITT - see `calculateCRC16()` in [cnc_protocol.h](comm/include/cnc_protocol.h)

## Key Patterns

### Header-Only Library Pattern (HMI)
```cpp
// In ONE .cpp file only:
#define OPENCNC_HMI_IMPLEMENTATION
#include "opencnc_hmi.h"
```

### Modal State Machine (Parser)
G-code parser maintains modal groups per RS274/NGC. When adding G-codes, identify the modal group and update `ModalState` accordingly. See `MotionMode`, `PlaneSelect` enums.

### Ring Buffer for Motion Queue
Lock-free `RingBuffer<T, Capacity>` in [traj_queue.h](trajectory_planner/include/traj_queue.h) - use for RT-safe inter-thread communication.

### Firmware HAL Interface
All platforms implement the same API from [hal_common.h](firmware/common/hal_common.h):
- `hal_motion_queue_push()` / `hal_motion_queue_pop()`
- `hal_motion_start()` / `hal_motion_pause()` / `hal_motion_stop()`
- `hal_get_status()` / `hal_get_position()`

## Testing

Tests use GoogleTest (fetched via CMake). Each component has dedicated tests:
- `parser_tests.cpp` - G-code parsing, modal states
- `planner_tests.cpp` - Trajectory segments, ring buffer
- `config_tests.cpp` - TOML parsing, config values
- `protocol_tests.cpp` - CRC, packet encode/decode

## Cross-Component Concerns

- **Position units**: Windows uses `double` mm; protocol uses `int32_t` nanometers; firmware uses `steps_t`
- **State enums**: Duplicate definitions exist in HMI, comm, and firmware - keep synchronized
- **Axis limits**: `MAX_AXES` varies (9 in HMI/parser, 6 in firmware) - respect per-component limits

## System State Synchronization (CiA402 Alignment)

Machine states are defined in multiple files and **must remain synchronized**. The state model aligns with **CiA402** (CANopen device profile for drives):

| State | HMI (`opencnc::MachineState`) | Comm (`StatusFlags::STATE_*`) | Firmware (`machine_state_t`) | CiA402 Equivalent |
|-------|-------------------------------|-------------------------------|------------------------------|-------------------|
| 0 | `DISCONNECTED` | - | - | Not ready to switch on |
| 1 | `IDLE` | `STATE_IDLE` | `STATE_IDLE` | Switched on |
| 2 | `RUNNING` | `STATE_RUNNING` | `STATE_RUNNING` | Operation enabled |
| 3 | `PAUSED` | `STATE_PAUSED` | `STATE_PAUSED` | Quick stop active |
| 4 | `HOMING` | `STATE_HOMING` | `STATE_HOMING` | Homing (manufacturer-specific) |
| 5 | `JOG` | `STATE_JOG` | `STATE_JOG` | Operation enabled (jog mode) |
| 6 | `PROBING` | `STATE_PROBING` | `STATE_PROBING` | Operation enabled (probe mode) |
| 14 | `ALARM` | `STATE_ALARM` | `STATE_ALARM` | Fault |
| 15 | `ESTOP` | `STATE_ESTOP` | `STATE_ESTOP` | Fault reaction active |

**Source of truth**: `firmware/common/hal_common.h` defines canonical values. HMI and comm layers must mirror these.

### CiA402 Motor Control Considerations
For future servo/stepper integration via CANopen or EtherCAT:
- Controlword/Statusword pattern from CiA402 should map to command packets
- State transitions follow CiA402 state machine (Fault → Fault Reset → Switch On → Enable)
- Homing methods (CiA402 0x6098) align with `home_direction` in config
- Target position (0x607A) uses same nanometer encoding as protocol

## FreeRTOS Task Architecture (Firmware)

All firmware platforms use FreeRTOS with consistent task structure:

| Task | Priority | Core (if dual) | Purpose |
|------|----------|----------------|---------|
| `motion_task` | Realtime (highest) | Core 1 | Motion queue, step timing |
| `comm_task` | High | Core 0 | USB/UART packet processing |
| `status_task` | Normal | Core 0 | 50Hz status broadcast |
| `watchdog_task` | Low | Core 0 | System health, E-stop monitoring |

Step generation uses **hardware timer ISR**, not a FreeRTOS task, for sub-microsecond determinism.

## CANopen Distributed Architecture (ESP32-P4)

Alternative architecture using ESP32-P4 as central CAN-FD master with distributed axis nodes.

### Hardware Setup

| Component | Role | Interface |
|-----------|------|-----------|
| ESP32-P4 | CANopen Master | 3× TWAI (native CAN-FD), Ethernet |
| STM32G474 | Axis Node | Native FDCAN |
| RP2040 | Axis Node | MCP2518FD (SPI) |
| TJA1051 | CAN Transceiver | CAN bus to MCU |

### CAN-FD Configuration

```c
// Timing for 500 kbps arbitration, 2 Mbps data
// 170 MHz clock (STM32G4)
FDCAN_NBTP = { .NBRP = 17, .NTSEG1 = 15, .NTSEG2 = 4, .NSJW = 4 };
FDCAN_DBTP = { .DBRP = 5,  .DTSEG1 = 6,  .DTSEG2 = 3, .DSJW = 3 };
```

### CANopen COB-ID Assignments

| COB-ID | Function | Direction | Description |
|--------|----------|-----------|-------------|
| 0x000 | NMT | Master→All | Network management |
| 0x080 | SYNC | Master→All | 1 kHz synchronization |
| 0x180+NodeID | TPDO1 | Node→Master | Statusword + Position |
| 0x200+NodeID | RPDO1 | Master→Node | Controlword + Target |
| 0x580+NodeID | SDO_TX | Node→Master | SDO response |
| 0x600+NodeID | SDO_RX | Master→Node | SDO request |
| 0x700+NodeID | Heartbeat | Node→Master | 1 Hz alive |
| 0x7E0-0x7E4 | Auto-Assign | Both | Custom discovery |

### Auto-Assign Protocol

Custom protocol for automatic node ID assignment:

```c
// COB-IDs
#define AUTOASSIGN_COB_DISCOVERY  0x7E0  // Master broadcasts
#define AUTOASSIGN_COB_RESPONSE   0x7E1  // Node responds
#define AUTOASSIGN_COB_ASSIGN     0x7E2  // Master assigns ID
#define AUTOASSIGN_COB_ACK        0x7E3  // Node acknowledges
#define AUTOASSIGN_COB_REPORT     0x7E4  // Node reports config

// Discovery frame (8 bytes)
typedef struct {
    uint8_t cmd;        // AUTOASSIGN_CMD_DISCOVER
    uint8_t bus_id;     // Which CAN bus (0-2)
    uint8_t reserved[6];
} autoassign_discover_t;

// Response frame (CAN-FD 64 bytes)
typedef struct {
    uint8_t  cmd;           // AUTOASSIGN_CMD_RESPONSE
    uint8_t  unique_id[8];  // Hardware unique ID
    uint8_t  node_type;     // NODE_TYPE_STEPPER, etc.
    uint8_t  axis;          // AXIS_X through AXIS_W
    uint16_t capabilities;  // CAP_HOMING, CAP_ENCODER, etc.
    char     name[16];      // Human-readable name
    // ... motion parameters
} autoassign_response_t;
```

### CiA 402 Drive Profile Objects

| Index | Name | Type | Access | Description |
|-------|------|------|--------|-------------|
| 0x6040 | Controlword | UINT16 | RW | Enable, fault reset, homing |
| 0x6041 | Statusword | UINT16 | RO | Ready, enabled, fault, homing done |
| 0x6060 | Modes of Operation | INT8 | RW | 1=PP, 7=IP, 8=CSP, 6=Homing |
| 0x607A | Target Position | INT32 | RW | Position in steps |
| 0x6064 | Position Actual | INT32 | RO | Current position |
| 0x60FF | Target Velocity | INT32 | RW | Velocity in steps/s |
| 0x606C | Velocity Actual | INT32 | RO | Current velocity |
| 0x60FD | Digital Inputs | UINT32 | RO | Limit switches, home |
| 0x60FE | Digital Outputs | UINT32 | RW | Enable, aux outputs |

### CiA 402 State Machine

```
                    ┌──────────────────────────┐
                    │                          │
                    ▼                          │
┌─────────────┐   Fault   ┌─────────────┐     │
│ Not Ready   ├─────────►│   Fault     │─────┘
│ to Switch On│           │             │  Fault Reset
└──────┬──────┘           └─────────────┘
       │                         ▲
       │ Automatic               │ Fault
       ▼                         │
┌─────────────┐           ┌──────┴──────┐
│ Switch On   │  Enable   │  Operation  │
│ Disabled    ├──────────►│  Enabled    │
└──────┬──────┘           └──────┬──────┘
       │                         │
       │ Shutdown                │ Disable
       ▼                         ▼
┌─────────────┐           ┌─────────────┐
│ Ready to    │◄──────────│ Switched On │
│ Switch On   │  Shutdown │             │
└─────────────┘           └─────────────┘
```

### Controlword Bit Definitions

```c
#define CIA402_CW_SWITCH_ON       (1 << 0)   // Bit 0
#define CIA402_CW_ENABLE_VOLTAGE  (1 << 1)   // Bit 1
#define CIA402_CW_QUICK_STOP      (1 << 2)   // Bit 2 (0 = quick stop)
#define CIA402_CW_ENABLE_OP       (1 << 3)   // Bit 3
#define CIA402_CW_NEW_SETPOINT    (1 << 4)   // Bit 4 (mode-specific)
#define CIA402_CW_FAULT_RESET     (1 << 7)   // Bit 7
#define CIA402_CW_HALT            (1 << 8)   // Bit 8
```

### Statusword Bit Definitions

```c
#define CIA402_SW_READY           (1 << 0)   // Bit 0: Ready to switch on
#define CIA402_SW_SWITCHED_ON     (1 << 1)   // Bit 1: Switched on
#define CIA402_SW_OP_ENABLED      (1 << 2)   // Bit 2: Operation enabled
#define CIA402_SW_FAULT           (1 << 3)   // Bit 3: Fault
#define CIA402_SW_VOLTAGE_EN      (1 << 4)   // Bit 4: Voltage enabled
#define CIA402_SW_QUICK_STOP      (1 << 5)   // Bit 5: Quick stop
#define CIA402_SW_SWITCH_ON_DIS   (1 << 6)   // Bit 6: Switch on disabled
#define CIA402_SW_WARNING         (1 << 7)   // Bit 7: Warning
#define CIA402_SW_REMOTE          (1 << 9)   // Bit 9: Remote (CANopen control)
#define CIA402_SW_TARGET_REACHED  (1 << 10)  // Bit 10: Target reached
#define CIA402_SW_HOMING_DONE     (1 << 12)  // Bit 12: Homing complete
```

### ESP32-P4 Build Commands

```bash
# ESP32-P4 Controller
cd firmware/esp32_p4_controller
idf.py set-target esp32p4
idf.py build
idf.py -p COM3 flash monitor
```

### STM32G4 Axis Node Build

```bash
# STM32G4 Axis Node
cd firmware/axis_node_stm32
mkdir build && cd build
cmake -DCMAKE_TOOLCHAIN_FILE=../toolchain-arm-none-eabi.cmake ..
make
st-flash write axis_node_stm32.bin 0x08000000
```

### RP2040 Axis Node Build

```bash
# RP2040 Axis Node (with MCP2518FD)
cd firmware/axis_node_rp2040
mkdir build && cd build
cmake ..
make
# Drag-drop axis_node_rp2040.uf2 to Pico in BOOTSEL mode
```

### Node Configuration (TOML)

Each axis node stores configuration in flash:

```toml
[node]
type = "stepper"        # stepper, servo, io_expander
axis = "X"              # X, Y, Z, A, B, C, U, V, W
bus = 0                 # CAN bus index (0-2)

[motion]
max_velocity = 50000        # steps/s
max_acceleration = 100000   # steps/s^2
steps_per_mm = 200          # microsteps * steps/rev / mm/rev

[homing]
velocity = 10000            # steps/s
velocity_slow = 1000        # steps/s for second pass
backoff = 200               # steps to back off after limit
direction = -1              # -1 = toward min, +1 = toward max

[limits]
min_position = -100000      # steps
max_position = 100000       # steps
enable_soft_limits = true

[pins]
step = "PA0"
dir = "PA1"
enable = "PA2"
limit_min = "PA3"
limit_max = "PA4"
```

### P4 Standalone Mode

ESP32-P4 can operate without Windows for basic functions:

| Control | Function |
|---------|----------|
| Rotary Encoder | Jog selected axis |
| Button A | Select axis (X→Y→Z→A) |
| Button B | Change jog step size (0.01→0.1→1.0→10.0 mm) |
| Button C | Start homing sequence |
| E-Stop | Hardware emergency stop input |
| Display | Position DRO, status, alarms |

### Shared CANopen Header Files

Located in `firmware/common/canopen/`:

| File | Purpose |
|------|---------|
| `canopen_types.h` | CAN-FD frames, COB-IDs, function codes |
| `cia402.h` | Drive profile OD indices, state machine |
| `nmt.h` | Network management master/slave |
| `sdo.h` | Service data object transfer |
| `pdo.h` | Process data object mappings |
| `auto_assign.h` | Custom node discovery protocol |
| `node_config.h` | Configuration structure, flash storage |
