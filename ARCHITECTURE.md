# OpenCNC - Real-Time CNC Control System Architecture

## Overview

OpenCNC is a modular, real-time CNC control system designed with the following principles:
- **Separation of Concerns**: HMI, Motion Control, and Hardware Abstraction are independent
- **Real-Time Determinism**: Critical motion control runs on dedicated microcontrollers/FPGAs
- **Flexibility**: Support multiple UI frameworks and hardware targets
- **Standards Compliance**: NGC G-code compatible (RS274/NGC - LinuxCNC dialect)

```
┌─────────────────────────────────────────────────────────────────────────┐
│                        Windows Desktop (Non-RT)                          │
│  ┌──────────────────────────────────────────────────────────────────┐   │
│  │                    HMI Layer (opencnc_hmi.h)                      │   │
│  │  ┌─────────────┐  ┌─────────────┐  ┌─────────────────────────┐   │   │
│  │  │   Win32     │  │    Qt5      │  │     Ultralight/WebView  │   │   │
│  │  │   Native    │  │   Widgets   │  │        (HTML/CSS/JS)    │   │   │
│  │  └─────────────┘  └─────────────┘  └─────────────────────────┘   │   │
│  └──────────────────────────────────────────────────────────────────┘   │
│                                  │                                       │
│  ┌──────────────────────────────────────────────────────────────────┐   │
│  │              G-Code Parser (libngc_parser)                        │   │
│  │         NGC/RS274 Compatible - LinuxCNC Derived                   │   │
│  └──────────────────────────────────────────────────────────────────┘   │
│                                  │                                       │
│  ┌──────────────────────────────────────────────────────────────────┐   │
│  │           Trajectory Planner (libtraj_planner)                    │   │
│  │    Look-ahead │ Jerk-limited │ Arc interpolation │ Blending      │   │
│  └──────────────────────────────────────────────────────────────────┘   │
│                                  │                                       │
│  ┌──────────────────────────────────────────────────────────────────┐   │
│  │            Communication Layer (libcnc_comm)                      │   │
│  │         USB CDC/HID │ Serial │ Ethernet (future)                  │   │
│  └──────────────────────────────────────────────────────────────────┘   │
└─────────────────────────────────────────────────────────────────────────┘
                                   │
                              USB / Serial
                                   │
┌─────────────────────────────────────────────────────────────────────────┐
│                      HAL Firmware (Real-Time)                            │
│  ┌──────────────────────────────────────────────────────────────────┐   │
│  │                   Motion Controller Core                          │   │
│  │    Step Generation │ PID Loops │ Limit Switches │ Spindle        │   │
│  └──────────────────────────────────────────────────────────────────┘   │
│                                                                          │
│  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐  ┌──────────────┐   │
│  │  ESP32-S3   │  │   STM32     │  │  Pico Pi    │  │    FPGA      │   │
│  │  (FreeRTOS) │  │  (HAL/LL)   │  │  (Pico SDK) │  │   (Verilog)  │   │
│  └─────────────┘  └─────────────┘  └─────────────┘  └──────────────┘   │
└─────────────────────────────────────────────────────────────────────────┘
```

## Component Details

### 1. HMI Layer (`hmi/`)
- **Single Header Library**: `opencnc_hmi.h`
- **Purpose**: Provides unified API for CNC control regardless of UI framework
- **Features**:
  - Machine state management
  - Jog controls
  - Program loading/execution
  - DRO (Digital Readout) interface
  - Real-time position/velocity feedback
  - Alarm/error handling

### 2. G-Code Parser (`gcode_parser/`)
- **Library**: `libngc_parser`
- **Based on**: LinuxCNC RS274/NGC interpreter concepts
- **Features**:
  - Full NGC dialect support
  - Canned cycles (G81-G89)
  - Subroutines and parameters
  - Expressions and variables
  - Tool compensation (G41/G42)
  - Coordinate systems (G54-G59.3)

### 3. Trajectory Planner (`trajectory_planner/`)
- **Library**: `libtraj_planner`
- **Features**:
  - Trapezoidal and S-curve velocity profiles
  - N-segment look-ahead buffer
  - Corner blending with tolerance
  - Arc interpolation (G2/G3)
  - Jerk-limited motion
  - Acceleration constraint handling

### 4. Communication Layer (`comm/`)
- **Library**: `libcnc_comm`
- **Protocols**:
  - USB CDC (Virtual COM Port) - Primary
  - USB HID (Custom) - Low latency
  - Serial (UART) - Fallback
- **Features**:
  - Packet framing with CRC
  - Command/response protocol
  - Streaming position updates
  - Emergency stop handling

### 5. HAL Firmware (`firmware/`)
- **Targets**: ESP32-S3, STM32F4/G4/H7, RP2040 (Pico), FPGA
- **Features**:
  - High-frequency step pulse generation (up to 200kHz+)
  - Hardware timer-based timing
  - GPIO management for limits, home, probe
  - Spindle PWM/analog control
  - Encoder feedback (optional)

### 6. Configuration System (`config/`)
- **Format**: TOML primary, YAML secondary
- **Scope**:
  - Machine geometry (axes, limits, homing)
  - Kinematics (Cartesian, CoreXY, Delta)
  - Motor parameters (steps/mm, max velocity, acceleration)
  - I/O mapping (pins, polarity)
  - Communication settings

## Data Flow

```
User Input → HMI → G-Code Parser → Canonical Commands
                                          ↓
                                  Trajectory Planner
                                          ↓
                                  Motion Segments
                                          ↓
                                  Communication Layer
                                          ↓
                              USB/Serial to Hardware
                                          ↓
                                  HAL Firmware
                                          ↓
                              Step/Direction Pulses
                                          ↓
                                  Motor Drivers
```

## Distributed CANopen Architecture (ESP32-P4 Controller)

An alternative architecture using an ESP32-P4 as a central CAN-FD master with distributed axis nodes:

```
┌─────────────────────────────────────────────────────────────────────────┐
│                        Windows Desktop (Non-RT)                          │
│  ┌──────────────────────────────────────────────────────────────────┐   │
│  │                    HMI Layer (opencnc_hmi.h)                      │   │
│  │  ┌─────────────┐  ┌─────────────┐  ┌─────────────────────────┐   │   │
│  │  │   Win32     │  │    Qt5      │  │     Ultralight/WebView  │   │   │
│  │  │   Native    │  │   Widgets   │  │        (HTML/CSS/JS)    │   │   │
│  │  └─────────────┘  └─────────────┘  └─────────────────────────┘   │   │
│  └──────────────────────────────────────────────────────────────────┘   │
└─────────────────────────────────────────────────────────────────────────┘
                                   │
                             Ethernet TCP
                             (Port 5000)
                                   │
┌─────────────────────────────────────────────────────────────────────────┐
│                      ESP32-P4 Central Controller                         │
│  ┌──────────────────────────────────────────────────────────────────┐   │
│  │  Core 0: Network/UI                Core 1: Real-Time Motion       │   │
│  │  ├─ Ethernet TCP Server            ├─ CANopen Master             │   │
│  │  ├─ MIPI-DSI Display               ├─ Trajectory Interpolation   │   │
│  │  ├─ Standalone Mode (Jog/Home)     ├─ 1kHz SYNC Generation       │   │
│  │  └─ Configuration Manager          └─ Position Monitoring        │   │
│  └──────────────────────────────────────────────────────────────────┘   │
│                                                                          │
│  ┌────────────┐   ┌────────────┐   ┌────────────┐                       │
│  │  TWAI0     │   │   TWAI1    │   │   TWAI2    │                       │
│  │ (CAN-FD 0) │   │ (CAN-FD 1) │   │ (CAN-FD 2) │                       │
│  └─────┬──────┘   └─────┬──────┘   └─────┬──────┘                       │
└────────┼────────────────┼────────────────┼──────────────────────────────┘
         │                │                │
    CAN Bus 0        CAN Bus 1        CAN Bus 2
   (500k/2M FD)     (500k/2M FD)     (500k/2M FD)
         │                │                │
┌────────┼───────┐ ┌──────┼───────┐ ┌──────┼───────┐
│  TJA1051       │ │  TJA1051      │ │  TJA1051      │
│  Transceiver   │ │  Transceiver  │ │  Transceiver  │
└────────┬───────┘ └──────┬───────┘ └──────┬───────┘
         │                │                │
    ┌────┴────┐      ┌────┴────┐      ┌────┴────┐
    │         │      │         │      │         │
┌───┴──┐ ┌───┴──┐ ┌──┴───┐ ┌──┴───┐ ┌──┴───┐ ┌──┴───┐
│Node 1│ │Node 2│ │Node 3│ │Node 4│ │Node 5│ │Node 6│
│ X    │ │ Y    │ │ Z    │ │ A    │ │ I/O  │ │Spare │
└──────┘ └──────┘ └──────┘ └──────┘ └──────┘ └──────┘
   │         │        │        │        │        │
   └─────────┴────────┴────────┴────────┴────────┘
                      │
            Stepper/Servo Drivers
```

### Hardware Components

| Component | Role | Key Features |
|-----------|------|--------------|
| ESP32-P4 | Central Controller | Dual-core 400MHz RISC-V, 3× TWAI (CAN-FD), MIPI-DSI, Ethernet EMAC |
| STM32G474 | Axis Node (Option 1) | 170MHz Cortex-M4F, Native FDCAN, 12-bit ADC |
| RP2040 | Axis Node (Option 2) | Dual Cortex-M0+, MCP2518FD external CAN controller |
| TJA1051 | CAN Transceiver | CAN-FD compatible, 5V tolerant, 2 Mbps data rate |

### CAN-FD Configuration

- **Arbitration Phase**: 500 kbps (17 time quanta at 170 MHz)
- **Data Phase**: 2 Mbps (10 time quanta at 170 MHz)
- **Frame Size**: Up to 64 bytes (CAN-FD)
- **Bus Topology**: 3 independent CAN buses, 3 nodes each max
- **Termination**: 120Ω at each bus end

### CANopen Protocol Stack

Based on CANopen DS301 with CiA 402 drive profile:

| COB-ID Range | Function | Description |
|--------------|----------|-------------|
| 0x000 | NMT | Network management commands |
| 0x080 | SYNC | Synchronization message (1 kHz) |
| 0x180-0x1FF | TPDO1 | Statusword + Position Actual |
| 0x200-0x27F | RPDO1 | Controlword + Target Position |
| 0x580-0x5FF | SDO TX | Object dictionary access |
| 0x600-0x67F | SDO RX | Object dictionary access |
| 0x700-0x77F | Heartbeat | Node alive monitoring |
| 0x7E0-0x7E4 | Auto-Assign | Custom node discovery protocol |

### Auto-Assign Protocol

Custom protocol for automatic node ID assignment at boot:

1. **Discovery** (0x7E0): Master broadcasts discovery request
2. **Response** (0x7E1): Nodes respond with unique ID and capabilities
3. **Assign** (0x7E2): Master assigns node ID based on axis/type
4. **Acknowledge** (0x7E3): Node confirms assignment
5. **Report** (0x7E4): Node reports full configuration

Node configuration is self-describing via TOML stored in flash:

```toml
[node]
type = "stepper"
axis = "X"
bus = 0

[motion]
max_velocity = 50000      # steps/s
max_acceleration = 100000 # steps/s^2
steps_per_mm = 200

[homing]
velocity = 10000
direction = -1
```

### CiA 402 Drive Profile

Object Dictionary entries for motion control:

| Index | Object | Access | Description |
|-------|--------|--------|-------------|
| 0x6040 | Controlword | RW | Drive control commands |
| 0x6041 | Statusword | RO | Drive status |
| 0x6060 | Modes of Operation | RW | CSP, CSV, Homing |
| 0x607A | Target Position | RW | Position setpoint |
| 0x6064 | Position Actual | RO | Current position |
| 0x60FF | Target Velocity | RW | Velocity setpoint |
| 0x606C | Velocity Actual | RO | Current velocity |

### Standalone Mode (P4)

ESP32-P4 can operate without Windows connection for:
- **Jogging**: Manual axis movement via rotary encoder
- **Homing**: Execute homing sequence for all/selected axes
- **Status Display**: MIPI-DSI touchscreen shows position, limits, alarms
- **E-Stop**: Hardware emergency stop input

### Firmware Directory Structure

```
firmware/
├── common/
│   └── canopen/              # Shared CANopen headers
│       ├── canopen_types.h   # CAN-FD frames, COB-IDs
│       ├── cia402.h          # Drive profile
│       ├── nmt.h             # Network management
│       ├── sdo.h             # Service data objects
│       ├── pdo.h             # Process data objects
│       ├── auto_assign.h     # Node discovery
│       └── node_config.h     # Configuration schema
├── esp32_p4_controller/      # Central controller
│   ├── main/
│   │   ├── main.c
│   │   ├── canopen_master.c
│   │   ├── eth_comm.c
│   │   ├── trajectory_task.c
│   │   ├── display_task.c
│   │   └── standalone_mode.c
│   └── README.md
├── axis_node_stm32/          # STM32G4 axis node
│   ├── Core/
│   │   ├── Src/main.c
│   │   ├── Src/canopen_slave.c
│   │   └── Src/stepper_control.c
│   └── README.md
└── axis_node_rp2040/         # RP2040 axis node
    ├── src/
    │   ├── main.c
    │   ├── mcp2518fd.c       # External CAN controller
    │   └── canopen_slave.c
    └── README.md
```

## Real-Time Considerations

### Windows Side (Soft Real-Time)
- Trajectory planning runs ahead of execution
- Large motion buffer (100+ segments)
- Communication runs in dedicated thread
- Position polling at 50-100Hz

### ESP32-P4 Controller (Medium Real-Time)
- FreeRTOS with task pinning to cores
- Core 1 dedicated to real-time motion
- 1 kHz SYNC generation for axis synchronization
- Motion buffer interpolation

### Axis Nodes (Hard Real-Time)
- Interrupt-driven step generation
- Deterministic timing via hardware timers
- SYNC-triggered PDO exchange
- Sub-microsecond step timing accuracy

## Build System

All components use **CMake** with **Ninja** generator:
- Windows: MinGW64 with GCC/Clang
- Firmware: Platform-specific toolchains (ESP-IDF, STM32CubeIDE, Pico SDK)

## Directory Structure

```
realtime-cnc/
├── ARCHITECTURE.md
├── README.md
├── CMakeLists.txt              # Top-level build
├── hmi/                        # Windows HMI library
├── gcode_parser/               # NGC G-code parser
├── trajectory_planner/         # Motion planning library
├── comm/                       # Communication layer
├── firmware/                   # HAL firmware for MCUs/FPGAs
├── config/                     # Configuration system
├── examples/                   # Example applications
├── tests/                      # Unit and integration tests
└── docs/                       # Documentation
```
