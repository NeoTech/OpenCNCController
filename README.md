# OpenCNC - Real-Time Distributed CNC Control System

A modular, real-time CNC control system featuring a distributed CANopen architecture. The Qt6 HMI (Linux or Windows) communicates over Ethernet to an ESP32-P4 central controller, which manages distributed axis nodes (STM32G4/RP2040) via CAN-FD bus.

## Features

### Core Capabilities
- **RS274/NGC G-Code Parser**: Full LinuxCNC-compatible dialect
- **Advanced Trajectory Planning**: Look-ahead, S-curve profiles, jerk-limited motion
- **CiA 402 Drive Profile**: Industry-standard motor control state machine

### Distributed Architecture
- **ESP32-P4 Central Controller**: Dual-core 400 MHz RISC-V with 3× CAN-FD buses
- **Ethernet HMI Link**: TCP/IP communication (replaces USB tether)
- **CANopen Protocol**: 1 kHz SYNC, PDO real-time control, SDO configuration
- **Auto-Assign Node Discovery**: Plug-and-play axis node configuration

### Hardware Support
- **HMI Platforms**: LattePanda Iota (Linux), Windows PC, Raspberry Pi 5
- **Central Controller**: ESP32-P4 with Ethernet and MIPI display
- **Axis Nodes**: STM32G474 (native CAN-FD) or RP2040 (via MCP2518FD)
- **Standalone Mode**: ESP32-P4 can operate without HMI (jog, home, DRO display)

### Software
- **Qt6 HMI**: Primary UI framework with Wayland support for embedded Linux
- **Header-Only Library**: Drop `opencnc_hmi.h` into Qt6, Win32, or any C++ UI
- **Embedded Linux**: Buildroot/Yocto images with PREEMPT_RT kernel
- **TOML Configuration**: Machine and axis node setup
- **FreeRTOS Based**: Consistent task model across all firmware targets

## System Architecture

```
┌─────────────────────────────────────────────────────────────────────┐
│              HMI Host (Linux RT or Windows)                         │
│         LattePanda Iota / Raspberry Pi 5 / Windows PC               │
├─────────────────────────────────────────────────────────────────────┤
│   Qt6 HMI (Wayland)  │  G-Code Parser  │  Trajectory Planner        │
├─────────────────────────────────────────────────────────────────────┤
│                    Ethernet Communication Layer                     │
└─────────────────────────────────┬───────────────────────────────────┘
                                  │ TCP/IP (Ethernet)
┌─────────────────────────────────┴───────────────────────────────────┐
│                    ESP32-P4 Central Controller                      │
│         CANopen Master │ Motion Buffer │ Local UI (optional)        │
├─────────────────────────────────────────────────────────────────────┤
│         CAN-FD Bus 0        │  CAN-FD Bus 1     │  CAN-FD Bus 2     │
│       (X, Y, Z axes)        │   (A, B, C)       │   (Spindle, I/O)  │
└──────────┬──────────────────┴────────┬──────────┴────────┬──────────┘
           │                           │                   │
   ┌───────┴───────┐           ┌───────┴───────┐   ┌───────┴───────┐
   │  Axis Node    │           │  Axis Node    │   │  I/O Node     │
   │  STM32G474    │           │  RP2040 +     │   │  (Future)     │
   │  or RP2040    │           │  MCP2518FD    │   │               │
   └───────────────┘           └───────────────┘   └───────────────┘
```

## HMI Platform Options

| Platform | OS | Boot Time | Use Case |
|----------|-----|-----------|----------|
| **LattePanda Iota** | Linux RT (Buildroot/Yocto/Arch) | ~3-8s | Production control panel |
| **Raspberry Pi 5** | Linux RT | ~5-10s | Lower-cost option |
| **Windows PC** | Windows 10/11 | - | Development, operator station |

See [hmi/linux/README.md](hmi/linux/README.md) for embedded Linux setup with PREEMPT_RT kernel.

## CANopen Architecture

### Protocol Stack
| Layer | Implementation |
|-------|----------------|
| **NMT** | Network management, node state control |
| **SYNC** | 1 kHz synchronization for coordinated motion |
| **PDO** | Real-time position commands & feedback |
| **SDO** | Configuration and parameter access |
| **Auto-Assign** | Custom plug-and-play node discovery |

### CiA 402 Drive Profile
Each axis node implements the industry-standard CiA 402 state machine:
- Controlword/Statusword for state transitions
- Profile Position Mode (PP) and Cyclic Sync Position (CSP)
- Homing modes with configurable sequences
- Digital I/O for limits, home switches, enables

### CAN Bus Allocation
| Bus | Node IDs | Purpose |
|-----|----------|---------|
| CAN 0 | 1-3 | Primary axes (X, Y, Z) |
| CAN 1 | 4-6 | Secondary axes (A, B, C) |
| CAN 2 | 7-9 | Auxiliary (spindle, I/O expanders) |

## Building

### Prerequisites

- **Windows HMI**: MinGW64 with GCC/Clang, CMake 3.20+, Ninja
- **ESP32-P4**: ESP-IDF v5.2+
- **STM32G4**: ARM GCC, STM32CubeG4 HAL
- **RP2040**: Pico SDK

### Windows Host (HMI + Libraries)

```bash
mkdir build && cd build
cmake -G Ninja -DCMAKE_BUILD_TYPE=Release ..
ninja
ctest --output-on-failure
```

### ESP32-P4 Central Controller

```bash
cd firmware/esp32_p4_controller
idf.py set-target esp32p4
idf.py build
idf.py -p COM3 flash monitor
```

### STM32G4 Axis Node

```bash
cd firmware/axis_node_stm32
mkdir build && cd build
cmake -DCMAKE_TOOLCHAIN_FILE=../toolchain-arm-none-eabi.cmake ..
make
st-flash write axis_node_stm32.bin 0x08000000
```

### RP2040 Axis Node

```bash
cd firmware/axis_node_rp2040
mkdir build && cd build
cmake ..
make
# Drag-drop axis_node_rp2040.uf2 to Pico in BOOTSEL mode
```

## Project Structure

```
realtime-cnc/
├── hmi/                        # HMI library and platform support
│   ├── include/opencnc_hmi.h   # Header-only controller interface
│   └── linux/                  # Embedded Linux setup
│       ├── buildroot/          # Buildroot external tree
│       ├── yocto/              # Yocto meta-layer
│       └── arch/               # Arch Linux setup scripts
├── gcode_parser/               # RS274/NGC G-code parser
├── trajectory_planner/         # Motion planning with look-ahead
├── comm/                       # Ethernet communication layer
├── config/                     # TOML configuration system
├── firmware/
│   ├── esp32_p4_controller/    # Central CANopen master
│   ├── axis_node_stm32/        # STM32G474 axis node
│   ├── axis_node_rp2040/       # RP2040 + MCP2518FD axis node
│   ├── common/canopen/         # Shared CANopen headers
│   ├── pico_test_fixture/      # HMI development test device
│   ├── esp32/                  # Legacy single-MCU firmware
│   ├── pico/                   # Legacy single-MCU firmware
│   └── fpga/                   # FPGA step generator (advanced)
├── examples/
│   ├── win32/                  # Minimal Win32 example
│   └── qt6/                    # Qt6 HMI example
└── tests/                      # Unit tests (GoogleTest)
```

## Quick Integration

### Header-Only HMI

```cpp
#define OPENCNC_HMI_IMPLEMENTATION
#include "opencnc_hmi.h"

int main() {
    opencnc::Controller cnc;
    
    // Connect via Ethernet to ESP32-P4 controller
    if (cnc.connect("192.168.1.100", 5000)) {
        cnc.loadProgram("part.ngc");
        cnc.start();
        
        while (cnc.isRunning()) {
            auto pos = cnc.getCurrentPosition();
            printf("X: %.3f Y: %.3f Z: %.3f\n", pos.x, pos.y, pos.z);
        }
    }
    return 0;
}
```

## Standalone Mode (ESP32-P4)

The central controller can operate without a Windows HMI:

| Control | Function |
|---------|----------|
| Rotary Encoder | Jog selected axis |
| Button A | Select axis (X→Y→Z→A) |
| Button B | Change jog step (0.01→0.1→1.0→10.0 mm) |
| Button C | Start homing sequence |
| E-Stop Input | Hardware emergency stop |
| MIPI Display | Position DRO, status, alarms |

## Configuration

### Machine Configuration (Windows)

```toml
# machine.toml
[machine]
name = "3-Axis Mill"
kinematics = "cartesian"

[controller]
address = "192.168.1.100"
port = 5000

[axes.x]
steps_per_mm = 800
max_velocity = 5000      # mm/min
max_acceleration = 500   # mm/s²
max_travel = 300         # mm
```

### Axis Node Configuration (Flash)

```toml
# Stored in axis node flash
[node]
type = "stepper"
axis = "X"
bus = 0

[motion]
max_velocity = 50000        # steps/s
max_acceleration = 100000   # steps/s²
steps_per_mm = 800

[homing]
velocity = 10000
direction = -1              # Toward min limit
```

## License

This project is licensed under the **PolyForm Noncommercial License 1.0.0**.

**What this means:**
- ✅ Personal use, hobby CNC projects, learning, and experimentation
- ✅ Educational institutions, research organizations, and non-profits
- ✅ Forking, modifying, and sharing the code (under the same terms)
- ❌ Commercial use — cannot be sold or used in business operations
- ❌ Cannot be offered as a commercial service or embedded in commercial products

See the [LICENSE](LICENSE) file for full terms, or visit [polyformproject.org](https://polyformproject.org/licenses/noncommercial/1.0.0).

## Contributing

Contributions welcome! Please see CONTRIBUTING.md for guidelines.
