# OpenCNC - Real-Time CNC Control System

A modular, real-time CNC control system with Windows HMI, trajectory planning, and support for low-cost microcontroller HAL devices (ESP32-S3, STM32, Raspberry Pi Pico, FPGA).

## Features

- **NGC G-Code Compatible**: RS274/NGC dialect parser (LinuxCNC compatible)
- **Advanced Trajectory Planning**: Look-ahead, S-curve profiles, jerk-limited motion
- **Flexible HMI**: Single-header library for Win32, Qt5, or Ultralight integration
- **Multiple Hardware Targets**: ESP32-S3, STM32F4/G4/H7, RP2040, FPGA
- **USB Communication**: Low-latency USB CDC/HID protocol
- **TOML/YAML Configuration**: Easy machine setup and customization

## System Architecture

```
┌─────────────────────────────────────────┐
│         Windows HMI Application         │
│   (Win32 / Qt5 / Ultralight WebView)    │
├─────────────────────────────────────────┤
│     opencnc_hmi.h (Header-only lib)     │
├─────────────────────────────────────────┤
│  G-Code Parser │ Trajectory Planner     │
├─────────────────────────────────────────┤
│       Communication Layer (USB)         │
└───────────────────┬─────────────────────┘
                    │ USB/Serial
┌───────────────────┴─────────────────────┐
│      HAL Firmware (Real-Time MCU)       │
│   ESP32-S3 / STM32 / Pico / FPGA        │
└─────────────────────────────────────────┘
```

## Building

### Prerequisites

- **Windows**: MinGW64 with GCC or Clang
- **Build System**: CMake 3.20+, Ninja
- **Optional**: Qt5 (for Qt HMI example)

### Quick Start

```bash
# Clone and build
cd realtime-cnc
mkdir build && cd build
cmake -G Ninja -DCMAKE_BUILD_TYPE=Release ..
ninja

# Run tests
ctest --output-on-failure
```

### Using Clang (recommended)

```bash
cmake -G Ninja -DCMAKE_C_COMPILER=clang -DCMAKE_CXX_COMPILER=clang++ ..
ninja
```

## Project Structure

```
realtime-cnc/
├── hmi/                    # Windows HMI header-only library
│   └── include/
│       └── opencnc_hmi.h   # Drop into any UI project
├── gcode_parser/           # NGC G-code parser library
├── trajectory_planner/     # Motion planning library
├── comm/                   # USB communication layer
├── firmware/               # HAL firmware for MCUs/FPGAs
│   ├── esp32/
│   ├── stm32/
│   ├── pico/
│   └── fpga/
├── config/                 # TOML/YAML configuration system
├── examples/               # Example applications
│   ├── win32_minimal/
│   ├── qt5_hmi/
│   └── ultralight_hmi/
└── tests/                  # Unit and integration tests
```

## Quick Integration

### Header-Only HMI (Simplest)

```cpp
#define OPENCNC_HMI_IMPLEMENTATION
#include "opencnc_hmi.h"

int main() {
    opencnc::Controller cnc;
    
    if (cnc.connect("COM3")) {
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

## VS Code IntelliSense Setup

The project includes multiple configurations for different targets (Windows host, ESP32, STM32, Pico, FPGA). To enable full IntelliSense:

### 1. Set SDK Environment Variables

Set these environment variables to point to your SDK installations:

```cmd
# ESP-IDF (ESP32 development)
setx IDF_PATH "C:\Espressif\frameworks\esp-idf-v5.1"

# Raspberry Pi Pico SDK
setx PICO_SDK_PATH "C:\SDKs\pico-sdk"

# STM32 CubeMX HAL drivers
setx STM32_CUBE_PATH "C:\STM32Cube\Repository\STM32Cube_FW_F4_V1.28.0"
```

Alternatively, place SDKs in `~/SDKs/` with standard names:
- `~/SDKs/FreeRTOS-Kernel/`
- `~/SDKs/pico-sdk/`

### 2. Install Required Extensions

- **C/C++** (`ms-vscode.cpptools`) - IntelliSense and debugging
- **CMake Tools** (`ms-vscode.cmake-tools`) - Build integration

### 3. Select Configuration

Use the status bar selector (bottom-right) or `Ctrl+Shift+P` → "C/C++: Select a Configuration":

| Configuration | Use When |
|---------------|----------|
| **Win32 (Host)** | Editing HMI, parser, planner, comm libraries |
| **ESP32-S3 (FreeRTOS)** | Editing `firmware/esp32/` |
| **STM32F4 (FreeRTOS)** | Editing `firmware/stm32/` |
| **Raspberry Pi Pico (FreeRTOS)** | Editing `firmware/pico/` |
| **FPGA (Verilog)** | Editing `firmware/fpga/` |

### 4. Configure CMake Tools

For Windows host builds, let CMake Tools provide include paths:

```json
// settings.json
{
    "cmake.configureOnOpen": true,
    "cmake.generator": "Ninja"
}
```

## Configuration Example

```toml
# machine.toml
[machine]
name = "3-Axis Mill"
kinematics = "cartesian"

[axes.x]
steps_per_mm = 800
max_velocity = 5000      # mm/min
max_acceleration = 500   # mm/s²
max_travel = 300         # mm

[axes.y]
steps_per_mm = 800
max_velocity = 5000
max_acceleration = 500
max_travel = 200

[axes.z]
steps_per_mm = 1600
max_velocity = 2000
max_acceleration = 200
max_travel = 100

[spindle]
type = "pwm"
min_rpm = 1000
max_rpm = 24000

[communication]
type = "usb_cdc"
port = "auto"
baud = 115200
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
