# Configuration System

TOML-based configuration for OpenCNC machine setup.

## Overview

The configuration system uses TOML format for human-readable machine configuration. It supports:

- Axis parameters (steps/mm, velocity, acceleration, travel limits)
- Kinematics type (Cartesian, CoreXY, Delta)
- Spindle settings (RPM, PWM frequency)
- Coolant control
- Communication settings
- Pin mapping

## Usage

### C++ API

```cpp
#include "cnc_config.h"

using namespace cnc::config;

// Load from file
MachineConfig config = MachineConfig::loadFromFile("machine.toml");

// Access configuration
std::cout << "Machine: " << config.name << "\n";
std::cout << "Axes: " << config.axes.size() << "\n";

for (const auto& axis : config.axes) {
    std::cout << axis.name << ": " 
              << axis.stepsPerMm << " steps/mm, "
              << axis.maxVelocity << " mm/min\n";
}
```

### Configuration File

```toml
[machine]
name = "My CNC"
version = "1.0"

[[axis]]
name = "X"
steps_per_mm = 800.0
max_velocity = 5000.0
max_acceleration = 500.0
max_travel = 300.0

[[axis]]
name = "Y"
steps_per_mm = 800.0
max_velocity = 5000.0
max_acceleration = 500.0
max_travel = 200.0

[[axis]]
name = "Z"
steps_per_mm = 400.0
max_velocity = 1500.0
max_acceleration = 200.0
max_travel = 100.0

[spindle]
max_rpm = 24000
pwm_frequency = 5000

[communication]
port = "COM3"
baud_rate = 115200
```

## Configuration Reference

### [machine]

| Key | Type | Description |
|-----|------|-------------|
| name | string | Machine name |
| version | string | Configuration version |

### [[axis]]

| Key | Type | Default | Description |
|-----|------|---------|-------------|
| name | string | required | Axis name (X, Y, Z, A, B, C) |
| steps_per_mm | float | 800.0 | Steps per millimeter |
| max_velocity | float | 5000.0 | Maximum velocity (mm/min) |
| max_acceleration | float | 500.0 | Maximum acceleration (mm/s²) |
| max_travel | float | 200.0 | Maximum travel distance (mm) |
| home_position | float | 0.0 | Position after homing (mm) |
| home_direction | int | -1 | Homing direction (-1 or +1) |
| home_feedrate | float | 500.0 | Homing velocity (mm/min) |
| home_pulloff | float | 2.0 | Pull-off after homing (mm) |
| step_invert | bool | false | Invert step signal |
| dir_invert | bool | false | Invert direction signal |
| limit_invert | bool | false | Invert limit switch signal |

### [kinematics]

| Key | Type | Default | Description |
|-----|------|---------|-------------|
| type | string | "cartesian" | Kinematics type |
| junction_deviation | float | 0.02 | Corner rounding (mm) |
| arc_tolerance | float | 0.002 | Arc linearization (mm) |
| soft_limits | bool | true | Enable software limits |
| hard_limits | bool | true | Enable hardware limits |

### [spindle]

| Key | Type | Default | Description |
|-----|------|---------|-------------|
| enabled | bool | true | Spindle enabled |
| min_rpm | int | 0 | Minimum RPM |
| max_rpm | int | 24000 | Maximum RPM |
| pwm_frequency | int | 5000 | PWM frequency (Hz) |

### [communication]

| Key | Type | Default | Description |
|-----|------|---------|-------------|
| port | string | "" | Serial port |
| baud_rate | int | 115200 | Baud rate |
| timeout | int | 1000 | Timeout (ms) |
| status_poll_rate | int | 50 | Status polling (Hz) |

## Building

```bash
mkdir build && cd build
cmake ..
cmake --build .

# Test configuration parsing
./config_test ../machine.toml
```
