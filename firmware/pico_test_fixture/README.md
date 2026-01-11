# OpenCNC Test Fixture - Pico Display 2.0

A hardware-in-the-loop test device that emulates CNC firmware over USB, enabling Windows HMI development without real CNC hardware.

## Hardware Requirements

- **Raspberry Pi Pico WH** (or Pico W with headers)
- **Pimoroni Pico Display 2.0** (320×240 IPS LCD, 4 buttons, RGB LED)

## Features

- ✅ Full USB CDC communication (appears as COM port)
- ✅ Complete protocol implementation (CRC-16, packet framing)
- ✅ CiA402-aligned state machine emulation
- ✅ Motion queue simulation with trapezoidal profiles
- ✅ Real-time status broadcast at 50 Hz
- ✅ Visual feedback on 320×240 display
- ✅ Physical buttons for mode control and fault injection
- ✅ RGB LED status indication

## Display Layout

```
┌──────────────────────────────────────┐
│ TEST FIXTURE          Mode: NORMAL   │  Header (20px)
├──────────────────────────────────────┤
│ State: RUNNING        Alarm: NONE    │  Status (24px)
├──────────────────────────────────────┤
│ X [▓▓▓▓▓▓▓▓░░░░░░░░] +125.432 mm    │
│ Y [▓▓▓▓░░░░░░░░░░░░]  +52.100 mm    │  Position (72px)
│ Z [▓▓▓▓▓▓▓▓▓░░░░░░░]  -8.200 mm     │
├──────────────────────────────────────┤
│ Queue: 12/32    Feed: 2500 mm/min    │  Motion (24px)
├──────────────────────────────────────┤
│ TX:1523  RX:1519  CRC:0  NAK:0       │  Stats (20px)
├──────────────────────────────────────┤
│ > CMD: MOVE_LINEAR X100 Y50          │
│ > ACK sent, queued segment #12       │  Log (60px)
│ > Position update: X=100.000         │
├──────────────────────────────────────┤
│ [A]Mode [B]State [X]Act [Y]Alt       │  Hints (20px)
└──────────────────────────────────────┘
```

## Button Controls

### Mode Cycling (Button A)

Button A always cycles through operating modes:

| Mode | Purpose |
|------|---------|
| **NORMAL** | Standard operation, respond to USB commands |
| **HOMING** | Simulate homing sequences |
| **FAULT** | Inject faults/alarms for testing error handling |
| **JOG** | Direct axis control via buttons |

### Mode-Specific Actions

| Mode | Button B | Button X | Button Y |
|------|----------|----------|----------|
| **NORMAL** | Cycle state (IDLE→RUN→PAUSE) | Hold: Jog X+ | Hold: Jog Y+ |
| **HOMING** | Start home all | Select axis | Cancel homing |
| **FAULT** | Inject selected alarm | Next alarm type | Clear all alarms |
| **JOG** | Cycle axis (X→Y→Z) | Hold: Jog + | Hold: Jog - |

### Injectable Alarms

In FAULT mode, cycle through these with X, inject with B:

1. LIMIT_X (axis limit switch)
2. LIMIT_Y
3. LIMIT_Z
4. SOFT_LIMIT (software limit exceeded)
5. ESTOP (emergency stop)
6. PROBE_FAIL
7. HOME_FAIL
8. SPINDLE
9. COMM_ERROR

## USB Identity

| Property | Value |
|----------|-------|
| VID | `0xCAFE` (TinyUSB test) |
| PID | `0x4001` (unique) |
| Manufacturer | "OpenCNC" |
| Product | "Test Fixture (Pico Display 2.0)" |
| Serial | Pico unique board ID |

The fixture appears as a standard COM port (CDC device) on Windows.

## Building

### Prerequisites

1. **Pico SDK** (v1.5.0+)
   ```bash
   git clone https://github.com/raspberrypi/pico-sdk.git
   cd pico-sdk && git submodule update --init
   export PICO_SDK_PATH=/path/to/pico-sdk
   ```

2. **ARM Toolchain**
   - Windows: [Arm GNU Toolchain](https://developer.arm.com/downloads/-/arm-gnu-toolchain-downloads)
   - Or via MSYS2: `pacman -S mingw-w64-x86_64-arm-none-eabi-gcc`

3. **CMake + Ninja**
   ```bash
   # MSYS2
   pacman -S mingw-w64-x86_64-cmake mingw-w64-x86_64-ninja
   ```

### Build Steps

```bash
cd firmware/pico_test_fixture
mkdir build && cd build
cmake -G Ninja ..
ninja
```

Output: `pico_test_fixture.uf2`

### Flashing

1. Hold **BOOTSEL** button on Pico
2. Connect USB cable (while holding BOOTSEL)
3. Release BOOTSEL - Pico appears as USB drive "RPI-RP2"
4. Copy `pico_test_fixture.uf2` to the drive
5. Pico reboots automatically

## Architecture

```
┌─────────────────────────────────────────────────────────┐
│                    Pico Test Fixture                     │
├─────────────────────────────────────────────────────────┤
│  Core 0 (USB + Protocol)  │  Core 1 (Display + Buttons) │
│  ─────────────────────────│───────────────────────────  │
│  • TinyUSB CDC task       │  • ST7789 display @10Hz     │
│  • Packet RX/TX           │  • Button polling + debounce│
│  • CRC-16 validation      │  • Mode/state visualization │
│  • Command dispatch       │  • Log message rendering    │
│  • Emulator tick @1kHz    │  • RGB LED status           │
│  • Status broadcast @50Hz │                             │
└─────────────────────────────────────────────────────────┘
```

### File Structure

```
pico_test_fixture/
├── CMakeLists.txt          # Build configuration
├── README.md               # This file
├── tusb_config.h           # TinyUSB configuration
├── include/
│   └── test_fixture.h      # Shared definitions, state struct
├── src/
│   ├── main.c              # Entry point, dual-core setup
│   ├── usb_descriptors.c   # Custom USB identity
│   ├── protocol_handler.h/c # Packet parsing, responses
│   ├── emulator.h/c        # State machine, motion sim
│   ├── display.h/c         # LCD rendering
│   └── button_handler.h/c  # Input processing
└── lib/
    └── pimoroni/           # (fetched via CMake)
```

## Protocol Compatibility

The test fixture implements the same protocol as production firmware:

- **Packet format**: `0xAA | CMD | SEQ | LEN | PAYLOAD | CRC16 | 0x55`
- **CRC**: CRC-16-CCITT (polynomial 0x1021, init 0xFFFF)
- **Position encoding**: Nanometers (int32_t)
- **Status rate**: 50 Hz broadcast

Supported commands:
- System: PING, GET_VERSION, GET_STATUS, RESET
- Motion: MOVE_LINEAR, MOVE_RAPID, JOG_START/STOP, MOTION_SEGMENT
- Control: START, PAUSE, RESUME, STOP, ESTOP, RESET_ESTOP
- Homing: HOME_ALL, HOME_AXIS
- Queue: QUEUE_STATUS, CLEAR_QUEUE
- Spindle/Coolant: ON/OFF commands

## Use Cases

### 1. HMI Development
Develop and test the Windows HMI without CNC hardware:
- Connect fixture via USB
- Open as COM port in HMI
- Full protocol communication works
- Visual confirmation on fixture display

### 2. Protocol Testing
Validate packet encoding/decoding:
- Send commands, verify responses
- Test CRC error handling (display shows CRC error count)
- Verify state machine transitions

### 3. Fault Handling
Test error recovery:
- Inject E-STOP via button
- Verify HMI receives alarm status
- Test reset sequences

### 4. Demos
Show system to stakeholders:
- Physical device with visual feedback
- Button control for live interaction
- No expensive CNC hardware required

## LED Status Colors

| Color | Meaning |
|-------|---------|
| Blue (dim) | Idle, waiting |
| Green (dim) | Running, motion active |
| Yellow (dim) | Homing in progress |
| Red (dim) | Alarm or E-STOP active |
| Cyan (flash) | Packet received/sent |
| Green (flash) | Valid command processed |

## Troubleshooting

### Fixture not appearing as COM port
- Check USB cable (some are charge-only)
- Try different USB port
- Reflash firmware

### No response to commands
- Check baud rate (115200 default, but CDC ignores it)
- Verify packet CRC calculation
- Check display for "CRC Err" counter

### Display blank
- Verify Pimoroni Display 2.0 is properly seated
- Check backlight (should be on by default)
- Debug via UART (GPIO 0/1)

## Debug Output

UART debug is enabled on GPIO 0 (TX) and GPIO 1 (RX) at 115200 baud:

```
=== OpenCNC Test Fixture ===
Version: 1.0.0
Core 0: USB + Protocol
Core 1: Display + Buttons

Core 1 launched
Waiting for USB connection...
Uptime: 5000 ms, RX: 0, TX: 250, State: IDLE
```

## Development Notes (TODO)

Items to complete when picking up development:

### 1. Pimoroni Library Integration
The `display.c` contains **stub functions** for graphics operations. Replace with actual Pimoroni API calls:

```c
// Current stubs in display.c:
static void gfx_clear(uint8_t color) { /* stub */ }
static void gfx_rect(...) { /* stub */ }
static void gfx_text(...) { /* stub */ }
static void gfx_update(void) { /* stub */ }

// Replace with Pimoroni pico_graphics C++ wrapper or direct ST7789 calls
```

**Option A**: Create C++ wrapper file (`display_impl.cpp`) that uses Pimoroni's `PicoGraphics` class and exposes C functions.

**Option B**: Use Pimoroni's C-compatible `st7789` driver directly with custom text rendering.

### 2. Pimoroni FetchContent
The CMakeLists.txt uses FetchContent to download `pimoroni-pico`. If this fails:
- Clone manually: `git clone https://github.com/pimoroni/pimoroni-pico.git lib/pimoroni`
- Update CMakeLists.txt to use local path instead of FetchContent

### 3. Font Rendering
Current code assumes a text rendering function exists. Options:
- Use Pimoroni's built-in bitmap fonts
- Add a simple 8x8 font array for minimal dependency
- Use Pimoroni's `pico_graphics` text functions

### 4. Framebuffer Memory
320×240 @ RGB565 = 153KB (exceeds Pico RAM). Current design uses RGB332 (76KB). Verify Pimoroni library supports this mode or adjust:
```cpp
// In Pimoroni:
PicoGraphics_PenRGB332 graphics(DISPLAY_WIDTH, DISPLAY_HEIGHT, nullptr);
```

### 5. Testing Without Hardware
To test protocol logic without Pico hardware:
- Extract `protocol_handler.c` and `emulator.c` 
- Build as Windows test with mock USB layer
- Add to `tests/` directory

### 6. USB Descriptor Cleanup
The string descriptor arrays in `usb_descriptors.c` have fixed sizes. Consider dynamic generation or verify lengths match actual strings.

## License

MIT License - Part of OpenCNC project
