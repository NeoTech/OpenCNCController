# OpenCNC ESP32-P4 Central Controller

Central motion controller for the OpenCNC distributed CNC system. Uses CAN-FD to communicate with axis nodes (STM32G4 or RP2040) and Ethernet to communicate with the Windows HMI.

## Hardware Requirements

### ESP32-P4 Module
- **CPU**: Dual-core RISC-V @ 400 MHz
- **Memory**: 768 KB SRAM, PSRAM support
- **Peripherals Used**:
  - 3× TWAI (CAN-FD) controllers
  - EMAC (Ethernet)
  - MIPI-DSI (Display)
  - GPIO (Buttons, encoder)

### External Components
- **TJA1051T/3** CAN transceivers (3×, one per bus)
- **LAN8720A** or similar Ethernet PHY
- **MIPI-DSI display** (optional, 800×480 recommended)
- **Rotary encoder** with button (for jog mode)
- **Buttons** (E-Stop, Axis Select, Jog Speed, Home)

## Pin Configuration

Default pin assignments (configurable via menuconfig):

| Function | GPIO | Notes |
|----------|------|-------|
| CAN0 TX | 4 | Primary axes (X, Y, Z) |
| CAN0 RX | 5 | |
| CAN1 TX | 6 | Secondary axes (A, B, C) |
| CAN1 RX | 7 | |
| CAN2 TX | 8 | Auxiliary (spindle, I/O) |
| CAN2 RX | 9 | |
| ETH RMII | 10-19 | Ethernet PHY interface |
| MIPI DSI | 40-47 | Display interface |
| Jog Enc A | 20 | Handwheel encoder |
| Jog Enc B | 21 | |
| Buttons | 22-25 | UI buttons |

## Building

### Prerequisites
1. Install ESP-IDF v5.2 or later
2. Set `IDF_PATH` environment variable
3. Ensure ESP32-P4 target support is available

### Build Commands
```bash
# Set target
idf.py set-target esp32p4

# Configure (optional)
idf.py menuconfig

# Build
idf.py build

# Flash
idf.py -p COM3 flash

# Monitor
idf.py -p COM3 monitor
```

## Configuration

Use `idf.py menuconfig` to configure:

### OpenCNC P4 Controller
- **Device name**: Network-discoverable name
- **Ethernet Configuration**: DHCP or static IP, HMI port
- **CAN Bus Configuration**: Bitrates, pin assignments
- **Motion Configuration**: SYNC rate, max axes, buffer size
- **Display Configuration**: Resolution, refresh rate
- **Standalone Mode**: Jog encoder/button pins

## Architecture

### Task Structure

| Task | Core | Priority | Function |
|------|------|----------|----------|
| motion | 1 | Highest | SYNC generation, trajectory |
| can_rx | 1 | High | CAN message processing |
| ethernet | 0 | Normal | HMI communication |
| display | 0 | Normal | Local display updates |
| ui | 0 | Normal | Button/encoder handling |

### CAN Bus Allocation

- **Bus 0**: Primary motion axes (X, Y, Z) - Node IDs 1-3
- **Bus 1**: Secondary motion axes (A, B, C) - Node IDs 4-6
- **Bus 2**: Auxiliary devices (spindle, I/O, probes) - Node IDs 7-9

### CANopen Protocol

- **NMT**: Network management, node state control
- **SYNC**: 1 kHz synchronization for motion
- **PDO**: Real-time position commands/feedback
- **SDO**: Configuration and diagnostics
- **EMCY**: Emergency messages
- **Auto-Assign**: Custom protocol for node addressing

## Boot Sequence

1. **Initialize hardware** - CAN buses, Ethernet, display
2. **Discovery phase** - Wait for axis nodes to announce themselves
3. **Auto-assign** - Assign node IDs based on serial numbers
4. **Configuration** - Read node capabilities, configure PDOs
5. **Operational** - Start motion control loop

## Communication Protocol

### Ethernet to Windows HMI
- TCP connection on port 5000 (configurable)
- Binary protocol with CRC-16
- Commands for motion, status, configuration

### CAN to Axis Nodes
- CANopen DS301/CiA 402 compliant
- CAN-FD: 500 kbps arbitration, 2 Mbps data
- 1 kHz SYNC rate for motion control

## Standalone Mode

When Windows HMI is not connected:
- **Jog mode**: Manual axis movement via handwheel
- **Homing**: Axis homing sequences
- **Status display**: Position and state on local display

## Error Handling

- **Node timeout**: Heartbeat monitoring, automatic stop on lost node
- **Communication errors**: CAN bus error counters, retransmission
- **Motion errors**: Following error detection, soft limit enforcement
- **E-Stop**: Hardware interrupt, immediate stop of all axes

## Development Notes

### TODO
- [ ] Implement multi-TWAI controller initialization
- [ ] Add CAN-FD data phase timing configuration
- [ ] Implement full trajectory interpolation
- [ ] Add probe input handling
- [ ] Implement spindle speed control
- [ ] Add tool change sequence support
- [ ] Implement persistent configuration storage
- [ ] Add OTA firmware update support

### Testing
- Use Pico Test Fixture for HMI development
- Use CAN analyzer (PCAN, Kvaser) for bus debugging
- Logic analyzer for timing verification
