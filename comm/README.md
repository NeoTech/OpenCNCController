# CNC Communication Layer

USB/Serial communication library for CNC motion controllers.

## Features

- **USB CDC (Virtual COM Port)**: Standard serial over USB
- **Packet Protocol**: Framed packets with CRC-16 validation
- **Asynchronous I/O**: Threaded polling with callbacks
- **Platform Support**: Windows (Linux/macOS planned)

## Protocol Overview

```
┌────────┬────────┬────────┬────────┬─────────────┬─────────┬────────┐
│ START  │  CMD   │  SEQ   │  LEN   │  PAYLOAD    │  CRC16  │  END   │
│  0xAA  │ 1 byte │ 1 byte │ 1 byte │ 0-248 bytes │ 2 bytes │  0x55  │
└────────┴────────┴────────┴────────┴─────────────┴─────────┴────────┘
```

- **START**: 0xAA packet delimiter
- **CMD**: Command type (see below)
- **SEQ**: Sequence number for request/response matching
- **LEN**: Payload length (0-248)
- **PAYLOAD**: Command-specific data
- **CRC16**: CRC-16-CCITT checksum
- **END**: 0x55 packet delimiter

## Command Types

| Category | Range | Description |
|----------|-------|-------------|
| System | 0x00-0x0F | Ping, version, status |
| Motion | 0x10-0x2F | Linear, arc, jog moves |
| Control | 0x30-0x3F | Start, pause, stop, E-Stop |
| Homing | 0x40-0x4F | Home, probe operations |
| Config | 0x50-0x5F | Get/set configuration |
| Spindle | 0x60-0x6F | Spindle, coolant control |
| I/O | 0x70-0x7F | Digital I/O operations |
| Buffer | 0x80-0x8F | Motion queue management |
| RT Status | 0xA0-0xAF | Real-time status updates |
| Response | 0xF0-0xFF | ACK, NAK, errors |

## Usage

```cpp
#include <cnc_comm.h>

using namespace cnc_comm;

// Create communication manager
CommManager comm;

// Set up callbacks
comm.setStatusCallback([](const RealtimeStatus& status) {
    printf("Position: X=%d Y=%d Z=%d\n",
           status.position.x, status.position.y, status.position.z);
});

comm.setErrorCallback([](ErrorCode code, const std::string& msg) {
    printf("Error: %s\n", msg.c_str());
});

// Connect to device
if (comm.connect("COM3", 115200)) {
    // Start polling thread
    comm.startPollingThread(100);  // 100 Hz
    
    // Check connection
    if (comm.ping()) {
        printf("Device connected!\n");
    }
    
    // Get status
    RealtimeStatus status;
    if (comm.getStatus(status)) {
        printf("Queue depth: %d\n", status.queue_depth);
    }
    
    // Send motion
    MotionSegmentPacket segment;
    segment.motion_type = 0;  // Linear
    segment.end_x = mmToWire(100.0);  // X = 100mm
    segment.end_y = mmToWire(50.0);   // Y = 50mm
    segment.feed_rate = 3000;         // 300 mm/min
    
    comm.sendMotionSegment(segment);
}
```

## Device Discovery

```cpp
// List available COM ports
auto ports = CommManager::enumeratePorts();
for (const auto& port : ports) {
    printf("Found: %s\n", port.c_str());
}

// Get detailed device info
auto devices = CommManager::enumerateDevices();
for (const auto& dev : devices) {
    printf("%s: %s (VID=%04X PID=%04X)\n",
           dev.port_name.c_str(),
           dev.description.c_str(),
           dev.vendor_id, dev.product_id);
}
```

## Position Encoding

Positions are encoded as 32-bit signed integers:

- **Metric**: Position in nanometers (1mm = 1,000,000)
- **Range**: ±2147.483647 meters

```cpp
// Convert mm to wire format
int32_t wire_pos = mmToWire(123.456);  // 123456000

// Convert wire format to mm
double mm_pos = wireToMm(123456000);   // 123.456
```

## Firmware Integration

The firmware side should:

1. Parse incoming packets using the same protocol
2. Send periodic status updates (10-50 Hz recommended)
3. ACK/NAK motion commands
4. Maintain motion queue with depth reporting

See `firmware/` directory for reference implementations.
