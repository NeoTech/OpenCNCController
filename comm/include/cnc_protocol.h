/*
 * CNC Communication Protocol Definitions
 * 
 * Detailed protocol specification for host-to-controller communication.
 * 
 * Copyright (c) 2024 OpenCNC Project
 * MIT License
 */

#ifndef CNC_PROTOCOL_H
#define CNC_PROTOCOL_H

#include "cnc_comm.h"

namespace cnc_comm {

/*
 * ===========================================================================
 * PACKET STRUCTURE
 * ===========================================================================
 * 
 * All packets follow this structure:
 * 
 * +--------+--------+--------+--------+--...--+--------+--------+--------+
 * | START  |  CMD   |  SEQ   |  LEN   | PAYLOAD...     |  CRC16 |  END   |
 * | 0xAA   | 1 byte | 1 byte | 1 byte | 0-248 bytes    | 2 bytes| 0x55   |
 * +--------+--------+--------+--------+--...--+--------+--------+--------+
 * 
 * START:   Start delimiter (0xAA)
 * CMD:     Command type (see CommandType enum)
 * SEQ:     Sequence number (0-255, wrapping)
 * LEN:     Payload length (0-248)
 * PAYLOAD: Command-specific data
 * CRC16:   CRC-16-CCITT of CMD + SEQ + LEN + PAYLOAD
 * END:     End delimiter (0x55)
 * 
 * Maximum packet size: 256 bytes
 * Maximum payload size: 248 bytes
 */

// ============================================================================
// Position Encoding
// ============================================================================

/*
 * Positions are encoded as 32-bit signed integers in nanometers or steps.
 * 
 * For metric systems (mm/min):
 *   Position value = actual_position_mm * 1000000 (nanometers)
 *   Range: ±2147.483647 meters
 * 
 * For step-based systems:
 *   Position value = actual_position_steps
 *   Range: ±2147483647 steps
 * 
 * The firmware is configured to use one encoding or the other.
 */

constexpr int32_t MM_TO_NM = 1000000;
constexpr int32_t INCH_TO_NM = 25400000;

inline int32_t mmToWire(double mm) {
    return static_cast<int32_t>(mm * MM_TO_NM);
}

inline double wireToMm(int32_t wire) {
    return static_cast<double>(wire) / MM_TO_NM;
}

// ============================================================================
// Motion Segment Flags
// ============================================================================

namespace MotionFlags {
    constexpr uint8_t RAPID = 0x01;         // G0 rapid move
    constexpr uint8_t ARC_CW = 0x02;        // G2 clockwise arc
    constexpr uint8_t ARC_CCW = 0x04;       // G3 counter-clockwise arc
    constexpr uint8_t PLANE_XY = 0x00;      // G17
    constexpr uint8_t PLANE_XZ = 0x10;      // G18
    constexpr uint8_t PLANE_YZ = 0x20;      // G19
    constexpr uint8_t EXACT_STOP = 0x40;    // G61 - stop at end
    constexpr uint8_t LAST_SEGMENT = 0x80;  // Last in program
}

// ============================================================================
// Status Flags
// ============================================================================

namespace StatusFlags {
    // Machine state (lower nibble)
    constexpr uint8_t STATE_IDLE = 0x00;
    constexpr uint8_t STATE_RUNNING = 0x01;
    constexpr uint8_t STATE_PAUSED = 0x02;
    constexpr uint8_t STATE_HOMING = 0x03;
    constexpr uint8_t STATE_PROBING = 0x04;
    constexpr uint8_t STATE_JOG = 0x05;
    constexpr uint8_t STATE_ALARM = 0x0E;
    constexpr uint8_t STATE_ESTOP = 0x0F;
    
    // Status flags (upper nibble)
    constexpr uint8_t FLAG_HOMED = 0x10;
    constexpr uint8_t FLAG_PROBE_TRIGGERED = 0x20;
    constexpr uint8_t FLAG_SPINDLE_ON = 0x40;
    constexpr uint8_t FLAG_COOLANT_ON = 0x80;
}

// Limit switch bit positions
namespace LimitBits {
    constexpr uint8_t X_MIN = 0x01;
    constexpr uint8_t X_MAX = 0x02;
    constexpr uint8_t Y_MIN = 0x04;
    constexpr uint8_t Y_MAX = 0x08;
    constexpr uint8_t Z_MIN = 0x10;
    constexpr uint8_t Z_MAX = 0x20;
    constexpr uint8_t PROBE = 0x40;
    constexpr uint8_t ESTOP = 0x80;
}

// ============================================================================
// Configuration IDs
// ============================================================================

namespace ConfigId {
    // Axis configuration (per axis, axis number in data)
    constexpr uint8_t STEPS_PER_MM = 0x01;
    constexpr uint8_t MAX_VELOCITY = 0x02;
    constexpr uint8_t MAX_ACCELERATION = 0x03;
    constexpr uint8_t MAX_JERK = 0x04;
    constexpr uint8_t SOFT_LIMIT_MIN = 0x05;
    constexpr uint8_t SOFT_LIMIT_MAX = 0x06;
    constexpr uint8_t HOME_DIRECTION = 0x07;
    constexpr uint8_t HOME_VELOCITY = 0x08;
    constexpr uint8_t STEP_INVERT = 0x09;
    constexpr uint8_t DIR_INVERT = 0x0A;
    
    // Global configuration
    constexpr uint8_t JUNCTION_DEVIATION = 0x20;
    constexpr uint8_t MIN_SEGMENT_TIME = 0x21;
    constexpr uint8_t ARC_TOLERANCE = 0x22;
    
    // Spindle configuration
    constexpr uint8_t SPINDLE_MAX_RPM = 0x30;
    constexpr uint8_t SPINDLE_MIN_RPM = 0x31;
    constexpr uint8_t SPINDLE_PWM_FREQ = 0x32;
    
    // I/O configuration
    constexpr uint8_t INPUT_INVERT = 0x40;
    constexpr uint8_t OUTPUT_INVERT = 0x41;
}

// ============================================================================
// Detailed Packet Payloads
// ============================================================================

#pragma pack(push, 1)

// GET_VERSION response (CMD 0x03)
struct VersionResponse {
    uint16_t protocol_version;  // Protocol version (0x0100 = 1.0)
    uint16_t firmware_version;  // Firmware version
    uint8_t board_type;         // 0=ESP32, 1=STM32, 2=Pico, 3=FPGA
    uint8_t num_axes;           // Number of supported axes
    uint8_t features;           // Feature flags
    char build_date[12];        // Build date string
};

// Jog command payload
struct JogCommand {
    uint8_t axis;               // Axis number (0=X, 1=Y, etc.)
    int32_t velocity;           // Velocity in steps/sec or nm/sec
    int32_t distance;           // 0 for continuous, else incremental
};

// Home command payload
struct HomeCommand {
    uint8_t axes_mask;          // Bit mask of axes to home
    uint8_t flags;              // Home direction, etc.
};

// Probe command payload
struct ProbeCommand {
    uint8_t axis;               // Axis to probe
    int32_t distance;           // Maximum probe distance
    uint16_t feed_rate;         // Probe feed rate
    uint8_t flags;              // Probe toward/away, error on no contact
};

// Spindle command payload
struct SpindleCommand {
    uint8_t direction;          // 0=off, 1=CW, 2=CCW
    uint16_t rpm;               // RPM * 10
};

// Set position command (for G92, etc.)
struct SetPositionCommand {
    uint8_t axes_mask;          // Which axes to set
    int32_t positions[6];       // New positions for X, Y, Z, A, B, C
};

// Queue status response
struct QueueStatusResponse {
    uint16_t depth;             // Current segments in queue
    uint16_t capacity;          // Maximum queue capacity
    uint16_t space;             // Available space
    uint8_t state;              // Queue state
    uint8_t reserved;
};

// Extended status (full status request)
struct ExtendedStatus {
    uint8_t machine_state;
    uint8_t alarm_code;
    uint8_t limit_switches;
    uint8_t home_switches;
    uint16_t queue_depth;
    uint16_t queue_capacity;
    int32_t pos_x;
    int32_t pos_y;
    int32_t pos_z;
    int32_t pos_a;
    int32_t pos_b;
    int32_t pos_c;
    int32_t vel_x;
    int32_t vel_y;
    int32_t vel_z;
    uint16_t spindle_rpm;
    uint16_t feed_override;     // Percentage * 10
    uint16_t spindle_override;
    uint16_t rapid_override;
    uint32_t runtime_seconds;
    uint32_t input_pins;
    uint32_t output_pins;
};

#pragma pack(pop)

// ============================================================================
// CRC-16-CCITT Implementation
// ============================================================================

/*
 * CRC-16-CCITT (polynomial 0x1021, initial value 0xFFFF)
 * 
 * This is the standard CRC used in many protocols including XMODEM.
 * Calculated over: CMD + SEQ + LEN + PAYLOAD
 */

inline uint16_t crc16_update(uint16_t crc, uint8_t data) {
    crc ^= static_cast<uint16_t>(data) << 8;
    for (int i = 0; i < 8; i++) {
        if (crc & 0x8000) {
            crc = (crc << 1) ^ 0x1021;
        } else {
            crc <<= 1;
        }
    }
    return crc;
}

inline uint16_t crc16_calculate(const uint8_t* data, size_t length) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < length; i++) {
        crc = crc16_update(crc, data[i]);
    }
    return crc;
}

} // namespace cnc_comm

#endif // CNC_PROTOCOL_H
