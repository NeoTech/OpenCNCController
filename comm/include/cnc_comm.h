/*
 * CNC Communication Layer
 * 
 * USB/Serial communication protocol for CNC motion controllers.
 * Supports ESP32-S3, STM32, and Raspberry Pi Pico targets.
 * 
 * Copyright (c) 2024 OpenCNC Project
 * MIT License
 */

#ifndef CNC_COMM_H
#define CNC_COMM_H

#include <cstdint>
#include <cstddef>
#include <string>
#include <vector>
#include <memory>
#include <functional>
#include <chrono>

namespace cnc_comm {

// ============================================================================
// Constants
// ============================================================================

constexpr size_t MAX_PACKET_SIZE = 256;
constexpr size_t MAX_PAYLOAD_SIZE = 248;
constexpr uint8_t PACKET_START_BYTE = 0xAA;
constexpr uint8_t PACKET_END_BYTE = 0x55;
constexpr uint16_t PROTOCOL_VERSION = 0x0100;  // 1.0

// ============================================================================
// Enumerations
// ============================================================================

enum class ConnectionType : uint8_t {
    NONE = 0,
    USB_CDC,        // USB Virtual COM Port
    USB_HID,        // USB HID (low latency)
    SERIAL,         // Hardware UART
    ETHERNET        // TCP/IP (future)
};

enum class ConnectionState : uint8_t {
    DISCONNECTED = 0,
    CONNECTING,
    CONNECTED,
    ERROR
};

enum class CommandType : uint8_t {
    // System commands (0x00-0x0F)
    PING = 0x00,
    PONG = 0x01,
    GET_VERSION = 0x02,
    VERSION_RESPONSE = 0x03,
    RESET = 0x04,
    GET_STATUS = 0x05,
    STATUS_RESPONSE = 0x06,
    
    // Motion commands (0x10-0x2F)
    MOVE_LINEAR = 0x10,
    MOVE_RAPID = 0x11,
    MOVE_ARC = 0x12,
    MOVE_DWELL = 0x13,
    JOG_START = 0x14,
    JOG_STOP = 0x15,
    
    // Control commands (0x30-0x3F)
    START = 0x30,
    PAUSE = 0x31,
    RESUME = 0x32,
    STOP = 0x33,
    ESTOP = 0x34,
    RESET_ESTOP = 0x35,
    
    // Homing and probing (0x40-0x4F)
    HOME_AXIS = 0x40,
    HOME_ALL = 0x41,
    PROBE = 0x42,
    SET_POSITION = 0x43,
    
    // Configuration (0x50-0x5F)
    GET_CONFIG = 0x50,
    SET_CONFIG = 0x51,
    SAVE_CONFIG = 0x52,
    LOAD_CONFIG = 0x53,
    
    // Spindle and coolant (0x60-0x6F)
    SPINDLE_ON = 0x60,
    SPINDLE_OFF = 0x61,
    SET_SPINDLE_SPEED = 0x62,
    COOLANT_ON = 0x63,
    COOLANT_OFF = 0x64,
    
    // I/O (0x70-0x7F)
    GET_INPUTS = 0x70,
    SET_OUTPUTS = 0x71,
    
    // Motion buffer (0x80-0x8F)
    QUEUE_STATUS = 0x80,
    CLEAR_QUEUE = 0x81,
    MOTION_SEGMENT = 0x82,
    
    // Real-time status (0xA0-0xAF)
    RT_POSITION = 0xA0,
    RT_VELOCITY = 0xA1,
    RT_STATUS = 0xA2,
    
    // Errors and acknowledgments (0xF0-0xFF)
    ACK = 0xF0,
    NAK = 0xF1,
    ERROR = 0xFE,
    INVALID = 0xFF
};

enum class ErrorCode : uint8_t {
    NONE = 0,
    INVALID_COMMAND,
    INVALID_PARAMETER,
    BUFFER_OVERFLOW,
    BUFFER_UNDERFLOW,
    CHECKSUM_ERROR,
    TIMEOUT,
    HARDWARE_ERROR,
    LIMIT_TRIGGERED,
    ESTOP_ACTIVE,
    NOT_HOMED,
    PROBE_ERROR
};

// ============================================================================
// Data Structures
// ============================================================================

#pragma pack(push, 1)

struct PacketHeader {
    uint8_t start_byte = PACKET_START_BYTE;
    uint8_t command;
    uint8_t sequence;
    uint8_t payload_length;
};

struct PacketFooter {
    uint16_t crc16;
    uint8_t end_byte = PACKET_END_BYTE;
};

// Position data (32 bytes)
struct PositionData {
    int32_t x;      // Position in steps or nm
    int32_t y;
    int32_t z;
    int32_t a;
    int32_t b;
    int32_t c;
    int32_t u;
    int32_t v;
};

// Real-time status (compact, 32 bytes)
struct RealtimeStatus {
    uint8_t state;
    uint8_t alarm_code;
    uint8_t limit_switches;     // Bit flags
    uint8_t home_switches;      // Bit flags
    uint32_t queue_depth;
    PositionData position;
};

// Motion segment packet
struct MotionSegmentPacket {
    uint8_t motion_type;        // Linear, arc, dwell
    uint8_t flags;              // Rapid, etc.
    uint16_t feed_rate;         // mm/min * 10
    int32_t end_x;              // Target position
    int32_t end_y;
    int32_t end_z;
    int32_t end_a;
    int32_t center_i;           // Arc center (if applicable)
    int32_t center_j;
    uint16_t dwell_ms;          // Dwell time
};

// Configuration packet
struct ConfigPacket {
    uint8_t config_id;
    uint8_t data_type;
    uint16_t data_length;
    uint8_t data[64];
};

#pragma pack(pop)

// High-level packet structure
struct Packet {
    CommandType command = CommandType::INVALID;
    uint8_t sequence = 0;
    std::vector<uint8_t> payload;
    bool valid = false;
    
    template<typename T>
    void setPayload(const T& data) {
        payload.resize(sizeof(T));
        std::memcpy(payload.data(), &data, sizeof(T));
    }
    
    template<typename T>
    bool getPayload(T& data) const {
        if (payload.size() < sizeof(T)) return false;
        std::memcpy(&data, payload.data(), sizeof(T));
        return true;
    }
};

// Device information
struct DeviceInfo {
    std::string port_name;
    std::string description;
    uint16_t vendor_id = 0;
    uint16_t product_id = 0;
    ConnectionType type = ConnectionType::NONE;
};

// Connection configuration
struct ConnectionConfig {
    std::string port;           // COM port or device path
    uint32_t baud_rate = 115200;
    int timeout_ms = 100;
    bool auto_reconnect = true;
    int reconnect_delay_ms = 1000;
};

// ============================================================================
// Callback Types
// ============================================================================

using PacketCallback = std::function<void(const Packet&)>;
using StatusCallback = std::function<void(const RealtimeStatus&)>;
using ErrorCallback = std::function<void(ErrorCode, const std::string&)>;
using ConnectionCallback = std::function<void(ConnectionState)>;

// ============================================================================
// Connection Interface
// ============================================================================

class Connection {
public:
    virtual ~Connection() = default;
    
    virtual bool open(const ConnectionConfig& config) = 0;
    virtual void close() = 0;
    virtual bool isOpen() const = 0;
    
    virtual int write(const uint8_t* data, size_t length) = 0;
    virtual int read(uint8_t* buffer, size_t max_length, int timeout_ms) = 0;
    
    virtual std::string getPortName() const = 0;
    virtual ConnectionType getType() const = 0;
};

// ============================================================================
// Communication Manager
// ============================================================================

class CommManager {
public:
    CommManager();
    ~CommManager();
    
    // Non-copyable
    CommManager(const CommManager&) = delete;
    CommManager& operator=(const CommManager&) = delete;
    
    // ========================================================================
    // Device Discovery
    // ========================================================================
    
    static std::vector<DeviceInfo> enumerateDevices();
    static std::vector<std::string> enumeratePorts();
    
    // ========================================================================
    // Connection Management
    // ========================================================================
    
    bool connect(const ConnectionConfig& config);
    bool connect(const std::string& port, uint32_t baud_rate = 115200);
    void disconnect();
    bool isConnected() const;
    ConnectionState getState() const;
    
    // ========================================================================
    // Packet Transmission
    // ========================================================================
    
    bool sendPacket(const Packet& packet);
    bool sendCommand(CommandType command);
    bool sendCommand(CommandType command, const void* payload, size_t length);
    
    template<typename T>
    bool sendCommand(CommandType command, const T& payload) {
        return sendCommand(command, &payload, sizeof(T));
    }
    
    // ========================================================================
    // Synchronous Operations
    // ========================================================================
    
    bool sendAndWait(const Packet& packet, Packet& response, int timeout_ms = 100);
    bool ping(int timeout_ms = 100);
    bool getVersion(uint16_t& version, int timeout_ms = 100);
    bool getStatus(RealtimeStatus& status, int timeout_ms = 100);
    
    // ========================================================================
    // Motion Commands
    // ========================================================================
    
    bool sendMotionSegment(const MotionSegmentPacket& segment);
    bool sendJogStart(int axis, int32_t velocity);
    bool sendJogStop(int axis);
    bool sendEstop();
    bool sendResetEstop();
    bool sendHomeAxis(int axis);
    bool sendHomeAll();
    
    // ========================================================================
    // Spindle/Coolant
    // ========================================================================
    
    bool sendSpindleOn(bool cw, uint16_t rpm);
    bool sendSpindleOff();
    bool sendCoolantOn(bool mist, bool flood);
    bool sendCoolantOff();
    
    // ========================================================================
    // Callbacks
    // ========================================================================
    
    void setPacketCallback(PacketCallback callback);
    void setStatusCallback(StatusCallback callback);
    void setErrorCallback(ErrorCallback callback);
    void setConnectionCallback(ConnectionCallback callback);
    
    // ========================================================================
    // Processing
    // ========================================================================
    
    void poll();  // Call periodically to process incoming data
    void startPollingThread(int poll_rate_hz = 100);
    void stopPollingThread();
    
private:
    struct Impl;
    std::unique_ptr<Impl> pImpl;
};

// ============================================================================
// Protocol Utilities
// ============================================================================

uint16_t calculateCRC16(const uint8_t* data, size_t length);
std::vector<uint8_t> encodePacket(const Packet& packet);
bool decodePacket(const uint8_t* data, size_t length, Packet& packet);
std::string commandTypeToString(CommandType cmd);
std::string errorCodeToString(ErrorCode error);

} // namespace cnc_comm

#endif // CNC_COMM_H
