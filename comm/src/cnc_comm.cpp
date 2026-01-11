/*
 * CNC Communication - Windows USB/Serial Implementation
 * 
 * Copyright (c) 2024 OpenCNC Project
 * MIT License
 */

#include "cnc_comm.h"
#include "cnc_protocol.h"
#include <thread>
#include <mutex>
#include <atomic>
#include <queue>
#include <condition_variable>

#ifdef _WIN32
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <windows.h>
#include <setupapi.h>
#include <devguid.h>
#endif

namespace cnc_comm {

// ============================================================================
// Serial Connection (Windows)
// ============================================================================

#ifdef _WIN32

class SerialConnection : public Connection {
public:
    SerialConnection() : handle_(INVALID_HANDLE_VALUE) {}
    
    ~SerialConnection() override {
        close();
    }
    
    bool open(const ConnectionConfig& config) override {
        if (handle_ != INVALID_HANDLE_VALUE) {
            close();
        }
        
        // Prepend \\.\ for COM ports > 9
        std::string port_path = "\\\\.\\" + config.port;
        
        handle_ = CreateFileA(
            port_path.c_str(),
            GENERIC_READ | GENERIC_WRITE,
            0,
            nullptr,
            OPEN_EXISTING,
            FILE_ATTRIBUTE_NORMAL,
            nullptr
        );
        
        if (handle_ == INVALID_HANDLE_VALUE) {
            return false;
        }
        
        // Configure serial port
        DCB dcb = {0};
        dcb.DCBlength = sizeof(dcb);
        
        if (!GetCommState(handle_, &dcb)) {
            close();
            return false;
        }
        
        dcb.BaudRate = config.baud_rate;
        dcb.ByteSize = 8;
        dcb.StopBits = ONESTOPBIT;
        dcb.Parity = NOPARITY;
        dcb.fDtrControl = DTR_CONTROL_ENABLE;
        dcb.fRtsControl = RTS_CONTROL_ENABLE;
        dcb.fOutxCtsFlow = FALSE;
        dcb.fOutxDsrFlow = FALSE;
        dcb.fDsrSensitivity = FALSE;
        dcb.fOutX = FALSE;
        dcb.fInX = FALSE;
        
        if (!SetCommState(handle_, &dcb)) {
            close();
            return false;
        }
        
        // Set timeouts
        COMMTIMEOUTS timeouts = {0};
        timeouts.ReadIntervalTimeout = MAXDWORD;
        timeouts.ReadTotalTimeoutMultiplier = 0;
        timeouts.ReadTotalTimeoutConstant = config.timeout_ms;
        timeouts.WriteTotalTimeoutMultiplier = 0;
        timeouts.WriteTotalTimeoutConstant = config.timeout_ms;
        
        if (!SetCommTimeouts(handle_, &timeouts)) {
            close();
            return false;
        }
        
        // Clear buffers
        PurgeComm(handle_, PURGE_RXCLEAR | PURGE_TXCLEAR);
        
        port_name_ = config.port;
        return true;
    }
    
    void close() override {
        if (handle_ != INVALID_HANDLE_VALUE) {
            CloseHandle(handle_);
            handle_ = INVALID_HANDLE_VALUE;
        }
    }
    
    bool isOpen() const override {
        return handle_ != INVALID_HANDLE_VALUE;
    }
    
    int write(const uint8_t* data, size_t length) override {
        if (handle_ == INVALID_HANDLE_VALUE) return -1;
        
        DWORD bytes_written = 0;
        if (!WriteFile(handle_, data, static_cast<DWORD>(length), 
                       &bytes_written, nullptr)) {
            return -1;
        }
        return static_cast<int>(bytes_written);
    }
    
    int read(uint8_t* buffer, size_t max_length, int timeout_ms) override {
        if (handle_ == INVALID_HANDLE_VALUE) return -1;
        
        // Update timeout if different
        COMMTIMEOUTS timeouts = {0};
        timeouts.ReadIntervalTimeout = MAXDWORD;
        timeouts.ReadTotalTimeoutMultiplier = 0;
        timeouts.ReadTotalTimeoutConstant = timeout_ms;
        SetCommTimeouts(handle_, &timeouts);
        
        DWORD bytes_read = 0;
        if (!ReadFile(handle_, buffer, static_cast<DWORD>(max_length), 
                      &bytes_read, nullptr)) {
            return -1;
        }
        return static_cast<int>(bytes_read);
    }
    
    std::string getPortName() const override {
        return port_name_;
    }
    
    ConnectionType getType() const override {
        return ConnectionType::SERIAL;
    }
    
private:
    HANDLE handle_;
    std::string port_name_;
};

#endif // _WIN32

// ============================================================================
// Communication Manager Implementation
// ============================================================================

struct CommManager::Impl {
    std::unique_ptr<Connection> connection;
    ConnectionState state = ConnectionState::DISCONNECTED;
    ConnectionConfig config;
    
    std::atomic<bool> poll_running{false};
    std::unique_ptr<std::thread> poll_thread;
    
    uint8_t sequence_number = 0;
    
    // Receive buffer
    std::vector<uint8_t> rx_buffer;
    static constexpr size_t RX_BUFFER_SIZE = 4096;
    
    // Response queue
    std::mutex response_mutex;
    std::condition_variable response_cv;
    std::queue<Packet> response_queue;
    
    // Callbacks
    PacketCallback packet_callback;
    StatusCallback status_callback;
    ErrorCallback error_callback;
    ConnectionCallback connection_callback;
    std::mutex callback_mutex;
    
    Impl() {
        rx_buffer.reserve(RX_BUFFER_SIZE);
    }
    
    void notifyConnection(ConnectionState new_state) {
        state = new_state;
        std::lock_guard<std::mutex> lock(callback_mutex);
        if (connection_callback) {
            connection_callback(new_state);
        }
    }
    
    void notifyPacket(const Packet& packet) {
        std::lock_guard<std::mutex> lock(callback_mutex);
        if (packet_callback) {
            packet_callback(packet);
        }
    }
    
    void notifyError(ErrorCode code, const std::string& message) {
        std::lock_guard<std::mutex> lock(callback_mutex);
        if (error_callback) {
            error_callback(code, message);
        }
    }
    
    void processReceivedData() {
        // Look for packets in receive buffer
        while (rx_buffer.size() >= 7) {  // Minimum packet size
            // Find start byte
            size_t start = 0;
            while (start < rx_buffer.size() && rx_buffer[start] != PACKET_START_BYTE) {
                start++;
            }
            
            if (start > 0) {
                rx_buffer.erase(rx_buffer.begin(), rx_buffer.begin() + start);
            }
            
            if (rx_buffer.size() < 7) break;
            
            // Check if we have complete packet
            uint8_t payload_len = rx_buffer[3];
            size_t packet_size = 4 + payload_len + 3;  // Header + payload + footer
            
            if (rx_buffer.size() < packet_size) break;
            
            // Validate end byte
            if (rx_buffer[packet_size - 1] != PACKET_END_BYTE) {
                rx_buffer.erase(rx_buffer.begin());
                continue;
            }
            
            // Decode packet
            Packet packet;
            if (decodePacket(rx_buffer.data(), packet_size, packet)) {
                // Add to response queue
                {
                    std::lock_guard<std::mutex> lock(response_mutex);
                    response_queue.push(packet);
                }
                response_cv.notify_one();
                
                // Handle status updates
                if (packet.command == CommandType::RT_STATUS ||
                    packet.command == CommandType::STATUS_RESPONSE) {
                    RealtimeStatus status;
                    if (packet.getPayload(status)) {
                        std::lock_guard<std::mutex> lock(callback_mutex);
                        if (status_callback) {
                            status_callback(status);
                        }
                    }
                }
                
                notifyPacket(packet);
            }
            
            rx_buffer.erase(rx_buffer.begin(), rx_buffer.begin() + packet_size);
        }
    }
};

CommManager::CommManager() : pImpl(std::make_unique<Impl>()) {}

CommManager::~CommManager() {
    stopPollingThread();
    disconnect();
}

std::vector<DeviceInfo> CommManager::enumerateDevices() {
    std::vector<DeviceInfo> devices;
    
#ifdef _WIN32
    // Enumerate COM ports
    auto ports = enumeratePorts();
    for (const auto& port : ports) {
        DeviceInfo info;
        info.port_name = port;
        info.description = "Serial Port";
        info.type = ConnectionType::SERIAL;
        devices.push_back(info);
    }
#endif
    
    return devices;
}

std::vector<std::string> CommManager::enumeratePorts() {
    std::vector<std::string> ports;
    
#ifdef _WIN32
    for (int i = 1; i <= 256; i++) {
        std::string port = "COM" + std::to_string(i);
        std::string path = "\\\\.\\" + port;
        
        HANDLE h = CreateFileA(
            path.c_str(),
            GENERIC_READ | GENERIC_WRITE,
            0, nullptr, OPEN_EXISTING, 0, nullptr
        );
        
        if (h != INVALID_HANDLE_VALUE) {
            CloseHandle(h);
            ports.push_back(port);
        }
    }
#endif
    
    return ports;
}

bool CommManager::connect(const ConnectionConfig& config) {
    disconnect();
    
    pImpl->config = config;
    pImpl->notifyConnection(ConnectionState::CONNECTING);
    
#ifdef _WIN32
    pImpl->connection = std::make_unique<SerialConnection>();
#else
    pImpl->notifyError(ErrorCode::HARDWARE_ERROR, "Platform not supported");
    return false;
#endif
    
    if (!pImpl->connection->open(config)) {
        pImpl->notifyConnection(ConnectionState::ERROR);
        pImpl->notifyError(ErrorCode::HARDWARE_ERROR, 
                           "Failed to open " + config.port);
        return false;
    }
    
    pImpl->notifyConnection(ConnectionState::CONNECTED);
    return true;
}

bool CommManager::connect(const std::string& port, uint32_t baud_rate) {
    ConnectionConfig config;
    config.port = port;
    config.baud_rate = baud_rate;
    return connect(config);
}

void CommManager::disconnect() {
    stopPollingThread();
    
    if (pImpl->connection) {
        pImpl->connection->close();
        pImpl->connection.reset();
    }
    
    pImpl->notifyConnection(ConnectionState::DISCONNECTED);
}

bool CommManager::isConnected() const {
    return pImpl->connection && pImpl->connection->isOpen();
}

ConnectionState CommManager::getState() const {
    return pImpl->state;
}

bool CommManager::sendPacket(const Packet& packet) {
    if (!isConnected()) return false;
    
    auto encoded = encodePacket(packet);
    int written = pImpl->connection->write(encoded.data(), encoded.size());
    
    return written == static_cast<int>(encoded.size());
}

bool CommManager::sendCommand(CommandType command) {
    Packet packet;
    packet.command = command;
    packet.sequence = pImpl->sequence_number++;
    return sendPacket(packet);
}

bool CommManager::sendCommand(CommandType command, const void* payload, size_t length) {
    Packet packet;
    packet.command = command;
    packet.sequence = pImpl->sequence_number++;
    packet.payload.resize(length);
    std::memcpy(packet.payload.data(), payload, length);
    return sendPacket(packet);
}

bool CommManager::sendAndWait(const Packet& packet, Packet& response, int timeout_ms) {
    if (!sendPacket(packet)) return false;
    
    std::unique_lock<std::mutex> lock(pImpl->response_mutex);
    
    if (pImpl->response_cv.wait_for(lock, std::chrono::milliseconds(timeout_ms),
                                     [this] { return !pImpl->response_queue.empty(); })) {
        response = pImpl->response_queue.front();
        pImpl->response_queue.pop();
        return true;
    }
    
    return false;
}

bool CommManager::ping(int timeout_ms) {
    Packet request, response;
    request.command = CommandType::PING;
    request.sequence = pImpl->sequence_number++;
    
    if (sendAndWait(request, response, timeout_ms)) {
        return response.command == CommandType::PONG;
    }
    return false;
}

bool CommManager::getVersion(uint16_t& version, int timeout_ms) {
    Packet request, response;
    request.command = CommandType::GET_VERSION;
    request.sequence = pImpl->sequence_number++;
    
    if (sendAndWait(request, response, timeout_ms)) {
        if (response.command == CommandType::VERSION_RESPONSE) {
            VersionResponse ver;
            if (response.getPayload(ver)) {
                version = ver.firmware_version;
                return true;
            }
        }
    }
    return false;
}

bool CommManager::getStatus(RealtimeStatus& status, int timeout_ms) {
    Packet request, response;
    request.command = CommandType::GET_STATUS;
    request.sequence = pImpl->sequence_number++;
    
    if (sendAndWait(request, response, timeout_ms)) {
        if (response.command == CommandType::STATUS_RESPONSE) {
            return response.getPayload(status);
        }
    }
    return false;
}

bool CommManager::sendMotionSegment(const MotionSegmentPacket& segment) {
    return sendCommand(CommandType::MOTION_SEGMENT, segment);
}

bool CommManager::sendJogStart(int axis, int32_t velocity) {
    JogCommand cmd;
    cmd.axis = static_cast<uint8_t>(axis);
    cmd.velocity = velocity;
    cmd.distance = 0;
    return sendCommand(CommandType::JOG_START, cmd);
}

bool CommManager::sendJogStop(int axis) {
    uint8_t axis_byte = static_cast<uint8_t>(axis);
    return sendCommand(CommandType::JOG_STOP, &axis_byte, 1);
}

bool CommManager::sendEstop() {
    return sendCommand(CommandType::ESTOP);
}

bool CommManager::sendResetEstop() {
    return sendCommand(CommandType::RESET_ESTOP);
}

bool CommManager::sendHomeAxis(int axis) {
    HomeCommand cmd;
    cmd.axes_mask = static_cast<uint8_t>(1 << axis);
    cmd.flags = 0;
    return sendCommand(CommandType::HOME_AXIS, cmd);
}

bool CommManager::sendHomeAll() {
    return sendCommand(CommandType::HOME_ALL);
}

bool CommManager::sendSpindleOn(bool cw, uint16_t rpm) {
    SpindleCommand cmd;
    cmd.direction = cw ? 1 : 2;
    cmd.rpm = rpm;
    return sendCommand(CommandType::SPINDLE_ON, cmd);
}

bool CommManager::sendSpindleOff() {
    return sendCommand(CommandType::SPINDLE_OFF);
}

bool CommManager::sendCoolantOn(bool mist, bool flood) {
    uint8_t flags = (mist ? 0x01 : 0) | (flood ? 0x02 : 0);
    return sendCommand(CommandType::COOLANT_ON, &flags, 1);
}

bool CommManager::sendCoolantOff() {
    return sendCommand(CommandType::COOLANT_OFF);
}

void CommManager::setPacketCallback(PacketCallback callback) {
    std::lock_guard<std::mutex> lock(pImpl->callback_mutex);
    pImpl->packet_callback = std::move(callback);
}

void CommManager::setStatusCallback(StatusCallback callback) {
    std::lock_guard<std::mutex> lock(pImpl->callback_mutex);
    pImpl->status_callback = std::move(callback);
}

void CommManager::setErrorCallback(ErrorCallback callback) {
    std::lock_guard<std::mutex> lock(pImpl->callback_mutex);
    pImpl->error_callback = std::move(callback);
}

void CommManager::setConnectionCallback(ConnectionCallback callback) {
    std::lock_guard<std::mutex> lock(pImpl->callback_mutex);
    pImpl->connection_callback = std::move(callback);
}

void CommManager::poll() {
    if (!isConnected()) return;
    
    uint8_t buffer[256];
    int bytes_read = pImpl->connection->read(buffer, sizeof(buffer), 0);
    
    if (bytes_read > 0) {
        pImpl->rx_buffer.insert(pImpl->rx_buffer.end(), 
                                 buffer, buffer + bytes_read);
        pImpl->processReceivedData();
    }
}

void CommManager::startPollingThread(int poll_rate_hz) {
    if (pImpl->poll_running) return;
    
    pImpl->poll_running = true;
    pImpl->poll_thread = std::make_unique<std::thread>([this, poll_rate_hz]() {
        int sleep_ms = 1000 / poll_rate_hz;
        while (pImpl->poll_running) {
            poll();
            std::this_thread::sleep_for(std::chrono::milliseconds(sleep_ms));
        }
    });
}

void CommManager::stopPollingThread() {
    pImpl->poll_running = false;
    if (pImpl->poll_thread && pImpl->poll_thread->joinable()) {
        pImpl->poll_thread->join();
    }
    pImpl->poll_thread.reset();
}

// ============================================================================
// Protocol Utilities
// ============================================================================

uint16_t calculateCRC16(const uint8_t* data, size_t length) {
    return crc16_calculate(data, length);
}

std::vector<uint8_t> encodePacket(const Packet& packet) {
    std::vector<uint8_t> encoded;
    encoded.reserve(4 + packet.payload.size() + 3);
    
    // Header
    encoded.push_back(PACKET_START_BYTE);
    encoded.push_back(static_cast<uint8_t>(packet.command));
    encoded.push_back(packet.sequence);
    encoded.push_back(static_cast<uint8_t>(packet.payload.size()));
    
    // Payload
    encoded.insert(encoded.end(), packet.payload.begin(), packet.payload.end());
    
    // CRC (over cmd + seq + len + payload)
    uint16_t crc = calculateCRC16(encoded.data() + 1, encoded.size() - 1);
    encoded.push_back(static_cast<uint8_t>(crc >> 8));
    encoded.push_back(static_cast<uint8_t>(crc & 0xFF));
    
    // End byte
    encoded.push_back(PACKET_END_BYTE);
    
    return encoded;
}

bool decodePacket(const uint8_t* data, size_t length, Packet& packet) {
    if (length < 7) return false;
    if (data[0] != PACKET_START_BYTE) return false;
    
    uint8_t payload_len = data[3];
    size_t expected_len = 4 + payload_len + 3;
    
    if (length < expected_len) return false;
    if (data[expected_len - 1] != PACKET_END_BYTE) return false;
    
    // Verify CRC
    uint16_t received_crc = (static_cast<uint16_t>(data[expected_len - 3]) << 8) |
                            data[expected_len - 2];
    uint16_t calculated_crc = calculateCRC16(data + 1, 3 + payload_len);
    
    if (received_crc != calculated_crc) return false;
    
    packet.command = static_cast<CommandType>(data[1]);
    packet.sequence = data[2];
    packet.payload.assign(data + 4, data + 4 + payload_len);
    packet.valid = true;
    
    return true;
}

std::string commandTypeToString(CommandType cmd) {
    switch (cmd) {
        case CommandType::PING: return "PING";
        case CommandType::PONG: return "PONG";
        case CommandType::GET_VERSION: return "GET_VERSION";
        case CommandType::GET_STATUS: return "GET_STATUS";
        case CommandType::MOVE_LINEAR: return "MOVE_LINEAR";
        case CommandType::MOVE_RAPID: return "MOVE_RAPID";
        case CommandType::JOG_START: return "JOG_START";
        case CommandType::JOG_STOP: return "JOG_STOP";
        case CommandType::ESTOP: return "ESTOP";
        case CommandType::HOME_ALL: return "HOME_ALL";
        case CommandType::ACK: return "ACK";
        case CommandType::NAK: return "NAK";
        case CommandType::ERROR: return "ERROR";
        default: return "UNKNOWN";
    }
}

std::string errorCodeToString(ErrorCode error) {
    switch (error) {
        case ErrorCode::NONE: return "No error";
        case ErrorCode::INVALID_COMMAND: return "Invalid command";
        case ErrorCode::INVALID_PARAMETER: return "Invalid parameter";
        case ErrorCode::BUFFER_OVERFLOW: return "Buffer overflow";
        case ErrorCode::CHECKSUM_ERROR: return "Checksum error";
        case ErrorCode::TIMEOUT: return "Timeout";
        case ErrorCode::HARDWARE_ERROR: return "Hardware error";
        case ErrorCode::LIMIT_TRIGGERED: return "Limit triggered";
        case ErrorCode::ESTOP_ACTIVE: return "E-Stop active";
        case ErrorCode::NOT_HOMED: return "Not homed";
        default: return "Unknown error";
    }
}

} // namespace cnc_comm
