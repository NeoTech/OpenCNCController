/*
 * OpenCNC HMI - Header-Only CNC Controller Interface
 * 
 * A single-header library for integrating CNC control into Windows applications.
 * Compatible with Win32, Qt5, Ultralight, and other UI frameworks.
 * 
 * Usage:
 *   #define OPENCNC_HMI_IMPLEMENTATION
 *   #include "opencnc_hmi.h"
 * 
 * Define OPENCNC_HMI_IMPLEMENTATION in exactly ONE source file before including.
 * 
 * Copyright (c) 2024 OpenCNC Project
 * MIT License
 */

#ifndef OPENCNC_HMI_H
#define OPENCNC_HMI_H

#include <cstdint>
#include <cstddef>
#include <string>
#include <vector>
#include <functional>
#include <memory>
#include <mutex>
#include <atomic>
#include <thread>
#include <chrono>

// Platform detection
#if defined(_WIN32) || defined(_WIN64)
    #define OPENCNC_PLATFORM_WINDOWS
    #ifndef WIN32_LEAN_AND_MEAN
        #define WIN32_LEAN_AND_MEAN
    #endif
    #include <windows.h>
#elif defined(__linux__)
    #define OPENCNC_PLATFORM_LINUX
#elif defined(__APPLE__)
    #define OPENCNC_PLATFORM_MACOS
#endif

// Export macros
#ifdef OPENCNC_SHARED
    #ifdef OPENCNC_BUILDING
        #define OPENCNC_API __declspec(dllexport)
    #else
        #define OPENCNC_API __declspec(dllimport)
    #endif
#else
    #define OPENCNC_API
#endif

namespace opencnc {

// ============================================================================
// Constants
// ============================================================================

constexpr int MAX_AXES = 9;          // X, Y, Z, A, B, C, U, V, W
constexpr int MAX_SPINDLES = 2;
constexpr int MAX_TOOLS = 256;
constexpr int MOTION_QUEUE_SIZE = 128;

// ============================================================================
// Enumerations
// ============================================================================

enum class MachineState : uint8_t {
    DISCONNECTED = 0,
    IDLE,
    RUNNING,
    PAUSED,
    ALARM,
    HOMING,
    JOG,
    PROBING,
    TOOL_CHANGE,
    ESTOP
};

enum class CoordSystem : uint8_t {
    MACHINE = 0,    // G53
    G54, G55, G56, G57, G58, G59,
    G59_1, G59_2, G59_3
};

enum class DistanceMode : uint8_t {
    ABSOLUTE = 0,   // G90
    INCREMENTAL     // G91
};

enum class FeedMode : uint8_t {
    UNITS_PER_MIN = 0,  // G94
    INVERSE_TIME        // G93
};

enum class PlaneSelect : uint8_t {
    XY = 0,  // G17
    XZ,      // G18
    YZ       // G19
};

enum class Units : uint8_t {
    MM = 0,  // G21
    INCH     // G20
};

enum class SpindleState : uint8_t {
    OFF = 0,
    CW,      // M3
    CCW      // M4
};

enum class CoolantState : uint8_t {
    OFF = 0,
    MIST,    // M7
    FLOOD,   // M8
    BOTH
};

enum class JogMode : uint8_t {
    CONTINUOUS = 0,
    INCREMENTAL
};

enum class AlarmCode : uint16_t {
    NONE = 0,
    HARD_LIMIT_X = 1,
    HARD_LIMIT_Y = 2,
    HARD_LIMIT_Z = 3,
    SOFT_LIMIT = 10,
    ESTOP_TRIGGERED = 20,
    SPINDLE_FAULT = 30,
    PROBE_FAIL = 40,
    HOMING_FAIL = 50,
    FOLLOWING_ERROR = 60,
    COMM_TIMEOUT = 100,
    COMM_ERROR = 101,
    FIRMWARE_ERROR = 200
};

// ============================================================================
// Data Structures
// ============================================================================

struct Position {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
    double a = 0.0;
    double b = 0.0;
    double c = 0.0;
    double u = 0.0;
    double v = 0.0;
    double w = 0.0;
    
    double& operator[](size_t idx) {
        switch(idx) {
            case 0: return x; case 1: return y; case 2: return z;
            case 3: return a; case 4: return b; case 5: return c;
            case 6: return u; case 7: return v; case 8: return w;
            default: return x;
        }
    }
    
    const double& operator[](size_t idx) const {
        switch(idx) {
            case 0: return x; case 1: return y; case 2: return z;
            case 3: return a; case 4: return b; case 5: return c;
            case 6: return u; case 7: return v; case 8: return w;
            default: return x;
        }
    }
    
    Position operator+(const Position& other) const {
        return {x + other.x, y + other.y, z + other.z,
                a + other.a, b + other.b, c + other.c,
                u + other.u, v + other.v, w + other.w};
    }
    
    Position operator-(const Position& other) const {
        return {x - other.x, y - other.y, z - other.z,
                a - other.a, b - other.b, c - other.c,
                u - other.u, v - other.v, w - other.w};
    }
};

struct Velocity {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
    double a = 0.0;
    double b = 0.0;
    double c = 0.0;
};

struct AxisLimits {
    double min_position = 0.0;
    double max_position = 0.0;
    double max_velocity = 0.0;
    double max_acceleration = 0.0;
    double max_jerk = 0.0;
    double steps_per_unit = 0.0;
    bool homed = false;
    bool enabled = true;
};

struct SpindleStatus {
    SpindleState state = SpindleState::OFF;
    double commanded_rpm = 0.0;
    double actual_rpm = 0.0;
    double load_percent = 0.0;
    bool at_speed = false;
};

struct ToolInfo {
    int number = 0;
    double length_offset = 0.0;
    double diameter = 0.0;
    double x_offset = 0.0;
    double z_offset = 0.0;
    std::string description;
};

struct ProgramStatus {
    std::string filename;
    int current_line = 0;
    int total_lines = 0;
    std::string current_block;
    double progress_percent = 0.0;
    std::chrono::seconds elapsed_time{0};
    std::chrono::seconds estimated_remaining{0};
};

struct MachineStatus {
    MachineState state = MachineState::DISCONNECTED;
    AlarmCode alarm = AlarmCode::NONE;
    std::string alarm_message;
    
    // Positions
    Position machine_position;      // Absolute machine coordinates
    Position work_position;         // Current work coordinate system
    Position target_position;       // Where we're going
    Position dtg;                   // Distance to go
    
    // Velocities
    Velocity current_velocity;
    double current_feed_rate = 0.0;
    
    // Modal state
    CoordSystem coord_system = CoordSystem::G54;
    DistanceMode distance_mode = DistanceMode::ABSOLUTE;
    FeedMode feed_mode = FeedMode::UNITS_PER_MIN;
    PlaneSelect plane = PlaneSelect::XY;
    Units units = Units::MM;
    
    // Spindle
    SpindleStatus spindle;
    
    // Coolant
    CoolantState coolant = CoolantState::OFF;
    
    // Tool
    int current_tool = 0;
    int next_tool = 0;
    
    // Overrides
    double feed_override = 100.0;   // Percentage
    double rapid_override = 100.0;
    double spindle_override = 100.0;
    
    // Digital I/O
    uint32_t input_pins = 0;
    uint32_t output_pins = 0;
    
    // Limits and home switches
    bool limit_x = false;
    bool limit_y = false;
    bool limit_z = false;
    bool home_x = false;
    bool home_y = false;
    bool home_z = false;
    bool probe_triggered = false;
    
    // Motion queue
    int queue_depth = 0;
    int queue_capacity = MOTION_QUEUE_SIZE;
    
    // Program
    ProgramStatus program;
};

// ============================================================================
// Callback Types
// ============================================================================

using StatusCallback = std::function<void(const MachineStatus&)>;
using ErrorCallback = std::function<void(AlarmCode, const std::string&)>;
using LogCallback = std::function<void(int level, const std::string&)>;
using ProgressCallback = std::function<void(double percent)>;
using LineCallback = std::function<void(int line, const std::string& block)>;

// ============================================================================
// Configuration
// ============================================================================

struct ControllerConfig {
    std::string connection_string;  // "COM3" or "usb:VID:PID" or "192.168.1.100:5000"
    int poll_rate_hz = 50;          // Status polling rate
    int motion_buffer_size = 64;    // Segments to keep buffered on device
    bool enable_soft_limits = true;
    bool enable_hard_limits = true;
    double default_feed_rate = 1000.0;
    double default_rapid_rate = 5000.0;
    Units default_units = Units::MM;
    
    AxisLimits axes[MAX_AXES];
};

// ============================================================================
// Forward Declarations
// ============================================================================

class CommunicationInterface;
class GCodeParser;
class TrajectoryPlanner;

// ============================================================================
// Controller Class
// ============================================================================

class OPENCNC_API Controller {
public:
    Controller();
    ~Controller();
    
    // Non-copyable, movable
    Controller(const Controller&) = delete;
    Controller& operator=(const Controller&) = delete;
    Controller(Controller&&) noexcept;
    Controller& operator=(Controller&&) noexcept;
    
    // ========================================================================
    // Connection
    // ========================================================================
    
    bool connect(const std::string& connection_string);
    void disconnect();
    bool isConnected() const;
    std::string getConnectionInfo() const;
    
    // ========================================================================
    // Configuration
    // ========================================================================
    
    bool loadConfig(const std::string& config_file);
    bool saveConfig(const std::string& config_file);
    void setConfig(const ControllerConfig& config);
    ControllerConfig getConfig() const;
    
    // ========================================================================
    // Program Control
    // ========================================================================
    
    bool loadProgram(const std::string& filename);
    bool loadProgramFromString(const std::string& gcode);
    void unloadProgram();
    bool isProgramLoaded() const;
    std::string getProgramFilename() const;
    
    bool start();
    bool pause();
    bool resume();
    bool stop();
    bool reset();
    
    bool isRunning() const;
    bool isPaused() const;
    
    // Single block mode
    void setSingleBlockMode(bool enabled);
    bool getSingleBlockMode() const;
    bool stepSingleBlock();
    
    // Optional stop (M1)
    void setOptionalStop(bool enabled);
    bool getOptionalStop() const;
    
    // Block delete (/)
    void setBlockDelete(bool enabled);
    bool getBlockDelete() const;
    
    // ========================================================================
    // Manual Control
    // ========================================================================
    
    // Jog control
    bool jogStart(int axis, double velocity, JogMode mode = JogMode::CONTINUOUS);
    bool jogStop(int axis);
    bool jogStopAll();
    bool jogIncrement(int axis, double distance, double velocity);
    
    // Handwheel/MPG
    bool enableHandwheel(int axis, double scale);
    bool disableHandwheel();
    
    // Homing
    bool homeAxis(int axis);
    bool homeAll();
    bool isHoming() const;
    void cancelHoming();
    
    // Probing
    bool probe(int axis, double distance, double feed);
    bool probeToward(const Position& target, double feed);
    bool probeAway(const Position& target, double feed);
    Position getProbePosition() const;
    
    // ========================================================================
    // MDI (Manual Data Input)
    // ========================================================================
    
    bool executeMDI(const std::string& gcode_line);
    bool executeMDIAsync(const std::string& gcode_line);
    
    // ========================================================================
    // Spindle Control
    // ========================================================================
    
    bool spindleOn(SpindleState direction, double rpm);
    bool spindleOff();
    bool setSpindleSpeed(double rpm);
    SpindleStatus getSpindleStatus() const;
    
    // ========================================================================
    // Coolant Control
    // ========================================================================
    
    bool setCoolant(CoolantState state);
    CoolantState getCoolantState() const;
    
    // ========================================================================
    // Tool Control
    // ========================================================================
    
    bool toolChange(int tool_number);
    int getCurrentTool() const;
    ToolInfo getToolInfo(int tool_number) const;
    bool setToolInfo(int tool_number, const ToolInfo& info);
    
    // ========================================================================
    // Overrides
    // ========================================================================
    
    void setFeedOverride(double percent);
    void setRapidOverride(double percent);
    void setSpindleOverride(double percent);
    double getFeedOverride() const;
    double getRapidOverride() const;
    double getSpindleOverride() const;
    
    // ========================================================================
    // Coordinate Systems
    // ========================================================================
    
    void setCoordSystem(CoordSystem cs);
    CoordSystem getCoordSystem() const;
    
    Position getOffset(CoordSystem cs) const;
    bool setOffset(CoordSystem cs, const Position& offset);
    
    void setG92Offset(const Position& offset);
    void clearG92Offset();
    
    // ========================================================================
    // Status & Feedback
    // ========================================================================
    
    MachineState getState() const;
    MachineStatus getStatus() const;
    Position getCurrentPosition() const;         // Work coordinates
    Position getMachinePosition() const;         // Machine coordinates
    Position getTargetPosition() const;
    Position getDistanceToGo() const;
    
    AlarmCode getAlarmCode() const;
    std::string getAlarmMessage() const;
    bool clearAlarm();
    
    // Program status
    int getCurrentLine() const;
    std::string getCurrentBlock() const;
    double getProgress() const;
    
    // ========================================================================
    // Emergency Stop
    // ========================================================================
    
    void emergencyStop();
    bool isEstopped() const;
    bool resetEstop();
    
    // ========================================================================
    // Digital I/O
    // ========================================================================
    
    bool getInput(int pin) const;
    uint32_t getInputs() const;
    bool setOutput(int pin, bool state);
    bool getOutput(int pin) const;
    uint32_t getOutputs() const;
    
    // ========================================================================
    // Callbacks
    // ========================================================================
    
    void setStatusCallback(StatusCallback callback);
    void setErrorCallback(ErrorCallback callback);
    void setLogCallback(LogCallback callback);
    void setProgressCallback(ProgressCallback callback);
    void setLineCallback(LineCallback callback);
    
    // ========================================================================
    // Polling (for non-callback use)
    // ========================================================================
    
    void poll();  // Call periodically if not using callbacks
    
private:
    struct Impl;
    std::unique_ptr<Impl> pImpl;
};

// ============================================================================
// Utility Functions
// ============================================================================

OPENCNC_API std::string machineStateToString(MachineState state);
OPENCNC_API std::string alarmCodeToString(AlarmCode code);
OPENCNC_API std::vector<std::string> enumeratePorts();

} // namespace opencnc

// ============================================================================
// IMPLEMENTATION
// ============================================================================

#ifdef OPENCNC_HMI_IMPLEMENTATION

namespace opencnc {

// ----------------------------------------------------------------------------
// Implementation Details
// ----------------------------------------------------------------------------

struct Controller::Impl {
    // Configuration
    ControllerConfig config;
    
    // State
    std::atomic<bool> connected{false};
    std::atomic<MachineState> state{MachineState::DISCONNECTED};
    MachineStatus status;
    mutable std::mutex status_mutex;
    
    // Program
    std::string program_filename;
    std::vector<std::string> program_lines;
    std::atomic<int> current_line{0};
    std::atomic<bool> program_loaded{false};
    std::atomic<bool> running{false};
    std::atomic<bool> paused{false};
    
    // Mode flags
    std::atomic<bool> single_block_mode{false};
    std::atomic<bool> optional_stop{false};
    std::atomic<bool> block_delete{false};
    
    // Callbacks
    StatusCallback status_callback;
    ErrorCallback error_callback;
    LogCallback log_callback;
    ProgressCallback progress_callback;
    LineCallback line_callback;
    std::mutex callback_mutex;
    
    // Communication
    std::unique_ptr<std::thread> poll_thread;
    std::atomic<bool> poll_running{false};
    
    // Probe result
    Position probe_position;
    std::atomic<bool> probe_valid{false};
    
    void log(int level, const std::string& message) {
        std::lock_guard<std::mutex> lock(callback_mutex);
        if (log_callback) {
            log_callback(level, message);
        }
    }
    
    void notifyStatus() {
        std::lock_guard<std::mutex> lock(callback_mutex);
        if (status_callback) {
            std::lock_guard<std::mutex> slock(status_mutex);
            status_callback(status);
        }
    }
    
    void notifyError(AlarmCode code, const std::string& message) {
        std::lock_guard<std::mutex> lock(callback_mutex);
        if (error_callback) {
            error_callback(code, message);
        }
    }
};

// ----------------------------------------------------------------------------
// Controller Implementation
// ----------------------------------------------------------------------------

Controller::Controller() : pImpl(std::make_unique<Impl>()) {
    // Initialize default axis limits
    for (int i = 0; i < MAX_AXES; i++) {
        pImpl->config.axes[i].enabled = (i < 3); // Enable X, Y, Z by default
        pImpl->config.axes[i].max_velocity = 5000.0;
        pImpl->config.axes[i].max_acceleration = 500.0;
        pImpl->config.axes[i].steps_per_unit = 800.0;
    }
}

Controller::~Controller() {
    disconnect();
}

Controller::Controller(Controller&&) noexcept = default;
Controller& Controller::operator=(Controller&&) noexcept = default;

bool Controller::connect(const std::string& connection_string) {
    if (pImpl->connected) {
        disconnect();
    }
    
    pImpl->config.connection_string = connection_string;
    pImpl->log(0, "Connecting to: " + connection_string);
    
    // TODO: Implement actual connection via comm layer
    // For now, simulate connection
    pImpl->connected = true;
    pImpl->state = MachineState::IDLE;
    
    {
        std::lock_guard<std::mutex> lock(pImpl->status_mutex);
        pImpl->status.state = MachineState::IDLE;
    }
    
    // Start polling thread
    pImpl->poll_running = true;
    pImpl->poll_thread = std::make_unique<std::thread>([this]() {
        while (pImpl->poll_running) {
            poll();
            std::this_thread::sleep_for(
                std::chrono::milliseconds(1000 / pImpl->config.poll_rate_hz)
            );
        }
    });
    
    pImpl->log(0, "Connected successfully");
    return true;
}

void Controller::disconnect() {
    if (!pImpl->connected) return;
    
    pImpl->poll_running = false;
    if (pImpl->poll_thread && pImpl->poll_thread->joinable()) {
        pImpl->poll_thread->join();
    }
    pImpl->poll_thread.reset();
    
    pImpl->connected = false;
    pImpl->state = MachineState::DISCONNECTED;
    
    {
        std::lock_guard<std::mutex> lock(pImpl->status_mutex);
        pImpl->status.state = MachineState::DISCONNECTED;
    }
    
    pImpl->log(0, "Disconnected");
}

bool Controller::isConnected() const {
    return pImpl->connected;
}

std::string Controller::getConnectionInfo() const {
    return pImpl->config.connection_string;
}

bool Controller::loadConfig(const std::string& config_file) {
    pImpl->log(0, "Loading config: " + config_file);
    // TODO: Implement TOML/YAML parsing via config library
    return true;
}

bool Controller::saveConfig(const std::string& config_file) {
    pImpl->log(0, "Saving config: " + config_file);
    // TODO: Implement TOML/YAML writing
    return true;
}

void Controller::setConfig(const ControllerConfig& config) {
    pImpl->config = config;
}

ControllerConfig Controller::getConfig() const {
    return pImpl->config;
}

bool Controller::loadProgram(const std::string& filename) {
    pImpl->log(0, "Loading program: " + filename);
    
    // TODO: Read file and parse G-code
    pImpl->program_filename = filename;
    pImpl->program_loaded = true;
    pImpl->current_line = 0;
    
    return true;
}

bool Controller::loadProgramFromString(const std::string& gcode) {
    pImpl->log(0, "Loading program from string");
    
    // Split into lines
    pImpl->program_lines.clear();
    std::string line;
    for (char c : gcode) {
        if (c == '\n') {
            pImpl->program_lines.push_back(line);
            line.clear();
        } else if (c != '\r') {
            line += c;
        }
    }
    if (!line.empty()) {
        pImpl->program_lines.push_back(line);
    }
    
    pImpl->program_filename = "<string>";
    pImpl->program_loaded = true;
    pImpl->current_line = 0;
    
    return true;
}

void Controller::unloadProgram() {
    stop();
    pImpl->program_lines.clear();
    pImpl->program_filename.clear();
    pImpl->program_loaded = false;
    pImpl->current_line = 0;
}

bool Controller::isProgramLoaded() const {
    return pImpl->program_loaded;
}

std::string Controller::getProgramFilename() const {
    return pImpl->program_filename;
}

bool Controller::start() {
    if (!pImpl->connected || !pImpl->program_loaded) return false;
    if (pImpl->running && !pImpl->paused) return false;
    
    if (pImpl->paused) {
        return resume();
    }
    
    pImpl->running = true;
    pImpl->paused = false;
    pImpl->state = MachineState::RUNNING;
    
    {
        std::lock_guard<std::mutex> lock(pImpl->status_mutex);
        pImpl->status.state = MachineState::RUNNING;
    }
    
    pImpl->log(0, "Program started");
    return true;
}

bool Controller::pause() {
    if (!pImpl->running) return false;
    
    pImpl->paused = true;
    pImpl->state = MachineState::PAUSED;
    
    {
        std::lock_guard<std::mutex> lock(pImpl->status_mutex);
        pImpl->status.state = MachineState::PAUSED;
    }
    
    pImpl->log(0, "Program paused");
    return true;
}

bool Controller::resume() {
    if (!pImpl->paused) return false;
    
    pImpl->paused = false;
    pImpl->state = MachineState::RUNNING;
    
    {
        std::lock_guard<std::mutex> lock(pImpl->status_mutex);
        pImpl->status.state = MachineState::RUNNING;
    }
    
    pImpl->log(0, "Program resumed");
    return true;
}

bool Controller::stop() {
    pImpl->running = false;
    pImpl->paused = false;
    pImpl->state = MachineState::IDLE;
    pImpl->current_line = 0;
    
    {
        std::lock_guard<std::mutex> lock(pImpl->status_mutex);
        pImpl->status.state = MachineState::IDLE;
    }
    
    pImpl->log(0, "Program stopped");
    return true;
}

bool Controller::reset() {
    stop();
    clearAlarm();
    pImpl->log(0, "Controller reset");
    return true;
}

bool Controller::isRunning() const {
    return pImpl->running;
}

bool Controller::isPaused() const {
    return pImpl->paused;
}

void Controller::setSingleBlockMode(bool enabled) {
    pImpl->single_block_mode = enabled;
}

bool Controller::getSingleBlockMode() const {
    return pImpl->single_block_mode;
}

bool Controller::stepSingleBlock() {
    if (!pImpl->single_block_mode || !pImpl->program_loaded) return false;
    // Execute one block
    pImpl->current_line++;
    return true;
}

void Controller::setOptionalStop(bool enabled) {
    pImpl->optional_stop = enabled;
}

bool Controller::getOptionalStop() const {
    return pImpl->optional_stop;
}

void Controller::setBlockDelete(bool enabled) {
    pImpl->block_delete = enabled;
}

bool Controller::getBlockDelete() const {
    return pImpl->block_delete;
}

bool Controller::jogStart(int axis, double velocity, JogMode mode) {
    if (!pImpl->connected || pImpl->running) return false;
    if (axis < 0 || axis >= MAX_AXES) return false;
    
    pImpl->state = MachineState::JOG;
    {
        std::lock_guard<std::mutex> lock(pImpl->status_mutex);
        pImpl->status.state = MachineState::JOG;
    }
    
    // TODO: Send jog command to firmware
    (void)velocity;
    (void)mode;
    
    return true;
}

bool Controller::jogStop(int axis) {
    if (axis < 0 || axis >= MAX_AXES) return false;
    
    // TODO: Send jog stop to firmware
    
    if (pImpl->state == MachineState::JOG) {
        pImpl->state = MachineState::IDLE;
        std::lock_guard<std::mutex> lock(pImpl->status_mutex);
        pImpl->status.state = MachineState::IDLE;
    }
    
    return true;
}

bool Controller::jogStopAll() {
    for (int i = 0; i < MAX_AXES; i++) {
        jogStop(i);
    }
    return true;
}

bool Controller::jogIncrement(int axis, double distance, double velocity) {
    if (!pImpl->connected || pImpl->running) return false;
    if (axis < 0 || axis >= MAX_AXES) return false;
    
    // TODO: Send incremental jog to firmware
    (void)distance;
    (void)velocity;
    
    return true;
}

bool Controller::enableHandwheel(int axis, double scale) {
    (void)axis;
    (void)scale;
    // TODO: Enable MPG mode
    return true;
}

bool Controller::disableHandwheel() {
    // TODO: Disable MPG mode
    return true;
}

bool Controller::homeAxis(int axis) {
    if (!pImpl->connected || pImpl->running) return false;
    if (axis < 0 || axis >= MAX_AXES) return false;
    
    pImpl->state = MachineState::HOMING;
    {
        std::lock_guard<std::mutex> lock(pImpl->status_mutex);
        pImpl->status.state = MachineState::HOMING;
    }
    
    pImpl->log(0, "Homing axis " + std::to_string(axis));
    
    // TODO: Send home command to firmware
    
    return true;
}

bool Controller::homeAll() {
    if (!pImpl->connected || pImpl->running) return false;
    
    pImpl->state = MachineState::HOMING;
    {
        std::lock_guard<std::mutex> lock(pImpl->status_mutex);
        pImpl->status.state = MachineState::HOMING;
    }
    
    pImpl->log(0, "Homing all axes");
    
    // TODO: Send home all command
    
    return true;
}

bool Controller::isHoming() const {
    return pImpl->state == MachineState::HOMING;
}

void Controller::cancelHoming() {
    if (pImpl->state == MachineState::HOMING) {
        pImpl->state = MachineState::IDLE;
        std::lock_guard<std::mutex> lock(pImpl->status_mutex);
        pImpl->status.state = MachineState::IDLE;
    }
}

bool Controller::probe(int axis, double distance, double feed) {
    if (!pImpl->connected || pImpl->running) return false;
    
    pImpl->state = MachineState::PROBING;
    {
        std::lock_guard<std::mutex> lock(pImpl->status_mutex);
        pImpl->status.state = MachineState::PROBING;
    }
    
    // TODO: Execute probe move
    (void)axis;
    (void)distance;
    (void)feed;
    
    return true;
}

bool Controller::probeToward(const Position& target, double feed) {
    (void)target;
    (void)feed;
    return true;
}

bool Controller::probeAway(const Position& target, double feed) {
    (void)target;
    (void)feed;
    return true;
}

Position Controller::getProbePosition() const {
    return pImpl->probe_position;
}

bool Controller::executeMDI(const std::string& gcode_line) {
    if (!pImpl->connected) return false;
    
    pImpl->log(0, "MDI: " + gcode_line);
    
    // TODO: Parse and execute single line
    
    return true;
}

bool Controller::executeMDIAsync(const std::string& gcode_line) {
    return executeMDI(gcode_line);
}

bool Controller::spindleOn(SpindleState direction, double rpm) {
    if (!pImpl->connected) return false;
    
    {
        std::lock_guard<std::mutex> lock(pImpl->status_mutex);
        pImpl->status.spindle.state = direction;
        pImpl->status.spindle.commanded_rpm = rpm;
    }
    
    pImpl->log(0, "Spindle on: " + std::to_string(rpm) + " RPM");
    
    return true;
}

bool Controller::spindleOff() {
    if (!pImpl->connected) return false;
    
    {
        std::lock_guard<std::mutex> lock(pImpl->status_mutex);
        pImpl->status.spindle.state = SpindleState::OFF;
        pImpl->status.spindle.commanded_rpm = 0;
    }
    
    pImpl->log(0, "Spindle off");
    
    return true;
}

bool Controller::setSpindleSpeed(double rpm) {
    if (!pImpl->connected) return false;
    
    {
        std::lock_guard<std::mutex> lock(pImpl->status_mutex);
        pImpl->status.spindle.commanded_rpm = rpm;
    }
    
    return true;
}

SpindleStatus Controller::getSpindleStatus() const {
    std::lock_guard<std::mutex> lock(pImpl->status_mutex);
    return pImpl->status.spindle;
}

bool Controller::setCoolant(CoolantState state) {
    if (!pImpl->connected) return false;
    
    {
        std::lock_guard<std::mutex> lock(pImpl->status_mutex);
        pImpl->status.coolant = state;
    }
    
    return true;
}

CoolantState Controller::getCoolantState() const {
    std::lock_guard<std::mutex> lock(pImpl->status_mutex);
    return pImpl->status.coolant;
}

bool Controller::toolChange(int tool_number) {
    if (!pImpl->connected) return false;
    
    pImpl->state = MachineState::TOOL_CHANGE;
    {
        std::lock_guard<std::mutex> lock(pImpl->status_mutex);
        pImpl->status.state = MachineState::TOOL_CHANGE;
        pImpl->status.next_tool = tool_number;
    }
    
    pImpl->log(0, "Tool change: T" + std::to_string(tool_number));
    
    return true;
}

int Controller::getCurrentTool() const {
    std::lock_guard<std::mutex> lock(pImpl->status_mutex);
    return pImpl->status.current_tool;
}

ToolInfo Controller::getToolInfo(int tool_number) const {
    (void)tool_number;
    return ToolInfo{};
}

bool Controller::setToolInfo(int tool_number, const ToolInfo& info) {
    (void)tool_number;
    (void)info;
    return true;
}

void Controller::setFeedOverride(double percent) {
    std::lock_guard<std::mutex> lock(pImpl->status_mutex);
    pImpl->status.feed_override = std::max(0.0, std::min(200.0, percent));
}

void Controller::setRapidOverride(double percent) {
    std::lock_guard<std::mutex> lock(pImpl->status_mutex);
    pImpl->status.rapid_override = std::max(0.0, std::min(100.0, percent));
}

void Controller::setSpindleOverride(double percent) {
    std::lock_guard<std::mutex> lock(pImpl->status_mutex);
    pImpl->status.spindle_override = std::max(0.0, std::min(200.0, percent));
}

double Controller::getFeedOverride() const {
    std::lock_guard<std::mutex> lock(pImpl->status_mutex);
    return pImpl->status.feed_override;
}

double Controller::getRapidOverride() const {
    std::lock_guard<std::mutex> lock(pImpl->status_mutex);
    return pImpl->status.rapid_override;
}

double Controller::getSpindleOverride() const {
    std::lock_guard<std::mutex> lock(pImpl->status_mutex);
    return pImpl->status.spindle_override;
}

void Controller::setCoordSystem(CoordSystem cs) {
    std::lock_guard<std::mutex> lock(pImpl->status_mutex);
    pImpl->status.coord_system = cs;
}

CoordSystem Controller::getCoordSystem() const {
    std::lock_guard<std::mutex> lock(pImpl->status_mutex);
    return pImpl->status.coord_system;
}

Position Controller::getOffset(CoordSystem cs) const {
    (void)cs;
    return Position{};
}

bool Controller::setOffset(CoordSystem cs, const Position& offset) {
    (void)cs;
    (void)offset;
    return true;
}

void Controller::setG92Offset(const Position& offset) {
    (void)offset;
}

void Controller::clearG92Offset() {
}

MachineState Controller::getState() const {
    return pImpl->state;
}

MachineStatus Controller::getStatus() const {
    std::lock_guard<std::mutex> lock(pImpl->status_mutex);
    return pImpl->status;
}

Position Controller::getCurrentPosition() const {
    std::lock_guard<std::mutex> lock(pImpl->status_mutex);
    return pImpl->status.work_position;
}

Position Controller::getMachinePosition() const {
    std::lock_guard<std::mutex> lock(pImpl->status_mutex);
    return pImpl->status.machine_position;
}

Position Controller::getTargetPosition() const {
    std::lock_guard<std::mutex> lock(pImpl->status_mutex);
    return pImpl->status.target_position;
}

Position Controller::getDistanceToGo() const {
    std::lock_guard<std::mutex> lock(pImpl->status_mutex);
    return pImpl->status.dtg;
}

AlarmCode Controller::getAlarmCode() const {
    std::lock_guard<std::mutex> lock(pImpl->status_mutex);
    return pImpl->status.alarm;
}

std::string Controller::getAlarmMessage() const {
    std::lock_guard<std::mutex> lock(pImpl->status_mutex);
    return pImpl->status.alarm_message;
}

bool Controller::clearAlarm() {
    {
        std::lock_guard<std::mutex> lock(pImpl->status_mutex);
        pImpl->status.alarm = AlarmCode::NONE;
        pImpl->status.alarm_message.clear();
        if (pImpl->status.state == MachineState::ALARM) {
            pImpl->status.state = MachineState::IDLE;
        }
    }
    
    if (pImpl->state == MachineState::ALARM) {
        pImpl->state = MachineState::IDLE;
    }
    
    return true;
}

int Controller::getCurrentLine() const {
    return pImpl->current_line;
}

std::string Controller::getCurrentBlock() const {
    int line = pImpl->current_line;
    if (line >= 0 && line < static_cast<int>(pImpl->program_lines.size())) {
        return pImpl->program_lines[line];
    }
    return "";
}

double Controller::getProgress() const {
    if (pImpl->program_lines.empty()) return 0.0;
    return 100.0 * pImpl->current_line / pImpl->program_lines.size();
}

void Controller::emergencyStop() {
    pImpl->running = false;
    pImpl->paused = false;
    pImpl->state = MachineState::ESTOP;
    
    {
        std::lock_guard<std::mutex> lock(pImpl->status_mutex);
        pImpl->status.state = MachineState::ESTOP;
        pImpl->status.alarm = AlarmCode::ESTOP_TRIGGERED;
        pImpl->status.alarm_message = "Emergency stop activated";
    }
    
    pImpl->log(2, "EMERGENCY STOP");
    pImpl->notifyError(AlarmCode::ESTOP_TRIGGERED, "Emergency stop activated");
}

bool Controller::isEstopped() const {
    return pImpl->state == MachineState::ESTOP;
}

bool Controller::resetEstop() {
    if (pImpl->state != MachineState::ESTOP) return false;
    
    pImpl->state = MachineState::IDLE;
    {
        std::lock_guard<std::mutex> lock(pImpl->status_mutex);
        pImpl->status.state = MachineState::IDLE;
        pImpl->status.alarm = AlarmCode::NONE;
        pImpl->status.alarm_message.clear();
    }
    
    pImpl->log(0, "E-Stop reset");
    return true;
}

bool Controller::getInput(int pin) const {
    std::lock_guard<std::mutex> lock(pImpl->status_mutex);
    return (pImpl->status.input_pins & (1u << pin)) != 0;
}

uint32_t Controller::getInputs() const {
    std::lock_guard<std::mutex> lock(pImpl->status_mutex);
    return pImpl->status.input_pins;
}

bool Controller::setOutput(int pin, bool state) {
    std::lock_guard<std::mutex> lock(pImpl->status_mutex);
    if (state) {
        pImpl->status.output_pins |= (1u << pin);
    } else {
        pImpl->status.output_pins &= ~(1u << pin);
    }
    return true;
}

bool Controller::getOutput(int pin) const {
    std::lock_guard<std::mutex> lock(pImpl->status_mutex);
    return (pImpl->status.output_pins & (1u << pin)) != 0;
}

uint32_t Controller::getOutputs() const {
    std::lock_guard<std::mutex> lock(pImpl->status_mutex);
    return pImpl->status.output_pins;
}

void Controller::setStatusCallback(StatusCallback callback) {
    std::lock_guard<std::mutex> lock(pImpl->callback_mutex);
    pImpl->status_callback = std::move(callback);
}

void Controller::setErrorCallback(ErrorCallback callback) {
    std::lock_guard<std::mutex> lock(pImpl->callback_mutex);
    pImpl->error_callback = std::move(callback);
}

void Controller::setLogCallback(LogCallback callback) {
    std::lock_guard<std::mutex> lock(pImpl->callback_mutex);
    pImpl->log_callback = std::move(callback);
}

void Controller::setProgressCallback(ProgressCallback callback) {
    std::lock_guard<std::mutex> lock(pImpl->callback_mutex);
    pImpl->progress_callback = std::move(callback);
}

void Controller::setLineCallback(LineCallback callback) {
    std::lock_guard<std::mutex> lock(pImpl->callback_mutex);
    pImpl->line_callback = std::move(callback);
}

void Controller::poll() {
    if (!pImpl->connected) return;
    
    // TODO: Poll hardware for real status
    // For now, just notify with current status
    pImpl->notifyStatus();
}

// ----------------------------------------------------------------------------
// Utility Functions
// ----------------------------------------------------------------------------

std::string machineStateToString(MachineState state) {
    switch (state) {
        case MachineState::DISCONNECTED: return "Disconnected";
        case MachineState::IDLE: return "Idle";
        case MachineState::RUNNING: return "Running";
        case MachineState::PAUSED: return "Paused";
        case MachineState::ALARM: return "Alarm";
        case MachineState::HOMING: return "Homing";
        case MachineState::JOG: return "Jog";
        case MachineState::PROBING: return "Probing";
        case MachineState::TOOL_CHANGE: return "Tool Change";
        case MachineState::ESTOP: return "E-Stop";
        default: return "Unknown";
    }
}

std::string alarmCodeToString(AlarmCode code) {
    switch (code) {
        case AlarmCode::NONE: return "No alarm";
        case AlarmCode::HARD_LIMIT_X: return "Hard limit X";
        case AlarmCode::HARD_LIMIT_Y: return "Hard limit Y";
        case AlarmCode::HARD_LIMIT_Z: return "Hard limit Z";
        case AlarmCode::SOFT_LIMIT: return "Soft limit exceeded";
        case AlarmCode::ESTOP_TRIGGERED: return "E-Stop triggered";
        case AlarmCode::SPINDLE_FAULT: return "Spindle fault";
        case AlarmCode::PROBE_FAIL: return "Probe failure";
        case AlarmCode::HOMING_FAIL: return "Homing failure";
        case AlarmCode::FOLLOWING_ERROR: return "Following error";
        case AlarmCode::COMM_TIMEOUT: return "Communication timeout";
        case AlarmCode::COMM_ERROR: return "Communication error";
        case AlarmCode::FIRMWARE_ERROR: return "Firmware error";
        default: return "Unknown alarm";
    }
}

std::vector<std::string> enumeratePorts() {
    std::vector<std::string> ports;
    
#ifdef OPENCNC_PLATFORM_WINDOWS
    // Enumerate COM ports on Windows
    for (int i = 1; i <= 256; i++) {
        std::string port = "COM" + std::to_string(i);
        HANDLE hPort = CreateFileA(
            ("\\\\.\\COM" + std::to_string(i)).c_str(),
            GENERIC_READ | GENERIC_WRITE,
            0, nullptr, OPEN_EXISTING, 0, nullptr
        );
        if (hPort != INVALID_HANDLE_VALUE) {
            CloseHandle(hPort);
            ports.push_back(port);
        }
    }
#endif
    
    return ports;
}

} // namespace opencnc

#endif // OPENCNC_HMI_IMPLEMENTATION

#endif // OPENCNC_HMI_H
