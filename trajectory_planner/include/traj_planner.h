/*
 * Trajectory Planner - Motion Planning Library
 * 
 * A trajectory planning library for CNC motion control with look-ahead,
 * S-curve profiles, and jerk-limited motion planning.
 * 
 * Copyright (c) 2024 OpenCNC Project
 * MIT License
 */

#ifndef TRAJ_PLANNER_H
#define TRAJ_PLANNER_H

#include <cstdint>
#include <cstddef>
#include <array>
#include <vector>
#include <memory>
#include <functional>
#include <atomic>

namespace traj_planner {

// ============================================================================
// Constants
// ============================================================================

constexpr int MAX_AXES = 9;
constexpr int DEFAULT_LOOKAHEAD_DEPTH = 32;
constexpr double EPSILON = 1e-9;

// ============================================================================
// Enumerations
// ============================================================================

enum class MotionType : uint8_t {
    RAPID,          // G0 - Maximum velocity traverse
    LINEAR,         // G1 - Linear interpolation
    ARC_CW,         // G2 - Clockwise arc
    ARC_CCW,        // G3 - Counter-clockwise arc
    DWELL           // G4 - Pause at position
};

enum class ProfileType : uint8_t {
    TRAPEZOIDAL,    // 3-phase: accel, cruise, decel
    S_CURVE,        // 7-phase: jerk-limited
    CONSTANT        // No acceleration (for dwelling)
};

enum class SegmentState : uint8_t {
    EMPTY,
    PLANNED,
    EXECUTING,
    COMPLETE
};

enum class PlannerState : uint8_t {
    IDLE,
    PLANNING,
    RUNNING,
    PAUSED,
    STOPPING,
    ERROR
};

// ============================================================================
// Data Structures
// ============================================================================

// Position in N-dimensional space
struct Position {
    std::array<double, MAX_AXES> coords{};
    
    double& operator[](size_t idx) { return coords[idx]; }
    const double& operator[](size_t idx) const { return coords[idx]; }
    
    Position operator+(const Position& other) const;
    Position operator-(const Position& other) const;
    Position operator*(double scalar) const;
    double dot(const Position& other) const;
    double magnitude() const;
    Position normalized() const;
};

// Axis constraints
struct AxisConstraints {
    double max_velocity = 5000.0;       // mm/min
    double max_acceleration = 500.0;    // mm/s²
    double max_jerk = 5000.0;           // mm/s³
    double steps_per_unit = 800.0;      // steps/mm
    bool enabled = true;
};

// Planner configuration
struct PlannerConfig {
    std::array<AxisConstraints, MAX_AXES> axes;
    
    double junction_deviation = 0.05;   // mm - for path blending
    double arc_tolerance = 0.002;       // mm - for arc linearization
    double min_segment_time = 0.001;    // seconds
    int lookahead_depth = DEFAULT_LOOKAHEAD_DEPTH;
    
    ProfileType default_profile = ProfileType::TRAPEZOIDAL;
    bool enable_jerk_limit = true;
    
    double rapid_feed_rate = 10000.0;   // mm/min
};

// Velocity profile for a segment
struct VelocityProfile {
    ProfileType type = ProfileType::TRAPEZOIDAL;
    
    double entry_velocity = 0.0;        // mm/s at segment start
    double cruise_velocity = 0.0;       // mm/s peak velocity
    double exit_velocity = 0.0;         // mm/s at segment end
    
    double acceleration = 0.0;          // mm/s²
    double deceleration = 0.0;          // mm/s²
    double jerk = 0.0;                  // mm/s³ (for S-curve)
    
    // Phase durations (seconds)
    double t_accel = 0.0;               // Acceleration phase
    double t_cruise = 0.0;              // Constant velocity phase
    double t_decel = 0.0;               // Deceleration phase
    
    // S-curve additional phases
    double t_jerk_accel_1 = 0.0;        // Jerk increasing (accel)
    double t_jerk_accel_2 = 0.0;        // Jerk decreasing (accel)
    double t_jerk_decel_1 = 0.0;        // Jerk increasing (decel)
    double t_jerk_decel_2 = 0.0;        // Jerk decreasing (decel)
    
    double total_time() const;
};

// Motion segment (one move command)
struct MotionSegment {
    uint32_t id = 0;
    SegmentState state = SegmentState::EMPTY;
    MotionType motion_type = MotionType::LINEAR;
    int line_number = 0;                // Source G-code line
    
    // Geometry
    Position start_pos;
    Position end_pos;
    Position direction;                 // Unit vector
    double length = 0.0;                // mm
    
    // Arc parameters (if arc motion)
    Position arc_center;
    double arc_radius = 0.0;
    double arc_angle = 0.0;             // radians
    int arc_plane = 0;                  // 0=XY, 1=XZ, 2=YZ
    
    // Commanded values
    double commanded_feed = 0.0;        // mm/min from G-code
    
    // Planned profile
    VelocityProfile profile;
    
    // Junction velocity with previous segment
    double max_junction_velocity = 0.0;
    
    // Execution tracking
    double progress = 0.0;              // 0.0 to 1.0
    double elapsed_time = 0.0;          // seconds since segment start
};

// Real-time position/velocity output
struct MotionState {
    Position position;
    Position velocity;
    Position acceleration;
    
    uint32_t current_segment_id = 0;
    double segment_progress = 0.0;
    
    bool in_motion = false;
    bool at_target = false;
};

// ============================================================================
// Callback Types
// ============================================================================

using SegmentCompleteCallback = std::function<void(uint32_t segment_id)>;
using MotionCompleteCallback = std::function<void()>;
using ErrorCallback = std::function<void(int code, const std::string& message)>;

// ============================================================================
// Trajectory Planner Class
// ============================================================================

class TrajectoryPlanner {
public:
    TrajectoryPlanner();
    ~TrajectoryPlanner();
    
    // Non-copyable
    TrajectoryPlanner(const TrajectoryPlanner&) = delete;
    TrajectoryPlanner& operator=(const TrajectoryPlanner&) = delete;
    
    // ========================================================================
    // Configuration
    // ========================================================================
    
    void setConfig(const PlannerConfig& config);
    PlannerConfig getConfig() const;
    
    void setAxisConstraints(int axis, const AxisConstraints& constraints);
    AxisConstraints getAxisConstraints(int axis) const;
    
    // ========================================================================
    // Motion Queue
    // ========================================================================
    
    // Add motion to queue (returns segment ID)
    uint32_t addLinearMove(const Position& target, double feed_rate);
    uint32_t addRapidMove(const Position& target);
    uint32_t addArcMove(const Position& target, const Position& center, 
                        bool clockwise, int plane, double feed_rate);
    uint32_t addDwell(double seconds);
    
    // Queue management
    void clearQueue();
    int getQueueDepth() const;
    int getQueueCapacity() const;
    bool isQueueFull() const;
    bool isQueueEmpty() const;
    
    // ========================================================================
    // Planning
    // ========================================================================
    
    // Plan queued segments (call after adding moves)
    void plan();
    
    // Replan from current position (after feed hold)
    void replan();
    
    // ========================================================================
    // Execution Control
    // ========================================================================
    
    void start();
    void pause();      // Decelerate to stop
    void resume();
    void stop();       // Immediate stop (with decel)
    void abort();      // Emergency stop
    
    bool isRunning() const;
    bool isPaused() const;
    
    // ========================================================================
    // Position Tracking
    // ========================================================================
    
    // Get current position (call from control loop)
    Position getCurrentPosition() const;
    void setCurrentPosition(const Position& pos);
    
    // Get interpolated state at time t (relative to segment start)
    MotionState interpolate(double dt);
    
    // Advance planner by dt seconds (call from control loop)
    MotionState update(double dt);
    
    // ========================================================================
    // State Queries
    // ========================================================================
    
    PlannerState getState() const;
    const MotionSegment* getCurrentSegment() const;
    const MotionSegment* getNextSegment() const;
    
    // Get planned segments for visualization
    std::vector<MotionSegment> getPlannedSegments() const;
    
    // ========================================================================
    // Callbacks
    // ========================================================================
    
    void setSegmentCompleteCallback(SegmentCompleteCallback callback);
    void setMotionCompleteCallback(MotionCompleteCallback callback);
    void setErrorCallback(ErrorCallback callback);
    
private:
    struct Impl;
    std::unique_ptr<Impl> pImpl;
};

// ============================================================================
// Motion Queue (Thread-Safe Ring Buffer)
// ============================================================================

class MotionQueue {
public:
    explicit MotionQueue(size_t capacity = 128);
    ~MotionQueue();
    
    bool push(const MotionSegment& segment);
    bool pop(MotionSegment& segment);
    bool peek(MotionSegment& segment) const;
    
    size_t size() const;
    size_t capacity() const;
    bool empty() const;
    bool full() const;
    
    void clear();
    
    // Access for planning (not thread-safe, use with external lock)
    MotionSegment* at(size_t index);
    const MotionSegment* at(size_t index) const;
    
private:
    struct Impl;
    std::unique_ptr<Impl> pImpl;
};

// ============================================================================
// Utility Functions
// ============================================================================

// Calculate junction velocity between two segments
double calculateJunctionVelocity(const MotionSegment& seg1,
                                  const MotionSegment& seg2,
                                  double junction_deviation,
                                  double max_velocity);

// Calculate maximum velocity for segment given constraints
double calculateMaxVelocity(const MotionSegment& segment,
                            const std::array<AxisConstraints, MAX_AXES>& constraints);

// Plan velocity profile for segment
VelocityProfile planTrapezoidalProfile(double length,
                                        double entry_vel,
                                        double exit_vel,
                                        double max_vel,
                                        double accel,
                                        double decel);

VelocityProfile planSCurveProfile(double length,
                                   double entry_vel,
                                   double exit_vel,
                                   double max_vel,
                                   double max_accel,
                                   double max_jerk);

// Interpolate position on velocity profile at time t
double interpolateProfile(const VelocityProfile& profile, double t,
                          double& velocity, double& acceleration);

// Arc linearization
std::vector<Position> linearizeArc(const Position& start,
                                    const Position& end,
                                    const Position& center,
                                    bool clockwise,
                                    int plane,
                                    double tolerance);

} // namespace traj_planner

#endif // TRAJ_PLANNER_H
