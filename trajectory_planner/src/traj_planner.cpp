/*
 * Trajectory Planner Implementation
 * 
 * Copyright (c) 2024 OpenCNC Project
 * MIT License
 */

#include "traj_planner.h"
#include "traj_segment.h"
#include "traj_queue.h"
#include <algorithm>
#include <cmath>
#include <mutex>

namespace traj_planner {

// ============================================================================
// Planner Implementation
// ============================================================================

struct TrajectoryPlanner::Impl {
    PlannerConfig config;
    MotionThreadQueue queue{256};
    
    std::atomic<PlannerState> state{PlannerState::IDLE};
    std::atomic<uint32_t> next_segment_id{1};
    
    Position current_position;
    mutable std::mutex position_mutex;
    
    MotionSegment current_segment;
    bool has_current_segment = false;
    double current_segment_time = 0.0;
    
    // Callbacks
    SegmentCompleteCallback segment_complete_cb;
    MotionCompleteCallback motion_complete_cb;
    ErrorCallback error_cb;
    std::mutex callback_mutex;
    
    void notifySegmentComplete(uint32_t id) {
        std::lock_guard<std::mutex> lock(callback_mutex);
        if (segment_complete_cb) {
            segment_complete_cb(id);
        }
    }
    
    void notifyMotionComplete() {
        std::lock_guard<std::mutex> lock(callback_mutex);
        if (motion_complete_cb) {
            motion_complete_cb();
        }
    }
    
    void notifyError(int code, const std::string& msg) {
        std::lock_guard<std::mutex> lock(callback_mutex);
        if (error_cb) {
            error_cb(code, msg);
        }
    }
};

TrajectoryPlanner::TrajectoryPlanner() : pImpl(std::make_unique<Impl>()) {
    // Initialize default axis constraints
    for (int i = 0; i < MAX_AXES; i++) {
        pImpl->config.axes[i].enabled = (i < 3);  // X, Y, Z enabled
    }
}

TrajectoryPlanner::~TrajectoryPlanner() = default;

void TrajectoryPlanner::setConfig(const PlannerConfig& config) {
    pImpl->config = config;
}

PlannerConfig TrajectoryPlanner::getConfig() const {
    return pImpl->config;
}

void TrajectoryPlanner::setAxisConstraints(int axis, const AxisConstraints& constraints) {
    if (axis >= 0 && axis < MAX_AXES) {
        pImpl->config.axes[axis] = constraints;
    }
}

AxisConstraints TrajectoryPlanner::getAxisConstraints(int axis) const {
    if (axis >= 0 && axis < MAX_AXES) {
        return pImpl->config.axes[axis];
    }
    return AxisConstraints{};
}

uint32_t TrajectoryPlanner::addLinearMove(const Position& target, double feed_rate) {
    Position start;
    {
        std::lock_guard<std::mutex> lock(pImpl->position_mutex);
        start = pImpl->current_position;
    }
    
    uint32_t id = pImpl->next_segment_id++;
    MotionSegment seg = createLinearSegment(id, start, target, feed_rate);
    
    pImpl->queue.push(seg);
    
    // Update position for next segment
    {
        std::lock_guard<std::mutex> lock(pImpl->position_mutex);
        pImpl->current_position = target;
    }
    
    return id;
}

uint32_t TrajectoryPlanner::addRapidMove(const Position& target) {
    return addLinearMove(target, pImpl->config.rapid_feed_rate);
}

uint32_t TrajectoryPlanner::addArcMove(const Position& target, const Position& center,
                                        bool clockwise, int plane, double feed_rate) {
    Position start;
    {
        std::lock_guard<std::mutex> lock(pImpl->position_mutex);
        start = pImpl->current_position;
    }
    
    uint32_t id = pImpl->next_segment_id++;
    MotionSegment seg = createArcSegment(id, start, target, center, 
                                          clockwise, plane, feed_rate);
    
    pImpl->queue.push(seg);
    
    {
        std::lock_guard<std::mutex> lock(pImpl->position_mutex);
        pImpl->current_position = target;
    }
    
    return id;
}

uint32_t TrajectoryPlanner::addDwell(double seconds) {
    uint32_t id = pImpl->next_segment_id++;
    
    MotionSegment seg;
    seg.id = id;
    seg.state = SegmentState::PLANNED;
    seg.motion_type = MotionType::DWELL;
    seg.length = 0.0;
    seg.profile.type = ProfileType::CONSTANT;
    seg.profile.t_cruise = seconds;
    
    {
        std::lock_guard<std::mutex> lock(pImpl->position_mutex);
        seg.start_pos = pImpl->current_position;
        seg.end_pos = pImpl->current_position;
    }
    
    pImpl->queue.push(seg);
    
    return id;
}

void TrajectoryPlanner::clearQueue() {
    pImpl->queue.clear();
}

int TrajectoryPlanner::getQueueDepth() const {
    return static_cast<int>(pImpl->queue.size());
}

int TrajectoryPlanner::getQueueCapacity() const {
    return static_cast<int>(pImpl->queue.capacity());
}

bool TrajectoryPlanner::isQueueFull() const {
    return pImpl->queue.full();
}

bool TrajectoryPlanner::isQueueEmpty() const {
    return pImpl->queue.empty();
}

void TrajectoryPlanner::plan() {
    std::lock_guard<std::mutex> lock(pImpl->queue.getMutex());
    
    size_t count = pImpl->queue.size();
    if (count == 0) return;
    
    // Forward pass: calculate maximum entry velocities
    for (size_t i = 0; i < count; i++) {
        MotionSegment* seg = pImpl->queue.at(i);
        if (!seg) continue;
        
        // Calculate max velocity for this segment
        double max_vel = calculateMaxVelocity(*seg, pImpl->config.axes);
        seg->profile.cruise_velocity = std::min(seg->commanded_feed / 60.0, max_vel);
        
        if (i > 0) {
            MotionSegment* prev = pImpl->queue.at(i - 1);
            if (prev) {
                seg->max_junction_velocity = calculateJunctionVelocity(
                    *prev, *seg, 
                    pImpl->config.junction_deviation,
                    std::min(prev->profile.cruise_velocity, seg->profile.cruise_velocity)
                );
            }
        } else {
            seg->max_junction_velocity = 0.0;
        }
    }
    
    // Backward pass: limit velocities based on deceleration constraints
    for (int i = static_cast<int>(count) - 1; i >= 0; i--) {
        MotionSegment* seg = pImpl->queue.at(i);
        if (!seg) continue;
        
        double exit_vel = 0.0;
        if (i < static_cast<int>(count) - 1) {
            MotionSegment* next = pImpl->queue.at(i + 1);
            if (next) {
                exit_vel = next->max_junction_velocity;
            }
        }
        
        // Get deceleration limit
        double max_decel = pImpl->config.axes[0].max_acceleration;  // Simplified
        
        // v² = v₀² + 2as → v₀ = √(v² - 2as)
        double max_entry = std::sqrt(exit_vel * exit_vel + 
                                      2.0 * max_decel * seg->length);
        
        seg->profile.entry_velocity = std::min({
            seg->max_junction_velocity,
            seg->profile.cruise_velocity,
            max_entry
        });
        seg->profile.exit_velocity = exit_vel;
        
        // Plan the velocity profile
        seg->profile = planTrapezoidalProfile(
            seg->length,
            seg->profile.entry_velocity,
            seg->profile.exit_velocity,
            seg->profile.cruise_velocity,
            max_decel,
            max_decel
        );
    }
}

void TrajectoryPlanner::replan() {
    // Re-plan from current position after pause
    plan();
}

void TrajectoryPlanner::start() {
    plan();  // Make sure everything is planned
    pImpl->state = PlannerState::RUNNING;
}

void TrajectoryPlanner::pause() {
    pImpl->state = PlannerState::PAUSED;
}

void TrajectoryPlanner::resume() {
    if (pImpl->state == PlannerState::PAUSED) {
        pImpl->state = PlannerState::RUNNING;
    }
}

void TrajectoryPlanner::stop() {
    pImpl->state = PlannerState::STOPPING;
    clearQueue();
    pImpl->state = PlannerState::IDLE;
}

void TrajectoryPlanner::abort() {
    pImpl->state = PlannerState::IDLE;
    clearQueue();
    pImpl->has_current_segment = false;
}

bool TrajectoryPlanner::isRunning() const {
    return pImpl->state == PlannerState::RUNNING;
}

bool TrajectoryPlanner::isPaused() const {
    return pImpl->state == PlannerState::PAUSED;
}

Position TrajectoryPlanner::getCurrentPosition() const {
    std::lock_guard<std::mutex> lock(pImpl->position_mutex);
    return pImpl->current_position;
}

void TrajectoryPlanner::setCurrentPosition(const Position& pos) {
    std::lock_guard<std::mutex> lock(pImpl->position_mutex);
    pImpl->current_position = pos;
}

MotionState TrajectoryPlanner::interpolate(double dt) {
    MotionState state;
    
    if (!pImpl->has_current_segment) {
        std::lock_guard<std::mutex> lock(pImpl->position_mutex);
        state.position = pImpl->current_position;
        return state;
    }
    
    const auto& seg = pImpl->current_segment;
    const auto& profile = seg.profile;
    
    double t = pImpl->current_segment_time + dt;
    double total_t = profile.total_time();
    
    if (t >= total_t) {
        state.position = seg.end_pos;
        state.at_target = true;
        return state;
    }
    
    // Calculate position along profile
    double velocity, acceleration;
    double distance = interpolateProfile(profile, t, velocity, acceleration);
    
    // Interpolate position
    if (seg.length > EPSILON) {
        double progress = distance / seg.length;
        for (int i = 0; i < MAX_AXES; i++) {
            state.position[i] = seg.start_pos[i] + 
                               (seg.end_pos[i] - seg.start_pos[i]) * progress;
            state.velocity[i] = seg.direction[i] * velocity;
            state.acceleration[i] = seg.direction[i] * acceleration;
        }
    } else {
        state.position = seg.start_pos;
    }
    
    state.current_segment_id = seg.id;
    state.segment_progress = distance / std::max(seg.length, EPSILON);
    state.in_motion = (velocity > EPSILON);
    
    return state;
}

MotionState TrajectoryPlanner::update(double dt) {
    if (pImpl->state != PlannerState::RUNNING) {
        MotionState state;
        std::lock_guard<std::mutex> lock(pImpl->position_mutex);
        state.position = pImpl->current_position;
        return state;
    }
    
    // Check if we need to load a new segment
    if (!pImpl->has_current_segment) {
        MotionSegment seg;
        if (pImpl->queue.pop(seg)) {
            pImpl->current_segment = seg;
            pImpl->current_segment_time = 0.0;
            pImpl->has_current_segment = true;
        } else {
            // Queue empty, motion complete
            pImpl->state = PlannerState::IDLE;
            pImpl->notifyMotionComplete();
            
            MotionState state;
            std::lock_guard<std::mutex> lock(pImpl->position_mutex);
            state.position = pImpl->current_position;
            state.at_target = true;
            return state;
        }
    }
    
    // Advance time
    pImpl->current_segment_time += dt;
    
    // Get interpolated state
    MotionState state = interpolate(0.0);
    
    // Check if segment is complete
    if (pImpl->current_segment_time >= pImpl->current_segment.profile.total_time()) {
        // Segment complete
        {
            std::lock_guard<std::mutex> lock(pImpl->position_mutex);
            pImpl->current_position = pImpl->current_segment.end_pos;
        }
        
        pImpl->notifySegmentComplete(pImpl->current_segment.id);
        pImpl->has_current_segment = false;
    }
    
    return state;
}

PlannerState TrajectoryPlanner::getState() const {
    return pImpl->state;
}

const MotionSegment* TrajectoryPlanner::getCurrentSegment() const {
    if (pImpl->has_current_segment) {
        return &pImpl->current_segment;
    }
    return nullptr;
}

const MotionSegment* TrajectoryPlanner::getNextSegment() const {
    MotionSegment seg;
    if (pImpl->queue.peek(seg)) {
        static MotionSegment next_seg;
        next_seg = seg;
        return &next_seg;
    }
    return nullptr;
}

std::vector<MotionSegment> TrajectoryPlanner::getPlannedSegments() const {
    std::vector<MotionSegment> segments;
    std::lock_guard<std::mutex> lock(pImpl->queue.getMutex());
    
    for (size_t i = 0; i < pImpl->queue.size(); i++) {
        const MotionSegment* seg = pImpl->queue.at(i);
        if (seg) {
            segments.push_back(*seg);
        }
    }
    
    return segments;
}

void TrajectoryPlanner::setSegmentCompleteCallback(SegmentCompleteCallback callback) {
    std::lock_guard<std::mutex> lock(pImpl->callback_mutex);
    pImpl->segment_complete_cb = std::move(callback);
}

void TrajectoryPlanner::setMotionCompleteCallback(MotionCompleteCallback callback) {
    std::lock_guard<std::mutex> lock(pImpl->callback_mutex);
    pImpl->motion_complete_cb = std::move(callback);
}

void TrajectoryPlanner::setErrorCallback(ErrorCallback callback) {
    std::lock_guard<std::mutex> lock(pImpl->callback_mutex);
    pImpl->error_cb = std::move(callback);
}

// ============================================================================
// Motion Queue Implementation
// ============================================================================

struct MotionQueue::Impl {
    MotionThreadQueue queue;
    
    explicit Impl(size_t capacity) : queue(capacity) {}
};

MotionQueue::MotionQueue(size_t capacity) : pImpl(std::make_unique<Impl>(capacity)) {}
MotionQueue::~MotionQueue() = default;

bool MotionQueue::push(const MotionSegment& segment) {
    return pImpl->queue.push(segment);
}

bool MotionQueue::pop(MotionSegment& segment) {
    return pImpl->queue.pop(segment);
}

bool MotionQueue::peek(MotionSegment& segment) const {
    return pImpl->queue.peek(segment);
}

size_t MotionQueue::size() const {
    return pImpl->queue.size();
}

size_t MotionQueue::capacity() const {
    return pImpl->queue.capacity();
}

bool MotionQueue::empty() const {
    return pImpl->queue.empty();
}

bool MotionQueue::full() const {
    return pImpl->queue.full();
}

void MotionQueue::clear() {
    pImpl->queue.clear();
}

MotionSegment* MotionQueue::at(size_t index) {
    return pImpl->queue.at(index);
}

const MotionSegment* MotionQueue::at(size_t index) const {
    return pImpl->queue.at(index);
}

// ============================================================================
// Utility Functions
// ============================================================================

double calculateJunctionVelocity(const MotionSegment& seg1,
                                  const MotionSegment& seg2,
                                  double junction_deviation,
                                  double max_velocity) {
    // Calculate angle between segments
    double cos_angle = seg1.direction.dot(seg2.direction);
    
    // If segments are nearly parallel, use full velocity
    if (cos_angle > 0.9999) {
        return max_velocity;
    }
    
    // If segments are nearly opposite, stop at junction
    if (cos_angle < -0.9999) {
        return 0.0;
    }
    
    // Use junction deviation formula from GRBL/Marlin
    double sin_half_angle = std::sqrt(0.5 * (1.0 - cos_angle));
    double junction_velocity = max_velocity * sin_half_angle;
    
    // Apply junction deviation limit
    // R = deviation / (1 - cos(angle))
    double r = junction_deviation / (1.0 - cos_angle);
    
    // v = sqrt(a * r) where a is centripetal acceleration limit
    double centripetal_limit = std::sqrt(500.0 * r);  // Assume 500 mm/s² limit
    
    return std::min({junction_velocity, centripetal_limit, max_velocity});
}

double calculateMaxVelocity(const MotionSegment& segment,
                            const std::array<AxisConstraints, MAX_AXES>& constraints) {
    double max_vel = 1e9;  // Start with very high value
    
    for (int i = 0; i < MAX_AXES; i++) {
        if (!constraints[i].enabled) continue;
        
        double axis_component = std::fabs(segment.direction[i]);
        if (axis_component > EPSILON) {
            double axis_max = constraints[i].max_velocity / 60.0;  // Convert to mm/s
            double segment_max = axis_max / axis_component;
            max_vel = std::min(max_vel, segment_max);
        }
    }
    
    return max_vel;
}

VelocityProfile planTrapezoidalProfile(double length,
                                        double entry_vel,
                                        double exit_vel,
                                        double max_vel,
                                        double accel,
                                        double decel) {
    VelocityProfile profile;
    profile.type = ProfileType::TRAPEZOIDAL;
    profile.entry_velocity = entry_vel;
    profile.exit_velocity = exit_vel;
    profile.acceleration = accel;
    profile.deceleration = decel;
    
    if (length < EPSILON) {
        return profile;
    }
    
    // Calculate acceleration distance
    double v_peak = max_vel;
    double t_accel = (v_peak - entry_vel) / accel;
    double d_accel = entry_vel * t_accel + 0.5 * accel * t_accel * t_accel;
    
    // Calculate deceleration distance
    double t_decel = (v_peak - exit_vel) / decel;
    double d_decel = v_peak * t_decel - 0.5 * decel * t_decel * t_decel;
    
    // Check if we can reach max velocity
    if (d_accel + d_decel > length) {
        // Triangle profile - can't reach max velocity
        // Solve for intersection velocity
        // d_accel + d_decel = length
        // (v² - v_entry²)/(2*a) + (v² - v_exit²)/(2*d) = length
        double numerator = length + entry_vel * entry_vel / (2.0 * accel) + 
                          exit_vel * exit_vel / (2.0 * decel);
        double denominator = 1.0 / (2.0 * accel) + 1.0 / (2.0 * decel);
        v_peak = std::sqrt(numerator / denominator);
        
        t_accel = (v_peak - entry_vel) / accel;
        t_decel = (v_peak - exit_vel) / decel;
        profile.t_cruise = 0.0;
    } else {
        // Trapezoidal profile with cruise phase
        double d_cruise = length - d_accel - d_decel;
        profile.t_cruise = d_cruise / v_peak;
    }
    
    profile.cruise_velocity = v_peak;
    profile.t_accel = t_accel;
    profile.t_decel = t_decel;
    
    return profile;
}

VelocityProfile planSCurveProfile(double length,
                                   double entry_vel,
                                   double exit_vel,
                                   double max_vel,
                                   double max_accel,
                                   double max_jerk) {
    // Simplified S-curve - for full implementation, see academic papers
    // on jerk-limited trajectory planning
    VelocityProfile profile = planTrapezoidalProfile(length, entry_vel, exit_vel,
                                                      max_vel, max_accel, max_accel);
    profile.type = ProfileType::S_CURVE;
    profile.jerk = max_jerk;
    
    // Add jerk phases (simplified - equal jerk time at start/end of accel/decel)
    double jerk_time = max_accel / max_jerk;
    profile.t_jerk_accel_1 = std::min(jerk_time, profile.t_accel / 2.0);
    profile.t_jerk_accel_2 = profile.t_jerk_accel_1;
    profile.t_jerk_decel_1 = std::min(jerk_time, profile.t_decel / 2.0);
    profile.t_jerk_decel_2 = profile.t_jerk_decel_1;
    
    return profile;
}

double interpolateProfile(const VelocityProfile& profile, double t,
                          double& velocity, double& acceleration) {
    double distance = 0.0;
    
    if (profile.type == ProfileType::CONSTANT) {
        velocity = 0.0;
        acceleration = 0.0;
        return 0.0;
    }
    
    if (t < 0) {
        velocity = profile.entry_velocity;
        acceleration = 0.0;
        return 0.0;
    }
    
    double t_end_accel = profile.t_accel;
    double t_end_cruise = t_end_accel + profile.t_cruise;
    double t_total = t_end_cruise + profile.t_decel;
    
    if (t >= t_total) {
        velocity = profile.exit_velocity;
        acceleration = 0.0;
        
        // Calculate total distance
        double d_accel = profile.entry_velocity * profile.t_accel + 
                        0.5 * profile.acceleration * profile.t_accel * profile.t_accel;
        double d_cruise = profile.cruise_velocity * profile.t_cruise;
        double d_decel = profile.cruise_velocity * profile.t_decel - 
                        0.5 * profile.deceleration * profile.t_decel * profile.t_decel;
        
        return d_accel + d_cruise + d_decel;
    }
    
    if (t < t_end_accel) {
        // Acceleration phase
        velocity = profile.entry_velocity + profile.acceleration * t;
        acceleration = profile.acceleration;
        distance = profile.entry_velocity * t + 0.5 * profile.acceleration * t * t;
    } else if (t < t_end_cruise) {
        // Cruise phase
        double dt = t - t_end_accel;
        velocity = profile.cruise_velocity;
        acceleration = 0.0;
        
        double d_accel = profile.entry_velocity * profile.t_accel + 
                        0.5 * profile.acceleration * profile.t_accel * profile.t_accel;
        distance = d_accel + profile.cruise_velocity * dt;
    } else {
        // Deceleration phase
        double dt = t - t_end_cruise;
        velocity = profile.cruise_velocity - profile.deceleration * dt;
        acceleration = -profile.deceleration;
        
        double d_accel = profile.entry_velocity * profile.t_accel + 
                        0.5 * profile.acceleration * profile.t_accel * profile.t_accel;
        double d_cruise = profile.cruise_velocity * profile.t_cruise;
        distance = d_accel + d_cruise + profile.cruise_velocity * dt - 
                  0.5 * profile.deceleration * dt * dt;
    }
    
    return distance;
}

std::vector<Position> linearizeArc(const Position& start,
                                    const Position& end,
                                    const Position& center,
                                    bool clockwise,
                                    int plane,
                                    double tolerance) {
    std::vector<Position> points;
    points.push_back(start);
    
    int axis1 = 0, axis2 = 1, axis3 = 2;
    switch (plane) {
        case 0: axis1 = 0; axis2 = 1; axis3 = 2; break;  // XY
        case 1: axis1 = 0; axis2 = 2; axis3 = 1; break;  // XZ
        case 2: axis1 = 1; axis2 = 2; axis3 = 0; break;  // YZ
    }
    
    // Calculate radius
    double radius = std::sqrt(
        std::pow(start[axis1] - center[axis1], 2) +
        std::pow(start[axis2] - center[axis2], 2)
    );
    
    // Calculate angles
    double start_angle = std::atan2(start[axis2] - center[axis2],
                                     start[axis1] - center[axis1]);
    double end_angle = std::atan2(end[axis2] - center[axis2],
                                   end[axis1] - center[axis1]);
    
    double total_angle = end_angle - start_angle;
    if (clockwise && total_angle > 0) total_angle -= 2.0 * M_PI;
    if (!clockwise && total_angle < 0) total_angle += 2.0 * M_PI;
    
    // Calculate number of segments based on tolerance
    // chord_error ≈ r * (1 - cos(θ/2)) ≈ r * θ² / 8 for small θ
    double max_angle_per_segment = 2.0 * std::sqrt(2.0 * tolerance / radius);
    int num_segments = static_cast<int>(std::ceil(std::fabs(total_angle) / max_angle_per_segment));
    num_segments = std::max(1, num_segments);
    
    double angle_step = total_angle / num_segments;
    double z_step = (end[axis3] - start[axis3]) / num_segments;
    
    for (int i = 1; i < num_segments; i++) {
        double angle = start_angle + angle_step * i;
        Position point;
        point[axis1] = center[axis1] + radius * std::cos(angle);
        point[axis2] = center[axis2] + radius * std::sin(angle);
        point[axis3] = start[axis3] + z_step * i;
        points.push_back(point);
    }
    
    points.push_back(end);
    return points;
}

} // namespace traj_planner
