/*
 * Motion Segment Definition
 * 
 * Copyright (c) 2024 OpenCNC Project
 * MIT License
 */

#ifndef TRAJ_SEGMENT_H
#define TRAJ_SEGMENT_H

#include "traj_planner.h"
#include <cmath>

namespace traj_planner {

// ============================================================================
// Position Implementation
// ============================================================================

inline Position Position::operator+(const Position& other) const {
    Position result;
    for (size_t i = 0; i < MAX_AXES; i++) {
        result[i] = coords[i] + other.coords[i];
    }
    return result;
}

inline Position Position::operator-(const Position& other) const {
    Position result;
    for (size_t i = 0; i < MAX_AXES; i++) {
        result[i] = coords[i] - other.coords[i];
    }
    return result;
}

inline Position Position::operator*(double scalar) const {
    Position result;
    for (size_t i = 0; i < MAX_AXES; i++) {
        result[i] = coords[i] * scalar;
    }
    return result;
}

inline double Position::dot(const Position& other) const {
    double result = 0.0;
    for (size_t i = 0; i < MAX_AXES; i++) {
        result += coords[i] * other.coords[i];
    }
    return result;
}

inline double Position::magnitude() const {
    return std::sqrt(dot(*this));
}

inline Position Position::normalized() const {
    double mag = magnitude();
    if (mag < EPSILON) {
        return Position{};
    }
    return *this * (1.0 / mag);
}

// ============================================================================
// Velocity Profile Implementation
// ============================================================================

inline double VelocityProfile::total_time() const {
    if (type == ProfileType::S_CURVE) {
        return t_jerk_accel_1 + t_jerk_accel_2 + t_accel +
               t_cruise +
               t_jerk_decel_1 + t_jerk_decel_2 + t_decel;
    }
    return t_accel + t_cruise + t_decel;
}

// ============================================================================
// Segment Helper Functions
// ============================================================================

// Calculate the distance for a given trapezoidal profile
inline double trapezoidalDistance(double v0, double v1, double v_max,
                                   double accel, double decel,
                                   double& t_accel, double& t_cruise, double& t_decel) {
    // Calculate acceleration phase
    double v_peak = v_max;
    t_accel = (v_peak - v0) / accel;
    double d_accel = v0 * t_accel + 0.5 * accel * t_accel * t_accel;
    
    // Calculate deceleration phase
    t_decel = (v_peak - v1) / decel;
    double d_decel = v_peak * t_decel - 0.5 * decel * t_decel * t_decel;
    
    // Cruise phase (may be zero)
    t_cruise = 0.0;
    
    return d_accel + d_decel;
}

// Check if segment is a rapid move
inline bool isRapidMove(const MotionSegment& segment) {
    return segment.motion_type == MotionType::RAPID;
}

// Check if segment is an arc
inline bool isArcMove(const MotionSegment& segment) {
    return segment.motion_type == MotionType::ARC_CW ||
           segment.motion_type == MotionType::ARC_CCW;
}

// Calculate angle between two direction vectors
inline double angleBetween(const Position& dir1, const Position& dir2) {
    double dot_product = dir1.dot(dir2);
    // Clamp to avoid numerical issues with acos
    dot_product = std::max(-1.0, std::min(1.0, dot_product));
    return std::acos(dot_product);
}

// Create segment from linear move
inline MotionSegment createLinearSegment(uint32_t id,
                                          const Position& start,
                                          const Position& end,
                                          double feed_rate,
                                          int line_number = 0) {
    MotionSegment seg;
    seg.id = id;
    seg.state = SegmentState::PLANNED;
    seg.motion_type = MotionType::LINEAR;
    seg.line_number = line_number;
    seg.start_pos = start;
    seg.end_pos = end;
    seg.commanded_feed = feed_rate;
    
    Position delta = end - start;
    seg.length = delta.magnitude();
    seg.direction = delta.normalized();
    
    return seg;
}

// Create segment from arc move
inline MotionSegment createArcSegment(uint32_t id,
                                       const Position& start,
                                       const Position& end,
                                       const Position& center,
                                       bool clockwise,
                                       int plane,
                                       double feed_rate,
                                       int line_number = 0) {
    MotionSegment seg;
    seg.id = id;
    seg.state = SegmentState::PLANNED;
    seg.motion_type = clockwise ? MotionType::ARC_CW : MotionType::ARC_CCW;
    seg.line_number = line_number;
    seg.start_pos = start;
    seg.end_pos = end;
    seg.arc_center = center;
    seg.arc_plane = plane;
    seg.commanded_feed = feed_rate;
    
    // Calculate arc geometry based on plane
    int axis1 = 0, axis2 = 1;
    switch (plane) {
        case 0: axis1 = 0; axis2 = 1; break;  // XY
        case 1: axis1 = 0; axis2 = 2; break;  // XZ
        case 2: axis1 = 1; axis2 = 2; break;  // YZ
    }
    
    // Calculate radius
    double dx = start[axis1] - center[axis1];
    double dy = start[axis2] - center[axis2];
    seg.arc_radius = std::sqrt(dx * dx + dy * dy);
    
    // Calculate angle
    double start_angle = std::atan2(start[axis2] - center[axis2],
                                     start[axis1] - center[axis1]);
    double end_angle = std::atan2(end[axis2] - center[axis2],
                                   end[axis1] - center[axis1]);
    
    seg.arc_angle = end_angle - start_angle;
    if (clockwise && seg.arc_angle > 0) seg.arc_angle -= 2.0 * M_PI;
    if (!clockwise && seg.arc_angle < 0) seg.arc_angle += 2.0 * M_PI;
    
    // Arc length
    seg.length = std::fabs(seg.arc_angle * seg.arc_radius);
    
    // Tangent direction at start
    if (clockwise) {
        seg.direction[axis1] = (start[axis2] - center[axis2]) / seg.arc_radius;
        seg.direction[axis2] = -(start[axis1] - center[axis1]) / seg.arc_radius;
    } else {
        seg.direction[axis1] = -(start[axis2] - center[axis2]) / seg.arc_radius;
        seg.direction[axis2] = (start[axis1] - center[axis1]) / seg.arc_radius;
    }
    
    return seg;
}

} // namespace traj_planner

#endif // TRAJ_SEGMENT_H
