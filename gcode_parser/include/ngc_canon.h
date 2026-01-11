/*
 * NGC Canonical Commands Interface
 * 
 * Defines the canonical command interface that bridges the G-code parser
 * to the trajectory planner. Based on LinuxCNC's canonical machining functions.
 * 
 * Copyright (c) 2024 OpenCNC Project
 * MIT License
 */

#ifndef NGC_CANON_H
#define NGC_CANON_H

#include <cstdint>
#include <functional>

namespace ngc_parser {

// ============================================================================
// Canonical Command Interface
// ============================================================================

// This interface can be implemented by the trajectory planner to receive
// parsed G-code commands directly.

class CanonInterface {
public:
    virtual ~CanonInterface() = default;
    
    // ========================================================================
    // Initialization and World Model
    // ========================================================================
    
    virtual void INIT_CANON() = 0;
    virtual void END_CANON() = 0;
    
    // Get current position in program coordinates
    virtual double GET_EXTERNAL_POSITION_X() = 0;
    virtual double GET_EXTERNAL_POSITION_Y() = 0;
    virtual double GET_EXTERNAL_POSITION_Z() = 0;
    virtual double GET_EXTERNAL_POSITION_A() = 0;
    virtual double GET_EXTERNAL_POSITION_B() = 0;
    virtual double GET_EXTERNAL_POSITION_C() = 0;
    
    // Get probe position
    virtual double GET_EXTERNAL_PROBE_POSITION_X() = 0;
    virtual double GET_EXTERNAL_PROBE_POSITION_Y() = 0;
    virtual double GET_EXTERNAL_PROBE_POSITION_Z() = 0;
    
    virtual bool GET_EXTERNAL_PROBE_TRIPPED_VALUE() = 0;
    
    // ========================================================================
    // Motion Commands
    // ========================================================================
    
    // Rapid positioning (G0)
    virtual void STRAIGHT_TRAVERSE(
        int line_number,
        double x, double y, double z,
        double a, double b, double c,
        double u, double v, double w
    ) = 0;
    
    // Linear interpolation (G1)
    virtual void STRAIGHT_FEED(
        int line_number,
        double x, double y, double z,
        double a, double b, double c,
        double u, double v, double w
    ) = 0;
    
    // Circular interpolation (G2/G3)
    // rotation: 1 for <= 360°, 2 for <= 720°, etc.
    virtual void ARC_FEED(
        int line_number,
        double end_x, double end_y,
        double center_x, double center_y,
        int rotation,           // +1 = CCW, -1 = CW
        double end_z,
        double a, double b, double c,
        double u, double v, double w
    ) = 0;
    
    // Probing (G38.x)
    virtual void STRAIGHT_PROBE(
        int line_number,
        double x, double y, double z,
        double a, double b, double c,
        double u, double v, double w,
        int probe_type          // 2, 3, 4, or 5 for G38.2-G38.5
    ) = 0;
    
    // ========================================================================
    // Dwell
    // ========================================================================
    
    virtual void DWELL(double seconds) = 0;
    
    // ========================================================================
    // Spindle Control
    // ========================================================================
    
    virtual void SET_SPINDLE_SPEED(double rpm) = 0;
    virtual void START_SPINDLE_CLOCKWISE() = 0;
    virtual void START_SPINDLE_COUNTERCLOCKWISE() = 0;
    virtual void STOP_SPINDLE_TURNING() = 0;
    virtual void ORIENT_SPINDLE(double orientation, int direction) = 0;
    
    // Spindle state queries
    virtual double GET_EXTERNAL_SPEED() = 0;
    virtual int GET_EXTERNAL_SPINDLE() = 0;  // 0=stopped, 1=CW, 2=CCW
    
    // ========================================================================
    // Tool Management
    // ========================================================================
    
    virtual void SELECT_TOOL(int tool_number) = 0;
    virtual void CHANGE_TOOL(int tool_number) = 0;
    virtual void USE_TOOL_LENGTH_OFFSET(double offset) = 0;
    
    virtual int GET_EXTERNAL_TOOL_SLOT() = 0;
    virtual double GET_EXTERNAL_TOOL_LENGTH_OFFSET() = 0;
    
    // ========================================================================
    // Coolant Control
    // ========================================================================
    
    virtual void MIST_ON() = 0;
    virtual void FLOOD_ON() = 0;
    virtual void MIST_OFF() = 0;
    virtual void FLOOD_OFF() = 0;
    
    virtual int GET_EXTERNAL_MIST() = 0;   // 0=off, 1=on
    virtual int GET_EXTERNAL_FLOOD() = 0;  // 0=off, 1=on
    
    // ========================================================================
    // Feed and Speed Control
    // ========================================================================
    
    virtual void SET_FEED_RATE(double rate) = 0;
    virtual void SET_FEED_REFERENCE(int reference) = 0;  // 1=XYZ, 2=workpiece
    
    virtual void ENABLE_FEED_OVERRIDE() = 0;
    virtual void DISABLE_FEED_OVERRIDE() = 0;
    virtual void ENABLE_SPEED_OVERRIDE() = 0;
    virtual void DISABLE_SPEED_OVERRIDE() = 0;
    
    virtual double GET_EXTERNAL_FEED_RATE() = 0;
    
    // ========================================================================
    // Plane Selection
    // ========================================================================
    
    virtual void SELECT_PLANE(int plane) = 0;  // 1=XY(G17), 2=XZ(G18), 3=YZ(G19)
    
    // ========================================================================
    // Units
    // ========================================================================
    
    virtual void USE_LENGTH_UNITS(int units) = 0;  // 1=inches, 2=mm
    virtual int GET_EXTERNAL_LENGTH_UNIT_TYPE() = 0;
    
    // ========================================================================
    // Coordinate Systems
    // ========================================================================
    
    virtual void SET_ORIGIN_OFFSETS(
        double x, double y, double z,
        double a, double b, double c,
        double u, double v, double w
    ) = 0;
    
    virtual void USE_ORIGIN_OFFSETS() = 0;
    virtual void CANCEL_ORIGIN_OFFSETS() = 0;
    
    virtual void SET_G92_OFFSET(
        double x, double y, double z,
        double a, double b, double c,
        double u, double v, double w
    ) = 0;
    
    virtual void CANCEL_G92_OFFSET() = 0;
    
    // ========================================================================
    // Path Control
    // ========================================================================
    
    // mode: 1=exact path (G61), 2=exact stop (G61.1), 3=blending (G64)
    virtual void SET_MOTION_CONTROL_MODE(int mode, double tolerance) = 0;
    
    // ========================================================================
    // Program Control
    // ========================================================================
    
    virtual void OPTIONAL_PROGRAM_STOP() = 0;
    virtual void PROGRAM_STOP() = 0;
    virtual void PROGRAM_END() = 0;
    
    // ========================================================================
    // Comments and Messages
    // ========================================================================
    
    virtual void COMMENT(const char* text) = 0;
    virtual void MESSAGE(const char* text) = 0;
    
    // ========================================================================
    // Digital I/O
    // ========================================================================
    
    virtual void SET_DIGITAL_OUTPUT(int index, int value) = 0;
    virtual void SET_ANALOG_OUTPUT(int index, double value) = 0;
    virtual int GET_DIGITAL_INPUT(int index) = 0;
    virtual double GET_ANALOG_INPUT(int index) = 0;
    
    virtual void WAIT_FOR_DIGITAL_INPUT(int index, int value, double timeout) = 0;
};

// ============================================================================
// Default Implementation (for testing/simulation)
// ============================================================================

class DefaultCanonInterface : public CanonInterface {
public:
    // Current position
    double pos_x = 0, pos_y = 0, pos_z = 0;
    double pos_a = 0, pos_b = 0, pos_c = 0;
    double pos_u = 0, pos_v = 0, pos_w = 0;
    
    // Probe position
    double probe_x = 0, probe_y = 0, probe_z = 0;
    bool probe_tripped = false;
    
    // State
    double feed_rate = 0;
    double spindle_speed = 0;
    int spindle_state = 0;
    int current_tool = 0;
    double tool_offset = 0;
    int mist = 0, flood = 0;
    int length_units = 2;  // mm
    
    void INIT_CANON() override {}
    void END_CANON() override {}
    
    double GET_EXTERNAL_POSITION_X() override { return pos_x; }
    double GET_EXTERNAL_POSITION_Y() override { return pos_y; }
    double GET_EXTERNAL_POSITION_Z() override { return pos_z; }
    double GET_EXTERNAL_POSITION_A() override { return pos_a; }
    double GET_EXTERNAL_POSITION_B() override { return pos_b; }
    double GET_EXTERNAL_POSITION_C() override { return pos_c; }
    
    double GET_EXTERNAL_PROBE_POSITION_X() override { return probe_x; }
    double GET_EXTERNAL_PROBE_POSITION_Y() override { return probe_y; }
    double GET_EXTERNAL_PROBE_POSITION_Z() override { return probe_z; }
    bool GET_EXTERNAL_PROBE_TRIPPED_VALUE() override { return probe_tripped; }
    
    void STRAIGHT_TRAVERSE(int, double x, double y, double z,
                           double a, double b, double c,
                           double u, double v, double w) override {
        pos_x = x; pos_y = y; pos_z = z;
        pos_a = a; pos_b = b; pos_c = c;
        pos_u = u; pos_v = v; pos_w = w;
    }
    
    void STRAIGHT_FEED(int, double x, double y, double z,
                       double a, double b, double c,
                       double u, double v, double w) override {
        pos_x = x; pos_y = y; pos_z = z;
        pos_a = a; pos_b = b; pos_c = c;
        pos_u = u; pos_v = v; pos_w = w;
    }
    
    void ARC_FEED(int, double end_x, double end_y,
                  double, double, int,
                  double end_z, double a, double b, double c,
                  double u, double v, double w) override {
        pos_x = end_x; pos_y = end_y; pos_z = end_z;
        pos_a = a; pos_b = b; pos_c = c;
        pos_u = u; pos_v = v; pos_w = w;
    }
    
    void STRAIGHT_PROBE(int, double x, double y, double z,
                        double a, double b, double c,
                        double u, double v, double w, int) override {
        pos_x = x; pos_y = y; pos_z = z;
        pos_a = a; pos_b = b; pos_c = c;
        pos_u = u; pos_v = v; pos_w = w;
    }
    
    void DWELL(double) override {}
    
    void SET_SPINDLE_SPEED(double rpm) override { spindle_speed = rpm; }
    void START_SPINDLE_CLOCKWISE() override { spindle_state = 1; }
    void START_SPINDLE_COUNTERCLOCKWISE() override { spindle_state = 2; }
    void STOP_SPINDLE_TURNING() override { spindle_state = 0; }
    void ORIENT_SPINDLE(double, int) override {}
    
    double GET_EXTERNAL_SPEED() override { return spindle_speed; }
    int GET_EXTERNAL_SPINDLE() override { return spindle_state; }
    
    void SELECT_TOOL(int t) override { current_tool = t; }
    void CHANGE_TOOL(int t) override { current_tool = t; }
    void USE_TOOL_LENGTH_OFFSET(double o) override { tool_offset = o; }
    
    int GET_EXTERNAL_TOOL_SLOT() override { return current_tool; }
    double GET_EXTERNAL_TOOL_LENGTH_OFFSET() override { return tool_offset; }
    
    void MIST_ON() override { mist = 1; }
    void FLOOD_ON() override { flood = 1; }
    void MIST_OFF() override { mist = 0; }
    void FLOOD_OFF() override { flood = 0; }
    
    int GET_EXTERNAL_MIST() override { return mist; }
    int GET_EXTERNAL_FLOOD() override { return flood; }
    
    void SET_FEED_RATE(double rate) override { feed_rate = rate; }
    void SET_FEED_REFERENCE(int) override {}
    
    void ENABLE_FEED_OVERRIDE() override {}
    void DISABLE_FEED_OVERRIDE() override {}
    void ENABLE_SPEED_OVERRIDE() override {}
    void DISABLE_SPEED_OVERRIDE() override {}
    
    double GET_EXTERNAL_FEED_RATE() override { return feed_rate; }
    
    void SELECT_PLANE(int) override {}
    
    void USE_LENGTH_UNITS(int u) override { length_units = u; }
    int GET_EXTERNAL_LENGTH_UNIT_TYPE() override { return length_units; }
    
    void SET_ORIGIN_OFFSETS(double, double, double,
                            double, double, double,
                            double, double, double) override {}
    void USE_ORIGIN_OFFSETS() override {}
    void CANCEL_ORIGIN_OFFSETS() override {}
    
    void SET_G92_OFFSET(double, double, double,
                        double, double, double,
                        double, double, double) override {}
    void CANCEL_G92_OFFSET() override {}
    
    void SET_MOTION_CONTROL_MODE(int, double) override {}
    
    void OPTIONAL_PROGRAM_STOP() override {}
    void PROGRAM_STOP() override {}
    void PROGRAM_END() override {}
    
    void COMMENT(const char*) override {}
    void MESSAGE(const char*) override {}
    
    void SET_DIGITAL_OUTPUT(int, int) override {}
    void SET_ANALOG_OUTPUT(int, double) override {}
    int GET_DIGITAL_INPUT(int) override { return 0; }
    double GET_ANALOG_INPUT(int) override { return 0; }
    void WAIT_FOR_DIGITAL_INPUT(int, int, double) override {}
};

} // namespace ngc_parser

#endif // NGC_CANON_H
