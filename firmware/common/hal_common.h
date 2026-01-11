/*
 * OpenCNC HAL Firmware - Common Definitions
 * 
 * Platform-independent definitions and structures.
 * 
 * Copyright (c) 2024 OpenCNC Project
 * MIT License
 */

#ifndef CNC_HAL_COMMON_H
#define CNC_HAL_COMMON_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// ============================================================================
// Configuration
// ============================================================================

#define MAX_AXES        6
#define MOTION_QUEUE_SIZE 32
#define STATUS_UPDATE_HZ 50

// ============================================================================
// Types
// ============================================================================

typedef int32_t steps_t;
typedef int32_t velocity_t;   // steps/sec
typedef int32_t accel_t;      // steps/sec^2

// ============================================================================
// Machine State
// ============================================================================

typedef enum {
    STATE_IDLE = 0,
    STATE_RUNNING,
    STATE_PAUSED,
    STATE_HOMING,
    STATE_PROBING,
    STATE_JOG,
    STATE_ALARM,
    STATE_ESTOP
} machine_state_t;

typedef enum {
    ALARM_NONE = 0,
    ALARM_LIMIT_X,
    ALARM_LIMIT_Y,
    ALARM_LIMIT_Z,
    ALARM_SOFT_LIMIT,
    ALARM_ESTOP,
    ALARM_PROBE_FAIL,
    ALARM_HOME_FAIL,
    ALARM_SPINDLE,
    ALARM_COMM_ERROR
} alarm_code_t;

// ============================================================================
// Motion Segment
// ============================================================================

typedef enum {
    MOTION_LINEAR = 0,
    MOTION_RAPID,
    MOTION_ARC,
    MOTION_DWELL
} motion_type_t;

typedef struct {
    uint32_t id;
    motion_type_t type;
    uint8_t flags;
    
    // Target position (steps)
    steps_t target[MAX_AXES];
    
    // Velocity profile
    velocity_t entry_velocity;
    velocity_t cruise_velocity;
    velocity_t exit_velocity;
    accel_t acceleration;
    accel_t deceleration;
    
    // Timing
    uint32_t accel_ticks;
    uint32_t cruise_ticks;
    uint32_t decel_ticks;
    
    // Dwell time (ms)
    uint16_t dwell_ms;
    
} motion_segment_t;

// ============================================================================
// Axis Configuration
// ============================================================================

typedef struct {
    double steps_per_mm;
    double max_velocity;        // mm/min
    double max_acceleration;    // mm/s^2
    double max_travel;          // mm
    
    bool enabled;
    bool step_invert;
    bool dir_invert;
    bool limit_invert;
    
    int8_t home_direction;      // -1 or +1
    double home_velocity;       // mm/min
    double home_pulloff;        // mm
    
} axis_config_t;

// ============================================================================
// Pin Configuration
// ============================================================================

typedef struct {
    // Step/direction pins
    int8_t step_pin[MAX_AXES];
    int8_t dir_pin[MAX_AXES];
    int8_t enable_pin[MAX_AXES];
    
    // Limit switches
    int8_t limit_min_pin[MAX_AXES];
    int8_t limit_max_pin[MAX_AXES];
    
    // Home switches (can be same as limits)
    int8_t home_pin[MAX_AXES];
    
    // Probe
    int8_t probe_pin;
    
    // E-Stop
    int8_t estop_pin;
    
    // Spindle
    int8_t spindle_pwm_pin;
    int8_t spindle_enable_pin;
    int8_t spindle_dir_pin;
    
    // Coolant
    int8_t coolant_mist_pin;
    int8_t coolant_flood_pin;
    
} pin_config_t;

// ============================================================================
// System Configuration
// ============================================================================

typedef struct {
    axis_config_t axes[MAX_AXES];
    pin_config_t pins;
    
    uint8_t num_axes;
    
    double junction_deviation;
    double arc_tolerance;
    
    uint32_t spindle_min_rpm;
    uint32_t spindle_max_rpm;
    uint32_t spindle_pwm_freq;
    
    bool soft_limits_enabled;
    bool hard_limits_enabled;
    
} system_config_t;

// ============================================================================
// Real-Time Status
// ============================================================================

typedef struct {
    machine_state_t state;
    alarm_code_t alarm;
    
    // Current position (steps)
    steps_t position[MAX_AXES];
    
    // Current velocity (steps/sec)
    velocity_t velocity[MAX_AXES];
    
    // Limit switch state (bit flags)
    uint8_t limit_state;
    uint8_t home_state;
    bool probe_triggered;
    
    // Motion queue
    uint8_t queue_depth;
    uint32_t current_segment_id;
    
    // Spindle
    bool spindle_on;
    bool spindle_cw;
    uint16_t spindle_rpm;
    
    // Coolant
    bool coolant_mist;
    bool coolant_flood;
    
    // Overrides
    uint8_t feed_override;      // 0-200%
    uint8_t rapid_override;     // 0-100%
    uint8_t spindle_override;   // 0-200%
    
} realtime_status_t;

// ============================================================================
// HAL Interface Functions
// ============================================================================

// Initialize hardware
void hal_init(void);

// Configuration
void hal_load_config(system_config_t* config);
void hal_save_config(const system_config_t* config);

// Motion queue
bool hal_motion_queue_push(const motion_segment_t* segment);
bool hal_motion_queue_pop(motion_segment_t* segment);
uint8_t hal_motion_queue_depth(void);
void hal_motion_queue_clear(void);

// Motion control
void hal_motion_start(void);
void hal_motion_pause(void);
void hal_motion_resume(void);
void hal_motion_stop(void);
void hal_motion_estop(void);

// Homing
void hal_home_axis(uint8_t axis);
void hal_home_all(void);
bool hal_is_homing(void);

// Probing
void hal_probe_start(uint8_t axis, int32_t distance, uint16_t feed);
bool hal_is_probing(void);
bool hal_probe_success(void);

// Spindle
void hal_spindle_set(bool on, bool cw, uint16_t rpm);
void hal_spindle_stop(void);

// Coolant
void hal_coolant_set(bool mist, bool flood);

// Status
void hal_get_status(realtime_status_t* status);
void hal_get_position(steps_t* position);

// I/O
uint32_t hal_get_inputs(void);
void hal_set_outputs(uint32_t outputs);

// Communication
void hal_comm_process(void);
void hal_send_status(void);

#ifdef __cplusplus
}
#endif

#endif // CNC_HAL_COMMON_H
