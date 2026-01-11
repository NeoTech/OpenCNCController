/**
 * @file cia402.h
 * @brief CiA 402 Drive Profile - State Machine and Objects
 * 
 * Implements the CANopen device profile for drives and motion control.
 * Used by both master (to control nodes) and slaves (state machine).
 */

#ifndef CIA402_H
#define CIA402_H

#include <stdint.h>
#include <stdbool.h>
#include "canopen_types.h"

#ifdef __cplusplus
extern "C" {
#endif

// =============================================================================
// CiA 402 Object Dictionary Indices
// =============================================================================

// Device Control
#define OD_CONTROLWORD              0x6040
#define OD_STATUSWORD               0x6041
#define OD_QUICK_STOP_OPTION        0x605A
#define OD_SHUTDOWN_OPTION          0x605B
#define OD_DISABLE_OPTION           0x605C
#define OD_HALT_OPTION              0x605D
#define OD_FAULT_REACTION           0x605E

// Modes of Operation
#define OD_MODES_OF_OPERATION       0x6060
#define OD_MODES_OF_OPERATION_DISP  0x6061

// Position Control
#define OD_POSITION_ACTUAL          0x6064
#define OD_POSITION_WINDOW          0x6067
#define OD_POSITION_WINDOW_TIME     0x6068
#define OD_TARGET_POSITION          0x607A
#define OD_POSITION_RANGE_LIMIT     0x607B
#define OD_SOFTWARE_POSITION_LIMIT  0x607D
#define OD_MAX_PROFILE_VELOCITY     0x607F
#define OD_PROFILE_VELOCITY         0x6081
#define OD_PROFILE_ACCELERATION     0x6083
#define OD_PROFILE_DECELERATION     0x6084

// Velocity Control
#define OD_VELOCITY_ACTUAL          0x606C
#define OD_TARGET_VELOCITY          0x60FF

// Homing
#define OD_HOMING_METHOD            0x6098
#define OD_HOMING_SPEED_SWITCH      0x6099
#define OD_HOMING_SPEED_ZERO        0x609A
#define OD_HOMING_ACCELERATION      0x609B
#define OD_HOME_OFFSET              0x607C

// Factor Group
#define OD_POSITION_FACTOR          0x6093
#define OD_VELOCITY_FACTOR          0x6094

// Digital I/O
#define OD_DIGITAL_INPUTS           0x60FD
#define OD_DIGITAL_OUTPUTS          0x60FE

// Vendor Specific (0x2000 - 0x5FFF)
#define OD_VENDOR_STEPS_PER_REV     0x2000
#define OD_VENDOR_MICROSTEPS        0x2001
#define OD_VENDOR_CURRENT_LIMIT     0x2002
#define OD_VENDOR_NODE_CONFIG       0x2010  // TOML config hash/version
#define OD_VENDOR_AXIS_NAME         0x2011  // Axis designation (X,Y,Z,A,B,C,U,V,W)
#define OD_VENDOR_DEVICE_TYPE       0x2012  // Stepper, servo, IO expander
#define OD_VENDOR_IO_CAPABILITY     0x2013  // Number of DI/DO/AI/AO

// =============================================================================
// Controlword Bits (0x6040)
// =============================================================================

#define CW_SWITCH_ON                (1 << 0)
#define CW_ENABLE_VOLTAGE           (1 << 1)
#define CW_QUICK_STOP               (1 << 2)    // Active low!
#define CW_ENABLE_OPERATION         (1 << 3)
#define CW_NEW_SETPOINT             (1 << 4)    // PP mode
#define CW_CHANGE_SET_IMMEDIATELY   (1 << 5)    // PP mode
#define CW_ABS_REL                  (1 << 6)    // PP mode: 0=abs, 1=rel
#define CW_FAULT_RESET              (1 << 7)
#define CW_HALT                     (1 << 8)

// Controlword command masks
#define CW_MASK_SHUTDOWN            0x0087      // xxxx.xxxx.0xxx.x110
#define CW_MASK_SWITCH_ON           0x008F      // xxxx.xxxx.0xxx.0111
#define CW_MASK_DISABLE_VOLTAGE     0x0082      // xxxx.xxxx.0xxx.xx0x
#define CW_MASK_QUICK_STOP          0x0086      // xxxx.xxxx.0xxx.x01x
#define CW_MASK_DISABLE_OPERATION   0x008F      // xxxx.xxxx.0xxx.0111
#define CW_MASK_ENABLE_OPERATION    0x008F      // xxxx.xxxx.0xxx.1111
#define CW_MASK_FAULT_RESET         0x0080      // xxxx.xxxx.1xxx.xxxx

// Controlword command values
#define CW_CMD_SHUTDOWN             0x0006      // Transition 2,6,8
#define CW_CMD_SWITCH_ON            0x0007      // Transition 3
#define CW_CMD_DISABLE_VOLTAGE      0x0000      // Transition 7,9,10,12
#define CW_CMD_QUICK_STOP           0x0002      // Transition 11
#define CW_CMD_DISABLE_OPERATION    0x0007      // Transition 5
#define CW_CMD_ENABLE_OPERATION     0x000F      // Transition 4
#define CW_CMD_FAULT_RESET          0x0080      // Transition 15

// =============================================================================
// Statusword Bits (0x6041)
// =============================================================================

#define SW_READY_TO_SWITCH_ON       (1 << 0)
#define SW_SWITCHED_ON              (1 << 1)
#define SW_OPERATION_ENABLED        (1 << 2)
#define SW_FAULT                    (1 << 3)
#define SW_VOLTAGE_ENABLED          (1 << 4)
#define SW_QUICK_STOP               (1 << 5)    // Active low!
#define SW_SWITCH_ON_DISABLED       (1 << 6)
#define SW_WARNING                  (1 << 7)
#define SW_MANUFACTURER_SPECIFIC    (1 << 8)
#define SW_REMOTE                   (1 << 9)    // 1 = remote control active
#define SW_TARGET_REACHED           (1 << 10)
#define SW_INTERNAL_LIMIT           (1 << 11)
#define SW_OP_MODE_SPECIFIC_12      (1 << 12)   // PP: setpoint ack
#define SW_OP_MODE_SPECIFIC_13      (1 << 13)   // PP: following error

// Statusword state masks
#define SW_STATE_MASK               0x006F      // Bits 0-3, 5-6

// =============================================================================
// CiA 402 State Machine States
// =============================================================================

typedef enum {
    CIA402_STATE_NOT_READY          = 0x00,     // Not ready to switch on
    CIA402_STATE_SWITCH_ON_DISABLED = 0x40,     // Switch on disabled
    CIA402_STATE_READY_TO_SWITCH_ON = 0x21,     // Ready to switch on
    CIA402_STATE_SWITCHED_ON        = 0x23,     // Switched on
    CIA402_STATE_OPERATION_ENABLED  = 0x27,     // Operation enabled
    CIA402_STATE_QUICK_STOP_ACTIVE  = 0x07,     // Quick stop active
    CIA402_STATE_FAULT_REACTION     = 0x0F,     // Fault reaction active
    CIA402_STATE_FAULT              = 0x08      // Fault
} cia402_state_t;

// State names for debugging
static const char* const CIA402_STATE_NAMES[] = {
    "NOT_READY_TO_SWITCH_ON",
    "SWITCH_ON_DISABLED",
    "READY_TO_SWITCH_ON",
    "SWITCHED_ON",
    "OPERATION_ENABLED",
    "QUICK_STOP_ACTIVE",
    "FAULT_REACTION_ACTIVE",
    "FAULT"
};

// =============================================================================
// Modes of Operation (0x6060)
// =============================================================================

typedef enum {
    CIA402_MODE_NO_MODE             = 0,
    CIA402_MODE_PROFILE_POSITION    = 1,        // PP
    CIA402_MODE_VELOCITY            = 2,        // VL
    CIA402_MODE_PROFILE_VELOCITY    = 3,        // PV
    CIA402_MODE_TORQUE              = 4,        // TQ
    CIA402_MODE_HOMING              = 6,        // HM
    CIA402_MODE_INTERPOLATED_POS    = 7,        // IP
    CIA402_MODE_CSP                 = 8,        // Cyclic Sync Position
    CIA402_MODE_CSV                 = 9,        // Cyclic Sync Velocity
    CIA402_MODE_CST                 = 10        // Cyclic Sync Torque
} cia402_mode_t;

// =============================================================================
// Homing Methods (0x6098)
// =============================================================================

typedef enum {
    HOMING_ACTUAL_POSITION          = 35,       // Use current position
    HOMING_INDEX_NEGATIVE           = 33,
    HOMING_INDEX_POSITIVE           = 34,
    HOMING_SWITCH_NEGATIVE          = 17,       // Home switch, negative dir
    HOMING_SWITCH_POSITIVE          = 18,       // Home switch, positive dir
    HOMING_LIMIT_NEGATIVE           = 1,        // Limit switch, negative
    HOMING_LIMIT_POSITIVE           = 2,        // Limit switch, positive
    HOMING_SWITCH_NEG_INDEX         = 3,        // Home switch + index
    HOMING_SWITCH_POS_INDEX         = 4
} cia402_homing_method_t;

// =============================================================================
// Drive Data Structure
// =============================================================================

typedef struct {
    // State machine
    cia402_state_t  state;
    uint16_t        controlword;
    uint16_t        statusword;
    
    // Mode
    int8_t          mode_of_operation;
    int8_t          mode_of_operation_display;
    
    // Position
    int32_t         position_actual;            // Encoder counts or steps
    int32_t         position_target;
    int32_t         position_demand;            // Interpolated target
    uint32_t        position_window;
    uint16_t        position_window_time;
    
    // Velocity
    int32_t         velocity_actual;
    int32_t         velocity_target;
    uint32_t        max_profile_velocity;
    uint32_t        profile_velocity;
    uint32_t        profile_acceleration;
    uint32_t        profile_deceleration;
    
    // Homing
    int8_t          homing_method;
    uint32_t        homing_speed_switch;
    uint32_t        homing_speed_zero;
    uint32_t        homing_acceleration;
    int32_t         home_offset;
    
    // Limits
    int32_t         sw_position_limit_min;
    int32_t         sw_position_limit_max;
    
    // Digital I/O
    uint32_t        digital_inputs;
    uint32_t        digital_outputs;
    
    // Flags
    bool            target_reached;
    bool            following_error;
    bool            homing_complete;
    bool            homing_error;
    
} cia402_drive_t;

// =============================================================================
// State Machine Functions
// =============================================================================

/**
 * @brief Get state from statusword
 */
static inline cia402_state_t cia402_get_state(uint16_t statusword) {
    uint16_t masked = statusword & SW_STATE_MASK;
    
    if ((masked & 0x4F) == 0x00) return CIA402_STATE_NOT_READY;
    if ((masked & 0x4F) == 0x40) return CIA402_STATE_SWITCH_ON_DISABLED;
    if ((masked & 0x6F) == 0x21) return CIA402_STATE_READY_TO_SWITCH_ON;
    if ((masked & 0x6F) == 0x23) return CIA402_STATE_SWITCHED_ON;
    if ((masked & 0x6F) == 0x27) return CIA402_STATE_OPERATION_ENABLED;
    if ((masked & 0x6F) == 0x07) return CIA402_STATE_QUICK_STOP_ACTIVE;
    if ((masked & 0x4F) == 0x0F) return CIA402_STATE_FAULT_REACTION;
    if ((masked & 0x4F) == 0x08) return CIA402_STATE_FAULT;
    
    return CIA402_STATE_NOT_READY;
}

/**
 * @brief Update state machine based on controlword
 * @param drive Drive data structure
 * @param controlword New controlword from master
 * @return true if state changed
 */
bool cia402_process_controlword(cia402_drive_t* drive, uint16_t controlword);

/**
 * @brief Build statusword from current state
 * @param drive Drive data structure
 * @return Statusword value
 */
uint16_t cia402_build_statusword(const cia402_drive_t* drive);

/**
 * @brief Initialize drive to default state
 */
void cia402_init(cia402_drive_t* drive);

/**
 * @brief Process state machine tick (call periodically)
 * @param drive Drive data structure
 */
void cia402_tick(cia402_drive_t* drive);

/**
 * @brief Trigger fault condition
 * @param drive Drive data structure
 * @param fault_code Fault code for diagnostics
 */
void cia402_trigger_fault(cia402_drive_t* drive, uint16_t fault_code);

/**
 * @brief Check if drive is ready for motion
 */
static inline bool cia402_is_operational(const cia402_drive_t* drive) {
    return drive->state == CIA402_STATE_OPERATION_ENABLED;
}

#ifdef __cplusplus
}
#endif

#endif // CIA402_H
