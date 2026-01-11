/**
 * @file node_config.h
 * @brief Node Configuration Schema and TOML Parser
 * 
 * Defines the self-describing configuration format for axis nodes.
 * Each node stores its configuration in flash and reports it during auto-assign.
 * 
 * Configuration is stored in a simplified TOML-like binary format for
 * embedded systems, with a corresponding TOML text format for PC tools.
 */

#ifndef NODE_CONFIG_H
#define NODE_CONFIG_H

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "auto_assign.h"

#ifdef __cplusplus
extern "C" {
#endif

// =============================================================================
// Configuration Version
// =============================================================================

#define NODE_CONFIG_VERSION     1
#define NODE_CONFIG_MAGIC       0x4E434647      // "NCFG"

// =============================================================================
// Configuration Keys (for binary format)
// =============================================================================

typedef enum {
    CFG_KEY_END             = 0x00,     // End of config
    
    // Identity
    CFG_KEY_SERIAL          = 0x01,     // uint32_t
    CFG_KEY_TYPE            = 0x02,     // uint8_t (node_type_t)
    CFG_KEY_AXIS            = 0x03,     // char
    CFG_KEY_NAME            = 0x04,     // string[16]
    CFG_KEY_VENDOR          = 0x05,     // string[16]
    
    // Capabilities
    CFG_KEY_CAPABILITIES    = 0x10,     // uint16_t bitfield
    CFG_KEY_DIGITAL_IN      = 0x11,     // uint8_t count
    CFG_KEY_DIGITAL_OUT     = 0x12,     // uint8_t count
    CFG_KEY_ANALOG_IN       = 0x13,     // uint8_t count
    CFG_KEY_ANALOG_OUT      = 0x14,     // uint8_t count
    
    // Motion Parameters
    CFG_KEY_STEPS_PER_MM    = 0x20,     // uint32_t (or steps/deg for rotary)
    CFG_KEY_MAX_VELOCITY    = 0x21,     // uint32_t steps/sec
    CFG_KEY_MAX_ACCEL       = 0x22,     // uint32_t steps/sec^2
    CFG_KEY_MAX_CURRENT     = 0x23,     // uint16_t mA
    CFG_KEY_MICROSTEPS      = 0x24,     // uint8_t (1,2,4,8,16,32,64,128,256)
    CFG_KEY_ENCODER_CPR     = 0x25,     // uint32_t counts per revolution
    
    // Limits
    CFG_KEY_SOFT_LIMIT_MIN  = 0x30,     // int32_t steps
    CFG_KEY_SOFT_LIMIT_MAX  = 0x31,     // int32_t steps
    CFG_KEY_HOME_OFFSET     = 0x32,     // int32_t steps
    CFG_KEY_HOME_DIRECTION  = 0x33,     // int8_t (-1 or +1)
    CFG_KEY_HOME_SPEED      = 0x34,     // uint32_t steps/sec
    
    // I/O Mapping
    CFG_KEY_LIMIT_SWITCH_POS = 0x40,    // uint8_t GPIO pin
    CFG_KEY_LIMIT_SWITCH_NEG = 0x41,    // uint8_t GPIO pin
    CFG_KEY_HOME_SWITCH     = 0x42,     // uint8_t GPIO pin
    CFG_KEY_ENABLE_PIN      = 0x43,     // uint8_t GPIO pin
    CFG_KEY_FAULT_PIN       = 0x44,     // uint8_t GPIO pin
    
    // Firmware
    CFG_KEY_FW_VERSION      = 0xF0,     // uint32_t packed (major.minor.build)
    CFG_KEY_HW_REVISION     = 0xF1,     // uint8_t
    CFG_KEY_BOOTLOADER_VER  = 0xF2,     // uint32_t
    
} config_key_t;

// =============================================================================
// Configuration Value Types
// =============================================================================

typedef enum {
    CFG_TYPE_UINT8      = 0,
    CFG_TYPE_INT8       = 1,
    CFG_TYPE_UINT16     = 2,
    CFG_TYPE_INT16      = 3,
    CFG_TYPE_UINT32     = 4,
    CFG_TYPE_INT32      = 5,
    CFG_TYPE_FLOAT      = 6,
    CFG_TYPE_STRING     = 7,
    CFG_TYPE_BOOL       = 8,
    CFG_TYPE_ARRAY      = 9,
} config_value_type_t;

// =============================================================================
// Binary Configuration Entry
// =============================================================================

typedef struct __attribute__((packed)) {
    uint8_t     key;        // config_key_t
    uint8_t     type;       // config_value_type_t  
    uint8_t     length;     // Value length in bytes
    uint8_t     data[61];   // Max 61 bytes for CAN-FD (64 - 3 header)
} config_entry_t;

// =============================================================================
// Full Node Configuration Structure
// =============================================================================

typedef struct {
    // Header
    uint32_t    magic;              // NODE_CONFIG_MAGIC
    uint8_t     version;            // NODE_CONFIG_VERSION
    uint16_t    checksum;           // CRC-16 of config data
    
    // Identity (from auto_assign.h node_config_t)
    uint32_t    serial_number;
    node_type_t type;
    axis_designation_t axis;
    char        name[16];           // Human-readable name
    char        vendor[16];         // Vendor name
    
    // Capabilities
    uint16_t    capabilities;       // Bitfield
    uint8_t     digital_inputs;
    uint8_t     digital_outputs;
    uint8_t     analog_inputs;
    uint8_t     analog_outputs;
    
    // Motion Parameters (for stepper/servo nodes)
    struct {
        uint32_t    steps_per_mm;       // Or steps_per_degree for rotary
        uint32_t    max_velocity;       // Steps per second
        uint32_t    max_acceleration;   // Steps per second^2
        uint16_t    max_current_ma;     // Motor current limit
        uint8_t     microsteps;         // 1,2,4,8,16,32,64,128,256
        uint32_t    encoder_cpr;        // Encoder counts per revolution
    } motion;
    
    // Limits
    struct {
        int32_t     soft_min;           // Soft limit minimum (steps)
        int32_t     soft_max;           // Soft limit maximum (steps)
        int32_t     home_offset;        // Home position offset
        int8_t      home_direction;     // -1 or +1
        uint32_t    home_speed;         // Homing velocity
    } limits;
    
    // I/O Pin Mapping
    struct {
        uint8_t     limit_pos_pin;
        uint8_t     limit_neg_pin;
        uint8_t     home_switch_pin;
        uint8_t     enable_pin;
        uint8_t     fault_pin;
        uint8_t     step_pin;           // For stepper
        uint8_t     dir_pin;            // For stepper
        uint8_t     alarm_pin;          // For servo
    } pins;
    
    // Firmware Info
    struct {
        uint8_t     major;
        uint8_t     minor;
        uint16_t    build;
        uint8_t     hw_revision;
    } firmware;
    
    // Runtime (not stored in flash)
    bool        valid;              // Config loaded successfully
    bool        modified;           // Needs to be saved
    
} full_node_config_t;

// =============================================================================
// Flash Storage Header
// =============================================================================

#define NODE_CONFIG_FLASH_SIZE      512     // Bytes reserved for config

typedef struct __attribute__((packed)) {
    uint32_t    magic;              // NODE_CONFIG_MAGIC
    uint8_t     version;            // NODE_CONFIG_VERSION
    uint8_t     entry_count;        // Number of config entries
    uint16_t    checksum;           // CRC-16 of all entries
    // Followed by config_entry_t entries
} config_flash_header_t;

// =============================================================================
// TOML Template (for documentation and PC tools)
// =============================================================================

// This is the TOML format that corresponds to the binary config:
/*
# OpenCNC Axis Node Configuration
# Copy this template and customize for each axis node

[identity]
serial = 0x12345678          # Unique hardware ID (usually from MCU)
type = "stepper_axis"        # stepper_axis, servo_axis, spindle, io_expander
axis = "X"                   # X, Y, Z, A, B, C, U, V, W
name = "X-Axis Stepper"      # Human-readable name
vendor = "OpenCNC"           # Vendor name

[capabilities]
position_control = true
velocity_control = true  
homing = true
limit_switches = true
encoder = false
can_fd = true

[io]
digital_inputs = 4
digital_outputs = 2
analog_inputs = 0
analog_outputs = 0

[motion]
steps_per_mm = 400           # Steps per mm (or per degree for rotary)
max_velocity = 100000        # Steps per second
max_acceleration = 500000    # Steps per second^2
max_current_ma = 2000        # Motor current in mA
microsteps = 16              # 1, 2, 4, 8, 16, 32, 64, 128, 256
encoder_cpr = 0              # Encoder counts per revolution (0 = no encoder)

[limits]
soft_min = -100000           # Soft limit minimum (steps)
soft_max = 100000            # Soft limit maximum (steps)
home_offset = 0              # Home position offset (steps)
home_direction = -1          # -1 = negative, +1 = positive
home_speed = 5000            # Homing speed (steps/sec)

[pins]
limit_pos = 10               # Positive limit switch GPIO
limit_neg = 11               # Negative limit switch GPIO
home_switch = 12             # Home switch GPIO
enable = 13                  # Motor enable GPIO
fault = 14                   # Fault input GPIO
step = 15                    # Step output GPIO (stepper only)
dir = 16                     # Direction output GPIO (stepper only)

[firmware]
version = "1.0.0"            # Firmware version (read-only)
hw_revision = 1              # Hardware revision (read-only)
*/

// =============================================================================
// Configuration Functions
// =============================================================================

/**
 * @brief Initialize config with defaults
 */
void node_config_init_defaults(full_node_config_t* config, uint32_t serial);

/**
 * @brief Load config from flash
 * @param config Output config structure
 * @param flash_addr Pointer to flash memory
 * @return true if valid config loaded
 */
bool node_config_load_flash(full_node_config_t* config, const void* flash_addr);

/**
 * @brief Save config to flash
 * @param config Config to save
 * @param flash_addr Pointer to flash memory
 * @return true if saved successfully
 */
bool node_config_save_flash(const full_node_config_t* config, void* flash_addr);

/**
 * @brief Serialize config to binary entries (for CAN transmission)
 * @param config Config to serialize
 * @param buffer Output buffer
 * @param buffer_size Buffer size
 * @return Number of bytes written
 */
uint16_t node_config_serialize(const full_node_config_t* config, uint8_t* buffer, uint16_t buffer_size);

/**
 * @brief Deserialize config from binary entries
 * @param config Output config
 * @param buffer Input buffer
 * @param length Buffer length
 * @return true if parsed successfully
 */
bool node_config_deserialize(full_node_config_t* config, const uint8_t* buffer, uint16_t length);

/**
 * @brief Convert full config to compact node_config_t for auto-assign
 */
void node_config_to_compact(const full_node_config_t* full, node_config_t* compact);

/**
 * @brief Expand compact config to full config
 */
void node_config_from_compact(full_node_config_t* full, const node_config_t* compact);

/**
 * @brief Calculate CRC-16 checksum
 */
uint16_t node_config_checksum(const uint8_t* data, uint16_t length);

/**
 * @brief Validate config structure
 * @return true if config is valid
 */
bool node_config_validate(const full_node_config_t* config);

/**
 * @brief Get human-readable node type name
 */
const char* node_config_type_name(node_type_t type);

/**
 * @brief Get axis name string
 */
static inline const char* node_config_axis_name(axis_designation_t axis) {
    switch (axis) {
        case AXIS_X: return "X";
        case AXIS_Y: return "Y";
        case AXIS_Z: return "Z";
        case AXIS_A: return "A";
        case AXIS_B: return "B";
        case AXIS_C: return "C";
        case AXIS_U: return "U";
        case AXIS_V: return "V";
        case AXIS_W: return "W";
        default:     return "?";
    }
}

// =============================================================================
// SDO Access Helpers (Object Dictionary mapping)
// =============================================================================

/**
 * @brief Read config value via SDO object index
 * @param config Node config
 * @param index OD index (0x2000-0x2FFF range)
 * @param subindex OD subindex
 * @param data Output buffer
 * @param size In: buffer size, Out: actual size
 * @return true if read successful
 */
bool node_config_sdo_read(
    const full_node_config_t* config,
    uint16_t index,
    uint8_t subindex,
    void* data,
    uint32_t* size
);

/**
 * @brief Write config value via SDO object index
 * @param config Node config
 * @param index OD index (0x2000-0x2FFF range)
 * @param subindex OD subindex
 * @param data Input data
 * @param size Data size
 * @return true if write successful
 */
bool node_config_sdo_write(
    full_node_config_t* config,
    uint16_t index,
    uint8_t subindex,
    const void* data,
    uint32_t size
);

// =============================================================================
// Default Configuration Templates
// =============================================================================

/**
 * @brief Create default stepper axis config
 */
static inline void node_config_default_stepper(
    full_node_config_t* config,
    uint32_t serial,
    axis_designation_t axis
) {
    node_config_init_defaults(config, serial);
    
    config->type = NODE_TYPE_STEPPER_AXIS;
    config->axis = axis;
    config->capabilities = NODE_CAP_POSITION | NODE_CAP_VELOCITY | 
                          NODE_CAP_HOMING | NODE_CAP_LIMIT_SWITCH |
                          NODE_CAP_DIGITAL_IN | NODE_CAP_CAN_FD;
    
    config->motion.steps_per_mm = 400;          // 200 steps * 2 microsteps / mm
    config->motion.max_velocity = 100000;       // 100k steps/sec
    config->motion.max_acceleration = 500000;   // 500k steps/sec^2
    config->motion.max_current_ma = 2000;       // 2A
    config->motion.microsteps = 16;
    
    config->digital_inputs = 4;
    config->digital_outputs = 2;
}

/**
 * @brief Create default servo axis config
 */
static inline void node_config_default_servo(
    full_node_config_t* config,
    uint32_t serial,
    axis_designation_t axis
) {
    node_config_init_defaults(config, serial);
    
    config->type = NODE_TYPE_SERVO_AXIS;
    config->axis = axis;
    config->capabilities = NODE_CAP_POSITION | NODE_CAP_VELOCITY |
                          NODE_CAP_HOMING | NODE_CAP_LIMIT_SWITCH |
                          NODE_CAP_ENCODER | NODE_CAP_FOC | NODE_CAP_CAN_FD;
    
    config->motion.encoder_cpr = 4000;          // 1000 line encoder * 4x
    config->motion.max_velocity = 200000;       // Counts/sec
    config->motion.max_acceleration = 1000000;  // Counts/sec^2
    config->motion.max_current_ma = 5000;       // 5A peak
    
    config->digital_inputs = 4;
    config->digital_outputs = 2;
}

/**
 * @brief Create default I/O expander config
 */
static inline void node_config_default_io(
    full_node_config_t* config,
    uint32_t serial
) {
    node_config_init_defaults(config, serial);
    
    config->type = NODE_TYPE_IO_EXPANDER;
    config->axis = AXIS_NONE;
    config->capabilities = NODE_CAP_DIGITAL_IN | NODE_CAP_DIGITAL_OUT |
                          NODE_CAP_ANALOG_IN | NODE_CAP_ANALOG_OUT | 
                          NODE_CAP_CAN_FD;
    
    config->digital_inputs = 16;
    config->digital_outputs = 16;
    config->analog_inputs = 4;
    config->analog_outputs = 2;
}

#ifdef __cplusplus
}
#endif

#endif // NODE_CONFIG_H
