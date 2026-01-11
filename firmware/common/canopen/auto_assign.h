/**
 * @file auto_assign.h
 * @brief CANopen Auto-Assign Node Addressing Protocol
 * 
 * Custom protocol for automatic node ID assignment at boot.
 * Uses reserved COB-ID range 0x7E0-0x7EF for discovery.
 * 
 * Protocol Flow:
 * 1. Node boots with ID 0, waits random delay (0-100ms)
 * 2. Node sends DISCOVERY frame with unique serial number
 * 3. Master assigns node ID via ASSIGN frame
 * 4. Node acknowledges with ACK frame using new ID
 * 5. Master queries node capabilities via SDO
 */

#ifndef AUTO_ASSIGN_H
#define AUTO_ASSIGN_H

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "canopen_types.h"

#ifdef __cplusplus
extern "C" {
#endif

// =============================================================================
// Auto-Assign Protocol Constants
// =============================================================================

// COB-IDs for auto-assign protocol (0x7E0 range)
#define AUTOASSIGN_COB_DISCOVERY    0x7E0   // Node → Master: "I exist"
#define AUTOASSIGN_COB_ASSIGN       0x7E1   // Master → Node: "Use this ID"
#define AUTOASSIGN_COB_ACK          0x7E2   // Node → Master: "ID accepted"
#define AUTOASSIGN_COB_QUERY        0x7E3   // Master → Node: "What are you?"
#define AUTOASSIGN_COB_REPORT       0x7E4   // Node → Master: "Here's my config"

// Timeouts
#define AUTOASSIGN_DISCOVERY_TIMEOUT_MS     5000    // Max wait for discovery
#define AUTOASSIGN_ACK_TIMEOUT_MS           500     // Wait for assignment ACK
#define AUTOASSIGN_QUERY_TIMEOUT_MS         1000    // Wait for config report
#define AUTOASSIGN_RANDOM_DELAY_MAX_MS      100     // Random boot delay

// Magic bytes for protocol validation
#define AUTOASSIGN_MAGIC_DISCOVERY  0xCA
#define AUTOASSIGN_MAGIC_ASSIGN     0xFE
#define AUTOASSIGN_MAGIC_ACK        0xED
#define AUTOASSIGN_MAGIC_QUERY      0xD0
#define AUTOASSIGN_MAGIC_REPORT     0xC0

// =============================================================================
// Node Types
// =============================================================================

typedef enum {
    NODE_TYPE_UNKNOWN           = 0x00,
    NODE_TYPE_STEPPER_AXIS      = 0x01,     // Stepper motor axis
    NODE_TYPE_SERVO_AXIS        = 0x02,     // Servo motor axis
    NODE_TYPE_SPINDLE           = 0x03,     // Spindle drive
    NODE_TYPE_IO_EXPANDER       = 0x10,     // Digital/analog I/O
    NODE_TYPE_PROBE             = 0x11,     // Touch probe
    NODE_TYPE_PENDANT           = 0x20,     // Handwheel/pendant
    NODE_TYPE_DISPLAY           = 0x21,     // Remote display
    NODE_TYPE_CUSTOM            = 0xFF      // Vendor defined
} node_type_t;

// =============================================================================
// Axis Designations
// =============================================================================

typedef enum {
    AXIS_NONE   = 0,
    AXIS_X      = 'X',
    AXIS_Y      = 'Y',
    AXIS_Z      = 'Z',
    AXIS_A      = 'A',
    AXIS_B      = 'B',
    AXIS_C      = 'C',
    AXIS_U      = 'U',
    AXIS_V      = 'V',
    AXIS_W      = 'W'
} axis_designation_t;

// =============================================================================
// Node Capabilities (bitfield)
// =============================================================================

typedef enum {
    NODE_CAP_POSITION       = (1 << 0),     // Position control
    NODE_CAP_VELOCITY       = (1 << 1),     // Velocity control
    NODE_CAP_HOMING         = (1 << 2),     // Homing support
    NODE_CAP_LIMIT_SWITCH   = (1 << 3),     // Limit switch inputs
    NODE_CAP_ENCODER        = (1 << 4),     // Encoder feedback
    NODE_CAP_ANALOG_IN      = (1 << 5),     // Analog inputs
    NODE_CAP_ANALOG_OUT     = (1 << 6),     // Analog outputs
    NODE_CAP_DIGITAL_IN     = (1 << 7),     // Digital inputs
    NODE_CAP_DIGITAL_OUT    = (1 << 8),     // Digital outputs
    NODE_CAP_PWM            = (1 << 9),     // PWM output
    NODE_CAP_CAN_FD         = (1 << 10),    // CAN-FD capable
    NODE_CAP_FOC            = (1 << 11),    // Field Oriented Control
} node_capability_t;

// =============================================================================
// Node Configuration (self-describing)
// =============================================================================

typedef struct {
    // Identification
    uint32_t    serial_number;              // Unique hardware ID
    uint8_t     node_id;                    // Assigned ID (0 = unassigned)
    uint8_t     bus_id;                     // Which CAN bus (0, 1, 2)
    
    // Type
    node_type_t         type;
    axis_designation_t  axis;               // For motion nodes
    
    // Capabilities
    uint16_t    capabilities;               // Bitfield of node_capability_t
    
    // I/O counts
    uint8_t     digital_inputs;
    uint8_t     digital_outputs;
    uint8_t     analog_inputs;
    uint8_t     analog_outputs;
    
    // Motion parameters (if applicable)
    uint32_t    max_velocity;               // Steps/sec or encoder counts/sec
    uint32_t    max_acceleration;
    uint32_t    steps_per_unit;             // Steps per mm or degree
    
    // Firmware info
    uint8_t     firmware_major;
    uint8_t     firmware_minor;
    uint16_t    firmware_build;
    
    // Status
    bool        configured;
    bool        operational;
    
} node_config_t;

// =============================================================================
// Auto-Assign Discovery Frame
// =============================================================================

// Discovery: Node announces presence
// Byte 0: Magic (0xCA)
// Byte 1-4: Serial number (32-bit)
// Byte 5: Node type
// Byte 6: Axis designation
// Byte 7: Capabilities low byte

static inline void autoassign_build_discovery(
    const node_config_t* node,
    can_frame_t* frame
) {
    frame->id = AUTOASSIGN_COB_DISCOVERY;
    frame->dlc = 8;
    frame->is_fd = false;
    frame->is_extended = false;
    
    frame->data[0] = AUTOASSIGN_MAGIC_DISCOVERY;
    frame->data[1] = (node->serial_number >> 0) & 0xFF;
    frame->data[2] = (node->serial_number >> 8) & 0xFF;
    frame->data[3] = (node->serial_number >> 16) & 0xFF;
    frame->data[4] = (node->serial_number >> 24) & 0xFF;
    frame->data[5] = (uint8_t)node->type;
    frame->data[6] = (uint8_t)node->axis;
    frame->data[7] = node->capabilities & 0xFF;
}

static inline bool autoassign_parse_discovery(
    const can_frame_t* frame,
    node_config_t* node
) {
    if (frame->id != AUTOASSIGN_COB_DISCOVERY) return false;
    if (frame->dlc < 8) return false;
    if (frame->data[0] != AUTOASSIGN_MAGIC_DISCOVERY) return false;
    
    node->serial_number = frame->data[1] |
                          (frame->data[2] << 8) |
                          (frame->data[3] << 16) |
                          (frame->data[4] << 24);
    node->type = (node_type_t)frame->data[5];
    node->axis = (axis_designation_t)frame->data[6];
    node->capabilities = frame->data[7];  // Low byte only
    node->node_id = 0;  // Not yet assigned
    
    return true;
}

// =============================================================================
// Auto-Assign Assignment Frame
// =============================================================================

// Assign: Master assigns node ID
// Byte 0: Magic (0xFE)
// Byte 1-4: Target serial number
// Byte 5: Assigned node ID
// Byte 6: Bus ID
// Byte 7: Reserved

static inline void autoassign_build_assign(
    uint32_t serial_number,
    uint8_t node_id,
    uint8_t bus_id,
    can_frame_t* frame
) {
    frame->id = AUTOASSIGN_COB_ASSIGN;
    frame->dlc = 8;
    frame->is_fd = false;
    frame->is_extended = false;
    
    frame->data[0] = AUTOASSIGN_MAGIC_ASSIGN;
    frame->data[1] = (serial_number >> 0) & 0xFF;
    frame->data[2] = (serial_number >> 8) & 0xFF;
    frame->data[3] = (serial_number >> 16) & 0xFF;
    frame->data[4] = (serial_number >> 24) & 0xFF;
    frame->data[5] = node_id;
    frame->data[6] = bus_id;
    frame->data[7] = 0;
}

static inline bool autoassign_parse_assign(
    const can_frame_t* frame,
    uint32_t* serial_number,
    uint8_t* node_id,
    uint8_t* bus_id
) {
    if (frame->id != AUTOASSIGN_COB_ASSIGN) return false;
    if (frame->dlc < 7) return false;
    if (frame->data[0] != AUTOASSIGN_MAGIC_ASSIGN) return false;
    
    *serial_number = frame->data[1] |
                     (frame->data[2] << 8) |
                     (frame->data[3] << 16) |
                     (frame->data[4] << 24);
    *node_id = frame->data[5];
    *bus_id = frame->data[6];
    
    return true;
}

// =============================================================================
// Auto-Assign Acknowledge Frame
// =============================================================================

// ACK: Node confirms assignment
// Byte 0: Magic (0xED)
// Byte 1: Assigned node ID
// Byte 2-5: Serial number (for verification)
// Byte 6-7: Capabilities high byte + CAN-FD flag

static inline void autoassign_build_ack(
    const node_config_t* node,
    can_frame_t* frame
) {
    frame->id = AUTOASSIGN_COB_ACK;
    frame->dlc = 8;
    frame->is_fd = false;
    frame->is_extended = false;
    
    frame->data[0] = AUTOASSIGN_MAGIC_ACK;
    frame->data[1] = node->node_id;
    frame->data[2] = (node->serial_number >> 0) & 0xFF;
    frame->data[3] = (node->serial_number >> 8) & 0xFF;
    frame->data[4] = (node->serial_number >> 16) & 0xFF;
    frame->data[5] = (node->serial_number >> 24) & 0xFF;
    frame->data[6] = (node->capabilities >> 8) & 0xFF;
    frame->data[7] = (node->capabilities & NODE_CAP_CAN_FD) ? 0x01 : 0x00;
}

static inline bool autoassign_parse_ack(
    const can_frame_t* frame,
    uint8_t* node_id,
    uint32_t* serial_number,
    bool* can_fd_capable
) {
    if (frame->id != AUTOASSIGN_COB_ACK) return false;
    if (frame->dlc < 8) return false;
    if (frame->data[0] != AUTOASSIGN_MAGIC_ACK) return false;
    
    *node_id = frame->data[1];
    *serial_number = frame->data[2] |
                     (frame->data[3] << 8) |
                     (frame->data[4] << 16) |
                     (frame->data[5] << 24);
    *can_fd_capable = (frame->data[7] & 0x01) != 0;
    
    return true;
}

// =============================================================================
// Extended Configuration Report (CAN-FD only)
// =============================================================================

// For CAN-FD nodes, we can send full config in one frame
typedef struct __attribute__((packed)) {
    uint8_t     magic;              // AUTOASSIGN_MAGIC_REPORT
    uint8_t     node_id;
    uint32_t    serial_number;
    uint8_t     type;
    uint8_t     axis;
    uint16_t    capabilities;
    uint8_t     digital_inputs;
    uint8_t     digital_outputs;
    uint8_t     analog_inputs;
    uint8_t     analog_outputs;
    uint32_t    max_velocity;
    uint32_t    max_acceleration;
    uint32_t    steps_per_unit;
    uint8_t     firmware_major;
    uint8_t     firmware_minor;
    uint16_t    firmware_build;
    uint8_t     reserved[24];       // Pad to 48 bytes
} autoassign_config_report_t;

_Static_assert(sizeof(autoassign_config_report_t) == 48, "Config report must be 48 bytes");

// =============================================================================
// Auto-Assign Master State Machine
// =============================================================================

typedef enum {
    AUTOASSIGN_MASTER_IDLE,
    AUTOASSIGN_MASTER_DISCOVERY_WAIT,       // Waiting for nodes to announce
    AUTOASSIGN_MASTER_ASSIGNING,            // Sending assignments
    AUTOASSIGN_MASTER_WAITING_ACK,          // Waiting for ACK
    AUTOASSIGN_MASTER_QUERYING,             // Getting node configs
    AUTOASSIGN_MASTER_COMPLETE
} autoassign_master_state_t;

typedef struct {
    autoassign_master_state_t state;
    
    // Discovered nodes
    node_config_t   nodes[CANOPEN_MAX_NODES];
    uint8_t         node_count;
    uint8_t         current_assign_idx;
    
    // Timing
    uint32_t        timeout_tick;
    uint32_t        discovery_timeout_ms;
    
    // Next available ID per bus
    uint8_t         next_id[3];             // For 3 CAN buses
    
    // Callbacks
    void (*on_node_discovered)(const node_config_t* node);
    void (*on_node_assigned)(uint8_t node_id, const node_config_t* node);
    void (*on_complete)(uint8_t total_nodes);
    void (*on_error)(const char* message);
    
} autoassign_master_t;

// =============================================================================
// Auto-Assign Slave State Machine
// =============================================================================

typedef enum {
    AUTOASSIGN_SLAVE_BOOT,                  // Just powered on
    AUTOASSIGN_SLAVE_WAIT_DELAY,            // Random delay before discovery
    AUTOASSIGN_SLAVE_DISCOVERY_SENT,        // Discovery sent, waiting for assign
    AUTOASSIGN_SLAVE_ASSIGNED,              // Got ID, sent ACK
    AUTOASSIGN_SLAVE_OPERATIONAL            // Normal CANopen operation
} autoassign_slave_state_t;

typedef struct {
    autoassign_slave_state_t state;
    node_config_t            config;
    
    // Timing
    uint32_t    random_delay_ms;
    uint32_t    delay_tick;
    uint32_t    timeout_tick;
    
    // Callbacks
    void (*on_assigned)(uint8_t node_id);
    void (*on_timeout)(void);
    
} autoassign_slave_t;

// =============================================================================
// Master Functions
// =============================================================================

/**
 * @brief Initialize auto-assign master
 */
void autoassign_master_init(autoassign_master_t* master);

/**
 * @brief Start discovery process
 * @param master Master context
 * @param timeout_ms How long to wait for all nodes
 */
void autoassign_master_start_discovery(autoassign_master_t* master, uint32_t timeout_ms);

/**
 * @brief Process received auto-assign frame
 */
void autoassign_master_process(
    autoassign_master_t* master,
    const can_frame_t* frame,
    void (*tx_callback)(const can_frame_t* frame)
);

/**
 * @brief Tick state machine
 * @param delta_ms Milliseconds since last tick
 */
void autoassign_master_tick(
    autoassign_master_t* master,
    uint32_t delta_ms,
    void (*tx_callback)(const can_frame_t* frame)
);

/**
 * @brief Check if discovery is complete
 */
static inline bool autoassign_master_is_complete(const autoassign_master_t* master) {
    return master->state == AUTOASSIGN_MASTER_COMPLETE;
}

/**
 * @brief Get discovered node by index
 */
static inline const node_config_t* autoassign_master_get_node(
    const autoassign_master_t* master, 
    uint8_t index
) {
    if (index >= master->node_count) return NULL;
    return &master->nodes[index];
}

// =============================================================================
// Slave Functions
// =============================================================================

/**
 * @brief Initialize auto-assign slave
 * @param slave Slave context
 * @param config This node's static configuration
 */
void autoassign_slave_init(autoassign_slave_t* slave, const node_config_t* config);

/**
 * @brief Start auto-assign process (call after power-on)
 * @param random_seed Random seed for delay (use hardware RNG or ADC noise)
 */
void autoassign_slave_start(autoassign_slave_t* slave, uint32_t random_seed);

/**
 * @brief Process received auto-assign frame
 */
void autoassign_slave_process(
    autoassign_slave_t* slave,
    const can_frame_t* frame,
    void (*tx_callback)(const can_frame_t* frame)
);

/**
 * @brief Tick state machine
 */
void autoassign_slave_tick(
    autoassign_slave_t* slave,
    uint32_t delta_ms,
    void (*tx_callback)(const can_frame_t* frame)
);

/**
 * @brief Check if slave has been assigned an ID
 */
static inline bool autoassign_slave_is_assigned(const autoassign_slave_t* slave) {
    return slave->state == AUTOASSIGN_SLAVE_ASSIGNED ||
           slave->state == AUTOASSIGN_SLAVE_OPERATIONAL;
}

/**
 * @brief Get assigned node ID
 */
static inline uint8_t autoassign_slave_get_id(const autoassign_slave_t* slave) {
    return slave->config.node_id;
}

#ifdef __cplusplus
}
#endif

#endif // AUTO_ASSIGN_H
