/**
 * @file eth_comm.h
 * @brief Ethernet Communication with Windows HMI
 * 
 * Handles TCP connection to the Windows HMI application.
 * Implements the OpenCNC binary protocol for motion commands,
 * status updates, and configuration.
 */

#ifndef ETH_COMM_H
#define ETH_COMM_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// =============================================================================
// Protocol Constants
// =============================================================================

#define ETH_PROTOCOL_VERSION        2
#define ETH_MAGIC_REQUEST           0xCA
#define ETH_MAGIC_RESPONSE          0xFE
#define ETH_MAX_PACKET_SIZE         1024
#define ETH_HEADER_SIZE             6

// =============================================================================
// Command IDs
// =============================================================================

typedef enum {
    // System commands
    CMD_PING                    = 0x01,
    CMD_GET_VERSION             = 0x02,
    CMD_GET_STATUS              = 0x03,
    CMD_RESET                   = 0x04,
    CMD_ESTOP                   = 0x05,
    CMD_CLEAR_ESTOP             = 0x06,
    
    // Node management
    CMD_GET_NODE_COUNT          = 0x10,
    CMD_GET_NODE_STATUS         = 0x11,
    CMD_GET_NODE_CONFIG         = 0x12,
    CMD_SET_NODE_CONFIG         = 0x13,
    
    // Motion control
    CMD_ENABLE_AXIS             = 0x20,
    CMD_DISABLE_AXIS            = 0x21,
    CMD_HOME_AXIS               = 0x22,
    CMD_HOME_ALL                = 0x23,
    CMD_JOG_START               = 0x24,
    CMD_JOG_STOP                = 0x25,
    
    // Position commands
    CMD_SET_POSITION            = 0x30,
    CMD_GET_POSITION            = 0x31,
    CMD_MOVE_ABSOLUTE           = 0x32,
    CMD_MOVE_RELATIVE           = 0x33,
    CMD_MOVE_LINEAR             = 0x34,     // Multi-axis linear move
    
    // Trajectory streaming
    CMD_TRAJ_CLEAR              = 0x40,
    CMD_TRAJ_ADD_SEGMENT        = 0x41,
    CMD_TRAJ_START              = 0x42,
    CMD_TRAJ_PAUSE              = 0x43,
    CMD_TRAJ_RESUME             = 0x44,
    CMD_TRAJ_GET_STATUS         = 0x45,
    
    // I/O control
    CMD_GET_DIGITAL_IN          = 0x50,
    CMD_SET_DIGITAL_OUT         = 0x51,
    CMD_GET_ANALOG_IN           = 0x52,
    CMD_SET_ANALOG_OUT          = 0x53,
    
    // Spindle control
    CMD_SPINDLE_ON              = 0x60,
    CMD_SPINDLE_OFF             = 0x61,
    CMD_SET_SPINDLE_SPEED       = 0x62,
    CMD_GET_SPINDLE_STATUS      = 0x63,
    
    // Configuration
    CMD_SAVE_CONFIG             = 0xF0,
    CMD_LOAD_CONFIG             = 0xF1,
    CMD_FACTORY_RESET           = 0xF2,
    
} eth_command_t;

// =============================================================================
// Response Status Codes
// =============================================================================

typedef enum {
    RESP_OK                     = 0x00,
    RESP_ERROR_UNKNOWN_CMD      = 0x01,
    RESP_ERROR_INVALID_PARAM    = 0x02,
    RESP_ERROR_AXIS_FAULT       = 0x03,
    RESP_ERROR_NOT_HOMED        = 0x04,
    RESP_ERROR_LIMIT_EXCEEDED   = 0x05,
    RESP_ERROR_BUFFER_FULL      = 0x06,
    RESP_ERROR_TIMEOUT          = 0x07,
    RESP_ERROR_ESTOP_ACTIVE     = 0x08,
    RESP_ERROR_NOT_CONNECTED    = 0x09,
    RESP_ERROR_CRC_MISMATCH     = 0x0A,
} eth_response_t;

// =============================================================================
// Packet Structures
// =============================================================================

#pragma pack(push, 1)

typedef struct {
    uint8_t     magic;          // ETH_MAGIC_REQUEST or ETH_MAGIC_RESPONSE
    uint8_t     command;        // eth_command_t
    uint16_t    length;         // Payload length
    uint16_t    crc;            // CRC-16 of payload
    // Followed by payload bytes
} eth_header_t;

typedef struct {
    uint8_t     magic;
    uint8_t     status;         // eth_response_t
    uint16_t    length;
    uint16_t    crc;
    // Followed by payload bytes
} eth_response_header_t;

// Status response payload
typedef struct {
    uint8_t     state;          // System state
    uint8_t     axis_count;
    uint8_t     io_node_count;
    uint8_t     flags;          // Bitmask: bit0=estop, bit1=homed, bit2=running
    uint32_t    uptime;
    int32_t     position[9];    // Current position for up to 9 axes
    uint16_t    statusword[9];  // CiA 402 statusword per axis
} eth_status_payload_t;

// Node config payload
typedef struct {
    uint8_t     node_id;
    uint8_t     type;           // node_type_t
    uint8_t     axis;           // axis_designation_t
    uint8_t     capabilities_lo;
    uint8_t     capabilities_hi;
    uint8_t     digital_in_count;
    uint8_t     digital_out_count;
    uint8_t     analog_in_count;
    uint32_t    max_velocity;
    uint32_t    max_acceleration;
    uint32_t    steps_per_unit;
} eth_node_config_payload_t;

// Move command payload
typedef struct {
    uint8_t     axis_mask;      // Bitmask of axes to move
    uint8_t     mode;           // 0=absolute, 1=relative
    uint16_t    reserved;
    int32_t     position[9];    // Target positions
    uint32_t    velocity;       // Feedrate
} eth_move_payload_t;

// Trajectory segment payload
typedef struct {
    uint8_t     type;           // 0=linear, 1=arc
    uint8_t     axis_mask;
    uint16_t    flags;
    int32_t     start[9];       // Start position
    int32_t     end[9];         // End position
    uint32_t    velocity;       // Feedrate
    uint32_t    acceleration;
} eth_traj_segment_t;

#pragma pack(pop)

// =============================================================================
// Connection State
// =============================================================================

typedef struct {
    bool        initialized;
    bool        connected;
    bool        got_ip;
    int         socket;
    uint32_t    last_rx_tick;
    uint32_t    rx_count;
    uint32_t    tx_count;
    uint32_t    error_count;
    
    // Receive buffer
    uint8_t     rx_buffer[ETH_MAX_PACKET_SIZE];
    uint16_t    rx_pos;
    
    // Transmit buffer
    uint8_t     tx_buffer[ETH_MAX_PACKET_SIZE];
    
} eth_state_t;

// =============================================================================
// Public Functions
// =============================================================================

/**
 * @brief Initialize Ethernet subsystem
 */
void eth_comm_init(void);

/**
 * @brief Process Ethernet communication (call from task loop)
 */
void eth_comm_tick(void);

/**
 * @brief Check if HMI is connected
 */
bool eth_comm_is_connected(void);

/**
 * @brief Get connection statistics
 */
void eth_comm_get_stats(uint32_t* rx_count, uint32_t* tx_count, uint32_t* errors);

/**
 * @brief Send unsolicited status update to HMI
 */
void eth_comm_send_status_update(void);

/**
 * @brief Send unsolicited alarm to HMI
 */
void eth_comm_send_alarm(uint8_t node_id, uint32_t alarm_code);

/**
 * @brief Close connection
 */
void eth_comm_disconnect(void);

/**
 * @brief Get IP address string
 */
const char* eth_comm_get_ip_address(void);

#ifdef __cplusplus
}
#endif

#endif // ETH_COMM_H
