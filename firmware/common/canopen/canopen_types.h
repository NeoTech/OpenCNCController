/**
 * @file canopen_types.h
 * @brief CANopen/CAN-FD core types and definitions
 * 
 * Shared between ESP32-P4 master and STM32G4/RP2040 slave nodes.
 * Supports CAN-FD with 64-byte payloads and bitrate switching.
 */

#ifndef CANOPEN_TYPES_H
#define CANOPEN_TYPES_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// =============================================================================
// CAN-FD Configuration
// =============================================================================

// CAN-FD supports up to 64 bytes per frame
#define CAN_FD_MAX_DLC          64
#define CAN_CLASSIC_MAX_DLC     8

// Bitrates (arbitration / data phase for CAN-FD)
#define CAN_BITRATE_ARBITRATION 500000      // 500 kbps arbitration
#define CAN_BITRATE_DATA        2000000     // 2 Mbps data phase (FD only)

// For CAN 2.0B compatibility mode
#define CAN_BITRATE_CLASSIC     1000000     // 1 Mbps

// =============================================================================
// CANopen Function Codes (COB-ID composition)
// =============================================================================

// COB-ID = Function Code (4 bits) + Node ID (7 bits)
// Standard CANopen function codes:
#define CANOPEN_FC_NMT              0x000   // Network Management
#define CANOPEN_FC_SYNC             0x080   // Synchronization
#define CANOPEN_FC_EMERGENCY        0x080   // Emergency (+ node ID)
#define CANOPEN_FC_TIMESTAMP        0x100   // Time stamp
#define CANOPEN_FC_TPDO1            0x180   // Transmit PDO 1
#define CANOPEN_FC_RPDO1            0x200   // Receive PDO 1
#define CANOPEN_FC_TPDO2            0x280   // Transmit PDO 2
#define CANOPEN_FC_RPDO2            0x300   // Receive PDO 2
#define CANOPEN_FC_TPDO3            0x380   // Transmit PDO 3
#define CANOPEN_FC_RPDO3            0x400   // Receive PDO 3
#define CANOPEN_FC_TPDO4            0x480   // Transmit PDO 4
#define CANOPEN_FC_RPDO4            0x500   // Receive PDO 4
#define CANOPEN_FC_SDO_TX           0x580   // SDO server -> client
#define CANOPEN_FC_SDO_RX           0x600   // SDO client -> server
#define CANOPEN_FC_NMT_ERROR        0x700   // NMT error control (heartbeat)

// Custom function codes for auto-assign protocol
#define CANOPEN_FC_AUTOASSIGN       0x7E0   // Auto-assign protocol (vendor-specific)

// =============================================================================
// Node ID Management
// =============================================================================

#define CANOPEN_NODE_ID_MIN         1
#define CANOPEN_NODE_ID_MAX         127
#define CANOPEN_NODE_ID_UNASSIGNED  0       // Before auto-assign
#define CANOPEN_NODE_ID_MASTER      0       // Master uses ID 0 for NMT

#define CANOPEN_MAX_NODES           9       // Practical limit for CNC performance

// =============================================================================
// CAN Frame Structure
// =============================================================================

typedef struct {
    uint32_t    id;             // 11-bit standard or 29-bit extended
    uint8_t     dlc;            // Data length (0-8 classic, 0-64 FD)
    uint8_t     data[CAN_FD_MAX_DLC];
    bool        is_extended;    // Extended ID flag
    bool        is_fd;          // CAN-FD frame flag
    bool        brs;            // Bitrate switch (FD only)
    bool        is_remote;      // Remote frame flag
} can_frame_t;

// =============================================================================
// NMT (Network Management) States
// =============================================================================

typedef enum {
    NMT_STATE_INITIALIZING      = 0x00,
    NMT_STATE_STOPPED           = 0x04,
    NMT_STATE_OPERATIONAL       = 0x05,
    NMT_STATE_PRE_OPERATIONAL   = 0x7F
} nmt_state_t;

// NMT Command Specifiers
typedef enum {
    NMT_CMD_START               = 0x01,     // Enter Operational
    NMT_CMD_STOP                = 0x02,     // Enter Stopped
    NMT_CMD_ENTER_PREOP         = 0x80,     // Enter Pre-operational
    NMT_CMD_RESET_NODE          = 0x81,     // Reset node
    NMT_CMD_RESET_COMM          = 0x82      // Reset communication
} nmt_command_t;

// =============================================================================
// SDO (Service Data Object) Types
// =============================================================================

// SDO Command Specifiers (CCS/SCS)
typedef enum {
    SDO_CCS_DOWNLOAD_INIT       = 1,        // Client download initiate
    SDO_CCS_DOWNLOAD_SEG        = 0,        // Client download segment
    SDO_CCS_UPLOAD_INIT         = 2,        // Client upload initiate
    SDO_CCS_UPLOAD_SEG          = 3,        // Client upload segment
    SDO_CCS_ABORT               = 4,        // Abort transfer
    SDO_CCS_BLOCK_UPLOAD        = 5,        // Block upload
    SDO_CCS_BLOCK_DOWNLOAD      = 6         // Block download
} sdo_command_t;

// SDO Abort Codes
typedef enum {
    SDO_ABORT_TOGGLE_BIT        = 0x05030000,
    SDO_ABORT_TIMEOUT           = 0x05040000,
    SDO_ABORT_INVALID_CS        = 0x05040001,
    SDO_ABORT_OUT_OF_MEMORY     = 0x05040005,
    SDO_ABORT_UNSUPPORTED       = 0x06010000,
    SDO_ABORT_READ_ONLY         = 0x06010001,
    SDO_ABORT_WRITE_ONLY        = 0x06010002,
    SDO_ABORT_OBJECT_NOT_EXIST  = 0x06020000,
    SDO_ABORT_PARAM_INVALID     = 0x06090030,
    SDO_ABORT_GENERAL           = 0x08000000
} sdo_abort_code_t;

// SDO Transfer State
typedef struct {
    uint16_t    index;
    uint8_t     subindex;
    uint8_t     data[CAN_FD_MAX_DLC];
    uint32_t    size;
    uint32_t    transferred;
    bool        toggle;
    bool        expedited;
    bool        active;
} sdo_transfer_t;

// =============================================================================
// PDO (Process Data Object) Types
// =============================================================================

// PDO Transmission Types
typedef enum {
    PDO_TRANS_SYNC_ACYCLIC      = 0,        // Sync, acyclic
    PDO_TRANS_SYNC_CYCLIC_1     = 1,        // Every SYNC
    PDO_TRANS_SYNC_CYCLIC_N     = 240,      // Every N SYNCs (1-240)
    PDO_TRANS_EVENT_MFR         = 254,      // Event-driven (manufacturer)
    PDO_TRANS_EVENT_PROFILE     = 255       // Event-driven (profile)
} pdo_transmission_type_t;

// PDO Mapping Entry
typedef struct {
    uint16_t    index;
    uint8_t     subindex;
    uint8_t     bit_length;
} pdo_mapping_t;

// PDO Configuration
typedef struct {
    uint32_t            cob_id;
    uint8_t             transmission_type;
    uint16_t            inhibit_time;       // x100µs
    uint16_t            event_timer;        // ms
    uint8_t             mapping_count;
    pdo_mapping_t       mapping[8];         // Up to 8 mapped objects
} pdo_config_t;

// =============================================================================
// SYNC Configuration
// =============================================================================

typedef struct {
    uint32_t    cob_id;                     // Usually 0x80
    uint32_t    period_us;                  // SYNC period in microseconds
    uint8_t     counter_overflow;           // 0 = no counter, 1-240 = overflow value
} sync_config_t;

// =============================================================================
// Emergency (EMCY) Message
// =============================================================================

typedef struct {
    uint16_t    error_code;
    uint8_t     error_register;
    uint8_t     mfr_specific[5];
} emcy_message_t;

// Standard Emergency Error Codes
typedef enum {
    EMCY_NO_ERROR               = 0x0000,
    EMCY_GENERIC                = 0x1000,
    EMCY_CURRENT                = 0x2000,
    EMCY_VOLTAGE                = 0x3000,
    EMCY_TEMPERATURE            = 0x4000,
    EMCY_DEVICE_HARDWARE        = 0x5000,
    EMCY_DEVICE_SOFTWARE        = 0x6000,
    EMCY_PROTOCOL_ERROR         = 0x8100,
    EMCY_CAN_OVERRUN            = 0x8110,
    EMCY_CAN_PASSIVE            = 0x8120,
    EMCY_CAN_BUSOFF             = 0x8140,
    EMCY_MOTION_ERROR           = 0xFF00,   // Vendor-specific
    EMCY_LIMIT_SWITCH           = 0xFF01,
    EMCY_FOLLOWING_ERROR        = 0xFF02
} emcy_error_code_t;

// =============================================================================
// Heartbeat / Error Control
// =============================================================================

typedef struct {
    uint8_t     node_id;
    nmt_state_t state;
    uint32_t    last_seen_ms;
    uint16_t    producer_time_ms;           // Expected heartbeat interval
    bool        active;
} heartbeat_entry_t;

// =============================================================================
// Object Dictionary Entry Types
// =============================================================================

typedef enum {
    OD_TYPE_BOOL        = 0x01,
    OD_TYPE_INT8        = 0x02,
    OD_TYPE_INT16       = 0x03,
    OD_TYPE_INT32       = 0x04,
    OD_TYPE_UINT8       = 0x05,
    OD_TYPE_UINT16      = 0x06,
    OD_TYPE_UINT32      = 0x07,
    OD_TYPE_REAL32      = 0x08,
    OD_TYPE_STRING      = 0x09,
    OD_TYPE_DOMAIN      = 0x0F
} od_data_type_t;

typedef enum {
    OD_ACCESS_RO        = 0x01,             // Read only
    OD_ACCESS_WO        = 0x02,             // Write only
    OD_ACCESS_RW        = 0x03,             // Read/write
    OD_ACCESS_CONST     = 0x04              // Constant
} od_access_t;

typedef struct {
    uint16_t        index;
    uint8_t         subindex;
    od_data_type_t  type;
    od_access_t     access;
    void*           data;
    uint32_t        size;
    const char*     name;
} od_entry_t;

// =============================================================================
// Utility Macros
// =============================================================================

// Build COB-ID from function code and node ID
#define CANOPEN_COB_ID(fc, node_id)     ((fc) + (node_id))

// Extract node ID from COB-ID
#define CANOPEN_NODE_FROM_COB(cob_id)   ((cob_id) & 0x7F)

// Check if COB-ID matches a function code range
#define CANOPEN_IS_FC(cob_id, fc)       (((cob_id) & 0x780) == (fc))

#ifdef __cplusplus
}
#endif

#endif // CANOPEN_TYPES_H
