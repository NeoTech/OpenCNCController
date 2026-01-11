/**
 * @file pdo.h
 * @brief CANopen Process Data Object (PDO) Protocol
 * 
 * PDOs provide real-time data exchange between CANopen nodes.
 * Used for cyclic position/velocity commands and feedback.
 * 
 * TPDO = Transmit PDO (data sent FROM this node)
 * RPDO = Receive PDO (data received BY this node)
 */

#ifndef PDO_H
#define PDO_H

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "canopen_types.h"
#include "cia402.h"

#ifdef __cplusplus
extern "C" {
#endif

// =============================================================================
// PDO Configuration
// =============================================================================

#define PDO_MAX_MAPPINGS        8           // Max OD entries per PDO
#define PDO_MAX_COUNT           4           // RPDO1-4, TPDO1-4

// CAN-FD allows 64-byte PDOs for high-density data
#define PDO_MAX_SIZE_CAN20      8
#define PDO_MAX_SIZE_CANFD      64

// =============================================================================
// PDO Transmission Types
// =============================================================================

typedef enum {
    PDO_TRANS_SYNC_ACYCLIC      = 0,        // Sync, acyclic
    PDO_TRANS_SYNC_1            = 1,        // Every SYNC
    PDO_TRANS_SYNC_N            = 240,      // Every N SYNCs (1-240)
    PDO_TRANS_RTR_SYNC          = 252,      // RTR with sync
    PDO_TRANS_RTR_ASYNC         = 253,      // RTR event
    PDO_TRANS_ASYNC_MFR         = 254,      // Manufacturer specific async
    PDO_TRANS_ASYNC_PROFILE     = 255       // Device profile async
} pdo_transmission_type_t;

// =============================================================================
// PDO Mapping Entry
// =============================================================================

typedef struct {
    uint16_t    index;                      // OD index
    uint8_t     subindex;                   // OD subindex
    uint8_t     bit_length;                 // Size in bits (8, 16, 32)
    uint8_t     bit_offset;                 // Offset in PDO data
} pdo_mapping_t;

// =============================================================================
// PDO Communication Parameters (0x1400-0x1403 for RPDO, 0x1800-0x1803 for TPDO)
// =============================================================================

typedef struct {
    uint32_t    cob_id;                     // COB-ID (bit 31 = valid)
    uint8_t     transmission_type;          // See pdo_transmission_type_t
    uint16_t    inhibit_time;               // Inhibit time in 100µs units
    uint16_t    event_timer;                // Event timer in ms
    uint8_t     sync_start;                 // SYNC counter start value
} pdo_comm_param_t;

// =============================================================================
// PDO Configuration
// =============================================================================

typedef struct {
    // Communication parameters
    pdo_comm_param_t    comm;
    
    // Mapping
    pdo_mapping_t       mappings[PDO_MAX_MAPPINGS];
    uint8_t             mapping_count;
    
    // Calculated
    uint8_t             data_length;        // Total bytes
    bool                enabled;
    bool                is_fd;              // Use CAN-FD framing
    
    // Runtime state
    uint8_t             sync_counter;       // For sync-triggered PDOs
    uint32_t            last_tx_tick;       // For inhibit timing
    bool                data_changed;       // For async on-change PDOs
    
} pdo_config_t;

// =============================================================================
// PDO Manager (per node)
// =============================================================================

typedef struct {
    uint8_t         node_id;
    
    // Receive PDOs (data we receive)
    pdo_config_t    rpdo[PDO_MAX_COUNT];
    
    // Transmit PDOs (data we send)
    pdo_config_t    tpdo[PDO_MAX_COUNT];
    
    // SYNC counter
    uint8_t         sync_counter;
    
} pdo_manager_t;

// =============================================================================
// Standard CiA 402 PDO Mappings for Motion Control
// =============================================================================

// RPDO1: Controlword + Target Position (master → slave)
static inline void pdo_setup_rpdo1_cia402(pdo_config_t* pdo, uint8_t node_id) {
    pdo->comm.cob_id = CANOPEN_COB_ID(CANOPEN_FC_PDO1_RX, node_id);
    pdo->comm.transmission_type = PDO_TRANS_SYNC_1;
    pdo->comm.inhibit_time = 0;
    pdo->comm.event_timer = 0;
    
    pdo->mappings[0] = (pdo_mapping_t){ OD_CONTROLWORD, 0, 16, 0 };
    pdo->mappings[1] = (pdo_mapping_t){ OD_TARGET_POSITION, 0, 32, 16 };
    pdo->mapping_count = 2;
    pdo->data_length = 6;
    pdo->enabled = true;
    pdo->is_fd = false;
}

// TPDO1: Statusword + Position Actual (slave → master)
static inline void pdo_setup_tpdo1_cia402(pdo_config_t* pdo, uint8_t node_id) {
    pdo->comm.cob_id = CANOPEN_COB_ID(CANOPEN_FC_PDO1_TX, node_id);
    pdo->comm.transmission_type = PDO_TRANS_SYNC_1;
    pdo->comm.inhibit_time = 0;
    pdo->comm.event_timer = 0;
    
    pdo->mappings[0] = (pdo_mapping_t){ OD_STATUSWORD, 0, 16, 0 };
    pdo->mappings[1] = (pdo_mapping_t){ OD_POSITION_ACTUAL, 0, 32, 16 };
    pdo->mapping_count = 2;
    pdo->data_length = 6;
    pdo->enabled = true;
    pdo->is_fd = false;
}

// RPDO2: Mode of Operation + Target Velocity
static inline void pdo_setup_rpdo2_cia402(pdo_config_t* pdo, uint8_t node_id) {
    pdo->comm.cob_id = CANOPEN_COB_ID(CANOPEN_FC_PDO2_RX, node_id);
    pdo->comm.transmission_type = PDO_TRANS_SYNC_1;
    
    pdo->mappings[0] = (pdo_mapping_t){ OD_MODES_OF_OPERATION, 0, 8, 0 };
    pdo->mappings[1] = (pdo_mapping_t){ OD_TARGET_VELOCITY, 0, 32, 8 };
    pdo->mapping_count = 2;
    pdo->data_length = 5;
    pdo->enabled = true;
    pdo->is_fd = false;
}

// TPDO2: Velocity Actual + Digital Inputs
static inline void pdo_setup_tpdo2_cia402(pdo_config_t* pdo, uint8_t node_id) {
    pdo->comm.cob_id = CANOPEN_COB_ID(CANOPEN_FC_PDO2_TX, node_id);
    pdo->comm.transmission_type = PDO_TRANS_SYNC_1;
    
    pdo->mappings[0] = (pdo_mapping_t){ OD_VELOCITY_ACTUAL, 0, 32, 0 };
    pdo->mappings[1] = (pdo_mapping_t){ OD_DIGITAL_INPUTS, 0, 32, 32 };
    pdo->mapping_count = 2;
    pdo->data_length = 8;
    pdo->enabled = true;
    pdo->is_fd = false;
}

// =============================================================================
// CAN-FD Extended PDO (all motion data in single frame)
// =============================================================================

// RPDO-FD: Full motion command in one 64-byte frame
static inline void pdo_setup_rpdo_fd_motion(pdo_config_t* pdo, uint8_t node_id) {
    pdo->comm.cob_id = CANOPEN_COB_ID(CANOPEN_FC_PDO1_RX, node_id);
    pdo->comm.transmission_type = PDO_TRANS_SYNC_1;
    
    // Byte 0-1: Controlword
    // Byte 2: Mode of operation
    // Byte 3: Reserved
    // Byte 4-7: Target position
    // Byte 8-11: Target velocity
    // Byte 12-15: Profile velocity
    // Byte 16-19: Profile acceleration
    // Byte 20-23: Profile deceleration
    // Byte 24-27: Digital outputs
    
    pdo->mappings[0] = (pdo_mapping_t){ OD_CONTROLWORD, 0, 16, 0 };
    pdo->mappings[1] = (pdo_mapping_t){ OD_MODES_OF_OPERATION, 0, 8, 16 };
    pdo->mappings[2] = (pdo_mapping_t){ OD_TARGET_POSITION, 0, 32, 32 };
    pdo->mappings[3] = (pdo_mapping_t){ OD_TARGET_VELOCITY, 0, 32, 64 };
    pdo->mappings[4] = (pdo_mapping_t){ OD_PROFILE_VELOCITY, 0, 32, 96 };
    pdo->mappings[5] = (pdo_mapping_t){ OD_PROFILE_ACCELERATION, 0, 32, 128 };
    pdo->mappings[6] = (pdo_mapping_t){ OD_PROFILE_DECELERATION, 0, 32, 160 };
    pdo->mappings[7] = (pdo_mapping_t){ OD_DIGITAL_OUTPUTS, 0, 32, 192 };
    pdo->mapping_count = 8;
    pdo->data_length = 28;
    pdo->enabled = true;
    pdo->is_fd = true;
}

// TPDO-FD: Full motion feedback in one frame
static inline void pdo_setup_tpdo_fd_motion(pdo_config_t* pdo, uint8_t node_id) {
    pdo->comm.cob_id = CANOPEN_COB_ID(CANOPEN_FC_PDO1_TX, node_id);
    pdo->comm.transmission_type = PDO_TRANS_SYNC_1;
    
    // Byte 0-1: Statusword
    // Byte 2: Mode display
    // Byte 3: Reserved
    // Byte 4-7: Position actual
    // Byte 8-11: Velocity actual
    // Byte 12-15: Digital inputs
    // Byte 16-17: Motor current (optional)
    // Byte 18-19: Temperature (optional)
    
    pdo->mappings[0] = (pdo_mapping_t){ OD_STATUSWORD, 0, 16, 0 };
    pdo->mappings[1] = (pdo_mapping_t){ OD_MODES_OF_OPERATION_DISP, 0, 8, 16 };
    pdo->mappings[2] = (pdo_mapping_t){ OD_POSITION_ACTUAL, 0, 32, 32 };
    pdo->mappings[3] = (pdo_mapping_t){ OD_VELOCITY_ACTUAL, 0, 32, 64 };
    pdo->mappings[4] = (pdo_mapping_t){ OD_DIGITAL_INPUTS, 0, 32, 96 };
    pdo->mapping_count = 5;
    pdo->data_length = 16;
    pdo->enabled = true;
    pdo->is_fd = true;
}

// =============================================================================
// PDO Frame Building
// =============================================================================

/**
 * @brief Build SYNC frame
 * @param counter SYNC counter value (0 = no counter)
 * @param frame Output CAN frame
 */
static inline void pdo_build_sync(uint8_t counter, can_frame_t* frame) {
    frame->id = CANOPEN_COB_ID(CANOPEN_FC_SYNC, 0);
    frame->is_fd = false;
    frame->is_extended = false;
    
    if (counter > 0) {
        frame->dlc = 1;
        frame->data[0] = counter;
    } else {
        frame->dlc = 0;
    }
}

/**
 * @brief Build PDO frame from mapping
 * @param pdo PDO configuration
 * @param od_read_cb Callback to read OD values
 * @param frame Output CAN frame
 */
typedef bool (*od_read_callback_t)(uint16_t index, uint8_t subindex, void* data, uint32_t* size);

static inline void pdo_build_tpdo(
    const pdo_config_t* pdo,
    od_read_callback_t od_read,
    can_frame_t* frame
) {
    frame->id = pdo->comm.cob_id & 0x7FF;  // Mask off valid bit
    frame->dlc = pdo->data_length;
    frame->is_fd = pdo->is_fd;
    frame->is_extended = false;
    
    memset(frame->data, 0, sizeof(frame->data));
    
    for (uint8_t i = 0; i < pdo->mapping_count; i++) {
        const pdo_mapping_t* map = &pdo->mappings[i];
        uint8_t byte_offset = map->bit_offset / 8;
        uint32_t value = 0;
        uint32_t size = map->bit_length / 8;
        
        if (od_read && od_read(map->index, map->subindex, &value, &size)) {
            memcpy(&frame->data[byte_offset], &value, size);
        }
    }
}

// =============================================================================
// PDO Frame Parsing
// =============================================================================

/**
 * @brief Process received PDO and update OD
 */
typedef bool (*od_write_callback_t)(uint16_t index, uint8_t subindex, const void* data, uint32_t size);

static inline void pdo_process_rpdo(
    const pdo_config_t* pdo,
    const can_frame_t* frame,
    od_write_callback_t od_write
) {
    if (!pdo->enabled) return;
    if ((frame->id & 0x7FF) != (pdo->comm.cob_id & 0x7FF)) return;
    
    for (uint8_t i = 0; i < pdo->mapping_count; i++) {
        const pdo_mapping_t* map = &pdo->mappings[i];
        uint8_t byte_offset = map->bit_offset / 8;
        uint32_t size = map->bit_length / 8;
        
        if (byte_offset + size <= frame->dlc && od_write) {
            od_write(map->index, map->subindex, &frame->data[byte_offset], size);
        }
    }
}

// =============================================================================
// PDO Manager Functions
// =============================================================================

/**
 * @brief Initialize PDO manager
 */
void pdo_manager_init(pdo_manager_t* mgr, uint8_t node_id);

/**
 * @brief Configure standard CiA 402 PDO set
 */
void pdo_manager_setup_cia402(pdo_manager_t* mgr, bool use_fd);

/**
 * @brief Process SYNC message
 * @param mgr PDO manager
 * @param sync_counter SYNC counter from frame (0 if none)
 */
void pdo_manager_sync(pdo_manager_t* mgr, uint8_t sync_counter);

/**
 * @brief Process received PDO
 */
void pdo_manager_process_rpdo(
    pdo_manager_t* mgr,
    const can_frame_t* frame,
    od_write_callback_t od_write
);

/**
 * @brief Transmit all pending TPDOs
 */
void pdo_manager_transmit_tpdos(
    pdo_manager_t* mgr,
    od_read_callback_t od_read,
    void (*tx_callback)(const can_frame_t* frame)
);

/**
 * @brief Enable/disable specific PDO
 */
void pdo_manager_enable(pdo_manager_t* mgr, bool is_tpdo, uint8_t pdo_num, bool enable);

#ifdef __cplusplus
}
#endif

#endif // PDO_H
