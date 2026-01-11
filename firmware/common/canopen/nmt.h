/**
 * @file nmt.h
 * @brief CANopen Network Management (NMT) Protocol
 * 
 * NMT is used to control the state of CANopen devices.
 * The NMT master can start, stop, and reset nodes.
 */

#ifndef NMT_H
#define NMT_H

#include <stdint.h>
#include <stdbool.h>
#include "canopen_types.h"

#ifdef __cplusplus
extern "C" {
#endif

// =============================================================================
// NMT Commands (sent by master)
// =============================================================================

typedef enum {
    NMT_CMD_START_REMOTE_NODE   = 0x01,     // Enter Operational
    NMT_CMD_STOP_REMOTE_NODE    = 0x02,     // Enter Stopped
    NMT_CMD_ENTER_PREOP         = 0x80,     // Enter Pre-operational
    NMT_CMD_RESET_NODE          = 0x81,     // Reset node (application reset)
    NMT_CMD_RESET_COMM          = 0x82      // Reset communication
} nmt_command_t;

// =============================================================================
// Heartbeat Producer (slave side)
// =============================================================================

typedef struct {
    uint8_t     node_id;
    nmt_state_t state;
    uint16_t    heartbeat_time_ms;          // 0 = disabled
    uint32_t    last_heartbeat_tick;
} nmt_heartbeat_producer_t;

// =============================================================================
// Heartbeat Consumer (master side)
// =============================================================================

typedef struct {
    uint8_t     node_id;
    nmt_state_t last_state;
    uint16_t    timeout_ms;
    uint32_t    last_received_tick;
    bool        timeout_occurred;
    bool        first_heartbeat_received;
} nmt_heartbeat_consumer_t;

// =============================================================================
// NMT Master Context
// =============================================================================

typedef struct {
    nmt_heartbeat_consumer_t consumers[CANOPEN_MAX_NODES];
    uint8_t                  node_count;
    uint32_t                 current_tick;
    
    // Callbacks
    void (*on_state_change)(uint8_t node_id, nmt_state_t old_state, nmt_state_t new_state);
    void (*on_heartbeat_timeout)(uint8_t node_id);
    void (*on_node_boot)(uint8_t node_id);
} nmt_master_t;

// =============================================================================
// NMT Slave Context
// =============================================================================

typedef struct {
    uint8_t                   node_id;
    nmt_state_t               state;
    nmt_heartbeat_producer_t  heartbeat;
    uint32_t                  current_tick;
    
    // Callbacks
    void (*on_nmt_command)(nmt_command_t cmd);
    void (*on_state_change)(nmt_state_t old_state, nmt_state_t new_state);
} nmt_slave_t;

// =============================================================================
// NMT Frame Building
// =============================================================================

/**
 * @brief Build NMT command frame
 * @param cmd NMT command
 * @param node_id Target node (0 = broadcast)
 * @param frame Output frame
 */
static inline void nmt_build_command(nmt_command_t cmd, uint8_t node_id, can_frame_t* frame) {
    frame->id = CANOPEN_COB_ID(CANOPEN_FC_NMT, 0);  // NMT uses COB-ID 0
    frame->dlc = 2;
    frame->is_fd = false;
    frame->is_extended = false;
    frame->data[0] = (uint8_t)cmd;
    frame->data[1] = node_id;
}

/**
 * @brief Build heartbeat frame
 * @param node_id Source node ID
 * @param state Current NMT state
 * @param frame Output frame
 */
static inline void nmt_build_heartbeat(uint8_t node_id, nmt_state_t state, can_frame_t* frame) {
    frame->id = CANOPEN_COB_ID(CANOPEN_FC_HEARTBEAT, node_id);
    frame->dlc = 1;
    frame->is_fd = false;
    frame->is_extended = false;
    frame->data[0] = (uint8_t)state;
}

/**
 * @brief Build boot-up message (heartbeat with state 0)
 * @param node_id Source node ID
 * @param frame Output frame
 */
static inline void nmt_build_bootup(uint8_t node_id, can_frame_t* frame) {
    frame->id = CANOPEN_COB_ID(CANOPEN_FC_HEARTBEAT, node_id);
    frame->dlc = 1;
    frame->is_fd = false;
    frame->is_extended = false;
    frame->data[0] = 0;  // Boot-up message
}

// =============================================================================
// NMT Frame Parsing
// =============================================================================

/**
 * @brief Parse NMT command frame
 * @param frame Input frame
 * @param cmd Output command
 * @param target_node Output target node ID
 * @return true if valid NMT command
 */
static inline bool nmt_parse_command(const can_frame_t* frame, nmt_command_t* cmd, uint8_t* target_node) {
    if (frame->id != 0 || frame->dlc < 2) {
        return false;
    }
    *cmd = (nmt_command_t)frame->data[0];
    *target_node = frame->data[1];
    return true;
}

/**
 * @brief Parse heartbeat frame
 * @param frame Input frame
 * @param node_id Output node ID
 * @param state Output NMT state
 * @return true if valid heartbeat
 */
static inline bool nmt_parse_heartbeat(const can_frame_t* frame, uint8_t* node_id, nmt_state_t* state) {
    if (!CANOPEN_IS_FC(frame->id, CANOPEN_FC_HEARTBEAT) || frame->dlc < 1) {
        return false;
    }
    *node_id = CANOPEN_NODE_FROM_COB(frame->id);
    *state = (nmt_state_t)frame->data[0];
    return true;
}

// =============================================================================
// NMT Master Functions
// =============================================================================

/**
 * @brief Initialize NMT master
 */
void nmt_master_init(nmt_master_t* master);

/**
 * @brief Add node to monitor
 * @param master NMT master context
 * @param node_id Node ID to monitor
 * @param timeout_ms Heartbeat timeout in milliseconds
 */
void nmt_master_add_node(nmt_master_t* master, uint8_t node_id, uint16_t timeout_ms);

/**
 * @brief Remove node from monitoring
 */
void nmt_master_remove_node(nmt_master_t* master, uint8_t node_id);

/**
 * @brief Process received heartbeat
 * @param master NMT master context
 * @param node_id Node that sent heartbeat
 * @param state Reported state
 */
void nmt_master_process_heartbeat(nmt_master_t* master, uint8_t node_id, nmt_state_t state);

/**
 * @brief Tick heartbeat timeout checking
 * @param master NMT master context
 * @param delta_ms Milliseconds since last tick
 */
void nmt_master_tick(nmt_master_t* master, uint32_t delta_ms);

/**
 * @brief Send NMT command to node
 * @param master NMT master context
 * @param cmd NMT command
 * @param node_id Target node (0 = broadcast)
 * @param tx_callback Function to transmit CAN frame
 */
void nmt_master_send_command(nmt_master_t* master, nmt_command_t cmd, uint8_t node_id,
                             void (*tx_callback)(const can_frame_t* frame));

/**
 * @brief Start all nodes (broadcast)
 */
static inline void nmt_master_start_all(nmt_master_t* master, 
                                        void (*tx_callback)(const can_frame_t* frame)) {
    nmt_master_send_command(master, NMT_CMD_START_REMOTE_NODE, 0, tx_callback);
}

/**
 * @brief Stop all nodes (broadcast)
 */
static inline void nmt_master_stop_all(nmt_master_t* master,
                                       void (*tx_callback)(const can_frame_t* frame)) {
    nmt_master_send_command(master, NMT_CMD_STOP_REMOTE_NODE, 0, tx_callback);
}

/**
 * @brief Get node state
 * @param master NMT master context
 * @param node_id Node to query
 * @return Last known state, or NMT_STATE_BOOT if unknown
 */
nmt_state_t nmt_master_get_node_state(const nmt_master_t* master, uint8_t node_id);

/**
 * @brief Check if all monitored nodes are operational
 */
bool nmt_master_all_operational(const nmt_master_t* master);

// =============================================================================
// NMT Slave Functions
// =============================================================================

/**
 * @brief Initialize NMT slave
 * @param slave NMT slave context
 * @param node_id This node's ID
 * @param heartbeat_ms Heartbeat interval (0 to disable)
 */
void nmt_slave_init(nmt_slave_t* slave, uint8_t node_id, uint16_t heartbeat_ms);

/**
 * @brief Process received NMT command
 * @param slave NMT slave context
 * @param cmd Received command
 * @param target_node Target node ID from frame
 */
void nmt_slave_process_command(nmt_slave_t* slave, nmt_command_t cmd, uint8_t target_node);

/**
 * @brief Tick heartbeat production
 * @param slave NMT slave context
 * @param delta_ms Milliseconds since last tick
 * @param tx_callback Function to transmit CAN frame
 */
void nmt_slave_tick(nmt_slave_t* slave, uint32_t delta_ms,
                    void (*tx_callback)(const can_frame_t* frame));

/**
 * @brief Send boot-up message
 * @param slave NMT slave context
 * @param tx_callback Function to transmit CAN frame
 */
void nmt_slave_send_bootup(nmt_slave_t* slave, void (*tx_callback)(const can_frame_t* frame));

/**
 * @brief Get current slave state
 */
static inline nmt_state_t nmt_slave_get_state(const nmt_slave_t* slave) {
    return slave->state;
}

/**
 * @brief Check if slave is operational
 */
static inline bool nmt_slave_is_operational(const nmt_slave_t* slave) {
    return slave->state == NMT_STATE_OPERATIONAL;
}

#ifdef __cplusplus
}
#endif

#endif // NMT_H
