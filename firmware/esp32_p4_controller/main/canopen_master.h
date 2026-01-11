/**
 * @file canopen_master.h
 * @brief CANopen Master for ESP32-P4 Controller
 * 
 * Manages all CANopen communication with axis nodes.
 * Handles NMT, SDO, PDO, and auto-assign protocols.
 */

#ifndef CANOPEN_MASTER_H
#define CANOPEN_MASTER_H

#include <stdint.h>
#include <stdbool.h>
#include "canopen_types.h"
#include "cia402.h"
#include "nmt.h"
#include "sdo.h"
#include "pdo.h"
#include "auto_assign.h"
#include "node_config.h"

#ifdef __cplusplus
extern "C" {
#endif

// =============================================================================
// Configuration
// =============================================================================

#define CANOPEN_MASTER_MAX_NODES    CANOPEN_MAX_NODES
#define CANOPEN_MASTER_NUM_BUSES    3

// =============================================================================
// Axis Node State
// =============================================================================

typedef struct {
    // Configuration
    node_config_t       config;
    full_node_config_t  full_config;
    bool                configured;
    
    // CiA 402 state
    cia402_state_t      cia402_state;
    uint16_t            controlword;
    uint16_t            statusword;
    int8_t              mode_of_operation;
    
    // Position/velocity
    int32_t             position_actual;
    int32_t             position_target;
    int32_t             velocity_actual;
    int32_t             velocity_target;
    
    // Homing state
    bool                homing_complete;
    bool                homing_in_progress;
    
    // Digital I/O
    uint32_t            digital_inputs;
    uint32_t            digital_outputs;
    
    // Error tracking
    bool                fault;
    uint32_t            last_emcy_code;
    
    // Communication
    uint8_t             bus_id;             // Which CAN bus (0, 1, 2)
    nmt_state_t         nmt_state;
    uint32_t            last_heartbeat_tick;
    bool                heartbeat_timeout;
    
} axis_node_state_t;

// =============================================================================
// Master Context
// =============================================================================

typedef struct {
    // Node management
    axis_node_state_t   nodes[CANOPEN_MASTER_MAX_NODES];
    uint8_t             node_count;
    uint8_t             axis_count;
    uint8_t             io_node_count;
    
    // Auto-assign
    autoassign_master_t autoassign;
    bool                discovery_complete;
    
    // NMT
    nmt_master_t        nmt;
    
    // SDO client (one per bus for parallel access)
    sdo_client_t        sdo[CANOPEN_MASTER_NUM_BUSES];
    
    // PDO management
    uint8_t             sync_counter;
    uint32_t            sync_tick;
    
    // System state
    bool                all_operational;
    bool                any_fault;
    
    // Callbacks
    void (*on_node_state_change)(uint8_t node_id, nmt_state_t old_state, nmt_state_t new_state);
    void (*on_position_update)(uint8_t node_id, int32_t position);
    void (*on_fault)(uint8_t node_id, uint32_t emcy_code);
    
} canopen_master_context_t;

// Global master context
extern canopen_master_context_t g_canopen_master;

// =============================================================================
// Initialization
// =============================================================================

/**
 * @brief Initialize CANopen master
 */
void canopen_master_init(void);

/**
 * @brief Start node discovery process
 * @param timeout_ms Discovery timeout in milliseconds
 */
void canopen_master_start_discovery(uint32_t timeout_ms);

/**
 * @brief Check if discovery is complete
 */
bool canopen_master_is_discovery_complete(void);

/**
 * @brief Get number of discovered axis nodes
 */
uint8_t canopen_master_get_axis_count(void);

/**
 * @brief Get number of discovered I/O nodes
 */
uint8_t canopen_master_get_io_node_count(void);

// =============================================================================
// Node Control
// =============================================================================

/**
 * @brief Configure PDO mappings for an axis node
 * @param node_id Node ID to configure
 */
void canopen_master_configure_axis_pdo(uint8_t node_id);

/**
 * @brief Start all nodes (NMT start)
 */
void canopen_master_start_all_nodes(void);

/**
 * @brief Stop all nodes (NMT stop)
 */
void canopen_master_stop_all_nodes(void);

/**
 * @brief Reset all nodes (NMT reset)
 */
void canopen_master_reset_all_nodes(void);

/**
 * @brief Check if all nodes are operational
 */
bool canopen_master_all_nodes_operational(void);

// =============================================================================
// Motion Control
// =============================================================================

/**
 * @brief Set target position for an axis
 * @param node_id Target node
 * @param position Target position in steps
 */
void canopen_master_set_position(uint8_t node_id, int32_t position);

/**
 * @brief Set target velocity for an axis
 * @param node_id Target node
 * @param velocity Target velocity in steps/sec
 */
void canopen_master_set_velocity(uint8_t node_id, int32_t velocity);

/**
 * @brief Set controlword for an axis
 * @param node_id Target node
 * @param controlword CiA 402 controlword
 */
void canopen_master_set_controlword(uint8_t node_id, uint16_t controlword);

/**
 * @brief Set mode of operation for an axis
 * @param node_id Target node
 * @param mode CiA 402 mode (CSP, CSV, PP, etc.)
 */
void canopen_master_set_mode(uint8_t node_id, int8_t mode);

/**
 * @brief Enable an axis (transition to Operation Enabled)
 * @param node_id Target node
 */
void canopen_master_enable_axis(uint8_t node_id);

/**
 * @brief Disable an axis (transition to Switch On Disabled)
 * @param node_id Target node
 */
void canopen_master_disable_axis(uint8_t node_id);

/**
 * @brief Quick stop an axis
 * @param node_id Target node
 */
void canopen_master_quick_stop(uint8_t node_id);

/**
 * @brief Clear fault on an axis
 * @param node_id Target node
 */
void canopen_master_clear_fault(uint8_t node_id);

// =============================================================================
// Homing
// =============================================================================

/**
 * @brief Start homing sequence for an axis
 * @param node_id Target node
 * @param method Homing method (CiA 402)
 */
void canopen_master_start_homing(uint8_t node_id, int8_t method);

/**
 * @brief Check if axis homing is complete
 * @param node_id Target node
 */
bool canopen_master_is_homing_complete(uint8_t node_id);

/**
 * @brief Start homing for all axes
 */
void canopen_master_home_all_axes(void);

// =============================================================================
// Status Query
// =============================================================================

/**
 * @brief Get current position of an axis
 * @param node_id Target node
 * @return Current position in steps
 */
int32_t canopen_master_get_position(uint8_t node_id);

/**
 * @brief Get current velocity of an axis
 * @param node_id Target node
 * @return Current velocity in steps/sec
 */
int32_t canopen_master_get_velocity(uint8_t node_id);

/**
 * @brief Get statusword of an axis
 * @param node_id Target node
 * @return CiA 402 statusword
 */
uint16_t canopen_master_get_statusword(uint8_t node_id);

/**
 * @brief Get CiA 402 state of an axis
 * @param node_id Target node
 * @return CiA 402 state
 */
cia402_state_t canopen_master_get_axis_state(uint8_t node_id);

/**
 * @brief Get node configuration
 * @param node_id Target node
 * @return Pointer to node config (NULL if not found)
 */
const node_config_t* canopen_master_get_node_config(uint8_t node_id);

/**
 * @brief Get digital inputs from a node
 * @param node_id Target node
 */
uint32_t canopen_master_get_digital_inputs(uint8_t node_id);

/**
 * @brief Set digital outputs on a node
 * @param node_id Target node
 * @param outputs Output bitfield
 */
void canopen_master_set_digital_outputs(uint8_t node_id, uint32_t outputs);

// =============================================================================
// SYNC and PDO
// =============================================================================

/**
 * @brief Send SYNC message on all buses
 */
void canopen_master_send_sync(void);

/**
 * @brief Transmit all pending RPDOs
 */
void canopen_master_transmit_rpdos(void);

/**
 * @brief Get SYNC counter
 */
uint8_t canopen_master_get_sync_counter(void);

// =============================================================================
// Message Processing
// =============================================================================

/**
 * @brief Process received heartbeat message
 * @param frame Received CAN frame
 */
void canopen_master_process_heartbeat(const can_frame_t* frame);

/**
 * @brief Process received TPDO message
 * @param frame Received CAN frame
 */
void canopen_master_process_tpdo(const can_frame_t* frame);

/**
 * @brief Process received SDO response
 * @param frame Received CAN frame
 */
void canopen_master_process_sdo(const can_frame_t* frame);

/**
 * @brief Process received emergency message
 * @param frame Received CAN frame
 */
void canopen_master_process_emcy(const can_frame_t* frame);

/**
 * @brief Process auto-assign protocol message
 * @param frame Received CAN frame
 */
void canopen_master_process_autoassign(const can_frame_t* frame);

// =============================================================================
// Tick Functions
// =============================================================================

/**
 * @brief Master tick - call at SYNC rate
 */
void canopen_master_tick(void);

/**
 * @brief Slow tick - call at 10 Hz for timeouts
 */
void canopen_master_slow_tick(void);

#ifdef __cplusplus
}
#endif

#endif // CANOPEN_MASTER_H
