/**
 * @file canopen_slave.h
 * @brief CANopen Slave Implementation for Axis Nodes
 * 
 * Implements CANopen DS301 and CiA 402 for stepper/servo axis control.
 */

#ifndef CANOPEN_SLAVE_H
#define CANOPEN_SLAVE_H

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
// Slave Context
// =============================================================================

typedef struct {
    // Configuration
    full_node_config_t* config;
    uint8_t             node_id;        // Assigned node ID (0 = unassigned)
    
    // Protocol handlers
    nmt_slave_t         nmt;
    sdo_server_t        sdo;
    pdo_manager_t       pdo;
    autoassign_slave_t  autoassign;
    
    // CiA 402 drive (pointer to external)
    cia402_drive_t*     drive;
    
    // SYNC handling
    uint8_t             sync_counter;
    bool                sync_received;
    
    // Timing
    uint32_t            tick_count;
    
} canopen_slave_context_t;

// Global slave context
extern canopen_slave_context_t g_canopen_slave;

// =============================================================================
// Initialization
// =============================================================================

/**
 * @brief Initialize CANopen slave
 * @param config Node configuration
 */
void canopen_slave_init(full_node_config_t* config);

/**
 * @brief Set drive reference (for CiA 402)
 * @param drive CiA 402 drive structure
 */
void canopen_slave_set_drive(cia402_drive_t* drive);

// =============================================================================
// Frame Processing
// =============================================================================

/**
 * @brief Process received CAN frame
 * @param frame Received frame
 */
void canopen_slave_process_frame(const can_frame_t* frame);

// =============================================================================
// Tick Functions
// =============================================================================

/**
 * @brief Main tick (call from main loop)
 */
void canopen_slave_tick(void);

/**
 * @brief 1ms tick (call from timer interrupt)
 */
void canopen_slave_1ms_tick(void);

// =============================================================================
// Status
// =============================================================================

/**
 * @brief Get current NMT state
 */
nmt_state_t canopen_slave_get_nmt_state(void);

/**
 * @brief Get assigned node ID
 */
uint8_t canopen_slave_get_node_id(void);

/**
 * @brief Check if node is operational
 */
bool canopen_slave_is_operational(void);

// =============================================================================
// CAN Transmit (must be implemented by platform)
// =============================================================================

/**
 * @brief Transmit CAN frame (platform-specific)
 * @param frame Frame to transmit
 */
void canopen_slave_transmit(const can_frame_t* frame);

#ifdef __cplusplus
}
#endif

#endif // CANOPEN_SLAVE_H
