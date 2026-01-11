/**
 * @file canopen_slave.c
 * @brief CANopen Slave Implementation
 */

#include "canopen_slave.h"
#include "main.h"
#include <string.h>

// External FDCAN handle
extern FDCAN_HandleTypeDef hfdcan1;

// Global slave context
canopen_slave_context_t g_canopen_slave = {0};

// Object Dictionary access callback
static bool od_access(uint16_t index, uint8_t subindex, bool is_write,
                      uint8_t* data, uint32_t* size, uint32_t* abort_code);

// Object Dictionary read/write for PDO
static bool od_read(uint16_t index, uint8_t subindex, void* data, uint32_t* size);
static bool od_write(uint16_t index, uint8_t subindex, const void* data, uint32_t size);

// =============================================================================
// CAN Transmit
// =============================================================================

void canopen_slave_transmit(const can_frame_t* frame)
{
    FDCAN_TxHeaderTypeDef tx_header = {0};
    
    tx_header.Identifier = frame->id;
    tx_header.IdType = frame->is_extended ? FDCAN_EXTENDED_ID : FDCAN_STANDARD_ID;
    tx_header.TxFrameType = frame->is_rtr ? FDCAN_REMOTE_FRAME : FDCAN_DATA_FRAME;
    tx_header.DataLength = frame->dlc << 16;  // Convert to DLC code
    tx_header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    tx_header.BitRateSwitch = frame->is_fd ? FDCAN_BRS_ON : FDCAN_BRS_OFF;
    tx_header.FDFormat = frame->is_fd ? FDCAN_FD_CAN : FDCAN_CLASSIC_CAN;
    tx_header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    tx_header.MessageMarker = 0;
    
    HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &tx_header, (uint8_t*)frame->data);
}

// =============================================================================
// Initialization
// =============================================================================

void canopen_slave_init(full_node_config_t* config)
{
    memset(&g_canopen_slave, 0, sizeof(g_canopen_slave));
    
    g_canopen_slave.config = config;
    g_canopen_slave.node_id = 0;  // Will be assigned
    
    // Initialize auto-assign slave
    node_config_t compact;
    node_config_to_compact(config, &compact);
    autoassign_slave_init(&g_canopen_slave.autoassign, &compact);
    
    // Use hardware RNG or ADC noise for random seed
    autoassign_slave_start(&g_canopen_slave.autoassign, 
                          HAL_GetTick() ^ *(uint32_t*)0x1FFF7590);
    
    // NMT and SDO will be initialized after node ID is assigned
}

void canopen_slave_set_drive(cia402_drive_t* drive)
{
    g_canopen_slave.drive = drive;
}

// =============================================================================
// Frame Processing
// =============================================================================

void canopen_slave_process_frame(const can_frame_t* frame)
{
    // Check for auto-assign frames first (before we have an ID)
    if (frame->id >= AUTOASSIGN_COB_DISCOVERY && frame->id <= AUTOASSIGN_COB_REPORT) {
        autoassign_slave_process(&g_canopen_slave.autoassign, frame, canopen_slave_transmit);
        
        // Check if we just got assigned
        if (autoassign_slave_is_assigned(&g_canopen_slave.autoassign) && 
            g_canopen_slave.node_id == 0) {
            
            g_canopen_slave.node_id = autoassign_slave_get_id(&g_canopen_slave.autoassign);
            
            // Initialize NMT slave
            nmt_slave_init(&g_canopen_slave.nmt, g_canopen_slave.node_id, 1000);
            
            // Initialize SDO server
            sdo_server_init(&g_canopen_slave.sdo, g_canopen_slave.node_id, od_access);
            
            // Initialize PDO manager
            pdo_manager_init(&g_canopen_slave.pdo, g_canopen_slave.node_id);
            pdo_manager_setup_cia402(&g_canopen_slave.pdo, true);  // Use CAN-FD
            
            // Send boot-up message
            nmt_slave_send_bootup(&g_canopen_slave.nmt, canopen_slave_transmit);
        }
        return;
    }
    
    // If we don't have an ID yet, ignore other frames
    if (g_canopen_slave.node_id == 0) return;
    
    // Route based on COB-ID
    uint8_t node_id_from_frame = CANOPEN_NODE_FROM_COB(frame->id);
    uint16_t fc = frame->id & 0x780;
    
    // NMT command (broadcast or to us)
    if (frame->id == 0) {
        nmt_command_t cmd;
        uint8_t target;
        if (nmt_parse_command(frame, &cmd, &target)) {
            if (target == 0 || target == g_canopen_slave.node_id) {
                nmt_slave_process_command(&g_canopen_slave.nmt, cmd, target);
            }
        }
        return;
    }
    
    // SYNC message
    if (fc == CANOPEN_FC_SYNC) {
        g_canopen_slave.sync_received = true;
        g_canopen_slave.sync_counter++;
        
        // Process SYNC in PDO manager
        pdo_manager_sync(&g_canopen_slave.pdo, frame->dlc > 0 ? frame->data[0] : 0);
        
        // Transmit TPDOs if operational
        if (nmt_slave_is_operational(&g_canopen_slave.nmt)) {
            pdo_manager_transmit_tpdos(&g_canopen_slave.pdo, od_read, canopen_slave_transmit);
        }
        return;
    }
    
    // Only process frames addressed to us
    if (node_id_from_frame != g_canopen_slave.node_id) return;
    
    switch (fc) {
        case CANOPEN_FC_SDO_RX:
            // SDO request
            sdo_server_process(&g_canopen_slave.sdo, frame, canopen_slave_transmit);
            break;
            
        case CANOPEN_FC_PDO1_RX:
        case CANOPEN_FC_PDO2_RX:
        case CANOPEN_FC_PDO3_RX:
        case CANOPEN_FC_PDO4_RX:
            // RPDO
            if (nmt_slave_is_operational(&g_canopen_slave.nmt)) {
                pdo_manager_process_rpdo(&g_canopen_slave.pdo, frame, od_write);
            }
            break;
            
        default:
            break;
    }
}

// =============================================================================
// Object Dictionary Access
// =============================================================================

static bool od_access(uint16_t index, uint8_t subindex, bool is_write,
                      uint8_t* data, uint32_t* size, uint32_t* abort_code)
{
    cia402_drive_t* drive = g_canopen_slave.drive;
    
    // Handle CiA 402 objects
    switch (index) {
        case OD_CONTROLWORD:
            if (is_write) {
                if (*size >= 2) {
                    drive->controlword = data[0] | (data[1] << 8);
                    cia402_process_controlword(drive, drive->controlword);
                    return true;
                }
            } else {
                *size = 2;
                data[0] = drive->controlword & 0xFF;
                data[1] = (drive->controlword >> 8) & 0xFF;
                return true;
            }
            break;
            
        case OD_STATUSWORD:
            if (!is_write) {
                drive->statusword = cia402_build_statusword(drive);
                *size = 2;
                data[0] = drive->statusword & 0xFF;
                data[1] = (drive->statusword >> 8) & 0xFF;
                return true;
            }
            break;
            
        case OD_MODES_OF_OPERATION:
            if (is_write) {
                if (*size >= 1) {
                    drive->mode_of_operation = (int8_t)data[0];
                    return true;
                }
            } else {
                *size = 1;
                data[0] = (uint8_t)drive->mode_of_operation;
                return true;
            }
            break;
            
        case OD_TARGET_POSITION:
            if (is_write) {
                if (*size >= 4) {
                    drive->position_target = data[0] | (data[1] << 8) |
                                             (data[2] << 16) | (data[3] << 24);
                    return true;
                }
            } else {
                *size = 4;
                memcpy(data, &drive->position_target, 4);
                return true;
            }
            break;
            
        case OD_POSITION_ACTUAL:
            if (!is_write) {
                *size = 4;
                memcpy(data, &drive->position_actual, 4);
                return true;
            }
            break;
            
        case OD_TARGET_VELOCITY:
            if (is_write) {
                if (*size >= 4) {
                    memcpy(&drive->velocity_target, data, 4);
                    return true;
                }
            } else {
                *size = 4;
                memcpy(data, &drive->velocity_target, 4);
                return true;
            }
            break;
            
        case OD_VELOCITY_ACTUAL:
            if (!is_write) {
                *size = 4;
                memcpy(data, &drive->velocity_actual, 4);
                return true;
            }
            break;
            
        case OD_DIGITAL_INPUTS:
            if (!is_write) {
                *size = 4;
                memcpy(data, &drive->digital_inputs, 4);
                return true;
            }
            break;
            
        case OD_DIGITAL_OUTPUTS:
            if (is_write) {
                if (*size >= 4) {
                    memcpy(&drive->digital_outputs, data, 4);
                    return true;
                }
            } else {
                *size = 4;
                memcpy(data, &drive->digital_outputs, 4);
                return true;
            }
            break;
            
        default:
            // Check vendor-specific config objects
            if (index >= 0x2000 && index <= 0x2FFF) {
                if (is_write) {
                    return node_config_sdo_write(g_canopen_slave.config, index, subindex, data, *size);
                } else {
                    return node_config_sdo_read(g_canopen_slave.config, index, subindex, data, size);
                }
            }
            break;
    }
    
    *abort_code = SDO_ABORT_OBJECT_NOT_EXIST;
    return false;
}

static bool od_read(uint16_t index, uint8_t subindex, void* data, uint32_t* size)
{
    uint32_t abort_code;
    return od_access(index, subindex, false, data, size, &abort_code);
}

static bool od_write(uint16_t index, uint8_t subindex, const void* data, uint32_t size)
{
    uint32_t abort_code;
    return od_access(index, subindex, true, (uint8_t*)data, &size, &abort_code);
}

// =============================================================================
// Tick Functions
// =============================================================================

void canopen_slave_tick(void)
{
    // Check auto-assign timeout
    if (!autoassign_slave_is_assigned(&g_canopen_slave.autoassign)) {
        // Still waiting for assignment
        return;
    }
    
    // Normal operation
    g_canopen_slave.sync_received = false;
}

void canopen_slave_1ms_tick(void)
{
    g_canopen_slave.tick_count++;
    
    // Auto-assign tick
    if (!autoassign_slave_is_assigned(&g_canopen_slave.autoassign)) {
        autoassign_slave_tick(&g_canopen_slave.autoassign, 1, canopen_slave_transmit);
        return;
    }
    
    // NMT heartbeat
    nmt_slave_tick(&g_canopen_slave.nmt, 1, canopen_slave_transmit);
    
    // SDO timeout
    sdo_server_tick(&g_canopen_slave.sdo, 1);
}

// =============================================================================
// Status
// =============================================================================

nmt_state_t canopen_slave_get_nmt_state(void)
{
    return nmt_slave_get_state(&g_canopen_slave.nmt);
}

uint8_t canopen_slave_get_node_id(void)
{
    return g_canopen_slave.node_id;
}

bool canopen_slave_is_operational(void)
{
    return nmt_slave_is_operational(&g_canopen_slave.nmt);
}
