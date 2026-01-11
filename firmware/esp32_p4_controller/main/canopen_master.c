/**
 * @file canopen_master.c
 * @brief CANopen Master Implementation
 */

#include "canopen_master.h"
#include <string.h>
#include "esp_log.h"
#include "driver/twai.h"

static const char *TAG = "canopen_master";

// Global master context
canopen_master_context_t g_canopen_master;

// =============================================================================
// CAN Transmit Helper
// =============================================================================

static void can_transmit(const can_frame_t* frame)
{
    twai_message_t tx_msg = {
        .identifier = frame->id,
        .data_length_code = frame->dlc,
        .extd = frame->is_extended,
        .rtr = frame->is_rtr
    };
    memcpy(tx_msg.data, frame->data, frame->dlc);
    
    esp_err_t err = twai_transmit(&tx_msg, pdMS_TO_TICKS(10));
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "CAN transmit failed: %s", esp_err_to_name(err));
    }
}

// =============================================================================
// Initialization
// =============================================================================

void canopen_master_init(void)
{
    ESP_LOGI(TAG, "Initializing CANopen master");
    
    memset(&g_canopen_master, 0, sizeof(g_canopen_master));
    
    // Initialize NMT master
    nmt_master_init(&g_canopen_master.nmt);
    g_canopen_master.nmt.on_state_change = NULL;  // TODO: Set callback
    g_canopen_master.nmt.on_heartbeat_timeout = NULL;
    
    // Initialize SDO clients
    for (int i = 0; i < CANOPEN_MASTER_NUM_BUSES; i++) {
        sdo_client_init(&g_canopen_master.sdo[i]);
    }
    
    // Initialize auto-assign master
    autoassign_master_init(&g_canopen_master.autoassign);
}

void canopen_master_start_discovery(uint32_t timeout_ms)
{
    ESP_LOGI(TAG, "Starting node discovery (timeout: %lu ms)", timeout_ms);
    
    g_canopen_master.discovery_complete = false;
    autoassign_master_start_discovery(&g_canopen_master.autoassign, timeout_ms);
}

bool canopen_master_is_discovery_complete(void)
{
    return g_canopen_master.discovery_complete || 
           autoassign_master_is_complete(&g_canopen_master.autoassign);
}

uint8_t canopen_master_get_axis_count(void)
{
    return g_canopen_master.axis_count;
}

uint8_t canopen_master_get_io_node_count(void)
{
    return g_canopen_master.io_node_count;
}

// =============================================================================
// Node Control
// =============================================================================

void canopen_master_configure_axis_pdo(uint8_t node_id)
{
    if (node_id == 0 || node_id > CANOPEN_MASTER_MAX_NODES) return;
    
    axis_node_state_t* node = &g_canopen_master.nodes[node_id - 1];
    
    ESP_LOGI(TAG, "Configuring PDO for node %d", node_id);
    
    // TODO: Configure via SDO
    // - Set RPDO1 mapping: Controlword + Target Position
    // - Set TPDO1 mapping: Statusword + Position Actual
    // - Set SYNC parameters
    
    node->configured = true;
}

void canopen_master_start_all_nodes(void)
{
    ESP_LOGI(TAG, "Starting all nodes");
    nmt_master_start_all(&g_canopen_master.nmt, can_transmit);
}

void canopen_master_stop_all_nodes(void)
{
    ESP_LOGI(TAG, "Stopping all nodes");
    nmt_master_stop_all(&g_canopen_master.nmt, can_transmit);
}

void canopen_master_reset_all_nodes(void)
{
    ESP_LOGI(TAG, "Resetting all nodes");
    nmt_master_send_command(&g_canopen_master.nmt, NMT_CMD_RESET_NODE, 0, can_transmit);
}

bool canopen_master_all_nodes_operational(void)
{
    return g_canopen_master.all_operational;
}

// =============================================================================
// Motion Control
// =============================================================================

void canopen_master_set_position(uint8_t node_id, int32_t position)
{
    if (node_id == 0 || node_id > CANOPEN_MASTER_MAX_NODES) return;
    g_canopen_master.nodes[node_id - 1].position_target = position;
}

void canopen_master_set_velocity(uint8_t node_id, int32_t velocity)
{
    if (node_id == 0 || node_id > CANOPEN_MASTER_MAX_NODES) return;
    g_canopen_master.nodes[node_id - 1].velocity_target = velocity;
}

void canopen_master_set_controlword(uint8_t node_id, uint16_t controlword)
{
    if (node_id == 0 || node_id > CANOPEN_MASTER_MAX_NODES) return;
    g_canopen_master.nodes[node_id - 1].controlword = controlword;
}

void canopen_master_set_mode(uint8_t node_id, int8_t mode)
{
    if (node_id == 0 || node_id > CANOPEN_MASTER_MAX_NODES) return;
    g_canopen_master.nodes[node_id - 1].mode_of_operation = mode;
}

void canopen_master_enable_axis(uint8_t node_id)
{
    if (node_id == 0 || node_id > CANOPEN_MASTER_MAX_NODES) return;
    
    axis_node_state_t* node = &g_canopen_master.nodes[node_id - 1];
    
    // State machine transitions: Disabled → Ready → Switched On → Enabled
    switch (node->cia402_state) {
        case CIA402_STATE_SWITCH_ON_DISABLED:
            node->controlword = CW_CMD_SHUTDOWN;
            break;
        case CIA402_STATE_READY_TO_SWITCH_ON:
            node->controlword = CW_CMD_SWITCH_ON;
            break;
        case CIA402_STATE_SWITCHED_ON:
            node->controlword = CW_CMD_ENABLE_OPERATION;
            break;
        case CIA402_STATE_FAULT:
            node->controlword = CW_CMD_FAULT_RESET;
            break;
        default:
            break;
    }
}

void canopen_master_disable_axis(uint8_t node_id)
{
    if (node_id == 0 || node_id > CANOPEN_MASTER_MAX_NODES) return;
    g_canopen_master.nodes[node_id - 1].controlword = CW_CMD_DISABLE_VOLTAGE;
}

void canopen_master_quick_stop(uint8_t node_id)
{
    if (node_id == 0 || node_id > CANOPEN_MASTER_MAX_NODES) return;
    g_canopen_master.nodes[node_id - 1].controlword = CW_CMD_QUICK_STOP;
}

void canopen_master_clear_fault(uint8_t node_id)
{
    if (node_id == 0 || node_id > CANOPEN_MASTER_MAX_NODES) return;
    
    axis_node_state_t* node = &g_canopen_master.nodes[node_id - 1];
    node->controlword = CW_CMD_FAULT_RESET;
    node->fault = false;
}

// =============================================================================
// Homing
// =============================================================================

void canopen_master_start_homing(uint8_t node_id, int8_t method)
{
    if (node_id == 0 || node_id > CANOPEN_MASTER_MAX_NODES) return;
    
    axis_node_state_t* node = &g_canopen_master.nodes[node_id - 1];
    
    // Set homing mode
    node->mode_of_operation = CIA402_MODE_HOMING;
    node->homing_in_progress = true;
    node->homing_complete = false;
    
    // Start homing via controlword
    node->controlword = CW_ENABLE_OPERATION | CW_NEW_SETPOINT;
    
    ESP_LOGI(TAG, "Started homing for node %d, method %d", node_id, method);
}

bool canopen_master_is_homing_complete(uint8_t node_id)
{
    if (node_id == 0 || node_id > CANOPEN_MASTER_MAX_NODES) return false;
    return g_canopen_master.nodes[node_id - 1].homing_complete;
}

void canopen_master_home_all_axes(void)
{
    for (uint8_t i = 0; i < g_canopen_master.axis_count; i++) {
        canopen_master_start_homing(i + 1, HOMING_SWITCH_NEGATIVE);
    }
}

// =============================================================================
// Status Query
// =============================================================================

int32_t canopen_master_get_position(uint8_t node_id)
{
    if (node_id == 0 || node_id > CANOPEN_MASTER_MAX_NODES) return 0;
    return g_canopen_master.nodes[node_id - 1].position_actual;
}

int32_t canopen_master_get_velocity(uint8_t node_id)
{
    if (node_id == 0 || node_id > CANOPEN_MASTER_MAX_NODES) return 0;
    return g_canopen_master.nodes[node_id - 1].velocity_actual;
}

uint16_t canopen_master_get_statusword(uint8_t node_id)
{
    if (node_id == 0 || node_id > CANOPEN_MASTER_MAX_NODES) return 0;
    return g_canopen_master.nodes[node_id - 1].statusword;
}

cia402_state_t canopen_master_get_axis_state(uint8_t node_id)
{
    if (node_id == 0 || node_id > CANOPEN_MASTER_MAX_NODES) {
        return CIA402_STATE_NOT_READY;
    }
    return g_canopen_master.nodes[node_id - 1].cia402_state;
}

const node_config_t* canopen_master_get_node_config(uint8_t node_id)
{
    if (node_id == 0 || node_id > CANOPEN_MASTER_MAX_NODES) return NULL;
    return &g_canopen_master.nodes[node_id - 1].config;
}

uint32_t canopen_master_get_digital_inputs(uint8_t node_id)
{
    if (node_id == 0 || node_id > CANOPEN_MASTER_MAX_NODES) return 0;
    return g_canopen_master.nodes[node_id - 1].digital_inputs;
}

void canopen_master_set_digital_outputs(uint8_t node_id, uint32_t outputs)
{
    if (node_id == 0 || node_id > CANOPEN_MASTER_MAX_NODES) return;
    g_canopen_master.nodes[node_id - 1].digital_outputs = outputs;
}

// =============================================================================
// SYNC and PDO
// =============================================================================

void canopen_master_send_sync(void)
{
    g_canopen_master.sync_counter++;
    
    can_frame_t sync_frame;
    pdo_build_sync(g_canopen_master.sync_counter, &sync_frame);
    
    can_transmit(&sync_frame);
}

void canopen_master_transmit_rpdos(void)
{
    // Build and transmit RPDO for each operational axis
    for (uint8_t i = 0; i < g_canopen_master.axis_count; i++) {
        axis_node_state_t* node = &g_canopen_master.nodes[i];
        
        if (node->nmt_state != NMT_STATE_OPERATIONAL) continue;
        
        // Build RPDO1: Controlword + Target Position
        can_frame_t rpdo;
        rpdo.id = CANOPEN_COB_ID(CANOPEN_FC_PDO1_RX, i + 1);
        rpdo.dlc = 6;
        rpdo.is_fd = false;
        rpdo.is_extended = false;
        rpdo.is_rtr = false;
        
        // Pack data
        rpdo.data[0] = node->controlword & 0xFF;
        rpdo.data[1] = (node->controlword >> 8) & 0xFF;
        rpdo.data[2] = node->position_target & 0xFF;
        rpdo.data[3] = (node->position_target >> 8) & 0xFF;
        rpdo.data[4] = (node->position_target >> 16) & 0xFF;
        rpdo.data[5] = (node->position_target >> 24) & 0xFF;
        
        can_transmit(&rpdo);
    }
}

uint8_t canopen_master_get_sync_counter(void)
{
    return g_canopen_master.sync_counter;
}

// =============================================================================
// Message Processing
// =============================================================================

void canopen_master_process_heartbeat(const can_frame_t* frame)
{
    uint8_t node_id;
    nmt_state_t state;
    
    if (!nmt_parse_heartbeat(frame, &node_id, &state)) return;
    if (node_id == 0 || node_id > CANOPEN_MASTER_MAX_NODES) return;
    
    axis_node_state_t* node = &g_canopen_master.nodes[node_id - 1];
    node->nmt_state = state;
    node->last_heartbeat_tick = xTaskGetTickCount();
    node->heartbeat_timeout = false;
    
    // Update NMT master tracking
    nmt_master_process_heartbeat(&g_canopen_master.nmt, node_id, state);
}

void canopen_master_process_tpdo(const can_frame_t* frame)
{
    uint8_t node_id = CANOPEN_NODE_FROM_COB(frame->id);
    if (node_id == 0 || node_id > CANOPEN_MASTER_MAX_NODES) return;
    
    axis_node_state_t* node = &g_canopen_master.nodes[node_id - 1];
    uint16_t fc = frame->id & 0x780;
    
    if (fc == CANOPEN_FC_PDO1_TX && frame->dlc >= 6) {
        // TPDO1: Statusword + Position Actual
        node->statusword = frame->data[0] | (frame->data[1] << 8);
        node->position_actual = frame->data[2] |
                                (frame->data[3] << 8) |
                                (frame->data[4] << 16) |
                                (frame->data[5] << 24);
        
        // Update CiA 402 state from statusword
        node->cia402_state = cia402_get_state(node->statusword);
        
        // Check homing status
        if (node->homing_in_progress) {
            if (node->statusword & SW_TARGET_REACHED) {
                node->homing_complete = true;
                node->homing_in_progress = false;
                ESP_LOGI(TAG, "Node %d homing complete", node_id);
            }
        }
        
        // Invoke callback
        if (g_canopen_master.on_position_update) {
            g_canopen_master.on_position_update(node_id, node->position_actual);
        }
    }
    else if (fc == CANOPEN_FC_PDO2_TX && frame->dlc >= 8) {
        // TPDO2: Velocity Actual + Digital Inputs
        node->velocity_actual = frame->data[0] |
                                (frame->data[1] << 8) |
                                (frame->data[2] << 16) |
                                (frame->data[3] << 24);
        node->digital_inputs = frame->data[4] |
                               (frame->data[5] << 8) |
                               (frame->data[6] << 16) |
                               (frame->data[7] << 24);
    }
}

void canopen_master_process_sdo(const can_frame_t* frame)
{
    uint8_t node_id = CANOPEN_NODE_FROM_COB(frame->id);
    if (node_id == 0 || node_id > CANOPEN_MASTER_MAX_NODES) return;
    
    // Determine which SDO client based on node's bus
    axis_node_state_t* node = &g_canopen_master.nodes[node_id - 1];
    uint8_t bus_id = node->bus_id;
    if (bus_id >= CANOPEN_MASTER_NUM_BUSES) bus_id = 0;
    
    sdo_client_process(&g_canopen_master.sdo[bus_id], frame, can_transmit);
}

void canopen_master_process_emcy(const can_frame_t* frame)
{
    uint8_t node_id = CANOPEN_NODE_FROM_COB(frame->id);
    if (node_id == 0 || node_id > CANOPEN_MASTER_MAX_NODES) return;
    if (frame->dlc < 8) return;
    
    axis_node_state_t* node = &g_canopen_master.nodes[node_id - 1];
    
    // Parse emergency code
    uint16_t error_code = frame->data[0] | (frame->data[1] << 8);
    uint8_t error_register = frame->data[2];
    
    node->fault = (error_code != 0);
    node->last_emcy_code = error_code;
    
    ESP_LOGW(TAG, "EMCY from node %d: error=0x%04X, reg=0x%02X",
             node_id, error_code, error_register);
    
    if (g_canopen_master.on_fault) {
        g_canopen_master.on_fault(node_id, error_code);
    }
    
    // Update system fault flag
    g_canopen_master.any_fault = false;
    for (uint8_t i = 0; i < g_canopen_master.node_count; i++) {
        if (g_canopen_master.nodes[i].fault) {
            g_canopen_master.any_fault = true;
            break;
        }
    }
}

void canopen_master_process_autoassign(const can_frame_t* frame)
{
    autoassign_master_process(&g_canopen_master.autoassign, frame, can_transmit);
    
    // Check if discovery just completed
    if (autoassign_master_is_complete(&g_canopen_master.autoassign) && 
        !g_canopen_master.discovery_complete) {
        
        g_canopen_master.discovery_complete = true;
        g_canopen_master.node_count = g_canopen_master.autoassign.node_count;
        
        // Copy discovered nodes and count types
        g_canopen_master.axis_count = 0;
        g_canopen_master.io_node_count = 0;
        
        for (uint8_t i = 0; i < g_canopen_master.node_count; i++) {
            const node_config_t* discovered = autoassign_master_get_node(&g_canopen_master.autoassign, i);
            if (discovered) {
                uint8_t node_id = discovered->node_id;
                if (node_id > 0 && node_id <= CANOPEN_MASTER_MAX_NODES) {
                    g_canopen_master.nodes[node_id - 1].config = *discovered;
                    g_canopen_master.nodes[node_id - 1].bus_id = discovered->bus_id;
                    
                    // Count by type
                    if (discovered->type == NODE_TYPE_STEPPER_AXIS ||
                        discovered->type == NODE_TYPE_SERVO_AXIS) {
                        g_canopen_master.axis_count++;
                    } else if (discovered->type == NODE_TYPE_IO_EXPANDER) {
                        g_canopen_master.io_node_count++;
                    }
                    
                    // Add to NMT monitoring
                    nmt_master_add_node(&g_canopen_master.nmt, node_id, 1000);
                }
            }
        }
        
        ESP_LOGI(TAG, "Discovery complete: %d nodes (%d axes, %d I/O)",
                 g_canopen_master.node_count,
                 g_canopen_master.axis_count,
                 g_canopen_master.io_node_count);
    }
}

// =============================================================================
// Tick Functions
// =============================================================================

void canopen_master_tick(void)
{
    // Send SYNC
    canopen_master_send_sync();
    
    // Transmit RPDOs to all axes
    canopen_master_transmit_rpdos();
}

void canopen_master_slow_tick(void)
{
    // Check heartbeat timeouts
    nmt_master_tick(&g_canopen_master.nmt, 100);  // 100ms per slow tick
    
    // Update all_operational flag
    g_canopen_master.all_operational = nmt_master_all_operational(&g_canopen_master.nmt);
    
    // Check for auto-assign timeout
    if (!g_canopen_master.discovery_complete) {
        autoassign_master_tick(&g_canopen_master.autoassign, 100, can_transmit);
    }
    
    // Process SDO timeouts
    for (int i = 0; i < CANOPEN_MASTER_NUM_BUSES; i++) {
        sdo_client_tick(&g_canopen_master.sdo[i], 100);
    }
}
