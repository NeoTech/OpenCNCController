/**
 * @file main.c
 * @brief OpenCNC ESP32-P4 Central Controller
 * 
 * Main entry point for the ESP32-P4 based CNC controller.
 * Manages:
 * - 3x CAN-FD buses for distributed axis control
 * - Ethernet connection to Windows HMI
 * - MIPI-DSI local display
 * - Standalone jog/homing mode
 * 
 * Architecture:
 * - Core 0: Ethernet, Display, User Interface
 * - Core 1: Real-time motion control, CAN communication
 */

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "esp_system.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "nvs_flash.h"
#include "driver/twai.h"

// Project headers
#include "canopen_master.h"
#include "eth_comm.h"
#include "trajectory_task.h"
#include "display_task.h"
#include "standalone_mode.h"

// CANopen shared headers
#include "canopen_types.h"
#include "cia402.h"
#include "nmt.h"
#include "sdo.h"
#include "pdo.h"
#include "auto_assign.h"
#include "node_config.h"

static const char *TAG = "opencnc_main";

// =============================================================================
// Hardware Pin Definitions
// =============================================================================

// CAN Bus 0: Primary axes (X, Y, Z)
#define CAN_BUS0_TX_PIN     CONFIG_OPENCNC_CAN_BUS0_TX_PIN
#define CAN_BUS0_RX_PIN     CONFIG_OPENCNC_CAN_BUS0_RX_PIN

// CAN Bus 1: Secondary axes (A, B, C)
#define CAN_BUS1_TX_PIN     CONFIG_OPENCNC_CAN_BUS1_TX_PIN
#define CAN_BUS1_RX_PIN     CONFIG_OPENCNC_CAN_BUS1_RX_PIN

// CAN Bus 2: Auxiliary (spindle, I/O, probes)
#define CAN_BUS2_TX_PIN     CONFIG_OPENCNC_CAN_BUS2_TX_PIN
#define CAN_BUS2_RX_PIN     CONFIG_OPENCNC_CAN_BUS2_RX_PIN

// =============================================================================
// System Configuration
// =============================================================================

#define CAN_BITRATE_ARB     CONFIG_OPENCNC_CAN_BITRATE_ARB
#define CAN_BITRATE_DATA    CONFIG_OPENCNC_CAN_BITRATE_DATA
#define SYNC_RATE_HZ        CONFIG_OPENCNC_SYNC_RATE_HZ
#define MAX_AXES            CONFIG_OPENCNC_MAX_AXES

// =============================================================================
// Global State
// =============================================================================

typedef enum {
    SYSTEM_STATE_INIT,              // Initializing
    SYSTEM_STATE_DISCOVERY,         // Auto-assigning nodes
    SYSTEM_STATE_CONFIGURING,       // Configuring nodes via SDO
    SYSTEM_STATE_IDLE,              // Ready, waiting for commands
    SYSTEM_STATE_RUNNING,           // Motion in progress
    SYSTEM_STATE_PAUSED,            // Motion paused
    SYSTEM_STATE_HOMING,            // Homing sequence
    SYSTEM_STATE_JOG,               // Jog mode
    SYSTEM_STATE_FAULT,             // Fault condition
    SYSTEM_STATE_ESTOP              // Emergency stop active
} system_state_t;

typedef struct {
    system_state_t  state;
    bool            hmi_connected;
    bool            standalone_mode;
    uint8_t         axis_count;
    uint8_t         io_node_count;
    uint32_t        uptime_seconds;
    
    // Motion state
    int32_t         position[MAX_AXES];     // Current position per axis
    int32_t         target[MAX_AXES];       // Target position per axis
    uint16_t        statusword[MAX_AXES];   // CiA402 statusword per axis
    
    // Error tracking
    uint32_t        error_code;
    char            error_message[64];
    
} system_context_t;

static system_context_t g_system = {
    .state = SYSTEM_STATE_INIT,
    .hmi_connected = false,
    .standalone_mode = false
};

// =============================================================================
// FreeRTOS Handles
// =============================================================================

static TaskHandle_t task_motion = NULL;
static TaskHandle_t task_can_rx = NULL;
static TaskHandle_t task_ethernet = NULL;
static TaskHandle_t task_display = NULL;
static TaskHandle_t task_ui = NULL;

static SemaphoreHandle_t mutex_state = NULL;
static QueueHandle_t queue_can_rx = NULL;
static QueueHandle_t queue_hmi_cmd = NULL;

// =============================================================================
// CAN Bus Initialization
// =============================================================================

static esp_err_t init_can_bus(int bus_num, int tx_pin, int rx_pin)
{
    ESP_LOGI(TAG, "Initializing CAN bus %d (TX=%d, RX=%d)", bus_num, tx_pin, rx_pin);
    
    // CAN-FD timing configuration
    // Arbitration phase: 500 kbps
    // Data phase: 2 Mbps
    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
    
    // General configuration
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(tx_pin, rx_pin, TWAI_MODE_NORMAL);
    g_config.rx_queue_len = 32;
    g_config.tx_queue_len = 16;
    
    // Filter configuration - accept all messages
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();
    
    // Install driver
    // Note: ESP32-P4 has 3 TWAI controllers, need to specify which one
    esp_err_t err = twai_driver_install(&g_config, &t_config, &f_config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to install TWAI driver for bus %d: %s", bus_num, esp_err_to_name(err));
        return err;
    }
    
    // Start driver
    err = twai_start();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start TWAI driver for bus %d: %s", bus_num, esp_err_to_name(err));
        return err;
    }
    
    ESP_LOGI(TAG, "CAN bus %d initialized successfully", bus_num);
    return ESP_OK;
}

// =============================================================================
// Motion Control Task (Core 1, High Priority)
// =============================================================================

static void motion_task(void *pvParameters)
{
    ESP_LOGI(TAG, "Motion control task started on core %d", xPortGetCoreID());
    
    TickType_t last_wake = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(1000 / SYNC_RATE_HZ);
    
    // Initialize trajectory planner
    trajectory_task_init();
    
    while (1) {
        // Wait for next cycle
        vTaskDelayUntil(&last_wake, period);
        
        // Only run motion when in appropriate state
        if (g_system.state == SYSTEM_STATE_RUNNING ||
            g_system.state == SYSTEM_STATE_HOMING ||
            g_system.state == SYSTEM_STATE_JOG) {
            
            // Generate SYNC message
            can_frame_t sync_frame;
            pdo_build_sync(0, &sync_frame);
            
            // Transmit SYNC on all buses
            // TODO: Implement multi-bus SYNC transmission
            
            // Get next trajectory points
            trajectory_task_tick();
            
            // Update target positions via PDO
            // TODO: Implement PDO transmission
        }
        
        // Always process heartbeat checking
        // TODO: Implement heartbeat monitoring
    }
}

// =============================================================================
// CAN RX Task (Core 1)
// =============================================================================

static void can_rx_task(void *pvParameters)
{
    ESP_LOGI(TAG, "CAN RX task started on core %d", xPortGetCoreID());
    
    twai_message_t rx_message;
    
    while (1) {
        // Wait for message with timeout
        esp_err_t err = twai_receive(&rx_message, pdMS_TO_TICKS(100));
        if (err != ESP_OK) {
            continue;
        }
        
        // Convert to our frame format
        can_frame_t frame = {
            .id = rx_message.identifier,
            .dlc = rx_message.data_length_code,
            .is_extended = rx_message.extd,
            .is_fd = false,  // ESP32-P4 TWAI supports FD
            .is_rtr = rx_message.rtr
        };
        memcpy(frame.data, rx_message.data, rx_message.data_length_code);
        
        // Route based on COB-ID
        uint16_t fc = frame.id & 0x780;  // Function code
        
        switch (fc) {
            case CANOPEN_FC_HEARTBEAT:
                // Process NMT heartbeat
                canopen_master_process_heartbeat(&frame);
                break;
                
            case CANOPEN_FC_PDO1_TX:
            case CANOPEN_FC_PDO2_TX:
            case CANOPEN_FC_PDO3_TX:
            case CANOPEN_FC_PDO4_TX:
                // Process TPDO (feedback from axis)
                canopen_master_process_tpdo(&frame);
                break;
                
            case CANOPEN_FC_SDO_TX:
                // Process SDO response
                canopen_master_process_sdo(&frame);
                break;
                
            case CANOPEN_FC_EMCY:
                // Process emergency message
                canopen_master_process_emcy(&frame);
                break;
                
            default:
                // Check for auto-assign protocol
                if (frame.id >= AUTOASSIGN_COB_DISCOVERY && frame.id <= AUTOASSIGN_COB_REPORT) {
                    canopen_master_process_autoassign(&frame);
                }
                break;
        }
    }
}

// =============================================================================
// Ethernet Communication Task (Core 0)
// =============================================================================

static void ethernet_task(void *pvParameters)
{
    ESP_LOGI(TAG, "Ethernet task started on core %d", xPortGetCoreID());
    
    // Initialize Ethernet
    eth_comm_init();
    
    while (1) {
        // Process HMI communication
        eth_comm_tick();
        
        // Update connection status
        g_system.hmi_connected = eth_comm_is_connected();
        
        // Check for standalone mode transition
        if (!g_system.hmi_connected && !g_system.standalone_mode) {
            // HMI disconnected, check if standalone is enabled
            #if CONFIG_OPENCNC_STANDALONE_ENABLE
            ESP_LOGI(TAG, "HMI disconnected, entering standalone mode");
            g_system.standalone_mode = true;
            #endif
        } else if (g_system.hmi_connected && g_system.standalone_mode) {
            ESP_LOGI(TAG, "HMI connected, exiting standalone mode");
            g_system.standalone_mode = false;
        }
        
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// =============================================================================
// Display Task (Core 0)
// =============================================================================

static void display_task_wrapper(void *pvParameters)
{
    ESP_LOGI(TAG, "Display task started on core %d", xPortGetCoreID());
    
    #if CONFIG_OPENCNC_DISPLAY_ENABLE
    display_task_init();
    
    while (1) {
        display_task_tick(&g_system);
        vTaskDelay(pdMS_TO_TICKS(1000 / CONFIG_OPENCNC_DISPLAY_REFRESH_HZ));
    }
    #else
    // Display disabled, task exits
    vTaskDelete(NULL);
    #endif
}

// =============================================================================
// User Interface Task (Core 0) - Buttons, Encoder for Standalone Mode
// =============================================================================

static void ui_task(void *pvParameters)
{
    ESP_LOGI(TAG, "UI task started on core %d", xPortGetCoreID());
    
    #if CONFIG_OPENCNC_STANDALONE_ENABLE
    standalone_mode_init();
    
    while (1) {
        if (g_system.standalone_mode) {
            standalone_mode_tick(&g_system);
        }
        vTaskDelay(pdMS_TO_TICKS(20));  // 50 Hz button polling
    }
    #else
    vTaskDelete(NULL);
    #endif
}

// =============================================================================
// System Initialization
// =============================================================================

static void system_init(void)
{
    ESP_LOGI(TAG, "=================================");
    ESP_LOGI(TAG, "OpenCNC ESP32-P4 Controller v%d.%d.%d",
             1, 0, 0);  // TODO: Use config values
    ESP_LOGI(TAG, "=================================");
    
    // Create mutex for state protection
    mutex_state = xSemaphoreCreateMutex();
    assert(mutex_state != NULL);
    
    // Create queues
    queue_can_rx = xQueueCreate(64, sizeof(can_frame_t));
    assert(queue_can_rx != NULL);
    
    queue_hmi_cmd = xQueueCreate(32, sizeof(uint8_t) * 64);
    assert(queue_hmi_cmd != NULL);
    
    // Initialize CAN buses
    ESP_LOGI(TAG, "Initializing CAN buses...");
    init_can_bus(0, CAN_BUS0_TX_PIN, CAN_BUS0_RX_PIN);
    // TODO: Initialize bus 1 and 2 when ESP-IDF supports multiple TWAI
    
    // Initialize CANopen master
    canopen_master_init();
    
    g_system.state = SYSTEM_STATE_DISCOVERY;
}

// =============================================================================
// Node Discovery Sequence
// =============================================================================

static void run_node_discovery(void)
{
    ESP_LOGI(TAG, "Starting node discovery...");
    
    // Start auto-assign process
    canopen_master_start_discovery(5000);  // 5 second timeout
    
    // Wait for discovery to complete
    while (!canopen_master_is_discovery_complete()) {
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    
    // Get discovered nodes
    g_system.axis_count = canopen_master_get_axis_count();
    g_system.io_node_count = canopen_master_get_io_node_count();
    
    ESP_LOGI(TAG, "Discovery complete: %d axes, %d I/O nodes",
             g_system.axis_count, g_system.io_node_count);
    
    // Move to configuration phase
    g_system.state = SYSTEM_STATE_CONFIGURING;
}

// =============================================================================
// Node Configuration Sequence
// =============================================================================

static void run_node_configuration(void)
{
    ESP_LOGI(TAG, "Configuring nodes...");
    
    // Configure PDO mappings for each axis
    for (int i = 0; i < g_system.axis_count; i++) {
        canopen_master_configure_axis_pdo(i + 1);  // Node IDs start at 1
    }
    
    // Start all nodes
    ESP_LOGI(TAG, "Starting all nodes...");
    canopen_master_start_all_nodes();
    
    // Wait for all nodes to become operational
    int timeout = 50;  // 5 seconds
    while (!canopen_master_all_nodes_operational() && timeout > 0) {
        vTaskDelay(pdMS_TO_TICKS(100));
        timeout--;
    }
    
    if (timeout == 0) {
        ESP_LOGW(TAG, "Some nodes failed to become operational");
        // Continue anyway, individual axis errors will be handled
    }
    
    g_system.state = SYSTEM_STATE_IDLE;
    ESP_LOGI(TAG, "System ready");
}

// =============================================================================
// Main Entry Point
// =============================================================================

void app_main(void)
{
    // Initialize NVS (required for Ethernet driver)
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    
    // Initialize system
    system_init();
    
    // Create tasks pinned to specific cores
    // Core 1: Real-time motion and CAN
    xTaskCreatePinnedToCore(motion_task, "motion", 8192, NULL, 
                            configMAX_PRIORITIES - 1, &task_motion, 1);
    xTaskCreatePinnedToCore(can_rx_task, "can_rx", 4096, NULL,
                            configMAX_PRIORITIES - 2, &task_can_rx, 1);
    
    // Core 0: Network and UI
    xTaskCreatePinnedToCore(ethernet_task, "ethernet", 8192, NULL,
                            5, &task_ethernet, 0);
    xTaskCreatePinnedToCore(display_task_wrapper, "display", 8192, NULL,
                            4, &task_display, 0);
    xTaskCreatePinnedToCore(ui_task, "ui", 4096, NULL,
                            3, &task_ui, 0);
    
    // Run discovery and configuration on main task
    run_node_discovery();
    run_node_configuration();
    
    // Main loop - system monitoring
    while (1) {
        g_system.uptime_seconds++;
        
        // Log status periodically
        if (g_system.uptime_seconds % 60 == 0) {
            ESP_LOGI(TAG, "Uptime: %lu min, State: %d, HMI: %s, Axes: %d",
                     g_system.uptime_seconds / 60,
                     g_system.state,
                     g_system.hmi_connected ? "connected" : "disconnected",
                     g_system.axis_count);
        }
        
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
