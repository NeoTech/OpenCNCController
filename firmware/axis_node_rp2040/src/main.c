/**
 * @file main.c
 * @brief RP2040 CAN-FD Axis Node
 * 
 * CANopen slave node using external MCP2518FD CAN-FD controller.
 * Implements CiA 402 drive profile for stepper motor control.
 */

#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/timer.h"
#include "hardware/irq.h"
#include "hardware/flash.h"

#include "mcp2518fd.h"
#include "canopen_types.h"
#include "cia402.h"
#include "nmt.h"
#include "sdo.h"
#include "pdo.h"
#include "auto_assign.h"
#include "node_config.h"

// =============================================================================
// Pin Definitions
// =============================================================================

// SPI to MCP2518FD (defined in mcp2518fd.h)
// GP2 = SCK, GP3 = MOSI, GP4 = MISO, GP5 = CS, GP6 = INT

// Stepper control
#define PIN_STEP        16
#define PIN_DIR         17
#define PIN_ENABLE      18

// Limit switches
#define PIN_LIMIT_MIN   19
#define PIN_LIMIT_MAX   20
#define PIN_HOME        21

// Status LED
#define PIN_LED         25

// =============================================================================
// Global State
// =============================================================================

static mcp2518fd_t g_can;
static cia402_drive_t g_drive;
static full_node_config_t g_config;
static nmt_slave_t g_nmt;
static sdo_server_t g_sdo;
static pdo_manager_t g_pdo;
static autoassign_slave_t g_autoassign;

static uint8_t g_node_id = 0;
static volatile bool g_sync_received = false;
static uint32_t g_tick_count = 0;

// Stepper state
static volatile int32_t g_position = 0;
static int32_t g_target_position = 0;
static int32_t g_current_velocity = 0;
static bool g_enabled = false;
static bool g_homing = false;
static bool g_is_homed = false;

// =============================================================================
// Forward Declarations
// =============================================================================

static void can_rx_callback(can_frame_t* frame);
static void can_transmit(const can_frame_t* frame);
static bool od_access(uint16_t index, uint8_t subindex, bool is_write,
                      uint8_t* data, uint32_t* size, uint32_t* abort_code);
static void timer_callback(uint alarm_id);
static void gpio_callback(uint gpio, uint32_t events);
static void load_config(void);

// =============================================================================
// Initialization
// =============================================================================

static void gpio_init_all(void)
{
    // Stepper outputs
    gpio_init(PIN_STEP);
    gpio_set_dir(PIN_STEP, GPIO_OUT);
    gpio_put(PIN_STEP, 0);
    
    gpio_init(PIN_DIR);
    gpio_set_dir(PIN_DIR, GPIO_OUT);
    gpio_put(PIN_DIR, 0);
    
    gpio_init(PIN_ENABLE);
    gpio_set_dir(PIN_ENABLE, GPIO_OUT);
    gpio_put(PIN_ENABLE, 1);  // Active low, start disabled
    
    // Limit switches (pull-up, active low)
    gpio_init(PIN_LIMIT_MIN);
    gpio_set_dir(PIN_LIMIT_MIN, GPIO_IN);
    gpio_pull_up(PIN_LIMIT_MIN);
    
    gpio_init(PIN_LIMIT_MAX);
    gpio_set_dir(PIN_LIMIT_MAX, GPIO_IN);
    gpio_pull_up(PIN_LIMIT_MAX);
    
    gpio_init(PIN_HOME);
    gpio_set_dir(PIN_HOME, GPIO_IN);
    gpio_pull_up(PIN_HOME);
    
    // LED
    gpio_init(PIN_LED);
    gpio_set_dir(PIN_LED, GPIO_OUT);
    gpio_put(PIN_LED, 0);
    
    // Configure interrupt for CAN INT pin
    gpio_set_irq_enabled_callback(MCP2518FD_PIN_INT, GPIO_IRQ_EDGE_FALL, true, gpio_callback);
}

static void can_init(void)
{
    g_can.pin_cs = MCP2518FD_PIN_CS;
    g_can.pin_int = MCP2518FD_PIN_INT;
    g_can.rx_callback = can_rx_callback;
    
    if (!mcp2518fd_init(&g_can)) {
        printf("CAN init failed!\n");
        return;
    }
    
    mcp2518fd_enable_interrupts(&g_can, true);
    mcp2518fd_set_mode(&g_can, MCP2518FD_MODE_NORMAL);
    
    printf("CAN initialized\n");
}

static void canopen_init(void)
{
    // Initialize auto-assign
    node_config_t compact;
    node_config_to_compact(&g_config, &compact);
    autoassign_slave_init(&g_autoassign, &compact);
    
    // Start discovery with random delay based on chip ID
    uint64_t chip_id;
    flash_get_unique_id((uint8_t*)&chip_id);
    autoassign_slave_start(&g_autoassign, (uint32_t)chip_id);
}

// =============================================================================
// CAN Callbacks
// =============================================================================

static void can_transmit(const can_frame_t* frame)
{
    mcp2518fd_transmit(&g_can, frame);
}

static void can_rx_callback(can_frame_t* frame)
{
    // Auto-assign frames (before we have an ID)
    if (frame->id >= AUTOASSIGN_COB_DISCOVERY && frame->id <= AUTOASSIGN_COB_REPORT) {
        autoassign_slave_process(&g_autoassign, frame, can_transmit);
        
        // Check if we just got assigned
        if (autoassign_slave_is_assigned(&g_autoassign) && g_node_id == 0) {
            g_node_id = autoassign_slave_get_id(&g_autoassign);
            
            // Initialize NMT slave
            nmt_slave_init(&g_nmt, g_node_id, 1000);
            
            // Initialize SDO server
            sdo_server_init(&g_sdo, g_node_id, od_access);
            
            // Initialize PDO manager
            pdo_manager_init(&g_pdo, g_node_id);
            pdo_manager_setup_cia402(&g_pdo, true);
            
            // Send boot-up message
            nmt_slave_send_bootup(&g_nmt, can_transmit);
            
            printf("Assigned node ID: %d\n", g_node_id);
        }
        return;
    }
    
    // If we don't have an ID yet, ignore other frames
    if (g_node_id == 0) return;
    
    uint8_t node_id_from_frame = CANOPEN_NODE_FROM_COB(frame->id);
    uint16_t fc = frame->id & 0x780;
    
    // NMT command
    if (frame->id == 0) {
        nmt_command_t cmd;
        uint8_t target;
        if (nmt_parse_command(frame, &cmd, &target)) {
            if (target == 0 || target == g_node_id) {
                nmt_slave_process_command(&g_nmt, cmd, target);
            }
        }
        return;
    }
    
    // SYNC
    if (fc == CANOPEN_FC_SYNC) {
        g_sync_received = true;
        
        // Process SYNC in PDO manager
        pdo_manager_sync(&g_pdo, frame->dlc > 0 ? frame->data[0] : 0);
        
        // Transmit TPDOs if operational
        if (nmt_slave_is_operational(&g_nmt)) {
            pdo_manager_transmit_tpdos(&g_pdo, NULL, can_transmit);
        }
        return;
    }
    
    // Only process frames addressed to us
    if (node_id_from_frame != g_node_id) return;
    
    switch (fc) {
        case CANOPEN_FC_SDO_RX:
            sdo_server_process(&g_sdo, frame, can_transmit);
            break;
            
        case CANOPEN_FC_PDO1_RX:
        case CANOPEN_FC_PDO2_RX:
        case CANOPEN_FC_PDO3_RX:
        case CANOPEN_FC_PDO4_RX:
            if (nmt_slave_is_operational(&g_nmt)) {
                pdo_manager_process_rpdo(&g_pdo, frame, NULL);
            }
            break;
            
        default:
            break;
    }
}

static void gpio_callback(uint gpio, uint32_t events)
{
    if (gpio == MCP2518FD_PIN_INT) {
        mcp2518fd_irq_handler(&g_can);
    }
}

// =============================================================================
// Object Dictionary
// =============================================================================

static bool od_access(uint16_t index, uint8_t subindex, bool is_write,
                      uint8_t* data, uint32_t* size, uint32_t* abort_code)
{
    switch (index) {
        case OD_CONTROLWORD:
            if (is_write) {
                if (*size >= 2) {
                    g_drive.controlword = data[0] | (data[1] << 8);
                    cia402_process_controlword(&g_drive, g_drive.controlword);
                    return true;
                }
            } else {
                *size = 2;
                data[0] = g_drive.controlword & 0xFF;
                data[1] = (g_drive.controlword >> 8) & 0xFF;
                return true;
            }
            break;
            
        case OD_STATUSWORD:
            if (!is_write) {
                g_drive.statusword = cia402_build_statusword(&g_drive);
                *size = 2;
                data[0] = g_drive.statusword & 0xFF;
                data[1] = (g_drive.statusword >> 8) & 0xFF;
                return true;
            }
            break;
            
        case OD_TARGET_POSITION:
            if (is_write) {
                if (*size >= 4) {
                    memcpy(&g_target_position, data, 4);
                    g_drive.position_target = g_target_position;
                    return true;
                }
            } else {
                *size = 4;
                memcpy(data, &g_target_position, 4);
                return true;
            }
            break;
            
        case OD_POSITION_ACTUAL:
            if (!is_write) {
                *size = 4;
                int32_t pos = g_position;
                memcpy(data, &pos, 4);
                return true;
            }
            break;
            
        case OD_TARGET_VELOCITY:
            if (is_write) {
                if (*size >= 4) {
                    memcpy(&g_drive.velocity_target, data, 4);
                    return true;
                }
            } else {
                *size = 4;
                memcpy(data, &g_drive.velocity_target, 4);
                return true;
            }
            break;
            
        case OD_VELOCITY_ACTUAL:
            if (!is_write) {
                *size = 4;
                memcpy(data, &g_current_velocity, 4);
                return true;
            }
            break;
            
        case OD_DIGITAL_INPUTS: {
            if (!is_write) {
                uint32_t inputs = 0;
                if (!gpio_get(PIN_LIMIT_MIN)) inputs |= (1 << 0);
                if (!gpio_get(PIN_LIMIT_MAX)) inputs |= (1 << 1);
                if (!gpio_get(PIN_HOME)) inputs |= (1 << 2);
                *size = 4;
                memcpy(data, &inputs, 4);
                return true;
            }
            break;
        }
            
        default:
            break;
    }
    
    *abort_code = SDO_ABORT_OBJECT_NOT_EXIST;
    return false;
}

// =============================================================================
// Timer (1 kHz tick)
// =============================================================================

static bool timer_callback_func(struct repeating_timer *t)
{
    g_tick_count++;
    
    // Auto-assign tick
    if (!autoassign_slave_is_assigned(&g_autoassign)) {
        autoassign_slave_tick(&g_autoassign, 1, can_transmit);
        return true;
    }
    
    // NMT heartbeat
    nmt_slave_tick(&g_nmt, 1, can_transmit);
    
    // SDO timeout
    sdo_server_tick(&g_sdo, 1);
    
    // Update drive state
    g_drive.position_actual = g_position;
    g_drive.velocity_actual = g_current_velocity;
    
    // Read limits
    bool limit_min = !gpio_get(PIN_LIMIT_MIN);
    bool limit_max = !gpio_get(PIN_LIMIT_MAX);
    
    g_drive.digital_inputs = 0;
    if (limit_min) g_drive.digital_inputs |= (1 << 0);
    if (limit_max) g_drive.digital_inputs |= (1 << 1);
    if (!gpio_get(PIN_HOME)) g_drive.digital_inputs |= (1 << 2);
    
    // Basic motion control (simplified for scaffold)
    if (g_enabled && cia402_is_enabled(&g_drive)) {
        int32_t error = g_target_position - g_position;
        
        // Check limits
        if ((error < 0 && limit_min) || (error > 0 && limit_max)) {
            error = 0;
        }
        
        // Simple step toward target
        if (error != 0) {
            int8_t dir = (error > 0) ? 1 : -1;
            gpio_put(PIN_DIR, dir > 0);
            
            // Generate step pulse
            gpio_put(PIN_STEP, 1);
            busy_wait_us(2);
            gpio_put(PIN_STEP, 0);
            
            g_position += dir;
        }
    }
    
    return true;
}

// =============================================================================
// Configuration
// =============================================================================

static void load_config(void)
{
    // Try to load from flash
    if (!node_config_load(&g_config)) {
        // Set defaults
        g_config.node_type = NODE_TYPE_STEPPER;
        g_config.axis = AXIS_X;
        g_config.motion.max_velocity = 50000;
        g_config.motion.max_acceleration = 100000;
        g_config.motion.steps_per_mm = 200;
    }
    
    printf("Config: axis=%c, max_vel=%ld\n", 
           'X' + g_config.axis, g_config.motion.max_velocity);
}

// =============================================================================
// LED Status
// =============================================================================

static void update_led(void)
{
    static uint32_t last_toggle = 0;
    uint32_t now = g_tick_count;
    
    if (g_node_id == 0) {
        // Waiting for assignment - slow blink
        if (now - last_toggle >= 500) {
            gpio_put(PIN_LED, !gpio_get(PIN_LED));
            last_toggle = now;
        }
    } else if (nmt_slave_is_operational(&g_nmt)) {
        // Operational - fast blink
        if (now - last_toggle >= 100) {
            gpio_put(PIN_LED, !gpio_get(PIN_LED));
            last_toggle = now;
        }
    } else {
        // Pre-operational - medium blink
        if (now - last_toggle >= 250) {
            gpio_put(PIN_LED, !gpio_get(PIN_LED));
            last_toggle = now;
        }
    }
}

// =============================================================================
// Main
// =============================================================================

int main()
{
    stdio_init_all();
    sleep_ms(100);
    
    printf("\n=== RP2040 CAN-FD Axis Node ===\n");
    
    // Initialize subsystems
    gpio_init_all();
    load_config();
    can_init();
    canopen_init();
    
    // Initialize drive
    memset(&g_drive, 0, sizeof(g_drive));
    g_drive.state = CIA402_STATE_NOT_READY;
    g_drive.mode_of_operation = CIA402_MODE_CSP;
    
    // Start 1 kHz timer
    struct repeating_timer timer;
    add_repeating_timer_ms(1, timer_callback_func, NULL, &timer);
    
    printf("Waiting for CAN master...\n");
    
    // Main loop
    while (true) {
        // Poll CAN receive
        can_frame_t frame;
        while (mcp2518fd_receive(&g_can, &frame)) {
            can_rx_callback(&frame);
        }
        
        // Update LED
        update_led();
        
        // Process CiA 402 state machine
        if (g_node_id != 0) {
            cia402_state_machine_tick(&g_drive);
            
            // Update enable output
            bool should_enable = cia402_is_enabled(&g_drive);
            if (should_enable != g_enabled) {
                g_enabled = should_enable;
                gpio_put(PIN_ENABLE, !g_enabled);  // Active low
                printf("Drive %s\n", g_enabled ? "enabled" : "disabled");
            }
        }
        
        // Small delay
        sleep_us(100);
    }
    
    return 0;
}
