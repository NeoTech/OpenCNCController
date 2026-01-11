/**
 * @file test_fixture.h
 * @brief OpenCNC Test Fixture - Shared definitions and state
 * 
 * Hardware: Raspberry Pi Pico WH + Pimoroni Pico Display 2.0
 * Purpose:  Hardware-in-the-loop testing for Windows HMI development
 */

#ifndef TEST_FIXTURE_H
#define TEST_FIXTURE_H

#include <stdint.h>
#include <stdbool.h>
#include "hal_common.h"

#ifdef __cplusplus
extern "C" {
#endif

// =============================================================================
// Version Information
// =============================================================================

#ifndef FIXTURE_VERSION_MAJOR
#define FIXTURE_VERSION_MAJOR   1
#endif
#ifndef FIXTURE_VERSION_MINOR
#define FIXTURE_VERSION_MINOR   0
#endif
#define FIXTURE_VERSION_PATCH   0

// =============================================================================
// Pico Display 2.0 Pin Definitions
// =============================================================================

// Buttons (directly on Display 2.0 board)
#define PIN_BUTTON_A        12
#define PIN_BUTTON_B        13
#define PIN_BUTTON_X        14
#define PIN_BUTTON_Y        15

// RGB LED
#define PIN_LED_R           6
#define PIN_LED_G           7
#define PIN_LED_B           8

// Display SPI
#define PIN_LCD_CS          17
#define PIN_LCD_SCK         18
#define PIN_LCD_MOSI        19
#define PIN_LCD_DC          16
#define PIN_LCD_BL          20

// =============================================================================
// Display Configuration
// =============================================================================

#ifndef DISPLAY_WIDTH
#define DISPLAY_WIDTH       320
#endif
#ifndef DISPLAY_HEIGHT
#define DISPLAY_HEIGHT      240
#endif
#ifndef DISPLAY_UPDATE_HZ
#define DISPLAY_UPDATE_HZ   10
#endif

// Display regions (Y coordinates)
#define REGION_HEADER_Y     0
#define REGION_HEADER_H     20
#define REGION_STATUS_Y     20
#define REGION_STATUS_H     24
#define REGION_POSITION_Y   44
#define REGION_POSITION_H   72
#define REGION_MOTION_Y     116
#define REGION_MOTION_H     24
#define REGION_STATS_Y      140
#define REGION_STATS_H      20
#define REGION_LOG_Y        160
#define REGION_LOG_H        60
#define REGION_BUTTONS_Y    220
#define REGION_BUTTONS_H    20

// =============================================================================
// Protocol Configuration
// =============================================================================

#ifndef STATUS_BROADCAST_HZ
#define STATUS_BROADCAST_HZ 50
#endif
#ifndef EMULATOR_TICK_HZ
#define EMULATOR_TICK_HZ    1000
#endif
#ifndef MOTION_QUEUE_SIZE
#define MOTION_QUEUE_SIZE   32
#endif

// Packet constants (from cnc_protocol.h)
#define PACKET_START_BYTE   0xAA
#define PACKET_END_BYTE     0x55
#define PACKET_MAX_SIZE     256
#define PAYLOAD_MAX_SIZE    248

// =============================================================================
// Test Fixture Operating Modes
// =============================================================================

typedef enum {
    MODE_NORMAL = 0,    // Normal operation - respond to commands
    MODE_HOMING,        // Simulate homing sequence
    MODE_FAULT_INJECT,  // Buttons inject faults
    MODE_JOG,           // Direct jog control via buttons
    MODE_COUNT
} fixture_mode_t;

// Mode names for display
static const char* const MODE_NAMES[] = {
    "NORMAL",
    "HOMING",
    "FAULT",
    "JOG"
};

// =============================================================================
// Emulated Machine State
// =============================================================================

typedef struct {
    // Current state (mirrors firmware state machine)
    machine_state_t     state;
    alarm_code_t        alarm;
    
    // Position in nanometers (6 axes max)
    int32_t             position_nm[MAX_AXES];
    int32_t             target_nm[MAX_AXES];
    int32_t             velocity_nm_s[MAX_AXES];    // nm/sec
    
    // Motion queue simulation
    uint8_t             queue_depth;
    uint8_t             queue_capacity;
    uint32_t            current_feed_rate;          // mm/min * 1000
    
    // Flags
    bool                in_motion;
    bool                homing_active;
    bool                probe_active;
    uint8_t             limit_switches;             // Bitmask
    
    // Overrides (percentage * 10, e.g., 1000 = 100.0%)
    uint16_t            feed_override;
    uint16_t            rapid_override;
    uint16_t            spindle_override;
    
    // Spindle
    bool                spindle_on;
    bool                spindle_cw;
    uint16_t            spindle_rpm;
    
    // Coolant
    bool                coolant_flood;
    bool                coolant_mist;
    
} emulated_state_t;

// =============================================================================
// Communication Statistics
// =============================================================================

typedef struct {
    uint32_t    packets_rx;
    uint32_t    packets_tx;
    uint32_t    crc_errors;
    uint32_t    nak_count;
    uint32_t    unknown_cmd;
    uint8_t     last_seq_rx;
    uint8_t     last_seq_tx;
    uint8_t     last_cmd;
} comm_stats_t;

// =============================================================================
// Display Log Buffer
// =============================================================================

#define LOG_LINE_COUNT      6
#define LOG_LINE_LENGTH     40

typedef struct {
    char        lines[LOG_LINE_COUNT][LOG_LINE_LENGTH];
    uint8_t     next_line;      // Ring buffer index
    bool        dirty;          // Needs redraw
} log_buffer_t;

// =============================================================================
// Button State
// =============================================================================

typedef struct {
    bool        a_pressed;
    bool        b_pressed;
    bool        x_pressed;
    bool        y_pressed;
    
    bool        a_just_pressed;
    bool        b_just_pressed;
    bool        x_just_pressed;
    bool        y_just_pressed;
    
    uint32_t    a_held_ms;
    uint32_t    b_held_ms;
    uint32_t    x_held_ms;
    uint32_t    y_held_ms;
} button_state_t;

// =============================================================================
// Global Fixture State (shared between cores)
// =============================================================================

typedef struct {
    // Operating mode
    volatile fixture_mode_t     mode;
    
    // Emulated machine state
    volatile emulated_state_t   machine;
    
    // Communication
    volatile comm_stats_t       comm;
    
    // Display
    volatile log_buffer_t       log;
    volatile bool               display_dirty;
    
    // Buttons
    volatile button_state_t     buttons;
    
    // Jog state (for MODE_JOG)
    volatile uint8_t            jog_axis;       // 0=X, 1=Y, 2=Z
    volatile int8_t             jog_direction;  // -1, 0, +1
    
    // Timing
    volatile uint32_t           uptime_ms;
    volatile uint32_t           last_rx_time;
    
} fixture_state_t;

// Global state instance (defined in main.c)
extern fixture_state_t g_fixture;

// =============================================================================
// Utility Macros
// =============================================================================

// Convert mm to nanometers
#define MM_TO_NM(mm)        ((int32_t)((mm) * 1000000.0))

// Convert nanometers to mm (for display)
#define NM_TO_MM(nm)        ((double)(nm) / 1000000.0)

// Clamp value to range
#define CLAMP(val, min, max) \
    ((val) < (min) ? (min) : ((val) > (max) ? (max) : (val)))

// Array size
#define ARRAY_SIZE(arr)     (sizeof(arr) / sizeof((arr)[0]))

#ifdef __cplusplus
}
#endif

#endif // TEST_FIXTURE_H
