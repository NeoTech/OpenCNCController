/**
 * @file display.c
 * @brief Display rendering for Pico Display 2.0
 * 
 * Uses Pimoroni pico_graphics library with ST7789 driver.
 * Renders to 320x240 LCD at 10Hz from Core 1.
 */

#include "display.h"
#include "test_fixture.h"
#include "emulator.h"
#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "hardware/pwm.h"
#include <string.h>
#include <stdio.h>

// Note: In actual build, include Pimoroni headers:
// #include "pico_display_2.hpp"
// #include "pico_graphics.hpp"
// For now, we'll create a C-compatible abstraction

// =============================================================================
// Display State
// =============================================================================

static bool s_display_initialized = false;

// LED flash state
static bool s_led_flash_active = false;
static uint32_t s_led_flash_start = 0;
static uint8_t s_led_flash_r, s_led_flash_g, s_led_flash_b;
#define LED_FLASH_DURATION_MS 100

// =============================================================================
// Low-Level Display Functions (placeholder for Pimoroni integration)
// =============================================================================

// These functions would be implemented using Pimoroni's pico_graphics
// For now, they're stubs that will be filled in during actual build

static void gfx_init(void) {
    // Initialize SPI for display
    spi_init(spi0, 62500000);  // 62.5 MHz
    gpio_set_function(PIN_LCD_SCK, GPIO_FUNC_SPI);
    gpio_set_function(PIN_LCD_MOSI, GPIO_FUNC_SPI);
    
    // CS, DC, Backlight as GPIO
    gpio_init(PIN_LCD_CS);
    gpio_set_dir(PIN_LCD_CS, GPIO_OUT);
    gpio_put(PIN_LCD_CS, 1);
    
    gpio_init(PIN_LCD_DC);
    gpio_set_dir(PIN_LCD_DC, GPIO_OUT);
    
    // Backlight PWM
    gpio_set_function(PIN_LCD_BL, GPIO_FUNC_PWM);
    uint slice = pwm_gpio_to_slice_num(PIN_LCD_BL);
    pwm_set_wrap(slice, 255);
    pwm_set_chan_level(slice, PWM_CHAN_A, 200);  // ~80% brightness
    pwm_set_enabled(slice, true);
}

static void gfx_clear(uint8_t color) {
    // Would clear framebuffer
    (void)color;
}

static void gfx_rect(int x, int y, int w, int h, uint8_t color) {
    // Would draw filled rectangle
    (void)x; (void)y; (void)w; (void)h; (void)color;
}

static void gfx_text(const char* str, int x, int y, uint8_t color) {
    // Would draw text
    (void)str; (void)x; (void)y; (void)color;
}

static void gfx_update(void) {
    // Would flush framebuffer to display via DMA
}

// =============================================================================
// LED Control
// =============================================================================

static void led_init(void) {
    // RGB LED pins as PWM
    gpio_set_function(PIN_LED_R, GPIO_FUNC_PWM);
    gpio_set_function(PIN_LED_G, GPIO_FUNC_PWM);
    gpio_set_function(PIN_LED_B, GPIO_FUNC_PWM);
    
    // Configure PWM for each channel
    uint slice_r = pwm_gpio_to_slice_num(PIN_LED_R);
    uint slice_g = pwm_gpio_to_slice_num(PIN_LED_G);
    uint slice_b = pwm_gpio_to_slice_num(PIN_LED_B);
    
    pwm_set_wrap(slice_r, 255);
    pwm_set_wrap(slice_g, 255);
    pwm_set_wrap(slice_b, 255);
    
    pwm_set_enabled(slice_r, true);
    pwm_set_enabled(slice_g, true);
    pwm_set_enabled(slice_b, true);
    
    // Start with LED off
    display_set_led(0, 0, 0);
}

void display_set_led(uint8_t r, uint8_t g, uint8_t b) {
    // LED is common anode, so invert values
    pwm_set_gpio_level(PIN_LED_R, 255 - r);
    pwm_set_gpio_level(PIN_LED_G, 255 - g);
    pwm_set_gpio_level(PIN_LED_B, 255 - b);
}

void display_flash_led(uint8_t r, uint8_t g, uint8_t b) {
    s_led_flash_active = true;
    s_led_flash_start = g_fixture.uptime_ms;
    s_led_flash_r = r;
    s_led_flash_g = g;
    s_led_flash_b = b;
    display_set_led(r, g, b);
}

void display_led_tick(void) {
    if (s_led_flash_active) {
        if (g_fixture.uptime_ms - s_led_flash_start >= LED_FLASH_DURATION_MS) {
            s_led_flash_active = false;
            
            // Set LED based on machine state
            switch (g_fixture.machine.state) {
                case STATE_RUNNING:
                    display_set_led(0, 50, 0);   // Dim green
                    break;
                case STATE_ALARM:
                case STATE_ESTOP:
                    display_set_led(50, 0, 0);  // Dim red
                    break;
                case STATE_HOMING:
                    display_set_led(50, 50, 0); // Dim yellow
                    break;
                default:
                    display_set_led(0, 0, 20);  // Dim blue (idle)
                    break;
            }
        }
    }
}

// =============================================================================
// Log Buffer
// =============================================================================

void display_log(const char* message) {
    // Add to ring buffer
    uint8_t idx = g_fixture.log.next_line;
    strncpy((char*)g_fixture.log.lines[idx], message, LOG_LINE_LENGTH - 1);
    g_fixture.log.lines[idx][LOG_LINE_LENGTH - 1] = '\0';
    g_fixture.log.next_line = (idx + 1) % LOG_LINE_COUNT;
    g_fixture.log.dirty = true;
    
    // Flash LED on activity
    display_flash_led(0, 100, 100);  // Cyan flash
}

void display_log_clear(void) {
    for (int i = 0; i < LOG_LINE_COUNT; i++) {
        g_fixture.log.lines[i][0] = '\0';
    }
    g_fixture.log.next_line = 0;
    g_fixture.log.dirty = true;
}

// =============================================================================
// Display Initialization
// =============================================================================

void display_init(void) {
    gfx_init();
    led_init();
    display_log_clear();
    s_display_initialized = true;
    
    display_log("Test Fixture Ready");
}

// =============================================================================
// Display Update (called from Core 1 at DISPLAY_UPDATE_HZ)
// =============================================================================

void display_update(void) {
    if (!s_display_initialized) return;
    
    // Clear screen
    gfx_clear(COLOR_BG);
    
    // -------------------------------------------------------------------------
    // Header (Y: 0-20)
    // -------------------------------------------------------------------------
    gfx_rect(0, REGION_HEADER_Y, DISPLAY_WIDTH, REGION_HEADER_H, COLOR_HEADER_BG);
    gfx_text("TEST FIXTURE", 4, 4, COLOR_WHITE);
    
    // Mode indicator
    char mode_str[20];
    snprintf(mode_str, sizeof(mode_str), "Mode: %s", MODE_NAMES[g_fixture.mode]);
    gfx_text(mode_str, 200, 4, COLOR_YELLOW);
    
    // -------------------------------------------------------------------------
    // Status Bar (Y: 20-44)
    // -------------------------------------------------------------------------
    char status_str[40];
    snprintf(status_str, sizeof(status_str), "State: %-10s", emulator_state_name());
    gfx_text(status_str, 4, REGION_STATUS_Y + 4, COLOR_TEXT);
    
    snprintf(status_str, sizeof(status_str), "Alarm: %s", emulator_alarm_name());
    uint8_t alarm_color = (g_fixture.machine.alarm == ALARM_NONE) ? 
                          COLOR_STATUS_OK : COLOR_STATUS_ERROR;
    gfx_text(status_str, 180, REGION_STATUS_Y + 4, alarm_color);
    
    // -------------------------------------------------------------------------
    // Position Display (Y: 44-116)
    // -------------------------------------------------------------------------
    static const char* axis_names[] = {"X", "Y", "Z"};
    static const int32_t axis_max_nm[] = {
        MM_TO_NM(EMU_DEFAULT_TRAVEL_X),
        MM_TO_NM(EMU_DEFAULT_TRAVEL_Y),
        MM_TO_NM(EMU_DEFAULT_TRAVEL_Z)
    };
    
    for (int i = 0; i < 3; i++) {
        int y = REGION_POSITION_Y + 4 + i * 24;
        
        // Axis label
        gfx_text(axis_names[i], 4, y, COLOR_TEXT);
        
        // Position bar background
        int bar_x = 24;
        int bar_w = 180;
        int bar_h = 16;
        gfx_rect(bar_x, y, bar_w, bar_h, COLOR_GAUGE_BG);
        
        // Position bar fill
        int32_t pos = g_fixture.machine.position_nm[i];
        int32_t max = axis_max_nm[i];
        if (max > 0) {
            int fill_w = (int)((int64_t)pos * bar_w / max);
            if (fill_w < 0) fill_w = 0;
            if (fill_w > bar_w) fill_w = bar_w;
            gfx_rect(bar_x, y, fill_w, bar_h, COLOR_GAUGE_FG);
        }
        
        // Position text
        char pos_str[20];
        snprintf(pos_str, sizeof(pos_str), "%+.3f mm", NM_TO_MM(pos));
        gfx_text(pos_str, 210, y, COLOR_TEXT);
    }
    
    // -------------------------------------------------------------------------
    // Motion Info (Y: 116-140)
    // -------------------------------------------------------------------------
    char motion_str[40];
    snprintf(motion_str, sizeof(motion_str), "Queue: %d/%d", 
             g_fixture.machine.queue_depth, g_fixture.machine.queue_capacity);
    gfx_text(motion_str, 4, REGION_MOTION_Y + 4, COLOR_TEXT);
    
    snprintf(motion_str, sizeof(motion_str), "Feed: %lu mm/min", 
             (unsigned long)(g_fixture.machine.current_feed_rate / 1000));
    gfx_text(motion_str, 140, REGION_MOTION_Y + 4, COLOR_TEXT);
    
    // -------------------------------------------------------------------------
    // Communication Stats (Y: 140-160)
    // -------------------------------------------------------------------------
    char stats_str[50];
    snprintf(stats_str, sizeof(stats_str), "TX:%lu RX:%lu CRC:%lu NAK:%lu",
             (unsigned long)g_fixture.comm.packets_tx,
             (unsigned long)g_fixture.comm.packets_rx,
             (unsigned long)g_fixture.comm.crc_errors,
             (unsigned long)g_fixture.comm.nak_count);
    gfx_text(stats_str, 4, REGION_STATS_Y + 2, COLOR_LIGHT_GRAY);
    
    // -------------------------------------------------------------------------
    // Log Area (Y: 160-220)
    // -------------------------------------------------------------------------
    gfx_rect(0, REGION_LOG_Y, DISPLAY_WIDTH, REGION_LOG_H, COLOR_DARK_GRAY);
    
    // Draw log lines (oldest to newest)
    for (int i = 0; i < LOG_LINE_COUNT; i++) {
        int line_idx = (g_fixture.log.next_line + i) % LOG_LINE_COUNT;
        int y = REGION_LOG_Y + 2 + i * 10;
        
        if (g_fixture.log.lines[line_idx][0] != '\0') {
            char prefixed[LOG_LINE_LENGTH + 2];
            snprintf(prefixed, sizeof(prefixed), "> %s", 
                     (const char*)g_fixture.log.lines[line_idx]);
            gfx_text(prefixed, 4, y, COLOR_LOG_TEXT);
        }
    }
    
    // -------------------------------------------------------------------------
    // Button Hints (Y: 220-240)
    // -------------------------------------------------------------------------
    gfx_rect(0, REGION_BUTTONS_Y, DISPLAY_WIDTH, REGION_BUTTONS_H, COLOR_HEADER_BG);
    
    const char* hint;
    switch (g_fixture.mode) {
        case MODE_NORMAL:
            hint = "[A]Mode [B]State [X]Jog+ [Y]Jog+";
            break;
        case MODE_HOMING:
            hint = "[A]Mode [B]Home [X]Axis [Y]Cancel";
            break;
        case MODE_FAULT_INJECT:
            hint = "[A]Mode [B]Alarm [X]Next [Y]Clear";
            break;
        case MODE_JOG:
            hint = "[A]Mode [B]Axis [X]+ [Y]-";
            break;
        default:
            hint = "[A]Mode [B]Action [X]+ [Y]-";
            break;
    }
    gfx_text(hint, 4, REGION_BUTTONS_Y + 4, COLOR_WHITE);
    
    // -------------------------------------------------------------------------
    // Flush to display
    // -------------------------------------------------------------------------
    gfx_update();
    
    // Update LED
    display_led_tick();
}
