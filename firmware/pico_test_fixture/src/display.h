/**
 * @file display.h
 * @brief Display rendering for Pico Display 2.0
 */

#ifndef DISPLAY_H
#define DISPLAY_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// =============================================================================
// Display Colors (RGB332 format for PEN_RGB332)
// =============================================================================

// RGB332: RRRGGGBB
#define COLOR_BLACK         0x00
#define COLOR_WHITE         0xFF
#define COLOR_RED           0xE0
#define COLOR_GREEN         0x1C
#define COLOR_BLUE          0x03
#define COLOR_YELLOW        0xFC
#define COLOR_CYAN          0x1F
#define COLOR_MAGENTA       0xE3
#define COLOR_ORANGE        0xF4
#define COLOR_DARK_GRAY     0x49
#define COLOR_LIGHT_GRAY    0x92

// Semantic colors
#define COLOR_BG            COLOR_BLACK
#define COLOR_TEXT          COLOR_WHITE
#define COLOR_HEADER_BG     COLOR_BLUE
#define COLOR_STATUS_OK     COLOR_GREEN
#define COLOR_STATUS_WARN   COLOR_YELLOW
#define COLOR_STATUS_ERROR  COLOR_RED
#define COLOR_GAUGE_BG      COLOR_DARK_GRAY
#define COLOR_GAUGE_FG      COLOR_CYAN
#define COLOR_LOG_TEXT      COLOR_LIGHT_GRAY

// =============================================================================
// Function Prototypes
// =============================================================================

/**
 * @brief Initialize display hardware and graphics library
 */
void display_init(void);

/**
 * @brief Full display refresh (call at DISPLAY_UPDATE_HZ from Core 1)
 */
void display_update(void);

/**
 * @brief Add a message to the log buffer
 * @param message Message string (will be truncated to LOG_LINE_LENGTH)
 */
void display_log(const char* message);

/**
 * @brief Clear the log buffer
 */
void display_log_clear(void);

/**
 * @brief Set RGB LED color
 * @param r Red (0-255)
 * @param g Green (0-255)
 * @param b Blue (0-255)
 */
void display_set_led(uint8_t r, uint8_t g, uint8_t b);

/**
 * @brief Flash LED briefly (non-blocking)
 * @param r Red
 * @param g Green
 * @param b Blue
 */
void display_flash_led(uint8_t r, uint8_t g, uint8_t b);

/**
 * @brief Update LED flash state (call from main loop)
 */
void display_led_tick(void);

#ifdef __cplusplus
}
#endif

#endif // DISPLAY_H
