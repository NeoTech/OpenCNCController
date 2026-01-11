/**
 * @file display_task.h
 * @brief Local MIPI-DSI Display Handler
 * 
 * Renders status information on the ESP32-P4's MIPI-DSI display.
 */

#ifndef DISPLAY_TASK_H
#define DISPLAY_TASK_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// Forward declaration of system context
struct system_context;

/**
 * @brief Initialize display subsystem
 */
void display_task_init(void);

/**
 * @brief Update display with current system state
 * @param ctx System context
 */
void display_task_tick(const void* ctx);

/**
 * @brief Set display brightness
 * @param brightness 0-100
 */
void display_set_brightness(uint8_t brightness);

/**
 * @brief Show message on display
 * @param line Line number (0-5)
 * @param message Message text
 */
void display_show_message(uint8_t line, const char* message);

/**
 * @brief Clear display
 */
void display_clear(void);

#ifdef __cplusplus
}
#endif

#endif // DISPLAY_TASK_H
