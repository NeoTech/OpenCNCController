/**
 * @file button_handler.h
 * @brief Button input handling for Pico Display 2.0
 */

#ifndef BUTTON_HANDLER_H
#define BUTTON_HANDLER_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// =============================================================================
// Debounce Configuration
// =============================================================================

#define BUTTON_DEBOUNCE_MS      50
#define BUTTON_REPEAT_DELAY_MS  500
#define BUTTON_REPEAT_RATE_MS   100

// =============================================================================
// Function Prototypes
// =============================================================================

/**
 * @brief Initialize button GPIOs
 */
void buttons_init(void);

/**
 * @brief Poll buttons and update state (call from Core 1)
 * Handles debouncing and edge detection
 */
void buttons_poll(void);

/**
 * @brief Process button actions based on current mode
 * Called after buttons_poll() to handle mode-specific actions
 */
void buttons_process(void);

/**
 * @brief Check if button A was just pressed (rising edge)
 */
bool button_a_pressed(void);

/**
 * @brief Check if button B was just pressed
 */
bool button_b_pressed(void);

/**
 * @brief Check if button X was just pressed
 */
bool button_x_pressed(void);

/**
 * @brief Check if button Y was just pressed
 */
bool button_y_pressed(void);

/**
 * @brief Check if button A is currently held
 */
bool button_a_held(void);

/**
 * @brief Check if button B is currently held
 */
bool button_b_held(void);

/**
 * @brief Check if button X is currently held
 */
bool button_x_held(void);

/**
 * @brief Check if button Y is currently held
 */
bool button_y_held(void);

#ifdef __cplusplus
}
#endif

#endif // BUTTON_HANDLER_H
