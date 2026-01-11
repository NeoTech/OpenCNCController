/**
 * @file standalone_mode.h
 * @brief Standalone Operation Mode
 * 
 * Provides basic CNC control without Windows HMI connection.
 * Supports jog mode, homing, and axis selection via local controls.
 */

#ifndef STANDALONE_MODE_H
#define STANDALONE_MODE_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// Forward declaration
struct system_context;

// =============================================================================
// Jog Mode
// =============================================================================

typedef enum {
    JOG_SPEED_SLOW      = 0,    // 0.1 mm per encoder tick
    JOG_SPEED_MEDIUM    = 1,    // 1.0 mm per encoder tick
    JOG_SPEED_FAST      = 2,    // 10.0 mm per encoder tick
} jog_speed_t;

// =============================================================================
// Button Functions
// =============================================================================

typedef enum {
    BTN_AXIS_SELECT     = 0,    // Cycle through axes
    BTN_JOG_SPEED       = 1,    // Cycle through jog speeds
    BTN_HOME            = 2,    // Start homing current axis
    BTN_ESTOP           = 3,    // Emergency stop
} button_function_t;

// =============================================================================
// Standalone State
// =============================================================================

typedef struct {
    bool        active;
    uint8_t     selected_axis;      // 0-8
    jog_speed_t jog_speed;
    
    // Encoder state
    int32_t     encoder_count;
    int32_t     last_encoder_count;
    
    // Button debounce
    uint32_t    button_state;
    uint32_t    button_last_tick[4];
    
} standalone_state_t;

// =============================================================================
// Public Functions
// =============================================================================

/**
 * @brief Initialize standalone mode
 */
void standalone_mode_init(void);

/**
 * @brief Process standalone mode (call at ~50Hz)
 * @param ctx System context
 */
void standalone_mode_tick(void* ctx);

/**
 * @brief Get standalone state
 */
const standalone_state_t* standalone_mode_get_state(void);

/**
 * @brief Set selected axis
 */
void standalone_mode_select_axis(uint8_t axis);

/**
 * @brief Set jog speed
 */
void standalone_mode_set_jog_speed(jog_speed_t speed);

/**
 * @brief Process jog encoder movement
 * @param delta Encoder delta
 */
void standalone_mode_jog(int32_t delta);

/**
 * @brief Start homing for selected axis
 */
void standalone_mode_home_axis(void);

#ifdef __cplusplus
}
#endif

#endif // STANDALONE_MODE_H
