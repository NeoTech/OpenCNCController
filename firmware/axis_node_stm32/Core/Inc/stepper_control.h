/**
 * @file stepper_control.h
 * @brief Stepper Motor Control with Acceleration
 * 
 * Implements step/direction pulse generation with trapezoidal velocity
 * profiles using TIM2 for step timing.
 */

#ifndef STEPPER_CONTROL_H
#define STEPPER_CONTROL_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

// =============================================================================
// Configuration
// =============================================================================

#define STEPPER_TIMER_FREQ       170000000  // 170 MHz timer clock
#define STEPPER_MIN_STEP_US      2          // Minimum step pulse width in microseconds
#define STEPPER_MAX_STEPS_PER_S  100000     // Maximum step rate (100 kHz)
#define STEPPER_MIN_PERIOD       (STEPPER_TIMER_FREQ / STEPPER_MAX_STEPS_PER_S)

// Default motion parameters (can be overridden by config)
#define DEFAULT_MAX_VELOCITY     50000      // Steps per second
#define DEFAULT_ACCELERATION     100000     // Steps per second^2

// =============================================================================
// Stepper State
// =============================================================================

typedef enum {
    STEPPER_STATE_IDLE = 0,
    STEPPER_STATE_ACCELERATING,
    STEPPER_STATE_CRUISING,
    STEPPER_STATE_DECELERATING,
    STEPPER_STATE_STOPPING,         // Emergency stop
    STEPPER_STATE_HOMING_FAST,
    STEPPER_STATE_HOMING_SLOW,
    STEPPER_STATE_HOMING_BACKOFF,
} stepper_state_t;

typedef enum {
    STEPPER_LIMIT_NONE = 0,
    STEPPER_LIMIT_MIN  = (1 << 0),
    STEPPER_LIMIT_MAX  = (1 << 1),
    STEPPER_LIMIT_HOME = (1 << 2),
} stepper_limit_t;

// =============================================================================
// Motion Command
// =============================================================================

typedef struct {
    int32_t  target_position;       // Target position in steps
    int32_t  max_velocity;          // Maximum velocity (steps/s)
    int32_t  acceleration;          // Acceleration (steps/s^2)
    int32_t  deceleration;          // Deceleration (steps/s^2)
    bool     absolute;              // Absolute or relative move
} stepper_motion_cmd_t;

// =============================================================================
// Stepper Context
// =============================================================================

typedef struct {
    // Current state
    stepper_state_t state;
    
    // Position tracking
    int32_t         position;           // Current position (steps)
    int32_t         target_position;    // Target position (steps)
    int32_t         home_offset;        // Home position offset
    
    // Velocity state
    int32_t         current_velocity;   // Current velocity (steps/s)
    int32_t         target_velocity;    // Target velocity for cruising
    int32_t         max_velocity;       // Maximum allowed velocity
    
    // Acceleration
    int32_t         acceleration;       // Acceleration (steps/s^2)
    int32_t         deceleration;       // Deceleration (steps/s^2)
    int32_t         decel_distance;     // Distance needed to decelerate
    
    // Step generation
    int8_t          direction;          // +1 or -1
    uint32_t        step_period;        // Current timer period (ticks)
    uint32_t        step_count;         // Steps completed in current move
    uint32_t        steps_to_go;        // Steps remaining
    
    // Limit switches
    uint8_t         limit_state;        // Current limit switch state
    bool            limit_inverted;     // Limit switches active low
    
    // Homing
    bool            is_homed;           // Has been homed
    int32_t         homing_velocity;    // Fast homing velocity
    int32_t         homing_velocity_slow; // Slow homing velocity
    int32_t         homing_backoff;     // Backoff distance after limit
    
    // Enable state
    bool            enabled;            // Motor is enabled
    bool            step_active;        // Step pulse is currently high
    
    // Statistics
    uint32_t        total_steps;        // Total steps executed
    uint32_t        error_count;        // Error counter
} stepper_context_t;

// =============================================================================
// Initialization
// =============================================================================

/**
 * @brief Initialize stepper control
 * @param ctx Stepper context
 */
void stepper_init(stepper_context_t* ctx);

/**
 * @brief Configure motion parameters
 * @param ctx Stepper context
 * @param max_velocity Maximum velocity (steps/s)
 * @param acceleration Acceleration (steps/s^2)
 * @param deceleration Deceleration (steps/s^2)
 */
void stepper_set_params(stepper_context_t* ctx, 
                        int32_t max_velocity,
                        int32_t acceleration, 
                        int32_t deceleration);

/**
 * @brief Configure homing parameters
 * @param ctx Stepper context
 * @param velocity Fast homing velocity
 * @param velocity_slow Slow homing velocity
 * @param backoff Backoff distance
 */
void stepper_set_homing_params(stepper_context_t* ctx,
                               int32_t velocity,
                               int32_t velocity_slow,
                               int32_t backoff);

// =============================================================================
// Motion Control
// =============================================================================

/**
 * @brief Enable motor driver
 * @param ctx Stepper context
 * @param enable Enable state
 */
void stepper_enable(stepper_context_t* ctx, bool enable);

/**
 * @brief Move to absolute position
 * @param ctx Stepper context
 * @param position Target position in steps
 * @return true if move started successfully
 */
bool stepper_move_to(stepper_context_t* ctx, int32_t position);

/**
 * @brief Move by relative distance
 * @param ctx Stepper context
 * @param distance Distance in steps (signed)
 * @return true if move started successfully
 */
bool stepper_move_by(stepper_context_t* ctx, int32_t distance);

/**
 * @brief Set target velocity for velocity mode
 * @param ctx Stepper context
 * @param velocity Target velocity (steps/s, signed for direction)
 */
void stepper_set_velocity(stepper_context_t* ctx, int32_t velocity);

/**
 * @brief Start homing sequence
 * @param ctx Stepper context
 * @param direction Homing direction (+1 or -1)
 * @return true if homing started
 */
bool stepper_home(stepper_context_t* ctx, int8_t direction);

/**
 * @brief Stop motion with deceleration
 * @param ctx Stepper context
 */
void stepper_stop(stepper_context_t* ctx);

/**
 * @brief Emergency stop (immediate)
 * @param ctx Stepper context
 */
void stepper_estop(stepper_context_t* ctx);

/**
 * @brief Set current position (for homing)
 * @param ctx Stepper context
 * @param position New position value
 */
void stepper_set_position(stepper_context_t* ctx, int32_t position);

// =============================================================================
// Status
// =============================================================================

/**
 * @brief Check if motion is complete
 * @param ctx Stepper context
 * @return true if stopped at target
 */
bool stepper_is_done(const stepper_context_t* ctx);

/**
 * @brief Check if motor is moving
 * @param ctx Stepper context
 * @return true if in motion
 */
bool stepper_is_moving(const stepper_context_t* ctx);

/**
 * @brief Check if motor is homed
 * @param ctx Stepper context
 * @return true if homed
 */
bool stepper_is_homed(const stepper_context_t* ctx);

/**
 * @brief Get current position
 * @param ctx Stepper context
 * @return Current position in steps
 */
int32_t stepper_get_position(const stepper_context_t* ctx);

/**
 * @brief Get current velocity
 * @param ctx Stepper context
 * @return Current velocity in steps/s
 */
int32_t stepper_get_velocity(const stepper_context_t* ctx);

/**
 * @brief Get following error
 * @param ctx Stepper context
 * @return Difference between target and actual position
 */
int32_t stepper_get_following_error(const stepper_context_t* ctx);

// =============================================================================
// Limit Switches
// =============================================================================

/**
 * @brief Update limit switch states
 * @param ctx Stepper context
 * @param limit_min Min limit switch state
 * @param limit_max Max limit switch state
 */
void stepper_update_limits(stepper_context_t* ctx, bool limit_min, bool limit_max);

/**
 * @brief Check if at limit
 * @param ctx Stepper context
 * @return Limit switch mask
 */
uint8_t stepper_get_limits(const stepper_context_t* ctx);

// =============================================================================
// Timer Interrupt Handler
// =============================================================================

/**
 * @brief Timer interrupt handler for step generation
 * 
 * Call from TIM2 IRQ handler. Generates step pulses and updates velocity.
 * 
 * @param ctx Stepper context
 */
void stepper_timer_irq(stepper_context_t* ctx);

/**
 * @brief 1ms tick for velocity calculations
 * @param ctx Stepper context
 */
void stepper_1ms_tick(stepper_context_t* ctx);

// =============================================================================
// Hardware Interface (to be implemented per platform)
// =============================================================================

/**
 * @brief Set step pin state
 * @param state Pin state (true = high)
 */
void stepper_hw_set_step(bool state);

/**
 * @brief Set direction pin state
 * @param state Pin state (true = positive direction)
 */
void stepper_hw_set_dir(bool state);

/**
 * @brief Set enable pin state
 * @param state Pin state (true = enabled)
 */
void stepper_hw_set_enable(bool state);

/**
 * @brief Set timer period for step timing
 * @param period Timer period in ticks
 */
void stepper_hw_set_period(uint32_t period);

/**
 * @brief Start step timer
 */
void stepper_hw_start_timer(void);

/**
 * @brief Stop step timer
 */
void stepper_hw_stop_timer(void);

#ifdef __cplusplus
}
#endif

#endif // STEPPER_CONTROL_H
