/**
 * @file stepper_control.c
 * @brief Stepper Motor Control Implementation
 */

#include "stepper_control.h"
#include "main.h"
#include <stdlib.h>

// External timer handle
extern TIM_HandleTypeDef htim2;

// =============================================================================
// Helper Macros
// =============================================================================

#define ABS(x)    ((x) < 0 ? -(x) : (x))
#define SIGN(x)   ((x) > 0 ? 1 : ((x) < 0 ? -1 : 0))
#define MIN(a,b)  ((a) < (b) ? (a) : (b))
#define MAX(a,b)  ((a) > (b) ? (a) : (b))

// =============================================================================
// Initialization
// =============================================================================

void stepper_init(stepper_context_t* ctx)
{
    ctx->state = STEPPER_STATE_IDLE;
    ctx->position = 0;
    ctx->target_position = 0;
    ctx->home_offset = 0;
    ctx->current_velocity = 0;
    ctx->target_velocity = 0;
    ctx->max_velocity = DEFAULT_MAX_VELOCITY;
    ctx->acceleration = DEFAULT_ACCELERATION;
    ctx->deceleration = DEFAULT_ACCELERATION;
    ctx->direction = 1;
    ctx->step_period = 0;
    ctx->step_count = 0;
    ctx->steps_to_go = 0;
    ctx->limit_state = STEPPER_LIMIT_NONE;
    ctx->limit_inverted = true;  // Most switches are NC (active low)
    ctx->is_homed = false;
    ctx->homing_velocity = 10000;
    ctx->homing_velocity_slow = 1000;
    ctx->homing_backoff = 200;
    ctx->enabled = false;
    ctx->step_active = false;
    ctx->total_steps = 0;
    ctx->error_count = 0;
    
    // Ensure motor is disabled
    stepper_hw_set_enable(false);
    stepper_hw_set_step(false);
    stepper_hw_set_dir(true);
}

void stepper_set_params(stepper_context_t* ctx, 
                        int32_t max_velocity,
                        int32_t acceleration, 
                        int32_t deceleration)
{
    ctx->max_velocity = max_velocity;
    ctx->acceleration = acceleration;
    ctx->deceleration = deceleration;
}

void stepper_set_homing_params(stepper_context_t* ctx,
                               int32_t velocity,
                               int32_t velocity_slow,
                               int32_t backoff)
{
    ctx->homing_velocity = velocity;
    ctx->homing_velocity_slow = velocity_slow;
    ctx->homing_backoff = backoff;
}

// =============================================================================
// Motion Control
// =============================================================================

void stepper_enable(stepper_context_t* ctx, bool enable)
{
    ctx->enabled = enable;
    stepper_hw_set_enable(enable);
}

static void start_move(stepper_context_t* ctx)
{
    int32_t distance = ctx->target_position - ctx->position;
    
    if (distance == 0) {
        ctx->state = STEPPER_STATE_IDLE;
        return;
    }
    
    ctx->direction = (distance > 0) ? 1 : -1;
    ctx->steps_to_go = ABS(distance);
    ctx->step_count = 0;
    
    // Calculate deceleration distance
    // d = v^2 / (2 * a)
    ctx->decel_distance = (ctx->max_velocity * ctx->max_velocity) / 
                          (2 * ctx->deceleration);
    
    stepper_hw_set_dir(ctx->direction > 0);
    ctx->state = STEPPER_STATE_ACCELERATING;
    
    // Start with slow step rate
    ctx->current_velocity = ctx->acceleration / 100;  // Initial velocity
    if (ctx->current_velocity < 100) ctx->current_velocity = 100;
    
    ctx->step_period = STEPPER_TIMER_FREQ / ctx->current_velocity;
    stepper_hw_set_period(ctx->step_period);
    stepper_hw_start_timer();
}

bool stepper_move_to(stepper_context_t* ctx, int32_t position)
{
    if (!ctx->enabled) return false;
    if (ctx->state != STEPPER_STATE_IDLE) return false;
    
    ctx->target_position = position;
    start_move(ctx);
    return true;
}

bool stepper_move_by(stepper_context_t* ctx, int32_t distance)
{
    return stepper_move_to(ctx, ctx->position + distance);
}

void stepper_set_velocity(stepper_context_t* ctx, int32_t velocity)
{
    if (!ctx->enabled) return;
    
    ctx->target_velocity = ABS(velocity);
    
    if (velocity == 0) {
        stepper_stop(ctx);
        return;
    }
    
    ctx->direction = SIGN(velocity);
    stepper_hw_set_dir(ctx->direction > 0);
    
    if (ctx->state == STEPPER_STATE_IDLE) {
        ctx->steps_to_go = 0x7FFFFFFF;  // Continuous
        ctx->step_count = 0;
        ctx->current_velocity = ctx->acceleration / 100;
        if (ctx->current_velocity < 100) ctx->current_velocity = 100;
        
        ctx->step_period = STEPPER_TIMER_FREQ / ctx->current_velocity;
        stepper_hw_set_period(ctx->step_period);
        stepper_hw_start_timer();
        ctx->state = STEPPER_STATE_ACCELERATING;
    }
}

bool stepper_home(stepper_context_t* ctx, int8_t direction)
{
    if (!ctx->enabled) return false;
    if (ctx->state != STEPPER_STATE_IDLE) return false;
    
    ctx->is_homed = false;
    ctx->direction = direction;
    ctx->steps_to_go = 0x7FFFFFFF;  // Move until limit
    
    stepper_hw_set_dir(direction > 0);
    ctx->current_velocity = ctx->acceleration / 100;
    if (ctx->current_velocity < 100) ctx->current_velocity = 100;
    
    ctx->target_velocity = ctx->homing_velocity;
    ctx->step_period = STEPPER_TIMER_FREQ / ctx->current_velocity;
    stepper_hw_set_period(ctx->step_period);
    stepper_hw_start_timer();
    ctx->state = STEPPER_STATE_HOMING_FAST;
    
    return true;
}

void stepper_stop(stepper_context_t* ctx)
{
    if (ctx->state == STEPPER_STATE_IDLE) return;
    
    ctx->state = STEPPER_STATE_DECELERATING;
    ctx->target_position = ctx->position;
}

void stepper_estop(stepper_context_t* ctx)
{
    stepper_hw_stop_timer();
    stepper_hw_set_step(false);
    ctx->state = STEPPER_STATE_STOPPING;
    ctx->current_velocity = 0;
    ctx->step_active = false;
}

void stepper_set_position(stepper_context_t* ctx, int32_t position)
{
    ctx->position = position;
    ctx->target_position = position;
}

// =============================================================================
// Status
// =============================================================================

bool stepper_is_done(const stepper_context_t* ctx)
{
    return ctx->state == STEPPER_STATE_IDLE;
}

bool stepper_is_moving(const stepper_context_t* ctx)
{
    return ctx->state != STEPPER_STATE_IDLE && 
           ctx->state != STEPPER_STATE_STOPPING;
}

bool stepper_is_homed(const stepper_context_t* ctx)
{
    return ctx->is_homed;
}

int32_t stepper_get_position(const stepper_context_t* ctx)
{
    return ctx->position;
}

int32_t stepper_get_velocity(const stepper_context_t* ctx)
{
    return ctx->current_velocity * ctx->direction;
}

int32_t stepper_get_following_error(const stepper_context_t* ctx)
{
    return ctx->target_position - ctx->position;
}

// =============================================================================
// Limit Switches
// =============================================================================

void stepper_update_limits(stepper_context_t* ctx, bool limit_min, bool limit_max)
{
    uint8_t prev_state = ctx->limit_state;
    
    ctx->limit_state = 0;
    if (limit_min) ctx->limit_state |= STEPPER_LIMIT_MIN;
    if (limit_max) ctx->limit_state |= STEPPER_LIMIT_MAX;
    
    // Check for limit hit during motion
    if (ctx->state == STEPPER_STATE_HOMING_FAST) {
        if ((ctx->direction < 0 && limit_min) || (ctx->direction > 0 && limit_max)) {
            // Hit limit during fast homing - back off
            stepper_estop(ctx);
            ctx->direction = -ctx->direction;
            stepper_hw_set_dir(ctx->direction > 0);
            ctx->target_velocity = ctx->homing_velocity_slow;
            ctx->steps_to_go = ctx->homing_backoff;
            ctx->current_velocity = 100;
            ctx->step_period = STEPPER_TIMER_FREQ / ctx->current_velocity;
            stepper_hw_set_period(ctx->step_period);
            stepper_hw_start_timer();
            ctx->state = STEPPER_STATE_HOMING_BACKOFF;
        }
    } else if (ctx->state == STEPPER_STATE_HOMING_BACKOFF) {
        if (ctx->steps_to_go == 0) {
            // Done backing off - approach slowly
            ctx->direction = -ctx->direction;
            stepper_hw_set_dir(ctx->direction > 0);
            ctx->target_velocity = ctx->homing_velocity_slow;
            ctx->steps_to_go = 0x7FFFFFFF;
            ctx->state = STEPPER_STATE_HOMING_SLOW;
        }
    } else if (ctx->state == STEPPER_STATE_HOMING_SLOW) {
        if ((ctx->direction < 0 && limit_min) || (ctx->direction > 0 && limit_max)) {
            // Found home position
            stepper_estop(ctx);
            ctx->position = 0;
            ctx->target_position = 0;
            ctx->is_homed = true;
            ctx->state = STEPPER_STATE_IDLE;
        }
    } else if (stepper_is_moving(ctx)) {
        // Normal motion - stop if limit in direction of travel
        if ((ctx->direction < 0 && limit_min) || (ctx->direction > 0 && limit_max)) {
            stepper_estop(ctx);
            ctx->error_count++;
        }
    }
}

uint8_t stepper_get_limits(const stepper_context_t* ctx)
{
    return ctx->limit_state;
}

// =============================================================================
// Timer Interrupt Handler
// =============================================================================

void stepper_timer_irq(stepper_context_t* ctx)
{
    if (ctx->state == STEPPER_STATE_IDLE || ctx->state == STEPPER_STATE_STOPPING) {
        return;
    }
    
    if (ctx->step_active) {
        // End step pulse
        stepper_hw_set_step(false);
        ctx->step_active = false;
        return;
    }
    
    // Generate step pulse
    stepper_hw_set_step(true);
    ctx->step_active = true;
    ctx->position += ctx->direction;
    ctx->total_steps++;
    
    if (ctx->steps_to_go > 0) {
        ctx->steps_to_go--;
        ctx->step_count++;
    }
    
    // Update velocity based on state
    switch (ctx->state) {
        case STEPPER_STATE_ACCELERATING:
            if (ctx->current_velocity < ctx->target_velocity) {
                // Simple linear acceleration approximation
                ctx->current_velocity += ctx->acceleration / (ctx->current_velocity + 100);
                if (ctx->current_velocity > ctx->target_velocity) {
                    ctx->current_velocity = ctx->target_velocity;
                }
            }
            if (ctx->current_velocity >= ctx->target_velocity) {
                ctx->state = STEPPER_STATE_CRUISING;
            }
            // Check if we need to start decelerating
            if (ctx->steps_to_go <= ctx->decel_distance) {
                ctx->state = STEPPER_STATE_DECELERATING;
            }
            break;
            
        case STEPPER_STATE_CRUISING:
            // Check if we need to start decelerating
            if (ctx->steps_to_go <= ctx->decel_distance) {
                ctx->state = STEPPER_STATE_DECELERATING;
            }
            break;
            
        case STEPPER_STATE_DECELERATING:
            if (ctx->current_velocity > 100) {
                ctx->current_velocity -= ctx->deceleration / (ctx->current_velocity + 100);
                if (ctx->current_velocity < 100) {
                    ctx->current_velocity = 100;
                }
            }
            if (ctx->steps_to_go == 0) {
                stepper_hw_stop_timer();
                ctx->state = STEPPER_STATE_IDLE;
                ctx->current_velocity = 0;
            }
            break;
            
        case STEPPER_STATE_HOMING_FAST:
        case STEPPER_STATE_HOMING_SLOW:
            // Accelerate to homing velocity
            if (ctx->current_velocity < ctx->target_velocity) {
                ctx->current_velocity += ctx->acceleration / (ctx->current_velocity + 100);
                if (ctx->current_velocity > ctx->target_velocity) {
                    ctx->current_velocity = ctx->target_velocity;
                }
            }
            break;
            
        case STEPPER_STATE_HOMING_BACKOFF:
            // Constant velocity backoff
            if (ctx->steps_to_go == 0) {
                stepper_update_limits(ctx, 
                    ctx->limit_state & STEPPER_LIMIT_MIN,
                    ctx->limit_state & STEPPER_LIMIT_MAX);
            }
            break;
            
        default:
            break;
    }
    
    // Update timer period based on velocity
    if (ctx->current_velocity > 0) {
        ctx->step_period = STEPPER_TIMER_FREQ / ctx->current_velocity;
        if (ctx->step_period < STEPPER_MIN_PERIOD) {
            ctx->step_period = STEPPER_MIN_PERIOD;
        }
        stepper_hw_set_period(ctx->step_period);
    }
}

void stepper_1ms_tick(stepper_context_t* ctx)
{
    // For velocity mode - update target based on input
    // This is called from main loop at 1kHz
}

// =============================================================================
// Hardware Interface (STM32G4 Implementation)
// =============================================================================

void stepper_hw_set_step(bool state)
{
    HAL_GPIO_WritePin(STEP_GPIO_Port, STEP_Pin, state ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void stepper_hw_set_dir(bool state)
{
    HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, state ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void stepper_hw_set_enable(bool state)
{
    // Enable is typically active low for drivers
    HAL_GPIO_WritePin(ENABLE_GPIO_Port, ENABLE_Pin, state ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

void stepper_hw_set_period(uint32_t period)
{
    __HAL_TIM_SET_AUTORELOAD(&htim2, period - 1);
}

void stepper_hw_start_timer(void)
{
    HAL_TIM_Base_Start_IT(&htim2);
}

void stepper_hw_stop_timer(void)
{
    HAL_TIM_Base_Stop_IT(&htim2);
}
