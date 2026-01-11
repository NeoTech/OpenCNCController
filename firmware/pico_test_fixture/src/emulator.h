/**
 * @file emulator.h
 * @brief CNC firmware emulator - state machine and motion simulation
 */

#ifndef EMULATOR_H
#define EMULATOR_H

#include <stdint.h>
#include <stdbool.h>
#include "test_fixture.h"

#ifdef __cplusplus
extern "C" {
#endif

// =============================================================================
// Motion Segment (simplified from cnc_protocol.h)
// =============================================================================

typedef struct {
    uint8_t     motion_type;        // 0=rapid, 1=linear, 2=arc_cw, 3=arc_ccw
    uint8_t     flags;
    int32_t     target_nm[MAX_AXES];
    uint32_t    feed_rate;          // mm/min * 1000
    uint32_t    duration_us;        // Computed duration
    bool        in_use;
} motion_segment_t;

// =============================================================================
// Emulator Configuration
// =============================================================================

// Default machine parameters
#define EMU_DEFAULT_MAX_VELOCITY    5000    // mm/min
#define EMU_DEFAULT_ACCELERATION    500     // mm/s²
#define EMU_DEFAULT_TRAVEL_X        300     // mm
#define EMU_DEFAULT_TRAVEL_Y        200     // mm  
#define EMU_DEFAULT_TRAVEL_Z        100     // mm
#define EMU_JOG_VELOCITY            1000    // mm/min

// Homing simulation
#define EMU_HOMING_VELOCITY         500     // mm/min
#define EMU_HOMING_DURATION_MS      3000    // Time to complete homing per axis

// =============================================================================
// Function Prototypes
// =============================================================================

/**
 * @brief Initialize emulator to default state
 */
void emulator_init(void);

/**
 * @brief Reset emulator to initial state (like power-on)
 */
void emulator_reset(void);

/**
 * @brief Tick the emulator (call at EMULATOR_TICK_HZ)
 * Updates positions, processes motion queue
 */
void emulator_tick(void);

/**
 * @brief Start motion execution
 */
void emulator_start(void);

/**
 * @brief Pause motion execution
 */
void emulator_pause(void);

/**
 * @brief Resume motion execution
 */
void emulator_resume(void);

/**
 * @brief Stop motion execution (controlled stop)
 */
void emulator_stop(void);

/**
 * @brief Emergency stop
 */
void emulator_estop(void);

/**
 * @brief Clear E-stop condition
 */
void emulator_clear_estop(void);

/**
 * @brief Start homing sequence
 * @param axis Axis index (0=X, 1=Y, 2=Z) or -1 for all
 */
void emulator_start_homing(int8_t axis);

/**
 * @brief Start jogging an axis
 * @param axis Axis index
 * @param velocity_nm_s Velocity in nm/sec (signed for direction)
 */
void emulator_jog_start(uint8_t axis, int32_t velocity_nm_s);

/**
 * @brief Stop jogging
 */
void emulator_jog_stop(void);

/**
 * @brief Add motion segment to queue
 * @param payload Raw payload from protocol
 * @param len Payload length
 * @return true if queued successfully
 */
bool emulator_queue_motion(const uint8_t* payload, uint8_t len);

/**
 * @brief Clear motion queue
 */
void emulator_clear_queue(void);

/**
 * @brief Inject a fault/alarm (for testing)
 * @param alarm Alarm code to inject
 */
void emulator_inject_alarm(alarm_code_t alarm);

/**
 * @brief Clear all alarms
 */
void emulator_clear_alarms(void);

/**
 * @brief Cycle through machine states (for button control)
 */
void emulator_cycle_state(void);

/**
 * @brief Get current state name string
 * @return State name for display
 */
const char* emulator_state_name(void);

/**
 * @brief Get current alarm name string
 * @return Alarm name for display
 */
const char* emulator_alarm_name(void);

#ifdef __cplusplus
}
#endif

#endif // EMULATOR_H
