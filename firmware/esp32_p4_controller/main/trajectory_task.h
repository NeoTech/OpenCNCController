/**
 * @file trajectory_task.h
 * @brief Trajectory Planning and Execution
 * 
 * Handles motion trajectory buffering, interpolation, and execution.
 * Generates position targets for SYNC-timed PDO transmission.
 */

#ifndef TRAJECTORY_TASK_H
#define TRAJECTORY_TASK_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// =============================================================================
// Configuration
// =============================================================================

#define TRAJ_BUFFER_SIZE            256     // Number of segments
#define TRAJ_MAX_AXES               9       // Maximum supported axes

// =============================================================================
// Segment Types
// =============================================================================

typedef enum {
    TRAJ_TYPE_LINEAR,           // Linear interpolation
    TRAJ_TYPE_RAPID,            // Maximum velocity move
    TRAJ_TYPE_ARC_CW,           // Clockwise arc (XY plane)
    TRAJ_TYPE_ARC_CCW,          // Counter-clockwise arc
    TRAJ_TYPE_DWELL,            // Pause for specified time
} traj_type_t;

// =============================================================================
// Motion Segment
// =============================================================================

typedef struct {
    traj_type_t type;
    uint8_t     axis_mask;      // Which axes are involved
    
    // Start/end positions (in steps)
    int32_t     start[TRAJ_MAX_AXES];
    int32_t     end[TRAJ_MAX_AXES];
    
    // For arcs: center point and radius
    int32_t     center[2];      // XY center
    int32_t     radius;
    
    // Velocity profile
    uint32_t    entry_velocity;     // Steps/sec
    uint32_t    cruise_velocity;    // Steps/sec
    uint32_t    exit_velocity;      // Steps/sec
    uint32_t    acceleration;       // Steps/sec^2
    
    // Duration
    uint32_t    duration_us;        // Total segment time
    
    // Dwell
    uint32_t    dwell_ms;
    
} traj_segment_t;

// =============================================================================
// Trajectory State
// =============================================================================

typedef enum {
    TRAJ_STATE_IDLE,
    TRAJ_STATE_RUNNING,
    TRAJ_STATE_PAUSED,
    TRAJ_STATE_COMPLETE,
    TRAJ_STATE_ERROR,
} traj_state_t;

typedef struct {
    traj_state_t    state;
    
    // Circular buffer
    traj_segment_t  buffer[TRAJ_BUFFER_SIZE];
    uint16_t        head;           // Next write position
    uint16_t        tail;           // Next read position
    uint16_t        count;          // Segments in buffer
    
    // Current segment execution
    uint16_t        current_segment;
    uint32_t        segment_progress;   // 0-1000000 (millionths)
    uint32_t        segment_start_tick;
    
    // Current interpolated positions
    int32_t         position[TRAJ_MAX_AXES];
    int32_t         velocity[TRAJ_MAX_AXES];
    
    // Statistics
    uint32_t        segments_executed;
    uint32_t        underruns;      // Buffer underrun count
    
} traj_context_t;

// =============================================================================
// Public Functions
// =============================================================================

/**
 * @brief Initialize trajectory planner
 */
void trajectory_task_init(void);

/**
 * @brief Process one tick (call at SYNC rate, e.g., 1kHz)
 */
void trajectory_task_tick(void);

/**
 * @brief Clear all segments from buffer
 */
void trajectory_task_clear(void);

/**
 * @brief Add segment to buffer
 * @param segment Segment to add
 * @return true if added, false if buffer full
 */
bool trajectory_task_add_segment(const traj_segment_t* segment);

/**
 * @brief Get buffer space available
 * @return Number of segments that can be added
 */
uint16_t trajectory_task_get_buffer_space(void);

/**
 * @brief Start trajectory execution
 */
void trajectory_task_start(void);

/**
 * @brief Pause trajectory execution
 */
void trajectory_task_pause(void);

/**
 * @brief Resume trajectory execution
 */
void trajectory_task_resume(void);

/**
 * @brief Get current state
 */
traj_state_t trajectory_task_get_state(void);

/**
 * @brief Get current interpolated position
 * @param positions Output array (9 elements)
 */
void trajectory_task_get_positions(int32_t* positions);

/**
 * @brief Set current position (for homing/manual positioning)
 * @param axis Axis index (0-8)
 * @param position New position
 */
void trajectory_task_set_position(uint8_t axis, int32_t position);

/**
 * @brief Get execution statistics
 */
void trajectory_task_get_stats(uint32_t* executed, uint32_t* underruns);

#ifdef __cplusplus
}
#endif

#endif // TRAJECTORY_TASK_H
