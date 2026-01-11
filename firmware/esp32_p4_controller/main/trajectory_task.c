/**
 * @file trajectory_task.c
 * @brief Trajectory Planning Implementation
 */

#include "trajectory_task.h"
#include "canopen_master.h"
#include <string.h>
#include <math.h>
#include "esp_log.h"

static const char *TAG = "trajectory";

// Global trajectory context
static traj_context_t g_traj = {0};

// =============================================================================
// Initialization
// =============================================================================

void trajectory_task_init(void)
{
    ESP_LOGI(TAG, "Initializing trajectory planner");
    
    memset(&g_traj, 0, sizeof(g_traj));
    g_traj.state = TRAJ_STATE_IDLE;
}

// =============================================================================
// Buffer Management
// =============================================================================

void trajectory_task_clear(void)
{
    g_traj.head = 0;
    g_traj.tail = 0;
    g_traj.count = 0;
    g_traj.current_segment = 0;
    g_traj.segment_progress = 0;
    g_traj.state = TRAJ_STATE_IDLE;
    
    ESP_LOGI(TAG, "Buffer cleared");
}

bool trajectory_task_add_segment(const traj_segment_t* segment)
{
    if (g_traj.count >= TRAJ_BUFFER_SIZE) {
        return false;
    }
    
    g_traj.buffer[g_traj.head] = *segment;
    g_traj.head = (g_traj.head + 1) % TRAJ_BUFFER_SIZE;
    g_traj.count++;
    
    return true;
}

uint16_t trajectory_task_get_buffer_space(void)
{
    return TRAJ_BUFFER_SIZE - g_traj.count;
}

// =============================================================================
// Execution Control
// =============================================================================

void trajectory_task_start(void)
{
    if (g_traj.count == 0) {
        ESP_LOGW(TAG, "Cannot start: buffer empty");
        return;
    }
    
    g_traj.state = TRAJ_STATE_RUNNING;
    g_traj.current_segment = g_traj.tail;
    g_traj.segment_progress = 0;
    g_traj.segment_start_tick = 0;  // Will be set on first tick
    
    ESP_LOGI(TAG, "Trajectory started, %d segments", g_traj.count);
}

void trajectory_task_pause(void)
{
    if (g_traj.state == TRAJ_STATE_RUNNING) {
        g_traj.state = TRAJ_STATE_PAUSED;
        ESP_LOGI(TAG, "Trajectory paused");
    }
}

void trajectory_task_resume(void)
{
    if (g_traj.state == TRAJ_STATE_PAUSED) {
        g_traj.state = TRAJ_STATE_RUNNING;
        ESP_LOGI(TAG, "Trajectory resumed");
    }
}

traj_state_t trajectory_task_get_state(void)
{
    return g_traj.state;
}

// =============================================================================
// Linear Interpolation
// =============================================================================

static void interpolate_linear(const traj_segment_t* seg, uint32_t progress)
{
    // progress is 0-1000000 (millionths)
    for (int i = 0; i < TRAJ_MAX_AXES; i++) {
        if (seg->axis_mask & (1 << i)) {
            int64_t delta = seg->end[i] - seg->start[i];
            g_traj.position[i] = seg->start[i] + (delta * progress) / 1000000;
        }
    }
}

// =============================================================================
// S-Curve Velocity Profile
// =============================================================================

static uint32_t calculate_s_curve_position(
    uint32_t progress,          // 0-1000000
    uint32_t duration_us,
    uint32_t entry_vel,
    uint32_t cruise_vel,
    uint32_t exit_vel,
    uint32_t accel
)
{
    // Simplified trapezoidal profile for now
    // TODO: Implement true S-curve with jerk limiting
    
    uint32_t accel_time = 0;
    uint32_t decel_time = 0;
    uint32_t cruise_time = 0;
    
    // Calculate acceleration/deceleration times
    if (cruise_vel > entry_vel && accel > 0) {
        accel_time = ((cruise_vel - entry_vel) * 1000000ULL) / accel;
    }
    if (cruise_vel > exit_vel && accel > 0) {
        decel_time = ((cruise_vel - exit_vel) * 1000000ULL) / accel;
    }
    
    uint32_t total_time = duration_us;
    cruise_time = total_time - accel_time - decel_time;
    if ((int32_t)cruise_time < 0) cruise_time = 0;
    
    // Calculate position based on profile phase
    uint32_t elapsed = (progress * total_time) / 1000000;
    
    if (elapsed < accel_time) {
        // Acceleration phase
        return (progress * progress) / 2000000;  // Simplified
    } else if (elapsed < accel_time + cruise_time) {
        // Cruise phase
        return progress;  // Simplified linear
    } else {
        // Deceleration phase
        return 1000000 - ((1000000 - progress) * (1000000 - progress)) / 2000000;
    }
}

// =============================================================================
// Tick Processing
// =============================================================================

void trajectory_task_tick(void)
{
    if (g_traj.state != TRAJ_STATE_RUNNING) {
        return;
    }
    
    if (g_traj.count == 0) {
        g_traj.state = TRAJ_STATE_COMPLETE;
        ESP_LOGI(TAG, "Trajectory complete, %lu segments executed", g_traj.segments_executed);
        return;
    }
    
    // Get current segment
    traj_segment_t* seg = &g_traj.buffer[g_traj.current_segment];
    
    // Handle dwell
    if (seg->type == TRAJ_TYPE_DWELL) {
        g_traj.segment_progress += 1000;  // 1ms per tick at 1kHz
        if (g_traj.segment_progress >= seg->dwell_ms * 1000) {
            // Advance to next segment
            goto next_segment;
        }
        return;
    }
    
    // Calculate progress for motion segment
    // duration_us at 1kHz = 1000us per tick
    uint32_t tick_progress = 1000000 / (seg->duration_us / 1000);  // Progress per tick
    g_traj.segment_progress += tick_progress;
    
    if (g_traj.segment_progress > 1000000) {
        g_traj.segment_progress = 1000000;
    }
    
    // Interpolate position based on segment type
    switch (seg->type) {
        case TRAJ_TYPE_LINEAR:
        case TRAJ_TYPE_RAPID:
            interpolate_linear(seg, g_traj.segment_progress);
            break;
            
        case TRAJ_TYPE_ARC_CW:
        case TRAJ_TYPE_ARC_CCW:
            // TODO: Implement arc interpolation
            interpolate_linear(seg, g_traj.segment_progress);
            break;
            
        default:
            break;
    }
    
    // Update axis targets
    for (int i = 0; i < TRAJ_MAX_AXES; i++) {
        if (seg->axis_mask & (1 << i)) {
            canopen_master_set_position(i + 1, g_traj.position[i]);
        }
    }
    
    // Check segment completion
    if (g_traj.segment_progress >= 1000000) {
next_segment:
        g_traj.segments_executed++;
        g_traj.tail = (g_traj.tail + 1) % TRAJ_BUFFER_SIZE;
        g_traj.count--;
        g_traj.current_segment = g_traj.tail;
        g_traj.segment_progress = 0;
        
        // Copy end position to start of next segment if needed
        if (g_traj.count > 0) {
            traj_segment_t* next = &g_traj.buffer[g_traj.current_segment];
            for (int i = 0; i < TRAJ_MAX_AXES; i++) {
                next->start[i] = g_traj.position[i];
            }
        }
    }
}

// =============================================================================
// Position Access
// =============================================================================

void trajectory_task_get_positions(int32_t* positions)
{
    memcpy(positions, g_traj.position, sizeof(g_traj.position));
}

void trajectory_task_set_position(uint8_t axis, int32_t position)
{
    if (axis < TRAJ_MAX_AXES) {
        g_traj.position[axis] = position;
    }
}

void trajectory_task_get_stats(uint32_t* executed, uint32_t* underruns)
{
    if (executed) *executed = g_traj.segments_executed;
    if (underruns) *underruns = g_traj.underruns;
}
