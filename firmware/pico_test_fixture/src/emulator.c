/**
 * @file emulator.c
 * @brief CNC firmware emulator - state machine and motion simulation
 */

#include "emulator.h"
#include "display.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

// =============================================================================
// Motion Queue
// =============================================================================

static motion_segment_t s_motion_queue[MOTION_QUEUE_SIZE];
static uint8_t s_queue_head = 0;
static uint8_t s_queue_tail = 0;
static motion_segment_t s_current_segment;
static uint32_t s_segment_elapsed_us = 0;
static bool s_segment_active = false;

// =============================================================================
// Homing State
// =============================================================================

static int8_t s_homing_axis = -1;
static uint32_t s_homing_start_time = 0;

// =============================================================================
// Jogging State
// =============================================================================

static bool s_jog_active = false;
static uint8_t s_jog_axis = 0;
static int32_t s_jog_velocity = 0;

// =============================================================================
// State/Alarm Names
// =============================================================================

static const char* const STATE_NAMES[] = {
    "IDLE",         // 0
    "RUNNING",      // 1
    "PAUSED",       // 2
    "HOMING",       // 3
    "PROBING",      // 4
    "JOG",          // 5
    "UNKNOWN_6",
    "UNKNOWN_7",
    "UNKNOWN_8",
    "UNKNOWN_9",
    "UNKNOWN_10",
    "UNKNOWN_11",
    "UNKNOWN_12",
    "UNKNOWN_13",
    "ALARM",        // 14
    "ESTOP"         // 15
};

static const char* const ALARM_NAMES[] = {
    "NONE",
    "LIMIT_X",
    "LIMIT_Y",
    "LIMIT_Z",
    "SOFT_LIMIT",
    "ESTOP",
    "PROBE_FAIL",
    "HOME_FAIL",
    "SPINDLE",
    "COMM_ERROR"
};

// =============================================================================
// Initialization
// =============================================================================

void emulator_init(void) {
    emulator_reset();
}

void emulator_reset(void) {
    // Reset state machine
    g_fixture.machine.state = STATE_IDLE;
    g_fixture.machine.alarm = ALARM_NONE;
    
    // Reset positions
    for (int i = 0; i < MAX_AXES; i++) {
        g_fixture.machine.position_nm[i] = 0;
        g_fixture.machine.target_nm[i] = 0;
        g_fixture.machine.velocity_nm_s[i] = 0;
    }
    
    // Reset queue
    emulator_clear_queue();
    
    // Reset motion state
    g_fixture.machine.in_motion = false;
    g_fixture.machine.homing_active = false;
    g_fixture.machine.probe_active = false;
    g_fixture.machine.current_feed_rate = 0;
    
    // Reset overrides to 100%
    g_fixture.machine.feed_override = 1000;
    g_fixture.machine.rapid_override = 1000;
    g_fixture.machine.spindle_override = 1000;
    
    // Reset spindle/coolant
    g_fixture.machine.spindle_on = false;
    g_fixture.machine.spindle_cw = true;
    g_fixture.machine.spindle_rpm = 0;
    g_fixture.machine.coolant_flood = false;
    g_fixture.machine.coolant_mist = false;
    
    // Reset limits
    g_fixture.machine.limit_switches = 0;
    
    // Reset internal state
    s_segment_active = false;
    s_jog_active = false;
    s_homing_axis = -1;
}

// =============================================================================
// State Machine Control
// =============================================================================

void emulator_start(void) {
    if (g_fixture.machine.state == STATE_IDLE || 
        g_fixture.machine.state == STATE_PAUSED) {
        g_fixture.machine.state = STATE_RUNNING;
        display_log("Motion started");
    }
}

void emulator_pause(void) {
    if (g_fixture.machine.state == STATE_RUNNING) {
        g_fixture.machine.state = STATE_PAUSED;
        display_log("Motion paused");
    }
}

void emulator_resume(void) {
    if (g_fixture.machine.state == STATE_PAUSED) {
        g_fixture.machine.state = STATE_RUNNING;
        display_log("Motion resumed");
    }
}

void emulator_stop(void) {
    g_fixture.machine.state = STATE_IDLE;
    g_fixture.machine.in_motion = false;
    s_segment_active = false;
    s_jog_active = false;
    display_log("Motion stopped");
}

void emulator_estop(void) {
    g_fixture.machine.state = STATE_ESTOP;
    g_fixture.machine.alarm = ALARM_ESTOP;
    g_fixture.machine.in_motion = false;
    s_segment_active = false;
    s_jog_active = false;
    g_fixture.machine.spindle_on = false;
    g_fixture.machine.coolant_flood = false;
    g_fixture.machine.coolant_mist = false;
}

void emulator_clear_estop(void) {
    if (g_fixture.machine.state == STATE_ESTOP) {
        g_fixture.machine.state = STATE_IDLE;
        g_fixture.machine.alarm = ALARM_NONE;
    }
}

void emulator_cycle_state(void) {
    // Cycle through states for testing
    switch (g_fixture.machine.state) {
        case STATE_IDLE:
            g_fixture.machine.state = STATE_RUNNING;
            break;
        case STATE_RUNNING:
            g_fixture.machine.state = STATE_PAUSED;
            break;
        case STATE_PAUSED:
            g_fixture.machine.state = STATE_IDLE;
            break;
        case STATE_ALARM:
        case STATE_ESTOP:
            g_fixture.machine.state = STATE_IDLE;
            g_fixture.machine.alarm = ALARM_NONE;
            break;
        default:
            g_fixture.machine.state = STATE_IDLE;
            break;
    }
}

// =============================================================================
// Homing
// =============================================================================

void emulator_start_homing(int8_t axis) {
    g_fixture.machine.state = STATE_HOMING;
    g_fixture.machine.homing_active = true;
    s_homing_axis = axis;
    s_homing_start_time = g_fixture.uptime_ms;
    
    char msg[LOG_LINE_LENGTH];
    if (axis < 0) {
        snprintf(msg, sizeof(msg), "Homing all axes...");
    } else {
        snprintf(msg, sizeof(msg), "Homing axis %c...", 'X' + axis);
    }
    display_log(msg);
}

static void homing_tick(void) {
    if (!g_fixture.machine.homing_active) return;
    
    uint32_t elapsed = g_fixture.uptime_ms - s_homing_start_time;
    
    // Simulate homing taking EMU_HOMING_DURATION_MS per axis
    int num_axes = (s_homing_axis < 0) ? 3 : 1;
    uint32_t total_duration = EMU_HOMING_DURATION_MS * num_axes;
    
    if (elapsed >= total_duration) {
        // Homing complete
        g_fixture.machine.homing_active = false;
        g_fixture.machine.state = STATE_IDLE;
        
        // Set positions to 0 (machine home)
        if (s_homing_axis < 0) {
            for (int i = 0; i < 3; i++) {
                g_fixture.machine.position_nm[i] = 0;
            }
        } else {
            g_fixture.machine.position_nm[s_homing_axis] = 0;
        }
        
        display_log("Homing complete");
        s_homing_axis = -1;
    }
}

// =============================================================================
// Jogging
// =============================================================================

void emulator_jog_start(uint8_t axis, int32_t velocity_nm_s) {
    if (axis >= MAX_AXES) return;
    
    s_jog_active = true;
    s_jog_axis = axis;
    s_jog_velocity = velocity_nm_s;
    g_fixture.machine.state = STATE_JOG;
    g_fixture.machine.in_motion = true;
}

void emulator_jog_stop(void) {
    s_jog_active = false;
    s_jog_velocity = 0;
    g_fixture.machine.in_motion = false;
    if (g_fixture.machine.state == STATE_JOG) {
        g_fixture.machine.state = STATE_IDLE;
    }
}

static void jog_tick(uint32_t dt_us) {
    if (!s_jog_active) return;
    
    // Update position based on velocity
    // velocity is in nm/sec, dt is in microseconds
    int32_t delta = (int32_t)((int64_t)s_jog_velocity * dt_us / 1000000);
    g_fixture.machine.position_nm[s_jog_axis] += delta;
    
    // Apply soft limits
    int32_t max_pos = 0;
    switch (s_jog_axis) {
        case 0: max_pos = MM_TO_NM(EMU_DEFAULT_TRAVEL_X); break;
        case 1: max_pos = MM_TO_NM(EMU_DEFAULT_TRAVEL_Y); break;
        case 2: max_pos = MM_TO_NM(EMU_DEFAULT_TRAVEL_Z); break;
    }
    
    if (g_fixture.machine.position_nm[s_jog_axis] < 0) {
        g_fixture.machine.position_nm[s_jog_axis] = 0;
    } else if (g_fixture.machine.position_nm[s_jog_axis] > max_pos) {
        g_fixture.machine.position_nm[s_jog_axis] = max_pos;
    }
}

// =============================================================================
// Motion Queue
// =============================================================================

bool emulator_queue_motion(const uint8_t* payload, uint8_t len) {
    // Check if queue is full
    uint8_t next_tail = (s_queue_tail + 1) % MOTION_QUEUE_SIZE;
    if (next_tail == s_queue_head) {
        return false;  // Queue full
    }
    
    // Parse motion segment from payload
    // Minimal parsing: motion_type (1), flags (1), feed_rate (4), targets (4*3)
    if (len < 18) {
        return false;  // Invalid payload
    }
    
    motion_segment_t* seg = &s_motion_queue[s_queue_tail];
    seg->motion_type = payload[0];
    seg->flags = payload[1];
    seg->feed_rate = payload[2] | (payload[3] << 8) | 
                     (payload[4] << 16) | (payload[5] << 24);
    
    // Parse target positions (X, Y, Z)
    for (int i = 0; i < 3; i++) {
        int offset = 6 + i * 4;
        seg->target_nm[i] = payload[offset] | (payload[offset+1] << 8) |
                            (payload[offset+2] << 16) | (payload[offset+3] << 24);
    }
    
    seg->in_use = true;
    s_queue_tail = next_tail;
    g_fixture.machine.queue_depth = (s_queue_tail - s_queue_head + MOTION_QUEUE_SIZE) % MOTION_QUEUE_SIZE;
    
    return true;
}

void emulator_clear_queue(void) {
    s_queue_head = 0;
    s_queue_tail = 0;
    s_segment_active = false;
    g_fixture.machine.queue_depth = 0;
    g_fixture.machine.queue_capacity = MOTION_QUEUE_SIZE;
    
    for (int i = 0; i < MOTION_QUEUE_SIZE; i++) {
        s_motion_queue[i].in_use = false;
    }
}

static void motion_tick(uint32_t dt_us) {
    if (g_fixture.machine.state != STATE_RUNNING) return;
    
    // If no active segment, try to get one from queue
    if (!s_segment_active) {
        if (s_queue_head != s_queue_tail) {
            // Get next segment
            memcpy(&s_current_segment, &s_motion_queue[s_queue_head], sizeof(motion_segment_t));
            s_queue_head = (s_queue_head + 1) % MOTION_QUEUE_SIZE;
            g_fixture.machine.queue_depth--;
            
            // Calculate segment duration based on distance and feed rate
            int64_t dx = s_current_segment.target_nm[0] - g_fixture.machine.position_nm[0];
            int64_t dy = s_current_segment.target_nm[1] - g_fixture.machine.position_nm[1];
            int64_t dz = s_current_segment.target_nm[2] - g_fixture.machine.position_nm[2];
            
            // Distance in nm
            double dist = 0;
            dist = dx*dx + dy*dy + dz*dz;
            if (dist > 0) {
                // sqrt approximation - just use a simple method
                // For simulation, accuracy isn't critical
                int64_t d = (int64_t)(dx > 0 ? dx : -dx) + 
                            (int64_t)(dy > 0 ? dy : -dy) + 
                            (int64_t)(dz > 0 ? dz : -dz);
                
                // Duration = distance / velocity
                // feed_rate is mm/min * 1000 = nm/min / 1000
                // Convert to nm/sec: feed_rate * 1000000 / 60000 = feed_rate * 16.67
                uint32_t velocity_nm_s = (s_current_segment.feed_rate * 1000) / 60;
                if (velocity_nm_s > 0) {
                    s_current_segment.duration_us = (uint32_t)((d * 1000000ULL) / velocity_nm_s);
                } else {
                    s_current_segment.duration_us = 1000000;  // 1 second default
                }
            } else {
                s_current_segment.duration_us = 0;
            }
            
            s_segment_elapsed_us = 0;
            s_segment_active = true;
            g_fixture.machine.in_motion = true;
            g_fixture.machine.current_feed_rate = s_current_segment.feed_rate;
        } else {
            // Queue empty, nothing to do
            g_fixture.machine.in_motion = false;
            return;
        }
    }
    
    // Execute current segment
    s_segment_elapsed_us += dt_us;
    
    if (s_segment_elapsed_us >= s_current_segment.duration_us || 
        s_current_segment.duration_us == 0) {
        // Segment complete - snap to target
        for (int i = 0; i < 3; i++) {
            g_fixture.machine.position_nm[i] = s_current_segment.target_nm[i];
        }
        s_segment_active = false;
    } else {
        // Interpolate position
        float t = (float)s_segment_elapsed_us / (float)s_current_segment.duration_us;
        for (int i = 0; i < 3; i++) {
            int32_t start = g_fixture.machine.position_nm[i];
            int32_t end = s_current_segment.target_nm[i];
            g_fixture.machine.position_nm[i] = start + (int32_t)((end - start) * t);
        }
    }
}

// =============================================================================
// Alarm Injection
// =============================================================================

void emulator_inject_alarm(alarm_code_t alarm) {
    g_fixture.machine.alarm = alarm;
    g_fixture.machine.state = STATE_ALARM;
    g_fixture.machine.in_motion = false;
    s_segment_active = false;
    s_jog_active = false;
    
    char msg[LOG_LINE_LENGTH];
    snprintf(msg, sizeof(msg), "Alarm: %s", emulator_alarm_name());
    display_log(msg);
}

void emulator_clear_alarms(void) {
    if (g_fixture.machine.alarm != ALARM_NONE) {
        g_fixture.machine.alarm = ALARM_NONE;
        g_fixture.machine.state = STATE_IDLE;
        g_fixture.machine.limit_switches = 0;
        display_log("Alarms cleared");
    }
}

// =============================================================================
// Main Tick (call at EMULATOR_TICK_HZ)
// =============================================================================

void emulator_tick(void) {
    static uint32_t last_tick_us = 0;
    uint32_t now_us = g_fixture.uptime_ms * 1000;
    uint32_t dt_us = now_us - last_tick_us;
    last_tick_us = now_us;
    
    // Cap dt to prevent huge jumps
    if (dt_us > 10000) dt_us = 10000;  // Max 10ms step
    
    // Update based on state
    switch (g_fixture.machine.state) {
        case STATE_RUNNING:
            motion_tick(dt_us);
            break;
            
        case STATE_HOMING:
            homing_tick();
            break;
            
        case STATE_JOG:
            jog_tick(dt_us);
            break;
            
        default:
            break;
    }
}

// =============================================================================
// State/Alarm Name Helpers
// =============================================================================

const char* emulator_state_name(void) {
    uint8_t state = (uint8_t)g_fixture.machine.state;
    if (state < sizeof(STATE_NAMES) / sizeof(STATE_NAMES[0])) {
        return STATE_NAMES[state];
    }
    return "UNKNOWN";
}

const char* emulator_alarm_name(void) {
    uint8_t alarm = (uint8_t)g_fixture.machine.alarm;
    if (alarm < sizeof(ALARM_NAMES) / sizeof(ALARM_NAMES[0])) {
        return ALARM_NAMES[alarm];
    }
    return "UNKNOWN";
}
