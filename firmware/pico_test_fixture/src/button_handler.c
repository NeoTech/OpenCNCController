/**
 * @file button_handler.c
 * @brief Button input handling for Pico Display 2.0
 * 
 * Handles debouncing, edge detection, and mode-specific actions.
 * Buttons are active-low with internal pull-ups.
 */

#include "button_handler.h"
#include "test_fixture.h"
#include "emulator.h"
#include "display.h"
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include <stdio.h>

// =============================================================================
// Internal State
// =============================================================================

typedef struct {
    uint8_t     gpio;
    bool        raw_state;          // Current GPIO reading (inverted)
    bool        debounced_state;    // After debounce
    bool        prev_state;         // Previous debounced state
    bool        just_pressed;       // Rising edge
    bool        just_released;      // Falling edge
    uint32_t    state_change_time;  // For debounce timing
    uint32_t    press_time;         // When button was pressed
} button_t;

static button_t s_buttons[4] = {
    { .gpio = PIN_BUTTON_A },
    { .gpio = PIN_BUTTON_B },
    { .gpio = PIN_BUTTON_X },
    { .gpio = PIN_BUTTON_Y }
};

#define BTN_A   0
#define BTN_B   1
#define BTN_X   2
#define BTN_Y   3

// Fault injection cycling
static uint8_t s_fault_inject_index = 0;
static const alarm_code_t INJECTABLE_ALARMS[] = {
    ALARM_LIMIT_X,
    ALARM_LIMIT_Y,
    ALARM_LIMIT_Z,
    ALARM_SOFT_LIMIT,
    ALARM_ESTOP,
    ALARM_PROBE_FAIL,
    ALARM_HOME_FAIL,
    ALARM_SPINDLE,
    ALARM_COMM_ERROR
};
#define NUM_INJECTABLE_ALARMS (sizeof(INJECTABLE_ALARMS) / sizeof(INJECTABLE_ALARMS[0]))

// =============================================================================
// Initialization
// =============================================================================

void buttons_init(void) {
    for (int i = 0; i < 4; i++) {
        gpio_init(s_buttons[i].gpio);
        gpio_set_dir(s_buttons[i].gpio, GPIO_IN);
        gpio_pull_up(s_buttons[i].gpio);
        
        s_buttons[i].raw_state = false;
        s_buttons[i].debounced_state = false;
        s_buttons[i].prev_state = false;
        s_buttons[i].just_pressed = false;
        s_buttons[i].just_released = false;
        s_buttons[i].state_change_time = 0;
        s_buttons[i].press_time = 0;
    }
}

// =============================================================================
// Button Polling
// =============================================================================

void buttons_poll(void) {
    uint32_t now = g_fixture.uptime_ms;
    
    for (int i = 0; i < 4; i++) {
        button_t* btn = &s_buttons[i];
        
        // Read GPIO (active low, so invert)
        bool raw = !gpio_get(btn->gpio);
        
        // Simple debounce: only accept state change after stable for DEBOUNCE_MS
        if (raw != btn->raw_state) {
            btn->raw_state = raw;
            btn->state_change_time = now;
        }
        
        // Store previous debounced state for edge detection
        btn->prev_state = btn->debounced_state;
        
        // Update debounced state
        if (now - btn->state_change_time >= BUTTON_DEBOUNCE_MS) {
            btn->debounced_state = btn->raw_state;
        }
        
        // Edge detection
        btn->just_pressed = btn->debounced_state && !btn->prev_state;
        btn->just_released = !btn->debounced_state && btn->prev_state;
        
        // Track press time for held detection
        if (btn->just_pressed) {
            btn->press_time = now;
        }
    }
    
    // Update global fixture state
    g_fixture.buttons.a_pressed = s_buttons[BTN_A].debounced_state;
    g_fixture.buttons.b_pressed = s_buttons[BTN_B].debounced_state;
    g_fixture.buttons.x_pressed = s_buttons[BTN_X].debounced_state;
    g_fixture.buttons.y_pressed = s_buttons[BTN_Y].debounced_state;
    
    g_fixture.buttons.a_just_pressed = s_buttons[BTN_A].just_pressed;
    g_fixture.buttons.b_just_pressed = s_buttons[BTN_B].just_pressed;
    g_fixture.buttons.x_just_pressed = s_buttons[BTN_X].just_pressed;
    g_fixture.buttons.y_just_pressed = s_buttons[BTN_Y].just_pressed;
    
    if (s_buttons[BTN_A].debounced_state) {
        g_fixture.buttons.a_held_ms = now - s_buttons[BTN_A].press_time;
    } else {
        g_fixture.buttons.a_held_ms = 0;
    }
    // Similar for other buttons...
}

// =============================================================================
// Button State Queries
// =============================================================================

bool button_a_pressed(void) { return s_buttons[BTN_A].just_pressed; }
bool button_b_pressed(void) { return s_buttons[BTN_B].just_pressed; }
bool button_x_pressed(void) { return s_buttons[BTN_X].just_pressed; }
bool button_y_pressed(void) { return s_buttons[BTN_Y].just_pressed; }

bool button_a_held(void) { return s_buttons[BTN_A].debounced_state; }
bool button_b_held(void) { return s_buttons[BTN_B].debounced_state; }
bool button_x_held(void) { return s_buttons[BTN_X].debounced_state; }
bool button_y_held(void) { return s_buttons[BTN_Y].debounced_state; }

// =============================================================================
// Button Action Processing
// =============================================================================

static void process_mode_normal(void) {
    // B: Cycle machine state
    if (button_b_pressed()) {
        emulator_cycle_state();
        display_log("State cycled");
    }
    
    // X held: Jog X+
    if (button_x_held()) {
        if (!g_fixture.machine.in_motion || g_fixture.jog_axis != 0) {
            emulator_jog_start(0, MM_TO_NM(EMU_JOG_VELOCITY) / 60);  // nm/sec
            g_fixture.jog_axis = 0;
        }
    }
    // Y held: Jog Y+
    else if (button_y_held()) {
        if (!g_fixture.machine.in_motion || g_fixture.jog_axis != 1) {
            emulator_jog_start(1, MM_TO_NM(EMU_JOG_VELOCITY) / 60);
            g_fixture.jog_axis = 1;
        }
    }
    // Neither X nor Y held: stop jog if active
    else if (g_fixture.machine.state == STATE_JOG) {
        emulator_jog_stop();
    }
}

static void process_mode_homing(void) {
    // B: Start homing all axes
    if (button_b_pressed()) {
        emulator_start_homing(-1);
    }
    
    // X: Cycle through axes to home individually
    static int8_t selected_axis = 0;
    if (button_x_pressed()) {
        selected_axis = (selected_axis + 1) % 3;
        char msg[32];
        snprintf(msg, sizeof(msg), "Selected axis: %c", 'X' + selected_axis);
        display_log(msg);
    }
    
    // Y: Cancel homing / reset
    if (button_y_pressed()) {
        if (g_fixture.machine.homing_active) {
            emulator_stop();
            display_log("Homing cancelled");
        }
    }
}

static void process_mode_fault_inject(void) {
    // B: Inject current fault
    if (button_b_pressed()) {
        emulator_inject_alarm(INJECTABLE_ALARMS[s_fault_inject_index]);
    }
    
    // X: Cycle to next fault type
    if (button_x_pressed()) {
        s_fault_inject_index = (s_fault_inject_index + 1) % NUM_INJECTABLE_ALARMS;
        char msg[32];
        snprintf(msg, sizeof(msg), "Next alarm: %d", INJECTABLE_ALARMS[s_fault_inject_index]);
        display_log(msg);
    }
    
    // Y: Clear all faults
    if (button_y_pressed()) {
        emulator_clear_alarms();
    }
}

static void process_mode_jog(void) {
    // B: Cycle through axes
    if (button_b_pressed()) {
        g_fixture.jog_axis = (g_fixture.jog_axis + 1) % 3;
        char msg[32];
        snprintf(msg, sizeof(msg), "Jog axis: %c", 'X' + g_fixture.jog_axis);
        display_log(msg);
    }
    
    // X held: Jog positive
    if (button_x_held()) {
        int32_t vel = MM_TO_NM(EMU_JOG_VELOCITY) / 60;
        emulator_jog_start(g_fixture.jog_axis, vel);
    }
    // Y held: Jog negative
    else if (button_y_held()) {
        int32_t vel = -MM_TO_NM(EMU_JOG_VELOCITY) / 60;
        emulator_jog_start(g_fixture.jog_axis, vel);
    }
    // Neither: stop
    else if (g_fixture.machine.state == STATE_JOG) {
        emulator_jog_stop();
    }
}

void buttons_process(void) {
    // A always cycles mode
    if (button_a_pressed()) {
        g_fixture.mode = (g_fixture.mode + 1) % MODE_COUNT;
        
        // Stop any active jog when changing modes
        if (g_fixture.machine.state == STATE_JOG) {
            emulator_jog_stop();
        }
        
        char msg[32];
        snprintf(msg, sizeof(msg), "Mode: %s", MODE_NAMES[g_fixture.mode]);
        display_log(msg);
    }
    
    // Process mode-specific actions
    switch (g_fixture.mode) {
        case MODE_NORMAL:
            process_mode_normal();
            break;
            
        case MODE_HOMING:
            process_mode_homing();
            break;
            
        case MODE_FAULT_INJECT:
            process_mode_fault_inject();
            break;
            
        case MODE_JOG:
            process_mode_jog();
            break;
            
        default:
            break;
    }
}
