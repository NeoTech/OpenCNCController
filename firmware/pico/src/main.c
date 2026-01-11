/*
 * OpenCNC HAL Firmware - Raspberry Pi Pico (RP2040)
 * 
 * Main entry point for Pico based motion controller.
 * Uses PIO for hardware step generation.
 * 
 * Copyright (c) 2024 OpenCNC Project
 * MIT License
 */

#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "hardware/gpio.h"
#include "hardware/timer.h"
#include "hardware/pwm.h"
#include "hardware/pio.h"
#include "tusb.h"

#include "../common/hal_common.h"

// ============================================================================
// Pin Definitions
// ============================================================================

#define STEP_X_PIN      0
#define DIR_X_PIN       1
#define STEP_Y_PIN      2
#define DIR_Y_PIN       3
#define STEP_Z_PIN      4
#define DIR_Z_PIN       5

#define LIMIT_X_PIN     10
#define LIMIT_Y_PIN     11
#define LIMIT_Z_PIN     12

#define PROBE_PIN       13
#define ESTOP_PIN       14

#define SPINDLE_PWM_PIN 15
#define SPINDLE_EN_PIN  16

#define COOLANT_MIST_PIN    17
#define COOLANT_FLOOD_PIN   18

// ============================================================================
// PIO Step Generator
// ============================================================================

// PIO program for step pulse generation
// This is a placeholder - real implementation would use .pio file
static const uint16_t step_program[] = {
    // set pins, 1      [delay]   ; Set step high
    // nop              [delay]   ; Hold
    // set pins, 0      [delay]   ; Set step low
    // ... timing control
    0xe001,  // set pins, 1
    0xa042,  // nop
    0xe000,  // set pins, 0
    0x0000,  // jmp 0
};

static PIO step_pio = pio0;
static uint step_sm = 0;

// ============================================================================
// Global State
// ============================================================================

static system_config_t g_config;
static realtime_status_t g_status;
static motion_segment_t g_motion_queue[MOTION_QUEUE_SIZE];
static volatile uint8_t g_queue_head = 0;
static volatile uint8_t g_queue_tail = 0;

static motion_segment_t g_current_segment;
static bool g_has_segment = false;
static steps_t g_current_position[MAX_AXES] = {0};
static uint32_t g_segment_tick = 0;

static struct repeating_timer g_step_timer;
static volatile bool g_timer_running = false;

static uint g_spindle_slice;

// ============================================================================
// Step Timer Callback
// ============================================================================

static bool step_timer_callback(struct repeating_timer *t) {
    (void)t;
    
    if (!g_has_segment) {
        return true;
    }
    
    g_segment_tick++;
    
    uint32_t total_ticks = g_current_segment.accel_ticks + 
                           g_current_segment.cruise_ticks + 
                           g_current_segment.decel_ticks;
    
    if (g_segment_tick >= total_ticks) {
        for (int i = 0; i < MAX_AXES; i++) {
            g_current_position[i] = g_current_segment.target[i];
        }
        g_has_segment = false;
    }
    
    return true;
}

// ============================================================================
// GPIO Configuration
// ============================================================================

static void configure_gpio(void) {
    // Step/direction pins as outputs
    const uint8_t output_pins[] = {
        STEP_X_PIN, DIR_X_PIN, STEP_Y_PIN, DIR_Y_PIN, STEP_Z_PIN, DIR_Z_PIN,
        SPINDLE_EN_PIN, COOLANT_MIST_PIN, COOLANT_FLOOD_PIN
    };
    
    for (size_t i = 0; i < sizeof(output_pins); i++) {
        gpio_init(output_pins[i]);
        gpio_set_dir(output_pins[i], GPIO_OUT);
        gpio_put(output_pins[i], 0);
    }
    
    // Limit/probe pins as inputs with pull-up
    const uint8_t input_pins[] = {
        LIMIT_X_PIN, LIMIT_Y_PIN, LIMIT_Z_PIN, PROBE_PIN, ESTOP_PIN
    };
    
    for (size_t i = 0; i < sizeof(input_pins); i++) {
        gpio_init(input_pins[i]);
        gpio_set_dir(input_pins[i], GPIO_IN);
        gpio_pull_up(input_pins[i]);
    }
    
    printf("GPIO configured\n");
}

// ============================================================================
// Spindle PWM Configuration
// ============================================================================

static void configure_spindle_pwm(void) {
    gpio_set_function(SPINDLE_PWM_PIN, GPIO_FUNC_PWM);
    g_spindle_slice = pwm_gpio_to_slice_num(SPINDLE_PWM_PIN);
    
    // 5 kHz PWM
    pwm_set_wrap(g_spindle_slice, 10000);
    pwm_set_clkdiv(g_spindle_slice, 2.5f);
    pwm_set_enabled(g_spindle_slice, true);
    
    printf("Spindle PWM configured\n");
}

// ============================================================================
// HAL Implementation
// ============================================================================

void hal_init(void) {
    printf("OpenCNC HAL initializing...\n");
    
    memset(&g_config, 0, sizeof(g_config));
    g_config.num_axes = 3;
    
    for (int i = 0; i < 3; i++) {
        g_config.axes[i].enabled = true;
        g_config.axes[i].steps_per_mm = 800.0;
        g_config.axes[i].max_velocity = 5000.0;
        g_config.axes[i].max_acceleration = 500.0;
        g_config.axes[i].max_travel = 200.0;
    }
    
    configure_gpio();
    configure_spindle_pwm();
    
    g_status.state = STATE_IDLE;
    g_status.alarm = ALARM_NONE;
    
    printf("HAL initialization complete\n");
}

bool hal_motion_queue_push(const motion_segment_t* segment) {
    uint8_t next_head = (g_queue_head + 1) % MOTION_QUEUE_SIZE;
    if (next_head == g_queue_tail) {
        return false;
    }
    
    memcpy(&g_motion_queue[g_queue_head], segment, sizeof(motion_segment_t));
    g_queue_head = next_head;
    return true;
}

bool hal_motion_queue_pop(motion_segment_t* segment) {
    if (g_queue_head == g_queue_tail) {
        return false;
    }
    
    memcpy(segment, &g_motion_queue[g_queue_tail], sizeof(motion_segment_t));
    g_queue_tail = (g_queue_tail + 1) % MOTION_QUEUE_SIZE;
    return true;
}

uint8_t hal_motion_queue_depth(void) {
    if (g_queue_head >= g_queue_tail) {
        return g_queue_head - g_queue_tail;
    }
    return MOTION_QUEUE_SIZE - g_queue_tail + g_queue_head;
}

void hal_motion_queue_clear(void) {
    g_queue_head = 0;
    g_queue_tail = 0;
}

void hal_motion_start(void) {
    if (g_status.state == STATE_IDLE || g_status.state == STATE_PAUSED) {
        g_status.state = STATE_RUNNING;
        
        if (!g_timer_running) {
            add_repeating_timer_us(-10, step_timer_callback, NULL, &g_step_timer);
            g_timer_running = true;
        }
        
        printf("Motion started\n");
    }
}

void hal_motion_pause(void) {
    if (g_status.state == STATE_RUNNING) {
        g_status.state = STATE_PAUSED;
        cancel_repeating_timer(&g_step_timer);
        g_timer_running = false;
        printf("Motion paused\n");
    }
}

void hal_motion_resume(void) {
    if (g_status.state == STATE_PAUSED) {
        g_status.state = STATE_RUNNING;
        add_repeating_timer_us(-10, step_timer_callback, NULL, &g_step_timer);
        g_timer_running = true;
        printf("Motion resumed\n");
    }
}

void hal_motion_stop(void) {
    g_status.state = STATE_IDLE;
    cancel_repeating_timer(&g_step_timer);
    g_timer_running = false;
    hal_motion_queue_clear();
    g_has_segment = false;
    printf("Motion stopped\n");
}

void hal_motion_estop(void) {
    g_status.state = STATE_ESTOP;
    g_status.alarm = ALARM_ESTOP;
    cancel_repeating_timer(&g_step_timer);
    g_timer_running = false;
    hal_motion_queue_clear();
    g_has_segment = false;
    
    gpio_put(SPINDLE_EN_PIN, 0);
    gpio_put(COOLANT_MIST_PIN, 0);
    gpio_put(COOLANT_FLOOD_PIN, 0);
    
    printf("EMERGENCY STOP!\n");
}

void hal_spindle_set(bool on, bool cw, uint16_t rpm) {
    (void)cw;
    
    if (on) {
        uint32_t duty = (rpm * 10000) / g_config.spindle_max_rpm;
        pwm_set_chan_level(g_spindle_slice, PWM_CHAN_A, duty);
        gpio_put(SPINDLE_EN_PIN, 1);
        
        g_status.spindle_on = true;
        g_status.spindle_cw = cw;
        g_status.spindle_rpm = rpm;
    } else {
        hal_spindle_stop();
    }
}

void hal_spindle_stop(void) {
    pwm_set_chan_level(g_spindle_slice, PWM_CHAN_A, 0);
    gpio_put(SPINDLE_EN_PIN, 0);
    
    g_status.spindle_on = false;
    g_status.spindle_rpm = 0;
}

void hal_coolant_set(bool mist, bool flood) {
    gpio_put(COOLANT_MIST_PIN, mist);
    gpio_put(COOLANT_FLOOD_PIN, flood);
    
    g_status.coolant_mist = mist;
    g_status.coolant_flood = flood;
}

void hal_get_status(realtime_status_t* status) {
    memcpy(status, &g_status, sizeof(realtime_status_t));
    
    for (int i = 0; i < MAX_AXES; i++) {
        status->position[i] = g_current_position[i];
    }
    
    status->limit_state = 0;
    if (!gpio_get(LIMIT_X_PIN)) status->limit_state |= 0x01;
    if (!gpio_get(LIMIT_Y_PIN)) status->limit_state |= 0x02;
    if (!gpio_get(LIMIT_Z_PIN)) status->limit_state |= 0x04;
    
    status->probe_triggered = !gpio_get(PROBE_PIN);
    status->queue_depth = hal_motion_queue_depth();
}

void hal_get_position(steps_t* position) {
    for (int i = 0; i < MAX_AXES; i++) {
        position[i] = g_current_position[i];
    }
}

// ============================================================================
// Core 1 - Motion Control
// ============================================================================

static void core1_main(void) {
    while (1) {
        if (g_status.state == STATE_RUNNING && !g_has_segment) {
            if (hal_motion_queue_pop(&g_current_segment)) {
                g_has_segment = true;
                g_segment_tick = 0;
                g_status.current_segment_id = g_current_segment.id;
            } else {
                g_status.state = STATE_IDLE;
                cancel_repeating_timer(&g_step_timer);
                g_timer_running = false;
            }
        }
        
        sleep_ms(1);
    }
}

// ============================================================================
// USB CDC Processing
// ============================================================================

static uint8_t rx_buffer[256];

static void process_usb(void) {
    if (tud_cdc_available()) {
        uint32_t count = tud_cdc_read(rx_buffer, sizeof(rx_buffer));
        // TODO: Process protocol packets
        (void)count;
    }
}

void hal_send_status(void) {
    realtime_status_t status;
    hal_get_status(&status);
    
    uint8_t packet[64];
    size_t len = 0;
    
    packet[len++] = 0xAA;
    packet[len++] = 0xA2;  // RT_STATUS
    packet[len++] = 0x00;
    packet[len++] = sizeof(realtime_status_t);
    
    memcpy(&packet[len], &status, sizeof(realtime_status_t));
    len += sizeof(realtime_status_t);
    
    packet[len++] = 0x00;
    packet[len++] = 0x00;
    packet[len++] = 0x55;
    
    if (tud_cdc_connected()) {
        tud_cdc_write(packet, len);
        tud_cdc_write_flush();
    }
}

// ============================================================================
// Main Entry Point
// ============================================================================

int main(void) {
    stdio_init_all();
    
    printf("OpenCNC HAL - Raspberry Pi Pico\n");
    printf("Firmware Version: 0.1.0\n");
    
    tusb_init();
    
    hal_init();
    
    // Launch motion control on Core 1
    multicore_launch_core1(core1_main);
    
    printf("System ready\n");
    
    // Core 0 - USB and status
    uint32_t last_status_time = 0;
    
    while (1) {
        tud_task();
        process_usb();
        
        uint32_t now = to_ms_since_boot(get_absolute_time());
        if (now - last_status_time >= (1000 / STATUS_UPDATE_HZ)) {
            hal_send_status();
            last_status_time = now;
        }
    }
    
    return 0;
}
