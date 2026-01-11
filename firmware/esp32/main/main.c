/*
 * OpenCNC HAL Firmware - ESP32-S3
 * 
 * Main entry point for ESP32-S3 based motion controller.
 * Uses ESP-IDF framework with FreeRTOS.
 * 
 * Copyright (c) 2024 OpenCNC Project
 * MIT License
 */

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "driver/gptimer.h"
#include "driver/ledc.h"
#include "esp_log.h"
#include "tinyusb.h"
#include "tusb_cdc_acm.h"

#include "../common/hal_common.h"

static const char *TAG = "CNC_HAL";

// ============================================================================
// Pin Definitions (ESP32-S3-DevKitC-1)
// ============================================================================

#define STEP_X_PIN      GPIO_NUM_1
#define DIR_X_PIN       GPIO_NUM_2
#define STEP_Y_PIN      GPIO_NUM_3
#define DIR_Y_PIN       GPIO_NUM_4
#define STEP_Z_PIN      GPIO_NUM_5
#define DIR_Z_PIN       GPIO_NUM_6

#define LIMIT_X_PIN     GPIO_NUM_7
#define LIMIT_Y_PIN     GPIO_NUM_8
#define LIMIT_Z_PIN     GPIO_NUM_9

#define PROBE_PIN       GPIO_NUM_10
#define ESTOP_PIN       GPIO_NUM_11

#define SPINDLE_PWM_PIN GPIO_NUM_12
#define SPINDLE_EN_PIN  GPIO_NUM_13

#define COOLANT_MIST_PIN    GPIO_NUM_14
#define COOLANT_FLOOD_PIN   GPIO_NUM_15

// ============================================================================
// Global State
// ============================================================================

static system_config_t g_config;
static realtime_status_t g_status;
static motion_segment_t g_motion_queue[MOTION_QUEUE_SIZE];
static volatile uint8_t g_queue_head = 0;
static volatile uint8_t g_queue_tail = 0;

static gptimer_handle_t g_step_timer = NULL;
static volatile bool g_timer_running = false;

// Current motion state
static motion_segment_t g_current_segment;
static bool g_has_segment = false;
static steps_t g_current_position[MAX_AXES] = {0};
static uint32_t g_segment_tick = 0;

// ============================================================================
// Step Timer ISR
// ============================================================================

static bool IRAM_ATTR step_timer_isr(gptimer_handle_t timer,
                                      const gptimer_alarm_event_data_t *edata,
                                      void *user_data) {
    (void)timer;
    (void)edata;
    (void)user_data;
    
    if (!g_has_segment) {
        return false;
    }
    
    // Bresenham-style step generation would go here
    // This is a simplified placeholder
    
    g_segment_tick++;
    
    uint32_t total_ticks = g_current_segment.accel_ticks + 
                           g_current_segment.cruise_ticks + 
                           g_current_segment.decel_ticks;
    
    if (g_segment_tick >= total_ticks) {
        // Segment complete
        for (int i = 0; i < MAX_AXES; i++) {
            g_current_position[i] = g_current_segment.target[i];
        }
        g_has_segment = false;
    }
    
    return false;
}

// ============================================================================
// GPIO Configuration
// ============================================================================

static void configure_gpio(void) {
    // Configure step/dir pins as outputs
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << STEP_X_PIN) | (1ULL << DIR_X_PIN) |
                        (1ULL << STEP_Y_PIN) | (1ULL << DIR_Y_PIN) |
                        (1ULL << STEP_Z_PIN) | (1ULL << DIR_Z_PIN) |
                        (1ULL << SPINDLE_EN_PIN) |
                        (1ULL << COOLANT_MIST_PIN) | (1ULL << COOLANT_FLOOD_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);
    
    // Configure limit/probe/estop pins as inputs with pull-up
    io_conf.pin_bit_mask = (1ULL << LIMIT_X_PIN) | (1ULL << LIMIT_Y_PIN) |
                           (1ULL << LIMIT_Z_PIN) | (1ULL << PROBE_PIN) |
                           (1ULL << ESTOP_PIN);
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    io_conf.intr_type = GPIO_INTR_ANYEDGE;
    gpio_config(&io_conf);
    
    ESP_LOGI(TAG, "GPIO configured");
}

// ============================================================================
// Step Timer Configuration
// ============================================================================

static void configure_step_timer(void) {
    gptimer_config_t timer_config = {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT,
        .direction = GPTIMER_COUNT_UP,
        .resolution_hz = 1000000,  // 1 MHz, 1us resolution
    };
    
    ESP_ERROR_CHECK(gptimer_new_timer(&timer_config, &g_step_timer));
    
    gptimer_alarm_config_t alarm_config = {
        .reload_count = 0,
        .alarm_count = 10,  // 10us initial period
        .flags.auto_reload_on_alarm = true,
    };
    ESP_ERROR_CHECK(gptimer_set_alarm_action(g_step_timer, &alarm_config));
    
    gptimer_event_callbacks_t cbs = {
        .on_alarm = step_timer_isr,
    };
    ESP_ERROR_CHECK(gptimer_register_event_callbacks(g_step_timer, &cbs, NULL));
    
    ESP_ERROR_CHECK(gptimer_enable(g_step_timer));
    
    ESP_LOGI(TAG, "Step timer configured");
}

// ============================================================================
// Spindle PWM Configuration
// ============================================================================

static void configure_spindle_pwm(void) {
    ledc_timer_config_t timer_conf = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_10_BIT,
        .timer_num = LEDC_TIMER_0,
        .freq_hz = 5000,
        .clk_cfg = LEDC_AUTO_CLK
    };
    ledc_timer_config(&timer_conf);
    
    ledc_channel_config_t channel_conf = {
        .gpio_num = SPINDLE_PWM_PIN,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CHANNEL_0,
        .timer_sel = LEDC_TIMER_0,
        .duty = 0,
        .hpoint = 0
    };
    ledc_channel_config(&channel_conf);
    
    ESP_LOGI(TAG, "Spindle PWM configured");
}

// ============================================================================
// USB CDC Configuration
// ============================================================================

static uint8_t rx_buffer[256];
static uint8_t tx_buffer[256];

static void usb_cdc_rx_callback(int itf, cdcacm_event_t *event) {
    (void)itf;
    
    if (event->type == CDC_EVENT_RX) {
        size_t rx_size = 0;
        esp_err_t ret = tinyusb_cdcacm_read(0, rx_buffer, sizeof(rx_buffer), &rx_size);
        if (ret == ESP_OK && rx_size > 0) {
            // Process received data
            // TODO: Parse protocol packets
        }
    }
}

static void configure_usb(void) {
    tinyusb_config_t tusb_cfg = {
        .device_descriptor = NULL,
        .string_descriptor = NULL,
        .external_phy = false,
        .configuration_descriptor = NULL,
    };
    ESP_ERROR_CHECK(tinyusb_driver_install(&tusb_cfg));
    
    tinyusb_config_cdcacm_t acm_cfg = {
        .usb_dev = TINYUSB_USBDEV_0,
        .cdc_port = TINYUSB_CDC_ACM_0,
        .rx_unread_buf_sz = 256,
        .callback_rx = &usb_cdc_rx_callback,
        .callback_rx_wanted_char = NULL,
        .callback_line_state_changed = NULL,
        .callback_line_coding_changed = NULL
    };
    ESP_ERROR_CHECK(tusb_cdc_acm_init(&acm_cfg));
    
    ESP_LOGI(TAG, "USB CDC configured");
}

// ============================================================================
// HAL Implementation
// ============================================================================

void hal_init(void) {
    ESP_LOGI(TAG, "OpenCNC HAL initializing...");
    
    // Load default configuration
    memset(&g_config, 0, sizeof(g_config));
    g_config.num_axes = 3;
    
    for (int i = 0; i < 3; i++) {
        g_config.axes[i].enabled = true;
        g_config.axes[i].steps_per_mm = 800.0;
        g_config.axes[i].max_velocity = 5000.0;
        g_config.axes[i].max_acceleration = 500.0;
        g_config.axes[i].max_travel = 200.0;
    }
    
    // Initialize peripherals
    configure_gpio();
    configure_step_timer();
    configure_spindle_pwm();
    configure_usb();
    
    // Initialize status
    g_status.state = STATE_IDLE;
    g_status.alarm = ALARM_NONE;
    
    ESP_LOGI(TAG, "HAL initialization complete");
}

bool hal_motion_queue_push(const motion_segment_t* segment) {
    uint8_t next_head = (g_queue_head + 1) % MOTION_QUEUE_SIZE;
    if (next_head == g_queue_tail) {
        return false;  // Queue full
    }
    
    memcpy(&g_motion_queue[g_queue_head], segment, sizeof(motion_segment_t));
    g_queue_head = next_head;
    return true;
}

bool hal_motion_queue_pop(motion_segment_t* segment) {
    if (g_queue_head == g_queue_tail) {
        return false;  // Queue empty
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
        gptimer_start(g_step_timer);
        g_timer_running = true;
        ESP_LOGI(TAG, "Motion started");
    }
}

void hal_motion_pause(void) {
    if (g_status.state == STATE_RUNNING) {
        g_status.state = STATE_PAUSED;
        gptimer_stop(g_step_timer);
        g_timer_running = false;
        ESP_LOGI(TAG, "Motion paused");
    }
}

void hal_motion_resume(void) {
    if (g_status.state == STATE_PAUSED) {
        g_status.state = STATE_RUNNING;
        gptimer_start(g_step_timer);
        g_timer_running = true;
        ESP_LOGI(TAG, "Motion resumed");
    }
}

void hal_motion_stop(void) {
    g_status.state = STATE_IDLE;
    gptimer_stop(g_step_timer);
    g_timer_running = false;
    hal_motion_queue_clear();
    g_has_segment = false;
    ESP_LOGI(TAG, "Motion stopped");
}

void hal_motion_estop(void) {
    g_status.state = STATE_ESTOP;
    g_status.alarm = ALARM_ESTOP;
    gptimer_stop(g_step_timer);
    g_timer_running = false;
    hal_motion_queue_clear();
    g_has_segment = false;
    
    // Disable all outputs
    gpio_set_level(SPINDLE_EN_PIN, 0);
    gpio_set_level(COOLANT_MIST_PIN, 0);
    gpio_set_level(COOLANT_FLOOD_PIN, 0);
    
    ESP_LOGW(TAG, "EMERGENCY STOP!");
}

void hal_spindle_set(bool on, bool cw, uint16_t rpm) {
    if (on) {
        uint32_t duty = (rpm * 1023) / g_config.spindle_max_rpm;
        ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, duty);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
        gpio_set_level(SPINDLE_EN_PIN, 1);
        
        g_status.spindle_on = true;
        g_status.spindle_cw = cw;
        g_status.spindle_rpm = rpm;
    } else {
        hal_spindle_stop();
    }
}

void hal_spindle_stop(void) {
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 0);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
    gpio_set_level(SPINDLE_EN_PIN, 0);
    
    g_status.spindle_on = false;
    g_status.spindle_rpm = 0;
}

void hal_coolant_set(bool mist, bool flood) {
    gpio_set_level(COOLANT_MIST_PIN, mist ? 1 : 0);
    gpio_set_level(COOLANT_FLOOD_PIN, flood ? 1 : 0);
    
    g_status.coolant_mist = mist;
    g_status.coolant_flood = flood;
}

void hal_get_status(realtime_status_t* status) {
    memcpy(status, &g_status, sizeof(realtime_status_t));
    
    // Update current position
    for (int i = 0; i < MAX_AXES; i++) {
        status->position[i] = g_current_position[i];
    }
    
    // Read limit switches
    status->limit_state = 0;
    if (!gpio_get_level(LIMIT_X_PIN)) status->limit_state |= 0x01;
    if (!gpio_get_level(LIMIT_Y_PIN)) status->limit_state |= 0x02;
    if (!gpio_get_level(LIMIT_Z_PIN)) status->limit_state |= 0x04;
    
    status->probe_triggered = !gpio_get_level(PROBE_PIN);
    status->queue_depth = hal_motion_queue_depth();
}

void hal_get_position(steps_t* position) {
    for (int i = 0; i < MAX_AXES; i++) {
        position[i] = g_current_position[i];
    }
}

// ============================================================================
// Motion Task
// ============================================================================

static void motion_task(void *pvParameters) {
    (void)pvParameters;
    
    while (1) {
        if (g_status.state == STATE_RUNNING && !g_has_segment) {
            // Try to get next segment from queue
            if (hal_motion_queue_pop(&g_current_segment)) {
                g_has_segment = true;
                g_segment_tick = 0;
                g_status.current_segment_id = g_current_segment.id;
            } else {
                // Queue empty, go idle
                g_status.state = STATE_IDLE;
                gptimer_stop(g_step_timer);
                g_timer_running = false;
            }
        }
        
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

// ============================================================================
// Status Task
// ============================================================================

static void status_task(void *pvParameters) {
    (void)pvParameters;
    
    while (1) {
        // Send status update over USB
        hal_send_status();
        
        vTaskDelay(pdMS_TO_TICKS(1000 / STATUS_UPDATE_HZ));
    }
}

void hal_send_status(void) {
    realtime_status_t status;
    hal_get_status(&status);
    
    // Build and send status packet
    // TODO: Implement protocol packet encoding
    
    uint8_t packet[64];
    size_t len = 0;
    
    packet[len++] = 0xAA;  // Start byte
    packet[len++] = 0xA2;  // RT_STATUS command
    packet[len++] = 0x00;  // Sequence
    packet[len++] = sizeof(realtime_status_t);
    
    memcpy(&packet[len], &status, sizeof(realtime_status_t));
    len += sizeof(realtime_status_t);
    
    // TODO: Add CRC
    packet[len++] = 0x00;
    packet[len++] = 0x00;
    packet[len++] = 0x55;  // End byte
    
    tinyusb_cdcacm_write_queue(0, packet, len);
    tinyusb_cdcacm_write_flush(0, 0);
}

// ============================================================================
// Main Entry Point
// ============================================================================

void app_main(void) {
    ESP_LOGI(TAG, "OpenCNC HAL - ESP32-S3");
    ESP_LOGI(TAG, "Firmware Version: 0.1.0");
    
    hal_init();
    
    // Create tasks
    xTaskCreatePinnedToCore(motion_task, "motion", 4096, NULL, 5, NULL, 1);
    xTaskCreatePinnedToCore(status_task, "status", 4096, NULL, 3, NULL, 0);
    
    ESP_LOGI(TAG, "System ready");
}
