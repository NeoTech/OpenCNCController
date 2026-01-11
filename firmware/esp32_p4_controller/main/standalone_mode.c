/**
 * @file standalone_mode.c
 * @brief Standalone Mode Implementation
 */

#include "standalone_mode.h"
#include "canopen_master.h"
#include "display_task.h"
#include <string.h>
#include "esp_log.h"
#include "driver/gpio.h"
#include "driver/pulse_cnt.h"

static const char *TAG = "standalone";

// Standalone state
static standalone_state_t g_standalone = {0};

// Encoder handle
static pcnt_unit_handle_t g_encoder = NULL;

// Jog step sizes (in steps) for each speed
static const int32_t JOG_STEPS[] = {
    40,     // SLOW: 0.1mm at 400 steps/mm
    400,    // MEDIUM: 1mm
    4000,   // FAST: 10mm
};

// Button GPIO pins (parsed from config)
static uint8_t g_button_pins[4] = {22, 23, 24, 25};

// =============================================================================
// Initialization
// =============================================================================

void standalone_mode_init(void)
{
    ESP_LOGI(TAG, "Initializing standalone mode");
    
    memset(&g_standalone, 0, sizeof(g_standalone));
    g_standalone.selected_axis = 0;  // X axis
    g_standalone.jog_speed = JOG_SPEED_MEDIUM;
    
    // Configure encoder using PCNT
    pcnt_unit_config_t unit_config = {
        .high_limit = 32767,
        .low_limit = -32768,
    };
    ESP_ERROR_CHECK(pcnt_new_unit(&unit_config, &g_encoder));
    
    // Configure encoder channels
    pcnt_chan_config_t chan_config = {
        .edge_gpio_num = CONFIG_OPENCNC_JOG_ENCODER_PIN_A,
        .level_gpio_num = CONFIG_OPENCNC_JOG_ENCODER_PIN_B,
    };
    pcnt_channel_handle_t chan = NULL;
    ESP_ERROR_CHECK(pcnt_new_channel(g_encoder, &chan_config, &chan));
    
    // Set edge actions for quadrature decoding
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(chan, 
        PCNT_CHANNEL_EDGE_ACTION_DECREASE,
        PCNT_CHANNEL_EDGE_ACTION_INCREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(chan,
        PCNT_CHANNEL_LEVEL_ACTION_KEEP,
        PCNT_CHANNEL_LEVEL_ACTION_INVERSE));
    
    // Start encoder
    ESP_ERROR_CHECK(pcnt_unit_enable(g_encoder));
    ESP_ERROR_CHECK(pcnt_unit_start(g_encoder));
    
    // Configure button GPIOs
    for (int i = 0; i < 4; i++) {
        gpio_config_t io_conf = {
            .pin_bit_mask = (1ULL << g_button_pins[i]),
            .mode = GPIO_MODE_INPUT,
            .pull_up_en = GPIO_PULLUP_ENABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE,
        };
        gpio_config(&io_conf);
    }
    
    ESP_LOGI(TAG, "Standalone mode initialized");
}

// =============================================================================
// Button Handling
// =============================================================================

static bool read_button(uint8_t btn)
{
    if (btn >= 4) return false;
    return gpio_get_level(g_button_pins[btn]) == 0;  // Active low
}

static bool button_pressed(uint8_t btn, uint32_t current_tick)
{
    bool pressed = read_button(btn);
    bool was_pressed = (g_standalone.button_state & (1 << btn)) != 0;
    
    if (pressed && !was_pressed) {
        // Debounce: check if enough time has passed
        if (current_tick - g_standalone.button_last_tick[btn] > 50) {
            g_standalone.button_state |= (1 << btn);
            g_standalone.button_last_tick[btn] = current_tick;
            return true;
        }
    } else if (!pressed) {
        g_standalone.button_state &= ~(1 << btn);
    }
    
    return false;
}

// =============================================================================
// Tick
// =============================================================================

void standalone_mode_tick(void* ctx)
{
    if (!g_standalone.active) {
        g_standalone.active = true;
        ESP_LOGI(TAG, "Standalone mode activated");
    }
    
    uint32_t current_tick = xTaskGetTickCount();
    
    // Process buttons
    if (button_pressed(BTN_AXIS_SELECT, current_tick)) {
        // Cycle to next axis
        uint8_t max_axes = canopen_master_get_axis_count();
        if (max_axes > 0) {
            g_standalone.selected_axis = (g_standalone.selected_axis + 1) % max_axes;
            ESP_LOGI(TAG, "Selected axis: %d", g_standalone.selected_axis);
            
            char msg[32];
            snprintf(msg, sizeof(msg), "Axis: %c", 'X' + g_standalone.selected_axis);
            display_show_message(0, msg);
        }
    }
    
    if (button_pressed(BTN_JOG_SPEED, current_tick)) {
        // Cycle jog speed
        g_standalone.jog_speed = (g_standalone.jog_speed + 1) % 3;
        
        const char* speed_names[] = {"Slow", "Medium", "Fast"};
        ESP_LOGI(TAG, "Jog speed: %s", speed_names[g_standalone.jog_speed]);
        
        char msg[32];
        snprintf(msg, sizeof(msg), "Speed: %s", speed_names[g_standalone.jog_speed]);
        display_show_message(1, msg);
    }
    
    if (button_pressed(BTN_HOME, current_tick)) {
        // Start homing for selected axis
        standalone_mode_home_axis();
    }
    
    if (button_pressed(BTN_ESTOP, current_tick)) {
        // Emergency stop
        ESP_LOGW(TAG, "E-STOP pressed");
        canopen_master_stop_all_nodes();
        display_show_message(3, "** E-STOP **");
    }
    
    // Read encoder
    int count = 0;
    pcnt_unit_get_count(g_encoder, &count);
    
    int32_t delta = count - g_standalone.last_encoder_count;
    g_standalone.last_encoder_count = count;
    
    if (delta != 0) {
        standalone_mode_jog(delta);
    }
}

// =============================================================================
// Jog Control
// =============================================================================

void standalone_mode_jog(int32_t delta)
{
    if (delta == 0) return;
    
    uint8_t axis = g_standalone.selected_axis;
    int32_t step_size = JOG_STEPS[g_standalone.jog_speed];
    int32_t movement = delta * step_size;
    
    // Get current position
    int32_t current_pos = canopen_master_get_position(axis + 1);
    int32_t new_pos = current_pos + movement;
    
    // Set new target position
    canopen_master_set_position(axis + 1, new_pos);
    
    // Ensure axis is enabled
    canopen_master_enable_axis(axis + 1);
}

// =============================================================================
// Other Functions
// =============================================================================

const standalone_state_t* standalone_mode_get_state(void)
{
    return &g_standalone;
}

void standalone_mode_select_axis(uint8_t axis)
{
    if (axis < canopen_master_get_axis_count()) {
        g_standalone.selected_axis = axis;
    }
}

void standalone_mode_set_jog_speed(jog_speed_t speed)
{
    g_standalone.jog_speed = speed;
}

void standalone_mode_home_axis(void)
{
    uint8_t axis = g_standalone.selected_axis;
    ESP_LOGI(TAG, "Homing axis %d", axis);
    
    canopen_master_start_homing(axis + 1, -1);  // -1 = default method
    
    char msg[32];
    snprintf(msg, sizeof(msg), "Homing %c...", 'X' + axis);
    display_show_message(2, msg);
}
