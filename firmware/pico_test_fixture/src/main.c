/**
 * @file main.c
 * @brief OpenCNC Test Fixture - Main entry point
 * 
 * Hardware: Raspberry Pi Pico WH + Pimoroni Pico Display 2.0
 * 
 * Architecture:
 *   Core 0: USB CDC processing, protocol handling, status broadcast
 *   Core 1: Display rendering, button polling
 * 
 * This firmware emulates a CNC controller for testing the Windows HMI
 * without real CNC hardware.
 */

#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "hardware/timer.h"
#include "tusb.h"

#include "test_fixture.h"
#include "protocol_handler.h"
#include "emulator.h"
#include "display.h"
#include "button_handler.h"

// =============================================================================
// Global Fixture State
// =============================================================================

fixture_state_t g_fixture = {0};

// =============================================================================
// Timing
// =============================================================================

static uint32_t s_last_status_time = 0;
static uint32_t s_last_emulator_time = 0;
static uint32_t s_last_display_time = 0;

#define STATUS_INTERVAL_US      (1000000 / STATUS_BROADCAST_HZ)
#define EMULATOR_INTERVAL_US    (1000000 / EMULATOR_TICK_HZ)
#define DISPLAY_INTERVAL_MS     (1000 / DISPLAY_UPDATE_HZ)

// =============================================================================
// Core 1 Entry Point (Display + Buttons)
// =============================================================================

void core1_main(void) {
    // Initialize display and buttons on Core 1
    display_init();
    buttons_init();
    
    display_log("Core 1 started");
    
    uint32_t last_update = 0;
    
    while (1) {
        uint32_t now = g_fixture.uptime_ms;
        
        // Poll buttons (~every 10ms, faster than display update)
        buttons_poll();
        buttons_process();
        
        // Update display at DISPLAY_UPDATE_HZ
        if (now - last_update >= DISPLAY_INTERVAL_MS) {
            display_update();
            last_update = now;
        }
        
        // Small sleep to prevent tight loop
        sleep_us(1000);  // 1ms
    }
}

// =============================================================================
// USB CDC Processing
// =============================================================================

static void process_usb_rx(void) {
    // Read available bytes from USB CDC
    while (tud_cdc_available()) {
        uint8_t byte = tud_cdc_read_char();
        
        // Feed to protocol parser
        if (protocol_process_byte(byte)) {
            // Complete packet received
            protocol_handle_packet();
            
            // Flash LED on packet
            display_flash_led(0, 255, 0);  // Green flash
        }
    }
}

// =============================================================================
// Main Loop (Core 0)
// =============================================================================

int main(void) {
    // Initialize stdio (UART for debug)
    stdio_init_all();
    
    // Wait a bit for USB enumeration
    sleep_ms(100);
    
    printf("\n\n=== OpenCNC Test Fixture ===\n");
    printf("Version: %d.%d.%d\n", 
           FIXTURE_VERSION_MAJOR, FIXTURE_VERSION_MINOR, FIXTURE_VERSION_PATCH);
    printf("Core 0: USB + Protocol\n");
    printf("Core 1: Display + Buttons\n\n");
    
    // Initialize TinyUSB
    tusb_init();
    
    // Initialize protocol handler
    protocol_init();
    
    // Initialize emulator
    emulator_init();
    
    // Initialize fixture state
    g_fixture.mode = MODE_NORMAL;
    g_fixture.machine.queue_capacity = MOTION_QUEUE_SIZE;
    g_fixture.machine.feed_override = 1000;  // 100%
    g_fixture.machine.rapid_override = 1000;
    g_fixture.machine.spindle_override = 1000;
    
    // Launch Core 1 for display
    multicore_launch_core1(core1_main);
    
    printf("Core 1 launched\n");
    printf("Waiting for USB connection...\n");
    
    // Main loop on Core 0
    uint32_t loop_count = 0;
    
    while (1) {
        // Update uptime
        g_fixture.uptime_ms = to_ms_since_boot(get_absolute_time());
        
        // TinyUSB device task
        tud_task();
        
        // Process USB receive
        if (tud_cdc_connected()) {
            process_usb_rx();
        }
        
        // Emulator tick at EMULATOR_TICK_HZ
        uint64_t now_us = time_us_64();
        if (now_us - s_last_emulator_time >= EMULATOR_INTERVAL_US) {
            emulator_tick();
            s_last_emulator_time = now_us;
        }
        
        // Status broadcast at STATUS_BROADCAST_HZ
        if (tud_cdc_connected()) {
            if (now_us - s_last_status_time >= STATUS_INTERVAL_US) {
                protocol_send_rt_status();
                s_last_status_time = now_us;
            }
        }
        
        // Watchdog-like timeout check
        if (g_fixture.last_rx_time > 0 && 
            g_fixture.uptime_ms - g_fixture.last_rx_time > 5000) {
            // No packets received for 5 seconds
            // Could trigger comm timeout alarm in real firmware
            // For test fixture, just note it
            static bool timeout_logged = false;
            if (!timeout_logged && g_fixture.machine.state != STATE_IDLE) {
                display_log("Comm timeout (5s)");
                timeout_logged = true;
            }
            if (g_fixture.last_rx_time > 0 && 
                g_fixture.uptime_ms - g_fixture.last_rx_time < 5000) {
                timeout_logged = false;
            }
        }
        
        // Debug output every 5 seconds
        loop_count++;
        if (loop_count % 500000 == 0) {
            printf("Uptime: %lu ms, RX: %lu, TX: %lu, State: %s\n",
                   (unsigned long)g_fixture.uptime_ms,
                   (unsigned long)g_fixture.comm.packets_rx,
                   (unsigned long)g_fixture.comm.packets_tx,
                   emulator_state_name());
        }
        
        // Small yield
        tight_loop_contents();
    }
    
    return 0;
}

// =============================================================================
// TinyUSB Callbacks
// =============================================================================

// Invoked when device is mounted
void tud_mount_cb(void) {
    display_log("USB mounted");
    display_set_led(0, 0, 50);  // Blue = connected
}

// Invoked when device is unmounted
void tud_umount_cb(void) {
    display_log("USB unmounted");
    display_set_led(50, 0, 0);  // Red = disconnected
}

// Invoked when usb bus is suspended
void tud_suspend_cb(bool remote_wakeup_en) {
    (void)remote_wakeup_en;
    display_log("USB suspended");
}

// Invoked when usb bus is resumed
void tud_resume_cb(void) {
    display_log("USB resumed");
}

// Invoked when CDC interface received data
void tud_cdc_rx_cb(uint8_t itf) {
    (void)itf;
    // Data will be processed in main loop
}

// Invoked when CDC line state changes
void tud_cdc_line_state_cb(uint8_t itf, bool dtr, bool rts) {
    (void)itf;
    (void)rts;
    
    if (dtr) {
        // Terminal connected
        display_log("Terminal connected");
    } else {
        display_log("Terminal disconnected");
    }
}
