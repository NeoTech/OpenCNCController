/**
 * @file tusb_config.h
 * @brief TinyUSB configuration for OpenCNC Test Fixture
 * 
 * Configures USB CDC device for serial communication with Windows HMI
 */

#ifndef TUSB_CONFIG_H
#define TUSB_CONFIG_H

#ifdef __cplusplus
extern "C" {
#endif

// =============================================================================
// Common Configuration
// =============================================================================

// RHPort number used for device (0 for Pico)
#define BOARD_TUD_RHPORT        0

// RHPort max operational speed
#define BOARD_TUD_MAX_SPEED     OPT_MODE_FULL_SPEED

// =============================================================================
// Device Configuration
// =============================================================================

// Device mode enabled
#define CFG_TUD_ENABLED         1

// High speed not available on RP2040
#define CFG_TUD_MAX_SPEED       OPT_MODE_FULL_SPEED

// =============================================================================
// Device Descriptor
// =============================================================================

// USB VID/PID - Using TinyUSB test VID with custom PID
// VID 0xCAFE is reserved for TinyUSB examples/testing
// PID 0x4001 is unique for OpenCNC Test Fixture
#define CFG_TUD_VID             0xCAFE
#define CFG_TUD_PID             0x4001

// =============================================================================
// Class Configuration
// =============================================================================

// CDC - Communication Device Class (serial port)
#define CFG_TUD_CDC             1
#define CFG_TUD_CDC_RX_BUFSIZE  512
#define CFG_TUD_CDC_TX_BUFSIZE  512

// Disable other classes we don't need
#define CFG_TUD_MSC             0
#define CFG_TUD_HID             0
#define CFG_TUD_MIDI            0
#define CFG_TUD_VENDOR          0

// =============================================================================
// Endpoint Configuration
// =============================================================================

// CDC requires 3 endpoints: notification, data in, data out
// EP0 is reserved for control
// EP1 = CDC notification (interrupt)
// EP2 = CDC data (bulk)
#define CFG_TUD_ENDPOINT0_SIZE  64

// =============================================================================
// Memory Configuration
// =============================================================================

// Use static memory allocation (no malloc)
#define CFG_TUSB_MEM_SECTION
#define CFG_TUSB_MEM_ALIGN      __attribute__((aligned(4)))

// =============================================================================
// OS Configuration
// =============================================================================

// No RTOS - bare metal
#define CFG_TUSB_OS             OPT_OS_NONE

// =============================================================================
// Debug Configuration
// =============================================================================

// Debug level: 0=none, 1=error, 2=warning, 3=info
#ifndef CFG_TUSB_DEBUG
#define CFG_TUSB_DEBUG          0
#endif

#ifdef __cplusplus
}
#endif

#endif // TUSB_CONFIG_H
