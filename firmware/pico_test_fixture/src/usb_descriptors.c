/**
 * @file usb_descriptors.c
 * @brief USB descriptors for OpenCNC Test Fixture
 * 
 * Provides custom VID/PID and device strings to identify as a test fixture
 * rather than production CNC hardware.
 */

#include "tusb.h"
#include "pico/unique_id.h"
#include <string.h>
#include <stdio.h>

// =============================================================================
// Device Descriptor
// =============================================================================

tusb_desc_device_t const desc_device = {
    .bLength            = sizeof(tusb_desc_device_t),
    .bDescriptorType    = TUSB_DESC_DEVICE,
    .bcdUSB             = 0x0200,   // USB 2.0
    
    // Use Interface Association Descriptor (IAD) for CDC
    .bDeviceClass       = TUSB_CLASS_MISC,
    .bDeviceSubClass    = MISC_SUBCLASS_COMMON,
    .bDeviceProtocol    = MISC_PROTOCOL_IAD,
    
    .bMaxPacketSize0    = CFG_TUD_ENDPOINT0_SIZE,
    
    .idVendor           = 0xCAFE,   // TinyUSB test VID
    .idProduct          = 0x4001,   // OpenCNC Test Fixture PID
    .bcdDevice          = 0x0100,   // Version 1.00
    
    .iManufacturer      = 0x01,
    .iProduct           = 0x02,
    .iSerialNumber      = 0x03,
    
    .bNumConfigurations = 0x01
};

// Invoked when received GET DEVICE DESCRIPTOR
uint8_t const* tud_descriptor_device_cb(void) {
    return (uint8_t const*)&desc_device;
}

// =============================================================================
// Configuration Descriptor
// =============================================================================

enum {
    ITF_NUM_CDC = 0,
    ITF_NUM_CDC_DATA,
    ITF_NUM_TOTAL
};

#define EPNUM_CDC_NOTIF     0x81
#define EPNUM_CDC_OUT       0x02
#define EPNUM_CDC_IN        0x82

#define CONFIG_TOTAL_LEN    (TUD_CONFIG_DESC_LEN + TUD_CDC_DESC_LEN)

uint8_t const desc_configuration[] = {
    // Config number, interface count, string index, total length, attributes, power in mA
    TUD_CONFIG_DESCRIPTOR(1, ITF_NUM_TOTAL, 0, CONFIG_TOTAL_LEN, 0x00, 100),

    // CDC: Interface number, string index, notification EP, notification EP size, data EP out, data EP in, data EP size
    TUD_CDC_DESCRIPTOR(ITF_NUM_CDC, 4, EPNUM_CDC_NOTIF, 8, EPNUM_CDC_OUT, EPNUM_CDC_IN, 64),
};

// Invoked when received GET CONFIGURATION DESCRIPTOR
uint8_t const* tud_descriptor_configuration_cb(uint8_t index) {
    (void)index;
    return desc_configuration;
}

// =============================================================================
// String Descriptors
// =============================================================================

// String descriptor indices
enum {
    STRID_LANGID = 0,
    STRID_MANUFACTURER,
    STRID_PRODUCT,
    STRID_SERIAL,
    STRID_CDC
};

// Supported language (English)
static uint16_t const desc_langid[] = {
    (TUSB_DESC_STRING << 8) | 4,
    0x0409  // English (US)
};

// String descriptors (UTF-16LE)
static uint16_t desc_str_manufacturer[] = {
    (TUSB_DESC_STRING << 8) | (8 * 2 + 2),
    'O', 'p', 'e', 'n', 'C', 'N', 'C', 0
};

static uint16_t desc_str_product[] = {
    (TUSB_DESC_STRING << 8) | (32 * 2 + 2),
    'T', 'e', 's', 't', ' ', 'F', 'i', 'x', 't', 'u', 'r', 'e', ' ',
    '(', 'P', 'i', 'c', 'o', ' ', 'D', 'i', 's', 'p', 'l', 'a', 'y', ' ', '2', '.', '0', ')', 0
};

static uint16_t desc_str_cdc[] = {
    (TUSB_DESC_STRING << 8) | (16 * 2 + 2),
    'C', 'N', 'C', ' ', 'S', 'e', 'r', 'i', 'a', 'l', ' ', 'P', 'o', 'r', 't', 0
};

// Serial number buffer (filled from Pico unique ID)
static uint16_t desc_str_serial[17];  // "XXXX-XXXX-XXXX-XXXX" + header
static bool serial_initialized = false;

/**
 * @brief Initialize serial number from Pico unique board ID
 */
static void init_serial_number(void) {
    if (serial_initialized) return;
    
    pico_unique_board_id_t board_id;
    pico_get_unique_board_id(&board_id);
    
    // Format: 16 hex chars
    char serial_str[17];
    snprintf(serial_str, sizeof(serial_str), 
             "%02X%02X%02X%02X%02X%02X%02X%02X",
             board_id.id[0], board_id.id[1], board_id.id[2], board_id.id[3],
             board_id.id[4], board_id.id[5], board_id.id[6], board_id.id[7]);
    
    // Convert to UTF-16
    desc_str_serial[0] = (TUSB_DESC_STRING << 8) | (16 * 2 + 2);
    for (int i = 0; i < 16; i++) {
        desc_str_serial[i + 1] = serial_str[i];
    }
    
    serial_initialized = true;
}

// Invoked when received GET STRING DESCRIPTOR
uint16_t const* tud_descriptor_string_cb(uint8_t index, uint16_t langid) {
    (void)langid;
    
    switch (index) {
        case STRID_LANGID:
            return desc_langid;
            
        case STRID_MANUFACTURER:
            return desc_str_manufacturer;
            
        case STRID_PRODUCT:
            return desc_str_product;
            
        case STRID_SERIAL:
            init_serial_number();
            return desc_str_serial;
            
        case STRID_CDC:
            return desc_str_cdc;
            
        default:
            return NULL;
    }
}
