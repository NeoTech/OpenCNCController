/**
 * @file sdo.h
 * @brief CANopen Service Data Object (SDO) Protocol
 * 
 * SDO provides client/server access to the Object Dictionary.
 * Used for configuration, parameter access, and diagnostics.
 * 
 * Supports expedited transfer (≤4 bytes) and segmented transfer.
 * CAN-FD allows larger expedited transfers up to 64 bytes.
 */

#ifndef SDO_H
#define SDO_H

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "canopen_types.h"

#ifdef __cplusplus
extern "C" {
#endif

// =============================================================================
// SDO Configuration
// =============================================================================

#define SDO_TIMEOUT_MS          1000        // Default timeout for SDO transfers
#define SDO_MAX_SEGMENT_SIZE    7           // CAN 2.0: 7 bytes per segment
#define SDO_FD_MAX_DATA_SIZE    56          // CAN-FD: 64 - 8 byte header

// =============================================================================
// SDO Transfer Types
// =============================================================================

typedef enum {
    SDO_TRANSFER_EXPEDITED,                 // ≤4 bytes in single frame
    SDO_TRANSFER_SEGMENTED,                 // Multi-frame transfer
    SDO_TRANSFER_BLOCK                      // Block transfer (not implemented)
} sdo_transfer_type_t;

// =============================================================================
// SDO State Machine
// =============================================================================

typedef enum {
    SDO_STATE_IDLE,
    SDO_STATE_DOWNLOAD_INITIATED,           // Upload from master view
    SDO_STATE_DOWNLOAD_SEGMENTED,
    SDO_STATE_UPLOAD_INITIATED,             // Download from master view  
    SDO_STATE_UPLOAD_SEGMENTED,
    SDO_STATE_COMPLETE,
    SDO_STATE_ABORTED
} sdo_state_t;

// =============================================================================
// SDO Transfer Context
// =============================================================================

typedef struct {
    // Transfer identification
    uint16_t    index;
    uint8_t     subindex;
    uint8_t     node_id;
    
    // Transfer state
    sdo_state_t state;
    bool        is_read;                    // true = read from server
    
    // Data buffer
    uint8_t*    data;
    uint32_t    data_size;                  // Total size
    uint32_t    transferred;                // Bytes transferred so far
    
    // Segmented transfer
    bool        toggle;                     // Toggle bit for segments
    
    // Timing
    uint32_t    start_tick;
    uint32_t    timeout_ms;
    
    // Result
    uint32_t    abort_code;
    
} sdo_transfer_t;

// =============================================================================
// SDO Client (Master Side)
// =============================================================================

typedef struct {
    sdo_transfer_t  current;
    uint8_t         rx_buffer[256];         // For segmented reads
    uint8_t         tx_buffer[256];         // For segmented writes
    
    // Callbacks
    void (*on_complete)(uint16_t index, uint8_t subindex, const uint8_t* data, uint32_t size);
    void (*on_abort)(uint16_t index, uint8_t subindex, uint32_t abort_code);
    
} sdo_client_t;

// =============================================================================
// SDO Server (Slave Side)
// =============================================================================

// Object access callback for SDO server
typedef bool (*sdo_access_callback_t)(
    uint16_t index,
    uint8_t subindex,
    bool is_write,
    uint8_t* data,
    uint32_t* size,
    uint32_t* abort_code
);

typedef struct {
    uint8_t                 node_id;
    sdo_transfer_t          current;
    uint8_t                 segment_buffer[256];
    sdo_access_callback_t   access_callback;
    
} sdo_server_t;

// =============================================================================
// SDO Frame Building - Client (Master)
// =============================================================================

/**
 * @brief Build expedited download initiate request (write ≤4 bytes)
 */
static inline void sdo_build_download_expedited(
    uint8_t node_id,
    uint16_t index,
    uint8_t subindex,
    const uint8_t* data,
    uint8_t size,
    can_frame_t* frame
) {
    frame->id = CANOPEN_COB_ID(CANOPEN_FC_SDO_RX, node_id);
    frame->dlc = 8;
    frame->is_fd = false;
    frame->is_extended = false;
    
    uint8_t cs = SDO_CCS_DOWNLOAD_INIT | SDO_EXPEDITED | SDO_SIZE_INDICATED;
    cs |= ((4 - size) << 2);  // n = 4 - size
    
    frame->data[0] = cs;
    frame->data[1] = index & 0xFF;
    frame->data[2] = (index >> 8) & 0xFF;
    frame->data[3] = subindex;
    
    memset(&frame->data[4], 0, 4);
    if (data && size <= 4) {
        memcpy(&frame->data[4], data, size);
    }
}

/**
 * @brief Build upload initiate request (read request)
 */
static inline void sdo_build_upload_request(
    uint8_t node_id,
    uint16_t index,
    uint8_t subindex,
    can_frame_t* frame
) {
    frame->id = CANOPEN_COB_ID(CANOPEN_FC_SDO_RX, node_id);
    frame->dlc = 8;
    frame->is_fd = false;
    frame->is_extended = false;
    
    frame->data[0] = SDO_CCS_UPLOAD_INIT;
    frame->data[1] = index & 0xFF;
    frame->data[2] = (index >> 8) & 0xFF;
    frame->data[3] = subindex;
    memset(&frame->data[4], 0, 4);
}

/**
 * @brief Build upload segment request
 */
static inline void sdo_build_upload_segment(
    uint8_t node_id,
    bool toggle,
    can_frame_t* frame
) {
    frame->id = CANOPEN_COB_ID(CANOPEN_FC_SDO_RX, node_id);
    frame->dlc = 8;
    frame->is_fd = false;
    frame->is_extended = false;
    
    frame->data[0] = SDO_CCS_UPLOAD_SEGMENT | (toggle ? 0x10 : 0x00);
    memset(&frame->data[1], 0, 7);
}

/**
 * @brief Build download segment request
 */
static inline void sdo_build_download_segment(
    uint8_t node_id,
    bool toggle,
    bool last,
    const uint8_t* data,
    uint8_t size,
    can_frame_t* frame
) {
    frame->id = CANOPEN_COB_ID(CANOPEN_FC_SDO_RX, node_id);
    frame->dlc = 8;
    frame->is_fd = false;
    frame->is_extended = false;
    
    uint8_t cs = SDO_CCS_DOWNLOAD_SEGMENT;
    cs |= (toggle ? 0x10 : 0x00);
    cs |= ((7 - size) << 1);  // n = 7 - size
    cs |= (last ? 0x01 : 0x00);
    
    frame->data[0] = cs;
    memset(&frame->data[1], 0, 7);
    if (data && size <= 7) {
        memcpy(&frame->data[1], data, size);
    }
}

/**
 * @brief Build abort transfer frame
 */
static inline void sdo_build_abort(
    uint8_t node_id,
    uint16_t index,
    uint8_t subindex,
    uint32_t abort_code,
    bool from_client,
    can_frame_t* frame
) {
    uint16_t fc = from_client ? CANOPEN_FC_SDO_RX : CANOPEN_FC_SDO_TX;
    frame->id = CANOPEN_COB_ID(fc, node_id);
    frame->dlc = 8;
    frame->is_fd = false;
    frame->is_extended = false;
    
    frame->data[0] = SDO_CCS_ABORT;
    frame->data[1] = index & 0xFF;
    frame->data[2] = (index >> 8) & 0xFF;
    frame->data[3] = subindex;
    frame->data[4] = abort_code & 0xFF;
    frame->data[5] = (abort_code >> 8) & 0xFF;
    frame->data[6] = (abort_code >> 16) & 0xFF;
    frame->data[7] = (abort_code >> 24) & 0xFF;
}

// =============================================================================
// SDO Frame Parsing
// =============================================================================

/**
 * @brief Parse SDO response header
 */
static inline bool sdo_parse_header(
    const can_frame_t* frame,
    uint8_t* cs,
    uint16_t* index,
    uint8_t* subindex
) {
    if (frame->dlc < 4) return false;
    
    *cs = frame->data[0];
    *index = frame->data[1] | (frame->data[2] << 8);
    *subindex = frame->data[3];
    return true;
}

/**
 * @brief Check if response is abort
 */
static inline bool sdo_is_abort(const can_frame_t* frame, uint32_t* abort_code) {
    if (frame->dlc < 8) return false;
    if ((frame->data[0] & 0xE0) != 0x80) return false;
    
    *abort_code = frame->data[4] |
                  (frame->data[5] << 8) |
                  (frame->data[6] << 16) |
                  (frame->data[7] << 24);
    return true;
}

/**
 * @brief Parse expedited upload response
 */
static inline bool sdo_parse_upload_expedited(
    const can_frame_t* frame,
    uint8_t* data,
    uint8_t* size
) {
    uint8_t cs = frame->data[0];
    
    // Check for upload initiate response
    if ((cs & 0xE0) != 0x40) return false;
    
    // Check expedited bit
    if (!(cs & SDO_EXPEDITED)) return false;
    
    // Get size
    if (cs & SDO_SIZE_INDICATED) {
        uint8_t n = (cs >> 2) & 0x03;
        *size = 4 - n;
    } else {
        *size = 4;
    }
    
    memcpy(data, &frame->data[4], *size);
    return true;
}

// =============================================================================
// SDO Client Functions
// =============================================================================

/**
 * @brief Initialize SDO client
 */
void sdo_client_init(sdo_client_t* client);

/**
 * @brief Start expedited read (≤4 bytes)
 * @return true if transfer started
 */
bool sdo_client_read(
    sdo_client_t* client,
    uint8_t node_id,
    uint16_t index,
    uint8_t subindex,
    void (*tx_callback)(const can_frame_t* frame)
);

/**
 * @brief Start expedited write (≤4 bytes)
 * @return true if transfer started
 */
bool sdo_client_write(
    sdo_client_t* client,
    uint8_t node_id,
    uint16_t index,
    uint8_t subindex,
    const void* data,
    uint32_t size,
    void (*tx_callback)(const can_frame_t* frame)
);

/**
 * @brief Process received SDO response
 */
void sdo_client_process(
    sdo_client_t* client,
    const can_frame_t* frame,
    void (*tx_callback)(const can_frame_t* frame)
);

/**
 * @brief Check for timeout
 * @param delta_ms Milliseconds since last tick
 */
void sdo_client_tick(sdo_client_t* client, uint32_t delta_ms);

/**
 * @brief Abort current transfer
 */
void sdo_client_abort(
    sdo_client_t* client,
    uint32_t abort_code,
    void (*tx_callback)(const can_frame_t* frame)
);

/**
 * @brief Check if client is idle
 */
static inline bool sdo_client_is_idle(const sdo_client_t* client) {
    return client->current.state == SDO_STATE_IDLE ||
           client->current.state == SDO_STATE_COMPLETE ||
           client->current.state == SDO_STATE_ABORTED;
}

// =============================================================================
// SDO Server Functions
// =============================================================================

/**
 * @brief Initialize SDO server
 * @param node_id This node's ID
 * @param access_cb Callback for object dictionary access
 */
void sdo_server_init(sdo_server_t* server, uint8_t node_id, sdo_access_callback_t access_cb);

/**
 * @brief Process received SDO request
 */
void sdo_server_process(
    sdo_server_t* server,
    const can_frame_t* frame,
    void (*tx_callback)(const can_frame_t* frame)
);

/**
 * @brief Check for timeout on segmented transfers
 */
void sdo_server_tick(sdo_server_t* server, uint32_t delta_ms);

// =============================================================================
// Utility: Read/Write typed values
// =============================================================================

/**
 * @brief Helper to read uint32 via SDO
 */
static inline bool sdo_client_read_u32(
    sdo_client_t* client,
    uint8_t node_id,
    uint16_t index,
    uint8_t subindex,
    void (*tx_callback)(const can_frame_t* frame)
) {
    return sdo_client_read(client, node_id, index, subindex, tx_callback);
}

/**
 * @brief Helper to write uint32 via SDO
 */
static inline bool sdo_client_write_u32(
    sdo_client_t* client,
    uint8_t node_id,
    uint16_t index,
    uint8_t subindex,
    uint32_t value,
    void (*tx_callback)(const can_frame_t* frame)
) {
    return sdo_client_write(client, node_id, index, subindex, &value, sizeof(value), tx_callback);
}

/**
 * @brief Helper to write int32 via SDO
 */
static inline bool sdo_client_write_i32(
    sdo_client_t* client,
    uint8_t node_id,
    uint16_t index,
    uint8_t subindex,
    int32_t value,
    void (*tx_callback)(const can_frame_t* frame)
) {
    return sdo_client_write(client, node_id, index, subindex, &value, sizeof(value), tx_callback);
}

/**
 * @brief Helper to write uint16 via SDO
 */
static inline bool sdo_client_write_u16(
    sdo_client_t* client,
    uint8_t node_id,
    uint16_t index,
    uint8_t subindex,
    uint16_t value,
    void (*tx_callback)(const can_frame_t* frame)
) {
    return sdo_client_write(client, node_id, index, subindex, &value, sizeof(value), tx_callback);
}

#ifdef __cplusplus
}
#endif

#endif // SDO_H
