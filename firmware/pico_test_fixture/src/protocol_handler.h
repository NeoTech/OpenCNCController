/**
 * @file protocol_handler.h
 * @brief Protocol parsing and response generation for test fixture
 */

#ifndef PROTOCOL_HANDLER_H
#define PROTOCOL_HANDLER_H

#include <stdint.h>
#include <stdbool.h>
#include "test_fixture.h"

#ifdef __cplusplus
extern "C" {
#endif

// =============================================================================
// Command Definitions (from cnc_protocol.h)
// =============================================================================

// System commands
#define CMD_PING                0x00
#define CMD_PONG                0x01
#define CMD_GET_VERSION         0x02
#define CMD_VERSION_RESPONSE    0x03
#define CMD_RESET               0x04
#define CMD_GET_STATUS          0x05
#define CMD_STATUS_RESPONSE     0x06

// Motion commands
#define CMD_MOVE_LINEAR         0x10
#define CMD_MOVE_RAPID          0x11
#define CMD_MOVE_ARC            0x12
#define CMD_MOVE_DWELL          0x13
#define CMD_JOG_START           0x14
#define CMD_JOG_STOP            0x15

// Control commands
#define CMD_START               0x30
#define CMD_PAUSE               0x31
#define CMD_RESUME              0x32
#define CMD_STOP                0x33
#define CMD_ESTOP               0x34
#define CMD_RESET_ESTOP         0x35

// Homing commands
#define CMD_HOME_AXIS           0x40
#define CMD_HOME_ALL            0x41
#define CMD_PROBE               0x42
#define CMD_SET_POSITION        0x43

// Config commands
#define CMD_GET_CONFIG          0x50
#define CMD_SET_CONFIG          0x51
#define CMD_SAVE_CONFIG         0x52

// Spindle/coolant
#define CMD_SPINDLE_ON          0x60
#define CMD_SPINDLE_OFF         0x61
#define CMD_COOLANT_ON          0x63
#define CMD_COOLANT_OFF         0x64

// I/O
#define CMD_GET_INPUTS          0x70
#define CMD_SET_OUTPUTS         0x71

// Buffer management
#define CMD_QUEUE_STATUS        0x80
#define CMD_CLEAR_QUEUE         0x81
#define CMD_MOTION_SEGMENT      0x82

// Real-time status
#define CMD_RT_POSITION         0xA0
#define CMD_RT_VELOCITY         0xA1
#define CMD_RT_STATUS           0xA2

// Response commands
#define CMD_ACK                 0xF0
#define CMD_NAK                 0xF1
#define CMD_ERROR               0xFE

// =============================================================================
// Packet Parser State
// =============================================================================

typedef enum {
    PARSE_IDLE,
    PARSE_GOT_START,
    PARSE_GOT_CMD,
    PARSE_GOT_SEQ,
    PARSE_GOT_LEN,
    PARSE_PAYLOAD,
    PARSE_CRC_HIGH,
    PARSE_CRC_LOW,
    PARSE_COMPLETE
} parse_state_t;

typedef struct {
    parse_state_t   state;
    uint8_t         cmd;
    uint8_t         seq;
    uint8_t         len;
    uint8_t         payload[PAYLOAD_MAX_SIZE];
    uint8_t         payload_idx;
    uint16_t        crc_received;
    uint16_t        crc_calculated;
} packet_parser_t;

// =============================================================================
// Function Prototypes
// =============================================================================

/**
 * @brief Initialize protocol handler
 */
void protocol_init(void);

/**
 * @brief Process incoming byte from USB CDC
 * @param byte Received byte
 * @return true if a complete valid packet was received
 */
bool protocol_process_byte(uint8_t byte);

/**
 * @brief Get the last received command (after protocol_process_byte returns true)
 * @return Command byte
 */
uint8_t protocol_get_last_cmd(void);

/**
 * @brief Get the last received sequence number
 * @return Sequence number
 */
uint8_t protocol_get_last_seq(void);

/**
 * @brief Get pointer to payload buffer
 * @return Pointer to payload data
 */
const uint8_t* protocol_get_payload(void);

/**
 * @brief Get payload length
 * @return Payload length in bytes
 */
uint8_t protocol_get_payload_len(void);

/**
 * @brief Handle a complete received packet
 * Dispatches to appropriate handler and generates response
 */
void protocol_handle_packet(void);

/**
 * @brief Send ACK response
 * @param seq Sequence number to acknowledge
 */
void protocol_send_ack(uint8_t seq);

/**
 * @brief Send NAK response
 * @param seq Sequence number
 * @param reason Error reason code
 */
void protocol_send_nak(uint8_t seq, uint8_t reason);

/**
 * @brief Send version response
 * @param seq Sequence number
 */
void protocol_send_version(uint8_t seq);

/**
 * @brief Send real-time status packet
 * Called at STATUS_BROADCAST_HZ rate
 */
void protocol_send_rt_status(void);

/**
 * @brief Build and send a packet
 * @param cmd Command byte
 * @param seq Sequence number
 * @param payload Payload data (NULL if none)
 * @param len Payload length
 */
void protocol_send_packet(uint8_t cmd, uint8_t seq, const uint8_t* payload, uint8_t len);

/**
 * @brief Calculate CRC-16-CCITT
 * @param data Data buffer
 * @param len Data length
 * @return CRC-16 value
 */
uint16_t protocol_crc16(const uint8_t* data, size_t len);

/**
 * @brief Get command name string for logging
 * @param cmd Command byte
 * @return Human-readable command name
 */
const char* protocol_cmd_name(uint8_t cmd);

#ifdef __cplusplus
}
#endif

#endif // PROTOCOL_HANDLER_H
