/**
 * @file protocol_handler.c
 * @brief Protocol parsing and response generation for test fixture
 */

#include "protocol_handler.h"
#include "emulator.h"
#include "display.h"
#include "tusb.h"
#include <string.h>
#include <stdio.h>

// =============================================================================
// Static Variables
// =============================================================================

static packet_parser_t s_parser;
static uint8_t s_tx_buffer[PACKET_MAX_SIZE];
static uint8_t s_tx_seq = 0;

// =============================================================================
// CRC-16-CCITT Implementation
// =============================================================================

// Polynomial: 0x1021, Init: 0xFFFF
static const uint16_t crc16_table[256] = {
    0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50A5, 0x60C6, 0x70E7,
    0x8108, 0x9129, 0xA14A, 0xB16B, 0xC18C, 0xD1AD, 0xE1CE, 0xF1EF,
    0x1231, 0x0210, 0x3273, 0x2252, 0x52B5, 0x4294, 0x72F7, 0x62D6,
    0x9339, 0x8318, 0xB37B, 0xA35A, 0xD3BD, 0xC39C, 0xF3FF, 0xE3DE,
    0x2462, 0x3443, 0x0420, 0x1401, 0x64E6, 0x74C7, 0x44A4, 0x5485,
    0xA56A, 0xB54B, 0x8528, 0x9509, 0xE5EE, 0xF5CF, 0xC5AC, 0xD58D,
    0x3653, 0x2672, 0x1611, 0x0630, 0x76D7, 0x66F6, 0x5695, 0x46B4,
    0xB75B, 0xA77A, 0x9719, 0x8738, 0xF7DF, 0xE7FE, 0xD79D, 0xC7BC,
    0x48C4, 0x58E5, 0x6886, 0x78A7, 0x0840, 0x1861, 0x2802, 0x3823,
    0xC9CC, 0xD9ED, 0xE98E, 0xF9AF, 0x8948, 0x9969, 0xA90A, 0xB92B,
    0x5AF5, 0x4AD4, 0x7AB7, 0x6A96, 0x1A71, 0x0A50, 0x3A33, 0x2A12,
    0xDBFD, 0xCBDC, 0xFBBF, 0xEB9E, 0x9B79, 0x8B58, 0xBB3B, 0xAB1A,
    0x6CA6, 0x7C87, 0x4CE4, 0x5CC5, 0x2C22, 0x3C03, 0x0C60, 0x1C41,
    0xEDAE, 0xFD8F, 0xCDEC, 0xDDCD, 0xAD2A, 0xBD0B, 0x8D68, 0x9D49,
    0x7E97, 0x6EB6, 0x5ED5, 0x4EF4, 0x3E13, 0x2E32, 0x1E51, 0x0E70,
    0xFF9F, 0xEFBE, 0xDFDD, 0xCFFC, 0xBF1B, 0xAF3A, 0x9F59, 0x8F78,
    0x9188, 0x81A9, 0xB1CA, 0xA1EB, 0xD10C, 0xC12D, 0xF14E, 0xE16F,
    0x1080, 0x00A1, 0x30C2, 0x20E3, 0x5004, 0x4025, 0x7046, 0x6067,
    0x83B9, 0x9398, 0xA3FB, 0xB3DA, 0xC33D, 0xD31C, 0xE37F, 0xF35E,
    0x02B1, 0x1290, 0x22F3, 0x32D2, 0x4235, 0x5214, 0x6277, 0x7256,
    0xB5EA, 0xA5CB, 0x95A8, 0x8589, 0xF56E, 0xE54F, 0xD52C, 0xC50D,
    0x34E2, 0x24C3, 0x14A0, 0x0481, 0x7466, 0x6447, 0x5424, 0x4405,
    0xA7DB, 0xB7FA, 0x8799, 0x97B8, 0xE75F, 0xF77E, 0xC71D, 0xD73C,
    0x26D3, 0x36F2, 0x0691, 0x16B0, 0x6657, 0x7676, 0x4615, 0x5634,
    0xD94C, 0xC96D, 0xF90E, 0xE92F, 0x99C8, 0x89E9, 0xB98A, 0xA9AB,
    0x5844, 0x4865, 0x7806, 0x6827, 0x18C0, 0x08E1, 0x3882, 0x28A3,
    0xCB7D, 0xDB5C, 0xEB3F, 0xFB1E, 0x8BF9, 0x9BD8, 0xABBB, 0xBB9A,
    0x4A75, 0x5A54, 0x6A37, 0x7A16, 0x0AF1, 0x1AD0, 0x2AB3, 0x3A92,
    0xFD2E, 0xED0F, 0xDD6C, 0xCD4D, 0xBDAA, 0xAD8B, 0x9DE8, 0x8DC9,
    0x7C26, 0x6C07, 0x5C64, 0x4C45, 0x3CA2, 0x2C83, 0x1CE0, 0x0CC1,
    0xEF1F, 0xFF3E, 0xCF5D, 0xDF7C, 0xAF9B, 0xBFBA, 0x8FD9, 0x9FF8,
    0x6E17, 0x7E36, 0x4E55, 0x5E74, 0x2E93, 0x3EB2, 0x0ED1, 0x1EF0
};

uint16_t protocol_crc16(const uint8_t* data, size_t len) {
    uint16_t crc = 0xFFFF;
    
    for (size_t i = 0; i < len; i++) {
        crc = (crc << 8) ^ crc16_table[((crc >> 8) ^ data[i]) & 0xFF];
    }
    
    return crc;
}

// =============================================================================
// Command Name Lookup
// =============================================================================

const char* protocol_cmd_name(uint8_t cmd) {
    switch (cmd) {
        case CMD_PING:              return "PING";
        case CMD_PONG:              return "PONG";
        case CMD_GET_VERSION:       return "GET_VERSION";
        case CMD_VERSION_RESPONSE:  return "VERSION_RSP";
        case CMD_RESET:             return "RESET";
        case CMD_GET_STATUS:        return "GET_STATUS";
        case CMD_STATUS_RESPONSE:   return "STATUS_RSP";
        case CMD_MOVE_LINEAR:       return "MOVE_LINEAR";
        case CMD_MOVE_RAPID:        return "MOVE_RAPID";
        case CMD_MOVE_ARC:          return "MOVE_ARC";
        case CMD_MOVE_DWELL:        return "DWELL";
        case CMD_JOG_START:         return "JOG_START";
        case CMD_JOG_STOP:          return "JOG_STOP";
        case CMD_START:             return "START";
        case CMD_PAUSE:             return "PAUSE";
        case CMD_RESUME:            return "RESUME";
        case CMD_STOP:              return "STOP";
        case CMD_ESTOP:             return "ESTOP";
        case CMD_RESET_ESTOP:       return "RESET_ESTOP";
        case CMD_HOME_AXIS:         return "HOME_AXIS";
        case CMD_HOME_ALL:          return "HOME_ALL";
        case CMD_PROBE:             return "PROBE";
        case CMD_SET_POSITION:      return "SET_POS";
        case CMD_QUEUE_STATUS:      return "QUEUE_STATUS";
        case CMD_CLEAR_QUEUE:       return "CLEAR_QUEUE";
        case CMD_MOTION_SEGMENT:    return "MOTION_SEG";
        case CMD_RT_POSITION:       return "RT_POSITION";
        case CMD_RT_VELOCITY:       return "RT_VELOCITY";
        case CMD_RT_STATUS:         return "RT_STATUS";
        case CMD_SPINDLE_ON:        return "SPINDLE_ON";
        case CMD_SPINDLE_OFF:       return "SPINDLE_OFF";
        case CMD_COOLANT_ON:        return "COOLANT_ON";
        case CMD_COOLANT_OFF:       return "COOLANT_OFF";
        case CMD_ACK:               return "ACK";
        case CMD_NAK:               return "NAK";
        case CMD_ERROR:             return "ERROR";
        default:                    return "UNKNOWN";
    }
}

// =============================================================================
// Protocol Initialization
// =============================================================================

void protocol_init(void) {
    memset(&s_parser, 0, sizeof(s_parser));
    s_parser.state = PARSE_IDLE;
    s_tx_seq = 0;
}

// =============================================================================
// Packet Parsing
// =============================================================================

bool protocol_process_byte(uint8_t byte) {
    switch (s_parser.state) {
        case PARSE_IDLE:
            if (byte == PACKET_START_BYTE) {
                s_parser.state = PARSE_GOT_START;
                s_parser.payload_idx = 0;
            }
            break;
            
        case PARSE_GOT_START:
            s_parser.cmd = byte;
            s_parser.state = PARSE_GOT_CMD;
            break;
            
        case PARSE_GOT_CMD:
            s_parser.seq = byte;
            s_parser.state = PARSE_GOT_SEQ;
            break;
            
        case PARSE_GOT_SEQ:
            s_parser.len = byte;
            if (s_parser.len == 0) {
                s_parser.state = PARSE_CRC_HIGH;
            } else if (s_parser.len > PAYLOAD_MAX_SIZE) {
                // Invalid length, reset
                s_parser.state = PARSE_IDLE;
            } else {
                s_parser.state = PARSE_PAYLOAD;
            }
            break;
            
        case PARSE_PAYLOAD:
            s_parser.payload[s_parser.payload_idx++] = byte;
            if (s_parser.payload_idx >= s_parser.len) {
                s_parser.state = PARSE_CRC_HIGH;
            }
            break;
            
        case PARSE_CRC_HIGH:
            s_parser.crc_received = (uint16_t)byte << 8;
            s_parser.state = PARSE_CRC_LOW;
            break;
            
        case PARSE_CRC_LOW:
            s_parser.crc_received |= byte;
            // Expect end byte next, but accept packet here
            s_parser.state = PARSE_COMPLETE;
            
            // Calculate CRC over cmd + seq + len + payload
            uint8_t crc_data[4 + PAYLOAD_MAX_SIZE];
            crc_data[0] = s_parser.cmd;
            crc_data[1] = s_parser.seq;
            crc_data[2] = s_parser.len;
            memcpy(&crc_data[3], s_parser.payload, s_parser.len);
            s_parser.crc_calculated = protocol_crc16(crc_data, 3 + s_parser.len);
            
            if (s_parser.crc_calculated == s_parser.crc_received) {
                // Valid packet!
                g_fixture.comm.packets_rx++;
                g_fixture.comm.last_seq_rx = s_parser.seq;
                g_fixture.comm.last_cmd = s_parser.cmd;
                g_fixture.last_rx_time = g_fixture.uptime_ms;
                s_parser.state = PARSE_IDLE;
                return true;
            } else {
                // CRC error
                g_fixture.comm.crc_errors++;
                s_parser.state = PARSE_IDLE;
            }
            break;
            
        case PARSE_COMPLETE:
            // Should have gotten end byte, reset
            s_parser.state = PARSE_IDLE;
            break;
    }
    
    return false;
}

uint8_t protocol_get_last_cmd(void) {
    return s_parser.cmd;
}

uint8_t protocol_get_last_seq(void) {
    return s_parser.seq;
}

const uint8_t* protocol_get_payload(void) {
    return s_parser.payload;
}

uint8_t protocol_get_payload_len(void) {
    return s_parser.len;
}

// =============================================================================
// Packet Transmission
// =============================================================================

void protocol_send_packet(uint8_t cmd, uint8_t seq, const uint8_t* payload, uint8_t len) {
    uint8_t idx = 0;
    
    s_tx_buffer[idx++] = PACKET_START_BYTE;
    s_tx_buffer[idx++] = cmd;
    s_tx_buffer[idx++] = seq;
    s_tx_buffer[idx++] = len;
    
    if (payload && len > 0) {
        memcpy(&s_tx_buffer[idx], payload, len);
        idx += len;
    }
    
    // Calculate CRC
    uint8_t crc_data[3 + PAYLOAD_MAX_SIZE];
    crc_data[0] = cmd;
    crc_data[1] = seq;
    crc_data[2] = len;
    if (len > 0) {
        memcpy(&crc_data[3], payload, len);
    }
    uint16_t crc = protocol_crc16(crc_data, 3 + len);
    
    s_tx_buffer[idx++] = (crc >> 8) & 0xFF;
    s_tx_buffer[idx++] = crc & 0xFF;
    s_tx_buffer[idx++] = PACKET_END_BYTE;
    
    // Send via TinyUSB CDC
    if (tud_cdc_connected()) {
        tud_cdc_write(s_tx_buffer, idx);
        tud_cdc_write_flush();
        g_fixture.comm.packets_tx++;
        g_fixture.comm.last_seq_tx = seq;
    }
}

void protocol_send_ack(uint8_t seq) {
    protocol_send_packet(CMD_ACK, seq, NULL, 0);
}

void protocol_send_nak(uint8_t seq, uint8_t reason) {
    g_fixture.comm.nak_count++;
    protocol_send_packet(CMD_NAK, seq, &reason, 1);
}

void protocol_send_version(uint8_t seq) {
    // VersionResponse structure (20 bytes)
    uint8_t payload[20] = {0};
    
    payload[0] = 1;                         // protocol_version major
    payload[1] = 0;                         // protocol_version minor
    payload[2] = FIXTURE_VERSION_MAJOR;     // firmware_version major
    payload[3] = FIXTURE_VERSION_MINOR;     // firmware_version minor
    payload[4] = FIXTURE_VERSION_PATCH;     // firmware_version patch
    payload[5] = 0xFF;                      // board_type: TEST_FIXTURE
    payload[6] = 3;                         // num_axes (X, Y, Z)
    payload[7] = 0;                         // reserved
    
    // Features bitmask
    uint32_t features = 0x0001;             // Basic motion support
    payload[8] = features & 0xFF;
    payload[9] = (features >> 8) & 0xFF;
    payload[10] = (features >> 16) & 0xFF;
    payload[11] = (features >> 24) & 0xFF;
    
    // Build date (placeholder)
    payload[12] = 26;   // Year - 2000
    payload[13] = 1;    // Month
    payload[14] = 11;   // Day
    
    protocol_send_packet(CMD_VERSION_RESPONSE, seq, payload, 20);
}

void protocol_send_rt_status(void) {
    // RealtimeStatus structure (32 bytes)
    uint8_t payload[32] = {0};
    
    payload[0] = (uint8_t)g_fixture.machine.state;
    payload[1] = (uint8_t)g_fixture.machine.alarm;
    payload[2] = g_fixture.machine.limit_switches;
    payload[3] = g_fixture.machine.queue_depth;
    
    // 6 axis positions (int32_t each = 24 bytes)
    for (int i = 0; i < 6; i++) {
        int32_t pos = (i < MAX_AXES) ? g_fixture.machine.position_nm[i] : 0;
        payload[4 + i*4 + 0] = pos & 0xFF;
        payload[4 + i*4 + 1] = (pos >> 8) & 0xFF;
        payload[4 + i*4 + 2] = (pos >> 16) & 0xFF;
        payload[4 + i*4 + 3] = (pos >> 24) & 0xFF;
    }
    
    // Feed override
    payload[28] = g_fixture.machine.feed_override & 0xFF;
    payload[29] = (g_fixture.machine.feed_override >> 8) & 0xFF;
    
    // Flags
    uint8_t flags = 0;
    if (g_fixture.machine.in_motion) flags |= 0x01;
    if (g_fixture.machine.homing_active) flags |= 0x02;
    if (g_fixture.machine.probe_active) flags |= 0x04;
    if (g_fixture.machine.spindle_on) flags |= 0x08;
    if (g_fixture.machine.coolant_flood) flags |= 0x10;
    if (g_fixture.machine.coolant_mist) flags |= 0x20;
    payload[30] = flags;
    payload[31] = 0;  // Reserved
    
    protocol_send_packet(CMD_RT_STATUS, s_tx_seq++, payload, 32);
}

// =============================================================================
// Packet Handling / Command Dispatch
// =============================================================================

void protocol_handle_packet(void) {
    uint8_t cmd = protocol_get_last_cmd();
    uint8_t seq = protocol_get_last_seq();
    const uint8_t* payload = protocol_get_payload();
    uint8_t len = protocol_get_payload_len();
    
    // Log the command
    char log_msg[LOG_LINE_LENGTH];
    snprintf(log_msg, sizeof(log_msg), "RX: %s (seq=%02X)", 
             protocol_cmd_name(cmd), seq);
    display_log(log_msg);
    
    switch (cmd) {
        case CMD_PING:
            protocol_send_packet(CMD_PONG, seq, NULL, 0);
            break;
            
        case CMD_GET_VERSION:
            protocol_send_version(seq);
            break;
            
        case CMD_GET_STATUS:
            protocol_send_rt_status();
            break;
            
        case CMD_RESET:
            emulator_reset();
            protocol_send_ack(seq);
            display_log("System reset");
            break;
            
        case CMD_START:
            emulator_start();
            protocol_send_ack(seq);
            break;
            
        case CMD_PAUSE:
            emulator_pause();
            protocol_send_ack(seq);
            break;
            
        case CMD_RESUME:
            emulator_resume();
            protocol_send_ack(seq);
            break;
            
        case CMD_STOP:
            emulator_stop();
            protocol_send_ack(seq);
            break;
            
        case CMD_ESTOP:
            emulator_estop();
            protocol_send_ack(seq);
            display_log("!! E-STOP triggered !!");
            break;
            
        case CMD_RESET_ESTOP:
            emulator_clear_estop();
            protocol_send_ack(seq);
            display_log("E-STOP cleared");
            break;
            
        case CMD_HOME_ALL:
            emulator_start_homing(-1);  // -1 = all axes
            protocol_send_ack(seq);
            break;
            
        case CMD_HOME_AXIS:
            if (len >= 1) {
                emulator_start_homing(payload[0]);
                protocol_send_ack(seq);
            } else {
                protocol_send_nak(seq, 0x01);  // Invalid payload
            }
            break;
            
        case CMD_JOG_START:
            if (len >= 5) {
                uint8_t axis = payload[0];
                int32_t velocity = payload[1] | (payload[2] << 8) | 
                                   (payload[3] << 16) | (payload[4] << 24);
                emulator_jog_start(axis, velocity);
                protocol_send_ack(seq);
            } else {
                protocol_send_nak(seq, 0x01);
            }
            break;
            
        case CMD_JOG_STOP:
            emulator_jog_stop();
            protocol_send_ack(seq);
            break;
            
        case CMD_MOTION_SEGMENT:
            // Add to motion queue
            if (emulator_queue_motion(payload, len)) {
                protocol_send_ack(seq);
            } else {
                protocol_send_nak(seq, 0x02);  // Queue full
            }
            break;
            
        case CMD_QUEUE_STATUS: {
            uint8_t status[4];
            status[0] = g_fixture.machine.queue_depth;
            status[1] = g_fixture.machine.queue_capacity;
            status[2] = g_fixture.machine.in_motion ? 1 : 0;
            status[3] = 0;
            protocol_send_packet(CMD_QUEUE_STATUS, seq, status, 4);
            break;
        }
            
        case CMD_CLEAR_QUEUE:
            emulator_clear_queue();
            protocol_send_ack(seq);
            display_log("Queue cleared");
            break;
            
        case CMD_SET_POSITION:
            if (len >= 13) {  // axis (1) + position (4) * 3 axes
                // Set work position offset
                protocol_send_ack(seq);
                display_log("Position set");
            } else {
                protocol_send_nak(seq, 0x01);
            }
            break;
            
        case CMD_SPINDLE_ON:
            g_fixture.machine.spindle_on = true;
            if (len >= 2) {
                g_fixture.machine.spindle_rpm = payload[0] | (payload[1] << 8);
            }
            protocol_send_ack(seq);
            break;
            
        case CMD_SPINDLE_OFF:
            g_fixture.machine.spindle_on = false;
            g_fixture.machine.spindle_rpm = 0;
            protocol_send_ack(seq);
            break;
            
        case CMD_COOLANT_ON:
            if (len >= 1) {
                g_fixture.machine.coolant_flood = payload[0] & 0x01;
                g_fixture.machine.coolant_mist = payload[0] & 0x02;
            } else {
                g_fixture.machine.coolant_flood = true;
            }
            protocol_send_ack(seq);
            break;
            
        case CMD_COOLANT_OFF:
            g_fixture.machine.coolant_flood = false;
            g_fixture.machine.coolant_mist = false;
            protocol_send_ack(seq);
            break;
            
        default:
            g_fixture.comm.unknown_cmd++;
            protocol_send_nak(seq, 0xFF);  // Unknown command
            snprintf(log_msg, sizeof(log_msg), "Unknown cmd: 0x%02X", cmd);
            display_log(log_msg);
            break;
    }
}
