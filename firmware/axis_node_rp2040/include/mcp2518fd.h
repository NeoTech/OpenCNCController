/**
 * @file mcp2518fd.h
 * @brief MCP2518FD External CAN-FD Controller Driver
 * 
 * SPI driver for Microchip MCP2518FD CAN-FD controller used with RP2040
 * which lacks native CAN peripheral.
 */

#ifndef MCP2518FD_H
#define MCP2518FD_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include "canopen_types.h"

// =============================================================================
// Configuration
// =============================================================================

// SPI Configuration
#define MCP2518FD_SPI_INSTANCE   spi0
#define MCP2518FD_SPI_BAUD       10000000    // 10 MHz SPI clock
#define MCP2518FD_PIN_MISO       4
#define MCP2518FD_PIN_MOSI       3
#define MCP2518FD_PIN_SCK        2
#define MCP2518FD_PIN_CS         5
#define MCP2518FD_PIN_INT        6

// Oscillator (typically 40 MHz crystal)
#define MCP2518FD_OSC_FREQ       40000000

// =============================================================================
// MCP2518FD Registers
// =============================================================================

// SPI Commands
#define MCP2518FD_CMD_RESET      0x00
#define MCP2518FD_CMD_READ       0x03
#define MCP2518FD_CMD_WRITE      0x02
#define MCP2518FD_CMD_READ_CRC   0x0B
#define MCP2518FD_CMD_WRITE_CRC  0x0A

// CAN FD Controller registers (SFR)
#define MCP2518FD_REG_CiCON      0x000   // CAN Control
#define MCP2518FD_REG_CiNBTCFG   0x004   // Nominal Bit Time Config
#define MCP2518FD_REG_CiDBTCFG   0x008   // Data Bit Time Config
#define MCP2518FD_REG_CiTDC      0x00C   // Transmitter Delay Compensation
#define MCP2518FD_REG_CiTBC      0x010   // Time Base Counter
#define MCP2518FD_REG_CiTSCON    0x014   // Time Stamp Control
#define MCP2518FD_REG_CiVEC      0x018   // Interrupt Vector
#define MCP2518FD_REG_CiINT      0x01C   // Interrupt Status
#define MCP2518FD_REG_CiINTEN    0x020   // Interrupt Enable
#define MCP2518FD_REG_CiFIFOBA   0x04C   // FIFO Base Address

// FIFO Control registers
#define MCP2518FD_REG_CiFIFOCON(n)   (0x05C + (n) * 12)   // FIFO Control
#define MCP2518FD_REG_CiFIFOSTA(n)   (0x060 + (n) * 12)   // FIFO Status
#define MCP2518FD_REG_CiFIFOUA(n)    (0x064 + (n) * 12)   // FIFO User Address

// Filter registers
#define MCP2518FD_REG_CiFLTCON(n)    (0x1D0 + (n) * 4)    // Filter Control
#define MCP2518FD_REG_CiFLTOBJ(n)    (0x1F0 + (n) * 8)    // Filter Object
#define MCP2518FD_REG_CiMASK(n)      (0x1F4 + (n) * 8)    // Filter Mask

// RAM
#define MCP2518FD_RAM_START      0x400
#define MCP2518FD_RAM_SIZE       2048

// =============================================================================
// CiCON Register Bits
// =============================================================================

#define CiCON_TXQEN              (1 << 4)     // TX Queue Enable
#define CiCON_STEF               (1 << 5)     // Store in TEF
#define CiCON_SERRLOM            (1 << 6)     // System Error Listen Only Mode
#define CiCON_REQOP_SHIFT        24           // Request Operation Mode
#define CiCON_REQOP_MASK         (0x7 << 24)
#define CiCON_OPMOD_SHIFT        21           // Operation Mode
#define CiCON_OPMOD_MASK         (0x7 << 21)
#define CiCON_TXBWS_SHIFT        28           // TX Bandwidth Sharing
#define CiCON_ABAT               (1 << 27)    // Abort All TX

// Operation Modes
#define MCP2518FD_MODE_NORMAL        0
#define MCP2518FD_MODE_SLEEP         1
#define MCP2518FD_MODE_INTERNAL_LB   2
#define MCP2518FD_MODE_LISTEN        3
#define MCP2518FD_MODE_CONFIG        4
#define MCP2518FD_MODE_EXTERNAL_LB   5
#define MCP2518FD_MODE_CAN20         6
#define MCP2518FD_MODE_RESTRICTED    7

// =============================================================================
// FIFO Configuration
// =============================================================================

#define MCP2518FD_FIFO_TX        1    // TX FIFO
#define MCP2518FD_FIFO_RX        2    // RX FIFO

// FIFO Control bits
#define CiFIFOCON_TXEN           (1 << 7)     // TX FIFO Enable
#define CiFIFOCON_TXPRI_SHIFT    0            // TX Priority
#define CiFIFOCON_TXPRI_MASK     0x1F
#define CiFIFOCON_RXTSEN         (1 << 5)     // RX Timestamp Enable
#define CiFIFOCON_FSIZE_SHIFT    24           // FIFO Size
#define CiFIFOCON_PLSIZE_SHIFT   29           // Payload Size

// Payload sizes
#define MCP2518FD_PLSIZE_8       0
#define MCP2518FD_PLSIZE_12      1
#define MCP2518FD_PLSIZE_16      2
#define MCP2518FD_PLSIZE_20      3
#define MCP2518FD_PLSIZE_24      4
#define MCP2518FD_PLSIZE_32      5
#define MCP2518FD_PLSIZE_48      6
#define MCP2518FD_PLSIZE_64      7

// =============================================================================
// Bit Timing Configuration
// =============================================================================

typedef struct {
    // Nominal bit rate (arbitration phase)
    uint32_t nominal_bitrate;   // Target bit rate (e.g., 500000)
    uint8_t  nbrp;              // Baud Rate Prescaler (1-256)
    uint8_t  ntseg1;            // Time Segment 1 (2-256)
    uint8_t  ntseg2;            // Time Segment 2 (1-128)
    uint8_t  nsjw;              // Sync Jump Width (1-128)
    
    // Data bit rate (data phase for CAN-FD)
    uint32_t data_bitrate;      // Target bit rate (e.g., 2000000)
    uint8_t  dbrp;              // Baud Rate Prescaler (1-256)
    uint8_t  dtseg1;            // Time Segment 1 (1-32)
    uint8_t  dtseg2;            // Time Segment 2 (1-16)
    uint8_t  dsjw;              // Sync Jump Width (1-16)
    
    // Transmitter Delay Compensation
    bool     tdc_enable;        // Enable TDC
    uint8_t  tdc_offset;        // TDC Offset (0-63)
} mcp2518fd_timing_t;

// =============================================================================
// TX/RX Message Objects
// =============================================================================

// Message object header in RAM
typedef struct __attribute__((packed)) {
    uint32_t id;                // ID field
    uint32_t flags;             // DLC, IDE, RTR, BRS, FDF, ESI, SEQ, etc.
    uint8_t  data[64];          // Data payload
} mcp2518fd_msg_obj_t;

// ID field bits
#define MSG_ID_SID_SHIFT         0
#define MSG_ID_SID_MASK          0x7FF
#define MSG_ID_EID_SHIFT         11
#define MSG_ID_EID_MASK          0x3FFFF

// Flags field bits
#define MSG_FLAGS_DLC_SHIFT      0
#define MSG_FLAGS_DLC_MASK       0x0F
#define MSG_FLAGS_IDE            (1 << 4)     // Extended ID
#define MSG_FLAGS_RTR            (1 << 5)     // Remote Frame
#define MSG_FLAGS_BRS            (1 << 6)     // Bit Rate Switch
#define MSG_FLAGS_FDF            (1 << 7)     // FD Frame
#define MSG_FLAGS_ESI            (1 << 8)     // Error State Indicator
#define MSG_FLAGS_SEQ_SHIFT      9            // Sequence Number
#define MSG_FLAGS_SEQ_MASK       0x7F

// =============================================================================
// Driver Context
// =============================================================================

typedef struct {
    // SPI instance
    void*    spi;
    uint8_t  pin_cs;
    uint8_t  pin_int;
    
    // Configuration
    mcp2518fd_timing_t timing;
    
    // State
    bool     initialized;
    uint8_t  mode;
    
    // Statistics
    uint32_t tx_count;
    uint32_t rx_count;
    uint32_t error_count;
    
    // Interrupt callback
    void (*rx_callback)(can_frame_t* frame);
} mcp2518fd_t;

// =============================================================================
// Initialization
// =============================================================================

/**
 * @brief Initialize MCP2518FD driver
 * @param ctx Driver context
 * @return true on success
 */
bool mcp2518fd_init(mcp2518fd_t* ctx);

/**
 * @brief Reset MCP2518FD
 * @param ctx Driver context
 */
void mcp2518fd_reset(mcp2518fd_t* ctx);

/**
 * @brief Configure bit timing for 500kbps/2Mbps CAN-FD
 * @param ctx Driver context
 * @return true on success
 */
bool mcp2518fd_configure_timing(mcp2518fd_t* ctx);

/**
 * @brief Set operation mode
 * @param ctx Driver context
 * @param mode Operation mode (MCP2518FD_MODE_*)
 * @return true on success
 */
bool mcp2518fd_set_mode(mcp2518fd_t* ctx, uint8_t mode);

/**
 * @brief Configure TX and RX FIFOs
 * @param ctx Driver context
 * @return true on success
 */
bool mcp2518fd_configure_fifos(mcp2518fd_t* ctx);

/**
 * @brief Configure receive filters
 * @param ctx Driver context
 * @param filter_num Filter number (0-31)
 * @param id ID to match
 * @param mask Mask (0 = match all)
 * @param extended true for extended ID
 * @return true on success
 */
bool mcp2518fd_configure_filter(mcp2518fd_t* ctx, uint8_t filter_num,
                                uint32_t id, uint32_t mask, bool extended);

// =============================================================================
// TX/RX
// =============================================================================

/**
 * @brief Transmit a CAN frame
 * @param ctx Driver context
 * @param frame Frame to transmit
 * @return true on success
 */
bool mcp2518fd_transmit(mcp2518fd_t* ctx, const can_frame_t* frame);

/**
 * @brief Check if TX FIFO has space
 * @param ctx Driver context
 * @return true if can transmit
 */
bool mcp2518fd_tx_ready(mcp2518fd_t* ctx);

/**
 * @brief Receive a CAN frame
 * @param ctx Driver context
 * @param frame Received frame
 * @return true if frame received
 */
bool mcp2518fd_receive(mcp2518fd_t* ctx, can_frame_t* frame);

/**
 * @brief Check if RX FIFO has data
 * @param ctx Driver context
 * @return true if frame available
 */
bool mcp2518fd_rx_available(mcp2518fd_t* ctx);

/**
 * @brief Set RX callback
 * @param ctx Driver context
 * @param callback Callback function
 */
void mcp2518fd_set_rx_callback(mcp2518fd_t* ctx, void (*callback)(can_frame_t*));

// =============================================================================
// Interrupt Handling
// =============================================================================

/**
 * @brief Process interrupt (call from GPIO ISR)
 * @param ctx Driver context
 */
void mcp2518fd_irq_handler(mcp2518fd_t* ctx);

/**
 * @brief Enable/disable interrupts
 * @param ctx Driver context
 * @param enable true to enable
 */
void mcp2518fd_enable_interrupts(mcp2518fd_t* ctx, bool enable);

// =============================================================================
// Status
// =============================================================================

/**
 * @brief Get error count
 * @param ctx Driver context
 * @param tx_errors Pointer to TX error count
 * @param rx_errors Pointer to RX error count
 */
void mcp2518fd_get_error_counts(mcp2518fd_t* ctx, uint8_t* tx_errors, uint8_t* rx_errors);

/**
 * @brief Check if bus is off
 * @param ctx Driver context
 * @return true if bus-off
 */
bool mcp2518fd_is_bus_off(mcp2518fd_t* ctx);

// =============================================================================
// Low-level SPI
// =============================================================================

/**
 * @brief Read register
 * @param ctx Driver context
 * @param addr Register address
 * @return Register value
 */
uint32_t mcp2518fd_read_reg(mcp2518fd_t* ctx, uint16_t addr);

/**
 * @brief Write register
 * @param ctx Driver context
 * @param addr Register address
 * @param value Value to write
 */
void mcp2518fd_write_reg(mcp2518fd_t* ctx, uint16_t addr, uint32_t value);

/**
 * @brief Read data block from RAM
 * @param ctx Driver context
 * @param addr RAM address
 * @param data Buffer
 * @param len Length in bytes
 */
void mcp2518fd_read_data(mcp2518fd_t* ctx, uint16_t addr, uint8_t* data, uint8_t len);

/**
 * @brief Write data block to RAM
 * @param ctx Driver context
 * @param addr RAM address
 * @param data Data to write
 * @param len Length in bytes
 */
void mcp2518fd_write_data(mcp2518fd_t* ctx, uint16_t addr, const uint8_t* data, uint8_t len);

#ifdef __cplusplus
}
#endif

#endif // MCP2518FD_H
