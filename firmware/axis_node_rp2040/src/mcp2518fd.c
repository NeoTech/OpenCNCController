/**
 * @file mcp2518fd.c
 * @brief MCP2518FD External CAN-FD Controller Driver Implementation
 */

#include "mcp2518fd.h"
#include "hardware/spi.h"
#include "hardware/gpio.h"
#include "pico/stdlib.h"
#include <string.h>

// =============================================================================
// DLC to byte count conversion
// =============================================================================

static const uint8_t dlc_to_len[] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 12, 16, 20, 24, 32, 48, 64};

static uint8_t len_to_dlc(uint8_t len)
{
    if (len <= 8) return len;
    if (len <= 12) return 9;
    if (len <= 16) return 10;
    if (len <= 20) return 11;
    if (len <= 24) return 12;
    if (len <= 32) return 13;
    if (len <= 48) return 14;
    return 15;
}

// =============================================================================
// SPI Low-level Functions
// =============================================================================

static void spi_select(mcp2518fd_t* ctx)
{
    gpio_put(ctx->pin_cs, 0);
}

static void spi_deselect(mcp2518fd_t* ctx)
{
    gpio_put(ctx->pin_cs, 1);
}

static void spi_transfer(mcp2518fd_t* ctx, const uint8_t* tx, uint8_t* rx, size_t len)
{
    spi_write_read_blocking(ctx->spi, tx, rx, len);
}

uint32_t mcp2518fd_read_reg(mcp2518fd_t* ctx, uint16_t addr)
{
    uint8_t tx[6] = {
        MCP2518FD_CMD_READ,
        (addr >> 8) & 0x0F,
        addr & 0xFF,
        0, 0, 0
    };
    uint8_t rx[6] = {0};
    
    spi_select(ctx);
    spi_transfer(ctx, tx, rx, 6);
    spi_deselect(ctx);
    
    return rx[3] | (rx[4] << 8) | (rx[5] << 16);
}

void mcp2518fd_write_reg(mcp2518fd_t* ctx, uint16_t addr, uint32_t value)
{
    uint8_t tx[7] = {
        MCP2518FD_CMD_WRITE,
        (addr >> 8) & 0x0F,
        addr & 0xFF,
        value & 0xFF,
        (value >> 8) & 0xFF,
        (value >> 16) & 0xFF,
        (value >> 24) & 0xFF
    };
    
    spi_select(ctx);
    spi_write_blocking(ctx->spi, tx, 7);
    spi_deselect(ctx);
}

void mcp2518fd_read_data(mcp2518fd_t* ctx, uint16_t addr, uint8_t* data, uint8_t len)
{
    uint8_t cmd[3] = {
        MCP2518FD_CMD_READ,
        (addr >> 8) & 0x0F,
        addr & 0xFF
    };
    
    spi_select(ctx);
    spi_write_blocking(ctx->spi, cmd, 3);
    spi_read_blocking(ctx->spi, 0, data, len);
    spi_deselect(ctx);
}

void mcp2518fd_write_data(mcp2518fd_t* ctx, uint16_t addr, const uint8_t* data, uint8_t len)
{
    uint8_t cmd[3] = {
        MCP2518FD_CMD_WRITE,
        (addr >> 8) & 0x0F,
        addr & 0xFF
    };
    
    spi_select(ctx);
    spi_write_blocking(ctx->spi, cmd, 3);
    spi_write_blocking(ctx->spi, data, len);
    spi_deselect(ctx);
}

// =============================================================================
// Initialization
// =============================================================================

void mcp2518fd_reset(mcp2518fd_t* ctx)
{
    uint8_t cmd[2] = {MCP2518FD_CMD_RESET, 0x00};
    
    spi_select(ctx);
    spi_write_blocking(ctx->spi, cmd, 2);
    spi_deselect(ctx);
    
    sleep_ms(10);  // Wait for reset
}

bool mcp2518fd_init(mcp2518fd_t* ctx)
{
    // Initialize SPI pins
    gpio_set_function(MCP2518FD_PIN_MISO, GPIO_FUNC_SPI);
    gpio_set_function(MCP2518FD_PIN_MOSI, GPIO_FUNC_SPI);
    gpio_set_function(MCP2518FD_PIN_SCK, GPIO_FUNC_SPI);
    
    // CS as GPIO output
    gpio_init(ctx->pin_cs);
    gpio_set_dir(ctx->pin_cs, GPIO_OUT);
    gpio_put(ctx->pin_cs, 1);  // Deselect
    
    // INT as input
    gpio_init(ctx->pin_int);
    gpio_set_dir(ctx->pin_int, GPIO_IN);
    gpio_pull_up(ctx->pin_int);  // Active low
    
    // Initialize SPI
    ctx->spi = MCP2518FD_SPI_INSTANCE;
    spi_init(ctx->spi, MCP2518FD_SPI_BAUD);
    spi_set_format(ctx->spi, 8, SPI_CPOL_0, SPI_CPHA_0, SPI_MSB_FIRST);
    
    // Reset device
    mcp2518fd_reset(ctx);
    
    // Enter configuration mode
    if (!mcp2518fd_set_mode(ctx, MCP2518FD_MODE_CONFIG)) {
        return false;
    }
    
    // Clear RAM
    uint8_t zeros[16] = {0};
    for (uint16_t addr = MCP2518FD_RAM_START; addr < MCP2518FD_RAM_START + MCP2518FD_RAM_SIZE; addr += 16) {
        mcp2518fd_write_data(ctx, addr, zeros, 16);
    }
    
    // Configure timing
    if (!mcp2518fd_configure_timing(ctx)) {
        return false;
    }
    
    // Configure FIFOs
    if (!mcp2518fd_configure_fifos(ctx)) {
        return false;
    }
    
    // Accept all standard frames for CANopen
    mcp2518fd_configure_filter(ctx, 0, 0, 0, false);
    
    ctx->initialized = true;
    return true;
}

bool mcp2518fd_set_mode(mcp2518fd_t* ctx, uint8_t mode)
{
    // Read current CiCON
    uint32_t con = mcp2518fd_read_reg(ctx, MCP2518FD_REG_CiCON);
    
    // Clear REQOP and set new mode
    con &= ~CiCON_REQOP_MASK;
    con |= ((uint32_t)mode << CiCON_REQOP_SHIFT);
    
    // Abort all TX
    con |= CiCON_ABAT;
    
    mcp2518fd_write_reg(ctx, MCP2518FD_REG_CiCON, con);
    
    // Wait for mode change
    for (int i = 0; i < 100; i++) {
        sleep_ms(1);
        con = mcp2518fd_read_reg(ctx, MCP2518FD_REG_CiCON);
        uint8_t opmod = (con & CiCON_OPMOD_MASK) >> CiCON_OPMOD_SHIFT;
        if (opmod == mode) {
            ctx->mode = mode;
            return true;
        }
    }
    
    return false;
}

bool mcp2518fd_configure_timing(mcp2518fd_t* ctx)
{
    // Configure for 500 kbps nominal, 2 Mbps data
    // Assuming 40 MHz oscillator
    
    // Nominal: 40 MHz / 4 = 10 MHz TQ
    // 500 kbps = 20 TQ per bit
    // SYNC=1, TSEG1=15, TSEG2=4, SJW=4
    uint32_t nbtcfg = 0;
    nbtcfg |= (4 - 1);           // BRP = 4 (prescaler - 1)
    nbtcfg |= ((15 - 1) << 8);   // TSEG1 = 15 (value - 1)
    nbtcfg |= ((4 - 1) << 16);   // TSEG2 = 4 (value - 1)
    nbtcfg |= ((4 - 1) << 24);   // SJW = 4 (value - 1)
    
    mcp2518fd_write_reg(ctx, MCP2518FD_REG_CiNBTCFG, nbtcfg);
    
    // Data: 40 MHz / 2 = 20 MHz TQ
    // 2 Mbps = 10 TQ per bit
    // SYNC=1, TSEG1=6, TSEG2=3, SJW=3
    uint32_t dbtcfg = 0;
    dbtcfg |= (2 - 1);           // BRP = 2
    dbtcfg |= ((6 - 1) << 8);    // TSEG1 = 6
    dbtcfg |= ((3 - 1) << 16);   // TSEG2 = 3
    dbtcfg |= ((3 - 1) << 24);   // SJW = 3
    
    mcp2518fd_write_reg(ctx, MCP2518FD_REG_CiDBTCFG, dbtcfg);
    
    // Enable TDC for data phase
    uint32_t tdc = 0;
    tdc |= (1 << 8);             // TDCMOD = Auto
    tdc |= (31 << 0);            // TDCO = 31
    
    mcp2518fd_write_reg(ctx, MCP2518FD_REG_CiTDC, tdc);
    
    return true;
}

bool mcp2518fd_configure_fifos(mcp2518fd_t* ctx)
{
    // Configure TX Queue
    uint32_t txqcon = 0;
    txqcon |= CiFIFOCON_TXEN;                    // TX enable
    txqcon |= (7 << CiFIFOCON_FSIZE_SHIFT);      // 8 messages
    txqcon |= (MCP2518FD_PLSIZE_64 << CiFIFOCON_PLSIZE_SHIFT);  // 64 bytes
    
    mcp2518fd_write_reg(ctx, MCP2518FD_REG_CiFIFOCON(MCP2518FD_FIFO_TX), txqcon);
    
    // Configure RX FIFO
    uint32_t rxcon = 0;
    rxcon |= (15 << CiFIFOCON_FSIZE_SHIFT);      // 16 messages
    rxcon |= (MCP2518FD_PLSIZE_64 << CiFIFOCON_PLSIZE_SHIFT);   // 64 bytes
    rxcon |= CiFIFOCON_RXTSEN;                   // Timestamp
    
    mcp2518fd_write_reg(ctx, MCP2518FD_REG_CiFIFOCON(MCP2518FD_FIFO_RX), rxcon);
    
    // Set FIFO base address (start of RAM)
    mcp2518fd_write_reg(ctx, MCP2518FD_REG_CiFIFOBA, MCP2518FD_RAM_START);
    
    return true;
}

bool mcp2518fd_configure_filter(mcp2518fd_t* ctx, uint8_t filter_num,
                                uint32_t id, uint32_t mask, bool extended)
{
    if (filter_num > 31) return false;
    
    // Filter object
    uint32_t fltobj = 0;
    if (extended) {
        fltobj |= id & 0x1FFFFFFF;
        fltobj |= (1 << 30);  // EXIDE = 1
    } else {
        fltobj |= id & 0x7FF;
    }
    
    mcp2518fd_write_reg(ctx, MCP2518FD_REG_CiFLTOBJ(filter_num), fltobj);
    
    // Mask
    uint32_t mskobj = 0;
    if (extended) {
        mskobj |= mask & 0x1FFFFFFF;
        mskobj |= (1 << 30);  // MIDE = 1 (must match IDE)
    } else {
        mskobj |= mask & 0x7FF;
    }
    
    mcp2518fd_write_reg(ctx, MCP2518FD_REG_CiMASK(filter_num), mskobj);
    
    // Enable filter, route to RX FIFO
    uint32_t fltcon = mcp2518fd_read_reg(ctx, MCP2518FD_REG_CiFLTCON(filter_num / 4));
    uint8_t shift = (filter_num % 4) * 8;
    fltcon &= ~(0xFF << shift);
    fltcon |= (0x80 | MCP2518FD_FIFO_RX) << shift;  // Enable + FIFO
    mcp2518fd_write_reg(ctx, MCP2518FD_REG_CiFLTCON(filter_num / 4), fltcon);
    
    return true;
}

// =============================================================================
// TX/RX
// =============================================================================

bool mcp2518fd_tx_ready(mcp2518fd_t* ctx)
{
    uint32_t sta = mcp2518fd_read_reg(ctx, MCP2518FD_REG_CiFIFOSTA(MCP2518FD_FIFO_TX));
    return !(sta & (1 << 0));  // Not full
}

bool mcp2518fd_transmit(mcp2518fd_t* ctx, const can_frame_t* frame)
{
    if (!mcp2518fd_tx_ready(ctx)) {
        return false;
    }
    
    // Get TX FIFO address
    uint32_t addr = mcp2518fd_read_reg(ctx, MCP2518FD_REG_CiFIFOUA(MCP2518FD_FIFO_TX));
    addr += MCP2518FD_RAM_START;
    
    // Build message object
    mcp2518fd_msg_obj_t obj = {0};
    
    if (frame->is_extended) {
        obj.id = frame->id & 0x1FFFFFFF;
    } else {
        obj.id = (frame->id & 0x7FF) << MSG_ID_SID_SHIFT;
    }
    
    obj.flags = len_to_dlc(frame->dlc);
    if (frame->is_extended) obj.flags |= MSG_FLAGS_IDE;
    if (frame->is_rtr) obj.flags |= MSG_FLAGS_RTR;
    if (frame->is_fd) {
        obj.flags |= MSG_FLAGS_FDF;
        obj.flags |= MSG_FLAGS_BRS;
    }
    
    memcpy(obj.data, frame->data, frame->dlc);
    
    // Write to RAM
    mcp2518fd_write_data(ctx, addr, (uint8_t*)&obj, 8 + frame->dlc);
    
    // Increment FIFO pointer (set UINC bit)
    uint32_t con = mcp2518fd_read_reg(ctx, MCP2518FD_REG_CiFIFOCON(MCP2518FD_FIFO_TX));
    con |= (1 << 8);  // UINC
    con |= (1 << 9);  // TXREQ
    mcp2518fd_write_reg(ctx, MCP2518FD_REG_CiFIFOCON(MCP2518FD_FIFO_TX), con);
    
    ctx->tx_count++;
    return true;
}

bool mcp2518fd_rx_available(mcp2518fd_t* ctx)
{
    uint32_t sta = mcp2518fd_read_reg(ctx, MCP2518FD_REG_CiFIFOSTA(MCP2518FD_FIFO_RX));
    return (sta & (1 << 0));  // Not empty
}

bool mcp2518fd_receive(mcp2518fd_t* ctx, can_frame_t* frame)
{
    if (!mcp2518fd_rx_available(ctx)) {
        return false;
    }
    
    // Get RX FIFO address
    uint32_t addr = mcp2518fd_read_reg(ctx, MCP2518FD_REG_CiFIFOUA(MCP2518FD_FIFO_RX));
    addr += MCP2518FD_RAM_START;
    
    // Read message object
    mcp2518fd_msg_obj_t obj;
    mcp2518fd_read_data(ctx, addr, (uint8_t*)&obj, sizeof(obj));
    
    // Parse message
    if (obj.flags & MSG_FLAGS_IDE) {
        frame->id = obj.id & 0x1FFFFFFF;
        frame->is_extended = true;
    } else {
        frame->id = (obj.id >> MSG_ID_SID_SHIFT) & MSG_ID_SID_MASK;
        frame->is_extended = false;
    }
    
    frame->is_rtr = (obj.flags & MSG_FLAGS_RTR) != 0;
    frame->is_fd = (obj.flags & MSG_FLAGS_FDF) != 0;
    frame->dlc = dlc_to_len[obj.flags & MSG_FLAGS_DLC_MASK];
    
    memcpy(frame->data, obj.data, frame->dlc);
    
    // Increment FIFO pointer
    uint32_t con = mcp2518fd_read_reg(ctx, MCP2518FD_REG_CiFIFOCON(MCP2518FD_FIFO_RX));
    con |= (1 << 8);  // UINC
    mcp2518fd_write_reg(ctx, MCP2518FD_REG_CiFIFOCON(MCP2518FD_FIFO_RX), con);
    
    ctx->rx_count++;
    return true;
}

void mcp2518fd_set_rx_callback(mcp2518fd_t* ctx, void (*callback)(can_frame_t*))
{
    ctx->rx_callback = callback;
}

// =============================================================================
// Interrupt Handling
// =============================================================================

void mcp2518fd_irq_handler(mcp2518fd_t* ctx)
{
    uint32_t intstat = mcp2518fd_read_reg(ctx, MCP2518FD_REG_CiINT);
    
    // RX interrupt
    if (intstat & (1 << 1)) {
        if (ctx->rx_callback) {
            can_frame_t frame;
            while (mcp2518fd_receive(ctx, &frame)) {
                ctx->rx_callback(&frame);
            }
        }
    }
    
    // Error interrupt
    if (intstat & (1 << 2)) {
        ctx->error_count++;
    }
    
    // Clear interrupts
    mcp2518fd_write_reg(ctx, MCP2518FD_REG_CiINT, intstat);
}

void mcp2518fd_enable_interrupts(mcp2518fd_t* ctx, bool enable)
{
    uint32_t inten = 0;
    
    if (enable) {
        inten |= (1 << 1);   // RXIE - RX interrupt
        inten |= (1 << 2);   // ERRIE - Error interrupt
    }
    
    mcp2518fd_write_reg(ctx, MCP2518FD_REG_CiINTEN, inten);
}

// =============================================================================
// Status
// =============================================================================

void mcp2518fd_get_error_counts(mcp2518fd_t* ctx, uint8_t* tx_errors, uint8_t* rx_errors)
{
    uint32_t trec = mcp2518fd_read_reg(ctx, 0x034);  // CiTREC
    *tx_errors = (trec >> 8) & 0xFF;
    *rx_errors = trec & 0xFF;
}

bool mcp2518fd_is_bus_off(mcp2518fd_t* ctx)
{
    uint32_t trec = mcp2518fd_read_reg(ctx, 0x034);
    return (trec & (1 << 21)) != 0;  // TXBO bit
}
