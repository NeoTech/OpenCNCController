/*
 * OpenCNC HAL - FPGA Top Module
 * 
 * Top-level module for CNC motion controller.
 * Target: Lattice iCE40UP5K / ECP5
 * 
 * Copyright (c) 2024 OpenCNC Project
 * MIT License
 */

module cnc_top #(
    parameter NUM_AXES = 3,
    parameter CLK_FREQ = 48_000_000  // 48 MHz
)(
    input wire clk,
    input wire rst_n,
    
    // Step/Direction outputs
    output wire [NUM_AXES-1:0] step,
    output wire [NUM_AXES-1:0] dir,
    
    // Limit switch inputs
    input wire [NUM_AXES-1:0] limit_min,
    input wire [NUM_AXES-1:0] limit_max,
    
    // Probe and E-Stop
    input wire probe,
    input wire estop_n,
    
    // Spindle
    output wire spindle_pwm,
    output wire spindle_en,
    output wire spindle_dir,
    
    // Coolant
    output wire coolant_mist,
    output wire coolant_flood,
    
    // SPI interface (to MCU)
    input wire spi_clk,
    input wire spi_mosi,
    output wire spi_miso,
    input wire spi_cs_n,
    
    // Status LEDs
    output wire [2:0] led
);

// ============================================================================
// Clock and Reset
// ============================================================================

wire pll_locked;
wire sys_clk;
wire sys_rst_n;

// PLL for clock generation (platform specific)
// For iCE40: use SB_PLL40_CORE
// For ECP5: use EHXPLLL

assign sys_clk = clk;  // Use input directly for now
assign sys_rst_n = rst_n & ~estop_n;

// ============================================================================
// Motion Segment FIFO
// ============================================================================

localparam FIFO_DEPTH = 32;
localparam STEP_WIDTH = 32;
localparam VELOCITY_WIDTH = 24;

wire segment_valid;
wire segment_ready;
wire [STEP_WIDTH-1:0] target_pos [NUM_AXES-1:0];
wire [VELOCITY_WIDTH-1:0] entry_velocity;
wire [VELOCITY_WIDTH-1:0] cruise_velocity;
wire [VELOCITY_WIDTH-1:0] exit_velocity;
wire [VELOCITY_WIDTH-1:0] acceleration;

// FIFO interface wires
wire fifo_write;
wire fifo_read;
wire fifo_empty;
wire fifo_full;
wire [7:0] fifo_count;

// ============================================================================
// Step Generator Instance
// ============================================================================

wire step_gen_busy;
wire [NUM_AXES-1:0] limit_hit;
wire [STEP_WIDTH-1:0] current_pos [NUM_AXES-1:0];

step_generator #(
    .NUM_AXES(NUM_AXES),
    .STEP_WIDTH(STEP_WIDTH),
    .VELOCITY_WIDTH(VELOCITY_WIDTH)
) step_gen (
    .clk(sys_clk),
    .rst_n(sys_rst_n),
    
    .enable(~fifo_empty),
    .axis_enable({NUM_AXES{1'b1}}),
    
    .segment_valid(segment_valid),
    .segment_ready(segment_ready),
    .target_pos(target_pos),
    .entry_velocity(entry_velocity),
    .cruise_velocity(cruise_velocity),
    .exit_velocity(exit_velocity),
    .acceleration(acceleration),
    
    .step_out(step),
    .dir_out(dir),
    
    .current_pos(current_pos),
    
    .busy(step_gen_busy),
    .limit_hit(limit_hit)
);

// ============================================================================
// SPI Slave Interface
// ============================================================================

wire [7:0] spi_rx_data;
wire spi_rx_valid;
reg [7:0] spi_tx_data;

spi_slave spi_if (
    .clk(sys_clk),
    .rst_n(sys_rst_n),
    
    .spi_clk(spi_clk),
    .spi_mosi(spi_mosi),
    .spi_miso(spi_miso),
    .spi_cs_n(spi_cs_n),
    
    .rx_data(spi_rx_data),
    .rx_valid(spi_rx_valid),
    .tx_data(spi_tx_data)
);

// ============================================================================
// Command Processing
// ============================================================================

localparam CMD_NOP         = 8'h00;
localparam CMD_GET_STATUS  = 8'h01;
localparam CMD_GET_POS     = 8'h02;
localparam CMD_PUSH_SEG    = 8'h10;
localparam CMD_START       = 8'h20;
localparam CMD_PAUSE       = 8'h21;
localparam CMD_STOP        = 8'h22;
localparam CMD_SET_SPINDLE = 8'h30;
localparam CMD_SET_COOLANT = 8'h31;

reg [2:0] cmd_state;
reg [7:0] cmd_byte;
reg motion_enable;

always @(posedge sys_clk or negedge sys_rst_n) begin
    if (!sys_rst_n) begin
        cmd_state <= 0;
        motion_enable <= 0;
    end else if (spi_rx_valid) begin
        case (spi_rx_data)
            CMD_START: motion_enable <= 1;
            CMD_PAUSE: motion_enable <= 0;
            CMD_STOP: begin
                motion_enable <= 0;
                // Clear FIFO
            end
        endcase
    end
end

// ============================================================================
// Spindle PWM Generator
// ============================================================================

reg [15:0] spindle_duty;
reg spindle_enabled;
reg spindle_direction;

pwm_generator #(
    .WIDTH(16),
    .CLK_FREQ(CLK_FREQ),
    .PWM_FREQ(5000)
) spindle_pwm_gen (
    .clk(sys_clk),
    .rst_n(sys_rst_n),
    .enable(spindle_enabled),
    .duty(spindle_duty),
    .pwm_out(spindle_pwm)
);

assign spindle_en = spindle_enabled;
assign spindle_dir = spindle_direction;

// ============================================================================
// Coolant Control
// ============================================================================

reg coolant_mist_en;
reg coolant_flood_en;

assign coolant_mist = coolant_mist_en;
assign coolant_flood = coolant_flood_en;

// ============================================================================
// Status LEDs
// ============================================================================

assign led[0] = step_gen_busy;        // Motion active
assign led[1] = |limit_hit;           // Limit triggered
assign led[2] = spindle_enabled;      // Spindle on

endmodule

// ============================================================================
// SPI Slave Module
// ============================================================================

module spi_slave (
    input wire clk,
    input wire rst_n,
    
    input wire spi_clk,
    input wire spi_mosi,
    output reg spi_miso,
    input wire spi_cs_n,
    
    output reg [7:0] rx_data,
    output reg rx_valid,
    input wire [7:0] tx_data
);

reg [2:0] bit_cnt;
reg [7:0] shift_in;
reg [7:0] shift_out;

reg spi_clk_d1, spi_clk_d2;
wire spi_clk_rise, spi_clk_fall;

always @(posedge clk) begin
    spi_clk_d1 <= spi_clk;
    spi_clk_d2 <= spi_clk_d1;
end

assign spi_clk_rise = spi_clk_d1 & ~spi_clk_d2;
assign spi_clk_fall = ~spi_clk_d1 & spi_clk_d2;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        bit_cnt <= 0;
        shift_in <= 0;
        shift_out <= 0;
        rx_data <= 0;
        rx_valid <= 0;
        spi_miso <= 0;
    end else begin
        rx_valid <= 0;
        
        if (spi_cs_n) begin
            bit_cnt <= 0;
            shift_out <= tx_data;
        end else begin
            if (spi_clk_rise) begin
                shift_in <= {shift_in[6:0], spi_mosi};
                bit_cnt <= bit_cnt + 1;
                
                if (bit_cnt == 7) begin
                    rx_data <= {shift_in[6:0], spi_mosi};
                    rx_valid <= 1;
                    shift_out <= tx_data;
                end
            end
            
            if (spi_clk_fall) begin
                spi_miso <= shift_out[7];
                shift_out <= {shift_out[6:0], 1'b0};
            end
        end
    end
end

endmodule

// ============================================================================
// PWM Generator Module
// ============================================================================

module pwm_generator #(
    parameter WIDTH = 16,
    parameter CLK_FREQ = 48_000_000,
    parameter PWM_FREQ = 5000
)(
    input wire clk,
    input wire rst_n,
    input wire enable,
    input wire [WIDTH-1:0] duty,
    output reg pwm_out
);

localparam PERIOD = CLK_FREQ / PWM_FREQ;
localparam COUNTER_WIDTH = $clog2(PERIOD);

reg [COUNTER_WIDTH-1:0] counter;
wire [COUNTER_WIDTH-1:0] compare;

assign compare = (duty * PERIOD) >> WIDTH;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        counter <= 0;
        pwm_out <= 0;
    end else if (!enable) begin
        counter <= 0;
        pwm_out <= 0;
    end else begin
        if (counter >= PERIOD - 1) begin
            counter <= 0;
        end else begin
            counter <= counter + 1;
        end
        
        pwm_out <= (counter < compare);
    end
end

endmodule
