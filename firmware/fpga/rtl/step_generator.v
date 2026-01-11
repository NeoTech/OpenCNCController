/*
 * OpenCNC HAL - FPGA Step Generator Module
 * 
 * Verilog HDL for high-performance step pulse generation.
 * Target: Lattice iCE40 / ECP5 or similar.
 * 
 * Copyright (c) 2024 OpenCNC Project
 * MIT License
 */

module step_generator #(
    parameter NUM_AXES = 3,
    parameter STEP_WIDTH = 32,
    parameter VELOCITY_WIDTH = 24
)(
    input wire clk,                          // System clock (50-100 MHz)
    input wire rst_n,                        // Active low reset
    
    // Control interface
    input wire enable,                       // Global enable
    input wire [NUM_AXES-1:0] axis_enable,   // Per-axis enable
    
    // Motion segment input (FIFO interface)
    input wire segment_valid,
    output wire segment_ready,
    input wire [STEP_WIDTH-1:0] target_pos [NUM_AXES-1:0],
    input wire [VELOCITY_WIDTH-1:0] entry_velocity,
    input wire [VELOCITY_WIDTH-1:0] cruise_velocity,
    input wire [VELOCITY_WIDTH-1:0] exit_velocity,
    input wire [VELOCITY_WIDTH-1:0] acceleration,
    
    // Step/Direction outputs
    output reg [NUM_AXES-1:0] step_out,
    output reg [NUM_AXES-1:0] dir_out,
    
    // Position feedback
    output reg [STEP_WIDTH-1:0] current_pos [NUM_AXES-1:0],
    
    // Status
    output wire busy,
    output wire [NUM_AXES-1:0] limit_hit
);

// ============================================================================
// Internal signals
// ============================================================================

// Bresenham DDA accumulators
reg [STEP_WIDTH:0] delta [NUM_AXES-1:0];
reg [STEP_WIDTH:0] error [NUM_AXES-1:0];
reg [STEP_WIDTH-1:0] steps_remaining [NUM_AXES-1:0];

// Velocity DDA
reg [VELOCITY_WIDTH+8:0] velocity_accum;
reg [VELOCITY_WIDTH-1:0] current_velocity;

// State machine
localparam STATE_IDLE = 3'd0;
localparam STATE_LOAD = 3'd1;
localparam STATE_ACCEL = 3'd2;
localparam STATE_CRUISE = 3'd3;
localparam STATE_DECEL = 3'd4;
localparam STATE_DONE = 3'd5;

reg [2:0] state;
reg [2:0] next_state;

// Step pulse generation
reg [7:0] step_pulse_counter [NUM_AXES-1:0];
parameter STEP_PULSE_WIDTH = 8'd10;  // Clock cycles

// Segment storage
reg [STEP_WIDTH-1:0] seg_target [NUM_AXES-1:0];
reg [VELOCITY_WIDTH-1:0] seg_entry_vel;
reg [VELOCITY_WIDTH-1:0] seg_cruise_vel;
reg [VELOCITY_WIDTH-1:0] seg_exit_vel;
reg [VELOCITY_WIDTH-1:0] seg_accel;

// ============================================================================
// Segment ready signal
// ============================================================================

assign segment_ready = (state == STATE_IDLE);
assign busy = (state != STATE_IDLE);

// ============================================================================
// State machine
// ============================================================================

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        state <= STATE_IDLE;
    end else begin
        state <= next_state;
    end
end

always @(*) begin
    next_state = state;
    
    case (state)
        STATE_IDLE: begin
            if (segment_valid && enable) begin
                next_state = STATE_LOAD;
            end
        end
        
        STATE_LOAD: begin
            next_state = STATE_ACCEL;
        end
        
        STATE_ACCEL: begin
            if (current_velocity >= seg_cruise_vel) begin
                next_state = STATE_CRUISE;
            end
        end
        
        STATE_CRUISE: begin
            // Check if we need to start decelerating
            // Simplified: check remaining steps
            if (all_steps_done()) begin
                next_state = STATE_DONE;
            end
        end
        
        STATE_DECEL: begin
            if (current_velocity <= seg_exit_vel || all_steps_done()) begin
                next_state = STATE_DONE;
            end
        end
        
        STATE_DONE: begin
            next_state = STATE_IDLE;
        end
        
        default: next_state = STATE_IDLE;
    endcase
end

// ============================================================================
// Check if all steps complete
// ============================================================================

function all_steps_done;
    integer i;
    begin
        all_steps_done = 1'b1;
        for (i = 0; i < NUM_AXES; i = i + 1) begin
            if (steps_remaining[i] != 0) begin
                all_steps_done = 1'b0;
            end
        end
    end
endfunction

// ============================================================================
// Load segment data
// ============================================================================

integer axis;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        for (axis = 0; axis < NUM_AXES; axis = axis + 1) begin
            seg_target[axis] <= 0;
            delta[axis] <= 0;
            error[axis] <= 0;
            steps_remaining[axis] <= 0;
            dir_out[axis] <= 0;
        end
        seg_entry_vel <= 0;
        seg_cruise_vel <= 0;
        seg_exit_vel <= 0;
        seg_accel <= 0;
        current_velocity <= 0;
    end else if (state == STATE_LOAD) begin
        seg_entry_vel <= entry_velocity;
        seg_cruise_vel <= cruise_velocity;
        seg_exit_vel <= exit_velocity;
        seg_accel <= acceleration;
        current_velocity <= entry_velocity;
        
        for (axis = 0; axis < NUM_AXES; axis = axis + 1) begin
            seg_target[axis] <= target_pos[axis];
            
            // Calculate delta and direction
            if (target_pos[axis] >= current_pos[axis]) begin
                delta[axis] <= target_pos[axis] - current_pos[axis];
                dir_out[axis] <= 1'b1;  // Positive direction
            end else begin
                delta[axis] <= current_pos[axis] - target_pos[axis];
                dir_out[axis] <= 1'b0;  // Negative direction
            end
            
            steps_remaining[axis] <= (target_pos[axis] >= current_pos[axis]) ?
                                     (target_pos[axis] - current_pos[axis]) :
                                     (current_pos[axis] - target_pos[axis]);
            error[axis] <= 0;
        end
    end
end

// ============================================================================
// Velocity control
// ============================================================================

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        velocity_accum <= 0;
    end else if (state == STATE_ACCEL) begin
        // Accelerate
        if (current_velocity < seg_cruise_vel) begin
            current_velocity <= current_velocity + (seg_accel >> 8);
        end
    end else if (state == STATE_DECEL) begin
        // Decelerate
        if (current_velocity > seg_exit_vel) begin
            current_velocity <= current_velocity - (seg_accel >> 8);
        end
    end
end

// ============================================================================
// Step pulse generation (DDA algorithm)
// ============================================================================

// Find maximum delta for Bresenham master axis
reg [STEP_WIDTH:0] max_delta;
reg [2:0] master_axis;

always @(*) begin
    max_delta = 0;
    master_axis = 0;
    for (axis = 0; axis < NUM_AXES; axis = axis + 1) begin
        if (delta[axis] > max_delta) begin
            max_delta = delta[axis];
            master_axis = axis[2:0];
        end
    end
end

// Step generation
reg velocity_tick;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        velocity_accum <= 0;
        velocity_tick <= 0;
    end else begin
        velocity_tick <= 0;
        
        if (state == STATE_ACCEL || state == STATE_CRUISE || state == STATE_DECEL) begin
            velocity_accum <= velocity_accum + current_velocity;
            
            // Generate step tick at velocity rate
            if (velocity_accum[VELOCITY_WIDTH+8]) begin
                velocity_accum[VELOCITY_WIDTH+8] <= 0;
                velocity_tick <= 1;
            end
        end else begin
            velocity_accum <= 0;
        end
    end
end

// Generate steps on each axis
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        for (axis = 0; axis < NUM_AXES; axis = axis + 1) begin
            step_out[axis] <= 0;
            step_pulse_counter[axis] <= 0;
            current_pos[axis] <= 0;
            error[axis] <= 0;
        end
    end else begin
        // Step pulse timing
        for (axis = 0; axis < NUM_AXES; axis = axis + 1) begin
            if (step_pulse_counter[axis] > 0) begin
                step_pulse_counter[axis] <= step_pulse_counter[axis] - 1;
                if (step_pulse_counter[axis] == 1) begin
                    step_out[axis] <= 0;
                end
            end
        end
        
        // DDA step generation
        if (velocity_tick && busy) begin
            for (axis = 0; axis < NUM_AXES; axis = axis + 1) begin
                if (steps_remaining[axis] > 0 && axis_enable[axis]) begin
                    error[axis] <= error[axis] + delta[axis];
                    
                    if (error[axis] >= max_delta) begin
                        error[axis] <= error[axis] - max_delta;
                        
                        // Generate step pulse
                        step_out[axis] <= 1;
                        step_pulse_counter[axis] <= STEP_PULSE_WIDTH;
                        
                        // Update position
                        if (dir_out[axis]) begin
                            current_pos[axis] <= current_pos[axis] + 1;
                        end else begin
                            current_pos[axis] <= current_pos[axis] - 1;
                        end
                        
                        steps_remaining[axis] <= steps_remaining[axis] - 1;
                    end
                end
            end
        end
    end
end

// ============================================================================
// Limit switch detection (directly connected)
// ============================================================================

assign limit_hit = {NUM_AXES{1'b0}};  // Connect to actual limit inputs

endmodule
