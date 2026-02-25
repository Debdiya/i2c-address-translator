`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 02/24/2026 02:39:35 PM
// Design Name: 
// Module Name: i2c_cdc
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////


//==============================================================================
// I2C Master Controller for FPGA
// Designed to interface with MPU6050 MEMS Motion Sensor
// Supports 100kHz (Standard) and 400kHz (Fast Mode) operation
//==============================================================================

//==============================================================================
// I2C Master Controller with Clock Divider
// Complete implementation with 4-phase clock generation
// Supports 100kHz (Standard Mode) and 400kHz (Fast Mode)
//==============================================================================

module i2c_master (
    input wire clk,              // System clock (50 MHz)
    input wire rst_n,            // Active low reset
    input wire enable,           // Start I2C transaction
    input wire rw,               // 1=Read, 0=Write
    input wire [6:0] slave_addr, // 7-bit slave address
    input wire [7:0] reg_addr,   // Register address to access
    input wire [7:0] data_wr,    // Data to write
    output reg [7:0] data_rd,    // Data read from slave
    output reg busy,             // Transaction in progress
    output reg ack_error,        // ACK not received
    inout wire sda,              // I2C data line
    output wire scl              // I2C clock line
);

    //==========================================================================
    // Parameters for Clock Generation
    //==========================================================================
    parameter SYS_CLK_FREQ = 50_000_000;  // 50 MHz system clock
    parameter I2C_CLK_FREQ = 400_000;      // 400 kHz I2C clock (Fast mode)
    
    // Calculate divider for I2C clock generation
    // We need 4 ticks per SCL period (for quarter cycles)
    // DIVIDER = SYS_CLK / (I2C_CLK * 4)
    // Example: 50M / (400k * 4) = 31.25 ≈ 31
    localparam DIVIDER = SYS_CLK_FREQ / (I2C_CLK_FREQ * 4);
    
    //==========================================================================
    // State Machine States
    //==========================================================================
    localparam IDLE        = 4'd0;
    localparam START       = 4'd1;
    localparam ADDR_WRITE  = 4'd2;
    localparam ACK1        = 4'd3;
    localparam REG_ADDR    = 4'd4;
    localparam ACK2        = 4'd5;
    localparam RESTART     = 4'd6;
    localparam ADDR_READ   = 4'd7;
    localparam ACK3        = 4'd8;
    localparam READ_DATA   = 4'd9;
    localparam MASTER_ACK  = 4'd10;
    localparam STOP        = 4'd11;
    localparam WRITE_DATA  = 4'd12;
    
    //==========================================================================
    // Clock Divider and Phase Generator
    //==========================================================================
    reg [15:0] clk_divider;     // Counter for clock division
    reg [1:0] scl_phase;        // 4 phases: 0=low, 1=low, 2=high, 3=high
    reg scl_enable;             // Enable clock generation
    reg scl_reg;                // SCL output register
    
    // Clock divider logic - generates 4 phases per I2C clock cycle
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            clk_divider <= 0;
            scl_phase <= 0;
            scl_reg <= 1;
        end else begin
            if (scl_enable) begin
                // Count up to DIVIDER-1, then reset and advance phase
                if (clk_divider == DIVIDER - 1) begin
                    clk_divider <= 0;
                    scl_phase <= scl_phase + 1;
                    
                    // Generate SCL based on phase
                    case (scl_phase)
                        2'd0: scl_reg <= 0;  // Phase 0: SCL low (change SDA)
                        2'd1: scl_reg <= 0;  // Phase 1: SCL low (prepare for rise)
                        2'd2: scl_reg <= 1;  // Phase 2: SCL high (sample SDA)
                        2'd3: scl_reg <= 1;  // Phase 3: SCL high (prepare for fall)
                    endcase
                end else begin
                    clk_divider <= clk_divider + 1;
                end
            end else begin
                // Clock disabled - idle high
                clk_divider <= 0;
                scl_phase <= 0;
                scl_reg <= 1;
            end
        end
    end
    
    assign scl = scl_reg;
    
    //==========================================================================
    // SDA Bidirectional Control
    //==========================================================================
    reg sda_out;
    reg sda_out_en;  // 0=High-Z (release), 1=Drive low
    
    // Tri-state buffer: drive low when enabled, otherwise high-Z
    assign sda = sda_out_en ? 1'b0 : 1'bz;
    
    //==========================================================================
    // Internal Registers
    //==========================================================================
    reg [3:0] state, next_state;
    reg [3:0] bit_count;
    reg [7:0] shift_reg;
    reg received_ack;
    
    //==========================================================================
    // State Machine - Sequential Logic
    //==========================================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            state <= IDLE;
        else
            state <= next_state;
    end
    
    //==========================================================================
    // State Machine - Next State Logic
    //==========================================================================
    always @(*) begin
        next_state = state;
        
        case (state)
            IDLE: begin
                if (enable)
                    next_state = START;
            end
            
            START: begin
                if (scl_phase == 2'd3)
                    next_state = ADDR_WRITE;
            end
            
            ADDR_WRITE: begin
                if (bit_count == 8 && scl_phase == 2'd3)
                    next_state = ACK1;
            end
            
            ACK1: begin
                if (scl_phase == 2'd3) begin
                    if (received_ack)
                        next_state = REG_ADDR;
                    else
                        next_state = STOP;
                end
            end
            
            REG_ADDR: begin
                if (bit_count == 8 && scl_phase == 2'd3)
                    next_state = ACK2;
            end
            
            ACK2: begin
                if (scl_phase == 2'd3) begin
                    if (received_ack) begin
                        if (rw)
                            next_state = RESTART;
                        else
                            next_state = WRITE_DATA;
                    end else
                        next_state = STOP;
                end
            end
            
            WRITE_DATA: begin
                if (bit_count == 8 && scl_phase == 2'd3)
                    next_state = MASTER_ACK;
            end
            
            RESTART: begin
                if (scl_phase == 2'd3)
                    next_state = ADDR_READ;
            end
            
            ADDR_READ: begin
                if (bit_count == 8 && scl_phase == 2'd3)
                    next_state = ACK3;
            end
            
            ACK3: begin
                if (scl_phase == 2'd3) begin
                    if (received_ack)
                        next_state = READ_DATA;
                    else
                        next_state = STOP;
                end
            end
            
            READ_DATA: begin
                if (bit_count == 8 && scl_phase == 2'd3)
                    next_state = MASTER_ACK;
            end
            
            MASTER_ACK: begin
                if (scl_phase == 2'd3)
                    next_state = STOP;
            end
            
            STOP: begin
                if (scl_phase == 2'd3)
                    next_state = IDLE;
            end
            
            default: next_state = IDLE;
        endcase
    end
    
    //==========================================================================
    // State Machine - Output Logic and Data Path
    //==========================================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            busy <= 0;
            scl_enable <= 0;
            sda_out_en <= 0;
            bit_count <= 0;
            shift_reg <= 0;
            data_rd <= 0;
            ack_error <= 0;
            received_ack <= 0;
        end else begin
            case (state)
                
                IDLE: begin
                    busy <= 0;
                    scl_enable <= 0;
                    sda_out_en <= 0;
                    bit_count <= 0;
                    ack_error <= 0;
                    
                    if (enable) begin
                        busy <= 1;
                    end
                end
                
                START: begin
                    // START: SDA falls while SCL is high
                    scl_enable <= 1;
                    
                    if (scl_phase == 2'd0) begin
                        sda_out_en <= 0;  // SDA high initially
                    end else if (scl_phase == 2'd2) begin
                        sda_out_en <= 1;  // SDA low while SCL high = START
                    end
                end
                
                ADDR_WRITE: begin
                    // Send slave address + write bit (0)
                    if (scl_phase == 2'd0) begin  // Change SDA on SCL low
                        if (bit_count == 0) begin
                            shift_reg <= {slave_addr, 1'b0};  // Address + Write
                            bit_count <= 1;
                        end else begin
                            bit_count <= bit_count + 1;
                        end
                        
                        // Drive SDA based on current bit
                        sda_out_en <= ~shift_reg[7];
                        shift_reg <= {shift_reg[6:0], 1'b0};
                    end
                end
                
                ACK1, ACK2, ACK3: begin
                    // Release SDA to read ACK from slave
                    if (scl_phase == 2'd0) begin
                        sda_out_en <= 0;  // Release SDA
                    end else if (scl_phase == 2'd2) begin
                        // Sample ACK on SCL high
                        received_ack <= ~sda;  // ACK is active low
                        if (sda) ack_error <= 1;
                        bit_count <= 0;
                    end
                end
                
                REG_ADDR: begin
                    // Send register address
                    if (scl_phase == 2'd0) begin
                        if (bit_count == 0) begin
                            shift_reg <= reg_addr;
                            bit_count <= 1;
                        end else begin
                            bit_count <= bit_count + 1;
                        end
                        
                        sda_out_en <= ~shift_reg[7];
                        shift_reg <= {shift_reg[6:0], 1'b0};
                    end
                end
                
                WRITE_DATA: begin
                    // Send data byte
                    if (scl_phase == 2'd0) begin
                        if (bit_count == 0) begin
                            shift_reg <= data_wr;
                            bit_count <= 1;
                        end else begin
                            bit_count <= bit_count + 1;
                        end
                        
                        sda_out_en <= ~shift_reg[7];
                        shift_reg <= {shift_reg[6:0], 1'b0};
                    end
                end
                
                RESTART: begin
                    // Repeated START: SDA falls while SCL high
                    if (scl_phase == 2'd0) begin
                        sda_out_en <= 0;  // SDA high
                    end else if (scl_phase == 2'd2) begin
                        sda_out_en <= 1;  // SDA low = START
                    end
                end
                
                ADDR_READ: begin
                    // Send slave address + read bit (1)
                    if (scl_phase == 2'd0) begin
                        if (bit_count == 0) begin
                            shift_reg <= {slave_addr, 1'b1};  // Address + Read
                            bit_count <= 1;
                        end else begin
                            bit_count <= bit_count + 1;
                        end
                        
                        sda_out_en <= ~shift_reg[7];
                        shift_reg <= {shift_reg[6:0], 1'b0};
                    end
                end
                
                READ_DATA: begin
                    // Read data from slave
                    if (scl_phase == 2'd0) begin
                        sda_out_en <= 0;  // Release SDA
                        if (bit_count == 0) begin
                            bit_count <= 1;
                        end else begin
                            bit_count <= bit_count + 1;
                        end
                    end else if (scl_phase == 2'd2) begin
                        // Sample SDA on SCL high
                        shift_reg <= {shift_reg[6:0], sda};
                        if (bit_count == 8) begin
                            data_rd <= {shift_reg[6:0], sda};
                        end
                    end
                end
                
                MASTER_ACK: begin
                    // Send NACK (we're only reading 1 byte)
                    if (scl_phase == 2'd0) begin
                        sda_out_en <= 0;  // NACK = high
                        bit_count <= 0;
                    end
                end
                
                STOP: begin
                    // STOP: SDA rises while SCL is high
                    if (scl_phase == 2'd0) begin
                        sda_out_en <= 1;  // SDA low
                    end else if (scl_phase == 2'd2) begin
                        sda_out_en <= 0;  // SDA high while SCL high = STOP
                        scl_enable <= 0;
                    end
                end
                
            endcase
        end
    end

endmodule

