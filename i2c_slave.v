`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 02/24/2026 03:36:01 PM
// Design Name: 
// Module Name: i2c_slave
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
// I2C Slave Controller with Clock Synchronization
// Receives transactions from I2C bus master
// Includes START/STOP detection and bit sampling
//==============================================================================

module i2c_slave (
    input wire clk,              // System clock (50 MHz)
    input wire rst_n,            // Active low reset
    
    // I2C Interface
    inout wire sda,              // I2C data line
    inout wire scl,              // I2C clock line
    
    // Slave Configuration
    input wire [6:0] slave_addr, // 7-bit slave address to respond to
    
    // Data Interface
    output reg [7:0] reg_addr_out,    // Register address received
    output reg [7:0] data_out,        // Data received (for writes)
    input wire [7:0] data_in,         // Data to send (for reads)
    output reg write_valid,           // Write data valid
    output reg read_req,              // Read request
    
    // Status
    output reg busy,                  // Transaction in progress
    output reg addr_match,            // Our address was called
    output reg [3:0] state_out        // Current state (for debugging)
);

    //==========================================================================
    // State Machine States
    //==========================================================================
    localparam IDLE        = 4'd0;
    localparam ADDR        = 4'd1;
    localparam ACK_ADDR    = 4'd2;
    localparam REG_ADDR    = 4'd3;
    localparam ACK_REG     = 4'd4;
    localparam DATA_WR     = 4'd5;
    localparam ACK_DATA_WR = 4'd6;
    localparam DATA_RD     = 4'd7;
    localparam ACK_DATA_RD = 4'd8;
    
    reg [3:0] state;
    
    //==========================================================================
    // Input Synchronization (Avoid Metastability)
    //==========================================================================
    reg [1:0] scl_sync;
    reg [1:0] sda_sync;
    wire scl_in;
    wire sda_in;
    
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            scl_sync <= 2'b11;
            sda_sync <= 2'b11;
        end else begin
            scl_sync <= {scl_sync[0], scl};
            sda_sync <= {sda_sync[0], sda};
        end
    end
    
    assign scl_in = scl_sync[1];
    assign sda_in = sda_sync[1];
    
    //==========================================================================
    // Edge Detection
    //==========================================================================
    reg scl_prev, sda_prev;
    wire scl_posedge, scl_negedge;
    wire sda_posedge, sda_negedge;
    
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            scl_prev <= 1;
            sda_prev <= 1;
        end else begin
            scl_prev <= scl_in;
            sda_prev <= sda_in;
        end
    end
    
    assign scl_posedge = scl_in && !scl_prev;
    assign scl_negedge = !scl_in && scl_prev;
    assign sda_posedge = sda_in && !sda_prev;
    assign sda_negedge = !sda_in && sda_prev;
    
    //==========================================================================
    // START and STOP Detection
    //==========================================================================
    reg start_detected, stop_detected;
    
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            start_detected <= 0;
            stop_detected <= 0;
        end else begin
            // START: SDA falls while SCL high
            if (sda_negedge && scl_in && scl_prev) begin
                start_detected <= 1;
                stop_detected <= 0;
            end
            // STOP: SDA rises while SCL high
            else if (sda_posedge && scl_in && scl_prev) begin
                stop_detected <= 1;
                start_detected <= 0;
            end
            else if (state != IDLE) begin
                start_detected <= 0;
                stop_detected <= 0;
            end
        end
    end
    
    //==========================================================================
    // Internal Registers
    //==========================================================================
    reg [3:0] bit_count;
    reg [7:0] shift_reg;
    reg [7:0] data_to_send;
    reg received_rw;  // 0=write, 1=read
    
    // SDA output control
    reg sda_out_en;   // 1=drive low, 0=release (high-Z)
    
    assign sda = sda_out_en ? 1'b0 : 1'bz;
    
    //==========================================================================
    // Main State Machine
    //==========================================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= IDLE;
            bit_count <= 0;
            sda_out_en <= 0;
            shift_reg <= 0;
            reg_addr_out <= 0;
            data_out <= 0;
            write_valid <= 0;
            read_req <= 0;
            busy <= 0;
            addr_match <= 0;
            received_rw <= 0;
            data_to_send <= 0;
        end else begin
            
            // Default: clear one-shot signals
            write_valid <= 0;
            read_req <= 0;
            
            case (state)
                
                IDLE: begin
                    sda_out_en <= 0;
                    bit_count <= 0;
                    busy <= 0;
                    addr_match <= 0;
                    
                    if (start_detected) begin
                        state <= ADDR;
                        busy <= 1;
                    end
                    
                    if (stop_detected) begin
                        state <= IDLE;
                    end
                end
                
                ADDR: begin
                    // Receive 7-bit address + R/W bit
                    if (scl_posedge) begin
                        // Shift in the new bit from SDA
                        shift_reg <= {shift_reg[6:0], sda_in};
                        bit_count <= bit_count + 1;
                        
                        if (bit_count == 7) begin
                            // Use a temporary concatenation for the check
                            if ({shift_reg[6:0], sda_in} >> 1 == slave_addr) begin
                                addr_match <= 1;
                                received_rw <= sda_in; // The 8th bit is the R/W bit
                                state <= ACK_ADDR;
                            end else begin
                                addr_match <= 0;
                                state <= IDLE;  // Not our address
                            end
                            bit_count <= 0;
                        end
                    end
                end
                
                ACK_ADDR: begin
                    // Send ACK (pull SDA low) on SCL low
                    if (scl_negedge) begin
                        sda_out_en <= 1;  // Pull SDA low = ACK
                        
                        if (received_rw == 0) begin
                            // Write operation
                            state <= REG_ADDR;
                        end else begin
                            // Read operation
                            read_req <= 1;
                            data_to_send <= data_in;
                            state <= DATA_RD;
                        end
                    end else if (scl_posedge) begin
                        sda_out_en <= 0;  // Release SDA after ACK
                    end
                end
                
                REG_ADDR: begin
                    // Receive register address
                    if (scl_posedge) begin
                        shift_reg <= {shift_reg[6:0], sda_in};
                        bit_count <= bit_count + 1;
                        
                        if (bit_count == 7) begin
                            reg_addr_out <= {shift_reg[6:0], sda_in};
                            state <= ACK_REG;
                            bit_count <= 0;
                        end
                    end
                end
                
                ACK_REG: begin
                    // Send ACK
                    if (scl_negedge) begin
                        sda_out_en <= 1;  // ACK
                        
                        if (received_rw == 0) begin
                            state <= DATA_WR;
                        end else begin
                            read_req <= 1;
                            data_to_send <= data_in;
                            state <= DATA_RD;
                        end
                    end else if (scl_posedge) begin
                        sda_out_en <= 0;
                    end
                end
                
                DATA_WR: begin
                    // Receive data byte
                    if (scl_posedge) begin
                        shift_reg <= {shift_reg[6:0], sda_in};
                        bit_count <= bit_count + 1;
                        
                        if (bit_count == 7) begin
                            data_out <= {shift_reg[6:0], sda_in};
                            write_valid <= 1;
                            state <= ACK_DATA_WR;
                            bit_count <= 0;
                        end
                    end
                end
                
                ACK_DATA_WR: begin
                    // Send ACK
                    if (scl_negedge) begin
                        sda_out_en <= 1;  // ACK
                        state <= IDLE;  // Done with write
                    end else if (scl_posedge) begin
                        sda_out_en <= 0;
                    end
                end
                
                DATA_RD: begin
                    // Send data byte to master
                    if (scl_negedge) begin
                        if (bit_count == 0) begin
                            shift_reg <= data_to_send;
                        end
                        
                        // Drive SDA based on current bit
                        sda_out_en <= ~shift_reg[7];
                        shift_reg <= {shift_reg[6:0], 1'b0};
                        bit_count <= bit_count + 1;
                        
                        if (bit_count == 7) begin
                            state <= ACK_DATA_RD;
                            bit_count <= 0;
                        end
                    end
                end
                
                ACK_DATA_RD: begin
                    // Receive ACK/NACK from master
                    if (scl_negedge) begin
                        sda_out_en <= 0;  // Release SDA
                    end else if (scl_posedge) begin
                        if (sda_in == 0) begin
                            // ACK - master wants more data
                            read_req <= 1;
                            data_to_send <= data_in;
                            state <= DATA_RD;
                        end else begin
                            // NACK - master is done
                            state <= IDLE;
                        end
                    end
                end
                
                default: state <= IDLE;
            endcase
            
            // STOP condition always returns to IDLE
            if (stop_detected) begin
                state <= IDLE;
                sda_out_en <= 0;
                busy <= 0;
            end
        end
    end
    
    // Export state for debugging
    always @(posedge clk) begin
        state_out <= state;
    end

endmodule

