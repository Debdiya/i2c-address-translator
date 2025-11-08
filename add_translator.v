// Code your design here
`timescale 1ns/1ps

module i2c_address_translator (
    input  wire clk,
    input  wire reset,
    input  wire enable,
    inout  wire scl,
    inout  wire sda,

  output wire [6:0] translated_addr,
    output wire [6:0] captured_addr_o,
    output wire       translate_hit,
    output wire [2:0] state_o,
  output wire [2:0] bitcount_o);

reg scl_driver = 1'b0, sda_driver = 1'b0;
assign scl = (scl_driver) ? 1'b0 : 1'bz;
assign sda = (sda_driver) ? 1'b0 : 1'bz;
 (* KEEP = "TRUE" *) reg scl_clk1, scl_clk2, sda_clk1, sda_clk2;

always @(posedge clk) begin
  scl_clk1 <= scl;  scl_clk2 <= scl_clk1;
  sda_clk1 <= sda;  sda_clk2 <= sda_clk1;
end

wire scl_up = (scl_clk2 == 1'b0) && (scl_clk1 == 1'b1);
wire scl_down = (scl_clk2 == 1'b1) && (scl_clk1 == 1'b0);
wire starting = (sda_clk2 == 1'b1) && (sda_clk1 == 1'b0) && (scl_clk1 == 1'b1);
wire stopping  = (sda_clk2 == 1'b0) && (sda_clk1 == 1'b1) && (scl_clk1 == 1'b1);

 localparam idle=3'b000, start=3'b001, address=3'b010, translate=3'b011, transfer=3'b100;
  (* KEEP = "TRUE" *) reg [2:0] current_state, next_state;

  (* KEEP = "TRUE" *) reg [2:0] bitcount;          
  (* KEEP = "TRUE" *) reg [6:0] captured_address;
  (* KEEP = "TRUE" *) reg read_write_bit;
  (* KEEP = "TRUE" *) reg [6:0] forward_address;
  (* KEEP = "TRUE" *) reg trans;

always @(posedge clk or posedge reset) begin
  if (reset) begin
    current_state <= idle;
    bitcount <= 3'd0;
    captured_address <= 7'd0;
    read_write_bit <= 1'b0;
    trans <= 1'b0;
  end else begin
    current_state <= next_state;
  end
end

always @* begin
  next_state = current_state;
  case (current_state)
    idle:     
     if (starting && enable) 
     next_state = start;
 start:     
    next_state =address ;
    address: 
      if (scl_up && (bitcount == 3'd7)) next_state = translate;  
    translate: next_state = transfer;
    transfer:  
     if (stopping) 
     next_state = idle;
    default:  
     next_state = idle;
  endcase
end

always @(posedge clk or posedge reset) begin
  if (reset) begin
    bitcount <= 3'd0;
  end else if (current_state == start) begin
    bitcount <= 3'd0;
  end else if (current_state == address && scl_up) begin
    bitcount <= bitcount + 3'd1;
  end
end

reg got_rw;
always @(posedge clk or posedge reset) begin
  if (reset) begin
    captured_address <= 7'd0;
    read_write_bit <= 1'b0;
    got_rw <= 1'b0;
  end else if (current_state == start) begin
    got_rw <= 1'b0;
  end else if (current_state == address && scl_up) begin
    captured_address[6 - bitcount] <= sda_clk1;
    if (bitcount == 3'd7) begin
      got_rw <= 1'b1; 
    end
  end
end

always @* begin
  forward_address = captured_address;
  if (captured_address == 7'h49) begin
    forward_address = 7'h48;
    trans= 1'b1;
  end else begin
    trans = 1'b0;
  end
end
assign translated_addr  = forward_address;
  assign captured_addr_o  = captured_address;
  assign translate_hit    = trans;
  assign state_o          = current_state;
  assign bitcount_o       = bitcount;

always @(posedge clk) begin
  if (current_state == translate) begin
    if (trans)
      $display("FPGA: Translating virtual address 0x%02h -> physical 0x48", captured_address);
    else
      $display("FPGA: Forwarding normal address 0x%02h", captured_address);
  end
end

endmodule
