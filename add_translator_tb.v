`timescale 1ns/1ps

module tb_i2c_address_translator;
reg clk = 0;
  reg reset = 1;
  reg enable = 1;
 wire scl, sda;
  pullup (scl);
  pullup (sda);
reg mst_scl_drive = 0;  
  reg mst_sda_drive = 0;  

  assign scl = (mst_scl_drive) ? 1'b0 : 1'bz;
  assign sda = (mst_sda_drive) ? 1'b0 : 1'bz;

  i2c_address_translator dut (
    .clk   (clk),
    .reset (reset),
    .enable   (enable),
    .scl   (scl),
    .sda   (sda)
  );

  always #5 clk = ~clk;

  task i2c_idle;
    begin
      
      mst_scl_drive = 0;
      mst_sda_drive = 0;
      #(500);
    end
  endtask

  task i2c_start;
    begin
      
      i2c_idle();
      
      mst_sda_drive = 1;  
      #(300);
      
      mst_scl_drive = 1;
      #(300);
    end
  endtask

  task i2c_stop;
    begin
      mst_sda_drive = 1;  
      #(200);
      
      mst_scl_drive = 0;
      #(300);
      
      mst_sda_drive = 0;
      #(400);
    end
  endtask

  task i2c_send_bit(input  bit b);
    begin
      
      if (b) mst_sda_drive = 0; 
      else   mst_sda_drive = 1;  
      #(250);

      mst_scl_drive = 0;  
      #(500);

     
      mst_scl_drive = 1;  
      #(250);
    end
  endtask

  task i2c_ack_slot_release; 
    begin
      mst_sda_drive = 0;  
      i2c_send_bit(1'b1);  
    end
  endtask

  task i2c_send_addr7_rw(input [6:0] addr, input bit rw);
    integer i;
    begin
      i2c_start();

      
      for (i = 6; i >= 0; i = i - 1)
        i2c_send_bit(addr[i]);
i2c_send_bit(rw);
      i2c_ack_slot_release();
i2c_stop();
    end
  endtask

  initial begin
   $dumpfile("i2c_address_translator_tb.vcd");
    $dumpvars(0, tb_i2c_address_translator);
reset = 1;
    #(100);
    reset = 0;
force dut.scl_driver = 1'b0;
    force dut.sda_driver = 1'b0;
#(1000);
    $display("TB:Sending the address 0x49 (expect translation) at time %0t", $time);
    i2c_send_addr7_rw(7'h49, 1'b0); 
#(3000);
$display("TB:Sending the address 0x48 (no translation) at time %0t", $time);
    i2c_send_addr7_rw(7'h48, 1'b1); 
#(5000);
$display("TB:Sending address 0x1A (no translation) at time %0t", $time);
    i2c_send_addr7_rw(7'h1A, 1'b0);
    #(5000);
 $display("TB:Test is complete at time %0t", $time);
    $finish;
  end
    endmodule


    

