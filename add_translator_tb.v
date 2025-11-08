`timescale 1ns/1ps

module tb_i2c_address_translator;

  // Clock/reset/en
  reg clk = 0;
  reg reset = 1;
  reg enable = 1;

  // Open-drain I2C wires with pull-ups
  wire scl, sda;
  pullup (scl);
  pullup (sda);

  // Simple I2C master (open-drain) drivers
  reg mst_scl_drive = 0;  // 1 -> drive 0, 0 -> release (Z)
  reg mst_sda_drive = 0;  // 1 -> drive 0, 0 -> release (Z)

  assign scl = (mst_scl_drive) ? 1'b0 : 1'bz;
  assign sda = (mst_sda_drive) ? 1'b0 : 1'bz;

  // DUT
  i2c_address_translator dut (
    .clk   (clk),
    .reset (reset),
    .enable   (enable),
    .scl   (scl),
    .sda   (sda)
  );

  // 100 MHz sysclk
  always #5 clk = ~clk;

  // ---------- I2C helper tasks ----------
  task i2c_idle;
    begin
      // release lines (pull-ups make them '1')
      mst_scl_drive = 0;
      mst_sda_drive = 0;
      #(500);
    end
  endtask

  task i2c_start;
    begin
      // Bus idle: SCL=1, SDA=1
      i2c_idle();
      // SDA: 1 -> 0 while SCL=1
      mst_sda_drive = 1;  // drive low
      #(300);
      // Pull SCL low to begin bit clocking
      mst_scl_drive = 1;
      #(300);
    end
  endtask

  task i2c_stop;
    begin
      // Ensure SDA low before STOP
      mst_sda_drive = 1;  // SDA=0
      #(200);
      // Release SCL high
      mst_scl_drive = 0;
      #(300);
      // Release SDA high while SCL=1 (STOP)
      mst_sda_drive = 0;
      #(400);
    end
  endtask

  task i2c_send_bit(input  bit b);
    begin
      // Data valid while SCL low
      if (b) mst_sda_drive = 0;  // '1' => release
      else   mst_sda_drive = 1;  // '0' => drive low
      #(250);

      // SCL high phase (sampled by DUT when SCL=1)
      mst_scl_drive = 0;  // release -> pull-up => 1
      #(500);

      // SCL low phase
      mst_scl_drive = 1;  // drive low
      #(250);
    end
  endtask

  task i2c_ack_slot_release; // Master releases SDA for ACK
    begin
      mst_sda_drive = 0;   // release SDA
      i2c_send_bit(1'b1);  // clock one bit (kept released)
    end
  endtask

  task i2c_send_addr7_rw(input [6:0] addr, input bit rw);
    integer i;
    begin
      i2c_start();

      // Send 7-bit address MSB->LSB
      for (i = 6; i >= 0; i = i - 1)
        i2c_send_bit(addr[i]);

      // R/W bit
      i2c_send_bit(rw);

      // ACK slot (master releases)
      i2c_ack_slot_release();

      i2c_stop();
    end
  endtask

  // ---------- Stimulus ----------
  initial begin
    // VCD
    $dumpfile("i2c_address_translator_tb.vcd");
    $dumpvars(0, tb_i2c_address_translator);

    // Reset
    reset = 1;
    #(100);
    reset = 0;

    // Ensure DUT releases lines (its internal open-drain regs are not driven in the RTL)
    // Avoid X on the bus by forcing them to 0 (release/Z on inouts).
    force dut.scl_driver = 1'b0;
    force dut.sda_driver = 1'b0;

    // Wait a bit
    #(1000);

    // 1) Send virtual address 0x49 (should translate to 0x48 inside DUT)
    $display("TB: Sending address 0x49 (expect translation) at time %0t", $time);
    i2c_send_addr7_rw(7'h49, 1'b0); // write

    #(3000);

    // 2) Send normal address 0x48 (no translation expected)
    $display("TB: Sending address 0x48 (no translation) at time %0t", $time);
    i2c_send_addr7_rw(7'h48, 1'b1); // read

    #(5000);

    // 3) Another non-mapped address (e.g., 0x1A)
    $display("TB: Sending address 0x1A (no translation) at time %0t", $time);
    i2c_send_addr7_rw(7'h1A, 1'b0);

    #(5000);

    // Done
    $display("TB: Test complete at time %0t", $time);
    $finish;
  end

endmodule
