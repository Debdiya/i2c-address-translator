//==============================================================================
// Complete I2C Address Translator with Time-Division Multiplexing
// Integrates i2c_master and i2c_slave modules
// Enables multiple devices with same physical address to coexist
//==============================================================================

module i2c_address_translator_complete (
    input wire clk,              // System clock (50 MHz)
    input wire rst_n,            // Active low reset
    
    // Bus Master Side (We are SLAVE)
    inout wire master_sda,
    inout wire master_scl,
    
    // Device Side (We are MASTER)
    inout wire device_sda,
    output wire device_scl,
    
    // TDM Control
    output reg device1_enable,   // Enable Device 1 (connect to power switch)
    output reg device2_enable,   // Enable Device 2 (connect to power switch)
    input wire [15:0] timeslot_duration,  // Timeslot duration in clock cycles
    input wire tdm_enable,                // Enable/disable TDM mode
    
    // Configuration
    input wire config_enable,
    input wire [1:0] config_entry,
    input wire [6:0] config_virtual_addr,
    input wire [6:0] config_physical_addr,
    
    // Status
    output wire [1:0] active_device,      // Which device is currently active
    output wire [3:0] tdm_state,          // TDM state for debugging
    output wire [3:0] slave_state,        // Slave state for debugging
    output wire busy,
    output wire error
);

    //==========================================================================
    // Parameters
    //==========================================================================
    localparam DEFAULT_TIMESLOT = 16'd50000;  // 1ms @ 50MHz
    localparam GUARD_TIME = 16'd1000;         // 20μs guard time
    
    //==========================================================================
    // TDM State Machine
    //==========================================================================
    localparam TDM_IDLE         = 4'd0;
    localparam TDM_DEVICE1_ON   = 4'd1;
    localparam TDM_DEVICE1_WAIT = 4'd2;
    localparam TDM_DEVICE1_OFF  = 4'd3;
    localparam TDM_DEVICE2_ON   = 4'd4;
    localparam TDM_DEVICE2_WAIT = 4'd5;
    localparam TDM_DEVICE2_OFF  = 4'd6;
    
    reg [3:0] tdm_state_reg;
    reg [23:0] timeslot_counter;
    reg [15:0] current_timeslot_duration;
    reg [1:0] active_device_reg;
    
    assign tdm_state = tdm_state_reg;
    assign active_device = active_device_reg;
    
    //==========================================================================
    // Address Translation Table
    //==========================================================================
    reg [6:0] virtual_addr_table [3:0];
    reg [6:0] physical_addr_table [3:0];
    reg [1:0] device_mapping [3:0];      // Which physical device (0 or 1)
    reg [3:0] table_valid;
    
    initial begin
        // Virtual 0x50 → Physical 0x68 → Device 0
        virtual_addr_table[0] = 7'h50;
        physical_addr_table[0] = 7'h68;
        device_mapping[0] = 2'd0;
        table_valid[0] = 1'b1;
        
        // Virtual 0x51 → Physical 0x68 → Device 1 (SAME PHYSICAL ADDRESS!)
        virtual_addr_table[1] = 7'h51;
        physical_addr_table[1] = 7'h68;
        device_mapping[1] = 2'd1;
        table_valid[1] = 1'b1;
        
        // Virtual 0x52 → Physical 0x69 → Device 0
        virtual_addr_table[2] = 7'h52;
        physical_addr_table[2] = 7'h69;
        device_mapping[2] = 2'd0;
        table_valid[2] = 1'b1;
        
        table_valid[3] = 1'b0;
    end
    
    // Dynamic configuration
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            // Keep initial values
        end else if (config_enable) begin
            virtual_addr_table[config_entry] <= config_virtual_addr;
            physical_addr_table[config_entry] <= config_physical_addr;
            table_valid[config_entry] <= 1'b1;
        end
    end
    
    //==========================================================================
    // TDM Time Slot Controller
    //==========================================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            tdm_state_reg <= TDM_IDLE;
            device1_enable <= 0;
            device2_enable <= 0;
            active_device_reg <= 2'd0;
            timeslot_counter <= 0;
            current_timeslot_duration <= DEFAULT_TIMESLOT;
        end else begin
            
            // Use configured duration or default
            current_timeslot_duration <= (timeslot_duration != 0) ? 
                                        timeslot_duration : DEFAULT_TIMESLOT;
            
            if (tdm_enable) begin
                case (tdm_state_reg)
                    
                    TDM_IDLE: begin
                        device1_enable <= 0;
                        device2_enable <= 0;
                        timeslot_counter <= 0;
                        tdm_state_reg <= TDM_DEVICE1_ON;
                    end
                    
                    TDM_DEVICE1_ON: begin
                        device1_enable <= 1;
                        device2_enable <= 0;
                        active_device_reg <= 2'd0;
                        timeslot_counter <= 0;
                        tdm_state_reg <= TDM_DEVICE1_WAIT;
                    end
                    
                    TDM_DEVICE1_WAIT: begin
                        timeslot_counter <= timeslot_counter + 1;
                        if (timeslot_counter >= current_timeslot_duration) begin
                            tdm_state_reg <= TDM_DEVICE1_OFF;
                        end
                    end
                    
                    TDM_DEVICE1_OFF: begin
                        device1_enable <= 0;
                        device2_enable <= 0;
                        
                        if (timeslot_counter >= current_timeslot_duration + GUARD_TIME) begin
                            timeslot_counter <= 0;
                            tdm_state_reg <= TDM_DEVICE2_ON;
                        end else begin
                            timeslot_counter <= timeslot_counter + 1;
                        end
                    end
                    
                    TDM_DEVICE2_ON: begin
                        device1_enable <= 0;
                        device2_enable <= 1;
                        active_device_reg <= 2'd1;
                        timeslot_counter <= 0;
                        tdm_state_reg <= TDM_DEVICE2_WAIT;
                    end
                    
                    TDM_DEVICE2_WAIT: begin
                        timeslot_counter <= timeslot_counter + 1;
                        if (timeslot_counter >= current_timeslot_duration) begin
                            tdm_state_reg <= TDM_DEVICE2_OFF;
                        end
                    end
                    
                    TDM_DEVICE2_OFF: begin
                        device1_enable <= 0;
                        device2_enable <= 0;
                        
                        if (timeslot_counter >= current_timeslot_duration + GUARD_TIME) begin
                            timeslot_counter <= 0;
                            tdm_state_reg <= TDM_DEVICE1_ON;
                        end else begin
                            timeslot_counter <= timeslot_counter + 1;
                        end
                    end
                    
                    default: tdm_state_reg <= TDM_IDLE;
                endcase
            end else begin
                // TDM disabled - enable both devices
                device1_enable <= 1;
                device2_enable <= 1;
                tdm_state_reg <= TDM_IDLE;
            end
        end
    end
    
    //==========================================================================
    // Transaction Control State Machine
    //==========================================================================
    localparam TRANS_IDLE         = 4'd0;
    localparam TRANS_TRANSLATE    = 4'd1;
    localparam TRANS_WAIT_DEVICE  = 4'd2;
    localparam TRANS_FORWARD      = 4'd3;
    localparam TRANS_COMPLETE     = 4'd4;
    
    reg [3:0] trans_state;
    reg [6:0] received_virtual_addr;
    reg [7:0] received_reg_addr;
    reg [7:0] received_data;
    reg received_rw;
    reg [6:0] translated_addr;
    reg [1:0] target_device;
    reg translation_found;
    reg transaction_busy;
    reg transaction_error;
    
    //==========================================================================
    // I2C Slave Interface Signals
    //==========================================================================
    wire [7:0] slave_reg_addr;
    wire [7:0] slave_data_out;
    reg [7:0] slave_data_in;
    wire slave_write_valid;
    wire slave_read_req;
    wire slave_busy;
    wire slave_addr_match;
    wire [3:0] slave_state_out;
    
    assign slave_state = slave_state_out;
    
    // Virtual address for slave (we respond to multiple addresses)
    reg [6:0] current_virtual_addr;
    
    //==========================================================================
    // I2C Slave Module Instantiation
    //==========================================================================
    i2c_slave slave_inst (
        .clk(clk),
        .rst_n(rst_n),
        .sda(master_sda),
        .scl(master_scl),
        .slave_addr(7'h00),  // Special: respond to address 0x00 initially
        .reg_addr_out(slave_reg_addr),
        .data_out(slave_data_out),
        .data_in(slave_data_in),
        .write_valid(slave_write_valid),
        .read_req(slave_read_req),
        .busy(slave_busy),
        .addr_match(slave_addr_match),
        .state_out(slave_state_out)
    );
    
    //==========================================================================
    // I2C Master Interface Signals
    //==========================================================================
    reg master_enable;
    reg master_rw;
    reg [6:0] master_slave_addr;
    reg [7:0] master_reg_addr;
    reg [7:0] master_data_wr;
    wire [7:0] master_data_rd;
    wire master_busy;
    wire master_ack_error;
    
    //==========================================================================
    // I2C Master Module Instantiation
    //==========================================================================
    i2c_master master_inst (
        .clk(clk),
        .rst_n(rst_n),
        .enable(master_enable),
        .rw(master_rw),
        .slave_addr(master_slave_addr),
        .reg_addr(master_reg_addr),
        .data_wr(master_data_wr),
        .data_rd(master_data_rd),
        .busy(master_busy),
        .ack_error(master_ack_error),
        .sda(device_sda),
        .scl(device_scl)
    );
    
    //==========================================================================
    // Address Translation Logic
    //==========================================================================
    
    integer i;
    always @(*) begin
        translation_found = 1'b0;
        translated_addr = received_virtual_addr;
        target_device = 2'd0;
       
        
        for ( i = 0; i < 4; i = i + 1) begin
            if (table_valid[i] && (received_virtual_addr == virtual_addr_table[i])) begin
                translated_addr = physical_addr_table[i];
                target_device = device_mapping[i];
                translation_found = 1'b1;
            end
        end
    end
    
    //==========================================================================
    // Transaction Control Logic
    //==========================================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            trans_state <= TRANS_IDLE;
            master_enable <= 0;
            transaction_busy <= 0;
            transaction_error <= 0;
            received_virtual_addr <= 0;
            received_reg_addr <= 0;
            received_data <= 0;
            received_rw <= 0;
        end else begin
            
            case (trans_state)
                
                TRANS_IDLE: begin
                    master_enable <= 0;
                    transaction_busy <= 0;
                    
                    // Capture transaction from slave
                    if (slave_addr_match) begin
                        received_virtual_addr <= slave_reg_addr[6:0];  // Use reg_addr as virtual addr
                        transaction_busy <= 1;
                        trans_state <= TRANS_TRANSLATE;
                    end
                    
                    if (slave_write_valid) begin
                        received_reg_addr <= slave_reg_addr;
                        received_data <= slave_data_out;
                        received_rw <= 0;
                    end
                    
                    if (slave_read_req) begin
                        received_reg_addr <= slave_reg_addr;
                        received_rw <= 1;
                        slave_data_in <= master_data_rd;
                    end
                end
                
                TRANS_TRANSLATE: begin
                    // Translation done in combinational logic
                    
                    if (tdm_enable) begin
                        // Check if correct device is active
                        if (target_device == active_device_reg) begin
                            trans_state <= TRANS_FORWARD;
                        end else begin
                            trans_state <= TRANS_WAIT_DEVICE;
                        end
                    end else begin
                        trans_state <= TRANS_FORWARD;
                    end
                end
                
                TRANS_WAIT_DEVICE: begin
                    // Wait for correct device timeslot
                    if (target_device == active_device_reg) begin
                        trans_state <= TRANS_FORWARD;
                    end
                end
                
                TRANS_FORWARD: begin
                    if (!master_busy && !master_enable) begin
                        // Start I2C master transaction
                        master_slave_addr <= translated_addr;
                        master_reg_addr <= received_reg_addr;
                        master_rw <= received_rw;
                        master_data_wr <= received_data;
                        master_enable <= 1;
                    end
                    else if (master_enable) begin
                        master_enable <= 0;
                    end
                    else if (!master_busy) begin
                        // Transaction complete
                        if (master_ack_error) begin
                            transaction_error <= 1;
                        end
                        
                        if (received_rw == 1) begin
                            slave_data_in <= master_data_rd;
                        end
                        
                        trans_state <= TRANS_COMPLETE;
                    end
                end
                
                TRANS_COMPLETE: begin
                    transaction_busy <= 0;
                    trans_state <= TRANS_IDLE;
                end
                
                default: trans_state <= TRANS_IDLE;
            endcase
        end
    end
    
    //==========================================================================
    // Status Outputs
    //==========================================================================
    assign busy = transaction_busy || slave_busy || master_busy;
    assign error = transaction_error || master_ack_error;

endmodule
