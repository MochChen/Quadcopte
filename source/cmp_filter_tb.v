`timescale 1ns/1ps
`include "cmp_filter.v"

module cmp_filter_tb();

    // Parameters
    parameter ALPHA = 99;
    
    // Inputs
    reg clk;
    reg rst_n;
    reg cmp_filter_en;
    reg [23:0] cur_pitch_gyro;
    reg [23:0] cur_roll_gyro;
    reg [23:0] cur_yaw_gyro;
    reg [23:0] cur_pitch_acc;
    reg [23:0] cur_roll_acc;
    
    // Outputs
    wire [23:0] cur_pitch;
    wire [23:0] cur_roll;
    wire [23:0] cur_yaw;
    
    // Instantiate the Unit Under Test (UUT)
    cmp_filter #(
        .ALPHA(ALPHA)
    ) uut (
        .clk(clk),
        .rst_n(rst_n),
        .cmp_filter_en(cmp_filter_en),
        .cur_pitch_gyro(cur_pitch_gyro),
        .cur_roll_gyro(cur_roll_gyro),
        .cur_yaw_gyro(cur_yaw_gyro),
        .cur_pitch_acc(cur_pitch_acc),
        .cur_roll_acc(cur_roll_acc),
        .cur_pitch(cur_pitch),
        .cur_roll(cur_roll),
        .cur_yaw(cur_yaw)
    );
    
    // Clock generation
    initial begin
        clk = 0;
        forever #5 clk = ~clk; // 100MHz clock
    end
    
    // Test procedure
    initial begin
        // Initialize Inputs
        rst_n = 0;
        cmp_filter_en = 0;
        cur_pitch_gyro = 0;
        cur_roll_gyro = 0;
        cur_yaw_gyro = 0;
        cur_pitch_acc = 0;
        cur_roll_acc = 0;
        
        // Reset the system
        #20;
        rst_n = 1;
        #10;
        
        // Test case 1: Basic filtering with ALPHA=99 (99% gyro, 1% acc)
        cmp_filter_en = 1;
        cur_pitch_gyro = 24'd9900;  // Will contribute 99% of 9900 = 9801
        cur_pitch_acc = 24'd10000;  // Will contribute 1% of 10000 = 100
        cur_roll_gyro = 24'd4950;   // Will contribute 99% of 4950 = 4900.5
        cur_roll_acc = 24'd5000;    // Will contribute 1% of 5000 = 50
        cur_yaw_gyro = 24'd1234;     // Should pass through directly
        
        #10;
        
        // Check outputs (note: integer division will truncate)
        if (cur_pitch !== 24'd9901) $display("Error: Pitch filter incorrect. Expected 9901, got %d", cur_pitch);
        if (cur_roll !== 24'd4950) $display("Error: Roll filter incorrect. Expected 4950, got %d", cur_roll);
        if (cur_yaw !== 24'd1234) $display("Error: Yaw filter incorrect. Expected 1234, got %d", cur_yaw);
        
        // Test case 2: Change inputs to verify continuous filtering
        cur_pitch_gyro = 24'd19800;  // 99% of 19800 = 19602
        cur_pitch_acc = 24'd20000;    // 1% of 20000 = 200
        cur_roll_gyro = -24'd9900;    // 99% of -9900 = -9801
        cur_roll_acc = -24'd10000;    // 1% of -10000 = -100
        cur_yaw_gyro = -24'd5678;     // Should pass through directly
        
        #10;
        
        // Check outputs
        if (cur_pitch !== 24'd19802) $display("Error: Pitch filter incorrect. Expected 19802, got %d", cur_pitch);
        if (cur_roll !== -24'd9901) $display("Error: Roll filter incorrect. Expected -9901, got %d", cur_roll);
        if (cur_yaw !== -24'd5678) $display("Error: Yaw filter incorrect. Expected -5678, got %d", cur_yaw);
        
        // Test case 3: Disable filtering
        cmp_filter_en = 0;
        cur_pitch_gyro = 24'd30000;
        cur_roll_gyro = 24'd30000;
        cur_yaw_gyro = 24'd30000;
        cur_pitch_acc = 24'd30000;
        cur_roll_acc = 24'd30000;
        
        #10;
        
        // Outputs should remain unchanged
        if (cur_pitch !== 24'd19802) $display("Error: Pitch changed when disabled. Expected 19802, got %d", cur_pitch);
        if (cur_roll !== -24'd9901) $display("Error: Roll changed when disabled. Expected -9901, got %d", cur_roll);
        if (cur_yaw !== -24'd5678) $display("Error: Yaw changed when disabled. Expected -5678, got %d", cur_yaw);
        
        // Test case 4: Reset
        rst_n = 0;
        #10;
        rst_n = 1;
        
        // All outputs should be zero
        if (cur_pitch !== 24'd0) $display("Error: Pitch not reset");
        if (cur_roll !== 24'd0) $display("Error: Roll not reset");
        if (cur_yaw !== 24'd0) $display("Error: Yaw not reset");
        
        // Test case 5: Edge case with minimum values
        cmp_filter_en = 1;
        cur_pitch_gyro = -24'd10000;
        cur_pitch_acc = -24'd10000;
        cur_roll_gyro = -24'd10000;
        cur_roll_acc = -24'd10000;
        cur_yaw_gyro = -24'd10000;
        
        #10;
        
        // Check outputs
        if (cur_pitch !== -24'd10000) $display("Error: Pitch filter incorrect for negative values");
        if (cur_roll !== -24'd10000) $display("Error: Roll filter incorrect for negative values");
        if (cur_yaw !== -24'd10000) $display("Error: Yaw filter incorrect for negative values");
        
        $display("Testbench completed");
        $finish;
    end
    
    // Monitor
    initial begin
        $monitor("At time %t: pitch=%d, roll=%d, yaw=%d", 
                 $time, cur_pitch, cur_roll, cur_yaw);
    end

endmodule