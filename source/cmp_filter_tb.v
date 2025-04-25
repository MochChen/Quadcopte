`timescale 1ns/1ps
`include "cmp_filter.v"

module cmp_filter_tb();

    // Parameters
    parameter ALPHA = 99;
    
    // Inputs
    reg clk;
    reg rst_n;
    reg cmp_filter_en;
    reg [23:0] cur_pitch_gyro; //最大就90 * 131 = 11790
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
        cur_pitch_gyro = 24'd9900;  // Will contribute 99% of 9900 = 9745.3125
        cur_pitch_acc = 24'd10000;  // Will contribute 1% of 10000 = 100

        cur_roll_gyro = 24'd4950;   // Will contribute 98.4375% of 4950 = 4872.65625
        cur_roll_acc = 24'd5000;    // Will contribute 1.5625% of 5000 = 78.125
        //cur_pitch <= (ALPHA * cur_pitch_gyro + (128 - ALPHA) * cur_pitch_acc) >> 7; // 近似除以128
        cur_yaw_gyro = 24'd1234;     // Should pass through directly
        
        #10;
        cmp_filter_en = 1;
        
        $display("Testbench completed");
        $finish;
    end
    
    // Monitor
    initial begin
        $monitor("At time %t: pitch=%d, roll=%d, yaw=%d", 
                 $time, cur_pitch, cur_roll, cur_yaw);
    end

endmodule