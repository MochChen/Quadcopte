`timescale 1ns/1ps
`include "cal_gyro.v"

module cal_gyro_tb();

    // Parameters
    parameter dt = 1;
    
    // Inputs
    reg clk;
    reg rst_n;
    reg cal_gyro_en;
    reg [7:0] mpu_data_packed_8;
    reg [7:0] mpu_data_packed_9;
    reg [7:0] mpu_data_packed_10;
    reg [7:0] mpu_data_packed_11;
    reg [7:0] mpu_data_packed_12;
    reg [7:0] mpu_data_packed_13;
    
    // Outputs
    wire [23:0] cur_pitch_gyro;
    wire [23:0] cur_roll_gyro;
    wire [23:0] cur_yaw_gyro;
    
    // Instantiate the Unit Under Test (UUT)
    cal_gyro #(
        .dt(dt)
    ) uut (
        .clk(clk),
        .rst_n(rst_n),
        .cal_gyro_en(cal_gyro_en),
        .mpu_data_packed_8(mpu_data_packed_8),
        .mpu_data_packed_9(mpu_data_packed_9),
        .mpu_data_packed_10(mpu_data_packed_10),
        .mpu_data_packed_11(mpu_data_packed_11),
        .mpu_data_packed_12(mpu_data_packed_12),
        .mpu_data_packed_13(mpu_data_packed_13),
        .cur_pitch_gyro(cur_pitch_gyro),
        .cur_roll_gyro(cur_roll_gyro),
        .cur_yaw_gyro(cur_yaw_gyro)
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
        cal_gyro_en = 0;
        mpu_data_packed_8 = 0;
        mpu_data_packed_9 = 0;
        mpu_data_packed_10 = 0;
        mpu_data_packed_11 = 0;
        mpu_data_packed_12 = 0;
        mpu_data_packed_13 = 0;
        
        // Reset the system
        #20;
        rst_n = 1;
        #10;
        
        // Test case 1: Simple integration
        cal_gyro_en = 1;
        mpu_data_packed_8 = 8'h00;  // Pitch LSB
        mpu_data_packed_9 = 8'h01;   // Pitch MSB (value = 1)
        mpu_data_packed_10 = 8'hFF;  // Roll LSB
        mpu_data_packed_11 = 8'hFF;  // Roll MSB (value = -1)
        mpu_data_packed_12 = 8'h00;  // Yaw LSB
        mpu_data_packed_13 = 8'h02;  // Yaw MSB (value = 2)
        
        // Let it integrate for 10 clock cycles
        #100;
        
        // Check outputs
        if (cur_pitch_gyro !== 24'd10) $display("Error: Pitch gyro incorrect");
        if (cur_roll_gyro !== -24'd10) $display("Error: Roll gyro incorrect");
        if (cur_yaw_gyro !== 24'd20) $display("Error: Yaw gyro incorrect");
        
        // Test case 2: Change input values
        mpu_data_packed_8 = 8'h00;   // Pitch LSB
        mpu_data_packed_9 = 8'h02;   // Pitch MSB (value = 2)
        mpu_data_packed_10 = 8'hFE;  // Roll LSB
        mpu_data_packed_11 = 8'hFF;  // Roll MSB (value = -2)
        mpu_data_packed_12 = 8'h00;  // Yaw LSB
        mpu_data_packed_13 = 8'h03;  // Yaw MSB (value = 3)
        
        // Let it integrate for 5 more clock cycles
        #50;
        
        // Check outputs
        if (cur_pitch_gyro !== 24'd20) $display("Error: Pitch gyro incorrect");
        if (cur_roll_gyro !== -24'd20) $display("Error: Roll gyro incorrect");
        if (cur_yaw_gyro !== 24'd35) $display("Error: Yaw gyro incorrect");
        
        // Test case 3: Disable integration
        cal_gyro_en = 0;
        #50;
        
        // Values should not change
        if (cur_pitch_gyro !== 24'd20) $display("Error: Pitch gyro changed when disabled");
        if (cur_roll_gyro !== -24'd20) $display("Error: Roll gyro changed when disabled");
        if (cur_yaw_gyro !== 24'd35) $display("Error: Yaw gyro changed when disabled");
        
        // Test case 4: Reset
        rst_n = 0;
        #10;
        rst_n = 1;
        
        // All outputs should be zero
        if (cur_pitch_gyro !== 24'd0) $display("Error: Pitch gyro not reset");
        if (cur_roll_gyro !== 24'd0) $display("Error: Roll gyro not reset");
        if (cur_yaw_gyro !== 24'd0) $display("Error: Yaw gyro not reset");
        
        $display("Testbench completed");
        $finish;
    end
    
    // Monitor
    initial begin
        $monitor("At time %t: pitch=%d, roll=%d, yaw=%d", 
                 $time, cur_pitch_gyro, cur_roll_gyro, cur_yaw_gyro);
    end

endmodule