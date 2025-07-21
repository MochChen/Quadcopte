`timescale 1ns/1ps
`include "cal_error.v"

module cal_error_tb();

    // Inputs
    reg clk;
    reg rst_n;
    reg cal_error_en;
    reg [23:0] tgt_pitch;
    reg [23:0] tgt_roll;
    reg [23:0] tgt_yaw;
    reg [23:0] cur_pitch;
    reg [23:0] cur_roll;
    reg [23:0] cur_yaw;
    
    // Outputs
    wire [23:0] pitch_error;
    wire [23:0] roll_error;
    wire [23:0] yaw_error;
    wire [23:0] i_pitch_error;
    wire [23:0] i_roll_error;
    wire [23:0] i_yaw_error;
    wire [23:0] d_pitch_error;
    wire [23:0] d_roll_error;
    wire [23:0] d_yaw_error;
    
    // Instantiate the Unit Under Test (UUT)
    cal_error uut (
        .clk(clk),
        .rst_n(rst_n),
        .cal_error_en(cal_error_en),
        .tgt_pitch(tgt_pitch),
        .tgt_roll(tgt_roll),
        .tgt_yaw(tgt_yaw),
        .cur_pitch(cur_pitch),
        .cur_roll(cur_roll),
        .cur_yaw(cur_yaw),
        .pitch_error(pitch_error),
        .roll_error(roll_error),
        .yaw_error(yaw_error),
        .i_pitch_error(i_pitch_error),
        .i_roll_error(i_roll_error),
        .i_yaw_error(i_yaw_error),
        .d_pitch_error(d_pitch_error),
        .d_roll_error(d_roll_error),
        .d_yaw_error(d_yaw_error)
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
        cal_error_en = 0;
        tgt_pitch = 0;
        tgt_roll = 0;
        tgt_yaw = 0;
        cur_pitch = 0;
        cur_roll = 0;
        cur_yaw = 0;
        
        // Reset the system
        #20;
        rst_n = 1;
        #10;
        
        // Test case 1: Basic error calculation
        cal_error_en = 1;
        tgt_pitch = 24'd1000;
        tgt_roll = 24'd2000;
        tgt_yaw = 24'd3000;
        cur_pitch = 24'd900;
        cur_roll = 24'd1900;
        cur_yaw = 24'd2900;
        
        #10;
        
        // Check outputs
        if (pitch_error !== 24'd100) $display("Error: Pitch error incorrect. Expected 100, got %d", pitch_error);
        if (roll_error !== 24'd100) $display("Error: Roll error incorrect. Expected 100, got %d", roll_error);
        if (yaw_error !== 24'd100) $display("Error: Yaw error incorrect. Expected 100, got %d", yaw_error);
        if (i_pitch_error !== 24'd100) $display("Error: I_pitch incorrect. Expected 100, got %d", i_pitch_error);
        if (i_roll_error !== 24'd100) $display("Error: I_roll incorrect. Expected 100, got %d", i_roll_error);
        if (i_yaw_error !== 24'd100) $display("Error: I_yaw incorrect. Expected 100, got %d", i_yaw_error);
        if (d_pitch_error !== 24'd100) $display("Error: D_pitch incorrect. Expected 100, got %d", d_pitch_error);
        if (d_roll_error !== 24'd100) $display("Error: D_roll incorrect. Expected 100, got %d", d_roll_error);
        if (d_yaw_error !== 24'd100) $display("Error: D_yaw incorrect. Expected 100, got %d", d_yaw_error);
        
        // Test case 2: Changing current values (error decreases)
        cur_pitch = 24'd950;
        cur_roll = 24'd1950;
        cur_yaw = 24'd2950;
        
        #10;
        
        // Check outputs
        if (pitch_error !== 24'd50) $display("Error: Pitch error incorrect. Expected 50, got %d", pitch_error);
        if (roll_error !== 24'd50) $display("Error: Roll error incorrect. Expected 50, got %d", roll_error);
        if (yaw_error !== 24'd50) $display("Error: Yaw error incorrect. Expected 50, got %d", yaw_error);
        if (i_pitch_error !== 24'd150) $display("Error: I_pitch incorrect. Expected 150, got %d", i_pitch_error);
        if (i_roll_error !== 24'd150) $display("Error: I_roll incorrect. Expected 150, got %d", i_roll_error);
        if (i_yaw_error !== 24'd150) $display("Error: I_yaw incorrect. Expected 150, got %d", i_yaw_error);
        if (d_pitch_error !== -24'd50) $display("Error: D_pitch incorrect. Expected -50, got %d", d_pitch_error);
        if (d_roll_error !== -24'd50) $display("Error: D_roll incorrect. Expected -50, got %d", d_roll_error);
        if (d_yaw_error !== -24'd50) $display("Error: D_yaw incorrect. Expected -50, got %d", d_yaw_error);
        
        // Test case 3: Negative errors
        cur_pitch = 24'd1100;
        cur_roll = 24'd2100;
        cur_yaw = 24'd3100;
        
        #10;
        
        // Check outputs
        if (pitch_error !== -24'd100) $display("Error: Pitch error incorrect. Expected -100, got %d", pitch_error);
        if (roll_error !== -24'd100) $display("Error: Roll error incorrect. Expected -100, got %d", roll_error);
        if (yaw_error !== -24'd100) $display("Error: Yaw error incorrect. Expected -100, got %d", yaw_error);
        if (i_pitch_error !== 24'd50) $display("Error: I_pitch incorrect. Expected 50, got %d", i_pitch_error);
        if (i_roll_error !== 24'd50) $display("Error: I_roll incorrect. Expected 50, got %d", i_roll_error);
        if (i_yaw_error !== 24'd50) $display("Error: I_yaw incorrect. Expected 50, got %d", i_yaw_error);
        if (d_pitch_error !== -24'd150) $display("Error: D_pitch incorrect. Expected -150, got %d", d_pitch_error);
        if (d_roll_error !== -24'd150) $display("Error: D_roll incorrect. Expected -150, got %d", d_roll_error);
        if (d_yaw_error !== -24'd150) $display("Error: D_yaw incorrect. Expected -150, got %d", d_yaw_error);
        
        // Test case 4: Disable error calculation
        cal_error_en = 0;
        tgt_pitch = 24'd5000;
        tgt_roll = 24'd6000;
        tgt_yaw = 24'd7000;
        cur_pitch = 24'd4000;
        cur_roll = 24'd5000;
        cur_yaw = 24'd6000;
        
        #10;
        
        // Outputs should remain unchanged
        if (pitch_error !== -24'd100) $display("Error: Pitch error changed when disabled");
        if (roll_error !== -24'd100) $display("Error: Roll error changed when disabled");
        if (yaw_error !== -24'd100) $display("Error: Yaw error changed when disabled");
        if (i_pitch_error !== 24'd50) $display("Error: I_pitch changed when disabled");
        if (i_roll_error !== 24'd50) $display("Error: I_roll changed when disabled");
        if (i_yaw_error !== 24'd50) $display("Error: I_yaw changed when disabled");
        
        // Test case 5: Reset
        rst_n = 0;
        #10;
        rst_n = 1;
        cal_error_en = 1;
        
        // All outputs should be zero
        if (pitch_error !== 24'd0) $display("Error: Pitch error not reset");
        if (roll_error !== 24'd0) $display("Error: Roll error not reset");
        if (yaw_error !== 24'd0) $display("Error: Yaw error not reset");
        if (i_pitch_error !== 24'd0) $display("Error: I_pitch not reset");
        if (i_roll_error !== 24'd0) $display("Error: I_roll not reset");
        if (i_yaw_error !== 24'd0) $display("Error: I_yaw not reset");
        
        // Test case 6: Large values and integration
        tgt_pitch = 24'h7FFFFF;  // Max positive 24-bit
        tgt_roll = 24'h7FFFFF;
        tgt_yaw = 24'h7FFFFF;
        cur_pitch = 24'h000000;
        cur_roll = 24'h000000;
        cur_yaw = 24'h000000;
        
        #10;
        
        // Check outputs
        if (pitch_error !== 24'h7FFFFF) $display("Error: Pitch error incorrect with large values");
        if (roll_error !== 24'h7FFFFF) $display("Error: Roll error incorrect with large values");
        if (yaw_error !== 24'h7FFFFF) $display("Error: Yaw error incorrect with large values");
        
        $display("Testbench completed");
        $finish;
    end
    
    // Monitor
    initial begin
        $monitor("At time %t: P_err=%d (I=%d, D=%d), R_err=%d (I=%d, D=%d), Y_err=%d (I=%d, D=%d)", 
                 $time,
                 pitch_error, i_pitch_error, d_pitch_error,
                 roll_error, i_roll_error, d_roll_error,
                 yaw_error, i_yaw_error, d_yaw_error);
    end

endmodule