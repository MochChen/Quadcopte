`timescale 1ns/1ps
`include "cordic.v"

module cordic_tb;
    reg clk;
    reg rst_n;
    reg signed [15:0] x_in;
    reg signed [15:0] y_in;
    reg start;
    wire done;
    wire signed [16:0] angle_out;
    wire signed [15:0] magnitude_out;
    
    // 实例化 DUT (Device Under Test)
    cordic #(
        .ITERATIONS(12)
    ) dut (
        .clk(clk),
        .rst_n(rst_n),
        .x_in(x_in),
        .y_in(y_in),
        .start(start),
        .done(done),
        .angle_out(angle_out),
        .magnitude_out(magnitude_out)
    );
    
    // 生成时钟信号
    always #5 clk = ~clk;
    
    initial begin
        // 初始化信号
        clk = 0;
        rst_n = 0;
        x_in = 0;
        y_in = 0;
        start = 0;
        
        // 复位
        #10 rst_n = 1;
        
        // Test 1: 
        #10 
        x_in = 16'sd10000;
        y_in = 16'sd10000; 
        start = 1; 
        #10 
        start = 0;
        wait(done);
        $display("  Test(  45°): angle_out = %d, magnitude_out = %d", angle_out, magnitude_out);
        
        // Test 2: 
        #20 
        x_in = -16'sd10000; 
        y_in = 16'sd10000; 
        start = 1; 
        #10
        start = 0;
        wait(done);
        $display("  Test( 135°): angle_out = %d, magnitude_out = %d", angle_out, magnitude_out);
        
        // Test 3: 
        #20 
        x_in = 16'sd10000; 
        y_in = -16'sd10000; 
        start = 1; 
        #10 
        start = 0;
        wait(done);
        $display("  Test( -45°): angle_out = %d, magnitude_out = %d", angle_out, magnitude_out);
        
        // Test 4: 
        #20 
        x_in = -16'sd10000; 
        y_in = -16'sd10000; 
        start = 1;
        #10 
        start = 0;
        wait(done);
        $display("  Test(-135°): angle_out = %d, magnitude_out = %d", angle_out, magnitude_out);
        
        #50 $finish;
    end
endmodule
