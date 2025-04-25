`timescale 1ns/1ps
`include "cordic.v"

module cordic_tb;
    reg clk;
    reg rst_n;
    reg signed [15:0] x;
    reg signed [15:0] y;
    reg crd_start;
    wire crd_done;
    wire signed [23:0] crd_angle;
    wire signed [23:0] crd_magnitude;
    
    wire signed [23:0] x_24 = {{8{x[15]}}, x};
    wire signed [23:0] y_24 = {{8{y[15]}}, y};

    // 实例化 DUT (Device Under Test)
    cordic dut (
        .clk(clk),
        .rst_n(rst_n),
        .x(x_24),
        .y(y_24),
        .crd_start(crd_start),
        .crd_done(crd_done),
        .crd_angle(crd_angle),
        .crd_magnitude(crd_magnitude)
    );
    
    // 生成时钟信号
    always #5 clk = ~clk;
    
    initial begin
        // 初始化信号
        clk = 0;
        rst_n = 0;
        x = 0;
        y = 0;
        crd_start = 0;
        
        // 复位
        #10 rst_n = 1;
        
        // Test 1: 
        #10 
        x = 16'sd1;
        y = 16'sd10; 
        crd_start = 1; 
        #10 
        crd_start = 0;
        wait(crd_done);
        $display("  Test(  45°): crd_angle = %d, crd_magnitude = %d", crd_angle, crd_magnitude);
        
        // Test 2: 
        #20 
        x = -16'sd10000; 
        y = 16'sd10000; 
        crd_start = 1; 
        #10
        crd_start = 0;
        wait(crd_done);
        $display("  Test( 135°): crd_angle = %d, crd_magnitude = %d", crd_angle, crd_magnitude);
        
        // Test 3: 
        #20 
        x = 16'sd10000; 
        y = -16'sd10000; 
        crd_start = 1; 
        #10 
        crd_start = 0;
        wait(crd_done);
        $display("  Test( -45°): crd_angle = %d, crd_magnitude = %d", crd_angle, crd_magnitude);
        
        // Test 4: 
        #20 
        x = -16'sd10000; 
        y = -16'sd10000; 
        crd_start = 1;
        #10 
        crd_start = 0;
        wait(crd_done);
        $display("  Test(-135°): crd_angle = %d, crd_magnitude = %d", crd_angle, crd_magnitude);
        
        #50 $finish;
    end
endmodule

