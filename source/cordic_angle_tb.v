// 输入 (y, x)	理论角度 (°)	你的 CORDIC 结果 (单位: 16384 = 90°)
// ( 0, 1)	    0°	            0
// ( 1, 1)	    45°	            8192
// ( 1, 0)	    90°	            16384
// ( -1, 0)	    -90°	        -16384
// ( 0, -1)	    180° / -180°	±32768

`timescale 1ns / 1ps
`include "cordic_angle.v"
module cordic_angle_tb;
    // 信号定义
    reg clk;
    reg rst_n;
    reg start;
    reg signed [15:0] x_in, y_in, z_in; 
    wire done;
    wire signed [15:0] pitch, roll;

    // 生成时钟信号
    always #5 clk = ~clk;  // 10ns 时钟周期

    // 被测模块 (DUT)
    cordic_angle #(.ITERATIONS(12)) dut (
        .clk(clk),
        .rst_n(rst_n),
        .x_in(x_in),
        .y_in(y_in),
        .z_in(z_in),
        .start(start),
        .done(done),
        .pitch(pitch),
        .roll(roll)
    );

    // **仿真流程**
    initial begin
        clk = 0;
        rst_n = 0;
        start = 0;
        x_in = 0;
        y_in = 0;
        z_in = 0;

        #20 rst_n = 1;  // 释放复位
        #10;

        // pitch = atan2(ay, sqrt(ax * ax + az * az)); 
        // roll  = atan2(-ax, sqrt(ay * ay + az * az)); 
        
        // **测试 1：水平放置，x = 0, y = 0, z = 1g**
        x_in = 16'd0;  
        y_in = 16'd0;
        z_in = 16'd10000;
        start = 1;
        #10 start = 0;
        
        // 等待计算完成
        wait(done);
        $display("  Test 1: Flat   0°     Pitch: %d, Roll: %d", pitch, roll);
        
        #50;  // 稍作延迟

        // **测试 2：俯仰角 45 度，x = 0, y = 0.707g, z = 0.707g**
        x_in = 16'd10000;
        y_in = 16'd0;  // 0.707 * 2^14
        z_in = 16'd10000;  
        start = 1;
        #10 start = 0;
        
        wait(done);
        $display("  Test 2: Pitch 45°     Pitch: %d, Roll: %d", pitch, roll);

        #50;

        // **测试 3：横滚角 -30 度，x = -0.5g, y = 0, z = 0.866g**
        x_in = 16'd0;  // -0.5 * 2^14
        y_in = -16'd8192;
        z_in = 16'd14189;  // 0.866 * 2^14
        start = 1;
        #10 start = 0;

        wait(done);
        $display("  Test 3: Roll -30°     Pitch: %d, Roll: %d", pitch, roll);

        #50;
        $finish;
    end

endmodule
