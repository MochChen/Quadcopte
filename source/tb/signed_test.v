`timescale 1ns / 1ps

module signed_arithmetic_tb;

    // 定义测试信号
    reg signed [7:0]  a;
    reg [7:0]  b; 
    reg signed [7:0]  c; 
    
    wire signed [23:0]  data;
    // 实例化加减法模块
    assign data  = -24'sd32768;
    wire signed [15:0] haha = b * a;
    
    // 测试过程
    initial begin
        
        // 测试 8 位有符号数
        a = -8'sd10;  b = 120; c = 8'sd127;
        #10;
        $display("%d", haha);

        // 测试结束
        #10;
        $finish;
    end

endmodule