`timescale 1ns / 1ps
//`include "drone_top.v"
module drone_top_tb;

    reg clk;
    reg rst_n;
    reg RxD;
    wire scl;
    wire sda;
    wire pwm_1_out, pwm_2_out, pwm_3_out, pwm_4_out;

    // 实例化被测模块
    drone_top uut (
        .clk(clk),
        .rst_n(rst_n),
        .scl(scl),
        .sda(sda),
        .pwm_1_out(pwm_1_out),
        .pwm_2_out(pwm_2_out),
        .pwm_3_out(pwm_3_out),
        .pwm_4_out(pwm_4_out),
        .RxD(RxD) 
    );

    // 产生时钟
    always #10 clk = ~clk;

    // 模拟串口输入
    task send_uart_byte(input [7:0] data);
        integer i;
        begin
            RxD = 0; // 起始位
            #8680;  // 115200 baud -> 一个bit时间约8.68us
            for (i = 0; i < 8; i = i + 1) begin
                RxD = data[i];
                #8680;
            end
            RxD = 1; // 停止位
            #8680;
        end
    endtask

    // 测试过程
    initial begin
        clk = 0;
        rst_n = 1;
        RxD = 1;

        // 复位
        #50 rst_n = 0;
        #50 rst_n = 1;

        // 发送串口命令（起飞 8'h01）
        #1000 send_uart_byte(8'h01);

        // 等待PID计算
        #50000;

        // 发送串口命令（前进 8'h03）
        send_uart_byte(8'h03);

        // 继续仿真一段时间
        #100000;
        
        // 停止仿真
        $finish;
    end
endmodule
