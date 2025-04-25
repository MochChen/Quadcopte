`timescale 1ns / 1ps

module drone_top_tb;

    reg clk;
    reg rst_n;
    reg RxD;
    reg signal_INT;
    wire scl;
    wire sda;           // 改为 wire，由 I?C 主设备驱动
    wire pwm_1, pwm_2, pwm_3, pwm_4;

    // 模拟 I?C 设备驱动 sda（比如 MPU6050）
    reg sda_drive = 1;  // 默认高电平（释放）
    reg sda_dir = 0;    // 0: 释放（高阻态），1: 驱动
    assign sda = sda_dir ? sda_drive : 1'bz;  // 三态控制
    //assign sda = sda_dir ? sda_drive : 1'b1;  // 三态控制,仿真时候设置全1输入

    // 实例化被测模块
    drone_top uut (
        .clk(clk),
        .rst_n(rst_n),
        .scl(scl),
        .sda(sda),
        .signal_INT(signal_INT),
        .pwm_1(pwm_1),
        .pwm_2(pwm_2),
        .pwm_3(pwm_3),
        .pwm_4(pwm_4),
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
        rst_n = 0;
        RxD = 1;
        sda_dir = 0;  // 初始释放 sda

        // 复位
        #50 rst_n = 1;

        #100 signal_INT = 1;

        // 发送串口命令（起飞 8'h01）
        #1000 send_uart_byte(8'h0A); // START_BYTE
        #1000 send_uart_byte(8'h04); // 前进
        #1000 send_uart_byte(8'h08); // STOP_BYTE

        #100000 signal_INT = 0;
        #500000 signal_INT = 1;
        #100000 signal_INT = 0;

        #500000 signal_INT = 1;
        #100000 signal_INT = 0;

        // 继续仿真一段时间
        #100000;
        
        // 停止仿真
        $finish;
    end
endmodule