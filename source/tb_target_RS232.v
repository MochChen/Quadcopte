`timescale 1ns/1ps
// `include "target_RS232.v"
// `include "async_receiver.v"
module tb_target_RS232;

    reg clk;
    reg rst_n;
    reg RxD;
    wire target_renew;
    wire signed [15:0] target_height;
    wire signed [15:0] target_pitch;
    wire signed [15:0] target_roll;
    wire signed [15:0] target_yaw;

    // 实例化被测模块
    target_RS232 uut (
        .clk(clk),
        .rst_n(rst_n),
        .RxD(RxD),
        .target_renew(target_renew),
        .target_height(target_height),
        .target_pitch(target_pitch),
        .target_roll(target_roll),
        .target_yaw(target_yaw)
    );

    // 时钟生成
    initial clk = 0;
    always #10 clk = ~clk;  // 50MHz

    // 任务：发送1字节串口数据（8n1格式）
    task uart_send_byte(input [7:0] data);
        integer i;
        begin
            RxD = 0; // start bit
            #(8680); // 115200bps = 8680ns 每位

            for (i = 0; i < 8; i = i + 1) begin
                RxD = data[i];
                #(8680);
            end

            RxD = 1; // stop bit
            #(8680);
        end
    endtask

    // 发送一组命令（START_BYTE + ACTION + STOP_BYTE）
    task send_packet(input [7:0] action);
        begin
            uart_send_byte(8'h0A);     // START_BYTE
            uart_send_byte(action);   // ACTION
            uart_send_byte(8'h08);     // STOP_BYTE
        end
    endtask

    initial begin
        // 初始状态
        RxD = 1;
        rst_n = 0;
        #100;
        rst_n = 1;

        // 等待系统稳定
        #1000;

        // 发送不同的动作指令测试
        $display("Sending Takeoff (0x01)");
        send_packet(8'h01);
        #100000;

        $display("Sending Forward (0x04)");
        send_packet(8'h04);
        #100000;

        $display("Sending Turn Left (0x08)");
        send_packet(8'h08);
        #100000;

        $display("Sending Land (0x03)");
        send_packet(8'h03);
        #100000;

        $stop;
    end

endmodule
