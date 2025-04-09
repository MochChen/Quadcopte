`timescale 1ns/1ps
`include "mpu_mid.v"

module mpu_mid_tb;

    reg clk = 0;
    reg rst_n = 1;
    reg init_start = 0;
    reg read_start = 0;

    wire en_start;
    wire rd_now;
    wire [2:0] n;
    wire [2:0] m;
    wire [15:0] data_packed;

    // 时钟生成
    always #5 clk = ~clk;  // 100MHz

    // DUT
    mpu_mid uut (
        .clk(clk),
        .rst_n(rst_n),
        .init_start(init_start),
        .read_start(read_start),
        .en_start(en_start),
        .rd_now(rd_now),
        .n(n),
        .m(m),
        .data_packed(data_packed)
    );

    initial begin
        // 初始化
        #40 rst_n = 0;
        #10 rst_n = 1;

        // 测试初始化流程
        #100 init_start = 1;
        #10 init_start = 0;


        // 测试读取流程
        #2000 read_start = 1;
        #10 read_start = 0;

        #10 $finish;
    end

endmodule
