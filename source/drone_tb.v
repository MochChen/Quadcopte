`timescale 1ns / 1ps
// `include "drone.v"


module drone_tb;

    reg clk;
    reg rst_n;

    wire scl;
    wire sda;

    reg [31:0]   axi_din_32;
    reg          axi_din_valid;
    wire [63:0]  axi_out_64H;
    wire [63:0]  axi_out_64L;

    wire pwm_1;
    wire pwm_2;
    wire pwm_3;
    wire pwm_4;

    // 模拟IIC设备驱动sda
    reg sda_drive = 1; 
    reg sda_dir = 0;
    assign sda = sda_dir ? sda_drive : 1'bz;


    // 实例化被测模�?
    drone uut_drone (
        .clk(clk),
        .rst_n(rst_n),

        .scl(scl),
        .sda(sda),

        .axi_din_32     (axi_din_32),
        .axi_din_valid  (axi_din_valid),
        .axi_out_64H    (axi_out_64H),
        .axi_out_64L    (axi_out_64L),

        .pwm_1(pwm_1),
        .pwm_2(pwm_2),
        .pwm_3(pwm_3),
        .pwm_4(pwm_4)
    );

    // 产生时钟
    always #10 clk = ~clk;


    // 测试过程
    initial begin
        clk     = 0;
        rst_n   = 0;
        sda_dir = 0;
        
        // 复位
        #50 rst_n = 1;

        #50000
        $finish;
    end
endmodule