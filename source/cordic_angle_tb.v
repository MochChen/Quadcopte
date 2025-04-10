`timescale 1ns / 1ps
`include "cordic_angle.v"

module cordic_angle_tb();

    reg clk = 0;
    reg rst_n = 0;

    reg signed [15:0] x;
    reg signed [15:0] y;
    reg signed [15:0] z;

    reg cdra_start;
    wire cdra_done;
    wire signed [15:0] crda_angle;

    // 产生50MHz时钟（20ns周期）
    always #10 clk = ~clk;

    // 实例化被测模块
    cordic_angle uut (
        .clk(clk),
        .rst_n(rst_n),
        .x(x),
        .y(y),
        .z(z),
        .cdra_start(cdra_start),
        .cdra_done(cdra_done),
        .crda_angle(crda_angle)
    );

    initial begin

        // 初始化
        rst_n = 0;
        cdra_start = 0;
        x = 0; y = 0; z = 0;
        #100;

        rst_n = 1;
        #100;

        // 提供一组 ax, ay, az 输入
        // 示例：ax = 0.5g, ay = 0.5g, az = 0.707g （正好是 pitch = roll = 30度）
        // 假设1g对应的数值为 16384（±2g 模式），所以：
        // x = 8192;     // ax ≈ 0.5g
        // y = 8192;     // ay ≈ 0.5g
        // z = 11585;    // az ≈ 0.707g
        x = 10000;        // ax ≈ 0g
        y = 10000;        // ay ≈ 0g
        z = 11585;    // az ≈ 0.707g
        #20;

        cdra_start = 1;
        #20;
        cdra_start = 0;

        // 等待计算完成
        wait (cdra_done == 1);
        #20;

        $display("  [输出] Pitch(acc) angle:%d", crda_angle);

        #100;
        $finish;
    end

endmodule
