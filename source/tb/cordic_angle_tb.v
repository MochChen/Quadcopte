`timescale 1ns / 1ps
`include "../cordic_angle.v"
`include "../cordic.v"

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
        x = 0;
        y = 0; 
        z = 0;
        #100;

        rst_n = 1;
        #100;

        // pitch = atan2(ax, sqrt(ay * ay + az * az));
        // roll  = atan2(ay, sqrt(ax * ax + az * az));
        
        // 示例

        x = 16'sd11;
        y = 16'sd258;
        z = 16'sd16068;
        #20;

        cdra_start = 1;
        #20;
        cdra_start = 0;

        // 等待计算完成
        wait (cdra_done == 1);
        #20;

        $display("  [输出] Pitch(acc) angle:%d°", (crda_angle * 90 / 11790));
        $display("  [输出] Pitch(acc) value:        %d", crda_angle);

        #100;
        $finish;
    end

endmodule
